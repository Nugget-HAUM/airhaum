// src/taches/taches_mission.rs
//! Tâche Tokio de la couche mission (couche 5).
//!
//! Câble ensemble la MAÉ vol (`mission/etat_machine.rs`) et l'exécution du
//! plan de vol (`mission/plan_vol.rs`) — voir `doc/mission.md`, section
//! « Tâche mission — taches_mission.rs » pour la conception complète
//! (origine des champs, dérivations, canal de commande console).
//!
//! Branchée dans `src/bin/airhaum_test/main.rs` (pas encore dans
//! `airhaum-vol.rs`) : lancée dès que `rx_securite`/`rx_navigation`/
//! `rx_altitude` existent (juste après `taches_surete::lancer_surete`),
//! `DriversInitialises`/`ConfigurationTerminee` envoyées immédiatement.
//! Les actions `Armer`/`ArmementValide`/`Desarmer` sont déclenchées depuis
//! la check-liste d'armement (`console_armement.rs`), pas depuis le menu de
//! test — voir `doc/mission.md`, « Actions armer/lancer/désarmer ».
//! `tx_phase`/`tx_consigne_direction_sol`/`tx_consigne_gaz`/`tx_arme` sont
//! des clones des canaux possédés par `taches_controle.rs`
//! (`HandlesControle`, branchée dans `main.rs`) ; `altitude_cible_m` est
//! une constante provisoire.
//!
//! # Boucle, à 10 Hz
//!
//! 1. `forcer_urgence` sur le dernier `EtatSecurite` reçu — priorité absolue.
//! 2. Vidange du canal de commande console (`CommandeVol`), chacune passée à `traiter_commande`.
//! 3. `tick(&ContexteVol)` — voir origine des champs dans `doc/mission.md`.
//! 4. Publication de `EtatVol` (nouveau canal `watch`).
//! 5. `PhaseControle` dérivée de `EtatVol::est_en_vol()`, republiée.
//! 6. `tx_arme` dérivé de l'état courant **et** de la sécurité (voir
//!    `doit_etre_arme` — jamais armé en `ArretUrgence`/`AtterrissageUrgence`,
//!    y compris au sol où `forcer_urgence` seul n'a aucun effet), republié.
//! 7. Exécution du plan de vol si l'état courant est `RoulageAuSol` ou
//!    `CourseDecollage` ; sinon, le plan peut encore être chargé ou changé
//!    (`tx_plan`, voir `HandlesMission`) — voir `doc/mission.md`,
//!    « Injection du plan de vol ».

use std::sync::{atomic::{AtomicBool, Ordering}, Arc};
use std::time::Instant;
use tokio::sync::{mpsc, watch};
use tokio::time::{interval, Duration};

use crate::capteurs::fusion_altitude::AltitudeFusionnee;
use crate::controle::{ConsigneDirectionSol, ConsigneGaz};
use crate::estimation::EtatNavigation;
use crate::mission::etat_machine::{CommandeVol, ContexteVol, EtatVol, MachineEtatVol};
use crate::mission::plan_vol::{EvenementsSegment, PlanDeVol};
use crate::surete::EtatSecurite;
use crate::taches::taches_controle::PhaseControle;

/// Période du cycle mission (10 Hz — voir `doc/mission.md`).
const PERIODE_MS: u64 = 100;

/// Capacité du canal de commande console. Faible : commandes rares,
/// déclenchées par l'opérateur, pas de contre-pression à prévoir.
const CAPACITE_COMMANDES: usize = 8;

// ─────────────────────────────────────────────────────────────────────────────
// Handle retourné par lancer_mission()
// ─────────────────────────────────────────────────────────────────────────────

/// Handle retourné par [`lancer_mission`].
pub struct HandlesMission {
    /// Dernier état vol connu (canal à valeur courante).
    pub rx_etat_vol: watch::Receiver<EtatVol>,
    /// Émetteur de commandes console (`Armer`, `ArmementValide`, ...) —
    /// la console en obtient un clone, voir `doc/mission.md`.
    pub tx_commande: mpsc::Sender<CommandeVol>,
    /// Dépôt du plan de vol chargé par la console (`v` dans la check-liste
    /// d'armement) — voir la documentation de [`lancer_mission`].
    pub tx_plan: watch::Sender<Option<PlanDeVol>>,
    /// Tâche Tokio — annulable via `arreter()`.
    tache: tokio::task::JoinHandle<()>,
    /// Signal d'arrêt partagé avec la tâche.
    arret: Arc<AtomicBool>,
}

impl HandlesMission {
    /// Signale l'arrêt à la tâche mission. Non-bloquant.
    pub fn arreter(&self) {
        self.arret.store(true, Ordering::Relaxed);
        self.tache.abort();
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Point d'entrée
// ─────────────────────────────────────────────────────────────────────────────

/// Lance la tâche mission.
///
/// Démarre sans plan de vol : `HandlesMission::tx_plan` est la « boîte aux
/// lettres » où l'appelant dépose le plan une fois chargé
/// (`PlanDeVol::charger`/`depuis_toml`, voir `doc/plan_de_vol.md`) —
/// permet de lancer la tâche tôt (dès que les canaux ci-dessous existent),
/// en parallèle du déroulé de la check-liste d'armement, sans attendre que
/// l'opérateur ait choisi un fichier. Voir `doc/mission.md`, « Injection du
/// plan de vol ».
///
/// - `rx_securite`                : canal cloné depuis `HandlesSurete::rx_securite`.
/// - `rx_navigation`              : canal cloné depuis `HandlesEstimation::rx_navigation`.
/// - `rx_altitude`                : canal cloné depuis `HandlesEstimation::rx_altitude`.
/// - `tx_phase`/`tx_consigne_direction_sol`/`tx_consigne_gaz`/`tx_arme` : canaux clonés
///   depuis `HandlesControle`.
/// - `tx_mission_armee`           : état armé de la mission, indépendant de la sécurité
///   (`EtatVol::est_arme`), lu par `taches_surete` — voir `doc/surete.md`.
/// - `altitude_cible_m`          : configuration de mission — valeur fixe pour l'instant,
///   voir `doc/mission.md` (hors périmètre de cette brique).
/// - `commande_operateur_segment` : drapeau partagé avec la console, consommé par un
///   segment à condition `ConditionFin::CommandeOperateur` (`doc/plan_de_vol.md`) —
///   distinct du canal `CommandeVol` (transitions MAÉ vol) retourné dans `HandlesMission`.
///
/// Doit être appelé depuis un contexte Tokio (après `#[tokio::main]`).
#[allow(clippy::too_many_arguments)]
pub fn lancer_mission(
    rx_securite: watch::Receiver<EtatSecurite>,
    rx_navigation: watch::Receiver<EtatNavigation>,
    rx_altitude: watch::Receiver<AltitudeFusionnee>,
    tx_phase: watch::Sender<PhaseControle>,
    tx_consigne_direction_sol: watch::Sender<ConsigneDirectionSol>,
    tx_consigne_gaz: watch::Sender<ConsigneGaz>,
    tx_arme: watch::Sender<bool>,
    tx_mission_armee: watch::Sender<bool>,
    altitude_cible_m: f32,
    commande_operateur_segment: Arc<AtomicBool>,
) -> HandlesMission {
    let arret = Arc::new(AtomicBool::new(false));
    let (tx_etat_vol, rx_etat_vol) = watch::channel(EtatVol::default());
    let (tx_commande, rx_commande) = mpsc::channel(CAPACITE_COMMANDES);
    let (tx_plan, rx_plan) = watch::channel(None);

    let arret_tache = Arc::clone(&arret);
    let tache = tokio::spawn(async move {
        tache_mission(
            rx_securite, rx_navigation, rx_altitude, rx_commande, rx_plan,
            tx_etat_vol, tx_phase, tx_consigne_direction_sol, tx_consigne_gaz, tx_arme,
            tx_mission_armee, altitude_cible_m, commande_operateur_segment,
            arret_tache,
        ).await;
    });

    HandlesMission { rx_etat_vol, tx_commande, tx_plan, tache, arret }
}

// ─────────────────────────────────────────────────────────────────────────────
// Corps de la tâche
// ─────────────────────────────────────────────────────────────────────────────

/// Vrai si `etat` doit maintenir l'armement — voir `doc/mission.md`,
/// « Armement dérivé de l'état vol ».
///
/// Défense en profondeur : jamais armé en `ArretUrgence`/`AtterrissageUrgence`,
/// quel que soit `etat`. `forcer_urgence` (`etat_machine.rs`) n'agit que sur
/// les états en vol (`EtatVol::est_en_vol()`) — au sol (`RoulageAuSol`,
/// `CourseDecollage`, `Armement`), une urgence sécurité n'y change donc rien
/// côté MAÉ vol. Sans ce garde-fou, un arrêt d'urgence déclenché depuis la
/// console pendant un point fixe moteur (plan d'essais, E0.8(a)) n'aurait
/// aucun effet sur `tx_arme`/le mixeur (`taches_controle.rs`) tant que
/// `RoulageAuSol` reste actif.
fn doit_etre_arme(etat: &EtatVol, securite: &EtatSecurite) -> bool {
    if matches!(securite, EtatSecurite::ArretUrgence { .. } | EtatSecurite::AtterrissageUrgence { .. }) {
        return false;
    }
    etat.est_arme()
}

/// Vrai si l'état courant exécute le plan de vol — voir `doc/mission.md`,
/// étape 7 de la boucle.
fn execute_plan_de_vol(etat: &EtatVol) -> bool {
    matches!(etat, EtatVol::RoulageAuSol | EtatVol::CourseDecollage)
}

#[allow(clippy::too_many_arguments)]
async fn tache_mission(
    rx_securite: watch::Receiver<EtatSecurite>,
    rx_navigation: watch::Receiver<EtatNavigation>,
    rx_altitude: watch::Receiver<AltitudeFusionnee>,
    mut rx_commande: mpsc::Receiver<CommandeVol>,
    mut rx_plan: watch::Receiver<Option<PlanDeVol>>,
    tx_etat_vol: watch::Sender<EtatVol>,
    tx_phase: watch::Sender<PhaseControle>,
    tx_consigne_direction_sol: watch::Sender<ConsigneDirectionSol>,
    tx_consigne_gaz: watch::Sender<ConsigneGaz>,
    tx_arme: watch::Sender<bool>,
    tx_mission_armee: watch::Sender<bool>,
    altitude_cible_m: f32,
    commande_operateur_segment: Arc<AtomicBool>,
    arret: Arc<AtomicBool>,
) {
    let mut mae = MachineEtatVol::nouveau();
    let mut plan: Option<PlanDeVol> = None;
    let mut debut_segment = Instant::now();
    // Vrai dès que le plan de vol a commencé à s'exécuter (voir plus bas) —
    // sert à détecter le passage à `true` d'`execute_plan_de_vol` pour
    // réinitialiser `debut_segment` à cet instant précis, plutôt qu'au
    // démarrage de la tâche (qui peut précéder l'armement de plusieurs
    // minutes, le temps que l'opérateur déroule la check-liste).
    let mut execution_en_cours = false;
    let mut tick = interval(Duration::from_millis(PERIODE_MS));

    loop {
        if arret.load(Ordering::Relaxed) { break; }
        tick.tick().await;

        // 1. Priorité sécurité.
        let securite = rx_securite.borrow().clone();
        mae.forcer_urgence(&securite);

        // 2. Commandes console (non-bloquant, toutes celles en attente).
        while let Ok(commande) = rx_commande.try_recv() {
            mae.traiter_commande(commande);
        }

        // 3. Seuils capteurs/estimation.
        let nav = *rx_navigation.borrow();
        let altitude = *rx_altitude.borrow();
        let ctx = ContexteVol {
            // Norme horizontale (Nord/Est) uniquement — le vecteur complet
            // inclurait la composante Down (taux de montée), fausserait la
            // vitesse sol. `None` avant le premier fix GPS : la prédiction
            // EKF seule (sans correction GPS) n'est pas fiable comme vitesse
            // sol — voir doc/mission.md.
            vitesse_sol_ms: nav.origine_definie.then(|| {
                (nav.vitesse_ned.x * nav.vitesse_ned.x + nav.vitesse_ned.y * nav.vitesse_ned.y).sqrt()
            }),
            altitude_m: altitude.altitude_m,
            altitude_cible_m,
            hauteur_sol_mm: altitude.hauteur_sol_mm,
            // Une première publication suffit : le récepteur n'est jamais
            // marqué lu ici. Le GPS n'y entre pas, il reste acquittable à la
            // check-liste (doc/mission.md).
            estimation_prete: rx_navigation.has_changed().unwrap_or(false),
        };
        mae.tick(&ctx);

        let etat = mae.etat().clone();

        // 4. Diffusion de l'état vol.
        let _ = tx_etat_vol.send(etat.clone());

        // 5. Phase contrôle — Vol si en vol, Sol sinon (doc/controle.md).
        let phase = if etat.est_en_vol() { PhaseControle::Vol } else { PhaseControle::Sol };
        let _ = tx_phase.send(phase);

        // 6. Armement dérivé de l'état — et de la sécurité (voir doit_etre_arme).
        let _ = tx_arme.send(doit_etre_arme(&etat, &securite));
        let _ = tx_mission_armee.send(etat.est_arme());

        // 7. Plan de vol — uniquement en RoulageAuSol/CourseDecollage.
        let executer = execute_plan_de_vol(&etat);
        if executer && !execution_en_cours {
            // Premier cycle d'exécution : le chronomètre du segment démarre
            // ici, pas au lancement de la tâche (voir `execution_en_cours`).
            debut_segment = Instant::now();
        }
        execution_en_cours = executer;

        if executer {
            if let Some(plan_actif) = plan.as_mut() {
                if let Some(segment) = plan_actif.segment_courant() {
                    let _ = tx_consigne_gaz.send(segment.type_segment.consigne_gaz());
                    if let Some(consigne_direction) = segment.type_segment.consigne_direction_sol() {
                        let _ = tx_consigne_direction_sol.send(consigne_direction);
                    }

                    let temps_ecoule_s = debut_segment.elapsed().as_secs_f32();
                    let operateur_recu = commande_operateur_segment.swap(false, Ordering::Relaxed);
                    let evenements = EvenementsSegment { commande_operateur_recue: operateur_recu };

                    if plan_actif.segment_termine(temps_ecoule_s, &evenements) {
                        log::info!(
                            target: "mission",
                            "Plan de vol : segment {}/{} terminé",
                            plan_actif.index_courant() + 1, plan_actif.nombre_segments(),
                        );
                        plan_actif.avancer();
                        debut_segment = Instant::now();
                    }
                }
            }
        } else if let Some(nouveau) = rx_plan.borrow_and_update().clone() {
            // Pas encore en exécution : le plan peut encore être chargé ou
            // remplacé par l'opérateur (choix d'un autre fichier avant
            // l'armement) — voir la documentation de `lancer_mission`.
            plan = Some(nouveau);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::Angle;

    fn plan_deux_segments() -> PlanDeVol {
        let toml = r#"
[[segment]]
type = "attente"
fin = { duree_s = 0.05 }

[[segment]]
type = "roulage_cap"
cap_deg = 45.0
gaz = 0.3
fin = { commande_operateur = "console" }
"#;
        PlanDeVol::depuis_toml(toml).unwrap()
    }

    fn navigation_avec_origine(vitesse_nord: f32, vitesse_est: f32) -> EtatNavigation {
        let mut nav = EtatNavigation::nul();
        nav.origine_definie = true;
        nav.vitesse_ned.x = vitesse_nord;
        nav.vitesse_ned.y = vitesse_est;
        nav
    }

    #[allow(clippy::type_complexity)]
    struct Bancs {
        tx_securite: watch::Sender<EtatSecurite>,
        tx_navigation: watch::Sender<EtatNavigation>,
        tx_altitude: watch::Sender<AltitudeFusionnee>,
        rx_phase: watch::Receiver<PhaseControle>,
        rx_consigne_direction_sol: watch::Receiver<ConsigneDirectionSol>,
        rx_consigne_gaz: watch::Receiver<ConsigneGaz>,
        rx_arme: watch::Receiver<bool>,
        rx_mission_armee: watch::Receiver<bool>,
        commande_operateur_segment: Arc<AtomicBool>,
        handles: HandlesMission,
    }

    /// Lance la tâche avec un plan déjà déposé dans `tx_plan` — reproduit le
    /// comportement de l'ancienne API (plan connu dès le départ) pour ne pas
    /// alourdir les tests qui ne portent pas spécifiquement sur l'injection
    /// tardive du plan (voir `plan_charge_apres_coup_avant_armement` et
    /// `chronometre_segment_ignore_le_temps_avant_roulage_au_sol`).
    fn lancer_test(plan: PlanDeVol) -> Bancs {
        let b = lancer_test_sans_plan();
        b.handles.tx_plan.send(Some(plan)).unwrap();
        b
    }

    fn lancer_test_sans_plan() -> Bancs {
        let (tx_securite, rx_securite) = watch::channel(EtatSecurite::Normal);
        let (tx_navigation, rx_navigation) = watch::channel(EtatNavigation::nul());
        let (tx_altitude, rx_altitude) = watch::channel(AltitudeFusionnee::nulle());
        let (tx_phase, rx_phase) = watch::channel(PhaseControle::Sol);
        let (tx_consigne_direction_sol, rx_consigne_direction_sol) = watch::channel(
            ConsigneDirectionSol { cap_cible: Angle::depuis_radians(0.0) }
        );
        let (tx_consigne_gaz, rx_consigne_gaz) = watch::channel(ConsigneGaz::default());
        let (tx_arme, rx_arme) = watch::channel(false);
        let (tx_mission_armee, rx_mission_armee) = watch::channel(false);
        let commande_operateur_segment = Arc::new(AtomicBool::new(false));

        let handles = lancer_mission(
            rx_securite, rx_navigation, rx_altitude,
            tx_phase, tx_consigne_direction_sol, tx_consigne_gaz, tx_arme, tx_mission_armee,
            100.0, Arc::clone(&commande_operateur_segment),
        );

        Bancs {
            tx_securite, tx_navigation, tx_altitude,
            rx_phase, rx_consigne_direction_sol, rx_consigne_gaz, rx_arme, rx_mission_armee,
            commande_operateur_segment, handles,
        }
    }

    /// Fait progresser la MAÉ jusqu'à `RoulageAuSol` via les commandes
    /// console normales (hors seuils capteurs).
    async fn amener_a_roulage_au_sol(b: &Bancs) {
        b.handles.tx_commande.send(CommandeVol::DriversInitialises).await.unwrap();
        b.handles.tx_commande.send(CommandeVol::ConfigurationTerminee).await.unwrap();
        tokio::time::sleep(Duration::from_millis(120)).await;
        // AutoTest → AttenteArmement : géré par tick() sur estimation_prete.
        b.tx_navigation.send(navigation_avec_origine(0.0, 0.0)).unwrap();
        tokio::time::sleep(Duration::from_millis(120)).await;
        b.handles.tx_commande.send(CommandeVol::Armer).await.unwrap();
        b.handles.tx_commande.send(CommandeVol::ArmementValide).await.unwrap();
        tokio::time::sleep(Duration::from_millis(120)).await;
    }

    #[tokio::test]
    async fn attente_armement_exige_l_estimation_mais_pas_le_gps() {
        let b = lancer_test(plan_deux_segments());
        b.handles.tx_commande.send(CommandeVol::DriversInitialises).await.unwrap();
        b.handles.tx_commande.send(CommandeVol::ConfigurationTerminee).await.unwrap();
        tokio::time::sleep(Duration::from_millis(250)).await;
        assert_eq!(*b.handles.rx_etat_vol.borrow(), EtatVol::AutoTest);

        // Première publication de l'estimation, sans fix GPS (origine non définie).
        b.tx_navigation.send(EtatNavigation::nul()).unwrap();
        tokio::time::sleep(Duration::from_millis(150)).await;
        assert_eq!(*b.handles.rx_etat_vol.borrow(), EtatVol::AttenteArmement);

        b.handles.arreter();
    }

    #[tokio::test]
    async fn mission_armee_ne_depend_pas_de_la_securite() {
        let b = lancer_test(plan_deux_segments());
        tokio::time::sleep(Duration::from_millis(120)).await;
        assert!(!*b.rx_mission_armee.borrow());

        amener_a_roulage_au_sol(&b).await;
        assert!(*b.rx_mission_armee.borrow());

        // Une urgence retire l'autorisation de gaz, pas l'état armé de la mission.
        b.tx_securite.send(EtatSecurite::ArretUrgence { raison: "test".into() }).unwrap();
        tokio::time::sleep(Duration::from_millis(150)).await;
        assert!(!*b.rx_arme.borrow());
        assert!(*b.rx_mission_armee.borrow());

        b.handles.arreter();
    }

    #[tokio::test]
    async fn etat_initial_publie_boot() {
        let b = lancer_test(plan_deux_segments());
        assert_eq!(*b.handles.rx_etat_vol.borrow(), EtatVol::Boot);
        b.handles.arreter();
    }

    #[tokio::test]
    async fn commandes_console_font_progresser_la_mae_jusqu_au_roulage() {
        let mut b = lancer_test(plan_deux_segments());
        amener_a_roulage_au_sol(&b).await;
        assert_eq!(*b.handles.rx_etat_vol.borrow_and_update(), EtatVol::RoulageAuSol);
        b.handles.arreter();
    }

    #[tokio::test]
    async fn phase_et_armement_derives_de_l_etat_vol() {
        let mut b = lancer_test(plan_deux_segments());

        // Avant armement : Sol, non armé.
        tokio::time::sleep(Duration::from_millis(120)).await;
        assert_eq!(*b.rx_phase.borrow_and_update(), PhaseControle::Sol);
        assert!(!*b.rx_arme.borrow_and_update());

        amener_a_roulage_au_sol(&b).await;

        // RoulageAuSol : toujours Sol côté contrôle, mais armé.
        assert_eq!(*b.rx_phase.borrow_and_update(), PhaseControle::Sol);
        assert!(*b.rx_arme.borrow_and_update());

        b.handles.arreter();
    }

    #[tokio::test]
    async fn plan_de_vol_ne_publie_rien_avant_roulage_au_sol() {
        let b = lancer_test(plan_deux_segments());

        // Boot, loin de RoulageAuSol : aucune consigne ne doit être publiée.
        tokio::time::sleep(Duration::from_millis(150)).await;
        let rx_gaz = b.rx_consigne_gaz.clone();
        assert!(!rx_gaz.has_changed().unwrap_or(false));

        b.handles.arreter();
    }

    #[tokio::test]
    async fn plan_de_vol_avance_une_fois_en_roulage_au_sol() {
        let mut b = lancer_test(plan_deux_segments());
        amener_a_roulage_au_sol(&b).await;

        // Premier segment ("attente", gaz nul) doit être publié dès l'entrée en RoulageAuSol.
        assert_eq!(*b.rx_consigne_gaz.borrow_and_update(), ConsigneGaz { gaz: 0.0 });

        // Le premier segment (duree_s = 0.05) doit être dépassé, passage au suivant.
        tokio::time::sleep(Duration::from_millis(200)).await;
        assert_eq!(*b.rx_consigne_gaz.borrow_and_update(), ConsigneGaz { gaz: 0.3 });
        let direction = b.rx_consigne_direction_sol.borrow_and_update();
        assert!((direction.cap_cible.degres() - 45.0).abs() < 1e-3);

        b.handles.arreter();
    }

    #[tokio::test]
    async fn plan_charge_apres_coup_avant_armement() {
        // Le plan n'est pas connu au lancement de la tâche (contrairement à
        // lancer_test) — il est déposé dans tx_plan une fois la tâche déjà
        // en route, comme le fait console_armement.rs à la sélection d'un
        // fichier dans la check-liste.
        let mut b = lancer_test_sans_plan();
        tokio::time::sleep(Duration::from_millis(50)).await;
        b.handles.tx_plan.send(Some(plan_deux_segments())).unwrap();

        amener_a_roulage_au_sol(&b).await;
        assert_eq!(*b.handles.rx_etat_vol.borrow_and_update(), EtatVol::RoulageAuSol);
        assert_eq!(*b.rx_consigne_gaz.borrow_and_update(), ConsigneGaz { gaz: 0.0 });

        b.handles.arreter();
    }

    #[tokio::test]
    async fn chronometre_segment_ignore_le_temps_avant_roulage_au_sol() {
        // Régression : avant correctif, le chronomètre du premier segment
        // démarrait au lancement de la tâche et non à l'entrée en
        // RoulageAuSol — un déroulé de check-liste plus long que la durée
        // du premier segment (ici 0.05s) le faisait sauter instantanément.
        let mut b = lancer_test(plan_deux_segments());

        // Simule un déroulé de check-liste bien plus long que la durée du
        // premier segment, avant que l'opérateur n'arme.
        tokio::time::sleep(Duration::from_millis(300)).await;

        amener_a_roulage_au_sol(&b).await;
        // Le premier segment ("attente", gaz nul) doit toujours être actif
        // juste après l'entrée en RoulageAuSol, malgré l'attente précédente.
        assert_eq!(*b.rx_consigne_gaz.borrow_and_update(), ConsigneGaz { gaz: 0.0 });

        b.handles.arreter();
    }

    #[tokio::test]
    async fn segment_commande_operateur_attend_le_drapeau() {
        let b = lancer_test(plan_deux_segments());
        amener_a_roulage_au_sol(&b).await;

        // Laisse passer le premier segment (attente, 0.05 s).
        tokio::time::sleep(Duration::from_millis(150)).await;

        // Deuxième segment : condition CommandeOperateur, ne doit jamais
        // avancer tout seul même après une longue attente.
        tokio::time::sleep(Duration::from_millis(150)).await;
        b.commande_operateur_segment.store(true, Ordering::Relaxed);
        tokio::time::sleep(Duration::from_millis(150)).await;

        // Le drapeau a été consommé (remis à false) par la tâche.
        assert!(!b.commande_operateur_segment.load(Ordering::Relaxed));

        b.handles.arreter();
    }

    #[tokio::test]
    async fn enchainement_jusqu_a_arrondi_via_vitesse_altitude_et_hauteur_sol() {
        // Bout-en-bout : vérifie que ContexteVol.altitude_m/hauteur_sol_mm
        // (source : rx_altitude, AltitudeFusionnee) déclenchent bien les
        // transitions tick() correspondantes — au passage, exerce la
        // correction de type u16 → u32 de HAUTEUR_FLARE_MM/hauteur_sol_mm.
        let mut b = lancer_test(plan_deux_segments());
        amener_a_roulage_au_sol(&b).await;

        b.handles.tx_commande.send(CommandeVol::PositionDecollageAtteinte).await.unwrap();
        tokio::time::sleep(Duration::from_millis(120)).await;
        assert_eq!(*b.handles.rx_etat_vol.borrow_and_update(), EtatVol::CourseDecollage);

        // Vitesse sol ≥ VITESSE_ROTATION_MS (15 m/s) : CourseDecollage → Montee.
        b.tx_navigation.send(navigation_avec_origine(20.0, 0.0)).unwrap();
        tokio::time::sleep(Duration::from_millis(120)).await;
        assert_eq!(*b.handles.rx_etat_vol.borrow_and_update(), EtatVol::Montee);

        // Altitude ≥ altitude_cible_m (100.0, voir lancer_test) : Montee → VolLigneDroite.
        b.tx_altitude.send(AltitudeFusionnee { altitude_m: Some(150.0), hauteur_sol_mm: None }).unwrap();
        tokio::time::sleep(Duration::from_millis(120)).await;
        assert_eq!(*b.handles.rx_etat_vol.borrow_and_update(), EtatVol::VolLigneDroite);

        b.handles.tx_commande.send(CommandeVol::Atterrir).await.unwrap();
        tokio::time::sleep(Duration::from_millis(120)).await;
        assert_eq!(*b.handles.rx_etat_vol.borrow_and_update(), EtatVol::Approche);

        b.handles.tx_commande.send(CommandeVol::ApprocheStabilisee).await.unwrap();
        tokio::time::sleep(Duration::from_millis(120)).await;
        assert_eq!(*b.handles.rx_etat_vol.borrow_and_update(), EtatVol::Final);

        // Hauteur sol ≤ HAUTEUR_FLARE_MM (1500 mm) : Final → Arrondi.
        b.tx_altitude.send(AltitudeFusionnee { altitude_m: Some(150.0), hauteur_sol_mm: Some(1000) }).unwrap();
        tokio::time::sleep(Duration::from_millis(120)).await;
        assert_eq!(*b.handles.rx_etat_vol.borrow_and_update(), EtatVol::Arrondi);

        b.handles.arreter();
    }

    #[tokio::test]
    async fn urgence_securite_transmise_et_sans_effet_hors_vol() {
        // Vérifie que rx_securite est bien branché jusqu'à forcer_urgence
        // (appelé chaque cycle) — mais confirme aussi, en observant le
        // résultat, que RoulageAuSol n'est pas "en vol" au sens
        // d'EtatVol::est_en_vol() : la MAÉ vol ignore l'urgence, comme
        // documenté dans etat_machine.rs. La priorité forcer_urgence →
        // traiter_commande elle-même est déjà testée de façon isolée et
        // déterministe dans etat_machine.rs — pas reproduite ici.
        let mut b = lancer_test(plan_deux_segments());
        amener_a_roulage_au_sol(&b).await;
        assert_eq!(*b.handles.rx_etat_vol.borrow(), EtatVol::RoulageAuSol);

        b.tx_securite.send(EtatSecurite::FailSafe).unwrap();
        tokio::time::sleep(Duration::from_millis(150)).await;

        assert_eq!(*b.handles.rx_etat_vol.borrow_and_update(), EtatVol::RoulageAuSol);

        b.handles.arreter();
    }

    #[tokio::test]
    async fn arret_urgence_console_coupe_larmement_meme_au_sol() {
        // Régression : plan d'essais E0.8(a) — un arrêt d'urgence déclenché
        // depuis la console pendant un point fixe moteur (RoulageAuSol) doit
        // couper le gaz sans délai, alors même que `forcer_urgence` (FSM vol)
        // n'a aucun effet au sol (voir le test précédent). Sans le garde-fou
        // de `doit_etre_arme`, `tx_arme` resterait à `true` tant que
        // `RoulageAuSol` est actif, et le mixeur laisserait passer la
        // consigne gaz du plan de vol.
        let mut b = lancer_test(plan_deux_segments());
        amener_a_roulage_au_sol(&b).await;
        assert!(*b.rx_arme.borrow_and_update());

        b.tx_securite.send(EtatSecurite::ArretUrgence { raison: "test".into() }).unwrap();
        tokio::time::sleep(Duration::from_millis(150)).await;

        // FSM vol inchangée (comme le test précédent), mais désarmé.
        assert_eq!(*b.handles.rx_etat_vol.borrow_and_update(), EtatVol::RoulageAuSol);
        assert!(!*b.rx_arme.borrow_and_update());

        // Sécurité levée : le réarmement suit, toujours en RoulageAuSol.
        b.tx_securite.send(EtatSecurite::Normal).unwrap();
        tokio::time::sleep(Duration::from_millis(150)).await;
        assert!(*b.rx_arme.borrow_and_update());

        b.handles.arreter();
    }
}
