// src/taches/taches_controle.rs
//! Tâche Tokio de la couche contrôle (couche 4).
//!
//! Branche les quatre contrôleurs d'axe (direction sol, roulis, tangage,
//! lacet) et le mixeur. La mission (couche 5) sélectionne la phase active
//! via [`PhaseControle`] et route ses consignes vers le canal du contrôleur
//! pertinent — voir `doc/controle.md`, « Arbitrage direction sol / lacet
//! vol ». `PhaseControle` est volontairement plus pauvre que `EtatVol` : le
//! contrôle ne lit jamais l'état de la MAE vol.
//!
//! Branchée dans `src/bin/airhaum_test/main.rs`, lancée dès que ses
//! dépendances existent (`rx_navigation`/`rx_taux_corps` de l'estimation,
//! `tx_consignes` du thread servo) — voir `doc/mission.md`, « Tâche mission ».
//! Gains des quatre contrôleurs provisoires (non réglés à l'essai) — seul
//! `controleur_direction_sol` est exercé tant que la phase Vol n'est pas
//! atteinte (E3+, `doc/plan_essais.md`).
//!
//! Gaz (`ConsigneGaz`, passthrough — pas de PID, voir `doc/controle.md`) et
//! armement sont pilotés par la mission comme les autres consignes ; par
//! défaut (aucune consigne reçue), gaz au minimum et non-armé — voir
//! « Défense en profondeur » ci-dessous.
//!
//! # Canaux
//!
//! ```text
//! tache_estimation                       tache_controle                      tache_servo
//! ────────────────                       ──────────────                      ───────────
//! EtatNavigation (watch) ───────────────▶ [attitude : lacet/roulis/tangage]
//! Vector3 (rx_taux_corps, watch) ───────▶ [taux corps : roulis/tangage/lacet]
//! (tache_mission, voir taches_mission.rs)
//! PhaseControle (watch) ────────────────▶ sélection du/des contrôleur(s) actif(s)
//! ConsigneDirectionSol (watch) ─────────▶ ControleurDirectionSol ┐
//! ConsigneRoulis (watch) ───────────────▶ ControleurRoulis       ├─ mixeur ──ConsignesServos (watch)──▶
//! ConsigneTangage (watch) ──────────────▶ ControleurTangage      │
//! ConsigneLacet (watch) ────────────────▶ ControleurLacet        ┘
//! ConsigneGaz (watch) ──────────────────▶ [passthrough] ─────────┤
//! bool (rx_arme, watch) ────────────────▶ [passthrough] ─────────┘
//! ```
//!
//! Les six `tx_consigne_*`/`tx_phase`/`tx_arme` sont possédés par cette
//! tâche (créés par `lancer_controle`), sur le même principe que
//! `tx_consignes` déjà possédé par `HandlesServo` — la mission en obtient
//! un clone (`watch::Sender` est `Clone`) pour y écrire ses consignes, voir
//! `airhaum_test/main.rs`.
//!
//! # Défense en profondeur : gaz au minimum si non armé
//!
//! `mixer()` force déjà le gaz au minimum quand `!arme`, quelle que soit
//! `ConsigneGaz` (voir `controle/mixeur.rs`) — l'Arduino applique sa propre
//! garde sur `arme` en aval, mais le mixeur ne doit pas laisser fuiter une
//! consigne gaz non nulle restée dans le canal `watch` après un
//! désarmement (plan d'essais, E0.8 : coupure gaz sans délai).
//!
//! # Gel des contrôleurs inactifs
//!
//! Seul(s) le(s) contrôleur(s) actif(s) selon `PhaseControle` sont mis à
//! jour (intégrés) à chaque cycle ; les autres sont gelés (pas d'appel à
//! `mettre_a_jour`, donc pas d'accumulation d'intégrale). À la transition
//! de phase, les contrôleurs qui viennent de devenir inactifs sont
//! réinitialisés (`reinitialiser()`) pour repartir sans intégrale emballée
//! la prochaine fois qu'ils seront sélectionnés — voir `doc/controle.md`,
//! « Points ouverts ».
//!
//! # Fréquences
//!
//! Réactif sur `rx_navigation.changed()` (pas de minuterie fixe), donc cadencé
//! par l'estimation d'état (~200 Hz, voir `doc/gestion_du_temps.md`). Le `dt`
//! est calculé entre deux `EtatNavigation` reçus via `Horodatage::delta_secondes`,
//! jamais supposé constant.

use std::sync::{atomic::{AtomicBool, Ordering}, Arc};
use tokio::sync::watch;

use crate::controle::{
    mixer, CommandesNormalisees,
    ConsigneDirectionSol, ControleurDirectionSol,
    ConsigneGaz,
    ConsigneLacet, ControleurLacet,
    ConsigneRoulis, ControleurRoulis,
    ConsigneTangage, ControleurTangage,
};
use crate::drivers::controleur_servo::ConsignesServos;
use crate::estimation::EtatNavigation;
use crate::types::{Angle, Horodatage, Vector3};

// ─────────────────────────────────────────────────────────────────────────────
// Phase de vol (point de vue contrôle)
// ─────────────────────────────────────────────────────────────────────────────

/// Sélectionne quel(s) contrôleur(s) d'axe sont actifs. Décidée par la
/// mission (couche 5) à partir de ses propres critères (ex. vitesse > V_rot,
/// déjà utilisé pour sa transition `CourseDecollage → Montee`, voir
/// `doc/mission.md`) — volontairement plus pauvre que `EtatVol` : le
/// contrôle ne lit jamais l'état de la MAE vol (voir `doc/controle.md`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum PhaseControle {
    /// `RoulageAuSol` / `CourseDecollage` : direction sol active.
    #[default]
    Sol,
    /// `Montee` et au-delà : roulis/tangage/lacet actifs.
    Vol,
}

// ─────────────────────────────────────────────────────────────────────────────
// Handle retourné par lancer_controle()
// ─────────────────────────────────────────────────────────────────────────────

/// Handle retourné par [`lancer_controle`].
pub struct HandlesControle {
    /// Sélectionne la phase active (`tache_mission` en obtient un clone).
    pub tx_phase: watch::Sender<PhaseControle>,
    /// Émet la consigne de cap au sol (`tache_mission` en obtient un clone).
    pub tx_consigne_direction_sol: watch::Sender<ConsigneDirectionSol>,
    /// Émet la consigne de roulis en vol (`tache_mission` en obtient un clone, inutilisé avant la phase Vol).
    pub tx_consigne_roulis: watch::Sender<ConsigneRoulis>,
    /// Émet la consigne de tangage en vol (`tache_mission` en obtient un clone, inutilisé avant la phase Vol).
    pub tx_consigne_tangage: watch::Sender<ConsigneTangage>,
    /// Émet la consigne de taux de lacet en vol (`tache_mission` en obtient un clone, inutilisé avant la phase Vol).
    pub tx_consigne_lacet: watch::Sender<ConsigneLacet>,
    /// Émet la consigne de gaz normalisée [0, 1] (`tache_mission` en obtient un clone).
    pub tx_consigne_gaz: watch::Sender<ConsigneGaz>,
    /// Émet l’état armé/désarmé (`tache_mission` en obtient un clone).
    pub tx_arme: watch::Sender<bool>,
    /// Tâche Tokio — annulable via `arreter()`.
    tache: tokio::task::JoinHandle<()>,
    /// Signal d'arrêt partagé avec la tâche.
    arret: Arc<AtomicBool>,
}

impl HandlesControle {
    /// Signale l'arrêt à la tâche de contrôle. Non-bloquant.
    pub fn arreter(&self) {
        self.arret.store(true, Ordering::Relaxed);
        self.tache.abort();
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Point d'entrée
// ─────────────────────────────────────────────────────────────────────────────

/// Lance la tâche de contrôle.
///
/// - `rx_navigation`     : canal watch cloné depuis `HandlesEstimation`.
/// - `rx_taux_corps`     : canal watch cloné depuis `HandlesEstimation` (vitesse
///   angulaire corps, biais retiré).
/// - `tx_consignes_servo`: canal watch cloné depuis `HandlesServo::tx_consignes`.
/// - les quatre contrôleurs sont construits par l'appelant (gains réglés à
///   l'essai, aucune valeur par défaut — voir `doc/controle.md`).
///
/// Doit être appelé depuis un contexte Tokio (après `#[tokio::main]`).
pub fn lancer_controle(
    rx_navigation: watch::Receiver<EtatNavigation>,
    rx_taux_corps: watch::Receiver<Vector3>,
    tx_consignes_servo: watch::Sender<ConsignesServos>,
    controleur_direction_sol: ControleurDirectionSol,
    controleur_roulis: ControleurRoulis,
    controleur_tangage: ControleurTangage,
    controleur_lacet: ControleurLacet,
) -> HandlesControle {
    let arret = Arc::new(AtomicBool::new(false));
    let (tx_phase, rx_phase) = watch::channel(PhaseControle::default());
    let (tx_consigne_direction_sol, rx_consigne_direction_sol) = watch::channel(
        ConsigneDirectionSol { cap_cible: Angle::depuis_radians(0.0) }
    );
    let (tx_consigne_roulis, rx_consigne_roulis) = watch::channel(
        ConsigneRoulis { roulis_cible: Angle::depuis_radians(0.0) }
    );
    let (tx_consigne_tangage, rx_consigne_tangage) = watch::channel(
        ConsigneTangage { tangage_cible: Angle::depuis_radians(0.0) }
    );
    let (tx_consigne_lacet, rx_consigne_lacet) = watch::channel(
        ConsigneLacet { taux_lacet_cible: 0.0 }
    );
    let (tx_consigne_gaz, rx_consigne_gaz) = watch::channel(ConsigneGaz::default());
    let (tx_arme, rx_arme) = watch::channel(false);

    let arret_tache = Arc::clone(&arret);
    let tache = tokio::spawn(async move {
        tache_controle(
            rx_navigation, rx_taux_corps, rx_phase,
            rx_consigne_direction_sol, rx_consigne_roulis, rx_consigne_tangage, rx_consigne_lacet,
            rx_consigne_gaz, rx_arme,
            tx_consignes_servo,
            controleur_direction_sol, controleur_roulis, controleur_tangage, controleur_lacet,
            arret_tache,
        ).await;
    });

    HandlesControle {
        tx_phase,
        tx_consigne_direction_sol, tx_consigne_roulis, tx_consigne_tangage, tx_consigne_lacet,
        tx_consigne_gaz, tx_arme,
        tache, arret,
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Corps de la tâche
// ─────────────────────────────────────────────────────────────────────────────

#[allow(clippy::too_many_arguments)]
async fn tache_controle(
    mut rx_navigation: watch::Receiver<EtatNavigation>,
    rx_taux_corps: watch::Receiver<Vector3>,
    rx_phase: watch::Receiver<PhaseControle>,
    rx_consigne_direction_sol: watch::Receiver<ConsigneDirectionSol>,
    rx_consigne_roulis: watch::Receiver<ConsigneRoulis>,
    rx_consigne_tangage: watch::Receiver<ConsigneTangage>,
    rx_consigne_lacet: watch::Receiver<ConsigneLacet>,
    rx_consigne_gaz: watch::Receiver<ConsigneGaz>,
    rx_arme: watch::Receiver<bool>,
    tx_consignes_servo: watch::Sender<ConsignesServos>,
    mut controleur_direction_sol: ControleurDirectionSol,
    mut controleur_roulis: ControleurRoulis,
    mut controleur_tangage: ControleurTangage,
    mut controleur_lacet: ControleurLacet,
    arret: Arc<AtomicBool>,
) {
    let mut dernier_horodatage: Option<Horodatage> = None;
    let mut derniere_phase: Option<PhaseControle> = None;

    loop {
        if arret.load(Ordering::Relaxed) { break; }

        tokio::select! {
            Ok(_) = rx_navigation.changed() => {
                let nav = *rx_navigation.borrow_and_update();

                let dt = match dernier_horodatage {
                    Some(precedent) => nav.horodatage.delta_secondes(precedent),
                    None => 0.0,
                };
                dernier_horodatage = Some(nav.horodatage);

                let phase = *rx_phase.borrow();

                // Gel des contrôleurs inactifs : à la transition de phase,
                // on réinitialise ceux qui viennent de devenir inactifs
                // pour repartir sans intégrale emballée (doc/controle.md).
                if derniere_phase != Some(phase) {
                    match phase {
                        PhaseControle::Sol => {
                            controleur_roulis.reinitialiser();
                            controleur_tangage.reinitialiser();
                            controleur_lacet.reinitialiser();
                        }
                        PhaseControle::Vol => {
                            controleur_direction_sol.reinitialiser();
                        }
                    }
                    derniere_phase = Some(phase);
                }

                let commandes = match phase {
                    PhaseControle::Sol => {
                        let consigne = *rx_consigne_direction_sol.borrow();
                        let direction = controleur_direction_sol.mettre_a_jour(
                            &consigne, nav.attitude.lacet, dt,
                        );
                        CommandesNormalisees { ailerons: 0.0, profondeur: 0.0, direction }
                    }
                    PhaseControle::Vol => {
                        let taux = *rx_taux_corps.borrow();

                        let consigne_roulis = *rx_consigne_roulis.borrow();
                        let ailerons = controleur_roulis.mettre_a_jour(
                            &consigne_roulis, nav.attitude.roulis, taux.x, dt,
                        );

                        let consigne_tangage = *rx_consigne_tangage.borrow();
                        let profondeur = controleur_tangage.mettre_a_jour(
                            &consigne_tangage, nav.attitude.tangage, taux.y, dt,
                        );

                        let consigne_lacet = *rx_consigne_lacet.borrow();
                        let direction = controleur_lacet.mettre_a_jour(
                            &consigne_lacet, taux.z, dt,
                        );

                        CommandesNormalisees { ailerons, profondeur, direction }
                    }
                };

                let consigne_gaz = *rx_consigne_gaz.borrow();
                let arme = *rx_arme.borrow();
                let consignes_servos = mixer(&commandes, &consigne_gaz, arme);
                let _ = tx_consignes_servo.send(consignes_servos);
            }

            else => break,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::Duration;
    use crate::drivers::controleur_servo::{IMPULSION_MAX_US, IMPULSION_MIN_US, IMPULSION_NEUTRE_US};

    fn controleurs_neutres() -> (ControleurDirectionSol, ControleurRoulis, ControleurTangage, ControleurLacet) {
        (
            ControleurDirectionSol::nouveau(1.0, 0.0, 0.0, 1.0),
            ControleurRoulis::nouveau(1.0, 1.0, 0.0, 0.0, 1.0),
            ControleurTangage::nouveau(1.0, 1.0, 0.0, 0.0, 1.0),
            ControleurLacet::nouveau(1.0, 0.0, 0.0, 1.0),
        )
    }

    fn lancer_test() -> (
        watch::Sender<EtatNavigation>,
        watch::Sender<Vector3>,
        watch::Receiver<ConsignesServos>,
        HandlesControle,
    ) {
        let (tx_navigation, rx_navigation) = watch::channel(EtatNavigation::nul());
        let (tx_taux_corps, rx_taux_corps) = watch::channel(Vector3::zero());
        let (tx_consignes_servo, rx_consignes_servo) = watch::channel(ConsignesServos::neutre());

        let (direction_sol, roulis, tangage, lacet) = controleurs_neutres();
        let handles = lancer_controle(
            rx_navigation, rx_taux_corps, tx_consignes_servo,
            direction_sol, roulis, tangage, lacet,
        );

        (tx_navigation, tx_taux_corps, rx_consignes_servo, handles)
    }

    #[tokio::test]
    async fn phase_sol_par_defaut_reagit_a_une_erreur_de_cap() {
        let (tx_navigation, _tx_taux_corps, mut rx_consignes_servo, handles) = lancer_test();

        let mut nav = EtatNavigation::nul();
        nav.attitude.lacet = Angle::depuis_degres(10.0);
        tx_navigation.send(nav).unwrap();

        tokio::time::sleep(Duration::from_millis(50)).await;

        let consignes = rx_consignes_servo.borrow_and_update().clone();
        assert_ne!(consignes.direction, IMPULSION_NEUTRE_US);
        // Ailerons/profondeur : contrôleurs de vol gelés en phase sol.
        assert_eq!(consignes.ailerons, IMPULSION_NEUTRE_US);
        assert_eq!(consignes.profondeur, IMPULSION_NEUTRE_US);

        handles.arreter();
    }

    #[tokio::test]
    async fn phase_vol_pilote_ailerons_profondeur_et_direction() {
        let (tx_navigation, _tx_taux_corps, mut rx_consignes_servo, handles) = lancer_test();

        handles.tx_phase.send(PhaseControle::Vol).unwrap();
        handles.tx_consigne_roulis.send(ConsigneRoulis { roulis_cible: Angle::depuis_degres(0.0) }).unwrap();
        handles.tx_consigne_tangage.send(ConsigneTangage { tangage_cible: Angle::depuis_degres(0.0) }).unwrap();

        let mut nav = EtatNavigation::nul();
        nav.attitude.roulis = Angle::depuis_degres(10.0);
        nav.attitude.tangage = Angle::depuis_degres(-10.0);
        tx_navigation.send(nav).unwrap();

        tokio::time::sleep(Duration::from_millis(50)).await;

        let consignes = rx_consignes_servo.borrow_and_update().clone();
        assert_ne!(consignes.ailerons, IMPULSION_NEUTRE_US);
        assert_ne!(consignes.profondeur, IMPULSION_NEUTRE_US);

        handles.arreter();
    }

    #[tokio::test]
    async fn gaz_et_armement_au_minimum_par_defaut() {
        let (tx_navigation, _tx_taux_corps, mut rx_consignes_servo, handles) = lancer_test();

        let mut nav = EtatNavigation::nul();
        nav.attitude.lacet = Angle::depuis_degres(45.0);
        tx_navigation.send(nav).unwrap();

        tokio::time::sleep(Duration::from_millis(50)).await;

        let consignes = rx_consignes_servo.borrow_and_update().clone();
        assert_eq!(consignes.gaz, IMPULSION_MIN_US);
        assert!(!consignes.arme);

        handles.arreter();
    }

    #[tokio::test]
    async fn gaz_suit_la_consigne_une_fois_arme() {
        let (tx_navigation, _tx_taux_corps, mut rx_consignes_servo, handles) = lancer_test();

        handles.tx_arme.send(true).unwrap();
        handles.tx_consigne_gaz.send(ConsigneGaz { gaz: 1.0 }).unwrap();
        tx_navigation.send(EtatNavigation::nul()).unwrap();

        tokio::time::sleep(Duration::from_millis(50)).await;

        let consignes = rx_consignes_servo.borrow_and_update().clone();
        assert_eq!(consignes.gaz, IMPULSION_MAX_US);
        assert!(consignes.arme);

        handles.arreter();
    }

    #[tokio::test]
    async fn gaz_force_au_minimum_si_non_arme_malgre_consigne_non_nulle() {
        // Défense en profondeur (voir doc/controle.md et E0.8 du plan d'essais).
        let (tx_navigation, _tx_taux_corps, mut rx_consignes_servo, handles) = lancer_test();

        handles.tx_consigne_gaz.send(ConsigneGaz { gaz: 1.0 }).unwrap();
        // tx_arme jamais envoyé : reste à sa valeur par défaut (false).
        tx_navigation.send(EtatNavigation::nul()).unwrap();

        tokio::time::sleep(Duration::from_millis(50)).await;

        let consignes = rx_consignes_servo.borrow_and_update().clone();
        assert_eq!(consignes.gaz, IMPULSION_MIN_US);
        assert!(!consignes.arme);

        handles.arreter();
    }

    #[tokio::test]
    async fn aucune_commande_quand_cap_deja_atteint() {
        let (tx_navigation, _tx_taux_corps, mut rx_consignes_servo, handles) = lancer_test();

        let mut nav = EtatNavigation::nul();
        nav.attitude.lacet = Angle::depuis_degres(0.0); // égal à la consigne par défaut
        tx_navigation.send(nav).unwrap();

        tokio::time::sleep(Duration::from_millis(50)).await;

        let consignes = rx_consignes_servo.borrow_and_update().clone();
        assert_eq!(consignes.direction, IMPULSION_NEUTRE_US);

        handles.arreter();
    }

    #[tokio::test]
    async fn changement_de_phase_gele_et_reinitialise_le_controleur_devenu_inactif() {
        let (tx_navigation, _tx_taux_corps, mut rx_consignes_servo, handles) = lancer_test();

        // Phase sol : on fait accumuler de l'intégrale au contrôleur direction sol.
        let mut nav = EtatNavigation::nul();
        nav.attitude.lacet = Angle::depuis_degres(45.0);
        tx_navigation.send(nav).unwrap();
        tokio::time::sleep(Duration::from_millis(50)).await;

        // Bascule en phase vol : direction sol doit être gelé (et réinitialisé).
        handles.tx_phase.send(PhaseControle::Vol).unwrap();
        tx_navigation.send(nav).unwrap();
        tokio::time::sleep(Duration::from_millis(50)).await;

        // Retour en phase sol, cap déjà atteint : si l'intégrale n'avait pas
        // été purgée, la commande direction resterait non nulle.
        handles.tx_phase.send(PhaseControle::Sol).unwrap();
        let mut nav_atteint = EtatNavigation::nul();
        nav_atteint.attitude.lacet = Angle::depuis_degres(0.0);
        tx_navigation.send(nav_atteint).unwrap();
        tokio::time::sleep(Duration::from_millis(50)).await;

        let consignes = rx_consignes_servo.borrow_and_update().clone();
        assert_eq!(consignes.direction, IMPULSION_NEUTRE_US);

        handles.arreter();
    }
}
