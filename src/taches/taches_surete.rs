// src/taches/taches_surete.rs
//! Tâche Tokio de supervision sécurité (MAÉ sécurité).
//!
//! Poll périodique (10 Hz) : agrège la santé des capteurs (`surete::sante`)
//! et le respect des limites de vol (`surete::limites`), produit un
//! `EtatSecurite` via `surete::urgences::MaeSecurite` et le publie sur un
//! canal `watch` consommé par la MAÉ vol et la télémétrie.
//!
//! # Sources surveillées
//!
//! | Source  | Canal                           | Signal instantané                 |
//! |---------|----------------------------------|------------------------------------|
//! | Baro    | `HandlesCapteurs::rx_baro`       | `valide` / `erreurs_consecutives`  |
//! | Télém.  | `HandlesCapteurs::rx_telem`      | `valide` / `erreurs_consecutives`  |
//! | IMU     | `HandlesEstimation::rx_attitude` | fraîcheur uniquement               |
//! | GPS     | `HandlesGps::rx_gps`             | `valide` / `erreurs_consecutives`  |
//! | Arduino | `HandlesServo::rx_etat`          | `rc_perdu` / `chien_de_garde`      |
//! | ESP32 (mode) | `HandlesServo::rx_etat`     | retour en `Manuel` après un passage en `Autopilote` **armé** → `ArretUrgence` (voir `autopilote_arme_engage`, ci-dessous — avant l'armement, SWA est libre : tout ce qui précède relève du contrôle) |
//! | Console | `Arc<AtomicBool>` partagé avec `main.rs` | arrêt manuel opérateur (appoint) |
//!
//! # Vivacité des threads
//!
//! Aucun `JoinHandle` n'est partagé avec cette tâche : chaque thread capteur
//! publie sur son canal `watch` à chaque itération, y compris en cas
//! d'erreur (voir `taches_capteurs.rs`). La détection de péremption de
//! `MoniteurSante` capture donc déjà le cas "thread mort" comme cas
//! particulier de "plus aucune donnée publiée", sans complexifier la
//! propriété des handles entre `main.rs` et cette tâche.
//!
//! Le FIFO IMU brut est consommé exclusivement par `taches_estimation` : la
//! santé IMU est donc dérivée de la fraîcheur de `rx_attitude` (publiée à
//! chaque prédiction EKF) plutôt que d'un signal dédié — si l'attitude cesse
//! d'être mise à jour, ça révèle un problème sur l'IMU *ou* l'estimation,
//! ce qui est de toute façon ce qui compte du point de vue sûreté.

use std::sync::{atomic::{AtomicBool, Ordering}, Arc};
use tokio::sync::watch;
use tokio::time::{interval, Duration};

use crate::drivers::controleur_servo::{EtatArduino, ModeArduino};
use crate::estimation::{Attitude, EtatNavigation};
use crate::surete::limites::{evaluer_limites, LimitesVol};
use crate::surete::sante::{evaluer_arduino, evaluer_capteur, pire_etat, EtatSante, MoniteurSante};
use crate::surete::urgences::{EntreesSecurite, MaeSecurite};
use crate::surete::EtatSecurite;
use crate::taches::taches_capteurs::{MesureBaro, MesureTelem};
use crate::taches::taches_gps::MesureGps;

/// Période du cycle de supervision (10 Hz).
const PERIODE_MS: u64 = 100;

/// Délai de péremption des sources à fréquence élevée (baro/télémètre/attitude).
const PEREMPTION_RAPIDE: Duration = Duration::from_millis(500);
/// Délai de péremption GPS — reprend le seuil documenté dans `doc/surete.md`
/// ("données absentes > 5 s" en croisière).
const PEREMPTION_GPS: Duration = Duration::from_secs(5);
/// Délai de péremption Arduino (trame de remontée à 10 Hz nominal).
const PEREMPTION_ARDUINO: Duration = Duration::from_millis(500);

// ─────────────────────────────────────────────────────────────────────────────
// Handle retourné par lancer_surete()
// ─────────────────────────────────────────────────────────────────────────────

/// Tout ce dont le reste du système a besoin pour consulter l'état sécurité.
pub struct HandlesSurete {
    /// Dernier état sécurité connu (canal à valeur courante).
    pub rx_securite: watch::Receiver<EtatSecurite>,
    /// Tâche Tokio — annulable via `arreter()`.
    tache: tokio::task::JoinHandle<()>,
    /// Signal d'arrêt partagé avec la tâche.
    arret: Arc<AtomicBool>,
}

impl HandlesSurete {
    /// Signale l'arrêt à la tâche sécurité. Non-bloquant.
    pub fn arreter(&self) {
        self.arret.store(true, Ordering::Relaxed);
        self.tache.abort();
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Point d'entrée
// ─────────────────────────────────────────────────────────────────────────────

/// Lance la tâche de supervision sécurité.
///
/// Les récepteurs `watch` sont clonés depuis les handles des autres tâches
/// (`HandlesCapteurs`, `HandlesEstimation`, `HandlesGps`, `HandlesServo`) —
/// ils restent utilisables normalement par leurs propriétaires d'origine.
///
/// `limites` configure la géofence/enveloppe de vol (`LimitesVol::default()`
/// désactive toute vérification tant que les valeurs réelles du terrain de
/// vol ne sont pas connues).
///
/// `rx_mission_armee` indique si la mission est armée (`EtatVol::est_arme`,
/// publié par `taches_mission`). Simple information en lecture : la mission
/// ne commande jamais la sécurité.
///
/// `arret_urgence_console` est un drapeau partagé avec `main.rs` : le
/// positionner à `true` (ex. touche dédiée de la console) force
/// `EtatSecurite::ArretUrgence` dès le prochain cycle. Solution d'appoint
/// pour les premiers vols rapprochés — le canal de sécurité principal reste
/// le RC/ESP32 (voir `mode_manuel_rc` dans `surete::urgences`), qui rend la
/// main à l'opérateur, gaz inclus, indépendamment du logiciel du Pi.
///
/// Doit être appelé depuis un contexte Tokio (après `#[tokio::main]`).
pub fn lancer_surete(
    rx_baro: watch::Receiver<MesureBaro>,
    rx_telem: watch::Receiver<MesureTelem>,
    rx_attitude: watch::Receiver<Attitude>,
    rx_navigation: watch::Receiver<EtatNavigation>,
    rx_gps: watch::Receiver<MesureGps>,
    rx_etat_arduino: watch::Receiver<EtatArduino>,
    rx_mission_armee: watch::Receiver<bool>,
    limites: LimitesVol,
    arret_urgence_console: Arc<AtomicBool>,
) -> HandlesSurete {
    let arret = Arc::new(AtomicBool::new(false));
    let (tx_securite, rx_securite) = watch::channel(EtatSecurite::Normal);

    let arret_tache = Arc::clone(&arret);
    let tache = tokio::spawn(async move {
        tache_surete(
            rx_baro, rx_telem, rx_attitude, rx_navigation, rx_gps, rx_etat_arduino,
            rx_mission_armee, limites, arret_urgence_console, tx_securite, arret_tache,
        ).await;
    });

    HandlesSurete { rx_securite, tache, arret }
}

// ─────────────────────────────────────────────────────────────────────────────
// Corps de la tâche
// ─────────────────────────────────────────────────────────────────────────────

/// État et raison sans leurs valeurs numériques : « aucune donnée depuis 15,9 s »
/// ou « 6 erreurs consécutives » restent une seule et même situation.
fn situation_a_journaliser(etat: &EtatSecurite) -> String {
    etat.to_string().chars().filter(|c| !c.is_ascii_digit()).collect()
}

async fn tache_surete(
    mut rx_baro: watch::Receiver<MesureBaro>,
    mut rx_telem: watch::Receiver<MesureTelem>,
    mut rx_attitude: watch::Receiver<Attitude>,
    rx_navigation: watch::Receiver<EtatNavigation>,
    mut rx_gps: watch::Receiver<MesureGps>,
    mut rx_etat_arduino: watch::Receiver<EtatArduino>,
    rx_mission_armee: watch::Receiver<bool>,
    limites: LimitesVol,
    arret_urgence_console: Arc<AtomicBool>,
    tx_securite: watch::Sender<EtatSecurite>,
    arret: Arc<AtomicBool>,
) {
    let mut mon_baro    = MoniteurSante::nouveau("baro", PEREMPTION_RAPIDE);
    let mut mon_telem   = MoniteurSante::nouveau("telem", PEREMPTION_RAPIDE);
    let mut mon_imu     = MoniteurSante::nouveau("imu", PEREMPTION_RAPIDE);
    let mut mon_gps     = MoniteurSante::nouveau("gps", PEREMPTION_GPS);
    let mut mon_arduino = MoniteurSante::nouveau("arduino", PEREMPTION_ARDUINO);
    let mut mae = MaeSecurite::nouveau();

    // Latch : devient vrai dès que l'ESP32 rapporte le mode Autopilote alors
    // que la mission est armée, et retombe au désarmement. Avant l'armement,
    // SWA est libre (contrôles, test des gouvernes) et ne doit pas déclencher
    // d'arrêt d'urgence — seul un *retour* au manuel après un passage en
    // autopilote armé signale une vraie reprise en main.
    let mut autopilote_arme_engage = false;
    let mut derniere_situation = situation_a_journaliser(&EtatSecurite::Normal);

    let mut tick = interval(Duration::from_millis(PERIODE_MS));

    loop {
        if arret.load(Ordering::Relaxed) { break; }
        tick.tick().await;

        let nouveau_baro = rx_baro.has_changed().unwrap_or(false);
        let baro = rx_baro.borrow_and_update().clone();
        mon_baro.actualiser(
            evaluer_capteur("baro", baro.valide, baro.erreurs_consecutives),
            nouveau_baro,
            true,
        );

        let nouveau_telem = rx_telem.has_changed().unwrap_or(false);
        let telem = rx_telem.borrow_and_update().clone();
        mon_telem.actualiser(
            evaluer_capteur("telem", telem.valide, telem.erreurs_consecutives),
            nouveau_telem,
            true,
        );

        let nouveau_imu = rx_attitude.has_changed().unwrap_or(false);
        let _ = rx_attitude.borrow_and_update();
        mon_imu.actualiser(EtatSante::Nominal, nouveau_imu, true);

        let nouveau_gps = rx_gps.has_changed().unwrap_or(false);
        let gps = rx_gps.borrow_and_update().clone();
        mon_gps.actualiser(
            evaluer_capteur("gps", gps.valide, gps.erreurs_consecutives),
            nouveau_gps,
            true,
        );

        let nouveau_arduino = rx_etat_arduino.has_changed().unwrap_or(false);
        let etat_arduino = rx_etat_arduino.borrow_and_update().clone();
        mon_arduino.actualiser(
            evaluer_arduino(etat_arduino.rc_perdu, etat_arduino.chien_de_garde),
            nouveau_arduino,
            true,
        );

        if !*rx_mission_armee.borrow() {
            autopilote_arme_engage = false;
        } else if etat_arduino.mode == ModeArduino::Autopilote {
            autopilote_arme_engage = true;
        }

        let nav = *rx_navigation.borrow();
        let etat_limites = evaluer_limites(&nav, &limites);

        let sante_globale = pire_etat([
            mon_baro.etat(),
            mon_telem.etat(),
            mon_imu.etat(),
            mon_gps.etat(),
            mon_arduino.etat(),
        ]);

        let entrees = EntreesSecurite {
            sante_capteurs: &sante_globale,
            limites: &etat_limites,
            lora_perdue: None,
            arret_urgence_sol: Some(arret_urgence_console.load(Ordering::Relaxed)),
            mode_manuel_rc: autopilote_arme_engage.then_some(etat_arduino.mode == ModeArduino::Manuel),
            tension_batterie_v: None,
        };

        let etat = mae.actualiser(&entrees).clone();

        // Une ligne par changement : l'état complet est enregistré à 10 Hz en
        // mission (`taches_enregistrement`).
        let situation = situation_a_journaliser(&etat);
        if situation != derniere_situation {
            if etat.est_critique() {
                log::error!(target: "surete", "État sécurité : {}", etat);
            } else if matches!(etat, EtatSecurite::Normal) {
                log::info!(target: "surete", "État sécurité : {}", etat);
            } else {
                log::warn!(target: "surete", "État sécurité : {}", etat);
            }
            derniere_situation = situation;
        }

        let _ = tx_securite.send(etat);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Tests
// ─────────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::estimation::{Attitude, EtatNavigation};
    use crate::taches::taches_capteurs::{MesureBaro, MesureTelem};
    use crate::taches::taches_gps::MesureGps;
    use crate::types::geometrie::Vector3;
    use crate::types::messages::Horodatage;

    fn navigation_sans_origine() -> EtatNavigation {
        EtatNavigation {
            horodatage: Horodatage::maintenant(),
            attitude: Attitude::nulle(),
            position_ned: Vector3::zero(),
            vitesse_ned: Vector3::zero(),
            biais_gyro: Vector3::zero(),
            origine_definie: false,
        }
    }

    fn arduino(mode: ModeArduino) -> EtatArduino {
        EtatArduino { mode, rc_perdu: false, chien_de_garde: false, ..EtatArduino::absent() }
    }

    fn lancer_test(etat_initial_arduino: EtatArduino) -> (watch::Sender<EtatArduino>, watch::Sender<bool>, HandlesSurete) {
        let (_tx_baro, rx_baro) = watch::channel(MesureBaro { donnees: None, valide: true, erreurs_consecutives: 0 });
        let (_tx_telem, rx_telem) = watch::channel(MesureTelem { distance_mm: None, valide: true, erreurs_consecutives: 0 });
        let (_tx_attitude, rx_attitude) = watch::channel(Attitude::nulle());
        let (_tx_nav, rx_nav) = watch::channel(navigation_sans_origine());
        let (_tx_gps, rx_gps) = watch::channel(MesureGps { donnees: None, valide: true, erreurs_consecutives: 0 });
        let (tx_arduino, rx_arduino) = watch::channel(etat_initial_arduino);
        let (tx_mission_armee, rx_mission_armee) = watch::channel(false);

        let handles = lancer_surete(
            rx_baro, rx_telem, rx_attitude, rx_nav, rx_gps, rx_arduino, rx_mission_armee,
            LimitesVol::default(), Arc::new(AtomicBool::new(false)),
        );

        (tx_arduino, tx_mission_armee, handles)
    }

    #[test]
    fn situation_ignore_les_valeurs_mais_pas_la_cause() {
        let mineure = |raison: &str| EtatSecurite::AlerteMineure { raison: raison.into() };
        assert_eq!(
            situation_a_journaliser(&mineure("telem : 5 erreurs consécutives")),
            situation_a_journaliser(&mineure("telem : 12 erreurs consécutives")),
        );
        assert_ne!(
            situation_a_journaliser(&mineure("telem : 5 erreurs consécutives")),
            situation_a_journaliser(&mineure("gps : 5 erreurs consécutives")),
        );
        assert_ne!(
            situation_a_journaliser(&mineure("gps")),
            situation_a_journaliser(&EtatSecurite::AlerteMajeure { raison: "gps".into() }),
        );
    }

    #[tokio::test]
    async fn mode_manuel_au_repos_sans_autopilote_prealable_ne_declenche_pas_arret() {
        let (_tx_arduino, _tx_mission_armee, mut handles) = lancer_test(arduino(ModeArduino::Manuel));

        tokio::time::sleep(Duration::from_millis(350)).await;

        assert_eq!(*handles.rx_securite.borrow_and_update(), EtatSecurite::Normal);
        handles.arreter();
    }

    #[tokio::test]
    async fn bascules_non_arme_ne_declenchent_pas_arret() {
        let (tx_arduino, _tx_mission_armee, mut handles) = lancer_test(arduino(ModeArduino::Autopilote));

        tokio::time::sleep(Duration::from_millis(150)).await;
        tx_arduino.send(arduino(ModeArduino::Manuel)).unwrap();
        tokio::time::sleep(Duration::from_millis(150)).await;

        assert_eq!(*handles.rx_securite.borrow_and_update(), EtatSecurite::Normal);
        handles.arreter();
    }

    #[tokio::test]
    async fn etat_absent_au_demarrage_ne_declenche_pas_arret() {
        // `EtatArduino::absent()` vaut Autopilote : la première trame reçue en
        // manuel ne doit pas être prise pour une reprise en main. Il vaut aussi
        // RC perdu : une alerte mineure transitoire reste donc normale ici.
        let (tx_arduino, _tx_mission_armee, mut handles) = lancer_test(EtatArduino::absent());

        tokio::time::sleep(Duration::from_millis(150)).await;
        tx_arduino.send(arduino(ModeArduino::Manuel)).unwrap();
        tokio::time::sleep(Duration::from_millis(150)).await;

        assert!(!handles.rx_securite.borrow_and_update().est_critique());
        handles.arreter();
    }

    #[tokio::test]
    async fn retour_manuel_apres_autopilote_arme_declenche_arret_urgence() {
        let (tx_arduino, tx_mission_armee, mut handles) = lancer_test(arduino(ModeArduino::Autopilote));
        tx_mission_armee.send(true).unwrap();

        // Laisser la tâche voir au moins un cycle en Autopilote avant le retour au manuel.
        tokio::time::sleep(Duration::from_millis(150)).await;
        tx_arduino.send(arduino(ModeArduino::Manuel)).unwrap();
        tokio::time::sleep(Duration::from_millis(150)).await;

        assert!(matches!(*handles.rx_securite.borrow_and_update(), EtatSecurite::ArretUrgence { .. }));
        handles.arreter();
    }

    #[tokio::test]
    async fn arme_en_manuel_sans_passage_en_autopilote_ne_declenche_pas_arret() {
        let (_tx_arduino, tx_mission_armee, mut handles) = lancer_test(arduino(ModeArduino::Manuel));
        tx_mission_armee.send(true).unwrap();

        tokio::time::sleep(Duration::from_millis(350)).await;

        assert_eq!(*handles.rx_securite.borrow_and_update(), EtatSecurite::Normal);
        handles.arreter();
    }

    #[tokio::test]
    async fn retour_en_autopilote_leve_l_arret_urgence() {
        let (tx_arduino, tx_mission_armee, mut handles) = lancer_test(arduino(ModeArduino::Autopilote));
        tx_mission_armee.send(true).unwrap();

        tokio::time::sleep(Duration::from_millis(150)).await;
        tx_arduino.send(arduino(ModeArduino::Manuel)).unwrap();
        tokio::time::sleep(Duration::from_millis(150)).await;
        assert!(handles.rx_securite.borrow_and_update().est_critique());

        // Volonté de l'opérateur : rendre la main à l'autopilote, gaz inclus (doc/surete.md).
        tx_arduino.send(arduino(ModeArduino::Autopilote)).unwrap();
        tokio::time::sleep(Duration::from_millis(1300)).await;
        assert!(!handles.rx_securite.borrow_and_update().est_critique());

        handles.arreter();
    }

    #[tokio::test]
    async fn desarmement_rend_swa_libre() {
        let (tx_arduino, tx_mission_armee, mut handles) = lancer_test(arduino(ModeArduino::Autopilote));
        tx_mission_armee.send(true).unwrap();
        tokio::time::sleep(Duration::from_millis(150)).await;

        tx_mission_armee.send(false).unwrap();
        tokio::time::sleep(Duration::from_millis(150)).await;
        tx_arduino.send(arduino(ModeArduino::Manuel)).unwrap();
        tokio::time::sleep(Duration::from_millis(150)).await;

        assert_eq!(*handles.rx_securite.borrow_and_update(), EtatSecurite::Normal);
        handles.arreter();
    }
}
