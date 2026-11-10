// src/taches/taches_enregistrement.rs
//! Tâche Tokio d'enregistrement des données de vol dans la boîte noire.
//!
//! Un échantillon complet, cible `nav`, à 10 Hz de l'armement jusqu'à la fin
//! de la mission, et à 1 Hz en dehors (attente sur la check-liste) pour garder
//! une trace légère de ce qui précède. Chaque changement d'état vol donne aussi
//! un échantillon immédiat, et l'entrée en `SauvegardeDonnees` pousse la boîte
//! noire sur disque. Voir `doc/journalisation.md`.
//!
//! Ne lit que des canaux `watch`, sans jamais les modifier. Passe par la façade
//! `log`, jamais bloquante pour les échantillons.

use std::sync::{atomic::{AtomicBool, Ordering}, Arc};
use tokio::sync::watch;
use tokio::time::{interval, Duration};

use crate::capteurs::fusion_altitude::AltitudeFusionnee;
use crate::drivers::controleur_servo::{ConsignesServos, EtatArduino};
use crate::estimation::EtatNavigation;
use crate::mission::etat_machine::EtatVol;
use crate::surete::EtatSecurite;

/// Période de la boucle : 10 Hz, cadence de la mission.
const PERIODE_MS: u64 = 100;
/// Un échantillon sur 10 hors mission : 1 Hz.
const DIVISEUR_HORS_MISSION: u32 = 10;

/// Retourné par [`lancer_enregistrement`].
pub struct EnregistrementVol {
    tache: tokio::task::JoinHandle<()>,
    arret: Arc<AtomicBool>,
}

impl EnregistrementVol {
    /// Signale l'arrêt à la tâche. Non-bloquant.
    pub fn arreter(&self) {
        self.arret.store(true, Ordering::Relaxed);
        self.tache.abort();
    }
}

/// Vrai si l'état vol impose la cadence complète : mission armée, désarmement
/// compris pour enregistrer la fin de mission jusqu'au bout.
fn cadence_complete(etat: &EtatVol) -> bool {
    etat.est_arme() || *etat == EtatVol::Desarmement
}

fn optionnel<T: std::fmt::Display>(valeur: Option<T>) -> String {
    valeur.map_or_else(|| "-".to_string(), |v| v.to_string())
}

/// Une ligne `clé=valeur`. L'état sécurité, seul champ libre, reste en dernier.
fn formater_echantillon(
    etat_vol: &EtatVol,
    securite: &EtatSecurite,
    esp32: &EtatArduino,
    consignes: &ConsignesServos,
    nav: &EtatNavigation,
    altitude: &AltitudeFusionnee,
) -> String {
    format!(
        "etat={} mode={:?} arme={} rc_perdu={} chien_de_garde={} \
         gaz_applique={} ch1={} ch2={} ch3={} ch4={} ch5={} \
         c_ail={} c_pro={} c_gaz={} c_dir={} \
         roulis={:.2} tangage={:.2} lacet={:.2} \
         pos_n={:.2} pos_e={:.2} pos_b={:.2} vit_n={:.2} vit_e={:.2} vit_b={:.2} \
         origine={} alt_m={} sol_mm={} securite=\"{}\"",
        etat_vol, esp32.mode, consignes.arme, esp32.rc_perdu, esp32.chien_de_garde,
        esp32.gaz_applique,
        esp32.canaux_rc[0], esp32.canaux_rc[1], esp32.canaux_rc[2], esp32.canaux_rc[3], esp32.canaux_rc[4],
        consignes.ailerons, consignes.profondeur, consignes.gaz, consignes.direction,
        nav.attitude.roulis.degres(), nav.attitude.tangage.degres(), nav.attitude.lacet.degres(),
        nav.position_ned.x, nav.position_ned.y, nav.position_ned.z,
        nav.vitesse_ned.x, nav.vitesse_ned.y, nav.vitesse_ned.z,
        nav.origine_definie, optionnel(altitude.altitude_m.map(|a| format!("{a:.2}"))),
        optionnel(altitude.hauteur_sol_mm), securite,
    )
}

/// Lance la tâche d'enregistrement. Les récepteurs sont des clones ou des
/// abonnements : leurs propriétaires d'origine n'en sont pas affectés.
///
/// Doit être appelé depuis un contexte Tokio (après `#[tokio::main]`).
pub fn lancer_enregistrement(
    rx_etat_vol: watch::Receiver<EtatVol>,
    rx_securite: watch::Receiver<EtatSecurite>,
    rx_esp32: watch::Receiver<EtatArduino>,
    rx_consignes: watch::Receiver<ConsignesServos>,
    rx_navigation: watch::Receiver<EtatNavigation>,
    rx_altitude: watch::Receiver<AltitudeFusionnee>,
) -> EnregistrementVol {
    let arret = Arc::new(AtomicBool::new(false));
    let arret_tache = Arc::clone(&arret);

    let tache = tokio::spawn(async move {
        let mut tick = interval(Duration::from_millis(PERIODE_MS));
        let mut compteur = 0u32;
        let mut etat_precedent: Option<EtatVol> = None;

        while !arret_tache.load(Ordering::Relaxed) {
            tick.tick().await;

            let etat_vol = rx_etat_vol.borrow().clone();
            let changement = etat_precedent.as_ref() != Some(&etat_vol);
            compteur = compteur.wrapping_add(1);

            if changement || cadence_complete(&etat_vol) || compteur % DIVISEUR_HORS_MISSION == 0 {
                let securite = rx_securite.borrow().clone();
                let esp32 = rx_esp32.borrow().clone();
                let consignes = rx_consignes.borrow().clone();
                let nav = *rx_navigation.borrow();
                let altitude = *rx_altitude.borrow();
                log::info!(target: "nav", "{}",
                    formater_echantillon(&etat_vol, &securite, &esp32, &consignes, &nav, &altitude));
            }

            // Après l'échantillon, pour que la dernière ligne de la mission
            // soit elle aussi sur disque.
            if changement && etat_vol == EtatVol::SauvegardeDonnees {
                log::info!(target: "mission", "Fin de mission : vidage de la boîte noire");
                log::logger().flush();
            }

            etat_precedent = Some(etat_vol);
        }
    });

    EnregistrementVol { tache, arret }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::drivers::controleur_servo::ModeArduino;

    #[test]
    fn cadence_complete_de_l_armement_a_la_fin_de_mission() {
        assert!(!cadence_complete(&EtatVol::AttenteArmement));
        assert!(cadence_complete(&EtatVol::Armement));
        assert!(cadence_complete(&EtatVol::RoulageAuSol));
        assert!(cadence_complete(&EtatVol::Desarmement));
        assert!(!cadence_complete(&EtatVol::SauvegardeDonnees));
    }

    #[test]
    fn echantillon_contient_les_donnees_de_vol() {
        let esp32 = EtatArduino { mode: ModeArduino::Manuel, gaz_applique: 1234, ..EtatArduino::absent() };
        let consignes = ConsignesServos { gaz: 1100, arme: true, ..ConsignesServos::neutre() };
        let securite = EtatSecurite::ArretUrgence { raison: "pilote".into() };

        let ligne = formater_echantillon(
            &EtatVol::RoulageAuSol, &securite, &esp32, &consignes,
            &EtatNavigation::nul(), &AltitudeFusionnee::nulle(),
        );

        for attendu in [
            "etat=RoulageAuSol", "mode=Manuel", "arme=true", "gaz_applique=1234",
            "c_gaz=1100", "alt_m=-", "securite=\"ArretUrgence: pilote\"",
        ] {
            assert!(ligne.contains(attendu), "{attendu} absent de : {ligne}");
        }
    }
}
