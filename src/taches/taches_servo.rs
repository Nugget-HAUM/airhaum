// src/taches/taches_servo.rs
//! Thread de commande des servos via l'Arduino Nano (liaison série logicielle).
//!
//! Architecture identique aux threads capteurs :
//! signal d'arrêt atomique, canaux `watch` pour les consignes et l'état.
//!
//! # Canaux
//!
//! ```text
//! couche_controle ──── ConsignesServos (watch) ──▶ thread_servo ──── UART ──▶ Nano
//!                                                       │
//!                                      EtatArduino (watch) ──▶ tache_mission / surete
//! ```
//!
//! La couche contrôle écrit dans `tx_consignes` à chaque cycle (50 Hz).
//! Le thread envoie la dernière consigne disponible sans attente.
//!
//! # Fréquences
//!
//! - Envoi consignes : 50 Hz (période 20 ms)
//! - Lecture remontée : tentative à chaque itération (non-bloquant), trame reçue ~10 Hz

use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::Duration;
use tokio::sync::watch;

use crate::drivers::controleur_servo::{ConsignesServos, ControleurServo, EtatArduino};
use crate::hal::uart::PortSerie;

const PERIODE_ENVOI_MS: u64 = 20; // 50 Hz

// ─────────────────────────────────────────────────────────────────────────────
// Handle
// ─────────────────────────────────────────────────────────────────────────────

/// Handle retourné par [`lancer_servo`].
pub struct HandlesServo {
    /// Émet les consignes vers le thread (couche contrôle → thread servo).
    pub tx_consignes: watch::Sender<ConsignesServos>,
    /// Dernier état Arduino connu (thread servo → mission / sûreté).
    pub rx_etat: watch::Receiver<EtatArduino>,
    /// Vrai dès que la première trame de remontée valide a été reçue.
    pub liaison_detectee: Arc<AtomicBool>,
    /// Renseigné si le port série n'a pas pu être ouvert au démarrage.
    /// Le thread tourne sur port fictif ; aucune donnée réelle ne sera reçue.
    pub erreur_port: Option<String>,
    /// Handle du thread (pour jointure à l'arrêt).
    pub tache: std::thread::JoinHandle<()>,
    /// Signal d'arrêt partagé avec le thread.
    arret: Arc<AtomicBool>,
}

impl HandlesServo {
    /// Signale l'arrêt au thread servo. Non-bloquant.
    pub fn arreter(&self) {
        self.arret.store(true, Ordering::Relaxed);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Points d'entrée
// ─────────────────────────────────────────────────────────────────────────────

/// Lance le thread servo sur le port série donné.
///
/// Toujours infaillible : si le port série ne peut pas être ouvert (UART non
/// configuré, droits insuffisants), le thread démarre sur un port fictif et
/// [`HandlesServo::erreur_port`] est renseigné. Les erreurs UART survenant
/// après le démarrage sont journalisées mais ne font jamais paniquer le thread.
#[cfg(target_os = "linux")]
pub fn lancer_servo(chemin_port: &str) -> HandlesServo {
    use crate::hal::uart_linux::{PortSerieLinux, BAUDRATE_ARDUINO};
    use crate::hal::uart::PortSerieMock;

    match PortSerieLinux::nouveau(chemin_port, BAUDRATE_ARDUINO) {
        Ok(port) => lancer_avec_port(port),
        Err(e) => {
            log::warn!(target: "servo", "Port série {} inaccessible : {:?}", chemin_port, e);
            let mut h = lancer_avec_port(PortSerieMock::nouveau());
            h.erreur_port = Some(format!("{} inaccessible", chemin_port));
            h
        }
    }
}

/// Variante hors-Linux utilisant le mock UART (développement / CI).
#[cfg(not(target_os = "linux"))]
pub fn lancer_servo(_chemin_port: &str) -> HandlesServo {
    use crate::hal::uart::PortSerieMock;
    lancer_avec_port(PortSerieMock::nouveau())
}

// ─────────────────────────────────────────────────────────────────────────────
// Implémentation commune
// ─────────────────────────────────────────────────────────────────────────────

fn lancer_avec_port<P: PortSerie + 'static>(port: P) -> HandlesServo {
    let arret = Arc::new(AtomicBool::new(false));
    let liaison_detectee = Arc::new(AtomicBool::new(false));

    let (tx_consignes, rx_consignes) = watch::channel(ConsignesServos::neutre());
    let (tx_etat, rx_etat) = watch::channel(EtatArduino::absent());

    let arret_thread = Arc::clone(&arret);
    let liaison_thread = Arc::clone(&liaison_detectee);

    let tache = std::thread::Builder::new()
        .name("servo".into())
        .spawn(move || {
            thread_servo(
                ControleurServo::nouveau(port),
                rx_consignes,
                tx_etat,
                arret_thread,
                liaison_thread,
            )
        })
        .expect("Impossible de créer le thread servo");

    HandlesServo { tx_consignes, rx_etat, liaison_detectee, erreur_port: None, tache, arret }
}

// ─────────────────────────────────────────────────────────────────────────────
// Corps du thread
// ─────────────────────────────────────────────────────────────────────────────

fn thread_servo<P: PortSerie>(
    mut controleur: ControleurServo<P>,
    rx_consignes: watch::Receiver<ConsignesServos>,
    tx_etat: watch::Sender<EtatArduino>,
    arret: Arc<AtomicBool>,
    liaison_detectee: Arc<AtomicBool>,
) {
    let mut rc_perdu_precedent = true;
    let mut chien_de_garde_precedent = false;

    loop {
        if arret.load(Ordering::Relaxed) { return; }

        // ── Envoi des consignes (50 Hz) ───────────────────────────────────────
        let consignes = rx_consignes.borrow().clone();
        if let Err(e) = controleur.envoyer(&consignes) {
            log::warn!(target: "servo", "Erreur envoi consignes : {:?}", e);
        }

        // ── Lecture de la remontée (non-bloquant, ~10 Hz côté Nano) ──────────
        match controleur.recevoir() {
            Ok(Some(etat)) => {
                liaison_detectee.store(true, Ordering::Relaxed);

                if etat.rc_perdu && !rc_perdu_precedent {
                    log::warn!(target: "servo", "Signal RC perdu");
                } else if !etat.rc_perdu && rc_perdu_precedent {
                    log::info!(target: "servo", "Signal RC rétabli");
                }
                rc_perdu_precedent = etat.rc_perdu;

                if etat.chien_de_garde && !chien_de_garde_precedent {
                    log::warn!(target: "servo", "Chien de garde Nano déclenché — trame Pi non reçue depuis 2s");
                } else if !etat.chien_de_garde && chien_de_garde_precedent {
                    log::info!(target: "servo", "Chien de garde Nano levé");
                }
                chien_de_garde_precedent = etat.chien_de_garde;

                let _ = tx_etat.send(etat);
            }
            Ok(None) => {}
            Err(e) => log::warn!(target: "servo", "Erreur lecture remontée : {:?}", e),
        }

        std::thread::sleep(Duration::from_millis(PERIODE_ENVOI_MS));
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Tests
// ─────────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::drivers::controleur_servo::{IMPULSION_MIN_US, IMPULSION_NEUTRE_US, ModeArduino};

    fn lancer_mock() -> HandlesServo {
        use crate::hal::uart::PortSerieMock;
        lancer_avec_port(PortSerieMock::nouveau())
    }

    #[test]
    fn demarre_et_arrete_proprement() {
        let h = lancer_mock();
        std::thread::sleep(Duration::from_millis(50));
        h.arreter();
        h.tache.join().expect("thread servo devrait se terminer");
    }

    #[test]
    fn etat_initial_absent() {
        let h = lancer_mock();
        let etat = h.rx_etat.borrow().clone();
        assert!(etat.rc_perdu, "état initial doit indiquer RC perdu");
        assert_eq!(etat.gaz_applique, IMPULSION_MIN_US);
        h.arreter();
    }

    #[test]
    fn consignes_neutre_par_defaut() {
        let h = lancer_mock();
        let c = h.tx_consignes.borrow().clone();
        assert!(!c.arme);
        assert_eq!(c.ailerons,   IMPULSION_NEUTRE_US);
        assert_eq!(c.gaz,        IMPULSION_MIN_US);
        h.arreter();
    }

    #[test]
    fn mise_a_jour_consignes() {
        let h = lancer_mock();
        h.tx_consignes.send(ConsignesServos {
            ailerons: 1700, profondeur: 1400, gaz: 1300, direction: 1600, arme: true,
        }).unwrap();
        std::thread::sleep(Duration::from_millis(30));
        let c = h.tx_consignes.borrow().clone();
        assert_eq!(c.ailerons, 1700);
        assert!(c.arme);
        h.arreter();
    }

    #[test]
    fn mode_arduino_par_defaut_autopilote() {
        let h = lancer_mock();
        let etat = h.rx_etat.borrow().clone();
        assert_eq!(etat.mode, ModeArduino::Autopilote);
        h.arreter();
    }
}
