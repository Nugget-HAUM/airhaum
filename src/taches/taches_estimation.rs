// src/taches/taches_estimation.rs
//! Tâche Tokio d'estimation d'état
//!
//! Consomme le FIFO IMU, les canaux baromètre, télémètre et GPS en continu,
//! applique le prétraitement et le filtre de Kalman étendu global, et publie
//! l'état de navigation complet via des canaux à valeur courante.
//!
//! # Rôle dans l'architecture
//!
//! ```text
//! thread capteur-mpu9250                tache-estimation           consommateurs
//! ──────────────────────                ─────────────────           ────────────
//!  MesureImu (FIFO mpsc) ──[prendre]──▶  TraitementImu
//!                                        EkfNavigation         ──▶  rx_navigation (watch)
//!                                                              ──▶  rx_attitude   (watch)
//! thread capteur-bmp280
//! thread capteur-vl53l0x                                       ──▶  rx_altitude   (watch)
//!  MesureBaro  (watch)  ──[clone]──▶    FusionAltitude
//!  MesureTelem (watch)  ──[clone]──▶
//!
//! thread capteur-gps
//!  MesureGps   (watch)  ──[clone]──▶    EkfNavigation.corriger_gps
//! ```

use std::sync::{Arc, atomic::{AtomicBool, Ordering}};
use tokio::sync::{watch, mpsc};

use crate::capteurs::fusion_altitude::{AltitudeFusionnee, fusionner};
use crate::capteurs::traitement_imu::TraitementImu;
use crate::estimation::{Attitude, EkfNavigation, EtatNavigation};
use crate::taches::taches_capteurs::{MesureBaro, MesureImu, MesureTelem};
use crate::taches::taches_gps::MesureGps;
use crate::types::Pression;

// ─────────────────────────────────────────────────────────────────────────────
// Handle retourné par lancer_estimation()
// ─────────────────────────────────────────────────────────────────────────────

/// Tout ce dont le reste du système a besoin pour consommer l'état estimé.
pub struct HandlesEstimation {
    /// Dernière attitude estimée — rétrocompatibilité avec les consommateurs existants.
    pub rx_attitude:   watch::Receiver<Attitude>,
    /// Dernière altitude fusionnée (baromètre + télémètre).
    pub rx_altitude:   watch::Receiver<AltitudeFusionnee>,
    /// État de navigation complet (attitude + position NED + vitesse + biais gyro).
    pub rx_navigation: watch::Receiver<EtatNavigation>,
    /// Tâche Tokio — annulable via `arreter()`.
    tache: tokio::task::JoinHandle<()>,
    /// Signal d'arrêt partagé avec la tâche.
    arret: Arc<AtomicBool>,
}

impl HandlesEstimation {
    /// Signale l'arrêt à la tâche d'estimation. Non-bloquant.
    pub fn arreter(&self) {
        self.arret.store(true, Ordering::Relaxed);
        self.tache.abort();
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Point d'entrée
// ─────────────────────────────────────────────────────────────────────────────

/// Lance la tâche d'estimation d'état.
///
/// - `rx_imu`              : FIFO IMU — transféré depuis `HandlesCapteurs::prendre_rx_imu()`.
/// - `rx_baro` / `rx_telem`: canaux watch clonés depuis `HandlesCapteurs`.
/// - `rx_gps`              : canal watch cloné depuis `HandlesGps`.
/// - `pression_reference`  : pression sol de calibration.
///
/// Doit être appelé depuis un contexte Tokio (après `#[tokio::main]`).
pub fn lancer_estimation(
    rx_imu:             mpsc::Receiver<MesureImu>,
    rx_baro:            watch::Receiver<MesureBaro>,
    rx_telem:           watch::Receiver<MesureTelem>,
    rx_gps:             watch::Receiver<MesureGps>,
    pression_reference: Pression,
) -> HandlesEstimation {
    let arret = Arc::new(AtomicBool::new(false));
    let (tx_attitude,   rx_attitude)   = watch::channel(Attitude::nulle());
    let (tx_altitude,   rx_altitude)   = watch::channel(AltitudeFusionnee::nulle());
    let (tx_navigation, rx_navigation) = watch::channel(EtatNavigation::nul());

    let arret_tache = Arc::clone(&arret);
    let tache = tokio::spawn(async move {
        tache_estimation(
            rx_imu, rx_baro, rx_telem, rx_gps,
            tx_attitude, tx_altitude, tx_navigation,
            pression_reference,
            arret_tache,
        ).await;
    });

    HandlesEstimation { rx_attitude, rx_altitude, rx_navigation, tache, arret }
}

// ─────────────────────────────────────────────────────────────────────────────
// Corps de la tâche
// ─────────────────────────────────────────────────────────────────────────────

async fn tache_estimation(
    mut rx_imu:   mpsc::Receiver<MesureImu>,
    mut rx_baro:  watch::Receiver<MesureBaro>,
    mut rx_telem: watch::Receiver<MesureTelem>,
    mut rx_gps:   watch::Receiver<MesureGps>,
    tx_attitude:   watch::Sender<Attitude>,
    tx_altitude:   watch::Sender<AltitudeFusionnee>,
    tx_navigation: watch::Sender<EtatNavigation>,
    pression_reference: Pression,
    arret: Arc<AtomicBool>,
) {
    let mut proc   = TraitementImu::nouveau();
    let mut filtre = EkfNavigation::nouveau();

    loop {
        if arret.load(Ordering::Relaxed) { break; }

        tokio::select! {
            // ── IMU : prédiction + correction accéléromètre ──────────────────
            mesure = rx_imu.recv() => {
                match mesure {
                    Some(m) => {
                        if let Some(traitee) = proc.traiter(&m) {
                            let dt = traitee.dt_s.unwrap_or(0.0);
                            filtre.predire(
                                traitee.donnees.accelerometre,
                                traitee.donnees.gyroscope,
                                dt,
                            );
                            filtre.corriger_accel(traitee.donnees.accelerometre);
                            let nav = filtre.etat();
                            let _ = tx_navigation.send(nav);
                            let _ = tx_attitude.send(nav.attitude);
                        }
                    }
                    None => {
                        log::info!(target: "estimation", "Canal IMU fermé — tâche terminée");
                        break;
                    }
                }
            }

            // ── GPS : correction position + vitesse ──────────────────────────
            Ok(_) = rx_gps.changed() => {
                let mesure = rx_gps.borrow_and_update().clone();
                if let Some(gps) = mesure.donnees {
                    if gps.type_fix.est_valide() {
                        filtre.corriger_gps(&gps);
                        let nav = filtre.etat();
                        let _ = tx_navigation.send(nav);
                        let _ = tx_attitude.send(nav.attitude);
                    }
                }
            }

            // ── Baromètre : nouvelle pression disponible ─────────────────────
            Ok(_) = rx_baro.changed() => {
                let baro  = rx_baro.borrow_and_update().clone();
                let telem = rx_telem.borrow().clone();
                let alt = fusionner(&baro, &telem, pression_reference);
                let _ = tx_altitude.send(alt);
            }

            // ── Télémètre : nouvelle distance disponible ─────────────────────
            Ok(_) = rx_telem.changed() => {
                let baro  = rx_baro.borrow().clone();
                let telem = rx_telem.borrow_and_update().clone();
                let alt = fusionner(&baro, &telem, pression_reference);
                let _ = tx_altitude.send(alt);
            }
        }
    }
}
