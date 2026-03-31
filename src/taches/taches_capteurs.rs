// src/taches/taches_capteurs.rs
//! Tâches asynchrones de lecture continue des capteurs
//!
//! Ce module instancie et pilote les drivers capteurs dans des tâches Tokio
//! indépendantes. Chaque tâche :
//!
//! - Initialise son capteur au démarrage
//! - Lit en continu et publie la dernière mesure via `tokio::sync::watch`
//! - Gère les erreurs I²C avec réinitialisation automatique et backoff exponentiel
//! - Ne fait jamais `panic!` — toute erreur est absorbée et signalée via les canaux
//!
//! # Architecture des canaux
//!
//! ```text
//! tache_bmp280   ──watch──→  rx_baro   ──┐
//! tache_vl53l0x  ──watch──→  rx_telem  ──┼──→ capteurs/ (fusion) → estimation/ (Kalman)
//! tache_mpu9250  ──watch──→  rx_imu    ──┘
//! ```
//!
//! # Utilisation
//!
//! ```ignore
//! let capteurs = lancer_capteurs().await;
//! let derniere_imu = capteurs.rx_imu.borrow().clone();
//! ```

use std::sync::Arc;
use std::sync::atomic::{AtomicU32, Ordering};
use tokio::sync::watch;
use tokio::task::JoinHandle;

use crate::types::{DonneesBarometre, DonneesImu};

// ─────────────────────────────────────────────────────────────────────────────
// Paramètres de robustesse
// ─────────────────────────────────────────────────────────────────────────────

/// Nombre d'erreurs consécutives avant de tenter une réinitialisation du capteur
const ERREURS_AVANT_REINIT: u32 = 5;

/// Nombre de réinitialisations consécutives avant suspension longue de la tâche
const REINIT_MAX: u32 = 10;

/// Délai initial du backoff après une erreur (ms)
const BACKOFF_INITIAL_MS: u64 = 100;

/// Facteur multiplicatif du backoff exponentiel
const BACKOFF_FACTEUR: u64 = 2;

/// Délai maximum du backoff — borne supérieure pour rester réactif
const BACKOFF_MAX_MS: u64 = 5_000;

// ─────────────────────────────────────────────────────────────────────────────
// Types publiés sur les canaux watch
// ─────────────────────────────────────────────────────────────────────────────

/// Mesure baromètre avec métadonnées de fiabilité
///
/// Le champ `valide` permet aux consommateurs (fusion, Kalman) de savoir
/// si la valeur est exploitable sans avoir à tester `donnees.is_some()`.
#[derive(Debug, Clone)]
pub struct MesureBaro {
    pub donnees: Option<DonneesBarometre>,
    /// `true` si la dernière lecture a réussi
    pub valide: bool,
    /// Nombre d'erreurs I²C consécutives en cours
    pub erreurs_consecutives: u32,
}

/// Mesure télémètre avec métadonnées de fiabilité
#[derive(Debug, Clone)]
pub struct MesureTelem {
    /// Distance en mm, `None` si hors portée ou capteur en erreur
    pub distance_mm: Option<u16>,
    pub valide: bool,
    pub erreurs_consecutives: u32,
}

/// Mesure IMU avec métadonnées de fiabilité
#[derive(Debug, Clone)]
pub struct MesureImu {
    pub donnees: Option<DonneesImu>,
    pub valide: bool,
    pub erreurs_consecutives: u32,
}

// ─────────────────────────────────────────────────────────────────────────────
// Compteurs de santé partagés
// ─────────────────────────────────────────────────────────────────────────────

/// Compteurs atomiques de santé — lisibles depuis n'importe quelle tâche
///
/// Destinés à la couche `surete/` (watchdog, sante.rs) une fois implémentée.
#[derive(Debug)]
pub struct SanteCapteurs {
    pub erreurs_baro:   Arc<AtomicU32>,
    pub erreurs_telem:  Arc<AtomicU32>,
    pub erreurs_imu:    Arc<AtomicU32>,
    pub reinit_baro:    Arc<AtomicU32>,
    pub reinit_telem:   Arc<AtomicU32>,
    pub reinit_imu:     Arc<AtomicU32>,
}

impl SanteCapteurs {
    fn nouveau() -> Self {
        Self {
            erreurs_baro:  Arc::new(AtomicU32::new(0)),
            erreurs_telem: Arc::new(AtomicU32::new(0)),
            erreurs_imu:   Arc::new(AtomicU32::new(0)),
            reinit_baro:   Arc::new(AtomicU32::new(0)),
            reinit_telem:  Arc::new(AtomicU32::new(0)),
            reinit_imu:    Arc::new(AtomicU32::new(0)),
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Handle retourné par lancer_capteurs()
// ─────────────────────────────────────────────────────────────────────────────

/// Tout ce dont le reste du système a besoin pour consommer les mesures capteurs
pub struct HandlesCapteurs {
    /// Dernière mesure baromètre disponible (non-bloquant via `.borrow()`)
    pub rx_baro:  watch::Receiver<MesureBaro>,
    /// Dernière mesure télémètre disponible
    pub rx_telem: watch::Receiver<MesureTelem>,
    /// Dernière mesure IMU disponible
    pub rx_imu:   watch::Receiver<MesureImu>,
    /// Compteurs de santé pour la supervision (surete/)
    pub sante:    Arc<SanteCapteurs>,
    /// Handles des tâches Tokio (pour abort propre à l'arrêt)
    pub taches:   Vec<JoinHandle<()>>,
}

// ─────────────────────────────────────────────────────────────────────────────
// Point d'entrée
// ─────────────────────────────────────────────────────────────────────────────

/// Lance les trois tâches capteurs en parallèle
///
/// À appeler une seule fois au démarrage du système, que ce soit en vol
/// (`airhaum-vol.rs`) ou depuis un test d'intégration.
///
/// Chaque tâche ouvre son propre descripteur `/dev/i2c-0`. Le noyau Linux
/// sérialise les accès physiques au bus — il n'y a pas de conflit.
pub async fn lancer_capteurs() -> HandlesCapteurs {
    let sante = Arc::new(SanteCapteurs::nouveau());

    let (tx_baro, rx_baro) = watch::channel(MesureBaro {
        donnees: None, valide: false, erreurs_consecutives: 0,
    });
    let (tx_telem, rx_telem) = watch::channel(MesureTelem {
        distance_mm: None, valide: false, erreurs_consecutives: 0,
    });
    let (tx_imu, rx_imu) = watch::channel(MesureImu {
        donnees: None, valide: false, erreurs_consecutives: 0,
    });

    let mut taches = Vec::new();

    taches.push(tokio::spawn({
        let cpt_err = Arc::clone(&sante.erreurs_baro);
        let cpt_reinit = Arc::clone(&sante.reinit_baro);
        async move { tache_bmp280(tx_baro, cpt_err, cpt_reinit).await; }
    }));

    taches.push(tokio::spawn({
        let cpt_err = Arc::clone(&sante.erreurs_telem);
        let cpt_reinit = Arc::clone(&sante.reinit_telem);
        async move { tache_vl53l0x(tx_telem, cpt_err, cpt_reinit).await; }
    }));

    taches.push(tokio::spawn({
        let cpt_err = Arc::clone(&sante.erreurs_imu);
        let cpt_reinit = Arc::clone(&sante.reinit_imu);
        async move { tache_mpu9250(tx_imu, cpt_err, cpt_reinit).await; }
    }));

    HandlesCapteurs { rx_baro, rx_telem, rx_imu, sante, taches }
}

// ─────────────────────────────────────────────────────────────────────────────
// Macro interne : boucle de résilience commune aux 3 tâches
// ─────────────────────────────────────────────────────────────────────────────
//
// Les trois tâches ont la même structure :
//   loop {
//     1. Initialiser le capteur (avec backoff si échec)
//     2. Boucle de lecture (break → retour à 1 si trop d'erreurs)
//   }
//
// Les différences sont : le type du capteur, la fonction de lecture,
// la valeur publiée en cas d'erreur. On les implémente séparément
// pour garder le code lisible et typé.

// ─────────────────────────────────────────────────────────────────────────────
// Tâche BMP280
// ─────────────────────────────────────────────────────────────────────────────

async fn tache_bmp280(
    tx: watch::Sender<MesureBaro>,
    cpt_err: Arc<AtomicU32>,
    cpt_reinit: Arc<AtomicU32>,
) {
    let mut erreurs_consecutives = 0u32;
    let mut nb_reinit = 0u32;
    let mut backoff_ms = BACKOFF_INITIAL_MS;

    loop {
        let init_result = init_bmp280().await;

        let mut capteur = match init_result {
            Ok(c) => {
                backoff_ms = BACKOFF_INITIAL_MS;
                nb_reinit = 0;
                c
            }
            Err(_) => {
                nb_reinit += 1;
                cpt_reinit.fetch_add(1, Ordering::Relaxed);
                let _ = tx.send(MesureBaro { donnees: None, valide: false, erreurs_consecutives: nb_reinit });
                if nb_reinit >= REINIT_MAX {
                    eprintln!("[BMP280] ❌ {} réinitialisations sans succès — suspension 30s", nb_reinit);
                    tokio::time::sleep(tokio::time::Duration::from_secs(30)).await;
                    nb_reinit = 0;
                    backoff_ms = BACKOFF_INITIAL_MS;
                } else {
                    eprintln!("[BMP280] Init échouée, nouvelle tentative dans {}ms", backoff_ms);
                    tokio::time::sleep(tokio::time::Duration::from_millis(backoff_ms)).await;
                    backoff_ms = (backoff_ms * BACKOFF_FACTEUR).min(BACKOFF_MAX_MS);
                }
                continue;
            }
        };

        // Boucle de lecture
        use crate::interfaces::barometre::Barometre;
        loop {
            match capteur.lire() {
                Ok(donnees) => {
                    erreurs_consecutives = 0;
                    backoff_ms = BACKOFF_INITIAL_MS;
                    let _ = tx.send(MesureBaro {
                        donnees: Some(donnees),
                        valide: true,
                        erreurs_consecutives: 0,
                    });
                }
                Err(e) => {
                    erreurs_consecutives += 1;
                    cpt_err.fetch_add(1, Ordering::Relaxed);
                    let _ = tx.send(MesureBaro {
                        donnees: None,
                        valide: false,
                        erreurs_consecutives,
                    });
                    if erreurs_consecutives >= ERREURS_AVANT_REINIT {
                        eprintln!("[BMP280] {} erreurs consécutives — réinitialisation ({:?})",
                                  erreurs_consecutives, e);
                        tokio::time::sleep(tokio::time::Duration::from_millis(backoff_ms)).await;
                        backoff_ms = (backoff_ms * BACKOFF_FACTEUR).min(BACKOFF_MAX_MS);
                        cpt_reinit.fetch_add(1, Ordering::Relaxed);
                        erreurs_consecutives = 0;
                        break; // → retour boucle d'init
                    } else {
                        tokio::time::sleep(tokio::time::Duration::from_millis(50)).await;
                    }
                }
            }
            tokio::task::yield_now().await;
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Tâche VL53L0X
// ─────────────────────────────────────────────────────────────────────────────

async fn tache_vl53l0x(
    tx: watch::Sender<MesureTelem>,
    cpt_err: Arc<AtomicU32>,
    cpt_reinit: Arc<AtomicU32>,
) {
    let mut erreurs_consecutives = 0u32;
    let mut nb_reinit = 0u32;
    let mut backoff_ms = BACKOFF_INITIAL_MS;

    loop {
        let init_result = init_vl53l0x().await;

        let mut capteur = match init_result {
            Ok(c) => { backoff_ms = BACKOFF_INITIAL_MS; nb_reinit = 0; c }
            Err(_) => {
                nb_reinit += 1;
                cpt_reinit.fetch_add(1, Ordering::Relaxed);
                let _ = tx.send(MesureTelem { distance_mm: None, valide: false, erreurs_consecutives: nb_reinit });
                if nb_reinit >= REINIT_MAX {
                    eprintln!("[VL53L0X] ❌ {} réinitialisations sans succès — suspension 30s", nb_reinit);
                    tokio::time::sleep(tokio::time::Duration::from_secs(30)).await;
                    nb_reinit = 0;
                    backoff_ms = BACKOFF_INITIAL_MS;
                } else {
                    eprintln!("[VL53L0X] Init échouée, nouvelle tentative dans {}ms", backoff_ms);
                    tokio::time::sleep(tokio::time::Duration::from_millis(backoff_ms)).await;
                    backoff_ms = (backoff_ms * BACKOFF_FACTEUR).min(BACKOFF_MAX_MS);
                }
                continue;
            }
        };

        use crate::interfaces::telemetre::Telemetre;
        loop {
            match capteur.mesurer_distance() {
                Ok(dist) => {
                    erreurs_consecutives = 0;
                    backoff_ms = BACKOFF_INITIAL_MS;
                    // Publier None si hors portée plutôt qu'une valeur mensongère
                    let dist_valide = if dist < capteur.obtenir_portee_max() {
                        Some(dist)
                    } else {
                        None
                    };
                    let _ = tx.send(MesureTelem {
                        distance_mm: dist_valide,
                        valide: dist_valide.is_some(),
                        erreurs_consecutives: 0,
                    });
                }
                Err(e) => {
                    erreurs_consecutives += 1;
                    cpt_err.fetch_add(1, Ordering::Relaxed);
                    let _ = tx.send(MesureTelem { distance_mm: None, valide: false, erreurs_consecutives });
                    if erreurs_consecutives >= ERREURS_AVANT_REINIT {
                        eprintln!("[VL53L0X] {} erreurs consécutives — réinitialisation ({:?})",
                                  erreurs_consecutives, e);
                        tokio::time::sleep(tokio::time::Duration::from_millis(backoff_ms)).await;
                        backoff_ms = (backoff_ms * BACKOFF_FACTEUR).min(BACKOFF_MAX_MS);
                        cpt_reinit.fetch_add(1, Ordering::Relaxed);
                        erreurs_consecutives = 0;
                        break;
                    } else {
                        tokio::time::sleep(tokio::time::Duration::from_millis(50)).await;
                    }
                }
            }
            tokio::task::yield_now().await;
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Tâche MPU9250
// ─────────────────────────────────────────────────────────────────────────────

async fn tache_mpu9250(
    tx: watch::Sender<MesureImu>,
    cpt_err: Arc<AtomicU32>,
    cpt_reinit: Arc<AtomicU32>,
) {
    let mut erreurs_consecutives = 0u32;
    let mut nb_reinit = 0u32;
    let mut backoff_ms = BACKOFF_INITIAL_MS;

    loop {
        let init_result = init_mpu9250().await;

        let mut capteur = match init_result {
            Ok(c) => { backoff_ms = BACKOFF_INITIAL_MS; nb_reinit = 0; c }
            Err(_) => {
                nb_reinit += 1;
                cpt_reinit.fetch_add(1, Ordering::Relaxed);
                let _ = tx.send(MesureImu { donnees: None, valide: false, erreurs_consecutives: nb_reinit });
                if nb_reinit >= REINIT_MAX {
                    eprintln!("[MPU9250] ❌ {} réinitialisations sans succès — suspension 30s", nb_reinit);
                    tokio::time::sleep(tokio::time::Duration::from_secs(30)).await;
                    nb_reinit = 0;
                    backoff_ms = BACKOFF_INITIAL_MS;
                } else {
                    eprintln!("[MPU9250] Init échouée, nouvelle tentative dans {}ms", backoff_ms);
                    tokio::time::sleep(tokio::time::Duration::from_millis(backoff_ms)).await;
                    backoff_ms = (backoff_ms * BACKOFF_FACTEUR).min(BACKOFF_MAX_MS);
                }
                continue;
            }
        };

        use crate::interfaces::imu::CentraleInertielle;
        loop {
            match capteur.lire() {
                Ok(donnees) => {
                    erreurs_consecutives = 0;
                    backoff_ms = BACKOFF_INITIAL_MS;
                    let _ = tx.send(MesureImu {
                        donnees: Some(donnees),
                        valide: true,
                        erreurs_consecutives: 0,
                    });
                }
                Err(e) => {
                    erreurs_consecutives += 1;
                    cpt_err.fetch_add(1, Ordering::Relaxed);
                    let _ = tx.send(MesureImu { donnees: None, valide: false, erreurs_consecutives });
                    if erreurs_consecutives >= ERREURS_AVANT_REINIT {
                        eprintln!("[MPU9250] {} erreurs consécutives — réinitialisation ({:?})",
                                  erreurs_consecutives, e);
                        tokio::time::sleep(tokio::time::Duration::from_millis(backoff_ms)).await;
                        backoff_ms = (backoff_ms * BACKOFF_FACTEUR).min(BACKOFF_MAX_MS);
                        cpt_reinit.fetch_add(1, Ordering::Relaxed);
                        erreurs_consecutives = 0;
                        break;
                    } else {
                        tokio::time::sleep(tokio::time::Duration::from_millis(10)).await;
                    }
                }
            }
            tokio::task::yield_now().await;
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Fonctions d'initialisation (séparées pour clarté et testabilité)
// ─────────────────────────────────────────────────────────────────────────────

#[cfg(target_os = "linux")]
async fn init_bmp280() -> Result<
    crate::drivers::barometre::Bmp280<crate::hal::i2c_linux::I2cLinux>,
    ()
> {
    use crate::hal::i2c_linux::I2cLinux;
    use crate::drivers::barometre::Bmp280;
    use crate::interfaces::barometre::Barometre;

    let i2c = I2cLinux::nouveau(0).map_err(|e| {
        eprintln!("[BMP280] Impossible d'ouvrir I²C: {:?}", e);
    })?;
    let mut bmp = Bmp280::nouveau(i2c);
    bmp.initialiser().map_err(|e| {
        eprintln!("[BMP280] Échec initialisation: {:?}", e);
    })?;
    Ok(bmp)
}

#[cfg(not(target_os = "linux"))]
async fn init_bmp280() -> Result<
    crate::drivers::barometre::Bmp280<crate::hal::i2c::I2cMock>,
    ()
> {
    use crate::hal::i2c::I2cMock;
    use crate::drivers::barometre::Bmp280;
    use crate::interfaces::barometre::Barometre;

    let i2c = I2cMock::nouveau();
    let mut bmp = Bmp280::nouveau(i2c);
    bmp.initialiser().map_err(|_| ())?;
    Ok(bmp)
}

#[cfg(target_os = "linux")]
async fn init_vl53l0x() -> Result<
    crate::drivers::telemetre::Vl53l0x<crate::hal::i2c_linux::I2cLinux>,
    ()
> {
    use crate::hal::i2c_linux::I2cLinux;
    use crate::drivers::telemetre::Vl53l0x;
    use crate::drivers::telemetre::vl53l0x::ADRESSE_VL53L0X;
    use crate::interfaces::telemetre::Telemetre;

    let i2c = I2cLinux::nouveau(0).map_err(|e| {
        eprintln!("[VL53L0X] Impossible d'ouvrir I²C: {:?}", e);
    })?;
    let mut vl53 = Vl53l0x::nouveau(i2c, ADRESSE_VL53L0X);
    vl53.initialiser().map_err(|e| {
        eprintln!("[VL53L0X] Échec initialisation: {:?}", e);
    })?;
    Ok(vl53)
}

#[cfg(not(target_os = "linux"))]
async fn init_vl53l0x() -> Result<
    crate::drivers::telemetre::Vl53l0x<crate::hal::i2c::I2cMock>,
    ()
> {
    use crate::hal::i2c::I2cMock;
    use crate::drivers::telemetre::Vl53l0x;
    use crate::drivers::telemetre::vl53l0x::ADRESSE_VL53L0X;
    use crate::interfaces::telemetre::Telemetre;

    let i2c = I2cMock::nouveau();
    let mut vl53 = Vl53l0x::nouveau(i2c, ADRESSE_VL53L0X);
    vl53.initialiser().map_err(|_| ())?;
    Ok(vl53)
}

#[cfg(target_os = "linux")]
async fn init_mpu9250() -> Result<
    crate::drivers::imu::Mpu9250<crate::hal::i2c_linux::I2cLinux>,
    ()
> {
    use crate::hal::i2c_linux::I2cLinux;
    use crate::drivers::imu::Mpu9250;
    use crate::drivers::imu::ADRESSE_MPU9250;
    use crate::interfaces::imu::CentraleInertielle;

    let i2c = I2cLinux::nouveau(0).map_err(|e| {
        eprintln!("[MPU9250] Impossible d'ouvrir I²C: {:?}", e);
    })?;
    let mut mpu = Mpu9250::nouveau(i2c, ADRESSE_MPU9250);
    mpu.initialiser().map_err(|e| {
        eprintln!("[MPU9250] Échec initialisation: {:?}", e);
    })?;
    Ok(mpu)
}

#[cfg(not(target_os = "linux"))]
async fn init_mpu9250() -> Result<
    crate::drivers::imu::Mpu9250<crate::hal::i2c::I2cMock>,
    ()
> {
    use crate::hal::i2c::I2cMock;
    use crate::drivers::imu::Mpu9250;
    use crate::drivers::imu::ADRESSE_MPU9250;
    use crate::interfaces::imu::CentraleInertielle;

    let i2c = I2cMock::nouveau();
    let mut mpu = Mpu9250::nouveau(i2c, ADRESSE_MPU9250);
    mpu.initialiser().map_err(|_| ())?;
    Ok(mpu)
}
