// src/taches/taches_capteurs.rs
//! Threads de lecture continue des capteurs
//!
//! Ce module instancie et pilote les drivers capteurs dans des `std::thread`
//! indépendants. Chaque thread :
//!
//! - Reçoit une référence partagée [`BusPartage`] vers le bus I²C commun
//! - Initialise son capteur au démarrage
//! - Lit en continu et publie les mesures via des canaux
//! - Gère les erreurs I²C avec réinitialisation automatique et backoff exponentiel
//! - Ne fait jamais `panic!` — toute erreur est absorbée et signalée via les canaux
//!
//! # Architecture du bus partagé
//!
//! ```text
//!                    Arc<std::sync::Mutex<I2cLinux>>
//!                    ┌──────────────────┐
//!  thread_bmp280 ───▶│                  │
//!  thread_vl53l0x───▶│   bus I²C (fd)  │──▶ /dev/i2c-1
//!  thread_mpu9250───▶│                  │
//!                    └──────────────────┘
//! ```
//!
//! Le mutex est bloquant. Le timeout kernel `I2C_TIMEOUT` (~10 ms) garantit
//! que le verrou est toujours libéré rapidement même en cas de capteur défaillant.
//!
//! # Architecture des canaux
//!
//! ```text
//! thread_bmp280  ──valeur courante──▶  rx_baro   → Kalman (correction)
//! thread_vl53l0x ──valeur courante──▶  rx_telem  → Kalman (correction)
//! thread_mpu9250 ──FIFO borné     ──▶  rx_imu    → Kalman (prédiction)
//! ```
//!
//! Le canal IMU est un FIFO borné (`mpsc`, [`FIFO_IMU_CAPACITE`] slots) :
//! toutes les mesures sont transmises dans l'ordre avec leurs horodatages,
//! indispensable pour l'intégration gyroscope du filtre de Kalman.
//! Les canaux baro/télémètre sont à valeur courante (`watch`) : seule la
//! dernière mesure compte pour les corrections Kalman.

use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use std::time::Duration;
use tokio::sync::{watch, mpsc};

use crate::hal::BusPartage;
use crate::types::{DonneesBarometre, DonneesImu, Result};

// ─────────────────────────────────────────────────────────────────────────────
// Paramètres de robustesse
// ─────────────────────────────────────────────────────────────────────────────

/// Nombre d'erreurs consécutives avant de tenter une réinitialisation du capteur.
const ERREURS_AVANT_REINIT: u32 = 5;

/// Nombre de réinitialisations consécutives avant suspension longue du thread.
const REINIT_MAX: u32 = 10;

/// Délai initial du backoff après une erreur (ms).
const BACKOFF_INITIAL_MS: u64 = 100;

/// Facteur multiplicatif du backoff exponentiel.
const BACKOFF_FACTEUR: u64 = 2;

/// Délai maximum du backoff — borne supérieure pour rester réactif.
const BACKOFF_MAX_MS: u64 = 5_000;

/// Cadence de lecture du BMP280 en mode normal (~43 Hz, cycle mesure ~23 ms).
const PERIODE_BARO_MS: u64 = 23;

/// Cadence de lecture de l'IMU (200 Hz → 5 ms).
const PERIODE_IMU_MS: u64 = 5;

/// Capacité du FIFO IMU : 20 mesures × 5 ms = 100 ms de buffer.
///
/// Si le consommateur (Kalman) a plus de 100 ms de retard, les mesures
/// excédentaires sont abandonnées avec un avertissement. Le thread IMU
/// n'est jamais bloqué par un consommateur lent.
pub const FIFO_IMU_CAPACITE: usize = 20;

// ─────────────────────────────────────────────────────────────────────────────
// Types publiés sur les canaux
// ─────────────────────────────────────────────────────────────────────────────

/// Mesure baromètre avec métadonnées de fiabilité.
#[derive(Debug, Clone)]
pub struct MesureBaro {
    pub donnees: Option<DonneesBarometre>,
    pub valide: bool,
    pub erreurs_consecutives: u32,
}

/// Mesure télémètre avec métadonnées de fiabilité.
#[derive(Debug, Clone)]
pub struct MesureTelem {
    pub distance_mm: Option<u16>,
    pub valide: bool,
    pub erreurs_consecutives: u32,
}

/// Mesure IMU avec métadonnées de fiabilité.
#[derive(Debug, Clone)]
pub struct MesureImu {
    pub donnees: Option<DonneesImu>,
    pub valide: bool,
    pub erreurs_consecutives: u32,
}

// ─────────────────────────────────────────────────────────────────────────────
// Compteurs de santé partagés
// ─────────────────────────────────────────────────────────────────────────────

/// Compteurs atomiques de santé — lisibles depuis n'importe quel thread ou tâche.
#[derive(Debug)]
pub struct SanteCapteurs {
    pub erreurs_baro:  Arc<AtomicU32>,
    pub erreurs_telem: Arc<AtomicU32>,
    pub erreurs_imu:   Arc<AtomicU32>,
    pub reinit_baro:   Arc<AtomicU32>,
    pub reinit_telem:  Arc<AtomicU32>,
    pub reinit_imu:    Arc<AtomicU32>,
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

/// Tout ce dont le reste du système a besoin pour consommer les mesures capteurs.
pub struct HandlesCapteurs {
    /// Dernière mesure baromètre disponible (canal à valeur courante).
    pub rx_baro:  watch::Receiver<MesureBaro>,
    /// Dernière mesure télémètre disponible (canal à valeur courante).
    pub rx_telem: watch::Receiver<MesureTelem>,
    /// File de mesures IMU horodatées (FIFO borné, [`FIFO_IMU_CAPACITE`] slots).
    ///
    /// `Some` à la création. Pris par `prendre_rx_imu()` au lancement du thread
    /// d'estimation — après quoi cette valeur est `None`.
    pub rx_imu:   Option<mpsc::Receiver<MesureImu>>,
    /// Compteurs de santé pour la supervision (`surete/`).
    pub sante:    Arc<SanteCapteurs>,
    /// Handles des threads capteurs (pour jointure à l'arrêt).
    pub taches:   Vec<std::thread::JoinHandle<()>>,
    /// Signal d'arrêt partagé avec les threads.
    arret:        Arc<AtomicBool>,
}

impl HandlesCapteurs {
    /// Signale l'arrêt à tous les threads capteurs.
    ///
    /// Les threads terminent leur itération courante puis s'arrêtent.
    /// Non-bloquant : retourne immédiatement sans attendre la fin des threads.
    pub fn arreter(&self) {
        self.arret.store(true, Ordering::Relaxed);
    }

    /// Transfère la propriété du récepteur IMU au thread d'estimation.
    ///
    /// Appelé une seule fois au démarrage. Après cet appel, `rx_imu` vaut `None`
    /// et l'accès direct aux mesures brutes n'est plus possible — seul le thread
    /// d'estimation consomme le FIFO.
    ///
    /// # Panics
    /// Panique si appelé une seconde fois (le récepteur a déjà été transféré).
    pub fn prendre_rx_imu(&mut self) -> mpsc::Receiver<MesureImu> {
        self.rx_imu.take()
            .expect("[HandlesCapteurs] rx_imu déjà transféré au thread d'estimation")
    }
}

/// Tente de détecter une reprise rapide en sondant les capteurs sur le bus
/// déjà ouvert, avant le lancement des threads.
///
/// Une reprise rapide est pertinente lorsque l'appareil est **en vol** et
/// que le logiciel vient de redémarrer (panique, watchdog, coupure brève).
/// Dans ce cas, les capteurs sont déjà configurés et actifs : on peut court-
/// circuiter toute la séquence sol et reprendre les commandes immédiatement.
///
/// Le bus passé en paramètre est le même qui sera ensuite utilisé par les
/// threads capteurs — il n'est pas rouvert. La fonction l'emprunte le temps
/// de lire quelques registres, puis les threads prennent le relais.
///
/// # À implémenter
///
/// Lire les registres de configuration de chaque capteur via `bus` :
/// - **BMP280** : `CTRL_MEAS` doit valoir `OSRS_T_X2 | OSRS_P_X8 | MODE_NORMAL`
/// - **MPU9250** : `PWR_MGMT_1` doit valoir `0x01` (PLL actif, non endormi)
/// - **VL53L0X** : vérifier que le mode continu est actif
///
/// En parallèle, l'estimation d'état (altitude, vitesse) pourra confirmer
/// que l'appareil est effectivement en vol avant de valider la reprise.
///
/// Retourne `true` si tous les capteurs sont déjà opérationnels et que les
/// conditions de vol sont détectées.
pub fn detecter_reprise_rapide<B: crate::hal::BusI2c + Send + 'static>(
    _bus: &BusPartage<B>,
) -> bool {
    // TODO : sonder les registres de configuration de chaque capteur
    // et vérifier les conditions de vol (altitude > seuil, vitesse > seuil).
    false
}

// ─────────────────────────────────────────────────────────────────────────────
// Point d'entrée
// ─────────────────────────────────────────────────────────────────────────────

/// Lance les trois threads capteurs en parallèle.
///
/// Ouvre le bus I²C **une seule fois** et le partage entre les threads via
/// [`BusPartage`] (`Arc<std::sync::Mutex<impl BusI2c>>`).
///
/// À appeler une seule fois au démarrage du système.
///
/// # Erreurs
/// Retourne une erreur si le bus I²C ne peut pas être ouvert.
/// Les erreurs survenant après le démarrage sont gérées en interne
/// par chaque thread (backoff + réinitialisation).
#[cfg(target_os = "linux")]
pub fn lancer_capteurs() -> Result<HandlesCapteurs> {
    use crate::hal::i2c_linux::I2cLinux;

    let bus: BusPartage<I2cLinux> = Arc::new(std::sync::Mutex::new(
        I2cLinux::nouveau(0).map_err(|e| {
            log::error!(target: "systeme", "Impossible d'ouvrir le bus I²C : {:?}", e);
            e
        })?
    ));

    Ok(lancer_avec_bus(bus))
}

/// Variante hors-Linux utilisant le mock I²C (développement / CI).
#[cfg(not(target_os = "linux"))]
pub fn lancer_capteurs() -> Result<HandlesCapteurs> {
    use crate::hal::i2c::I2cMock;

    let bus: BusPartage<I2cMock> = Arc::new(std::sync::Mutex::new(I2cMock::nouveau()));
    Ok(lancer_avec_bus(bus))
}

/// Implémentation commune — indépendante de l'implémentation du bus.
///
/// Séparée de `lancer_capteurs` pour permettre les tests d'intégration avec
/// un bus mock injecté explicitement.
pub fn lancer_avec_bus<B>(bus: BusPartage<B>) -> HandlesCapteurs
where
    B: crate::hal::BusI2c + Send + 'static,
{
    let sante = Arc::new(SanteCapteurs::nouveau());
    let arret = Arc::new(AtomicBool::new(false));

    let (tx_baro, rx_baro) = watch::channel(MesureBaro {
        donnees: None, valide: false, erreurs_consecutives: 0,
    });
    let (tx_telem, rx_telem) = watch::channel(MesureTelem {
        distance_mm: None, valide: false, erreurs_consecutives: 0,
    });
    let (tx_imu, rx_imu) = mpsc::channel(FIFO_IMU_CAPACITE);

    let mut taches = Vec::new();

    taches.push(std::thread::Builder::new()
        .name("capteur-bmp280".into())
        .spawn({
            let bus    = Arc::clone(&bus);
            let err    = Arc::clone(&sante.erreurs_baro);
            let reinit = Arc::clone(&sante.reinit_baro);
            let arret  = Arc::clone(&arret);
            move || thread_bmp280(bus, tx_baro, err, reinit, arret)
        })
        .expect("Impossible de créer le thread capteur-bmp280"));

    taches.push(std::thread::Builder::new()
        .name("capteur-vl53l0x".into())
        .spawn({
            let bus    = Arc::clone(&bus);
            let err    = Arc::clone(&sante.erreurs_telem);
            let reinit = Arc::clone(&sante.reinit_telem);
            let arret  = Arc::clone(&arret);
            move || thread_vl53l0x(bus, tx_telem, err, reinit, arret)
        })
        .expect("Impossible de créer le thread capteur-vl53l0x"));

    taches.push(std::thread::Builder::new()
        .name("capteur-mpu9250".into())
        .spawn({
            let bus    = Arc::clone(&bus);
            let err    = Arc::clone(&sante.erreurs_imu);
            let reinit = Arc::clone(&sante.reinit_imu);
            let arret  = Arc::clone(&arret);
            move || thread_mpu9250(bus, tx_imu, err, reinit, arret)
        })
        .expect("Impossible de créer le thread capteur-mpu9250"));

    HandlesCapteurs { rx_baro, rx_telem, rx_imu: Some(rx_imu), sante, taches, arret }
}

// ─────────────────────────────────────────────────────────────────────────────
// Thread BMP280
// ─────────────────────────────────────────────────────────────────────────────

fn thread_bmp280<B: crate::hal::BusI2c + 'static>(
    bus:    BusPartage<B>,
    tx:     watch::Sender<MesureBaro>,
    cpt_err:    Arc<AtomicU32>,
    cpt_reinit: Arc<AtomicU32>,
    arret:  Arc<AtomicBool>,
) {
    let mut erreurs_consecutives = 0u32;
    let mut nb_reinit  = 0u32;
    let mut backoff_ms = BACKOFF_INITIAL_MS;

    loop {
        if arret.load(Ordering::Relaxed) { break; }

        let mut capteur = match init_bmp280(&bus) {
            Ok(c)  => { backoff_ms = BACKOFF_INITIAL_MS; nb_reinit = 0; c }
            Err(_) => {
                nb_reinit += 1;
                cpt_reinit.fetch_add(1, Ordering::Relaxed);
                let _ = tx.send(MesureBaro { donnees: None, valide: false, erreurs_consecutives: nb_reinit });
                if nb_reinit >= REINIT_MAX {
                    log::error!(target: "baro", "BMP280 : {} réinitialisations sans succès — suspension 30s", nb_reinit);
                    std::thread::sleep(Duration::from_secs(30));
                    nb_reinit = 0;
                    backoff_ms = BACKOFF_INITIAL_MS;
                } else {
                    log::warn!(target: "baro", "BMP280 : init échouée — nouvelle tentative dans {}ms", backoff_ms);
                    std::thread::sleep(Duration::from_millis(backoff_ms));
                    backoff_ms = (backoff_ms * BACKOFF_FACTEUR).min(BACKOFF_MAX_MS);
                }
                continue;
            }
        };

        use crate::interfaces::barometre::Barometre;
        loop {
            if arret.load(Ordering::Relaxed) { return; }

            match capteur.lire() {
                Ok(donnees) => {
                    erreurs_consecutives = 0;
                    backoff_ms = BACKOFF_INITIAL_MS;
                    let _ = tx.send(MesureBaro {
                        donnees: Some(donnees), valide: true, erreurs_consecutives: 0,
                    });
                    // Pacing : BMP280 en mode normal produit une mesure toutes les ~23 ms
                    std::thread::sleep(Duration::from_millis(PERIODE_BARO_MS));
                }
                Err(e) => {
                    erreurs_consecutives += 1;
                    cpt_err.fetch_add(1, Ordering::Relaxed);
                    let _ = tx.send(MesureBaro { donnees: None, valide: false, erreurs_consecutives });
                    if erreurs_consecutives >= ERREURS_AVANT_REINIT {
                        log::warn!(target: "baro", "BMP280 : {} erreurs consécutives — réinitialisation ({:?})",
                                   erreurs_consecutives, e);
                        std::thread::sleep(Duration::from_millis(backoff_ms));
                        backoff_ms = (backoff_ms * BACKOFF_FACTEUR).min(BACKOFF_MAX_MS);
                        cpt_reinit.fetch_add(1, Ordering::Relaxed);
                        erreurs_consecutives = 0;
                        break; // retour boucle externe → réinitialisation
                    } else {
                        std::thread::sleep(Duration::from_millis(50));
                    }
                }
            }
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Thread VL53L0X
// ─────────────────────────────────────────────────────────────────────────────

fn thread_vl53l0x<B: crate::hal::BusI2c + 'static>(
    bus:    BusPartage<B>,
    tx:     watch::Sender<MesureTelem>,
    cpt_err:    Arc<AtomicU32>,
    cpt_reinit: Arc<AtomicU32>,
    arret:  Arc<AtomicBool>,
) {
    let mut erreurs_consecutives = 0u32;
    let mut nb_reinit  = 0u32;
    let mut backoff_ms = BACKOFF_INITIAL_MS;

    loop {
        if arret.load(Ordering::Relaxed) { break; }

        let mut capteur = match init_vl53l0x(&bus) {
            Ok(c)  => { backoff_ms = BACKOFF_INITIAL_MS; nb_reinit = 0; c }
            Err(_) => {
                nb_reinit += 1;
                cpt_reinit.fetch_add(1, Ordering::Relaxed);
                let _ = tx.send(MesureTelem { distance_mm: None, valide: false, erreurs_consecutives: nb_reinit });
                if nb_reinit >= REINIT_MAX {
                    log::error!(target: "telem", "VL53L0X : {} réinitialisations sans succès — suspension 30s", nb_reinit);
                    std::thread::sleep(Duration::from_secs(30));
                    nb_reinit = 0;
                    backoff_ms = BACKOFF_INITIAL_MS;
                } else {
                    log::warn!(target: "telem", "VL53L0X : init échouée — nouvelle tentative dans {}ms", backoff_ms);
                    std::thread::sleep(Duration::from_millis(backoff_ms));
                    backoff_ms = (backoff_ms * BACKOFF_FACTEUR).min(BACKOFF_MAX_MS);
                }
                continue;
            }
        };

        use crate::interfaces::telemetre::Telemetre;
        loop {
            if arret.load(Ordering::Relaxed) { return; }

            match capteur.mesurer_distance() {
                Ok(dist) => {
                    erreurs_consecutives = 0;
                    backoff_ms = BACKOFF_INITIAL_MS;
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
                    // Pas de sleep explicite : le VL53L0X bloque pendant la mesure (~30 ms)
                }
                Err(crate::types::ErreursAirHaum::HorsPortee) => {
                    // Comportement normal : aucun obstacle dans la portée du capteur.
                    // Le matériel fonctionne correctement — on ne compte pas cela
                    // comme une erreur et on ne déclenche pas de réinitialisation.
                    erreurs_consecutives = 0;
                    backoff_ms = BACKOFF_INITIAL_MS;
                    let _ = tx.send(MesureTelem {
                        distance_mm: None,
                        valide: true,
                        erreurs_consecutives: 0,
                    });
                }
                Err(e) => {
                    erreurs_consecutives += 1;
                    cpt_err.fetch_add(1, Ordering::Relaxed);
                    let _ = tx.send(MesureTelem { distance_mm: None, valide: false, erreurs_consecutives });
                    if erreurs_consecutives >= ERREURS_AVANT_REINIT {
                        log::warn!(target: "telem", "VL53L0X : {} erreurs consécutives — réinitialisation ({:?})",
                                   erreurs_consecutives, e);
                        std::thread::sleep(Duration::from_millis(backoff_ms));
                        backoff_ms = (backoff_ms * BACKOFF_FACTEUR).min(BACKOFF_MAX_MS);
                        cpt_reinit.fetch_add(1, Ordering::Relaxed);
                        erreurs_consecutives = 0;
                        break;
                    } else {
                        std::thread::sleep(Duration::from_millis(50));
                    }
                }
            }
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Thread MPU9250
// ─────────────────────────────────────────────────────────────────────────────

fn thread_mpu9250<B: crate::hal::BusI2c + 'static>(
    bus:    BusPartage<B>,
    tx:     mpsc::Sender<MesureImu>,
    cpt_err:    Arc<AtomicU32>,
    cpt_reinit: Arc<AtomicU32>,
    arret:  Arc<AtomicBool>,
) {
    let mut erreurs_consecutives = 0u32;
    let mut nb_reinit  = 0u32;
    let mut backoff_ms = BACKOFF_INITIAL_MS;
    // Cadence les avertissements FIFO plein : au plus un toutes les 5 secondes.
    let mut derniere_alerte_fifo: Option<std::time::Instant> = None;

    loop {
        if arret.load(Ordering::Relaxed) { break; }

        let mut capteur = match init_mpu9250(&bus) {
            Ok(c)  => { backoff_ms = BACKOFF_INITIAL_MS; nb_reinit = 0; c }
            Err(_) => {
                nb_reinit += 1;
                cpt_reinit.fetch_add(1, Ordering::Relaxed);
                // try_send : non-bloquant, abandonne si le FIFO est plein
                let _ = tx.try_send(MesureImu { donnees: None, valide: false, erreurs_consecutives: nb_reinit });
                if nb_reinit >= REINIT_MAX {
                    log::error!(target: "imu", "MPU9250 : {} réinitialisations sans succès — suspension 30s", nb_reinit);
                    std::thread::sleep(Duration::from_secs(30));
                    nb_reinit = 0;
                    backoff_ms = BACKOFF_INITIAL_MS;
                } else {
                    log::warn!(target: "imu", "MPU9250 : init échouée — nouvelle tentative dans {}ms", backoff_ms);
                    std::thread::sleep(Duration::from_millis(backoff_ms));
                    backoff_ms = (backoff_ms * BACKOFF_FACTEUR).min(BACKOFF_MAX_MS);
                }
                continue;
            }
        };

        use crate::interfaces::imu::CentraleInertielle;
        loop {
            if arret.load(Ordering::Relaxed) { return; }

            match capteur.lire() {
                Ok(donnees) => {
                    erreurs_consecutives = 0;
                    backoff_ms = BACKOFF_INITIAL_MS;
                    if tx.try_send(MesureImu {
                        donnees: Some(donnees), valide: true, erreurs_consecutives: 0,
                    }).is_err() {
                        // FIFO plein : on cadence l'avertissement à 1 par 5 secondes
                        // pour ne pas saturer la sortie à 200 Hz.
                        let maintenant = std::time::Instant::now();
                        let afficher = derniere_alerte_fifo
                            .map_or(true, |t| maintenant.duration_since(t) >= Duration::from_secs(5));
                        if afficher {
                            log::warn!(target: "imu", "MPU9250 : FIFO plein — mesure abandonnée (consommateur lent ?)");
                            derniere_alerte_fifo = Some(maintenant);
                        }
                    }
                    std::thread::sleep(Duration::from_millis(PERIODE_IMU_MS));
                }
                Err(e) => {
                    erreurs_consecutives += 1;
                    cpt_err.fetch_add(1, Ordering::Relaxed);
                    let _ = tx.try_send(MesureImu { donnees: None, valide: false, erreurs_consecutives });
                    if erreurs_consecutives >= ERREURS_AVANT_REINIT {
                        log::warn!(target: "imu", "MPU9250 : {} erreurs consécutives — réinitialisation ({:?})",
                                   erreurs_consecutives, e);
                        std::thread::sleep(Duration::from_millis(backoff_ms));
                        backoff_ms = (backoff_ms * BACKOFF_FACTEUR).min(BACKOFF_MAX_MS);
                        cpt_reinit.fetch_add(1, Ordering::Relaxed);
                        erreurs_consecutives = 0;
                        break;
                    } else {
                        std::thread::sleep(Duration::from_millis(10));
                    }
                }
            }
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Fonctions d'initialisation
// ─────────────────────────────────────────────────────────────────────────────

fn init_bmp280<B: crate::hal::BusI2c + 'static>(
    bus: &BusPartage<B>,
) -> std::result::Result<crate::drivers::barometre::Bmp280<BusPartage<B>>, ()>
{
    use crate::drivers::barometre::Bmp280;
    use crate::interfaces::barometre::Barometre;

    let mut bmp = Bmp280::nouveau(Arc::clone(bus));
    bmp.initialiser().map_err(|e| {
        log::error!(target: "baro", "BMP280 échec initialisation : {:?}", e);
    })?;
    Ok(bmp)
}

fn init_vl53l0x<B: crate::hal::BusI2c + 'static>(
    bus: &BusPartage<B>,
) -> std::result::Result<crate::drivers::telemetre::Vl53l0x<BusPartage<B>>, ()>
{
    use crate::drivers::telemetre::Vl53l0x;
    use crate::drivers::telemetre::vl53l0x::ADRESSE_VL53L0X;
    use crate::interfaces::telemetre::Telemetre;

    let mut vl53 = Vl53l0x::nouveau(Arc::clone(bus), ADRESSE_VL53L0X);
    vl53.initialiser().map_err(|e| {
        log::error!(target: "telem", "VL53L0X échec initialisation : {:?}", e);
    })?;
    Ok(vl53)
}

fn init_mpu9250<B: crate::hal::BusI2c + 'static>(
    bus: &BusPartage<B>,
) -> std::result::Result<crate::drivers::imu::Mpu9250<BusPartage<B>>, ()>
{
    use crate::drivers::imu::Mpu9250;
    use crate::drivers::imu::ADRESSE_MPU9250;
    use crate::interfaces::imu::CentraleInertielle;

    let mut mpu = Mpu9250::nouveau(Arc::clone(bus), ADRESSE_MPU9250);
    mpu.initialiser().map_err(|e| {
        log::error!(target: "imu", "MPU9250 échec initialisation : {:?}", e);
    })?;
    Ok(mpu)
}
