// src/taches/taches_gps.rs
//! Thread de lecture continue du GPS (u-blox NEO-M8N sur UART).
//!
//! Architecture identique aux threads I²C (`taches_capteurs`) :
//! backoff exponentiel, réinitialisation automatique, signal d'arrêt atomique.
//!
//! # Canal de sortie
//!
//! Le GPS produit des trames NAV-PVT à ~1–5 Hz selon la configuration du module.
//! Le canal est un `watch` (valeur courante) : seule la dernière position compte
//! pour la navigation et la MAÉ vol.
//!
//! ```text
//! thread_gps ──── MesureGps (watch) ──▶ tache_mission (vitesse sol, position)
//! ```
//!
//! # Séparation des bus
//!
//! Le GPS est sur UART (`/dev/ttyS2`), indépendant du bus I²C partagé par les
//! autres capteurs. Il se lance avec [`lancer_gps`], distinct de `lancer_capteurs`.

use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use std::time::Duration;
use tokio::sync::watch;

use crate::interfaces::gps::CapteurGps;
use crate::types::{DonneesGps, TypeFixGps};

// ─────────────────────────────────────────────────────────────────────────────
// Paramètres de robustesse
// ─────────────────────────────────────────────────────────────────────────────

/// Nombre d'itérations consécutives sans trame avant de tenter une réinit.
/// (Le GPS produit ~1–5 trames/s ; 10 s sans trame = anomalie.)
const SILENCE_AVANT_REINIT: u32 = 50;

/// Nombre de réinitialisations consécutives avant suspension longue.
const REINIT_MAX: u32 = 10;

/// Délai initial du backoff après une réinit échouée (ms).
const BACKOFF_INITIAL_MS: u64 = 200;

/// Facteur multiplicatif du backoff.
const BACKOFF_FACTEUR: u64 = 2;

/// Délai maximum du backoff.
const BACKOFF_MAX_MS: u64 = 5_000;

/// Pause entre deux appels à `mettre_a_jour()` (ms).
/// Le GPS envoie ~1–5 Hz, mais on lit le buffer plus souvent pour
/// absorber plusieurs trames d'un coup sans latence.
const PERIODE_GPS_MS: u64 = 200;

/// Nombre de satellites minimum pour considérer le fix comme stable.
const SEUIL_SATELLITES: u8 = 6;

// ─────────────────────────────────────────────────────────────────────────────
// Types publiés sur le canal
// ─────────────────────────────────────────────────────────────────────────────

/// Mesure GPS avec métadonnées de fiabilité.
#[derive(Debug, Clone)]
pub struct MesureGps {
    /// Dernière position valide. `None` si aucun fix depuis le démarrage.
    pub donnees: Option<DonneesGps>,
    /// Vrai si le dernier fix est >= 2D.
    pub valide:  bool,
    /// Nombre d'erreurs consécutives (I/O ou silence prolongé).
    pub erreurs_consecutives: u32,
}

// ─────────────────────────────────────────────────────────────────────────────
// Compteurs de santé
// ─────────────────────────────────────────────────────────────────────────────

/// Compteurs atomiques de santé GPS — lisibles depuis n'importe quel thread.
#[derive(Debug)]
pub struct SanteGps {
    pub erreurs: Arc<AtomicU32>,
    pub reinit:  Arc<AtomicU32>,
}

impl SanteGps {
    fn nouveau() -> Self {
        Self {
            erreurs: Arc::new(AtomicU32::new(0)),
            reinit:  Arc::new(AtomicU32::new(0)),
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Handle
// ─────────────────────────────────────────────────────────────────────────────

/// Handle retourné par [`lancer_gps`].
pub struct HandlesGps {
    /// Dernière mesure GPS disponible (canal à valeur courante).
    pub rx_gps: watch::Receiver<MesureGps>,
    /// Compteurs de santé pour la supervision.
    pub sante:  Arc<SanteGps>,
    /// Handle du thread GPS (pour jointure à l'arrêt).
    pub tache:  std::thread::JoinHandle<()>,
    /// Signal d'arrêt partagé avec le thread.
    arret:      Arc<AtomicBool>,
}

impl HandlesGps {
    /// Signale l'arrêt au thread GPS. Non-bloquant.
    pub fn arreter(&self) {
        self.arret.store(true, Ordering::Relaxed);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Point d'entrée Linux
// ─────────────────────────────────────────────────────────────────────────────

/// Lance le thread GPS sur le port série donné.
///
/// # Arguments
///
/// * `chemin_port` — chemin du périphérique UART, ex. `"/dev/ttyS2"`
///
/// # Erreurs
///
/// Retourne une erreur si le port série ne peut pas être ouvert.
/// Les erreurs survenant après le démarrage sont gérées en interne
/// (backoff + réinitialisation), jamais propagées.
#[cfg(target_os = "linux")]
pub fn lancer_gps(chemin_port: &str) -> crate::types::Result<HandlesGps> {
    use crate::hal::uart_linux::{PortSerieLinux, BAUDRATE_GPS};
    use crate::drivers::gps::DriverGps;

    let chemin = chemin_port.to_string();
    let port = PortSerieLinux::nouveau(&chemin, BAUDRATE_GPS)?;
    let driver = DriverGps::nouveau(port);

    Ok(lancer_avec_driver(driver, chemin))
}

/// Variante hors-Linux utilisant le mock UART (développement / CI).
#[cfg(not(target_os = "linux"))]
pub fn lancer_gps(_chemin_port: &str) -> crate::types::Result<HandlesGps> {
    use crate::hal::uart::PortSerieMock;
    use crate::drivers::gps::DriverGps;

    let driver = DriverGps::nouveau(PortSerieMock::nouveau());
    Ok(lancer_avec_driver(driver, "mock".into()))
}

// ─────────────────────────────────────────────────────────────────────────────
// Implémentation commune
// ─────────────────────────────────────────────────────────────────────────────

fn lancer_avec_driver<D>(driver: D, nom_port: String) -> HandlesGps
where
    D: CapteurGps + Send + 'static,
{
    let sante = Arc::new(SanteGps::nouveau());
    let arret = Arc::new(AtomicBool::new(false));

    let (tx_gps, rx_gps) = watch::channel(MesureGps {
        donnees: None, valide: false, erreurs_consecutives: 0,
    });

    let sante_thread = Arc::clone(&sante);
    let arret_thread = Arc::clone(&arret);

    let tache = std::thread::Builder::new()
        .name("capteur-gps".into())
        .spawn(move || {
            thread_gps(
                driver,
                tx_gps,
                Arc::clone(&sante_thread.erreurs),
                Arc::clone(&sante_thread.reinit),
                arret_thread,
                nom_port,
            )
        })
        .expect("Impossible de créer le thread capteur-gps");

    HandlesGps { rx_gps, sante, tache, arret }
}

// ─────────────────────────────────────────────────────────────────────────────
// Corps du thread
// ─────────────────────────────────────────────────────────────────────────────

fn thread_gps<D: CapteurGps>(
    mut driver:     D,
    tx:             watch::Sender<MesureGps>,
    cpt_err:        Arc<AtomicU32>,
    cpt_reinit:     Arc<AtomicU32>,
    arret:          Arc<AtomicBool>,
    nom_port:       String,
) {
    let mut nb_reinit  = 0u32;
    let mut backoff_ms = BACKOFF_INITIAL_MS;

    // ── Boucle externe : (ré)initialisation ──────────────────────────────────
    loop {
        if arret.load(Ordering::Relaxed) { return; }

        match driver.initialiser() {
            Ok(()) => {
                backoff_ms = BACKOFF_INITIAL_MS;
                nb_reinit  = 0;
            }
            Err(e) => {
                nb_reinit += 1;
                cpt_reinit.fetch_add(1, Ordering::Relaxed);
                let _ = tx.send(MesureGps {
                    donnees: None, valide: false,
                    erreurs_consecutives: nb_reinit,
                });
                if nb_reinit >= REINIT_MAX {
                    log::error!(target: "gps", "{} : {} réinitialisations sans succès — suspension 30s",
                                nom_port, nb_reinit);
                    std::thread::sleep(Duration::from_secs(30));
                    nb_reinit  = 0;
                    backoff_ms = BACKOFF_INITIAL_MS;
                } else {
                    log::warn!(target: "gps", "GPS init échouée ({:?}) — nouvelle tentative dans {}ms", e, backoff_ms);
                    std::thread::sleep(Duration::from_millis(backoff_ms));
                    backoff_ms = (backoff_ms * BACKOFF_FACTEUR).min(BACKOFF_MAX_MS);
                }
                continue;
            }
        }

        // ── Boucle interne : lecture continue ────────────────────────────────
        let mut silence_consecutif = 0u32;
        let mut fix_precedent:  Option<TypeFixGps> = None;
        let mut sats_precedents: u8 = 0;

        loop {
            if arret.load(Ordering::Relaxed) { return; }

            let nouvelle = driver.mettre_a_jour();

            if nouvelle {
                silence_consecutif = 0;
                let donnees = driver.derniere_donnee();
                let valide  = donnees.map(|d| d.type_fix.est_valide()).unwrap_or(false);

                // ── Détection des changements d'état fix ─────────────────────
                if let Some(ref d) = donnees {
                    let fix   = d.type_fix;
                    let sats  = d.nombre_satellites;

                    if Some(fix) != fix_precedent {
                        match fix {
                            TypeFixGps::Aucun =>
                                log::warn!(target: "gps", "Fix GPS perdu — {} sats", sats),
                            TypeFixGps::Fix2D =>
                                log::warn!(target: "gps", "Fix GPS 2D — {} sats  H={:.1}m", sats, d.precision_h),
                            TypeFixGps::Fix3D =>
                                log::info!(target: "gps", "Fix GPS 3D — {} sats  H={:.1}m  V={:.1}m",
                                    sats, d.precision_h, d.precision_v),
                            TypeFixGps::GnssDr =>
                                log::info!(target: "gps", "Fix GPS GNSS+DR — {} sats  H={:.1}m",
                                    sats, d.precision_h),
                        }
                        fix_precedent = Some(fix);
                    }

                    if sats_precedents >= SEUIL_SATELLITES && sats < SEUIL_SATELLITES {
                        log::warn!(target: "gps", "Fix GPS dégradé — {} sats < seuil {}", sats, SEUIL_SATELLITES);
                    } else if sats_precedents < SEUIL_SATELLITES && sats >= SEUIL_SATELLITES {
                        log::info!(target: "gps", "Fix GPS stable — {} sats ≥ seuil {}", sats, SEUIL_SATELLITES);
                    }
                    sats_precedents = sats;
                }

                let _ = tx.send(MesureGps {
                    donnees,
                    valide,
                    erreurs_consecutives: 0,
                });
            } else {
                silence_consecutif += 1;
                if silence_consecutif >= SILENCE_AVANT_REINIT {
                    cpt_err.fetch_add(1, Ordering::Relaxed);
                    log::warn!(target: "gps", "{} itérations sans trame — réinitialisation", silence_consecutif);
                    std::thread::sleep(Duration::from_millis(backoff_ms));
                    backoff_ms = (backoff_ms * BACKOFF_FACTEUR).min(BACKOFF_MAX_MS);
                    cpt_reinit.fetch_add(1, Ordering::Relaxed);
                    let _ = tx.send(MesureGps {
                        donnees: None, valide: false,
                        erreurs_consecutives: silence_consecutif,
                    });
                    break; // retour boucle externe → réinitialisation
                }
            }

            std::thread::sleep(Duration::from_millis(PERIODE_GPS_MS));
        }
    }
}
