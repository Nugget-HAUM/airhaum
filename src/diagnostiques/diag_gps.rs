// src/diagnostiques/diag_gps.rs
//! Fonctions de diagnostic du GPS u-blox NEO-M8N.

use crate::hal::uart::PortSerie;
use crate::drivers::gps::DriverGps;
use crate::interfaces::gps::CapteurGps;
use crate::types::{TypeFixGps, Result};

// ─────────────────────────────────────────────────────────────────────────────
// Ouverture du port (helpers internes)
// ─────────────────────────────────────────────────────────────────────────────

#[cfg(target_os = "linux")]
fn ouvrir_port() -> Result<impl PortSerie> {
    use crate::hal::uart_linux::{PortSerieLinux, PORT_GPS_DEFAUT, BAUDRATE_GPS};
    PortSerieLinux::nouveau(PORT_GPS_DEFAUT, BAUDRATE_GPS)
}

#[cfg(not(target_os = "linux"))]
fn ouvrir_port() -> Result<impl PortSerie> {
    use crate::hal::uart::PortSerieMock;
    Ok(PortSerieMock::nouveau())
}

// ─────────────────────────────────────────────────────────────────────────────
// Test de communication
// ─────────────────────────────────────────────────────────────────────────────

/// Vérifie que le port série s'ouvre et que le driver s'initialise.
///
/// Ne vérifie pas la présence de trames (le GPS peut ne pas avoir de fix).
/// Succès = port ouvert + buffer drainé sans erreur I/O.
pub fn test_communication() -> Result<()> {
    let port = ouvrir_port()?;
    let mut driver = DriverGps::nouveau(port);
    driver.initialiser()?;
    println!("  GPS : port série ouvert, buffer drainé ✓");
    Ok(())
}

// ─────────────────────────────────────────────────────────────────────────────
// Attente du premier fix
// ─────────────────────────────────────────────────────────────────────────────

/// Attend le premier fix GPS jusqu'au `timeout_s` donné.
///
/// Affiche l'avancement (nombre de trames reçues, état du fix) à chaque
/// nouvelle trame. Retourne `Ok(())` dès qu'un fix >= 2D est obtenu.
pub fn attendre_fix(timeout_s: u64) -> Result<()> {
    let port = ouvrir_port()?;
    let mut driver = DriverGps::nouveau(port);
    driver.initialiser()?;

    let debut = std::time::Instant::now();
    let timeout = std::time::Duration::from_secs(timeout_s);
    let mut n_trames = 0u32;

    println!("  Attente fix GPS (timeout {}s)…", timeout_s);
    println!("  {:>6}  {:>6}  {:>8}  {:>8}  {:>5}  {}", "Trame", "Fix", "Lat", "Lon", "Sats", "Précision H");

    while debut.elapsed() < timeout {
        if driver.mettre_a_jour() {
            n_trames += 1;
            if let Some(d) = driver.derniere_donnee() {
                let fix_str = match d.type_fix {
                    TypeFixGps::Aucun   => "aucun",
                    TypeFixGps::Fix2D   => "2D",
                    TypeFixGps::Fix3D   => "3D",
                    TypeFixGps::GnssDr  => "GNSS+DR",
                };
                println!("  {:>6}  {:>6}  {:>8.4}  {:>8.4}  {:>5}  {:.1} m",
                    n_trames, fix_str,
                    d.latitude, d.longitude,
                    d.nombre_satellites,
                    d.precision_h);

                if d.type_fix.est_valide() {
                    println!("\n  ✓ Fix {} obtenu en {:.1}s",
                        fix_str, debut.elapsed().as_secs_f32());
                    return Ok(());
                }
            }
        }
        std::thread::sleep(std::time::Duration::from_millis(200));
    }

    println!("\n  ⚠ Timeout : fix non obtenu en {}s ({} trames reçues)", timeout_s, n_trames);
    Ok(())
}

// ─────────────────────────────────────────────────────────────────────────────
// Mesures continues
// ─────────────────────────────────────────────────────────────────────────────

/// Affiche les mesures GPS en continu pendant `duree_s` secondes.
pub fn mesures_continues(duree_s: u64) -> Result<()> {
    let port = ouvrir_port()?;
    let mut driver = DriverGps::nouveau(port);
    driver.initialiser()?;

    let debut = std::time::Instant::now();
    let duree = std::time::Duration::from_secs(duree_s);
    let mut n = 0u32;

    println!("  {:>6}  {:>8}  {:>10}  {:>10}  {:>8}  {:>8}  {:>5}  {:>5}",
        "N", "Fix", "Latitude", "Longitude", "Alt MSL", "Vit sol", "Sats", "Préc H");

    while debut.elapsed() < duree {
        if driver.mettre_a_jour() {
            n += 1;
            if let Some(d) = driver.derniere_donnee() {
                let fix_str = match d.type_fix {
                    TypeFixGps::Aucun   => "aucun",
                    TypeFixGps::Fix2D   => "2D   ",
                    TypeFixGps::Fix3D   => "3D   ",
                    TypeFixGps::GnssDr  => "DR   ",
                };
                println!("  {:>6}  {}  {:>10.5}  {:>10.5}  {:>7.1}m  {:>6.1}m/s  {:>5}  {:>4.1}m",
                    n, fix_str,
                    d.latitude, d.longitude,
                    d.altitude_msl,
                    d.vitesse_sol,
                    d.nombre_satellites,
                    d.precision_h);
            }
        }
        std::thread::sleep(std::time::Duration::from_millis(200));
    }
    println!("  {} trames reçues en {}s", n, duree_s);
    Ok(())
}

// ─────────────────────────────────────────────────────────────────────────────
// Diagnostic complet
// ─────────────────────────────────────────────────────────────────────────────

/// Diagnostic complet : test communication + 10 s de mesures.
pub fn diagnostic_complet() -> Result<()> {
    println!("\n── GPS NEO-M8N : Diagnostic ──────────────────────────────────");
    test_communication()?;
    println!();
    mesures_continues(10)?;
    println!("── Diagnostic GPS terminé ────────────────────────────────────");
    Ok(())
}
