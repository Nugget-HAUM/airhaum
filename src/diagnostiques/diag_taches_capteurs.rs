// src/diagnostiques/diag_taches_capteurs.rs
//! Diagnostic des tâches capteurs en fonctionnement simultané
//!
//! Teste la brique `taches/taches_capteurs.rs` telle qu'elle sera
//! utilisée en vol : les 3 capteurs tournent en parallèle sur le
//! vrai bus I²C et on mesure les fréquences réelles.

use std::sync::atomic::Ordering;
use std::time::{Duration, Instant};

use crate::taches::taches_capteurs::lancer_capteurs;

/// Lance les 3 capteurs en parallèle et mesure leurs fréquences pendant `duree_secs` secondes.
///
/// Affiche un tableau de synthèse et retourne `Ok(())` si tous les capteurs
/// ont produit des mesures. Appelé depuis `airhaum-test.rs` option `tc`.
pub async fn test_capteurs_simultanes(duree_secs: u64) -> crate::types::Result<()> {
    println!("\n=== Capteurs simultanés ({} secondes) ===", duree_secs);
    println!("Lance les tâches de vol réelles et mesure les fréquences en parallèle.\n");

    let capteurs = lancer_capteurs().await;
    println!("✓ Tâches capteurs lancées\n");

    // Attendre la première mesure de chaque capteur (max 5s)
    let debut_init = Instant::now();
    loop {
        let baro_ok  = capteurs.rx_baro.borrow().donnees.is_some();
        let telem_ok = capteurs.rx_telem.borrow().valide;
        let imu_ok   = capteurs.rx_imu.borrow().donnees.is_some();
        if baro_ok && telem_ok && imu_ok { break; }
        if debut_init.elapsed() > Duration::from_secs(5) {
            eprintln!("⚠ Timeout démarrage (baro={} telem={} imu={})", baro_ok, telem_ok, imu_ok);
            break;
        }
        tokio::time::sleep(tokio::time::Duration::from_millis(100)).await;
    }
    println!("✓ Première mesure reçue ({:.0}ms)\n", debut_init.elapsed().as_millis());

    // Comptage sur duree_secs
    let mut rx_baro  = capteurs.rx_baro.clone();
    let mut rx_telem = capteurs.rx_telem.clone();
    let mut rx_imu   = capteurs.rx_imu.clone();
    rx_baro.borrow_and_update();
    rx_telem.borrow_and_update();
    rx_imu.borrow_and_update();

    let (mut n_baro, mut n_telem, mut n_imu) = (0u64, 0u64, 0u64);
    let (mut e_baro, mut e_telem, mut e_imu) = (0u64, 0u64, 0u64);
    let debut = Instant::now();
    let duree = Duration::from_secs(duree_secs);
    let mut dernier_affichage = Instant::now();

    while debut.elapsed() < duree {
        tokio::select! {
            Ok(_) = rx_baro.changed() => {
                if rx_baro.borrow_and_update().valide { n_baro += 1; } else { e_baro += 1; }
            }
            Ok(_) = rx_telem.changed() => {
                if rx_telem.borrow_and_update().valide { n_telem += 1; } else { e_telem += 1; }
            }
            Ok(_) = rx_imu.changed() => {
                if rx_imu.borrow_and_update().valide { n_imu += 1; } else { e_imu += 1; }
            }
            _ = tokio::time::sleep(tokio::time::Duration::from_millis(1)) => {}
        }
        if dernier_affichage.elapsed() >= Duration::from_secs(2) {
            println!("  t={:.0}s | BMP280: {} mes  VL53L0X: {} mes  MPU9250: {} mes",
                     debut.elapsed().as_secs_f32(), n_baro, n_telem, n_imu);
            dernier_affichage = Instant::now();
        }
    }

    let duree_reelle = debut.elapsed().as_secs_f32();

    // Synthèse
    println!("\n╔════════════════════════════════════════════╗");
    println!("║  RÉSULTATS — {:.1}s                          ║", duree_reelle);
    println!("╚════════════════════════════════════════════╝");
    println!("  {:<10} {:>8}  {:>8}  {:>10}", "Capteur", "Hz réel", "Mesures", "Erreurs");
    println!("  {:-<10}-+-{:-<8}-+-{:-<8}-+-{:-<10}", "", "", "", "");
    println!("  {:<10} {:>8.2}  {:>8}  {:>10}", "BMP280",  n_baro  as f32 / duree_reelle, n_baro,  e_baro);
    println!("  {:<10} {:>8.2}  {:>8}  {:>10}", "VL53L0X", n_telem as f32 / duree_reelle, n_telem, e_telem);
    println!("  {:<10} {:>8.2}  {:>8}  {:>10}", "MPU9250", n_imu   as f32 / duree_reelle, n_imu,   e_imu);

    let reinit_baro  = capteurs.sante.reinit_baro.load(Ordering::Relaxed);
    let reinit_telem = capteurs.sante.reinit_telem.load(Ordering::Relaxed);
    let reinit_imu   = capteurs.sante.reinit_imu.load(Ordering::Relaxed);
    println!("\n  Réinitialisations : BMP280={}  VL53L0X={}  MPU9250={}",
             reinit_baro, reinit_telem, reinit_imu);
    println!("\n💡 Comparer avec les tests individuels (12/25/37) pour détecter");
    println!("   une dégradation due à la contention sur le bus I²C.");

    for t in capteurs.taches { t.abort(); }
    Ok(())
}
