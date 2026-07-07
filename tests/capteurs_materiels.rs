// tests/capteurs_materiels.rs
//! Tests d'intégration — capteurs réels sur cible (Orange Pi + /dev/i2c-0)
//!
//! Ces tests sont ignorés par défaut (`#[ignore]`). Sur la cible :
//!
//! ```bash
//! cargo test --test capteurs_materiels -- --ignored --nocapture
//! ```
//!
//! Prérequis :
//! - `/dev/i2c-0` accessible (droits ou groupe `i2c`)
//! - BMP280, VL53L0X, MPU9250 branchés sur le bus
//! - Répertoire de config existant : `/home/airhaum/config`
//! - Ne pas bouger l'appareil pendant les 6 premières secondes (calibration gyro)

use std::sync::atomic::Ordering;
use std::time::{Duration, Instant};
use tokio::time::sleep;

use airhaum::taches::taches_capteurs::lancer_capteurs;

// ─── Démarrage ────────────────────────────────────────────────────────────────

/// Vérifie que les 3 capteurs produisent une première mesure dans les 10s.
/// Le MPU9250 peut prendre jusqu'à 6s au premier démarrage (calibration gyro).
#[tokio::test]
#[ignore]
async fn test_demarrage_capteurs() {
    airhaum::systeme::calibration::initialiser_gestionnaire("/home/airhaum/config");

    let mut capteurs = lancer_capteurs()
        .expect("Bus I²C inaccessible — vérifier /dev/i2c-0 et les permissions");

    // Transférer rx_imu pour que le thread MPU9250 ne soit pas bloqué sur FIFO plein.
    let _rx_imu = capteurs.prendre_rx_imu();

    let debut = Instant::now();
    let timeout = Duration::from_secs(10);

    loop {
        let baro_ok  = capteurs.rx_baro.borrow().valide;
        let telem_ok = capteurs.rx_telem.borrow().valide;

        if baro_ok && telem_ok {
            println!("✓ Démarrage complet en {:.0}ms", debut.elapsed().as_millis());
            capteurs.arreter();
            return;
        }

        assert!(
            debut.elapsed() < timeout,
            "Timeout démarrage capteurs (baro={} telem={})",
            baro_ok, telem_ok
        );

        sleep(Duration::from_millis(100)).await;
    }
}

// ─── Fréquences simultanées ───────────────────────────────────────────────────

/// Vérifie que les 3 capteurs produisent des mesures à leur fréquence nominale
/// pendant 10s sous charge I²C simultanée.
///
/// Seuils (marge -30% par rapport aux fréquences mesurées en mode individuel) :
/// - BMP280  : > 5 Hz  (nominal ~7.5 Hz)
/// - VL53L0X : > 7 Hz  (nominal ~10.5 Hz)
/// - MPU9250 : > 100 Hz (nominal ~170 Hz)
#[tokio::test]
#[ignore]
async fn test_frequences_simultanees() {
    const DUREE_SECS: u64 = 10;

    airhaum::systeme::calibration::initialiser_gestionnaire("/home/airhaum/config");

    let mut capteurs = lancer_capteurs()
        .expect("Bus I²C inaccessible — vérifier /dev/i2c-0 et les permissions");

    // IMU : mpsc::Receiver — déplacé dans une tâche dédiée qui compte les messages.
    // Le sender est détenu par le thread MPU9250 et sera droppé quand ce thread
    // verra le flag arret (au plus PERIODE_IMU_MS=5ms après arreter()).
    let rx_imu = capteurs.prendre_rx_imu();
    let imu_compteur = tokio::spawn(async move {
        let mut rx = rx_imu;
        let mut n: u64 = 0;
        while rx.recv().await.is_some() {
            n += 1;
        }
        n
    });

    // Attendre la première mesure valide baro/telem (init capteurs).
    let debut_init = Instant::now();
    loop {
        if capteurs.rx_baro.borrow().valide && capteurs.rx_telem.borrow().valide {
            println!("✓ Init capteurs en {:.0}ms", debut_init.elapsed().as_millis());
            break;
        }
        assert!(
            debut_init.elapsed() < Duration::from_secs(10),
            "Timeout init capteurs"
        );
        sleep(Duration::from_millis(100)).await;
    }

    // Compter les mesures baro/telem via watch::changed() pendant DUREE_SECS.
    let mut rx_baro  = capteurs.rx_baro.clone();
    let mut rx_telem = capteurs.rx_telem.clone();
    rx_baro.borrow_and_update();
    rx_telem.borrow_and_update();

    let (mut n_baro, mut n_telem) = (0u64, 0u64);
    let (mut e_baro, mut e_telem) = (0u64, 0u64);
    let debut_mesure = Instant::now();
    let fin = Duration::from_secs(DUREE_SECS);

    while debut_mesure.elapsed() < fin {
        tokio::select! {
            Ok(_) = rx_baro.changed() => {
                if rx_baro.borrow_and_update().valide { n_baro += 1; } else { e_baro += 1; }
            }
            Ok(_) = rx_telem.changed() => {
                if rx_telem.borrow_and_update().valide { n_telem += 1; } else { e_telem += 1; }
            }
            _ = sleep(Duration::from_millis(1)) => {}
        }
    }

    let duree = debut_mesure.elapsed().as_secs_f32();

    // Arrêt : pose le flag → le thread MPU9250 voit le flag dans ≤5ms et drop
    // le sender → imu_compteur se débloque. Timeout 6s au cas où le thread est
    // en backoff au moment de l'arrêt (BACKOFF_MAX_MS = 5s).
    capteurs.arreter();
    let n_imu = tokio::time::timeout(Duration::from_secs(6), imu_compteur)
        .await
        .unwrap_or(Ok(0))
        .unwrap_or(0);

    let hz_baro  = n_baro  as f32 / duree;
    let hz_telem = n_telem as f32 / duree;
    let hz_imu   = n_imu   as f32 / duree;
    let reinit_baro  = capteurs.sante.reinit_baro.load(Ordering::Relaxed);
    let reinit_telem = capteurs.sante.reinit_telem.load(Ordering::Relaxed);
    let reinit_imu   = capteurs.sante.reinit_imu.load(Ordering::Relaxed);

    println!("\n--- Résultats sur {:.1}s ---", duree);
    println!("  BMP280  : {:>8.2} Hz  erreurs: {}  réinit: {}", hz_baro,  e_baro,  reinit_baro);
    println!("  VL53L0X : {:>8.2} Hz  erreurs: {}  réinit: {}", hz_telem, e_telem, reinit_telem);
    println!("  MPU9250 : {:>8.2} Hz  -             réinit: {}", hz_imu,           reinit_imu);

    assert!(hz_baro  > 5.0,   "BMP280 trop lent : {:.2} Hz (min 5)", hz_baro);
    assert!(hz_telem > 7.0,   "VL53L0X trop lent : {:.2} Hz (min 7)", hz_telem);
    assert!(hz_imu   > 100.0, "MPU9250 trop lent : {:.2} Hz (min 100)", hz_imu);

    assert_eq!(reinit_baro,  0, "BMP280 s'est réinitialisé {} fois", reinit_baro);
    assert_eq!(reinit_telem, 0, "VL53L0X s'est réinitialisé {} fois", reinit_telem);
    assert_eq!(reinit_imu,   0, "MPU9250 s'est réinitialisé {} fois", reinit_imu);

    println!("\n✓ Test fréquences simultanées OK");
}
