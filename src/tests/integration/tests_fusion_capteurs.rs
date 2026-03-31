// tests/integration/tests_fusion_capteurs.rs
//! Tests d'intégration : capteurs en fonctionnement simultané
//!
//! Ces tests vérifient que les tâches capteurs (`taches/taches_capteurs.rs`)
//! fonctionnent correctement ensemble, en conditions proches du vol réel :
//! bus I²C partagé, lectures concurrentes, fréquences mesurées en simultané.
//!
//! # Exécution
//!
//! ```bash
//! # Sur la cible Linux avec les capteurs branchés :
//! cargo test --test tests_fusion_capteurs -- --nocapture
//!
//! # Sur machine de dev (mode mock) :
//! cargo test --test tests_fusion_capteurs -- --nocapture
//! ```
//!
//! # Distinction avec les tests unitaires
//!
//! Les tests unitaires (tests/unitaires/) testent chaque module isolément
//! avec des mocks. Ces tests d'intégration testent le système assemblé,
//! avec les vraies tâches async et les vrais canaux watch.

use std::sync::atomic::Ordering;
use std::time::{Duration, Instant};
use tokio::time::sleep;

use airhaum::taches::taches_capteurs::lancer_capteurs;

// ─────────────────────────────────────────────────────────────────────────────
// Test principal : fréquences simultanées
// ─────────────────────────────────────────────────────────────────────────────

/// Vérifie que les 3 capteurs produisent des mesures en parallèle
/// et mesure leurs fréquences réelles d'échantillonnage.
///
/// Ce test est l'équivalent intégration du test `tc` de la console de
/// diagnostic. Il valide que :
/// 1. Les 3 tâches démarrent et s'initialisent correctement
/// 2. Chaque capteur produit des mesures à sa fréquence attendue
/// 3. La concurrence sur le bus I²C ne dégrade pas les fréquences
/// 4. Il n'y a pas de réinitialisation intempestive
#[tokio::test]
async fn test_capteurs_simultanes_frequences() {
    const DUREE_TEST_SECS: u64 = 10;
    const TIMEOUT_PREMIERE_MESURE_MS: u64 = 5000;

    airhaum::systeme::calibration::initialiser_gestionnaire("/home/airhaum/config");

    println!("\n=== Test capteurs simultanés ({} secondes) ===", DUREE_TEST_SECS);

    let capteurs = lancer_capteurs().await;

    // ── Attendre la première mesure de chaque capteur ──────────────────────
    let debut_init = Instant::now();
    loop {
        let baro_ok  = capteurs.rx_baro.borrow().donnees.is_some();
        let telem_ok = capteurs.rx_telem.borrow().valide;
        let imu_ok   = capteurs.rx_imu.borrow().donnees.is_some();

        if baro_ok && telem_ok && imu_ok {
            println!("✓ Les 3 capteurs ont produit leur première mesure ({:.0}ms)",
                     debut_init.elapsed().as_millis());
            break;
        }

        if debut_init.elapsed() > Duration::from_millis(TIMEOUT_PREMIERE_MESURE_MS) {
            // En mode mock, les capteurs démarrent quasi instantanément.
            // Sur cible réelle, l'init MPU9250 prend ~500ms (calibration gyro).
            // Si on est ici, c'est une vraie anomalie.
            println!("⚠ Timeout démarrage (baro={} telem={} imu={})",
                     baro_ok, telem_ok, imu_ok);
            // On ne fait pas `assert!` ici : on laisse le test continuer
            // pour collecter des métriques même en cas de démarrage lent.
            break;
        }

        sleep(Duration::from_millis(50)).await;
    }

    // ── Collecter les mesures pendant DUREE_TEST_SECS ─────────────────────
    let mut rx_baro  = capteurs.rx_baro.clone();
    let mut rx_telem = capteurs.rx_telem.clone();
    let mut rx_imu   = capteurs.rx_imu.clone();

    // Marquer comme vus pour que changed() ne déclenche pas immédiatement
    rx_baro.borrow_and_update();
    rx_telem.borrow_and_update();
    rx_imu.borrow_and_update();

    let mut compteurs = CompteursMesures::nouveau();
    let debut_mesure = Instant::now();
    let fin = Duration::from_secs(DUREE_TEST_SECS);

    while debut_mesure.elapsed() < fin {
        tokio::select! {
            Ok(_) = rx_baro.changed() => {
                let m = rx_baro.borrow_and_update().clone();
                compteurs.enregistrer_baro(m.valide);
            }
            Ok(_) = rx_telem.changed() => {
                let m = rx_telem.borrow_and_update().clone();
                compteurs.enregistrer_telem(m.valide);
            }
            Ok(_) = rx_imu.changed() => {
                let m = rx_imu.borrow_and_update().clone();
                compteurs.enregistrer_imu(m.valide);
            }
            _ = sleep(Duration::from_millis(1)) => {}
        }
    }

    let duree_reelle = debut_mesure.elapsed().as_secs_f32();

    // ── Résultats ──────────────────────────────────────────────────────────
    let (hz_baro,  err_baro)  = compteurs.stats_baro(duree_reelle);
    let (hz_telem, err_telem) = compteurs.stats_telem(duree_reelle);
    let (hz_imu,   err_imu)   = compteurs.stats_imu(duree_reelle);

    let reinit_baro  = capteurs.sante.reinit_baro.load(Ordering::Relaxed);
    let reinit_telem = capteurs.sante.reinit_telem.load(Ordering::Relaxed);
    let reinit_imu   = capteurs.sante.reinit_imu.load(Ordering::Relaxed);

    println!("\n--- Résultats sur {:.1}s ---", duree_reelle);
    println!("  BMP280  : {:>8.2} Hz  erreurs: {}  réinit: {}", hz_baro,  err_baro,  reinit_baro);
    println!("  VL53L0X : {:>8.2} Hz  erreurs: {}  réinit: {}", hz_telem, err_telem, reinit_telem);
    println!("  MPU9250 : {:>8.2} Hz  erreurs: {}  réinit: {}", hz_imu,   err_imu,   reinit_imu);

    // ── Assertions ─────────────────────────────────────────────────────────
    // Seuils conservateurs : on accepte une dégradation de 30% par rapport
    // aux fréquences mesurées en mode individuel (tf).
    // Fréquences de référence mesurées : BMP280≈7.5Hz, VL53L0X≈10.5Hz, MPU9250≈170Hz
    assert!(
        hz_baro > 5.0,
        "BMP280 trop lent en simultané : {:.2} Hz (min attendu 5 Hz)", hz_baro
    );
    assert!(
        hz_telem > 7.0,
        "VL53L0X trop lent en simultané : {:.2} Hz (min attendu 7 Hz)", hz_telem
    );
    assert!(
        hz_imu > 100.0,
        "MPU9250 trop lent en simultané : {:.2} Hz (min attendu 100 Hz)", hz_imu
    );

    // Pas de réinitialisation en fonctionnement normal
    assert_eq!(reinit_baro,  0, "BMP280  s'est réinitialisé {} fois", reinit_baro);
    assert_eq!(reinit_telem, 0, "VL53L0X s'est réinitialisé {} fois", reinit_telem);
    assert_eq!(reinit_imu,   0, "MPU9250 s'est réinitialisé {} fois", reinit_imu);

    // Taux d'erreur < 5% sur chaque capteur
    let total_baro  = compteurs.n_baro  + err_baro  as u64;
    let total_telem = compteurs.n_telem + err_telem as u64;
    let total_imu   = compteurs.n_imu   + err_imu   as u64;

    if total_baro > 0 {
        let taux = err_baro as f32 / total_baro as f32 * 100.0;
        assert!(taux < 5.0, "Taux d'erreur BMP280 trop élevé : {:.1}%", taux);
    }
    if total_telem > 0 {
        let taux = err_telem as f32 / total_telem as f32 * 100.0;
        assert!(taux < 5.0, "Taux d'erreur VL53L0X trop élevé : {:.1}%", taux);
    }
    if total_imu > 0 {
        let taux = err_imu as f32 / total_imu as f32 * 100.0;
        assert!(taux < 5.0, "Taux d'erreur MPU9250 trop élevé : {:.1}%", taux);
    }

    println!("\n✓ Test capteurs simultanés OK");

    // Arrêt propre des tâches
    for t in capteurs.taches {
        t.abort();
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Test : démarrage et présence des premières mesures
// ─────────────────────────────────────────────────────────────────────────────

/// Vérifie que les 3 capteurs produisent une première mesure dans les 5s
#[tokio::test]
async fn test_demarrage_capteurs() {
    airhaum::systeme::calibration::initialiser_gestionnaire("/home/airhaum/config");

    let capteurs = lancer_capteurs().await;

    let debut = Instant::now();
    let timeout = Duration::from_secs(5);

    loop {
        let baro_ok  = capteurs.rx_baro.borrow().donnees.is_some();
        let telem_ok = capteurs.rx_telem.borrow().valide;
        let imu_ok   = capteurs.rx_imu.borrow().donnees.is_some();

        if baro_ok && telem_ok && imu_ok {
            println!("✓ Démarrage complet en {:.0}ms", debut.elapsed().as_millis());
            for t in capteurs.taches { t.abort(); }
            return;
        }

        assert!(
            debut.elapsed() < timeout,
            "Timeout démarrage capteurs (baro={} telem={} imu={})",
            baro_ok, telem_ok, imu_ok
        );

        sleep(Duration::from_millis(100)).await;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Structure utilitaire de comptage
// ─────────────────────────────────────────────────────────────────────────────

struct CompteursMesures {
    n_baro:  u64,
    n_telem: u64,
    n_imu:   u64,
    e_baro:  u64,
    e_telem: u64,
    e_imu:   u64,
}

impl CompteursMesures {
    fn nouveau() -> Self {
        Self { n_baro: 0, n_telem: 0, n_imu: 0, e_baro: 0, e_telem: 0, e_imu: 0 }
    }

    fn enregistrer_baro(&mut self, valide: bool) {
        if valide { self.n_baro += 1; } else { self.e_baro += 1; }
    }
    fn enregistrer_telem(&mut self, valide: bool) {
        if valide { self.n_telem += 1; } else { self.e_telem += 1; }
    }
    fn enregistrer_imu(&mut self, valide: bool) {
        if valide { self.n_imu += 1; } else { self.e_imu += 1; }
    }

    fn stats_baro(&self, duree: f32)  -> (f32, u64) { (self.n_baro  as f32 / duree, self.e_baro)  }
    fn stats_telem(&self, duree: f32) -> (f32, u64) { (self.n_telem as f32 / duree, self.e_telem) }
    fn stats_imu(&self, duree: f32)   -> (f32, u64) { (self.n_imu   as f32 / duree, self.e_imu)   }
}
