// tests/capteurs_architecture.rs
//! Tests d'intégration — architecture des canaux capteurs (hors matériel)
//!
//! Vérifie que la mécanique de `lancer_avec_bus` est correcte :
//! types de canaux, transfert du receiver IMU, compteurs de santé, arrêt.
//! Tourne sur machine de développement avec `I2cMock`.
//!
//! # Exécution
//! cargo test --test capteurs_architecture

use std::sync::{Arc, Mutex};
use std::sync::atomic::Ordering;
use std::time::Duration;

use airhaum::hal::i2c::I2cMock;
use airhaum::taches::taches_capteurs::lancer_avec_bus;

fn mock_bus(simuler_erreur: bool) -> Arc<Mutex<I2cMock>> {
    let mut mock = I2cMock::nouveau();
    mock.simuler_erreur = simuler_erreur;
    Arc::new(Mutex::new(mock))
}

// ─── Structure des handles ────────────────────────────────────────────────────

#[test]
fn les_canaux_watch_sont_lisibles_immediatement() {
    let bus = mock_bus(false);
    let capteurs = lancer_avec_bus(bus);

    // rx_baro et rx_telem sont des watch::Receiver : borrow() ne bloque pas.
    let baro = capteurs.rx_baro.borrow();
    assert!(!baro.valide, "baro invalide au démarrage avant toute mesure");

    let telem = capteurs.rx_telem.borrow();
    assert!(!telem.valide, "telem invalide au démarrage avant toute mesure");

    capteurs.arreter();
}

#[test]
fn prendre_rx_imu_transfere_le_receiver() {
    let bus = mock_bus(false);
    let mut capteurs = lancer_avec_bus(bus);

    // Premier appel : doit réussir.
    let _rx = capteurs.prendre_rx_imu();

    // Deuxième appel : doit paniquer (récepteur déjà transféré).
    let resultat = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        capteurs.prendre_rx_imu();
    }));
    assert!(resultat.is_err(), "un second prendre_rx_imu() doit paniquer");

    capteurs.arreter();
}

// ─── Compteurs de santé ───────────────────────────────────────────────────────

#[test]
fn les_erreurs_sont_comptees_quand_le_bus_est_ko() {
    // Mock en erreur totale : les threads voient des erreurs dès la première
    // transaction I²C et incrémentent les compteurs.
    let bus = mock_bus(true);
    let capteurs = lancer_avec_bus(bus);

    std::thread::sleep(Duration::from_millis(200));

    // Avec simuler_erreur=true, les threads échouent à l'init → reinit_* s'incrémente.
    // erreurs_* ne compte que les échecs de mesure (boucle interne, après init réussie).
    let reinit_baro  = capteurs.sante.reinit_baro.load(Ordering::Relaxed);
    let reinit_telem = capteurs.sante.reinit_telem.load(Ordering::Relaxed);
    let reinit_imu   = capteurs.sante.reinit_imu.load(Ordering::Relaxed);

    assert!(reinit_baro  > 0, "aucune réinit baro comptée");
    assert!(reinit_telem > 0, "aucune réinit telem comptée");
    assert!(reinit_imu   > 0, "aucune réinit imu comptée");

    capteurs.arreter();
}

// ─── Arrêt ───────────────────────────────────────────────────────────────────

#[test]
fn arreter_pose_le_flag_sans_bloquer() {
    let bus = mock_bus(true);
    let capteurs = lancer_avec_bus(bus);

    // arreter() doit retourner immédiatement (pas de join bloquant).
    let debut = std::time::Instant::now();
    capteurs.arreter();
    assert!(
        debut.elapsed() < Duration::from_millis(50),
        "arreter() a bloqué plus de 50ms"
    );
}
