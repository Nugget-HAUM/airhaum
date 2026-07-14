// src/bin/airhaum_test/main.rs
//! Véhicule d'intégration progressive d'AirHaum II.
//!
//! Ce binaire suit la même séquence de démarrage qu'`airhaum-vol`, en ajoutant
//! à chaque étape des diagnostics et des validations explicites.
//! Chaque itération de développement implémente une étape supplémentaire :
//!
//! ```text
//! [1] Initialisation matérielle  ✓
//! [2] Vérification des capteurs  ← itération courante
//! [3] Calibration
//! [4] Prêt à armer
//! [5] Armé
//! [6] Prêt à rouler
//! [7] Vol autonome
//! [8] Atterrissage / désarmement
//! ```
//!
//! Règle : ne pas avancer à l'étape N+1 tant que l'étape N n'est pas validée sur cible réelle.

mod console_calibration;
mod console_armement;
mod console_test;

use std::io::{self, Write};
use airhaum::mission::etat_machine::{MachineEtatVol, CommandeVol, ContexteVol};
use airhaum::taches::taches_gps::HandlesGps;

use console_calibration::afficher_bilan_calibrations;
use console_armement::afficher_checklist_armement;
use console_test::{
    afficher_menu, afficher_sante,
    afficher_attitude, afficher_navigation, afficher_altitude,
    attendre_fix_gps, afficher_gps, sauvegarder_assistance_gps,
    afficher_etat_arduino, tester_debattements_servos, afficher_journal,
    afficher_serie_telemetre,
};

#[tokio::main]
async fn main() {
    // Le handle doit rester en vie jusqu'à la fin de main.
    // Son Drop envoie le signal d'arrêt au thread écrivain et attend le flush.
    let _journal = airhaum::systeme::journalisation::initialiser()
        .expect("Impossible d'initialiser la journalisation");

    // ── Étape 1 : Initialisation matérielle ──────────────────────────────────
    let mut mae = MachineEtatVol::nouveau();
    log::info!(target: "systeme", "Lancement AirHaum II v{}", airhaum::VERSION);
    log::info!(target: "mission", "MAÉ : {}", mae.etat());

    airhaum::systeme::calibration::initialiser_gestionnaire("/home/airhaum/config");

    // Ouverture du bus I²C — reste ouvert pour toute la durée du programme.
    // Le même descripteur est ensuite partagé entre les threads capteurs.
    #[cfg(target_os = "linux")]
    let bus = {
        use airhaum::hal::i2c_linux::I2cLinux;
        match I2cLinux::nouveau(0) {
            Ok(i2c) => std::sync::Arc::new(std::sync::Mutex::new(i2c)),
            Err(e) => {
                log::error!(target: "systeme", "Bus I²C inaccessible : {:?}", e);
                log::error!(target: "systeme", "Vérifiez que l'I²C est activé (armbian-config) et les permissions (groupe i2c)");
                std::process::exit(1);
            }
        }
    };
    #[cfg(not(target_os = "linux"))]
    let bus = std::sync::Arc::new(std::sync::Mutex::new(
        airhaum::hal::i2c::I2cMock::nouveau()
    ));

    mae.traiter_commande(CommandeVol::DriversInitialises);
    log::info!(target: "mission", "Bus I²C OK → {}", mae.etat());

    // ── GPS : lancement du thread de réception ────────────────────────────────
    // Lancé tôt pour maximiser les chances d'avoir un fix GPS au moment où
    // l'opérateur valide les calibrations et passe au menu.
    #[cfg(target_os = "linux")]
    let gps: Option<HandlesGps> = {
        use airhaum::hal::uart_linux::PORT_GPS_DEFAUT;
        match airhaum::taches::taches_gps::lancer_gps(PORT_GPS_DEFAUT) {
            Ok(h)  => Some(h),
            Err(e) => { log::warn!(target: "gps", "GPS non disponible : {:?}", e); None }
        }
    };
    #[cfg(not(target_os = "linux"))]
    let gps: Option<HandlesGps> = None;

    // ── Servo/Arduino : lancement du thread de commande ──────────────────────
    // Lancé tôt pour que la liaison soit détectée avant la check-liste armement.
    // Toujours infaillible : si le port est inaccessible, erreur_port est renseigné.
    #[cfg(target_os = "linux")]
    let servo = {
        use airhaum::hal::uart_linux::PORT_ARDUINO_DEFAUT;
        airhaum::taches::taches_servo::lancer_servo(PORT_ARDUINO_DEFAUT)
    };
    #[cfg(not(target_os = "linux"))]
    let servo = airhaum::taches::taches_servo::lancer_servo("");

    // ── Détection de reprise rapide ───────────────────────────────────────────
    // L'appareil est peut-être déjà en vol (redémarrage logiciel suite à une
    // panique ou un watchdog). Dans ce cas les capteurs sont déjà actifs et on
    // reprend les commandes sans repasser par la séquence sol.
    let reprise_rapide = airhaum::taches::taches_capteurs::detecter_reprise_rapide(&bus);

    if reprise_rapide {
        println!("\n╔══════════════════════════════════════════════════════╗");
        println!("║  ⚡ DÉMARRAGE : Reprise rapide (appareil en vol)     ║");
        println!("║     Séquence sol ignorée — reprise des commandes     ║");
        println!("╚══════════════════════════════════════════════════════╝\n");
    } else {
        println!("\n╔══════════════════════════════════════════════════════╗");
        println!("║     DÉMARRAGE : Séquence sol nominale                ║");
        println!("║{:^54}║", airhaum::VERSION);
        println!("╚══════════════════════════════════════════════════════╝\n");

        // ── Bilan des calibrations ────────────────────────────────────────────
        // Lisibles sans bus I²C actif — l'opérateur peut recalibrer avant que
        // les drivers démarrent.
        if !afficher_bilan_calibrations() {
            log::info!(target: "systeme", "Session abandonnée à l'initialisation — arrêt opérateur");
            println!("\nAu revoir !");
            return;
        }
    }

    // ── Étape 2 : Lancement et vérification des capteurs ─────────────────────
    let mut capteurs = airhaum::taches::taches_capteurs::lancer_avec_bus(bus);

    if reprise_rapide {
        mae.traiter_commande(CommandeVol::RepriseRapide);
        log::info!(target: "mission", "Reprise rapide → {}", mae.etat());
    } else {
        mae.traiter_commande(CommandeVol::ConfigurationTerminee);
        log::info!(target: "mission", "Threads capteurs lancés → {}", mae.etat());

        println!("Vérification des capteurs :");
        let debut = std::time::Instant::now();
        let mut imu_ok = false;
        loop {
            let baro_ok  = capteurs.rx_baro.borrow().donnees.is_some();
            let telem_ok = capteurs.rx_telem.borrow().valide;
            if let Some(rx) = capteurs.rx_imu.as_mut() {
                while let Ok(m) = rx.try_recv() {
                    if m.donnees.is_some() { imu_ok = true; }
                }
            }
            print!("\r  BMP280 {}  VL53L0X {}  MPU9250 {}   ",
                if baro_ok  { "✓" } else { "·" },
                if telem_ok { "✓" } else { "·" },
                if imu_ok   { "✓" } else { "·" });
            io::stdout().flush().unwrap();
            if baro_ok && telem_ok && imu_ok { println!(); break; }
            if debut.elapsed() > std::time::Duration::from_secs(10) {
                println!();
                log::warn!(target: "systeme", "Timeout vérification capteurs (baro={} telem={} imu={})",
                    baro_ok, telem_ok, imu_ok);
                break;
            }
            tokio::time::sleep(tokio::time::Duration::from_millis(100)).await;
        }

        // ── Lancement de l'estimation d'état ─────────────────────────────────
        let rx_imu = capteurs.prendre_rx_imu();

        let pression_ref = {
            use airhaum::systeme::calibration::gestionnaire;
            use airhaum::drivers::barometre::calibration::CalibrationBarometre;
            use airhaum::types::Pression;
            gestionnaire()
                .charger::<CalibrationBarometre>()
                .ok()
                .flatten()
                .map(|c| Pression::depuis_pascals(c.obtenir_pression_reference()))
                .unwrap_or_else(|| {
                    log::warn!(target: "baro", "Référence pression non calibrée — ISA standard utilisée");
                    Pression::niveau_mer_standard()
                })
        };

        let rx_gps_estimation = gps.as_ref()
            .map(|g| g.rx_gps.clone())
            .unwrap_or_else(|| {
                let (_, rx) = tokio::sync::watch::channel(
                    airhaum::taches::taches_gps::MesureGps {
                        donnees: None, valide: false, erreurs_consecutives: 0,
                    }
                );
                rx
            });

        let mut estimation = airhaum::taches::taches_estimation::lancer_estimation(
            rx_imu,
            capteurs.rx_baro.clone(),
            capteurs.rx_telem.clone(),
            rx_gps_estimation,
            pression_ref,
        );

        // Attente de la première attitude (timeout 5s)
        match tokio::time::timeout(
            std::time::Duration::from_secs(5),
            estimation.rx_attitude.changed(),
        ).await {
            Ok(Ok(_)) => {
                mae.tick(&ContexteVol {
                    estimation_prete: true,
                    vitesse_sol_ms:   None,
                    altitude_m:       None,
                    altitude_cible_m: 100.0,
                    hauteur_sol_mm:   None,
                });
                log::info!(target: "mission", "Estimation active → {}", mae.etat());
            }
            _ => log::error!(target: "systeme", "Timeout : thread d'estimation non démarré"),
        }

        // ── Check-liste armement ──────────────────────────────────────────────
        if afficher_checklist_armement(&estimation, gps.as_ref(), &servo) {

        // ── Menu de test ──────────────────────────────────────────────────────
        loop {
            afficher_menu(&mae);

            let mut choix = String::new();
            io::stdin().read_line(&mut choix).unwrap();
            let choix = choix.trim();

            match choix {
                "1" => {
                    match airhaum::diagnostiques::diag_bmp280::tester_bmp280() {
                        Ok(_) => log::info!(target: "diag", "Test BMP280 terminé avec succès"),
                        Err(e) => log::error!(target: "diag", "Test BMP280 : {:?}", e),
                    }
                }
                "11" => {
                    match airhaum::diagnostiques::diag_bmp280::calibrer_bmp280() {
                        Ok(_) => log::info!(target: "diag", "Calibration BMP280 effectuée"),
                        Err(e) => log::error!(target: "diag", "Calibration BMP280 : {:?}", e),
                    }
                }
                "12" => {
                    match airhaum::diagnostiques::diag_bmp280::test_frequence_bmp280(100) {
                        Ok(stats) => log::info!(target: "diag", "BMP280 : {:.2} Hz (jitter ±{:.2} ms)", stats.hz_moyen, stats.jitter_ms),
                        Err(e) => log::error!(target: "diag", "Fréquence BMP280 : {:?}", e),
                    }
                }
                "2" => {
                    #[cfg(target_os = "linux")]
                    let mut i2c = match airhaum::hal::I2cLinux::nouveau(0) {
                        Ok(bus) => bus,
                        Err(e) => { log::error!(target: "systeme", "Erreur I²C : {:?}", e); continue; }
                    };
                    #[cfg(not(target_os = "linux"))]
                    let mut i2c = airhaum::hal::i2c::I2cMock::nouveau();
                    match airhaum::diagnostiques::diag_vl53l0x::test_communication(&mut i2c) {
                        Ok(_) => log::info!(target: "diag", "Communication VL53L0X OK"),
                        Err(e) => log::error!(target: "diag", "Communication VL53L0X : {:?}", e),
                    }
                }
                "21" => {
                    #[cfg(target_os = "linux")]
                    let i2c = match airhaum::hal::I2cLinux::nouveau(0) {
                        Ok(bus) => bus,
                        Err(e) => { log::error!(target: "systeme", "Erreur I²C : {:?}", e); continue; }
                    };
                    #[cfg(not(target_os = "linux"))]
                    let i2c = airhaum::hal::i2c::I2cMock::nouveau();
                    match airhaum::diagnostiques::test_initialisation(i2c) {
                        Ok(_) => log::info!(target: "diag", "Initialisation VL53L0X OK"),
                        Err(e) => log::error!(target: "diag", "Initialisation VL53L0X : {:?}", e),
                    }
                }
                "22" => {
                    #[cfg(target_os = "linux")]
                    let i2c = match airhaum::hal::I2cLinux::nouveau(0) {
                        Ok(bus) => bus,
                        Err(e) => { log::error!(target: "systeme", "Erreur I²C : {:?}", e); continue; }
                    };
                    #[cfg(not(target_os = "linux"))]
                    let i2c = airhaum::hal::i2c::I2cMock::nouveau();
                    match airhaum::diagnostiques::diag_vl53l0x::test_mesure_unique(i2c) {
                        Ok(_) => log::info!(target: "diag", "Mesure VL53L0X OK"),
                        Err(e) => log::error!(target: "diag", "Mesure VL53L0X : {:?}", e),
                    }
                }
                "23" => {
                    #[cfg(target_os = "linux")]
                    let i2c = match airhaum::hal::I2cLinux::nouveau(0) {
                        Ok(bus) => bus,
                        Err(e) => { log::error!(target: "systeme", "Erreur I²C : {:?}", e); continue; }
                    };
                    #[cfg(not(target_os = "linux"))]
                    let i2c = airhaum::hal::i2c::I2cMock::nouveau();
                    match airhaum::diagnostiques::diag_vl53l0x::test_mesures_continues(i2c, 10) {
                        Ok(_) => log::info!(target: "diag", "Mesures continues VL53L0X OK"),
                        Err(e) => log::error!(target: "diag", "Mesures continues VL53L0X : {:?}", e),
                    }
                }
                "24" => {
                    #[cfg(target_os = "linux")]
                    let i2c = match airhaum::hal::I2cLinux::nouveau(0) {
                        Ok(bus) => bus,
                        Err(e) => { log::error!(target: "systeme", "Erreur I²C : {:?}", e); continue; }
                    };
                    #[cfg(not(target_os = "linux"))]
                    let i2c = airhaum::hal::i2c::I2cMock::nouveau();
                    match airhaum::diagnostiques::diag_vl53l0x::diagnostic_complet(i2c) {
                        Ok(_) => log::info!(target: "diag", "Diagnostic VL53L0X OK"),
                        Err(e) => log::error!(target: "diag", "Diagnostic VL53L0X : {:?}", e),
                    }
                }
                "25" => {
                    #[cfg(target_os = "linux")]
                    let i2c = match airhaum::hal::I2cLinux::nouveau(0) {
                        Ok(bus) => bus,
                        Err(e) => { log::error!(target: "systeme", "Erreur I²C : {:?}", e); continue; }
                    };
                    #[cfg(not(target_os = "linux"))]
                    let i2c = airhaum::hal::i2c::I2cMock::nouveau();
                    match airhaum::diagnostiques::diag_vl53l0x::test_frequence(i2c, 100) {
                        Ok(stats) => log::info!(target: "diag", "VL53L0X : {:.2} Hz (jitter ±{:.2} ms)", stats.hz_moyen, stats.jitter_ms),
                        Err(e) => log::error!(target: "diag", "Fréquence VL53L0X : {:?}", e),
                    }
                }
                "26" => afficher_serie_telemetre(&capteurs).await,
                "3" => {
                    #[cfg(target_os = "linux")]
                    let i2c = match airhaum::hal::I2cLinux::nouveau(0) {
                        Ok(bus) => bus,
                        Err(e) => { log::error!(target: "systeme", "Erreur I²C : {:?}", e); continue; }
                    };
                    #[cfg(not(target_os = "linux"))]
                    let i2c = airhaum::hal::i2c::I2cMock::nouveau();
                    match airhaum::diagnostiques::diag_mpu9250::test_communication(i2c) {
                        Ok(_) => log::info!(target: "diag", "Communication MPU9250 OK"),
                        Err(e) => log::error!(target: "diag", "Communication MPU9250 : {:?}", e),
                    }
                }
                "31" => {
                    #[cfg(target_os = "linux")]
                    let i2c = match airhaum::hal::I2cLinux::nouveau(0) {
                        Ok(bus) => bus,
                        Err(e) => { log::error!(target: "systeme", "Erreur I²C : {:?}", e); continue; }
                    };
                    #[cfg(not(target_os = "linux"))]
                    let i2c = airhaum::hal::i2c::I2cMock::nouveau();
                    match airhaum::diagnostiques::diag_mpu9250::test_initialisation(i2c) {
                        Ok(_) => log::info!(target: "diag", "Initialisation MPU9250 OK"),
                        Err(e) => log::error!(target: "diag", "Initialisation MPU9250 : {:?}", e),
                    }
                }
                "32" => {
                    #[cfg(target_os = "linux")]
                    let i2c = match airhaum::hal::I2cLinux::nouveau(0) {
                        Ok(bus) => bus,
                        Err(e) => { log::error!(target: "systeme", "Erreur I²C : {:?}", e); continue; }
                    };
                    #[cfg(not(target_os = "linux"))]
                    let i2c = airhaum::hal::i2c::I2cMock::nouveau();
                    match airhaum::diagnostiques::diag_mpu9250::test_mesure_unique(i2c) {
                        Ok(_) => log::info!(target: "diag", "Mesure MPU9250 OK"),
                        Err(e) => log::error!(target: "diag", "Mesure MPU9250 : {:?}", e),
                    }
                }
                "33" => {
                    #[cfg(target_os = "linux")]
                    let i2c = match airhaum::hal::I2cLinux::nouveau(0) {
                        Ok(bus) => bus,
                        Err(e) => { log::error!(target: "systeme", "Erreur I²C : {:?}", e); continue; }
                    };
                    #[cfg(not(target_os = "linux"))]
                    let i2c = airhaum::hal::i2c::I2cMock::nouveau();
                    match airhaum::diagnostiques::diag_mpu9250::test_mesures_continues(i2c, 10) {
                        Ok(_) => log::info!(target: "diag", "Mesures continues MPU9250 OK"),
                        Err(e) => log::error!(target: "diag", "Mesures continues MPU9250 : {:?}", e),
                    }
                }
                "34" => {
                    #[cfg(target_os = "linux")]
                    let i2c = match airhaum::hal::I2cLinux::nouveau(0) {
                        Ok(bus) => bus,
                        Err(e) => { log::error!(target: "systeme", "Erreur I²C : {:?}", e); continue; }
                    };
                    #[cfg(not(target_os = "linux"))]
                    let i2c = airhaum::hal::i2c::I2cMock::nouveau();
                    match airhaum::diagnostiques::diag_mpu9250::calibrer_gyro(i2c) {
                        Ok(_) => log::info!(target: "diag", "Calibration gyro MPU9250 OK"),
                        Err(e) => log::error!(target: "diag", "Calibration gyro MPU9250 : {:?}", e),
                    }
                }
                "35" => {
                    #[cfg(target_os = "linux")]
                    let i2c = match airhaum::hal::I2cLinux::nouveau(0) {
                        Ok(bus) => bus,
                        Err(e) => { log::error!(target: "systeme", "Erreur I²C : {:?}", e); continue; }
                    };
                    #[cfg(not(target_os = "linux"))]
                    let i2c = airhaum::hal::i2c::I2cMock::nouveau();
                    match airhaum::diagnostiques::diag_mpu9250::calibrer_mag(i2c) {
                        Ok(_) => log::info!(target: "diag", "Calibration mag MPU9250 OK"),
                        Err(e) => log::error!(target: "diag", "Calibration mag MPU9250 : {:?}", e),
                    }
                }
                "36" => {
                    #[cfg(target_os = "linux")]
                    let i2c = match airhaum::hal::I2cLinux::nouveau(0) {
                        Ok(bus) => bus,
                        Err(e) => { log::error!(target: "systeme", "Erreur I²C : {:?}", e); continue; }
                    };
                    #[cfg(not(target_os = "linux"))]
                    let i2c = airhaum::hal::i2c::I2cMock::nouveau();
                    match airhaum::diagnostiques::diag_mpu9250::diagnostic_complet(i2c) {
                        Ok(_) => log::info!(target: "diag", "Diagnostic MPU9250 OK"),
                        Err(e) => log::error!(target: "diag", "Diagnostic MPU9250 : {:?}", e),
                    }
                }
                "37" => {
                    #[cfg(target_os = "linux")]
                    let i2c = match airhaum::hal::I2cLinux::nouveau(0) {
                        Ok(bus) => bus,
                        Err(e) => { log::error!(target: "systeme", "Erreur I²C : {:?}", e); continue; }
                    };
                    #[cfg(not(target_os = "linux"))]
                    let i2c = airhaum::hal::i2c::I2cMock::nouveau();
                    match airhaum::diagnostiques::diag_mpu9250::test_frequence(i2c, 100) {
                        Ok(stats) => log::info!(target: "diag", "MPU9250 : {:.2} Hz (jitter ±{:.2} ms)", stats.hz_moyen, stats.jitter_ms),
                        Err(e) => log::error!(target: "diag", "Fréquence MPU9250 : {:?}", e),
                    }
                }
                "4" => {
                    match gps.as_ref() {
                        None => log::warn!(target: "gps", "GPS non disponible"),
                        Some(g) => {
                            let mesure = g.rx_gps.borrow();
                            println!("\n── État GPS actuel ──────────────────────────────────");
                            if let Some(d) = mesure.donnees.as_ref() {
                                let fix_str = match d.type_fix {
                                    airhaum::types::TypeFixGps::Aucun  => "Aucun",
                                    airhaum::types::TypeFixGps::Fix2D  => "2D",
                                    airhaum::types::TypeFixGps::Fix3D  => "3D",
                                    airhaum::types::TypeFixGps::GnssDr => "GNSS+DR",
                                };
                                println!("  Fix        : {}", fix_str);
                                println!("  Position   : {:.6}°N  {:.6}°E", d.latitude, d.longitude);
                                println!("  Altitude   : {:.1} m MSL", d.altitude_msl);
                                println!("  Vitesse    : {:.1} m/s  cap {:.1}°", d.vitesse_sol, d.cap);
                                println!("  Satellites : {}", d.nombre_satellites);
                                println!("  Précision  : H={:.1}m  V={:.1}m", d.precision_h, d.precision_v);
                            } else {
                                println!("  ⏳ Aucune trame reçue — attente du module GPS");
                            }
                        }
                    }
                }
                "41" => {
                    match gps.as_ref() {
                        None => log::warn!(target: "gps", "GPS non disponible"),
                        Some(g) => attendre_fix_gps(g).await,
                    }
                }
                "42" => {
                    match gps.as_ref() {
                        None => log::warn!(target: "gps", "GPS non disponible"),
                        Some(g) => afficher_gps(g).await,
                    }
                }
                "43" => {
                    match gps.as_ref() {
                        None => log::warn!(target: "gps", "GPS non disponible"),
                        Some(g) => sauvegarder_assistance_gps(g).await,
                    }
                }
                "5"  => afficher_etat_arduino(&servo),
                "51" => tester_debattements_servos(&servo),
                "s" => afficher_sante(&capteurs, gps.as_ref()),
                "a" => afficher_attitude(&mut estimation).await,
                "n" => afficher_navigation(&mut estimation).await,
                "h" => afficher_altitude(&mut estimation).await,
                "c" => {
                    if !afficher_checklist_armement(&estimation, gps.as_ref(), &servo) {
                        log::info!(target: "systeme", "Session terminée — arrêt opérateur");
                        println!("\nAu revoir !");
                        break;
                    }
                }
                "t" => {
                    println!("\n=== Test de tous les capteurs I²C ===\n");
                    match airhaum::diagnostiques::diag_bmp280::tester_bmp280() {
                        Ok(_) => log::info!(target: "diag", "BMP280 OK"),
                        Err(e) => log::error!(target: "diag", "BMP280 : {:?}", e),
                    }
                    #[cfg(target_os = "linux")]
                    let i2c = match airhaum::hal::I2cLinux::nouveau(0) {
                        Ok(bus) => bus,
                        Err(e) => { log::error!(target: "systeme", "Erreur I²C pour VL53L0X : {:?}", e); continue; }
                    };
                    #[cfg(not(target_os = "linux"))]
                    let i2c = airhaum::hal::i2c::I2cMock::nouveau();
                    match airhaum::diagnostiques::diag_vl53l0x::diagnostic_complet(i2c) {
                        Ok(_) => log::info!(target: "diag", "VL53L0X OK"),
                        Err(e) => log::error!(target: "diag", "VL53L0X : {:?}", e),
                    }
                    println!("\n=== Tests terminés ===");
                }
                "j" => afficher_journal(20),
                "q" | "Q" => {
                    log::info!(target: "systeme", "Session terminée — arrêt opérateur");
                    println!("\nAu revoir !");
                    break;
                }
                _ => println!("\n⚠ Choix invalide"),
            }

            println!("\nAppuyez sur Entrée pour continuer...");
            let mut pause = String::new();
            io::stdin().read_line(&mut pause).unwrap();
        }

        } else {
            log::info!(target: "systeme", "Session abandonnée — arrêt opérateur");
            println!("\nAu revoir !");
        }

        estimation.arreter();
    }

    log::info!(target: "systeme", "Système arrêté");
    servo.arreter();
    if let Some(g) = gps { g.arreter(); }
    capteurs.arreter();
}
