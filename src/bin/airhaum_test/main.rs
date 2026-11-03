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
use airhaum::mission::etat_machine::CommandeVol;
use airhaum::mission::plan_vol::PlanDeVol;
use airhaum::taches::taches_gps::HandlesGps;

use console_calibration::afficher_bilan_calibrations;
use console_armement::afficher_checklist_armement;
use console_test::{
    afficher_menu, afficher_sante,
    afficher_attitude, afficher_navigation, afficher_altitude,
    attendre_fix_gps, afficher_gps, sauvegarder_assistance_gps, campagne_statique_gps,
    afficher_etat_arduino, tester_debattements_servos, campagne_arduino, afficher_journal,
    afficher_serie_telemetre,
};

#[tokio::main]
async fn main() {
    // Le handle doit rester en vie jusqu'à la fin de main.
    // Son Drop envoie le signal d'arrêt au thread écrivain et attend le flush.
    let _journal = airhaum::systeme::journalisation::initialiser()
        .expect("Impossible d'initialiser la journalisation");

    // ── Étape 1 : Initialisation matérielle ──────────────────────────────────
    log::info!(target: "systeme", "Lancement AirHaum II v{}", airhaum::VERSION);

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

    log::info!(target: "systeme", "Bus I²C OK");

    // ── GPS : lancement du thread de réception ────────────────────────────────
    // Lancé tôt pour maximiser les chances d'avoir un fix GPS au moment où
    // l'opérateur valide les calibrations et passe au menu.
    #[cfg(target_os = "linux")]
    let mut gps: Option<HandlesGps> = {
        use airhaum::hal::uart_linux::PORT_GPS_DEFAUT;
        match airhaum::taches::taches_gps::lancer_gps(PORT_GPS_DEFAUT) {
            Ok(h)  => Some(h),
            Err(e) => { log::warn!(target: "gps", "GPS non disponible : {:?}", e); None }
        }
    };
    #[cfg(not(target_os = "linux"))]
    let mut gps: Option<HandlesGps> = None;

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
        if !afficher_bilan_calibrations(&mut gps) {
            log::info!(target: "systeme", "Session abandonnée à l'initialisation — arrêt opérateur");
            println!("\nAu revoir !");
            return;
        }
    }

    // ── Étape 2 : Lancement et vérification des capteurs ─────────────────────
    let mut capteurs = airhaum::taches::taches_capteurs::lancer_avec_bus(bus);

    if reprise_rapide {
        // Branchement MAÉ vol non implémenté pour ce chemin — voir
        // `doc/mission.md` (reprise rapide reste un cas ouvert).
        log::info!(target: "mission", "Reprise rapide détectée");
    } else {
        log::info!(target: "mission", "Threads capteurs lancés");

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
            rx_gps_estimation.clone(),
            pression_ref,
        );

        // ── Contrôle : lancement de la tâche (couche 4) ───────────────────────
        // Lancée dès que ses dépendances existent (rx_navigation/rx_taux_corps
        // de l'estimation, tx_consignes du thread servo, tous deux déjà prêts
        // à ce stade) — même principe que les autres tâches, aucune attente
        // supplémentaire pour l'opérateur.
        //
        // Gains PROVISOIRES, non réglés : « aucune valeur a priori » tant
        // qu'un essai réel n'a pas eu lieu (doc/controle.md, « Points
        // ouverts »). Seul `controleur_direction_sol` est exercé pour
        // l'instant (phase Sol, E0/E1 du plan d'essais) ; roulis/tangage/
        // lacet ne seront sollicités qu'à partir de la phase Vol (E3+) — leurs
        // gains n'ont donc aucune conséquence avant cette échéance.
        let controleur_direction_sol = airhaum::controle::ControleurDirectionSol::nouveau(1.0, 0.0, 0.0, 1.0);
        let controleur_roulis  = airhaum::controle::ControleurRoulis::nouveau(1.0, 1.0, 0.0, 0.0, 1.0);
        let controleur_tangage = airhaum::controle::ControleurTangage::nouveau(1.0, 1.0, 0.0, 0.0, 1.0);
        let controleur_lacet   = airhaum::controle::ControleurLacet::nouveau(1.0, 0.0, 0.0, 1.0);

        let controle = airhaum::taches::taches_controle::lancer_controle(
            estimation.rx_navigation.clone(),
            estimation.rx_taux_corps.clone(),
            servo.tx_consignes.clone(),
            controleur_direction_sol, controleur_roulis, controleur_tangage, controleur_lacet,
        );

        // ── Sûreté : lancement de la tâche de supervision ─────────────────────
        // LimitesVol::default() désactive géofence/plafond tant que les
        // valeurs réelles du terrain de vol ne sont pas configurées.
        //
        // arret_urgence_console : petit plus pour les premiers vols
        // rapprochés (canal de sécurité principal = RC/Nano, voir menu "u").
        //
        // Lancée avant l'attente de la première attitude (juste en dessous) :
        // `estimation.rx_attitude` existe dès la construction de la tâche
        // d'estimation, pas besoin d'attendre sa première valeur.
        let arret_urgence_console = std::sync::Arc::new(std::sync::atomic::AtomicBool::new(false));
        // État armé de la mission, publié par la tâche mission (lancée juste
        // après) et lu par la sûreté : SWA n'a d'effet sur la sécurité qu'armé.
        let (tx_mission_armee, rx_mission_armee) = tokio::sync::watch::channel(false);
        let surete = airhaum::taches::taches_surete::lancer_surete(
            capteurs.rx_baro.clone(),
            capteurs.rx_telem.clone(),
            estimation.rx_attitude.clone(),
            estimation.rx_navigation.clone(),
            rx_gps_estimation,
            servo.rx_etat.clone(),
            rx_mission_armee,
            airhaum::surete::limites::LimitesVol::default(),
            std::sync::Arc::clone(&arret_urgence_console),
        );

        // ── Mission : lancement de la tâche (MAÉ vol + plan de vol) ──────────
        // Lancée dès que ses dépendances existent (rx_securite, rx_navigation,
        // rx_altitude ci-dessus) — aucune attente supplémentaire pour
        // l'opérateur. `DriversInitialises`/`ConfigurationTerminee` sont
        // envoyées immédiatement : drivers et capteurs sont déjà prêts à ce
        // stade. `estimation_prête` (AutoTest → AttenteArmement) est ensuite
        // dérivée en continu par la tâche elle-même depuis `rx_navigation` —
        // plus besoin d'un tick manuel ponctuel ici. Le plan de vol n'est pas
        // encore connu (choisi plus bas, dans la check-liste) : voir
        // `HandlesMission::tx_plan`, `doc/mission.md`. Les quatre canaux
        // `tx_phase`/`tx_consigne_direction_sol`/`tx_consigne_gaz`/`tx_arme`
        // sont des clones de ceux de `controle` ci-dessus (`HandlesControle`),
        // qui en reste le propriétaire — la mission n'obtient qu'un droit
        // d'écriture, voir `doc/mission.md`.
        let commande_operateur_segment = std::sync::Arc::new(std::sync::atomic::AtomicBool::new(false));
        // Provisoire — configuration de mission réelle hors périmètre de
        // cette brique (voir doc/mission.md).
        const ALTITUDE_CIBLE_M: f32 = 100.0;

        let handles_mission = airhaum::taches::taches_mission::lancer_mission(
            surete.rx_securite.clone(),
            estimation.rx_navigation.clone(),
            estimation.rx_altitude.clone(),
            controle.tx_phase.clone(), controle.tx_consigne_direction_sol.clone(),
            controle.tx_consigne_gaz.clone(), controle.tx_arme.clone(), tx_mission_armee,
            ALTITUDE_CIBLE_M, std::sync::Arc::clone(&commande_operateur_segment),
        );
        let _ = handles_mission.tx_commande.send(CommandeVol::DriversInitialises).await;
        let _ = handles_mission.tx_commande.send(CommandeVol::ConfigurationTerminee).await;

        // Attente de la première attitude (timeout 5s) — diagnostic seul :
        // la MAÉ vol dérive elle-même `estimation_prête` en continu.
        match tokio::time::timeout(
            std::time::Duration::from_secs(5),
            estimation.rx_attitude.changed(),
        ).await {
            Ok(Ok(_)) => log::info!(target: "mission", "Estimation active"),
            _ => log::error!(target: "systeme", "Timeout : thread d'estimation non démarré"),
        }

        // ── Check-liste armement ──────────────────────────────────────────────
        let mut sequence_prevol = airhaum::mission::armement::SequencePrevol::nouvelle();
        let mut plan_charge: Option<(String, PlanDeVol)> = None;
        if afficher_checklist_armement(
            &estimation, gps.as_ref(), &servo, &controle, &surete,
            &mut sequence_prevol, &mut plan_charge, &handles_mission,
        ).await {

        // ── Menu de test ──────────────────────────────────────────────────────
        loop {
            afficher_menu(&handles_mission.rx_etat_vol.borrow().clone());

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
                "44" => {
                    match gps.as_ref() {
                        None => log::warn!(target: "gps", "GPS non disponible"),
                        Some(g) => campagne_statique_gps(g).await,
                    }
                }
                "5"  => afficher_etat_arduino(&servo),
                "51" => {
                    let etat_vol = handles_mission.rx_etat_vol.borrow().clone();
                    tester_debattements_servos(&servo, &controle, &etat_vol, &mut sequence_prevol);
                }
                "52" => campagne_arduino(&servo).await,
                "6" => {
                    match airhaum::diagnostiques::test_communication_lora() {
                        Ok(_) => log::info!(target: "diag", "Communication LoRa (SX1276) OK"),
                        Err(e) => log::error!(target: "diag", "Communication LoRa : {:?}", e),
                    }
                }
                "61" => {
                    match airhaum::diagnostiques::envoyer_message_test("Hello LoRa!") {
                        Ok(_) => log::info!(target: "diag", "Envoi LoRa terminé (TX_DONE reçu)"),
                        Err(e) => log::error!(target: "diag", "Envoi LoRa : {:?}", e),
                    }
                }
                "s" => afficher_sante(&capteurs, gps.as_ref()),
                "a" => afficher_attitude(&mut estimation).await,
                "n" => afficher_navigation(&mut estimation).await,
                "h" => afficher_altitude(&mut estimation).await,
                "c" => {
                    if !afficher_checklist_armement(
                        &estimation, gps.as_ref(), &servo, &controle, &surete,
                        &mut sequence_prevol, &mut plan_charge, &handles_mission,
                    ).await {
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
                "u" | "U" => {
                    arret_urgence_console.store(true, std::sync::atomic::Ordering::Relaxed);
                    let _ = servo.tx_consignes.send(airhaum::drivers::controleur_servo::ConsignesServos::neutre());
                    log::error!(target: "surete", "Arrêt d'urgence déclenché depuis la console");
                    println!("\n⚠ ARRÊT D'URGENCE — consigne neutre envoyée (gaz coupés, non armé).");
                    println!("  Ce n'est qu'un appoint : le canal de sécurité principal reste le RC/Nano.");
                }
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

        handles_mission.arreter();
        controle.arreter();
        estimation.arreter();
        surete.arreter();
    }

    log::info!(target: "systeme", "Système arrêté");
    servo.arreter();
    if let Some(g) = gps { g.arreter(); }
    capteurs.arreter();
}
