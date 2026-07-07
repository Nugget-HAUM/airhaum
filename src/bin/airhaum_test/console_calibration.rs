// src/bin/airhaum_test/console_calibration.rs
//! Console de calibration — bilan des calibrations avec recalibration interactive.
//!
//! Appelée en début de séquence sol, avant le lancement des drivers.
//! Retourne `true` pour continuer, `false` pour quitter le programme.

use std::io::{self, Write};

/// Affiche le bilan des calibrations et propose de recalibrer les capteurs.
///
/// Retourne `true` pour continuer vers la check-liste, `false` pour quitter.
pub fn afficher_bilan_calibrations() -> bool {
    use airhaum::systeme::calibration::{gestionnaire, EtatCalibration, CalibrationPersistante};
    use airhaum::drivers::imu::calibration::{CalibrationGyro, CalibrationAccel, CalibrationMag};
    use airhaum::drivers::barometre::calibration::CalibrationBarometre;
    use airhaum::drivers::telemetre::calibration::CalibrationTelemetre;

    fn fmt_duree(s: u64) -> String {
        if s == 0    { return "—".into(); }
        if s < 60    { return format!("{}s", s); }
        if s < 3600  { return format!("{}m", s / 60); }
        if s < 86400 { return format!("{}h {:02}m", s / 3600, (s % 3600) / 60); }
        format!("{}j {:02}h", s / 86400, (s % 86400) / 3600)
    }

    fn ligne<C: CalibrationPersistante>(nom: &str, etat: &EtatCalibration<C>) {
        let (statut, age, restant): (String, String, String) = match etat {
            EtatCalibration::Absente      => ("— Absente ".into(), "—".into(), "—".into()),
            EtatCalibration::Valide(c)    => ("✓ Valide  ".into(), fmt_duree(c.age_secondes()), fmt_duree(c.temps_restant_secondes())),
            EtatCalibration::Expiree(c)   => ("✗ Expirée ".into(), fmt_duree(c.age_secondes()), "—".into()),
            EtatCalibration::Corrompue(_) => ("✗ Corrompue".into(), "—".into(), "—".into()),
        };
        println!("  {:<20} {:<12} {:<14} {}", nom, statut, age, restant);
    }

    loop {
        let g = gestionnaire();
        let etat_gyro  = g.inspecter::<CalibrationGyro>();
        let etat_accel = g.inspecter::<CalibrationAccel>();
        let etat_mag   = g.inspecter::<CalibrationMag>();
        let etat_baro  = g.inspecter::<CalibrationBarometre>();
        let etat_telem = g.inspecter::<CalibrationTelemetre>();

        println!("\n── Bilan des calibrations ───────────────────────────────────────────");
        println!("  {:<20} {:<12} {:<14} {}", "Capteur", "Statut", "Âge", "Validité restante");
        println!("  {}", "─".repeat(62));
        ligne("Gyroscope",     &etat_gyro);
        ligne("Accéléromètre", &etat_accel);
        ligne("Magnétomètre",  &etat_mag);
        ligne("Baromètre",     &etat_baro);
        ligne("Télémètre",     &etat_telem);
        println!("  {}", "─".repeat(62));

        let gyro_ko  = matches!(etat_gyro,  EtatCalibration::Absente | EtatCalibration::Expiree(_));
        let accel_ko = matches!(etat_accel, EtatCalibration::Absente | EtatCalibration::Expiree(_));
        let mag_ko   = matches!(etat_mag,   EtatCalibration::Absente | EtatCalibration::Expiree(_));
        let baro_ko  = matches!(etat_baro,  EtatCalibration::Absente | EtatCalibration::Expiree(_));
        // Télémètre : calibration automatique à l'init du driver, pas d'action opérateur requise.
        let telem_ko = matches!(etat_telem, EtatCalibration::Absente | EtatCalibration::Expiree(_));

        let mut a_renouveler: Vec<&str> = Vec::new();
        if gyro_ko  { a_renouveler.push("gyroscope"); }
        if accel_ko { a_renouveler.push("accéléromètre"); }
        if mag_ko   { a_renouveler.push("magnétomètre"); }
        if baro_ko  { a_renouveler.push("baromètre"); }

        if a_renouveler.is_empty() && !telem_ko {
            println!("  ✓ Toutes les calibrations sont valides.");
        } else if a_renouveler.is_empty() {
            println!("  ✓ Calibrations valides.  (télémètre : remise à zéro automatique au démarrage)");
        } else {
            println!("  ⚠  À renouveler : {}", a_renouveler.join("  "));
            if telem_ko {
                println!("     Télémètre : remise à zéro automatique au démarrage.");
            }
        }

        println!();
        println!("  Recalibrer :  g. Gyroscope    a. Accéléromètre    m. Magnétomètre    b. Baromètre");
        println!("  [Entrée] Continuer → check-liste armement    j. Journal    q. Quitter");
        print!("  Votre choix : ");
        io::stdout().flush().unwrap();

        let mut choix = String::new();
        io::stdin().read_line(&mut choix).unwrap();

        match choix.trim() {
            "" => return true,
            "q" | "Q" => return false,

            "g" | "G" => {
                println!();
                #[cfg(target_os = "linux")]
                let i2c = match airhaum::hal::I2cLinux::nouveau(0) {
                    Ok(bus) => bus,
                    Err(e)  => { log::error!(target: "systeme", "Erreur I²C : {:?}", e); continue; }
                };
                #[cfg(not(target_os = "linux"))]
                let i2c = airhaum::hal::i2c::I2cMock::nouveau();
                match airhaum::diagnostiques::diag_mpu9250::calibrer_gyro(i2c) {
                    Ok(_)  => log::info!(target: "calibration", "Calibration gyroscope terminée"),
                    Err(e) => log::error!(target: "calibration", "Calibration gyroscope : {:?}", e),
                }
            }

            "a" | "A" => {
                println!();
                #[cfg(target_os = "linux")]
                let i2c = match airhaum::hal::I2cLinux::nouveau(0) {
                    Ok(bus) => bus,
                    Err(e)  => { log::error!(target: "systeme", "Erreur I²C : {:?}", e); continue; }
                };
                #[cfg(not(target_os = "linux"))]
                let i2c = airhaum::hal::i2c::I2cMock::nouveau();
                match airhaum::diagnostiques::diag_mpu9250::calibrer_accel(i2c) {
                    Ok(_)  => log::info!(target: "calibration", "Calibration accéléromètre terminée"),
                    Err(e) => log::error!(target: "calibration", "Calibration accéléromètre : {:?}", e),
                }
            }

            "m" | "M" => {
                println!();
                #[cfg(target_os = "linux")]
                let i2c = match airhaum::hal::I2cLinux::nouveau(0) {
                    Ok(bus) => bus,
                    Err(e)  => { log::error!(target: "systeme", "Erreur I²C : {:?}", e); continue; }
                };
                #[cfg(not(target_os = "linux"))]
                let i2c = airhaum::hal::i2c::I2cMock::nouveau();
                match airhaum::diagnostiques::diag_mpu9250::calibrer_mag(i2c) {
                    Ok(_)  => log::info!(target: "calibration", "Calibration magnétomètre terminée"),
                    Err(e) => log::error!(target: "calibration", "Calibration magnétomètre : {:?}", e),
                }
            }

            "b" | "B" => {
                println!();
                match airhaum::diagnostiques::diag_bmp280::calibrer_bmp280() {
                    Ok(_)  => log::info!(target: "calibration", "Calibration baromètre terminée"),
                    Err(e) => log::error!(target: "calibration", "Calibration baromètre : {:?}", e),
                }
            }

            "j" | "J" => super::console_test::afficher_journal(20),

            _ => println!("  Choix non reconnu."),
        }
    }
}
