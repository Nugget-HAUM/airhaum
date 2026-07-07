// src/bin/airhaum_test/console_armement.rs
//! Console de check-liste armement.
//!
//! Appelée automatiquement après le démarrage de l'estimation, et rappelable
//! depuis le menu (option `c`). Les items non encore implémentés sont affichés
//! comme neutres (—) et ne bloquent pas.

use std::io::{self, Write};
use std::sync::atomic::Ordering;
use airhaum::taches::taches_estimation::HandlesEstimation;
use airhaum::taches::taches_gps::HandlesGps;
use airhaum::taches::taches_servo::HandlesServo;

/// Présente la check-liste d'armement.
///
/// Retourne `true` pour continuer vers le menu, `false` pour quitter le programme.
pub fn afficher_checklist_armement(
    estimation: &HandlesEstimation,
    gps: Option<&HandlesGps>,
    servo: &HandlesServo,
) -> bool {
    use airhaum::systeme::calibration::{gestionnaire, EtatCalibration, CalibrationPersistante};
    use airhaum::drivers::imu::calibration::{CalibrationGyro, CalibrationAccel, CalibrationMag};
    use airhaum::drivers::barometre::calibration::CalibrationBarometre;
    use airhaum::types::TypeFixGps;

    fn duree(s: u64) -> String {
        if s < 60    { return format!("{}s", s); }
        if s < 3600  { return format!("{}m", s / 60); }
        format!("{}h{:02}m", s / 3600, (s % 3600) / 60)
    }

    fn ligne(sym: &str, nom: &str, etat: &str, detail: &str, suffixe: &str) {
        println!("  {}  {:<34}  {:<8}  {}{}", sym, nom, etat, detail, suffixe);
    }

    let mut acquitte_gps = false;
    let mut acquitte_mag = false;
    let mut prevol_confirme = false;

    loop {
        print!("\x1B[2J\x1B[1;1H");
        io::stdout().flush().unwrap();

        println!("╔══════════════════════════════════════════════════════════╗");
        println!("║     CHECK-LISTE ARMEMENT — AirHaum II                   ║");
        println!("╚══════════════════════════════════════════════════════════╝\n");

        let g = gestionnaire();

        // ── Bloquants ─────────────────────────────────────────────────────────
        println!("  {:<36}  {:<8}  {}", "BLOQUANTS", "État", "Détail");
        println!("  {}", "─".repeat(66));

        macro_rules! calib {
            ($T:ty) => {
                match g.inspecter::<$T>() {
                    EtatCalibration::Valide(c)    => (true,  format!("{} restantes", duree(c.temps_restant_secondes()))),
                    EtatCalibration::Expiree(c)   => (false, format!("expirée il y a {}", duree(c.age_secondes()))),
                    EtatCalibration::Absente      => (false, "absente".to_string()),
                    EtatCalibration::Corrompue(_) => (false, "corrompue".to_string()),
                }
            }
        }

        let (gyro_ok,  gyro_detail)  = calib!(CalibrationGyro);
        let (accel_ok, accel_detail) = calib!(CalibrationAccel);
        let (baro_ok,  baro_detail)  = calib!(CalibrationBarometre);

        let estim_ok = estimation.rx_navigation.has_changed().is_ok();
        let estim_detail = if estim_ok {
            let nav = estimation.rx_navigation.borrow();
            format!("roulis={:+.1}°  tangage={:+.1}°",
                nav.attitude.roulis.degres(), nav.attitude.tangage.degres())
        } else {
            "inactif".to_string()
        };

        for (ok, nom, detail) in [
            (gyro_ok,  "Calibration gyroscope",     gyro_detail.as_str()),
            (accel_ok, "Calibration accéléromètre", accel_detail.as_str()),
            (baro_ok,  "Calibration baromètre",      baro_detail.as_str()),
            (estim_ok, "Estimation active (EKF)",    estim_detail.as_str()),
        ] {
            ligne(
                if ok { "✓" } else { "✗" },
                nom,
                if ok { "OK" } else { "KO" },
                detail,
                if ok { "" } else { "  ← BLOQUANT" },
            );
        }

        for nom in ["Plan de vol chargé", "Sûreté nominale"] {
            ligne("—", nom, "—", "(non implémenté)", "");
        }

        ligne(
            if prevol_confirme { "✓" } else { "✗" },
            "Vérification pré-vol servos",
            if prevol_confirme { "OK" } else { "KO" },
            if prevol_confirme {
                "débattements confirmés par opérateur"
            } else {
                "p. Confirmer après vérification visuelle  ← BLOQUANT"
            },
            "",
        );

        let (arduino_ok, arduino_detail) = if let Some(ref err) = servo.erreur_port {
            (false, format!("port série inaccessible — {}", err))
        } else {
            let liaison = servo.liaison_detectee.load(Ordering::Relaxed);
            let etat = servo.rx_etat.borrow();
            let rc_ok = !etat.rc_perdu;
            match (liaison, rc_ok) {
                (false, _) => (false, "aucune trame reçue — Nano absent ?".to_string()),
                (true, false) => (false, "liaison OK  RC perdu (émetteur éteint ?)".to_string()),
                (true, true) => {
                    let mode = match etat.mode {
                        airhaum::drivers::controleur_servo::ModeArduino::Autopilote => "autopilote",
                        airhaum::drivers::controleur_servo::ModeArduino::Manuel => "manuel",
                    };
                    (true, format!("liaison OK  RC OK  mode={}", mode))
                }
            }
        };
        ligne(
            if arduino_ok { "✓" } else { "✗" },
            "Liaison Arduino + RC",
            if arduino_ok { "OK" } else { "KO" },
            &arduino_detail,
            if arduino_ok { "" } else { "  ← BLOQUANT" },
        );

        // ── Avertissements ────────────────────────────────────────────────────
        println!("\n  {:<36}  {:<8}  {}", "AVERTISSEMENTS", "", "");
        println!("  {}", "─".repeat(66));

        let (gps_ok, gps_detail) = match gps {
            None => (false, "module absent".to_string()),
            Some(h) => {
                let m = h.rx_gps.borrow();
                match m.donnees.as_ref() {
                    None => (false, "aucune trame".to_string()),
                    Some(d) => {
                        let ok = m.valide && d.nombre_satellites >= 6 && d.precision_h <= 5.0;
                        let fix_str = match d.type_fix {
                            TypeFixGps::Aucun  => "aucun",
                            TypeFixGps::Fix2D  => "2D",
                            TypeFixGps::Fix3D  => "3D",
                            TypeFixGps::GnssDr => "GNSS+DR",
                        };
                        (ok, format!("{}  {} sats  H={:.1}m", fix_str, d.nombre_satellites, d.precision_h))
                    }
                }
            }
        };
        let (mag_ok, mag_detail) = calib!(CalibrationMag);

        for (ok, acquitte, nom, detail, touche) in [
            (gps_ok, acquitte_gps, "GPS fix 3D",               gps_detail.as_str(), "g"),
            (mag_ok, acquitte_mag, "Calibration magnétomètre", mag_detail.as_str(), "m"),
        ] {
            let detail_fmt = if !ok && !acquitte {
                format!("{}   {touche}. Acquitter", detail)
            } else if acquitte && !ok {
                format!("{} (acquitté)", detail)
            } else {
                detail.to_string()
            };
            ligne(if ok { "✓" } else { "⚠" }, nom, if ok { "OK" } else { "KO" }, &detail_fmt, "");
        }

        // ── Résultat ──────────────────────────────────────────────────────────
        println!("  {}", "─".repeat(66));

        let bloquants_ok = gyro_ok && accel_ok && baro_ok && estim_ok && arduino_ok && prevol_confirme;
        let warnings_ok  = (gps_ok || acquitte_gps) && (mag_ok || acquitte_mag);

        println!();
        if bloquants_ok && warnings_ok {
            println!("  ✓ Système prêt pour l'armement.");
        } else if !bloquants_ok {
            println!("  ✗ Armement impossible — items bloquants non satisfaits.");
        } else {
            println!("  ⚠  Acquittez les avertissements actifs pour continuer.");
        }

        println!();
        println!("  Acquitter :  g. GPS    m. Magnétomètre    p. Vérification pré-vol");
        println!("  j. Journal    [Entrée] Continuer vers le menu    q. Quitter le programme");
        print!("\n  Votre choix : ");
        io::stdout().flush().unwrap();

        let mut input = String::new();
        io::stdin().read_line(&mut input).unwrap();
        match input.trim() {
            "" => return true,
            "q" | "Q" => return false,
            "g" | "G" if !gps_ok && !acquitte_gps => { acquitte_gps = true; }
            "m" | "M" if !mag_ok && !acquitte_mag => { acquitte_mag = true; }
            "p" | "P" if !prevol_confirme => { prevol_confirme = true; }
            "j" | "J" => super::console_test::afficher_journal(20),
            _ => {}
        }
    }
}
