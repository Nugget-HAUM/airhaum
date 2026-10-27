// src/bin/airhaum_test/console_armement.rs
//! Console de check-liste armement.
//!
//! Appelée automatiquement après le démarrage de l'estimation, et rappelable
//! depuis le menu (option `c`). Les items non encore implémentés sont affichés
//! comme neutres (—) et ne bloquent pas.

use std::fs;
use std::io::{self, Write};
use std::sync::atomic::Ordering;
use std::time::Duration;
use tokio::sync::watch;
use airhaum::mission::armement::SequencePrevol;
use airhaum::mission::etat_machine::{CommandeVol, EtatVol};
use airhaum::mission::plan_vol::{repertoire_plans_de_vol, PlanDeVol};
use airhaum::taches::taches_estimation::HandlesEstimation;
use airhaum::taches::taches_gps::HandlesGps;
use airhaum::taches::taches_mission::HandlesMission;
use airhaum::taches::taches_servo::HandlesServo;
use airhaum::taches::taches_surete::HandlesSurete;

/// Présente la liste numérotée des fichiers `.toml` du dossier plans de vol
/// (voir `repertoire_plans_de_vol`) et charge celui choisi par l'opérateur.
///
/// Ne modifie `plan_charge` qu'en cas de succès — un échec de chargement
/// (TOML malformé, plan sans segment, dossier inaccessible) est simplement
/// affiché puis on revient à la check-liste, item toujours à ✗. Dépose
/// aussi le plan dans `tx_plan` (`HandlesMission::tx_plan`), à destination
/// de la tâche mission — voir `doc/mission.md`, « Injection du plan de vol ».
fn selectionner_plan_de_vol(
    plan_charge: &mut Option<(String, PlanDeVol)>,
    tx_plan: &watch::Sender<Option<PlanDeVol>>,
) {
    let repertoire = repertoire_plans_de_vol();
    let entrees = match fs::read_dir(&repertoire) {
        Err(e) => {
            println!("\n  Répertoire de plans de vol inaccessible ({}) : {}", repertoire, e);
            attendre_entree();
            return;
        }
        Ok(e) => e,
    };

    let mut fichiers: Vec<_> = entrees
        .filter_map(|e| e.ok())
        .filter(|e| e.path().extension().map(|ext| ext == "toml").unwrap_or(false))
        .collect();
    fichiers.sort_by_key(|e| e.file_name());

    if fichiers.is_empty() {
        println!("\n  Aucun fichier .toml trouvé dans {}", repertoire);
        attendre_entree();
        return;
    }

    println!("\n── Plans de vol disponibles — {} ──────────────────────", repertoire);
    for (i, f) in fichiers.iter().enumerate() {
        println!("  {}. {}", i + 1, f.file_name().to_string_lossy());
    }
    print!("\n  Numéro à charger (Entrée pour annuler) : ");
    io::stdout().flush().unwrap();

    let mut choix = String::new();
    io::stdin().read_line(&mut choix).unwrap();
    let choix = choix.trim();
    if choix.is_empty() {
        return;
    }

    let indice = match choix.parse::<usize>() {
        Ok(n) if n >= 1 && n <= fichiers.len() => n - 1,
        _ => { println!("\n  Choix invalide."); attendre_entree(); return; }
    };

    let entree = &fichiers[indice];
    match PlanDeVol::charger(&entree.path()) {
        Ok(plan) => {
            println!("\n  ✓ Plan de vol chargé : {} ({} segment(s))",
                entree.file_name().to_string_lossy(), plan.nombre_segments());
            let _ = tx_plan.send(Some(plan.clone()));
            *plan_charge = Some((entree.file_name().to_string_lossy().into_owned(), plan));
        }
        Err(e) => println!("\n  ✗ Échec du chargement : {}", e),
    }
    attendre_entree();
}

fn attendre_entree() {
    println!("\nAppuyez sur Entrée pour continuer...");
    let mut pause = String::new();
    io::stdin().read_line(&mut pause).unwrap();
}

/// Attend une mise à jour de l'état vol (jusqu'à 500 ms) après l'envoi d'une
/// commande — la tâche mission ne vide son canal de commandes qu'à son
/// propre cycle (10 Hz, `doc/mission.md`) : on laisse le temps au prochain
/// redessin de refléter le nouvel état plutôt que l'ancien.
async fn attendre_changement_etat(rx: &mut watch::Receiver<EtatVol>) {
    let _ = tokio::time::timeout(Duration::from_millis(500), rx.changed()).await;
}

/// Présente la check-liste d'armement puis, une fois armée, l'écran
/// d'opérations en vol (état courant, désarmement) — voir `doc/mission.md`,
/// « Actions armer/lancer/désarmer ».
///
/// Retourne `true` pour continuer vers le menu de test, `false` pour
/// mettre fin au programme (opérateur, ou mission menée à son terme
/// jusqu'à `SauvegardeDonnées`).
pub async fn afficher_checklist_armement(
    estimation: &HandlesEstimation,
    gps: Option<&HandlesGps>,
    servo: &HandlesServo,
    surete: &HandlesSurete,
    sequence_prevol: &mut SequencePrevol,
    plan_charge: &mut Option<(String, PlanDeVol)>,
    handles_mission: &HandlesMission,
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
    let mut rx_etat_vol = handles_mission.rx_etat_vol.clone();

    loop {
        let etat_vol = rx_etat_vol.borrow_and_update().clone();

        print!("\x1B[2J\x1B[1;1H");
        io::stdout().flush().unwrap();

        if etat_vol == EtatVol::SauvegardeDonnees {
            println!("╔══════════════════════════════════════════════════════════╗");
            println!("║   MISSION TERMINÉE — désarmé, données sauvegardées      ║");
            println!("╚══════════════════════════════════════════════════════════╝\n");
            return false;
        }

        println!("╔══════════════════════════════════════════════════════════╗");
        println!("║     CHECK-LISTE ARMEMENT — AirHaum II                   ║");
        println!("╚══════════════════════════════════════════════════════════╝\n");
        println!("  MAÉ vol : {}\n", etat_vol);

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
            (gyro_ok,  "Calibration session (gyro+Z)", gyro_detail.as_str()),
            (accel_ok, "Calibration montage accél.",  accel_detail.as_str()),
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

        let plan_ok = plan_charge.is_some();
        let plan_detail = match plan_charge.as_ref() {
            Some((nom, plan)) => format!("{} ({} segment(s))", nom, plan.nombre_segments()),
            None => "non chargé — v. Charger un plan de vol".to_string(),
        };
        ligne(
            if plan_ok { "✓" } else { "✗" },
            "Plan de vol chargé",
            if plan_ok { "OK" } else { "KO" },
            &plan_detail,
            if plan_ok { "" } else { "  ← BLOQUANT" },
        );

        let etat_securite = surete.rx_securite.borrow().clone();
        let surete_ok = airhaum::surete::armement::surete_nominale(&etat_securite);
        ligne(
            if surete_ok { "✓" } else { "✗" },
            "Sûreté nominale",
            if surete_ok { "OK" } else { "KO" },
            &etat_securite.to_string(),
            if surete_ok { "" } else { "  ← BLOQUANT" },
        );

        let prevol_confirme = sequence_prevol.complete();
        // Fixup compilation (E0716) : le format! doit être lié à une variable pour
        // survivre jusqu'à l'appel à ligne() — équipe sûreté/armement, à revoir si
        // ce bloc est retouché.
        let detail_prevol = if prevol_confirme {
            "3/3 surfaces testées (min/max/neutre)".to_string()
        } else {
            format!(
                "{}/3 surfaces testées — p. Lancer le test  ← BLOQUANT",
                sequence_prevol.nombre_testees()
            )
        };
        ligne(
            if prevol_confirme { "✓" } else { "✗" },
            "Vérification pré-vol servos",
            if prevol_confirme { "OK" } else { "KO" },
            &detail_prevol,
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

        let bloquants_ok = gyro_ok && accel_ok && baro_ok && estim_ok && arduino_ok && prevol_confirme && surete_ok && plan_ok;
        let warnings_ok  = (gps_ok || acquitte_gps) && (mag_ok || acquitte_mag);

        println!();
        if bloquants_ok && warnings_ok {
            println!("  ✓ Système prêt pour l'armement.");
        } else if !bloquants_ok {
            println!("  ✗ Armement impossible — items bloquants non satisfaits.");
        } else {
            println!("  ⚠  Acquittez les avertissements actifs pour continuer.");
        }

        let peut_armer = etat_vol == EtatVol::AttenteArmement && bloquants_ok && warnings_ok;
        let peut_lancer = etat_vol == EtatVol::Armement;
        let peut_desarmer = etat_vol == EtatVol::RoulageAuSol;

        println!();
        println!("  Acquitter :  g. GPS    m. Magnétomètre    p. Vérification pré-vol    v. Plan de vol");
        if peut_armer   { println!("  a. Armer"); }
        if peut_lancer  { println!("  l. Lancer la mission"); }
        if peut_desarmer { println!("  d. Désarmer"); }
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
            "p" | "P" if !prevol_confirme => {
                super::console_test::tester_debattements_servos(servo, sequence_prevol);
            }
            "v" | "V" => selectionner_plan_de_vol(plan_charge, &handles_mission.tx_plan),
            "a" | "A" if peut_armer => {
                let _ = handles_mission.tx_commande.send(CommandeVol::Armer).await;
                attendre_changement_etat(&mut rx_etat_vol).await;
            }
            "l" | "L" if peut_lancer => {
                let _ = handles_mission.tx_commande.send(CommandeVol::ArmementValide).await;
                attendre_changement_etat(&mut rx_etat_vol).await;
            }
            "d" | "D" if peut_desarmer => {
                let _ = handles_mission.tx_commande.send(CommandeVol::Desarmer).await;
                attendre_changement_etat(&mut rx_etat_vol).await;
                // Pas de confirmation actionneurs réelle aujourd'hui (v1,
                // aucun module ne l'émet) — enchaînée directement, voir
                // doc/mission.md, « Désarmement ».
                let _ = handles_mission.tx_commande.send(CommandeVol::ActionneursDesarmes).await;
                attendre_changement_etat(&mut rx_etat_vol).await;
            }
            "j" | "J" => super::console_test::afficher_journal(20),
            _ => {}
        }
    }
}
