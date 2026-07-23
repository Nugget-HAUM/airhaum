// src/bin/airhaum_test/console_calibration.rs
//! Console de calibration — bilan des calibrations avec recalibration interactive.
//!
//! Appelée en début de séquence sol, avant le lancement des drivers.
//! Retourne `true` pour continuer, `false` pour quitter le programme.

use std::io::{self, Write};
use airhaum::taches::taches_gps::HandlesGps;

/// Affiche le bilan des calibrations et propose de recalibrer les capteurs.
///
/// `gps` : handle du thread GPS déjà lancé (voir `main.rs` — lancé tôt, avant
/// cette console, pour maximiser le temps d'acquisition). Passé en référence
/// mutable car le menu « Terrain » peut le tuer et le relancer si l'opérateur
/// change de terrain actif (voir [`relancer_gps`]).
///
/// Retourne `true` pour continuer vers la check-liste, `false` pour quitter.
pub fn afficher_bilan_calibrations(gps: &mut Option<HandlesGps>) -> bool {
    use airhaum::systeme::calibration::{gestionnaire, EtatCalibration, CalibrationPersistante};
    use airhaum::drivers::imu::calibration::{CalibrationGyro, CalibrationAccel, CalibrationMag};
    use airhaum::drivers::barometre::calibration::CalibrationBarometre;
    use airhaum::drivers::telemetre::calibration::CalibrationTelemetre;
    use airhaum::drivers::gps::RegistreTerrains;

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

        // Terrain GPS — statut à part : ce n'est pas une calibration qui expire,
        // l'information utile est « lequel est actif », pas un âge de validité.
        let registre = g.charger::<RegistreTerrains>().ok().flatten();
        let (statut_terrain, age_terrain, info_terrain): (String, String, String) = match &registre {
            None => ("— Absent   ".into(), "—".into(), "aucun terrain enregistré".into()),
            Some(r) => match r.entree_active() {
                Some(t) => ("✓ Actif    ".into(), fmt_duree(t.age_secondes()),
                    format!("« {} »  ({} connu(s))", t.nom, r.terrains.len())),
                None => ("⚠ Aucun actif".into(), "—".into(),
                    format!("{} terrain(s) enregistré(s), aucun sélectionné", r.terrains.len())),
            },
        };
        println!("  {:<20} {:<12} {:<14} {}", "GPS (terrain)", statut_terrain, age_terrain, info_terrain);

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
        println!("  t. Terrain de vol (assistance GPS)");
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

            "t" | "T" => menu_terrain_gps(gps),

            "j" | "J" => super::console_test::afficher_journal(20),

            _ => println!("  Choix non reconnu."),
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Sélection du terrain de vol actif (assistance GPS)
// ─────────────────────────────────────────────────────────────────────────────

/// Affiche les terrains connus (registre `RegistreTerrains`), le fix GPS
/// courant pour comparaison visuelle, et laisse l'opérateur choisir le
/// terrain actif. En cas de changement, relance le thread GPS pour que la
/// nouvelle sélection soit injectée immédiatement (voir [`relancer_gps`]).
///
/// N'ajoute pas de terrain : le registre est pensé pour être édité à la main
/// (voir l'en-tête généré par `RegistreTerrains::vers_toml`) — ce menu ne fait
/// que choisir parmi les terrains déjà déclarés dans le fichier.
fn menu_terrain_gps(gps: &mut Option<HandlesGps>) {
    use airhaum::systeme::calibration::{gestionnaire, CalibrationPersistante};
    use airhaum::drivers::gps::RegistreTerrains;

    let registre = match gestionnaire().charger::<RegistreTerrains>() {
        Ok(Some(r)) => r,
        Ok(None) => {
            println!("\n  Aucun registre de terrains — créez le fichier \
                       calibration_{}.toml dans le répertoire de calibration \
                       (voir l'en-tête généré par `RegistreTerrains::vers_toml` pour le format).",
                RegistreTerrains::identifiant_capteur());
            return;
        }
        Err(e) => {
            println!("\n  ⚠ Lecture du registre de terrains impossible : {:?}", e);
            return;
        }
    };

    if registre.terrains.is_empty() {
        println!("\n  Registre de terrains présent mais vide — éditez le fichier pour en ajouter.");
        return;
    }

    println!("\n── Terrain de vol (assistance GPS) ──────────────────────────────");
    if let Some(g) = gps.as_ref() {
        let mesure = g.rx_gps.borrow();
        match mesure.donnees.as_ref() {
            Some(d) => println!("  GPS actuel : {:.5}°  {:.5}°  {} sats  H={:.1}m",
                d.latitude, d.longitude, d.nombre_satellites, d.precision_h),
            None => println!("  GPS actuel : aucune position reçue pour l'instant"),
        }
    } else {
        println!("  GPS non disponible — la sélection prendra effet au prochain démarrage.");
    }
    println!();

    for (i, t) in registre.terrains.iter().enumerate() {
        let actif = registre.terrain_actif.as_deref() == Some(t.nom.as_str());
        println!("  {:2}) {} {:<24} {:>10.5}°  {:>10.5}°   orbites={}o",
            i + 1, if actif { "*" } else { " " }, t.nom,
            t.latitude, t.longitude, t.taille_orbites());
    }
    println!();
    print!("  Choix (numéro, [Entrée] pour annuler) : ");
    io::stdout().flush().unwrap();

    let mut choix = String::new();
    io::stdin().read_line(&mut choix).unwrap();
    let choix = choix.trim();
    if choix.is_empty() {
        return;
    }

    let index: usize = match choix.parse::<usize>() {
        Ok(n) if n >= 1 && n <= registre.terrains.len() => n - 1,
        _ => { println!("  Choix invalide."); return; }
    };
    let nom = registre.terrains[index].nom.clone();

    let mut registre = registre;
    registre.terrain_actif = Some(nom.clone());
    match gestionnaire().sauvegarder(&registre) {
        Ok(())  => log::info!(target: "gps", "Terrain actif changé : « {} »", nom),
        Err(e)  => { println!("  ⚠ Sauvegarde du registre échouée : {:?}", e); return; }
    }

    if gps.is_some() {
        println!("  Relance du GPS avec le terrain « {} »…", nom);
        relancer_gps(gps);
    } else {
        println!("  Terrain « {} » sélectionné — sera injecté au prochain démarrage du GPS.", nom);
    }
}

/// Arrête proprement le thread GPS en cours (s'il existe) et en relance un
/// nouveau — repasse par `DriverGps::initialiser()` en entier, donc relit le
/// terrain actif à jour et réinjecte la bonne assistance. Plus simple et plus
/// prévisible qu'un rappel ciblé de `importer_assistance()` sur un module déjà
/// en cours d'acquisition (comportement mal défini côté firmware).
fn relancer_gps(gps: &mut Option<HandlesGps>) {
    if let Some(g) = gps.take() {
        g.arreter();
        let _ = g.tache.join();
    }

    #[cfg(target_os = "linux")]
    {
        use airhaum::hal::uart_linux::PORT_GPS_DEFAUT;
        match airhaum::taches::taches_gps::lancer_gps(PORT_GPS_DEFAUT) {
            Ok(h)  => { *gps = Some(h); log::info!(target: "gps", "GPS relancé"); }
            Err(e) => log::warn!(target: "gps", "Relance GPS échouée : {:?}", e),
        }
    }
}
