// src/diagnostiques/diag_mpu9250.rs
//! Fonctions de diagnostic pour le MPU9250

use crate::hal::BusI2c;
use crate::types::ErreursAirHaum;
use crate::drivers::imu::Mpu9250;
use crate::interfaces::imu::CentraleInertielle;
use crate::drivers::imu::{CalibrationGyro, CalibrationAccel, CalibrationMag};
use crate::drivers::imu::calcul_calibration::*;
use crate::drivers::gps::calibration::RegistreTerrains;
use crate::systeme::calibration::{gestionnaire, EtatCalibration, CalibrationPersistante};
use crate::types::Vector3;
use std::collections::VecDeque;
use std::io::{self, Write};
use std::time::{Duration, Instant};

/// Test de communication : vérifie le WHO_AM_I
pub fn test_communication<I: BusI2c>(i2c: I) -> crate::types::Result<()> {
    println!("=== Test de communication MPU9250 ===");

    let mut mpu = Mpu9250::nouveau(i2c, crate::drivers::imu::ADRESSE_MPU9250);

    print!("Vérification de l'identité... ");
    if mpu.verifier_identite()? {
        println!("✓ MPU9250 détecté (WHO_AM_I: 0x73)");
    } else {
        return Err(ErreursAirHaum::ErreurCommunication(
            "MPU9250: WHO_AM_I incorrect".into()
        ));
    }

    println!("✓ Communication MPU9250 OK");
    Ok(())
}

/// Test d'initialisation complète
pub fn test_initialisation<I: BusI2c>(i2c: I) -> crate::types::Result<()> {
    println!("=== Test d'initialisation MPU9250 ===");

    let mut mpu = Mpu9250::nouveau(i2c, crate::drivers::imu::ADRESSE_MPU9250);

    print!("Initialisation... ");
    mpu.initialiser()?;
    println!("✓");

    println!("État: {}", mpu.obtenir_etat());
    println!("✓ Initialisation MPU9250 réussie");
    Ok(())
}

/// Test de mesure unique
pub fn test_mesure_unique<I: BusI2c>(i2c: I) -> crate::types::Result<()> {
    println!("=== Test de mesure MPU9250 ===");

    let mut mpu = Mpu9250::nouveau(i2c, crate::drivers::imu::ADRESSE_MPU9250);
    mpu.initialiser()?;

    print!("Lecture des données... ");
    let donnees = mpu.lire()?;

    println!("✓");
    println!("  Gyroscope  (rad/s) : x={:+.4}  y={:+.4}  z={:+.4}",
             donnees.gyroscope.x, donnees.gyroscope.y, donnees.gyroscope.z);
    println!("  Accélèro   (m/s²)  : x={:+.4}  y={:+.4}  z={:+.4}",
             donnees.accelerometre.x, donnees.accelerometre.y, donnees.accelerometre.z);
    println!("  Magnéto    (µT)    : x={:+.2}  y={:+.2}  z={:+.2}",
             donnees.magnetometre.x, donnees.magnetometre.y, donnees.magnetometre.z);
    println!("  Température        : {:.1} °C", donnees.temperature.celsius());
    Ok(())
}

/// Test de mesures continues
pub fn test_mesures_continues<I: BusI2c>(i2c: I, n: usize) -> crate::types::Result<()> {
    println!("=== Test mesures continues MPU9250 ({} mesures) ===", n);

    let mut mpu = Mpu9250::nouveau(i2c, crate::drivers::imu::ADRESSE_MPU9250);
    mpu.initialiser()?;

    let mut ok = 0usize;
    for i in 0..n {
        match mpu.lire() {
            Ok(d) => {
                ok += 1;
                println!("[{:03}] G({:+.3},{:+.3},{:+.3}) A({:+.3},{:+.3},{:+.3}) T={:.1}°C",
                         i,
                         d.gyroscope.x, d.gyroscope.y, d.gyroscope.z,
                         d.accelerometre.x, d.accelerometre.y, d.accelerometre.z,
                         d.temperature.celsius());
            }
            Err(e) => eprintln!("[{:03}] Erreur: {:?}", i, e),
        }
        std::thread::sleep(std::time::Duration::from_millis(5)); // ~200Hz
    }

    println!("\n{}/{} mesures réussies", ok, n);
    Ok(())
}


/// Test de fréquence d'échantillonnage du MPU9250
///
/// Mesure la cadence *réelle* en lançant `n_mesures` lectures consécutives
/// sans pause forcée.
///
/// # Fréquence attendue
///
/// Le MPU9250 est configuré à 200 Hz (SMPLRT_DIV = 4, DLPF activé).
/// L'overhead I²C et le décodage ramènent généralement à 150–180 Hz effectifs.
///
/// # Exemple de sortie
///
/// ```text
/// MPU9250: 172.3 Hz (dt = 5.8 ms), jitter ±0.4 ms
/// ```
pub fn test_frequence<I2C: crate::hal::BusI2c>(i2c: I2C, n_mesures: usize) -> crate::types::Result<crate::types::mesure_frequence::MesureFrequence> {
    use crate::types::mesure_frequence;
    use crate::drivers::imu::Mpu9250;
    use crate::interfaces::imu::CentraleInertielle;
    use crate::drivers::imu::ADRESSE_MPU9250;

    const FREQUENCE_CIBLE_HZ: f32 = 200.0;

    println!("\n=== Test de fréquence MPU9250 ===");
    println!("Nombre de mesures : {}", n_mesures);
    println!("Fréquence nominale cible : {:.1} Hz\n", FREQUENCE_CIBLE_HZ);

    let mut mpu = Mpu9250::nouveau(i2c, ADRESSE_MPU9250);

    print!("Initialisation... ");
    mpu.initialiser()?;
    println!("✓");

    let mut intervalles_us: Vec<u64> = Vec::with_capacity(n_mesures);
    let mut n_erreurs = 0usize;
    let mut dernier_instant = std::time::Instant::now();
    let mut premiere_mesure = true;

    println!("Acquisition en cours (sans pause forcée)...");

    for i in 0..n_mesures {
        let maintenant = std::time::Instant::now();

        match mpu.lire() {
            Ok(_) => {
                if !premiere_mesure {
                    intervalles_us.push(dernier_instant.elapsed().as_micros() as u64);
                }
                premiere_mesure = false;
                dernier_instant = maintenant;
            }
            Err(e) => {
                n_erreurs += 1;
                if i < 5 || i % 20 == 0 {
                    eprintln!("  Erreur #{}: {:?}", i, e);
                }
                premiere_mesure = true;
            }
        }
    }

    let stats = mesure_frequence::calculer_stats("MPU9250", &intervalles_us, n_erreurs);
    stats.afficher_resume();

    if stats.est_dans_tolerance(FREQUENCE_CIBLE_HZ, 20.0) {
        println!("  ✓ Fréquence dans la tolérance ±20% par rapport à {:.1} Hz", FREQUENCE_CIBLE_HZ);
    } else {
        println!(
            "  ⚠ Fréquence hors tolérance : {:.2} Hz vs {:.1} Hz attendus",
            stats.hz_moyen, FREQUENCE_CIBLE_HZ
        );
    }

    Ok(stats)
}




// ─────────────────────────────────────────────────────────────────────────────
// Procédures de calibration — doc/calibration.md
// ─────────────────────────────────────────────────────────────────────────────
//
// Déroulé interactif seulement : les calculs sont dans
// `drivers/imu/calcul_calibration.rs`.

/// Période d'échantillonnage des procédures (100 Hz).
const PERIODE_ECHANTILLON: Duration = Duration::from_millis(10);

fn erreur(message: impl Into<String>) -> ErreursAirHaum {
    ErreursAirHaum::CalibrationEchouee(message.into())
}

fn attendre_entree(message: &str) {
    print!("{} [Entrée] ", message);
    io::stdout().flush().ok();
    let mut ligne = String::new();
    io::stdin().read_line(&mut ligne).ok();
}

fn confirmer(message: &str) -> bool {
    print!("{} [o/N] ", message);
    io::stdout().flush().ok();
    let mut ligne = String::new();
    io::stdin().read_line(&mut ligne).ok();
    matches!(ligne.trim(), "o" | "O" | "oui")
}

fn initialiser_mpu<I: BusI2c>(i2c: I) -> crate::types::Result<Mpu9250<I>> {
    let mut mpu = Mpu9250::nouveau(i2c, crate::drivers::imu::ADRESSE_MPU9250);
    mpu.initialiser()?;
    Ok(mpu)
}

/// Acquiert `duree` d'échantillons à 100 Hz.
fn acquerir<I: BusI2c>(mpu: &mut Mpu9250<I>, duree: Duration) -> crate::types::Result<Vec<EchantillonCapteur>> {
    let debut = Instant::now();
    let mut echantillons = Vec::new();
    while debut.elapsed() < duree {
        echantillons.push(mpu.lire_repere_capteur()?);
        std::thread::sleep(PERIODE_ECHANTILLON);
    }
    Ok(echantillons)
}

/// Acquiert `duree` d'échantillons et exige l'immobilité.
fn acquerir_immobile<I: BusI2c>(mpu: &mut Mpu9250<I>, duree: Duration) -> crate::types::Result<Vec<EchantillonCapteur>> {
    // Laisse retomber les vibrations de l'appui sur Entrée
    std::thread::sleep(Duration::from_secs(1));
    let echantillons = acquerir(mpu, duree)?;
    if !est_immobile(&echantillons) {
        return Err(erreur("mouvement détecté pendant la mesure — recommencer sans toucher l'avion"));
    }
    Ok(echantillons)
}

fn calibration_montage() -> crate::types::Result<CalibrationAccel> {
    match gestionnaire().inspecter::<CalibrationAccel>() {
        EtatCalibration::Valide(c) => Ok(c),
        EtatCalibration::Expiree(_) => Err(erreur(
            "calibration de montage expirée (6 mois) — la refaire à l'atelier (a)")),
        _ => Err(erreur(
            "calibration de montage absente ou d'ancien format — la faire d'abord à l'atelier (a)")),
    }
}

/// Calibration de session sur le terrain (`doc/calibration.md`, § 4) :
/// biais gyroscope, décalage Z de l'accéléromètre et vérification du
/// montage. Rien n'est enregistré si la vérification échoue.
pub fn calibrer_gyro<I: BusI2c>(i2c: I) -> crate::types::Result<()> {
    println!("=== Calibration de session : gyroscope + Z accéléromètre ===");
    let montage = calibration_montage()?;
    let mut mpu = initialiser_mpu(i2c)?;

    attendre_entree("Avion queue au sol, ne plus le toucher pendant 6 s.");
    let echantillons = acquerir_immobile(&mut mpu, Duration::from_secs(5))?;

    let biais = moyenne(echantillons.iter().map(|e| e.gyro));
    let f = moyenne(echantillons.iter().map(|e| e.accel));
    let oz = decalage_z(f).ok_or_else(|| erreur("norme de l'accéléromètre incohérente au repos"))?;

    let (roulis, tangage) = attitude_depuis_gravite(gravite_apparente_avion(&montage.montage, corriger_z(f, oz)));
    let (ecart_roulis, ecart_tangage) = (roulis - montage.attitude_sol_roulis_deg,
                                         tangage - montage.attitude_sol_tangage_deg);
    println!("  Attitude au sol : roulis {:+.1}°, tangage {:+.1}°  (référence atelier : {:+.1}°, {:+.1}°)",
             roulis, tangage, montage.attitude_sol_roulis_deg, montage.attitude_sol_tangage_deg);
    if ecart_roulis.abs() > TOLERANCE_VERIFICATION_DEG || ecart_tangage.abs() > TOLERANCE_VERIFICATION_DEG {
        return Err(erreur(format!(
            "montage non conforme : écart roulis {:+.1}°, tangage {:+.1}° (tolérance ±{:.0}°). \
             La carte a peut-être bougé : refaire la calibration atelier (a). Rien n'a été enregistré.",
            ecart_roulis, ecart_tangage, TOLERANCE_VERIFICATION_DEG
        )));
    }

    let c = CalibrationGyro::nouvelle(biais.x, biais.y, biais.z, oz);
    gestionnaire().sauvegarder(&c)?;
    log::info!(target: "calibration",
        "Session : biais gyro ({:+.4}, {:+.4}, {:+.4}) rad/s, décalage Z {:+.3} m/s², montage vérifié (écarts {:+.1}°, {:+.1}°)",
        biais.x, biais.y, biais.z, oz, ecart_roulis, ecart_tangage);
    println!("✓ Montage vérifié, gyroscope et Z enregistrés (décalage Z {:+.2} m/s²)", oz);
    Ok(())
}

/// Calibration de montage à l'atelier (`doc/calibration.md`, § 3) : queue
/// au sol, puis empennage levé jusqu'au fuselage horizontal.
pub fn calibrer_accel<I: BusI2c>(i2c: I) -> crate::types::Result<()> {
    println!("=== Calibration de montage (atelier) ===");
    println!("  Sol plat vérifié au niveau, avion complet, niveau à bulle sur la");
    println!("  ligne de référence du fuselage.");
    let mut mpu = initialiser_mpu(i2c)?;

    attendre_entree("1/2 — Avion queue au sol, ne plus le toucher.");
    let f_sol = moyenne(acquerir_immobile(&mut mpu, Duration::from_secs(3))?.iter().map(|e| e.accel));
    let oz_sol = decalage_z(f_sol).ok_or_else(|| erreur("norme incohérente (queue au sol)"))?;
    println!("  ✓ Position queue au sol mesurée");

    println!("2/2 — Levez lentement l'empennage jusqu'au fuselage horizontal, puis");
    println!("      tenez-le immobile : capture automatique après 3 s d'immobilité.");
    let taille_fenetre = 300; // 3 s à 100 Hz
    let mut fenetre: VecDeque<EchantillonCapteur> = VecDeque::with_capacity(taille_fenetre);
    let debut = Instant::now();
    let mut dernier_affichage = Instant::now();
    let f_horizontal = loop {
        if debut.elapsed() > Duration::from_secs(120) {
            println!();
            return Err(erreur("position horizontale non atteinte en 2 minutes"));
        }
        let e = mpu.lire_repere_capteur()?;
        if fenetre.len() == taille_fenetre { fenetre.pop_front(); }
        fenetre.push_back(e);

        // Angles calculés après correction de Z : le Z brut gonflé écrase l'écart
        let ecart = ecart_angulaire_deg(corriger_z(e.accel, oz_sol), corriger_z(f_sol, oz_sol));
        if dernier_affichage.elapsed() > Duration::from_millis(500) {
            print!("\r  écart de tangage : {:5.1}°   ", ecart);
            io::stdout().flush().ok();
            dernier_affichage = Instant::now();
        }
        if fenetre.len() == taille_fenetre {
            let echantillons: Vec<EchantillonCapteur> = fenetre.iter().copied().collect();
            let f = moyenne(echantillons.iter().map(|e| e.accel));
            if est_immobile(&echantillons)
                && ecart_angulaire_deg(corriger_z(f, oz_sol), corriger_z(f_sol, oz_sol)) >= ECART_MONTAGE_MIN_DEG
            {
                break f;
            }
        }
        std::thread::sleep(PERIODE_ECHANTILLON);
    };
    println!("\n  ✓ Position horizontale capturée — vous pouvez reposer la queue");

    let oz_hor = decalage_z(f_horizontal).ok_or_else(|| erreur("norme incohérente (horizontale)"))?;
    if (oz_sol - oz_hor).abs() > 0.3 {
        return Err(erreur(format!(
            "décalage Z incohérent entre les deux positions ({:+.2} / {:+.2} m/s²) — recommencer",
            oz_sol, oz_hor)));
    }
    let oz = (oz_sol + oz_hor) / 2.0;
    let montage = rotation_montage(corriger_z(f_sol, oz), corriger_z(f_horizontal, oz))?;
    let (roulis_sol, tangage_sol) = attitude_depuis_gravite(
        gravite_apparente_avion(&montage, corriger_z(f_sol, oz)));

    println!();
    println!("  Écart de tangage entre les positions : {:.1}°",
             ecart_angulaire_deg(corriger_z(f_sol, oz), corriger_z(f_horizontal, oz)));
    println!("  Axe de la carte vers le nez          : {}", axe_carte_dominant(montage[0]));
    println!("  Axe de la carte vers le bas          : {}", axe_carte_dominant(montage[2]));
    println!("  Décalage Z                           : {:+.2} m/s²", oz);
    println!("  Attitude queue au sol (référence)    : roulis {:+.1}°, tangage {:+.1}°", roulis_sol, tangage_sol);
    if !confirmer("Enregistrer cette calibration de montage ?") {
        return Err(erreur("abandonnée par l'opérateur, rien n'a été enregistré"));
    }

    let c = CalibrationAccel::nouvelle(oz, montage, roulis_sol, tangage_sol);
    gestionnaire().sauvegarder(&c)?;
    log::info!(target: "calibration",
        "Montage : nez = {}, bas = {}, décalage Z {:+.3} m/s², attitude sol ({:+.2}°, {:+.2}°)",
        axe_carte_dominant(montage[0]), axe_carte_dominant(montage[2]), oz, roulis_sol, tangage_sol);
    println!("✓ Calibration de montage enregistrée (valable 6 mois)");
    println!("  La calibration de session (g) est à refaire.");
    Ok(())
}

/// Mesures brutes d'un tour magnétomètre, enregistrées en CSV dans le
/// répertoire des logs pour analyse hors ligne (accepté ou refusé).
struct EnregistrementTour {
    fichier: Option<io::BufWriter<std::fs::File>>,
    chemin:  String,
    debut:   Instant,
}

impl EnregistrementTour {
    fn ouvrir() -> Self {
        let chemin = format!("{}/tour_mag_{}.csv",
            crate::systeme::journalisation::repertoire_logs(),
            chrono::Local::now().format("%Y%m%d_%H%M%S"));
        let fichier = std::fs::File::create(&chemin).map(io::BufWriter::new).map_err(|e| {
            log::warn!(target: "calibration", "Enregistrement du tour impossible ({}) : {}", chemin, e);
        }).ok();
        let mut enr = Self { fichier, chemin, debut: Instant::now() };
        enr.ligne("t_s,phase,ax,ay,az,gx,gy,gz,mx,my,mz");
        enr
    }

    fn ligne(&mut self, texte: &str) {
        if let Some(f) = self.fichier.as_mut() {
            let _ = writeln!(f, "{}", texte);
        }
    }

    fn ecrire(&mut self, phase: &str, e: &EchantillonCapteur) {
        let mag = e.mag.map(|m| format!("{:.3},{:.3},{:.3}", m.x, m.y, m.z))
                       .unwrap_or_else(|| ",,".into());
        let texte = format!("{:.3},{},{:.4},{:.4},{:.4},{:.5},{:.5},{:.5},{}",
            self.debut.elapsed().as_secs_f32(), phase,
            e.accel.x, e.accel.y, e.accel.z, e.gyro.x, e.gyro.y, e.gyro.z, mag);
        self.ligne(&texte);
    }

    fn fermer(mut self) {
        if let Some(mut f) = self.fichier.take() {
            let _ = f.flush();
            println!("  Mesures du tour enregistrées : {}", self.chemin);
        }
    }
}

/// Calibration magnétomètre par un tour au sol (`doc/calibration.md`, § 5).
pub fn calibrer_mag<I: BusI2c>(i2c: I) -> crate::types::Result<()> {
    println!("=== Calibration magnétomètre : tour au sol ===");

    // Champ terrestre du terrain actif ; à défaut, valeurs approximatives du
    // Mans (non bloquant — dette technique, doc/suivi_projet.md)
    let terrain = gestionnaire().charger::<RegistreTerrains>().ok().flatten()
        .and_then(|r| r.entree_active().cloned());
    let nom_terrain = terrain.as_ref().map(|t| t.nom.clone()).unwrap_or_else(|| "aucun".into());
    let (inclinaison, champ, par_defaut) = match terrain
        .and_then(|t| t.inclinaison_magnetique_deg.zip(t.champ_magnetique_ut))
    {
        Some((i, c)) => (i, c, false),
        None => (INCLINAISON_DEFAUT_DEG, CHAMP_DEFAUT_UT, true),
    };
    let session = match gestionnaire().inspecter::<CalibrationGyro>() {
        EtatCalibration::Valide(c) => c,
        _ => return Err(erreur("calibration de session absente ou expirée — faire d'abord (g)")),
    };
    let biais = Vector3::nouveau(session.offset_x, session.offset_y, session.offset_z);
    println!("  Terrain « {} » : inclinaison {:.1}°, champ {:.1} µT", nom_terrain, inclinaison, champ);
    if par_defaut {
        println!("  ⚠ champ terrestre non renseigné pour ce terrain : valeurs par défaut (le Mans).");
        println!("    Décalage vertical moins précis loin du Mans — renseigner");
        println!("    inclinaison_magnetique_deg et champ_magnetique_ut dans {} (calculateur NOAA).",
                 RegistreTerrains::identifiant_capteur());
        log::warn!(target: "calibration",
            "Magnétomètre : terrain « {} » sans champ terrestre renseigné — valeurs par défaut ({:.1}°, {:.1} µT)",
            nom_terrain, inclinaison, champ);
    }
    let mut mpu = initialiser_mpu(i2c)?;

    attendre_entree("Avion en configuration de vol, moteur arrêté, loin du métal. Ne plus le toucher.");
    let depart = acquerir_immobile(&mut mpu, Duration::from_secs(3))?;
    let mut enregistrement = EnregistrementTour::ouvrir();
    for e in &depart { enregistrement.ecrire("depart", e); }
    let haut = corriger_z(moyenne(depart.iter().map(|e| e.accel)), session.offset_z_accel).normalise();
    let mut mesures: Vec<MesureTour> = depart.iter()
        .filter_map(|e| e.mag.map(|mag| MesureTour { mag, haut })).collect();

    println!("Faites pivoter l'avion lentement sur ses roues : un tour complet (~1 min).");
    println!("  Lever l'empennage pour tourner est permis : l'inclinaison est suivie.");
    let debut = Instant::now();
    let mut precedent = Instant::now();
    let mut dernier_affichage = Instant::now();
    let mut angle = 0.0f32;
    let mut verticale = haut;
    while angle.abs() < 360.0 {
        if debut.elapsed() > Duration::from_secs(180) {
            println!();
            return Err(erreur(format!("tour incomplet en 3 minutes ({:.0}°)", angle.abs())));
        }
        let e = mpu.lire_repere_capteur()?;
        let maintenant = Instant::now();
        let dt = (maintenant - precedent).as_secs_f32().min(0.05);
        precedent = maintenant;
        verticale = suivre_verticale(verticale, e.gyro, biais, corriger_z(e.accel, session.offset_z_accel), dt);
        angle += angle_autour(verticale, e.gyro, biais, dt);
        if let Some(mag) = e.mag { mesures.push(MesureTour { mag, haut: verticale }); }
        enregistrement.ecrire("tour", &e);
        if dernier_affichage.elapsed() > Duration::from_millis(500) {
            print!("\r  {:3.0}° / 360°   ", angle.abs());
            io::stdout().flush().ok();
            dernier_affichage = Instant::now();
        }
        std::thread::sleep(PERIODE_ECHANTILLON);
    }
    println!("\n  ✓ Tour complet");

    attendre_entree("Immobilisez l'avion.");
    let fin = acquerir_immobile(&mut mpu, Duration::from_secs(3))?;
    for e in &fin { enregistrement.ecrire("fin", e); }
    enregistrement.fermer();
    let haut_fin = corriger_z(moyenne(fin.iter().map(|e| e.accel)), session.offset_z_accel);
    let derive = ecart_angulaire_deg(haut, haut_fin);
    if derive > 2.0 {
        println!("  ⚠ l'inclinaison de l'avion a changé de {:.1}° entre le début et la fin du tour", derive);
    }

    let r = ajuster_tour_mag(&mesures, inclinaison)?;
    println!("  Mesures : {}   champ horizontal : {:.1} µT (attendu {:.1})",
             mesures.len(), r.champ_horizontal_ut, champ * inclinaison.to_radians().cos());
    println!("  Dispersion : horizontale {:.2} µT, verticale {:.2} µT",
             r.dispersion_ut, r.dispersion_verticale_ut);
    println!("  Décalage dû à l'avion : ({:+.1}, {:+.1}, {:+.1}) µT", r.decalage.x, r.decalage.y, r.decalage.z);
    let defauts = defauts_tour_mag(&r, champ, inclinaison);
    if !defauts.is_empty() {
        for d in &defauts { println!("  ✗ {}", d); }
        return Err(erreur("calibration magnétomètre refusée, rien n'a été enregistré"));
    }

    let c = CalibrationMag::nouvelle(r.decalage.x, r.decalage.y, r.decalage.z,
                                     r.champ_horizontal_ut, r.dispersion_ut);
    gestionnaire().sauvegarder(&c)?;
    log::info!(target: "calibration",
        "Magnétomètre : décalage ({:+.2}, {:+.2}, {:+.2}) µT, champ horizontal {:.2} µT, dispersion {:.2} / {:.2} µT (horiz./vert.), {} mesures",
        r.decalage.x, r.decalage.y, r.decalage.z, r.champ_horizontal_ut, r.dispersion_ut, r.dispersion_verticale_ut, mesures.len());
    println!("✓ Calibration magnétomètre enregistrée (valable 1 h)");
    Ok(())
}

/// Diagnostic complet
pub fn diagnostic_complet<I: BusI2c>(i2c: I) -> crate::types::Result<()> {
    println!("=== Diagnostic complet MPU9250 ===\n");

    let mut mpu = Mpu9250::nouveau(i2c, crate::drivers::imu::ADRESSE_MPU9250);

    // 1. Identité
    println!("1. Identité hardware");
    if !mpu.verifier_identite()? {
        return Err(ErreursAirHaum::ErreurCommunication("WHO_AM_I incorrect".into()));
    }

    // 2. Initialisation
    println!("\n2. Initialisation");
    mpu.initialiser()?;
    println!("   État: {}", mpu.obtenir_etat());

    // 3. Mesures
    println!("\n3. Mesures (5 échantillons)");
    for i in 0..5 {
        match mpu.lire() {
            Ok(d) => println!("   [{}] G({:+.3},{:+.3},{:+.3}) A({:+.3},{:+.3},{:+.3})",
                              i,
                              d.gyroscope.x, d.gyroscope.y, d.gyroscope.z,
                              d.accelerometre.x, d.accelerometre.y, d.accelerometre.z),
            Err(e) => eprintln!("   [{}] Erreur: {:?}", i, e),
        }
        std::thread::sleep(std::time::Duration::from_millis(5));
    }

    println!("\n✓ Diagnostic MPU9250 terminé");
    Ok(())
}
