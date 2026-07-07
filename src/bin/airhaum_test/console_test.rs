// src/bin/airhaum_test/console_test.rs
//! Console de test — menu interactif et fonctions de visualisation en temps réel.

use std::io::{self, Write};
use airhaum::mission::etat_machine::MachineEtatVol;
use airhaum::taches::taches_capteurs::HandlesCapteurs;
use airhaum::taches::taches_estimation::HandlesEstimation;
use airhaum::taches::taches_gps::HandlesGps;
use airhaum::taches::taches_servo::HandlesServo;

pub fn afficher_menu(mae: &MachineEtatVol) {
    println!("\n╔════════════════════════════════════════╗");
    println!("║     AIRHAUM II - Console de Test       ║");
    println!("║  MAÉ : {:<32}║", format!("{}", mae.etat()));
    println!("╚════════════════════════════════════════╝");
    println!("\nTests disponibles:");
    println!("  1.  BMP280  - Test mesure baromètre");
    println!("  11. BMP280  - Calibration pré-vol");
    println!("  12. BMP280  - Test fréquence (100 mesures)");
    println!("  2.  VL53L0X - Test i2c télémètre");
    println!("  21. VL53L0X - Initialisation complète");
    println!("  22. VL53L0X - Mesure unique");
    println!("  23. VL53L0X - Mesures continues (10x)");
    println!("  24. VL53L0X - Diagnostic complet");
    println!("  25. VL53L0X - Test fréquence (100 mesures)");
    println!("  3.  MPU9250 - Test i2c centrale inertielle");
    println!("  31. MPU9250 - Initialisation complète");
    println!("  32. MPU9250 - Mesure unique");
    println!("  33. MPU9250 - Mesures continues (10x)");
    println!("  34. MPU9250 - Calibration gyroscope");
    println!("  35. MPU9250 - Calibration magnétomètre");
    println!("  36. MPU9250 - Diagnostic complet");
    println!("  37. MPU9250 - Test fréquence (100 mesures)");
    println!("  4.  GPS     - État actuel (depuis thread)");
    println!("  41. GPS     - Attendre fix (max 60s)");
    println!("  42. GPS     - Données en temps réel (10s)");
    println!("  5.  Arduino - État liaison et RC");
    println!("  51. Arduino - Test servos (débattements)");
    println!("  s.  Santé capteurs - état en temps réel");
    println!("  a.  Attitude    - EKF en temps réel (10 s)");
    println!("  n.  Navigation  - EKF état complet : position NED, vitesse, biais gyro (10 s)");
    println!("  h.  Hauteur     - altitude baro + sol télémètre en temps réel (10 s)");
    println!("  c.  Check-liste armement");
    println!("  t.  Tous les capteurs i2c");
    println!("  j.  Journal — 20 dernières lignes");
    println!("  q.  Quitter");
    print!("\nVotre choix: ");
    io::stdout().flush().unwrap();
}

/// Affiche l'attitude EKF en temps réel pendant 10 secondes.
///
/// Valide l'étape [C] : inclinez la carte pour vérifier que roulis et tangage
/// varient correctement.
pub async fn afficher_attitude(estimation: &mut HandlesEstimation) {
    let debut = std::time::Instant::now();
    let duree = std::time::Duration::from_secs(10);
    let mut n  = 0u32;

    println!("\n── Attitude EKF (10 secondes) ──────────────────────────────────");
    println!("  Inclinez la carte pour vérifier roulis et tangage.");
    println!("  {:>6}  {}", "N", "Attitude");

    while debut.elapsed() < duree {
        match tokio::time::timeout(
            std::time::Duration::from_millis(100),
            estimation.rx_attitude.changed(),
        ).await {
            Ok(Ok(_)) => {
                n += 1;
                let att = *estimation.rx_attitude.borrow_and_update();
                if n % 40 == 1 {
                    println!("  {:>6}  {}", n, att);
                }
            }
            _ => {}
        }
    }
    println!("── {} mises à jour reçues en {:.1}s ─────────────────────────────────",
        n, debut.elapsed().as_secs_f32());
}

/// Affiche l'état de navigation complet (EKF 13 états) pendant 10 secondes.
///
/// Valide l'étape [E] : position NED, vitesse, biais gyro estimé doivent être
/// cohérents avec les mouvements de la carte et le fix GPS.
pub async fn afficher_navigation(estimation: &mut HandlesEstimation) {
    let debut = std::time::Instant::now();
    let duree = std::time::Duration::from_secs(10);
    let mut n  = 0u32;

    println!("\n── Navigation EKF (10 secondes) ────────────────────────────────");
    println!("  {:>6}  {:<38}  {:<30}  {}",
        "N", "Attitude (°)", "Position NED (m)", "Vitesse NED (m/s)");

    while debut.elapsed() < duree {
        match tokio::time::timeout(
            std::time::Duration::from_millis(100),
            estimation.rx_navigation.changed(),
        ).await {
            Ok(Ok(_)) => {
                n += 1;
                let nav = *estimation.rx_navigation.borrow_and_update();
                if n % 40 == 1 {
                    let att = nav.attitude;
                    let p   = nav.position_ned;
                    let v   = nav.vitesse_ned;
                    let bg  = nav.biais_gyro;
                    let orig = if nav.origine_definie { "GPS" } else { "—  " };
                    println!("  {:>6}  roulis={:+5.1}° tangage={:+5.1}° lacet={:+6.1}°  \
                              N={:+8.1} E={:+8.1} B={:+7.1} [{}]  \
                              vN={:+5.1} vE={:+5.1} vB={:+5.1}  \
                              bg=({:+.4},{:+.4},{:+.4}) rad/s",
                        n,
                        att.roulis.degres(), att.tangage.degres(), att.lacet.degres(),
                        p.x, p.y, p.z, orig,
                        v.x, v.y, v.z,
                        bg.x, bg.y, bg.z,
                    );
                }
            }
            _ => {}
        }
    }
    println!("── {} mises à jour reçues en {:.1}s ─────────────────────────────────",
        n, debut.elapsed().as_secs_f32());
}

/// Affiche l'altitude fusionnée (baro + télémètre) pendant 10 secondes.
///
/// Valide l'étape [D] : altitude barométrique et hauteur sol doivent varier
/// en cohérence avec les mouvements de la carte.
pub async fn afficher_altitude(estimation: &mut HandlesEstimation) {
    let debut = std::time::Instant::now();
    let duree = std::time::Duration::from_secs(10);
    let mut n  = 0u32;

    println!("\n── Altitude fusionnée (10 secondes) ───────────────────────────");
    println!("  {:>6}  {:>12}  {:>14}", "N", "Altitude m", "Hauteur sol mm");

    while debut.elapsed() < duree {
        match tokio::time::timeout(
            std::time::Duration::from_millis(200),
            estimation.rx_altitude.changed(),
        ).await {
            Ok(Ok(_)) => {
                n += 1;
                let alt = *estimation.rx_altitude.borrow_and_update();
                if n % 5 == 1 {
                    let alt_str = alt.altitude_m
                        .map(|h| format!("{:+.1}", h))
                        .unwrap_or_else(|| "—".into());
                    let sol_str = alt.hauteur_sol_mm
                        .map(|d| format!("{}", d))
                        .unwrap_or_else(|| "—".into());
                    println!("  {:>6}  {:>12}  {:>14}", n, alt_str, sol_str);
                }
            }
            _ => {}
        }
    }
    println!("── {} mises à jour reçues en {:.1}s ─────────────────────────────────",
        n, debut.elapsed().as_secs_f32());
}

/// Attend le premier fix GPS depuis le thread, jusqu'à 60 secondes.
pub async fn attendre_fix_gps(gps: &HandlesGps) {
    use airhaum::types::TypeFixGps;
    let debut   = std::time::Instant::now();
    let timeout = std::time::Duration::from_secs(60);
    let mut rx  = gps.rx_gps.clone();
    let mut n_trames = 0u32;

    println!("\n── Attente fix GPS (max 60s) ─────────────────────────────────");
    println!("  {:>6}  {:>7}  {:>10}  {:>10}  {:>5}  {}", "Trame", "Fix", "Lat", "Lon", "Sats", "Préc H");

    while debut.elapsed() < timeout {
        match tokio::time::timeout(std::time::Duration::from_millis(500), rx.changed()).await {
            Ok(Ok(_)) => {
                let mesure = rx.borrow_and_update();
                if let Some(d) = mesure.donnees.as_ref() {
                    n_trames += 1;
                    let fix_str = match d.type_fix {
                        TypeFixGps::Aucun  => "aucun",
                        TypeFixGps::Fix2D  => "2D",
                        TypeFixGps::Fix3D  => "3D",
                        TypeFixGps::GnssDr => "GNSS+DR",
                    };
                    println!("  {:>6}  {:>7}  {:>10.4}  {:>10.4}  {:>5}  {:.1}m",
                        n_trames, fix_str,
                        d.latitude, d.longitude,
                        d.nombre_satellites, d.precision_h);
                    if mesure.valide {
                        println!("\n  ✓ Fix {} obtenu en {:.1}s", fix_str, debut.elapsed().as_secs_f32());
                        return;
                    }
                }
            }
            _ => {}
        }
    }
    println!("\n  ⚠ Timeout : fix non obtenu en 60s ({} trames reçues)", n_trames);
}

/// Affiche les données GPS en temps réel pendant 10 secondes.
pub async fn afficher_gps(gps: &HandlesGps) {
    use airhaum::types::TypeFixGps;
    let debut = std::time::Instant::now();
    let duree = std::time::Duration::from_secs(10);
    let mut rx = gps.rx_gps.clone();
    let mut n  = 0u32;

    println!("\n── GPS en temps réel (10 secondes) ─────────────────────────────");
    println!("  {:>5}  {:>7}  {:>10}  {:>10}  {:>7}  {:>6}  {:>5}  {:>6}",
        "N", "Fix", "Latitude", "Longitude", "Alt MSL", "Vit sol", "Sats", "Préc H");

    while debut.elapsed() < duree {
        match tokio::time::timeout(std::time::Duration::from_millis(500), rx.changed()).await {
            Ok(Ok(_)) => {
                let mesure = rx.borrow_and_update();
                if let Some(d) = mesure.donnees.as_ref() {
                    n += 1;
                    let fix_str = match d.type_fix {
                        TypeFixGps::Aucun  => "aucun  ",
                        TypeFixGps::Fix2D  => "2D     ",
                        TypeFixGps::Fix3D  => "3D     ",
                        TypeFixGps::GnssDr => "GNSS+DR",
                    };
                    println!("  {:>5}  {}  {:>10.5}  {:>10.5}  {:>6.1}m  {:>5.1}m/s  {:>5}  {:>5.1}m",
                        n, fix_str,
                        d.latitude, d.longitude,
                        d.altitude_msl, d.vitesse_sol,
                        d.nombre_satellites, d.precision_h);
                }
            }
            _ => {}
        }
    }
    println!("── {} positions reçues en {:.1}s ─────────────────────────────────",
        n, debut.elapsed().as_secs_f32());
}

/// Affiche les `n` dernières lignes du fil de vie (journal texte lisible).
pub fn afficher_journal(n: usize) {
    use std::fs;
    use std::io::{BufRead, BufReader};
    use airhaum::systeme::journalisation::repertoire_logs;

    let repertoire = repertoire_logs();
    let entrees = match fs::read_dir(&repertoire) {
        Err(e) => {
            println!("\n  Répertoire de logs inaccessible ({}) : {}", repertoire, e);
            return;
        }
        Ok(e) => e,
    };

    let mut fichiers: Vec<_> = entrees
        .filter_map(|e| e.ok())
        .filter(|e| e.file_name().to_string_lossy().starts_with("fil_de_vie_"))
        .collect();

    fichiers.sort_by_key(|e| e.file_name());

    let dernier = match fichiers.last() {
        None => { println!("\n  Aucun fichier journal trouvé dans {}", repertoire); return; }
        Some(f) => f,
    };

    let f = match fs::File::open(dernier.path()) {
        Err(e) => { println!("\n  Impossible d'ouvrir {} : {}", dernier.path().display(), e); return; }
        Ok(f) => f,
    };

    let toutes: Vec<String> = BufReader::new(f)
        .lines()
        .filter_map(|l| l.ok())
        .collect();

    println!("\n── Journal  {} — {} dernières lignes ─────────────────────────────────",
        dernier.file_name().to_string_lossy(), n);

    if toutes.is_empty() {
        println!("  (journal vide)");
        return;
    }

    let debut = toutes.len().saturating_sub(n);
    for ligne in &toutes[debut..] {
        println!("  {}", ligne);
    }
}

/// Affiche l'état courant de la liaison Arduino et du signal RC.
pub fn afficher_etat_arduino(servo: &HandlesServo) {
    use std::sync::atomic::Ordering;
    use airhaum::drivers::controleur_servo::ModeArduino;

    let liaison = servo.liaison_detectee.load(Ordering::Relaxed);
    let etat = servo.rx_etat.borrow();

    println!("\n── État Arduino / RC ──────────────────────────────────────────");
    println!("  Liaison détectée : {}", if liaison { "oui" } else { "non (aucune trame reçue)" });
    println!("  Mode             : {}", match etat.mode { ModeArduino::Autopilote => "autopilote", ModeArduino::Manuel => "manuel" });
    println!("  Signal RC        : {}", if etat.rc_perdu { "perdu" } else { "présent" });
    println!("  Chien de garde   : {}", if etat.chien_de_garde { "déclenché" } else { "nominal" });
    println!("  Gaz appliqué     : {} µs", etat.gaz_applique);
    println!("  Canaux RC (µs)   : CH1={}  CH2={}  CH3={}  CH4={}  CH5={}",
        etat.canaux_rc[0], etat.canaux_rc[1], etat.canaux_rc[2],
        etat.canaux_rc[3], etat.canaux_rc[4]);
}

/// Test interactif des débattements des surfaces de vol.
///
/// Envoie min/max/neutre pour chaque surface successivement en mode non armé
/// (gaz verrouillés à zéro par le Nano). L'opérateur confirme visuellement
/// que chaque surface s'actionne dans le bon sens.
pub fn tester_debattements_servos(servo: &HandlesServo) {
    use std::sync::atomic::Ordering;
    use airhaum::drivers::controleur_servo::{ConsignesServos, IMPULSION_MIN_US, IMPULSION_NEUTRE_US, IMPULSION_MAX_US};

    println!("\n── Test débattements servos ───────────────────────────────────────");
    println!("  Mode non armé — gaz verrouillés à zéro par le Nano.");
    println!("  Observez que chaque surface s'actionne dans le bon sens.");
    println!("  [Entrée] confirmer et passer à l'étape suivante, q annuler.\n");

    if !servo.liaison_detectee.load(Ordering::Relaxed) {
        println!("  ⚠  Aucune liaison Arduino détectée — les consignes seront envoyées sans confirmation de réception.\n");
    }

    let surfaces: &[(&str, fn(u16) -> ConsignesServos)] = &[
        ("Ailerons",   |v| ConsignesServos { ailerons: v, profondeur: IMPULSION_NEUTRE_US, gaz: IMPULSION_MIN_US, direction: IMPULSION_NEUTRE_US, arme: false }),
        ("Profondeur", |v| ConsignesServos { ailerons: IMPULSION_NEUTRE_US, profondeur: v, gaz: IMPULSION_MIN_US, direction: IMPULSION_NEUTRE_US, arme: false }),
        ("Direction",  |v| ConsignesServos { ailerons: IMPULSION_NEUTRE_US, profondeur: IMPULSION_NEUTRE_US, gaz: IMPULSION_MIN_US, direction: v, arme: false }),
    ];

    loop {
        println!("  Choisir la surface à tester :");
        for (i, (nom, _)) in surfaces.iter().enumerate() {
            println!("    {}. {}", i + 1, nom);
        }
        println!("    q. Quitter le test");
        print!("  Votre choix : ");
        io::stdout().flush().unwrap();

        let mut choix = String::new();
        io::stdin().read_line(&mut choix).unwrap();
        let choix = choix.trim();

        let idx = match choix {
            "q" | "Q" => break,
            "1" => 0,
            "2" => 1,
            "3" => 2,
            _ => { println!("  Choix invalide."); continue; }
        };

        let (nom, mk) = &surfaces[idx];
        println!("  Test {} — minimum puis maximum puis neutre.", nom);

        for (label, val) in [
            ("minimum", IMPULSION_MIN_US),
            ("maximum", IMPULSION_MAX_US),
            ("neutre",  IMPULSION_NEUTRE_US),
        ] {
            let _ = servo.tx_consignes.send(mk(val));
            print!("  {} → {} ({} µs)  [Entrée] pour continuer : ", nom, label, val);
            io::stdout().flush().unwrap();
            let mut input = String::new();
            io::stdin().read_line(&mut input).unwrap();
            if input.trim().eq_ignore_ascii_case("q") {
                let _ = servo.tx_consignes.send(ConsignesServos::neutre());
                println!("  Test interrompu — servos repositionnés au neutre.");
                return;
            }
        }
        let _ = servo.tx_consignes.send(ConsignesServos::neutre());
        println!("  {} repositionné au neutre.\n", nom);
    }

    let _ = servo.tx_consignes.send(ConsignesServos::neutre());
    println!("  Fin du test — servos au neutre.");
}

/// Affiche l'état en temps réel des threads capteurs depuis les canaux.
pub fn afficher_sante(capteurs: &HandlesCapteurs, gps: Option<&HandlesGps>) {
    use std::sync::atomic::Ordering;
    let baro  = capteurs.rx_baro.borrow();
    let telem = capteurs.rx_telem.borrow();
    println!("\n── Santé capteurs ──────────────────────────");
    println!("  BMP280  : {}  erreurs={} réinit={}",
        if baro.valide  { "✓ OK" } else { "✗ KO" },
        capteurs.sante.erreurs_baro.load(Ordering::Relaxed),
        capteurs.sante.reinit_baro.load(Ordering::Relaxed));
    println!("  VL53L0X : {}  erreurs={} réinit={}",
        if telem.valide { "✓ OK" } else { "✗ KO" },
        capteurs.sante.erreurs_telem.load(Ordering::Relaxed),
        capteurs.sante.reinit_telem.load(Ordering::Relaxed));
    println!("  MPU9250 : erreurs={} réinit={}",
        capteurs.sante.erreurs_imu.load(Ordering::Relaxed),
        capteurs.sante.reinit_imu.load(Ordering::Relaxed));
    if let Some(p) = baro.donnees.as_ref() {
        println!("  Dernière pression : {:.1} Pa  T={:.1}°C",
            p.pression.pascals(), p.temperature.celsius());
    }
    if let Some(d) = telem.distance_mm {
        println!("  Dernière distance : {} mm", d);
    }
    if let Some(g) = gps {
        let mesure = g.rx_gps.borrow();
        let fix_str = if mesure.valide { "✓ OK" } else { "⏳ En attente de fix" };
        println!("  GPS     : {}  erreurs={} réinit={}",
            fix_str,
            g.sante.erreurs.load(Ordering::Relaxed),
            g.sante.reinit.load(Ordering::Relaxed));
        if let Some(d) = mesure.donnees.as_ref() {
            println!("  Dernière position : {:.5}°N {:.5}°E  alt={:.0}m  sats={}",
                d.latitude, d.longitude, d.altitude_msl, d.nombre_satellites);
        }
    } else {
        println!("  GPS     : module non disponible");
    }
}
