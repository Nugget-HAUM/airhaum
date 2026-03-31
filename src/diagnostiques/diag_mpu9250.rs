// src/diagnostiques/diag_mpu9250.rs
//! Fonctions de diagnostic pour le MPU9250

use crate::hal::BusI2c;
use crate::types::ErreursAirHaum;
use crate::drivers::imu::Mpu9250;
use crate::interfaces::imu::CentraleInertielle;

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




/// Procédure de calibration gyro (à appeler au sol, immobile)
pub fn calibrer_gyro<I: BusI2c>(i2c: I) -> crate::types::Result<()> {
    println!("=== Calibration gyroscope MPU9250 ===");
    println!("Poser l'appareil au sol et ne pas bouger");

    let mut mpu = Mpu9250::nouveau(i2c, crate::drivers::imu::ADRESSE_MPU9250);
    mpu.initialiser()?;

    mpu.calibrer_gyro()?;
    println!("✓ Calibration gyro terminée et sauvegardée");
    Ok(())
}

/// Procédure de calibration magnétomètre (procédure figure-8)
pub fn calibrer_mag<I: BusI2c>(i2c: I) -> crate::types::Result<()> {
    println!("=== Calibration magnétomètre MPU9250 ===");
    println!("Préparez-vous à effectuer des rotations lentes sur les 3 axes");

    let mut mpu = Mpu9250::nouveau(i2c, crate::drivers::imu::ADRESSE_MPU9250);
    mpu.initialiser()?;

    mpu.calibrer_mag()?;
    println!("✓ Calibration magnétomètre terminée et sauvegardée");
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
