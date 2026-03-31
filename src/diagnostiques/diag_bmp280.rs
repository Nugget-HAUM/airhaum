// src/diagnostiques/diag_bmp280.rs
//! Module de diagnostic pour le baromètre BMP280



use crate::*;
use crate::types::Pression;
use crate::types::{Result, mesure_frequence::{self, MesureFrequence}};

use interfaces::barometre::Barometre;
use std::thread;
use std::time::Duration;
//use types::{Pression};


/// Fréquence nominale cible du BMP280 en mode Normal, oversampling x1
///
/// Le BMP280 peut atteindre ~26 Hz en mode normal. En mode forcé (utilisé ici),
/// la fréquence dépend du temps de mesure (~6 ms à oversampling x1).
pub const FREQUENCE_CIBLE_HZ: f32 = 26.0;


// ============================================================================
// Tests 
// ============================================================================

pub fn tester_bmp280() -> Result<()> {
   #[cfg(target_os = "linux")]
   test_bmp280_reel();

   #[cfg(not(target_os = "linux"))]
   test_bmp280_mock();

   Ok(())
}


#[cfg(target_os = "linux")]
fn test_bmp280_reel() {
    use hal::i2c_linux::I2cLinux;
    use drivers::barometre::Bmp280;

    println!("🔧 Ouverture du bus I²C...");
    
    // Sur Orange Pi Zero, le bus I²C est généralement /dev/i2c-0
    // Vérifiez avec: ls /dev/i2c-*
    let i2c = match I2cLinux::nouveau(0) {
        Ok(i2c) => {
            println!("✓ Bus I²C ouvert: /dev/i2c-0\n");
            i2c
        }
        Err(e) => {
            eprintln!("✗ Erreur ouverture I²C: {}", e);
            eprintln!("\nVérifiez que:");
            eprintln!("  1. Le module I²C est chargé: lsmod | grep i2c");
            eprintln!("  2. Le périphérique existe: ls -l /dev/i2c-*");
            eprintln!("  3. Vous avez les permissions: sudo usermod -aG i2c $USER");
            eprintln!("  4. Le BMP280 est bien câblé (VCC, GND, SDA, SCL)");
            return;
        }
    };

    let mut bmp = Bmp280::nouveau(i2c);

    println!("🔧 Initialisation du BMP280...");
    match bmp.initialiser() {
        Ok(_) => println!("✓ BMP280 initialisé avec succès\n"),
        Err(e) => {
            eprintln!("✗ Erreur initialisation: {}", e);
            eprintln!("\nVérifiez que:");
            eprintln!("  1. L'adresse I²C est correcte (0x76 ou 0x77)");
            eprintln!("  2. Le câblage est bon");
            eprintln!("  3. Le capteur est alimenté en 3.3V");
            eprintln!("\nPour scanner le bus I²C:");
            eprintln!("  sudo i2cdetect -y 0");
            return;
        }
    }

    println!("📊 Lecture des données (10 échantillons)...\n");
    println!("{:<5} {:<15} {:<15} {:<15}", "N°", "Pression", "Température", "Altitude");
    println!("{:-<60}", "");

    let pression_reference = Pression::niveau_mer_standard();

    for i in 1..=5 {
        match bmp.lire() {
            Ok(donnees) => {
                let pression_hpa = donnees.pression.hectopascals();
                let temp_c = donnees.temperature.celsius();
                let altitude = donnees.pression.vers_altitude(pression_reference);
               /*
                println!(
                    "{:<5} {:<15.2} {:<15.2} {:<15.1}",
                    i,
                    format!("{:.2} hPa", pression_hpa),
                    format!("{:.2} °C", temp_c),
                    format!("{:.1} m", altitude)
                );
               */
                println!(
                "{:<5} {:>12.2} hPa   {:>10.2} °C   {:>10.1} m",
                i,
                pression_hpa,
                temp_c,
                altitude
                );
            }
            Err(e) => {
                eprintln!("✗ Erreur lecture {}: {}", i, e);
            }
        }

        if i < 10 {
            thread::sleep(Duration::from_millis(250));
        }
    }

    println!("\n✓ Test terminé avec succès");
}

#[cfg(not(target_os = "linux"))]
fn test_bmp280_mock() {
    use hal::i2c::I2cMock;
    use drivers::barometre::Bmp280;

    let mut i2c = I2cMock::nouveau();
    
    // Simuler l'ID du BMP280
    i2c.precharger_registre(0x76, 0xD0, 0x58);
    
    let mut bmp = Bmp280::nouveau(i2c);
    
    println!("🔧 Initialisation du BMP280 (mock)...");
    match bmp.initialiser() {
        Ok(_) => println!("✓ BMP280 initialisé (mock)\n"),
        Err(e) => {
            println!("✗ Erreur: {}", e);
            return;
        }
    }
    
    println!("Note: Utilisez Linux pour tester avec le vrai capteur");
}




/// Test de fréquence d'échantillonnage du BMP280
///
/// Mesure la fréquence *réelle* en lançant `n_mesures` lectures consécutives
/// sans pause artificielle et en chronométrant chaque intervalle.
///
/// # Sortie typique attendue
///
/// ```text
/// BMP280: 25.8 Hz (dt = 38.7 ms), jitter ±1.2 ms
/// ```
pub fn test_frequence_bmp280(n_mesures: usize) -> Result<MesureFrequence> {
    println!("\n=== Test de fréquence BMP280 ===");
    println!("Nombre de mesures : {}", n_mesures);
    println!("Fréquence nominale cible : {:.1} Hz\n", FREQUENCE_CIBLE_HZ);

    #[cfg(target_os = "linux")]
    {
        use crate::hal::i2c_linux::I2cLinux;
        use crate::drivers::barometre::Bmp280;
        use crate::interfaces::barometre::Barometre;

        let i2c = I2cLinux::nouveau(0)?;
        let mut bmp = Bmp280::nouveau(i2c);

        print!("Initialisation... ");
        bmp.initialiser()?;
        println!("✓");

        let mut intervalles_us: Vec<u64> = Vec::with_capacity(n_mesures);
        let mut n_erreurs = 0usize;
        let mut dernier_instant = std::time::Instant::now();
        let mut premiere_mesure = true;

        println!("Acquisition en cours (sans pause forcée)...");

        for i in 0..n_mesures {
            let maintenant = std::time::Instant::now();

            match bmp.lire() {
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
                    // On remet l'instant à jour pour ne pas fausser l'intervalle suivant
                    premiere_mesure = true;
                }
            }
        }

        let stats = mesure_frequence::calculer_stats("BMP280", &intervalles_us, n_erreurs);
        stats.afficher_resume();

        let ok = stats.est_dans_tolerance(FREQUENCE_CIBLE_HZ, 30.0);
        if ok {
            println!("  ✓ Fréquence dans la tolérance ±30% par rapport à {:.1} Hz", FREQUENCE_CIBLE_HZ);
        } else {
            println!(
                "  ⚠ Fréquence hors tolérance : {:.2} Hz vs {:.1} Hz attendus",
                stats.hz_moyen, FREQUENCE_CIBLE_HZ
            );
        }
        return Ok(stats);
    }
}

pub fn calibrer_bmp280() -> Result<()> {
    #[cfg(target_os = "linux")]
    {
        use hal::i2c_linux::I2cLinux;
        use drivers::barometre::Bmp280;
        use interfaces::barometre::Barometre;

        let i2c = I2cLinux::nouveau(0)?;
        let mut bmp = Bmp280::nouveau(i2c);
        bmp.initialiser()?;
        bmp.calibrer_pression_sol(3600)?;
    }
    Ok(())
}

