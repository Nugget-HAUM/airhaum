// src/bin/airhaum-test.rs
//! Console de test et diagnostic pour AirHaum II
//! 
//! Ce binaire permet de tester individuellement chaque composant matériel et logiciel 
//! Les briques du code utilisé en vol sont testés individuellement et par groupe logique 


use std::io::{self, Write};

fn afficher_menu() {
    println!("\n╔════════════════════════════════════════╗");
    println!("║     AIRHAUM II - Console de Test       ║");
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
    println!("  tc. Capteurs simultanés - fréquences réelles (10s)");
    println!("  t. Tous les capteurs i2c");
    println!("  q. Quitter");
    print!("\nVotre choix: ");
    io::stdout().flush().unwrap();
}

//fn main() {
#[tokio::main]
async fn main() {
    println!("Lancement AirHaum II v{}", airhaum::VERSION);
    println!("Initialisation de la console de test...");

    // Initialisation des services système
    airhaum::systeme::calibration::initialiser_gestionnaire("/home/airhaum/config");
    
    // Création du bus I²C
    #[cfg(target_os = "linux")]
    let mut i2c = match airhaum::hal::I2cLinux::nouveau(0) {
        Ok(bus) => bus,
        Err(e) => {
            eprintln!("❌ Impossible d'ouvrir le bus I²C: {:?}", e);
            eprintln!("Vérifiez que:");
            eprintln!("  - L'I²C est activé (armbian-config)");
            eprintln!("  - Vous avez les permissions (groupe i2c)");
            std::process::exit(1);
        }
    };
    
    #[cfg(not(target_os = "linux"))]
    let mut i2c = airhaum::hal::i2c::I2cMock::nouveau();
    
    loop {
        afficher_menu();
        
        let mut choix = String::new();
        io::stdin().read_line(&mut choix).unwrap();
        let choix = choix.trim();
        
        match choix {
            "1" => {
                // Test BMP280
                match airhaum::diagnostiques::diag_bmp280::tester_bmp280() {
                    Ok(_) => println!("\n✓ Test BMP280 terminé avec succès"),
                    Err(e) => eprintln!("\n✗ Erreur lors du test BMP280: {:?}", e),
                }
            }
            "11" => {
                match airhaum::diagnostiques::diag_bmp280::calibrer_bmp280() {
                   Ok(_) => println!("\n✓ Calibration BMP280 effectuée"),
                   Err(e) => eprintln!("\n✗ Erreur calibration BMP280: {:?}", e),
               }
            }

            "12" => {
                match airhaum::diagnostiques::diag_bmp280::test_frequence_bmp280(100) {
                    Ok(stats) => {
                        println!("\n✓ BMP280 : {:.2} Hz (jitter ±{:.2} ms)", stats.hz_moyen, stats.jitter_ms);
                    }
                    Err(e) => eprintln!("\n✗ Erreur fréquence BMP280: {:?}", e),
                }
            }
            "2" => {
                // Test communication VL53L0X
                match airhaum::diagnostiques::diag_vl53l0x::test_communication(&mut i2c) {
                    Ok(_) => println!("\n✓ Communication VL53L0X OK"),
                    Err(e) => eprintln!("\n✗ Erreur: {:?}", e),
                }
            }
            "21" => {
                // Test initialisation VL53L0X
                #[cfg(target_os = "linux")]
                let i2c_init = match airhaum::hal::I2cLinux::nouveau(0) {
                    Ok(bus) => bus,
                    Err(e) => {
                        eprintln!("❌ Erreur I²C: {:?}", e);
                        continue;
                    }
                };
                
                #[cfg(not(target_os = "linux"))]
                let i2c_init = airhaum::hal::i2c::I2cMock::nouveau();
                
                match airhaum::diagnostiques::test_initialisation(i2c_init) {
                    Ok(_) => println!("\n✓ Initialisation VL53L0X réussie"),
                    Err(e) => eprintln!("\n✗ Erreur: {:?}", e),
                }
            }
            "22" => {
                // Test mesure unique VL53L0X
                #[cfg(target_os = "linux")]
                let i2c_mesure = match airhaum::hal::I2cLinux::nouveau(0) {
                    Ok(bus) => bus,
                    Err(e) => {
                        eprintln!("❌ Erreur I²C: {:?}", e);
                        continue;
                    }
                };
                
                #[cfg(not(target_os = "linux"))]
                let i2c_mesure = airhaum::hal::i2c::I2cMock::nouveau();
                
                match airhaum::diagnostiques::diag_vl53l0x::test_mesure_unique(i2c_mesure) {
                    Ok(_) => println!("\n✓ Mesure effectuée"),
                    Err(e) => eprintln!("\n✗ Erreur: {:?}", e),
                }
            }
            "23" => {
                // Test mesures continues VL53L0X
                #[cfg(target_os = "linux")]
                let i2c_continu = match airhaum::hal::I2cLinux::nouveau(0) {
                    Ok(bus) => bus,
                    Err(e) => {
                        eprintln!("❌ Erreur I²C: {:?}", e);
                        continue;
                    }
                };
                
                #[cfg(not(target_os = "linux"))]
                let i2c_continu = airhaum::hal::i2c::I2cMock::nouveau();
                
                match airhaum::diagnostiques::diag_vl53l0x::test_mesures_continues(i2c_continu, 10) {
                    Ok(_) => println!("\n✓ Tests terminés"),
                    Err(e) => eprintln!("\n✗ Erreur: {:?}", e),
                }
            }
            "24" => {
                // Diagnostic complet VL53L0X
                #[cfg(target_os = "linux")]
                let i2c_diag = match airhaum::hal::I2cLinux::nouveau(0) {
                    Ok(bus) => bus,
                    Err(e) => {
                        eprintln!("❌ Erreur I²C: {:?}", e);
                        continue;
                    }
                };
                
                #[cfg(not(target_os = "linux"))]
                let i2c_diag = airhaum::hal::i2c::I2cMock::nouveau();
                
                match airhaum::diagnostiques::diag_vl53l0x::diagnostic_complet(i2c_diag) {
                    Ok(_) => println!("\n✓ Diagnostic terminé"),
                    Err(e) => eprintln!("\n✗ Erreur: {:?}", e),
                }
            }
            "25" => {
                #[cfg(target_os = "linux")]
                let i2c_freq = match airhaum::hal::I2cLinux::nouveau(0) {
                    Ok(bus) => bus,
                    Err(e) => { eprintln!("❌ Erreur I²C: {:?}", e); continue; }
                };
                #[cfg(not(target_os = "linux"))]
                let i2c_freq = airhaum::hal::i2c::I2cMock::nouveau();

                match airhaum::diagnostiques::diag_vl53l0x::test_frequence(i2c_freq, 100) {
                    Ok(stats) => {
                        println!("\n✓ VL53L0X : {:.2} Hz (jitter ±{:.2} ms)", stats.hz_moyen, stats.jitter_ms);
                    }
                    Err(e) => eprintln!("\n✗ Erreur fréquence VL53L0X: {:?}", e),
                }
            }



            "3" => {
                #[cfg(target_os = "linux")]
                let i2c_mpu = match airhaum::hal::I2cLinux::nouveau(0) {
                    Ok(bus) => bus,
                    Err(e) => { eprintln!("❌ Erreur I²C: {:?}", e); continue; }
                };
                #[cfg(not(target_os = "linux"))]
                let i2c_mpu = airhaum::hal::i2c::I2cMock::nouveau();
                match airhaum::diagnostiques::diag_mpu9250::test_communication(i2c_mpu) {
                    Ok(_) => println!("\n✓ Communication MPU9250 OK"),
                    Err(e) => eprintln!("\n✗ Erreur: {:?}", e),
                }
            }
            "31" => {
                #[cfg(target_os = "linux")]
                let i2c_mpu = match airhaum::hal::I2cLinux::nouveau(0) {
                    Ok(bus) => bus,
                    Err(e) => { eprintln!("❌ Erreur I²C: {:?}", e); continue; }
                };
                #[cfg(not(target_os = "linux"))]
                let i2c_mpu = airhaum::hal::i2c::I2cMock::nouveau();
                match airhaum::diagnostiques::diag_mpu9250::test_initialisation(i2c_mpu) {
                    Ok(_) => println!("\n✓ Initialisation MPU9250 réussie"),
                    Err(e) => eprintln!("\n✗ Erreur: {:?}", e),
                }
            }
            "32" => {
                #[cfg(target_os = "linux")]
                let i2c_mpu = match airhaum::hal::I2cLinux::nouveau(0) {
                    Ok(bus) => bus,
                    Err(e) => { eprintln!("❌ Erreur I²C: {:?}", e); continue; }
                };
                #[cfg(not(target_os = "linux"))]
                let i2c_mpu = airhaum::hal::i2c::I2cMock::nouveau();
                match airhaum::diagnostiques::diag_mpu9250::test_mesure_unique(i2c_mpu) {
                    Ok(_) => println!("\n✓ Mesure effectuée"),
                    Err(e) => eprintln!("\n✗ Erreur: {:?}", e),
                }
            }
            "33" => {
                #[cfg(target_os = "linux")]
                let i2c_mpu = match airhaum::hal::I2cLinux::nouveau(0) {
                    Ok(bus) => bus,
                    Err(e) => { eprintln!("❌ Erreur I²C: {:?}", e); continue; }
                };
                #[cfg(not(target_os = "linux"))]
                let i2c_mpu = airhaum::hal::i2c::I2cMock::nouveau();
                match airhaum::diagnostiques::diag_mpu9250::test_mesures_continues(i2c_mpu, 10) {
                    Ok(_) => println!("\n✓ Tests terminés"),
                    Err(e) => eprintln!("\n✗ Erreur: {:?}", e),
                }
            }
            "34" => {
                #[cfg(target_os = "linux")]
                let i2c_mpu = match airhaum::hal::I2cLinux::nouveau(0) {
                    Ok(bus) => bus,
                    Err(e) => { eprintln!("❌ Erreur I²C: {:?}", e); continue; }
                };
                #[cfg(not(target_os = "linux"))]
                let i2c_mpu = airhaum::hal::i2c::I2cMock::nouveau();
                match airhaum::diagnostiques::diag_mpu9250::calibrer_gyro(i2c_mpu) {
                    Ok(_) => println!("\n✓ Calibration gyro terminée"),
                    Err(e) => eprintln!("\n✗ Erreur: {:?}", e),
                }
            }
            "35" => {
                #[cfg(target_os = "linux")]
                let i2c_mpu = match airhaum::hal::I2cLinux::nouveau(0) {
                    Ok(bus) => bus,
                    Err(e) => { eprintln!("❌ Erreur I²C: {:?}", e); continue; }
                };
                #[cfg(not(target_os = "linux"))]
                let i2c_mpu = airhaum::hal::i2c::I2cMock::nouveau();
                match airhaum::diagnostiques::diag_mpu9250::calibrer_mag(i2c_mpu) {
                    Ok(_) => println!("\n✓ Calibration mag terminée"),
                    Err(e) => eprintln!("\n✗ Erreur: {:?}", e),
                }
            }
            "36" => {
                #[cfg(target_os = "linux")]
                let i2c_mpu = match airhaum::hal::I2cLinux::nouveau(0) {
                    Ok(bus) => bus,
                    Err(e) => { eprintln!("❌ Erreur I²C: {:?}", e); continue; }
                };
                #[cfg(not(target_os = "linux"))]
                let i2c_mpu = airhaum::hal::i2c::I2cMock::nouveau();
                match airhaum::diagnostiques::diag_mpu9250::diagnostic_complet(i2c_mpu) {
                    Ok(_) => println!("\n✓ Diagnostic terminé"),
                    Err(e) => eprintln!("\n✗ Erreur: {:?}", e),
                }
            }
            "37" => {
                #[cfg(target_os = "linux")]
                let i2c_freq = match airhaum::hal::I2cLinux::nouveau(0) {
                    Ok(bus) => bus,
                    Err(e) => { eprintln!("❌ Erreur I²C: {:?}", e); continue; }
                };
                #[cfg(not(target_os = "linux"))]
                let i2c_freq = airhaum::hal::i2c::I2cMock::nouveau();

                match airhaum::diagnostiques::diag_mpu9250::test_frequence(i2c_freq, 100) {
                    Ok(stats) => {
                        println!("\n✓ MPU9250 : {:.2} Hz (jitter ±{:.2} ms)", stats.hz_moyen, stats.jitter_ms);
                    }
                    Err(e) => eprintln!("\n✗ Erreur fréquence MPU9250: {:?}", e),
                }
            }

            "tc" => {
                match airhaum::diagnostiques::diag_taches_capteurs::test_capteurs_simultanes(10).await {
                    Ok(_) => println!("\n✓ Test capteurs simultanés terminé"),
                    Err(e) => eprintln!("\n✗ Erreur: {:?}", e),
                }
            }

            "t" => {
                // Test de tous les capteurs
                println!("\n=== Test de tous les capteurs I²C ===\n");
                
                // BMP280
                match airhaum::diagnostiques::diag_bmp280::tester_bmp280() {
                    Ok(_) => println!("✓ BMP280 OK"),
                    Err(e) => eprintln!("✗ BMP280 Erreur: {:?}", e),
                }
                
                // VL53L0X
                #[cfg(target_os = "linux")]
                let i2c_vl53 = match airhaum::hal::I2cLinux::nouveau(0) {
                    Ok(bus) => bus,
                    Err(e) => {
                        eprintln!("❌ Erreur I²C pour VL53L0X: {:?}", e);
                        continue;
                    }
                };
                
                #[cfg(not(target_os = "linux"))]
                let i2c_vl53 = airhaum::hal::i2c::I2cMock::nouveau();
                
                match airhaum::diagnostiques::diag_vl53l0x::diagnostic_complet(i2c_vl53) {
                    Ok(_) => println!("✓ VL53L0X OK"),
                    Err(e) => eprintln!("✗ VL53L0X Erreur: {:?}", e),
                }
                
                println!("\n=== Tests terminés ===");
            }
            "q" | "Q" => {
                println!("\nAu revoir !");
                break;
            }
            _ => {
                println!("\n⚠ Choix invalide");
            }
        }
        
        println!("\nAppuyez sur Entrée pour continuer...");
        let mut pause = String::new();
        io::stdin().read_line(&mut pause).unwrap();
    }
}
