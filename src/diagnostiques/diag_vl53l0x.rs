// src/diagnostiques/vl53l0x.rs
//! Module de diagnostic pour le capteur VL53L0X

use crate::hal::BusI2c;
use crate::drivers::telemetre::vl53l0x::{Vl53l0x, ADRESSE_VL53L0X};
use crate::interfaces::telemetre::Telemetre;
use crate::types::Result;

/// Test de communication basique avec le VL53L0X
pub fn test_communication<I2C: BusI2c>(i2c: &mut I2C) -> Result<()> {
    println!("\n=== Test de communication VL53L0X ===");
    
    let mut vl53 = Vl53l0x::nouveau(i2c, ADRESSE_VL53L0X);
    
    // Vérifier l'identité du capteur
    print!("Vérification de l'identité du capteur... ");
    match vl53.verifier_identite() {
        Ok(true) => {
            println!("✓ VL53L0X détecté (ID: 0xEE)");
            Ok(())
        }
        Ok(false) => {
            println!("✗ ID incorrect");
            Err(crate::types::ErreursAirHaum::ErreurInitialisation(
                "VL53L0X: ID incorrect".into()
            ))
        }
        Err(e) => {
            println!("✗ Erreur de communication");
            Err(e)
        }
    }
}

/// Test d'initialisation complète du VL53L0X
pub fn test_initialisation<I2C: BusI2c>(i2c: I2C) -> Result<()> {
    println!("\n=== Test d'initialisation VL53L0X ===");
    
    let mut vl53 = Vl53l0x::nouveau(i2c, ADRESSE_VL53L0X);
    
    print!("Initialisation du capteur... ");
    match vl53.initialiser() {
        Ok(_) => {
            println!("✓ Initialisation réussie");
            
            println!("\nCaractéristiques du capteur:");
            println!("  - Portée max: {} mm", vl53.obtenir_portee_max());
            println!("  - Précision: ±{} mm", vl53.obtenir_precision());
            
            Ok(())
        }
        Err(e) => {
            println!("✗ Échec: {:?}", e);
            Err(e)
        }
    }
}

/// Test de mesure unique
pub fn test_mesure_unique<I2C: BusI2c>(i2c: I2C) -> Result<()> {
    println!("\n=== Test de mesure unique VL53L0X ===");
    
    let mut vl53 = Vl53l0x::nouveau(i2c, ADRESSE_VL53L0X);
    
    // Initialiser d'abord
    print!("Initialisation... ");
    vl53.initialiser()?;
    println!("✓");
    
    // Effectuer une mesure
    print!("Mesure de distance... ");
    match vl53.mesurer_distance() {
        Ok(distance) => {
            println!("✓");
            println!("Distance mesurée: {} mm ({:.2} m)", distance, distance as f32 / 1000.0);
            
            if distance > vl53.obtenir_portee_max() {
                println!("⚠ Attention: hors de portée");
            }
            
            Ok(())
        }
        Err(e) => {
            println!("✗ Échec: {:?}", e);
            Err(e)
        }
    }
}

/// Test de mesures continues
pub fn test_mesures_continues<I2C: BusI2c>(i2c: I2C, nombre_mesures: usize) -> Result<()> {
    println!("\n=== Test de mesures continues VL53L0X ===");
    println!("Nombre de mesures: {}", nombre_mesures);
    
    let mut vl53 = Vl53l0x::nouveau(i2c, ADRESSE_VL53L0X);
    
    // Initialiser
    print!("Initialisation... ");
    vl53.initialiser()?;
    println!("✓");
    
    println!("\nDébut des mesures:");
    println!("{:>5} | {:>10} | {:>10}", "N°", "Distance", "Temps");
    println!("{:-<5}-+-{:-<10}-+-{:-<10}", "", "", "");
    
    let mut distances = Vec::new();
    
    for i in 0..nombre_mesures {
        let debut = std::time::Instant::now();
        
        match vl53.mesurer_distance() {
            Ok(distance) => {
                let duree = debut.elapsed().as_millis();
                println!("{:>5} | {:>8} mm | {:>8} ms", i + 1, distance, duree);
                distances.push(distance);
            }
            Err(e) => {
                println!("{:>5} | ERREUR: {:?}", i + 1, e);
            }
        }
        
        // Pause entre les mesures
        std::thread::sleep(std::time::Duration::from_millis(100));
    }
    
    // Statistiques
    if !distances.is_empty() {
        println!("\n--- Statistiques ---");
        let min = *distances.iter().min().unwrap();
        let max = *distances.iter().max().unwrap();
        let moyenne = distances.iter().sum::<u16>() as f32 / distances.len() as f32;
        
        println!("Minimum:  {} mm", min);
        println!("Maximum:  {} mm", max);
        println!("Moyenne:  {:.1} mm", moyenne);
        println!("Écart:    {} mm", max - min);
        println!("Mesures réussies: {}/{}", distances.len(), nombre_mesures);
    }
    
    Ok(())
}

/// Diagnostic complet du VL53L0X
pub fn diagnostic_complet<I2C: BusI2c>(i2c: I2C) -> Result<()> {
    println!("\n╔════════════════════════════════════════╗");
    println!("║   DIAGNOSTIC COMPLET VL53L0X           ║");
    println!("╚════════════════════════════════════════╝");
    
    let mut vl53 = Vl53l0x::nouveau(i2c, ADRESSE_VL53L0X);
    
    // Test 1: Vérification identité
    println!("\n[1/4] Vérification de l'identité...");
    match vl53.verifier_identite() {
        Ok(true) => println!("  ✓ VL53L0X détecté correctement"),
        Ok(false) => {
            println!("  ✗ ID du capteur incorrect");
            return Err(crate::types::ErreursAirHaum::ErreurInitialisation(
                "ID incorrect".into()
            ));
        }
        Err(e) => {
            println!("  ✗ Erreur de communication: {:?}", e);
            return Err(e);
        }
    }
    
    // Test 2: Initialisation
    println!("\n[2/4] Initialisation du capteur...");
    match vl53.initialiser() {
        Ok(_) => println!("  ✓ Initialisation réussie"),
        Err(e) => {
            println!("  ✗ Échec initialisation: {:?}", e);
            return Err(e);
        }
    }
    
    // Test 3: Mesure unique
    println!("\n[3/4] Test de mesure unique...");
    match vl53.mesurer_distance() {
        Ok(distance) => {
            println!("  ✓ Mesure réussie: {} mm", distance);
        }
        Err(e) => {
            println!("  ✗ Échec mesure: {:?}", e);
            return Err(e);
        }
    }
    
    // Test 4: Série de mesures
    println!("\n[4/4] Série de 5 mesures rapides...");
    let mut succes = 0;
    for i in 0..5 {
        match vl53.mesurer_distance() {
            Ok(distance) => {
                println!("  Mesure {}: {} mm", i + 1, distance);
                succes += 1;
            }
            
            Err(e) => {
                println!("  Mesure {}: ERREUR - {:?}", i + 1, e);
            }
        }
        std::thread::sleep(std::time::Duration::from_millis(50));
    }
    
    println!("\n╔════════════════════════════════════════╗");
    println!("║   RÉSULTAT: {}/4 tests réussis         ║", if succes == 5 { 4 } else { 3 });
    println!("╚════════════════════════════════════════╝");
    
    Ok(())
}
