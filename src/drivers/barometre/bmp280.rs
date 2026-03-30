// src/drivers/barometre/bmp280.rs
// Driver pour le capteur de pression/température BMP280 (Bosch)

#![allow(dead_code)]
#![allow(unused_imports)]

use crate::hal::i2c::BusI2c;
use crate::interfaces::barometre::Barometre;
use crate::types::{Result, ErreursAirHaum, DonneesBarometre, Pression, Temperature, Horodatage, EtatCapteur};
use crate::drivers::barometre::calibration::CalibrationBarometre;

/// Adresse I²C par défaut du BMP280
pub const ADRESSE_BMP280: u8 = 0x76; // Peut être 0x77 selon le câblage SDO

/// Registres du BMP280
mod registres {
    pub const ID: u8 = 0xD0;           // Chip ID (devrait être 0x58)
    pub const ID_VALEUR_ATTENDUE: u8 = 0x58; 
    pub const RESET: u8 = 0xE0;        // Soft reset
    pub const RESET_COMMANDE: u8 = 0xB6;
    pub const STATUS: u8 = 0xF3;       // Status
    pub const CTRL_MEAS: u8 = 0xF4;    // Contrôle mesure
    pub const CONFIG: u8 = 0xF5;       // Configuration
    pub const PRESS_MSB: u8 = 0xF7;    // Pression MSB
    pub const PRESS_LSB: u8 = 0xF8;    // Pression LSB
    pub const PRESS_XLSB: u8 = 0xF9;   // Pression XLSB
    pub const TEMP_MSB: u8 = 0xFA;     // Température MSB
    pub const TEMP_LSB: u8 = 0xFB;     // Température LSB
    pub const TEMP_XLSB: u8 = 0xFC;    // Température XLSB
    
    // Registres de calibration (0x88 - 0x9F)
    pub const CALIB_START: u8 = 0x88;
}

/// Valeurs pour le registre CTRL_MEAS
mod ctrl {
    // Oversampling température
    pub const OSRS_T_SKIP: u8 = 0b000 << 5;
    pub const OSRS_T_X1: u8 = 0b001 << 5;
    pub const OSRS_T_X2: u8 = 0b010 << 5;
    pub const OSRS_T_X16: u8 = 0b101 << 5;
    
    // Oversampling pression
    pub const OSRS_P_SKIP: u8 = 0b000 << 2;
    pub const OSRS_P_X1: u8 = 0b001 << 2;
    pub const OSRS_P_X2: u8 = 0b010 << 2;
    pub const OSRS_P_X16: u8 = 0b101 << 2;
    
    // Mode
    pub const MODE_SLEEP: u8 = 0b00;
    pub const MODE_FORCED: u8 = 0b01;
    pub const MODE_NORMAL: u8 = 0b11;
}

/// Coefficients de calibration du BMP280
/// Ces valeurs sont uniques à chaque capteur et stockées en usine
#[derive(Debug, Clone, Copy)]
struct Calibration {
    dig_t1: u16,
    dig_t2: i16,
    dig_t3: i16,
    dig_p1: u16,
    dig_p2: i16,
    dig_p3: i16,
    dig_p4: i16,
    dig_p5: i16,
    dig_p6: i16,
    dig_p7: i16,
    dig_p8: i16,
    dig_p9: i16,
}

impl Calibration {
    /// Charge les coefficients depuis le capteur
    fn lire_depuis_capteur<I: BusI2c>(i2c: &mut I) -> Result<Self> {
        let mut buffer = [0u8; 24];
        i2c.ecrire_lire(ADRESSE_BMP280, &[registres::CALIB_START], &mut buffer)?;
        
        let calib = Self {
            dig_t1: u16::from_le_bytes([buffer[0], buffer[1]]),
            dig_t2: i16::from_le_bytes([buffer[2], buffer[3]]),
            dig_t3: i16::from_le_bytes([buffer[4], buffer[5]]),
            dig_p1: u16::from_le_bytes([buffer[6], buffer[7]]),
            dig_p2: i16::from_le_bytes([buffer[8], buffer[9]]),
            dig_p3: i16::from_le_bytes([buffer[10], buffer[11]]),
            dig_p4: i16::from_le_bytes([buffer[12], buffer[13]]),
            dig_p5: i16::from_le_bytes([buffer[14], buffer[15]]),
            dig_p6: i16::from_le_bytes([buffer[16], buffer[17]]),
            dig_p7: i16::from_le_bytes([buffer[18], buffer[19]]),
            dig_p8: i16::from_le_bytes([buffer[20], buffer[21]]),
            dig_p9: i16::from_le_bytes([buffer[22], buffer[23]]),
        };
        
        // Validation des plages (selon datasheet)
        if calib.dig_t1 == 0 || calib.dig_p1 == 0 {
            return Err(ErreursAirHaum::CalibrationEchouee(
                "Coefficients invalides (zéros)".into()
            ));
        }

        Ok(calib)
    }
    
    /// Compense la température (retourne la température en °C * 100 et t_fine)
    fn compenser_temperature(&self, adc_t: i32) -> (i32, i32) {
        let var1 = ((((adc_t >> 3) - ((self.dig_t1 as i32) << 1))) * (self.dig_t2 as i32)) >> 11;
        let var2 = (((((adc_t >> 4) - (self.dig_t1 as i32)) * 
                      ((adc_t >> 4) - (self.dig_t1 as i32))) >> 12) * 
                     (self.dig_t3 as i32)) >> 14;
        let t_fine = var1 + var2;
        let t = (t_fine * 5 + 128) >> 8;
        (t, t_fine)
    }
    
    /// Compense la pression (nécessite t_fine de la compensation température)
    fn compenser_pression(&self, adc_p: i32, t_fine: i32) -> u32 {
        let mut var1: i64 = (t_fine as i64) - 128000;
        let mut var2: i64 = var1 * var1 * (self.dig_p6 as i64);
        var2 = var2 + ((var1 * (self.dig_p5 as i64)) << 17);
        var2 = var2 + ((self.dig_p4 as i64) << 35);
        var1 = ((var1 * var1 * (self.dig_p3 as i64)) >> 8) + 
               ((var1 * (self.dig_p2 as i64)) << 12);
        var1 = ((((1i64) << 47) + var1)) * (self.dig_p1 as i64) >> 33;
        
        if var1 == 0 {
            return 0; // Évite division par zéro
        }
        
        let mut p: i64 = 1048576 - (adc_p as i64);
        p = (((p << 31) - var2) * 3125) / var1;
        var1 = ((self.dig_p9 as i64) * (p >> 13) * (p >> 13)) >> 25;
        var2 = ((self.dig_p8 as i64) * p) >> 19;
        p = ((p + var1 + var2) >> 8) + ((self.dig_p7 as i64) << 4);
        
        //(p / 256) as u32
        (p >> 8) as u32  // ← Format Q24.8 → Pascals
    }
}

/// Driver BMP280
pub struct Bmp280<I: BusI2c> {
    i2c: I,
    calibration: Option<Calibration>,                   // Calibration usine (ROM)
    calibration_systeme: Option<CalibrationBarometre>,  // Calibration pré-vol
    etat: EtatCapteur,                                  // Etat de la machine à état
    config_ctrl_meas: u8,  
    derniere_pression: Option<f32>,                     // Pour détecter variations anormales
    derniere_lecture: Option<Horodatage>,
}

impl<I: BusI2c> Bmp280<I> {
    /// Crée une nouvelle instance du driver
    pub fn nouveau(i2c: I) -> Self {
        Self {
            i2c,
            calibration: None,
            calibration_systeme: None,  
            etat: EtatCapteur::Inconnu,
            config_ctrl_meas: ctrl::OSRS_T_X16 | ctrl::OSRS_P_X16, 
            derniere_pression: None,
            derniere_lecture: None,
        }
    }

   /// Déclenche une mesure en mode FORCED
   fn declencher_mesure(&mut self) -> Result<()> {
      let ctrl = self.config_ctrl_meas | ctrl::MODE_FORCED;
      self.i2c.ecrire_registre_u8(ADRESSE_BMP280, registres::CTRL_MEAS, ctrl)?;
      Ok(())
   }
    
    /// Vérifie l'ID du chip
    fn verifier_id(&mut self) -> Result<()> {
        let id = self.i2c.lire_registre_u8(ADRESSE_BMP280, registres::ID)?;
        if id != registres::ID_VALEUR_ATTENDUE { 
            return Err(ErreursAirHaum::DonneesInvalides(
                 format!("BMP280 ID invalide: 0x{:02X} (attendu 0x{:02X})", 
                    id, registres::ID_VALEUR_ATTENDUE)
           ));
        }
        Ok(())
    }
    
    /// Reset logiciel du capteur
    fn reset(&mut self) -> Result<()> {
        self.i2c.ecrire_registre_u8(ADRESSE_BMP280, registres::RESET, 0xB6)?;
        std::thread::sleep(std::time::Duration::from_millis(10));
        Ok(())
    }
   

   fn attendre_conversion(&mut self) -> Result<()> {
      const MAX_ATTENTE_MS: u64 = 100; // Temps max selon datasheet 43.2 à 58ms en forced
      const INTERVALLE_POLL_US: u64 = 500; // Polling toutes les 500µs réduction charge I2C

      let debut = std::time::Instant::now();

      while debut.elapsed().as_millis() < MAX_ATTENTE_MS as u128 {

         //for _ in 0..MAX_POLL {
         //std::thread::sleep(std::time::Duration::from_millis(5));
         let status = self.i2c.lire_registre_u8(ADRESSE_BMP280, registres::STATUS)?;

         // Bit 3 = measuring
         if (status & 0b0000_1000) == 0 {
            return Ok(());   
         }
         std::thread::sleep(std::time::Duration::from_micros(INTERVALLE_POLL_US)); 
      }

   Err(ErreursAirHaum::TimeoutCapteur("BMP280".into()))
   }


 
    /// Lit les données brutes ADC (température + pression)
    fn lire_donnees_brutes(&mut self) -> Result<(i32, i32)> {
       // NOTE: attendre_conversion() doit être appelé AVANT cette fonction
       // par la fonction lire() après declencher_mesure()
 
       // Lit 6 octets depuis PRESS_MSB
        let mut buffer = [0u8; 6];
        self.i2c.ecrire_lire(ADRESSE_BMP280, &[registres::PRESS_MSB], &mut buffer)?;
        
        // Reconstruit les valeurs ADC 20-bit
        let adc_p = ((buffer[0] as i32) << 12) | ((buffer[1] as i32) << 4) | ((buffer[2] as i32) >> 4);
        let adc_t = ((buffer[3] as i32) << 12) | ((buffer[4] as i32) << 4) | ((buffer[5] as i32) >> 4);
        
        // Vérifier que les conversions sont terminées
        //const ADC_INVALID: i32 = 0x80000;
        //if adc_p == ADC_INVALID || adc_t == ADC_INVALID {
        //   return Err(ErreursAirHaum::LectureCapteurEchouee(
        //      "BMP280: conversion non terminée".into()
        //   ));
        //}
        Ok((adc_t, adc_p))
    }


    /// Détecte si une configuration valide existe déjà
    fn detecter_configuration_valide(&mut self) -> Result<bool> {
        // 1. Vérifier l'ID
        if self.verifier_id().is_err() {
            return Ok(false);
        }
        
        // 2. Lire le registre de contrôle
        let ctrl = self.i2c.lire_registre_u8(ADRESSE_BMP280, registres::CTRL_MEAS)?;
        
        // 3. Vérifier qu'il correspond à notre config attendue (masquer les bits de mode)
        let config_valide = (ctrl & 0b11111100) == self.config_ctrl_meas;
        
        Ok(config_valide)
    }


    /// Purge les buffers du capteur
    fn purger_buffers(&mut self) -> Result<()> {
        // Déclencher et lire une mesure pour vider les buffers
        self.declencher_mesure()?;
        self.attendre_conversion()?;
        let _ = self.lire_donnees_brutes()?;
        Ok(())
    }


/// Initialisation complète (reset + config + chargement calibration)
fn initialisation_complete(&mut self) -> Result<()> {
    use std::time::{Duration, Instant};
    use crate::types::constantes::TIMEOUT_INIT_BMP280_MS;
    
    let debut = Instant::now();
    let timeout = Duration::from_millis(TIMEOUT_INIT_BMP280_MS);
    
    // 1. Vérifier l'ID
    self.verifier_id()?;
    if debut.elapsed() > timeout {
        self.etat = EtatCapteur::Inconnu;
        return Err(ErreursAirHaum::Timeout);
    }
    self.etat = EtatCapteur::NonConfigure;
    
    // 2. Reset
    self.reset()?;
    if debut.elapsed() > timeout {
        self.etat = EtatCapteur::Inconnu;
        return Err(ErreursAirHaum::Timeout);
    }
    
    // 3. Charger la calibration usine (ROM)
    self.calibration = Some(Calibration::lire_depuis_capteur(&mut self.i2c)?);
    if debut.elapsed() > timeout {
        self.etat = EtatCapteur::Inconnu;
        return Err(ErreursAirHaum::Timeout);
    }
    
    // 4. Configurer le capteur
    self.i2c.ecrire_registre_u8(ADRESSE_BMP280, registres::CONFIG, 0b000_000_00)?;
    let ctrl = ctrl::OSRS_T_X16 | ctrl::OSRS_P_X16 | ctrl::MODE_SLEEP;
    self.i2c.ecrire_registre_u8(ADRESSE_BMP280, registres::CTRL_MEAS, ctrl)?;
    
    self.etat = EtatCapteur::Configure;
    
    // 5. Test de lecture pour validation matérielle
    let _ = self.lire_interne()?;
    
    // 6. ← NOUVEAU : Tenter de charger la calibration système depuis flash
    match crate::systeme::calibration::gestionnaire()
        .charger::<CalibrationBarometre>() 
    {
        Ok(Some(calib)) => {
            // Calibration trouvée et valide
            self.calibration_systeme = Some(calib);
            self.etat = EtatCapteur::nouveau_operationnel();
            println!("✓ BMP280 opérationnel avec calibration chargée (P_ref={:.1} hPa)", 
                     calib.obtenir_pression_reference() / 100.0);
        }
        Ok(None) => {
            // Pas de calibration ou expirée
            self.etat = EtatCapteur::Configure;
            println!("⚠ BMP280 configuré mais nécessite calibration pré-vol");
        }
        Err(e) => {
            // Erreur de lecture du fichier (non bloquant)
            eprintln!("⚠ Erreur chargement calibration BMP280: {:?}", e);
            self.etat = EtatCapteur::Configure;
            println!("⚠ BMP280 configuré mais nécessite calibration pré-vol");
        }
    }
    
    Ok(())
}



/// Calibre la pression de référence au sol
///
/// Cette méthode doit être appelée au sol, immobile, avant le vol.
/// Elle mesure la pression actuelle et la stocke comme référence pour
/// le calcul d'altitude relative pendant le vol.
///
/// # Arguments
///
/// * `validite_sec` - Durée de validité de la calibration en secondes
///                    Recommandations :
///                    - Vol court (< 30 min) : 3600 s (1 heure)
///                    - Vol moyen : 1800 s (30 minutes)
///                    - Conditions météo instables : 600 s (10 minutes)
///
/// # Erreurs
///
/// Retourne une erreur si :
/// - Le capteur n'est pas configuré (état Inconnu ou NonConfiguré)
/// - La lecture de pression échoue
/// - L'écriture du fichier de calibration échoue
///
/// # Exemple
///
/// ```ignore
/// // Au sol, avant armement
/// bmp280.calibrer_pression_sol(3600)?;
/// println!("✓ Baromètre calibré et prêt pour le vol");
/// ```
pub fn calibrer_pression_sol(&mut self, validite_sec: u64) -> Result<()> {
    // Vérifier que le capteur est au moins configuré
    if matches!(self.etat, EtatCapteur::Inconnu | EtatCapteur::NonConfigure) {
        return Err(ErreursAirHaum::CapteurNonInitialise(
            "BMP280 doit être configuré avant calibration".into()
        ));
    }
    
    println!("🔧 Calibration BMP280 en cours...");
    
    // Effectuer plusieurs lectures pour moyenner et réduire le bruit
    let nb_lectures = 10;
    let mut somme_pression = 0.0;
    
    for i in 0..nb_lectures {
        let donnees = self.lire_interne()?;
        somme_pression += donnees.pression.pascals();
        
        if i < nb_lectures - 1 {
            std::thread::sleep(std::time::Duration::from_millis(50));
        }
    }
    
    let pression_moyenne = somme_pression / nb_lectures as f32;
    
    // Créer la calibration
    let calib = CalibrationBarometre::nouvelle(pression_moyenne, validite_sec);
    
    // Sauvegarder en flash
    crate::systeme::calibration::gestionnaire()
        .sauvegarder(&calib)?;
    
    // Stocker localement
    self.calibration_systeme = Some(calib);
    
    // Passer en état Opérationnel
    self.etat = EtatCapteur::nouveau_operationnel();
    
    println!("✓ BMP280 calibré : P_ref = {:.2} hPa ({:.1} m équivalent)", 
             pression_moyenne / 100.0,
             44330.0 * (1.0 - (pression_moyenne / 101325.0_f32).powf(0.1903)));
    
    Ok(())
}




/// Calcule l'altitude relative en utilisant la calibration système
///
/// Utilise la formule barométrique internationale avec la pression de référence
/// établie lors de la calibration pré-vol.
///
/// # Erreurs
///
/// Retourne une erreur si :
/// - Le capteur n'est pas opérationnel
/// - Aucune calibration système n'est chargée
/// - La lecture de pression échoue
///
/// # Retour
///
/// Altitude en mètres au-dessus du point de calibration
///
/// # Exemple
///
/// ```ignore
/// loop {
///     match bmp280.altitude_relative() {
///         Ok(alt) => println!("Altitude: {:.1} m", alt),
///         Err(e) => eprintln!("Erreur: {:?}", e),
///     }
///     std::thread::sleep(Duration::from_millis(100));
/// }
/// ```
pub fn altitude_relative(&mut self) -> Result<f32> {
    // 1. Extraire la pression de référence AVANT tout emprunt mutable
    let p_ref = self.calibration_systeme.as_ref()
        .ok_or_else(|| {
            ErreursAirHaum::CalibrationEchouee(
                "Aucune calibration système - appeler calibrer_pression_sol() avant le vol".into()
            )
        })?
        .obtenir_pression_reference();  // ← Obtenir la valeur (f32) immédiatement
    
    // 2. Maintenant on peut emprunter self mutablement
    let donnees = self.lire()?;
    
    // 3. Calculer l'altitude
    Ok(donnees.pression.vers_altitude(Pression::depuis_pascals(p_ref)))

}



/// Invalide la calibration système
///
/// Supprime la calibration du capteur ET du fichier de stockage persistant.
/// Le capteur repasse en état Configuré et nécessitera une nouvelle calibration
/// avant d'être opérationnel.
///
/// À appeler typiquement lors du désarmement pour forcer une recalibration
/// au prochain vol.
///
/// # Exemple
///
/// ```ignore
/// // Lors du désarmement ou changement de terrain
/// bmp280.invalider_calibration()?;
/// println!("Calibration invalidée - recalibration nécessaire au prochain vol");
/// ```
pub fn invalider_calibration(&mut self) -> Result<()> {
    // Supprimer la calibration locale
    self.calibration_systeme = None;
    
    // Supprimer le fichier
    match crate::systeme::calibration::gestionnaire()
        .supprimer::<CalibrationBarometre>() 
    {
        Ok(_) => {},
        Err(e) => {
            eprintln!("⚠ Erreur suppression fichier calibration: {:?}", e);
            // Non bloquant, on continue
        }
    }
    
    // Repasser en Configure si on était Opérationnel
    if self.etat.est_utilisable() {
        self.etat = EtatCapteur::Configure;
    }
    
    println!("⚠ Calibration BMP280 invalidée");
    Ok(())
}

/// Obtient la pression de référence actuelle (si calibration chargée)
///
/// Utile pour l'affichage ou la journalisation.
///
/// # Retour
///
/// - `Some(pression)` si une calibration est chargée
/// - `None` si aucune calibration
pub fn obtenir_pression_reference(&self) -> Option<f32> {
    self.calibration_systeme.as_ref()
        .map(|c| c.obtenir_pression_reference())
}

/// Vérifie si le capteur a une calibration système valide
pub fn a_calibration_systeme(&self) -> bool {
    self.calibration_systeme.is_some()
}







     /// Vérifie la cohérence des données
    fn verifier_coherence(&self, donnees: &DonneesBarometre) -> bool {
        use crate::types::constantes::*;
        
        // Vérifier les plages de valeurs
        let pression_ok = donnees.pression.pascals() >= PRESSION_MIN_PA 
                       && donnees.pression.pascals() <= PRESSION_MAX_PA;
        
        let temp_ok = donnees.temperature.celsius() >= TEMP_MIN_BMP280_C 
                   && donnees.temperature.celsius() <= TEMP_MAX_BMP280_C;
        
        if !pression_ok || !temp_ok {
            return false;
        }
        
        // Vérifier les variations (si on a une lecture précédente)
        if let (Some(derniere_p), Some(derniere_lecture)) = (self.derniere_pression, self.derniere_lecture) {
            let dt = donnees.horodatage.delta_secondes(derniere_lecture);
            if dt > 0.0 {
                let variation_pa_s = (donnees.pression.pascals() - derniere_p).abs() / dt;
                if variation_pa_s > VARIATION_PRESSION_MAX_PA_S {
                    return false;
                }
            }
        }
        
        true
    }


    /// Lecture interne (sans vérification d'état)
    fn lire_interne(&mut self) -> Result<DonneesBarometre> {
        let calib = self.calibration.ok_or_else(|| {
            ErreursAirHaum::CalibrationEchouee("Calibration non chargée".into())
        })?;
        
        // Déclenche une mesure en mode FORCED
        self.declencher_mesure()?;
        self.attendre_conversion()?;
        
        // Lire les données ADC
        let (adc_t, adc_p) = self.lire_donnees_brutes()?;
        
        // Compenser température
        let (temp_compensee, t_fine) = calib.compenser_temperature(adc_t);
        let temperature = Temperature::depuis_celsius(temp_compensee as f32 / 100.0);
        
        // Compenser pression
        let press_compensee = calib.compenser_pression(adc_p, t_fine);
        let pression = Pression::depuis_pascals(press_compensee as f32);
        
        Ok(DonneesBarometre {
            horodatage: Horodatage::maintenant(),
            pression,
            temperature,
        })
    }


}


impl<I: BusI2c> Barometre for Bmp280<I> {

    fn initialiser(&mut self) -> Result<()> {
        // Tentative de reprise rapide si configuration valide détectée
        if self.detecter_configuration_valide()? {
            println!("⚡ BMP280: Reprise rapide détectée");
            self.calibration = Some(Calibration::lire_depuis_capteur(&mut self.i2c)?);
            self.purger_buffers()?;
            self.etat = EtatCapteur::nouveau_operationnel();
            return Ok(());
        }
        
        // Sinon, initialisation complète
        println!("🔧 BMP280: Initialisation complète");
        self.initialisation_complete()
    }



    fn lire(&mut self) -> Result<DonneesBarometre> {
        // Vérifier l'état avant de lire
        if !self.etat.est_utilisable() {
            return Err(ErreursAirHaum::CapteurNonInitialise(
                format!("BMP280 non opérationnel (état: {})", self.etat)
            ));
        }
        
        // Effectuer la lecture
        let donnees = self.lire_interne()?;
        
        // Vérifier la cohérence
        if !self.verifier_coherence(&donnees) {
            self.etat = EtatCapteur::nouveau_degrade(
                "Données hors plages acceptables ou variation anormale"
            );
            return Err(ErreursAirHaum::DonneesInvalides(
                format!("BMP280: P={:.1} Pa, T={:.1}°C", 
                    donnees.pression.pascals(),
                    donnees.temperature.celsius())
            ));
        }
        
        // Mémoriser pour prochaine vérification
        self.derniere_pression = Some(donnees.pression.pascals());
        self.derniere_lecture = Some(donnees.horodatage);
        
        Ok(donnees)
    }  
  




 
    fn configurer_frequence(&mut self, _frequence_hz: u32) -> Result<()> {
        // Le BMP280 en mode normal échantillonne automatiquement
        // La fréquence dépend de l'oversampling et du standby time
        // Pour l'instant, on garde la config par défaut
        Ok(())
    }
    
   fn obtenir_etat(&self) -> &EtatCapteur {
       &self.etat
   }    

}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::hal::i2c::I2cMock;
    
    fn creer_bmp280_mock() -> Bmp280<I2cMock> {
        let mut i2c = I2cMock::nouveau();
        
        // Simuler l'ID du BMP280
        i2c.precharger_registre(ADRESSE_BMP280, registres::ID, 0x58);
        
        // Simuler des coefficients de calibration (valeurs réalistes)
        let calib_data = [
            0x88, 0x6E, // dig_T1 = 28296
            0x6A, 0x66, // dig_T2 = 26218
            0x32, 0x00, // dig_T3 = 50
            0xC0, 0x8E, // dig_P1 = 36544
            0xC8, 0xD6, // dig_P2 = -10552
            0xD0, 0x0B, // dig_P3 = 3024
            0x27, 0x0B, // dig_P4 = 2855
            0x00, 0x00, // dig_P5 = 0
            0xF9, 0xFF, // dig_P6 = -7
            0x8C, 0x3C, // dig_P7 = 15500
            0xF8, 0xC6, // dig_P8 = -14600
            0x70, 0x17, // dig_P9 = 6000
        ];
        
        for (i, &byte) in calib_data.iter().enumerate() {
            i2c.precharger_registre(ADRESSE_BMP280, registres::CALIB_START + i as u8, byte);
        }
        
        // Ces valeurs correspondent à environ 101325 Pa et 20°C
        // Format: 20 bits décalés de 4 bits (voir datasheet BMP280)
    
        // Pression ADC: ~415000 (décalé de 4 bits = 0x65666 en 20 bits)
        i2c.precharger_registre(ADRESSE_BMP280, registres::PRESS_MSB, 0x65);
        i2c.precharger_registre(ADRESSE_BMP280, registres::PRESS_LSB, 0x66);
        i2c.precharger_registre(ADRESSE_BMP280, registres::PRESS_XLSB, 0x60);
    
         // Température ADC: ~519888 (décalé de 4 bits = 0x7EDD0 en 20 bits)
        i2c.precharger_registre(ADRESSE_BMP280, registres::TEMP_MSB, 0x7E);
        i2c.precharger_registre(ADRESSE_BMP280, registres::TEMP_LSB, 0xDD);
        i2c.precharger_registre(ADRESSE_BMP280, registres::TEMP_XLSB, 0x00);

        Bmp280::nouveau(i2c)
    }
    
    #[test]
    fn test_bmp280_initialisation() {
        let mut bmp = creer_bmp280_mock();
        assert!(bmp.initialiser().is_ok());
        assert!(bmp.est_operationnel());
    }
    
    #[test]
    fn test_bmp280_id_invalide() {
        let mut i2c = I2cMock::nouveau();
        i2c.precharger_registre(ADRESSE_BMP280, registres::ID, 0xFF); // Mauvais ID
        
        let mut bmp = Bmp280::nouveau(i2c);
        assert!(bmp.initialiser().is_err());
   }    


   #[test]
   fn test_bmp280_lecture_donnees() {
       let mut bmp = creer_bmp280_mock();
       bmp.initialiser().unwrap();
     
       let donnees = bmp.lire().unwrap();
       //assert!(donnees.pression.pascals() > 80000.0);
       //assert!(donnees.pression.pascals() < 120000.0);
       // Vérifier que les valeurs sont dans des plages réalistes
       assert!(donnees.pression.pascals() > 80000.0, "Pression trop basse");
       assert!(donnees.pression.pascals() < 120000.0, "Pression trop haute");
       assert!(donnees.temperature.celsius() > -40.0, "Température trop basse");
       assert!(donnees.temperature.celsius() < 85.0, "Température trop haute");
  
   }

   #[test]
   fn test_bmp280_altitude_calcul() {
       let mut bmp = creer_bmp280_mock();
       bmp.initialiser().unwrap();

    
       // Test 1: Avec la pression standard au niveau de la mer
       let p_mer = Pression::niveau_mer_standard();
       let altitude = bmp.altitude_estimee(p_mer.pascals()).unwrap();
       // Tolérance de ±200m car c'est un mock avec des valeurs approximatives
       assert!(altitude.abs() < 200.0,
            "Altitude au niveau de la mer devrait être proche de 0m, obtenu: {:.1}m",
            altitude);


       // Test 2: Lire la pression du mock et l'utiliser comme référence
       let donnees = bmp.lire().unwrap();
       let pression_mesuree = donnees.pression.pascals();
    
       // À la même pression que la référence, altitude doit être 0
       let altitude_ref = bmp.altitude_estimee(pression_mesuree).unwrap();
       assert!(altitude_ref.abs() < 1.0,
            "Altitude à la pression de référence devrait être 0m, obtenu: {:.2}m",
            altitude_ref);
    
       // Test 3: Simuler qu'on est à 1000m d'altitude
       // Inversons la logique : on est au niveau mer, référence est à 1000m plus haut
       let altitude_inverse = bmp.altitude_estimee(90000.0).unwrap();
       // On devrait être environ 1000m SOUS la référence, donc altitude négative
       assert!(altitude_inverse < -900.0 && altitude_inverse > -1100.0,
            "Par rapport à une référence à 90000 Pa, on devrait être ~-1000m, obtenu: {:.1}m",
            altitude_inverse);

   }

   #[test]
   fn test_bmp280_donnees_consecutives() {
       let mut bmp = creer_bmp280_mock();
       bmp.initialiser().unwrap();
    
       // Vérifier que plusieurs lectures successives fonctionnent
       let d1 = bmp.lire().unwrap();
       let d2 = bmp.lire().unwrap();
    
       // Les valeurs devraient être identiques (mock statique)
       assert_eq!(d1.pression.pascals(), d2.pression.pascals());
       assert_eq!(d1.temperature.celsius(), d2.temperature.celsius());
   }


}
