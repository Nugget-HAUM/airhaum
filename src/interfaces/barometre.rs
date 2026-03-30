// src/interfaces/barometre.rs
// Trait définissant l'interface d'un baromètre

   #![allow(dead_code)]
   #![allow(unused_imports)]

/// Imports
use crate::types::{Result, DonneesBarometre};
use crate::types::EtatCapteur;


/// Interface générique pour un baromètre
/// 
/// Ce trait permet de découpler la logique métier des implémentations
/// hardware spécifiques (BMP280, MS5611, etc.)

pub trait Barometre: Send + Sync {
    /// Initialise le baromètre
    /// 
    /// Cette méthode doit :
    /// - Vérifier la communication avec le capteur
    /// - Configurer les registres appropriés
    /// - Charger les coefficients de calibration
    fn initialiser(&mut self) -> Result<()>;
   
    /// Lit les données du baromètre
    /// 
    /// Retourne la pression et la température actuelles.
    /// Cette méthode peut bloquer pendant la durée de conversion
    /// du capteur (typiquement quelques ms).
    fn lire(&mut self) -> Result<DonneesBarometre>;
   
    /// Configure la fréquence d'échantillonnage
    /// 
    /// # Arguments
    /// * `frequence_hz` - Fréquence souhaitée en Hz
    /// 
    /// Note : La fréquence réelle peut être limitée par le hardware
    fn configurer_frequence(&mut self, frequence_hz: u32) -> Result<()>;
 
    /// Obtient l'état actuel du capteur
    fn obtenir_etat(&self) -> &EtatCapteur;
   
    /// Vérifie si le capteur est opérationnel (nouvelle signature)
    fn est_operationnel(&self) -> bool {
      self.obtenir_etat().est_utilisable()
   }   
 
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::{Pression, Temperature, Horodatage};
    
    /// Mock pour tester le code qui utilise le trait Barometre
    struct BarometreMock {
        operationnel: bool,
        pression: f32,
    }
    
    impl BarometreMock {
        fn nouveau() -> Self {
            Self {
                operationnel: true,
                pression: 101325.0, // Pression standard au niveau de la mer
            }
        }
    }
    
    impl Barometre for BarometreMock {
        fn initialiser(&mut self) -> Result<()> {
            self.operationnel = true;
            Ok(())
        }
        
        fn lire(&mut self) -> Result<DonneesBarometre> {
            Ok(DonneesBarometre {
                horodatage: Horodatage::maintenant(),
                pression: Pression::depuis_pascals(self.pression),
                temperature: Temperature::depuis_celsius(20.0),
            })
        }
        
        fn configurer_frequence(&mut self, _frequence_hz: u32) -> Result<()> {
            Ok(())
        }
        
        fn est_operationnel(&self) -> bool {
            self.operationnel
        }
        
        fn altitude_estimee(&mut self, pression_ref: f32) -> Result<f32> {
            let alt = 44330.0 * (1.0 - (self.pression / pression_ref).powf(0.1903));
            Ok(alt)
        }
    }
    
    #[test]
    fn test_mock_barometre() {
        let mut baro = BarometreMock::nouveau();
        assert!(baro.initialiser().is_ok());
        assert!(baro.est_operationnel());
        
        let donnees = baro.lire().unwrap();
        assert_eq!(donnees.pression.pascals(), 101325.0);
    }
    
    #[test]
    fn test_altitude_niveau_mer() {
        let mut baro = BarometreMock::nouveau();
        let alt = baro.altitude_estimee(101325.0).unwrap();
        assert!(alt.abs() < 1.0); // ~0m au niveau de la mer
    }
}
