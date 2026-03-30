// src/hal/i2c.rs
// Abstraction générique du bus I²C

   #![allow(dead_code)]
   #![allow(unused_imports)]

use crate::types::{Result, ErreursAirHaum};

/// Trait pour l'abstraction d'un bus I²C
/// 
/// Cette abstraction permet de :
/// - Découpler le code métier du hardware spécifique
/// - Faciliter les tests avec des mocks
/// - Supporter différents backends (linux-i2c, embedded-hal, etc.)
pub trait BusI2c: Send + Sync {
    /// Écrit des données vers un périphérique I²C
    /// 
    /// # Arguments
    /// * `adresse` - Adresse 7-bit du périphérique
    /// * `donnees` - Données à écrire
    fn ecrire(&mut self, adresse: u8, donnees: &[u8]) -> Result<()>;
    
    /// Lit des données depuis un périphérique I²C
    /// 
    /// # Arguments
    /// * `adresse` - Adresse 7-bit du périphérique
    /// * `buffer` - Buffer où stocker les données lues
    fn lire(&mut self, adresse: u8, buffer: &mut [u8]) -> Result<()>;
    
    /// Écrit un registre puis lit la réponse (write-read transaction)
    /// 
    /// Cette opération atomique est courante avec les capteurs I²C.
    /// Elle permet d'écrire l'adresse du registre puis de lire sa valeur
    /// sans relâcher le bus.
    /// 
    /// # Arguments
    /// * `adresse` - Adresse 7-bit du périphérique
    /// * `registre` - Adresse du registre à lire
    /// * `buffer` - Buffer où stocker les données lues
    fn ecrire_lire(&mut self, adresse: u8, registre: &[u8], buffer: &mut [u8]) -> Result<()>;
    
    /// Écrit une valeur dans un registre 8-bit
    /// 
    /// Fonction helper courante pour les capteurs I²C
    fn ecrire_registre_u8(&mut self, adresse: u8, registre: u8, valeur: u8) -> Result<()> {
        self.ecrire(adresse, &[registre, valeur])
    }
    
    /// Lit un registre 8-bit
    fn lire_registre_u8(&mut self, adresse: u8, registre: u8) -> Result<u8> {
        let mut buffer = [0u8; 1];
        self.ecrire_lire(adresse, &[registre], &mut buffer)?;
        Ok(buffer[0])
    }
    
    /// Lit un registre 16-bit (big-endian)
    fn lire_registre_u16_be(&mut self, adresse: u8, registre: u8) -> Result<u16> {
        let mut buffer = [0u8; 2];
        self.ecrire_lire(adresse, &[registre], &mut buffer)?;
        Ok(u16::from_be_bytes(buffer))
    }
    
    /// Lit un registre 16-bit (little-endian)
    fn lire_registre_u16_le(&mut self, adresse: u8, registre: u8) -> Result<u16> {
        let mut buffer = [0u8; 2];
        self.ecrire_lire(adresse, &[registre], &mut buffer)?;
        Ok(u16::from_le_bytes(buffer))
    }

   fn ecrire_registre_atomique(&mut self, adresse: u8, registre: u8, valeur: u8) -> Result<()> {
    // Implémentation par défaut : comportement I2C standard
    self.ecrire(adresse, &[registre, valeur])
}

}

/// Mock I²C pour les tests
/// 
/// Simule un bus I²C en mémoire avec des registres virtuels
//#[cfg(test)]

pub struct I2cMock {
    // Stockage interne : (adresse_device, adresse_registre) -> valeur
    registres: std::collections::HashMap<(u8, u8), u8>,
    #[cfg(test)]
    pub simuler_erreur: bool,
}

//#[cfg(test)]
impl I2cMock {
    pub fn nouveau() -> Self {
        Self {
            registres: std::collections::HashMap::new(),
            #[cfg(test)]
            simuler_erreur: false,   
         }
    }
    
    /// Précharge un registre avec une valeur (pour simuler un capteur)
    pub fn precharger_registre(&mut self, adresse: u8, registre: u8, valeur: u8) {
        self.registres.insert((adresse, registre), valeur);
    }
    
    /// Vérifie qu'un registre a une certaine valeur (pour les tests)
    pub fn verifier_registre(&self, adresse: u8, registre: u8) -> Option<u8> {
        self.registres.get(&(adresse, registre)).copied()
    }
}

//#[cfg(test)]
impl BusI2c for I2cMock {
    fn ecrire(&mut self, adresse: u8, donnees: &[u8]) -> Result<()> {
        #[cfg(test)]
        if self.simuler_erreur {
            return Err(ErreursAirHaum::ErreurI2C("Erreur simulée".into()));
        }
        if donnees.len() >= 2 {
            // Format: [registre, valeur, ...]
            let registre = donnees[0];
            for (i, &valeur) in donnees[1..].iter().enumerate() {
                self.registres.insert((adresse, registre + i as u8), valeur);
            }
        }
        Ok(())
    }
    
    fn lire(&mut self, adresse: u8, buffer: &mut [u8]) -> Result<()> {
        // Pour le mock, on retourne des zéros par défaut
        for byte in buffer.iter_mut() {
            *byte = self.registres.get(&(adresse, 0)).copied().unwrap_or(0);
        }
        Ok(())
    }
    
    fn ecrire_lire(&mut self, adresse: u8, registre: &[u8], buffer: &mut [u8]) -> Result<()> {
        if registre.is_empty() {
            return Err(ErreursAirHaum::ErreurI2C("Registre vide".into()));
        }
        
        let reg_addr = registre[0];

        for (i, byte) in buffer.iter_mut().enumerate() {
           let key = (adresse, reg_addr + i as u8);
           *byte = self.registres.get(&key).copied().ok_or_else(|| {
              ErreursAirHaum::ErreurI2C(
                 format!("Registre 0x{:02X}:0x{:02X} non préchargé dans le mock", 
                    adresse, reg_addr + i as u8)
              )
           })?;
        }

        Ok(())
    }
}

///Cette option permet d'utiliser le bus I2C par référence, ce qui est utile quand plusieurs capteurs partagent le même bus.
impl<T: BusI2c> BusI2c for &mut T {
    fn ecrire(&mut self, adresse: u8, donnees: &[u8]) -> Result<()> {
        (*self).ecrire(adresse, donnees)
    }
    
    fn lire(&mut self, adresse: u8, buffer: &mut [u8]) -> Result<()> {
        (*self).lire(adresse, buffer)
    }
    
    fn ecrire_lire(&mut self, adresse: u8, registre: &[u8], buffer: &mut [u8]) -> Result<()> {
        (*self).ecrire_lire(adresse, registre, buffer)
    }
}



#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_mock_ecriture_lecture() {
        let mut i2c = I2cMock::nouveau();
        
        // Écriture dans un registre
        i2c.ecrire_registre_u8(0x76, 0xF4, 0x27).unwrap();
        
        // Vérification
        assert_eq!(i2c.verifier_registre(0x76, 0xF4), Some(0x27));
    }
    
    #[test]
    fn test_mock_lecture_registre() {
        let mut i2c = I2cMock::nouveau();
        
        // Précharge un registre
        i2c.precharger_registre(0x76, 0xD0, 0x58); // ID du BMP280
        
        // Lecture
        let id = i2c.lire_registre_u8(0x76, 0xD0).unwrap();
        assert_eq!(id, 0x58);
    }
    
    #[test]
    fn test_mock_lecture_u16() {
        let mut i2c = I2cMock::nouveau();
        
        // Précharge deux registres consécutifs
        i2c.precharger_registre(0x76, 0x88, 0x12);
        i2c.precharger_registre(0x76, 0x89, 0x34);
        
        // Lecture 16-bit little-endian
        let valeur = i2c.lire_registre_u16_le(0x76, 0x88).unwrap();
        assert_eq!(valeur, 0x3412);
    }
}
