// src/hal/i2c_linux.rs
// Implémentation du trait BusI2c pour Linux en utilisant i2cdev

use crate::hal::i2c::BusI2c;
use crate::types::{Result, ErreursAirHaum};
use i2cdev::core::I2CDevice;
use i2cdev::linux::LinuxI2CDevice;
  

/// Backend I²C pour Linux (/dev/i2c-X)
pub struct I2cLinux {
    device: LinuxI2CDevice,
}

impl I2cLinux {
    /// Crée une nouvelle instance du bus I²C Linux
    /// 
    /// # Arguments
    /// * `bus` - Numéro du bus I²C (ex: 0 pour /dev/i2c-0, 1 pour /dev/i2c-1)
    /// 
    /// # Exemple
    /// ```no_run
    /// use airhaum::hal::i2c_linux::I2cLinux;
    /// 
    /// let i2c = I2cLinux::nouveau(0).unwrap(); // Ouvre /dev/i2c-0
    /// ```
    pub fn nouveau(bus: u8) -> Result<Self> {
        let chemin = format!("/dev/i2c-{}", bus);
        let device = LinuxI2CDevice::new(&chemin, 0x00)
            .map_err(|e| ErreursAirHaum::ErreurI2C(
                format!("Impossible d'ouvrir {}: {}", chemin, e)
            ))?;
        
        Ok(Self { device })
    }
    
    /// Configure l'adresse du périphérique esclave
    fn definir_adresse_esclave(&mut self, adresse: u8) -> Result<()> {
        self.device.set_slave_address(adresse as u16)
            .map_err(|e| ErreursAirHaum::ErreurI2C(
                format!("Impossible de définir l'adresse esclave 0x{:02X}: {}", adresse, e)
            ))
    }
}

impl BusI2c for I2cLinux {
    fn ecrire(&mut self, adresse: u8, donnees: &[u8]) -> Result<()> {
        self.definir_adresse_esclave(adresse)?;
        self.device.write(donnees)
            .map_err(|e| ErreursAirHaum::ErreurI2C(
                format!("Échec écriture I²C vers 0x{:02X}: {}", adresse, e)
            ))
    }
    
    fn lire(&mut self, adresse: u8, buffer: &mut [u8]) -> Result<()> {
        self.definir_adresse_esclave(adresse)?;
        self.device.read(buffer)
            .map_err(|e| ErreursAirHaum::ErreurI2C(
                format!("Échec lecture I²C depuis 0x{:02X}: {}", adresse, e)
            ))
    }
    
    fn ecrire_lire(&mut self, adresse: u8, registre: &[u8], buffer: &mut [u8]) -> Result<()> {
        self.definir_adresse_esclave(adresse)?;
        
        // Effectue une transaction write-read atomique
        self.device.write(registre)
            .map_err(|e| ErreursAirHaum::ErreurI2C(
                format!("Échec écriture registre I²C: {}", e)
            ))?;
        
        self.device.read(buffer)
            .map_err(|e| ErreursAirHaum::ErreurI2C(
                format!("Échec lecture réponse I²C: {}", e)
            ))
    }

   fn ecrire_registre_atomique(&mut self, adresse: u8, registre: u8, valeur: u8) -> Result<()> {
       self.definir_adresse_esclave(adresse)?;
      self.device.smbus_write_byte_data(registre, valeur)
        .map_err(|e| ErreursAirHaum::ErreurI2C(
            format!("Échec écriture SMBus vers 0x{:02X}: {}", adresse, e)
        ))
   }

}

// Note: Pour une meilleure performance, on pourrait utiliser i2c_smbus_* 
// ou i2c_rdwr pour des transactions vraiment atomiques, mais i2cdev 
// suffit pour commencer.
