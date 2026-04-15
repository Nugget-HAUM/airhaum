// src/hal/i2c_linux.rs
//! Implémentation de [`BusI2c`] pour Linux via le driver `i2cdev`.
//!
//! # Atomicité des transactions
//!
//! La méthode [`I2cLinux::ecrire_lire`] utilise l'ioctl `I2C_RDWR` via
//! `LinuxI2CDevice::write_read`, qui envoie les deux messages (write + read)
//! en une seule requête noyau avec un **repeated START** entre les deux.
//! Cela garantit qu'aucune autre transaction ne peut s'intercaler entre
//! l'écriture du registre et la lecture de la réponse.
//!
//! # Partage concurrent
//!
//! `I2cLinux` implémente `Send` mais **pas `Sync`** : il ne doit pas être
//! accédé simultanément depuis plusieurs threads. Le partage concurrent entre
//! les tâches Tokio est géré par [`crate::hal::BusPartage`]
//! (`Arc<tokio::sync::Mutex<I2cLinux>>`), qui sérialise les accès.
//!
//! # Exemple
//! ```no_run
//! use airhaum::hal::i2c_linux::I2cLinux;
//!
//! let i2c = I2cLinux::nouveau(1).unwrap(); // Ouvre /dev/i2c-1
//! ```

use crate::hal::i2c::BusI2c;
use crate::types::{Result, ErreursAirHaum};
use i2cdev::core::{I2CDevice, I2CTransfer, I2CMessage};
use i2cdev::linux::{LinuxI2CDevice, LinuxI2CMessage};
use std::os::unix::io::AsRawFd;

/// Timeout maximal d'une transaction I²C, en unités de 10 ms (valeur noyau Linux).
///
/// Valeur 1 = 10 ms. Borne la durée d'une transaction même en cas de capteur
/// défaillant ou de bus bloqué, garantissant que le mutex partagé est toujours
/// libéré rapidement.
///
/// Choix : une transaction I²C normale (ex. 14 octets MPU9250 à 400 kHz) prend
/// < 1 ms. 10 ms offre une marge ×10 sans dépasser la période IMU (5 ms à 200 Hz
/// correspond à 2 cycles manqués au pire, acceptable pour détection d'anomalie).
const I2C_TIMEOUT_10MS: libc::c_ulong = 1;

/// Numéro de l'ioctl I2C_TIMEOUT (linux/i2c-dev.h).
const I2C_TIMEOUT: libc::c_ulong = 0x0702;

/// Backend I²C pour Linux (`/dev/i2c-X`).
///
/// Encapsule un [`LinuxI2CDevice`] et l'adapte au trait [`BusI2c`].
/// Une instance représente une ouverture du fichier device ; elle ne doit
/// être accédée que depuis un seul thread à la fois.
pub struct I2cLinux {
    device: LinuxI2CDevice,
}

// SAFETY : LinuxI2CDevice contient un file descriptor Unix, qui est
// intrinsèquement lié à un processus et non à un thread. Envoyer la valeur
// vers un autre thread (Send) est sûr tant qu'un seul thread l'utilise à
// la fois — ce que garantit le Mutex dans BusPartage.
// LinuxI2CDevice n'est pas Sync et I2cLinux ne le sera pas non plus.
unsafe impl Send for I2cLinux {}

impl I2cLinux {
    /// Ouvre le bus I²C numéro `bus` (`/dev/i2c-{bus}`).
    ///
    /// # Arguments
    /// * `bus` - Numéro du bus (0 pour `/dev/i2c-0`, 1 pour `/dev/i2c-1`, …)
    ///
    /// # Erreurs
    /// Retourne [`ErreursAirHaum::ErreurI2C`] si le fichier n'existe pas
    /// ou si les permissions sont insuffisantes (groupe `i2c` requis).
    pub fn nouveau(bus: u8) -> Result<Self> {
        let chemin = format!("/dev/i2c-{}", bus);
        // Adresse esclave initiale 0x00 — elle sera redéfinie avant chaque opération.
        let device = LinuxI2CDevice::new(&chemin, 0x00).map_err(|e| {
            ErreursAirHaum::ErreurI2C(format!("Impossible d'ouvrir {} : {}", chemin, e))
        })?;

        // Configure le timeout des transactions I²C au niveau du noyau.
        // Si le hardware ne répond pas dans ce délai, l'ioctl retourne une erreur
        // et le mutex partagé est libéré — les autres capteurs ne sont pas bloqués.
        // SAFETY : fd valide (vient d'être ouvert), constantes conformes à linux/i2c-dev.h.
        let ret = unsafe { libc::ioctl(device.as_raw_fd(), I2C_TIMEOUT, I2C_TIMEOUT_10MS) };
        if ret < 0 {
            return Err(ErreursAirHaum::ErreurI2C(format!(
                "Impossible de configurer I2C_TIMEOUT sur {} : errno {}",
                chemin,
                unsafe { *libc::__errno_location() }
            )));
        }

        Ok(Self { device })
    }

    /// Positionne l'adresse esclave courante du file descriptor.
    ///
    /// Appelé avant chaque opération ; requis car le même fd peut être utilisé
    /// pour plusieurs périphériques (plusieurs capteurs sur le même bus).
    fn definir_adresse_esclave(&mut self, adresse: u8) -> Result<()> {
        self.device
            .set_slave_address(adresse as u16)
            .map_err(|e| {
                ErreursAirHaum::ErreurI2C(format!(
                    "Impossible de définir l'adresse esclave 0x{:02X} : {}",
                    adresse, e
                ))
            })
    }
}

impl BusI2c for I2cLinux {
    fn ecrire(&mut self, adresse: u8, donnees: &[u8]) -> Result<()> {
        self.definir_adresse_esclave(adresse)?;
        self.device.write(donnees).map_err(|e| {
            ErreursAirHaum::ErreurI2C(format!(
                "Échec écriture I²C vers 0x{:02X} : {}",
                adresse, e
            ))
        })
    }

    fn lire(&mut self, adresse: u8, buffer: &mut [u8]) -> Result<()> {
        self.definir_adresse_esclave(adresse)?;
        self.device.read(buffer).map_err(|e| {
            ErreursAirHaum::ErreurI2C(format!(
                "Échec lecture I²C depuis 0x{:02X} : {}",
                adresse, e
            ))
        })
    }

    /// Transaction write-then-read atomique via ioctl `I2C_RDWR`.
    ///
    /// Les deux messages (écriture du registre + lecture de la réponse) sont
    /// envoyés en une seule requête noyau avec un **repeated START**.
    /// Aucune autre transaction ne peut s'intercaler entre les deux.
    fn ecrire_lire(&mut self, adresse: u8, registre: &[u8], buffer: &mut [u8]) -> Result<()> {
        self.definir_adresse_esclave(adresse)?;

        let reg = registre.to_vec();
        let mut msgs = [
            LinuxI2CMessage::write(&reg),
            LinuxI2CMessage::read(buffer),
        ];

        self.device.transfer(&mut msgs)
            .map(|_| ())
            .map_err(|e| {
                ErreursAirHaum::ErreurI2C(format!(
                    "Échec write-read I²C vers 0x{:02X} (registre 0x{:02X}) : {}",
                    adresse,
                    registre.first().copied().unwrap_or(0),
                    e
                ))
            })
    }
}
