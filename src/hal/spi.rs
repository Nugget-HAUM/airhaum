// src/hal/spi.rs
//! Abstraction du bus SPI.
//!
//! Miroir de [`super::i2c`] et [`super::uart`] pour le bus SPI. Contrairement
//! à I²C, le format des transactions (adressage registre, bit lecture/écriture,
//! etc.) varie par périphérique — la HAL n'expose donc qu'une primitive de
//! transfert full-duplex brute. Le format registre du SX1276 vit dans
//! `drivers/lora/sx1276.rs`, pas ici.

use crate::types::{Result, ErreursAirHaum};

// ─────────────────────────────────────────────────────────────────────────────
// Trait central
// ─────────────────────────────────────────────────────────────────────────────

/// Abstraction d'un bus SPI en mode full-duplex.
///
/// Deux implémentations concrètes :
/// - `SpiLinux` (à venir) : périphérique physique Linux (`/dev/spidevX.Y`)
/// - [`SpiMock`] : buffer en mémoire pour les tests
pub trait BusSpi: Send {
    /// Transaction full-duplex : émet `ecrire` sur MOSI en recevant
    /// simultanément la même quantité d'octets sur MISO dans `lire`.
    ///
    /// `lire` doit avoir exactement la même longueur que `ecrire`.
    fn transferer(&mut self, ecrire: &[u8], lire: &mut [u8]) -> Result<()>;
}

// ─────────────────────────────────────────────────────────────────────────────
// Mock pour les tests
// ─────────────────────────────────────────────────────────────────────────────

/// Bus SPI simulé en mémoire.
///
/// Permet de précharger les octets attendus en réponse (MISO) et de vérifier
/// les octets émis (MOSI) dans les tests unitaires, sans matériel physique.
///
/// # Exemple
/// ```
/// use airhaum::hal::spi::{BusSpi, SpiMock};
///
/// let mut spi = SpiMock::nouveau();
/// spi.precharger_reponse(&[0x12]); // valeur de registre simulée
/// let mut lire = [0u8; 2];
/// spi.transferer(&[0x42, 0x00], &mut lire).unwrap();
/// assert_eq!(lire[1], 0x12);
/// ```
pub struct SpiMock {
    /// Octets à renvoyer sur MISO, dans l'ordre des transferts.
    reponses: std::collections::VecDeque<u8>,
    /// Octets reçus sur MOSI (capturés pour vérification dans les tests).
    ecrits: Vec<u8>,
}

impl SpiMock {
    pub fn nouveau() -> Self {
        Self {
            reponses: std::collections::VecDeque::new(),
            ecrits:   Vec::new(),
        }
    }

    /// Précharge des octets qui seront renvoyés sur MISO par les prochains
    /// appels à `transferer()`.
    pub fn precharger_reponse(&mut self, data: &[u8]) {
        self.reponses.extend(data.iter().copied());
    }

    /// Retourne les octets émis sur MOSI depuis la création du mock (pour
    /// vérification dans les tests).
    pub fn octets_ecrits(&self) -> &[u8] {
        &self.ecrits
    }
}

impl BusSpi for SpiMock {
    fn transferer(&mut self, ecrire: &[u8], lire: &mut [u8]) -> Result<()> {
        if lire.len() != ecrire.len() {
            return Err(ErreursAirHaum::ErreurSpi(
                "transferer : tailles écrire/lire différentes".into(),
            ));
        }
        self.ecrits.extend_from_slice(ecrire);
        for octet in lire.iter_mut() {
            *octet = self.reponses.pop_front().unwrap_or(0);
        }
        Ok(())
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Tests
// ─────────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_mock_ecrit_capture() {
        let mut spi = SpiMock::nouveau();
        let mut lire = [0u8; 3];
        spi.transferer(&[0x01, 0x02, 0x03], &mut lire).unwrap();
        assert_eq!(spi.octets_ecrits(), &[0x01, 0x02, 0x03]);
    }

    #[test]
    fn test_mock_reponse_prechargee() {
        let mut spi = SpiMock::nouveau();
        spi.precharger_reponse(&[0xAA, 0xBB]);
        let mut lire = [0u8; 2];
        spi.transferer(&[0x00, 0x00], &mut lire).unwrap();
        assert_eq!(lire, [0xAA, 0xBB]);
    }

    #[test]
    fn test_mock_reponse_manquante_vaut_zero() {
        let mut spi = SpiMock::nouveau();
        let mut lire = [0xFFu8; 2];
        spi.transferer(&[0x00, 0x00], &mut lire).unwrap();
        assert_eq!(lire, [0x00, 0x00]);
    }

    #[test]
    fn test_tailles_differentes_erreur() {
        let mut spi = SpiMock::nouveau();
        let mut lire = [0u8; 1];
        assert!(spi.transferer(&[0x00, 0x00], &mut lire).is_err());
    }
}
