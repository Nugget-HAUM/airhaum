// src/hal/spi_linux.rs
//! Implémentation Linux de [`BusSpi`] via la crate `rppal`.
//!
//! Cible uniquement (`#[cfg(target_os = "linux")]`). Sur les autres OS
//! (développement, CI), utiliser [`super::spi::SpiMock`].
//!
//! Paramètres et câblage repris tels quels du prototype `OrangePi/SX1276`
//! (hors dépôt), déjà validé sur le matériel réel (Orange Pi Zero + SX1276) :
//! voir son fichier `Notes` pour le détail des broches.

use rppal::spi::{Bus, Mode, SlaveSelect, Spi};

use super::spi::BusSpi;
use crate::types::{ErreursAirHaum, Result};

// ─────────────────────────────────────────────────────────────────────────────
// Constantes par défaut — correspondent au câblage SX1276 de l'Orange Pi Zero
// ─────────────────────────────────────────────────────────────────────────────

/// Bus SPI du module LoRa SX1276 (SPI1 — broches GPIO dédiées, distinctes du SPI0 usuel).
pub const BUS_LORA_DEFAUT: Bus = Bus::Spi1;
/// Chip select du SX1276 (Ss0).
pub const SELECT_LORA_DEFAUT: SlaveSelect = SlaveSelect::Ss0;
/// Vitesse d'horloge SPI, en Hz.
pub const VITESSE_LORA_DEFAUT_HZ: u32 = 1_000_000;
/// Mode SPI du SX1276 (CPOL=0, CPHA=0).
pub const MODE_LORA_DEFAUT: Mode = Mode::Mode0;

// ─────────────────────────────────────────────────────────────────────────────
// Structure
// ─────────────────────────────────────────────────────────────────────────────

/// Bus SPI physique Linux, pour le module LoRa SX1276.
pub struct SpiLinux {
    spi: Spi,
}

impl SpiLinux {
    /// Ouvre le bus SPI avec les paramètres par défaut du SX1276.
    pub fn nouveau() -> Result<Self> {
        let spi = Spi::new(BUS_LORA_DEFAUT, SELECT_LORA_DEFAUT, VITESSE_LORA_DEFAUT_HZ, MODE_LORA_DEFAUT)
            .map_err(|e| ErreursAirHaum::ErreurSpi(format!("Impossible d'ouvrir le bus SPI : {}", e)))?;
        Ok(Self { spi })
    }
}

impl BusSpi for SpiLinux {
    fn transferer(&mut self, ecrire: &[u8], lire: &mut [u8]) -> Result<()> {
        self.spi.transfer(lire, ecrire)
            .map_err(|e| ErreursAirHaum::ErreurSpi(format!("Échec transfert SPI : {}", e)))?;
        Ok(())
    }
}
