// src/diagnostiques/diag_lora.rs
//! Fonctions de diagnostic du module LoRa SX1276.

use crate::hal::spi::BusSpi;
use crate::drivers::lora::Sx1276;
use crate::types::Result;

// ─────────────────────────────────────────────────────────────────────────────
// Ouverture du bus (helpers internes)
// ─────────────────────────────────────────────────────────────────────────────

#[cfg(target_os = "linux")]
fn ouvrir_spi() -> Result<impl BusSpi> {
    use crate::hal::spi_linux::SpiLinux;
    SpiLinux::nouveau()
}

#[cfg(not(target_os = "linux"))]
fn ouvrir_spi() -> Result<impl BusSpi> {
    use crate::hal::spi::SpiMock;
    Ok(SpiMock::nouveau())
}

/// Fréquence porteuse par défaut (bande EU868).
const FREQUENCE_DEFAUT_HZ: u32 = 868_000_000;

// ─────────────────────────────────────────────────────────────────────────────
// Test de communication
// ─────────────────────────────────────────────────────────────────────────────

/// Vérifie que le bus SPI s'ouvre et que le SX1276 répond (registre VERSION).
pub fn test_communication() -> Result<()> {
    let spi = ouvrir_spi()?;
    let mut radio = Sx1276::nouveau(spi);
    radio.initialiser(FREQUENCE_DEFAUT_HZ)?;
    println!("  LoRa : SPI ouvert, SX1276 détecté et initialisé ✓");
    Ok(())
}

// ─────────────────────────────────────────────────────────────────────────────
// Envoi d'un message de test
// ─────────────────────────────────────────────────────────────────────────────

/// Émet `message` — succès signifie que le SX1276 a signalé TX_DONE, pas que
/// le message a été reçu. À comparer manuellement avec la réception côté
/// Wemos/PC pour valider la liaison de bout en bout.
pub fn envoyer_message_test(message: &str) -> Result<()> {
    let spi = ouvrir_spi()?;
    let mut radio = Sx1276::nouveau(spi);
    radio.initialiser(FREQUENCE_DEFAUT_HZ)?;

    println!("  LoRa : envoi de \"{}\" ({} octets)…", message, message.len());
    radio.envoyer(message.as_bytes())?;
    println!("  LoRa : TX_DONE reçu ✓ — vérifier la réception côté Wemos/PC");
    Ok(())
}
