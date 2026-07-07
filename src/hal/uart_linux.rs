// src/hal/uart_linux.rs
//! Implémentation Linux du port série UART via la crate `serialport`.
//!
//! Cible uniquement (`#[cfg(target_os = "linux")]`).
//! Sur les autres OS (développement, CI), utiliser [`super::uart::PortSerieMock`].

use std::time::Duration;
use serialport::{DataBits, FlowControl, Parity, StopBits};

use super::uart::PortSerie;
use crate::types::{ErreursAirHaum, Result};

// ─────────────────────────────────────────────────────────────────────────────
// Constantes par défaut — correspondent à la configuration du NEO-M8N
// ─────────────────────────────────────────────────────────────────────────────

/// Port série du GPS (NEO-M8N) sur l'Orange Pi.
/// Temporairement désactivé (/dev/ttyS3 inexistant) : ttyS2 est réaffecté au Nano.
pub const PORT_GPS_DEFAUT: &str = "/dev/ttyS3";
/// Débit en bauds du GPS.
pub const BAUDRATE_GPS: u32 = 115_200;

/// Port série de la liaison Pi ↔ Arduino Nano (UART2, broches GPIO — UART1 RX endommagé).
pub const PORT_ARDUINO_DEFAUT: &str = "/dev/ttyS2";
/// Débit en bauds de la liaison logicielle Arduino.
pub const BAUDRATE_ARDUINO: u32 = 57_600;

// ─────────────────────────────────────────────────────────────────────────────
// Structure
// ─────────────────────────────────────────────────────────────────────────────

/// Port série physique Linux.
///
/// Ouvert en lecture/écriture non-bloquante (`timeout = 0`) : `lire()` retourne
/// immédiatement `TimedOut` s'il n'y a pas d'octet disponible. C'est le comportement
/// attendu par le thread GPS qui tourne en boucle serrée.
pub struct PortSerieLinux {
    port: Box<dyn serialport::SerialPort>,
    chemin: String,
}

impl PortSerieLinux {
    /// Ouvre le port série avec les paramètres du NEO-M8N.
    ///
    /// # Arguments
    ///
    /// * `chemin` — chemin du périphérique, ex. `"/dev/ttyS2"`
    /// * `baudrate` — débit en bauds, ex. `115_200`
    ///
    /// # Erreurs
    ///
    /// Retourne [`ErreursAirHaum::ErreurIO`] si le port n'existe pas ou si les
    /// permissions sont insuffisantes (l'utilisateur doit être dans le groupe `dialout`).
    pub fn nouveau(chemin: &str, baudrate: u32) -> Result<Self> {
        let port = serialport::new(chemin, baudrate)
            .data_bits(DataBits::Eight)
            .flow_control(FlowControl::None)
            .parity(Parity::None)
            .stop_bits(StopBits::One)
            .timeout(Duration::from_millis(0))   // lecture non-bloquante
            .open()
            .map_err(|e| ErreursAirHaum::ErreurIO(
                format!("Ouverture port série {}: {}", chemin, e)
            ))?;

        Ok(Self { port, chemin: chemin.to_string() })
    }
}

impl PortSerie for PortSerieLinux {
    fn lire(&mut self, buf: &mut [u8]) -> std::io::Result<usize> {
        self.port.read(buf)
    }

    fn ecrire(&mut self, buf: &[u8]) -> std::io::Result<usize> {
        use std::io::Write;
        self.port.write(buf)
    }

    fn reconfigurer_baudrate(&mut self, baudrate: u32) -> std::io::Result<()> {
        // Modification en place via ioctl(TCSETS) — pas besoin de fermer/rouvrir.
        self.port.set_baud_rate(baudrate)
            .map_err(|e| std::io::Error::new(
                std::io::ErrorKind::Other,
                format!("Reconfiguration {} bauds sur {}: {}", baudrate, self.chemin, e)
            ))
    }
}
