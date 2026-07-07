// src/hal/uart.rs
//! Abstraction du port série UART.
//!
//! Miroir de [`super::i2c`] pour le bus UART. Le trait [`PortSerie`] découple
//! les drivers (GPS, LoRa à venir) des détails système (`/dev/ttyS*`, ioctl…),
//! ce qui permet de les tester sur n'importe quelle machine avec [`PortSerieMock`].

use std::collections::VecDeque;
use std::io;

// ─────────────────────────────────────────────────────────────────────────────
// Trait central
// ─────────────────────────────────────────────────────────────────────────────

/// Abstraction d'un port série bidirectionnel.
///
/// Deux implémentations concrètes :
/// - [`PortSerieLinux`](super::uart_linux::PortSerieLinux) : port physique Linux
/// - [`PortSerieMock`] : buffer en mémoire pour les tests
pub trait PortSerie: Send {
    /// Lit des octets disponibles dans `buf`. Non-bloquant si le port a été
    /// ouvert avec `timeout = 0` — retourne `TimedOut` s'il n'y a rien.
    fn lire(&mut self, buf: &mut [u8]) -> io::Result<usize>;

    /// Écrit `buf` sur le port. Utilisé pour les commandes de configuration UBX.
    fn ecrire(&mut self, buf: &[u8]) -> io::Result<usize>;

    /// Ferme et réouvre le port à la nouvelle vitesse en bauds.
    ///
    /// Utilisé pendant la séquence d'initialisation GPS :
    /// on démarre à 9 600 bauds (config usine NEO-M8N) pour détecter
    /// le protocole, puis on repasse à 115 200 après configuration.
    fn reconfigurer_baudrate(&mut self, baudrate: u32) -> io::Result<()>;
}

// ─────────────────────────────────────────────────────────────────────────────
// Mock pour les tests
// ─────────────────────────────────────────────────────────────────────────────

/// Port série simulé en mémoire.
///
/// Permet d'injecter des trames UBX byte-par-byte dans les tests unitaires
/// sans aucun matériel physique.
///
/// # Exemple
/// ```ignore
/// let mut mock = PortSerieMock::nouveau();
/// mock.injecter(&[0xB5, 0x62, ...]); // trame UBX
/// let mut driver = DriverGps::nouveau(mock);
/// driver.mettre_a_jour();
/// ```
pub struct PortSerieMock {
    /// Octets disponibles à la prochaine lecture.
    donnees: VecDeque<u8>,
    /// Octets écrits sur le port (capturés pour vérification dans les tests).
    ecrits: Vec<u8>,
}

impl PortSerieMock {
    pub fn nouveau() -> Self {
        Self {
            donnees: VecDeque::new(),
            ecrits:  Vec::new(),
        }
    }

    /// Injecte des octets qui seront lus par le prochain appel à `lire()`.
    pub fn injecter(&mut self, data: &[u8]) {
        self.donnees.extend(data.iter().copied());
    }

    /// Retourne les octets écrits via `ecrire()` (pour vérification dans les tests).
    pub fn octets_ecrits(&self) -> &[u8] {
        &self.ecrits
    }
}

impl PortSerie for PortSerieMock {
    fn lire(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        if self.donnees.is_empty() {
            // Simule un timeout (comportement non-bloquant)
            return Err(io::Error::new(io::ErrorKind::TimedOut, "mock: pas de données"));
        }
        let n = buf.len().min(self.donnees.len());
        for b in buf[..n].iter_mut() {
            *b = self.donnees.pop_front().unwrap();
        }
        Ok(n)
    }

    fn ecrire(&mut self, buf: &[u8]) -> io::Result<usize> {
        self.ecrits.extend_from_slice(buf);
        Ok(buf.len())
    }

    fn reconfigurer_baudrate(&mut self, _baudrate: u32) -> io::Result<()> {
        Ok(()) // pas de port physique — no-op
    }
}
