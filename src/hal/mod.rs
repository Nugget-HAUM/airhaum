// src/hal/mod.rs
//! Hardware Abstraction Layer (HAL)
//!
//! Couche d'abstraction matérielle découplant le code métier des
//! implémentations spécifiques à la plateforme.
//!
//! # Composants
//!
//! - [`i2c::BusI2c`] : trait central pour toutes les communications I²C
//! - [`i2c::I2cMock`] : implémentation en mémoire pour les tests
//! - [`i2c_linux::I2cLinux`] : implémentation Linux (`/dev/i2c-X`) — cible uniquement
//! - [`BusPartage`] : alias de type pour le partage sûr du bus entre tâches
//!
//! # Partage du bus I²C entre tâches
//!
//! Plusieurs capteurs cohabitent sur le même bus physique I²C. Pour éviter
//! l'entrelacement de transactions entre tâches Tokio concurrentes, toute la
//! couche driver utilise un bus partagé plutôt que d'ouvrir un fd par capteur.
//!
//! ```ignore
//! use airhaum::hal::{BusPartage, i2c_linux::I2cLinux};
//! use std::sync::{Arc, Mutex};
//!
//! // Une seule ouverture du bus, partagée entre tous les threads capteurs.
//! let bus: BusPartage<I2cLinux> = Arc::new(Mutex::new(I2cLinux::nouveau(1)?));
//!
//! let bus_bmp  = Arc::clone(&bus);
//! let bus_vl53 = Arc::clone(&bus);
//! let bus_mpu  = Arc::clone(&bus);
//! ```
//!
//! Dans chaque thread, l'accès se fait via :
//! ```ignore
//! // Si un thread a paniqué en tenant le verrou, le mutex est corrompu.
//! // On propage une erreur I²C plutôt que de paniquer à notre tour.
//! let mut bus = bus_bmp.lock()
//!     .map_err(|_| ErreursAirHaum::ErreurI2C("verrou I²C corrompu".into()))?;
//! bus.ecrire_lire(adresse, &[registre], &mut buf)?;
//! ```
//!
//! Le `Mutex` std est bloquant : l'attente suspend le thread OS appelant.
//! C'est correct ici car les threads capteurs sont des `std::thread` dédiés.

pub mod i2c;
pub mod uart;

#[cfg(target_os = "linux")]
pub mod i2c_linux;

#[cfg(target_os = "linux")]
pub mod uart_linux;

pub use i2c::BusI2c;
pub use uart::PortSerie;

#[cfg(target_os = "linux")]
pub use i2c_linux::I2cLinux;

#[cfg(target_os = "linux")]
pub use uart_linux::PortSerieLinux;

/// Bus I²C partagé entre plusieurs threads capteurs.
///
/// Combine [`Arc`](std::sync::Arc) (propriété partagée) et
/// [`std::sync::Mutex`] (exclusion mutuelle) pour garantir
/// qu'un seul thread accède au bus à la fois.
///
/// Le verrou est acquis de manière bloquante (`.lock().unwrap()`), ce qui est
/// correct ici car les threads capteurs sont des `std::thread` dédiés — pas
/// des tâches Tokio. La durée de verrouillage est bornée par le timeout kernel
/// `I2C_TIMEOUT` (~10 ms), garantissant que le mutex est toujours libéré
/// rapidement même en cas de capteur défaillant.
///
/// Paramètre `B` : n'importe quelle implémentation de [`BusI2c`].
/// En production : `BusPartage<I2cLinux>`.
/// En test : `BusPartage<I2cMock>`.
pub type BusPartage<B> = std::sync::Arc<std::sync::Mutex<B>>;
