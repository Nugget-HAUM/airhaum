// src/drivers/barometre/mod.rs
//! Drivers pour capteurs barométriques
//!
//! Ce module contient les drivers pour les capteurs de pression atmosphérique
//! utilisés pour mesurer l'altitude.

pub mod bmp280;
pub mod calibration;

// Réexportations pour faciliter l'utilisation
pub use bmp280::{Bmp280, ADRESSE_BMP280};
pub use calibration::CalibrationBarometre;
