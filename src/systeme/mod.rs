// src/systeme/mod.rs
//! Services système transversaux
//!
//! Ce module regroupe les services qui sont utilisés par plusieurs
//! composants du système :
//! - Gestion de la calibration persistante
//! - Configuration système (à venir)
//! - Journalisation / boîte noire (à venir)
//! - Watchdog (à venir)

pub mod calibration;

// Réexportation pour faciliter l'utilisation
pub use calibration::{
    CalibrationPersistante,
    GestionnaireCalibration,
    initialiser_gestionnaire,
    gestionnaire,
};
