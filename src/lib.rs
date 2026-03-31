//! AirHaum II – Cœur du système
//!
//! Ce crate contient l'ensemble de la logique métier du pilote automatique :
//! types fondamentaux, interfaces, HAL, drivers, estimation, contrôle, mission.

//#![deny(missing_docs)]   // Exiger la présence de documentation
//#![warn(missing_docs)]     // Alerte au lieu de deny pour la phase développement


pub mod types;       // Types fondamentaux (géométrie, messages, erreurs)
pub mod interfaces;  // Traits définissant les interfaces
pub mod hal;         // Hardware Abstraction Layer
pub mod drivers;     // Drivers pour capteurs et périphériques
pub mod diagnostiques;
pub mod systeme;
pub mod taches;

// Les modules suivants pourront être ajoutés progressivement
// pub mod capteurs;
// pub mod estimation;
// pub mod controle;
// pub mod mission;
// pub mod communication;

// Ré-export des types d'erreur pour simplifier l'usage
pub use types::{ErreursAirHaum, Result};
pub use diagnostiques::tester_bmp280; 



/// Version du crate (informationnelle)
pub const VERSION: &str = "AirHaum II – développement 0.023";

/// Point d'entrée logique du système de vol (à implémenter)
pub fn demarrer_vol_autonome() {
    // À terme :
    // - initialisation matérielle
    // - démarrage des tâches
    // - supervision
}

/// Point d'entrée logique pour les outils de test (à implémenter)
pub fn demarrer_tests() {
    // Console de diagnostic matériel
}
/*
/// Teste le baromètre BMP280
pub fn tester_bmp280() -> Result<()> {
    // initialisation
    // lecture
    // vérification cohérence
    Ok(())
}
*/
/// Teste le GPS
pub fn tester_gps() -> Result<()> {
    Ok(())
}
