// src/estimation/mod.rs
//! Couche 3 — Estimation d'état
//!
//! Transforme les mesures prétraitées (couche 2) en état estimé de l'appareil :
//! attitude (roulis/tangage/lacet), altitude, vitesse.
//!
//! # Progression
//!
//! - [`attitude`] : filtre complémentaire (première implémentation, sans GPS)
//! - `ekf`        : filtre de Kalman étendu (remplacera le filtre complémentaire
//!                  en implémentant le même trait — pas de refactoring amont)

pub mod attitude;
pub mod ekf_attitude;
pub mod ekf_navigation;

pub use attitude::{Attitude, FiltreComplementaire};
pub use ekf_attitude::EkfAttitude;
pub use ekf_navigation::{EkfNavigation, EtatNavigation};
