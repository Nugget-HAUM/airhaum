// src/taches/mod.rs
//! Description à ajouter
//!
//! 



pub mod taches_capteurs;
pub mod taches_estimation;
pub mod taches_gps;
pub mod taches_servo;

pub use taches_capteurs::{lancer_capteurs, HandlesCapteurs, MesureBaro, MesureTelem, MesureImu, SanteCapteurs};
pub use taches_estimation::{lancer_estimation, HandlesEstimation};
pub use taches_gps::{lancer_gps, HandlesGps, MesureGps, SanteGps};
pub use taches_servo::{lancer_servo, HandlesServo};
pub use crate::capteurs::AltitudeFusionnee;
