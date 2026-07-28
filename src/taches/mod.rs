// src/taches/mod.rs
//! Description à ajouter
//!
//! 



pub mod taches_capteurs;
pub mod taches_controle;
pub mod taches_estimation;
pub mod taches_gps;
pub mod taches_servo;
pub mod taches_surete;

pub use taches_capteurs::{lancer_capteurs, HandlesCapteurs, MesureBaro, MesureTelem, MesureImu, SanteCapteurs};
pub use taches_controle::{lancer_controle, HandlesControle, PhaseControle};
pub use taches_estimation::{lancer_estimation, HandlesEstimation};
pub use taches_gps::{lancer_gps, HandlesGps, MesureGps, SanteGps, ResultatAssistance};
pub use taches_servo::{lancer_servo, HandlesServo};
pub use taches_surete::{lancer_surete, HandlesSurete};
pub use crate::capteurs::AltitudeFusionnee;
