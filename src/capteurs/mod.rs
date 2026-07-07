// src/capteurs/mod.rs
//! Couche 2 — Prétraitement et fusion bas niveau des capteurs
//!
//! Cette couche reçoit les mesures **déjà en unités SI et calibrées** produites
//! par les drivers (couche 1), et les enrichit pour les rendre directement
//! consommables par les estimateurs (couche 3) :
//!
//! - Calcul du Δt inter-mesures (indispensable pour l'intégration gyroscope)
//! - Fusion multi-capteurs pour l'altitude (baro + télémètre)
//!
//! # Ce que cette couche ne fait PAS
//!
//! - Conversion d'unités : faite par les drivers
//! - Application de calibration : faite par les drivers
//! - Estimation d'état (attitude, position) : couche 3 (`estimation/`)

pub mod traitement_imu;
pub mod fusion_altitude;

pub use fusion_altitude::AltitudeFusionnee;
