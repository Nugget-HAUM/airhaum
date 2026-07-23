// src/communications/mod.rs
//! Couche 6 : communication LoRa (télémétrie descendante, commandes montantes).
//!
//! Voir doc/lora.md pour les décisions d'architecture (accès au canal,
//! fiabilité, format de trame).

pub mod protocole;

pub use protocole::{TypeMessage, Trame, CompteurSequence, encoder, decoder};
