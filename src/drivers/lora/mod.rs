// src/drivers/lora/mod.rs
//! Driver du transceiver LoRa SX1276 côté Pi (liaison SPI).
//!
//! Distinct du dossier `lora/` à la racine du projet, qui contient le firmware
//! du pont Wemos et le logiciel de la station sol (voir `doc/lora.md`).

pub mod sx1276;

pub use sx1276::{Sx1276, TAILLE_MAX_PAYLOAD};
