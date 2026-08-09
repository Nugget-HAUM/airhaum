// src/mission/mod.rs
//! Couche mission — MAÉ vol, comportements de décollage, navigation et atterrissage.
//!
//! Dépend des couches estimation (3) et contrôle (4) — `plan_vol.rs`
//! produit directement des types de `controle::` (`ConsigneDirectionSol`,
//! `ConsigneGaz`), voir `doc/plan_de_vol.md`.

pub mod armement;
pub mod etat_machine;
pub mod plan_vol;
