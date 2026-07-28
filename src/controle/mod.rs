// src/controle/mod.rs
//! Couche 4 — Contrôle
//!
//! Transforme les consignes de vol produites par la couche mission (couche 5)
//! en commandes envoyées aux servos. Consomme l'état estimé (couche 3,
//! `EtatNavigation`) et ne dépend jamais de la couche mission — voir
//! `doc/controle.md` pour l'architecture complète et l'ordre des briques.

pub mod controleur_direction_sol;
pub mod controleur_lacet;
pub mod controleur_roulis;
pub mod controleur_tangage;
pub mod mixeur;
pub mod pid;

pub use controleur_direction_sol::{ConsigneDirectionSol, ControleurDirectionSol};
pub use controleur_lacet::{ConsigneLacet, ControleurLacet};
pub use controleur_roulis::{ConsigneRoulis, ControleurRoulis};
pub use controleur_tangage::{ConsigneTangage, ControleurTangage};
pub use mixeur::{mixer, CommandesNormalisees, ConsigneGaz};
pub use pid::Pid;
