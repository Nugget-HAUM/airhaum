// src/types/mod.rs
// Module regroupant tous les types fondamentaux du projet
//! Types fondamentaux du projet
//!
//! Ce module regroupe tous les types de base utilisés partout ailleurs :
//! géométrie, messages capteurs, erreurs, constantes physiques.




   #![allow(dead_code)]
   #![allow(unused_imports)]

pub mod geometrie;
pub mod messages;
pub mod erreurs;
pub mod constantes;
pub mod etat_capteur;

// Réexport des types les plus utilisés pour faciliter l'import
pub use geometrie::{Vector3, Quaternion, Angle};
pub use messages::{
    DonneesCapteur, 
    DonneesBarometre,     
    DonneesGps,           
    DonneesImu,           
    DonneesTelemetre,     
    MessageSysteme, 
    Horodatage,
    Temperature,          
    Pression,             
    Distance,             
};
pub use erreurs::{ErreursAirHaum, Result};
pub use constantes::*;  // Exporte toutes les constantes
pub use etat_capteur::EtatCapteur;



