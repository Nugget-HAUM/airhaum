//! Fonctions de diagnostic matériel

pub mod diag_bmp280;
pub mod diag_vl53l0x;
pub mod diag_mpu9250;
pub mod diag_taches_capteurs;
pub mod diag_gps;



//mod lora;
// etc.

pub use diag_bmp280::tester_bmp280;          // Import des fonctions dans le module²
pub use diag_vl53l0x::{
   test_communication,
   test_initialisation,
   test_mesure_unique,
   test_mesures_continues,
   diagnostic_complet};

pub use diag_gps::{
    test_communication as test_communication_gps,
    attendre_fix,
    mesures_continues as mesures_continues_gps,
    diagnostic_complet as diagnostic_complet_gps,
};


//pub use gps::tester_gps;
//pub use lora::tester_lora;
