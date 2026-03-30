//! Fonctions de diagnostic matériel

pub mod diag_bmp280;     // Import du module (fichier) 
pub mod diag_vl53l0x; 
pub mod diag_mpu9250;

//mod lora;
// etc.

pub use diag_bmp280::tester_bmp280;          // Import des fonctions dans le module²
pub use diag_vl53l0x::{
   test_communication, 
   test_initialisation, 
   test_mesure_unique, 
   test_mesures_continues, 
   diagnostic_complet};


//pub use gps::tester_gps;
//pub use lora::tester_lora;
