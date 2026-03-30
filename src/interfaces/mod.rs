// src/interfaces/mod.rs
//! Traits définissant les interfaces des composants
//!
//! Ces traits permettent l'abstraction et la testabilité du système.



   #![allow(dead_code)]
   #![allow(unused_imports)]

pub mod barometre;
pub mod telemetre;
pub mod imu;

pub use barometre::Barometre;
pub use telemetre::Telemetre;
pub use imu::CentraleInertielle;



