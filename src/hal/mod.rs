// src/hal/mod.rs
//! Hardware Abstraction Layer (HAL)
//!
//! Couche d'abstraction matérielle permettant de découpler
//! le code métier des implémentations spécifiques à la plateforme.


   #![allow(dead_code)]
   #![allow(unused_imports)]

pub mod i2c;

#[cfg(target_os = "linux")]
pub mod i2c_linux;

pub use i2c::BusI2c;

#[cfg(target_os = "linux")]
pub use i2c_linux::I2cLinux;

