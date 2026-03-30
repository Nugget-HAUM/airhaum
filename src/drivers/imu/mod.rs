// src/drivers/imu/mod.rs
//! Drivers pour centrales inertielles
//!
//! Ce module contient les drivers pour les IMU 9 axes.

pub mod mpu9250;
pub mod calibration;

pub use mpu9250::{Mpu9250, ADRESSE_MPU9250, ADRESSE_MPU9250_ALT};
pub use calibration::{CalibrationGyro, CalibrationAccel, CalibrationMag};
