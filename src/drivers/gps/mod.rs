// src/drivers/gps/mod.rs
//! Driver GPS u-blox NEO-M8N (protocole UBX sur UART).

pub mod ubx_parser;
pub mod ublox;
pub mod calibration;

pub use ublox::DriverGps;
pub use calibration::AssistanceGps;
