// src/drivers/gps/mod.rs
//! Driver GPS u-blox NEO-M8N (protocole UBX sur UART).

pub mod ubx_parser;
pub mod ublox;

pub use ublox::DriverGps;
