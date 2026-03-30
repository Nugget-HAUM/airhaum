// src/types/constantes.rs
// Constantes physiques pour l'aéronautique

   #![allow(dead_code)]
   #![allow(unused_imports)]

/// Accélération de la pesanteur terrestre (m/s²)
pub const GRAVITE_TERRESTRE: f32 = 9.80665;

/// Pression atmosphérique standard au niveau de la mer (Pa)
pub const PRESSION_NIVEAU_MER_STANDARD: f32 = 101325.0;

/// Température standard au niveau de la mer (°C)
pub const TEMPERATURE_STANDARD: f32 = 15.0;

/// Rayon terrestre moyen (m)
pub const RAYON_TERRE: f32 = 6371000.0;

/// Conversion degrés → radians
pub const DEG_VERS_RAD: f32 = std::f32::consts::PI / 180.0;

/// Conversion radians → degrés
pub const RAD_VERS_DEG: f32 = 180.0 / std::f32::consts::PI;



// === Timeouts capteurs ===

/// Timeout d'initialisation BMP280 (ms)
pub const TIMEOUT_INIT_BMP280_MS: u64 = 2000;

/// Timeout d'initialisation VL53L0X (ms)
pub const TIMEOUT_INIT_VL53L0X_MS: u64 = 3000;

/// Timeout de lecture capteur générique (ms)
pub const TIMEOUT_LECTURE_CAPTEUR_MS: u64 = 100;

// === Limites de cohérence capteurs ===

/// Pression atmosphérique minimale acceptable (Pa) - ~300 hPa
pub const PRESSION_MIN_PA: f32 = 30000.0;

/// Pression atmosphérique maximale acceptable (Pa) - ~1100 hPa
pub const PRESSION_MAX_PA: f32 = 110000.0;

/// Température minimale BMP280 (°C)
pub const TEMP_MIN_BMP280_C: f32 = -40.0;

/// Température maximale BMP280 (°C)
pub const TEMP_MAX_BMP280_C: f32 = 85.0;

/// Distance maximale valide VL53L0X (mm)
pub const DISTANCE_MAX_VL53L0X_MM: u16 = 8190;

/// Variation maximale de pression acceptable entre deux lectures (Pa/s)
pub const VARIATION_PRESSION_MAX_PA_S: f32 = 5000.0;
