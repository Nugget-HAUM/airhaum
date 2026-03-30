// src/drivers/imu/calibration.rs
//! Calibrations de la centrale inertielle MPU9250
//!
//! Trois calibrations indépendantes avec durées de validité différentes :
//!
//! | Calibration | Validité | Procédure          |
//! |-------------|----------|--------------------|
//! | Gyroscope   | 4 heures | 5s immobile au sol |
//! | Accéléro    | 7 jours  | Sol plat           |
//! | Magnéto     | 30 jours | Procédure figure-8 |

use crate::types::{Result, ErreursAirHaum, Horodatage};
use crate::systeme::calibration::CalibrationPersistante;

// ============================================================================
// CalibrationGyro
// ============================================================================

/// Calibration du gyroscope
///
/// Stocke les offsets mesurés capteur immobile.
/// Valide 4 heures (dérive thermique significative).
#[derive(Debug, Clone, Copy)]
pub struct CalibrationGyro {
    timestamp_unix_sec: u64,
    validite_sec: u64,
    /// Offsets en rad/s sur chaque axe
    pub offset_x: f32,
    pub offset_y: f32,
    pub offset_z: f32,
}

impl CalibrationGyro {
    pub fn nouvelle(offset_x: f32, offset_y: f32, offset_z: f32) -> Self {
        Self {
            timestamp_unix_sec: maintenant_unix(),
            validite_sec: 4 * 3600, // 4 heures
            offset_x,
            offset_y,
            offset_z,
        }
    }
}

impl CalibrationPersistante for CalibrationGyro {
    fn identifiant_capteur() -> &'static str {
        "imu_gyro"
    }

    fn vers_toml(&self) -> String {
        format!(
            "# Calibration gyroscope MPU9250\n\
             # Générée automatiquement - ne pas éditer manuellement\n\
             \n\
             timestamp_unix_sec = {}\n\
             validite_sec = {}\n\
             offset_x = {:.8}\n\
             offset_y = {:.8}\n\
             offset_z = {:.8}\n",
            self.timestamp_unix_sec,
            self.validite_sec,
            self.offset_x,
            self.offset_y,
            self.offset_z,
        )
    }

    fn depuis_toml(contenu: &str) -> Result<Self> {
        let mut timestamp = None;
        let mut validite = None;
        let mut offset_x = None;
        let mut offset_y = None;
        let mut offset_z = None;

        for ligne in contenu.lines() {
            let ligne = ligne.trim();
            if ligne.starts_with('#') || ligne.is_empty() { continue; }
            if let Some((cle, val)) = ligne.split_once('=') {
                match cle.trim() {
                    "timestamp_unix_sec" => timestamp = Some(parse_u64(val)?),
                    "validite_sec"       => validite  = Some(parse_u64(val)?),
                    "offset_x"           => offset_x  = Some(parse_f32(val)?),
                    "offset_y"           => offset_y  = Some(parse_f32(val)?),
                    "offset_z"           => offset_z  = Some(parse_f32(val)?),
                    _ => {}
                }
            }
        }

        Ok(Self {
            timestamp_unix_sec: champ_requis(timestamp, "timestamp_unix_sec")?,
            validite_sec:       champ_requis(validite,  "validite_sec")?,
            offset_x:           champ_requis(offset_x,  "offset_x")?,
            offset_y:           champ_requis(offset_y,  "offset_y")?,
            offset_z:           champ_requis(offset_z,  "offset_z")?,
        })
    }

    fn est_valide(&self) -> bool {
        self.age_secondes() < self.validite_sec
    }

    fn obtenir_horodatage(&self) -> Horodatage {
        Horodatage::maintenant()
    }

    fn age_secondes(&self) -> u64 {
        maintenant_unix().saturating_sub(self.timestamp_unix_sec)
    }
}

// ============================================================================
// CalibrationAccel
// ============================================================================

/// Calibration de l'accéléromètre
///
/// Stocke les offsets et facteurs d'échelle par axe.
/// Valide 7 jours.
#[derive(Debug, Clone, Copy)]
pub struct CalibrationAccel {
    timestamp_unix_sec: u64,
    validite_sec: u64,
    /// Offsets en m/s²
    pub offset_x: f32,
    pub offset_y: f32,
    pub offset_z: f32,
    /// Facteurs d'échelle (nominalement 1.0)
    pub scale_x: f32,
    pub scale_y: f32,
    pub scale_z: f32,
}

impl CalibrationAccel {
    pub fn nouvelle(
        offset_x: f32, offset_y: f32, offset_z: f32,
        scale_x: f32,  scale_y: f32,  scale_z: f32,
    ) -> Self {
        Self {
            timestamp_unix_sec: maintenant_unix(),
            validite_sec: 7 * 24 * 3600, // 7 jours
            offset_x, offset_y, offset_z,
            scale_x, scale_y, scale_z,
        }
    }
}

impl CalibrationPersistante for CalibrationAccel {
    fn identifiant_capteur() -> &'static str {
        "imu_accel"
    }

    fn vers_toml(&self) -> String {
        format!(
            "# Calibration accéléromètre MPU9250\n\
             # Générée automatiquement - ne pas éditer manuellement\n\
             \n\
             timestamp_unix_sec = {}\n\
             validite_sec = {}\n\
             offset_x = {:.8}\n\
             offset_y = {:.8}\n\
             offset_z = {:.8}\n\
             scale_x = {:.8}\n\
             scale_y = {:.8}\n\
             scale_z = {:.8}\n",
            self.timestamp_unix_sec,
            self.validite_sec,
            self.offset_x, self.offset_y, self.offset_z,
            self.scale_x,  self.scale_y,  self.scale_z,
        )
    }

    fn depuis_toml(contenu: &str) -> Result<Self> {
        let mut timestamp = None;
        let mut validite  = None;
        let mut offset_x  = None;
        let mut offset_y  = None;
        let mut offset_z  = None;
        let mut scale_x   = None;
        let mut scale_y   = None;
        let mut scale_z   = None;

        for ligne in contenu.lines() {
            let ligne = ligne.trim();
            if ligne.starts_with('#') || ligne.is_empty() { continue; }
            if let Some((cle, val)) = ligne.split_once('=') {
                match cle.trim() {
                    "timestamp_unix_sec" => timestamp = Some(parse_u64(val)?),
                    "validite_sec"       => validite  = Some(parse_u64(val)?),
                    "offset_x"           => offset_x  = Some(parse_f32(val)?),
                    "offset_y"           => offset_y  = Some(parse_f32(val)?),
                    "offset_z"           => offset_z  = Some(parse_f32(val)?),
                    "scale_x"            => scale_x   = Some(parse_f32(val)?),
                    "scale_y"            => scale_y   = Some(parse_f32(val)?),
                    "scale_z"            => scale_z   = Some(parse_f32(val)?),
                    _ => {}
                }
            }
        }

        Ok(Self {
            timestamp_unix_sec: champ_requis(timestamp, "timestamp_unix_sec")?,
            validite_sec:       champ_requis(validite,  "validite_sec")?,
            offset_x:           champ_requis(offset_x,  "offset_x")?,
            offset_y:           champ_requis(offset_y,  "offset_y")?,
            offset_z:           champ_requis(offset_z,  "offset_z")?,
            scale_x:            champ_requis(scale_x,   "scale_x")?,
            scale_y:            champ_requis(scale_y,   "scale_y")?,
            scale_z:            champ_requis(scale_z,   "scale_z")?,
        })
    }

    fn est_valide(&self) -> bool {
        self.age_secondes() < self.validite_sec
    }

    fn obtenir_horodatage(&self) -> Horodatage {
        Horodatage::maintenant()
    }

    fn age_secondes(&self) -> u64 {
        maintenant_unix().saturating_sub(self.timestamp_unix_sec)
    }
}

// ============================================================================
// CalibrationMag
// ============================================================================

/// Calibration du magnétomètre AK8963
///
/// Stocke :
/// - Les coefficients de sensibilité usine (ASA, lus une fois depuis le chip)
/// - Les corrections hard iron (offset)
/// - Les corrections soft iron simplifiées (scale)
///
/// Valide 30 jours.
#[derive(Debug, Clone, Copy)]
pub struct CalibrationMag {
    timestamp_unix_sec: u64,
    validite_sec: u64,
    /// Coefficients de sensibilité usine AK8963 (Hadj = (ASA-128)/256 + 1)
    pub asa_x: f32,
    pub asa_y: f32,
    pub asa_z: f32,
    /// Corrections hard iron (offset en µT)
    pub hard_iron_x: f32,
    pub hard_iron_y: f32,
    pub hard_iron_z: f32,
    /// Corrections soft iron simplifiées (scale, nominalement 1.0)
    pub soft_iron_x: f32,
    pub soft_iron_y: f32,
    pub soft_iron_z: f32,
}

impl CalibrationMag {
    pub fn nouvelle(
        asa_x: f32, asa_y: f32, asa_z: f32,
        hard_iron_x: f32, hard_iron_y: f32, hard_iron_z: f32,
        soft_iron_x: f32, soft_iron_y: f32, soft_iron_z: f32,
    ) -> Self {
        Self {
            timestamp_unix_sec: maintenant_unix(),
            validite_sec: 30 * 24 * 3600, // 30 jours
            asa_x, asa_y, asa_z,
            hard_iron_x, hard_iron_y, hard_iron_z,
            soft_iron_x, soft_iron_y, soft_iron_z,
        }
    }

    /// Crée une calibration minimale avec uniquement les coefficients usine
    ///
    /// Utilisée lors de la première initialisation, avant la procédure
    /// de calibration hard/soft iron complète.
    pub fn depuis_asa_uniquement(asa_x: f32, asa_y: f32, asa_z: f32) -> Self {
        Self::nouvelle(
            asa_x, asa_y, asa_z,
            0.0, 0.0, 0.0,   // hard iron nuls
            1.0, 1.0, 1.0,   // soft iron neutres
        )
    }
}

impl CalibrationPersistante for CalibrationMag {
    fn identifiant_capteur() -> &'static str {
        "imu_mag"
    }

    fn vers_toml(&self) -> String {
        format!(
            "# Calibration magnétomètre AK8963\n\
             # Générée automatiquement - ne pas éditer manuellement\n\
             \n\
             timestamp_unix_sec = {}\n\
             validite_sec = {}\n\
             # Coefficients sensibilité usine\n\
             asa_x = {:.8}\n\
             asa_y = {:.8}\n\
             asa_z = {:.8}\n\
             # Corrections hard iron (µT)\n\
             hard_iron_x = {:.8}\n\
             hard_iron_y = {:.8}\n\
             hard_iron_z = {:.8}\n\
             # Corrections soft iron\n\
             soft_iron_x = {:.8}\n\
             soft_iron_y = {:.8}\n\
             soft_iron_z = {:.8}\n",
            self.timestamp_unix_sec, self.validite_sec,
            self.asa_x, self.asa_y, self.asa_z,
            self.hard_iron_x, self.hard_iron_y, self.hard_iron_z,
            self.soft_iron_x, self.soft_iron_y, self.soft_iron_z,
        )
    }

    fn depuis_toml(contenu: &str) -> Result<Self> {
        let mut timestamp    = None;
        let mut validite     = None;
        let mut asa_x        = None;
        let mut asa_y        = None;
        let mut asa_z        = None;
        let mut hard_iron_x  = None;
        let mut hard_iron_y  = None;
        let mut hard_iron_z  = None;
        let mut soft_iron_x  = None;
        let mut soft_iron_y  = None;
        let mut soft_iron_z  = None;

        for ligne in contenu.lines() {
            let ligne = ligne.trim();
            if ligne.starts_with('#') || ligne.is_empty() { continue; }
            if let Some((cle, val)) = ligne.split_once('=') {
                match cle.trim() {
                    "timestamp_unix_sec" => timestamp   = Some(parse_u64(val)?),
                    "validite_sec"       => validite    = Some(parse_u64(val)?),
                    "asa_x"              => asa_x       = Some(parse_f32(val)?),
                    "asa_y"              => asa_y       = Some(parse_f32(val)?),
                    "asa_z"              => asa_z       = Some(parse_f32(val)?),
                    "hard_iron_x"        => hard_iron_x = Some(parse_f32(val)?),
                    "hard_iron_y"        => hard_iron_y = Some(parse_f32(val)?),
                    "hard_iron_z"        => hard_iron_z = Some(parse_f32(val)?),
                    "soft_iron_x"        => soft_iron_x = Some(parse_f32(val)?),
                    "soft_iron_y"        => soft_iron_y = Some(parse_f32(val)?),
                    "soft_iron_z"        => soft_iron_z = Some(parse_f32(val)?),
                    _ => {}
                }
            }
        }

        Ok(Self {
            timestamp_unix_sec: champ_requis(timestamp,   "timestamp_unix_sec")?,
            validite_sec:       champ_requis(validite,    "validite_sec")?,
            asa_x:              champ_requis(asa_x,       "asa_x")?,
            asa_y:              champ_requis(asa_y,       "asa_y")?,
            asa_z:              champ_requis(asa_z,       "asa_z")?,
            hard_iron_x:        champ_requis(hard_iron_x, "hard_iron_x")?,
            hard_iron_y:        champ_requis(hard_iron_y, "hard_iron_y")?,
            hard_iron_z:        champ_requis(hard_iron_z, "hard_iron_z")?,
            soft_iron_x:        champ_requis(soft_iron_x, "soft_iron_x")?,
            soft_iron_y:        champ_requis(soft_iron_y, "soft_iron_y")?,
            soft_iron_z:        champ_requis(soft_iron_z, "soft_iron_z")?,
        })
    }

    fn est_valide(&self) -> bool {
        self.age_secondes() < self.validite_sec
    }

    fn obtenir_horodatage(&self) -> Horodatage {
        Horodatage::maintenant()
    }

    fn age_secondes(&self) -> u64 {
        maintenant_unix().saturating_sub(self.timestamp_unix_sec)
    }
}

// ============================================================================
// Utilitaires internes
// ============================================================================

fn maintenant_unix() -> u64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap_or_default()
        .as_secs()
}

fn parse_u64(val: &str) -> Result<u64> {
    val.trim().parse::<u64>().map_err(|_| {
        ErreursAirHaum::CalibrationEchouee(format!("Valeur u64 invalide: '{}'", val.trim()))
    })
}

fn parse_f32(val: &str) -> Result<f32> {
    val.trim().parse::<f32>().map_err(|_| {
        ErreursAirHaum::CalibrationEchouee(format!("Valeur f32 invalide: '{}'", val.trim()))
    })
}

fn champ_requis<T>(opt: Option<T>, nom: &str) -> Result<T> {
    opt.ok_or_else(|| {
        ErreursAirHaum::CalibrationEchouee(format!("Champ '{}' manquant", nom))
    })
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_roundtrip_gyro() {
        let c1 = CalibrationGyro::nouvelle(0.001, -0.002, 0.0005);
        let toml = c1.vers_toml();
        let c2 = CalibrationGyro::depuis_toml(&toml).unwrap();
        assert!((c1.offset_x - c2.offset_x).abs() < 1e-6);
        assert!((c1.offset_y - c2.offset_y).abs() < 1e-6);
        assert!((c1.offset_z - c2.offset_z).abs() < 1e-6);
        assert_eq!(c1.timestamp_unix_sec, c2.timestamp_unix_sec);
    }

    #[test]
    fn test_roundtrip_accel() {
        let c1 = CalibrationAccel::nouvelle(
            0.05, -0.03, 0.12,
            1.002, 0.998, 1.001,
        );
        let toml = c1.vers_toml();
        let c2 = CalibrationAccel::depuis_toml(&toml).unwrap();
        assert!((c1.offset_x - c2.offset_x).abs() < 1e-6);
        assert!((c1.scale_z  - c2.scale_z).abs()  < 1e-6);
    }

    #[test]
    fn test_roundtrip_mag() {
        let c1 = CalibrationMag::nouvelle(
            1.18, 1.15, 1.14,
            -12.5, 8.3, 3.1,
            1.02, 0.98, 1.01,
        );
        let toml = c1.vers_toml();
        let c2 = CalibrationMag::depuis_toml(&toml).unwrap();
        assert!((c1.asa_x       - c2.asa_x).abs()       < 1e-6);
        assert!((c1.hard_iron_x - c2.hard_iron_x).abs() < 1e-6);
        assert!((c1.soft_iron_y - c2.soft_iron_y).abs() < 1e-6);
    }

    #[test]
    fn test_gyro_valide_apres_creation() {
        let c = CalibrationGyro::nouvelle(0.0, 0.0, 0.0);
        assert!(c.est_valide());
    }

    #[test]
    fn test_mag_depuis_asa_uniquement() {
        let c = CalibrationMag::depuis_asa_uniquement(1.18, 1.15, 1.14);
        assert_eq!(c.hard_iron_x, 0.0);
        assert_eq!(c.soft_iron_x, 1.0);
        assert!(c.est_valide());
    }
}
