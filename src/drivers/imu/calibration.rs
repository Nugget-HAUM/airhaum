// src/drivers/imu/calibration.rs
//! Calibrations de la centrale inertielle MPU9250
//!
//! Procédures et justification des validités : `doc/calibration.md`.
//!
//! | Calibration                  | Validité | Procédure                                   |
//! |------------------------------|----------|---------------------------------------------|
//! | Gyroscope + Z accéléromètre  | 1 heure  | 5 s immobile au sol (terrain), vérifie le montage |
//! | Accéléromètre — montage      | 6 mois   | queue au sol puis empennage à l'horizontale (atelier) |
//! | Magnétomètre                 | 1 heure  | un tour complet sur les roues (terrain)     |
//!
//! Format de fichier en version 2 : les fichiers antérieurs (méthodes
//! remplacées en 2026-10) sont refusés au chargement, ce qui impose de
//! recalibrer.

use crate::types::{Result, ErreursAirHaum, Horodatage};
use crate::systeme::calibration::CalibrationPersistante;
use crate::drivers::imu::calcul_calibration::Matrice3;

/// Version du format des fichiers de calibration IMU.
const VERSION_FORMAT: u64 = 2;

// ============================================================================
// CalibrationGyro
// ============================================================================

/// Calibration de session : biais du gyroscope et décalage de Z de
/// l'accéléromètre, mesurés ensemble 5 s immobile sur le terrain
/// (`doc/calibration.md`, § 4).
///
/// Valide 1 heure. N'est enregistrée qu'après vérification du montage :
/// une calibration gyroscope valide garantit un montage vérifié.
#[derive(Debug, Clone, Copy)]
pub struct CalibrationGyro {
    timestamp_unix_sec: u64,
    validite_sec: u64,
    /// Biais en rad/s sur chaque axe (repère capteur)
    pub offset_x: f32,
    pub offset_y: f32,
    pub offset_z: f32,
    /// Décalage de l'axe Z de l'accéléromètre en m/s² (repère capteur),
    /// réajusté à chaque session — remplace celui de l'atelier.
    pub offset_z_accel: f32,
}

impl CalibrationGyro {
    pub fn nouvelle(offset_x: f32, offset_y: f32, offset_z: f32, offset_z_accel: f32) -> Self {
        Self {
            timestamp_unix_sec: maintenant_unix(),
            validite_sec: 3600, // 1 heure
            offset_x,
            offset_y,
            offset_z,
            offset_z_accel,
        }
    }
}

impl CalibrationPersistante for CalibrationGyro {
    fn identifiant_capteur() -> &'static str {
        "imu_gyro"
    }

    fn vers_toml(&self) -> String {
        format!(
            "# Calibration gyroscope + Z accéléromètre MPU9250 (session terrain)\n\
             # Générée automatiquement - ne pas éditer manuellement\n\
             # Procédure : doc/calibration.md § 4\n\
             \n\
             version = {}\n\
             timestamp_unix_sec = {}\n\
             validite_sec = {}\n\
             # Biais gyroscope (rad/s, repère capteur)\n\
             offset_x = {:.8}\n\
             offset_y = {:.8}\n\
             offset_z = {:.8}\n\
             # Décalage Z accéléromètre (m/s², repère capteur)\n\
             offset_z_accel = {:.8}\n",
            VERSION_FORMAT,
            self.timestamp_unix_sec,
            self.validite_sec,
            self.offset_x,
            self.offset_y,
            self.offset_z,
            self.offset_z_accel,
        )
    }

    fn depuis_toml(contenu: &str) -> Result<Self> {
        let champs = lire_champs(contenu)?;
        Ok(Self {
            timestamp_unix_sec: champs.u64("timestamp_unix_sec")?,
            validite_sec:       champs.u64("validite_sec")?,
            offset_x:           champs.f32("offset_x")?,
            offset_y:           champs.f32("offset_y")?,
            offset_z:           champs.f32("offset_z")?,
            offset_z_accel:     champs.f32("offset_z_accel")?,
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

    fn duree_validite_secondes(&self) -> u64 { self.validite_sec }
}

// ============================================================================
// CalibrationAccel
// ============================================================================

/// Calibration de montage de la carte dans l'avion, faite à l'atelier
/// (`doc/calibration.md`, § 3).
///
/// Valide 6 mois, vérifiée à chaque session par la calibration gyroscope.
#[derive(Debug, Clone, Copy)]
pub struct CalibrationAccel {
    timestamp_unix_sec: u64,
    validite_sec: u64,
    /// Décalage de l'axe Z en m/s² (repère capteur), mesuré à l'atelier.
    /// Utilisé tant qu'aucune calibration de session ne l'a réajusté.
    pub offset_z: f32,
    /// Rotation repère capteur → repère avion (lignes = axes avion X, Y, Z
    /// exprimés dans le repère capteur).
    pub montage: Matrice3,
    /// Attitude de l'avion queue au sol à l'atelier (degrés), référence de
    /// la vérification de session.
    pub attitude_sol_roulis_deg: f32,
    pub attitude_sol_tangage_deg: f32,
}

impl CalibrationAccel {
    pub fn nouvelle(
        offset_z: f32,
        montage: Matrice3,
        attitude_sol_roulis_deg: f32,
        attitude_sol_tangage_deg: f32,
    ) -> Self {
        Self {
            timestamp_unix_sec: maintenant_unix(),
            validite_sec: 183 * 24 * 3600, // 6 mois
            offset_z,
            montage,
            attitude_sol_roulis_deg,
            attitude_sol_tangage_deg,
        }
    }
}

impl CalibrationPersistante for CalibrationAccel {
    fn identifiant_capteur() -> &'static str {
        "imu_accel"
    }

    fn vers_toml(&self) -> String {
        let m = &self.montage;
        format!(
            "# Calibration de montage accéléromètre MPU9250 (atelier)\n\
             # Générée automatiquement - ne pas éditer manuellement\n\
             # Procédure : doc/calibration.md § 3\n\
             \n\
             version = {}\n\
             timestamp_unix_sec = {}\n\
             validite_sec = {}\n\
             # Décalage Z (m/s², repère capteur)\n\
             offset_z = {:.8}\n\
             # Rotation capteur → avion : ligne i = axe avion i (X nez, Y aile droite, Z bas)\n\
             montage_11 = {:.8}\n\
             montage_12 = {:.8}\n\
             montage_13 = {:.8}\n\
             montage_21 = {:.8}\n\
             montage_22 = {:.8}\n\
             montage_23 = {:.8}\n\
             montage_31 = {:.8}\n\
             montage_32 = {:.8}\n\
             montage_33 = {:.8}\n\
             # Attitude queue au sol à l'atelier (degrés)\n\
             attitude_sol_roulis_deg = {:.4}\n\
             attitude_sol_tangage_deg = {:.4}\n",
            VERSION_FORMAT,
            self.timestamp_unix_sec,
            self.validite_sec,
            self.offset_z,
            m[0][0], m[0][1], m[0][2],
            m[1][0], m[1][1], m[1][2],
            m[2][0], m[2][1], m[2][2],
            self.attitude_sol_roulis_deg,
            self.attitude_sol_tangage_deg,
        )
    }

    fn depuis_toml(contenu: &str) -> Result<Self> {
        let champs = lire_champs(contenu)?;
        let mut montage = [[0.0f32; 3]; 3];
        for (i, ligne) in montage.iter_mut().enumerate() {
            for (j, valeur) in ligne.iter_mut().enumerate() {
                *valeur = champs.f32(&format!("montage_{}{}", i + 1, j + 1))?;
            }
        }
        Ok(Self {
            timestamp_unix_sec:       champs.u64("timestamp_unix_sec")?,
            validite_sec:             champs.u64("validite_sec")?,
            offset_z:                 champs.f32("offset_z")?,
            montage,
            attitude_sol_roulis_deg:  champs.f32("attitude_sol_roulis_deg")?,
            attitude_sol_tangage_deg: champs.f32("attitude_sol_tangage_deg")?,
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

    fn duree_validite_secondes(&self) -> u64 { self.validite_sec }
}

// ============================================================================
// CalibrationMag
// ============================================================================

/// Calibration du magnétomètre par un tour de l'avion au sol, sur le
/// terrain (`doc/calibration.md`, § 5).
///
/// Décalage dû à l'avion (« hard iron ») seulement : un tour à plat ne
/// permet pas d'estimer une correction « soft iron ». Les coefficients
/// usine ASA ne sont pas stockés : le driver les relit à chaque
/// initialisation.
///
/// Valide 1 heure.
#[derive(Debug, Clone, Copy)]
pub struct CalibrationMag {
    timestamp_unix_sec: u64,
    validite_sec: u64,
    /// Décalage en µT (repère capteur, axes AK8963 réalignés)
    pub hard_iron_x: f32,
    pub hard_iron_y: f32,
    pub hard_iron_z: f32,
    /// Composante horizontale du champ mesurée pendant le tour (µT)
    pub champ_horizontal_ut: f32,
    /// Dispersion RMS des points autour du cercle ajusté (µT)
    pub dispersion_ut: f32,
}

impl CalibrationMag {
    pub fn nouvelle(
        hard_iron_x: f32, hard_iron_y: f32, hard_iron_z: f32,
        champ_horizontal_ut: f32, dispersion_ut: f32,
    ) -> Self {
        Self {
            timestamp_unix_sec: maintenant_unix(),
            validite_sec: 3600, // 1 heure
            hard_iron_x, hard_iron_y, hard_iron_z,
            champ_horizontal_ut, dispersion_ut,
        }
    }
}

impl CalibrationPersistante for CalibrationMag {
    fn identifiant_capteur() -> &'static str {
        "imu_mag"
    }

    fn vers_toml(&self) -> String {
        format!(
            "# Calibration magnétomètre AK8963 (tour au sol, terrain)\n\
             # Générée automatiquement - ne pas éditer manuellement\n\
             # Procédure : doc/calibration.md § 5\n\
             \n\
             version = {}\n\
             timestamp_unix_sec = {}\n\
             validite_sec = {}\n\
             # Décalage dû à l'avion (µT, repère capteur)\n\
             hard_iron_x = {:.8}\n\
             hard_iron_y = {:.8}\n\
             hard_iron_z = {:.8}\n\
             # Qualité du tour (µT)\n\
             champ_horizontal_ut = {:.4}\n\
             dispersion_ut = {:.4}\n",
            VERSION_FORMAT,
            self.timestamp_unix_sec, self.validite_sec,
            self.hard_iron_x, self.hard_iron_y, self.hard_iron_z,
            self.champ_horizontal_ut, self.dispersion_ut,
        )
    }

    fn depuis_toml(contenu: &str) -> Result<Self> {
        let champs = lire_champs(contenu)?;
        Ok(Self {
            timestamp_unix_sec:  champs.u64("timestamp_unix_sec")?,
            validite_sec:        champs.u64("validite_sec")?,
            hard_iron_x:         champs.f32("hard_iron_x")?,
            hard_iron_y:         champs.f32("hard_iron_y")?,
            hard_iron_z:         champs.f32("hard_iron_z")?,
            champ_horizontal_ut: champs.f32("champ_horizontal_ut")?,
            dispersion_ut:       champs.f32("dispersion_ut")?,
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

    fn duree_validite_secondes(&self) -> u64 { self.validite_sec }
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

/// Champs `cle = valeur` d'un fichier de calibration, version vérifiée.
struct Champs(std::collections::HashMap<String, String>);

impl Champs {
    fn brut(&self, nom: &str) -> Result<&str> {
        self.0.get(nom).map(String::as_str).ok_or_else(|| {
            ErreursAirHaum::CalibrationEchouee(format!("Champ '{}' manquant", nom))
        })
    }

    fn u64(&self, nom: &str) -> Result<u64> {
        let val = self.brut(nom)?;
        val.parse::<u64>().map_err(|_| {
            ErreursAirHaum::CalibrationEchouee(format!("Valeur u64 invalide: '{}'", val))
        })
    }

    fn f32(&self, nom: &str) -> Result<f32> {
        let val = self.brut(nom)?;
        val.parse::<f32>().map_err(|_| {
            ErreursAirHaum::CalibrationEchouee(format!("Valeur f32 invalide: '{}'", val))
        })
    }
}

fn lire_champs(contenu: &str) -> Result<Champs> {
    let mut champs = std::collections::HashMap::new();
    for ligne in contenu.lines() {
        let ligne = ligne.trim();
        if ligne.starts_with('#') || ligne.is_empty() { continue; }
        if let Some((cle, val)) = ligne.split_once('=') {
            champs.insert(cle.trim().to_string(), val.trim().to_string());
        }
    }
    let champs = Champs(champs);
    match champs.u64("version") {
        Ok(VERSION_FORMAT) => Ok(champs),
        _ => Err(ErreursAirHaum::CalibrationEchouee(
            "ancien format de calibration (méthode remplacée, voir doc/calibration.md) — recalibrer".into()
        )),
    }
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::drivers::imu::calcul_calibration::MONTAGE_DEFAUT;

    #[test]
    fn test_roundtrip_gyro() {
        let c1 = CalibrationGyro::nouvelle(0.001, -0.002, 0.0005, 4.36);
        let c2 = CalibrationGyro::depuis_toml(&c1.vers_toml()).unwrap();
        assert!((c1.offset_x - c2.offset_x).abs() < 1e-6);
        assert!((c1.offset_y - c2.offset_y).abs() < 1e-6);
        assert!((c1.offset_z - c2.offset_z).abs() < 1e-6);
        assert!((c1.offset_z_accel - c2.offset_z_accel).abs() < 1e-6);
        assert_eq!(c1.timestamp_unix_sec, c2.timestamp_unix_sec);
        assert_eq!(c2.duree_validite_secondes(), 3600);
    }

    #[test]
    fn test_roundtrip_accel() {
        let c1 = CalibrationAccel::nouvelle(4.36, MONTAGE_DEFAUT, 0.4, 10.2);
        let c2 = CalibrationAccel::depuis_toml(&c1.vers_toml()).unwrap();
        assert!((c1.offset_z - c2.offset_z).abs() < 1e-6);
        assert_eq!(c1.montage, c2.montage);
        assert!((c1.attitude_sol_tangage_deg - c2.attitude_sol_tangage_deg).abs() < 1e-4);
        assert_eq!(c2.duree_validite_secondes(), 183 * 24 * 3600);
    }

    #[test]
    fn test_roundtrip_mag() {
        let c1 = CalibrationMag::nouvelle(-12.5, 8.3, 3.1, 21.6, 0.8);
        let c2 = CalibrationMag::depuis_toml(&c1.vers_toml()).unwrap();
        assert!((c1.hard_iron_x - c2.hard_iron_x).abs() < 1e-6);
        assert!((c1.hard_iron_z - c2.hard_iron_z).abs() < 1e-6);
        assert!((c1.champ_horizontal_ut - c2.champ_horizontal_ut).abs() < 1e-4);
    }

    #[test]
    fn test_ancien_format_refuse() {
        // Fichier accéléromètre du 2026-10-21 (méthode une position)
        let ancien = "timestamp_unix_sec = 1792533981\nvalidite_sec = 604800\n\
                      offset_x = 0.38\noffset_y = 0.81\noffset_z = 4.36\n\
                      scale_x = 1.0\nscale_y = 1.0\nscale_z = 1.0\n";
        assert!(CalibrationAccel::depuis_toml(ancien).is_err());
    }

    #[test]
    fn test_gyro_valide_apres_creation() {
        let c = CalibrationGyro::nouvelle(0.0, 0.0, 0.0, 0.0);
        assert!(c.est_valide());
    }
}
