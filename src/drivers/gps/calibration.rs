// src/drivers/gps/calibration.rs
//! Assistance GPS persistante — position et base d'orbites (AssistNow Autonomous)
//!
//! Réduit le TTFF au redémarrage en l'absence de sauvegarde matérielle (VBCKP)
//! sur le module. Voir doc/assistance_gps.md.

use crate::types::{Result, ErreursAirHaum, Horodatage};
use crate::systeme::calibration::CalibrationPersistante;

/// Validité de l'assistance sauvegardée (garde-fou large — la position reste
/// géographiquement pertinente bien plus longtemps que ça, mais on évite de
/// rejouer indéfiniment une sauvegarde très ancienne). Voir doc/assistance_gps.md.
pub const VALIDITE_ASSISTANCE_SEC: u64 = 60 * 60 * 24 * 30; // 30 jours

/// Assistance GPS sauvegardée : dernière position connue et base d'orbites
/// prédites (AssistNow Autonomous, trames UBX-MGA-DBD brutes et opaques).
#[derive(Debug, Clone)]
pub struct AssistanceGps {
    timestamp_unix_sec: u64,
    pub latitude:     f64,
    pub longitude:    f64,
    pub altitude_msl: f32,
    /// Trames UBX-MGA-DBD capturées, chacune préfixée de sa longueur (u16 LE).
    pub orbites: Vec<u8>,
}

impl AssistanceGps {
    pub fn nouvelle(latitude: f64, longitude: f64, altitude_msl: f32, orbites: Vec<u8>) -> Self {
        Self {
            timestamp_unix_sec: unix_maintenant(),
            latitude, longitude, altitude_msl, orbites,
        }
    }
}

fn unix_maintenant() -> u64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap_or_default()
        .as_secs()
}

fn vers_hex(donnees: &[u8]) -> String {
    donnees.iter().map(|b| format!("{:02x}", b)).collect()
}

fn depuis_hex(texte: &str) -> Option<Vec<u8>> {
    if texte.len() % 2 != 0 { return None; }
    (0..texte.len())
        .step_by(2)
        .map(|i| u8::from_str_radix(&texte[i..i + 2], 16).ok())
        .collect()
}

impl CalibrationPersistante for AssistanceGps {
    fn identifiant_capteur() -> &'static str {
        "assistance_gps"
    }

    fn vers_toml(&self) -> String {
        format!(
            "# Assistance GPS — générée automatiquement, ne pas éditer\n\
             \n\
             timestamp_unix_sec = {}\n\
             latitude = {}\n\
             longitude = {}\n\
             altitude_msl = {}\n\
             orbites_hex = \"{}\"\n",
            self.timestamp_unix_sec,
            self.latitude,
            self.longitude,
            self.altitude_msl,
            vers_hex(&self.orbites),
        )
    }

    fn depuis_toml(contenu: &str) -> Result<Self> {
        let mut timestamp_unix_sec = None;
        let mut latitude     = None;
        let mut longitude    = None;
        let mut altitude_msl = None;
        let mut orbites      = None;

        for ligne in contenu.lines() {
            let ligne = ligne.trim();
            if ligne.starts_with('#') || ligne.is_empty() {
                continue;
            }
            if let Some((cle, valeur)) = ligne.split_once('=') {
                let valeur = valeur.trim().trim_matches('"');
                match cle.trim() {
                    "timestamp_unix_sec" => timestamp_unix_sec = valeur.parse::<u64>().ok(),
                    "latitude"           => latitude = valeur.parse::<f64>().ok(),
                    "longitude"          => longitude = valeur.parse::<f64>().ok(),
                    "altitude_msl"       => altitude_msl = valeur.parse::<f32>().ok(),
                    "orbites_hex"        => orbites = depuis_hex(valeur),
                    _ => {}
                }
            }
        }

        Ok(Self {
            timestamp_unix_sec: timestamp_unix_sec.ok_or_else(|| ErreursAirHaum::CalibrationEchouee(
                "Champ 'timestamp_unix_sec' manquant".into()))?,
            latitude: latitude.ok_or_else(|| ErreursAirHaum::CalibrationEchouee(
                "Champ 'latitude' manquant".into()))?,
            longitude: longitude.ok_or_else(|| ErreursAirHaum::CalibrationEchouee(
                "Champ 'longitude' manquant".into()))?,
            altitude_msl: altitude_msl.ok_or_else(|| ErreursAirHaum::CalibrationEchouee(
                "Champ 'altitude_msl' manquant".into()))?,
            orbites: orbites.unwrap_or_default(),
        })
    }

    fn est_valide(&self) -> bool {
        self.age_secondes() < self.duree_validite_secondes()
    }

    fn obtenir_horodatage(&self) -> Horodatage {
        Horodatage::maintenant()
    }

    fn age_secondes(&self) -> u64 {
        unix_maintenant().saturating_sub(self.timestamp_unix_sec)
    }

    fn duree_validite_secondes(&self) -> u64 {
        VALIDITE_ASSISTANCE_SEC
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn roundtrip_serialisation() {
        let a = AssistanceGps::nouvelle(47.993716, 0.185957, 55.5, vec![0xAA, 0xBB, 0x01, 0x00, 0xFF]);
        let toml = a.vers_toml();
        let b = AssistanceGps::depuis_toml(&toml).unwrap();
        assert_eq!(a.latitude, b.latitude);
        assert_eq!(a.longitude, b.longitude);
        assert_eq!(a.altitude_msl, b.altitude_msl);
        assert_eq!(a.orbites, b.orbites);
        assert!(b.est_valide());
    }

    #[test]
    fn orbites_vides() {
        let a = AssistanceGps::nouvelle(0.0, 0.0, 0.0, vec![]);
        let toml = a.vers_toml();
        let b = AssistanceGps::depuis_toml(&toml).unwrap();
        assert!(b.orbites.is_empty());
    }
}
