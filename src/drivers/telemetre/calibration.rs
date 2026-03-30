// src/drivers/telemetre/calibration.rs
//! Calibration système du télémètre VL53L0X
//!
//! Contrairement au BMP280, le VL53L0X ne nécessite pas de calibration
//! de mesure pré-vol. Ce fichier stocke uniquement le fait que le capteur
//! a été correctement initialisé, permettant une reprise rapide au
//! redémarrage sans refaire la séquence complète d'initialisation ST.

//use crate::types::{Result, ErreursAirHaum};
use crate::types::{Result, ErreursAirHaum, Horodatage};
use crate::systeme::calibration::CalibrationPersistante;
//use std::time::Duration;

/// Calibration du télémètre VL53L0X
///
/// Stocke l'état d'initialisation du capteur pour permettre
/// une reprise rapide au redémarrage.
#[derive(Debug, Clone, Copy)]
pub struct CalibrationTelemetre {
    /// Moment où l'initialisation a été effectuée
    timestamp_unix_sec: u64,  // Timestamp Unix pour persistance inter-sessions
    /// Durée de validité en secondes
    validite_sec: u64,
}

impl CalibrationTelemetre {
    /// Crée une nouvelle calibration après initialisation réussie
    pub fn nouvelle(validite_sec: u64) -> Self {
        Self {
            timestamp_unix_sec: std::time::SystemTime::now()
             .duration_since(std::time::UNIX_EPOCH)
             .unwrap_or_default()
             .as_secs(),
            validite_sec,
        }
    }
}

impl CalibrationPersistante for CalibrationTelemetre {
    fn identifiant_capteur() -> &'static str {
        "telemetre"
    }

    fn vers_toml(&self) -> String {
        format!(
            "# Calibration télémètre VL53L0X\n\
             # Générée automatiquement - ne pas éditer manuellement\n\
             \n\
             timestamp_unix_sec = {}\n\
             validite_sec = {}\n",
            self.timestamp_unix_sec,
            self.validite_sec
        )
    }

    fn depuis_toml(contenu: &str) -> Result<Self> {
        let mut timestamp_unix_sec = None;
        let mut validite = None;

        for ligne in contenu.lines() {
            let ligne = ligne.trim();
            if ligne.starts_with('#') || ligne.is_empty() {
                continue;
            }
            if let Some((cle, valeur)) = ligne.split_once('=') {
                match cle.trim() {
                     "timestamp_unix_sec" => {
                         timestamp_unix_sec = Some(
                          valeur.trim().parse::<u64>()
                           .map_err(|_| ErreursAirHaum::CalibrationEchouee(
                               format!("Timestamp invalide: {}", valeur)
                           ))?

                       );
                    }
                    "validite_sec" => {
                        validite = Some(
                            valeur.trim().parse::<u64>()
                                .map_err(|_| ErreursAirHaum::CalibrationEchouee(
                                    format!("Validité invalide: {}", valeur)
                                ))?
                        );
                    }
                    _ => {}
                }
            }
        }

        let timestamp = timestamp_unix_sec.ok_or_else(||
            ErreursAirHaum::CalibrationEchouee("Champ 'timestamp_unix_sec' manquant".into())
         )?; 

        let validite_sec = validite.ok_or_else(||
            ErreursAirHaum::CalibrationEchouee(
                "Champ 'validite_sec' manquant".into()
            )
        )?;

        Ok(Self {
           timestamp_unix_sec: timestamp,
           validite_sec,
        }) 
    }

    fn est_valide(&self) -> bool {
         self.age_secondes() < self.validite_sec
    }

    fn obtenir_horodatage(&self) -> Horodatage {
    // Non significatif pour les calibrations persistantes,
    // utiliser age_secondes() pour l'âge réel.
    Horodatage::maintenant()
    }

    fn age_secondes(&self) -> u64 {
      let maintenant = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap_or_default()
        .as_secs();
      maintenant.saturating_sub(self.timestamp_unix_sec)
    }   


}






#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_creation_calibration() {
        let calib = CalibrationTelemetre::nouvelle(3600);
        assert!(calib.est_valide());
    }

    #[test]
    fn test_roundtrip_serialisation() {
        let calib1 = CalibrationTelemetre::nouvelle(3600);
        let toml = calib1.vers_toml();
        let calib2 = CalibrationTelemetre::depuis_toml(&toml).unwrap();

        assert_eq!(calib1.timestamp_unix_sec, calib2.timestamp_unix_sec);
        assert_eq!(calib1.validite_sec, calib2.validite_sec);
    }
}
