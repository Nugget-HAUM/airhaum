// src/drivers/barometre/calibration.rs
//! Calibration système du baromètre
//!
//! Le baromètre nécessite une calibration de la pression de référence au sol
//! pour permettre le calcul d'altitude relative pendant le vol.
//!
//! ## Pourquoi calibrer ?
//!
//! Le BMP280 fournit des mesures de pression absolue très précises grâce à
//! ses coefficients de calibration usine. Cependant, pour calculer l'altitude,
//! nous avons besoin de connaître la pression au niveau du sol au moment du vol.
//!
//! Cette pression de référence varie selon :
//! - L'altitude du terrain de décollage
//! - Les conditions météorologiques (anticyclone/dépression)
//! - La température ambiante
//!
//! ## Procédure de calibration
//!
//! 1. Poser l'avion au sol, immobile
//! 2. Appeler `calibrer_pression_sol()` sur le driver
//! 3. La pression actuelle est mesurée et stockée comme référence
//! 4. La calibration est sauvegardée en flash
//!
//! ## Validité
//!
//! La calibration est généralement valide pour :
//! - Un seul vol (invalidée au désarmement)
//! - OU une durée limitée (ex: 1 heure) pour tenir compte des changements météo
//!
//! Pour des vols de longue durée, une recalibration peut être nécessaire.

use crate::types::{Result, ErreursAirHaum, Horodatage};
use crate::systeme::calibration::CalibrationPersistante;
//use std::time::Duration;

/// Calibration du baromètre
///
/// Stocke la pression de référence au niveau du sol pour le calcul d'altitude.
///
/// # Exemple
///
/// ```ignore
/// // Créer une calibration lors de la mesure au sol
/// let calib = CalibrationBarometre::nouvelle(
///     101325.0,  // Pression mesurée en Pa
///     3600       // Valide 1 heure
/// );
///
/// // Sauvegarder
/// systeme::calibration::gestionnaire().sauvegarder(&calib)?;
///
/// // Plus tard, au redémarrage...
/// if let Some(calib) = systeme::calibration::gestionnaire()
///     .charger::<CalibrationBarometre>()? 
/// {
///     let p_ref = calib.obtenir_pression_reference();
/// }
/// ```
#[derive(Debug, Clone, Copy)]
pub struct CalibrationBarometre {
    /// Moment où la calibration a été effectuée
    timestamp_unix_sec: u64,    

    /// Pression atmosphérique au niveau du sol en Pascals
    ///
    /// Utilisée comme référence (P₀) dans la formule barométrique :
    /// altitude = 44330 × (1 - (P/P₀)^0.1903)
    pression_reference_sol: f32,
    
    /// Durée de validité de la calibration en secondes
    ///
    /// Après ce délai, la calibration est considérée comme expirée
    /// et doit être refaite pour tenir compte des changements météo.
    validite_sec: u64,
}

impl CalibrationBarometre {
    /// Crée une nouvelle calibration avec la pression actuelle au sol
    ///
    /// # Arguments
    ///
    /// * `pression_sol` - Pression mesurée au sol en Pascals
    /// * `validite_sec` - Durée de validité en secondes
    ///
    /// # Recommandations de validité
    ///
    /// - Vol court (< 30 min) : 3600 s (1 heure)
    /// - Vol moyen : 1800 s (30 minutes)
    /// - Conditions météo instables : 600 s (10 minutes)
    ///
    /// # Exemple
    ///
    /// ```ignore
    /// let calib = CalibrationBarometre::nouvelle(
    ///     donnees.pression.pascals(),
    ///     3600  // Valide 1 heure
    /// );
    /// ```
    pub fn nouvelle(pression_sol: f32, validite_sec: u64) -> Self {
        Self {
            timestamp_unix_sec: std::time::SystemTime::now()
               .duration_since(std::time::UNIX_EPOCH)
               .unwrap_or_default()
               .as_secs(),
            pression_reference_sol: pression_sol,
            validite_sec,
        }
    }
    
    /// Obtient la pression de référence au sol
    ///
    /// Cette valeur doit être utilisée comme P₀ dans la formule barométrique
    /// pour calculer l'altitude relative.
    ///
    /// # Retour
    ///
    /// Pression en Pascals (Pa)
    pub fn obtenir_pression_reference(&self) -> f32 {
        self.pression_reference_sol
    }
    
    /// Obtient l'âge de la calibration en secondes
    ///
    /// Utile pour afficher depuis combien de temps la calibration a été faite.
    pub fn age_secondes_f32(&self) -> f32 {
    let maintenant = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap_or_default()
        .as_secs();
    maintenant.saturating_sub(self.timestamp_unix_sec) as f32
    }
}

// ============================================================================
// Implémentation du trait CalibrationPersistante
// ============================================================================

impl CalibrationPersistante for CalibrationBarometre {
    fn identifiant_capteur() -> &'static str {
        "barometre"
    }
    
    fn vers_toml(&self) -> String {
        format!(
            "# Calibration baromètre BMP280\n\
             # Générée automatiquement - ne pas éditer manuellement\n\
             \n\
             # Horodatage de la calibration (microsecondes depuis démarrage système)\n\
             timestamp_unix_sec = {}\n\
             \n\
             # Pression de référence au niveau du sol (Pascals)\n\
             # Utilisée comme P₀ dans le calcul d'altitude barométrique\n\
             pression_reference_sol = {:.2}\n\
             \n\
             # Durée de validité (secondes)\n\
             # Au-delà, la calibration est considérée comme expirée\n\
             validite_sec = {}\n",
            self.timestamp_unix_sec,
            self.pression_reference_sol,
            self.validite_sec
        )
    }
    
    fn depuis_toml(contenu: &str) -> Result<Self> {
  
        let mut timestamp_unix_sec = None;
        let mut pression_ref = None;
        let mut validite = None;
        
        // Parser le TOML ligne par ligne
        // Note: parser simple suffisant pour notre format contrôlé
        // Pour un format plus complexe, utiliser la crate 'toml'
        for ligne in contenu.lines() {
            let ligne = ligne.trim();
            
            // Ignorer les commentaires et lignes vides
            if ligne.starts_with('#') || ligne.is_empty() {
                continue;
            }
            
            // Parser les paires clé=valeur
            if let Some((cle, valeur)) = ligne.split_once('=') {
                let cle = cle.trim();
                let valeur = valeur.trim();
                
                match cle {
                   "timestamp_unix_sec" => {
                      timestamp_unix_sec = Some(valeur.trim().parse::<u64>()
                       .map_err(|_| ErreursAirHaum::CalibrationEchouee(
                            format!("Timestamp invalide: {}", valeur)
                                ))?
                        );
                                  
                    }
                    "pression_reference_sol" => {
                        pression_ref = Some(
                            valeur.parse::<f32>()
                                .map_err(|_| ErreursAirHaum::CalibrationEchouee(
                                    format!("Pression invalide: {}", valeur)
                                ))?
                        );
                    }
                    "validite_sec" => {
                        validite = Some(
                            valeur.parse::<u64>()
                                .map_err(|_| ErreursAirHaum::CalibrationEchouee(
                                    format!("Validité invalide: {}", valeur)
                                ))?
                        );
                    }
                    _ => {
                        // Ignorer les clés inconnues (compatibilité future)
                    }
                }
            }
        }
        
        // Vérifier que tous les champs obligatoires sont présents
        let timestamp = timestamp_unix_sec.ok_or_else(||
             ErreursAirHaum::CalibrationEchouee("Champ 'timestamp_unix_sec' manquant".into())
        )?;
        
        let pression = pression_ref.ok_or_else(|| 
            ErreursAirHaum::CalibrationEchouee(
                "Champ 'pression_reference_sol' manquant".into()
            )
        )?;
        
        let validite_sec = validite.ok_or_else(|| 
            ErreursAirHaum::CalibrationEchouee(
                "Champ 'validite_sec' manquant".into()
            )
        )?;
        
        // Valider les plages de valeurs
        if pression < 30000.0 || pression > 120000.0 {
            return Err(ErreursAirHaum::CalibrationEchouee(
                format!("Pression hors limites réalistes: {} Pa", pression)
            ));
        }
        
        if validite_sec == 0 || validite_sec > 86400 {
            return Err(ErreursAirHaum::CalibrationEchouee(
                format!("Validité hors limites: {} s", validite_sec)
            ));
        }
        
        Ok(Self {
            timestamp_unix_sec: timestamp,
            pression_reference_sol: pression,
            validite_sec,
        })
    }
    
    fn est_valide(&self) -> bool {
      self.age_secondes() < self.validite_sec
    }
    

    fn obtenir_horodatage(&self) -> Horodatage {
         Horodatage::maintenant() // non significatif pour la persistance
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
        let calib = CalibrationBarometre::nouvelle(101325.0, 3600);
        
        assert_eq!(calib.obtenir_pression_reference(), 101325.0);
        assert!(calib.est_valide());
    }
    
    #[test]
    fn test_serialisation_toml() {
        let calib = CalibrationBarometre::nouvelle(101325.0, 3600);
        let toml = calib.vers_toml();
        
        assert!(toml.contains("htimestamp_unix_sec"));
        assert!(toml.contains("pression_reference_sol = 101325.00"));
        assert!(toml.contains("validite_sec = 3600"));
    }
    
    #[test]
    fn test_deserialisation_toml() {
        let toml = "timestamp_unix_sec = 1700000000\n\
                    pression_reference_sol = 101325.50\n\
                    validite_sec = 3600\n";
        
        let calib = CalibrationBarometre::depuis_toml(toml).unwrap();
        
        assert!((calib.pression_reference_sol - 101325.50).abs() < 0.01);
        assert_eq!(calib.validite_sec, 3600);
    }
    
    #[test]
    fn test_roundtrip_serialisation() {
        let calib1 = CalibrationBarometre::nouvelle(101325.0, 3600);
        let toml = calib1.vers_toml();
        let calib2 = CalibrationBarometre::depuis_toml(&toml).unwrap();
        
        assert_eq!(calib1.timestamp_unix_sec, calib2.timestamp_unix_sec);
        assert!((calib1.pression_reference_sol - calib2.pression_reference_sol).abs() < 0.01);
        assert_eq!(calib1.validite_sec, calib2.validite_sec);
    }
    
    #[test]
    fn test_validation_pression_hors_limites() {
        let toml = "timestamp_unix_sec = 1700000000\n\
                    pression_reference_sol = 200000.0\n\
                    validite_sec = 3600\n";
        
        let resultat = CalibrationBarometre::depuis_toml(toml);
        assert!(resultat.is_err());
    }
    
    #[test]
    fn test_validation_champs_manquants() {
        let toml = "horodatage_micros = 1000000\n\
                    validite_sec = 3600\n";
        
        let resultat = CalibrationBarometre::depuis_toml(toml);
        assert!(resultat.is_err());
    }
    
    #[test]
    fn test_ignore_commentaires() {
        let toml = "# Ceci est un commentaire\n\
                    horodatage_micros = 1000000\n\
                    # Encore un commentaire\n\
                    pression_reference_sol = 101325.0\n\
                    validite_sec = 3600\n";
        
        let calib = CalibrationBarometre::depuis_toml(toml).unwrap();
        assert_eq!(calib.pression_reference_sol, 101325.0);
    }
}
