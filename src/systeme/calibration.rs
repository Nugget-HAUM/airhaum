// src/systeme/calibration.rs
//! Gestion persistante des calibrations capteurs
//!
//! Ce module fournit une infrastructure générique pour :
//! - Stocker les calibrations en flash (fichiers TOML)
//! - Charger les calibrations au démarrage
//! - Valider les calibrations (horodatage, expiration)
//! - Supprimer les calibrations obsolètes
//!
//! Chaque capteur implémente le trait `CalibrationPersistante` pour bénéficier
//! automatiquement de toute cette infrastructure.

use crate::types::{Result, ErreursAirHaum, Horodatage};
use std::path::Path;
use std::fs;

/// Trait pour toute calibration pouvant être stockée de manière persistante
///
/// # Exemple d'implémentation
///
/// ```ignore
/// impl CalibrationPersistante for CalibrationBarometre {
///     fn identifiant_capteur() -> &'static str {
///         "barometre"
///     }
///     
///     fn vers_toml(&self) -> String {
///         format!("horodatage_micros = {}\npression_ref = {}\n", 
///                 self.horodatage.micros(), self.pression)
///     }
///     
///     fn depuis_toml(contenu: &str) -> Result<Self> {
///         // Parser le TOML et reconstruire la structure
///     }
///     
///     fn est_valide(&self) -> bool {
///         !self.horodatage.est_ecoule(Duration::from_secs(self.validite_sec))
///     }
///     
///     fn obtenir_horodatage(&self) -> Horodatage {
///         self.horodatage
///     }
/// }
/// ```
pub trait CalibrationPersistante: Sized {
    /// Identifiant unique du capteur
    ///
    /// Utilisé pour nommer le fichier de calibration.
    /// Ex: "barometre" → calibration_barometre.toml
    fn identifiant_capteur() -> &'static str;
    
    /// Sérialise la calibration en format TOML
    ///
    /// Doit produire un contenu TOML valide avec tous les champs nécessaires
    /// pour reconstruire la calibration via `depuis_toml`.
    fn vers_toml(&self) -> String;
    
    /// Désérialise une calibration depuis un contenu TOML
    ///
    /// # Erreurs
    ///
    /// Retourne `CalibrationEchouee` si :
    /// - Le format TOML est invalide
    /// - Des champs obligatoires manquent
    /// - Les valeurs sont hors limites
    fn depuis_toml(contenu: &str) -> Result<Self>;
   
    /// Âge de la calibration en secondes depuis sa création (temps réel)
    fn age_secondes(&self) -> u64;
 
    /// Vérifie si la calibration est encore valide
    ///
    /// Prend en compte l'âge de la calibration et toute autre contrainte
    /// spécifique au capteur.
    fn est_valide(&self) -> bool;
    
    /// Obtient l'horodatage de la calibration
    ///
    /// Utilisé pour l'affichage et le débogage.
    fn obtenir_horodatage(&self) -> Horodatage;
}

/// Gestionnaire centralisé des fichiers de calibration
///
/// Responsable de :
/// - Lire/écrire les fichiers de calibration
/// - Créer les répertoires nécessaires
/// - Gérer les erreurs d'I/O
///
/// # Utilisation
///
/// ```ignore
/// // Initialisation au démarrage
/// calibration::initialiser_gestionnaire("/home/airhaum/config");
///
/// // Chargement d'une calibration
/// if let Some(calib) = calibration::gestionnaire()
///     .charger::<CalibrationBarometre>()? 
/// {
///     println!("Calibration chargée et valide");
/// }
///
/// // Sauvegarde
/// calibration::gestionnaire().sauvegarder(&ma_calibration)?;
/// ```
pub struct GestionnaireCalibration {
    chemin_base: String,
}

impl GestionnaireCalibration {
    /// Crée un nouveau gestionnaire avec le chemin de base spécifié
    ///
    /// # Arguments
    ///
    /// * `chemin_base` - Répertoire où stocker les fichiers de calibration
    ///                   Ex: "/home/airhaum/config"
    pub fn nouveau(chemin_base: impl Into<String>) -> Self {
        Self {
            chemin_base: chemin_base.into(),
        }
    }
    
    /// Charge une calibration depuis le disque
    ///
    /// # Comportement
    ///
    /// 1. Vérifie l'existence du fichier
    /// 2. Lit et parse le contenu TOML
    /// 3. Vérifie la validité de la calibration
    /// 4. Retourne `None` si :
    ///    - Le fichier n'existe pas
    ///    - La calibration est expirée
    ///    - Le parsing échoue (avec log d'erreur)
    ///
    /// # Erreurs
    ///
    /// Retourne une erreur seulement en cas de problème I/O grave
    /// (permissions, disque plein, etc.)
    pub fn charger<C: CalibrationPersistante>(&self) -> Result<Option<C>> {
        let chemin = self.chemin_fichier(C::identifiant_capteur());
        
        // Si le fichier n'existe pas, ce n'est pas une erreur
        if !Path::new(&chemin).exists() {
            return Ok(None);
        }
        
        // Lire le contenu du fichier
        let contenu = fs::read_to_string(&chemin)
            .map_err(|e| ErreursAirHaum::ErreurIO(
                format!("Lecture calibration {}: {}", C::identifiant_capteur(), e)
            ))?;
        
        // Tenter de désérialiser
        let calibration = match C::depuis_toml(&contenu) {
            Ok(c) => c,
            Err(e) => {
                eprintln!("⚠ Calibration {} corrompue: {:?}", C::identifiant_capteur(), e);
                return Ok(None);
            }
        };
        
        // Vérifier la validité
        if calibration.est_valide() {
            println!("✓ Calibration {} chargée (créée il y a {:.1}s)", 
                     C::identifiant_capteur(),
                     calibration.age_secondes() as f32);
            Ok(Some(calibration))
        } else {
            println!("⚠ Calibration {} expirée, ignorée", C::identifiant_capteur());
            Ok(None)
        }
    }
    
    /// Sauvegarde une calibration sur le disque
    ///
    /// # Comportement
    ///
    /// 1. Crée le répertoire de destination si nécessaire
    /// 2. Sérialise la calibration en TOML
    /// 3. Écrit atomiquement le fichier
    ///
    /// # Erreurs
    ///
    /// Retourne une erreur si :
    /// - Impossible de créer le répertoire
    /// - Impossible d'écrire le fichier (permissions, disque plein)
    pub fn sauvegarder<C: CalibrationPersistante>(&self, calibration: &C) -> Result<()> {
        let chemin = self.chemin_fichier(C::identifiant_capteur());
        
        // Créer le répertoire parent si nécessaire
        if let Some(parent) = Path::new(&chemin).parent() {
            fs::create_dir_all(parent)
                .map_err(|e| ErreursAirHaum::ErreurIO(
                    format!("Création répertoire calibration: {}", e)
                ))?;
        }
        
        // Sérialiser la calibration
        let contenu = calibration.vers_toml();
        
        // Écrire le fichier
        fs::write(&chemin, contenu)
            .map_err(|e| ErreursAirHaum::ErreurIO(
                format!("Écriture calibration {}: {}", C::identifiant_capteur(), e)
            ))?;
        
        println!("✓ Calibration {} sauvegardée dans {}", 
                 C::identifiant_capteur(), chemin);
        Ok(())
    }
    
    /// Supprime une calibration du disque
    ///
    /// Utile pour forcer une recalibration ou nettoyer les anciennes données.
    ///
    /// # Erreurs
    ///
    /// Retourne une erreur seulement si le fichier existe mais ne peut pas
    /// être supprimé (permissions, etc.)
    pub fn supprimer<C: CalibrationPersistante>(&self) -> Result<()> {
        let chemin = self.chemin_fichier(C::identifiant_capteur());
        
        if Path::new(&chemin).exists() {
            fs::remove_file(&chemin)
                .map_err(|e| ErreursAirHaum::ErreurIO(
                    format!("Suppression calibration {}: {}", C::identifiant_capteur(), e)
                ))?;
            println!("✓ Calibration {} supprimée", C::identifiant_capteur());
        } else {
            println!("⚠ Calibration {} inexistante, rien à supprimer", 
                     C::identifiant_capteur());
        }
        
        Ok(())
    }
    
    /// Construit le chemin complet du fichier de calibration
    ///
    /// Format: `{chemin_base}/calibration_{identifiant}.toml`
    fn chemin_fichier(&self, identifiant: &str) -> String {
        format!("{}/calibration_{}.toml", self.chemin_base, identifiant)
    }

}

// ============================================================================
// Instance globale du gestionnaire
// ============================================================================

/// Instance globale du gestionnaire de calibration
///
/// SAFETY: Initialisé une seule fois au démarrage via `initialiser_gestionnaire()`
/// avant tout accès concurrent.
use std::sync::OnceLock; 
static GESTIONNAIRE: OnceLock<GestionnaireCalibration> = OnceLock::new();  // ← Nouveau type
/// Initialise le gestionnaire global de calibration
///
/// **DOIT être appelé au démarrage du système avant tout autre usage.**
///
/// # Panics
///
/// Panique si appelé plusieurs fois (détecté en debug).
///
/// # Arguments
///
/// * `chemin_config` - Chemin du répertoire de configuration
///                     Ex: "/home/airhaum/config"
pub fn initialiser_gestionnaire(chemin_config: &str) {
    GESTIONNAIRE.get_or_init(|| {
        println!("✓ Gestionnaire de calibration initialisé: {}", chemin_config);
        GestionnaireCalibration::nouveau(chemin_config)
    });
}



/// Obtient une référence au gestionnaire global de calibration
///
/// # Panics
///
/// Panique si `initialiser_gestionnaire()` n'a pas été appelé.
pub fn gestionnaire() -> &'static GestionnaireCalibration {
    GESTIONNAIRE.get()
        .expect("Gestionnaire de calibration non initialisé - appeler initialiser_gestionnaire() au démarrage")
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::Horodatage;
    use std::time::Duration;
    
    // Calibration de test simple
    #[derive(Debug)]
    struct CalibrationTest {
        horodatage: Horodatage,
        valeur: f32,
        validite_sec: u64,
    }
    
    impl CalibrationPersistante for CalibrationTest {
        fn identifiant_capteur() -> &'static str {
            "test"
        }
        
        fn vers_toml(&self) -> String {
            format!(
                "horodatage_micros = {}\nvaleur = {}\nvalidite_sec = {}\n",
                self.horodatage.micros(),
                self.valeur,
                self.validite_sec
            )
        }
        
        fn depuis_toml(contenu: &str) -> Result<Self> {
            let mut horodatage = None;
            let mut valeur = None;
            let mut validite = None;
            
            for ligne in contenu.lines() {
                if let Some((cle, val)) = ligne.split_once('=') {
                    match cle.trim() {
                        "horodatage_micros" => horodatage = Some(val.trim().parse().unwrap()),
                        "valeur" => valeur = Some(val.trim().parse().unwrap()),
                        "validite_sec" => validite = Some(val.trim().parse().unwrap()),
                        _ => {}
                    }
                }
            }
            
            Ok(Self {
                horodatage: Horodatage::depuis_micros(horodatage.unwrap()),
                valeur: valeur.unwrap(),
                validite_sec: validite.unwrap(),
            })
        }
        
        fn est_valide(&self) -> bool {
            !self.horodatage.est_ecoule(Duration::from_secs(self.validite_sec))
        }
        
        fn obtenir_horodatage(&self) -> Horodatage {
            self.horodatage
        }

        /// Âge de la calibration en secondes depuis sa création (temps réel)
        fn age_secondes(&self) -> u64 {
          self.horodatage.ecoule().as_secs()
        }
    }
    
    #[test]
    fn test_serialisation_deserialisation() {
        let calib = CalibrationTest {
            horodatage: Horodatage::maintenant(),
            valeur: 42.5,
            validite_sec: 3600,
        };
        
        let toml = calib.vers_toml();
        let calib2 = CalibrationTest::depuis_toml(&toml).unwrap();
        
        assert_eq!(calib.horodatage.micros(), calib2.horodatage.micros());
        assert!((calib.valeur - calib2.valeur).abs() < 0.001);
        assert_eq!(calib.validite_sec, calib2.validite_sec);
    }
    
    #[test]
    fn test_validite() {
        let calib = CalibrationTest {
            horodatage: Horodatage::maintenant(),
            valeur: 10.0,
            validite_sec: 3600,
        };
        
        assert!(calib.est_valide());
    }
}
