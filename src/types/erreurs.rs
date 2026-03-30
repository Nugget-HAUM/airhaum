// src/types/erreurs.rs
// Système d'erreurs personnalisé pour le projet

#![allow(dead_code)]
#![allow(unused_imports)]

use std::fmt;

/// Type Result personnalisé pour le projet
pub type Result<T> = std::result::Result<T,ErreursAirHaum>;

/// Énumération de toutes les erreurs possibles dans le projet
#[derive(Debug, Clone)]
pub enum ErreursAirHaum {
    // Erreurs Materiel/Communication
    ErreurI2C(String),
    ErreurUart(String),
    ErreurSpi(String),
    ErreurGpio(String),
    
    // Erreurs Capteurs
    CapteurNonInitialise(String),
    LectureCapteurEchouee(String),
    DonneesInvalides(String),
    CalibrationEchouee(String),
    TimeoutCapteur(String),
    HorsPortee,     
  
    ErreurInitialisation(String),     
    ErreurCommunication(String),
    
    // Erreurs GPS
    GpsFixPerdu,
    TrameMalformee(String),
    
    // Erreurs Estimation
    FiltreDivergent,
    DonneesInsuffisantes,
    
    // Erreurs Contrôle
    ConsigneHorsLimites(String),
    ActionneurDefaillant(String),
    
    // Erreurs Mission
    ModeVolInvalide { actuel: String, demande: String },
    MissionImpossible(String),
    ZoneInterdite,
    
    // Erreurs Communication
    LiaisonPerdue,
    ProtocoleInvalide(String),
    MessageCorrompu,
    
    // Erreurs Sûreté
    NiveauBatterieCritique,
    TemperatureHorsLimites { temperature: f32, limite: f32 },
    ArretUrgenceActive,
    SystemeNonArme,
    
    // Erreurs Configuration
    ConfigurationInvalide(String),
    FichierIntrouvable(String),
    ErreurIO(String),
    
    // Erreurs génériques
    Timeout,
    OperationNonSupportee(String),
    ErreurInterne(String),
}

impl fmt::Display for ErreursAirHaum {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match self {
            // Hardware
            ErreursAirHaum::ErreurI2C(msg) => write!(f, "Erreur I2C: {}", msg),
            ErreursAirHaum::ErreurUart(msg) => write!(f, "Erreur UART: {}", msg),
            ErreursAirHaum::ErreurSpi(msg) => write!(f, "Erreur SPI: {}", msg),
            ErreursAirHaum::ErreurGpio(msg) => write!(f, "Erreur GPIO: {}", msg),
            
            // Capteurs
            ErreursAirHaum::CapteurNonInitialise(nom) => {
                write!(f, "Capteur non initialisé: {}", nom)
            }
            ErreursAirHaum::LectureCapteurEchouee(nom) => {
                write!(f, "Échec de lecture du capteur: {}", nom)
            }
            ErreursAirHaum::DonneesInvalides(msg) => {
                write!(f, "Données invalides: {}", msg)
            }

            ErreursAirHaum::HorsPortee => {
                write!(f, "Mesure hors portée")
            }
            ErreursAirHaum::CalibrationEchouee(msg) => {
                write!(f, "Échec de calibration: {}", msg)
            }
            ErreursAirHaum::TimeoutCapteur(nom) => {
                write!(f, "Timeout lors de la lecture du capteur: {}", nom)
            }
            ErreursAirHaum::ErreurInitialisation(msg) => {
                write!(f, "Erreur initialisation: {}", msg)
            }
            ErreursAirHaum::ErreurCommunication(msg) => {
                write!(f, "Erreur communication: {}", msg)
            }           
 
            // GPS
            ErreursAirHaum::GpsFixPerdu => {
                write!(f, "Fix GPS perdu")
            }
            ErreursAirHaum::TrameMalformee(msg) => {
                write!(f, "Trame GPS malformée: {}", msg)
            }
            
            // Estimation
            ErreursAirHaum::FiltreDivergent => {
                write!(f, "Filtre de Kalman divergent")
            }
            ErreursAirHaum::DonneesInsuffisantes => {
                write!(f, "Données insuffisantes pour l'estimation")
            }
            
            // Contrôle
            ErreursAirHaum::ConsigneHorsLimites(msg) => {
                write!(f, "Consigne hors limites: {}", msg)
            }
            ErreursAirHaum::ActionneurDefaillant(nom) => {
                write!(f, "Actionneur défaillant: {}", nom)
            }
            
            // Mission
            ErreursAirHaum::ModeVolInvalide { actuel, demande } => {
                write!(f, "Transition de mode invalide: {} → {}", actuel, demande)
            }
            ErreursAirHaum::MissionImpossible(raison) => {
                write!(f, "Mission impossible: {}", raison)
            }
            ErreursAirHaum::ZoneInterdite => {
                write!(f, "Zone interdite (geofence)")
            }
            
            // Communication
            ErreursAirHaum::LiaisonPerdue => {
                write!(f, "Liaison radio perdue")
            }
            ErreursAirHaum::ProtocoleInvalide(msg) => {
                write!(f, "Protocole invalide: {}", msg)
            }
            ErreursAirHaum::MessageCorrompu => {
                write!(f, "Message corrompu")
            }
            
            // Sûreté
            ErreursAirHaum::NiveauBatterieCritique => {
                write!(f, "Niveau de batterie critique")
            }
            ErreursAirHaum::TemperatureHorsLimites { temperature, limite } => {
                write!(f, "Température hors limites: {}°C (limite: {}°C)", 
                       temperature, limite)
            }
            ErreursAirHaum::ArretUrgenceActive => {
                write!(f, "Arrêt d'urgence activé")
            }
            ErreursAirHaum::SystemeNonArme => {
                write!(f, "Système non armé")
            }
            
            // Configuration
            ErreursAirHaum::ConfigurationInvalide(msg) => {
                write!(f, "Configuration invalide: {}", msg)
            }
            ErreursAirHaum::FichierIntrouvable(fichier) => {
                write!(f, "Fichier introuvable: {}", fichier)
            }
            ErreursAirHaum::ErreurIO(msg) => {
                write!(f, "Erreur I/O: {}", msg)
            }
           
            // Génériques
            ErreursAirHaum::Timeout => {
                write!(f, "Timeout")
            }
            ErreursAirHaum::OperationNonSupportee(msg) => {
                write!(f, "Opération non supportée: {}", msg)
            }
            ErreursAirHaum::ErreurInterne(msg) => {
                write!(f, "Erreur interne: {}", msg)
            }
        }
    }
}

impl std::error::Error for ErreursAirHaum {}

// Conversions depuis d'autres types d'erreurs communs
impl From<std::io::Error> for ErreursAirHaum {
    fn from(err: std::io::Error) -> Self {
        ErreursAirHaum::ErreurInterne(format!("Erreur I/O: {}", err))
    }
}

impl From<std::fmt::Error> for ErreursAirHaum {
    fn from(err: std::fmt::Error) -> Self {
        ErreursAirHaum::ErreurInterne(format!("Erreur de formatage: {}", err))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_affichage_erreur() {
        let err = ErreursAirHaum::CapteurNonInitialise("MPU9265".to_string());
        assert_eq!(format!("{}", err), "Capteur non initialisé: MPU9265");
    }

    #[test]
    fn test_type_restultat() {
        fn fonction_test() -> Result<i32> {
            Ok(42)
        }
        assert_eq!(fonction_test().unwrap(), 42);
    }

    #[test]
    fn test_temperature_hors_limites() {
        let err = ErreursAirHaum::TemperatureHorsLimites {
            temperature: 85.0,
            limite: 80.0,
        };
        let msg = format!("{}", err);
        assert!(msg.contains("85"));
        assert!(msg.contains("80"));
    }
}
