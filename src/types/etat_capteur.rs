// src/types/etat_capteur.rs
//! Machine à états pour les capteurs

use std::fmt;
use super::Horodatage;

/// État d'un capteur dans le système
#[derive(Debug, Clone, PartialEq)]
pub enum EtatCapteur {
    /// État par défaut au démarrage ou après perte de contrôle
    Inconnu,
    
    /// Capteur présent mais non configuré
    NonConfigure,
    
    /// Registres valides, capteur actif, données non encore garanties
    Configure,
    
    /// Capteur calibré, données cohérentes et utilisables
    Operationnel {
        depuis: Horodatage,
    },
    
    /// Capteur fonctionnel mais données partielles ou douteuses
    Degrade {
        raison: String,
        depuis: Horodatage,
    },
}

impl EtatCapteur {
    /// Vérifie si le capteur est utilisable pour la navigation
    pub fn est_utilisable(&self) -> bool {
        matches!(self, EtatCapteur::Operationnel { .. })
    }
    
    /// Vérifie si le capteur nécessite une réinitialisation
    pub fn necessite_reinitialisation(&self) -> bool {
        matches!(
            self,
            EtatCapteur::Inconnu | EtatCapteur::Degrade { .. }
        )
    }
    
    /// Transitions autorisées depuis l'état actuel
    pub fn peut_transitionner_vers(&self, nouvel_etat: &EtatCapteur) -> bool {
        use EtatCapteur::*;
        
        match (self, nouvel_etat) {
            // Depuis Inconnu : peut aller vers NonConfiguré ou rester Inconnu
            (Inconnu, NonConfigure) => true,
            (Inconnu, Inconnu) => true,
            
            // Depuis NonConfiguré : peut configurer ou échouer
            (NonConfigure, Configure) => true,
            (NonConfigure, Inconnu) => true,
            
            // Depuis Configuré : peut devenir opérationnel ou échouer
            (Configure, Operationnel { .. }) => true,
            (Configure, Inconnu) => true,
            
            // Depuis Opérationnel : peut se dégrader ou échouer
            (Operationnel { .. }, Degrade { .. }) => true,
            (Operationnel { .. }, Inconnu) => true,
            (Operationnel { .. }, Operationnel { .. }) => true, // Mise à jour timestamp
            
            // Depuis Dégradé : retour à Inconnu pour réinit
            (Degrade { .. }, Inconnu) => true,
            
            // Toutes autres transitions sont interdites
            _ => false,
        }
    }
    
    /// Crée un nouvel état Opérationnel avec timestamp actuel
    pub fn nouveau_operationnel() -> Self {
        EtatCapteur::Operationnel {
            depuis: Horodatage::maintenant(),
        }
    }
    
    /// Crée un nouvel état Dégradé avec raison
    pub fn nouveau_degrade(raison: impl Into<String>) -> Self {
        EtatCapteur::Degrade {
            raison: raison.into(),
            depuis: Horodatage::maintenant(),
        }
    }
}

impl fmt::Display for EtatCapteur {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            EtatCapteur::Inconnu => write!(f, "Inconnu"),
            EtatCapteur::NonConfigure => write!(f, "NonConfiguré"),
            EtatCapteur::Configure => write!(f, "Configuré"),
            EtatCapteur::Operationnel { depuis } => {
                write!(f, "Opérationnel (depuis {}s)", depuis.seconds())
            }
            EtatCapteur::Degrade { raison, depuis } => {
                write!(f, "Dégradé: {} (depuis {}s)", raison, depuis.seconds())
            }
        }
    }
}

impl Default for EtatCapteur {
    fn default() -> Self {
        EtatCapteur::Inconnu
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_transitions_valides() {
        let inconnu = EtatCapteur::Inconnu;
        let non_config = EtatCapteur::NonConfigure;
        
        assert!(inconnu.peut_transitionner_vers(&non_config));
        assert!(non_config.peut_transitionner_vers(&EtatCapteur::Configure));
    }
    
    #[test]
    fn test_transitions_invalides() {
        let inconnu = EtatCapteur::Inconnu;
        let operationnel = EtatCapteur::nouveau_operationnel();
        
        // On ne peut pas passer directement d'Inconnu à Opérationnel
        assert!(!inconnu.peut_transitionner_vers(&operationnel));
    }
    
    #[test]
    fn test_est_utilisable() {
        assert!(!EtatCapteur::Inconnu.est_utilisable());
        assert!(!EtatCapteur::NonConfigure.est_utilisable());
        assert!(!EtatCapteur::Configure.est_utilisable());
        assert!(EtatCapteur::nouveau_operationnel().est_utilisable());
        assert!(!EtatCapteur::nouveau_degrade("test").est_utilisable());
    }
}
