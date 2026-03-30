// src/interfaces/telemetre.rs
//! Interface pour les télémètres (capteurs de distance)

use crate::types::Result;
use crate::types::EtatCapteur;

/// Trait pour les capteurs de distance (télémètres)
pub trait Telemetre {
    /// Initialise le capteur
    fn initialiser(&mut self) -> Result<()>;
    
    /// Effectue une mesure de distance
    /// 
    /// # Retour
    /// * Distance en millimètres
    fn mesurer_distance(&mut self) -> Result<u16>;
    
    /// Vérifie si le capteur est prêt
    fn est_pret(&mut self) -> Result<bool>;
    
    /// Obtient la précision estimée de la mesure
    /// 
    /// # Retour
    /// * Précision en millimètres (±)
    fn obtenir_precision(&self) -> u16;
    
    /// Obtient la portée maximale du capteur
    /// 
    /// # Retour
    /// * Portée maximale en millimètres
    fn obtenir_portee_max(&self) -> u16;

    /// Obtient l'état actuel du capteur
    fn obtenir_etat(&self) -> &EtatCapteur;

    /// Vérifie si le capteur est opérationnel
    fn est_operationnel(&self) -> bool {
       self.obtenir_etat().est_utilisable()
    }
}
