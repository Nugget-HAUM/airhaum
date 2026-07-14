// src/interfaces/gps.rs
//! Trait d'interface pour les récepteurs GPS.

use crate::types::{DonneesGps, Result};

/// Interface commune pour tout récepteur GPS.
///
/// Le GPS est un capteur **streaming** : contrairement aux capteurs I²C où l'on
/// demande une mesure, le récepteur envoie des trames en continu sur l'UART.
/// L'interface reflète cette nature asynchrone :
///
/// - [`mettre_a_jour`] consomme les octets disponibles sans bloquer
/// - [`derniere_donnee`] expose la dernière trame complète reçue
///
/// [`mettre_a_jour`]: CapteurGps::mettre_a_jour
/// [`derniere_donnee`]: CapteurGps::derniere_donnee
pub trait CapteurGps {
    /// Initialise le récepteur (vide le buffer d'entrée, vérifie la communication).
    ///
    /// À appeler une seule fois au démarrage, avant la boucle de mise à jour.
    fn initialiser(&mut self) -> Result<()>;

    /// Lit les octets disponibles sur l'UART et met à jour l'état interne.
    ///
    /// **Non-bloquant** : retourne immédiatement s'il n'y a pas de données.
    /// Retourne `true` si au moins une trame NAV-PVT complète et valide a été
    /// parsée lors de cet appel (nouvelle position disponible).
    fn mettre_a_jour(&mut self) -> bool;

    /// Retourne la dernière position complète disponible.
    ///
    /// `None` si aucune trame valide n'a encore été reçue depuis le démarrage
    /// ou depuis la dernière réinitialisation.
    fn derniere_donnee(&self) -> Option<DonneesGps>;

    /// Vrai si le récepteur a produit au moins une position avec fix >= 2D.
    fn est_operationnel(&self) -> bool;
}

/// Assistance à l'acquisition (AssistNow Autonomous + aide position), propre
/// aux récepteurs u-blox — séparée de [`CapteurGps`] pour ne pas imposer cette
/// capacité à un futur driver GPS non-UBX. Voir doc/assistance_gps.md.
pub trait AssistanceGnss {
    /// Interroge le récepteur (orbites prédites) et associe la dernière
    /// position connue, pour sauvegarde persistante.
    fn exporter_assistance(&mut self) -> Result<crate::drivers::gps::AssistanceGps>;

    /// Réinjecte une assistance précédemment sauvegardée (position + orbites).
    /// À appeler avant le démarrage de l'acquisition satellite.
    fn importer_assistance(&mut self, assistance: &crate::drivers::gps::AssistanceGps) -> Result<()>;
}
