// src/surete/armement.rs
//! Condition de sûreté pour l'armement : "Sûreté nominale".
//!
//! Le reste de la check-liste d'armement (calibrations, estimation active,
//! liaison Arduino/RC — voir `doc/mission.md`) est actuellement évalué
//! directement dans `bin/airhaum_test/console_armement.rs`, faute d'un
//! besoin partagé au-delà de la console. Ce module couvre la partie qui
//! dépend de `surete/` : il sera le point d'extension si le reste de la
//! check-liste doit un jour être factorisé hors de la console (ex. pour un
//! futur `airhaum-vol` sans interactivité).

use crate::surete::EtatSecurite;

/// Vrai si l'état sécurité courant permet l'armement.
///
/// `EtatSecurite::est_nominal()` couvre déjà `Normal` et `AlerteMineure` —
/// y compris l'`AlerteMineure` résiduelle laissée par
/// `surete::urgences::MaeSecurite` après un incident survenu plus tôt dans
/// la session (plafond volontaire, voir sa documentation). Toute alerte
/// majeure ou état critique bloque l'armement.
pub fn surete_nominale(etat: &EtatSecurite) -> bool {
    etat.est_nominal()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_normal_autorise_armement() {
        assert!(surete_nominale(&EtatSecurite::Normal));
    }

    #[test]
    fn test_alerte_mineure_autorise_armement() {
        assert!(surete_nominale(&EtatSecurite::AlerteMineure { raison: "test".into() }));
    }

    #[test]
    fn test_alerte_majeure_bloque_armement() {
        assert!(!surete_nominale(&EtatSecurite::AlerteMajeure { raison: "test".into() }));
    }

    #[test]
    fn test_etat_critique_bloque_armement() {
        assert!(!surete_nominale(&EtatSecurite::FailSafe));
    }
}
