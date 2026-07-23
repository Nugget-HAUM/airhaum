// src/surete/mod.rs
//! Module sûreté — surveillance système et transitions d'urgence.
//!
//! La MAÉ sécurité est **orthogonale et prioritaire** sur la MAÉ vol.
//! Elle publie son état via un `watch::Sender<EtatSecurite>` que la MAÉ vol
//! consulte à chaque itération.
//!
//! Communication **unidirectionnelle** : sécurité → vol uniquement.
//! La MAÉ vol ne commande jamais la MAÉ sécurité.
//!
//! Implémentation de la boucle : `taches/taches_surete.rs`.

pub mod armement;
pub mod limites;
pub mod sante;
pub mod urgences;

use std::fmt;

/// État courant de la MAÉ sécurité.
///
/// Les états sont ordonnés par criticité croissante. Un état **critique**
/// (`est_critique()`) ne se résout jamais instantanément — voir
/// `surete::urgences::MaeSecurite`, qui applique une hystérésis (plusieurs
/// cycles sains consécutifs requis) avant de lever le verrou. De plus, si un
/// incident sérieux (`AlerteMajeure` ou pire) a été détecté à un moment du
/// vol, le rétablissement plafonne à `AlerteMineure` — jamais un retour
/// direct à `Normal` — pour signaler durablement qu'il faut écourter le vol
/// plutôt que reprendre la mission comme si de rien n'était.
#[derive(Debug, Clone, PartialEq, Default)]
pub enum EtatSecurite {
    /// Tous les paramètres dans les limites nominales.
    #[default]
    Normal,

    /// Anomalie détectée, vol maintenu, opérateur notifié.
    AlerteMineure { raison: String },

    /// Anomalie sérieuse, comportement vol restreint (pas de nouvelles phases).
    AlerteMajeure { raison: String },

    /// Liaison perdue ou condition dégradée : atterrissage au plus tôt.
    /// La MAÉ vol bascule en loiter puis amorce une approche.
    FailSafe,

    /// Situation critique : atterrissage immédiat, priorité absolue.
    AtterrissageUrgence { raison: String },

    /// Situation mortelle : coupure immédiate du moteur pour éviter un accident
    /// (ex. emballement, perte de contrôle au sol), puis planeur forcé vers
    /// `AtterrissageUrgence`.
    ArretUrgence { raison: String },
}

impl EtatSecurite {
    /// Vrai si la MAÉ vol doit abandonner son plan nominal immédiatement.
    pub fn est_critique(&self) -> bool {
        matches!(
            self,
            EtatSecurite::FailSafe
                | EtatSecurite::AtterrissageUrgence { .. }
                | EtatSecurite::ArretUrgence { .. }
        )
    }

    /// Vrai si le vol autonome nominal peut continuer.
    pub fn est_nominal(&self) -> bool {
        matches!(
            self,
            EtatSecurite::Normal | EtatSecurite::AlerteMineure { .. }
        )
    }
}

impl fmt::Display for EtatSecurite {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            EtatSecurite::Normal =>
                write!(f, "Normal"),
            EtatSecurite::AlerteMineure { raison } =>
                write!(f, "AlerteMineure: {}", raison),
            EtatSecurite::AlerteMajeure { raison } =>
                write!(f, "AlerteMajeure: {}", raison),
            EtatSecurite::FailSafe =>
                write!(f, "FailSafe"),
            EtatSecurite::AtterrissageUrgence { raison } =>
                write!(f, "AtterrissageUrgence: {}", raison),
            EtatSecurite::ArretUrgence { raison } =>
                write!(f, "ArretUrgence: {}", raison),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_etat_nominal() {
        assert!(EtatSecurite::Normal.est_nominal());
        assert!(EtatSecurite::AlerteMineure { raison: "test".into() }.est_nominal());
        assert!(!EtatSecurite::FailSafe.est_nominal());
    }

    #[test]
    fn test_etat_critique() {
        assert!(!EtatSecurite::Normal.est_critique());
        assert!(!EtatSecurite::AlerteMajeure { raison: "test".into() }.est_critique());
        assert!(EtatSecurite::FailSafe.est_critique());
        assert!(EtatSecurite::AtterrissageUrgence { raison: "batterie".into() }.est_critique());
        assert!(EtatSecurite::ArretUrgence { raison: "emballement".into() }.est_critique());
    }
}
