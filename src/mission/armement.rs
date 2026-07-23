// src/mission/armement.rs
//! Séquence de vérification pré-vol : activation et validation des servos.
//!
//! Logique pure — la séquence interactive elle-même (affichage, envoi des
//! consignes, saisie opérateur) reste dans `bin/airhaum_test/console_test.rs`.
//! Ce module ne fait que représenter et faire progresser l'état de la
//! séquence, pour que la check-liste d'armement puisse vérifier qu'elle a
//! réellement été exécutée cette session plutôt que de se fier à une simple
//! attestation opérateur non vérifiée.

/// Surface de commande testée individuellement lors de la séquence pré-vol.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SurfaceServo {
    Ailerons,
    Profondeur,
    Direction,
}

const NOMBRE_SURFACES: usize = 3;

impl SurfaceServo {
    fn indice(self) -> usize {
        match self {
            SurfaceServo::Ailerons => 0,
            SurfaceServo::Profondeur => 1,
            SurfaceServo::Direction => 2,
        }
    }
}

/// État de la séquence de vérification pré-vol pour la session en cours.
///
/// Une surface n'est marquée testée qu'après un aller-retour complet
/// minimum → maximum → neutre confirmé par l'opérateur (voir
/// `console_test::tester_debattements_servos`) — une interruption (`q`) ne
/// la marque pas.
#[derive(Debug, Default)]
pub struct SequencePrevol {
    testees: [bool; NOMBRE_SURFACES],
}

impl SequencePrevol {
    pub fn nouvelle() -> Self {
        Self { testees: [false; NOMBRE_SURFACES] }
    }

    /// Marque une surface comme testée cette session.
    pub fn marquer_testee(&mut self, surface: SurfaceServo) {
        self.testees[surface.indice()] = true;
    }

    /// Vrai si les trois surfaces ont été testées cette session.
    pub fn complete(&self) -> bool {
        self.testees.iter().all(|&t| t)
    }

    /// Nombre de surfaces déjà testées, pour affichage de progression.
    pub fn nombre_testees(&self) -> usize {
        self.testees.iter().filter(|&&t| t).count()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_incomplete_au_depart() {
        let seq = SequencePrevol::nouvelle();
        assert!(!seq.complete());
        assert_eq!(seq.nombre_testees(), 0);
    }

    #[test]
    fn test_complete_apres_les_trois_surfaces() {
        let mut seq = SequencePrevol::nouvelle();
        seq.marquer_testee(SurfaceServo::Ailerons);
        assert!(!seq.complete());
        seq.marquer_testee(SurfaceServo::Profondeur);
        assert!(!seq.complete());
        seq.marquer_testee(SurfaceServo::Direction);
        assert!(seq.complete());
        assert_eq!(seq.nombre_testees(), 3);
    }

    #[test]
    fn test_marquage_redondant_sans_effet() {
        let mut seq = SequencePrevol::nouvelle();
        seq.marquer_testee(SurfaceServo::Ailerons);
        seq.marquer_testee(SurfaceServo::Ailerons);
        assert_eq!(seq.nombre_testees(), 1);
    }
}
