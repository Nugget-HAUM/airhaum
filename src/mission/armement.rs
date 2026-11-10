// src/mission/armement.rs
//! Séquence de vérification pré-vol : activation et validation des servos.
//!
//! Logique pure — la séquence interactive elle-même (affichage, envoi des
//! consignes, saisie opérateur) reste dans `bin/airhaum_test/console_test.rs`.
//! Ce module ne fait que représenter et faire progresser l'état de la
//! séquence, pour que la check-liste d'armement puisse vérifier qu'elle a
//! réellement été exécutée cette session plutôt que de se fier à une simple
//! attestation opérateur non vérifiée.
//!
//! # Sens attendus
//!
//! Une commande positive doit augmenter l'angle correspondant, convention de
//! l'estimation (`doc/calibration.md`) — roulis à droite, nez haut, nez à
//! droite. C'est ce que supposent les contrôleurs d'axe : un sens inversé ici
//! ferait diverger l'autopilote. Les étapes passent par le mixeur, comme les
//! consignes de l'autopilote : le test porte sur la chaîne réellement utilisée.
//! Voir `doc/controle.md`, « Sens des commandes ».

use crate::controle::{mixer, CommandesNormalisees, ConsigneGaz};
use crate::drivers::controleur_servo::ConsignesServos;

/// Surface de commande testée individuellement lors de la séquence pré-vol.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SurfaceServo {
    Ailerons,
    Profondeur,
    Direction,
}

const NOMBRE_SURFACES: usize = 3;

impl SurfaceServo {
    /// Toutes les surfaces, dans l'ordre de la séquence.
    pub const TOUTES: [SurfaceServo; NOMBRE_SURFACES] =
        [SurfaceServo::Ailerons, SurfaceServo::Profondeur, SurfaceServo::Direction];

    fn indice(self) -> usize {
        match self {
            SurfaceServo::Ailerons => 0,
            SurfaceServo::Profondeur => 1,
            SurfaceServo::Direction => 2,
        }
    }

    /// Nom affiché à l'opérateur.
    pub fn nom(self) -> &'static str {
        match self {
            SurfaceServo::Ailerons => "AILERONS",
            SurfaceServo::Profondeur => "PROFONDEUR",
            SurfaceServo::Direction => "DIRECTION",
        }
    }

    /// Étapes de la surface : un côté, l'autre côté, puis le neutre.
    /// Formulations vues de l'arrière de l'avion.
    pub fn etapes(self) -> [EtapePrevol; 3] {
        let (positif, negatif) = match self {
            SurfaceServo::Ailerons => (
                ("ROULIS À DROITE", "aileron droit LEVÉ, aileron gauche BAISSÉ"),
                ("ROULIS À GAUCHE", "aileron gauche LEVÉ, aileron droit BAISSÉ"),
            ),
            SurfaceServo::Profondeur => (
                ("CABRER", "gouverne LEVÉE"),
                ("PIQUER", "gouverne BAISSÉE"),
            ),
            SurfaceServo::Direction => (
                ("NEZ À DROITE", "gouverne à DROITE"),
                ("NEZ À GAUCHE", "gouverne à GAUCHE"),
            ),
        };
        let etape = |(commande, attendu), valeur| EtapePrevol { surface: self, commande, attendu, valeur };
        [
            etape(positif, 1.0),
            etape(negatif, -1.0),
            etape(("NEUTRE", "gouverne alignée"), 0.0),
        ]
    }
}

/// Une position à confirmer visuellement par l'opérateur.
#[derive(Debug, Clone, Copy)]
pub struct EtapePrevol {
    pub surface: SurfaceServo,
    pub commande: &'static str,
    pub attendu: &'static str,
    /// Commande normalisée de la surface testée, [-1, 1].
    pub valeur: f32,
}

impl EtapePrevol {
    /// Consignes de l'étape : seule la surface testée bouge, jamais armé,
    /// gaz au minimum.
    pub fn consignes(&self) -> ConsignesServos {
        let mut commandes = CommandesNormalisees::default();
        match self.surface {
            SurfaceServo::Ailerons => commandes.ailerons = self.valeur,
            SurfaceServo::Profondeur => commandes.profondeur = self.valeur,
            SurfaceServo::Direction => commandes.direction = self.valeur,
        }
        mixer(&commandes, &ConsigneGaz::default(), false)
    }
}

/// État de la séquence de vérification pré-vol pour la session en cours.
///
/// Une surface n'est marquée testée qu'après confirmation explicite par
/// l'opérateur de toutes ses étapes (voir
/// `console_test::tester_debattements_servos`) — une interruption ou un écart
/// constaté ne la marque pas.
#[derive(Debug, Default)]
pub struct SequencePrevol {
    testees: [bool; NOMBRE_SURFACES],
    forcee: bool,
}

impl SequencePrevol {
    pub fn nouvelle() -> Self {
        Self { testees: [false; NOMBRE_SURFACES], forcee: false }
    }

    /// Passe la vérification sans la dérouler, sur décision de l'opérateur
    /// (`doc/mission.md`, « Vérification pré-vol forcée »).
    pub fn forcer(&mut self) {
        self.forcee = true;
    }

    /// Vrai si la vérification a été passée sans être déroulée.
    pub fn forcee(&self) -> bool {
        self.forcee && !self.testees.iter().all(|&t| t)
    }

    /// Marque une surface comme testée cette session.
    pub fn marquer_testee(&mut self, surface: SurfaceServo) {
        self.testees[surface.indice()] = true;
    }

    /// Vrai si les trois surfaces ont été testées cette session, ou si
    /// l'opérateur a forcé la vérification.
    pub fn complete(&self) -> bool {
        self.forcee || self.testees.iter().all(|&t| t)
    }

    /// Nombre de surfaces déjà testées, pour affichage de progression.
    pub fn nombre_testees(&self) -> usize {
        self.testees.iter().filter(|&&t| t).count()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::drivers::controleur_servo::{IMPULSION_MAX_US, IMPULSION_MIN_US, IMPULSION_NEUTRE_US};

    #[test]
    fn test_etapes_ne_bougent_que_la_surface_testee_sans_armer() {
        for surface in SurfaceServo::TOUTES {
            for etape in surface.etapes() {
                let c = etape.consignes();
                assert!(!c.arme);
                assert_eq!(c.gaz, IMPULSION_MIN_US);
                let (testee, autres) = match surface {
                    SurfaceServo::Ailerons => (c.ailerons, [c.profondeur, c.direction]),
                    SurfaceServo::Profondeur => (c.profondeur, [c.ailerons, c.direction]),
                    SurfaceServo::Direction => (c.direction, [c.ailerons, c.profondeur]),
                };
                assert_eq!(testee == IMPULSION_NEUTRE_US, etape.valeur == 0.0);
                assert_eq!(autres, [IMPULSION_NEUTRE_US; 2]);
            }
        }
    }

    #[test]
    fn test_forcer_complete_la_sequence_sans_marquer_les_surfaces() {
        let mut seq = SequencePrevol::nouvelle();
        assert!(!seq.forcee());

        seq.forcer();

        assert!(seq.complete());
        assert!(seq.forcee());
        assert_eq!(seq.nombre_testees(), 0);
    }

    #[test]
    fn test_verification_deroulee_prime_sur_le_forcage() {
        let mut seq = SequencePrevol::nouvelle();
        seq.forcer();
        for surface in SurfaceServo::TOUTES {
            seq.marquer_testee(surface);
        }
        assert!(seq.complete());
        assert!(!seq.forcee());
    }

    #[test]
    fn test_etapes_appliquent_les_sens_du_mixeur() {
        // Mêmes impulsions que l'autopilote pour une même commande :
        // doc/essais/E0.4.md, profondeur et direction inversées.
        assert_eq!(SurfaceServo::Ailerons.etapes()[0].consignes().ailerons, IMPULSION_MAX_US);
        assert_eq!(SurfaceServo::Profondeur.etapes()[0].consignes().profondeur, IMPULSION_MIN_US);
        assert_eq!(SurfaceServo::Direction.etapes()[0].consignes().direction, IMPULSION_MIN_US);
    }

    #[test]
    fn test_sens_positif_conforme_a_la_convention_de_l_estimation() {
        // Une commande positive doit augmenter l'angle : voir l'en-tête du module.
        assert_eq!(SurfaceServo::Ailerons.etapes()[0].commande, "ROULIS À DROITE");
        assert_eq!(SurfaceServo::Profondeur.etapes()[0].commande, "CABRER");
        assert_eq!(SurfaceServo::Direction.etapes()[0].commande, "NEZ À DROITE");
        for surface in SurfaceServo::TOUTES {
            assert_eq!(surface.etapes().map(|e| e.valeur), [1.0, -1.0, 0.0]);
        }
    }

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
