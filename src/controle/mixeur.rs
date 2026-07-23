// src/controle/mixeur.rs
//! Mixeur — commandes normalisées → `ConsignesServos` (µs).
//!
//! Avion classique à 3 gouvernes indépendantes (pas de V-tail ni flying
//! wing) : mixage trivial 1:1 par axe, aucun couplage à gérer. Voir
//! `doc/controle.md`.

use crate::drivers::controleur_servo::{
    ConsignesServos, IMPULSION_MAX_US, IMPULSION_MIN_US, IMPULSION_NEUTRE_US,
};

/// Demi-plage de débattement autour du neutre (µs).
const DEMI_PLAGE_US: f32 = (IMPULSION_MAX_US - IMPULSION_MIN_US) as f32 / 2.0;

/// Commandes de surfaces normalisées, bornées à [-1, 1] (au-delà : saturé).
///
/// La provenance de `direction` dépend de la phase de vol (contrôleur de
/// direction sol ou de lacet en vol) mais le mixeur n'a pas à le savoir —
/// voir « Arbitrage direction sol / lacet vol » dans `doc/controle.md`.
#[derive(Debug, Clone, Copy, Default)]
pub struct CommandesNormalisees {
    pub ailerons:   f32,
    pub profondeur: f32,
    pub direction:  f32,
}

/// Convertit une commande normalisée [-1, 1] en impulsion µs autour du neutre.
fn normalisee_vers_us(commande: f32) -> u16 {
    let us = IMPULSION_NEUTRE_US as f32 + commande.clamp(-1.0, 1.0) * DEMI_PLAGE_US;
    us.clamp(IMPULSION_MIN_US as f32, IMPULSION_MAX_US as f32) as u16
}

/// Assemble les `ConsignesServos` à partir des commandes de surfaces
/// normalisées et d'une consigne gaz déjà en µs (passthrough — pas de PID
/// gaz tant que `controleur_vitesse.rs`/`controleur_altitude.rs` n'existent
/// pas, voir `doc/controle.md`).
pub fn mixer(commandes: &CommandesNormalisees, gaz_us: u16, arme: bool) -> ConsignesServos {
    ConsignesServos {
        ailerons:   normalisee_vers_us(commandes.ailerons),
        profondeur: normalisee_vers_us(commandes.profondeur),
        gaz:        gaz_us.clamp(IMPULSION_MIN_US, IMPULSION_MAX_US),
        direction:  normalisee_vers_us(commandes.direction),
        arme,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn neutre_a_commande_nulle() {
        assert_eq!(normalisee_vers_us(0.0), IMPULSION_NEUTRE_US);
    }

    #[test]
    fn butees_a_plus_ou_moins_un() {
        assert_eq!(normalisee_vers_us(1.0), IMPULSION_MAX_US);
        assert_eq!(normalisee_vers_us(-1.0), IMPULSION_MIN_US);
    }

    #[test]
    fn saturation_au_dela_de_un() {
        assert_eq!(normalisee_vers_us(5.0), IMPULSION_MAX_US);
        assert_eq!(normalisee_vers_us(-5.0), IMPULSION_MIN_US);
    }

    #[test]
    fn mixer_assemble_les_trois_axes() {
        let commandes = CommandesNormalisees {
            ailerons: 0.5,
            profondeur: -0.5,
            direction: 1.0,
        };
        let consignes = mixer(&commandes, 1_200, true);

        assert_eq!(consignes.ailerons, IMPULSION_NEUTRE_US + (DEMI_PLAGE_US * 0.5) as u16);
        assert_eq!(consignes.profondeur, IMPULSION_NEUTRE_US - (DEMI_PLAGE_US * 0.5) as u16);
        assert_eq!(consignes.direction, IMPULSION_MAX_US);
        assert_eq!(consignes.gaz, 1_200);
        assert!(consignes.arme);
    }

    #[test]
    fn mixer_clampe_le_gaz_hors_plage() {
        let consignes = mixer(&CommandesNormalisees::default(), 3_000, false);
        assert_eq!(consignes.gaz, IMPULSION_MAX_US);
        assert!(!consignes.arme);
    }
}
