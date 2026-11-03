// src/controle/mixeur.rs
//! Mixeur — commandes normalisées → `ConsignesServos` (µs).
//!
//! Avion classique à 3 gouvernes indépendantes (pas de V-tail ni flying
//! wing) : mixage trivial 1:1 par axe, aucun couplage à gérer. Voir
//! `doc/controle.md`.
//!
//! Seul le mixeur connaît le sens de montage des servos : pour le reste du
//! système, une commande positive augmente l'angle correspondant (roulis à
//! droite, nez haut, nez à droite). Voir `doc/controle.md`, « Sens des
//! commandes ».

use crate::drivers::controleur_servo::{
    ConsignesServos, IMPULSION_MAX_US, IMPULSION_MIN_US, IMPULSION_NEUTRE_US,
};

/// Sens de montage des servos, relevés à l'essai (`doc/essais/E0.4.md`) :
/// -1 si une impulsion croissante produit l'effet inverse de la convention.
const SENS_AILERONS: f32 = 1.0;
const SENS_PROFONDEUR: f32 = -1.0;
const SENS_DIRECTION: f32 = -1.0;

/// Demi-plage de débattement autour du neutre (µs).
const DEMI_PLAGE_US: f32 = (IMPULSION_MAX_US - IMPULSION_MIN_US) as f32 / 2.0;

/// Plage complète de débattement (µs).
const PLAGE_US: f32 = (IMPULSION_MAX_US - IMPULSION_MIN_US) as f32;

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

/// Consigne de gaz normalisée [0, 1] (au-delà : saturé), fournie par la
/// mission. Passthrough direct — pas de régulation asservie tant que
/// `controleur_vitesse.rs`/`controleur_altitude.rs` n'existent pas (voir
/// `doc/controle.md`). Normalisée comme les autres commandes plutôt
/// qu'exprimée en µs, pour ne pas exposer les bornes matérielles du servo
/// à la mission.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct ConsigneGaz {
    pub gaz: f32,
}

/// Convertit une commande normalisée [-1, 1] en impulsion µs autour du neutre.
fn normalisee_vers_us(commande: f32) -> u16 {
    let us = IMPULSION_NEUTRE_US as f32 + commande.clamp(-1.0, 1.0) * DEMI_PLAGE_US;
    us.clamp(IMPULSION_MIN_US as f32, IMPULSION_MAX_US as f32) as u16
}

/// Convertit une consigne gaz normalisée [0, 1] en impulsion µs.
fn gaz_normalise_vers_us(gaz: f32) -> u16 {
    let us = IMPULSION_MIN_US as f32 + gaz.clamp(0.0, 1.0) * PLAGE_US;
    us.clamp(IMPULSION_MIN_US as f32, IMPULSION_MAX_US as f32) as u16
}

/// Assemble les `ConsignesServos` à partir des commandes de surfaces et de
/// la consigne gaz, toutes normalisées.
///
/// Défense en profondeur : si `!arme`, le gaz est forcé au minimum quelle
/// que soit `consigne_gaz` — l'Arduino applique déjà sa propre garde sur
/// `arme`, mais le mixeur ne doit pas laisser fuiter une consigne gaz non
/// nulle restée dans un canal `watch` après un désarmement (voir le plan
/// d'essais, E0.8 : coupure gaz sans délai).
pub fn mixer(commandes: &CommandesNormalisees, consigne_gaz: &ConsigneGaz, arme: bool) -> ConsignesServos {
    ConsignesServos {
        ailerons:   normalisee_vers_us(SENS_AILERONS * commandes.ailerons),
        profondeur: normalisee_vers_us(SENS_PROFONDEUR * commandes.profondeur),
        gaz:        if arme { gaz_normalise_vers_us(consigne_gaz.gaz) } else { IMPULSION_MIN_US },
        direction:  normalisee_vers_us(SENS_DIRECTION * commandes.direction),
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
    fn gaz_nul_au_minimum_et_gaz_plein_au_maximum() {
        assert_eq!(gaz_normalise_vers_us(0.0), IMPULSION_MIN_US);
        assert_eq!(gaz_normalise_vers_us(1.0), IMPULSION_MAX_US);
    }

    #[test]
    fn gaz_saturation_hors_plage() {
        assert_eq!(gaz_normalise_vers_us(5.0), IMPULSION_MAX_US);
        assert_eq!(gaz_normalise_vers_us(-5.0), IMPULSION_MIN_US);
    }

    #[test]
    fn mixer_assemble_les_quatre_axes() {
        let commandes = CommandesNormalisees {
            ailerons: 0.5,
            profondeur: -0.5,
            direction: 1.0,
        };
        let consignes = mixer(&commandes, &ConsigneGaz { gaz: 0.5 }, true);

        assert_eq!(consignes.ailerons, IMPULSION_NEUTRE_US + (DEMI_PLAGE_US * 0.5) as u16);
        assert_eq!(consignes.profondeur, IMPULSION_NEUTRE_US + (DEMI_PLAGE_US * 0.5) as u16);
        assert_eq!(consignes.direction, IMPULSION_MIN_US);
        assert_eq!(consignes.gaz, IMPULSION_MIN_US + (PLAGE_US * 0.5) as u16);
        assert!(consignes.arme);
    }

    #[test]
    fn sens_de_montage_releves_a_l_essai() {
        // doc/essais/E0.4.md : profondeur et direction montées à l'envers.
        let commandes = CommandesNormalisees { ailerons: 1.0, profondeur: 1.0, direction: 1.0 };
        let consignes = mixer(&commandes, &ConsigneGaz::default(), false);

        assert_eq!(consignes.ailerons, IMPULSION_MAX_US);
        assert_eq!(consignes.profondeur, IMPULSION_MIN_US);
        assert_eq!(consignes.direction, IMPULSION_MIN_US);
    }

    #[test]
    fn mixer_force_le_gaz_au_minimum_si_non_arme() {
        // Défense en profondeur : même avec une consigne gaz pleine, un
        // appareil non armé ne doit jamais recevoir de gaz non nul.
        let consignes = mixer(&CommandesNormalisees::default(), &ConsigneGaz { gaz: 1.0 }, false);
        assert_eq!(consignes.gaz, IMPULSION_MIN_US);
        assert!(!consignes.arme);
    }
}
