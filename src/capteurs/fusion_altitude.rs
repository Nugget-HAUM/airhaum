// src/capteurs/fusion_altitude.rs
//! Fusion baromètre + télémètre en altitude estimée.
//!
//! Module pur (sans I/O) : les entrées sont des références vers des mesures
//! déjà lues depuis les canaux watch. Aucun accès capteur direct.
//!
//! # Rôle dans l'architecture
//!
//! ```text
//! rx_baro  (watch) ──▶  fusionner()  ──▶  AltitudeFusionnee
//! rx_telem (watch) ──▶
//! ```
//!
//! La référence de pression (`pression_reference`) doit être fournie par la
//! calibration pré-vol (`CalibrationBarometre`). En l'absence de calibration,
//! `Pression::niveau_mer_standard()` (ISA, 101325 Pa) est utilisée.

use crate::taches::taches_capteurs::{MesureBaro, MesureTelem};
use crate::types::Pression;

// ─────────────────────────────────────────────────────────────────────────────
// Type de sortie
// ─────────────────────────────────────────────────────────────────────────────

/// Altitude estimée issue de la fusion baromètre + télémètre.
///
/// Les deux champs sont indépendants : un peut être `Some` même si l'autre
/// est `None` (ex. télémètre hors portée en altitude).
#[derive(Debug, Clone, Copy)]
pub struct AltitudeFusionnee {
    /// Altitude barométrique en mètres au-dessus du niveau de référence.
    /// `None` si la mesure baro est absente ou invalide.
    pub altitude_m: Option<f32>,
    /// Hauteur au-dessus du sol en millimètres (télémètre laser).
    /// `None` si hors portée, invalide ou capteur non initialisé.
    pub hauteur_sol_mm: Option<u32>,
}

impl AltitudeFusionnee {
    /// État initial avant toute mesure.
    pub fn nulle() -> Self {
        Self { altitude_m: None, hauteur_sol_mm: None }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Fonction de fusion
// ─────────────────────────────────────────────────────────────────────────────

/// Fusionne une mesure baromètre et une mesure télémètre en altitude estimée.
///
/// `pression_reference` : pression mesurée au sol lors de la calibration pré-vol.
/// Si non calibré, passer `Pression::niveau_mer_standard()`.
pub fn fusionner(
    baro:  &MesureBaro,
    telem: &MesureTelem,
    pression_reference: Pression,
) -> AltitudeFusionnee {
    let altitude_m = baro.donnees
        .as_ref()
        .filter(|_| baro.valide)
        .map(|d| d.pression.vers_altitude(pression_reference));

    let hauteur_sol_mm = telem.distance_mm
        .filter(|_| telem.valide)
        .map(|d| d as u32);

    AltitudeFusionnee { altitude_m, hauteur_sol_mm }
}

// ─────────────────────────────────────────────────────────────────────────────
// Tests
// ─────────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::{DonneesBarometre, Temperature, Horodatage};

    fn baro_valide(pression_pa: f32) -> MesureBaro {
        MesureBaro {
            donnees: Some(DonneesBarometre {
                pression:     Pression::depuis_pascals(pression_pa),
                temperature:  Temperature::depuis_celsius(20.0),
                horodatage:   Horodatage::maintenant(),
            }),
            valide: true,
            erreurs_consecutives: 0,
        }
    }

    fn baro_invalide() -> MesureBaro {
        MesureBaro { donnees: None, valide: false, erreurs_consecutives: 1 }
    }

    fn telem_valide(distance_mm: u16) -> MesureTelem {
        MesureTelem { distance_mm: Some(distance_mm), valide: true, erreurs_consecutives: 0 }
    }

    fn telem_invalide() -> MesureTelem {
        MesureTelem { distance_mm: None, valide: false, erreurs_consecutives: 1 }
    }

    #[test]
    fn baro_au_niveau_mer_donne_altitude_nulle() {
        let alt = fusionner(
            &baro_valide(101325.0),
            &telem_invalide(),
            Pression::niveau_mer_standard(),
        );
        let h = alt.altitude_m.expect("altitude attendue");
        assert!(h.abs() < 1.0, "au niveau de la mer, altitude ≈ 0m, obtenu {:.2}m", h);
    }

    #[test]
    fn baro_invalide_donne_altitude_none() {
        let alt = fusionner(
            &baro_invalide(),
            &telem_invalide(),
            Pression::niveau_mer_standard(),
        );
        assert!(alt.altitude_m.is_none());
    }

    #[test]
    fn altitude_1000m_coherente() {
        // À ~1000 m d'altitude, la pression est environ 89875 Pa (ISA standard)
        let alt = fusionner(
            &baro_valide(89875.0),
            &telem_invalide(),
            Pression::niveau_mer_standard(),
        );
        let h = alt.altitude_m.expect("altitude attendue");
        assert!(h > 900.0 && h < 1100.0, "altitude attendue ≈ 1000m, obtenu {:.1}m", h);
    }

    #[test]
    fn telem_valide_donne_hauteur() {
        let alt = fusionner(
            &baro_invalide(),
            &telem_valide(500),
            Pression::niveau_mer_standard(),
        );
        assert_eq!(alt.hauteur_sol_mm, Some(500));
    }

    #[test]
    fn telem_invalide_donne_hauteur_none() {
        let alt = fusionner(
            &baro_invalide(),
            &telem_invalide(),
            Pression::niveau_mer_standard(),
        );
        assert!(alt.hauteur_sol_mm.is_none());
    }

    #[test]
    fn baro_et_telem_valides_les_deux_presents() {
        let alt = fusionner(
            &baro_valide(101325.0),
            &telem_valide(1200),
            Pression::niveau_mer_standard(),
        );
        assert!(alt.altitude_m.is_some());
        assert_eq!(alt.hauteur_sol_mm, Some(1200));
    }
}
