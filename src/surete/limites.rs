// src/surete/limites.rs
//! Géofence et enveloppe de vol.
//!
//! Module de logique pure — aucune E/S. Évalue la position/vitesse estimées
//! (`EtatNavigation`, produit par l'EKF de navigation) contre des limites
//! configurées. Alimente `EtatSecurite` via `surete/urgences.rs` (à venir).
//!
//! Repère NED : x = Nord, y = Est, z = Bas (altitude = -z). L'origine est le
//! premier fix GPS (voir `estimation/ekf_navigation.rs`) — tant qu'elle n'est
//! pas définie, aucune limite ne peut être évaluée de façon fiable.

use crate::estimation::EtatNavigation;

/// Marge d'avertissement avant le dépassement franc (ex. 0.9 = 90 % de la limite).
const MARGE_AVERTISSEMENT: f32 = 0.9;

/// État d'une grandeur surveillée vis-à-vis de ses limites.
#[derive(Debug, Clone, PartialEq)]
pub enum EtatLimites {
    /// Toutes les grandeurs surveillées sont dans les limites.
    DansLesLimites,
    /// Une limite est approchée (> `MARGE_AVERTISSEMENT`) mais pas dépassée.
    Approche { raison: String },
    /// Une limite est dépassée.
    Depassee { raison: String },
}

impl EtatLimites {
    pub fn est_depassee(&self) -> bool {
        matches!(self, EtatLimites::Depassee { .. })
    }
}

/// Limites configurées pour la mission. `None` désactive la vérification
/// correspondante.
#[derive(Debug, Clone, Default)]
pub struct LimitesVol {
    /// Rayon géofence horizontal autour de l'origine (m).
    pub rayon_max_m: Option<f32>,
    /// Plafond d'altitude au-dessus de l'origine (m).
    pub altitude_max_m: Option<f32>,
}

/// Évalue la position/vitesse courante contre les limites configurées.
///
/// Retourne le premier dépassement franc rencontré, sinon la première
/// approche de limite, sinon `DansLesLimites`.
pub fn evaluer_limites(nav: &EtatNavigation, limites: &LimitesVol) -> EtatLimites {
    if !nav.origine_definie {
        return EtatLimites::DansLesLimites;
    }

    let distance_horizontale =
        (nav.position_ned.x.powi(2) + nav.position_ned.y.powi(2)).sqrt();
    let altitude = -nav.position_ned.z;

    if let Some(rayon_max) = limites.rayon_max_m {
        if distance_horizontale > rayon_max {
            return EtatLimites::Depassee {
                raison: format!("Géofence : {distance_horizontale:.0} m > {rayon_max:.0} m"),
            };
        }
    }
    if let Some(altitude_max) = limites.altitude_max_m {
        if altitude > altitude_max {
            return EtatLimites::Depassee {
                raison: format!("Plafond : {altitude:.0} m > {altitude_max:.0} m"),
            };
        }
    }

    if let Some(rayon_max) = limites.rayon_max_m {
        if distance_horizontale > rayon_max * MARGE_AVERTISSEMENT {
            return EtatLimites::Approche {
                raison: format!("Géofence : {distance_horizontale:.0} m (limite {rayon_max:.0} m)"),
            };
        }
    }
    if let Some(altitude_max) = limites.altitude_max_m {
        if altitude > altitude_max * MARGE_AVERTISSEMENT {
            return EtatLimites::Approche {
                raison: format!("Plafond : {altitude:.0} m (limite {altitude_max:.0} m)"),
            };
        }
    }

    EtatLimites::DansLesLimites
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::Vector3;

    fn nav_a(position_ned: Vector3) -> EtatNavigation {
        EtatNavigation {
            position_ned,
            origine_definie: true,
            ..EtatNavigation::nul()
        }
    }

    #[test]
    fn test_origine_non_definie_ignore_les_limites() {
        let nav = EtatNavigation::nul(); // origine_definie = false
        let limites = LimitesVol { rayon_max_m: Some(10.0), altitude_max_m: Some(10.0) };
        assert_eq!(evaluer_limites(&nav, &limites), EtatLimites::DansLesLimites);
    }

    #[test]
    fn test_dans_les_limites() {
        let nav = nav_a(Vector3::nouveau(50.0, 0.0, -20.0)); // 50 m, alt 20 m
        let limites = LimitesVol { rayon_max_m: Some(200.0), altitude_max_m: Some(120.0) };
        assert_eq!(evaluer_limites(&nav, &limites), EtatLimites::DansLesLimites);
    }

    #[test]
    fn test_approche_geofence() {
        let nav = nav_a(Vector3::nouveau(190.0, 0.0, -20.0)); // 190 m / 200 m = 95 %
        let limites = LimitesVol { rayon_max_m: Some(200.0), altitude_max_m: None };
        assert!(matches!(evaluer_limites(&nav, &limites), EtatLimites::Approche { .. }));
    }

    #[test]
    fn test_geofence_depassee() {
        let nav = nav_a(Vector3::nouveau(250.0, 0.0, -20.0));
        let limites = LimitesVol { rayon_max_m: Some(200.0), altitude_max_m: None };
        let etat = evaluer_limites(&nav, &limites);
        assert!(etat.est_depassee());
    }

    #[test]
    fn test_plafond_depasse() {
        let nav = nav_a(Vector3::nouveau(0.0, 0.0, -150.0)); // alt 150 m
        let limites = LimitesVol { rayon_max_m: None, altitude_max_m: Some(120.0) };
        let etat = evaluer_limites(&nav, &limites);
        assert!(etat.est_depassee());
    }

    #[test]
    fn test_limite_desactivee_ignoree() {
        let nav = nav_a(Vector3::nouveau(10_000.0, 0.0, -10_000.0));
        let limites = LimitesVol { rayon_max_m: None, altitude_max_m: None };
        assert_eq!(evaluer_limites(&nav, &limites), EtatLimites::DansLesLimites);
    }
}
