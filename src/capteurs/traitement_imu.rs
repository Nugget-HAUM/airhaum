// src/capteurs/traitement_imu.rs
//! Prétraitement des mesures de la centrale inertielle
//!
//! Le driver MPU9250 publie des [`DonneesImu`] déjà converties en unités SI
//! (rad/s, m/s², µT) avec la calibration appliquée. Cette couche y ajoute
//! le seul élément manquant pour l'estimation d'état : le **Δt inter-mesures**.
//!
//! Sans ce Δt, ni le filtre complémentaire ni l'EKF ne peuvent intégrer
//! le gyroscope pour estimer l'attitude.
//!
//! # Flux de données
//!
//! ```text
//! MesureImu (FIFO)          MesureImuTraitee
//! ──────────────────         ──────────────────────────────
//!  donnees: DonneesImu  ──▶   donnees: DonneesImu  (inchangées)
//!  valide: bool               dt_s:    Option<f32>  (ajouté ici)
//! ```

use crate::types::{DonneesImu, Horodatage};
use crate::taches::taches_capteurs::MesureImu;

// ─────────────────────────────────────────────────────────────────────────────
// Type de sortie
// ─────────────────────────────────────────────────────────────────────────────

/// Mesure IMU prétraitée, prête à être consommée par l'estimateur.
///
/// Les données sont en unités SI avec calibration déjà appliquée.
/// Le seul ajout de cette couche est `dt_s`.
#[derive(Debug, Clone, Copy)]
pub struct MesureImuTraitee {
    /// Accéléromètre (m/s²), gyroscope (rad/s), magnétomètre (µT).
    /// Calibration et conversion d'unités déjà appliquées par le driver.
    pub donnees: DonneesImu,

    /// Durée écoulée depuis la mesure valide précédente, en secondes.
    ///
    /// `None` pour la toute première mesure (pas encore de référence).
    /// Le consommateur doit alors ignorer l'intégration gyroscope ou
    /// utiliser un dt nominal de secours (ex. 5 ms pour un IMU à 200 Hz).
    pub dt_s: Option<f32>,
}

// ─────────────────────────────────────────────────────────────────────────────
// Préprocesseur
// ─────────────────────────────────────────────────────────────────────────────

/// Préprocesseur stateful des mesures IMU.
///
/// Transforme chaque [`MesureImu`] en [`MesureImuTraitee`] en calculant
/// le Δt depuis la dernière mesure valide.
///
/// Les mesures invalides (capteur en erreur) sont ignorées sans faire avancer
/// la référence temporelle : le Δt suivant englobera le temps de l'interruption.
pub struct TraitementImu {
    dernier_horodatage: Option<Horodatage>,
}

impl TraitementImu {
    pub fn nouveau() -> Self {
        Self { dernier_horodatage: None }
    }

    /// Traite une mesure IMU brute.
    ///
    /// Retourne `None` si la mesure est invalide (données absentes).
    /// Dans ce cas l'état interne n'est pas modifié.
    pub fn traiter(&mut self, mesure: &MesureImu) -> Option<MesureImuTraitee> {
        let donnees = mesure.donnees?;

        let dt_s = self.dernier_horodatage.map(|precedent| {
            donnees.horodatage.delta_secondes(precedent)
        });

        self.dernier_horodatage = Some(donnees.horodatage);

        Some(MesureImuTraitee { donnees, dt_s })
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Tests
// ─────────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::{Vector3, Temperature};

    fn mesure_valide(micros: u64) -> MesureImu {
        MesureImu {
            donnees: Some(DonneesImu {
                horodatage:    Horodatage::depuis_micros(micros),
                accelerometre: Vector3::nouveau(0.0, 0.0, 9.81),
                gyroscope:     Vector3::nouveau(0.0, 0.0, 0.0),
                magnetometre:  Vector3::nouveau(0.0, 0.0, 0.0),
                temperature:   Temperature::depuis_celsius(25.0),
            }),
            valide: true,
            erreurs_consecutives: 0,
        }
    }

    fn mesure_invalide() -> MesureImu {
        MesureImu { donnees: None, valide: false, erreurs_consecutives: 1 }
    }

    #[test]
    fn premiere_mesure_dt_none() {
        let mut proc = TraitementImu::nouveau();
        let r = proc.traiter(&mesure_valide(0)).unwrap();
        assert!(r.dt_s.is_none(), "Première mesure : dt doit être None");
    }

    #[test]
    fn dt_calcule_correctement() {
        let mut proc = TraitementImu::nouveau();
        proc.traiter(&mesure_valide(0));
        let r = proc.traiter(&mesure_valide(5_000)).unwrap(); // 5 ms
        let dt = r.dt_s.unwrap();
        assert!((dt - 0.005).abs() < 1e-6,
            "dt attendu 0.005 s, obtenu {} s", dt);
    }

    #[test]
    fn mesure_invalide_retourne_none() {
        let mut proc = TraitementImu::nouveau();
        assert!(proc.traiter(&mesure_invalide()).is_none());
        assert!(proc.dernier_horodatage.is_none(),
            "Une mesure invalide ne doit pas faire avancer la référence temporelle");
    }

    #[test]
    fn mesure_invalide_ne_perturbe_pas_dt() {
        // t=0 valide, t=? invalide, t=10ms valide → dt doit être 10ms (pas 5ms)
        let mut proc = TraitementImu::nouveau();
        proc.traiter(&mesure_valide(0));
        proc.traiter(&mesure_invalide());
        let r = proc.traiter(&mesure_valide(10_000)).unwrap();
        let dt = r.dt_s.unwrap();
        assert!((dt - 0.010).abs() < 1e-6,
            "dt attendu 0.010 s, obtenu {} s", dt);
    }

    #[test]
    fn donnees_inchangees() {
        let mut proc = TraitementImu::nouveau();
        let r = proc.traiter(&mesure_valide(0)).unwrap();
        assert!((r.donnees.accelerometre.z - 9.81).abs() < 1e-4);
        assert_eq!(r.donnees.horodatage.micros(), 0);
    }
}
