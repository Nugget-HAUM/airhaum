// src/controle/controleur_direction_sol.rs
//! Contrôleur de direction au sol — maintien de cap pendant `RoulageAuSol` et
//! `CourseDecollage`.
//!
//! Distinct du contrôleur de lacet en vol (`controleur_lacet.rs`, à venir),
//! comme `AP_SteerController` vs `AP_YawController` sur ArduPlane — même
//! gouverne physique (`direction`), logique différente. PID simple, pas de
//! cascade angle→taux (pas de notion de taux de lacet pertinente à l'arrêt
//! ou à faible vitesse sol). Voir `doc/controle.md`.
//!
//! La mesure de cap (`EtatNavigation.attitude.lacet`) est aujourd'hui estimée
//! par intégration gyro seule, sans fusion magnétomètre ni cap GPS — limite
//! connue documentée dans `doc/suivi_projet.md`, qui n'a pas à être traitée
//! par cette brique.

use crate::controle::pid::Pid;
use crate::types::Angle;

/// Consigne de cap au sol.
#[derive(Debug, Clone, Copy)]
pub struct ConsigneDirectionSol {
    pub cap_cible: Angle,
}

/// Contrôleur de direction au sol.
#[derive(Debug, Clone)]
pub struct ControleurDirectionSol {
    pid: Pid,
}

impl ControleurDirectionSol {
    /// Bornes de sortie fixées à [-1, 1] (commande normalisée du mixeur).
    pub fn nouveau(kp: f32, ki: f32, kd: f32, limite_integrale: f32) -> Self {
        Self {
            pid: Pid::nouveau(kp, ki, kd, limite_integrale, -1.0, 1.0),
        }
    }

    /// Calcule la commande direction normalisée à partir du cap mesuré.
    pub fn mettre_a_jour(&mut self, consigne: &ConsigneDirectionSol, cap_mesure: Angle, dt: f32) -> f32 {
        let erreur = cap_mesure.erreur_vers(consigne.cap_cible).radians();
        self.pid.mettre_a_jour(erreur, dt)
    }

    /// Remet à zéro l'état interne du PID (voir `doc/controle.md`, gel des
    /// contrôleurs inactifs).
    pub fn reinitialiser(&mut self) {
        self.pid.reinitialiser();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn commande_nulle_quand_cap_atteint() {
        let mut controleur = ControleurDirectionSol::nouveau(1.0, 0.0, 0.0, 1.0);
        let consigne = ConsigneDirectionSol { cap_cible: Angle::depuis_degres(90.0) };
        let commande = controleur.mettre_a_jour(&consigne, Angle::depuis_degres(90.0), 0.01);
        assert!(commande.abs() < 1e-6);
    }

    #[test]
    fn commande_de_signe_oppose_a_l_erreur_de_cap() {
        // cap mesuré au-dessus de la cible (erreur cible-mesure négative) → commande négative.
        // Sens physique réel (quel câblage tourne à droite/gauche) : à déterminer au calibrage matériel.
        let mut controleur = ControleurDirectionSol::nouveau(1.0, 0.0, 0.0, 1.0);
        let consigne = ConsigneDirectionSol { cap_cible: Angle::depuis_degres(0.0) };
        let commande = controleur.mettre_a_jour(&consigne, Angle::depuis_degres(10.0), 0.01);
        assert!(commande < 0.0);
    }

    #[test]
    fn gere_le_rebouclage_a_zero_degre() {
        // mesure à 359°, cible à 1° : l'erreur doit être petite (+2°), pas -358°
        let mut controleur = ControleurDirectionSol::nouveau(1.0, 0.0, 0.0, 1.0);
        let consigne = ConsigneDirectionSol { cap_cible: Angle::depuis_degres(1.0) };
        let commande = controleur.mettre_a_jour(&consigne, Angle::depuis_degres(359.0), 0.01);
        // erreur ≈ +2° en rad ≈ 0.035, * kp=1.0 → commande positive et petite
        assert!(commande > 0.0 && commande < 0.1);
    }

    #[test]
    fn reinitialiser_efface_l_integrale() {
        let mut controleur = ControleurDirectionSol::nouveau(0.0, 1.0, 0.0, 10.0);
        let consigne = ConsigneDirectionSol { cap_cible: Angle::depuis_degres(10.0) };
        controleur.mettre_a_jour(&consigne, Angle::depuis_degres(0.0), 1.0);
        controleur.reinitialiser();
        // sans réinitialisation, l'intégrale accumulée aurait influencé cet appel
        let commande = controleur.mettre_a_jour(&consigne, Angle::depuis_degres(10.0), 1.0);
        assert!(commande.abs() < 1e-6);
    }
}
