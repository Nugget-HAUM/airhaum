// src/controle/controleur_roulis.rs
//! Contrôleur de roulis en vol — cascade angle → taux → commande ailerons.
//!
//! Comme `AP_RollController` sur ArduPlane : une boucle externe (P seul) sur
//! l'erreur d'angle produit un taux de roulis cible, une boucle interne (PID
//! complet, via `Pid`) asservit ce taux à partir du gyroscope. Voir
//! `doc/controle.md`, section « Cascade angle → taux ».
//!
//! **Source du taux mesuré** : gyroscope corps (`traitement_imu.rs`, biais
//! statique déjà retiré), corrigé du biais dynamique estimé par l'EKF
//! (`EtatNavigation.biais_gyro.x`) — calcul fait par l'appelant, ce module
//! reste découplé de sa source comme `controleur_direction_sol.rs`.
//!
//! Pas de rebouclage à gérer sur l'angle de roulis : l'enveloppe de vol de
//! cet avion reste bien en-deçà de ±180°, mais `Angle::erreur_vers` est
//! réutilisée par cohérence avec les autres contrôleurs d'angle.

use crate::controle::pid::Pid;
use crate::types::Angle;

/// Consigne de roulis (0 = ailes à plat, vol en ligne droite).
#[derive(Debug, Clone, Copy)]
pub struct ConsigneRoulis {
    pub roulis_cible: Angle,
}

/// Contrôleur de roulis en vol : cascade angle → taux.
#[derive(Debug, Clone)]
pub struct ControleurRoulis {
    /// Gain de la boucle externe (P seul) : erreur d'angle (rad) → taux cible (rad/s).
    gain_p_angle: f32,
    /// Boucle interne : erreur de taux (rad/s) → commande ailerons normalisée [-1, 1].
    pid_taux: Pid,
}

impl ControleurRoulis {
    /// Bornes de sortie fixées à [-1, 1] (commande normalisée du mixeur).
    pub fn nouveau(gain_p_angle: f32, kp: f32, ki: f32, kd: f32, limite_integrale: f32) -> Self {
        Self {
            gain_p_angle,
            pid_taux: Pid::nouveau(kp, ki, kd, limite_integrale, -1.0, 1.0),
        }
    }

    /// Calcule la commande ailerons normalisée.
    ///
    /// - `roulis_mesure` : `EtatNavigation.attitude.roulis`.
    /// - `taux_roulis_mesure` : gyroscope corps, axe roulis, biais statique et
    ///   dynamique retirés (rad/s).
    pub fn mettre_a_jour(
        &mut self,
        consigne: &ConsigneRoulis,
        roulis_mesure: Angle,
        taux_roulis_mesure: f32,
        dt: f32,
    ) -> f32 {
        let erreur_angle = roulis_mesure.erreur_vers(consigne.roulis_cible).radians();
        let taux_cible = self.gain_p_angle * erreur_angle;
        let erreur_taux = taux_cible - taux_roulis_mesure;
        self.pid_taux.mettre_a_jour(erreur_taux, dt)
    }

    /// Remet à zéro l'état interne du PID de taux (voir `doc/controle.md`,
    /// gel des contrôleurs inactifs).
    pub fn reinitialiser(&mut self) {
        self.pid_taux.reinitialiser();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn commande_nulle_quand_roulis_et_taux_atteints() {
        let mut controleur = ControleurRoulis::nouveau(1.0, 1.0, 0.0, 0.0, 1.0);
        let consigne = ConsigneRoulis { roulis_cible: Angle::depuis_degres(0.0) };
        let commande = controleur.mettre_a_jour(&consigne, Angle::depuis_degres(0.0), 0.0, 0.01);
        assert!(commande.abs() < 1e-6);
    }

    #[test]
    fn commande_de_signe_coherent_avec_erreur_angle() {
        // roulis mesuré au-dessus de la cible, taux nul : erreur d'angle négative
        // → taux cible négatif → commande négative.
        // Sens physique réel (quel câblage abaisse quelle aile) : à déterminer
        // au calibrage matériel, comme pour `controleur_direction_sol.rs`.
        let mut controleur = ControleurRoulis::nouveau(1.0, 1.0, 0.0, 0.0, 1.0);
        let consigne = ConsigneRoulis { roulis_cible: Angle::depuis_degres(0.0) };
        let commande = controleur.mettre_a_jour(&consigne, Angle::depuis_degres(10.0), 0.0, 0.01);
        assert!(commande < 0.0);
    }

    #[test]
    fn commande_nulle_si_taux_mesure_egal_taux_cible() {
        // Erreur d'angle non nulle, mais l'appareil tourne déjà exactement au
        // taux demandé par la boucle externe : la boucle interne (P seul ici)
        // ne doit plus rien commander — c'est l'amortissement propre à la
        // cascade angle → taux.
        let mut controleur = ControleurRoulis::nouveau(2.0, 1.0, 0.0, 0.0, 1.0);
        let consigne = ConsigneRoulis { roulis_cible: Angle::depuis_degres(10.0) };
        // erreur d'angle = 10°, gain_p_angle = 2.0 → taux_cible = 20°/s en rad/s
        let taux_cible = 2.0 * Angle::depuis_degres(10.0).radians();
        let commande = controleur.mettre_a_jour(&consigne, Angle::depuis_degres(0.0), taux_cible, 0.01);
        assert!(commande.abs() < 1e-6);
    }

    #[test]
    fn taux_mesure_amortit_la_commande() {
        // Même erreur d'angle, mais un taux mesuré non nul (dans le sens de la
        // correction) doit réduire la commande par rapport à un taux mesuré nul.
        let consigne = ConsigneRoulis { roulis_cible: Angle::depuis_degres(0.0) };

        let mut sans_taux = ControleurRoulis::nouveau(1.0, 1.0, 0.0, 0.0, 1.0);
        let commande_sans_taux = sans_taux.mettre_a_jour(&consigne, Angle::depuis_degres(10.0), 0.0, 0.01);

        let mut avec_taux = ControleurRoulis::nouveau(1.0, 1.0, 0.0, 0.0, 1.0);
        let taux_partiel = 0.5 * Angle::depuis_degres(-10.0).radians();
        let commande_avec_taux = avec_taux.mettre_a_jour(&consigne, Angle::depuis_degres(10.0), taux_partiel, 0.01);

        assert!(commande_avec_taux.abs() < commande_sans_taux.abs());
    }

    #[test]
    fn saturation_a_grande_erreur_angle() {
        let mut controleur = ControleurRoulis::nouveau(10.0, 10.0, 0.0, 0.0, 1.0);
        let consigne = ConsigneRoulis { roulis_cible: Angle::depuis_degres(90.0) };
        let commande = controleur.mettre_a_jour(&consigne, Angle::depuis_degres(-90.0), 0.0, 0.01);
        assert_eq!(commande, 1.0);
    }

    #[test]
    fn reinitialiser_efface_l_integrale() {
        let mut controleur = ControleurRoulis::nouveau(1.0, 0.0, 1.0, 0.0, 10.0);
        let consigne = ConsigneRoulis { roulis_cible: Angle::depuis_degres(10.0) };
        controleur.mettre_a_jour(&consigne, Angle::depuis_degres(0.0), 0.0, 1.0);
        controleur.reinitialiser();
        // sans réinitialisation, l'intégrale accumulée aurait influencé cet appel
        let commande = controleur.mettre_a_jour(&consigne, Angle::depuis_degres(10.0), 0.0, 1.0);
        assert!(commande.abs() < 1e-6);
    }
}
