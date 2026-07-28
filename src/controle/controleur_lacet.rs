// src/controle/controleur_lacet.rs
//! Contrôleur de lacet en vol — coordination de virage (amortissement du
//! dérapage), commande direction.
//!
//! Contrairement à `controleur_roulis.rs`/`controleur_tangage.rs`, **pas de
//! cascade angle → taux** : `EtatNavigation.attitude.lacet` est estimé par
//! intégration gyro seule (pas de magnétomètre fiable ni de cap GPS fusionnés,
//! voir `controleur_direction_sol.rs`/`doc/suivi_projet.md`) et dérive sans
//! borne sur la durée d'un vol — l'asservir directement reviendrait à
//! asservir sur du bruit. Seul le **taux** de lacet (gyroscope, fiable à
//! court terme) est utilisé.
//!
//! **Coordination de virage complète** (taux cible = g·tan(roulis)/vitesse
//! air) nécessiterait une vitesse air fiable, absente du catalogue matériel
//! — la vitesse sol GPS est un candidat trop bruité/en retard et diverge de
//! la vitesse air en présence de vent, donc écartée pour cette brique. La
//! consigne (`ConsigneLacet::taux_lacet_cible`) est donc fournie telle
//! quelle par l'appelant : `0.0` pour un amortissement pur du dérapage
//! (valeur envisagée pour cette itération), avec la possibilité d'un
//! feedforward ultérieur si une vitesse air fiable devient disponible.
//!
//! **Source du taux mesuré** : gyroscope corps (`traitement_imu.rs`, biais
//! statique déjà retiré), corrigé du biais dynamique estimé par l'EKF
//! (`EtatNavigation.biais_gyro.z`) — calcul fait par l'appelant, comme pour
//! les autres contrôleurs d'axe.

use crate::controle::pid::Pid;

/// Consigne de taux de lacet (rad/s), pas un cap — voir doc de module.
#[derive(Debug, Clone, Copy)]
pub struct ConsigneLacet {
    pub taux_lacet_cible: f32,
}

/// Contrôleur de lacet en vol : simple PID de taux (pas de cascade).
#[derive(Debug, Clone)]
pub struct ControleurLacet {
    pid_taux: Pid,
}

impl ControleurLacet {
    /// Bornes de sortie fixées à [-1, 1] (commande normalisée du mixeur).
    pub fn nouveau(kp: f32, ki: f32, kd: f32, limite_integrale: f32) -> Self {
        Self {
            pid_taux: Pid::nouveau(kp, ki, kd, limite_integrale, -1.0, 1.0),
        }
    }

    /// Calcule la commande direction normalisée.
    ///
    /// `taux_lacet_mesure` : gyroscope corps, axe lacet, biais statique et
    /// dynamique retirés (rad/s).
    pub fn mettre_a_jour(&mut self, consigne: &ConsigneLacet, taux_lacet_mesure: f32, dt: f32) -> f32 {
        let erreur_taux = consigne.taux_lacet_cible - taux_lacet_mesure;
        self.pid_taux.mettre_a_jour(erreur_taux, dt)
    }

    /// Remet à zéro l'état interne du PID (voir `doc/controle.md`, gel des
    /// contrôleurs inactifs).
    pub fn reinitialiser(&mut self) {
        self.pid_taux.reinitialiser();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn commande_nulle_quand_taux_atteint() {
        let mut controleur = ControleurLacet::nouveau(1.0, 0.0, 0.0, 1.0);
        let consigne = ConsigneLacet { taux_lacet_cible: 0.0 };
        let commande = controleur.mettre_a_jour(&consigne, 0.0, 0.01);
        assert!(commande.abs() < 1e-6);
    }

    #[test]
    fn amortit_un_derapage_positif() {
        // Taux de lacet mesuré positif alors que la cible est nulle (pur
        // amortissement) : la commande doit s'opposer au dérapage.
        // Sens physique réel (quel câblage tourne à droite/gauche) : à
        // déterminer au calibrage matériel, comme pour les autres contrôleurs.
        let mut controleur = ControleurLacet::nouveau(1.0, 0.0, 0.0, 1.0);
        let consigne = ConsigneLacet { taux_lacet_cible: 0.0 };
        let commande = controleur.mettre_a_jour(&consigne, 0.5, 0.01);
        assert!(commande < 0.0);
    }

    #[test]
    fn commande_de_signe_oppose_pour_derapage_negatif() {
        let mut controleur = ControleurLacet::nouveau(1.0, 0.0, 0.0, 1.0);
        let consigne = ConsigneLacet { taux_lacet_cible: 0.0 };
        let commande = controleur.mettre_a_jour(&consigne, -0.5, 0.01);
        assert!(commande > 0.0);
    }

    #[test]
    fn suit_une_consigne_de_taux_non_nulle() {
        // Avec une consigne non nulle (feedforward futur), seul l'écart au
        // taux cible compte, pas le taux mesuré en absolu.
        let mut controleur = ControleurLacet::nouveau(1.0, 0.0, 0.0, 1.0);
        let consigne = ConsigneLacet { taux_lacet_cible: 0.3 };
        let commande = controleur.mettre_a_jour(&consigne, 0.3, 0.01);
        assert!(commande.abs() < 1e-6);
    }

    #[test]
    fn saturation_a_grand_ecart_de_taux() {
        let mut controleur = ControleurLacet::nouveau(10.0, 0.0, 0.0, 1.0);
        let consigne = ConsigneLacet { taux_lacet_cible: 0.0 };
        let commande = controleur.mettre_a_jour(&consigne, 5.0, 0.01);
        assert_eq!(commande, -1.0);
    }

    #[test]
    fn reinitialiser_efface_l_integrale() {
        let mut controleur = ControleurLacet::nouveau(0.0, 1.0, 0.0, 10.0);
        let consigne = ConsigneLacet { taux_lacet_cible: 0.0 };
        controleur.mettre_a_jour(&consigne, 0.5, 1.0);
        controleur.reinitialiser();
        // sans réinitialisation, l'intégrale accumulée aurait influencé cet appel
        let commande = controleur.mettre_a_jour(&consigne, 0.0, 1.0);
        assert!(commande.abs() < 1e-6);
    }
}
