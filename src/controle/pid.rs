// src/controle/pid.rs
//! PID générique, réutilisé par tous les contrôleurs d'axe (`controle/`).
//!
//! Voir `doc/controle.md` : limitation de l'intégrale par clamp simple
//! (borne `limite_integrale`), pas de back-calculation ; pas de
//! feedforward ni de mise à l'échelle selon la vitesse air pour cette
//! première étape.

/// Contrôleur PID générique sur une seule grandeur scalaire.
///
/// Le pas de temps `dt` est toujours passé explicitement à
/// [`Pid::mettre_a_jour`] (jamais supposé constant), cohérent avec
/// `doc/gestion_du_temps.md`.
#[derive(Debug, Clone)]
pub struct Pid {
    kp: f32,
    ki: f32,
    kd: f32,
    limite_integrale: f32,
    sortie_min: f32,
    sortie_max: f32,
    integrale: f32,
    derniere_erreur: Option<f32>,
}

impl Pid {
    /// Crée un nouveau PID.
    ///
    /// - `limite_integrale` : borne de la valeur accumulée par le terme
    ///   intégral (avant multiplication par `ki`), symétrique. Doit être
    ///   positive.
    /// - `sortie_min`/`sortie_max` : bornes de la commande retournée par
    ///   [`Pid::mettre_a_jour`].
    pub fn nouveau(
        kp: f32,
        ki: f32,
        kd: f32,
        limite_integrale: f32,
        sortie_min: f32,
        sortie_max: f32,
    ) -> Self {
        Self {
            kp,
            ki,
            kd,
            limite_integrale: limite_integrale.abs(),
            sortie_min,
            sortie_max,
            integrale: 0.0,
            derniere_erreur: None,
        }
    }

    /// Calcule la commande à partir de l'erreur courante (consigne − mesure)
    /// et du pas de temps écoulé depuis le dernier appel.
    ///
    /// `dt` doit être strictement positif ; un `dt` nul ou négatif désactive
    /// l'intégration et la dérivation pour ce cycle (seul le terme
    /// proportionnel est appliqué), afin de rester robuste à une horloge
    /// dégradée plutôt que de produire une valeur aberrante.
    pub fn mettre_a_jour(&mut self, erreur: f32, dt: f32) -> f32 {
        let proportionnel = self.kp * erreur;

        let (integral, derivee) = if dt > 0.0 {
            self.integrale = (self.integrale + erreur * dt)
                .clamp(-self.limite_integrale, self.limite_integrale);

            let taux = match self.derniere_erreur {
                Some(precedente) => (erreur - precedente) / dt,
                None => 0.0,
            };
            self.derniere_erreur = Some(erreur);

            (self.ki * self.integrale, self.kd * taux)
        } else {
            (self.ki * self.integrale, 0.0)
        };

        (proportionnel + integral + derivee).clamp(self.sortie_min, self.sortie_max)
    }

    /// Remet à zéro l'état interne (intégrale et mémoire de dérivée).
    ///
    /// À appeler par l'appelant lors d'un changement de contexte où
    /// l'historique du PID n'a plus de sens (ex. contrôleur mis en veille
    /// puis réactivé — voir la note sur le gel des contrôleurs inactifs
    /// dans `doc/controle.md`).
    pub fn reinitialiser(&mut self) {
        self.integrale = 0.0;
        self.derniere_erreur = None;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn proportionnel_seul() {
        let mut pid = Pid::nouveau(2.0, 0.0, 0.0, 10.0, -100.0, 100.0);
        assert_eq!(pid.mettre_a_jour(3.0, 0.01), 6.0);
    }

    #[test]
    fn integrale_accumule_dans_le_temps() {
        let mut pid = Pid::nouveau(0.0, 1.0, 0.0, 100.0, -100.0, 100.0);
        pid.mettre_a_jour(2.0, 1.0);
        let sortie = pid.mettre_a_jour(2.0, 1.0);
        // intégrale = 2*1 + 2*1 = 4, ki = 1 → sortie = 4
        assert!((sortie - 4.0).abs() < 1e-6);
    }

    #[test]
    fn anti_windup_clamp_intégrale() {
        let mut pid = Pid::nouveau(0.0, 1.0, 0.0, 5.0, -100.0, 100.0);
        for _ in 0..100 {
            pid.mettre_a_jour(10.0, 1.0);
        }
        // l'intégrale ne doit jamais dépasser la limite fixée
        let sortie = pid.mettre_a_jour(10.0, 1.0);
        assert!((sortie - 5.0).abs() < 1e-6);
    }

    #[test]
    fn saturation_sortie() {
        let mut pid = Pid::nouveau(10.0, 0.0, 0.0, 100.0, -1.0, 1.0);
        assert_eq!(pid.mettre_a_jour(5.0, 0.01), 1.0);
        assert_eq!(pid.mettre_a_jour(-5.0, 0.01), -1.0);
    }

    #[test]
    fn pas_de_a_coup_derivee_au_premier_appel() {
        let mut pid = Pid::nouveau(0.0, 0.0, 1.0, 10.0, -100.0, 100.0);
        // premier appel : pas d'erreur précédente connue → dérivée nulle
        assert_eq!(pid.mettre_a_jour(50.0, 0.01), 0.0);
    }

    #[test]
    fn derivee_reagit_a_la_variation_erreur() {
        let mut pid = Pid::nouveau(0.0, 0.0, 2.0, 10.0, -100.0, 100.0);
        pid.mettre_a_jour(0.0, 1.0);
        let sortie = pid.mettre_a_jour(3.0, 1.0);
        // taux = (3-0)/1 = 3, kd = 2 → sortie = 6
        assert!((sortie - 6.0).abs() < 1e-6);
    }

    #[test]
    fn dt_non_positif_desactive_integrale_et_derivee() {
        let mut pid = Pid::nouveau(1.0, 1.0, 1.0, 10.0, -100.0, 100.0);
        let sortie = pid.mettre_a_jour(5.0, 0.0);
        // seul le proportionnel s'applique : 1.0 * 5.0
        assert_eq!(sortie, 5.0);
    }

    #[test]
    fn reinitialiser_efface_integrale_et_derivee() {
        let mut pid = Pid::nouveau(0.0, 1.0, 1.0, 10.0, -100.0, 100.0);
        pid.mettre_a_jour(5.0, 1.0);
        pid.reinitialiser();
        // sans réinitialisation, l'intégrale accumulée (5.0) et la dérivée
        // (basée sur l'erreur précédente) auraient influencé cet appel
        let sortie = pid.mettre_a_jour(0.0, 1.0);
        assert_eq!(sortie, 0.0);
    }
}
