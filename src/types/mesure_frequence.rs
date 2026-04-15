// src/types/mesure_frequence.rs
//! Type représentant les statistiques de fréquence d'échantillonnage d'un capteur.
//!
//! Utilisé à la fois dans les diagnostics (vérification pré-vol) et comme
//! paramètre de vol surveillé en continu. Le filtre de Kalman se servira
//! du champ `hz_moyen` pour calculer son `dt` de façon adaptive.

/// Statistiques de fréquence d'échantillonnage mesurées expérimentalement
///
/// Produit par les fonctions `test_frequence_*` de chaque module de diagnostic.
///
/// # Relation avec le filtre de Kalman
///
/// Le `dt` utilisé dans l'équation de prédiction doit refléter la fréquence
/// *réelle* du capteur, pas sa fréquence nominale. Ce type capture l'écart
/// entre les deux et fournit les données nécessaires.
///
/// # Exemple
///
/// ```ignore
/// let stats = diag_mpu9250::test_frequence(i2c, 100)?;
/// println!("MPU9250: {:.1} Hz (jitter ±{:.2} ms)", stats.hz_moyen, stats.jitter_ms);
/// // Utilisation dans Kalman :
/// let dt = 1.0 / stats.hz_moyen;
/// ```
#[derive(Debug, Clone, Copy)]
pub struct MesureFrequence {
    /// Nom du capteur mesuré (pour affichage)
    pub capteur: &'static str,

    /// Fréquence d'échantillonnage moyenne mesurée (Hz)
    ///
    /// Valeur principale à utiliser pour le `dt` du filtre de Kalman.
    pub hz_moyen: f32,

    /// Fréquence minimale observée sur l'ensemble des mesures (Hz)
    pub hz_min: f32,

    /// Fréquence maximale observée sur l'ensemble des mesures (Hz)
    pub hz_max: f32,

    /// Gigue temporelle : écart-type des intervalles entre mesures (ms)
    ///
    /// Un jitter faible indique une cadence régulière, favorable à Kalman.
    /// Un jitter élevé (> 20% de 1/hz_moyen) devrait alerter.
    pub jitter_ms: f32,

    /// Nombre de mesures réussies lors du test
    pub n_mesures: usize,

    /// Nombre de mesures en erreur (timeout, I²C, etc.)
    pub n_erreurs: usize,
}

impl MesureFrequence {
    /// Retourne `true` si la fréquence est dans la plage acceptable
    ///
    /// # Arguments
    /// * `hz_cible` - Fréquence nominale attendue du capteur
    /// * `tolerance_pct` - Tolérance en pourcentage (ex: 10.0 pour ±10%)
    pub fn est_dans_tolerance(&self, hz_cible: f32, tolerance_pct: f32) -> bool {
        let ecart = ((self.hz_moyen - hz_cible) / hz_cible * 100.0).abs();
        ecart <= tolerance_pct
    }

    /// Retourne `true` si le taux d'erreur est acceptable (< 5%)
    pub fn taux_erreur_acceptable(&self) -> bool {
        let total = self.n_mesures + self.n_erreurs;
        if total == 0 {
            return false;
        }
        (self.n_erreurs as f32 / total as f32) < 0.05
    }

    /// Affiche un résumé formaté sur stdout
    pub fn afficher_resume(&self) {
        println!("\n--- Fréquence {} ---", self.capteur);
        println!(
            "  Moyenne : {:>8.2} Hz  (dt = {:.3} ms)",
            self.hz_moyen,
            if self.hz_moyen > 0.0 { 1000.0 / self.hz_moyen } else { f32::INFINITY }
        );
        println!(
            "  Min/Max : {:>8.2} / {:.2} Hz",
            self.hz_min, self.hz_max
        );
        println!("  Jitter  : {:>8.3} ms", self.jitter_ms);
        println!(
            "  Succès  : {}/{} ({:.1}%)",
            self.n_mesures,
            self.n_mesures + self.n_erreurs,
            if self.n_mesures + self.n_erreurs > 0 {
                self.n_mesures as f32 / (self.n_mesures + self.n_erreurs) as f32 * 100.0
            } else {
                0.0
            }
        );
    }
}

/// Calcule les statistiques de fréquence à partir d'une liste d'intervalles (µs)
///
/// Fonction utilitaire partagée par tous les modules de diagnostic.
/// Prend en entrée les intervalles entre mesures successives en microsecondes.
pub fn calculer_stats(
    capteur: &'static str,
    intervalles_us: &[u64],
    n_erreurs: usize,
) -> MesureFrequence {
    if intervalles_us.is_empty() {
        return MesureFrequence {
            capteur,
            hz_moyen: 0.0,
            hz_min: 0.0,
            hz_max: 0.0,
            jitter_ms: 0.0,
            n_mesures: 0,
            n_erreurs,
        };
    }

    let n = intervalles_us.len() as f32;
    let somme: u64 = intervalles_us.iter().sum();
    let moyenne_us = somme as f32 / n;

    let min_us = *intervalles_us.iter().min().unwrap() as f32;
    let max_us = *intervalles_us.iter().max().unwrap() as f32;

    // Écart-type des intervalles (jitter)
    let variance = intervalles_us
        .iter()
        .map(|&v| {
            let diff = v as f32 - moyenne_us;
            diff * diff
        })
        .sum::<f32>()
        / n;
    let ecart_type_us = variance.sqrt();

    MesureFrequence {
        capteur,
        hz_moyen: if moyenne_us > 0.0 { 1_000_000.0 / moyenne_us } else { 0.0 },
        hz_min: if max_us > 0.0 { 1_000_000.0 / max_us } else { 0.0 },
        hz_max: if min_us > 0.0 { 1_000_000.0 / min_us } else { 0.0 },
        jitter_ms: ecart_type_us / 1000.0,
        n_mesures: intervalles_us.len(),
        n_erreurs,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_stats_regulieres() {
        // 100 intervalles parfaitement réguliers à 5ms = 200 Hz
        let intervalles: Vec<u64> = vec![5000; 100];
        let stats = calculer_stats("test", &intervalles, 0);

        assert!((stats.hz_moyen - 200.0).abs() < 0.1);
        assert!(stats.jitter_ms < 0.001);
        assert_eq!(stats.n_mesures, 100);
        assert_eq!(stats.n_erreurs, 0);
    }

    #[test]
    fn test_tolerance() {
        let stats = MesureFrequence {
            capteur: "test",
            hz_moyen: 195.0,
            hz_min: 190.0,
            hz_max: 200.0,
            jitter_ms: 0.5,
            n_mesures: 100,
            n_erreurs: 0,
        };
        // 195 Hz vs 200 Hz cible → écart 2.5% → dans 10%
        assert!(stats.est_dans_tolerance(200.0, 10.0));
        // Trop serré
        assert!(!stats.est_dans_tolerance(200.0, 1.0));
    }

    #[test]
    fn test_taux_erreur() {
        let stats = MesureFrequence {
            capteur: "test",
            hz_moyen: 200.0,
            hz_min: 190.0,
            hz_max: 210.0,
            jitter_ms: 1.0,
            n_mesures: 97,
            n_erreurs: 3,
        };
        // 3% d'erreurs → acceptable
        assert!(stats.taux_erreur_acceptable());
    }
}
