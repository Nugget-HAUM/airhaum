// src/estimation/attitude.rs
//! Estimation de l'attitude par filtre complémentaire
//!
//! Le filtre complémentaire fusionne deux sources complémentaires :
//!
//! - **Gyroscope** : précis à court terme, dérive à long terme
//! - **Accéléromètre** : référence absolue (gravité), bruité en vol accéléré
//!
//! La fusion pondère les deux par le coefficient `alpha` :
//!
//! ```text
//! roulis  = α × (roulis  + gx × dt) + (1−α) × atan2(ay, az)
//! tangage = α × (tangage + gy × dt) + (1−α) × atan2(−ax, √(ay²+az²))
//! lacet   =      lacet   + gz × dt          (gyro seul — magnétomètre requis)
//! ```
//!
//! # Limites de cette implémentation
//!
//! - **Gimbal lock** à ±90° de tangage (angles d'Euler). Acceptable pour un
//!   avion en vol normal (enveloppe ±45° tangage, ±60° roulis). L'EKF utilisera
//!   des quaternions et lèvera cette limite.
//! - **Lacet** : sans correction magnétomètre, le lacet dérive avec le gyroscope.
//! - **Vol accéléré** : la correction accéléromètre est invalide si la norme
//!   de l'accélération s'écarte significativement de g. L'EKF gérera ce cas.
//!
//! # Remplacement par l'EKF (étape E)
//!
//! L'EKF implémentera le même trait `EstimateurAttitude` (à créer en étape E).
//! Aucun changement dans les couches amont ne sera nécessaire.

use crate::types::Angle;
use crate::capteurs::traitement_imu::MesureImuTraitee;

// ─────────────────────────────────────────────────────────────────────────────
// Constantes
// ─────────────────────────────────────────────────────────────────────────────

/// Coefficient du filtre complémentaire par défaut.
///
/// α proche de 1 → on fait confiance au gyroscope, correction accéléromètre lente.
/// Constante de temps : τ = α × dt / (1−α)
///   à dt = 5 ms  → τ ≈ 0.25 s
///   à dt = 10 ms → τ ≈ 0.49 s
pub const ALPHA_DEFAUT: f32 = 0.98;

/// Seuil d'écart à g en-deçà duquel on accepte la correction accéléromètre.
///
/// Si la norme de l'accélération est trop éloignée de 9.81 m/s²
/// (virage serré, turbulence), la correction accel est ignorée pour ce cycle.
const SEUIL_ACCEL_G: f32 = 0.3; // ±0.3 m/s² autour de 9.81

/// Norme de l'accélération de référence (m/s²)
const G: f32 = 9.80665;

// ─────────────────────────────────────────────────────────────────────────────
// Type de sortie
// ─────────────────────────────────────────────────────────────────────────────

/// Attitude estimée de l'appareil.
///
/// Convention d'axes (NED, corps) :
/// - X : vers l'avant (nez)
/// - Y : vers la droite
/// - Z : vers le bas
///
/// - `roulis`  positif : aile droite vers le bas
/// - `tangage` positif : nez vers le haut
/// - `lacet`   positif : nez vers la droite
#[derive(Debug, Clone, Copy)]
pub struct Attitude {
    pub roulis:  Angle,
    pub tangage: Angle,
    pub lacet:   Angle,
}

impl Attitude {
    /// Attitude nulle (appareil de niveau, cap nul).
    pub fn nulle() -> Self {
        Self {
            roulis:  Angle::depuis_radians(0.0),
            tangage: Angle::depuis_radians(0.0),
            lacet:   Angle::depuis_radians(0.0),
        }
    }
}

impl std::fmt::Display for Attitude {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f,
            "roulis={:+6.1}°  tangage={:+6.1}°  lacet={:+7.1}°",
            self.roulis.degres(),
            self.tangage.degres(),
            self.lacet.degres(),
        )
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Filtre complémentaire
// ─────────────────────────────────────────────────────────────────────────────

/// Filtre complémentaire gyroscope + accéléromètre.
pub struct FiltreComplementaire {
    attitude:   Attitude,
    alpha:      f32,
    initialise: bool,
}

impl FiltreComplementaire {
    /// Crée un filtre avec le coefficient `alpha` fourni.
    pub fn nouveau(alpha: f32) -> Self {
        assert!(alpha > 0.0 && alpha < 1.0, "alpha doit être dans ]0, 1[");
        Self {
            attitude:   Attitude::nulle(),
            alpha,
            initialise: false,
        }
    }

    /// Crée un filtre avec le coefficient par défaut ([`ALPHA_DEFAUT`]).
    pub fn avec_alpha_defaut() -> Self {
        Self::nouveau(ALPHA_DEFAUT)
    }

    /// Met à jour l'attitude depuis une mesure IMU prétraitée.
    ///
    /// Retourne une référence vers l'attitude courante mise à jour.
    pub fn mettre_a_jour(&mut self, mesure: &MesureImuTraitee) -> &Attitude {
        let a = mesure.donnees.accelerometre;
        let g_vec = mesure.donnees.gyroscope;

        // ── Initialisation : première mesure, attitude depuis accel ──────────
        if !self.initialise {
            self.attitude = Self::attitude_depuis_accel(a.x, a.y, a.z);
            self.initialise = true;
            return &self.attitude;
        }

        let dt = match mesure.dt_s {
            Some(d) if d > 0.0 && d < 1.0 => d, // dt incohérent → cycle ignoré
            _ => return &self.attitude,
        };

        // ── Prédiction gyroscope ─────────────────────────────────────────────
        let roulis_gyro  = self.attitude.roulis.radians()  + g_vec.x * dt;
        let tangage_gyro = self.attitude.tangage.radians() + g_vec.y * dt;
        let lacet_gyro   = self.attitude.lacet.radians()   + g_vec.z * dt;

        // ── Correction accéléromètre ──────────────────────────────────────────
        // Ignorée si la norme de l'accélération s'éloigne trop de g
        // (virage serré, turbulence).
        let norme_accel = a.norme();
        let attitude_accel = if (norme_accel - G).abs() <= SEUIL_ACCEL_G {
            Some(Self::attitude_depuis_accel(a.x, a.y, a.z))
        } else {
            None
        };

        // ── Fusion ───────────────────────────────────────────────────────────
        let (roulis, tangage) = match attitude_accel {
            Some(ref accel) => (
                self.alpha * roulis_gyro  + (1.0 - self.alpha) * accel.roulis.radians(),
                self.alpha * tangage_gyro + (1.0 - self.alpha) * accel.tangage.radians(),
            ),
            None => (roulis_gyro, tangage_gyro), // gyro seul
        };

        self.attitude = Attitude {
            roulis:  Angle::depuis_radians(roulis),
            tangage: Angle::depuis_radians(tangage),
            lacet:   Angle::depuis_radians(lacet_gyro), // pas de correction lacet
        };

        &self.attitude
    }

    /// Retourne l'attitude courante sans la mettre à jour.
    pub fn attitude(&self) -> &Attitude {
        &self.attitude
    }

    // ── Utilitaires ──────────────────────────────────────────────────────────

    /// Calcule l'attitude depuis l'accéléromètre seul.
    ///
    /// Valide uniquement à l'arrêt ou en vol stabilisé (gravité dominante).
    fn attitude_depuis_accel(ax: f32, ay: f32, az: f32) -> Attitude {
        Attitude {
            roulis:  Angle::depuis_radians(ay.atan2(az)),
            tangage: Angle::depuis_radians((-ax).atan2((ay * ay + az * az).sqrt())),
            lacet:   Angle::depuis_radians(0.0), // indéfini sans magnétomètre
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Tests
// ─────────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::{Vector3, Temperature, Horodatage, DonneesImu};
    use crate::capteurs::traitement_imu::MesureImuTraitee;

    const EPSILON_DEG: f32 = 0.5; // tolérance en degrés pour les tests

    fn mesure(ax: f32, ay: f32, az: f32, gx: f32, gy: f32, gz: f32, dt_s: f32) -> MesureImuTraitee {
        MesureImuTraitee {
            donnees: DonneesImu {
                horodatage:    Horodatage::maintenant(),
                accelerometre: Vector3::nouveau(ax, ay, az),
                gyroscope:     Vector3::nouveau(gx, gy, gz),
                magnetometre:  Vector3::zero(),
                temperature:   Temperature::depuis_celsius(25.0),
            },
            dt_s: Some(dt_s),
        }
    }

    fn mesure_init(ax: f32, ay: f32, az: f32) -> MesureImuTraitee {
        MesureImuTraitee {
            donnees: DonneesImu {
                horodatage:    Horodatage::maintenant(),
                accelerometre: Vector3::nouveau(ax, ay, az),
                gyroscope:     Vector3::zero(),
                magnetometre:  Vector3::zero(),
                temperature:   Temperature::depuis_celsius(25.0),
            },
            dt_s: None, // première mesure
        }
    }

    #[test]
    fn board_plat_roulis_tangage_nuls() {
        let mut f = FiltreComplementaire::avec_alpha_defaut();
        let att = f.mettre_a_jour(&mesure_init(0.0, 0.0, G));
        assert!(att.roulis.degres().abs()  < EPSILON_DEG, "roulis={}", att.roulis.degres());
        assert!(att.tangage.degres().abs() < EPSILON_DEG, "tangage={}", att.tangage.degres());
    }

    #[test]
    fn board_incline_roulis_30deg() {
        let mut f = FiltreComplementaire::avec_alpha_defaut();
        let angle = 30_f32.to_radians();
        // Aile droite vers le bas de 30° : ay = G*sin(30°), az = G*cos(30°)
        let att = f.mettre_a_jour(&mesure_init(0.0, G * angle.sin(), G * angle.cos()));
        assert!((att.roulis.degres() - 30.0).abs() < EPSILON_DEG,
            "roulis attendu 30°, obtenu {:.2}°", att.roulis.degres());
    }

    #[test]
    fn board_incline_tangage_20deg() {
        let mut f = FiltreComplementaire::avec_alpha_defaut();
        let angle = 20_f32.to_radians();
        // Nez vers le haut de 20° : ax = -G*sin(20°), az = G*cos(20°)
        let att = f.mettre_a_jour(&mesure_init(-G * angle.sin(), 0.0, G * angle.cos()));
        assert!((att.tangage.degres() - 20.0).abs() < EPSILON_DEG,
            "tangage attendu 20°, obtenu {:.2}°", att.tangage.degres());
    }

    #[test]
    fn gyro_integre_le_lacet() {
        let mut f = FiltreComplementaire::avec_alpha_defaut();
        f.mettre_a_jour(&mesure_init(0.0, 0.0, G)); // init
        // 90°/s pendant 1s = 90° de lacet
        let vitesse_lacet = 90_f32.to_radians(); // rad/s
        let dt = 0.005_f32; // 5 ms
        let n = (1.0 / dt) as usize;
        let mut att = Attitude::nulle();
        for _ in 0..n {
            att = *f.mettre_a_jour(&mesure(0.0, 0.0, G, 0.0, 0.0, vitesse_lacet, dt));
        }
        assert!((att.lacet.degres() - 90.0).abs() < 1.0,
            "lacet attendu 90°, obtenu {:.2}°", att.lacet.degres());
    }

    #[test]
    fn correction_accel_ignoree_si_vol_accelere() {
        // Norme accel = 20 m/s² (virage serré) → correction ignorée
        let mut f = FiltreComplementaire::avec_alpha_defaut();
        f.mettre_a_jour(&mesure_init(0.0, 0.0, G)); // init à plat
        let att = f.mettre_a_jour(&mesure(0.0, 0.0, 20.0, 0.0, 0.0, 0.0, 0.005));
        // Pas de gyro, donc attitude ne change pas
        assert!(att.roulis.degres().abs()  < EPSILON_DEG);
        assert!(att.tangage.degres().abs() < EPSILON_DEG);
    }

    #[test]
    fn convergence_vers_angle_accel() {
        // Inclinaison réelle 45°, filtre initialisé à 0° → doit converger
        let mut f = FiltreComplementaire::avec_alpha_defaut();
        f.mettre_a_jour(&mesure_init(0.0, 0.0, G)); // init à plat
        let angle = 45_f32.to_radians();
        let ay_cible = G * angle.sin();
        let az_cible = G * angle.cos();
        // 500 cycles à 5ms = 2.5s de convergence
        let mut att = Attitude::nulle();
        for _ in 0..500 {
            att = *f.mettre_a_jour(&mesure(0.0, ay_cible, az_cible, 0.0, 0.0, 0.0, 0.005));
        }
        assert!((att.roulis.degres() - 45.0).abs() < 1.0,
            "convergence attendue vers 45°, obtenu {:.2}°", att.roulis.degres());
    }
}
