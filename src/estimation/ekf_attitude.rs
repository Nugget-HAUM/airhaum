// src/estimation/ekf_attitude.rs
//! Filtre de Kalman étendu pour l'estimation d'attitude.
//!
//! Remplace `FiltreComplementaire` à l'étape [E]. Interface identique :
//! même signature `mettre_a_jour`, même type de sortie `Attitude`.
//!
//! # État
//!
//! Quaternion unitaire `q = [qw, qx, qy, qz]` — rotation du repère inertiel
//! (NED) vers le repère corps. Identité = carte posée à plat.
//!
//! # Modèle de processus (gyroscope)
//!
//! ```text
//! q_{k+1} = q_k ⊗ Δq(ω, dt)          (intégration exacte)
//! Jacobien : F ≈ I + (dt/2) × Ω_mat(ω)
//! Bruit    : Q = q_bruit × dt × I₄
//! ```
//!
//! # Modèle de mesure (accéléromètre)
//!
//! ```text
//! h(q) = gravité attendue dans le repère corps = q* ⊗ [0,0,G] ⊗ q
//! Jacobien : H (3×4) = ∂h/∂q
//! Bruit    : R = r_bruit × I₃
//! ```
//!
//! # Limites
//!
//! - Correction accéléromètre désactivée si |a − G| > `SEUIL_ACCEL`.
//! - Lacet : seul le gyroscope contribue (pas de magnétomètre à ce stade).
//! - EKF additif avec renormalisation (non-singularité garantie par la
//!   petitesse des corrections).

use crate::capteurs::traitement_imu::MesureImuTraitee;
use crate::estimation::attitude::Attitude;
use crate::types::{Quaternion, Vector3};

// ─────────────────────────────────────────────────────────────────────────────
// Constantes
// ─────────────────────────────────────────────────────────────────────────────

const G: f32 = 9.80665;
/// Seuil de rejet accéléromètre : |‖a‖ − G| > SEUIL_ACCEL → pas de correction.
const SEUIL_ACCEL: f32 = 0.3;
/// Variance du bruit process par seconde (rad²/s par s de propagation).
const Q_BRUIT_DEFAUT: f32 = 1e-4;
/// Variance du bruit mesure accéléromètre ((m/s²)²).
const R_BRUIT_DEFAUT: f32 = 0.09;

// ─────────────────────────────────────────────────────────────────────────────
// Type aliases (algèbre linéaire locale)
// ─────────────────────────────────────────────────────────────────────────────

type M44 = [[f32; 4]; 4]; // 4 lignes × 4 colonnes
type M34 = [[f32; 4]; 3]; // 3 lignes × 4 colonnes
type M43 = [[f32; 3]; 4]; // 4 lignes × 3 colonnes
type M33 = [[f32; 3]; 3]; // 3 lignes × 3 colonnes
type V4  = [f32; 4];
type V3  = [f32; 3];

// ─────────────────────────────────────────────────────────────────────────────
// Structure
// ─────────────────────────────────────────────────────────────────────────────

/// Filtre de Kalman étendu pour l'attitude.
pub struct EkfAttitude {
    q:          Quaternion, // Quaternion d'orientation courant
    p:          M44,        // Matrice de covariance
    q_bruit:    f32,        // Variance bruit process
    r_bruit:    f32,        // Variance bruit mesure
    attitude:   Attitude,   // Dernière attitude calculée (sortie)
    initialise: bool,
}

// ─────────────────────────────────────────────────────────────────────────────
// API publique
// ─────────────────────────────────────────────────────────────────────────────

impl EkfAttitude {
    /// Crée un EKF avec les paramètres de bruit par défaut.
    pub fn nouveau() -> Self {
        Self {
            q:          Quaternion::identite(),
            p:          identite44(),
            q_bruit:    Q_BRUIT_DEFAUT,
            r_bruit:    R_BRUIT_DEFAUT,
            attitude:   Attitude::nulle(),
            initialise: false,
        }
    }

    /// Met à jour l'estimation d'attitude depuis une mesure IMU prétraitée.
    ///
    /// - Première mesure (`dt_s = None`) : initialisation depuis l'accéléromètre.
    /// - Mesures suivantes : prédiction gyroscope + correction accéléromètre.
    pub fn mettre_a_jour(&mut self, mesure: &MesureImuTraitee) -> &Attitude {
        let a     = mesure.donnees.accelerometre;
        let omega = mesure.donnees.gyroscope;

        match mesure.dt_s {
            None => {
                self.initialiser_depuis_accel(a);
            }
            Some(dt) if dt > 0.0 => {
                self.predire(omega, dt);
                self.corriger(a);
            }
            _ => {
                // dt invalide (nul ou négatif) : correction seule
                self.corriger(a);
            }
        }

        self.attitude_depuis_quat();
        &self.attitude
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Implémentation interne
// ─────────────────────────────────────────────────────────────────────────────

impl EkfAttitude {
    /// Initialise le quaternion depuis l'accéléromètre (même formule que le FC).
    fn initialiser_depuis_accel(&mut self, a: Vector3) {
        let roulis  = a.y.atan2(a.z);
        let tangage = (-a.x).atan2((a.y * a.y + a.z * a.z).sqrt());

        // Euler ZYX → Quaternion (lacet initial = 0)
        let cr = (roulis  / 2.0).cos();  let sr = (roulis  / 2.0).sin();
        let cp = (tangage / 2.0).cos();  let sp = (tangage / 2.0).sin();
        self.q = Quaternion::nouveau(cp * cr, cp * sr, sp * cr, -sp * sr).normalise();
        self.p = identite44();
        self.initialise = true;
    }

    /// Étape de prédiction : intégration gyroscope + propagation covariance.
    fn predire(&mut self, omega: Vector3, dt: f32) {
        // État
        self.q = self.q.multiplier(&Quaternion::depuis_gyroscope(omega, dt)).normalise();

        // Covariance  P ← F·P·Fᵀ + Q
        let f    = omega_vers_f(omega, dt);
        let fpft = mult44(&mult44(&f, &self.p), &transposer44(&f));
        self.p   = ajouter44(&fpft, &diag44(self.q_bruit * dt));
    }

    /// Étape de correction : mise à jour accéléromètre via gain de Kalman.
    fn corriger(&mut self, a: Vector3) {
        // Rejet vol accéléré
        let norme_a = (a.x * a.x + a.y * a.y + a.z * a.z).sqrt();
        if (norme_a - G).abs() > SEUIL_ACCEL { return; }

        // Innovation  y = a_mesuré − h(q)
        let g_attendu = self.q.monde_vers_corps(Vector3::nouveau(0.0, 0.0, G));
        let y: V3 = [a.x - g_attendu.x, a.y - g_attendu.y, a.z - g_attendu.z];

        // Jacobien H (3×4)
        let h  = jacobien_h(&self.q);
        let ht = transposer34(&h);

        // S = H·P·Hᵀ + R·I₃
        let s = ajouter_diag33(&mult34_43(&mult34_44(&h, &self.p), &ht), self.r_bruit);
        let s_inv = match inverser33(&s) { Some(m) => m, None => return };

        // Gain de Kalman  K = P·Hᵀ·S⁻¹
        let k = mult43_33(&mult44_43(&self.p, &ht), &s_inv);

        // Correction état  q ← normaliser(q + K·y)
        let delta = mult43_v3(&k, &y);
        self.q = Quaternion::nouveau(
            self.q.w + delta[0], self.q.x + delta[1],
            self.q.y + delta[2], self.q.z + delta[3],
        ).normalise();

        // Correction covariance  P ← (I − K·H)·P
        self.p = mult44(&soustraire_identite44(&mult43_34(&k, &h)), &self.p);
    }

    /// Extrait l'attitude courante depuis le quaternion.
    fn attitude_depuis_quat(&mut self) {
        let (roulis, tangage, lacet) = self.q.vers_euler();
        self.attitude = Attitude { roulis, tangage, lacet };
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Algèbre linéaire (fonctions privées)
// ─────────────────────────────────────────────────────────────────────────────

fn identite44() -> M44 {
    let mut m = [[0.0_f32; 4]; 4];
    for i in 0..4 { m[i][i] = 1.0; }
    m
}

fn diag44(s: f32) -> M44 {
    let mut m = [[0.0_f32; 4]; 4];
    for i in 0..4 { m[i][i] = s; }
    m
}

fn mult44(a: &M44, b: &M44) -> M44 {
    let mut r = [[0.0_f32; 4]; 4];
    for i in 0..4 { for j in 0..4 { for k in 0..4 { r[i][j] += a[i][k] * b[k][j]; } } }
    r
}

fn transposer44(m: &M44) -> M44 {
    let mut r = [[0.0_f32; 4]; 4];
    for i in 0..4 { for j in 0..4 { r[i][j] = m[j][i]; } }
    r
}

fn ajouter44(a: &M44, b: &M44) -> M44 {
    let mut r = [[0.0_f32; 4]; 4];
    for i in 0..4 { for j in 0..4 { r[i][j] = a[i][j] + b[i][j]; } }
    r
}

/// I − m
fn soustraire_identite44(m: &M44) -> M44 {
    let mut r = identite44();
    for i in 0..4 { for j in 0..4 { r[i][j] -= m[i][j]; } }
    r
}

/// Jacobien F ≈ I + (dt/2)·Ω_mat(ω).
///
/// Dérivé de la cinématique quaternion q_{k+1} = q_k ⊗ Δq(ω, dt).
fn omega_vers_f(w: Vector3, dt: f32) -> M44 {
    let h = dt * 0.5;
    [
        [ 1.0,     -h*w.x, -h*w.y, -h*w.z],
        [ h*w.x,   1.0,    h*w.z,  -h*w.y],
        [ h*w.y,  -h*w.z,  1.0,    h*w.x ],
        [ h*w.z,   h*w.y, -h*w.x,  1.0   ],
    ]
}

/// Jacobien H = ∂h(q)/∂q  (3×4) où h(q) = gravité dans le repère corps.
///
/// ```text
/// g_corps.x = 2G(qx·qz − qw·qy)
/// g_corps.y = 2G(qy·qz + qw·qx)
/// g_corps.z =  G(1 − 2(qx² + qy²))
/// ```
fn jacobien_h(q: &Quaternion) -> M34 {
    let g2 = 2.0 * G;
    [
        [-g2 * q.y,     g2 * q.z,      -g2 * q.w,     g2 * q.x ],
        [ g2 * q.x,     g2 * q.w,       g2 * q.z,     g2 * q.y ],
        [ 0.0,         -4.0 * G * q.x, -4.0 * G * q.y, 0.0     ],
    ]
}

fn transposer34(h: &M34) -> M43 {
    let mut r = [[0.0_f32; 3]; 4];
    for i in 0..3 { for j in 0..4 { r[j][i] = h[i][j]; } }
    r
}

/// H·P  (3×4 × 4×4 = 3×4)
fn mult34_44(h: &M34, p: &M44) -> M34 {
    let mut r = [[0.0_f32; 4]; 3];
    for i in 0..3 { for j in 0..4 { for k in 0..4 { r[i][j] += h[i][k] * p[k][j]; } } }
    r
}

/// A·B  (3×4 × 4×3 = 3×3)
fn mult34_43(a: &M34, b: &M43) -> M33 {
    let mut r = [[0.0_f32; 3]; 3];
    for i in 0..3 { for j in 0..3 { for k in 0..4 { r[i][j] += a[i][k] * b[k][j]; } } }
    r
}

/// P·Hᵀ  (4×4 × 4×3 = 4×3)
fn mult44_43(p: &M44, ht: &M43) -> M43 {
    let mut r = [[0.0_f32; 3]; 4];
    for i in 0..4 { for j in 0..3 { for k in 0..4 { r[i][j] += p[i][k] * ht[k][j]; } } }
    r
}

/// K·S⁻¹  (4×3 × 3×3 = 4×3)
fn mult43_33(k: &M43, s: &M33) -> M43 {
    let mut r = [[0.0_f32; 3]; 4];
    for i in 0..4 { for j in 0..3 { for kk in 0..3 { r[i][j] += k[i][kk] * s[kk][j]; } } }
    r
}

/// K·y  (4×3 × 3 = 4)
fn mult43_v3(k: &M43, y: &V3) -> V4 {
    let mut r = [0.0_f32; 4];
    for i in 0..4 { for j in 0..3 { r[i] += k[i][j] * y[j]; } }
    r
}

/// K·H  (4×3 × 3×4 = 4×4)
fn mult43_34(k: &M43, h: &M34) -> M44 {
    let mut r = [[0.0_f32; 4]; 4];
    for i in 0..4 { for j in 0..4 { for kk in 0..3 { r[i][j] += k[i][kk] * h[kk][j]; } } }
    r
}

fn ajouter_diag33(m: &M33, s: f32) -> M33 {
    let mut r = *m;
    for i in 0..3 { r[i][i] += s; }
    r
}

/// Inversion d'une matrice 3×3 par la règle de Cramer.
/// Retourne `None` si la matrice est singulière (|det| < ε).
fn inverser33(m: &M33) -> Option<M33> {
    let [a, b, c] = [m[0][0], m[0][1], m[0][2]];
    let [d, e, f] = [m[1][0], m[1][1], m[1][2]];
    let [g, h, k] = [m[2][0], m[2][1], m[2][2]];

    let det = a * (e*k - f*h) - b * (d*k - f*g) + c * (d*h - e*g);
    if det.abs() < 1e-9 { return None; }
    let inv = 1.0 / det;

    Some([
        [(e*k - f*h) * inv, (c*h - b*k) * inv, (b*f - c*e) * inv],
        [(f*g - d*k) * inv, (a*k - c*g) * inv, (c*d - a*f) * inv],
        [(d*h - e*g) * inv, (b*g - a*h) * inv, (a*e - b*d) * inv],
    ])
}

// ─────────────────────────────────────────────────────────────────────────────
// Tests
// ─────────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use std::f32::consts::PI;
    use crate::types::{DonneesImu, Horodatage, Temperature};

    /// Construit une `MesureImuTraitee` de test.
    fn mesure(dt: Option<f32>, ax: f32, ay: f32, az: f32, gx: f32, gy: f32, gz: f32)
    -> crate::capteurs::traitement_imu::MesureImuTraitee {
        crate::capteurs::traitement_imu::MesureImuTraitee {
            donnees: DonneesImu {
                horodatage:   Horodatage::maintenant(),
                accelerometre: Vector3::nouveau(ax, ay, az),
                gyroscope:     Vector3::nouveau(gx, gy, gz),
                magnetometre:  Vector3::zero(),
                temperature:   Temperature::depuis_celsius(20.0),
            },
            dt_s: dt,
        }
    }

    // ── Initialisation ────────────────────────────────────────────────────────

    #[test]
    fn initialisation_carte_plate() {
        let mut ekf = EkfAttitude::nouveau();
        let att = ekf.mettre_a_jour(&mesure(None, 0.0, 0.0, G, 0.0, 0.0, 0.0));
        assert!(att.roulis.degres().abs()  < 0.1, "roulis  = {:.3}", att.roulis.degres());
        assert!(att.tangage.degres().abs() < 0.1, "tangage = {:.3}", att.tangage.degres());
    }

    #[test]
    fn initialisation_roulis_30deg() {
        let mut ekf = EkfAttitude::nouveau();
        let angle = 30.0_f32.to_radians();
        // Accélération mesurée pour board tilté de 30° (roulis positif = aile droite en bas)
        let att = ekf.mettre_a_jour(&mesure(
            None,
            0.0, G * angle.sin(), G * angle.cos(),
            0.0, 0.0, 0.0,
        ));
        assert!((att.roulis.degres() - 30.0).abs() < 0.5,
            "roulis = {:.3}", att.roulis.degres());
        assert!(att.tangage.degres().abs() < 0.5);
    }

    // ── Prédiction gyroscope ──────────────────────────────────────────────────

    #[test]
    fn prediction_repos_ne_change_pas_attitude() {
        let mut ekf = EkfAttitude::nouveau();
        ekf.mettre_a_jour(&mesure(None, 0.0, 0.0, G, 0.0, 0.0, 0.0));

        for _ in 0..20 {
            let att = ekf.mettre_a_jour(&mesure(Some(0.01), 0.0, 0.0, G, 0.0, 0.0, 0.0));
            assert!(att.roulis.degres().abs()  < 0.5);
            assert!(att.tangage.degres().abs() < 0.5);
        }
    }

    #[test]
    fn prediction_integre_lacet_45deg() {
        let mut ekf = EkfAttitude::nouveau();
        ekf.mettre_a_jour(&mesure(None, 0.0, 0.0, G, 0.0, 0.0, 0.0));

        // ω = 45°/s autour de Z, pendant 1 s → lacet ≈ 45°
        let att = ekf.mettre_a_jour(&mesure(Some(1.0), 0.0, 0.0, G, 0.0, 0.0, PI / 4.0));
        assert!((att.lacet.degres() - 45.0).abs() < 1.0,
            "lacet = {:.3}", att.lacet.degres());
    }

    #[test]
    fn prediction_integre_roulis_gyro() {
        let mut ekf = EkfAttitude::nouveau();
        ekf.mettre_a_jour(&mesure(None, 0.0, 0.0, G, 0.0, 0.0, 0.0));

        // ω = 30°/s autour de X, dt = 1 s → roulis ≈ 30°
        // (accel hors seuil → correction désactivée)
        let att = ekf.mettre_a_jour(&mesure(Some(1.0), 0.0, G * 0.5, G * 0.866, 30.0_f32.to_radians(), 0.0, 0.0));
        assert!((att.roulis.degres() - 30.0).abs() < 2.0,
            "roulis = {:.3}", att.roulis.degres());
    }

    // ── Correction accéléromètre ──────────────────────────────────────────────

    #[test]
    fn correction_carte_plate_reste_plate() {
        let mut ekf = EkfAttitude::nouveau();
        ekf.mettre_a_jour(&mesure(None, 0.0, 0.0, G, 0.0, 0.0, 0.0));

        // Innovation nulle → pas de changement d'état
        let att = ekf.mettre_a_jour(&mesure(Some(0.01), 0.0, 0.0, G, 0.0, 0.0, 0.0));
        assert!(att.roulis.degres().abs()  < 0.01);
        assert!(att.tangage.degres().abs() < 0.01);
    }

    #[test]
    fn convergence_correction_roulis_errone() {
        // Initialiser à 15° de roulis, puis appliquer 200 mesures "board plat"
        // → le filtre doit converger vers 0°
        let mut ekf = EkfAttitude::nouveau();
        let angle = 15.0_f32.to_radians();
        ekf.mettre_a_jour(&mesure(None, 0.0, G*angle.sin(), G*angle.cos(), 0.0, 0.0, 0.0));

        let att_init = { let att = ekf.mettre_a_jour(&mesure(Some(0.01), 0.0, 0.0, G, 0.0, 0.0, 0.0)); att.roulis.degres() };
        // La correction ne doit pas aggraver l'erreur au premier pas
        assert!(att_init.abs() < 15.5, "premier pas aggravé : {:.2}", att_init);

        for _ in 0..200 {
            ekf.mettre_a_jour(&mesure(Some(0.01), 0.0, 0.0, G, 0.0, 0.0, 0.0));
        }
        let roulis_final = { let att = ekf.mettre_a_jour(&mesure(Some(0.01), 0.0, 0.0, G, 0.0, 0.0, 0.0)); att.roulis.degres() };
        assert!(roulis_final.abs() < 1.0,
            "roulis non convergé après 200 pas : {:.2}°", roulis_final);
    }

    #[test]
    fn rejet_vol_accelere() {
        // Si ‖a‖ s'écarte de G, pas de correction → l'erreur reste
        let mut ekf = EkfAttitude::nouveau();
        let angle = 10.0_f32.to_radians();
        ekf.mettre_a_jour(&mesure(None, 0.0, G*angle.sin(), G*angle.cos(), 0.0, 0.0, 0.0));

        // Accélération anormale (vol accéléré)
        let a_anormale = 2.0 * G; // ‖a‖ = 2G >> G ± 0.3
        for _ in 0..50 {
            ekf.mettre_a_jour(&mesure(Some(0.01), 0.0, 0.0, a_anormale, 0.0, 0.0, 0.0));
        }
        let att = ekf.mettre_a_jour(&mesure(Some(0.01), 0.0, 0.0, a_anormale, 0.0, 0.0, 0.0));
        // L'erreur initiale de 10° ne doit pas converger vers 0 (correction rejetée)
        assert!(att.roulis.degres().abs() > 5.0,
            "correction ne devrait pas s'appliquer : {:.2}°", att.roulis.degres());
    }

    // ── Algèbre linéaire ──────────────────────────────────────────────────────

    #[test]
    fn mult44_identite() {
        let i = identite44();
        let a = [[1.0,2.0,3.0,4.0],[5.0,6.0,7.0,8.0],[9.0,10.0,11.0,12.0],[13.0,14.0,15.0,16.0]];
        let r = mult44(&i, &a);
        for i in 0..4 { for j in 0..4 { assert!((r[i][j] - a[i][j]).abs() < 1e-6); } }
    }

    #[test]
    fn inverser33_matrice_identite() {
        let i = [[1.0,0.0,0.0],[0.0,1.0,0.0],[0.0,0.0,1.0]];
        let inv = inverser33(&i).expect("identité inversible");
        for row in 0..3 { for col in 0..3 {
            let attendu = if row == col { 1.0 } else { 0.0 };
            assert!((inv[row][col] - attendu).abs() < 1e-6);
        } }
    }

    #[test]
    fn inverser33_matrice_singuliere_retourne_none() {
        let m = [[1.0,2.0,3.0],[4.0,5.0,6.0],[7.0,8.0,9.0]]; // det = 0
        assert!(inverser33(&m).is_none());
    }
}
