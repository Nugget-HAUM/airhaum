// src/types/geometrie.rs
// Types pour représenter des données géométriques et physiques

   #![allow(dead_code)]
   #![allow(unused_imports)]

use std::ops::{Add, Sub, Mul, Div};

/// Vecteur 3D générique (position, vitesse, accélération, etc.)
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Vector3 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Vector3 {
    pub fn nouveau(x: f32, y: f32, z: f32) -> Self {
        Self { x, y, z }
    }

    pub fn zero() -> Self {
        Self::nouveau(0.0, 0.0, 0.0)
    }

    /// Norme (longueur) du vecteur
    pub fn norme(&self) -> f32 {
        (self.x * self.x + self.y * self.y + self.z * self.z).sqrt()
    }

    /// Vecteur normalisé (longueur = 1)
    pub fn normalise(&self) -> Self {
        let n = self.norme();
        if n > 0.0 {
            Self::nouveau(self.x / n, self.y / n, self.z / n)
        } else {
            Self::zero()
        }
    }

    /// Produit scalaire
    pub fn dot(&self, other: &Self) -> f32 {
        self.x * other.x + self.y * other.y + self.z * other.z
    }

    /// Produit vectoriel
    pub fn cross(&self, other: &Self) -> Self {
        Self::nouveau(
            self.y * other.z - self.z * other.y,
            self.z * other.x - self.x * other.z,
            self.x * other.y - self.y * other.x,
        )
    }
}

// Opérations arithmétiques sur Vector3
impl Add for Vector3 {
    type Output = Self;
    fn add(self, other: Self) -> Self {
        Self::nouveau(self.x + other.x, self.y + other.y, self.z + other.z)
    }
}

impl Sub for Vector3 {
    type Output = Self;
    fn sub(self, other: Self) -> Self {
        Self::nouveau(self.x - other.x, self.y - other.y, self.z - other.z)
    }
}

impl Mul<f32> for Vector3 {
    type Output = Self;
    fn mul(self, scalar: f32) -> Self {
        Self::nouveau(self.x * scalar, self.y * scalar, self.z * scalar)
    }
}

impl Div<f32> for Vector3 {
    type Output = Self;
    fn div(self, scalar: f32) -> Self {
        Self::nouveau(self.x / scalar, self.y / scalar, self.z / scalar)
    }
}

/// Quaternion pour représenter l'attitude (évite le gimbal lock)
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Quaternion {
    pub w: f32,  // Partie réelle
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Quaternion {
    pub fn nouveau(w: f32, x: f32, y: f32, z: f32) -> Self {
        Self { w, x, y, z }
    }

    /// Quaternion identité (pas de rotation)
    pub fn identite() -> Self {
        Self::nouveau(1.0, 0.0, 0.0, 0.0)
    }

    /// Norme du quaternion
    pub fn norme(&self) -> f32 {
        (self.w * self.w + self.x * self.x + self.y * self.y + self.z * self.z).sqrt()
    }

    /// Normalise le quaternion (norme = 1)
    pub fn normalise(&self) -> Self {
        let n = self.norme();
        const EPSILON: f32 = 1e-6;
    
        if n > EPSILON {
           Self::nouveau(self.w / n, self.x / n, self.y / n, self.z / n)
        } else {
          // Log une erreur si vous avez un système de logging
           Self::identite()
        }
    }

    /// Conjugué du quaternion (pour rotation inverse)
    pub fn conjugue(&self) -> Self {
        Self::nouveau(self.w, -self.x, -self.y, -self.z)
    }

    /// Convertit en angles d'Euler (roulis, tangage, lacet) — convention ZYX, repère NED.
    ///
    /// - Roulis positif  : aile droite vers le bas
    /// - Tangage positif : nez vers le haut
    /// - Lacet positif   : nez vers la droite
    pub fn vers_euler(&self) -> (Angle, Angle, Angle) {
        let roulis = Angle::depuis_radians(
            (2.0 * (self.w * self.x + self.y * self.z))
                .atan2(1.0 - 2.0 * (self.x * self.x + self.y * self.y))
        );
        // Clamp pour éviter NaN sur les erreurs d'arrondi flottant proches de ±1
        let sin_tangage = (2.0 * (self.w * self.y - self.z * self.x)).clamp(-1.0, 1.0);
        let tangage = Angle::depuis_radians(sin_tangage.asin());
        let lacet = Angle::depuis_radians(
            (2.0 * (self.w * self.z + self.x * self.y))
                .atan2(1.0 - 2.0 * (self.y * self.y + self.z * self.z))
        );
        (roulis, tangage, lacet)
    }

    /// Produit de Hamilton : `self ⊗ autre`.
    ///
    /// Non commutatif : `a.multiplier(b) ≠ b.multiplier(a)` en général.
    pub fn multiplier(&self, autre: &Self) -> Self {
        Self {
            w: self.w * autre.w - self.x * autre.x - self.y * autre.y - self.z * autre.z,
            x: self.w * autre.x + self.x * autre.w + self.y * autre.z - self.z * autre.y,
            y: self.w * autre.y - self.x * autre.z + self.y * autre.w + self.z * autre.x,
            z: self.w * autre.z + self.x * autre.y - self.y * autre.x + self.z * autre.w,
        }
    }

    /// Tourne le vecteur `v` du repère monde (NED inertiel) vers le repère corps.
    ///
    /// Formule : `v_corps = q* ⊗ [0,v] ⊗ q`
    ///
    /// Utilisation principale : gravité attendue dans le repère corps pour l'EKF.
    /// ```text
    /// let g_corps = q.monde_vers_corps(Vector3::nouveau(0.0, 0.0, 9.81));
    /// ```
    pub fn monde_vers_corps(&self, v: Vector3) -> Vector3 {
        let v_quat = Self::nouveau(0.0, v.x, v.y, v.z);
        let r = self.conjugue().multiplier(&v_quat).multiplier(self);
        Vector3::nouveau(r.x, r.y, r.z)
    }

    /// Quaternion de rotation incrémentale depuis une vitesse angulaire gyroscope.
    ///
    /// `omega` : vitesse angulaire en rad/s (repère corps).
    /// `dt_s`  : pas de temps en secondes.
    ///
    /// Pour intégrer : `q_new = q_courant.multiplier(&Quaternion::depuis_gyroscope(omega, dt)).normalise()`
    pub fn depuis_gyroscope(omega: Vector3, dt_s: f32) -> Self {
        let norme_omega = (omega.x * omega.x + omega.y * omega.y + omega.z * omega.z).sqrt();
        if norme_omega < 1e-6 {
            return Self::identite();
        }
        let demi_angle = norme_omega * dt_s * 0.5;
        let s = demi_angle.sin() / norme_omega;
        Self {
            w: demi_angle.cos(),
            x: omega.x * s,
            y: omega.y * s,
            z: omega.z * s,
        }
    }
}

/// Angle avec conversion automatique radians/degrés
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Angle {
    radians: f32,
}

impl Angle {
    pub fn depuis_radians(rad: f32) -> Self {
        Self { radians: rad }
    }

    pub fn depuis_degres(deg: f32) -> Self {
        Self { radians: deg.to_radians() }
    }

    pub fn radians(&self) -> f32 {
        self.radians
    }

    pub fn degres(&self) -> f32 {
        self.radians.to_degrees()
    }

    /// Normalise l'angle entre -π et π
    pub fn normalise(&self) -> Self {
        let mut angle = self.radians;
        while angle > std::f32::consts::PI {
            angle -= 2.0 * std::f32::consts::PI;
        }
        while angle < -std::f32::consts::PI {
            angle += 2.0 * std::f32::consts::PI;
        }
        Self::depuis_radians(angle)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f32::consts::PI;

    // ── Vector3 ──────────────────────────────────────────────────────────────

    #[test]
    fn test_vector3_norme() {
        let v = Vector3::nouveau(3.0, 4.0, 0.0);
        assert_eq!(v.norme(), 5.0);
    }

    #[test]
    fn test_vector3_normalise() {
        let v = Vector3::nouveau(3.0, 4.0, 0.0);
        let n = v.normalise();
        assert!((n.norme() - 1.0).abs() < 0.0001);
    }

    // ── Angle ─────────────────────────────────────────────────────────────────

    #[test]
    fn test_angle_conversion() {
        let a = Angle::depuis_degres(180.0);
        assert!((a.radians() - PI).abs() < 0.0001);
    }

    // ── Quaternion : propriétés de base ──────────────────────────────────────

    #[test]
    fn test_quaternion_identite() {
        let q = Quaternion::identite();
        assert_eq!(q.w, 1.0);
        assert_eq!(q.x, 0.0);
    }

    #[test]
    fn test_quaternion_normalise_zero() {
        let q = Quaternion::nouveau(0.0, 0.0, 0.0, 0.0);
        assert_eq!(q.normalise(), Quaternion::identite());
    }

    #[test]
    fn test_quaternion_normalise_tres_petit() {
        let q = Quaternion::nouveau(1e-10, 1e-10, 1e-10, 1e-10);
        assert!(!q.normalise().w.is_nan());
    }

    // ── Quaternion : produit de Hamilton ─────────────────────────────────────

    /// i × j = k  (quaternions unités de base)
    #[test]
    fn produit_i_fois_j_est_k() {
        let i = Quaternion::nouveau(0.0, 1.0, 0.0, 0.0);
        let j = Quaternion::nouveau(0.0, 0.0, 1.0, 0.0);
        let k = i.multiplier(&j);
        assert!((k.w).abs() < 1e-6);
        assert!((k.x).abs() < 1e-6);
        assert!((k.y).abs() < 1e-6);
        assert!((k.z - 1.0).abs() < 1e-6);
    }

    /// j × i = −k  (non-commutativité)
    #[test]
    fn produit_j_fois_i_est_moins_k() {
        let i = Quaternion::nouveau(0.0, 1.0, 0.0, 0.0);
        let j = Quaternion::nouveau(0.0, 0.0, 1.0, 0.0);
        let r = j.multiplier(&i);
        assert!((r.z + 1.0).abs() < 1e-6);
    }

    /// q × q^-1 = identité
    #[test]
    fn produit_par_conjugue_donne_identite() {
        let q = Quaternion::nouveau(0.5, 0.5, 0.5, 0.5).normalise();
        let r = q.multiplier(&q.conjugue());
        assert!((r.w - 1.0).abs() < 1e-5);
        assert!(r.x.abs() < 1e-5);
        assert!(r.y.abs() < 1e-5);
        assert!(r.z.abs() < 1e-5);
    }

    // ── Quaternion : rotation monde→corps ────────────────────────────────────

    /// L'identité ne tourne pas le vecteur.
    #[test]
    fn monde_vers_corps_identite_inchange() {
        let q = Quaternion::identite();
        let v = Vector3::nouveau(1.0, 2.0, 3.0);
        let r = q.monde_vers_corps(v);
        assert!((r.x - 1.0).abs() < 1e-5);
        assert!((r.y - 2.0).abs() < 1e-5);
        assert!((r.z - 3.0).abs() < 1e-5);
    }

    /// Roulis +90° : la gravité monde [0,0,g] doit apparaître en +Y corps.
    ///
    /// Convention : roulis positif = aile droite vers le bas.
    /// Avec l'aile droite au sol, l'axe Y corps pointe vers le bas → g_y = +g.
    #[test]
    fn monde_vers_corps_roulis_90deg() {
        let angle = PI / 2.0;
        let q = Quaternion::nouveau(
            (angle / 2.0).cos(), (angle / 2.0).sin(), 0.0, 0.0
        ).normalise();
        let g_monde = Vector3::nouveau(0.0, 0.0, 9.81);
        let g_corps = q.monde_vers_corps(g_monde);
        assert!(g_corps.x.abs() < 1e-4, "gx attendu ≈ 0, obtenu {:.4}", g_corps.x);
        assert!((g_corps.y - 9.81).abs() < 1e-3, "gy attendu ≈ 9.81, obtenu {:.4}", g_corps.y);
        assert!(g_corps.z.abs() < 1e-4, "gz attendu ≈ 0, obtenu {:.4}", g_corps.z);
    }

    /// Tangage +30° (nez vers le haut) : g_x = −g×sin(30°), g_z = g×cos(30°).
    #[test]
    fn monde_vers_corps_tangage_30deg() {
        let angle = PI / 6.0; // 30°
        let q = Quaternion::nouveau(
            (angle / 2.0).cos(), 0.0, (angle / 2.0).sin(), 0.0
        ).normalise();
        let g = 9.81_f32;
        let g_corps = q.monde_vers_corps(Vector3::nouveau(0.0, 0.0, g));
        let attendu_x = -g * angle.sin();
        let attendu_z =  g * angle.cos();
        assert!((g_corps.x - attendu_x).abs() < 1e-4,
            "gx attendu {:.4}, obtenu {:.4}", attendu_x, g_corps.x);
        assert!(g_corps.y.abs() < 1e-4, "gy attendu ≈ 0, obtenu {:.4}", g_corps.y);
        assert!((g_corps.z - attendu_z).abs() < 1e-4,
            "gz attendu {:.4}, obtenu {:.4}", attendu_z, g_corps.z);
    }

    // ── Quaternion : intégration gyroscope ───────────────────────────────────

    /// Vitesse angulaire nulle → pas de rotation (identité).
    #[test]
    fn depuis_gyroscope_repos() {
        let q = Quaternion::depuis_gyroscope(Vector3::zero(), 0.01);
        assert!((q.w - 1.0).abs() < 1e-6);
        assert!(q.x.abs() < 1e-6);
        assert!(q.y.abs() < 1e-6);
        assert!(q.z.abs() < 1e-6);
    }

    /// Lacet +45°/s pendant 1 s → lacet extrait ≈ 45°.
    #[test]
    fn depuis_gyroscope_lacet_45deg() {
        let omega = Vector3::nouveau(0.0, 0.0, PI / 4.0); // 45°/s autour de Z
        let delta = Quaternion::depuis_gyroscope(omega, 1.0);
        let q = Quaternion::identite().multiplier(&delta).normalise();
        let (_roulis, _tangage, lacet) = q.vers_euler();
        assert!((lacet.degres() - 45.0).abs() < 0.01,
            "lacet attendu 45°, obtenu {:.4}°", lacet.degres());
    }

    // ── Quaternion : extraction Euler ─────────────────────────────────────────

    /// Identité → roulis = tangage = lacet = 0.
    #[test]
    fn vers_euler_identite() {
        let (r, t, l) = Quaternion::identite().vers_euler();
        assert!(r.degres().abs() < 0.001);
        assert!(t.degres().abs() < 0.001);
        assert!(l.degres().abs() < 0.001);
    }

    /// Roulis +30° → angle extrait ≈ 30°.
    #[test]
    fn vers_euler_roulis_30deg() {
        let angle = 30.0_f32.to_radians();
        let q = Quaternion::nouveau(
            (angle / 2.0).cos(), (angle / 2.0).sin(), 0.0, 0.0
        ).normalise();
        let (roulis, tangage, _) = q.vers_euler();
        assert!((roulis.degres() - 30.0).abs() < 0.01,
            "roulis attendu 30°, obtenu {:.4}°", roulis.degres());
        assert!(tangage.degres().abs() < 0.01);
    }

    /// Tangage +20° → angle extrait ≈ 20°.
    #[test]
    fn vers_euler_tangage_20deg() {
        let angle = 20.0_f32.to_radians();
        let q = Quaternion::nouveau(
            (angle / 2.0).cos(), 0.0, (angle / 2.0).sin(), 0.0
        ).normalise();
        let (roulis, tangage, _) = q.vers_euler();
        assert!((tangage.degres() - 20.0).abs() < 0.01,
            "tangage attendu 20°, obtenu {:.4}°", tangage.degres());
        assert!(roulis.degres().abs() < 0.01);
    }

    /// Valeurs proches de ±1 dans l'asin ne produisent pas de NaN (clamp actif).
    #[test]
    fn vers_euler_pas_de_nan_au_tangage_90deg() {
        // tangage 90° : sin = 1.0, valeur potentiellement > 1 par erreur flottante
        let angle = PI / 2.0;
        let q = Quaternion::nouveau(
            (angle / 2.0).cos(), 0.0, (angle / 2.0).sin(), 0.0
        ).normalise();
        let (_, tangage, _) = q.vers_euler();
        assert!(!tangage.radians().is_nan(), "tangage NaN à 90°");
        assert!((tangage.degres() - 90.0).abs() < 0.01);
    }
}
