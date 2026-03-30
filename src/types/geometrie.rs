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

    /// Convertit en angles d'Euler (roll, pitch, yaw)
    pub fn vers_euler(&self) -> (Angle, Angle, Angle) {
        let roll = Angle::depuis_radians(
            (2.0 * (self.w * self.x + self.y * self.z)).atan2(
                1.0 - 2.0 * (self.x * self.x + self.y * self.y)
            )
        );
        
        let pitch = Angle::depuis_radians(
            (2.0 * (self.w * self.y - self.z * self.x)).asin()
        );
        
        let yaw = Angle::depuis_radians(
            (2.0 * (self.w * self.z + self.x * self.y)).atan2(
                1.0 - 2.0 * (self.y * self.y + self.z * self.z)
            )
        );

        (roll, pitch, yaw)
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

    #[test]
    fn test_quaternion_identite() {
        let q = Quaternion::identite();
        assert_eq!(q.w, 1.0);
        assert_eq!(q.x, 0.0);
    }

    #[test]
    fn test_angle_conversion() {
        let a = Angle::depuis_degres(180.0);
        assert!((a.radians() - std::f32::consts::PI).abs() < 0.0001);
    }

   #[test]
   fn test_quaternion_normalise_zero() {
       let q = Quaternion::nouveau(0.0, 0.0, 0.0, 0.0);
       let normalisee = q.normalise();
       // Ne devrait pas paniquer et retourner un quaternion valide
       assert_eq!(normalisee, Quaternion::identite());
   }

   #[test]
   fn test_quaternion_normalise_tres_petit() {
       let q = Quaternion::nouveau(1e-10, 1e-10, 1e-10, 1e-10);
       let normalisee = q.normalise();
       // Ne devrait pas produire de NaN
       assert!(!normalisee.w.is_nan());
   }

}
