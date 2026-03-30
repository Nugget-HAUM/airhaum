// src/types/messages.rs
// Types pour les données des capteurs et messages système

   #![allow(dead_code)]
   #![allow(unused_imports)]

use super::geometrie::{Vector3, Quaternion};

use std::time::Instant;    // Necessaire pour Horodatage
use std::sync::OnceLock;   // Necessaire pour Horodatage

// Horodatage monotone basé sur CLOCK_MONOTONIC
// 
// NOTES : Sur Armbian (Linux non-RT), les garanties sont :
// - Le temps avance toujours (monotone)
// - Précision microseconde pour l'horodatage
// - MAIS pas de garantie temps réel sur l'exécution des tâches
// 


// Horodatage système en microsecondes
// 
// # Comportement
// Le "temps zéro" correspond au **premier appel** à `Horodatage::maintenant()`,
// pas au démarrage du programme. Cela garantit une référence stable pour toutes
// les mesures du système.
// 
// # Exemple
// ```
// let t1 = Horodatage::maintenant(); // t1.micros() ≈ 0
// std::thread::sleep(Duration::from_millis(10));
// let t2 = Horodatage::maintenant(); // t2.micros() ≈ 10000
// ```

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub struct Horodatage {
    micros: u64,
}

impl Horodatage {
    pub fn maintenant() -> Self {
        static DEBUT: OnceLock<Instant> = OnceLock::new();
        let debut = DEBUT.get_or_init(Instant::now);
        let ecoule = debut.elapsed();
        Self {
            micros: ecoule.as_micros() as u64,
        }    
    }

    /// Durée écoulée depuis ce horodatage
    pub fn ecoule(&self) -> std::time::Duration {
        let maintenant = Self::maintenant();
        std::time::Duration::from_micros(
            maintenant.micros.saturating_sub(self.micros)
        )
    }
    
    /// Vérifie si un certain délai s'est écoulé
    pub fn est_ecoule(&self, duree: std::time::Duration) -> bool {
        self.ecoule() >= duree
    }

    pub fn depuis_micros(micros: u64) -> Self {
        Self { micros }
    }

    pub fn micros(&self) -> u64 {
        self.micros
    }

    pub fn millis(&self) -> u64 {
        self.micros / 1000
    }

    pub fn seconds(&self) -> f32 {
        self.micros as f32 / 1_000_000.0
    }

    // Delta de temps entre deux timestamps
    // Utile pour calculer dt dans les filtres de Kalman
    pub fn delta_secondes(&self, anterieur: Horodatage) -> f32 {
        let delta_us = self.micros.saturating_sub(anterieur.micros);
        delta_us as f32 / 1_000_000.0
    }
}

/// Données brutes de l'IMU (MPU9265)
#[derive(Debug, Clone, Copy)]
pub struct DonneesImu {
    pub horodatage: Horodatage,
    pub accelerometre: Vector3,  // m/s²
    pub gyroscope: Vector3,      // rad/s
    pub magnetometre: Vector3,   // µT (microtesla)
    pub temperature: Temperature,
}

/// Données du baromètre (BMP280)
#[derive(Debug, Clone, Copy)]
pub struct DonneesBarometre {
    pub horodatage: Horodatage,
    pub pression: Pression,      // Pa (Pascal)
    pub temperature: Temperature,
}

/// Données GPS (NEO-M8N)
#[derive(Debug, Clone, Copy)]
pub struct DonneesGps {
    pub horodatage: Horodatage,
    pub latitude: f64,           // degrés
    pub longitude: f64,          // degrés
    pub altitude: f32,           // mètres (au-dessus du niveau de la mer)
    pub vitesse: f32,            // m/s
    pub cap: f32,                // degrés (0-360)
    pub precision_horizontale: f32,  // mètres (HDOP)
    pub precision_verticale: f32,    // mètres (VDOP)
    pub nombre_satellites: u8,
    pub fix_valide: bool,
}

/// Données du télémètre (VL53L0X)
#[derive(Debug, Clone, Copy)]
pub struct DonneesTelemetre {
    pub horodatage: Horodatage,
    pub distance: Distance,      // mètres
    pub qualite_signal: u8,      // 0-255
}

/// Type fort pour la température
#[derive(Debug, Clone, Copy, PartialEq, PartialOrd)]
pub struct Temperature {
    celsius: f32,
}

impl Temperature {
    pub fn depuis_celsius(celsius: f32) -> Self {
        Self { celsius }
    }

    pub fn celsius(&self) -> f32 {
        self.celsius
    }

    pub fn kelvin(&self) -> f32 {
        self.celsius + 273.15
    }
}

/// Type fort pour la pression atmosphérique
#[derive(Debug, Clone, Copy, PartialEq, PartialOrd)]
pub struct Pression {
    pascals: f32,
}

impl Pression {
    pub fn depuis_pascals(pa: f32) -> Self {
        Self { pascals: pa }
    }

    pub fn depuis_hectopascals(hpa: f32) -> Self {
        Self { pascals: hpa * 100.0 }
    }

    /// Retourne la pression standard au niveau de la mer (101325 Pa)
    pub fn niveau_mer_standard() -> Self {
        Self::depuis_pascals(crate::types::PRESSION_NIVEAU_MER_STANDARD)
    }

    pub fn pascals(&self) -> f32 {
        self.pascals
    }

    pub fn hectopascals(&self) -> f32 {
        self.pascals / 100.0
    }

    /// Convertit la pression en altitude approximative (formule barométrique)
    /// Nécessite la pression au niveau de la mer comme référence
    pub fn vers_altitude(&self, pression_niveau_mer: Pression) -> f32 {
        let p0 = pression_niveau_mer.pascals();
        let p = self.pascals();
        44330.0 * (1.0 - (p / p0).powf(0.1903))
    }
}

/// Type fort pour les distances
#[derive(Debug, Clone, Copy, PartialEq, PartialOrd)]
pub struct Distance {
    metres: f32,
}

impl Distance {
    pub fn depuis_metres(m: f32) -> Self {
        Self { metres: m }
    }

    pub fn depuis_millimetres(mm: f32) -> Self {
        Self { metres: mm / 1000.0 }
    }

    pub fn metres(&self) -> f32 {
        self.metres
    }

    pub fn millimetres(&self) -> f32 {
        self.metres * 1000.0
    }
}

/// Énumération générale pour tous les types de données capteurs
#[derive(Debug, Clone, Copy)]
pub enum DonneesCapteur {
    Imu(DonneesImu),
    Barometre(DonneesBarometre),
    Gps(DonneesGps),
    Telemetre(DonneesTelemetre),
}

/// Messages système pour communication inter-tâches
#[derive(Debug, Clone)]
pub enum MessageSysteme {
    /// Demande d'arrêt d'urgence
    ArretUrgence,
    
    /// Changement de mode de vol
    ChangementMode(ModeVol),
    
    /// Commande de mission
    CommandeMission(CommandeMission),
    
    /// État de santé du système
    EtatSante(EtatSante),
}

/// Modes de vol possibles
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ModeVol {
    Desarme,
    Manuel,
    Stabilise,
    Altitude,
    Position,
    Mission,
    RetourMaison,
    Urgence,
}

/// Commandes de mission
#[derive(Debug, Clone)]
pub enum CommandeMission {
    Decoller(f32),      // altitude cible en mètres
    Atterrir,
    AllerVers { latitude: f64, longitude: f64, altitude: f32 },
    Loiter,             // Maintien position
    RetourMaison,
}

/// État de santé du système
#[derive(Debug, Clone, Copy)]
pub struct EtatSante {
    pub batterie_ok: bool,
    pub capteurs_ok: bool,
    pub gps_ok: bool,
    pub liaison_radio_ok: bool,
    pub niveau_batterie: f32,  // 0.0 à 1.0
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_temperature_conversion() {
        let t = Temperature::depuis_celsius(20.0);
        assert_eq!(t.celsius(), 20.0);
        assert!((t.kelvin() - 293.15).abs() < 0.01);
    }

    #[test]
    fn test_pression_conversion() {
        let p = Pression::depuis_hectopascals(1013.25);
        assert!((p.pascals() - 101325.0).abs() < 0.1);
    }

    #[test]
    fn test_pression_vers_altitude() {
        let p_mer = Pression::depuis_hectopascals(1013.25);
        let p_1000m = Pression::depuis_hectopascals(898.0);
        let alt = p_1000m.vers_altitude(p_mer);
        assert!((alt - 1000.0).abs() < 50.0); // Approximation ±50m
    }

    #[test]
    fn test_distance_conversion() {
        let d = Distance::depuis_millimetres(1500.0);
        assert_eq!(d.metres(), 1.5);
    }

    #[test]
    fn test_horodatage_monotone() {
        // Vérifie que le temps avance toujours
        let h1 = Horodatage::maintenant();
        std::thread::sleep(std::time::Duration::from_millis(10));
        let h2 = Horodatage::maintenant();
        assert!(h2 > h1, "Le temps doit avancer");
    }
    
    #[test]
    fn test_horodatage_precision() {
        // Vérifie la précision microseconde
        let h1 = Horodatage::maintenant();
        let h2 = Horodatage::maintenant();
        let delta = h2.micros() - h1.micros();
        assert!(delta < 1000, "Deux appels successifs doivent être < 1ms");
    }


}
