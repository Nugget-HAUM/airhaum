// src/interfaces/imu.rs
//! Interface générique pour une centrale inertielle (IMU)
//!
//! Ce trait permet de découpler la logique métier des implémentations
//! hardware spécifiques (MPU9250, ICM-42688, etc.)

use crate::types::{Result, DonneesImu, EtatCapteur};

/// Interface générique pour une centrale inertielle
///
/// Fournit gyroscope, accéléromètre et magnétomètre.
pub trait CentraleInertielle: Send {
    /// Initialise la centrale inertielle
    ///
    /// Applique la logique de reprise rapide si les calibrations
    /// sont valides, sinon effectue une initialisation complète.
    fn initialiser(&mut self) -> Result<()>;

    /// Lit les données brutes compensées de l'IMU
    ///
    /// Retourne gyro (rad/s), accel (m/s²), mag (µT), température.
    fn lire(&mut self) -> Result<DonneesImu>;

    // Les calibrations ne font pas partie de l'interface : ce sont des
    // procédures interactives propres au matériel et à l'avion
    // (`doc/calibration.md`, `diagnostiques/diag_mpu9250.rs`).

    /// Obtient l'état actuel du capteur
    fn obtenir_etat(&self) -> &EtatCapteur;

    /// Vérifie si le capteur est opérationnel
    fn est_operationnel(&self) -> bool {
        self.obtenir_etat().est_utilisable()
    }
}
