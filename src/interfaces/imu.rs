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

    /// Calibre le gyroscope
    ///
    /// Le capteur doit être immobile pendant ~5 secondes.
    /// Mesure et stocke les offsets sur chaque axe.
    fn calibrer_gyro(&mut self) -> Result<()>;

    /// Calibre l'accéléromètre
    ///
    /// Le capteur doit être posé sur une surface plane.
    /// Mesure et stocke les offsets et facteurs d'échelle.
    fn calibrer_accel(&mut self) -> Result<()>;

    /// Calibre le magnétomètre
    ///
    /// Nécessite une procédure active (rotations lentes sur les 3 axes).
    /// Calcule les corrections hard iron et soft iron.
    fn calibrer_mag(&mut self) -> Result<()>;

    /// Obtient l'état actuel du capteur
    fn obtenir_etat(&self) -> &EtatCapteur;

    /// Vérifie si le capteur est opérationnel
    fn est_operationnel(&self) -> bool {
        self.obtenir_etat().est_utilisable()
    }
}
