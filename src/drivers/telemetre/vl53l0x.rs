// src/drivers/telemetre/vl53l0x.rs
//! Driver pour le capteur de distance laser VL53L0X
//!
//! Le VL53L0X est un capteur de distance par temps de vol (ToF - Time of Flight)
//! avec une portée jusqu'à 2 mètres et une précision millimétrique.
//!
//! Datasheet: https://www.st.com/resource/en/datasheet/vl53l0x.pdf

use crate::hal::BusI2c;
use crate::types::{Result, ErreursAirHaum, EtatCapteur, Horodatage};
use crate::interfaces::telemetre::Telemetre;

/// Adresse I²C par défaut du VL53L0X
pub const ADRESSE_VL53L0X: u8 = 0x29;

/// Registres du VL53L0X
#[allow(dead_code)]
mod registres {
    pub const SYSRANGE_START: u8 = 0x00;
    pub const SYSTEM_SEQUENCE_CONFIG: u8 = 0x01;
    pub const SYSTEM_RANGE_CONFIG: u8 = 0x09;
    pub const SYSTEM_INTERMEASUREMENT_PERIOD: u8 = 0x04;
    pub const SYSTEM_INTERRUPT_CONFIG_GPIO: u8 = 0x0A;
    pub const SYSTEM_INTERRUPT_CLEAR: u8 = 0x0B;

    pub const RESULT_INTERRUPT_STATUS: u8 = 0x13;
    pub const RESULT_RANGE_STATUS: u8 = 0x14;

    pub const IDENTIFICATION_MODEL_ID: u8 = 0xC0;
    pub const IDENTIFICATION_REVISION_ID: u8 = 0xC2;

    pub const VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV: u8 = 0x89;

    pub const MSRC_CONFIG_CONTROL: u8 = 0x60;

    pub const FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT: u8 = 0x44;
    pub const FINAL_RANGE_CONFIG_VALID_PHASE_HIGH: u8 = 0x48;
    pub const FINAL_RANGE_CONFIG_VALID_PHASE_LOW: u8 = 0x47;

    pub const GLOBAL_CONFIG_VCSEL_WIDTH: u8 = 0x32;
    pub const GLOBAL_CONFIG_SPAD_ENABLES_REF_0: u8 = 0xB0;
    pub const GLOBAL_CONFIG_REF_EN_START_SELECT: u8 = 0xB6;

    pub const DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD: u8 = 0x4E;
    pub const DYNAMIC_SPAD_REF_EN_START_OFFSET: u8 = 0x4F;

    pub const POWER_MANAGEMENT_GO1_POWER_FORCE: u8 = 0x80;

    pub const CROSSTALK_COMPENSATION_PEAK_RATE_MCPS: u8 = 0x20;
}

use registres::*;

/// Structure représentant le capteur VL53L0X
pub struct Vl53l0x<I2C: BusI2c> {
    i2c: I2C,
    adresse: u8,
    timeout_ms: u16,
    #[allow(dead_code)]
    budget_temps_mesure_us: u32,
    etat: EtatCapteur,
    derniere_distance: Option<u16>,
    derniere_lecture: Option<Horodatage>,
    stop_variable: u8,
}

impl<I2C: BusI2c> Vl53l0x<I2C> {
    /// Crée une nouvelle instance du driver VL53L0X
    ///
    /// # Arguments
    /// * `i2c` - Bus I²C
    /// * `adresse` - Adresse I²C du capteur (par défaut 0x29)
    pub fn nouveau(i2c: I2C, adresse: u8) -> Self {
        Self {
            i2c,
            adresse,
            timeout_ms: 500,
            budget_temps_mesure_us: 33000,
            etat: EtatCapteur::Inconnu,
            derniere_distance: None,
            derniere_lecture: None,
            stop_variable: 0,
        }
    }

    /// Vérifie l'identité du capteur (Model ID doit être 0xEE)
    pub fn verifier_identite(&mut self) -> Result<bool> {
        let model_id = self.lire_u8(IDENTIFICATION_MODEL_ID)?;
        println!("VL53L0X Model ID: 0x{:02X} (attendu: 0xEE)", model_id);
        Ok(model_id == 0xEE)
    }

    // -------------------------------------------------------------------------
    // Initialisation
    // -------------------------------------------------------------------------

    /// Initialisation des données (data init)
    ///
    /// Séquence identique à la référence ST / Pololu :
    ///   1. Mode 2V8
    ///   2. Lecture stop_variable depuis registre interne 0x91
    ///   3. Config GPIO interrupt (new sample ready)
    ///   4. Désactivation MSRC/pre-range signal rate check
    ///   5. SYSTEM_SEQUENCE_CONFIG = 0xFF
    fn data_init(&mut self) -> Result<()> {
        // Mode 2V8 (3.3V → bit 0 de 0x89)
        let vhv = self.lire_u8(VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV)?;
        self.ecrire_u8(VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV, vhv | 0x01)?;

        // Lecture de stop_variable depuis la banque interne ST
        self.ecrire_u8(0x88, 0x00)?;
        self.ecrire_u8(0x80, 0x01)?;
        self.ecrire_u8(0xFF, 0x01)?;
        self.ecrire_u8(0x00, 0x00)?;
        self.stop_variable = self.lire_u8(0x91)?;
        println!("VL53L0X stop_variable: 0x{:02X}", self.stop_variable);
        self.ecrire_u8(0x00, 0x01)?;
        self.ecrire_u8(0xFF, 0x00)?;
        self.ecrire_u8(0x80, 0x00)?;

        // Config interruption GPIO : new sample ready
        self.ecrire_u8(SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04)?;
        let gpio_hv = self.lire_u8(0x84)?;
        self.ecrire_u8(0x84, gpio_hv & !0x10)?;
        self.ecrire_u8(SYSTEM_INTERRUPT_CLEAR, 0x01)?;

        // Désactiver SIGNAL_RATE_MSRC et SIGNAL_RATE_PRE_RANGE
        let msrc = self.lire_u8(MSRC_CONFIG_CONTROL)?;
        self.ecrire_u8(MSRC_CONFIG_CONTROL, msrc | 0x12)?;

        // Signal rate limit 0.25 MCPS (valeur par défaut ST)
        self.ecrire_u8(FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, 0x00)?;
        self.ecrire_u8(FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT + 1, 0x32)?;

        // Activer toutes les étapes de séquence
        self.ecrire_u8(SYSTEM_SEQUENCE_CONFIG, 0xFF)?;

        Ok(())
    }

    /// Initialisation statique (tuning settings propriétaires ST)
    fn static_init(&mut self) -> Result<()> {
        self.ecrire_u8(0xFF, 0x01)?; self.ecrire_u8(0x00, 0x00)?;
        self.ecrire_u8(0xFF, 0x00)?; self.ecrire_u8(0x09, 0x00)?;
        self.ecrire_u8(0x10, 0x00)?; self.ecrire_u8(0x11, 0x00)?;
        self.ecrire_u8(0x24, 0x01)?; self.ecrire_u8(0x25, 0xFF)?;
        self.ecrire_u8(0x75, 0x00)?;

        self.ecrire_u8(0xFF, 0x01)?; self.ecrire_u8(0x4E, 0x2C)?;
        self.ecrire_u8(0x48, 0x00)?; self.ecrire_u8(0x30, 0x20)?;

        self.ecrire_u8(0xFF, 0x00)?; self.ecrire_u8(0x30, 0x09)?;
        self.ecrire_u8(0x54, 0x00)?; self.ecrire_u8(0x31, 0x04)?;
        self.ecrire_u8(0x32, 0x03)?; self.ecrire_u8(0x40, 0x83)?;
        self.ecrire_u8(0x46, 0x25)?; self.ecrire_u8(0x60, 0x00)?;
        self.ecrire_u8(0x27, 0x00)?; self.ecrire_u8(0x50, 0x06)?;
        self.ecrire_u8(0x51, 0x00)?; self.ecrire_u8(0x52, 0x96)?;
        self.ecrire_u8(0x56, 0x08)?; self.ecrire_u8(0x57, 0x30)?;
        self.ecrire_u8(0x61, 0x00)?; self.ecrire_u8(0x62, 0x00)?;
        self.ecrire_u8(0x64, 0x00)?; self.ecrire_u8(0x65, 0x00)?;
        self.ecrire_u8(0x66, 0xA0)?;

        self.ecrire_u8(0xFF, 0x01)?; self.ecrire_u8(0x22, 0x32)?;
        self.ecrire_u8(0x47, 0x14)?; self.ecrire_u8(0x49, 0xFF)?;
        self.ecrire_u8(0x4A, 0x00)?;

        self.ecrire_u8(0xFF, 0x00)?; self.ecrire_u8(0x7A, 0x0A)?;
        self.ecrire_u8(0x7B, 0x00)?; self.ecrire_u8(0x78, 0x21)?;

        self.ecrire_u8(0xFF, 0x01)?; self.ecrire_u8(0x23, 0x34)?;
        self.ecrire_u8(0x42, 0x00)?; self.ecrire_u8(0x44, 0xFF)?;
        self.ecrire_u8(0x45, 0x26)?; self.ecrire_u8(0x46, 0x05)?;
        self.ecrire_u8(0x40, 0x40)?; self.ecrire_u8(0x0E, 0x06)?;
        self.ecrire_u8(0x20, 0x1A)?; self.ecrire_u8(0x43, 0x40)?;

        self.ecrire_u8(0xFF, 0x00)?; self.ecrire_u8(0x34, 0x03)?;
        self.ecrire_u8(0x35, 0x44)?;

        self.ecrire_u8(0xFF, 0x01)?; self.ecrire_u8(0x31, 0x04)?;
        self.ecrire_u8(0x4B, 0x09)?; self.ecrire_u8(0x4C, 0x05)?;
        self.ecrire_u8(0x4D, 0x04)?;

        self.ecrire_u8(0xFF, 0x00)?; self.ecrire_u8(0x44, 0x00)?;
        self.ecrire_u8(0x45, 0x20)?; self.ecrire_u8(0x47, 0x08)?;
        self.ecrire_u8(0x48, 0x28)?; self.ecrire_u8(0x67, 0x00)?;
        self.ecrire_u8(0x70, 0x04)?; self.ecrire_u8(0x71, 0x01)?;
        self.ecrire_u8(0x72, 0xFE)?; self.ecrire_u8(0x76, 0x00)?;
        self.ecrire_u8(0x77, 0x00)?;

        self.ecrire_u8(0xFF, 0x01)?; self.ecrire_u8(0x0D, 0x01)?;

        self.ecrire_u8(0xFF, 0x00)?; self.ecrire_u8(0x80, 0x01)?;
        self.ecrire_u8(0x01, 0xF8)?;

        self.ecrire_u8(0xFF, 0x01)?; self.ecrire_u8(0x8E, 0x01)?;
        self.ecrire_u8(0x00, 0x01)?;
        self.ecrire_u8(0xFF, 0x00)?; self.ecrire_u8(0x80, 0x00)?;

        Ok(())
    }

    /// Calibration de référence (VHV puis Phase)
    ///
    /// Identique à perform_calibration() du main.rs de référence.
    fn perform_ref_calibration(&mut self) -> Result<()> {
        // --- Calibration VHV ---
        self.ecrire_u8(SYSTEM_SEQUENCE_CONFIG, 0x01)?;
        self.ecrire_u8(SYSRANGE_START, 0x41)?;

        let mut timeout_ms = 0u32;
        loop {
            std::thread::sleep(std::time::Duration::from_millis(10));
            let val = self.lire_u8(RESULT_INTERRUPT_STATUS)?;
            if (val & 0x07) != 0 {
                println!("  Calibration VHV OK (INTERRUPT_STATUS=0x{:02X})", val);
                break;
            }
            timeout_ms += 10;
            if timeout_ms > 500 {
                return Err(ErreursAirHaum::ErreurCommunication(
                    "VL53L0X: Timeout calibration VHV".into()
                ));
            }
        }
        self.ecrire_u8(SYSTEM_INTERRUPT_CLEAR, 0x01)?;
        self.ecrire_u8(SYSRANGE_START, 0x00)?;

        // --- Calibration Phase ---
        self.ecrire_u8(SYSTEM_SEQUENCE_CONFIG, 0x02)?;
        self.ecrire_u8(SYSRANGE_START, 0x01)?;

        timeout_ms = 0;
        loop {
            std::thread::sleep(std::time::Duration::from_millis(10));
            let val = self.lire_u8(RESULT_INTERRUPT_STATUS)?;
            if (val & 0x07) != 0 {
                println!("  Calibration Phase OK (INTERRUPT_STATUS=0x{:02X})", val);
                break;
            }
            timeout_ms += 10;
            if timeout_ms > 500 {
                return Err(ErreursAirHaum::ErreurCommunication(
                    "VL53L0X: Timeout calibration Phase".into()
                ));
            }
        }
        self.ecrire_u8(SYSTEM_INTERRUPT_CLEAR, 0x01)?;
        self.ecrire_u8(SYSRANGE_START, 0x00)?;

        // Restaurer la séquence complète
        self.ecrire_u8(SYSTEM_SEQUENCE_CONFIG, 0xE8)?;

        Ok(())
    }

    // -------------------------------------------------------------------------
    // Mesure
    // -------------------------------------------------------------------------

    /// Démarre une mesure single-shot
    pub fn demarrer_mesure(&mut self) -> Result<()> {
        // Séquence start identique à la référence
        self.ecrire_u8(0x80, 0x01)?;
        self.ecrire_u8(0xFF, 0x01)?;
        self.ecrire_u8(0x00, 0x00)?;
        self.ecrire_u8(0x91, self.stop_variable)?;
        self.ecrire_u8(0x00, 0x01)?;
        self.ecrire_u8(0xFF, 0x00)?;
        self.ecrire_u8(0x80, 0x00)?;

        self.ecrire_u8(SYSRANGE_START, 0x01)?;

        Ok(())
    }

    /// Attend qu'une mesure soit disponible
    ///
    /// Surveille RESULT_INTERRUPT_STATUS (0x13) bits [2:0], comme la référence.
    fn attendre_mesure_disponible(&mut self) -> Result<()> {
        let debut = std::time::Instant::now();

        loop {
            // CORRECTION : registre 0x13 (RESULT_INTERRUPT_STATUS), masque 0x07
            let status = self.lire_u8(RESULT_INTERRUPT_STATUS)?;
            if (status & 0x07) != 0 {
                return Ok(());
            }

            if debut.elapsed().as_millis() > self.timeout_ms as u128 {
                return Err(ErreursAirHaum::ErreurCommunication(
                    "VL53L0X: Timeout mesure".into()
                ));
            }

            std::thread::sleep(std::time::Duration::from_millis(10));
        }
    }

    /// Lit la distance mesurée depuis les registres 0x1E / 0x1F
    ///
    /// Deux lectures u8 séparées pour éviter tout problème d'atomicité
    /// du lire_u16_be sur le bus I2C réel.
    fn lire_distance(&mut self) -> Result<u16> {
        // RESULT_RANGE_STATUS = 0x14, octet de distance à +10 = 0x1E (high) et 0x1F (low)
        let high = self.lire_u8(RESULT_RANGE_STATUS + 10)? as u16;
        let low  = self.lire_u8(RESULT_RANGE_STATUS + 11)? as u16;
        let distance = (high << 8) | low;

        // Clear interrupt
        self.ecrire_u8(SYSTEM_INTERRUPT_CLEAR, 0x01)?;

        Ok(distance)
    }

    /// Effectue une mesure complète (démarrage + attente + lecture)
    pub fn mesure_simple(&mut self) -> Result<u16> {
        self.demarrer_mesure()?;
        self.attendre_mesure_disponible()?;
        self.lire_distance()
    }

    // -------------------------------------------------------------------------
    // Utilitaires
    // -------------------------------------------------------------------------

    /// Configure le budget de temps de mesure
    #[allow(dead_code)]
    fn definir_budget_temps_mesure(&mut self, budget_us: u32) -> Result<()> {
        self.budget_temps_mesure_us = budget_us;
        Ok(())
    }

    /// Vérifie la cohérence d'une mesure
    fn verifier_coherence_distance(&self, distance: u16) -> bool {
        //use crate::types::constantes::DISTANCE_MAX_VL53L0X_MM;

        // 8190–8191 mm = valeur d'erreur du VL53L0X
        if distance == 0 {
            return false;
        }

        // Variation max : 1000 mm entre deux mesures consécutives
        if let Some(derniere) = self.derniere_distance {
            let variation = (distance as i32 - derniere as i32).abs();
            if variation > 1000 {
                return false;
            }
        }

        true
    }

    /// Détecte si une configuration valide existe en flash
    fn detecter_configuration_valide(&mut self) -> Result<bool> {
        if !self.verifier_identite()? {
            return Ok(false);
        }

        match crate::systeme::calibration::gestionnaire()
            .charger::<crate::drivers::telemetre::CalibrationTelemetre>()
        {
            Ok(Some(_)) => Ok(true),
            _ => Ok(false),
        }
    }

    /// Séquence d'initialisation complète avec gestion des états
    fn initialisation_avec_etats(&mut self) -> Result<()> {
        use std::time::{Duration, Instant};
        use crate::types::constantes::TIMEOUT_INIT_VL53L0X_MS;

        let debut = Instant::now();
        let timeout = Duration::from_millis(TIMEOUT_INIT_VL53L0X_MS);

        // 1. Vérification identité hardware
        if !self.verifier_identite()? {
            return Err(ErreursAirHaum::ErreurInitialisation(
                "VL53L0X: Model ID incorrect (attendu 0xEE)".into()
            ));
        }
        self.etat = EtatCapteur::NonConfigure;

        // 2. Data init (mode 2V8, stop_variable, GPIO, MSRC)
        self.data_init()?;

        // 3. Static init (tuning settings propriétaires ST)
        self.static_init()?;

        self.etat = EtatCapteur::Configure;

        if debut.elapsed() > timeout {
            self.etat = EtatCapteur::Inconnu;
            return Err(ErreursAirHaum::Timeout);
        }

        // 4. Calibrations de référence (VHV + Phase)
        self.perform_ref_calibration()?;

        // 5. Test de mesure pour valider l'ensemble
        let _ = self.mesure_simple()?;

        self.etat = EtatCapteur::nouveau_operationnel();

        // 6. Persister la calibration (pour reprise rapide au prochain démarrage)
        crate::systeme::calibration::gestionnaire()
            .sauvegarder(&crate::drivers::telemetre::CalibrationTelemetre::nouvelle(3600))?;

        println!("VL53L0X: Initialisation OK !");
        Ok(())
    }

    // -------------------------------------------------------------------------
    // Méthodes utilitaires I²C
    // -------------------------------------------------------------------------

    /// Écriture d'un registre 8 bits
    ///
    /// CORRECTION : utilise `ecrire()` (transaction [reg, val]) et non
    /// `ecrire_lire()` qui implique une lecture en retour inutile et fausse.
    fn ecrire_u8(&mut self, registre: u8, valeur: u8) -> Result<()> {
        self.i2c.ecrire(self.adresse, &[registre, valeur])
    }

    /// Lecture d'un registre 8 bits
    fn lire_u8(&mut self, registre: u8) -> Result<u8> {
        self.i2c.lire_registre_u8(self.adresse, registre)
    }

    /// Lecture d'un registre 16 bits big-endian (non utilisé directement,
    /// remplacé par deux lire_u8 dans lire_distance pour plus de robustesse)
    #[allow(dead_code)]
    fn lire_u16_be(&mut self, registre: u8) -> Result<u16> {
        self.i2c.lire_registre_u16_be(self.adresse, registre)
    }
}

// =============================================================================
// Implémentation du trait Telemetre
// =============================================================================

impl<I2C: BusI2c> Telemetre for Vl53l0x<I2C> {
    fn initialiser(&mut self) -> Result<()> {
        // Tentative de reprise rapide si une calibration valide existe
        if self.detecter_configuration_valide()? {
            println!("⚡ VL53L0X: Reprise rapide - pas de calibration");

            self.data_init()?;
            self.static_init()?;

            self.etat = EtatCapteur::nouveau_operationnel();
            return Ok(());
        }

        // Sinon, initialisation complète
        println!("🔧 VL53L0X: Initialisation complète");
        self.initialisation_avec_etats()
    }

    fn mesurer_distance(&mut self) -> Result<u16> {
        use crate::types::DISTANCE_MAX_VL53L0X_MM;

        if !self.etat.est_utilisable() {
            return Err(ErreursAirHaum::CapteurNonInitialise(
                format!("VL53L0X non opérationnel (état: {})", self.etat)
            ));
        }

        let distance = self.mesure_simple()?;

        // Hors portée : comportement normal du capteur, pas une erreur matérielle
        if distance >= DISTANCE_MAX_VL53L0X_MM {
          return Err(ErreursAirHaum::HorsPortee);
        }

        // Incohérence réelle (zéro, variation anormale)
        if !self.verifier_coherence_distance(distance) {
            self.etat = EtatCapteur::nouveau_degrade(
                format!("Variation anormale: {}mm", distance)
            );
            return Err(ErreursAirHaum::DonneesInvalides(
                format!("VL53L0X: distance={}mm", distance)
            ));
        }

        self.derniere_distance = Some(distance);
        self.derniere_lecture = Some(Horodatage::maintenant());

        Ok(distance)
    }

    fn est_pret(&mut self) -> Result<bool> {
        Ok(self.etat.est_utilisable())
    }

    fn obtenir_precision(&self) -> u16 {
        10 // ±10 mm
    }

    fn obtenir_portee_max(&self) -> u16 {
        2000 // 2000 mm = 2 m
    }

    fn obtenir_etat(&self) -> &EtatCapteur {
        &self.etat
    }
}

// =============================================================================
// Tests
// =============================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::hal::i2c::I2cMock;

    #[test]
    fn test_creation_driver() {
        let i2c = I2cMock::nouveau();
        let _vl53 = Vl53l0x::nouveau(i2c, ADRESSE_VL53L0X);
    }

    #[test]
    fn test_verification_identite() {
        let mut i2c = I2cMock::nouveau();
        i2c.precharger_registre(ADRESSE_VL53L0X, IDENTIFICATION_MODEL_ID, 0xEE);

        let mut vl53 = Vl53l0x::nouveau(i2c, ADRESSE_VL53L0X);
        assert!(vl53.verifier_identite().unwrap());
    }

    #[test]
    fn test_identite_incorrecte() {
        let mut i2c = I2cMock::nouveau();
        i2c.precharger_registre(ADRESSE_VL53L0X, IDENTIFICATION_MODEL_ID, 0xAB);

        let mut vl53 = Vl53l0x::nouveau(i2c, ADRESSE_VL53L0X);
        assert!(!vl53.verifier_identite().unwrap());
    }
}
