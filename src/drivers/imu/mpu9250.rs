// src/drivers/imu/mpu9250.rs
//! Driver pour la centrale inertielle MPU9250
//!
//! Le MPU9250 est un IMU 9 axes (gyroscope + accéléromètre + magnétomètre AK8963)
//! communiquant en I²C. Le magnétomètre AK8963 est accessible via le bus I²C
//! interne du MPU9250 en mode I2C master.
//!
//! ## Architecture I²C
//!
//! ```text
//! Orange Pi ──I2C──► MPU9250 (0x68)
//!                       │
//!                       └──I2C interne──► AK8963 (0x0C)
//! ```
//!
//! Le MPU9250 lit automatiquement l'AK8963 via ses registres SLV0 et place
//! les données magnétomètre dans EXT_SENS_DATA_00..06. Une seule transaction
//! I²C depuis le Pi récupère ainsi les 9 axes.

use crate::hal::BusI2c;
use crate::types::{
    Result, ErreursAirHaum, EtatCapteur, Horodatage,
    DonneesImu, Vector3, Temperature,
};
use crate::interfaces::imu::CentraleInertielle;
use crate::drivers::imu::calibration::{CalibrationGyro, CalibrationAccel, CalibrationMag};
use crate::drivers::imu::calcul_calibration::{
    EchantillonCapteur, Matrice3, MONTAGE_DEFAUT, appliquer_matrice,
};
use crate::systeme::calibration::{EtatCalibration, CalibrationPersistante};

/// Adresse I²C par défaut du MPU9250 (AD0 = GND)
pub const ADRESSE_MPU9250: u8 = 0x68;
/// Adresse I²C alternative (AD0 = VCC)
pub const ADRESSE_MPU9250_ALT: u8 = 0x69;
/// Adresse I²C du magnétomètre AK8963 (fixe, bus interne)
const ADRESSE_AK8963: u8 = 0x0C;

/// Facteur de conversion gyro ±250°/s → rad/s
/// LSB sensitivity = 131.0 LSB/(°/s), puis °/s → rad/s
const GYRO_SCALE: f32 = 1.0 / 131.0 * (std::f32::consts::PI / 180.0);

/// Facteur de conversion accel ±4g → m/s²
/// LSB sensitivity = 8192.0 LSB/g
const ACCEL_SCALE: f32 = 9.80665 / 8192.0;

/// Facteur de conversion magnétomètre AK8963 mode 16 bits → µT
/// 0.15 µT/LSB en 16 bits
const MAG_SCALE: f32 = 0.15;

/// Registres MPU9250
#[allow(dead_code)]
mod reg {
    pub const SELF_TEST_X_GYRO:  u8 = 0x00;
    pub const SELF_TEST_Y_GYRO:  u8 = 0x01;
    pub const SELF_TEST_Z_GYRO:  u8 = 0x02;
    pub const SELF_TEST_X_ACCEL: u8 = 0x0D;
    pub const SELF_TEST_Y_ACCEL: u8 = 0x0E;
    pub const SELF_TEST_Z_ACCEL: u8 = 0x0F;
    pub const SMPLRT_DIV:        u8 = 0x19;
    pub const CONFIG:            u8 = 0x1A;
    pub const GYRO_CONFIG:       u8 = 0x1B;
    pub const ACCEL_CONFIG:      u8 = 0x1C;
    pub const ACCEL_CONFIG2:     u8 = 0x1D;
    pub const LP_ACCEL_ODR:      u8 = 0x1E;
    pub const WOM_THR:           u8 = 0x1F;
    pub const FIFO_EN:           u8 = 0x23;
    pub const I2C_MST_CTRL:      u8 = 0x24;
    pub const I2C_SLV0_ADDR:     u8 = 0x25;
    pub const I2C_SLV0_REG:      u8 = 0x26;
    pub const I2C_SLV0_CTRL:     u8 = 0x27;
    pub const INT_PIN_CFG:       u8 = 0x37;
    pub const INT_ENABLE:        u8 = 0x38;
    pub const INT_STATUS:        u8 = 0x3A;
    pub const ACCEL_XOUT_H:      u8 = 0x3B;
    pub const ACCEL_XOUT_L:      u8 = 0x3C;
    pub const ACCEL_YOUT_H:      u8 = 0x3D;
    pub const ACCEL_YOUT_L:      u8 = 0x3E;
    pub const ACCEL_ZOUT_H:      u8 = 0x3F;
    pub const ACCEL_ZOUT_L:      u8 = 0x40;
    pub const TEMP_OUT_H:        u8 = 0x41;
    pub const TEMP_OUT_L:        u8 = 0x42;
    pub const GYRO_XOUT_H:       u8 = 0x43;
    pub const GYRO_XOUT_L:       u8 = 0x44;
    pub const GYRO_YOUT_H:       u8 = 0x45;
    pub const GYRO_YOUT_L:       u8 = 0x46;
    pub const GYRO_ZOUT_H:       u8 = 0x47;
    pub const GYRO_ZOUT_L:       u8 = 0x48;
    pub const EXT_SENS_DATA_00:  u8 = 0x49; // données AK8963 (7 octets)
    pub const I2C_MST_STATUS:    u8 = 0x36;
    pub const USER_CTRL:         u8 = 0x6A;
    pub const PWR_MGMT_1:        u8 = 0x6B;
    pub const PWR_MGMT_2:        u8 = 0x6C;
    pub const WHO_AM_I:          u8 = 0x75;
}

/// Registres AK8963 (magnétomètre)
#[allow(dead_code)]
mod reg_ak {
    pub const WHO_AM_I:  u8 = 0x00; // doit retourner 0x48
    pub const INFO:      u8 = 0x01;
    pub const ST1:       u8 = 0x02; // data ready
    pub const HXL:       u8 = 0x03; // début données mag (6 octets)
    pub const HXH:       u8 = 0x04;
    pub const HYL:       u8 = 0x05;
    pub const HYH:       u8 = 0x06;
    pub const HZL:       u8 = 0x07;
    pub const HZH:       u8 = 0x08;
    pub const ST2:       u8 = 0x09; // overflow / mode
    pub const CNTL1:     u8 = 0x0A; // mode de mesure
    pub const CNTL2:     u8 = 0x0B; // soft reset
    pub const ASTC:      u8 = 0x0C; // self-test
    pub const ASAX:      u8 = 0x10; // sensitivity X (3 octets: X, Y, Z)
    pub const ASAY:      u8 = 0x11;
    pub const ASAZ:      u8 = 0x12;
}

/// Driver MPU9250
pub struct Mpu9250<I2C: BusI2c> {
    i2c: I2C,
    adresse: u8,
    etat: EtatCapteur,

    // Calibrations en mémoire
    calib_gyro:  Option<CalibrationGyro>,
    calib_accel: Option<CalibrationAccel>,
    calib_mag:   Option<CalibrationMag>,

    /// Coefficients de sensibilité usine AK8963, relus à chaque initialisation
    /// (repère capteur, axes réalignés)
    asa: Vector3,

    // Dernière lecture pour cohérence temporelle
    derniere_lecture: Option<Horodatage>,
}

impl<I2C: BusI2c> Mpu9250<I2C> {
    /// Crée une nouvelle instance du driver MPU9250
    pub fn nouveau(i2c: I2C, adresse: u8) -> Self {
        Self {
            i2c,
            adresse,
            etat: EtatCapteur::Inconnu,
            calib_gyro:  None,
            calib_accel: None,
            calib_mag:   None,
            asa: Vector3::nouveau(1.0, 1.0, 1.0),
            derniere_lecture: None,
        }
    }

    // =========================================================================
    // Vérification identité
    // =========================================================================

    /// Vérifie le WHO_AM_I du MPU9250 (doit retourner 0x73)
    pub fn verifier_identite(&mut self) -> Result<bool> {
        let id = self.lire_u8(reg::WHO_AM_I)?;
        log::debug!(target: "imu", "MPU9250 WHO_AM_I: 0x{:02X}", id);
        Ok(id == 0x73)
    }

    // =========================================================================
    // Initialisation
    // =========================================================================

    /// Reset matériel complet, puis attente stabilisation
    fn reset(&mut self) -> Result<()> {
        self.ecrire_u8(reg::PWR_MGMT_1, 0x80)?; // H_RESET
        std::thread::sleep(std::time::Duration::from_millis(100));
        Ok(())
    }

    /// Réveil + sélection horloge PLL automatique
    fn reveiller(&mut self) -> Result<()> {
        // CLKSEL = 1 (PLL gyro X), SLEEP = 0
        self.ecrire_u8(reg::PWR_MGMT_1, 0x01)?;
        std::thread::sleep(std::time::Duration::from_millis(10));
        Ok(())
    }

    /// Configure le gyroscope en ±250°/s, DLPF activé
    fn configurer_gyro(&mut self) -> Result<()> {
        // GYRO_CONFIG : FS_SEL = 0 (±250°/s), Fchoice_b = 00
        self.ecrire_u8(reg::GYRO_CONFIG, 0x00)?;
        // CONFIG : DLPF_CFG = 3 (41Hz bandwidth, 1kHz Fs)
        self.ecrire_u8(reg::CONFIG, 0x03)?;
        // SMPLRT_DIV = 4 → 200Hz (1000 / (1+4))
        self.ecrire_u8(reg::SMPLRT_DIV, 0x04)?;
        Ok(())
    }

    /// Configure l'accéléromètre en ±4g, DLPF activé
    fn configurer_accel(&mut self) -> Result<()> {
        // ACCEL_CONFIG : AFS_SEL = 1 (±4g)
        self.ecrire_u8(reg::ACCEL_CONFIG, 0x08)?;
        // ACCEL_CONFIG2 : DLPF = 3 (41Hz bandwidth)
        self.ecrire_u8(reg::ACCEL_CONFIG2, 0x03)?;
        Ok(())
    }

    /// Active le mode I2C master pour piloter l'AK8963 en interne
    fn activer_i2c_master(&mut self) -> Result<()> {
        // Désactiver bypass I2C (le Pi ne parle plus directement à l'AK8963)
        let cfg = self.lire_u8(reg::INT_PIN_CFG)?;
        self.ecrire_u8(reg::INT_PIN_CFG, cfg & !0x02)?;

        // Activer I2C master dans USER_CTRL
        let uc = self.lire_u8(reg::USER_CTRL)?;
        self.ecrire_u8(reg::USER_CTRL, uc | 0x20)?;

        // I2C master clock : 400kHz (valeur 0x0D)
        self.ecrire_u8(reg::I2C_MST_CTRL, 0x0D)?;
        std::thread::sleep(std::time::Duration::from_millis(10));
        Ok(())
    }

    /// Écrit un registre de l'AK8963 via le bus I2C master du MPU9250
    fn ak_ecrire(&mut self, registre_ak: u8, valeur: u8) -> Result<()> {
        // SLV0_ADDR : bit7 = 0 (écriture), adresse AK8963
        self.ecrire_u8(reg::I2C_SLV0_ADDR, ADRESSE_AK8963)?;
        self.ecrire_u8(reg::I2C_SLV0_REG,  registre_ak)?;
        // DO : donnée à écrire
        self.ecrire_u8(0x63, valeur)?; // I2C_SLV0_DO
        // SLV0_CTRL : EN=1, LENG=1
        self.ecrire_u8(reg::I2C_SLV0_CTRL, 0x81)?;
        std::thread::sleep(std::time::Duration::from_millis(10));
        Ok(())
    }

    /// Lit N registres de l'AK8963 via le bus I2C master, résultat dans EXT_SENS_DATA
    fn ak_lire_setup(&mut self, registre_ak: u8, n: u8) -> Result<()> {
        // SLV0_ADDR : bit7 = 1 (lecture), adresse AK8963
        self.ecrire_u8(reg::I2C_SLV0_ADDR, ADRESSE_AK8963 | 0x80)?;
        self.ecrire_u8(reg::I2C_SLV0_REG,  registre_ak)?;
        // SLV0_CTRL : EN=1, LENG=n
        self.ecrire_u8(reg::I2C_SLV0_CTRL, 0x80 | n)?;
        std::thread::sleep(std::time::Duration::from_millis(10));
        Ok(())
    }

    /// Initialise l'AK8963 et lit les coefficients de sensibilité usine (ASA)
    ///
    /// Retourne les facteurs multiplicatifs corrigés Hadj = (ASA - 128) / 256 + 1,
    /// dans le repère capteur (axes réalignés sur l'accéléromètre).
    fn init_ak8963(&mut self) -> Result<Vector3> {
        // Reset AK8963
        self.ak_ecrire(reg_ak::CNTL2, 0x01)?;
        std::thread::sleep(std::time::Duration::from_millis(10));

        // Passer en mode Power Down avant de changer de mode
        self.ak_ecrire(reg_ak::CNTL1, 0x00)?;
        std::thread::sleep(std::time::Duration::from_millis(10));

        // Passer en mode Fuse ROM Access pour lire les ASA
        self.ak_ecrire(reg_ak::CNTL1, 0x0F)?;
        std::thread::sleep(std::time::Duration::from_millis(10));

        // Lire les 3 octets ASA depuis EXT_SENS_DATA via SLV0
        self.ak_lire_setup(reg_ak::ASAX, 3)?;
        std::thread::sleep(std::time::Duration::from_millis(10));

        let mut buf = [0u8; 3];
        self.lire_multi(reg::EXT_SENS_DATA_00, &mut buf)?;

        let asa_x = (buf[0] as f32 - 128.0) / 256.0 + 1.0;
        let asa_y = (buf[1] as f32 - 128.0) / 256.0 + 1.0;
        let asa_z = (buf[2] as f32 - 128.0) / 256.0 + 1.0;

        log::debug!(target: "imu", "AK8963 ASA: x={:.4} y={:.4} z={:.4}", asa_x, asa_y, asa_z);

        // Repasser en Power Down
        self.ak_ecrire(reg_ak::CNTL1, 0x00)?;
        std::thread::sleep(std::time::Duration::from_millis(10));

        // Activer mode continu 2 (100Hz), 16 bits
        // CNTL1 : MODE=0110 (continu 2), BIT=1 (16 bits)
        self.ak_ecrire(reg_ak::CNTL1, 0x16)?;
        std::thread::sleep(std::time::Duration::from_millis(10));

        // Configurer SLV0 pour lire en continu ST1 + 6 octets HX..HZ + ST2 = 8 octets
        self.ak_lire_setup(reg_ak::ST1, 8)?;

        // Repère capteur : axes réalignés comme les mesures (voir mag_repere_capteur)
        Ok(Vector3::nouveau(asa_y, asa_x, asa_z))
    }

    /// Séquence d'initialisation complète
    fn init_complete(&mut self) -> Result<()> {
        // 1. Vérification identité
        if !self.verifier_identite()? {
            return Err(ErreursAirHaum::ErreurInitialisation(
                "MPU9250: WHO_AM_I incorrect (attendu 0x73)".into()
            ));
        }
        self.etat = EtatCapteur::NonConfigure;

        // 2. Reset + réveil
        self.reset()?;
        self.reveiller()?;

        // 3. Configuration gyro + accel
        self.configurer_gyro()?;
        self.configurer_accel()?;
        self.etat = EtatCapteur::Configure;

        // 4. I2C master + init AK8963
        self.activer_i2c_master()?;
        self.asa = self.init_ak8963()?;

        // 5. Calibrations
        self.charger_calibrations();

        self.etat = EtatCapteur::nouveau_operationnel();
        log::info!(target: "imu", "MPU9250 : initialisation OK");
        Ok(())
    }

    /// Charge les calibrations enregistrées, **même expirées** : l'expiration
    /// est un critère d'armement, pas d'utilisation (`doc/calibration.md`).
    /// Aucune calibration automatique : une calibration absente reste absente
    /// et la check-liste d'armement la signale.
    fn charger_calibrations(&mut self) {
        fn charger<C: CalibrationPersistante>(nom: &str) -> Option<C> {
            match crate::systeme::calibration::gestionnaire().inspecter::<C>() {
                EtatCalibration::Valide(c) => {
                    log::info!(target: "imu", "MPU9250 : calibration {} chargée", nom);
                    Some(c)
                }
                EtatCalibration::Expiree(c) => {
                    log::warn!(target: "imu", "MPU9250 : calibration {} expirée — appliquée, à renouveler avant armement", nom);
                    Some(c)
                }
                EtatCalibration::Absente => {
                    log::warn!(target: "imu", "MPU9250 : calibration {} absente", nom);
                    None
                }
                EtatCalibration::Corrompue(e) => {
                    log::warn!(target: "imu", "MPU9250 : calibration {} inutilisable ({}) — à refaire", nom, e);
                    None
                }
            }
        }
        self.calib_gyro  = charger::<CalibrationGyro>("gyroscope");
        self.calib_accel = charger::<CalibrationAccel>("accéléromètre (montage)");
        self.calib_mag   = charger::<CalibrationMag>("magnétomètre");
        if self.calib_accel.is_none() {
            log::warn!(target: "imu", "MPU9250 : montage par défaut (carte à plat, X vers le nez)");
        }
    }

    // =========================================================================
    // Lecture dans le repère capteur (procédures de calibration)
    // =========================================================================

    /// Lit une mesure dans le repère capteur, **sans aucune calibration**
    /// (unités SI, ASA appliqués, axes AK8963 réalignés). Sert aux
    /// procédures de calibration (`diagnostiques/diag_mpu9250.rs`).
    pub fn lire_repere_capteur(&mut self) -> Result<EchantillonCapteur> {
        let (ax, ay, az, _temp, gx, gy, gz) = self.lire_tout_brut()?;
        let mag = self.lire_mag_brut()?.map(|(mx, my, mz)| self.mag_repere_capteur(mx, my, mz));
        Ok(EchantillonCapteur {
            accel: Vector3::nouveau(ax as f32, ay as f32, az as f32) * ACCEL_SCALE,
            gyro:  Vector3::nouveau(gx as f32, gy as f32, gz as f32) * GYRO_SCALE,
            mag,
        })
    }

    // =========================================================================
    // Lecture des données brutes
    // =========================================================================

    /// Lit toutes les données en une seule transaction I²C (14 octets)
    ///
    /// Ordre : ACCEL_X H/L, ACCEL_Y H/L, ACCEL_Z H/L,
    ///         TEMP H/L,
    ///         GYRO_X H/L, GYRO_Y H/L, GYRO_Z H/L
    fn lire_tout_brut(&mut self) -> Result<(i16, i16, i16, i16, i16, i16, i16)> {
        let mut buf = [0u8; 14];
        self.lire_multi(reg::ACCEL_XOUT_H, &mut buf)?;
        Ok((
            i16::from_be_bytes([buf[0],  buf[1]]),   // accel X
            i16::from_be_bytes([buf[2],  buf[3]]),   // accel Y
            i16::from_be_bytes([buf[4],  buf[5]]),   // accel Z
            i16::from_be_bytes([buf[6],  buf[7]]),   // temp
            i16::from_be_bytes([buf[8],  buf[9]]),   // gyro X
            i16::from_be_bytes([buf[10], buf[11]]),  // gyro Y
            i16::from_be_bytes([buf[12], buf[13]]),  // gyro Z
        ))
    }

    /// Lit les données magnétomètre depuis EXT_SENS_DATA (placées par le master I2C)
    ///
    /// Format ST1 (1) + HX/HY/HZ (6) + ST2 (1) = 8 octets
    fn lire_mag_brut(&mut self) -> Result<Option<(i16, i16, i16)>> {
        let mut buf = [0u8; 8];
        self.lire_multi(reg::EXT_SENS_DATA_00, &mut buf)?;

        let st1 = buf[0];
        let st2 = buf[7];

        // DRDY (data ready) non set → pas de nouvelles données
        if st1 & 0x01 == 0 {
            return Ok(None);
        }

        // HOFL (overflow) → données invalides
        if st2 & 0x08 != 0 {
            return Ok(None);
        }

        // AK8963 : little-endian
        Ok(Some((
            i16::from_le_bytes([buf[1], buf[2]]), // HX
            i16::from_le_bytes([buf[3], buf[4]]), // HY
            i16::from_le_bytes([buf[5], buf[6]]), // HZ
        )))
    }

    // =========================================================================
    // Application des calibrations
    // =========================================================================
    //
    // Repère de sortie : avion (X nez, Y aile droite, Z bas), conventions de
    // `estimation/attitude.rs` — voir `doc/calibration.md`, « Repère ».

    fn montage(&self) -> Matrice3 {
        self.calib_accel.map(|c| c.montage).unwrap_or(MONTAGE_DEFAUT)
    }

    fn appliquer_calib_gyro(&self, gx: i16, gy: i16, gz: i16) -> Vector3 {
        let biais = self.calib_gyro
            .map(|c| Vector3::nouveau(c.offset_x, c.offset_y, c.offset_z))
            .unwrap_or(Vector3::zero());
        let w = Vector3::nouveau(gx as f32, gy as f32, gz as f32) * GYRO_SCALE - biais;
        appliquer_matrice(&self.montage(), w)
    }

    /// Accéléromètre en gravité apparente (+g sur Z avion immobile à plat).
    fn appliquer_calib_accel(&self, ax: i16, ay: i16, az: i16) -> Vector3 {
        // Décalage Z de la session s'il existe, sinon celui de l'atelier
        let decalage_z = self.calib_gyro.map(|c| c.offset_z_accel)
            .or(self.calib_accel.map(|c| c.offset_z))
            .unwrap_or(0.0);
        let f = Vector3::nouveau(
            ax as f32 * ACCEL_SCALE,
            ay as f32 * ACCEL_SCALE,
            az as f32 * ACCEL_SCALE - decalage_z,
        );
        appliquer_matrice(&self.montage(), f) * -1.0
    }

    /// Conversion µT avec ASA, puis réalignement des axes AK8963 sur ceux de
    /// l'accéléromètre (fiche technique MPU9250 : X_mag = Y_accel,
    /// Y_mag = X_accel, Z_mag = −Z_accel). `self.asa` est déjà réaligné.
    fn mag_repere_capteur(&self, mx: i16, my: i16, mz: i16) -> Vector3 {
        Vector3::nouveau(
             my as f32 * MAG_SCALE * self.asa.x,
             mx as f32 * MAG_SCALE * self.asa.y,
            -(mz as f32) * MAG_SCALE * self.asa.z,
        )
    }

    fn appliquer_calib_mag(&self, mx: i16, my: i16, mz: i16) -> Vector3 {
        let decalage = self.calib_mag
            .map(|c| Vector3::nouveau(c.hard_iron_x, c.hard_iron_y, c.hard_iron_z))
            .unwrap_or(Vector3::zero());
        appliquer_matrice(&self.montage(), self.mag_repere_capteur(mx, my, mz) - decalage)
    }

    // =========================================================================
    // Utilitaires I²C
    // =========================================================================

    fn ecrire_u8(&mut self, registre: u8, valeur: u8) -> Result<()> {
        self.i2c.ecrire(self.adresse, &[registre, valeur])
    }

    fn lire_u8(&mut self, registre: u8) -> Result<u8> {
        self.i2c.lire_registre_u8(self.adresse, registre)
    }

    fn lire_multi(&mut self, registre: u8, buffer: &mut [u8]) -> Result<()> {
        self.i2c.ecrire_lire(self.adresse, &[registre], buffer)
    }
}

// =============================================================================
// Implémentation du trait CentraleInertielle
// =============================================================================

impl<I2C: BusI2c> CentraleInertielle for Mpu9250<I2C> {
    fn initialiser(&mut self) -> Result<()> {
        // Vérification identité rapide d'abord
        if !self.verifier_identite()? {
            return Err(ErreursAirHaum::ErreurInitialisation(
                "MPU9250: WHO_AM_I incorrect".into()
            ));
        }

        // Reprise rapide (sans reset matériel) si gyroscope et montage sont
        // déjà calibrés, même expirés : cas d'un redémarrage en vol
        let gest = crate::systeme::calibration::gestionnaire();
        let gyro_ok  = matches!(gest.inspecter::<CalibrationGyro>(),
                                EtatCalibration::Valide(_) | EtatCalibration::Expiree(_));
        let accel_ok = matches!(gest.inspecter::<CalibrationAccel>(),
                                EtatCalibration::Valide(_) | EtatCalibration::Expiree(_));

        if gyro_ok && accel_ok {
            log::info!(target: "imu", "MPU9250 : reprise rapide");
            // On réapplique quand même la config hardware (reset programme)
            self.reveiller()?;
            self.configurer_gyro()?;
            self.configurer_accel()?;
            self.activer_i2c_master()?;
            self.asa = self.init_ak8963()?;
            self.charger_calibrations();

            self.etat = EtatCapteur::nouveau_operationnel();
            return Ok(());
        }

        // Initialisation complète
        log::info!(target: "imu", "MPU9250 : initialisation complète");
        self.init_complete()
    }

    fn lire(&mut self) -> Result<DonneesImu> {
        if !self.etat.est_utilisable() {
            return Err(ErreursAirHaum::CapteurNonInitialise(
                format!("MPU9250 non opérationnel (état: {})", self.etat)
            ));
        }

        // Lecture accel + temp + gyro en une transaction
        let (ax, ay, az, temp_brut, gx, gy, gz) = self.lire_tout_brut()?;

        // Température : TEMP_degC = (TEMP_OUT / 333.87) + 21.0
        let temperature = Temperature::depuis_celsius(
            temp_brut as f32 / 333.87 + 21.0
        );

        // Application calibrations
        let gyroscope    = self.appliquer_calib_gyro(gx, gy, gz);
        let accelerometre = self.appliquer_calib_accel(ax, ay, az);

        // Magnétomètre (peut ne pas avoir de nouvelles données à chaque cycle)
        let magnetometre = match self.lire_mag_brut()? {
            Some((mx, my, mz)) => self.appliquer_calib_mag(mx, my, mz),
            None => Vector3::nouveau(0.0, 0.0, 0.0), // pas de nouvelles données
        };

        let horodatage = Horodatage::maintenant();
        self.derniere_lecture = Some(horodatage);

        Ok(DonneesImu {
            horodatage,
            accelerometre,
            gyroscope,
            magnetometre,
            temperature,
        })
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
        let _mpu = Mpu9250::nouveau(i2c, ADRESSE_MPU9250);
    }

    #[test]
    fn test_verification_identite() {
        let mut i2c = I2cMock::nouveau();
        i2c.precharger_registre(ADRESSE_MPU9250, reg::WHO_AM_I, 0x73);
        let mut mpu = Mpu9250::nouveau(i2c, ADRESSE_MPU9250);
        assert!(mpu.verifier_identite().unwrap());
    }

    #[test]
    fn test_identite_incorrecte() {
        let mut i2c = I2cMock::nouveau();
        i2c.precharger_registre(ADRESSE_MPU9250, reg::WHO_AM_I, 0xAB);
        let mut mpu = Mpu9250::nouveau(i2c, ADRESSE_MPU9250);
        assert!(!mpu.verifier_identite().unwrap());
    }

    #[test]
    fn test_appliquer_calib_gyro_sans_calib() {
        let i2c = I2cMock::nouveau();
        let mpu = Mpu9250::nouveau(i2c, ADRESSE_MPU9250);
        // Sans calibration, doit appliquer uniquement le scale
        let v = mpu.appliquer_calib_gyro(0, 0, 0);
        assert_eq!(v.x, 0.0);
        assert_eq!(v.y, 0.0);
        assert_eq!(v.z, 0.0);
    }
}
