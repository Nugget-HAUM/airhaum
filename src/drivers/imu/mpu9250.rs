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
            derniere_lecture: None,
        }
    }

    // =========================================================================
    // Vérification identité
    // =========================================================================

    /// Vérifie le WHO_AM_I du MPU9250 (doit retourner 0x71)
    pub fn verifier_identite(&mut self) -> Result<bool> {
        let id = self.lire_u8(reg::WHO_AM_I)?;
        println!("MPU9250 WHO_AM_I: 0x{:02X} (attendu: 0x73)", id);
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
    /// Retourne (asa_x, asa_y, asa_z) comme facteurs multiplicatifs corrigés :
    /// Hadj = (ASA - 128) / 256 + 1
    fn init_ak8963(&mut self) -> Result<(f32, f32, f32)> {
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

        println!("AK8963 ASA: x={:.4} y={:.4} z={:.4}", asa_x, asa_y, asa_z);

        // Repasser en Power Down
        self.ak_ecrire(reg_ak::CNTL1, 0x00)?;
        std::thread::sleep(std::time::Duration::from_millis(10));

        // Activer mode continu 2 (100Hz), 16 bits
        // CNTL1 : MODE=0110 (continu 2), BIT=1 (16 bits)
        self.ak_ecrire(reg_ak::CNTL1, 0x16)?;
        std::thread::sleep(std::time::Duration::from_millis(10));

        // Configurer SLV0 pour lire en continu ST1 + 6 octets HX..HZ + ST2 = 8 octets
        self.ak_lire_setup(reg_ak::ST1, 8)?;

        Ok((asa_x, asa_y, asa_z))
    }

    /// Séquence d'initialisation complète
    fn init_complete(&mut self) -> Result<()> {
        // 1. Vérification identité
        if !self.verifier_identite()? {
            return Err(ErreursAirHaum::ErreurInitialisation(
                "MPU9250: WHO_AM_I incorrect (attendu 0x71)".into()
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
        let (asa_x, asa_y, asa_z) = self.init_ak8963()?;

        // 5. Charger ou créer calibrations
        self.charger_ou_creer_calibrations(asa_x, asa_y, asa_z)?;

        self.etat = EtatCapteur::nouveau_operationnel();
        println!("MPU9250: Initialisation OK");
        Ok(())
    }

    /// Charge les calibrations depuis le disque, ou les crée si absentes/expirées
    fn charger_ou_creer_calibrations(
        &mut self,
        asa_x: f32, asa_y: f32, asa_z: f32
    ) -> Result<()> {
        let gest = crate::systeme::calibration::gestionnaire();

        // --- Gyro ---
        self.calib_gyro = match gest.charger::<CalibrationGyro>()? {
            Some(c) => {
                println!("⚡ MPU9250: Calibration gyro chargée");
                Some(c)
            }
            None => {
                println!("🔧 MPU9250: Calibration gyro manquante, calibration...");
                let c = self.effectuer_calibration_gyro()?;
                gest.sauvegarder(&c)?;
                Some(c)
            }
        };

        // --- Accel ---
        self.calib_accel = match gest.charger::<CalibrationAccel>()? {
            Some(c) => {
                println!("⚡ MPU9250: Calibration accel chargée");
                Some(c)
            }
            None => {
                println!("🔧 MPU9250: Calibration accel manquante, calibration...");
                let c = self.effectuer_calibration_accel()?;
                gest.sauvegarder(&c)?;
                Some(c)
            }
        };

        // --- Mag ---
        self.calib_mag = match gest.charger::<CalibrationMag>()? {
            Some(c) => {
                println!("⚡ MPU9250: Calibration mag chargée");
                Some(c)
            }
            None => {
                // Première fois : on sauvegarde avec ASA uniquement,
                // l'opérateur devra lancer la procédure mag complète avant l'armement
                println!("⚠ MPU9250: Calibration mag absente, utilisation ASA usine uniquement");
                println!("  → Lancer calibrer_mag() avant l'armement");
                let c = CalibrationMag::depuis_asa_uniquement(asa_x, asa_y, asa_z);
                gest.sauvegarder(&c)?;
                Some(c)
            }
        };

        Ok(())
    }

    // =========================================================================
    // Procédures de calibration
    // =========================================================================

    /// Calibration gyroscope : moyenne sur 200 échantillons, capteur immobile
    fn effectuer_calibration_gyro(&mut self) -> Result<CalibrationGyro> {
        println!("  Calibration gyro : ne pas bouger le capteur (5s)...");
        const N: i32 = 200;
        let mut sum_x = 0i32;
        let mut sum_y = 0i32;
        let mut sum_z = 0i32;

        for _ in 0..N {
            let (gx, gy, gz) = self.lire_gyro_brut()?;
            sum_x += gx as i32;
            sum_y += gy as i32;
            sum_z += gz as i32;
            std::thread::sleep(std::time::Duration::from_millis(25));
        }

        let offset_x = (sum_x / N) as f32 * GYRO_SCALE;
        let offset_y = (sum_y / N) as f32 * GYRO_SCALE;
        let offset_z = (sum_z / N) as f32 * GYRO_SCALE;

        println!("  Gyro offsets: x={:.5} y={:.5} z={:.5} rad/s",
                 offset_x, offset_y, offset_z);

        Ok(CalibrationGyro::nouvelle(offset_x, offset_y, offset_z))
    }

    /// Calibration accéléromètre : sol plat, Z aligné avec gravité
    fn effectuer_calibration_accel(&mut self) -> Result<CalibrationAccel> {
        println!("  Calibration accel : poser le capteur à plat (2s)...");
        std::thread::sleep(std::time::Duration::from_millis(500));

        const N: i32 = 100;
        let mut sum_x = 0i32;
        let mut sum_y = 0i32;
        let mut sum_z = 0i32;

        for _ in 0..N {
            let (ax, ay, az) = self.lire_accel_brut()?;
            sum_x += ax as i32;
            sum_y += ay as i32;
            sum_z += az as i32;
            std::thread::sleep(std::time::Duration::from_millis(10));
        }

        let mean_x = (sum_x / N) as f32 * ACCEL_SCALE;
        let mean_y = (sum_y / N) as f32 * ACCEL_SCALE;
        let mean_z = (sum_z / N) as f32 * ACCEL_SCALE;

        // Offsets : X et Y doivent être nuls, Z doit être ±9.80665 m/s²
        let offset_x = mean_x;
        let offset_y = mean_y;
        let offset_z = mean_z - 9.80665; // retirer la gravité

        // Scale : ratio entre gravité mesurée et gravité attendue sur Z
        let scale_z = 9.80665 / (mean_z - offset_z).abs().max(0.1);

        println!("  Accel offsets: x={:.4} y={:.4} z={:.4} m/s²",
                 offset_x, offset_y, offset_z);

        Ok(CalibrationAccel::nouvelle(
            offset_x, offset_y, offset_z,
            1.0, 1.0, scale_z,
        ))
    }

    // =========================================================================
    // Lecture des données brutes
    // =========================================================================

    /// Lit les données brutes gyro (16 bits signés, non compensées)
    fn lire_gyro_brut(&mut self) -> Result<(i16, i16, i16)> {
        let mut buf = [0u8; 6];
        self.lire_multi(reg::GYRO_XOUT_H, &mut buf)?;
        Ok((
            i16::from_be_bytes([buf[0], buf[1]]),
            i16::from_be_bytes([buf[2], buf[3]]),
            i16::from_be_bytes([buf[4], buf[5]]),
        ))
    }

    /// Lit les données brutes accel (16 bits signés, non compensées)
    fn lire_accel_brut(&mut self) -> Result<(i16, i16, i16)> {
        let mut buf = [0u8; 6];
        self.lire_multi(reg::ACCEL_XOUT_H, &mut buf)?;
        Ok((
            i16::from_be_bytes([buf[0], buf[1]]),
            i16::from_be_bytes([buf[2], buf[3]]),
            i16::from_be_bytes([buf[4], buf[5]]),
        ))
    }

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

    fn appliquer_calib_gyro(&self, gx: i16, gy: i16, gz: i16) -> Vector3 {
        let calib = self.calib_gyro.as_ref();
        let (ox, oy, oz) = calib
            .map(|c| (c.offset_x, c.offset_y, c.offset_z))
            .unwrap_or((0.0, 0.0, 0.0));

        Vector3::nouveau(
            gx as f32 * GYRO_SCALE - ox,
            gy as f32 * GYRO_SCALE - oy,
            gz as f32 * GYRO_SCALE - oz,
        )
    }

    fn appliquer_calib_accel(&self, ax: i16, ay: i16, az: i16) -> Vector3 {
        let calib = self.calib_accel.as_ref();
        let (ox, oy, oz, sx, sy, sz) = calib
            .map(|c| (c.offset_x, c.offset_y, c.offset_z,
                      c.scale_x,  c.scale_y,  c.scale_z))
            .unwrap_or((0.0, 0.0, 0.0, 1.0, 1.0, 1.0));

        Vector3::nouveau(
            (ax as f32 * ACCEL_SCALE - ox) * sx,
            (ay as f32 * ACCEL_SCALE - oy) * sy,
            (az as f32 * ACCEL_SCALE - oz) * sz,
        )
    }

    fn appliquer_calib_mag(&self, mx: i16, my: i16, mz: i16) -> Vector3 {
        let calib = self.calib_mag.as_ref();
        let (asa_x, asa_y, asa_z,
             hix, hiy, hiz,
             six, siy, siz) = calib
            .map(|c| (c.asa_x, c.asa_y, c.asa_z,
                      c.hard_iron_x, c.hard_iron_y, c.hard_iron_z,
                      c.soft_iron_x, c.soft_iron_y, c.soft_iron_z))
            .unwrap_or((1.0, 1.0, 1.0,
                        0.0, 0.0, 0.0,
                        1.0, 1.0, 1.0));

        Vector3::nouveau(
            (mx as f32 * MAG_SCALE * asa_x - hix) * six,
            (my as f32 * MAG_SCALE * asa_y - hiy) * siy,
            (mz as f32 * MAG_SCALE * asa_z - hiz) * siz,
        )
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

        // Reprise rapide si calibrations valides
        let gest = crate::systeme::calibration::gestionnaire();
        let gyro_ok  = gest.charger::<CalibrationGyro>()?.is_some();
        let accel_ok = gest.charger::<CalibrationAccel>()?.is_some();
        let mag_ok   = gest.charger::<CalibrationMag>()?.is_some();

        if gyro_ok && accel_ok && mag_ok {
            println!("⚡ MPU9250: Reprise rapide");
            // On réapplique quand même la config hardware (reset programme)
            self.reveiller()?;
            self.configurer_gyro()?;
            self.configurer_accel()?;
            self.activer_i2c_master()?;
            // Réinit AK8963 sans refaire les ASA (on a les calib)
            self.ak_ecrire(reg_ak::CNTL1, 0x00)?;
            std::thread::sleep(std::time::Duration::from_millis(10));
            self.ak_ecrire(reg_ak::CNTL1, 0x16)?;
            self.ak_lire_setup(reg_ak::ST1, 8)?;

            // Recharger les calibrations en mémoire
            self.calib_gyro  = gest.charger::<CalibrationGyro>()?;
            self.calib_accel = gest.charger::<CalibrationAccel>()?;
            self.calib_mag   = gest.charger::<CalibrationMag>()?;

            self.etat = EtatCapteur::nouveau_operationnel();
            return Ok(());
        }

        // Initialisation complète
        println!("🔧 MPU9250: Initialisation complète");
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

    fn calibrer_gyro(&mut self) -> Result<()> {
        let c = self.effectuer_calibration_gyro()?;
        crate::systeme::calibration::gestionnaire().sauvegarder(&c)?;
        self.calib_gyro = Some(c);
        Ok(())
    }

    fn calibrer_accel(&mut self) -> Result<()> {
        let c = self.effectuer_calibration_accel()?;
        crate::systeme::calibration::gestionnaire().sauvegarder(&c)?;
        self.calib_accel = Some(c);
        Ok(())
    }

    fn calibrer_mag(&mut self) -> Result<()> {
        let asa_x = self.calib_mag.map(|c| c.asa_x).unwrap_or(1.0);
        let asa_y = self.calib_mag.map(|c| c.asa_y).unwrap_or(1.0);
        let asa_z = self.calib_mag.map(|c| c.asa_z).unwrap_or(1.0);

        println!("Calibration magnétomètre :");
        println!("  Effectuez des rotations lentes sur les 3 axes (figure-8)");
        println!("  Collecte pendant 30 secondes...");

        const N: usize = 300; // ~1 échantillon toutes les 100ms = 30s
        let mut min_x = f32::MAX;  let mut max_x = f32::MIN;
        let mut min_y = f32::MAX;  let mut max_y = f32::MIN;
        let mut min_z = f32::MAX;  let mut max_z = f32::MIN;

        let mut echantillons = 0usize;

        for i in 0..N {
            std::thread::sleep(std::time::Duration::from_millis(100));
            if let Some((mx, my, mz)) = self.lire_mag_brut()? {
                let x = mx as f32 * MAG_SCALE * asa_x;
                let y = my as f32 * MAG_SCALE * asa_y;
                let z = mz as f32 * MAG_SCALE * asa_z;
                min_x = min_x.min(x); max_x = max_x.max(x);
                min_y = min_y.min(y); max_y = max_y.max(y);
                min_z = min_z.min(z); max_z = max_z.max(z);
                echantillons += 1;
            }
            if i % 50 == 49 {
                println!("  {}s / 30s...", (i + 1) / 10);
            }
        }

        if echantillons < 50 {
            return Err(ErreursAirHaum::CalibrationEchouee(
                format!("Trop peu d'échantillons mag: {}", echantillons)
            ));
        }

        // Hard iron = centre de l'ellipsoïde
        let hard_iron_x = (max_x + min_x) / 2.0;
        let hard_iron_y = (max_y + min_y) / 2.0;
        let hard_iron_z = (max_z + min_z) / 2.0;

        // Soft iron simplifié = normalisation par rapport à l'axe le plus long
        let range_x = (max_x - min_x) / 2.0;
        let range_y = (max_y - min_y) / 2.0;
        let range_z = (max_z - min_z) / 2.0;
        let range_max = range_x.max(range_y).max(range_z).max(0.1);
        let soft_iron_x = range_max / range_x.max(0.1);
        let soft_iron_y = range_max / range_y.max(0.1);
        let soft_iron_z = range_max / range_z.max(0.1);

        println!("  Hard iron: x={:.2} y={:.2} z={:.2} µT",
                 hard_iron_x, hard_iron_y, hard_iron_z);
        println!("  Soft iron: x={:.3} y={:.3} z={:.3}",
                 soft_iron_x, soft_iron_y, soft_iron_z);

        let c = CalibrationMag::nouvelle(
            asa_x, asa_y, asa_z,
            hard_iron_x, hard_iron_y, hard_iron_z,
            soft_iron_x, soft_iron_y, soft_iron_z,
        );
        crate::systeme::calibration::gestionnaire().sauvegarder(&c)?;
        self.calib_mag = Some(c);

        println!("✓ Calibration magnétomètre terminée");
        Ok(())
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
        i2c.precharger_registre(ADRESSE_MPU9250, reg::WHO_AM_I, 0x71);
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
