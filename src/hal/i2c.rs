// src/hal/i2c.rs
//! Abstraction générique du bus I²C
//!
//! Ce module expose :
//! - [`BusI2c`] : trait central découplant le code métier du hardware
//! - [`I2cMock`] : implémentation en mémoire pour les tests unitaires et d'intégration
//!
//! # Modèle de partage du bus
//!
//! Sur un drone plusieurs capteurs cohabitent sur le même bus physique I²C.
//! Pour garantir que les transactions ne s'entrelacent pas — même sous Tokio —
//! l'accès concurrent se fait via un bus partagé :
//!
//! ```ignore
//! // Dans taches_capteurs.rs
//! let bus: BusPartage = BusPartage::new(Mutex::new(I2cLinux::nouveau(0)?));
//! let bus_bmp = Arc::clone(&bus);
//! let bus_vl53 = Arc::clone(&bus);
//! ```
//!
//! Voir [`crate::hal::BusPartage`] et `taches/taches_capteurs.rs`.
//!
//! # Horodatage
//!
//! [`BusI2c::lire_avec_horodatage`] combine lecture et horodatage en une seule
//! opération atomique, minimisant la dérive entre la fin de la transaction I²C
//! et la capture du timestamp. L'horodatage doit toujours être pris **après**
//! la lecture, une fois que les données sont dans le buffer.

use crate::types::{Result, ErreursAirHaum, Horodatage};

/// Trait pour l'abstraction d'un bus I²C.
///
/// Toutes les opérations prennent `&mut self` car le bus est un état mutable
/// (adresse esclave courante, buffers internes du driver). Le partage concurrent
/// est géré en amont via [`crate::hal::BusPartage`].
///
/// # Implémentations
/// - [`crate::hal::i2c_linux::I2cLinux`] : production (Linux / Raspberry Pi)
/// - [`I2cMock`] : tests unitaires et d'intégration hors cible
pub trait BusI2c: Send {
    // ─── Primitives obligatoires ───────────────────────────────────────────

    /// Écrit des données vers un périphérique I²C.
    ///
    /// # Arguments
    /// * `adresse` - Adresse 7-bit du périphérique esclave
    /// * `donnees` - Octets à écrire (généralement `[registre, valeur, ...]`)
    fn ecrire(&mut self, adresse: u8, donnees: &[u8]) -> Result<()>;

    /// Lit des données depuis un périphérique I²C.
    ///
    /// # Arguments
    /// * `adresse` - Adresse 7-bit du périphérique esclave
    /// * `buffer`  - Buffer destination ; sa longueur détermine le nombre d'octets lus
    fn lire(&mut self, adresse: u8, buffer: &mut [u8]) -> Result<()>;

    /// Effectue une transaction write-then-read sans relâcher le bus (repeated START).
    ///
    /// Opération native sur le bus I²C (ioctl `I2C_RDWR` sur Linux).
    /// C'est la manière correcte de lire un registre sur la quasi-totalité
    /// des capteurs I²C.
    ///
    /// # Arguments
    /// * `adresse`  - Adresse 7-bit du périphérique esclave
    /// * `registre` - Un ou plusieurs octets identifiant le registre à lire
    /// * `buffer`   - Buffer destination
    fn ecrire_lire(&mut self, adresse: u8, registre: &[u8], buffer: &mut [u8]) -> Result<()>;

    // ─── Helpers avec implémentation par défaut ────────────────────────────

    /// Écrit une valeur dans un registre 8-bit.
    ///
    /// Raccourci pour `ecrire(adresse, &[registre, valeur])`.
    fn ecrire_registre_u8(&mut self, adresse: u8, registre: u8, valeur: u8) -> Result<()> {
        self.ecrire(adresse, &[registre, valeur])
    }

    /// Lit un registre 8-bit.
    fn lire_registre_u8(&mut self, adresse: u8, registre: u8) -> Result<u8> {
        let mut buffer = [0u8; 1];
        self.ecrire_lire(adresse, &[registre], &mut buffer)?;
        Ok(buffer[0])
    }

    /// Lit un registre 16-bit (big-endian).
    fn lire_registre_u16_be(&mut self, adresse: u8, registre: u8) -> Result<u16> {
        let mut buffer = [0u8; 2];
        self.ecrire_lire(adresse, &[registre], &mut buffer)?;
        Ok(u16::from_be_bytes(buffer))
    }

    /// Lit un registre 16-bit (little-endian).
    fn lire_registre_u16_le(&mut self, adresse: u8, registre: u8) -> Result<u16> {
        let mut buffer = [0u8; 2];
        self.ecrire_lire(adresse, &[registre], &mut buffer)?;
        Ok(u16::from_le_bytes(buffer))
    }

    /// Lit des données et les horodate immédiatement après la fin de la transaction.
    ///
    /// L'horodatage est capturé **après** que les données sont dans `buffer`,
    /// minimisant la dérive par rapport à l'instant réel de la mesure.
    /// C'est la primitive recommandée pour toute acquisition capteur.
    ///
    /// # Retour
    /// `Ok(Horodatage)` — les données sont dans `buffer`, le timestamp dans la valeur
    ///
    /// # Exemple
    /// ```ignore
    /// let mut buf = [0u8; 6];
    /// let ts = bus.lire_avec_horodatage(ADRESSE, &[REG_DATA], &mut buf)?;
    /// let donnees = DonneesImu::depuis_brut(&buf, ts);
    /// ```
    fn lire_avec_horodatage(
        &mut self,
        adresse: u8,
        registre: &[u8],
        buffer: &mut [u8],
    ) -> Result<Horodatage> {
        self.ecrire_lire(adresse, registre, buffer)?;
        // Horodatage pris ici : après la transaction I²C, avant tout traitement.
        Ok(Horodatage::maintenant())
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Impl pour &mut T — permet de passer un &mut BusI2c là où un BusI2c est attendu
// ─────────────────────────────────────────────────────────────────────────────

impl<T: BusI2c> BusI2c for &mut T {
    fn ecrire(&mut self, adresse: u8, donnees: &[u8]) -> Result<()> {
        (*self).ecrire(adresse, donnees)
    }
    fn lire(&mut self, adresse: u8, buffer: &mut [u8]) -> Result<()> {
        (*self).lire(adresse, buffer)
    }
    fn ecrire_lire(&mut self, adresse: u8, registre: &[u8], buffer: &mut [u8]) -> Result<()> {
        (*self).ecrire_lire(adresse, registre, buffer)
    }
}


// ─────────────────────────────────────────────────────────────────────────────
// Impl pour Arc<std::sync::Mutex<B>> — permet aux drivers de tenir un bus
// partagé et de l'utiliser depuis des std::thread dédiés.
//
// Le verrou est acquis de manière bloquante (.lock()), ce qui est correct ici
// car les drivers sont appelés depuis des std::thread (pas de runtime Tokio
// dans ce chemin). La durée de verrouillage est bornée par le timeout kernel
// I2C_TIMEOUT (~10 ms), garantissant que le mutex est toujours libéré.
//
// Si un thread a paniqué en tenant le verrou, le mutex est corrompu : on
// propage une erreur I²C plutôt que de paniquer à notre tour.
// ─────────────────────────────────────────────────────────────────────────────

impl<B: BusI2c + Send> BusI2c for std::sync::Arc<std::sync::Mutex<B>> {
    fn ecrire(&mut self, adresse: u8, donnees: &[u8]) -> Result<()> {
        self.lock()
            .map_err(|_| ErreursAirHaum::ErreurI2C("verrou I²C corrompu".into()))?
            .ecrire(adresse, donnees)
    }

    fn lire(&mut self, adresse: u8, buffer: &mut [u8]) -> Result<()> {
        self.lock()
            .map_err(|_| ErreursAirHaum::ErreurI2C("verrou I²C corrompu".into()))?
            .lire(adresse, buffer)
    }

    fn ecrire_lire(&mut self, adresse: u8, registre: &[u8], buffer: &mut [u8]) -> Result<()> {
        self.lock()
            .map_err(|_| ErreursAirHaum::ErreurI2C("verrou I²C corrompu".into()))?
            .ecrire_lire(adresse, registre, buffer)
    }
}





// ─────────────────────────────────────────────────────────────────────────────
// I2cMock
// ─────────────────────────────────────────────────────────────────────────────

/// Implémentation en mémoire de [`BusI2c`] pour les tests.
///
/// Simule un bus I²C via une table de registres `(adresse_device, adresse_registre) → u8`.
///
/// # Utilisation dans les tests unitaires
/// ```
/// use airhaum::hal::i2c::{I2cMock, BusI2c};
///
/// let mut i2c = I2cMock::nouveau();
/// i2c.precharger_registre(0x76, 0xD0, 0x58); // Simule l'ID du BMP280
/// let id = i2c.lire_registre_u8(0x76, 0xD0).unwrap();
/// assert_eq!(id, 0x58);
/// ```
///
/// # Utilisation dans les tests d'intégration (hors `#[test]`)
/// `I2cMock` est disponible sans flag `cfg(test)` pour permettre son usage
/// dans les binaires de test d'intégration. Ne jamais l'instancier en production.
pub struct I2cMock {
    /// Table des registres simulés : `(adresse_device, adresse_registre) → valeur`
    registres: std::collections::HashMap<(u8, u8), u8>,
    /// Si `true`, toutes les opérations retournent une erreur I²C.
    /// Utile pour tester les chemins de récupération d'erreur des drivers.
    pub simuler_erreur: bool,
}

impl I2cMock {
    /// Crée un mock vide avec tous les registres à 0 et sans simulation d'erreur.
    pub fn nouveau() -> Self {
        Self {
            registres: std::collections::HashMap::new(),
            simuler_erreur: false,
        }
    }

    /// Précharge un registre avec une valeur pour simuler la réponse d'un capteur.
    pub fn precharger_registre(&mut self, adresse: u8, registre: u8, valeur: u8) {
        self.registres.insert((adresse, registre), valeur);
    }

    /// Lit la valeur actuelle d'un registre simulé (pour assertions dans les tests).
    ///
    /// Retourne `None` si le registre n'a pas été préchargé ni écrit.
    pub fn verifier_registre(&self, adresse: u8, registre: u8) -> Option<u8> {
        self.registres.get(&(adresse, registre)).copied()
    }
}

impl BusI2c for I2cMock {
    fn ecrire(&mut self, adresse: u8, donnees: &[u8]) -> Result<()> {
        if self.simuler_erreur {
            return Err(ErreursAirHaum::ErreurI2C("Erreur simulée".into()));
        }
        if donnees.len() >= 2 {
            // Format attendu : [adresse_registre, valeur_0, valeur_1, ...]
            let registre = donnees[0];
            for (i, &valeur) in donnees[1..].iter().enumerate() {
                self.registres.insert((adresse, registre + i as u8), valeur);
            }
        }
        // Écriture d'un seul octet (ex: commande sans registre) : ignorée silencieusement.
        Ok(())
    }

    fn lire(&mut self, adresse: u8, buffer: &mut [u8]) -> Result<()> {
        if self.simuler_erreur {
            return Err(ErreursAirHaum::ErreurI2C("Erreur simulée".into()));
        }
        // Lecture brute sans sélection de registre : on lit à partir du registre 0x00
        // et on incrémente l'adresse pour chaque octet, fidèlement au comportement I²C.
        for (i, byte) in buffer.iter_mut().enumerate() {
            *byte = self.registres
                .get(&(adresse, i as u8))
                .copied()
                .unwrap_or(0);
        }
        Ok(())
    }

    fn ecrire_lire(&mut self, adresse: u8, registre: &[u8], buffer: &mut [u8]) -> Result<()> {
        if self.simuler_erreur {
            return Err(ErreursAirHaum::ErreurI2C("Erreur simulée".into()));
        }
        if registre.is_empty() {
            return Err(ErreursAirHaum::ErreurI2C(
                "ecrire_lire : le slice 'registre' ne peut pas être vide".into(),
            ));
        }

        let reg_addr = registre[0];
        for (i, byte) in buffer.iter_mut().enumerate() {
            let key = (adresse, reg_addr + i as u8);
            *byte = self.registres.get(&key).copied().ok_or_else(|| {
                ErreursAirHaum::ErreurI2C(format!(
                    "Mock : registre 0x{:02X}:0x{:02X} non préchargé",
                    adresse,
                    reg_addr + i as u8
                ))
            })?;
        }
        Ok(())
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Tests
// ─────────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_mock_ecriture_lecture_u8() {
        let mut i2c = I2cMock::nouveau();
        i2c.ecrire_registre_u8(0x76, 0xF4, 0x27).unwrap();
        assert_eq!(i2c.verifier_registre(0x76, 0xF4), Some(0x27));
    }

    #[test]
    fn test_mock_lecture_registre_precharge() {
        let mut i2c = I2cMock::nouveau();
        i2c.precharger_registre(0x76, 0xD0, 0x58);
        let id = i2c.lire_registre_u8(0x76, 0xD0).unwrap();
        assert_eq!(id, 0x58);
    }

    #[test]
    fn test_mock_lecture_u16_le() {
        let mut i2c = I2cMock::nouveau();
        i2c.precharger_registre(0x76, 0x88, 0x12);
        i2c.precharger_registre(0x76, 0x89, 0x34);
        let valeur = i2c.lire_registre_u16_le(0x76, 0x88).unwrap();
        assert_eq!(valeur, 0x3412);
    }

    #[test]
    fn test_mock_lecture_u16_be() {
        let mut i2c = I2cMock::nouveau();
        i2c.precharger_registre(0x76, 0x88, 0x12);
        i2c.precharger_registre(0x76, 0x89, 0x34);
        let valeur = i2c.lire_registre_u16_be(0x76, 0x88).unwrap();
        assert_eq!(valeur, 0x1234);
    }

    #[test]
    fn test_mock_simuler_erreur() {
        let mut i2c = I2cMock::nouveau();
        i2c.simuler_erreur = true;
        assert!(i2c.lire_registre_u8(0x76, 0xD0).is_err());
        assert!(i2c.ecrire_registre_u8(0x76, 0xF4, 0x27).is_err());
    }

    #[test]
    fn test_mock_ecrire_registre_vide() {
        let mut i2c = I2cMock::nouveau();
        // ecrire avec un seul octet (pas de valeur) : silencieux
        assert!(i2c.ecrire(0x76, &[0xF4]).is_ok());
    }

    #[test]
    fn test_mock_ecrire_lire_registre_manquant() {
        let mut i2c = I2cMock::nouveau();
        // Registre non préchargé : doit retourner une erreur explicite
        let mut buf = [0u8; 1];
        let err = i2c.ecrire_lire(0x76, &[0xFF], &mut buf);
        assert!(err.is_err());
        let msg = format!("{:?}", err.unwrap_err());
        assert!(msg.contains("non préchargé"));
    }

    #[test]
    fn test_mock_lire_brut_sequentiel() {
        let mut i2c = I2cMock::nouveau();
        // lire() sans registre lit à partir de l'offset 0x00
        i2c.precharger_registre(0x68, 0x00, 0xAA);
        i2c.precharger_registre(0x68, 0x01, 0xBB);
        let mut buf = [0u8; 2];
        i2c.lire(0x68, &mut buf).unwrap();
        assert_eq!(buf, [0xAA, 0xBB]);
    }

    #[test]
    fn test_lire_avec_horodatage() {
        let mut i2c = I2cMock::nouveau();
        i2c.precharger_registre(0x76, 0xF7, 0x51);
        i2c.precharger_registre(0x76, 0xF8, 0x80);
        i2c.precharger_registre(0x76, 0xF9, 0x00);

        let mut buf = [0u8; 3];
        let ts = i2c.lire_avec_horodatage(0x76, &[0xF7], &mut buf).unwrap();

        assert_eq!(buf, [0x51, 0x80, 0x00]);
        // Le timestamp doit être non-nul (temps écoulé depuis le démarrage du mock)
        // et inférieur à 1 seconde (test rapide).
        assert!(ts.micros() < 1_000_000);
    }
}
