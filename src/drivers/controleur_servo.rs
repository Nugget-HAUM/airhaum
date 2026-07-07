// src/drivers/controleur_servo.rs
//! Driver de communication Pi ↔ Arduino Nano (interface servos/RC).
//!
//! Implémente le protocole binaire décrit dans `doc/protocole_uart_arduino.md` :
//! - trame consigne Pi → Arduino : 11 octets à 50 Hz
//! - trame remontée Arduino → Pi : 15 octets à 10 Hz
//!
//! La resynchronisation est gérée en interne : un octet de début erroné
//! ou une somme de contrôle invalide provoque l'avancement du tampon
//! jusqu'au prochain octet de début valide.

use std::io;
use crate::hal::uart::PortSerie;

// ─────────────────────────────────────────────────────────────────────────────
// Constantes de protocole
// ─────────────────────────────────────────────────────────────────────────────

const DEBUT_CONSIGNE: u8  = 0xAA;
const DEBUT_REMONTEE: u8  = 0xBB;
const TAILLE_CONSIGNE: usize = 11;
const TAILLE_REMONTEE: usize = 15;

pub const IMPULSION_MIN_US:    u16 = 1_000;
pub const IMPULSION_NEUTRE_US: u16 = 1_500;
pub const IMPULSION_MAX_US:    u16 = 2_000;

// ─────────────────────────────────────────────────────────────────────────────
// Types publics
// ─────────────────────────────────────────────────────────────────────────────

/// Consignes envoyées au Nano à 50 Hz.
#[derive(Debug, Clone)]
pub struct ConsignesServos {
    /// Ailerons en µs (1 000–2 000, neutre 1 500).
    pub ailerons:   u16,
    /// Gouverne de profondeur en µs.
    pub profondeur: u16,
    /// Gaz en µs. Ignoré par le Nano si `arme` est faux.
    pub gaz:        u16,
    /// Gouverne de direction en µs.
    pub direction:  u16,
    /// Vrai uniquement à partir de l'état `Armement` de la MAÉ vol.
    pub arme:       bool,
}

impl ConsignesServos {
    /// Position de sécurité : servos au neutre, gaz zéro, non armé.
    pub fn neutre() -> Self {
        Self {
            ailerons:   IMPULSION_NEUTRE_US,
            profondeur: IMPULSION_NEUTRE_US,
            gaz:        IMPULSION_MIN_US,
            direction:  IMPULSION_NEUTRE_US,
            arme:       false,
        }
    }
}

/// Mode d'arbitrage actif sur le Nano.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ModeArduino {
    Autopilote,
    Manuel,
}

/// État remonté par le Nano à 10 Hz.
#[derive(Debug, Clone)]
pub struct EtatArduino {
    pub mode:           ModeArduino,
    /// Vrai si le signal RC est absent depuis plus de 1 seconde.
    pub rc_perdu:       bool,
    /// Vrai si aucune trame Pi valide depuis 2 secondes.
    pub chien_de_garde: bool,
    /// Valeurs PWM RC des canaux 1 à 5 en µs (A1–A5 sur le Nano).
    pub canaux_rc:      [u16; 5],
    /// Consigne gaz réellement appliquée au variateur (µs).
    pub gaz_applique:   u16,
}

impl EtatArduino {
    /// État initial avant tout contact avec le Nano.
    pub fn absent() -> Self {
        Self {
            mode:           ModeArduino::Autopilote,
            rc_perdu:       true,
            chien_de_garde: false,
            canaux_rc:      [IMPULSION_NEUTRE_US; 5],
            gaz_applique:   IMPULSION_MIN_US,
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Driver
// ─────────────────────────────────────────────────────────────────────────────

/// Driver Pi ↔ Arduino Nano.
pub struct ControleurServo<P: PortSerie> {
    port:   P,
    tampon: Vec<u8>,
}

impl<P: PortSerie> ControleurServo<P> {
    pub fn nouveau(port: P) -> Self {
        Self {
            port,
            tampon: Vec::with_capacity(TAILLE_REMONTEE * 2),
        }
    }

    /// Envoie une trame consigne. À appeler à 50 Hz.
    pub fn envoyer(&mut self, c: &ConsignesServos) -> io::Result<()> {
        let fanions: u8 = u8::from(c.arme);
        let [a0, a1] = c.ailerons.to_le_bytes();
        let [p0, p1] = c.profondeur.to_le_bytes();
        let [g0, g1] = c.gaz.to_le_bytes();
        let [d0, d1] = c.direction.to_le_bytes();

        let charge: [u8; 9] = [fanions, a0, a1, p0, p1, g0, g1, d0, d1];
        let checksum = charge.iter().fold(0u8, |acc, &b| acc ^ b);

        let mut trame = [0u8; TAILLE_CONSIGNE];
        trame[0] = DEBUT_CONSIGNE;
        trame[1..10].copy_from_slice(&charge);
        trame[10] = checksum;

        self.port.ecrire(&trame)?;
        Ok(())
    }

    /// Tente de lire et parser une trame de remontée. Non-bloquant.
    ///
    /// Retourne `Some(EtatArduino)` si une trame complète et valide a été reçue,
    /// `None` si les données sont insuffisantes ou absentes.
    pub fn recevoir(&mut self) -> io::Result<Option<EtatArduino>> {
        let mut buf = [0u8; 64];
        match self.port.lire(&mut buf) {
            Ok(n) if n > 0 => self.tampon.extend_from_slice(&buf[..n]),
            Err(e) if e.kind() == io::ErrorKind::TimedOut => {}
            Err(e) => return Err(e),
            _ => {}
        }

        loop {
            // Chercher l'octet de début
            let Some(pos) = self.tampon.iter().position(|&b| b == DEBUT_REMONTEE) else {
                self.tampon.clear();
                return Ok(None);
            };

            // Pas assez d'octets : attendre la prochaine lecture
            if self.tampon.len() - pos < TAILLE_REMONTEE {
                if pos > 0 { self.tampon.drain(..pos); }
                return Ok(None);
            }

            let trame = &self.tampon[pos..pos + TAILLE_REMONTEE];
            let checksum_calcule = trame[1..14].iter().fold(0u8, |acc, &b| acc ^ b);

            if checksum_calcule == trame[14] {
                let etat = Self::parser_remontee(trame);
                self.tampon.drain(..pos + TAILLE_REMONTEE);
                return Ok(Some(etat));
            }

            // Somme de contrôle invalide : avancer d'un octet et réessayer
            self.tampon.drain(..pos + 1);
        }
    }

    fn parser_remontee(trame: &[u8]) -> EtatArduino {
        let fanions = trame[1];
        EtatArduino {
            mode:           if fanions & 0x01 != 0 { ModeArduino::Manuel } else { ModeArduino::Autopilote },
            rc_perdu:       fanions & 0x02 != 0,
            chien_de_garde: fanions & 0x04 != 0,
            canaux_rc: [
                u16::from_le_bytes([trame[2],  trame[3]]),
                u16::from_le_bytes([trame[4],  trame[5]]),
                u16::from_le_bytes([trame[6],  trame[7]]),
                u16::from_le_bytes([trame[8],  trame[9]]),
                u16::from_le_bytes([trame[10], trame[11]]),
            ],
            gaz_applique: u16::from_le_bytes([trame[12], trame[13]]),
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Tests
// ─────────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::hal::uart::PortSerieMock;

    fn driver_mock() -> ControleurServo<PortSerieMock> {
        ControleurServo::nouveau(PortSerieMock::nouveau())
    }

    fn trame_remontee(fanions: u8, canaux: [u16; 5], gaz: u16) -> Vec<u8> {
        let mut t = vec![0u8; TAILLE_REMONTEE];
        t[0] = DEBUT_REMONTEE;
        t[1] = fanions;
        for (i, &c) in canaux.iter().enumerate() {
            let [lo, hi] = c.to_le_bytes();
            t[2 + i * 2] = lo;
            t[3 + i * 2] = hi;
        }
        let [g0, g1] = gaz.to_le_bytes();
        t[12] = g0;
        t[13] = g1;
        t[14] = t[1..14].iter().fold(0u8, |acc, &b| acc ^ b);
        t
    }

    #[test]
    fn encodage_trame_consigne() {
        let mut driver = driver_mock();
        let c = ConsignesServos { ailerons: 1500, profondeur: 1500, gaz: 1000, direction: 1500, arme: false };
        driver.envoyer(&c).unwrap();

        let octets = driver.port.octets_ecrits();
        assert_eq!(octets.len(), TAILLE_CONSIGNE);
        assert_eq!(octets[0], DEBUT_CONSIGNE);

        // Vérifier la somme de contrôle
        let checksum_attendu = octets[1..10].iter().fold(0u8, |acc, &b| acc ^ b);
        assert_eq!(octets[10], checksum_attendu);
    }

    #[test]
    fn bit_arme_correct() {
        let mut driver = driver_mock();
        let mut c = ConsignesServos::neutre();

        c.arme = false;
        driver.envoyer(&c).unwrap();
        assert_eq!(driver.port.octets_ecrits()[1] & 0x01, 0);

        c.arme = true;
        driver.envoyer(&c).unwrap();
        let octets = driver.port.octets_ecrits();
        assert_eq!(octets[TAILLE_CONSIGNE + 1] & 0x01, 1);
    }

    #[test]
    fn valeurs_canaux_correctes() {
        let mut driver = driver_mock();
        let c = ConsignesServos { ailerons: 1234, profondeur: 1600, gaz: 1800, direction: 1100, arme: true };
        driver.envoyer(&c).unwrap();

        let o = driver.port.octets_ecrits();
        assert_eq!(u16::from_le_bytes([o[2], o[3]]), 1234);
        assert_eq!(u16::from_le_bytes([o[4], o[5]]), 1600);
        assert_eq!(u16::from_le_bytes([o[6], o[7]]), 1800);
        assert_eq!(u16::from_le_bytes([o[8], o[9]]), 1100);
    }

    #[test]
    fn parse_trame_remontee_nominale() {
        let mut driver = driver_mock();
        let canaux = [1500u16, 1500, 1000, 1500, 1800];
        driver.port.injecter(&trame_remontee(0x00, canaux, 1000));

        let etat = driver.recevoir().unwrap().expect("trame attendue");
        assert_eq!(etat.mode, ModeArduino::Autopilote);
        assert!(!etat.rc_perdu);
        assert!(!etat.chien_de_garde);
        assert_eq!(etat.canaux_rc[0], 1500);
        assert_eq!(etat.canaux_rc[4], 1800);
        assert_eq!(etat.gaz_applique, 1000);
    }

    #[test]
    fn parse_flags_remontee() {
        let mut driver = driver_mock();
        // bit 0 = manuel, bit 1 = rc perdu, bit 2 = chien de garde
        driver.port.injecter(&trame_remontee(0x07, [1500; 5], 1000));

        let etat = driver.recevoir().unwrap().expect("trame attendue");
        assert_eq!(etat.mode, ModeArduino::Manuel);
        assert!(etat.rc_perdu);
        assert!(etat.chien_de_garde);
    }

    #[test]
    fn resynchronisation_sur_checksum_invalide() {
        let mut driver = driver_mock();
        // Trame corrompue suivie d'une trame valide
        let mut corrompue = trame_remontee(0x00, [1500; 5], 1000);
        corrompue[14] ^= 0xFF; // invalider la somme de contrôle
        let valide = trame_remontee(0x01, [1200, 1300, 1000, 1400, 1800], 1000);

        driver.port.injecter(&corrompue);
        driver.port.injecter(&valide);

        // La trame corrompue doit être ignorée
        let etat = driver.recevoir().unwrap().expect("trame valide attendue");
        assert_eq!(etat.mode, ModeArduino::Manuel);
        assert_eq!(etat.canaux_rc[0], 1200);
    }

    #[test]
    fn pas_de_trame_si_donnees_insuffisantes() {
        let mut driver = driver_mock();
        driver.port.injecter(&[DEBUT_REMONTEE, 0x00, 0x01]); // tronquée
        assert!(driver.recevoir().unwrap().is_none());
    }
}
