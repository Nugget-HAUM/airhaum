// src/drivers/lora/sx1276.rs
// Driver pour le transceiver LoRa SX1276 (Semtech), en mode LoRa (pas FSK/OOK)
//
// Carte des registres et valeurs de configuration (fréquence, MODEM_CONFIG_1/2/3,
// PA_CONFIG, LNA, préambule) alignées sur `OrangePi/SX1276` (hors dépôt), un
// prototype minimal déjà testé avec succès en émission sur le matériel réel.
//
// Validé sur matériel réel le 2026-07-18 : initialiser() + envoyer() via
// diagnostiques::diag_lora, réception côté Wemos/PC (RSSI -36 dBm, SNR 9 dB).
// recevoir() reste non testé — voir doc/lora.md "Validation matérielle".

#![allow(dead_code)]

use crate::hal::spi::BusSpi;
use crate::types::{Result, ErreursAirHaum};
use std::time::{Duration, Instant};

/// Oscillateur de référence du SX1276 (presque toujours 32 MHz sur les modules du commerce).
const OSCILLATEUR_HZ: u64 = 32_000_000;

/// Identifiant de silicium attendu au registre VERSION (famille SX1276/77/78/79).
const VERSION_ATTENDUE: u8 = 0x12;

/// Taille du FIFO allouée à chacun des sens TX/RX (FIFO 256 octets partagé en deux moitiés).
///
/// Borne aussi la taille maximale d'une trame applicative (cf. `communications::protocole`),
/// qui est ce qui est effectivement passé à [`Sx1276::envoyer`].
pub const TAILLE_MAX_PAYLOAD: usize = 128;

/// Délai d'attente de repos du polling (IRQ, jamais câblé en interruption GPIO — voir doc/lora.md).
const PAS_POLLING_MS: u64 = 1;

/// Temps de vol maximal attendu pour un paquet — au-delà, on considère l'émission échouée.
const TIMEOUT_TX_MS: u64 = 2000;

mod registres {
    pub const FIFO: u8 = 0x00;
    pub const OP_MODE: u8 = 0x01;
    pub const FRF_MSB: u8 = 0x06;
    pub const FRF_MID: u8 = 0x07;
    pub const FRF_LSB: u8 = 0x08;
    pub const PA_CONFIG: u8 = 0x09;
    pub const LNA: u8 = 0x0C;
    pub const FIFO_ADDR_PTR: u8 = 0x0D;
    pub const FIFO_TX_BASE_ADDR: u8 = 0x0E;
    pub const FIFO_RX_BASE_ADDR: u8 = 0x0F;
    pub const FIFO_RX_CURRENT_ADDR: u8 = 0x10;
    pub const IRQ_FLAGS: u8 = 0x12;
    pub const RX_NB_BYTES: u8 = 0x13;
    pub const MODEM_CONFIG_1: u8 = 0x1D;
    pub const MODEM_CONFIG_2: u8 = 0x1E;
    pub const PREAMBLE_MSB: u8 = 0x20;
    pub const PREAMBLE_LSB: u8 = 0x21;
    pub const PAYLOAD_LENGTH: u8 = 0x22;
    pub const MODEM_CONFIG_3: u8 = 0x26;
    pub const VERSION: u8 = 0x42;
}

/// Valeurs du registre OP_MODE (0x01).
mod op_mode {
    pub const LONG_RANGE_MODE: u8 = 0x80; // bit7 : 1 = mode LoRa (vs FSK/OOK)
    pub const MODE_SLEEP: u8 = 0x00;
    pub const MODE_STDBY: u8 = 0x01;
    pub const MODE_TX: u8 = 0x03;
    pub const MODE_RXCONTINUOUS: u8 = 0x05;
}

/// Bits du registre IRQ_FLAGS (0x12) — écrire 1 sur un bit l'efface (RAZ).
mod irq {
    pub const RX_TIMEOUT: u8 = 0x80;
    pub const RX_DONE: u8 = 0x40;
    pub const PAYLOAD_CRC_ERROR: u8 = 0x20;
    pub const TX_DONE: u8 = 0x08;
    pub const TOUT_EFFACER: u8 = 0xFF;
}

/// Champs des registres MODEM_CONFIG_1/2/3.
mod modem {
    pub const BW_125K: u8 = 0x70;       // ModemConfig1[7:4] = 0111
    pub const CR_4_5: u8 = 0x02;        // ModemConfig1[3:1] = 001
    pub const HEADER_EXPLICITE: u8 = 0x00; // ModemConfig1[0] = 0 (en-tête explicite)
    pub const CRC_ON: u8 = 0x04;        // ModemConfig2[2]
    pub const AGC_AUTO_ON: u8 = 0x04;   // ModemConfig3[2]
}

/// Facteur d'étalement par défaut (SF7) : le plus faible airtime disponible,
/// privilégié pour la latence de la commande d'urgence (cf. doc/lora.md).
/// Valeur à revoir une fois la portée réelle mesurée sur le terrain.
const FACTEUR_ETALEMENT_DEFAUT: u8 = 7;

/// PA_CONFIG : sortie sur pin PA_BOOST, puissance maximale (~17 dBm).
/// Valeur reprise du prototype `OrangePi/SX1276` — à revisiter au regard de la
/// réglementation applicable (duty cycle / PIRE max, cf. doc/lora.md) une fois
/// le budget de liaison mesuré sur le terrain.
const PA_CONFIG_DEFAUT: u8 = 0x8F; // PaSelect=1, MaxPower=0, OutputPower=15

/// LNA : gain maximal + boost HF. Valeur reprise du prototype `OrangePi/SX1276`.
const LNA_GAIN_MAX_BOOST: u8 = 0x23;

/// Préambule par défaut du SX1276 (8 symboles) — écrit explicitement plutôt que
/// de dépendre de la valeur de reset matériel, le driver ne pilotant pas la
/// broche RESET (cf. doc/lora.md, pas de logique GPIO câblée).
const PREAMBLE_SYMBOLES_DEFAUT: u16 = 8;

/// Driver du transceiver SX1276 en mode LoRa.
///
/// État de repos = réception continue (cf. doc/lora.md, "Accès au canal") :
/// après [`initialiser`](Self::initialiser), le module écoute en permanence ;
/// [`envoyer`](Self::envoyer) interrompt brièvement cette écoute puis y revient.
pub struct Sx1276<S: BusSpi> {
    spi: S,
    pret: bool,
}

impl<S: BusSpi> Sx1276<S> {
    pub fn nouveau(spi: S) -> Self {
        Self { spi, pret: false }
    }

    // ─── Accès registres bas niveau ────────────────────────────────────────

    fn lire_registre(&mut self, reg: u8) -> Result<u8> {
        let mut lire = [0u8; 2];
        self.spi.transferer(&[reg & 0x7F, 0x00], &mut lire)?;
        Ok(lire[1])
    }

    fn ecrire_registre(&mut self, reg: u8, valeur: u8) -> Result<()> {
        let mut lire = [0u8; 2];
        self.spi.transferer(&[reg | 0x80, valeur], &mut lire)?;
        Ok(())
    }

    fn lire_rafale(&mut self, reg: u8, buffer: &mut [u8]) -> Result<()> {
        if buffer.len() > TAILLE_MAX_PAYLOAD {
            return Err(ErreursAirHaum::DonneesInvalides(
                "SX1276 : lecture rafale au-delà de la taille FIFO allouée".into(),
            ));
        }
        let mut ecrire = [0u8; TAILLE_MAX_PAYLOAD + 1];
        let mut lire = [0u8; TAILLE_MAX_PAYLOAD + 1];
        let n = buffer.len() + 1;
        ecrire[0] = reg & 0x7F;
        self.spi.transferer(&ecrire[..n], &mut lire[..n])?;
        buffer.copy_from_slice(&lire[1..n]);
        Ok(())
    }

    fn ecrire_rafale(&mut self, reg: u8, donnees: &[u8]) -> Result<()> {
        if donnees.len() > TAILLE_MAX_PAYLOAD {
            return Err(ErreursAirHaum::DonneesInvalides(
                "SX1276 : écriture rafale au-delà de la taille FIFO allouée".into(),
            ));
        }
        let mut ecrire = [0u8; TAILLE_MAX_PAYLOAD + 1];
        let mut lire = [0u8; TAILLE_MAX_PAYLOAD + 1];
        let n = donnees.len() + 1;
        ecrire[0] = reg | 0x80;
        ecrire[1..n].copy_from_slice(donnees);
        self.spi.transferer(&ecrire[..n], &mut lire[..n])?;
        Ok(())
    }

    // ─── Initialisation ─────────────────────────────────────────────────────

    fn verifier_version(&mut self) -> Result<()> {
        let version = self.lire_registre(registres::VERSION)?;
        if version != VERSION_ATTENDUE {
            return Err(ErreursAirHaum::DonneesInvalides(format!(
                "SX1276 version invalide : 0x{:02X} (attendu 0x{:02X})",
                version, VERSION_ATTENDUE
            )));
        }
        Ok(())
    }

    /// Programme la fréquence porteuse (ex. 868 000 000 pour 868 MHz).
    fn regler_frequence_hz(&mut self, frequence_hz: u32) -> Result<()> {
        let frf = ((frequence_hz as u64) << 19) / OSCILLATEUR_HZ;
        self.ecrire_registre(registres::FRF_MSB, (frf >> 16) as u8)?;
        self.ecrire_registre(registres::FRF_MID, (frf >> 8) as u8)?;
        self.ecrire_registre(registres::FRF_LSB, frf as u8)?;
        Ok(())
    }

    fn configurer_modem(&mut self) -> Result<()> {
        let config1 = modem::BW_125K | modem::CR_4_5 | modem::HEADER_EXPLICITE;
        self.ecrire_registre(registres::MODEM_CONFIG_1, config1)?;

        let config2 = (FACTEUR_ETALEMENT_DEFAUT << 4) | modem::CRC_ON;
        self.ecrire_registre(registres::MODEM_CONFIG_2, config2)?;

        self.ecrire_registre(registres::MODEM_CONFIG_3, modem::AGC_AUTO_ON)?;

        self.ecrire_registre(registres::PREAMBLE_MSB, (PREAMBLE_SYMBOLES_DEFAUT >> 8) as u8)?;
        self.ecrire_registre(registres::PREAMBLE_LSB, PREAMBLE_SYMBOLES_DEFAUT as u8)?;

        self.ecrire_registre(registres::LNA, LNA_GAIN_MAX_BOOST)?;

        Ok(())
    }

    /// Initialise le module et le laisse en réception continue (état de repos).
    ///
    /// `frequence_hz` : fréquence porteuse, ex. `868_000_000` (bande EU868 — à
    /// confirmer contre la réglementation applicable avant tout essai).
    pub fn initialiser(&mut self, frequence_hz: u32) -> Result<()> {
        self.verifier_version()?;

        // LongRangeMode ne peut être changé qu'en sommeil (datasheet §6.3).
        self.ecrire_registre(registres::OP_MODE, op_mode::LONG_RANGE_MODE | op_mode::MODE_SLEEP)?;
        self.ecrire_registre(registres::OP_MODE, op_mode::LONG_RANGE_MODE | op_mode::MODE_STDBY)?;

        // FIFO 256 octets scindée en deux moitiés TX/RX.
        self.ecrire_registre(registres::FIFO_TX_BASE_ADDR, 0x80)?;
        self.ecrire_registre(registres::FIFO_RX_BASE_ADDR, 0x00)?;

        self.regler_frequence_hz(frequence_hz)?;
        self.configurer_modem()?;
        self.ecrire_registre(registres::PA_CONFIG, PA_CONFIG_DEFAUT)?;

        self.ecrire_registre(registres::IRQ_FLAGS, irq::TOUT_EFFACER)?;
        self.mettre_en_reception()?;

        self.pret = true;
        Ok(())
    }

    fn verifier_pret(&self) -> Result<()> {
        if !self.pret {
            return Err(ErreursAirHaum::CapteurNonInitialise("SX1276 non initialisé".into()));
        }
        Ok(())
    }

    // ─── Réception (état de repos) ─────────────────────────────────────────

    /// Repasse en écoute continue. Appelé automatiquement après [`envoyer`](Self::envoyer)
    /// et à la fin de [`initialiser`](Self::initialiser).
    fn mettre_en_reception(&mut self) -> Result<()> {
        self.ecrire_registre(registres::FIFO_ADDR_PTR, 0x00)?;
        self.ecrire_registre(registres::OP_MODE, op_mode::LONG_RANGE_MODE | op_mode::MODE_RXCONTINUOUS)?;
        Ok(())
    }

    /// Sondage non bloquant : renvoie `Some(n)` si un paquet vient d'être reçu
    /// (copié dans `buffer[..n]`), `None` sinon. À appeler en boucle depuis un
    /// thread dédié, comme les autres drivers de la couche capteurs/GPS.
    pub fn recevoir(&mut self, buffer: &mut [u8]) -> Result<Option<usize>> {
        self.verifier_pret()?;

        let irq_flags = self.lire_registre(registres::IRQ_FLAGS)?;
        if irq_flags & irq::RX_DONE == 0 {
            return Ok(None);
        }

        if irq_flags & irq::PAYLOAD_CRC_ERROR != 0 {
            self.ecrire_registre(registres::IRQ_FLAGS, irq::TOUT_EFFACER)?;
            return Err(ErreursAirHaum::MessageCorrompu);
        }

        let taille = self.lire_registre(registres::RX_NB_BYTES)? as usize;
        if taille > buffer.len() {
            self.ecrire_registre(registres::IRQ_FLAGS, irq::TOUT_EFFACER)?;
            return Err(ErreursAirHaum::DonneesInvalides(format!(
                "SX1276 : paquet reçu ({} octets) plus grand que le buffer fourni ({})",
                taille, buffer.len()
            )));
        }

        let addr = self.lire_registre(registres::FIFO_RX_CURRENT_ADDR)?;
        self.ecrire_registre(registres::FIFO_ADDR_PTR, addr)?;
        self.lire_rafale(registres::FIFO, &mut buffer[..taille])?;

        self.ecrire_registre(registres::IRQ_FLAGS, irq::TOUT_EFFACER)?;
        Ok(Some(taille))
    }

    // ─── Émission (interruption brève du repos) ────────────────────────────

    /// Émet `payload`, attend la fin d'émission (bloquant, borné par
    /// `TIMEOUT_TX_MS`), puis repasse automatiquement en réception.
    pub fn envoyer(&mut self, payload: &[u8]) -> Result<()> {
        self.verifier_pret()?;

        if payload.is_empty() || payload.len() > TAILLE_MAX_PAYLOAD {
            return Err(ErreursAirHaum::DonneesInvalides(format!(
                "SX1276 : payload de {} octets hors plage [1, {}]",
                payload.len(), TAILLE_MAX_PAYLOAD
            )));
        }

        self.ecrire_registre(registres::OP_MODE, op_mode::LONG_RANGE_MODE | op_mode::MODE_STDBY)?;
        self.ecrire_registre(registres::FIFO_ADDR_PTR, 0x80)?;
        self.ecrire_rafale(registres::FIFO, payload)?;
        self.ecrire_registre(registres::PAYLOAD_LENGTH, payload.len() as u8)?;

        self.ecrire_registre(registres::IRQ_FLAGS, irq::TOUT_EFFACER)?;
        self.ecrire_registre(registres::OP_MODE, op_mode::LONG_RANGE_MODE | op_mode::MODE_TX)?;

        let debut = Instant::now();
        loop {
            let irq_flags = self.lire_registre(registres::IRQ_FLAGS)?;
            if irq_flags & irq::TX_DONE != 0 {
                self.ecrire_registre(registres::IRQ_FLAGS, irq::TOUT_EFFACER)?;
                break;
            }
            if debut.elapsed() > Duration::from_millis(TIMEOUT_TX_MS) {
                self.mettre_en_reception()?;
                return Err(ErreursAirHaum::Timeout);
            }
            std::thread::sleep(Duration::from_millis(PAS_POLLING_MS));
        }

        self.mettre_en_reception()
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Tests
// ─────────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::hal::spi::SpiMock;

    /// Précharge les réponses nécessaires à une initialisation réussie :
    /// VERSION correcte, puis IRQ_FLAGS=0x00 pour les lectures suivant
    /// initialiser() (aucune n'en a besoin ici, initialiser() n'attend rien).
    fn spi_avec_version_valide() -> SpiMock {
        let mut spi = SpiMock::nouveau();
        spi.precharger_reponse(&[0x00, VERSION_ATTENDUE]); // réponse à lire_registre(VERSION)
        spi
    }

    #[test]
    fn test_initialisation_ok() {
        let spi = spi_avec_version_valide();
        let mut radio = Sx1276::nouveau(spi);
        assert!(radio.initialiser(868_000_000).is_ok());
        assert!(radio.pret);
    }

    #[test]
    fn test_initialisation_version_invalide() {
        let mut spi = SpiMock::nouveau();
        spi.precharger_reponse(&[0x00, 0xFF]); // mauvaise version
        let mut radio = Sx1276::nouveau(spi);
        assert!(radio.initialiser(868_000_000).is_err());
    }

    #[test]
    fn test_recevoir_avant_init_erreur() {
        let spi = SpiMock::nouveau();
        let mut radio = Sx1276::nouveau(spi);
        let mut buf = [0u8; 16];
        assert!(radio.recevoir(&mut buf).is_err());
    }

    #[test]
    fn test_recevoir_rien_disponible() {
        let mut spi = spi_avec_version_valide();
        spi.precharger_reponse(&[0x00, 0x00]); // IRQ_FLAGS = 0 après initialiser()->mettre_en_reception()
        let mut radio = Sx1276::nouveau(spi);
        radio.initialiser(868_000_000).unwrap();

        let mut buf = [0u8; 16];
        assert_eq!(radio.recevoir(&mut buf).unwrap(), None);
    }

    // Le mock ne distingue pas lecture/écriture : chaque transaction SPI (y
    // compris une écriture de registre) consomme des octets de la file de
    // réponses préchargées, fidèle au comportement réel d'un bus full-duplex
    // (MISO renvoie toujours quelque chose, même pendant une écriture). Les
    // réponses doivent donc être préchargées *après* initialiser() — dont les
    // écritures internes consommeraient sinon les octets prévus pour le test —
    // et espacées de "bourrage" pour chaque écriture intercalée dans la
    // séquence testée.

    #[test]
    fn test_recevoir_paquet() {
        let spi = spi_avec_version_valide();
        let mut radio = Sx1276::nouveau(spi);
        radio.initialiser(868_000_000).unwrap();

        radio.spi.precharger_reponse(&[0x00, irq::RX_DONE]);    // IRQ_FLAGS
        radio.spi.precharger_reponse(&[0x00, 0x03]);            // RX_NB_BYTES
        radio.spi.precharger_reponse(&[0x00, 0x00]);            // FIFO_RX_CURRENT_ADDR
        radio.spi.precharger_reponse(&[0x00, 0x00]);            // bourrage : écriture FIFO_ADDR_PTR
        radio.spi.precharger_reponse(&[0x00, 0xAA, 0xBB, 0xCC]); // rafale FIFO (adresse + 3 octets utiles)

        let mut buf = [0u8; 16];
        let n = radio.recevoir(&mut buf).unwrap().unwrap();
        assert_eq!(n, 3);
        assert_eq!(&buf[..3], &[0xAA, 0xBB, 0xCC]);
    }

    #[test]
    fn test_recevoir_erreur_crc() {
        let spi = spi_avec_version_valide();
        let mut radio = Sx1276::nouveau(spi);
        radio.initialiser(868_000_000).unwrap();

        radio.spi.precharger_reponse(&[0x00, irq::RX_DONE | irq::PAYLOAD_CRC_ERROR]);

        let mut buf = [0u8; 16];
        assert!(radio.recevoir(&mut buf).is_err());
    }

    #[test]
    fn test_envoyer_payload_trop_grand() {
        let spi = spi_avec_version_valide();
        let mut radio = Sx1276::nouveau(spi);
        radio.initialiser(868_000_000).unwrap();

        let payload = [0u8; TAILLE_MAX_PAYLOAD + 1];
        assert!(radio.envoyer(&payload).is_err());
    }

    #[test]
    fn test_envoyer_succes() {
        let spi = spi_avec_version_valide();
        let mut radio = Sx1276::nouveau(spi);
        radio.initialiser(868_000_000).unwrap();

        // 6 écritures précèdent la première lecture d'IRQ_FLAGS dans envoyer() :
        // OP_MODE(standby, 2) + FIFO_ADDR_PTR(2) + rafale FIFO(4, payload 3 octets)
        // + PAYLOAD_LENGTH(2) + IRQ_FLAGS clear(2) + OP_MODE(TX, 2) = 14 octets de bourrage.
        radio.spi.precharger_reponse(&[0x00; 14]);
        radio.spi.precharger_reponse(&[0x00, irq::TX_DONE]); // IRQ_FLAGS lu dans la boucle d'attente

        assert!(radio.envoyer(&[0x01, 0x02, 0x03]).is_ok());
    }

    #[test]
    fn test_frequence_868mhz_calcul() {
        // Frf attendu pour 868 MHz avec un oscillateur 32 MHz : ~0xD90000
        let frf = ((868_000_000u64) << 19) / OSCILLATEUR_HZ;
        assert_eq!((frf >> 16) as u8, 0xD9);
    }
}
