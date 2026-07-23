// src/communications/protocole.rs
//! Enveloppe générique des trames applicatives échangées sur la liaison LoRa.
//!
//! Format binaire (little-endian) : `[type:1][séquence:2][payload:N][crc16:2]`.
//!
//! Ce module ne connaît pas le contenu du payload — sa structure (télémétrie,
//! commande, …) est définie dans `telemetrie.rs` / `commandes.rs`. Il ne fait
//! qu'encoder/décoder l'enveloppe et vérifier son intégrité, sur le même
//! principe que [`crate::drivers::gps::ubx_parser`] pour les trames UBX.
//!
//! Le CRC applicatif est redondant avec le CRC matériel du SX1276
//! (`RxPayloadCrcOn`, cf. `drivers/lora/sx1276.rs`), qui ne couvre que le saut
//! radio. Il protège en plus le saut USB/série Wemos → PC, qui n'a pas de
//! contrôle d'intégrité propre (cf. doc/lora.md, "Architecture matérielle").

use crate::drivers::lora::TAILLE_MAX_PAYLOAD as TAILLE_MAX_TRAME;
use crate::types::{Result, ErreursAirHaum};

/// Taille de l'en-tête : type (1 octet) + numéro de séquence (2 octets).
pub const TAILLE_ENTETE: usize = 3;
/// Taille du CRC en fin de trame.
pub const TAILLE_CRC: usize = 2;
/// Taille maximale du payload utile, une fois l'en-tête et le CRC déduits de
/// la taille maximale acceptée par [`crate::drivers::lora::Sx1276::envoyer`].
pub const TAILLE_MAX_PAYLOAD: usize = TAILLE_MAX_TRAME - TAILLE_ENTETE - TAILLE_CRC;

/// Type d'une trame applicative.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TypeMessage {
    /// Télémétrie descendante — best-effort, pas d'accusé de réception.
    Telemetrie,
    /// Commande montante — accusé de réception + retransmission attendus côté émetteur.
    Commande,
    /// Accusé de réception d'une commande.
    Ack,
    /// Arrêt d'urgence — traité dès réception côté Pi, sans attendre d'accusé.
    Urgence,
    /// Signal de vie périodique, dans les deux sens (détection de perte de liaison).
    Battement,
}

impl TypeMessage {
    fn vers_octet(self) -> u8 {
        match self {
            Self::Telemetrie => 0x01,
            Self::Commande   => 0x02,
            Self::Ack        => 0x03,
            Self::Urgence    => 0x04,
            Self::Battement  => 0x05,
        }
    }

    fn depuis_octet(octet: u8) -> Result<Self> {
        match octet {
            0x01 => Ok(Self::Telemetrie),
            0x02 => Ok(Self::Commande),
            0x03 => Ok(Self::Ack),
            0x04 => Ok(Self::Urgence),
            0x05 => Ok(Self::Battement),
            _ => Err(ErreursAirHaum::ProtocoleInvalide(
                format!("type de message inconnu : 0x{:02X}", octet)
            )),
        }
    }
}

/// Trame décodée — le payload est un emprunt du buffer d'origine (pas de copie).
#[derive(Debug, PartialEq, Eq)]
pub struct Trame<'a> {
    pub type_message: TypeMessage,
    pub sequence: u16,
    pub payload: &'a [u8],
}

/// Encode une trame dans `buffer` et retourne la longueur totale écrite.
///
/// `buffer` doit faire au moins `TAILLE_ENTETE + payload.len() + TAILLE_CRC`
/// octets ; le résultat est directement utilisable comme argument de
/// [`crate::drivers::lora::Sx1276::envoyer`].
pub fn encoder(
    type_message: TypeMessage,
    sequence: u16,
    payload: &[u8],
    buffer: &mut [u8],
) -> Result<usize> {
    if payload.len() > TAILLE_MAX_PAYLOAD {
        return Err(ErreursAirHaum::DonneesInvalides(format!(
            "protocole : payload de {} octets dépasse la taille maximale {}",
            payload.len(), TAILLE_MAX_PAYLOAD
        )));
    }
    let taille_trame = TAILLE_ENTETE + payload.len() + TAILLE_CRC;
    if buffer.len() < taille_trame {
        return Err(ErreursAirHaum::DonneesInvalides(
            "protocole : buffer de sortie trop petit".into()
        ));
    }

    buffer[0] = type_message.vers_octet();
    buffer[1..3].copy_from_slice(&sequence.to_le_bytes());
    buffer[3..3 + payload.len()].copy_from_slice(payload);

    let crc = crc16(&buffer[..TAILLE_ENTETE + payload.len()]);
    buffer[TAILLE_ENTETE + payload.len()..taille_trame].copy_from_slice(&crc.to_le_bytes());

    Ok(taille_trame)
}

/// Décode et vérifie une trame reçue.
pub fn decoder(buffer: &[u8]) -> Result<Trame<'_>> {
    if buffer.len() < TAILLE_ENTETE + TAILLE_CRC {
        return Err(ErreursAirHaum::TrameMalformee(
            "protocole : trame trop courte pour contenir un en-tête et un CRC".into()
        ));
    }

    let taille_payload = buffer.len() - TAILLE_ENTETE - TAILLE_CRC;
    let crc_recu = u16::from_le_bytes([
        buffer[TAILLE_ENTETE + taille_payload],
        buffer[TAILLE_ENTETE + taille_payload + 1],
    ]);
    let crc_calcule = crc16(&buffer[..TAILLE_ENTETE + taille_payload]);
    if crc_recu != crc_calcule {
        return Err(ErreursAirHaum::MessageCorrompu);
    }

    let type_message = TypeMessage::depuis_octet(buffer[0])?;
    let sequence = u16::from_le_bytes([buffer[1], buffer[2]]);
    let payload = &buffer[TAILLE_ENTETE..TAILLE_ENTETE + taille_payload];

    Ok(Trame { type_message, sequence, payload })
}

/// Compteur de séquence à enroulement (`u16`), partagé par les émetteurs
/// (télémétrie, commandes) pour l'anti-rejeu — voir doc/lora.md, "Format de trame".
#[derive(Debug, Default)]
pub struct CompteurSequence(u16);

impl CompteurSequence {
    pub fn nouveau() -> Self {
        Self(0)
    }

    /// Retourne le prochain numéro de séquence et avance le compteur.
    pub fn suivant(&mut self) -> u16 {
        let n = self.0;
        self.0 = self.0.wrapping_add(1);
        n
    }
}

/// CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) — choisi pour sa simplicité
/// d'implémentation sans table ni dépendance externe, pas pour une propriété
/// particulière (le CRC matériel du SX1276 couvre déjà la robustesse radio).
fn crc16(donnees: &[u8]) -> u16 {
    let mut crc: u16 = 0xFFFF;
    for &octet in donnees {
        crc ^= (octet as u16) << 8;
        for _ in 0..8 {
            crc = if crc & 0x8000 != 0 { (crc << 1) ^ 0x1021 } else { crc << 1 };
        }
    }
    crc
}

// ─────────────────────────────────────────────────────────────────────────────
// Tests
// ─────────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_aller_retour() {
        let mut buffer = [0u8; TAILLE_MAX_TRAME];
        let payload = b"Hello LoRa!";
        let n = encoder(TypeMessage::Telemetrie, 42, payload, &mut buffer).unwrap();

        let trame = decoder(&buffer[..n]).unwrap();
        assert_eq!(trame.type_message, TypeMessage::Telemetrie);
        assert_eq!(trame.sequence, 42);
        assert_eq!(trame.payload, payload);
    }

    #[test]
    fn test_payload_vide() {
        let mut buffer = [0u8; TAILLE_MAX_TRAME];
        let n = encoder(TypeMessage::Battement, 0, &[], &mut buffer).unwrap();
        assert_eq!(n, TAILLE_ENTETE + TAILLE_CRC);

        let trame = decoder(&buffer[..n]).unwrap();
        assert_eq!(trame.type_message, TypeMessage::Battement);
        assert!(trame.payload.is_empty());
    }

    #[test]
    fn test_payload_trop_grand() {
        let mut buffer = [0u8; TAILLE_MAX_TRAME + 16];
        let payload = vec![0u8; TAILLE_MAX_PAYLOAD + 1];
        assert!(encoder(TypeMessage::Telemetrie, 0, &payload, &mut buffer).is_err());
    }

    #[test]
    fn test_buffer_sortie_trop_petit() {
        let mut buffer = [0u8; 2];
        assert!(encoder(TypeMessage::Ack, 0, &[0x01], &mut buffer).is_err());
    }

    #[test]
    fn test_crc_corrompu_detecte() {
        let mut buffer = [0u8; TAILLE_MAX_TRAME];
        let n = encoder(TypeMessage::Commande, 7, b"RTL", &mut buffer).unwrap();
        buffer[3] ^= 0xFF; // corrompt un octet du payload
        assert!(matches!(decoder(&buffer[..n]), Err(ErreursAirHaum::MessageCorrompu)));
    }

    #[test]
    fn test_trame_trop_courte() {
        let buffer = [0u8; 3];
        assert!(decoder(&buffer).is_err());
    }

    #[test]
    fn test_type_inconnu_detecte() {
        let mut buffer = [0u8; TAILLE_MAX_TRAME];
        let n = encoder(TypeMessage::Urgence, 1, &[], &mut buffer).unwrap();
        buffer[0] = 0xEE; // type invalide
        // Le CRC ne couvre pas la sémantique du type, seulement les octets :
        // il faut recalculer un CRC valide pour isoler l'erreur testée.
        let crc = crc16(&buffer[..TAILLE_ENTETE]);
        buffer[TAILLE_ENTETE..TAILLE_ENTETE + TAILLE_CRC].copy_from_slice(&crc.to_le_bytes());
        assert!(matches!(decoder(&buffer[..n]), Err(ErreursAirHaum::ProtocoleInvalide(_))));
    }

    #[test]
    fn test_compteur_sequence_enroulement() {
        let mut compteur = CompteurSequence::nouveau();
        assert_eq!(compteur.suivant(), 0);
        assert_eq!(compteur.suivant(), 1);

        let mut compteur_max = CompteurSequence(u16::MAX);
        assert_eq!(compteur_max.suivant(), u16::MAX);
        assert_eq!(compteur_max.suivant(), 0); // enroulement
    }
}
