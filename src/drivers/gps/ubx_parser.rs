// src/drivers/gps/ubx_parser.rs
//! Parseur UBX streaming — zéro allocation à l'exécution.
//!
//! Automate à états (FSM) qui consomme un octet à la fois et reconnaît les
//! trames du protocole binaire u-blox UBX. Conçu pour l'embarqué :
//!
//! - Aucune allocation dynamique (buffer payload fixe)
//! - Checksum Fletcher vérifié avant d'accepter une trame
//! - Résynchronisation automatique sur n'importe quel octet errant
//! - Taille payload bornée : trame surdimensionnée → resync silencieux
//!
//! # Messages parsés
//!
//! | Classe | ID   | Nom          | Contenu principal                        |
//! |--------|------|--------------|------------------------------------------|
//! | 0x01   | 0x07 | NAV-PVT      | Position, vitesse, cap, heure, qualité   |
//! | 0x01   | 0x03 | NAV-STATUS   | État fix, TTFF                           |
//! | 0x01   | 0x04 | NAV-DOP      | Dilution of Precision (HDOP, VDOP…)      |
//!
//! # Format trame UBX
//!
//! ```text
//! 0xB5  0x62  CLASS  ID  LEN_LSB  LEN_MSB  [PAYLOAD × LEN]  CK_A  CK_B
//! ```

use core::convert::TryInto;

// ─────────────────────────────────────────────────────────────────────────────
// Constantes
// ─────────────────────────────────────────────────────────────────────────────

/// Taille maximale du payload (suffit pour NAV-PVT = 92 octets et la plupart
/// des messages NAV-*). Une trame plus grande est ignorée (resync).
pub const TAILLE_MAX_PAYLOAD: usize = 512;

// ─────────────────────────────────────────────────────────────────────────────
// États de l'automate
// ─────────────────────────────────────────────────────────────────────────────

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum EtatParseur {
    AttenteSync1,   // Attente du premier octet de synchronisation (0xB5)
    AttenteSync2,   // Attente du second octet de synchronisation (0x62)
    LectureClasse,
    LectureId,
    LectureLongueurLsb,
    LectureLongueurMsb,
    LecturePayload,
    LectureCkA,
    LectureCkB,
}

// ─────────────────────────────────────────────────────────────────────────────
// Structures de données parsées
// ─────────────────────────────────────────────────────────────────────────────

/// NAV-PVT (0x01 / 0x07) — message principal de navigation u-blox.
///
/// Payload standard : 92 octets (firmware u-blox M8).
/// Tous les champs sont en unités natives UBX ; les conversions SI
/// sont effectuées dans le driver [`super::ublox::DriverGps`].
#[derive(Debug, Clone, Copy)]
pub struct NavPvt {
    pub i_tow:     u32,  // GPS Time of Week, ms
    pub fix_type:  u8,   // 0=no fix, 2=2D, 3=3D, 4=GNSS+DR
    pub flags:     u8,   // bit0=gnssFixOK
    pub num_sv:    u8,   // nombre de satellites
    pub lon:       f64,  // degrés (converti depuis i32 × 1e-7)
    pub lat:       f64,  // degrés (converti depuis i32 × 1e-7)
    pub height:    i32,  // altitude ellipsoïde, mm
    pub h_msl:     i32,  // altitude MSL, mm
    pub h_acc:     u32,  // précision horizontale, mm
    pub v_acc:     u32,  // précision verticale, mm
    pub vel_n:     i32,  // vitesse Nord, mm/s
    pub vel_e:     i32,  // vitesse Est, mm/s
    pub vel_d:     i32,  // vitesse Bas (down), mm/s
    pub g_speed:   i32,  // vitesse sol, mm/s
    pub head_mot:  i32,  // cap de déplacement, 1e-5 degrés
}

/// NAV-STATUS (0x01 / 0x03).
#[derive(Debug, Clone, Copy)]
pub struct NavStatus {
    pub i_tow:    u32,
    pub gps_fix:  u8,
    pub flags:    u8,
    pub fix_stat: u8,
    pub ttff:     u32, // Time to First Fix, ms
    pub msss:     u32, // ms depuis le démarrage
}

/// NAV-DOP (0x01 / 0x04) — Dilution of Precision.
#[derive(Debug, Clone, Copy)]
pub struct NavDop {
    pub i_tow: u32,
    pub g_dop: u16,
    pub p_dop: u16,
    pub t_dop: u16,
    pub v_dop: u16,
    pub h_dop: u16,
    pub n_dop: u16,
    pub e_dop: u16,
}

/// En-tête du dernier message reconnu.
#[derive(Debug, Clone, Copy)]
pub struct EnteteMessage {
    pub classe:   u8,
    pub id:       u8,
    pub longueur: u16,
}

// ─────────────────────────────────────────────────────────────────────────────
// Parseur
// ─────────────────────────────────────────────────────────────────────────────

pub struct UbxParseur {
    etat:        EtatParseur,
    classe:      u8,
    id:          u8,
    longueur:    u16,
    idx_payload: usize,
    payload:     [u8; TAILLE_MAX_PAYLOAD],
    ck_a:        u8,
    ck_b:        u8,
    // Derniers messages décodés
    last_entete:     Option<EnteteMessage>,
    last_nav_pvt:    Option<NavPvt>,
    last_nav_status: Option<NavStatus>,
    last_nav_dop:    Option<NavDop>,
}

impl UbxParseur {
    pub fn nouveau() -> Self {
        Self {
            etat:        EtatParseur::AttenteSync1,
            classe:      0,
            id:          0,
            longueur:    0,
            idx_payload: 0,
            payload:     [0u8; TAILLE_MAX_PAYLOAD],
            ck_a:        0,
            ck_b:        0,
            last_entete:     None,
            last_nav_pvt:    None,
            last_nav_status: None,
            last_nav_dop:    None,
        }
    }

    /// Alimente l'automate avec un octet.
    ///
    /// Retourne `true` si une trame complète et valide vient d'être reconnue.
    /// Dans ce cas, les getters [`last_nav_pvt`], [`last_nav_status`],
    /// [`last_nav_dop`] et [`last_entete`] sont mis à jour.
    ///
    /// [`last_nav_pvt`]: UbxParseur::last_nav_pvt
    /// [`last_nav_status`]: UbxParseur::last_nav_status
    /// [`last_nav_dop`]: UbxParseur::last_nav_dop
    /// [`last_entete`]: UbxParseur::last_entete
    pub fn alimenter(&mut self, octet: u8) -> bool {
        match self.etat {
            EtatParseur::AttenteSync1 => {
                if octet == 0xB5 { self.etat = EtatParseur::AttenteSync2; }
                false
            }
            EtatParseur::AttenteSync2 => {
                if octet == 0x62 {
                    self.ck_a = 0;
                    self.ck_b = 0;
                    self.etat = EtatParseur::LectureClasse;
                } else if octet != 0xB5 {
                    // 0xB5 répété : on reste en AttenteSync2
                    self.etat = EtatParseur::AttenteSync1;
                }
                false
            }
            EtatParseur::LectureClasse => {
                self.classe = octet;
                self.accumule_checksum(octet);
                self.etat = EtatParseur::LectureId;
                false
            }
            EtatParseur::LectureId => {
                self.id = octet;
                self.accumule_checksum(octet);
                self.etat = EtatParseur::LectureLongueurLsb;
                false
            }
            EtatParseur::LectureLongueurLsb => {
                self.longueur = octet as u16;
                self.accumule_checksum(octet);
                self.etat = EtatParseur::LectureLongueurMsb;
                false
            }
            EtatParseur::LectureLongueurMsb => {
                self.longueur |= (octet as u16) << 8;
                self.accumule_checksum(octet);
                if (self.longueur as usize) > TAILLE_MAX_PAYLOAD {
                    // Payload trop grand — trame invalide, resync
                    self.etat = EtatParseur::AttenteSync1;
                    return false;
                }
                self.idx_payload = 0;
                self.etat = if self.longueur == 0 {
                    EtatParseur::LectureCkA
                } else {
                    EtatParseur::LecturePayload
                };
                false
            }
            EtatParseur::LecturePayload => {
                if self.idx_payload < TAILLE_MAX_PAYLOAD {
                    self.payload[self.idx_payload] = octet;
                }
                self.idx_payload += 1;
                self.accumule_checksum(octet);
                if self.idx_payload >= self.longueur as usize {
                    self.etat = EtatParseur::LectureCkA;
                }
                false
            }
            EtatParseur::LectureCkA => {
                if octet == self.ck_a {
                    self.etat = EtatParseur::LectureCkB;
                } else {
                    self.etat = EtatParseur::AttenteSync1;
                }
                false
            }
            EtatParseur::LectureCkB => {
                let valide = octet == self.ck_b;
                self.etat = EtatParseur::AttenteSync1;
                if valide {
                    self.last_entete = Some(EnteteMessage {
                        classe:   self.classe,
                        id:       self.id,
                        longueur: self.longueur,
                    });
                    match (self.classe, self.id) {
                        (0x01, 0x07) => self.parse_nav_pvt(),
                        (0x01, 0x03) => self.parse_nav_status(),
                        (0x01, 0x04) => self.parse_nav_dop(),
                        _            => {}
                    }
                }
                valide
            }
        }
    }

    // ── Getters ───────────────────────────────────────────────────────────────

    pub fn last_nav_pvt(&self)    -> Option<NavPvt>       { self.last_nav_pvt }
    pub fn last_nav_status(&self) -> Option<NavStatus>    { self.last_nav_status }
    pub fn last_nav_dop(&self)    -> Option<NavDop>        { self.last_nav_dop }
    pub fn last_entete(&self)     -> Option<EnteteMessage> { self.last_entete }

    /// Copie le payload brut du dernier message dans `dest`.
    /// Retourne le nombre d'octets copiés.
    pub fn copier_payload(&self, dest: &mut [u8]) -> usize {
        if let Some(h) = self.last_entete {
            let n = (h.longueur as usize).min(dest.len());
            dest[..n].copy_from_slice(&self.payload[..n]);
            n
        } else {
            0
        }
    }

    // ── Internals ─────────────────────────────────────────────────────────────

    #[inline]
    fn accumule_checksum(&mut self, octet: u8) {
        self.ck_a = self.ck_a.wrapping_add(octet);
        self.ck_b = self.ck_b.wrapping_add(self.ck_a);
    }

    // ── Parseurs de payload ───────────────────────────────────────────────────

    /// NAV-PVT (0x01/0x07) — 92 octets minimum.
    ///
    /// Offsets NAV-PVT (u-blox M8, firmware ≥ 2.0) :
    /// ```text
    ///  0  iTOW (u32)     4  year (u16)   6  month   7  day
    ///  8  hour           9  min         10  sec     11  valid
    /// 12  tAcc (u32)    16  nano (i32)  20  fixType 21  flags
    /// 22  flags2        23  numSV
    /// 24  lon (i32)     28  lat (i32)   32  height  36  hMSL
    /// 40  hAcc (u32)    44  vAcc (u32)
    /// 48  velN (i32)    52  velE        56  velD    60  gSpeed
    /// 64  headMot (i32) …
    /// ```
    fn parse_nav_pvt(&mut self) {
        let plen = self.longueur as usize;
        if plen < 44 { self.last_nav_pvt = None; return; }

        let p = &self.payload;

        let lon_raw = lire_i32(p, 24).unwrap_or(0);
        let lat_raw = lire_i32(p, 28).unwrap_or(0);

        // Vitesses : disponibles seulement si le payload est assez long
        let vel_n   = if plen >= 52 { lire_i32(p, 48).unwrap_or(0) } else { 0 };
        let vel_e   = if plen >= 56 { lire_i32(p, 52).unwrap_or(0) } else { 0 };
        let vel_d   = if plen >= 60 { lire_i32(p, 56).unwrap_or(0) } else { 0 };
        let g_speed = if plen >= 64 { lire_i32(p, 60).unwrap_or(0) } else { 0 };
        let head_mot = if plen >= 68 { lire_i32(p, 64).unwrap_or(0) } else { 0 };

        self.last_nav_pvt = Some(NavPvt {
            i_tow:    lire_u32(p, 0).unwrap_or(0),
            fix_type: p.get(20).copied().unwrap_or(0),
            flags:    p.get(21).copied().unwrap_or(0),
            num_sv:   p.get(23).copied().unwrap_or(0),
            lon:      lon_raw as f64 * 1e-7,
            lat:      lat_raw as f64 * 1e-7,
            height:   lire_i32(p, 32).unwrap_or(0),
            h_msl:    lire_i32(p, 36).unwrap_or(0),
            h_acc:    lire_u32(p, 40).unwrap_or(0),
            v_acc:    lire_u32(p, 44).unwrap_or(0),
            vel_n,
            vel_e,
            vel_d,
            g_speed,
            head_mot,
        });
    }

    /// NAV-STATUS (0x01/0x03) — 16 octets.
    fn parse_nav_status(&mut self) {
        if (self.longueur as usize) < 16 { self.last_nav_status = None; return; }
        let p = &self.payload;
        self.last_nav_status = Some(NavStatus {
            i_tow:    lire_u32(p, 0).unwrap_or(0),
            gps_fix:  p.get(4).copied().unwrap_or(0),
            flags:    p.get(5).copied().unwrap_or(0),
            fix_stat: p.get(6).copied().unwrap_or(0),
            ttff:     lire_u32(p, 8).unwrap_or(0),
            msss:     lire_u32(p, 12).unwrap_or(0),
        });
    }

    /// NAV-DOP (0x01/0x04) — 18 octets.
    fn parse_nav_dop(&mut self) {
        if (self.longueur as usize) < 18 { self.last_nav_dop = None; return; }
        let p = &self.payload;
        self.last_nav_dop = Some(NavDop {
            i_tow: lire_u32(p, 0).unwrap_or(0),
            g_dop: lire_u16(p, 4).unwrap_or(0),
            p_dop: lire_u16(p, 6).unwrap_or(0),
            t_dop: lire_u16(p, 8).unwrap_or(0),
            v_dop: lire_u16(p, 10).unwrap_or(0),
            h_dop: lire_u16(p, 12).unwrap_or(0),
            n_dop: lire_u16(p, 14).unwrap_or(0),
            e_dop: lire_u16(p, 16).unwrap_or(0),
        });
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Helpers de lecture little-endian (retournent Option pour sécurité)
// ─────────────────────────────────────────────────────────────────────────────

fn lire_u16(b: &[u8], o: usize) -> Option<u16> {
    if o + 2 <= b.len() {
        Some(u16::from_le_bytes(b[o..o+2].try_into().unwrap()))
    } else { None }
}

fn lire_u32(b: &[u8], o: usize) -> Option<u32> {
    if o + 4 <= b.len() {
        Some(u32::from_le_bytes(b[o..o+4].try_into().unwrap()))
    } else { None }
}

fn lire_i32(b: &[u8], o: usize) -> Option<i32> {
    if o + 4 <= b.len() {
        Some(i32::from_le_bytes(b[o..o+4].try_into().unwrap()))
    } else { None }
}

// ─────────────────────────────────────────────────────────────────────────────
// Tests
// ─────────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    /// Construit une trame UBX valide avec le payload donné.
    fn trame_ubx(classe: u8, id: u8, payload: &[u8]) -> Vec<u8> {
        let len = payload.len() as u16;
        let mut trame = vec![0xB5, 0x62, classe, id,
                             (len & 0xFF) as u8, (len >> 8) as u8];
        trame.extend_from_slice(payload);
        // Checksum Fletcher
        let mut ck_a = 0u8;
        let mut ck_b = 0u8;
        for &b in &trame[2..] {
            ck_a = ck_a.wrapping_add(b);
            ck_b = ck_b.wrapping_add(ck_a);
        }
        trame.push(ck_a);
        trame.push(ck_b);
        trame
    }

    #[test]
    fn resync_sur_octets_aleatoires() {
        let mut p = UbxParseur::nouveau();
        // Octets parasites avant la vraie trame
        for &b in &[0x00, 0xFF, 0xB5, 0x00, 0xAA] {
            assert!(!p.alimenter(b));
        }
        // Trame vide (classe=0x05, id=0x01, pas de payload)
        let trame = trame_ubx(0x05, 0x01, &[]);
        let n = trame.len();
        for (i, &b) in trame.iter().enumerate() {
            let complet = p.alimenter(b);
            if i == n - 1 {
                assert!(complet, "la trame doit être reconnue au dernier octet");
            } else {
                assert!(!complet);
            }
        }
        let h = p.last_entete().expect("entête attendu");
        assert_eq!(h.classe, 0x05);
        assert_eq!(h.id, 0x01);
    }

    #[test]
    fn checksum_invalide_ignore() {
        let mut trame = trame_ubx(0x01, 0x07, &[0u8; 92]);
        // Corrompt le checksum
        let n = trame.len();
        trame[n - 1] ^= 0xFF;
        let mut p = UbxParseur::nouveau();
        let mut complet = false;
        for &b in &trame { complet |= p.alimenter(b); }
        assert!(!complet, "trame avec checksum invalide doit être rejetée");
        assert!(p.last_nav_pvt().is_none());
    }

    #[test]
    fn offsets_velocites_nav_pvt() {
        // Construit un payload NAV-PVT de 92 octets avec des vitesses connues.
        // velN=1000 mm/s, velE=2000, velD=3000, gSpeed=4000
        let mut payload = [0u8; 92];
        // lon=0, lat=0 (offset 24/28)
        // hMSL = 50000 mm (offset 36)
        payload[36..40].copy_from_slice(&50_000i32.to_le_bytes());
        // hAcc (offset 40), vAcc (offset 44) = 0
        // velN @ 48
        payload[48..52].copy_from_slice(&1_000i32.to_le_bytes());
        // velE @ 52
        payload[52..56].copy_from_slice(&2_000i32.to_le_bytes());
        // velD @ 56
        payload[56..60].copy_from_slice(&3_000i32.to_le_bytes());
        // gSpeed @ 60
        payload[60..64].copy_from_slice(&4_000i32.to_le_bytes());
        // fix_type @ 20 = 3 (3D)
        payload[20] = 3;

        let trame = trame_ubx(0x01, 0x07, &payload);
        let mut p = UbxParseur::nouveau();
        for &b in &trame { p.alimenter(b); }

        let pvt = p.last_nav_pvt().expect("NAV-PVT attendu");
        assert_eq!(pvt.vel_n, 1_000, "velN");
        assert_eq!(pvt.vel_e, 2_000, "velE");
        assert_eq!(pvt.vel_d, 3_000, "velD");
        assert_eq!(pvt.g_speed, 4_000, "gSpeed");
        assert_eq!(pvt.fix_type, 3);
    }

    #[test]
    fn nav_dop_parse() {
        let mut payload = [0u8; 18];
        // hDOP @ offset 12 = 120 (= 1.20 avec facteur 0.01)
        payload[12..14].copy_from_slice(&120u16.to_le_bytes());
        let trame = trame_ubx(0x01, 0x04, &payload);
        let mut p = UbxParseur::nouveau();
        for &b in &trame { p.alimenter(b); }
        let dop = p.last_nav_dop().expect("NAV-DOP attendu");
        assert_eq!(dop.h_dop, 120);
    }
}
