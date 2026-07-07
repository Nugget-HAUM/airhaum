// src/drivers/gps/ublox.rs
//! Driver u-blox NEO-M8N (protocole UBX binaire sur UART).
//!
//! Implémente [`CapteurGps`] sur n'importe quelle implémentation de [`PortSerie`].
//! En production : `DriverGps<PortSerieLinux>`.
//! En test : `DriverGps<PortSerieMock>`.

use crate::hal::uart::PortSerie;
use crate::interfaces::gps::CapteurGps;
use crate::types::{DonneesGps, ErreursAirHaum, Horodatage, TypeFixGps, Result};
use super::ubx_parser::UbxParseur;

// ─────────────────────────────────────────────────────────────────────────────
// Constantes
// ─────────────────────────────────────────────────────────────────────────────

/// Taille du buffer de lecture série (petit et fixe = faible latence).
const TAILLE_BUF_LECTURE: usize = 64;

/// Nombre d'octets drainés au démarrage pour vider le buffer UART stale.
const OCTETS_DRAIN_INIT: usize = 1024;

/// Baudrate usine du NEO-M8N (config sortie d'usine).
const BAUDRATE_USINE: u32 = 9_600;

/// Baudrate cible après configuration.
const BAUDRATE_CIBLE: u32 = 115_200;

/// Octets lus pour la détection du protocole au démarrage.
const OCTETS_DETECTION: usize = 512;

// ─────────────────────────────────────────────────────────────────────────────
// Protocole détecté à l'initialisation
// ─────────────────────────────────────────────────────────────────────────────

#[derive(Debug, PartialEq)]
enum ProtocoleDetecte {
    /// Trames NMEA (config usine — baudrate 9 600).
    Nmea,
    /// Trames UBX binaires (module déjà partiellement configuré).
    Ubx,
    /// Données présentes mais ni NMEA ni UBX (GPS probablement à 115 200,
    /// on lit du bruit car on est à 9 600 côté host).
    Inconnu,
    /// Aucune donnée reçue.
    Aucun,
}

// ─────────────────────────────────────────────────────────────────────────────
// Structure
// ─────────────────────────────────────────────────────────────────────────────

/// Driver GPS u-blox sur UART, générique sur le type de port série.
pub struct DriverGps<P: PortSerie> {
    port:             P,
    parseur:          UbxParseur,
    derniere_donnee:  Option<DonneesGps>,
    /// HDOP du dernier NAV-DOP reçu (mis à jour indépendamment de NAV-PVT).
    hdop:             Option<f32>,
}

// ─────────────────────────────────────────────────────────────────────────────
// Implémentation de l'interface
// ─────────────────────────────────────────────────────────────────────────────

impl<P: PortSerie> DriverGps<P> {
    pub fn nouveau(port: P) -> Self {
        Self {
            port,
            parseur:         UbxParseur::nouveau(),
            derniere_donnee: None,
            hdop:            None,
        }
    }

    /// Accès au port en test uniquement — permet d'injecter des données
    /// après l'initialisation (qui draine le buffer).
    #[cfg(test)]
    pub fn port_mut(&mut self) -> &mut P {
        &mut self.port
    }
}

impl<P: PortSerie> CapteurGps for DriverGps<P> {
    /// Initialise le module GPS.
    ///
    /// # Séquence
    ///
    /// 1. Ouvre le port à 9 600 bauds (vitesse usine du NEO-M8N)
    /// 2. Lit quelques octets et détecte le protocole (NMEA / UBX / inconnu)
    /// 3. Si NMEA → séquence de configuration complète puis passage à 115 200
    /// 4. Si UBX ou inconnu → reconfigure directement à 115 200 (module déjà configuré)
    /// 5. Draine le buffer final
    fn initialiser(&mut self) -> Result<()> {
        // ── Étape 1 : descendre à 9 600 pour la détection ─────────────────────
        self.port.reconfigurer_baudrate(BAUDRATE_USINE)
            .map_err(|e| ErreursAirHaum::ErreurIO(format!("GPS init 9600: {}", e)))?;

        std::thread::sleep(std::time::Duration::from_millis(150));

        // ── Étape 2 : lire et détecter le protocole ────────────────────────────
        let mut buf_det = [0u8; 64];
        let mut octets: Vec<u8> = Vec::with_capacity(OCTETS_DETECTION);
        for _ in 0..8 {
            match self.port.lire(&mut buf_det) {
                Ok(n) if n > 0 => octets.extend_from_slice(&buf_det[..n]),
                _ => {}
            }
            std::thread::sleep(std::time::Duration::from_millis(20));
            if octets.len() >= OCTETS_DETECTION { break; }
        }

        let protocole = detecter_protocole(&octets);
        log::info!(target: "gps", "Protocole détecté à {} bauds : {:?}", BAUDRATE_USINE, protocole);

        // ── Étape 3 : si NMEA → désactiver NMEA et basculer GPS à 115 200 ─────
        if protocole == ProtocoleDetecte::Nmea {
            log::info!(target: "gps", "Configuration du module (NMEA → UBX 115 200)");
            basculer_vers_ubx_115200(&mut self.port)?;
        }

        // ── Étape 4 : reconfigurer le host à 115 200 (tous les chemins) ────────
        std::thread::sleep(std::time::Duration::from_millis(100));
        self.port.reconfigurer_baudrate(BAUDRATE_CIBLE)
            .map_err(|e| ErreursAirHaum::ErreurIO(format!("GPS init 115200: {}", e)))?;

        // ── Étape 5 : activer les messages UBX (tous les chemins) ──────────────
        log::info!(target: "gps", "Activation des messages UBX sur UART1");
        activer_messages(&mut self.port)?;
        log::info!(target: "gps", "GPS initialisé");

        // ── Étape 6 : drainer le buffer final ─────────────────────────────────
        std::thread::sleep(std::time::Duration::from_millis(100));
        let mut buf = [0u8; 64];
        let mut draines = 0;
        while draines < OCTETS_DRAIN_INIT {
            match self.port.lire(&mut buf) {
                Ok(0) | Err(_) => break,
                Ok(n) => draines += n,
            }
        }

        Ok(())
    }

    fn mettre_a_jour(&mut self) -> bool {
        let mut buf = [0u8; TAILLE_BUF_LECTURE];
        let mut nouvelle_position = false;

        let n = match self.port.lire(&mut buf) {
            Ok(n)  => n,
            Err(e) if e.kind() == std::io::ErrorKind::TimedOut => return false,
            Err(e) => {
                log::warn!(target: "gps", "Erreur lecture UART : {}", e);
                return false;
            }
        };

        for &octet in &buf[..n] {
            if self.parseur.alimenter(octet) {
                // Trame complète — mise à jour de l'état interne
                if let Some(h) = self.parseur.last_entete() {
                    match (h.classe, h.id) {
                        (0x01, 0x07) => {
                            if let Some(pvt) = self.parseur.last_nav_pvt() {
                                self.derniere_donnee = Some(pvt_vers_donnees(pvt, self.hdop));
                                nouvelle_position = true;
                            }
                        }
                        (0x01, 0x04) => {
                            if let Some(dop) = self.parseur.last_nav_dop() {
                                self.hdop = Some(dop.h_dop as f32 * 0.01);
                            }
                        }
                        _ => {}
                    }
                }
            }
        }

        nouvelle_position
    }

    fn derniere_donnee(&self) -> Option<DonneesGps> {
        self.derniere_donnee
    }

    fn est_operationnel(&self) -> bool {
        self.derniere_donnee
            .map(|d| d.type_fix.est_valide())
            .unwrap_or(false)
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Conversion NAV-PVT → DonneesGps
// ─────────────────────────────────────────────────────────────────────────────

fn pvt_vers_donnees(pvt: super::ubx_parser::NavPvt, hdop: Option<f32>) -> DonneesGps {
    // headMot est en 1e-5 degrés dans NAV-PVT
    let cap_deg = pvt.head_mot as f32 * 1e-5;
    // Normalise le cap entre 0 et 360
    let cap = cap_deg.rem_euclid(360.0);

    // h_acc et v_acc sont en mm dans NAV-PVT
    // Si hdop est disponible (NAV-DOP), on l'utilise en précision horizontale
    // sinon on convertit h_acc en mètres
    let precision_h = hdop.unwrap_or(pvt.h_acc as f32 / 1000.0);
    let precision_v = pvt.v_acc as f32 / 1000.0;

    DonneesGps {
        horodatage:         Horodatage::maintenant(),
        latitude:           pvt.lat,
        longitude:          pvt.lon,
        altitude_msl:       pvt.h_msl as f32 / 1000.0,
        vitesse_sol:        pvt.g_speed as f32 / 1000.0,
        cap,
        vel_nord:           pvt.vel_n as f32 / 1000.0,
        vel_est:            pvt.vel_e as f32 / 1000.0,
        vel_bas:            pvt.vel_d as f32 / 1000.0,
        precision_h,
        precision_v,
        nombre_satellites:  pvt.num_sv,
        type_fix:           TypeFixGps::depuis_ubx(pvt.fix_type),
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Helpers d'initialisation
// ─────────────────────────────────────────────────────────────────────────────

/// Détecte le protocole à partir des octets reçus à 9 600 bauds.
fn detecter_protocole(octets: &[u8]) -> ProtocoleDetecte {
    if octets.is_empty() {
        return ProtocoleDetecte::Aucun;
    }
    // UBX : séquence de synchronisation 0xB5 0x62
    if octets.windows(2).any(|w| w == [0xB5, 0x62]) {
        return ProtocoleDetecte::Ubx;
    }
    // NMEA : toute trame valide commence par '$G' ($GP, $GN, $GL, $GA…)
    if octets.windows(2).any(|w| w[0] == b'$' && w[1] == b'G') {
        return ProtocoleDetecte::Nmea;
    }
    ProtocoleDetecte::Inconnu
}

/// Construit et envoie une trame UBX avec checksum Fletcher.
fn envoyer_ubx<P: PortSerie>(
    port: &mut P,
    classe: u8,
    id: u8,
    payload: &[u8],
) -> Result<()> {
    let len = payload.len() as u16;
    // Corps du message (sans les octets de synchro) : classe, id, len_lsb, len_msb, payload
    let mut corps: Vec<u8> = vec![classe, id, (len & 0xFF) as u8, (len >> 8) as u8];
    corps.extend_from_slice(payload);

    // Checksum Fletcher sur le corps
    let mut ck_a = 0u8;
    let mut ck_b = 0u8;
    for &b in &corps {
        ck_a = ck_a.wrapping_add(b);
        ck_b = ck_b.wrapping_add(ck_a);
    }

    // Trame complète : sync + corps + checksum
    let mut trame: Vec<u8> = vec![0xB5, 0x62];
    trame.extend_from_slice(&corps);
    trame.push(ck_a);
    trame.push(ck_b);

    port.ecrire(&trame)
        .map_err(|e| ErreursAirHaum::ErreurIO(format!("UBX {:02X}/{:02X}: {}", classe, id, e)))?;
    Ok(())
}

/// Exécutée à 9 600 bauds quand NMEA est détecté.
///
/// Désactive les trames NMEA et bascule le GPS à 115 200 bauds en sortie UBX.
/// Le host doit ensuite appeler `reconfigurer_baudrate(BAUDRATE_CIBLE)`.
fn basculer_vers_ubx_115200<P: PortSerie>(port: &mut P) -> Result<()> {
    // ── Désactiver les trames NMEA ────────────────────────────────────────────
    let nmea_msgs: &[(u8, u8)] = &[
        (0xF0, 0x00), // GxGGA
        (0xF0, 0x01), // GxGLL
        (0xF0, 0x02), // GxGSA
        (0xF0, 0x03), // GxGSV
        (0xF0, 0x04), // GxRMC
        (0xF0, 0x05), // GxVTG
    ];
    for &(classe, id) in nmea_msgs {
        envoyer_ubx(port, 0x06, 0x01, &[classe, id, 0, 0, 0, 0, 0, 0])?;
        std::thread::sleep(std::time::Duration::from_millis(100));
    }

    // ── CFG-PRT : bascule le GPS à 115 200 bauds, sortie UBX uniquement ──────
    // Le module bascule immédiatement — le host doit suivre juste après.
    #[rustfmt::skip]
    envoyer_ubx(port, 0x06, 0x00, &[
        0x01, 0x00, 0x00, 0x00,  // PortID = 1 (UART1 interne du GPS)
        0xD0, 0x08, 0x00, 0x00,  // mode : 8N1
        0x00, 0xC2, 0x01, 0x00,  // baudrate = 115 200
        0x07, 0x00,              // InProtoMask : UBX + NMEA
        0x01, 0x00,              // OutProtoMask : UBX uniquement
        0x00, 0x00, 0x00, 0x00,  // flags & reserved
    ])?;

    Ok(())
}

/// Exécutée à 115 200 bauds dans tous les cas.
///
/// Active les messages NAV sur UART1 du GPS et règle la cadence à 10 Hz.
/// Appelée après avoir atteint 115 200 bauds, que le GPS parte de NMEA 9 600
/// ou qu'il soit déjà à 115 200 avec un état de messages inconnu.
fn activer_messages<P: PortSerie>(port: &mut P) -> Result<()> {
    // ── Activer les messages UBX sur UART1 du GPS ─────────────────────────────
    // Payload CFG-MSG (8 octets) :
    // [msgClass, msgID, rate_DDC, rate_UART1, rate_UART2, rate_USB, rate_SPI, réservé]
    let ubx_msgs: &[(u8, u8)] = &[
        (0x01, 0x07), // NAV-PVT    — position, vitesse, heure
        (0x01, 0x04), // NAV-DOP    — dilution of precision
        (0x01, 0x03), // NAV-STATUS — état du fix
        (0x01, 0x20), // NAV-TIMEGPS — semaine GPS
    ];
    for &(classe, id) in ubx_msgs {
        envoyer_ubx(port, 0x06, 0x01, &[classe, id, 0, 1, 0, 0, 0, 0])?;
        std::thread::sleep(std::time::Duration::from_millis(100));
    }

    // ── CFG-RATE : cadence 100 ms (10 Hz) ────────────────────────────────────
    // measRate=100ms, navRate=1, timeRef=GPS time
    envoyer_ubx(port, 0x06, 0x08, &[0x64, 0x00, 0x01, 0x00, 0x01, 0x00])?;
    std::thread::sleep(std::time::Duration::from_millis(100));

    Ok(())
}

// ─────────────────────────────────────────────────────────────────────────────
// Tests
// ─────────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::hal::uart::PortSerieMock;

    /// Construit une trame NAV-PVT complète (92 octets) avec les champs donnés,
    /// correctement encadrée avec les bytes de synchronisation et le checksum UBX.
    fn trame_nav_pvt(lat_e7: i32, lon_e7: i32, h_msl_mm: i32, fix_type: u8,
                     g_speed_mm: i32, num_sv: u8) -> Vec<u8> {
        let mut payload = [0u8; 92];
        payload[20] = fix_type;
        payload[23] = num_sv;
        payload[24..28].copy_from_slice(&lon_e7.to_le_bytes());
        payload[28..32].copy_from_slice(&lat_e7.to_le_bytes());
        payload[36..40].copy_from_slice(&h_msl_mm.to_le_bytes());
        payload[60..64].copy_from_slice(&g_speed_mm.to_le_bytes());

        let len = payload.len() as u16;
        let mut trame = vec![0xB5, 0x62, 0x01, 0x07,
                             (len & 0xFF) as u8, (len >> 8) as u8];
        trame.extend_from_slice(&payload);
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
    fn pas_de_donnee_avant_premiere_trame() {
        let mock = PortSerieMock::nouveau();
        let driver = DriverGps::nouveau(mock);
        assert!(driver.derniere_donnee().is_none());
        assert!(!driver.est_operationnel());
    }

    #[test]
    fn position_extraite_apres_trame_valide() {
        // lat = 48.8566° N = 488566000 × 1e-7, lon = 2.3522° E = 23522000 × 1e-7
        let trame = trame_nav_pvt(488_566_000, 23_522_000, 50_000, 3, 5_000, 8);

        let mut driver = DriverGps::nouveau(PortSerieMock::nouveau());
        driver.initialiser().unwrap();
        // Injection APRÈS initialiser() qui draine le buffer stale.
        // La trame (100 octets) peut nécessiter plusieurs appels car le buffer
        // de lecture est 64 octets — on boucle jusqu'à complétion.
        driver.port_mut().injecter(&trame);
        let nouvelle = (0..5).any(|_| driver.mettre_a_jour());

        assert!(nouvelle, "une nouvelle position doit être détectée");
        let d = driver.derniere_donnee().expect("données attendues");
        assert!((d.latitude  - 48.8566).abs() < 1e-4, "latitude");
        assert!((d.longitude - 2.3522).abs()  < 1e-4, "longitude");
        assert!((d.altitude_msl - 50.0).abs() < 0.01, "altitude MSL");
        assert!((d.vitesse_sol - 5.0).abs()   < 0.01, "vitesse sol");
        assert_eq!(d.nombre_satellites, 8);
        assert_eq!(d.type_fix, TypeFixGps::Fix3D);
        assert!(driver.est_operationnel());
    }

    #[test]
    fn fix_invalide_non_operationnel() {
        let mut mock = PortSerieMock::nouveau();
        let trame = trame_nav_pvt(0, 0, 0, 0 /* no fix */, 0, 0);
        mock.injecter(&trame);

        let mut driver = DriverGps::nouveau(mock);
        driver.mettre_a_jour();
        assert!(!driver.est_operationnel());
    }

    #[test]
    fn retourne_false_sans_donnees() {
        let mock = PortSerieMock::nouveau();  // vide
        let mut driver = DriverGps::nouveau(mock);
        assert!(!driver.mettre_a_jour());
    }
}
