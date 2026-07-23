// src/drivers/gps/calibration.rs
//! Assistance GPS persistante — registre de terrains connus (position + base
//! d'orbites AssistNow Autonomous par terrain).
//!
//! Réduit le TTFF au redémarrage en l'absence de sauvegarde matérielle (VBCKP)
//! sur le module, ou lorsque les éphémérides sont périmées (position toujours
//! utile, elle). Un seul terrain peut être « actif » à la fois — c'est celui
//! injecté au démarrage du GPS (`DriverGps::initialiser`). Voir
//! `doc/assistance_gps.md`.

use crate::types::{Result, Horodatage};
use crate::systeme::calibration::CalibrationPersistante;

// ─────────────────────────────────────────────────────────────────────────────
// Valeur protocole — capturée/injectée par le driver, sans notion de nom
// ─────────────────────────────────────────────────────────────────────────────

/// Assistance GPS telle que manipulée par le protocole UBX (voir
/// [`crate::interfaces::gps::AssistanceGnss`]) : position et base d'orbites,
/// sans identité de terrain — c'est la couche registre ci-dessous qui associe
/// un nom à une valeur de ce type pour la persistance.
#[derive(Debug, Clone)]
pub struct AssistanceGps {
    timestamp_unix_sec: u64,
    pub latitude:     f64,
    pub longitude:    f64,
    pub altitude_msl: f32,
    /// Trames UBX-MGA-DBD capturées, chacune préfixée de sa longueur (u16 LE).
    pub orbites: Vec<u8>,
}

impl AssistanceGps {
    pub fn nouvelle(latitude: f64, longitude: f64, altitude_msl: f32, orbites: Vec<u8>) -> Self {
        Self {
            timestamp_unix_sec: unix_maintenant(),
            latitude, longitude, altitude_msl, orbites,
        }
    }
}

fn unix_maintenant() -> u64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap_or_default()
        .as_secs()
}

fn vers_hex(donnees: &[u8]) -> String {
    donnees.iter().map(|b| format!("{:02x}", b)).collect()
}

fn depuis_hex(texte: &str) -> Option<Vec<u8>> {
    if texte.len() % 2 != 0 { return None; }
    (0..texte.len())
        .step_by(2)
        .map(|i| u8::from_str_radix(&texte[i..i + 2], 16).ok())
        .collect()
}

// ─────────────────────────────────────────────────────────────────────────────
// Entrée nommée du registre — un terrain connu
// ─────────────────────────────────────────────────────────────────────────────

/// Un terrain de vol connu : position + orbites associées, identifié par nom.
///
/// `orbites` peut être vide — cas d'un terrain déclaré (à la main, dans le
/// fichier) mais jamais visité avec une sauvegarde réelle sur site (console
/// de test, option « sauvegarder assistance »). La position seule reste
/// utile comme aide `TIME_UTC`/`POS_LLH`, même sans orbites.
#[derive(Debug, Clone)]
pub struct EntreeTerrain {
    pub nom: String,
    timestamp_unix_sec: u64,
    pub latitude: f64,
    pub longitude: f64,
    pub altitude_msl: f32,
    orbites: Vec<u8>,
}

impl EntreeTerrain {
    /// Construit une entrée à partir d'une assistance capturée sur site.
    pub fn depuis_assistance(nom: String, assistance: &AssistanceGps) -> Self {
        Self {
            nom,
            timestamp_unix_sec: assistance.timestamp_unix_sec,
            latitude:           assistance.latitude,
            longitude:          assistance.longitude,
            altitude_msl:       assistance.altitude_msl,
            orbites:            assistance.orbites.clone(),
        }
    }

    /// Convertit vers la valeur attendue par `AssistanceGnss::importer_assistance`.
    pub fn vers_assistance(&self) -> AssistanceGps {
        AssistanceGps {
            timestamp_unix_sec: self.timestamp_unix_sec,
            latitude:           self.latitude,
            longitude:          self.longitude,
            altitude_msl:       self.altitude_msl,
            orbites:            self.orbites.clone(),
        }
    }

    /// Nombre d'octets d'orbites stockés — 0 si le terrain n'a jamais été
    /// sauvegardé sur site (déclaré à la main, position seule).
    pub fn taille_orbites(&self) -> usize {
        self.orbites.len()
    }

    /// Temps écoulé depuis la dernière sauvegarde réelle sur site, en
    /// secondes. `timestamp_unix_sec` vaut 0 pour un terrain jamais visité
    /// (déclaré à la main) — dans ce cas l'âge n'a pas de sens et vaut 0.
    pub fn age_secondes(&self) -> u64 {
        if self.timestamp_unix_sec == 0 {
            return 0;
        }
        unix_maintenant().saturating_sub(self.timestamp_unix_sec)
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Registre persistant — liste de terrains + sélection active
// ─────────────────────────────────────────────────────────────────────────────

/// Registre des terrains de vol connus, avec un terrain actif (celui injecté
/// au démarrage du GPS). Fichier volontairement éditable à la main : ajouter
/// un terrain ne demande que `nom`/`latitude`/`longitude` (le reste a des
/// valeurs par défaut raisonnables), voir [`RegistreTerrains::depuis_toml`].
#[derive(Debug, Clone, Default)]
pub struct RegistreTerrains {
    pub terrain_actif: Option<String>,
    pub terrains: Vec<EntreeTerrain>,
}

impl RegistreTerrains {
    pub fn vide() -> Self {
        Self { terrain_actif: None, terrains: Vec::new() }
    }

    pub fn trouver(&self, nom: &str) -> Option<&EntreeTerrain> {
        self.terrains.iter().find(|t| t.nom == nom)
    }

    /// L'entrée correspondant à [`terrain_actif`](Self::terrain_actif), s'il
    /// est défini et connu du registre.
    pub fn entree_active(&self) -> Option<&EntreeTerrain> {
        self.terrain_actif.as_deref().and_then(|nom| self.trouver(nom))
    }

    /// Met à jour l'entrée nommée `nom` avec une assistance fraîchement
    /// capturée sur site, ou la crée si elle n'existe pas encore.
    pub fn definir(&mut self, nom: &str, assistance: &AssistanceGps) {
        match self.terrains.iter_mut().find(|t| t.nom == nom) {
            Some(existant) => *existant = EntreeTerrain::depuis_assistance(nom.to_string(), assistance),
            None => self.terrains.push(EntreeTerrain::depuis_assistance(nom.to_string(), assistance)),
        }
    }
}

impl CalibrationPersistante for RegistreTerrains {
    fn identifiant_capteur() -> &'static str {
        "terrains_gps"
    }

    fn vers_toml(&self) -> String {
        let mut s = String::new();
        s.push_str(
            "# Registre des terrains de vol GPS — position + orbites AssistNow\n\
             # Éditable à la main : ajouter un terrain ne demande que nom/latitude/\n\
             # longitude (les autres champs ont des valeurs par défaut) ; altitude_msl,\n\
             # timestamp_unix_sec et orbites_hex sont mis à jour automatiquement par une\n\
             # sauvegarde réelle sur site (console de test, option « sauvegarder\n\
             # assistance »). Voir doc/assistance_gps.md.\n\n"
        );

        match &self.terrain_actif {
            Some(nom) => s.push_str(&format!("terrain_actif = \"{}\"\n\n", nom)),
            None      => s.push_str("# terrain_actif = \"\"  (aucun terrain sélectionné)\n\n"),
        }

        for (i, t) in self.terrains.iter().enumerate() {
            s.push_str(&format!("terrain.{i}.nom = \"{}\"\n",       t.nom));
            s.push_str(&format!("terrain.{i}.latitude = {}\n",      t.latitude));
            s.push_str(&format!("terrain.{i}.longitude = {}\n",     t.longitude));
            s.push_str(&format!("terrain.{i}.altitude_msl = {}\n",  t.altitude_msl));
            s.push_str(&format!("terrain.{i}.timestamp_unix_sec = {}\n", t.timestamp_unix_sec));
            s.push_str(&format!("terrain.{i}.orbites_hex = \"{}\"\n\n",  vers_hex(&t.orbites)));
        }

        s
    }

    fn depuis_toml(contenu: &str) -> Result<Self> {
        use std::collections::BTreeMap;

        struct Partiel {
            nom:                Option<String>,
            latitude:           Option<f64>,
            longitude:          Option<f64>,
            altitude_msl:       Option<f32>,
            timestamp_unix_sec: Option<u64>,
            orbites:            Option<Vec<u8>>,
        }
        impl Partiel {
            fn vide() -> Self {
                Self { nom: None, latitude: None, longitude: None, altitude_msl: None,
                       timestamp_unix_sec: None, orbites: None }
            }
        }

        let mut terrain_actif = None;
        let mut partiels: BTreeMap<usize, Partiel> = BTreeMap::new();

        for ligne in contenu.lines() {
            let ligne = ligne.trim();
            if ligne.starts_with('#') || ligne.is_empty() {
                continue;
            }
            if let Some((cle, valeur)) = ligne.split_once('=') {
                let cle    = cle.trim();
                let valeur = valeur.trim().trim_matches('"');

                if cle == "terrain_actif" {
                    terrain_actif = Some(valeur.to_string());
                } else if let Some(reste) = cle.strip_prefix("terrain.") {
                    if let Some((idx_str, champ)) = reste.split_once('.') {
                        if let Ok(idx) = idx_str.parse::<usize>() {
                            let p = partiels.entry(idx).or_insert_with(Partiel::vide);
                            match champ {
                                "nom"                => p.nom = Some(valeur.to_string()),
                                "latitude"           => p.latitude = valeur.parse().ok(),
                                "longitude"          => p.longitude = valeur.parse().ok(),
                                "altitude_msl"       => p.altitude_msl = valeur.parse().ok(),
                                "timestamp_unix_sec" => p.timestamp_unix_sec = valeur.parse().ok(),
                                "orbites_hex"        => p.orbites = depuis_hex(valeur),
                                _ => {}
                            }
                        }
                    }
                }
            }
        }

        let mut terrains = Vec::new();
        for (idx, p) in partiels {
            match (p.nom, p.latitude, p.longitude) {
                (Some(nom), Some(latitude), Some(longitude)) => {
                    terrains.push(EntreeTerrain {
                        nom,
                        latitude,
                        longitude,
                        altitude_msl:       p.altitude_msl.unwrap_or(0.0),
                        timestamp_unix_sec: p.timestamp_unix_sec.unwrap_or(0),
                        orbites:            p.orbites.unwrap_or_default(),
                    });
                }
                _ => log::warn!(target: "gps",
                    "Terrain #{} incomplet (nom/latitude/longitude requis) — ignoré", idx),
            }
        }

        Ok(Self { terrain_actif, terrains })
    }

    /// Le registre lui-même n'expire pas — contrairement à une calibration
    /// capteur, la position d'un terrain reste valable indéfiniment (voir
    /// doc/assistance_gps.md). L'expiration au sens `GestionnaireCalibration`
    /// rendrait le fichier ignoré au chargement, ce qui n'a pas de sens ici.
    fn est_valide(&self) -> bool {
        true
    }

    fn obtenir_horodatage(&self) -> Horodatage {
        Horodatage::maintenant()
    }

    fn age_secondes(&self) -> u64 {
        self.entree_active().map(EntreeTerrain::age_secondes).unwrap_or(0)
    }

    fn duree_validite_secondes(&self) -> u64 {
        u64::MAX
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Tests
// ─────────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    fn assistance_test() -> AssistanceGps {
        AssistanceGps::nouvelle(47.993716, 0.185957, 55.5, vec![0xAA, 0xBB, 0x01, 0x00, 0xFF])
    }

    #[test]
    fn roundtrip_registre() {
        let mut r = RegistreTerrains::vide();
        r.definir("Piste principale", &assistance_test());
        r.definir("Terrain B", &AssistanceGps::nouvelle(48.0, 2.0, 100.0, vec![]));
        r.terrain_actif = Some("Piste principale".into());

        let toml = r.vers_toml();
        let r2 = RegistreTerrains::depuis_toml(&toml).unwrap();

        assert_eq!(r2.terrain_actif.as_deref(), Some("Piste principale"));
        assert_eq!(r2.terrains.len(), 2);
        let t = r2.trouver("Piste principale").unwrap();
        assert!((t.latitude - 47.993716).abs() < 1e-9);
        assert!((t.longitude - 0.185957).abs() < 1e-9);
        assert_eq!(t.taille_orbites(), 5);
        let b = r2.trouver("Terrain B").unwrap();
        assert_eq!(b.taille_orbites(), 0);
        assert!(r2.est_valide());
    }

    #[test]
    fn registre_vide_parse_sans_terrain_actif() {
        let r = RegistreTerrains::depuis_toml("").unwrap();
        assert!(r.terrain_actif.is_none());
        assert!(r.terrains.is_empty());
        assert!(r.entree_active().is_none());
    }

    #[test]
    fn edition_manuelle_champs_minimaux() {
        // Un humain qui édite le fichier à la main ne renseigne que le minimum.
        let toml = "terrain_actif = \"Nouveau terrain\"\n\n\
                     terrain.0.nom = \"Nouveau terrain\"\n\
                     terrain.0.latitude = 48.8566\n\
                     terrain.0.longitude = 2.3522\n";
        let r = RegistreTerrains::depuis_toml(toml).unwrap();
        let t = r.entree_active().expect("terrain actif attendu");
        assert_eq!(t.nom, "Nouveau terrain");
        assert_eq!(t.altitude_msl, 0.0, "valeur par défaut attendue");
        assert_eq!(t.taille_orbites(), 0);
        assert_eq!(t.age_secondes(), 0, "jamais sauvegardé sur site — âge nul");
    }

    #[test]
    fn terrain_incomplet_ignore_sans_bloquer_les_autres() {
        let toml = "terrain.0.nom = \"Incomplet\"\n\
                     terrain.0.latitude = 48.0\n\
                     \n\
                     terrain.1.nom = \"Complet\"\n\
                     terrain.1.latitude = 47.0\n\
                     terrain.1.longitude = 1.0\n";
        let r = RegistreTerrains::depuis_toml(toml).unwrap();
        assert!(r.trouver("Incomplet").is_none(), "longitude manquante — doit être ignoré");
        assert!(r.trouver("Complet").is_some());
    }

    #[test]
    fn definir_met_a_jour_une_entree_existante() {
        let mut r = RegistreTerrains::vide();
        r.definir("A", &AssistanceGps::nouvelle(1.0, 1.0, 0.0, vec![]));
        assert_eq!(r.terrains.len(), 1);
        r.definir("A", &AssistanceGps::nouvelle(2.0, 2.0, 0.0, vec![0x01]));
        assert_eq!(r.terrains.len(), 1, "même nom — mise à jour, pas doublon");
        let t = r.trouver("A").unwrap();
        assert!((t.latitude - 2.0).abs() < 1e-9);
        assert_eq!(t.taille_orbites(), 1);
    }
}
