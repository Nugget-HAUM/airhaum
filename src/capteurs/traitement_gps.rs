// src/capteurs/traitement_gps.rs
//! Garde-fou GPS : rejette les fix physiquement incohérents avec le précédent.
//!
//! Le driver u-blox publie des [`DonneesGps`] déjà validées au niveau
//! protocole (checksum UBX, `type_fix`), mais rien ne garantit que la valeur
//! elle-même soit plausible : un glitch multipath ou un saut de fix peut
//! produire une position ou une vitesse aberrante tout en gardant un
//! `type_fix` valide. Sans filtre, [`EkfNavigation::corriger_gps`] applique
//! la correction sans remise en question — c'est le garde-fou manquant
//! documenté dans `doc/suivi_projet.md` ("Pas encore de garde-fou sur les
//! valeurs elles-mêmes : position aberrante, saut de vitesse").
//!
//! [`EkfNavigation::corriger_gps`]: crate::estimation::EkfNavigation::corriger_gps
//!
//! # Flux de données
//!
//! ```text
//! MesureGps (watch) ──▶ TraitementGps::traiter() ──▶ Option<DonneesGps>
//!                        (fix invalide, saut de        (None si rejeté —
//!                         position ou de vitesse         l'EKF n'est pas
//!                         rejetés)                        corrigé)
//! ```

use crate::taches::taches_gps::MesureGps;
use crate::types::{DonneesGps, RAYON_TERRE};

// ─────────────────────────────────────────────────────────────────────────────
// Constantes de plausibilité
// ─────────────────────────────────────────────────────────────────────────────

/// Vitesse horizontale maximale plausible entre deux fix consécutifs (m/s).
/// Marge large pour un avion RC (≈ 360 km/h) : au-delà, un déplacement de
/// position implique un glitch GPS plutôt qu'une manœuvre réelle.
const VITESSE_MAX_PLAUSIBLE_MS: f32 = 100.0;

/// Variation de vitesse maximale plausible entre deux fix consécutifs, en
/// accélération équivalente (m/s²) — ≈ 3g, marge large pour une manœuvre
/// violente.
const ACCELERATION_MAX_PLAUSIBLE_MS2: f32 = 30.0;

/// Distance minimale (m) en-dessous de laquelle le test de vitesse implicite
/// par position est ignoré. Évite les faux positifs dus au seul bruit de
/// position au point fixe : l'imprécision `hAcc` peut à elle seule donner
/// l'illusion d'un déplacement de quelques mètres entre deux fix à 10 Hz,
/// ce qui impliquerait une vitesse énorme sur un intervalle aussi court.
const DISTANCE_MIN_TEST_M: f32 = 3.0;

// ─────────────────────────────────────────────────────────────────────────────
// Préprocesseur
// ─────────────────────────────────────────────────────────────────────────────

/// Préprocesseur stateful des mesures GPS.
///
/// Conserve le dernier fix **accepté** (pas le dernier reçu) pour évaluer la
/// plausibilité du suivant : un fix rejeté n'avance pas cette référence, afin
/// qu'un unique glitch isolé ne fasse pas rejeter en cascade tous les fix
/// suivants (qui restent, eux, cohérents avec le dernier fix sain).
pub struct TraitementGps {
    dernier_accepte: Option<DonneesGps>,
}

impl TraitementGps {
    pub fn nouveau() -> Self {
        Self { dernier_accepte: None }
    }

    /// Filtre une mesure GPS brute.
    ///
    /// Retourne `None` si aucune donnée n'est disponible, si le fix est
    /// invalide (`type_fix` < 2D), ou si la position/vitesse implique un
    /// déplacement physiquement incohérent avec le dernier fix accepté.
    pub fn traiter(&mut self, mesure: &MesureGps) -> Option<DonneesGps> {
        let donnees = mesure.donnees?;
        if !donnees.type_fix.est_valide() {
            return None;
        }

        if let Some(precedent) = self.dernier_accepte {
            if !plausible(&precedent, &donnees) {
                log::warn!(target: "gps",
                    "Fix GPS rejeté par le garde-fou — saut de position/vitesse implausible");
                return None;
            }
        }

        self.dernier_accepte = Some(donnees);
        Some(donnees)
    }
}

/// Vrai si `actuel` est un déplacement physiquement plausible depuis `precedent`.
fn plausible(precedent: &DonneesGps, actuel: &DonneesGps) -> bool {
    let dt = actuel.horodatage.delta_secondes(precedent.horodatage);
    if dt <= 0.0 {
        // Pas de référence temporelle exploitable (horloge non monotone
        // entre les deux mesures) — on n'entrave pas, faute de pouvoir juger.
        return true;
    }

    let distance = distance_horizontale_m(precedent, actuel);
    if distance > DISTANCE_MIN_TEST_M {
        let vitesse_impliquee = distance / dt;
        if vitesse_impliquee > VITESSE_MAX_PLAUSIBLE_MS {
            return false;
        }
    }

    let dv_nord = actuel.vel_nord - precedent.vel_nord;
    let dv_est  = actuel.vel_est  - precedent.vel_est;
    let acceleration = (dv_nord * dv_nord + dv_est * dv_est).sqrt() / dt;
    if acceleration > ACCELERATION_MAX_PLAUSIBLE_MS2 {
        return false;
    }

    true
}

/// Distance horizontale approximative entre deux positions GPS, en mètres.
///
/// Approximation plane (équirectangulaire) — suffisante ici : les deux fix
/// comparés sont consécutifs (dixièmes de seconde d'écart), donc séparés
/// d'au plus quelques centaines de mètres même dans le pire cas plausible.
fn distance_horizontale_m(a: &DonneesGps, b: &DonneesGps) -> f32 {
    let lat_moy_rad = ((a.latitude + b.latitude) / 2.0).to_radians();
    let dlat = (b.latitude - a.latitude).to_radians();
    let dlon = (b.longitude - a.longitude).to_radians();
    let dx = dlon * lat_moy_rad.cos() * RAYON_TERRE as f64;
    let dy = dlat * RAYON_TERRE as f64;
    (dx * dx + dy * dy).sqrt() as f32
}

// ─────────────────────────────────────────────────────────────────────────────
// Tests
// ─────────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::{Horodatage, TypeFixGps};

    /// Fix GPS de test à une position/vitesse donnée, à l'horodatage `micros`.
    fn fix(micros: u64, lat: f64, lon: f64, vel_nord: f32, vel_est: f32, type_fix: TypeFixGps) -> MesureGps {
        MesureGps {
            donnees: Some(DonneesGps {
                horodatage:        Horodatage::depuis_micros(micros),
                latitude:          lat,
                longitude:         lon,
                altitude_msl:      50.0,
                vitesse_sol:       (vel_nord * vel_nord + vel_est * vel_est).sqrt(),
                cap:               0.0,
                vel_nord,
                vel_est,
                vel_bas:           0.0,
                precision_h:       2.0,
                precision_v:       3.0,
                nombre_satellites: 8,
                type_fix,
                temps_premier_fix: None,
            }),
            valide: type_fix.est_valide(),
            erreurs_consecutives: 0,
        }
    }

    fn mesure_invalide() -> MesureGps {
        MesureGps { donnees: None, valide: false, erreurs_consecutives: 1 }
    }

    // Paris (48.8566°N, 2.3522°E) — position de référence pour les tests.
    const LAT0: f64 = 48.8566;
    const LON0: f64 = 2.3522;

    #[test]
    fn premier_fix_valide_toujours_accepte() {
        let mut proc = TraitementGps::nouveau();
        let r = proc.traiter(&fix(0, LAT0, LON0, 0.0, 0.0, TypeFixGps::Fix3D));
        assert!(r.is_some(), "aucun historique — rien à comparer, doit être accepté");
    }

    #[test]
    fn fix_sans_donnees_rejete() {
        let mut proc = TraitementGps::nouveau();
        assert!(proc.traiter(&mesure_invalide()).is_none());
    }

    #[test]
    fn fix_type_invalide_rejete() {
        let mut proc = TraitementGps::nouveau();
        let r = proc.traiter(&fix(0, LAT0, LON0, 0.0, 0.0, TypeFixGps::Aucun));
        assert!(r.is_none());
    }

    #[test]
    fn deplacement_realiste_accepte() {
        let mut proc = TraitementGps::nouveau();
        proc.traiter(&fix(0, LAT0, LON0, 15.0, 0.0, TypeFixGps::Fix3D));
        // 1.5 m parcourus en 100 ms ≈ 15 m/s vers le nord — cohérent avec vel_nord.
        let dlat = 1.5 / (RAYON_TERRE as f64) * (180.0 / std::f64::consts::PI);
        let r = proc.traiter(&fix(100_000, LAT0 + dlat, LON0, 15.0, 0.0, TypeFixGps::Fix3D));
        assert!(r.is_some(), "déplacement cohérent avec la vitesse rapportée — doit être accepté");
    }

    #[test]
    fn saut_de_position_rejete() {
        let mut proc = TraitementGps::nouveau();
        proc.traiter(&fix(0, LAT0, LON0, 0.0, 0.0, TypeFixGps::Fix3D));
        // Saut de ~1° de latitude (~111 km) en 100 ms : glitch GPS classique.
        let r = proc.traiter(&fix(100_000, LAT0 + 1.0, LON0, 0.0, 0.0, TypeFixGps::Fix3D));
        assert!(r.is_none(), "saut de position implausible — doit être rejeté");
    }

    #[test]
    fn saut_de_vitesse_rejete() {
        let mut proc = TraitementGps::nouveau();
        proc.traiter(&fix(0, LAT0, LON0, 10.0, 0.0, TypeFixGps::Fix3D));
        // vel_nord 10 → 200 m/s en 100 ms : accélération implausible, même
        // sans déplacement de position notable.
        let r = proc.traiter(&fix(100_000, LAT0, LON0, 200.0, 0.0, TypeFixGps::Fix3D));
        assert!(r.is_none(), "saut de vitesse implausible — doit être rejeté");
    }

    #[test]
    fn petit_bruit_de_position_au_point_fixe_accepte() {
        let mut proc = TraitementGps::nouveau();
        proc.traiter(&fix(0, LAT0, LON0, 0.0, 0.0, TypeFixGps::Fix3D));
        // ~1 m de bruit de position (bien en-dessous de DISTANCE_MIN_TEST_M),
        // typique du bruit hAcc au point fixe — ne doit pas déclencher le
        // test de vitesse implicite.
        let dlat = 1.0 / (RAYON_TERRE as f64) * (180.0 / std::f64::consts::PI);
        let r = proc.traiter(&fix(10_000, LAT0 + dlat, LON0, 0.0, 0.0, TypeFixGps::Fix3D));
        assert!(r.is_some(), "bruit de position sous le seuil — doit être accepté");
    }

    #[test]
    fn glitch_isole_ne_bloque_pas_les_fix_suivants() {
        let mut proc = TraitementGps::nouveau();
        proc.traiter(&fix(0, LAT0, LON0, 0.0, 0.0, TypeFixGps::Fix3D));
        let glitch = proc.traiter(&fix(100_000, LAT0 + 1.0, LON0, 0.0, 0.0, TypeFixGps::Fix3D));
        assert!(glitch.is_none());

        // Le fix suivant, cohérent avec le dernier fix *accepté* (t=0, pas
        // le glitch de t=100ms), doit repasser.
        let r = proc.traiter(&fix(200_000, LAT0, LON0, 0.0, 0.0, TypeFixGps::Fix3D));
        assert!(r.is_some(), "le glitch rejeté ne doit pas devenir la nouvelle référence");
    }
}
