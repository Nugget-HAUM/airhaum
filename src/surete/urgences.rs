// src/surete/urgences.rs
//! Machine à états de la sécurité (MAÉ sécurité).
//!
//! Orthogonale et prioritaire sur la MAÉ vol (voir doc/surete.md). Combine
//! la santé des capteurs (`sante.rs`) et le respect des limites de vol
//! (`limites.rs`) en un `EtatSecurite` unique.
//!
//! Les sources pas encore câblées (liaison LoRa, batterie) sont des
//! `Option` dans `EntreesSecurite` : `None` signifie "non surveillé", pas
//! "nominal" — elles n'entrent simplement pas dans l'évaluation tant que
//! `communications/` et `systeme/energie.rs` n'existent pas.
//!
//! `arret_urgence_sol` est câblé dès aujourd'hui (arrêt console, voir
//! `taches/taches_surete.rs`) — la LoRa n'en sera qu'une seconde source une
//! fois implémentée, pas un préalable.
//!
//! Implémentation de la boucle : `taches/taches_surete.rs` (à venir).

use crate::surete::limites::EtatLimites;
use crate::surete::sante::EtatSante;
use crate::surete::EtatSecurite;

/// Seuil de tension batterie critique (V) — déclenche `AtterrissageUrgence`.
///
/// Valeur provisoire, à ajuster une fois `systeme/energie.rs` implémenté et
/// la chimie/capacité de la batterie connues.
const TENSION_BATTERIE_CRITIQUE_V: f32 = 6.6; // ex. 2S LiPo, ~3.3 V/cellule

/// Nombre de cycles sains consécutifs requis pour lever un verrou critique.
/// Une seule bonne lecture ne suffit pas — évite qu'un capteur limite fasse
/// osciller la MAÉ sécurité en pleine manœuvre d'urgence.
const CYCLES_RETABLISSEMENT_CRITIQUE: u32 = 10;

/// Rang à partir duquel un incident est considéré assez sérieux pour
/// justifier d'écourter le vol, même après rétablissement (voir
/// `MaeSecurite::deja_degrade_ce_vol`).
const RANG_ALERTE_MAJEURE: u8 = 2;

/// Entrées consommées à chaque cycle pour évaluer l'état sécurité.
pub struct EntreesSecurite<'a> {
    /// Pire état de santé parmi tous les capteurs/liaisons (voir `sante::pire_etat`).
    pub sante_capteurs: &'a EtatSante,
    /// État vis-à-vis de la géofence / enveloppe de vol.
    pub limites: &'a EtatLimites,
    /// Vrai si la liaison LoRa sol est perdue depuis plus que le délai configuré.
    pub lora_perdue: Option<bool>,
    /// Vrai si une commande d'arrêt d'urgence a été reçue depuis le sol
    /// (console opérateur aujourd'hui, LoRa à terme — même signal, sources
    /// multiples possibles).
    pub arret_urgence_sol: Option<bool>,
    /// Vrai si le Nano a basculé en mode manuel (le pilote RC a pris la
    /// main). L'autopilote n'a alors plus lieu d'agir : le Nano court-
    /// circuite déjà les consignes Pi au niveau matériel, indépendamment du
    /// logiciel — ce signal sert à garder la MAÉ vol et la télémétrie
    /// cohérentes avec cette prise de main, pas à sécuriser le vol lui-même.
    pub mode_manuel_rc: Option<bool>,
    /// Tension batterie (V).
    pub tension_batterie_v: Option<f32>,
}

/// Rang de criticité — reflète l'ordre documenté sur `EtatSecurite`.
fn rang(etat: &EtatSecurite) -> u8 {
    match etat {
        EtatSecurite::Normal => 0,
        EtatSecurite::AlerteMineure { .. } => 1,
        EtatSecurite::AlerteMajeure { .. } => 2,
        EtatSecurite::FailSafe => 3,
        EtatSecurite::AtterrissageUrgence { .. } => 4,
        EtatSecurite::ArretUrgence { .. } => 5,
    }
}

/// Évalue l'état sécurité à partir des seules entrées du cycle courant, sans
/// mémoire. Ordre de priorité décroissant, reflétant le tableau des
/// déclencheurs de `doc/surete.md`.
fn evaluer(entrees: &EntreesSecurite) -> EtatSecurite {
    if entrees.arret_urgence_sol == Some(true) {
        return EtatSecurite::ArretUrgence {
            raison: "Commande d'arrêt d'urgence reçue du sol".into(),
        };
    }

    if entrees.mode_manuel_rc == Some(true) {
        return EtatSecurite::ArretUrgence {
            raison: "Nano en mode manuel — pilote RC a pris la main".into(),
        };
    }

    if let Some(tension) = entrees.tension_batterie_v {
        if tension < TENSION_BATTERIE_CRITIQUE_V {
            return EtatSecurite::AtterrissageUrgence {
                raison: format!("Batterie critique : {tension:.2} V"),
            };
        }
    }

    if let EtatLimites::Depassee { raison } = entrees.limites {
        return EtatSecurite::AtterrissageUrgence { raison: raison.clone() };
    }

    if entrees.lora_perdue == Some(true) {
        return EtatSecurite::FailSafe;
    }

    match entrees.sante_capteurs {
        EtatSante::HorsService { raison } => {
            return EtatSecurite::AlerteMajeure { raison: raison.clone() };
        }
        EtatSante::Degrade { raison } => {
            return EtatSecurite::AlerteMineure { raison: raison.clone() };
        }
        EtatSante::Nominal => {}
    }

    if let EtatLimites::Approche { raison } = entrees.limites {
        return EtatSecurite::AlerteMineure { raison: raison.clone() };
    }

    EtatSecurite::Normal
}

/// MAÉ sécurité avec état.
pub struct MaeSecurite {
    etat: EtatSecurite,
    cycles_bons_consecutifs: u32,
    /// Vrai dès qu'un `AlerteMajeure` ou pire a été atteint pendant ce vol.
    /// Plafonne alors tout rétablissement à `AlerteMineure`. Effacé
    /// uniquement par `reinitialiser()`.
    deja_degrade_ce_vol: bool,
}

impl MaeSecurite {
    pub fn nouveau() -> Self {
        Self {
            etat: EtatSecurite::Normal,
            cycles_bons_consecutifs: 0,
            deja_degrade_ce_vol: false,
        }
    }

    pub fn etat(&self) -> &EtatSecurite {
        &self.etat
    }

    /// Réévalue l'état à partir des entrées du cycle courant.
    pub fn actualiser(&mut self, entrees: &EntreesSecurite) -> &EtatSecurite {
        let nouvel_etat = evaluer(entrees);

        if rang(&nouvel_etat) >= RANG_ALERTE_MAJEURE {
            self.deja_degrade_ce_vol = true;
        }

        if self.etat.est_critique() {
            if rang(&nouvel_etat) > rang(&self.etat) {
                // Aggravation : jamais retardée.
                self.etat = nouvel_etat;
                self.cycles_bons_consecutifs = 0;
            } else if !nouvel_etat.est_critique() {
                // Amélioration réelle (retombé sous le seuil critique) :
                // le cycle compte pour l'hystérésis de rétablissement.
                self.cycles_bons_consecutifs += 1;
                if self.cycles_bons_consecutifs >= CYCLES_RETABLISSEMENT_CRITIQUE {
                    self.etat = self.appliquer_plafond(EtatSecurite::Normal);
                    self.cycles_bons_consecutifs = 0;
                }
            } else {
                // Toujours critique (même sévérité, ou repli partiel mais
                // encore critique) : aucun progrès réel, ne pas avancer le
                // compteur — sinon une condition critique qui persiste sans
                // jamais s'aggraver finirait par se lever seule.
                self.cycles_bons_consecutifs = 0;
            }
        } else {
            self.etat = self.appliquer_plafond(nouvel_etat);
            self.cycles_bons_consecutifs = 0;
        }

        &self.etat
    }

    /// Plafonne `Normal` à `AlerteMineure` si un incident sérieux a marqué
    /// ce vol — appliqué à chaque cycle, pas seulement au rétablissement,
    /// pour qu'aucun chemin ne puisse réafficher `Normal` par la suite.
    fn appliquer_plafond(&self, etat: EtatSecurite) -> EtatSecurite {
        if self.deja_degrade_ce_vol && matches!(etat, EtatSecurite::Normal) {
            EtatSecurite::AlerteMineure {
                raison: "Rétabli après incident — vol à écourter".into(),
            }
        } else {
            etat
        }
    }

    /// Réinitialisation complète (prochain armement) : efface l'état, le
    /// verrou et la mémoire d'incident.
    pub fn reinitialiser(&mut self) {
        self.etat = EtatSecurite::Normal;
        self.cycles_bons_consecutifs = 0;
        self.deja_degrade_ce_vol = false;
    }
}

impl Default for MaeSecurite {
    fn default() -> Self {
        Self::nouveau()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn entrees_nominales<'a>(sante: &'a EtatSante, limites: &'a EtatLimites) -> EntreesSecurite<'a> {
        EntreesSecurite {
            sante_capteurs: sante,
            limites,
            lora_perdue: None,
            arret_urgence_sol: None,
            mode_manuel_rc: None,
            tension_batterie_v: None,
        }
    }

    #[test]
    fn test_normal() {
        let sante = EtatSante::Nominal;
        let limites = EtatLimites::DansLesLimites;
        let mut mae = MaeSecurite::nouveau();
        assert_eq!(*mae.actualiser(&entrees_nominales(&sante, &limites)), EtatSecurite::Normal);
    }

    #[test]
    fn test_sante_degradee_alerte_mineure() {
        let sante = EtatSante::Degrade { raison: "baro".into() };
        let limites = EtatLimites::DansLesLimites;
        let mut mae = MaeSecurite::nouveau();
        assert!(matches!(mae.actualiser(&entrees_nominales(&sante, &limites)), EtatSecurite::AlerteMineure { .. }));
    }

    #[test]
    fn test_sante_hors_service_alerte_majeure() {
        let sante = EtatSante::HorsService { raison: "imu".into() };
        let limites = EtatLimites::DansLesLimites;
        let mut mae = MaeSecurite::nouveau();
        assert!(matches!(mae.actualiser(&entrees_nominales(&sante, &limites)), EtatSecurite::AlerteMajeure { .. }));
    }

    #[test]
    fn test_geofence_depassee_atterrissage_urgence() {
        let sante = EtatSante::Nominal;
        let limites = EtatLimites::Depassee { raison: "geofence".into() };
        let mut mae = MaeSecurite::nouveau();
        assert!(matches!(mae.actualiser(&entrees_nominales(&sante, &limites)), EtatSecurite::AtterrissageUrgence { .. }));
    }

    #[test]
    fn test_batterie_critique() {
        let sante = EtatSante::Nominal;
        let limites = EtatLimites::DansLesLimites;
        let mut entrees = entrees_nominales(&sante, &limites);
        entrees.tension_batterie_v = Some(6.0);
        let mut mae = MaeSecurite::nouveau();
        assert!(matches!(mae.actualiser(&entrees), EtatSecurite::AtterrissageUrgence { .. }));
    }

    #[test]
    fn test_arret_urgence_priorite_absolue() {
        let sante = EtatSante::HorsService { raison: "tout".into() };
        let limites = EtatLimites::Depassee { raison: "geofence".into() };
        let mut entrees = entrees_nominales(&sante, &limites);
        entrees.arret_urgence_sol = Some(true);
        let mut mae = MaeSecurite::nouveau();
        assert!(matches!(mae.actualiser(&entrees), EtatSecurite::ArretUrgence { .. }));
    }

    #[test]
    fn test_mode_manuel_rc_priorite_absolue() {
        let sante = EtatSante::Nominal;
        let limites = EtatLimites::DansLesLimites;
        let mut entrees = entrees_nominales(&sante, &limites);
        entrees.mode_manuel_rc = Some(true);
        let mut mae = MaeSecurite::nouveau();
        assert!(matches!(mae.actualiser(&entrees), EtatSecurite::ArretUrgence { .. }));
    }

    #[test]
    fn test_alerte_non_critique_se_resorbe_naturellement() {
        let sante_mauvaise = EtatSante::Degrade { raison: "baro".into() };
        let sante_bonne = EtatSante::Nominal;
        let limites = EtatLimites::DansLesLimites;
        let mut mae = MaeSecurite::nouveau();

        mae.actualiser(&entrees_nominales(&sante_mauvaise, &limites));
        assert!(matches!(mae.etat(), EtatSecurite::AlerteMineure { .. }));

        mae.actualiser(&entrees_nominales(&sante_bonne, &limites));
        assert_eq!(*mae.etat(), EtatSecurite::Normal);
    }

    #[test]
    fn test_etat_critique_hysteresis_avant_retablissement() {
        let sante = EtatSante::Nominal;
        let limites_depassee = EtatLimites::Depassee { raison: "geofence".into() };
        let limites_ok = EtatLimites::DansLesLimites;
        let mut mae = MaeSecurite::nouveau();

        mae.actualiser(&entrees_nominales(&sante, &limites_depassee));
        assert!(mae.etat().est_critique());

        // La géofence redevient respectée, mais un seul cycle sain ne suffit pas.
        mae.actualiser(&entrees_nominales(&sante, &limites_ok));
        assert!(mae.etat().est_critique());

        // Après suffisamment de cycles sains, le verrou se lève.
        for _ in 0..CYCLES_RETABLISSEMENT_CRITIQUE {
            mae.actualiser(&entrees_nominales(&sante, &limites_ok));
        }
        assert!(!mae.etat().est_critique());
    }

    #[test]
    fn test_retablissement_plafonne_apres_incident() {
        let sante = EtatSante::Nominal;
        let limites_depassee = EtatLimites::Depassee { raison: "geofence".into() };
        let limites_ok = EtatLimites::DansLesLimites;
        let mut mae = MaeSecurite::nouveau();

        mae.actualiser(&entrees_nominales(&sante, &limites_depassee));
        for _ in 0..=CYCLES_RETABLISSEMENT_CRITIQUE {
            mae.actualiser(&entrees_nominales(&sante, &limites_ok));
        }

        // Plus jamais Normal ce vol-ci : l'incident laisse une trace.
        assert!(matches!(mae.etat(), EtatSecurite::AlerteMineure { .. }));
        assert_ne!(*mae.etat(), EtatSecurite::Normal);
    }

    #[test]
    fn test_escalade_autorisee_pendant_verrouillage() {
        let sante = EtatSante::Nominal;
        let limites_depassee = EtatLimites::Depassee { raison: "geofence".into() };
        let mut mae = MaeSecurite::nouveau();
        mae.actualiser(&entrees_nominales(&sante, &limites_depassee));
        assert!(matches!(mae.etat(), EtatSecurite::AtterrissageUrgence { .. }));

        let mut entrees = entrees_nominales(&sante, &limites_depassee);
        entrees.arret_urgence_sol = Some(true);
        mae.actualiser(&entrees);
        assert!(matches!(mae.etat(), EtatSecurite::ArretUrgence { .. }));
    }

    #[test]
    fn test_condition_critique_persistante_ne_se_leve_pas_seule() {
        // Régression : une condition critique qui persiste sans jamais
        // s'améliorer ne doit jamais se lever d'elle-même, même après un
        // grand nombre de cycles.
        let sante = EtatSante::Nominal;
        let limites_depassee = EtatLimites::Depassee { raison: "geofence".into() };
        let mut mae = MaeSecurite::nouveau();

        for _ in 0..(CYCLES_RETABLISSEMENT_CRITIQUE * 3) {
            mae.actualiser(&entrees_nominales(&sante, &limites_depassee));
        }
        assert!(mae.etat().est_critique());
    }

    #[test]
    fn test_reinitialisation_efface_lincident_et_revient_a_normal() {
        let sante = EtatSante::Nominal;
        let limites_depassee = EtatLimites::Depassee { raison: "geofence".into() };
        let limites_ok = EtatLimites::DansLesLimites;
        let mut mae = MaeSecurite::nouveau();
        mae.actualiser(&entrees_nominales(&sante, &limites_depassee));
        assert!(mae.etat().est_critique());

        mae.reinitialiser();
        assert_eq!(*mae.etat(), EtatSecurite::Normal);

        // Après réinitialisation, un cycle nominal donne bien Normal (pas plafonné).
        mae.actualiser(&entrees_nominales(&sante, &limites_ok));
        assert_eq!(*mae.etat(), EtatSecurite::Normal);
    }
}
