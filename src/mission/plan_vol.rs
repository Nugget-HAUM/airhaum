// src/mission/plan_vol.rs
//! Plan de vol — segments et conditions de fin. Voir `doc/plan_de_vol.md`.
//!
//! Le plan de vol est le fichier édité par l'opérateur avant le lancement
//! du programme, chargé au démarrage et condition bloquante de l'item
//! checklist "Plan de vol chargé" (`doc/mission.md`). Logique pure — ne
//! sait ni avancer toute seule dans le temps (le temps écoulé dans le
//! segment courant est fourni par l'appelant, pas mesuré ici) ni router
//! les consignes vers les canaux `watch` du contrôle : c'est le rôle de
//! `taches/taches_mission.rs`.
//!
//! Portée de cette itération : uniquement les segments de phase Sol
//! (`attente`, `roulage_cap`) — voir `doc/plan_de_vol.md` pour les
//! décisions actées/différées (conditions combinées, actions annexes...).

use std::fs;
use std::path::Path;

use serde::Deserialize;

use crate::controle::{ConsigneDirectionSol, ConsigneGaz};
use crate::types::{Angle, ErreursAirHaum, Result};

/// Répertoire par défaut des fichiers de plan de vol, utilisé si la variable
/// d'environnement `AIRHAUM_PLANS_DIR` n'est pas définie — même principe que
/// `REPERTOIRE_LOGS_DEFAUT`/`AIRHAUM_LOGS_DIR` (`systeme/journalisation.rs`).
pub const REPERTOIRE_PLANS_DE_VOL_DEFAUT: &str = "/home/airhaum/plans_de_vol";

/// Résout le répertoire des plans de vol à utiliser : `AIRHAUM_PLANS_DIR`
/// si définie, sinon [`REPERTOIRE_PLANS_DE_VOL_DEFAUT`]. Permet de faire
/// tourner la sélection de plan sur un poste de développement où
/// `/home/airhaum` n'existe pas.
pub fn repertoire_plans_de_vol() -> String {
    std::env::var("AIRHAUM_PLANS_DIR").unwrap_or_else(|_| REPERTOIRE_PLANS_DE_VOL_DEFAUT.to_owned())
}

// ─────────────────────────────────────────────────────────────────────────────
// Types — miroir de l'esquisse `doc/plan_de_vol.md`
// ─────────────────────────────────────────────────────────────────────────────

/// Comportement d'un segment.
///
/// `#[serde(tag = "type")]` : le champ TOML `type = "..."` sélectionne le
/// variant, les autres champs du segment sont ceux du variant (aplatis
/// dans la table via `#[serde(flatten)]` sur `Segment::type_segment`).
#[derive(Debug, Clone, Copy, PartialEq, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum TypeSegment {
    /// Immobilisation, gaz coupés. Aucune nouvelle consigne de direction
    /// sol publiée : le contrôleur continue de réguler sur le dernier cap
    /// reçu, hérité du segment précédent.
    Attente,
    /// Roulage à cap constant.
    RoulageCap {
        cap_deg: f32,
        /// Passthrough direct, comme `ConsigneGaz` — voir `doc/controle.md`.
        gaz: f32,
    },
}

impl TypeSegment {
    /// Consigne de gaz produite par ce segment — toujours applicable.
    pub fn consigne_gaz(&self) -> ConsigneGaz {
        match self {
            TypeSegment::Attente => ConsigneGaz { gaz: 0.0 },
            TypeSegment::RoulageCap { gaz, .. } => ConsigneGaz { gaz: *gaz },
        }
    }

    /// Consigne de direction sol produite par ce segment, si applicable.
    /// `None` pour `Attente` : pas de nouvelle consigne à publier, voir
    /// la documentation du variant.
    pub fn consigne_direction_sol(&self) -> Option<ConsigneDirectionSol> {
        match self {
            TypeSegment::Attente => None,
            TypeSegment::RoulageCap { cap_deg, .. } => {
                Some(ConsigneDirectionSol { cap_cible: Angle::depuis_degres(*cap_deg) })
            }
        }
    }
}

/// Origine d'une commande opérateur explicite (voir `doc/plan_de_vol.md`,
/// reprend le pattern à trois origines déjà utilisé pour `Desarmer`). Seule
/// `Console` est câblée dans cette itération.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SourceCommande {
    Console,
    // Lora — anticipé (communications/commandes.rs, A FAIRE), non implémenté.
}

/// Condition de fin d'un segment.
///
/// `#[serde(untagged)]` : le variant est déduit du nom du champ présent
/// dans la table TOML (`duree_s` ou `commande_operateur`) — pas de tag
/// explicite, cohérent avec la table `fin = { duree_s = 5 }` de
/// `doc/plan_de_vol.md`. `Et`/`Ou` réservés mais non implémentés (voir
/// doc) — aucun cas d'usage identifié dans `doc/plan_essais.md`.
#[derive(Debug, Clone, Copy, PartialEq, Deserialize)]
#[serde(untagged)]
pub enum ConditionFin {
    Duree {
        #[serde(rename = "duree_s")]
        secs: f32,
    },
    CommandeOperateur {
        #[serde(rename = "commande_operateur")]
        source: SourceCommande,
    },
}

/// Un segment : comportement + condition de fin.
#[derive(Debug, Clone, Copy, PartialEq, Deserialize)]
pub struct Segment {
    #[serde(flatten)]
    pub type_segment: TypeSegment,
    pub fin: ConditionFin,
}

/// Événements externes pertinents pour l'évaluation d'une condition de fin,
/// fournis par l'appelant (`taches_mission.rs`) — ce module ne lit aucune
/// source d'événement lui-même.
#[derive(Debug, Clone, Copy, Default)]
pub struct EvenementsSegment {
    /// Vrai si une commande opérateur (`SourceCommande::Console`) a été
    /// reçue depuis le début du segment courant.
    pub commande_operateur_recue: bool,
}

// ─────────────────────────────────────────────────────────────────────────────
// Forme brute désérialisée — un simple `Vec<Segment>` sous la clé `segment`
// (array-of-tables `[[segment]]`).
// ─────────────────────────────────────────────────────────────────────────────

#[derive(Debug, Deserialize)]
struct PlanDeVolToml {
    #[serde(rename = "segment", default)]
    segments: Vec<Segment>,
}

// ─────────────────────────────────────────────────────────────────────────────
// PlanDeVol — état d'avancement
// ─────────────────────────────────────────────────────────────────────────────

/// Plan de vol chargé, avec sa progression dans la liste de segments.
#[derive(Debug, Clone)]
pub struct PlanDeVol {
    segments: Vec<Segment>,
    index_courant: usize,
}

impl PlanDeVol {
    /// Parse un plan de vol depuis un contenu TOML déjà en mémoire.
    ///
    /// # Erreurs
    ///
    /// `ConfigurationInvalide` si le TOML est malformé ou si le plan ne
    /// contient aucun segment — l'item checklist "Plan de vol chargé"
    /// (`doc/mission.md`) exige au moins un segment défini.
    pub fn depuis_toml(contenu: &str) -> Result<Self> {
        let brut: PlanDeVolToml = toml::from_str(contenu)
            .map_err(|e| ErreursAirHaum::ConfigurationInvalide(format!("plan de vol : {e}")))?;

        if brut.segments.is_empty() {
            return Err(ErreursAirHaum::ConfigurationInvalide(
                "plan de vol : aucun segment défini".to_string(),
            ));
        }

        Ok(Self { segments: brut.segments, index_courant: 0 })
    }

    /// Charge et parse un plan de vol depuis un fichier.
    pub fn charger(chemin: &Path) -> Result<Self> {
        let contenu = fs::read_to_string(chemin).map_err(|e| {
            ErreursAirHaum::FichierIntrouvable(format!("{} ({e})", chemin.display()))
        })?;
        Self::depuis_toml(&contenu)
    }

    /// Segment actif, ou `None` si le plan est terminé.
    pub fn segment_courant(&self) -> Option<&Segment> {
        self.segments.get(self.index_courant)
    }

    /// Vrai une fois tous les segments épuisés.
    pub fn est_termine(&self) -> bool {
        self.index_courant >= self.segments.len()
    }

    /// Passe au segment suivant. Sans effet si le plan est déjà terminé.
    pub fn avancer(&mut self) {
        if !self.est_termine() {
            self.index_courant += 1;
        }
    }

    /// Nombre total de segments du plan.
    pub fn nombre_segments(&self) -> usize {
        self.segments.len()
    }

    /// Index (0-based) du segment courant — pour affichage/journalisation
    /// (`taches_mission.rs`), pas utilisé par la logique de progression
    /// elle-même.
    pub fn index_courant(&self) -> usize {
        self.index_courant
    }

    /// Le segment courant a-t-il atteint sa condition de fin ?
    ///
    /// `temps_ecoule_dans_segment_s` : temps écoulé depuis le début du
    /// segment courant, mesuré par l'appelant (ce module ne connaît pas
    /// l'heure). Vrai si le plan est déjà terminé (rien à faire de plus).
    pub fn segment_termine(&self, temps_ecoule_dans_segment_s: f32, evenements: &EvenementsSegment) -> bool {
        match self.segment_courant() {
            None => true,
            Some(segment) => match &segment.fin {
                ConditionFin::Duree { secs } => temps_ecoule_dans_segment_s >= *secs,
                ConditionFin::CommandeOperateur { source } => match source {
                    SourceCommande::Console => evenements.commande_operateur_recue,
                },
            },
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const EXEMPLE_TOML: &str = r#"
[[segment]]
type = "attente"
fin = { duree_s = 5 }

[[segment]]
type = "roulage_cap"
cap_deg = 0.0
gaz = 0.2
fin = { duree_s = 20 }
"#;

    // ── Parsing ──────────────────────────────────────────────────────────

    #[test]
    fn parse_exemple_doc_plan_de_vol() {
        let plan = PlanDeVol::depuis_toml(EXEMPLE_TOML).unwrap();
        assert_eq!(plan.nombre_segments(), 2);

        let premier = plan.segment_courant().unwrap();
        assert_eq!(premier.type_segment, TypeSegment::Attente);
        assert_eq!(premier.fin, ConditionFin::Duree { secs: 5.0 });
    }

    #[test]
    fn parse_segment_roulage_cap_avec_ses_champs() {
        let plan = PlanDeVol::depuis_toml(EXEMPLE_TOML).unwrap();
        let deuxieme = &plan.segments[1];
        assert_eq!(
            deuxieme.type_segment,
            TypeSegment::RoulageCap { cap_deg: 0.0, gaz: 0.2 }
        );
        assert_eq!(deuxieme.fin, ConditionFin::Duree { secs: 20.0 });
    }

    #[test]
    fn parse_condition_fin_commande_operateur() {
        let toml = r#"
[[segment]]
type = "attente"
fin = { commande_operateur = "console" }
"#;
        let plan = PlanDeVol::depuis_toml(toml).unwrap();
        assert_eq!(
            plan.segment_courant().unwrap().fin,
            ConditionFin::CommandeOperateur { source: SourceCommande::Console }
        );
    }

    #[test]
    fn refuse_un_plan_sans_segment() {
        let erreur = PlanDeVol::depuis_toml("").unwrap_err();
        assert!(matches!(erreur, ErreursAirHaum::ConfigurationInvalide(_)));
    }

    #[test]
    fn refuse_un_toml_malforme() {
        let erreur = PlanDeVol::depuis_toml("ceci n'est pas du toml valide [[[").unwrap_err();
        assert!(matches!(erreur, ErreursAirHaum::ConfigurationInvalide(_)));
    }

    #[test]
    fn refuse_un_type_de_segment_inconnu() {
        let toml = r#"
[[segment]]
type = "decollage_immediat"
fin = { duree_s = 1 }
"#;
        assert!(PlanDeVol::depuis_toml(toml).is_err());
    }

    #[test]
    fn charger_fichier_absent_renvoie_fichier_introuvable() {
        let erreur = PlanDeVol::charger(Path::new("/chemin/inexistant/plan.toml")).unwrap_err();
        assert!(matches!(erreur, ErreursAirHaum::FichierIntrouvable(_)));
    }

    // ── Progression ──────────────────────────────────────────────────────

    #[test]
    fn progression_jusqu_a_la_fin_du_plan() {
        let mut plan = PlanDeVol::depuis_toml(EXEMPLE_TOML).unwrap();
        assert!(!plan.est_termine());

        plan.avancer();
        assert!(!plan.est_termine());
        assert_eq!(plan.segment_courant().unwrap().type_segment, TypeSegment::RoulageCap { cap_deg: 0.0, gaz: 0.2 });

        plan.avancer();
        assert!(plan.est_termine());
        assert!(plan.segment_courant().is_none());
    }

    #[test]
    fn avancer_apres_la_fin_est_sans_effet() {
        let mut plan = PlanDeVol::depuis_toml(EXEMPLE_TOML).unwrap();
        plan.avancer();
        plan.avancer();
        assert!(plan.est_termine());
        plan.avancer();
        plan.avancer();
        assert!(plan.est_termine());
    }

    // ── Condition de fin ─────────────────────────────────────────────────

    #[test]
    fn condition_duree_non_atteinte_puis_atteinte() {
        let plan = PlanDeVol::depuis_toml(EXEMPLE_TOML).unwrap();
        let evenements = EvenementsSegment::default();
        assert!(!plan.segment_termine(4.9, &evenements));
        assert!(plan.segment_termine(5.0, &evenements));
        assert!(plan.segment_termine(5.1, &evenements));
    }

    #[test]
    fn condition_commande_operateur_attend_l_evenement() {
        let toml = r#"
[[segment]]
type = "attente"
fin = { commande_operateur = "console" }
"#;
        let plan = PlanDeVol::depuis_toml(toml).unwrap();
        assert!(!plan.segment_termine(999.0, &EvenementsSegment::default()));
        assert!(plan.segment_termine(0.0, &EvenementsSegment { commande_operateur_recue: true }));
    }

    #[test]
    fn plan_deja_termine_est_toujours_considere_termine() {
        let mut plan = PlanDeVol::depuis_toml(EXEMPLE_TOML).unwrap();
        plan.avancer();
        plan.avancer();
        assert!(plan.segment_termine(0.0, &EvenementsSegment::default()));
    }

    // ── Consignes produites ──────────────────────────────────────────────

    #[test]
    fn consigne_attente_gaz_nul_et_pas_de_direction() {
        let seg = TypeSegment::Attente;
        assert_eq!(seg.consigne_gaz(), ConsigneGaz { gaz: 0.0 });
        assert!(seg.consigne_direction_sol().is_none());
    }

    #[test]
    fn consigne_roulage_cap_direction_et_gaz_attendus() {
        let seg = TypeSegment::RoulageCap { cap_deg: 90.0, gaz: 0.3 };
        assert_eq!(seg.consigne_gaz(), ConsigneGaz { gaz: 0.3 });
        let direction = seg.consigne_direction_sol().unwrap();
        assert!((direction.cap_cible.degres() - 90.0).abs() < 1e-3);
    }
}
