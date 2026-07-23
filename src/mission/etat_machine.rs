// src/mission/etat_machine.rs
//! MAÉ vol — états, contexte, commandes et logique de transition.
//!
//! Ce module est de la logique pure : aucune dépendance Tokio, aucun I/O.
//! La tâche associée (`taches/taches_mission.rs`, à venir) possède le
//! `watch::Sender<EtatVol>` et appelle la MAÉ à chaque itération.
//!
//! # Deux chemins de transition
//!
//! | Méthode              | Rôle                                                  |
//! |----------------------|-------------------------------------------------------|
//! | `tick(&ContexteVol)` | Seuils capteurs/estimation détectés par polling       |
//! | `traiter_commande`   | Commandes externes (sol via LoRa) ou internes système |
//! | `forcer_urgence`     | Priorité absolue : forcé par la MAÉ sécurité          |
//!
//! # Priorité sécurité
//!
//! La tâche mission appelle `forcer_urgence` en **premier** à chaque itération,
//! avant `tick` et `traiter_commande`. La MAÉ vol ne commande jamais la MAÉ sécurité.
//! Le module comportement (`atterrissage.rs`, à venir) lira `EtatSecurite` pour
//! adapter son exécution (ex. couper le moteur en `Approche` si `ArretUrgence`).

use std::fmt;
use crate::surete::EtatSecurite;

// ─────────────────────────────────────────────────────────────────────────────
// Seuils de transition (à migrer vers config/ quand la couche config existera)
// ─────────────────────────────────────────────────────────────────────────────

/// Vitesse sol minimale pour la rotation au décollage (m/s).
const VITESSE_ROTATION_MS: f32 = 15.0;

/// Hauteur sol maximale pour déclencher le flare (mm, VL53L0X).
/// Correspond à ~1,5 m, dans la portée utile du capteur (~2 m).
const HAUTEUR_FLARE_MM: u16 = 1_500;


// ─────────────────────────────────────────────────────────────────────────────
// États
// ─────────────────────────────────────────────────────────────────────────────

/// Ensemble des états de la MAÉ vol.
///
/// Regroupés par phase ; un seul état est actif à la fois.
/// Les urgences ne sont pas des états distincts : la MAÉ sécurité force
/// un état vol existant, et le module comportement adapte son exécution.
#[derive(Debug, Clone, PartialEq, Default)]
pub enum EtatVol {

    // --- Pré-vol ---

    /// Démarrage système, initialisation des tâches.
    #[default]
    Boot,
    /// Démarrage des drivers, configuration des capteurs.
    Initialisation,
    /// Vérification fonctionnelle : tous capteurs opérationnels, calibrations valides.
    AutoTest,
    /// Système prêt, en attente de la commande d'armement sol.
    AttenteArmement,

    // --- Décollage ---

    /// Activation des actionneurs, vérification finale pré-vol.
    Armement,
    /// Déplacement sol à faible vitesse (mise en position en bout de piste).
    RoulageAuSol,
    /// Pleine puissance, accélération sur piste.
    CourseDecollage,
    /// Avion en l'air, montée vers l'altitude de croisière.
    Montee,

    // --- Croisière ---

    /// Cap constant, altitude constante, sans waypoint actif.
    VolLigneDroite,
    /// Suivi de waypoints, corrections de cap automatiques.
    Navigation,
    /// Loiter : maintien de position en orbite (attente atterrissage, perte liaison…).
    AttenteEnCercle,

    // --- Atterrissage ---

    /// Descente vers l'axe d'atterrissage.
    /// En urgence, le module `atterrissage.rs` lira `EtatSecurite` pour adapter
    /// son comportement (ex. moteur coupé si `ArretUrgence`).
    Approche,
    /// Descente stabilisée sur l'axe de piste.
    Final,
    /// Flare : réduction puissance, assiette cabrée.
    /// Déclenché par le VL53L0X (hauteur sol < `HAUTEUR_FLARE_MM`). Phase la plus critique.
    Arrondi,

    // --- Fin de mission ---

    /// Désactivation des actionneurs.
    Desarmement,
    /// Clôture et flush de la boîte noire.
    SauvegardeDonnees,
}

impl EtatVol {
    /// Vrai si l'appareil est en vol (transitions d'urgence applicables).
    pub fn est_en_vol(&self) -> bool {
        matches!(
            self,
            EtatVol::Montee
                | EtatVol::VolLigneDroite
                | EtatVol::Navigation
                | EtatVol::AttenteEnCercle
                | EtatVol::Approche
                | EtatVol::Final
                | EtatVol::Arrondi
        )
    }
}

impl fmt::Display for EtatVol {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let nom = match self {
            EtatVol::Boot              => "Boot",
            EtatVol::Initialisation    => "Initialisation",
            EtatVol::AutoTest          => "AutoTest",
            EtatVol::AttenteArmement   => "AttenteArmement",
            EtatVol::Armement          => "Armement",
            EtatVol::RoulageAuSol      => "RoulageAuSol",
            EtatVol::CourseDecollage   => "CourseDecollage",
            EtatVol::Montee            => "Montée",
            EtatVol::VolLigneDroite    => "VolLigneDroite",
            EtatVol::Navigation        => "Navigation",
            EtatVol::AttenteEnCercle   => "AttenteEnCercle",
            EtatVol::Approche          => "Approche",
            EtatVol::Final             => "Final",
            EtatVol::Arrondi           => "Arrondi",
            EtatVol::Desarmement       => "Désarmement",
            EtatVol::SauvegardeDonnees => "SauvegardeDonnées",
        };
        write!(f, "{}", nom)
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Contexte de vol (entrée du tick)
// ─────────────────────────────────────────────────────────────────────────────

/// Snapshot de l'état du système fourni à chaque appel de `tick`.
///
/// Rempli par la tâche mission depuis les couches estimation et capteurs.
/// Les champs sont `Option` car certaines sources peuvent être indisponibles
/// (GPS perdu, capteur dégradé). Une transition conditionnelle ne se déclenche
/// que si la valeur est `Some` et dépasse le seuil.
///
/// Ce struct grandira avec les couches estimation et contrôle.
pub struct ContexteVol {
    /// Vitesse sol estimée (m/s). Nécessaire pour la rotation et l'arrêt.
    pub vitesse_sol_ms: Option<f32>,
    /// Altitude barométrique courante (m).
    pub altitude_m: Option<f32>,
    /// Altitude de croisière cible (m), issue de la configuration de mission.
    pub altitude_cible_m: f32,
    /// Hauteur sol mesurée par le VL53L0X (mm). `None` si hors portée (~2 m).
    pub hauteur_sol_mm: Option<u16>,
    /// Le thread d'estimation est actif et a produit au moins une attitude valide.
    /// Requis pour la transition `AutoTest → AttenteArmement`.
    pub estimation_prete: bool,
}

// ─────────────────────────────────────────────────────────────────────────────
// Commandes externes et internes
// ─────────────────────────────────────────────────────────────────────────────

/// Commandes déclenchant des transitions dans la MAÉ vol.
///
/// Deux origines possibles :
/// - **Sol** : reçues via LoRa par la couche communication (à venir)
/// - **Système** : émises par un module interne (contrôleur d'approche, etc.)
#[derive(Debug, Clone)]
pub enum CommandeVol {

    // --- Sol (via LoRa) ---

    /// Demande d'armement opérateur (AttenteArmement → Armement).
    Armer,
    /// Demande de désarmement (RoulageAuSol → Desarmement).
    /// Trois origines possibles :
    /// - Navigation sol : dernier point atteint, moteur coupé (automatique)
    /// - Opérateur : via LoRa
    /// - Télécommande RC : signal capté par l'Arduino Nano (arrêt d'urgence principal)
    Desarmer,
    /// Demande d'atterrissage opérateur (croisière → Approche).
    Atterrir,
    /// Demande de loiter opérateur (croisière → AttenteEnCercle).
    Loiter,
    /// Reprise de navigation opérateur (AttenteEnCercle → Navigation).
    ReprendreNavigation,
    /// Activation navigation waypoints (VolLigneDroite → Navigation).
    ActiverNavigation,
    /// Désactivation navigation (Navigation → VolLigneDroite).
    DesactiverNavigation,

    // --- Système (émises par modules internes) ---

    /// Drivers initialisés, démarrage de l'autotest (Boot → Initialisation).
    DriversInitialises,
    /// Configuration drivers terminée (Initialisation → AutoTest).
    ConfigurationTerminee,
    /// Reprise rapide détectée : capteurs déjà opérationnels depuis le démarrage
    /// précédent, autotest implicitement passé (Initialisation → AttenteArmement).
    RepriseRapide,
    /// Vérifications pré-vol passées, actionneurs opérationnels (Armement → RoulageAuSol).
    ArmementValide,
    /// Position en bout de piste atteinte (RoulageAuSol → CourseDecollage).
    PositionDecollageAtteinte,
    /// Approche stabilisée sur l'axe, émise par le contrôleur d'approche (Approche → Final).
    ApprocheStabilisee,
    /// Contact sol détecté (Arrondi → RoulageAuSol).
    ContactSol,
    /// Actionneurs confirmés désactivés (Desarmement → SauvegardeDonnees).
    ActionneursDesarmes,
}

// ─────────────────────────────────────────────────────────────────────────────
// Machine à états
// ─────────────────────────────────────────────────────────────────────────────

/// MAÉ vol — encapsule l'état courant et la logique de transition.
pub struct MachineEtatVol {
    etat: EtatVol,
}

impl MachineEtatVol {
    pub fn nouveau() -> Self {
        Self { etat: EtatVol::Boot }
    }

    pub fn etat(&self) -> &EtatVol {
        &self.etat
    }

    /// Vérifie les seuils capteurs/estimation et effectue la transition si nécessaire.
    ///
    /// Appelé à chaque itération de la tâche mission (5–10 Hz), **après**
    /// `forcer_urgence` et **avant** `traiter_commande`.
    /// Retourne `Some(nouvel_etat)` si une transition a eu lieu.
    pub fn tick(&mut self, ctx: &ContexteVol) -> Option<EtatVol> {
        let nouvel_etat = match &self.etat {

            EtatVol::AutoTest => {
                ctx.estimation_prete.then_some(EtatVol::AttenteArmement)?
            }

            EtatVol::CourseDecollage => {
                let v = ctx.vitesse_sol_ms?;
                (v >= VITESSE_ROTATION_MS).then_some(EtatVol::Montee)?
            }

            EtatVol::Montee => {
                let alt = ctx.altitude_m?;
                (alt >= ctx.altitude_cible_m).then_some(EtatVol::VolLigneDroite)?
            }

            EtatVol::Final => {
                let h = ctx.hauteur_sol_mm?;
                (h <= HAUTEUR_FLARE_MM).then_some(EtatVol::Arrondi)?
            }

            _ => return None,
        };

        log::info!(target: "mission", "MAÉ vol tick : {} → {}", self.etat, nouvel_etat);
        self.etat = nouvel_etat.clone();
        Some(nouvel_etat)
    }

    /// Traite une commande externe (sol via LoRa) ou interne (module système).
    ///
    /// Retourne `Some(nouvel_etat)` si la commande était valide dans l'état courant,
    /// `None` si elle est ignorée (commande hors-contexte, comportement normal).
    pub fn traiter_commande(&mut self, commande: CommandeVol) -> Option<EtatVol> {
        use EtatVol::*;
        use CommandeVol::*;

        let nouvel_etat = match (&self.etat, commande) {

            // --- Pré-vol ---
            (Boot,           DriversInitialises)    => Initialisation,
            (Initialisation, ConfigurationTerminee) => AutoTest,
            // AutoTest → AttenteArmement : géré par tick() sur estimation_prete
            // Reprise en vol : bypass de toute la séquence sol → loiter immédiat
            (Initialisation, RepriseRapide)         => AttenteEnCercle,

            // --- Décollage ---
            (AttenteArmement, Armer)                 => Armement,
            (Armement,        ArmementValide)        => RoulageAuSol,
            (RoulageAuSol,    PositionDecollageAtteinte) => CourseDecollage,
            // CourseDecollage → Montee : géré par tick() sur vitesse_sol

            // --- Croisière ---
            // Montee → VolLigneDroite : géré par tick() sur altitude
            (VolLigneDroite, ActiverNavigation)      => Navigation,
            (Navigation,     DesactiverNavigation)   => VolLigneDroite,
            (VolLigneDroite, Loiter)                 => AttenteEnCercle,
            (Navigation,     Loiter)                 => AttenteEnCercle,
            (AttenteEnCercle, ReprendreNavigation)   => Navigation,

            // --- Atterrissage ---
            (VolLigneDroite,  Atterrir)              => Approche,
            (Navigation,      Atterrir)              => Approche,
            (AttenteEnCercle, Atterrir)              => Approche,
            (Approche,        ApprocheStabilisee)    => Final,
            // Final → Arrondi : géré par tick() sur hauteur_sol
            (Arrondi,         ContactSol)            => RoulageAuSol,
            (RoulageAuSol,    Desarmer)              => Desarmement,
            (Desarmement,     ActionneursDesarmes)   => SauvegardeDonnees,

            _ => return None,
        };

        log::info!(target: "mission", "MAÉ vol commande : {} → {}", self.etat, nouvel_etat);
        self.etat = nouvel_etat.clone();
        Some(nouvel_etat)
    }

    /// Force une transition d'urgence depuis tout état vol en l'air.
    ///
    /// Appelé en **premier** à chaque itération, avant `tick` et `traiter_commande`.
    /// Ignore les alertes non critiques (`Normal`, `AlerteMineure`, `AlerteMajeure`).
    ///
    /// Mapping MAÉ sécurité → MAÉ vol :
    /// - `ArretUrgence` | `AtterrissageUrgence` → `Approche`
    /// - `FailSafe` → `AttenteEnCercle` (loiter, puis atterrissage autonome)
    ///
    /// Retourne `true` si une transition a eu lieu.
    pub fn forcer_urgence(&mut self, securite: &EtatSecurite) -> bool {
        if !self.etat.est_en_vol() {
            return false;
        }

        let cible = match securite {
            EtatSecurite::ArretUrgence { .. }        => EtatVol::Approche,
            EtatSecurite::AtterrissageUrgence { .. } => EtatVol::Approche,
            EtatSecurite::FailSafe                   => EtatVol::AttenteEnCercle,
            _ => return false,
        };

        if self.etat == cible {
            return false;
        }

        log::warn!(target: "mission", "MAÉ vol urgence ({}) : {} → {}", securite, self.etat, cible);
        self.etat = cible;
        true
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Tests
// ─────────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    fn mae_en(etat: EtatVol) -> MachineEtatVol {
        MachineEtatVol { etat }
    }

    fn ctx_vide() -> ContexteVol {
        ContexteVol {
            vitesse_sol_ms: None,
            altitude_m: None,
            altitude_cible_m: 100.0,
            hauteur_sol_mm: None,
            estimation_prete: false,
        }
    }

    // --- Pré-vol ---

    #[test]
    fn prevol_par_commandes() {
        let mut mae = MachineEtatVol::nouveau();
        mae.traiter_commande(CommandeVol::DriversInitialises);
        assert_eq!(*mae.etat(), EtatVol::Initialisation);
        mae.traiter_commande(CommandeVol::ConfigurationTerminee);
        assert_eq!(*mae.etat(), EtatVol::AutoTest);
    }

    #[test]
    fn autotest_vers_attente_par_tick() {
        let mut mae = mae_en(EtatVol::AutoTest);
        let ctx = ContexteVol { estimation_prete: true, ..ctx_vide() };
        mae.tick(&ctx);
        assert_eq!(*mae.etat(), EtatVol::AttenteArmement);
    }

    #[test]
    fn autotest_bloque_si_capteurs_non_prets() {
        let mut mae = mae_en(EtatVol::AutoTest);
        mae.tick(&ctx_vide());
        assert_eq!(*mae.etat(), EtatVol::AutoTest);
    }

    // --- Décollage ---

    #[test]
    fn rotation_declenchee_par_vitesse() {
        let mut mae = mae_en(EtatVol::CourseDecollage);
        let ctx = ContexteVol { vitesse_sol_ms: Some(VITESSE_ROTATION_MS + 1.0), ..ctx_vide() };
        mae.tick(&ctx);
        assert_eq!(*mae.etat(), EtatVol::Montee);
    }

    #[test]
    fn rotation_bloquee_sous_vitesse() {
        let mut mae = mae_en(EtatVol::CourseDecollage);
        let ctx = ContexteVol { vitesse_sol_ms: Some(5.0), ..ctx_vide() };
        mae.tick(&ctx);
        assert_eq!(*mae.etat(), EtatVol::CourseDecollage);
    }

    // --- Atterrissage ---

    #[test]
    fn chemin_nominal_atterrissage() {
        let mut mae = mae_en(EtatVol::VolLigneDroite);

        mae.traiter_commande(CommandeVol::Atterrir);
        assert_eq!(*mae.etat(), EtatVol::Approche);

        mae.traiter_commande(CommandeVol::ApprocheStabilisee);
        assert_eq!(*mae.etat(), EtatVol::Final);

        let ctx = ContexteVol { hauteur_sol_mm: Some(HAUTEUR_FLARE_MM - 1), ..ctx_vide() };
        mae.tick(&ctx);
        assert_eq!(*mae.etat(), EtatVol::Arrondi);

        mae.traiter_commande(CommandeVol::ContactSol);
        assert_eq!(*mae.etat(), EtatVol::RoulageAuSol);

        mae.traiter_commande(CommandeVol::Desarmer);
        assert_eq!(*mae.etat(), EtatVol::Desarmement);

        mae.traiter_commande(CommandeVol::ActionneursDesarmes);
        assert_eq!(*mae.etat(), EtatVol::SauvegardeDonnees);
    }

    #[test]
    fn roulage_decollage_ne_desarme_pas_sur_arret() {
        let mut mae = mae_en(EtatVol::Armement);
        mae.traiter_commande(CommandeVol::ArmementValide);
        assert_eq!(*mae.etat(), EtatVol::RoulageAuSol);

        // tick() avec vitesse = 0 : ne doit rien déclencher (arrêt temporaire entre points)
        let ctx = ContexteVol { vitesse_sol_ms: Some(0.0), ..ctx_vide() };
        mae.tick(&ctx);
        assert_eq!(*mae.etat(), EtatVol::RoulageAuSol);
    }

    #[test]
    fn desarmer_depuis_roulage() {
        let mut mae = mae_en(EtatVol::RoulageAuSol);
        mae.traiter_commande(CommandeVol::Desarmer);
        assert_eq!(*mae.etat(), EtatVol::Desarmement);
    }

    // --- Commande ignorée hors contexte ---

    #[test]
    fn commande_ignoree_hors_contexte() {
        let mut mae = MachineEtatVol::nouveau(); // Boot
        assert!(mae.traiter_commande(CommandeVol::Armer).is_none());
        assert_eq!(*mae.etat(), EtatVol::Boot);
    }

    // --- Urgences ---

    #[test]
    fn failsafe_vers_loiter() {
        let mut mae = mae_en(EtatVol::Navigation);
        assert!(mae.forcer_urgence(&EtatSecurite::FailSafe));
        assert_eq!(*mae.etat(), EtatVol::AttenteEnCercle);
    }

    #[test]
    fn arret_urgence_vers_approche() {
        let mut mae = mae_en(EtatVol::VolLigneDroite);
        let sec = EtatSecurite::ArretUrgence { raison: "emballement".into() };
        assert!(mae.forcer_urgence(&sec));
        assert_eq!(*mae.etat(), EtatVol::Approche);
    }

    #[test]
    fn urgence_ignoree_au_sol() {
        let mut mae = mae_en(EtatVol::AttenteArmement);
        assert!(!mae.forcer_urgence(&EtatSecurite::FailSafe));
        assert_eq!(*mae.etat(), EtatVol::AttenteArmement);
    }

    #[test]
    fn alerte_mineure_ne_force_pas() {
        let mut mae = mae_en(EtatVol::Navigation);
        let sec = EtatSecurite::AlerteMineure { raison: "test".into() };
        assert!(!mae.forcer_urgence(&sec));
        assert_eq!(*mae.etat(), EtatVol::Navigation);
    }
}
