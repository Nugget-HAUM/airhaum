// src/taches/taches_controle.rs
//! Tâche Tokio de la couche contrôle (couche 4) — brique minimale.
//!
//! Ne branche pour l'instant que `controleur_direction_sol` (roulage,
//! course décollage) : `controleur_roulis`/`controleur_tangage`/`controleur_lacet`
//! n'existent pas encore, les commandes ailerons/profondeur restent donc au
//! neutre. Gaz et armement sont figés à zéro/non-armé — `ConsigneGaz` n'est
//! pas encore défini et la couche mission n'existe pas encore pour les
//! piloter. Voir `doc/controle.md`.
//!
//! # Canaux
//!
//! ```text
//! tache_estimation                  tache_controle                  tache_servo
//! ────────────────                  ──────────────                  ───────────
//! EtatNavigation (watch) ──────────▶ ControleurDirectionSol
//! (mission, à venir)
//! ConsigneDirectionSol (watch) ────▶ + mixeur ────ConsignesServos (watch)───▶
//! ```
//!
//! `tx_consigne_direction_sol` est possédé par cette tâche (créé par
//! `lancer_controle`), sur le même principe que `tx_consignes` déjà possédé
//! par `HandlesServo` — la mission, quand elle existera, en obtiendra un
//! clone pour y écrire ses consignes.
//!
//! # Fréquences
//!
//! Réactif sur `rx_navigation.changed()` (pas de minuterie fixe), donc cadencé
//! par l'estimation d'état (~200 Hz, voir `doc/gestion_du_temps.md`). Le `dt`
//! est calculé entre deux `EtatNavigation` reçus via `Horodatage::delta_secondes`,
//! jamais supposé constant.

use std::sync::{atomic::{AtomicBool, Ordering}, Arc};
use tokio::sync::watch;

use crate::controle::{mixer, CommandesNormalisees, ConsigneDirectionSol, ControleurDirectionSol};
use crate::drivers::controleur_servo::{ConsignesServos, IMPULSION_MIN_US};
use crate::estimation::EtatNavigation;
use crate::types::{Angle, Horodatage};

// ─────────────────────────────────────────────────────────────────────────────
// Handle
// ─────────────────────────────────────────────────────────────────────────────

/// Handle retourné par [`lancer_controle`].
pub struct HandlesControle {
    /// Émet la consigne de cap au sol (mission → contrôle, à venir).
    pub tx_consigne_direction_sol: watch::Sender<ConsigneDirectionSol>,
    /// Tâche Tokio — annulable via `arreter()`.
    tache: tokio::task::JoinHandle<()>,
    /// Signal d'arrêt partagé avec la tâche.
    arret: Arc<AtomicBool>,
}

impl HandlesControle {
    /// Signale l'arrêt à la tâche de contrôle. Non-bloquant.
    pub fn arreter(&self) {
        self.arret.store(true, Ordering::Relaxed);
        self.tache.abort();
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Point d'entrée
// ─────────────────────────────────────────────────────────────────────────────

/// Lance la tâche de contrôle (brique minimale : direction sol uniquement).
///
/// - `rx_navigation`     : canal watch cloné depuis `HandlesEstimation`.
/// - `tx_consignes_servo`: canal watch cloné depuis `HandlesServo::tx_consignes`.
/// - `kp`/`ki`/`kd`/`limite_integrale` : gains du PID de direction sol —
///   aucune valeur par défaut, à régler à l'essai (voir `doc/controle.md`).
///
/// Doit être appelé depuis un contexte Tokio (après `#[tokio::main]`).
pub fn lancer_controle(
    rx_navigation: watch::Receiver<EtatNavigation>,
    tx_consignes_servo: watch::Sender<ConsignesServos>,
    kp: f32,
    ki: f32,
    kd: f32,
    limite_integrale: f32,
) -> HandlesControle {
    let arret = Arc::new(AtomicBool::new(false));
    let (tx_consigne_direction_sol, rx_consigne_direction_sol) = watch::channel(
        ConsigneDirectionSol { cap_cible: Angle::depuis_radians(0.0) }
    );

    let controleur_direction_sol = ControleurDirectionSol::nouveau(kp, ki, kd, limite_integrale);

    let arret_tache = Arc::clone(&arret);
    let tache = tokio::spawn(async move {
        tache_controle(
            rx_navigation,
            rx_consigne_direction_sol,
            tx_consignes_servo,
            controleur_direction_sol,
            arret_tache,
        ).await;
    });

    HandlesControle { tx_consigne_direction_sol, tache, arret }
}

// ─────────────────────────────────────────────────────────────────────────────
// Corps de la tâche
// ─────────────────────────────────────────────────────────────────────────────

async fn tache_controle(
    mut rx_navigation: watch::Receiver<EtatNavigation>,
    rx_consigne_direction_sol: watch::Receiver<ConsigneDirectionSol>,
    tx_consignes_servo: watch::Sender<ConsignesServos>,
    mut controleur_direction_sol: ControleurDirectionSol,
    arret: Arc<AtomicBool>,
) {
    let mut dernier_horodatage: Option<Horodatage> = None;

    loop {
        if arret.load(Ordering::Relaxed) { break; }

        tokio::select! {
            Ok(_) = rx_navigation.changed() => {
                let nav = *rx_navigation.borrow_and_update();

                let dt = match dernier_horodatage {
                    Some(precedent) => nav.horodatage.delta_secondes(precedent),
                    None => 0.0,
                };
                dernier_horodatage = Some(nav.horodatage);

                let consigne = *rx_consigne_direction_sol.borrow();
                let commande_direction = controleur_direction_sol.mettre_a_jour(
                    &consigne, nav.attitude.lacet, dt,
                );

                let commandes = CommandesNormalisees {
                    ailerons:   0.0,
                    profondeur: 0.0,
                    direction:  commande_direction,
                };
                // Gaz/armement figés tant que la mission ne les pilote pas —
                // voir la note de brique minimale en tête de fichier.
                let consignes_servos = mixer(&commandes, IMPULSION_MIN_US, false);
                let _ = tx_consignes_servo.send(consignes_servos);
            }

            else => break,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::Duration;
    use crate::drivers::controleur_servo::IMPULSION_NEUTRE_US;

    fn lancer_test(kp: f32) -> (
        watch::Sender<EtatNavigation>,
        watch::Receiver<ConsignesServos>,
        HandlesControle,
    ) {
        let (tx_navigation, rx_navigation) = watch::channel(EtatNavigation::nul());
        let (tx_consignes_servo, rx_consignes_servo) = watch::channel(ConsignesServos::neutre());

        let handles = lancer_controle(rx_navigation, tx_consignes_servo, kp, 0.0, 0.0, 1.0);

        (tx_navigation, rx_consignes_servo, handles)
    }

    #[tokio::test]
    async fn reagit_a_une_erreur_de_cap() {
        let (tx_navigation, mut rx_consignes_servo, handles) = lancer_test(1.0);

        // Consigne par défaut (cap_cible = 0°) inchangée : seul le cap mesuré s'écarte.
        let mut nav = EtatNavigation::nul();
        nav.attitude.lacet = Angle::depuis_degres(10.0);
        tx_navigation.send(nav).unwrap();

        tokio::time::sleep(Duration::from_millis(50)).await;

        let consignes = rx_consignes_servo.borrow_and_update().clone();
        assert_ne!(consignes.direction, IMPULSION_NEUTRE_US);

        handles.arreter();
    }

    #[tokio::test]
    async fn gaz_et_armement_toujours_figes_en_securite() {
        let (tx_navigation, mut rx_consignes_servo, handles) = lancer_test(1.0);

        let mut nav = EtatNavigation::nul();
        nav.attitude.lacet = Angle::depuis_degres(45.0);
        tx_navigation.send(nav).unwrap();

        tokio::time::sleep(Duration::from_millis(50)).await;

        let consignes = rx_consignes_servo.borrow_and_update().clone();
        assert_eq!(consignes.gaz, IMPULSION_MIN_US);
        assert!(!consignes.arme);
        // Ailerons/profondeur : pas encore de contrôleur, doivent rester au neutre.
        assert_eq!(consignes.ailerons, IMPULSION_NEUTRE_US);
        assert_eq!(consignes.profondeur, IMPULSION_NEUTRE_US);

        handles.arreter();
    }

    #[tokio::test]
    async fn aucune_commande_quand_cap_deja_atteint() {
        let (tx_navigation, mut rx_consignes_servo, handles) = lancer_test(1.0);

        let mut nav = EtatNavigation::nul();
        nav.attitude.lacet = Angle::depuis_degres(0.0); // égal à la consigne par défaut
        tx_navigation.send(nav).unwrap();

        tokio::time::sleep(Duration::from_millis(50)).await;

        let consignes = rx_consignes_servo.borrow_and_update().clone();
        assert_eq!(consignes.direction, IMPULSION_NEUTRE_US);

        handles.arreter();
    }
}
