// src/surete/sante.rs
//! Surveillance de la santé des capteurs et vivacité des threads.
//!
//! Module de logique pure — aucune E/S, aucun thread. Il évalue un état de
//! santé à partir des signaux déjà publiés par `taches/taches_capteurs.rs`,
//! `taches/taches_gps.rs` et `taches/taches_servo.rs`. La boucle qui
//! l'appelle périodiquement vivra dans `taches/taches_surete.rs`.
//!
//! Deux défauts qu'un simple compteur d'erreurs ne détecte pas :
//! - un thread **bloqué** (au lieu d'erreur, il ne publie plus rien : le
//!   canal `watch` continue de renvoyer la dernière valeur, `valide` et
//!   fraîche en apparence) → détecté par péremption (`delai_peremption`)
//! - un état qui **oscille** juste après une période dégradée → une
//!   hystérésis retarde le retour à `Nominal` de quelques cycles sains

use std::time::{Duration, Instant};

/// État de santé d'une source surveillée (capteur, liaison, thread).
#[derive(Debug, Clone, PartialEq)]
pub enum EtatSante {
    /// Aucune anomalie détectée.
    Nominal,
    /// Anomalie en cours, mais pas encore critique.
    Degrade { raison: String },
    /// Source indisponible : aucune donnée fiable ne peut en être tirée.
    HorsService { raison: String },
}

impl EtatSante {
    pub fn est_nominal(&self) -> bool {
        matches!(self, EtatSante::Nominal)
    }

    pub fn est_hors_service(&self) -> bool {
        matches!(self, EtatSante::HorsService { .. })
    }
}

/// Nombre de cycles consécutifs sains requis avant de repasser en `Nominal`
/// depuis un état dégradé — évite les oscillations sur un capteur limite.
const CYCLES_RETABLISSEMENT: u32 = 5;

// ─────────────────────────────────────────────────────────────────────────────
// Suivi temporel par source
// ─────────────────────────────────────────────────────────────────────────────

/// Suit la santé d'une source dans le temps : combine un état instantané
/// (fourni par l'appelant à chaque cycle) avec une détection de péremption
/// et une hystérésis au rétablissement.
pub struct MoniteurSante {
    nom: String,
    etat: EtatSante,
    derniere_maj: Instant,
    delai_peremption: Duration,
    cycles_bons_consecutifs: u32,
}

impl MoniteurSante {
    pub fn nouveau(nom: impl Into<String>, delai_peremption: Duration) -> Self {
        Self {
            nom: nom.into(),
            etat: EtatSante::Nominal,
            derniere_maj: Instant::now(),
            delai_peremption,
            cycles_bons_consecutifs: 0,
        }
    }

    pub fn etat(&self) -> &EtatSante {
        &self.etat
    }

    /// À appeler à chaque cycle de supervision.
    ///
    /// * `etat_instantane` — évaluation du cycle courant, sans mémoire
    ///   (voir [`evaluer_capteur`] / [`evaluer_arduino`])
    /// * `nouvelle_donnee` — vrai si une nouvelle valeur est arrivée sur le
    ///   canal depuis le dernier appel (`watch::Receiver::has_changed()`)
    /// * `thread_vivant` — `!JoinHandle::is_finished()`
    pub fn actualiser(
        &mut self,
        etat_instantane: EtatSante,
        nouvelle_donnee: bool,
        thread_vivant: bool,
    ) -> &EtatSante {
        if !thread_vivant {
            self.etat = EtatSante::HorsService {
                raison: format!("{} : thread arrêté", self.nom),
            };
            self.cycles_bons_consecutifs = 0;
            return &self.etat;
        }

        if nouvelle_donnee {
            self.derniere_maj = Instant::now();
        }

        if self.derniere_maj.elapsed() > self.delai_peremption {
            self.etat = EtatSante::HorsService {
                raison: format!(
                    "{} : aucune donnée depuis {:.1}s",
                    self.nom,
                    self.derniere_maj.elapsed().as_secs_f32()
                ),
            };
            self.cycles_bons_consecutifs = 0;
            return &self.etat;
        }

        match etat_instantane {
            EtatSante::Nominal => {
                self.cycles_bons_consecutifs += 1;
                if self.etat.est_nominal() || self.cycles_bons_consecutifs >= CYCLES_RETABLISSEMENT {
                    self.etat = EtatSante::Nominal;
                }
                // Sinon : encore en cours de rétablissement, l'état affiché
                // reste dégradé jusqu'à ce que le seuil soit atteint.
            }
            autre => {
                self.cycles_bons_consecutifs = 0;
                self.etat = autre;
            }
        }

        &self.etat
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Évaluations instantanées (sans mémoire)
// ─────────────────────────────────────────────────────────────────────────────

const SEUIL_DEGRADE_ERREURS: u32 = 5;
const SEUIL_HORS_SERVICE_ERREURS: u32 = 20;

/// Évalue l'état instantané d'un capteur à partir des métadonnées publiées
/// par son thread (`valide`, `erreurs_consecutives`).
pub fn evaluer_capteur(nom: &str, valide: bool, erreurs_consecutives: u32) -> EtatSante {
    if erreurs_consecutives >= SEUIL_HORS_SERVICE_ERREURS {
        EtatSante::HorsService {
            raison: format!("{nom} : {erreurs_consecutives} erreurs consécutives"),
        }
    } else if !valide || erreurs_consecutives >= SEUIL_DEGRADE_ERREURS {
        EtatSante::Degrade {
            raison: format!("{nom} : {erreurs_consecutives} erreurs consécutives"),
        }
    } else {
        EtatSante::Nominal
    }
}

/// Évalue l'état instantané de la liaison Arduino/RC à partir des drapeaux
/// de la trame de remontée (pas de compteur d'erreurs pour cette liaison).
pub fn evaluer_arduino(rc_perdu: bool, chien_de_garde: bool) -> EtatSante {
    if chien_de_garde {
        EtatSante::Degrade { raison: "ESP32 : chien de garde déclenché".into() }
    } else if rc_perdu {
        EtatSante::Degrade { raison: "ESP32 : signal RC perdu".into() }
    } else {
        EtatSante::Nominal
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Agrégation
// ─────────────────────────────────────────────────────────────────────────────

/// Détermine l'état le plus critique parmi plusieurs sources surveillées.
pub fn pire_etat<'a>(etats: impl IntoIterator<Item = &'a EtatSante>) -> EtatSante {
    let mut pire = EtatSante::Nominal;
    for etat in etats {
        pire = match (&pire, etat) {
            (_, EtatSante::HorsService { .. }) => etat.clone(),
            (EtatSante::HorsService { .. }, _) => pire,
            (EtatSante::Nominal, EtatSante::Degrade { .. }) => etat.clone(),
            _ => pire,
        };
    }
    pire
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_evaluer_capteur_nominal() {
        assert_eq!(evaluer_capteur("baro", true, 0), EtatSante::Nominal);
    }

    #[test]
    fn test_evaluer_capteur_degrade() {
        assert!(matches!(evaluer_capteur("baro", true, 5), EtatSante::Degrade { .. }));
        assert!(matches!(evaluer_capteur("baro", false, 0), EtatSante::Degrade { .. }));
    }

    #[test]
    fn test_evaluer_capteur_hors_service() {
        assert!(matches!(evaluer_capteur("baro", false, 20), EtatSante::HorsService { .. }));
    }

    #[test]
    fn test_evaluer_arduino() {
        assert_eq!(evaluer_arduino(false, false), EtatSante::Nominal);
        assert!(matches!(evaluer_arduino(true, false), EtatSante::Degrade { .. }));
        assert!(matches!(evaluer_arduino(false, true), EtatSante::Degrade { .. }));
    }

    #[test]
    fn test_moniteur_thread_mort() {
        let mut m = MoniteurSante::nouveau("imu", Duration::from_secs(1));
        let etat = m.actualiser(EtatSante::Nominal, true, false);
        assert!(etat.est_hors_service());
    }

    #[test]
    fn test_moniteur_peremption() {
        let mut m = MoniteurSante::nouveau("imu", Duration::from_millis(10));
        std::thread::sleep(Duration::from_millis(20));
        let etat = m.actualiser(EtatSante::Nominal, false, true);
        assert!(etat.est_hors_service());
    }

    #[test]
    fn test_moniteur_hysteresis_retablissement() {
        let mut m = MoniteurSante::nouveau("baro", Duration::from_secs(10));
        m.actualiser(EtatSante::Degrade { raison: "test".into() }, true, true);
        assert!(!m.etat().est_nominal());

        // Quelques cycles sains, pas encore assez pour repasser Nominal.
        for _ in 0..CYCLES_RETABLISSEMENT - 1 {
            m.actualiser(EtatSante::Nominal, true, true);
        }
        assert!(!m.etat().est_nominal());

        // Un cycle sain de plus atteint le seuil.
        m.actualiser(EtatSante::Nominal, true, true);
        assert!(m.etat().est_nominal());
    }

    #[test]
    fn test_pire_etat() {
        let nominal = EtatSante::Nominal;
        let degrade = EtatSante::Degrade { raison: "x".into() };
        let hors_service = EtatSante::HorsService { raison: "y".into() };

        assert_eq!(pire_etat([&nominal, &degrade]), degrade);
        assert_eq!(pire_etat([&degrade, &hors_service]), hors_service);
        assert_eq!(pire_etat([&nominal]), EtatSante::Nominal);
    }
}
