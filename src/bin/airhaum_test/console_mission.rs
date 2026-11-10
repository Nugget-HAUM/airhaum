// src/bin/airhaum_test/console_mission.rs
//! Écran de mission : déroulé du plan et chronomètre, rafraîchis chaque
//! seconde de `l` (lancer la mission) jusqu'à la fin de mission.
//! Voir `doc/mission.md`, « Écran de mission ».

use std::io::{self, Write};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use airhaum::drivers::controleur_servo::{ConsignesServos, EtatArduino, ModeArduino};
use airhaum::mission::etat_machine::{CommandeVol, EtatVol};
use airhaum::mission::plan_vol::PlanDeVol;
use airhaum::taches::taches_mission::{HandlesMission, ProgressionPlan};
use airhaum::taches::taches_servo::HandlesServo;
use airhaum::taches::taches_surete::HandlesSurete;

use super::console_test::{entree_disponible, lire_reponse};

/// Période de rafraîchissement de l'écran.
const PERIODE_ECRAN_MS: i32 = 1000;

/// `mm:ss`, secondes entamées comptées (un compte à rebours finit à 00:00).
fn mmss(secondes: f32) -> String {
    let s = secondes.max(0.0).ceil() as u32;
    format!("{:02}:{:02}", s / 60, s % 60)
}

/// Durée totale du plan, `None` si un segment attend une commande opérateur.
fn duree_totale_s(plan: &PlanDeVol) -> Option<f32> {
    plan.segments().iter().map(|s| s.duree_s()).sum()
}

/// Lignes du déroulé : segments faits (✓), en cours (▶) et à venir, puis la
/// fin de mission. Sans progression, le plan est affiché tel quel.
fn lignes_deroule(plan: &PlanDeVol, progression: Option<&ProgressionPlan>) -> Vec<String> {
    let courant = progression.map(|p| p.segment);
    let mut lignes = Vec::new();

    // Temps restant jusqu'à la fin de mission, tant qu'il est calculable.
    let mut avant_fin: Option<f32> = Some(0.0);

    for (i, segment) in plan.segments().iter().enumerate() {
        let duree = segment.duree_s().map_or("commande opérateur".to_string(), |d| format!("{d:.0} s"));
        let (marque, detail) = match courant {
            Some(c) if i < c => ("✓", String::new()),
            Some(c) if i == c => {
                let ecoule = progression.map_or(0.0, |p| p.ecoule_segment_s);
                match segment.duree_s() {
                    Some(d) => {
                        avant_fin = avant_fin.map(|t| t + (d - ecoule).max(0.0));
                        ("▶", format!("reste {}", mmss(d - ecoule)))
                    }
                    None => {
                        avant_fin = None;
                        ("▶", "s. Segment suivant".to_string())
                    }
                }
            }
            _ => {
                avant_fin = match (avant_fin, segment.duree_s()) {
                    (Some(t), Some(d)) => Some(t + d),
                    _ => None,
                };
                (" ", String::new())
            }
        };
        let debut = format!("  {} {}. {}", marque, i + 1, segment.type_segment);
        lignes.push(format!("{:<39} {:<20} {}", debut, duree, detail).trim_end().to_string());
    }

    let fin = match (courant, avant_fin) {
        (Some(c), _) if c >= plan.nombre_segments() => "atteinte".to_string(),
        (Some(_), Some(t)) => format!("dans {}", mmss(t)),
        _ => String::new(),
    };
    lignes.push(format!("{:<39} {:<20} {}", "    Fin : désarmement automatique", "", fin).trim_end().to_string());
    lignes
}

/// Tableau des impulsions servo : consignes envoyées par le Pi, canaux RC reçus
/// par l'ESP32, gaz appliqué. La remontée ne donne pas la position appliquée
/// aux gouvernes : « — », plutôt qu'une valeur déduite du mode.
fn lignes_servos(consignes: &ConsignesServos, esp32: &EtatArduino) -> Vec<String> {
    let rc = &esp32.canaux_rc;
    vec![
        format!("  {:<18} {:>9} {:>11} {:>6} {:>10}", "Servos (µs)", "Ailerons", "Profondeur", "Gaz", "Direction"),
        format!("  {:<18} {:>9} {:>11} {:>6} {:>10}", "Envoyé (Pi)",
            consignes.ailerons, consignes.profondeur, consignes.gaz, consignes.direction),
        format!("  {:<18} {:>9} {:>11} {:>6} {:>10}", "Reçu (RC)", rc[0], rc[1], rc[2], rc[3]),
        format!("  {:<18} {:>9} {:>11} {:>6} {:>10}", "Appliqué (ESP32)", "—", "—", esp32.gaz_applique, "—"),
    ]
}

/// En-tête : état vol et chronomètre de la mission.
fn ligne_titre(etat: &EtatVol, plan: &PlanDeVol, progression: Option<&ProgressionPlan>) -> String {
    let ecoule = progression.map_or(0.0, |p| p.ecoule_mission_s);
    let total = duree_totale_s(plan).map_or("--:--".to_string(), mmss);
    format!("  MISSION EN COURS — {:<22} écoulé {} / {}", etat.to_string(), mmss(ecoule), total)
}

/// Affiche l'écran de mission jusqu'à la fin de mission, ou jusqu'à `q`
/// (retour à la check-liste, la mission continue).
///
/// Touches : `d` désarmer (fin de mission anticipée, en `RoulageAuSol`),
/// `s` segment suivant (segment à commande opérateur), `q` retour.
pub async fn afficher_ecran_mission(
    nom_plan: &str,
    plan: &PlanDeVol,
    handles_mission: &HandlesMission,
    servo: &HandlesServo,
    surete: &HandlesSurete,
    commande_operateur_segment: &Arc<AtomicBool>,
) {
    loop {
        let etat = handles_mission.rx_etat_vol.borrow().clone();
        let progression = *handles_mission.rx_progression.borrow();
        let esp32 = servo.rx_etat.borrow().clone();
        let consignes = servo.tx_consignes.borrow().clone();
        let arme = consignes.arme;
        let securite = surete.rx_securite.borrow().clone();

        print!("\x1B[2J\x1B[1;1H");

        if etat == EtatVol::SauvegardeDonnees {
            let duree = progression.map_or("--:--".to_string(), |p| mmss(p.ecoule_mission_s));
            println!("╔══════════════════════════════════════════════════════════╗");
            println!("║   MISSION TERMINÉE — désarmé, données sauvegardées      ║");
            println!("╚══════════════════════════════════════════════════════════╝\n");
            println!("  Plan : {}   durée : {}\n", nom_plan, duree);
            for ligne in lignes_deroule(plan, progression.as_ref()) {
                println!("{ligne}");
            }
            println!("\n  Sécurité : {}", securite);
            println!("\n  [Entrée] Terminer");
            io::stdout().flush().unwrap();
            let _ = lire_reponse();
            return;
        }

        println!("╔══════════════════════════════════════════════════════════╗");
        println!("{}", ligne_titre(&etat, plan, progression.as_ref()));
        println!("╚══════════════════════════════════════════════════════════╝\n");
        println!("  Plan : {}\n", nom_plan);
        for ligne in lignes_deroule(plan, progression.as_ref()) {
            println!("{ligne}");
        }

        let mode = match esp32.mode {
            ModeArduino::Autopilote => "Autopilote".to_string(),
            ModeArduino::Manuel => "MANUEL — opérateur aux commandes".to_string(),
        };
        println!("\n  Mode : {}   Armé : {}", mode, if arme { "oui" } else { "non" });
        println!();
        for ligne in lignes_servos(&consignes, &esp32) {
            println!("{ligne}");
        }
        println!();
        if esp32.rc_perdu || esp32.chien_de_garde {
            println!("  ⚠ ESP32 : RC perdue={}  chien de garde={}", esp32.rc_perdu, esp32.chien_de_garde);
        }
        println!("  Sécurité : {}", securite);

        let peut_desarmer = etat == EtatVol::RoulageAuSol;
        let attend_operateur = progression
            .and_then(|p| plan.segments().get(p.segment))
            .is_some_and(|s| s.duree_s().is_none());

        println!();
        if peut_desarmer { print!("  d. Désarmer (fin de mission)   "); }
        if attend_operateur { print!("s. Segment suivant   "); }
        println!("q. Retour à la check-liste");
        print!("\n  Votre choix : ");
        io::stdout().flush().unwrap();

        if !entree_disponible(PERIODE_ECRAN_MS) {
            continue;
        }
        match lire_reponse().as_str() {
            "d" if peut_desarmer => {
                log::info!(target: "mission", "Désarmement demandé à l'écran de mission");
                let _ = handles_mission.tx_commande.send(CommandeVol::Desarmer).await;
            }
            "s" if attend_operateur => {
                log::info!(target: "mission", "Fin de segment demandée par l'opérateur");
                commande_operateur_segment.store(true, Ordering::Relaxed);
            }
            "q" => return,
            _ => {}
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn plan() -> PlanDeVol {
        PlanDeVol::depuis_toml(r#"
[[segment]]
type = "attente"
fin = { duree_s = 20 }

[[segment]]
type = "roulage_cap"
cap_deg = 0.0
gaz = 0.1
fin = { duree_s = 60 }

[[segment]]
type = "attente"
fin = { duree_s = 10 }
"#).unwrap()
    }

    fn progression(segment: usize, ecoule_segment_s: f32, ecoule_mission_s: f32) -> ProgressionPlan {
        ProgressionPlan { segment, ecoule_segment_s, ecoule_mission_s }
    }

    #[test]
    fn chronometre_et_comptes_a_rebours() {
        let p = progression(1, 7.0, 27.0);
        assert!(ligne_titre(&EtatVol::RoulageAuSol, &plan(), Some(&p)).contains("écoulé 00:27 / 01:30"));

        let lignes = lignes_deroule(&plan(), Some(&p));
        assert!(lignes[0].starts_with("  ✓ 1. attente"));
        assert!(lignes[1].starts_with("  ▶ 2. roulage cap 0°, gaz 10 %"));
        assert!(lignes[1].ends_with("reste 00:53"), "{}", lignes[1]);
        assert!(lignes[2].starts_with("    3. attente"));
        assert!(lignes[3].ends_with("dans 01:03"), "{}", lignes[3]);
    }

    #[test]
    fn servos_envoyes_recus_et_gaz_applique() {
        let consignes = ConsignesServos { ailerons: 1600, profondeur: 1400, gaz: 1100, direction: 2000, arme: true };
        let esp32 = EtatArduino {
            canaux_rc: [1402, 1498, 1003, 1500, 2000], gaz_applique: 1100, ..EtatArduino::absent()
        };
        let lignes = lignes_servos(&consignes, &esp32);
        fn valeurs(l: &str) -> Vec<&str> { l.split_whitespace().rev().take(4).collect() }
        assert_eq!(valeurs(&lignes[1]), ["2000", "1100", "1400", "1600"]);
        assert_eq!(valeurs(&lignes[2]), ["1500", "1003", "1498", "1402"]);
        assert_eq!(valeurs(&lignes[3]), ["—", "1100", "—", "—"]);
    }

    #[test]
    fn plan_non_lance_puis_termine() {
        let lignes = lignes_deroule(&plan(), None);
        assert!(lignes.iter().all(|l| !l.contains('✓') && !l.contains('▶')));

        let lignes = lignes_deroule(&plan(), Some(&progression(3, 0.0, 90.0)));
        assert!(lignes[..3].iter().all(|l| l.starts_with("  ✓")));
        assert!(lignes[3].ends_with("atteinte"));
    }

    #[test]
    fn segment_a_commande_operateur_sans_echeance() {
        let plan = PlanDeVol::depuis_toml(r#"
[[segment]]
type = "attente"
fin = { commande_operateur = "console" }

[[segment]]
type = "attente"
fin = { duree_s = 10 }
"#).unwrap();
        assert!(ligne_titre(&EtatVol::RoulageAuSol, &plan, None).ends_with("--:--"));

        let lignes = lignes_deroule(&plan, Some(&progression(0, 4.0, 4.0)));
        assert!(lignes[0].contains("commande opérateur") && lignes[0].ends_with("s. Segment suivant"));
        assert!(!lignes[2].contains("dans"));
    }
}

#[cfg(test)]
mod apercu {
    use super::*;

    #[test]
    #[ignore = "aperçu visuel : cargo test apercu -- --ignored --nocapture"]
    fn apercu_ecran() {
        let plan = tests_plan();
        for p in [None, Some(ProgressionPlan { segment: 1, ecoule_segment_s: 7.0, ecoule_mission_s: 27.0 }),
                  Some(ProgressionPlan { segment: 3, ecoule_segment_s: 0.0, ecoule_mission_s: 90.0 })] {
            println!("{}", ligne_titre(&EtatVol::RoulageAuSol, &plan, p.as_ref()));
            for l in lignes_deroule(&plan, p.as_ref()) { println!("{l}"); }
            println!();
        }
        let esp32 = EtatArduino { canaux_rc: [1402, 1498, 1003, 1500, 2000], gaz_applique: 1100, ..EtatArduino::absent() };
        let consignes = ConsignesServos { gaz: 1100, arme: true, ..ConsignesServos::neutre() };
        for l in lignes_servos(&consignes, &esp32) { println!("{l}"); }
    }

    fn tests_plan() -> PlanDeVol {
        PlanDeVol::depuis_toml("[[segment]]\ntype = \"attente\"\nfin = { duree_s = 20 }\n[[segment]]\ntype = \"roulage_cap\"\ncap_deg = 0.0\ngaz = 0.1\nfin = { duree_s = 60 }\n[[segment]]\ntype = \"attente\"\nfin = { duree_s = 10 }\n").unwrap()
    }
}
