// src/systeme/journalisation.rs
//! Moteur de journalisation — boîte noire (JSONL) + fil de vie (texte)
//!
//! Les appelants utilisent exclusivement la façade `log` (`log::info!`, etc.).
//! Ce module est le seul à connaître les fichiers, le canal et le thread écrivain.
//!
//! # Utilisation
//!
//! ```rust,no_run
//! airhaum::systeme::journalisation::initialiser().expect("journalisation");
//! log::info!(target: "mission", "système démarré");
//! ```

use std::fs::{self, File};
use std::io::Write;
use std::sync::mpsc::{self, SyncSender};
use std::thread;
use std::time::{Duration, Instant};

use chrono::Local;
use log::{LevelFilter, Log, Metadata, Record};

const CAPACITE_CANAL: usize = 256;

/// Répertoire par défaut des fichiers de log, utilisé si la variable
/// d'environnement `AIRHAUM_LOGS_DIR` n'est pas définie.
pub const REPERTOIRE_LOGS_DEFAUT: &str = "/home/airhaum/logs";

/// Résout le répertoire de logs à utiliser : `AIRHAUM_LOGS_DIR` si définie,
/// sinon [`REPERTOIRE_LOGS_DEFAUT`]. Permet de faire tourner la journalisation
/// sur un poste de développement où `/home/airhaum` n'existe pas.
pub fn repertoire_logs() -> String {
    std::env::var("AIRHAUM_LOGS_DIR").unwrap_or_else(|_| REPERTOIRE_LOGS_DEFAUT.to_owned())
}

// ─────────────────────────────────────────────────────────────────────────────
// Message interne
// ─────────────────────────────────────────────────────────────────────────────

struct EntreeJournal {
    ts:     chrono::DateTime<Local>,
    niveau: log::Level,
    cible:  String,
    msg:    String,
}

// ─────────────────────────────────────────────────────────────────────────────
// Moteur — implémente log::Log
// ─────────────────────────────────────────────────────────────────────────────

struct MoteurJournalisation {
    emetteur: SyncSender<Option<EntreeJournal>>,
}

impl Log for MoteurJournalisation {
    fn enabled(&self, metadata: &Metadata) -> bool {
        metadata.level() <= log::max_level()
    }

    fn log(&self, record: &Record) {
        if !self.enabled(record.metadata()) {
            return;
        }
        let entree = EntreeJournal {
            ts:     Local::now(),
            niveau: record.level(),
            cible:  record.target().to_owned(),
            msg:    record.args().to_string(),
        };
        // Non-bloquant : entrée silencieusement abandonnée si le canal est plein
        let _ = self.emetteur.try_send(Some(entree));
    }

    fn flush(&self) {}
}

// ─────────────────────────────────────────────────────────────────────────────
// Thread écrivain
// ─────────────────────────────────────────────────────────────────────────────

fn thread_ecrivain(
    rx:          mpsc::Receiver<Option<EntreeJournal>>,
    mut boite_noire: File,
    mut fil_de_vie:  File,
) {
    let mut derniere_nav: Option<Instant> = None;

    loop {
        match rx.recv() {
            Ok(Some(e)) => {
                ecrire_boite_noire(&mut boite_noire, &e);
                ecrire_fil_de_vie(&mut fil_de_vie, &e, &mut derniere_nav);
            }
            Ok(None) | Err(_) => break,
        }
    }
}

fn ecrire_boite_noire(f: &mut File, e: &EntreeJournal) {
    let msg = echapper_json(&e.msg);
    let _ = writeln!(
        f,
        r#"{{"ts":"{}","lvl":"{}","cible":"{}","msg":"{}"}}"#,
        e.ts.format("%Y-%m-%dT%H:%M:%S%.3f"),
        e.niveau,
        e.cible,
        msg,
    );
}

fn ecrire_fil_de_vie(
    f:            &mut File,
    e:            &EntreeJournal,
    derniere_nav: &mut Option<Instant>,
) {
    use log::Level::*;
    let ecrire = match e.niveau {
        Error | Warn => true,
        Info => {
            if e.cible == "nav" {
                // Limité à 1 Hz pour rester lisible
                let ok = derniere_nav
                    .map(|t| t.elapsed() >= Duration::from_secs(1))
                    .unwrap_or(true);
                if ok {
                    *derniere_nav = Some(Instant::now());
                }
                ok
            } else {
                true // transitions MAÉ, calibrations, armement, etc.
            }
        }
        Debug | Trace => false,
    };

    if ecrire {
        let _ = writeln!(
            f,
            "[{}] {:<5}  {:<8}  {}",
            e.ts.format("%H:%M:%S"),
            e.niveau,
            e.cible,
            e.msg,
        );
    }
}

fn echapper_json(s: &str) -> String {
    let mut out = String::with_capacity(s.len());
    for c in s.chars() {
        match c {
            '"'  => out.push_str("\\\""),
            '\\' => out.push_str("\\\\"),
            '\n' => out.push_str("\\n"),
            '\r' => out.push_str("\\r"),
            '\t' => out.push_str("\\t"),
            c    => out.push(c),
        }
    }
    out
}

// ─────────────────────────────────────────────────────────────────────────────
// Handle de session
// ─────────────────────────────────────────────────────────────────────────────

/// Handle retourné par `initialiser()`.
///
/// Tant qu'il est en vie, le thread écrivain tourne. À sa destruction (fin de
/// `main` ou `drop` explicite), il envoie le signal d'arrêt au thread et attend
/// que les tampons soient vidés sur disque avant de rendre la main.
pub struct JournalisationHandle {
    arret:  SyncSender<Option<EntreeJournal>>,
    thread: Option<thread::JoinHandle<()>>,
}

impl Drop for JournalisationHandle {
    fn drop(&mut self) {
        let _ = self.arret.send(None);
        if let Some(t) = self.thread.take() {
            let _ = t.join();
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Point d'entrée public
// ─────────────────────────────────────────────────────────────────────────────

/// Initialise la journalisation. À appeler une seule fois au démarrage.
///
/// Retourne un [`JournalisationHandle`] à conserver jusqu'à la fin du programme.
/// Quand il est libéré, les tampons sont vidés sur disque avant que le processus
/// ne se termine — garantie d'arrêt propre en plus du gestionnaire de panique.
pub fn initialiser() -> Result<JournalisationHandle, Box<dyn std::error::Error>> {
    let repertoire = repertoire_logs();
    fs::create_dir_all(&repertoire)?;

    let ts = Local::now().format("%Y%m%d_%H%M%S");
    let boite_noire = File::create(format!("{}/boite_noire_{}.jsonl", repertoire, ts))?;
    let fil_de_vie  = File::create(format!("{}/fil_de_vie_{}.log",  repertoire, ts))?;

    let (tx, rx) = mpsc::sync_channel(CAPACITE_CANAL);
    let tx_panique = tx.clone();
    let tx_arret   = tx.clone();

    let handle = thread::Builder::new()
        .name("journalisation".into())
        .spawn(move || thread_ecrivain(rx, boite_noire, fil_de_vie))?;

    log::set_boxed_logger(Box::new(MoteurJournalisation { emetteur: tx }))?;
    log::set_max_level(LevelFilter::Debug);

    let gestionnaire_precedent = std::panic::take_hook();
    std::panic::set_hook(Box::new(move |info| {
        log::error!(target: "systeme", "Arrêt inattendu : {}", info);
        let _ = tx_panique.try_send(None);
        thread::sleep(Duration::from_millis(100));
        gestionnaire_precedent(info);
    }));

    Ok(JournalisationHandle { arret: tx_arret, thread: Some(handle) })
}
