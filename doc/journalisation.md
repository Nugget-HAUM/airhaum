# Journalisation — AirHaum II

## Rôle

La journalisation remplit deux fonctions distinctes qui justifient deux fichiers séparés :

- **Boîte noire** : archive complète pour l'analyse post-vol ou post-crash.
  Conçue pour être traitée par script, pas lue directement.
- **Fil de vie** : flux lisible en temps réel depuis un terminal séparé (`tail -f`).
  Conçu pour l'opérateur pendant les tests au sol et les vols de développement.

La journalisation ne doit **jamais ralentir** les threads de mesure. Le thread IMU
tourne à ~200 Hz ; toute I/O directe dans ce chemin est proscrite.

---


## Architecture

```
threads capteurs / estimation / mission / sûreté
          │  log::info!(…)  log::warn!(…)  log::error!(…)
          ▼
   canal mpsc non-bloquant   (émetteur cloneable, jamais bloquant)
          │
          ▼
   thread écrivain dédié
          ├──▶  boite_noire_YYYYMMDD_HHmmss.jsonl   (boîte noire)
          └──▶  fil_de_vie_YYYYMMDD_HHmmss.log             (fil de vie)
```


### Principe

- Façade : crate `log` (`log::info!`, `log::warn!`, `log::error!`, `log::debug!`).
  Les couches métier n'ont aucune dépendance au moteur.
- Moteur : implémenté dans `src/systeme/journalisation.rs`.
  Il enregistre les `log::Record` dans un canal `std::sync::mpsc` vers le thread
  écrivain. L'émetteur est non-bloquant : si le canal est plein, l'entrée est
  silencieusement abandonnée plutôt que de bloquer l'appelant.
- Un seul thread écrivain consomme le canal et écrit dans les deux fichiers.
  La mise en tampon est déléguée au système d'exploitation.


### Vidage

Le vidage explicite n'est déclenché que par le **gestionnaire de panique** : un gestionnaire Rust installé
au démarrage force le vidage des tampons avant de terminer le processus.
Cela garantit que les derniers enregistrements sont sur disque en cas de crash
logiciel. Une coupure d'alimentation peut entraîner la perte des dernières
secondes — compromis accepté.

---


## Boîte noire

**Répertoire** : `/home/airhaum/logs` par défaut, surchargeable via la variable
d'environnement `AIRHAUM_LOGS_DIR` (utile en développement, ce chemin
n'existant pas hors de la cible embarquée).  
**Fichier** : `<répertoire>/boite_noire_YYYYMMDD_HHmmss.jsonl`  
**Format** : JSON Lines — un objet JSON par ligne, parseable par `jq` ou Python.  
**Rétention** : toutes les sessions conservées (nettoyage manuel si nécessaire).


### Format d'un enregistrement

```json
{"ts":"2026-04-22T10:15:32.456","lvl":"INFO","cible":"mission","msg":"AttenteArmement → Armement"}
{"ts":"2026-04-22T10:15:33.100","lvl":"INFO","cible":"nav","r":-0.3,"t":1.2,"l":45.6,"pN":0.0,"pE":0.0,"pB":0.0,"vN":12.3,"vE":0.1,"vB":-0.2,"alt":150.3}
{"ts":"2026-04-22T10:15:34.891","lvl":"WARN","cible":"gps","msg":"Fix perdu — 4 sats < seuil 6"}
```

Les enregistrements de navigation utilisent des champs structurés (pas un champ `msg`)
pour faciliter l'import dans des outils d'analyse (pandas, gnuplot).


### Contenu et fréquences

| Source | Niveau | Fréquence | Contenu |
|--------|--------|-----------|---------|
| Drivers (init, erreurs, réinit) | INFO / WARN / ERROR | À chaque occurrence | Message texte |
| Machine à états vol | INFO | À chaque transition | États source et cible |
| Machine à états sécurité | WARN / ERROR | À chaque transition | États + cause |
| Navigation (attitude, position, vitesse) | INFO | **10 Hz** | Champs structurés |
| Santé capteurs (compteurs erreurs) | INFO | **1 Hz** | Compteurs |
| GPS (fix, satellites, précision) | INFO / WARN | À chaque changement + 1 Hz | Champs structurés |
| Calibration (chargement, expiration) | INFO / WARN | À chaque occurrence | Message texte |
| Armement / désarmement | INFO | À chaque occurrence | Message texte |
| DEBUG drivers (registres, valeurs) | DEBUG | Désactivé en vol | Message texte |

Les données IMU brutes (200 Hz) **ne sont jamais journalisées** — l'état estimé
à 10 Hz suffit pour reconstituer la dynamique de vol.

---


## Fil de vie

**Fichier** : `/home/airhaum/logs/fil_de_vie_YYYYMMDD_HHmmss.log`  
**Format** : texte brut, lisible directement avec `tail -f`.  
**Usage** : terminal secondaire pendant les tests, indépendant de la console de test.


### Format d'une ligne

```
[10:15:32] INFO   mission  AttenteArmement → Armement
[10:15:33] INFO   nav      r=+0.3° t=-1.2° l=+45.6°  alt=+150.3m  v=12.3m/s
[10:15:34] WARN   gps      Fix perdu — 4 sats < seuil 6
[10:15:34] ERROR  baro     Erreur I²C — réinit #3
```


### Contenu et fréquences

| Source | Fréquence | Justification |
|--------|-----------|---------------|
| WARN et ERROR (tous) | À chaque occurrence | Toujours visible immédiatement |
| Transitions MAÉ vol et sécurité | À chaque occurrence | Suivi de l'état système |
| Résumé navigation (attitude + altitude + vitesse) | **1 Hz** | Lisible par un humain |
| Santé capteurs | Sur dégradation uniquement | Pas de bruit en nominal |
| DEBUG | Jamais | Réservé à la boîte noire si activé |

Le fil de vie est volontairement **silencieux en nominal** : en vol stabilisé sans
anomalie, seule la ligne de navigation à 1 Hz défile. Les événements ressortent
clairement du flux.

---


## Implémentation

| Fichier | Rôle |
|---------|------|
| `src/systeme/journalisation.rs` | Trait `log::Log`, canal `std::sync::mpsc`, thread écrivain, gestionnaire de panique |


### Dépendances

```toml
log    = "0.4"   # façade standard — utilisée par tous les modules
chrono = "0.4"   # horodatage ISO 8601 des enregistrements
```


### Cibles de journalisation

Chaque `log::info!(target: "...", ...)` utilise une cible qui permet de filtrer
la boîte noire avec `jq` ou Python.

| Cible | Émetteur | Contenu |
|-------|----------|---------|
| `systeme` | `main.rs` | Démarrage, init I²C, erreurs fatales |
| `mission` | `main.rs` | Transitions MAÉ vol |
| `gps` | Thread GPS | Fix, satellites, disponibilité |
| `baro` | Thread baromètre | Pression de référence, anomalies |
| `calibration` | Console calibration | Résultats de calibration capteurs |
| `nav` | Tâche estimation | Attitude, position, vitesse — **10 Hz** (à venir) |
| `diag` | Menu `airhaum-test` | Résultats diagnostics et tests |

Exemple de filtrage boîte noire :
```bash
jq 'select(.cible == "mission")' boite_noire_*.jsonl
jq 'select(.lvl == "ERROR" or .lvl == "WARN")' boite_noire_*.jsonl
```

---


## Ouverture d'une session de monitoring

```bash
# Terminal 1 : console de test
cargo run --bin airhaum-test

# Terminal 2 : fil de vie en temps réel
tail -f /home/airhaum/logs/fil_de_vie_$(ls -t /home/airhaum/logs/fil_de_vie_*.log | head -1 | xargs basename)

# Ou plus simplement, le dernier fichier :
tail -f $(ls -t /home/airhaum/logs/fil_de_vie_*.log | head -1)
```

---


## Points résolus

- ✓ Fichier par session, pas de limite de rétention
- ✓ Deux fichiers : boîte noire (JSON Lines) + fil de vie (texte)
- ✓ Données IMU brutes exclues — état estimé à 10 Hz suffisant
- ✓ Vidage uniquement sur panique (compromis perf / sécurité)
- ✓ Canal non-bloquant : threads haute fréquence jamais bloqués
- ✓ Implémenté : `src/systeme/journalisation.rs` opérationnel, `log::*` actif dans `airhaum-test`




