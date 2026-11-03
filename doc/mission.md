# /doc/mission.md
# Mission et machine à états vol — AirHaum II

## Rôle de la couche mission

La couche mission orchestre les comportements de haut niveau de l'appareil.
Elle consomme l'état estimé (attitude, position, vitesse) et les états de
sécurité pour piloter les contrôleurs de vol via des consignes.

Implémentation : `src/mission/etat_machine.rs`  
Tâche associée : `src/taches/taches_mission.rs`  
Fréquence d'exécution : 5–10 Hz (voir `doc/gestion_du_temps.md`)

---

## Machine à états vol

### Principes

- Un seul état actif à la fois.
- Les transitions sont déclenchées par des **événements** (commandes sol,
  seuils capteurs, conditions temporelles), jamais par scrutation directe.
- La FSM sécurité est **prioritaire** : elle peut forcer une transition vers
  `FailSafe` ou `AtterrissageUrgence` depuis n'importe quel état vol.
- La FSM vol publie son état courant via un `watch::Sender<EtatVol>` consommé
  par la télémétrie et la couche sécurité.

---

### États

#### Pré-vol

| État              | Rôle |
|-------------------|------|
| `Boot`            | Démarrage système, initialisation des tâches Tokio |
| `Initialisation`  | Démarrage des drivers, configuration des capteurs |
| `AutoTest`        | Vérification fonctionnelle : tous capteurs opérationnels, calibrations valides |
| `AttenteArmement` | Système prêt, en attente de la commande d'armement sol |

Condition de sortie de `AutoTest` → `AttenteArmement` : tous les capteurs
sont en état `Opérationnel` au sens de `EtatCapteur` (voir `doc/initialisation.md`).

---

### Check-liste d'armement

La check-liste d'armement est la **porte de transition** entre `AttenteArmement`
et `Armement`. Elle est présentée à l'opérateur sous forme d'une console. L'armement ne peut être commandé
que si tous les items **bloquants** sont au vert ; les items **avertissements**
peuvent être acquittés explicitement par l'opérateur.


#### Items bloquants

| Item | Source | Condition |
|------|--------|-----------|
| Calibration gyroscope | `systeme/calibration` | Valide et non expirée |
| Calibration accéléromètre | `systeme/calibration` | Valide et non expirée |
| Calibration baromètre | `systeme/calibration` | Valide et non expirée |
| Estimation active | `taches/taches_estimation` | EKF initialisé, attitude publiée |
| Plan de vol chargé | `mission/plan_vol` | Au moins un segment défini — voir `doc/plan_de_vol.md` |
| Sûreté nominale | `surete/` | FSM sécurité en état `Normal` |
| Liaison ESP32 + RC | `hal/uart` | Port série opérationnel, canal RC valide |
| Vérification pré-vol | `mission/armement` | Séquence guidée sur chaque gouverne, SWA en autopilote et système non armé, chaque position confirmée explicitement par l'opérateur |


#### Items avertissements (acquittables)

| Item | Source | Condition nominale |
|------|--------|--------------------|
| GPS fix 3D | `taches/taches_gps` | Fix 3D, ≥ 6 satellites, précision H ≤ 5 m |
| Calibration magnétomètre | `systeme/calibration` | Valide et non expirée |

Le GPS est acquittable pour permettre les vols de test en
mode attitude sans navigation autonome. La calibration magnétomètre est
acquittable car le lacet est estimé par gyro seul en l'absence de magnétomètre
fiable ; le dérapage accumulé reste acceptable pour la phase de test.


#### Implémentation

La check-liste est évaluée en temps réel depuis les canaux `watch` existants :
aucune communication supplémentaire inter-tâches n'est requise. Elle est
réévaluée à chaque rafraîchissement de l'affichage et reflète l'état
courant du système.

La commande d'armement n'est rendue disponible que lorsque tous les items
bloquants sont satisfaits et que les avertissements actifs ont été acquittés.

#### Chargement du plan de vol (`console_armement.rs`)

Item interactif, comme la vérification pré-vol servos (touche `p`) : la
touche `v` (plan de **v**ol) affiche la liste numérotée des fichiers `.toml`
du dossier plans de vol ; l'opérateur saisit un chiffre, le fichier
correspondant est chargé via `PlanDeVol::charger` et le résultat retourné
à la check-liste (✓ avec le nom du fichier en détail, ou ✗ avec le message
d'erreur — TOML malformé ou plan sans segment).

Dossier résolu comme pour les logs (`AIRHAUM_LOGS_DIR`,
`doc/journalisation.md`) : `AIRHAUM_PLANS_DIR` si définie, sinon
`/home/airhaum/plans_de_vol` (`mission/plan_vol::REPERTOIRE_PLANS_DE_VOL_DEFAUT`).

Un chargement réussi dépose aussi le plan dans `HandlesMission::tx_plan` —
voir « Injection du plan de vol » plus bas — en plus de l'état local
affiché par la check-liste.

#### Actions armer/lancer/désarmer (`console_armement.rs`)

Ces trois actions sont déclenchées **depuis l'écran de check-liste
lui-même**, pas depuis le menu de test/débogage (`console_test.rs`) : ce
dernier reste un outil de diagnostic, l'opérateur n'a normalement pas
besoin d'y passer pour mener une mission. Elles apparaissent
conditionnellement, selon l'état vol courant (`HandlesMission::rx_etat_vol`,
relu à chaque rafraîchissement de l'écran) :

| Touche | Commande envoyée | Condition d'affichage |
|--------|-------------------|------------------------|
| `a` — Armer | `CommandeVol::Armer` | `AttenteArmement`, tous les bloquants au vert et avertissements acquittés |
| `l` — Lancer la mission | `CommandeVol::ArmementValide` | `Armement` |
| `d` — Désarmer | `CommandeVol::Desarmer` puis `CommandeVol::ActionneursDesarmes` | `RoulageAuSol` (seul état où `Desarmer` est une transition valide, voir `etat_machine.rs`) |

`ActionneursDesarmes` est documentée comme émise par un module système une
fois les actionneurs confirmés désactivés — ce module n'existe pas encore
en v1, donc la console l'envoie directement à la suite de `Desarmer`
(simplification acceptée, à revoir si une vraie confirmation actionneurs
est implémentée).

Une fois `SauvegardeDonnées` atteint, la check-liste affiche un message de
fin et **met fin au programme** — décision actée : un lancement de
programme correspond à une mission (enchaîner plusieurs missions dans un
même lancement est un sujet séparé, non traité ici, voir
`doc/suivi_projet.md`).

Après l'envoi d'une commande, un court délai (jusqu'à 500 ms) est observé
avant le prochain redessin, le temps que la tâche mission (cycle 10 Hz)
la traite — voir « Boucle, ordre des opérations » ci-dessous.

---

#### Décollage

| État              | Rôle |
|-------------------|------|
| `Armement`        | Activation des actionneurs, vérification finale pré-vol |
| `RoulageAuSol`    | Déplacement sol à faible vitesse (ex. mise en position en bout de piste) |
| `CourseDecollage` | Pleine puissance, accélération sur piste |
| `Montee`          | Avion en l'air, montée vers l'altitude de croisière |

Note : la distinction `RoulageAuSol` / `CourseDecollage` permet d'envisager
un roulage autonome jusqu'en bout de piste avant mise en puissance.
La rotation n'est pas un état distinct — c'est une logique interne au
contrôleur d'attitude déclenchée par la vitesse sol.

Note abandon : `CourseDecollage` peut revenir à `RoulageAuSol` en cas
d'abandon normal de la course (décision mission/opérateur, vitesse jugée
insuffisante) — simple réduction des gaz, pas de nouvel état de
décélération puisque l'avion n'est pas freiné. Distinct d'un abandon en
urgence (voir transitions d'urgence ci-dessous).


#### Croisière

| État               | Rôle |
|--------------------|------|
| `VolLigneDroite`   | Cap constant, altitude constante, sans waypoint actif |
| `Navigation`       | Suivi de waypoints, corrections de cap automatiques |
| `AttenteEnCercle`  | Loiter : maintien de position en orbite (attente atterrissage, perte liaison…) |


#### Atterrissage

| État           | Rôle |
|----------------|------|
| `Approche`     | Descente vers l'axe d'atterrissage |
| `Final`        | Descente stabilisée sur l'axe de piste |
| `Arrondi`      | Flare : réduction puissance, assiette cabrée, déclenché par le VL53L0X (hauteur sol < seuil) |
| `RoulageAusol`  | Course à l'atterrissage et déplacements |

Note VL53L0X : portée utile ~2 m, suffisante pour déclencher le flare.
La phase `Arrondi` est l'état le plus critique de la FSM.


#### Fin de mission

| État               | Rôle |
|--------------------|------|
| `Desarmement`      | Désactivation des actionneurs |
| `SauvegardeDonnees`| Clôture et flush de la boîte noire |

---

### Diagramme de transitions (chemin nominal)

```
Boot → Initialisation → AutotTest → AttenteArmement
                                         │ commande armement
                                      Armement
                                         │
                                    RoulageAuSol ◄──────────┐
                                         │ position atteinte│ abandon course
                                         ▼                  │ (vitesse insuff. /
                                  CourseDecollage ───────────┘  décision opérateur)
                                         │ vitesse > V_rot
                                       Montee
                                         │ altitude cible
                          ┌──────────────┼──────────────┐
                          │              │               │
                  VolLigneDroite    Navigation    AttenteEnCercle
                          │              │               │
                          └──────────────┴───────────────┘
                                         │ commande atterrissage
                                      Approche
                                         │
                                       Final
                                         │ hauteur < seuil VL53
                                      Arrondi
                                         │ contact sol
                                    RoulageAuSol
                                         │ vitesse ~ 0
                                    Desarmement
                                         │
                                  SauvegardeDonnees
```

### Transitions d'urgence (depuis tout état vol)

```
[Tout état] ──(FSM sécurité : alertdurgence)──▶ CoupureDuMoteur
[Tout état] ──(FSM sécurité : alerte majeure)──▶ AtterrissageUrgence
[Tout état] ──(FSM sécurité : failsafe)─────────▶ FailSafe (loiter puis approche)
```

Ces transitions sont sous le contrôle exclusif de la FSM sécurité
(voir `doc/surete.md`).

---

## Tâche mission — `taches_mission.rs`

Câble ensemble la MAÉ vol (`etat_machine.rs`) et l'exécution du plan de vol
(`plan_vol.rs`, `doc/plan_de_vol.md`) — logique pure toutes les deux,
aucun I/O direct dans ce module-là. Fréquence : 10 Hz, même principe que
`taches_surete.rs`.

### Boucle, ordre des opérations

À chaque cycle, dans cet ordre :

1. **`forcer_urgence`** sur le dernier `EtatSecurite` reçu (`HandlesSurete::rx_securite`)
   — priorité absolue, comme documenté dans `etat_machine.rs`.
2. **Commandes console** : vidange du canal `mpsc<CommandeVol>` (voir
   ci-dessous), chaque commande passée à `traiter_commande`.
3. **`tick(&ContexteVol)`** — voir origine des champs ci-dessous.
4. **Publication de `EtatVol`** sur un nouveau canal `watch` (voir
   « Diffusion de l'état vol »).
5. **`PhaseControle`** (`taches_controle.rs`) dérivée de l'état courant via
   `EtatVol::est_en_vol()` — déjà l'API qui distingue exactement les deux
   groupes voulus (`Vol` si vrai, `Sol` sinon), pas de nouvelle logique à
   dupliquer. Republiée chaque cycle sur `tx_phase`.
6. **Armement (`tx_arme`)** dérivé de l'état courant — voir « Armement
   dérivé de l'état vol » ci-dessous. Republié chaque cycle.
7. **Exécution du plan de vol** (avancement, publication de
   `ConsigneDirectionSol`/`ConsigneGaz`) — uniquement si l'état courant
   est `RoulageAuSol` ou `CourseDecollage` ; en dehors de ces deux états
   (donc avant l'action « lancer la mission », voir plus bas), le plan
   peut encore être chargé ou remplacé (voir « Injection du plan de vol »).

### Origine des champs de `ContexteVol`

| Champ | Source |
|-------|--------|
| `vitesse_sol_ms` | Norme **horizontale** de `EtatNavigation.vitesse_ned` : `sqrt(x² + y²)` — x = vitesse Nord, y = vitesse Est (convention NED, voir `estimation/ekf_navigation.rs`). Le vecteur complet (`Vector3::norme()`) inclurait la composante Down (vitesse verticale/taux de montée), ce qui fausserait la vitesse *sol* utilisée pour la rotation au décollage. |
| `altitude_m` | `AltitudeFusionnee.altitude_m` |
| `hauteur_sol_mm` | `AltitudeFusionnee.hauteur_sol_mm` — **correction de type** : `ContexteVol.hauteur_sol_mm` était `Option<u16>`, incohérent avec `AltitudeFusionnee.hauteur_sol_mm: Option<u32>` ; aligné sur `u32` (`HAUTEUR_FLARE_MM` aussi migrée en `u32`), aucune perte, juste une incohérence de type non détectée jusqu'ici faute de branchement réel. |
| `altitude_cible_m` | Configuration de mission — reste hors périmètre de cette brique (valeur fixe/`TODO` acceptée pour l'instant, aucun essai courant n'en dépend). |
| `estimation_prete` | Première publication de `EtatNavigation` par l'estimation. Ne dépend pas du GPS : celui-ci reste un avertissement acquittable de la check-liste, armer sans fix est possible (pas de vitesse sol, donc pas de transition `CourseDecollage → Montee` ; géofence inactive). |

### Armement dérivé de l'état vol

`tx_arme` n'est piloté par aucune commande directe : c'est une fonction
pure de `EtatVol` **et** de `EtatSecurite` (`doit_etre_arme`), republiée
chaque cycle. La tâche publie aussi `tx_mission_armee` (`EtatVol::est_arme`,
sans la sécurité), lu par `taches_surete` : voir `doc/surete.md`, « Reprise en
main par l'opérateur ».

| Plage d'états | `arme` |
|---------------|--------|
| `Armement` → `Arrondi` inclus (toute la séquence décollage/croisière/atterrissage jusqu'au flare) | `true` |
| Tout le reste (`Boot`…`AttenteArmement`, `RoulageAuSol` *post-atterrissage*, `Desarmement`, `SauvegardeDonnees`) | `false` |
| **Quel que soit l'état vol** : `EtatSecurite::ArretUrgence` ou `AtterrissageUrgence` | `false` |

`Armement` est déjà armé (« Activation des actionneurs », voir table des
états ci-dessus) ; `Desarmement` coupe immédiatement, sans délai —
cohérent avec `doc/plan_essais.md` E0.8(b). La défense en profondeur du
mixeur (`doc/controle.md`) reste en place indépendamment de cette
dérivation.

Le dernier cas (urgence sécurité) existe pour couvrir E0.8(a) : `forcer_urgence`
(`etat_machine.rs`) n'agit que sur les états **en vol**
(`EtatVol::est_en_vol()`) — un arrêt d'urgence déclenché depuis la console
pendant un point fixe moteur (`RoulageAuSol`) ne change donc rien à la
MAÉ vol elle-même. Sans ce garde-fou dans `doit_etre_arme`, `tx_arme`
resterait à `true` (donc le mixeur laisserait passer la consigne gaz du
plan de vol) tant que l'appareil reste au sol. La sécurité prime : la
levée de l'urgence réarme automatiquement si l'état vol le justifie encore
(pas de mémoire d'incident ici, voir `MaeSecurite` pour la nuance
correspondante côté sécurité).

### Canal de commande console — `mpsc<CommandeVol>`

`CommandeVol` documente deux origines (« Sol » via LoRa, « Système ») —
aucune des deux n'existe encore matériellement (`communications/commandes.rs`
est `A FAIRE`). En v1, la seule origine réelle est la **console locale**
(SSH/Wifi, voir mémoire projet), sur le même principe que
`arret_urgence_console` déjà ajouté en appoint dans `taches_surete.rs`.

`taches_mission.rs` possède le côté récepteur (`mpsc::Receiver<CommandeVol>`) ;
la console (`console_armement.rs`) obtient un clone du côté émetteur pour
envoyer les commandes opérateur. Canal `mpsc` (pas `watch`) : les
commandes sont des événements discrets à traiter un par un, pas une
valeur courante — même choix que `rx_imu` (`taches_estimation.rs`).

**Deux actions opérateur distinctes**, confirmées par le vocabulaire déjà
utilisé dans `doc/plan_essais.md` (E0.6, E0.7 : *« armer, lancer une
mission »*) :

- **Armer** (`CommandeVol::Armer`) : envoyée une fois la checklist
  d'armement complète (`AttenteArmement → Armement`).
- **Lancer la mission** (`CommandeVol::ArmementValide`) : confirmation
  opérateur *séparée*, une fois en état `Armement` (`Armement →
  RoulageAuSol`). Le commentaire d'`ArmementValide` dans `etat_machine.rs`
  la classait comme « Système » — c'est en réalité une action opérateur
  explicite via la console en v1, corrigé en conséquence. C'est cette
  transition qui amène l'état courant à `RoulageAuSol`, la condition dont
  dépend l'exécution du plan de vol (étape 7 ci-dessus).
- **Désarmer** (`CommandeVol::Desarmer`, suivie immédiatement de
  `CommandeVol::ActionneursDesarmes`) : voir « Actions armer/lancer/
  désarmer » ci-dessus pour le détail et la justification du chaînage.

Ces trois actions sont déclenchées depuis la check-liste d'armement
(`console_armement.rs`), pas depuis le menu de test.

### Injection du plan de vol

`lancer_mission` ne prend pas de `PlanDeVol` en paramètre : la tâche est
lancée tôt (dès que `rx_securite`/`rx_navigation`/`rx_altitude` existent,
juste après `taches_surete::lancer_surete` dans `main.rs`), *avant* que
l'opérateur ait choisi un fichier dans la check-liste — pas d'attente
artificielle ajoutée, drivers/capteurs continuent de s'initialiser en
parallèle comme aujourd'hui.

À la place, `HandlesMission::tx_plan` (`watch::Sender<Option<PlanDeVol>>`)
sert de dépôt : initialisé à `None`, la console y écrit le plan chargé une
fois un fichier sélectionné (`v`, voir « Chargement du plan de vol »
ci-dessus). Tant que l'état courant n'exécute pas encore le plan
(`RoulageAuSol`/`CourseDecollage`, étape 7 ci-dessus), la tâche relit ce
dépôt à chaque cycle — l'opérateur peut donc changer de fichier avant
l'armement. Une fois l'exécution commencée, le plan est figé (plus de
relecture de `tx_plan`) : pas de changement de plan en cours de roulage.

### Diffusion de l'état vol

Aucun canal n'exposait `EtatVol` avant cette brique. Nouveau
`watch::Sender<EtatVol>`/`Receiver<EtatVol>`, possédé par
`taches_mission.rs` — même principe que `tx_securite` dans
`taches_surete.rs`. Consommé par la télémétrie (à venir) et les tests.

---

## Modules associés

| Fichier                      | Rôle |
|------------------------------|------|
| `mission/etat_machine.rs`    | Enum `EtatVol`, logique de transition |
| `mission/plan_vol.rs`        | Segments et conditions de fin — voir `doc/plan_de_vol.md` |
| `mission/armement.rs`        | Séquence de vérification pré-vol, activation et validation des servos |
| `mission/decollage.rs`       | Comportements Armement → Montée |
| `mission/atterrissage.rs`    | Comportements Approche → LandingRoll |
| `mission/navigation.rs`      | Suivi de waypoints, loiter (phase Vol — E5+) |
| `taches/taches_mission.rs`   | Tâche Tokio, boucle FSM |

`geofence.rs` et `failsafe.rs` ont été abandonnés (vestiges d'une itération
antérieure) : déjà couverts respectivement par `surete/limites.rs`
(géofence avec hystérésis) et `urgences.rs::forcer_urgence` (failsafe) —
voir `doc/surete.md`.
