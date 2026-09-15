# Plan de vol — AirHaum II

## Rôle

Le plan de vol est le fichier édité par l'opérateur avant le lancement du
programme, qui paramètre le comportement attendu de l'appareil. Il ne
redéfinit jamais les états ni les transitions de la FSM vol
(`doc/mission.md`) — celle-ci reste fixée dans le code
(`mission/etat_machine.rs`). Le plan de vol fournit les valeurs consommées
à chaque état : cap et régime en roulage, durée d'une attente, etc.

Chargé au démarrage, condition l'item bloquant "Plan de vol chargé" de la
checklist d'armement (`doc/mission.md`).

Implémentation prévue : `src/mission/plan_vol.rs`

---

## Principe d'architecture

```
opérateur ──fichier TOML──▶ plan_vol.rs ──segment courant──▶ mission (FSM) ──consigne──▶ contrôle (doc/controle.md)
```

Le plan de vol est une liste ordonnée de **segments**. Un segment est un
comportement borné dans le temps : un type, des paramètres, et une
**condition de fin** qui déclenche le passage au segment suivant. La
mission avance dans la liste et route la consigne du segment courant vers
le(s) contrôleur(s) pertinent(s), exactement comme elle route déjà
`PhaseControle` (`doc/controle.md`) — le plan de vol ne parle jamais
directement au contrôle.

---

## Modèle de segment

| Type | Rôle | Paramètres | Consigne(s) produite(s) | Phase |
|------|------|------------|--------------------------|-------|
| `attente` | Immobilisation, gaz coupés | — | `ConsigneGaz{gaz: 0.0}` ; pas de nouvelle `ConsigneDirectionSol` publiée (le contrôleur direction sol continue de réguler sur le dernier cap reçu, hérité du segment précédent) | Sol |
| `roulage_cap` | Roulage à cap constant | `cap_deg`, `gaz` (0–1, passthrough comme `doc/controle.md`) | `ConsigneDirectionSol{cap_cible}` + `ConsigneGaz{gaz}` | Sol |

Phase Vol (ligne droite, virage, navigation, approche...) : hors périmètre
de cette itération. Même modèle segment/condition de fin, mais alimentera
`ConsigneRoulis`/`ConsigneTangage`/`ConsigneLacet` (`doc/controle.md`)
quand ces essais seront abordés (E3 et au-delà, `doc/plan_essais.md`).

Les segments sont scopés par phase de vol (ici uniquement `Sol` pour
l'instant) : un segment ne peut référencer que les consignes valides pour
la phase où il s'exécute — évite qu'un segment de roulage référence par
erreur une consigne de roulis. Le fichier reste toutefois une liste plate
pour cette itération (voir "Format du fichier" plus bas) : le
regroupement explicite par phase n'est introduit dans le format qu'au
moment où la phase Vol aura ses propres segments, pour ne pas
complexifier un format aujourd'hui mono-phase.

---

## Condition de fin

Structure composable dès la conception, même si seules les feuilles
simples sont utilisées dans cette itération :

- `duree_s` — délai fixe écoulé (ex. E0.6, temporisation immobile).
- `commande_operateur` — événement externe explicite (ex. E2.3 : décision
  d'abandon, ou l'attente en bout de piste avant course de décollage).
  Source de la commande : voir ci-dessous.
- `Et(...)` / `Ou(...)` — réservés pour des conditions combinées
  (ex. "durée max OU commande opérateur"), **non implémentés dans cette
  itération**, aucun cas d'usage identifié dans `doc/plan_essais.md` à ce
  stade. À réévaluer si un essai concret en a besoin.

D'autres feuilles (seuil de vitesse, distance parcourue) viendront
naturellement avec les essais qui en ont besoin (E2+) — non anticipées ici
pour rester au plus près du besoin actuel.

---

## Source de commande

Reprend le pattern déjà utilisé pour `Desarmer` (trois origines : sol
automatique, LoRa, RC/Arduino — voir mémoire projet). Pour cette
itération, une seule source est câblée :

- `console` — commande explicite depuis la console opérateur locale.

`lora` est anticipé comme variante future (`communications/commandes.rs`,
`A FAIRE`) mais non implémenté — l'abstraction évite d'avoir à retoucher
la boucle d'exécution des segments quand la LoRa arrivera.

---

## Décisions actées / différées

- **`autocontinue` (inspiration ArduPilot `MAV_CMD_DO_PAUSE_CONTINUE`)** :
  envisagé, écarté pour cette itération — aucun cas d'usage identifié
  dans `doc/plan_essais.md`. Propriété orthogonale à un segment, peu
  coûteuse à ajouter plus tard si un besoin apparaît.
- **Conditions combinées (`Et`/`Ou`)** : structure prévue mais non
  utilisée — voir "Condition de fin" ci-dessus.
- **Actions annexes façon `DO_*` d'ArduPilot** (ex. déclenchement d'un
  servo en cours de segment) : aucun besoin identifié dans le plan
  d'essais actuel (E0–E2), non modélisées.

---

## Format du fichier

TOML, cohérent avec `systeme/calibration.rs` et les `config/*.toml`
prévus dans `doc/architecture.md`.

Exemple couvrant E0.6 (mission minimale, attente immobile) suivi d'un
roulage à cap constant (E1.1) :

```toml
[[segment]]
type = "attente"
fin = { duree_s = 5 }

[[segment]]
type = "roulage_cap"
cap_deg = 0.0
gaz = 0.2
fin = { duree_s = 20 }
```

---

## Esquisse des types (pour la session de code)

Pas une implémentation — juste la forme envisagée, à affiner à
l'écriture :

```
enum TypeSegment {
    Attente,
    RoulageCap { cap_deg: f32, gaz: f32 },
}

enum SourceCommande {
    Console,
    // Lora — à venir
}

enum ConditionFin {
    Duree { secs: f32 },
    CommandeOperateur { source: SourceCommande },
    // Et(Vec<ConditionFin>), Ou(Vec<ConditionFin>) — réservé, non utilisé
}

struct Segment {
    type_segment: TypeSegment,
    fin: ConditionFin,
}
```

---

## Interface avec `taches_mission.rs`

Tranchée — voir `doc/mission.md`, section « Tâche mission —
`taches_mission.rs` » pour le détail complet (boucle, ordre des
opérations, canal de commande console). Résumé :

- `lancer_mission` ne reçoit pas de `PlanDeVol` directement : la tâche est
  lancée avant que l'opérateur en ait choisi un (voir « Injection du plan
  de vol », `doc/mission.md`) ; le chargement lui-même
  (`PlanDeVol::charger`/`depuis_toml`) reste à l'appelant
  (`console_armement.rs`), même principe que `lancer_controle` qui reçoit
  des contrôleurs déjà construits.
- Le plan de vol n'avance et ne publie ses consignes
  (`ConsigneDirectionSol`/`ConsigneGaz`) que lorsque l'état vol courant
  est `RoulageAuSol` ou `CourseDecollage`, atteint via l'action opérateur
  « lancer la mission » (`CommandeVol::ArmementValide`) — en dehors de ces
  deux états, ce code n'est simplement pas appelé.
- Fréquence : 10 Hz, alignée sur le reste de la tâche mission.

`mission/navigation.rs` reste le nom réservé aux segments de phase Vol à
base de waypoints (E5+, hors périmètre de `plan_vol.rs`) — articulation
déjà actée, voir `doc/architecture.md`.

---

## Chargement du fichier — `console_armement.rs`

Tranché — voir `doc/mission.md`, section « Chargement du plan de vol »,
pour le détail complet (touche `v`, liste numérotée, résultat reflété dans
l'item checklist). Résumé :

- Dossier résolu comme pour les logs (même principe que `AIRHAUM_LOGS_DIR`,
  `doc/journalisation.md`) : variable d'environnement `AIRHAUM_PLANS_DIR`,
  sinon `/home/airhaum/plans_de_vol` par défaut
  (`mission/plan_vol::REPERTOIRE_PLANS_DE_VOL_DEFAUT`).
- Sélection interactive dans la check-liste d'armement — pas d'argument CLI.
- L'item checklist "Plan de vol chargé" (`doc/mission.md`) reflète
  directement le résultat de `PlanDeVol::charger` (✓/✗ avec détail),
  y compris en cas de TOML malformé ou de plan sans segment.

Branchement de `taches_mission.rs` — tranché également, voir
`doc/mission.md`, « Actions armer/lancer/désarmer » : `lancer_mission` est
appelée dans `src/bin/airhaum_test/main.rs` (pas encore `airhaum-vol.rs`),
`Armer`/`ArmementValide`/`Desarmer` déclenchées depuis la check-liste
d'armement elle-même. `taches_controle.rs` est également branchée (voir
`doc/controle.md`) : les consignes du plan de vol atteignent réellement
l'Arduino. Reste hors périmètre : gains des contrôleurs et altitude cible
réels (valeurs provisoires, à régler à l'essai).

---

## Modules associés

| Fichier | Rôle |
|---------|------|
| `mission/plan_vol.rs` | Types `Segment`/`ConditionFin`, chargement et parsing TOML |
| `mission/etat_machine.rs` | FSM vol — son état (`RoulageAuSol`/`CourseDecollage`) conditionne l'exécution du plan de vol, voir `doc/mission.md` |
| `taches/taches_mission.rs` | Tâche Tokio, avance le plan de vol et route ses consignes selon l'état vol — voir `doc/mission.md` |
