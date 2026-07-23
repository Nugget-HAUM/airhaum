# Contrôle — AirHaum II

## Rôle

La couche contrôle (couche 4) transforme les consignes de vol produites par
la couche mission en commandes envoyées aux servos. Elle consomme l'état
estimé (`EtatNavigation`, EKF navigation) et produit les `ConsignesServos`
envoyées à l'Arduino Nano via `taches_servo`.

Implémentation : `src/controle/`
Tâche associée : `src/taches/taches_controle.rs`

---

## Principe d'architecture

Communication strictement descendante mission → contrôle, sur le même
principe que sûreté → vol (`doc/surete.md`) : **le contrôle ne lit jamais
l'état de la MAE vol (`EtatVol`)**. C'est la mission qui décide de la phase
de vol — avec les critères qu'elle utilise déjà pour ses propres
transitions, ex. vitesse > V_rot — et qui route sa consigne vers le
contrôleur pertinent, via un canal `watch` dédié par contrôleur.

Chaque axe/contrôleur est indépendant, avec ses propres gains PID, sur le
modèle d'ArduPlane (`APM_Control` : `AP_RollController`, `AP_PitchController`,
`AP_YawController`, `AP_SteerController`) plutôt qu'une structure de
consigne englobante unique.

Chemin de données pour un axe :

```
mission ──ConsigneXxx (watch)──▶ controleur_xxx ──commande normalisée──▶ mixeur ──ConsignesServos (watch)──▶ taches_servo ──UART──▶ Nano
```

---

## Découpage en briques (ordre de développement)

Ordre calé sur la campagne d'essais progressive (roulage lent → vitesse
croissante → décollage → vol rasant en ligne droite → virages → ...) :

| # | Fait | Brique | Rôle | Dépend de |
|---|:---:|--------|------|-----------|
| 1 | [x] | `pid.rs` | PID générique réutilisé par tous les contrôleurs | — |
| 2 | [x] | `mixeur.rs` | Commandes normalisées → `ConsignesServos` (µs) | testable dès la brique 1 |
| 3 | [x] | `controleur_direction_sol.rs` | Cap au sol → commande direction (roulage, course décollage) | `pid.rs` |
| 4 | [x] | `taches_controle.rs` (minimal) | Branchement direction sol + mixeur : premier essai de roulage possible | 1-3 |
| 5 | [ ] | `controleur_roulis.rs` | Roulis → commande ailerons (cascade angle→taux) | `pid.rs` |
| 6 | [ ] | `controleur_tangage.rs` | Tangage → commande profondeur (cascade angle→taux) | `pid.rs` |
| 7 | [ ] | `controleur_lacet.rs` | Coordination de virage en vol → commande direction | `pid.rs` — après validation du vol rasant en ligne droite |
| — | [ ] | `controleur_altitude.rs`, `controleur_vitesse.rs`, `controleur_position.rs` | Hors périmètre tant que non nécessaires (navigation autonome) | — |

---

## PID générique — `pid.rs`

Un seul type `Pid`, réutilisé par tous les contrôleurs d'axe :

- Gains Kp / Ki / Kd configurables par instance.
- `dt` toujours mesuré (jamais supposé constant), cohérent avec
  `doc/gestion_du_temps.md`.
- Limitation de l'intégrale (anti-emballement) : clamp simple (borne
  `i_max`), pas de back-calculation — choix ArduPilot.
- Saturation de sortie configurable (bornes de la commande normalisée,
  ex. [-1, 1]).
- Pas de feedforward, pas de mise à l'échelle selon la vitesse air pour
  cette première étape — écart assumé par rapport à ArduPilot (pas de
  capteur de vitesse air au catalogue matériel). À revisiter si un tube
  Pitot est ajouté.

---

## Cascade angle → taux (contrôleurs de vol)

Comme ArduPlane (`AP_RollController`, `AP_PitchController`,
`AP_YawController`) : chaque contrôleur d'axe en vol combine deux boucles
internes, pas un simple PID sur l'angle :

1. **Boucle externe (P seul)** : erreur d'angle (consigne − mesure
   `EtatNavigation.attitude`) → taux angulaire cible.
2. **Boucle interne (PID complet, via `Pid`)** : erreur de taux (taux
   cible − taux mesuré) → commande normalisée.

**Source du taux mesuré** : gyro traité par `traitement_imu.rs` (repère
corps, biais statique retiré), corrigé du biais dynamique estimé par
l'EKF navigation (`EtatNavigation.biais_gyro`). Pas de nouveau flux
capteur.

### `controleur_roulis.rs`

Consigne : `ConsigneRoulis { roulis_cible: Angle }` (0 pour vol en ligne
droite, ailes à plat).
Sortie : commande ailerons normalisée [-1, 1].

### `controleur_tangage.rs`

Consigne : `ConsigneTangage { tangage_cible: Angle }`.
Sortie : commande profondeur normalisée [-1, 1].

### `controleur_lacet.rs`

Rôle : coordination de virage en vol (amortissement du dérapage), pas un
simple asservissement de cap — interface précise à définir lors de cette
brique (après validation du vol rasant en ligne droite, donc pas
prioritaire).
Sortie : commande direction normalisée [-1, 1].

---

## Contrôleur de direction sol — `controleur_direction_sol.rs`

Distinct du contrôleur de lacet en vol, comme `AP_SteerController` vs
`AP_YawController` sur ArduPlane — même gouverne physique (`direction`),
logique différente (maintien de cap au sol via frottement roues/gouverne,
pas de dynamique de vol).

Consigne : `ConsigneDirectionSol { cap_cible: Angle }`.
PID simple, pas de cascade angle→taux (pas de notion de "taux de lacet"
pertinente à l'arrêt ou à faible vitesse sol).
Sortie : commande direction normalisée [-1, 1].

Actif pendant `RoulageAuSol` et `CourseDecollage` (décision et routage
faits par la mission, voir ci-dessous — pas par le contrôle).

---

## Arbitrage direction sol / lacet vol

Le contrôle ne lit jamais `EtatVol` (couche 5) — cohérent avec la
communication strictement descendante mission → contrôle (même principe
que sûreté → vol, `doc/surete.md`).

C'est la **mission** qui décide de la phase (avec le critère
vitesse > V_rot déjà utilisé pour sa propre transition
`CourseDecollage → Montee`, voir `doc/mission.md`) et qui route sa
consigne vers le canal `watch` du contrôleur pertinent :

- `RoulageAuSol` / `CourseDecollage` → `ConsigneDirectionSol`
- `Montee` et au-delà → `ConsigneLacet` (+ `ConsigneRoulis` / `ConsigneTangage`)

Le VL53L0X garde son rôle déjà documenté (déclenchement de l'`Arrondi` à
l'atterrissage, `doc/mission.md`), sans lien avec cet arbitrage.

---

## Mixeur — `mixeur.rs`

Avion classique à 3 gouvernes indépendantes (pas de V-tail ni flying
wing) → mixage trivial 1:1 par axe, pas de couplage à gérer :

- commande ailerons normalisée → `ailerons` (µs)
- commande profondeur normalisée → `profondeur` (µs)
- commande direction normalisée (direction sol ou lacet vol, selon ce qui
  est actif) → `direction` (µs)
- conversion : `neutre (1500) + commande * demi_plage`, clamp [1000, 2000]

**Gaz** : hors PID pour l'instant. Consigne transmise telle quelle par la
mission (`ConsigneGaz`, passthrough direct), pas de régulation asservie
tant que `controleur_vitesse.rs` / `controleur_altitude.rs` n'existent
pas.

---

## Fréquences

Voir `doc/gestion_du_temps.md` : contrôle attitude 200 Hz (aligné
estimation attitude), contrôle altitude 50 Hz (futur, hors périmètre
actuel). Direction sol : fréquence à définir, probablement alignée sur
l'attitude (200 Hz) par simplicité d'implémentation partagée avec
`pid.rs`, à ajuster si besoin.

Les consignes servos restent envoyées à 50 Hz par `taches_servo` (canal
`watch`, seule la dernière valeur du contrôle est utilisée — le contrôle
tournant plus vite, aucune perte d'information).

---

## Campagne de validation progressive

Chaque brique est validée par des tests unitaires (données synthétiques,
comme `tests/capteurs_architecture.rs`) puis par un essai réel une fois
la mission et la sûreté prêtes — l'armement est de toute façon
conditionné aux prérequis sûreté (`doc/mission.md`) :

| Étape | Ce qui est testé | Contrôleur(s) validé(s) |
|-------|-------------------|--------------------------|
| 1 | Roulage à très faible vitesse, maintien de cap | `controleur_direction_sol` |
| 2 | Vitesse croissante jusqu'à l'approche de V_rot | `controleur_direction_sol` (robustesse) |
| 3 | Décollage, vol rasant en ligne droite (quelques cm du sol) | `controleur_roulis`, `controleur_tangage` |
| 4 | Virages en vol | `controleur_lacet` |
| ... | Navigation, altitude, position | contrôleurs futurs, hors périmètre actuel |

Progression jusqu'à ce que le comportement observé corresponde exactement
à l'attendu à chaque étape avant de passer à la suivante.

---

## Points ouverts / à trancher lors de l'implémentation

- Valeurs numériques des gains (Kp/Ki/Kd, bornes IMAX, gain P externe
  angle→taux) — à déterminer/régler à l'essai, aucune valeur a priori.
- Interface précise de `controleur_lacet.rs` (coordination de virage) —
  à définir lors de cette brique.
- Fréquence exacte de `controleur_direction_sol` — probablement 200 Hz
  par simplicité, à confirmer.
- Format exact de `ConsigneGaz` (pourcentage vs µs direct).
- **Gel des contrôleurs inactifs (`taches_controle.rs`)** : si
  `controleur_direction_sol` et `controleur_lacet` (ou plus généralement
  tout contrôleur non sélectionné par la phase mission courante)
  continuent de tourner en arrière-plan sans que leur sortie soit
  utilisée, leur terme intégral accumule une erreur jamais appliquée —
  au changement de phase, le contrôleur repart avec une intégrale déjà
  emballée et produit un à-coup. `taches_controle.rs` devra ne mettre à jour
  (intégrer) que le contrôleur dont le canal de consigne est
  actuellement alimenté par la mission, et geler les autres. À traiter
  lors de l'écriture de cette brique, pas avant.

---

## Modules associés

| Fichier                                  | Rôle |
|-------------------------------------------|------|
| `controle/pid.rs`                        | PID générique (limitation de l'intégrale) |
| `controle/controleur_direction_sol.rs`   | Cap au sol (roulage, course décollage) |
| `controle/controleur_roulis.rs`          | Roulis en vol, cascade angle→taux |
| `controle/controleur_tangage.rs`         | Tangage en vol, cascade angle→taux |
| `controle/controleur_lacet.rs`           | Coordination de virage en vol |
| `controle/mixeur.rs`                     | Commandes normalisées → `ConsignesServos` |
| `taches/taches_controle.rs`              | Tâche Tokio, boucle contrôle, branchement consignes/mixeur |
