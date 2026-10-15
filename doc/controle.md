# Contrôle — AirHaum II

## Rôle

La couche contrôle (couche 4) transforme les consignes de vol produites par
la couche mission en commandes envoyées aux servos. Elle consomme l'état
estimé (`EtatNavigation`, EKF navigation) et produit les `ConsignesServos`
envoyées au contrôleur servo ESP32 via `taches_servo`.

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
mission ──ConsigneXxx (watch)──▶ controleur_xxx ──commande normalisée──▶ mixeur ──ConsignesServos (watch)──▶ taches_servo ──UART──▶ ESP32
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
| 4 | [x] | `taches_controle.rs` | Branchement complet (direction sol, roulis, tangage, lacet, gaz) + mixeur, arbitrage `PhaseControle`. Branchée dans `airhaum_test/main.rs`, alimentée par `taches_mission.rs` (gains provisoires, non réglés à l'essai) | 1, 2, 3, 5, 6, 7 |
| 5 | [x] | `controleur_roulis.rs` | Roulis → commande ailerons (cascade angle→taux) | `pid.rs` |
| 6 | [x] | `controleur_tangage.rs` | Tangage → commande profondeur (cascade angle→taux) | `pid.rs` |
| 7 | [x] | `controleur_lacet.rs` | Coordination de virage en vol → commande direction | `pid.rs` — après validation du vol rasant en ligne droite |
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
capteur : publié comme dérivée de canaux déjà calculés par
`taches_estimation.rs` (`rx_taux_corps`, `watch::Receiver<Vector3>`, mêmes
axes x/y/z que le gyroscope : roulis/tangage/lacet).

### `controleur_roulis.rs`

Consigne : `ConsigneRoulis { roulis_cible: Angle }` (0 pour vol en ligne
droite, ailes à plat).
Sortie : commande ailerons normalisée [-1, 1].

### `controleur_tangage.rs`

Consigne : `ConsigneTangage { tangage_cible: Angle }`.
Sortie : commande profondeur normalisée [-1, 1].

---

## Contrôleur de lacet en vol — `controleur_lacet.rs`

Rôle : coordination de virage (amortissement du dérapage). **Pas de cascade
angle → taux** contrairement à `controleur_roulis.rs`/`controleur_tangage.rs` :
`EtatNavigation.attitude.lacet` est estimé par intégration gyro seule et
dérive sans borne sur la durée d'un vol (pas de magnétomètre ni de cap GPS
fusionnés) — l'asservir directement reviendrait à asservir sur du bruit.
Seul le **taux** de lacet (gyroscope, fiable à court terme) est utilisé,
via un PID simple.

Consigne : `ConsigneLacet { taux_lacet_cible: f32 }` (rad/s, pas un cap).
Sortie : commande direction normalisée [-1, 1].

**Interface tranchée lors de cette brique** : une coordination de virage
complète (taux cible = g·tan(roulis)/vitesse air) nécessiterait une vitesse
air fiable, absente du catalogue matériel — la vitesse sol GPS a été
écartée comme substitut (trop bruitée/en retard, diverge de la vitesse air
avec du vent). `taux_lacet_cible` est donc fourni tel quel par l'appelant :
`0.0` pour un amortissement pur du dérapage (valeur envisagée pour cette
itération), avec la possibilité d'un feedforward ultérieur si une vitesse
air fiable devient disponible (tube Pitot).

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

## Arbitrage direction sol / lacet vol — `PhaseControle`

Le contrôle ne lit jamais `EtatVol` (couche 5) — cohérent avec la
communication strictement descendante mission → contrôle (même principe
que sûreté → vol, `doc/surete.md`).

C'est la **mission** qui décide de la phase (avec le critère
vitesse > V_rot déjà utilisé pour sa propre transition
`CourseDecollage → Montee`, voir `doc/mission.md`) et qui la publie via
`PhaseControle` (`taches_controle.rs`), un type volontairement plus pauvre
qu'`EtatVol` — seules deux valeurs :

- `PhaseControle::Sol` (`RoulageAuSol` / `CourseDecollage`) → `controleur_direction_sol` actif.
- `PhaseControle::Vol` (`Montee` et au-delà) → `controleur_roulis` / `controleur_tangage` / `controleur_lacet` actifs.

La mission écrit la phase sur `tx_phase` **et** route sa consigne vers le
canal `watch` du (des) contrôleur(s) pertinent(s) ; `taches_controle.rs` ne
fait que lire `PhaseControle` pour savoir quel(s) contrôleur(s) mettre à
jour à chaque cycle.

**Gel des contrôleurs inactifs** : implémenté — seul(s) le(s) contrôleur(s)
actif(s) selon `PhaseControle` sont mis à jour (intégrés) ; les autres sont
gelés (pas d'appel à `mettre_a_jour`, donc pas d'accumulation d'intégrale
sur une erreur jamais appliquée). À la transition de phase, les
contrôleurs qui viennent de devenir inactifs sont réinitialisés
(`reinitialiser()`) pour repartir sans intégrale emballée la prochaine
fois qu'ils seront sélectionnés.

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

**Gaz — `ConsigneGaz { gaz: f32 }`** : hors PID, passthrough direct par la
mission, pas de régulation asservie tant que `controleur_vitesse.rs` /
`controleur_altitude.rs` n'existent pas. Normalisée `[0, 1]` comme les
autres commandes (plutôt qu'exprimée en µs) pour ne pas exposer les
bornes matérielles du servo à la mission ; conversion dans le mixeur :
`min (1000) + gaz * plage`, clamp [1000, 2000].

**Défense en profondeur** : si `!arme`, le mixeur force le gaz au minimum
quelle que soit `ConsigneGaz` — l'ESP32 applique déjà sa propre garde
sur `arme` en aval, mais le mixeur ne doit pas laisser fuiter une
consigne gaz non nulle restée dans le canal `watch` après un
désarmement (plan d'essais, E0.8 : coupure gaz sans délai).

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
- Fréquence exacte de `controleur_direction_sol` — probablement 200 Hz
  par simplicité, à confirmer.
---

## Modules associés

| Fichier                                  | Rôle |
|-------------------------------------------|------|
| `controle/pid.rs`                        | PID générique (limitation de l'intégrale) |
| `controle/controleur_direction_sol.rs`   | Cap au sol (roulage, course décollage) |
| `controle/controleur_roulis.rs`          | Roulis en vol, cascade angle→taux |
| `controle/controleur_tangage.rs`         | Tangage en vol, cascade angle→taux |
| `controle/controleur_lacet.rs`           | Coordination de virage en vol (amortissement du dérapage, PID de taux) |
| `controle/mixeur.rs`                     | Commandes normalisées + `ConsigneGaz` → `ConsignesServos`, défense en profondeur si non armé |
| `taches/taches_controle.rs`              | Tâche Tokio, boucle contrôle, `PhaseControle`, branchement consignes/mixeur, gel des contrôleurs inactifs |
| `taches/taches_estimation.rs`            | Publie aussi `rx_taux_corps` (`Vector3`, biais retiré) pour la boucle interne des cascades angle→taux |
