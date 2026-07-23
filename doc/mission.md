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
| Plan de vol chargé | `mission/navigation` | Au moins un waypoint défini |
| Sûreté nominale | `surete/` | FSM sécurité en état `Normal` |
| Liaison Arduino | `hal/uart` | Port série opérationnel, canal RC valide |
| Vérification pré-vol | `mission/armement` | Séquence d'activation unitaire des servos (débattements nominaux confirmés) |


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
                                    RoulageAuSol ──────┐
                                         │             │ position atteinte
                                  CourseDecollage       │
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

## Modules associés

| Fichier                      | Rôle |
|------------------------------|------|
| `mission/etat_machine.rs`    | Enum `EtatVol`, logique de transition |
| `mission/armement.rs`        | Séquence de vérification pré-vol, activation et validation des servos |
| `mission/decollage.rs`       | Comportements Armement → Montée |
| `mission/atterrissage.rs`    | Comportements Approche → LandingRoll |
| `mission/navigation.rs`      | Suivi de waypoints, loiter |
| `mission/geofence.rs`        | Limites géographiques |
| `mission/failsafe.rs`        | Comportements dégradés |
| `taches/taches_mission.rs`   | Tâche Tokio, boucle FSM |
