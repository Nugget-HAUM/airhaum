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
| `mission/decollage.rs`       | Comportements Armement → Montée |
| `mission/atterrissage.rs`    | Comportements Approche → LandingRoll |
| `mission/navigation.rs`      | Suivi de waypoints, loiter |
| `mission/geofence.rs`        | Limites géographiques |
| `mission/failsafe.rs`        | Comportements dégradés |
| `taches/taches_mission.rs`   | Tâche Tokio, boucle FSM |
