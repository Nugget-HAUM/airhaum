# Sûreté — AirHaum II

## Rôle

Le module sûreté est **prioritaire sur toute autre couche**.
Il surveille en permanence la santé du système et peut forcer des transitions
dans la MAÉ (Machine a état) vol indépendamment de son état courant.

Implémentation : `src/surete/`  
Tâche associée : `src/taches/taches_surete.rs`

---

## Principe d'architecture

La MAÉ sécurité est **orthogonale** à la MAÉ vol :

- Elle consomme l'état agrégé des capteurs, l'état de la liaison LoRa,
  la géofence et les mesures de santé système.
- Elle publie son état via un `watch::Sender<EtatSecurite>` lu par la MAÉ vol
  et la télémétrie.
- La communication est **unidirectionnelle** : sécurité → vol.
  La MAÉ vol ne commande jamais la MAÉ sécurité.

---

## Machine à états sécurité

### États

| État                  | Signification |
|-----------------------|---------------|
| `Normal`              | Tous les paramètres dans les limites nominales |
| `AlerteMineure`       | Anomalie détectée, vol maintenu, opérateur notifié |
| `AlerteMajeure`       | Anomalie sérieuse, comportement vol restreint |
| `FailSafe`            | Atterrissage au plus vite |
| `AtterrissageUrgence` | Atterrissage immédiat, priorité absolue |
| `ArretUrgence`        | Coupure immédiate du moteur, puis passage en AtterissageUrgence en planeur |

Tout état peut basculer vers `AtterrissageUrgence` sur condition critique.

---

## Sources de déclenchement

| Source                                    | Seuil / condition                                  | Effet minimal |
|--------------------------------------------|-----------------------------------------------------|---------------|
| Perte liaison LoRa                          | > délai configurable (non câblé — voir `EntreesSecurite::lora_perdue`) | `FailSafe` |
| Capteur dégradé (baro/télém./IMU/GPS)       | `EtatSante::Degrade` (voir `sante::pire_etat`)       | `AlerteMineure` |
| Capteur hors service / thread silencieux    | `EtatSante::HorsService` (péremption ou watchdog)    | `AlerteMajeure` |
| Géofence dépassée                           | position hors limites                                | `AtterrissageUrgence` |
| Batterie critique                           | tension < seuil configurable (non câblé — pas de capteur v1) | `AtterrissageUrgence` |
| Commande arrêt d'urgence sol                | console opérateur aujourd'hui (LoRa en seconde source à terme) | `ArretUrgence` |
| ESP32 en mode manuel après passage en autopilote | le pilote RC reprend la main (voir `mode_manuel_rc`) | `ArretUrgence` |

Note : la perte GPS en phase de décollage ou atterrissage a une criticité
différente qu'en croisière. La politique exacte (ex. loiter) est à affiner
par la couche mission qui consommera `EtatSecurite` — pas encore implémentée.

---

## Interaction avec la FSM vol

La FSM vol consulte `EtatSecurite` à chaque itération.
En cas d'état `FailSafe`, `AtterrissageUrgence` ou `ArretUrgence`, elle
abandonne son état courant et bascule sur le comportement d'urgence
correspondant.

La FSM vol **ne peut pas ignorer** un état sécurité critique.
Ce mécanisme est la seule voie de forçage externe des états vol.

---

## Modules associés

| Fichier                   | Rôle |
|---------------------------|------|
| `surete/armement.rs`      | Conditions nécessaires à l'armement |
| `surete/limites.rs`       | Géofence, enveloppe de vol |
| `surete/sante.rs`         | Surveillance capteurs, watchdog threads |
| `surete/urgences.rs`      | Gestion des situations critiques |
| `taches/taches_surete.rs` | Tâche Tokio, boucle FSM sécurité |
