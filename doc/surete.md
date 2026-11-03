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
  La MAÉ vol ne commande jamais la MAÉ sécurité ; celle-ci lit seulement si
  la mission est armée (`EtatVol::est_arme`), voir « Reprise en main ».

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
| `ArretUrgence`        | Le Pi ne demande plus de gaz (`arme = false`). Commande sol : définitif, vol plané jusqu'au sol. Reprise RC : le temps de la reprise. Voir « Reprise en main par l'opérateur » |

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
| ESP32 en mode manuel après passage en autopilote armé | le pilote RC reprend la main (voir `mode_manuel_rc`) | `ArretUrgence` |

Note : la perte GPS en phase de décollage ou atterrissage a une criticité
différente qu'en croisière. La politique exacte (ex. loiter) est à affiner
par la couche mission qui consommera `EtatSecurite` — pas encore implémentée.

---

## Reprise en main par l'opérateur

L'interrupteur d'arbitrage (SWA) est le levier de sécurité de l'opérateur :
il doit pouvoir reprendre la main à tout moment, et cette commande prime
sur tout le logiciel.

**La sécurité repose sur l'ESP32, jamais sur le seul Pi.** L'arbitrage est
local à l'ESP32 et fonctionne quel que soit l'état du Pi. L'ESP32 force aussi
les gaz à zéro en autopilote si le Pi ne les autorise pas ou reste muet plus
de 2 s. `ArretUrgence` est un état logiciel du Pi : il garde la mission et la
télémétrie cohérentes avec la prise de main, il ne la sécurise pas.

Le levier rend la main sur toutes les commandes, **gaz inclus et sans verrou** :
on attend du manuel qu'il réponde tout de suite, et un évitement d'obstacle
peut exiger de la puissance. Le temps d'actionner le levier est déjà critique,
aucun geste supplémentaire n'est ajouté. L'ESC exigeant des gaz à zéro pour
s'armer, le manche de gaz est naturellement en bas au début du processus.

L'opérateur n'est pas forcément un pilote expérimenté. Il peut reprendre la
main pour une raison précise (éviter un obstacle, par exemple), puis rendre
la main à l'autopilote pour qu'il gère l'atterrissage plus finement qu'il ne
le ferait lui-même. L'autopilote a alors besoin de gaz fonctionnels.

**Remettre l'autopilote en marche est donc un acte conscient, qui signifie
« autopilote gaz inclus ».** Aucune confirmation supplémentaire n'est demandée.

**L'arrêt d'urgence n'a de sens qu'une fois armé.** Tout ce qui précède relève
du contrôle : avant l'armement, comme après le désarmement, SWA est libre et
ses bascules n'ont aucun effet sur l'état sécurité. Le tableau ci-dessous vaut
après un passage en autopilote armé.

| Étape | Comportement |
|-------|--------------|
| Passage en manuel | L'ESP32 applique les canaux RC, gaz compris (le manche CH3) ; le Pi n'a plus d'autorité. Côté Pi : `ArretUrgence`, `arme = false` |
| En vol | La MAÉ vol est forcée en `Approche` ; elle y reste après le retour en autopilote : pas de reprise de la mission, l'objectif devient l'atterrissage |
| Au sol | La MAÉ vol ne change pas d'état et le plan de vol continue de se dérouler, chronomètre compris ; le retour en autopilote reprend la mission là où elle en est. Le décollage n'est pas bloqué : l'enjeu est moindre, l'opérateur reste responsable, et vérifier le levier avant de décoller est un réflexe sain à ne pas dissuader |
| Retour en autopilote | Les gouvernes repassent immédiatement sous le contrôle du Pi. Après 1 s (10 cycles sains), `ArretUrgence` se lève et les gaz reviennent à l'autopilote |
| Suite du vol | L'état sécurité reste plafonné à `AlerteMineure` jusqu'à la relance du programme : l'incident reste visible |

Ces 1 s évitent qu'un contact bref de l'interrupteur rende les gaz. Elles ne
remplacent pas la décision de l'opérateur.

**À ne pas confondre avec l'arrêt d'urgence commandé depuis le sol** (console
aujourd'hui, LoRa à terme). Celui-ci ne passe pas par l'arbitrage : l'opérateur
ne dispose d'aucun geste pour le révoquer, il ne se lève donc jamais seul et
reste actif jusqu'à la relance du programme.

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
