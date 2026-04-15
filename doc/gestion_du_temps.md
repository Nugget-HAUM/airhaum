# Timing et contraintes temporelles – AirHaum II

## Objectif du document

Ce document décrit les hypothèses, contraintes et choix liés à la gestion du temps dans le système AirHaum II.

L'objectif n'est pas de garantir un temps réel dur, mais d'assurer un **comportement déterministe suffisant** pour un pilote automatique embarqué sous Linux non temps réel.

---

## Hypothèses générales

- Système exécuté sous **Linux (Armbian)**
- Pas de RTOS ni de noyau PREEMPT_RT
- Utilisation d'un runtime asynchrone (Tokio)
- Horodatage basé sur un temps **monotone**

---

## Modèle temporel

### 1. Temps capteur

- Horodatage effectué **au plus près du driver**
- Basé sur `CLOCK_MONOTONIC`
- Résolution microseconde

Objectif :
- préserver la cohérence des données capteurs
- permettre une estimation correcte des dérivées (dt)

---

### 2. Temps système

- Temps interne utilisé pour :
  - watchdog
  - supervision
  - journalisation

Tolérance :
- dérive faible acceptable
- priorité inférieure aux boucles de contrôle

---

### 3. Temps estimation

- Le pas de temps (`dt`) est **calculé**, jamais supposé constant
- Utilisation exclusive des horodatages capteurs

Objectif :
- robustesse face aux variations de fréquence
- absence d'hypothèse temps réel stricte

---

## Fréquences cibles

| Sous-système       | Fréquence cible | Tolérance |
|--------------------|-----------------|-----------|
| IMU                | 200 – 500 Hz    | ±20 %     |
| Baromètre          | 25 – 50 Hz      | ±30 %     |
| GPS                | 5 – 10 Hz       | ±1 Hz     |
| Estimation attitude| 200 Hz          | ±20 %     |
| Estimation position| 50 Hz           | ±30 %     |
| Contrôle attitude  | 200 Hz          | ±20 %     |
| Contrôle altitude  | 50 Hz           | ±30 %     |
| Mission            | 5 – 10 Hz       | ±50 %     |
| Communication      | 10 – 50 Hz      | ±50 %     |

Les fréquences reelles sont suivient au cours du fonctionnement 
et font l'objet d'alertes en cas de dérive. 

---

## Priorités logiques

Ordre de priorité conceptuel (du plus critique au moins critique) :

1. Arrêt d'urgence / sûreté
2. Lecture capteurs critiques
3. Estimation d'état
4. Contrôle
5. Mission
6. Communication
7. Services système

En cas de surcharge :
- la mission est dégradée
- la communication est ralentie
- la sûreté reste prioritaire

---

## Modèle de concurrence

La séparation entre lecture capteurs et traitement applicatif suit une frontière explicite :

**Couche hardware — threads synchrones**

Chaque capteur I2C tourne dans un `std::thread` dédié, en boucle à fréquence fixe cadencée par `thread::sleep`. Ce modèle est cohérent avec la nature synchrone du bus I2C sous Linux (appels `ioctl` bloquants). Les drivers restent purement synchrones.

- Horodatage effectué au niveau du driver, au plus près de la mesure
- Fréquence pilotée par le thread, indépendamment du runtime applicatif
- Bus I2C partagé via `Arc<std::sync::Mutex<>>` ; un timeout kernel (`I2C_TIMEOUT`) borne la durée maximale de toute transaction et garantit que le mutex est toujours libéré

**Couche applicative — runtime async Tokio**

L'estimation, le contrôle, la mission et la sûreté s'exécutent comme tâches Tokio. Ils consomment les mesures capteurs via des canaux décrits ci-dessous.

**Interface entre les deux couches — canaux différenciés par capteur**

Le type de canal est choisi en fonction des besoins du filtre de Kalman :

| Capteur      | Fréquence  | Type de canal         | Justification |
|--------------|------------|-----------------------|---------------|
| IMU          | 200–500 Hz | FIFO borné (20 slots) | Toutes les mesures sont nécessaires au Kalman : un horodatage manqué fausse le dt et dégrade l'intégration gyroscope |
| Baromètre    | ~43 Hz     | Valeur courante       | Le Kalman tourne plus vite que le baro ; la dernière valeur disponible suffit pour la correction |
| GPS          | 5–10 Hz    | Valeur courante       | Même raison ; fréquence très inférieure à la boucle d'estimation |

- **Canal FIFO borné** (`tokio::sync::mpsc`, capacité 20) : le thread capteur publie via `try_send()` (non-bloquant). Si le canal est plein, la mesure est abandonnée et un avertissement est émis — le thread capteur n'est jamais bloqué. La tâche Kalman vide le canal à chaque cycle et traite les mesures dans l'ordre avec leurs horodatages.

- **Canal à valeur courante** (`tokio::sync::watch`) : seule la dernière mesure est conservée. Côté thread : `tx.send(mesure)` (synchrone, non-bloquant). Côté async : `rx.borrow()` ou `rx.changed().await`.

La sûreté surveille la fraîcheur des données : l'absence de mise à jour au-delà d'un seuil déclenche une alerte indépendamment de la cause (timeout I2C, capteur défaillant, thread suspendu).

---

## Jitter et latence

- Le jitter est considéré acceptable tant que :
  - l'estimation reste stable
  - les contrôleurs ne divergent pas

- Toute latence excessive déclenche :
  - un signalement santé
  - un éventuel mode dégradé

---

## Limitations connues

- Pas de garantie temps réel stricte
- Sensible à la charge CPU globale
- Dépendance au scheduler Linux

Ces limitations sont acceptées et prises en compte par la conception logicielle.

---

## Évolutions possibles

- Migration vers PREEMPT_RT
- Séparation des cœurs CPU
- Boucles critiques isolées
- Ajout de mesures statistiques de jitter

---

## Conclusion

AirHaum II privilégie une approche **robuste et explicite du temps**, adaptée à un système autonome embarqué non certifié, plutôt qu'une illusion de temps réel dur.

