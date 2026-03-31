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

