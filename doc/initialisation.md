// doc/initialisation.md 


# Initialisation des capteurs – AirHaum II

## Principe

Le système ne fait **aucune hypothèse** sur l’état réel du matériel.
Chaque capteur est responsable de détecter sa propre situation.

---

## Logique d’initialisation

Au démarrage du programme, chaque driver applique la logique suivante :

1. **Observation de l’état matériel**
   - présence du capteur
   - cohérence de registres sentinelles
   - activité des données (FIFO, variations, fréquence)

2. **Choix automatique de l’action**
   - si une configuration valide est détectée :
     → reprise rapide (purge des buffers, resynchronisation)
     objectif en cas de crash logiciel en vol pouvoir relancer le programme 
     dans un délai bref et éviter un crash de l'avion
   - si l’état est incohérent :
     → mise en état dégadé du capteur qui peut tourner mais empechera d'armer
     l'avion. Cas typique au sol en attende d'une calibratoin complète 

Cette décision est **entièrement locale au driver**.

---

## Logique calibration

**Calibration des capteurs:**
 - Quand elle est nécessaire, se fait au sol, pré-vol.
 - Elle est stockée en flash avec timestamp unix en seconde.
 - Validité: jusqu'au prochain désarmement, délai spécifique
   au capteur ou mise hors tension

**Condition d'armement** : tous capteurs en état Opérationnel avec calibrations
 valides.
**Redémarrage en vol** : réutilisation des calibrations stockées. Aucune
 recalibration en vol, pas de calibration automatique au démarrage.

---

## États des capteurs

Chaque capteur évolue selon les états suivants :

- **Inconnu** : état par défaut au démarrage ou perte de contrôle
- **NonConfiguré** : capteur présent mais non configuré
- **Configuré** : registres valides, capteur actif, données non encore garanties
- **Opérationnel** : capteur calibré, données cohérentes et utilisables
- **Dégradé** : capteur fonctionnel mais données partielles ou douteuses

Le système consomme uniquement l’état exposé par le driver, sans supposition implicite.


Inconnu → NonConfiguré → Configuré → Opérationnel
   ↑           ↓             ↓            ↓
   └──────── Dégradé ←───────┘────────────┘


**Dégradation** : tout état peut basculer vers Dégradé, qui force un 
retour à Inconnu pour réinitialisation complète.

Le passage d'un ou plusieurs capteurs en mode dégradé sera logé et remonté au controle
de mission pour décision. 

---

## Vérification de santé & timeout

Dans les cas suivant tout écart est signalé sans bloquer l’exécution et peut déclencher
les mécanismes de sûreté:

**Vérification des données** Une vérification continue de la cohérence des données est
 effectuée en fonctionnement normal. 
**Timeout de sécurité** : Délai anormal entre Inconnu et Configuré (A définir) 
   
---

## Objectifs

Cette approche vise :
- un redémarrage logiciel rapide, y compris en vol
- un comportement déterministe et testable
- une séparation claire des responsabilités
- une robustesse adaptée à un environnement non temps réel
---


