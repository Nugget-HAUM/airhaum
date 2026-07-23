# Interface Pi / RC / Servos — AirHaum II

## Rôle

L'Arduino Nano est le **point d'intersection** entre trois systèmes :
- l'autopilote (Orange Pi) qui produit des consignes de vol,
- la télécommande RC qui permet la prise de main manuelle et l'arret d'urgence,
- les actionneurs (servos + variateur) qui agissent physiquement sur l'appareil.

Il est le **dernier maillon de sécurité matériel** : il garantit que les
actionneurs reçoivent toujours un signal valide quelle que soit l'origine
de la commande, et que le pilote peut physiquement reprendre la main à
tout moment.

Implémentation côté Pi : `src/drivers/controleur_servo.rs`  
Protocole Pi ↔ Arduino : binaire personnalisé — UART2 Pi (`/dev/ttyS2`) ↔ Nano D0/D1 via adaptateur de niveau 3,3V/5V  
- **Nano → Pi** (remontée) : TX matériel sur D1 à 57 600 bauds (implémenté).  
- **Pi → Nano** (consignes) : RX matériel sur D0 à 57 600 bauds (implémenté).  
Protocole RC → Arduino : PWM individuel sur 5 broches analogiques (A1–A5) via PCINT1


---

## Comportement au démarrage

À la mise sous tension, avant réception de toute trame Pi ou RC valide :

- Gaz maintenus à zéro (1 000 µs).
- Servos en position neutre (1 500 µs sur tous les canaux).
- Mode **manuel** par défaut (CH7/SWA sur A5 détermine le mode dès réception RC).

Le Nano reste dans cet état d'attente jusqu'à réception de trames valides
des deux sources. Le bit d'armement étant à 0 par définition, les gaz
restent bloqués jusqu'à armement explicite par le Pi.


---

## Arbitrage manuel / autopilote

L'arbitrage est **local à l'Arduino**, sans intervention du Pi.

Un canal RC dédié (CH7 — interrupteur SWA, broche A5) fait office d'interrupteur de mode :

| Position canal  | Mode actif | Source des consignes servos          |
|-----------------|------------|--------------------------------------|
| Bas (1 000 µs)  | Manuel     | Retransmission directe des canaux RC |
| Haut (2 000 µs) | Autopilote | Consignes reçues du Pi               |

La bascule est **instantanée**, sans transition progressive.
Aucune logique de reprise douce n'est prévue dans les premières versions.

Le Pi n'a pas connaissance en temps réel de l'état du mode actif via ce
canal — il reçois l'information via la remontée d'état Arduino (voir ci-dessous).


---

## Flux de données

### Pi → Arduino (prioritaire)

| Donnée          | Contenu                              | Fréquence |
|-----------------|--------------------------------------|-----------|
| Consignes servo | 4 canaux en µs + bit d'armement      | 50 Hz     |

Le Pi envoie ses consignes en permanence. Chaque trame valide reçue
réarme le chien de garde Arduino. L'Arduino applique ou ignore les
consignes selon le mode actif, mais continue de les recevoir.


### Arduino → Pi (remontée)

| Donnée          | Contenu                                       | Fréquence                   |
|-----------------|-----------------------------------------------|-----------------------------|
| Canaux RC       | Valeurs PWM µs des 5 canaux (A1–A5)          | 10 Hz                       |
| État mode actif | Manuel ou Autopilote                          | 10 Hz                       |
| État variateur  | Consigne gaz appliquée                        | 10 Hz                       |

La remontée RC est destinée à la **boîte noire** du Pi et à l'analyse
post-vol. Dans les premières versions elle n'est pas utilisée pour le contrôle en temps réel.


### RC → Arduino

Le récepteur RC (FS-IA10B) transmet chaque canal sous forme d'une **impulsion
PWM individuelle** (1 000–2 000 µs, 50 Hz). L'Arduino lit 5 canaux en parallèle
via une interruption PCINT1 unique (ISR partagée), sans blocage de la boucle
principale. Les mesures sont horodatées au µs avec `micros()`.

| Broche | Canal RC | Fonction            |
|--------|----------|---------------------|
| A1     | CH1      | Ailerons            |
| A2     | CH2      | Profondeur          |
| A3     | CH3      | Gaz                 |
| A4     | CH4      | Direction           |
| A5     | CH7 (SWA)| Arbitrage mode      |

Implémentation : `nano/RcPwm.h` / `nano/RcPwm.cpp`.

**Pourquoi pas IBus ?**  
IBus est un protocole série à 115 200 bauds sur un seul fil (tous les canaux multiplexés).
Deux problèmes ont conduit à l'abandonner :
1. Une SoftwareSerial à 115 200 bauds sur ATmega328P interfère avec les interruptions Timer1 de la lib Servo et produit des trames manquées.
2. Sans accès série de débogage, il était impossible de diagnostiquer pourquoi les trames IBus n'étaient pas décodées correctement.

Note : D0/D1 (UART matériel) est désormais utilisé pour la liaison Pi ↔ Nano.

Le PWM individuel ne nécessite aucune précision de timing à l'émission (tolérance ±200 µs contre ±4 µs pour l'UART) et s'accommode parfaitement de PCINT.


---

## Gestion du moteur et du variateur

Le moteur est le seul composant présentant un **risque physique immédiat**.

Le variateur s'arrête sur réception d'une impulsion à position minimale
(~1000 µs). L'Arduino envoie **activement** cette consigne gaz zéro —
il ne cesse jamais d'émettre, car une absence de signal produit un
comportement indéfini selon le variateur.

Conditions déclenchant la consigne gaz zéro forcée :
- commande explicite du Pi (état `ArretUrgence` ou `Desarmement`),
- chien de garde Pi : absence de trame valide pendant 2 secondes,
- sécurité de perte signal télécommande (voir ci-dessous).

En cas de déclenchement du chien de garde, les surfaces de vol sont
mises à plat — l'avion passe en vol plané.
Le pilote peut reprendre la main via le canal d'arbitrage à tout moment.

---

## Sécurité de perte signal télécommande

Sur perte du signal RC (absence de pulse valide sur A1–A5 pendant plus de
1 seconde), le Nano force les gaz à zéro et remonte le flag
`securite_rc_perdue` au Pi via la trame de remontée.

Ce comportement conserve la capacité d'arrêt d'urgence via télécommande,
qui constitue l'un des verrous de sécurité les plus importants du système.
Le Pi réagit dans sa MAÉ vol (atterrissage d'urgence, désarmement selon
la phase de vol).

**Note V2 :** surveiller la qualité du signal RC (RSSI) pour déclencher
un retour automatique au point de décollage avant la perte totale du signal,
plutôt que de couper le moteur une fois le signal déjà perdu.


---

## Chien de garde Pi

L'Arduino surveille la réception des trames Pi.
Sur absence de trame valide pendant **2 secondes** (100 trames manquées
à 50 Hz), il positionne le flag `chien_de_garde` dans la trame de
remontée. Le Pi reçoit ce flag et gère la réaction dans sa MAÉ vol
(loiter, atterrissage d'urgence, etc.).

Le Nano force les gaz à zéro et maintient les surfaces de vol à plat,
l'appareil passe en vol plané. Le flag `chien_de_garde`
est positionné dans la trame de remontée ; le Pi réagit dans sa MAÉ vol
à son réveil. Le pilote peut reprendre la main à tout moment via le canal
d'arbitrage, indépendamment de l'état du chien de garde.

---

## Mode test sol et armement

Avant l'armement, le Pi doit pouvoir vérifier que les servos s'actionnent
dans le bon sens sans risque de démarrage moteur.

Chaque trame Pi → Arduino contient un **bit d'armement**. L'Arduino
applique la logique suivante :

| CH7/SWA (A5)  | Bit armement Pi | Comportement Arduino                     |
|---------------|-----------------|------------------------------------------|
| Autopilote    | Non armé        | Servos Pi actifs, gaz forcé à zéro       |
| Autopilote    | Armé            | Tous les canaux Pi actifs                |
| Manuel        | indifférent     | Retransmission directe RC complète       |

La MAÉ vol envoie le bit armement uniquement à l'entrée de l'état
`Armement` (après validation opérateur). Le canal RC reste le sélecteur
de priorité absolue, indépendant de l'armement.

Structure détaillée des trames : voir `doc/protocole_uart_arduino.md`.


---

## Canaux actionneurs

Canaux de sortie minimum prévus (Arduino → servos / variateur) :

| Canal | Actionneur             |
|-------|------------------------|
| 1     | Ailerons               |
| 2     | Gouverne de profondeur |
| 3     | Gaz (variateur)        |
| 4     | Gouverne de direction  |

Canaux additionnels (volets, train rentrant…) extensibles selon matériel.

Plage acceptée par canal : 1 000–2 000 µs. Le clippages des valeurs hors
plage et les limites mécaniques par canal sont reportés en V2.

CH7 (SWA) est réservé à l'arbitrage manuel / autopilote (entrée uniquement sur A5 —
aucune sortie servo associée).


---

## Raccordement physique sur l'Arduino Nano

### Sorties (Arduino → servos / variateur) : 4 broches

Bibliothèque `Servo.h` standard (Timer1). Broches retenues :

| Broche | Actionneur             |
|--------|------------------------|
| D3     | Servo ailerons         |
| D5     | Servo profondeur       |
| D6     | Variateur (gaz)        |
| D9     | Servo direction        |

### Entrée RC (récepteur → Arduino) : 5 broches

Lecture PWM individuelle non-bloquante via PCINT1 (`RcPwm.h`).
Un seul vecteur d'interruption couvre les 5 broches (PC1–PC5).

| Broche | Canal RC | Fonction            |
|--------|----------|---------------------|
| A1     | CH1      | Ailerons            |
| A2     | CH2      | Profondeur          |
| A3     | CH3      | Gaz                 |
| A4     | CH4      | Direction           |
| A5     | CH7 SWA  | Arbitrage mode      |

### Liaison Pi ↔ Arduino

Un **adaptateur de niveau 3,3V / 5V** est intercalé entre le Pi (3,3V)
et le Nano (5V). Câblage physique sur D0 (RX Nano ← Pi TX) et D1 (TX Nano → Pi RX).

Liaison à 57 600 bauds via l'UART matériel du Nano (`Serial`).
Le Pi dispose de l'UART2 matériel (`/dev/ttyS2`, broches GPIO).

### Tableau de câblage

| Broche | Direction    | Usage                                                       |
|--------|--------------|-------------------------------------------------------------|
| D3     | Sortie servo | Ailerons                                                    |
| D5     | Sortie servo | Profondeur                                                  |
| D6     | Sortie servo | Gaz (variateur)                                             |
| D0     | Entrée série | Nano ← Pi TX (via adaptateur 3,3V/5V) — RX consignes Pi    |
| D1     | Sortie série | Nano → Pi RX (via adaptateur 3,3V/5V) — TX remontée Nano   |
| D9     | Sortie servo | Direction                                                   |
| D13    | Sortie LED   | Diagnostic (1/2/4 Hz selon état RC)                         |
| A1     | Entrée PWM   | RC CH1 — Ailerons                                           |
| A2     | Entrée PWM   | RC CH2 — Profondeur                                         |
| A3     | Entrée PWM   | RC CH3 — Gaz                                                |
| A4     | Entrée PWM   | RC CH4 — Direction                                          |
| A5     | Entrée PWM   | RC CH7 SWA — Arbitrage mode                                 |


---

Structure des trames : voir `doc/protocole_uart_arduino.md`.
