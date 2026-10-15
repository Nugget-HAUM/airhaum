# Interface Pi / RC / Servos — AirHaum II

## Rôle

Le contrôleur servo (ESP32) est le **point d'intersection** entre trois systèmes :
- l'autopilote (Orange Pi) qui produit des consignes de vol,
- la télécommande RC qui permet la prise de main manuelle et l'arret d'urgence,
- les actionneurs (servos + variateur) qui agissent physiquement sur l'appareil.

Il est le **dernier maillon de sécurité matériel** : il garantit que les
actionneurs reçoivent toujours un signal valide quelle que soit l'origine
de la commande, et que le pilote peut physiquement reprendre la main à
tout moment.

Carte : **WEMOS D1 MINI ESP32** (module ESP32-WROOM-32), 3,3 V.  
Firmware : `esp32/esp32.ino` + `esp32/RcIbus.h` / `.cpp`
(Arduino IDE, carte « WEMOS D1 MINI ESP32 », core esp32 3.x).  
Implémentation côté Pi : `src/drivers/controleur_servo.rs`  
Protocole Pi ↔ ESP32 : binaire personnalisé — UART2 Pi (`/dev/ttyS2`) ↔ UART1 ESP32, 3,3 V direct  
- **ESP32 → Pi** (remontée) : TX sur IO19 à 57 600 bauds.  
- **Pi → ESP32** (consignes) : RX sur IO26 à 57 600 bauds.  
Protocole RC → ESP32 : iBUS (FlySky) sur UART2, RX IO23, 115 200 bauds.

L'ESP32 remplace l'Arduino Nano utilisé jusqu'à E0.1 (lecture RC par PWM
individuel abandonnée, voir `doc/essais/E0.1.md`). L'ancien firmware est
archivé dans `doc/archives/nano/`, sans évolution prévue. Le protocole
Pi ↔ contrôleur est inchangé : aucune modification côté Pi.


---

## Comportement au démarrage

À la mise sous tension, avant réception de toute trame Pi ou RC valide :

- Gaz maintenus à zéro (1 000 µs).
- Servos en position neutre (1 500 µs sur tous les canaux).
- Mode **manuel** par défaut (CH7/SWA détermine le mode dès réception RC).

Les sorties servo sont configurées en tout premier dans `setup()`, avant
les liaisons série, pour émettre ces valeurs le plus tôt possible.

L'ESP32 reste dans cet état d'attente jusqu'à réception de trames valides
des deux sources. Le bit d'armement étant à 0 par définition, les gaz
restent bloqués jusqu'à armement explicite par le Pi.


---

## Arbitrage manuel / autopilote

L'arbitrage est **local à l'ESP32**, sans intervention du Pi.

Un canal RC dédié (CH7 — interrupteur SWA) fait office d'interrupteur de mode :

| Position canal  | Mode actif | Source des consignes servos          |
|-----------------|------------|--------------------------------------|
| Bas (1 000 µs)  | Manuel     | Retransmission directe des canaux RC |
| Haut (2 000 µs) | Autopilote | Consignes reçues du Pi               |

La bascule est **instantanée**, sans transition progressive.
Aucune logique de reprise douce n'est prévue dans les premières versions.

Le canal d'arbitrage est fixé par la constante `CANAL_IBUS_ARBITRAGE`
(`esp32.ino`). Si l'émetteur n'émet que 6 canaux, SWA doit être réaffecté
à CH5 ou CH6 et cette constante modifiée en conséquence.

Le Pi n'a pas connaissance en temps réel de l'état du mode actif via ce
canal — il reçoit l'information via la remontée d'état ESP32 (voir ci-dessous).


---

## Flux de données

### Pi → ESP32 (prioritaire)

| Donnée          | Contenu                              | Fréquence |
|-----------------|--------------------------------------|-----------|
| Consignes servo | 4 canaux en µs + bit d'armement      | 50 Hz     |

Le Pi envoie ses consignes en permanence. Chaque trame valide reçue
réarme le chien de garde ESP32. L'ESP32 applique ou ignore les
consignes selon le mode actif, mais continue de les recevoir.


### ESP32 → Pi (remontée)

| Donnée          | Contenu                                       | Fréquence                   |
|-----------------|-----------------------------------------------|-----------------------------|
| Canaux RC       | Valeurs µs des 5 canaux utiles (CH1–CH4, CH7) | 10 Hz                       |
| État mode actif | Manuel ou Autopilote                          | 10 Hz                       |
| État variateur  | Consigne gaz appliquée                        | 10 Hz                       |

La remontée RC est destinée à la **boîte noire** du Pi et à l'analyse
post-vol. Dans les premières versions elle n'est pas utilisée pour le contrôle en temps réel.

Option de campagne de mesure : `#define DIAGNOSTIC_REMONTEE_50HZ`
(`esp32.ino`) porte la remontée à 50 Hz. Désactivée par défaut.


### RC → ESP32

Le récepteur FlySky transmet tous ses canaux sur un **seul fil**, au
format **iBUS** : trame de 32 octets toutes les ~7 ms, 115 200 bauds 8N1.

| Octets  | Contenu                                                   |
|---------|-----------------------------------------------------------|
| 0       | `0x20` — longueur de trame                                |
| 1       | `0x40` — commande « canaux »                              |
| 2–29    | 14 canaux × 2 octets, petit-boutiste (1 000–2 000 µs)     |
| 30–31   | Somme de contrôle = `0xFFFF` − Σ octets 0–29, petit-boutiste |

La trame est décodée par `rcibus_traiter()` (`esp32/RcIbus.cpp`), appelée
à chaque tour de `loop()` : resynchronisation sur l'en-tête `0x20 0x40`,
rejet des trames dont la somme de contrôle est fausse (compteur
`ibus_rej` visible dans les traces USB). La réception est entièrement
assurée par l'UART matériel : aucune interruption logicielle.

| Canal iBUS | Fonction            | Index remontée |
|------------|---------------------|----------------|
| CH1        | Ailerons            | 0              |
| CH2        | Profondeur          | 1              |
| CH3        | Gaz                 | 2              |
| CH4        | Direction           | 3              |
| CH7 (SWA)  | Arbitrage mode      | 4              |

Côté émetteur, la sortie série du récepteur doit être réglée sur iBUS
(*RX Setup → Output mode → Serial : i-BUS*). Calibration des manches :
voir `doc/recepteur_rc.md`.

**Pas de filtre de lissage.** L'iBUS transporte des valeurs numériques :
le bruit de mesure de la capture PWM par interruption, qui avait motivé
un filtre EMA sur le Nano, n'existe plus. Les valeurs reçues sont
appliquées et remontées telles quelles (bornées à 1 000–2 000 µs).

**Pourquoi l'iBUS fonctionne sur l'ESP32 et pas sur le Nano**  
L'iBUS avait été abandonné sur le Nano : une SoftwareSerial à 115 200 bauds
y entrait en conflit avec les interruptions de la lib Servo, et l'unique
UART matériel, occupé par le Pi, ne laissait aucune sortie de débogage.
L'ESP32 dispose de trois UART matériels : un pour l'USB (flash et traces),
un pour le Pi, un pour l'iBUS. Aucun ne partage de ressource avec la
génération des impulsions servo.


---

## Gestion du moteur et du variateur

Le moteur est le seul composant présentant un **risque physique immédiat**.

Le variateur s'arrête sur réception d'une impulsion à position minimale
(~1000 µs). L'ESP32 envoie **activement** cette consigne gaz zéro —
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

Sur perte du signal RC (aucune trame iBUS valide pendant plus de
1 seconde), l'ESP32 force les gaz à zéro et remonte le flag
`securite_rc_perdue` au Pi via la trame de remontée.

**Point à valider en essai** : selon sa configuration, le récepteur peut
continuer d'émettre des trames iBUS à la perte de la liaison radio
(dernières valeurs ou valeurs de failsafe). Dans ce cas, l'ESP32 ne voit
pas de perte RC. Le failsafe doit être configuré sur l'émetteur (gaz au
minimum au moins), et le comportement vérifié en coupant l'émetteur
(`doc/plan_essais.md`, E0.9).

Ce comportement conserve la capacité d'arrêt d'urgence via télécommande,
qui constitue l'un des verrous de sécurité les plus importants du système.
Le Pi réagit dans sa MAÉ vol (atterrissage d'urgence, désarmement selon
la phase de vol).

**Note V2 :** surveiller la qualité du signal RC (RSSI) pour déclencher
un retour automatique au point de décollage avant la perte totale du signal,
plutôt que de couper le moteur une fois le signal déjà perdu.


---

## Chien de garde Pi

L'ESP32 surveille la réception des trames Pi.
Sur absence de trame valide pendant **2 secondes** (100 trames manquées
à 50 Hz), il positionne le flag `chien_de_garde` dans la trame de
remontée. Le Pi reçoit ce flag et gère la réaction dans sa MAÉ vol
(loiter, atterrissage d'urgence, etc.).

L'ESP32 force les gaz à zéro et maintient les surfaces de vol à plat,
l'appareil passe en vol plané. Le flag `chien_de_garde`
est positionné dans la trame de remontée ; le Pi réagit dans sa MAÉ vol
à son réveil. Le pilote peut reprendre la main à tout moment via le canal
d'arbitrage, indépendamment de l'état du chien de garde.

---

## Mode test sol et armement

Avant l'armement, le Pi doit pouvoir vérifier que les servos s'actionnent
dans le bon sens sans risque de démarrage moteur.

Chaque trame Pi → ESP32 contient un **bit d'armement**. L'ESP32 applique
la logique suivante :

| CH7/SWA       | Bit armement Pi | Comportement ESP32                       |
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

Canaux de sortie minimum prévus (ESP32 → servos / variateur) :

| Canal | Actionneur             |
|-------|------------------------|
| 1     | Ailerons               |
| 2     | Gouverne de profondeur |
| 3     | Gaz (variateur)        |
| 4     | Gouverne de direction  |

Canaux additionnels (volets, train rentrant…) extensibles selon matériel.

Plage acceptée par canal : 1 000–2 000 µs. Le clippages des valeurs hors
plage et les limites mécaniques par canal sont reportés en V2.

CH7 (SWA) est réservé à l'arbitrage manuel / autopilote (entrée uniquement —
aucune sortie servo associée).


---

## Raccordement physique sur l'ESP32

Les broches sont désignées par leur numéro GPIO (sérigraphie `IOxx` de la
carte). Toutes sont hors des broches lues au démarrage (IO0, IO2, IO5,
IO12, IO15) et des broches de la mémoire flash (SD0–SD3, CMD, CLK).

### Sorties (ESP32 → servos / variateur) : 4 broches

Impulsions générées par le périphérique matériel **LEDC** : 50 Hz,
résolution 16 bits (≈ 0,3 µs), sans gigue logicielle. Aucune bibliothèque
externe.

| Broche | Actionneur             |
|--------|------------------------|
| IO21   | Servo ailerons         |
| IO17   | Servo profondeur       |
| IO16   | Variateur (gaz)        |
| IO18   | Servo direction        |

IO16 et IO17 ne sont libres que sur un module **WROOM** : sur un module
WROVER, elles sont réservées à la PSRAM.

### Entrée RC (récepteur → ESP32) : 1 broche

| Broche | Usage                                         |
|--------|-----------------------------------------------|
| IO23   | Sortie iBUS du récepteur (UART2, RX seul)     |

UART2 est ouvert sans broche TX : sinon le core le rattacherait à sa
broche TX par défaut, IO17, utilisée ici par le servo de profondeur.

### Liaison Pi ↔ ESP32

Liaison directe en 3,3 V, sans adaptateur de niveau. TX et RX sont croisés :

| ESP32         | Pi                      |
|---------------|-------------------------|
| IO26 (RX)     | TX UART2 (`/dev/ttyS2`) |
| IO19 (TX)     | RX UART2 (`/dev/ttyS2`) |
| GND           | GND                     |

Liaison à 57 600 bauds via l'UART1 de l'ESP32 (`Serial1`).

### Port USB : flash et débogage

L'UART0 (USB) reste libre. Il sert au flash et aux traces de débogage
(115 200 bauds, 2 Hz, actives par défaut, `#define TRACES_USB`) :

```
rc ok | ail 1500 pro 1500 gaz 1000 dir 1500 arb 1000 | MANUEL | pi ok | gaz_app 1000 | ibus_rej 0
```

### Alimentation

Alimentation 5 V externe sur la broche VCC de la carte ; le régulateur
de la carte fournit le 3,3 V. La masse doit être commune avec le
récepteur, les servos, le variateur et le Pi.

### Tableau de câblage

| Broche | Direction    | Usage                                               |
|--------|--------------|-----------------------------------------------------|
| IO21   | Sortie servo | Ailerons                                            |
| IO17   | Sortie servo | Profondeur                                          |
| IO16   | Sortie servo | Gaz (variateur)                                     |
| IO18   | Sortie servo | Direction                                           |
| IO23   | Entrée série | iBUS récepteur RC (115 200 bauds)                   |
| IO26   | Entrée série | ESP32 ← Pi TX — consignes Pi (57 600 bauds)         |
| IO19   | Sortie série | ESP32 → Pi RX — remontée d'état (57 600 bauds)      |
| IO2    | Sortie LED   | LED intégrée : 1 Hz aucune trame iBUS, 2 Hz RC perdu, 4 Hz RC OK |
| VCC    | Alimentation | 5 V externe                                         |
| GND    | Masse        | Commune à tout le montage                           |


---

## Dépannage

**Redémarrage en boucle, `csum err` ou `flash read err` dans le moniteur
série** : la puce lit des données corrompues dans sa mémoire flash, avant
même l'exécution du firmware. Cas rencontré à la mise en service : carte
fonctionnelle seule, en boucle une fois montée sur la platine, plantage
au passage AUTO → MANUEL. Cause : une soudure défectueuse sur la platine,
trouvée par inspection complète des soudures. Points à vérifier : IO12
à 0 V au démarrage, aucune liaison sur les broches de la flash, aucune
broche reliée par erreur à un plot voisin.


---

Structure des trames : voir `doc/protocole_uart_arduino.md`.
