# Protocole UART Pi ↔ Arduino Nano — AirHaum II

Liaison série matérielle sur D0 (RX0, consignes Pi→Nano) et D1 (TX0, remontée Nano→Pi), 57 600 bauds.
Adaptateur de niveau 3,3V / 5V intercalé entre Pi et Nano.

Deux types de trames : **consignes** (Pi → Arduino, 50 Hz) et **remontée**
(Arduino → Pi, 10 Hz). Chaque trame commence par un octet de début distinct
pour permettre la resynchronisation en cas d'octet perdu.

---

## Trame Pi → Arduino (consignes, 50 Hz)

| Octet | Champ                | Type      | Détail                                       |
|-------|----------------------|-----------|----------------------------------------------|
| 0     | Début de trame       | uint8     | `0xAA`                                       |
| 1     | Fanions              | uint8     | bit 0 = armé (0 = non armé, 1 = armé)        |
| 2–3   | Canal 1 — ailerons   | uint16 LE | Valeur en µs (1 000–2 000, neutre 1 500)     |
| 4–5   | Canal 2 — profondeur | uint16 LE | Valeur en µs                                 |
| 6–7   | Canal 3 — gaz        | uint16 LE | Valeur en µs (ignoré si bit armé = 0)        |
| 8–9   | Canal 4 — direction  | uint16 LE | Valeur en µs                                 |
| 10    | Somme de contrôle    | uint8     | XOR des octets 1 à 9                         |

Taille totale : **11 octets**.  
Débit à 50 Hz : 550 octets/s (< 1 % de la bande à 57 600 bauds).

### Règle gaz zéro

Quand le bit armé vaut 0, l'Arduino ignore la valeur du canal 3 et applique
la consigne gaz minimale (1 000 µs) quelle que soit la valeur reçue.

### Chien de garde

Toute trame valide (somme de contrôle correcte) réarme le chien de garde.
Sur absence de trame valide pendant **2 secondes** (100 trames manquées) :
- les gaz sont forcés à zéro (1 000 µs),
- les surfaces de vol sont mises à plat (1 500 µs),
- le bit 2 du fanion de remontée (`chien_de_garde`) est positionné.

Aucune bascule automatique de mode : le Pi reçoit le flag à son réveil et
réagit dans sa MAÉ vol. Le pilote peut reprendre la main à tout moment
via le canal d'arbitrage RC (liaison récepteur RC - Arduino Nano).

---

## Trame Arduino → Pi (remontée, 10 Hz)

| Octet | Champ             | Type      | Détail                                              |
|-------|-------------------|-----------|-----------------------------------------------------|
| 0     | Début de trame    | uint8     | `0xBB`                                              |
| 1     | Fanions           | uint8     | bit 0 = mode (0 autopilote / 1 manuel), bit 1 = sécurité RC perdue, bit 2 = chien de garde Pi déclenché |
| 2–3   | Canal RC 1 (A1)   | uint16 LE | Valeur PWM µs — CH1 Ailerons                        |
| 4–5   | Canal RC 2 (A2)   | uint16 LE | CH2 Profondeur                                      |
| 6–7   | Canal RC 3 (A3)   | uint16 LE | CH3 Gaz                                             |
| 8–9   | Canal RC 4 (A4)   | uint16 LE | CH4 Direction                                       |
| 10–11 | Canal RC 5 (A5)   | uint16 LE | CH7 SWA — canal d'arbitrage                         |
| 12–13 | Gaz appliqué      | uint16 LE | Consigne réellement envoyée au variateur (µs)       |
| 14    | Somme de contrôle | uint8     | XOR des octets 1 à 13                               |

Taille totale : **15 octets**.  
Débit à 10 Hz : 150 octets/s.

---

## Resynchronisation

Si la somme de contrôle ne correspond pas, le récepteur rejette la trame
et attend le prochain octet de début (`0xAA` ou `0xBB`).

Les deux valeurs de début sont choisies pour ne pas apparaître dans les
données utiles : les valeurs servo sont comprises entre 1 000 et 2 000
(`0x03E8` à `0x07D0`), dont les octets de poids fort sont toujours ≤ `0x07`,
jamais `0xAA` ni `0xBB`.
