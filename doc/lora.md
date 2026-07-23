# Liaison LoRa — AirHaum II

## Contexte

Liaison bidirectionnelle sol ↔ avion : télémétrie descendante (position, état
vol, état sûreté, batterie) et commandes montantes (dont l'arrêt d'urgence,
déjà anticipé comme message prioritaire dans `doc/surete.md`). Matériel fixé :
un SX1276 SPI sur le Pi, un second SX1276 monté sur un Wemos relié en USB au
PC sol.

## Architecture matérielle et code

- **Pi** : `src/drivers/lora/sx1276.rs` — driver SPI, polling du registre IRQ
  (pas d'interruption GPIO câblée, cohérent avec le pattern des autres
  drivers en thread dédié).
- **Wemos** : `lora/wemos/` — pont **transparent** SPI ↔ USB, aucune logique
  de protocole. Objectif : pas de flash à refaire à chaque évolution du
  protocole, toute la logique applicative vit côté Pi et côté PC.
- **PC** : `lora/pc/` — station sol, technologie non arbitrée (bash pour
  démarrer, réévaluation python/rust plus tard selon les besoins réels).

## Accès au canal

Le SX1276 est half-duplex. État de repos = **réception**. L'émission de
télémétrie est une interruption brève et périodique de ce repos. Ce choix
minimise la latence de réception d'une commande montante, la plus critique
étant l'arrêt d'urgence.

## Fiabilité

- Télémétrie descendante : *best-effort*, pas d'accusé de réception — une
  trame perdue est remplacée par la suivante, sans conséquence.
- Commandes montantes : accusé de réception + retransmission côté sol tant
  que l'ack n'est pas reçu.
- Arrêt d'urgence : exécuté côté Pi dès réception, sans attendre d'ack ;
  le sol continue de le répéter jusqu'à confirmation.

## Format de trame

En-tête commun à tous les messages : type, numéro de séquence, CRC
applicatif. Le numéro de séquence sert d'anti-rejeu simple (éviter qu'une
retransmission fasse exécuter une commande deux fois). Pas de chiffrement —
risque de brouillage/interception accepté pour un usage personnel.

## Détection de perte de liaison

Heartbeat périodique dans les deux sens, indépendant du contenu utile.
Alimente `surveillance_lien.rs` et l'état sûreté (`FailSafe` sur timeout,
cf. `doc/surete.md`). Valeur de timeout à définir à l'implémentation
(point de départ envisagé : 2-3 s).

## Paramètres radio

Un seul jeu de paramètres (SF/BW/puissance) pour la v1, choisi pour un
airtime faible — la latence de la commande d'urgence prime sur le débit de
télémétrie. Valeurs reprises du prototype `OrangePi/SX1276` (hors dépôt) :
BW 125 kHz, SF7, CR 4/5, PA_BOOST ~17 dBm.

## Validation matérielle

Premier essai réussi le 2026-07-18 : émission `Sx1276::envoyer()` côté Pi,
réception côté Wemos/PC — RSSI -36 dBm, SNR 9 dB, erreur de fréquence
~900 Hz (~1 ppm à 868 MHz, dans la tolérance normale d'un cristal bon
marché). Valide le chemin d'émission du driver (init, configuration modem,
FIFO TX, polling IRQ) de bout en bout sur matériel réel. Le chemin de
réception côté Pi (`recevoir()`) n'est pas encore testé — le Wemos n'étant
qu'un pont transparent, ce test n'exerçait qu'un seul sens de la liaison.

## Contenu télémétrie v1

Position, état de la MAE vol, état sûreté. Pas d'ajout tant que ce socle
n'est pas validé en vol. Batterie exclue de la v1 : aucun capteur de tension
en place (`systeme/energie.rs` toujours "A FAIRE") — à ajouter au protocole
une fois ce matériel disponible.

## Limites / points ouverts

- Technologie du logiciel PC non arbitrée.
- Chemin de réception du driver Pi (`recevoir()`) non testé sur matériel réel
  (voir Validation matérielle ci-dessus).
- Timeout de perte de liaison à ajuster à l'usage réel.
- Pas de protection contre le brouillage ou l'interception (acceptable en
  usage personnel, à revoir si le contexte change).
- Suivi batterie absent du matériel v1 (voir ci-dessus).
