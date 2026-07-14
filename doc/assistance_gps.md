# Assistance GPS — AirHaum II

## Contexte

Le NEO-M8N n'a vraisemblablement pas d'alimentation de sauvegarde sur `VBCKP` :
cold start à chaque coupure. Constaté le 2026-07-14 : ~20 min pour un premier
fix, ~29 min pour un fix stable à 6 satellites (fil de vie analysé — pas de
boucle de réinit logicielle en cause, acquisition satellite authentiquement
lente).

## Solution retenue

Deux aides UBX combinées, sans matériel ni connectivité internet :

- **Position/heure approximative** (`UBX-MGA-INI-POS_LLH` + `TIME_UTC`) —
  réduit l'espace de recherche satellite/Doppler. Reste valable indéfiniment
  tant que le terrain de vol ne change pas.
- **Orbites prédites** (`UBX-MGA-DBD`, AssistNow Autonomous — générées par le
  récepteur lui-même) — substitut d'éphéméride, utile surtout dans les
  premières heures/jours après la sauvegarde, mais réinjectée sans condition
  de validité (dégradation progressive, pas de risque à la rejouer).

## Architecture

- `ublox.rs` : `exporter_assistance()` (poll MGA-DBD + dernière position
  connue) et `importer_assistance()` (position/heure + rejeu MGA-DBD),
  appelée dans `initialiser()` avant `activer_messages()`.
- `ubx_parser.rs` : capture brute des trames MGA-DBD (classe `0x13`, ID `0x80`)
  — blob opaque, rejoué tel quel, pas décodé.
- Persistance via `systeme::calibration` (`CalibrationPersistante`),
  identifiant `assistance_gps` : horodatage, position, blob d'orbites en
  base64. Pas d'expiration courte pour la position ; le blob est réinjecté
  sans blocage de validité.
- Console de test, option **43** : sauvegarde manuelle (position + orbites),
  déclenchée par le pilote avant extinction. L'injection au démarrage est
  automatique, sans action utilisateur.

## Limites

- Sauvegarde manuelle seulement — une coupure imprévue entre deux sauvegardes
  n'est pas couverte.
- Classes/ID UBX (`0x13/0x40` MGA-INI, `0x13/0x80` MGA-DBD) à confirmer contre
  la doc constructeur avant implémentation.
