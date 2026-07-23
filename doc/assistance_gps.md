# Assistance GPS — AirHaum II

## Contexte

Constaté le 2026-07-14 : ~20 min pour un premier fix, ~29 min pour un fix
stable à 6 satellites (fil de vie analysé — pas de boucle de réinit
logicielle en cause, acquisition satellite authentiquement lente).

Mise à jour 2026-07-18 : le module GPS a bien une pile de sauvegarde sur
`VBCKP` (confirmé par inspection physique), contrairement à l'hypothèse
initiale. Un test le même jour a montré un TTFF de 85,6 s au redémarrage au
même endroit, **sans qu'aucune assistance logicielle n'ait été sauvegardée**
(donc entièrement dû à la BBR interne du module). Ça n'invalide pas le besoin
de cette fonctionnalité : la pile ne préserve utilement que ce qui n'a pas
expiré / n'est pas devenu faux entre-temps —
- éphémérides diffusées : ~2-4h de validité quelle que soit l'alimentation ;
- orbites prédites (MGA-DBD, voir plus bas) : validité plus longue (heures à
  quelques jours) mais dégradation progressive, pas indéfinie ;
- position/heure stockées : valables indéfiniment dans le temps, mais
  deviennent une **fausse indication** (pas juste inutile) si le terrain de
  vol change de plusieurs dizaines de km.

Donc les essais précédents (plusieurs semaines d'inactivité, ou terrain
different de ~30 km) restent des cold starts de fait malgré la pile — la
fonctionnalité logicielle ci-dessous reste utile pour ces cas, et pour
disposer d'assistance sur plusieurs terrains distincts sans dépendre de l'état
interne (non consultable) de la BBR du module.

## Solution retenue

Deux aides UBX combinées, sans matériel ni connectivité internet :

- **Position/heure approximative** (`UBX-MGA-INI-POS_LLH` + `TIME_UTC`) —
  réduit l'espace de recherche satellite/Doppler. Reste valable indéfiniment
  *pour un terrain donné* — voir attention ci-dessous sur le changement de
  terrain.
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
- **Registre multi-terrains** (`drivers/gps/calibration.rs`,
  `RegistreTerrains`/`EntreeTerrain`) : plusieurs terrains de vol connus,
  chacun avec sa position + ses orbites propres, et un `terrain_actif`
  (celui injecté au démarrage). Remplace l'ancien slot unique `AssistanceGps`
  (qui reste le type de valeur protocole, sans notion de nom, utilisé par
  `AssistanceGnss`). Persisté via `systeme::calibration`, identifiant
  `terrains_gps` — fichier volontairement éditable à la main pour ajouter un
  terrain (nom/latitude/longitude suffisent, voir l'en-tête du fichier
  généré). Le registre lui-même n'expire jamais (`est_valide() = true`) : la
  notion de péremption ne s'applique qu'aux orbites d'une entrée, pas au
  fichier.
- Console de calibration (séquence sol), option **t** : liste les terrains
  connus + le fix GPS courant pour comparaison visuelle, change
  `terrain_actif`. Comme le GPS est lancé *avant* cette console (pour
  maximiser le temps d'acquisition), changer de terrain ici **tue et relance**
  le thread GPS pour que la nouvelle sélection soit injectée immédiatement
  (repasse par `DriverGps::initialiser()` en entier — plus simple et plus
  prévisible qu'un rappel ciblé de `importer_assistance()` sur un module déjà
  en cours d'acquisition).
- Console de test, option **43** : sauvegarde manuelle (position + orbites)
  dans l'entrée du terrain **actif**, déclenchée par le pilote avant
  extinction. Si aucun terrain n'est actif (registre absent ou premier
  lancement), amorce automatiquement un nouveau terrain à partir du fix GPS
  courant — demande juste un nom, sans édition manuelle du fichier requise
  pour la toute première sauvegarde. Échoue seulement si aucun fix GPS n'est
  disponible. L'injection au démarrage est automatique, sans action
  utilisateur (sous réserve d'avoir choisi le bon terrain actif au préalable).

## Limites

- Sauvegarde manuelle seulement — une coupure imprévue entre deux sauvegardes
  n'est pas couverte.
- **Changer de terrain sans mettre à jour `terrain_actif`** avant de voler
  ailleurs réinjecte une position fausse de plusieurs dizaines de km — peut
  ralentir l'acquisition plutôt que rester neutre. Toujours vérifier/changer
  le terrain actif (menu « t ») en changeant de site.
- Classes/ID UBX (`0x13/0x40` MGA-INI, `0x13/0x80` MGA-DBD) à confirmer contre
  la doc constructeur avant implémentation — non invalidé par le test du
  2026-07-18 (TTFF rapide observé sans qu'aucune assistance logicielle n'ait
  été utilisée ce jour-là).
