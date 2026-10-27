# Calibration des capteurs — AirHaum II

Procédures de calibration et règles de validité, arrêtées le 2026-10-27.
**Statut : implémenté le 2026-10-27, validé sur cible le 2026-10-28 (E0.2, `doc/essais/E0.2.md`) ; magnétomètre à refaire dehors.**
Calculs : `drivers/imu/calcul_calibration.rs` (testés sur données
simulées) ; déroulé console : `diagnostiques/diag_mpu9250.rs`, menu du
bilan des calibrations (`a`, `g`, `m`). Les fichiers de l'ancienne
méthode (§ 6) sont refusés au chargement (format version 2) et affichés
« Illisible » : à refaire.

Règles générales inchangées (`doc/initialisation.md`) : calibration au sol,
stockée en flash avec horodatage Unix, **jamais de calibration automatique
au démarrage** (un redémarrage en vol réutilise les calibrations stockées).
Les étapes « terrain » ci-dessous sont donc toujours déclenchées par
l'opérateur depuis la console de calibration.

---

## 1. Vue d'ensemble

| Calibration | Lieu | Procédure | Validité |
|---|---|---|---|
| Accéléromètre — montage | Atelier, sol plat | queue au sol, puis empennage levé à l'horizontale (§ 3) | 6 mois, vérifiée à chaque session (§ 4) |
| Gyroscope + Z accéléromètre | Terrain | 5 s immobile, queue au sol (§ 4) | 1 h |
| Magnétomètre | Terrain, loin du métal | un tour complet sur les roues (§ 5) | 1 h |
| Baromètre | Terrain | avion immobile, avant armement | 1 h |
| Télémètre | — | remise à zéro automatique au démarrage | — |

Ordre sur le terrain : gyroscope (§ 4), puis magnétomètre (§ 5, utilise le
gyroscope et l'accéléromètre), puis baromètre au plus près de l'armement.

**Repère.** La calibration de montage (§ 3) fournit la rotation carte →
avion. Le driver l'applique aux trois capteurs : tout ce qui sort du driver
est dans les axes de l'avion, convention de l'estimation
(`estimation/attitude.rs`) — X nez, Y aile droite, Z vers le bas ; roulis
positif aile droite en bas, tangage positif nez haut, lacet positif nez à
droite. L'accéléromètre est délivré en *gravité apparente* (opposé de la
force spécifique) : +g sur Z avion immobile à plat. Le gyroscope et le
magnétomètre sont délivrés tels quels dans ces axes. L'orientation de la
carte dans l'avion n'a pas à être connue : elle est déduite de la
procédure. Sans calibration de montage, le driver suppose une carte à
plat, X vers le nez. Les axes de l'AK8963 sont réalignés au préalable sur
ceux de l'accéléromètre (X↔Y permutés, Z inversé — fiche technique
MPU9250).

**Calibration périmée.** L'expiration est un critère d'armement, pas
d'utilisation : le driver applique toujours la dernière calibration
enregistrée, même expirée (un redémarrage en vol au-delà d'une heure ne
doit pas faire perdre la rotation de montage ni les biais).

---

## 2. Validités — justification

- **1 h minimum pour tout ce qui se fait sur le terrain** : une relance de
  l'application (ou un redémarrage en vol) ne doit pas imposer de
  recalibrer.
- **Gyroscope, 1 h** : le biais varie avec la température et à chaque mise
  sous tension ; sur une heure la dérive reste faible et l'EKF estime le
  biais résiduel en continu.
- **Z accéléromètre, 1 h** : le décalage de Z varie avec la température
  (atelier ≠ terrain au petit matin) ; réajusté avec le gyroscope.
- **Magnétomètre, 1 h** : la perturbation dépend de l'équipement embarqué
  (batterie changée ou déplacée entre deux vols, câblage) ; un tour au sol
  par session suit ces changements.
- **Baromètre, 1 h** : la pression varie avec la météo (≈ 1 hPa/h au pire,
  1 hPa ≈ 8 m) ; l'atterrissage s'appuie sur le télémètre.
- **Montage accéléromètre, 6 mois** : grandeur mécanique, qui ne change
  que si la carte bouge (choc, remontage) — c'est ce que contrôle la
  vérification de chaque session (§ 4). L'échéance couvre ce qu'elle ne
  voit pas : dérive lente des biais X/Y de 1 à 2° (vieillissement,
  contraintes mécaniques, saisons), inférieure à sa tolérance de ±3°, et
  orientation de la carte autour de la verticale, que la gravité ne
  permet pas de contrôler. À refaire aussi avant l'échéance après tout
  démontage, choc, ou échec de la vérification.

---

## 3. Accéléromètre — montage (atelier)

**Matériel** : long niveau à bulle posé sur la ligne de référence du
fuselage. Sol plat, vérifié au niveau dans le sens transversal (le roulis
de référence est celui du sol sous les roues). Avion complet, carte dans
sa position de vol.

1. Queue au sol, avion immobile 3 s.
2. Lever l'empennage lentement jusqu'à ce que le niveau indique le
   fuselage horizontal (ou le poser sur un support de la bonne hauteur,
   réutilisable ensuite comme cale), tenir immobile 3 s.
3. Reposer la queue.

Les phases immobiles sont détectées par le gyroscope. Résultats :

- **référence horizontale** (étape 2) → roulis et tangage de montage. Le
  biais du capteur à l'attitude de vol est absorbé dans cette rotation ;
  l'erreur ne réapparaît qu'à grande attitude et reste faible ;
- **axe de tangage** : entre les étapes 1 et 2, la carte tourne autour de
  l'axe transversal de l'avion → lacet de montage (quel axe de la carte
  pointe vers le nez) ;
- **attitude au sol de référence** (étape 1) → vérification à chaque
  lancement (§ 4) ;
- **décalage de Z** initial (norme au repos = 1 g).

Refaire tous les 6 mois, et après tout démontage, choc, ou échec de la
vérification de session (§ 4). Prérequis : Δ tangage entre les deux
positions ≥ 3° (sinon l'axe de tangage est mal défini).

---

## 4. Gyroscope + Z accéléromètre + vérification du montage (terrain, à chaque session)

Avion queue au sol, immobile 5 s, aucune manipulation. Une seule
commande console, qui :

- calibre le **gyroscope** (moyenne au repos) ;
- réajuste le **décalage de Z** pour que la norme mesurée vaille 1 g. Seule
  la norme est recalée sur le terrain : elle ne dépend pas de la pente du
  sol, contrairement à l'attitude. Recaler l'attitude ici prendrait la
  pente du terrain (1 à 2°) pour un défaut du capteur ;
- **vérifie** le montage : attitude au repos = attitude au sol de référence
  à ±3° (tolérance élargie pour la pente du terrain). En cas d'écart, rien
  n'est enregistré : la calibration gyroscope, bloquante pour l'armement,
  reste à faire, et la calibration atelier doit être refaite. Une
  calibration gyroscope valide garantit donc un montage vérifié dans
  l'heure.

---

## 5. Magnétomètre (terrain, loin du métal)

**Conditions** : avion en configuration de vol (batterie en place, système
sous tension, moteur arrêté), sur le terrain, à l'écart des voitures,
clôtures et dalles en béton armé — donc pas à l'intérieur. Après le § 4.

1. Immobile 3 s.
2. Faire pivoter l'avion lentement sur ses roues (en poussant la queue de
   côté), un tour complet en une minute environ. Lever l'empennage pour
   tourner (roue de queue) est permis. La console affiche la progression
   (« 210° / 360° »), mesurée par le gyroscope.
3. Immobile 3 s.

**Calcul.** La verticale vue par la carte est suivie à chaque instant :
gyroscope (corrigé du biais de session), recalé lentement sur
l'accéléromètre (constante de temps 2 s). Elle varie d'une dizaine de
degrés quand on lève l'empennage. Pour chaque mesure, le champ moins le
décalage dû à l'avion doit avoir une composante horizontale égale à `Bh`
et une composante verticale égale à `−Bh·tan(I)`. Le décalage (3 axes) et
`Bh` sont ajustés par moindres carrés, au départ d'un cercle ajusté autour
de la verticale moyenne. Le décalage vertical repose sur l'**inclinaison
du champ terrestre** au terrain (modèle WMM, valeurs stockées dans le registre des
terrains : `calibration_terrains_gps.toml`, ≈ 63° et ≈ 48 µT au Mans).
Erreur induite : 1° d'inclinaison ≈ 2 µT vertical, sans effet sur le cap à
plat, ≈ 2,5° de cap à 30° d'inclinaison en virage.

L'inclinaison de l'avion pendant le tour est donc prise en compte, mais
elle ne suffit pas à mesurer le décalage vertical sans `I` : 10 à 20° de
tangage ne font varier la verticale vue par la carte que de 1,5 à 6 %.

Pas de correction « soft iron » : un tour à plat ne permet pas de
l'estimer.

**Refus de la calibration** si : tour incomplet (< 360°), points trop
dispersés (composante horizontale ou verticale, plus de 10 % de la valeur
attendue), ou composante horizontale incohérente avec le terrain (écart
de plus de 25 %).

Le tour étant refait à chaque session (validité 1 h), ces contrôles de
qualité tiennent lieu de vérification. Le registre des terrains contient,
pour chaque terrain, `inclinaison_magnetique_deg` et `champ_magnetique_ut`
(calculateur NOAA :
<https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml>), valables
plusieurs années. Pour un terrain non renseigné (déplacement imprévu), la
calibration n'est pas bloquée : elle utilise des valeurs approximatives du
Mans (63°, 48 µT) et l'affiche. En France métropolitaine, l'erreur
d'inclinaison reste de quelques degrés au plus (≈ 2 µT de décalage
vertical par degré).

---

## 6. Constats à l'origine de ces procédures (E0.2, 2026-10-21)

- **Accéléromètre** : l'ancienne méthode retranche toute la mesure au
  repos ; avion incliné sur sa roulette, l'inclinaison est enregistrée
  comme un défaut du capteur. Le facteur `scale_z` vaut toujours 1 par
  construction.
- **Z à 1,44 g au repos** (`offset_z` = 4,36 m/s², configuration ±4 g
  correcte) : décalage ou erreur d'échelle du composant, hors tolérance
  d'un MPU9250 d'origine. Traité comme un décalage (cas le plus courant ;
  écart entre les deux hypothèses < 0,06 g jusqu'à ~30° d'attitude).
  Non corrigé, il ferait rejeter toutes les corrections accéléromètre par
  l'estimation (seuil sur la norme).
- **Magnétomètre** : min/max sur 30 s, qui suppose toutes les orientations
  couvertes — impossible avec l'envergure de l'avion. Décalage obtenu
  (137 µT, près de trois fois le champ terrestre) inexploitable. Axes de
  l'AK8963 non réalignés sur ceux de l'accéléromètre.
- **Tour magnétomètre à l'atelier (2026-10-27)** : l'empennage levé pour
  tourner incline la carte de 10 à 15°, ce qui fausse un modèle à
  verticale fixe (dispersion 2,5 µT). Avec la verticale suivie : 1,1 à
  1,4 µT, pour un bruit de mesure de ≈ 0,85 µT. Champ horizontal trouvé
  ≈ 19 µT au lieu de ≈ 22 µT, et décalage différent de 17 à 23 µT d'un
  endroit de l'atelier à l'autre : le champ y est perturbé. La calibration
  doit être faite dehors.
