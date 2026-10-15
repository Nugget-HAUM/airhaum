# Plan d'essais — AirHaum II

## 1. Objet du document

Ce document liste les essais en conditions réelles à mener pour valider
progressivement le comportement du système, du banc statique jusqu'au vol
autonome. Il sert de référentiel d'essais indépendant de l'avancement du
développement : chaque essai reste valable tel quel, qu'il soit réalisable
dès aujourd'hui ou seulement plus tard.

Une première passe fixe la liste et le déroulé de chaque essai. Une
deuxième passe a affiné cette liste (suppression de redondances, ajout
d'essais manquants). Une troisième passe (à faire) rebalaie chaque essai
un à un pour vérifier une interprétation commune de son contenu.

---

## 2. Principes généraux

### Progression incrémentale

On ne passe à l'étape suivante que si le comportement observé à l'étape
courante correspond **exactement** à l'attendu. Un essai partiellement
concluant est rejoué, éventuellement après correction, avant de passer à
la suite.

### Convention : un lancement de programme par essai

La mémoire d'incident de la MAÉ sécurité (`deja_degrade_ce_vol`, plafond de
rétablissement) et l'arrêt d'urgence console ne s'effacent aujourd'hui
qu'en relançant le programme (`MaeSecurite::reinitialiser()` n'est jamais
appelé en pratique — voir `doc/suivi_projet.md`). **Un roulage compte
comme un vol à part entière** de ce point de vue : le programme doit être
relancé entre deux essais, y compris entre deux essais de roulage
consécutifs, jamais réutilisé pour plusieurs essais dans le même
lancement. Cette convention conditionne la conduite de toutes les
sessions à partir de E1.

### Sécurité transverse (valable pour tous les essais)

- Pilote RC présent en permanence, mode manuel accessible immédiatement
  (interrupteur d'arbitrage à portée de main, testé en tout début de
  session).
- Procédure d'arrêt d'urgence connue et répétée par tous les participants
  avant le début de la session (qui coupe quoi, qui donne l'ordre).
- Zone d'essai adaptée : dégagée, dimensionnée pour l'essai concerné,
  public et biens tenus à distance de sécurité.
- Conditions météo compatibles avec l'essai (vent, visibilité, absence de
  pluie) — seuils précisés essai par essai si pertinent.
- Checklist d'armement passée en revue avant tout essai impliquant
  l'armement du système, quel que soit l'état des items acquittables.
- Journalisation vérifiée active avant chaque essai (boîte noire).
- Compte rendu post-essai systématique (modèle en annexe, §11).

### Format de chaque fiche essai

- **Objectif** — ce que l'essai cherche à démontrer.
- **Prérequis** — matériel, réglages, essais précédents validés.
- **Procédure** — déroulé des opérations.
- **Critères de réussite (go)** — comportement attendu pour valider
  l'essai.
- **Critères d'arrêt immédiat (abort)** — signes qui imposent d'interrompre
  l'essai sur-le-champ.
- **Vérifications post-essai** — ce qu'on relit dans les logs / observe sur
  le matériel après coup.

---

## 3. E0 — Essais banc (hors mouvement)

Réalisés avion posé au sol, **hélice démontée pour l'ensemble de la
série** : aucun essai E0 ne produit de poussée réelle, donc aucun besoin
de retenir physiquement l'avion. L'hélice n'est montée qu'à partir de
E1.0, premier essai en conditions réelles de vibration moteur.

### E0.1 — Séquence de mise sous tension

- **Objectif** : vérifier l'état de repos sûr à la mise sous tension.
- **Prérequis** : avion assemblé, batterie propulsion débranchée ou
  hélice démontée.
- **Procédure** : mettre sous tension Pi puis ESP32 (ou ordre inverse),
  observer les sorties avant toute réception de trame valide.
- **Critères de réussite** : gaz maintenus au minimum, servos en position
  neutre, mode manuel actif par défaut.
- **Critères d'arrêt** : tout mouvement de gouverne ou de gaz non
  neutre/nul à la mise sous tension.
- **Vérifications post-essai** : néant (essai très bref).

### E0.2 — Calibration complète

- **Objectif** : valider le déroulé complet de la calibration des capteurs
  qui en nécessitent une (gyroscope, accéléromètre, baromètre) via la
  console dédiée.
- **Prérequis** : E0.1 validé, avion immobile, capteurs alimentés.
- **Procédure** : dérouler la séquence de calibration de chaque capteur
  concerné via la console de calibration, en respectant les conditions
  requises par capteur (immobilité, mise à niveau, etc.).
- **Critères de réussite** : calibration acceptée et stockée pour chaque
  capteur, horodatage cohérent, calibration signalée valide par la
  checklist d'armement.
- **Critères d'arrêt** : échec de calibration répété, incohérence entre
  calibration affichée et attendue.
- **Vérifications post-essai** : présence et horodatage des fichiers de
  calibration stockés.

### E0.3 — Arbitrage manuel / autopilote

- **Objectif** : vérifier la bascule instantanée entre mode manuel et
  autopilote dans les deux sens, y compris la réaction de sécurité du Pi
  lors d'une reprise manuelle.
- **Prérequis** : E0.1 validé, réception RC et liaison Pi actives.
- **Procédure** : basculer l'interrupteur d'arbitrage plusieurs fois, dans
  les deux sens, d'abord gaz au neutre puis avec le système armé en
  autopilote.
- **Critères de réussite** : bascule instantanée, sans à-coup ni délai
  perceptible ; en mode manuel, restitution fidèle des mouvements RC ; en
  mode autopilote, prise en compte des consignes Pi ; **une reprise
  manuelle depuis l'autopilote déclenche l'état `ArretUrgence` côté Pi**
  (coupure moteur, état atteint visible dans les logs).
- **Critères d'arrêt** : bascule qui ne prend pas effet ou effet retardé ;
  absence de passage en `ArretUrgence` lors de la reprise manuelle.
- **Vérifications post-essai** : cohérence de l'état de mode remonté par
  l'ESP32 avec la position réelle de l'interrupteur ; état de sécurité
  atteint dans les logs.

### E0.4 — Sens et débattement des servos

- **Objectif** : valider le sens de braquage et le débattement de chaque
  gouverne via la séquence d'activation unitaire des servos exécutée par
  la logique d'armement (`mission/armement.rs`) — item bloquant
  "Vérification pré-vol" de la checklist (`doc/mission.md`). Ce n'est pas
  un essai isolé : cette même séquence s'exécute à chaque tentative
  d'armement, pour toute la durée du projet. E0.4 est la première
  observation attentive de cette séquence, avant de lui faire confiance
  comme verrou d'armement pour la suite.
- **Prérequis** : E0.3 validé.
- **Procédure** : déclencher la séquence d'armement, observer l'activation
  unitaire de chaque servo (ailerons, profondeur, direction) telle que
  pilotée automatiquement, confirmer visuellement sens et débattement.
- **Critères de réussite** : séquence exécutée sans blocage, sens et
  débattement conformes pour chaque axe, item "Vérification pré-vol"
  validé dans la checklist.
- **Critères d'arrêt** : sens inversé, débattement asymétrique ou
  incomplet, séquence interrompue.
- **Vérifications post-essai** : néant.

### E0.5 — Checklist d'armement à blanc

- **Objectif** : vérifier que l'armement n'est proposé que lorsque tous
  les items bloquants sont satisfaits, et que les items avertissements
  sont correctement acquittables. L'item "Vérification pré-vol" (sens et
  débattement des servos) n'est pas rejoué ici — déjà couvert par E0.4.
  Jalon important : au-delà de ce point, la MAÉ vol peut changer d'état et
  ouvre l'accès aux premiers essais de mission.
- **Prérequis** : E0.2 et E0.4 validés, capteurs branchés et alimentés.
- **Procédure** : dérouler la checklist dans différentes configurations
  (calibration absente, capteur débranché, GPS sans fix, etc.) et vérifier
  le blocage ou déblocage de la commande d'armement en conséquence.
- **Critères de réussite** : commande d'armement indisponible tant qu'un
  item bloquant n'est pas satisfait ; disponible une fois tous les
  bloquants au vert et les avertissements acquittés.
- **Critères d'arrêt** : armement possible alors qu'un item bloquant est
  en défaut.
- **Vérifications post-essai** : néant.

### E0.6 — Mission minimale de bout en bout

- **Objectif** : valider le déroulé complet d'une mission, aussi triviale
  soit-elle, de l'armement jusqu'à un désarmement et une sauvegarde des
  données **normaux** (pas une procédure d'urgence), gaz maintenus nuls du
  début à la fin. Ce jalon valide toute la plomberie (checklist, armement,
  transitions d'état, désarmement, clôture de la boîte noire) sans aucun
  risque physique, avant d'introduire des gaz réels en E0.7.
- **Prérequis** : E0.5 validé.
- **Procédure** : charger une mission minimale (ex. temporisation immobile
  de quelques secondes, sans déplacement prévu), dérouler la checklist,
  armer, lancer la mission, laisser la MAÉ vol transiter jusqu'à une fin
  normale (`Desarmement` puis `SauvegardeDonnees`), gaz maintenus nuls tout
  du long.
- **Critères de réussite** : enchaînement des états conforme au diagramme
  nominal jusqu'à `Desarmement → SauvegardeDonnees`, gaz restés nuls,
  flush complet de la boîte noire.
- **Critères d'arrêt** : blocage ou saut d'état inattendu, gaz non nuls
  alors qu'aucune consigne ne le justifie, log tronqué.
- **Vérifications post-essai** : trace complète des transitions d'état
  dans les logs, intégrité du fichier de log.

### E0.7 — Point fixe moteur (hélice démontée)

- **Objectif** : valider le passage réel jusqu'à `RoulageAuSol` avec une
  consigne de gaz non nulle réellement appliquée au variateur. Hélice
  démontée comme le reste de la série E0 : pas de poussée produite, donc
  aucun besoin de retenir l'avion. Sert de socle aux essais suivants
  (coupure gaz). Aucune sécurité ne limite la consigne de gaz dans l'état
  `RoulageAuSol`.
- **Prérequis** : E0.6 validé, hélice démontée, consignes de sécurité
  moteur rappelées (le moteur tourne réellement, même sans hélice).
- **Procédure** : dérouler la checklist, armer, lancer une mission de
  roulage, une fois en `RoulageAuSol` monter progressivement la consigne
  de gaz.
- **Critères de réussite** : transition `Armement → RoulageAuSol`
  conforme, consigne de gaz réellement appliquée au variateur (moteur
  tourne en conséquence).
- **Critères d'arrêt** : échec de la transition d'état, consigne non
  appliquée au variateur.
- **Vérifications post-essai** : trace d'état MAE vol dans les logs,
  consigne de gaz appliquée.

### E0.8 — Coupure gaz forcée, sources logicielles

- **Objectif** : vérifier que les commandes d'arrêt émises depuis le Pi
  (sans intervention RC) appliquent bien une consigne gaz nulle sans
  délai. Les sources RC (bascule manuelle en E0.3, perte de signal en
  E0.9) ne sont pas rejouées ici.
- **Prérequis** : E0.7 validé (point fixe moteur établi).
- **Procédure** : à partir du point fixe (gaz non nuls, autopilote,
  armé), déclencher séparément : (a) arrêt d'urgence depuis la console
  opérateur, (b) désarmement logiciel normal.
- **Critères de réussite** : gaz ramenés à zéro dans les deux cas, sans
  délai perceptible.
- **Critères d'arrêt** : absence de coupure ou délai de coupure notable.
- **Vérifications post-essai** : horodatage de la commande d'arrêt vs
  horodatage de la coupure effective dans les logs.

### E0.9 — Liaison RC : portée et perte de signal

- **Objectif** : vérifier la qualité du lien RC sur la distance
  d'utilisation prévue, puis la réaction logicielle à sa perte.
- **Prérequis** : E0.8 validé.
- **Procédure** :
  a) *Portée* — s'éloigner progressivement de l'appareil, moteur à
     l'arrêt puis moteur en marche à un régime modéré représentatif du
     point fixe (E0.7/E0.8) — sans hélice, un régime de croisière réel
     ferait survitesser le moteur à vide — en surveillant la qualité de
     réception (absence de saut ou de gigue sur les canaux) jusqu'à la
     distance d'utilisation prévue.
  b) *Perte de signal* — couper l'émetteur (ou l'éloigner suffisamment)
     en mode manuel puis en mode autopilote, gaz non nuls dans les deux
     cas.
- **Critères de réussite** : (a) aucune perte de trame perceptible jusqu'à
  la distance prévue, avec et sans moteur en marche ; (b) gaz ramenés à
  zéro, surfaces mises à plat, flag de perte RC remonté au Pi, dans les
  deux modes.
- **Critères d'arrêt** : (a) dégradation du signal en deçà de la distance
  prévue ; (b) absence de réaction après le délai attendu, dans l'un ou
  l'autre mode.
- **Vérifications post-essai** : (b) délai réel de détection, état de
  sécurité atteint ; (a) qualité de lien observée par palier de distance.

### E0.10 — Watchdog liaison Pi ↔ ESP32

- **Objectif** : vérifier la réaction à une perte de liaison série.
- **Prérequis** : E0.9 validé.
- **Procédure** : couper volontairement la liaison Pi → ESP32 (ex.
  débrancher ou arrêter le processus côté Pi) pendant que le système est
  en mode autopilote, gaz non nuls.
- **Critères de réussite** : gaz ramenés à zéro et surfaces mises à plat
  après le délai de garde attendu, flag de perte de liaison remonté dès
  rétablissement.
- **Critères d'arrêt** : absence de réaction après le délai attendu.
- **Vérifications post-essai** : délai réel de détection comparé au délai
  attendu, présence du flag dans la trame de remontée.

### E0.11 — Dégradation capteur à chaud

- **Objectif** : vérifier la réaction de la couche sûreté à la
  dégradation d'un capteur en cours de fonctionnement.
- **Prérequis** : E0.10 validé, tous capteurs opérationnels.
- **Procédure** :
  a) *GPS* — dégrader la réception sans aucune manipulation matérielle,
     en plaçant simplement l'antenne sans vue dégagée du ciel (sous abri,
     à l'intérieur), hors armement puis en tentant un armement.
  b) *Capteurs I2C (IMU, baromètre, télémètre)* — essai ponctuel et
     unique (pas une manipulation à répéter à chaque session) : couper
     l'alimentation ou débrancher un capteur pendant le fonctionnement,
     hors armement puis en tentant un armement. Si le matériel permet une
     coupure moins invasive qu'un débranchement de connecteur (interrupteur
     dédié, alimentation séparée par capteur), la préférer.
- **Critères de réussite** : état du capteur bascule vers `Dégradé` ou
  `HorsService` selon le cas, état de sécurité atteint (`AlerteMineure` ou
  `AlerteMajeure`) cohérent avec la table de `doc/surete.md`, armement
  bloqué si tenté pendant la dégradation.
- **Critères d'arrêt** : absence de détection de la dégradation, ou
  armement possible malgré un capteur dégradé ou hors service.
- **Vérifications post-essai** : trace de l'état capteur et de l'état de
  sécurité dans les logs.

### E0.12 — Reprise rapide après redémarrage logiciel

- **Objectif** : vérifier qu'un redémarrage du programme (simulant un
  crash logiciel) permet une reprise rapide sans recalibration complète.
- **Prérequis** : E0.2 validé, capteurs opérationnels.
- **Procédure** : arrêter brutalement le programme (kill) puis le
  relancer, capteurs restant alimentés.
- **Critères de réussite** : reprise rapide des drivers (purge et
  resynchronisation), calibrations rechargées depuis la flash sans
  recalibration manuelle, retour à un état cohérent en un délai bref.
- **Critères d'arrêt** : recalibration exigée à tort, incohérence d'état
  après reprise, délai de reprise excessif.
- **Vérifications post-essai** : logs de reprise (état par capteur), délai
  mesuré.

---

## 4. E1 — Roulage à faible vitesse

### E1.0 — Vibrations moteur au sol (hélice montée)

- **Objectif** : vérifier que l'estimation d'attitude reste stable sous
  vibration moteur réelle, avant tout essai de roulage. Premier essai de
  la série E1 où l'hélice est montée — jusqu'ici (E0), elle était
  démontée par précaution.
- **Prérequis** : E0 validé intégralement, hélice montée, avion calé ou
  retenu physiquement, zone dégagée.
- **Procédure** : faire tourner le moteur au régime de croisière prévu,
  avion calé et immobile, observer l'estimation d'attitude sur la durée.
- **Critères de réussite** : attitude estimée stable (pas de dérive ni de
  bruit excessif) malgré les vibrations réelles.
- **Critères d'arrêt** : divergence ou bruit excessif de l'estimation
  d'attitude sous vibration ; calage insuffisant (l'avion tend à
  avancer).
- **Vérifications post-essai** : bruit et dérive de l'attitude estimée
  dans les logs, comparés au relevé hélice démontée (E0.7).

### E1.1 — Roulage rectiligne, maintien de cap

- **Objectif** : valider le maintien de cap au sol à très faible vitesse.
- **Prérequis** : E0 et E1.0 validés, piste ou zone dégagée rectiligne.
- **Procédure** : armer, faire rouler l'avion en ligne droite à faible
  vitesse sur quelques dizaines de mètres, en autopilote.
- **Critères de réussite** : cap maintenu sans oscillation ni dérive
  excessive, corrections de gouverne cohérentes avec l'écart de cap.
- **Critères d'arrêt** : divergence de cap, oscillations croissantes,
  absence de réaction de la gouverne.
- **Vérifications post-essai** : trace de cap vs consigne dans les logs,
  amplitude des commandes de direction.

### E1.2 — Virages au sol à faible vitesse

- **Objectif** : valider les changements de cap commandés à faible
  vitesse. Essai conditionné à la largeur du site disponible — non requis
  pour valider les essais suivants (E1.3 et au-delà), à réaliser dès
  qu'un site adapté est disponible.
- **Prérequis** : E1.1 validé.
- **Procédure** : faire rouler l'avion en enchaînant des consignes de cap
  différentes (virages à grand rayon).
- **Critères de réussite** : convergence propre vers chaque nouveau cap,
  sans dépassement excessif ni instabilité.
- **Critères d'arrêt** : dépassement important, oscillation entretenue.
- **Vérifications post-essai** : trace de cap vs consigne, temps de
  convergence.

### E1.3 — Perte de signal RC en mouvement

- **Objectif** : confirmer en mouvement le comportement déjà validé au
  banc en E0.9(b) (coupure gaz, surfaces à plat, flag remonté), en cas de
  perte réelle du signal RC — distinct d'une reprise manuelle volontaire
  (E1.4).
- **Prérequis** : E1.1 validé (E1.2 non requis — voir sa note de
  portée).
- **Procédure** : pendant un roulage automatique, couper l'émetteur RC
  (ou l'éloigner suffisamment) pour provoquer une perte réelle de signal.
- **Critères de réussite** : coupure moteur immédiate, surfaces mises à
  plat, flag de perte RC remonté, comportement conforme à E0.9(b).
- **Critères d'arrêt** : tout retard ou absence de coupure — rétablir le
  signal et reprendre la main sans délai.
- **Vérifications post-essai** : délai réel de détection, horodatage vs
  coupure effective, état de sécurité atteint dans les logs.

### E1.4 — Reprise manuelle en cours de roulage automatique

- **Objectif** : confirmer en mouvement le comportement déjà validé au
  banc en E0.3 (bascule instantanée, passage en `ArretUrgence`).
- **Prérequis** : E1.3 validé.
- **Procédure** : pendant un roulage automatique, basculer l'interrupteur
  d'arbitrage en position manuelle et reprendre le contrôle au manche.
- **Critères de réussite** : bascule instantanée, contrôle manuel effectif
  sans interférence de l'autopilote, `ArretUrgence` atteint côté Pi comme
  en E0.3.
- **Critères d'arrêt** : délai de bascule perceptible, comportement
  incohérent avec les commandes manuelles.
- **Vérifications post-essai** : cohérence de l'état de mode remonté avec
  la bascule effectuée.

### E1.5 — Fin de session : désarmement et sauvegarde des données

- **Objectif** : valider la séquence de fin de session (désarmement,
  clôture de la boîte noire). Confirme en conditions réelles, après un
  roulage effectif, ce qui a déjà été validé en logique pure (gaz nuls)
  par E0.6.
- **Prérequis** : E1.1, E1.3 et E1.4 validés (E1.2 non requis — voir sa
  note de portée).
- **Procédure** : à l'issue d'un roulage, ramener l'avion à vitesse nulle
  et laisser la MAÉ vol transiter vers `Desarmement` puis
  `SauvegardeDonnees` (ou déclencher le désarmement explicitement selon
  le mode retenu).
- **Critères de réussite** : transition propre, flush complet de la boîte
  noire, aucune donnée manquante en fin de fichier de log.
- **Critères d'arrêt** : transition bloquée, log tronqué ou corrompu.
- **Vérifications post-essai** : intégrité du fichier de log ; relancer le
  programme avant la session suivante (voir convention §2).

---

## 5. E2 — Roulage à vitesse croissante

### E2.1 — Vitesse intermédiaire, maintien de cap

- **Objectif** : valider la robustesse du maintien de cap à une vitesse
  significativement supérieure à E1.
- **Prérequis** : E1.0, E1.1, E1.3, E1.4 et E1.5 validés (E1.2 non requis
  — voir sa note de portée).
- **Procédure** : roulage en ligne droite à vitesse intermédiaire,
  répété plusieurs fois.
- **Critères de réussite** : maintien de cap comparable à E1.1 malgré la
  vitesse plus élevée.
- **Critères d'arrêt** : dégradation du comportement par rapport à E1.1
  (oscillations, dérive).
- **Vérifications post-essai** : comparaison des traces de cap E1 vs E2.

### E2.2 — Approche de la vitesse de rotation, sans décollage

- **Objectif** : valider le comportement à l'approche de la vitesse de
  décollage sans quitter le sol.
- **Prérequis** : E2.1 validé, piste suffisamment longue pour accélérer
  puis freiner/couper en sécurité.
- **Procédure** : accélération franche jusqu'à une vitesse proche de la
  vitesse de rotation, sans lever la profondeur, puis réduction des gaz.
- **Critères de réussite** : maintien de cap conservé sous accélération,
  réduction des gaz sans réaction indésirable.
- **Critères d'arrêt** : perte de contrôle directionnel, début de
  décollage non souhaité.
- **Vérifications post-essai** : trace de vitesse sol vs cap.

### E2.3 — Arrêt de course au décollage (transition d'état normale)

- **Objectif** : valider la transition `CourseDecollage → RoulageAuSol`
  lors d'un abandon normal (pas une procédure d'urgence, cf. E2.4) —
  décision de ne pas décoller, réduction des gaz, retour au roulage.
  Comble le diagramme de `doc/mission.md`, qui ne documente aujourd'hui
  que la progression avant. Pas de nouvel état de décélération : l'avion
  n'étant pas freiné, la décélération se fait par simple réduction des
  gaz et roulis libre.
- **Prérequis** : E2.2 validé.
- **Procédure** : entrer en `CourseDecollage` (accélération franche comme
  E2.2), déclencher un abandon normal (commande d'abandon, pas un arrêt
  d'urgence), observer le retour à `RoulageAuSol` et la décélération.
- **Critères de réussite** : transition d'état conforme, gaz réduits
  progressivement (pas de coupure brutale), maintien de cap conservé
  pendant la décélération (`controleur_direction_sol` reprend la main).
- **Critères d'arrêt** : transition non prise en compte, perte de
  contrôle directionnel pendant la décélération.
- **Vérifications post-essai** : trace d'état MAE vol
  (`CourseDecollage → RoulageAuSol`), trace vitesse/gaz.

### E2.4 — Rejet de décollage (urgence)

- **Objectif** : valider la capacité à interrompre une accélération en
  urgence avant décollage — distinct de E2.3 (abandon normal, sans
  urgence).
- **Prérequis** : E2.3 validé.
- **Procédure** : lancer une accélération comme en E2.2, puis déclencher
  un arrêt d'urgence ou une reprise manuelle en cours d'accélération.
- **Critères de réussite** : coupure moteur immédiate, arrêt de l'appareil
  dans la distance de piste restante.
- **Critères d'arrêt** : coupure retardée, distance d'arrêt insuffisante.
- **Vérifications post-essai** : horodatage commande vs coupure, distance
  parcourue après coupure (estimation).

---

## 6. E3 — Décollage et vol rasant en ligne droite

- **Objectif** : valider le décollage et le maintien d'une trajectoire
  rectiligne à basse hauteur (quelques mètres), avant de chercher la
  montée en altitude.
- **Prérequis** : E2 validé intégralement, dégagement suffisant devant et
  au-delà de la piste.
- **Procédure** : décollage en autopilote, maintien d'assiette et de cap
  en ligne droite à basse hauteur sur une portion de trajectoire, sans
  virage ni changement d'altitude volontaire, avant repose ou remise de
  gaz manuelle.
- **Critères de réussite** : décollage sans divergence de cap ni
  d'assiette, tenue d'attitude stable en ligne droite.
- **Critères d'arrêt** : divergence d'assiette, oscillation non amortie,
  toute situation nécessitant une reprise manuelle immédiate.
- **Vérifications post-essai** : traces d'assiette (roulis/tangage) vs
  consignes, réactivité et amortissement des corrections.

---

## 7. E4 — Virages en vol

- **Objectif** : valider les changements de cap coordonnés en vol.
- **Prérequis** : E3 validé intégralement.
- **Procédure** : en vol stabilisé, enchaîner des consignes de virage à
  grand rayon, puis progressivement plus serrés.
- **Critères de réussite** : virage coordonné (sans dérapage excessif),
  retour stable en ligne droite après chaque virage.
- **Critères d'arrêt** : dérapage important, perte d'altitude non
  maîtrisée en virage, divergence d'assiette.
- **Vérifications post-essai** : traces d'assiette et de cap pendant les
  virages.

---

## 8. E5 — Navigation, altitude, position

Étape plus lointaine, détaillée à un niveau volontairement plus général :
elle sera précisée à l'approche de sa réalisation.

- **Objectif** : valider le suivi de plan de vol (waypoints), le maintien
  d'altitude, et le comportement en attente (loiter).
- **Prérequis** : E4 validé intégralement.
- **Procédure** : à définir (suivi d'un trajet simple à deux ou trois
  points, puis loiter).
- **Critères de réussite** : à définir.
- **Critères d'arrêt** : sortie de geofence, perte de contrôle
  d'altitude — critères précis à définir.
- **Vérifications post-essai** : trace de trajectoire vs plan de vol.

---

## 9. E6 — Atterrissage

- **Objectif** : valider la séquence d'approche, de flare et de roulage à
  l'atterrissage.
- **Prérequis** : E5 validé (ou au minimum E4 selon le mode d'approche
  retenu).
- **Procédure** : approche stabilisée, déclenchement du flare, roulage à
  l'arrêt.
- **Critères de réussite** : toucher des roues stabilisé, sans taux de
  descente excessif au flare, roulage à l'arrêt maîtrisé (rejoint les
  critères de E1/E2).
- **Critères d'arrêt** : approche instable, flare déclenché trop tôt/tard,
  toute situation nécessitant une reprise manuelle.
- **Vérifications post-essai** : trace de hauteur (télémètre) déclenchant
  le flare, taux de descente au toucher.

---

## 10. Hors périmètre (v1)

Sources de déclenchement sécurité identifiées (`doc/surete.md`) mais sans
matériel ou logique en place pour l'instant — à intégrer au plan dès
qu'elles deviennent pertinentes, plutôt qu'à tester prématurément :

- **Géofence** : logique implémentée (`surete/limites.rs`, hystérésis) et
  testée unitairement, mais pas encore validée par un essai réel — à
  couvrir dès que la navigation (E5) est en place pour la déclencher en
  conditions réelles.
- **Batterie critique** : aucun capteur de tension au catalogue matériel
  v1 (`systeme/energie.rs`).
- **Arrêt d'urgence et télémétrie via LoRa** : deuxième canal de sécurité
  prévu pour la phase où le contact visuel opérateur ne peut plus être
  garanti (vol autonome hors vue directe). Tant que les essais restent en
  vue directe avec la RC comme canal de sécurité principal, ce lien n'a
  pas besoin d'être validé dans ce plan. À couvrir (portée, télémétrie en
  mouvement, détection de perte de liaison, arrêt d'urgence via LoRa)
  lorsque cette phase sera abordée.

---

## 11. Annexe — Modèle de compte rendu d'essai

Pour chaque session :

- Date, lieu, météo (vent, visibilité, température)
- Participants et rôles (pilote RC, opérateur console, observateurs)
- Essai(s) réalisé(s), numéro(s) de fiche
- Résultat : validé / à rejouer / échec, avec description des écarts
  observés
- Anomalies rencontrées (même mineures)
- Référence du fichier de log associé
- Décision : passage à l'essai suivant / répétition / retour développement
