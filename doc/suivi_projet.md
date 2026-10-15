# Suivi du projet — AirHaum II

---

## Processus — fin de brique

Avant de commit un module/brique terminé : relire la checklist "Prochaines
étapes" ci-dessous et la section correspondante de `doc/architecture.md`
(statuts fichier par fichier) — pas seulement le doc dédié à la couche
concernée (ex. `doc/controle.md`). Ces deux fichiers de suivi transverses
ne se mettent pas à jour tout seuls.

---

## Prochaines étapes

- [x] Refonte architecturale de la couche capteurs I2C (threads synchrones dédiés)
- [x] Migration du BMP280 en mode normal
- [x] Machine à états de vol (MAÉ) — Boot → AttenteArmement + reprise rapide
- [x] **Estimation d'état**
  - [x] **[A] Prétraitement IMU** — `src/capteurs/traitement_imu.rs`
  - [x] **[B] Attitude — filtre complémentaire** — `src/estimation/attitude.rs`
  - [x] **[C] Thread d'estimation** — `src/taches/taches_estimation.rs` (branché dans `airhaum-test`)
  - [x] **[D] Fusion altitude** — `src/capteurs/fusion_altitude.rs`
  - [x] **[E] EKF** — `src/estimation/ekf_attitude.rs` + `src/estimation/ekf_navigation.rs`
        (13 états : position/vitesse NED, quaternion, biais gyro — nommage final différent
        du `ekf.rs` unique prévu initialement, mais même rôle)
- [x] Calibration — `src/systeme/calibration.rs` + console dédiée `console_calibration.rs`
- [x] Système de journalisation — voir `doc/journalisation.md`, implémenté et opérationnel
- [x] **Contrôleur d'attitude** — `src/controle/controleur_roulis.rs`, `controleur_tangage.rs`,
      `controleur_lacet.rs` (cascade angle→taux, voir `doc/controle.md`)
- [ ] Contrôleur d'altitude
- [x] **Tâche mission, câblée** — `src/taches/taches_mission.rs` câble la
      MAÉ vol (`etat_machine.rs`) et le plan de vol (`plan_vol.rs`, segments
      Sol) : boucle 10 Hz, canal de commande console (`Armer`/
      `ArmementValide`/`Desarmer` distincts), dérivation
      `PhaseControle`/`tx_arme` — voir `doc/mission.md`, section « Tâche
      mission ». Branchée dans `airhaum_test/main.rs` : plan de vol chargé
      et sélectionné depuis la check-liste (`console_armement.rs`, dossier
      `/home/airhaum/plans_de_vol`), armer/lancer/désarmer déclenchés
      depuis ce même écran (pas le menu de test). Premier test réel (E0.5
      checklist) mené sur cible (Orange Pi) le 2026-09-01, concluant.
      `taches_controle.rs` également branchée (`doc/controle.md`) : les
      consignes de la mission (gaz, cap au sol) atteignent réellement
      l'Arduino via le mixeur. Reste à faire : test E0.6 complet (mission
      minimale de bout en bout) sur cible, puis E0.7 (point fixe moteur,
      débloqué par ce branchement) ; gains des contrôleurs et altitude
      cible réels (provisoires pour l'instant, `ALTITUDE_CIBLE_M` compris) ;
      `airhaum-vol.rs` toujours un stub.
- [ ] Couche mission — reste `decollage.rs`, `atterrissage.rs`, et la logique
      de navigation (points de passage, modes de vol, phase Vol — E5+) au-dessus

---

## Dettes techniques
- [ ] ** Fréquence cilbe ** le drivers MPU tourne a 125-110hz et non 200 comme visé

- [ ] **Arrêt des threads non-réactif** : `arreter()` positionne un drapeau mais un thread
  en veille longue (30 s, cas réinitialisation max) ne le voit qu'à son réveil.
  À corriger avant mise en production.

- [ ] **Vivacité des threads** : un thread qui panique silencieusement n'est pas détecté.
  La couche sûreté devra vérifier `JoinHandle::is_finished()` périodiquement.

- [ ] **`MaeSecurite::reinitialiser()` jamais appelé en pratique** : la mémoire
  d'incident (`deja_degrade_ce_vol`, plafond de rétablissement à `AlerteMineure`)
  et l'arrêt d'urgence console (`arret_urgence_console` dans `airhaum-test`)
  ne peuvent aujourd'hui être effacés qu'en relançant le binaire. Sans
  conséquence tant que la convention reste 1 vol = 1 lancement du programme —
  **et qu'un roulage au sol compte comme un vol à part entière** : on relance
  le programme entre un essai de roulage et le vol qui suit, jamais les deux
  dans le même lancement. C'est ce qui garantit qu'un incident survenu
  pendant un roulage (plafond `AlerteMineure`, non bloquant pour
  `surete::armement::surete_nominale()`) ne peut pas passer inaperçu jusqu'à
  un armement ultérieur dans la même session programme.
  À traiter si cette hypothèse change (plusieurs vols — roulages compris —
  par lancement) : prévoir un point d'entrée explicite (ex. lors du prochain
  armement) qui appelle `reinitialiser()` et remet `arret_urgence_console` à
  `false`.

- panic silencieux : Si un thread capteur panique, le mutex peut être empoisonné → erreur I2C → réinit en boucle.

- Pas de gestion des erreurs I2C critiques

- [ ] **Lacet estimé par gyro seul, pas de fusion magnétomètre/cap GPS dans
  l'EKF** (`estimation/ekf_navigation.rs`). Le magnétomètre AK8963 est lu
  et calibré (`drivers/imu/mpu9250.rs`, hard/soft iron) mais jamais
  fusionné : les seules corrections de l'EKF sont `corriger_accel`
  (roulis/tangage via la gravité) et `corriger_gps` (position + vitesse
  NED). Le lacet dérive donc par intégration gyro pure entre deux
  prédictions ; la correction indirecte via le couplage vitesse/quaternion
  du modèle de prédiction est faible et quasi nulle à basse vitesse
  (roulage). Cohérent avec `doc/mission.md` qui documente ce choix comme
  acceptable pour la phase de test (calibration magnétomètre acquittable
  à l'armement). À améliorer dans un second temps : fusion magnétomètre
  et/ou cap déduit de la vitesse sol GPS, pour un lacet plus précis en
  vol. Ne bloque pas le développement de la couche contrôle (`doc/controle.md`),
  qui consomme `EtatNavigation.attitude.lacet` tel qu'il existe aujourd'hui.

- [x] Validation des données GPS — checksum UBX/NMEA rejeté avec resync
  (`ubx_parser.rs`), fix non valide ignoré par la fusion. Garde-fou sur les
  valeurs elles-mêmes (position aberrante, saut de vitesse) ajouté via
  `capteurs/traitement_gps.rs` (`TraitementGps`), branché dans
  `taches_estimation.rs` avant `EkfNavigation::corriger_gps`.

- [ ] **Pas de RTC à sauvegarde batterie — horloge système non fiable sans réseau**
  (reporté à la phase d'autonomie, acceptable pour la v1). Vérifié sur le Pi le
  2026-07-18 : `sun6i-rtc` (RTC intégrée au SoC) repart de ~epoch Unix à chaque
  démarrage (`dmesg` : `setting system clock to 1970-01-01T00:00:14 UTC`),
  `fake-hwclock` est masqué/inactif — seul `chronyd` (NTP) corrige l'horloge,
  et seulement si le réseau a une sortie internet. En v1, le programme est
  toujours lancé via une connexion SSH en Wifi (au pire point d'accès 4G d'un
  téléphone), donc une horloge valide au lancement est garantie en pratique —
  pas de correctif nécessaire pour l'instant. À traiter avant la phase
  autonome (lancement sans opérateur connecté), car ça impacte alors trois
  endroits identifiés dans le code :
  - `envoyer_time_utc()` (`drivers/gps/ublox.rs`) enverrait une heure fausse
    de plusieurs décennies au GPS comme aide `TIME_UTC` (au lieu d'une
    hypothèse à quelques secondes près comme le code le suppose).
  - `est_valide()` des calibrations baromètre/IMU (`drivers/{barometre,imu}/calibration.rs`)
    utilise `age_secondes() = maintenant.saturating_sub(timestamp_stocké)` :
    une horloge repartie en arrière fait remonter `saturating_sub` à 0, donc
    une calibration périmée est acceptée comme fraîche silencieusement.
  - `systeme/journalisation.rs` horodate chaque entrée et nomme les fichiers
    de log via `chrono::Local::now()` — logs datés vers 1970 en cas de vol
    autonome sans réseau, gênant pour l'analyse post-vol.
  Piste envisagée : extraire le calendrier UTC déjà présent dans NAV-PVT
  (offsets 4-11, non parsé actuellement par `ubx_parser.rs`) et régler
  l'horloge système au premier fix GPS valide.

- [ ] **VL53L0X : pas de récupération logicielle en cas de verrouillage interne**
  XSHUT est câblé en dur sur le 3V3 (pas de GPIO) : si le capteur se verrouille
  en interne (ACK sur l'adresse I²C mais échec de toute lecture registre —
  observé le 2026-07-08, résolu par débranchement/rebranchement physique),
  seul un cycle d'alimentation peut le débloquer. De plus, `thread_vl53l0x`
  (`taches_capteurs.rs`) ne distingue pas ce cas d'une erreur I²C transitoire :
  il boucle indéfiniment (réinit + backoff + suspension 30s) sans distinguer
  ce cas d'une erreur I²C transitoire aux yeux de la couche sûreté
  (`surete/sante.rs`, implémentée depuis, mais pas encore vérifiée sur ce cas
  précis). À corriger en deux temps : câbler XSHUT sur une GPIO pour
  permettre un reset matériel logiciel, puis faire remonter l'état
  "capteur irrécupérable" à la sûreté après échec du reset.

---

## Points d'architecture à trancher

- [x] **Journalisation** : implémentée dans `src/systeme/journalisation.rs`,
  voir `doc/journalisation.md`. Chemin de sortie configurable via
  `AIRHAUM_LOGS_DIR` (défaut `/home/airhaum/logs`).

- [ ] **Reprise autopilote après prise de main manuelle volontaire (hors urgence)** :
  aujourd'hui, toute reprise de main manuelle pendant que le système est
  armé/en mission déclenche `ArretUrgence` (`doc/surete.md`), état dont on
  ne sort qu'en relançant le programme (voir la note sur
  `MaeSecurite::reinitialiser()` ci-dessus). Aucun mécanisme ne permet à
  l'opérateur de rendre la main à l'autopilote après une reprise
  volontaire non liée à une urgence réelle (ex. évitement ponctuel d'un
  obstacle), sans redémarrer. À trancher : faut-il distinguer une reprise
  manuelle "urgence" d'une reprise "volontaire réversible", et si oui
  comment (nouvelle commande explicite de ré-armement de l'autopilote,
  distinction au niveau du canal RC, etc.) ?

---

## Évolutions matérielles a achever :

- [x] GPS branché sur PI (`/dev/ttyS1`, UART1, 115 200 bauds)
- [x] Arduino branché sur PI (`/dev/ttyS2`, 57 600 bauds) — liaison testée
      informellement, hors série d'essais formelle (`doc/plan_essais.md`)
- [x] **Firmware Nano codé** — `nano/nano.ino` implémente la lecture RC (5
      canaux PWM individuels A1–A5 via `nano/RcPwm.h`/`.cpp`, une seule ISR
      PCINT1), l'arbitrage manuel/autopilote local au Nano (canal SWA), la
      sortie servos + variateur (D3 ailerons, D5 profondeur, D6 gaz, D9
      direction), le chien de garde liaison Pi (2 s → gaz coupés, surfaces à
      plat), la sécurité perte RC (1 s → gaz coupés), et la remontée d'état
      à 10 Hz (canaux RC, mode actif, gaz appliqué) — voir
      `doc/interface_pi_rc_servos.md` et `doc/protocole_uart_arduino.md`.
      Câblé côté Pi : `src/drivers/controleur_servo.rs` +
      `src/taches/taches_servo.rs`, branchés dans `airhaum_test/main.rs`
      (`doc/architecture.md`). Reste à faire : câblage physique définitif
      du récepteur RC et de l'ESC/moteur sur le Nano si non déjà fait, et
      validation formelle par la série d'essais dédiée
      (`doc/plan_essais.md`, E0.3 arbitrage, E0.7 point fixe moteur, E0.8–E0.10
      coupure gaz/perte RC/watchdog liaison).
      **Remplacé par l'ESP32** (ci-dessous) après la clôture de E0.1 ;
      firmware archivé dans `doc/archives/nano/`.
- [x] **Contrôleur servo ESP32** (2026-10-15) — WEMOS D1 MINI ESP32
      (ESP32-WROOM-32), firmware `esp32/esp32.ino` + `esp32/RcIbus.*` :
      RC en iBUS sur UART matériel (fin du bruit de mesure PWM du Nano,
      `doc/essais/E0.1.md`), sorties servo/variateur LEDC matérielles,
      liaison Pi en 3,3 V direct (plus d'adaptateur de niveau). Même
      logique de sécurité que le Nano et protocole Pi inchangé — aucune
      modification côté Rust. Monté sur la platine, RC → servos validé,
      liaison Pi validée (check-liste d'armement : « Liaison Arduino + RC
      OK »). Voir `doc/interface_pi_rc_servos.md`. Reste à faire :
      re-vérifier la calibration des manches en iBUS
      (`doc/recepteur_rc.md`), valider le failsafe du récepteur (perte RC
      réellement détectée à la coupure de l'émetteur), puis rejouer la
      série E0 à partir de E0.1 sur la nouvelle chaîne.

