# Suivi du projet — AirHaum II

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
- [ ] Contrôleur d'attitude
- [ ] Contrôleur d'altitude
- [ ] Couche mission — la MAÉ vol (états/transitions) est faite ; reste la logique
      de navigation (points de passage, modes de vol) au-dessus

---

## Dettes techniques
- [ ] ** Fréquence cilbe ** le drivers MPU tourne a 125-110hz et non 200 comme visé

- [ ] **Arrêt des threads non-réactif** : `arreter()` positionne un drapeau mais un thread
  en veille longue (30 s, cas réinitialisation max) ne le voit qu'à son réveil.
  À corriger avant mise en production.

- [ ] **Vivacité des threads** : un thread qui panique silencieusement n'est pas détecté.
  La couche sûreté devra vérifier `JoinHandle::is_finished()` périodiquement.

- panic silencieux : Si un thread capteur panique, le mutex peut être empoisonné → erreur I2C → réinit en boucle.

- Pas de gestion des erreurs I2C critiques

- [x] Validation des données GPS — checksum UBX/NMEA rejeté avec resync
  (`ubx_parser.rs`), fix non valide ignoré par la fusion (`type_fix.est_valide()`
  dans `taches_estimation.rs`). Pas encore de garde-fou sur les valeurs
  elles-mêmes (position aberrante, saut de vitesse).

- [ ] **VL53L0X : pas de récupération logicielle en cas de verrouillage interne**
  XSHUT est câblé en dur sur le 3V3 (pas de GPIO) : si le capteur se verrouille
  en interne (ACK sur l'adresse I²C mais échec de toute lecture registre —
  observé le 2026-07-08, résolu par débranchement/rebranchement physique),
  seul un cycle d'alimentation peut le débloquer. De plus, `thread_vl53l0x`
  (`taches_capteurs.rs`) ne distingue pas ce cas d'une erreur I²C transitoire :
  il boucle indéfiniment (réinit + backoff + suspension 30s) sans jamais
  remonter d'alerte à la couche sûreté (`surete/sante.rs`, pas encore
  implémenté). À corriger en deux temps : câbler XSHUT sur une GPIO pour
  permettre un reset matériel logiciel, puis faire remonter l'état
  "capteur irrécupérable" à la sûreté après échec du reset.

---

## Points d'architecture à trancher

- [x] **Journalisation** : implémentée dans `src/systeme/journalisation.rs`,
  voir `doc/journalisation.md`. Chemin de sortie configurable via
  `AIRHAUM_LOGS_DIR` (défaut `/home/airhaum/logs`).

---

## Évolutions matérielles a achever :

- [x] GPS branché sur PI (`/dev/ttyS1`, UART1, 115 200 bauds)
- [x] Arduino branché sur PI (`/dev/ttyS2`, 57 600 bauds) — liaison testée logiciellement
- Récepteur RC : câblage individuel PWM (A1–A5) à faire sur le Nano — voir
  `doc/interface_pi_rc_servos.md` et `doc/recepteur_rc.md` (IBus abandonné)
- ESC / moteurs (PWM) à relier sur arduino

