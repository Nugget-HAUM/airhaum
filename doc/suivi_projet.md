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

---

## Points d'architecture à trancher

- [x] **Journalisation** : implémentée dans `src/systeme/journalisation.rs`,
  voir `doc/journalisation.md`. Chemin de sortie configurable via
  `AIRHAUM_LOGS_DIR` (défaut `/home/airhaum/logs`).

---

## Évolutions matérielles a achever :

- GPS UART à brancher sur PI (`/dev/ttyS3` désactivé dans `hal/uart_linux.rs`,
  le port n'existe pas encore matériellement)
- [x] Arduino branché sur PI (`/dev/ttyS2`, 57 600 bauds) — liaison testée logiciellement
- Récepteur RC : câblage individuel PWM (A1–A5) à faire sur le Nano — voir
  `doc/interface_pi_rc_servos.md` et `doc/recepteur_rc.md` (IBus abandonné)
- ESC / moteurs (PWM) à relier sur arduino

