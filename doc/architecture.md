_______________________
Arborescence détaillée: 

A éditer avec : tree -I "target"



airhaum/

├── Cargo.lock                      # ok
├── Cargo.toml                      # ok
├── readme.md                       # ok
├── doc/                            # ok
│   ├── architecture.md             # ok: Architecture du projet 
│   ├── gestion_du_temps.md         # ok: Doc de la gestion du temps
│   ├── initialisation.md           # ok: Doc initialisation et calibration capteurs
│   ├── projet.md                   # ok: Doc générale du projet
│    
├── config/                         # A FAIRE
│   ├── default.toml                # A FAIRE: Configuration par défaut
│   ├── calibration.toml            # A FAIRE: Données de calibration
│   
├── src/                            # ok
│   ├── lib.rs                      # ok: Export des modules publics
│   │
│   ├── bin/                        # ok: Binaires
│   │   ├── airhaum-vol.rs          # A FAIRE: Binaire de vol autonome (construit progressivement depuis airhaum-test)
│   │   └── airhaum_test/           # ok: Console de test / diagnostic (éclatée en modules)
│   │       ├── main.rs                  # ok
│   │       ├── console_test.rs          # ok
│   │       ├── console_armement.rs      # ok
│   │       └── console_calibration.rs   # ok
│   │
│   ├── diagnostiques/              # ok: Fonctions de diagnostic matériel
│   │   ├── mod.rs                  # ok
│   │   ├── diag_bmp280.rs          # ok: Diagnostic du baromètre
│   │   ├── diag_mpu9250.rs         # ok: Diagnostic de la centrale inertielle
│   │   ├── diag_vl53l0x.rs         # ok: Diagnostic du télémètre
│   │   ├── diag_gps.rs             # ok: Diagnostic du GPS
│   │   └── diag_taches_capteurs.rs # ok: Diagnostic des tâches capteurs (fréquences, santé)
│   │
│   ├── drivers/                    # ok: Couche 1B: Drivers spécifiques (dépend de hal/ et interfaces/)
│   │   ├── mod.rs                  # ok
│   │   ├── imu/                    # ok
│   │   │   ├── mod.rs              # ok
│   │   │   ├── mpu9250.rs          # ok: Driver MPU-9250 (accéléro, gyro, magnéto)
│   │   │   └── calibration.rs      # ok
│   │   ├── gps/                    # ok
│   │   │   ├── mod.rs              # ok
│   │   │   ├── ublox.rs            # ok: Driver u-blox NEO-M8N (init, lecture, config UART)
│   │   │   └── ubx_parser.rs       # ok: Parser protocole UBX binaire (FSM, checksum)
│   │   ├── barometre/              # ok
│   │   │   ├── mod.rs              # ok
│   │   │   ├── calibration.rs      # ok
│   │   │   └── bmp280.rs           # ok: Driver BMP280
│   │   ├── telemetre/              # ok
│   │   │   ├── mod.rs              # ok
│   │   │   ├── calibration.rs      # ok
│   │   │   └── vl53l0x.rs          # ok: Driver VL53L0X
│   │   ├── lora/                   # A FAIRE 
│   │   │   ├── mod.rs              # A FAIRE 
│   │   │   └── sx1276.rs           # A FAIRE 
│   │   ├── controleur_servo.rs     # ok: Communication Arduino (protocole binaire, testé)
│   │   └── vision.rs               # A FAIRE: Trait pour future ESP32-CAM
│   │
│   ├── hal/                        # ok: Couche 1A: Hardware Abstraction Layer (ne doit jamais connaître les drivers)
│   │   ├── mod.rs                  # ok
│   │   ├── i2c.rs                  # ok: Trait BusI2c + I2cMock pour tests
│   │   ├── i2c_linux.rs            # ok: Implémentation Linux de BusI2c (/dev/i2c-X)
│   │   ├── uart.rs                 # ok: Trait PortSerie (abstraction UART)
│   │   ├── uart_linux.rs           # ok: Implémentation Linux de PortSerie (/dev/ttyX)
│   │   ├── spi.rs                  # A FAIRE: Abstraction du bus SPI
│   │   └── gpio.rs                 # A FAIRE: Accès générique aux broches GPIO
│   │
│   ├── interfaces/                 # ok: Traits (dépend uniquement de types/)
│   │   ├── mod.rs                  # ok
│   │   ├── imu.rs                  # ok: Trait CentraleInertielle
│   │   ├── gps.rs                  # ok: Trait Gps
│   │   ├── barometre.rs            # ok: Trait Barometre
│   │   ├── telemetre.rs            # ok: Trait Telemetre
│   │   ├── vision.rs               # A FAIRE: Trait système de vision
│   │   ├── estimateur.rs           # A FAIRE: Trait estimateur d'état
│   │   ├── controleur.rs           # A FAIRE: Trait consignes de vol
│   │   └── actionneurs.rs          # A FAIRE: Trait dispositifs actionneurs
│   │
│   ├── capteurs/                   # ok (partiel): Couche 2: Pré-traitement capteurs
│   │   ├── mod.rs                  # ok
│   │   ├── traitement_imu.rs       # ok: Pré-traitement IMU (biais, repère)
│   │   ├── fusion_altitude.rs      # ok: Fusion baro + télémètre
│   │   ├── traitement_gps.rs       # A FAIRE
│   │   ├── traitement_telemetre.rs # A FAIRE
│   │   └── etat_capteurs.rs        # A FAIRE
│   │
│   ├── estimation/                 # ok: Couche 3: Estimation d'état
│   │   ├── mod.rs                  # ok
│   │   ├── attitude.rs             # ok: Estimation attitude (filtre complémentaire)
│   │   ├── ekf_attitude.rs         # ok: EKF attitude (quaternion, 7 états)
│   │   └── ekf_navigation.rs       # ok: EKF navigation (position/vitesse NED, 13 états)
│   │
│   ├── controle/                   # A FAIRE: Couche 4: Contrôle
│   │   ├── mod.rs                  # A FAIRE
│   │   ├── pid.rs                  # A FAIRE: Implémentation PID générique
│   │   ├── controleur_attitude.rs  # A FAIRE
│   │   ├── controleur_altitude.rs  # A FAIRE 
│   │   ├── controleur_position.rs  # A FAIRE
│   │   ├── controleur_vitesse.rs   # A FAIRE
│   │   └── mixeur.rs               # A FAIRE: Conversion commandes → servos
│   │
│   ├── mission/                    # ok (partiel): Couche 5: Mission
│   │   ├── mod.rs                  # ok
│   │   ├── etat_machine.rs         # ok: Machine à états de vol
│   │   ├── decollage.rs            # A FAIRE 
│   │   ├── atterrissage.rs         # A FAIRE
│   │   ├── navigation.rs           # A FAIRE 
│   │   ├── geofence.rs             # A FAIRE 
│   │   └── failsafe.rs             # A FAIRE 
│   │
│   ├── communications/             # A FAIRE: Couche 6: Communication
│   │   ├── mod.rs                  # A FAIRE 
│   │   ├── protocole.rs            # A FAIRE: Définition protocole
│   │   ├── telemetrie.rs           # A FAIRE 
│   │   ├── commandes.rs            # A FAIRE 
│   │   └── surveillance_lien.rs    # A FAIRE 
│   │
│   ├── surete/                     # ok (partiel)
│   │   ├── mod.rs                  # ok: EtatSecurite, règles d'armement
│   │   ├── armement.rs             # A FAIRE
│   │   ├── limites.rs              # A FAIRE 
│   │   ├── sante.rs                # A FAIRE
│   │   └── urgences.rs             # A FAIRE 
│   │
│   ├── systeme/                    # ok (partiel): Couche 7: Services système
│   │   ├── mod.rs                  # ok
│   │   ├── calibration.rs          # ok: Gestion des calibrations persistantes (TOML)
│   │   ├── journalisation.rs       # ok: Boîte noire / fil de vie — voir doc/journalisation.md
│   │   ├── watchdog.rs             # A FAIRE: Surveillance santé système
│   │   ├── configuration.rs        # A FAIRE: Chargement/sauvegarde config
│   │   ├── temps.rs                # A FAIRE: Gestion temps système, timestamps
│   │   └── energie.rs              # A FAIRE: Surveillance batterie
│   │
│   ├── types/                      # ok: Types fondamentaux (pas de dépendances)
│   │   ├── mod.rs                  # ok
│   │   ├── etat_capteur.rs         # ok: Machine à états des capteurs
│   │   ├── messages.rs             # ok: MesureBaro, MesureImu, MesureTelem, etc.
│   │   ├── geometrie.rs            # ok: Vector3, Quaternion, etc.
│   │   ├── constantes.rs           # ok: Constantes physiques
│   │   ├── mesure_frequence.rs     # ok: Mesure de fréquence d'échantillonnage
│   │   └── erreurs.rs              # ok: ErreursAirHaum
│   │
│   ├── utilitaires/                # A FAIRE: Utilitaires (pas de dépendances métier)
│   │   ├── mod.rs                  # A FAIRE
│   │   ├── maths.rs                # A FAIRE: Fonctions mathématiques courantes
│   │   └── filtres.rs              # A FAIRE: Filtres passe-bas, etc.
│   │
│   └── taches/                     # ok (partiel): Tâches d'exécution
│       ├── mod.rs                  # ok  
│       ├── taches_capteurs.rs      # ok: Threads capteurs I²C + canaux watch/mpsc
│       ├── taches_estimation.rs    # ok: Tâche Tokio EKF navigation
│       ├── taches_gps.rs           # ok: Tâche Tokio lecture GPS UART
│       ├── taches_servo.rs         # ok: Thread liaison Arduino (envoi consignes / lecture état)
│       ├── taches_controle.rs      # A FAIRE
│       ├── taches_mission.rs       # A FAIRE
│       ├── taches_communication.rs # A FAIRE
│       ├── taches_surete.rs        # A FAIRE
│       └── taches_systeme.rs       # A FAIRE
│ 
├── tests/                          # ok (partiel)
│   ├── capteurs_architecture.rs    # ok: Tests mock (I2cMock) — canaux, handles, arrêt
│   ├── capteurs_materiels.rs       # ok: Tests matériel (#[ignore]) — fréquences réelles sur cible
│   └── ...                         # A FAIRE: Tests estimation, mission, etc.
│
└── logs/                           # Logs et boîte noire (gitignored)




1 - Couche HAL (Hardware Abstraction Layer) : Drivers pour chaque périphérique avec traits Rust permettant le mock pour tests
2 - Couche Capteurs : Fusion de données (IMU + GPS + baro), filtres de Kalman
3-  Couche Estimation d'état : Position, attitude, vitesse estimées
4 - Couche Contrôle : PID/contrôleurs pour stabilisation et navigation
5 - Couche Mission : Logique de haut niveau (décollage, waypoints, atterrissage)
6 - Couche Communication : LoRa télémétrie et commandes
7 - Couche System (Services transversaux)
