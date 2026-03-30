_______________________
Arborescence détaillée: 


airhaum/

├── Cargo.lock                      # ok
├── Cargo.toml                      # ok
├── lisezmoi.md                     # ok
├── doc/                            # ok
│   ├── architecture.md             # ok:Architecture du projet 
│   ├── gestion_du_temps.md         # ok:Doc de la gestion du temps
│   ├── initialisation.md           # ok:Doc initialisation et calibration capteurs
│   ├── projet.md                   # ok:Doc générale du projet
│    
├── config/                         # A FAIRE
│   ├── default.toml                # A FAIRE Configuration par défaut
│   ├── calibration.toml            # A FAIRE Données de calibration
│   
├── src/                            # ok
│   ├── lib.rs                      # ok:Export des modules publiques
│   │
│   ├── bin/                        # ok:Liste des binaires
│   │   ├── airhaum-vol.rs          # ok:Binaire de vol autonome
│   │   ├── airhaum-test.rs         # ok:Console de test / diagnostic
│   │   └── airhaum-debug.rs        # A FAIRE Binaire phase de développement
│   │
│   ├── diagnostiques/              # ok:Fonctions de diagnostic matériel
│   │   ├── mod.rs                  # ok
│   │   ├── diag_bmp280.rs          # ok
│   │   ├── diag_gps.rs             # A FAIRE
│   │   └── diag_vl53lox.rs         # ok
│   │
│   ├── types/                      # ok:Types fondamentaux (pas de dépendances)
│   │   ├── mod.rs                  # ok
│   │   ├── etat_capteur.rs         # ok:Machine a état des capteurs
│   │   ├── messages.rs             # ok:SystemMessage, SensorData, etc.
│   │   ├── geometrie.rs            # ok:Vector3, Quaternion, etc.
│   │   ├── constantes.rs           # ok:Constantes physiques.
│   │   └── erreurs.rs              # ok:Erreurs personnalisées
│   │
│   ├── utilitaires/                # A FAIRE:Utilitaires (pas de dépendances métier)
│   │   ├── mod.rs                  # A FAIRE:
│   │   ├── maths.rs                # A FAIRE:Fonctions math courantes
│   │   └── filtres.rs              # A FAIRE:Filtres passe-bas, etc.
│   │
│   ├── interfaces/                 # ok:Traits (dépend uniquement de types/)
│   │   ├── mod.rs                  # ok
│   │   ├── imu.rs                  # A FAIRE:Trait CentraleInertielle
│   │   ├── gps.rs                  # A FAIRE:Trait Gps
│   │   ├── barometre.rs            # ok:Trait barometre
│   │   ├── telemetre.rs            # ok:Trait télémètre
│   │   ├── vision.rs               # A FAIRE:Trait interface système de vision
│   │   ├── estimateur.rs           # A FAIRE:Trait estimateur d’état de l'appareil 
│   │   ├── controleur.rs           # A FAIRE:Trait consignes de vol, moteur 
│   │   └── actionneurs.rs          # A FAIRE:Trait dispositifs qui agissent physiquement
│   │
│   ├── hal/                        # ok:Couche 1A: Hardware Abstraction Layer (ne doit jamais connaître les drivers)
│   │   ├── mod.rs                  # ok
│   │   ├── i2c.rs                  # ok:Trait BusI2c (abstraction) + Mock pour tests
│   │   ├── i2c_linux.rs            # ok:Implémentation spécifique à linux de BusI2c (/dev/i2c-X)
│   │   ├── uart.rs                 # A FAIRE:Abstraction des communications série.
│   │   ├── spi.rs                  # A FAIRE:Abstraction du bus SPI
│   │   └── gpio.rs                 # A FAIRE:Accès générique aux broches GPIO.
│   │
│   ├── drivers/                    # ok:Couche 1B: Drivers spécifiques (dépend de hal/ et interfaces/)
│   │   ├── mod.rs                  # ok
│   │   ├── imu/                    # ok
│   │   │   ├── mod.rs              # ok
│   │   │   ├── mpu.rs              # ok
│   │   │   └── calibration.rs      # ok
│   │   ├── gps/                    # A FAIRE
│   │   │   ├── mod.rs              # A FAIRE
│   │   │   ├── analyseur_nmea.rs   # A FAIRE
│   │   │   └── ublox.rs            # A FAIRE
│   │   ├── barometre/              # ok
│   │   │   ├── mod.rs              # ok
│   │   │   ├── calibration.rs      # ok
│   │   │   └── bmp280.rs           # ok
│   │   ├── telemetre/              # ok
│   │   │   ├── mod.rs              # ok
│   │   │   ├── calibration.rs      # ok
│   │   │   └── vl53l0x.rs          # ok
│   │   ├── lora/                   # A FAIRE 
│   │   │   ├── mod.rs              # A FAIRE 
│   │   │   └── sx1276.rs           # A FAIRE 
│   │   ├── controleur_servo.rs     # A FAIRE Communication Arduino
│   │   └── vision.rs               # A FAIRE Trait pour future ESP32-CAM
│   │
│   ├── capteurs/                   # A FAIRE Couche 2: Fusion capteurs bas niveau (Pré traitement capteurs)
│   │   ├── mod.rs                  # A FAIRE
│   │   ├── traitement_imu.rs       # A FAIRE
│   │   ├── traitement_gps.rs       # A FAIRE
│   │   ├── fusion_altitude.rs      # A FAIRE
│   │   ├── traitement_telemetre.rs # A FAIRE
│   │   └── etat_capteurs.rs        # A FAIRE
│   │
│   ├── estimation/                 # A FAIRE Couche 3: Estimation d'état
│   │   ├── mod.rs                  # A FAIRE
│   │   ├── ekf.rs                  # A FAIRE Extended Kalman Filter
│   │   ├── attitude.rs             # A FAIRE Estimation attitude
│   │   ├── position.rs             # A FAIRE
│   │   └── types.rs                # A FAIRE EtatDrone, etc.
│   │
│   ├── controle/                   # A FAIRE Couche 4: Controle
│   │   ├── mod.rs                  # A FAIRE
│   │   ├── pid.rs                  # A FAIRE Implémentation PID générique
│   │   ├── controleur_attitude.rs  # A FAIRE
│   │   ├── controleur_altitude.rs  # A FAIRE 
│   │   ├── controleur_position.rs  # A FAIRE
│   │   ├── controleur_vitesse.rs   # A FAIRE
│   │   └── mixeur.rs               # A FAIRE Conversion commandes → servos
│   │
│   ├── mission/                    # A FAIRE Couche 5: Mission
│   │   ├── mod.rs                  # A FAIRE
│   │   ├── modes_de_vol.rs         # A FAIRE Enum et types
│   │   ├── etat_machine.rs         # A FAIRE Machine à états
│   │   ├── decollage.rs            # A FAIRE 
│   │   ├── atterrissage.rs         # A FAIRE
│   │   ├── navigation.rs           # A FAIRE 
│   │   ├── geofence.rs             # A FAIRE 
│   │   └── failsafe.rs             # A FAIRE 
│   │
│   ├── communications/             # A FAIRE Couche 6: Communication
│   │   ├── mod.rs                  # A FAIRE 
│   │   ├── protocole.rs            # A FAIRE Définition protocole
│   │   ├── telemetrie.rs           # A FAIRE 
│   │   ├── commandes.rs            # A FAIRE 
│   │   └── surveillance_lien.rs    # A FAIRE 
│   │
│   ├── surete/                     # A FAIRE
│   │   ├── mod.rs                  # A FAIRE
│   │   ├── armement.rs             # A FAIRE
│   │   ├── limites.rs              # A FAIRE 
│   │   ├── sante.rs                # A FAIRE
│   │   └── urgences.rs             # A FAIRE 
│   │
│   ├── systeme/                    # A FAIRE Couche 7: Services système
│   │   ├── mod.rs                  #  
│   │   ├── armement.rs             # A FAIRE vérification/autorisation décollage
│   │   ├── journalisation.rs       # A FAIRE Boite noire / enregistrement données
│   │   ├── watchdog.rs             # A FAIRE Surveillance santé système
│   │   ├── calibration.rs          # Gestion des calibration persistantes
│   │   ├── configuration.rs        # A FAIRE Chargement/sauvegarde config
│   │   ├── temps.rs                # A FAIRE Gestion temps système, timestamps
│   │   └── energie.rs              # A FAIRE Surveillance batterie
│   │
│   └── taches/                     # A FAIRE Taches async (exécution)
│       ├── mod.rs                  # A FAIRE 
│       ├── taches_capteurs.rs      # A FAIRE 
│       ├── taches_estimation.rs    # A FAIRE
│       ├── taches_controle.rs      # A FAIRE
│       ├── taches_mission.rs       # A FAIRE
│       ├── taches_communication.rs # A FAIRE
│       ├── taches_surete.rs        # A FAIRE
│       └── taches_systeme.rs       # A FAIRE
│ 
├── tests/                          # A FAIRE
│   ├── unitaires/                  # A FAIRE Tests unitaires
│   │   ├── tests_capteurs.rs       # A FAIRE
│   │   ├── tests_controle.rs       # A FAIRE
│   │   └── tests_estimation.rs     # A FAIRE
│   └── integration/                # A FAIRE Tests d'intégration
│       ├── tests_fusion_capteurs.rs# A FAIRE
│       ├── tests_controle.rs       # A FAIRE 
│       └── tests_mission.rs        # A FAIRE
│
└── logs/                           # Logs et boite noire (gitignored: ignoré par Git...)




1 - Couche HAL (Hardware Abstraction Layer) : Drivers pour chaque périphérique avec traits Rust permettant le mock pour tests
2 - Couche Capteurs : Fusion de données (IMU + GPS + baro), filtres de Kalman
3-  Couche Estimation d'état : Position, attitude, vitesse estimées
4 - Couche Contrôle : PID/contrôleurs pour stabilisation et navigation
5 - Couche Mission : Logique de haut niveau (décollage, waypoints, atterrissage)
6 - Couche Communication : LoRa télémétrie et commandes
7 - Couche System (Services transversaux)


