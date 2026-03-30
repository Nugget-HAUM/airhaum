# AirHaum II

## Présentation

**AirHaum II** est un projet de pilote automatique expérimental visant à rendre un avion radiocommandé partiellement ou totalement autonome.

Le projet est développé en **Rust** et s'appuie sur une architecture modulaire inspirée des systèmes avioniques modernes (PX4, ArduPilot), avec un fort accent sur :
- la **séparation des responsabilités**,
- la **testabilité**,
- la **sûreté de fonctionnement**,
- et la **lisibilité long terme**.

Ce dépôt constitue à la fois :
- un projet personnel avancé,
- un terrain d'expérimentation technique,
- et une base saine pour un système embarqué critique non certifié.

---

## Objectifs du projet

- Acquisition fiable de capteurs (IMU, baromètre, GPS, télémètre)
- Estimation de l'état de l'appareil (attitude, position, vitesse)
- Contrôle de vol (stabilisation, altitude, navigation)
- Gestion de missions autonomes (décollage, navigation, atterrissage)
- Communication sol ↔ bord (télémétrie, commandes, arrêt d'urgence)
- Mise en œuvre de mécanismes de sûreté (failsafe, watchdog, limites)

---

## Non-objectifs

- Certification avionique (DO-178, DO-254)
- Temps réel dur garanti (RTOS)
- Compatibilité avec des flottes hétérogènes
- Interface utilisateur complète type GCS

---

## Plateforme matérielle cible

- **Orange Pi Zero** sous Armbian (Linux)
- **MPU9265** (IMU) – I2C
- **BMP280** (baromètre) – I2C
- **NEO-M8N** (GPS) – UART
- **VL53L0X** (télémètre) – I2C
- **SX1276** (LoRa) – SPI
- **Arduino Nano** – UART (commande servos + secours radio)
- *(Optionnel)* ESP32-CAM – UART (odométrie visuelle)

---

## Architecture logicielle

Le projet est structuré en couches clairement séparées :

1. **HAL** – Abstraction du matériel (I2C, SPI, UART, GPIO)
2. **Drivers** – Implémentations spécifiques des capteurs
3. **Capteurs** – Prétraitement et fusion bas niveau
4. **Estimation** – Estimation d'état (EKF, attitude, position)
5. **Contrôle** – Régulation (PID, mixage, consignes)
6. **Mission** – Logique de haut niveau et machine à états
7. **Communication** – Protocole, télémétrie, commandes
8. **Système** – Configuration, journalisation, watchdog
9. **Tâches** – Orchestration asynchrone

Chaque couche dépend uniquement des couches inférieures, selon le principe d'inversion de dépendances.

---

## Philosophie de conception

- Utilisation de **types forts** (unités, horodatage, géométrie)
- Interfaces définies par des **traits Rust**
- Code testable avec mocks matériels
- Gestion explicite des erreurs
- Priorité absolue à la sûreté sur la mission

---

## Statut

Le projet est en **développement actif**. Certaines briques sont fonctionnelles (HAL, drivers, types fondamentaux), tandis que d'autres sont en cours de conception.

---

## Avertissement

Ce projet pilote un aéronef réel.

⚠ **Toute utilisation sur un appareil volant se fait sous l'entière responsabilité de l'utilisateur.**

---

## Licence

Projet personnel – licence à définir.

