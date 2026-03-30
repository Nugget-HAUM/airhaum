AirHaum II 

Projet pour rendre un avion radio commandé autonome 
 

Matériel : 
- Orange pi Zero sous armbian
- MPU 9265 en I2c sur le pi
- BMP 280 en I2C sur le pi 
- NEO-M8N en UART sur le pi 
- VL53Lox I2C - pour les phases décollage/atterissage, sur le pi
- SX1276 SPI sur le pi - pour la télémétrie, commandes mission, et arrêt d'urgence
- Arduino nano en UART sur le pi  pour le contrôle des servo et l'interface avec la radio commande en arrêt d'urgence
- En option une ESP32Cam en UART pour de l'odométrie 

Logiciel : 
- Programme en RUST avec code francophone (notamment dans les noms de variable et fonction) 
- Runtime asynchrone: Tokio
- Communication inter-tâches : Hybride (critique: atomics ou structures lock-free, Commandes/événements : channels, élémétrie : broadcast)
- Représentation du temps et fréquences: types forts et fréquences synchronisées, Horodatage : Au niveau driver
- Gestion de la calibration : Détection automatique du contexte, calibration avant armement, éventuellement recalibration en vol
- Horodatage et synchronisation: à definir
- Protocole Arduino ↔ Orange Pi : Protocole custom binaire
