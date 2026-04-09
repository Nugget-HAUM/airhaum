# Suivi du projet — AirHaum II

---

## Prochaines étapes

- [x] Refonte architecturale de la couche capteurs I2C (threads synchrones dédiés)
- [x] Migration du BMP280 en mode normal 
- [ ] Mise en place Machine à état avion ?
- [ ] Système de journalisation (architecture à définir — voir ci-dessous)
- [ ] Filtre de Kalman — prédiction IMU
- [ ] Filtre de Kalman — corrections baro / GPS
- [ ] Contrôleur d'attitude
- [ ] Contrôleur d'altitude
- [ ] Couche mission (points de passage, modes de vol)

---

## Dettes techniques

- [ ] **Arrêt des threads non-réactif** : `arreter()` positionne un drapeau mais un thread
  en veille longue (30 s, cas réinitialisation max) ne le voit qu'à son réveil.
  À corriger avant mise en production.

- [ ] **Vivacité des threads** : un thread qui panique silencieusement n'est pas détecté.
  La couche sûreté devra vérifier `JoinHandle::is_finished()` périodiquement.

---

## Points d'architecture à trancher

- [ ] **Journalisation** : niveaux, destination (fichier rotatif ? mémoire circulaire ?),
  impact sur les threads à haute fréquence (200 Hz IMU ne doit pas être ralenti).
  Piste : canal interne vers un thread dédié à l'écriture.

---

## Évolutions matérielles a achever :

- GPS UART à brancher sur PI 
- Arduino à brancher sur PI 
- Récepteur RC a relier sur arduino 
- ESC / moteurs (PWM  à relier sur arduino 

