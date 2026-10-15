Commande Flysky 
Récepteur FlySky — sortie iBUS lue par l'ESP32 (UART2, IO23), voir
`doc/interface_pi_rc_servos.md`. Réglage émetteur : *RX Setup → Output
mode → Serial : i-BUS*.

Les valeurs ci-dessous ont été mesurées sur le FS-iA10B en PWM individuel,
lu par l'ancien Nano. L'iBUS transporte les mêmes valeurs de canaux, mais
elles restent à re-vérifier sur la nouvelle chaîne (traces USB de l'ESP32,
ou remontée côté Pi).

Canaux et valeurs mesurées (µs) :
CH1 : axe horizontal droite — ailerons    (1381 milieu, 900 gauche, 1879 droite)
CH2 : axe vertical droite  — profondeur  (1500 milieu, 1880 bas, 900 haut)
CH3 : gaz vertical gauche  — gaz         (1006 bas, 1994 haut)
CH4 : axe horizontal gauche — direction  (1504 milieu, 1013 gauche, 1980 droite)
CH5 : potard haut gauche VRA             (1000 à 2000)
CH6 : potard haut droit VRB              (1000 à 2000)
CH7 : interrupteur gauche SWA            (1000 = manuel, 2000 = autopilote)
CH8 : non configuré
CH9 : non configuré
CH10: non configuré 
CH11: non 
CH12: non 
CH13: non
