// RcIbus.h — Lecture des canaux RC au format iBUS (FlySky) sur un UART matériel
//
// Trame iBUS (récepteur → ESP32, 115 200 bauds 8N1, ~7 ms) : 32 octets
//   [0]      0x20  longueur de trame
//   [1]      0x40  commande « canaux »
//   [2..29]  14 canaux × 2 octets, petit-boutiste (1000–2000 µs)
//   [30..31] somme de contrôle = 0xFFFF − Σ octets[0..29], petit-boutiste
//
// Non bloquant : rcibus_traiter() consomme les octets disponibles à chaque
// tour de loop() ; la boucle principale lit les dernières valeurs validées.

#pragma once
#include <Arduino.h>

#define RCIBUS_NB_CANAUX  14

void     rcibus_init(HardwareSerial &port, int8_t broche_rx, int8_t broche_tx);
void     rcibus_traiter();                  // à appeler à chaque tour de loop()
uint16_t rcibus_lire(uint8_t canal);        // µs, 0 si aucune trame reçue
bool     rcibus_pret();                     // vrai dès la première trame valide
uint32_t rcibus_ts_derniere();              // millis() de la dernière trame valide
uint32_t rcibus_nb_rejetees();              // trames rejetées (somme de contrôle) — debug
