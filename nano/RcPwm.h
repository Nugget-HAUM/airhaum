// RcPwm.h — Lecture PWM individuel de RCPWM_NB canaux RC (ATmega328P)
//
// Brochage : A1(CH1-ail), A2(CH2-pro), A3(CH3-gaz), A4(CH4-dir), A5(CH7-SWA)
// Pins PC1-PC5 contigus → PCINT1_vect, RC_OFFSET = 1.
// Plage valide : 800–2200 µs (RC standard : 1000–2000 µs)
//
// Un seul ISR (PCINT1_vect) couvre les 5 canaux simultanément.
// Non bloquant : la boucle principale lit les dernières valeurs mémorisées.

#pragma once
#include <stdint.h>

#define RCPWM_NB  5   // A1..A5 (PC1..PC5)

void     rcpwm_init();
uint16_t rcpwm_lire(uint8_t canal);  // largeur pulse µs (0 si jamais reçu)
bool     rcpwm_pret();               // vrai si tous les canaux ont reçu ≥1 pulse
uint8_t  rcpwm_nb_prets();           // nombre de canaux ayant reçu ≥1 pulse (debug)
uint32_t rcpwm_ts_dernier();         // millis() du dernier pulse valide reçu
