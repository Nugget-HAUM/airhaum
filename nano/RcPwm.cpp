// RcPwm.cpp — Lecture non-bloquante de 5 canaux RC PWM par PCINT1
//
// Brochage contigu : A1(PC1)..A5(PC5), RC_OFFSET = 1.
// Canal i → bit PC (i + RC_OFFSET).
//
// Sur front montant : mémorise micros().
// Sur front descendant : calcule la largeur et la stocke si dans 800-2200 µs.
// Durée ISR : ~30 cycles → Timer1 (servos) retardé < 2 µs.

#include <avr/io.h>
#include <avr/interrupt.h>
#include <Arduino.h>
#include "RcPwm.h"

#define RC_PIN_REG  PINC
#define RC_DDR      DDRC
#define RC_PORT     PORTC
#define RC_OFFSET   1                                         // A1 = PC1 = premier canal
#define RC_MASK     (((1 << RCPWM_NB) - 1) << RC_OFFSET)   // 0x3E pour A1..A5

static volatile uint32_t rise_us[RCPWM_NB];
static volatile uint32_t confirmed_rise_us[RCPWM_NB]; // front montant de la dernière impulsion valide
static volatile uint16_t pulse_us[RCPWM_NB];
static volatile uint8_t  pret_mask;           // bit i = canal i a reçu ≥1 pulse
static volatile uint32_t ts_dernier_ms;
static volatile uint8_t  prev_pins;

#define RC_PERIODE_MIN_US 14000UL   // 71 Hz max
#define RC_PERIODE_MAX_US 30000UL   // 33 Hz min

void rcpwm_init()
{
    RC_DDR  &= ~RC_MASK;
    RC_PORT |=  RC_MASK;    // pull-ups (signal RC idle = HIGH)

    for (uint8_t i = 0; i < RCPWM_NB; i++) {
        rise_us[i]           = 0;
        confirmed_rise_us[i] = 0;
        pulse_us[i]          = 0;
    }
    pret_mask     = 0;
    ts_dernier_ms = 0;
    prev_pins     = RC_PIN_REG & RC_MASK;

    PCMSK1 = RC_MASK;
    PCIFR  = (1 << PCIF1);
    PCICR |= (1 << PCIE1);
}

uint16_t rcpwm_lire(uint8_t canal)
{
    if (canal >= RCPWM_NB) return 0;
    uint16_t v;
    uint8_t sreg = SREG; cli();
    v = pulse_us[canal];
    SREG = sreg;
    return v;
}

bool rcpwm_pret()
{
    return (pret_mask & ((1 << RCPWM_NB) - 1)) == ((1 << RCPWM_NB) - 1);
}

uint8_t rcpwm_nb_prets()
{
    uint8_t n = 0;
    for (uint8_t i = 0; i < RCPWM_NB; i++)
        if (pret_mask & (1 << i)) n++;
    return n;
}

uint32_t rcpwm_ts_dernier()
{
    uint32_t t;
    uint8_t sreg = SREG; cli();
    t = ts_dernier_ms;
    SREG = sreg;
    return t;
}

ISR(PCINT1_vect)
{
    uint32_t now_us  = micros();
    uint8_t  pins    = RC_PIN_REG & RC_MASK;
    uint8_t  changed = pins ^ prev_pins;
    prev_pins = pins;

    for (uint8_t i = 0; i < RCPWM_NB; i++) {
        uint8_t bit = (1 << (i + RC_OFFSET));
        if (!(changed & bit)) continue;
        if (pins & bit) {
            rise_us[i] = now_us;                // front montant : mémorise
        } else {
            uint32_t w       = now_us - rise_us[i];       // largeur d'impulsion
            uint32_t periode = rise_us[i] - confirmed_rise_us[i]; // période depuis dernière impulsion valide
            bool periode_ok  = (confirmed_rise_us[i] == 0)
                             || (periode >= RC_PERIODE_MIN_US && periode <= RC_PERIODE_MAX_US);
            if (w >= 800 && w <= 2200 && periode_ok) {
                pulse_us[i]          = (uint16_t)w;
                pret_mask           |= (1 << i);
                ts_dernier_ms        = millis();
                confirmed_rise_us[i] = rise_us[i]; // mémorise uniquement si impulsion validée
            }
        }
    }
}
