// nano/nano.ino
// Contrôleur servo / arbitrage RC — AirHaum II
// Cible : Arduino Nano (ATmega328P)
//
// Câblage — voir doc/interface_pi_rc_servos.md :
//   A1 ← CH1 récepteur RC (ailerons)       CHn → An pour CH1-CH4
//   A2 ← CH2 récepteur RC (profondeur)
//   A3 ← CH3 récepteur RC (gaz)
//   A4 ← CH4 récepteur RC (direction)
//   A5 ← CH7 récepteur RC (SWA — mode auto/manuel)
//   D0  (RX0)  ← Pi TX  (consignes, 57 600 bauds, via adaptateur 3,3V/5V)
//   D1  (TX0)  → Pi RX  (remontée état, 57 600 bauds, via adaptateur 3,3V/5V)
//   D3           → servo ailerons
//   D5           → servo profondeur
//   D6           → variateur (gaz)
//   D9           → servo direction  (D10 défectueux sur ce Nano)
//
// LED pin 13 :
//   1 Hz  = aucun canal RC reçu
//   2 Hz  = signal partiel (1–4 canaux) ou RC perdu
//   4 Hz  = RC OK (5 canaux, trames en cours)

#include <Servo.h>
#include "RcPwm.h"

// ─── Liaison Pi TX sur D1 (TX0 matériel, 57 600 bauds) ──────────────────────
// UART matériel : pas de cli(), aucune interférence avec Timer1 (Servo) ni PCINT1 (RcPwm).
// RX consignes Pi sur D0, TX remontée sur D1.

// ─── Broches ─────────────────────────────────────────────────────────────────

static const uint8_t BROCHE_AIL  = 3;
static const uint8_t BROCHE_PRO  = 5;
static const uint8_t BROCHE_GAZ  = 6;
static const uint8_t BROCHE_DIR  = 9;
static const uint8_t BROCHE_LED  = 13;

// ─── Protocole Pi ↔ Nano ─────────────────────────────────────────────────────

static const uint8_t  DEBUT_CONSIGNE  = 0xAA;
static const uint8_t  DEBUT_REMONTEE  = 0xBB;
static const uint8_t  TAILLE_CONSIGNE = 11;
// Remontée : 0xBB + fanions + 5 canaux×2 + gaz_applique×2 + checksum = 15 octets
static const uint8_t  TAILLE_REMONTEE = 15;

static const uint16_t US_MIN    = 1000;
static const uint16_t US_NEUTRE = 1500;
static const uint16_t US_MAX    = 2000;

static const uint32_t DELAI_CHIEN_DE_GARDE_MS = 2000UL;
static const uint32_t DELAI_RC_PERDU_MS       = 1000UL;
static const uint32_t PERIODE_REMONTEE_MS     =  100UL;

// ─── Objets ──────────────────────────────────────────────────────────────────

Servo servo_ailerons, servo_profondeur, servo_direction, servo_gaz;

// ─── État ────────────────────────────────────────────────────────────────────

static uint16_t cons_ailerons   = US_NEUTRE;
static uint16_t cons_profondeur = US_NEUTRE;
static uint16_t cons_gaz        = US_MIN;
static uint16_t cons_direction  = US_NEUTRE;
static bool     arme            = false;

static const uint8_t CH_AIL        = 0;  // A1 ← CH1
static const uint8_t CH_PRO        = 1;  // A2 ← CH2
static const uint8_t CH_GAZ        = 2;  // A3 ← CH3
static const uint8_t CH_DIR        = 3;  // A4 ← CH4
static const uint8_t CH_SWA        = 4;  // A5 ← CH7 (interrupteur auto/manuel)
static const uint8_t NB_CANAUX_RC  = 5;

static uint16_t canaux_rc[NB_CANAUX_RC];

static bool     mode_manuel    = false;
static bool     rc_perdu       = false;
static bool     chien_de_garde = false;
static uint16_t gaz_applique   = US_MIN;
static bool     rc_actif       = false;   // vrai dès que tous les canaux ont reçu ≥1 pulse

static uint32_t ts_trame_pi = 0;
static uint32_t ts_remontee = 0;
static uint32_t ts_led      = 0;
static bool     led_etat    = false;

static uint8_t  tampon_pi[TAILLE_CONSIGNE];
static uint8_t  pos_tampon = 0;

// ─── Utilitaire ──────────────────────────────────────────────────────────────

static inline uint16_t clipper_us(uint16_t v) {
    if (v < US_MIN) return US_MIN;
    if (v > US_MAX) return US_MAX;
    return v;
}

// ─── Réception trame Pi ──────────────────────────────────────────────────────

static void traiter_trame_consigne(const uint8_t *t) {
    uint8_t checksum = 0;
    for (uint8_t i = 1; i < 10; i++) checksum ^= t[i];
    if (checksum != t[10]) return;

    cons_ailerons   = clipper_us((uint16_t)t[2] | ((uint16_t)t[3] << 8));
    cons_profondeur = clipper_us((uint16_t)t[4] | ((uint16_t)t[5] << 8));
    cons_gaz        = clipper_us((uint16_t)t[6] | ((uint16_t)t[7] << 8));
    cons_direction  = clipper_us((uint16_t)t[8] | ((uint16_t)t[9] << 8));
    arme            = (t[1] & 0x01) != 0;
    ts_trame_pi     = millis();
}

static void lire_consignes_pi() {
    while (Serial.available() > 0) {
        uint8_t b = (uint8_t)Serial.read();
        if (pos_tampon == 0 && b != DEBUT_CONSIGNE) continue;
        tampon_pi[pos_tampon++] = b;
        if (pos_tampon == TAILLE_CONSIGNE) {
            traiter_trame_consigne(tampon_pi);
            pos_tampon = 0;
        }
    }
}

// ─── Lecture canaux RC via PWM individuel (A1-A5) ────────────────────────────

static void lire_rc() {
    if (rcpwm_pret()) {
        for (uint8_t i = 0; i < NB_CANAUX_RC; i++)
            canaux_rc[i] = rcpwm_lire(i);
        rc_actif = true;
    }

    rc_perdu    = rc_actif && ((millis() - rcpwm_ts_dernier()) > DELAI_RC_PERDU_MS);
    mode_manuel = (canaux_rc[CH_SWA] < US_NEUTRE);  // SWA bas = manuel, haut = autopilote
}

// ─── Application des consignes ───────────────────────────────────────────────

static void appliquer_consignes() {
    chien_de_garde = (millis() - ts_trame_pi) > DELAI_CHIEN_DE_GARDE_MS;

    uint16_t ail, pro, gaz, dir;

    if (mode_manuel) {
        ail = rc_perdu ? US_NEUTRE : canaux_rc[CH_AIL];
        pro = rc_perdu ? US_NEUTRE : canaux_rc[CH_PRO];
        dir = rc_perdu ? US_NEUTRE : canaux_rc[CH_DIR];
        gaz = rc_perdu ? US_MIN    : canaux_rc[CH_GAZ];
    } else {
        ail = cons_ailerons;
        pro = cons_profondeur;
        dir = cons_direction;
        gaz = (arme && !chien_de_garde && !rc_perdu) ? cons_gaz : US_MIN;
    }

    gaz_applique = gaz;
    servo_ailerons.writeMicroseconds(ail);
    servo_profondeur.writeMicroseconds(pro);
    servo_direction.writeMicroseconds(dir);
    servo_gaz.writeMicroseconds(gaz);
}

// ─── Envoi trame remontée ────────────────────────────────────────────────────

static void envoyer_remontee() {
    if ((millis() - ts_remontee) < PERIODE_REMONTEE_MS) return;
    ts_remontee = millis();

    uint8_t fanions = 0;
    if (mode_manuel)    fanions |= 0x01;
    if (rc_perdu)       fanions |= 0x02;
    if (chien_de_garde) fanions |= 0x04;

    uint8_t t[TAILLE_REMONTEE];
    t[0] = DEBUT_REMONTEE;
    t[1] = fanions;
    for (uint8_t i = 0; i < NB_CANAUX_RC; i++) {
        t[2 + i * 2]     =  canaux_rc[i] & 0xFF;
        t[2 + i * 2 + 1] = (canaux_rc[i] >> 8) & 0xFF;
    }
    // NB_CANAUX_RC=5 → octets 2..11, gaz_applique en 12-13
    t[12] =  gaz_applique & 0xFF;
    t[13] = (gaz_applique >> 8) & 0xFF;

    uint8_t checksum = 0;
    for (uint8_t i = 1; i < 14; i++) checksum ^= t[i];
    t[14] = checksum;

    Serial.write(t, TAILLE_REMONTEE);
}

// ─── LED ─────────────────────────────────────────────────────────────────────

static void battre_led() {
    uint32_t periode;
    if (!rc_actif) {
        // 1 Hz = 0 canal, 2 Hz = 1–4 canaux reçus
        periode = (rcpwm_nb_prets() == 0) ? 500UL : 250UL;
    } else {
        periode = rc_perdu ? 250UL : 125UL;
    }
    if ((millis() - ts_led) >= periode) {
        ts_led   = millis();
        led_etat = !led_etat;
        digitalWrite(BROCHE_LED, led_etat ? HIGH : LOW);
    }
}

// ─── Setup / Loop ────────────────────────────────────────────────────────────

void setup() {
    pinMode(BROCHE_LED, OUTPUT);
    for (uint8_t i = 0; i < 6; i++) {
        digitalWrite(BROCHE_LED, HIGH); delay(100);
        digitalWrite(BROCHE_LED, LOW);  delay(100);
    }

    Serial.begin(57600);
    rcpwm_init();                      // RC PWM sur A1-A5 via PCINT1

    servo_ailerons.attach(BROCHE_AIL);    // D3
    servo_profondeur.attach(BROCHE_PRO);  // D5
    servo_gaz.attach(BROCHE_GAZ);         // D6
    servo_direction.attach(BROCHE_DIR);   // D9

    servo_ailerons.writeMicroseconds(US_NEUTRE);
    servo_profondeur.writeMicroseconds(US_NEUTRE);
    servo_gaz.writeMicroseconds(US_MIN);
    servo_direction.writeMicroseconds(US_NEUTRE);

    for (uint8_t i = 0; i < NB_CANAUX_RC; i++) canaux_rc[i] = US_NEUTRE;
    canaux_rc[CH_GAZ] = US_MIN;
    canaux_rc[CH_SWA] = US_MIN;   // SWA bas au démarrage → mode manuel par défaut

    uint32_t now = millis();
    ts_trame_pi  = now;
    ts_remontee  = now;
}

void loop() {
    lire_rc();
    lire_consignes_pi();
    appliquer_consignes();
    envoyer_remontee();
    battre_led();
}
