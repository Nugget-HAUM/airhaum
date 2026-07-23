// test_ibus_led.ino — v6
//
// Câblage :
//   D9  (RX IBus) ← i-BUS SERVO récepteur RC  (115 200 bauds)
//   D7  (TX debug) → FTDI RX                  (9 600 bauds)
//   GND → FTDI GND
//
// LED pin 13 :
//   5 clignements rapides au boot = firmware v6
//   1 Hz  = aucune trame IBus valide depuis le démarrage
//   2 Hz  = trames reçues puis signal perdu (>1 s)
//   4 Hz  = IBus OK
//
// Affichage debug (toutes les 500 ms) :
//   CH1..CH14 sur une ligne — bougez chaque axe/interrupteur pour identifier

#include <SoftwareSerial.h>

SoftwareSerial dbg(4, 7);      // TX=D7 → FTDI, RX=D4 (inutilisé)
SoftwareSerial ibus_ss(9, 12); // RX=D9 ← IBus SERVO, TX=D12 (inutilisé)

static const uint8_t  BROCHE_LED     = 13;
static const uint32_t DELAI_RC_PERDU = 1000UL;
static const uint32_t PERIODE_DEBUG  =  500UL;

static const uint8_t NB_CANAUX = 14;

// Parseur IBus manuel
static uint8_t  ibus_buf[32];
static uint8_t  ibus_pos      = 0;
static uint32_t ts_ibus_octet = 0;
static uint16_t canaux_rc[NB_CANAUX];
static uint32_t ts_trame_rc   = 0;
static bool     ibus_actif    = false;
static bool     rc_perdu      = true;
static uint32_t cnt_trames    = 0;

// LED
static uint32_t ts_led    = 0;
static bool     led_etat  = false;

// Debug
static uint32_t ts_debug  = 0;

static void lire_rc() {
    while (ibus_ss.available() > 0) {
        uint32_t now = millis();
        if (now - ts_ibus_octet > 3) ibus_pos = 0;
        ts_ibus_octet = now;

        uint8_t b = (uint8_t)ibus_ss.read();
        if (ibus_pos == 0 && b != 0x20) continue;
        if (ibus_pos == 1 && b != 0x40) { ibus_pos = 0; continue; }
        ibus_buf[ibus_pos++] = b;
        if (ibus_pos < 32) continue;
        ibus_pos = 0;

        uint16_t cs = 0xFFFF;
        for (uint8_t i = 0; i < 30; i++) cs -= ibus_buf[i];
        if (cs != ((uint16_t)ibus_buf[31] << 8 | ibus_buf[30])) continue;

        ibus_actif  = true;
        ts_trame_rc = millis();
        cnt_trames++;
        for (uint8_t i = 0; i < NB_CANAUX; i++)
            canaux_rc[i] = (uint16_t)ibus_buf[2 + i*2] | ((uint16_t)ibus_buf[3 + i*2] << 8);
    }
    rc_perdu = (millis() - ts_trame_rc) > DELAI_RC_PERDU;
}

static void battre_led() {
    uint32_t periode = !ibus_actif ? 500UL : (rc_perdu ? 250UL : 125UL);
    if ((millis() - ts_led) >= periode) {
        ts_led   = millis();
        led_etat = !led_etat;
        digitalWrite(BROCHE_LED, led_etat ? HIGH : LOW);
    }
}

static void afficher_debug() {
    if ((millis() - ts_debug) < PERIODE_DEBUG) return;
    ts_debug = millis();

    dbg.print(F("cnt="));
    dbg.print(cnt_trames);
    dbg.print(rc_perdu ? F(" PERDU") : F(" OK   "));
    dbg.print(F("  "));
    for (uint8_t i = 0; i < NB_CANAUX; i++) {
        dbg.print(F("CH"));
        dbg.print(i + 1);
        dbg.print('=');
        dbg.print(canaux_rc[i]);
        if (i < NB_CANAUX - 1) dbg.print(' ');
    }
    dbg.println();

    ibus_ss.listen();
}

void setup() {
    pinMode(BROCHE_LED, OUTPUT);
    // 5 clignements rapides = v6
    for (uint8_t i = 0; i < 5; i++) {
        digitalWrite(BROCHE_LED, HIGH); delay(80);
        digitalWrite(BROCHE_LED, LOW);  delay(80);
    }
    dbg.begin(9600);
    pinMode(4, INPUT_PULLUP);
    dbg.println(F("=== boot v6 (14 canaux IBus) ==="));
    dbg.println(F("Bougez chaque axe/switch pour identifier les canaux"));

    ibus_ss.begin(115200);
    ibus_ss.listen();
    ts_trame_rc = 0;
}

void loop() {
    lire_rc();
    battre_led();
    afficher_debug();
}
