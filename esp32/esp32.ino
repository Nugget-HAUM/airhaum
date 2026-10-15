// esp32/esp32.ino
// Contrôleur servo / arbitrage RC — AirHaum II
// Cible : WEMOS D1 MINI ESP32 (ESP32-WROOM-32) — remplace doc/archives/nano/nano.ino
// Carte Arduino IDE : « WEMOS D1 MINI ESP32 » (core esp32 3.x ; 2.x accepté)
//
// Décision de plateforme : doc/essais/E0.1.md (clôture du 2026-10-05).
// Protocole Pi ↔ contrôleur inchangé : doc/protocole_uart_arduino.md —
// aucune modification nécessaire côté Rust (src/drivers/controleur_servo.rs).
//
// Câblage — voir doc/interface_pi_rc_servos.md (numéros GPIO) :
//   UART0  USB             : flash + traces de débogage (115 200 bauds)
//   UART1  IO26 (RX)  ← Pi TX  (consignes, 57 600 bauds, 3,3 V direct)
//          IO19 (TX)  → Pi RX  (remontée état, 57 600 bauds, 3,3 V direct)
//   UART2  IO23 (RX)  ← récepteur RC, sortie iBUS (115 200 bauds), sans TX
//   IO21  → servo ailerons      (CH1)
//   IO17  → servo profondeur    (CH2)
//   IO16  → variateur (gaz)     (CH3)
//   IO18  → servo direction     (CH4)
//
// LED intégrée IO2 :
//   1 Hz  = aucune trame iBUS reçue
//   2 Hz  = RC perdu (plus de trame iBUS valide depuis DELAI_RC_PERDU_MS)
//   4 Hz  = RC OK

#include "RcIbus.h"

// ─── Broches ─────────────────────────────────────────────────────────────────

static const int8_t  BROCHE_PI_RX    = 26;
static const int8_t  BROCHE_PI_TX    = 19;
static const int8_t  BROCHE_IBUS_RX  = 23;
// iBUS en réception seule : -1 = aucune broche TX. Évite que le core
// rattache UART2 à sa broche TX par défaut (IO17), utilisée ici en sortie servo.
static const int8_t  BROCHE_IBUS_TX  = -1;

static const uint8_t BROCHE_AIL      = 21;
static const uint8_t BROCHE_PRO      = 17; 
static const uint8_t BROCHE_GAZ      = 16;
static const uint8_t BROCHE_DIR      = 18;
static const uint8_t BROCHE_LED      = 2;

// ─── Ports série ─────────────────────────────────────────────────────────────

#define PORT_PI    Serial1
#define PORT_IBUS  Serial2
#define PORT_DEBUG Serial

// Traces lisibles sur l'USB (2 Hz) : n'interfèrent ni avec la liaison Pi
// ni avec l'iBUS, qui ont chacun leur UART. Commenter pour les retirer.
#define TRACES_USB

// ─── Protocole Pi ↔ contrôleur ───────────────────────────────────────────────

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

// Remontée à 50 Hz pour les campagnes de mesure (rejeu de E0.1) ; 10 Hz sinon.
// Avec l'iBUS, les valeurs remontées sont celles appliquées : pas de filtre.
// #define DIAGNOSTIC_REMONTEE_50HZ

#if defined(DIAGNOSTIC_REMONTEE_50HZ)
static const uint32_t PERIODE_REMONTEE_MS     =   20UL;
#else
static const uint32_t PERIODE_REMONTEE_MS     =  100UL;
#endif

// ─── Canaux RC ───────────────────────────────────────────────────────────────
// Index dans la trame de remontée (inchangés depuis le Nano) → canal iBUS.
// Canal d'arbitrage : CH7 (interrupteur SWA), comme sur le Nano. L'iBUS
// transporte jusqu'à 14 canaux, la limite de 6 voies du FS-iA6B ne vaut que
// pour ses sorties PWM. Si l'émetteur n'émet que 6 canaux, réaffecter SWA
// à CH5 ou CH6 (doc/essais/E0.1.md) et changer CANAL_IBUS_ARBITRAGE.

static const uint8_t CH_AIL        = 0;
static const uint8_t CH_PRO        = 1;
static const uint8_t CH_GAZ        = 2;
static const uint8_t CH_DIR        = 3;
static const uint8_t CH_SWA        = 4;
static const uint8_t NB_CANAUX_RC  = 5;

static const uint8_t CANAL_IBUS_ARBITRAGE = 6;   // CH7 (index iBUS 0 = CH1)
static const uint8_t CANAUX_IBUS[NB_CANAUX_RC] = {
    0,                      // CH1 ailerons
    1,                      // CH2 profondeur
    2,                      // CH3 gaz
    3,                      // CH4 direction
    CANAL_IBUS_ARBITRAGE,   // arbitrage manuel / autopilote
};

// ─── Sorties servo (LEDC, matériel) ──────────────────────────────────────────
// Impulsions générées par le périphérique LEDC : aucune gigue logicielle,
// contrairement à la lib Servo du Nano (Timer1 + interruptions).
// 50 Hz sur 16 bits → résolution 20 000 µs / 65 536 ≈ 0,3 µs.

static const uint32_t LEDC_FREQ_HZ    = 50;
static const uint8_t  LEDC_RESOLUTION = 16;
static const uint32_t LEDC_PERIODE_US = 1000000UL / LEDC_FREQ_HZ;

#if ESP_ARDUINO_VERSION_MAJOR < 3
// Core 2.x : l'API LEDC adresse un canal, pas une broche.
static uint8_t canal_ledc(uint8_t broche) {
    switch (broche) {
        case BROCHE_AIL: return 0;
        case BROCHE_PRO: return 1;
        case BROCHE_GAZ: return 2;
        default:         return 3;   // BROCHE_DIR
    }
}
#endif

static void ecrire_us(uint8_t broche, uint16_t us) {
    uint32_t rapport = ((uint32_t)us << LEDC_RESOLUTION) / LEDC_PERIODE_US;
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcWrite(broche, rapport);
#else
    ledcWrite(canal_ledc(broche), rapport);
#endif
}

static void attacher_sortie(uint8_t broche, uint16_t us_initial) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcAttach(broche, LEDC_FREQ_HZ, LEDC_RESOLUTION);
#else
    ledcSetup(canal_ledc(broche), LEDC_FREQ_HZ, LEDC_RESOLUTION);
    ledcAttachPin(broche, canal_ledc(broche));
#endif
    ecrire_us(broche, us_initial);
}

// ─── État ────────────────────────────────────────────────────────────────────

static uint16_t cons_ailerons   = US_NEUTRE;
static uint16_t cons_profondeur = US_NEUTRE;
static uint16_t cons_gaz        = US_MIN;
static uint16_t cons_direction  = US_NEUTRE;
static bool     arme            = false;

static uint16_t canaux_rc[NB_CANAUX_RC];

static bool     mode_manuel    = false;
static bool     rc_perdu       = false;
static bool     chien_de_garde = false;
static uint16_t gaz_applique   = US_MIN;
static bool     rc_actif       = false;   // vrai dès la première trame iBUS valide

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
    while (PORT_PI.available() > 0) {
        uint8_t b = (uint8_t)PORT_PI.read();
        if (pos_tampon == 0 && b != DEBUT_CONSIGNE) continue;
        tampon_pi[pos_tampon++] = b;
        if (pos_tampon == TAILLE_CONSIGNE) {
            traiter_trame_consigne(tampon_pi);
            pos_tampon = 0;
        }
    }
}

// ─── Lecture canaux RC (iBUS) ────────────────────────────────────────────────
// Pas de filtre : l'iBUS transporte des valeurs numériques, sans le bruit de
// mesure de la capture PWM par interruption qui motivait l'EMA sur le Nano.

static void lire_rc() {
    rcibus_traiter();
    if (rcibus_pret()) {
        for (uint8_t i = 0; i < NB_CANAUX_RC; i++)
            canaux_rc[i] = clipper_us(rcibus_lire(CANAUX_IBUS[i]));
        rc_actif = true;
    }

    rc_perdu    = rc_actif && ((millis() - rcibus_ts_derniere()) > DELAI_RC_PERDU_MS);
    mode_manuel = (canaux_rc[CH_SWA] < US_NEUTRE);  // bas = manuel, haut = autopilote
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
    ecrire_us(BROCHE_AIL, ail);
    ecrire_us(BROCHE_PRO, pro);
    ecrire_us(BROCHE_DIR, dir);
    ecrire_us(BROCHE_GAZ, gaz);
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

    PORT_PI.write(t, TAILLE_REMONTEE);
}

// ─── LED ─────────────────────────────────────────────────────────────────────

static void battre_led() {
    uint32_t periode;
    if (!rc_actif)     periode = 500UL;
    else if (rc_perdu) periode = 250UL;
    else               periode = 125UL;

    if ((millis() - ts_led) >= periode) {
        ts_led   = millis();
        led_etat = !led_etat;
        digitalWrite(BROCHE_LED, led_etat ? HIGH : LOW);
    }
}

// ─── Traces USB ──────────────────────────────────────────────────────────────

#if defined(TRACES_USB)
static uint32_t ts_trace = 0;

static void tracer() {
    if ((millis() - ts_trace) < 500UL) return;
    ts_trace = millis();
    PORT_DEBUG.printf("rc %s%s | ail %4u pro %4u gaz %4u dir %4u arb %4u | %s | pi %s%s | gaz_app %4u | ibus_rej %lu\n",
        rc_actif ? "ok" : "--", rc_perdu ? " PERDU" : "",
        canaux_rc[CH_AIL], canaux_rc[CH_PRO], canaux_rc[CH_GAZ], canaux_rc[CH_DIR], canaux_rc[CH_SWA],
        mode_manuel ? "MANUEL" : "AUTO",
        chien_de_garde ? "muet" : "ok", arme ? " ARME" : "",
        gaz_applique, (unsigned long)rcibus_nb_rejetees());
}
#endif

// ─── Setup / Loop ────────────────────────────────────────────────────────────

void setup() {
    // Sorties en premier : gaz à zéro et surfaces au neutre le plus tôt possible.
    attacher_sortie(BROCHE_AIL, US_NEUTRE);
    attacher_sortie(BROCHE_PRO, US_NEUTRE);
    attacher_sortie(BROCHE_GAZ, US_MIN);
    attacher_sortie(BROCHE_DIR, US_NEUTRE);

    pinMode(BROCHE_LED, OUTPUT);

    PORT_DEBUG.begin(115200);
    PORT_PI.begin(57600, SERIAL_8N1, BROCHE_PI_RX, BROCHE_PI_TX);
    rcibus_init(PORT_IBUS, BROCHE_IBUS_RX, BROCHE_IBUS_TX);

    for (uint8_t i = 0; i < NB_CANAUX_RC; i++) canaux_rc[i] = US_NEUTRE;
    canaux_rc[CH_GAZ] = US_MIN;
    canaux_rc[CH_SWA] = US_MIN;   // arbitrage bas au démarrage → mode manuel par défaut

    uint32_t now = millis();
    ts_trame_pi  = now;
    ts_remontee  = now;

#if defined(TRACES_USB)
    PORT_DEBUG.println("AirHaum II — contrôleur servo ESP32 démarré");
#endif
}

void loop() {
    lire_rc();
    lire_consignes_pi();
    appliquer_consignes();
    envoyer_remontee();
    battre_led();
#if defined(TRACES_USB)
    tracer();
#endif
}
