// RcIbus.cpp — Décodage iBUS sur UART matériel (ESP32)
//
// Contrairement à la version Nano (SoftwareSerial, abandonnée), la réception
// est entièrement gérée par l'UART matériel et son tampon : aucune
// interruption logicielle, aucune interaction avec la génération des
// impulsions servo (LEDC, matériel lui aussi).

#include "RcIbus.h"

static const uint8_t  IBUS_TAILLE_TRAME = 32;
static const uint8_t  IBUS_LONGUEUR     = 0x20;
static const uint8_t  IBUS_COMMANDE     = 0x40;
static const uint32_t IBUS_BAUDS        = 115200;

static HardwareSerial *port_ibus = nullptr;

static uint8_t  tampon[IBUS_TAILLE_TRAME];
static uint8_t  pos = 0;

static uint16_t canaux[RCIBUS_NB_CANAUX];
static bool     pret          = false;
static uint32_t ts_derniere   = 0;
static uint32_t nb_rejetees   = 0;

void rcibus_init(HardwareSerial &port, int8_t broche_rx, int8_t broche_tx)
{
    port_ibus = &port;
    port_ibus->begin(IBUS_BAUDS, SERIAL_8N1, broche_rx, broche_tx);
    for (uint8_t i = 0; i < RCIBUS_NB_CANAUX; i++) canaux[i] = 0;
    pos = 0;
}

static void traiter_trame()
{
    uint16_t somme = 0xFFFF;
    for (uint8_t i = 0; i < IBUS_TAILLE_TRAME - 2; i++) somme -= tampon[i];
    uint16_t recue = (uint16_t)tampon[30] | ((uint16_t)tampon[31] << 8);
    if (somme != recue) { nb_rejetees++; return; }

    for (uint8_t i = 0; i < RCIBUS_NB_CANAUX; i++) {
        // Les 4 bits de poids fort servent aux canaux étendus (> 14) sur
        // certains récepteurs : seule la valeur sur 12 bits est retenue.
        canaux[i] = ((uint16_t)tampon[2 + i * 2] | ((uint16_t)tampon[3 + i * 2] << 8)) & 0x0FFF;
    }
    pret        = true;
    ts_derniere = millis();
}

void rcibus_traiter()
{
    if (port_ibus == nullptr) return;
    while (port_ibus->available() > 0) {
        uint8_t b = (uint8_t)port_ibus->read();
        // Resynchronisation sur l'en-tête 0x20 0x40
        if (pos == 0 && b != IBUS_LONGUEUR) continue;
        if (pos == 1 && b != IBUS_COMMANDE) { pos = (b == IBUS_LONGUEUR) ? 1 : 0; continue; }
        tampon[pos++] = b;
        if (pos == IBUS_TAILLE_TRAME) {
            traiter_trame();
            pos = 0;
        }
    }
}

uint16_t rcibus_lire(uint8_t canal)
{
    return (canal < RCIBUS_NB_CANAUX) ? canaux[canal] : 0;
}

bool rcibus_pret()            { return pret; }
uint32_t rcibus_ts_derniere() { return ts_derniere; }
uint32_t rcibus_nb_rejetees() { return nb_rejetees; }
