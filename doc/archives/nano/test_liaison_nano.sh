#!/bin/bash
# test_liaison_nano.sh — dump brut des octets reçus sur la liaison Nano, sans
# décodage ni vérification de checksum. Sert à vérifier que quelque chose
# arrive physiquement sur le port avant de chercher un bug de framing/protocole.
# Usage : ./test_liaison_nano.sh [port] [bauds]
# Sur le Pi : ./test_liaison_nano.sh /dev/ttyS2 57600

PORT=${1:-/dev/ttyS2}
BAUDS=${2:-57600}

if [ ! -e "$PORT" ]; then
    echo "Port $PORT introuvable. Ports série disponibles :"
    ls -1 /dev/tty{S,USB,ACM}* 2>/dev/null
    exit 1
fi

echo "Écoute brute sur $PORT à $BAUDS bauds (Ctrl+C pour quitter)"
echo "Attendu si le Nano tourne : un octet 0xBB (bb) environ tous les 15 octets, à ~10 Hz."
echo "-----------------------------------------------------------------------------"

stty -F "$PORT" "$BAUDS" raw cs8 -parenb -cstopb -echo

timeout 10 cat "$PORT" | od -A x -t x1z -v

echo "-----------------------------------------------------------------------------"
echo "Rien affiché ci-dessus après 10s = aucun octet reçu. À vérifier :"
echo "  - câblage TX Nano (D1) -> RX Pi, et alimentation du Nano"
echo "  - adaptateur de niveau 3,3V/5V bien câblé dans le bon sens"
echo "  - le Nano exécute bien nano.ino (LED D13 doit clignoter)"
echo "  - bon port ($PORT) et bons droits (groupe dialout, ou sudo)"
