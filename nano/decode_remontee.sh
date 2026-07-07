#!/bin/bash
# decode_remontee.sh — décode les trames de remontée Nano → Pi (0xBB, 15 octets)
# Usage : ./decode_remontee.sh [port]
# Sur le Pi : ./decode_remontee.sh /dev/ttyS2

PORT=${1:-/dev/ttyS2}

echo "Remontée Nano sur $PORT  (Ctrl+C pour quitter)"
echo "MODE    RC   CG   CH1   CH2   CH3   CH4   CH5   GAZ"
echo "------------------------------------------------------"

stty -F "$PORT" 57600 raw cs8 -parenb -cstopb -echo

dd if="$PORT" bs=1 2>/dev/null | stdbuf -oL od -v -An -tu1 | awk '
BEGIN { n = 0 }
{
    for (f = 1; f <= NF; f++) {
        b[n++] = int($f)

        while (n >= 15) {
            # Chercher header 0xBB (187)
            if (b[0] != 187) {
                for (i = 0; i < n-1; i++) b[i] = b[i+1]
                n--
                continue
            }

            # Vérifier checksum XOR sur octets 1..13
            cs = 0
            for (i = 1; i < 14; i++) cs = xor(cs, b[i])

            if (cs != b[14]) {
                for (i = 0; i < n-1; i++) b[i] = b[i+1]
                n--
                continue
            }

            # Décoder fanions
            fanions = b[1]
            mode    = (and(fanions, 1) ? "MANUEL" : "AUTO  ")
            rc      = (and(fanions, 2) ? "PERDU" : "OK   ")
            cg      = (and(fanions, 4) ? "OUI" : "non")

            # Décoder canaux RC (5 canaux × 2 octets)
            for (i = 0; i < 5; i++)
                ch[i] = b[2+i*2] + b[3+i*2]*256

            # Gaz appliqué
            gaz = b[12] + b[13]*256

            printf "%s  %s  %s  %4d  %4d  %4d  %4d  %4d  %4d\n",
                mode, rc, cg, ch[0], ch[1], ch[2], ch[3], ch[4], gaz
            fflush()

            for (i = 0; i < n-15; i++) b[i] = b[i+15]
            n -= 15
        }
    }
}
'
