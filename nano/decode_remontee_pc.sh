#!/bin/bash
# decode_remontee_pc.sh — décode les trames de remontée Nano → Pi (0xBB, 15 octets)
# Version gawk (PC Linux) — voir decode_remontee_pi.sh pour Orange Pi (Python3)
# Usage : ./decode_remontee_pc.sh [port]

PORT=${1:-/dev/ttyUSB0}

echo "Remontée Nano sur $PORT  (Ctrl+C pour quitter)"
echo "MODE    RC   CG    AIL   PRO   GAZ   DIR   SWA  GAZ_APP"
echo "-----------------------------------------------------------"

stty -F "$PORT" 57600 raw cs8 -parenb -cstopb -echo

dd if="$PORT" bs=1 2>/dev/null | stdbuf -oL od -v -An -tu1 | gawk '
BEGIN { n = 0; TAILLE = 15 }
{
    for (f = 1; f <= NF; f++) {
        b[n++] = int($f)

        while (n >= TAILLE) {
            if (b[0] != 187) {
                for (i = 0; i < n-1; i++) b[i] = b[i+1]
                n--
                continue
            }

            cs = 0
            for (i = 1; i < 14; i++) cs = xor(cs, b[i])

            if (cs != b[14]) {
                for (i = 0; i < n-1; i++) b[i] = b[i+1]
                n--
                continue
            }

            fanions = b[1]
            mode = (and(fanions, 1) ? "MANUEL" : "AUTO  ")
            rc   = (and(fanions, 2) ? "PERDU " : "OK    ")
            cg   = (and(fanions, 4) ? "OUI" : "non")

            # 5 canaux : AIL PRO GAZ DIR SWA
            for (i = 0; i < 5; i++)
                ch[i] = b[2+i*2] + b[3+i*2]*256

            gaz = b[12] + b[13]*256

            printf "%s  %s  %s  %4d  %4d  %4d  %4d  %4d  %4d\n",
                mode, rc, cg, ch[0], ch[1], ch[2], ch[3], ch[4], gaz
            fflush()

            for (i = 0; i < n-TAILLE; i++) b[i] = b[i+TAILLE]
            n -= TAILLE
        }
    }
}
'
