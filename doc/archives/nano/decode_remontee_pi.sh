#!/bin/bash
# decode_remontee_pi.sh — décode les trames de remontée Nano → Pi (0xBB, 15 octets)
# Version Python3 (Orange Pi / Raspberry Pi — pas de dépendance od/awk/mawk)
# Usage : sudo ./decode_remontee_pi.sh [port]
# Sur le Pi : sudo ./decode_remontee_pi.sh /dev/ttyS2

PORT=${1:-/dev/ttyS2}

echo "Remontée Nano sur $PORT  (Ctrl+C pour quitter)"
echo "MODE    RC   CG    AIL   PRO   GAZ   DIR   SWA  GAZ_APP"
echo "-----------------------------------------------------------"

stty -F "$PORT" 57600 raw cs8 -parenb -cstopb -echo

python3 - "$PORT" << 'PYEOF'
import sys

port = sys.argv[1]
buf  = bytearray()
TAILLE = 15  # 0xBB + fanions + 5 canaux×2 + gaz×2 + checksum

try:
    with open(port, 'rb', buffering=0) as f:
        while True:
            b = f.read(1)
            if not b:
                continue
            buf.extend(b)
            while len(buf) >= TAILLE:
                if buf[0] != 0xBB:
                    del buf[0]
                    continue
                cs = 0
                for x in buf[1:14]:
                    cs ^= x
                if cs != buf[14]:
                    del buf[0]
                    continue
                fanions = buf[1]
                mode = "MANUEL" if (fanions & 1) else "AUTO  "
                rc   = "PERDU " if (fanions & 2) else "OK    "
                cg   = "OUI" if (fanions & 4) else "non"
                ch   = [buf[2 + i*2] | (buf[3 + i*2] << 8) for i in range(5)]
                gaz  = buf[12] | (buf[13] << 8)
                # ch: AIL PRO GAZ DIR SWA
                print(f"{mode}  {rc}  {cg}  {ch[0]:4d}  {ch[1]:4d}  {ch[2]:4d}  {ch[3]:4d}  {ch[4]:4d}  {gaz:4d}", flush=True)
                del buf[:TAILLE]
except KeyboardInterrupt:
    pass
PYEOF
