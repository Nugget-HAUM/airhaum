#!/bin/bash
# bascule_wifi.sh — bascule le Wifi de l'Orange Pi vers un réseau connu ou nouveau.
#
# Usage :
#   ./bascule_wifi.sh                       Scanne les réseaux à portée et propose un choix
#   ./bascule_wifi.sh <SSID>                Bascule vers un réseau déjà enregistré
#   ./bascule_wifi.sh <SSID> <mot_de_passe> Ajoute et connecte un nouveau réseau
#
# À exécuter directement sur le Pi (ou via ssh airhaum@orangepizero.local).

set -e

SSID="$1"
MDP="$2"

# Se connecte à $1 (SSID), avec mot de passe optionnel $2.
# Réutilise la connexion déjà enregistrée si elle existe (pas de mot de passe à ressaisir).
connecter() {
    local ssid="$1"
    local mdp="$2"

    if nmcli -t -f NAME connection show | grep -Fxq "$ssid"; then
        echo "Réseau déjà connu, bascule vers : $ssid"
        nmcli connection up "$ssid"
    elif [ -n "$mdp" ]; then
        echo "Nouveau réseau, connexion à : $ssid"
        nmcli device wifi connect "$ssid" password "$mdp"
    else
        echo "Nouveau réseau (ouvert), connexion à : $ssid"
        nmcli device wifi connect "$ssid"
    fi
}

if [ -n "$SSID" ]; then
    connecter "$SSID" "$MDP"
    echo
    nmcli device status
    exit 0
fi

echo "Réseau actuel :"
nmcli -t -f NAME,DEVICE,ACTIVE connection show --active | grep wlan0 || echo "  (aucun)"
echo

echo "Recherche des réseaux Wifi à portée..."
# Le rescan actif nécessite une autorisation polkit refusée à un utilisateur
# non-root en session SSH ("not authorized") ; sudo l'obtient toujours (peut
# demander le mot de passe). Sans rescan réussi, nmcli ne retourne que le
# cache déjà connu par NetworkManager (souvent le seul réseau associé).
erreur_rescan=$(sudo nmcli device wifi rescan 2>&1 >/dev/null) || {
    echo "  Avertissement : rescan impossible ($erreur_rescan)."
    echo "  Liste potentiellement incomplète (cache NetworkManager)."
}
sleep 2

# Dédoublonne par SSID (plusieurs bornes du même réseau) et ignore les lignes vides.
mapfile -t reseaux < <(nmcli -t -f SSID,SIGNAL,SECURITY device wifi list | awk -F: '$1!="" && !seen[$1]++')

if [ ${#reseaux[@]} -eq 0 ]; then
    echo "Aucun réseau détecté."
    exit 1
fi

connus=$(nmcli -t -f NAME connection show)

echo
echo "Réseaux disponibles :"
for i in "${!reseaux[@]}"; do
    IFS=: read -r nom signal securite <<< "${reseaux[$i]}"
    marque=""
    grep -Fxq "$nom" <<< "$connus" && marque=" (connu)"
    printf "  %2d) %-25s signal=%-4s %s%s\n" "$((i+1))" "$nom" "$signal" "${securite:-ouvert}" "$marque"
done
echo

read -rp "Choix (numéro, ou Ctrl+C pour annuler) : " choix
index=$((choix - 1))
if [ "$index" -lt 0 ] || [ "$index" -ge "${#reseaux[@]}" ]; then
    echo "Choix invalide."
    exit 1
fi

IFS=: read -r nom _ securite <<< "${reseaux[$index]}"

if grep -Fxq "$nom" <<< "$connus" || [ -z "$securite" ]; then
    connecter "$nom"
else
    read -rsp "Mot de passe pour $nom : " MDP
    echo
    connecter "$nom" "$MDP"
fi

echo
nmcli device status
