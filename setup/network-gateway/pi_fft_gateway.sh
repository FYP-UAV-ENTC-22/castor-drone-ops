#!/bin/bash
# Configure a drone Pi's "FFT" NetworkManager connection to route internet
# traffic through a gateway machine running base_station_setup.sh.
#
# Usage: ./pi_fft_gateway.sh <pi-host-or-ip> <ssh-user> <ssh-password> [gateway-ip]
# Example: ./pi_fft_gateway.sh drone2-pi.local drone2 drone
#
# If gateway-ip is omitted, it's auto-detected as THIS machine's IP on the
# FFT interface - i.e. running this script (with no 4th arg) points the
# drone at you. Pass a gateway-ip explicitly to point a drone at a
# teammate's machine instead (e.g. whoever currently has internet).
#
# Safe to re-run / re-point: clears any previously-set custom route on the
# Pi's FFT profile before adding the new one, so switching which teammate a
# drone routes through doesn't leave stale routes behind.
#
# Prereq on the Pi: already joined to FFT with a saved NetworkManager
# profile literally named "FFT" (check with `nmcli connection show` on the
# Pi if unsure).
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/lib/detect_fft.sh"

PI_HOST="$1"
PI_USER="$2"
PI_PASS="$3"
GW="${4:-}"

if [ -z "$GW" ]; then
    FFT_IF=$(detect_fft_interface) || {
        echo "ERROR: this machine isn't connected to '$FFT_SSID' wifi, and no gateway-ip was given." >&2
        echo "Either connect to FFT, or pass the gateway IP as a 4th argument:" >&2
        echo "  $0 $PI_HOST $PI_USER <password> <gateway-ip>" >&2
        exit 1
    }
    GW=$(fft_interface_ip "$FFT_IF")
    echo "No gateway-ip given - auto-detected this machine's FFT address: $GW"
fi

CONN="FFT"
DNS="8.8.8.8"

echo "Pointing $PI_HOST's default route at $GW"

SSHPASS="$PI_PASS" sshpass -e ssh -o StrictHostKeyChecking=accept-new -o ConnectTimeout=8 "$PI_USER@$PI_HOST" "
set -e
echo '$PI_PASS' | sudo -S nmcli connection modify '$CONN' ipv4.dns '$DNS' ipv4.ignore-auto-dns yes
echo '$PI_PASS' | sudo -S nmcli connection modify '$CONN' ipv4.routes ''
echo '$PI_PASS' | sudo -S nmcli connection modify '$CONN' +ipv4.routes '0.0.0.0/0 $GW 100'
echo '$PI_PASS' | sudo -S nmcli device reapply wlan0
echo '--- route table ---'
ip route
echo '--- ping gateway ($GW) over FFT ---'
ping -c 2 -W 2 $GW
echo '--- ping 8.8.8.8 ---'
ping -c 2 -W 2 8.8.8.8
echo '--- ping google.com (DNS check) ---'
ping -c 2 -W 2 google.com
"
