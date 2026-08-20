#!/bin/bash
# Configure a drone Pi's "FFT" NetworkManager connection to route internet
# traffic through a gateway machine running base_station_setup.sh.
#
# Usage: ./pi_fft_gateway.sh <pi-host-or-ip> <ssh-user> <ssh-password> [gateway-ip|off]
# Examples:
#   ./pi_fft_gateway.sh drone2-pi.local drone2 drone            # point at me
#   ./pi_fft_gateway.sh drone2-pi.local drone2 drone 192.168.1.42  # point at a teammate
#   ./pi_fft_gateway.sh drone2-pi.local drone2 drone off         # stop routing via any gateway
#
# If gateway-ip is omitted, it's auto-detected as THIS machine's IP on the
# FFT interface - i.e. running this script (with no 4th arg) points the
# drone at you. Pass a gateway-ip explicitly to point a drone at a
# teammate's machine instead (e.g. whoever currently has internet). Pass
# "off" to remove the custom route/DNS entirely (back to FFT-only, no
# internet) - not required for safety (an unreachable gateway just fails
# closed on its own), but useful if you want the Pi to stop trying.
#
# Safe to re-run / re-point: clears any previously-set custom route on the
# Pi's FFT profile before adding the new one, so switching which teammate a
# drone routes through doesn't leave stale routes behind.
#
# Prereq on the Pi: already joined to FFT with a saved NetworkManager
# profile literally named "FFT" (check with `nmcli connection show` on the
# Pi if unsure).
#
# Uses SSH key login when available (see setup/ssh-access/add_my_key_to_pi.sh),
# falling back to password login automatically if no key is set up yet. The
# password is always needed regardless, for the sudo commands run remotely.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/lib/detect_fft.sh"

PI_HOST="$1"
PI_USER="$2"
PI_PASS="$3"
GW="${4:-}"

# Tries key-based login first (fast, no password on the wire for the SSH
# session itself); falls back to password login only if that specifically
# fails (ssh exit code 255 = connection/auth error, not a remote command
# failure) so a real failure in the remote command doesn't get silently
# retried and double-applied.
ssh_pi() {
    local remote_cmd="$1" rc=0
    ssh -o BatchMode=yes -o ConnectTimeout=5 -o StrictHostKeyChecking=accept-new "$PI_USER@$PI_HOST" "$remote_cmd" || rc=$?
    if [ "$rc" -eq 255 ]; then
        echo "(no SSH key set up for $PI_HOST yet - falling back to password login;" >&2
        echo " run setup/ssh-access/add_my_key_to_pi.sh $PI_HOST $PI_USER <password> to avoid this next time)" >&2
        SSHPASS="$PI_PASS" sshpass -e ssh -o StrictHostKeyChecking=accept-new -o ConnectTimeout=8 "$PI_USER@$PI_HOST" "$remote_cmd"
        return $?
    fi
    return $rc
}

if [ "$GW" = "off" ]; then
    echo "Removing FFT gateway route/DNS from $PI_HOST"
    ssh_pi "
set -e
echo '$PI_PASS' | sudo -S nmcli connection modify FFT ipv4.routes ''
echo '$PI_PASS' | sudo -S nmcli connection modify FFT ipv4.dns '' ipv4.ignore-auto-dns no
echo '$PI_PASS' | sudo -S nmcli device reapply wlan0
sleep 1
echo '--- route table ---'
ip route
"
    exit 0
fi

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

ssh_pi "
set -e
echo '$PI_PASS' | sudo -S nmcli connection modify '$CONN' ipv4.dns '$DNS' ipv4.ignore-auto-dns yes
echo '$PI_PASS' | sudo -S nmcli connection modify '$CONN' ipv4.routes ''
echo '$PI_PASS' | sudo -S nmcli connection modify '$CONN' +ipv4.routes '0.0.0.0/0 $GW 100'
echo '$PI_PASS' | sudo -S nmcli device reapply wlan0
sleep 1
echo '--- route table ---'
ip route
echo '--- ping gateway ($GW) over FFT ---'
ping -c 2 -W 2 $GW
echo '--- ping 8.8.8.8 ---'
ping -c 2 -W 2 8.8.8.8
echo '--- ping google.com (DNS check) ---'
ping -c 2 -W 2 google.com
"
