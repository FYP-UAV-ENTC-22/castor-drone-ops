#!/bin/bash
# Configure a Pi drone's "FFT" NetworkManager connection to route internet
# traffic through the base station (which NATs it out its internet interface).
#
# Usage: ./pi_fft_gateway.sh <pi-host-or-ip> <ssh-user> <ssh-password>
# Example: ./pi_fft_gateway.sh drone2-pi.local drone2 drone
#
# Prereqs on the Pi: already joined + connected to the "FFT" wifi (profile
# name must be exactly "FFT" — check with `nmcli connection show` if unsure).
set -euo pipefail

PI_HOST="$1"
PI_USER="$2"
PI_PASS="$3"

GW="192.168.1.103"   # base station's IP on the FFT interface (wlo1) - fixed, don't change per-drone
CONN="FFT"            # NetworkManager connection/profile name for the FFT wifi
DNS="8.8.8.8"

SSHPASS="$PI_PASS" sshpass -e ssh -o StrictHostKeyChecking=accept-new -o ConnectTimeout=8 "$PI_USER@$PI_HOST" "
set -e
echo '$PI_PASS' | sudo -S nmcli connection modify '$CONN' ipv4.dns '$DNS' ipv4.ignore-auto-dns yes
echo '$PI_PASS' | sudo -S nmcli connection modify '$CONN' +ipv4.routes '0.0.0.0/0 $GW 100'
echo '$PI_PASS' | sudo -S nmcli device reapply wlan0
echo '--- route table ---'
ip route
echo '--- ping base station over FFT ---'
ping -c 2 -W 2 $GW
echo '--- ping 8.8.8.8 ---'
ping -c 2 -W 2 8.8.8.8
echo '--- ping google.com (DNS check) ---'
ping -c 2 -W 2 google.com
"
