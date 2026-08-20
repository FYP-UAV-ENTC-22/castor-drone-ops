#!/bin/bash
# Sets up SSH key-based login to a drone Pi for whoever runs this script, so
# future SSH/deploy commands don't need a login password. The Pi account's
# password is still needed separately for sudo commands run over SSH (key
# auth only covers the SSH login step, not sudo).
#
# Safe to re-run, and every teammate runs this independently against
# whichever drone(s) they work with - each person's own key gets added
# alongside everyone else's (existing authorized_keys entries are kept,
# ssh-copy-id only appends).
#
# Usage: ./add_my_key_to_pi.sh <pi-host> <pi-user> <pi-password>
# Example: ./add_my_key_to_pi.sh drone1-pi.local drone1 drone
set -euo pipefail

PI_HOST="$1"
PI_USER="$2"
PI_PASS="$3"

KEY_PATH="$HOME/.ssh/id_ed25519"
if [ ! -f "$KEY_PATH" ]; then
    echo "No SSH key found at $KEY_PATH - generating one..."
    ssh-keygen -t ed25519 -f "$KEY_PATH" -N "" -C "$(whoami)@$(hostname)"
else
    echo "Using existing key: $KEY_PATH"
fi

echo "Copying ${KEY_PATH}.pub to $PI_USER@$PI_HOST..."
SSHPASS="$PI_PASS" sshpass -e ssh-copy-id -o StrictHostKeyChecking=accept-new -i "${KEY_PATH}.pub" "$PI_USER@$PI_HOST"

echo
echo "Verifying passwordless login..."
if ssh -o BatchMode=yes -o ConnectTimeout=5 "$PI_USER@$PI_HOST" 'echo OK' | grep -q OK; then
    echo "Success - you can now SSH into $PI_HOST without a password:"
    echo "  ssh $PI_USER@$PI_HOST"
    echo "(sudo commands over that connection will still prompt for the account password)"
else
    echo "Something went wrong - key-based login isn't working yet." >&2
    exit 1
fi
