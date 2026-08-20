#!/bin/bash
# One-time setup: generates a dedicated SSH keypair on a drone Pi for
# read-only access to the drone-ops GitHub repo, and configures the Pi's
# SSH client to use it for github.com. Safe to re-run - reuses the key if
# one already exists.
#
# Usage: ./setup_github_deploy_key.sh <pi-host> <pi-user>
# Example: ./setup_github_deploy_key.sh drone2-pi.local drone2
#
# Requires SSH key login to the Pi already set up (see
# setup/ssh-access/add_my_key_to_pi.sh) - this script doesn't touch sudo,
# so no password handling needed here.
#
# After running, you MUST manually add the printed public key to:
#   github.com/FYP-UAV-ENTC-22/drone-ops -> Settings -> Deploy keys -> Add deploy key
# Leave "Allow write access" UNCHECKED - this key should be read-only.
# (No GitHub API/CLI access from here to do that step automatically. Also
# note: when this was done for drone1, the checkbox got left checked by
# mistake and had to be caught/fixed after the fact by testing an actual
# push - verify with a similar test if this matters for your case.)
set -euo pipefail

PI_HOST="$1"
PI_USER="$2"
KEY_NAME="drone_ops_deploy_key"

ssh -o BatchMode=yes -o ConnectTimeout=8 -o StrictHostKeyChecking=accept-new \
    "$PI_USER@$PI_HOST" "KEY_NAME='$KEY_NAME' PI_USER='$PI_USER' bash -s" <<'REMOTE'
set -e
if [ -f ~/.ssh/"$KEY_NAME" ]; then
    echo "Deploy key already exists on this Pi."
else
    echo "Generating deploy key..."
    ssh-keygen -t ed25519 -f ~/.ssh/"$KEY_NAME" -N "" -C "$PI_USER-deploy-key" >/dev/null
fi

if grep -q "^Host github.com$" ~/.ssh/config 2>/dev/null; then
    echo "github.com already configured in ~/.ssh/config."
else
    mkdir -p ~/.ssh
    {
        echo ""
        echo "Host github.com"
        echo "    HostName github.com"
        echo "    User git"
        echo "    IdentityFile ~/.ssh/$KEY_NAME"
        echo "    IdentitiesOnly yes"
    } >> ~/.ssh/config
    chmod 600 ~/.ssh/config
    echo "Added github.com entry to ~/.ssh/config."
fi

echo
echo "=== Public key (add this as a READ-ONLY deploy key on GitHub) ==="
cat ~/.ssh/"$KEY_NAME".pub
REMOTE

echo
echo "Next: add the public key above at"
echo "  github.com/FYP-UAV-ENTC-22/drone-ops -> Settings -> Deploy keys -> Add deploy key"
echo "Title it '$PI_USER', and leave \"Allow write access\" UNCHECKED."
echo "Then verify read-only worked with: ssh $PI_USER@$PI_HOST 'ssh -T git@github.com'"
