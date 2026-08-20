#!/bin/bash
# Pulls the latest drone-ops code onto a drone Pi (clones on first run).
#
# Usage: ./deploy.sh <pi-host> <pi-user> [pi-password]
# Example: ./deploy.sh drone1-pi.local drone1
#
# Uses SSH key login if set up (see setup/ssh-access/add_my_key_to_pi.sh),
# falling back to password login if a password is given and no key works.
# No sudo involved - git operations run as the Pi's own user - so if key
# login is already set up, pi-password can be omitted entirely.
#
# Prereq: the Pi needs its own GitHub deploy key configured - see
# setup/deploy/setup_github_deploy_key.sh (one-time, per Pi).
set -euo pipefail

PI_HOST="$1"
PI_USER="$2"
PI_PASS="${3:-}"

REPO_URL="git@github.com:FYP-UAV-ENTC-22/drone-ops.git"
REPO_DIR="drone-ops"

ssh_pi() {
    local remote_cmd="$1" rc=0
    ssh -o BatchMode=yes -o ConnectTimeout=5 -o StrictHostKeyChecking=accept-new "$PI_USER@$PI_HOST" "$remote_cmd" || rc=$?
    if [ "$rc" -eq 255 ]; then
        if [ -z "$PI_PASS" ]; then
            echo "ERROR: key login failed and no password given. Either run" >&2
            echo "setup/ssh-access/add_my_key_to_pi.sh first, or pass a password as the 3rd argument." >&2
            return 255
        fi
        echo "(no SSH key set up for $PI_HOST - falling back to password login)" >&2
        SSHPASS="$PI_PASS" sshpass -e ssh -o StrictHostKeyChecking=accept-new -o ConnectTimeout=8 "$PI_USER@$PI_HOST" "$remote_cmd"
        return $?
    fi
    return $rc
}

echo "Deploying drone-ops to $PI_HOST..."
ssh_pi "
set -e
if [ -d ~/$REPO_DIR/.git ]; then
    cd ~/$REPO_DIR
    echo '--- pulling latest (fast-forward only) ---'
    git pull --ff-only
else
    echo '--- cloning (first time) ---'
    git clone $REPO_URL ~/$REPO_DIR
    cd ~/$REPO_DIR
fi
echo '--- current commit ---'
git log --oneline -1
"
