#!/bin/bash
# Fast-iterate sync: copy the local working tree straight to a drone over SSH.
# No GitHub round-trip, so this works over FFT alone - the Pi does not need
# internet, which matters because the NAT gateway is the least reliable part
# of the setup.
#
# Usage:
#   ./sync.sh <pi-host> <pi-user> [-n]
#
#   ./sync.sh drone1-pi.local drone1        # copy now
#   ./sync.sh drone1-pi.local drone1 -n     # dry run: show what WOULD change
#
# Workflow this is meant for:
#   edit -> ./sync.sh -> test on the drone -> repeat
#   ... and only once it actually works:  git commit && git push
#
# Relationship to deploy.sh:
#   deploy.sh pulls from GitHub - authoritative, git-tracked, needs internet.
#   sync.sh  pushes your working tree - fast, includes uncommitted edits.
# After syncing uncommitted work the Pi's checkout is "dirty" relative to its
# git clone. That is fine and intentional; `git status` there shows exactly
# what is unpushed. To get the Pi back to a clean tracked state, commit+push
# and run deploy.sh, or use --reset below.
#
#   ./sync.sh <pi-host> <pi-user> --reset   # discard Pi-side changes, match origin/main
set -euo pipefail

PI_HOST="${1:?usage: $0 <pi-host> <pi-user> [-n|--reset]}"
PI_USER="${2:?usage: $0 <pi-host> <pi-user> [-n|--reset]}"
MODE="${3:-}"

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
# Relative to the remote user's home. Do NOT use "$HOME" or "~" here: rsync
# does not expand either after "host:", so it would create a literal
# "$HOME" directory under the remote home instead.
DEST="drone-ops/"

if [ "$MODE" = "--reset" ]; then
    echo "Resetting $PI_HOST to origin/main (discards Pi-side edits, keeps .venv) ..."
    ssh -o BatchMode=yes -o ConnectTimeout=8 "$PI_USER@$PI_HOST" '
        set -e
        cd ~/drone-ops
        git fetch origin
        git reset --hard origin/main
        git clean -fd -e .venv -e "*.venv*"
        git log --oneline -1
    '
    exit 0
fi

if ! command -v rsync >/dev/null 2>&1; then
    echo "ERROR: rsync is not installed locally (sudo apt-get install -y rsync)" >&2
    exit 1
fi
if ! ssh -o BatchMode=yes -o ConnectTimeout=8 "$PI_USER@$PI_HOST" 'command -v rsync >/dev/null'; then
    echo "ERROR: rsync is not installed on $PI_HOST (sudo apt-get install -y rsync)" >&2
    exit 1
fi

DRY=""
if [ "$MODE" = "-n" ] || [ "$MODE" = "--dry-run" ]; then
    DRY="--dry-run"
    echo "DRY RUN - nothing will be written."
fi

# --delete keeps the Pi an exact mirror, so a file deleted locally does not
# linger and get run by accident. The excludes are what stop that from being
# destructive: .venv is built on the Pi and is NOT in git, so losing it would
# mean reinstalling pymavlink over a link that may have no internet.
echo "Syncing $REPO_DIR -> $PI_USER@$PI_HOST:~/drone-ops/"
# shellcheck disable=SC2086
rsync -az --delete $DRY \
    --itemize-changes \
    --exclude '.git/' \
    --exclude '.venv/' \
    --exclude '__pycache__/' \
    --exclude '*.pyc' \
    -e "ssh -o BatchMode=yes -o ConnectTimeout=8" \
    "$REPO_DIR/" "$PI_USER@$PI_HOST:$DEST"

if [ -n "$DRY" ]; then
    echo "(dry run - re-run without -n to apply)"
    exit 0
fi

echo
echo "--- Pi-side state vs its last commit ---"
ssh -o BatchMode=yes -o ConnectTimeout=8 "$PI_USER@$PI_HOST" '
    cd ~/drone-ops
    git log --oneline -1
    if [ -n "$(git status --porcelain)" ]; then
        echo "uncommitted differences now on the Pi:"
        git status --short
    else
        echo "Pi matches its last commit exactly."
    fi
'
