#!/bin/bash
# Pulls flight logs from a drone back to this machine for review. The
# reverse of sync.sh - logs are generated on the Pi and live in
# ~/drone-ops/companion/logs/, which is gitignored (not tracked, not synced
# by sync.sh/deploy.sh in that direction).
#
# Usage:
#   ./fetch_logs.sh <pi-host> <pi-user> [local-dest]
#
# Example:
#   ./fetch_logs.sh drone1-pi.local drone1
#   ./fetch_logs.sh drone1-pi.local drone1 ~/Desktop/flight-review
#
# Safe to re-run: only copies new/changed files (rsync, no --delete), so
# logs already fetched are left alone and nothing on the Pi is ever removed.
set -euo pipefail

PI_HOST="${1:?usage: $0 <pi-host> <pi-user> [local-dest]}"
PI_USER="${2:?usage: $0 <pi-host> <pi-user> [local-dest]}"
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
DEST="${3:-$REPO_DIR/companion/logs}"

if ! command -v rsync >/dev/null 2>&1; then
    echo "ERROR: rsync is not installed locally (sudo apt-get install -y rsync)" >&2
    exit 1
fi

mkdir -p "$DEST"

# Check the remote logs/ dir exists before invoking rsync: if nothing has
# flown since the logger was added, it won't, and rsync's failure message
# for that case ("No such file or directory") is easy to mistake for a real
# transfer error rather than the expected "nothing to fetch yet".
if ! ssh -o BatchMode=yes -o ConnectTimeout=8 "$PI_USER@$PI_HOST" \
        '[ -d drone-ops/companion/logs ]'; then
    echo "No logs directory on $PI_HOST yet - nothing has been flown since"
    echo "flight logging was added. Nothing to fetch."
    exit 0
fi

echo "Fetching logs from $PI_USER@$PI_HOST:~/drone-ops/companion/logs/ -> $DEST/"
rsync -az --itemize-changes \
    -e "ssh -o BatchMode=yes -o ConnectTimeout=8" \
    "$PI_USER@$PI_HOST:drone-ops/companion/logs/" "$DEST/"
# set -e means we only get here on success; itemize-changes already printed
# what moved (or nothing, if everything was already up to date locally)

echo
echo "Logs are in $DEST"
n=$(find "$DEST" -maxdepth 1 -name '*.csv' | wc -l)
echo "$n flight CSV log(s) total."
