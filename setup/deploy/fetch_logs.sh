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
echo "Fetching logs from $PI_USER@$PI_HOST:~/drone-ops/companion/logs/ -> $DEST/"
rsync -az --itemize-changes \
    -e "ssh -o BatchMode=yes -o ConnectTimeout=8" \
    "$PI_USER@$PI_HOST:drone-ops/companion/logs/" "$DEST/" \
    2>&1 | grep -v '^$' || echo "(nothing new)"

echo
echo "Logs are in $DEST"
n=$(find "$DEST" -maxdepth 1 -name '*.csv' | wc -l)
echo "$n flight CSV log(s) total."
