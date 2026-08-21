#!/bin/bash
# Full field-test sequence for ctbr_acro_rc.py: preflight -> arm test -> fly.
#
# Run this DIRECTLY ON THE PI, not via SSH from elsewhere - every gate below
# needs a human who can actually see the propellers to type "YES". This
# script cannot verify propeller state itself and does not try to.
#
# It does NOT run the flight automatically. The flight command is printed
# at the end for you to run deliberately - arming and taking off is the one
# step that should never happen as a side effect of running a checklist.
#
# Usage: ./field_test.sh
set -euo pipefail

P="$HOME/drone-ops/companion/.venv/bin/python3"
T="$HOME/drone-ops/companion/tools"
FLIGHT="$HOME/drone-ops/companion/test_codes/ctbr_acro_rc.py"

confirm() {
    local reply
    read -r -p "$1 (type YES to continue, anything else aborts): " reply
    if [ "$reply" != "YES" ]; then
        echo "Aborted - nothing further will run."
        exit 1
    fi
}

section() {
    echo
    echo "==============================================================="
    echo " $1"
    echo "==============================================================="
}

section "STEP 1/4: Preflight check (propellers may be on or off for this)"
sudo "$P" "$T/preflight.py"
echo
confirm "Did preflight print GO (not NO-GO)?"

section "STEP 2/4: Arm test - THIS ARMS THE VEHICLE, MOTORS WILL SPIN TO IDLE"
confirm "Are propellers physically removed from ALL motors, right now?"
sudo "$P" "$T/uart_arm_param_test.py"
echo
confirm "Did that complete with 'disarm: OK - vehicle is disarmed' and sane motor PWM?"

section "STEP 3/4: Fit propellers"
echo "Fit propellers now. Once fitted:"
confirm "Propellers fitted and secure, transmitter ON, override switch (ch7) LOW?"

section "STEP 4/4: Ready to fly"
echo "1. Flip ch7 HIGH to enable override."
echo "2. Pilot holds the transmitter, thumb on ch7, throttle ready to raise to hover"
echo "   if control is handed back (stick currently sits near minimum)."
echo "3. When ready, run this yourself - it is deliberately not run for you:"
echo
echo "   sudo $P $FLIGHT --set-trainer --force-arm --alt 1 --hold 15"
echo
echo "   (start with --alt 1 --hold 15, not a larger value, until this exact"
echo "   build has flown cleanly at least once - see CLAUDE.md for why)"
echo
echo "To abort at any point during flight: flip ch7 LOW (instant), or Ctrl-C"
echo "the script (triggers LAND)."
echo
echo "Logs will be written to ~/drone-ops/companion/logs/ - pull them back with"
echo "setup/deploy/fetch_logs.sh from the base station after landing."
