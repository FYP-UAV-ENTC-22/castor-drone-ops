# Field Test Plan — drone1

Full command reference for testing all three flight scripts in the field.
Nothing here runs anything by itself — copy/paste each command when ready.

## 0. Before you leave (from the base station)

```
cd /home/pankaja/Projects/FYP/drone-ops
./setup/deploy/sync.sh drone1-pi.local drone1
```
Confirms drone1 has the latest code. Re-run if you've made any changes since.

## 1. On the Pi — preflight + arm test (shared across all three scripts)

SSH in, then either run `field_test.sh` (chains both steps below with
confirmation gates) or the two tools separately:

```
ssh drone1@drone1-pi.local
~/drone-ops/companion/tools/field_test.sh
```
— or manually:
```
P=~/drone-ops/companion/.venv/bin/python3
T=~/drone-ops/companion/tools

echo drone | sudo -S $P $T/preflight.py                    # props on or off, checks GPS/EKF/override/fence
echo drone | sudo -S $P $T/uart_arm_param_test.py           # PROPS OFF — arm/hold/disarm with motor PWM evidence
```
Want **GO** from preflight and a clean arm/disarm before fitting propellers.

## 2. Fit propellers

Only after step 1 passes. Transmitter on, `ch7` (override enable) **LOW**.

## 3. Test flights — recommended order

Fly these in order, not back-to-back blind — land, check the console output
looked sane, and pull the log before moving to the next. Order is
deliberate: most flight-proven architecture first, newest/least-flown last.

### 3a. `ctbr_acro_rc.py` — thrust-based, 2 prior flights (one hit the now-fixed ramp bug)

```
sudo ~/drone-ops/companion/.venv/bin/python3 \
    ~/drone-ops/companion/test_codes/ctbr_acro_rc.py \
    --set-trainer --force-arm --alt 3 --hold 25
```
`--hold 25` = ~10s climb (`--climb-rate` default `0.3`) + ~15s actual hold.

### 3b. `accbr_acro_rc.py` — acceleration-based, ramp fix + position filter, zero real flights

```
sudo ~/drone-ops/companion/.venv/bin/python3 \
    ~/drone-ops/companion/test_codes/accbr_acro_rc.py \
    --connect /dev/ttyAMA0 --baud 57600 \
    --set-trainer --force-arm --alt 3 --hold 25
```
`--connect`/`--baud` needed here — this script's *default* is still USB;
passing UART explicitly matches what was actually verified as flight-worthy.
Same `--hold 25` split as above (`--climb-rate` default is also `0.3`).

### 3c. `guided_acc_hold.py` — GUIDED mode, ArduPilot's own controller, never flown

```
sudo ~/drone-ops/companion/.venv/bin/python3 \
    ~/drone-ops/companion/test_codes/guided_acc_hold.py \
    --alt 3 --hold 20 --force-arm
```
No `--connect` override needed — UART is already this script's default.
`--hold 20` here is **post-altitude hold only** (native `MAV_CMD_NAV_TAKEOFF`
climb isn't counted), so total airtime ≈ climb + 20s.

## 4. After each flight

```
# on the Pi, quick sanity glance
tail -20 ~/drone-ops/companion/logs/*_events.log
```
```
# back on the base station, pull everything
cd /home/pankaja/Projects/FYP/drone-ops
./setup/deploy/fetch_logs.sh drone1-pi.local drone1
```

## Abort, any time

`ch7` LOW = instant manual control (throttle sits near minimum — raise it
toward hover before cutting override). `Ctrl-C` on the script = commanded
LAND. Pilot should be on the transmitter, thumb on `ch7`, for all three
tests.
