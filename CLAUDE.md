# drone-ops

FYP multi-drone project. Raspberry Pi 5 companion computers (drone1,
drone2, drone3 planned) connect over a long-range wifi network called
**FFT**, which has no internet of its own. A teammate's laptop acts as an
internet gateway for FFT so drones (and optionally other teammates) can
reach the internet while working. See `setup/network-gateway/README.md`
for the full design/rationale - this file is the quick-reference index.

## Drone inventory

| Name   | Host               | SSH user | Status |
|--------|--------------------|----------|--------|
| drone1 | drone1-pi.local    | drone1   | Active (network gateway, SSH key, GitHub deploy key). Flight controller: iFlight **BlitzF745**, ArduCopter **4.6.2**, quadrotor. See "Flight controller links" below - **not flight-ready**. |
| drone2 | drone2-pi.local    | drone2   | Not yet provisioned |
| drone3 | drone3-pi.local    | drone3   | Not yet provisioned |

All run Ubuntu 24.04 (`cat /etc/os-release` to confirm), NetworkManager +
netplan, on the `192.168.1.0/24` FFT subnet.

## Flight controller links (drone1) - READ BEFORE FLYING

Two physical links to the BlitzF745, both MAVLink2:

| Link | Device | Baud | State |
|------|--------|------|-------|
| GPIO UART (pins 8/10) | `/dev/ttyAMA0` | 57600 | FC->Pi reliable. **Pi->FC INTERMITTENT - unresolved hardware fault.** |
| USB | `/dev/ttyACM0` | 115200 | Bidirectional, but re-enumerated spontaneously once mid-session |

**The UART fault is the top blocker.** Measured across four runs with no
changes in between: 0 replies, then success on attempt 1, then 11/16, then
**0/13 in 141.7s**. FC heartbeats arrive throughout, so `T3` -> Pi pin 10 is
sound; the fault is confined to **Pi pin 8 -> FC `R3`**. That signature is a
marginal joint (cold solder / partially seated pin / wire broken inside its
insulation), which is the worst case for flight: it passes preflight and
drops out under vibration.

Both sides have been ruled out in software, so don't re-debug them:
- Pi pinmux verified: `pin 14`/`pin 15` claimed by `1f00030000.serial`, function `uart0`
- FC `SERIAL3` = MAVLink2 @ 57600, `OPTIONS = 0` (no inversion, no flow control)
- Board default is `SERIAL3 -> UART3 (GPS)`; this aircraft re-tasks it to
  MAVLink2 with GPS moved to `SERIAL4`. That is intentional and correct.

To verify a repair: `companion/tools/uart_arm_param_test.py --no-arm` should
report **13/13 in a few seconds**. Anything less - or 13/13 only sometimes -
means the joint is still marginal. Flex the harness while it runs.

Note: ArduPilot answers param requests far more reliably on a link it has seen
a **GCS heartbeat** on (`MAV_TYPE_GCS`). All tools here send one before any
param traffic; without it, reads fail even on a healthy link.

### Other flight blockers (unresolved)

- **GPS-only position hold at 1 m.** `EK3_SRC1_POSXY/VELXY = 3` (GPS), fix
  type 3. ~1-2.5 m accuracy that wanders, and `ctbr_acro_rc.py`'s XY loop
  chases it with 12 deg of tilt authority at 1 m altitude.
- **Spool-up vs climb ramp.** Motors measured ~1.2 s dead + ~0.5 s ramp to
  the 1100us `MOT_SPIN_ARM` idle, but `alt_sp` starts ramping at t=0, so the
  altitude PID integrates against unresponsive motors at liftoff.
- **`--force-arm` bypasses `ARMING_CHECK = 1`**, including EKF/GPS checks.
- **LAND fallback is untested** - it can only be exercised in flight.
- **`FS_GCS_ENABLE = 0`**: the FC does nothing if the companion link dies.
  After `RC_OVERRIDE_TIME = 3.0 s` control reverts to the transmitter, whose
  throttle sits at 991 (min) - so a pilot must be holding it at hover.

## SSH access

Prefer key-based login over passwords. To get your own key onto a drone:
```
setup/ssh-access/add_my_key_to_pi.sh <pi-host> <pi-user> <pi-password>
```
Safe to re-run; each teammate adds their own key independently. Note: key
auth only covers the SSH *login* - `sudo` commands run over that
connection still prompt for the account password (there's no NOPASSWD
sudo configured, deliberately not set up without asking first - it's a
real security tradeoff for hardware that flies).

## Network gateway (FFT has no internet)

Three scripts in `setup/network-gateway/`, one for each role:
- `base_station_setup.sh` - become the gateway (whoever currently has internet)
- `pi_fft_gateway.sh <pi-host> <pi-user> <pi-password> [gateway-ip|off]` - point a drone at a gateway
- `teammate_fft_client.sh on <gateway-ip> | off` - point your own laptop at a gateway (safe fallback only, never demotes your real internet)
- `stop_gateway.sh` - stop being the gateway (optional tidy-up, not required for safety - everything fails closed)

Full detail, verification commands, and reversal steps in
`setup/network-gateway/README.md`.

## Getting code onto a drone (GitHub -> Pi)

Each Pi has its own SSH deploy key for read-only access to this repo
(private org repo). One-time per Pi:
```
setup/deploy/setup_github_deploy_key.sh <pi-host> <pi-user>
```
Generates a keypair on the Pi and configures its SSH client for
github.com; prints the public key, which then has to be added manually at
`github.com/FYP-UAV-ENTC-22/drone-ops` -> Settings -> Deploy keys (no
GitHub API/CLI access available to automate that step). **Leave "Allow
write access" unchecked.**

Two ways to get code onto a drone. **Use `sync.sh` while developing**; the
GitHub round-trip is slow and needs the Pi to have working internet through
the NAT gateway, which is the least reliable part of the setup.

```
setup/deploy/sync.sh <pi-host> <pi-user>          # rsync working tree over SSH
setup/deploy/sync.sh <pi-host> <pi-user> -n       # dry run first
setup/deploy/sync.sh <pi-host> <pi-user> --reset  # discard Pi edits, match origin/main
```
Works over FFT alone, includes uncommitted edits, and preserves the Pi's
`.venv` (which is gitignored and would be painful to rebuild without
internet). Commit and push only once the code actually works.

To pull the authoritative git-tracked state instead (clones on first run,
needs internet on the Pi):
```
setup/deploy/deploy.sh <pi-host> <pi-user> [pi-password]
```
Password is optional if SSH key login is already set up (no sudo
involved in this script). Workflow: edit locally -> commit -> push ->
run `deploy.sh` against whichever drone(s) need the update.

## Companion tools (all read-only unless noted)

| Tool | Purpose |
|------|---------|
| `tools/check_fc_link.py` | Heartbeat/link check, scans common bauds. Safe with props on. |
| `tools/check_fc_params.py` | Dumps the params `ctbr_acro_rc.py` needs + board version. Safe with props on. |
| `tools/check_serial_ports.py` | Dumps every `SERIALx_PROTOCOL/BAUD/OPTIONS`. Safe with props on. |
| `tools/uart_arm_param_test.py` | Params + arm/hold/disarm. `--no-arm` is safe with props on; **arming needs props OFF**. |
| `tools/arm_disarm_test.py` | Arm, hold `--dwell`, disarm, with motor-PWM evidence. **Props OFF.** |
| `test_codes/ctbr_acro_rc.py` | The CTBR flight script. **Arms and takes off.** |

`ctbr_acro_rc.py` writes no parameters at all unless `--set-trainer` is
passed, and treats any unreadable flight-critical parameter as fatal rather
than falling back to a default - a zero-deadzone default silently maps every
small attitude correction inside the FC's real deadzone, producing no
rotation at all. `ACRO_TRAINER` must be 0 (this vehicle has 2); it is the one
setting that cannot be compensated in software, because ArduPilot derives its
levelling rate from the attitude target rather than from stick input.

Companion-computer code that runs on the drones lives in `companion/`.
Dependencies are in `companion/requirements.txt`, installed into a venv
(system-wide pip is blocked by PEP 668 on these Pis' Ubuntu 24.04):
```
cd ~/drone-ops/companion
python3 -m venv .venv          # first time only
.venv/bin/pip install -r requirements.txt
.venv/bin/python test_codes/ctbr_acro_rc.py --connect <mavlink-endpoint>
```
`.venv/` is gitignored - each Pi builds its own, `deploy.sh` doesn't
touch it, so re-run the pip install step after `requirements.txt` changes.

## Known TODOs / flagged issues

- **drone1's GitHub deploy key currently has write access, not read-only
  as intended.** Caught by testing an actual push (it succeeded and had
  to be reverted - see git history around 2026-08-20). User was informed
  and chose to leave it for now. To fix: delete the `drone1` deploy key on
  GitHub and re-add it with "Allow write access" left unchecked.
- Teammate laptops on FFT don't have static/reserved IPs yet - discussed
  using AP-side DHCP reservations (the FFT access point at `192.168.1.1`
  has a web admin UI, reachable despite being ISP-locked/no internet) but
  not yet implemented. Needed so `pi_fft_gateway.sh`/`teammate_fft_client.sh`
  gateway IPs stay stable across reconnects.
- drone2 and drone3 hardware/OS not yet set up - repeat the drone1
  workflow (network gateway pointing, SSH key, GitHub deploy key) once
  they exist.

## Pi 5 gotcha: header UART (GPIO14/15) is disabled by default

Raspberry Pi 5 exposes multiple UART peripherals via the RP1 chip; the
"primary" one for the 40-pin header pins 14/15 doesn't appear as a
`/dev` device until it's explicitly enabled. `ttyAMA10` may already
exist and look valid (it's a real, separate on-SoC UART, `status: okay`)
but is NOT connected to the header - don't waste time testing it if the
flight controller is wired to pins 14/15.

Check with:
```
cat /proc/device-tree/axi/pcie@120000/rp1/serial@30000/status   # "disabled" = needs enabling
```
Fix (needs a reboot):
```
echo "dtoverlay=uart0" | sudo tee -a /boot/firmware/config.txt
sudo reboot
```
After reboot, the flight controller should appear at `/dev/ttyAMA0`.
Verify with `companion/tools/check_fc_link.py --port /dev/ttyAMA0`
(read-only, safe with propellers attached - no arm/mode/RC-override
commands sent). Confirmed working on drone1 at 57600 baud.
