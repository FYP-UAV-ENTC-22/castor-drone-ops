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
| drone1 | drone1-pi.local    | drone1   | Active, fully set up (network gateway, SSH key, GitHub deploy key) |
| drone2 | drone2-pi.local    | drone2   | Not yet provisioned |
| drone3 | drone3-pi.local    | drone3   | Not yet provisioned |

All run Ubuntu 24.04 (`cat /etc/os-release` to confirm), NetworkManager +
netplan, on the `192.168.1.0/24` FFT subnet.

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

To actually pull the latest code onto a Pi (clones on first run):
```
setup/deploy/deploy.sh <pi-host> <pi-user> [pi-password]
```
Password is optional if SSH key login is already set up (no sudo
involved in this script). Workflow: edit locally -> commit -> push ->
run `deploy.sh` against whichever drone(s) need the update.

Companion-computer code that runs on the drones lives in `companion/` -
currently an empty placeholder, not yet built out.

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
