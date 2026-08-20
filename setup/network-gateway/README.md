# FFT Drone Internet Gateway

Lets Raspberry Pi 5 drone companion computers on the long-range **FFT** wifi
network (which has no internet, no working default gateway) reach the
internet through a teammate's laptop acting as a NAT gateway on its second
(internet-connected) interface — without breaking that laptop's own normal
internet use.

**Portable by design**: everyone on the team is often connected to FFT at
once, but usually only one person's laptop has actual internet at any given
moment (dongle, hotspot, ethernet — whatever). Both scripts auto-detect
interfaces at run time (by SSID / by testing real connectivity, not by
assumed device names or "has a default route"), so:
- `base_station_setup.sh` is safe for **any teammate** to run on **any
  laptop**. If that laptop currently has no working internet, it detects
  that and aborts cleanly — it does not guess or half-configure anything.
- `pi_fft_gateway.sh` points a drone at *whoever runs it* by default (your
  own FFT IP), or at an explicit gateway IP if you pass one.

## Topology

```
Internet ── <auto-detected WAN iface> ── [ TEAMMATE'S LAPTOP ] ── <auto-detected FFT iface> ── drone Pis
                                          (whoever currently has internet)     192.168.1.0/24
```

- **Gateway machine**: whichever teammate's laptop currently has internet.
  Runs `base_station_setup.sh`, which finds the FFT interface (by SSID) and
  the internet interface (by testing connectivity), then NATs between them.
- **Drones**: Raspberry Pi 5s running Ubuntu 24.04, on FFT at
  `192.168.1.17x`, reached over SSH at `<hostname>.local`.
  - drone1: `drone1-pi.local`, user `drone1`
  - drone2/drone3: same pattern, added later

FFT's own DHCP advertises a dead-end gateway (`192.168.1.1`, unreachable) on
every machine — it's already at a low route priority everywhere, so it
doesn't need to be removed, just out-prioritized.

## Usage

### 1. Become the gateway (whoever currently has internet)

```
sudo ./base_station_setup.sh
```

Auto-detects your FFT interface and your internet interface, sets up
IP forwarding + NAT/FORWARD rules (persisted across reboot), and prints your
FFT-side IP at the end. If you don't have internet right now, it says so and
exits without touching anything — just have someone else run it instead.

Safe to re-run (e.g. after switching from ethernet to a phone hotspot): it
removes its own previously-added rules before adding fresh ones, so nothing
stale piles up.

### 2. Point a drone at the gateway

```
./pi_fft_gateway.sh <pi-host> <pi-user> <pi-password> [gateway-ip]
```

Examples:
```
# Point drone1 at ME (auto-detects my own FFT IP - I just ran base_station_setup.sh)
./pi_fft_gateway.sh drone1-pi.local drone1 drone

# Point drone2 at a specific teammate's gateway IP instead
./pi_fft_gateway.sh drone2-pi.local drone2 drone 192.168.1.42
```

Sets DNS (`8.8.8.8`) and a default route via the gateway IP on the drone's
`FFT` NetworkManager profile, applied live (`nmcli device reapply`, no wifi
drop) and persisted via netplan (survives reboot — confirmed by an actual
reboot test on drone1).

Re-running is safe / repointing is clean: it clears any previously-set
custom route on the Pi before adding the new one, so switching which
teammate a drone routes through never leaves stale routes behind.

Prereq on the Pi: already joined to FFT with a saved NetworkManager profile
literally named `FFT` (check with `nmcli connection show` on the Pi if
unsure).

## What each script changes

### `base_station_setup.sh` (on the gateway machine)
- `net.ipv4.ip_forward=1`, persisted in `/etc/sysctl.conf`
- iptables (tagged with comment `drone-ops-fft-gateway` for clean re-runs):
  - `POSTROUTING -o <WAN_IF> -j MASQUERADE`
  - `FORWARD -i <FFT_IF> -o <WAN_IF> -j ACCEPT`
  - `FORWARD -i <WAN_IF> -o <FFT_IF> -m state --state ESTABLISHED,RELATED -j ACCEPT`
- Persisted via `iptables-persistent` / `netfilter-persistent` →
  `/etc/iptables/rules.v4`
- Note: installing `iptables-persistent` on a machine that has `ufw`
  installed but *inactive* will remove the `ufw` package (apt dependency
  conflict). No functional change if ufw wasn't enabled; reinstall with
  `sudo apt-get install ufw` if you want it back. If `ufw` is *active* when
  you run this script, it adjusts `DEFAULT_FORWARD_POLICY` and adds a route
  rule instead of colliding with it.

### `pi_fft_gateway.sh` (on the drone Pi, via SSH)
On the Pi's `FFT` NetworkManager connection profile:
- `ipv4.dns=8.8.8.8`, `ipv4.ignore-auto-dns=yes` (FFT provides no DNS)
- `ipv4.routes` cleared, then set to `0.0.0.0/0 <gateway-ip> 100` (metric
  100 beats the dead-end DHCP default's higher metric)

## Verifying

From a drone Pi:
```
ip route                    # default via <gateway-ip> dev wlan0 metric 100
resolvectl dns wlan0        # 8.8.8.8
ping -c 2 <gateway-ip>      # gateway over FFT
ping -c 2 8.8.8.8           # raw internet
ping -c 2 google.com        # DNS + internet
```

From the gateway machine:
```
sudo iptables -L FORWARD -n -v   # packet counters on the tagged rules should increase
ip route get 8.8.8.8             # should go out your WAN interface, not FFT
```

## Reversing / disabling

**Stop one drone from routing through a gateway** (run on that Pi):
```
sudo nmcli connection modify FFT ipv4.routes ''
sudo nmcli connection modify FFT ipv4.dns "" ipv4.ignore-auto-dns no
sudo nmcli device reapply wlan0
```
Falls back to FFT-only (no internet), as before.

**Disable a gateway machine's forwarding entirely:**
```
sudo iptables -t nat -S POSTROUTING | grep drone-ops-fft-gateway
sudo iptables -S FORWARD | grep drone-ops-fft-gateway
# for each line printed above, replace the leading -A with -D and run it, e.g.:
sudo iptables -t nat -D POSTROUTING -o <WAN_IF> -m comment --comment drone-ops-fft-gateway -j MASQUERADE
sudo netfilter-persistent save
```
Optionally also set `net.ipv4.ip_forward=0` in `/etc/sysctl.conf` and
`sudo sysctl -p`, though leaving forwarding on is harmless without the
NAT/FORWARD rules — nothing will actually route.

**Just unplugging the internet source:** no action needed. The rules are
interface-name-based; with the WAN interface gone they simply have nothing
to match. Drones lose internet automatically, the gateway machine's own
traffic is unaffected, and reconnecting resumes everything with no
re-running of anything.

## Files

- `lib/detect_fft.sh` — shared detection functions (source, don't execute)
- `base_station_setup.sh` — run with `sudo` on whichever laptop currently
  has internet
- `pi_fft_gateway.sh` — run from that laptop against a drone Pi:
  `./pi_fft_gateway.sh <pi-host> <ssh-user> <ssh-password> [gateway-ip]`
