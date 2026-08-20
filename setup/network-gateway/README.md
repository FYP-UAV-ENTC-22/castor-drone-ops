# FFT Drone Internet Gateway

Lets Raspberry Pi 5 drone companion computers on the long-range **FFT** wifi
network (which has no internet, no working default gateway) reach the
internet through this Ubuntu base station's second (internet-connected)
interface, via NAT/IP forwarding — without breaking the base station's own
normal internet use.

Set up: 2026-08-20.

## Topology

```
Internet ── enx8a882d85e643 (USB tether/dongle) ── [ BASE STATION ] ── wlo1 (FFT wifi) ── drone Pis
            10.41.105.0/24, gw 10.41.105.43        192.168.1.103        192.168.1.0/24
```

- **Base station** (this machine, Ubuntu 24.04): bridges the two networks.
  - `wlo1` → FFT wifi, static-ish IP `192.168.1.103/24` (drones use this as their gateway)
  - `enx8a882d85e643` → USB tether/dongle with real internet access
- **Drones**: Raspberry Pi 5s running Ubuntu 24.04, on FFT at `192.168.1.17x`,
  reached over SSH at `<hostname>.local`.
  - drone1: `drone1-pi.local` / `192.168.1.177`, user `drone1`
  - drone2/drone3: same pattern, added later

FFT's own DHCP advertises a dead-end gateway (`192.168.1.1`, unreachable) —
both sides already had that at a low route priority, so it doesn't need to be
removed, just out-prioritized.

## What was changed

### Base station (`base_station_setup.sh`)
- `net.ipv4.ip_forward=1`, persisted in `/etc/sysctl.conf`
- iptables:
  - `POSTROUTING -o enx8a882d85e643 -j MASQUERADE`
  - `FORWARD -i wlo1 -o enx8a882d85e643 -j ACCEPT`
  - `FORWARD -i enx8a882d85e643 -o wlo1 -m state --state ESTABLISHED,RELATED -j ACCEPT`
- Rules persisted via `iptables-persistent` / `netfilter-persistent` →
  `/etc/iptables/rules.v4` (survives reboot)
- Side effect: installing `iptables-persistent` removed the `ufw` package
  (it wasn't enabled, so no functional change at the time). Reinstall with
  `sudo apt-get install ufw` if you want it back.

### Each drone Pi (`pi_fft_gateway.sh`)
On the Pi's `FFT` NetworkManager connection profile:
- `ipv4.dns=8.8.8.8`, `ipv4.ignore-auto-dns=yes` (FFT provides no DNS)
- `+ipv4.routes "0.0.0.0/0 192.168.1.103 100"` (default route via the base
  station, metric 100 — beats the dead-end DHCP default's higher metric)
- Applied live with `nmcli device reapply wlan0` (no wifi drop, SSH survives)

This machine uses NetworkManager with the **netplan** renderer, so
`nmcli connection modify` writes through to `/etc/netplan/90-NM-<uuid>.yaml`,
which is regenerated into the live NM connection on every boot — confirmed
persistent by an actual reboot test (route/DNS/connectivity all survived).

## Adding drone2 / drone3

Prerequisite: the Pi must already be joined to FFT (i.e. it has a saved
NetworkManager profile literally named `FFT` — check with
`nmcli connection show` on the Pi if unsure).

```
./pi_fft_gateway.sh drone2-pi.local drone2 <drone2-ssh-password>
./pi_fft_gateway.sh drone3-pi.local drone3 <drone3-ssh-password>
```

The script SSHes in, applies the DNS + route config, reapplies the
connection, and runs a verification ping (base station, 8.8.8.8, google.com).
No base-station-side changes are needed per drone — one gateway serves all
of them.

Note: the script takes the SSH password as a plain argument, which lands in
your shell history. Fine for this local, throwaway use; don't reuse it for
anything beyond the drones' SSH login.

## Verifying

From a drone Pi:
```
ip route                    # should show: default via 192.168.1.103 dev wlan0 metric 100
resolvectl dns wlan0        # should show: 8.8.8.8
ping -c 2 192.168.1.103     # base station over FFT
ping -c 2 8.8.8.8           # raw internet
ping -c 2 google.com        # DNS + internet
```

From the base station:
```
sudo iptables -L FORWARD -n -v   # packet counters on the wlo1<->enx8a882d85e643 rules should increase
ip route get 8.8.8.8             # should still go out enx8a882d85e643, not wlo1
```

## Reversing / disabling

**Stop one drone from routing through the base station** (run on that Pi):
```
sudo nmcli connection modify FFT -ipv4.routes "0.0.0.0/0 192.168.1.103 100"
sudo nmcli connection modify FFT ipv4.dns "" ipv4.ignore-auto-dns no
sudo nmcli device reapply wlan0
```
Falls back to FFT-only (no internet), as before.

**Disable the base station's forwarding entirely:**
```
sudo iptables -t nat -D POSTROUTING -o enx8a882d85e643 -j MASQUERADE
sudo iptables -D FORWARD -i wlo1 -o enx8a882d85e643 -j ACCEPT
sudo iptables -D FORWARD -i enx8a882d85e643 -o wlo1 -m state --state ESTABLISHED,RELATED -j ACCEPT
sudo netfilter-persistent save
```
Optionally also set `net.ipv4.ip_forward=0` in `/etc/sysctl.conf` and
`sudo sysctl -p`, though leaving forwarding on is harmless without the
NAT/FORWARD rules — nothing will actually route.

**Just unplugging the internet dongle:** no action needed. The rules are
interface-name-based; with `enx8a882d85e643` gone they simply have nothing to
match. Drones lose internet automatically, the base station's own traffic is
unaffected, and plugging the dongle back in resumes everything with no
re-running of anything.

## Files

- `base_station_setup.sh` — idempotent, run with `sudo` on the base station.
  Re-running it is safe (deletes-then-reapplies the iptables rules).
- `pi_fft_gateway.sh` — run from the base station against a drone Pi:
  `./pi_fft_gateway.sh <pi-host> <ssh-user> <ssh-password>`
