#!/bin/bash
# Base station gateway setup: bridge FFT wifi (wlo1) <-> internet (enx8a882d85e643)
set -e

FFT_IF="wlo1"
WAN_IF="enx8a882d85e643"

echo "=== 1. IP forwarding ==="
sysctl -w net.ipv4.ip_forward=1
if grep -qE '^\s*net\.ipv4\.ip_forward\s*=' /etc/sysctl.conf; then
    sed -i 's/^\s*net\.ipv4\.ip_forward\s*=.*/net.ipv4.ip_forward=1/' /etc/sysctl.conf
else
    echo "net.ipv4.ip_forward=1" >> /etc/sysctl.conf
fi
sysctl -p /etc/sysctl.conf
grep '^net.ipv4.ip_forward' /etc/sysctl.conf

echo
echo "=== 2. ufw check ==="
UFW_ACTIVE=false
if command -v ufw >/dev/null 2>&1 && ufw status | grep -q "Status: active"; then
    UFW_ACTIVE=true
    echo "ufw is active -> setting default forward policy to ACCEPT and adding route rule"
    sed -i 's/^DEFAULT_FORWARD_POLICY=.*/DEFAULT_FORWARD_POLICY="ACCEPT"/' /etc/default/ufw
    ufw route allow in on "$FFT_IF" out on "$WAN_IF" || true
    ufw reload
else
    echo "ufw not active, skipping ufw-specific config"
fi

echo
echo "=== 3. iptables NAT/FORWARD rules ==="
# Idempotent: delete first (ignore errors) then add, so re-running this script is safe
iptables -t nat -D POSTROUTING -o "$WAN_IF" -j MASQUERADE 2>/dev/null || true
iptables -D FORWARD -i "$FFT_IF" -o "$WAN_IF" -j ACCEPT 2>/dev/null || true
iptables -D FORWARD -i "$WAN_IF" -o "$FFT_IF" -m state --state ESTABLISHED,RELATED -j ACCEPT 2>/dev/null || true

iptables -t nat -A POSTROUTING -o "$WAN_IF" -j MASQUERADE
iptables -A FORWARD -i "$FFT_IF" -o "$WAN_IF" -j ACCEPT
iptables -A FORWARD -i "$WAN_IF" -o "$FFT_IF" -m state --state ESTABLISHED,RELATED -j ACCEPT

echo "Current rules:"
iptables -t nat -L POSTROUTING -n -v
iptables -L FORWARD -n -v

echo
echo "=== 4. Persist iptables rules ==="
export DEBIAN_FRONTEND=noninteractive
if ! dpkg -l | grep -q iptables-persistent; then
    echo iptables-persistent iptables-persistent/autosave_v4 boolean true | debconf-set-selections
    echo iptables-persistent iptables-persistent/autosave_v6 boolean true | debconf-set-selections
    apt-get update -qq
    apt-get install -y iptables-persistent
fi
netfilter-persistent save
echo "Saved to /etc/iptables/rules.v4"

echo
echo "=== 5. Base station FFT-side IP (use as Pi gateway) ==="
ip -4 addr show "$FFT_IF" | grep inet

echo
echo "=== DONE ==="
