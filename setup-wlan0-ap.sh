#!/usr/bin/env bash
set -euo pipefail

if [[ $EUID -ne 0 ]]; then
    echo "Run as root: sudo $0 <ssid> <password>" >&2
    exit 1
fi

if [[ $# -ne 2 ]]; then
    echo "Usage: sudo $0 <ssid> <password>" >&2
    exit 2
fi

ssid=$1
password=$2
connection_name=forklift-wlan0-ap

if (( ${#password} < 8 )); then
    echo "The Wi-Fi password must be at least 8 characters." >&2
    exit 2
fi

if ! command -v nmcli >/dev/null 2>&1; then
    echo "NetworkManager is required. Install it with: apt install network-manager" >&2
    exit 1
fi

if ! command -v iw >/dev/null 2>&1; then
    echo "The iw command is required. Install it with: apt install iw" >&2
    exit 1
fi

if ! ip link show wlan0 >/dev/null 2>&1; then
    echo "wlan0 was not found. Check the Pi's Wi-Fi adapter." >&2
    exit 1
fi

nmcli connection delete "$connection_name" >/dev/null 2>&1 || true
nmcli radio wifi on
nmcli connection add \
    type wifi \
    ifname wlan0 \
    con-name "$connection_name" \
    autoconnect yes \
    ssid "$ssid" \
    wifi-sec.key-mgmt wpa-psk \
    wifi-sec.psk "$password" \
    ipv4.method shared \
    802-11-wireless.mode ap \
    802-11-wireless.band bg \
    802-11-wireless.channel 1 \
    ipv4.addresses 192.168.50.1/24 \
    ipv6.method disabled
nmcli connection up "$connection_name"

if ! nmcli -g GENERAL.STATE device show wlan0 | grep -q '^100'; then
    echo "NetworkManager did not connect wlan0." >&2
    exit 1
fi

if ! iw dev wlan0 info | grep -q 'type AP'; then
    echo "wlan0 connected, but it is not operating as an access point." >&2
    exit 1
fi

cat <<'EOF'
Access point is active on wlan0.
Connect a computer to the configured SSID, then open:
  https://192.168.50.1/

NetworkManager provides DHCP on 192.168.50.0/24. The application still
requires the configured HTTPS login and the browser may warn about the
certificate when using the IP address.
EOF