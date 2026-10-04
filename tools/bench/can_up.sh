#!/usr/bin/env bash
# SPDX-License-Identifier: MIT
# Bring up the USB-CAN adapter (candleLight firmware, kernel driver gs_usb) as a SocketCAN interface at 1 Mbit/s, with the
# kernel restarting the controller 100 ms after a bus-off (TFC-FDIR-010 on the PC side). Needs sudo. Safe to re-run.
# UNTESTED until the adapter arrives (sim/README.md says the same): the first run on real hardware is the test.
#   tools/bench/can_up.sh [iface] [bitrate]      defaults: can0 1000000
set -euo pipefail
IFACE="${1:-can0}"
BITRATE="${2:-1000000}"
sudo modprobe gs_usb
if ! ip link show "$IFACE" >/dev/null 2>&1; then
  echo "no interface $IFACE: is the adapter plugged in? (dmesg | tail)" >&2
  exit 1
fi
sudo ip link set "$IFACE" down
sudo ip link set "$IFACE" type can bitrate "$BITRATE" restart-ms 100
sudo ip link set "$IFACE" txqueuelen 1000
sudo ip link set "$IFACE" up
ip -details -brief link show "$IFACE"
echo "up. Watch the bus:  python3 -m tfc_peers listen --iface $IFACE   (or candump $IFACE)"
