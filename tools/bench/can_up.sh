#!/usr/bin/env bash
# SPDX-License-Identifier: MIT
# Bring up the USB-CAN adapter (candleLight firmware, kernel driver gs_usb) as a SocketCAN interface at 1 Mbit/s, asking the
# kernel to restart the controller 100 ms after a bus-off (TFC-FDIR-010 on the PC side). Needs sudo. Safe to re-run.
# The SH-C31A (DSD TECH, Cannable 2.0) on 5 Oct 2026 refused the restart option ("Device doesn't support restart from Bus Off"),
# which made the whole command fail and left no can0. The script now tries it first and, if the driver refuses, brings the
# interface up without it and says so: after a bus-off the PC side then needs `ip link set can0 down; ip link set can0 up`.
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
if ! sudo ip link set "$IFACE" type can bitrate "$BITRATE" restart-ms 100 2>/dev/null; then
  echo "warning: $IFACE does not support an automatic bus-off restart; bringing it up without it." >&2
  echo "         After a bus-off, restart by hand:  sudo ip link set $IFACE down; sudo ip link set $IFACE up" >&2
  sudo ip link set "$IFACE" type can bitrate "$BITRATE"
fi
sudo ip link set "$IFACE" txqueuelen 1000
sudo ip link set "$IFACE" up
ip -details -brief link show "$IFACE"
echo "up. Watch the bus:  python3 -m tfc_peers listen --iface $IFACE   (or candump $IFACE)"
