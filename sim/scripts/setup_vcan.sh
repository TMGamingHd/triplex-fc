#!/usr/bin/env bash
# Create a virtual CAN interface (default vcan0) so the virtual peers can run without hardware.
# Needs sudo (kernel module + network interface). Safe to re-run. Undo: sudo ip link del vcan0
set -euo pipefail
IFACE="${1:-vcan0}"
sudo modprobe vcan
if ! ip link show "$IFACE" >/dev/null 2>&1; then
  sudo ip link add dev "$IFACE" type vcan
fi
sudo ip link set up "$IFACE"
ip -brief link show "$IFACE"
