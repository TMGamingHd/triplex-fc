#!/usr/bin/env bash
# SPDX-License-Identifier: MIT
# The software triplex on one vcan0: three instances of the real firmware (native_sim), nodes A, B and C, with nothing from Python but a monitor.
#   tools/bench/sil_triplex.sh --build [--flight] [--sim-imu]   build the three images into build/native_sim, native_sim_n1, native_sim_n2
#   tools/bench/sil_triplex.sh --test                           run the live test (sim/tests/test_live_triplex.py) against them
#   tools/bench/sil_triplex.sh --run [seconds]                  run the three for a while and print each node's console (default 10 s)
# --flight : the command comes from the flight function (CONFIG_TFC_FLIGHT_FUNCTION); --sim-imu : the sensors come from the simulator's frames on the bus
# (CONFIG_TFC_SIM_BUS_IMU). Needs vcan0 (sim/scripts/setup_vcan.sh) and the Zephyr environment (firmware/env.sh).
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT"
mode="${1:-}"; shift || true
flight=n
simimu=n
secs=10
for a in "$@"; do
  case "$a" in
    --flight) flight=y ;;
    --sim-imu) simimu=y ;;
    [0-9]*) secs="$a" ;;
    *) echo "unknown option $a" >&2; exit 2 ;;
  esac
done
case "$mode" in
  --build)
    # shellcheck disable=SC1091
    . firmware/env.sh
    for n in 0 1 2; do
      dir=build/native_sim; [ "$n" != 0 ] && dir="build/native_sim_n$n"
      west build -p auto -b native_sim/native/64 firmware/app -d "$dir" -- -DCONFIG_TFC_NODE_ID="$n" -DCONFIG_TFC_FLIGHT_FUNCTION="$flight" -DCONFIG_TFC_SIM_BUS_IMU="$simimu"
    done ;;
  --test)
    (cd sim && python3 -m unittest tests.test_live_triplex -v) ;;
  --run)
    tmp="$(mktemp -d)"
    trap 'kill $(jobs -p) 2>/dev/null || true' EXIT
    build/native_sim/zephyr/zephyr.exe >"$tmp/a.log" 2>&1 &
    build/native_sim_n1/zephyr/zephyr.exe >"$tmp/b.log" 2>&1 &
    build/native_sim_n2/zephyr/zephyr.exe >"$tmp/c.log" 2>&1 &
    sleep "$secs"
    kill $(jobs -p) 2>/dev/null || true
    wait 2>/dev/null || true
    for f in a b c; do echo "=== node ${f^^}"; cat "$tmp/$f.log"; done
    rm -rf "$tmp" ;;
  *) sed -n 2,9p "$0"; exit 2 ;;
esac
