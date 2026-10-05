#!/usr/bin/env bash
# SPDX-License-Identifier: MIT
# The software triplex on one vcan0: three instances of the real firmware (native_sim), nodes A, B and C, with nothing from Python but a monitor.
#   tools/bench/sil_triplex.sh --build [--flight] [--sim-imu] [--launch]   build the three flight computers into build/native_sim, native_sim_n1, native_sim_n2,
#                                                               and the actuator node into build/act_native
#   tools/bench/sil_triplex.sh --build-drop                     one more flight image, build/flight_b_drop: node B withholds every sensor frame from its flight function in
#                                                               frames 370 to 398, so the replicas diverge deterministically (the live resync test)
#   tools/bench/sil_triplex.sh --build-t0                       the launch images with the T0 line simulated 40 frames before the end of the countdown: build/launch_t0_a, _b, _c (the live T0 test)
#   tools/bench/sil_triplex.sh --build-phases                   the flight images with the mission phases and the WARM role on: build/phases_a, _b, _c (the live phases test)
#   tools/bench/sil_triplex.sh --build-bias                     one more flight image, build/flight_b_bias: node B's gyro reads 3 dps too much (the live sensor-split test)
#   tools/bench/sil_triplex.sh --build-mixed                    the mixed-release set (ADR-021): build/mixed_a, _b (release 0xA001), mixed_c (release 0xB002, commands 0.05 degree off) and
#                                                               mixed_c_near (release 0xB002, commands 0.012 degree off, inside the version tolerance)
#   tools/bench/sil_triplex.sh --test                           run the live test (sim/tests/test_live_triplex.py) against them
#   tools/bench/sil_triplex.sh --closed-loop [seconds]          the closed loop: the loop_* images, ACT and the vehicle simulator (build/host/tfc_simd) on vcan0
#   tools/bench/sil_triplex.sh --run [seconds]                  run the four for a while and print each node's console (default 10 s)
# By default the command is the scripted function of the frame number that the Python virtual peers reproduce: the digests then do not depend on the inputs, so
# the test of SYNC takeover is not disturbed by a late frame. --flight: the command comes from the flight function (CONFIG_TFC_FLIGHT_FUNCTION: consensus,
# estimator, controller), which is the closed loop but is sensitive to TS-16 (replicated estimators on a lossy bus). --sim-imu: the sensors come from the
# simulator's frames on the bus (CONFIG_TFC_SIM_BUS_IMU). Needs vcan0 (sim/scripts/setup_vcan.sh) and the Zephyr environment (firmware/env.sh).
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT"
mode="${1:-}"; shift || true
flight=n
simimu=n
launch=n
prefix=triplex
secs=10
for a in "$@"; do
  case "$a" in
    --flight) flight=y; prefix=flight ;;
    --sim-imu) simimu=y; prefix=loop ;;
    --launch) flight=y; simimu=y; launch=y; prefix=launch ;;
    [0-9]*) secs="$a" ;;
    *) echo "unknown option $a" >&2; exit 2 ;;
  esac
done
case "$mode" in
  --build)
    # shellcheck disable=SC1091
    . firmware/env.sh
    for n in 0 1 2; do
      letters=(a b c)
      dir="build/${prefix}_${letters[$n]}"
      west build -p auto -b native_sim/native/64 firmware/app -d "$dir" -- -DCONFIG_TFC_NODE_ID="$n" -DCONFIG_TFC_FLIGHT_FUNCTION="$flight" -DCONFIG_TFC_SIM_BUS_IMU="$simimu" -DCONFIG_TFC_LAUNCH_SEQUENCE="$launch"
    done
    west build -p auto -b native_sim/native/64 firmware/act -d build/act_native ;;
  --build-drop)
    # shellcheck disable=SC1091
    . firmware/env.sh
    west build -p auto -b native_sim/native/64 firmware/app -d build/flight_b_drop -- -DCONFIG_TFC_NODE_ID=1 -DCONFIG_TFC_FLIGHT_FUNCTION=y -DCONFIG_TFC_TEST_DROP_PEERS_FIRST=370 -DCONFIG_TFC_TEST_DROP_PEERS_FRAMES=29 ;;
  --build-mixed)
    # shellcheck disable=SC1091
    . firmware/env.sh
    for spec in "a 0 0xA001 0" "b 1 0xA001 0" "c 2 0xB002 50" "c_near 2 0xB002 12"; do
      set -- $spec
      west build -p auto -b native_sim/native/64 firmware/app -d "build/mixed_$1" -- -DCONFIG_TFC_NODE_ID="$2" -DCONFIG_TFC_FLIGHT_FUNCTION=y -DCONFIG_TFC_RELEASE_ID="$3" -DCONFIG_TFC_TEST_CMD_OFFSET_MDEG="$4"
    done ;;
  --build-t0)
    # shellcheck disable=SC1091
    . firmware/env.sh
    for n in 0 1 2; do
      letters=(a b c)
      west build -p auto -b native_sim/native/64 firmware/app -d "build/launch_t0_${letters[$n]}" -- -DCONFIG_TFC_NODE_ID="$n" -DCONFIG_TFC_FLIGHT_FUNCTION=y -DCONFIG_TFC_SIM_BUS_IMU=y -DCONFIG_TFC_LAUNCH_SEQUENCE=y -DCONFIG_TFC_TEST_T0_AT_FRAMES_TO_ZERO=40
    done ;;
  --build-phases)
    # shellcheck disable=SC1091
    . firmware/env.sh
    for n in 0 1 2; do
      letters=(a b c)
      west build -p auto -b native_sim/native/64 firmware/app -d "build/phases_${letters[$n]}" -- -DCONFIG_TFC_NODE_ID="$n" -DCONFIG_TFC_FLIGHT_FUNCTION=y -DCONFIG_TFC_PHASES=y
    done ;;
  --build-bias)
    # shellcheck disable=SC1091 disable=SC1091 disable=SC1091
    . firmware/env.sh
    west build -p auto -b native_sim/native/64 firmware/app -d build/flight_b_bias -- -DCONFIG_TFC_NODE_ID=1 -DCONFIG_TFC_FLIGHT_FUNCTION=y -DCONFIG_TFC_TEST_GYRO_BIAS_MDPS=3000 ;;
  --test)
    (cd sim && python3 -m unittest tests.test_live_triplex tests.test_live_closed_loop tests.test_live_resync tests.test_live_split tests.test_live_release tests.test_live_phases -v) ;;
  --closed-loop)
    tmp="$(mktemp -d)"
    trap 'kill $(jobs -p) 2>/dev/null || true' EXIT
    for n in a b c; do build/loop_$n/zephyr/zephyr.exe >"$tmp/$n.log" 2>&1 & done
    build/act_native/zephyr/zephyr.exe >"$tmp/act.log" 2>&1 &
    sleep 0.5
    build/host/tfc_simd --iface vcan0 >"$tmp/sim.log" 2>&1 &
    sleep "${1:-${secs}}"
    kill $(jobs -p) 2>/dev/null || true
    wait 2>/dev/null || true
    echo "=== simulator"; cat "$tmp/sim.log"
    for f in a act; do echo "=== node ${f^^}"; grep -v '^\[frame [0-9]*00\] ' "$tmp/$f.log" || true; done
    rm -rf "$tmp" ;;
  --run)
    tmp="$(mktemp -d)"
    trap 'kill $(jobs -p) 2>/dev/null || true' EXIT
    build/${prefix}_a/zephyr/zephyr.exe >"$tmp/a.log" 2>&1 &
    build/${prefix}_b/zephyr/zephyr.exe >"$tmp/b.log" 2>&1 &
    build/${prefix}_c/zephyr/zephyr.exe >"$tmp/c.log" 2>&1 &
    build/act_native/zephyr/zephyr.exe >"$tmp/act.log" 2>&1 &
    sleep "$secs"
    kill $(jobs -p) 2>/dev/null || true
    wait 2>/dev/null || true
    for f in a b c act; do echo "=== node ${f^^}"; cat "$tmp/$f.log"; done
    rm -rf "$tmp" ;;
  *) sed -n 2,9p "$0"; exit 2 ;;
esac
