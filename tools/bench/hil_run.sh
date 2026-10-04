#!/usr/bin/env bash
# SPDX-License-Identifier: MIT
# One bench run: record the bus into a log while the virtual peers run, then ask the real core, offline, what a flight
# computer would have decided about what was on the bus (tfc_replay). The same command works on vcan0 and on can0.
#   tools/bench/hil_run.sh IFACE SECONDS [tfc_peers run arguments...]
#   tools/bench/hil_run.sh vcan0 20 --nodes B,C --fault B:bias:start=400,mag=3
# The peers follow SYNC from the flight computer, so start the flight computer first. The log goes to logs/hil-<time>.log.
set -euo pipefail
cd "$(dirname "$0")/../.."
[ $# -ge 2 ] || { sed -n '2,9p' "$0" | sed 's/^# \{0,1\}//'; exit 2; }
IFACE="$1"; SECS="$2"; shift 2
mkdir -p logs
LOG="logs/hil-$(date +%Y%m%d-%H%M%S).log"
cd sim
python3 -m tfc_peers log --iface "$IFACE" --out "../$LOG" --duration "$((SECS + 2))" &
LOGGER=$!
sleep 0.5
timeout "$SECS" python3 -m tfc_peers run --iface "$IFACE" --follow-sync --frames 0 "$@" || true
wait "$LOGGER"
cd ..
echo
echo "log: $LOG ($(wc -l < "$LOG") frames)"
# The log's clock is the PC's, not the flight computer's: the replay is aligned to the first SYNC (tools/bench/log_t0.py) and is
# approximate where a frame is within a few hundred microseconds of the vote deadline. It judges the content, not the real timing.
if [ -x build/host/tfc_replay ]; then
  ALIGN="$(python3 tools/bench/log_t0.py "$LOG" || true)"
  if [ -z "$ALIGN" ]; then echo "no SYNC in the log: is the flight computer running? Nothing to align the replay to."; exit 1; fi
  read -r T0 F0 <<< "$ALIGN"
  build/host/tfc_replay "$LOG" --t0 "$T0" --first-frame "$F0" --startup-grace 500 --verbose | head -40
else
  echo "build/host/tfc_replay is not built; decode with: python3 -m tfc_peers decode $LOG"
fi
