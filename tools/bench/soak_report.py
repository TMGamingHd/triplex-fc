#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""What a long run with nothing wrong proves about false alarms.

The campaign measures the detection of faults well and cannot measure the other half: how often a computer that is *healthy* is isolated, because that depends on how the real IMUs, the real
bus and the real clocks behave when nothing is injected. TS-1 sets the target at under one false isolation per 100 hours of flight. A run with no fault, long enough, bounds it; this tool turns
the record of such a run into the bound.

    python3 -m tfc_peers log --iface can0 --out logs/soak.log --duration 36000       # ten hours, three flight computers and ACT up, no faults injected, no operator command
    python3 tools/bench/soak_report.py logs/soak.log [--confidence 0.95] [--target-per-hour 0.01] [--strict]

From the heartbeats (0x400 to 0x402: each computer's view of the three, its Safe request, its bus alarm, its reset counter) and ACT's output (0x300) it counts, over the frames that SYNC numbered:
the **isolations** (a computer's view of a node changing from healthy to latched, on probation or disabled, counted per episode and, being a healthy run, all of them false), the Safe requests,
the bus alarms, the resets, and ACT leaving Nominal. The rate bound is the exact one-sided upper limit for a Poisson count (Garwood): with no event it is -ln(1 - confidence) / hours, about 3 / hours at
95 %. To show under one false isolation in 100 hours at 95 % with none seen takes 300 hours; the report says how many hours were run and how many a demonstration would need.
Exit status 0; with --strict, 1 if any event was seen or the bound is above the target.
"""
from __future__ import annotations

import argparse
import math
import re
import sys
from collections import defaultdict
from pathlib import Path

LINE = re.compile(r"^\((\d+)\.(\d+)\)\s+\S+\s+([0-9A-Fa-f]+)#([0-9A-Fa-f]*)\s*$")
SYNC = 0x010
HEARTBEAT = 0x400
ACT_OUT = 0x300
FRAME_S = 0.01
STATE_NAME = {0: "healthy", 1: "latched", 2: "probation", 3: "disabled"}


def poisson_cdf(k: int, lam: float) -> float:
    term = math.exp(-lam)
    total = term
    for i in range(1, k + 1):
        term *= lam / i
        total += term
    return total


def upper_limit(events: int, confidence: float) -> float:
    """The exact one-sided upper confidence limit of a Poisson mean for `events` seen: the lambda at which seeing `events` or fewer has probability 1 - confidence."""
    target = 1.0 - confidence
    lo, hi = 0.0, float(events) + 10.0 * math.sqrt(events + 1.0) + 10.0
    for _ in range(200):
        mid = 0.5 * (lo + hi)
        if poisson_cdf(events, mid) > target:
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)


def frames_of(path: Path):
    for line in path.read_text().splitlines():
        m = LINE.match(line.strip())
        if m:
            yield int(m.group(1)) * 1_000_000 + int(m.group(2).ljust(6, "0")[:6]), int(m.group(3), 16), bytes.fromhex(m.group(4))


def analyse(frames) -> dict:
    sync_frames = 0
    first_t = last_t = None
    view: dict[tuple[int, int], int] = {}          # (reporter, subject) -> last state seen
    episodes: dict[tuple[int, int], int] = defaultdict(int)
    safe_prev: dict[int, bool] = {}
    alarm_prev: dict[int, bool] = {}
    reset_prev: dict[int, int] = {}
    safe_req = bus_alarm = resets = 0
    act_prev = None
    act_left_nominal = 0
    heartbeats = defaultdict(int)
    for t, can_id, data in frames:
        first_t = t if first_t is None else first_t
        last_t = t
        if can_id == SYNC and len(data) == 8:
            sync_frames += 1
        elif HEARTBEAT <= can_id < HEARTBEAT + 3 and len(data) == 8:
            rep = can_id - HEARTBEAT
            heartbeats[rep] += 1
            for subject in range(3):
                st = (data[2] >> (2 * subject)) & 3
                if view.get((rep, subject), 0) == 0 and st != 0:
                    episodes[(rep, subject)] += 1
                view[(rep, subject)] = st
            safe = bool((data[1] >> 2) & 1)
            alarm = bool((data[1] >> 3) & 1)
            if safe and not safe_prev.get(rep, False):
                safe_req += 1
            if alarm and not alarm_prev.get(rep, False):
                bus_alarm += 1
            safe_prev[rep], alarm_prev[rep] = safe, alarm
            if rep in reset_prev and data[3] != reset_prev[rep]:
                resets += 1
            reset_prev[rep] = data[3]
        elif can_id == ACT_OUT and len(data) == 8:
            state = (data[4] | (data[5] << 8)) & 7
            if act_prev == 1 and state != 1:
                act_left_nominal += 1
            act_prev = state
    # one isolation seen by all three reporters is one event: take the most any reporter saw (a healthy run has all of them agree; a reporter that disagrees is itself a finding)
    isolations = max((sum(episodes[(r, s)] for s in range(3)) for r in range(3)), default=0)
    return {"sync_frames": sync_frames, "seconds": 0.0 if first_t is None else (last_t - first_t) / 1e6, "isolations": isolations, "episodes": dict(episodes),
            "safe_requests": safe_req, "bus_alarms": bus_alarm, "resets": resets, "act_left_nominal": act_left_nominal, "heartbeats": dict(heartbeats)}


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("log", type=Path)
    ap.add_argument("--confidence", type=float, default=0.95)
    ap.add_argument("--target-per-hour", type=float, default=0.01, help="the false-isolation target (TS-1: one per 100 hours)")
    ap.add_argument("--strict", action="store_true")
    a = ap.parse_args(argv)
    r = analyse(frames_of(a.log))
    hours = r["sync_frames"] * FRAME_S / 3600.0
    if r["sync_frames"] < 1000:
        print(f"only {r['sync_frames']} SYNC frames in {a.log}: not a run")
        return 2
    print(f"{a.log}: {r['sync_frames']} frames ({hours:.2f} h by SYNC, {r['seconds'] / 3600.0:.2f} h by the clock), heartbeats {dict(sorted(r['heartbeats'].items()))}")
    if abs(hours - r["seconds"] / 3600.0) > 0.02 * max(hours, 1e-9):
        print("  note: the frames SYNC counted and the clock disagree by more than 2 %: frames were lost at the capture point; the bound below uses the frames counted")
    rows = [("isolations (a healthy computer put out of the vote)", r["isolations"]), ("Safe requests", r["safe_requests"]), ("bus alarms", r["bus_alarms"]),
            ("resets", r["resets"]), ("ACT left Nominal", r["act_left_nominal"])]
    worst = 0.0
    for name, k in rows:
        up = upper_limit(k, a.confidence) / hours
        worst = max(worst, up) if name.startswith("isolations") else worst
        print(f"  {name:<52} {k:>4}   rate at most {up:.4f} per hour ({a.confidence:.0%}), one in {1.0 / up:,.0f} hours" if up > 0 else f"  {name:<52} {k:>4}")
    if r["isolations"]:
        print("  who: " + ", ".join(f"computer {'ABC'[rep]} saw {'ABC'[s]} {n} time(s)" for (rep, s), n in sorted(r["episodes"].items())))
    need = -math.log(1.0 - a.confidence) / a.target_per_hour
    print(f"  to show a rate under {a.target_per_hour:g} per hour at {a.confidence:.0%} with none seen takes {need:.0f} hours; this run was {hours:.1f}")
    met = r["isolations"] == 0 and hours >= need
    print("  " + ("TARGET DEMONSTRATED" if met else "target not demonstrated" + (" (a false isolation was seen: find out why before anything else)" if r["isolations"] else " (not enough hours)")))
    return 1 if a.strict and (r["isolations"] or worst > a.target_per_hour) else 0


if __name__ == "__main__":
    sys.exit(main())
