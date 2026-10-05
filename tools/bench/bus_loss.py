#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""How much of the schedule actually arrives, and when (the real bus loss rate that TS-23 and the command tolerance are waiting for).

Record the bus for an hour or more with the three flight computers and ACT running:

    python3 -m tfc_peers log --iface can0 --out logs/s4.log --duration 3600
    python3 tools/bench/bus_loss.py logs/s4.log [--resync-period 100] [--vote-us 7000] [--console A=a.txt B=b.txt C=c.txt]

For every frame number carried by SYNC the tool says which scheduled frames arrived (gyro, accel and command of each computer, ACT's output, each heartbeat, and in the resync frames the twelve state frames),
and how late after SYNC. It reports, per id and in all, the share of frames **missing at the capture point**, the arrival time against SYNC (mean, 99th percentile, maximum), and the share that came after the
vote time. A capture point sees what was put on the bus; what each computer *received* is in its own console (`--console`: the last status line's `missing`, `crc` and `seq` counters against its frame count),
and that is the figure that feeds the digest persistence and the command tolerance. The closing section says what the numbers mean for the settings in `docs/RESYNC.md` and `docs/TRADE_STUDIES.md` (TS-23).
Exit status 0; the tool reports and never judges.
"""
from __future__ import annotations

import argparse
import re
import sys
from collections import defaultdict
from pathlib import Path

LINE = re.compile(r"^\((\d+)\.(\d+)\)\s+\S+\s+([0-9A-Fa-f]+)#([0-9A-Fa-f]*)\s*$")
STATUS = re.compile(r"\[frame (\d+)\].*?\| crc=(\d+) seq=(\d+) missing=(\d+)")
SYNC = 0x010
SCHEDULE = {**{0x100 + n: f"gyro {'ABC'[n]}" for n in range(3)}, **{0x110 + n: f"accel {'ABC'[n]}" for n in range(3)}, **{0x200 + n: f"command {'ABC'[n]}" for n in range(3)},
            0x300: "ACT output", **{0x400 + n: f"heartbeat {'ABC'[n]}" for n in range(3)}}


def read(path: Path):
    for line in path.read_text().splitlines():
        m = LINE.match(line.strip())
        if m:
            yield int(m.group(1)) * 1_000_000 + int(m.group(2).ljust(6, "0")[:6]), int(m.group(3), 16), bytes.fromhex(m.group(4))


def analyse(frames, resync_period: int = 0, vote_us: int = 7000):
    """Returns (stats, n_frames): stats[id] = {"expected", "seen", "delays"}."""
    cycles: dict[int, dict[int, int]] = {}      # frame number -> {id: delay after SYNC (us)}
    sync_t: dict[int, int] = {}
    current = None
    for t, can_id, data in frames:
        if can_id == SYNC and len(data) >= 4:
            current = int.from_bytes(data[:4], "little")
            sync_t[current] = t
            cycles.setdefault(current, {})
            continue
        if current is None:
            continue
        if can_id in SCHEDULE or 0x420 <= can_id < 0x42C:
            cycles[current].setdefault(can_id, t - sync_t[current])
    numbers = sorted(cycles)
    if len(numbers) < 3:
        return {}, 0
    numbers = numbers[1:-1]  # the first and last cycles are cut by the capture
    stats: dict[int, dict] = defaultdict(lambda: {"expected": 0, "seen": 0, "delays": []})
    for k in numbers:
        want = list(SCHEDULE)
        if resync_period and k % resync_period == resync_period - 1:
            want += list(range(0x420, 0x42C))
        for can_id in want:
            s = stats[can_id]
            s["expected"] += 1
            if can_id in cycles[k]:
                s["seen"] += 1
                s["delays"].append(cycles[k][can_id])
    return dict(stats), len(numbers)


def pct(values, p):
    if not values:
        return 0
    v = sorted(values)
    return v[min(len(v) - 1, int(round(p / 100.0 * (len(v) - 1))))]


def console_loss(text: str):
    """(frames, missing, crc, seq) from the last status line of a console log, or None."""
    last = None
    for line in text.splitlines():
        m = STATUS.search(line)
        if m:
            frame, crc, seq, missing = (int(g) for g in m.groups())  # the status line's order
            last = (frame, missing, crc, seq)
    return last


def report(stats, n_frames, vote_us, consoles) -> str:
    out: list[str] = []
    out.append(f"{n_frames} frames analysed.\n")
    out.append("| Frame | Expected | Missing | Missing % | Delay mean (us) | Delay p99 (us) | Delay max (us) | After the vote time |")
    out.append("|---|---|---|---|---|---|---|---|")
    tot_e = tot_m = 0
    for can_id in sorted(stats):
        s = stats[can_id]
        if not s["expected"]:
            continue
        miss = s["expected"] - s["seen"]
        tot_e += s["expected"]
        tot_m += miss
        d = s["delays"]
        name = SCHEDULE.get(can_id, f"resync 0x{can_id:03X}")
        late = sum(1 for x in d if x > vote_us)
        mean = int(sum(d) / len(d)) if d else 0
        out.append(f"| {name} | {s['expected']} | {miss} | {100.0 * miss / s['expected']:.4f} | {mean} | {pct(d, 99)} | {max(d) if d else 0} | {late} |")
    rate = tot_m / tot_e if tot_e else 0.0
    out.append(f"\n**All scheduled frames: {tot_m} missing of {tot_e} ({100.0 * rate:.4f} %) at the capture point.**\n")
    per_node: list[float] = []
    for name, path in consoles:
        got = console_loss(Path(path).read_text())
        if got is None:
            out.append(f"- console {name}: no status line found")
            continue
        frames, missing, crc, seq = got
        r = (missing + crc + seq) / max(1, frames * 9)  # nine slot frames per computer-frame: gyro, accel and command of three computers
        per_node.append(r)
        out.append(f"- console {name}: frame {frames}, missing {missing}, crc {crc}, seq {seq}: **{100.0 * r:.4f} % of the slot frames it was owed**")
    worst = max(per_node) if per_node else rate
    out.append("\n### What it means for the settings (docs/RESYNC.md, TS-23)")
    out.append(f"The figure used below is the worst per-node loss if consoles were given, otherwise the capture-point loss: **{100.0 * worst:.4f} %**.")
    if worst <= 1e-4:
        out.append("- At or below 0.01 %: the defaults hold: resync every 100 frames, digest persistence 250, the 0.01 degree command tolerance. Consider a longer period (200 to 500) only if the burst of twelve frames costs frame budget.")
    elif worst <= 1e-3:
        out.append("- 0.01 % to 0.1 %: the defaults hold (TS-23: no run of three frames over the manager's tolerance with the resync, 0 of 16 flights); a persistence of 250 never false-flagged in 32 patterns at 0.1 %.")
    elif worst <= 1e-2:
        out.append("- 0.1 % to 1 %: shorten the period to 20 to 50 frames so that the digest check stays usable (TS-23), and expect single frames over the 0.01 degree command tolerance; widen the manager's command tolerance toward ACT's 0.05 degree or move to agreement of inputs (docs/RESYNC.md section 6b).")
    else:
        out.append("- Above 1 %: the bus is the problem, not the settings. Look at the wiring, the terminations and the transceiver before tuning (error counters: `bus_off`, `err_passive` in the status line).")
    return "\n".join(out)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="bus_loss", description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("log")
    ap.add_argument("--resync-period", type=int, default=0, help="frames between state resynchronisations (0: none expected)")
    ap.add_argument("--vote-us", type=int, default=7000, help="the vote time after SYNC, for counting late frames (7000 on the board, 8000 on native_sim)")
    ap.add_argument("--console", action="append", default=[], metavar="NAME=FILE", help="a node's console log (repeatable)")
    args = ap.parse_args(argv)
    consoles = [tuple(c.split("=", 1)) for c in args.console]
    stats, n = analyse(read(Path(args.log)), args.resync_period, args.vote_us)
    if not n:
        print("no complete frame cycles in the log (is SYNC in it?)", file=sys.stderr)
        return 2
    print(report(stats, n, args.vote_us, consoles))
    return 0


if __name__ == "__main__":
    sys.exit(main())
