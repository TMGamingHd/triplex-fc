# SPDX-License-Identifier: MIT
"""TS-15: how a computer degrades when its sensing does (docs/TRADE_STUDIES.md section 9a). Runs the same fault scenarios with the manager as it was (option A: a
computer is one unit) and with the sensor split (option B), and tabulates, per class of fault: command-voter availability (computer-frames that took part in the
command vote, as a share of the possible ones after the fault starts), false isolations (computers latched that have no fault of their own), Safe requests, and the
frames with a non-held output beyond the vote tolerance.

    python3 -m campaign.ts15 [--seeds N]        (TFC_REPLAY_BIN names the release tfc_replay)
"""
from __future__ import annotations

import argparse
import itertools
import os
import tempfile
from collections import defaultdict

from tfc_peers import peers
from . import oracles
from .model import TOL, Scenario
from .runner import execute

N = "ABC"
STARTS = (60, 100, 140)
SENSOR = {"bias": "{n}:bias:start={s},mag=3.0", "stuck": "{n}:stuck:start={s}", "saturate": "{n}:saturate:start={s}", "noise": "{n}:noise:start={s},mult=30"}
COMMAND = {"cmd_offset": "{n}:cmd_offset:start={s},mag=1.0", "cmdinvert": "{n}:cmdinvert:start={s}"}


def scenarios() -> list[tuple[str, Scenario, set[int], set[int]]]:
    """(class, scenario, computers with a fault of their own, IMUs with a fault of their own)."""
    out = []

    def add(cls, faults, comp, imu, ctx="triplex", frames=0, start=100):
        out.append((cls, Scenario(group="ts15", faults=faults, context=ctx, frames=frames or start + 200), comp, imu))

    for (k, t), n, s in itertools.product(SENSOR.items(), range(3), STARTS):
        add("one IMU (Triplex)", [t.format(n=N[n], s=s)], set(), {n}, start=s)
        add("one IMU (Duplex)", [t.format(n=N[n], s=s)], set(), {n}, ctx=f"duplex-{N[(n + 1) % 3]}", start=s)
    for (k1, t1), (k2, t2), (a, b), s in itertools.product(SENSOR.items(), SENSOR.items(), ((0, 1), (1, 2), (2, 0)), STARTS):
        if k1 <= k2:
            add("two IMUs", [t1.format(n=N[a], s=s), t2.format(n=N[b], s=s + 3)], set(), {a, b}, start=s)
    for (k, t), n, s in itertools.product(COMMAND.items(), range(3), STARTS):
        add("one computer (commands)", [t.format(n=N[n], s=s)], {n}, set(), start=s)
    for (ik, it), (ck, ct), (a, b), s in itertools.product(SENSOR.items(), COMMAND.items(), ((0, 1), (1, 2), (2, 0)), STARTS):
        add("IMU of one, commands of another", [it.format(n=N[a], s=s), ct.format(n=N[b], s=s + 3)], {b}, {a}, start=s)
    for (ik, it), (ck, ct), n, s in itertools.product(SENSOR.items(), COMMAND.items(), range(3), STARTS):
        add("IMU and commands of the same computer", [it.format(n=N[n], s=s), ct.format(n=N[n], s=s + 3)], {n}, {n}, start=s)
    return out


def measure(sc: Scenario, split: bool, comp: set[int], imu: set[int]) -> dict:
    sc = Scenario(**{**sc.__dict__, "split": split})
    with tempfile.TemporaryDirectory(prefix="tfc-ts15-") as tmp:
        kv, rows, _ = execute(sc, tmp)
    start = min(int(f.split("start=")[1].split(",")[0]) for f in sc.faults)
    after = [r for r in rows if r["frame"] >= start]
    possible = 3 * len(after)
    voted = sum(bin(r["valid"] & 7).count("1") for r in after)
    dropped = {"ABC".index(c) for c in sc.context.split("-")[1]} if "-" in sc.context else set()  # the context's deliberate dropouts are not isolations to count
    false_comp = {n for n in range(3) if any(r["latched"] & (1 << n) for r in rows)} - comp - dropped
    exposure = 0
    for r in after:
        truth = oracles.truth_channels(r["frame"], False)
        if any(not (r["held"] & (1 << ch)) and abs(r[f"out{ch}"] - truth[ch]) / TOL[ch] > 1.0 for ch in range(8)):
            exposure += 1
    return {"voted": voted, "possible": possible, "false": len(false_comp), "safe": any(r["safe"] for r in rows), "exposure": exposure,
            "frames": len(after), "sensor_voted": sum(bin(r["s_valid"] & 7).count("1") for r in after) if split else None}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="ts15", description=__doc__)
    ap.parse_args(argv)
    peers_truth = peers.truth  # (the scenarios use the standard motion)
    del peers_truth
    table: dict[str, dict[str, list]] = defaultdict(lambda: {"A": [], "B": []})
    for cls, sc, comp, imu in scenarios():
        table[cls]["A"].append(measure(sc, False, comp, imu))
        table[cls]["B"].append(measure(sc, True, comp, imu))
    print("| Class of fault | Runs | Option | Command-voter availability | False isolations (computers) | Safe requests | Frames with the output beyond tolerance |")
    print("|---|---|---|---|---|---|---|")
    for cls, d in table.items():
        for opt in ("A", "B"):
            rs = d[opt]
            avail = 100.0 * sum(r["voted"] for r in rs) / sum(r["possible"] for r in rs)
            print(f"| {cls} | {len(rs)} | {opt} | {avail:.1f} % | {sum(r['false'] for r in rs)} | {sum(1 for r in rs if r['safe'])} of {len(rs)} | {sum(r['exposure'] for r in rs)} |")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
