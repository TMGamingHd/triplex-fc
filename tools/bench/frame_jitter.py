#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Frame timing from logic-analyser edges (TFC-SYS-001: p99 frame-start jitter no greater than 100 us).

Capture the FRAME line of a node (docs/design/SUPERVISOR.md: it rises at the start of every frame) with the logic analyser, export the edge
times, and run:

    python3 tools/bench/frame_jitter.py edges.csv [--unit s|ms|us|ns] [--period-us 10000] [--p99-us 100]

The file holds one rising edge per line, time first; a second column holding 0 or 1 (a level export) keeps only the rows where it is 1;
lines that do not start with a number (headers) are skipped; commas, tabs and spaces separate columns. The edges are fitted to a straight
line of frame number against time, so a clock that is a few tens of ppm off does not count as jitter (it is reported as a ppm figure
instead); a gap of more than one and a half periods counts the frames that did not happen. Exit status 0 if the run passes, 1 if not.
"""
from __future__ import annotations

import argparse
import math
import re
import sys

UNITS = {"s": 1.0, "ms": 1e-3, "us": 1e-6, "ns": 1e-9}
NUM = re.compile(r"^[-+]?(\d+\.?\d*|\.\d+)([eE][-+]?\d+)?$")


def read_edges(lines, unit_s: float) -> list[float]:
    edges: list[float] = []
    for line in lines:
        cols = [c for c in re.split(r"[,\t ;]+", line.strip()) if c]
        if not cols or not NUM.match(cols[0]):
            continue
        if len(cols) > 1 and cols[1] in ("0", "1") and cols[1] == "0":
            continue  # a level export: only the rows where the line is high
        edges.append(float(cols[0]) * unit_s)
    return edges


def percentile(sorted_values: list[float], p: float) -> float:
    if not sorted_values:
        return 0.0
    k = min(len(sorted_values) - 1, max(0, math.ceil(p / 100.0 * len(sorted_values)) - 1))
    return sorted_values[k]


def analyse(edges: list[float], period_s: float) -> dict:
    n = len(edges)
    if n < 3:
        raise ValueError("need at least three edges")
    periods = [b - a for a, b in zip(edges, edges[1:])]
    missed = sum(max(0, round(p / period_s) - 1) for p in periods if p > 1.5 * period_s)
    short = sum(1 for p in periods if p < 0.5 * period_s)
    idx = [round((t - edges[0]) / period_s) for t in edges]
    mi = sum(idx) / n
    mt = sum(edges) / n
    sxx = sum((i - mi) ** 2 for i in idx)
    slope = sum((i - mi) * (t - mt) for i, t in zip(idx, edges)) / sxx if sxx else period_s
    icpt = mt - slope * mi
    resid_us = sorted(abs(t - (icpt + slope * i)) * 1e6 for i, t in zip(idx, edges))
    period_err = sorted(abs(p - slope) * 1e6 for p in periods if 0.5 * period_s <= p <= 1.5 * period_s)
    return {
        "edges": n,
        "mean_period_us": slope * 1e6,
        "clock_error_ppm": (slope / period_s - 1.0) * 1e6,
        "jitter_p50_us": percentile(resid_us, 50),
        "jitter_p99_us": percentile(resid_us, 99),
        "jitter_max_us": resid_us[-1],
        "period_dev_p99_us": percentile(period_err, 99),
        "period_dev_max_us": period_err[-1] if period_err else 0.0,
        "missed_frames": missed,
        "short_periods": short,
    }


def main(argv: list[str]) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("file", help="edge times, one rising edge per line (or a level export)")
    ap.add_argument("--unit", choices=sorted(UNITS), default="s", help="unit of the time column (default s)")
    ap.add_argument("--period-us", type=float, default=10000.0, help="nominal frame period (default 10000)")
    ap.add_argument("--p99-us", type=float, default=100.0, help="pass threshold for the 99th percentile of the jitter (default 100)")
    a = ap.parse_args(argv)
    with open(a.file, encoding="utf-8") as fh:
        edges = read_edges(fh, UNITS[a.unit])
    try:
        r = analyse(edges, a.period_us * 1e-6)
    except ValueError as e:
        print(f"error: {e}", file=sys.stderr)
        return 2
    print(f"edges={r['edges']}  mean period {r['mean_period_us']:.3f} us ({r['clock_error_ppm']:+.1f} ppm from nominal)")
    print(f"jitter about the fitted grid: p50 {r['jitter_p50_us']:.1f} us, p99 {r['jitter_p99_us']:.1f} us, max {r['jitter_max_us']:.1f} us")
    print(f"period deviation: p99 {r['period_dev_p99_us']:.1f} us, max {r['period_dev_max_us']:.1f} us")
    print(f"missed frames: {r['missed_frames']}   short periods: {r['short_periods']}")
    ok = r["jitter_p99_us"] <= a.p99_us and r["missed_frames"] == 0 and r["short_periods"] == 0
    print("PASS" if ok else f"FAIL (p99 limit {a.p99_us:.0f} us, no missed frames, no short periods)")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
