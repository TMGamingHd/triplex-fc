# SPDX-License-Identifier: MIT
"""Command line: python3 -m campaign.run [--group NAME ...] [--workers N] [--out FILE.jsonl] [--limit N]"""
from __future__ import annotations

import argparse
import json
import sys
import time
from collections import Counter, defaultdict
from concurrent.futures import ProcessPoolExecutor

from . import grids
from .runner import run_scenario


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="campaign", description=__doc__)
    ap.add_argument("--group", action="append", help="run only these groups (default: all); --list shows them")
    ap.add_argument("--list", action="store_true")
    ap.add_argument("--workers", type=int, default=8)
    ap.add_argument("--out", default="campaign.jsonl")
    ap.add_argument("--limit", type=int, default=0, help="run at most N scenarios per group (for a quick look)")
    ap.add_argument("--quiet", action="store_true")
    ap.add_argument("--strict", action="store_true", help="exit 1 if any anomaly was raised (for CI)")
    args = ap.parse_args(argv)
    table = grids.all_groups()
    if args.list:
        for name, fn in table.items():
            print(f"{name:<22} {len(fn()):>6} scenarios")
        return 0
    names = args.group or list(table)
    scenarios = []
    for n in names:
        g = table[n]()
        scenarios += g[: args.limit] if args.limit else g
    print(f"{len(scenarios)} scenarios, {args.workers} workers", flush=True)
    t0 = time.time()
    results = []
    with ProcessPoolExecutor(max_workers=args.workers) as ex:
        for i, r in enumerate(ex.map(run_scenario, [(s,) for s in scenarios], chunksize=8), 1):
            results.append(r)
            if not args.quiet and i % 500 == 0:
                print(f"  {i}/{len(scenarios)} ({time.time() - t0:.0f} s)", flush=True)
    with open(args.out, "w") as fh:
        for r in results:
            fh.write(json.dumps(r, default=str) + "\n")
    codes = Counter()
    by_group = defaultdict(Counter)
    for r in results:
        for code, _ in r["anomalies"]:
            codes[code] += 1
            by_group[r["scenario"]["group"]][code] += 1
    print(f"done in {time.time() - t0:.0f} s; results in {args.out}")
    print("anomaly codes:", dict(codes) or "none")
    for g, c in sorted(by_group.items()):
        print(f"  {g:<22} {dict(c)}")
    return 1 if codes and args.strict else 0


if __name__ == "__main__":
    sys.exit(main())
