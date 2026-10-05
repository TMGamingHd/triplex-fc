#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""The behavioural-compatibility gate between two releases (ADR-021, TFC-ARCH-006).

    python3 tools/compat/compat_gate.py --golden REF [--factor 1.5] [--tol-deg 0.01]
    python3 tools/compat/compat_gate.py --golden-include DIR      (a core/include directory of the golden release, already checked out)

Builds `tools/compat/tfc_compat.cpp` against the current `core/include` and against the golden release's, flies the fixed set of scenarios through both (three flight functions with their IMU
models over the 6-DOF vehicle, their commands voted by a mid-value), and requires the two releases' voted commands to agree in every frame within the **version tolerance**: `factor` times the
manager's command vote tolerance (default 1.5 x 0.01 degree). A release that fails is not to be fielded against that golden release (the golden computer would be the odd one out, and the manager
would hold and ask, `docs/design/RESYNC.md` section 6). With `--golden REF` the golden include directory is taken from a temporary `git worktree` of REF. A golden release whose interface the harness
cannot build against is reported as incomparable (exit 3), which is itself a finding: the bus and API are supposed to be frozen for it (TFC-ARCH-004).
Exit status: 0 compatible, 1 a command differs by more than the tolerance, 2 usage or build error of the current release, 3 the golden release does not build against the harness.
"""
from __future__ import annotations

import argparse
import math
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
FLAGS = ["-std=c++17", "-O2", "-ffp-contract=off", "-fno-exceptions", "-fno-rtti", "-w"]


def build(include: Path, out: Path) -> tuple[bool, str]:
    cmd = ["g++", *FLAGS, f"-I{include}", f"-I{ROOT / 'sim' / 'vehicle'}", f"-I{ROOT / 'tools' / 'vehicle'}", str(ROOT / "tools" / "compat" / "tfc_compat.cpp"), "-o", str(out)]
    r = subprocess.run(cmd, capture_output=True, text=True)
    return r.returncode == 0, (r.stderr.strip().splitlines() or [""])[0][:200]


def fly(binary: Path) -> dict[str, list[tuple[float, float]]]:
    out = subprocess.run([str(binary)], capture_output=True, text=True, check=True).stdout
    traces: dict[str, list[tuple[float, float]]] = {}
    for line in out.splitlines():
        name, _k, p, y = line.split(",")
        traces.setdefault(name, []).append((float(p), float(y)))
    return traces


def compare(a: dict, b: dict, tol: float) -> tuple[bool, list[str]]:
    lines: list[str] = []
    ok = True
    for name in sorted(a):
        if name not in b or len(a[name]) != len(b[name]):
            lines.append(f"  {name}: the two releases did not fly the same number of frames")
            ok = False
            continue
        worst, at = 0.0, 0
        for k, ((p1, y1), (p2, y2)) in enumerate(zip(a[name], b[name])):
            d = max(abs(p1 - p2), abs(y1 - y2))
            if not math.isfinite(d):
                d = float("inf")
            if d > worst:
                worst, at = d, k
        verdict = "ok" if worst <= tol else "FAILS"
        ok = ok and worst <= tol
        lines.append(f"  {name:<22} largest command difference {worst:.4f} degree at frame {at}  (limit {tol:.4f})  {verdict}")
    return ok, lines


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="compat_gate", description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--golden", metavar="REF", help="a git ref (tag or commit) of the golden release")
    g.add_argument("--golden-include", metavar="DIR", help="the golden release's core/include directory")
    ap.add_argument("--factor", type=float, default=1.5, help="the version tolerance as a multiple of the command vote tolerance (default 1.5, TFC-ARCH-006)")
    ap.add_argument("--tol-deg", type=float, default=0.01, help="the manager's command vote tolerance in degrees (default 0.01)")
    args = ap.parse_args(argv)
    tol = args.factor * args.tol_deg
    with tempfile.TemporaryDirectory(prefix="tfc-compat-") as d:
        tmp = Path(d)
        worktree = None
        try:
            if args.golden:
                worktree = tmp / "golden"
                r = subprocess.run(["git", "-C", str(ROOT), "worktree", "add", "--detach", str(worktree), args.golden], capture_output=True, text=True)
                if r.returncode:
                    print(f"cannot check out {args.golden}: {r.stderr.strip()}", file=sys.stderr)
                    return 2
                golden_inc = worktree / "core" / "include"
            else:
                golden_inc = Path(args.golden_include)
            ok_now, err = build(ROOT / "core" / "include", tmp / "current")
            if not ok_now:
                print(f"the current release does not build against the harness: {err}", file=sys.stderr)
                return 2
            ok_old, err = build(golden_inc, tmp / "golden_bin")
            if not ok_old:
                print(f"the golden release does not build against the harness (its interface differs): {err}", file=sys.stderr)
                return 3
            now, old = fly(tmp / "current"), fly(tmp / "golden_bin")
        finally:
            if worktree is not None:
                subprocess.run(["git", "-C", str(ROOT), "worktree", "remove", "--force", str(worktree)], capture_output=True)
    ok, lines = compare(now, old, tol)
    print(f"commands of the current release against the golden one, version tolerance {tol:.4f} degree ({args.factor} x {args.tol_deg}):")
    print("\n".join(lines))
    print("COMPATIBLE" if ok else "NOT COMPATIBLE: the golden computer would be the odd one out; do not field this release against it")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
