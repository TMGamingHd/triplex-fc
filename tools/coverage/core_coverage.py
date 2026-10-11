# SPDX-License-Identifier: MIT
"""Structural coverage of the flight core (core/include) and the supervisor logic (supervisor/include), and of the vehicle simulator (sim/vehicle), by the C++ unit,
fuzz and recovery tests, with gcov.

    python3 tools/coverage/core_coverage.py [--min-line PCT] [--min-branch PCT] [--sim-min-line PCT] [--sim-min-branch PCT] [--list] [--tests NAME...]

The two groups are measured in one build and gated apart: --min-line/--min-branch for the flight code, --sim-min-line/--sim-min-branch for the simulator (which is
host-only and holds a model, so its gate is lower: a numerical model has guards that no test can reach, such as a non-finite state).

Builds the tests with --coverage in a temporary directory, runs them, merges the per-translation-unit gcov data (a
header's inline functions are instrumented once per test file that includes it) and prints line and branch coverage per
header. Branch coverage counts the compiler's branches (including those it inserts for noexcept/overflow/short-circuit
evaluation), so 100% is not expected; --list prints the uncovered lines so each can be justified or tested.
"""
from __future__ import annotations

import argparse
import gzip
import json
import subprocess
import sys
import tempfile
from collections import defaultdict
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="core_coverage", description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--min-line", type=float, default=0.0)
    ap.add_argument("--min-branch", type=float, default=0.0)
    ap.add_argument("--sim-min-line", type=float, default=0.0)
    ap.add_argument("--sim-min-branch", type=float, default=0.0)
    ap.add_argument("--list", action="store_true", help="print the uncovered lines and branches")
    ap.add_argument("--tests", nargs="*", default=None, metavar="NAME", help="build and run only these test files (test_NAME.cpp, plus test_main.cpp): a quick look while writing tests; the gates need the whole suite")
    args = ap.parse_args(argv)
    with tempfile.TemporaryDirectory(prefix="tfc-cov-") as d:
        tmp = Path(d)
        sources = sorted(str(p) for p in (ROOT / "tests").glob("*.cpp"))
        if args.tests:
            keep = {f"test_{n}.cpp" for n in args.tests} | {"test_main.cpp"}
            sources = [p for p in sources if Path(p).name in keep]
        subprocess.run(["g++", "-std=c++17", "-O0", "-g", "--coverage", "-fprofile-update=atomic", "-pthread", "-fno-exceptions", "-fno-rtti", f"-I{ROOT / 'core/include'}",
                        f"-I{ROOT / 'tests'}", f"-I{ROOT / 'tools' / 'vehicle'}", f"-I{ROOT / 'sim' / 'vehicle'}", f"-I{ROOT / 'firmware' / 'app' / 'src'}", f"-I{ROOT / 'supervisor/include'}", *sources, "-o", str(tmp / "tests_cov")], check=True, cwd=tmp)
        r = subprocess.run([str(tmp / "tests_cov")], capture_output=True, text=True, cwd=tmp)
        if r.returncode != 0:
            print(r.stdout[-2000:])
            print("tests failed: coverage of a failing run is meaningless", file=sys.stderr)
            return 1
        gcdas = sorted(tmp.glob("*.gcda"))
        subprocess.run(["gcov", "--json-format", "--branch-probabilities", "--branch-counts", *map(str, gcdas)], check=True, cwd=tmp,
                       capture_output=True)
        lines: dict[str, dict[int, int]] = defaultdict(lambda: defaultdict(int))
        branches: dict[str, dict[tuple[int, int], int]] = defaultdict(lambda: defaultdict(int))
        for gz in sorted(tmp.glob("*.gcov.json.gz")):
            data = json.loads(gzip.open(gz).read())
            for f in data["files"]:
                path = str(Path(f["file"]).resolve()) if Path(f["file"]).is_absolute() else str((tmp / f["file"]).resolve())
                if "core/include" not in path and "supervisor/include" not in path and "sim/vehicle" not in path:
                    continue
                for ln in f["lines"]:
                    n = ln["line_number"]
                    lines[path][n] += ln["count"]
                    for i, b in enumerate(ln.get("branches", [])):
                        branches[path][(n, i)] += b["count"]
    ok = True
    for title, keys, min_line, min_branch in (("flight code (core/include, supervisor/include)", ("core/include", "supervisor/include"), args.min_line, args.min_branch),
                                              ("vehicle simulator (sim/vehicle)", ("sim/vehicle",), args.sim_min_line, args.sim_min_branch)):
        print(f"\n{title}")
        print(f"{'header':<24}{'lines':>14}{'branches':>18}")
        tl = tc = tb = tbc = 0
        for path in sorted(lines):
            if not any(k in path for k in keys):
                continue
            total = len(lines[path])
            covered = sum(1 for c in lines[path].values() if c > 0)
            nb = len(branches[path])
            cb = sum(1 for c in branches[path].values() if c > 0)
            print(f"{Path(path).name:<24}{covered:>6}/{total:<5}{100 * covered / total:>6.1f}%{cb:>8}/{nb:<5}{100 * cb / max(nb, 1):>6.1f}%")
            tl, tc, tb, tbc = tl + total, tc + covered, tb + nb, tbc + cb
            if args.list:
                miss = sorted(n for n, c in lines[path].items() if c == 0)
                if miss:
                    print("    uncovered lines:", ", ".join(map(str, miss)))
                mb = sorted({n for (n, _i), c in branches[path].items() if c == 0})
                if mb:
                    print("    lines with an untaken branch:", ", ".join(map(str, mb)))
        lp, bp = 100 * tc / max(tl, 1), 100 * tbc / max(tb, 1)
        print(f"{'TOTAL':<24}{tc:>6}/{tl:<5}{lp:>6.1f}%{tbc:>8}/{tb:<5}{bp:>6.1f}%")
        if lp < min_line or bp < min_branch:
            print(f"{title}: below the gate (line >= {min_line}%, branch >= {min_branch}%)", file=sys.stderr)
            ok = False
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
