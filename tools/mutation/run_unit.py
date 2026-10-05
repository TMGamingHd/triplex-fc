# SPDX-License-Identifier: MIT
"""Mutation check of the C++ test suite: build every test against a mutated copy of core/include and confirm at least
one test fails. Usage: python3 tools/mutation/run_unit.py [--jobs N] [NAME ...]   (exit 1 if a mutant survives)."""
from __future__ import annotations

import argparse
import os
import subprocess
import sys
import tempfile
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from mutations import MUTATIONS, ROOT, build_include  # noqa: E402

TESTS = sorted((ROOT / "tests").glob("*.cpp"))
FLAGS = ["-std=c++17", "-O1", "-g", "-fsanitize=address,undefined", "-fno-exceptions", "-fno-rtti", "-w"]


def one(name: str) -> tuple[str, str, str]:
    with tempfile.TemporaryDirectory(prefix=f"mutu-{name}-", dir=os.environ.get("TMPDIR", "/tmp")) as d:
        tmp = Path(d)
        inc = build_include(name, tmp)
        exe = tmp / "tests_mut"
        r = subprocess.run(["g++", *FLAGS, f"-I{inc}", f"-I{tmp / 'include_sup'}", f"-I{ROOT / 'tests'}", f"-I{ROOT / 'tools' / 'vehicle'}", f"-I{ROOT / 'sim' / 'vehicle'}", f"-I{ROOT / 'firmware' / 'app' / 'src'}", *map(str, TESTS), "-o", str(exe)], capture_output=True, text=True)
        if r.returncode:
            return name, "BUILD-FAILED", r.stderr.strip().splitlines()[0][:120] if r.stderr.strip() else ""
        try:
            run = subprocess.run([str(exe)], capture_output=True, text=True, timeout=600)
        except subprocess.TimeoutExpired:  # a mutant that makes the code loop for ever is caught: the suite does not finish
            return name, "killed", "timed out (an endless loop)"
        out = run.stdout + run.stderr
        failing = [ln.split("]", 1)[1].strip() for ln in out.splitlines() if ln.startswith("[FAIL]")]
        if run.returncode == 0:
            return name, "SURVIVED", ""
        return name, "killed", (failing[0] if failing else "sanitizer or crash")[:110]


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="run_unit", description=__doc__)
    ap.add_argument("names", nargs="*")
    ap.add_argument("--jobs", type=int, default=max(1, (os.cpu_count() or 2) // 2))
    args = ap.parse_args(argv)
    names = args.names or list(MUTATIONS)
    survived = []
    with ThreadPoolExecutor(max_workers=args.jobs) as ex:
        for name, verdict, info in ex.map(one, names):
            print(f"{name:<34} {verdict:<13} {info}", flush=True)
            if verdict != "killed":
                survived.append(name)
    print(f"\n{len(names) - len(survived)}/{len(names)} mutants killed by the C++ tests")
    if survived:
        print("NOT killed:", ", ".join(survived))
    return 1 if survived else 0


if __name__ == "__main__":
    sys.exit(main())
