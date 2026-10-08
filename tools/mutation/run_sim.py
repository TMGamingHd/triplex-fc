# SPDX-License-Identifier: MIT
"""Mutation check of the vehicle simulator's tests: build the tests that include a simulator header against a mutated copy of sim/vehicle and confirm at
least one test fails. Usage: python3 tools/mutation/run_sim.py [--jobs N] [--equivalents] [NAME ...]   (exit 1 if a mutant survives).

--equivalents also runs the mutants that are documented as equivalent (changes that cannot alter behaviour): they are expected to survive, and the run says
so; one that is killed means the test suite is stronger than the note says and the note should go.
"""
from __future__ import annotations

import argparse
import os
import re
import subprocess
import sys
import tempfile
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from sim_mutations import EQUIVALENT, MUTATIONS, ROOT, build_vehicle  # noqa: E402

# no sanitizers: a mutant is killed by a failing check, which they do not help with, and the long example flights of the file tests run several times slower under them
FLAGS = ["-std=c++17", "-O1", "-fno-exceptions", "-fno-rtti", "-w"]
SIM_HEADERS = sorted(p.name for p in (ROOT / "sim" / "vehicle").glob("*.hpp"))


def sim_tests() -> list[Path]:
    """The test files that include a simulator header, plus the harness's main()."""
    pattern = re.compile(r'#include\s+"(' + "|".join(re.escape(h) for h in SIM_HEADERS) + r')"')
    tests = [p for p in sorted((ROOT / "tests").glob("*.cpp")) if p.name != "test_main.cpp" and pattern.search(p.read_text())]
    return [ROOT / "tests" / "test_main.cpp", *tests]


def one(name: str) -> tuple[str, str, str]:
    with tempfile.TemporaryDirectory(prefix=f"mutsim-{name}-", dir=os.environ.get("TMPDIR", "/tmp")) as d:
        tmp = Path(d)
        try:
            inc = build_vehicle(name, tmp)
        except SystemExit as e:
            return name, "BAD-MUTANT", str(e)
        exe = tmp / "tests_mut"
        r = subprocess.run(["g++", *FLAGS, f"-I{ROOT / 'core' / 'include'}", f"-I{inc}", f"-I{ROOT / 'tests'}", f"-I{ROOT / 'tools' / 'vehicle'}",
                            f"-I{ROOT / 'firmware' / 'app' / 'src'}", f"-I{ROOT / 'supervisor' / 'include'}", *map(str, sim_tests()), "-o", str(exe)],
                           capture_output=True, text=True)
        if r.returncode:
            return name, "BUILD-FAILED", (r.stderr.strip().splitlines() or [""])[0][:120]
        try:
            run = subprocess.run([str(exe)], capture_output=True, text=True, timeout=900)
        except subprocess.TimeoutExpired:  # a mutant that makes the code loop for ever is caught: the suite does not finish
            return name, "killed", "timed out (an endless loop)"
        out = run.stdout + run.stderr
        failing = [ln.split("]", 1)[1].strip() for ln in out.splitlines() if ln.startswith("[FAIL]")]
        if run.returncode == 0:
            return name, "SURVIVED", ""
        return name, "killed", (failing[0] if failing else "sanitizer or crash")[:110]


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="run_sim", description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("names", nargs="*")
    ap.add_argument("--jobs", type=int, default=max(1, (os.cpu_count() or 2) // 2))
    ap.add_argument("--equivalents", action="store_true", help="also run the documented equivalent mutants (they are expected to survive)")
    args = ap.parse_args(argv)
    names = args.names or list(MUTATIONS)
    equivalents = list(EQUIVALENT) if args.equivalents and not args.names else []
    print(f"tests built against each mutant: {', '.join(p.name for p in sim_tests())}", flush=True)
    survived = []
    with ThreadPoolExecutor(max_workers=args.jobs) as ex:
        for name, verdict, info in ex.map(one, names + equivalents):
            if name in equivalents:
                note = "as documented" if verdict == "SURVIVED" else "KILLED: the note says it cannot change behaviour; check it"
                print(f"{name:<34} {verdict:<13} (equivalent, {note}) {info}", flush=True)
                continue
            print(f"{name:<34} {verdict:<13} {info}", flush=True)
            if verdict != "killed":
                survived.append(name)
    print(f"\n{len(names) - len(survived)}/{len(names)} mutants killed by the C++ tests")
    if survived:
        print("NOT killed:", ", ".join(survived))
    return 1 if survived else 0


if __name__ == "__main__":
    sys.exit(main())
