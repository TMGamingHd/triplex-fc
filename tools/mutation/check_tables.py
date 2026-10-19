#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Are the mutant tables still mutants? Every deliberate bug is a piece of text to find and the text to put in its place; a refactor that moves the code turns the mutant into
nothing, and the run then reports it as "not killed" (the simulator's) or crashes on it (the core's). Found the hard way: the weekly workflow of October 2026 reported 17
survivors that were 17 stale patterns, and a behaviour with no live mutant is a behaviour that mutation testing has silently stopped checking.

  python3 tools/mutation/check_tables.py          exit 1 if any pattern does not occur exactly once in its file (CI, in a second)
  python3 tools/mutation/check_tables.py --report which headers of core/, supervisor/ and sim/vehicle have how many mutants
"""
from __future__ import annotations

import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import mutations  # noqa: E402
import sim_mutations  # noqa: E402

ROOT = HERE.parents[1]


def core_path(f: str) -> Path:
    return ROOT / "supervisor" / "include" / f if f.startswith("sup/") else ROOT / "core" / "include" / "tfc" / f


def stale() -> list[str]:
    out: list[str] = []
    for n, (f, old, _new) in mutations.MUTATIONS.items():
        c = core_path(f).read_text().count(old)
        if c != 1:
            out.append(f"core mutant {n}: the text occurs {c} times in {f} (it must occur once; the code moved, update tools/mutation/mutations.py)")
    for n, (f, old, _new) in sim_mutations.MUTATIONS.items():
        c = (ROOT / "sim" / "vehicle" / f).read_text().count(old)
        if c != 1:
            out.append(f"simulator mutant {n}: the text occurs {c} times in {f} (it must occur once; the code moved, update tools/mutation/sim_mutations.py)")
    for n, (f, old, _new, _why) in sim_mutations.EQUIVALENT.items():
        if (ROOT / "sim" / "vehicle" / f).read_text().count(old) != 1:
            out.append(f"equivalent simulator mutant {n}: the text no longer occurs once in {f}")
    return out


def report() -> list[tuple[str, int, int]]:
    """(header, lines, mutants) for every header the mutants could target."""
    count: dict[str, int] = {}
    for _n, (f, _o, _w) in mutations.MUTATIONS.items():
        count[f] = count.get(f, 0) + 1
    for _n, (f, _o, _w) in sim_mutations.MUTATIONS.items():
        count["sim/" + f] = count.get("sim/" + f, 0) + 1
    rows = []
    for p in sorted((ROOT / "core" / "include" / "tfc").glob("*.hpp")):
        rows.append((p.name, len(p.read_text().splitlines()), count.get(p.name, 0)))
    for p in sorted((ROOT / "supervisor" / "include" / "sup").glob("*.hpp")):
        rows.append(("sup/" + p.name, len(p.read_text().splitlines()), count.get("sup/" + p.name, 0)))
    for p in sorted((ROOT / "sim" / "vehicle").glob("*.hpp")):
        rows.append(("sim/" + p.name, len(p.read_text().splitlines()), count.get("sim/" + p.name, 0)))
    return rows


def main(argv: list[str]) -> int:
    if "--report" in argv:
        print(f"{'header':<30} {'lines':>6} {'mutants':>8}")
        for h, n, k in report():
            print(f"{h:<30} {n:>6} {k:>8}{'   <-- none' if k == 0 else ''}")
        return 0
    bad = stale()
    for line in bad:
        print(line)
    if bad:
        return 1
    print(f"ok: {len(mutations.MUTATIONS)} core and {len(sim_mutations.MUTATIONS)} simulator mutants, every one matches exactly once")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
