# SPDX-License-Identifier: MIT
"""Mutation check of the campaign itself: inject known bugs into a COPY of core/, rebuild tfc_replay, run a campaign
and confirm it flags them. A campaign that cannot fail proves nothing.  Usage: python3 -m campaign.mutate [NAME ...]"""
from __future__ import annotations

import argparse
import json
import os
import shutil
import subprocess
import sys
import tempfile
from collections import Counter
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools" / "mutation"))
from mutations import CAMPAIGN_SKIP, MUTATIONS, build_include  # noqa: E402  (the mutant table is shared with the C++ unit-test check)


def _compile(inc: Path, exe: Path) -> Path:
    subprocess.run(["g++", "-std=c++17", "-O1", f"-I{inc}", str(ROOT / "tools/replay/replay.cpp"), "-o", str(exe)],
                   check=True, capture_output=True)
    return exe


def build(name: str, tmp: Path) -> Path:
    return _compile(build_include(name, tmp), tmp / "replay_mut")


def build_unmutated(tmp: Path) -> Path:
    """The baseline binary: the current core compiled exactly like a mutant, so a stale or differently built default
    tfc_replay can never be mistaken for the real code's findings."""
    inc = tmp / "include"
    shutil.copytree(ROOT / "core/include", inc)
    return _compile(inc, tmp / "replay_clean")


def _anomalies(path: Path) -> Counter:
    """(scenario key, anomaly code) -> how many times it was raised."""
    c: Counter = Counter()
    for line in path.read_text().splitlines():
        r = json.loads(line)
        for code, _ in r["anomalies"]:
            c[(r["key"], code)] += 1
    return c


# Groups in the order a mutant is most likely to be caught (cheap and sharp first). A mutant is "caught" at the first group
# that raises an anomaly the unmutated core does not; a mutant that survives them all has run the whole campaign.
ORDER = ["total_loss", "commands_strikes", "commands_misc", "commands_transient", "commands_persistent", "dropout", "bias_gyro", "digest",
         "cmd_offset", "stuck", "phase_sweep", "corrupt", "late", "reboot", "saturate", "babble", "seqgap", "startup_edges", "contexts",
         "drift", "spike", "bias_accel", "correlated", "cascades", "corrupt_periodic", "intermittent", "new_intermittent", "pairs", "new_pairs",
         "long_run"]


def _campaign(exe: str | None, out: Path, groups: list[str]) -> None:
    env = dict(os.environ)
    if exe:
        env["TFC_REPLAY_BIN"] = exe
    args = [sys.executable, "-m", "campaign.run", "--workers", str(os.cpu_count() or 4), "--quiet", "--out", str(out)]
    for g in groups:
        args += ["--group", g]
    subprocess.run(args, cwd=ROOT / "sim", env=env, check=True, capture_output=True)


def _all_groups() -> list[str]:
    sys.path.insert(0, str(ROOT / "sim"))
    from campaign import grids  # noqa: PLC0415
    names = list(grids.all_groups())
    return [g for g in ORDER if g in names] + [g for g in names if g not in ORDER]


def _caught(exe: str, tmp: Path, groups: list[str], baseline: Counter, full: bool) -> Counter:
    """Anomalies beyond the baseline; stops at the first group that has any unless `full`. The key "@group" marks where."""
    codes: Counter = Counter()
    for g in groups:
        out = tmp / f"{g}.jsonl"
        _campaign(exe, out, [g])
        for (_key, code), n in (_anomalies(out) - baseline).items():
            codes[code] += n
        if codes and not full:
            codes[f"@{g}"] = 1
            break
    return codes


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="campaign.mutate", description=__doc__)
    ap.add_argument("names", nargs="*", help="mutations to run (default: all)")
    ap.add_argument("--group", action="append", default=[], help="campaign groups to run (default: all)")
    ap.add_argument("--full", action="store_true", help="run every group for every mutant instead of stopping at the first catch")
    args = ap.parse_args(argv)
    names = args.names or [n for n in MUTATIONS if n not in CAMPAIGN_SKIP]
    base = os.environ.get("TMPDIR", "/tmp")
    summary = {}
    groups = args.group or _all_groups()
    with tempfile.TemporaryDirectory(prefix="mut-baseline-", dir=base) as d:  # the unmutated core's own findings are not credit
        clean = Path(d) / "clean.jsonl"
        _campaign(str(build_unmutated(Path(d))), clean, groups)
        baseline = _anomalies(clean)
    print(f"baseline (unmutated core): {sum(baseline.values())} anomalies, subtracted from every mutation", flush=True)
    for name in names:
        with tempfile.TemporaryDirectory(prefix=f"mut-{name}-", dir=base) as d:
            tmp = Path(d)
            exe = build(name, tmp)
            codes = _caught(str(exe), tmp, groups, baseline, args.full)
            where = next((c[1:] for c in codes if c.startswith("@")), "")
            real = {c: n for c, n in codes.items() if not c.startswith("@")}
            summary[name] = real
            print(f"{name:<34} {'CAUGHT ' if real else 'MISSED!'} {('in ' + where + ' ') if where else ''}{real}", flush=True)
    missed = [n for n, c in summary.items() if not c]
    print("\nMISSED mutations:", missed or "none")
    return 1 if missed else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
