#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""The numbers the documents quote, from one place.

A project whose claim is "measured numbers only" cannot have its test count in six documents typed by hand. Every number a *current-state* document quotes
(README, STATUS, PROOF, LIMITATIONS, the coding standard, the safety case, the requirements) is written as a marker,

    <!--m:cpp_tests-->883<!--/m-->          the value is replaced by this tool;   <!--m:cpp_tests:raw-->  leaves out the thousands separator

and takes its value from one of two places:

  * COMPUTED here, from the repository (counts of tests, decisions, requirements, fault-matrix rows, mutants, the campaign's own totals). They cannot be stale.
  * RECORDED in docs/metrics.json: results of runs that take minutes to hours (coverage, the mutation run, the firmware sizes), each with the command that produced it, the date
    and the commit. They can be old, so `check` prints their age and `list` shows it; a marker never hides where a number came from.

Historical records (the bodies of the decisions, the trade studies, the bench log) keep the numbers they had on their date and are not marked.

  python3 tools/docs/metrics.py list                 every metric, its value and where it comes from
  python3 tools/docs/metrics.py apply                rewrite every marker in the documents with the current value
  python3 tools/docs/metrics.py check                exit 1 if a marker is stale or names an unknown metric (CI)
  python3 tools/docs/metrics.py record NAME VALUE --command "..."   store the result of a long run, with today's date and the commit
"""
from __future__ import annotations

import argparse
import datetime
import json
import re
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
STORE = ROOT / "docs" / "metrics.json"
MARKER = re.compile(r"<!--m:([a-z0-9_]+)(?::([a-z]+))?-->(.*?)<!--/m-->", re.S)
# The documents whose numbers are current-state claims. The decisions, trade studies and the bench log are dated records and are left alone.
CURRENT_DOCS = [
    "README.md", "CONTRIBUTING.md", "firmware/README.md", "console/README.md", "sim/README.md",
    "docs/README.md", "docs/STATUS.md", "docs/LIMITATIONS.md", "docs/project/PROOF.md",
    "docs/verification/CODING_STANDARD.md", "docs/verification/SAFETY_CASE.md", "docs/verification/REQUIREMENTS.md",
    "docs/verification/FAULT_CAMPAIGN.md", "docs/verification/CODE_REVIEW_CORE.md",
    "docs/design/ARCHITECTURE.md", "docs/design/ACT_LOGIC.md", "docs/design/SUPERVISOR.md",
]


def _read(rel: str) -> str:
    return (ROOT / rel).read_text()


def _count(rx: str, text: str, flags: int = re.M) -> int:
    return len(re.findall(rx, text, flags))


def _python_tests() -> int:
    code = "import unittest;print(unittest.defaultTestLoader.discover('tests', top_level_dir='.').countTestCases())"
    r = subprocess.run([sys.executable, "-c", code], cwd=ROOT / "sim", capture_output=True, text=True, check=True)
    return int(r.stdout.strip().splitlines()[-1])


def _fault_kinds() -> int:
    code = "from tfc_peers import faults;print(len(faults.KINDS))"
    r = subprocess.run([sys.executable, "-c", code], cwd=ROOT / "sim", capture_output=True, text=True, check=True)
    return int(r.stdout.strip().splitlines()[-1])


def _mutants() -> dict[str, int]:
    sys.path.insert(0, str(ROOT / "tools" / "mutation"))
    import mutations  # noqa: PLC0415
    import sim_mutations  # noqa: PLC0415

    unit = len(mutations.MUTATIONS)
    reach = unit - len([n for n in mutations.MUTATIONS if n in mutations.CAMPAIGN_SKIP])
    return {"mutants_unit": unit, "mutants_campaign": reach, "mutants_sim": len(sim_mutations.MUTATIONS)}


def computed() -> dict[str, tuple[object, str]]:
    """name -> (value, how it was found)."""
    out: dict[str, tuple[object, str]] = {}
    tests = "".join(p.read_text() for p in sorted((ROOT / "tests").glob("*.cpp")))
    out["cpp_tests"] = (_count(r"^TFC_TEST\(", tests), "TFC_TEST( in tests/*.cpp")
    out["python_tests"] = (_python_tests(), "unittest discovery in sim/")
    adr = _read("docs/decisions/DECISIONS.md")
    nums = sorted({int(m) for m in re.findall(r"^#{2,3} ADR-(\d+)", adr, re.M)})
    out["adrs"] = (len(nums), "ADR headings in decisions/DECISIONS.md")
    out["adr_latest"] = (f"ADR-{nums[-1]:03d}" if nums else "none", "the highest ADR heading")
    req = _read("docs/verification/REQUIREMENTS.md")
    rows = re.findall(r"^\| (TFC-[A-Z]+-\d+) \| .*? \| ([A-Z, ]+?) \| (.*?) \|$", req, re.M)
    out["requirements"] = (len(rows), "rows of verification/REQUIREMENTS.md")
    out["requirements_test"] = (len([r for r in rows if "T" in [x.strip() for x in r[1].split(",")]]), "requirements verified (in part) by an automated test")
    out["requirements_hw"] = (len([r for r in rows if "M" in [x.strip() for x in r[1].split(",")]]), "requirements with a hardware measurement in their verification")
    out["requirements_rig_pending"] = (len([r for r in rows if re.search(r"rig pending|waits for", r[2], re.I)]), "requirements whose status says rig pending or waits for")
    tr = subprocess.run([sys.executable, str(ROOT / "tools" / "trace" / "check_trace.py"), "--baseline", str(ROOT / "tools" / "trace" / "baseline.json")], capture_output=True, text=True)
    m = re.search(r"(\d+) requirements; (\d+) verified by an automated test \(T\); (\d+) of those can be traced to a test; (\d+) cannot", tr.stdout)
    out["trace_tests"] = (int(m.group(2)) if m else 0, "requirements verified by a test (tools/trace/check_trace.py)")
    out["trace_traced"] = (int(m.group(3)) if m else 0, "of those, named by a test, a CI step or the fault matrix")
    out["trace_orphans"] = (int(m.group(4)) if m else 0, "of those, with nothing that names them (tools/trace/baseline.json, which may only shrink)")
    fm = _read("docs/verification/FAULT_MATRIX.md")
    out["fault_matrix_rows"] = (_count(r"^\| F\d+ \|", fm), "rows of verification/FAULT_MATRIX.md")
    out["fault_kinds"] = (_fault_kinds(), "entries of tfc_peers.faults.KINDS")
    camp = _read("docs/verification/FAULT_CAMPAIGN.md")
    m = re.search(r"^\| \*\*total\*\* \| \*\*([\d,]+)\*\* \| \*\*([\d,]+)\*\* \|", camp, re.M)
    out["campaign_scenarios"] = (int(m.group(1).replace(",", "")) if m else 0, "the total row of verification/FAULT_CAMPAIGN.md (generated by campaign.report)")
    out["campaign_frames"] = (int(m.group(2).replace(",", "")) if m else 0, "the same row")
    out["campaign_frames_million"] = (f"{int(m.group(2).replace(',', '')) / 1.0e6:.1f}" if m else "0", "the same row, in millions")
    for k, v in _mutants().items():
        out[k] = (v, "tools/mutation (len of the table, less the mutants the campaign cannot reach for mutants_campaign)")
    sys.path.insert(0, str(ROOT / "tools" / "mutation"))
    import check_tables  # noqa: PLC0415

    rep = check_tables.report()
    out["headers_without_mutants"] = (len([r for r in rep if r[2] == 0 and not r[0].startswith("sim/")]), "headers of core/ and supervisor/ that no mutant targets")
    out["sim_headers_without_mutants"] = (len([r for r in rep if r[2] == 0 and r[0].startswith("sim/")]), "headers of sim/vehicle that no mutant targets")
    core = sorted((ROOT / "core" / "include" / "tfc").glob("*.hpp"))
    sup = sorted((ROOT / "supervisor" / "include" / "sup").glob("*.hpp"))
    out["core_headers"] = (len(core) + len(sup), "headers under core/include and supervisor/include")
    out["core_lines"] = (sum(len(p.read_text().splitlines()) for p in core + sup), "lines of those headers")
    out["fmeca_rows"] = (_count(r"^\| FM-[A-Z]+\d+\*? \|", _read("docs/verification/FMECA.md")) if (ROOT / "docs/verification/FMECA.md").exists() else 0, "rows of verification/FMECA.md")
    return out


def recorded() -> dict[str, dict]:
    return json.loads(STORE.read_text()) if STORE.exists() else {}


def values() -> dict[str, object]:
    v = {k: val for k, (val, _how) in computed().items()}
    for k, rec in recorded().items():
        v[k] = rec["value"]
    return v


def fmt(value: object, style: str | None) -> str:
    if isinstance(value, bool):
        return str(value)
    if isinstance(value, int):
        return str(value) if style == "raw" else f"{value:,}"
    if isinstance(value, float):
        return f"{value:.1f}"
    return str(value)


def render(text: str, vals: dict[str, object], problems: list[str], where: str) -> str:
    def sub(m: re.Match) -> str:
        name, style = m.group(1), m.group(2)
        if name not in vals:
            problems.append(f"{where}: unknown metric '{name}'")
            return m.group(0)
        return f"<!--m:{name}{':' + style if style else ''}-->{fmt(vals[name], style)}<!--/m-->"

    return MARKER.sub(sub, text)


def docs() -> list[Path]:
    return [ROOT / d for d in CURRENT_DOCS if (ROOT / d).exists()]


def cmd_list() -> int:
    comp = computed()
    rec = recorded()
    print(f"{'metric':<28} {'value':>12}  source")
    for k, (v, how) in comp.items():
        print(f"{k:<28} {fmt(v, None):>12}  computed: {how}")
    today = datetime.date.today()
    for k, r in rec.items():
        age = (today - datetime.date.fromisoformat(r["date"])).days
        print(f"{k:<28} {fmt(r['value'], None):>12}  recorded {r['date']} ({age} d ago) at {r['commit']}: {r['command']}")
    return 0


def cmd_apply(check: bool) -> int:
    vals = values()
    problems: list[str] = []
    stale: list[str] = []
    n = 0
    for p in docs():
        text = p.read_text()
        n += len(MARKER.findall(text))
        new = render(text, vals, problems, str(p.relative_to(ROOT)))
        if new != text:
            stale.append(str(p.relative_to(ROOT)))
            if not check:
                p.write_text(new)
    for line in problems:
        print("error:", line)
    if check:
        for f in stale:
            print(f"stale: {f} quotes a number that is not the current one (python3 tools/docs/metrics.py apply)")
        today = datetime.date.today()
        for k, r in recorded().items():
            age = (today - datetime.date.fromisoformat(r["date"])).days
            if age > 30:
                print(f"note: the recorded metric {k} is {age} days old ({r['command']})")
        if problems or stale:
            return 1
        print(f"ok: {n} markers in {len(docs())} documents are current")
        return 0
    print(f"{n} markers in {len(docs())} documents; {len(stale)} rewritten" + (f": {', '.join(stale)}" if stale else ""))
    return 1 if problems else 0


def cmd_record(name: str, value: str, command: str) -> int:
    try:
        val: object = int(value)
    except ValueError:
        try:
            val = float(value)
        except ValueError:
            val = value
    commit = subprocess.run(["git", "rev-parse", "--short", "HEAD"], cwd=ROOT, capture_output=True, text=True).stdout.strip() or "unknown"
    store = recorded()
    store[name] = {"value": val, "date": datetime.date.today().isoformat(), "commit": commit, "command": command}
    STORE.write_text(json.dumps(store, indent=2, sort_keys=True) + "\n")
    print(f"recorded {name} = {val} ({commit})")
    return 0


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("list")
    sub.add_parser("apply")
    sub.add_parser("check")
    r = sub.add_parser("record")
    r.add_argument("name")
    r.add_argument("value")
    r.add_argument("--command", required=True)
    a = ap.parse_args(argv)
    if a.cmd == "list":
        return cmd_list()
    if a.cmd == "apply":
        return cmd_apply(False)
    if a.cmd == "check":
        return cmd_apply(True)
    return cmd_record(a.name, a.value, a.command)


if __name__ == "__main__":
    sys.exit(main())
