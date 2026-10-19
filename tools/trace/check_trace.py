#!/usr/bin/env python3
"""Requirement-to-test trace: which requirements marked as verified by an automated test (T) can be found from a test.

docs/verification/REQUIREMENTS.md says how each requirement is verified in prose; this tool checks that the claim can be followed by a machine. A requirement is *traced* when its full identifier
(`TFC-FDIR-001`) appears in a test or in the code a test exercises (a comment `// Verifies: TFC-FDIR-001` is the convention), or its short identifier (`FDIR-001`) is in a row of
docs/verification/FAULT_MATRIX.md, which names the test of each fault. The orphans are listed; `--baseline FILE` fails when there are more of them than the baseline's (a gate that may only be lowered),
and `--write-baseline FILE` records the present ones.

    python3 tools/trace/check_trace.py                       # the report
    python3 tools/trace/check_trace.py --baseline tools/trace/baseline.json
"""
import argparse
import glob
import json
import os
import re
import sys

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
SOURCES = [".github/workflows/*.yml", "tests/*.cpp", "sim/tests/*.py", "sim/campaign/*.py", "core/**/*.hpp", "sim/**/*.hpp", "firmware/**/*.cpp", "tools/**/*.py", "console/tfc_console/*.py"]


def requirements(path):
    rows = []
    for line in open(path, encoding="utf-8"):
        m = re.match(r"\| (TFC-([A-Z0-9]+-\d+)) \|.*\| ([^|]*) \| ([^|]*) \|\s*$", line)
        if m:
            rows.append({"id": m.group(1), "short": m.group(2), "how": m.group(3).strip(), "status": m.group(4).strip()})
    return rows


def scan():
    text = ""
    for pattern in SOURCES:
        for p in glob.glob(os.path.join(ROOT, pattern), recursive=True):
            if os.path.basename(p) == "check_trace.py":
                continue
            text += open(p, encoding="utf-8", errors="ignore").read()
    return text


def analyse():
    reqs = requirements(os.path.join(ROOT, "docs/verification/REQUIREMENTS.md"))
    matrix = open(os.path.join(ROOT, "docs/verification/FAULT_MATRIX.md"), encoding="utf-8").read()
    code = scan()
    # a requirement that says it is not built (or is a boundary or out of scope) has no test to find: it is not an orphan
    automated = [r for r in reqs if "T" in r["how"] and not re.match(r"(Not built|Out of v1|Boundary only)", r["status"])]
    orphans = [r["id"] for r in automated if not (r["id"] in code or re.search(r"\b" + re.escape(r["short"]) + r"\b", matrix))]
    return reqs, automated, orphans


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--baseline")
    ap.add_argument("--write-baseline")
    args = ap.parse_args()
    reqs, automated, orphans = analyse()
    print(f"{len(reqs)} requirements; {len(automated)} verified by an automated test (T); {len(automated) - len(orphans)} of those can be traced to a test; {len(orphans)} cannot")
    by = {}
    for o in orphans:
        by.setdefault(o.split("-")[1], []).append(o)
    for k in sorted(by):
        print(f"  {k}: {len(by[k])}  {' '.join(x.split('-', 1)[1] for x in by[k][:8])}{' ...' if len(by[k]) > 8 else ''}")
    if args.write_baseline:
        json.dump({"orphans": orphans}, open(args.write_baseline, "w"), indent=1)
        print("baseline written:", args.write_baseline)
    if args.baseline:
        base = json.load(open(args.baseline))["orphans"]
        new = [o for o in orphans if o not in base]
        if new:
            print("FAIL: requirements marked T with no test that names them (name one with `// Verifies: ID`):", " ".join(new))
            return 1
        print(f"ok: no new orphan ({len(base) - len(orphans)} fewer than the baseline's {len(base)})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
