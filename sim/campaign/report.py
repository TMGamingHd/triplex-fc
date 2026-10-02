# SPDX-License-Identifier: MIT
"""Turn campaign results (JSON lines from `python3 -m campaign.run`) into the Markdown tables of docs/FAULT_CAMPAIGN.md.
Usage: python3 -m campaign.report RESULTS.jsonl [GROUP ...] > tables.md      (tables, and a response curve per GROUP)
       python3 -m campaign.report RESULTS.jsonl --update-doc docs/FAULT_CAMPAIGN.md   (rewrite the marked blocks in place)
The numbers in the docs come from here, not from hand."""
from __future__ import annotations

import json
import re
import statistics
import sys
from collections import Counter, defaultdict

REASONS = {1: "missing", 2: "crc", 4: "seq", 8: "vote", 16: "digest", 32: "stuck", 64: "intermittent"}
# the parameter that drives each group's response curve (a key of the scenario tag)
CURVE_KEY = {
    "bias_gyro": "rel", "bias_accel": "rel", "drift": "rel_rate", "corrupt": "p", "cmd_offset": "mag", "late": "us", "reboot": "down",
    "dropout": "dur", "stuck": "dur", "digest": "xor", "scale": "factor", "noise": "mult", "clip": "limit", "repeat": "n",
    "bitflip": "bit", "stuckbit": "bit", "early": "us", "jitter": "us", "clockdrift": "d", "replay": "age", "duplicate": "gap",
    "partial": "mask", "oscillate": "hz", "invert": "axis", "swap": "other", "saturate": "dur",
}
SINGLE = ["dropout", "stuck", "bias_gyro", "bias_accel", "drift", "spike", "saturate", "corrupt", "cmd_offset", "digest", "babble",
          "seqgap", "reboot", "late", "scale", "noise", "invert", "swap", "zero", "clip", "oscillate", "repeat", "bitflip", "stuckbit",
          "cmdstuck", "cmdinvert", "partial", "duplicate", "replay", "seqstuck", "early", "jitter", "clockdrift"]
MULTI = ["pairs", "new_pairs", "correlated", "cascades", "contexts", "startup_edges", "intermittent", "new_intermittent", "corrupt_periodic",
         "commands_transient", "commands_persistent", "commands_strikes", "commands_misc", "recovery_edges", "total_loss", "long_run"]


def load(path: str) -> list[dict]:
    with open(path) as fh:
        return [json.loads(line) for line in fh]


def start_of(r: dict) -> int | None:
    m = re.search(r"start=(\d+)", r["scenario"]["faults"][0]) if r["scenario"]["faults"] else None
    return int(m.group(1)) if m else None


def target(r: dict) -> str | None:
    return r["scenario"]["faults"][0].split(":")[0] if r["scenario"]["faults"] else None


def pct(xs: list[float], p: float) -> float:
    xs = sorted(xs)
    return xs[min(len(xs) - 1, int(p * (len(xs) - 1) + 0.5))]


def reasons(bits: int) -> str:
    return "+".join(n for b, n in REASONS.items() if bits & b) or "-"


def lat_text(xs: list[int]) -> str:
    if not xs:
        return "-"
    return f"{min(xs)} / {int(statistics.median(xs))} / {int(pct(xs, 0.95))} / {max(xs)}"


def single_rows(results: list[dict]) -> list[str]:
    by = defaultdict(list)
    for r in results:
        by[r["scenario"]["group"]].append(r)
    out = ["| Kind | Scenarios | Detect-class: isolated | Latency, frames after start (min / median / p95 / max) | Ignore-class: falsely isolated | Gray: isolated | Caught by | Duplex: isolated / Safe / neither |",
           "|---|---|---|---|---|---|---|---|"]
    for g in SINGLE:
        rs = by.get(g, [])
        if not rs:
            continue
        tri = [r for r in rs if r["scenario"]["context"] == "triplex" and not r["scenario"]["rest"]]
        det = [r for r in tri if r["scenario"]["expect"] == "detect"]
        ign = [r for r in tri if r["scenario"]["expect"] == "ignore"]
        gray = [r for r in tri if r["scenario"]["expect"] == "gray"]

        def isolated(r):
            t = target(r)
            return t is not None and str("ABC".index(t)) in r["latch"]

        lat = [r["latch"][str("ABC".index(target(r)))] - (start_of(r) or 0) for r in det if isolated(r) and start_of(r) is not None]
        why = Counter()
        for r in tri:
            if isolated(r):
                why[reasons(r["reason_at_latch"].get(str("ABC".index(target(r))), 0))] += 1
        top = ", ".join(f"{k}" for k, _ in why.most_common(2)) or "-"
        dpx = [r for r in rs if r["scenario"]["context"].startswith("duplex") and not r["scenario"]["rest"]]
        d_iso = sum(1 for r in dpx if isolated(r))
        d_safe = sum(1 for r in dpx if not isolated(r) and r["metrics"].get("safe_ever"))
        d_none = len(dpx) - d_iso - d_safe
        out.append(f"| `{g}` | {len(rs)} | {sum(isolated(r) for r in det)}/{len(det)} | {lat_text(lat)} | {sum(isolated(r) for r in ign)}/{len(ign)} | "
                   f"{sum(isolated(r) for r in gray)}/{len(gray)} | {top} | {f'{d_iso} / {d_safe} / {d_none}' if dpx else '-'} |")
    return out


def curve(results: list[dict], group: str, key: str | None = None) -> list[str]:
    key = key or CURVE_KEY[group]
    rows = defaultdict(list)
    for r in results:
        sc = r["scenario"]
        if sc["group"] != group or sc["context"] != "triplex" or sc["rest"] or key not in sc["tag"]:
            continue
        rows[sc["tag"][key]].append(r)
    out = [f"| `{key}` | Scenarios | Isolated | Latency, frames (min / median / max) | Caught by |", "|---|---|---|---|---|"]

    def sortkey(v):
        return (0, v) if isinstance(v, (int, float)) else (1, str(v))

    for v in sorted(rows, key=sortkey):
        rs = rows[v]
        iso = [r for r in rs if target(r) is not None and str("ABC".index(target(r))) in r["latch"]]
        lat = [r["latch"][str("ABC".index(target(r)))] - (start_of(r) or 0) for r in iso if start_of(r) is not None]
        why = Counter(reasons(r["reason_at_latch"].get(str("ABC".index(target(r))), 0)) for r in iso)
        lt = f"{min(lat)} / {int(statistics.median(lat))} / {max(lat)}" if lat else "-"
        out.append(f"| {v} | {len(rs)} | {len(iso)} | {lt} | {', '.join(k for k, _ in why.most_common(2)) or '-'} |")
    return out


def multi_rows(results: list[dict]) -> list[str]:
    by = defaultdict(list)
    for r in results:
        by[r["scenario"]["group"]].append(r)
    out = ["| Group | Scenarios | Frames simulated | Ended in triplex / duplex / simplex / safe | Safe requested at some point | Nodes disabled (total) | Anomalies |",
           "|---|---|---|---|---|---|---|"]
    for g in MULTI:
        rs = by.get(g, [])
        if not rs:
            continue
        modes = Counter(r["final_mode"] for r in rs)
        an = Counter(code for r in rs for code, _ in r["anomalies"])
        out.append(f"| `{g}` | {len(rs)} | {sum(r['metrics'].get('frames', 0) for r in rs):,} | "
                   f"{modes.get('triplex', 0)} / {modes.get('duplex', 0)} / {modes.get('simplex', 0)} / {modes.get('safe', 0)} | "
                   f"{sum(1 for r in rs if r['metrics'].get('safe_ever'))} | {sum(r['counters'].get('nodes_disabled', 0) for r in rs)} | "
                   f"{dict(an) if an else 'none'} |")
    return out


def properties(results: list[dict]) -> list[str]:
    codes = Counter(code for r in results for code, _ in r["anomalies"])
    frames = sum(r["metrics"].get("frames", 0) for r in results)
    props = [("M1", "mode is Safe or follows the healthy count"), ("M2", "only legal node-state transitions"), ("M3", "a latched node never votes"),
             ("M4", "Safe request is sticky until cleared"), ("M5", "every output finite and bounded"), ("M6", "the same scenario twice gives identical decisions (a 1-in-25 sample)"),
             ("M7", "a held output repeats exactly"), ("M8", "a probation never starts while another runs (except total loss)"),
             ("M9", "while Safe is requested every output channel is held"), ("M10", "every node-state change is reported by its event, and every event is a state change"), ("S1", "no healthy node is ever isolated (single fault)"),
             ("S2", "output within one vote tolerance of the truth (single fault, not held)"), ("S3", "no Safe request after a single fault in triplex"),
             ("S4", "no readmission while a detect-class fault is still active"), ("X_RUN", "every scenario runs to completion")]
    out = [f"{len(results):,} scenarios, {frames:,} frames of manager decisions checked.", "",
           "| Property | Statement | Violations |", "|---|---|---|"]
    for code, text in props:
        out.append(f"| {code} | {text} | {codes.get(code, 0)} |")
    adv = {c: n for c, n in codes.items() if c.startswith("E_")}
    out += ["", f"Advisory expectation codes (E_*) raised: {adv if adv else 'none'}."]
    return out


def group_rows(results: list[dict]) -> list[str]:
    def name(r: dict) -> str:
        g = r["scenario"]["group"]
        return "phase_* (every kind, 40 start instants, Triplex and Duplex)" if g.startswith("phase_") else g

    by = Counter(name(r) for r in results)
    frames = defaultdict(int)
    for r in results:
        frames[name(r)] += r["metrics"].get("frames", 0)
    out = ["| Group | Scenarios | Frames |", "|---|---|---|"]
    for g in sorted(by):
        out.append(f"| `{g}` | {by[g]:,} | {frames[g]:,} |" if not g.startswith("phase_*") else f"| `phase_*` ({g.split('(')[1][:-1]}) | {by[g]:,} | {frames[g]:,} |")
    out.append(f"| **total** | **{sum(by.values()):,}** | **{sum(frames.values()):,}** |")
    return out


def blocks(results: list[dict]) -> dict[str, str]:
    """Named Markdown blocks; a doc marks where each goes with <!-- campaign:NAME:begin --> ... <!-- campaign:NAME:end -->."""
    out = {
        "properties": "\n".join(properties(results)),
        "single": "\n".join(single_rows(results)),
        "multi": "\n".join(multi_rows(results)),
        "groups": "\n".join(group_rows(results)),
    }
    for g in CURVE_KEY:
        out[f"curve-{g}"] = "\n".join(curve(results, g))
    out["curve-spike"] = "\n".join(curve(results, "spike", "p"))
    return out


def update_doc(path: str, results: list[dict]) -> int:
    text = open(path).read()
    new = blocks(results)
    count = 0
    for name, body in new.items():
        begin, end = f"<!-- campaign:{name}:begin -->", f"<!-- campaign:{name}:end -->"
        if begin in text and end in text:
            a, b = text.index(begin) + len(begin), text.index(end)
            text = text[:a] + "\n" + body + "\n" + text[b:]
            count += 1
    open(path, "w").write(text)
    return count


def main(argv=None) -> int:
    argv = argv if argv is not None else sys.argv[1:]
    if not argv:
        print(__doc__)
        return 2
    results = load(argv[0])
    if len(argv) == 3 and argv[1] == "--update-doc":
        n = update_doc(argv[2], results)
        print(f"updated {n} block(s) in {argv[2]}")
        return 0
    print("### Properties\n")
    print("\n".join(properties(results)))
    print("\n### Single faults, by kind (triplex, fault on one node)\n")
    print("\n".join(single_rows(results)))
    print("\n### Multi-fault, recovery and interaction groups\n")
    print("\n".join(multi_rows(results)))
    for g in argv[1:]:
        print(f"\n### Response curve: `{g}`\n")
        print("\n".join(curve(results, g)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
