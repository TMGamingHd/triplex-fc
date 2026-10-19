#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""The FMECA: check its data against the repository, and write docs/verification/FMECA.md from it.

  python3 tools/fmeca/fmeca.py check    exit 1 if the data contradicts the fault matrix, the campaign, the requirements or the limitations, or the document is not the one the data makes (CI)
  python3 tools/fmeca/fmeca.py write    rewrite docs/verification/FMECA.md
  python3 tools/fmeca/fmeca.py summary  the numbers, on the terminal

The data is tools/fmeca/data.py. What the checks guarantee is in FMECA.md section 8: no row of the fault matrix and no fault kind of the campaign is left out of the analysis, every
critical item has a compensation that a test exercises or a recommendation that says what is missing, the single points are the ones LIMITATIONS.md lists, and the measured detection
figures in the document are the campaign's own.
"""
from __future__ import annotations

import re
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(Path(__file__).resolve().parent))
import data as D  # noqa: E402

DOC = ROOT / "docs" / "verification" / "FMECA.md"
SEV_NAME = {1: "Catastrophic", 2: "Critical", 3: "Major", 4: "Minor"}
SEV_NUMBER = {1: 4, 2: 3, 3: 2, 4: 1}   # ECSS-Q-ST-30-02C Table 5-1
PN_NAME = {4: "Probable", 3: "Occasional", 2: "Remote", 1: "Extremely remote"}
CRITICAL_CN = 6                           # ECSS-Q-ST-30-02C 5.3a
EVIDENCE_ORDER = ["none", "rig pending", "host, rig pending", "verified on the host", "out of v1"]


@dataclass
class FaultRow:
    id: str
    fault: str
    reqs: list[str]
    tests: str
    status: str


def read(rel: str) -> str:
    return (ROOT / rel).read_text()


def fault_rows() -> dict[str, FaultRow]:
    out: dict[str, FaultRow] = {}
    for line in read("docs/verification/FAULT_MATRIX.md").splitlines():
        m = re.match(r"^\| (F\d+) \| (.*?) \| (.*?) \| (.*?) \| (.*?) \| (.*?) \| (.*?) \| (.*?) \|$", line)
        if m:
            fid, fault, _inj, _det, _resp, req, tests, status = m.groups()
            out[fid] = FaultRow(fid, fault, re.findall(r"[A-Z]+-\d+", req), tests, status)
    return out


def requirement_ids() -> set[str]:
    return set(re.findall(r"^\| (TFC-[A-Z]+-\d+) \|", read("docs/verification/REQUIREMENTS.md"), re.M))


def campaign() -> tuple[dict[str, dict], set[str]]:
    """Per single fault kind of the campaign: its figures; and the names of all the groups."""
    text = read("docs/verification/FAULT_CAMPAIGN.md")
    kinds: dict[str, dict] = {}
    blk = re.search(r"<!-- campaign:single:begin -->(.*?)<!-- campaign:single:end -->", text, re.S)
    for line in (blk.group(1) if blk else "").splitlines():
        m = re.match(r"^\| `(\w+)` \| (\d+) \| (\d+)/(\d+) \| ([^|]*) \| (\d+)/(\d+) \|", line)
        if not m:
            continue
        lat = [int(x) for x in re.findall(r"\d+", m.group(5))]
        kinds[m.group(1)] = {"scenarios": int(m.group(2)), "iso": int(m.group(3)), "of": int(m.group(4)), "p95": lat[2] if len(lat) == 4 else None,
                             "max": lat[3] if len(lat) == 4 else None, "false": int(m.group(6)), "ignore": int(m.group(7))}
    groups: set[str] = set()
    blk = re.search(r"<!-- campaign:groups:begin -->(.*?)<!-- campaign:groups:end -->", text, re.S)
    for line in (blk.group(1) if blk else "").splitlines():
        m = re.match(r"^\| `(\w+)` \|", line)
        if m:
            groups.add(m.group(1))
    return kinds, groups


def fault_kinds() -> set[str]:
    r = subprocess.run([sys.executable, "-c", "from tfc_peers import faults;print(' '.join(faults.KINDS))"], cwd=ROOT / "sim", capture_output=True, text=True, check=True)
    return set(r.stdout.split())


def evidence_of(modes_faults: tuple[str, ...], rows: dict[str, FaultRow], scope: str, comp: str = "") -> str:
    if scope == "out of v1":
        return "out of v1"
    if not modes_faults or comp.lower().startswith("none"):   # (a mode whose compensation is "none" has none, whatever partial defence a test exercises)
        return "none"
    classes = []
    for f in modes_faults:
        if f not in rows:   # (reported as an error by check_data; it is no evidence)
            classes.append("none")
            continue
        s = rows[f].status
        passing = "Passing" in s
        pending = re.search(r"rig pending|HIL|not started|waits|untested|not yet|on a board", s, re.I) is not None
        classes.append("host, rig pending" if passing and pending else ("verified on the host" if passing else "rig pending"))
    return min(classes, key=EVIDENCE_ORDER.index)


def cn(m: D.Mode) -> int:
    return SEV_NUMBER[m.sev] * m.pn


def critical(m: D.Mode) -> bool:
    return m.sev == 1 or cn(m) >= CRITICAL_CN


def label(m: D.Mode) -> str:
    return f"{m.sev}{m.suffix}"


def kind_of_group(name: str) -> str:
    return "bias" if name in ("bias_gyro", "bias_accel") else name


def measured(m: D.Mode, camp: dict[str, dict]) -> str:
    sel = [camp[k] for k in m.kinds if k in camp]
    if not sel:
        return ""
    iso, of = sum(k["iso"] for k in sel), sum(k["of"] for k in sel)
    false, ign = sum(k["false"] for k in sel), sum(k["ignore"] for k in sel)
    p95 = max((k["p95"] for k in sel if k["p95"] is not None), default=None)
    mx = max((k["max"] for k in sel if k["max"] is not None), default=None)
    lat = f"; p95 {p95} / max {mx} frames" if p95 is not None else ""
    return f"campaign: isolated {iso}/{of}{lat}; falsely isolated {false}/{ign}"


# ----------------------------------------------------------------------------------------------------------------------------------------------------------------------
# the checks
# ----------------------------------------------------------------------------------------------------------------------------------------------------------------------
def check_data() -> list[str]:
    errors: list[str] = []
    rows = fault_rows()
    reqs = requirement_ids()
    camp, groups = campaign()
    kinds = fault_kinds()
    ids = [m.id for m in D.MODES]
    for dup in {i for i in ids if ids.count(i) > 1}:
        errors.append(f"{dup}: the identifier is used twice")
    rec_ids = {r.id for r in D.RECS}
    if len(rec_ids) != len(D.RECS):
        errors.append("a recommendation identifier is used twice")
    for r in D.RECS:
        if r.status not in ("open", "done", "accepted"):
            errors.append(f"{r.id}: status '{r.status}' is not open, done or accepted")
    covered_faults: set[str] = set()
    covered_kinds: set[str] = set()
    used_recs: set[str] = set()
    for m in D.MODES:
        if not re.fullmatch(r"FM-[A-Z]\d+", m.id):
            errors.append(f"{m.id}: not an identifier of the form FM-X1")
        if m.sev not in SEV_NAME or m.pn not in PN_NAME:
            errors.append(f"{m.id}: severity {m.sev} or probability {m.pn} is outside 1 to 4")
        if m.suffix not in ("", "R", "SP", "SH"):
            errors.append(f"{m.id}: suffix '{m.suffix}' is not R, SP or SH")
        if m.suffix == "R" and m.pn >= 3:
            errors.append(f"{m.id}: an R mode needs the redundant items to fail too, so a probability number of {m.pn} is not credible; give the reason or lower it")
        if not (m.detect and m.comp and m.pn_basis and m.effect):
            errors.append(f"{m.id}: detection, compensation, effect and the basis of the probability must all be written")
        for f in m.faults:
            if f not in rows:
                errors.append(f"{m.id}: {f} is not a row of the fault matrix")
            covered_faults.add(f)
        for k in m.kinds:
            if k not in kinds and k not in groups:
                errors.append(f"{m.id}: '{k}' is neither a fault kind nor a campaign group")
            covered_kinds.add(kind_of_group(k))
        for r in m.recs:
            used_recs.add(r)
            if r not in rec_ids:
                errors.append(f"{m.id}: {r} is not a recommendation")
        if critical(m) and evidence_of(m.faults, rows, m.scope, m.comp) in ("none",) and not m.recs:
            errors.append(f"{m.id}: a critical item with nothing that exercises its compensation needs a recommendation that says what is missing")
        if critical(m) and not m.scope and evidence_of(m.faults, rows, m.scope, m.comp) == "none" and not any(
                next(r for r in D.RECS if r.id == x).status in ("open", "accepted") for x in m.recs if x in rec_ids):
            errors.append(f"{m.id}: a critical item with no test must point at an open or accepted recommendation")
    for f in sorted(set(rows) - covered_faults):
        errors.append(f"{f}: a row of the fault matrix that no failure mode of the FMECA names ({rows[f].fault[:60]})")
    for k in sorted(kinds - covered_kinds):
        errors.append(f"fault kind '{k}' is not in any failure mode's kinds")
    for f, row in rows.items():
        for r in row.reqs:
            if f"TFC-{r}" not in reqs:
                errors.append(f"{f}: the fault matrix names {r}, which is not in REQUIREMENTS.md")
    for r in D.RECS:
        if r.id not in used_recs and not any(r.id in cell for _n, cells in D.DATA_ITEMS for cell in cells):
            errors.append(f"{r.id}: no failure mode and no cell of the software FMEA raised this recommendation")
    # the single points of LIMITATIONS 1 are all in the analysis as single points
    lim = read("docs/LIMITATIONS.md")
    sp_items = " ".join(m.item for m in D.MODES if m.suffix == "SP").lower()
    for need, word in (("ACT", "act"), ("the shared CAN bus", "can bus"), ("the supervisor", "supervisor"), ("the power rails", "power")):
        if need.lower() in lim.lower() and word not in sp_items:
            errors.append(f"LIMITATIONS.md names {need} as a single point and no single-point failure mode of the FMECA is under it")
    # the software FMEA table
    for name, cells in D.DATA_ITEMS:
        if len(cells) != len(D.DATA_FAULTS):
            errors.append(f"software FMEA '{name}': {len(cells)} cells for {len(D.DATA_FAULTS)} fault types")
            continue
        for cell in cells:
            for e in check_cell(cell, rows, kinds, groups, rec_ids):
                errors.append(f"software FMEA '{name}': {e}")
    return errors


def check_cell(cell: str, rows: dict[str, FaultRow], kinds: set[str], groups: set[str], rec_ids: set[str]) -> list[str]:
    if cell == "n/a" or cell.startswith("host:"):
        return []
    if cell.startswith("GAP "):
        return [] if cell[4:] in rec_ids else [f"'{cell}' names a recommendation that does not exist"]
    if re.fullmatch(r"F\d+(, F\d+)*", cell):
        return [f"{f} is not a row of the fault matrix" for f in cell.split(", ") if f not in rows]
    return [] if cell in kinds or cell in groups else [f"'{cell}' is not a fault kind, a campaign group, a fault-matrix row, 'host: ...', 'n/a' or 'GAP REC-nn'"]


# ----------------------------------------------------------------------------------------------------------------------------------------------------------------------
# the document
# ----------------------------------------------------------------------------------------------------------------------------------------------------------------------
def cell_text(cell: str) -> str:
    if cell == "n/a":
        return "n/a"
    if cell.startswith("GAP "):
        return f"**GAP** ({cell[4:]})"
    if cell.startswith("host:"):
        return "host test"
    if cell.startswith("F"):
        return cell
    return f"`{cell}`"


def esc(s: str) -> str:
    return s.replace("|", "\\|")


def render() -> str:
    rows = fault_rows()
    camp, _groups = campaign()
    modes = list(D.MODES)
    ev = {m.id: evidence_of(m.faults, rows, m.scope, m.comp) for m in modes}
    crit = [m for m in modes if critical(m)]
    sp = [m for m in modes if m.suffix == "SP"]
    recs_open = [r for r in D.RECS if r.status == "open"]
    n_cells = sum(len(c) for _n, c in D.DATA_ITEMS)
    gaps = [(n, f, c) for n, cells in D.DATA_ITEMS for f, c in zip(D.DATA_FAULTS, cells) if c.startswith("GAP ")]
    nas = sum(1 for _n, cells in D.DATA_ITEMS for c in cells if c == "n/a")
    out: list[str] = []
    w = out.append
    w("# FMECA: failure modes, effects and criticality of the system")
    w("")
    w("> Status: **reference, generated** from `tools/fmeca/data.py` by `tools/fmeca/fmeca.py` (do not edit this file: edit the data and run `python3 tools/fmeca/fmeca.py write`). CI runs `check`, which fails when this page is not the one the data makes, when a row of the fault matrix or a fault kind of the campaign is left out, or when a critical item has neither a test nor a recommendation.")
    w("> Method: ECSS-Q-ST-30-02C (FMECA, hardware-software interaction analysis) and NASA-GB-8719.13 appendix D (software FMEA). The severity and probability ratings are **my engineering judgement** (there are no failure-rate data); they are written to be changed by the owner. One analyst, no independent review.")
    w("")
    w("The [fault matrix](FAULT_MATRIX.md) says what is injected and what is expected; the [hazard analysis](HAZARDS.md) (STPA) starts from the losses and works down; this page works from the items up and ranks the failure modes by how bad and how likely they are, so that the project knows which ones to spend its remaining effort on. [`FMEA.md`](FMEA.md) is the older gap analysis that grew the fault kinds from 13 to 32.")
    w("")
    w("## 1. Where it stands")
    w("")
    w(f"* **{len(modes)} failure modes** of {len({m.item for m in modes})} items cover all {len(rows)} rows of the fault matrix and all {len(fault_kinds())} fault kinds of the campaign (the tool refuses to build the page otherwise).")
    w(f"* **{len(crit)} critical items** (severity 1, or CN of {CRITICAL_CN} or more: ECSS 5.3). Evidence for their compensation: "
      + ", ".join(f"{sum(1 for m in crit if ev[m.id] == e)} {e}" for e in EVIDENCE_ORDER if any(ev[m.id] == e for m in crit)) + ".")
    w(f"* **{len(sp)} single points** (suffix SP): " + ", ".join(f"{m.id} ({m.item.split(' (')[0].lower()})" for m in sp) + ". These are the limits of the claim, and `LIMITATIONS.md` section 1 lists the same ones.")
    w(f"* **{len(recs_open)} open recommendations** of {len(D.RECS)} ({sum(1 for r in D.RECS if r.status == 'done')} done, {sum(1 for r in D.RECS if r.status == 'accepted')} accepted as limits): section 7.")
    w(f"* **Software FMEA** of {len(D.DATA_ITEMS)} data items against {len(D.DATA_FAULTS)} fault types: {n_cells} cells, {nas} not applicable, **{len(gaps)} gaps** that nothing exercises: section 6.")
    w("")
    w("### Criticality matrix")
    w("")
    w("Severity (rows) against probability (columns); each cell lists the failure modes. The upper right is where the effort belongs. ECSS 5.3: a failure mode is a critical item if it is severity 1 or its CN (severity number x probability number) is 6 or more.")
    w("")
    w("| Severity \\ Probability | 1 Extremely remote | 2 Remote | 3 Occasional | 4 Probable |")
    w("|---|---|---|---|---|")
    for s in (1, 2, 3, 4):
        cells = []
        for p in (1, 2, 3, 4):
            ids = [f"{m.id}{('*' if critical(m) else '')}" for m in modes if m.sev == s and m.pn == p]
            cells.append(" ".join(ids) if ids else "")
        w(f"| {s} {SEV_NAME[s]} (SN {SEV_NUMBER[s]}) | " + " | ".join(cells) + " |")
    w("")
    w("`*` marks a critical item.")
    w("")
    w("## 2. How to read it")
    w("")
    w("**Severity** is the worst end effect *if nothing compensates* (ECSS 4.2b), tailored to this project (its losses are in [`HAZARDS.md`](HAZARDS.md)):")
    w("")
    w("| Cat | Name | Meaning here |")
    w("|---|---|---|")
    w("| 1 | Catastrophic | a wrong or absent actuator command: loss of the vehicle or of its attitude control; an unintended launch; harm to people at the platform |")
    w("| 2 | Critical | the mission is lost: Safe in flight, a stage that misses its tower, a run that cannot go on |")
    w("| 3 | Major | redundancy lost or accuracy degraded; the run goes on; the operator is asked |")
    w("| 4 | Minor | no effect on the run; logged |")
    w("")
    w("**Suffixes** (ECSS 4.2d): **R** the effect needs the failure of every redundant item; **SP** a single point (no redundancy); **SH** a safety hazard to people. **Probability** (PN) is the probability that the end effect occurs, with redundancy taken into account: for an R mode, that the redundant items fail too. PN 1 extremely remote, 2 remote, 3 occasional, 4 probable (ECSS Table 5-2: above 1e-1, 1e-3 to 1e-1, 1e-5 to 1e-3, below 1e-5 per mission; here qualitative, with the reason in the data). **CN = SN x PN** (5.2).")
    w("")
    w("**Evidence** is what stands behind the compensation, from the fault matrix: *verified on the host* (a test passes, on the host, the virtual rig or the real firmware on `vcan0`); *host, rig pending* (the software side passes, the hardware side waits for the rig); *rig pending*; *none* (nothing exercises it: the row carries a recommendation); *out of v1*. **Measured** gives the campaign's own figures for the fault kinds of the row: how many detectable scenarios were isolated, the 95th percentile and the maximum latency in 10 ms frames, and how many scenarios that must be left alone were isolated by mistake (it is 0 everywhere). The latencies are host figures: the board's are a measurement still to make (`STATUS.md`).")
    w("")
    w("**Why the criticality number has no detection factor.** ECSS-Q-ST-30-02C 5.2 ranks a failure mode by severity times probability (CN = SN x PN, 1 to 16); detection and compensation are written down, not multiplied in. The automotive practice of RPN = severity x occurrence x detection (and the AIAG-VDA handbook of 2019 that replaced it with an action-priority table because multiplying ordinal ratings misleads) rates *how likely the fault is to be caught*. Here that is not a judgement but a measurement, so it stands as evidence beside the rating: the campaign's own detection latencies and false isolations, and the fault-matrix status of the test. The ECSS process-FMECA clause 8 does add a detection number; it is for processes and is not used. MIL-STD-1629A\'s Cm = beta x alpha x lambda x t needs part failure rates, which this project does not have.")
    w("")
    w("## 3. The worksheet")
    w("")
    items: list[str] = []
    for m in modes:
        if m.item not in items:
            items.append(m.item)
    for it in items:
        w(f"### {it}")
        w("")
        w("| ID | Failure mode | Sev | PN | CN | Seen by | Compensated by | Evidence |")
        w("|---|---|---|---|---|---|---|---|")
        for m in (x for x in modes if x.item == it):
            star = "*" if critical(m) else ""
            ref = ", ".join(m.faults)
            meas = measured(m, camp)
            partial = ev[m.id] == "none" and ref
            evidence = (ev[m.id] + (f" (a partial defence is tested: {ref})" if partial else (f" ({ref})" if ref else ""))) + (f"<br>{meas}" if meas else "") + (f"<br>{', '.join(m.recs)}" if m.recs else "")
            w(f"| {m.id}{star} | **{esc(m.mode)}**<br>cause: {esc(m.cause)}<br>effect: {esc(m.effect)} | {label(m)} | {m.pn} | {cn(m)} | {esc(m.detect)} | {esc(m.comp)} | {esc(evidence)} |")
        w("")
    w("`*` a critical item. The probability numbers' reasons are in `tools/fmeca/data.py` (`pn_basis`), one line each.")
    w("")
    w("## 4. Critical items and what stands behind them")
    w("")
    w("| ID | Item | Sev | CN | Evidence | Recommendations |")
    w("|---|---|---|---|---|---|")
    for m in sorted(crit, key=lambda x: (-cn(x), x.id)):
        w(f"| {m.id} | {esc(m.item.split(' (')[0])}: {esc(m.mode.split(':')[0])} | {label(m)} | {cn(m)} | {ev[m.id]} | {', '.join(m.recs) or '-'} |")
    w("")
    w("A critical item with evidence *verified on the host* has its compensation exercised by a test; the ones to read first are the single points (the highest CN) and the rows whose evidence is *none* or *rig pending*.")
    w("")
    w("## 5. Hardware-software interaction analysis")
    w("")
    w("ECSS-Q-ST-30-02C clause 7: for each hardware failure the software must answer, the symptom the software sees, what it does, and what that does to the product. These are the failure modes above that start in hardware.")
    w("")
    w("| ID | Hardware failure | Symptom the software sees | Software action | Evidence |")
    w("|---|---|---|---|---|")
    for m in (x for x in modes if x.hw):
        w(f"| {m.id} | {esc(m.mode)} ({esc(m.item.split(' (')[0])}) | {esc(m.detect)} | {esc(m.comp)} | {ev[m.id]} |")
    w("")
    w("## 6. Software FMEA: data faults against the interfaces")
    w("")
    w("NASA-GB-8719.13 appendix D asks, for every data item the software receives or sends, what happens when it is missing, corrupted, wrong, out of range, late, early or out of sequence, repeated or frozen. Each cell names where that fault is exercised. A fault kind of the campaign (`kind`) is **injected on the bus through the virtual peers** and checked by the campaign's oracles; F## names a row of the fault matrix, and *host test* a unit or decoder test: those are covered at the level of the code and not injected end to end. *n/a* means the fault means nothing for the item; **GAP** means nothing exercises it.")
    w("")
    w("| Data item | " + " | ".join(D.DATA_FAULTS) + " |")
    w("|---|" + "---|" * len(D.DATA_FAULTS))
    for name, cells in D.DATA_ITEMS:
        w(f"| {name} | " + " | ".join(cell_text(c) for c in cells) + " |")
    w("")
    if gaps:
        w("The gaps: " + "; ".join(f"{n.split(' (')[0].lower()} / {f} ({c[4:]})" for n, f, c in gaps) + ".")
        w("")
    w("## 7. Recommendations")
    w("")
    w("| ID | Recommendation | Status | Raised by |")
    w("|---|---|---|---|")
    for r in D.RECS:
        w(f"| {r.id} | {esc(r.text)} | {r.status} | {esc(r.source)} |")
    w("")
    w("## 8. What the checks guarantee, and how to keep it so")
    w("")
    w("`python3 tools/fmeca/fmeca.py check` fails the build when: a row of `FAULT_MATRIX.md` or a fault kind of `tfc_peers.faults.KINDS` is in no failure mode; a failure mode names a row, a kind, a group or a recommendation that does not exist; a requirement named by the fault matrix is not in `REQUIREMENTS.md`; an R mode has a probability number of 3 or 4; a critical item has no test and no open or accepted recommendation; `LIMITATIONS.md` names a single point and no SP mode is under it; a cell of the software FMEA is not a kind, a row, a host test, n/a or a GAP; or this page differs from what the data makes (the measured figures included, which are the campaign's own).")
    w("")
    w("When a fault kind is added: a row in `FAULT_MATRIX.md`, a name in `KINDS`, a failure mode here (or a kind added to an existing one), and, if it is a data fault, a cell of section 6. When the rig produces a measurement: change the evidence of the rows it settles (in `FAULT_MATRIX.md`; this page follows) and the probability numbers it moves (in `data.py`).")
    w("")
    w("## 9. Sources")
    w("")
    w("Read for this page, not recalled: **ECSS-Q-ST-30-02C** (6 March 2009), clauses 4.2 (severity categories and the suffixes), 4.3 and 5.3 (critical items), 5.2 (the criticality number), 7 (hardware-software interaction analysis) and Annexes B and C (the worksheets): <https://ecss.nl/standard/ecss-q-st-30-02c-failure-modes-effects-and-criticality-analysis-fmeafmeca/>. **NASA-GB-8719.13**, the Software Safety Guidebook, section 6.6.8 and appendix D (software FMEA: the data and event fault types, the worksheet, the limits of the method): <https://s3vi.ndc.nasa.gov/ssri-kb/static/resources/nasa-gb-8719.13.pdf>. NASA-STD-8739.8A replaced NASA-STD-8719.13; the guidebook still carries the procedure. The AIAG-VDA action-priority change is from secondary sources only.")
    w("")
    return "\n".join(out) + "\n"


def main(argv: list[str]) -> int:
    cmd = argv[0] if argv else "check"
    if cmd == "write":
        errs = check_data()
        if errs:
            print("\n".join(errs))
            return 1
        DOC.write_text(render())
        print(f"wrote {DOC.relative_to(ROOT)}")
        return 0
    if cmd == "summary":
        text = render()
        print("\n".join(text.splitlines()[:16]))
        return 0
    errs = check_data()
    if not errs and (not DOC.exists() or DOC.read_text() != render()):
        errs.append("docs/verification/FMECA.md is not what the data makes (python3 tools/fmeca/fmeca.py write)")
    if errs:
        print("\n".join(errs))
        return 1
    print(f"ok: {len(D.MODES)} failure modes cover {len(fault_rows())} fault-matrix rows and {len(fault_kinds())} fault kinds; FMECA.md is current")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
