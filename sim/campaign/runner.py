# SPDX-License-Identifier: MIT
"""Run one scenario through the real C++ core (tfc_replay) and evaluate it."""
from __future__ import annotations

import csv
import dataclasses
import hashlib
import os
import subprocess
import tempfile
from collections import defaultdict
from pathlib import Path

from tfc_peers import bus as B
from tfc_peers import peers
from tfc_peers.commands import parse_commands
from tfc_peers.faults import parse_fault, parse_node
from tfc_peers.peers import Scenario

from . import oracles
from .model import Result, Scenario as Sc, faulty_nodes, parsed_faults

ROOT = Path(__file__).resolve().parents[2]
REPLAY = os.environ.get("TFC_REPLAY_BIN", str(ROOT / "build/host/tfc_replay"))


def _read_dump(path: str) -> list[dict]:
    rows = []
    with open(path) as fh:
        for r in csv.DictReader(fh):
            rows.append({k: (float(v) if k.startswith("out") else int(v)) for k, v in r.items()})
    return rows


def _parse_stdout(text: str) -> dict:
    kv = {}
    for line in text.splitlines():
        if "=" in line and " " not in line:
            k, v = line.split("=", 1)
            kv[k] = v
    return kv


def execute(sc: Sc, tmp: str, want_dump: bool = True) -> tuple[dict, list[dict], str]:
    """Record the scenario, replay it, return (summary key=values, dump rows, raw stdout)."""
    orig_truth = peers.truth
    if sc.rest:
        peers.truth = lambda t: ((0.0, 0.0, 0.0), (0.0, 0.0, 1.0))
    try:
        nodes = [parse_node(n) for n in sc.nodes.split(",")]
        faults = [parse_fault(f) for f in sc.all_fault_specs()]
        commands = [x for c in sc.commands for x in parse_commands(c)]
        scen = Scenario(nodes, faults, sc.seed, commands)
        log = os.path.join(tmp, "s.log")
        csvp = os.path.join(tmp, "s.csv")
        lb = B.LogBus(log)
        B.record(scen, lb, sc.frames)
        lb.close()
    finally:
        peers.truth = orig_truth
    cmd = [REPLAY, log, "--policy", sc.policy]
    if sc.split:
        cmd.append("--sensor-split")
    if sc.release:
        cmd += ["--release", sc.release]
    if want_dump:
        cmd += ["--dump", csvp]
    r = subprocess.run(cmd, capture_output=True, text=True, timeout=120)
    if r.returncode not in (0, 1):
        raise RuntimeError(f"replay exit {r.returncode}: {r.stderr.strip()[:200]}")
    rows = _read_dump(csvp) if want_dump else []
    return _parse_stdout(r.stdout), rows, r.stdout


def commands_by_op(sc: Sc) -> dict[str, set[int]]:
    """Frames at which each operation is EXECUTED (not armed, not forged, not a replay); `armed-OP` executes two frames later."""
    d: dict[str, set[int]] = defaultdict(set)
    for spec in sc.commands:
        for c in parse_commands(spec):
            if not c.arm and not c.forged and not c.replay:
                d[c.op].add(c.frame)
    return d


def evaluate(sc: Sc, tmp: str) -> Result:
    res = Result(sc)
    try:
        orig = peers.truth
        if sc.rest:
            peers.truth = lambda t: ((0.0, 0.0, 0.0), (0.0, 0.0, 1.0))
        try:
            kv, rows, _ = execute(sc, tmp)
        finally:
            peers.truth = orig
    except Exception as e:  # a crash or a malformed run is itself a finding
        res.ok = False
        res.error = f"{type(e).__name__}: {e}"
        res.anomalies.append(("X_RUN", res.error))
        return res

    # A hash of everything the manager decided, frame by frame (shared columns only, so it compares across versions of
    # the replay tool): two builds of the core that make the same decisions have the same trace.
    shared = ("frame", "mode", "healthy", "valid", "latched", "probation", "disabled", "safe", "alarm", "held", "unresolved",
              "newly_latched", "reason_a", "reason_b", "reason_c", *(f"out{i}" for i in range(8)))
    h = hashlib.sha256()
    for r in rows:
        h.update(repr(tuple(r[c] for c in shared)).encode())
    res.metrics["trace"] = h.hexdigest()[:16]
    res.counters = {k: int(v) for k, v in kv.items() if v.lstrip("-").isdigit()}
    res.metrics["sensor_latch"] = {n: int(kv[f"slatch.{name}"]) for n, name in enumerate("ABC") if kv.get(f"slatch.{name}", "-") != "-"}
    res.final_mode = kv.get("mode", "")
    for n, name in enumerate("ABC"):
        lf = kv.get(f"latch.{name}", "-")
        if lf != "-":
            res.latch[n] = int(lf)
        res.final_state[n] = kv.get(f"state.{name}", "")
        res.strikes[n] = int(kv.get(f"strikes.{name}", "0"))
    for r in rows:
        for n in range(3):
            if r["newly_latched"] & (1 << n) and n not in res.reason_at_latch:
                res.reason_at_latch[n] = r[f"reason_{'abc'[n]}"]

    cmds = commands_by_op(sc)
    for code, detail in oracles.check_always(sc, rows, cmds):
        res.anomalies.append((code, detail))

    bad = faulty_nodes(sc)
    ctx_alive = 3 - (len(sc.context.split("-")[1]) if "-" in sc.context else 0)
    first_latch = dict(res.latch)
    # S-properties: a single faulty node (the context's dropped nodes are deliberate, not faults under test)
    s_anoms, metrics = oracles.check_single_fault(sc, rows, first_latch)
    res.metrics.update(metrics)  # measurements are always recorded; the S properties only judge single-fault runs
    if len(bad - {"ABC".index(c) for c in (sc.context.split("-")[1] if "-" in sc.context else "")}) <= 1:
        res.anomalies += s_anoms
    res.metrics.update({
        "frames": len(rows),
        "min_mode": min((r["mode"] for r in rows), default=3),
        "safe_ever": any(r["safe"] for r in rows),
        "alarm_frames": sum(1 for r in rows if r["alarm"]),
        "held_frames": sum(1 for r in rows if r["held"]),
        "unresolved_frames": sum(1 for r in rows if r["unresolved"]),
    })

    # S4: no readmission while a detect-class fault is active on the node
    prev = {n: "H" for n in range(3)}
    for r in rows:
        for n in range(3):
            st = oracles.node_state(r, n)
            if prev[n] == "P" and st == "H" and sc.expect in ("detect", "detect_or_safe"):
                if any(f.node == n and f.kind not in ("babble",) and (oracles.active_on(sc, n, r["frame"]) or
                       oracles.active_on(sc, n, r["frame"] - 5)) for f in parsed_faults(sc)):
                    res.anomalies.append(("S4", f"node {'ABC'[n]} readmitted at frame {r['frame']} while its fault is active"))
            prev[n] = st

    # Expectation (advisory): derived from the fault's parameters in grids.py. Nodes the context dropped on
    # purpose (Duplex/Simplex set-up) are not part of the fault under test.
    ctx_nodes = {"ABC".index(c) for c in (sc.context.split("-")[1] if "-" in sc.context else "")}
    bad = bad - ctx_nodes
    res_latch_fut = {n: f for n, f in res.latch.items() if n not in ctx_nodes}
    target = next(iter(bad), None) if len(bad) == 1 else None
    fstart = min((f.start for f in parsed_faults(sc)), default=0)
    if sc.expect in ("detect", "detect_or_safe") and target is not None:
        got = res_latch_fut.get(target)
        safe = res.metrics.get("safe_ever", False)
        if got is None and not (sc.expect == "detect_or_safe" and safe):
            res.anomalies.append(("E_MISS", f"expected node {'ABC'[target]} to be isolated; final={res.final_state.get(target)}"))
        elif got is not None and sc.latency_max is not None and got - fstart > sc.latency_max:
            res.anomalies.append(("E_SLOW", f"latched at {got} ({got - fstart} after start; limit {sc.latency_max})"))
    elif sc.expect == "ignore":
        if res_latch_fut:
            res.anomalies.append(("E_FALSE", f"expected no isolation but latched {dict(res_latch_fut)}"))
        if res.metrics.get("safe_ever") and sc.context == "triplex":
            res.anomalies.append(("E_FALSE", "expected no Safe request"))
    # Tag-driven checks (alarm, sequence errors, readmission frame, final state)
    tag = sc.tag
    if "alarm_expected" in tag:
        if tag["alarm_expected"] and res.metrics["alarm_frames"] < 1:
            res.anomalies.append(("E_ALARM", "bus alarm expected but never raised"))
        if not tag["alarm_expected"] and res.metrics["alarm_frames"] > 0:
            res.anomalies.append(("E_ALARM", f"bus alarm raised {res.metrics['alarm_frames']} frames but not expected"))
    if "seq_bad_expected" in tag and res.counters.get("seq_bad", -1) != tag["seq_bad_expected"]:
        res.anomalies.append(("E_SEQ", f"seq_bad={res.counters.get('seq_bad')} expected {tag['seq_bad_expected']}"))
    node = "ABC".index(tag.get("node", "B")) if tag.get("node", "B") in "ABC" else 1
    if "readmit_at" in tag:
        got = None
        pv = "H"
        for r in rows:
            st = oracles.node_state(r, node)
            if pv == "P" and st == "H" and got is None:
                got = r["frame"]
            pv = st
        if got != tag["readmit_at"]:
            res.anomalies.append(("E_READMIT", f"readmitted at {got}, expected {tag['readmit_at']}"))
    if "starts_expected" in tag:
        starts = [r["frame"] for r in rows if r.get("newly_started", 0) & (1 << node)]
        if starts != tag["starts_expected"]:
            res.anomalies.append(("E_START", f"probation of node {'ABC'[node]} started at {starts}, expected {tag['starts_expected']}"))
    if "final_mode" in tag and res.final_mode != tag["final_mode"]:
        res.anomalies.append(("E_MODE", f"final mode {res.final_mode}, expected {tag['final_mode']}"))
    for key in ("reintegrations", "nodes_disabled", "probation_failures", "commands_refused", "commands_accepted", "commands_unauthentic",
                "commands_replayed", "arms_expired", "critical_commands"):
        if f"{key}_expected" in tag and res.counters.get(key, -1) != tag[f"{key}_expected"]:
            res.anomalies.append(("E_COUNT", f"{key}={res.counters.get(key)} expected {tag[key + '_expected']}"))
        if f"{key}_min" in tag and res.counters.get(key, -1) < tag[f"{key}_min"]:
            res.anomalies.append(("E_COUNT", f"{key}={res.counters.get(key)} expected at least {tag[key + '_min']}"))
    if "strikes_expected" in tag and res.strikes.get(node) != tag["strikes_expected"]:
        res.anomalies.append(("E_COUNT", f"strikes={res.strikes.get(node)} expected {tag['strikes_expected']}"))
    if "final_state" in tag and res.final_state.get(node) != tag["final_state"]:
        res.anomalies.append(("E_STATE", f"node {'ABC'[node]} final state {res.final_state.get(node)}, expected {tag['final_state']}"))
    if sc.split:
        res.anomalies += split_checks(sc, res, rows, tmp)
    if "release_expect" in sc.tag:
        if sc.tag["release_expect"] == "conflict":  # a Safe request after a single fault is the design here: nothing in the data says which release is right (ADR-021), so S3 does not apply
            res.anomalies = [a for a in res.anomalies if a[0] != "S3"]
        res.anomalies += release_checks(sc, res)
    return res


def release_checks(sc: Sc, res: Result) -> list[tuple[str, str]]:
    """ADR-021: with the releases reported and two sharing one, a disagreement between the lone computer and the pair beyond the version tolerance isolates nobody and requests Safe;
    within it, nothing happens at all. (Without `--release` the same faults isolate the odd computer: that is the weakness the rule is for, F63.)"""
    out: list[tuple[str, str]] = []
    safe = res.metrics.get("safe_ever", False)
    if res.latch:
        out.append(("E_REL_ISOLATED", f"a computer was latched ({res.latch}) although the evidence is a disagreement between releases"))
    if sc.tag["release_expect"] == "conflict" and not safe:
        out.append(("E_REL_NO_SAFE", "the releases disagreed beyond the version tolerance and no Safe request was raised"))
    if sc.tag["release_expect"] == "none" and safe:
        out.append(("E_REL_SAFE", "a difference within the version tolerance raised a Safe request"))
    return out


def split_checks(sc: Sc, res: Result, rows: list[dict], tmp: str) -> list[tuple[str, str]]:
    """What the sensor split promises (ADR-020 case 1, TS-15): a fault of a computer's IMU removes the IMU channel and not the computer; a fault of the computer's
    commands removes the computer and not its IMU; and the split detects what the unsplit manager detected, no later than two frames after it. `tag["class"]` is
    sensor, command or pair (an IMU fault on one computer and a command fault on another); other scenarios are judged by the always-true properties only."""
    out: list[tuple[str, str]] = []
    cls = sc.tag.get("class")
    s_latch = res.metrics.get("sensor_latch", {})
    if cls in ("sensor", "command", "pair") and sc.context == "triplex":
        imu_node = "ABC".index(sc.tag["imu"]) if "imu" in sc.tag else None
        cmd_node = "ABC".index(sc.tag["cmd"]) if "cmd" in sc.tag else None
        stray_computers = {n for n in res.latch if n != cmd_node}
        stray_imus = {n for n in s_latch if n != imu_node}
        if stray_computers:
            out.append(("E_SPLIT_COMPUTER", f"a computer was latched for a fault it does not have: {sorted(stray_computers)} (latches {res.latch})"))
        if stray_imus:
            out.append(("E_SPLIT_IMU", f"an IMU channel was latched that has no fault: {sorted(stray_imus)} (sensor latches {s_latch})"))
        base = dataclasses.replace(sc, split=False)
        with tempfile.TemporaryDirectory(prefix="tfc-camp-base-") as btmp:
            kv, _rows, _ = execute(base, btmp, want_dump=False)
        off = {n: int(kv[f"latch.{name}"]) for n, name in enumerate("ABC") if kv.get(f"latch.{name}", "-") != "-"}
        fstart = min((f.start for f in parsed_faults(sc)), default=0)
        if imu_node is not None and imu_node in off:
            got = s_latch.get(imu_node)
            if got is None:
                out.append(("E_SPLIT_MISS", f"the unsplit manager latched computer {'ABC'[imu_node]} at {off[imu_node]} for its IMU fault; the split never latched the IMU"))
            elif got > off[imu_node] + 2:
                out.append(("E_SPLIT_SLOW", f"IMU {'ABC'[imu_node]} latched at {got}, the unsplit computer at {off[imu_node]} (start {fstart})"))
        if cmd_node is not None and cmd_node in off and res.latch.get(cmd_node) is None:
            out.append(("E_SPLIT_MISS", f"the unsplit manager latched computer {'ABC'[cmd_node]} at {off[cmd_node]} for its command fault; the split never did"))
        if cmd_node is not None and cmd_node in off and res.latch.get(cmd_node) is not None and res.latch[cmd_node] > off[cmd_node] + 2:
            out.append(("E_SPLIT_SLOW", f"computer {'ABC'[cmd_node]} latched at {res.latch[cmd_node]} with the split, at {off[cmd_node]} without"))
    return out


def _sampled_for_determinism(key: str) -> bool:
    """A fixed 1-in-25 sample of scenarios (by hash of the key, so the same ones every run) is executed twice."""
    return int(hashlib.sha1(key.encode()).hexdigest()[:8], 16) % 25 == 0


def run_scenario(args: tuple) -> dict:
    sc: Sc = args[0]
    with tempfile.TemporaryDirectory(prefix="tfc-camp-") as tmp:
        res = evaluate(sc, tmp)
    if res.ok and _sampled_for_determinism(sc.key()):  # M6: the same scenario twice gives the same decisions, frame for frame
        with tempfile.TemporaryDirectory(prefix="tfc-camp-") as tmp:
            again = evaluate(sc, tmp)
        if again.metrics.get("trace") != res.metrics.get("trace") or again.latch != res.latch or again.counters != res.counters:
            res.anomalies.append(("M6", f"second run differs: trace {res.metrics.get('trace')} vs {again.metrics.get('trace')}"))
    sc_d = sc.__dict__.copy()
    return {"scenario": sc_d, "key": sc.key(), "ok": res.ok, "error": res.error, "latch": res.latch,
            "reason_at_latch": res.reason_at_latch, "final_state": res.final_state, "strikes": res.strikes,
            "final_mode": res.final_mode, "counters": res.counters, "metrics": res.metrics,
            "anomalies": res.anomalies}
