# SPDX-License-Identifier: MIT
"""Safety properties checked on every campaign run (from the --dump CSV and the replay summary).

Properties that must ALWAYS hold (any anomaly is a finding):
  M1 mode consistency        mode == Safe if a Safe request is raised, else the number of healthy nodes
  M2 legal state transitions Healthy->Latched/Disabled, Latched->Probation/Disabled, Probation->Healthy/Latched/
                             Disabled, Disabled->Latched only on a clear-disabled command
  M3 valid mask              a node that was out of the vote the previous frame never votes this frame
  M4 Safe stickiness         once raised, the Safe request stays until a clear-safe command
  M5 output sanity           every output finite and bounded
  M6 determinism             the same scenario twice gives byte-identical dumps (sampled)
  M7 hold exactness          a held output channel repeats the previous frame's output exactly (last good value)
  M8 one probation           a probation never starts while another runs, except in total loss (no node healthy, ADR-014)
  M9 Safe holds everything   while a Safe request is raised every output channel is flagged held
  M10 events match states    every node-state change is reported by its event bit (latched, started, readmitted, failed, disabled),
                             and every event bit corresponds to a state change
Properties that hold when at most ONE node is faulty (and redundancy remains):
  S1 no false isolation      a node without a fault is never latched out
  S2 output integrity        a non-held output never differs from the truth by more than the vote tolerance
  S3 no spurious Safe        a single fault in triplex never raises a Safe request
  S4 no readmission while a detect-class fault is active on the node
"""
from __future__ import annotations

import math

from tfc_peers import peers
from tfc_peers.faults import Fault

from .model import INF, TOL, Scenario, faulty_nodes, parsed_faults

H, L, P, D = "H", "L", "P", "D"
LEGAL = {H: {H, L, D}, L: {L, P, D}, P: {P, H, L, D}, D: {D}}


def truth_channels(k: int, rest: bool = False) -> list[float]:
    if rest:
        g, a = (0.0, 0.0, 0.0), (0.0, 0.0, 1.0)
    else:
        g, a = peers.truth(k * 10_000 / 1e6 + 0.0005)
    clamp = lambda v: max(-30.0, min(30.0, v))  # noqa: E731
    return [g[0], g[1], g[2], a[0], a[1], a[2], clamp(0.1 * g[0]), clamp(0.1 * g[1])]


def node_state(row: dict, n: int) -> str:
    bit = 1 << n
    if row["disabled"] & bit:
        return D
    if row["probation"] & bit:
        return P
    if row["latched"] & bit:
        return L
    return H


def fault_end(f: Fault) -> int:
    if f.kind == "reboot":
        return f.start + int(f.params["down"])
    return f.end if f.end is not None else INF


def _events_match(r: dict, prev: dict | None, k: int) -> list[tuple[str, str]]:
    """M10: the report's event bits and the node states agree (only when the dump has the event columns)."""
    if "newly_readmitted" not in r:
        return []
    out: list[tuple[str, str]] = []
    for n in range(3):
        bit = 1 << n
        before = node_state(prev, n) if prev is not None else H
        after = node_state(r, n)
        ev = {"latched": bool(r["newly_latched"] & bit), "started": bool(r["newly_started"] & bit), "readmitted": bool(r["newly_readmitted"] & bit),
              "failed": bool(r["probation_failed"] & bit), "disabled": bool(r["newly_disabled"] & bit)}
        # transition => event (a Healthy node that becomes Latched/Disabled by the monitor reports `latched`; by command, `disabled`)
        if before == H and after == L and not ev["latched"]:
            out.append(("M10", f"frame {k}: node {'ABC'[n]} Healthy->Latched without a newly_latched event"))
        if before == L and after == P and not ev["started"]:
            out.append(("M10", f"frame {k}: node {'ABC'[n]} Latched->Probation without a started event"))
        if before == P and after == H and not ev["readmitted"]:
            out.append(("M10", f"frame {k}: node {'ABC'[n]} Probation->Healthy without a readmitted event"))
        if before == P and after == L and not ev["failed"]:
            out.append(("M10", f"frame {k}: node {'ABC'[n]} Probation->Latched without a failed event"))
        if before != D and after == D and not ev["disabled"]:
            out.append(("M10", f"frame {k}: node {'ABC'[n]} ->Disabled without a newly_disabled event"))
        # event => transition
        if ev["started"] and after != P:
            out.append(("M10", f"frame {k}: node {'ABC'[n]} started-probation event but state is {after}"))
        if ev["readmitted"] and not (before == P and after == H):
            out.append(("M10", f"frame {k}: node {'ABC'[n]} readmitted event without Probation->Healthy"))
        if ev["failed"] and not (before == P and after == L):
            out.append(("M10", f"frame {k}: node {'ABC'[n]} failed-probation event without Probation->Latched"))
        if ev["disabled"] and after != D:
            out.append(("M10", f"frame {k}: node {'ABC'[n]} disabled event but state is {after}"))
    return out


def check_always(sc: Scenario, rows: list[dict], cmds: dict[str, set[int]]) -> list[tuple[str, str]]:
    out: list[tuple[str, str]] = []
    prev_state = {n: H for n in range(3)}
    prev_latched = 0
    safe_since = None
    prev_row = None
    for r in rows:
        k = r["frame"]
        if r.get("newly_started", 0) and bin(r["probation"]).count("1") > 1 and r["latched"] != 0b111:
            out.append(("M8", f"frame {k}: probation mask {r['probation']:03b} has more than one node"))
        # (exempt the frame on which Safe is first requested: channels with a trusted vote that frame are frozen at their
        #  fresh value, see docs/verification/FAULT_CAMPAIGN.md edge case E2)
        if prev_row is not None and r["held"] and not (r["safe"] and not prev_row["safe"]):
            for ch in range(8):
                if r["held"] & (1 << ch) and r[f"out{ch}"] != prev_row[f"out{ch}"]:
                    out.append(("M7", f"frame {k}: held out{ch}={r[f'out{ch}']} but previous frame had {prev_row[f'out{ch}']}"))
                    break
        if r["safe"] and r["held"] != 0xFF:
            out.append(("M9", f"frame {k}: Safe requested but held mask is {r['held']:#04x}"))
        out.extend(_events_match(r, prev_row, k))
        prev_row = r
        expected_mode = 0 if r["safe"] else r["healthy"]
        if r["mode"] != expected_mode or not 0 <= r["healthy"] <= 3:
            out.append(("M1", f"frame {k}: mode={r['mode']} healthy={r['healthy']} safe={r['safe']}"))
        if r["valid"] & prev_latched:
            out.append(("M3", f"frame {k}: valid={r['valid']:03b} overlaps previous latched={prev_latched:03b}"))
        for n in range(3):
            st = node_state(r, n)
            if st not in LEGAL[prev_state[n]]:
                if not (prev_state[n] == D and st == L and any(abs(k - c) <= 1 for c in cmds.get("clear-disabled", ()))):
                    out.append(("M2", f"frame {k}: node {'ABC'[n]} {prev_state[n]}->{st}"))
            prev_state[n] = st
        prev_latched = r["latched"]
        if safe_since is not None and not r["safe"]:
            if not any(abs(k - c) <= 1 for c in cmds.get("clear-safe", ())):
                out.append(("M4", f"frame {k}: Safe request dropped without a clear-safe command"))
        if r["safe"] and safe_since is None:
            safe_since = k
        if not r["safe"]:
            safe_since = None
        for ch in range(8):
            v = r[f"out{ch}"]
            if not math.isfinite(v) or abs(v) > 1e4:
                out.append(("M5", f"frame {k}: out{ch}={v}"))
    return out


def active_on(sc: Scenario, node: int, k: int) -> list[Fault]:
    res = []
    for f in parsed_faults(sc):
        if f.node != node:
            continue
        if f.kind == "reboot":
            if f.start <= k < fault_end(f):
                res.append(f)
        elif f.active(k):
            res.append(f)
    return res


def check_single_fault(sc: Scenario, rows: list[dict], first_latch: dict) -> tuple[list[tuple[str, str]], dict]:
    """S1-S3 for scenarios with at most one faulty node and a context that leaves redundancy."""
    out: list[tuple[str, str]] = []
    metrics = {"max_err": 0.0, "max_err_ch": -1, "max_err_frame": -1, "held_frames": sum(1 for r in rows if r["held"]),
               "max_held_err": 0.0}
    bad = faulty_nodes(sc)
    ctx_dropped = set()
    if sc.context.startswith("duplex-") or sc.context.startswith("simplex-"):
        ctx_dropped = {"ABC".index(c) for c in sc.context.split("-")[1]}
    absent = {i for i, c in enumerate('ABC') if c not in sc.nodes}  # not simulated at all: absent means invalid
    allowed = bad | ctx_dropped | absent
    for n, k in first_latch.items():
        if n not in allowed:
            out.append(("S1", f"healthy node {'ABC'[n]} latched at frame {k}"))
    alive = 3 - len(ctx_dropped | absent)
    s2_first, s2_count, s2_worst = None, 0, 0.0
    for r in rows:
        k = r["frame"]
        truth = truth_channels(k, sc.rest)
        worst_ch, worst_ratio = -1, 0.0
        for ch in range(8):
            err = abs(r[f"out{ch}"] - truth[ch])
            held = bool(r["held"] & (1 << ch))
            ratio = err / TOL[ch]
            if held:
                metrics["max_held_err"] = max(metrics["max_held_err"], ratio)
                continue
            if ratio > metrics["max_err"]:
                metrics["max_err"], metrics["max_err_ch"], metrics["max_err_frame"] = ratio, ch, k
            if ratio > worst_ratio:
                worst_ch, worst_ratio = ch, ratio
        if alive >= 2 and k >= 8 and worst_ratio > 1.0:
            s2_count += 1
            if s2_first is None:
                s2_first = (k, worst_ch, worst_ratio, r["mode"])
            s2_worst = max(s2_worst, worst_ratio)
        if alive >= 2 and r["safe"] and len(bad) <= 1 and r["mode"] == 0 and sc.context == "triplex":
            out.append(("S3", f"frame {k}: Safe requested after a single fault in triplex"))
            break
    if s2_first:
        k, ch, ratio, mode = s2_first
        out.append(("S2", f"{s2_count} frames with the output beyond tolerance; first at frame {k}: out{ch} {ratio:.2f}x tol "
                          f"(mode {mode}); worst {s2_worst:.2f}x"))
    metrics["s2_frames"] = s2_count
    return out, metrics
