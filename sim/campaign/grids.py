# SPDX-License-Identifier: MIT
"""Scenario grids: every fault kind over its parameter space, all target nodes, several start times, and redundancy
contexts. Each scenario carries the outcome expected from the fault's parameters (derived from the documented
thresholds: vote tolerance 1.0 dps / 0.02 g / 0.01 deg, 3-of-5, leaky count K=0.9 T=3, stuck limit 20, vote at 7 ms).
"expect" values: detect (must be isolated), detect_or_safe (Duplex: isolated, or Safe requested), ignore (must not be
isolated), gray (near a threshold: report, do not judge), any (only the always-true safety properties apply)."""
from __future__ import annotations

import itertools
import math

from .model import Scenario

N = "ABC"
FRAMES = 400
GYRO_TOL, ACCEL_TOL, CMD_TOL = 1.0, 0.02, 0.01


def _ctxs(node: int, duplex: bool = True) -> list[str]:
    out = ["triplex"]
    if duplex:
        out.append(f"duplex-{N[(node + 1) % 3]}")
    return out


def _sc(group, faults, expect="any", lat=None, **kw) -> Scenario:
    tag = kw.pop("tag", {})
    return Scenario(group=group, faults=faults, expect=expect, latency_max=lat, tag=tag, **kw)


def _dx(expect: str, ctx: str) -> str:
    """In a Duplex context a detectable fault may end in a Safe request instead of an isolation."""
    return "detect_or_safe" if expect == "detect" and ctx.startswith("duplex") else expect


# ------------------------------------------------------------------ one group per fault kind
def dropout() -> list[Scenario]:
    out = []
    for n, start, dur in itertools.product(range(3), (0, 1, 50, 100), (1, 2, 3, 4, 5, 8, 20, None)):
        for ctx in _ctxs(n):
            spec = f"{N[n]}:dropout:start={start}" + (f",end={start + dur}" if dur else "")
            exp = "ignore" if dur and dur <= 2 else "detect"
            out.append(_sc("dropout", [spec], _dx(exp, ctx), 2, context=ctx, frames=FRAMES,
                           tag=dict(node=N[n], start=start, dur=dur)))
    return out


def stuck() -> list[Scenario]:
    out = []
    for n, start, dur, rest in itertools.product(range(3), (20, 100), (5, 10, 19, 20, 21, 22, 23, 25, 40, 100, None), (False, True)):
        for ctx in _ctxs(n):
            spec = f"{N[n]}:stuck:start={start}" + (f",end={start + dur}" if dur else "")
            if rest:
                exp = "detect" if dur is None or dur >= 23 else ("ignore" if dur <= 19 else "gray")
                lat = 22
            else:
                exp = "detect" if dur is None or dur >= 60 else "gray"
                lat = 40
            out.append(_sc("stuck", [spec], _dx(exp, ctx), lat, context=ctx, frames=FRAMES, rest=rest,
                           tag=dict(node=N[n], start=start, dur=dur, rest=rest)))
    return out


def _bias(sensor: str) -> list[Scenario]:
    tol = GYRO_TOL if sensor == "gyro" else ACCEL_TOL
    out = []
    mags = (0.05, 0.2, 0.5, 0.8, 0.9, 0.95, 1.0, 1.05, 1.2, 1.5, 2, 3, 10, 100, 10000)
    for n, axis, m, sign in itertools.product(range(3), range(3), mags, (1, -1)):
        for ctx in _ctxs(n, duplex=(axis == 0)):
            spec = f"{N[n]}:bias:start=100,sensor={sensor},axis={axis},mag={sign * m * tol:g}"
            exp = "detect" if m >= 1.5 else ("ignore" if m <= 0.8 else "gray")
            out.append(_sc(f"bias_{sensor}", [spec], _dx(exp, ctx), 3, context=ctx, frames=FRAMES,
                           tag=dict(node=N[n], axis=axis, rel=m * sign)))
    for n, dur, m in itertools.product(range(3), (1, 2, 3, 4, 5, 20), (1.5, 3, 10)):  # windowed bias
        spec = f"{N[n]}:bias:start=100,end={100 + dur},sensor={sensor},axis=0,mag={m * tol:g}"
        out.append(_sc(f"bias_{sensor}", [spec], "ignore" if dur <= 2 else "detect", 3, frames=FRAMES,
                       tag=dict(node=N[n], dur=dur, rel=m)))
    return out


def drift() -> list[Scenario]:
    out = []
    for sensor, n, axis, r in itertools.product(("gyro", "accel"), range(3), (0, 2), (0.0005, 0.001, 0.003, 0.01, 0.02, 0.05, 0.1, 0.5, 2, 10)):
        tol = GYRO_TOL if sensor == "gyro" else ACCEL_TOL
        t_cross = 1.0 / r
        exp = "detect" if t_cross <= 250 else ("ignore" if t_cross > 520 else "gray")
        lat = int(1.6 * t_cross) + 8
        for ctx in _ctxs(n, duplex=(axis == 0 and sensor == "gyro")):
            spec = f"{N[n]}:drift:start=100,sensor={sensor},axis={axis},rate={r * tol:g}"
            out.append(_sc("drift", [spec], "detect_or_safe" if exp == "detect" and ctx.startswith("duplex") else exp, lat,
                           context=ctx, frames=700, tag=dict(node=N[n], rel_rate=r, sensor=sensor)))
    return out


def spike() -> list[Scenario]:
    out = []
    for sensor, n, p, m, seed in itertools.product(("gyro", "accel"), range(3), (0.005, 0.02, 0.05, 0.1, 0.2, 0.4, 0.6, 1.0),
                                                   (0.5, 0.9, 1.5, 3, 20, 1000), range(1, 5)):
        if sensor == "accel" and seed > 2:
            continue
        tol = GYRO_TOL if sensor == "gyro" else ACCEL_TOL
        spec = f"{N[n]}:spike:start=100,sensor={sensor},mag={m * tol:g},p={p}"
        if m <= 0.5:
            exp = "ignore"
        elif m <= 0.9:
            exp = "gray"  # near the tolerance, sensor noise tips some samples over it
        elif p >= 0.6:
            exp = "detect"
        elif p <= 0.02:
            exp = "ignore"
        else:
            exp = "gray"
        out.append(_sc("spike", [spec], exp, 40, seed=seed, frames=FRAMES, tag=dict(node=N[n], p=p, rel=m, sensor=sensor)))
    return out


def saturate() -> list[Scenario]:
    out = []
    for n, start, dur in itertools.product(range(3), (0, 1, 100), (None, 1, 2, 3, 10)):
        for ctx in _ctxs(n):
            spec = f"{N[n]}:saturate:start={start}" + (f",end={start + dur}" if dur else "")
            exp = "ignore" if dur and dur <= 2 else "detect"
            out.append(_sc("saturate", [spec], _dx(exp, ctx), 3, context=ctx, frames=FRAMES, tag=dict(node=N[n], start=start, dur=dur)))
    return out


def corrupt() -> list[Scenario]:
    out = []
    for n, p, seed in itertools.product(range(3), (0.005, 0.01, 0.03, 0.05, 0.1, 0.2, 0.3, 0.5, 0.7, 0.9, 1.0), range(1, 7)):
        exp = "detect" if p >= 0.7 else ("ignore" if p <= 0.005 else "gray")  # random errors cluster: see FAULT_CAMPAIGN.md
        out.append(_sc("corrupt", [f"{N[n]}:corrupt:start=100,p={p}"], exp, 30, seed=seed, frames=FRAMES,
                       tag=dict(node=N[n], p=p)))
    return out


PERIODS = [(2, 1), (3, 1), (4, 1), (5, 1), (5, 2), (5, 3), (6, 1), (6, 2), (8, 1), (10, 1), (10, 2), (10, 3), (20, 1), (20, 5), (50, 10)]


def _frac_expect(period: int, duty: int) -> tuple[str, int]:
    f = duty / period
    if duty >= 3 or f >= 0.4:
        return "detect", 45
    if duty == 1 and period >= 5:
        return "ignore", 0
    return "gray", 60


def corrupt_periodic() -> list[Scenario]:
    out = []
    for n, (per, duty) in itertools.product(range(3), PERIODS):
        exp, lat = _frac_expect(per, duty)
        out.append(_sc("corrupt_periodic", [f"{N[n]}:corrupt:start=100,p=1.0,period={per},duty={duty}"], exp, lat or None, frames=700,
                       tag=dict(node=N[n], period=per, duty=duty)))
    return out


def cmd_offset() -> list[Scenario]:
    out = []
    for n, mag, sign in itertools.product(range(3), (0.0005, 0.001, 0.003, 0.005, 0.009, 0.01, 0.011, 0.015, 0.02, 0.05, 0.5, 5.0, 29.0, 31.0, 1000.0), (1, -1)):
        exp = "detect" if mag >= 0.02 else ("ignore" if mag <= 0.005 else "gray")
        for ctx in _ctxs(n):
            out.append(_sc("cmd_offset", [f"{N[n]}:cmd_offset:start=100,mag={sign * mag}"], _dx(exp, ctx), 3, context=ctx,
                           frames=FRAMES, tag=dict(node=N[n], mag=sign * mag)))
    return out


def digest() -> list[Scenario]:
    out = []
    for n, x in itertools.product(range(3), (0, 1, 2, 4, 0x80, 0x8000, 0xFFFF, 0x10000, 0x10001, 0x1234)):
        exp = "ignore" if (x & 0xFFFF) == 0 else "detect"
        for ctx in _ctxs(n):
            out.append(_sc("digest", [f"{N[n]}:digest:start=100,xor={x}"], _dx(exp, ctx), 3, context=ctx, frames=FRAMES,
                           tag=dict(node=N[n], xor=x)))
    return out


def babble() -> list[Scenario]:
    out = []
    for n, k in itertools.product(range(3), (0, 1, 2, 3, 4, 8, 20, 60)):
        out.append(_sc("babble", [f"{N[n]}:babble:start=100,n={k}"], "ignore", frames=FRAMES,
                       tag=dict(node=N[n], n=k, alarm_expected=k >= 3)))
    # ids above the simulator id and between the schedule's ids are out-of-schedule too (E14): counted, alarm at 3 or more per frame
    for n, bid, k in itertools.product(range(3), (0x301, 0x403, 0x4F0, 0x511, 0x520, 0x6F0, 0x7F0), (2, 5)):
        out.append(_sc("babble", [f"{N[n]}:babble:start=100,n={k},id={bid}"], "ignore", frames=FRAMES,
                       tag=dict(node=N[n], n=k, id=bid, alarm_expected=k >= 3)))
    return out


def seqgap() -> list[Scenario]:
    """Frames carry the number of the SYNC frame they belong to (ADR-018): a node whose number is wrong is a node out of phase."""
    out = []
    for n, start, gap in itertools.product(range(3), (0, 100), (0, 1, 2, 3, 10, 100, 255, 256, 257, 511)):
        eff = gap % 256 != 0
        # a number that stays wrong: isolated within 3 frames (a no-op offset, a multiple of 256, is the right number)
        out.append(_sc("seqgap", [f"{N[n]}:seqgap:start={start},gap={gap}"], "detect" if eff else "ignore", 3, frames=FRAMES,
                       tag=dict(node=N[n], gap=gap, persistent=True)))
        # one frame with a wrong number: tolerated (3-of-5), seen once on each of the node's three streams
        # (at frame 0 a number one behind cannot be told from a late frame of a cycle that never existed: nothing to contradict it)
        seen = eff and not (start == 0 and gap % 256 == 255)
        out.append(_sc("seqgap", [f"{N[n]}:seqgap:start={start},end={start + 1},gap={gap}"], "ignore", frames=FRAMES,
                       tag=dict(node=N[n], gap=gap, seq_bad_expected=3 if seen else 0)))
    for n in range(3):  # a damaged frame fills its slot: the clean copy of it that turns up a cycle later is a repeat, not a late frame
        out.append(_sc("seqgap", [f"{N[n]}:corrupt:start=100,end=101,p=1.0", f"{N[n]}:replay:start=101,end=102,age=1"], "ignore", frames=FRAMES,
                       tag=dict(node=N[n], kind="damaged-then-stale", seq_bad_expected=3)))
    return out


def reboot() -> list[Scenario]:
    out = []
    for n, down in itertools.product(range(3), (1, 2, 3, 4, 5, 10, 20, 31, 32, 33, 34, 40, 50, 100, 300)):
        for ctx in _ctxs(n):
            # a node that resyncs from SYNC comes back in phase: only the missing frames count (3-of-5 needs three of them)
            exp = "detect" if down >= 3 else "ignore"
            out.append(_sc("reboot", [f"{N[n]}:reboot:start=100,down={down}"], _dx(exp, ctx), 2, context=ctx, frames=FRAMES,
                           tag=dict(node=N[n], down=down, resync=1)))
            # a node that does not resync restarts its frame number at 0: every frame is the wrong frame, for good
            out.append(_sc("reboot", [f"{N[n]}:reboot:start=100,down={down},resync=0"], _dx("detect", ctx), 3 if down < 3 else 2, context=ctx,
                           frames=FRAMES, tag=dict(node=N[n], down=down, resync=0)))
    return out


def late() -> list[Scenario]:
    out = []
    for n in range(3):
        cmd_thr, accel_thr, gyro_thr = 2000 - 300 * n, 4700 - 200 * n, 5500 - 200 * n
        us_values = {100, 500, 1000, 1500, 3000, 4000, 6000, 8000, 9000}
        for thr in (cmd_thr, accel_thr, gyro_thr):
            us_values |= {thr - 1, thr, thr + 1}
        for us in sorted(v for v in us_values if 0 < v <= 9000):
            is_late = us >= cmd_thr or us >= accel_thr or us >= gyro_thr
            for ctx in _ctxs(n):
                out.append(_sc("late", [f"{N[n]}:late:start=100,us={us}"], _dx("detect" if is_late else "ignore", ctx), 6, context=ctx,
                               frames=FRAMES, tag=dict(node=N[n], us=us, cmd_thr=cmd_thr)))
    return out


def intermittent() -> list[Scenario]:
    out = []
    kinds = {
        "dropout": "{n}:dropout:start=100,period={p},duty={d}",
        "bias": "{n}:bias:start=100,mag=3.0,period={p},duty={d}",
        "digest": "{n}:digest:start=100,xor=1,period={p},duty={d}",
        "cmd_offset": "{n}:cmd_offset:start=100,mag=1.0,period={p},duty={d}",
        "saturate": "{n}:saturate:start=100,period={p},duty={d}",
        "reboot-less-stuck": "{n}:stuck:start=100,period={p},duty={d}",
    }
    for kind, tmpl in kinds.items():
        for n, (per, duty) in itertools.product(range(3), PERIODS):
            exp, lat = _frac_expect(per, duty)
            if kind.endswith("stuck"):
                exp, lat = "gray", None  # a stuck sensor that unfreezes is judged by the voter, not predictable here
            out.append(_sc("intermittent", [tmpl.format(n=N[n], p=per, d=duty)], exp, lat or None, frames=700,
                           tag=dict(kind=kind, node=N[n], period=per, duty=duty)))
    return out


# ------------------------------------------------------------------ recovery, commands, interactions
def commands_transient() -> list[Scenario]:
    """A 30-frame bias on a node, then `reintegrate` at many moments. Latched at 102 by the vote (a transient-looking cause): the dwell is
    50 frames and ends at 152."""
    out = []
    for n, at in itertools.product(range(3), (0, 50, 101, 102, 103, 105, 150, 250, 301, 302, 303, 350, 400, 600)):
        spec = f"{N[n]}:bias:start=100,end=130"
        latch, dwell_end = 102, 152
        if at <= latch:
            readmit = None  # the node is not latched yet: the command is refused and nothing else asks
        else:
            readmit = max(at, dwell_end) + 100
        out.append(_sc("commands_transient", [spec], "any", commands=[f"{at}:reintegrate:{N[n]}"], frames=900,
                       tag=dict(node=N[n], at=at, readmit_at=readmit, final_state="healthy" if readmit and readmit < 900 else "latched")))
    return out


def commands_persistent() -> list[Scenario]:
    out = []
    for n, at in itertools.product(range(3), (150, 302, 400, 600)):
        out.append(_sc("commands_persistent", [f"{N[n]}:bias:start=100"], "detect", 3, commands=[f"{at}:reintegrate:{N[n]}"], frames=900,
                       tag=dict(node=N[n], at=at, final_state="latched", reintegrations_expected=0, strikes_expected=1,
                                probation_failures_min=1 if at >= 302 else 0)))
    return out


def commands_strikes() -> list[Scenario]:
    """Repeat offenders: strike 3 (or 2 for a physical cause) disables a node; commands after that are refused."""
    out = []
    for n in range(3):
        nm = N[n]
        # three transient windows with an operator command between them: Disabled at the third latch
        faults = [f"{nm}:bias:start=100,end=130", f"{nm}:bias:start=600,end=630", f"{nm}:bias:start=1200,end=1230"]
        out.append(_sc("commands_strikes", faults, "any", commands=[f"450:reintegrate:{nm}", f"700:reintegrate:{nm}", f"1300:reintegrate:{nm}"],
                       frames=1500, tag=dict(node=nm, final_state="disabled", strikes_expected=3, nodes_disabled_expected=1,
                                             reintegrations_expected=2, commands_refused_min=1)))
        # only two transient latches: still alive
        out.append(_sc("commands_strikes", faults[:2], "any", commands=[f"450:reintegrate:{nm}", f"700:reintegrate:{nm}"], frames=1500,
                       tag=dict(node=nm, final_state="healthy", strikes_expected=2, nodes_disabled_expected=0, reintegrations_expected=2)))
        # a stuck sensor (physical) is disabled at the second strike (vehicle at rest: only the stuck detector sees it)
        out.append(_sc("commands_strikes", [f"{nm}:stuck:start=100,end=140", f"{nm}:stuck:start=500,end=540"], "any", rest=True,
                       commands=[f"300:reintegrate:{nm}"], frames=900,
                       tag=dict(node=nm, final_state="disabled", strikes_expected=2, nodes_disabled_expected=1)))
        # intermittent (physical class): readmitted once, latching again disables it
        out.append(_sc("commands_strikes", [f"{nm}:corrupt:start=100,end=200,p=1.0,period=3,duty=1", f"{nm}:corrupt:start=600,p=1.0,period=3,duty=1"],
                       "any", commands=[f"450:reintegrate:{nm}"], frames=1000,
                       tag=dict(node=nm, final_state="disabled", strikes_expected=2, nodes_disabled_expected=1)))
        # disable by operator (Triplex -> Duplex: plain), clear (ARM at 100, EXECUTE at 102), then it can be readmitted through the normal path
        out.append(_sc("commands_strikes", [], "any", commands=[f"50:disable:{nm}", f"100:armed-clear-disabled:{nm}", f"110:reintegrate:{nm}"], frames=700,
                       tag=dict(node=nm, final_state="healthy", strikes_expected=0, readmit_at=402)))  # clear at 102 + dwell 200 + probation 100
    return out


def commands_misc() -> list[Scenario]:
    c = []
    A = lambda *cmds, **kw: c.append(_sc("commands_misc", list(kw.pop("faults", [])), "any", commands=list(cmds), frames=kw.pop("frames", 700), tag=kw))  # noqa: E731
    A("50:reintegrate:B", final_state="healthy", note="reintegrate a healthy node: refused")
    A("50:disable:B", final_state="disabled", note="disable a healthy node in Triplex: plain")
    A("50:disable:B", "60:disable:B", final_state="disabled", note="disable twice")
    A("50:disable:B", "60:reintegrate:B", final_state="disabled", note="reintegrate a disabled node: refused")
    A("50:disable:B", "60:armed-clear-disabled:B", "70:reintegrate:B", final_state="healthy", note="disable, clear (armed), reintegrate")
    A("50:disable:B", "60:clear-disabled:B", "70:reintegrate:B", final_state="disabled", commands_refused_min=2, note="clear-disabled without an ARM is refused")
    A("50:clear-disabled:B", final_state="healthy", note="clear a node that is not disabled: refused")
    A("50:clear-safe", final_state="healthy", note="clear-safe with no Safe request: refused (no ARM)")
    A("0:disable:A", "0:disable:B", final_state="healthy", commands_refused_min=1, note="the second plain disable would leave one voter: refused")
    A("0:disable:A", "0:armed-disable:B", final_state="disabled", note="the same, armed")
    A("50:disable:A", "50:disable:B", "50:disable:C", final_state="healthy", commands_refused_min=2, note="disable all three, plain: only the first gets through")
    A("50:disable:A", "50:armed-disable:B", "60:armed-disable:C", final_state="disabled", critical_commands_expected=1,
      note="disable all three, armed: the last voter is reported loudly")
    A("50:disable:B", "50:disable:B", final_state="disabled", note="same command twice in one frame")
    A("50:reintegrate:B", "50:disable:B", "50:reintegrate:B", final_state="disabled", note="command order inside one frame")
    A("100:reintegrate:B", faults=["B:dropout:start=100,end=120"], final_state="latched", note="command before the latch frame is refused; nobody asks again")
    A("103:reintegrate:B", faults=["B:dropout:start=100,end=120"], final_state="healthy", note="command just after latch")
    for n in range(3):
        A(f"150:reintegrate:{N[n]}", f"150:reintegrate:{N[(n + 1) % 3]}",
          faults=[f"{N[n]}:dropout:start=100,end=120", f"{N[(n + 1) % 3]}:dropout:start=100,end=120"], frames=900,
          note="two latched nodes both asked: one probation at a time", final_state="healthy")
    A("160:reintegrate:B", faults=["B:bias:start=100,end=130", "A:dropout:start=100", "C:dropout:start=100"], frames=900,
      note="B alone cannot be judged, so its bias is never detected: the command finds B healthy", final_state="healthy")
    for at in (302, 330, 400):
        A(f"{at}:disable:B", "310:reintegrate:B", faults=["B:bias:start=100,end=130"], frames=900, note="disable while latched / during probation")
    A("400:disable:B", "300:reintegrate:B", faults=["B:bias:start=100,end=130"], frames=900, note="disable while on probation", final_state="disabled")
    A("350:armed-clear-safe", faults=["C:dropout:start=5", "B:digest:start=100"], frames=700, note="clear-safe while the disagreement persists: re-raised")
    A("250:armed-clear-safe", faults=["C:dropout:start=5", "B:digest:start=100,end=200"], frames=700, note="clear-safe after the cause is gone")
    A("250:clear-safe", faults=["C:dropout:start=5", "B:digest:start=100,end=200"], frames=700, final_mode="safe", note="the same without an ARM: stays Safe")
    A("1:clear-safe", "2:clear-safe", "3:clear-safe", note="clear-safe spam")
    return c


def ground_security() -> list[Scenario]:
    """Authentication, replay protection, ARM/EXECUTE and the interlock tiers (ADR-019), through the real core."""
    out = []
    for n in range(3):
        nm, o, p = N[n], N[(n + 1) % 3], N[(n + 2) % 3]
        # a frame with a wrong tag never acts, never reaches the queue, and never moves the counter
        out.append(_sc("ground_security", [], "any", commands=[f"100:forged-disable:{nm}", "110:forged-armed-clear-safe", f"120:reintegrate:{o}"], frames=300,
                       tag=dict(node=nm, final_state="healthy", commands_unauthentic_expected=3, commands_replayed_expected=0)))
        # a replay is refused; the original acts once
        out.append(_sc("ground_security", [], "any", commands=[f"100:disable:{nm}", "110:replay", "120:replay"], frames=300,
                       tag=dict(node=nm, final_state="disabled", commands_replayed_expected=2, commands_accepted_expected=1)))
        # ARM/EXECUTE mismatch (another node), expiry, and the one-shot nature of an ARM
        out.append(_sc("ground_security", [], "any", commands=[f"50:disable:{nm}", f"100:arm-clear-disabled:{o}", f"101:clear-disabled:{nm}"], frames=300,
                       tag=dict(node=nm, final_state="disabled", commands_refused_min=1)))
        out.append(_sc("ground_security", [], "any", commands=[f"50:disable:{nm}", f"100:arm-clear-disabled:{nm}", f"400:clear-disabled:{nm}"], frames=600,
                       tag=dict(node=nm, final_state="disabled", arms_expired_expected=1)))
        out.append(_sc("ground_security", [], "any", commands=[f"50:disable:{nm}", f"100:arm-clear-disabled:{nm}", f"101:clear-disabled:{nm}", f"150:disable:{nm}",
                                                              f"160:clear-disabled:{nm}"], frames=300,
                       tag=dict(node=nm, final_state="disabled", commands_refused_min=1)))
        # the ARM window to the frame: valid for 250 frames (an EXECUTE at +249 acts, at +250 the ARM has expired)
        out.append(_sc("ground_security", [], "any", commands=[f"50:disable:{nm}", f"100:arm-clear-disabled:{nm}", f"349:clear-disabled:{nm}"], frames=400,
                       tag=dict(node=nm, final_state="latched", arms_expired_expected=0)))
        out.append(_sc("ground_security", [], "any", commands=[f"50:disable:{nm}", f"100:arm-clear-disabled:{nm}", f"350:clear-disabled:{nm}"], frames=400,
                       tag=dict(node=nm, final_state="disabled", arms_expired_expected=1)))
        # the interlock tiers: Duplex -> Simplex needs an ARM (the third node is dropped), the last voter needs one and is reported loudly
        out.append(_sc("ground_security", [f"{p}:dropout:start=5"], "any", commands=[f"100:disable:{nm}"], frames=300,
                       tag=dict(node=nm, final_state="healthy", final_mode="duplex", commands_refused_min=1)))
        out.append(_sc("ground_security", [f"{p}:dropout:start=5"], "any", commands=[f"100:armed-disable:{nm}"], frames=300,
                       tag=dict(node=nm, final_state="disabled", final_mode="simplex", critical_commands_expected=0)))
        out.append(_sc("ground_security", [f"{o}:dropout:start=5", f"{p}:dropout:start=5"], "any", commands=[f"200:armed-disable:{nm}"], frames=400,
                       tag=dict(node=nm, final_state="disabled", final_mode="safe", critical_commands_expected=1)))
        out.append(_sc("ground_security", [f"{o}:dropout:start=5", f"{p}:dropout:start=5"], "any", commands=[f"200:disable:{nm}"], frames=400,
                       tag=dict(node=nm, final_state="healthy", commands_refused_min=1, critical_commands_expected=0)))
    return out


def pairs() -> list[Scenario]:
    """Two faulty nodes at once: the always-true safety properties must hold; outcomes are recorded for the report."""
    out = []
    single = {
        "bias+": "{n}:bias:start={s},mag=3.0", "bias-": "{n}:bias:start={s},mag=-3.0", "bias-small": "{n}:bias:start={s},mag=0.6",
        "drift": "{n}:drift:start={s},rate=0.05", "dropout": "{n}:dropout:start={s}", "stuck": "{n}:stuck:start={s}",
        "digest": "{n}:digest:start={s},xor=1", "cmd": "{n}:cmd_offset:start={s},mag=1.0", "saturate": "{n}:saturate:start={s}",
        "corrupt": "{n}:corrupt:start={s},p=0.5", "late": "{n}:late:start={s},us=4000", "reboot": "{n}:reboot:start={s},down=30",
    }
    names = list(single)
    for (a_name, b_name), (na, nb), (sa, sb) in itertools.product(itertools.product(names, repeat=2), ((0, 1), (1, 2), (2, 0)), ((100, 100), (100, 102), (100, 150))):
        if a_name > b_name:
            continue
        faults = [single[a_name].format(n=N[na], s=sa), single[b_name].format(n=N[nb], s=sb)]
        out.append(_sc("pairs", faults, "any", frames=450, tag=dict(a=a_name, b=b_name, na=N[na], nb=N[nb], sa=sa, sb=sb)))
    return out


def correlated() -> list[Scenario]:
    """Same fault on two nodes (a common-cause fault): the voter cannot be right; record what it does."""
    out = []
    for (n1, n2), mag, sensor, start2 in itertools.product(((0, 1), (1, 2), (0, 2)), (0.5, 0.9, 1.5, 3.0, -3.0, 30.0), ("gyro", "accel"), (100, 101, 110)):
        tol = GYRO_TOL if sensor == "gyro" else ACCEL_TOL
        faults = [f"{N[n1]}:bias:start=100,sensor={sensor},mag={mag * tol:g}", f"{N[n2]}:bias:start={start2},sensor={sensor},mag={mag * tol:g}"]
        out.append(_sc("correlated", faults, "any", frames=400, tag=dict(mag=mag, sensor=sensor, nodes=N[n1] + N[n2], start2=start2)))
    return out


def startup_edges() -> list[Scenario]:
    out = []
    for nodes in ("A,B,C", "B,C", "A,C", "A,B", "A", "B", "C"):
        for frames in (1, 2, 3, 4, 5, 6, 10, 50):
            out.append(_sc("startup_edges", [], "any", nodes=nodes, frames=frames, tag=dict(nodes=nodes)))
    for start in (0, 1, 2, 3):
        for kind in ("bias:mag=3.0", "dropout", "stuck", "digest:xor=1", "cmd_offset:mag=1.0", "saturate", "corrupt:p=1.0", "reboot:down=10"):
            for n in range(3):
                out.append(_sc("startup_edges", [f"{N[n]}:{kind.split(':')[0]}:start={start}" + ("," + kind.split(":", 1)[1] if ":" in kind else "")],
                               "any", frames=60, tag=dict(start=start, kind=kind, node=N[n])))
    return out


def contexts() -> list[Scenario]:
    """The same fault on the *last survivor* (Simplex): the voter cannot out-vote it; only frame-level detectors work."""
    out = []
    specs = ["{n}:bias:start=100,mag=3.0", "{n}:dropout:start=100", "{n}:stuck:start=100", "{n}:digest:start=100,xor=1",
             "{n}:cmd_offset:start=100,mag=1.0", "{n}:saturate:start=100", "{n}:corrupt:start=100,p=1.0", "{n}:reboot:start=100,down=30",
             "{n}:drift:start=100,rate=0.05", "{n}:spike:start=100,mag=20,p=1.0", "{n}:late:start=100,us=4000", "{n}:seqgap:start=100,gap=3"]
    for n, spec in itertools.product(range(3), specs):
        others = "".join(N[(n + 1) % 3] + N[(n + 2) % 3])
        out.append(_sc("contexts", [spec.format(n=N[n])], "any", context=f"simplex-{others}", frames=400, tag=dict(node=N[n], spec=spec)))
        for rest in (False, True):
            out.append(_sc("contexts", [spec.format(n=N[n])], "any", context=f"duplex-{N[(n + 1) % 3]}", frames=400, rest=rest,
                           tag=dict(node=N[n], spec=spec, rest=rest)))
    return out


# ------------------------------------------------------------------ kinds added by the FMEA gap analysis (docs/FMEA.md)
GYRO_AMP = (10.0, 6.0, 3.0)  # peak truth rate per axis, dps (peers.truth)
ACCEL_AMP = (0.05, 0.03, 0.0)  # peak variation per axis, g (axis 2 is constant gravity 1.0 g)
GYRO_RATE = tuple(a * 2 * math.pi * f for a, f in zip(GYRO_AMP, (0.8, 0.5, 0.3)))  # dps per second at the steepest point


def _tol(sensor: str) -> float:
    return GYRO_TOL if sensor == "gyro" else ACCEL_TOL


def _amp(sensor: str, axis: int) -> float:
    return GYRO_AMP[axis] if sensor == "gyro" else (ACCEL_AMP[axis] if axis < 2 else 1.0)


def _by_peak(peak_rel: float) -> str:
    """Expectation from the worst-case error in units of the vote tolerance (as in the bias grid)."""
    return "detect" if peak_rel >= 1.5 else ("ignore" if peak_rel <= 0.5 else "gray")


def _new_each_context(group, spec_fn, expect, lat, *, duplex=True, rest=(False,), frames=FRAMES, **tag):
    out = []
    for n in range(3):
        for ctx in _ctxs(n, duplex):
            for r in rest:
                out.append(_sc(group, [spec_fn(N[n])], _dx(expect, ctx), lat, context=ctx, rest=r, frames=frames,
                               tag=dict(node=N[n], rest=r, **tag)))
    return out


def new_sensor_faults() -> list[Scenario]:
    out = []
    for sensor, axis, f in itertools.product(("gyro", "accel"), (0, 1, 2), (-1.0, 0.0, 0.5, 0.9, 0.95, 0.98, 0.99, 1.01, 1.02, 1.05, 1.1, 1.2, 2.0, 10.0)):
        peak = abs(f - 1.0) * _amp(sensor, axis) / _tol(sensor)
        out += _new_each_context("scale", lambda nd, a=axis, fa=f, se=sensor: f"{nd}:scale:start=100,sensor={se},axis={a},factor={fa}",
                                 _by_peak(peak), None, duplex=(axis == 0 and sensor == "gyro") or (sensor == "accel" and axis == 2),
                                 sensor=sensor, axis=axis, factor=f)
    for sensor, m in itertools.product(("gyro", "accel"), (1.0, 2.0, 3.0, 5.0, 8.0, 10.0, 15.0, 20.0, 50.0, 200.0)):
        exp = "detect" if m >= 20 else ("ignore" if m <= 3 else "gray")  # sigma*mult against tol = 10 sigma
        out += _new_each_context("noise", lambda nd, se=sensor, mm=m: f"{nd}:noise:start=100,sensor={se},mult={mm}", exp, None, sensor=sensor, mult=m)
    for sensor, axis in itertools.product(("gyro", "accel"), (0, 1, 2)):
        out += _new_each_context("invert", lambda nd, a=axis, se=sensor: f"{nd}:invert:start=100,sensor={se},axis={a}",
                                 "detect", 3 if (sensor == "accel" and axis == 2) else None, duplex=(axis == 0), sensor=sensor, axis=axis)
        for other in range(3):
            if other == axis:
                continue
            out += _new_each_context("swap", lambda nd, a=axis, o=other, se=sensor: f"{nd}:swap:start=100,sensor={se},axis={a},other={o}",
                                     "detect", None, duplex=False, sensor=sensor, axis=axis, other=other)
    out += _new_each_context("zero", lambda nd: f"{nd}:zero:start=100", "detect", 3, rest=(False, True))  # gravity is zeroed even at rest
    for sensor, lim in itertools.product(("gyro", "accel"), (0.5, 0.9, 0.95, 0.97, 0.99, 1.0, 1.05, 2.0, 4.0, 8.0, 9.5, 10.5, 12.0)):
        peak = (10.0 - lim if sensor == "gyro" else 1.0 - lim) / _tol(sensor)
        exp = "detect" if peak >= 1.5 else ("ignore" if peak <= 0.5 else "gray")
        if sensor == "accel" and lim >= 1.0:
            exp = "ignore"
        out += _new_each_context("clip", lambda nd, se=sensor, L=lim: f"{nd}:clip:start=100,sensor={se},limit={L}", exp, None, duplex=False, sensor=sensor, limit=lim)
    for sensor, axis, amp, hz in itertools.product(("gyro", "accel"), (0, 2), (0.01, 0.3, 1.0, 3.0, 30.0), (1.0, 5.0, 25.0, 33.0, 49.0, 50.0, 51.0, 99.0, 100.0, 101.0, 250.0)):
        tol = _tol(sensor)
        a = amp * (tol if sensor == "accel" else 1.0)
        err = [a * math.sin(2 * math.pi * hz * (k * 0.01 + 0.0005)) for k in range(100, 400)]
        frac = sum(abs(e) > 1.2 * tol for e in err) / len(err)
        peak = max(abs(e) for e in err)
        exp = "detect" if frac >= 0.4 else ("ignore" if peak < 0.8 * tol else "gray")
        out += _new_each_context("oscillate", lambda nd, se=sensor, ax=axis, A=a, h=hz: f"{nd}:oscillate:start=100,sensor={se},axis={ax},amp={A:g},hz={h}",
                                 exp, None, duplex=False, sensor=sensor, axis=axis, amp=a, hz=hz, frac=round(frac, 2))
    for n_hold in (2, 3, 4, 5, 8, 12, 19, 20, 21, 22, 30, 60):
        exp = "ignore" if n_hold == 2 else ("detect" if n_hold >= 12 else "gray")
        out += _new_each_context("repeat", lambda nd, nh=n_hold: f"{nd}:repeat:start=100,n={nh}", exp, None, duplex=False, n=n_hold)
    return out


def new_bit_faults() -> list[Scenario]:
    out = []
    for sensor, bit, p, seed in itertools.product(("gyro", "accel"), range(16), (0.02, 0.1, 0.5, 1.0), (1, 2)):
        lsb_rel = (2 ** bit) * (0.125 if sensor == "gyro" else 1 / 2048) / _tol(sensor)
        if lsb_rel <= 0.5:
            exp = "ignore"
        elif lsb_rel < 1.5:
            exp = "gray"
        else:
            exp = "detect" if p >= 0.7 else ("ignore" if p <= 0.02 else "gray")
        out.append(_sc("bitflip", [f"B:bitflip:start=100,sensor={sensor},bit={bit},p={p}"], exp, None, seed=seed, frames=FRAMES,
                       tag=dict(node="B", sensor=sensor, bit=bit, p=p)))
    for sensor, bit, value, n in itertools.product(("gyro", "accel"), range(16), (0, 1), range(3)):
        lsb_rel = (2 ** bit) * (0.125 if sensor == "gyro" else 1 / 2048) / _tol(sensor)
        exp = "ignore" if lsb_rel <= 0.5 else ("detect" if lsb_rel >= 16 else "gray")
        out.append(_sc("stuckbit", [f"{N[n]}:stuckbit:start=100,sensor={sensor},bit={bit},value={value}"], exp, None, frames=FRAMES,
                       tag=dict(node=N[n], sensor=sensor, bit=bit, value=value)))
    return out


def new_command_faults() -> list[Scenario]:
    out = []
    out += _new_each_context("cmdstuck", lambda nd: f"{nd}:cmdstuck:start=100", "detect", 6)
    out += _new_each_context("cmdstuck", lambda nd: f"{nd}:cmdstuck:start=100", "ignore", None, duplex=False, rest=(True,))
    out += _new_each_context("cmdinvert", lambda nd: f"{nd}:cmdinvert:start=100", "detect", 6)
    out += _new_each_context("cmdinvert", lambda nd: f"{nd}:cmdinvert:start=100", "ignore", None, duplex=False, rest=(True,))
    return out


def new_frame_faults() -> list[Scenario]:
    out = []
    for mask in range(1, 8):
        out += _new_each_context("partial", lambda nd, m=mask: f"{nd}:partial:start=100,mask={m}", "detect", 6, mask=mask)
    for gap in (0, 1, 100, 300, 1000, 3000, 6000, 9000):
        out += _new_each_context("duplicate", lambda nd, g=gap: f"{nd}:duplicate:start=100,gap_us={g}", "detect", 3, duplex=False, gap=gap)
    for age in (1, 2, 3, 5, 10, 50, 100, 255, 256, 257, 300):
        out += _new_each_context("replay", lambda nd, a=age: f"{nd}:replay:start=300,age={a}", "detect", 6, age=age)
    out += _new_each_context("seqstuck", lambda nd: f"{nd}:seqstuck:start=100", "detect", 6)
    return out


def new_timing_faults() -> list[Scenario]:
    out = []
    for n in range(3):
        thr = 4500 + 200 * n  # the gyro frame crosses into the previous frame's window before its 7 ms vote
        for us in sorted({100, 500, 1000, 2000, 3000, 4000, thr - 1, thr, thr + 1, 6000, 8000, 9000}):
            # Past `thr` the gyro/accel frames land in the previous frame's window carrying the NEXT frame's number: detected since ADR-018
            # (E11). Below it the frame still arrives in the right window with the right number: invisible until arrival times are
            # checked (docs/DEFERRED.md).
            exp = "gray" if abs(us - thr) <= 1 else ("detect" if us > thr else "ignore")
            for ctx in _ctxs(n):
                out.append(_sc("early", [f"{N[n]}:early:start=100,us={us}"], exp, 6, context=ctx, frames=FRAMES,
                               tag=dict(node=N[n], us=us, thr=thr)))
    for n in range(3):
        cmd_thr = 2000 - 300 * n
        for us, seed in itertools.product((50, 200, 500, 1000, 1300, 1500, 2000, 3000, 4000, 4500, 5000, 8000), (1, 2, 3)):
            exp = "ignore" if us < cmd_thr else ("detect" if us >= 4000 else "gray")
            out.append(_sc("jitter", [f"{N[n]}:jitter:start=100,us={us}"], exp, None, seed=seed, frames=FRAMES, tag=dict(node=N[n], us=us)))
    for n in range(3):
        cmd_thr = 2000 - 300 * n
        for d in (-1000, -200, -50, -20, -10, -5, -1, 1, 2, 5, 10, 20, 30, 50, 100, 200, 1000):
            span = 600 * abs(d)
            if d > 0:
                exp = "detect" if span >= cmd_thr + 1000 else ("ignore" if span < cmd_thr - 100 else "gray")
            else:
                exp = "detect" if span >= 5500 else ("ignore" if span < 4400 else "gray")  # beyond the gyro threshold the number gives it away
            out.append(_sc("clockdrift", [f"{N[n]}:clockdrift:start=100,us_per_frame={d}"], exp, None, frames=700, tag=dict(node=N[n], d=d)))
    return out


NEW_KIND_PERIODIC = {
    "zero": "{n}:zero:start=100,period={p},duty={d}",
    "partial": "{n}:partial:start=100,mask=7,period={p},duty={d}",
    "cmdinvert": "{n}:cmdinvert:start=100,period={p},duty={d}",
    "invert-accel-z": "{n}:invert:start=100,sensor=accel,axis=2,period={p},duty={d}",
    "scale-accel-z": "{n}:scale:start=100,sensor=accel,axis=2,factor=1.5,period={p},duty={d}",
    "replay": "{n}:replay:start=300,age=5,period={p},duty={d}",
}


def new_intermittent() -> list[Scenario]:
    out = []
    for kind, tmpl in NEW_KIND_PERIODIC.items():
        for n, (per, duty) in itertools.product(range(3), PERIODS):
            exp, lat = _frac_expect(per, duty)
            if kind == "replay":
                exp, lat = "gray", None
            out.append(_sc("new_intermittent", [tmpl.format(n=N[n], p=per, d=duty)], exp, lat or None, frames=800,
                           tag=dict(kind=kind, node=N[n], period=per, duty=duty)))
    return out


def new_pairs() -> list[Scenario]:
    """Each new kind on one node with a classic fault on another: the always-true safety properties must hold."""
    news = {
        "scale": "{n}:scale:start={s},factor=1.5", "noise": "{n}:noise:start={s},mult=30", "invert": "{n}:invert:start={s}",
        "swap": "{n}:swap:start={s}", "zero": "{n}:zero:start={s}", "clip": "{n}:clip:start={s},limit=4", "oscillate": "{n}:oscillate:start={s},amp=3,hz=33",
        "repeat": "{n}:repeat:start={s},n=10", "bitflip": "{n}:bitflip:start={s},bit=9,p=0.5", "stuckbit": "{n}:stuckbit:start={s},bit=9,value=1",
        "cmdstuck": "{n}:cmdstuck:start={s}", "cmdinvert": "{n}:cmdinvert:start={s}", "partial": "{n}:partial:start={s},mask=3",
        "duplicate": "{n}:duplicate:start={s}", "replay": "{n}:replay:start={s},age=7", "seqstuck": "{n}:seqstuck:start={s}",
        "early": "{n}:early:start={s},us=6000", "jitter": "{n}:jitter:start={s},us=3000", "clockdrift": "{n}:clockdrift:start={s},us_per_frame=40",
    }
    olds = {"bias": "{n}:bias:start={s},mag=3.0", "dropout": "{n}:dropout:start={s}", "digest": "{n}:digest:start={s},xor=1",
            "cmd": "{n}:cmd_offset:start={s},mag=1.0", "late": "{n}:late:start={s},us=4000", "stuck": "{n}:stuck:start={s}"}
    out = []
    for (kn, kt), (on, ot), (na, nb), (sa, sb) in itertools.product(news.items(), olds.items(), ((0, 1), (2, 0)), ((100, 100), (100, 103), (100, 160))):
        out.append(_sc("new_pairs", [kt.format(n=N[na], s=sa), ot.format(n=N[nb], s=sb)], "any", frames=450,
                       tag=dict(a=kn, b=on, na=N[na], nb=N[nb], sa=sa, sb=sb)))
    for (k1, t1), (k2, t2) in itertools.combinations(news.items(), 2):  # two new kinds on two nodes
        out.append(_sc("new_pairs", [t1.format(n="A", s=100), t2.format(n="C", s=104)], "any", frames=450, tag=dict(a=k1, b=k2, na="A", nb="C", sa=100, sb=104)))
    return out


# ------------------------------------------------------------------ cascades, total loss, long runs
def cascades() -> list[Scenario]:
    """A first fault is handled and recovery begins; then a second fault hits the reference nodes, the probationer,
    or the readmitted node. Only the always-true safety properties are judged; outcomes are recorded."""
    out = []
    second = {"bias": "{n}:bias:start={s},mag=3.0", "dropout": "{n}:dropout:start={s}", "digest": "{n}:digest:start={s},xor=1",
              "cmd": "{n}:cmd_offset:start={s},mag=1.0", "stuck": "{n}:stuck:start={s}", "late": "{n}:late:start={s},us=4000",
              "bias-short": "{n}:bias:start={s},end={e},mag=3.0"}
    # B is latched at 102 by a 30-frame bias; the operator asks at 302; probation runs 303..402, readmission at 402
    for (kind, tmpl), nsec, s in itertools.product(second.items(), ("A", "C", "B"), (150, 250, 302, 303, 310, 350, 401, 402, 403, 450, 600)):
        faults = ["B:bias:start=100,end=130", tmpl.format(n=nsec, s=s, e=s + 20)]
        out.append(_sc("cascades", faults, "any", commands=["302:reintegrate:B", "700:reintegrate:B", "700:reintegrate:A", "700:reintegrate:C"],
                       frames=1100, tag=dict(first="B", second=nsec, kind=kind, s=s)))
    return out


def total_loss() -> list[Scenario]:
    """Every node judged bad at once (a bus-wide outage plus the last survivor failing): is there a way back?"""
    out = []
    for dur, at in itertools.product((2, 3, 4, 10, 50), (400, 800)):
        faults = [f"{c}:dropout:start=100,end={100 + dur}" for c in N]
        cmds = [f"{at}:reintegrate:{c}" for c in N]
        out.append(_sc("total_loss", faults, "any", commands=cmds, frames=at + 600,
                       tag=dict(kind="all-dropout", dur=dur, at=at, node="B", final_state="healthy")))
    for at in (400, 800):
        faults = ["A:dropout:start=100,end=120", "B:dropout:start=100,end=120", "C:dropout:start=150,end=170"]
        cmds = [f"{at}:reintegrate:{c}" for c in N]
        out.append(_sc("total_loss", faults, "any", commands=cmds, frames=at + 600,
                       tag=dict(kind="last-survivor-fails", at=at, node="B", final_state="healthy")))
    # the cohort refuses a member that is still wrong (B returns biased); the other two come back
    out.append(_sc("total_loss", [f"{c}:dropout:start=100,end=104" for c in N] + ["B:bias:start=104"], "any",
                   commands=[f"400:reintegrate:{c}" for c in N], frames=900,
                   tag=dict(kind="cohort-refuses-the-odd-one", node="B", final_state="latched", probation_failures_min=1, reintegrations_expected=2)))
    # two candidates that disagree cannot be judged: nothing is readmitted until they agree (B is fixed at frame 400), then 100 clean frames
    out.append(_sc("total_loss", [f"{c}:dropout:start=100,end=104" for c in N] + ["B:bias:start=104,end=400"], "any",
                   commands=["120:reintegrate:A", "120:reintegrate:B"], frames=700,
                   tag=dict(kind="disagreeing-candidates", node="A", readmit_at=499, final_state="healthy", probation_failures_min=0)))
    for at in (400, 800):  # the same, but the commands come one node at a time, as an operator would
        faults = [f"{c}:dropout:start=100,end=104" for c in N]
        cmds = [f"{at}:reintegrate:A", f"{at + 150}:reintegrate:B", f"{at + 300}:reintegrate:C"]
        out.append(_sc("total_loss", faults, "any", commands=cmds, frames=at + 900,
                       tag=dict(kind="sequential-requests", at=at, node="C", final_state="healthy")))
    return out


def long_run() -> list[Scenario]:
    """Ten minutes of flight: no false positives from healthy traffic, sparse glitches left alone, no counter trouble."""
    out = [_sc("long_run", [], "ignore", frames=60000, tag=dict(kind="healthy"))]
    out.append(_sc("long_run", ["B:corrupt:start=100,p=1.0,period=50,duty=1"], "ignore", frames=60000, tag=dict(kind="1-in-50 corrupt")))
    out.append(_sc("long_run", ["C:spike:start=0,mag=20,p=0.01"], "ignore", frames=60000, tag=dict(kind="1% spikes")))
    out.append(_sc("long_run", [f"{c}:late:start={s},end={s + 1},us=4000" for c, s in (("A", 5000), ("B", 20000), ("C", 40000))], "ignore",
                   frames=60000, tag=dict(kind="isolated late frames")))
    return out


# ------------------------------------------------------------------ phase sweep
PHASE_KINDS = {
    "dropout": "{n}:dropout:start={s}", "stuck": "{n}:stuck:start={s}", "bias": "{n}:bias:start={s},mag=3.0",
    "drift": "{n}:drift:start={s},rate=0.05", "spike": "{n}:spike:start={s},mag=20,p=0.5", "saturate": "{n}:saturate:start={s}",
    "corrupt": "{n}:corrupt:start={s},p=0.5", "cmd_offset": "{n}:cmd_offset:start={s},mag=1.0", "digest": "{n}:digest:start={s},xor=1",
    "babble": "{n}:babble:start={s},n=5", "seqgap": "{n}:seqgap:start={s},gap=3", "reboot": "{n}:reboot:start={s},down=30",
    "late": "{n}:late:start={s},us=4000", "scale": "{n}:scale:start={s},factor=1.5", "noise": "{n}:noise:start={s},mult=30",
    "invert": "{n}:invert:start={s}", "swap": "{n}:swap:start={s}", "zero": "{n}:zero:start={s}", "clip": "{n}:clip:start={s},limit=4",
    "oscillate": "{n}:oscillate:start={s},amp=3,hz=33", "repeat": "{n}:repeat:start={s},n=10", "bitflip": "{n}:bitflip:start={s},bit=9,p=0.5",
    "stuckbit": "{n}:stuckbit:start={s},bit=9,value=1", "cmdstuck": "{n}:cmdstuck:start={s}", "cmdinvert": "{n}:cmdinvert:start={s}",
    "partial": "{n}:partial:start={s},mask=3", "duplicate": "{n}:duplicate:start={s}", "replay": "{n}:replay:start={s},age=3",
    "seqstuck": "{n}:seqstuck:start={s}", "early": "{n}:early:start={s},us=6000", "jitter": "{n}:jitter:start={s},us=3000",
    "clockdrift": "{n}:clockdrift:start={s},us_per_frame=40",
}


def phase_sweep() -> list[Scenario]:
    """Every fault kind started at 40 different instants (the vehicle's motion is sinusoidal: the slope, and with it
    what a stale or frozen value looks like, depends on the moment), in Triplex and in Duplex. A single start frame
    hid the frozen-command defect (E16); this group exists so a phase-dependent behaviour cannot hide again."""
    out = []
    for (kind, tmpl), s, ctx in itertools.product(PHASE_KINDS.items(), range(40, 200, 4), ("triplex", "duplex")):
        node = (s // 4) % 3
        c = "triplex" if ctx == "triplex" else f"duplex-{N[(node + 1) % 3]}"
        out.append(_sc("phase_" + kind, [tmpl.format(n=N[node], s=s)], "any", context=c, frames=s + 80,
                       tag=dict(kind=kind, start=s, node=N[node])))
    return out


def recovery_edges() -> list[Scenario]:
    """The exact timing and bookkeeping of the recovery path, one thing at a time (each pins down a mutant the broader
    groups could not tell from the real code)."""
    out = []
    for n in range(3):
        nm = N[n]
        others = [N[(n + 1) % 3], N[(n + 2) % 3]]
        # the leaky score is cleared on readmission: ONE bad frame right after must not isolate the node again (it would, from
        # the score left at the latch, if it were not cleared). Latched by the leaky count at ~112, probation 450..550.
        out.append(_sc("recovery_edges", [f"{nm}:corrupt:start=100,end=200,p=1.0,period=3,duty=1", f"{nm}:corrupt:start=553,end=554,p=1.0"], "any",
                       commands=[f"450:reintegrate:{nm}"], frames=900,
                       tag=dict(node=nm, readmit_at=550, final_state="healthy", strikes_expected=1, nodes_disabled_expected=0)))
        # frames with no reference (the other two silent for two frames, below the latch threshold) neither advance nor fail a probation
        out.append(_sc("recovery_edges", [f"{nm}:bias:start=100,end=130"] + [f"{o}:dropout:start=320,end=322" for o in others], "any",
                       commands=[f"302:reintegrate:{nm}"], frames=700, tag=dict(node=nm, readmit_at=404, final_state="healthy", probation_failures_min=0)))
        # a failed probation restarts the dwell: probation at 302 fails at 303, the next one may start 50 whole frames later (353, the
        # request at 304 is waiting), then fails at 354; the request at 505 starts the third at once (505)
        out.append(_sc("recovery_edges", [f"{nm}:bias:start=100"], "any", commands=[f"302:reintegrate:{nm}", f"304:reintegrate:{nm}", f"505:reintegrate:{nm}"],
                       frames=800, tag=dict(node=nm, starts_expected=[302, 353, 505], final_state="latched")))
        # clear-disabled really clears the strikes: after three strikes and a maintenance clear, a fresh transient fault is strike 1, not 4
        faults = [f"{nm}:bias:start=100,end=130", f"{nm}:bias:start=600,end=630", f"{nm}:bias:start=1200,end=1230", f"{nm}:bias:start=2400,end=2430"]
        cmds = [f"450:reintegrate:{nm}", f"700:reintegrate:{nm}", f"1300:armed-clear-disabled:{nm}", f"1510:reintegrate:{nm}", f"2700:reintegrate:{nm}"]
        out.append(_sc("recovery_edges", faults, "any", commands=cmds, frames=3000,
                       tag=dict(node=nm, final_state="healthy", strikes_expected=1, nodes_disabled_expected=1)))
    return out


def duplex_boundary() -> list[Scenario]:
    """Where Duplex stops being able to blame a node: a gyro bias of m x tolerance on one of two nodes, 20 seeds each. Gyro channels move less
    than a tolerance per frame, so the decision band is the classic one (about 50% isolated at 2.0x; the motion reference of ADR-017 had
    shifted it to 2.3x, E19). At 2.4x and above every seed must be isolated; below 1.7x none may be."""
    out = []
    for n, m, seed in itertools.product(range(3), (1.0, 1.4, 1.6, 2.4, 2.6, 3.0), range(1, 21)):
        exp = "detect" if m >= 2.4 else "ignore"
        out.append(Scenario(group="duplex_boundary", faults=[f"{N[n]}:bias:start=100,sensor=gyro,axis=0,mag={m * GYRO_TOL:g}"], expect=exp,
                            latency_max=4 if exp == "detect" else None, context=f"duplex-{N[(n + 1) % 3]}", frames=170, seed=seed,
                            tag=dict(node=N[n], rel=m)))
    return out


def all_groups() -> dict:
    return {
        "dropout": dropout, "stuck": stuck, "bias_gyro": lambda: _bias("gyro"), "bias_accel": lambda: _bias("accel"),
        "drift": drift, "spike": spike, "saturate": saturate, "corrupt": corrupt, "corrupt_periodic": corrupt_periodic,
        "cmd_offset": cmd_offset, "digest": digest, "babble": babble, "seqgap": seqgap, "reboot": reboot, "late": late,
        "intermittent": intermittent, "commands_transient": commands_transient, "commands_persistent": commands_persistent,
        "commands_misc": commands_misc, "commands_strikes": commands_strikes, "pairs": pairs, "correlated": correlated, "startup_edges": startup_edges,
        "contexts": contexts, "new_sensor": new_sensor_faults, "new_bits": new_bit_faults, "new_command": new_command_faults,
        "new_frame": new_frame_faults, "new_timing": new_timing_faults, "new_intermittent": new_intermittent, "new_pairs": new_pairs,
        "cascades": cascades, "ground_security": ground_security, "duplex_boundary": duplex_boundary, "total_loss": total_loss, "long_run": long_run, "phase_sweep": phase_sweep, "recovery_edges": recovery_edges,
    }
