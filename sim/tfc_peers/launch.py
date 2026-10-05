# SPDX-License-Identifier: MIT
"""The launch checklist, automated (docs/LAUNCH_SEQUENCE.md section 5 and docs/procedures/P-S2-02-launch-checklist.md).

`Observer` follows the bus (the flight computers' heartbeats, ACT's output, SYNC's mission frame) and says whether it is go or no-go and why. `run` waits for a go, sends the launch
as an ARM and an EXECUTE, then watches the countdown on SYNC's mission frame until T-zero or a scrub. Everything the bus and the clock provide is passed in, so it is tested with scripted frames.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Callable

from . import protocol as P

MAX_AGE_S = 0.5          # a heartbeat or an ACT frame older than this does not count
NODES = ("A", "B", "C")


@dataclass
class Observer:
    ready: list[bool] = field(default_factory=lambda: [False] * 3)
    mode: list[int] = field(default_factory=lambda: [0] * 3)
    safe: list[bool] = field(default_factory=lambda: [False] * 3)
    hb_time: list[float] = field(default_factory=lambda: [-math.inf] * 3)
    act_state: int | None = None
    act_time: float = -math.inf
    mission: int = 0
    mission_time: float = -math.inf

    def feed(self, frame: P.Frame, now: float) -> None:
        if P.ID_HEARTBEAT <= frame.id < P.ID_HEARTBEAT + 3:
            h = P.unpack_heartbeat(frame)
            if h is not None:
                n = frame.id - P.ID_HEARTBEAT
                self.ready[n], self.mode[n], self.safe[n], self.hb_time[n] = h.ready, h.mode, h.safe_requested, now
        elif frame.id == P.ID_ACT_OUT:
            a = P.unpack_act_out(frame)
            if a is not None:
                self.act_state, self.act_time = a.state, now
        elif frame.id == P.ID_SYNC:
            s = P.unpack_sync(frame)
            if s is not None:
                self.mission, self.mission_time = s.mission, now

    def verdict(self, now: float) -> list[str]:
        """The reasons it is a no-go (an empty list means go)."""
        why: list[str] = []
        for n, name in enumerate(NODES):
            if now - self.hb_time[n] > MAX_AGE_S:
                why.append(f"flight computer {name}: no heartbeat")
            elif not self.ready[n]:
                why.append(f"flight computer {name}: not ready (IMU calibration, sensors or attitude)")
            elif self.mode[n] != 3:
                why.append(f"flight computer {name}: not in Triplex (mode {self.mode[n]})")
            if self.safe[n] and now - self.hb_time[n] <= MAX_AGE_S:
                why.append(f"flight computer {name}: a Safe request is up")
        if now - self.act_time > MAX_AGE_S:
            why.append("ACT: no output frame")
        elif self.act_state != 1:
            why.append(f"ACT: not Nominal (state {self.act_state})")
        if now - self.mission_time > MAX_AGE_S:
            why.append("no SYNC")
        return why

    def table(self, now: float) -> str:
        out = []
        for n, name in enumerate(NODES):
            alive = now - self.hb_time[n] <= MAX_AGE_S
            out.append(f"{name}:{'ready' if alive and self.ready[n] else ('wait' if alive else 'none')}")
        act = "none" if now - self.act_time > MAX_AGE_S else ("nominal" if self.act_state == 1 else f"state {self.act_state}")
        return f"{'  '.join(out)}  ACT:{act}  mission frame {self.mission}"


def run(recv: Callable[[float], P.Frame | None], send: Callable[[P.Frame], None], now: Callable[[], float], sleep: Callable[[float], None],
        counter: int, say: Callable[[str], None] = print, wait_s: float = 60.0, check_only: bool = False, countdown_timeout_s: float = 20.0) -> int:
    """Exit code: 0 lift-off, 1 no-go (timed out or --check), 2 scrubbed, 3 the command was not accepted, 4 the countdown never completed."""
    obs = Observer()
    t_end = now() + wait_s
    last_print = -math.inf

    def pump(seconds: float) -> None:
        end = now() + seconds
        while now() < end:
            f = recv(max(0.0, end - now()))
            if f is not None:
                obs.feed(f, now())

    # ---- 1. wait for a go ----
    while True:
        pump(0.1)
        why = obs.verdict(now())
        if now() - last_print >= 1.0:
            say(f"[go/no-go] {obs.table(now())}  ->  {'GO' if not why else 'NO-GO'}")
            last_print = now()
        if not why:
            break
        if check_only or now() > t_end:
            say("NO-GO:")
            for w in why:
                say(f"  - {w}")
            return 1
    if check_only:
        say("GO: every item holds.")
        return 0
    say("GO. Sending the launch command: ARM, then EXECUTE.")
    # ---- 2. the command ----
    send(P.pack_ground(P.GROUND_OPS["launch"], 0, counter & 0xFF, None, True, False))
    sleep(0.05)
    send(P.pack_ground(P.GROUND_OPS["launch"], 0, (counter + 1) & 0xFF, None, False, False))
    # ---- 3. the countdown ----
    t_accept = now() + 3.0
    while obs.mission == 0 and now() < t_accept:
        pump(0.05)
    if obs.mission == 0:
        say("The countdown did not start: the sync master refused (see its console: a no-go reason, or the ARM window passed).")
        return 3
    say("COUNTDOWN started.")
    t_limit = now() + countdown_timeout_s
    last_t = None
    while now() < t_limit:
        pump(0.05)
        m = obs.mission
        if m == 0:
            say("SCRUBBED: the mission frame went back to zero.")
            return 2
        if P.mission_in_flight(m):
            say("T-ZERO: lift-off.")
            return 0
        t_minus = (P.mission_frames_to_zero(m) + 99) // 100
        if t_minus != last_t:
            say(f"T-{t_minus} s")
            last_t = t_minus
    say("The countdown did not complete in time.")
    return 4
