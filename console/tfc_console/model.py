# SPDX-License-Identifier: MIT
"""What the console knows about the rig: every frame on the bus, decoded and folded into one state.

`Telemetry.feed(t, frame)` takes a frame with its time (a monotonic clock live, the log's clock in a replay) and keeps the latest of everything: the three flight computers' heartbeats and samples,
ACT's output, SYNC's frame and mission frame, the simulator's status, the operator's commands. `snapshot(now)` is the state as one JSON-able dict; `metrics(now)` is the few numbers a chart plots.

Two rules govern what is here. (1) **The console decides nothing about the flight.** The state of a node (healthy, latched, probation, disabled) is what the flight computers *say* in their heartbeats, and
the vote status is what ACT says; the console's own arithmetic (the deviation of each node from the median, the go/no-go) is labelled as the console's reconstruction, and the go/no-go uses the rule of the
launch checklist (`tfc_peers/launch.py`), which a test holds it to. (2) **A silent source is shown as silent**, never as its last value: every number has the time of its last frame, and `snapshot` says which are stale.
"""
from __future__ import annotations

import math
import statistics
from collections import deque
from dataclasses import dataclass, field
from typing import Callable

from tfc_peers import protocol as P

from . import constants as K

EVENT_LEVELS = ("info", "ok", "warn", "crit")


@dataclass
class IdStat:
    count: int = 0
    crc_bad: int = 0
    first_t: float = 0.0
    last_t: float = 0.0
    mark_count: int = 0      # for the rate: the count and the time at the last mark
    mark_t: float = 0.0
    rate: float = 0.0
    last_hex: str = ""
    last_text: str = ""


@dataclass
class Node:
    name: str
    hb: P.Heartbeat | None = None
    hb_t: float = -math.inf
    hb_count: int = 0
    sample_t: dict[str, float] = field(default_factory=lambda: {"gyro": -math.inf, "accel": -math.inf, "cmd": -math.inf})
    values: dict[str, tuple] = field(default_factory=dict)         # the latest gyro, accel, cmd of this node
    digest: int | None = None
    last_seq: dict[str, int] = field(default_factory=dict)
    seq_gaps: int = 0
    crc_bad: int = 0
    share: P.StateShare | None = None
    share_t: float = -math.inf
    resync_frames: int = 0
    console: dict[str, object] = field(default_factory=dict)       # the last status line this node printed
    console_t: float = -math.inf
    first_seen_t: float | None = None


@dataclass
class Event:
    seq: int
    t: float
    frame: int | None
    level: str
    src: str
    kind: str
    text: str
    node: str | None = None
    fields: dict[str, object] = field(default_factory=dict)

    def as_dict(self) -> dict:
        return {"seq": self.seq, "t": round(self.t, 3), "frame": self.frame, "level": self.level, "src": self.src, "kind": self.kind, "text": self.text, "node": self.node, "fields": self.fields}


class VoteMonitor:
    """The console's own look at the vote: for each frame in which at least two nodes sent a channel, each node's distance from the median, in units of the voter's tolerance for that channel.

    A node's samples are compared only with the others' of the *same* frame (the low byte of the frame number is in every frame), because a signal that moves 10 dps in 10 ms makes two samples of
    neighbouring frames look like a fault. This is a reconstruction from the bus: the flight computers' own verdict is the heartbeat's view, and ACT's, the vote status.
    """

    STREAMS = {"gyro": (0, 3), "accel": (3, 3), "cmd": (6, 2)}
    MAX_AGE_S = 0.025

    def __init__(self) -> None:
        self.pending: dict[tuple[str, int], dict] = {}
        self.latest: list[dict | None] = [None] * len(K.CHANNELS)
        self.peak = [[0.0] * len(K.CHANNELS) for _ in range(3)]       # the largest |deviation| / tolerance of each node on each channel since the last take_peaks()
        self.compared = 0
        self.over = [0, 0, 0]                                         # frames in which a node was beyond the tolerance on some channel
        self.over_pending = [False, False, False]

    def reset(self) -> None:
        self.__init__()

    def add(self, t: float, stream: str, node: int, seq: int, values: tuple, expected: int) -> None:
        for key in [k for k, v in self.pending.items() if t - v["t"] > self.MAX_AGE_S or t < v["t"] - 1.0]:
            entry = self.pending.pop(key)
            if len(entry["vals"]) >= 2 and not entry["done"]:
                self._compare(key[0], entry)
        entry = self.pending.setdefault((stream, seq), {"t": t, "vals": {}, "done": False})
        entry["vals"][node] = values
        if len(entry["vals"]) >= max(2, expected) and not entry["done"]:
            self._compare(stream, entry)
            entry["done"] = True

    def _compare(self, stream: str, entry: dict) -> None:
        first, count = self.STREAMS[stream]
        vals = entry["vals"]
        self.compared += 1
        beyond = [False, False, False]
        for c in range(count):
            ch = first + c
            xs = {n: v[c] for n, v in vals.items() if math.isfinite(v[c])}
            if len(xs) < 2:
                continue
            med = statistics.median(xs.values())
            tol = K.VOTE_TOL[ch]
            dev = {n: x - med for n, x in xs.items()}
            norm = {n: abs(d) / tol for n, d in dev.items()}
            self.latest[ch] = {"t": entry["t"], "vals": [xs.get(n) for n in range(3)], "med": med, "dev": [dev.get(n) for n in range(3)], "norm": [norm.get(n) for n in range(3)], "n": len(xs)}
            for n, v in norm.items():
                self.peak[n][ch] = max(self.peak[n][ch], v)
                if v > 1.0:
                    beyond[n] = True
        for n in range(3):
            if beyond[n]:
                self.over[n] += 1

    def take_peaks(self) -> list[list[float]]:
        out = [row[:] for row in self.peak]
        self.peak = [[0.0] * len(K.CHANNELS) for _ in range(3)]
        return out


class Telemetry:
    def __init__(self, on_event: Callable[[Event], None] | None = None) -> None:
        self.on_event = on_event
        self.events: deque[Event] = deque(maxlen=5000)
        self._event_seq = 0
        self.reset()

    # ------------------------------------------------------------------ state
    def reset(self) -> None:
        """Forget everything (a replay that seeks, a new run). The event log is kept: it is the record of what the console saw."""
        self.nodes = [Node(n) for n in K.NODE_NAMES]
        self.vote = VoteMonitor()
        self.ids: dict[int, IdStat] = {}
        self.rx_total = 0
        self.crc_bad_total = 0
        self.oos: dict[int, int] = {}                  # out-of-schedule ids seen, and how many frames of each
        self.t_first: float | None = None
        self.t_last = 0.0
        # SYNC
        self.sync_no: int | None = None
        self.sync_t = -math.inf
        self.sync_count = 0
        self.sync_restarts = 0
        self.mission = 0
        self.frame_rate = 0.0
        self._rate_mark: tuple[float, int] | None = None
        # ACT
        self.act: P.ActOut | None = None
        self.act_t = -math.inf
        self.act_count = 0
        # the simulator, as the bus carries it
        self.sim_state: P.SimState | None = None
        self.sim_state_t = -math.inf
        self.sim_tm: P.SimTelemetry | None = None
        self.sim_flags: P.SimFlags | None = None
        self.sim_flags_t = -math.inf
        self.sim_rates: tuple | None = None
        self.sim_accel: tuple | None = None
        # the simulator's own truth (UDP from tfc_simd --telemetry)
        self.truth: dict | None = None
        self.truth_t = -math.inf
        self._q_peak = 0.0
        self._q_peak_t = 0.0
        self._maxq_reported = False
        # the operator's frames on the bus
        self.ground: deque[dict] = deque(maxlen=200)
        self.milestones: dict[str, float] = {}
        self._notes: dict[str, object] = {}            # the last value of each thing an event is made from a change of

    # ------------------------------------------------------------------ events
    def emit(self, t: float, level: str, src: str, kind: str, text: str, node: str | None = None, fields: dict | None = None, frame: int | None = None) -> Event:
        self._event_seq += 1
        e = Event(self._event_seq, t, self.sync_no if frame is None else frame, level, src, kind, text, node, fields or {})
        self.events.append(e)
        if self.on_event is not None:
            self.on_event(e)
        return e

    def _changed(self, key: str, value: object) -> bool:
        """True (and remembers it) when `value` is not what was last seen under `key`; the first sighting counts as a change of None to the value."""
        old = self._notes.get(key, None)
        self._notes[key] = value
        return old != value

    def _seen(self, key: str) -> bool:
        return key in self._notes

    # ------------------------------------------------------------------ frames
    def feed(self, t: float, f: P.Frame) -> None:
        if self.t_first is None:
            self.t_first = t
        self.t_last = max(self.t_last, t)
        self.rx_total += 1
        st = self.ids.get(f.id)
        if st is None:
            st = self.ids[f.id] = IdStat(first_t=t, mark_t=t)
        st.count += 1
        st.last_t = t
        st.last_hex = f.data.hex()
        good = len(f.data) == 8 and P.check(f)
        if not good:
            st.crc_bad += 1
            self.crc_bad_total += 1
            n = self._node_of(f.id)
            if n is not None:
                self.nodes[n].crc_bad += 1
            return
        if not K.in_schedule(f.id) and f.id != P.ID_GROUND:
            self.oos[f.id] = self.oos.get(f.id, 0) + 1
            return
        handler = self._HANDLERS.get(f.id) or self._range_handler(f.id)
        if handler is not None:
            handler(self, t, f)

    @staticmethod
    def _node_of(can_id: int) -> int | None:
        for base in (P.ID_GYRO_BASE, P.ID_ACCEL_BASE, P.ID_CMD_BASE, P.ID_HEARTBEAT, P.ID_STATE):
            if base <= can_id <= base + 2:
                return can_id - base
        if P.ID_RESYNC <= can_id < P.ID_RESYNC + 12:
            return (can_id - P.ID_RESYNC) // 4
        return None

    def _range_handler(self, can_id: int):
        for base, fn in ((P.ID_GYRO_BASE, Telemetry._on_gyro), (P.ID_ACCEL_BASE, Telemetry._on_accel), (P.ID_CMD_BASE, Telemetry._on_cmd),
                         (P.ID_HEARTBEAT, Telemetry._on_heartbeat), (P.ID_STATE, Telemetry._on_state)):
            if base <= can_id <= base + 2:
                return fn
        if P.ID_RESYNC <= can_id < P.ID_RESYNC + 12:
            return Telemetry._on_resync
        return None

    def _expected(self, t: float) -> int:
        alive = sum(1 for n in self.nodes if t - n.hb_t <= K.HEARTBEAT_TIMEOUT_S)
        return alive if alive >= 2 else 3

    def _seq_check(self, node: Node, stream: str, seq: int) -> None:
        last = node.last_seq.get(stream)
        if last is not None and not P.seq_is_next(last, seq) and last != seq:
            node.seq_gaps += 1
        node.last_seq[stream] = seq

    def _on_sensor(self, t: float, f: P.Frame, base: int, stream: str, lsb: float) -> None:
        n = f.id - base
        s = P.unpack_vec3(f, lsb)
        if s is None:
            return
        node = self.nodes[n]
        node.values[stream] = s.values
        node.sample_t[stream] = t
        self._seq_check(node, stream, s.seq)
        self.vote.add(t, stream, n, s.seq, s.values, self._expected(t))

    def _on_gyro(self, t: float, f: P.Frame) -> None:
        self._on_sensor(t, f, P.ID_GYRO_BASE, "gyro", P.GYRO_LSB_DPS)

    def _on_accel(self, t: float, f: P.Frame) -> None:
        self._on_sensor(t, f, P.ID_ACCEL_BASE, "accel", P.ACCEL_LSB_G)

    def _on_cmd(self, t: float, f: P.Frame) -> None:
        n = f.id - P.ID_CMD_BASE
        c = P.unpack_cmd(f)
        if c is None:
            return
        node = self.nodes[n]
        node.values["cmd"] = (c.pitch_deg, c.yaw_deg)
        node.digest = c.digest
        node.sample_t["cmd"] = t
        self._seq_check(node, "cmd", c.seq)
        self.vote.add(t, "cmd", n, c.seq, (c.pitch_deg, c.yaw_deg), self._expected(t))

    def _on_state(self, t: float, f: P.Frame) -> None:
        n = f.id - P.ID_STATE
        s = P.unpack_state_share(f)
        if s is None:
            return
        node = self.nodes[n]
        node.share, node.share_t = s, t

    def _on_resync(self, t: float, f: P.Frame) -> None:
        r = P.unpack_resync(f)
        if r is not None:
            self.nodes[r[0]].resync_frames += 1

    def _on_sync(self, t: float, f: P.Frame) -> None:
        s = P.unpack_sync(f)
        if s is None:
            return
        was = self.sync_no
        if was is not None and s.frame_no < was and was - s.frame_no > 0:
            self.sync_restarts += 1
            self.emit(t, "warn", "bus", "sync-restart", f"the frame number went back ({was} to {s.frame_no}): the sync master was restarted")
            self.vote.reset()
            self._rate_mark = None
        elif was is None:
            self.emit(t, "info", "bus", "sync-up", f"SYNC is on the bus (frame {s.frame_no})")
        if self.sync_t > -math.inf and t - self.sync_t > K.SYNC_TIMEOUT_S and self._notes.get("sync-silent"):
            self.emit(t, "ok", "bus", "sync-back", "SYNC is back")
        self._notes["sync-silent"] = False
        self.sync_no, self.sync_t = s.frame_no, t
        self.sync_count += 1
        old = self.mission
        self.mission = s.mission
        if old != self.mission:
            self._mission_changed(t, old, self.mission)

    def _mission_changed(self, t: float, old: int, new: int) -> None:
        if old == 0 and P.mission_in_countdown(new):
            self.milestones["countdown"] = t
            self.emit(t, "info", "bus", "countdown", "the countdown started (T-10 s)")
        elif P.mission_in_countdown(old) and new == 0:
            self.milestones.pop("countdown", None)
            self.emit(t, "warn", "bus", "scrub", "the countdown was scrubbed: the mission frame went back to 0")
        elif not P.mission_in_flight(old) and P.mission_in_flight(new):
            self.milestones["t0"] = t
            self.emit(t, "ok", "bus", "t-zero", "T-ZERO: the mission frame passed the countdown")
        elif P.mission_in_flight(old) and not P.mission_in_flight(new):
            self.emit(t, "warn", "bus", "mission-reset", f"the mission frame went back to {new} in flight")

    def _on_heartbeat(self, t: float, f: P.Frame) -> None:
        n = f.id - P.ID_HEARTBEAT
        h = P.unpack_heartbeat(f)
        if h is None:
            return
        node = self.nodes[n]
        name = node.name
        was_silent = self._notes.get(f"hb-silent-{n}")
        first = node.hb is None
        old = node.hb
        node.hb, node.hb_t = h, t
        node.hb_count += 1
        if node.first_seen_t is None:
            node.first_seen_t = t
        self._notes[f"hb-silent-{n}"] = False
        if first:
            self.emit(t, "info", "bus", "node-up", f"node {name} is on the bus (release {h.release_hash:#06x}, resets {h.reset_count})", name)
        elif was_silent:
            self.emit(t, "ok", "bus", "node-back", f"node {name} is sending heartbeats again", name)
        if old is None:
            return
        if h.mode != old.mode:
            self.emit(t, "ok" if h.mode == 3 else "warn" if h.mode in (1, 2) else "crit", name, "node-mode",
                      f"node {name} sees the system as {K.MODE_NAMES.get(old.mode, old.mode)} -> {K.MODE_NAMES.get(h.mode, h.mode)}", name)
        if h.role != old.role:
            self.emit(t, "info", name, "node-role", f"node {name} is now {K.ROLE_NAMES[h.role]} (was {K.ROLE_NAMES[old.role]})", name)
        if h.safe_requested != old.safe_requested:
            self.emit(t, "crit" if h.safe_requested else "ok", name, "safe-request", f"node {name}: Safe request {'RAISED' if h.safe_requested else 'cleared'}", name)
        if h.bus_alarm != old.bus_alarm:
            self.emit(t, "warn" if h.bus_alarm else "ok", name, "bus-alarm", f"node {name}: bus alarm {'RAISED' if h.bus_alarm else 'cleared'}", name)
        if h.quarantined != old.quarantined:
            self.emit(t, "crit" if h.quarantined else "ok", name, "quarantine", f"node {name}: {'quarantined (a reset loop)' if h.quarantined else 'released from quarantine'}", name)
        if h.ready != old.ready:
            self.emit(t, "ok" if h.ready else "warn", name, "ready", f"node {name}: {'READY for launch' if h.ready else 'no longer ready'}", name)
        if h.reset_count != old.reset_count:
            self.emit(t, "warn", name, "reset", f"node {name} has reset (count {old.reset_count} -> {h.reset_count})", name)
        if h.release_hash != old.release_hash:
            self.emit(t, "warn", name, "release", f"node {name} reports release {h.release_hash:#06x} (was {old.release_hash:#06x})", name)
        for subject in range(3):
            if h.node_state[subject] != old.node_state[subject]:
                lvl = {0: "ok", 1: "warn", 2: "info", 3: "crit"}[h.node_state[subject]]
                self.emit(t, lvl, name, "view", f"node {name} now sees node {K.NODE_NAMES[subject]} as {K.VIEW_NAMES[h.node_state[subject]]} (was {K.VIEW_NAMES[old.node_state[subject]]})", K.NODE_NAMES[subject],
                          {"observer": name, "subject": K.NODE_NAMES[subject], "to": K.VIEW_NAMES[h.node_state[subject]], "from": K.VIEW_NAMES[old.node_state[subject]]})

    def _on_act(self, t: float, f: P.Frame) -> None:
        a = P.unpack_act_out(f)
        if a is None:
            return
        old = self.act
        self.act, self.act_t = a, t
        self.act_count += 1
        if old is None:
            self.emit(t, "info", "ACT", "act-up", "ACT is sending its output")
            return
        if a.state != old.state:
            lvl = "ok" if a.state == 1 else "info" if a.state == 0 else "crit"
            self.emit(t, lvl, "ACT", "act-state", f"ACT {K.STATE_NAMES_SAFE[old.state]} -> {K.STATE_NAMES_SAFE[a.state]} (cause: {P.CAUSE_NAMES[min(a.cause, 4)]})")
        if a.excluded_nodes != old.excluded_nodes:
            gone = [K.NODE_NAMES[i] for i in range(3) if (a.excluded_nodes >> i) & 1 and not (old.excluded_nodes >> i) & 1]
            back = [K.NODE_NAMES[i] for i in range(3) if not (a.excluded_nodes >> i) & 1 and (old.excluded_nodes >> i) & 1]
            if gone:
                self.emit(t, "warn", "ACT", "act-excluded", f"ACT excluded node {', '.join(gone)} from its vote", gone[0])
            if back:
                self.emit(t, "ok", "ACT", "act-readmitted", f"ACT readmitted node {', '.join(back)}", back[0])
        self._vote_status_changed(t, a, old)

    # A vote status other than Triplex for a frame or two is a late or lost frame, and the flight computers ride it out (3 of the last 5); reporting each as a warning would bury the one that lasts.
    # So the change is held for BLIP_S before it is called an event: a status that returns inside it is one quiet line, one that stays is the warning.
    BLIP_S = 0.05

    def _vote_status_changed(self, t: float, a: P.ActOut, old: P.ActOut) -> None:
        pend = self._notes.get("vote-pending")            # (since t, since frame, status, reported) of a departure from the full vote
        settled = 0                                       # Triplex: all three commands voting, which is what the vote is for; anything else is a departure from it
        name = lambda v: P.VOTE_STATUS_NAMES[min(v, 5)]  # noqa: E731
        if a.vote_status == settled:
            if pend is not None:
                self._notes["vote-pending"] = None
                if pend[3]:                                # it had already been reported: say it is over
                    self.emit(t, "ok", "ACT", "vote-status", f"ACT's vote status is back to Triplex (it was {name(pend[2])} from frame {pend[1]})")
                else:
                    self.emit(t, "info", "ACT", "vote-blip", f"ACT's vote status blipped to {name(pend[2])} for {max(1, round((t - pend[0]) / K.FRAME_S))} frame(s)")
            return
        if pend is None or pend[2] != a.vote_status:
            pend = (t, self.sync_no, a.vote_status, False)
            self._notes["vote-pending"] = pend
        if not pend[3] and t - pend[0] >= self.BLIP_S:
            self._notes["vote-pending"] = (pend[0], pend[1], pend[2], True)
            self.emit(t, "warn", "ACT", "vote-status", f"ACT's vote status is {name(a.vote_status)}, not Triplex (since frame {pend[1]})", frame=pend[1])

    def _on_ground(self, t: float, f: P.Frame) -> None:
        g = P.unpack_ground(f)
        if g is None:
            return
        name = P.GROUND_OP_NAMES.get(g.op, f"op{g.op}")
        who = P.PHASE_NAMES[g.node] if g.op in (7,) and g.node < 8 else (K.NODE_NAMES[g.node] if g.node < 3 else f"node{g.node}")
        ok = g.tag == P.ground_mac(P.ground_key(), g.op | (P.ARM_FLAG if g.arm else 0), g.node, g.counter)
        rec = {"t": round(t, 3), "frame": self.sync_no, "op": name, "target": who, "arm": g.arm, "counter": g.counter, "tag_ok": ok}
        self.ground.append(rec)
        self.emit(t, "info", "bus", "ground-frame", f"a ground command on the bus: {'ARM ' if g.arm else ''}{name} {who if name not in P.NODELESS_OPS else ''}".rstrip() + f" (counter {g.counter}{'' if ok else ', tag NOT valid under this console key'})", None, rec)

    def _on_sim_state(self, t: float, f: P.Frame) -> None:
        s = P.unpack_sim_state(f)
        if s is not None:
            self.sim_state, self.sim_state_t = s, t

    def _on_sim_tm(self, t: float, f: P.Frame) -> None:
        s = P.unpack_sim_telemetry(f)
        if s is not None:
            self.sim_tm = s

    def _on_sim_flags(self, t: float, f: P.Frame) -> None:
        s = P.unpack_sim_flags(f)
        if s is None:
            return
        old = self.sim_flags
        self.sim_flags, self.sim_flags_t = s, t
        if old is None:
            return
        names = ((P.SIM_FLAG_SAFED, "the vehicle went to Safe", "crit"), (P.SIM_FLAG_PLATFORM_SATURATED, "the platform is saturated", "warn"), (P.SIM_FLAG_ENGINE_OUT, "an engine is out", "warn"),
                 (P.SIM_FLAG_COMMAND_HELD, "the simulator is holding the last command (no ACT frame)", "warn"), (P.SIM_FLAG_ABORTED, "the run was aborted", "crit"))
        for bit, text, lvl in names:
            if (s.flags & bit) != (old.flags & bit):
                self.emit(t, lvl if s.flags & bit else "ok", "SIM", "sim-flag", f"simulator: {text}" if s.flags & bit else f"simulator: cleared: {text}")
        if s.engines_on != old.engines_on:
            self.emit(t, "info", "SIM", "engines", f"engines on: {old.engines_on} -> {s.engines_on}")
        if old.time_frames == 0 and s.time_frames > 0:
            self.milestones["liftoff"] = t
            self.emit(t, "ok", "SIM", "liftoff", "LIFT-OFF: the simulator's flight time started")

    def _on_sim_rates(self, t: float, f: P.Frame) -> None:
        s = P.unpack_vec3(f, P.GYRO_LSB_DPS)
        if s is not None:
            self.sim_rates = s.values

    def _on_sim_accel(self, t: float, f: P.Frame) -> None:
        s = P.unpack_vec3(f, P.ACCEL_LSB_G)
        if s is not None:
            self.sim_accel = s.values

    _HANDLERS = {P.ID_SYNC: _on_sync, P.ID_ACT_OUT: _on_act, P.ID_GROUND: _on_ground, P.ID_SIM_STATE: _on_sim_state, P.ID_SIM_TELEMETRY: _on_sim_tm,
                 P.ID_SIM_FLAGS: _on_sim_flags, P.ID_SIM_RATES: _on_sim_rates, P.ID_SIM_ACCEL: _on_sim_accel}

    # ------------------------------------------------------------------ the simulator's truth, and the nodes' consoles
    def feed_truth(self, t: float, d: dict) -> None:
        old = self.truth
        self.truth, self.truth_t = d, t
        q = float(d.get("q", 0.0))
        if q > self._q_peak:
            self._q_peak, self._q_peak_t = q, float(d.get("ft", 0.0))
            self._maxq_reported = False
        elif not self._maxq_reported and self._q_peak > 1000.0 and q < 0.9 * self._q_peak:
            self._maxq_reported = True
            self.milestones["max-q"] = t
            self.emit(t, "info", "SIM", "max-q", f"max-Q passed: {self._q_peak / 1000.0:.1f} kPa at T+{self._q_peak_t:.1f} s (the dynamic pressure is now 10 % below its peak)")
        if old is None:
            return
        for s in range(6):
            bit = 1 << s
            if (int(d.get("stages_ignited", 0)) & bit) and not (int(old.get("stages_ignited", 0)) & bit):
                self.emit(t, "ok", "SIM", "stage-ignition", f"stage {s + 1} ignited at T+{float(d.get('ft', 0.0)):.1f} s")
            if not (int(d.get("stages_active", 0)) & bit) and (int(old.get("stages_active", 0)) & bit):
                self.emit(t, "info", "SIM", "stage-separation", f"stage {s + 1} separated at T+{float(d.get('ft', 0.0)):.1f} s")
        if d.get("crashed") and not old.get("crashed"):
            self.emit(t, "crit", "SIM", "crashed", "THE VEHICLE WAS DESTROYED (it reached the ground faster than the crash speed)")
        if old.get("clamped") and not d.get("clamped"):
            self.milestones.setdefault("liftoff", t)

    def feed_console(self, t: float, src: str, parsed) -> None:
        """A parsed line (`lines.parse_line`) from the console of `src` (A, B, C, ACT, SUP, SIM): into the node's last status, and into the event log."""
        if parsed.kind == "status" and src in K.NODE_NAMES:
            node = self.nodes[K.NODE_NAMES.index(src)]
            node.console, node.console_t = parsed.fields, t
            return
        if parsed.kind == "act-status":
            self._notes["act-console"] = parsed.fields
            return
        lvl = parsed.level
        text = f"[{src}] {parsed.text}"
        self.emit(t, lvl, src, parsed.kind, text, parsed.node or (src if src in K.NODE_NAMES else None), parsed.fields, frame=parsed.frame)

    # ------------------------------------------------------------------ derived state
    def _alive(self, last_t: float, now: float, limit: float) -> bool:
        return now - last_t <= limit

    def check_timeouts(self, now: float) -> None:
        """Emit the events of things going silent (called from the ticker: silence is the absence of frames, so nothing feeds it)."""
        if self.sync_t > -math.inf and now - self.sync_t > K.SYNC_TIMEOUT_S and not self._notes.get("sync-silent"):
            self._notes["sync-silent"] = True
            self.emit(now, "crit", "bus", "sync-lost", f"no SYNC for {now - self.sync_t:.1f} s")
        for i, n in enumerate(self.nodes):
            if n.hb is not None and now - n.hb_t > K.HEARTBEAT_TIMEOUT_S and not self._notes.get(f"hb-silent-{i}"):
                self._notes[f"hb-silent-{i}"] = True
                self.emit(now, "crit", "bus", "node-silent", f"node {n.name}: no heartbeat for {now - n.hb_t:.1f} s", n.name)
        if self.act is not None and now - self.act_t > K.ACT_TIMEOUT_S and not self._notes.get("act-silent"):
            self._notes["act-silent"] = True
            self.emit(now, "crit", "ACT", "act-silent", f"no ACT output for {now - self.act_t:.1f} s")
        elif self.act is not None and now - self.act_t <= K.ACT_TIMEOUT_S and self._notes.get("act-silent"):
            self._notes["act-silent"] = False
            self.emit(now, "ok", "ACT", "act-back", "ACT is sending its output again")

    def update_rates(self, now: float) -> None:
        for st in self.ids.values():
            dt = now - st.mark_t
            if dt >= 1.0:
                st.rate = (st.count - st.mark_count) / dt
                st.mark_count, st.mark_t = st.count, now
        if self.sync_no is not None:
            if self._rate_mark is None:
                self._rate_mark = (now, self.sync_no)
            elif now - self._rate_mark[0] >= 1.0:
                self.frame_rate = (self.sync_no - self._rate_mark[1]) / (now - self._rate_mark[0])
                self._rate_mark = (now, self.sync_no)

    def views(self) -> list[list[int | None]]:
        """views[observer][subject]: each node's own view of every node (the heartbeat's byte 2), None for an observer that has never been heard."""
        return [list(n.hb.node_state) if n.hb is not None else [None, None, None] for n in self.nodes]

    def health_of(self, subject: int, now: float) -> str:
        """What the system says about node `subject`: the view of the live nodes other than itself, the worst if they differ (the heartbeats are the flight computers' own verdicts)."""
        seen = [self.nodes[o].hb.node_state[subject] for o in range(3) if o != subject and self.nodes[o].hb is not None and now - self.nodes[o].hb_t <= K.HEARTBEAT_TIMEOUT_S]
        if not seen:
            own = self.nodes[subject].hb
            return K.VIEW_NAMES[own.node_state[subject]] if own is not None and now - self.nodes[subject].hb_t <= K.HEARTBEAT_TIMEOUT_S else "unknown"
        return K.VIEW_NAMES[max(seen, key=lambda v: (v == 3, v == 1, v == 2, v == 0))]  # disagreement shows the most serious of them

    def go_nogo(self, now: float) -> dict:
        """The launch checklist (tfc_peers/launch.py, Observer.verdict) as a list of items: `required` ones decide, the others inform. A test holds `go` to the checklist's own verdict."""
        items: list[dict] = []

        def add(key: str, label: str, ok: bool | None, detail: str, required: bool = True) -> None:
            items.append({"key": key, "label": label, "ok": ok, "detail": detail, "required": required})

        sync_ok = self._alive(self.sync_t, now, K.SYNC_TIMEOUT_S)
        add("sync", "SYNC on the bus", sync_ok, f"frame {self.sync_no}" if sync_ok else "no SYNC")
        for i, n in enumerate(self.nodes):
            alive = n.hb is not None and self._alive(n.hb_t, now, K.HEARTBEAT_TIMEOUT_S)
            nm = n.name
            add(f"hb-{nm}", f"FC-{nm} heartbeat", alive, "present" if alive else "no heartbeat")
            add(f"ready-{nm}", f"FC-{nm} ready (IMU calibrated, sensors and attitude good)", bool(alive and n.hb.ready), "ready" if alive and n.hb.ready else "not ready" if alive else "no heartbeat")
            add(f"triplex-{nm}", f"FC-{nm} in Triplex", (n.hb.mode == 3) if alive else False, K.MODE_NAMES.get(n.hb.mode, "?") if alive else "no heartbeat")
            add(f"safe-{nm}", f"FC-{nm} has no Safe request up", (not n.hb.safe_requested) if alive else None, "none" if alive and not n.hb.safe_requested else "A SAFE REQUEST IS UP" if alive else "no heartbeat")
        act_alive = self.act is not None and self._alive(self.act_t, now, K.ACT_TIMEOUT_S)
        add("act-out", "ACT output present", act_alive, "present" if act_alive else "no ACT output")
        add("act-nominal", "ACT Nominal", (self.act.state == 1) if act_alive else False, P.STATE_NAMES[self.act.state] if act_alive and self.act.state < len(P.STATE_NAMES) else "no ACT output")
        bad = [n.name for i, n in enumerate(self.nodes) if self.health_of(i, now) not in ("healthy",)]
        add("health", "every node healthy in every other node's view", not bad, "all healthy" if not bad else f"not healthy: {', '.join(bad)}", required=False)
        alarm = any(n.hb is not None and n.hb.bus_alarm and self._alive(n.hb_t, now, K.HEARTBEAT_TIMEOUT_S) for n in self.nodes)
        add("bus-alarm", "no bus alarm", not alarm, "none" if not alarm else "a node has raised the bus alarm", required=False)
        tr = self.truth is not None and self._alive(self.truth_t, now, K.TRUTH_TIMEOUT_S)
        add("clamped", "vehicle clamped on the pad", (bool(self.truth.get("clamped")) if tr else None), "clamped" if tr and self.truth.get("clamped") else "released" if tr else "no simulator telemetry", required=False)
        go = all(i["ok"] for i in items if i["required"])
        return {"go": go, "items": items, "reasons": [f"{i['label']}: {i['detail']}" for i in items if i["required"] and not i["ok"]]}

    def phase(self, now: float) -> dict:
        m = self.mission
        sync_ok = self._alive(self.sync_t, now, K.SYNC_TIMEOUT_S)
        if not sync_ok and self.sync_no is None:
            name = "no-sync"
        elif m == 0:
            name = "pad"
        elif P.mission_in_countdown(m):
            name = "countdown"
        else:
            name = "flight"
        return {"name": name, "mission_frame": m, "t_minus_s": P.mission_frames_to_zero(m) * K.FRAME_S if P.mission_in_countdown(m) else None,
                "flight_s": P.mission_flight_frames(m) * K.FRAME_S if P.mission_in_flight(m) else None}

    def alerts(self, now: float, gng: dict) -> list[dict]:
        out: list[dict] = []

        def add(level: str, key: str, text: str) -> None:
            out.append({"level": level, "key": key, "text": text})

        if self.sync_no is None:
            add("info", "no-sync", "No SYNC on the bus yet: no flight computer is running, or this is the wrong interface.")
        elif not self._alive(self.sync_t, now, K.SYNC_TIMEOUT_S):
            add("crit", "sync-lost", f"No SYNC for {now - self.sync_t:.1f} s.")
        for i, n in enumerate(self.nodes):
            if n.hb is None:
                continue
            alive = self._alive(n.hb_t, now, K.HEARTBEAT_TIMEOUT_S)
            if not alive:
                add("crit", f"silent-{n.name}", f"Node {n.name} is silent ({now - n.hb_t:.1f} s since its last heartbeat).")
                continue
            h = n.hb
            if h.safe_requested:
                add("crit", f"safe-{n.name}", f"Node {n.name} has a Safe request up.")
            if h.quarantined:
                add("crit", f"quarantine-{n.name}", f"Node {n.name} is quarantined (a reset loop): it stays silent until power-cycled.")
            if h.bus_alarm:
                add("warn", f"alarm-{n.name}", f"Node {n.name} reports a bus alarm (out-of-schedule traffic: a babbling sender).")
            health = self.health_of(i, now)
            if health not in ("healthy", "unknown"):
                add("crit" if health in ("disabled",) else "warn", f"health-{n.name}", f"Node {n.name} is {health} in the others' view.")
        hashes = {n.hb.release_hash for n in self.nodes if n.hb is not None}
        if len(hashes) > 1:
            add("info", "release", "The nodes run different releases (heartbeat hashes: " + ", ".join(f"{n.name}={n.hb.release_hash:#06x}" for n in self.nodes if n.hb is not None) + ").")
        alive_modes = {n.hb.mode for n in self.nodes if n.hb is not None and self._alive(n.hb_t, now, K.HEARTBEAT_TIMEOUT_S)}
        if alive_modes and min(alive_modes) < 3:
            add("warn" if min(alive_modes) >= 2 else "crit", "mode", f"Redundancy is down to {K.MODE_NAMES.get(min(alive_modes))}.")
        if self.act is not None:
            if not self._alive(self.act_t, now, K.ACT_TIMEOUT_S):
                add("crit", "act-silent", "ACT is silent.")
            elif self.act.state >= 2:
                add("crit", "act-safe", f"ACT is in {P.STATE_NAMES[self.act.state]} (cause: {P.CAUSE_NAMES[min(self.act.cause, 4)]}).")
            if self.act.excluded_nodes:
                add("warn", "act-excl", "ACT has excluded node " + ", ".join(K.NODE_NAMES[i] for i in range(3) if (self.act.excluded_nodes >> i) & 1) + " from its vote.")
            if self.act.held:
                add("warn", "act-held", "ACT has no trustworthy vote this frame (holding its last output).")
        if self.sim_flags is not None and self._alive(self.sim_flags_t, now, K.SIM_TIMEOUT_S):
            fl = self.sim_flags.flags
            if fl & P.SIM_FLAG_PLATFORM_SATURATED:
                add("warn", "sim-sat", "The platform is saturated: the vehicle's tilt is beyond its travel.")
            if fl & P.SIM_FLAG_ENGINE_OUT:
                add("warn", "sim-engine", "An engine is out.")
            if fl & P.SIM_FLAG_ABORTED:
                add("crit", "sim-abort", "The simulator run was aborted.")
        if self.truth and self.truth.get("crashed"):
            add("crit", "crashed", "The vehicle was destroyed.")
        if self.oos:
            add("warn", "oos", "Out-of-schedule traffic on ids " + ", ".join(f"{i:#05x}" for i in sorted(self.oos)[:6]) + ".")
        over = [n for i, n in enumerate(self.nodes) if self.vote.peak[i] and max(self.vote.peak[i]) > 1.0]
        for n in over:
            add("warn", f"dev-{n.name}", f"Node {n.name} is beyond the voter's tolerance on a channel (the console's own reading of the bus).")
        order = {"crit": 0, "warn": 1, "info": 2}
        return sorted(out, key=lambda a: order[a["level"]])

    # ------------------------------------------------------------------ the snapshot and the chart metrics
    def snapshot(self, now: float) -> dict:
        self.update_rates(now)
        gng = self.go_nogo(now)
        nodes = []
        for i, n in enumerate(self.nodes):
            h = n.hb
            beating = h is not None and self._alive(n.hb_t, now, K.HEARTBEAT_TIMEOUT_S)
            sampling = any(self._alive(v, now, K.HEARTBEAT_TIMEOUT_S) for v in n.sample_t.values())
            alive = beating or sampling                       # a virtual peer (tfc_peers) sends samples and no heartbeat: it is on the bus, and it has no mode, role or view to show
            nodes.append({
                "name": n.name, "seen": h is not None or any(v > -math.inf for v in n.sample_t.values()), "alive": alive, "heartbeat": beating, "hb_age": None if h is None else round(max(0.0, now - n.hb_t), 3),
                "mode": None if h is None else h.mode, "mode_name": None if h is None else K.MODE_NAMES.get(h.mode, "?"),
                "role": None if h is None else K.ROLE_NAMES[h.role], "ready": None if h is None else h.ready, "safe_requested": None if h is None else h.safe_requested,
                "bus_alarm": None if h is None else h.bus_alarm, "quarantined": None if h is None else h.quarantined, "resets": None if h is None else h.reset_count,
                "release": None if h is None else f"{h.release_hash:#06x}", "protocol": None if h is None else h.protocol_version,
                "view": None if h is None else [K.VIEW_NAMES[v] for v in h.node_state], "health": self.health_of(i, now),
                "strikes": None if n.share is None else list(n.share.strikes), "cmd_counter": None if n.share is None else n.share.command_counter,
                "digest": n.digest, "crc_bad": n.crc_bad, "seq_gaps": n.seq_gaps, "resync_frames": n.resync_frames, "hb_count": n.hb_count,
                "sample_age": {k: (None if v == -math.inf else round(max(0.0, now - v), 3)) for k, v in n.sample_t.items()},
                "cmd": None if "cmd" not in n.values else [round(x, 4) for x in n.values["cmd"]],
                "console": n.console, "console_age": None if n.console_t == -math.inf else round(now - n.console_t, 1),
                "over_frames": self.vote.over[i],
            })
        act = None
        if self.act is not None:
            a = self.act
            act = {"alive": self._alive(self.act_t, now, K.ACT_TIMEOUT_S), "age": round(max(0.0, now - self.act_t), 3), "state": a.state, "state_name": P.STATE_NAMES[a.state] if a.state < len(P.STATE_NAMES) else str(a.state),
                   "pitch": round(a.pitch_deg, 4), "yaw": round(a.yaw_deg, 4), "held": a.held, "vote_status": P.VOTE_STATUS_NAMES[min(a.vote_status, 5)],
                   "voted": [bool((a.voted_nodes >> i) & 1) for i in range(3)], "excluded": [bool((a.excluded_nodes >> i) & 1) for i in range(3)], "cause": P.CAUSE_NAMES[min(a.cause, 4)], "frames": self.act_count}
        sim = None
        if self.sim_state is not None:
            fl = self.sim_flags
            tm = self.sim_tm
            sim = {"alive": self._alive(self.sim_state_t, now, K.SIM_TIMEOUT_S), "alt": self.sim_state.altitude_m, "speed": self.sim_state.speed_ms, "mass": self.sim_state.mass_kg,
                   "q": None if tm is None else tm.dynamic_pressure_pa, "err_p": None if tm is None else round(tm.pitch_error_deg, 3), "err_y": None if tm is None else round(tm.yaw_error_deg, 3),
                   "flags": None if fl is None else fl.flags, "engines_on": None if fl is None else fl.engines_on, "time_s": None if fl is None else round(fl.time_frames * K.FRAME_S, 2),
                   "flag_names": [] if fl is None else [n for bit, n in ((1, "safed"), (2, "platform saturated"), (4, "engine out"), (8, "command held"), (16, "aborted")) if fl.flags & bit]}
        truth = None
        if self.truth is not None:
            truth = dict(self.truth)
            truth["alive"] = self._alive(self.truth_t, now, K.TRUTH_TIMEOUT_S)
        ph = self.phase(now)
        return {
            "t": round(now, 3), "frame": self.sync_no, "frame_rate": round(self.frame_rate, 2), "rx_total": self.rx_total, "crc_bad_total": self.crc_bad_total,
            "sync": {"alive": self._alive(self.sync_t, now, K.SYNC_TIMEOUT_S), "age": None if self.sync_t == -math.inf else round(max(0.0, now - self.sync_t), 3), "restarts": self.sync_restarts},
            "phase": ph, "nodes": nodes, "act": act, "sim": sim, "truth": truth, "views": [[None if v is None else K.VIEW_NAMES[v] for v in row] for row in self.views()],
            "vote": {"channels": [None if c is None else {"vals": c["vals"], "med": c["med"], "dev": c["dev"], "norm": c["norm"], "n": c["n"], "age": round(max(0.0, now - c["t"]), 3)} for c in self.vote.latest],
                     "compared": self.vote.compared},
            "go_nogo": gng, "alerts": self.alerts(now, gng), "milestones": {k: round(v, 3) for k, v in self.milestones.items()},
            "ground": list(self.ground)[-12:], "oos": {f"{i:#05x}": c for i, c in sorted(self.oos.items())},
        }

    def metrics(self, now: float) -> dict[str, float | None]:
        """The numbers a chart plots, one value per series: the peak deviation of each node since the last call, the commands, ACT's output, the vehicle."""
        peaks = self.vote.take_peaks()
        m: dict[str, float | None] = {}
        cp, cy = self.vote.latest[6], self.vote.latest[7]    # the commands of one frame (the latest sample of each node could be from neighbouring frames)
        for i, nm in enumerate("abc"):
            m[f"dev_{nm}"] = max(peaks[i]) if self.nodes[i].hb is not None else None
            m[f"cmd_p_{nm}"] = None if cp is None else cp["vals"][i]
            m[f"cmd_y_{nm}"] = None if cy is None else cy["vals"][i]
        m["act_p"] = None if self.act is None else self.act.pitch_deg
        m["act_y"] = None if self.act is None else self.act.yaw_deg
        if self.sim_state is not None:
            m.update(alt=self.sim_state.altitude_m, speed=self.sim_state.speed_ms, mass=self.sim_state.mass_kg)
        if self.sim_tm is not None:
            m.update(q=self.sim_tm.dynamic_pressure_pa, err_p=self.sim_tm.pitch_error_deg, err_y=self.sim_tm.yaw_error_deg)
        if self.truth is not None and self._alive(self.truth_t, now, K.TRUTH_TIMEOUT_S):
            tr = self.truth
            m.update(t_alt=tr.get("alt"), t_range=tr.get("range"), t_speed=tr.get("speed"), t_mach=tr.get("mach"), t_q=tr.get("q"), t_mass=tr.get("mass"), t_thrust=tr.get("thrust"),
                     t_tilt_p=tr.get("tilt_p"), t_tilt_y=tr.get("tilt_y"), t_ref_p=tr.get("ref_p"), t_ref_y=tr.get("ref_y"), t_gim_p=tr.get("gim_p"), t_gim_y=tr.get("gim_y"), t_ft=tr.get("ft"))
        m["frame_rate"] = self.frame_rate
        m["rx_fps"] = sum(s.rate for s in self.ids.values())
        m["mission"] = float(self.mission)
        return m
