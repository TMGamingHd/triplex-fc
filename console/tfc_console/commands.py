# SPDX-License-Identifier: MIT
"""The operator's commands: authenticated ground frames (`docs/design/PROTOCOL.md`, ADR-019) sent on the bus, and the log of what became of each.

The console is a second `tfc_peers command`: it builds the same frames (`protocol.pack_ground`, the key from `$TFC_GROUND_KEY` or the public bench key) and takes its counters from the same file
(`~/.cache/tfc_peers/ground_counter`), so a command typed in a terminal and one clicked in the page never reuse a counter. What it adds is the discipline around the frame:

* **two steps are two steps.** A command that needs an ARM (launch, clear-safe, clear-disabled, a disable that leaves too few voters) is sent as an ARM frame and, 50 ms later, the EXECUTE frame, exactly as the
  command line does, and only after the page has shown what it is and the operator has said yes (`confirmed`). The flight computers enforce the rule again (an EXECUTE without a matching ARM is refused): the page's
  confirmation is the operator's deliberate step, not a substitute for theirs.
* **a launch needs a go.** The console refuses to send `launch` while its own go/no-go (the checklist's rule) is a no-go, unless `force` is set, which exists to test that the flight computers refuse it too.
* **the answer is what the nodes print.** The bus carries no reply to a command: the flight computer says `GROUND COMMAND reintegrate B: accepted` on its console. When a console is attached the log shows each node's
  answer; when none is, it says so, and what the heartbeats do afterwards is the only evidence.
* **a replay cannot send** (there is no bus), and a command is never sent to an interface that is not the one being watched.
"""
from __future__ import annotations

import threading
import time
from collections import deque

from tfc_peers import protocol as P
from tfc_peers.bus import SocketCanBus
from tfc_peers.cli import _next_counter
from tfc_peers.commands import parse_phase
from tfc_peers.faults import FaultSpecError

from . import constants as K
from .hub import Hub
from .model import Event
from .server import ApiError

ARM_GAP_S = 0.05            # what `tfc_peers command --arm` leaves between the ARM and the EXECUTE: five frames

# (op, label, what the target is, whether an ARM is needed, one line of what it does)
CATALOG = (
    ("reintegrate", "Reintegrate", "computer-or-imu", "never", "Readmit a latched-out computer or IMU: it goes on probation and must agree for a second before it votes again."),
    ("disable", "Disable", "computer-or-imu", "tiered", "Take a computer or IMU out of the vote for the rest of the run. ARM needed when it would leave fewer voters than the phase's nominal number."),
    ("clear-disabled", "Clear disabled", "computer-or-imu", "always", "Forget that a node was disabled, so it can be reintegrated (the strikes against it are cleared)."),
    ("clear-safe", "Clear Safe", "none", "always", "Leave Safe: ACT accepts only after its votes have been trustworthy for a second, from two nodes, with no Safe request up."),
    ("launch", "Launch", "none", "always", "Start the countdown (T-10 s). The sync master acts after its own go/no-go."),
    ("scrub", "Scrub", "none", "never", "Abort the countdown and return to the pad. Refused after T-zero."),
    ("phase", "Set phase", "phase", "never", "Move the mission phase (0 to 7). Refused when fewer computers vote than the new phase needs."),
    ("noop", "No-op", "none", "never", "Changes nothing and is answered like any command: tests the command path end to end."),
    ("warm", "Rest as WARM", "computer", "tiered", "Rest a voting computer as WARM (it keeps running, shadow-voted, and does not vote). Reintegrate promotes it."),
)
BY_OP = {c[0]: c for c in CATALOG}


def catalog() -> list[dict]:
    return [{"op": op, "label": label, "target": target, "arm": arm, "doc": doc, "id": P.GROUND_OPS[op]} for op, label, target, arm, doc in CATALOG]


def parse_target(op: str, text: object) -> int:
    """The node field of the frame for `op`: 0 to 2 a computer, 4 to 6 an IMU channel, the phase number for `phase`, 0 for a command that takes no target."""
    kind = BY_OP[op][2]
    if kind == "none":
        return 0
    t = str(text if text is not None else "").strip()
    if not t:
        raise ApiError(400, f"{op} needs a target ({'a phase 0..7 or its name' if kind == 'phase' else 'A, B or C' + (', or IMU-A, IMU-B, IMU-C' if kind == 'computer-or-imu' else '')})")
    try:
        if kind == "phase":
            return parse_phase(t)
        up = t.upper()
        if up.startswith("IMU-") and kind == "computer-or-imu" and up[4:] in K.NODE_NAMES:
            return 4 + K.NODE_NAMES.index(up[4:])
        if up in K.NODE_NAMES:
            return K.NODE_NAMES.index(up)
    except FaultSpecError as e:
        raise ApiError(400, str(e)) from e
    raise ApiError(400, f"{t!r} is not a valid target for {op}")


def target_text(op: str, node: int) -> str:
    kind = BY_OP[op][2]
    if kind == "none":
        return ""
    if kind == "phase":
        return P.PHASE_NAMES[node] if node < 8 else str(node)
    return f"IMU-{K.NODE_NAMES[node - 4]}" if node >= 4 else K.NODE_NAMES[node]


class CommandService:
    def __init__(self, hub: Hub, iface_of) -> None:
        self.hub = hub
        self.iface_of = iface_of                   # a function: the interface the console is attached to, or None
        self.log: deque[dict] = deque(maxlen=200)
        self._n = 0
        self._lock = threading.Lock()
        hub.event_listeners.append(self._on_event)

    # ------------------------------------------------------------------ sending
    def send(self, op: str, target: object = None, mode: str = "auto", confirmed: bool = False, force: bool = False) -> dict:
        if op not in BY_OP:
            raise ApiError(400, f"unknown command {op!r}; known: {', '.join(BY_OP)}")
        iface = self.iface_of()
        if iface is None or self.hub.replay:
            raise ApiError(409, "this console is not attached to a live bus (a replay is read-only), so there is nowhere to send a command")
        node = parse_target(op, target)
        _op, _label, _tk, arm_policy, _doc = BY_OP[op]
        if mode == "auto":
            mode = "armed" if arm_policy == "always" else "single"
        if mode not in ("single", "armed", "arm", "execute", "forged", "replay"):
            raise ApiError(400, f"unknown mode {mode!r}")
        if arm_policy == "always" and mode == "single":
            raise ApiError(409, f"{op} always needs an ARM first (ADR-019): send it as `armed`")
        needs_confirm = mode in ("armed", "arm") or arm_policy == "always"
        if needs_confirm and not confirmed:
            raise ApiError(409, f"{op} is a two-step command: the operator must confirm it")
        if op == "launch" and mode in ("armed", "single", "arm"):
            with self.hub.lock:
                gng = self.hub.model.go_nogo(self.hub.clock())
            if not gng["go"] and not force:
                raise ApiError(409, "NO-GO, the launch was not sent: " + "; ".join(gng["reasons"][:6]) + " (`force` sends it anyway, to test that the flight computers refuse it too)")
        steps: list[tuple[bool, bool]] = {"single": [(False, False)], "armed": [(True, False), (False, False)], "arm": [(True, False)], "execute": [(False, False)], "forged": [(False, True)], "replay": []}[mode]
        key = P.ground_key()
        frames: list[tuple[P.Frame, dict]] = []
        counter = 0
        with self._lock:                            # the counter file and the bus are used by one command at a time
            if mode == "replay":
                last = next((r for r in reversed(self.log) if r.get("frames")), None)
                if last is None:
                    raise ApiError(409, "there is no earlier command of this console to replay")
                frames = [(P.pack_ground(P.GROUND_OPS[last["op"]], last["node"], last["frames"][-1]["counter"], key, bool(last["frames"][-1]["arm"])), last["frames"][-1])]
                counter = last["frames"][-1]["counter"]
            else:
                counter = _next_counter(len(steps), None)
                for i, (arm, forged) in enumerate(steps):
                    c = (counter + i) & 0xFF
                    frames.append((P.pack_ground(P.GROUND_OPS[op], node, c, key, arm, forged), {"counter": c, "arm": arm, "forged": forged}))
            try:
                bus = SocketCanBus(iface)
            except OSError as e:
                raise ApiError(409, str(e)) from e
            try:
                for i, (fr, _meta) in enumerate(frames):
                    bus.send(0, fr)
                    if i + 1 < len(frames):
                        time.sleep(ARM_GAP_S)
            finally:
                bus.close()
        self._n += 1
        now = self.hub.clock()
        rec = {"id": self._n, "t": round(now, 3), "wall": time.time(), "frame": self.hub.model.sync_no, "op": op, "target": target_text(op, node), "node": node, "mode": mode,
               "frames": [m for _f, m in frames], "responses": [], "status": "sent", "forced": bool(force)}
        self.log.append(rec)
        what = f"{'ARM + ' if mode == 'armed' else ''}{op}{(' ' + rec['target']) if rec['target'] else ''}" + (f" [{mode}]" if mode in ("forged", "replay", "arm", "execute") else "")
        self.hub.note("info", "OP", "command", f"operator command sent: {what} (counter {counter})", {"id": rec["id"], "op": op, "target": rec["target"], "mode": mode})
        threading.Timer(3.0, self._settle, args=(rec["id"],)).start()
        return rec

    # ------------------------------------------------------------------ the answers
    def _on_event(self, e: Event) -> None:
        if e.kind != "ground-command" or e.src not in (*K.NODE_NAMES, "ACT"):
            return
        f = e.fields
        with self._lock:
            for rec in reversed(self.log):
                if rec["op"] == f.get("op") and e.t - rec["t"] < 6.0 + 1.0 and rec["status"] in ("sent", "answered"):
                    rec["responses"].append({"src": e.src, "arm": bool(f.get("arm")), "result": f.get("result", ""), "frame": e.frame})
                    rec["status"] = "answered"
                    return

    def _settle(self, rec_id: int) -> None:
        with self._lock:
            for rec in self.log:
                if rec["id"] != rec_id:
                    continue
                if not rec["responses"]:
                    rec["status"] = "no answer"
                    return
                final = [r for r in rec["responses"] if not (r["arm"] and rec["mode"] == "armed")] or rec["responses"]
                oks = [r for r in final if r["result"].startswith(("accepted", "already"))]
                rec["status"] = "accepted" if oks else "refused"
                return

    def snapshot(self) -> dict:
        with self._lock:
            return {"log": [dict(r) for r in list(self.log)[-30:]], "iface": self.iface_of(), "can_send": self.iface_of() is not None and not self.hub.replay}
