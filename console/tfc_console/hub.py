# SPDX-License-Identifier: MIT
"""The hub: the one place the sources (the bus, a log, the nodes' consoles, the simulator's telemetry) deliver to and the web page reads from.

A source calls `ingest_frame`, `ingest_line` or `ingest_truth`; the hub keeps the model (`model.Telemetry`) under one lock, a ten-per-second history of the chart metrics, the event log and the raw console lines,
and publishes a `state` message to every connected page ten times a second. In a replay the clock is the log's, so the ticks follow the log's time, not the wall's.
"""
from __future__ import annotations

import json
import queue
import re
import threading
import time
from collections import deque
from typing import Callable

from tfc_peers import protocol as P

from . import constants as K
from .lines import parse_line
from .model import Event, Telemetry

HISTORY_SAMPLES = 6000          # ten minutes of ten-per-second metrics
FRAME_RING = 6000               # the bus monitor's window: the last frames, decoded when asked
LINE_RING = 1500                # raw console lines kept per source
_ANSI = re.compile(r"\x1b\[[0-9;?]*[ -/]*[@-~]")   # a console on a terminal is coloured (Zephyr's logger does it): the page wants the words


class Hub:
    def __init__(self, clock: Callable[[], float] = time.monotonic, replay: bool = False) -> None:
        self.lock = threading.RLock()
        self.clock = clock
        self.replay = replay
        self.model = Telemetry(on_event=self._on_event)
        self.history: deque[tuple[float, dict]] = deque(maxlen=HISTORY_SAMPLES)
        self.frames: deque[tuple[int, float, int, bytes]] = deque(maxlen=FRAME_RING)
        self.frame_index = 0
        self.lines: dict[str, deque[dict]] = {}
        self.subscribers: set[queue.Queue] = set()
        self.status: dict[str, object] = {}              # what the page shows about the source: set by the source
        self.extra: dict[str, Callable[[], object]] = {}   # more state for the snapshot (the rig, the faults, the commands), each a function so the hub does not import them
        self.recorder = None
        self.event_listeners: list[Callable[[Event], None]] = []   # a service that wants to see every event (the command log matches the nodes' answers to the command)
        self._seq = 0
        self._last_snapshot: dict | None = None
        self._last_tick_t = -1.0
        self._stop = threading.Event()
        self._ticker: threading.Thread | None = None

    # ------------------------------------------------------------------ the sources deliver here
    def _record(self, fn_name: str, *args) -> None:
        """Hand something to the recorder. A recorder that fails (a full disk) is dropped with a note rather than breaking whoever was delivering: the system goes on, the recording stops, and says so."""
        rec = self.recorder
        if rec is None:
            return
        try:
            getattr(rec, fn_name)(*args)
        except Exception as e:  # noqa: BLE001
            self.recorder = None
            self.model.emit(self.clock(), "crit", "CON", "record", f"the recording failed and was stopped: {type(e).__name__}: {e}")

    def ingest_frame(self, t: float, frame: P.Frame) -> None:
        with self.lock:
            self.model.feed(t, frame)
            self.frame_index += 1
            self.frames.append((self.frame_index, t, frame.id, bytes(frame.data)))
            self._record("frame", t, frame)

    def ingest_line(self, src: str, text: str, t: float | None = None) -> None:
        text = _ANSI.sub("", text)
        parsed = parse_line(text)
        if parsed is None:
            return
        with self.lock:
            now = self.clock() if t is None else t
            self.model.feed_console(now, src, parsed)
            rec = {"t": round(now, 3), "src": src, "text": parsed.text if parsed.kind in ("status", "act-status") else text.rstrip("\r\n"), "kind": parsed.kind, "level": parsed.level}
            self.lines.setdefault(src, deque(maxlen=LINE_RING)).append(rec)
            self._record("line", now, src, text)
            self._publish("line", rec)

    def ingest_truth(self, d: dict, t: float | None = None) -> None:
        with self.lock:
            now = self.clock() if t is None else t
            self.model.feed_truth(now, d)
            self._record("truth", now, d)

    def note(self, level: str, src: str, kind: str, text: str, fields: dict | None = None) -> None:
        """An event the console itself makes (an operator command sent, a process started)."""
        with self.lock:
            self.model.emit(self.clock(), level, src, kind, text, None, fields)

    # ------------------------------------------------------------------ the clock
    def set_live(self) -> None:
        """The clock is the machine's again (after a replay)."""
        with self.lock:
            self.clock = time.monotonic
            self.replay = False

    def set_replay(self, clock: Callable[[], float]) -> None:
        with self.lock:
            self.clock = clock
            self.replay = True

    def start(self) -> None:
        if self._ticker is None:
            self._ticker = threading.Thread(target=self._run, name="hub-ticker", daemon=True)
            self._ticker.start()

    def stop(self) -> None:
        self._stop.set()

    def _run(self) -> None:
        while not self._stop.wait(0.1):
            if not self.replay:
                self.tick()
            else:
                self.publish_state()          # a replay ticks from the log's time (`tick_to`); this keeps a paused page alive

    def tick(self, now: float | None = None) -> None:
        """One sample of the model: the timeouts, the chart metrics into the history, the state out to every page."""
        with self.lock:
            now = self.clock() if now is None else now
            self.model.check_timeouts(now)
            self.history.append((now, self.model.metrics(now)))
            self._last_tick_t = now
            self._last_snapshot = self._build(now)
        self.publish_state()

    def tick_to(self, source_t: float) -> None:
        """Replay: tick at every tenth of a second of the log's time that has passed."""
        while self._last_tick_t < 0 or source_t - self._last_tick_t >= 0.1:
            nxt = source_t if self._last_tick_t < 0 else self._last_tick_t + 0.1
            self.tick(nxt)
            if self._last_tick_t >= source_t:
                break

    def _build(self, now: float) -> dict:
        snap = self.model.snapshot(now)
        snap["source"] = dict(self.status)
        snap["replay"] = self.replay
        snap["recording"] = None if self.recorder is None else self.recorder.describe()
        for key, fn in self.extra.items():
            try:
                snap[key] = fn()
            except Exception as e:  # noqa: BLE001 - a broken panel must not stop the state from reaching the page
                snap[key] = {"error": f"{type(e).__name__}: {e}"}
        return snap

    def snapshot(self) -> dict:
        with self.lock:
            now = self.clock() if not self.replay else (self._last_tick_t if self._last_tick_t >= 0 else 0.0)
            return self._last_snapshot if self._last_snapshot is not None else self._build(now)

    def publish_state(self) -> None:
        with self.lock:
            snap = self._last_snapshot
            if snap is None:
                return
            m = self.history[-1][1] if self.history else {}
            self._seq += 1
            # a replay that is paused has no new tick, but its position, speed and state change when the operator seeks or presses play: those are not the model's, so they are refreshed here
            snap = {**snap, "source": dict(self.status), "replay": self.replay, "recording": None if self.recorder is None else self.recorder.describe()}
            payload = {"seq": self._seq, "snapshot": snap, "metrics": m}
        self._publish("state", payload)

    # ------------------------------------------------------------------ the pages
    def _on_event(self, e: Event) -> None:
        for fn in list(self.event_listeners):
            try:
                fn(e)
            except Exception:  # noqa: BLE001 - a listener's bug must not stop the event reaching the page
                pass
        self._publish("event", e.as_dict())
        self._record("event", e.t, e.as_dict())

    def subscribe(self) -> queue.Queue:
        q: queue.Queue = queue.Queue(maxsize=400)
        with self.lock:
            self.subscribers.add(q)
        return q

    def unsubscribe(self, q: queue.Queue) -> None:
        with self.lock:
            self.subscribers.discard(q)

    def _publish(self, kind: str, payload: object) -> None:
        msg = f"event: {kind}\ndata: {dumps(payload)}\n\n"
        with self.lock:
            subs = list(self.subscribers)
        for q in subs:
            try:
                q.put_nowait(msg)
            except queue.Full:                       # a page that cannot keep up loses its oldest message, and the rest of the system never waits for it
                try:
                    q.get_nowait()
                    q.put_nowait(msg)
                except (queue.Empty, queue.Full):
                    pass

    def history_columns(self, seconds: float | None = None) -> dict:
        """The history as columns: {"t": [...], "series": {name: [...]}}, the last `seconds` of it."""
        with self.lock:
            rows = list(self.history)
        if seconds is not None and rows:
            lo = rows[-1][0] - seconds
            rows = [r for r in rows if r[0] >= lo]
        names: list[str] = sorted({k for _t, m in rows for k in m})
        return {"t": [round(t, 2) for t, _m in rows], "series": {n: [_clean(m.get(n)) for _t, m in rows] for n in names}}

    def recent_events(self, limit: int = 400) -> list[dict]:
        with self.lock:
            return [e.as_dict() for e in list(self.model.events)[-limit:]]

    def recent_lines(self, limit: int = 300) -> dict[str, list[dict]]:
        with self.lock:
            return {src: list(d)[-limit:] for src, d in self.lines.items()}

    def recent_frames(self, after: int = 0, limit: int = 300, ids: set[int] | None = None) -> dict:
        with self.lock:
            rows = [r for r in self.frames if r[0] > after and (ids is None or r[2] in ids)]
            last = self.frame_index
        rows = rows[-limit:]
        out = []
        for idx, t, can_id, data in rows:
            fr = P.Frame(can_id, data)
            ok = len(data) == 8 and P.check(fr)
            out.append({"i": idx, "t": round(t, 4), "id": can_id, "name": K.id_name(can_id), "hex": data.hex(), "ok": ok, "text": P.describe(fr) if ok else f"CRC-BAD  {data.hex()}"})
        return {"last": last, "frames": out}

    def bus_table(self) -> dict:
        with self.lock:
            now = self.clock() if not self.replay else max(self._last_tick_t, 0.0)
            rows = []
            for can_id, st in sorted(self.model.ids.items()):
                rows.append({"id": can_id, "name": K.id_name(can_id), "count": st.count, "rate": round(st.rate, 1), "expected": K.expected_hz(can_id), "crc_bad": st.crc_bad, "age": round(now - st.last_t, 3), "in_schedule": K.in_schedule(can_id) or can_id == P.ID_GROUND, "hex": st.last_hex})
            return {"rows": rows, "rx_total": self.model.rx_total, "crc_bad_total": self.model.crc_bad_total, "oos": {f"{i:#05x}": c for i, c in sorted(self.model.oos.items())},
                    "frame_rate": round(self.model.frame_rate, 2), "nodes": [{"name": n.name, "crc_bad": n.crc_bad, "seq_gaps": n.seq_gaps} for n in self.model.nodes]}

    def reset_model(self) -> None:
        """A replay that seeks: everything the model has seen is forgotten and the history with it (the pages are told to reload)."""
        with self.lock:
            self.model.reset()
            self.model.events.clear()
            self.history.clear()
            self.frames.clear()
            self.lines.clear()
            self._last_tick_t = -1.0
            self._last_snapshot = None
        self._publish("reset", {"t": time.time()})


def dumps(payload: object) -> str:
    """Compact JSON. A NaN or an infinity somewhere in it (JSON has none) goes out as null, rather than the page losing the whole state."""
    try:
        return json.dumps(payload, separators=(",", ":"), allow_nan=False, default=_json_default)
    except ValueError:
        return json.dumps(_scrub(payload), separators=(",", ":"), allow_nan=False, default=_json_default)


def _clean(v: object) -> object:
    if isinstance(v, float):
        if v != v or v in (float("inf"), float("-inf")):
            return None
        return round(v, 4)
    return v


def _scrub(o: object) -> object:
    if isinstance(o, float):
        return o if o == o and o not in (float("inf"), float("-inf")) else None
    if isinstance(o, dict):
        return {k: _scrub(v) for k, v in o.items()}
    if isinstance(o, (list, tuple)):
        return [_scrub(v) for v in o]
    return o


def _json_default(o: object) -> object:
    if isinstance(o, (set, frozenset)):
        return sorted(o)
    if isinstance(o, bytes):
        return o.hex()
    raise TypeError(f"not JSON serialisable: {type(o).__name__}")
