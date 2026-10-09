# SPDX-License-Identifier: MIT
"""Where frames go: a SocketCAN interface (vcan0 / can0), a candump-format log, or memory.

SocketCAN uses the standard library only (AF_CAN raw sockets), so there is nothing to install.
The log format is the one written by `candump -L`, so recordings replay with `canplayer`.
"""
from __future__ import annotations

import socket
import struct
import time
from pathlib import Path
from typing import Iterator, Protocol

from .peers import FRAME_US, Scenario
from .protocol import ID_SYNC, Frame, unpack_sync

_CAN_FRAME = struct.Struct("=IB3x8s")  # struct can_frame: id, dlc, pad, data[8]
CAN_EFF_FLAG = 0x80000000


class Bus(Protocol):
    def send(self, t_us: int, frame: Frame) -> None: ...
    def close(self) -> None: ...


class ListBus:
    """In-memory bus for tests."""

    def __init__(self) -> None:
        self.sent: list[tuple[int, Frame]] = []

    def send(self, t_us: int, frame: Frame) -> None:
        self.sent.append((t_us, frame))

    def close(self) -> None:
        pass


class LogBus:
    """Writes `(sec.usec) iface ID#HEXDATA` lines, the `candump -L` format."""

    def __init__(self, path: str | Path, iface: str = "vcan0") -> None:
        self._fh = open(path, "w", encoding="ascii")
        self._iface = iface

    def send(self, t_us: int, frame: Frame) -> None:
        sec, usec = divmod(t_us, 1_000_000)
        self._fh.write(f"({sec}.{usec:06d}) {self._iface} {frame.id:03X}#{frame.hex().upper()}\n")

    def close(self) -> None:
        self._fh.close()


def read_log(path: str | Path) -> Iterator[tuple[int, str, Frame]]:
    """Parse a candump -L log into (t_us, iface, Frame). Malformed lines raise ValueError."""
    with open(path, encoding="ascii") as fh:
        for n, line in enumerate(fh, 1):
            line = line.strip()
            if not line:
                continue
            try:
                stamp, iface, body = line.split()
                sec, usec = stamp.strip("()").split(".")
                can_id, data = body.split("#")
                yield int(sec) * 1_000_000 + int(usec), iface, Frame(int(can_id, 16), bytes.fromhex(data))
            except ValueError as e:
                raise ValueError(f"{path}:{n}: cannot parse {line!r}: {e}") from e


class SocketCanBus:
    def __init__(self, iface: str) -> None:
        self._sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        try:
            self._sock.bind((iface,))
        except OSError as e:
            self._sock.close()
            raise OSError(f"cannot open CAN interface {iface!r}: {e}. "
                          f"For a virtual bus run sim/scripts/setup_vcan.sh") from e
        self.iface = iface

    def send(self, t_us: int, frame: Frame) -> None:  # t_us ignored: real time is the clock
        self._sock.send(_CAN_FRAME.pack(frame.id, len(frame.data), frame.data.ljust(8, b"\0")))

    def set_filter(self, filters: list[tuple[int, int]]) -> None:
        """Only receive frames matching (can_id, mask) pairs, like a hardware acceptance filter."""
        raw = b"".join(struct.pack("=II", can_id, mask) for can_id, mask in filters)
        self._sock.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FILTER, raw)

    def recv(self, timeout: float | None = None) -> Frame | None:
        self._sock.settimeout(timeout)
        try:
            raw = self._sock.recv(_CAN_FRAME.size)
        except (socket.timeout, BlockingIOError):
            return None
        can_id, dlc, data = _CAN_FRAME.unpack(raw)
        return Frame(can_id & 0x7FF, data[:dlc])

    def close(self) -> None:
        self._sock.close()


def record(scenario: Scenario, bus: Bus, frames: int) -> int:
    """Generate `frames` major frames into `bus` with virtual time (no sleeping). Returns count sent."""
    end_us = frames * FRAME_US
    timed = [tf for k in range(frames) for tf in scenario.frames(k) if tf.t_us < end_us]  # the recording stops at the end
    timed.sort(key=lambda tf: tf.t_us)  # timing faults (early, clock drift) cross frame boundaries; a log is time-ordered
    for tf in timed:
        bus.send(tf.t_us, tf.frame)
    return len(timed)


def _wait_until(deadline_ns: int) -> None:
    remain = (deadline_ns - time.monotonic_ns()) / 1e9
    if remain > 0.0005:
        time.sleep(remain - 0.0003)
    while time.monotonic_ns() < deadline_ns:
        pass


def _stats(frames: int, late_us: list[float], **extra: float) -> dict[str, float]:
    late_us.sort()
    if not late_us:
        return {"frames": frames, "sent": 0, "late_p50_us": 0.0, "late_p99_us": 0.0, "late_max_us": 0.0, **extra}
    return {
        "frames": frames,
        "sent": len(late_us),
        "late_p50_us": late_us[len(late_us) // 2],
        "late_p99_us": late_us[min(len(late_us) - 1, int(len(late_us) * 0.99))],
        "late_max_us": late_us[-1],
        **extra,
    }


def run_realtime(scenario: Scenario, bus: Bus, frames: int) -> dict[str, float]:
    """Send `frames` major frames at 100 Hz wall-clock time (free-running, no SYNC).

    `frames <= 0` means run until Ctrl+C. Ctrl+C always ends the run cleanly and returns the statistics
    (with `interrupted` = 1). Returns send-lateness statistics in microseconds. Python on a desktop Linux
    kernel is not a real-time system: use these numbers to see how good the fake peers' timing is, never
    as a measurement of the flight computers.
    """
    start_ns = time.monotonic_ns() + 50_000_000
    late_us: list[float] = []
    k = 0
    interrupted = 0
    try:
        while frames <= 0 or k < frames:
            for tf in scenario.frames(k):
                deadline = start_ns + tf.t_us * 1000
                _wait_until(deadline)
                bus.send(tf.t_us, tf.frame)
                late_us.append((time.monotonic_ns() - deadline) / 1000.0)
            k += 1
    except KeyboardInterrupt:
        interrupted = 1
    return _stats(k, late_us, interrupted=interrupted)


class SyncBus(Protocol):
    def send(self, t_us: int, frame: Frame) -> None: ...
    def recv(self, timeout: float | None = None) -> Frame | None: ...
    def set_filter(self, filters: list[tuple[int, int]]) -> None: ...


def run_synced(scenario: Scenario, bus: SyncBus, frames: int, sync_timeout_s: float = 2.0,
               on_note=None, poll=None) -> dict[str, float]:
    """Follow a live sync master: for each of `frames` SYNC frames, send that frame's traffic with
    the schedule offsets measured from the instant SYNC arrived, as a time-triggered node would.

    `frames <= 0` means follow until Ctrl+C (which ends the run cleanly and returns the statistics, with
    `interrupted` = 1). The SYNC frame number is the frame index, so peers that start late (or restart)
    still agree with the flight computer on which frame it is. Raises TimeoutError if no SYNC arrives for
    `sync_timeout_s` (before the first SYNC, or after the flight computer stops). `poll(k)`, if given, is called once per SYNC with
    the frame number, before that frame's traffic is generated: the place to change the scenario between frames (`--control`).
    """
    note = on_note or (lambda msg: None)
    bus.set_filter([(ID_SYNC, 0x7FF)])
    late_us: list[float] = []
    followed = rewinds = interrupted = 0
    try:
        while frames <= 0 or followed < frames:
            raw = bus.recv(sync_timeout_s)
            if raw is None:
                raise TimeoutError(f"no SYNC on the bus for {sync_timeout_s:.1f} s: start the flight computer "
                                   f"(the sync master) first, or it has stopped")
            t_rx = time.monotonic_ns()
            sync = unpack_sync(raw)
            if sync is None:
                continue
            k = sync.frame_no
            if k < scenario.next_frame:
                scenario.reset()
                rewinds += 1
                note(f"SYNC frame number went back to {k}: sync master restarted; peers restarted too")
            if poll is not None:
                poll(k)
            base = k * FRAME_US
            for tf in scenario.frames(k):
                deadline = t_rx + (tf.t_us - base) * 1000
                _wait_until(deadline)
                bus.send(tf.t_us, tf.frame)
                late_us.append((time.monotonic_ns() - deadline) / 1000.0)
            followed += 1
    except KeyboardInterrupt:
        interrupted = 1
    return _stats(followed, late_us, rewinds=rewinds, interrupted=interrupted)


__all__ = ["Bus", "ListBus", "LogBus", "SocketCanBus", "read_log", "record", "run_realtime", "run_synced", "FRAME_US"]
