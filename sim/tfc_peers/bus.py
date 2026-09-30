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

from .peers import FRAME_US, Scenario, TimedFrame
from .protocol import Frame

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
    n = 0
    for k in range(frames):
        for tf in scenario.frames(k):
            bus.send(tf.t_us, tf.frame)
            n += 1
    return n


def run_realtime(scenario: Scenario, bus: Bus, frames: int) -> dict[str, float]:
    """Send `frames` major frames at 100 Hz wall-clock time.

    Returns send-lateness statistics in microseconds. Python on a desktop Linux kernel is not a
    real-time system: use these numbers to see how good the fake peers' timing is, never as a
    measurement of the flight computers.
    """
    start_ns = time.monotonic_ns() + 50_000_000
    late_us: list[float] = []
    for k in range(frames):
        pending: list[TimedFrame] = scenario.frames(k)
        for tf in pending:
            deadline = start_ns + tf.t_us * 1000
            remain = (deadline - time.monotonic_ns()) / 1e9
            if remain > 0.0005:
                time.sleep(remain - 0.0003)
            while time.monotonic_ns() < deadline:
                pass
            bus.send(tf.t_us, tf.frame)
            late_us.append((time.monotonic_ns() - deadline) / 1000.0)
    late_us.sort()
    if not late_us:
        return {"frames": 0, "sent": 0, "late_p50_us": 0.0, "late_p99_us": 0.0, "late_max_us": 0.0}
    return {
        "frames": frames,
        "sent": len(late_us),
        "late_p50_us": late_us[len(late_us) // 2],
        "late_p99_us": late_us[min(len(late_us) - 1, int(len(late_us) * 0.99))],
        "late_max_us": late_us[-1],
    }


__all__ = ["Bus", "ListBus", "LogBus", "SocketCanBus", "read_log", "record", "run_realtime", "FRAME_US"]
