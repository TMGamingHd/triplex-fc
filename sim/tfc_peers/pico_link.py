# SPDX-License-Identifier: MIT
"""The PC side of the Pico link (core/include/tfc/pico_link.hpp, docs/PICO.md).

Frame: 0xA5 | type | length | payload (0..12 bytes) | CRC-8 over type, length, payload (SAE J1850, as on the flight bus). The golden frames in
sim/tests/test_pico_link.py are the ones pinned in tests/test_pico.cpp. `PicoClient` talks to the board over a serial port (pyserial, imported only when a real port is
opened); everything else works without it, and the tests use a fake port.
"""
from __future__ import annotations

import math
import struct
import time
from dataclasses import dataclass
from typing import Iterator, Protocol

from .protocol import crc8

SYNC = 0xA5
MAX_PAYLOAD = 12
T_PLATFORM, T_RELAY, T_PING, T_STATUS = 0x01, 0x02, 0x03, 0x81
ANGLE_LSB_DEG = 0.01

FLAG_SATURATED = 0x01
FLAG_HOLDING = 0x02
FLAG_LEVELLING = 0x04
FLAG_LINK_LOST = 0x08
FLAG_WATCHDOG_RESET = 0x10
FLAG_REJECTED = 0x20

NODES = {"A": 0, "B": 1, "C": 2, "ACT": 3}
MAX_CUT_MS = 30000


def _quantize_angle(deg: float) -> int:
    """Round half away from zero, saturating at the 16-bit limits; NaN is zero. The same arithmetic as the C++ (float division)."""
    if math.isnan(deg):
        return 0
    q = float(struct.unpack("<f", struct.pack("<f", deg / ANGLE_LSB_DEG))[0])
    if q >= 32767.0:
        return 32767
    if q <= -32768.0:
        return -32768
    return int(q + 0.5) if q >= 0 else int(q - 0.5)


def encode(msg_type: int, payload: bytes = b"") -> bytes:
    payload = payload[:MAX_PAYLOAD]
    body = bytes([msg_type, len(payload)]) + payload
    return bytes([SYNC]) + body + bytes([crc8(body)])


def pack_platform(seq: int, tilt_x_deg: float, tilt_y_deg: float) -> bytes:
    return encode(T_PLATFORM, struct.pack("<Bhh", seq & 0xFF, _quantize_angle(tilt_x_deg), _quantize_angle(tilt_y_deg)))


def pack_relay(channel: int, cut_ms: int) -> bytes:
    return encode(T_RELAY, struct.pack("<BH", channel & 0xFF, max(0, min(cut_ms, 0xFFFF))))


def pack_ping() -> bytes:
    return encode(T_PING)


@dataclass
class Status:
    seq_echo: int = 0
    out_x_deg: float = 0.0
    out_y_deg: float = 0.0
    flags: int = 0
    relays: int = 0
    command_age_ms: int = 0

    def describe(self) -> str:
        names = [n for bit, n in ((FLAG_SATURATED, "saturated"), (FLAG_HOLDING, "holding"), (FLAG_LEVELLING, "levelling"), (FLAG_LINK_LOST, "link-lost"),
                                  (FLAG_WATCHDOG_RESET, "watchdog-reset"), (FLAG_REJECTED, "rejected-command")) if self.flags & bit]
        cut = [n for n, ch in NODES.items() if self.relays & (1 << ch)]
        return (f"platform out x {self.out_x_deg:7.2f} y {self.out_y_deg:7.2f} deg, command age {self.command_age_ms} ms, flags [{', '.join(names) or '-'}], "
                f"nodes cut [{', '.join(cut) or '-'}]")


def pack_status(s: Status) -> bytes:
    return encode(T_STATUS, struct.pack("<BhhBBH", s.seq_echo & 0xFF, _quantize_angle(s.out_x_deg), _quantize_angle(s.out_y_deg), s.flags & 0xFF, s.relays & 0xFF,
                                        s.command_age_ms & 0xFFFF))


def unpack_status(msg_type: int, payload: bytes) -> Status | None:
    if msg_type != T_STATUS or len(payload) != 9:
        return None
    seq, x, y, flags, relays, age = struct.unpack("<BhhBBH", payload)
    return Status(seq, x * ANGLE_LSB_DEG, y * ANGLE_LSB_DEG, flags, relays, age)


class Parser:
    """Finds frames in a byte stream, drops what does not check, resynchronises on the next 0xA5."""

    def __init__(self) -> None:
        self._buf = bytearray()
        self.bad_frames = 0
        self.skipped_bytes = 0

    def feed(self, data: bytes) -> Iterator[tuple[int, bytes]]:
        self._buf += data
        while True:
            try:
                start = self._buf.index(SYNC)
            except ValueError:
                self.skipped_bytes += len(self._buf)
                self._buf.clear()
                return
            self.skipped_bytes += start
            del self._buf[:start]
            if len(self._buf) < 3:
                return
            length = self._buf[2]
            if length > MAX_PAYLOAD:  # this 0xA5 was data
                self.bad_frames += 1
                del self._buf[:1]
                continue
            total = 4 + length
            if len(self._buf) < total:
                return
            body = bytes(self._buf[1:3 + length])
            if self._buf[3 + length] != crc8(body):
                self.bad_frames += 1
                del self._buf[:1]
                continue
            msg_type = self._buf[1]
            payload = bytes(self._buf[3:3 + length])
            del self._buf[:total]
            yield msg_type, payload


class Port(Protocol):
    def write(self, data: bytes) -> int: ...
    def read(self, size: int = 1) -> bytes: ...


class PicoClient:
    """Commands the platform and the injector. `port` is a pyserial `Serial` (see `open_serial`) or anything with write and read."""

    def __init__(self, port: Port) -> None:
        self.port = port
        self.parser = Parser()
        self.seq = 0
        self.last_status: Status | None = None

    def platform(self, tilt_x_deg: float, tilt_y_deg: float) -> None:
        self.port.write(pack_platform(self.seq, tilt_x_deg, tilt_y_deg))
        self.seq = (self.seq + 1) & 0xFF

    def cut(self, node: str, ms: int) -> None:
        """Cut node A, B, C or ACT for `ms` milliseconds (at most 30 s); it comes back by itself. Send again before it ends to keep it cut."""
        if node.upper() not in NODES:
            raise ValueError(f"unknown node {node!r}; known: {', '.join(NODES)}")
        self.port.write(pack_relay(NODES[node.upper()], min(ms, MAX_CUT_MS)))

    def restore(self, node: str) -> None:
        self.cut(node, 0)

    def restore_all(self) -> None:
        for n in NODES:
            self.restore(n)

    def ping(self) -> None:
        self.port.write(pack_ping())

    def poll(self) -> list[Status]:
        """Read what has arrived and return the status messages in it (the last one is kept in `last_status`)."""
        out: list[Status] = []
        data = self.port.read(256)
        for msg_type, payload in self.parser.feed(data):
            s = unpack_status(msg_type, payload)
            if s is not None:
                out.append(s)
                self.last_status = s
        return out

    def wait_status(self, timeout_s: float = 1.0) -> Status | None:
        end = time.monotonic() + timeout_s
        self.ping()
        while time.monotonic() < end:
            got = self.poll()
            if got:
                return got[-1]
            time.sleep(0.005)
        return None


def open_serial(path: str, baud: int = 115200, timeout_s: float = 0.01) -> Port:
    import serial  # pyserial: only needed with a real board

    return serial.Serial(path, baudrate=baud, timeout=timeout_s)  # the baud rate is ignored by a USB CDC port
