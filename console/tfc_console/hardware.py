# SPDX-License-Identifier: MIT
"""The serial side of the rig: the nodes' consoles, the supervisor and the Pico, for when the hardware is on the bench.

* **A node's console** (a Nucleo's ST-LINK virtual COM port, `/dev/ttyACM*`, or `native_sim`'s console) is read line by line and handed to the hub as that node's console: the same lines, the same parser, the
  same event log as a process the rig started. This is how the reasons for a latch get on the page when the nodes are boards.
* **The supervisor** (`docs/design/SUPERVISOR.md` section 6) takes typed commands, one per line, and answers in text. The console offers only the commands that section lists, and nothing else is written to the port.
* **The Pico** (`docs/design/PICO.md`) is the platform driver and fault injector, framed binary (`tfc_peers/pico_link.py`). The console polls its status twice a second and sends relay cuts; `tfc_simd --pico` is
  what streams the platform's tilts, so the console never writes platform commands (two writers on one port would interleave frames).

Nothing here has been run on a board: the parts arrive on 9 October 2026. It is tested against pseudo-terminals and fake ports (`sim/tests/test_console_hardware.py`), and the page says "not run on a board".
"""
from __future__ import annotations

import glob
import re
import threading
import time
from typing import Callable

from tfc_peers import pico_link as PL

from .hub import Hub

SUPERVISOR_VERBS = {"reset": True, "cycle": True, "hold": True, "release": True, "safe-now": False, "safe-clear": False, "launch": False, "scrub": False, "t0": False, "override-ok": False, "time": False, "status": False}
SUPERVISOR_UNITS = ("A", "B", "C", "ACT")


def list_ports() -> list[str]:
    ports = sorted(set(glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*") + glob.glob("/dev/serial/by-id/*")))
    return ports


def _open_serial(port: str, baud: int = 115200, timeout: float = 0.05):
    import serial  # pyserial: only needed with a real port

    return serial.Serial(port, baudrate=baud, timeout=timeout)


class TextLink(threading.Thread):
    """A port that speaks lines of text: a node's console or the supervisor."""

    def __init__(self, hub: Hub, role: str, port: str, opener: Callable = _open_serial) -> None:
        super().__init__(name=f"serial-{role}", daemon=True)
        self.hub, self.role, self.port_name = hub, role, port
        self.ser = opener(port, 115200, 0.05)
        self.lines = 0
        self.connected = True
        self._stop_flag = threading.Event()
        self._wlock = threading.Lock()

    def run(self) -> None:
        buf = b""
        while not self._stop_flag.is_set():
            try:
                chunk = self.ser.read(256)
            except Exception as e:  # noqa: BLE001 - a pulled cable raises whatever the driver raises
                self.connected = False
                self.hub.note("crit", "CON", "serial", f"{self.role} console on {self.port_name} lost: {e}")
                return
            if not chunk:
                continue
            buf += chunk
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                text = raw.decode("utf-8", "replace").rstrip("\r")
                if text.strip():
                    self.lines += 1
                    self.hub.ingest_line(self.role, text)

    def write_line(self, text: str) -> None:
        with self._wlock:
            self.ser.write((text.strip() + "\n").encode())

    def stop(self) -> None:
        self._stop_flag.set()
        self.join(timeout=1.0)
        try:
            self.ser.close()
        except Exception:  # noqa: BLE001
            pass


class PicoLink(threading.Thread):
    def __init__(self, hub: Hub, port: str, opener: Callable = _open_serial) -> None:
        super().__init__(name="serial-pico", daemon=True)
        self.hub, self.port_name = hub, port
        self.ser = opener(port, 115200, 0.01)
        self.client = PL.PicoClient(self.ser)
        self.status: PL.Status | None = None
        self.status_t = 0.0
        self.connected = True
        self._lock = threading.Lock()
        self._stop_flag = threading.Event()

    def run(self) -> None:
        while not self._stop_flag.wait(0.5):
            try:
                with self._lock:
                    self.client.ping()
                    got = self.client.poll()
                if got:
                    self.status, self.status_t = got[-1], time.monotonic()
            except Exception as e:  # noqa: BLE001
                self.connected = False
                self.hub.note("crit", "CON", "serial", f"the Pico on {self.port_name} was lost: {e}")
                return

    def cut(self, node: str, ms: int) -> None:
        with self._lock:
            self.client.cut(node, ms)

    def restore(self, node: str) -> None:
        with self._lock:
            if node == "all":
                self.client.restore_all()
            else:
                self.client.restore(node)

    def stop(self) -> None:
        self._stop_flag.set()
        self.join(timeout=1.5)
        try:
            self.ser.close()
        except Exception:  # noqa: BLE001
            pass


class Hardware:
    def __init__(self, hub: Hub, opener: Callable = _open_serial) -> None:
        self.hub, self.opener = hub, opener
        self.consoles: dict[str, TextLink] = {}
        self.supervisor: TextLink | None = None
        self.pico: PicoLink | None = None
        self.lock = threading.Lock()

    def connect(self, kind: str, port: str, role: str | None = None) -> None:
        if not re.fullmatch(r"[\w./:-]{1,120}", port):
            raise ValueError("not a port name")
        with self.lock:
            if kind == "console":
                if role not in ("A", "B", "C", "ACT"):
                    raise ValueError("a node console is for A, B, C or ACT")
                if role in self.consoles:
                    self.consoles.pop(role).stop()
                link = TextLink(self.hub, role, port, self.opener)
                self.consoles[role] = link
                link.start()
                self.hub.note("info", "CON", "serial", f"node {role}'s console is read from {port}")
            elif kind == "supervisor":
                if self.supervisor:
                    self.supervisor.stop()
                self.supervisor = TextLink(self.hub, "SUP", port, self.opener)
                self.supervisor.start()
                self.hub.note("info", "CON", "serial", f"the supervisor is on {port}")
            elif kind == "pico":
                if self.pico:
                    self.pico.stop()
                self.pico = PicoLink(self.hub, port, self.opener)
                self.pico.start()
                self.hub.note("info", "CON", "serial", f"the Pico is on {port}")
            else:
                raise ValueError(f"unknown kind {kind!r}")

    def disconnect(self, kind: str, role: str | None = None) -> None:
        with self.lock:
            if kind == "console" and role in self.consoles:
                self.consoles.pop(role).stop()
            elif kind == "supervisor" and self.supervisor:
                self.supervisor.stop()
                self.supervisor = None
            elif kind == "pico" and self.pico:
                self.pico.stop()
                self.pico = None

    def supervisor_command(self, text: str) -> None:
        if self.supervisor is None or not self.supervisor.connected:
            raise ValueError("the supervisor is not connected")
        words = text.split()
        if not words or words[0] not in SUPERVISOR_VERBS:
            raise ValueError(f"not a supervisor command; known: {', '.join(SUPERVISOR_VERBS)}")
        if SUPERVISOR_VERBS[words[0]]:
            if len(words) != 2 or words[1].upper() not in SUPERVISOR_UNITS:
                raise ValueError(f"{words[0]} takes one of {', '.join(SUPERVISOR_UNITS)}")
            line = f"{words[0]} {words[1].upper()}"
        elif len(words) != 1:
            raise ValueError(f"{words[0]} takes no argument")
        else:
            line = words[0]
        self.supervisor.write_line(line)
        self.hub.note("info", "OP", "supervisor", f"supervisor command sent: {line}")

    def pico_cut(self, node: str, ms: int) -> None:
        if self.pico is None or not self.pico.connected:
            raise ValueError("the Pico is not connected")
        node = node.upper()
        if node not in PL.NODES:
            raise ValueError(f"unknown node {node!r}; known: {', '.join(PL.NODES)}")
        if not 1 <= ms <= PL.MAX_CUT_MS:
            raise ValueError(f"a cut is 1 to {PL.MAX_CUT_MS} ms (it ends by itself, so a crash of this console cannot leave a node cut)")
        self.pico.cut(node, ms)
        self.hub.note("warn", "OP", "fault", f"TEST ACTION: the injector cuts the power of {node} for {ms} ms")

    def pico_restore(self, node: str) -> None:
        if self.pico is None or not self.pico.connected:
            raise ValueError("the Pico is not connected")
        node = node.lower() if node.lower() == "all" else node.upper()
        if node != "all" and node not in PL.NODES:
            raise ValueError(f"unknown node {node!r}")
        self.pico.restore(node)
        self.hub.note("info", "OP", "fault", f"the injector restores the power of {node}")

    def snapshot(self) -> dict:
        st = self.pico.status if self.pico else None
        return {
            "ports": list_ports(),
            "consoles": {r: {"port": c.port_name, "connected": c.connected, "lines": c.lines} for r, c in self.consoles.items()},
            "supervisor": None if self.supervisor is None else {"port": self.supervisor.port_name, "connected": self.supervisor.connected, "lines": self.supervisor.lines},
            "pico": None if self.pico is None else {"port": self.pico.port_name, "connected": self.pico.connected, "age": None if self.pico.status_t == 0 else round(time.monotonic() - self.pico.status_t, 1),
                                                    "status": None if st is None else {"out_x": st.out_x_deg, "out_y": st.out_y_deg, "flags": st.flags, "relays": st.relays, "command_age_ms": st.command_age_ms, "text": st.describe()}},
            "verbs": [{"verb": v, "unit": u} for v, u in SUPERVISOR_VERBS.items()], "units": list(SUPERVISOR_UNITS),
            "note": "Nothing here has been run on a board (the parts arrive on 9 October 2026).",
        }

    def shutdown(self) -> None:
        for c in list(self.consoles.values()):
            c.stop()
        if self.supervisor:
            self.supervisor.stop()
        if self.pico:
            self.pico.stop()
