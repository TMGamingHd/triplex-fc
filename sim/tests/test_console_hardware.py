# SPDX-License-Identifier: MIT
"""The serial side: a node's console read from a port, the supervisor's commands (only the documented ones), and the Pico's relays. Pseudo-terminals and a fake Pico stand in for the boards, which have not arrived."""
import os
import time
import unittest

from .console_support import ROOT  # first: puts console/ on the path
from tfc_peers import pico_link as PL
from tfc_console.hardware import Hardware, SUPERVISOR_UNITS, SUPERVISOR_VERBS
from tfc_console.hub import Hub
from tfc_console.server import wait_for


class FakePico:
    """A Pico on a fake port: answers a ping with a status, and records every frame written to it."""

    def __init__(self):
        self.rx = bytearray()
        self.written = []
        self.parser = PL.Parser()

    def write(self, data):
        self.written.append(bytes(data))
        for msg_type, payload in self.parser.feed(data):
            if msg_type == PL.T_PING:
                self.rx += PL.pack_status(PL.Status(seq_echo=1, out_x_deg=1.5, out_y_deg=-2.5, flags=PL.FLAG_HOLDING, relays=0b0010, command_age_ms=40))
        return len(data)

    def read(self, size=1):
        time.sleep(0.01)
        out, self.rx = bytes(self.rx[:size]), self.rx[size:]
        return out

    def close(self):
        pass


def pty_pair():
    master, slave = os.openpty()
    return master, os.ttyname(slave), slave


class TextPorts(unittest.TestCase):
    def setUp(self):
        self.hub = Hub()
        self.hw = Hardware(self.hub)
        self.master, self.name, self.slave = pty_pair()

    def tearDown(self):
        self.hw.shutdown()
        os.close(self.master)
        os.close(self.slave)

    def test_a_nodes_console_on_a_port_feeds_the_hub_like_a_process_does(self):
        self.hw.connect("console", self.name, "B")
        os.write(self.master, b"[frame 12] node C LATCHED OUT: frame missing\r\n[frame 100] TRIPLEX  A+ B+ C+  | crc=0 wcet_frame=8800\r\n")
        self.assertTrue(wait_for(lambda: self.hub.model.nodes[1].console.get("wcet_frame") == 8800, 3.0))
        e = next(e for e in self.hub.model.events if e.kind == "latched-out")
        self.assertEqual((e.src, e.node, e.fields["reason"]), ("B", "C", "frame missing"))
        self.assertEqual(self.hw.snapshot()["consoles"]["B"]["lines"], 2)

    def test_a_console_is_for_a_node_and_a_port_name_is_a_name(self):
        with self.assertRaises(ValueError):
            self.hw.connect("console", self.name, "X")
        with self.assertRaises(ValueError):
            self.hw.connect("console", "/dev/tty; rm -rf /", "A")
        with self.assertRaises(ValueError):
            self.hw.connect("modem", self.name)

    def test_a_port_that_disappears_is_reported_not_swallowed(self):
        self.hw.connect("console", self.name, "A")
        os.close(self.slave)                               # the board is unplugged
        self.slave = os.open("/dev/null", os.O_RDONLY)
        os.close(self.master)
        self.master = os.open("/dev/null", os.O_RDONLY)
        self.assertTrue(wait_for(lambda: not self.hw.snapshot()["consoles"]["A"]["connected"], 3.0))
        self.assertTrue(any(e.kind == "serial" and e.level == "crit" for e in self.hub.model.events))

    def test_the_supervisor_takes_only_the_documented_commands(self):
        self.hw.connect("supervisor", self.name)
        for ok in ("status", "time", "safe-now", "safe-clear", "launch", "scrub", "t0", "override-ok", "reset A", "cycle ACT", "hold b", "release c"):
            self.hw.supervisor_command(ok)
        got = b""
        os.set_blocking(self.master, False)
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline and got.count(b"\n") < 12:
            try:
                got += os.read(self.master, 4096)
            except BlockingIOError:
                time.sleep(0.02)
        lines = got.decode().replace("\r", "").split("\n")
        for expect in ("status", "reset A", "cycle ACT", "hold B", "release C", "t0"):
            self.assertIn(expect, lines)
        for bad in ("rm -rf /", "reset", "reset Z", "reset A B", "status now", "sel A", "adopt A", ""):
            with self.assertRaises(ValueError, msg=bad):
                self.hw.supervisor_command(bad)

    def test_a_supervisor_that_is_not_connected_takes_nothing(self):
        with self.assertRaises(ValueError):
            self.hw.supervisor_command("status")

    def test_the_verbs_are_the_designs(self):
        text = (ROOT / "docs" / "design" / "SUPERVISOR.md").read_text()
        for verb in SUPERVISOR_VERBS:
            self.assertIn(f"`{verb}", text, verb)
        self.assertEqual(set(SUPERVISOR_UNITS), {"A", "B", "C", "ACT"})


class ThePico(unittest.TestCase):
    def setUp(self):
        self.hub = Hub()
        self.fake = FakePico()
        self.hw = Hardware(self.hub, opener=lambda port, baud, timeout: self.fake)
        self.hw.connect("pico", "/dev/ttyACM0")

    def tearDown(self):
        self.hw.shutdown()

    def frames(self):
        p = PL.Parser()
        out = []
        for chunk in self.fake.written:
            out += list(p.feed(chunk))
        return out

    def test_its_status_is_polled_and_shown(self):
        self.assertTrue(wait_for(lambda: self.hw.snapshot()["pico"]["status"] is not None, 3.0))
        st = self.hw.snapshot()["pico"]["status"]
        self.assertEqual((st["out_x"], st["out_y"], st["relays"], st["command_age_ms"]), (1.5, -2.5, 2, 40))
        self.assertIn("holding", st["text"])
        self.assertIn("B", st["text"])                              # relay bit 1 is node B: cut

    def test_a_cut_is_a_relay_frame_for_that_node_with_its_time_and_is_a_logged_test_action(self):
        self.hw.pico_cut("b", 2500)
        relay = [(t, p) for t, p in self.frames() if t == PL.T_RELAY]
        self.assertEqual(len(relay), 1)
        self.assertEqual((relay[0][1][0], int.from_bytes(relay[0][1][1:3], "little")), (1, 2500))
        e = self.hub.model.events[-1]
        self.assertEqual((e.src, e.kind), ("OP", "fault"))
        self.assertIn("TEST ACTION", e.text)

    def test_a_cut_is_bounded_and_for_a_known_node(self):
        for node, ms in (("D", 100), ("A", 0), ("A", PL.MAX_CUT_MS + 1)):
            with self.assertRaises(ValueError):
                self.hw.pico_cut(node, ms)
        self.assertEqual([t for t, _p in self.frames() if t == PL.T_RELAY], [])

    def test_restore_one_or_all(self):
        self.hw.pico_restore("all")
        self.hw.pico_restore("ACT")
        relay = [p for t, p in self.frames() if t == PL.T_RELAY]
        self.assertEqual(len(relay), 5)                              # all four nodes, then ACT
        self.assertTrue(all(int.from_bytes(p[1:3], "little") == 0 for p in relay))
        with self.assertRaises(ValueError):
            self.hw.pico_restore("Q")

    def test_the_console_never_writes_platform_commands(self):
        """Two writers on the Pico's port would interleave frames: tfc_simd --pico streams the platform, and the console only polls and cuts."""
        time.sleep(0.7)
        self.assertEqual({t for t, _p in self.frames()}, {PL.T_PING})


if __name__ == "__main__":
    unittest.main()
