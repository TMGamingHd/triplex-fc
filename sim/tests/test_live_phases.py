# SPDX-License-Identifier: MIT
"""Live: mission phases and the WARM role (ADR-023, TFC-PHASE-001..004, TFC-FDIR-043) between three instances of the real firmware on vcan0.

The three images are built with CONFIG_TFC_PHASES (tools/bench/sil_triplex.sh --build-phases). In the coast phase the operator rests computer C as WARM (it stays shadow-voted but does not
vote: the system is Duplex by mode), tries a phase that needs three computers (refused: nothing changes), tests the command path with `noop`, and then promotes C, which votes again after
the probation length. Skipped unless the images are built and vcan0 is present.
"""
import os
import re
import subprocess
import tempfile
import time
import unittest
from pathlib import Path

from tfc_peers import bus as B
from tfc_peers import protocol as P

ROOT = Path(__file__).resolve().parents[2]


def _bin(env, default):
    for c in (os.environ.get(env, ""), str(ROOT / default)):
        if c and os.access(c, os.X_OK):
            return c
    return None


BINS = [_bin(f"TFC_PHASES_BIN_{n.upper()}", f"build/phases_{n}/zephyr/zephyr.exe") for n in "abc"]
STATUS = re.compile(r"\[frame (\d+)\] (\w+)\s+A(.) B(.) C(.)\s+\| .*phase=(\d+) warm=0x([0-9a-f]+)")


@unittest.skipIf(None in BINS, "the phases images are not built (tools/bench/sil_triplex.sh --build-phases)")
@unittest.skipIf(not Path("/sys/class/net/vcan0").exists(), "vcan0 not present (run sim/scripts/setup_vcan.sh)")
class LivePhases(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.procs = []
        self.logs = []
        self.counter = 0

    def tearDown(self):
        for p in self.procs:
            if p.poll() is None:
                p.kill()
                p.wait()
        self.tmp.cleanup()

    def log(self, i=0):
        return Path(self.logs[i]).read_text()

    def last_status(self, i=0):
        return [m for m in map(STATUS.match, self.log(i).splitlines()) if m][-1]

    def pump(self, bus, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            bus.recv(0.1)

    def send(self, bus, op, node=0, arm=False):
        self.counter += 1
        bus.send(0, P.pack_ground(P.GROUND_OPS[op], node, self.counter & 0xFF, None, arm, False))
        self.pump(bus, 0.25)

    def test_the_operator_rests_a_computer_as_warm_a_phase_that_needs_three_is_refused_and_the_promotion_brings_it_back(self):
        for i, b in enumerate(BINS):
            path = Path(self.tmp.name) / f"node{i}.log"
            self.logs.append(path)
            with open(path, "w") as f:
                self.procs.append(subprocess.Popen([b], stdout=f, stderr=subprocess.STDOUT, text=True))
        bus = B.SocketCanBus("vcan0")
        bus.set_filter([(P.ID_SYNC, 0x7FF)])
        try:
            self.pump(bus, 4.0)
            for i in range(3):
                self.assertNotRegex(self.log(i), r"node [ABC] LATCHED OUT", self.log(i))
            self.assertEqual(self.last_status().group(6), "1", "power-up")
            self.send(bus, "noop")
            self.send(bus, "phase", 4)  # coast: two voters, the third may rest
            self.send(bus, "warm", 2)
            self.pump(bus, 1.0)
            st = self.last_status()
            self.assertEqual((st.group(2), st.group(3), st.group(4), st.group(5), st.group(6), st.group(7)), ("DUPLEX", "+", "+", "w", "4", "4"), self.log())
            self.send(bus, "phase", 2)  # pre-launch needs three voters
            self.send(bus, "reintegrate", 2)  # the promotion
            self.pump(bus, 3.0)
            st = self.last_status()
            self.assertEqual((st.group(2), st.group(3), st.group(4), st.group(5), st.group(6), st.group(7)), ("TRIPLEX", "+", "+", "+", "4", "0"), self.log())
            self.send(bus, "phase", 2)  # and now it can be reached
            self.pump(bus, 1.5)  # (the status line comes once a second)
        finally:
            bus.close()
        text = self.log()
        self.assertIn("GROUND COMMAND noop: accepted", text)
        self.assertIn("GROUND COMMAND phase P4: accepted", text)
        self.assertIn("GROUND COMMAND warm C: accepted", text)
        self.assertIn("GROUND COMMAND phase P2: refused: the mission phase needs more computers", text)
        self.assertIn("GROUND COMMAND phase P2: accepted", text)
        self.assertEqual(self.last_status().group(6), "2")
        for i in range(3):
            self.assertNotRegex(self.log(i), r"node [ABC] LATCHED OUT")
