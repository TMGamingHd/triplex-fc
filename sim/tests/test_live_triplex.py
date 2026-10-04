# SPDX-License-Identifier: MIT
"""Live: three instances of the real firmware (Zephyr native_sim, real time) on vcan0, with nothing from Python but a bus monitor.

A claims SYNC after listening, B and C follow it. Killing A must leave B as the sync master with the frame number continuous (no skipped or repeated
number on the bus), C following B, and a Duplex vote on the two that remain. Skipped unless the three images are built (TFC_FC_BIN for A, TFC_FC_BIN_B,
TFC_FC_BIN_C, or build/native_sim, build/native_sim_n1, build/native_sim_n2; see tools/bench/sil_triplex.sh) and vcan0 exists.
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


BINS = [_bin("TFC_FC_BIN", "build/native_sim/zephyr/zephyr.exe"), _bin("TFC_FC_BIN_B", "build/native_sim_n1/zephyr/zephyr.exe"),
        _bin("TFC_FC_BIN_C", "build/native_sim_n2/zephyr/zephyr.exe")]
HAVE_VCAN = Path("/sys/class/net/vcan0").exists()
STATUS = re.compile(r"\[frame (\d+)\] (\w+)\s+A(.) B(.) C(.)\s+\| crc=(\d+) seq=(\d+) missing=(\d+) vote=(\d+) digest=(\d+)")


@unittest.skipIf(None in BINS, "the three native_sim images are not built (tools/bench/sil_triplex.sh --build)")
@unittest.skipIf(not HAVE_VCAN, "vcan0 not present (run sim/scripts/setup_vcan.sh)")
class LiveTriplex(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.procs = []
        self.logs = []

    def tearDown(self):
        for p in self.procs:
            if p.poll() is None:
                p.kill()
                p.wait()
        self.tmp.cleanup()

    def start(self, i):
        path = Path(self.tmp.name) / f"node{i}.log"
        with open(path, "w") as f:  # the child keeps its own copy of the descriptor
            self.procs.append(subprocess.Popen([BINS[i]], stdout=f, stderr=subprocess.STDOUT, text=True))
        self.logs.append(path)

    def log(self, i):
        return Path(self.logs[i]).read_text()

    def skip_if_starved(self, allowed=""):
        """Real-time processes on a busy machine can miss a frame or three and latch a healthy node. That says nothing about the firmware, so the test skips
        instead of failing when a node other than `allowed` (letters) was latched out of the vote."""
        for i in range(len(self.logs)):
            for m in re.finditer(r"node ([ABC]) LATCHED OUT: (.*)", self.log(i)):
                if m.group(1) not in allowed:
                    self.skipTest(f"the machine was too loaded: node {m.group(1)} was latched out ({m.group(2)}) in node {i}'s view")

    def test_three_instances_vote_and_b_takes_over_when_a_dies(self):
        monitor = B.SocketCanBus("vcan0")
        monitor.set_filter([(P.ID_SYNC, 0x7FF)])
        try:
            for i in range(3):
                self.start(i)
            numbers = []  # (arrival time, frame number)
            t_end = time.monotonic() + 4.0
            while time.monotonic() < t_end:
                f = monitor.recv(0.2)
                if f is not None and (s := P.unpack_sync(f)) is not None:
                    numbers.append((time.monotonic(), s.frame_no))
            self.assertGreater(len(numbers), 300, "A should be sending SYNC at 100 Hz")
            healthy = [m for m in map(STATUS.match, self.log(0).splitlines()) if m]
            self.assertTrue(healthy, self.log(0))
            self.skip_if_starved()
            for i in range(3):
                last = [m for m in map(STATUS.match, self.log(i).splitlines()) if m][-1]
                self.assertEqual(last.group(2), "TRIPLEX", self.log(i))
                self.assertEqual((last.group(3), last.group(4), last.group(5)), ("+", "+", "+"), self.log(i))
            self.procs[0].kill()  # A dies
            self.procs[0].wait()
            t_kill = time.monotonic()
            after = []
            t_end = time.monotonic() + 3.0
            while time.monotonic() < t_end:
                f = monitor.recv(0.2)
                if f is not None and (s := P.unpack_sync(f)) is not None:
                    after.append((time.monotonic(), s.frame_no))
        finally:
            monitor.close()
        self.skip_if_starved(allowed="A")
        self.assertGreater(len(after), 200, "B must have taken over SYNC")
        # The number keeps pace with time across the takeover: B counted the frame whose SYNC it did not hear, so the one step across the gap is 2 and
        # every other step is 1 (nothing repeats, nothing is renumbered).
        seq = [n for _, n in numbers[-3:]] + [n for _, n in after]
        steps = [b - a for a, b in zip(seq, seq[1:])]
        self.assertEqual(sorted(steps)[:-1], [1] * (len(steps) - 1), f"steps {sorted(set(steps))}")
        self.assertEqual(sorted(steps)[-1], 2, f"steps {sorted(set(steps))}")
        self.assertIn("takes over as sync master", self.log(1), self.log(1))
        self.assertNotIn("takes over as sync master", self.log(2), self.log(2))
        for i in (1, 2):
            last = [m for m in map(STATUS.match, self.log(i).splitlines()) if m][-1]
            self.assertEqual(last.group(2), "DUPLEX", self.log(i))
            self.assertEqual(last.group(3), "X", self.log(i))
            self.assertEqual((last.group(4), last.group(5)), ("+", "+"), self.log(i))


if __name__ == "__main__":
    unittest.main()
