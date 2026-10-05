# SPDX-License-Identifier: MIT
"""Live: computers of different releases (ADR-021, TFC-ARCH-004 and ARCH-007) between three instances of the real firmware on vcan0.

Computers A and B report release 0xA001, computer C release 0xB002 (tools/bench/sil_triplex.sh --build-mixed). In one run C's commands are 0.05 degree off, beyond the version tolerance
(1.5 times the vote tolerance of 0.01 degree): nobody may be latched out, the outputs are held and a Safe request is raised; the operator then trusts the pair (disable C, then clear Safe under
an ARM) and the system flies on in Duplex. In the other run C's commands are 0.012 degree off, inside the tolerance: nothing may happen at all, all three stay in the vote.
Skipped unless the mixed images are built and vcan0 is present.
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


A, Bn, C, C_NEAR = (_bin("TFC_MIXED_BIN_A", "build/mixed_a/zephyr/zephyr.exe"), _bin("TFC_MIXED_BIN_B", "build/mixed_b/zephyr/zephyr.exe"),
                    _bin("TFC_MIXED_BIN_C", "build/mixed_c/zephyr/zephyr.exe"), _bin("TFC_MIXED_BIN_C_NEAR", "build/mixed_c_near/zephyr/zephyr.exe"))
HAVE_VCAN = Path("/sys/class/net/vcan0").exists()
STATUS = re.compile(r"\[frame (\d+)\] (\w+)\s+A(.) B(.) C(.)\s+\| crc=(\d+)")


@unittest.skipIf(None in (A, Bn, C, C_NEAR), "the images are not built (tools/bench/sil_triplex.sh --build-mixed)")
@unittest.skipIf(not HAVE_VCAN, "vcan0 not present (run sim/scripts/setup_vcan.sh)")
class LiveRelease(unittest.TestCase):
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

    def start(self, binary):
        path = Path(self.tmp.name) / f"node{len(self.logs)}.log"
        with open(path, "w") as f:
            self.procs.append(subprocess.Popen([binary], stdout=f, stderr=subprocess.STDOUT, text=True))
        self.logs.append(path)

    def log(self, i):
        return Path(self.logs[i]).read_text()

    def last_status(self, i):
        return [m for m in map(STATUS.match, self.log(i).splitlines()) if m][-1]

    def run_for(self, seconds, monitor=None, send=None):
        t_end = time.monotonic() + seconds
        while time.monotonic() < t_end:
            if monitor is not None:
                monitor.recv(0.1)
            else:
                time.sleep(0.1)

    def test_a_disagreement_between_releases_isolates_nobody_holds_and_asks_and_the_operator_picks_a_side(self):
        for b in (A, Bn, C):
            self.start(b)
        monitor = B.SocketCanBus("vcan0")
        monitor.set_filter([(P.ID_SYNC, 0x7FF)])
        try:
            self.run_for(5.0, monitor)
            for i in range(3):
                self.assertIn("release 0x", self.log(i))
            self.assertIn("release 0xa001", self.log(0))
            self.assertIn("release 0xb002", self.log(2))
            for i in range(3):
                self.assertNotRegex(self.log(i), r"node [ABC] LATCHED OUT", self.log(i))  # nobody isolated
                self.assertIn("SAFE REQUESTED", self.log(i), self.log(i))
                self.assertEqual(self.last_status(i).group(2), "SAFE", self.log(i))
                self.assertEqual((self.last_status(i).group(3), self.last_status(i).group(4), self.last_status(i).group(5)), ("+", "+", "+"), self.log(i))
            # the operator trusts the pair: disable C (Triplex to Duplex is plain), then clear Safe under an ARM
            monitor.send(0, P.pack_ground(P.GROUND_OPS["disable"], 2, 1, None, False, False))
            self.run_for(0.2, monitor)
            monitor.send(0, P.pack_ground(P.GROUND_OPS["clear-safe"], 0, 2, None, True, False))
            self.run_for(0.1, monitor)
            monitor.send(0, P.pack_ground(P.GROUND_OPS["clear-safe"], 0, 3, None, False, False))
            self.run_for(3.0, monitor)
        finally:
            monitor.close()
        for i in (0, 1):
            last = self.last_status(i)
            self.assertEqual((last.group(2), last.group(3), last.group(4), last.group(5)), ("DUPLEX", "+", "+", "D"), self.log(i))

    def test_a_difference_inside_the_version_tolerance_is_not_a_fault_and_not_a_conflict(self):
        for b in (A, Bn, C_NEAR):
            self.start(b)
        monitor = B.SocketCanBus("vcan0")
        monitor.set_filter([(P.ID_SYNC, 0x7FF)])
        try:
            self.run_for(6.0, monitor)
        finally:
            monitor.close()
        for i in range(3):
            self.assertNotIn("SAFE REQUESTED", self.log(i), self.log(i))
            self.assertNotRegex(self.log(i), r"node [ABC] LATCHED OUT", self.log(i))
            last = self.last_status(i)
            self.assertEqual((last.group(2), last.group(3), last.group(4), last.group(5)), ("TRIPLEX", "+", "+", "+"), self.log(i))


if __name__ == "__main__":
    unittest.main()
