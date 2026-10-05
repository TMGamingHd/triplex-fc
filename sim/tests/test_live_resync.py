# SPDX-License-Identifier: MIT
"""Live: the state resynchronisation (docs/design/RESYNC.md) between three instances of the real firmware on vcan0.

Computers A and C are the flight-function images; B is the same with a test knob (tools/bench/sil_triplex.sh --build-drop) that withholds every sensor frame from
its flight function in frames 370 to 398, so its estimator holds its rates while the other two follow the vote: B's state is degrees away at frame 399, the last frame of
the fourth resync period, and its commands leave the vote's tolerance at once, so the fault manager latches B out (rightly: a computer that far off is not to be believed).
The test reads the bus and the consoles and checks that
  - every computer sends its state in four frames in the last frame of each period, and not elsewhere,
  - at frame 399 B's shared state is far from A's and C's, which are the same words,
  - B is latched out, and at the resync B takes the vote of the two healthy computers: from frame 400 the three digests are equal again,
  - an operator's `reintegrate B` then succeeds: B passes its probation (which compares digests: before the resync that was impossible) and the system is Triplex again.
Skipped unless build/flight_a, _c and build/flight_b_drop exist and vcan0 is present.
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


BINS = [_bin("TFC_FLIGHT_BIN_A", "build/flight_a/zephyr/zephyr.exe"), _bin("TFC_FLIGHT_BIN_B_DROP", "build/flight_b_drop/zephyr/zephyr.exe"),
        _bin("TFC_FLIGHT_BIN_C", "build/flight_c/zephyr/zephyr.exe")]
HAVE_VCAN = Path("/sys/class/net/vcan0").exists()
RESYNC_FRAME = 399
REINTEGRATE_FRAME = 450
LAST_FRAME = 800
STATUS = re.compile(r"\[frame (\d+)\] (\w+)\s+A(.) B(.) C(.)\s+\| crc=(\d+)")


@unittest.skipIf(None in BINS, "the images are not built (tools/bench/sil_triplex.sh --build --flight and --build-drop)")
@unittest.skipIf(not HAVE_VCAN, "vcan0 not present (run sim/scripts/setup_vcan.sh)")
class LiveResync(unittest.TestCase):
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
        with open(path, "w") as f:
            self.procs.append(subprocess.Popen([BINS[i]], stdout=f, stderr=subprocess.STDOUT, text=True))
        self.logs.append(path)

    def log(self, i):
        return Path(self.logs[i]).read_text()

    def record(self):
        """Run the three computers; send `reintegrate B` once the resync is behind us. Returns (digests[frame][node], shares[frame][node][chunk] = words)."""
        monitor = B.SocketCanBus("vcan0")
        monitor.set_filter([(P.ID_SYNC, 0x7FF), (P.ID_CMD_BASE, 0x7FC), (P.ID_RESYNC, 0x7F0)])
        digests: dict[int, dict[int, int]] = {}
        shares: dict[int, dict[int, dict[int, tuple[int, int, int]]]] = {}
        try:
            for i in range(3):
                self.start(i)
            frame = None
            sent = False
            t_end = time.monotonic() + 25.0
            while time.monotonic() < t_end and (frame is None or frame < LAST_FRAME):
                f = monitor.recv(0.2)
                if f is None:
                    continue
                if (s := P.unpack_sync(f)) is not None:
                    frame = s.frame_no
                    if frame >= REINTEGRATE_FRAME and not sent:
                        monitor.send(0, P.pack_ground(P.GROUND_OPS["reintegrate"], 1, 1, None, False, False))
                        sent = True
                    continue
                if frame is None:
                    continue
                if P.ID_CMD_BASE <= f.id < P.ID_CMD_BASE + 3 and (c := P.unpack_cmd(f)) is not None:
                    digests.setdefault(frame, {})[f.id - P.ID_CMD_BASE] = c.digest
                elif (r := P.unpack_resync(f)) is not None:
                    shares.setdefault(frame, {}).setdefault(r[0], {})[r[1]] = r[2]
        finally:
            monitor.close()
        return digests, shares

    def test_a_diverged_replica_takes_the_vote_agrees_again_and_rejoins(self):
        digests, shares = self.record()
        self.assertGreaterEqual(max(digests), LAST_FRAME - 5, "the run did not reach the frames under test")
        # the state is sent in the last frame of each period, by every computer, in four frames, and in no other frame
        self.assertTrue(shares, "no resync frame on the bus")
        self.assertTrue(all(k % 100 == 99 for k in shares), sorted(shares))
        for k in (99, 199, 299, 399):
            if k in shares:
                complete = [n for n in range(3) if len(shares[k].get(n, {})) == 4]
                self.assertEqual(complete, [0, 1, 2], f"frame {k}: nodes with all four chunks {complete}")
        self.assertIn(RESYNC_FRAME, shares, "the fourth resync was not seen")
        words = {n: [w for c in range(4) for w in shares[RESYNC_FRAME][n][c]] for n in range(3)}
        self.assertEqual(words[0], words[2], "A and C see the same inputs: their shared states must be the same words")
        far = max(abs(a - b) for a, b in zip(words[0][:4], words[1][:4]))
        self.assertGreater(far, 150, f"B's attitude should be far from the others' (largest quaternion word difference {far} lsb)")
        # B was thrown out of the vote for its commands, rightly
        self.assertRegex(self.log(0), r"node B LATCHED OUT: vote disagreement", self.log(0))
        for n in (0, 2):
            self.assertNotRegex(self.log(n), r"node [AC] LATCHED OUT", self.log(n))
        # digests: B's differs before the resync, all three agree from the frame after it
        before = [k for k in range(RESYNC_FRAME - 14, RESYNC_FRAME) if len(digests.get(k, {})) == 3 and digests[k][1] != digests[k][0]]
        self.assertGreaterEqual(len(before), 10, f"B's digest should differ in the frames before the resync ({len(before)} of 14)")
        after = [k for k in range(RESYNC_FRAME + 1, RESYNC_FRAME + 61) if len(digests.get(k, {})) == 3]
        agree = [k for k in after if len(set(digests[k].values())) == 1]
        self.assertGreaterEqual(len(after), 50, "too few frames with all three commands")
        self.assertGreaterEqual(len(agree), len(after) - 3, f"the digests should agree again after the resync ({len(agree)} of {len(after)})")
        # the consoles: B's state was replaced by the vote of the two healthy computers; the others had nothing to correct
        self.assertRegex(self.log(1), rf"\[frame {RESYNC_FRAME}\] RESYNC: adopted the vote of 2 computers; .*\(this computer's state was replaced\)", self.log(1))
        for n in (0, 2):
            self.assertNotRegex(self.log(n), rf"\[frame {RESYNC_FRAME}\] RESYNC", self.log(n))
        for n in range(3):
            self.assertNotRegex(self.log(n), r"\[frame (?:99|199|299)\] RESYNC", "the earlier resyncs found nothing to correct")
        # and B rejoins: it passes its probation, which needs equal digests
        self.assertRegex(self.log(0), r"node B REINTEGRATED into the vote", self.log(0))
        last = [m for m in map(STATUS.match, self.log(0).splitlines()) if m][-1]
        self.assertEqual((last.group(2), last.group(3), last.group(4), last.group(5)), ("TRIPLEX", "+", "+", "+"), self.log(0))


if __name__ == "__main__":
    unittest.main()
