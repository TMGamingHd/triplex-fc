# SPDX-License-Identifier: MIT
"""Live: the sensor/compute split (ADR-020 case 1, TFC-ARCH-001) between three instances of the real firmware on vcan0.

Computers A and C are the flight-function images; B is the same with a test knob (tools/bench/sil_triplex.sh --build-bias) that adds 3 dps to its gyro x, so its IMU is wrong and its
computer is not. The consoles must show that B's **IMU** is latched out of the sensor consensus and that **computer B stays in the command vote** (all three computers `+`, mode
TRIPLEX), and that the command of B on the bus still agrees with the others. Skipped unless build/flight_a, _c and build/flight_b_bias exist and vcan0 is present.
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


BINS = [_bin("TFC_FLIGHT_BIN_A", "build/flight_a/zephyr/zephyr.exe"), _bin("TFC_FLIGHT_BIN_B_BIAS", "build/flight_b_bias/zephyr/zephyr.exe"),
        _bin("TFC_FLIGHT_BIN_C", "build/flight_c/zephyr/zephyr.exe")]
HAVE_VCAN = Path("/sys/class/net/vcan0").exists()
STATUS = re.compile(r"\[frame (\d+)\] (\w+)\s+A(.) B(.) C(.)\s+\| crc=(\d+)")


@unittest.skipIf(None in BINS, "the images are not built (tools/bench/sil_triplex.sh --build --flight and --build-bias)")
@unittest.skipIf(not HAVE_VCAN, "vcan0 not present (run sim/scripts/setup_vcan.sh)")
class LiveSplit(unittest.TestCase):
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

    def test_a_computer_with_a_bad_imu_stays_a_voter_and_only_its_imu_is_excluded(self):
        monitor = B.SocketCanBus("vcan0")
        monitor.set_filter([(P.ID_SYNC, 0x7FF), (P.ID_CMD_BASE, 0x7FC)])
        cmds: dict[int, dict[int, tuple[float, float]]] = {}
        try:
            for i in range(3):
                self.start(i)
            frame = None
            t_end = time.monotonic() + 12.0
            while time.monotonic() < t_end and (frame is None or frame < 600):
                f = monitor.recv(0.2)
                if f is None:
                    continue
                if (s := P.unpack_sync(f)) is not None:
                    frame = s.frame_no
                elif frame is not None and P.ID_CMD_BASE <= f.id < P.ID_CMD_BASE + 3 and (c := P.unpack_cmd(f)) is not None:
                    cmds.setdefault(frame, {})[f.id - P.ID_CMD_BASE] = (c.pitch_deg, c.yaw_deg)
        finally:
            monitor.close()
        self.assertGreaterEqual(max(cmds), 550, "the run did not reach frame 550")
        for i in (0, 2):
            self.assertRegex(self.log(i), r"IMU B LATCHED OUT \(computer B stays in the command vote\)", self.log(i))
        for i in range(3):
            self.assertNotRegex(self.log(i), r"node [ABC] LATCHED OUT", self.log(i))   # no computer is removed
            last = [m for m in map(STATUS.match, self.log(i).splitlines()) if m][-1]
            self.assertEqual((last.group(2), last.group(3), last.group(4), last.group(5)), ("TRIPLEX", "+", "+", "+"), self.log(i))
        # B's own commands, from the consensus of the two good IMUs, still agree with the others' on the bus (within the manager's tolerance, in most frames)
        agree = sum(1 for k in range(300, 550) if len(cmds.get(k, {})) == 3 and max(abs(cmds[k][1][0] - cmds[k][0][0]), abs(cmds[k][1][0] - cmds[k][2][0])) <= 0.05)
        self.assertGreaterEqual(agree, 220, f"B's commands agree with the others' in {agree} of 250 frames")


if __name__ == "__main__":
    unittest.main()
