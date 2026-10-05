# SPDX-License-Identifier: MIT
"""Live: edge cases of tfc_simd on vcan0 with Python sending the SYNC and ACT frames: a corrupt frame number, a number that goes backwards, a jump forward, ACT absent.
The simulator must stay responsive and keep publishing sensor inputs. Skipped unless build/host/tfc_simd and vcan0 exist."""
import os
import subprocess
import time
import unittest
from pathlib import Path

from tfc_peers import bus as B
from tfc_peers import protocol as P

ROOT = Path(__file__).resolve().parents[2]
SIMD = os.environ.get("TFC_SIMD_BIN") or str(ROOT / "build/host/tfc_simd")


@unittest.skipIf(not os.access(SIMD, os.X_OK), "tfc_simd is not built")
@unittest.skipIf(not Path("/sys/class/net/vcan0").exists(), "vcan0 not present")
class SimdEdges(unittest.TestCase):
    def setUp(self):
        self.proc = subprocess.Popen([SIMD, "--iface", "vcan0", "--quiet"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        self.bus = B.SocketCanBus("vcan0")
        self.mon = B.SocketCanBus("vcan0")
        self.mon.set_filter([(P.ID_SIM_RATES, 0x7FF)])
        time.sleep(0.3)

    def tearDown(self):
        self.proc.terminate()
        self.proc.wait(timeout=5)
        self.bus.close()
        self.mon.close()

    def frame(self, k, act=True):
        self.bus.send(0, P.pack_sync(k, k & 0xFF))
        if act:
            time.sleep(0.002)
            self.bus.send(0, P.pack_act_out(P.ActOut(state=1), k & 0xFF))

    def send_frames(self, numbers, act=True):
        """Send SYNC (and ACT's frame) for each number at 100 Hz; return how many simulator sensor frames came back meanwhile."""
        n = 0
        for k in numbers:
            self.frame(k, act)
            end = time.monotonic() + 0.01
            while time.monotonic() < end:
                if self.mon.recv(max(0.0, end - time.monotonic())) is not None:
                    n += 1
        return n

    def test_a_corrupt_frame_number_does_not_hang_the_simulator(self):
        self.assertGreater(self.send_frames(range(5)), 0)
        self.frame(4_000_000_000)  # hours of frames: must be ignored, not stepped
        time.sleep(0.2)
        self.assertIsNone(self.proc.poll(), "tfc_simd must still be running")
        self.assertGreater(self.send_frames(range(5, 40)), 20, "and still publishing, a frame for nearly every SYNC")

    def test_a_number_going_backwards_starts_a_new_run_and_a_big_jump_forward_too(self):
        self.assertGreater(self.send_frames(range(300, 330)), 20)
        self.assertGreater(self.send_frames(range(5, 20)), 10)           # backwards
        # forwards by far more than a minute: a new run, which brings the world to that frame first (seconds in a debug build, so keep sending until it answers)
        seen = 0
        for start in range(7000, 7000 + 600, 20):
            seen += self.send_frames(range(start, start + 20))
            if seen > 10:
                break
        self.assertGreater(seen, 10)
        self.assertIsNone(self.proc.poll())

    def test_without_acts_frames_the_simulator_holds_and_keeps_going(self):
        self.assertGreater(self.send_frames(range(0, 100), act=False), 80)  # a sensor frame for every SYNC, from the held command
        self.assertIsNone(self.proc.poll())


if __name__ == "__main__":
    unittest.main()
