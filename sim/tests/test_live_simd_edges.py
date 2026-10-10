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
        time.sleep(0.8)  # tfc_simd designs the reference vehicle's gain tables before it listens: 0.36 s on the developer's machine (measured on 8 Oct 2026, the same on main before the flight console), so 0.3 s lost the first frames

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

    def test_with_hold_the_vehicle_waits_for_t_zero_and_a_late_simulator_joins_the_flight(self):
        self.proc.terminate()
        self.proc.wait(timeout=5)
        self.proc = subprocess.Popen([SIMD, "--iface", "vcan0", "--hold", "--quiet"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        time.sleep(0.3)
        flags_mon = B.SocketCanBus("vcan0")
        flags_mon.set_filter([(P.ID_SIM_FLAGS, 0x7FF)])
        try:
            def last_time_frames(seconds):
                best = None
                end = time.monotonic() + seconds
                while time.monotonic() < end:
                    f = flags_mon.recv(0.05)
                    if f is not None and (s := P.unpack_sim_flags(f)) is not None:
                        best = s.time_frames
                return best
            # on the pad (mission 0): flight time stays 0
            for k in range(0, 40):
                self.bus.send(0, P.pack_sync(k, k & 0xFF, 0))
                time.sleep(0.002)
                self.bus.send(0, P.pack_act_out(P.ActOut(state=1), k & 0xFF))
                time.sleep(0.008)
            self.assertEqual(last_time_frames(0.1), 0)
            # T-zero: the mission frame reaches 1001 and the clamps open
            for i in range(60):
                k = 40 + i
                self.bus.send(0, P.pack_sync(k, k & 0xFF, 1001 + i))
                time.sleep(0.002)
                self.bus.send(0, P.pack_act_out(P.ActOut(state=1), k & 0xFF))
                time.sleep(0.008)
            self.assertGreater(last_time_frames(0.1), 40)
        finally:
            flags_mon.close()
        # a simulator that starts after T-zero joins at the flight age the mission frame says (here 2000 frames of flight at frame 9000)
        self.proc.terminate()
        self.proc.wait(timeout=5)
        self.proc = subprocess.Popen([SIMD, "--iface", "vcan0", "--hold", "--quiet"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        time.sleep(0.3)
        mon = B.SocketCanBus("vcan0")
        mon.set_filter([(P.ID_SIM_FLAGS, 0x7FF)])
        try:
            got = None
            for i in range(400):
                k = 9000 + i
                self.bus.send(0, P.pack_sync(k, k & 0xFF, 1001 + 2000 + i))
                time.sleep(0.002)
                self.bus.send(0, P.pack_act_out(P.ActOut(state=1), k & 0xFF))
                time.sleep(0.008)
                f = mon.recv(0.0)
                while f is not None:
                    s = P.unpack_sim_flags(f)
                    if s is not None:
                        got = s.time_frames
                    f = mon.recv(0.0)
                if got is not None and got >= 2000:
                    break
            self.assertIsNotNone(got)
            self.assertGreaterEqual(got, 2000)
            self.assertLess(got, 2400)
        finally:
            mon.close()

    def test_a_long_flight_is_followed_a_starship_class_ascent_is_450_s_after_the_pad_and_the_countdown_and_a_number_beyond_the_limit_is_not(self):
        """The limit on a SYNC frame number is 100,000 (1,000 s), no longer 30,000: a late simulator joins a flight 45,000 frames old at frame 60,000, and a number past the limit is ignored."""
        self.proc.terminate()
        self.proc.wait(timeout=5)
        self.proc = subprocess.Popen([SIMD, "--iface", "vcan0", "--hold", "--quiet"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        time.sleep(0.3)
        mon = B.SocketCanBus("vcan0")
        mon.set_filter([(P.ID_SIM_FLAGS, 0x7FF)])

        def run(frame0, mission0, count):
            got = None
            for i in range(count):
                k = frame0 + i
                self.bus.send(0, P.pack_sync(k, k & 0xFF, mission0 + i))
                time.sleep(0.002)
                self.bus.send(0, P.pack_act_out(P.ActOut(state=1), k & 0xFF))
                time.sleep(0.008)
                f = mon.recv(0.0)
                while f is not None:
                    s = P.unpack_sim_flags(f)
                    if s is not None:
                        got = s.time_frames
                    f = mon.recv(0.0)
            return got
        try:
            self.assertIsNone(run(100_001, 1001 + 45000, 60), "a SYNC frame number past 100,000 is not a frame number of this run: ignored, the world does not move")
            got = None
            for start in range(0, 600, 60):               # the fast-forward to that age takes seconds in a debug build: keep sending until it answers
                got = run(60_000 + start, 1001 + 45000 + start, 60)
                if got is not None and got >= 45000:
                    break
            self.assertIsNotNone(got)
            self.assertGreaterEqual(got, 45000)
            self.assertLess(got, 46000)
        finally:
            mon.close()

    def test_without_acts_frames_the_simulator_holds_and_keeps_going(self):
        self.assertGreater(self.send_frames(range(0, 100), act=False), 80)  # a sensor frame for every SYNC, from the held command
        self.assertIsNone(self.proc.poll())


if __name__ == "__main__":
    unittest.main()
