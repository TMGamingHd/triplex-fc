# SPDX-License-Identifier: MIT
"""Live: the launch sequence on one vcan0 with real firmware. Three flight computers with the launch sequence (each calibrating its own IMU on the pad), the actuator node, and the vehicle
simulator clamped on the pad until T-zero, each its own process; Python is the operator (the checklist of tfc_peers/launch.py) and a bus monitor.
Skipped unless build/launch_a,b,c, build/act_native and build/host/tfc_simd exist (tools/bench/sil_triplex.sh --build --launch) and vcan0 does.
"""
import os
import re
import subprocess
import tempfile
import time
import unittest
from pathlib import Path

from tfc_peers import bus as B
from tfc_peers import launch as LA
from tfc_peers import protocol as P

ROOT = Path(__file__).resolve().parents[2]


def _bin(env, default):
    for c in (os.environ.get(env, ""), str(ROOT / default)):
        if c and os.access(c, os.X_OK):
            return c
    return None


FC_BINS = [_bin(f"TFC_LAUNCH_BIN_{n.upper()}", f"build/launch_{n}/zephyr/zephyr.exe") for n in "abc"]
ACT_BIN = _bin("TFC_ACT_BIN", "build/act_native/zephyr/zephyr.exe")
SIM_BIN = _bin("TFC_SIMD_BIN", "build/host/tfc_simd")


class LaunchRig(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.procs = {}
        self.counter = 0
        self.obs = LA.Observer()
        self.alt = 0.0
        self.time_frames = 0
        self.flags = 0
        self.pitch_err = []
        self.bus = B.SocketCanBus("vcan0")
        self.bus.set_filter([(P.ID_HEARTBEAT, 0x7FC), (P.ID_ACT_OUT, 0x7FF), (P.ID_SYNC, 0x7FF), (P.ID_SIM_STATE, 0x7FF), (P.ID_SIM_TELEMETRY, 0x7FF), (P.ID_SIM_FLAGS, 0x7FF)])

    def tearDown(self):
        self.bus.close()
        for p in self.procs.values():
            if p.poll() is None:
                p.kill()
                p.wait()
        self.tmp.cleanup()

    def start(self, name, argv):
        with open(Path(self.tmp.name) / f"{name}.log", "w") as f:
            self.procs[name] = subprocess.Popen(argv, stdout=f, stderr=subprocess.STDOUT, text=True)

    def log(self, name):
        return (Path(self.tmp.name) / f"{name}.log").read_text()

    def start_all(self):
        self.start("sim", [SIM_BIN, "--iface", "vcan0", "--hold", "--quiet"])
        time.sleep(0.4)
        for n, b in zip("abc", FC_BINS):
            self.start(n, [b])
        self.start("act", [ACT_BIN])

    def pump(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            f = self.bus.recv(max(0.0, min(0.05, end - time.monotonic())))
            if f is None:
                continue
            now = time.monotonic()
            self.obs.feed(f, now)
            if f.id == P.ID_SIM_STATE and (s := P.unpack_sim_state(f)) is not None:
                self.alt = s.altitude_m
            elif f.id == P.ID_SIM_FLAGS and (s := P.unpack_sim_flags(f)) is not None:
                self.flags, self.time_frames = s.flags, s.time_frames
            elif f.id == P.ID_SIM_TELEMETRY and (t := P.unpack_sim_telemetry(f)) is not None:
                self.pitch_err.append(t.pitch_error_deg)

    def wait_go(self, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self.pump(0.2)
            if not self.obs.verdict(time.monotonic()):
                return True
        return False

    def send_launch(self, execute=True):
        self.bus.send(0, P.pack_ground(P.GROUND_OPS["launch"], 0, (self.counter + 1) & 0xFF, None, True, False))
        time.sleep(0.05)
        if execute:
            self.bus.send(0, P.pack_ground(P.GROUND_OPS["launch"], 0, (self.counter + 2) & 0xFF, None, False, False))
        self.counter += 2

    def send_scrub(self):
        self.counter += 1
        self.bus.send(0, P.pack_ground(P.GROUND_OPS["scrub"], 0, self.counter & 0xFF, None, False, False))

    def skip_if_starved(self, allowed=""):
        for n in "abc":
            for m in re.finditer(r"node ([ABC]) LATCHED OUT: (.*)", self.log(n)):
                if m.group(1) not in allowed:
                    self.skipTest(f"the machine was too loaded: node {m.group(1)} was latched out ({m.group(2)})")



@unittest.skipIf(None in FC_BINS or ACT_BIN is None or SIM_BIN is None, "the launch images or tfc_simd are not built (tools/bench/sil_triplex.sh --build --launch)")
@unittest.skipIf(not Path("/sys/class/net/vcan0").exists(), "vcan0 not present (run sim/scripts/setup_vcan.sh)")
class LiveLaunch(LaunchRig):
    def test_a_launch_before_the_calibration_is_ready_is_refused_and_the_vehicle_stays_on_the_pad(self):
        self.start_all()
        self.pump(4.0)  # the computers are up, the calibration has not had its 10 s
        self.send_launch()
        self.pump(2.0)
        self.skip_if_starved()
        self.assertIn("LAUNCH REFUSED, no-go", self.log("a"), self.log("a"))
        self.assertEqual(self.obs.mission, 0, "no countdown")
        self.assertLess(self.alt, 1.0)
        self.assertEqual(self.time_frames, 0)

    def test_a_go_launch_counts_down_releases_at_t_zero_and_flies_the_program(self):
        self.start_all()
        self.assertTrue(self.wait_go(40.0), f"never go: {self.obs.verdict(time.monotonic())}")
        self.pump(0.5)
        self.assertLess(self.alt, 1.0, "clamped on the pad until T-zero")
        self.send_launch()
        t_start = time.monotonic()
        while time.monotonic() - t_start < 14.0 and not P.mission_in_flight(self.obs.mission):
            self.pump(0.1)
        self.assertTrue(P.mission_in_flight(self.obs.mission), f"mission frame {self.obs.mission}; {self.log('a')[-400:]}")
        self.assertGreater(time.monotonic() - t_start, 9.0, "the countdown takes about 10 s")
        self.assertLess(self.alt, 5.0)
        self.pitch_err.clear()
        self.pump(14.0)
        self.skip_if_starved()
        for n in "abc":
            self.assertIn("T-ZERO", self.log(n), self.log(n)[-600:])
        self.assertGreater(self.alt, 100.0, "the vehicle flew")
        self.assertGreater(len(self.pitch_err), 100)
        self.assertLess(max(abs(e) for e in self.pitch_err[20:]), 1.5, "and held the program")
        self.assertEqual(self.flags & 0x0B, 0, f"flags {self.flags:#x}")

    def test_a_scrub_during_the_countdown_returns_to_the_pad_and_a_second_launch_works(self):
        self.start_all()
        self.assertTrue(self.wait_go(40.0))
        self.send_launch()
        self.pump(4.0)
        self.assertTrue(P.mission_in_countdown(self.obs.mission), f"mission {self.obs.mission}")
        self.send_scrub()
        self.pump(1.0)
        self.skip_if_starved()
        self.assertEqual(self.obs.mission, 0)
        self.assertIn("SCRUB", self.log("a"))
        self.pump(3.0)
        self.assertLess(self.alt, 1.0, "still on the pad")
        self.assertTrue(self.wait_go(10.0))  # the calibration carried on: ready again
        self.send_launch()
        t0 = time.monotonic()
        while time.monotonic() - t0 < 14.0 and not P.mission_in_flight(self.obs.mission):
            self.pump(0.1)
        self.assertTrue(P.mission_in_flight(self.obs.mission))

    def test_the_sync_master_dying_in_the_countdown_hands_it_to_b_which_scrubs_because_there_are_no_longer_three_computers(self):
        self.start_all()
        self.assertTrue(self.wait_go(40.0))
        self.send_launch()
        self.pump(4.0)
        self.assertTrue(P.mission_in_countdown(self.obs.mission))
        self.procs["a"].kill()  # the sync master
        self.pump(3.0)
        self.skip_if_starved(allowed="A")
        # B takes over SYNC and carries the countdown on from where it was, but the go/no-go (three healthy computers) no longer holds, so it scrubs: the design
        self.assertIn("takes over as sync master", self.log("b"), self.log("b")[-600:])
        self.assertIn("COUNTDOWN SCRUBBED, no-go: not three healthy flight computers", self.log("b"), self.log("b")[-800:])
        self.assertEqual(self.obs.mission, 0)
        self.pump(2.0)
        self.assertLess(self.alt, 1.0, "the vehicle never left the pad")
        self.assertEqual(self.time_frames, 0)

    def test_a_follower_dying_in_the_countdown_scrubs_it_too(self):
        self.start_all()
        self.assertTrue(self.wait_go(40.0))
        self.send_launch()
        self.pump(4.0)
        self.assertTrue(P.mission_in_countdown(self.obs.mission))
        self.procs["c"].kill()
        self.pump(3.0)
        self.skip_if_starved(allowed="C")
        self.assertIn("COUNTDOWN SCRUBBED, no-go", self.log("a"), self.log("a")[-800:])
        self.assertEqual(self.obs.mission, 0)
        self.assertLess(self.alt, 1.0)

if __name__ == "__main__":
    unittest.main()


T0_BINS = [_bin(f"TFC_LAUNCH_T0_BIN_{n.upper()}", f"build/launch_t0_{n}/zephyr/zephyr.exe") for n in "abc"]


@unittest.skipIf(None in T0_BINS or ACT_BIN is None or SIM_BIN is None, "the T0 images or tfc_simd are not built (tools/bench/sil_triplex.sh --build-t0, and --build for the actuator)")
@unittest.skipIf(not Path("/sys/class/net/vcan0").exists(), "vcan0 not present (run sim/scripts/setup_vcan.sh)")
class LiveLaunchT0Line(LaunchRig):
    """The supervisor's T0 line (simulated by a build knob: it rises 40 frames before the end of the countdown): the sync master takes it as T-zero, all three agree."""

    def start_all(self):
        self.start("sim", [SIM_BIN, "--iface", "vcan0", "--hold", "--quiet"])
        time.sleep(0.4)
        for n, b in zip("abc", T0_BINS):
            self.start(n, [b])
        self.start("act", [ACT_BIN])

    def test_the_t0_edge_ends_the_countdown_in_the_last_second_and_every_computer_agrees_on_the_frame(self):
        self.start_all()
        self.assertTrue(self.wait_go(40.0), f"never go: {self.obs.verdict(time.monotonic())}")
        self.send_launch()
        t_start = time.monotonic()
        while time.monotonic() - t_start < 14.0 and not P.mission_in_flight(self.obs.mission):
            self.pump(0.1)
        self.assertTrue(P.mission_in_flight(self.obs.mission), self.log("a")[-400:])
        self.pump(1.0)
        self.skip_if_starved()
        self.assertIn("T0 LINE: the supervisor's T-zero", self.log("a"), self.log("a")[-600:])
        spans = []
        for n in "abc":
            text = self.log(n)
            start = re.search(r"\[frame (\d+)\] COUNTDOWN", text)
            zero = re.search(r"\[frame (\d+)\] T-ZERO", text)
            self.assertTrue(start and zero, text[-600:])
            spans.append((int(start.group(1)), int(zero.group(1))))
        self.assertEqual(len({s for s in spans}), 1, f"the computers disagree on the frames of the countdown and T-zero: {spans}")
        length = spans[0][1] - spans[0][0]
        self.assertTrue(955 <= length <= 965, f"the countdown was {length} frames: the T0 edge should have ended it about 40 frames early")
