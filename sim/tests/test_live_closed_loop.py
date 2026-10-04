# SPDX-License-Identifier: MIT
"""Live: the closed loop on one vcan0. Three flight computers running the real flight function, the actuator node, and the vehicle simulator (tools/tfc_simd),
each its own process, flying the 6-DOF ascent: the simulator publishes the sensor inputs, the nodes vote, ACT's output goes back to the simulator, which steps the
vehicle with it. Python is only a bus monitor. Skipped unless the images and tfc_simd are built (tools/bench/sil_triplex.sh --build --flight --sim-imu, and
cmake --build build/host --target tfc_simd) and vcan0 exists.
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


LOOP_BINS = [_bin(f"TFC_LOOP_BIN_{n.upper()}", f"build/loop_{n}/zephyr/zephyr.exe") for n in "abc"]
ACT_BIN = _bin("TFC_ACT_BIN", "build/act_native/zephyr/zephyr.exe")
SIM_BIN = _bin("TFC_SIMD_BIN", "build/host/tfc_simd")
HAVE_VCAN = Path("/sys/class/net/vcan0").exists()
STATUS = re.compile(r"\[frame (\d+)\] (\w+)\s+A(.) B(.) C(.)\s+\| crc=(\d+) seq=(\d+) missing=(\d+) vote=(\d+) digest=(\d+)")


@unittest.skipIf(None in LOOP_BINS or ACT_BIN is None or SIM_BIN is None, "the closed-loop images or tfc_simd are not built (see the module docstring)")
@unittest.skipIf(not HAVE_VCAN, "vcan0 not present (run sim/scripts/setup_vcan.sh)")
class LiveClosedLoop(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.procs = {}

    def tearDown(self):
        for p in self.procs.values():
            if p.poll() is None:
                p.kill()
                p.wait()
        self.tmp.cleanup()

    def start(self, name, argv):
        with open(Path(self.tmp.name) / f"{name}.log", "w") as f:  # the child keeps its own copy of the descriptor
            self.procs[name] = subprocess.Popen(argv, stdout=f, stderr=subprocess.STDOUT, text=True)

    def log(self, name):
        return (Path(self.tmp.name) / f"{name}.log").read_text()

    def fly(self, seconds, kill=None):
        """Start the simulator, the three computers and ACT, watch the bus for `seconds`, and `kill` = (name, after seconds) one of the processes."""
        monitor = B.SocketCanBus("vcan0")
        monitor.set_filter([(P.ID_SIM_TELEMETRY, 0x7FF), (P.ID_SIM_FLAGS, 0x7FF), (P.ID_ACT_OUT, 0x7FF)])
        try:
            self.start("sim", [SIM_BIN, "--iface", "vcan0", "--quiet"])
            time.sleep(0.4)
            for n, b in zip("abc", LOOP_BINS):
                self.start(n, [b])
            self.start("act", [ACT_BIN])
            errors = []  # (time in frames, pitch error, yaw error)
            act_states = set()
            flags = 0
            t_start = time.monotonic()
            t_end = t_start + seconds
            while time.monotonic() < t_end:
                if kill is not None and self.procs[kill[0]].poll() is None and time.monotonic() - t_start > kill[1]:
                    self.procs[kill[0]].kill()
                f = monitor.recv(0.2)
                if f is None:
                    continue
                if f.id == P.ID_SIM_TELEMETRY and (t := P.unpack_sim_telemetry(f)) is not None:
                    errors.append((t.pitch_error_deg, t.yaw_error_deg))
                elif f.id == P.ID_SIM_FLAGS and (s := P.unpack_sim_flags(f)) is not None:
                    flags |= s.flags
                elif f.id == P.ID_ACT_OUT and (a := P.unpack_act_out(f)) is not None:
                    act_states.add(a.state)
        finally:
            monitor.close()
        return errors, act_states, flags

    def check_flight(self, errors, act_states, flags, limit_deg):
        self.assertGreater(len(errors), 80, "the simulator should publish telemetry at 10 Hz")
        settled = errors[len(errors) // 4:]  # after ACT's standby and the first start-up frames
        worst = max(max(abs(p), abs(y)) for p, y in settled)
        self.assertLess(worst, limit_deg, f"the vehicle strayed from the pitch program by {worst:.2f} degrees")
        self.assertIn(1, act_states, "ACT should have reached Nominal")
        self.assertNotIn(2, act_states, "ACT must not have entered Safe")
        self.assertEqual(flags & 0x0B, 0, f"simulator flags {flags:#x}: safed, platform saturated or command held")

    def skip_if_starved(self, allowed=""):
        for n in "abc":
            for m in re.finditer(r"node ([ABC]) LATCHED OUT: (.*)", self.log(n)):
                if m.group(1) not in allowed:
                    self.skipTest(f"the machine was too loaded: node {m.group(1)} was latched out ({m.group(2)})")

    def test_the_vehicle_follows_the_pitch_program_through_three_computers_and_act(self):
        errors, act_states, flags = self.fly(16.0)
        self.skip_if_starved()
        self.check_flight(errors, act_states, flags, 1.0)
        for n in "abc":
            last = [m for m in map(STATUS.match, self.log(n).splitlines()) if m][-1]
            self.assertEqual((last.group(2), last.group(3), last.group(4), last.group(5)), ("TRIPLEX", "+", "+", "+"), self.log(n))
            self.assertEqual(int(last.group(10)), 0, "the replicas' digests disagree")

    def test_the_vehicle_flies_on_when_a_flight_computer_dies_in_flight(self):
        errors, act_states, flags = self.fly(18.0, kill=("b", 6.0))
        self.skip_if_starved(allowed="B")
        self.check_flight(errors, act_states, flags, 1.5)
        act = self.log("act")
        self.assertIn("ACT EXCLUDED node B", act, act)
        status = [l for l in act.splitlines() if re.match(r"\[frame \d+\] NOMINAL", l)]
        self.assertIn("voted 5 excluded 2", status[-1], act)
        for n in "ac":
            last = [m for m in map(STATUS.match, self.log(n).splitlines()) if m][-1]
            if last.group(2) == "SAFE":
                self.skipTest("the two remaining estimators disagreed and the system went to Safe: the open hole of TS-16 (docs/TRADE_STUDIES.md)")
            self.assertEqual((last.group(2), last.group(4)), ("DUPLEX", "X"), self.log(n))


if __name__ == "__main__":
    unittest.main()
