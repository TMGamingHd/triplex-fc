# SPDX-License-Identifier: MIT
"""Live: the actuator node's hardware SAFE line (ADR-023, ADR-022, TFC-SAFE-002) with the real firmware on vcan0.

Three flight computers vote and ACT follows them. ACT's image is built with the SAFE line reading as asserted from SYNC frame 500 (the bench aid CONFIG_TFC_TEST_SAFE_LINE_AT_FRAME, standing in for the
supervisor's `SAFE` line or the FORCE-SAFE switch): ACT must go from Nominal to Safe by itself, name the line as the cause, and then ramp to neutral with no flight computer asking for it.
Skipped unless the images are built (tools/bench/sil_triplex.sh --build and --build-safeline) and vcan0 is present.
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


FCS = [_bin(f"TFC_TRIPLEX_BIN_{n.upper()}", f"build/triplex_{n}/zephyr/zephyr.exe") for n in "abc"]
ACT = _bin("TFC_ACT_SAFELINE_BIN", "build/act_safeline/zephyr/zephyr.exe")
ACT_PAD = _bin("TFC_ACT_SAFEPAD_BIN", "build/act_safepad/zephyr/zephyr.exe")


@unittest.skipIf(None in FCS or ACT is None or ACT_PAD is None, "the images are not built (tools/bench/sil_triplex.sh --build and --build-safeline)")
@unittest.skipIf(not Path("/sys/class/net/vcan0").exists(), "vcan0 not present (run sim/scripts/setup_vcan.sh)")
class LiveActSafeLine(unittest.TestCase):
    def run_act(self, act_binary, seconds=12.0):
        """Three flight computers and one ACT image on vcan0; returns ACT's console and the Safe states seen in its output frames."""
        tmp = tempfile.TemporaryDirectory()
        procs, logs = [], []
        bus = B.SocketCanBus("vcan0")
        bus.set_filter([(P.ID_ACT_OUT, 0x7FF)])
        try:
            for i, binary in enumerate([*FCS, act_binary]):
                path = Path(tmp.name) / f"node{i}.log"
                logs.append(path)
                with open(path, "w") as f:
                    procs.append(subprocess.Popen([binary], stdout=f, stderr=subprocess.STDOUT, text=True))
            end = time.monotonic() + seconds  # SYNC frame 500 is five seconds after the first SYNC
            states = []
            while time.monotonic() < end:
                f = bus.recv(0.1)
                if f is not None and (a := P.unpack_act_out(f)) is not None:
                    states.append(a.state)
            return logs[3].read_text(), states
        finally:
            bus.close()
            for p in procs:
                if p.poll() is None:
                    p.kill()
                    p.wait()
            tmp.cleanup()

    def test_the_hardware_line_puts_act_in_safe_by_itself_and_it_ramps_to_neutral(self):
        text, states = self.run_act(ACT)
        self.assertIn("ACT STANDBY -> NOMINAL", text, text[-800:])
        m = re.search(r"\[frame (\d+)\] ACT NOMINAL -> SAFE \(hold\), cause: the SAFE line", text)
        self.assertIsNotNone(m, text[-800:])
        self.assertGreaterEqual(int(m.group(1)), 500)
        self.assertIn("-> SAFE (ramp)", text, "the hold ends in a ramp to neutral without anyone asking")
        self.assertTrue({2, 3, 4} & set(states), f"ACT's output frame never showed a Safe state: {sorted(set(states))}")

    def test_on_the_pad_safe_goes_to_neutral_at_once_with_no_hold_and_no_ramp(self):
        text, states = self.run_act(ACT_PAD)  # built with the launch sequence on: the flight computers send mission frame 0, so the vehicle is on the pad
        m = re.search(r"\[frame (\d+)\] ACT NOMINAL -> SAFE \(neutral\), cause: the SAFE line", text)
        self.assertIsNotNone(m, text[-800:])
        self.assertGreaterEqual(int(m.group(1)), 500)
        self.assertNotIn("(hold)", text.split(m.group(0))[1], "no hold after the entry")
        self.assertNotIn("(ramp)", text, "no ramp: it is already neutral")
        self.assertIn(4, states, "the output frame shows Safe-neutral")
