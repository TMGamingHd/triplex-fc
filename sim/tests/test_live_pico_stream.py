# SPDX-License-Identifier: MIT
"""Live: tfc_simd streams the vehicle's tilts to the Pico's serial port (docs/PICO.md). The port here is a pseudo-terminal that Python reads, so no board is needed; SYNC and
ACT frames are sent on vcan0 by Python in place of the flight computers. Skipped unless build/host/tfc_simd exists and vcan0 does.
"""
import os
import pty
import subprocess
import time
import unittest
from pathlib import Path

from tfc_peers import bus as B
from tfc_peers import pico_link as L
from tfc_peers import protocol as P

ROOT = Path(__file__).resolve().parents[2]
SIMD = os.environ.get("TFC_SIMD_BIN") or str(ROOT / "build/host/tfc_simd")
HAVE_SIMD = os.access(SIMD, os.X_OK)
HAVE_VCAN = Path("/sys/class/net/vcan0").exists()


@unittest.skipIf(not HAVE_SIMD, "tfc_simd is not built (cmake --build build/host --target tfc_simd)")
@unittest.skipIf(not HAVE_VCAN, "vcan0 not present (run sim/scripts/setup_vcan.sh)")
class PicoStream(unittest.TestCase):
    def run_stream(self, pitch_cmd_deg: float, frames: int = 220):
        master, slave = pty.openpty()
        os.set_blocking(master, False)
        proc = subprocess.Popen([SIMD, "--iface", "vcan0", "--pico", os.ttyname(slave), "--quiet"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        bus = B.SocketCanBus("vcan0")
        parser = L.Parser()
        platform = []
        try:
            time.sleep(0.3)
            t0 = time.monotonic()
            for k in range(frames):
                bus.send(0, P.pack_sync(k, k & 0xFF))
                time.sleep(0.003)  # ACT's output comes a few milliseconds into the frame
                bus.send(0, P.pack_act_out(P.ActOut(pitch_deg=pitch_cmd_deg, yaw_deg=0.0, state=1), k & 0xFF))
                try:
                    data = os.read(master, 4096)
                except BlockingIOError:
                    data = b""
                for t, payload in parser.feed(data):
                    if t == L.T_PLATFORM:
                        platform.append((payload[0], int.from_bytes(payload[1:3], "little", signed=True) * 0.01, int.from_bytes(payload[3:5], "little", signed=True) * 0.01))
                time.sleep(max(0.0, t0 + (k + 1) * 0.01 - time.monotonic()))
            time.sleep(0.05)
            try:
                data = os.read(master, 65536)
            except BlockingIOError:
                data = b""
            for t, payload in parser.feed(data):
                if t == L.T_PLATFORM:
                    platform.append((payload[0], int.from_bytes(payload[1:3], "little", signed=True) * 0.01, int.from_bytes(payload[3:5], "little", signed=True) * 0.01))
        finally:
            bus.close()
            proc.terminate()
            proc.wait(timeout=5)
            os.close(master)
            os.close(slave)
        return platform, parser

    def test_one_platform_frame_per_simulated_frame_with_a_rolling_sequence_number(self):
        platform, parser = self.run_stream(0.0)
        self.assertGreater(len(platform), 180)
        self.assertEqual(parser.bad_frames, 0)
        steps = [(b[0] - a[0]) & 0xFF for a, b in zip(platform, platform[1:])]
        self.assertGreaterEqual(steps.count(1), len(steps) - 3, f"the sequence should rise by one each frame: {sorted(set(steps))}")
        self.assertLess(max(max(abs(x), abs(y)) for _, x, y in platform), 1.0)  # a vehicle flying straight up at neutral stays near level

    def test_a_pitch_command_tilts_the_platform_the_way_the_gimbal_convention_says(self):
        platform, _ = self.run_stream(5.0)
        _, x, y = platform[-1]
        self.assertGreater(y, 0.3, "a positive pitch command raises the tilt about Y (docs/VEHICLE_SIM.md)")
        self.assertLess(abs(x), 0.2)


if __name__ == "__main__":
    unittest.main()
