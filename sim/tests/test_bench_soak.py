# SPDX-License-Identifier: MIT
"""The two analysis tools of the noise and soak procedure (docs/procedures/P-S1-02-soak-and-noise.md): the Allan deviation of an IMU at rest and the false-alarm bound of a run with nothing wrong."""
import math
import random
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools" / "bench"))
import allan  # noqa: E402
import soak_report as soak  # noqa: E402


def line(t_s, can_id, data):
    return f"({t_s:.6f}) can0 {can_id:03X}#{bytes(data).hex().upper()}"


def crc8_frame(payload7):
    crc = 0xFF
    for b in payload7:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1D) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc ^ 0xFF


def frame(payload6, seq):
    p = bytes(payload6) + bytes([seq & 0xFF])
    return p + bytes([crc8_frame(p)])


class Allan(unittest.TestCase):
    def test_white_noise_has_the_slope_minus_one_half_and_the_level_of_its_density(self):
        r = random.Random(7)
        sigma0 = 0.1
        x = [r.gauss(0.0, sigma0) for _ in range(100000)]
        c = allan.allan_deviation(x, 100.0)
        self.assertAlmostEqual(c[0][1], sigma0, delta=0.01 * sigma0 * 3)
        self.assertAlmostEqual(allan.slope(c, 0, 4), -0.5, delta=0.05)
        density = allan.white_noise_density(c)
        self.assertAlmostEqual(density, sigma0 / math.sqrt(50.0), delta=0.1 * sigma0 / math.sqrt(50.0))   # (white noise of rms sigma0 at 100 Hz: sigma0 / sqrt(50 Hz) per root hertz)

    def test_a_random_walk_has_the_slope_plus_one_half(self):
        r = random.Random(3)
        acc, x = 0.0, []
        for _ in range(100000):
            acc += r.gauss(0.0, 0.01)
            x.append(acc)
        c = allan.allan_deviation(x, 100.0)
        # a random walk in the rate is the integral of white noise: the deviation of a rate that walks rises as sqrt(tau)
        self.assertAlmostEqual(allan.slope(c, len(c) - 6, len(c) - 3), 0.5, delta=0.15)

    def test_a_bias_instability_floor_is_found_where_the_curve_turns_up(self):
        r = random.Random(5)
        x = []
        walk = 0.0
        for _ in range(200000):
            walk += r.gauss(0.0, 0.002)
            x.append(r.gauss(0.0, 0.1) + walk)
        c = allan.allan_deviation(x, 100.0)
        bi = allan.bias_instability(c)
        self.assertIsNotNone(bi)
        self.assertGreater(bi[1], 0.5)                                  # the minimum is at a tau of a second or more
        self.assertIsNone(allan.bias_instability(c[:3]))               # too short a curve says nothing

    def test_a_log_is_read_with_the_missing_frames_held_and_a_repeat_ignored(self):
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / "rest.log"
            rows = []
            seqs = list(range(0, 10)) + [11, 11] + list(range(12, 20))            # frame 10 is missing and frame 11 comes twice
            for k, s_ in enumerate(seqs):
                x = 32 * (k + 1)                                                 # 1 dps in counts of 1/32, rising by one each frame
                payload = x.to_bytes(2, "little", signed=True) + (0).to_bytes(2, "little") + (0).to_bytes(2, "little")
                rows.append(line(0.01 * k, 0x100, frame(payload, s_)))
            rows.append(line(1.0, 0x101, frame(b"\x00" * 6, 0)))                 # another node's frame: not read
            p.write_text("\n".join(rows) + "\n")
            axes, gaps, n = allan.read_log(p, 0, "gyro")
            self.assertEqual(gaps, 1)
            self.assertEqual(n, 20)                                              # the numbers 0 to 19 once each: 19 frames that came and one held for the one that did not
            self.assertAlmostEqual(axes[0][0], 1.0)
            self.assertEqual(axes[0][10], axes[0][9])                            # the held sample repeats the one before it

    def test_the_tool_exits_non_zero_in_strict_mode_when_the_noise_is_above_the_datasheet(self):
        r = random.Random(9)
        with tempfile.TemporaryDirectory() as d:
            quiet, loud = Path(d) / "quiet.csv", Path(d) / "loud.csv"
            quiet.write_text("\n".join(f"{r.gauss(0.0, 0.05):.5f}" for _ in range(20000)))      # density about 7 mdps/rtHz: inside the limit of 8
            loud.write_text("\n".join(f"{r.gauss(0.0, 0.5):.5f}" for _ in range(20000)))        # about 70 mdps/rtHz: far above it
            with self.assertRaises(SystemExit) as cm:
                allan.main(["--csv", str(quiet), "--rate", "100", "--strict"]) or sys.exit(0)
            self.assertEqual(cm.exception.code, 0)
            self.assertEqual(allan.main(["--csv", str(loud), "--rate", "100", "--strict"]), 1)


class Soak(unittest.TestCase):
    def test_the_poisson_limits_are_the_exact_ones(self):
        for k, want in ((0, 2.9957), (1, 4.7439), (2, 6.2958), (3, 7.7537)):
            self.assertAlmostEqual(soak.upper_limit(k, 0.95), want, places=3)
        self.assertAlmostEqual(soak.upper_limit(0, 0.90), -math.log(0.10), places=6)

    def heartbeat(self, t, node, states=(0, 0, 0), safe=False, alarm=False, resets=0):
        b1 = (int(safe) << 2) | (int(alarm) << 3)
        b2 = states[0] | (states[1] << 2) | (states[2] << 4)
        return line(t, 0x400 + node, frame(bytes([2, b1, b2, resets, 0, 0]), 0))

    def log(self, frames=2000, events=()):
        rows = []
        for k in range(frames):
            t = 0.01 * k
            rows.append(line(t, 0x010, frame(k.to_bytes(4, "little") + b"\x00\x00", k)))
            for node in range(3):
                states = [0, 0, 0]
                for (start, end, subject) in events:
                    if start <= k < end:
                        states[subject] = 1
                rows.append(self.heartbeat(t + 0.001, node, tuple(states)))
        return "\n".join(rows) + "\n"

    def test_a_clean_run_counts_nothing_and_a_latch_seen_by_all_three_is_one_isolation(self):
        with tempfile.TemporaryDirectory() as d:
            clean = Path(d) / "clean.log"
            clean.write_text(self.log())
            r = soak.analyse(soak.frames_of(clean))
            self.assertEqual((r["sync_frames"], r["isolations"], r["safe_requests"], r["bus_alarms"], r["resets"]), (2000, 0, 0, 0, 0))
            one = Path(d) / "one.log"
            one.write_text(self.log(events=[(500, 800, 1)]))                      # node B put out for 300 frames, as every computer's heartbeat says
            r = soak.analyse(soak.frames_of(one))
            self.assertEqual(r["isolations"], 1)
            self.assertEqual(r["episodes"][(0, 1)], 1)
            two = Path(d) / "two.log"
            two.write_text(self.log(events=[(500, 800, 1), (1000, 1100, 1)]))
            self.assertEqual(soak.analyse(soak.frames_of(two))["isolations"], 2)

    def test_the_safe_requests_the_alarms_the_resets_and_act_leaving_nominal_are_counted_on_their_rising_edge(self):
        rows = []
        for k in range(1200):
            t = 0.01 * k
            rows.append(line(t, 0x010, frame(k.to_bytes(4, "little") + b"\x00\x00", k)))
            rows.append(self.heartbeat(t, 0, safe=(300 <= k < 400), alarm=(500 <= k < 510), resets=(1 if k >= 700 else 0)))
            word = 1 if k < 900 else 2
            rows.append(line(t, 0x300, frame(bytes([0, 0, 0, 0, word & 0xFF, word >> 8]), k)))
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / "events.log"
            p.write_text("\n".join(rows) + "\n")
            r = soak.analyse(soak.frames_of(p))
        self.assertEqual((r["safe_requests"], r["bus_alarms"], r["resets"], r["act_left_nominal"], r["isolations"]), (1, 1, 1, 1, 0))

    def test_the_report_says_how_many_hours_a_demonstration_needs_and_strict_refuses_an_event(self):
        with tempfile.TemporaryDirectory() as d:
            clean = Path(d) / "clean.log"
            clean.write_text(self.log())
            self.assertEqual(soak.main([str(clean)]), 0)
            self.assertEqual(soak.main([str(clean), "--strict"]), 1)         # twenty seconds is not 300 hours: the bound is above the target
            tiny = Path(d) / "tiny.log"
            tiny.write_text(self.log(frames=10))
            self.assertEqual(soak.main([str(tiny)]), 2)                      # not a run


if __name__ == "__main__":
    unittest.main()
