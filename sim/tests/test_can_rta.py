# SPDX-License-Identifier: MIT
"""The response-time analysis of the flight bus (tools/bench/can_rta.py): the frame length, the bound against a case worked by hand, the bound against the schedule's own worst corner, the identifiers
against core/include/tfc/protocol.hpp, and the verdict on the real schedule and on one that is too jittery."""
import importlib.util
import re
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location("can_rta", ROOT / "tools" / "bench" / "can_rta.py")
rta = importlib.util.module_from_spec(spec)
spec.loader.exec_module(rta)


class CanRta(unittest.TestCase):
    def test_a_standard_frame_with_eight_bytes_is_135_bits_at_worst(self):
        self.assertEqual(rta.frame_bits(8), 135)        # Davis, Burns, Bril and Lukkien: 55 + 10 n bits plus the stuff bits
        self.assertEqual(rta.frame_bits(0), 34 + 13 + 33 // 4)

    def test_one_frame_alone_responds_in_its_own_length_and_blocking_adds_the_longest_lower_priority_one(self):
        tau = 1.0
        only = [("a", 0x100, 0, 10000, 1)]
        self.assertEqual(rta.bound(only, tau, 0.0)[0x100], rta.frame_bits() * tau)
        two = [("hi", 0x100, 0, 10000, 1), ("lo", 0x200, 0, 10000, 1)]
        b = rta.bound(two, tau, 0.0)
        c = rta.frame_bits() * tau
        self.assertEqual(b[0x100], 2 * c)               # blocked by the low one already on the wire, then its own time
        self.assertEqual(b[0x200], 2 * c)               # waits for the high one, then its own time

    def test_the_bound_is_never_below_the_schedules_worst_corner(self):
        frames = rta.schedule(True)
        b = rta.bound(frames, 1.0, 100.0)
        c = rta.corners(frames, 1.0, 100.0, True)
        for (_n, ident, _o, _d, _p) in frames:
            self.assertGreaterEqual(b[ident] + 1e-9, c[ident], hex(ident))

    def test_the_identifiers_are_the_protocols(self):
        text = (ROOT / "core" / "include" / "tfc" / "protocol.hpp").read_text()
        const = {m.group(1): int(m.group(2), 16) for m in re.finditer(r"constexpr uint32_t (k\w+) = (0x[0-9A-Fa-f]+);", text)}
        ids = {i for (_n, i, _o, _d, _p) in rta.schedule(True)}
        for name in ("kSync", "kGyroBase", "kAccelBase", "kCmdBase", "kActOut", "kHeartbeat", "kResync", "kSimRates"):
            self.assertIn(name, const)
        self.assertIn(const["kSync"], ids)
        for n in range(3):
            self.assertIn(const["kGyroBase"] + n, ids)
            self.assertIn(const["kAccelBase"] + n, ids)
            self.assertIn(const["kCmdBase"] + n, ids)
            self.assertIn(const["kHeartbeat"] + n, ids)
        self.assertIn(const["kActOut"], ids)
        self.assertEqual({i for i in ids if const["kResync"] <= i < const["kResync"] + 12}, set(range(const["kResync"], const["kResync"] + 12)))

    def test_the_real_schedule_meets_its_deadlines_at_the_design_jitter_and_a_very_jittery_one_does_not(self):
        self.assertEqual(rta.main(["--cycles", "200", "--strict"]), 0)
        self.assertEqual(rta.main(["--cycles", "200", "--strict", "--jitter-us", "3000"]), 3)

    def test_a_slower_bus_loads_more_and_meets_less(self):
        self.assertEqual(rta.main(["--cycles", "100", "--strict", "--bitrate", "250000"]), 3)


if __name__ == "__main__":
    unittest.main()
