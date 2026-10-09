# SPDX-License-Identifier: MIT
"""The console draws the voter's tolerance band, the ARM window and the phase table from copies of core/ defaults. A copy that drifts from the header would draw a wrong band and say nothing: this reads the headers."""
import re
import unittest

from .console_support import ROOT, P  # noqa: F401  (console_support puts console/ on the path)
from tfc_console import constants as K

TYPES = (ROOT / "core" / "include" / "tfc" / "redundancy_types.hpp").read_text()
ACT = (ROOT / "core" / "include" / "tfc" / "act.hpp").read_text()
MONITOR = (ROOT / "core" / "include" / "tfc" / "fault_monitor.hpp").read_text()
VOTER = (ROOT / "core" / "include" / "tfc" / "voter.hpp").read_text()


def numbers(text):
    return [float(x) for x in re.findall(r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?(?=F)", text)]


class CopiesOfCoreDefaults(unittest.TestCase):
    def test_the_vote_tolerances_are_the_redundancy_configs(self):
        m = re.search(r"std::array<float, kVoteChannels> tol\{\{([^}]*)\}\}", TYPES)
        self.assertIsNotNone(m)
        self.assertEqual(tuple(numbers(m.group(1))), K.VOTE_TOL)

    def test_the_persistence_window_is_m_of_n(self):
        self.assertEqual((int(re.search(r"persist_m = (\d+)", TYPES).group(1)), int(re.search(r"persist_n = (\d+)", TYPES).group(1))), (K.PERSIST_M, K.PERSIST_N))

    def test_the_arm_window_and_the_command_window(self):
        self.assertEqual(int(re.search(r"arm_window_frames = (\d+)", TYPES).group(1)), K.ARM_WINDOW_FRAMES)
        self.assertEqual(int(re.search(r"command_window = (\d+)", TYPES).group(1)), K.COMMAND_WINDOW)

    def test_the_strike_limits_and_the_bus_alarm(self):
        self.assertEqual((int(re.search(r"max_strikes = (\d+)", TYPES).group(1)), int(re.search(r"max_strikes_physical = (\d+)", TYPES).group(1))), (K.STRIKES_MAX, K.STRIKES_MAX_PHYSICAL))
        self.assertEqual(int(re.search(r"bus_alarm_per_frame = (\d+)", TYPES).group(1)), K.BUS_ALARM_PER_FRAME)

    def test_acts_tolerance(self):
        self.assertEqual(float(re.search(r"tol_deg = ([\d.]+)F", ACT).group(1)), K.ACT_TOL_DEG)

    def test_the_phase_table_is_the_core_table(self):
        rules = re.search(r"kRules\{\{(.*?)\}\};", TYPES, re.S).group(1)
        pairs = [(int(a), int(b)) for a, b in re.findall(r"\{(\d+)U, (\d+)U\}", rules)]
        self.assertEqual(tuple(pairs), K.PHASE_RULES)
        self.assertEqual(K.PHASE_NAMES, P.PHASE_NAMES)

    def test_the_mode_numbers(self):
        m = re.search(r"enum class Mode : uint8_t \{([^}]*)\}", MONITOR).group(1)
        core = {int(v): n.lower() for n, v in re.findall(r"(\w+) = (\d+)", m)}
        self.assertEqual({k: v.lower() for k, v in K.MODE_NAMES.items()}, core)

    def test_the_vote_status_names_are_in_the_cores_order(self):
        m = re.search(r"enum class VoteStatus : uint8_t \{(.*?)\};", VOTER, re.S).group(1)
        names = re.findall(r"^\s*(\w+),?\s*(?://.*)?$", m, re.M)
        self.assertEqual(list(P.VOTE_STATUS_NAMES), names)

    def test_the_view_names_are_the_heartbeats(self):
        """Heartbeat byte 2: 0 healthy, 1 latched, 2 probation, 3 disabled (docs/design/PROTOCOL.md)."""
        self.assertEqual(K.VIEW_NAMES, ("healthy", "latched", "probation", "disabled"))
        text = (ROOT / "docs" / "design" / "PROTOCOL.md").read_text()
        self.assertIn("0 healthy, 1 latched, 2 probation, 3 disabled", text)


class TheBusTable(unittest.TestCase):
    def test_every_scheduled_id_is_named_and_in_schedule(self):
        for can_id, name, _what in K.SCHEDULE:
            self.assertTrue(K.in_schedule(can_id), name)
        for node in range(3):
            for base in (0x100, 0x110, 0x200, 0x400, 0x410):
                self.assertTrue(K.in_schedule(base + node))
        for k in range(12):
            self.assertTrue(K.in_schedule(0x420 + k))
        for i in range(0x500, 0x510):
            self.assertTrue(K.in_schedule(i), hex(i))

    def test_other_ids_are_out_of_schedule(self):
        for can_id in (0x020, 0x2F0, 0x520, 0x7FF, 0x103):
            self.assertFalse(K.in_schedule(can_id), hex(can_id))

    def test_names_of_the_ids(self):
        self.assertEqual(K.id_name(0x101), "GYRO B")
        self.assertEqual(K.id_name(0x426), "RESYNC B.2")
        self.assertEqual(K.id_name(0x300), "ACT")
        self.assertTrue(K.id_name(0x020).startswith("?"))

    def test_the_expected_rates_are_the_measured_ones(self):
        """Measured on the live rig (25 s of vcan0 with five processes, 8 Oct 2026): the 100 Hz ids at 96 to 100 a second, the state share at 10, the resync chunks at about 1, the simulator's status at 10."""
        self.assertEqual([K.expected_hz(i) for i in (0x010, 0x100, 0x112, 0x200, 0x300, 0x402, 0x501)], [100.0] * 7)
        self.assertEqual([K.expected_hz(i) for i in (0x410, 0x503, 0x505)], [10.0] * 3)
        self.assertEqual(K.expected_hz(0x42B), 1.0)
        self.assertIsNone(K.expected_hz(0x510))
        self.assertIsNone(K.expected_hz(0x021))

    def test_the_schedule_matches_the_protocol_ids(self):
        ids = {n: i for i, n, _ in K.SCHEDULE}
        self.assertEqual(ids["SYNC"], P.ID_SYNC)
        self.assertEqual(ids["ACT"], P.ID_ACT_OUT)
        self.assertEqual(ids["GROUND"], P.ID_GROUND)
        self.assertEqual(ids["SIM_STATE"], P.ID_SIM_STATE)


if __name__ == "__main__":
    unittest.main()
