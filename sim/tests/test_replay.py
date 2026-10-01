# SPDX-License-Identifier: MIT
"""End to end: virtual peers -> candump log -> tfc_replay (the real C++ core/ code).

Expectations come from docs/REQUIREMENTS.md and docs/FAULT_MATRIX.md. Skipped when the
tfc_replay binary is not built (set TFC_REPLAY_BIN, or build into build/host).
"""
import os
import subprocess
import tempfile
import unittest
from pathlib import Path

from tfc_peers import bus as B
from tfc_peers.faults import parse_fault
from tfc_peers.peers import Scenario

ROOT = Path(__file__).resolve().parents[2]
CANDIDATES = [os.environ.get("TFC_REPLAY_BIN", ""), str(ROOT / "build/host/tfc_replay")]
REPLAY = next((c for c in CANDIDATES if c and os.access(c, os.X_OK)), None)
FRAMES = 400


@unittest.skipIf(REPLAY is None, "tfc_replay not built (cmake --build build/host)")
class ReplayThroughCore(unittest.TestCase):
    def replay(self, faults, *expect, nodes=(0, 1, 2), frames=FRAMES, seed=1):
        sc = Scenario(list(nodes), [parse_fault(f) for f in faults], seed)
        with tempfile.TemporaryDirectory() as d:
            log = os.path.join(d, "t.log")
            bus = B.LogBus(log)
            B.record(sc, bus, frames)
            bus.close()
            r = subprocess.run([REPLAY, log, *expect], capture_output=True, text=True, timeout=60)
        out = dict(line.split("=", 1) for line in r.stdout.splitlines() if "=" in line and " " not in line)
        self.assertEqual(r.returncode, 0, f"{faults} {expect}\n{r.stdout}{r.stderr}")
        return out

    def test_healthy_triplex_has_no_false_positives(self):
        out = self.replay([], "--expect-mode", "triplex", "--expect-no-latch", "A",
                          "--expect-no-latch", "B", "--expect-no-latch", "C")
        self.assertEqual({out[k] for k in ("crc_bad", "seq_bad", "missing", "digest_flags", "vote_disagreements",
                                           "stuck_flags")}, {"0"})

    def test_F01_fail_silent_node_isolated_within_3_frames(self):  # TFC-FDIR-001
        for node, letter in enumerate("ABC"):
            with self.subTest(node=letter):
                self.replay([f"{letter}:dropout:start=100"], "--expect-latch", f"{letter}:100-102",
                            "--expect-mode", "duplex")

    def test_F03_stuck_isolated_within_40_frames(self):  # TFC-FDIR-003
        self.replay(["B:stuck:start=100"], "--expect-latch", "B:101-140", "--expect-mode", "duplex")

    def test_F04_bias_step_isolated_in_exactly_3_frames(self):  # TFC-FDIR-002; matches the SIL test
        self.replay(["B:bias:start=100,mag=3"], "--expect-latch", "B:102", "--expect-mode", "duplex")

    def test_F04_bias_on_accel_and_other_axes(self):
        self.replay(["C:bias:start=50,sensor=accel,axis=2,mag=0.2"], "--expect-latch", "C:52")
        self.replay(["A:bias:start=50,axis=1,mag=-4"], "--expect-latch", "A:52")

    def test_F05_slow_drift_isolated_once_past_tolerance(self):
        # 0.05 dps/frame crosses the 1.0 dps tolerance after ~20 frames, then 3 more to latch.
        self.replay(["B:drift:start=100,rate=0.05"], "--expect-latch", "B:115-135", "--expect-mode", "duplex")

    def test_F06_sparse_spikes_do_not_isolate(self):  # TFC-FDIR-004
        out = self.replay(["B:spike:start=0,mag=20,p=0.05"], "--expect-no-latch", "B", "--expect-mode", "triplex")
        self.assertGreater(int(out["vote_disagreements"]), 0)  # the spikes were seen, then filtered

    def test_F07_saturated_sensor_isolated(self):
        self.replay(["B:saturate:start=100"], "--expect-latch", "B:102", "--expect-mode", "duplex")

    def test_F08_one_corrupted_frame_is_dropped_not_isolated(self):
        out = self.replay(["B:corrupt:start=100,end=101,p=1.0"], "--expect-no-latch", "B",
                          "--expect-min", "crc_bad:3", "--expect-mode", "triplex")
        self.assertEqual(out["crc_bad"], "3")
        # the damaged frame still used up a sequence number, so the next good frame is not a "gap"
        self.assertEqual(out["seq_bad"], "0")

    def test_F08_two_separate_corrupt_frames_in_one_window_do_not_isolate(self):  # TFC-FDIR-004
        # Two bad samples inside the 5-frame window is below the 3-of-5 threshold. Before the
        # SeqTracker fix each corrupt frame cost two samples (itself + a false sequence gap) -> latch.
        out = self.replay(["B:corrupt:start=100,end=101,p=1.0", "B:corrupt:start=102,end=103,p=1.0"],
                          "--expect-no-latch", "B", "--expect-mode", "triplex")
        self.assertEqual((out["crc_bad"], out["seq_bad"]), ("6", "0"))

    def test_F08_corruption_does_not_hide_a_real_sequence_gap(self):
        out = self.replay(["B:corrupt:start=100,end=101,p=1.0", "B:seqgap:start=110,gap=4"],
                          "--expect-no-latch", "B")
        self.assertEqual((out["crc_bad"], out["seq_bad"]), ("3", "3"))

    def test_F08_persistent_corruption_isolates_the_node(self):
        self.replay(["B:corrupt:start=100,p=1.0"], "--expect-latch", "B:102", "--expect-mode", "duplex")

    def test_F09_wrong_but_valid_command_isolated(self):
        self.replay(["B:cmd_offset:start=100,mag=1.0"], "--expect-latch", "B:102", "--expect-mode", "duplex")

    def test_F10_digest_divergence_isolated_within_frames(self):  # TFC-FDIR-011 (flag) + persistence
        out = self.replay(["B:digest:start=100"], "--expect-latch", "B:102", "--expect-mode", "duplex")
        self.assertGreaterEqual(int(out["digest_flags"]), 3)

    def test_F11_babble_is_seen_and_ignored(self):  # TFC-FDIR-009 (logic part; bus timing needs HIL)
        self.replay(["B:babble:start=100,n=5"], "--expect-min", f"out_of_schedule:{(FRAMES - 100) * 5}",
                    "--expect-min", f"bus_alarm_frames:{FRAMES - 100}",  # the flood is flagged on every frame it occurs
                    "--expect-no-latch", "A", "--expect-no-latch", "B", "--expect-no-latch", "C",
                    "--expect-mode", "triplex")

    def test_sequence_gap_costs_one_frame_not_the_node(self):
        out = self.replay(["B:seqgap:start=100,gap=4"], "--expect-no-latch", "B", "--expect-mode", "triplex")
        self.assertEqual(out["seq_bad"], "3")  # one jump seen on each of B's three streams

    def test_dropout_then_recovery_is_still_latched_until_reintegrated(self):
        self.replay(["B:dropout:start=100,end=150"], "--expect-latch", "B:100-102", "--expect-mode", "duplex")

    def test_F16_second_step_fault_in_duplex_is_attributed_by_continuity(self):  # TFC-FDIR-017
        # B is out (duplex A,C). C then steps by 3 dps: C jumped away from the last agreed value and A did
        # not, so C is blamed and the system continues in simplex on A. Before the arbitration it fell to safe.
        self.replay(["B:bias:start=100", "C:bias:start=200,axis=1"], "--expect-latch", "B:102",
                    "--expect-latch", "C:202", "--expect-no-latch", "A", "--expect-mode", "simplex")

    def test_F16_second_fault_too_small_to_attribute_requests_safe_and_stays(self):  # TFC-FDIR-008
        # A 1.5 dps step is above the 1.0 dps tolerance but not clearly an outlier: unresolved. Hold the
        # output and request Safe (sticky). Nobody is blamed afterwards either: the held reference is stale
        # (an earlier version blamed the healthy node A on it).
        out = self.replay(["B:bias:start=100", "C:bias:start=200,axis=1,mag=1.5"], "--expect-latch", "B:102",
                          "--expect-no-latch", "A", "--expect-no-latch", "C", "--expect-mode", "safe",
                          "--expect-min", "safe_request_frames:100", "--expect-min", "held_frames:100")
        self.assertGreater(int(out["unresolved_frames"]), 0)

    def test_duplex_slow_drift_requests_safe_and_blames_nobody(self):
        # Both survivors stay within reach of the last agreed value while the gap opens: not attributable.
        self.replay(["B:bias:start=100", "C:drift:start=200,rate=0.05"], "--expect-latch", "B:102",
                    "--expect-no-latch", "A", "--expect-no-latch", "C", "--expect-mode", "safe")

    def test_duplex_three_spikes_blame_only_the_culprit_and_do_not_reach_safe(self):  # TFC-FDIR-017
        spikes = [f"C:spike:start={s},end={s + 1},p=1.0" for s in (100, 102, 104)]
        self.replay(["B:dropout:start=50", *spikes], "--expect-latch", "B:52", "--expect-latch", "C:104",
                    "--expect-no-latch", "A", "--expect-mode", "simplex")

    def test_duplex_digest_mismatch_cannot_be_attributed_so_requests_safe(self):
        self.replay(["B:dropout:start=50", "C:digest:start=100"], "--expect-no-latch", "A",
                    "--expect-no-latch", "C", "--expect-mode", "safe", "--expect-min", "safe_request_frames:100")

    def test_one_lost_frame_costs_one_sample(self):  # TFC-FDIR-016
        out = self.replay(["B:dropout:start=100,end=101"], "--expect-no-latch", "B", "--expect-mode", "triplex")
        self.assertEqual((out["missing"], out["seq_bad"]), ("1", "0"))

    def test_two_isolated_lost_frames_do_not_isolate_a_healthy_node(self):  # TFC-FDIR-016
        self.replay(["B:dropout:start=100,end=101", "B:dropout:start=103,end=104"], "--expect-no-latch", "B",
                    "--expect-mode", "triplex")

    def test_a_node_returning_after_a_silence_with_its_counter_running_is_not_penalised(self):
        out = self.replay(["B:dropout:start=100,end=150"], "--expect-latch", "B:100-102")
        self.assertEqual(out["seq_bad"], "0")

    def test_absent_node_is_just_an_invalid_node(self):  # STAGED_BUILD rule 2: absent means invalid
        self.replay([], "--expect-latch", "A:2", "--expect-mode", "duplex", nodes=(1, 2))
        self.replay([], "--expect-mode", "simplex", nodes=(2,))

    def test_different_seeds_give_the_same_fdir_outcome(self):
        for seed in (1, 2, 3, 4):
            with self.subTest(seed=seed):
                self.replay(["B:bias:start=100"], "--expect-latch", "B:102", "--expect-no-latch", "A",
                            "--expect-no-latch", "C", seed=seed)
                self.replay([], "--expect-mode", "triplex", seed=seed)


class ReplayToolContract(unittest.TestCase):
    @unittest.skipIf(REPLAY is None, "tfc_replay not built")
    def test_failed_expectation_exits_nonzero_and_bad_input_exits_2(self):
        with tempfile.TemporaryDirectory() as d:
            log = os.path.join(d, "t.log")
            bus = B.LogBus(log)
            B.record(Scenario([0, 1, 2]), bus, 20)
            bus.close()
            r = subprocess.run([REPLAY, log, "--expect-latch", "B:5"], capture_output=True, text=True)
            self.assertEqual(r.returncode, 1)
            self.assertIn("EXPECT FAILED", r.stdout)
            bad = os.path.join(d, "bad.log")
            Path(bad).write_text("garbage\n")
            self.assertEqual(subprocess.run([REPLAY, bad], capture_output=True).returncode, 2)
            self.assertEqual(subprocess.run([REPLAY], capture_output=True).returncode, 2)


if __name__ == "__main__":
    unittest.main()
