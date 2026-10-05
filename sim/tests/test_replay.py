# SPDX-License-Identifier: MIT
"""End to end: virtual peers -> candump log -> tfc_replay (the real C++ core/ code).

Expectations come from docs/verification/REQUIREMENTS.md and docs/verification/FAULT_MATRIX.md. Skipped when the
tfc_replay binary is not built (set TFC_REPLAY_BIN, or build into build/host).
"""
import os
import subprocess
import tempfile
import unittest
from pathlib import Path

from tfc_peers import bus as B
from tfc_peers.commands import parse_commands
from tfc_peers.faults import parse_fault
from tfc_peers.peers import Scenario

ROOT = Path(__file__).resolve().parents[2]
CANDIDATES = [os.environ.get("TFC_REPLAY_BIN", ""), str(ROOT / "build/host/tfc_replay")]
REPLAY = next((c for c in CANDIDATES if c and os.access(c, os.X_OK)), None)
FRAMES = 400


@unittest.skipIf(REPLAY is None, "tfc_replay not built (cmake --build build/host)")
class ReplayBase(unittest.TestCase):
    def replay(self, faults, *expect, nodes=(0, 1, 2), frames=FRAMES, seed=1, commands=()):
        sc = Scenario(list(nodes), [parse_fault(f) for f in faults], seed, [x for c in commands for x in parse_commands(c)])
        with tempfile.TemporaryDirectory() as d:
            log = os.path.join(d, "t.log")
            bus = B.LogBus(log)
            B.record(sc, bus, frames)
            bus.close()
            r = subprocess.run([REPLAY, log, *expect], capture_output=True, text=True, timeout=60)
        out = dict(line.split("=", 1) for line in r.stdout.splitlines() if "=" in line and " " not in line)
        self.assertEqual(r.returncode, 0, f"{faults} {expect}\n{r.stdout}{r.stderr}")
        return out


@unittest.skipIf(REPLAY is None, "tfc_replay not built (cmake --build build/host)")
class ReplayPhases(ReplayBase):
    """Mission phases and the WARM role through the real core (ADR-023, `--phases`): scripted `phase`, `warm`, `noop` and `reintegrate` commands."""

    def test_the_operator_rests_a_computer_a_phase_that_needs_three_is_refused_and_the_promotion_brings_it_back(self):
        out = self.replay([], "--phases", "--expect-no-latch", "A", "--expect-no-latch", "B", frames=700,
                          commands=["50:noop", "100:phase:coast", "150:warm:C", "300:phase:pre-launch", "350:reintegrate:C", "450:phase:pre-launch"])
        self.assertEqual(out["phase"], "2")
        self.assertEqual(out["warm"], "0x0")
        self.assertEqual(out["phase_changes"], "2")
        self.assertEqual(out["commands_refused"], "1")
        self.assertEqual(out["healthy"], "3")
        self.assertEqual(out["reintegrations"], "1")

    def test_a_computer_still_resting_at_the_end_is_reported_and_the_system_is_duplex_by_mode(self):
        out = self.replay([], "--phases", "--expect-mode", "duplex", frames=400, commands=["100:phase:coast", "150:warm:C"])
        self.assertEqual((out["phase"], out["warm"], out["healthy"]), ("4", "0x4", "2"))

    def test_without_phases_the_commands_are_refused_and_nothing_else_changes(self):
        out = self.replay([], "--expect-mode", "triplex", frames=300, commands=["100:phase:coast"])
        self.assertEqual(out["commands_refused"], "1")
        self.assertNotIn("phase", out)


@unittest.skipIf(REPLAY is None, "tfc_replay not built (cmake --build build/host)")
class ReplayThroughCore(ReplayBase):
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
        out = self.replay(["B:corrupt:start=100,end=101,p=1.0", "B:seqgap:start=110,end=111,gap=4"],
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

    def test_a_one_frame_glitch_of_the_frame_number_costs_one_frame_not_the_node(self):
        out = self.replay(["B:seqgap:start=100,end=101,gap=4"], "--expect-no-latch", "B", "--expect-mode", "triplex")
        self.assertEqual(out["seq_bad"], "3")  # the wrong number seen once on each of B's three streams

    def test_a_node_whose_frame_number_stays_wrong_is_isolated(self):  # ADR-018: frames carry SYNC's number, so a wrong one is a wrong frame
        out = self.replay(["B:seqgap:start=100,gap=4"], "--expect-latch", "B:102", "--expect-mode", "duplex")
        self.assertGreater(int(out["seq_bad"]), 100)

    def test_a_reboot_that_resyncs_comes_back_in_phase_and_one_that_does_not_is_isolated_for_good(self):
        out = self.replay(["B:reboot:start=100,down=30"], "--expect-latch", "B:100-103", "--expect-mode", "duplex")
        self.assertEqual(out["seq_bad"], "0")  # no sequence break at all: the frame number came from SYNC
        out = self.replay(["B:reboot:start=100,down=30,resync=0"], "--expect-latch", "B:100-103", frames=500)
        self.assertGreater(int(out["seq_bad"]), 100)  # still restarting from 0: every frame is the wrong frame

    def test_a_two_frame_silence_is_not_enough_to_isolate(self):  # E4: 3-of-5 needs three bad frames
        self.replay(["B:reboot:start=100,down=2"], "--expect-no-latch", "B", "--expect-mode", "triplex")

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


@unittest.skipIf(REPLAY is None, "tfc_replay not built (cmake --build build/host)")
class NodeLifeCycle(ReplayBase):
    """Latch -> dwell -> probation (shadow vote) -> readmission, strikes and disabling (ADR-010)."""

    def test_F21_a_transient_fault_is_readmitted_after_the_operator_asks(self):  # TFC-FDIR-006
        out = self.replay(["B:bias:start=100,end=130"], "--expect-latch", "B:102", "--expect-state", "B:healthy",
                          "--expect-mode", "triplex", "--expect-min", "reintegrations:1", frames=700,
                          commands=["450:reintegrate:B"])
        self.assertEqual(out["strikes.B"], "1")  # the strike stays on record

    def test_manual_policy_leaves_a_clean_node_out_until_asked(self):
        self.replay(["B:bias:start=100,end=130"], "--expect-state", "B:latched", "--expect-mode", "duplex", frames=700)

    def test_auto_policy_readmits_a_first_transient_latch_without_a_command(self):
        self.replay(["B:dropout:start=100,end=120"], "--policy", "auto", "--expect-state", "B:healthy",
                    "--expect-mode", "triplex", frames=700)

    def test_auto_policy_does_not_readmit_a_digest_mismatch(self):
        self.replay(["B:digest:start=100,end=120"], "--policy", "auto", "--expect-state", "B:latched",
                    "--expect-mode", "duplex", frames=700)

    def test_F22_a_node_that_is_still_faulty_is_refused_by_the_shadow_vote(self):  # TFC-FDIR-020
        self.replay(["B:bias:start=100"], "--expect-state", "B:latched", "--expect-mode", "duplex",
                    "--expect-min", "probation_failures:2", frames=1000, commands=["450:reintegrate:B", "800:reintegrate:B"])

    def test_F23_a_repeat_offender_is_disabled_on_its_third_latch(self):  # TFC-FDIR-021
        out = self.replay(["B:bias:start=100,end=130", "B:bias:start=600,end=630", "B:bias:start=1200,end=1230"],
                          "--expect-state", "B:disabled", "--expect-mode", "duplex", "--expect-min", "nodes_disabled:1",
                          "--expect-min", "commands_refused:1", frames=1500,
                          commands=["450:reintegrate:B", "700:reintegrate:B", "1300:reintegrate:B"])
        self.assertEqual(out["strikes.B"], "3")

    def test_a_stuck_sensor_is_disabled_at_its_second_strike_the_cause_is_physical(self):  # TFC-FDIR-021
        # Vehicle at rest is the case only the stuck detector can see; the cause is failed hardware.
        from tfc_peers import peers
        orig = peers.truth
        peers.truth = lambda t: ((0.0, 0.0, 0.0), (0.0, 0.0, 1.0))
        try:
            out = self.replay(["B:stuck:start=100,end=140", "B:stuck:start=500,end=540"], "--expect-state", "B:disabled",
                              frames=900, commands=["300:reintegrate:B"])
        finally:
            peers.truth = orig
        self.assertEqual(out["strikes.B"], "2")

    def test_F24_a_rebooted_node_comes_back_with_a_restarted_counter(self):
        self.replay(["B:reboot:start=100,down=50"], "--expect-latch", "B:100-102", "--expect-state", "B:healthy",
                    "--expect-mode", "triplex", frames=700, commands=["450:reintegrate:B"])

    def test_F25_late_frames_are_stale_data_and_isolate_the_node(self):
        # The command frame leaves at ~9.3 ms, after FC-A's 7 ms vote: it is judged a frame late (stale).
        self.replay(["B:late:start=100,us=4000"], "--expect-latch", "B:100-106", "--expect-state", "B:latched",
                    "--expect-mode", "duplex")

    def test_F26_an_intermittent_fault_that_three_of_five_cannot_see_is_isolated(self):  # TFC-FDIR-023
        # One bad frame in three, for ever: 3-of-5 never fills (the vote masks each bad frame, but the node is
        # plainly sick). The leaky alpha-count (ADR-013) isolates it about a dozen frames after it starts.
        out = self.replay(["B:corrupt:start=100,p=1.0,period=3,duty=1"], "--expect-latch", "B:105-130",
                          "--expect-state", "B:latched", frames=500)
        self.assertGreater(int(out["crc_bad"]), 50)

    def test_F26_two_bad_in_five_and_two_in_ten_are_isolated_too(self):  # TFC-FDIR-023
        self.replay(["B:corrupt:start=100,p=1.0,period=5,duty=2"], "--expect-latch", "B:105-125")
        self.replay(["B:corrupt:start=100,p=1.0,period=10,duty=2"], "--expect-latch", "B:105-145", frames=600)

    def test_F26_sparse_trouble_is_left_alone(self):  # TFC-FDIR-004 / 023: no false isolations
        for period in (10, 20):  # one bad frame in 10 (10%) and in 20 (5%)
            with self.subTest(period=period):
                self.replay([f"B:corrupt:start=100,p=1.0,period={period},duty=1"], "--expect-no-latch", "B",
                            "--expect-mode", "triplex", frames=900)
        self.replay(["B:spike:start=0,mag=20,p=0.05"], "--expect-no-latch", "B", "--expect-mode", "triplex", frames=900)

    def test_F26_a_recurring_intermittent_node_is_disabled_at_its_second_strike(self):  # TFC-FDIR-007
        # The cause is "intermittent" = physical class: readmitted once, latching again disables it.
        out = self.replay(["B:corrupt:start=100,end=200,p=1.0,period=3,duty=1",
                           "B:corrupt:start=600,p=1.0,period=3,duty=1"],
                          "--expect-state", "B:disabled", "--expect-min", "nodes_disabled:1", frames=1000,
                          commands=["450:reintegrate:B"])
        self.assertEqual(out["strikes.B"], "2")

    def test_operator_can_disable_and_clear_a_node(self):
        self.replay([], "--expect-state", "B:healthy", "--expect-mode", "triplex", frames=800,
                    commands=["200:disable:B", "300:armed-clear-disabled:B", "310:reintegrate:B"])
        self.replay([], "--expect-state", "B:disabled", "--expect-mode", "duplex", frames=400,
                    commands=["200:disable:B"])

    def test_refusals_are_counted_not_applied(self):
        self.replay([], "--expect-state", "B:healthy", "--expect-mode", "triplex", "--expect-min", "commands_refused:2",
                    frames=400, commands=["200:reintegrate:B", "210:clear-disabled:B"])


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
