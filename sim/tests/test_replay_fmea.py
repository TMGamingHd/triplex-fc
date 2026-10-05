# SPDX-License-Identifier: MIT
"""The fault kinds added by the FMEA gap analysis (FAULT_MATRIX F27-F45) and the campaign findings (F46-F51), through the
real core: peers -> candump log -> tfc_replay. Latch frames are the measured ones (docs/verification/FAULT_CAMPAIGN.md) with a small margin."""
import unittest

from tests.test_replay import REPLAY, ReplayBase


@unittest.skipIf(REPLAY is None, "tfc_replay not built (cmake --build build/host)")
class NewFaultKindsThroughTheCore(ReplayBase):
    def isolated(self, spec, lo, hi=None, mode="duplex", **kw):
        return self.replay([spec], "--expect-latch", f"B:{lo}-{hi or lo + 3}", "--expect-state", "B:latched", "--expect-mode", mode, **kw)

    def test_F27_a_scale_factor_error_is_a_vote_miscompare(self):
        self.isolated("B:scale:start=100,axis=0,factor=1.5", 102)
        self.isolated("B:scale:start=100,sensor=accel,axis=2,factor=1.05", 102)  # gravity is constant: 0.05 g is 2.5 tolerances

    def test_F27_a_small_scale_error_is_left_alone(self):
        self.replay(["B:scale:start=100,sensor=accel,axis=2,factor=1.005"], "--expect-no-latch", "B", "--expect-mode", "triplex")

    def test_F28_noise_ten_times_the_tolerance_is_isolated_and_ordinary_noise_is_not(self):
        self.isolated("B:noise:start=100,mult=30", 102)
        self.replay(["B:noise:start=100,mult=2"], "--expect-no-latch", "B", "--expect-mode", "triplex")

    def test_F29_F30_F31_inverted_swapped_and_zeroed_sensors_are_isolated(self):
        self.isolated("B:invert:start=100,axis=0", 102)
        self.isolated("B:swap:start=100,axis=0,other=1", 102)
        self.isolated("B:zero:start=100", 102)

    def test_F32_a_wrong_full_scale_range_is_isolated_only_if_it_clips_real_signal(self):
        self.isolated("B:clip:start=100,limit=4", 102)
        self.replay(["B:clip:start=100,limit=12"], "--expect-no-latch", "B", "--expect-mode", "triplex")  # the 10 dps signal never reaches it

    def test_F33_vibration_is_isolated_and_a_sample_rate_alias_is_a_constant_offset(self):
        self.isolated("B:oscillate:start=100,amp=3,hz=33", 104, 108)
        self.isolated("B:oscillate:start=100,amp=6,hz=100", 102)  # 100 Hz sampled at 100 Hz is DC: 6 sin(0.31) = 1.85 dps
        self.replay(["B:oscillate:start=100,amp=0.3,hz=33"], "--expect-no-latch", "B", "--expect-mode", "triplex")

    def test_F34_a_sample_held_for_many_frames_falls_behind_the_motion(self):
        self.isolated("B:repeat:start=100,n=10", 107, 110)
        self.replay(["B:repeat:start=100,n=2"], "--expect-no-latch", "B", "--expect-mode", "triplex")

    def test_F35_a_bit_upset_before_the_crc_is_a_valid_frame_with_wrong_data(self):
        out = self.replay(["B:bitflip:start=100,bit=12,p=1.0"], "--expect-latch", "B:102-105", "--expect-state", "B:latched")
        self.assertEqual(out["crc_bad"], "0")  # the checksum cannot see it: only the vote can
        self.replay(["B:bitflip:start=100,bit=1,p=1.0"], "--expect-no-latch", "B", "--expect-mode", "triplex")  # 2 LSB = 0.25 dps

    def test_F36_a_stuck_data_line_is_isolated(self):
        self.isolated("B:stuckbit:start=100,bit=9,value=1", 102)

    def test_F37_F38_a_frozen_or_inverted_command_is_a_command_vote_miscompare(self):
        self.isolated("B:cmdstuck:start=100", 102)
        self.isolated("B:cmdinvert:start=100", 102)

    def test_F39_missing_frame_types_are_missing_frames(self):
        for mask in (1, 2, 4, 7):
            with self.subTest(mask=mask):
                self.isolated(f"B:partial:start=100,mask={mask}", 102)

    def test_F40_duplicated_frames_are_sequence_errors(self):
        out = self.isolated("B:duplicate:start=100", 102)
        self.assertGreater(int(out["seq_bad"]), 100)

    def test_F41_replayed_frames_carry_old_data_and_old_counters(self):
        self.isolated("B:replay:start=100,age=3", 102)

    def test_F42_a_frozen_sequence_counter_is_isolated(self):
        out = self.isolated("B:seqstuck:start=100", 103)
        self.assertGreater(int(out["seq_bad"]), 100)

    def test_F43_a_stream_that_is_a_whole_frame_early_is_isolated_because_frames_carry_SYNCs_number(self):
        # campaign edge case E11, fixed by ADR-018: 6 ms early puts the gyro and accel frames into the previous frame's window.
        # The counter used to stay contiguous and the data were one frame ahead (under 0.5 dps): invisible. Now each frame says
        # which SYNC frame it belongs to, so a frame numbered for the NEXT cycle is plainly out of phase.
        out = self.replay(["B:early:start=100,us=6000"], "--expect-latch", "B:100-103", "--expect-mode", "duplex")
        self.assertGreater(int(out["seq_bad"]), 100)

    def test_F43_early_inside_the_acceptance_window_is_still_invisible_until_arrival_times_are_checked(self):
        # Up to about 4.5 ms early the frame still arrives in the right window, with the right number: only a check of the arrival
        # time against the slot (docs/design/FUTURE_WORK.md, E11 option B) can see it.
        self.replay(["B:early:start=100,us=4500"], "--expect-no-latch", "B", "--expect-mode", "triplex")
        self.replay(["B:early:start=100,us=1000"], "--expect-no-latch", "B", "--expect-mode", "triplex")

    def test_F44_timing_jitter_beyond_the_vote_deadline_loses_frames(self):
        self.isolated("B:jitter:start=100,us=3000", 105, 112)
        self.replay(["B:jitter:start=100,us=500"], "--expect-no-latch", "B", "--expect-mode", "triplex")

    def test_F45_a_drifting_clock_is_isolated_when_the_command_crosses_the_deadline(self):
        self.isolated("B:clockdrift:start=100,us_per_frame=40", 140, 150)
        self.replay(["B:clockdrift:start=100,us_per_frame=1"], "--expect-no-latch", "B", "--expect-mode", "triplex", frames=500)

    def test_every_new_kind_can_be_replayed_as_an_intermittent_fault_without_a_false_alarm_on_the_others(self):
        for spec in ("B:zero:start=100,period=10,duty=1", "B:partial:start=100,mask=7,period=10,duty=1",
                     "B:invert:start=100,sensor=accel,axis=2,period=10,duty=1", "B:cmdinvert:start=100,period=10,duty=1"):
            with self.subTest(spec=spec):
                self.replay([spec], "--expect-no-latch", "A", "--expect-no-latch", "C", frames=600)


@unittest.skipIf(REPLAY is None, "tfc_replay not built (cmake --build build/host)")
class CampaignFindingsThroughTheCore(ReplayBase):
    def test_F46_total_loss_is_recoverable_when_two_candidates_are_asked(self):
        faults = [f"{c}:dropout:start=100,end=104" for c in "ABC"]
        out = self.replay(faults, "--expect-latch", "A:102", "--expect-latch", "B:102", "--expect-latch", "C:102", "--expect-state", "A:healthy",
                          "--expect-state", "B:healthy", "--expect-state", "C:healthy", "--expect-mode", "triplex", "--expect-min", "reintegrations:3",
                          frames=800, commands=["400:reintegrate:A", "400:reintegrate:B", "400:reintegrate:C"])
        self.assertEqual(out["probation_failures"], "0")

    def test_F46_two_of_three_are_enough_and_the_one_nobody_asked_for_stays_out(self):
        faults = [f"{c}:dropout:start=100,end=104" for c in "ABC"]
        self.replay(faults, "--expect-state", "A:healthy", "--expect-state", "C:healthy", "--expect-state", "B:latched", "--expect-mode", "duplex",
                    frames=800, commands=["400:reintegrate:A", "400:reintegrate:C"])

    def test_F46_a_lone_candidate_has_nobody_to_be_judged_against(self):
        faults = [f"{c}:dropout:start=100,end=104" for c in "ABC"]
        self.replay(faults, "--expect-state", "A:probation", "--expect-mode", "safe", frames=800, commands=["400:reintegrate:A"])

    def test_F46_the_cohort_throws_out_a_member_that_is_still_wrong(self):
        faults = [f"{c}:dropout:start=100,end=104" for c in "ABC"] + ["B:bias:start=104,mag=3"]
        self.replay(faults, "--expect-state", "A:healthy", "--expect-state", "C:healthy", "--expect-state", "B:latched", "--expect-mode", "duplex",
                    frames=800, commands=["400:reintegrate:A", "400:reintegrate:B", "400:reintegrate:C"])

    def test_F50_a_frozen_command_in_duplex_is_blamed_on_the_frozen_node_where_the_command_moves_fast(self):
        # The old arbitration compared both nodes with the last agreed value, so when the command moved more than two
        # tolerances per frame (the steep parts of the motion, around frames 62, 125 and 187) the HEALTHY node was the one
        # that "jumped" and was isolated. Judged against where the motion should be, the node that stopped following it is.
        for start in (55, 62, 70, 118, 125, 132, 180, 187, 194):
            with self.subTest(start=start):
                self.replay([f"B:cmdstuck:start={start}"], "--expect-no-latch", "A", "--expect-latch", f"B:{start}-{start + 8}", nodes=(0, 1),
                            frames=start + 60)

    def test_F50_a_frozen_command_never_blames_the_healthy_node_at_any_phase(self):
        # Where the command barely moves (the peaks, near frames 31, 94, 156) neither node can be singled out: the system
        # holds the last good value and requests Safe, but it never isolates the healthy node.
        for start in range(40, 200, 11):
            with self.subTest(start=start):
                self.replay([f"B:cmdstuck:start={start}"], "--expect-no-latch", "A", nodes=(0, 1), frames=start + 60)

    def test_F50_stale_or_wrong_commands_in_duplex_never_blame_the_healthy_node(self):
        for spec in ("B:late:start=62,us=4000", "B:replay:start=62,age=1", "B:cmd_offset:start=62,mag=1.0", "B:cmdinvert:start=62"):
            with self.subTest(spec=spec):
                self.replay([spec], "--expect-no-latch", "A", nodes=(0, 1), frames=140)

    def test_F49_a_flood_on_an_id_above_the_simulator_id_raises_the_bus_alarm(self):
        for bid in ("0x511", "0x520", "0x7F0", "0x4F0", "0x301"):
            with self.subTest(id=bid):
                out = self.replay([f"B:babble:start=100,n=5,id={bid}"], "--expect-no-latch", "B", "--expect-mode", "triplex")
                self.assertGreater(int(out["bus_alarm_frames"]), 100)
                self.assertGreater(int(out["out_of_schedule"]), 1000)

    def test_F51_three_different_digests_isolate_nobody_and_request_safe(self):
        out = self.replay(["B:digest:start=100,xor=1", "C:digest:start=100,xor=2"], "--expect-no-latch", "A", "--expect-no-latch", "B",
                          "--expect-no-latch", "C", "--expect-mode", "safe", "--expect-min", "digest_flags:1")
        self.assertEqual(out["nodes_disabled"], "0")
        self.assertGreater(int(out["unresolved_frames"]), 0)


@unittest.skipIf(REPLAY is None, "tfc_replay not built (cmake --build build/host)")
class GroundCommandsThroughTheCore(ReplayBase):
    """E10 (ADR-019): authenticated, replay-protected, two-step operator commands and the interlock tiers."""

    def test_F52_a_command_with_a_wrong_tag_has_no_effect_and_is_counted(self):
        out = self.replay([], "--expect-state", "B:healthy", "--expect-mode", "triplex", frames=200, commands=["100:forged-disable:B"])
        self.assertEqual((out["commands_unauthentic"], out["commands_accepted"]), ("1", "0"))

    def test_F52_a_replayed_command_is_refused(self):
        out = self.replay([], "--expect-state", "B:disabled", "--expect-mode", "duplex", frames=200, commands=["100:disable:B", "110:replay"])
        self.assertEqual((out["commands_replayed"], out["commands_accepted"]), ("1", "1"))  # applied once, the repeat dropped

    def test_F52_clear_safe_and_clear_disabled_need_an_arm(self):
        cause = ["C:dropout:start=5", "B:digest:start=100,end=200"]  # C dies; B's digest diverges for a while: unattributable, Safe
        self.replay(cause, "--expect-mode", "safe", nodes=(0, 1, 2), frames=400, commands=["250:clear-safe"])  # refused: still Safe
        self.replay(cause, "--expect-mode", "duplex", nodes=(0, 1, 2), frames=400, commands=["250:armed-clear-safe"])
        out = self.replay([], "--expect-state", "B:disabled", frames=300, commands=["100:disable:B", "200:clear-disabled:B"])
        self.assertGreaterEqual(int(out["commands_refused"]), 1)
        self.replay([], "--expect-state", "B:latched", frames=300, commands=["100:disable:B", "200:armed-clear-disabled:B"])

    def test_F52_an_arm_that_is_never_followed_expires(self):
        out = self.replay([], "--expect-state", "B:disabled", frames=700, commands=["100:disable:B", "200:arm-clear-disabled:B", "500:clear-disabled:B"])
        self.assertEqual(out["arms_expired"], "1")  # 250 frames after 200

    def test_F53_the_interlock_tiers(self):
        # Triplex -> Duplex: plain. Duplex -> Simplex: refused unless armed. The last voter: armed, and reported loudly.
        self.replay([], "--expect-state", "B:disabled", "--expect-mode", "duplex", frames=200, commands=["100:disable:B"])
        out = self.replay([], "--expect-state", "B:healthy", "--expect-mode", "duplex", nodes=(0, 1), frames=200, commands=["100:disable:B"])
        self.assertGreaterEqual(int(out["commands_refused"]), 1)  # C is absent: Duplex; a plain disable of B would leave one voter
        self.replay([], "--expect-state", "B:disabled", "--expect-mode", "simplex", nodes=(0, 1), frames=200, commands=["100:armed-disable:B"])
        out = self.replay([], "--expect-state", "A:disabled", "--expect-mode", "safe", nodes=(0,), frames=300, commands=["200:armed-disable:A"])
        self.assertEqual(out["critical_commands"], "1")

    def test_F53_a_node_that_is_already_out_of_the_vote_can_be_disabled_plainly(self):
        self.replay(["B:dropout:start=50"], "--expect-state", "B:disabled", "--expect-mode", "duplex", frames=300, commands=["200:disable:B"])


if __name__ == "__main__":
    unittest.main()
