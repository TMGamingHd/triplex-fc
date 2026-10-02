# SPDX-License-Identifier: MIT
"""Live: the real FC-A firmware (Zephyr native_sim, real time) against virtual peers on vcan0.

FC-A is the sync master; the peers follow its SYNC. Everything FC-A decides comes from its console.
Skipped unless the native_sim binary is built (TFC_FC_BIN, or build/native_sim) and vcan0 exists
(sim/scripts/setup_vcan.sh).
"""
import os
import re
import subprocess
import time
import unittest
from pathlib import Path

from tfc_peers import bus as B
from tfc_peers.commands import parse_commands
from tfc_peers.faults import parse_fault
from tfc_peers.peers import Scenario

ROOT = Path(__file__).resolve().parents[2]
CANDIDATES = [os.environ.get("TFC_FC_BIN", ""), str(ROOT / "build/native_sim/zephyr/zephyr.exe")]
FC_BIN = next((c for c in CANDIDATES if c and os.access(c, os.X_OK)), None)
HAVE_VCAN = Path("/sys/class/net/vcan0").exists()
LATCH = re.compile(r"\[frame (\d+)\] node ([ABC]) LATCHED OUT: (.*)")
JOIN = re.compile(r"\[frame (\d+)\] node ([ABC]) joined the bus")
STATUS = re.compile(r"\[frame (\d+)\] (\w+)\s+A(.) B(.) C(.)\s+\| crc=(\d+) seq=(\d+) missing=(\d+) vote=(\d+) digest=(\d+)")


@unittest.skipIf(FC_BIN is None, "FC-A native_sim binary not built (west build -b native_sim/native/64 ...)")
@unittest.skipIf(not HAVE_VCAN, "vcan0 not present (run sim/scripts/setup_vcan.sh)")
class LiveFcAgainstPeers(unittest.TestCase):
    def live(self, faults, peer_frames=450, nodes=(1, 2), commands=()):
        """Start FC-A, then the peers `0.3 s` later; return FC-A's console lines."""
        stop_s = 0.3 + peer_frames / 100 + 0.6
        fc = subprocess.Popen([FC_BIN, f"-stop_at={stop_s:.1f}"], stdout=subprocess.PIPE,
                              stderr=subprocess.STDOUT, text=True)
        try:
            time.sleep(0.3)
            bus = B.SocketCanBus("vcan0")
            try:
                B.run_synced(Scenario(list(nodes), [parse_fault(f) for f in faults], 1,
                                      [x for c in commands for x in parse_commands(c)]), bus, peer_frames)
            finally:
                bus.close()
            out, _ = fc.communicate(timeout=30)
        finally:
            if fc.poll() is None:
                fc.kill()
        return out.splitlines()

    def test_healthy_peers_join_and_never_raise_an_alarm(self):
        lines = self.live([])
        text = "\n".join(lines)
        joined = {m.group(2) for m in map(JOIN.match, lines) if m}
        self.assertEqual(joined, {"B", "C"}, text)
        self.assertIn("MODE SIMPLEX -> TRIPLEX", text)
        # While the peers are alive (before they finish): nobody latches, the mode stays TRIPLEX, and
        # there are no CRC, sequence, vote or digest alarms. `missing` may be non-zero: on a busy machine
        # (shared CI runner) a Python peer frame occasionally lands after FC-A's 7 ms vote. The 3-of-5
        # filter absorbs those; require only that they stay rare.
        first_latch = min((int(m.group(1)) for m in map(LATCH.match, lines) if m), default=10**9)
        checked = 0
        for m in map(STATUS.match, lines):
            if m and int(m.group(1)) > 100 and int(m.group(1)) < first_latch:
                checked += 1
                frame, crc, seq, missing, vote, digest = int(m.group(1)), *map(int, m.groups()[5:])
                self.assertEqual(m.group(2), "TRIPLEX", text)
                self.assertEqual((crc, seq, vote, digest), (0, 0, 0, 0), text)
                self.assertLessEqual(missing, 0.05 * frame + 2, text)
        self.assertGreaterEqual(checked, 2, text)

    def latches(self, lines):
        return {m.group(2): (int(m.group(1)), m.group(3)) for m in map(LATCH.match, lines) if m}

    def assert_latch(self, lines, node, start, reason, slack=3):
        """`node` latched out for `reason`. Ideal timing is start+2 (3-of-5); an occasional late peer
        frame on a busy machine can add a bad sample, so allow `slack` frames either way."""
        text = "\n".join(lines)
        self.assertIn(node, self.latches(lines), text)
        frame, why = self.latches(lines)[node]
        self.assertGreaterEqual(frame, start, text)
        self.assertLessEqual(frame, start + 2 + slack, text)
        self.assertIn(reason, why, text)
        return frame

    def test_bias_on_B_is_isolated_two_frames_after_it_starts(self):
        lines = self.live(["B:bias:start=300,mag=3"], peer_frames=450)
        self.assert_latch(lines, "B", 300, "vote disagreement")
        self.assertIn("MODE TRIPLEX -> DUPLEX", "\n".join(lines))

    def test_digest_divergence_on_C_is_labelled_as_such(self):
        lines = self.live(["C:digest:start=300"], peer_frames=450)
        self.assert_latch(lines, "C", 300, "digest mismatch")

    def test_cmd_offset_on_B_is_a_vote_disagreement(self):
        lines = self.live(["B:cmd_offset:start=300"], peer_frames=450)
        self.assert_latch(lines, "B", 300, "vote disagreement")

    def test_babble_on_B_raises_the_bus_alarm_without_blaming_a_node(self):  # TFC-FDIR-009
        lines = self.live(["B:babble:start=300,n=8"], peer_frames=450)
        text = "\n".join(lines)
        alarm = [int(m.group(1)) for m in (re.match(r"\[frame (\d+)\] BUS ALARM RAISED", ln) for ln in lines) if m]
        self.assertTrue(alarm, text)
        self.assertGreaterEqual(alarm[0], 300, text)
        self.assertLessEqual(alarm[0], 306, text)  # the first flooded frame, give or take jitter
        # The stray IDs belong to no node: nobody is blamed for them (only the peers' end of run latches).
        self.assertNotIn("TRIPLEX -> DUPLEX", text)
        flooded = [m for m in map(STATUS.match, lines) if m and 320 < int(m.group(1))]
        self.assertTrue(any("oos=" in ln and int(re.search(r"oos=(\d+)", ln).group(1)) > 500 for ln in lines), text)
        self.assertTrue(flooded, text)

    def first(self, lines, pattern):
        for ln in lines:
            m = re.match(r"\[frame (\d+)\] " + pattern, ln)
            if m:
                return int(m.group(1))
        return None

    def test_transient_fault_then_operator_reintegration_end_to_end(self):  # TFC-FDIR-006/020
        # Bias on B for 30 frames (latched ~302), a vote latch is transient-looking so the dwell is 50 frames (ends ~352), the operator
        # asks at 450, probation starts at once (~450), 100 agreeing frames, readmitted ~550, back in Triplex.
        lines = self.live(["B:bias:start=300,end=330"], peer_frames=620, commands=["450:reintegrate:B"])
        text = "\n".join(lines)
        latched = self.first(lines, r"node B LATCHED OUT")
        probation = self.first(lines, r"node B ON PROBATION")
        back = self.first(lines, r"node B REINTEGRATED")
        self.assertTrue(300 <= (latched or 0) <= 306, text)
        self.assertIn("GROUND COMMAND reintegrate B: accepted", text)
        self.assertTrue(probation and 450 <= probation <= 458, text)
        self.assertTrue(back and probation + 100 <= back <= probation + 108, text)
        self.assertIn("MODE DUPLEX -> TRIPLEX", text)

    def test_authenticated_arm_execute_commands_and_the_interlock_live(self):  # TFC-FDIR-031/032/033
        # The tag is computed by the Python peers and checked by the real firmware (two independent SipHash implementations).
        # 450 disable C: Triplex -> Duplex, plain. 500 disable B: would leave one voter, refused. 550 armed-disable B: accepted.
        # 600: a forged command and a replay leave no trace on the console (only counters).
        lines = self.live([], peer_frames=700, commands=["450:disable:C", "500:disable:B", "550:armed-disable:B", "600:forged-clear-safe", "620:replay"])
        text = "\n".join(lines)
        self.assertIn("GROUND COMMAND disable C: accepted", text)
        self.assertIn("GROUND COMMAND disable B: refused: needs an ARM frame first", text)
        self.assertIn("GROUND COMMAND ARM disable B: accepted", text)
        self.assertRegex(text, r"GROUND COMMAND disable B: accepted")
        self.assertNotIn("CRITICAL", text)  # Simplex on A, not yet the last voter
        self.assertNotRegex(text, r"\[frame (6[0-9][0-9])\] GROUND COMMAND")  # the forged and the replayed frames were dropped silently

    def test_a_node_that_is_still_faulty_fails_probation_live(self):  # TFC-FDIR-020
        lines = self.live(["B:bias:start=300"], peer_frames=520, commands=["450:reintegrate:B"])
        text = "\n".join(lines)
        self.assertIsNotNone(self.first(lines, r"node B ON PROBATION"), text)
        failed = self.first(lines, r"node B FAILED PROBATION, back to latched: vote disagreement")
        self.assertIsNotNone(failed, text)
        self.assertNotIn("REINTEGRATED", text)
        self.assertNotIn("MODE DUPLEX -> TRIPLEX", text)  # the bad node never got back into the vote

    def test_intermittent_node_is_isolated_by_the_leaky_count_live(self):  # TFC-FDIR-023
        # One bad frame in three, from frame 300: 3-of-5 can never fire; the alpha-count does, ~20 frames in.
        lines = self.live(["B:corrupt:start=300,p=1.0,period=3,duty=1"], peer_frames=450)
        text = "\n".join(lines)
        m = next((re.match(r"\[frame (\d+)\] node B LATCHED OUT: (.*)", ln) for ln in lines
                  if re.match(r"\[frame (\d+)\] node B LATCHED OUT", ln)), None)
        self.assertIsNotNone(m, text)
        self.assertTrue(303 <= int(m.group(1)) <= 340, text)
        self.assertIn("intermittent fault", m.group(2), text)

    def test_silent_C_is_isolated_within_three_frames(self):
        lines = self.live(["C:dropout:start=300"], peer_frames=450)
        self.assert_latch(lines, "C", 300, "frame missing", slack=1)


if __name__ == "__main__":
    unittest.main()
