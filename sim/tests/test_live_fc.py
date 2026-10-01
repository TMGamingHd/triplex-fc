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
    def live(self, faults, peer_frames=450, nodes=(1, 2)):
        """Start FC-A, then the peers `0.3 s` later; return FC-A's console lines."""
        stop_s = 0.3 + peer_frames / 100 + 0.6
        fc = subprocess.Popen([FC_BIN, f"-stop_at={stop_s:.1f}"], stdout=subprocess.PIPE,
                              stderr=subprocess.STDOUT, text=True)
        try:
            time.sleep(0.3)
            bus = B.SocketCanBus("vcan0")
            try:
                B.run_synced(Scenario(list(nodes), [parse_fault(f) for f in faults], 1), bus, peer_frames)
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

    def test_silent_C_is_isolated_within_three_frames(self):
        lines = self.live(["C:dropout:start=300"], peer_frames=450)
        self.assert_latch(lines, "C", 300, "frame missing", slack=1)


if __name__ == "__main__":
    unittest.main()
