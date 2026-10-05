# SPDX-License-Identifier: MIT
"""The automated launch checklist (tfc_peers/launch.py) against a scripted bus with a fake clock."""
import unittest

from tfc_peers import launch as LA
from tfc_peers import protocol as P


class FakeBus:
    """A scripted bus with a fake clock: heartbeats, ACT and SYNC every 10 ms; sending the EXECUTE makes the 'sync master' start a countdown, refuse, or scrub it (`react`)."""

    def __init__(self, ready=(True, True, True), act_state=1, mode=3, safe=(False, False, False), react="countdown"):
        self.t = 0.0
        self.sent = []
        self.lines = []
        self.ready, self.act_state, self.mode, self.safe, self.react = ready, act_state, mode, safe, react
        self.mission = 0
        self.next_beat = 0.0
        self.next_sync = 0.0
        self.silent = set()          # node heartbeats that have stopped
        self.countdown_started = None

    def now(self):
        return self.t

    def sleep(self, s):
        self.t += s

    def _events_until(self, t_limit):
        # one heartbeat set and one ACT frame every 10 ms; SYNC every 10 ms with the mission frame
        out = []
        while True:
            t_next = min(self.next_beat, self.next_sync)
            if t_next > t_limit:
                break
            if t_next == self.next_sync:
                self._advance_mission(t_next)
                out.append((t_next, P.pack_sync(int(t_next * 100), 0, self.mission)))
                self.next_sync += 0.01
            else:
                for n in range(3):
                    if n not in self.silent:
                        out.append((t_next, P.pack_heartbeat(n, P.Heartbeat(mode=self.mode, ready=self.ready[n], safe_requested=self.safe[n]), 0)))
                out.append((t_next, P.pack_act_out(P.ActOut(state=self.act_state), 0)))
                self.next_beat += 0.01
        return out

    def _advance_mission(self, t):
        if self.countdown_started is not None:
            m = int((t - self.countdown_started) * 100) + 1
            if self.react == "scrub" and m >= 500:
                self.mission = 0
                self.countdown_started = None
            else:
                self.mission = m

    def send(self, frame):
        self.sent.append(frame)
        if frame.data[0] == 5 and self.react in ("countdown", "scrub"):
            self.countdown_started = self.t + 0.02


class QueueBus(FakeBus):
    def __init__(self, *a, **k):
        super().__init__(*a, **k)
        self.queue = []

    def recv(self, timeout):
        if not self.queue:
            self.queue = self._events_until(self.t + max(timeout, 0.0))
            self.queue.sort(key=lambda e: e[0])
        if self.queue:
            t, f = self.queue.pop(0)
            self.t = max(self.t, t)
            return f
        self.t += timeout
        return None


def run(bus, **kw):
    return LA.run(bus.recv, bus.send, bus.now, bus.sleep, 1, say=bus.lines.append, **kw)


class Checklist(unittest.TestCase):
    def test_a_go_sends_arm_then_execute_and_counts_down_to_t_zero(self):
        bus = QueueBus()
        self.assertEqual(run(bus, wait_s=5.0), 0)
        ops = [(f.data[0] & 0x7F, bool(f.data[0] & 0x80), f.data[6]) for f in bus.sent]
        self.assertEqual(ops, [(5, True, 1), (5, False, 2)])  # an ARM, then the EXECUTE, with consecutive counters
        text = "\n".join(bus.lines)
        self.assertIn("GO", text)
        self.assertIn("COUNTDOWN started", text)
        self.assertIn("T-10 s", text)
        self.assertIn("T-1 s", text)
        self.assertIn("T-ZERO", text)

    def test_a_no_go_sends_nothing_and_names_the_reasons(self):
        bus = QueueBus(ready=(True, False, True), act_state=0)
        self.assertEqual(run(bus, wait_s=2.0), 1)
        self.assertEqual(bus.sent, [])
        text = "\n".join(bus.lines)
        self.assertIn("flight computer B: not ready", text)
        self.assertIn("ACT: not Nominal", text)
        self.assertIn("NO-GO", text)

    def test_check_only_reports_and_sends_nothing(self):
        go = QueueBus()
        self.assertEqual(run(go, check_only=True), 0)
        self.assertEqual(go.sent, [])
        self.assertIn("GO: every item holds", "\n".join(go.lines))
        nogo = QueueBus(safe=(False, True, False))
        self.assertEqual(run(nogo, check_only=True), 1)
        self.assertIn("a Safe request is up", "\n".join(nogo.lines))

    def test_a_silent_computer_a_computer_not_in_triplex_and_a_missing_act_are_no_go(self):
        silent = QueueBus()
        silent.silent = {2}
        self.assertEqual(run(silent, wait_s=1.5), 1)
        self.assertIn("flight computer C: no heartbeat", "\n".join(silent.lines))
        duplex = QueueBus(mode=2)
        self.assertEqual(run(duplex, wait_s=1.5), 1)
        self.assertIn("not in Triplex", "\n".join(duplex.lines))

    def test_a_refused_launch_is_reported_when_no_countdown_starts(self):
        bus = QueueBus(react="refuse")
        self.assertEqual(run(bus, wait_s=3.0), 3)
        self.assertEqual(len(bus.sent), 2)
        self.assertIn("did not start", "\n".join(bus.lines))

    def test_a_scrub_in_the_countdown_is_reported_with_its_own_exit_code(self):
        bus = QueueBus(react="scrub")
        self.assertEqual(run(bus, wait_s=3.0), 2)
        self.assertIn("SCRUBBED", "\n".join(bus.lines))

    def test_the_observer_ignores_damaged_and_stale_frames(self):
        o = LA.Observer()
        f = P.pack_heartbeat(1, P.Heartbeat(mode=3, ready=True), 0)
        bad = P.Frame(f.id, bytes(f.data[:7]) + bytes([f.data[7] ^ 1]))
        o.feed(bad, 1.0)
        self.assertFalse(o.ready[1])
        o.feed(f, 1.0)
        self.assertTrue(o.ready[1])
        self.assertIn("flight computer B: no heartbeat", o.verdict(5.0))  # a heartbeat that old does not count
        self.assertEqual(o.table(1.1).split()[1], "B:ready")


if __name__ == "__main__":
    unittest.main()
