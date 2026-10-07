# SPDX-License-Identifier: MIT
import os
import tempfile
import unittest

from tfc_peers import bus as B
from tfc_peers import protocol as P
from tfc_peers.commands import parse_command
from tfc_peers.faults import FaultSpecError, parse_fault
from tfc_peers.peers import BABBLE_ID_BASE, FRAME_US, Scenario


def traffic(nodes, faults, frames, seed=1):
    sc = Scenario(nodes, [parse_fault(f) for f in faults], seed)
    return [sc.frames(k) for k in range(frames)]


def by_id(frames_k, can_id):
    return [tf.frame for tf in frames_k if tf.frame.id == can_id]


class HealthyTraffic(unittest.TestCase):
    def test_one_frame_has_nine_scheduled_frames_in_order_within_10ms(self):
        (k0,) = traffic([0, 1, 2], [], 1)
        self.assertEqual(len(k0), 9)
        times = [tf.t_us for tf in k0]
        self.assertEqual(times, sorted(times))
        self.assertTrue(all(0 <= t < FRAME_US for t in times))
        ids = {tf.frame.id for tf in k0}
        self.assertEqual(ids, {0x100, 0x101, 0x102, 0x110, 0x111, 0x112, 0x200, 0x201, 0x202})

    def test_sensors_follow_architecture_schedule(self):
        (k0,) = traffic([0], [], 1)
        t = {tf.frame.id: tf.t_us for tf in k0}
        self.assertTrue(1500 <= t[0x100] < 3000 and 1500 <= t[0x110] < 3000)  # sensor exchange
        self.assertTrue(5000 <= t[0x200] < 6500)  # command slot

    def test_healthy_replicas_send_identical_commands(self):
        for k in traffic([0, 1, 2], [], 50):
            cmds = [by_id(k, 0x200 + n)[0] for n in range(3)]
            self.assertEqual({c.data[:6] for c in cmds}, {cmds[0].data[:6]})

    def test_sensor_noise_is_independent_per_node(self):
        (k0,) = traffic([1, 2], [], 1)
        self.assertNotEqual(by_id(k0, 0x101)[0].data[:6], by_id(k0, 0x102)[0].data[:6])

    def test_all_frames_have_good_crc_and_sequential_seq(self):
        last = {}
        for k, fk in enumerate(traffic([0, 1, 2], [], 300)):
            for tf in fk:
                self.assertTrue(P.check(tf.frame))
                if tf.frame.id in last:
                    self.assertTrue(P.seq_is_next(last[tf.frame.id], tf.frame.data[6]))
                last[tf.frame.id] = tf.frame.data[6]

    def test_deterministic_for_a_seed_and_different_across_seeds(self):
        def flat(seed):
            return [(tf.t_us, tf.frame.id, tf.frame.data) for fk in traffic([1, 2], [], 30, seed) for tf in fk]
        self.assertEqual(flat(1), flat(1))
        self.assertNotEqual(flat(1), flat(2))

    def test_cannot_go_backwards_without_reset(self):
        sc = Scenario([1])
        sc.frames(0)
        sc.frames(1)
        with self.assertRaises(ValueError):
            sc.frames(1)
        sc.reset()
        self.assertEqual(sc.next_frame, 0)
        sc.frames(0)

    def test_joining_late_gives_exactly_what_a_run_from_zero_gives(self):
        faults = [parse_fault("B:stuck:start=40"), parse_fault("C:spike:start=0")]
        full = Scenario([1, 2], faults, 5)
        expected = [[(tf.t_us, tf.frame.id, tf.frame.data) for tf in full.frames(k)] for k in range(80)]
        late = Scenario([1, 2], faults, 5)
        for k in (57, 58, 70):  # first request jumps ahead, later ones may skip too
            self.assertEqual([(tf.t_us, tf.frame.id, tf.frame.data) for tf in late.frames(k)], expected[k])


class FaultEffects(unittest.TestCase):
    def gyro(self, fk, node):
        return P.unpack_vec3(by_id(fk, 0x100 + node)[0], P.GYRO_LSB_DPS).values

    def test_dropout_silences_only_that_node_inside_the_window(self):
        t = traffic([0, 1, 2], ["B:dropout:start=10,end=20"], 30)
        for k, fk in enumerate(t):
            present = {tf.frame.id & 0xF for tf in fk if tf.frame.id in (0x101, 0x111, 0x201)}
            self.assertEqual(bool(present), not 10 <= k < 20, f"frame {k}")
            self.assertTrue(by_id(fk, 0x100) and by_id(fk, 0x102))

    def test_bias_shifts_one_axis_by_magnitude(self):
        healthy = traffic([1], [], 40)
        faulty = traffic([1], ["B:bias:start=20,mag=3,axis=1"], 40)
        for k in range(40):
            dy = self.gyro(faulty[k], 1)[1] - self.gyro(healthy[k], 1)[1]
            self.assertAlmostEqual(dy, 3.0 if k >= 20 else 0.0, delta=0.13, msg=f"frame {k}")

    def test_drift_grows_linearly(self):
        healthy = traffic([1], [], 60)
        faulty = traffic([1], ["B:drift:start=10,rate=0.1"], 60)
        for k in (10, 30, 59):
            d = self.gyro(faulty[k], 1)[0] - self.gyro(healthy[k], 1)[0]
            self.assertAlmostEqual(d, 0.1 * (k - 10 + 1), delta=0.13)

    def test_stuck_freezes_output_bit_identically(self):
        t = traffic([1], ["B:stuck:start=20"], 60)
        frozen = {by_id(fk, 0x101)[0].data[:6] for fk in t[20:]}
        frozen_a = {by_id(fk, 0x111)[0].data[:6] for fk in t[20:]}
        self.assertEqual((len(frozen), len(frozen_a)), (1, 1))
        self.assertNotEqual(frozen, {by_id(t[0], 0x101)[0].data[:6]})

    def test_saturate_pins_outputs_to_full_scale(self):
        t = traffic([1], ["B:saturate:start=5"], 8)
        # sign alternates with frame parity: odd frames negative, even frames positive
        self.assertEqual(self.gyro(t[5], 1), (-1024.0, -1024.0, -1024.0))
        self.assertEqual(self.gyro(t[6], 1), (1023.96875, 1023.96875, 1023.96875))

    def test_spike_is_sparse_and_large(self):
        healthy = traffic([1], [], 400)
        faulty = traffic([1], ["B:spike:start=0,mag=20,p=0.05"], 400)
        big = [k for k in range(400) if max(abs(a - b) for a, b in
               zip(self.gyro(faulty[k], 1), self.gyro(healthy[k], 1))) > 15]
        self.assertTrue(5 <= len(big) <= 40, len(big))

    def test_corrupt_breaks_crc_on_some_frames_only(self):
        t = traffic([1], ["B:corrupt:start=0,p=0.3"], 200)
        flat = [tf.frame for fk in t for tf in fk]
        bad = [f for f in flat if not P.check(f)]
        self.assertTrue(0.15 * len(flat) < len(bad) < 0.45 * len(flat), (len(bad), len(flat)))

    def test_seqgap_skips_sequence_once(self):
        t = traffic([1], ["B:seqgap:start=10,gap=4"], 20)
        seqs = [by_id(fk, 0x101)[0].data[6] for fk in t]
        steps = [(b - a) & 0xFF for a, b in zip(seqs, seqs[1:])]
        self.assertEqual([i + 1 for i, s in enumerate(steps) if s != 1], [10])
        self.assertEqual(steps[9], 5)

    def test_cmd_offset_changes_command_but_keeps_crc_valid(self):
        healthy, faulty = traffic([1], [], 5), traffic([1], ["B:cmd_offset:start=2,mag=1.5"], 5)
        for k in range(5):
            h = P.unpack_cmd(by_id(healthy[k], 0x201)[0])
            f = P.unpack_cmd(by_id(faulty[k], 0x201)[0])
            self.assertAlmostEqual(f.pitch_deg - h.pitch_deg, 1.5 if k >= 2 else 0.0, places=3)

    def test_digest_fault_changes_only_the_digest(self):
        healthy, faulty = traffic([1], [], 5), traffic([1], ["B:digest:start=2,xor=16"], 5)
        h, f = (P.unpack_cmd(by_id(x[3], 0x201)[0]) for x in (healthy, faulty))
        self.assertEqual((h.pitch_deg, h.yaw_deg), (f.pitch_deg, f.yaw_deg))
        self.assertEqual(h.digest ^ f.digest, 16)

    def test_babble_adds_out_of_schedule_high_priority_frames(self):
        t = traffic([1], ["B:babble:start=5,n=7"], 10)
        self.assertEqual(len([tf for tf in t[4] if tf.frame.id < 0x100]), 0)
        extra = [tf for tf in t[5] if tf.frame.id < 0x100]
        self.assertEqual(len(extra), 7)
        self.assertTrue(all(BABBLE_ID_BASE <= tf.frame.id < 0x100 and P.check(tf.frame) for tf in extra))

    def test_fault_windows_are_respected(self):
        t = traffic([1], ["B:bias:start=5,end=8,mag=50"], 12)
        big = [k for k in range(12) if abs(self.gyro(t[k], 1)[0]) > 30]
        self.assertEqual(big, [5, 6, 7])


class RecoveryFaults(unittest.TestCase):
    def seqs(self, t, node=1):
        return [by_id(fk, 0x100 + node)[0].data[6] if by_id(fk, 0x100 + node) else None for fk in t]

    def test_reboot_is_silent_then_back_in_phase_with_sync(self):
        t = traffic([1], ["B:reboot:start=20,down=10"], 50)
        self.assertEqual(self.seqs(t)[19], 19)
        self.assertEqual(set(self.seqs(t)[20:30]), {None})          # silent
        self.assertEqual(self.seqs(t)[30:33], [30, 31, 32])           # back, numbered from SYNC: no break
        self.assertEqual(self.seqs(t)[49], 49)

    def test_a_reboot_that_does_not_resync_restarts_its_frame_number_at_zero(self):
        t = traffic([1], ["B:reboot:start=20,down=10,resync=0"], 50)
        self.assertEqual(self.seqs(t)[30:33], [0, 1, 2])
        self.assertEqual(self.seqs(t)[49], 19)

    def test_seqgap_offsets_the_frame_number_while_it_is_active(self):
        t = traffic([1], ["B:seqgap:start=10,end=12,gap=5"], 20)
        self.assertEqual(self.seqs(t)[8:14], [8, 9, 15, 16, 12, 13])

    def test_late_shifts_every_scheduled_frame_inside_the_window_only(self):
        base = traffic([1], [], 20)
        late = traffic([1], ["B:late:start=5,end=8,us=4000"], 20)
        for k in range(20):
            for a, b in zip(base[k], late[k]):
                self.assertEqual(b.t_us - a.t_us, 4000 if 5 <= k < 8 else 0, k)
                self.assertEqual(a.frame.data, b.frame.data)  # same data, just later

    def test_late_must_leave_inside_the_frame(self):
        with self.assertRaises(FaultSpecError):
            parse_fault("B:late:us=9500")

    def test_intermittent_option_gates_any_fault(self):
        t = traffic([1], ["B:dropout:start=10,period=5,duty=2"], 40)
        silent = [k for k in range(40) if not by_id(t[k], 0x101)]
        self.assertEqual(silent, [10, 11, 15, 16, 20, 21, 25, 26, 30, 31, 35, 36])
        self.assertEqual(str(parse_fault("B:dropout:start=10,period=5,duty=2")), "B:dropout:start=10,period=5,duty=2")

    def test_bad_intermittent_patterns_are_rejected(self):
        for bad in ("B:bias:period=1", "B:bias:period=5,duty=5", "B:bias:period=5,duty=0", "B:bias:duty=2"):
            with self.subTest(bad), self.assertRaises(FaultSpecError):
                parse_fault(bad)

    def test_scripted_commands_become_ground_frames_in_the_right_frame(self):
        sc = Scenario([1], commands=[parse_command("5:reintegrate:B"), parse_command("7:clear-safe")])
        got = {}
        for k in range(10):
            for tf in sc.frames(k):
                if tf.frame.id == 0x510:
                    got[k] = (tf.t_us - k * 10_000, P.unpack_ground(tf.frame))
        self.assertEqual(sorted(got), [5, 7])
        self.assertEqual(got[5][0], 6500)  # before the 7 ms vote, so it applies in that very frame
        self.assertEqual((got[5][1].op, got[5][1].node), (1, 1))
        self.assertEqual(got[7][1].op, 4)

    def test_command_specs(self):
        self.assertEqual(str(parse_command("450:Reintegrate:b")), "450:reintegrate:B")
        self.assertEqual(str(parse_command("600:clear-safe")), "600:clear-safe")
        for bad in ("450", "x:reintegrate:B", "450:explode:B", "450:reintegrate", "-1:disable:A", "450:disable:D"):
            with self.subTest(bad), self.assertRaises(FaultSpecError):
                parse_command(bad)

    def test_commands_survive_late_join_fast_forward(self):
        sc = Scenario([1], commands=[parse_command("30:disable:C")])
        ids = [tf.frame.id for tf in sc.frames(30)]
        self.assertIn(0x510, ids)  # frame 30 requested directly (late joiner) still carries its command


class FaultSpecParsing(unittest.TestCase):
    def test_valid_spec(self):
        f = parse_fault("b:bias:start=100,end=200,mag=2.5,axis=1")
        self.assertEqual((f.node, f.kind, f.start, f.end), (1, "bias", 100, 200))
        self.assertEqual((f.params["mag"], f.params["axis"], f.params["sensor"]), (2.5, 1, "gyro"))
        self.assertEqual(str(parse_fault(str(f))), str(f))  # round-trips

    def test_rejects_bad_specs(self):
        for bad in ("B", "D:bias", "B:nope", "B:bias:mag", "B:bias:colour=red", "B:bias:start=5,end=5",
                    "B:bias:axis=3", "B:bias:sensor=mag", "B:bias:start=-1"):
            with self.subTest(bad), self.assertRaises(FaultSpecError):
                parse_fault(bad)


class FakeSyncBus:
    """Delivers scripted SYNC frames immediately and records what the peers send."""

    def __init__(self, frame_numbers):
        self.queue = [P.pack_sync(n, n & 0xFF) for n in frame_numbers]
        self.sent = []
        self.filters = None

    def set_filter(self, filters):
        self.filters = filters

    def recv(self, timeout=None):
        return self.queue.pop(0) if self.queue else None

    def send(self, t_us, frame):
        self.sent.append((t_us, frame))


class FollowSync(unittest.TestCase):
    def test_sends_the_frame_numbered_by_sync_and_listens_only_for_sync(self):
        bus = FakeSyncBus([100, 101, 102])
        notes = []
        st = B.run_synced(Scenario([1, 2], seed=3), bus, 3, on_note=notes.append)
        self.assertEqual(bus.filters, [(0x010, 0x7FF)])
        self.assertEqual(st["sent"], 18)  # 3 frames x 2 nodes x 3 streams
        self.assertEqual(notes, [])
        ref = Scenario([1, 2], seed=3)
        expected = [tf for k in range(103) for tf in ref.frames(k) if k >= 100]
        self.assertEqual([(t, f.id, f.data) for t, f in bus.sent],
                         [(tf.t_us, tf.frame.id, tf.frame.data) for tf in expected])

    def test_sync_master_restart_restarts_the_peers(self):
        bus = FakeSyncBus([5, 6, 0, 1])
        notes = []
        st = B.run_synced(Scenario([1]), bus, 4, on_note=notes.append)
        self.assertEqual(st["rewinds"], 1)
        self.assertEqual(len(notes), 1)
        self.assertEqual(len(bus.sent), 12)

    def test_frames_zero_follows_until_ctrl_c_and_returns_stats(self):
        class StopAfter(FakeSyncBus):
            def recv(self, timeout=None):
                if not self.queue:
                    raise KeyboardInterrupt
                return self.queue.pop(0)

        bus = StopAfter([20, 21, 22, 23])
        st = B.run_synced(Scenario([1, 2]), bus, 0)
        self.assertEqual((st["frames"], st["sent"], st["interrupted"]), (4, 24, 1))

    def test_free_running_frames_zero_runs_until_interrupted(self):
        class Boom(B.ListBus):
            def send(self, t_us, frame):
                super().send(t_us, frame)
                if len(self.sent) == 20:
                    raise KeyboardInterrupt

        bus = Boom()
        st = B.run_realtime(Scenario([1]), bus, 0)
        self.assertEqual(st["interrupted"], 1)
        self.assertEqual(len(bus.sent), 20)

    def test_a_finite_run_is_not_marked_interrupted(self):
        st = B.run_synced(Scenario([1]), FakeSyncBus([1, 2]), 2)
        self.assertEqual(st["interrupted"], 0)

    def test_no_sync_raises_a_helpful_error(self):
        with self.assertRaisesRegex(TimeoutError, "sync master"):
            B.run_synced(Scenario([1]), FakeSyncBus([]), 1, sync_timeout_s=0.01)

    def test_bad_crc_sync_is_ignored(self):
        bus = FakeSyncBus([7])
        bad = P.Frame(0x010, bus.queue[0].data[:7] + bytes([bus.queue[0].data[7] ^ 1]))
        bus.queue.insert(0, bad)
        B.run_synced(Scenario([1]), bus, 1)
        self.assertEqual(len(bus.sent), 3)


class CliInputs(unittest.TestCase):
    def run_cli(self, *argv):
        import contextlib
        import io
        from tfc_peers.cli import main
        err = io.StringIO()
        with contextlib.redirect_stderr(err), contextlib.redirect_stdout(io.StringIO()):
            return main(list(argv)), err.getvalue()

    def test_empty_node_list_is_an_error_not_an_empty_log(self):
        code, err = self.run_cli("record", "--nodes", "", "--frames", "2", "--out", os.devnull)
        self.assertEqual(code, 2)
        self.assertIn("--nodes is empty", err)

    def test_node_list_forms(self):
        for nodes, per_frame in (("b,c", 6), ("0,1,2", 9), (" A , B ", 6)):
            with self.subTest(nodes=nodes):
                code, _ = self.run_cli("record", "--nodes", nodes, "--frames", "1", "--out", os.devnull)
                self.assertEqual(code, 0)

    def test_fault_on_unsimulated_node_and_unknown_node_are_errors(self):
        self.assertEqual(self.run_cli("record", "--nodes", "B", "--fault", "C:bias", "--out", os.devnull)[0], 2)
        self.assertEqual(self.run_cli("record", "--nodes", "D", "--out", os.devnull)[0], 2)


class Launcher(unittest.TestCase):
    def test_launcher_works_from_any_directory(self):
        import subprocess
        import sys
        import tempfile
        launcher = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "tfc-peers")
        self.assertTrue(os.access(launcher, os.X_OK), "sim/tfc-peers must be executable")
        with tempfile.TemporaryDirectory() as d:
            r = subprocess.run([sys.executable, launcher, "faults"], cwd=d, capture_output=True, text=True)
            self.assertEqual(r.returncode, 0, r.stderr)
            self.assertIn("dropout", r.stdout)
            r = subprocess.run([launcher, "decode", "/nonexistent"], cwd=d, capture_output=True, text=True)
            self.assertEqual(r.returncode, 2)  # runs directly (shebang) and reports errors normally


class Buses(unittest.TestCase):
    def test_log_roundtrip_in_candump_format(self):
        sc = Scenario([0, 1, 2], [parse_fault("C:corrupt:p=0.5")], 3)
        lb = B.ListBus()
        B.record(sc, lb, 20)
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, "x.log")
            fb = B.LogBus(path, "vcan7")
            for t, f in lb.sent:
                fb.send(t, f)
            fb.close()
            with open(path) as fh:
                first = fh.readline()
            self.assertRegex(first, r"^\(0\.001500\) vcan7 100#[0-9A-F]{16}$")
            back = list(B.read_log(path))
        self.assertEqual([(t, f.id, f.data) for t, f in lb.sent], [(t, f.id, f.data) for t, _, f in back])

    def test_read_log_rejects_garbage(self):
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, "bad.log")
            with open(path, "w") as fh:
                fh.write("not a candump line\n")
            with self.assertRaises(ValueError):
                list(B.read_log(path))

    def test_realtime_runner_sends_everything_roughly_on_time(self):
        lb = B.ListBus()
        st = B.run_realtime(Scenario([1, 2]), lb, 5)
        self.assertEqual(st["sent"], 30)
        self.assertEqual(len(lb.sent), 30)
        self.assertLess(st["late_p50_us"], 5000)  # generous: CI machines are noisy


if __name__ == "__main__":
    unittest.main()
