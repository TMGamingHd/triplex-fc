# SPDX-License-Identifier: MIT
"""The fault kinds added by the FMEA gap analysis (docs/verification/FMEA.md, FAULT_MATRIX F27-F45): what each one puts on the
wire, parameter validation, and a regression check that the older kinds' traffic did not change."""
import hashlib
import math
import tempfile
import unittest

from tfc_peers import bus as B
from tfc_peers import protocol as P
from tfc_peers.faults import KINDS, FaultSpecError, parse_fault
from tfc_peers.peers import FRAME_US, Scenario, truth


def run(faults, frames=120, seed=1, nodes=(0, 1, 2)):
    sc = Scenario(list(nodes), [parse_fault(f) for f in faults], seed)
    return [sc.frames(k) for k in range(frames)]


def gyro(frames_k, node=1):
    f = [tf.frame for tf in frames_k if tf.frame.id == P.ID_GYRO_BASE + node]
    return [P.unpack_vec3(x, P.GYRO_LSB_DPS) for x in f]


def accel(frames_k, node=1):
    f = [tf.frame for tf in frames_k if tf.frame.id == P.ID_ACCEL_BASE + node]
    return [P.unpack_vec3(x, P.ACCEL_LSB_G) for x in f]


def cmd(frames_k, node=1):
    f = [tf.frame for tf in frames_k if tf.frame.id == P.ID_CMD_BASE + node]
    return [P.unpack_cmd(x) for x in f]


def series(traffic, extract, axis=None, node=1):
    out = []
    for fk in traffic:
        for s in extract(fk, node):
            out.append(s.values[axis] if axis is not None else s)
    return out


class SensorFaults(unittest.TestCase):
    def setUp(self):
        self.base = run([])

    def test_scale_multiplies_one_axis(self):
        t = run(["B:scale:start=20,axis=0,factor=2.0"])
        a, b = series(self.base, gyro, 0), series(t, gyro, 0)
        self.assertEqual(a[:20], b[:20])
        for x, y in zip(a[20:], b[20:]):
            self.assertAlmostEqual(y, 2.0 * x, delta=0.13)  # one LSB of rounding
        self.assertEqual(series(self.base, gyro, 1), series(t, gyro, 1))  # other axes untouched

    def test_scale_on_accel_axis_z(self):
        t = run(["C:scale:start=0,sensor=accel,axis=2,factor=1.5"])
        z = series(t, lambda fk, n: accel(fk, n), 2, node=2)
        self.assertTrue(all(abs(v - 1.5) < 0.01 for v in z))

    def test_noise_raises_the_spread(self):
        t = run(["B:noise:start=0,mult=20"], frames=400)
        err = [v - truth(k * FRAME_US / 1e6 + 0.0005)[0][0] for k, v in enumerate(series(t, gyro, 0))]
        sigma = math.sqrt(sum(e * e for e in err) / len(err))
        self.assertGreater(sigma, 1.5)  # healthy sigma is 0.1; 20x is 2.0
        self.assertLess(sigma, 2.6)

    def test_invert_flips_one_axis(self):
        t = run(["B:invert:start=10,axis=1"])
        a, b = series(self.base, gyro, 1), series(t, gyro, 1)
        for x, y in zip(a[10:], b[10:]):
            self.assertAlmostEqual(y, -x, delta=0.13)
        self.assertEqual(series(self.base, gyro, 0), series(t, gyro, 0))

    def test_swap_exchanges_two_axes(self):
        t = run(["B:swap:start=0,axis=0,other=1"])
        a0, a1 = series(self.base, gyro, 0), series(self.base, gyro, 1)
        b0, b1 = series(t, gyro, 0), series(t, gyro, 1)
        for i in range(len(a0)):
            self.assertAlmostEqual(b0[i], a1[i], delta=0.13)
            self.assertAlmostEqual(b1[i], a0[i], delta=0.13)

    def test_zero_reads_exactly_zero_on_both_sensors(self):
        t = run(["B:zero:start=5"])
        for ax in range(3):
            self.assertTrue(all(v == 0.0 for v in series(t, gyro, ax)[5:]))
            self.assertTrue(all(v == 0.0 for v in series(t, accel, ax)[5:]))

    def test_clip_limits_the_output(self):
        t = run(["B:clip:start=0,limit=4"], frames=250)
        for ax in range(3):
            self.assertTrue(all(abs(v) <= 4.0 for v in series(t, gyro, ax)))
        self.assertGreater(max(abs(v) for v in series(self.base, gyro, 0)), 4.0)  # the healthy signal does exceed it

    def test_oscillate_adds_a_sinusoid_sampled_at_100_hz(self):
        t = run(["B:oscillate:start=0,axis=0,amp=3.0,hz=7.0"], frames=100)
        a, b = series(self.base, gyro, 0), series(t, gyro, 0)
        for k, (x, y) in enumerate(zip(a, b)):
            want = 3.0 * math.sin(2 * math.pi * 7.0 * (k * 0.01 + 0.0005))
            self.assertAlmostEqual(y - x, want, delta=0.13)

    def test_oscillate_at_the_sample_rate_aliases_to_a_constant_offset(self):
        t = run(["B:oscillate:start=0,axis=0,amp=3.0,hz=100"], frames=50)
        d = [y - x for x, y in zip(series(self.base, gyro, 0), series(t, gyro, 0))]
        self.assertTrue(all(abs(v - 3.0 * math.sin(2 * math.pi * 0.05)) < 0.13 for v in d))  # 100 Hz looks like DC

    def test_repeat_holds_the_sample_for_n_frames(self):
        t = run(["B:repeat:start=10,n=4"], frames=60)
        g = series(t, gyro, 0)
        for blk in range(10, 58, 4):
            self.assertEqual(len(set(g[blk:blk + 4])), 1, f"block at {blk}")
        self.assertNotEqual(g[10], g[14])

    def test_bitflip_certain_flips_one_axis_by_the_bit_weight(self):
        t = run(["B:bitflip:start=0,bit=12,p=1.0"], frames=80)
        for a, b in zip(self.base, t):
            diffs = [abs(y - x) for x, y in zip(gyro(a)[0].values, gyro(b)[0].values)]
            self.assertEqual(sum(1 for d in diffs if d > 1.0), 1)  # exactly one axis hit
            self.assertAlmostEqual(max(diffs), 4096 * P.GYRO_LSB_DPS, delta=0.5)  # 128 dps

    def test_stuckbit_forces_the_bit_on_every_axis(self):
        t = run(["B:stuckbit:start=0,bit=12,value=1"], frames=40)
        for fk in t:
            fr = [tf.frame for tf in fk if tf.frame.id == P.ID_GYRO_BASE + 1][0]
            for ax in range(3):
                self.assertTrue(int.from_bytes(fr.data[2 * ax:2 * ax + 2], "little") & (1 << 12))

    def test_bit_faults_leave_the_crc_valid(self):
        for spec in ("B:bitflip:start=0,bit=3,p=1.0", "B:stuckbit:start=0,bit=9,value=0"):
            for fk in run([spec], frames=30):
                for tf in fk:
                    self.assertTrue(P.check(tf.frame))  # the upset happened before the CRC was computed

    def test_sensor_faults_only_touch_the_chosen_sensor(self):
        t = run(["B:scale:start=0,sensor=accel,axis=2,factor=0.5"])
        self.assertEqual(series(self.base, gyro, 0), series(t, gyro, 0))
        self.assertNotEqual(series(self.base, accel, 2), series(t, accel, 2))


class CommandAndFrameFaults(unittest.TestCase):
    def setUp(self):
        self.base = run([])

    def test_cmdstuck_freezes_the_command(self):
        t = run(["B:cmdstuck:start=30"], frames=100)
        pitch = [c.pitch_deg for fk in t for c in cmd(fk)]
        self.assertEqual(len(set(pitch[30:])), 1)
        self.assertGreater(len(set(c.pitch_deg for fk in self.base for c in cmd(fk))), 20)
        yaw = [c.yaw_deg for fk in t for c in cmd(fk)]
        self.assertEqual(len(set(yaw[30:])), 1)

    def test_cmdinvert_flips_both_axes(self):
        t = run(["B:cmdinvert:start=0"], frames=60)
        for a, b in zip(self.base, t):
            ca, cb = cmd(a)[0], cmd(b)[0]
            self.assertAlmostEqual(cb.pitch_deg, -ca.pitch_deg, delta=0.0011)
            self.assertAlmostEqual(cb.yaw_deg, -ca.yaw_deg, delta=0.0011)

    def test_partial_masks_suppress_exactly_the_chosen_frame_types(self):
        for mask, gone in ((1, {P.ID_GYRO_BASE}), (2, {P.ID_ACCEL_BASE}), (4, {P.ID_CMD_BASE}),
                           (3, {P.ID_GYRO_BASE, P.ID_ACCEL_BASE}), (7, {P.ID_GYRO_BASE, P.ID_ACCEL_BASE, P.ID_CMD_BASE})):
            t = run([f"B:partial:start=0,mask={mask}"], frames=5)
            present = {tf.frame.id for fk in t for tf in fk}
            for base_id in (P.ID_GYRO_BASE, P.ID_ACCEL_BASE, P.ID_CMD_BASE):
                self.assertEqual(base_id + 1 in present, base_id not in gone, f"mask {mask} id {base_id + 1:#x}")
            self.assertTrue({0x100, 0x102, 0x200, 0x202} <= present)  # the other nodes are unaffected

    def test_duplicate_sends_every_scheduled_frame_twice(self):
        t = run(["B:duplicate:start=10,gap_us=300"], frames=20)
        for k in (5, 15):
            n = sum(1 for tf in t[k] if tf.frame.id in (0x101, 0x111, 0x201))
            self.assertEqual(n, 3 if k < 10 else 6)
        k = 15
        by = {}
        for tf in t[k]:
            by.setdefault(tf.frame.id, []).append(tf)
        for i in (0x101, 0x111, 0x201):
            a, b = by[i]
            self.assertEqual(a.frame, b.frame)
            self.assertEqual(b.t_us - a.t_us, 300)

    def test_replay_resends_old_frames_with_old_counters(self):
        t = run(["B:replay:start=30,age=7"], frames=60)
        for k in (30, 45):
            for i in (0x101, 0x111, 0x201):
                sent = [tf.frame for tf in t[k] if tf.frame.id == i][0]
                old = [tf.frame for tf in self.base_k(k - 7) if tf.frame.id == i][0]
                self.assertEqual(sent.data, old.data)
                self.assertTrue(P.check(sent))
        self.assertEqual([tf.frame for tf in t[29] if tf.frame.id == 0x101],
                         [tf.frame for tf in self.base_k(29) if tf.frame.id == 0x101])

    def base_k(self, k):
        return self.base[k]

    def test_replay_reaches_back_the_full_300_frames(self):
        t = run(["B:replay:start=300,age=300"], frames=301)
        sent = [tf.frame for tf in t[300] if tf.frame.id == 0x101][0]
        old = [tf.frame for tf in run([], frames=1)[0] if tf.frame.id == 0x101][0]
        self.assertEqual(sent.data, old.data)

    def test_seqstuck_freezes_the_counter(self):
        t = run(["B:seqstuck:start=20"], frames=60)
        seqs = [tf.frame.data[6] for fk in t for tf in fk if tf.frame.id == 0x101]
        self.assertEqual(seqs[:20], list(range(20)))
        self.assertEqual(set(seqs[20:]), {20})


class TimingFaults(unittest.TestCase):
    def times(self, spec, ident=0x101, frames=40):
        base = {k: [tf.t_us for tf in fk if tf.frame.id == ident] for k, fk in enumerate(run([], frames=frames))}
        t = {k: [tf.t_us for tf in fk if tf.frame.id == ident] for k, fk in enumerate(run([spec], frames=frames))}
        return base, t

    def test_early_moves_every_frame_earlier(self):
        base, t = self.times("B:early:start=10,us=1000")
        self.assertEqual(t[5], base[5])
        for k in range(10, 40):
            self.assertEqual(t[k][0], base[k][0] - 1000)

    def test_early_never_goes_before_time_zero(self):
        _, t = self.times("B:early:start=0,us=9000", frames=3)
        self.assertEqual(t[0][0], 0)

    def test_jitter_stays_inside_the_band_and_is_deterministic(self):
        base, t = self.times("B:jitter:start=0,us=700", frames=100)
        diffs = [t[k][0] - base[k][0] for k in range(100)]
        self.assertTrue(all(abs(d) <= 700 for d in diffs))
        self.assertGreater(len(set(diffs)), 50)
        _, t2 = self.times("B:jitter:start=0,us=700", frames=100)
        self.assertEqual(t, t2)

    def test_clockdrift_accumulates(self):
        base, t = self.times("B:clockdrift:start=10,us_per_frame=20")
        for k in (10, 20, 39):
            self.assertEqual(t[k][0] - base[k][0], 20 * (k - 10 + 1))

    def test_negative_clockdrift_is_early_and_clamped(self):
        _, t = self.times("B:clockdrift:start=0,us_per_frame=-1000", frames=5)
        self.assertEqual(t[0][0], 700)  # node B gyro slot 1700, minus one frame of drift (1000)
        self.assertEqual(t[3][0], 3 * FRAME_US + 1700 - 4000)

    def test_record_sorts_the_log_and_stops_at_the_end(self):
        sc = Scenario([0, 1, 2], [parse_fault("B:clockdrift:start=0,us_per_frame=1000")], 1)
        with tempfile.TemporaryDirectory() as d:
            bus = B.LogBus(f"{d}/t.log")
            n = B.record(sc, bus, 50)
            bus.close()
            rows = list(B.read_log(f"{d}/t.log"))
        self.assertEqual(len(rows), n)
        times = [r[0] for r in rows]
        self.assertEqual(times, sorted(times))
        self.assertTrue(all(t < 50 * FRAME_US for t in times))  # frames the drifting node would send later are not in the log


class BabbleIds(unittest.TestCase):
    def test_babble_id_moves_the_flood_to_other_out_of_schedule_ids(self):
        from tfc_peers.peers import BABBLE_ID_BASE
        self.assertEqual(KINDS["babble"][2]["id"], BABBLE_ID_BASE)  # the default is the original behaviour
        t = run(["B:babble:start=0,n=20,id=0x520"], frames=3)
        ids = {tf.frame.id for fk in t for tf in fk if tf.frame.id >= 0x500}
        self.assertEqual(ids, set(range(0x520, 0x530)))
        self.assertTrue(all(P.check(tf.frame) for fk in t for tf in fk))

    def test_babble_ids_that_overlap_the_schedule_are_rejected(self):
        for bad in ("B:babble:id=0x100", "B:babble:id=0xFA", "B:babble:id=0x501", "B:babble:id=0x7F1", "B:babble:id=-1", "B:babble:id=0x4F8"):
            with self.subTest(spec=bad):
                with self.assertRaises(FaultSpecError):
                    parse_fault(bad)
        for good in ("B:babble:id=0x020", "B:babble:id=0x4F0", "B:babble:id=0x511", "B:babble:id=0x7F0", "B:babble:id=1312"):
            parse_fault(good)


class Validation(unittest.TestCase):
    def test_every_new_kind_is_listed_with_a_unique_fault_matrix_row(self):
        rows = [row for row, _d, _p in KINDS.values()]
        new = [r for r in rows if r.startswith("F") and int(r[1:]) >= 27]
        self.assertEqual(len(new), 19)
        self.assertEqual(len(set(new)), 19)
        self.assertEqual(len(KINDS), 32)

    def test_bad_values_are_rejected_with_a_message(self):
        for bad in ("B:scale:factor=abc", "B:scale:factor=nan", "B:scale:factor=inf", "B:scale:axis=3", "B:scale:axis=1.5",
                    "B:scale:sensor=mag", "B:noise:mult=0.5", "B:swap:axis=1,other=1", "B:clip:limit=0", "B:clip:limit=-1",
                    "B:oscillate:hz=0", "B:oscillate:amp=-1", "B:repeat:n=1", "B:repeat:n=0", "B:bitflip:bit=16", "B:bitflip:bit=-1",
                    "B:bitflip:p=1.5", "B:stuckbit:bit=16", "B:stuckbit:value=2", "B:partial:mask=0", "B:partial:mask=8",
                    "B:duplicate:gap_us=-1", "B:duplicate:gap_us=10000", "B:replay:age=0", "B:replay:age=301", "B:early:us=0",
                    "B:early:us=9001", "B:jitter:us=0", "B:jitter:us=9001", "B:clockdrift:us_per_frame=0",
                    "B:clockdrift:us_per_frame=1001", "B:spike:p=2", "B:corrupt:p=-0.1", "B:babble:n=201", "B:babble:n=-1",
                    "B:reboot:down=0", "B:late:us=0", "B:late:us=9001", "B:bias:mag=nan", "B:drift:rate=inf"):
            with self.subTest(spec=bad):
                with self.assertRaises(FaultSpecError):
                    parse_fault(bad)

    def test_good_values_parse_and_numbers_are_normalised(self):
        f = parse_fault("B:scale:axis=2.0,factor=3")
        self.assertEqual(f.params["axis"], 2)
        self.assertIsInstance(f.params["axis"], int)
        self.assertIsInstance(f.params["factor"], float)
        for spec in ("A:zero", "C:cmdstuck:start=5,end=9", "B:partial:mask=7", "B:replay:age=300", "B:clockdrift:us_per_frame=-1000",
                     "B:babble:n=0", "B:seqgap:gap=0", "B:bitflip:bit=15,p=0.0"):
            parse_fault(spec)

    def test_every_kind_runs_with_its_defaults(self):
        for kind in KINDS:
            with self.subTest(kind=kind):
                run([f"B:{kind}:start=2"], frames=40)

    def test_every_kind_works_as_an_intermittent_fault(self):
        for kind in KINDS:
            with self.subTest(kind=kind):
                run([f"B:{kind}:start=2,period=4,duty=2"], frames=40)

    def test_unchanged_traffic_of_the_first_thirteen_kinds(self):
        """Adding kinds must not change what the old kinds send (their own random streams, same code order)."""
        specs = ["B:dropout:start=10", "B:stuck:start=10", "B:bias:start=10,mag=2", "B:drift:start=10", "B:spike:start=0,p=0.3",
                 "B:saturate:start=5", "B:corrupt:start=5,p=0.3", "B:cmd_offset:start=5", "B:digest:start=5", "B:babble:start=5",
                 "B:seqgap:start=5", "B:reboot:start=20,down=7,resync=0", "B:late:start=5,us=3000", "C:corrupt:start=3,p=1.0,period=3,duty=1"]
        h = hashlib.sha256()
        for s in specs:
            sc = Scenario([0, 1, 2], [parse_fault(s)], 7)
            for k in range(80):
                for tf in sc.frames(k):
                    h.update(repr((tf.t_us, tf.frame.id, tf.frame.data)).encode())
        self.assertEqual(h.hexdigest(), "cd67ada9fce9a851fc854f12aee390fadb840e36b36c4664de6dd4730f2d73d3")  # re-pinned when the gyro frames went to 1/32 dps per count (ADR-032): the same faults, the same random streams, finer gyro bytes


if __name__ == "__main__":
    unittest.main()
