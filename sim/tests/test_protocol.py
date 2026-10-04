# SPDX-License-Identifier: MIT
import unittest

from tfc_peers import protocol as P

# Frames produced by the C++ implementation (core/include/tfc/protocol.hpp) for these inputs.
# If protocol.hpp changes, regenerate these from C++ first, then fix protocol.py to match.
GOLDEN = [
    ("gyro B {1,-2,3} seq5", P.pack_gyro(1, (1.0, -2.0, 3.0), 5), 0x101, "0800f0ff18000590"),
    ("accel C {0,.5,1} seq255", P.pack_accel(2, (0.0, 0.5, 1.0), 255), 0x112, "000000040008ffda"),
    ("gyro A saturating", P.pack_gyro(0, (9999.0, -9999.0, 0.06), 0), 0x100, "ff7f008000000063"),
    ("cmd B", P.pack_cmd(1, 1.234, -5.678, 0xBEEF, 7), 0x201, "d204d2e9efbe07bd"),
    ("cmd C saturating", P.pack_cmd(2, -40.0, 40.0, 1, 128), 0x202, "0080ff7f0100807f"),
    ("sync 0x01020304 seq9", P.pack_sync(0x01020304, 9), 0x010, "0403020100000915"),
    ("sync 0 seq0", P.pack_sync(0, 0), 0x010, "000000000000000a"),
    ("sync max seq255", P.pack_sync(0xFFFFFFFF, 255), 0x010, "ffffffff0000ff7b"),
    # authenticated ground frames (SipHash-2-4 tag under the public bench key): the bytes were produced by core/ (pack_ground_auth)
    ("ground reintegrate B counter5", P.pack_ground(P.GROUND_OPS["reintegrate"], 1, 5), 0x510, "01015111f4fc05c0"),
    ("ground disable C counter0", P.pack_ground(P.GROUND_OPS["disable"], 2, 0), 0x510, "02027c782ce00045"),
    ("ground clear-disabled A counter255", P.pack_ground(P.GROUND_OPS["clear-disabled"], 0, 255), 0x510, "0300d314200cffe9"),
    ("ground clear-safe counter9", P.pack_ground(P.GROUND_OPS["clear-safe"], 0, 9), 0x510, "04007c5aaa820940"),
    ("ground ARM disable B counter8", P.pack_ground(P.GROUND_OPS["disable"], 1, 8, arm=True), 0x510, "8201d33d99d008bf"),
    # protocol v2: the bytes were produced by core/ (tests/test_protocol_v2.cpp pins the same ones)
    ("act out example", P.pack_act_out(P.ActOut(1.234, -5.678, 3, True, 2, 5, 2, 1), 9), 0x300, "d204d2e9ab2a092d"),
    ("act out standby zero", P.pack_act_out(P.ActOut(), 0), 0x300, "000000000000000a"),
    ("act out saturating, all ones", P.pack_act_out(P.ActOut(-40.0, 40.0, 7, True, 7, 7, 7, 7), 255), 0x300, "0080ff7fffffff5d"),
    ("heartbeat B", P.pack_heartbeat(1, P.Heartbeat(mode=2, safe_requested=True, role=1, quarantined=True, node_state=(0, 1, 3), reset_count=7,
                                                    release_hash=0xBEEF), 4), 0x401, "02563407efbe0496"),
    ("heartbeat A defaults", P.pack_heartbeat(0, P.Heartbeat(), 0), 0x400, "02000000000000b0"),
    ("state share C", P.pack_state_share(2, P.StateShare((1, 15, 2), 200), 6), 0x412, "f102c800000006f7"),
    ("sim rates", P.pack_sim_rates((1.5, -2.0, 0.125), 3), 0x501, "0c00f0ff0100032c"),
    ("sim accel", P.pack_sim_accel((0.0, 0.5, 1.0), 255), 0x502, "000000040008ffda"),
    ("sim state", P.pack_sim_state(P.SimState(31963.0, 995.4, 14869.0), 17), 0x503, "7c0ce303153a118d"),
    ("sim state saturating", P.pack_sim_state(P.SimState(1.0e9, -5.0, 70000.0), 0), 0x503, "ffff0000ffff00b9"),
    ("sim telemetry", P.pack_sim_telemetry(P.SimTelemetry(31500.0, -0.31, 1.25), 18), 0x504, "4e0ccafee20412b7"),
    ("sim flags", P.pack_sim_flags(P.SimFlags(P.SIM_FLAG_SAFED | P.SIM_FLAG_ENGINE_OUT, 4, 6500), 19), 0x505, "050464190000131e"),
    ("sim flags max", P.pack_sim_flags(P.SimFlags(0xFF, 5, 0xFFFFFFFF), 255), 0x505, "ff05ffffffffff7e"),
]


class ProtocolTests(unittest.TestCase):
    def test_crc8_check_value(self):
        self.assertEqual(P.crc8(b"123456789"), 0x4B)  # same value asserted in the C++ tests

    def test_golden_frames_match_cpp(self):
        for name, frame, can_id, hex_data in GOLDEN:
            with self.subTest(name):
                self.assertEqual(frame.id, can_id)
                self.assertEqual(frame.hex(), hex_data)

    def test_roundtrip(self):
        f = P.pack_gyro(1, (1.0, -2.0, 3.0), 5)
        s = P.unpack_vec3(f, P.GYRO_LSB_DPS)
        self.assertIsNotNone(s)
        self.assertEqual(s.values, (1.0, -2.0, 3.0))
        self.assertEqual(s.seq, 5)
        c = P.unpack_cmd(P.pack_cmd(1, 1.234, -5.678, 0xBEEF, 7))
        self.assertAlmostEqual(c.pitch_deg, 1.234, places=3)
        self.assertAlmostEqual(c.yaw_deg, -5.678, places=3)
        self.assertEqual((c.digest, c.seq), (0xBEEF, 7))

    def test_v2_frames_round_trip(self):
        a = P.unpack_act_out(P.pack_act_out(P.ActOut(1.234, -5.678, 3, True, 2, 5, 2, 1), 9))
        self.assertEqual((a.state, a.held, a.vote_status, a.voted_nodes, a.excluded_nodes, a.cause, a.seq), (3, True, 2, 5, 2, 1, 9))
        self.assertAlmostEqual(a.pitch_deg, 1.234, places=3)
        self.assertAlmostEqual(a.yaw_deg, -5.678, places=3)
        h = P.unpack_heartbeat(P.pack_heartbeat(2, P.Heartbeat(mode=3, bus_alarm=True, role=2, node_state=(3, 2, 1), reset_count=255, release_hash=0x1234), 77))
        self.assertEqual((h.protocol_version, h.mode, h.safe_requested, h.bus_alarm, h.role, h.quarantined), (2, 3, False, True, 2, False))
        self.assertEqual((h.node_state, h.reset_count, h.release_hash, h.seq), ((3, 2, 1), 255, 0x1234, 77))
        s = P.unpack_state_share(P.pack_state_share(0, P.StateShare((15, 0, 7), 9), 1))
        self.assertEqual((s.strikes, s.command_counter), ((15, 0, 7), 9))
        st = P.unpack_sim_state(P.pack_sim_state(P.SimState(31960.0, 995.0, 14869.0), 2))
        self.assertEqual((st.altitude_m, st.speed_ms, st.mass_kg), (31960.0, 995.0, 14869.0))
        t = P.unpack_sim_telemetry(P.pack_sim_telemetry(P.SimTelemetry(31500.0, -0.31, 1.25), 3))
        self.assertEqual(t.dynamic_pressure_pa, 31500.0)
        self.assertAlmostEqual(t.pitch_error_deg, -0.31, places=3)
        self.assertAlmostEqual(t.yaw_error_deg, 1.25, places=3)
        fl = P.unpack_sim_flags(P.pack_sim_flags(P.SimFlags(P.SIM_FLAG_PLATFORM_SATURATED | P.SIM_FLAG_ABORTED, 3, 123456), 4))
        self.assertEqual((fl.flags, fl.engines_on, fl.time_frames), (0x12, 3, 123456))

    def test_v2_fields_are_masked_not_spilled_and_do_not_overlap(self):
        back = P.unpack_act_out(P.pack_act_out(P.ActOut(state=0xFF, vote_status=0xFF, voted_nodes=0xFF), 0))
        self.assertEqual((back.state, back.vote_status, back.voted_nodes, back.excluded_nodes, back.cause, back.held), (7, 7, 7, 0, 0, False))
        for bit in range(16):
            data = bytearray(P.pack_act_out(P.ActOut(), 0).data)
            data[4 + bit // 8] = 1 << (bit % 8)
            data[7] = P.crc8(bytes(data[:7]))
            a = P.unpack_act_out(P.Frame(0x300, bytes(data)))
            set_fields = sum(bool(x) for x in (a.state, a.held, a.vote_status, a.voted_nodes, a.excluded_nodes, a.cause))
            self.assertEqual(set_fields, 1, bit)
        hb = P.unpack_heartbeat(P.pack_heartbeat(0, P.Heartbeat(mode=0xFF, role=0xFF, node_state=(0xFF, 0, 0)), 0))
        self.assertEqual((hb.mode, hb.role, hb.node_state, hb.safe_requested, hb.quarantined), (3, 3, (3, 0, 0), False, False))

    def test_v2_decoders_reject_damage_the_wrong_id_and_a_node_that_does_not_exist(self):
        def corrupt(frame, byte, mask):
            d = bytearray(frame.data)
            d[byte] ^= mask
            return P.Frame(frame.id, bytes(d))
        a = P.pack_act_out(P.ActOut(1.0, 1.0), 1)
        self.assertIsNone(P.unpack_act_out(corrupt(a, 7, 1)))
        self.assertIsNone(P.unpack_act_out(P.Frame(0x400, a.data)))
        self.assertIsNone(P.unpack_heartbeat(P.Frame(0x403, P.pack_heartbeat(0, P.Heartbeat(), 0).data)))
        self.assertIsNone(P.unpack_heartbeat(corrupt(P.pack_heartbeat(1, P.Heartbeat(), 0), 2, 0x10)))
        self.assertIsNone(P.unpack_state_share(P.Frame(0x413, P.pack_state_share(0, P.StateShare(), 0).data)))
        self.assertIsNone(P.unpack_state_share(corrupt(P.pack_state_share(1, P.StateShare(), 0), 0, 1)))
        for pack, unpack, other in ((lambda: P.pack_sim_state(P.SimState(), 0), P.unpack_sim_state, 0x504),
                                    (lambda: P.pack_sim_telemetry(P.SimTelemetry(), 0), P.unpack_sim_telemetry, 0x503),
                                    (lambda: P.pack_sim_flags(P.SimFlags(), 0), P.unpack_sim_flags, 0x504)):
            f = pack()
            self.assertIsNotNone(unpack(f))
            self.assertIsNone(unpack(corrupt(f, 1, 1)))
            self.assertIsNone(unpack(P.Frame(other, f.data)))

    def test_v2_describe_names_every_frame(self):
        cases = [
            (P.pack_act_out(P.ActOut(1.234, -5.678, 3, True, 1, 5, 2, 1), 9), ("ACT", "Safe-ramp", "HELD", "lost-votes")),
            (P.pack_heartbeat(1, P.Heartbeat(mode=2, safe_requested=True, role=1, quarantined=True, node_state=(0, 1, 3), reset_count=7, release_hash=0xBEEF), 4),
             ("HB    B", "SAFE-REQUESTED", "QUARANTINED", "view=HLD", "0xbeef")),
            (P.pack_state_share(2, P.StateShare((1, 15, 2), 200), 6), ("STATE C", "command_counter=200")),
            (P.pack_sim_rates((1.5, -2.0, 0.125), 3), ("SIMRT", "dps")),
            (P.pack_sim_accel((0.0, 0.5, 1.0), 3), ("SIMAC", "g")),
            (P.pack_sim_state(P.SimState(31963.0, 995.0, 14869.0), 1), ("SIMST", "altitude=31960", "mass=14869")),
            (P.pack_sim_telemetry(P.SimTelemetry(31500.0, -0.31, 1.25), 1), ("SIMTL", "q=31500")),
            (P.pack_sim_flags(P.SimFlags(P.SIM_FLAG_SAFED | P.SIM_FLAG_ENGINE_OUT, 4, 6500), 1), ("SIMFL", "t=65.00", "safed,engine-out")),
        ]
        for frame, needles in cases:
            text = P.describe(frame)
            for n in needles:
                self.assertIn(n, text)
            bad = P.Frame(frame.id, frame.data[:7] + bytes([frame.data[7] ^ 1]))
            self.assertIn("CRC-BAD", P.describe(bad))

    def test_v2_quantiser_saturates_and_ignores_nan_and_negatives(self):
        self.assertEqual(P._quantize_u16(float("nan"), 1.0), 0)
        self.assertEqual(P._quantize_u16(-3.0, 1.0), 0)
        self.assertEqual(P._quantize_u16(float("inf"), 1.0), 65535)
        self.assertEqual(P._quantize_u16(float("-inf"), 1.0), 0)
        self.assertEqual(P._quantize_u16(1e12, 10.0), 65535)
        self.assertEqual(P._quantize_u16(14.5, 1.0), 15)  # rounds half up like the C++

    def test_sync_roundtrip_and_rejects_corruption_and_other_ids(self):
        f = P.pack_sync(123456, 77)
        s = P.unpack_sync(f)
        self.assertEqual((s.frame_no, s.seq), (123456, 77))
        self.assertIn("frame=123456", P.describe(f))
        for bit in range(64):
            data = bytearray(f.data)
            data[bit // 8] ^= 1 << (bit % 8)
            self.assertIsNone(P.unpack_sync(P.Frame(f.id, bytes(data))), bit)
        self.assertIsNone(P.unpack_sync(P.Frame(0x100, f.data)))

    def test_ground_roundtrip_describe_and_corruption(self):
        f = P.pack_ground(P.GROUND_OPS["reintegrate"], 1, 77)
        g = P.unpack_ground(f)
        self.assertEqual((g.op, g.node, g.counter, g.arm), (1, 1, 77, False))
        self.assertIn("reintegrate B", P.describe(f))
        for bit in range(64):
            data = bytearray(f.data)
            data[bit // 8] ^= 1 << (bit % 8)
            self.assertIsNone(P.unpack_ground(P.Frame(f.id, bytes(data))), bit)
        self.assertIsNone(P.unpack_ground(P.Frame(0x100, f.data)))
        self.assertIn("CRC-BAD", P.describe(P.Frame(f.id, f.data[:7] + bytes([f.data[7] ^ 1]))))

    def test_siphash_matches_the_published_reference_vectors(self):
        key = bytes(range(16))
        want = {0: "310e0edd47db6f72", 1: "fd67dc93c539f874", 2: "5a4fa9d909806c0d", 15: "e545be4961ca29a1"}
        for n, hexdigest in want.items():
            self.assertEqual(P.siphash24(key, bytes(range(n))).to_bytes(8, "little").hex(), hexdigest, n)

    def test_ground_tag_binds_op_node_counter_arm_and_key(self):
        base = P.pack_ground(2, 1, 7)
        for other in (P.pack_ground(1, 1, 7), P.pack_ground(2, 2, 7), P.pack_ground(2, 1, 8), P.pack_ground(2, 1, 7, arm=True),
                      P.pack_ground(2, 1, 7, key=bytes(range(1, 17)))):
            self.assertNotEqual(base.data[2:6], other.data[2:6])
        self.assertNotEqual(base.data[2:6], P.pack_ground(2, 1, 7, forged=True).data[2:6])
        g = P.unpack_ground(P.pack_ground(2, 1, 7, arm=True))
        self.assertTrue(g.arm and g.op == 2)

    def test_ground_key_can_come_from_the_environment(self):
        import os
        os.environ["TFC_GROUND_KEY"] = "ff" * 16
        try:
            self.assertEqual(P.ground_key(), b"\xff" * 16)
            os.environ["TFC_GROUND_KEY"] = "zz"
            with self.assertRaises(ValueError):
                P.ground_key()
        finally:
            del os.environ["TFC_GROUND_KEY"]
        self.assertEqual(P.ground_key(), P.BENCH_KEY)

    def test_every_single_bit_flip_is_detected(self):
        f = P.pack_cmd(0, 12.345, -6.789, 0x1234, 42)
        for bit in range(64):
            data = bytearray(f.data)
            data[bit // 8] ^= 1 << (bit % 8)
            with self.subTest(bit=bit):
                self.assertFalse(P.check(P.Frame(f.id, bytes(data))))
                self.assertIsNone(P.unpack_cmd(P.Frame(f.id, bytes(data))))

    def test_quantize_saturates_and_rounds_half_away_from_zero(self):
        self.assertEqual(P.quantize(1e30, P.GYRO_LSB_DPS), 32767)
        self.assertEqual(P.quantize(-1e30, P.GYRO_LSB_DPS), -32768)
        self.assertEqual(P.quantize(float("inf"), 1.0), 32767)
        self.assertEqual(P.quantize(float("nan"), 1.0), 0)
        self.assertEqual(P.quantize(0.5, 1.0), 1)
        self.assertEqual(P.quantize(-0.5, 1.0), -1)
        self.assertEqual(P.quantize(0.4, 1.0), 0)

    def test_seq_wraps_at_256(self):
        self.assertTrue(P.seq_is_next(255, 0))
        self.assertTrue(P.seq_is_next(7, 8))
        self.assertFalse(P.seq_is_next(7, 9))
        self.assertFalse(P.seq_is_next(7, 7))

    def test_frame_validation(self):
        with self.assertRaises(ValueError):
            P.Frame(0x800, b"")
        with self.assertRaises(ValueError):
            P.Frame(0x100, b"\x00" * 9)

    def test_describe_flags_bad_crc_and_unknown_ids(self):
        f = P.pack_gyro(0, (1.0, 2.0, 3.0), 1)
        self.assertIn("GYRO", P.describe(f))
        bad = P.Frame(f.id, f.data[:7] + bytes([f.data[7] ^ 1]))
        self.assertIn("CRC-BAD", P.describe(bad))
        self.assertIn("out-of-schedule", P.describe(P.Frame(0x023, b"\x00" * 8)))


if __name__ == "__main__":
    unittest.main()
