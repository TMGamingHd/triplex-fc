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
