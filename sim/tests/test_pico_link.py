# SPDX-License-Identifier: MIT
"""The PC side of the Pico link: golden frames (the same bytes as tests/test_pico.cpp), the parser, and the client with a fake port."""
import unittest

from tfc_peers import pico_link as L


class FakePort:
    def __init__(self, to_read: bytes = b"") -> None:
        self.written = bytearray()
        self.to_read = bytearray(to_read)

    def write(self, data: bytes) -> int:
        self.written += data
        return len(data)

    def read(self, size: int = 1) -> bytes:
        out = bytes(self.to_read[:size])
        del self.to_read[:size]
        return out


class Golden(unittest.TestCase):
    def test_the_frames_are_the_ones_pinned_in_the_cpp_tests(self):
        self.assertEqual(L.pack_platform(7, 1.234, -5.678).hex(), "a50105077b00c8fd45")
        self.assertEqual(L.pack_relay(2, 1500).hex(), "a5020302dc05f4")
        self.assertEqual(L.pack_ping().hex(), "a503006a")
        st = L.Status(9, 12.34, -45.0, L.FLAG_SATURATED | L.FLAG_LEVELLING, 6, 250)
        self.assertEqual(L.pack_status(st).hex(), "a5810909d2046cee0506fa0094")
        self.assertEqual(L.pack_status(L.Status()).hex(), "a58109000000000000000000b8")
        self.assertEqual(L.pack_platform(255, 1.0e9, -1.0e9).hex(), "a50105ffff7f0080b5")

    def test_quantiser_rounds_half_away_from_zero_saturates_and_maps_nan_to_zero(self):
        self.assertEqual(L._quantize_angle(0.005), 1)
        self.assertEqual(L._quantize_angle(-0.005), -1)
        self.assertEqual(L._quantize_angle(float("nan")), 0)
        self.assertEqual(L._quantize_angle(1e9), 32767)
        self.assertEqual(L._quantize_angle(-1e9), -32768)

    def test_status_round_trips(self):
        s = L.Status(4, -1.5, 2.5, L.FLAG_HOLDING | L.FLAG_REJECTED, 0x0F, 12345)
        frames = list(L.Parser().feed(L.pack_status(s)))
        self.assertEqual(len(frames), 1)
        back = L.unpack_status(*frames[0])
        self.assertEqual((back.seq_echo, back.flags, back.relays, back.command_age_ms), (4, L.FLAG_HOLDING | L.FLAG_REJECTED, 0x0F, 12345))
        self.assertAlmostEqual(back.out_x_deg, -1.5)
        self.assertIsNone(L.unpack_status(L.T_PING, b""))
        self.assertIsNone(L.unpack_status(L.T_STATUS, b"\x00" * 8))
        self.assertIn("nodes cut [B, C]", L.Status(relays=0b0110).describe())
        self.assertIn("saturated", L.Status(flags=L.FLAG_SATURATED).describe())


class ParserTests(unittest.TestCase):
    def test_finds_frames_through_garbage_a_bad_crc_a_false_start_and_split_reads(self):
        good = L.pack_relay(1, 100)
        bad = bytearray(good)
        bad[-1] ^= 0x55
        p = L.Parser()
        got = list(p.feed(b"\x00\x13\xff\x42\x99"))
        self.assertEqual((got, p.skipped_bytes), ([], 5))
        got += list(p.feed(good[:3]))  # a frame split across two reads
        got += list(p.feed(good[3:]))
        self.assertEqual(len(got), 1)
        got += list(p.feed(bytes(bad)))
        self.assertEqual((len(got), p.bad_frames), (1, 1))
        got += list(p.feed(bytes([L.SYNC, 0xFF]) + good))  # an 0xA5 whose length is impossible, then a good frame
        self.assertEqual(len(got), 2)
        self.assertEqual(p.bad_frames, 2)
        got += list(p.feed(L.pack_ping() + L.encode(L.T_STATUS, bytes(range(1, 13)))))
        self.assertEqual([g[0] for g in got[2:]], [L.T_PING, L.T_STATUS])
        self.assertEqual(len(got[3][1]), 12)
        self.assertEqual(len(L.encode(L.T_STATUS, bytes(50))), 16)  # clamped to the largest payload


class Client(unittest.TestCase):
    def test_commands_go_out_as_frames_with_a_rolling_sequence_number(self):
        port = FakePort()
        c = L.PicoClient(port)
        c.platform(1.0, -2.0)
        c.platform(3.0, 4.0)
        frames = list(L.Parser().feed(bytes(port.written)))
        self.assertEqual([f[0] for f in frames], [L.T_PLATFORM, L.T_PLATFORM])
        self.assertEqual([f[1][0] for f in frames], [0, 1])
        c.seq = 255
        c.platform(0.0, 0.0)
        c.platform(0.0, 0.0)
        tail = list(L.Parser().feed(bytes(port.written)))[2:]
        self.assertEqual([f[1][0] for f in tail], [255, 0])  # wraps

    def test_cut_and_restore_name_the_nodes_and_never_ask_for_more_than_the_maximum(self):
        port = FakePort()
        c = L.PicoClient(port)
        c.cut("b", 5000)
        c.cut("ACT", 10 ** 6)
        c.restore("A")
        c.restore_all()
        frames = list(L.Parser().feed(bytes(port.written)))
        self.assertEqual(frames[0], (L.T_RELAY, bytes([1]) + (5000).to_bytes(2, "little")))
        self.assertEqual(frames[1], (L.T_RELAY, bytes([3]) + (L.MAX_CUT_MS).to_bytes(2, "little")))
        self.assertEqual(frames[2], (L.T_RELAY, bytes([0, 0, 0])))
        self.assertEqual(len(frames), 3 + 4)
        with self.assertRaises(ValueError):
            c.cut("D", 100)

    def test_poll_and_wait_status_read_the_boards_answer(self):
        s = L.Status(5, 10.0, -10.0, L.FLAG_LEVELLING, 0b0001, 1500)
        port = FakePort(b"\x01\x02" + L.pack_status(s))
        c = L.PicoClient(port)
        got = c.poll()
        self.assertEqual(len(got), 1)
        self.assertEqual(c.last_status.command_age_ms, 1500)
        port2 = FakePort(L.pack_status(s))
        got2 = L.PicoClient(port2).wait_status(0.2)
        self.assertIsNotNone(got2)
        self.assertEqual(bytes(port2.written), L.pack_ping())  # it asked
        self.assertIsNone(L.PicoClient(FakePort()).wait_status(0.03))  # nothing came


if __name__ == "__main__":
    unittest.main()
