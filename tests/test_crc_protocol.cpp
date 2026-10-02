// SPDX-License-Identifier: MIT
#include <cstring>

#include "tfc/protocol.hpp"
#include "tfc_test.hpp"

using namespace tfc;

TFC_TEST(crc8_matches_sae_j1850_check_value) {
  const char* s = "123456789";
  CHECK(crc8(reinterpret_cast<const uint8_t*>(s), std::strlen(s)) == 0x4B);
}

TFC_TEST(gyro_roundtrip_within_quantization) {
  const Vec3 in{{12.5F, -300.125F, 0.0F}};
  const Frame f = pack_gyro(1, in, 42);
  CHECK(f.id == id::kGyroBase + 1);
  const DecodedVec3 d = unpack_vec3(f, kGyroLsbDps);
  CHECK(d.ok);
  CHECK(d.seq == 42);
  for (unsigned i = 0; i < 3; ++i) CHECK(std::fabs(d.x.v[i] - in.v[i]) <= kGyroLsbDps * 0.5F + 1e-6F);
}

TFC_TEST(accel_saturates_instead_of_wrapping) {
  const Vec3 in{{100.0F, -100.0F, 1.0F}};  // far beyond +-16 g
  const DecodedVec3 d = unpack_vec3(pack_accel(0, in, 0), kAccelLsbG);
  CHECK(d.ok);
  CHECK(d.x.v[0] > 15.9F && d.x.v[0] < 16.1F);
  CHECK(d.x.v[1] < -15.9F && d.x.v[1] > -16.1F);
}

TFC_TEST(any_single_bit_flip_is_detected) {
  const Frame good = pack_gyro(2, Vec3{{1.0F, 2.0F, 3.0F}}, 7);
  for (unsigned byte = 0; byte < 8; ++byte) {
    for (unsigned bit = 0; bit < 8; ++bit) {
      Frame bad = good;
      bad.data[byte] = static_cast<uint8_t>(bad.data[byte] ^ (1U << bit));
      CHECK(!unpack_vec3(bad, kGyroLsbDps).ok);
    }
  }
}

TFC_TEST(wrong_length_is_rejected) {
  Frame f = pack_gyro(0, Vec3{}, 0);
  f.len = 7;
  CHECK(!unpack_vec3(f, kGyroLsbDps).ok);
}

TFC_TEST(command_roundtrip_and_digest) {
  const Command c{-12.345F, 7.001F, 0xBEEF};
  const DecodedCommand d = unpack_cmd(pack_cmd(2, c, 200));
  CHECK(d.ok);
  CHECK(d.seq == 200);
  CHECK(d.cmd.state_digest == 0xBEEF);
  CHECK(std::fabs(d.cmd.pitch_deg - c.pitch_deg) < 0.001F);
  CHECK(std::fabs(d.cmd.yaw_deg - c.yaw_deg) < 0.001F);
}

TFC_TEST(sequence_wraps_at_256) {
  CHECK(seq_is_next(255, 0));
  CHECK(seq_is_next(0, 1));
  CHECK(!seq_is_next(0, 2));
}

TFC_TEST(phase_a_frame_with_the_current_number_is_on_time_once) {
  PhaseTracker t;
  t.next_frame();
  CHECK(t.classify(10, 10) == FrameTiming::OnTime);
  CHECK(t.classify(10, 10) == FrameTiming::Bad);  // a second frame with the same number: a duplicate or a replay
  t.next_frame();
  CHECK(t.classify(11, 11) == FrameTiming::OnTime);
}

TFC_TEST(phase_a_frame_of_an_earlier_cycle_that_never_arrived_is_a_late_frame_not_an_error) {
  PhaseTracker t;
  for (uint8_t k = 0; k < 10; ++k) {
    t.next_frame();
    CHECK(t.classify(k, k) == FrameTiming::OnTime);
  }
  t.next_frame();                                          // cycle 10: nothing arrives on time
  t.next_frame();                                          // cycle 11: frame 10 arrives now, and so does 11
  CHECK(t.classify(10, 11) == FrameTiming::Late);          // it missed the vote of its own cycle: one lost sample, nothing more
  CHECK(t.classify(11, 11) == FrameTiming::OnTime);
  CHECK(t.classify(10, 11) == FrameTiming::Bad);           // but only once
}

TFC_TEST(phase_a_stalled_sender_flushing_several_late_frames_is_not_an_error) {
  PhaseTracker t;
  t.next_frame();
  CHECK(t.classify(0, 0) == FrameTiming::OnTime);
  for (int i = 0; i < 3; ++i) t.next_frame();              // cycles 1, 2, 3 empty
  CHECK(t.classify(1, 3) == FrameTiming::Late);            // the sender wakes up and flushes them in order
  CHECK(t.classify(2, 3) == FrameTiming::Late);
  CHECK(t.classify(3, 3) == FrameTiming::OnTime);
  CHECK(t.classify(2, 3) == FrameTiming::Bad);             // each number once
}

TFC_TEST(phase_a_stream_that_is_permanently_one_number_behind_is_bad_after_its_first_frame) {
  // Every cycle delivers one frame, labelled for the cycle before. The first is accepted as a late frame (its slot was empty);
  // after that no slot is empty, so a frame carrying an earlier number is not a late frame, it is a wrong number.
  PhaseTracker t;
  t.next_frame();                                         // cycle 0: nothing arrives
  t.next_frame();                                         // cycle 1: the frame numbered 0
  CHECK(t.classify(0, 1) == FrameTiming::Late);
  for (uint8_t k = 2; k < 20; ++k) {
    t.next_frame();
    CHECK(t.classify(static_cast<uint8_t>(k - 1U), k) == FrameTiming::Bad);
  }
}

TFC_TEST(phase_a_damaged_frame_fills_its_slot_so_a_stale_copy_later_is_not_a_late_frame) {
  PhaseTracker t;
  t.next_frame();
  t.note_damaged();                                       // cycle 0: a frame arrived, damaged
  t.next_frame();
  CHECK(t.classify(0, 1) == FrameTiming::Bad);            // its number turning up a cycle later is a repeat, not a late frame
}

TFC_TEST(phase_a_stream_that_is_a_whole_frame_early_is_bad_every_time) {
  // The frames of cycle k + 1 arrive during cycle k: the numbers are contiguous, only the phase is wrong. A counter that
  // merely counted frames could never see this (campaign E11).
  PhaseTracker t;
  for (uint8_t k = 0; k < 40; ++k) {
    t.next_frame();
    CHECK(t.classify(static_cast<uint8_t>(k + 1U), k) == FrameTiming::Bad);
  }
}

TFC_TEST(phase_numbers_wrap_at_256_and_far_numbers_are_bad) {
  PhaseTracker t;
  uint32_t k = 250U;
  for (int i = 0; i < 20; ++i, ++k) {
    t.next_frame();
    CHECK(t.classify(static_cast<uint8_t>(k & 0xFFU), k) == FrameTiming::OnTime);  // across the wrap 255 -> 0
  }
  t.next_frame();
  CHECK(t.classify(static_cast<uint8_t>((k - 32U) & 0xFFU), k) == FrameTiming::Bad);  // too old to be a late frame
  CHECK(t.classify(static_cast<uint8_t>((k + 100U) & 0xFFU), k) == FrameTiming::Bad);  // nonsense
  CHECK(t.classify(static_cast<uint8_t>((k - 31U) & 0xFFU), k) == FrameTiming::Late);  // the oldest a late frame can be
}

TFC_TEST(phase_a_node_that_restarts_or_joins_late_is_in_phase_at_once) {
  PhaseTracker t;
  for (uint8_t i = 0; i < 5; ++i) t.next_frame();
  t.next_frame();
  CHECK(t.classify(137, 137) == FrameTiming::OnTime);  // it took the number from SYNC: no "sequence break" on a restart
}

TFC_TEST(siphash24_matches_the_published_reference_vectors) {
  AuthKey key{};
  for (unsigned i = 0; i < 16U; ++i) key[i] = static_cast<uint8_t>(i);
  CHECK(key == kBenchKey);
  std::array<uint8_t, 16> in{};
  for (unsigned i = 0; i < 16U; ++i) in[i] = static_cast<uint8_t>(i);
  CHECK(siphash24(key, in.data(), 0) == 0x726fdb47dd0e0e31ULL);   // vectors[0] of the reference implementation
  CHECK(siphash24(key, in.data(), 1) == 0x74f839c593dc67fdULL);
  CHECK(siphash24(key, in.data(), 2) == 0x0d6c8009d9a94f5aULL);
  CHECK(siphash24(key, in.data(), 15) == 0xa129ca6149be45e5ULL);  // crosses a word boundary with a tail
  CHECK(siphash24(key, in.data(), 8) != siphash24(key, in.data(), 9));
}

TFC_TEST(ground_tag_binds_the_key_the_operation_the_node_the_counter_and_the_arm_flag) {
  const AuthKey other = {{1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16}};
  const Frame f = pack_ground_auth(GroundOp::Disable, 1, 7, kBenchKey);
  CHECK(ground_authentic(f, kBenchKey) && !ground_authentic(f, other));
  const DecodedGround d = unpack_ground(f);
  CHECK(d.ok && d.op == 2U && !d.arm && d.node == 1U && d.counter == 7U && d.tag != 0U);
  for (unsigned byte = 0; byte < 7U; ++byte) {  // any change to a covered byte invalidates the tag (and the CRC is re-sealed to hide it)
    Frame g = f;
    g.data[byte] = static_cast<uint8_t>(g.data[byte] ^ 1U);
    g.data[7] = crc8(g.data.data(), 7);
    CHECK(!ground_authentic(g, kBenchKey));
  }
  const Frame arm = pack_ground_auth(GroundOp::Disable, 1, 7, kBenchKey, true);
  CHECK(unpack_ground(arm).arm && unpack_ground(arm).op == 2U && (arm.data[0] & kArmFlag) != 0U);
  CHECK(arm.data[2] != f.data[2] || arm.data[3] != f.data[3]);  // the arm flag changes the tag
  CHECK(!ground_authentic(pack_ground(GroundOp::Disable, 1, 7), kBenchKey));  // a frame with no tag is not authentic
  Frame wrong_id = f;
  wrong_id.id = id::kSim;
  CHECK(!unpack_ground(wrong_id).ok);
}

TFC_TEST(sync_roundtrip_and_layout) {
  const Frame f = pack_sync(0x01020304U, 9);
  CHECK(f.id == id::kSync);
  CHECK(f.data[0] == 0x04 && f.data[1] == 0x03 && f.data[2] == 0x02 && f.data[3] == 0x01);
  CHECK(f.data[4] == 0 && f.data[5] == 0);  // reserved
  const DecodedSync d = unpack_sync(f);
  CHECK(d.ok);
  CHECK(d.frame_no == 0x01020304U);
  CHECK(d.seq == 9);
}

TFC_TEST(sync_any_single_bit_flip_is_detected) {
  const Frame f = pack_sync(123456U, 77);
  for (unsigned bit = 0; bit < 64; ++bit) {
    Frame g = f;
    g.data[bit / 8U] = static_cast<uint8_t>(g.data[bit / 8U] ^ (1U << (bit % 8U)));
    CHECK(!unpack_sync(g).ok);
  }
}

TFC_TEST(arbitration_priority_order) {
  CHECK(id::kSync < id::kGyroBase);
  CHECK(id::kGyroBase < id::kCmdBase);
  CHECK(id::kCmdBase < id::kActOut);
  CHECK(id::kActOut < id::kHeartbeat);
}
