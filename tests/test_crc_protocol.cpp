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

TFC_TEST(arbitration_priority_order) {
  CHECK(id::kSync < id::kGyroBase);
  CHECK(id::kGyroBase < id::kCmdBase);
  CHECK(id::kCmdBase < id::kActOut);
  CHECK(id::kActOut < id::kHeartbeat);
}
