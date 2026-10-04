// SPDX-License-Identifier: MIT
// The flight function (core/include/tfc/flight.hpp): the same bits as the stages composed by hand, the same on every replica, masks the sensors
// of an excluded node, and ignores frames that are not sensor frames.
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>

#include "tfc/consensus.hpp"
#include "tfc/controller.hpp"
#include "tfc/estimator.hpp"
#include "tfc/flight.hpp"
#include "tfc_test.hpp"

namespace {

uint32_t bits(float f) {
  uint32_t u = 0U;
  std::memcpy(&u, &f, sizeof u);
  return u;
}

struct Lcg {
  uint32_t s;
  float uniform() {
    s = (s * 1664525U) + 1013904223U;
    return (static_cast<float>(s >> 8) / 8388608.0F) - 1.0F;
  }
};

tfc::GainSchedule gains() {
  tfc::GainSchedule g;
  (void)g.add(0U, tfc::ControllerGains{1.0F, 0.6F, 0.1F});
  (void)g.add(500U, tfc::ControllerGains{1.6F, 0.9F, 0.2F});
  return g;
}

tfc::Guidance guidance() {
  tfc::Guidance g;
  (void)g.add(1U, 0U, 0.0F);
  (void)g.add(1U, 400U, 5.0F);
  return g;
}

// The three nodes' frames for frame k: a platform tilting slowly about Y, each node with its own noise.
std::array<tfc::Frame, 6> frames(uint32_t k, std::array<Lcg, 3>& noise, float gyro_error_node1 = 0.0F) {
  std::array<tfc::Frame, 6> fr{};
  const float tilt = 0.00873F * static_cast<float>(k) * 0.01F;  // 0.5 degree per second, in radians
  for (uint8_t n = 0; n < 3U; ++n) {
    tfc::Vec3 g;
    tfc::Vec3 a;
    g.v[0] = 0.1F * noise[n].uniform();
    g.v[1] = 0.5F + (0.1F * noise[n].uniform()) + (n == 1U ? gyro_error_node1 : 0.0F);
    g.v[2] = 0.1F * noise[n].uniform();
    a.v[0] = std::sin(tilt) + (0.002F * noise[n].uniform());
    a.v[1] = 0.002F * noise[n].uniform();
    a.v[2] = std::cos(tilt) + (0.002F * noise[n].uniform());
    fr[n] = tfc::pack_gyro(n, g, static_cast<uint8_t>(k));
    fr[3U + n] = tfc::pack_accel(n, a, static_cast<uint8_t>(k));
  }
  return fr;
}

}  // namespace

TFC_TEST(flight_function_gives_the_bits_of_the_stages_composed_by_hand) {
  tfc::FlightFunction ff(gains(), guidance());
  tfc::SensorConsensus cons;
  tfc::AttitudeEstimator est;
  tfc::Controller ctl;
  const tfc::GainSchedule gs = gains();
  const tfc::Guidance gd = guidance();
  std::array<Lcg, 3> noise{Lcg{1U}, Lcg{2U}, Lcg{3U}};
  for (uint32_t k = 0; k < 800U; ++k) {
    const std::array<tfc::Frame, 6> fr = frames(k, noise);
    ff.begin_frame(k, 0x07U);
    cons.begin_frame(0x07U);
    for (const tfc::Frame& f : fr) {
      CHECK(ff.on_frame(f));
      (void)cons.on_frame(f);
    }
    est.update(cons.consensus(), 0.01F);
    ctl.set_gains(gs.at(k));
    tfc::Command want = ctl.step(est.attitude(), gd.at(k), 0.01F);
    want.state_digest = static_cast<uint16_t>(est.digest() ^ ctl.digest());
    const tfc::Command got = ff.step();
    CHECK(bits(got.pitch_deg) == bits(want.pitch_deg) && bits(got.yaw_deg) == bits(want.yaw_deg) && got.state_digest == want.state_digest);
    CHECK(ff.frame() == k);
  }
  CHECK(ff.estimator().attitude().valid);
  CHECK(std::fabs(ff.estimator().attitude().tilt_y_deg) > 1.0F);  // it followed the tilt
}

TFC_TEST(flight_function_replicas_fed_the_same_frames_stay_bit_identical) {
  std::array<tfc::FlightFunction, 3> ff{tfc::FlightFunction(gains(), guidance()), tfc::FlightFunction(gains(), guidance()),
                                        tfc::FlightFunction(gains(), guidance())};
  std::array<Lcg, 3> noise{Lcg{7U}, Lcg{8U}, Lcg{9U}};
  for (uint32_t k = 0; k < 1500U; ++k) {
    const std::array<tfc::Frame, 6> fr = frames(k, noise);
    std::array<tfc::Command, 3> c{};
    for (unsigned r = 0; r < 3U; ++r) {
      ff[r].begin_frame(k, 0x07U);
      for (const tfc::Frame& f : fr) {
        (void)ff[r].on_frame(f);
      }
      c[r] = ff[r].step();
    }
    CHECK(bits(c[0].pitch_deg) == bits(c[1].pitch_deg) && bits(c[1].pitch_deg) == bits(c[2].pitch_deg));
    CHECK(c[0].state_digest == c[1].state_digest && c[1].state_digest == c[2].state_digest);
  }
}

TFC_TEST(flight_function_does_not_use_the_sensors_of_a_node_that_is_not_usable) {
  tfc::FlightFunction masked(gains(), guidance());
  tfc::FlightFunction clean(gains(), guidance());
  tfc::FlightFunction unmasked(gains(), guidance());
  std::array<Lcg, 3> n1{Lcg{4U}, Lcg{5U}, Lcg{6U}};
  std::array<Lcg, 3> n2 = n1;
  std::array<Lcg, 3> n3 = n1;
  for (uint32_t k = 0; k < 600U; ++k) {
    const std::array<tfc::Frame, 6> bad = frames(k, n1, 40.0F);  // node B's gyro reads 40 dps too much about Y
    const std::array<tfc::Frame, 6> good = frames(k, n2);
    masked.begin_frame(k, 0x05U);  // B is excluded
    clean.begin_frame(k, 0x05U);
    unmasked.begin_frame(k, 0x07U);
    for (unsigned i = 0; i < 6U; ++i) {
      (void)masked.on_frame(bad[i]);
      (void)clean.on_frame(good[i]);
      (void)unmasked.on_frame(frames(k, n3, 40.0F)[i]);
    }
    const tfc::Command m = masked.step();
    const tfc::Command c = clean.step();
    (void)unmasked.step();
    CHECK(bits(m.pitch_deg) == bits(c.pitch_deg));  // the bad node's data is simply not used
  }
  CHECK(masked.consensus().gyro_nodes == 0x05U);
}

TFC_TEST(flight_function_ignores_frames_that_are_not_sensor_frames) {
  tfc::FlightFunction ff(gains(), guidance());
  ff.begin_frame(0U, 0x07U);
  CHECK(!ff.on_frame(tfc::pack_sync(0U, 0U)));
  CHECK(!ff.on_frame(tfc::pack_cmd(0U, tfc::Command{}, 0U)));
  tfc::Frame corrupt = tfc::pack_gyro(0U, tfc::Vec3{}, 0U);
  corrupt.data[0] = static_cast<uint8_t>(corrupt.data[0] ^ 0xFFU);
  CHECK(!ff.on_frame(corrupt));
  tfc::Frame bad_node = tfc::pack_gyro(0U, tfc::Vec3{}, 0U);
  bad_node.id = tfc::id::kGyroBase + 3U;
  CHECK(!ff.on_frame(bad_node));
  const tfc::Command c = ff.step();  // no data: the controller holds (neutral at the start)
  CHECK(c.pitch_deg == 0.0F && c.yaw_deg == 0.0F);
}

TFC_TEST(the_tables_can_be_read_back_point_by_point) {
  const tfc::Guidance g = guidance();
  CHECK(g.size(1U) == 2U && g.size(0U) == 0U && g.size(2U) == 0U);
  CHECK(g.point(1U, 1U).frame == 400U && g.point(1U, 1U).deg == 5.0F);
  CHECK(g.point(1U, 2U).frame == 0U && g.point(2U, 0U).deg == 0.0F);  // out of range: a zero point
  const tfc::GainSchedule s = gains();
  CHECK(s.frame_at(1U) == 500U && s.gains_at(1U).kp == 1.6F);
  CHECK(s.frame_at(2U) == 0U && s.gains_at(2U).kp == tfc::ControllerGains{}.kp);
}
