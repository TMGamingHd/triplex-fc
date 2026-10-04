// SPDX-License-Identifier: MIT
// The control loop: sensor consensus, attitude estimator, controller and guidance, each on its own, and then closed around a
// model of the vehicle and its platform (tools/vehicle). The closed-loop tests run three replicas through the real frame
// protocol (CRC, quantisation) and check that they stay bit-identical, that the vehicle is held on its pitch program through a
// gust, an engine-out and a failed sensor, and that the loop is stable.
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>

#include "tfc/consensus.hpp"
#include "tfc/controller.hpp"
#include "tfc/estimator.hpp"
#include "tfc/voter.hpp"
#include "tfc_test.hpp"
#include "vehicle_model.hpp"

using namespace tfc;

namespace {

bool near(float a, float b, float tol) { return std::fabs(a - b) <= tol; }

// The accelerometer reading of a platform tilted by (tx, ty) degrees, in g.
Vec3 gravity_for(float tx_deg, float ty_deg) {
  const float tx = tx_deg * kDegToRad;
  const float ty = ty_deg * kDegToRad;
  return Vec3{{-std::sin(ty), std::cos(ty) * std::sin(tx), std::cos(ty) * std::cos(tx)}};
}

ConsensusInput input(const Vec3& gyro, const Vec3& accel) {
  ConsensusInput in;
  in.gyro_dps = gyro;
  in.accel_g = accel;
  in.gyro_ok = true;
  in.accel_ok = true;
  return in;
}

// Deterministic noise (the same LCG as the simulated IMU).
class Noise {
 public:
  explicit Noise(uint32_t seed) : s_(seed) {}
  float uniform() {
    s_ = s_ * 1664525U + 1013904223U;
    return static_cast<float>(s_ >> 8) / 8388608.0F - 1.0F;
  }

 private:
  uint32_t s_;
};

uint64_t bits64(double d) {
  uint64_t u = 0U;
  std::memcpy(&u, &d, sizeof u);
  return u;
}

uint32_t bits(float f) {
  uint32_t u = 0U;
  std::memcpy(&u, &f, sizeof u);
  return u;
}

}  // namespace

// ---- the arithmetic the estimator is built on ----

TFC_TEST(atan2_approx_is_within_1e5_radians_everywhere_on_the_circle) {
  float worst = 0.0F;
  for (int i = 0; i < 3600; ++i) {
    const float t = (static_cast<float>(i) * 0.1F - 180.0F) * kDegToRad;
    const float y = std::sin(t);
    const float x = std::cos(t);
    const float err = std::fabs(det::atan2_approx(y, x) - std::atan2(y, x));
    // at +-pi the two branches differ by 2 pi: compare the angle modulo that
    const float wrapped = err > 6.0F ? std::fabs(err - 2.0F * kPiF) : err;
    worst = wrapped > worst ? wrapped : worst;
  }
  CHECK(worst < 2.0e-5F);
  CHECK(det::atan2_approx(0.0F, 0.0F) == 0.0F);
  CHECK(near(det::atan2_approx(1.0F, 0.0F), 0.5F * kPiF, 1e-5F));
  CHECK(near(det::atan2_approx(-1.0F, 0.0F), -0.5F * kPiF, 1e-5F));
  CHECK(near(det::atan2_approx(0.0F, -1.0F), kPiF, 1e-5F));
}

// ---- sensor consensus ----

TFC_TEST(consensus_takes_the_median_and_ignores_one_wild_sample) {
  SensorConsensus c;
  c.begin_frame(0x07U);
  const std::array<float, 3> gx{1.0F, 1.2F, 50.0F};  // node 2's gyro is wild
  for (uint8_t n = 0; n < 3U; ++n) {
    CHECK(c.on_frame(pack_gyro(n, Vec3{{gx[n], 0.0F, 0.0F}}, 1U)));
    CHECK(c.on_frame(pack_accel(n, Vec3{{0.0F, 0.0F, 1.0F}}, 1U)));
  }
  const ConsensusInput in = c.consensus();
  CHECK(in.gyro_ok && in.accel_ok);
  CHECK(near(in.gyro_dps.v[0], 1.2F, 0.13F));
  CHECK(in.gyro_nodes == 0x07U && in.accel_nodes == 0x07U);
}

TFC_TEST(consensus_uses_only_the_nodes_the_manager_allows) {
  SensorConsensus c;
  c.begin_frame(0x05U);  // node 1 is latched out
  for (uint8_t n = 0; n < 3U; ++n) {
    CHECK(c.on_frame(pack_gyro(n, Vec3{{n == 1U ? 99.0F : 2.0F, 0.0F, 0.0F}}, 1U)));
    CHECK(c.on_frame(pack_accel(n, Vec3{{0.0F, 0.0F, 1.0F}}, 1U)));
  }
  const ConsensusInput in = c.consensus();
  CHECK(in.gyro_ok && near(in.gyro_dps.v[0], 2.0F, 0.13F));
  CHECK(in.gyro_nodes == 0x05U);
}

TFC_TEST(consensus_with_one_node_is_a_value_and_with_two_that_disagree_is_not) {
  SensorConsensus c;
  c.begin_frame(0x01U);
  CHECK(c.on_frame(pack_gyro(0, Vec3{{3.0F, 0.0F, 0.0F}}, 1U)));
  CHECK(c.on_frame(pack_accel(0, Vec3{{0.0F, 0.0F, 1.0F}}, 1U)));
  ConsensusInput in = c.consensus();
  CHECK(in.gyro_ok && in.accel_ok);  // simplex: nothing to cross-check, but a value
  c.begin_frame(0x03U);
  CHECK(c.on_frame(pack_gyro(0, Vec3{{3.0F, 0.0F, 0.0F}}, 1U)));
  CHECK(c.on_frame(pack_gyro(1, Vec3{{9.0F, 0.0F, 0.0F}}, 1U)));
  CHECK(c.on_frame(pack_accel(0, Vec3{{0.0F, 0.0F, 1.0F}}, 1U)));
  CHECK(c.on_frame(pack_accel(1, Vec3{{0.0F, 0.0F, 1.0F}}, 1U)));
  in = c.consensus();
  CHECK(!in.gyro_ok);  // duplex miscompare: nobody can say which is right, so it is not used
  CHECK(in.accel_ok);
  CHECK(in.gyro_dps.v[0] == 0.0F);
}

TFC_TEST(consensus_is_not_a_value_with_no_data_or_three_that_disagree) {
  SensorConsensus c;
  c.begin_frame(0x07U);
  ConsensusInput in = c.consensus();
  CHECK(!in.gyro_ok && !in.accel_ok && in.gyro_nodes == 0U);
  const std::array<float, 3> gx{0.0F, 10.0F, 20.0F};
  for (uint8_t n = 0; n < 3U; ++n) {
    CHECK(c.on_frame(pack_gyro(n, Vec3{{gx[n], 0.0F, 0.0F}}, 1U)));
  }
  in = c.consensus();
  CHECK(!in.gyro_ok);  // three valid, no two agree
}

TFC_TEST(consensus_ignores_what_is_not_a_good_sensor_frame) {
  SensorConsensus c;
  c.begin_frame(0x07U);
  Frame bad = pack_gyro(0, Vec3{{1.0F, 0.0F, 0.0F}}, 1U);
  bad.data[7] ^= 0x55U;  // damaged
  CHECK(!c.on_frame(bad));
  Frame cmd = pack_cmd(0, Command{1.0F, 2.0F, 7U}, 1U);
  CHECK(!c.on_frame(cmd));  // not a sensor frame
  Frame far = pack_gyro(0, Vec3{{1.0F, 0.0F, 0.0F}}, 1U);
  far.id = id::kGyroBase + 3U;  // a node that does not exist
  CHECK(!c.on_frame(far));
  CHECK(!c.consensus().gyro_ok);
  c.begin_frame(0x07U);  // a new frame forgets the old samples
  CHECK(c.on_frame(pack_gyro(0, Vec3{{1.0F, 0.0F, 0.0F}}, 2U)));
  c.begin_frame(0x07U);
  CHECK(c.consensus().gyro_nodes == 0U);
}

// ---- attitude estimator ----

TFC_TEST(estimator_finds_a_static_tilt_from_gravity) {
  AttitudeEstimator est;
  const ConsensusInput in = input(Vec3{}, gravity_for(10.0F, -5.0F));
  for (int i = 0; i < 400; ++i) {
    est.update(in, 0.01F);
  }
  const Attitude a = est.attitude();
  CHECK(a.valid);
  CHECK(near(a.tilt_x_deg, 10.0F, 0.2F));
  CHECK(near(a.tilt_y_deg, -5.0F, 0.2F));
  CHECK(near(a.rate_x_dps, 0.0F, 0.2F) && near(a.rate_y_dps, 0.0F, 0.2F));
}

TFC_TEST(estimator_aligns_from_the_first_good_gravity_reading) {
  AttitudeEstimator est;
  est.update(input(Vec3{}, gravity_for(-12.0F, 7.0F)), 0.01F);  // one step: no waiting for a filter to converge
  CHECK(near(est.attitude().tilt_x_deg, -12.0F, 0.05F));
  CHECK(near(est.attitude().tilt_y_deg, 7.0F, 0.05F));
  CHECK(near(est.bias_x_dps(), 0.0F, 1e-4F));  // and nothing for the bias integrator to learn from
}

TFC_TEST(estimator_waits_for_a_trustworthy_reading_before_aligning) {
  AttitudeEstimator est;
  est.update(input(Vec3{}, Vec3{{0.0F, 0.0F, 1.7F}}), 0.01F);   // acceleration, not gravity
  est.update(input(Vec3{}, Vec3{{0.0F, 0.0F, 0.5F}}), 0.01F);   // too weak to be gravity
  est.update(input(Vec3{}, Vec3{{0.0F, 0.0F, -1.0F}}), 0.01F);  // upside down: no shortest rotation from level that is useful here
  CHECK(near(est.attitude().tilt_x_deg, 0.0F, 1e-3F) && near(est.attitude().tilt_y_deg, 0.0F, 1e-3F));
  est.update(input(Vec3{}, gravity_for(5.0F, 5.0F)), 0.01F);
  CHECK(near(est.attitude().tilt_x_deg, 5.0F, 0.1F) && near(est.attitude().tilt_y_deg, 5.0F, 0.1F));
}

TFC_TEST(estimator_integrates_the_gyro_when_the_accelerometer_is_not_trusted) {
  AttitudeEstimator est;
  ConsensusInput in = input(Vec3{{10.0F, 0.0F, 0.0F}}, gravity_for(0.0F, 0.0F));
  in.accel_ok = false;
  for (int i = 0; i < 100; ++i) {
    est.update(in, 0.01F);
  }
  CHECK(near(est.attitude().tilt_x_deg, 10.0F, 0.2F));  // 10 dps for one second
  CHECK(est.accel_skips() == 100U && est.steps() == 100U);
  CHECK(near(est.attitude().rate_x_dps, 10.0F, 1e-3F));
}

TFC_TEST(estimator_tracks_a_rotation_about_y_with_the_gravity_correction_on) {
  AttitudeEstimator est;
  float ty = 0.0F;
  for (int i = 0; i < 300; ++i) {
    ty += 10.0F * 0.01F;  // 10 dps about Y for three seconds
    est.update(input(Vec3{{0.0F, 10.0F, 0.0F}}, gravity_for(0.0F, ty)), 0.01F);
  }
  CHECK(near(est.attitude().tilt_y_deg, 30.0F, 0.3F));
  CHECK(near(est.attitude().tilt_x_deg, 0.0F, 0.3F));
  CHECK(near(est.attitude().rate_y_dps, 10.0F, 0.2F));
}

TFC_TEST(estimator_learns_a_constant_gyro_bias) {
  AttitudeEstimator est;
  const ConsensusInput in = input(Vec3{{1.0F, -0.5F, 0.3F}}, gravity_for(0.0F, 0.0F));  // sitting still, gyros read a bias
  for (int i = 0; i < 20000; ++i) {
    est.update(in, 0.01F);
  }
  CHECK(near(est.bias_x_dps(), 1.0F, 0.05F));
  CHECK(near(est.bias_y_dps(), -0.5F, 0.05F));
  CHECK(near(est.attitude().tilt_x_deg, 0.0F, 0.1F) && near(est.attitude().tilt_y_deg, 0.0F, 0.1F));
  CHECK(near(est.attitude().rate_x_dps, 0.0F, 0.05F));
}

TFC_TEST(estimator_limits_the_bias_it_will_believe) {
  EstimatorConfig c;
  c.bias_limit_dps = 2.0F;
  AttitudeEstimator est(c);
  const ConsensusInput hi = input(Vec3{{20.0F, 0.0F, 0.0F}}, gravity_for(0.0F, 0.0F));
  for (int i = 0; i < 6000; ++i) {
    est.update(hi, 0.01F);
  }
  CHECK(est.bias_x_dps() <= 2.0F + 1e-3F && est.bias_x_dps() > 1.9F);
  AttitudeEstimator neg(c);
  const ConsensusInput lo = input(Vec3{{-20.0F, 0.0F, 0.0F}}, gravity_for(0.0F, 0.0F));
  for (int i = 0; i < 6000; ++i) {
    neg.update(lo, 0.01F);
  }
  CHECK(neg.bias_x_dps() >= -2.0F - 1e-3F && neg.bias_x_dps() < -1.9F);
}

TFC_TEST(estimator_ignores_an_accelerometer_outside_the_gate_and_one_that_is_not_a_number) {
  AttitudeEstimator est;
  const ConsensusInput level = input(Vec3{}, gravity_for(0.0F, 0.0F));
  for (int i = 0; i < 50; ++i) {
    est.update(level, 0.01F);
  }
  const uint32_t before = est.accel_skips();
  ConsensusInput thrust = input(Vec3{}, Vec3{{0.0F, 0.0F, 1.6F}});  // 1.6 g: acceleration, not gravity
  est.update(thrust, 0.01F);
  CHECK(est.accel_skips() == before + 1U);
  thrust.accel_g.v[0] = std::numeric_limits<float>::quiet_NaN();
  est.update(thrust, 0.01F);
  CHECK(est.accel_skips() == before + 2U);
  ConsensusInput weak = input(Vec3{}, Vec3{{0.0F, 0.0F, 0.5F}});
  est.update(weak, 0.01F);
  CHECK(est.accel_skips() == before + 3U);
  CHECK(near(est.attitude().tilt_x_deg, 0.0F, 0.01F));
}

TFC_TEST(estimator_holds_the_last_rates_without_a_trustworthy_gyro_and_says_so) {
  AttitudeEstimator est;
  est.update(input(Vec3{{5.0F, 0.0F, 0.0F}}, gravity_for(0.0F, 0.0F)), 0.01F);
  ConsensusInput bad = input(Vec3{}, gravity_for(0.0F, 0.0F));
  bad.gyro_ok = false;
  est.update(bad, 0.01F);
  CHECK(est.gyro_holds() == 1U);
  CHECK(!est.attitude().valid);
  CHECK(near(est.attitude().rate_x_dps, 5.0F, 0.5F));  // the held rate, not zero
}

TFC_TEST(estimator_digest_is_the_same_for_the_same_updates_and_differs_otherwise) {
  AttitudeEstimator a;
  AttitudeEstimator b;
  AttitudeEstimator c;
  for (int i = 0; i < 100; ++i) {
    const ConsensusInput in = input(Vec3{{1.0F, 2.0F, 0.0F}}, gravity_for(1.0F, 1.0F));
    a.update(in, 0.01F);
    b.update(in, 0.01F);
    c.update(in, 0.01F);
  }
  c.update(input(Vec3{{40.0F, 0.0F, 0.0F}}, gravity_for(1.0F, 1.0F)), 0.01F);  // one more, different update
  CHECK(a.digest() == b.digest());
  CHECK(a.digest() != c.digest());
}

TFC_TEST(estimator_treats_a_non_finite_gyro_as_untrustworthy_and_its_state_stays_finite) {
  AttitudeEstimator est;
  est.update(input(Vec3{{2.0F, 0.0F, 0.0F}}, gravity_for(3.0F, 0.0F)), 0.01F);
  const float inf = std::numeric_limits<float>::infinity();
  const float nan = std::numeric_limits<float>::quiet_NaN();
  for (const Vec3& bad : {Vec3{{inf, 0.0F, 0.0F}}, Vec3{{0.0F, nan, 0.0F}}, Vec3{{0.0F, 0.0F, -inf}}}) {
    est.update(input(bad, gravity_for(3.0F, 0.0F)), 0.01F);  // the consensus said "ok" but the values are not numbers
    const Attitude a = est.attitude();
    CHECK(!a.valid);
    CHECK(std::isfinite(a.tilt_x_deg) && std::isfinite(a.tilt_y_deg) && std::isfinite(a.rate_x_dps) && std::isfinite(a.rate_y_dps));
  }
  CHECK(est.gyro_holds() == 3U);
  CHECK(near(est.attitude().tilt_x_deg, 3.0F, 0.2F));
  est.update(input(Vec3{}, gravity_for(3.0F, 0.0F)), 0.01F);
  CHECK(est.attitude().valid);  // and it recovers with the next good sample
}

// ---- guidance ----

TFC_TEST(guidance_interpolates_holds_at_the_ends_and_gives_the_slope) {
  Guidance g;
  CHECK(g.at(10U).tilt_y_deg == 0.0F);  // nothing in the table
  CHECK(g.add(1, 100U, 0.0F));
  CHECK(g.add(1, 300U, -20.0F));
  CHECK(g.add(0, 0U, 2.0F));
  Reference r = g.at(50U);
  CHECK(near(r.tilt_y_deg, 0.0F, 1e-6F) && near(r.tilt_x_deg, 2.0F, 1e-6F));  // before the first point: held
  r = g.at(200U);
  CHECK(near(r.tilt_y_deg, -10.0F, 1e-4F));
  CHECK(near(r.rate_y_dps, -10.0F, 1e-3F));  // -20 degrees in 2 s
  r = g.at(500U);
  CHECK(near(r.tilt_y_deg, -20.0F, 1e-6F) && r.rate_y_dps == 0.0F);  // after the last: held
  CHECK(near(g.at(100U).tilt_y_deg, 0.0F, 1e-6F));
  CHECK(near(g.at(300U).tilt_y_deg, -20.0F, 1e-4F));
}

TFC_TEST(guidance_refuses_a_bad_table) {
  Guidance g;
  CHECK(!g.add(2, 10U, 1.0F));  // no such plane
  CHECK(g.add(0, 10U, 1.0F));
  CHECK(!g.add(0, 10U, 2.0F));  // frames must increase
  CHECK(!g.add(0, 5U, 2.0F));
  for (unsigned i = 1; i < Guidance::kMaxPoints; ++i) {
    CHECK(g.add(0, 10U + i, 1.0F));
  }
  CHECK(!g.add(0, 100U, 1.0F));  // full
}

// ---- controller ----

namespace {
Attitude attitude(float tx, float ty, float rx = 0.0F, float ry = 0.0F) {
  Attitude a;
  a.tilt_x_deg = tx;
  a.tilt_y_deg = ty;
  a.rate_x_dps = rx;
  a.rate_y_dps = ry;
  a.valid = true;
  return a;
}
}  // namespace

TFC_TEST(controller_pushes_against_an_error_in_each_plane_with_the_right_sign) {
  Controller c;
  Command cmd{};
  for (int i = 0; i < 20; ++i) {
    cmd = c.step(attitude(-1.0F, 2.0F), Reference{}, 0.01F);  // pitch plane too high, yaw plane too low
  }
  CHECK(cmd.pitch_deg < 0.0F && cmd.yaw_deg > 0.0F);
  CHECK(near(cmd.pitch_deg, -2.2F * 2.0F, 0.8F));  // about kp * error (the integrator adds a little)
}

TFC_TEST(controller_damps_with_the_rate) {
  Controller c;
  const Command still = c.step(attitude(0.0F, 0.0F, 0.0F, 0.0F), Reference{}, 0.01F);
  Controller d;
  const Command moving = d.step(attitude(0.0F, 0.0F, 0.0F, 3.0F), Reference{}, 0.01F);  // pitching up at 3 dps
  CHECK(still.pitch_deg == 0.0F);
  CHECK(moving.pitch_deg < 0.0F);
}

TFC_TEST(controller_never_exceeds_the_gimbal_limit_or_the_slew_limit) {
  Controller c;
  float last = 0.0F;
  for (int i = 0; i < 200; ++i) {
    const Command cmd = c.step(attitude(0.0F, -30.0F), Reference{}, 0.01F);  // a huge error
    CHECK(std::fabs(cmd.pitch_deg) <= 8.0F + 1e-5F);
    CHECK(std::fabs(cmd.pitch_deg - last) <= 0.6F + 1e-5F);
    last = cmd.pitch_deg;
  }
  CHECK(near(last, 8.0F, 1e-4F));
  CHECK(c.saturated_frames() > 0U);
  Controller n;
  for (int i = 0; i < 200; ++i) {
    (void)n.step(attitude(0.0F, 30.0F), Reference{}, 0.01F);
  }
  CHECK(near(n.output_pitch_deg(), -8.0F, 1e-4F));
}

TFC_TEST(controller_integrator_does_not_wind_up_while_saturated) {
  Controller c;
  for (int i = 0; i < 2000; ++i) {
    (void)c.step(attitude(0.0F, -30.0F), Reference{}, 0.01F);
  }
  // the error disappears: with a wound-up integrator the command would stay pinned for a long time
  float cmd = 0.0F;
  for (int i = 0; i < 100; ++i) {
    cmd = c.step(attitude(0.0F, 0.0F), Reference{}, 0.01F).pitch_deg;
  }
  CHECK(std::fabs(cmd) < 4.0F);
}

TFC_TEST(controller_integrates_when_saturation_comes_from_the_rate_term_against_the_angle_error) {
  Controller c;
  Reference r;
  r.rate_y_dps = 20.0F;  // a demanded rate far above the actual: the derivative term saturates the command upwards
  r.rate_x_dps = -20.0F;
  for (int i = 0; i < 100; ++i) {
    (void)c.step(attitude(-0.1F, 0.1F), r, 0.01F);  // the angle errors are small and of the opposite sign to the saturation
  }
  CHECK(c.saturated_frames() == 0U);  // integrating is allowed: the angle error does not push further into the limit
  CHECK(near(c.output_pitch_deg(), 8.0F, 1e-4F) && near(c.output_yaw_deg(), -8.0F, 1e-4F));
}

TFC_TEST(controller_integrator_removes_a_steady_error) {
  Controller c;
  Command cmd{};
  for (int i = 0; i < 300; ++i) {
    cmd = c.step(attitude(0.0F, 0.5F), Reference{}, 0.01F);  // a persistent half degree, no rate
  }
  const float early = cmd.pitch_deg;
  for (int i = 0; i < 1500; ++i) {
    cmd = c.step(attitude(0.0F, 0.5F), Reference{}, 0.01F);
  }
  CHECK(cmd.pitch_deg < early);  // the integrator keeps pushing
  CHECK(cmd.pitch_deg > -8.0F);
}

TFC_TEST(controller_holds_the_last_command_when_the_estimate_is_not_trustworthy) {
  Controller c;
  Command cmd = c.step(attitude(0.0F, 3.0F), Reference{}, 0.01F);
  Attitude bad = attitude(0.0F, -20.0F);
  bad.valid = false;
  const Command held = c.step(bad, Reference{}, 0.01F);
  CHECK(held.pitch_deg == cmd.pitch_deg && held.yaw_deg == cmd.yaw_deg);
  CHECK(c.holds() == 1U);
  const uint16_t d = c.digest();
  (void)c.step(bad, Reference{}, 0.01F);
  CHECK(c.digest() == d);  // nothing moved
  cmd = c.step(attitude(0.0F, 3.0F), Reference{}, 0.01F);
  CHECK(c.digest() != d);
  CHECK(c.output_yaw_deg() == cmd.yaw_deg);
}

// ---- the loop, closed around the vehicle ----

namespace {

struct Fault {
  bool active = false;
  unsigned node = 0U;
  float gyro_bias_dps = 0.0F;
  bool stuck = false;
};

// Three replicas of the loop, each with its own sensor-consensus stage fed through real frames, an estimator and a controller;
// the sensors are three IMUs on the platform; the actuator node votes the three commands. Everything but the vehicle is the flight code.
class Rig {
 public:
  Rig() : noise_{Noise(11U), Noise(22U), Noise(33U)} {}

  vehicle::Vehicle veh;
  Guidance guide;
  Fault fault;
  uint8_t usable = 0x07U;
  bool replicas_identical = true;
  double max_pitch_err = 0.0;
  double max_yaw_err = 0.0;
  double sum_sq_pitch = 0.0;
  double sum_sq_yaw = 0.0;
  uint32_t frames = 0U;
  uint32_t votes_lost = 0U;
  uint32_t k = 0U;

  void step() {
    std::array<double, 3> g{};
    std::array<double, 3> a{};
    veh.imu_truth(g, a);
    const uint8_t seq = static_cast<uint8_t>(k);
    std::array<Frame, 6> fr{};
    for (uint8_t n = 0; n < 3U; ++n) {
      Vec3 gy;
      Vec3 ac;
      for (unsigned i = 0; i < 3U; ++i) {
        gy.v[i] = static_cast<float>(g[i]) + (0.1F * noise_[n].uniform());
        ac.v[i] = static_cast<float>(a[i]) + (0.002F * noise_[n].uniform());
      }
      if (fault.active && fault.node == n) {
        gy.v[0] += fault.gyro_bias_dps;
        gy.v[1] += fault.gyro_bias_dps;
        if (fault.stuck) {
          gy = stuck_;
        } else {
          stuck_ = gy;
        }
      }
      fr[n] = pack_gyro(n, gy, seq);
      fr[3U + n] = pack_accel(n, ac, seq);
    }
    std::array<Command, 3> cmd{};
    const Reference ref = guide.at(k);
    for (unsigned r = 0; r < 3U; ++r) {
      cons_[r].begin_frame(usable);
      for (const Frame& f : fr) {
        (void)cons_[r].on_frame(f);
      }
      est_[r].update(cons_[r].consensus(), 0.01F);
      cmd[r] = ctl_[r].step(est_[r].attitude(), ref, 0.01F);
      cmd[r].state_digest = static_cast<uint16_t>(est_[r].digest() ^ ctl_[r].digest());
    }
    for (unsigned r = 1; r < 3U; ++r) {
      replicas_identical = replicas_identical && bits(cmd[r].pitch_deg) == bits(cmd[0].pitch_deg) &&
                           bits(cmd[r].yaw_deg) == bits(cmd[0].yaw_deg) && cmd[r].state_digest == cmd[0].state_digest;
    }
    // the actuator node: vote the commands as they travel on the bus (quantised), hold if they cannot be voted
    std::array<float, 3> pc{};
    std::array<float, 3> yc{};
    for (unsigned r = 0; r < 3U; ++r) {
      const DecodedCommand d = unpack_cmd(pack_cmd(static_cast<uint8_t>(r), cmd[r], seq));
      pc[r] = d.cmd.pitch_deg;
      yc[r] = d.cmd.yaw_deg;
    }
    const VoteResult vp = vote3(pc, 0x07U, 0.05F);
    const VoteResult vy = vote3(yc, 0x07U, 0.05F);
    if (vp.status == VoteStatus::Triplex && vy.status == VoteStatus::Triplex) {
      act_p_ = vp.value;
      act_y_ = vy.value;
    } else {
      ++votes_lost;
    }
    veh.step(act_p_, act_y_, 0.01);
    const double ep = veh.pitch_plane_deg() - static_cast<double>(ref.tilt_y_deg);
    const double ey = veh.yaw_plane_deg() - static_cast<double>(ref.tilt_x_deg);
    max_pitch_err = std::fmax(max_pitch_err, std::fabs(ep));
    max_yaw_err = std::fmax(max_yaw_err, std::fabs(ey));
    sum_sq_pitch += ep * ep;
    sum_sq_yaw += ey * ey;
    ++frames;
    ++k;
  }

  void run(uint32_t n) {
    for (uint32_t i = 0; i < n; ++i) {
      step();
    }
  }
  [[nodiscard]] double rms_pitch() const { return std::sqrt(sum_sq_pitch / frames); }
  [[nodiscard]] double rms_yaw() const { return std::sqrt(sum_sq_yaw / frames); }

 private:
  std::array<Noise, 3> noise_;
  std::array<SensorConsensus, 3> cons_{};
  std::array<AttitudeEstimator, 3> est_{};
  std::array<Controller, 3> ctl_{};
  float act_p_ = 0.0F;
  float act_y_ = 0.0F;
  Vec3 stuck_{};
};

// With TFC_LOOP_VERBOSE set, print what a loop run did (for tuning and for the write-up).
void report(const char* name, const Rig& rig) {
  if (std::getenv("TFC_LOOP_VERBOSE") != nullptr) {
    std::printf("    [%s] frames %u  rms pitch %.3f yaw %.3f  max pitch %.3f yaw %.3f  end pitch plane %.3f yaw %.3f  votes lost %u\n", name,
                rig.frames, rig.rms_pitch(), rig.rms_yaw(), rig.max_pitch_err, rig.max_yaw_err, rig.veh.pitch_plane_deg(),
                rig.veh.yaw_plane_deg(), rig.votes_lost);
  }
}

// The pitch program of pitch_program() at frame k, in degrees: flat to frame 500, then 20 degrees nose-down over 6000 frames, then flat.
double program_deg(uint32_t k) {
  if (k <= 500U) {
    return 0.0;
  }
  return k >= 6500U ? -20.0 : -20.0 * (static_cast<double>(k) - 500.0) / 6000.0;
}

void pitch_program(Rig& rig) {
  rig.guide.add(1, 0U, 0.0F);
  rig.guide.add(1, 500U, 0.0F);
  rig.guide.add(1, 6500U, -20.0F);  // pitch over by 20 degrees in one minute, from the fifth second
}

}  // namespace

TFC_TEST(loop_holds_the_vehicle_on_its_pitch_program) {
  Rig rig;
  pitch_program(rig);
  rig.run(8000);
  report("nominal", rig);
  CHECK(rig.replicas_identical);
  CHECK(rig.votes_lost == 0U);
  CHECK(rig.rms_pitch() < 0.3);
  CHECK(rig.max_pitch_err < 1.0);
  CHECK(rig.max_yaw_err < 0.5);
  CHECK(std::fabs(rig.veh.pitch_plane_deg() + 20.0) < 0.5);  // ended on the program
}

TFC_TEST(loop_survives_a_gust_in_the_pitch_plane) {
  Rig rig;
  pitch_program(rig);
  rig.run(2000);
  rig.veh.set_disturbance(0.0, 0.08);  // rad/s^2 for two seconds
  rig.run(200);
  rig.veh.set_disturbance(0.0, 0.0);
  rig.run(1800);
  report("gust", rig);
  CHECK(rig.replicas_identical);
  CHECK(rig.max_pitch_err < 3.0);
  const double ref_now = program_deg(rig.k);
  CHECK(std::fabs(rig.veh.pitch_plane_deg() - ref_now) < 0.6);  // back on the program within eight seconds
}

TFC_TEST(loop_survives_an_engine_out) {
  Rig rig;
  pitch_program(rig);
  rig.run(3000);
  rig.veh.set_effectiveness_scale(0.8);  // thrust lost: less control authority
  rig.veh.set_disturbance(0.05, 0.1);    // and a steady torque
  rig.run(5000);
  report("engine-out", rig);
  CHECK(rig.replicas_identical);
  CHECK(rig.max_pitch_err < 4.0);
  CHECK(rig.max_yaw_err < 4.0);
  const double ref_now = program_deg(rig.k);
  CHECK(std::fabs(rig.veh.pitch_plane_deg() - ref_now) < 1.0);
  CHECK(std::fabs(rig.veh.yaw_plane_deg()) < 1.0);
}

TFC_TEST(loop_masks_a_sensor_with_a_large_gyro_bias) {
  Rig clean;
  pitch_program(clean);
  clean.run(4000);
  Rig faulty;
  pitch_program(faulty);
  faulty.fault.active = true;
  faulty.fault.node = 1U;
  faulty.fault.gyro_bias_dps = 12.0F;  // one IMU reads 12 dps too much on two axes
  faulty.run(4000);
  CHECK(faulty.replicas_identical);
  CHECK(faulty.rms_pitch() < clean.rms_pitch() + 0.05);  // the median hides it
  CHECK(faulty.max_pitch_err < 1.0);
}

TFC_TEST(loop_masks_a_frozen_sensor) {
  Rig rig;
  pitch_program(rig);
  rig.run(1500);
  rig.fault.active = true;
  rig.fault.node = 2U;
  rig.fault.stuck = true;
  rig.run(4000);
  CHECK(rig.replicas_identical);
  CHECK(rig.max_pitch_err < 1.0);
}

TFC_TEST(loop_with_the_sensor_of_a_latched_node_excluded_still_flies_on_two) {
  Rig rig;
  pitch_program(rig);
  rig.usable = 0x05U;  // the manager has excluded node 1: duplex sensing
  rig.run(5000);
  CHECK(rig.replicas_identical);
  CHECK(rig.max_pitch_err < 1.0);
}

TFC_TEST(loop_without_a_trustworthy_gyro_holds_its_command_and_does_not_invent_one) {
  Rig rig;
  pitch_program(rig);
  rig.run(1000);
  rig.usable = 0x03U;  // two nodes, and one of them wrong by far more than the tolerance: they cannot be told apart
  rig.fault.active = true;
  rig.fault.node = 1U;
  rig.fault.gyro_bias_dps = 12.0F;
  rig.run(200);
  CHECK(rig.replicas_identical);  // all three replicas see the same thing and do the same
  rig.usable = 0x07U;             // the third sensor is back: the loop recovers
  rig.fault.active = false;
  rig.run(1500);
  CHECK(rig.max_pitch_err < 6.0);
  CHECK(std::fabs(rig.veh.pitch_plane_deg() - program_deg(rig.k)) < 1.0);
}

TFC_TEST(loop_is_deterministic_run_to_run) {
  Rig a;
  Rig b;
  pitch_program(a);
  pitch_program(b);
  a.run(2000);
  b.run(2000);
  CHECK(bits64(a.max_pitch_err) == bits64(b.max_pitch_err));
  CHECK(bits64(a.sum_sq_pitch) == bits64(b.sum_sq_pitch));
  CHECK(std::fabs(a.veh.pitch_plane_deg() - b.veh.pitch_plane_deg()) == 0.0);
}
