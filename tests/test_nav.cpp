// SPDX-License-Identifier: MIT
// The navigator (core/include/tfc/nav.hpp, docs/design/GNC.md section 3): the turn from the sensor frame to the navigation frame, a vehicle at rest on the pad staying there, a coasting orbit keeping its energy
// and its period, constant thrust giving the kinematics of constant acceleration, the held accelerometer, the GNSS fix (the first, a correction, one outside the gate, a run of them), and the collector that
// assembles a fix from its three frames.
#include <array>
#include <cmath>

#include "tfc_test.hpp"

#include "tfc/nav.hpp"

namespace {
using tfc::dm::Vec3;
bool near_abs(double a, double b, double tol) { return std::fabs(a - b) <= tol; }
double dist(Vec3 a, Vec3 b) { return tfc::dm::norm(a - b); }
const std::array<float, 4> kIdentity{1.0F, 0.0F, 0.0F, 0.0F};
}  // namespace

TFC_TEST(nav_the_sensor_frame_is_turned_into_the_navigation_frame_by_the_quaternion_and_a_permutation) {
  // the level frame's axes are the navigation frame's (Y, Z, X): a vector on the sensor frame's Z (the long axis, up on the pad) is the navigation +X
  const Vec3 up = tfc::nav::Navigator::sensor_to_nav(kIdentity, Vec3{0.0, 0.0, 1.0});
  CHECK(near_abs(up.x, 1.0, 1e-15) && near_abs(up.y, 0.0, 1e-15) && near_abs(up.z, 0.0, 1e-15));
  const Vec3 sx = tfc::nav::Navigator::sensor_to_nav(kIdentity, Vec3{1.0, 0.0, 0.0});   // sensor X is downrange
  CHECK(near_abs(sx.y, 1.0, 1e-15) && near_abs(sx.x, 0.0, 1e-15));
  const Vec3 sy = tfc::nav::Navigator::sensor_to_nav(kIdentity, Vec3{0.0, 1.0, 0.0});   // sensor Y is crossrange
  CHECK(near_abs(sy.z, 1.0, 1e-15));
  // the sensor frame turned 90 degrees about its own Y (the pitch plane): the long axis now points along the sensor-X direction of the level frame, which is downrange
  const float s = static_cast<float>(std::sqrt(0.5));
  const std::array<float, 4> q{s, 0.0F, s, 0.0F};
  const Vec3 pitched = tfc::nav::Navigator::sensor_to_nav(q, Vec3{0.0, 0.0, 1.0});
  CHECK(near_abs(pitched.y, 1.0, 1e-6) && near_abs(pitched.x, 0.0, 1e-6));
}

TFC_TEST(nav_a_vehicle_at_rest_on_the_pad_reading_the_pads_reaction_stays_where_it_is) {
  tfc::nav::NavConfig cfg;
  tfc::nav::Navigator n(cfg);
  const double g_local = tfc::dm::norm(cfg.gravity.at(cfg.r0));
  for (int k = 0; k < 10000; ++k) {
    n.propagate(kIdentity, Vec3{0.0, 0.0, g_local / tfc::dm::kG0}, true, 0.01);
  }
  CHECK(dist(n.position(), cfg.r0) < 1e-6 && tfc::dm::norm(n.velocity()) < 1e-6);
  CHECK(!n.has_fix() && n.frames_since_fix() == 10000U);
  CHECK(near_abs(n.specific_force_mag(), g_local, 1e-9) && near_abs(n.altitude(), 0.0, 1e-6));
}

TFC_TEST(nav_a_coasting_orbit_keeps_its_radius_and_closes_after_a_period) {
  tfc::nav::NavConfig cfg;
  const double r = cfg.gravity.radius + 400000.0;
  cfg.r0 = Vec3{r, 0.0, 0.0};
  const double v = std::sqrt(cfg.gravity.mu / r);
  cfg.v0 = Vec3{0.0, v, 0.0};
  tfc::nav::Navigator n(cfg);
  const double period = tfc::dm::kTwoPi * std::sqrt(r * r * r / cfg.gravity.mu);
  const int steps = static_cast<int>(period / 0.01);
  double worst = 0.0;
  for (int k = 0; k < steps; ++k) {
    n.propagate(kIdentity, Vec3{}, true, 0.01);
    worst = std::fmax(worst, std::fabs(tfc::dm::norm(n.position()) - r));
  }
  CHECK(worst < 5.0);                                   // it stays on its circle to a few metres
  CHECK(dist(n.position(), cfg.r0) < 200.0);            // and is back where it started (to the part of a period the step count left out)
  const double e0 = 0.5 * v * v - cfg.gravity.mu / r;
  const double e1 = 0.5 * tfc::dm::dot(n.velocity(), n.velocity()) - cfg.gravity.mu / tfc::dm::norm(n.position());
  CHECK(std::fabs(e1 - e0) < 1e-3 * std::fabs(e0) * 1e-3);   // the energy is kept to a part in a million
}

TFC_TEST(nav_constant_thrust_gives_the_kinematics_of_constant_acceleration_and_the_second_order_oblateness_term_is_the_simulators) {
  tfc::nav::NavConfig cfg;
  cfg.gravity.mu = 0.0;   // no gravity: only the specific force
  tfc::nav::Navigator n(cfg);
  for (int k = 0; k < 1000; ++k) {
    n.propagate(kIdentity, Vec3{0.0, 0.0, 2.0}, true, 0.01);   // 2 g up the long axis
  }
  const double a = 2.0 * tfc::dm::kG0;
  CHECK(near_abs(n.velocity().x, a * 10.0, 1e-8) && near_abs(n.position().x - cfg.r0.x, 0.5 * a * 100.0, 1e-6));
  CHECK(near_abs(n.velocity().y, 0.0, 1e-12));
  CHECK(near_abs(n.specific_force().x, a, 1e-9) && near_abs(n.acceleration().x, a, 1e-9));
  // oblateness: at the equator J2 pulls the field up a little (less gravity there), and at the pole it adds to it; the formula is the planet model's
  tfc::nav::Gravity g;
  g.j2 = 1.08263e-3;
  g.pole = Vec3{0.0, 0.0, 1.0};
  const double eq = tfc::dm::norm(g.at(Vec3{g.radius, 0.0, 0.0}));
  const double po = tfc::dm::norm(g.at(Vec3{0.0, 0.0, g.radius}));
  tfc::nav::Gravity sphere;
  CHECK(eq > tfc::dm::norm(sphere.at(Vec3{g.radius, 0.0, 0.0})) && po < tfc::dm::norm(sphere.at(Vec3{0.0, 0.0, g.radius})) + 1.0 && po > eq - 100.0 && eq != po);
}

TFC_TEST(nav_a_lost_accelerometer_is_held_for_a_few_frames_then_the_vehicle_coasts) {
  tfc::nav::NavConfig cfg;
  cfg.gravity.mu = 0.0;
  tfc::nav::Navigator n(cfg);
  n.propagate(kIdentity, Vec3{0.0, 0.0, 1.0}, true, 0.01);
  const double v1 = n.velocity().x;
  for (int k = 0; k < 5; ++k) {
    n.propagate(kIdentity, Vec3{0.0, 0.0, 9.0}, false, 0.01);   // the value offered is not used: the last good one is
  }
  CHECK(near_abs(n.velocity().x, v1 * 6.0, 1e-12));
  const double v6 = n.velocity().x;
  n.propagate(kIdentity, Vec3{0.0, 0.0, 9.0}, false, 0.01);        // the sixth frame without it: coast
  CHECK(near_abs(n.velocity().x, v6, 1e-12));
}

TFC_TEST(nav_the_gnss_fix_starts_corrects_and_is_gated) {
  tfc::nav::NavConfig cfg;
  cfg.gravity.mu = 0.0;
  cfg.gate_frames = 3U;
  tfc::nav::Navigator n(cfg);
  const Vec3 r_true{cfg.r0.x + 100.0, 20.0, -30.0};
  const Vec3 v_true{5.0, 1.0, 0.0};
  CHECK(!n.has_fix());
  n.apply_fix(tfc::nav::GnssFix{r_true, v_true});    // the first fix starts the solution
  CHECK(n.has_fix() && dist(n.position(), r_true) == 0.0 && dist(n.velocity(), v_true) == 0.0 && n.fixes() == 1U && n.frames_since_fix() == 0U);
  // the solution is then pushed off by 40 m and fixes pull it back by the fraction configured each time
  tfc::nav::Navigator m(cfg);
  m.apply_fix(tfc::nav::GnssFix{r_true, v_true});
  m.set_state(tfc::nav::Navigator::State{r_true + Vec3{40.0, 0.0, 0.0}, v_true, true});
  m.apply_fix(tfc::nav::GnssFix{r_true, v_true});
  CHECK(near_abs(dist(m.position(), r_true), 40.0 * (1.0 - cfg.k_pos), 1e-9));
  for (int k = 0; k < 40; ++k) {
    m.apply_fix(tfc::nav::GnssFix{r_true, v_true});
  }
  CHECK(dist(m.position(), r_true) < 1e-3);
  CHECK(near_abs(m.last_position_innovation().x, 0.0, 1e-3) && near_abs(tfc::dm::norm(m.last_velocity_innovation()), 0.0, 1e-12));
  // a fix far outside the gate is not believed, and a run of them starts the solution again from the fix
  const Vec3 far{r_true.x + 5000.0, 0.0, 0.0};
  m.apply_fix(tfc::nav::GnssFix{far, v_true});
  CHECK(m.rejected_fixes() == 1U && dist(m.position(), r_true) < 1e-3);
  m.apply_fix(tfc::nav::GnssFix{far, v_true});
  CHECK(m.restarts() == 0U);
  m.apply_fix(tfc::nav::GnssFix{far, v_true});
  CHECK(m.restarts() == 1U && dist(m.position(), far) == 0.0);
  // a velocity outside the gate counts too
  tfc::nav::Navigator w(cfg);
  w.apply_fix(tfc::nav::GnssFix{r_true, v_true});
  w.apply_fix(tfc::nav::GnssFix{r_true, v_true + Vec3{100.0, 0.0, 0.0}});
  CHECK(w.rejected_fixes() == 1U);
  CHECK(n.config().gate_frames == 3U);
}

TFC_TEST(nav_the_collector_assembles_a_fix_from_three_frames_and_ignores_the_rest) {
  tfc::nav::GnssCollector c;
  tfc::nav::GnssFix g;
  g.r = Vec3{6578137.0, -123456.0, 4321.0};
  g.v = Vec3{-7612.34, 12.5, -0.07};
  const tfc::GnssRaw raw = tfc::nav::to_raw(g);
  CHECK(!c.ready());
  CHECK(!c.offer(tfc::pack_gnss(0, raw, 7U)));
  CHECK(!c.offer(tfc::pack_gnss(2, raw, 7U)));
  CHECK(c.offer(tfc::pack_gnss(1, raw, 7U)) && c.ready());
  const tfc::nav::GnssFix back = c.fix();
  CHECK(dist(back.r, g.r) < 0.5 * 1.8 && dist(back.v, g.v) < 0.01);
  // a new fix with another sequence byte forgets the parts of the last
  CHECK(!c.offer(tfc::pack_gnss(0, raw, 8U)) && !c.offer(tfc::pack_gnss(1, raw, 8U)));
  CHECK(!c.offer(tfc::pack_gnss(2, raw, 9U)));    // a part of yet another: the two before it are gone
  CHECK(!c.offer(tfc::pack_gnss(0, raw, 9U)));
  CHECK(c.offer(tfc::pack_gnss(1, raw, 9U)));
  // not a GNSS frame, or a damaged one
  CHECK(!c.offer(tfc::pack_sim_rates(tfc::Vec3{}, 1U)));
  tfc::Frame bad = tfc::pack_gnss(0, raw, 10U);
  bad.data[7] ^= 0x55U;
  CHECK(!c.offer(bad));
  // negative numbers and large ones survive the 24-bit packing; a value beyond it saturates
  tfc::GnssRaw big;
  big.pos_m = {-8000000, 8388607, 123};
  big.vel_cms = {-8388607, 99999999, -1};
  tfc::GnssRaw out;
  uint8_t seq = 0U;
  unsigned part = 0U;
  CHECK(tfc::unpack_gnss(tfc::pack_gnss(0, big, 3U), out, seq, part) && out.pos_m[0] == -8000000 && out.pos_m[1] == 8388607 && part == 0U && seq == 3U);
  CHECK(tfc::unpack_gnss(tfc::pack_gnss(1, big, 3U), out, seq, part) && out.pos_m[2] == 123 && out.vel_cms[0] == -8388607 && part == 1U);
  CHECK(tfc::unpack_gnss(tfc::pack_gnss(2, big, 3U), out, seq, part) && out.vel_cms[1] == 8388607 && out.vel_cms[2] == -1 && part == 2U);
  CHECK(tfc::unpack_gnss(tfc::pack_gnss(0, tfc::GnssRaw{{-9000000, 0, 0}, {}}, 3U), out, seq, part) && out.pos_m[0] == -8388607);
}

TFC_TEST(nav_propulsion_and_surface_commands_survive_the_frames_and_are_voted_by_the_consumer) {
  tfc::PropCommand p;
  p.throttle = 0.735F;
  p.groups = 0x0DU;
  p.events = tfc::propbit::kSeparate | tfc::propbit::kChute1;
  p.phase = 7U;
  p.roll_deg = -3.3F;
  const tfc::DecodedProp d = tfc::unpack_prop(tfc::pack_prop(2U, p, 42U));
  CHECK(d.ok && d.seq == 42U && std::fabs(d.p.throttle - 0.735F) <= 0.0026F && d.p.groups == 0x0DU && d.p.events == p.events && d.p.phase == 7U && std::fabs(d.p.roll_deg + 3.25F) < 0.13F);
  tfc::PropCommand over;
  over.throttle = 1.7F;
  over.roll_deg = 900.0F;
  const tfc::DecodedProp o = tfc::unpack_prop(tfc::pack_prop(0U, over, 1U));
  CHECK(o.p.throttle == 1.0F && o.p.roll_deg > 31.0F);
  tfc::PropCommand under;
  under.throttle = -1.0F;
  under.roll_deg = -900.0F;
  const tfc::DecodedProp u = tfc::unpack_prop(tfc::pack_prop(1U, under, 1U));
  CHECK(u.p.throttle == 0.0F && u.p.roll_deg < -31.0F);
  tfc::Frame bad = tfc::pack_prop(0U, p, 1U);
  bad.data[3] ^= 1U;
  CHECK(!tfc::unpack_prop(bad).ok);
  tfc::Frame wrong_node = tfc::pack_prop(0U, p, 1U);
  wrong_node.id = tfc::id::kPropBase + 3U;
  CHECK(!tfc::unpack_prop(wrong_node).ok && !tfc::unpack_prop(tfc::pack_cmd(0U, tfc::Command{}, 1U)).ok);
  tfc::SurfCommand s;
  s.deg = {12.34F, -45.67F, 90.0F, -101.0F};
  const tfc::DecodedSurf ds = tfc::unpack_surf(tfc::pack_surf(1U, s, 9U));
  CHECK(ds.ok && ds.seq == 9U);
  for (unsigned i = 0; i < 4U; ++i) {
    CHECK(std::fabs(ds.s.deg[i] - s.deg[i]) <= 0.026F);
  }
  tfc::SurfCommand sat;
  sat.deg = {500.0F, -500.0F, 0.0F, 0.04F};
  const tfc::DecodedSurf dsat = tfc::unpack_surf(tfc::pack_surf(0U, sat, 1U));
  CHECK(dsat.s.deg[0] > 102.0F && dsat.s.deg[1] < -102.0F && dsat.s.deg[2] == 0.0F && dsat.s.deg[3] == 0.05F);
  tfc::Frame bads = tfc::pack_surf(0U, s, 1U);
  bads.data[0] ^= 1U;
  CHECK(!tfc::unpack_surf(bads).ok);
  tfc::Frame wrong = tfc::pack_surf(0U, s, 1U);
  wrong.id = tfc::id::kSurfBase + 3U;
  CHECK(!tfc::unpack_surf(wrong).ok);
  // the consumer's vote: the middle value, and two of three for each bit
  CHECK(tfc::mid3(1.0F, 9.0F, 3.0F) == 3.0F && tfc::mid3(5.0F, 2.0F, 1.0F) == 2.0F && tfc::mid3(0.0F, 4.0F, 9.0F) == 4.0F);
  CHECK(tfc::majority3(0x0FU, 0x33U, 0x55U) == 0x17U);
}
