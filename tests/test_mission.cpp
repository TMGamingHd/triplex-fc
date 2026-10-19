// SPDX-License-Identifier: MIT
// The mission sequencer (core/include/tfc/mission.hpp, docs/design/GNC.md section 7): the tables' tracks, every kind of phase and every way a phase ends, the attitude holds, and three flights in a point-mass model
// in which the vehicle's attitude is the reference the sequencer gives and the thrust is what it commands: an ascent to a circular orbit (PEG, seeded and not), a booster's boost-back, entry and landing burn to the
// catch point, and a parachute descent's events.
#include <cmath>
#include <tuple>

#include "tfc_test.hpp"

#include "tfc/mission.hpp"

namespace {
using namespace tfc::gnc;
using tfc::dm::Vec3;
using tfc::dm::dot;
using tfc::dm::norm;

constexpr double kR = 6378137.0;

bool near_abs(double a, double b, double tol) { return std::fabs(a - b) <= tol; }

Inputs inputs_at(double alt, Vec3 v, uint32_t frame = 0U) {
  Inputs in;
  in.frame = frame;
  in.r = Vec3{kR + alt, 0.0, 0.0};
  in.v = v;
  return in;
}

// Tables of n_phases coast phases that do nothing, to be filled in by the test.
Tables plain(unsigned n_phases) {
  Tables t;
  t.n_phases = static_cast<uint8_t>(n_phases);
  t.mass0 = 100000.0;
  t.mixer[0].gimbal_pitch = 1.0F;
  t.mixer[0].gimbal_yaw = 1.0F;
  for (unsigned i = 0; i < n_phases; ++i) {
    t.phase[i].kind = kind::kCoast;
    t.phase[i].end = end::kNever;
    t.phase[i].slew_dps = 1000.0F;
  }
  return t;
}

Vec3 axis_of(const Mission& m) { return tfc::dm::rotate(m.target(), Vec3{1.0, 0.0, 0.0}); }

// Two phases; the first ends by `end_kind` at `value`. Steps with `before` for a few frames (it must stay in phase 0), then with `after` (it must move on).
bool ends_when(uint8_t end_kind, float value, const Inputs& before, const Inputs& after, double mass = 100000.0, unsigned settle_frames = 3U, void (*tweak)(Tables&) = nullptr) {
  Tables t = plain(2);
  t.phase[0].end = end_kind;
  t.phase[0].end_value = value;
  t.site = Vec3{kR, 0.0, 0.0};
  if (tweak != nullptr) {
    tweak(t);
  }
  Mission m;
  m.configure(&t);
  m.set_mass(mass);
  m.start(before);
  for (unsigned i = 0; i < settle_frames; ++i) {
    (void)m.step(before);
  }
  if (m.phase() != 0U) {
    return false;
  }
  (void)m.step(after);
  (void)m.step(after);
  return m.phase() == 1U;
}

// A point mass under gravity and the drag the guidance predicts with; the vehicle's attitude is the sequencer's reference.
struct Body {
  Vec3 r;
  Vec3 v;
  double m = 1.0e5;
  double t = 0.0;
  double exhaust = 3250.0;
};

void advance(Body& b, const Output& out, const tfc::nav::Gravity& g, const tfc::descent::DragModel& drag, double dt, double thrust_scale = 1.0) {
  const Vec3 axis = tfc::dm::rotate(out.q_ref, Vec3{1.0, 0.0, 0.0});
  const double thrust = static_cast<double>(out.thrust) * thrust_scale;
  const auto accel = [&](Vec3 pos, Vec3 vel) {
    const double alt = norm(pos) - g.radius;
    const double rho = drag.rho0 * tfc::dm::exp_(-std::fmax(alt, 0.0) / drag.scale_height);
    return g.at(pos) - (vel * (0.5 * rho * norm(vel) / drag.beta)) + (axis * (thrust / b.m));
  };
  const Vec3 a0 = accel(b.r, b.v);
  const Vec3 r1 = b.r + (b.v * dt) + (a0 * (0.5 * dt * dt));
  const Vec3 a1 = accel(r1, b.v + (a0 * dt));
  b.v = b.v + ((a0 + a1) * (0.5 * dt));
  b.r = r1;
  b.m = std::fmax(b.m - (thrust / b.exhaust * dt), 1000.0);
  b.t += dt;
}
}  // namespace

TFC_TEST(sequencer_the_gain_and_throttle_tracks_interpolate_between_their_points_and_hold_beyond_them) {
  GainTrack g;
  const tfc::att::Gains3 none{};
  CHECK(g.at(10U).pitch.kp == none.pitch.kp && g.alloc_at(10U).from_pitch[0] == 0.0F && g.thrust_at(10U) == 0.0F);   // an empty track: the default gains and nothing else
  g.n = 3U;
  g.frame = {10U, 110U, 210U};
  g.gains[0].pitch.kp = 1.0F;
  g.gains[1].pitch.kp = 3.0F;
  g.gains[2].pitch.kp = 3.0F;
  g.gains[0].roll.kd = 0.0F;
  g.gains[1].roll.kd = 2.0F;
  g.thrust = {100.0F, 200.0F, 200.0F};
  g.alloc[0].from_pitch[2] = 1.0F;
  g.alloc[1].from_pitch[2] = 3.0F;
  g.alloc[0].from_yaw[1] = 4.0F;
  g.alloc[1].from_yaw[1] = 8.0F;
  g.alloc[0].from_roll[0] = -2.0F;
  g.alloc[1].from_roll[0] = 2.0F;
  CHECK(near_abs(static_cast<double>(g.at(0U).pitch.kp), 1.0, 1e-6));     // before the first point: held
  CHECK(near_abs(static_cast<double>(g.at(60U).pitch.kp), 2.0, 1e-6));    // half way between the first two
  CHECK(near_abs(static_cast<double>(g.at(60U).roll.kd), 1.0, 1e-6));
  CHECK(near_abs(static_cast<double>(g.at(1000U).pitch.kp), 3.0, 1e-6));  // after the last: held
  CHECK(near_abs(static_cast<double>(g.thrust_at(60U)), 150.0, 1e-4));
  CHECK(near_abs(static_cast<double>(g.alloc_at(60U).from_pitch[2]), 2.0, 1e-6));
  CHECK(near_abs(static_cast<double>(g.alloc_at(60U).from_yaw[1]), 6.0, 1e-6));
  CHECK(near_abs(static_cast<double>(g.alloc_at(60U).from_roll[0]), 0.0, 1e-6));
  ThrottleTrack tr;
  CHECK(near_abs(static_cast<double>(tr.at(5U, 0.7F)), 0.7, 1e-6));       // no points: the fallback
  tr.n = 2U;
  tr.frame = {100U, 200U};
  tr.value = {1.0F, 0.5F};
  CHECK(near_abs(static_cast<double>(tr.at(0U, 0.0F)), 1.0, 1e-6));
  CHECK(near_abs(static_cast<double>(tr.at(150U, 0.0F)), 0.75, 1e-6));
  CHECK(near_abs(static_cast<double>(tr.at(900U, 0.0F)), 0.5, 1e-6));
}

TFC_TEST(sequencer_a_machine_without_tables_or_not_started_gives_the_inputs_back_and_its_state_can_be_taken_over) {
  Mission m;
  CHECK(!m.configured() && !m.started());
  m.start(inputs_at(0.0, Vec3{}));
  CHECK(!m.started());
  Tables t = plain(2);
  t.phase[0].end = end::kTime;
  t.phase[0].end_value = 0.05F;
  m.configure(&t);
  CHECK(m.configured() && near_abs(m.mass(), 100000.0, 1e-9));
  const Output idle = m.step(inputs_at(0.0, Vec3{}));   // configured but not started: the phase 0 output and no progress
  CHECK(idle.phase == 0U && m.phase_time_s() == 0.0);
  m.start(inputs_at(0.0, Vec3{}));
  CHECK(m.started());
  for (int i = 0; i < 20; ++i) {
    (void)m.step(inputs_at(0.0, Vec3{}));
  }
  CHECK(m.phase() == 1U);
  const Mission::State s = m.state();
  Mission other;
  other.configure(&t);
  other.set_state(s);
  CHECK(other.phase() == 1U && other.started() && near_abs(other.phase_time_s(), m.phase_time_s(), 1e-12));
  (void)other.step(inputs_at(0.0, Vec3{}));   // the last phase has no successor: it stays
  CHECK(other.phase() == 1U);
}

TFC_TEST(sequencer_a_program_phase_flies_the_pitch_program_and_the_throttle_track_and_burns_mass) {
  Tables t = plain(2);
  t.phase[0].kind = kind::kProgram;
  t.phase[0].end = end::kTime;
  t.phase[0].end_value = 2.0F;
  t.phase[0].throttle = 0.9F;
  t.phase[0].mdot = 100.0F;
  t.phase[0].thrust = 1.0e6F;
  t.phase[0].groups = 3U;
  t.phase[0].throttle_track = 0U;
  t.throttle[0].n = 2U;
  t.throttle[0].frame = {0U, 100U};
  t.throttle[0].value = {1.0F, 0.5F};
  CHECK(t.program.add(1U, 0U, 0.0F) && t.program.add(1U, 100U, 30.0F) && t.program.add(0U, 0U, 0.0F));
  t.phase[1].kind = kind::kProgram;   // the same phase with a constant throttle and no track
  t.phase[1].throttle = 0.6F;
  t.phase[1].mdot = 50.0F;
  t.phase[1].groups = 3U;
  Mission m;
  m.configure(&t);
  m.start(inputs_at(0.0, Vec3{}));
  Output out;
  for (int i = 0; i < 50; ++i) {
    out = m.step(inputs_at(0.0, Vec3{}));
  }
  CHECK(out.phase == 0U && near_abs(static_cast<double>(out.throttle), 0.75, 0.02) && out.groups == 3U);   // half way down the throttle track
  const double tilt = std::acos(axis_of(m).x) * tfc::dm::kRadToDeg;
  CHECK(near_abs(tilt, 15.0, 0.5));                                                                         // half way up the program
  CHECK(m.mass() < 100000.0 && m.mass() > 99900.0);
  for (int i = 0; i < 160; ++i) {
    out = m.step(inputs_at(0.0, Vec3{}));
  }
  CHECK(out.phase == 1U && near_abs(static_cast<double>(out.throttle), 0.6, 1e-6));                          // past the end of the track: the constant
}

TFC_TEST(sequencer_every_way_a_phase_ends) {
  const Vec3 slow{0.0, 100.0, 0.0};
  const Vec3 fast{0.0, 300.0, 0.0};
  // never
  {
    Tables t = plain(2);
    Mission m;
    m.configure(&t);
    m.start(inputs_at(0.0, slow));
    for (int i = 0; i < 10; ++i) {
      (void)m.step(inputs_at(0.0, fast));
    }
    CHECK(m.phase() == 0U);
  }
  // after a time (and the clock starts again in the next phase)
  {
    Tables t = plain(2);
    t.phase[0].end = end::kTime;
    t.phase[0].end_value = 0.04F;
    Mission m;
    m.configure(&t);
    m.start(inputs_at(0.0, slow));
    for (int i = 0; i < 10; ++i) {
      (void)m.step(inputs_at(0.0, slow));
    }
    CHECK(m.phase() == 1U && m.phase_time_s() < 0.07);
  }
  CHECK(ends_when(end::kSpeedAbove, 200.0F, inputs_at(0.0, slow), inputs_at(0.0, fast)));
  CHECK(ends_when(end::kApoapsis, 0.0F, inputs_at(1000.0, Vec3{50.0, 100.0, 0.0}), inputs_at(1000.0, Vec3{-5.0, 100.0, 0.0})));
  CHECK(!ends_when(end::kApoapsis, 100.0F, inputs_at(1000.0, Vec3{50.0, 100.0, 0.0}), inputs_at(1000.0, Vec3{-5.0, 100.0, 0.0})));   // not before the time
  CHECK(ends_when(end::kAltitudeBelow, 500.0F, inputs_at(1000.0, Vec3{-50.0, 100.0, 0.0}), inputs_at(400.0, Vec3{-50.0, 100.0, 0.0})));
  CHECK(!ends_when(end::kAltitudeBelow, 500.0F, inputs_at(1000.0, Vec3{-50.0, 100.0, 0.0}), inputs_at(400.0, Vec3{50.0, 100.0, 0.0})));   // rising through it is not descending through it
  CHECK(ends_when(end::kAltitudeAbove, 500.0F, inputs_at(100.0, Vec3{50.0, 100.0, 0.0}), inputs_at(900.0, Vec3{50.0, 100.0, 0.0})));
  CHECK(ends_when(end::kMassBelow, 90000.0F, inputs_at(0.0, slow), inputs_at(0.0, slow), 100000.0, 3U, [](Tables& t) { t.phase[0].mdot = 300000.0F; t.phase[0].kind = kind::kCoast; t.phase[0].groups = 1U; }));
  {
    Tables t = plain(2);
    t.phase[0].end = end::kAligned;
    t.phase[0].end_value = 5.0F;
    Mission m;
    m.configure(&t);
    m.start(inputs_at(0.0, slow));
    for (int i = 0; i < 99; ++i) {
      (void)m.step(inputs_at(0.0, slow));
    }
    CHECK(m.phase() == 0U);
    for (int i = 0; i < 5; ++i) {
      (void)m.step(inputs_at(0.0, slow));
    }
    CHECK(m.phase() == 1U);
    // a reference far from the attitude does not end it
    Tables t2 = plain(2);
    t2.phase[0].end = end::kAligned;
    t2.phase[0].end_value = 5.0F;
    t2.phase[0].hold = hold::kFixed;
    t2.phase[0].p[1] = 1.0F;
    Mission m2;
    m2.configure(&t2);
    m2.start(inputs_at(0.0, slow));
    for (int i = 0; i < 200; ++i) {
      (void)m2.step(inputs_at(0.0, slow));
    }
    CHECK(m2.phase() == 0U);
  }
}

TFC_TEST(sequencer_the_landing_phases_end_at_the_stopping_height_and_at_the_catch_point) {
  const auto landing_tables = [](Tables& t) {
    t.landing_height = 40.0;
    t.landing.decel_plan = 20.0;
    t.landing.sink_ms = 0.5;
    t.landing.aim_below_m = 2.0;
    t.ignition_margin = 1.0;
  };
  // ignition: under the height and at the stopping height for the speed (v^2 / 40 = 2500 m at 316 m/s)
  CHECK(ends_when(end::kIgnition, 6000.0F, inputs_at(8000.0, Vec3{-316.0, 0.0, 0.0}), inputs_at(2000.0, Vec3{-316.0, 0.0, 0.0}), 100000.0, 3U, landing_tables));
  CHECK(!ends_when(end::kIgnition, 6000.0F, inputs_at(8000.0, Vec3{-316.0, 0.0, 0.0}), inputs_at(7000.0, Vec3{-316.0, 0.0, 0.0}), 100000.0, 3U, landing_tables));   // not under the height
  CHECK(!ends_when(end::kIgnition, 6000.0F, inputs_at(5000.0, Vec3{-100.0, 0.0, 0.0}), inputs_at(4900.0, Vec3{-100.0, 0.0, 0.0}), 100000.0, 3U, landing_tables));   // under it but not yet due
  {
    // a vehicle in orbit on the far side of the planet is far "below" the site along the site's up: it must not light up there
    Inputs far = inputs_at(239000.0, Vec3{316.0, 0.0, 0.0});
    far.r = Vec3{-(kR + 239000.0), 0.0, 0.0};
    CHECK(!ends_when(end::kIgnition, 6000.0F, inputs_at(8000.0, Vec3{-316.0, 0.0, 0.0}), far, 100000.0, 3U, landing_tables));
  }
  // touchdown: at the catch point, which the burn aims 2 m under
  CHECK(ends_when(end::kTouchdown, 0.0F, inputs_at(60.0, Vec3{-1.0, 0.0, 0.0}), inputs_at(37.5, Vec3{-1.0, 0.0, 0.0}), 100000.0, 3U, landing_tables));
  // the catch height follows the mass
  Tables t = plain(1);
  t.landing_height = 40.0;
  t.landing_height_slope = -1.0e-4;
  t.landing_mass_ref = 50000.0;
  Mission m;
  m.configure(&t);
  m.set_mass(150000.0);
  CHECK(near_abs(m.catch_height(), 40.0 - 10.0, 1e-9));
}

TFC_TEST(sequencer_the_attitude_holds_point_where_they_are_named_and_the_reference_turns_at_the_phases_rate) {
  const Vec3 v{100.0, 2000.0, 300.0};
  const Inputs in = inputs_at(100000.0, v);
  const Vec3 up{1.0, 0.0, 0.0};
  const Vec3 vel = tfc::dm::unit(v, up);
  const Vec3 horiz = tfc::dm::unit(Vec3{0.0, 2000.0, 300.0}, vel);
  struct Case {
    uint8_t hold;
    Vec3 expect;
  };
  const Case cases[] = {{hold::kPrograde, vel}, {hold::kRetrograde, vel * -1.0}, {hold::kRadial, up}, {hold::kFixed, Vec3{0.0, 0.0, 1.0}}, {hold::kRetroHorizontal, horiz * -1.0}, {hold::kProHorizontal, horiz}};
  for (const Case& c : cases) {
    Tables t = plain(1);
    t.phase[0].hold = c.hold;
    t.phase[0].p[2] = 1.0F;
    Mission m;
    m.configure(&t);
    m.start(in);
    (void)m.step(in);
    CHECK(norm(axis_of(m) - c.expect) < 1e-6);
  }
  // inertial: keeps the reference it began with, and a slow slew takes a while to get there
  Tables t = plain(1);
  t.phase[0].hold = hold::kFixed;
  t.phase[0].p[1] = 1.0F;
  t.phase[0].slew_dps = 9.0F;
  Mission m;
  m.configure(&t);
  m.start(in);
  Output out;
  for (int i = 0; i < 100; ++i) {
    out = m.step(in);   // a second at 9 degrees a second
  }
  const Vec3 now = tfc::dm::rotate(out.q_ref, Vec3{1.0, 0.0, 0.0});
  CHECK(near_abs(std::acos(dot(now, Vec3{1.0, 0.0, 0.0})) * tfc::dm::kRadToDeg, 9.0, 0.3));
  CHECK(norm(out.w_ref) > 0.1);
  Tables t2 = plain(1);
  Mission m2;
  m2.configure(&t2);
  m2.start(in);
  Output o2 = m2.step(in);
  CHECK(norm(tfc::dm::rotate(o2.q_ref, Vec3{1.0, 0.0, 0.0}) - Vec3{1.0, 0.0, 0.0}) < 1e-9);   // inertial coast: stays where it was
}

TFC_TEST(sequencer_a_hold_and_a_done_phase_command_no_engines_and_a_burning_coast_burns_mass) {
  Tables t = plain(3);
  t.phase[0].kind = kind::kHold;
  t.phase[0].groups = 7U;
  t.phase[0].end = end::kTime;
  t.phase[0].end_value = 0.02F;
  t.phase[1].kind = kind::kCoast;   // hot staging: engines on while coasting in attitude
  t.phase[1].groups = 3U;
  t.phase[1].throttle = 0.5F;
  t.phase[1].mdot = 1000.0F;
  t.phase[1].thrust = 1.0e6F;
  t.phase[1].end = end::kTime;
  t.phase[1].end_value = 0.5F;
  t.phase[2].kind = kind::kDone;
  t.phase[2].groups = 3U;
  Mission m;
  m.configure(&t);
  m.start(inputs_at(0.0, Vec3{}));
  Output out = m.step(inputs_at(0.0, Vec3{}));
  CHECK(out.groups == 0U && out.throttle == 0.0F);
  for (int i = 0; i < 10; ++i) {
    out = m.step(inputs_at(0.0, Vec3{}));
  }
  CHECK(out.phase == 1U && out.groups == 3U && near_abs(static_cast<double>(out.throttle), 0.5, 1e-6) && near_abs(static_cast<double>(out.thrust), 0.5e6, 1.0));
  CHECK(m.mass() < 100000.0);
  for (int i = 0; i < 80; ++i) {
    out = m.step(inputs_at(0.0, Vec3{}));
  }
  CHECK(out.phase == 2U && out.groups == 0U && out.thrust == 0.0F);
  Tables t3 = plain(1);
  t3.phase[0].kind = kind::kCoast;   // a coast with no groups commands zero throttle whatever the phase says
  t3.phase[0].throttle = 0.8F;
  Mission m3;
  m3.configure(&t3);
  m3.start(inputs_at(0.0, Vec3{}));
  CHECK(m3.step(inputs_at(0.0, Vec3{})).throttle == 0.0F);
}

TFC_TEST(sequencer_a_parachute_phase_raises_its_events_at_their_altitudes_and_keeps_them_and_hands_no_attitude_to_the_loop) {
  Tables t = plain(1);
  t.phase[0].kind = kind::kChute;
  t.phase[0].p[0] = 3000.0F;
  t.phase[0].p[1] = 800.0F;
  t.phase[0].events = tfc::propbit::kJettison;
  Mission m;
  m.configure(&t);
  m.start(inputs_at(5000.0, Vec3{-50.0, 0.0, 0.0}));
  Output out = m.step(inputs_at(5000.0, Vec3{-50.0, 0.0, 0.0}));
  CHECK(!out.control && out.events == tfc::propbit::kJettison && out.groups == 0U);
  out = m.step(inputs_at(2500.0, Vec3{50.0, 0.0, 0.0}));   // climbing: nothing
  CHECK(out.events == tfc::propbit::kJettison);
  out = m.step(inputs_at(2500.0, Vec3{-50.0, 0.0, 0.0}));
  CHECK((out.events & tfc::propbit::kChute0) != 0U && (out.events & tfc::propbit::kChute1) == 0U);
  out = m.step(inputs_at(700.0, Vec3{-30.0, 0.0, 0.0}));
  CHECK((out.events & tfc::propbit::kChute0) != 0U && (out.events & tfc::propbit::kChute1) != 0U);
  out = m.step(inputs_at(5000.0, Vec3{-30.0, 0.0, 0.0}));   // the events stay once raised
  CHECK((out.events & tfc::propbit::kChute1) != 0U);
}

TFC_TEST(sequencer_a_ship_is_steered_by_peg_to_a_circular_orbit_with_the_design_solution_and_without_it) {
  const tfc::nav::Gravity g;
  const double r_t = kR + 250000.0;
  const double v_t = std::sqrt(g.mu / r_t);
  Tables t = plain(2);
  t.nav.gravity = g;
  t.mass0 = 1.7e6;
  t.phase[0].kind = kind::kPeg;
  t.phase[0].end = end::kCutoff;
  t.phase[0].groups = 1U;
  t.phase[0].thrust = 1.55e7F;
  t.phase[0].mdot = 4300.0F;
  t.phase[0].throttle = 1.0F;
  t.phase[0].p[0] = static_cast<float>(r_t);
  t.phase[0].p[1] = static_cast<float>(v_t);
  t.phase[0].p[2] = 0.0F;
  t.phase[0].p[8] = 1.0e5F;
  t.phase[0].slew_dps = 1000.0F;
  t.phase[1].hold = hold::kPrograde;
  const Vec3 r0{kR + 60000.0, 0.0, 0.0};
  const Vec3 v0{400.0, 1500.0, 0.0};
  double seed[tfc::peg::kUnknowns] = {};
  for (int pass = 0; pass < 2; ++pass) {
    Mission m;
    m.configure(&t);
    m.set_design_mode(pass == 0);
    Body b;
    b.r = r0;
    b.v = v0;
    b.m = t.mass0;
    b.exhaust = static_cast<double>(t.phase[0].thrust) / static_cast<double>(t.phase[0].mdot);   // (the model the sequencer holds is the vehicle)
    Inputs in;
    in.r = b.r;
    in.v = b.v;
    in.specific_force = pass == 0 ? 0.0 : 9.0;
    m.start(in);
    bool cut = false;
    for (int k = 0; k < 60000 && !cut; ++k) {
      if (pass == 0 && k == 1) {
        const auto sol = m.peg_solution();   // (the design records the solution at the start of the burn)
        for (std::size_t i = 0; i < tfc::peg::kUnknowns; ++i) {
          seed[i] = sol[i];
        }
      }
      in.r = b.r;
      in.v = b.v;
      in.specific_force = pass == 0 ? 0.0 : static_cast<double>(t.phase[0].thrust) / b.m;
      const Output out = m.step(in);
      in.q = out.q_ref;
      advance(b, out, g, tfc::descent::DragModel{0.0, 7200.0, 1.0e9}, 0.01);
      cut = m.phase() == 1U;
    }
    CHECK(cut);
    CHECK(near_abs(norm(b.r), r_t, 8000.0));
    CHECK(near_abs(norm(b.v), v_t, 25.0));
    CHECK(m.peg_out().time_to_go >= 0.0);
    if (pass == 0) {
      CHECK(seed[4] > 100.0);
      for (std::size_t i = 0; i < tfc::peg::kUnknowns; ++i) {
        t.phase[0].p[3U + i] = static_cast<float>(seed[i]);   // the design's solution is the flight's start
      }
    }
  }
}

TFC_TEST(sequencer_a_booster_burns_back_falls_and_lands_on_the_catch_point_in_a_point_mass_model) {
  const tfc::nav::Gravity g;
  tfc::descent::DragModel drag;
  drag.beta = 4500.0;
  Tables t = plain(6);
  t.nav.gravity = g;
  t.mass0 = 9.0e5;
  t.site = Vec3{kR, 0.0, 0.0};
  t.ground_radius = kR;
  t.drag = drag;
  t.landing_height = 37.8;
  t.landing.tilt_max_deg = 40.0;
  t.landing.decel_plan = 40.0;
  t.engines.thrust_each = 2.449e6;
  t.engines.counts = {3U, 3U, 3U, 13U};
  t.engines.options = 4U;
  t.ignition_margin = 1.2;
  t.arm_height = 70.0;
  t.phase[0].kind = kind::kCoast;
  t.phase[0].hold = hold::kRetroHorizontal;
  t.phase[0].end = end::kAligned;
  t.phase[0].end_value = 8.0F;
  t.phase[0].slew_dps = 60.0F;
  t.phase[1].kind = kind::kBoostback;
  t.phase[1].end = end::kCutoff;
  t.phase[1].groups = 15U;
  t.phase[1].thrust = 3.18e7F;
  t.phase[1].mdot = 9800.0F;
  t.phase[1].throttle = 1.0F;
  t.phase[1].p[0] = 500.0F;      // aim 500 m beyond
  t.phase[1].p[1] = 300000.0F;   // reserve
  t.phase[1].p[2] = 5.0F;        // lifted 5 degrees
  t.phase[1].p[3] = 0.0F;
  t.phase[1].slew_dps = 60.0F;
  t.phase[2].hold = hold::kRadial;
  t.phase[2].end = end::kAltitudeBelow;
  t.phase[2].end_value = 70000.0F;
  t.phase[2].slew_dps = 60.0F;
  t.phase[3].kind = kind::kGlide;
  t.phase[3].end = end::kIgnition;
  t.phase[3].end_value = 8000.0F;
  t.phase[3].p[0] = 12.0F;       // alpha max
  t.phase[3].p[2] = 0.0F;
  t.phase[3].p[3] = 3.5F;
  t.phase[3].p[4] = 330.0F;
  t.phase[3].p[5] = 3600.0F;
  t.phase[3].p[6] = 10.0F;       // some brake, so that the tilt-back branch is taken
  t.phase[3].slew_dps = 30.0F;
  t.phase[4].kind = kind::kLanding;
  t.phase[4].end = end::kTouchdown;
  t.phase[4].p[0] = 7.0F;
  t.phase[4].p[1] = 7.0F;
  t.phase[4].p[2] = 7.0F;
  t.phase[4].p[3] = 15.0F;
  t.phase[4].mdot = 9800.0F / 13.0F;
  t.phase[4].slew_dps = 40.0F;
  t.phase[5].kind = kind::kDone;
  Mission m;
  m.configure(&t);
  Body b;
  b.r = Vec3{kR + 62000.0, 52000.0, 0.0};
  b.v = Vec3{350.0, 1350.0, 0.0};
  b.m = t.mass0;
  b.exhaust = 3200.0;
  Inputs in;
  in.r = b.r;
  in.v = b.v;
  // the vehicle starts pointing along its velocity; the first phase turns it round
  in.q = tfc::att::attitude_from_axes(tfc::dm::unit(b.v, Vec3{1.0, 0.0, 0.0}), Vec3{0.0, 0.0, 1.0});
  m.start(in);
  bool saw_boost = false;
  bool saw_glide = false;
  bool saw_landing = false;
  bool landed = false;
  double sink = 0.0;
  double miss = 0.0;
  double tilt_deg = 0.0;
  double max_alpha = 0.0;
  for (int k = 0; k < 60000 && !landed; ++k) {
    in.r = b.r;
    in.v = b.v;
    in.specific_force = 0.0;
    const Output out = m.step(in);
    in.q = out.q_ref;
    saw_boost = saw_boost || out.phase == 1U;
    saw_glide = saw_glide || out.phase == 3U;
    saw_landing = saw_landing || out.phase == 4U;
    max_alpha = std::fmax(max_alpha, m.aoa_deg());
    b.m = std::fmax(b.m, 1000.0);
    advance(b, out, g, drag, 0.01);
    m.set_mass(m.mass());   // (the sequencer's own estimate is used)
    const double h = norm(b.r) - kR - m.catch_height();
    if (out.phase >= 4U && h <= 1.0) {   // the pins at the arms: the catch
      landed = true;
      const Vec3 up = tfc::dm::unit(b.r, Vec3{1.0, 0.0, 0.0});
      sink = -dot(b.v, up);
      miss = norm(tfc::dm::perp(b.r - Vec3{kR + m.catch_height(), 0.0, 0.0}, up));
      tilt_deg = std::acos(dot(tfc::dm::rotate(out.q_ref, Vec3{1.0, 0.0, 0.0}), up)) * tfc::dm::kRadToDeg;
    }
  }
  CHECK(saw_boost && saw_glide && saw_landing);
  CHECK(landed);
  CHECK(miss < 12.0);
  CHECK(sink < 4.0 && sink > -1.0);
  CHECK(tilt_deg < 8.0);
  CHECK(max_alpha > 0.0 && max_alpha <= 12.0 + 1e-9);
  CHECK(m.landing_out().engines >= 3U);
  CHECK(m.mass() > 3.0e5);   // propellant was left
}

TFC_TEST(sequencer_a_landing_burn_waits_until_it_is_due_and_the_boost_back_stops_at_its_reserve) {
  const tfc::nav::Gravity g;
  Tables t = plain(2);
  t.nav.gravity = g;
  t.site = Vec3{kR, 0.0, 0.0};
  t.ground_radius = kR;
  t.landing_height = 30.0;
  t.engines.counts = {1U, 2U, 3U, 3U};
  t.phase[0].kind = kind::kLanding;
  t.phase[0].p[0] = 1.0F;
  t.phase[0].p[1] = 1.0F;
  t.phase[0].p[2] = 1.0F;
  t.phase[0].p[3] = 1.0F;
  Mission m;
  m.configure(&t);
  m.start(inputs_at(20000.0, Vec3{-200.0, 0.0, 0.0}));
  const Output idle = m.step(inputs_at(20000.0, Vec3{-200.0, 0.0, 0.0}));
  CHECK(idle.groups == 0U && idle.throttle == 0.0F && m.landing_out().ignite == false);
  const Output burning = m.step(inputs_at(30.0 + 40.0, Vec3{-200.0, 0.0, 0.0}));
  CHECK(burning.groups == 1U && burning.throttle > 0.0F);
  // a boost-back with the reserve already reached stops at the first look
  Tables b = plain(1);
  b.nav.gravity = g;
  b.site = Vec3{kR, 0.0, 0.0};
  b.ground_radius = kR;
  b.phase[0].kind = kind::kBoostback;
  b.phase[0].groups = 1U;
  b.phase[0].thrust = 1.0e7F;
  b.phase[0].mdot = 1000.0F;
  b.phase[0].p[1] = 200000.0F;
  b.mass0 = 100000.0;
  Mission bm;
  bm.configure(&b);
  bm.start(inputs_at(50000.0, Vec3{300.0, 1000.0, 0.0}));
  const Output o = bm.step(inputs_at(50000.0, Vec3{300.0, 1000.0, 0.0}));
  CHECK(bm.cutoff() && o.groups == 0U && o.throttle == 0.0F);
}

TFC_TEST(sequencer_a_phase_can_set_the_mass_estimate_when_it_begins) {
  Tables t = plain(2);
  t.phase[0].end = end::kTime;
  t.phase[0].end_value = 0.03F;
  t.phase[1].mass_set = 42000.0F;
  Mission m;
  m.configure(&t);
  m.start(inputs_at(0.0, Vec3{}));
  for (int i = 0; i < 6; ++i) {
    (void)m.step(inputs_at(0.0, Vec3{}));
  }
  CHECK(m.phase() == 1U && near_abs(m.mass(), 42000.0, 1e-9));
}

TFC_TEST(sequencer_odd_tables_and_states_are_tolerated) {
  // a full-length track is searched to its end, not past it
  GainTrack g;
  g.n = static_cast<uint8_t>(kGainPoints);
  ThrottleTrack tr;
  tr.n = static_cast<uint8_t>(kThrottlePoints);
  for (unsigned i = 0; i < kGainPoints; ++i) {
    g.frame[i] = 10U * (i + 1U);
    g.gains[i].pitch.kp = static_cast<float>(i);
    g.thrust[i] = static_cast<float>(i);
    tr.frame[i] = 10U * (i + 1U);
    tr.value[i] = 0.1F * static_cast<float>(i);
  }
  CHECK(near_abs(static_cast<double>(g.at(100000U).pitch.kp), 15.0, 1e-6) && near_abs(static_cast<double>(g.thrust_at(100000U)), 15.0, 1e-6));
  CHECK(near_abs(static_cast<double>(tr.at(100000U, 0.0F)), 1.5, 1e-6));
  g.n = 200U;   // more points than the table holds: read as full
  tr.n = 200U;
  CHECK(near_abs(static_cast<double>(g.at(100000U).pitch.kp), 15.0, 1e-6) && near_abs(static_cast<double>(tr.at(100000U, 0.0F)), 1.5, 1e-6));
  // tables that are not there, or have no phases
  Mission none;
  none.configure(nullptr);
  CHECK(!none.configured() && near_abs(none.mass(), 1.0, 1e-12));
  Tables empty;
  Mission m;
  m.configure(&empty);
  CHECK(!m.configured());
  m.set_state(Mission::State{0U, 0U, 5.0, true, false, false});   // a state taken from a peer, with nothing to run it on
  CHECK(m.step(inputs_at(0.0, Vec3{})).phase == 0U);
  // the last phase ends and there is nowhere to go: it stays
  Tables t = plain(1);
  t.phase[0].end = end::kTime;
  t.phase[0].end_value = 0.01F;
  Mission last;
  last.configure(&t);
  last.start(inputs_at(0.0, Vec3{}));
  for (int i = 0; i < 5; ++i) {
    (void)last.step(inputs_at(0.0, Vec3{}));
  }
  CHECK(last.phase() == 0U);
  // climbing through an altitude is not descending through it, and the other way
  CHECK(!ends_when(end::kAltitudeAbove, 500.0F, inputs_at(100.0, Vec3{50.0, 100.0, 0.0}), inputs_at(900.0, Vec3{-50.0, 100.0, 0.0})));
}

TFC_TEST(sequencer_a_burn_that_has_cut_off_stays_cut_off_and_a_prediction_that_fails_leaves_the_steering_alone) {
  const tfc::nav::Gravity g;
  // a PEG phase that is the last: after the cut-off it only holds the attitude
  Tables t = plain(1);
  t.nav.gravity = g;
  t.phase[0].kind = kind::kPeg;
  t.phase[0].end = end::kCutoff;
  t.phase[0].groups = 1U;
  t.phase[0].thrust = 1.0e6F;
  t.phase[0].mdot = 300.0F;
  t.phase[0].p[0] = static_cast<float>(kR + 100000.0);
  t.phase[0].p[1] = static_cast<float>(std::sqrt(g.mu / (kR + 100000.0)));
  t.phase[0].p[8] = 1000.0F;
  Mission m;
  m.configure(&t);
  m.set_design_mode(true);
  Inputs in = inputs_at(99999.0, Vec3{0.0, std::sqrt(g.mu / (kR + 99999.0)), 0.0});
  m.start(in);
  Output out;
  for (int k = 0; k < 3000 && !m.cutoff(); ++k) {
    out = m.step(in);
    in.q = out.q_ref;
  }
  CHECK(m.cutoff());
  const Output after = m.step(in);
  CHECK(after.groups == 0U && after.throttle == 0.0F && m.phase() == 0U);
  // a boost-back that is the last phase: after its cut-off it holds; and one whose prediction never reaches the ground (a state leaving the planet) does not steer
  Tables b = plain(1);
  b.nav.gravity = g;
  b.site = Vec3{kR, 0.0, 0.0};
  b.ground_radius = kR;
  b.phase[0].kind = kind::kBoostback;
  b.phase[0].groups = 1U;
  b.phase[0].thrust = 1.0e7F;
  b.phase[0].mdot = 1000.0F;
  b.mass0 = 100000.0;
  Mission bm;
  bm.configure(&b);
  Inputs esc = inputs_at(50000.0, Vec3{20000.0, 0.0, 0.0});
  bm.start(esc);
  const Output o1 = bm.step(esc);
  CHECK(!bm.cutoff() && o1.groups == 1U);
  Mission bc;
  bc.configure(&b);
  bc.set_mass(0.0);   // mass at or below the reserve (0): cut off at the first look
  bc.start(esc);
  (void)bc.step(esc);
  CHECK(bc.cutoff());
  (void)bc.step(esc);
  CHECK(bc.cutoff());
  // a glide whose prediction never reaches the ground keeps its last miss
  Tables gl = plain(1);
  gl.nav.gravity = g;
  gl.site = Vec3{kR, 0.0, 0.0};
  gl.phase[0].kind = kind::kGlide;
  gl.phase[0].p[0] = 10.0F;
  gl.phase[0].p[3] = 1.0F;
  gl.phase[0].p[4] = 300.0F;
  gl.phase[0].p[5] = 3000.0F;
  Mission gm;
  gm.configure(&gl);
  gm.start(esc);
  const Output og = gm.step(esc);
  CHECK(og.groups == 0U && norm(gm.miss()) == 0.0);
}

TFC_TEST(sequencer_a_belly_entry_holds_the_nose_raised_from_the_velocity_and_the_prediction_steers_the_angle_and_the_bank) {
  const tfc::nav::Gravity g;
  const auto belly = [&g](double alpha, double alpha_min, double alpha_max, double per_m, double beta, double bank_per_m, double bank_max) {
    Tables t = plain(1);
    t.nav.gravity = g;
    t.site = Vec3{kR, 0.0, 0.0};
    t.ground_radius = kR;
    t.phase[0].kind = kind::kBelly;
    t.phase[0].p[0] = static_cast<float>(alpha);
    t.phase[0].p[1] = static_cast<float>(alpha_min);
    t.phase[0].p[2] = static_cast<float>(alpha_max);
    t.phase[0].p[3] = static_cast<float>(per_m);
    t.phase[0].p[4] = static_cast<float>(beta);
    t.phase[0].p[5] = static_cast<float>(bank_per_m);
    t.phase[0].p[6] = static_cast<float>(bank_max);
    return t;
  };
  const Vec3 v{-50.0, 100.0, 0.0};
  const auto at = [&v](double y, double z) {
    Inputs in = inputs_at(30000.0, v);
    in.r = Vec3{kR + 30000.0, y, z};
    return in;
  };
  const Vec3 vel = tfc::dm::unit(v, Vec3{1.0, 0.0, 0.0});
  const auto angle_deg = [&vel](const Vec3& axis) { return std::acos(dot(axis, vel)) * tfc::dm::kRadToDeg; };
  const auto fly = [](const Tables& t, const Inputs& in) {
    Mission m;
    m.configure(&t);
    m.start(in);
    const Output out = m.step(in);
    CHECK(out.groups == 0U && out.throttle == 0.0F);   // no thrust: the surfaces do the steering
    return std::make_pair(axis_of(m), m.miss());
  };
  // without a gain the angle is the nominal one, raised toward the vertical (the +x of the site) across the velocity, and with no bank the lift is in the plane of the fall
  {
    const Tables t = belly(70.0, 50.0, 88.0, 0.0, 0.0, 0.0, 0.0);
    const auto [axis, miss] = fly(t, at(0.0, 0.0));
    CHECK(near_abs(angle_deg(axis), 70.0, 0.01) && axis.x > vel.x && near_abs(axis.z, 0.0, 1.0e-9));
    (void)miss;
  }
  // a miss short of the site lowers the angle to its smallest (the most lift), a miss beyond it raises it to its largest
  {
    const Tables t = belly(70.0, 50.0, 88.0, 1.0, 0.0, 0.0, 0.0);
    CHECK(near_abs(angle_deg(fly(t, at(-1.0e6, 0.0)).first), 50.0, 0.01));
    CHECK(near_abs(angle_deg(fly(t, at(1.0e6, 0.0)).first), 88.0, 0.01));
  }
  // the ballistic coefficient of the phase replaces the tables' in the prediction: a lighter fall comes down short of where a heavier one does
  {
    const Tables light = belly(70.0, 50.0, 88.0, 0.0, 50.0, 0.0, 0.0);
    const Tables heavy = belly(70.0, 50.0, 88.0, 0.0, 5000.0, 0.0, 0.0);
    CHECK(fly(light, at(0.0, 0.0)).second.y != fly(heavy, at(0.0, 0.0)).second.y);
  }
  // a miss across the track banks the whole attitude about the velocity toward the site, up to the most it may bank
  {
    const Tables t = belly(70.0, 50.0, 88.0, 0.0, 0.0, 1.0, 20.0);
    const Vec3 right = fly(t, at(0.0, 5000.0)).first;     // the vehicle is to the +z of the track: the lift tilts to -z
    const Vec3 left = fly(t, at(0.0, -5000.0)).first;
    CHECK(near_abs(right.z, -std::sin(70.0 * tfc::dm::kDegToRad) * std::sin(20.0 * tfc::dm::kDegToRad), 1.0e-6));
    CHECK(near_abs(left.z, -right.z, 1.0e-6));
    CHECK(near_abs(angle_deg(right), 70.0, 0.01));      // the bank turns the lift, it does not change the angle
    const Tables soft = belly(70.0, 50.0, 88.0, 0.0, 0.0, 1.0e-4, 20.0);
    const Vec3 small = fly(soft, at(0.0, 5000.0)).first;
    CHECK(small.z < 0.0 && small.z > right.z);            // below the limit the bank follows the miss
  }
  // a state that never reaches the ground has no miss and the angle stays nominal
  {
    const Tables t = belly(70.0, 50.0, 88.0, 1.0, 0.0, 1.0, 20.0);
    Inputs esc = inputs_at(50000.0, Vec3{20000.0, 0.0, 0.0});
    Mission m;
    m.configure(&t);
    m.start(esc);
    (void)m.step(esc);
    CHECK(norm(m.miss()) == 0.0);
  }
}

TFC_TEST(sequencer_a_program_phase_rolls_a_little_out_and_back_when_it_is_asked_to) {
  const auto roll_at = [](double frames, double peak, double start, double duration) {
    Tables t = plain(1);
    t.plane_normal = Vec3{0.0, 0.0, 1.0};
    t.phase[0].kind = kind::kProgram;
    t.phase[0].p[0] = static_cast<float>(peak);
    t.phase[0].p[1] = static_cast<float>(start);
    t.phase[0].p[2] = static_cast<float>(duration);
    Mission m;
    m.configure(&t);
    const Inputs in = inputs_at(100.0, Vec3{});
    m.start(in);
    for (int i = 0; i < static_cast<int>(frames); ++i) {
      (void)m.step(in);
    }
    const Vec3 z = tfc::dm::rotate(m.target(), Vec3{0.0, 0.0, 1.0});
    return std::acos(std::fmin(1.0, std::fmax(-1.0, dot(z, Vec3{0.0, 0.0, 1.0})))) * tfc::dm::kRadToDeg;   // how far the body's z has turned from the plane's normal: the roll
  };
  CHECK(near_abs(roll_at(50.0, 20.0, 1.0, 4.0), 0.0, 1e-3));      // before it starts
  CHECK(near_abs(roll_at(300.0, 20.0, 1.0, 4.0), 20.0, 0.3));     // at the middle of it: the peak
  CHECK(roll_at(200.0, 20.0, 1.0, 4.0) > 5.0 && roll_at(200.0, 20.0, 1.0, 4.0) < 20.0);   // on the way
  CHECK(near_abs(roll_at(600.0, 20.0, 1.0, 4.0), 0.0, 1e-3));     // and back, with nothing left of it
  CHECK(near_abs(roll_at(300.0, 20.0, 1.0, 0.0), 0.0, 1e-3));     // no duration: no roll
}

// Verifies: TFC-SIM-020
TFC_TEST(sequencer_a_turning_planet_carries_the_site_and_the_guidance_aims_at_where_it_will_be) {
  const tfc::nav::Gravity g;
  const double w = 7.2921159e-5;
  const auto belly = [&g, w](double omega) {
    Tables t = plain(1);
    t.nav.gravity = g;
    t.site = Vec3{kR, 0.0, 0.0};
    t.ground_radius = kR;
    t.pole = Vec3{0.0, 0.0, 1.0};
    t.planet_omega = omega;
    t.drag.omega = t.pole * omega;
    t.phase[0].kind = kind::kBelly;
    t.phase[0].p[0] = 70.0F;
    t.phase[0].p[1] = 50.0F;
    t.phase[0].p[2] = 88.0F;
    (void)w;
    return t;
  };
  // the site goes round the pole at the planet's rate, and keeps its distance from the centre; a planet that stands still leaves it where it is
  {
    const Tables t = belly(w);
    Mission m;
    m.configure(&t);
    Inputs in = inputs_at(30000.0, Vec3{-50.0, 100.0, 0.0});
    in.epoch_s = 1000.0;
    m.start(in);
    const Vec3 s = m.site_at(0.0);
    const double a = w * 1000.0;
    CHECK(near_abs(s.x, kR * std::cos(a), 1.0e-3) && near_abs(s.y, kR * std::sin(a), 1.0e-3) && near_abs(s.z, 0.0, 1.0e-6));
    const Vec3 later = m.site_at(500.0);
    CHECK(near_abs(norm(later), kR, 1.0e-3) && near_abs(later.y, kR * std::sin(w * 1500.0), 1.0e-3));
    // the ground moves at the planet's turn where the vehicle is: east at 465 m/s at the equator
    const Vec3 gv = m.ground_velocity(Vec3{kR, 0.0, 0.0});
    CHECK(near_abs(gv.y, w * kR, 1.0e-6) && near_abs(gv.x, 0.0, 1.0e-9));
    // the catch point is the site raised by the centre of gravity's height
    const Tables h = [&belly] { Tables x = belly(7.2921159e-5); x.landing_height = 30.0; return x; }();
    Mission mh;
    mh.configure(&h);
    mh.start(in);
    CHECK(near_abs(norm(mh.catch_point_at(0.0)), kR + 30.0, 1.0e-6));
    const Tables still = belly(0.0);
    Mission ms;
    ms.configure(&still);
    ms.start(in);
    CHECK(norm(ms.site_at(500.0) - still.site) == 0.0);
    CHECK(norm(ms.ground_velocity(Vec3{kR, 0.0, 0.0})) == 0.0);
  }
  // a fall that ends a few minutes from now is aimed at the place the pad will be then, hundreds of kilometres from where it is: the miss moves by the pad's travel
  {
    const auto miss_y = [](const Tables& t) {
      Inputs in = inputs_at(30000.0, Vec3{-50.0, 100.0, 0.0});
      Mission m;
      m.configure(&t);
      m.start(in);
      (void)m.step(in);
      return m.miss().y;
    };
    const double fixed = miss_y(belly(0.0));
    const double turning = miss_y(belly(w));
    CHECK(turning > fixed + 1.0e4 && turning < fixed + 3.0e5);   // (a pad that moves east, with the vehicle's fall going east: the site is farther on)
  }
  // flown relative to the ground: a landing over the pad with the pad's own velocity is the landing over a pad that stands still
  {
    const auto land = [&g](double omega) {
      Tables t = plain(1);
      t.nav.gravity = g;
      t.site = Vec3{kR, 0.0, 0.0};
      t.ground_radius = kR;
      t.landing_height = 30.0;
      t.engines.counts = {1U, 2U, 3U, 3U};
      t.pole = Vec3{0.0, 0.0, 1.0};
      t.planet_omega = omega;
      t.phase[0].kind = kind::kLanding;
      t.phase[0].p[0] = 1.0F;
      t.phase[0].p[1] = 1.0F;
      t.phase[0].p[2] = 1.0F;
      t.phase[0].p[3] = 1.0F;
      return t;
    };
    const Tables still = land(0.0);
    const Tables turning = land(w);
    Inputs a = inputs_at(400.0, Vec3{-80.0, 20.0, 0.0});
    Inputs b = a;
    b.v = b.v + Vec3{0.0, w * (kR + 400.0), 0.0};
    Mission ma;
    ma.configure(&still);
    ma.start(a);
    (void)ma.step(a);
    Mission mb;
    mb.configure(&turning);
    mb.start(b);
    (void)mb.step(b);
    CHECK(norm(ma.landing_out().direction - mb.landing_out().direction) < 1.0e-9 && near_abs(ma.landing_out().throttle, mb.landing_out().throttle, 1.0e-9));
  }
}

// ---- the engine health word (ADR-058) ----

namespace {
// Tables whose group 0 lights engines 0 to 2 and whose group 1 lights engines 3 to 6, each worth 2.2 MN (engine 3 worth twice that), for a landing with the options 3, 3, 3 and 7 engines.
Tables engines_named() {
  const tfc::nav::Gravity g;
  Tables t = plain(1);
  t.nav.gravity = g;
  t.site = Vec3{kR, 0.0, 0.0};
  t.ground_radius = kR;
  t.landing_height = 30.0;
  t.engines.counts = {3U, 3U, 3U, 7U};
  t.engines.options = 4U;
  t.engines.thrust_each = 2.2e6;
  t.group_engines[0] = 0x07U;
  t.group_engines[1] = 0x78U;
  for (unsigned e = 0; e < 7U; ++e) {
    t.engine_thrust[e] = e == 3U ? 4.4e6F : 2.2e6F;
  }
  t.phase[0].kind = kind::kLanding;
  t.phase[0].p[0] = 1.0F;   // group 0: three engines
  t.phase[0].p[1] = 1.0F;
  t.phase[0].p[2] = 1.0F;
  t.phase[0].p[3] = 3.0F;   // groups 0 and 1: seven
  return t;
}
}  // namespace

// Verifies: TFC-SIM-022
TFC_TEST(sequencer_the_engines_declared_out_take_their_share_of_the_thrust_the_flow_and_the_mass_the_guidance_counts_on) {
  Tables t = plain(1);
  t.mass0 = 100000.0;
  t.phase[0].kind = kind::kProgram;
  t.phase[0].groups = 1U;
  t.phase[0].throttle = 1.0F;
  t.phase[0].thrust = 6.6e6F;
  t.phase[0].mdot = 100.0F;
  t.group_engines[0] = 0x0FU;
  t.engine_thrust = {2.2e6F, 2.2e6F, 2.2e6F, 4.4e6F};   // (an engine worth twice the others)
  const auto burn = [&t](uint64_t out) {
    Mission m;
    m.configure(&t);
    Inputs in = inputs_at(1000.0, Vec3{});
    in.engine_out = out;
    m.start(in);
    Output o;
    for (unsigned i = 0; i < 100U; ++i) {
      o = m.step(in);
    }
    return std::make_pair(m.mass(), o.thrust);
  };
  const auto healthy = burn(0U);
  CHECK(near_abs(healthy.first, 100000.0 - 100.0, 1.0e-6) && near_abs(static_cast<double>(healthy.second), 6.6e6, 1.0));
  const auto lost_big = burn(0x08U);   // the big engine is out: 4.4 of 11 MN, so 60 % of the thrust and the flow is left
  CHECK(near_abs(lost_big.first, 100000.0 - 60.0, 1.0e-6) && near_abs(static_cast<double>(lost_big.second), 0.6 * 6.6e6, 1.0));
  const auto lost_small = burn(0x01U);
  CHECK(near_abs(lost_small.first, 100000.0 - 80.0, 1.0e-6));
  // tables that name no engines are not affected by the word
  Tables none = plain(1);
  none.mass0 = 100000.0;
  none.phase[0].kind = kind::kProgram;
  none.phase[0].groups = 1U;
  none.phase[0].throttle = 1.0F;
  none.phase[0].mdot = 100.0F;
  Mission m;
  m.configure(&none);
  Inputs in = inputs_at(1000.0, Vec3{});
  in.engine_out = ~uint64_t{0};
  m.start(in);
  for (unsigned i = 0; i < 100U; ++i) {
    (void)m.step(in);
  }
  CHECK(near_abs(m.mass(), 100000.0 - 100.0, 1.0e-6));
  // the share and the counts as the sequencer reports them
  Mission r;
  const Tables named = engines_named();
  r.configure(&named);
  Inputs gone = inputs_at(1000.0, Vec3{});
  gone.engine_out = 0x09U;   // engines 0 and 3
  r.start(gone);
  (void)r.step(gone);
  CHECK(r.engines_in(0x03U, false) == 7U && r.engines_in(0x03U, true) == 5U && r.engines_in(0x01U, true) == 2U);
  CHECK(near_abs(r.health_of(0x01U), 2.0 / 3.0, 1.0e-9) && near_abs(r.health_of(0x02U), 0.6, 1.0e-9) && r.health_of(0x80U) == 1.0);
}

// Verifies: TFC-SIM-022
TFC_TEST(sequencer_a_landing_burn_with_engines_out_runs_the_number_of_engines_it_asks_for_from_the_pool_and_counts_what_is_left) {
  const Tables t = engines_named();
  const auto burn = [&t](uint64_t out) {
    Mission m;
    m.configure(&t);
    Inputs in = inputs_at(60.0, Vec3{-30.0, 0.0, 0.0});   // 30 m above the catch point at 30 m/s: due, and three engines can do it
    in.engine_out = out;
    m.start(in);
    (void)m.step(in);
    const Output o = m.step(in);
    return std::make_tuple(o.groups, o.engines, m.landing_out().engines, m.landing_pool());
  };
  // nothing out: the option's own groups and no count (every engine of them), as before
  const auto ok = burn(0U);
  CHECK(std::get<0>(ok) == 1U && std::get<1>(ok) == 0U && std::get<2>(ok) == 3U && std::get<3>(ok) == 3U);
  // an engine of the option out: the pool of every group of every option, three engines asked for (the controller takes the healthy ones)
  const auto one = burn(0x01U);
  CHECK(std::get<0>(one) == 3U && std::get<1>(one) == 3U && std::get<2>(one) == 3U);
  // an engine out that the option does not light changes nothing
  const auto elsewhere = burn(0x20U);
  CHECK(std::get<0>(elsewhere) == 1U && std::get<1>(elsewhere) == 0U);
  // only two engines left in the pool: no option can run more
  const auto two = burn(0x1FU);   // engines 0 to 4 out: two left
  CHECK(std::get<2>(two) == 2U);
  CHECK(std::get<2>(burn(0x7FU)) == 1U);   // none left: one is counted, so that the throttle is a number
}

// Verifies: TFC-SIM-022
TFC_TEST(sequencer_the_burn_is_lit_earlier_and_planned_gentler_when_the_engines_left_decelerate_less) {
  const auto setup = [](Tables& t) {
    t.landing_height = 40.0;
    t.landing.decel_plan = 20.0;
    t.landing.sink_ms = 0.5;
    t.landing.aim_below_m = 2.0;
    t.ignition_margin = 1.0;
    t.engines.thrust_each = 1.0e6;   // one engine's thrust over the mass of 100 t is 10 m/s^2: 0.95 of it is less than gravity
    t.engines.counts = {1U, 1U, 1U, 1U};
    t.engines.options = 4U;
  };
  // at 5000 m and 100 m/s with engines that give 11 m/s^2 net of the weight the plan is that and it stops in 450 m: not yet. With engines that cannot even hold the weight the plan is the floor of 1 m/s^2: the stopping height is the whole 5000 m
  CHECK(!ends_when(end::kIgnition, 6000.0F, inputs_at(8000.0, Vec3{-100.0, 0.0, 0.0}), inputs_at(4900.0, Vec3{-100.0, 0.0, 0.0}), 100000.0, 3U, [](Tables& t) {
    t.landing_height = 40.0;
    t.landing.decel_plan = 20.0;
    t.landing.sink_ms = 0.5;
    t.landing.aim_below_m = 2.0;
    t.ignition_margin = 1.0;
    t.engines.thrust_each = 2.2e6;   // plenty
    t.engines.counts = {1U, 1U, 1U, 1U};
    t.engines.options = 4U;
  }));
  CHECK(ends_when(end::kIgnition, 6000.0F, inputs_at(8000.0, Vec3{-100.0, 0.0, 0.0}), inputs_at(4900.0, Vec3{-100.0, 0.0, 0.0}), 100000.0, 3U, setup));
}
