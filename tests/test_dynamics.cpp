// SPDX-License-Identifier: MIT
// The dynamics beyond a rigid body on a first-order servo (docs/design/DYNAMICS.md): a second-order gimbal servo and its backlash, jet damping, what a stage's separation does to the vehicle
// that stays, and the vehicle's own roll controller. Each is checked against an answer that does not come from the code: the overshoot and the peak time of a second-order step response,
// the play of a backlash worked by hand, the decay rate of a turning vehicle's exhaust damping (md l^2 / I), the speed and the rates a separation hands on, and a roll controller's
// exponential decay and its undamped period. With every setting at its default none of it changes anything.
#include <array>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

#include "tfc_test.hpp"

#include "closed_loop.hpp"
#include "spec_io.hpp"
#include "vehicle6.hpp"

namespace {

bool close(double a, double b, double rel) { return std::fabs(a - b) <= rel * std::fmax(std::fabs(a), std::fabs(b)); }
bool near_abs(double a, double b, double tol) { return std::fabs(a - b) <= tol; }

// A bare stage of 1000 kg, 4 m long and 0.5 m in radius, in the dark and out of gravity, with room for every limit.
sim::Params bare() {
  sim::Params p;
  p.ground_contact = false;
  p.gravity_scale = 0.0;
  p.gimbal_limit_deg = 45.0;
  p.gimbal_rate_dps = 10000.0;
  sim::StageSpec st;
  st.name = "body";
  st.dry_mass = 1000.0;
  st.length = 4.0;
  st.radius = 0.5;
  st.x_cg_dry = 2.0;
  p.spec.stages.push_back(st);
  p.spec.planet.atmosphere = sim::AtmosphereKind::None;
  return p;
}

}  // namespace

// ---- the servo ----

TFC_TEST(dynamics_a_second_order_servo_overshoots_and_peaks_as_the_formulas_say) {
  const double zeta = 0.3;
  const double fn = 5.0;
  sim::Params p = bare();
  p.spec.actuator.order = 2;
  p.spec.actuator.natural_hz = fn;
  p.spec.actuator.damping = zeta;
  sim::Vehicle6 v(p);
  double peak = 0.0;
  double t_peak = 0.0;
  for (int k = 1; k <= 1000; ++k) {  // one second in milliseconds
    v.step(0.001, 1.0, 0.0);
    if (v.gimbal_pitch_deg() > peak) {
      peak = v.gimbal_pitch_deg();
      t_peak = 0.001 * k;
    }
  }
  const double wn = 2.0 * 3.14159265358979323846 * fn;
  const double wd = wn * std::sqrt(1.0 - (zeta * zeta));
  const double overshoot = std::exp(-3.14159265358979323846 * zeta / std::sqrt(1.0 - (zeta * zeta)));  // 37.2 %
  CHECK(near_abs(peak, 1.0 + overshoot, 0.002));
  CHECK(near_abs(t_peak, 3.14159265358979323846 / wd, 0.002));  // 0.1048 s
  // and the whole curve against the closed form, theta(t) = 1 - exp(-zeta wn t) (cos wd t + (zeta wn / wd) sin wd t), in both planes at once
  sim::Vehicle6 c(p);
  for (int k = 1; k <= 300; ++k) {
    c.step(0.001, 1.0, -1.0);
    if (k % 50 == 0) {
      const double t = 0.001 * k;
      const double exact = 1.0 - (std::exp(-zeta * wn * t) * (std::cos(wd * t) + ((zeta * wn / wd) * std::sin(wd * t))));
      CHECK(near_abs(c.gimbal_pitch_deg(), exact, 1e-6) && near_abs(c.gimbal_yaw_deg(), -exact, 1e-6));
    }
  }
  for (int k = 0; k < 2000; ++k) {
    v.step(0.001, 1.0, 0.0);
  }
  CHECK(near_abs(v.gimbal_pitch_deg(), 1.0, 1e-3));  // and settles on the command
  CHECK(v.gimbal_yaw_deg() == 0.0);                  // the other plane is untouched
}

TFC_TEST(dynamics_a_second_order_servo_obeys_its_rate_and_travel_limits_and_loses_its_speed_at_a_stop) {
  sim::Params p = bare();
  p.gimbal_rate_dps = 20.0;
  p.gimbal_limit_deg = 2.0;
  p.spec.actuator.order = 2;
  p.spec.actuator.natural_hz = 20.0;
  p.spec.actuator.damping = 0.5;
  sim::Vehicle6 v(p);
  double last = 0.0;
  double fastest = 0.0;
  for (int k = 0; k < 500; ++k) {
    v.step(0.001, 5.0, -5.0);  // asks for far more than the travel
    fastest = std::fmax(fastest, std::fabs(v.gimbal_pitch_deg() - last) / 0.001);
    last = v.gimbal_pitch_deg();
  }
  CHECK(fastest <= 20.0 + 1e-6);                                   // the rate limit
  CHECK(v.gimbal_pitch_deg() == 2.0 && v.gimbal_yaw_deg() == -2.0);  // the travel limit, both ways, held
  // a command beyond the travel is the same as one at the travel: the servo is driven toward the stop, not toward the command
  {
    sim::Vehicle6 far(p);
    sim::Vehicle6 near(p);
    for (int k = 0; k < 60; ++k) {
      far.step(0.001, 50.0, -50.0);
      near.step(0.001, 2.0, -2.0);
      CHECK(far.gimbal_pitch_deg() == near.gimbal_pitch_deg() && far.gimbal_yaw_deg() == near.gimbal_yaw_deg());
    }
  }
  // against a stop the servo loses its speed: an under-damped one overshoots into the stop, and the moment it arrives there a command the other way moves it off at once (a servo that kept
  // the speed it arrived with would stay pinned until its own damping had taken that speed away)
  sim::Params ring = p;
  ring.spec.actuator.natural_hz = 2.0;
  ring.spec.actuator.damping = 0.1;
  ring.gimbal_rate_dps = 1000.0;
  sim::Vehicle6 u(ring);
  int k = 0;
  while (u.gimbal_pitch_deg() != 2.0 && k < 3000) {
    u.step(0.001, 5.0, -5.0);
    ++k;
  }
  CHECK(k < 3000 && u.gimbal_pitch_deg() == 2.0 && u.gimbal_yaw_deg() == -2.0);  // both planes hit their stops on the same step (they are the same servo mirrored)
  u.step(0.001, -5.0, 5.0);
  CHECK(u.gimbal_pitch_deg() < 2.0 && u.gimbal_yaw_deg() > -2.0);
}

TFC_TEST(dynamics_the_rate_limit_stops_a_second_order_servo_from_winding_up_its_speed) {
  sim::Params p = bare();
  p.gimbal_rate_dps = 20.0;
  p.spec.actuator.order = 2;
  p.spec.actuator.natural_hz = 20.0;
  p.spec.actuator.damping = 0.3;
  sim::Vehicle6 v(p);
  double peak = 0.0;
  double fastest = 0.0;
  double last = 0.0;
  for (int k = 0; k < 2000; ++k) {
    v.step(0.001, 1.0, 0.0);
    peak = std::fmax(peak, v.gimbal_pitch_deg());
    fastest = std::fmax(fastest, std::fabs(v.gimbal_pitch_deg() - last) / 0.001);
    last = v.gimbal_pitch_deg();
  }
  CHECK(fastest <= 20.0 + 1e-6 && near_abs(v.gimbal_pitch_deg(), 1.0, 1e-3));
  // The overshoot of a rate-limited servo has no closed form; this is the measured one (1.0881) and it is held: the speed the servo is allowed is the rate limit, so that it arrives at the
  // command with that speed and not with the speed it would have built up had nothing held it (the same run with the velocity left to grow overshoots to 1.190).
  CHECK(peak > 1.083 && peak < 1.093);
  // the limit is on the speed, not only on the step: with no rate limit the same servo's peak is the closed form's 1 + exp(-pi zeta / sqrt(1 - zeta^2))
  sim::Params q = p;
  q.gimbal_rate_dps = 1.0e6;
  sim::Vehicle6 w(q);
  double peak_free = 0.0;
  for (int k = 0; k < 500; ++k) {
    w.step(0.001, 1.0, 0.0);
    peak_free = std::fmax(peak_free, w.gimbal_pitch_deg());
  }
  CHECK(near_abs(peak_free, 1.0 + std::exp(-3.14159265358979323846 * 0.3 / std::sqrt(1.0 - (0.3 * 0.3))), 0.002));
}

TFC_TEST(dynamics_backlash_holds_the_engine_until_the_servo_has_taken_up_the_play) {
  sim::Params p = bare();
  p.spec.actuator.backlash_deg = 1.0;  // order 1 with no lag: the servo is at the command
  sim::Vehicle6 v(p);
  v.step(0.001, 0.4, 0.0);
  CHECK(v.gimbal_pitch_deg() == 0.0);  // 0.4 is inside the half play of 0.5: the engine has not moved
  v.step(0.001, 0.5, 0.0);
  CHECK(v.gimbal_pitch_deg() == 0.0);  // just at the edge
  v.step(0.001, 2.0, 0.0);
  CHECK(near_abs(v.gimbal_pitch_deg(), 1.5, 1e-12));  // the servo at 2.0, the engine half the play behind it
  v.step(0.001, 1.2, 0.0);
  CHECK(near_abs(v.gimbal_pitch_deg(), 1.5, 1e-12));  // the command comes back by 0.8: still inside the play, the engine does not move
  v.step(0.001, 0.0, 0.0);
  CHECK(near_abs(v.gimbal_pitch_deg(), 0.5, 1e-12));  // the servo at 0: the engine half the play on the other side
  CHECK(v.gimbal_yaw_deg() == 0.0);
  // the yaw plane has the same play (a separate servo, the same rules)
  sim::Vehicle6 w(p);
  w.step(0.001, 0.0, -0.4);
  CHECK(w.gimbal_yaw_deg() == 0.0);
  w.step(0.001, 0.0, -2.0);
  CHECK(near_abs(w.gimbal_yaw_deg(), -1.5, 1e-12) && w.gimbal_pitch_deg() == 0.0);
}

TFC_TEST(dynamics_a_servo_with_dynamics_can_carry_both_a_second_order_response_and_the_play) {
  sim::Params p = bare();
  p.spec.actuator.order = 2;
  p.spec.actuator.natural_hz = 10.0;
  p.spec.actuator.damping = 1.0;
  p.spec.actuator.backlash_deg = 0.4;
  sim::Vehicle6 v(p);
  for (int k = 0; k < 3000; ++k) {
    v.step(0.001, 1.0, 0.0);
  }
  CHECK(near_abs(v.gimbal_pitch_deg(), 0.8, 1e-3));  // settled: the servo at 1.0, the engine 0.2 behind it
}

// ---- jet damping ----

TFC_TEST(dynamics_jet_damping_slows_a_turning_vehicle_by_the_exhaust_it_throws_off) {
  const double w0 = 0.05;  // rad/s about the pitch axis (body Z)
  const auto turned = [&](bool jet) {
    sim::Params p = bare();
    p.spec.jet_damping = jet;
    p.spec.stages[0].tanks.push_back(sim::TankSpec{2000.0, 0.5, 0.5, 900.0, {}});
    sim::EngineSpec e;
    e.thrust_vac = 200000.0;
    e.exit_area = 0.0;
    e.isp_vac = 300.0;
    p.spec.engines.push_back(e);
    sim::Scenario sc;
    sc.has_initial = true;
    sc.initial.r = sim::V3{sim::kEarthR + 100000.0, 0.0, 0.0};
    sc.initial.w = sim::V3{0.0, 0.0, w0};
    sim::Vehicle6 v(p, sc);
    const sim::MassProps mp = v.mass_props();
    for (int k = 0; k < 200; ++k) {
      v.step(0.001, 0.0, 0.0);
    }
    struct R {
      double wz;
      sim::MassProps mp;
    };
    return R{v.state().w.z, mp};
  };
  const auto on = turned(true);
  const auto off = turned(false);
  const double md = 200000.0 / (300.0 * 9.80665);  // kg/s
  const double l = 0.0 - on.mp.x_cg;                 // the engine at the aft end
  const double expected = -md * l * l * w0 / on.mp.i_t * 0.2;  // the change over 0.2 s (the rate hardly changes in it)
  CHECK(near_abs(on.wz - off.wz, expected, 0.02 * std::fabs(expected)));
  CHECK(on.wz < off.wz);                                  // damping
  CHECK(off.wz > w0);                                     // without it the vehicle spins up as it loses inertia: the exhaust is taken to leave with no angular momentum, which is what jet damping corrects
}

TFC_TEST(dynamics_jet_damping_off_by_default_changes_nothing_and_acts_only_on_a_turning_vehicle) {
  sim::Params p = bare();
  p.spec.stages[0].tanks.push_back(sim::TankSpec{2000.0, 0.5, 0.5, 900.0, {}});
  sim::EngineSpec e;
  e.thrust_vac = 200000.0;
  e.exit_area = 0.0;
  p.spec.engines.push_back(e);
  sim::Params q = p;
  q.spec.jet_damping = true;
  sim::Vehicle6 a(p);
  sim::Vehicle6 b(q);
  for (int k = 0; k < 500; ++k) {
    a.step(0.001, 0.0, 0.0);
    b.step(0.001, 0.0, 0.0);
  }
  CHECK(a.state().w.z == 0.0 && b.state().w.z == 0.0);  // nothing turning: nothing to damp
  CHECK(a.state().v.x == b.state().v.x);
}

// ---- separation ----

TFC_TEST(dynamics_a_separation_pushes_the_vehicle_that_stays_and_leaves_it_turning) {
  sim::Params p = bare();
  p.spec.stages[0].name = "lower";
  p.spec.stages[0].separate_time_s = 1.0;
  p.spec.stages[0].separation_dv_ms = 3.0;
  p.spec.stages[0].tipoff_pitch_dps = 2.0;
  p.spec.stages[0].tipoff_yaw_dps = -1.0;
  p.spec.stages[0].tipoff_roll_dps = 0.5;
  sim::StageSpec up;
  up.name = "upper";
  up.dry_mass = 500.0;
  up.x_start = 4.0;
  up.length = 2.0;
  up.radius = 0.4;
  p.spec.stages.push_back(up);
  sim::Scenario sc;
  sc.has_initial = true;
  sc.initial.r = sim::V3{sim::kEarthR + 100000.0, 0.0, 0.0};
  sim::Vehicle6 v(p, sc);
  for (int k = 0; k < 900; ++k) {
    v.step(0.001, 0.0, 0.0);
  }
  CHECK(v.state().v.x == 0.0 && sim::norm(v.state().w) == 0.0);  // before: at rest
  for (int k = 0; k < 200; ++k) {
    v.step(0.001, 0.0, 0.0);
  }
  CHECK(v.stage_active(1U) && !v.stage_active(0U));
  CHECK(near_abs(v.state().v.x, 3.0, 1e-12) && v.state().v.y == 0.0 && v.state().v.z == 0.0);
  const double d2r = 3.14159265358979323846 / 180.0;
  CHECK(near_abs(v.state().w.z, 2.0 * d2r, 1e-12) && near_abs(v.state().w.y, -1.0 * d2r, 1e-12));
  // the roll rate is held at zero by the reference model's ideal roll control, which the default keeps
  CHECK(v.state().w.x == 0.0);
  // a clean separation (the defaults) hands nothing on
  sim::Params q = p;
  q.spec.stages[0].separation_dv_ms = 0.0;
  q.spec.stages[0].tipoff_pitch_dps = 0.0;
  q.spec.stages[0].tipoff_yaw_dps = 0.0;
  q.spec.stages[0].tipoff_roll_dps = 0.0;
  sim::Vehicle6 w(q, sc);
  for (int k = 0; k < 1100; ++k) {
    w.step(0.001, 0.0, 0.0);
  }
  CHECK(w.state().v.x == 0.0 && sim::norm(w.state().w) == 0.0);
}

TFC_TEST(dynamics_the_push_of_a_separation_is_along_the_vehicles_own_axis_wherever_it_points) {
  sim::Params p = bare();
  p.spec.stages[0].separate_time_s = 0.5;
  p.spec.stages[0].separation_dv_ms = 3.0;
  sim::StageSpec up;
  up.name = "upper";
  up.dry_mass = 500.0;
  up.x_start = 4.0;
  up.length = 2.0;
  up.radius = 0.4;
  p.spec.stages.push_back(up);
  sim::Scenario sc;
  sc.has_initial = true;
  sc.initial.r = sim::V3{sim::kEarthR + 100000.0, 0.0, 0.0};
  sc.initial.q = sim::from_axis_angle(sim::V3{0.0, 0.0, 1.0}, 0.5 * 3.14159265358979323846);  // the long axis along the inertial Y
  sim::Vehicle6 v(p, sc);
  for (int k = 0; k < 600; ++k) {
    v.step(0.001, 0.0, 0.0);
  }
  CHECK(near_abs(v.state().v.y, 3.0, 1e-9) && near_abs(v.state().v.x, 0.0, 1e-9) && near_abs(v.state().v.z, 0.0, 1e-9));
}

TFC_TEST(dynamics_each_tipoff_rate_goes_to_its_own_axis_and_a_roll_that_is_held_has_none_to_hand_on) {
  const double d2r = 3.14159265358979323846 / 180.0;
  for (int axis = 0; axis < 3; ++axis) {
    sim::Params p = bare();
    p.ideal_roll_control = false;
    p.spec.stages[0].separate_time_s = 0.5;
    (axis == 0 ? p.spec.stages[0].tipoff_roll_dps : (axis == 1 ? p.spec.stages[0].tipoff_yaw_dps : p.spec.stages[0].tipoff_pitch_dps)) = 1.5;
    sim::StageSpec up;
    up.name = "upper";
    up.dry_mass = 500.0;
    up.x_start = 4.0;
    up.length = 2.0;
    up.radius = 0.4;
    p.spec.stages.push_back(up);
    sim::Scenario sc;
    sc.has_initial = true;
    sc.initial.r = sim::V3{sim::kEarthR + 100000.0, 0.0, 0.0};
    sim::Vehicle6 v(p, sc);
    for (int k = 0; k < 501; ++k) {
      v.step(0.001, 0.0, 0.0);
    }
    const sim::V3 w = v.state().w;
    CHECK(near_abs(w.x, axis == 0 ? 1.5 * d2r : 0.0, 1e-12) && near_abs(w.y, axis == 1 ? 1.5 * d2r : 0.0, 1e-12) && near_abs(w.z, axis == 2 ? 1.5 * d2r : 0.0, 1e-12));
  }
  // with the roll held ideally (the default) a roll tip-off is dropped at the release, and the pitch and yaw rates are exactly what was given
  sim::Params p = bare();
  p.spec.stages[0].separate_time_s = 0.5;
  p.spec.stages[0].tipoff_roll_dps = 5.0;
  sim::StageSpec up;
  up.name = "upper";
  up.dry_mass = 500.0;
  up.x_start = 4.0;
  up.length = 2.0;
  up.radius = 0.4;
  p.spec.stages.push_back(up);
  sim::Scenario sc;
  sc.has_initial = true;
  sc.initial.r = sim::V3{sim::kEarthR + 100000.0, 0.0, 0.0};
  sim::Vehicle6 v(p, sc);
  for (int k = 0; k < 501; ++k) {
    v.step(0.001, 0.0, 0.0);
  }
  CHECK(v.state().w.x == 0.0 && v.state().w.y == 0.0 && v.state().w.z == 0.0);
}

TFC_TEST(dynamics_a_roll_only_tipoff_is_handed_on_as_a_roll_rate_when_the_roll_is_not_held) {
  sim::Params p = bare();
  p.ideal_roll_control = false;
  p.spec.stages[0].separate_time_s = 0.5;
  p.spec.stages[0].tipoff_roll_dps = 4.0;
  sim::StageSpec up;
  up.name = "upper";
  up.dry_mass = 500.0;
  up.x_start = 4.0;
  up.length = 2.0;
  up.radius = 0.4;
  p.spec.stages.push_back(up);
  sim::Scenario sc;
  sc.has_initial = true;
  sc.initial.r = sim::V3{sim::kEarthR + 100000.0, 0.0, 0.0};
  sim::Vehicle6 v(p, sc);
  for (int k = 0; k < 600; ++k) {
    v.step(0.001, 0.0, 0.0);
  }
  CHECK(near_abs(v.state().w.x, 4.0 * 3.14159265358979323846 / 180.0, 1e-12));
  CHECK(near_abs(v.state().roll, 4.0 * 3.14159265358979323846 / 180.0 * 0.1, 2e-5));  // and has turned through that rate times the 0.1 s since
}

// ---- the roll controller ----

TFC_TEST(dynamics_a_roll_controller_brings_a_roll_rate_down_exponentially_and_without_it_the_roll_runs_free) {
  const double w0 = 0.2;  // rad/s
  const double ix = 1000.0 * 0.5 * 0.5;  // the structure's roll inertia, m r^2 = 250 kg m^2
  const auto run = [&](bool controlled, double seconds) {
    sim::Params p = bare();
    p.ideal_roll_control = controlled;  // the reference model's default (true) holds the roll: the controller must override it; the free vehicle has it off
    if (controlled) {
      p.spec.roll.enabled = true;
      p.spec.roll.stage = 0;
      p.spec.roll.torque_max = 1.0e6;
      p.spec.roll.kd = 500.0;  // a time constant of I / kd = 0.5 s
    }
    sim::Scenario sc;
    sc.has_initial = true;
    sc.initial.r = sim::V3{sim::kEarthR + 100000.0, 0.0, 0.0};
    sc.initial.w = sim::V3{w0, 0.0, 0.0};
    sim::Vehicle6 v(p, sc);
    for (int k = 0; k < static_cast<int>(seconds * 1000.0); ++k) {
      v.step(0.001, 0.0, 0.0);
    }
    return v.state().w.x;
  };
  CHECK(close(run(true, 1.0), w0 * std::exp(-500.0 * 1.0 / ix), 1e-4));
  CHECK(close(run(true, 0.5), w0 * std::exp(-1.0), 1e-4));
  CHECK(close(run(false, 1.0), w0, 1e-9));  // free: nothing slows it
}

TFC_TEST(dynamics_a_roll_controller_is_limited_in_torque_and_holds_the_angle_with_a_spring) {
  // a torque limit makes the deceleration constant: T / I
  {
    sim::Params p = bare();
    p.spec.roll.enabled = true;
    p.spec.roll.torque_max = 25.0;
    p.spec.roll.kd = 1.0e6;  // far more than the limit: it saturates
    sim::Scenario sc;
    sc.has_initial = true;
    sc.initial.r = sim::V3{sim::kEarthR + 100000.0, 0.0, 0.0};
    sc.initial.w = sim::V3{0.5, 0.0, 0.0};
    sim::Vehicle6 v(p, sc);
    for (int k = 0; k < 1000; ++k) {
      v.step(0.001, 0.0, 0.0);
    }
    CHECK(near_abs(v.state().w.x, 0.5 - (25.0 / 250.0 * 1.0), 1e-9));  // 0.1 rad/s per second for a second
    // the other side
    sim::Scenario sn = sc;
    sn.initial.w = sim::V3{-0.5, 0.0, 0.0};
    sim::Vehicle6 u(p, sn);
    for (int k = 0; k < 1000; ++k) {
      u.step(0.001, 0.0, 0.0);
    }
    CHECK(near_abs(u.state().w.x, -0.5 + 0.1, 1e-9));
  }
  // a spring alone (kd = 0) is a harmonic oscillator: the angle swings by w0 / wn at the frequency sqrt(kp / I)
  {
    const double fn = 1.0;
    const double wn = 2.0 * 3.14159265358979323846 * fn;
    sim::Params p = bare();
    p.spec.roll.enabled = true;
    p.spec.roll.torque_max = 1.0e9;
    p.spec.roll.kp = 250.0 * wn * wn;
    sim::Scenario sc;
    sc.has_initial = true;
    sc.initial.r = sim::V3{sim::kEarthR + 100000.0, 0.0, 0.0};
    sc.initial.w = sim::V3{0.1, 0.0, 0.0};
    sim::Vehicle6 v(p, sc);
    double peak = 0.0;
    double t_peak = 0.0;
    for (int k = 1; k <= 1000; ++k) {
      v.step(0.001, 0.0, 0.0);
      if (v.state().roll > peak) {
        peak = v.state().roll;
        t_peak = 0.001 * k;
      }
    }
    CHECK(close(peak, 0.1 / wn, 1e-3) && near_abs(t_peak, 0.25, 0.002));  // a quarter period
    // and the angle at any time is the closed form, (w0 / wn) sin(wn t)
    sim::Vehicle6 u(p, sc);
    for (int k = 1; k <= 400; ++k) {
      u.step(0.001, 0.0, 0.0);
      if (k % 100 == 0) {
        CHECK(near_abs(u.state().roll, (0.1 / wn) * std::sin(wn * 0.001 * k), 1e-9));
      }
    }
  }
  // the controller works while its stage is on the vehicle: after the stage separates the roll runs free
  {
    sim::Params p = bare();
    p.spec.roll.enabled = true;
    p.spec.roll.stage = 0;
    p.spec.roll.torque_max = 1.0e6;
    p.spec.roll.kd = 500.0;
    p.spec.stages[0].separate_time_s = 0.5;
    sim::StageSpec up;
    up.name = "upper";
    up.dry_mass = 500.0;
    up.x_start = 4.0;
    up.length = 2.0;
    up.radius = 0.4;
    p.spec.stages.push_back(up);
    sim::Scenario sc;
    sc.has_initial = true;
    sc.initial.r = sim::V3{sim::kEarthR + 100000.0, 0.0, 0.0};
    sc.initial.w = sim::V3{0.2, 0.0, 0.0};
    sim::Vehicle6 v(p, sc);
    for (int k = 0; k < 600; ++k) {
      v.step(0.001, 0.0, 0.0);
    }
    const double at_sep = v.state().w.x;
    for (int k = 0; k < 400; ++k) {
      v.step(0.001, 0.0, 0.0);
    }
    CHECK(at_sep < 0.2 * 0.5 && near_abs(v.state().w.x, at_sep, 1e-9));  // slowed before, constant after
  }
}

// ---- a closed loop may be given the tables it flies ----

TFC_TEST(dynamics_a_closed_loop_flies_the_same_flight_with_tables_it_is_given_as_with_tables_it_designs) {
  sim::Loop a;
  a.frames = 1500U;
  const sim::Result ra = sim::run(a);
  const sim::FlightTables tables = sim::flight_tables(a.cfg.design, a.cfg.plan);
  sim::Loop b = a;
  b.cfg.tables = &tables;
  const sim::Result rb = sim::run(b);
  CHECK(ra.max_deg == rb.max_deg && ra.rms_deg == rb.rms_deg && ra.safe_frames == rb.safe_frames && ra.finite && rb.finite && ra.max_deg > 0.0);
  // and the runner says which tables it flies
  sim::RunnerConfig cfg;
  cfg.tables = &tables;
  const sim::SimRunner r(cfg);
  CHECK(&r.tables() != &tables && r.tables().gains.size() == tables.gains.size() && r.tables().guidance.size(1U) == tables.guidance.size(1U));
}

// ---- the files ----

TFC_TEST(dynamics_files_read_the_servo_the_roll_controller_jet_damping_and_the_separation_and_write_them_back) {
  const std::string text = R"({
    "actuator": {"order": 2, "natural_hz": 8, "damping": 0.6, "backlash_deg": 0.25},
    "roll_control": {"stage": 0, "torque_max_nm": 800, "kp_nm_per_rad": 120, "kd_nm_s_per_rad": 90},
    "jet_damping": true,
    "stages": [{"dry_mass_kg": 100, "length_m": 2, "radius_m": 0.2, "separation_dv_ms": 1.5, "tipoff_pitch_dps": 0.5, "tipoff_yaw_dps": -0.25, "tipoff_roll_dps": 1}]
  })";
  sim::VehicleFile v;
  std::vector<std::string> errors;
  CHECK(sim::read_vehicle(text, v, errors) && errors.empty());
  const sim::VehicleSpec& g = v.params.spec;
  CHECK(g.actuator.order == 2 && g.actuator.natural_hz == 8.0 && g.actuator.damping == 0.6 && g.actuator.backlash_deg == 0.25);
  CHECK(g.roll.enabled && g.roll.stage == 0 && g.roll.torque_max == 800.0 && g.roll.kp == 120.0 && g.roll.kd == 90.0);
  CHECK(g.jet_damping);
  CHECK(g.stages[0].separation_dv_ms == 1.5 && g.stages[0].tipoff_pitch_dps == 0.5 && g.stages[0].tipoff_yaw_dps == -0.25 && g.stages[0].tipoff_roll_dps == 1.0);
  const std::string once = sim::write_vehicle(v);
  sim::VehicleFile w;
  CHECK(sim::read_vehicle(once, w, errors) && errors.empty() && sim::write_vehicle(w) == once);
  CHECK(w.params.spec.actuator.order == 2 && w.params.spec.jet_damping && w.params.spec.roll.kd == 90.0);
  // a servo with only a backlash, a separation with only a roll tip-off, and a roll controller on a later stage are all written
  sim::VehicleFile only;
  CHECK(sim::read_vehicle(R"({"actuator": {"backlash_deg": 0.3}, "stages": [{"dry_mass_kg": 100, "length_m": 2, "radius_m": 0.2, "tipoff_roll_dps": 2}, {"dry_mass_kg": 10, "length_m": 1, "radius_m": 0.2, "x_start_m": 2}],
                             "roll_control": {"stage": 1, "torque_max_nm": 5}})", only, errors) && errors.empty());
  const std::string only_text = sim::write_vehicle(only);
  sim::VehicleFile only2;
  CHECK(sim::read_vehicle(only_text, only2, errors) && errors.empty() && only2.params.spec.actuator.backlash_deg == 0.3 && only2.params.spec.actuator.order == 1 &&
        only2.params.spec.stages[0].tipoff_roll_dps == 2.0 && only2.params.spec.stages[0].separation_dv_ms == 0.0 && only2.params.spec.roll.stage == 1 && !only2.params.spec.jet_damping);
  // a vehicle without any of it writes none of it (the reference vehicle's file does not change)
  sim::VehicleFile plain;
  CHECK(sim::read_vehicle(R"({"stages": [{"dry_mass_kg": 100, "length_m": 2, "radius_m": 0.2}]})", plain, errors));
  const std::string plain_text = sim::write_vehicle(plain);
  CHECK(plain_text.find("actuator") == std::string::npos && plain_text.find("\"roll_control\"") == std::string::npos && plain_text.find("jet_damping") == std::string::npos &&
        plain_text.find("tipoff") == std::string::npos);
}

TFC_TEST(dynamics_refusals_name_the_field) {
  const auto refused = [](const std::string& fragment, const std::string& what) {
    sim::VehicleFile v;
    std::vector<std::string> errors;
    const std::string text = R"({"stages": [{"dry_mass_kg": 100, "length_m": 2, "radius_m": 0.2}], )" + fragment + "}";
    if (sim::read_vehicle(text, v, errors) && errors.empty()) {
      const std::vector<std::string> problems = sim::validate(v.params.spec);
      for (const std::string& p : problems) {
        if (p.find(what) != std::string::npos) {
          return true;
        }
      }
      return false;
    }
    for (const std::string& e : errors) {
      if (e.find(what) != std::string::npos) {
        return true;
      }
    }
    return false;
  };
  CHECK(refused(R"("actuator": {"order": 3})", "actuator.order must be 1 or 2"));
  CHECK(refused(R"("actuator": {"order": 2, "natural_hz": 0})", "natural_hz and damping above zero"));
  CHECK(refused(R"("actuator": {"order": 2, "damping": 0})", "natural_hz and damping above zero"));
  CHECK(refused(R"("actuator": {"backlash_deg": -1})", "backlash_deg must not be negative"));
  CHECK(refused(R"("roll_control": {"stage": 4, "torque_max_nm": 10})", "roll_control.stage must name a stage"));
  CHECK(refused(R"("roll_control": {"stage": 0})", "torque_max_nm must be positive"));
  CHECK(refused(R"("roll_control": {"torque_max_nm": 10, "kp_nm_per_rad": -1})", "gains not negative"));
  CHECK(refused(R"("roll_control": {"torque_max_nm": 10, "kd_nm_s_per_rad": -1})", "gains not negative"));
  CHECK(refused(R"("roll_contol": {"torque_max_nm": 10})", "roll_control"));  // a typo is refused with a suggestion
  CHECK(refused(R"("jet_dampng": true)", "jet_damping"));
  sim::VehicleSpec g;
  sim::StageSpec st;
  st.dry_mass = 1.0;
  st.length = 1.0;
  st.tipoff_pitch_dps = std::nan("");
  g.stages.push_back(st);
  bool named = false;
  for (const std::string& p : sim::validate(g)) {
    named = named || p.find("tip-off rates must be numbers") != std::string::npos;
  }
  CHECK(named);
}
