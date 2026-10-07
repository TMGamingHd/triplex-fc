# SPDX-License-Identifier: MIT
"""The mutants of the vehicle simulator: small deliberate bugs injected into a COPY of sim/vehicle. A test suite that cannot tell a mutant from the real
code is not testing that behaviour. Each entry is name -> (file, text to find, text to put instead); the text must occur exactly once in the file (the
runner refuses to run otherwise, so a refactor that moves code cannot silently turn a mutant into a no-op).

Used by tools/mutation/run_sim.py. The flight core has its own set (tools/mutation/mutations.py); this one covers what the simulator computes: the
dynamics, the atmosphere, the ground, the platform, the IMU model, the runner and the design of the tables the flight computers carry. The first sweep
(6 Oct 2026) injected 67 bugs: 53 were caught, 13 of the 14 survivors were gaps in the tests (now closed by tests/test_sim_model.cpp and the flight
snapshot of tests/test_vehicle.cpp) and one is equivalent (EQUIVALENT below, with its reason). The ground model added 10 more.
"""
from __future__ import annotations

import shutil
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]

MUTATIONS: dict[str, tuple[str, str, str]] = {}
EQUIVALENT: dict[str, tuple[str, str, str, str]] = {}


def m(name: str, f: str, old: str, new: str) -> None:
    MUTATIONS[name] = (f, old, new)


def equivalent(name: str, f: str, old: str, new: str, reason: str) -> None:
    EQUIVALENT[name] = (f, old, new, reason)


V = "vehicle6.hpp"
m("aero_moment_sign", V, "cross(V3{p_.x_cp - mp.x_cg, 0.0, 0.0}", "cross(V3{mp.x_cg - p_.x_cp, 0.0, 0.0}")
m("normal_force_sign", V, "const double fn = -l.dynamic_pressure", "const double fn = l.dynamic_pressure")
m("cn_scale_ignored", V, "p_.c_n_alpha * p_.cn_scale * l.alpha", "p_.c_n_alpha * l.alpha")
m("cd_scale_ignored", V, "p_.cd_scale * axial_coefficient(l.mach)", "axial_coefficient(l.mach)")
m("thrust_no_back_pressure", V, "(frac * p_.thrust_scale * en.thrust_vac) - (air.pressure * en.exit_area));", "(frac * p_.thrust_scale * en.thrust_vac));")
m("mdot_ignores_thrust_scale", V, "const double md = frac * p_.thrust_scale * en.thrust_vac / (en.isp_vac * kG0);", "const double md = frac * en.thrust_vac / (en.isp_vac * kG0);")
m("engine_arm_sign", V, "cross(V3{en.pos.x - mp.x_cg, en.pos.y, en.pos.z}, f)", "cross(V3{mp.x_cg - en.pos.x, en.pos.y, en.pos.z}, f)")
m("engine_y_swapped", V, "k == 1 ? p.engine_offset : (k == 2 ? -p.engine_offset : 0.0)", "k == 1 ? -p.engine_offset : (k == 2 ? p.engine_offset : 0.0)")
m("engine_z_swapped", V, "k == 3 ? p.engine_offset : (k == 4 ? -p.engine_offset : 0.0)", "k == 3 ? -p.engine_offset : (k == 4 ? p.engine_offset : 0.0)")
m("gimbal_pitch_sign", V, "-std::sin(dp) * std::cos(dy), std::sin(dy)}", "std::sin(dp) * std::cos(dy), std::sin(dy)}")
m("gimbal_yaw_sign", V, "-std::sin(dp) * std::cos(dy), std::sin(dy)}", "-std::sin(dp) * std::cos(dy), -std::sin(dy)}")
m("gimbal_limit_ignored", V, "const double target = std::clamp(cmd, -limit, limit);", "const double target = cmd;")
m("gimbal_rate_ignored", V, "const double d = std::clamp(want, -rate * h, rate * h);", "const double d = want;")
m("gimbal_lag_ignored", V, "const double want = lag > 0.0 ?", "const double want = false ?")
m("misalign_yaw_ignored", V, "(((en.gimbal ? gimbal_y_[st] : 0.0) + en.cant_yaw_deg) + p_.thrust_misalign_yaw_deg)", "(((en.gimbal ? gimbal_y_[st] : 0.0) + en.cant_yaw_deg))")
m("misalign_pitch_ignored", V, "(((en.gimbal ? gimbal_p_[st] : 0.0) + en.cant_pitch_deg) + p_.thrust_misalign_pitch_deg)", "(((en.gimbal ? gimbal_p_[st] : 0.0) + en.cant_pitch_deg))")
m("gravity_scale_ignored", V, "-p_.gravity_scale * kEarthMu / (rn * rn * rn)", "-kEarthMu / (rn * rn * rn)")
m("inertia_rate_term_dropped", V, "(m_total.y - gyro.y - (dit * s.w.y)) / mp.i_t", "(m_total.y - gyro.y) / mp.i_t")
m("gyroscopic_term_dropped", V, ": cross(s.w, iw);", ": V3{};")
m("dI_sign", V, "dit += (mp2.i_t - mp.i_t) * -l.mdot_stage[st];", "dit += (mp2.i_t - mp.i_t) * l.mdot_stage[st];")
m("dIx_dropped", V, "- (dix * s.w.x)) / mp.i_x", ") / mp.i_x")
m("rk4_k4_weight", V, "k.r = (k1.r + (k2.r * 2.0) + (k3.r * 2.0) + k4.r) / 6.0;", "k.r = (k1.r + (k2.r * 2.0) + (k3.r * 2.0) + k3.r) / 6.0;")
m("rk4_w_weight", V, "k.w = (k1.w + (k2.w * 2.0) + (k3.w * 2.0) + k4.w) / 6.0;", "k.w = (k1.w + (k2.w * 2.0) + (k3.w * 2.0) + k3.w) / 6.0;")
equivalent("rk4_mass_weight", V, "k.m = (k1.m + (2.0 * k2.m) + (2.0 * k3.m) + k4.m) / 6.0;", "k.m = (k1.m + (2.0 * k2.m) + (2.0 * k3.m) + k3.m) / 6.0;", "the mass flow does not depend on the state (only on which engines are on and on `burning()`), so the four stages of a step all see the same derivative and the weights of the mass update cannot matter")
m("quaternion_not_normalised", V, "s_.q = normalized(s_.q);", "")
m("mass_floor_removed", V, "s_.m = std::max(s_.m, fixed_);  // the propellant cannot go below empty", "")
m("burnout_never", V, "if (active_[s] && s_.prop[s] > kEmptyProp) {\n        return true;", "if (active_[s]) {\n        return true;")
m("engine_out_never", V, "if (f.index >= 0 && f.index < static_cast<int>(failed_.size()) && t_ >= f.time) {", "if (f.index >= 0 && f.index < static_cast<int>(failed_.size()) && t_ >= f.time && false) {")
m("engine_out_wrong_engine", V, "failed_[static_cast<std::size_t>(f.index)] = true;", "failed_[0] = true;")
m("wind_ignored", V, "V3 w = normalized(sc_.wind_dir) * (sc_.wind_scale * mean_wind_speed(altitude));", "V3 w{};")
m("wind_scale_ignored", V, "(sc_.wind_scale * mean_wind_speed(altitude))", "mean_wind_speed(altitude)")
m("gust_shape", V, "std::cos(2.0 * kPi * (t - g.t0) / g.duration)", "std::cos(kPi * (t - g.t0) / g.duration)")
m("wind_jet_stream_value", V, "{12000.0, 28.0}", "{12000.0, 18.0}")
m("dry_cg_shift_ignored", V, " + sc_.dry_cg_shift;", ";")
m("tilt_x_sign", V, "t.x_deg = -std::asin(", "t.x_deg = std::asin(")
m("vehicle_true_gyro_axes", V, "gyro_dps = V3{s_.w.y, s_.w.z, s_.w.x} * kRad2Deg;", "gyro_dps = V3{s_.w.x, s_.w.y, s_.w.z} * kRad2Deg;")
m("vehicle_true_accel_axes", V, "accel_g = V3{f.y, f.z, f.x};", "accel_g = V3{f.x, f.y, f.z};")
m("transonic_rise_removed", V, "0.30 + (0.45 * std::exp(-d * d))", "0.30")
m("aero_needs_forward_flow", V, "if (v_abs > 1.0 && vrel.x > 0.0) {", "if (v_abs > 1.0) {")
m("prop_inertia", V, "tm * ((3.0 * tr * tr) + (hp * hp)) / 12.0", "tm * ((3.0 * tr * tr) + (hp * hp)) / 6.0")
m("tank_bottom_ignored", V, "tk.x_bottom + (0.5 * hp)", "(0.5 * hp)")
m("roll_hold_ignored", V, "    if (p_.ideal_roll_control) {\n      d.w.x = 0.0;\n    }\n", "")  # killed by the flight snapshot of tests/test_vehicle.cpp: its derivative feeds the gyroscopic terms inside a stage
A = "atmosphere.hpp"
m("geopotential_radius", A, "constexpr double kGeopotR = 6356766.0;", "constexpr double kGeopotR = 6371000.0;")
m("sound_speed_gamma", A, "std::sqrt(1.4 * kRAir", "std::sqrt(1.3 * kRAir")
m("layer3_lapse", A, "{32000.0, 228.65, 868.0187, 0.0028}", "{32000.0, 228.65, 868.0187, 0.0027}")
m("exponential_tail_scale", A, "-(h - kTop) / 6500.0", "-(h - kTop) / 7500.0")
P = "platform.hpp"
m("platform_travel_ignored", P, "std::clamp(target, -c_.limit_deg, c_.limit_deg)", "target")
m("platform_rate_ignored", P, "rate = std::clamp((goal - angle) / c_.tau_s, -c_.rate_limit_dps, c_.rate_limit_dps);", "rate = (goal - angle) / c_.tau_s;")
m("platform_saturation_never", P, "saturated_ = std::fabs(target.x_deg) > c_.limit_deg || std::fabs(target.y_deg) > c_.limit_deg;", "saturated_ = false;")
m("platform_gravity_sign", P, "accel_g = V3{-std::sin(theta),", "accel_g = V3{std::sin(theta),")
m("platform_rate_axes", P, "gyro_dps = V3{rx_, ry_ * std::cos(phi), -ry_ * std::sin(phi)};", "gyro_dps = V3{rx_, ry_ * std::cos(phi), ry_ * std::sin(phi)};")
I = "imu_model.hpp"
m("imu_bias_sign", I, "(g.v[i] * gs_[i]) + gb_[i]", "(g.v[i] * gs_[i]) - gb_[i]")
m("imu_latency_ignored", I, "const unsigned lat = e_.latency_frames < kHistory ? e_.latency_frames : kHistory - 1U;", "const unsigned lat = 0U;")
m("imu_stale_inverted", I, "< e_.stale_prob", "> e_.stale_prob")
m("imu_accel_scale_ignored", I, "(a.v[i] * as_[i]) + ab_[i]", "a.v[i] + ab_[i]")
R = "runner.hpp"
m("runner_frame_period", R, "vehicle_.step(0.01, pitch_deg, yaw_deg);", "vehicle_.step(0.011, pitch_deg, yaw_deg);")
m("runner_held_flag", R, "held_ = act == nullptr;", "held_ = false;")
m("runner_pad_moves", R, "if (!clamped_) {  // on the pad nothing moves, whatever ACT says", "if (true) {  // on the pad nothing moves, whatever ACT says")
m("runner_safed_state", R, "act->state == static_cast<uint8_t>(2U)", "act->state == static_cast<uint8_t>(3U)")
m("runner_command_not_held", R, "      held_pitch_ = static_cast<double>(act->pitch_deg);", "      held_pitch_ = 0.0 * static_cast<double>(act->pitch_deg);")
D = "design.hpp"
m("gain_kp_without_divergence", D, "next = n.t + every_s;\n    GainPoint p;\n    p.t = n.t;\n    const double b = std::max(n.b_ctl, b_min);\n    p.kp = std::min(((wn * wn) + std::max(0.0, n.a_div)) / b, kp_max);", "next = n.t + every_s;\n    GainPoint p;\n    p.t = n.t;\n    const double b = std::max(n.b_ctl, b_min);\n    p.kp = std::min((wn * wn) / b, kp_max);")
m("gain_kd_damping", D, "next = n.t + every_s;\n    GainPoint p;\n    p.t = n.t;\n    const double b = std::max(n.b_ctl, b_min);\n    p.kp = std::min(((wn * wn) + std::max(0.0, n.a_div)) / b, kp_max);\n    p.kd = 2.0 * zeta * wn / b;", "next = n.t + every_s;\n    GainPoint p;\n    p.t = n.t;\n    const double b = std::max(n.b_ctl, b_min);\n    p.kp = std::min(((wn * wn) + std::max(0.0, n.a_div)) / b, kp_max);\n    p.kd = zeta * wn / b;")
m("kick_not_smooth", D, "c.kick_deg * s * s * (3.0 - (2.0 * s))", "c.kick_deg * s")
m("b_ctl_arm", V, "num += ti * (mp.x_cg - en.pos.x);", "num += ti * 8.0;")
m("a_div_sign", V, "return l.dynamic_pressure * area * p_.c_n_alpha * (p_.x_cp - mp.x_cg) / mp.i_t;", "return l.dynamic_pressure * area * p_.c_n_alpha * (mp.x_cg - p_.x_cp) / mp.i_t;")
m("flight_path_angle", D, "std::acos(std::clamp(dot(normalized(s.v), V3{1.0, 0.0, 0.0}), -1.0, 1.0))", "0.5 * std::acos(std::clamp(dot(normalized(s.v), V3{1.0, 0.0, 0.0}), -1.0, 1.0))")
m("guidance_table_sampling", D, "const std::size_t idx = (nominal.size() - 1U) * k / (tfc::Guidance::kMaxPoints - 1U);", "const std::size_t idx = (nominal.size() - 1U) * k / (tfc::Guidance::kMaxPoints);")

# ---- the general vehicle (6 Oct 2026): stages, tanks, engines at positions, throttle, transients, effectors, the design of the tables ----
m("stage_ignition_time_ignored", V, "go = t_ >= st.ignite_time_s;", "go = true;")
m("ignition_after_separation_ignores_delay", V, "go = ts >= 0.0 && t_ >= ts + st.ignite_delay_s;", "go = ts >= 0.0 && t_ >= ts;")
m("separation_ignores_burnout_delay", V, "t_ >= t_burnout_[s] + st.separate_delay_s", "t_ >= t_burnout_[s]")
m("separation_by_time_ignored", V, "const bool by_time = st.separate_time_s >= 0.0 && t_ >= st.separate_time_s;", "const bool by_time = false;")
m("separation_keeps_the_mass", V, "active_[s] = false;\n        t_sep_[s] = t_;\n        refresh_fixed();\n        s_.m = total_mass(s_.prop);", "active_[s] = false;\n        t_sep_[s] = t_;")
m("payload_never_jettisoned", V, "if (payload_active_[i] && g_.payloads[i].jettison_time_s >= 0.0 && t_ >= g_.payloads[i].jettison_time_s) {", "if (false) {")
m("burnout_not_detected", V, "if (t_ign_[s] >= 0.0 && t_burnout_[s] < 0.0 && s_.prop[s] <= kEmptyProp) {", "if (false) {")
m("engine_start_offset_ignored", V, "t_ >= t_ign_[st] + en.start_offset_s && s_.prop[st] > kEmptyProp && !failed_[e] && (en", "t_ >= t_ign_[st] && s_.prop[st] > kEmptyProp && !failed_[e] && (en")
m("engine_cutoff_ignored", V, "&& !failed_[e] && (en.cutoff_time_s < 0.0 || t_ < en.cutoff_time_s);", "&& !failed_[e];")
m("rise_and_tail_swapped", V, "const double tau = cmd > frac_[e] ? en.rise_s : en.tail_s;", "const double tau = cmd > frac_[e] ? en.tail_s : en.rise_s;")
m("throttle_not_applied", V, "const double cmd = lit ? throttle_of(g_.stages[st], t_ - t_ign_[st]) * duty : 0.0;", "const double cmd = lit ? duty : 0.0;")
m("throttle_not_interpolated", V, "return st.throttle[i - 1][1] + (f * (st.throttle[i][1] - st.throttle[i - 1][1]));", "return st.throttle[i][1];")
m("thruster_minus_command_sign", V, "duty = std::clamp(-cmd_pitch_deg / en.full_cmd_deg, 0.0, 1.0);", "duty = std::clamp(cmd_pitch_deg / en.full_cmd_deg, 0.0, 1.0);")
m("thruster_full_command_ignored", V, "duty = std::clamp(cmd_yaw_deg / en.full_cmd_deg, 0.0, 1.0);", "duty = std::clamp(cmd_yaw_deg, 0.0, 1.0);")
m("thruster_own_direction_ignored", V, "dir = normalized(en.dir);", "dir = V3{1.0, 0.0, 0.0};")
m("fin_pitch_force_sign", V, "const double fy = -qs * fin_p_[f] * kDeg2Rad;", "const double fy = qs * fin_p_[f] * kDeg2Rad;")
m("fin_yaw_force_sign", V, "const double fz = qs * fin_y_[f] * kDeg2Rad;", "const double fz = -qs * fin_y_[f] * kDeg2Rad;")
m("fin_area_of_one_fin", V, "const double qs = l.dynamic_pressure * 2.0 * fin.area_each * fin.lift_slope;", "const double qs = l.dynamic_pressure * fin.area_each * fin.lift_slope;")
m("fin_moment_arm_reversed", V, "cross(V3{fin.x_hinge - mp.x_cg, 0.0, 0.0}, V3{0.0, fy, fz})", "cross(V3{mp.x_cg - fin.x_hinge, 0.0, 0.0}, V3{0.0, fy, fz})")
m("fin_gain_ignored_in_command", V, "fin_p_[f] = slew(fin_p_[f], fin.gain * cmd_pitch_deg,", "fin_p_[f] = slew(fin_p_[f], cmd_pitch_deg,")
m("fin_effectiveness_ignores_gain", V, "b += q * 2.0 * fin.area_each * fin.lift_slope * fin.gain * (mp.x_cg - fin.x_hinge) / mp.i_t;", "b += q * 2.0 * fin.area_each * fin.lift_slope * (mp.x_cg - fin.x_hinge) / mp.i_t;")
m("wheel_axes_swapped", V, "return V3{0.0, m_yaw, m_pitch};", "return V3{0.0, m_pitch, m_yaw};")
m("wheel_does_not_take_momentum", V, "s_.wheel_h.z = std::clamp(s_.wheel_h.z - (wheel_m_.z * h), -g_.wheels.momentum_max, g_.wheels.momentum_max);", "s_.wheel_h.z = std::clamp(s_.wheel_h.z + (wheel_m_.z * h), -g_.wheels.momentum_max, g_.wheels.momentum_max);")
m("wheel_saturation_ignored", V, "if ((s_.wheel_h.z <= -w.momentum_max && m_pitch > 0.0) || (s_.wheel_h.z >= w.momentum_max && m_pitch < 0.0)) {", "if (false) {")
m("wheel_momentum_not_in_the_dynamics", V, "cross(s.w, iw + V3{0.0, s.wheel_h.y, s.wheel_h.z})", "cross(s.w, iw)")
m("tanks_drain_in_parallel_always", V, "if (st.sequential_drain) {", "if (false) {")
m("parallel_tank_share_ignored", V, "tm = capacity(s) > 0.0 ? total * (tk.propellant / capacity(s)) : 0.0;", "tm = total;")
m("start_in_flight_keeps_default_mass", V, "init.prop = s_.prop;\n        init.m = s_.m;", "")
m("thruster_effectiveness_sign", V, "const double mz = std::fabs(((en.pos.x - mp.x_cg) * d.y * ti) - (en.pos.y * d.x * ti));", "const double mz = ((en.pos.x - mp.x_cg) * d.y * ti) - (en.pos.y * d.x * ti);")
m("wheel_effectiveness_ignored", V, "b += g_.wheels.torque_max / (g_.wheels.full_cmd_deg * kDeg2Rad) / mp.i_t;", "b += 0.0;")
m("engine_cant_ignored", V, "((en.gimbal ? gimbal_p_[st] : 0.0) + en.cant_pitch_deg)", "((en.gimbal ? gimbal_p_[st] : 0.0))")
m("shared_direction_cache_wrong_stage", V, "dir = dir_gimbal[st];", "dir = dir_gimbal[0];")
m("engine_position_x_ignored", V, "cross(V3{en.pos.x - mp.x_cg, en.pos.y, en.pos.z}, f)", "cross(V3{-mp.x_cg, en.pos.y, en.pos.z}, f)")
m("stage_gimbal_override_ignored", V, "const double limit = st.gimbal_limit_deg >= 0.0 ? st.gimbal_limit_deg : p_.gimbal_limit_deg;", "const double limit = p_.gimbal_limit_deg;")
m("adaptive_stops_at_two_points", D, "double worst = tolerance;", "double worst = 1.0e30;")
m("adaptive_picks_the_smallest_error", D, "if (e > worst) {", "if (e < worst) {")
m("adaptive_ignores_kd_error", D, "const double e = std::max(ekp, ekd);", "const double e = ekp;")
m("program_table_not_interpolated", D, "return c.table[i - 1][1] + (f * (c.table[i][1] - c.table[i - 1][1]));", "return c.table[i][1];")
m("adaptive_ignores_b_min", D, "for (const NominalPoint& n : nominal) {\n    GainPoint p;\n    p.t = n.t;\n    const double b = std::max(n.b_ctl, b_min);", "for (const NominalPoint& n : nominal) {\n    GainPoint p;\n    p.t = n.t;\n    const double b = std::max(n.b_ctl, 1.0e-3);")
m("uniform_gain_ignores_b_min", D, "next = n.t + every_s;\n    GainPoint p;\n    p.t = n.t;\n    const double b = std::max(n.b_ctl, b_min);", "next = n.t + every_s;\n    GainPoint p;\n    p.t = n.t;\n    const double b = std::max(n.b_ctl, 1.0e-3);")
m("kp_cap_ignored", D, "p.kp = std::min(((wn * wn) + std::max(0.0, n.a_div)) / b, kp_max);\n    p.kd = 2.0 * zeta * wn / b;\n    p.ki = ki_over_kp * p.kp;\n    all.push_back(p);", "p.kp = ((wn * wn) + std::max(0.0, n.a_div)) / b;\n    p.kd = 2.0 * zeta * wn / b;\n    p.ki = ki_over_kp * p.kp;\n    all.push_back(p);")
m("runner_engine_out_flag_never", R, "if (vehicle_.engines_on() < vehicle_.engine_count()) {", "if (false) {")

# ---- the ground (added with it, 6 Oct 2026) ----
m("ground_never_holds", V, "if (p_.ground_contact && held_by_ground(h)) {", "if (false) {")
m("ground_released_while_weak", V, "if (vr == 0.0 && a_up > 0.0) {", "if (vr == 0.0) {")
m("ground_no_gravity_in_the_balance", V, "- (p_.gravity_scale * kEarthMu / (rn * rn));", ";")
m("ground_crash_speed_ignored", V, "if (-vr > p_.crash_speed_ms) {", "if (false) {")
m("ground_crash_never_freezes", V, "if (crashed_) {\n      return true;", "if (crashed_) {\n      return false;")
m("ground_ignores_departure", V, "if (vr > 0.0) {\n      grounded_ = false;", "if (false) {\n      grounded_ = false;")
m("ground_holds_no_propellant_burn", V, "s_.m = std::max(s_.m - (l.mdot * h), fixed_);", "s_.m = std::max(s_.m, fixed_);")
m("ground_keeps_its_velocity", V, "s_.r = up * kEarthR;\n    s_.v = V3{};\n    s_.w = V3{};", "s_.r = up * kEarthR;\n    s_.w = V3{};")
m("ground_keeps_its_rotation", V, "s_.r = up * kEarthR;\n    s_.v = V3{};\n    s_.w = V3{};", "s_.r = up * kEarthR;\n    s_.v = V3{};")
m("ground_fast_path_in_the_air_off", V, "if (!grounded_ && rn > kEarthR + 1e-6) {", "if (!grounded_ && rn > kEarthR + 1e9) {")


def build_vehicle(name: str, tmp: Path) -> Path:
    """Copy sim/vehicle into tmp, apply the mutant `name` (a key of MUTATIONS, or an EQUIVALENT one, or "BASELINE" for none), and return the directory."""
    dst = tmp / "vehicle"
    shutil.copytree(ROOT / "sim" / "vehicle", dst)
    if name == "BASELINE":
        return dst
    f, old, new = MUTATIONS[name] if name in MUTATIONS else EQUIVALENT[name][:3]
    path = dst / f
    text = path.read_text()
    if text.count(old) != 1:
        raise SystemExit(f"mutant {name}: the text to replace occurs {text.count(old)} times in {f} (it must occur exactly once); update the mutant")
    path.write_text(text.replace(old, new))
    return dst
