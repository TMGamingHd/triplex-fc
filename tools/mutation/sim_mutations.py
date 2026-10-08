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
m("thrust_no_back_pressure", V, " - (air.pressure * p_.exit_area_each));", ");")
m("mdot_ignores_thrust_scale", V, "l.mdot += p_.thrust_scale * p_.thrust_vac_each /", "l.mdot += p_.thrust_vac_each /")
m("engine_arm_sign", V, "V3{-mp.x_cg, engine_y(i), engine_z(i)}", "V3{mp.x_cg, engine_y(i), engine_z(i)}")
m("engine_y_swapped", V, "i == 1 ? p_.engine_offset : (i == 2 ? -p_.engine_offset : 0.0)", "i == 1 ? -p_.engine_offset : (i == 2 ? p_.engine_offset : 0.0)")
m("engine_z_swapped", V, "i == 3 ? p_.engine_offset : (i == 4 ? -p_.engine_offset : 0.0)", "i == 3 ? -p_.engine_offset : (i == 4 ? p_.engine_offset : 0.0)")
m("gimbal_pitch_sign", V, "-std::sin(dp) * std::cos(dy), std::sin(dy)}", "std::sin(dp) * std::cos(dy), std::sin(dy)}")
m("gimbal_yaw_sign", V, "-std::sin(dp) * std::cos(dy), std::sin(dy)}", "-std::sin(dp) * std::cos(dy), -std::sin(dy)}")
m("gimbal_limit_ignored", V, "std::clamp(cmd, -p_.gimbal_limit_deg, p_.gimbal_limit_deg)", "cmd")
m("gimbal_rate_ignored", V, "const double d = std::clamp(want, -p_.gimbal_rate_dps * h, p_.gimbal_rate_dps * h);", "const double d = want;")
m("gimbal_lag_ignored", V, "p_.gimbal_lag_s > 0.0 ?", "false ?")
m("misalign_yaw_ignored", V, "(gimbal_y_ + p_.thrust_misalign_yaw_deg)", "gimbal_y_")
m("misalign_pitch_ignored", V, "(gimbal_p_ + p_.thrust_misalign_pitch_deg)", "gimbal_p_")
m("gravity_scale_ignored", V, "-p_.gravity_scale * kEarthMu / (rn * rn * rn)", "-kEarthMu / (rn * rn * rn)")
m("inertia_rate_term_dropped", V, "(m_total.y - gyro.y - (dit * s.w.y)) / mp.i_t", "(m_total.y - gyro.y) / mp.i_t")
m("gyroscopic_term_dropped", V, "const V3 gyro = cross(s.w, iw);", "const V3 gyro{};")
m("dI_sign", V, "const double dit = (mp2.i_t - mp.i_t) * -l.mdot;", "const double dit = (mp2.i_t - mp.i_t) * l.mdot;")
m("dIx_dropped", V, "- (dix * s.w.x)) / mp.i_x", ") / mp.i_x")
m("rk4_k4_weight", V, "k.r = (k1.r + (k2.r * 2.0) + (k3.r * 2.0) + k4.r) / 6.0;", "k.r = (k1.r + (k2.r * 2.0) + (k3.r * 2.0) + k3.r) / 6.0;")
m("rk4_w_weight", V, "k.w = (k1.w + (k2.w * 2.0) + (k3.w * 2.0) + k4.w) / 6.0;", "k.w = (k1.w + (k2.w * 2.0) + (k3.w * 2.0) + k3.w) / 6.0;")
equivalent("rk4_mass_weight", V, "k.m = (k1.m + (2.0 * k2.m) + (2.0 * k3.m) + k4.m) / 6.0;", "k.m = (k1.m + (2.0 * k2.m) + (2.0 * k3.m) + k3.m) / 6.0;", "the mass flow does not depend on the state (only on which engines are on and on `burning()`), so the four stages of a step all see the same derivative and the weights of the mass update cannot matter")
m("quaternion_not_normalised", V, "s_.q = normalized(s_.q);", "")
m("mass_floor_removed", V, "s_.m = std::max(s_.m, p_.m_dry);", "")
m("burnout_never", V, "return s_.m > p_.m_dry + 1e-6;", "return true;")
m("engine_out_never", V, "sc_.engine_out_time >= 0.0 && t_ >= sc_.engine_out_time", "false && t_ >= sc_.engine_out_time")
m("engine_out_wrong_engine", V, "engine_on_[static_cast<std::size_t>(sc_.engine_out_index)] = false;", "engine_on_[0] = false;")
m("wind_ignored", V, "V3 w = normalized(sc_.wind_dir) * (sc_.wind_scale * mean_wind_speed(altitude));", "V3 w{};")
m("wind_scale_ignored", V, "(sc_.wind_scale * mean_wind_speed(altitude))", "mean_wind_speed(altitude)")
m("gust_shape", V, "std::cos(2.0 * kPi * (t - g.t0) / g.duration)", "std::cos(kPi * (t - g.t0) / g.duration)")
m("wind_jet_stream_value", V, "{12000.0, 28.0}", "{12000.0, 18.0}")
m("dry_cg_shift_ignored", V, "const double xd = p_.x_dry_cg + sc_.dry_cg_shift;", "const double xd = p_.x_dry_cg;")
m("tilt_x_sign", V, "t.x_deg = -std::asin(", "t.x_deg = std::asin(")
m("vehicle_true_gyro_axes", V, "gyro_dps = V3{s_.w.y, s_.w.z, s_.w.x} * kRad2Deg;", "gyro_dps = V3{s_.w.x, s_.w.y, s_.w.z} * kRad2Deg;")
m("vehicle_true_accel_axes", V, "accel_g = V3{f.y, f.z, f.x};", "accel_g = V3{f.x, f.y, f.z};")
m("transonic_rise_removed", V, "0.30 + (0.45 * std::exp(-d * d))", "0.30")
m("aero_needs_forward_flow", V, "if (v_abs > 1.0 && vrel.x > 0.0) {", "if (v_abs > 1.0) {")
m("prop_inertia", V, "mprop * ((3.0 * r * r) + (hp * hp)) / 12.0", "mprop * ((3.0 * r * r) + (hp * hp)) / 6.0")
m("tank_bottom_ignored", V, "const double xp = p_.x_tank_bottom + (0.5 * hp);", "const double xp = 0.5 * hp;")
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
m("gain_kp_without_divergence", D, "((wn * wn) + std::max(0.0, n.a_div)) / b", "(wn * wn) / b")
m("gain_kd_damping", D, "p.kd = 2.0 * zeta * wn / b;", "p.kd = zeta * wn / b;")
m("kick_not_smooth", D, "c.kick_deg * s * s * (3.0 - (2.0 * s))", "c.kick_deg * s")
m("b_ctl_arm", D, "np.b_ctl = l.thrust * mp.x_cg / mp.i_t;", "np.b_ctl = l.thrust * 8.0 / mp.i_t;")
m("a_div_sign", D, "p.c_n_alpha * (p.x_cp - mp.x_cg) / mp.i_t", "p.c_n_alpha * (mp.x_cg - p.x_cp) / mp.i_t")
m("flight_path_angle", D, "std::acos(std::clamp(dot(normalized(s.v), V3{1.0, 0.0, 0.0}), -1.0, 1.0))", "0.5 * std::acos(std::clamp(dot(normalized(s.v), V3{1.0, 0.0, 0.0}), -1.0, 1.0))")
m("guidance_table_sampling", D, "const std::size_t idx = (nominal.size() - 1U) * k / (tfc::Guidance::kMaxPoints - 1U);", "const std::size_t idx = (nominal.size() - 1U) * k / (tfc::Guidance::kMaxPoints);")

# ---- the ground (added with it, 6 Oct 2026) ----
m("ground_never_holds", V, "if (p_.ground_contact && held_by_ground(h)) {", "if (false) {")
m("ground_released_while_weak", V, "if (vr == 0.0 && a_up > 0.0) {", "if (vr == 0.0) {")
m("ground_no_gravity_in_the_balance", V, "- (p_.gravity_scale * kEarthMu / (rn * rn));", ";")
m("ground_crash_speed_ignored", V, "if (-vr > p_.crash_speed_ms) {", "if (false) {")
m("ground_crash_never_freezes", V, "if (crashed_) {\n      return true;", "if (crashed_) {\n      return false;")
m("ground_ignores_departure", V, "if (vr > 0.0) {\n      grounded_ = false;", "if (false) {\n      grounded_ = false;")
m("ground_holds_no_propellant_burn", V, "if (!crashed_ && burning()) {\n      s_.m = std::max(s_.m - (l.mdot * h), p_.m_dry);\n    }", "")
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
