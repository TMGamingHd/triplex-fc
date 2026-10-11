// SPDX-License-Identifier: MIT
// The vehicle's whole state as one line of JSON, for the 3D viewer (host only). See docs/design/VIEWER.md and ADR-035.
//
// The flight console's telemetry (tfc_simd --telemetry) is thirty numbers ten times a second: enough for charts, not enough to draw the vehicle. The viewer needs where it is and which way it points
// (the whole state, not two tilts), the air it is flying through, the forces on it and where they act, what each engine, gimbal and fin is doing, and how much liquid is in each tank. That is a
// "pose": one JSON object per line with every number taken from the simulator's own state (Vehicle6) or from the model's own functions (the atmosphere, the wind, the aerodynamics), never invented here.
// It is an output only; nothing the flight does depends on it. The first line of a run is the "spec": the vehicle as the simulator reads it, and the world around it (the air and wind at every
// kilometre) so the viewer draws the same atmosphere the model flew through.
//
// Frames: the planet's centre is the origin of the inertial frame the model integrates in; +X is up at the launch point at T-zero, +Y downrange, +Z crossrange. The body frame is the vehicle's:
// +X along the long axis toward the nose, Y and Z lateral. `q` rotates the body frame into the inertial one (w, x, y, z). Lengths in metres, speeds in m/s, angles in degrees unless the key says rad.
#pragma once
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "spec_io.hpp"
#include "vehicle6.hpp"

namespace sim {

// What the flight computers are doing, which the vehicle does not know: the program they follow and their command (zero when there is no flight software, as in a bare vehicle).
struct PoseExtra {
  double ref_pitch_deg = 0.0;   // the program's tilt about the pitch plane
  double ref_yaw_deg = 0.0;
  double cmd_pitch_deg = 0.0;   // ACT's voted gimbal command
  double cmd_yaw_deg = 0.0;
  int act_mode = -1;            // -1: not known
  bool clamped = false;         // held on the pad by the clamps
  unsigned frame = 0U;          // the simulator's frame number
};

namespace viewer_detail {

class Out {
 public:
  void raw(const char* s) { s_ += s; }
  void key(const char* k) {
    comma();
    s_ += '"';
    s_ += k;
    s_ += "\":";
    fresh_ = true;
  }
  void num(double v, int decimals) {
    comma();
    if (!std::isfinite(v)) {
      s_ += "null";
      return;
    }
    char b[48];
    const int n = std::snprintf(b, sizeof b, "%.*f", decimals, v);
    if (n > 0) {
      s_.append(b, static_cast<std::size_t>(n));
    }
  }
  void integer(long v) {
    comma();
    s_ += std::to_string(v);
  }
  void text(const std::string& v) {
    comma();
    s_ += '"';
    for (const char c : v) {
      if (c == '"' || c == '\\') {
        s_ += '\\';
        s_ += c;
      } else if (static_cast<unsigned char>(c) >= 0x20U) {
        s_ += c;
      }
    }
    s_ += '"';
  }
  void open(char c) {
    comma();
    s_ += c;
    fresh_ = true;
  }
  void close(char c) {
    s_ += c;
    fresh_ = false;
  }
  void kv(const char* k, double v, int d) {
    key(k);
    num(v, d);
  }
  void kvi(const char* k, long v) {
    key(k);
    integer(v);
  }
  void vec(const char* k, const V3& v, int d) {
    key(k);
    open('[');
    num(v.x, d);
    num(v.y, d);
    num(v.z, d);
    close(']');
  }
  [[nodiscard]] const std::string& str() const { return s_; }

 private:
  void comma() {
    if (!fresh_) {
      s_ += ',';
    }
    fresh_ = false;
  }
  std::string s_;
  bool fresh_ = true;
};

}  // namespace viewer_detail

// The world the vehicle flies in, sampled where the model would be asked: the air (temperature, pressure, density, speed of sound) and the mean wind speed every `step_m` up to `top_m`.
// The planet's own atmosphere, with the scenario's dispersions, as `Vehicle6::atmosphere` gives it: what the viewer draws is what the vehicle flew through.
inline std::string viewer_spec_json(const VehicleFile& vf, const Vehicle6& veh, double step_m = 1000.0, double top_m = 120000.0) {
  using viewer_detail::Out;
  std::string vehicle = write_vehicle(vf);
  for (char& c : vehicle) {
    if (c == '\n' || c == '\r' || c == '\t') {
      c = ' ';
    }
  }
  Out o;
  o.open('{');
  o.key("k");
  o.text("spec");
  o.kvi("v", 1);
  o.key("name");
  o.text(vf.name);
  o.vec("pole", veh.pole(), 6);
  o.kv("radius_m", veh.planet().radius, 1);
  o.kv("rotation_rad_s", veh.planet().rotation_rate, 13);
  o.kv("mu", veh.planet().mu, 1);
  o.key("air");  // the profile: arrays by altitude
  o.open('{');
  o.kv("step_m", step_m, 1);
  const std::string keys[5] = {"temperature_k", "pressure_pa", "density", "sound_ms", "wind_ms"};
  std::vector<double> cols[5];
  for (double h = 0.0; h <= top_m + 0.5; h += step_m) {
    const Air a = veh.atmosphere(h);
    cols[0].push_back(a.temperature);
    cols[1].push_back(a.pressure);
    cols[2].push_back(a.density);
    cols[3].push_back(a.sound);
    cols[4].push_back(veh.mean_wind_ms(h));
  }
  for (int c = 0; c < 5; ++c) {
    o.key(keys[c].c_str());
    o.open('[');
    for (const double v : cols[c]) {
      o.num(v, c == 2 ? 7 : 3);
    }
    o.close(']');
  }
  o.close('}');
  o.raw(",\"vehicle\":");
  o.raw(vehicle.c_str());
  o.close('}');
  return o.str();
}

// One pose. `t_s` is the time since T-zero (negative while held on the pad).
inline std::string viewer_pose_json(const Vehicle6& veh, double t_s, const PoseExtra& x) {
  using viewer_detail::Out;
  const State& s = veh.state();
  const Loads l = veh.current_loads();
  const MassProps mp = veh.mass_props();
  const VehicleSpec& g = veh.spec();
  const double alt = veh.altitude();
  const Air air = veh.atmosphere(alt);
  const V3 wind = veh.wind_at(alt, veh.time());
  const double rot = veh.planet().rotation_rate;
  const V3 omega = veh.pole() * rot;
  const V3 v_air_i = rot != 0.0 ? s.v - wind - cross(omega, s.r) : s.v - wind;   // as Vehicle6::loads has it: the air moves with a rotating planet
  const V3 v_air_b = rotate_inv(s.q, v_air_i);
  double cn_alpha = 0.0;
  double x_cp = 0.0;
  double ca = 0.0;
  double s_ref = 0.0;
  veh.aero_summary(l.mach, cn_alpha, x_cp, ca, s_ref);
  const double d_ref = s_ref > 0.0 ? std::sqrt(4.0 * s_ref / kPi) : 1.0;
  const Tilts tl = veh.tilts();
  Out o;
  o.open('{');
  o.key("k");
  o.text("pose");
  o.kv("t", t_s, 3);
  o.kvi("fr", static_cast<long>(x.frame));
  o.kv("tt", veh.time(), 3);
  o.vec("r", s.r, 2);
  o.vec("v", s.v, 3);
  o.key("q");
  o.open('[');
  o.num(s.q.w, 7);
  o.num(s.q.x, 7);
  o.num(s.q.y, 7);
  o.num(s.q.z, 7);
  o.close(']');
  o.vec("w", s.w, 5);
  o.kv("alt", alt, 2);
  o.kv("rng", veh.range(), 1);
  o.kv("spd", veh.speed(), 3);
  o.kv("mach", l.mach, 4);
  o.kv("qd", l.dynamic_pressure, 1);
  o.kv("alpha", l.alpha * kRad2Deg, 4);
  o.vec("vair", v_air_b, 3);   // the vehicle's velocity through the air, in the body frame
  o.vec("wind", wind, 3);      // the wind, in the inertial frame
  o.key("air");
  o.open('[');
  o.num(air.temperature, 3);
  o.num(air.pressure, 3);
  o.num(air.density, 7);
  o.num(air.sound, 3);
  o.close(']');
  o.kv("m", s.m, 1);
  o.kv("cg", mp.x_cg, 4);
  o.kv("cp", x_cp, 4);
  o.kv("cna", cn_alpha, 4);
  o.kv("ca", ca, 4);
  o.kv("dref", d_ref, 4);
  o.vec("fth", l.f_thrust, 1);
  o.vec("fae", l.f_aero, 1);
  o.vec("mth", l.m_thrust, 1);
  o.vec("mae", l.m_aero, 1);
  o.kv("thr", l.thrust, 0);
  o.kv("mdot", l.mdot, 3);
  o.key("eng");
  o.open('[');
  for (std::size_t e = 0; e < g.engines.size(); ++e) {
    o.num(veh.engine_failed(e) ? -1.0 : veh.engine_fraction(e), 3);
  }
  o.close(']');
  o.key("gim");
  o.open('[');
  for (std::size_t st = 0; st < g.stages.size(); ++st) {
    o.num(veh.stage_gimbal_pitch_deg(st), 3);
    o.num(veh.stage_gimbal_yaw_deg(st), 3);
  }
  o.close(']');
  if (!g.fins.empty()) {
    o.key("fin");
    o.open('[');
    for (std::size_t f = 0; f < g.fins.size(); ++f) {
      o.num(veh.fin_pitch_deg(f), 3);
      o.num(veh.fin_yaw_deg(f), 3);
    }
    o.close(']');
  }
  o.key("prop");
  o.open('[');
  for (std::size_t st = 0; st < g.stages.size(); ++st) {
    o.num(veh.propellant(st), 1);
  }
  o.close(']');
  o.key("tank");
  o.open('[');
  for (std::size_t st = 0; st < g.stages.size(); ++st) {
    for (std::size_t k = 0; k < g.stages[st].tanks.size(); ++k) {
      o.num(veh.tank_liquid_kg(st, k), 1);
    }
  }
  o.close(']');
  unsigned active = 0U;
  unsigned ignited = 0U;
  for (std::size_t st = 0; st < g.stages.size() && st < kMaxStages; ++st) {
    active |= veh.stage_active(st) ? (1U << st) : 0U;
    ignited |= veh.stage_ignited(st) ? (1U << st) : 0U;
  }
  o.kvi("stg", static_cast<long>(active));
  o.kvi("ign", static_cast<long>(ignited));
  unsigned pay = 0U;
  for (std::size_t i = 0; i < g.payloads.size() && i < kMaxPayloads; ++i) {
    pay |= veh.payload_active(i) ? (1U << i) : 0U;
  }
  o.kvi("pay", static_cast<long>(pay));
  o.kvi("gnd", veh.on_ground() ? 1 : 0);
  o.kvi("crash", veh.crashed() ? 1 : 0);
  o.kvi("clamp", x.clamped ? 1 : 0);
  if (veh.slosh_count() > 0U) {
    o.key("slosh");
    o.open('[');
    for (std::size_t i = 0; i < veh.slosh_count(); ++i) {
      o.num(s.slosh[i][0], 5);
      o.num(s.slosh[i][1], 5);
    }
    o.close(']');
  }
  if (g.flex.enabled) {
    o.key("flex");
    o.open('[');
    o.num(s.flex[0], 5);
    o.num(s.flex[1], 5);
    o.close(']');
  }
  o.key("tilt");
  o.open('[');
  o.num(tl.y_deg, 4);
  o.num(tl.x_deg, 4);
  o.close(']');
  o.key("ref");
  o.open('[');
  o.num(x.ref_pitch_deg, 4);
  o.num(x.ref_yaw_deg, 4);
  o.close(']');
  o.key("cmd");
  o.open('[');
  o.num(x.cmd_pitch_deg, 3);
  o.num(x.cmd_yaw_deg, 3);
  o.close(']');
  o.kvi("act", x.act_mode);
  o.close('}');
  return o.str();
}

}  // namespace sim
