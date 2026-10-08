// SPDX-License-Identifier: MIT
// Reading and writing vehicle description files (docs/design/VEHICLE_SPEC.md): a JSON file says what a rocket or a spacecraft is, and this turns it into the `Params`, `Scenario` and design `Plan`
// the simulator runs. Every field is checked (its type, its range, whether the file names a stage that exists) and every unknown field is refused, with the nearest known name suggested:
// a typo that silently flew a different rocket would be worse than no file. All the problems are reported at once, each with the line and column of the value.
#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "design.hpp"
#include "json.hpp"
#include "vehicle6.hpp"

namespace sim {

// Everything a vehicle file says.
struct VehicleFile {
  std::string name;
  std::string description;
  Params params;
  Scenario scenario;
  Plan plan;
};

namespace detail {

inline std::size_t edit_distance(const std::string& a, const std::string& b) {
  std::vector<std::size_t> prev(b.size() + 1U);
  std::vector<std::size_t> cur(b.size() + 1U);
  for (std::size_t j = 0; j <= b.size(); ++j) {
    prev[j] = j;
  }
  for (std::size_t i = 1; i <= a.size(); ++i) {
    cur[0] = i;
    for (std::size_t j = 1; j <= b.size(); ++j) {
      cur[j] = std::min({prev[j] + 1U, cur[j - 1U] + 1U, prev[j - 1U] + (a[i - 1U] == b[j - 1U] ? 0U : 1U)});
    }
    prev = cur;
  }
  return prev[b.size()];
}

// A reader of one JSON object: it knows where it is (for the messages), marks the fields it was asked for, and at the end refuses the ones it was not.
class ObjectReader {
 public:
  ObjectReader(const Json& j, std::string path, std::vector<std::string>& errors) : j_(j), path_(std::move(path)), errors_(errors) {
    if (j.type != Json::Type::Object) {
      error(j, "", "expected an object { ... }");
      ok_ = false;
    }
    used_.assign(j.fields.size(), false);
  }

  [[nodiscard]] bool ok() const { return ok_; }
  [[nodiscard]] const std::string& path() const { return path_; }

  void error(const Json& at, const std::string& key, const std::string& what) {
    errors_.push_back((path_.empty() ? std::string() : path_) + (key.empty() ? std::string() : (path_.empty() ? "" : ".") + key) + ": " + what + " (line " + std::to_string(at.line) + ", column " +
                      std::to_string(at.col) + ")");
  }

  [[nodiscard]] const Json* get(const std::string& key) {
    known_.push_back(key);
    if (!ok_) {
      return nullptr;
    }
    for (std::size_t i = 0; i < j_.fields.size(); ++i) {
      if (j_.fields[i].first == key) {
        used_[i] = true;
        return &j_.fields[i].second;
      }
    }
    return nullptr;
  }

  bool num(const std::string& key, double& out) {
    const Json* v = get(key);
    if (v == nullptr) {
      return false;
    }
    if (v->type != Json::Type::Number) {
      error(*v, key, "expected a number");
      return false;
    }
    out = v->number;
    return true;
  }
  bool integer(const std::string& key, int& out) {
    double d = 0.0;
    const Json* v = get(key);
    if (v == nullptr) {
      return false;
    }
    if (v->type != Json::Type::Number || v->number != std::floor(v->number) || std::fabs(v->number) > 1.0e9) {
      error(*v, key, "expected a whole number");
      return false;
    }
    d = v->number;
    out = static_cast<int>(d);
    return true;
  }
  bool flag(const std::string& key, bool& out) {
    const Json* v = get(key);
    if (v == nullptr) {
      return false;
    }
    if (v->type != Json::Type::Bool) {
      error(*v, key, "expected true or false");
      return false;
    }
    out = v->boolean;
    return true;
  }
  bool str(const std::string& key, std::string& out) {
    const Json* v = get(key);
    if (v == nullptr) {
      return false;
    }
    if (v->type != Json::Type::String) {
      error(*v, key, "expected a string");
      return false;
    }
    out = v->text;
    return true;
  }
  bool vec3(const std::string& key, V3& out) {
    const Json* v = get(key);
    if (v == nullptr) {
      return false;
    }
    if (v->type != Json::Type::Array || v->items.size() != 3U || v->items[0].type != Json::Type::Number || v->items[1].type != Json::Type::Number ||
        v->items[2].type != Json::Type::Number) {
      error(*v, key, "expected three numbers [x, y, z]");
      return false;
    }
    out = V3{v->items[0].number, v->items[1].number, v->items[2].number};
    return true;
  }
  // an array of [a, b] pairs (a throttle table, a pitch program)
  bool pairs(const std::string& key, std::vector<std::array<double, 2>>& out) {
    const Json* v = get(key);
    if (v == nullptr) {
      return false;
    }
    if (v->type != Json::Type::Array) {
      error(*v, key, "expected a list of [time, value] pairs");
      return false;
    }
    out.clear();
    for (std::size_t i = 0; i < v->items.size(); ++i) {
      const Json& it = v->items[i];
      if (it.type != Json::Type::Array || it.items.size() != 2U || it.items[0].type != Json::Type::Number || it.items[1].type != Json::Type::Number) {
        error(it, key + "[" + std::to_string(i) + "]", "expected a pair of numbers [time, value]");
        return false;
      }
      out.push_back({it.items[0].number, it.items[1].number});
    }
    return true;
  }
  // an array of objects: the elements, each as a reader with its own path
  bool list(const std::string& key, std::vector<const Json*>& out) {
    const Json* v = get(key);
    if (v == nullptr) {
      return false;
    }
    if (v->type != Json::Type::Array) {
      error(*v, key, "expected a list [ ... ]");
      return false;
    }
    out.clear();
    for (const Json& it : v->items) {
      out.push_back(&it);
    }
    return true;
  }

  // Refuse what was not asked for.
  void finish() {
    if (!ok_) {
      return;
    }
    for (std::size_t i = 0; i < j_.fields.size(); ++i) {
      if (used_[i]) {
        continue;
      }
      std::string best;
      std::size_t best_d = 4U;  // only a close name is worth suggesting
      for (const std::string& k : known_) {
        const std::size_t d = edit_distance(j_.fields[i].first, k);
        if (d < best_d) {
          best_d = d;
          best = k;
        }
      }
      error(j_.fields[i].second, j_.fields[i].first, "unknown field" + (best.empty() ? std::string() : " (did you mean \"" + best + "\"?)"));
    }
  }

 private:
  const Json& j_;
  std::string path_;
  std::vector<std::string>& errors_;
  std::vector<bool> used_;
  std::vector<std::string> known_;
  bool ok_ = true;
};

inline std::string child(const std::string& path, const std::string& key) { return path.empty() ? key : path + "." + key; }
inline std::string item(const std::string& path, const std::string& key, std::size_t i) { return child(path, key) + "[" + std::to_string(i) + "]"; }

inline void read_tank(const Json& j, const std::string& path, TankSpec& t, std::vector<std::string>& errors) {
  ObjectReader o(j, path, errors);
  o.num("propellant_kg", t.propellant);
  o.num("x_bottom_m", t.x_bottom);
  o.num("radius_m", t.radius);
  o.num("density_kg_m3", t.density);
  o.finish();
}

inline void read_stage(const Json& j, const std::string& path, StageSpec& st, std::vector<std::string>& errors) {
  ObjectReader o(j, path, errors);
  o.str("name", st.name);
  o.num("dry_mass_kg", st.dry_mass);
  o.num("x_start_m", st.x_start);
  o.num("length_m", st.length);
  o.num("radius_m", st.radius);
  o.num("inertia_factor", st.inertia_factor);
  o.num("x_cg_dry_m", st.x_cg_dry);
  std::vector<const Json*> tanks;
  if (o.list("tanks", tanks)) {
    for (std::size_t i = 0; i < tanks.size(); ++i) {
      TankSpec t;
      read_tank(*tanks[i], item(path, "tanks", i), t, errors);
      st.tanks.push_back(t);
    }
  }
  o.flag("sequential_drain", st.sequential_drain);
  o.num("ignite_time_s", st.ignite_time_s);
  o.integer("ignite_after_sep_of", st.ignite_after_sep_of);
  o.num("ignite_delay_s", st.ignite_delay_s);
  o.num("separate_time_s", st.separate_time_s);
  o.flag("separate_on_burnout", st.separate_on_burnout);
  o.num("separate_delay_s", st.separate_delay_s);
  o.num("gimbal_limit_deg", st.gimbal_limit_deg);
  o.num("gimbal_rate_dps", st.gimbal_rate_dps);
  o.num("gimbal_lag_s", st.gimbal_lag_s);
  o.pairs("throttle", st.throttle);
  o.finish();
}

inline bool read_control(const std::string& s, Control& c) {
  if (s == "none") {
    c = Control::None;
  } else if (s == "pitch+") {
    c = Control::PitchPlus;
  } else if (s == "pitch-") {
    c = Control::PitchMinus;
  } else if (s == "yaw+") {
    c = Control::YawPlus;
  } else if (s == "yaw-") {
    c = Control::YawMinus;
  } else {
    return false;
  }
  return true;
}

inline const char* control_name(Control c) {
  switch (c) {
    case Control::PitchPlus: return "pitch+";
    case Control::PitchMinus: return "pitch-";
    case Control::YawPlus: return "yaw+";
    case Control::YawMinus: return "yaw-";
    case Control::None: break;
  }
  return "none";
}

// An engine entry may stand for several: "count" engines, all at the position, or spread evenly on a circle of "ring_radius_m" around it (starting at "ring_start_deg" from +Y toward +Z).
inline void read_engines(const Json& j, const std::string& path, std::vector<EngineSpec>& out, std::vector<std::string>& errors) {
  ObjectReader o(j, path, errors);
  EngineSpec e;
  o.integer("stage", e.stage);
  o.vec3("position_m", e.pos);
  o.num("thrust_vac_n", e.thrust_vac);
  o.num("exit_area_m2", e.exit_area);
  o.num("isp_vac_s", e.isp_vac);
  o.flag("gimbal", e.gimbal);
  o.num("cant_pitch_deg", e.cant_pitch_deg);
  o.num("cant_yaw_deg", e.cant_yaw_deg);
  o.num("start_offset_s", e.start_offset_s);
  o.num("cutoff_time_s", e.cutoff_time_s);
  o.num("rise_s", e.rise_s);
  o.num("tail_s", e.tail_s);
  o.vec3("direction", e.dir);
  std::string control;
  if (o.str("control", control) && !read_control(control, e.control)) {
    o.error(*o.get("control"), "control", "expected \"none\", \"pitch+\", \"pitch-\", \"yaw+\" or \"yaw-\"");
  }
  o.num("full_cmd_deg", e.full_cmd_deg);
  int count = 1;
  o.integer("count", count);
  double ring_r = 0.0;
  double ring_start = 0.0;
  o.num("ring_radius_m", ring_r);
  o.num("ring_start_deg", ring_start);
  o.finish();
  if (count < 1 || count > static_cast<int>(kMaxEngines)) {
    errors.push_back(path + ".count: expected 1 to " + std::to_string(kMaxEngines));
    return;
  }
  for (int i = 0; i < count; ++i) {
    EngineSpec k = e;
    if (ring_r > 0.0) {
      const double a = (ring_start * kDeg2Rad) + (2.0 * kPi * i / count);
      k.pos = V3{e.pos.x, e.pos.y + (ring_r * std::cos(a)), e.pos.z + (ring_r * std::sin(a))};
    }
    out.push_back(k);
  }
}

inline void read_fin(const Json& j, const std::string& path, FinSpec& f, std::vector<std::string>& errors) {
  ObjectReader o(j, path, errors);
  o.str("name", f.name);
  o.integer("stage", f.stage);
  o.num("x_hinge_m", f.x_hinge);
  o.num("area_each_m2", f.area_each);
  o.num("lift_slope", f.lift_slope);
  o.num("gain", f.gain);
  o.num("limit_deg", f.limit_deg);
  o.num("rate_dps", f.rate_dps);
  o.num("lag_s", f.lag_s);
  o.finish();
}

inline void read_plan(const Json& j, const std::string& path, Plan& plan, std::vector<std::string>& errors) {
  ObjectReader o(j, path, errors);
  DesignConfig& d = plan.trajectory;
  o.num("t_end_s", d.t_end);
  o.num("dt_s", d.dt);
  o.num("vertical_s", d.vertical_s);
  o.num("kick_ramp_s", d.kick_ramp_s);
  o.num("kick_deg", d.kick_deg);
  o.num("follow_from_s", d.follow_from_s);
  o.num("blend_s", d.blend_s);
  o.pairs("program", d.table);
  const Json* g = o.get("gains");
  if (g != nullptr) {
    ObjectReader go(*g, child(path, "gains"), errors);
    go.num("every_s", plan.gains.every_s);
    go.num("wn", plan.gains.wn);
    go.num("zeta", plan.gains.zeta);
    go.num("ki_over_kp", plan.gains.ki_over_kp);
    go.num("kp_max", plan.gains.kp_max);
    go.num("b_min", plan.gains.b_min);
    go.num("tolerance", plan.gains.tolerance);
    go.finish();
  }
  o.finish();
}

}  // namespace detail

// Read a vehicle from the text of a file. True if it is a usable description; otherwise `errors` says what is wrong, each with its place.
inline bool read_vehicle(const std::string& text, VehicleFile& out, std::vector<std::string>& errors) {
  using detail::child;
  using detail::item;
  using detail::ObjectReader;
  Json root;
  std::string perr;
  if (!parse_json(text, root, perr)) {
    errors.push_back(perr);
    return false;
  }
  const std::size_t before = errors.size();
  out = VehicleFile{};
  Params& p = out.params;
  ObjectReader top(root, "", errors);
  top.str("name", out.name);
  top.str("description", out.description);
  if (const Json* v = top.get("vehicle")) {  // the vehicle-wide settings and dispersions
    ObjectReader o(*v, "vehicle", errors);
    o.num("gimbal_limit_deg", p.gimbal_limit_deg);
    o.num("gimbal_rate_dps", p.gimbal_rate_dps);
    o.num("gimbal_lag_s", p.gimbal_lag_s);
    o.flag("ideal_roll_control", p.ideal_roll_control);
    o.flag("ground_contact", p.ground_contact);
    o.num("crash_speed_ms", p.crash_speed_ms);
    o.num("max_substep_s", p.max_substep);
    o.num("thrust_scale", p.thrust_scale);
    o.num("cd_scale", p.cd_scale);
    o.num("cn_scale", p.cn_scale);
    o.num("thrust_misalign_pitch_deg", p.thrust_misalign_pitch_deg);
    o.num("thrust_misalign_yaw_deg", p.thrust_misalign_yaw_deg);
    o.finish();
  }
  if (const Json* v = top.get("aero")) {
    ObjectReader o(*v, "aero", errors);
    o.num("diameter_m", p.diameter);
    o.num("c_n_alpha", p.c_n_alpha);
    o.num("x_cp_m", p.x_cp);
    o.finish();
  }
  std::vector<const Json*> list;
  if (top.list("stages", list)) {
    for (std::size_t i = 0; i < list.size(); ++i) {
      StageSpec st;
      detail::read_stage(*list[i], item("", "stages", i), st, errors);
      p.spec.stages.push_back(st);
    }
  }
  if (top.list("engines", list)) {
    for (std::size_t i = 0; i < list.size(); ++i) {
      detail::read_engines(*list[i], item("", "engines", i), p.spec.engines, errors);
    }
  }
  if (top.list("payloads", list)) {
    for (std::size_t i = 0; i < list.size(); ++i) {
      ObjectReader o(*list[i], item("", "payloads", i), errors);
      PayloadSpec pl;
      o.str("name", pl.name);
      o.num("mass_kg", pl.mass);
      o.num("x_m", pl.x);
      o.num("jettison_time_s", pl.jettison_time_s);
      o.finish();
      p.spec.payloads.push_back(pl);
    }
  }
  if (top.list("fins", list)) {
    for (std::size_t i = 0; i < list.size(); ++i) {
      FinSpec f;
      detail::read_fin(*list[i], item("", "fins", i), f, errors);
      p.spec.fins.push_back(f);
    }
  }
  if (const Json* v = top.get("wheels")) {
    ObjectReader o(*v, "wheels", errors);
    WheelSpec& w = p.spec.wheels;
    w.enabled = true;
    o.integer("stage", w.stage);
    o.num("torque_max_nm", w.torque_max);
    o.num("momentum_max_nms", w.momentum_max);
    o.num("full_cmd_deg", w.full_cmd_deg);
    o.finish();
  }
  if (const Json* v = top.get("scenario")) {
    ObjectReader o(*v, "scenario", errors);
    Scenario& sc = out.scenario;
    o.num("wind_scale", sc.wind_scale);
    o.vec3("wind_direction", sc.wind_dir);
    o.num("dry_cg_shift_m", sc.dry_cg_shift);
    std::vector<const Json*> gl;
    if (o.list("gusts", gl)) {
      for (std::size_t i = 0; i < gl.size(); ++i) {
        ObjectReader g(*gl[i], item("scenario", "gusts", i), errors);
        Gust gu;
        g.num("t0_s", gu.t0);
        g.num("duration_s", gu.duration);
        g.vec3("peak_ms", gu.peak);
        g.finish();
        sc.gusts.push_back(gu);
      }
    }
    if (o.list("engine_failures", gl)) {
      for (std::size_t i = 0; i < gl.size(); ++i) {
        ObjectReader g(*gl[i], item("scenario", "engine_failures", i), errors);
        EngineFailure f;
        g.num("time_s", f.time);
        g.integer("engine", f.index);
        g.finish();
        sc.engine_failures.push_back(f);
      }
    }
    if (const Json* st = o.get("start")) {  // start in flight: a circular orbit, or a state
      ObjectReader s(*st, "scenario.start", errors);
      sc.has_initial = true;
      double altitude = 0.0;
      bool circular = false;
      s.num("altitude_m", altitude);
      s.flag("circular_orbit", circular);
      V3 pos{kEarthR + altitude, 0.0, 0.0};
      V3 vel{};
      s.vec3("position_m", pos);
      s.vec3("velocity_ms", vel);
      if (circular) {
        vel = V3{0.0, std::sqrt(kEarthMu / norm(pos)), 0.0};
      }
      V3 rates{};
      s.vec3("rates_dps", rates);
      sc.initial.r = pos;
      sc.initial.v = vel;
      sc.initial.w = rates * kDeg2Rad;
      s.finish();
    }
    o.finish();
  }
  if (const Json* v = top.get("design")) {
    detail::read_plan(*v, "design", out.plan, errors);
  }
  top.finish();
  if (errors.size() == before) {
    for (const std::string& m : validate(p.spec)) {
      errors.push_back(m);
    }
    if (!(p.gimbal_limit_deg > 0.0) || !(p.gimbal_rate_dps > 0.0) || !(p.diameter > 0.0) || !(p.max_substep > 0.0) || !(out.plan.trajectory.t_end > 0.0) || !(out.plan.trajectory.dt > 0.0)) {
      errors.push_back("vehicle/aero/design: gimbal_limit_deg, gimbal_rate_dps, diameter_m, max_substep_s, t_end_s and dt_s must be positive");
    }
    if (p.spec.stages.empty()) {
      errors.push_back("stages: a vehicle needs at least one stage");
    }
  }
  return errors.size() == before;
}

inline bool load_vehicle_file(const std::string& path, VehicleFile& out, std::vector<std::string>& errors) {
  std::ifstream f(path);
  if (!f) {
    errors.push_back(path + ": cannot open the file");
    return false;
  }
  std::stringstream ss;
  ss << f.rdbuf();
  const std::size_t before = errors.size();
  const bool ok = read_vehicle(ss.str(), out, errors);
  for (std::size_t i = before; i < errors.size(); ++i) {
    errors[i] = path + ": " + errors[i];
  }
  return ok;
}

namespace detail {

// The shortest text that reads back as the same double.
inline std::string num_text(double v) {
  std::array<char, 40> b{};
  for (int digits = 6; digits <= 17; ++digits) {
    (void)std::snprintf(b.data(), b.size(), "%.*g", digits, v);
    if (std::strtod(b.data(), nullptr) == v) {
      break;
    }
  }
  return std::string(b.data());
}
inline std::string quoted(const std::string& s) {
  std::string o = "\"";
  for (const char c : s) {
    if (c == '"' || c == '\\') {
      o.push_back('\\');
    }
    o.push_back(c);
  }
  return o + "\"";
}
inline std::string vec_text(const V3& v) { return "[" + num_text(v.x) + ", " + num_text(v.y) + ", " + num_text(v.z) + "]"; }
inline std::string pairs_text(const std::vector<std::array<double, 2>>& t) {
  std::string s = "[";
  for (std::size_t i = 0; i < t.size(); ++i) {
    s += (i == 0U ? "" : ", ") + std::string("[") + num_text(t[i][0]) + ", " + num_text(t[i][1]) + "]";
  }
  return s + "]";
}

}  // namespace detail

// A vehicle as the text of a file that read_vehicle reads back as the same vehicle. A `Params` without a spec is written out as the stage, tank and engines it stands for.
inline std::string write_vehicle(const VehicleFile& v) {
  using namespace detail;
  const Params& p = v.params;
  const VehicleSpec g = p.spec.empty() ? legacy_spec(p) : p.spec;
  std::string o = "{\n";
  o += "  \"name\": " + quoted(v.name) + ",\n  \"description\": " + quoted(v.description) + ",\n";
  o += "  \"vehicle\": {\"gimbal_limit_deg\": " + num_text(p.gimbal_limit_deg) + ", \"gimbal_rate_dps\": " + num_text(p.gimbal_rate_dps) + ", \"gimbal_lag_s\": " + num_text(p.gimbal_lag_s) +
       ", \"ideal_roll_control\": " + (p.ideal_roll_control ? "true" : "false") + ", \"ground_contact\": " + (p.ground_contact ? "true" : "false") + ", \"crash_speed_ms\": " +
       num_text(p.crash_speed_ms) + ", \"max_substep_s\": " + num_text(p.max_substep) + ", \"thrust_scale\": " + num_text(p.thrust_scale) + ", \"cd_scale\": " + num_text(p.cd_scale) +
       ", \"cn_scale\": " + num_text(p.cn_scale) + ", \"thrust_misalign_pitch_deg\": " + num_text(p.thrust_misalign_pitch_deg) + ", \"thrust_misalign_yaw_deg\": " +
       num_text(p.thrust_misalign_yaw_deg) + "},\n";
  o += "  \"aero\": {\"diameter_m\": " + num_text(p.diameter) + ", \"c_n_alpha\": " + num_text(p.c_n_alpha) + ", \"x_cp_m\": " + num_text(p.x_cp) + "},\n";
  o += "  \"stages\": [\n";
  for (std::size_t s = 0; s < g.stages.size(); ++s) {
    const StageSpec& st = g.stages[s];
    o += "    {\"name\": " + quoted(st.name) + ", \"dry_mass_kg\": " + num_text(st.dry_mass) + ", \"x_start_m\": " + num_text(st.x_start) + ", \"length_m\": " + num_text(st.length) +
         ", \"radius_m\": " + num_text(st.radius) + ", \"inertia_factor\": " + num_text(st.inertia_factor) + ", \"x_cg_dry_m\": " + num_text(st.x_cg_dry) + ",\n     \"tanks\": [";
    for (std::size_t k = 0; k < st.tanks.size(); ++k) {
      const TankSpec& t = st.tanks[k];
      o += std::string(k == 0U ? "" : ", ") + "{\"propellant_kg\": " + num_text(t.propellant) + ", \"x_bottom_m\": " + num_text(t.x_bottom) + ", \"radius_m\": " + num_text(t.radius) +
           ", \"density_kg_m3\": " + num_text(t.density) + "}";
    }
    o += "], \"sequential_drain\": " + std::string(st.sequential_drain ? "true" : "false") + ",\n     \"ignite_time_s\": " + num_text(st.ignite_time_s) + ", \"ignite_after_sep_of\": " +
         std::to_string(st.ignite_after_sep_of) + ", \"ignite_delay_s\": " + num_text(st.ignite_delay_s) + ", \"separate_time_s\": " + num_text(st.separate_time_s) +
         ", \"separate_on_burnout\": " + (st.separate_on_burnout ? "true" : "false") + ", \"separate_delay_s\": " + num_text(st.separate_delay_s) + ",\n     \"gimbal_limit_deg\": " +
         num_text(st.gimbal_limit_deg) + ", \"gimbal_rate_dps\": " + num_text(st.gimbal_rate_dps) + ", \"gimbal_lag_s\": " + num_text(st.gimbal_lag_s);
    if (!st.throttle.empty()) {
      o += ", \"throttle\": " + pairs_text(st.throttle);
    }
    o += std::string("}") + (s + 1U < g.stages.size() ? "," : "") + "\n";
  }
  o += "  ],\n  \"engines\": [\n";
  for (std::size_t e = 0; e < g.engines.size(); ++e) {
    const EngineSpec& en = g.engines[e];
    o += "    {\"stage\": " + std::to_string(en.stage) + ", \"position_m\": " + vec_text(en.pos) + ", \"thrust_vac_n\": " + num_text(en.thrust_vac) + ", \"exit_area_m2\": " +
         num_text(en.exit_area) + ", \"isp_vac_s\": " + num_text(en.isp_vac) + ", \"gimbal\": " + (en.gimbal ? "true" : "false") + ", \"cant_pitch_deg\": " + num_text(en.cant_pitch_deg) +
         ", \"cant_yaw_deg\": " + num_text(en.cant_yaw_deg) + ", \"start_offset_s\": " + num_text(en.start_offset_s) + ", \"cutoff_time_s\": " + num_text(en.cutoff_time_s) +
         ", \"rise_s\": " + num_text(en.rise_s) + ", \"tail_s\": " + num_text(en.tail_s) + ", \"direction\": " + vec_text(en.dir) + ", \"control\": " + quoted(control_name(en.control)) +
         ", \"full_cmd_deg\": " + num_text(en.full_cmd_deg) + "}" + (e + 1U < g.engines.size() ? "," : "") + "\n";
  }
  o += "  ],\n  \"payloads\": [";
  for (std::size_t i = 0; i < g.payloads.size(); ++i) {
    const PayloadSpec& pl = g.payloads[i];
    o += std::string(i == 0U ? "" : ", ") + "{\"name\": " + quoted(pl.name) + ", \"mass_kg\": " + num_text(pl.mass) + ", \"x_m\": " + num_text(pl.x) + ", \"jettison_time_s\": " +
         num_text(pl.jettison_time_s) + "}";
  }
  o += "],\n  \"fins\": [";
  for (std::size_t i = 0; i < g.fins.size(); ++i) {
    const FinSpec& f = g.fins[i];
    o += std::string(i == 0U ? "" : ", ") + "{\"name\": " + quoted(f.name) + ", \"stage\": " + std::to_string(f.stage) + ", \"x_hinge_m\": " + num_text(f.x_hinge) + ", \"area_each_m2\": " +
         num_text(f.area_each) + ", \"lift_slope\": " + num_text(f.lift_slope) + ", \"gain\": " + num_text(f.gain) + ", \"limit_deg\": " + num_text(f.limit_deg) + ", \"rate_dps\": " +
         num_text(f.rate_dps) + ", \"lag_s\": " + num_text(f.lag_s) + "}";
  }
  o += "]";
  if (g.wheels.enabled) {
    o += ",\n  \"wheels\": {\"stage\": " + std::to_string(g.wheels.stage) + ", \"torque_max_nm\": " + num_text(g.wheels.torque_max) + ", \"momentum_max_nms\": " + num_text(g.wheels.momentum_max) +
         ", \"full_cmd_deg\": " + num_text(g.wheels.full_cmd_deg) + "}";
  }
  const Scenario& sc = v.scenario;
  o += ",\n  \"scenario\": {\"wind_scale\": " + num_text(sc.wind_scale) + ", \"wind_direction\": " + vec_text(sc.wind_dir) + ", \"dry_cg_shift_m\": " + num_text(sc.dry_cg_shift) + ", \"gusts\": [";
  for (std::size_t i = 0; i < sc.gusts.size(); ++i) {
    o += std::string(i == 0U ? "" : ", ") + "{\"t0_s\": " + num_text(sc.gusts[i].t0) + ", \"duration_s\": " + num_text(sc.gusts[i].duration) + ", \"peak_ms\": " + vec_text(sc.gusts[i].peak) + "}";
  }
  o += "], \"engine_failures\": [";
  std::vector<EngineFailure> failures = sc.engine_failures;
  if (sc.engine_out_time >= 0.0) {
    failures.insert(failures.begin(), EngineFailure{sc.engine_out_time, sc.engine_out_index});
  }
  for (std::size_t i = 0; i < failures.size(); ++i) {
    o += std::string(i == 0U ? "" : ", ") + "{\"time_s\": " + num_text(failures[i].time) + ", \"engine\": " + std::to_string(failures[i].index) + "}";
  }
  o += "]";
  if (sc.has_initial) {
    o += ", \"start\": {\"position_m\": " + vec_text(sc.initial.r) + ", \"velocity_ms\": " + vec_text(sc.initial.v) + ", \"rates_dps\": " + vec_text(sc.initial.w * kRad2Deg) + "}";
  }
  o += "},\n";
  const Plan& pl = v.plan;
  const DesignConfig& d = pl.trajectory;
  o += "  \"design\": {\"t_end_s\": " + num_text(d.t_end) + ", \"dt_s\": " + num_text(d.dt) + ", \"vertical_s\": " + num_text(d.vertical_s) + ", \"kick_ramp_s\": " + num_text(d.kick_ramp_s) +
       ", \"kick_deg\": " + num_text(d.kick_deg) + ", \"follow_from_s\": " + num_text(d.follow_from_s) + ", \"blend_s\": " + num_text(d.blend_s);
  if (!d.table.empty()) {
    o += ", \"program\": " + pairs_text(d.table);
  }
  o += ", \"gains\": {\"every_s\": " + num_text(pl.gains.every_s) + ", \"wn\": " + num_text(pl.gains.wn) + ", \"zeta\": " + num_text(pl.gains.zeta) + ", \"ki_over_kp\": " + num_text(pl.gains.ki_over_kp) +
       ", \"kp_max\": " + num_text(pl.gains.kp_max) + ", \"b_min\": " + num_text(pl.gains.b_min) + ", \"tolerance\": " + num_text(pl.gains.tolerance) + "}}\n}\n";
  return o;
}

}  // namespace sim
