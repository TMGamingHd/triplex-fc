// SPDX-License-Identifier: MIT
// tfc_cfd: the Euler flow solver (sim/cfd, docs/design/CFD.md) on a vehicle's body, and its validation against the exact solutions.
//
//   tfc_cfd --validate [--quick] [--only sod|wedge|cone|sphere|gas|viscous] [--threads N]   the solver against Sod's tube, a wedge, cones and a sphere, with the errors and a verdict
//   tfc_cfd VEHICLE.json --mach M [--alpha DEG] ...     one flow: the forces on the body of the vehicle (its stages from --stage-from on) as coefficients
//   tfc_cfd VEHICLE.json --machs 2,3,5 --alpha 4 --table OUT.json   a table of CA, CN-alpha and the centre of pressure by Mach number, in the form of `aero.table`
//
// Options for a flow: --h M (the cells of the core, default a sixth of the body's radius), --iters N, --orders R (stop when the density residual has fallen so far), --threads N, --xref M (the point the
// moment is about, from the aft end), --stage-from K (0: the whole stack), --field FILE (the flow for the viewer), --probe extrap|CELLS (how the surface pressure is read: extrapolated from 0.5 and 1 cell off, or at CELLS cells off; default extrap), --json FILE.
// The gas is calorically perfect (gamma 1.4) and the flow inviscid: see the document for what that does to a number.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "gasdyn.hpp"
#include "solver.hpp"
#include "spec_io.hpp"
#include "atmosphere.hpp"
#include "vehicle_body.hpp"
#include "vehicle_surfaces.hpp"
#include "viscous.hpp"

namespace {

using sim::V3;
namespace cfd = sim::cfd;
namespace gd = sim::cfd::gd;
namespace visc = sim::cfd::visc;
constexpr double kDeg = 3.14159265358979323846 / 180.0;
constexpr double kGamma = 1.4;

double now_s() { return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count(); }

// ------------------------------------------------------------------------------------------------ a flow past a body of revolution

struct FlowCase {
  double mach = 3.0;
  double alpha_deg = 0.0;
  double h = 0.0;               // the core cell; 0: a sixth of the body's radius
  int iters = 4000;
  double orders = 5.0;
  int threads = 0;
  double x_ref = -1.0;          // m from the aft end; negative: 40 % of the length
  cfd::Config cfg;
  double margin_d = 1.5;        // the domain's extent beyond the body in the flow direction, in diameters
  double lateral_r = 6.0;       // the domain's extent to the side, in radii
  bool verbose = true;
  int report_every = 250;
  std::string field_path;       // where to write the flow field for the viewer (empty: none)
  double fs_km = 30.0;          // the altitude of the free stream (the standard atmosphere) for everything that needs a temperature and a density: the viscous layer, the heating, the real gas
  bool viscous = false;         // add the boundary layer's friction and heating (viscous.hpp) to the Euler solution
  double wall_k = -1.0;         // the wall temperature, K; negative: radiative equilibrium at `emissivity`
  double emissivity = 0.85;
  double re_transition = 1.0e6; // the edge Reynolds number (on the running length) at which the layer begins to turn turbulent
  bool base_model = true;       // true: the base drag of the build-up's law (0.25 / M, less with the engines on) in place of the Euler solution's, which has no wake
  double power = 0.0;           // the fraction of the engines' thrust that is running (the plume fills the wake)
  bool real = false;            // the real gas: vibrating, dissociating air in equilibrium, at the free stream's altitude; otherwise the perfect gas
  std::vector<cfd::Porous> lattices;            // the grid fins, as porous boxes
  std::vector<std::string> lattice_names;
  std::vector<std::string> part_names;          // by part number: 0 the skin, 1 the base, 10 + i the surface i of the file
  std::vector<int> part_ids;
  double r_extent = 0.0;        // the largest radius of anything on the vehicle, for the grid's core (0: the body's)
  std::string vehicle;          // the vehicle's name and the first stage of the body, for the header of that file
  std::size_t stage_from = 0U;
};

struct FlowResult {
  bool ok = false;
  double ca_skin = 0.0;         // axial force coefficient, drag positive, of the skin (the body without its base)
  double ca_base = 0.0;         // the base's own part: the base-drag law's, or (--base euler) the Euler solution's pressure behind the body
  double ca_base_euler = 0.0;   // what the Euler solution itself has there, which has no wake and means little (both flat ends)
  double ca_fore = 0.0;         // the pressure force on a flat forward end alone, when the body has one (the booster without the ship): windward at a small angle of attack
  double cm_fore = 0.0;         // and its moment
  double ca_friction = 0.0;     // the skin friction's axial coefficient (zero without --viscous)
  double cn_friction = 0.0;
  double heat_peak_kw = 0.0;    // the highest heat flux into the skin, kW/m^2 (the stagnation region excluded: its flat-plate law is not good there)
  double heat_total_mw = 0.0;   // the heat rate into the whole skin, MW
  double t_wall_max_k = 0.0;
  double t_wall_mean_k = 0.0;
  double laminar_fraction = 0.0;
  struct Part {
    std::string name;
    double ca = 0.0;
    double cn = 0.0;
    double cm = 0.0;
  };
  std::vector<Part> parts;      // the flaps and the grid fins apart (their share of the totals above, which include them)
  double cn = 0.0;              // normal force coefficient (positive: toward the side the flow comes from)
  double x_cp = 0.0;            // m from the aft end: where the skin's force acts, by the moment about x_ref
  double cm = 0.0;              // moment coefficient about x_ref on the reference diameter (positive nose up)
  double residual_drop = 0.0;
  int iterations = 0;
  std::size_t cells = 0;
  double seconds = 0.0;
  double s_ref = 0.0;
  double d_ref = 0.0;
  double x_ref = 0.0;
};

// The grid for a body between x0 and x1 with a maximum radius r: fine over the body and a little beyond it, growing outward to the domain's edges.
std::array<std::vector<double>, 3> make_nodes(double x0, double x1, double r_body, const FlowCase& c, double h) {
  const double r = std::max(r_body, c.r_extent > 0.0 ? c.r_extent / 1.6 : 0.0);   // the core reaches 1.6 radii: far enough for the flaps and the fins too
  const double d = 2.0 * r;
  const double lat = c.lateral_r * r;
  return {cfd::graded_nodes(x0 - (c.margin_d * d), x1 + (c.margin_d * d), x0 - (0.5 * d), x1 + (0.5 * d), h, 1.1, 4.0 * h),
          cfd::graded_nodes(-lat, lat, -1.6 * r, 1.6 * r, h, 1.1, 4.0 * h),
          cfd::graded_nodes(0.0, lat, 0.0, 1.6 * r, h, 1.1, 4.0 * h)};
}

// ------------------------------------------------------------------------------------------------ the flow field for the viewer

// A file the 3D viewer reads (docs/design/CFD.md section 7): a JSON header on one line, then the numbers. The body as its outline; the pressure coefficient on the skin on a grid of stations along the
// outline and angles round it (0 to 180 degrees: the other half is the mirror image); and the flow in a box round the body as 16-bit numbers (0 is "inside the body", 1 to 65535 the value on the scale the header gives):
// the pressure ratio and the density ratio on a logarithmic scale and the Mach number on a linear one. The viewer decodes them to colours, to a section through the flow and to a schlieren picture.
bool write_field(const std::string& path, cfd::Euler3D& s, const cfd::Shape& body, const std::vector<std::pair<double, double>>& profile, const FlowCase& c, double x0, double x1, double r_max, double d_ref,
                 const FlowResult& r, double h) {
  const double speed = c.mach * std::sqrt(kGamma);
  const double q = 0.5 * speed * speed;
  s.prepare_sampling();
  // the surface: stations along the outline, 5 cm... as fine as the grid
  const int nt = 36;
  std::vector<float> cp;
  std::vector<double> sx;
  std::vector<double> sr;
  for (std::size_t i = 0; i + 1U < profile.size(); ++i) {
    const double dx = profile[i + 1U].first - profile[i].first;
    const double dr = profile[i + 1U].second - profile[i].second;
    const int ns = std::max(1, static_cast<int>(std::ceil(std::hypot(dx, dr) / (0.7 * h))));
    for (int k = 0; k < ns; ++k) {
      const double f = (k + 0.5) / ns;
      sx.push_back(profile[i].first + (f * dx));
      sr.push_back(profile[i].second + (f * dr));
    }
  }
  for (std::size_t i = 0; i < sx.size(); ++i) {
    // the outward normal in the meridian plane of the piece this station is on
    double nx = 0.0;
    double nr = 1.0;
    for (std::size_t k = 0; k + 1U < profile.size(); ++k) {
      if (sx[i] >= profile[k].first - 1e-9 && sx[i] <= profile[k + 1U].first + 1e-9) {
        const double dx = profile[k + 1U].first - profile[k].first;
        const double dr = profile[k + 1U].second - profile[k].second;
        const double len = std::hypot(dx, dr);
        if (len > 0.0) {
          nx = dr / len;
          nr = -dx / len;
          if (body.phi(V3{sx[i] + (nx * 1e-3), sr[i] + (nr * 1e-3), 0.0}) < 0.0) {
            nx = -nx;
            nr = -nr;
          }
          break;
        }
      }
    }
    for (int j = 0; j < nt; ++j) {
      const double th = 3.14159265358979 * (j + 0.5) / nt;
      const cfd::Panel pn{V3{sx[i], sr[i] * std::cos(th), sr[i] * std::sin(th)}, V3{nx, nr * std::cos(th), nr * std::sin(th)}, 1.0, 0};
      cp.push_back(static_cast<float>((s.wall_pressure(pn) - 1.0) / q));
    }
  }
  // the volume: x along the body, y across (both signs), z from the plane of symmetry out
  const double d = 2.0 * r_max;
  const double vx0 = x0 - (0.4 * d);
  const double vx1 = x1 + (0.4 * d);
  const double vy = 2.6 * r_max;
  const double hv = std::max(h, (vx1 - vx0) / 280.0);
  const int nx = std::max(8, static_cast<int>(std::ceil((vx1 - vx0) / hv)));
  const int ny = std::max(8, static_cast<int>(std::ceil(2.0 * vy / hv)));
  const int nz = std::max(4, static_cast<int>(std::ceil(vy / hv)));
  std::vector<std::uint16_t> bp(static_cast<std::size_t>(nx) * static_cast<std::size_t>(ny) * static_cast<std::size_t>(nz), 0U);
  std::vector<std::uint16_t> bd = bp;
  std::vector<std::uint16_t> bm = bp;
  const double lp_lo = -3.0;   // log2 of the pressure over the free stream's: 1/8 to 256
  const double lp_hi = 8.0;
  const double ld_lo = -3.0;   // and of the density
  const double ld_hi = 6.0;
  const double m_hi = std::max(2.0, 1.5 * c.mach);
  auto enc = [](double v, double lo, double hi) {
    const double t = std::min(1.0, std::max(0.0, (v - lo) / (hi - lo)));
    return static_cast<std::uint16_t>(1 + static_cast<int>(std::lround(t * 65534.0)));
  };
  for (int k = 0; k < nz; ++k) {
    for (int j = 0; j < ny; ++j) {
      for (int i = 0; i < nx; ++i) {
        const V3 x{vx0 + ((i + 0.5) * hv), -vy + ((j + 0.5) * hv), (k + 0.5) * hv};
        const std::size_t at = (static_cast<std::size_t>(k) * static_cast<std::size_t>(ny) + static_cast<std::size_t>(j)) * static_cast<std::size_t>(nx) + static_cast<std::size_t>(i);
        cfd::Prim p;
        if (body.phi(x) < 0.5 * hv || !s.sample_prepared(x, p)) {
          continue;
        }
        bp[at] = enc(std::log2(std::max(p.p, 1e-9)), lp_lo, lp_hi);
        bd[at] = enc(std::log2(std::max(p.rho, 1e-9)), ld_lo, ld_hi);
        bm[at] = enc(std::sqrt((p.u * p.u) + (p.v * p.v) + (p.w * p.w)) / s.gas().sound(p.rho, p.p), 0.0, m_hi);
      }
    }
  }
  std::FILE* f = std::fopen(path.c_str(), "wb");
  if (f == nullptr) {
    std::fprintf(stderr, "cannot write %s\n", path.c_str());
    return false;
  }
  std::string vname = c.vehicle;
  vname.erase(std::remove(vname.begin(), vname.end(), '"'), vname.end());
  std::string head = "{\"k\": \"cfd\", \"v\": 1, \"vehicle\": \"" + vname + "\", \"stage_from\": " + std::to_string(c.stage_from) + ", \"mach\": " + std::to_string(c.mach) + ", \"alpha_deg\": " + std::to_string(c.alpha_deg) + ", \"gamma\": 1.4, \"h_m\": " + std::to_string(h) +
                     ", \"d_ref_m\": " + std::to_string(d_ref) + ", \"ca_skin\": " + std::to_string(r.ca_skin) + ", \"cn\": " + std::to_string(r.cn) + ", \"x_cp_m\": " + std::to_string(r.x_cp) +
                     ", \"cells\": " + std::to_string(r.cells) + ", \"iterations\": " + std::to_string(r.iterations) + ", \"residual_drop\": " + std::to_string(r.residual_drop) +
                     ", \"body\": {\"x0\": " + std::to_string(x0) + ", \"x1\": " + std::to_string(x1) + ", \"r_max\": " + std::to_string(r_max) + ", \"profile\": [";
  for (std::size_t i = 0; i < profile.size(); ++i) {
    char b[64];
    std::snprintf(b, sizeof(b), "%s[%.4f, %.4f]", i == 0U ? "" : ", ", profile[i].first, profile[i].second);
    head += b;
  }
  head += "]}, \"surface\": {\"stations\": " + std::to_string(sx.size()) + ", \"angles\": " + std::to_string(nt) + ", \"x\": [";
  for (std::size_t i = 0; i < sx.size(); ++i) {
    char b[48];
    std::snprintf(b, sizeof(b), "%s%.3f", i == 0U ? "" : ", ", sx[i]);
    head += b;
  }
  head += "]}, \"volume\": {\"bits\": 16, \"nx\": " + std::to_string(nx) + ", \"ny\": " + std::to_string(ny) + ", \"nz\": " + std::to_string(nz) + ", \"x0\": " + std::to_string(vx0) + ", \"y0\": " + std::to_string(-vy) +
          ", \"z0\": 0.0, \"h\": " + std::to_string(hv) + ", \"log2_p\": [" + std::to_string(lp_lo) + ", " + std::to_string(lp_hi) + "], \"log2_rho\": [" + std::to_string(ld_lo) + ", " + std::to_string(ld_hi) +
          "], \"mach\": [0, " + std::to_string(m_hi) + "]}}\n";
  // the layout: the header line (padded with spaces so that what follows starts on a multiple of four bytes), then the surface pressure coefficients (float32, station-major), then the three volumes
  // (uint16, little-endian, x fastest, then y, then z): pressure, density, Mach number
  while ((head.size() % 4U) != 0U) {
    head.insert(head.size() - 1U, " ");
  }
  std::fwrite(head.data(), 1, head.size(), f);
  std::fwrite(cp.data(), sizeof(float), cp.size(), f);
  std::fwrite(bp.data(), sizeof(std::uint16_t), bp.size(), f);
  std::fwrite(bd.data(), sizeof(std::uint16_t), bd.size(), f);
  std::fwrite(bm.data(), sizeof(std::uint16_t), bm.size(), f);
  std::fclose(f);
  std::printf("  wrote %s: %zu stations x %d angles of Cp, a %d x %d x %d volume of 16-bit numbers (%zu kB)\n", path.c_str(), sx.size(), nt, nx, ny, nz, (head.size() + (cp.size() * 4U) + (6U * bp.size())) / 1024U);
  return true;
}

// The boundary layer over the skin of the body (viscous.hpp): at every surface element the inviscid wall state is the layer's edge, the running length is the meridian's (Mangler's for a laminar layer) blended
// into the length round the body where the flow goes across it, and the friction and the heat follow from the reference-temperature laws. Dimensional, at the free stream's altitude.
void add_viscous(cfd::Euler3D& s, const std::vector<cfd::Panel>& panels, const std::vector<std::pair<double, double>>& profile, const FlowCase& c, double x_ref, double d_ref, FlowResult& out) {
  const sim::Air at = sim::air_at(c.fs_km * 1000.0);
  const double rgas = visc::kGasConstant;
  const double a_inf = std::sqrt(kGamma * rgas * at.temperature);
  const double v_inf = c.mach * a_inf;
  const double q_inf = 0.5 * at.density * v_inf * v_inf;
  const double u_ref = std::sqrt(rgas * at.temperature);
  const double p_ref = at.density * rgas * at.temperature;
  // the meridian from the nose: arc length, radius and Mangler's length at each station of the outline
  std::vector<double> xs;
  std::vector<double> rs;
  std::vector<double> sm;
  for (std::size_t i = profile.size(); i-- > 0U;) {
    if (!xs.empty() && std::fabs(profile[i].first - xs.back()) < 1e-9 && std::fabs(profile[i].second - rs.back()) < 1e-9) {
      continue;
    }
    sm.push_back(xs.empty() ? 0.0 : sm.back() + std::hypot(profile[i].first - xs.back(), profile[i].second - rs.back()));
    xs.push_back(profile[i].first);
    rs.push_back(profile[i].second);
  }
  const std::vector<double> mang = visc::mangler_length(sm, rs);
  s.prepare_sampling();
  double fx = 0.0;
  double fy = 0.0;
  double mz = 0.0;
  double area = 0.0;
  double lam_area = 0.0;
  double heat = 0.0;
  double t_sum = 0.0;
  double t_max = 0.0;
  double q_peak = 0.0;
  const double nose_x = xs.front();
  const double length = std::fabs(nose_x - xs.back());
  for (const cfd::Panel& pn : panels) {
    if (pn.part != 0) {
      continue;
    }
    const cfd::Prim w = s.wall_state(pn);
    V3 u{w.u, w.v, w.w};
    u = u - (pn.n * dot(u, pn.n));
    const double ut = norm(u) * u_ref;
    if (ut < 5.0) {
      continue;
    }
    // the station: the first outline point at or past the panel's x, counting from the nose
    std::size_t k = 0;
    while (k + 1U < xs.size() && xs[k + 1U] > pn.c.x) {
      ++k;
    }
    const double f = (xs[k] - pn.c.x) / std::max(xs[k] - xs[std::min(k + 1U, xs.size() - 1U)], 1e-9);
    const double x_mang = mang[k] + (f * (mang[std::min(k + 1U, xs.size() - 1U)] - mang[k]));
    const double r = std::hypot(pn.c.y, pn.c.z);
    const double theta = std::atan2(pn.c.z, pn.c.y);
    const double dxm = xs[std::min(k + 1U, xs.size() - 1U)] - xs[k];
    const double drm = rs[std::min(k + 1U, xs.size() - 1U)] - rs[k];
    const V3 mer = normalized(V3{dxm, drm * std::cos(theta), drm * std::sin(theta)});
    const double cm = std::fabs(dot(normalized(u), mer));
    const double x_eff = std::max(1e-3, (cm * x_mang) + ((1.0 - cm) * std::max(r * theta, 0.02 * length)));
    visc::Edge e;
    e.rho = w.rho * at.density;
    e.p = w.p * p_ref;
    e.u = ut;
    if (s.gas().air != nullptr) {
      e.t = s.gas().air->t_from_p(w.rho, w.p) * at.temperature;
      e.gamma = s.gas().gamma_eff(w.rho, w.p);
    } else {
      e.t = e.p / (e.rho * rgas);
    }
    double t_wall = c.wall_k;
    if (!(t_wall > 0.0)) {
      t_wall = visc::radiative_equilibrium([&](double tw) { return visc::heat_flux(e, visc::local_layer(e, x_eff, tw, c.re_transition), tw); }, c.emissivity);
    }
    const visc::Layer ly = visc::local_layer(e, x_eff, t_wall, c.re_transition);
    const double tau = ly.cf * 0.5 * e.rho * e.u * e.u;
    const V3 t = normalized(u);
    const double a2 = 2.0 * pn.area;   // the mirror image counts too
    fx += tau * t.x * a2;
    fy += tau * t.y * a2;
    mz += ((pn.c.x - x_ref) * tau * t.y - (pn.c.y * tau * t.x)) * a2;
    const double q = visc::heat_flux(e, ly, t_wall);
    area += a2;
    lam_area += (1.0 - ly.turbulence) * a2;
    heat += q * a2;
    t_sum += t_wall * a2;
    t_max = std::max(t_max, t_wall);
    if (std::fabs(pn.c.x - nose_x) > 0.03 * length) {
      q_peak = std::max(q_peak, q);
    }
  }
  const double s_ref = out.s_ref;
  out.ca_friction = -fx / (q_inf * s_ref);
  out.cn_friction = -fy / (q_inf * s_ref);
  out.cm += mz / (q_inf * s_ref * d_ref);
  out.heat_peak_kw = q_peak / 1000.0;
  out.heat_total_mw = heat / 1.0e6;
  out.t_wall_max_k = t_max;
  out.t_wall_mean_k = area > 0.0 ? t_sum / area : 0.0;
  out.laminar_fraction = area > 0.0 ? lam_area / area : 0.0;
}

FlowResult run_body(const cfd::Shape& body, double x0, double x1, double r_max, double d_ref, FlowCase c, const std::vector<std::pair<double, double>>* profile = nullptr) {
  FlowResult out;
  const double h = c.h > 0.0 ? c.h : r_max / 6.0;
  c.cfg.threads = c.threads > 0 ? c.threads : static_cast<int>(std::max(1U, std::thread::hardware_concurrency()));
  // the real gas: the temperature and density of the free stream make the characteristic temperatures non-dimensional
  std::unique_ptr<cfd::RealAir> real;
  if (c.real) {
    const sim::Air at = sim::air_at(c.fs_km * 1000.0);
    real = std::make_unique<cfd::RealAir>(3000.0 / at.temperature, 59500.0 / at.temperature, 150000.0 / at.density);
    c.cfg.gas = cfd::Gas(kGamma, real.get());
    if (c.verbose) {
      std::printf("  real gas: air in equilibrium at %.0f km (%.0f K, %.3g kg/m^3), vibrating and dissociating (CFD.md section 11)\n", c.fs_km, at.temperature, at.density);
    }
  } else {
    c.cfg.gas = cfd::Gas(kGamma);
  }
  // the flow runs toward -x (the nose is at the largest x): air velocity relative to the body, with the angle of attack in the x-y plane
  const double speed = c.mach * std::sqrt(kGamma);
  const cfd::Prim inf{1.0, -speed * std::cos(c.alpha_deg * kDeg), -speed * std::sin(c.alpha_deg * kDeg), 0.0, 1.0};
  c.cfg.bc = {cfd::Bc::Outflow, cfd::Bc::Farfield, cfd::Bc::Farfield, cfd::Bc::Farfield, cfd::Bc::Symmetry, cfd::Bc::Farfield};
  const std::array<std::vector<double>, 3> nodes = make_nodes(x0, x1, r_max, c, h);
  cfd::Euler3D s(nodes, inf, &body, c.cfg);
  s.set_porous(c.lattices);
  out.cells = s.cell_count();
  if (c.verbose) {
    std::printf("  grid %d x %d x %d = %zu cells (core %.3g m, body radius %.3g m, %.1f cells across it), %zu ghost cells, %d threads\n", s.cells(0), s.cells(1), s.cells(2), s.cell_count(), h, r_max, r_max / h,
                s.ghost_count(), c.cfg.threads);
  }
  std::vector<cfd::Panel> panels;
  body.panels(0.5 * h, panels);
  const double q = 0.5 * inf.rho * speed * speed;
  out.d_ref = d_ref;
  out.s_ref = 3.14159265358979 * 0.25 * d_ref * d_ref;
  const double x_ref = c.x_ref >= 0.0 ? c.x_ref : x0 + (0.4 * (x1 - x0));
  out.x_ref = x_ref;
  const double t0 = now_s();
  std::vector<std::array<double, 3>> hist;
  int it = 0;
  while (it < c.iters) {
    const int chunk = std::min(c.report_every, c.iters - it);
    for (int k = 0; k < chunk; ++k) {
      s.step();
    }
    it += chunk;
    if (!s.healthy()) {
      std::printf("  the solution went bad (not a number) at iteration %d\n", it);
      return out;
    }
    cfd::Forces f = s.integrate(panels, V3{x_ref, 0, 0}, true, -2);
    for (std::size_t l = 0; l < c.lattices.size(); ++l) {
      const cfd::Forces fl = s.porous_force(l, V3{x_ref, 0, 0}, true);
      f.force = f.force + fl.force;
      f.moment = f.moment + fl.moment;
    }
    const double ca = -f.force.x / (q * out.s_ref);
    const double cn = -f.force.y / (q * out.s_ref);
    const double cm = f.moment.z / (q * out.s_ref * d_ref);
    hist.push_back({ca, cn, cm});
    if (c.verbose) {
      std::printf("  it %5d  residual down %.2f orders   CA %.5f  CN %.5f  Cm %.5f   %.0f s\n", it, s.residual_drop(), ca, cn, cm, now_s() - t0);
    }
    // converged: the residual has fallen as far as asked, or the forces have stopped moving
    if (s.residual_drop() >= c.orders) {
      break;
    }
    if (hist.size() >= 4U && it >= 1000) {
      bool steady = true;
      for (std::size_t j = 0; j < 3U; ++j) {
        const double a = hist[hist.size() - 1U][j];
        const double b = hist[hist.size() - 3U][j];
        if (std::fabs(a - b) > 0.002 * std::max(std::fabs(a), 0.02)) {
          steady = false;
        }
      }
      if (steady) {
        break;
      }
    }
  }
  cfd::Forces fs = s.integrate(panels, V3{x_ref, 0, 0}, true, -2);
  const cfd::Forces fb1 = s.integrate(panels, V3{x_ref, 0, 0}, true, 1);
  const cfd::Forces fb2 = s.integrate(panels, V3{x_ref, 0, 0}, true, 2);
  const cfd::Forces fb{fb1.force + fb2.force, fb1.moment + fb2.moment, fb1.area + fb2.area};
  for (std::size_t l = 0; l < c.lattices.size(); ++l) {
    const cfd::Forces fl = s.porous_force(l, V3{x_ref, 0, 0}, true);
    fs.force = fs.force + fl.force;
    fs.moment = fs.moment + fl.moment;
    out.parts.push_back({c.lattice_names[l] + " (lattice)", -fl.force.x / (q * out.s_ref), -fl.force.y / (q * out.s_ref), fl.moment.z / (q * out.s_ref * d_ref)});
  }
  for (std::size_t k = 0; k < c.part_ids.size(); ++k) {
    const cfd::Forces fp = s.integrate(panels, V3{x_ref, 0, 0}, true, c.part_ids[k]);
    if (fp.area <= 0.0) {
      continue;   // a flap on the z < 0 side: its force is counted with its mirror image, the one at z > 0
    }
    out.parts.push_back({c.part_names[k] + " + its mirror", -fp.force.x / (q * out.s_ref), -fp.force.y / (q * out.s_ref), fp.moment.z / (q * out.s_ref * d_ref)});
  }
  out.ca_skin = -fs.force.x / (q * out.s_ref);
  out.ca_base_euler = -fb.force.x / (q * out.s_ref);
  out.ca_fore = -fb2.force.x / (q * out.s_ref);
  out.cm_fore = fb2.moment.z / (q * out.s_ref * d_ref);
  out.ca_base = out.ca_base_euler;
  if (c.base_model) {   // the base drag of the build-up's law, on the area of the aft-facing base, less with the engines on (aero.hpp): an Euler solution has no wake
    double a_base = 0.0;
    for (const cfd::Panel& pn : panels) {
      if (pn.part == 1 && pn.n.x < -0.9) {
        a_base += 2.0 * pn.area;
      }
    }
    const double cdb = c.mach < 1.0 ? 0.12 + (0.13 * c.mach * c.mach) : 0.25 / c.mach;
    out.ca_base = cdb * a_base / out.s_ref * (1.0 - (0.7 * std::min(std::max(c.power, 0.0), 1.0)));
  }
  out.cn = -fs.force.y / (q * out.s_ref);
  out.cm = fs.moment.z / (q * out.s_ref * d_ref);
  out.x_cp = std::fabs(fs.force.y) > 1e-9 * q * out.s_ref ? x_ref + (fs.moment.z / fs.force.y) : 0.0;
  if (c.viscous && profile != nullptr) {
    add_viscous(s, panels, *profile, c, x_ref, d_ref, out);
  }
  out.residual_drop = s.residual_drop();
  out.iterations = s.iteration();
  out.seconds = now_s() - t0;
  out.ok = true;
  if (!c.field_path.empty() && profile != nullptr) {
    write_field(c.field_path, s, body, *profile, c, x0, x1, r_max, d_ref, out, h);
  }
  return out;
}


// ------------------------------------------------------------------------------------------------ the vehicle

bool load(const char* path, sim::VehicleFile& vf) {
  std::vector<std::string> errors;
  if (!sim::load_vehicle_file(path, vf, errors)) {
    for (const std::string& e : errors) {
      std::fprintf(stderr, "%s\n", e.c_str());
    }
    return false;
  }
  return true;
}

struct Body {
  std::vector<std::pair<double, double>> profile;
  double x0 = 0.0;
  double x1 = 0.0;
  double r = 0.0;
  double d_ref = 0.0;
};

bool make_body(const sim::VehicleFile& vf, std::size_t stage_from, std::size_t stage_to, Body& b) {
  b.profile = cfd::body_profile(vf.params.spec, stage_from, 16, stage_to);
  if (b.profile.size() < 2U) {
    std::fprintf(stderr, "the vehicle has no outer body (sections) from stage %zu on\n", stage_from);
    return false;
  }
  b.x0 = b.profile.front().first;
  b.x1 = b.profile.back().first;
  for (const auto& p : b.profile) {
    b.r = std::max(b.r, p.second);
  }
  b.d_ref = vf.params.spec.aero.reference_diameter_m > 0.0 ? vf.params.spec.aero.reference_diameter_m : 2.0 * b.r;
  return true;
}

// ------------------------------------------------------------------------------------------------ the validation

struct Check {
  const char* name;
  double value;
  double expect;
  double tol;   // relative, or absolute when `abs`
  bool abs;
  bool pass;
};

std::vector<Check> g_checks;

void verdict(const char* name, double value, double expect, double tol, bool absolute = false) {
  const double err = absolute ? std::fabs(value - expect) : std::fabs(value - expect) / std::fabs(expect);
  const bool pass = err <= tol;
  g_checks.push_back({name, value, expect, tol, absolute, pass});
  std::printf("    %-58s %10.5f   exact %10.5f   error %s%.2f%s   %s\n", name, value, expect, absolute ? "" : "", absolute ? err : 100.0 * err, absolute ? "" : " %", pass ? "ok" : "OUT OF BAND");
}

void validate_sod(int threads) {
  std::printf("\n1. Sod's shock tube (time-accurate, 3D code on a slab), exact solution at t = 0.2\n");
  for (const int n : {100, 200, 400}) {
    cfd::Config cfg;
    cfg.threads = 1;
    (void)threads;
    cfg.local_dt = false;
    cfg.cfl = 0.5;
    cfg.bc = {cfd::Bc::Outflow, cfd::Bc::Outflow, cfd::Bc::Symmetry, cfd::Bc::Symmetry, cfd::Bc::Symmetry, cfd::Bc::Symmetry};
    const std::array<std::vector<double>, 3> nd{cfd::uniform_nodes(0.0, 1.0, n), cfd::uniform_nodes(0.0, 0.02, 2), cfd::uniform_nodes(0.0, 0.02, 2)};
    cfd::Euler3D s(nd, cfd::Prim{1.0, 0, 0, 0, 1.0}, nullptr, cfg);
    for (int k = 0; k < 2; ++k) {
      for (int j = 0; j < 2; ++j) {
        for (int i = 0; i < n; ++i) {
          s.set_prim(i, j, k, s.centre(0, i) < 0.5 ? cfd::Prim{1.0, 0, 0, 0, 1.0} : cfd::Prim{0.125, 0, 0, 0, 0.1});
        }
      }
    }
    while (s.time() < 0.2 - 1e-12) {
      s.step(std::min(s.max_dt(), 0.2 - s.time()));
    }
    const gd::Riemann ex({1.0, 0.0, 1.0}, {0.125, 0.0, 0.1}, kGamma);
    double l1 = 0.0;
    for (int i = 0; i < n; ++i) {
      l1 += std::fabs(s.prim(i, 0, 0).rho - ex.sample((s.centre(0, i) - 0.5) / 0.2).rho) / n;
    }
    char name[96];
    std::snprintf(name, sizeof(name), "%d cells: L1 error of the density", n);
    verdict(name, l1, 0.0, n == 100 ? 0.012 : (n == 200 ? 0.006 : 0.0035), true);
  }
}

void validate_wedge(int threads, bool quick) {
  std::printf("\n2. A wedge of 10 degrees at Mach 2: the oblique shock (theta-beta-Mach)\n");
  const double th = 10.0 * kDeg;
  const double m = 2.0;
  const double beta = gd::weak_shock_angle(m, th, kGamma);
  const gd::Normal ex = gd::oblique_shock(m, beta, th, kGamma);
  const cfd::HalfSpace top(V3{0, 0, 0}, V3{-std::sin(th), std::cos(th), 0});
  const cfd::HalfSpace aft(V3{0, 0, 0}, V3{-1, 0, 0});
  const cfd::Intersection wedge(top, aft);
  for (const int nx : quick ? std::vector<int>{40, 80} : std::vector<int>{40, 80, 160}) {
    cfd::Config cfg;
    cfg.threads = nx >= 160 ? threads : 1;
    cfg.bc = {cfd::Bc::Farfield, cfd::Bc::Outflow, cfd::Bc::Symmetry, cfd::Bc::Farfield, cfd::Bc::Symmetry, cfd::Bc::Symmetry};
    const cfd::Prim inf{1.0, m * std::sqrt(kGamma), 0.0, 0.0, 1.0};
    const std::array<std::vector<double>, 3> nd{cfd::uniform_nodes(-1.0, 3.0, nx), cfd::uniform_nodes(0.0, 1.7, nx * 17 / 40), cfd::uniform_nodes(0.0, 0.1, 2)};
    cfd::Euler3D s(nd, inf, &wedge, cfg);
    s.run_to_steady(nx * 20, 9.0);
    cfd::Prim p;
    s.sample(V3{2.5, (2.5 * std::tan(th)) + 0.2, 0.05}, p);
    // the angle of the shock from where the pressure is half way up on two vertical lines
    auto y_half = [&](double x) {
      cfd::Prim q;
      for (double y = 1.65; y > 0.3; y -= 0.002) {
        if (s.sample(V3{x, y, 0.05}, q) && q.p > 0.5 * (1.0 + ex.p2_p1)) {
          return y;
        }
      }
      return 0.0;
    };
    const double b = std::atan2(y_half(1.5) - y_half(0.9), 0.6);
    char name[96];
    std::snprintf(name, sizeof(name), "%d x %d cells: pressure behind the shock / free stream", nx, nx * 17 / 40);
    verdict(name, p.p / 1.0, ex.p2_p1, nx == 40 ? 0.01 : 0.01);
    std::snprintf(name, sizeof(name), "%d x %d cells: shock angle (deg)", nx, nx * 17 / 40);
    verdict(name, b / kDeg, beta / kDeg, 0.05);
  }
}

// The mean pressure on the side of a cone between two stations, over the free stream's, read at the panels.
double cone_side_pressure(cfd::Euler3D& s, const cfd::Revolution& body, double x_lo, double x_hi, double h) {
  std::vector<cfd::Panel> pan;
  body.panels(0.5 * h, pan);
  double sum = 0.0;
  double area = 0.0;
  for (const cfd::Panel& p : pan) {
    if (p.part == 0 && p.c.x > x_lo && p.c.x < x_hi) {
      sum += s.wall_pressure(p) * p.area;
      area += p.area;
    }
  }
  return sum / area;
}

void validate_cone(int threads, bool quick, double mach, double half_angle_deg, const std::vector<double>& hs, const std::vector<double>& tols) {
  const double th = half_angle_deg * kDeg;
  const gd::Cone ex = gd::cone_flow(mach, th, kGamma);
  std::printf("\n3. A cone of %.0f degrees at Mach %.1f (Taylor-Maccoll: shock %.2f deg, surface pressure %.4f x free stream, surface Mach %.3f)\n", half_angle_deg, mach, ex.shock_angle / kDeg, ex.p_surface_p1,
              ex.mach_surface);
  const double len = 4.0 * (half_angle_deg > 12.0 ? 1.0 : 1.4);
  const double rb = len * std::tan(th);
  const cfd::Revolution body({{0.0, 0.0}, {len, rb}, {len, 0.0}});
  std::size_t k = 0;
  for (const double h : hs) {
    if (quick && k >= 1U) {
      break;
    }
    cfd::Config cfg;
    cfg.threads = threads;
    cfg.bc = {cfd::Bc::Farfield, cfd::Bc::Outflow, cfd::Bc::Farfield, cfd::Bc::Farfield, cfd::Bc::Symmetry, cfd::Bc::Farfield};
    const double speed = mach * std::sqrt(kGamma);
    const cfd::Prim inf{1.0, speed, 0.0, 0.0, 1.0};
    const std::array<std::vector<double>, 3> nd{cfd::graded_nodes(-1.5, len + 1.5, -0.2, len + 0.5, h, 1.08, 4 * h), cfd::graded_nodes(-3.0, 3.0, -1.5 * rb, 1.5 * rb, h, 1.08, 4 * h),
                                                cfd::graded_nodes(0.0, 3.0, 0.0, 1.5 * rb, h, 1.08, 4 * h)};
    cfd::Euler3D s(nd, inf, &body, cfg);
    s.run_to_steady(5000, 6.0);
    const double p = cone_side_pressure(s, body, 0.35 * len, 0.75 * len, h);
    // the shock radius at x = 0.9 len: where the pressure is half way between the free stream's and the surface's
    double r_shock = 0.0;
    cfd::Prim q;
    for (double y = 2.5; y > 0.0; y -= 0.005) {
      if (s.sample(V3{0.9 * len, y, 0.0}, q) && q.p > 1.0 + (0.5 * (ex.p_surface_p1 - 1.0))) {
        r_shock = y;
        break;
      }
    }
    char name[96];
    std::snprintf(name, sizeof(name), "h = %.3g (%.1f cells across the base radius): surface pressure", h, rb / h);
    verdict(name, p, ex.p_surface_p1, tols[k]);
    std::snprintf(name, sizeof(name), "h = %.3g: shock angle at 0.9 of the length (deg)", h);
    verdict(name, std::atan(r_shock / (0.9 * len)) / kDeg, ex.shock_angle / kDeg, 0.05);
    ++k;
  }
}

void validate_sphere(int threads, bool quick) {
  if (quick) {
    return;
  }
  const double m = 3.0;
  std::printf("\n4. A sphere at Mach %.0f: the stagnation pressure (Rayleigh pitot) and the shock stand-off (Billig's correlation)\n", m);
  const cfd::Sphere ball(V3{0, 0, 0}, 1.0);
  const double speed = m * std::sqrt(kGamma);
  for (const double h : {0.1, 0.07}) {
    cfd::Config cfg;
    cfg.threads = threads;
    cfg.bc = {cfd::Bc::Farfield, cfd::Bc::Outflow, cfd::Bc::Farfield, cfd::Bc::Farfield, cfd::Bc::Symmetry, cfd::Bc::Farfield};
    const cfd::Prim inf{1.0, -speed, 0.0, 0.0, 1.0};   // the flow comes from +x
    const std::array<std::vector<double>, 3> nd{cfd::graded_nodes(-4.0, 3.0, -1.6, 1.6, h, 1.08, 4 * h), cfd::graded_nodes(-3.5, 3.5, -1.6, 1.6, h, 1.08, 4 * h), cfd::graded_nodes(0.0, 3.5, 0.0, 1.6, h, 1.08, 4 * h)};
    cfd::Euler3D s(nd, inf, &ball, cfg);
    s.run_to_steady(6000, 6.0);
    std::vector<cfd::Panel> pan;
    ball.panels(0.5 * h, pan);
    double p_stag = 0.0;
    double w = 0.0;
    for (const cfd::Panel& p : pan) {
      if (p.n.x > 0.995) {   // within 6 degrees of the nose of the sphere: the pressure there is flat to 1 %
        p_stag += s.wall_pressure(p) * p.area;
        w += p.area;
      }
    }
    // the shock: the first point along the axis from upstream where the pressure has risen half way to the stagnation value
    double x_shock = 0.0;
    cfd::Prim q;
    for (double x = 3.0; x > 1.0; x -= 0.002) {
      if (s.sample(V3{x, 0.0, 0.0}, q) && q.p > 0.5 * (1.0 + gd::pitot_ratio(m, kGamma))) {
        x_shock = x;
        break;
      }
    }
    char name[96];
    std::snprintf(name, sizeof(name), "h = %.3g (%.0f cells across the radius): stagnation pressure", h, 1.0 / h);
    verdict(name, p_stag / w, gd::pitot_ratio(m, kGamma), h > 0.08 ? 0.09 : 0.06);   // a blunt body is the weak case of an immersed boundary: 6 % low at 10 cells, 3 % at 14
    std::snprintf(name, sizeof(name), "h = %.3g: shock stand-off over the radius", h);
    verdict(name, x_shock - 1.0, 0.143 * std::exp(3.24 / (m * m)), 0.25);   // one cell is 0.1
  }
}

// The real gas against its own exact solution: a gas rushing at a wall at Mach 10 throws off a shock and comes to rest; behind it the density, pressure and temperature are those of the steady normal shock
// of the equation of state (solved from the conservation laws in gasdyn.hpp), and the solver, a 1D slab of the 3D code, has to find them. Then a sphere at Mach 20, perfect and real.
void validate_gas(int threads, bool quick) {
  std::printf("\n5. Real gas: air that vibrates and dissociates (CFD.md section 11), at 40 km\n");
  const sim::Air at = sim::air_at(40000.0);
  const cfd::RealAir air(3000.0 / at.temperature, 59500.0 / at.temperature, 150000.0 / at.density);
  for (const double m : {5.0, 10.0}) {
    const double u0 = m * std::sqrt(kGamma);
    cfd::Config cfg;
    cfg.gas = cfd::Gas(kGamma, &air);
    cfg.local_dt = false;
    cfg.cfl = 0.4;
    cfg.bc = {cfd::Bc::Symmetry, cfd::Bc::Outflow, cfd::Bc::Symmetry, cfd::Bc::Symmetry, cfd::Bc::Symmetry, cfd::Bc::Symmetry};
    const int n = 200;
    const std::array<std::vector<double>, 3> nd{cfd::uniform_nodes(0.0, 4.0, n), cfd::uniform_nodes(0.0, 0.08, 2), cfd::uniform_nodes(0.0, 0.08, 2)};
    cfd::Euler3D sl(nd, cfd::Prim{1.0, -u0, 0.0, 0.0, 1.0}, nullptr, cfg);
    double w = 0.0;
    const gd::RealShock ex = gd::real_wall_shock(air, u0, w);
    const double t_end = 2.0 / w;
    while (sl.time() < t_end - 1e-12) {
      sl.step(std::min(sl.max_dt(), t_end - sl.time()));
    }
    const cfd::Prim p = sl.prim(5, 0, 0);
    char name[96];
    std::snprintf(name, sizeof(name), "Mach %.0f gas thrown off a wall: pressure behind the shock", m);
    verdict(name, p.p, ex.p2, 0.02);
    std::snprintf(name, sizeof(name), "Mach %.0f: density behind the shock (%.1f x the free stream's)", m, ex.rho2);
    verdict(name, p.rho, ex.rho2, 0.03);
    std::snprintf(name, sizeof(name), "Mach %.0f: temperature behind the shock (%.0f K)", m, ex.t2 * at.temperature);
    verdict(name, air.t_from_p(p.rho, p.p), ex.t2, 0.03);
  }
  if (quick) {
    return;
  }
  // a sphere at Mach 20, perfect and real: the real gas has the denser, cooler shock layer, so the shock lies closer to the body and the nose's pressure is higher
  const double m = 20.0;
  const cfd::Sphere ball(V3{0, 0, 0}, 1.0);
  const double h = 0.1;
  double standoff[2] = {0.0, 0.0};
  double p_stag[2] = {0.0, 0.0};
  for (int real = 0; real < 2; ++real) {
    cfd::Config cfg;
    cfg.threads = threads;
    cfg.gas = real != 0 ? cfd::Gas(kGamma, &air) : cfd::Gas(kGamma);
    cfg.bc = {cfd::Bc::Farfield, cfd::Bc::Outflow, cfd::Bc::Farfield, cfd::Bc::Farfield, cfd::Bc::Symmetry, cfd::Bc::Farfield};
    const double speed = m * std::sqrt(kGamma);
    const cfd::Prim inf{1.0, -speed, 0.0, 0.0, 1.0};
    const std::array<std::vector<double>, 3> nd{cfd::graded_nodes(-4.0, 3.0, -1.6, 1.6, h, 1.08, 4 * h), cfd::graded_nodes(-3.5, 3.5, -1.6, 1.6, h, 1.08, 4 * h), cfd::graded_nodes(0.0, 3.5, 0.0, 1.6, h, 1.08, 4 * h)};
    cfd::Euler3D s(nd, inf, &ball, cfg);
    s.run_to_steady(5000, 6.0);
    std::vector<cfd::Panel> pan;
    ball.panels(0.5 * h, pan);
    double sum = 0.0;
    double wsum = 0.0;
    for (const cfd::Panel& pn : pan) {
      if (pn.n.x > 0.995) {
        sum += s.wall_pressure(pn) * pn.area;
        wsum += pn.area;
      }
    }
    p_stag[real] = sum / wsum;
    cfd::Prim q;
    for (double x = 3.0; x > 1.0; x -= 0.002) {
      if (s.sample(V3{x, 0.0, 0.0}, q) && q.rho > 0.5 * (1.0 + 6.0)) {   // the shock: the density has risen half way to the perfect gas's six
        standoff[real] = x - 1.0;
        break;
      }
    }
  }
  const gd::RealShock rs = gd::real_normal_shock(air, m * std::sqrt(kGamma));
  const double p0_real = gd::real_stagnation_pressure(air, rs);
  const double p0_perfect = gd::pitot_ratio(m, kGamma);
  char name[112];
  std::snprintf(name, sizeof(name), "sphere Mach 20, perfect gas: stagnation pressure (Rayleigh)");
  verdict(name, p_stag[0], p0_perfect, 0.10);
  std::snprintf(name, sizeof(name), "sphere Mach 20, real gas: stagnation pressure (the isentrope of the equation of state)");
  verdict(name, p_stag[1], p0_real, 0.10);
  std::printf("    (the shock's stand-off, %.2f radii perfect and %.2f real, is under two cells at this grid and is not checked)\n", standoff[0], standoff[1]);
}

// The viscous layer against its exact laminar solution and against Van Driest (no flow grid: the layer is a function of the edge state). The laminar flat plate in the reference-temperature method of viscous.hpp
// against the similarity solution of the compressible boundary layer (Levy-Lees, solved in gasdyn.hpp as a boundary-value problem; at Mach 0 it is Blasius's 0.664); the turbulent average friction of a long
// plate against Van Driest's second method; a cone against the plate by Mangler's length.
void validate_viscous() {
  std::printf("\n6. The viscous layer (CFD.md section 13): friction of a flat plate and a cone, laminar and turbulent\n");
  const double te = 220.0;
  auto edge = [&](double m) {
    cfd::visc::Edge e;
    e.t = te;
    e.p = 1000.0;
    e.rho = e.p / (cfd::visc::kGasConstant * te);
    e.u = m * std::sqrt(1.4 * cfd::visc::kGasConstant * te);
    return e;
  };
  const char* lam_names[] = {"Mach 0.01 laminar plate: Cf sqrt(Re_x), Blasius 0.664", "Mach 2 insulated laminar plate: Cf sqrt(Re_x)", "Mach 5 insulated laminar plate: Cf sqrt(Re_x)"};
  int k = 0;
  for (const double m : {0.01, 2.0, 5.0}) {
    const gd::Laminar ex = gd::laminar_flat_plate(m, -1.0, te);
    const cfd::visc::Layer ly = cfd::visc::local_layer(edge(m), 1.0, te * ex.t_wall_ratio, 1e30);
    verdict(lam_names[k++], ly.cf * std::sqrt(ly.re_x), ex.cf_sqrt_re, 0.04);
  }
  const char* turb_names[] = {"Mach 2 turbulent plate (100 m): average Cf, Van Driest II", "Mach 5 turbulent plate (100 m): average Cf, Van Driest II", "Mach 8 turbulent plate (100 m): average Cf, Van Driest II"};
  k = 0;
  for (const double m : {2.0, 5.0, 8.0}) {
    const cfd::visc::Edge e = edge(m);
    const double tw = te * (1.0 + (0.5 * 0.4 * m * m * std::cbrt(0.71)));
    const double len = 100.0;
    double sum = 0.0;
    for (int i = 0; i < 4000; ++i) {
      sum += cfd::visc::local_layer(e, len * (i + 0.5) / 4000.0, tw, 1e-9).cf * len / 4000.0;
    }
    const double re_l = e.rho * e.u * len / cfd::visc::sutherland(te);
    verdict(turb_names[k++], sum / len, gd::van_driest_ii(m, -1.0, te, re_l), 0.20);   // the band is the method's: the layer is a reference-temperature one, Van Driest's is a theory
  }
  std::vector<double> s(301);
  std::vector<double> rc(301);
  for (std::size_t i = 0; i < s.size(); ++i) {
    s[i] = 0.01 * static_cast<double>(i);
    rc[i] = 0.1 * s[i];
  }
  const std::vector<double> ml = cfd::visc::mangler_length(s, rc);
  verdict("cone: Mangler's length at two thirds of the way, 2/3", ml[200], 2.0 / 3.0, 0.01);
}

int validate(bool quick, int threads, const std::string& only) {
  std::printf("tfc_cfd --validate%s%s: the Euler solver against the exact solutions (gamma 1.4)\n", quick ? " --quick" : "", only.empty() ? "" : (" --only " + only).c_str());
  const double t0 = now_s();
  auto want = [&](const char* name) { return only.empty() || only == name; };
  if (want("sod")) {
    validate_sod(threads);
  }
  if (want("wedge")) {
    validate_wedge(threads, quick);
  }
  if (want("cone")) {
    validate_cone(threads, quick, 2.5, 15.0, {0.2, 0.14, 0.1}, {0.03, 0.02, 0.02});
    validate_cone(threads, quick, 5.0, 10.0, {0.1, 0.07}, {0.03, 0.02});
  }
  if (want("sphere")) {
    validate_sphere(threads, quick);
  }
  if (want("gas")) {
    validate_gas(threads, quick);
  }
  if (want("viscous")) {
    validate_viscous();
  }
  int bad = 0;
  for (const Check& c : g_checks) {
    bad += c.pass ? 0 : 1;
  }
  std::printf("\n%zu checks, %d out of band, %.0f s\n", g_checks.size(), bad, now_s() - t0);
  return bad == 0 ? 0 : 1;
}

// ------------------------------------------------------------------------------------------------ a flow on a vehicle

std::vector<double> parse_list(const char* s) {
  std::vector<double> v;
  const char* p = s;
  while (*p != '\0') {
    char* end = nullptr;
    v.push_back(std::strtod(p, &end));
    if (end == p) {
      break;
    }
    p = (*end == ',') ? end + 1 : end;
  }
  return v;
}

void print_result(const FlowResult& r, const FlowCase& c, const Body& b) {
  std::printf("\nMach %.2f, angle of attack %.1f deg, %zu cells, %d iterations (residual down %.2f orders), %.0f s\n", c.mach, c.alpha_deg, r.cells, r.iterations, r.residual_drop, r.seconds);
  std::printf("  reference diameter %.2f m, area %.2f m^2, body from %.1f to %.1f m (%.1f m)\n", r.d_ref, r.s_ref, b.x0, b.x1, b.x1 - b.x0);
  std::printf("  axial force coefficient CA  %.4f (pressure on the skin)  %+.4f (base: %s; the Euler solution's own is %+.4f)\n", r.ca_skin, r.ca_base, c.base_model ? "the base-drag law" : "the Euler solution's", r.ca_base_euler);
  if (std::fabs(r.ca_fore) > 1e-9 || std::fabs(r.cm_fore) > 1e-9) {
    std::printf("    of the Euler solution's own, the flat forward end: CA %+.4f, Cm %+.4f (a real face when the body flies nose first; the table keeps it apart)\n", r.ca_fore, r.cm_fore);
  }
  if (c.viscous) {
    std::printf("  skin friction: CA %+.4f, CN %+.4f (total CA %.4f); the layer is %.0f %% laminar\n", r.ca_friction, r.cn_friction, r.ca_skin + r.ca_friction + r.ca_base, 100.0 * r.laminar_fraction);
    std::printf("  heating: the highest flux off the nose %.0f kW/m^2, %.1f MW into the whole skin, the wall at %.0f K at most and %.0f K on average (%s)\n", r.heat_peak_kw, r.heat_total_mw, r.t_wall_max_k, r.t_wall_mean_k,
                c.wall_k > 0.0 ? "fixed" : "radiative equilibrium");
  }
  std::printf("  normal force coefficient CN %.4f%s\n", r.cn, c.alpha_deg > 0.0 ? "" : " (zero at no angle of attack)");
  for (const FlowResult::Part& p : r.parts) {
    std::printf("    of which %-28s CA %+.4f  CN %+.4f  Cm %+.4f\n", p.name.c_str(), p.ca, p.cn, p.cm);
  }
  if (c.alpha_deg > 0.0) {
    std::printf("  CN-alpha %.3f per radian, centre of pressure %.2f m from the aft end (%.2f calibres from the nose)\n", r.cn / (c.alpha_deg * kDeg), r.x_cp, (b.x1 - r.x_cp) / r.d_ref);
  }
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) {
    std::fprintf(stderr, "usage: tfc_cfd --validate [--quick] [--only sod|wedge|cone|sphere|gas|viscous] [--threads N]\n       tfc_cfd VEHICLE.json --mach M [--alpha DEG] [--h M] [--iters N] [--orders R] [--threads N] [--xref M] [--stage-from K] [--probe extrap|CELLS] [--json FILE] [--field FILE] [--gas real] [--altitude KM] [--surfaces stowed|deployed] [--deflect NAME=DEG,...]\n"
                         "       tfc_cfd VEHICLE.json --machs 2,3,5 --alpha 4 [options] --table OUT.json\n");
    return 2;
  }
  FlowCase c;
  std::string file;
  std::string json_out;
  std::string table_out;
  std::vector<double> machs;
  std::size_t stage_from = 0U;
  std::size_t stage_to = 1000U;
  bool do_validate = false;
  bool real_gas = false;
  std::string surfaces_mode = "none";   // none | stowed | deployed: the flaps and the grid fins of the file on the body
  std::string deflect_text;              // NAME=DEG,NAME=DEG: a surface whose name contains NAME is put at DEG
  cfd::SurfaceOptions sopt;
  bool quick = false;
  std::string only;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    auto next = [&]() -> const char* {
      if (i + 1 >= argc) {
        std::fprintf(stderr, "%s needs a value\n", a.c_str());
        std::exit(2);
      }
      return argv[++i];
    };
    if (a == "--validate") {
      do_validate = true;
    } else if (a == "--quick") {
      quick = true;
    } else if (a == "--only") {
      only = next();
    } else if (a == "--mach") {
      c.mach = std::atof(next());
    } else if (a == "--machs") {
      machs = parse_list(next());
    } else if (a == "--alpha") {
      c.alpha_deg = std::atof(next());
    } else if (a == "--h") {
      c.h = std::atof(next());
    } else if (a == "--iters") {
      c.iters = std::atoi(next());
    } else if (a == "--orders") {
      c.orders = std::atof(next());
    } else if (a == "--threads") {
      c.threads = std::atoi(next());
    } else if (a == "--xref") {
      c.x_ref = std::atof(next());
    } else if (a == "--surfaces") {
      surfaces_mode = next();
    } else if (a == "--deflect") {
      deflect_text = next();
    } else if (a == "--flap-thickness") {
      sopt.flap_thickness_m = std::atof(next());
    } else if (a == "--gas") {
      real_gas = std::strcmp(next(), "real") == 0;
    } else if (a == "--altitude") {
      c.fs_km = std::atof(next());
    } else if (a == "--viscous") {
      c.viscous = true;
    } else if (a == "--wall-temp") {
      c.wall_k = std::atof(next());
    } else if (a == "--emissivity") {
      c.emissivity = std::atof(next());
    } else if (a == "--transition") {
      c.re_transition = std::atof(next());
    } else if (a == "--base") {
      c.base_model = std::strcmp(next(), "euler") != 0;
    } else if (a == "--power") {
      c.power = std::atof(next());
    } else if (a == "--stage-to") {
      stage_to = static_cast<std::size_t>(std::atoi(next()));
    } else if (a == "--stage-from") {
      stage_from = static_cast<std::size_t>(std::atoi(next()));
    } else if (a == "--probe") {
      const char* v = next();
      c.cfg.probe_extrapolate = std::strcmp(v, "extrap") == 0;
      if (!c.cfg.probe_extrapolate) {
        c.cfg.probe_cells = std::atof(v);
      }
    } else if (a == "--json") {
      json_out = next();
    } else if (a == "--field") {
      c.field_path = next();
    } else if (a == "--table") {
      table_out = next();
    } else if (a.rfind("--", 0) == 0) {
      std::fprintf(stderr, "unknown option %s\n", a.c_str());
      return 2;
    } else {
      file = a;
    }
  }
  c.real = real_gas;
  const int threads = c.threads > 0 ? c.threads : static_cast<int>(std::max(1U, std::thread::hardware_concurrency()));
  if (do_validate) {
    return validate(quick, threads, only);
  }
  if (file.empty()) {
    std::fprintf(stderr, "no vehicle file\n");
    return 2;
  }
  sim::VehicleFile vf;
  if (!load(file.c_str(), vf)) {
    return 1;
  }
  Body b;
  if (!make_body(vf, stage_from, stage_to, b)) {
    return 1;
  }
  const cfd::Revolution body(b.profile);
  // the flaps and the grid fins: stowed (a flap at its stowed angle, a fin folded away: absent) or deployed (flaps at 60 degrees, fins at 0), then --deflect puts the named ones where it says
  cfd::PlacedSurfaces placed;
  cfd::Assembly assembly;
  if (surfaces_mode != "none") {
    const sim::VehicleSpec& g = vf.params.spec;
    std::vector<double> defl(g.surfaces.size(), 0.0);
    std::vector<bool> stowed(g.surfaces.size(), false);
    for (std::size_t i = 0; i < g.surfaces.size(); ++i) {
      const bool flap = g.surfaces[i].kind == sim::SurfaceKind::Flap;
      defl[i] = surfaces_mode == "stowed" ? g.surfaces[i].stow_deg : (flap ? 60.0 : 0.0);
      stowed[i] = surfaces_mode == "stowed" && !flap;
    }
    for (std::size_t pos = 0; pos < deflect_text.size();) {
      const std::size_t comma = std::min(deflect_text.find(',', pos), deflect_text.size());
      const std::string item = deflect_text.substr(pos, comma - pos);
      const std::size_t eq = item.find('=');
      if (eq != std::string::npos) {
        const std::string key = item.substr(0, eq);
        const double deg = std::atof(item.c_str() + eq + 1U);
        for (std::size_t i = 0; i < g.surfaces.size(); ++i) {
          if (g.surfaces[i].name.find(key) != std::string::npos) {
            defl[i] = deg;
            stowed[i] = false;
          }
        }
      }
      pos = comma + 1U;
    }
    cfd::place_surfaces(g, stage_from, defl, stowed, sopt, placed);
    assembly.add(&body);
    for (const auto& sh : placed.shapes) {
      assembly.add(sh.get());
    }
    c.lattices = placed.lattices;
    c.lattice_names = placed.lattice_names;
    c.part_names = placed.shape_names;
    c.part_ids = placed.shape_parts;
    c.r_extent = b.r;
    for (std::size_t i = 0; i < g.surfaces.size(); ++i) {
      const sim::SurfaceSpec& sp = g.surfaces[i];
      if (static_cast<std::size_t>(sp.stage) >= stage_from) {
        c.r_extent = std::max(c.r_extent, sp.radius + std::max(sp.chord, sp.span) + 0.5);
      }
    }
    std::printf("  surfaces %s: %zu flaps (%.1f m thick), %zu grid-fin lattices\n", surfaces_mode.c_str(), placed.shapes.size(), sopt.flap_thickness_m, placed.lattices.size());
  }
  const cfd::Shape& shape = surfaces_mode != "none" ? static_cast<const cfd::Shape&>(assembly) : static_cast<const cfd::Shape&>(body);
  c.vehicle = vf.name;
  c.stage_from = stage_from;
  std::printf("%s: body from x = %.1f to %.1f m, radius up to %.2f m\n", vf.name.c_str(), b.x0, b.x1, b.r);
  if (machs.empty()) {
    const FlowResult r = run_body(shape, b.x0, b.x1, b.r, b.d_ref, c, &b.profile);
    if (!r.ok) {
      return 1;
    }
    print_result(r, c, b);
    if (!json_out.empty()) {
      std::FILE* f = std::fopen(json_out.c_str(), "w");
      if (f == nullptr) {
        return 1;
      }
      std::fprintf(f,
                   "{\"mach\": %g, \"alpha_deg\": %g, \"ca_skin\": %.6f, \"ca_base\": %.6f, \"ca_base_euler\": %.6f, \"ca_fore\": %.6f, \"cm_fore\": %.6f, \"ca_friction\": %.6f, \"cn\": %.6f, \"cm\": %.6f, \"x_cp_m\": %.4f, \"x_ref_m\": %.4f, "
                   "\"s_ref_m2\": %.6f, \"d_ref_m\": %.4f, \"cells\": %zu, \"iterations\": %d, \"residual_drop\": %.2f, \"heat_peak_kw_m2\": %.2f, \"t_wall_max_k\": %.1f}\n",
                   c.mach, c.alpha_deg, r.ca_skin, r.ca_base, r.ca_base_euler, r.ca_fore, r.cm_fore, r.ca_friction, r.cn, r.cm, r.x_cp, r.x_ref, r.s_ref, r.d_ref, r.cells, r.iterations, r.residual_drop, r.heat_peak_kw, r.t_wall_max_k);
      std::fclose(f);
    }
    return 0;
  }
  // a table: CA from the flow at no angle of attack, the slope of CN and the centre of pressure from the flow at `alpha`
  const double alpha = c.alpha_deg > 0.0 ? c.alpha_deg : 4.0;
  std::string rows;
  for (const double m : machs) {
    FlowCase c0 = c;
    c0.mach = m;
    c0.alpha_deg = 0.0;
    c0.verbose = false;
    FlowCase c1 = c0;
    c1.alpha_deg = alpha;
    const FlowResult r0 = run_body(shape, b.x0, b.x1, b.r, b.d_ref, c0);
    const FlowResult r1 = run_body(shape, b.x0, b.x1, b.r, b.d_ref, c1);
    if (!r0.ok || !r1.ok) {
      return 1;
    }
    std::printf("Mach %5.2f: CA %.4f (skin) %+.4f (base)   CN-alpha %.3f /rad   x_cp %.2f m   (%.0f + %.0f s)\n", m, r0.ca_skin, r0.ca_base, r1.cn / (alpha * kDeg), r1.x_cp, r0.seconds, r1.seconds);
    char row[256];
    std::snprintf(row, sizeof(row), "%s{\"mach\": %g, \"ca\": %.4f, \"cn_alpha\": %.4f, \"x_cp\": %.3f}", rows.empty() ? "" : ",\n    ", m, r0.ca_skin, r1.cn / (alpha * kDeg), r1.x_cp);
    rows += row;
  }
  if (!table_out.empty()) {
    std::FILE* f = std::fopen(table_out.c_str(), "w");
    if (f == nullptr) {
      return 1;
    }
    std::fprintf(f, "{\"note\": \"from tfc_cfd on %s: Euler, calorically perfect gas, skin only (no base), alpha %g deg\",\n \"table\": [\n    %s\n ]}\n", vf.name.c_str(), alpha, rows.c_str());
    std::fclose(f);
  }
  return 0;
}
