// SPDX-License-Identifier: MIT
// The vehicle's flaps and grid fins as geometry for the flow solver (docs/design/CFD.md section 12), from the surfaces of its file and the deflection each is at.
//
//  * A flap hinged along the axis (Starship's) is a plate with pointed ends: the length `span` along the axis from its hinge, the width `chord` out from it in the direction the plant's own frame gives for the
//    deflection (0: lying on the skin, 90: standing straight out), of a thickness that the grid must resolve (about three cells). It is cut out of the grid like the body is, and the force on it is read at
//    its surface elements, which carry the number 10 + its index in the file.
//  * A grid fin is a lattice, too fine for any grid: a porous box (`Porous` in solver.hpp) with the fin's frame (`chord` across, `span` along the radius) and its depth along the cells, whose cells' axis is the
//    plant's own for the deflection. A fin that is stowed (folded) is not there.
// The sizes the file does not give (a flap's thickness, a lattice's losses) are the numbers of `SurfaceOptions`, each stated where it is used: they are estimates and the document says so.
#pragma once
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "shape.hpp"
#include "solver.hpp"
#include "spec.hpp"
#include "surfaces.hpp"

namespace sim::cfd {

struct SurfaceOptions {
  double flap_thickness_m = 1.2;      // the thickness of a flap (the viewer's models are about 1.4 m); it must be at least about three cells
  double flap_bevel_ratio = 3.0;      // the pointed ends taper over this many thicknesses
  double grid_depth_m = 0.7;          // the lattice's depth along its cells
  double grid_k_axial = 0.3;          // the loss of the air that goes along the cells, in dynamic pressures (the webs' blockage and friction)
  double grid_k_cross = 20.0;         // the loss of the air that goes across them (the webs stop it): large, so the lattice turns the stream toward its axis
};

struct PlacedSurfaces {
  std::vector<std::unique_ptr<Shape>> shapes;     // the flaps
  std::vector<std::string> shape_names;
  std::vector<int> shape_parts;
  std::vector<std::unique_ptr<Shape>> regions;    // the lattices' regions
  std::vector<Porous> lattices;
  std::vector<std::string> lattice_names;
  std::vector<double> lattice_frame_area;         // the frame area of each, m^2
};

// `deflections[i]` is the deflection of surface i in degrees (the stowed angle for a flap that is stowed; a fin that is stowed is absent, whatever angle it is given).
inline void place_surfaces(const VehicleSpec& g, std::size_t first_stage, const std::vector<double>& deflections, const std::vector<bool>& stowed, const SurfaceOptions& o, PlacedSurfaces& out) {
  for (std::size_t i = 0; i < g.surfaces.size(); ++i) {
    const SurfaceSpec& s = g.surfaces[i];
    if (static_cast<std::size_t>(s.stage) < first_stage) {
      continue;
    }
    const double d = i < deflections.size() ? deflections[i] : s.stow_deg;
    const SurfaceFrame f = surface_frame(s, d);
    if (s.kind == SurfaceKind::Flap && s.axial_hinge) {
      const V3 hinge = surface_point(s, d) - (f.chord * (0.5 * s.chord)) - (f.span * (0.5 * s.span));
      const V3 centre = hinge + (f.chord * (0.5 * s.chord)) + (f.span * (0.5 * s.span)) + (f.normal * (0.5 * o.flap_thickness_m));
      out.shapes.push_back(std::make_unique<BeveledPlate>(centre, f.span, f.normal, f.chord, s.span, o.flap_thickness_m, o.flap_bevel_ratio * o.flap_thickness_m, 0.5 * s.chord, 10 + static_cast<int>(i)));
      out.shape_names.push_back(s.name);
      out.shape_parts.push_back(10 + static_cast<int>(i));
    } else if (s.kind == SurfaceKind::GridFin && !(i < stowed.size() && stowed[i])) {
      const V3 centre = surface_point(s, d);
      out.regions.push_back(std::make_unique<OrientedBox>(centre, f.chord, f.normal, f.span, V3{0.5 * o.grid_depth_m, 0.5 * s.chord, 0.5 * s.span}, 0));
      Porous p;
      p.region = out.regions.back().get();
      p.axis = normalized(f.chord);
      p.k_axial = o.grid_k_axial;
      p.k_cross = o.grid_k_cross;
      p.depth = o.grid_depth_m;
      out.lattices.push_back(p);
      out.lattice_names.push_back(s.name);
      out.lattice_frame_area.push_back(s.chord * s.span);
    }
  }
}

}  // namespace sim::cfd
