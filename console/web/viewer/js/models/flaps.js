// The articulated surfaces of a vehicle (a Starship-class ship's four flaps, the booster's grid fins), drawn from the vehicle file's `surfaces` table and moved by what the simulator reports in every pose
// (`pose.surf`, the deflection of each in degrees, and `pose.sdep`, a bit mask of those deployed): the same numbers the aerodynamics use (sim/vehicle/surfaces.hpp, docs/design/AERODYNAMICS.md section 9),
// so a surface that stands out in the picture stands out in the air.
//
// A flap with `axial_hinge` is hinged along the body's axis, on the skin, from `x_hinge` forward by `span_m`; its plate is `chord_m` wide. At 0 it lies on the skin with its free edge round the body toward
// the side `chord_dir` names; at 90 it stands straight out: edge-on to a stream along the axis (the climb: where the real ship holds them) and broadside to one across the vehicle (the belly flop).
// A grid fin hangs on a shaft along the radius: folded flat against the skin until it is deployed, then standing out, turned about its shaft by the deflection. A fin eases between the two states over about
// a second and a half, as an actuator does.
// Frames: the stage's group, model axes (Y along the body, Z toward the body's +Y, X toward the body's +Z), like everything in models/. Inside a holder turned to the azimuth: Z is the radius, X the tangent
// (toward increasing azimuth), Y the axis.
import * as THREE from '../../vendor/three.module.js';

const DEG = Math.PI / 180;
const FOLD_S = 1.5;                         // seconds a grid fin takes to fold or deploy

let _steel = null, _dark = null, _tile = null, _web = null;
const steel = () => _steel || (_steel = new THREE.MeshStandardMaterial({ color: 0xd2d7dd, metalness: 0.55, roughness: 0.38 }));
const dark = () => _dark || (_dark = new THREE.MeshStandardMaterial({ color: 0x15171a, metalness: 0.4, roughness: 0.6, side: THREE.DoubleSide }));
const tile = () => _tile || (_tile = new THREE.MeshStandardMaterial({ color: 0x0d0e10, metalness: 0.15, roughness: 0.5, side: THREE.DoubleSide }));
const web = () => _web || (_web = new THREE.MeshStandardMaterial({ color: 0x2a2d31, metalness: 0.85, roughness: 0.42 }));

/** The panel of an axial-hinge flap, flat on the skin (the pivot turns it out): `len` along the axis from y = 0, `wid` round the body from the hinge toward the side `s` (+1 or -1), on a hull of radius R.
 *  Faces: the outer one (a steel frame round a black tile face), the inner one (dark), and the edge all round. The plan is a trapezoid: the free edge is shorter than the hinge edge, its corners rounded. */
export function axialFlapGeometry(R, len, wid, s, thick = 0.6, gap = 0.22) {
  const NU = 14, NV = 12, pos = [], idx = [], groups = [];
  const half = (v) => { const t = Math.max(0, (v - 0.8) / 0.2); return 0.5 * len * (1 - 0.2 * v) * (1 - 0.35 * t * t); };       // half the length at v (0 at the hinge, 1 at the free edge)
  const at = (u, v, rad) => {
    const th = (s * v * wid) / R;
    return [rad * Math.sin(th), len / 2 + u * half(v), rad * Math.cos(th) - R];                                                // wrapped round the hull, the hinge at (0, ., 0)
  };
  const layer = (rad) => { const base = pos.length / 3; for (let j = 0; j <= NV; j++) for (let i = 0; i <= NU; i++) pos.push(...at((2 * i) / NU - 1, j / NV, rad)); return base; };
  const face = (base, flip, pick) => {
    const bins = [[], [], []];
    for (let j = 0; j < NV; j++) for (let i = 0; i < NU; i++) {
      const a = base + j * (NU + 1) + i, b = a + 1, c = a + NU + 1, d = c + 1;
      bins[pick(i, j)].push(...(flip !== (s < 0) ? [a, c, b, b, c, d] : [a, b, c, b, d, c]));
    }
    bins.forEach((tri, m) => { if (tri.length) { groups.push([idx.length, tri.length, m]); idx.push(...tri); } });
  };
  const outer = layer(R + gap + thick), inner = layer(R + gap);
  face(outer, false, (i, j) => (i < 2 || i >= NU - 2 || j >= NV - 2 || j < 1 ? 0 : 1));
  face(inner, true, () => 2);
  const start = idx.length, ring = [];
  for (let i = 0; i <= NU; i++) ring.push(i);
  for (let j = 1; j <= NV; j++) ring.push(j * (NU + 1) + NU);
  for (let i = NU - 1; i >= 0; i--) ring.push(NV * (NU + 1) + i);
  for (let j = NV - 1; j >= 1; j--) ring.push(j * (NU + 1));
  for (let k = 0; k < ring.length; k++) { const a = ring[k], b = ring[(k + 1) % ring.length]; idx.push(outer + a, inner + a, outer + b, outer + b, inner + a, inner + b); }
  groups.push([start, idx.length - start, 2]);
  const geo = new THREE.BufferGeometry();
  geo.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3));
  geo.setIndex(idx);
  for (const [st, n, m] of groups) geo.addGroup(st, n, m);
  geo.computeVertexNormals();
  return geo;
}

/** A grid fin standing out along +Z from its shaft at the origin: a frame `depth` deep along Y (the cells' axis) and `span` long along Z, a lattice of thin webs, `thick` across X. */
export function gridFinMesh(depth, span, thick = 0.5) {
  const g = new THREE.Group();
  const bar = (sx, sy, sz, y, z, m) => { const b = new THREE.Mesh(new THREE.BoxGeometry(sx, sy, sz), m); b.position.set(0, y, z); g.add(b); };
  const t = Math.max(0.08, thick * 0.12), nY = 7, nZ = 4;
  bar(thick, depth, t * 1.8, 0, 0, steel());                                                         // the root bar, on the shaft
  bar(thick, depth, t * 1.8, 0, span, steel());                                                      // the tip bar
  bar(thick, t * 1.8, span, depth / 2, span / 2, steel()); bar(thick, t * 1.8, span, -depth / 2, span / 2, steel());   // the two sides
  for (let i = 1; i < nY; i++) bar(thick, t, span, -depth / 2 + (i * depth) / nY, span / 2, web());   // the webs along the span
  for (let k = 1; k < nZ; k++) bar(thick, depth, t, 0, (k * span) / nZ, web());                       // and across it
  return g;
}

/** One surface of the file's `surfaces`: {group (a holder at its azimuth), kind, spec, index, set(deg, deployed, dt)}, or null for a kind that is not drawn (a flap hinged on the tangent). */
export function buildSurface(sf, index) {
  const R = sf.radius_m, holder = new THREE.Group();
  holder.rotation.y = (sf.azimuth_deg || 0) * DEG;                       // the body's azimuth 0 is model +Z, 90 is +X
  const f = { group: holder, kind: sf.kind, spec: sf, index, fold: sf.deployed === false ? 1 : 0, deg: sf.stow_deg || 0 };
  if (sf.kind === 'grid_fin') {
    const shaft = new THREE.Group(); shaft.position.set(0, sf.x_hinge_m, R + 0.3); holder.add(shaft);      // the shaft on the skin, at the middle of the fin's depth
    const fold = new THREE.Group(); shaft.add(fold);                      // folds about the depth axis (Y): 0 standing out along the radius, 90 lying along the skin round the body
    const turn = new THREE.Group(); fold.add(turn);                       // turns about the shaft (Z) by the deflection
    turn.add(gridFinMesh(sf.chord_m, sf.span_m));
    const housing = new THREE.Mesh(new THREE.CylinderGeometry(0.55, 0.65, 1.0, 14), steel()); housing.rotation.x = Math.PI / 2; housing.position.z = -0.05; shaft.add(housing);
    let seen = false;
    f.set = (deg, deployed, dt = 0.016) => {
      const want = deployed === false ? 1 : 0;
      if (!seen) { f.fold = want; seen = true; }                          // the first time it is as the pose says, not on its way
      else { const step = Math.min(dt, 0.1) / FOLD_S; f.fold += Math.max(-step, Math.min(step, want - f.fold)); }
      const e = f.fold * f.fold * (3 - 2 * f.fold);
      fold.rotation.y = e * (Math.PI / 2);
      turn.rotation.z = -(deg || 0) * DEG;                                // the simulator's chord turns from the axis toward the tangent with the deflection
    };
    f.set(sf.stow_deg || 0, sf.deployed !== false, 1);
    return f;
  }
  if (sf.axial_hinge) {
    const s = sf.chord_dir >= 0 ? 1 : -1, len = sf.span_m, wid = sf.chord_m;
    const pivot = new THREE.Group(); pivot.position.set(0, sf.x_hinge_m, R); holder.add(pivot);
    pivot.add(new THREE.Mesh(axialFlapGeometry(R, len, wid, s), [steel(), tile(), dark()]));
    const barrel = new THREE.Mesh(new THREE.CylinderGeometry(0.42, 0.42, len * 1.02, 14), steel()); barrel.position.set(0, len / 2, 0.28); pivot.add(barrel);       // the hinge barrel, along the hinge line
    for (const y of [0.04 * len, 0.96 * len]) { const br = new THREE.Mesh(new THREE.BoxGeometry(0.9, 0.9, 1.1), dark()); br.position.set(0, y, 0.1); pivot.add(br); }   // and a bracket at each end
    f.pivot = pivot;
    f.set = (deg) => { f.deg = deg; pivot.rotation.y = -s * deg * DEG; };  // about the axis: the free edge, flat toward +X for s = +1, turns out to +Z
    f.set(sf.stow_deg || 0);
    return f;
  }
  return null;
}

/** Every surface of the spec that is drawn, added to the group of its stage; returns the list (the model keeps it and moves them each frame). */
export function buildFlaps(spec, stageGroups) {
  const out = [];
  (spec.surfaces || []).forEach((sf, index) => {
    const g = stageGroups[sf.stage];
    if (!g) return;
    const f = buildSurface(sf, index);
    if (!f) return;
    g.add(f.group);
    out.push(f);
  });
  return out;
}

/** Moves a surface to what a pose says: `deg` its deflection, `deployed` whether it is (a grid fin folds when it is not), `dt` the seconds since the last call (the fins' fold is slow). */
export function setFlap(f, deg, deployed, dt = 0.016) { f.set(deg, deployed, dt); }
