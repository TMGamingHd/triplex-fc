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
import { tileMaterial } from './starship.js';
import { canvasTexture } from './common.js';

const DEG = Math.PI / 180;
const FOLD_S = 1.5;                         // seconds a grid fin takes to fold or deploy

let _steel = null, _dark = null, _web = null, _rib = null, _lee = null, _grid = null;
const steel = () => _steel || (_steel = new THREE.MeshStandardMaterial({ color: 0xd2d7dd, metalness: 0.55, roughness: 0.38 }));
const dark = () => _dark || (_dark = new THREE.MeshStandardMaterial({ color: 0x15171a, metalness: 0.4, roughness: 0.6, side: THREE.DoubleSide }));
const web = () => _web || (_web = new THREE.MeshStandardMaterial({ color: 0x3a3d42, metalness: 0.85, roughness: 0.5 }));
const rib = () => _rib || (_rib = new THREE.MeshStandardMaterial({ color: 0x8c9199, metalness: 0.7, roughness: 0.5 }));
/** The leeward face of a flap: bare stainless, with the weld lines of its sheets and a grain: a texture, so that it reads as raw steel and not as paint. */
const leeward = () => {
  if (_lee) return _lee;
  const tex = canvasTexture(256, 256, (g, w, h) => {
    g.fillStyle = '#c9ced4'; g.fillRect(0, 0, w, h);
    const im = g.getImageData(0, 0, w, h);
    for (let i = 0; i < im.data.length; i += 4) { const n = (((i * 2654435761) >>> 24) / 255 - 0.5) * 22; im.data[i] += n; im.data[i + 1] += n; im.data[i + 2] += n; }
    g.putImageData(im, 0, 0);
    g.fillStyle = 'rgba(70,74,80,.9)'; for (const y of [0, 128]) g.fillRect(0, y, w, 2);               // the horizontal welds of the sheets
    g.fillStyle = 'rgba(70,74,80,.8)'; for (const x of [0, 85, 170]) g.fillRect(x, 0, 2, h);           // and the vertical seams
    g.fillStyle = 'rgba(240,243,247,.7)'; for (const y of [3, 131]) g.fillRect(0, y, w, 1);            // the bright edge of a bead
  }, { repeat: [1, 1], color: true });
  return (_lee = new THREE.MeshStandardMaterial({ map: tex, metalness: 0.5, roughness: 0.42, side: THREE.DoubleSide }));
};

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
  const bins5 = (base, flip, pick) => {
    const bins = [[], [], [], [], []];
    for (let j = 0; j < NV; j++) for (let i = 0; i < NU; i++) {
      const a = base + j * (NU + 1) + i, b = a + 1, c = a + NU + 1, d = c + 1;
      bins[pick(i, j)].push(...(flip !== (s < 0) ? [a, c, b, b, c, d] : [a, b, c, b, d, c]));
    }
    bins.forEach((tri, m) => { if (tri.length) { groups.push([idx.length, tri.length, m]); idx.push(...tri); } });
  };
  bins5(outer, false, () => 1);                                                                  // the windward face: the tiles, all of it
  bins5(inner, true, (i, j) => ((i === 3 || i === 10 || j === 4 || j === 8) ? 4 : 3));            // the leeward face: bare steel with the ribs showing as darker lines
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

/** A grid fin standing out along +Z from its shaft at the origin: a robust squared-off frame `depth` deep along Y (the cells' axis) and `span` long along Z, `thick` across X, filled with a dense square lattice
 *  (perpendicular ribs, open pockets the air goes straight through), raw dark metal with a lighter bead along the frame's joints. */
export function gridFinMesh(depth, span, thick = 0.5) {
  const g = new THREE.Group();
  const bar = (sx, sy, sz, y, z, m) => { const b = new THREE.Mesh(new THREE.BoxGeometry(sx, sy, sz), m); b.position.set(0, y, z); g.add(b); };
  const f = Math.max(0.2, thick * 0.38), t = Math.max(0.07, thick * 0.1), nY = 9, nZ = 6;       // the frame's bar, an inner rib
  bar(thick, depth, f, 0, f / 2, web());                                                          // the root bar, on the shaft
  bar(thick, depth, f, 0, span - f / 2, web());                                                   // the tip bar
  bar(thick, f, span, depth / 2 - f / 2, span / 2, web()); bar(thick, f, span, -depth / 2 + f / 2, span / 2, web());   // the two sides
  for (let i = 1; i < nY; i++) bar(thick * 0.9, t, span - 2 * f, -depth / 2 + (i * depth) / nY, span / 2, web());      // the ribs along the span
  for (let k = 1; k < nZ; k++) bar(thick * 0.9, depth - 2 * f, t, 0, f + (k * (span - 2 * f)) / nZ, web());            // and across it
  for (const [y, z] of [[depth / 2 - f, f], [-depth / 2 + f, f], [depth / 2 - f, span - f], [-depth / 2 + f, span - f]]) {   // a weld bead at each inner corner of the frame
    const bead = new THREE.Mesh(new THREE.SphereGeometry(f * 0.42, 8, 6), rib()); bead.position.set(thick * 0.5, y, z); g.add(bead);
  }
  for (const y of [depth / 2, -depth / 2]) { const bead = new THREE.Mesh(new THREE.BoxGeometry(thick * 1.04, t * 1.2, span * 0.96), rib()); bead.position.set(0, y, span / 2); g.add(bead); }
  return g;
}

/** The pod a grid fin mounts on: a rounded structural fairing welded to the skin that houses the hydraulic actuator, with a reinforced base. `pin`: a catch pin stands out of it, along the radius, where the tower's
 *  arm takes the stage. Frame: the fin's holder (Z the radius, X the tangent, Y the axis), the origin on the skin at the fin's height. */
export function finPod(R, depth, withPin) {
  const g = new THREE.Group();
  const body = new THREE.Mesh(new THREE.SphereGeometry(1, 20, 14), dark()); body.scale.set(Math.max(1.1, depth * 0.34), Math.max(1.4, depth * 0.52), 0.95); body.position.set(0, 0, R + 0.2); g.add(body);   // the pod: an egg 2.8 m tall, 1.9 m wide
  const base = new THREE.Mesh(new THREE.BoxGeometry(Math.max(2.2, depth * 0.55), Math.max(2.8, depth * 0.75), 0.5), web()); base.position.set(0, 0, R - 0.02); g.add(base);                       // its reinforced base on the skin
  const plate = new THREE.Mesh(new THREE.BoxGeometry(Math.max(2.6, depth * 0.62), Math.max(3.2, depth * 0.84), 0.12), rib()); plate.position.set(0, 0, R + 0.28); g.add(plate);                  // a doubler plate round it, its edge a bright weld
  if (withPin) {
    const pin = new THREE.Mesh(new THREE.CylinderGeometry(0.36, 0.36, 1.5, 16), steel()); pin.rotation.x = Math.PI / 2; pin.position.set(0, 0, R + 1.55); g.add(pin);                              // the catch pin, out of the pod's outer face
    const collar = new THREE.Mesh(new THREE.CylinderGeometry(0.62, 0.62, 0.35, 16), web()); collar.rotation.x = Math.PI / 2; collar.position.set(0, 0, R + 0.95); g.add(collar);
  }
  return g;
}

/** One surface of the file's `surfaces`: {group (a holder at its azimuth), kind, spec, index, set(deg, deployed, dt)}, or null for a kind that is not drawn (a flap hinged on the tangent). */
export function buildSurface(sf, index) {
  const R = sf.radius_m, holder = new THREE.Group();
  holder.rotation.y = (sf.azimuth_deg || 0) * DEG;                       // the body's azimuth 0 is model +Z, 90 is +X
  const f = { group: holder, kind: sf.kind, spec: sf, index, fold: sf.deployed === false ? 1 : 0, deg: sf.stow_deg || 0 };
  if (sf.kind === 'grid_fin') {
    const flank = Math.abs(Math.sin((sf.azimuth_deg || 0) * DEG)) > 0.9;                                       // the two flank fins are the ones the arms take: their pods carry the pins
    const podG = finPod(R, sf.chord_m, flank); podG.position.set(0, sf.x_hinge_m, 0); holder.add(podG);
    const shaft = new THREE.Group(); shaft.position.set(0, sf.x_hinge_m, R + 1.05); holder.add(shaft);     // the shaft above the pod, at the middle of the fin's depth
    const fold = new THREE.Group(); shaft.add(fold);                      // folds about the depth axis (Y): 0 standing out along the radius, 90 lying along the skin round the body
    const turn = new THREE.Group(); fold.add(turn);                       // turns about the shaft (Z) by the deflection
    turn.add(gridFinMesh(sf.chord_m, sf.span_m));
    const housing = new THREE.Mesh(new THREE.CylinderGeometry(0.62, 0.72, 0.9, 16), steel()); housing.rotation.x = Math.PI / 2; housing.position.z = -0.05; shaft.add(housing);
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
    pivot.add(new THREE.Mesh(axialFlapGeometry(R, len, wid, s), [steel(), tileMaterial(), dark(), leeward(), rib()]));
    // the hinge, exposed and heavy: a steel barrel along the hinge line, a bracket at each end, an actuator housing at the middle with two linkage rods to the plate, and a half-shell fairing over the barrel that
    // keeps the stream out of the gap
    const barrel = new THREE.Mesh(new THREE.CylinderGeometry(0.42, 0.42, len * 1.02, 14), steel()); barrel.position.set(0, len / 2, 0.28); pivot.add(barrel);
    for (const y of [0.04 * len, 0.96 * len]) { const br = new THREE.Mesh(new THREE.BoxGeometry(0.9, 0.9, 1.1), dark()); br.position.set(0, y, 0.1); pivot.add(br); }
    const act = new THREE.Mesh(new THREE.BoxGeometry(0.9, Math.max(1.4, len * 0.14), 0.8), dark()); act.position.set(-s * 0.25, len * 0.5, 0.5); pivot.add(act);
    for (const y of [0.42, 0.58]) { const rod = new THREE.Mesh(new THREE.CylinderGeometry(0.09, 0.09, 1.6, 8), steel()); rod.position.set(-s * 0.55, len * y, 0.95); rod.rotation.z = s * 0.9; pivot.add(rod); }
    const fair = new THREE.Mesh(new THREE.CylinderGeometry(0.72, 0.72, len * 0.98, 16, 1, true, 0, Math.PI), dark()); fair.position.set(0, len / 2, 0.3); fair.rotation.y = s > 0 ? Math.PI / 2 + Math.PI : Math.PI / 2; pivot.add(fair);
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
