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
const FOLD_S = 1.5;
const POD_UP_M = 1.5;                       // the fin's shaft is this far above the catch pin (the file puts the fins at `x_hinge_m` 62.5 and the pins at `catch_pin_x_m` 61.0): the fins rest on the arms                         // seconds a grid fin takes to fold or deploy

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

/** The plan of a Starship flap, as the pictures of the ship show it: a flat, thick plate in the shape of a trapezoid. The long edge is the hinge, along the body; the outer edge is shorter and the leading edge (the one
 *  toward the nose) is swept back; the corners are cut, not round. `aft`: the big rear flaps (a long root, a strongly swept leading edge, the trailing edge nearly square to the hull); the forward flaps are smaller
 *  and the more pointed ("ears"). Returns the outline as [along the hinge (0 to len), outward from it (0 to wid)] points, round the plate. */
export function flapOutline(len, wid, aft) {
  const lead = aft ? [0.66, 0.15] : [0.62, 0.20], c = Math.min(0.4, 0.06 * wid);      // where the outer edge begins and ends along the axis (as fractions of the length); the cut of the corners
  const y0 = len * lead[1], y1 = len * lead[0];
  return [[0, 0], [len, 0], [len - c * 0.3, c * 1.2], [y1 + c * 1.1, wid - c * 0.55], [y1, wid], [y0 + c * 0.6, wid], [y0 - c * 0.2, wid - c * 0.8], [0 + c * 0.2, c * 1.1]];
}

/** A flat flap: the outline extruded `thick` out of the plane, the hinge along Y at the origin, the plate lying toward `s` (+1 or -1) along X and its thickness along +Z. Faces: the outer one (the tile face), the inner
 *  one (the leeward face: bare steel) and the edge all round; the texture follows the plan, so that tiles and weld lines are in the plate's own scale. */
export function flatFlapGeometry(len, wid, s, aft, thick = 0.9) {
  const ol = flapOutline(len, wid, aft), n = ol.length, pos = [], uv = [], idx = [], groups = [];
  const at = (pt, z) => [s * pt[1], pt[0], z];
  const face = (z, flip, mat) => {
    const base = pos.length / 3;
    for (const pt of ol) { pos.push(...at(pt, z)); uv.push(pt[1] / 3.0, pt[0] / 3.0); }
    const st = idx.length;
    for (let i = 1; i < n - 1; i++) idx.push(...((flip !== (s < 0)) ? [base, base + i + 1, base + i] : [base, base + i, base + i + 1]));
    groups.push([st, idx.length - st, mat]);
  };
  face(thick, false, 1);                                     // outer: the tiles
  face(0, true, 3);                                          // inner: the steel
  const base = pos.length / 3, st = idx.length;
  for (let i = 0; i < n; i++) { pos.push(...at(ol[i], thick), ...at(ol[i], 0)); uv.push(ol[i][1] / 3.0, ol[i][0] / 3.0, ol[i][1] / 3.0, ol[i][0] / 3.0); }
  for (let i = 0; i < n; i++) {
    const j = (i + 1) % n, a = base + 2 * i, b = base + 2 * j;            // outer(i), inner(i), outer(j), inner(j)
    idx.push(...((s > 0) ? [a, a + 1, b, b, a + 1, b + 1] : [a, b, a + 1, b, b + 1, a + 1]));
  }
  groups.push([st, idx.length - st, 2]);
  const geo = new THREE.BufferGeometry();
  geo.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3));
  geo.setAttribute('uv', new THREE.Float32BufferAttribute(uv, 2));
  geo.setIndex(idx);
  for (const [a, c, m] of groups) geo.addGroup(a, c, m);
  geo.computeVertexNormals();
  return geo;
}

/** A grid fin, as the pictures of Falcon and Super Heavy show it: a flat lattice whose face is `width` along X (the tangent) by `span` along Z (the radius), standing out from its shaft at the origin along +Z, and `depth`
 *  thin along Y, the axis: the cells open along Y, the way the air goes when the stage falls, so that with the stage upright the fin is parallel to the ground. A strong frame round a square lattice, dark metal, a lighter
 *  bead along the joints of the frame. */
export function gridFinMesh(width, span, depth = 0.7) {
  const g = new THREE.Group();
  const bar = (sx, sy, sz, x, z, m) => { const b = new THREE.Mesh(new THREE.BoxGeometry(sx, sy, sz), m); b.position.set(x, 0, z); g.add(b); };
  const f = 0.26, t = 0.09, nX = 12, nZ = 8;                                                      // the frame's bar, a rib, the cells across and along
  bar(width, depth, f, 0, f / 2, web());                                                          // the root bar, on the shaft
  bar(width, depth, f, 0, span - f / 2, web());                                                   // the tip bar
  bar(f, depth, span, -width / 2 + f / 2, span / 2, web()); bar(f, depth, span, width / 2 - f / 2, span / 2, web());   // the two sides
  for (let i = 1; i < nX; i++) bar(t, depth * 0.94, span - 2 * f, -width / 2 + (i * width) / nX, span / 2, web());     // the ribs along the radius
  for (let k = 1; k < nZ; k++) bar(width - 2 * f, depth * 0.94, t, 0, f + (k * (span - 2 * f)) / nZ, web());          // and across it
  for (const [x, z] of [[-width / 2 + f, f], [width / 2 - f, f], [-width / 2 + f, span - f], [width / 2 - f, span - f]]) {   // a weld bead at each inner corner of the frame
    const bead = new THREE.Mesh(new THREE.SphereGeometry(f * 0.42, 8, 6), rib()); bead.position.set(x, depth * 0.5, z); g.add(bead);
  }
  for (const x of [-width / 2, width / 2]) { const bead = new THREE.Mesh(new THREE.BoxGeometry(t * 1.3, depth * 1.04, span * 0.96), rib()); bead.position.set(x, 0, span / 2); g.add(bead); }
  return g;
}

/** The pod a grid fin mounts on: a rounded structural fairing welded to the skin that houses the hydraulic actuator, with a reinforced base. `pin`: a catch pin stands out of it, along the radius, where the tower's
 *  arm takes the stage. Frame: the fin's holder (Z the radius, X the tangent, Y the axis), the origin on the skin at the fin's height. */
export function finPod(R, depth, withPin) {
  const g = new THREE.Group();
  const body = new THREE.Mesh(new THREE.SphereGeometry(1, 20, 14), dark()); body.scale.set(1.15, 2.4, 0.95); body.position.set(0, POD_UP_M * 0.5, R + 0.2); g.add(body);       // the pod: an egg 4.8 m tall from the pin to the shaft, 2.3 m wide
  const base = new THREE.Mesh(new THREE.BoxGeometry(2.4, 4.6, 0.5), web()); base.position.set(0, POD_UP_M * 0.5, R - 0.02); g.add(base);                                  // its reinforced base on the skin
  const plate = new THREE.Mesh(new THREE.BoxGeometry(2.8, 5.0, 0.12), rib()); plate.position.set(0, POD_UP_M * 0.5, R + 0.28); g.add(plate);                              // a doubler plate round it, its edge a bright weld
  if (withPin) {
    const pin = new THREE.Mesh(new THREE.CylinderGeometry(0.36, 0.36, 1.5, 16), steel()); pin.rotation.x = Math.PI / 2; pin.position.set(0, 0, R + 1.55); g.add(pin);       // the catch pin, out of the pod's outer face, a metre and a half under the fin
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
    const podG = finPod(R, sf.chord_m, flank); podG.position.set(0, sf.x_hinge_m - POD_UP_M, 0); holder.add(podG);
    const shaft = new THREE.Group(); shaft.position.set(0, sf.x_hinge_m, R + 1.05); holder.add(shaft);     // the shaft above the pod, at the middle of the fin's depth
    const fold = new THREE.Group(); shaft.add(fold);                      // folds about the tangent (X): 0 standing out along the radius, parallel to the ground; 90 lying along the skin toward the tail
    const turn = new THREE.Group(); fold.add(turn);                       // turns about the shaft (Z) by the deflection
    turn.add(gridFinMesh(sf.chord_m, sf.span_m));                       // the face: the file's chord_m across (the tangent) by span_m out (the radius): its area_m2
    const housing = new THREE.Mesh(new THREE.CylinderGeometry(0.62, 0.72, 0.9, 16), steel()); housing.rotation.x = Math.PI / 2; housing.position.z = -0.05; shaft.add(housing);
    let seen = false;
    f.set = (deg, deployed, dt = 0.016) => {
      const want = deployed === false ? 1 : 0;
      if (!seen) { f.fold = want; seen = true; }                          // the first time it is as the pose says, not on its way
      else { const step = Math.min(dt, 0.1) / FOLD_S; f.fold += Math.max(-step, Math.min(step, want - f.fold)); }
      const e = f.fold * f.fold * (3 - 2 * f.fold);
      fold.rotation.x = e * (Math.PI / 2);
      turn.rotation.z = -(deg || 0) * DEG;                                // the simulator's chord turns from the axis toward the tangent with the deflection
    };
    f.set(sf.stow_deg || 0, sf.deployed !== false, 1);
    return f;
  }
  if (sf.axial_hinge) {
    const s = sf.chord_dir >= 0 ? 1 : -1, len = sf.span_m, wid = sf.chord_m, aft = len > 7.5;                 // the big rear flaps and the smaller forward ones
    const pivot = new THREE.Group(); pivot.position.set(0, sf.x_hinge_m, R + 0.15); holder.add(pivot);
    pivot.add(new THREE.Mesh(flatFlapGeometry(len, wid, s, aft), [steel(), tileMaterial(), dark(), leeward(), rib()]));
    // the hinge, tight to the body: a barrel along the hinge line with a bracket at each end and a low half-shell fairing over it that keeps the stream out of the gap
    const barrel = new THREE.Mesh(new THREE.CylinderGeometry(0.3, 0.3, len * 1.0, 14), steel()); barrel.position.set(0, len / 2, 0.22); pivot.add(barrel);
    for (const y of [0.05 * len, 0.95 * len]) { const br = new THREE.Mesh(new THREE.BoxGeometry(0.7, 0.7, 0.8), dark()); br.position.set(0, y, 0.1); pivot.add(br); }
    const fair = new THREE.Mesh(new THREE.CylinderGeometry(0.46, 0.46, len * 0.96, 16, 1, true, 0, Math.PI), dark()); fair.position.set(0, len / 2, 0.22); fair.rotation.y = s > 0 ? Math.PI / 2 + Math.PI : Math.PI / 2; pivot.add(fair);
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
