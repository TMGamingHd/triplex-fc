// The flaps of a vehicle (a Starship-class ship's four), drawn from the vehicle file's `surfaces` table and moved by the deflection the simulator reports in every pose (`pose.surf`): the same numbers the
// aerodynamics use (sim/vehicle/surfaces.hpp, docs/design/AERODYNAMICS.md section 9), so a flap that stands out in the picture is a flap that stands out in the air.
//   - lying flush (deflection 0) a flap is a curved panel that follows the hull, a little proud of it, in bare steel with a dark seam round it, and a hinge fairing on its hinge edge;
//   - the hinge is the straight line across the panel at `x_hinge`, on the skin; the free edge points forward (chord_dir +1) or aft (-1) and lifts radially as the deflection grows (the simulator's convention,
//     surface_frame in surfaces.hpp), so 90 degrees is a plate standing straight out into the stream;
//   - the outline is a trapezoid, wider at the hinge, with its outer corners rounded: public pictures show that, not a rectangle; the numbers (chord, span, area) are the file's, and the file's are estimates.
// Frames: the stage's group, model axes (Y along the body, Z toward the body's +Y, X toward the body's +Z), like everything in models/.
import * as THREE from '../../vendor/three.module.js';

const DEG = Math.PI / 180;
const NU = 16, NV = 10;                       // the grid of a face: across the span, along the chord

let _steel = null, _dark = null, _tile = null;
const steel = () => _steel || (_steel = new THREE.MeshStandardMaterial({ color: 0xc9ced4, metalness: 0.5, roughness: 0.42 }));
const dark = () => _dark || (_dark = new THREE.MeshStandardMaterial({ color: 0x15171a, metalness: 0.4, roughness: 0.6, side: THREE.DoubleSide }));
const tile = () => _tile || (_tile = new THREE.MeshStandardMaterial({ color: 0x1b1c1f, metalness: 0.1, roughness: 0.55, side: THREE.DoubleSide }));

/** The half-width of the outline at v (0 at the hinge, 1 at the free edge): the taper, and the corners rounded over the last fifth of the chord. */
function halfWidth(span, v, taper) {
  const t = Math.max(0, (v - 0.8) / 0.2);
  return 0.5 * span * (1 - taper * v) * (1 - 0.3 * t * t);
}

/** A panel of the outline above, bent round the hull of radius R: its outer face at R + thick, its inner at R + gap, the chord along +Y * dir from the hinge at the origin. Faces: outer (steel), inner (tile), edge (dark). */
export function flapGeometry(R, span, chord, dir, taper = 0.22, thick = 0.32, gap = 0.04) {
  const pos = [], idx = [], groups = [];
  const at = (u, v, rad) => {                                 // u in [-1, 1] across the span, v in [0, 1] along the chord
    const x = u * halfWidth(span, v, taper), y = dir * v * chord;
    return [x, y, Math.sqrt(Math.max(rad * rad - x * x, 1)) - R];   // the hinge is at the radius R on the middle line; the sides follow the hull down
  };
  const layer = (rad) => {
    const base = pos.length / 3;
    for (let j = 0; j <= NV; j++) for (let i = 0; i <= NU; i++) pos.push(...at((2 * i) / NU - 1, j / NV, rad));
    return base;
  };
  const face = (base, flip, mat) => {
    const start = idx.length;
    for (let j = 0; j < NV; j++) for (let i = 0; i < NU; i++) {
      const a = base + j * (NU + 1) + i, b = a + 1, c = a + NU + 1, d = c + 1;
      if (flip !== (dir < 0)) idx.push(a, c, b, b, c, d); else idx.push(a, b, c, b, d, c);
    }
    groups.push([start, idx.length - start, mat]);
  };
  const outer = layer(R + thick), inner = layer(R + gap);
  face(outer, false, 0);
  face(inner, true, 1);
  // the edge: a strip round the outline joining the two faces
  const start = idx.length, ring = [];
  for (let i = 0; i <= NU; i++) ring.push(i);                                           // hinge edge, left to right
  for (let j = 1; j <= NV; j++) ring.push(j * (NU + 1) + NU);                           // right side, up the chord
  for (let i = NU - 1; i >= 0; i--) ring.push(NV * (NU + 1) + i);                       // free edge, right to left
  for (let j = NV - 1; j >= 1; j--) ring.push(j * (NU + 1));                            // left side, down the chord
  for (let k = 0; k < ring.length; k++) {
    const a = ring[k], b = ring[(k + 1) % ring.length];
    idx.push(outer + a, inner + a, outer + b, outer + b, inner + a, inner + b);
  }
  groups.push([start, idx.length - start, 2]);
  const geo = new THREE.BufferGeometry();
  geo.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3));
  geo.setIndex(idx);
  for (const [s, n, m] of groups) geo.addGroup(s, n, m);
  geo.computeVertexNormals();
  return geo;
}

/** The flap of one `surfaces` entry: a holder turned to its azimuth, a pivot on the hinge that the deflection turns, and the panel and its hinge fairing in the pivot. Returns {group, pivot, dir, spec}. */
export function buildFlap(sf) {
  const R = sf.radius_m, span = sf.span_m, chord = sf.chord_m, dir = sf.chord_dir >= 0 ? 1 : -1;
  const holder = new THREE.Group();
  holder.rotation.y = (sf.azimuth_deg || 0) * DEG;            // the body's azimuth 0 is model +Z, 90 is +X
  const pivot = new THREE.Group();
  pivot.position.set(0, sf.x_hinge_m, R);
  holder.add(pivot);
  const panel = new THREE.Mesh(flapGeometry(R, span, chord, dir), [steel(), tile(), dark()]);
  pivot.add(panel);
  const fair = new THREE.Mesh(new THREE.CylinderGeometry(0.3, 0.3, span * 0.98, 12), dark());   // the hinge fairing, along the tangent
  fair.rotation.z = Math.PI / 2; fair.position.set(0, 0, 0.12);
  pivot.add(fair);
  const f = { group: holder, pivot, dir, spec: sf };
  setFlap(f, sf.stow_deg || 0);
  return f;
}

/** The deflection: positive lifts the free edge out of the skin, whichever way it points. */
export function setFlap(f, deg) {
  f.pivot.rotation.x = f.dir * deg * DEG;
}

/** Every flap of the spec's `surfaces`, added to the group of its stage; returns the list (the model keeps it and moves them each frame). */
export function buildFlaps(spec, stageGroups) {
  const out = [];
  (spec.surfaces || []).forEach((sf, index) => {
    if (sf.kind !== 'flap') return;
    const g = stageGroups[sf.stage];
    if (!g) return;
    const f = buildFlap(sf);
    f.index = index;
    g.add(f.group);
    out.push(f);
  });
  return out;
}
