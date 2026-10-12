// The Starship dressing: what makes a 9 m stainless stack look like one, laid over the generic body that vehicle.js builds from the spec (the diameters, the stage lengths and the engines are the spec's own).
//   booster: a hot-stage ring with its vents at the top, grid fins stowed against the skin below it (three on Block 3, four before), two catch pins, the engine skirt and its thrust plate, soot on the lower tank;
//   ship:    heat-shield tiles on the belly and the nose (its flaps are the vehicle file's surfaces: models/flaps.js).
// The proportions are of the vehicle class (public figures), not drawings; no markings are drawn. The details a person may want to change (the fins' number, size and place, the flaps, the ring) are `dress`,
// the page's Starship panel; V3 is the default. docs/design/VIEWER.md lists what is drawn from the spec and what is a visual choice.
import * as THREE from '../../vendor/three.module.js';
import { lathe, canvasTexture, outline, tileTextures } from './common.js';

const TAU = Math.PI * 2;

/** The parts of the dressing that are a choice and not the vehicle file's: Block 3 as reported (two grid fins, each 50 % larger in area than Block 2's, set lower on the booster) and Block 2 before it.
 *  finScale is linear (the fin of Block 2 is 2.2 m by 3.4 m here), finDrop the metres from the bottom of the hot-stage ring to the top of the fin. These are illustrations of public descriptions, not drawings. */
export const DRESS = {
  v3: { name: 'Block 3 (V3)', fins: 3, finScale: 1.22, finDrop: 5.0, ringHeight: 3.4, pins: true },
  v2: { name: 'Block 2 (V2)', fins: 4, finScale: 1.0, finDrop: 1.6, ringHeight: 3.4, pins: true },
};
export const dressOf = (d) => Object.assign({}, DRESS.v3, d || {});

function ventTexture() {
  return canvasTexture(256, 64, (g, w, h) => {
    g.fillStyle = '#6a6e73'; g.fillRect(0, 0, w, h);
    for (let i = 0; i < 24; i++) { g.fillStyle = '#0b0c0e'; g.fillRect(i * (w / 24) + 3, 10, w / 24 - 8, h - 20); }   // the slots a stage's exhaust escapes through
    g.fillStyle = '#9aa0a6'; g.fillRect(0, 0, w, 6); g.fillRect(0, h - 6, w, 6);
  }, { color: true });
}

function gridFin(mats, w = 2.2, hgt = 3.4, depth = 0.55) {
  // a frame and a lattice of thin bars: the grid fin
  const g = new THREE.Group();
  const bar = (sx, sy, sz, x, y, z) => { const m = new THREE.Mesh(new THREE.BoxGeometry(sx, sy, sz), mats.fin); m.position.set(x, y, z); g.add(m); };
  const t = 0.07, n = 7;
  bar(w, t * 2, depth, 0, hgt / 2, 0); bar(w, t * 2, depth, 0, -hgt / 2, 0); bar(t * 2, hgt, depth, w / 2, 0, 0); bar(t * 2, hgt, depth, -w / 2, 0, 0);
  for (let i = 1; i < n; i++) { bar(t, hgt, depth * 0.9, -w / 2 + (w * i) / n, 0, 0); bar(w, t, depth * 0.9, 0, -hgt / 2 + (hgt * i) / n, 0); }
  const hinge = new THREE.Mesh(new THREE.CylinderGeometry(0.22, 0.22, w * 0.5, 12), mats.steel); hinge.rotation.z = Math.PI / 2; hinge.position.set(0, -hgt / 2 - 0.15, 0); g.add(hinge);
  return g;
}

export function decorateStarship(model) {
  const spec = model.spec;
  const steel = model.stages[0].body.material;                // the steel vehicle.js made for the body
  const dark = new THREE.MeshStandardMaterial({ color: 0x14161a, metalness: 0.6, roughness: 0.55, side: THREE.DoubleSide });
  const fin = new THREE.MeshStandardMaterial({ color: 0x2a2d31, metalness: 0.9, roughness: 0.42 });
  const plate = new THREE.MeshStandardMaterial({ color: 0x3a3733, metalness: 0.8, roughness: 0.6, side: THREE.DoubleSide });
  const m = { steel, dark, fin, plate };
  const booster = model.stages[0], ship = model.stages[1];
  const R = spec.diameter / 2, dress = dressOf(model.opts && model.opts.dress);
  const topB = (booster.spec.x_start_m || 0) + booster.spec.length_m;

  // ---- the booster
  {
    const g = booster.group;
    // the thrust plate and the skirt's inner wall: the engines come out of a dark plate
    const plateGeo = new THREE.CircleGeometry(R * 0.985, 64); plateGeo.rotateX(-Math.PI / 2); // faces down
    const p = new THREE.Mesh(plateGeo, m.plate); p.position.y = 2.7; g.add(p);
    const inner = new THREE.Mesh(lathe([{ x: 0, r: R - 0.04 }, { x: 2.7, r: R - 0.04 }], { segments: 64, flip: true, colour: () => [.5, .5, .5] }), new THREE.MeshStandardMaterial({ color: 0x2b2e33, metalness: 0.7, roughness: 0.6, vertexColors: true, side: THREE.DoubleSide })); g.add(inner);
    // the hot-stage ring: a collar a little proud of the skin, slotted all round
    const ringTop = topB, ringH = dress.ringHeight;
    const ringTex = ventTexture(); ringTex.repeat.set(2, 1);
    const ringMat = new THREE.MeshStandardMaterial({ map: ringTex, metalness: 0.8, roughness: 0.5, vertexColors: true }); model.hull.push(ringMat);
    const ring = new THREE.Mesh(lathe([{ x: ringTop - ringH, r: R + 0.04 }, { x: ringTop - ringH + 0.25, r: R + 0.17 }, { x: ringTop - 0.35, r: R + 0.17 }, { x: ringTop, r: R + 0.03 }], { segments: 96, colour: () => [1, 1, 1] }), ringMat);
    ring.geometry.attributes.uv.array.forEach((v, i, a) => { if (i % 2 === 1) a[i] = (a[i] * 1.8 - (ringTop - ringH)) / ringH; });   // v from 0 at the bottom of the ring to 1 at the top
    ring.geometry.attributes.uv.needsUpdate = true;
    g.add(ring);
    // a rim of darker steel under the ring and a raceway (the cable and propellant-line fairing) down the side away from the grid fins, from the engine bay to the ring
    const rim = new THREE.Mesh(lathe([{ x: ringTop - ringH - 0.5, r: R + 0.02 }, { x: ringTop - ringH - 0.5, r: R + 0.09 }, { x: ringTop - ringH, r: R + 0.09 }], { segments: 96, colour: () => [0.55, 0.55, 0.57] }), steel); g.add(rim);
    const raceway = new THREE.Group(); raceway.rotation.y = Math.PI;                          // (azimuth 180: the fins are at 96 and 264 and the spine side is open)
    const rl = ringTop - ringH - 0.5 - 5.0;
    const rb = new THREE.Mesh(new THREE.BoxGeometry(0.85, rl, 0.3), steel); rb.position.set(0, 5.0 + rl / 2, R + 0.1); raceway.add(rb);
    for (let y = 8; y < 5.0 + rl; y += 9) { const band = new THREE.Mesh(new THREE.BoxGeometry(1.05, 0.28, 0.36), m.plate); band.position.set(0, y, R + 0.12); raceway.add(band); }   // its straps
    g.add(raceway);
    // the grid fins are the vehicle file's `surfaces` (models/flaps.js folds and deploys them as the simulator says). A file without any gets the viewer's stowed ones, so that a booster always has its fins.
    if (!(spec.surfaces || []).some((x) => x.kind === 'grid_fin' && x.stage === 0)) {
      const fw = 2.2 * dress.finScale, fh = 3.4 * dress.finScale, fy = ringTop - ringH - dress.finDrop - fh / 2;
      for (let i = 0; i < dress.fins; i++) {
        const a = Math.PI / dress.fins + (i * TAU) / dress.fins;
        const holder = new THREE.Group(); holder.rotation.y = a;
        const f = gridFin(m, fw, fh, 0.55 * dress.finScale); f.position.set(0, fy, R + 0.32);
        holder.add(f); g.add(holder);
      }
    }
    // the catch pins are on the pods of the two flank grid fins (models/flaps.js): the fins are the stage's load-bearing points, the arms take it there
  }
  // ---- the ship
  if (ship) {
    const g = ship.group;
    const x0 = ship.spec.x_start_m, tubeEnd = (ship.spec.sections || []).filter((s) => s.kind === 'tube').reduce((a, s) => Math.max(a, s.x_start_m + s.length_m), x0 + 20);
    const bellyHalf = (200 * Math.PI) / 360;              // tiles cover 200 degrees of the tube, centred on the belly
    const belly = 0;                                      // the belly (the heat shield, the side that meets the air in the entry) faces model +Z: the body's +y, azimuth 0 of the surfaces table (lathe's phi 0 is +Z, phi pi/2 is +X: the tiles were 90 degrees away, on the flaps' side)
    const tileMat = tileMaterial().clone(); model.hull.push(tileMat);                    // (this model's own: the cutaway clips it)
    const tubeProf = outline((ship.spec.sections || []).filter((s) => s.kind !== 'nose'));
    const noseProf = outline((ship.spec.sections || []).filter((s) => s.kind === 'nose'));
    const grow = (p, d) => p.map((q) => ({ x: q.x, r: q.r + d }));
    if (tubeProf.length) g.add(new THREE.Mesh(lathe(grow(tubeProf, 0.035), { segments: 40, phiStart: belly - bellyHalf, phiLength: 2 * bellyHalf, ring: 2.6, colour: () => [1, 1, 1], uRepeat: 0.33 }), tileMat));
    if (noseProf.length) g.add(new THREE.Mesh(lathe(grow(noseProf, 0.035), { segments: 56, phiStart: belly - Math.PI * 0.72, phiLength: Math.PI * 1.44, ring: 2.6, colour: () => [1, 1, 1], uRepeat: 0.33 }), tileMat));
    // the edge of the heat shield: a strip of darker steel along each side of the tiles, where the tiles end and the bare skin begins
    for (const sgn of [-1, 1]) {
      const edge = new THREE.Mesh(lathe(grow(tubeProf, 0.05), { segments: 2, phiStart: belly + sgn * (bellyHalf - 0.012), phiLength: 0.024, ring: 2.6, colour: () => [0.32, 0.33, 0.35] }), steel);
      edge.material = m.plate; g.add(edge);
    }
    // four small attitude-thruster pods near the tip of the nose, dark rectangles in the skin
    const noseSec = (ship.spec.sections || []).find((q) => q.kind === 'nose');
    if (noseSec) for (let k = 0; k < 4; k++) {
      const holder = new THREE.Group(); holder.rotation.y = (k * Math.PI) / 2 + Math.PI / 4;
      const pod = new THREE.Mesh(new THREE.BoxGeometry(0.55, 0.9, 0.12), m.dark); pod.position.set(0, noseSec.x_start_m + noseSec.length_m - 1.8, 1.05); pod.rotation.x = -0.28; holder.add(pod); g.add(holder);
    }
    // the flaps are not drawn here: they are the vehicle file's `surfaces`, built and moved by models/flaps.js
    // the base: the engine skirt of the ship
    const sk = new THREE.Mesh(new THREE.CircleGeometry(R * 0.97, 48), m.plate); sk.geometry.rotateX(-Math.PI / 2); sk.position.y = x0 + 1.6; g.add(sk);
  }
}

let _tile = null;
export function tileMaterial() {
  if (_tile) return _tile;
  const t = tileTextures();
  return (_tile = new THREE.MeshStandardMaterial({ color: 0x777777, map: t.map, bumpMap: t.bump, bumpScale: 2.2, metalness: 0.05, roughness: 0.52 }));
}
