// The Starship dressing: what makes a 9 m stainless stack look like one, laid over the generic body that vehicle.js builds from the spec (the diameters, the stage lengths and the engines are the spec's own).
//   booster: a hot-stage ring with its vents at the top, grid fins stowed against the skin below it (three on Block 3, four before), two catch pins, the engine skirt and its thrust plate, soot on the lower tank;
//   ship:    heat-shield tiles on the belly and the nose, two forward and two aft flaps stowed against the skin.
// The proportions are of the vehicle class (public figures), not drawings; no markings are drawn. The details a person may want to change (the fins' number, size and place, the flaps, the ring) are `dress`,
// the page's Starship panel; V3 is the default. docs/design/VIEWER.md lists what is drawn from the spec and what is a visual choice.
import * as THREE from '../../vendor/three.module.js';
import { lathe, canvasTexture, outline, tileTextures } from './common.js';

const TAU = Math.PI * 2;

/** The parts of the dressing that are a choice and not the vehicle file's: Block 3 as reported (three grid fins, each 50 % larger in area than Block 2's, set lower on the booster) and Block 2 before it.
 *  flapAngle is how far the flaps stand out from the skin (0: stowed flat against it, as in the climb; the viewer shows them part-way so that they can be seen, and says so). finScale is linear (the fin of Block 2 is 2.2 m by 3.4 m here), finDrop the metres from the bottom of the hot-stage ring to the top of the fin. These are illustrations of public descriptions, not drawings. */
export const DRESS = {
  v3: { name: 'Block 3 (V3)', fins: 3, finScale: 1.22, finDrop: 5.0, flapScale: 1.0, flapAngle: 30, ringHeight: 3.4, pins: true },
  v2: { name: 'Block 2 (V2)', fins: 4, finScale: 1.0, finDrop: 1.6, flapScale: 1.0, flapAngle: 30, ringHeight: 3.4, pins: true },
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

let _flapSteel = null;
/** Bare steel that is seen from the side: less of a mirror than the hull's, which reflects a dark sky and turns black in shadow. */
function flapSteel() { return _flapSteel || (_flapSteel = new THREE.MeshStandardMaterial({ color: 0xc4c9cf, metalness: 0.45, roughness: 0.5 })); }

function flap(mats, w, hgt, thick = 0.4) {
  const tileMat = tileMaterial();
  const g = new THREE.Group();
  const shape = new THREE.Shape();                    // a rounded, slightly tapered flap
  shape.moveTo(-w / 2, 0); shape.lineTo(w / 2, 0); shape.lineTo(w * 0.42, hgt); shape.lineTo(-w * 0.42, hgt); shape.closePath();
  const geo = new THREE.ExtrudeGeometry(shape, { depth: thick, bevelEnabled: true, bevelSize: 0.06, bevelThickness: 0.06, bevelSegments: 1 });
  geo.translate(0, 0, -thick / 2);
  const m = new THREE.Mesh(geo, flapSteel()); g.add(m);
  const pg = new THREE.PlaneGeometry(w * 0.9, hgt * 0.93); pg.attributes.uv.array.forEach((v, i, a) => { a[i] = v * (i % 2 ? hgt / 2.6 : w / 2.6); });
  const tileSide = new THREE.Mesh(pg, tileMat); tileSide.rotation.y = Math.PI; tileSide.position.set(0, hgt / 2, -(thick / 2 + 0.09)); g.add(tileSide);   // the tiles on the face toward the belly (the windward side), bare steel on the outer face
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
    const ring = new THREE.Mesh(lathe([{ x: ringTop - ringH, r: R + 0.06 }, { x: ringTop - 0.2, r: R + 0.06 }, { x: ringTop, r: R + 0.02 }], { segments: 96, colour: () => [1, 1, 1] }),
      new THREE.MeshStandardMaterial({ map: ringTex, metalness: 0.8, roughness: 0.5, vertexColors: true }));
    ring.geometry.attributes.uv.array.forEach((v, i, a) => { if (i % 2 === 1) a[i] = (a[i] * 1.8 - (ringTop - ringH)) / ringH; });   // v from 0 at the bottom of the ring to 1 at the top
    ring.geometry.attributes.uv.needsUpdate = true;
    g.add(ring);
    // the grid fins stowed against the skin, below the ring, round the vehicle at equal angles; two catch pins above them
    const fw = 2.2 * dress.finScale, fh = 3.4 * dress.finScale, fy = ringTop - ringH - dress.finDrop - fh / 2;
    for (let i = 0; i < dress.fins; i++) {
      const a = Math.PI / dress.fins + (i * TAU) / dress.fins;
      const holder = new THREE.Group(); holder.rotation.y = a;
      const f = gridFin(m, fw, fh, 0.55 * dress.finScale); f.position.set(0, fy, R + 0.32);                   // the fin's face is tangent to the skin
      holder.add(f); g.add(holder);
      // a short fairing at the hinge
      const fair = new THREE.Mesh(new THREE.BoxGeometry(1.1 * dress.finScale, 0.5, 0.45), m.steel); fair.position.set(0, fy - fh / 2 - 0.2, R + 0.2); holder.add(fair);
    }
    if (dress.pins) for (const a of [0, Math.PI]) {
      const holder = new THREE.Group(); holder.rotation.y = a + Math.PI / 2;
      const pin = new THREE.Mesh(new THREE.CylinderGeometry(0.28, 0.28, 1.5, 14), m.steel); pin.rotation.x = Math.PI / 2; pin.position.set(0, ringTop - ringH - 0.9, R + 0.55); holder.add(pin);
      g.add(holder);
    }
  }
  // ---- the ship
  if (ship) {
    const g = ship.group;
    const x0 = ship.spec.x_start_m, tubeEnd = (ship.spec.sections || []).filter((s) => s.kind === 'tube').reduce((a, s) => Math.max(a, s.x_start_m + s.length_m), x0 + 20);
    const bellyHalf = (200 * Math.PI) / 360;              // tiles cover 200 degrees of the tube, centred on the belly
    const belly = Math.PI / 2;                            // the belly faces model +Z (the body's y axis)
    const tileMat = tileMaterial();
    const tubeProf = outline((ship.spec.sections || []).filter((s) => s.kind !== 'nose'));
    const noseProf = outline((ship.spec.sections || []).filter((s) => s.kind === 'nose'));
    const grow = (p, d) => p.map((q) => ({ x: q.x, r: q.r + d }));
    if (tubeProf.length) g.add(new THREE.Mesh(lathe(grow(tubeProf, 0.035), { segments: 40, phiStart: belly - bellyHalf, phiLength: 2 * bellyHalf, ring: 2.6, colour: () => [1, 1, 1], uRepeat: 0.33 }), tileMat));
    if (noseProf.length) g.add(new THREE.Mesh(lathe(grow(noseProf, 0.035), { segments: 56, phiStart: belly - Math.PI * 0.72, phiLength: Math.PI * 1.44, ring: 2.6, colour: () => [1, 1, 1], uRepeat: 0.33 }), tileMat));
    // the flaps: forward pair just under the nose, aft pair at the tail, on the sides of the belly. Each hangs on a hinge along its edge nearer the belly and stands `flapAngle` degrees out of the skin; a fairing covers the hinge.
    // Stowed flat (0) they are dark plates on a dark skin and cannot be seen, so the default is 30 degrees, an illustration and not the vehicle's state in the climb.
    const ang = (THREE.MathUtils.clamp(dress.flapAngle ?? 30, 0, 90) * Math.PI) / 180;
    const place = (x, w, h, a, side) => {
      const holder = new THREE.Group(); holder.rotation.y = a;
      const pivot = new THREE.Group(); pivot.position.set(side * (w / 2), x, R + 0.3); holder.add(pivot);       // the hinge line: along the body, on the belly-side edge of the flap
      const f = flap(m, w, h); f.position.set(-side * (w / 2), 0, 0); f.userData.tile = true; pivot.add(f);
      pivot.rotation.y = side * ang;                                                                                // out of the skin, on the leeward side
      const fair = new THREE.Mesh(new THREE.CylinderGeometry(0.34, 0.34, h * 0.96, 10), m.dark); fair.position.set(0, h / 2, 0); pivot.add(fair);
      g.add(holder); return f;
    };
    const fwdX = tubeEnd - 6.2, aftX = x0 + 1.4;
    for (const s of [-1, 1]) { place(fwdX, 3.0 * dress.flapScale, 6.0 * dress.flapScale, belly + s * (Math.PI * 0.46), s); place(aftX, 4.6 * dress.flapScale, 9.0 * dress.flapScale, belly + s * (Math.PI * 0.46), s); }
    // the base: the engine skirt of the ship
    const sk = new THREE.Mesh(new THREE.CircleGeometry(R * 0.97, 48), m.plate); sk.geometry.rotateX(-Math.PI / 2); sk.position.y = x0 + 1.6; g.add(sk);
  }
}

let _tile = null;
function tileMaterial() {
  if (_tile) return _tile;
  const t = tileTextures();
  return (_tile = new THREE.MeshStandardMaterial({ color: 0x777777, map: t.map, bumpMap: t.bump, bumpScale: 2.2, metalness: 0.05, roughness: 0.52 }));
}
