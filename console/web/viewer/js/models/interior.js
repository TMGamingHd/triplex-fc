// The inside of a vehicle, for the cutaway ("Inside" on the camera bar): what the hull hides, drawn from the vehicle file and moved by the simulator's state.
//   * every tank of the file as a stainless shell with domed ends, and the liquid in it at the level the simulator has it (`pose.tank`, kg per tank: the oxidiser pale blue, the fuel amber), so that the booster
//     is seen emptying as it burns and the stage that returns is seen with what it kept for the landing;
//   * the thrust structure under the tanks (a cone of ribs from the engine plate up to the tank bottom) and the downcomer that feeds the engines;
//   * for a stage that has a nose (the ship): two header tanks and the payload, a stack of flat satellites, which leaves when the pose says the payload has gone (`pose.pay`).
// The shapes are the file's (a tank's place and size, the diameters); the ribs, the downcomer, the header tanks and the satellites are drawn from knowledge of the class, not from drawings, and say so
// (docs/design/VIEWER.md). The parts are children of their stage's group, so a stage that leaves takes its inside with it.
import * as THREE from '../../vendor/three.module.js';
import { lathe } from './common.js';

// (made again for every model: the cutaway gives each model's materials a clipping plane of that model's own)
function materials() {
  const shell = new THREE.MeshStandardMaterial({ color: 0xb9bfc6, metalness: 0.9, roughness: 0.32, side: THREE.DoubleSide });
  const frame = new THREE.MeshStandardMaterial({ color: 0x6d737a, metalness: 0.85, roughness: 0.45, side: THREE.DoubleSide });
  const pipe = new THREE.MeshStandardMaterial({ color: 0xcfd4d9, metalness: 0.95, roughness: 0.25, side: THREE.DoubleSide });
  const lox = new THREE.MeshStandardMaterial({ color: 0x5fc0f2, metalness: 0.0, roughness: 0.18, emissive: 0x0c2a3c, side: THREE.DoubleSide });
  const ch4 = new THREE.MeshStandardMaterial({ color: 0xe9a33a, metalness: 0.0, roughness: 0.2, emissive: 0x3a2105, side: THREE.DoubleSide });
  const sat = new THREE.MeshStandardMaterial({ color: 0x2c3f5e, metalness: 0.6, roughness: 0.35, side: THREE.DoubleSide });
  const satEdge = new THREE.MeshStandardMaterial({ color: 0xd8dde3, metalness: 0.8, roughness: 0.4 });
  return { shell, frame, pipe, lox, ch4, sat, satEdge };
}

/** A tank's outline: a cylinder `len` long from `x0`, radius `r`, closed by an elliptical dome at each end `dome` high. */
function tankProfile(x0, len, r, dome, n = 10) {
  const p = [];
  for (let i = 0; i <= n; i++) { const a = -Math.PI / 2 + (i / n) * (Math.PI / 2); p.push({ x: x0 + dome * Math.sin(a), r: Math.max(0, r * Math.cos(a)) }); }
  for (let i = 0; i <= n; i++) { const a = (i / n) * (Math.PI / 2); p.push({ x: x0 + len + dome * Math.sin(a), r: Math.max(0, r * Math.cos(a)) }); }
  return p;
}

export function buildInterior(model) {
  const spec = model.spec, M = materials();
  const parts = [], tanks = [];
  const add = (stage, obj) => { const g = model.stages[stage] && model.stages[stage].group; if (!g) return; obj.visible = false; g.add(obj); parts.push(obj); return obj; };

  spec.tanks.forEach((t, i) => {
    const area = Math.PI * t.radius_m * t.radius_m, full = t.propellant_kg / (t.density_kg_m3 * area), ox = t.density_kg_m3 >= 1000;
    const dome = t.radius_m * 0.28;
    const shell = add(t.stage, new THREE.Mesh(lathe(tankProfile(t.x_bottom_m, full, t.radius_m, dome), { segments: 40, colour: () => [1, 1, 1], ring: 100 }), M.shell));
    const liquidMat = ox ? M.lox : M.ch4;
    const cyl = new THREE.CylinderGeometry(t.radius_m * 0.975, t.radius_m * 0.975, 1, 40, 1); cyl.translate(0, 0.5, 0);      // from y = 0 to 1: scaled to the level, its foot at the bottom of the tank
    const liquid = add(t.stage, new THREE.Mesh(cyl, liquidMat)); liquid.position.y = t.x_bottom_m;
    const foot = add(t.stage, new THREE.Mesh(new THREE.SphereGeometry(t.radius_m * 0.975, 40, 12, 0, Math.PI * 2, Math.PI / 2, Math.PI / 2), liquidMat)); foot.scale.y = dome / (t.radius_m * 0.975); foot.position.y = t.x_bottom_m;   // the liquid in the lower dome
    tanks.push({ t, i, area, full, liquid, foot, shell });
  });

  // per stage: the thrust structure and the downcomer
  spec.stages.forEach((st, si) => {
    const R = (st.radius_m || 4.5), x0 = st.x_start_m || 0;
    const plate = x0 + (si === 0 ? 2.7 : 1.6), top = plate + Math.min(7, (st.length_m || 40) * 0.12);
    const cone = add(si, new THREE.Mesh(lathe([{ x: plate, r: R * 0.97 }, { x: top, r: R * 0.5 }], { segments: 36, colour: () => [1, 1, 1], ring: 100 }), M.frame));
    cone.name = 'thrust structure';
    for (let k = 0; k < 18; k++) {                                                 // the ribs of the thrust structure
      const a = (k / 18) * Math.PI * 2, rib = new THREE.Mesh(new THREE.BoxGeometry(0.16, top - plate, 0.5), M.frame);
      const rm = R * 0.735; rib.position.set(Math.sin(a) * rm, (plate + top) / 2, Math.cos(a) * rm);
      rib.rotation.set(0, a, 0); const tilt = Math.atan2(R * 0.47, top - plate); rib.rotateX(-tilt); add(si, rib);
    }
    const mine = tanks.filter((q) => q.t.stage === si);
    if (mine.length) {                                                              // the downcomer: a pipe down the side from the top tank to the engines
      const upper = Math.max(...mine.map((q) => q.t.x_bottom_m + q.full));
      const pipe = new THREE.Mesh(new THREE.CylinderGeometry(R * 0.07, R * 0.07, upper - plate, 14), M.pipe); pipe.position.set(0, (upper + plate) / 2, -R * 0.78); add(si, pipe);
    }
    // a stage with a nose: header tanks and the payload
    const nose = (st.sections || []).find((s) => s.kind === 'nose');
    if (nose) {
      const x = nose.x_start_m;
      for (const [dx, rr] of [[-1, 1.5], [1, 1.5]]) {
        const h = new THREE.Mesh(new THREE.CapsuleGeometry(rr, 3.2, 8, 20), M.shell); h.position.set(dx * 1.7, x + 3.6, 0); add(si, h);
      }
    }
  });

  // the payload: a stack of flat satellites in the nose, one for every ~2 t (the file's payload mass over twenty), thin plates the width of the bay
  const ship = spec.stages.findIndex((s) => (s.sections || []).some((q) => q.kind === 'nose'));
  const payload = new THREE.Group(); payload.name = 'payload (inside)';
  if (ship >= 0 && spec.payloads.length) {
    const nose = spec.stages[ship].sections.find((s) => s.kind === 'nose'), n = 18;
    for (let k = 0; k < n; k++) {
      const y = nose.x_start_m + 5.4 + k * 0.28 + (spec.payloads[0].x_m ? 0 : 0);
      const slab = new THREE.Mesh(new THREE.BoxGeometry(3.4, 0.12, 2.2), M.sat); slab.position.set(0, y, 0); payload.add(slab);
      const panel = new THREE.Mesh(new THREE.BoxGeometry(3.42, 0.14, 0.12), M.satEdge); panel.position.set(0, y, 1.1); payload.add(panel);
    }
    add(ship, payload);
  }

  const clipMats = [M.shell, M.frame, M.pipe, M.lox, M.ch4, M.sat, M.satEdge];
  return {
    parts, tanks, payload: ship >= 0 ? payload : null, clipMats,
    setVisible(on) { for (const p of parts) p.visible = on; if (on) this._shown = true; },
    /** Per frame (only while the cutaway is on): the level of the liquid in every tank, and the payload while it is aboard. */
    update(pose) {
      for (const q of tanks) {
        const kg = (pose.tank || [])[q.i] || 0, level = Math.max(0, Math.min(q.full, kg / (q.t.density_kg_m3 * q.area)));
        q.liquid.scale.y = Math.max(1e-4, level); q.liquid.visible = level > 0.02; q.foot.visible = kg > 1;
      }
      if (payload) payload.visible = !!((pose.pay ?? 255) & 1);
    },
  };
}
