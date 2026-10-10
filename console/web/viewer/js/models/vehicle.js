// The vehicle as a model: built from the spec the simulator sent (so it is the vehicle that flies: its stages, its diameters, its engines where the engine list puts them) and dressed in a style.
// Model axes: +Y is the vehicle's long axis toward the nose (the simulator's body X), +Z the body Y, +X the body Z. The origin is the tail of the first stage; the world places it so that the centre of gravity is at the vehicle's origin.
import * as THREE from '../../vendor/three.module.js';
import { lathe, outline, engineMesh, weldTextures, tileTextures, bar } from './common.js';
import { decorateStarship } from './starship.js';
import { fit, distribute } from './gltf.js';

const Y = new THREE.Vector3(0, 1, 0);

/** The generated styles; the page adds one `gltf:NAME` for every model file it finds. */
export const STYLES = [
  { id: 'auto', label: 'Auto (from the vehicle file)' },
  { id: 'starship', label: 'Starship' },
  { id: 'plain', label: 'Plain white' },
];

/** Which style a vehicle gets when the page does not say: Starship for the vehicle of that name, the generic painted rocket otherwise. */
export function autoStyle(spec) { return /starship/i.test(spec.name) ? 'starship' : 'painted'; }

const cache = {};
function steelMaterials() {
  if (cache.steel) return cache.steel;
  const { bump, rough } = weldTextures();
  const steel = new THREE.MeshStandardMaterial({ color: 0xb4b9be, metalness: 1.0, roughness: 0.46, roughnessMap: rough, bumpMap: bump, bumpScale: 1.6, vertexColors: true });
  const tiles = tileTextures();
  const tile = new THREE.MeshStandardMaterial({ color: 0xffffff, map: tiles.map, bumpMap: tiles.bump, bumpScale: 2.4, metalness: 0.05, roughness: 0.55 });
  const dark = new THREE.MeshStandardMaterial({ color: 0x15171a, metalness: 0.7, roughness: 0.5, side: THREE.DoubleSide });
  const fin = new THREE.MeshStandardMaterial({ color: 0x24272b, metalness: 0.85, roughness: 0.45, side: THREE.DoubleSide });
  return (cache.steel = { steel, tile, dark, fin });
}
function paintMaterials() {
  if (cache.paint) return cache.paint;
  const mk = (c, m = 0.15, r = 0.45) => new THREE.MeshStandardMaterial({ color: c, metalness: m, roughness: r, vertexColors: true });
  return (cache.paint = { white: mk(0xe8eaec), grey: mk(0x8f959c, 0.3, 0.4), orange: mk(0xd9622b, 0.1, 0.5), black: mk(0x1c1d20, 0.3, 0.5), fin: new THREE.MeshStandardMaterial({ color: 0x30353b, metalness: 0.4, roughness: 0.5, side: THREE.DoubleSide }) });
}

const engCache = new Map();
function engineProto(exitArea, vacuum) {
  const key = exitArea.toFixed(3) + (vacuum ? 'v' : 's');
  if (!engCache.has(key)) engCache.set(key, engineMesh(exitArea, vacuum));
  return engCache.get(key);
}

export class VehicleModel {
  /** opts.gltf: {scene, axis, fitLength, roll, offset, scale} for a style `gltf:NAME` (the scene is loaded by the page: models/gltf.js). */
  constructor(spec, style = 'auto', opts = {}) {
    this.spec = spec; this.opts = opts;
    if (style.startsWith('gltf:') && !(opts.gltf && opts.gltf.scene)) style = 'auto';     // the file is not loaded (yet): the generated vehicle stands in
    this.style = style === 'auto' ? autoStyle(spec) : style;
    this.root = new THREE.Group(); this.root.name = 'vehicle';
    this.stages = []; this.payloads = []; this.engines = []; this.parts = {};
    this.build();
  }

  build() {
    if (this.style.startsWith('gltf:') && this.opts.gltf) return this.buildImported();
    const spec = this.spec;
    const steelStyle = this.style === 'starship';
    const mats = steelStyle ? steelMaterials() : paintMaterials();
    const colours = steelStyle ? null : [mats.white, mats.grey, mats.white, mats.orange];
    spec.stages.forEach((st, si) => {
      const g = new THREE.Group(); g.name = 'stage ' + (si + 1);
      const sections = st.sections && st.sections.length ? st.sections : [{ kind: 'tube', x_start_m: st.x_start_m || 0, length_m: st.length_m, d_aft_m: 2 * (st.radius_m || 0.9), d_fore_m: 2 * (st.radius_m || 0.9) }];
      const prof = outline(sections);
      const soot = steelStyle && si === 0 ? (x) => { const k = Math.min(1, 0.32 + 0.68 * Math.min(1, Math.max(0, (x - 1.0) / 14))); return [k, k * 0.97, k * 0.95]; } : () => [1, 1, 1];
      const body = new THREE.Mesh(lathe(prof, { segments: 72, colour: soot }), steelStyle ? mats.steel : colours[si % colours.length]);
      body.castShadow = true; g.add(body);
      this.stages.push({ group: g, spec: st, body, profile: prof });
      this.root.add(g);
    });
    // payloads that carry their own outline (a fairing): a body that leaves with them
    spec.payloads.forEach((pl, pi) => {
      const g = new THREE.Group(); g.name = 'payload ' + pl.name;
      if (pl.sections && pl.sections.length) {
        const m = steelStyle ? mats.steel : paintMaterials().white;
        g.add(new THREE.Mesh(lathe(outline(pl.sections), { segments: 64, colour: () => [1, 1, 1] }), m));
      }
      this.payloads.push({ group: g, spec: pl, index: pi }); this.root.add(g);
    });
    // fixed fins of the stages (stabilizers): flat trapezoids round the body
    spec.stages.forEach((st, si) => {
      for (const f of st.stabilizers || []) this.stages[si].group.add(this.fins(f, st, (steelStyle ? mats.fin : paintMaterials().fin)));
    });
    this.engineParts();
    if (this.style === 'starship') decorateStarship(this);
  }

  /** An imported picture in place of the generated body: the stages' groups are there (for the engines and for the separation), the outline of each is still the vehicle file's (the lenses use it), the picture is in them. */
  buildImported() {
    const spec = this.spec, g = this.opts.gltf;
    spec.stages.forEach((st, si) => {
      const grp = new THREE.Group(); grp.name = 'stage ' + (si + 1);
      const sections = st.sections && st.sections.length ? st.sections : [{ kind: 'tube', x_start_m: st.x_start_m || 0, length_m: st.length_m, d_aft_m: 2 * (st.radius_m || 0.9), d_fore_m: 2 * (st.radius_m || 0.9) }];
      this.stages.push({ group: grp, spec: st, body: null, profile: outline(sections) }); this.root.add(grp);
    });
    spec.payloads.forEach((pl, pi) => { const grp = new THREE.Group(); this.payloads.push({ group: grp, spec: pl, index: pi }); this.root.add(grp); });
    const fitted = fit(g.scene.clone(true), spec, g);
    distribute(fitted, this.stages.map((s) => s.group));
    this.fitted = fitted;
    this.engineParts(false);
  }

  fins(f, st, mat) {
    const grp = new THREE.Group();
    const r = (this.bodyRadiusAt(f.x_le_root_m)) || (st.radius_m || 0.9);
    const shape = new THREE.Shape();                   // root chord on the body, then the tip: (along the body, out from it)
    shape.moveTo(0, 0); shape.lineTo(-f.root_chord_m, 0); shape.lineTo(-f.sweep_m - f.tip_chord_m, f.span_m); shape.lineTo(-f.sweep_m, f.span_m); shape.closePath();
    const geo = new THREE.ExtrudeGeometry(shape, { depth: Math.max(f.thickness_m || 0.02, 0.02), bevelEnabled: false });
    geo.translate(0, 0, -(f.thickness_m || 0.02) / 2);
    for (let i = 0; i < f.count; i++) {
      const m = new THREE.Mesh(geo, mat);
      const a = (i / f.count) * Math.PI * 2;
      // the shape's x is along -body, its y outward: turn so x -> Y (up the body) with the sign flipped, y -> radial
      const holder = new THREE.Group(); holder.rotation.y = a;
      m.rotation.set(0, Math.PI / 2, 0); m.rotation.order = 'YXZ';
      const inner = new THREE.Group(); inner.add(m);
      // place: the fin's local (x, y, z) = (along -Y, out along +X, thickness Z)
      m.matrixAutoUpdate = false; m.matrix.makeBasis(new THREE.Vector3(0, -1, 0), new THREE.Vector3(1, 0, 0), new THREE.Vector3(0, 0, 1)); m.matrix.setPosition(r * 0.98, f.x_le_root_m, 0);
      holder.add(m); grp.add(holder);
    }
    return grp;
  }
  bodyRadiusAt(x) {
    for (const st of this.stages) { const p = st.profile; for (let i = 0; i < p.length - 1; i++) if (x >= p[i].x && x <= p[i + 1].x) { const f = (x - p[i].x) / ((p[i + 1].x - p[i].x) || 1); return p[i].r + (p[i + 1].r - p[i].r) * f; } }
    return 0;
  }
  radiusAt(x) { return this.bodyRadiusAt(Math.min(x, this.spec.length - 1e-3)) || 0; }

  engineParts(withMesh = true) {
    this.spec.engines.forEach((e, i) => {
      const vac = (e.exit_area_m2 || 0) > 3 || (e.isp_vac_s || 0) > 360;
      const proto = engineProto(e.exit_area_m2 || 0.12, vac);
      const holder = new THREE.Group(); holder.name = 'engine ' + i;
      const L = proto.userData.length * 0.9;
      const inner = withMesh ? proto.clone(true) : new THREE.Group();            // an imported picture has its own engines: the holder (where the plume and the gimbal are) is still the vehicle file's
      inner.position.y = -L;
      holder.add(inner);
      const p = e.position_m || [0, 0, 0];
      holder.position.set(p[2], p[0] + L, p[1]);       // the nozzle exit is at the position: the pivot (where the engine swings) is a bit up the bell
      holder.userData = { exitRadius: proto.userData.exitRadius, pivot: L };
      this.stages[e.stage || 0].group.add(holder);
      const dir = e.direction_m ? null : null;
      this.engines.push({ holder, inner, spec: e, index: i, stage: e.stage || 0, gimbal: e.gimbal !== false && !e.control || e.gimbal === true, exitRadius: proto.userData.exitRadius, pivot: L, vacuum: vac, exitPos: new THREE.Vector3(p[2], p[0], p[1]), plume: null, dirFixed: dir });
    });
  }

  /** Per frame: the parts that follow the state of the vehicle. */
  apply(pose) {
    const sp = this.spec;
    const stg = pose.stg ?? 255, pay = pose.pay ?? 255;
    this.stages.forEach((s, i) => { s.group.visible = !!(stg & (1 << i)); });
    this.payloads.forEach((p) => { p.group.visible = !!(pay & (1 << p.index)); });
    const gim = pose.gim || [];
    const q = new THREE.Quaternion(), d = new THREE.Vector3();
    for (const en of this.engines) {
      const gp = en.gimbal ? (gim[2 * en.stage] || 0) : 0, gy = en.gimbal ? (gim[2 * en.stage + 1] || 0) : 0;
      const cp = (en.spec.cant_pitch_deg || 0) + gp, cy = (en.spec.cant_yaw_deg || 0) + gy;
      const a = cp * Math.PI / 180, b = cy * Math.PI / 180;
      d.set(Math.sin(b), Math.cos(a) * Math.cos(b), -Math.sin(a) * Math.cos(b));   // the thrust direction (the simulator's (cos dp cos dy, -sin dp cos dy, sin dy) in the model's axes)
      en.holder.quaternion.copy(q.setFromUnitVectors(Y, d.normalize()));
    }
  }
}
