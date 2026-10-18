// The gyro: the vehicle's attitude as a three-gimbal gyroscope, and its turn rates as three gauges.
//
// The attitude is turned into the three angles a gimbal set carries (`gimbalAngles`): the outer ring turns about the local vertical (the direction the vehicle leans toward), the middle ring about the horizontal axis
// it is hinged on (how far it leans: the tilt), and the inner ring about the vehicle's own axis (the roll). The vehicle sits in the inner ring, so the rings and the vehicle show the attitude the simulator reports and no other:
// q_body = R_vertical(lean) * R_horizontal(tilt) * R_axis(roll). It is the same rotation, split the way a gyroscope's frames split it.
//
// Why this split and not heading, pitch and roll: a launcher stands on its tail, where heading and roll are the same axis (gimbal lock: the angles of an aircraft's attitude are not defined there), while the lean, the tilt and the roll
// about the axis are. At the pad the tilt is zero and the lean is not defined: the last lean is kept, so that nothing jumps when the vehicle starts to tilt.
//
// The instrument is drawn into the corner of the same canvas, after the world (a scissor and a viewport of its own), so it is in a screenshot and costs one more draw of about nine thousand triangles (three rings of 1,920, the vehicle's outline and a few small parts).
import * as THREE from '../vendor/three.module.js';

const RAD = Math.PI / 180, DEG = 180 / Math.PI;
const Y = new THREE.Vector3(0, 1, 0), X = new THREE.Vector3(1, 0, 0);
const wrap = (a) => { a = (a + Math.PI) % (2 * Math.PI); if (a < 0) a += 2 * Math.PI; return a - Math.PI; };

/** The local frame at the vehicle in the scene's axes: x across the range, y up (away from the planet's centre), z downrange; the launch frame carried along by the planet's turn and the vehicle's travel. */
export function localFrame(posScene, pole, rotation, tt, out = new THREE.Quaternion()) {
  const padQ = new THREE.Quaternion().setFromAxisAngle(pole, rotation * tt);
  const upPad = new THREE.Vector3(0, 1, 0).applyQuaternion(padQ), up = new THREE.Vector3(posScene[0], posScene[1], posScene[2]).normalize();
  return out.setFromUnitVectors(upPad, up).multiply(padQ);
}

/** The vehicle's attitude (a quaternion in the local frame, the long axis along +Y) as the three angles of the gimbal set. `prevLean` is kept when the vehicle is vertical and the lean is not defined. Angles in radians:
 *  tilt 0..π from the vertical, lean (-π, π] from downrange toward the +x side, roll (-π, π] about the long axis. */
export function gimbalAngles(qRel, prevLean = 0) {
  const a = Y.clone().applyQuaternion(qRel);
  const tilt = Math.acos(Math.max(-1, Math.min(1, a.y)));
  const lean = Math.hypot(a.x, a.z) > 1e-4 ? Math.atan2(a.x, a.z) : prevLean;
  const qat = new THREE.Quaternion().setFromAxisAngle(Y, lean).multiply(new THREE.Quaternion().setFromAxisAngle(X, tilt));
  const qs = qat.invert().multiply(qRel);                                // what is left is a turn about the long axis
  const roll = wrap(2 * Math.atan2(qs.y, qs.w));
  return { tilt, lean, roll };
}

const ring = (radius, tube, colour) => new THREE.Mesh(new THREE.TorusGeometry(radius, tube, 10, 96), new THREE.MeshLambertMaterial({ color: colour, emissive: colour, emissiveIntensity: 0.28, toneMapped: false }));
const post = (from, to, colour) => {
  const d = to.clone().sub(from), m = new THREE.Mesh(new THREE.CylinderGeometry(0.018, 0.018, d.length(), 8), new THREE.MeshLambertMaterial({ color: colour, toneMapped: false }));
  m.position.copy(from).addScaledVector(d, 0.5); m.quaternion.setFromUnitVectors(Y, d.normalize()); return m;
};

export class GyroInstrument {
  constructor() {
    this.scene = new THREE.Scene();
    this.cam = new THREE.PerspectiveCamera(30, 1, 0.1, 40);
    this.cam.position.set(3.7, 2.0, 3.0); this.cam.lookAt(0, -0.05, 0);
    this.scene.add(new THREE.AmbientLight(0xffffff, 1.5)); const sun = new THREE.DirectionalLight(0xffffff, 2.2); sun.position.set(2, 4, 3); this.scene.add(sun);
    const green = 0x3ddc97, cyan = 0x4cc9f0, amber = 0xffb454;
    this.lean = new THREE.Group(); this.tilt = new THREE.Group(); this.roll = new THREE.Group();
    this.lean.add(this.tilt); this.tilt.add(this.roll); this.scene.add(this.lean);
    // the outer ring hinges on the vertical, the middle on the horizontal axis it carries, the inner on the vehicle's axis: each ring's plane holds its own axis and the next ring's
    const outer = ring(0.98, 0.022, green), middle = ring(0.80, 0.022, cyan), inner = ring(0.62, 0.022, amber); inner.rotation.y = Math.PI / 2;
    this.lean.add(outer); this.tilt.add(middle); this.roll.add(inner);
    // a tab on each ring, where its angle is read against the next frame
    const tab = (colour) => new THREE.Mesh(new THREE.SphereGeometry(0.055, 14, 10), new THREE.MeshLambertMaterial({ color: colour, emissive: colour, emissiveIntensity: 0.6, toneMapped: false }));
    const t1 = tab(green); t1.position.set(0.98, 0, 0); outer.add(t1);
    const t2 = tab(cyan); t2.position.set(0, 0.8, 0); middle.add(t2);
    const t3 = tab(amber); t3.position.set(0, 0, 0.62); this.roll.add(t3);
    // the posts that hold a ring in the next: the vertical on the frame, the horizontal axis between the outer and the middle, the long axis between the middle and the inner
    this.lean.add(post(new THREE.Vector3(0, -1.12, 0), new THREE.Vector3(0, -0.98, 0), 0x8696b0), post(new THREE.Vector3(0, 0.98, 0), new THREE.Vector3(0, 1.12, 0), 0x8696b0));
    this.tilt.add(post(new THREE.Vector3(-0.98, 0, 0), new THREE.Vector3(-0.8, 0, 0), 0x8696b0), post(new THREE.Vector3(0.8, 0, 0), new THREE.Vector3(0.98, 0, 0), 0x8696b0));
    this.roll.add(post(new THREE.Vector3(0, -0.8, 0), new THREE.Vector3(0, -0.62, 0), 0x8696b0), post(new THREE.Vector3(0, 0.62, 0), new THREE.Vector3(0, 0.8, 0), 0x8696b0));
    // the frame that does not move: the horizon, a mark downrange, and the vertical
    const hz = new THREE.Mesh(new THREE.RingGeometry(1.14, 1.16, 96), new THREE.MeshBasicMaterial({ color: 0x8696b0, transparent: true, opacity: 0.55, side: THREE.DoubleSide, toneMapped: false })); hz.rotation.x = -Math.PI / 2; this.scene.add(hz);
    const disc = new THREE.Mesh(new THREE.CircleGeometry(1.14, 64), new THREE.MeshBasicMaterial({ color: 0x8696b0, transparent: true, opacity: 0.07, side: THREE.DoubleSide, toneMapped: false, depthWrite: false })); disc.rotation.x = -Math.PI / 2; this.scene.add(disc);
    const down = new THREE.Mesh(new THREE.ConeGeometry(0.07, 0.2, 14), new THREE.MeshBasicMaterial({ color: 0xffffff, toneMapped: false })); down.rotation.x = Math.PI / 2; down.position.set(0, 0, 1.3); this.scene.add(down);
    this.rocket = null; this.mask = null; this.lastLean = 0; this.rect = null; this.visible = true;
    this.angles = { tilt: 0, lean: 0, roll: 0 };
  }

  /** The vehicle in the inner ring: the outline of what is still on it, a unit long, with a stripe along one side so that its roll can be seen. */
  setVehicle(profiles) {
    if (this.rocket) { this.roll.remove(this.rocket); this.rocket.traverse((o) => { if (o.geometry) o.geometry.dispose(); if (o.material) o.material.dispose(); }); this.rocket = null; }
    const pts = profiles.flat().filter((p) => Number.isFinite(p.x) && Number.isFinite(p.r)).sort((a, b) => a.x - b.x);
    if (pts.length < 2) return;
    const x0 = pts[0].x, x1 = pts[pts.length - 1].x, L = Math.max(x1 - x0, 1e-3), k = 1.0 / L;
    const g = new THREE.Group();
    const lathe = new THREE.Mesh(new THREE.LatheGeometry(pts.map((p) => new THREE.Vector2(Math.max(p.r * k, 1e-4), (p.x - x0) * k - 0.5)), 28), new THREE.MeshLambertMaterial({ color: 0xdbe3f1, emissive: 0x39445a, toneMapped: false }));
    g.add(lathe);
    const R = Math.max(...pts.map((p) => p.r)) * k;
    const stripe = new THREE.Mesh(new THREE.BoxGeometry(0.012, 0.9, 0.006), new THREE.MeshBasicMaterial({ color: 0xff5d73, toneMapped: false })); stripe.position.set(0, 0, R + 0.004); g.add(stripe);
    this.rocket = g; this.roll.add(g);
  }

  /** The attitude, as the quaternion of the vehicle in the local frame, and the stages that are on the vehicle (the outline is rebuilt when they change). */
  set(qRel, mask, profilesFor) {
    if (mask !== this.mask) { this.mask = mask; this.setVehicle(profilesFor(mask)); }
    const a = this.angles = gimbalAngles(qRel, this.lastLean);
    this.lastLean = a.lean;
    this.lean.rotation.y = a.lean; this.tilt.rotation.x = a.tilt; this.roll.rotation.y = a.roll;
    return a;
  }

  /** Draws into the rectangle of `el` (CSS pixels), over what is on the canvas; inside a dock that scrolls, only the part of it that the dock shows. */
  render(renderer, el) {
    if (!el || !this.visible) return;
    const r = el.getBoundingClientRect(), W = renderer.domElement.clientWidth, H = renderer.domElement.clientHeight;
    if (r.width < 8 || r.height < 8 || r.bottom < 0 || r.top > H) return;
    let c = { l: r.left, t: r.top, r: r.right, b: r.bottom };
    const dock = el.closest && el.closest('.v-dock');
    if (dock) { const d = dock.getBoundingClientRect(); c = { l: Math.max(c.l, d.left), t: Math.max(c.t, d.top), r: Math.min(c.r, d.right), b: Math.min(c.b, d.bottom) }; }
    if (c.r - c.l < 2 || c.b - c.t < 2) return;
    this.cam.aspect = r.width / r.height; this.cam.updateProjectionMatrix();
    const keep = renderer.toneMapping; renderer.toneMapping = THREE.NoToneMapping;
    renderer.setScissorTest(true); renderer.setViewport(r.left, H - r.bottom, r.width, r.height); renderer.setScissor(c.l, H - c.b, c.r - c.l, c.b - c.t);
    renderer.clearDepth(); renderer.render(this.scene, this.cam);
    renderer.setScissorTest(false); renderer.setViewport(0, 0, W, H);
    renderer.toneMapping = keep;
  }
}

/** The three rate gauges and the three angles, as the DOM of a card: returns {el, update(angles, w, level)}. `w` is the body's rate in rad/s (about the axis, then the two lateral ones, as the simulator gives it). */
export function gyroReadout(h) {
  const row = (label, unit) => {
    const bar = h('i'), val = h('b', {}, '0.0'), el = h('div', { class: 'gy-row' }, h('span', { class: 'gy-k' }, label), h('span', { class: 'gy-track' }, h('em', {}), bar), val, h('small', {}, unit));
    return { el, bar, val };
  };
  const roll = row('Roll', '°/s'), pitch = row('Pitch', '°/s'), yaw = row('Yaw', '°/s');
  const ang = h('div', { class: 'gy-ang' }, ['Tilt', 'Lean', 'Roll'].map((k) => h('div', {}, h('span', {}, k), h('b', {}, '0.0°'))));
  const el = h('div', { class: 'gy-read' }, ang, roll.el, pitch.el, yaw.el);
  const full = 20;                                                                          // °/s at the end of a gauge
  const set = (r, dps) => { const f = Math.max(-1, Math.min(1, dps / full)); r.bar.style.width = (Math.abs(f) * 50).toFixed(1) + '%'; r.bar.style.left = f < 0 ? (50 + f * 50).toFixed(1) + '%' : '50%'; r.val.textContent = dps.toFixed(1); };
  return {
    el,
    update(a, w) {
      const t = ang.children; const s = (i, v) => { const txt = v.toFixed(1) + '°'; if (t[i].lastChild.textContent !== txt) t[i].lastChild.textContent = txt; };
      s(0, a.tilt * DEG); s(1, a.lean * DEG); s(2, a.roll * DEG);
      set(roll, (w[0] || 0) * DEG); set(yaw, (w[1] || 0) * DEG); set(pitch, (w[2] || 0) * DEG);
    },
  };
}

/** The instrument in a box of the page, and what it needs each frame: made by a lens, which puts `el` where it wants it and calls `update(pose)` with the pose of the body that is followed. `full` adds the rate gauges and the three angles. */
export function createGyroBox(app, h, { full = false } = {}) {
  const world = app.world, inst = new GyroInstrument(), view = h('div', { class: 'gy-view' }), read = gyroReadout(h);
  const cap = h('div', { class: 'gy-cap' }, h('span', {}, h('small', {}, 'TILT'), h('b', {}, '0.0°')), h('span', {}, h('small', {}, 'ROLL'), h('b', {}, '0.0°')), h('span', {}, h('small', {}, 'RATE'), h('b', {}, '0.0°/s')));
  const el = h('div', { class: 'gy-box' + (full ? ' full' : '') }, view, full ? read.el : cap);
  const draw = (r) => { if (el.isConnected) inst.render(r, view); };
  world.insets.push(draw);
  const profiles = (mask) => app.vehicle ? app.vehicle.stages.filter((st, i) => (mask >> i) & 1).map((st) => st.profile || []) : [];
  const q = new (world.qs.constructor)();
  return {
    el, inst,
    update(pose) {
      q.copy(world.localQ).invert().multiply(world.qs);
      const a = inst.set(q, pose.stg ?? 255, profiles), w = pose.w || [0, 0, 0];
      if (full) read.update(a, w);
      else { const c = cap.children, t = [`${(a.tilt * DEG).toFixed(1)}°`, `${(a.roll * DEG).toFixed(1)}°`, `${(Math.hypot(w[0] || 0, w[1] || 0, w[2] || 0) * DEG).toFixed(1)}°/s`]; for (let i = 0; i < 3; i++) if (c[i].lastChild.textContent !== t[i]) c[i].lastChild.textContent = t[i]; }
      return a;
    },
    dispose() { const i = world.insets.indexOf(draw); if (i >= 0) world.insets.splice(i, 1); },
  };
}
