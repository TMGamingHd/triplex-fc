// Loads: what the flight does to the structure. The dynamic pressure and the angle of attack and their product (the load that the aerodynamics put across the vehicle), the acceleration along and across it, and the
// force, the shear and the bending moment at every station from the tail to the nose.
//
// How the internal loads are made. The simulator gives the force and the moment on the whole vehicle (thrust, aerodynamics) and where its mass is; this lens spreads the mass along the vehicle (the dry structure uniformly
// over each stage, the propellant in each tank at the level the simulator has it, the payloads where they are), lets every piece accelerate as a rigid body would (the vehicle's acceleration plus its angular acceleration
// times the distance from the centre of mass), puts the aerodynamic normal force at the centre of pressure and the axial drag along the length, and cuts the vehicle at each station: what the aft part must carry to
// make the forward part do what it does. A lumped-mass estimate, not a structural analysis: it shows where the loads are and how they change, not what a margin is.
import * as THREE from '../../vendor/three.module.js';
import { Chart, legend } from '../../../js/charts.js';
import { h, stat, statGrid, card, seg, toggle, provenance, fmt, store } from '../ui.js';
import { Label } from '../gfx.js';
import { G0 } from '../physics.js';

const N = 160;

/** The mass at N stations along the vehicle: [{x, m}], from the spec and the pose. */
export function massModel(spec, pose, len) {
  const dx = len / N, m = new Float64Array(N), stg = pose.stg ?? 255, pay = pose.pay ?? 255;
  const add = (x0, x1, kg) => { if (!(kg > 0)) return; const i0 = Math.max(0, Math.floor(x0 / dx)), i1 = Math.min(N - 1, Math.max(i0, Math.floor((x1 - 1e-9) / dx))); const n = i1 - i0 + 1; for (let i = i0; i <= i1; i++) m[i] += kg / n; };
  spec.stages.forEach((st, i) => { if ((stg >> i) & 1) add(st.x_start_m || 0, (st.x_start_m || 0) + st.length_m, st.dry_mass_kg); });
  spec.payloads.forEach((p, i) => { if ((pay >> i) & 1) add(p.x_m - 0.01, p.x_m + 0.01, p.mass_kg); });
  spec.tanks.forEach((t, k) => { if (!((stg >> t.stage) & 1)) return; const kg = (pose.tank || [])[k] || 0, hgt = kg / (t.density_kg_m3 * Math.PI * t.radius_m * t.radius_m); add(t.x_bottom_m, t.x_bottom_m + Math.max(hgt, dx * 0.5), kg); });
  return { dx, m, x: Array.from({ length: N }, (_, i) => (i + 0.5) * dx) };
}

/** The loads at each station: axial compression (N), shear (N), bending moment (N m). */
export function loadsAlong(spec, pose, len) {
  const { dx, m, x } = massModel(spec, pose, len), fth = pose.fth || [0, 0, 0], fae = pose.fae || [0, 0, 0], mth = pose.mth || [0, 0, 0], mae = pose.mae || [0, 0, 0];
  const mTot = m.reduce((a, b) => a + b, 0) || 1, cg = pose.cg;
  let I = 0; for (let i = 0; i < N; i++) I += m[i] * (x[i] - cg) * (x[i] - cg); I = Math.max(I, 1);
  // the specific force: the accelerations that the engines and the air give (gravity pulls every piece alike and loads nothing)
  const ax = (fth[0] + fae[0]) / mTot, ay = (fth[1] + fae[1]) / mTot, az = (fth[2] + fae[2]) / mTot;
  const aY = (mth[1] + mae[1]) / I, aZ = (mth[2] + mae[2]) / I;                    // the angular accelerations about body y and z
  // the external loads that sit on the stations: the thrust at the tail, the lateral aerodynamic force at the centre of pressure, the drag along the length
  const iCp = Math.min(N - 1, Math.max(0, Math.floor(pose.cp / dx)));
  const Fx = new Float64Array(N), Fy = new Float64Array(N), Fz = new Float64Array(N);
  Fx[0] += fth[0]; Fy[0] += fth[1]; Fz[0] += fth[2];
  Fy[iCp] += fae[1]; Fz[iCp] += fae[2];
  for (let i = 0; i < N; i++) Fx[i] += fae[0] / N;
  // from the nose down: the net force each station's mass needs, minus what is applied to it, is what the structure behind it must carry
  const axial = new Float64Array(N), shear = new Float64Array(N), moment = new Float64Array(N);
  let sx = 0, sy = 0, sz = 0, sxy = 0, sxz = 0;
  for (let i = N - 1; i >= 0; i--) {
    const r = x[i] - cg;
    const nx = m[i] * ax - Fx[i], ny = m[i] * (ay + aZ * r) - Fy[i], nz = m[i] * (az - aY * r) - Fz[i];
    sx += nx; sy += ny; sz += nz; sxy += ny * x[i]; sxz += nz * x[i];
    const xs = i * dx;                                                           // the section at the aft face of station i: the moment of what is ahead of it about that section
    axial[i] = sx; shear[i] = Math.hypot(sy, sz); moment[i] = Math.hypot(sxy - xs * sy, sxz - xs * sz);
  }
  return { x, dx, axial, shear, moment, mTot, ax, ay, az, aY, aZ, I };
}

export default {
  id: 'loads', label: 'Loads', key: '5',
  mount(app) {
    const { world, spec } = app, S = (this.s = { which: store.get('loads.which', 'moment') });
    const k = (l, u, hint) => stat(l, u, hint);
    S.st = { q: k('Dynamic pressure', 'kPa'), alpha: k('Angle of attack', '°'), qa: k('q · α', 'kPa·°', 'The dynamic pressure times the angle of attack: the measure of how hard the air pushes across the vehicle'), gx: k('Axial accel.', 'g'), gl: k('Lateral accel.', 'g'),
      aa: k('Angular accel.', '°/s²', 'About the centre of mass, from the total moment and the mass distribution'), N: k('Aero normal force', 'kN'), TL: k('Thrust, lateral part', 'kN', 'The part of the thrust across the axis: the engines gimballed'),
      cmax: k('Peak compression', 'MN'), cmaxAt: k('… at', 'm'), smax: k('Peak shear', 'MN'), mmax: k('Peak bending', 'MN·m'), mmaxAt: k('… at', 'm') };
    const s = S.st;
    S.canvas = h('canvas', { class: 'v-profile', width: 320, height: 320 });
    S.which = seg([['axial', 'Compression'], ['shear', 'Shear'], ['moment', 'Bending']], S.which, (v) => { S.which = v; store.set('loads.which', v); }, 'load');
    app.left.replaceChildren(card('The flight\'s load on the vehicle', statGrid(3, s.q, s.alpha, s.qa, s.gx, s.gl, s.aa, s.N, s.TL)), card('Along the vehicle', [statGrid(3, s.cmax, s.smax, s.mmax, s.cmaxAt, h('div'), s.mmaxAt)]),
      card('Loads at each station', [h('div', { class: 'v-row' }, S.which), S.canvas, h('p', { class: 'note' }, 'The vehicle is drawn along the left edge (nose at the top); the curve is the load at that station. The shaded part is the stage that is still on.')]));
    S.charts = { qa: new Chart(h('canvas'), { height: 110, window: 150, series: [{ key: 'qalpha', label: 'q·α  Pa·rad', color: 'var(--warn)' }] }), gx: new Chart(h('canvas'), { height: 110, window: 150, series: [{ key: 'gx', label: 'axial g', color: 'var(--ok)' }, { key: 'glat', label: 'lateral g', color: 'var(--nC)' }] }),
      q: new Chart(h('canvas'), { height: 110, window: 150, series: [{ key: 'q', label: 'q Pa', color: 'var(--info)' }] }) };
    let chN = 0; const ch = (t, c) => card(t, [c.c, legend(c.o.series)], null, { open: chN++ === 0 });
    // the diagram beside the vehicle in 3D: the chosen load plotted across the vehicle, in the plane of the lateral force
    const g = S.group = new THREE.Group(); world.modelHolder.add(g);
    S.poly = new THREE.Mesh(new THREE.BufferGeometry(), new THREE.MeshBasicMaterial({ color: 0xffb454, transparent: true, opacity: 0.35, side: THREE.DoubleSide, depthTest: false, depthWrite: false, toneMapped: false })); S.poly.renderOrder = 70; S.poly.frustumCulled = false;
    S.edge = new THREE.Line(new THREE.BufferGeometry(), new THREE.LineBasicMaterial({ color: 0xffb454, depthTest: false, toneMapped: false })); S.edge.renderOrder = 71; S.edge.frustumCulled = false;
    S.peakL = new Label({ text: '', color: '#ffb454', border: '#ffb454', size: 0.026 }); g.add(S.poly, S.edge, S.peakL.object);
    app.right.replaceChildren(card('Look', [toggle('The load beside the vehicle', true, (v) => { g.visible = v; }), h('p', { class: 'note' }, 'Drawn beside the vehicle, in the plane the lateral load acts in; the camera can go round it.')]), ch('q · α', S.charts.qa), ch('Acceleration', S.charts.gx), ch('Dynamic pressure', S.charts.q),
      card('Where the numbers come from', [provenance('measured', 'the dynamic pressure, the angle of attack, the force and the moment on the vehicle, the masses and where they are, the propellant in each tank.'),
        provenance('derived', 'the loads along the vehicle: a lumped-mass estimate (the vehicle cut at each station) from those. Not a structural analysis, and no allowable is shown: the project has none.')]));
    world.rig.set('orbit'); world.rig.yaw = Math.PI * 0.5; world.rig.pitch = 0.05; world.rig.dist = 1.5; world.rig.anchor = 0.5;
  },
  update(app, pose, d, dt) {
    const S = this.s, s = S.st, spec = app.spec, len = spec.length;
    const L = loadsAlong(spec, pose, len);
    const argmax = (a) => { let k = 0; for (let i = 1; i < a.length; i++) if (a[i] > a[k]) k = i; return k; };
    const kc = argmax(L.axial), ks = argmax(L.shear), km = argmax(L.moment);
    s.q.set((d.q / 1000).toFixed(1)); s.alpha.set(d.alpha.toFixed(2)); s.qa.set((d.q * d.alpha / 1000).toFixed(1), d.q * d.alpha > 1e5 ? 'warn' : ''); s.gx.set(d.gx.toFixed(2)); s.gl.set(d.glat.toFixed(3)); s.aa.set((Math.hypot(L.aY, L.aZ) * 180 / Math.PI).toFixed(3));
    s.N.set((d.N / 1000).toFixed(0)); s.TL.set((Math.hypot(pose.fth[1], pose.fth[2]) / 1000).toFixed(0));
    s.cmax.set((L.axial[kc] / 1e6).toFixed(1)); s.cmaxAt.set(L.x[kc].toFixed(0)); s.smax.set((L.shear[ks] / 1e6).toFixed(2)); s.mmax.set((L.moment[km] / 1e6).toFixed(0)); s.mmaxAt.set(L.x[km].toFixed(0));
    this.draw(app, pose, L);
    // 3D
    const arr = S.which === 'axial' ? L.axial : S.which === 'shear' ? L.shear : L.moment, peak = Math.max(...arr, 1), scale = (spec.diameter * 2.4) / peak;
    const R = spec.diameter / 2, pts = [], idx = [], pos = [];
    for (let i = 0; i < N; i++) { const y = L.x[i] - L.dx / 2, off = R * 1.35 + arr[i] * scale; pos.push(0, y, R * 1.35, 0, y, off); pts.push(new THREE.Vector3(0, y, off)); }
    for (let i = 0; i < N - 1; i++) { const a = 2 * i; idx.push(a, a + 1, a + 2, a + 1, a + 3, a + 2); }
    S.poly.geometry.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3)); S.poly.geometry.setIndex(idx); S.poly.geometry.computeBoundingSphere();
    S.edge.geometry.setFromPoints(pts);
    const kk = argmax(arr);
    S.peakL.setText(`${S.which === 'axial' ? 'compression' : S.which === 'shear' ? 'shear' : 'bending'} peak ${S.which === 'moment' ? (peak / 1e6).toFixed(0) + ' MN·m' : (peak / 1e6).toFixed(1) + ' MN'} at ${L.x[kk].toFixed(0)} m`);
    S.peakL.object.position.set(0, L.x[kk], R * 1.35 + arr[kk] * scale + R * 0.2); S.peakL.setAnchor(0, 0.5);
    for (const c of Object.values(S.charts)) c.draw(app.data.hist);
  },
  draw(app, pose, L) {
    const S = this.s, c = S.canvas, spec = app.spec, ctx = c.getContext('2d');
    const dpr = window.devicePixelRatio || 1, W = c.clientWidth || 320, H = 320;
    if (c.width !== Math.round(W * dpr)) { c.width = Math.round(W * dpr); c.height = Math.round(H * dpr); }
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0); ctx.clearRect(0, 0, W, H);
    const css = (v) => getComputedStyle(document.documentElement).getPropertyValue(v).trim();
    const arr = S.which === 'axial' ? L.axial : S.which === 'shear' ? L.shear : L.moment, peak = Math.max(...arr, 1);
    const len = spec.length, top = 8, bot = 22, Y = (x) => H - bot - (x / len) * (H - top - bot), X0 = 36, XW = W - X0 - 12;
    // the stages that are on, as a strip
    ctx.fillStyle = css('--panel3'); spec.stages.forEach((st, i) => { if (((pose.stg ?? 255) >> i) & 1) ctx.fillRect(10, Y(st.x_start_m + st.length_m), 14, Y(st.x_start_m) - Y(st.x_start_m + st.length_m)); });
    ctx.strokeStyle = css('--line'); ctx.fillStyle = css('--muted'); ctx.font = '10px system-ui'; ctx.textAlign = 'right';
    for (let k = 0; k <= 4; k++) { const x = len * k / 4; ctx.beginPath(); ctx.moveTo(X0, Y(x)); ctx.lineTo(X0 + XW, Y(x)); ctx.stroke(); ctx.fillText(x.toFixed(0) + ' m', X0 - 3, Y(x) + 3); }
    ctx.strokeStyle = css('--warn'); ctx.fillStyle = css('--warn') + '33'; ctx.lineWidth = 2; ctx.beginPath(); ctx.moveTo(X0, Y(0));
    for (let i = 0; i < N; i++) ctx.lineTo(X0 + XW * arr[i] / peak, Y(L.x[i]));
    ctx.lineTo(X0, Y(len)); ctx.stroke(); ctx.globalAlpha = 0.25; ctx.fillStyle = css('--warn'); ctx.fill(); ctx.globalAlpha = 1;
    ctx.fillStyle = css('--muted'); ctx.textAlign = 'center'; ctx.fillText((S.which === 'moment' ? (peak / 1e6).toFixed(0) + ' MN·m' : (peak / 1e6).toFixed(1) + ' MN') + ' at the right edge', X0 + XW / 2, H - 6);
    // the centre of mass and the centre of pressure
    ctx.lineWidth = 1.5; ctx.strokeStyle = css('--ok'); ctx.beginPath(); ctx.moveTo(X0, Y(pose.cg)); ctx.lineTo(X0 + XW, Y(pose.cg)); ctx.setLineDash([3, 3]); ctx.stroke(); ctx.strokeStyle = css('--accent'); ctx.beginPath(); ctx.moveTo(X0, Y(pose.cp)); ctx.lineTo(X0 + XW, Y(pose.cp)); ctx.stroke(); ctx.setLineDash([]);
    ctx.textAlign = 'left'; ctx.fillStyle = css('--ok'); ctx.fillText('CG', X0 + 3, Y(pose.cg) - 3); ctx.fillStyle = css('--accent'); ctx.fillText('CP', X0 + 3, Y(pose.cp) - 3);
  },
  unmount(app) { const S = this.s; if (!S) return; S.group.removeFromParent(); this.s = null; },
};
