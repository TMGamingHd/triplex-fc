// Attitude and guidance: which way the vehicle points against which way the flight computers want it to, how fast it is turning, what the gimbals are doing about it, and what the computers and the voter say.
// A wire ghost of the vehicle shows the attitude the program asks for; the angle between it and the vehicle is the error the controller is working on.
import * as THREE from '../../vendor/three.module.js';
import { Chart, legend } from '../../../js/charts.js';
import { h, stat, statGrid, card, toggle, provenance, fmt, store } from '../ui.js';
import { Arrow, Label, line, setLine } from '../gfx.js';
import { quatToScene } from '../data.js';

const RAD = Math.PI / 180, DEG = 180 / Math.PI;
const ACT = ['Standby', 'Nominal', 'Safe'];

function wireGhost(vehicle, cg) {
  const g = new THREE.Group(), mat = new THREE.LineBasicMaterial({ color: 0x4cc9f0, transparent: true, opacity: 0.7, depthTest: false, toneMapped: false });
  const pts = [];
  for (const st of vehicle.stages) {
    const p = st.profile; if (!p.length) continue;
    const M = 12;
    for (let j = 0; j < M; j++) { const a = (j / M) * Math.PI * 2; for (let i = 0; i < p.length - 1; i++) pts.push(new THREE.Vector3(Math.cos(a) * p[i].r, p[i].x - cg, Math.sin(a) * p[i].r), new THREE.Vector3(Math.cos(a) * p[i + 1].r, p[i + 1].x - cg, Math.sin(a) * p[i + 1].r)); }
    const x0 = p[0].x, x1 = p[p.length - 1].x;
    for (let x = x0; x <= x1; x += Math.max(6, (x1 - x0) / 6)) { const r = vehicle.bodyRadiusAt(Math.min(x, x1 - 1e-3)) || p[0].r; for (let j = 0; j < 24; j++) { const a = (j / 24) * Math.PI * 2, b = ((j + 1) / 24) * Math.PI * 2; pts.push(new THREE.Vector3(Math.cos(a) * r, x - cg, Math.sin(a) * r), new THREE.Vector3(Math.cos(b) * r, x - cg, Math.sin(b) * r)); } }
  }
  const geo = new THREE.BufferGeometry().setFromPoints(pts);
  const l = new THREE.LineSegments(geo, mat); l.renderOrder = 80; l.frustumCulled = false; g.add(l);
  return g;
}

export default {
  id: 'attitude', label: 'Attitude', key: '6',
  mount(app) {
    const { world, vehicle, spec } = app, S = (this.s = { ghost: store.get('att.ghost', true), axes: store.get('att.axes', true) });
    const k = (l, u, hint) => stat(l, u, hint);
    S.st = { tp: k('Tilt, pitch', '°', 'The long axis from the vertical in the pitch plane (toward downrange)'), ty: k('Tilt, yaw', '°'), rp: k('Program, pitch', '°', 'What the flight computers\' guidance asks for'), ry: k('Program, yaw', '°'),
      ep: k('Error, pitch', '°'), ey: k('Error, yaw', '°'), err: k('Error, total', '°', 'The angle between the vehicle\'s axis and the program\'s'),
      w0: k('Roll rate', '°/s'), w1: k('Yaw rate', '°/s'), w2: k('Pitch rate', '°/s'), gp: k('Gimbal, pitch', '°', 'The first stage\'s engines, actual'), gy: k('Gimbal, yaw', '°'), cp: k('Command, pitch', '°', 'The voted command the actuator node sent'), cy: k('Command, yaw', '°'), act: k('Actuator node', ''), };
    const s = S.st;
    S.nodes = h('div', { class: 'v-nodes' });
    S.diagram = h('canvas', { class: 'v-diagram', width: 300, height: 240 });
    app.left.replaceChildren(card('Attitude against the program', statGrid(3, s.tp, s.rp, s.ep, s.ty, s.ry, s.ey)), card('Rates and gimbals', statGrid(3, s.w2, s.w1, s.w0, s.gp, s.gy, s.err, s.cp, s.cy, s.act)),
      card('The pitch plane', [S.diagram, h('p', { class: 'note' }, 'Dashed: the local vertical. The vehicle\'s axis, the program, the velocity and the thrust, in the plane that contains the long axis and the vertical.')]));
    S.charts = {
      err: new Chart(h('canvas'), { height: 110, window: 120, series: [{ key: 'err_p', label: 'pitch error °', color: 'var(--nA)' }, { key: 'err_y', label: 'yaw error °', color: 'var(--nC)' }], zero: true }),
      tilt: new Chart(h('canvas'), { height: 110, window: 120, series: [{ key: 'tilt_p', label: 'pitch tilt °', color: 'var(--info)' }, { key: 'ref_p', label: 'program °', color: 'var(--faint)', dash: [5, 4] }] }),
      gim: new Chart(h('canvas'), { height: 110, window: 120, series: [{ key: 'gim_p', label: 'gimbal pitch °', color: 'var(--accent)' }, { key: 'gim_y', label: 'gimbal yaw °', color: 'var(--nSIM)' }], zero: true }),
    };
    const ch = (t, c) => card(t, [c.c, legend(c.o.series)]);
    app.right.replaceChildren(card('Show', [toggle('The attitude the program asks for (wire ghost)', S.ghost, (v) => { S.ghost = v; store.set('att.ghost', v); }), toggle('Body axes and the velocity', S.axes, (v) => { S.axes = v; store.set('att.axes', v); })]),
      card('The computers', [S.nodes, h('p', { class: 'note' }, 'From the console\'s own state: shown when the rig, or a replay of it, is running.')]), ch('Attitude error', S.charts.err), ch('Tilt against the program', S.charts.tilt), ch('Gimbal', S.charts.gim),
      card('Where the numbers come from', [provenance('measured', 'the attitude, the rates, the gimbal angles, the program and the actuator node\'s command: the simulator\'s state and the flight computers\' own output.'), provenance('illustrative', 'the wire ghost: the vehicle turned by the error, about the centre of mass.')]));
    // 3D: the triad, the velocity, the ghost
    const g = S.group = new THREE.Group(); world.overlay.add(g);       // in the vehicle's frame, the origin at its centre of mass
    S.ax = [new Arrow(0xff5d73), new Arrow(0x3ddc97), new Arrow(0x6ea8ff)]; S.axL = ['X (nose)', 'Y', 'Z'].map((t, i) => new Label({ text: t, color: ['#ff5d73', '#3ddc97', '#6ea8ff'][i], size: 0.024 }));
    S.vel = new Arrow(0xffffff); S.velL = new Label({ text: 'VELOCITY', color: '#ffffff', size: 0.024 }); S.up = new Arrow(0x8696b0); S.upL = new Label({ text: 'UP', color: '#8696b0', size: 0.024 });
    S.rate = new Arrow(0xffd166); S.rateL = new Label({ text: '', color: '#ffd166', size: 0.024 });
    for (const a of [...S.ax, S.vel, S.up, S.rate]) g.add(a.object); for (const l of [...S.axL, S.velL, S.upL, S.rateL]) g.add(l.object);
    S.ghostObj = wireGhost(vehicle, app.pose ? app.pose.cg : spec.length * 0.45); g.add(S.ghostObj);
    S.ghostCg = app.pose ? app.pose.cg : spec.length * 0.45;
    S.errL = new Label({ text: '', color: '#4cc9f0', border: '#4cc9f0', size: 0.026 }); g.add(S.errL.object);
    world.rig.set('orbit'); world.rig.yaw = 0.9; world.rig.pitch = 0.2; world.rig.dist = 1.9; world.rig.anchor = 0.5;
  },
  update(app, pose, d, dt) {
    const S = this.s, s = S.st, spec = app.spec, L = spec.length, R = spec.diameter / 2;
    const tilt = pose.tilt || [0, 0], ref = pose.ref || [0, 0];
    const ep = tilt[0] - ref[0], ey = tilt[1] - ref[1];
    // the attitude error as an angle between two axes
    const q = new THREE.Quaternion(pose.q[1], pose.q[2], pose.q[3], pose.q[0]);
    const u = new THREE.Vector3(1, 0, 0).applyQuaternion(q);
    const uref = new THREE.Vector3(Math.cos(ref[1] * RAD) * Math.cos(ref[0] * RAD), Math.cos(ref[1] * RAD) * Math.sin(ref[0] * RAD), -Math.sin(ref[1] * RAD)).normalize();
    const err = Math.acos(Math.min(1, Math.max(-1, u.dot(uref)))) * DEG;
    s.tp.set(tilt[0].toFixed(2)); s.ty.set(tilt[1].toFixed(2)); s.rp.set(ref[0].toFixed(2)); s.ry.set(ref[1].toFixed(2)); s.ep.set(ep.toFixed(2), Math.abs(ep) > 3 ? 'warn' : ''); s.ey.set(ey.toFixed(2), Math.abs(ey) > 3 ? 'warn' : ''); s.err.set(err.toFixed(2), err > 5 ? 'warn' : '');
    const w = pose.w || [0, 0, 0];
    s.w0.set((w[0] * DEG).toFixed(2)); s.w1.set((w[1] * DEG).toFixed(2)); s.w2.set((w[2] * DEG).toFixed(2));
    const gim = pose.gim || [0, 0], cmd = pose.cmd || [0, 0];
    s.gp.set((gim[0] || 0).toFixed(2)); s.gy.set((gim[1] || 0).toFixed(2)); s.cp.set(cmd[0].toFixed(2)); s.cy.set(cmd[1].toFixed(2));
    const sn = app.data.snapshot, act = sn && sn.act;
    s.act.set(act && act.alive ? `${act.state_name}` : pose.act >= 0 && pose.act < 3 ? ACT[pose.act] : '—', act && act.state_name && act.state_name.startsWith('Safe') ? 'warn' : '');
    this.drawNodes(app, sn);
    this.drawDiagram(app, pose, tilt, ref);
    // 3D
    const ax = [new THREE.Vector3(0, 1, 0), new THREE.Vector3(0, 0, 1), new THREE.Vector3(1, 0, 0)], len = L * 0.22;
    S.ax.forEach((a, i) => { a.object.visible = S.axes; a.set(new THREE.Vector3(0, 0, 0), ax[i], len, Math.max(0.2, R * 0.05)); S.axL[i].object.visible = S.axes; S.axL[i].object.position.copy(ax[i]).multiplyScalar(len * 1.08); });
    const vq = q.clone().invert(), vsim = new THREE.Vector3(pose.v[0], pose.v[1], pose.v[2]).applyQuaternion(vq), upsim = new THREE.Vector3(pose.r[0], pose.r[1], pose.r[2]).normalize().applyQuaternion(vq);
    const toModel = (v) => new THREE.Vector3(v.z, v.x, v.y);
    const vm = toModel(vsim), um = toModel(upsim);
    S.vel.object.visible = S.velL.object.visible = S.axes && pose.spd > 5; S.up.object.visible = S.upL.object.visible = S.axes;
    S.vel.set(new THREE.Vector3(), vm, L * 0.34, Math.max(0.2, R * 0.05)); S.velL.object.position.copy(vm.clone().normalize()).multiplyScalar(L * 0.36); S.velL.setText(`VELOCITY ${pose.spd.toFixed(0)} m/s`);
    S.up.set(new THREE.Vector3(), um, L * 0.2, Math.max(0.15, R * 0.04)); S.upL.object.position.copy(um.clone().normalize()).multiplyScalar(L * 0.21);
    // the rate: the angular velocity vector, in the body frame
    const wm = new THREE.Vector3(w[2], w[0], w[1]), wmag = wm.length() * DEG;
    S.rate.object.visible = S.rateL.object.visible = wmag > 0.05 && S.axes;
    if (wmag > 0.05) { S.rate.set(new THREE.Vector3(R * 1.8, 0, 0), wm, Math.min(L * 0.3, L * 0.06 * Math.pow(wmag, 0.7)), Math.max(0.2, R * 0.04)); S.rateL.setText(`${wmag.toFixed(2)} °/s`); S.rateL.object.position.set(R * 1.8, 0, 0).addScaledVector(wm.clone().normalize(), L * 0.07 * Math.pow(wmag, 0.7) + 2); }
    // the ghost: the vehicle turned by the error
    S.ghostObj.visible = S.ghost && err > 0.05; S.errL.object.visible = S.ghost && err > 0.05;
    if (S.ghost && err > 0.01) {
      const dq = new THREE.Quaternion().setFromUnitVectors(u, uref), qref = dq.multiply(q), rel = q.clone().invert().multiply(qref);
      const sq = quatToScene([rel.w, rel.x, rel.y, rel.z]);
      S.ghostObj.quaternion.set(sq[0], sq[1], sq[2], sq[3]);
      S.ghostObj.position.set(0, 0, 0);
      if (Math.abs(S.ghostCg - pose.cg) > 0.5) { S.ghostObj.removeFromParent(); S.ghostObj = wireGhost(app.vehicle, pose.cg); S.group.add(S.ghostObj); S.ghostCg = pose.cg; }
      S.errL.setText(`error ${err.toFixed(2)}°`); S.errL.object.position.set(R * 1.3, spec.length - pose.cg, 0);
    }
    for (const c of Object.values(S.charts)) c.draw(app.data.hist);
  },
  drawNodes(app, sn) {
    const el = this.s.nodes;
    if (!sn || !sn.nodes) { if (el.dataset.empty !== '1') { el.dataset.empty = '1'; el.replaceChildren(h('p', { class: 'note' }, 'No flight computers are connected to the console: this flight came from the simulator or a pose file alone.')); } return; }
    el.dataset.empty = '';
    if (el.childElementCount !== 4) { el.replaceChildren(...[0, 1, 2].map(() => h('div', { class: 'v-node' })), h('div', { class: 'v-node act' })); }
    sn.nodes.forEach((n, i) => {
      const lvl = n.health === 'healthy' ? 'ok' : n.health === 'unknown' ? 'muted' : n.health === 'disabled' ? 'crit' : 'warn';
      const row = el.children[i]; const txt = `${n.name}   ${n.alive ? n.health : 'silent'}   ${n.mode_name || '—'}${n.master ? '   sync master' : ''}   cmd ${n.cmd ? n.cmd[0].toFixed(2) + ' / ' + n.cmd[1].toFixed(2) + '°' : '—'}`;
      if (row.textContent !== txt) row.textContent = txt; row.className = 'v-node ' + lvl;
    });
    const a = sn.act, row = el.children[3];
    const t = a ? `ACT   ${a.alive ? a.state_name : 'silent'}   ${a.vote_status}   voted ${a.voted.map((x, i) => (x ? 'ABC'[i] : '·')).join('')}   out ${a.pitch.toFixed(2)} / ${a.yaw.toFixed(2)}°` : 'ACT —';
    if (row.textContent !== t) row.textContent = t; row.className = 'v-node act ' + (a && a.alive && a.state_name === 'Nominal' ? 'ok' : 'warn');
  },
  /** The plane of the vehicle's axis and the vertical: dashed vertical, the axis, the program, the velocity and the thrust, as lines from a point. */
  drawDiagram(app, pose, tilt, ref) {
    const c = this.s.diagram, ctx = c.getContext('2d'), dpr = window.devicePixelRatio || 1, W = c.clientWidth || 300, H = 240;
    if (c.width !== Math.round(W * dpr)) { c.width = Math.round(W * dpr); c.height = Math.round(H * dpr); }
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0); ctx.clearRect(0, 0, W, H);
    const css = (v) => getComputedStyle(document.documentElement).getPropertyValue(v).trim(), cx = W / 2, cy = H - 28, Lr = H - 50;
    // angles from the local vertical in the pitch plane (positive toward downrange, to the right)
    const rv = Math.hypot(...pose.r), up = pose.r.map((x) => x / rv);
    const vhat = pose.spd > 1 ? pose.v.map((x) => x / pose.spd) : up;
    const velPitch = Math.atan2(vhat[1], vhat[0]) * DEG;                           // in the launch frame: the same measure as the tilt
    const ray = (deg, len, colour, label, dash) => {
      const a = deg * RAD, x = cx + Math.sin(a) * len, y = cy - Math.cos(a) * len;
      ctx.strokeStyle = colour; ctx.fillStyle = colour; ctx.lineWidth = 2; ctx.setLineDash(dash || []); ctx.beginPath(); ctx.moveTo(cx, cy); ctx.lineTo(x, y); ctx.stroke(); ctx.setLineDash([]);
      ctx.beginPath(); ctx.arc(x, y, 3.5, 0, 7); ctx.fill(); ctx.font = '600 11px system-ui'; ctx.textAlign = x > cx ? 'left' : 'right'; ctx.fillText(label, x + (x > cx ? 6 : -6), y + 4);
    };
    ctx.strokeStyle = css('--line2'); ctx.setLineDash([4, 4]); ctx.beginPath(); ctx.moveTo(cx, cy); ctx.lineTo(cx, cy - Lr); ctx.stroke(); ctx.setLineDash([]);
    ctx.fillStyle = css('--faint'); ctx.font = '10px system-ui'; ctx.textAlign = 'center'; ctx.fillText('vertical', cx, cy - Lr - 4); ctx.fillText('downrange →', cx + W * 0.28, cy + 16);
    ray(ref[0], Lr * 0.92, css('--faint'), `program ${ref[0].toFixed(1)}°`, [6, 4]);
    ray(velPitch, Lr * 0.62, css('--text'), `velocity ${velPitch.toFixed(1)}°`);
    ray(tilt[0], Lr, css('--accent'), `axis ${tilt[0].toFixed(1)}°`);
    const gp = (pose.gim || [0])[0] || 0;
    ray(tilt[0] - gp, Lr * 0.45, css('--crit'), 'thrust', [2, 3]);                  // the thrust is the axis turned by the gimbal angle, toward the tail's side
    ctx.fillStyle = css('--muted'); ctx.textAlign = 'left'; ctx.fillText(`gimbal ${gp.toFixed(2)}°`, 8, 14);
  },
  unmount(app) { const S = this.s; if (!S) return; S.group.removeFromParent(); this.s = null; },
};
