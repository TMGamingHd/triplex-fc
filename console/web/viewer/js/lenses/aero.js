// Aerodynamics: the air the vehicle is flying through and what it does to it. Where the pressure is high on its skin, how the air goes round it, the shock in front of it, the forces on it and where they act:
// above all the centre of pressure and the centre of mass, and the distance between them, which decides whether the air turns the vehicle back into the wind or away from it.
import * as THREE from '../../vendor/three.module.js';
import { Chart, legend } from '../../../js/charts.js';
import { h, stat, statGrid, card, seg, toggle, slider, provenance, fmt, store } from '../ui.js';
import { Label, Arrow, mark, line, setLine, legendCanvas } from '../gfx.js';
import { CpSurface, Streamlines, Shock, Condensation } from '../flow.js';
import { RAD } from '../physics.js';

const bodyToModel = (v) => new THREE.Vector3(v[2], v[0], v[1]);

export default {
  id: 'aero', label: 'Aerodynamics', key: '2',
  mount(app) {
    const { world, vehicle, spec } = app;
    const S = (this.s = { opts: store.get('aero.opts', { view: 'flow', cgcp: true, forces: true, wind: true, arc: true, streams: true, shock: true, cond: true, pmode: 0, opacity: 0.85, contours: true }), objs: [] });
    const o = S.opts, save = () => store.set('aero.opts', o);
    { const v = new URLSearchParams(window.__query || location.search).get('view'); if (v) o.view = v; }      // ?view=flow|pressure|stability, for looking at one view in a screenshot
    const g = S.group = new THREE.Group(); g.name = 'aero overlays'; world.modelHolder.add(g);
    // 3D objects
    S.cg = mark('cg'); S.cp = mark('cp'); g.add(S.cg, S.cp);
    S.cgL = new Label({ text: 'CG', color: '#3ddc97', border: '#3ddc97' }); S.cgL.setAnchor(1.15, 0.5); S.cpL = new Label({ text: 'CP', color: '#4cc9f0', border: '#4cc9f0' }); S.cpL.setAnchor(1.15, 0.5); g.add(S.cgL.object, S.cpL.object);
    S.dim = line([new THREE.Vector3(), new THREE.Vector3(), new THREE.Vector3(), new THREE.Vector3()], 0xffd166); g.add(S.dim);
    S.dimL = new Label({ text: '', color: '#ffd166', border: '#ffd166' }); g.add(S.dimL.object);
    S.arrows = {};
    const mk = (id, colour, text) => { const a = new Arrow(colour), l = new Label({ text, color: '#' + colour.toString(16).padStart(6, '0'), size: 0.026 }); l.setAnchor(0, 0.5); g.add(a.object, l.object); S.arrows[id] = { a, l }; };
    mk('wind', 0x7ee0d0, 'WIND'); mk('aero', 0xff7eb6, 'AERO'); mk('drag', 0xffb454, 'DRAG'); mk('lift', 0xc58bff, 'LIFT'); mk('thrust', 0xff5d73, 'THRUST'); mk('weight', 0xdbe3f1, 'WEIGHT');
    S.arc = line([new THREE.Vector3(), new THREE.Vector3()], 0xffffff); g.add(S.arc); S.arcL = new Label({ text: 'α', color: '#ffffff' }); g.add(S.arcL.object);
    S.cpSurf = new CpSurface(vehicle); S.streams = new Streamlines(vehicle); S.shock = new Shock(vehicle); S.cond = new Condensation(world);
    S.charts = {};

    const set = (key, v) => { o[key] = v; save(); this.applyView(app); };
    // the view switch: three ways of looking at the same flight
    const views = seg([['flow', 'Flow', 'Streamlines, the shock and the pressure on the skin'], ['pressure', 'Pressure', 'The pressure on the skin, without the streamlines'], ['stability', 'CP and CG', 'Side view with the centre of pressure and the centre of mass and the forces']], o.view, (v) => { o.view = v; save(); this.applyView(app, true); }, 'view');
    S.views = views;
    const pmode = seg([[0, 'Cp', 'Pressure coefficient'], [1, 'kPa', 'Static pressure on the skin'], [2, 'Temp', 'Recovery temperature (illustrative)']], o.pmode, (v) => { o.pmode = +v; save(); }, 'pressure map');

    // ---- the left dock: the condition and the forces
    S.stats = {
      mach: stat('Mach', '', 'The speed through the air over the speed of sound'), q: stat('Dynamic pressure', 'kPa', '½ ρ V²'), alpha: stat('Angle of attack', '°', 'The angle between the vehicle\'s axis and its velocity through the air'), alt: stat('Altitude', 'km'),
      v: stat('Air speed', 'm/s'), re: stat('Reynolds', '', 'ρ V L / μ, with the vehicle\'s length'), t0: stat('Stagnation T', 'K', 'T (1 + 0.2 M²): the temperature where the air is brought to rest'), heat: stat('Nose heating', 'kW/m²', 'Sutton-Graves convective heat flux at the stagnation point of a nose of an assumed 0.5 m radius: an illustration'),
      cn: stat('C_N', '', 'Normal-force coefficient: the force at right angles to the axis over q S'), ca: stat('C_A', '', 'Axial-force coefficient'), cd: stat('C_D', '', 'Drag coefficient: the force against the velocity over q S'), cl: stat('C_L', '', 'Lift coefficient: the force across the velocity over q S'),
      normal: stat('Normal force', 'kN'), drag: stat('Drag', 'kN'), ld: stat('L / D', ''), mom: stat('Moment at CG', 'MN·m', 'The aerodynamic moment about the centre of mass'),
    };
    const L = S.stats;
    S.regime = h('div', { class: 'v-regime' });
    const left = [
      card('Flight condition', [statGrid(2, L.mach, L.q, L.alpha, L.alt, L.v, L.re, L.t0, L.heat), S.regime]),
      card('Coefficients and forces', statGrid(4, L.cn, L.ca, L.cd, L.cl, L.normal, L.drag, L.ld, L.mom)),
    ];
    S.stab = { margin: stat('Static margin', 'cal', '(x_CG − x_CP) / diameter: positive when the centre of pressure is behind the centre of mass'), xcg: stat('x CG', 'm', 'From the tail'), xcp: stat('x CP', 'm', 'From the tail'), gap: stat('CP − CG', 'm') };
    S.diagram = h('canvas', { class: 'v-diagram', width: 300, height: 330 });
    S.verdict = h('div', { class: 'v-verdict' });
    left.push(card('Stability: centre of pressure and centre of mass', [statGrid(2, S.stab.margin, S.stab.gap, S.stab.xcg, S.stab.xcp), S.diagram, S.verdict,
      h('p', { class: 'note' }, 'The centre of pressure is where the sum of the air\'s forces acts; the centre of mass is where the vehicle balances. Behind it (positive margin) the air turns the vehicle back into the wind; ahead of it the air turns it further away and the gimbals must hold it: that is normal for a launcher.')]));
    app.left.replaceChildren(...left);

    // ---- the right dock: what to draw, and the history
    const rows = [
      h('div', { class: 'v-row' }, views),
      toggle('Centre of pressure and centre of mass', o.cgcp, (v) => set('cgcp', v)), toggle('Forces: thrust, weight, aerodynamic, drag, lift', o.forces, (v) => set('forces', v)),
      toggle('Relative wind', o.wind, (v) => set('wind', v)), toggle('Angle of attack', o.arc, (v) => set('arc', v)),
      toggle('Streamlines', o.streams, (v) => set('streams', v)), toggle('Shock', o.shock, (v) => set('shock', v)), toggle('Transonic cloud', o.cond, (v) => set('cond', v), 'The Prandtl-Glauert cloud near Mach 1 in humid air'),
      h('div', { class: 'v-row' }, h('span', { class: 'v-lbl' }, 'Pressure on the skin'), pmode), slider('Opacity', 0, 1, 0.05, o.opacity, (v) => { o.opacity = v; save(); }, (v) => Math.round(v * 100) + ' %'),
      toggle('Contour lines', o.contours, (v) => { o.contours = v; save(); S.cpSurf.mat.uniforms.uBands.value = v ? 1 : 0; }),
      h('div', { class: 'v-legend' }, legendCanvas(), h('div', { class: 'v-legend-ends' }, h('span', { text: 'suction' }), h('span', { text: 'ambient' }), h('span', { text: 'stagnation' }))),
    ];
    const mkChart = (series, opt = {}) => { const c = new Chart(h('canvas'), { height: 110, window: 60, series, ...opt }); return c; };
    S.charts = { alpha: mkChart([{ key: 'alpha', label: 'α °', color: 'var(--accent)' }], { zero: true }), q: mkChart([{ key: 'q', label: 'q Pa', color: 'var(--warn)' }]), mach: mkChart([{ key: 'mach', label: 'Mach', color: 'var(--ok)' }]),
      margin: mkChart([{ key: 'static_margin', label: 'static margin (cal)', color: 'var(--nC)' }], { zero: true }), pos: mkChart([{ key: 'cg', label: 'x CG m', color: 'var(--ok)' }, { key: 'cp', label: 'x CP m', color: 'var(--accent)' }]) };
    const ch = (t, c) => card(t, [c.c, legend(c.o.series)]);
    app.right.replaceChildren(
      card('What to draw', rows),
      card('Where the numbers come from', [provenance('measured', 'Mach, angle of attack, the air, the forces, the centre of mass and the centre of pressure: the simulator\'s own state.'),
        provenance('derived', 'the coefficients, Reynolds number, stagnation temperature and nose heating: formulas on those numbers. The pressure map is the modified Newtonian estimate (valid above about Mach 3).'),
        provenance('illustrative', 'the streamlines (slender-body potential flow), the shock (Taylor-Maccoll cone shock) and the transonic cloud (humidity assumed). Not a flow solution.')]),
      ch('Angle of attack', S.charts.alpha), ch('Dynamic pressure', S.charts.q), ch('Mach number', S.charts.mach), ch('Static margin', S.charts.margin), ch('Centre of mass and centre of pressure along the vehicle', S.charts.pos));
    S.cpSurf.mat.uniforms.uBands.value = o.contours ? 1 : 0;
    this.applyView(app, true);
  },

  applyView(app, camera = false) {
    const S = this.s, o = S.opts, v = o.view;
    S.streams.setVisible(o.streams && v === 'flow'); S.shock.setVisible(o.shock && v !== 'stability'); S.cpSurf.setVisible(v !== 'stability');
    S.showMarks = o.cgcp || v === 'stability'; S.showForces = o.forces || v === 'stability';
    if (camera) {
      const r = app.world.rig;
      if (v === 'stability') { r.set('orbit'); r.yaw = Math.PI / 2; r.pitch = 0.0; r.dist = 1.95; r.anchor = 0.5; }
      else if (v === 'flow') { r.set('orbit'); r.yaw = 0.9; r.pitch = 0.12; r.dist = 1.5; r.anchor = 0.55; }
      else { r.set('orbit'); r.yaw = 0.7; r.pitch = 0.15; r.dist = 1.1; r.anchor = 0.6; }
    }
  },

  update(app, pose, d, dt) {
    const S = this.s, o = S.opts, { world, vehicle, spec } = app, g = S.group;
    const Lv = spec.length, R = spec.diameter / 2;
    const view = o.view;
    const cgY = pose.cg, cpY = pose.cp, stab = view === 'stability';
    // markers and their dimension line
    const showM = S.showMarks;
    S.cg.visible = S.cp.visible = S.cgL.object.visible = S.cpL.object.visible = S.dim.visible = S.dimL.object.visible = showM;
    S.cg.position.set(0, cgY, 0); S.cp.position.set(0, cpY, 0);
    const off = R * 1.5 + 2;
    S.cgL.object.position.set(0, cgY, -R * 0.4); S.cpL.object.position.set(0, cpY, -R * 0.4);
    const ptsA = [new THREE.Vector3(0, cgY, R * 1.0), new THREE.Vector3(0, cgY, off), new THREE.Vector3(0, cpY, off), new THREE.Vector3(0, cpY, R * 1.0)];      // beside the vehicle, in the plane of the side view
    setLine(S.dim, ptsA);
    const gap = cpY - cgY, stable = gap < 0;
    S.dimL.setText(`${Math.abs(d.margin).toFixed(2)} cal  ${stable ? 'stable' : 'unstable'}\n${Math.abs(gap).toFixed(1)} m ${gap < 0 ? 'CP behind CG' : 'CP ahead of CG'}`, stable ? '#3ddc97' : '#ffb454');
    S.dimL.object.position.set(0, (cgY + cpY) / 2, off + R * 0.15); S.dimL.setAnchor(1.02, 0.5);
    S.dim.material.color.set(stable ? 0x3ddc97 : 0xffb454);

    // forces
    const fmodel = (v) => bodyToModel(v);
    const thr = fmodel(pose.fth || [0, 0, 0]), aero = fmodel(pose.fae || [0, 0, 0]);
    const q = new THREE.Quaternion(pose.q[1], pose.q[2], pose.q[3], pose.q[0]);            // body -> inertial, in the simulator's axes
    const rv = new THREE.Vector3(pose.r[0], pose.r[1], pose.r[2]), rn = rv.length(), gAcc = spec.planet.mu / (rn * rn);
    const gBody = rv.clone().multiplyScalar(-gAcc / rn).applyQuaternion(q.clone().invert());   // gravity in the body axes
    const W = pose.m * gAcc;
    const wdir = bodyToModel([gBody.x, gBody.y, gBody.z]).normalize().multiplyScalar(W);
    const vel = fmodel(pose.vair || [1, 0, 0]);
    const drag = new THREE.Vector3(); const lift = fmodel(d.Lvec);
    drag.copy(vel).normalize().multiplyScalar(-d.D);
    const Fmax = Math.max(thr.length(), W, aero.length(), 1);
    const Lref = Lv * 0.42, len = (F) => Lref * Math.pow(Math.min(F / Fmax, 1), 0.55);
    const A = S.arrows, showF = S.showForces;
    const place = (id, origin, vec, label, show) => {
      const a = A[id], F = vec.length();
      a.a.object.visible = a.l.object.visible = show && F > 1;
      if (!(show && F > 1)) return;
      const Ln = Math.max(len(F), Lv * 0.03);
      a.a.set(origin, vec, Ln, Math.max(0.15, Math.min(R * 0.12, Ln * 0.03)));
      a.l.setText(`${label} ${F >= 1e6 ? (F / 1e6).toFixed(2) + ' MN' : (F / 1e3).toFixed(1) + ' kN'}`);
      a.l.object.position.copy(origin).addScaledVector(vec.clone().normalize(), Ln * 1.04);
      a.l.setAnchor(0, { drag: -0.5, lift: 1.6, aero: 0.5, thrust: 0.5, weight: 0.5 }[id] ?? 0.5);
    };
    place('thrust', new THREE.Vector3(0, 0, 0), thr, 'THRUST', showF);
    place('weight', new THREE.Vector3(0, cgY, 0), wdir, 'WEIGHT', showF);
    place('aero', new THREE.Vector3(0, cpY, 0), aero, 'AERO', showF);
    place('drag', new THREE.Vector3(0, cpY, 0), drag, 'DRAG', showF && stab);
    place('lift', new THREE.Vector3(0, cpY, 0), lift, 'LIFT', showF && stab);
    // the relative wind: an arrow ahead of the nose pointing the way the air moves
    {
      const a = A.wind, f = new THREE.Vector3(d.fhat[2], d.fhat[0], d.fhat[1]);
      const show = o.wind && d.V > 5;
      a.a.object.visible = a.l.object.visible = show;
      if (show) { const Ln = Lv * 0.3, start = new THREE.Vector3(0, Lv, 0).addScaledVector(f, -Ln * 1.15 - R * 1.5); a.a.set(start, f, Ln, Math.max(0.2, R * 0.08)); a.l.setText(`WIND ${d.V.toFixed(0)} m/s`); a.l.object.position.copy(start); }
    }
    // the angle of attack: an arc at the CG between the axis and the velocity through the air
    {
      const vhat = new THREE.Vector3(d.vhat[2], d.vhat[0], d.vhat[1]), ax = new THREE.Vector3(0, 1, 0), r0 = Lv * 0.18;
      const show = o.arc && d.alpha > 0.15 && d.V > 5;
      S.arc.visible = S.arcL.object.visible = show;
      if (show) {
        const ang = ax.angleTo(vhat), n = new THREE.Vector3().crossVectors(ax, vhat).normalize(), pts = [];
        for (let i = 0; i <= 16; i++) pts.push(ax.clone().applyAxisAngle(n, ang * i / 16).multiplyScalar(r0).add(new THREE.Vector3(0, cgY, 0)));
        pts.push(new THREE.Vector3(0, cgY, 0)); pts.unshift(new THREE.Vector3(0, cgY, 0));
        setLine(S.arc, pts); S.arcL.setText(`α ${d.alpha.toFixed(1)}°`); S.arcL.object.position.copy(ax.clone().applyAxisAngle(n, ang * 0.5).multiplyScalar(r0 * 1.15).add(new THREE.Vector3(0, cgY, 0)));
      }
    }
    // the flow
    const mode = o.pmode;
    S.cpSurf.update(d, mode === 2 ? 2 : 0, o.opacity);
    S.streams.update(d, pose.stg ?? 255, spec, app.wall, dt);
    const segs = S.streams.profile(pose.stg ?? 255);
    if (segs.length && S.shock.visible) { const tip = segs[segs.length - 1][2]; S.shock.update(d, tip, S.streams.noseAngle(segs), Math.min(spec.length * 0.9, 90), app.wall); }
    S.cond.update({ ...d, alt: pose.alt }, (spec.length) * 0.86, R, world.camera, [1, 1, 1].map((x, i) => (world.sunT ? world.sunT[i] : 1)), o.cond && view !== 'stability', app.wall);

    // the readouts
    const st = S.stats;
    st.mach.set(d.M.toFixed(2)); st.q.set((d.q / 1000).toFixed(1)); st.alpha.set(d.alpha.toFixed(2)); st.alt.set((pose.alt / 1000).toFixed(1)); st.v.set(d.V.toFixed(0));
    st.re.set(fmt.eng(d.Re, 1)); st.t0.set(d.T0.toFixed(0), d.T0 > 1500 ? 'warn' : ''); st.heat.set((d.heat / 1000).toFixed(d.heat > 1e5 ? 0 : 1), d.heat > 5e5 ? 'warn' : '');
    st.cn.set(d.cn.toFixed(3)); st.ca.set(d.ca.toFixed(3)); st.cd.set(d.cd.toFixed(3)); st.cl.set(d.cl.toFixed(3)); st.normal.set((d.N / 1000).toFixed(1)); st.drag.set((d.D / 1000).toFixed(1)); st.ld.set(d.ld.toFixed(2));
    const mom = Math.hypot((pose.mae || [0, 0, 0])[1], (pose.mae || [0, 0, 0])[2]); st.mom.set((mom / 1e6).toFixed(2));
    S.regime.textContent = `${d.regime}${d.mu ? `, Mach angle ${d.mu.toFixed(1)}°` : ''}; ${d.layer.name}`;
    S.stab.margin.set(d.margin.toFixed(2), d.margin >= 0 ? 'ok' : 'warn'); S.stab.gap.set(Math.abs(gap).toFixed(1)); S.stab.xcg.set(pose.cg.toFixed(1)); S.stab.xcp.set(pose.cp.toFixed(1));
    S.verdict.className = 'v-verdict ' + (stable ? 'ok' : 'warn');
    S.verdict.textContent = d.q < 200 ? 'Almost no air: the aerodynamic forces are negligible; the position of the centre of pressure is the vehicle\'s own shape.' : stable ? 'Statically stable: the air turns the vehicle back into the wind.' : 'Statically unstable: the air turns the vehicle away from the wind. The gimbals hold it.';
    this.drawDiagram(app, pose, d);
    for (const c of Object.values(S.charts)) c.draw(app.data.hist);
  },

  /** A side view of the vehicle's outline with the centre of mass, the centre of pressure, the wind and the angle of attack, on a canvas. */
  drawDiagram(app, pose, d) {
    const c = this.s.diagram, spec = app.spec, ctx = c.getContext('2d');
    const dpr = window.devicePixelRatio || 1, W = c.clientWidth || 300, H = 330;
    if (c.width !== Math.round(W * dpr)) { c.width = Math.round(W * dpr); c.height = Math.round(H * dpr); }
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0); ctx.clearRect(0, 0, W, H);
    const css = (v) => getComputedStyle(document.documentElement).getPropertyValue(v).trim();
    const L = spec.length, pad = 16, scale = (H - 2 * pad) / L, cx = W * 0.42;
    const Y = (x) => H - pad - x * scale;
    // the outline of the stages still on the vehicle
    ctx.fillStyle = css('--panel3'); ctx.strokeStyle = css('--line2'); ctx.lineWidth = 1.5;
    app.vehicle.stages.forEach((s, i) => {
      if (!((pose.stg ?? 255) & (1 << i))) return;
      ctx.beginPath(); const p = s.profile;
      ctx.moveTo(cx - p[0].r * scale, Y(p[0].x)); for (const q of p) ctx.lineTo(cx - q.r * scale, Y(q.x)); for (let k = p.length - 1; k >= 0; k--) ctx.lineTo(cx + p[k].r * scale, Y(p[k].x)); ctx.closePath(); ctx.fill(); ctx.stroke();
    });
    // the wind and the angle of attack: the air moves toward the tail, tilted by alpha in the plane of the angle of attack
    const a = d.alpha * RAD, wx = Math.sin(a) * 40, wy = Math.cos(a) * 40;
    ctx.strokeStyle = css('--nSIM'); ctx.fillStyle = css('--nSIM'); ctx.lineWidth = 2;
    const sx = W * 0.82, sy = 40; ctx.beginPath(); ctx.moveTo(sx - wx, sy - wy + 20); ctx.lineTo(sx, sy + 20); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(sx, sy + 20); ctx.lineTo(sx - 5 + wx * 0.1, sy + 10); ctx.lineTo(sx + 5 + wx * 0.1, sy + 10); ctx.fill();
    ctx.font = '11px system-ui'; ctx.fillStyle = css('--muted'); ctx.fillText(`wind  α ${d.alpha.toFixed(1)}°`, sx - 56, sy + 38);
    // CG and CP
    const mk = (x, colour, text, side) => {
      ctx.strokeStyle = colour; ctx.fillStyle = colour; ctx.lineWidth = 2;
      ctx.beginPath(); ctx.moveTo(cx - 52, Y(x)); ctx.lineTo(cx + 52, Y(x)); ctx.stroke();
      ctx.beginPath(); ctx.arc(cx, Y(x), 6, 0, 7); ctx.fill();
      ctx.font = '600 12px system-ui'; ctx.textAlign = side > 0 ? 'left' : 'right'; ctx.fillText(text, cx + side * 58, Y(x) + 4); ctx.textAlign = 'left';
    };
    mk(pose.cg, css('--ok'), `CG ${pose.cg.toFixed(1)} m`, 1); mk(pose.cp, css('--accent'), `CP ${pose.cp.toFixed(1)} m`, -1);
    ctx.strokeStyle = css('--warn'); ctx.setLineDash([3, 3]); ctx.beginPath(); ctx.moveTo(cx + 70, Y(pose.cg)); ctx.lineTo(cx + 70, Y(pose.cp)); ctx.stroke(); ctx.setLineDash([]);
    ctx.fillStyle = css('--warn'); ctx.font = '11px system-ui'; ctx.fillText(`${d.margin.toFixed(2)} cal`, cx + 76, (Y(pose.cg) + Y(pose.cp)) / 2 + 4);
    ctx.fillStyle = css('--faint'); ctx.fillText('nose', 6, 14); ctx.fillText('tail', 6, H - 4);
  },

  unmount(app) {
    const S = this.s; if (!S) return;
    S.cpSurf.dispose(); S.streams.dispose(); S.shock.dispose(); S.cond.dispose();
    S.group.removeFromParent();
    this.s = null;
  },
};
