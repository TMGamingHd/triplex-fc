// Propulsion: the engines and the propellant. Which engines are running and at what throttle, what the engines are making against the air they push into, how much propellant is in which tank, how fast it is going,
// and what it is worth in speed. The vehicle can be seen through (a cutaway) with the liquid in its tanks at the level the simulator has them.
import * as THREE from '../../vendor/three.module.js';
import { Chart, legend } from '../../../js/charts.js';
import { h, stat, statGrid, card, toggle, slider, provenance, fmt, store } from '../ui.js';
import { Arrow, Label } from '../gfx.js';
import { us1976 } from '../data.js';
import { G0 } from '../physics.js';

export default {
  id: 'propulsion', label: 'Propulsion', key: '4',
  mount(app) {
    const { world, vehicle, spec } = app;
    const S = (this.s = { cutaway: store.get('prop.cut', true), showTags: store.get('prop.tags', true), saved: [] });
    // engine tags: a disc at each nozzle exit that shows the state of the engine through the skirt
    S.tags = vehicle.engines.map((en) => {
      const m = new THREE.Mesh(new THREE.CircleGeometry(en.exitRadius * 1.1, 20), new THREE.MeshBasicMaterial({ color: 0x555555, depthTest: false, transparent: true, opacity: 0.85, side: THREE.DoubleSide, toneMapped: false }));
      m.rotation.x = Math.PI / 2; m.position.y = -en.pivot - 0.05; m.renderOrder = 60; en.holder.add(m); return m;
    });
    // the tanks: liquid at the level the simulator has it, inside an outline of the full tank
    S.tanks = [];
    const g = S.group = new THREE.Group(); world.modelHolder.add(g);
    spec.tanks.forEach((t, i) => {
      const area = Math.PI * t.radius_m * t.radius_m, full = t.propellant_kg / (t.density_kg_m3 * area);
      const ox = t.density_kg_m3 >= 1000, col = ox ? 0x4fc3f7 : 0xffb454;
      const liquid = new THREE.Mesh(new THREE.CylinderGeometry(t.radius_m * 0.985, t.radius_m * 0.985, 1, 40, 1), new THREE.MeshBasicMaterial({ color: col, transparent: true, opacity: 0.55, depthWrite: false, side: THREE.DoubleSide, toneMapped: false }));
      const outline = new THREE.LineSegments(new THREE.EdgesGeometry(new THREE.CylinderGeometry(t.radius_m, t.radius_m, full, 40, 1)), new THREE.LineBasicMaterial({ color: col, transparent: true, opacity: 0.6, depthTest: false, toneMapped: false }));
      outline.position.set(0, t.x_bottom_m + full / 2, 0); outline.renderOrder = 55;
      liquid.renderOrder = 50; g.add(liquid, outline);
      S.tanks.push({ t, liquid, outline, full, area, ox });
    });
    S.thrust = new Arrow(0xff5d73); S.thrustL = new Label({ text: 'THRUST', color: '#ff5d73', size: 0.026 }); g.add(S.thrust.object, S.thrustL.object);
    this.applyCutaway(app);

    const k = (l, u, hint) => stat(l, u, hint);
    S.st = { thr: k('Thrust', 'MN'), frac: k('of vacuum thrust', '%', 'The thrust the running engines make now over what the same engines make in vacuum: it rises as the air thins'), isp: k('Specific impulse', 's', 'Thrust / (mass flow g₀): the effective one at this altitude'), mdot: k('Mass flow', 'kg/s'),
      thr_t: k('Throttle', '%', 'The mean thrust fraction of the engines that are running'), eng: k('Engines', '', 'Running of those on the vehicle (a failed engine is counted apart)'), twr: k('Thrust / weight', ''), pl: k('Pressure loss', 'MN', 'p_ambient × the exit areas of the running engines: what the air takes off the vacuum thrust'),
      pa: k('Ambient pressure', 'kPa'), burn: k('Burn time left', 's', 'Propellant of the stage that is firing over the total mass flow'), dv: k('Delta-v left', 'm/s', 'The stage\'s own: Isp g₀ ln(m / (m − propellant)), with the effective Isp now') };
    const s = S.st;
    S.stages = h('div', { class: 'v-stages' });
    S.map = h('canvas', { class: 'v-map', width: 300, height: 260 });
    app.left.replaceChildren(card('Engines now', statGrid(3, s.thr, s.frac, s.isp, s.mdot, s.thr_t, s.eng, s.twr, s.pl, s.pa)), card('Propellant', [S.stages, statGrid(2, s.burn, s.dv)]),
      card('Engine map (the firing stage seen from behind)', [S.map, h('p', { class: 'note' }, 'Green: running (brighter, the higher the throttle). Grey: off. Red: failed. Rings are the engines\' positions in the vehicle file.')]));
    S.charts = {
      thrust: new Chart(h('canvas'), { height: 110, window: 150, series: [{ key: 'thrust', label: 'thrust N', color: 'var(--crit)' }] }), isp: new Chart(h('canvas'), { height: 110, window: 150, series: [{ key: 'isp', label: 'Isp s', color: 'var(--ok)' }] }),
      gim: new Chart(h('canvas'), { height: 110, window: 150, series: [{ key: 'gim_p', label: 'gimbal pitch °', color: 'var(--accent)' }, { key: 'gim_y', label: 'gimbal yaw °', color: 'var(--nC)' }], zero: true }), mass: new Chart(h('canvas'), { height: 110, window: 150, series: [{ key: 'mass', label: 'mass kg', color: 'var(--nB)' }] }),
    };
    S.curve = h('canvas', { class: 'v-profile', width: 320, height: 260 });
    let chN = 0; const ch = (t, c) => card(t, [c.c, legend(c.o.series)], null, { open: chN++ === 0 });
    app.right.replaceChildren(card('Show', [toggle('See through the vehicle (cutaway with the tanks)', S.cutaway, (v) => { S.cutaway = v; store.set('prop.cut', v); this.applyCutaway(app); }), toggle('State of each engine', S.showTags, (v) => { S.showTags = v; store.set('prop.tags', v); })]),
      card('Thrust against altitude (this stage)', [S.curve, h('p', { class: 'note' }, 'Vacuum thrust minus the ambient pressure times the exit area: the engines make more as the air thins. The dot is the vehicle now.')]),
      ch('Thrust', S.charts.thrust), ch('Effective specific impulse', S.charts.isp), ch('Gimbal of the first stage', S.charts.gim), ch('Mass', S.charts.mass),
      card('Where the numbers come from', [provenance('measured', 'the thrust fraction of every engine, the propellant in every tank, the mass flow, the gimbal: the simulator\'s state.'), provenance('derived', 'specific impulse, pressure loss, burn time and delta-v are formulas on those.'),
        provenance('illustrative', 'the colour of the liquid, and the outline of a tank (a cylinder: the simulator\'s model of it). The nozzle\'s own exit pressure is not modelled, so how far a plume spreads is drawn from an assumption.')]));
  },
  applyCutaway(app) {
    const S = this.s, root = app.vehicle.root;
    for (const r of S.saved) { r.m.transparent = r.t; r.m.opacity = r.o; r.m.depthWrite = r.d; r.m.needsUpdate = true; }
    S.saved = [];
    if (!S.cutaway) return;
    const seen = new Set();
    root.traverse((o) => {
      if (!o.isMesh || !o.material || o.material.isShaderMaterial) return;
      for (let p = o.parent; p; p = p.parent) if (p.name && p.name.startsWith('engine')) return;
      if (seen.has(o.material)) return; seen.add(o.material);
      const m = o.material; S.saved.push({ m, t: m.transparent, o: m.opacity, d: m.depthWrite }); m.transparent = true; m.opacity = 0.16; m.depthWrite = false; m.needsUpdate = true;
    });
  },
  update(app, pose, d, dt) {
    const S = this.s, s = S.st, { spec, vehicle } = app, air = pose.air || us1976(pose.alt), pa = air[1];
    const eng = pose.eng || [], stg = pose.stg ?? 255;
    // the engines: state and totals
    let running = 0, failed = 0, on = 0, sum = 0, thrVac = 0, ae = 0, total = 0, lead = -1;
    spec.engines.forEach((e, i) => { if (!((stg >> e.stage) & 1)) return; total++; const f = eng[i]; if (f < 0) failed++; else if (f > 0.02) { running++; sum += f; thrVac += f * e.thrust_vac_n; ae += e.exit_area_m2; if (lead < 0 || e.stage < lead) lead = e.stage; } });
    const thr = pose.thr, mdot = pose.mdot, isp = mdot > 1 ? thr / (mdot * G0) : 0, loss = pa * ae, wt = pose.m * G0;
    s.thr.set((thr / 1e6).toFixed(2)); s.frac.set(thrVac > 1 ? (100 * thr / thrVac).toFixed(1) : '—'); s.isp.set(isp ? isp.toFixed(0) : '—'); s.mdot.set(fmt.eng(mdot, 1)); s.thr_t.set(running ? (100 * sum / running).toFixed(0) : '—');
    s.eng.set(`${running}/${total}` + (failed ? `  (${failed} failed)` : ''), failed ? 'crit' : ''); s.twr.set((thr / wt).toFixed(2)); s.pl.set((loss / 1e6).toFixed(2)); s.pa.set((pa / 1000).toFixed(pa > 1000 ? 1 : 3));
    // the stage that is firing: how long, how far
    const leadStage = lead >= 0 ? lead : spec.stages.findIndex((_, i) => (stg >> i) & 1);
    const propLead = pose.prop ? pose.prop[Math.max(0, leadStage)] : 0;
    s.burn.set(mdot > 1 ? (propLead / mdot).toFixed(0) : '—'); s.dv.set(isp && propLead > 0 ? (isp * G0 * Math.log(pose.m / Math.max(1, pose.m - propLead))).toFixed(0) : '—');
    // the stages and their tanks
    const el = S.stages;
    if (el.childElementCount !== spec.stages.length) el.replaceChildren(...spec.stages.map((st, i) => h('div', { class: 'v-stage' }, h('div', { class: 'v-stage-h' }, h('b', {}, st.name || 'stage ' + (i + 1)), h('span', { class: 'v-stage-state' })), h('div', { class: 'v-bar' }, h('i')), h('div', { class: 'v-stage-n' }))));
    spec.stages.forEach((st, i) => {
      const row = el.children[i], tanks = S.tanks.filter((t) => t.t.stage === i), cap = tanks.reduce((a, t) => a + t.t.propellant_kg, 0), left = pose.prop ? pose.prop[i] : 0, onv = !!((stg >> i) & 1);
      row.classList.toggle('gone', !onv);
      row.querySelector('.v-stage-state').textContent = onv ? `${(100 * left / Math.max(cap, 1)).toFixed(0)} %` : 'separated';
      row.querySelector('.v-bar i').style.width = (onv ? 100 * left / Math.max(cap, 1) : 0).toFixed(1) + '%';
      row.querySelector('.v-stage-n').textContent = onv ? tanks.map((t) => `${t.ox ? 'oxidizer' : 'fuel'} ${fmt.eng((pose.tank || [])[S.tanks.indexOf(t)] || 0)} kg`).join(' · ') : '';
    });
    // 3D: the liquid, the tags, the thrust arrow
    S.tanks.forEach((t, i) => {
      const kg = (pose.tank || [])[i] || 0, hgt = Math.max(0.001, kg / (t.t.density_kg_m3 * t.area));
      t.liquid.visible = t.outline.visible = !!((stg >> t.t.stage) & 1) && S.cutaway;
      t.liquid.scale.y = hgt; t.liquid.position.set(0, t.t.x_bottom_m + hgt / 2, 0);
      const sl = pose.slosh && pose.slosh[2 * i] != null ? pose.slosh : null; if (sl) { t.liquid.position.x = (sl[2 * i + 1] || 0) * 4; t.liquid.position.z = (sl[2 * i] || 0) * 4; }
    });
    vehicle.engines.forEach((en, i) => {
      const tag = S.tags[i], f = eng[i];
      tag.visible = S.showTags && ((stg >> en.stage) & 1) !== 0;
      tag.material.color.setRGB(f < 0 ? 1 : f > 0.02 ? 0.2 : 0.35, f < 0 ? 0.2 : f > 0.02 ? 0.55 + 0.45 * f : 0.35, f < 0 ? 0.25 : f > 0.02 ? 0.35 : 0.37);
    });
    const th = new THREE.Vector3(pose.fth[2], pose.fth[0], pose.fth[1]);
    S.thrust.object.visible = S.thrustL.object.visible = th.length() > 1;
    if (th.length() > 1) { const Ln = spec.length * 0.28; S.thrust.set(new THREE.Vector3(0, 1, 0), th, Ln, Math.max(0.2, spec.diameter * 0.04)); S.thrustL.setText(`THRUST ${(th.length() / 1e6).toFixed(2)} MN`); S.thrustL.object.position.set(0, Ln * 1.05 + 1, 0); }
    this.drawMap(app, pose, leadStage); this.drawCurve(app, pose, leadStage);
    for (const c of Object.values(S.charts)) c.draw(app.data.hist);
  },
  drawMap(app, pose, stage) {
    const c = this.s.map, spec = app.spec, ctx = c.getContext('2d'), eng = pose.eng || [];
    const dpr = window.devicePixelRatio || 1, W = c.clientWidth || 300, H = 260;
    if (c.width !== Math.round(W * dpr)) { c.width = Math.round(W * dpr); c.height = Math.round(H * dpr); }
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0); ctx.clearRect(0, 0, W, H);
    const css = (v) => getComputedStyle(document.documentElement).getPropertyValue(v).trim();
    const list = spec.engines.map((e, i) => [e, i]).filter(([e]) => e.stage === Math.max(0, stage));
    const R = spec.diameter / 2, sc = Math.min(W, H) / 2 / (R * 1.08), cx = W / 2, cy = H / 2;
    ctx.strokeStyle = css('--line2'); ctx.lineWidth = 1.5; ctx.beginPath(); ctx.arc(cx, cy, R * sc, 0, 7); ctx.stroke();
    for (const [e, i] of list) {
      const x = cx + e.position_m[1] * sc, y = cy - e.position_m[2] * sc, f = eng[i], r = Math.max(4, Math.sqrt(e.exit_area_m2 / Math.PI) * sc);
      ctx.beginPath(); ctx.arc(x, y, r, 0, 7);
      ctx.fillStyle = f < 0 ? css('--crit') : f > 0.02 ? `rgba(61,220,151,${0.35 + 0.65 * Math.min(1, f)})` : css('--panel3'); ctx.fill(); ctx.strokeStyle = e.gimbal === false ? css('--faint') : css('--text'); ctx.lineWidth = e.gimbal === false ? 1 : 1.6; ctx.stroke();
      if (r > 7 || list.length <= 12) { ctx.fillStyle = css('--text'); ctx.font = '9px system-ui'; ctx.textAlign = 'center'; ctx.fillText(String(i + 1), x, y + 3); }
    }
    ctx.fillStyle = css('--muted'); ctx.font = '11px system-ui'; ctx.textAlign = 'left'; ctx.fillText(`${list.length} engines; a thick ring gimbals, a thin one is fixed`, 6, H - 6);
  },
  drawCurve(app, pose, stage) {
    const c = this.s.curve, spec = app.spec, ctx = c.getContext('2d'), st = Math.max(0, stage);
    const dpr = window.devicePixelRatio || 1, W = c.clientWidth || 320, H = 260;
    if (c.width !== Math.round(W * dpr)) { c.width = Math.round(W * dpr); c.height = Math.round(H * dpr); }
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0); ctx.clearRect(0, 0, W, H);
    const css = (v) => getComputedStyle(document.documentElement).getPropertyValue(v).trim();
    const list = spec.engines.filter((e) => e.stage === st), Fv = list.reduce((a, e) => a + e.thrust_vac_n, 0), Ae = list.reduce((a, e) => a + e.exit_area_m2, 0);
    const L = 46, R = 10, T = 10, B = 26, pw = W - L - R, ph = H - T - B, top = 120, lo = Fv * 0.78;
    const X = (F) => L + pw * Math.min(1, Math.max(0, (F - lo) / (Fv - lo + 1))), Y = (km) => T + ph * (1 - km / top);
    ctx.strokeStyle = css('--line'); ctx.fillStyle = css('--muted'); ctx.font = '10px system-ui'; ctx.textAlign = 'right';
    for (const km of [0, 30, 60, 90, 120]) { ctx.beginPath(); ctx.moveTo(L, Y(km)); ctx.lineTo(L + pw, Y(km)); ctx.stroke(); ctx.fillText(km + ' km', L - 4, Y(km) + 3); }
    ctx.textAlign = 'center'; for (let i = 0; i <= 4; i++) { const F = lo + (Fv - lo) * i / 4; ctx.fillText((F / 1e6).toFixed(0), X(F), H - 12); } ctx.fillText('thrust at full throttle, MN', L + pw / 2, H - 1);
    ctx.strokeStyle = css('--crit'); ctx.lineWidth = 2; ctx.beginPath();
    for (let km = 0; km <= top; km++) { const F = Fv - spec.airAt(km * 1000)[1] * Ae; const x = X(F), y = Y(km); km === 0 ? ctx.moveTo(x, y) : ctx.lineTo(x, y); } ctx.stroke();
    const alt = pose.alt / 1000, F = Fv - (pose.air ? pose.air[1] : 101325) * Ae;
    ctx.fillStyle = css('--warn'); ctx.beginPath(); ctx.arc(X(F), Y(Math.min(alt, top)), 5, 0, 7); ctx.fill();
    ctx.fillStyle = css('--muted'); ctx.textAlign = 'left'; ctx.fillText(`sea level ${(((Fv - 101325 * Ae)) / 1e6).toFixed(1)} MN, vacuum ${(Fv / 1e6).toFixed(1)} MN`, L + 4, T + 12);
  },
  unmount(app) {
    const S = this.s; if (!S) return;
    for (const r of S.saved) { r.m.transparent = r.t; r.m.opacity = r.o; r.m.depthWrite = r.d; r.m.needsUpdate = true; }
    S.tags.forEach((t) => t.removeFromParent()); S.group.removeFromParent(); this.s = null;
  },
};
