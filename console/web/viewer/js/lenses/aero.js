// Aerodynamics: the air the vehicle is flying through and what it does to it. Where the pressure is high on its skin, how the air goes round it, the shock in front of it, the forces on it and where they act:
// above all the centre of pressure and the centre of mass, and the distance between them, which decides whether the air turns the vehicle back into the wind or away from it.
import * as THREE from '../../vendor/three.module.js';
import { Chart, legend } from '../../../js/charts.js';
import { h, stat, statGrid, card, seg, toggle, slider, provenance, fmt, store } from '../ui.js';
import { Label, Arrow, mark, line, setLine, legendCanvas, declutter } from '../gfx.js';
import { CpSurface, Streamlines, Shock, Condensation } from '../flow.js';
import { CfdView, listCases, loadCase, nearest } from '../cfd.js';
import { GpuFlow, GpuSection } from '../gpuflow.js';
import { RAD } from '../physics.js';

const bodyToModel = (v) => new THREE.Vector3(v[2], v[0], v[1]);

export default {
  id: 'aero', label: 'Aerodynamics', key: '2',
  mount(app) {
    const { world, vehicle, spec } = app;
    const S = (this.s = { opts: store.get('aero.opts2', { view: 'flow', cgcp: true, forces: true, xcgcp: false, xforces: false, wind: true, arc: true, streams: true, shock: true, cond: true, pmode: 0, opacity: 0.85, contours: true, csec: true, cskin: true, live: true, cmode: 0, copac: 0.9, ccase: 'auto' }), objs: [] });
    const o = S.opts, save = () => store.set('aero.opts2', o);
    { const q = new URLSearchParams(window.__query || location.search), v = q.get('view'); if (v) o.view = v; if (q.get('cmode') != null) o.cmode = +q.get('cmode'); if (q.get('ccase')) o.ccase = q.get('ccase'); }      // ?view=flow|pressure|stability, for looking at one view in a screenshot
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
    S.cpSurf = new CpSurface(vehicle); S.cpSurf.setFrame(g); S.cfd = new CfdView(g); S.live = null; S.liveSec = null; S.cfdState = { list: [], loading: '', failed: '' }; listCases().then((m) => { S.cfdState.list = m.cases || []; this.fillCases && this.fillCases(); });
    S.streams = new Streamlines(vehicle); S.shock = new Shock(vehicle); S.cond = new Condensation(world);
    S.charts = {};

    const set = (key, v) => { o[key] = v; save(); this.applyView(app); };
    // the view switch: three ways of looking at the same flight
    const views = seg([['flow', 'Flow', 'Streamlines, the shock and the pressure on the skin'], ['pressure', 'Pressure', 'The pressure on the skin, without the streamlines'], ['stability', 'CP and CG', 'Side view with the centre of pressure and the centre of mass and the forces'], ['cfd', 'CFD', 'The flow the Euler solver computed: a section through it and the pressure on the skin']], o.view, (v) => { o.view = v; save(); this.applyView(app, true); }, 'view');
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
    S.cfdStats = { case: stat('Case', '', 'The Mach number and the angle of attack the solver was run at, which are not the flight\'s'), cells: stat('Cells', '', 'The grid the solver used'), ca: stat('C_A (skin)', '', 'Axial force coefficient from the pressure on the skin; the base of an Euler solution has no wake, so it is left out'), cn: stat('C_N', '', 'Normal-force coefficient of the case'),
      cna: stat('C_N alpha', '/rad', 'The slope of the normal force with the angle of attack, from this case'), xcp: stat('x CP', 'm', 'The solver\'s centre of pressure, from the tail, by the moment about the nose'), simcp: stat('x CP, build-up', 'm', 'What the simulator\'s engineering model puts the centre of pressure at, now'), simcna: stat('C_N alpha, build-up', '/rad', 'The simulator\'s normal-force slope now') };
    S.cfdCard = card('What the solver says', [statGrid(2, S.cfdStats.case, S.cfdStats.cells, S.cfdStats.ca, S.cfdStats.cn, S.cfdStats.cna, S.cfdStats.xcp, S.cfdStats.simcna, S.cfdStats.simcp), S.cfdNote = h('p', { class: 'note' }, ''),
      provenance('measured', 'a computed flow (the Euler equations of a perfect gas on this vehicle\'s own outline, tfc_cfd, ADR-050). The simulator\'s flight does not use it: the forces it flies are the engineering build-up in the last two numbers.')]);
    S.cfdCard.id = 'v-aero-cfdcard'; left.push(S.cfdCard);
    this.fillCases = () => {
      const cur = o.ccase, name = app.spec.name;
      S.cfdPick.replaceChildren(h('option', { value: 'auto' }, 'The case nearest the flight'), ...S.cfdState.list.filter((c) => !c.vehicle || c.vehicle === name).map((c) => h('option', { value: c.name }, `Mach ${+c.mach.toFixed(2)}, ${+c.alpha_deg.toFixed(1)}°`)));
      S.cfdPick.value = [...S.cfdPick.options].some((x) => x.value === cur) ? cur : 'auto';
    };
    app.left.replaceChildren(...left);

    // ---- the right dock: what to draw, and the history
    // what to draw: a view tells one story, and only its own switches are shown. Flow: the air. Pressure: the skin. CP and CG: the forces and the stability. The extras of the other views are off until asked for.
    const grp = (id, ...kids) => h('div', { id, class: 'v-opts' }, kids);
    const rows = [
      h('div', { class: 'v-row' }, views),
      h('p', { class: 'note', id: 'v-aero-says' }, ''),
      grp('v-aero-flow', toggle('Streamlines', o.streams, (v) => set('streams', v)), toggle('Shock', o.shock, (v) => set('shock', v)), toggle('Airflow', o.wind, (v) => set('wind', v), 'The velocity of the vehicle through the air, drawn as the air coming at the nose. It is not the wind: the wind is the air\'s own motion over the ground. Above the air it reads as a plain velocity.'), toggle('Angle of attack', o.arc, (v) => set('arc', v)),
        toggle('Transonic cloud', o.cond, (v) => set('cond', v), 'The Prandtl-Glauert cloud near Mach 1 in humid air'),
        h('div', { class: 'v-legend' }, legendCanvas(), h('div', { class: 'v-legend-ends' }, h('span', { text: 'fast, low pressure' }), h('span', { text: 'ambient' }), h('span', { text: 'slow, high pressure' })))),
      grp('v-aero-pressure', h('div', { class: 'v-row' }, h('span', { class: 'v-lbl' }, 'Pressure on the skin'), pmode), slider('Opacity', 0, 1, 0.05, o.opacity, (v) => { o.opacity = v; save(); }, (v) => Math.round(v * 100) + ' %'),
        toggle('Contour lines', o.contours, (v) => { o.contours = v; save(); S.cpSurf.mat.uniforms.uBands.value = v ? 1 : 0; }),
        h('div', { class: 'v-legend' }, legendCanvas(), h('div', { class: 'v-legend-ends' }, h('span', { text: 'suction' }), h('span', { text: 'ambient' }), h('span', { text: 'stagnation' })))),
      grp('v-aero-cfd', toggle('Live solver (zero angle of attack)', o.live, (v) => set('live', v), 'Solves the axisymmetric Euler equations on the graphics card as you watch, at the Mach number of the flight: a flow that settles in a second or two and follows the speed. The computed case below has the angle of attack and the 3D flow; this has the time.'),
        toggle('Section through the flow', o.csec, (v) => set('csec', v), 'A plane through the axis, in the plane of the angle of attack'), toggle('The solver\'s pressure on the skin', o.cskin, (v) => set('cskin', v)),
        h('div', { class: 'v-row' }, h('span', { class: 'v-lbl' }, 'Show'), seg([[0, 'Mach', 'The Mach number of the flow'], [1, 'Pressure', 'The pressure over the free stream\'s, logarithmic'], [2, 'Schlieren', 'The density\'s gradient: shocks and expansions as a shadowgraph shows them']], o.cmode, (v) => { o.cmode = +v; save(); }, 'quantity')),
        S.cfdDepth = slider('Section depth', 0, 1, 1, 0, (v) => S.cfd.slice(v), (v) => `${(v * (S.cfd.cfd ? S.cfd.cfd.head.volume.h : 0)).toFixed(1)} m off the plane`), slider('Opacity', 0.1, 1, 0.05, o.copac, (v) => { o.copac = v; save(); }, (v) => Math.round(v * 100) + ' %'),
        S.cfdPick = h('select', { 'aria-label': 'case', onchange: (e) => { o.ccase = e.target.value; save(); } }, h('option', { value: 'auto' }, 'The case nearest the flight')),
        h('div', { class: 'v-legend' }, legendCanvas(), h('div', { class: 'v-legend-ends' }, h('span', { text: 'slow / low' }), h('span', { text: '' }), h('span', { text: 'fast / high' })))),
      grp('v-aero-stability', toggle('Centre of pressure and centre of mass', o.cgcp, (v) => set('cgcp', v)), toggle('Forces: thrust, weight, aerodynamic, drag, lift', o.forces, (v) => set('forces', v))),
      grp('v-aero-extra', h('div', { class: 'v-lbl', style: { margin: '8px 0 2px' } }, 'Also draw, in this view'), toggle('Centre of pressure and centre of mass', o.xcgcp, (v) => set('xcgcp', v)), toggle('Forces', o.xforces, (v) => set('xforces', v))),
    ];
    S.says = { flow: 'The air going round the vehicle: streamlines, the shock, the airflow (the velocity of the vehicle through the air) and the angle of attack.', pressure: 'The pressure on the skin.', stability: 'Where the forces act and whether the air turns the vehicle back into the airflow.', cfd: 'The flow a solver computed (the Euler equations, tfc_cfd) for the case nearest this flight: the shock the nose makes, the expansion at its shoulder, the pressure on the skin. Not made up and not live: a computed case, at its Mach number and angle of attack.' };
    const mkChart = (series, opt = {}) => { const c = new Chart(h('canvas'), { height: 110, window: 60, series, ...opt }); return c; };
    S.charts = { alpha: mkChart([{ key: 'alpha', label: 'α °', color: 'var(--accent)' }], { zero: true }), q: mkChart([{ key: 'q', label: 'q Pa', color: 'var(--warn)' }]), mach: mkChart([{ key: 'mach', label: 'Mach', color: 'var(--ok)' }]),
      margin: mkChart([{ key: 'static_margin', label: 'static margin (cal)', color: 'var(--nC)' }], { zero: true }), pos: mkChart([{ key: 'cg', label: 'x CG m', color: 'var(--ok)' }, { key: 'cp', label: 'x CP m', color: 'var(--accent)' }]) };
    let chN = 0; const ch = (t, c) => card(t, [c.c, legend(c.o.series)], null, { open: chN++ === 0 });
    app.right.replaceChildren(
      card('What to draw', rows),
      card('Where the numbers come from', [provenance('measured', 'Mach, angle of attack, the air, the forces, the centre of mass and the centre of pressure: the simulator\'s own state.'),
        provenance('derived', 'the coefficients, Reynolds number, stagnation temperature and nose heating: formulas on those numbers (continuum formulas: they are not shown where the air is thinner than the vehicle is wide, Knudsen number above 1). The pressure map is the modified Newtonian estimate (valid above about Mach 3).'),
        provenance('illustrative', 'the streamlines (slender-body potential flow), the shock (Taylor-Maccoll cone shock) and the transonic cloud (humidity assumed). Not a flow solution.')]),
      ch('Angle of attack', S.charts.alpha), ch('Dynamic pressure', S.charts.q), ch('Mach number', S.charts.mach), ch('Static margin', S.charts.margin), ch('Centre of mass and centre of pressure along the vehicle', S.charts.pos));
    S.cpSurf.mat.uniforms.uBands.value = o.contours ? 1 : 0;
    this.fillCases();
    this.applyView(app, true);
  },

  applyView(app, camera = false) {
    const S = this.s, o = S.opts, v = o.view;
    S.streams.setVisible(o.streams && v === 'flow'); S.shock.setVisible(o.shock && v === 'flow'); S.cpSurf.setVisible(v === 'pressure' || (v === 'cfd' && o.cskin)); S.cfd.setVisible(v === 'cfd' && o.csec && !(o.live && S.live && S.live.ok)); if (S.liveSec) S.liveSec.setVisible(v === 'cfd' && o.csec && o.live); if (v !== 'cfd') S.cpSurf.setCfd(null);
    S.showMarks = v === 'stability' ? o.cgcp : o.xcgcp;                        // the markers and the dimension line
    S.showForces = v === 'stability' ? o.forces : o.xforces;
    S.showWind = v === 'flow' && o.wind; S.showArc = v === 'flow' && o.arc; S.showCond = v === 'flow' && o.cond;
    const show = (id, on) => { const el = document.getElementById(id); if (el) el.style.display = on ? '' : 'none'; };
    show('v-aero-flow', v === 'flow'); show('v-aero-pressure', v === 'pressure'); show('v-aero-stability', v === 'stability'); show('v-aero-extra', v !== 'stability' && v !== 'cfd'); show('v-aero-cfd', v === 'cfd'); show('v-aero-cfdcard', v === 'cfd');
    const says = document.getElementById('v-aero-says'); if (says) says.textContent = S.says[v] || '';
    if (camera) {
      const r = app.world.rig;
      if (v === 'stability') { r.set('orbit'); r.yaw = Math.PI / 2; r.pitch = 0.0; r.dist = 1.95; r.anchor = 0.5; }
      else if (v === 'flow') { r.set('orbit'); r.yaw = 0.9; r.pitch = 0.12; r.dist = 1.5; r.anchor = 0.55; }
      else if (v === 'cfd') { r.set('orbit'); r.yaw = Math.PI / 2; r.pitch = 0.05; r.dist = 1.35; r.anchor = 0.78; }
      else { r.set('orbit'); r.yaw = 0.7; r.pitch = 0.15; r.dist = 1.1; r.anchor = 0.6; }
    }
  },

  update(app, pose, d, dt) {
    const S = this.s, o = S.opts, { world, vehicle, spec } = app, g = S.group;
    const ext = spec.extent(pose.stg, pose.pay), Lv = ext.length, R = ext.diameter / 2;      // what is still there: the ship, after the booster has left
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
      const show = S.showWind && d.V > 5;
      const inAir = d.air;
      a.a.object.visible = a.l.object.visible = show;
      if (show) { const Ln = Lv * 0.3, start = new THREE.Vector3(0, ext.x1, 0).addScaledVector(f, -Ln * 1.15 - R * 1.5); a.a.set(start, f, Ln, Math.max(0.2, R * 0.08)); a.l.setText(inAir ? `AIRFLOW ${d.V.toFixed(0)} m/s` : `VELOCITY ${d.V.toFixed(0)} m/s (no air)`); a.l.object.position.copy(start); }
    }
    // the angle of attack: an arc at the CG between the axis and the velocity through the air
    {
      const vhat = new THREE.Vector3(d.vhat[2], d.vhat[0], d.vhat[1]), ax = new THREE.Vector3(0, 1, 0), r0 = Lv * 0.18;
      const show = S.showArc && d.alpha > 0.15 && d.V > 5;
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
    { const on = o.view === 'flow' && d.air; S.streams.setVisible(o.streams && on); S.shock.setVisible(o.shock && on); }   // no streamlines and no shock where the air is too thin to flow round the vehicle
    S.cpSurf.update(d, mode === 2 ? 2 : 0, o.opacity);
    S.streams.update(d, pose.stg ?? 255, spec, app.wall, dt);
    const segs = S.streams.profile(pose.stg ?? 255);
    if (segs.length && S.shock.visible) { const tip = segs[segs.length - 1][2]; S.shock.update(d, tip, S.streams.noseAngle(segs), Math.min(Lv * 0.45, 55), app.wall); }
    S.cond.update({ ...d, alt: pose.alt }, ext.x0 + ext.length * 0.86, R, world.camera, [1, 1, 1].map((x, i) => (world.sunT ? world.sunT[i] : 1)), S.showCond && d.air, app.wall);

    // the solver's flow: the case nearest this flight, loaded when it changes
    if (view === 'cfd') {
      const cs = S.cfdState, stageFrom = (() => { const m = pose.stg ?? 255; for (let i = 0; i < 8; i++) if (m & (1 << i)) return i; return 0; })();
      const want = o.ccase !== 'auto' ? cs.list.find((c) => c.name === o.ccase) : nearest(cs.list.filter((c) => !c.vehicle || c.vehicle === spec.name), spec.name, stageFrom, d.M, d.alpha);
      if (want && want.name !== S.cfd.name && cs.loading !== want.name && cs.failed !== want.name) {
        cs.loading = want.name;
        loadCase(want.name).then((c) => { if (this.s !== S) return; S.cfd.set(c, want.name); S.cfdDepth.input.max = String(S.cfd.nz - 1); S.cfdDepth.input.value = '0'; cs.loading = ''; }).catch(() => { cs.failed = want.name; cs.loading = ''; });
      }
      const cf = S.cfd.cfd;
      S.cfd.update({ mode: o.cmode, opacity: o.copac, phiDeg: d.phi });
      this.liveStep(app, pose, d, dt);
      S.cpSurf.setCfd(cf && o.cskin ? S.cfd.skinTex : null, cf ? cf.head.body.x0 : 0, cf ? cf.head.body.x1 : 1, (d.phi || 0) * Math.PI / 180);
      const st = S.cfdStats;
      if (cf) {
        const H = cf.head, a = H.alpha_deg;
        st.case.set(`M ${H.mach.toFixed(1)}, ${a.toFixed(0)}°`); st.cells.set(H.cells >= 1e6 ? (H.cells / 1e6).toFixed(2) + 'M' : Math.round(H.cells / 1e3) + 'k');
        st.ca.set(H.ca_skin.toFixed(3)); st.cn.set(H.cn.toFixed(3)); st.cna.set(a > 0 ? (H.cn / (a * Math.PI / 180)).toFixed(2) : '—'); st.xcp.set(a > 0 ? H.x_cp_m.toFixed(1) : '—');
        st.simcna.set((pose.cna ?? 0).toFixed(2)); st.simcp.set((pose.cp ?? 0).toFixed(1));
        S.cfdNote.textContent = `Flight: Mach ${d.M.toFixed(2)}, ${d.alpha.toFixed(1)}°. Solver: Mach ${H.mach.toFixed(2)}, ${a.toFixed(1)}° (${H.iterations} iterations, residual down ${H.residual_drop.toFixed(1)} orders, ${H.h_m} m cells). ${S.liveShown ? `The section is the live solver\'s, axisymmetric at no angle of attack, at Mach ${S.live.mach.toFixed(2)}; the skin pressure and these numbers are the computed case\'s.` : 'The section is the computed case\'s.'}`;
      } else {
        st.case.set('none'); S.cfdNote.textContent = cs.list.length ? (cs.loading ? 'Loading the flow field…' : 'No case for this vehicle and these stages.') : 'No flow fields on the server: make some with tools/cfd/make_library.py (console/cfd/README.md).';
      }
    }
    // the readouts
    const st = S.stats;
    st.mach.set(d.M.toFixed(2)); st.q.set((d.q / 1000).toFixed(1)); st.alpha.set(d.alpha.toFixed(2)); st.alt.set((pose.alt / 1000).toFixed(1)); st.v.set(d.V.toFixed(0));
    if (d.air) { st.re.set(fmt.eng(d.Re, 1)); st.t0.set(d.T0.toFixed(0), d.T0 > 1500 ? 'warn' : ''); st.heat.set((d.heat / 1000).toFixed(d.heat > 1e5 ? 0 : 1), d.heat > 5e5 ? 'warn' : ''); }
    else { st.re.set('no air'); st.t0.set('–'); st.heat.set('–'); }          // a continuum formula on a few molecules would print a number with nothing behind it
    st.cn.set(d.cn.toFixed(3)); st.ca.set(d.ca.toFixed(3)); st.cd.set(d.cd.toFixed(3)); st.cl.set(d.cl.toFixed(3)); st.normal.set((d.N / 1000).toFixed(1)); st.drag.set((d.D / 1000).toFixed(1)); st.ld.set(d.ld.toFixed(2));
    const mom = Math.hypot((pose.mae || [0, 0, 0])[1], (pose.mae || [0, 0, 0])[2]); st.mom.set((mom / 1e6).toFixed(2));
    S.regime.textContent = `${d.regime}${d.mu ? `, Mach angle ${d.mu.toFixed(1)}°` : ''}; ${d.layer.name}`;
    S.stab.margin.set(d.margin.toFixed(2), d.margin >= 0 ? 'ok' : 'warn'); S.stab.gap.set(Math.abs(gap).toFixed(1)); S.stab.xcg.set(pose.cg.toFixed(1)); S.stab.xcp.set(pose.cp.toFixed(1));
    S.verdict.className = 'v-verdict ' + (stable ? 'ok' : 'warn');
    S.verdict.textContent = d.q < 200 ? 'Almost no air: the aerodynamic forces are negligible; the position of the centre of pressure is the vehicle\'s own shape.' : stable ? 'Statically stable: the air turns the vehicle back into the wind.' : 'Statically unstable: the air turns the vehicle away from the wind. The gimbals hold it.';
    declutter([S.cpL, S.cgL, S.dimL, A.aero.l, A.thrust.l, A.weight.l, A.drag.l, A.lift.l, A.wind.l, S.arcL], world.camera, innerWidth, innerHeight);
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
    const ext = spec.extent(pose.stg, pose.pay), L = ext.length, pad = 16, scale = (H - 2 * pad) / L, cx = W * 0.42;
    const Y = (x) => H - pad - (x - ext.x0) * scale;
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

  /** The live solver: made when it is first wanted, rebuilt when the stages on the vehicle change, run for a few iterations a frame at the flight's Mach number, and drawn as the section. */
  liveStep(app, pose, d, dt) {
    const S = this.s, o = S.opts, { world, spec } = app;
    const want = o.live && o.csec;
    S.liveShown = false;
    if (!want) { if (S.liveSec) S.liveSec.setVisible(false); S.cfd.setVisible(o.csec); return; }
    const mask = pose.stg ?? 255;
    if (S.live && S.liveMask !== mask) { S.liveSec.dispose(); S.live.dispose(); S.live = null; S.liveSec = null; }     // a stage has gone: another body
    if (!S.live) {
      const ext = spec.extent(mask, pose.pay), L = ext.length, D = ext.diameter, span = L + 2.5 * D;
      const h = Math.max(Math.min(span / 640, D / 28), span / 1600), nx = Math.ceil(span / h), nr = Math.min(400, Math.ceil(4 * D / h));
      const prof = []; app.vehicle.stages.forEach((st, i) => { if (mask & (1 << i)) for (const q of st.profile) prof.push({ x: q.x, r: q.r }); });
      prof.sort((a, b) => a.x - b.x);
      S.live = new GpuFlow(world.renderer, { nx, nr, h, x0: ext.x0 - D });
      S.liveMask = mask;
      if (!S.live.ok) { S.liveNote = 'This browser cannot render to float textures: no live solver.'; return; }
      S.live.setBody(prof, ext.x0 - D);
      S.liveSec = new GpuSection(S.group, S.live);
      S.liveSec.setVisible(true);
    }
    if (!S.live.ok) return;
    const supersonic = d.air && d.M > 1.25;
    S.liveShown = supersonic;
    S.liveSec.setVisible(supersonic);
    S.cfd.setVisible(!supersonic && o.csec);
    if (!supersonic) return;
    const m = Math.min(d.M, 25);
    if (Math.abs(m - S.live.mach) > 0.01 * S.live.mach) { if (Math.abs(m / S.live.mach - 1) > 0.5) S.live.reset(); S.live.setMach(m); }
    S.liveIts = Math.max(8, Math.min(120, (S.liveIts || 40) * (dt > 0.03 ? 0.8 : 1.05)));         // as many iterations a frame as the frame can take
    S.live.step(Math.round(S.liveIts));
    S.liveSec.update({ mode: o.cmode, opacity: o.copac, phiDeg: d.phi });
  },

  unmount(app) {
    const S = this.s; if (!S) return;
    if (S.liveSec) S.liveSec.dispose(); if (S.live) S.live.dispose(); S.cpSurf.dispose(); S.cfd.dispose(); S.streams.dispose(); S.shock.dispose(); S.cond.dispose();
    S.group.removeFromParent();
    this.s = null;
  },
};
