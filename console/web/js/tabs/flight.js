// Flight: where the vehicle is and how it is flying. The numbers come from the simulator itself (`tfc_simd --telemetry`, "truth"), not from a sensor; the bus carries a coarser copy (10 Hz altitude, speed, mass,
// dynamic pressure, attitude error) which is used when the truth is not there. The dashed curve is the nominal flight of the same vehicle (`tfc_fly`, the real flight software, no departures).
import { h, svg, setText, num, eng, store } from '../util.js';
import { Chart, Trajectory, legend } from '../charts.js';
import { api } from '../net.js';

let R = { nominal: null, nominalState: 'idle' };

function rocket() {
  const root = svg('svg', { viewBox: '0 0 200 300', class: 'diagram', role: 'img', 'aria-label': 'The vehicle: stages, engines, attitude and gimbal', style: 'max-height:330px' });
  R.pad = svg('g', {}, svg('line', { x1: 10, y1: 270, x2: 190, y2: 270, stroke: 'var(--line2)', 'stroke-width': 2 }), svg('text', { x: 12, y: 286, fill: 'var(--faint)', 'font-size': 10 }, 'pad'));
  root.append(R.pad);
  R.rk = svg('g', {});
  R.rkTilt = svg('g', { transform: 'translate(100 200)' });
  R.stageEls = [];
  for (let i = 0; i < 4; i++) {
    const g = svg('g', {});
    const body = svg('rect', { x: -22, y: -(i + 1) * 60 + 60, width: 44, height: 56, rx: 6, fill: 'var(--panel3)', stroke: 'var(--line2)', 'stroke-width': 2 });
    const fill = svg('rect', { x: -22, y: 0, width: 44, height: 0, rx: 6, fill: 'var(--info)', opacity: .55 });
    const lab = svg('text', { x: 0, y: -(i + 1) * 60 + 60 + 33, 'text-anchor': 'middle', fill: 'var(--text)', 'font-size': 13, 'font-weight': 700 }, `S${i + 1}`);
    g.append(body, fill, lab); R.rkTilt.append(g); R.stageEls.push({ g, body, fill, lab, top: -(i + 1) * 60 + 60 });
  }
  R.nose = svg('path', { d: 'M-22 -240 L0 -278 L22 -240z', fill: 'var(--panel3)', stroke: 'var(--line2)', 'stroke-width': 2, transform: 'translate(0 0)' });
  R.flameG = svg('g', { transform: 'translate(0 60)' });
  R.flame = svg('path', { d: 'M-12 0 L0 42 L12 0z', fill: 'var(--warn)', opacity: 0 });
  R.flameG.append(R.flame);
  R.rkTilt.append(R.nose, R.flameG);
  root.append(R.rkTilt);
  return root;
}

function nominalLoad() {
  if (R.nominalState !== 'idle') return;
  R.nominalState = 'loading';
  api('/api/vehicle/nominal').then(async ({ job }) => {
    for (let i = 0; i < 120; i++) {
      await new Promise((r) => setTimeout(r, 700));
      const j = await api(`/api/job?id=${job}`);
      if (j.state === 'done') { const c = j.result.columns; R.nominal = c.t_s ? { t: c.t_s, alt: c.altitude_m, range: c.range_m, speed: c.speed_ms, q: c.dynamic_pressure_pa } : null; R.nominalState = R.nominal ? 'ready' : 'none'; return; }
      if (j.state === 'failed') { R.nominalState = 'none'; return; }
    }
  }).catch(() => { R.nominalState = 'none'; });
}
const interp = (xs, ys, x) => { if (!xs || !xs.length || x < xs[0] || x > xs[xs.length - 1]) return null; let lo = 0, hi = xs.length - 1; while (hi - lo > 1) { const m = (lo + hi) >> 1; if (xs[m] <= x) lo = m; else hi = m; } const f = (x - xs[lo]) / ((xs[hi] - xs[lo]) || 1); return ys[lo] + f * (ys[hi] - ys[lo]); };

export default {
  id: 'flight', label: 'Flight', icon: 'flight',
  mount(root) {
    R.traj = new Trajectory(h('canvas'), 400);
    R.view = store.get('flight.view', 'whole');
    const viewSeg = h('div', { class: 'seg', role: 'group', 'aria-label': 'Trajectory scale' }, [['whole', 'Whole flight'], ['follow', 'Follow vehicle']].map(([v, t]) => h('button', { type: 'button', 'aria-pressed': v === R.view ? 'true' : 'false', onclick: (e) => { R.view = v; store.set('flight.view', v); for (const b of viewSeg.children) b.setAttribute('aria-pressed', b === e.currentTarget ? 'true' : 'false'); } }, t)));
    R.rocket = rocket();
    R.tiltTxt = h('div', { class: 'note', style: { textAlign: 'center', marginTop: '4px' } });
    R.win = store.get('flight.window', 120);
    const mk = (series, opt = {}) => { const c = new Chart(h('canvas'), { height: 150, window: R.win, series, ...opt }); return c; };
    R.charts = {
      alt: mk([{ key: 't_alt', label: 'altitude m', color: 'var(--accent)' }, { key: 'alt', label: 'altitude (bus)', color: 'var(--faint)', dash: [3, 3] }]),
      speed: mk([{ key: 't_speed', label: 'speed m/s', color: 'var(--ok)' }, { key: 'speed', label: 'speed (bus)', color: 'var(--faint)', dash: [3, 3] }]),
      q: mk([{ key: 't_q', label: 'q Pa', color: 'var(--warn)' }, { key: 'q', label: 'q (bus)', color: 'var(--faint)', dash: [3, 3] }]),
      err: mk([{ key: 'err_p', label: 'pitch error °', color: 'var(--nA)' }, { key: 'err_y', label: 'yaw error °', color: 'var(--nC)' }], { zero: true }),
      gim: mk([{ key: 'act_p', label: 'ACT pitch °', color: 'var(--nACT)' }, { key: 'act_y', label: 'ACT yaw °', color: 'var(--nSIM)' }, { key: 't_gim_p', label: 'gimbal pitch (actual)', color: 'var(--accent)', dash: [4, 3] }], { zero: true }),
      tilt: mk([{ key: 't_tilt_p', label: 'pitch tilt °', color: 'var(--info)' }, { key: 't_ref_p', label: 'program °', color: 'var(--faint)', dash: [5, 4] }]),
      mass: mk([{ key: 't_mass', label: 'mass kg', color: 'var(--nB)' }, { key: 'mass', label: 'mass (bus)', color: 'var(--faint)', dash: [3, 3] }]),
      thrust: mk([{ key: 't_thrust', label: 'thrust N', color: 'var(--crit)' }]),
    };
    const card = (title, ch, sub) => h('article', { class: 'card' }, h('header', {}, h('h3', {}, title), h('div', { class: 'tools' }, sub || '')), h('div', { class: 'body' }, ch.c, legend(ch.o.series)));
    const winSeg = h('div', { class: 'seg', role: 'group', 'aria-label': 'Chart window' }, [30, 120, 300].map((w) => h('button', { type: 'button', 'aria-pressed': w === R.win ? 'true' : 'false', onclick: (e) => { R.win = w; store.set('flight.window', w); for (const c of Object.values(R.charts)) c.setWindow(w); for (const b of winSeg.children) b.setAttribute('aria-pressed', b === e.currentTarget ? 'true' : 'false'); } }, w < 60 ? `${w} s` : `${w / 60} min`)));
    R.stats = h('div', { class: 'stats', style: { gridTemplateColumns: 'repeat(auto-fill, minmax(92px, 1fr))' } });
    R.stages = h('table', {}, h('thead', {}, h('tr', {}, h('th', {}, 'stage'), h('th', {}, 'state'), h('th', { class: 'num' }, 'propellant kg'), h('th', {}, ''))), h('tbody'));
    R.flags = h('div', { class: 'stack', style: { gap: '6px' } });
    R.nomNote = h('div', { class: 'note' });
    R.legend = h('div', { class: 'legend' }, h('span', {}, h('i', { style: { background: 'var(--accent)' } }), 'flown'), h('span', {}, h('i', { style: { background: 'var(--faint)' } }), 'nominal (tfc_fly)'));
    root.append(h('div', { class: 'stack' }, R.flags,
      h('div', { class: 'grid c-3-2', style: { alignItems: 'start' } },
        h('article', { class: 'card' }, h('header', {}, h('h3', {}, 'Trajectory'), h('div', { class: 'tools' }, R.legend, viewSeg)), h('div', { class: 'body' }, R.traj.c, R.nomNote)),
        h('article', { class: 'card' }, h('header', {}, h('h3', {}, 'The vehicle')), h('div', { class: 'body', style: { display: 'grid', gridTemplateColumns: '150px 1fr', gap: '10px', alignItems: 'center' } }, h('div', {}, R.rocket, R.tiltTxt), h('div', {}, R.stats)))),
      h('div', { class: 'grid c-2-1' },
        h('article', { class: 'card' }, h('header', {}, h('h3', {}, 'Stages')), h('div', { class: 'body flush' }, R.stages)),
        h('article', { class: 'card' }, h('header', {}, h('h3', {}, 'Time window')), h('div', { class: 'body' }, winSeg, h('p', { class: 'note', style: { marginBottom: 0 } }, 'Solid lines are the simulator’s own state (UDP, 10 Hz). Dotted grey lines are the copy the bus carries (frames 0x503 to 0x505), which is what the flight computers could have known.')))),
      h('div', { class: 'grid cols2' }, card('Altitude', R.charts.alt), card('Speed', R.charts.speed), card('Dynamic pressure', R.charts.q), card('Attitude error against the program', R.charts.err),
        card('Gimbal: ACT’s command', R.charts.gim), card('Pitch tilt and the pitch program', R.charts.tilt), card('Mass', R.charts.mass), card('Thrust', R.charts.thrust))));
  },
  update(S) {
    const s = S.snap; if (!s) return;
    nominalLoad();
    const t = s.truth, b = s.sim;
    // banners
    const fl = [];
    if (t && t.crashed) fl.push(['crit', 'The vehicle was destroyed.']);
    if (b && b.flag_names.length) for (const f of b.flag_names) fl.push([f === 'engine out' || f === 'platform saturated' || f === 'command held' ? 'warn' : 'crit', `Simulator flag: ${f}.`]);
    const fk = JSON.stringify(fl);
    if (R.fk !== fk) { R.fk = fk; R.flags.innerHTML = ''; for (const [lv, tx] of fl) R.flags.append(h('div', { class: `banner ${lv}` }, tx)); }
    if (!t && !b) { R.stats.innerHTML = ''; R.stats.append(h('div', { class: 'empty' }, 'No simulator is running. Start the rig on the Rig tab.')); R.sk = null; }
    // the trail from the history
    const H = S.hist, n = H.t.length;
    const rr = H.s.t_range || [], aa = H.s.t_alt || [];
    const trail = { range: [], alt: [] };
    for (let i = 0; i < n; i++) if (rr[i] != null && aa[i] != null && (H.s.t_ft || [])[i] != null && (H.s.t_ft[i] > 0 || trail.range.length)) { trail.range.push(rr[i]); trail.alt.push(aa[i]); }
    const nominal = R.nominal ? { range: R.nominal.range, alt: R.nominal.alt } : null;
    let now = null;
    if (trail.range.length > 1) { const k = trail.range.length - 1; now = { range: trail.range[k], alt: trail.alt[k], angle: Math.atan2(trail.range[k] - trail.range[Math.max(0, k - 5)], Math.max(1, trail.alt[k] - trail.alt[Math.max(0, k - 5)])) }; }
    else if (t) now = { range: t.range, alt: t.alt, angle: 0 };
    const marks = [];
    for (const e of S.events) {
      if (!['max-q', 'stage-separation', 'stage-ignition', 'liftoff'].includes(e.kind)) continue;
      let bi = -1, bd = 1e9; for (let i = 0; i < n; i++) { const d = Math.abs(H.t[i] - e.t); if (d < bd) { bd = d; bi = i; } }
      if (bi >= 0 && bd < 1.5 && rr[bi] != null) marks.push({ range: rr[bi], alt: aa[bi], label: e.kind === 'max-q' ? 'max-Q' : e.kind === 'liftoff' ? 'lift-off' : e.text.replace(/^\[\w+\] /, '').replace(/ at T.*/, ''), color: e.kind === 'max-q' ? 'var(--warn)' : 'var(--info)' });
    }
    R.traj.draw({ nominal, trail: trail.range.length ? trail : null, marks, now, view: R.view });
    setText(R.nomNote, R.nominalState === 'ready' ? `Nominal: the reference vehicle flown by tfc_fly with the real flight software, no departures, platform sensors (${(R.nominal.alt[R.nominal.alt.length - 1] / 1000).toFixed(1)} km, ${(R.nominal.range[R.nominal.range.length - 1] / 1000).toFixed(1)} km downrange at the end).` : R.nominalState === 'loading' ? 'Computing the nominal flight…' : 'The nominal flight is not available (build/host/tfc_fly is not built).');
    // the rocket
    if (t) {
      const stages = R.stageEls;
      const nst = (t.prop || []).length;
      const maxp = (R.maxProp ||= []);
      (t.prop || []).forEach((p, i) => { maxp[i] = Math.max(maxp[i] || 0, p); });
      stages.forEach((st, i) => {
        const on = i < nst && ((t.stages_active >> i) & 1);
        st.g.style.display = i < nst ? '' : 'none';
        st.body.setAttribute('opacity', on ? 1 : .25);
        const frac = i < nst && maxp[i] > 0 ? t.prop[i] / maxp[i] : 0;
        st.fill.setAttribute('x', -22); st.fill.setAttribute('width', 44);
        st.fill.setAttribute('height', 56 * frac); st.fill.setAttribute('y', st.top + 56 * (1 - frac));
        st.body.setAttribute('stroke', ((t.stages_ignited >> i) & 1) && on ? 'var(--warn)' : 'var(--line2)');
      });
      const top = -(Math.max(1, nst) - 1) * 60;                           // the top edge of the highest stage drawn: the nose sits on it
      R.nose.setAttribute('d', `M-22 ${top} L0 ${top - 38} L22 ${top}z`);
      R.flame.setAttribute('opacity', !t.clamped && t.engines_on > 0 && t.thrust > 0 ? .9 : 0);          // the vehicle is held until T-zero: no flame on the pad
      R.flameG.setAttribute('transform', `translate(0 56) rotate(${-(t.gim_p || 0) * 3})`);   // the flame leans with the gimbal (drawn three times larger than it is)
      R.pad.style.display = t.clamped ? '' : 'none';                       // the pad is only there until the vehicle leaves it
      R.rkTilt.setAttribute('transform', `translate(100 ${t.clamped ? 214 : 190}) rotate(${t.tilt_p})`);
      setText(R.tiltTxt, `pitch tilt ${t.tilt_p.toFixed(1)}° · gimbal ${t.gim_p.toFixed(2)}°`);
    } else setText(R.tiltTxt, 'no simulator telemetry');
    // the numbers
    const dnom = (R.nominal && t && t.ft > 0) ? { alt: interp(R.nominal.t, R.nominal.alt, t.ft), speed: interp(R.nominal.t, R.nominal.speed, t.ft) } : null;
    const rows = t ? [
      ['T+', t.ft.toFixed(1), 's'], ['Altitude', eng(t.alt, 1), 'm'], ['Range', eng(t.range, 1), 'm'], ['Speed', t.speed.toFixed(1), 'm/s'], ['Mach', t.mach.toFixed(2), ''], ['Dynamic pressure', (t.q / 1000).toFixed(2), 'kPa'],
      ['Mass', eng(t.mass, 1), 'kg'], ['Thrust', t.clamped ? 'held' : (t.thrust / 1000).toFixed(0), t.clamped ? '' : 'kN'], ['Engines', t.clamped ? 'on the pad' : `${t.engines_on}/${t.engines}`, ''], ['Pitch tilt', t.tilt_p.toFixed(2), '°'], ['Program', t.ref_p.toFixed(2), '°'],
      ['Tilt − program', (t.tilt_p - t.ref_p).toFixed(2), '°'], ['Gimbal pitch', t.gim_p.toFixed(2), '°'], ['Gimbal yaw', t.gim_y.toFixed(2), '°'],
      ...(dnom && dnom.alt != null ? [['Alt − nominal', eng(t.alt - dnom.alt, 1), 'm'], ['Speed − nominal', (t.speed - dnom.speed).toFixed(1), 'm/s']] : []),
    ] : b ? [['Altitude', eng(b.alt, 1), 'm'], ['Speed', b.speed.toFixed(0), 'm/s'], ['Mass', eng(b.mass, 1), 'kg'], ['Dyn. pressure', b.q != null ? (b.q / 1000).toFixed(2) : '—', 'kPa'], ['Pitch error', b.err_p, '°'], ['Yaw error', b.err_y, '°']] : [];
    const sk = JSON.stringify(rows.map((r) => r[0]));
    if (R.sk !== sk) { R.sk = sk; R.stats.innerHTML = ''; R.vals = rows.map((r) => { const v = h('div', { class: 'v', style: { fontSize: '15px' } }); R.stats.append(h('div', { class: 'stat' }, h('div', { class: 'k' }, r[0]), v)); return v; }); }
    rows.forEach((r, i) => { R.vals[i].textContent = ''; R.vals[i].append(String(r[1]), h('small', {}, r[2])); });
    // stages table
    const tb = R.stages.querySelector('tbody'); tb.innerHTML = '';
    if (t && t.prop) t.prop.forEach((p, i) => {
      const act = (t.stages_active >> i) & 1, ign = (t.stages_ignited >> i) & 1;
      const state = !act ? 'separated' : t.clamped ? 'on the pad' : ign ? (t.engines_on > 0 && i === (R.stageEls.findIndex((_, k) => (t.stages_active >> k) & 1)) ? 'burning' : 'ignited') : 'waiting';
      const frac = (R.maxProp || [])[i] ? p / R.maxProp[i] : 0;
      tb.append(h('tr', {}, h('td', {}, `Stage ${i + 1}`), h('td', {}, h('span', { class: `chip ${state === 'burning' ? 'warn' : state === 'separated' ? 'muted' : state === 'ignited' ? 'info' : 'muted'}` }, state)), h('td', { class: 'num' }, eng(p, 0)), h('td', { style: { width: '35%' } }, h('div', { class: 'progress' }, h('i', { style: { width: `${(frac * 100).toFixed(0)}%` } })))));
    });
    else tb.append(h('tr', {}, h('td', { colspan: 4, class: 'empty' }, 'No simulator telemetry.')));
    for (const c of Object.values(R.charts)) c.draw(H);
  },
};
