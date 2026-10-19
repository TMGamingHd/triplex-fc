// Broadcast: the flight as a webcast shows it. The picture is the whole screen; over it, at the bottom, one block for each stage (its speed, its altitude and its engines), the mission clock between them, and the
// event that has just happened in a line above the clock. Nothing else is on the screen: the panels and the bars are away until the pointer moves, and go again after three seconds of stillness.
//
// It reads the flight and does not change it. The numbers are the pose's own (the speed through the planet's frame and the height above the ground, for the stage's own body: a stage that has been let go has a pose of its own),
// the engines are the simulator's per-engine state (on, off or failed), and the events are the ones the page derives from the poses (main.js `track`). The units are the viewer's two: metric, or the miles an American
// broadcast says; click a number to change them.
import { h, store } from '../ui.js';
import { createGyroBox } from '../gyro.js';

const CALL = [                                              // what the page's event texts are called on the screen
  [/^lift-off/, 'LIFTOFF'], [/^max-Q/, 'MAX Q'], [/ separated$/, (m, t) => t.replace(/ separated$/, '').toUpperCase() + ' SEPARATION'], [/ ignited$/, (m, t) => t.replace(/ ignited$/, '').toUpperCase() + ' IGNITION'],
  [/^engines off/, 'ENGINE CUTOFF'], [/^passed 100 km/, 'KARMAN LINE'], [/ caught$/, (m, t) => t.replace(/ caught$/, '').toUpperCase() + ' CAUGHT'], [/ jettisoned$/, (m, t) => t.replace(/ jettisoned$/, '').toUpperCase() + ' DEPLOYED'],
  [/^engine \d+ failed/, (m, t) => t.toUpperCase()], [/^the vehicle was destroyed/, 'VEHICLE LOST'],
];
export const callName = (text) => { for (const [re, to] of CALL) { const m = re.exec(text); if (m) return typeof to === 'function' ? to(m, text) : to; } return text.toUpperCase(); };
const MARK = (text) => /lift-off|max-Q|separated|ignited|engines off|caught|100 km|jettisoned|failed|destroyed/.test(text);

const hms = (sec) => { const s = sec < 0 ? '−' : '+', a = Math.floor(Math.abs(sec)); return `T${s} ${String(Math.floor(a / 3600)).padStart(2, '0')}:${String(Math.floor(a / 60) % 60).padStart(2, '0')}:${String(a % 60).padStart(2, '0')}`; };
const group = (n) => Math.round(n).toLocaleString('en-US');

/** The stages of the flight and, for each, the body that has it now: the main body's pose while its bit is set, the body of its own after it has been let go, nothing if it is gone. */
export function stageViews(app, raw) {
  const spec = app.data.spec, out = [];
  spec.stages.forEach((st, i) => {
    const name = (st.name || 'stage ' + (i + 1)).toUpperCase();
    let pose = null, engines = [];
    if (((raw.stg ?? 255) >> i) & 1) { pose = raw; engines = spec.engines.map((e, k) => (e.stage === i ? k : -1)).filter((k) => k >= 0).map((k) => ({ k, e: spec.engines[k] })); }
    else {
      const o = (raw.others || []).find((x) => x.stage === i && x.pose);
      if (o) { pose = o.pose; engines = app.data.specFor(o).engines.map((e, k) => ({ k, e })); }
    }
    out.push({ i, name, pose, engines });
  });
  return out;
}

function engineCanvas() {
  const c = h('canvas', { class: 'cast-eng', width: 96, height: 96, 'aria-hidden': 'true' }); c.dataset.sig = '';
  return c;
}
function drawEngines(c, engines, states) {
  const sig = states.map((s) => (s < 0 ? 'x' : s > 0.02 ? 'o' : '.')).join('') + engines.length;
  if (c.dataset.sig === sig) return; c.dataset.sig = sig;
  const dpr = Math.min(2, window.devicePixelRatio || 1), W = c.clientWidth || 48; if (c.width !== Math.round(W * dpr)) { c.width = c.height = Math.round(W * dpr); }
  const ctx = c.getContext('2d'); ctx.setTransform(c.width / W, 0, 0, c.width / W, 0, 0); ctx.clearRect(0, 0, W, W);
  if (!engines.length) return;
  const pos = engines.map(({ e }) => [e.position_m[2] || 0, -(e.position_m[1] || 0)]);
  const R = Math.max(0.5, ...pos.map((p) => Math.hypot(p[0], p[1]))), s = (W / 2 - 4) / R, dot = Math.max(1.6, Math.min(3.6, W / (2.8 * Math.sqrt(engines.length) + 3)));
  engines.forEach((en, n) => {
    const x = W / 2 + pos[n][0] * s, y = W / 2 + pos[n][1] * s, st = states[n];
    ctx.beginPath(); ctx.arc(x, y, dot, 0, 7);
    if (st < 0) { ctx.strokeStyle = '#ff5d73'; ctx.lineWidth = 1.2; ctx.stroke(); ctx.beginPath(); ctx.moveTo(x - dot, y - dot); ctx.lineTo(x + dot, y + dot); ctx.stroke(); }
    else { ctx.fillStyle = st > 0.02 ? '#ffffff' : 'rgba(255,255,255,.2)'; ctx.fill(); }
  });
}

export default {
  id: 'cast', label: 'Broadcast', key: '0',
  mount(app) {
    const S = (this.s = { units: store.get('cast.units', 'metric'), blocks: [], seen: 0, shownAt: -1e9, gyroOn: store.get('cast.gyro', true), eventsKey: '' });
    document.body.classList.add('cast');
    app.left.replaceChildren(); app.right.replaceChildren();
    const unitsBtn = (e) => { S.units = S.units === 'metric' ? 'imperial' : 'metric'; store.set('cast.units', S.units); e.stopPropagation(); };
    const num = (label) => { const v = h('b', {}, '—'), u = h('small', {}, ''); return { v, u, el: h('div', { class: 'cast-num', title: 'Click to change the units', onclick: unitsBtn }, h('span', { class: 'cast-k' }, label), h('div', { class: 'cast-v' }, v, u)) }; };
    const spec = app.data.spec;
    const shown = spec.stages.length > 2 ? [0, spec.stages.length - 1] : spec.stages.map((_, i) => i);
    S.blocks = shown.map((i) => {
      const sp = num('SPEED'), al = num('ALTITUDE'), eng = engineCanvas(), cnt = h('span', { class: 'cast-count' }, ''), name = h('div', { class: 'cast-name' }, h('span', {}, (spec.stages[i].name || 'stage').toUpperCase()), cnt);
      const el = h('section', { class: 'cast-stage', dataset: { i } }, h('div', { class: 'cast-eng-wrap' }, eng), h('div', { class: 'cast-info' }, name, h('div', { class: 'cast-nums' }, sp.el, al.el)));
      return { i, el, sp, al, eng, name, cnt };
    });
    S.clock = h('div', { class: 'cast-clock' }, 'T+ 00:00:00');
    S.call = h('div', { class: 'cast-call', 'aria-live': 'polite' }, '');
    S.track = h('div', { class: 'cast-track' }, h('i', { class: 'cast-line' }), h('i', { class: 'cast-play' }), S.marks = h('div', { class: 'cast-marks' }));
    S.last = h('div', { class: 'cast-last' }, '');
    const mid = h('section', { class: 'cast-mid' }, S.call, S.clock, S.track, S.last);
    const brand = h('div', { class: 'cast-brand' }, h('span', { class: 'cast-logo' }, 'TFC'), h('span', { class: 'cast-sub' }, spec.name), S.live = h('span', { class: 'cast-live' }, ''));
    S.gyro = createGyroBox(app, h); S.gyroWrap = h('div', { class: 'cast-gyro' + (S.gyroOn ? '' : ' off'), title: 'The attitude as a gyroscope: the outer ring turns about the vertical, the middle about the horizontal axis, the inner about the vehicle\'s own axis (G hides it)' }, S.gyro.el);
    const bar = h('div', { class: 'cast-bar' }, S.blocks[0] ? S.blocks[0].el : h('section'), mid, S.blocks[1] ? S.blocks[1].el : h('section', { class: 'cast-stage spare' }));
    S.root = h('div', { id: 'cast' }, brand, S.gyroWrap, bar);
    document.getElementById('ui').append(S.root);
    // the page's chrome comes back while the pointer moves and goes after three seconds
    const stay = ['', '0', null].indexOf(new URLSearchParams(window.__query || location.search).get('awake')) < 0;                    // ?awake=1: the chrome stays (for a screenshot of it)
    const wake = () => { document.body.classList.add('awake'); clearTimeout(S.sleep); if (!stay) S.sleep = setTimeout(() => document.body.classList.remove('awake'), 3000); };
    S.wake = wake; S.onkey = (e) => { if (e.key.toLowerCase() === 'g' && !/INPUT|SELECT|TEXTAREA/.test((document.activeElement && document.activeElement.tagName) || '')) { S.gyroOn = !S.gyroOn; store.set('cast.gyro', S.gyroOn); S.gyroWrap.classList.toggle('off', !S.gyroOn); } wake(); };
    window.addEventListener('pointermove', wake); window.addEventListener('keydown', S.onkey); wake();
    if (!store.get('cast.seen', false)) { store.set('cast.seen', true); app.setAuto && app.setAuto(true); }
  },

  update(app, pose, d, dt) {
    const S = this.s, raw = app.raw || pose, t = raw.t;
    S.clock.textContent = hms(t); if (S.clock.className !== 'cast-clock' + (t < 0 ? ' pre' : '')) S.clock.className = 'cast-clock' + (t < 0 ? ' pre' : '');
    const views = stageViews(app, raw), imp = S.units === 'imperial';
    for (const b of S.blocks) {
      const v = views[b.i], p = v && v.pose;
      const sp = p ? (imp ? p.spd * 2.23694 : p.spd * 3.6) : null, al = p ? (imp ? p.alt / 1609.344 : p.alt / 1000) : null;
      const set = (n, val, unit) => { const text = val == null ? '—' : val; if (n.v.textContent !== text) n.v.textContent = text; if (n.u.textContent !== unit) n.u.textContent = unit; };
      set(b.sp, sp == null ? null : group(sp), imp ? 'MPH' : 'KM/H'); set(b.al, al == null ? null : (al < 10 ? al.toFixed(2) : al.toFixed(1)), imp ? 'MI' : 'KM');
      b.el.classList.toggle('gone', !p);
      const states = p && v.engines.length ? v.engines.map(({ k }) => (p.eng || [])[k] ?? 0) : [];
      drawEngines(b.eng, v ? v.engines : [], states);
      const on = states.filter((x) => x > 0.02).length, txt = v && v.engines.length ? `${on} ${on === 1 ? 'ENGINE' : 'ENGINES'}` : '';
      if (b.cnt.textContent !== txt) b.cnt.textContent = txt;
      b.el.classList.toggle('lit', on > 0);
    }
    // the events: the line of them under the clock, a marker for each, the newest said above the clock for a few seconds
    const so = app.data.source, rec = so && so.kind === 'pose-file' ? so : null;
    const t0 = rec ? Math.min(0, rec.start) : 0, t1 = rec ? Math.max(rec.end, t + 1) : Math.max(600, t * 1.15);
    const frac = (x) => Math.max(0, Math.min(1, (x - t0) / Math.max(1, t1 - t0)));
    S.track.children[1].style.left = (frac(t) * 100).toFixed(2) + '%'; S.track.children[0].style.setProperty('--done', (frac(t) * 100).toFixed(2) + '%');
    const evs = app.events.filter((e) => MARK(e.text)), key = evs.map((e) => e.t.toFixed(1) + e.text).join('|') + '|' + t1.toFixed(0);
    if (key !== S.eventsKey) {
      S.eventsKey = key;
      S.marks.replaceChildren(...evs.map((e) => h('i', { style: { left: (frac(e.t) * 100).toFixed(2) + '%' }, title: `${callName(e.text)}  ${hms(e.t)}`, class: e.level })));
    }
    const recent = evs.filter((e) => e.t <= t && t - e.t < 6).pop();
    const line = recent ? callName(recent.text) : '', last = evs.filter((e) => e.t <= t).pop();
    if (S.call.textContent !== line) S.call.textContent = line; S.call.classList.toggle('on', !!recent);
    const lastTxt = last ? `${callName(last.text)}  ${hms(last.t)}` : '';
    if (S.last.textContent !== lastTxt) S.last.textContent = lastTxt;
    const live = rec ? (rec.playing ? 'REPLAY' : 'PAUSED') : app.data.live ? 'LIVE' : '';
    if (S.live.textContent !== live) { S.live.textContent = live; S.live.dataset.k = live; }
    if (S.gyroOn) S.gyro.update(pose);
    S.gyro.inst.visible = S.gyroOn && !document.body.classList.contains('hideui');
  },

  unmount(app) {
    const S = this.s; if (!S) return;
    window.removeEventListener('pointermove', S.wake); window.removeEventListener('keydown', S.onkey); clearTimeout(S.sleep);
    S.gyro.dispose(); S.root.remove(); document.body.classList.remove('cast', 'awake');
    this.s = null;
  },
};
