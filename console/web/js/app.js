// The shell: the header, the tab rail, the event stream, and the render loop that keeps the visible tab current.
import { $, h, icon, setText, clock, store, toast, NODES } from './util.js';
import { api, stream, hasToken } from './net.js';
import { S, setHistory, pushMetrics, addEvent, addLine } from './state.js';
import { resetColorCache } from './charts.js';
import mission from './tabs/mission.js';
import launch from './tabs/launch.js';
import voting from './tabs/voting.js';
import flight from './tabs/flight.js';
import commands from './tabs/commands.js';
import faults from './tabs/faults.js';
import vehicle from './tabs/vehicle.js';
import rig from './tabs/rig.js';
import bus from './tabs/bus.js';
import events from './tabs/events.js';

const TABS = [mission, launch, voting, flight, commands, faults, vehicle, rig, bus, events];
const ctx = { api, toast, S, go: (id) => select(id), tabs: TABS };
let current = null;
let dirty = true;

/* ---------- tabs ---------- */
const nav = $('#nav'), main = $('#main');
nav.append();
const transport = h('div', { id: 'transport', class: 'card hide', style: { marginBottom: '12px' } });
main.append(transport);
for (const [i, t] of TABS.entries()) {
  const btn = h('button', { role: 'tab', id: `tab-${t.id}`, 'aria-selected': 'false', 'aria-controls': `pane-${t.id}`, title: `${t.label} (${(i + 1) % 10})`, onclick: () => select(t.id) },
    icon(t.icon), h('span', { class: 'lbl' }, t.label), h('span', { class: 'badge hide', id: `badge-${t.id}` }));
  nav.append(btn);
  if (i === 3 || i === 6) nav.append(h('hr'));
  const pane = h('section', { class: 'tab', id: `pane-${t.id}`, role: 'tabpanel', 'aria-labelledby': `tab-${t.id}` });
  main.append(pane);
  t.mount(pane, ctx);
}
nav.append(h('div', { class: 'hint' }, 'Keys 1 to 0 switch tabs.', h('br'), 'The console decides nothing about the flight: it shows what the computers say and what it can compute from the bus.'));

function select(id) {
  const t = TABS.find((x) => x.id === id) || TABS[0];
  current = t;
  for (const x of TABS) {
    const on = x === t;
    $(`#pane-${x.id}`).classList.toggle('active', on);
    $(`#tab-${x.id}`).setAttribute('aria-selected', on ? 'true' : 'false');
  }
  location.hash = t.id;
  store.set('tab', t.id);
  dirty = true;
  main.scrollTop = 0;
}
addEventListener('keydown', (e) => {
  if (e.target.closest && e.target.closest('input, textarea, select, [contenteditable]') || e.ctrlKey || e.metaKey || e.altKey) return;
  const i = e.key === '0' ? 9 : +e.key - 1;
  if (i >= 0 && i < TABS.length) select(TABS[i].id);
});
addEventListener('hashchange', () => { const id = location.hash.slice(1); if (id && id !== current?.id) select(id); });

/* ---------- theme ---------- */
function setTheme(t) { document.documentElement.dataset.theme = t; store.set('theme', t); resetColorCache(); dirty = true; }
$('#theme-btn').addEventListener('click', () => setTheme(document.documentElement.dataset.theme === 'dark' ? 'light' : 'dark'));
setTheme(store.get('theme', 'dark'));

/* ---------- header ---------- */
const chipClass = (el, base, lv) => { el.className = `${base} ${lv}`; };
function header() {
  const s = S.snap;
  const src = $('#src-chip');
  if (!s || S.conn === 'down') {
    chipClass(src, 'chip', S.conn === 'down' ? 'crit' : 'muted'); src.innerHTML = '';
    src.append(h('i', { class: 'dot crit' }), S.conn === 'down' ? 'console server unreachable' : 'connecting…');
    return;
  }
  const so = s.source || {};
  src.innerHTML = '';
  if (so.kind === 'live') {
    const flowing = s.sync && s.sync.alive;
    chipClass(src, 'chip', flowing ? 'ok' : 'warn');
    src.append(h('i', { class: `dot ${flowing ? 'ok live' : 'warn'}` }), `LIVE ${so.iface}`, h('span', { class: 'mono', style: { fontWeight: 500 } }, flowing ? `${s.frame_rate.toFixed(1)} Hz` : 'no SYNC'));
  } else if (so.kind === 'replay') {
    chipClass(src, 'chip', 'info'); src.append(h('i', { class: 'dot info' }), `REPLAY ${so.file}`, h('span', { class: 'mono', style: { fontWeight: 500 } }, `${so.speed}×`));
  } else {
    chipClass(src, 'chip', 'muted'); src.append(h('i', { class: 'dot' }), 'no source — open Rig');
  }
  // the mission clock
  const ph = s.phase, ck = $('#clock'), tr = s.truth;
  let v = '--:--.-', l = 'no sync', cls = '';
  if (ph.name === 'pad') { v = 'PAD'; l = 'on the pad'; }
  else if (ph.name === 'countdown') { v = `T-${clock(ph.t_minus_s)}`; l = 'countdown'; cls = 'cd'; }
  else if (ph.name === 'flight') { v = `T+${clock(ph.flight_s)}`; l = 'flight'; cls = 'fl'; }
  setText($('#clock-v'), v); setText($('#clock-l'), l); ck.className = cls;
  // redundancy mode: the worst of the live nodes' own view
  const alive = s.nodes.filter((n) => n.heartbeat);          // the nodes that report a mode (a virtual peer sends samples and no heartbeat)
  const mc = $('#mode-chip');
  if (!alive.length) { chipClass(mc, 'chip', 'muted'); setText(mc, 'no nodes'); }
  else {
    const m = Math.min(...alive.map((n) => n.mode));
    const name = ['SAFE', 'SIMPLEX', 'DUPLEX', 'TRIPLEX'][m];
    const up = s.nodes.filter((n) => n.alive).length;
    chipClass(mc, 'chip', m === 3 ? 'ok' : m === 2 ? 'warn' : 'crit'); setText(mc, `${name} ${up}/3`);
  }
  const ac = $('#act-chip');
  if (!s.act) { chipClass(ac, 'chip', 'muted'); setText(ac, 'ACT —'); }
  else {
    const lv = !s.act.alive ? 'crit' : s.act.state === 1 ? 'ok' : s.act.state === 0 ? 'info' : 'crit';
    chipClass(ac, 'chip', lv); setText(ac, `ACT ${s.act.alive ? s.act.state_name.toUpperCase() : 'SILENT'}`);
  }
  // alerts
  const al = s.alerts || [], crit = al.filter((a) => a.level === 'crit').length, warn = al.filter((a) => a.level === 'warn').length;
  const ab = $('#alert-chip');
  chipClass(ab, 'chip', crit ? 'crit' : warn ? 'warn' : 'ok');
  setText(ab, crit ? `${crit} critical${warn ? ` · ${warn} warning` : ''}` : warn ? `${warn} warning${warn > 1 ? 's' : ''}` : 'all nominal');
  const bg = $('#badge-mission');
  if (crit || warn) { bg.classList.remove('hide'); bg.classList.toggle('warn', !crit); setText(bg, crit || warn); } else bg.classList.add('hide');
  // the record button
  const rb = $('#rec-btn');
  rb.classList.toggle('on', !!s.recording);
  setText(rb.lastChild, s.recording ? `REC ${s.recording.frames > 999 ? (s.recording.frames / 1000).toFixed(1) + 'k' : s.recording.frames}` : 'REC');
  // the replay transport
  transportBar(s);
}
$('#alert-chip').addEventListener('click', () => select('mission'));
$('#rec-btn').addEventListener('click', async () => {
  try {
    if (S.snap && S.snap.recording) { const r = await api('/api/record', { stop: true }); toast(`Recording saved: ${r.recording.file.split('/').pop()} (${r.recording.frames} frames)`, 'ok'); }
    else { const r = await api('/api/record', {}); toast(`Recording to ${r.recording.file.split('/').pop()}`, 'info'); }
  } catch (e) { toast(e.message, 'warn'); }
});

let tbKey = '';
function transportBar(s) {
  const so = s.source || {};
  if (so.kind !== 'replay') { transport.classList.add('hide'); tbKey = ''; return; }
  transport.classList.remove('hide');
  if (tbKey !== 'built') {
    tbKey = 'built'; transport.innerHTML = '';
    const play = h('button', { class: 'btn', id: 'tb-play', onclick: () => api('/api/replay', { play: !(S.snap.source.playing) }).catch((e) => toast(e.message, 'warn')) });
    const seek = h('input', { type: 'range', id: 'tb-seek', min: 0, max: 1000, value: 0, style: { flex: 1 }, 'aria-label': 'Replay position' });
    seek.addEventListener('change', () => { const so2 = S.snap.source; api('/api/replay', { seek: so2.start + (seek.value / 1000) * (so2.end - so2.start) }).catch((e) => toast(e.message, 'warn')); });
    const speed = h('select', { id: 'tb-speed', 'aria-label': 'Replay speed' }, [0.25, 0.5, 1, 2, 4, 8, 16].map((x) => h('option', { value: x }, `${x}×`)));
    speed.addEventListener('change', () => api('/api/replay', { speed: +speed.value }).catch((e) => toast(e.message, 'warn')));
    transport.append(h('div', { class: 'body', style: { display: 'flex', gap: '12px', alignItems: 'center' } }, h('span', { class: 'chip info' }, 'REPLAY'), play, seek, h('span', { class: 'mono', id: 'tb-pos' }), speed,
      h('span', { class: 'note', id: 'tb-note' })));
  }
  setText($('#tb-play'), so.playing ? '❚❚ Pause' : '▶ Play');
  const seek = $('#tb-seek'); if (document.activeElement !== seek) seek.value = Math.round(((so.pos - so.start) / Math.max(1e-6, so.end - so.start)) * 1000);
  setText($('#tb-pos'), `${clock(so.pos - so.start)} / ${clock(so.end - so.start)}`);
  const sp = $('#tb-speed'); if (document.activeElement !== sp) sp.value = so.speed;
  setText($('#tb-note'), `${so.file}${so.sidecar ? ' · with the nodes’ consoles and the simulator’s telemetry' : ' · bus only'}`);
}

/* ---------- the stream ---------- */
let renderQueued = false;
function schedule() { dirty = true; if (!renderQueued) { renderQueued = true; requestAnimationFrame(frame); } }
function frame() {
  renderQueued = false;
  if (!dirty) return;
  dirty = false;
  try { header(); if (current) current.update(S, ctx); } catch (e) { console.error(e); showError(e); }
}
function showError(e) {
  let box = $('#js-error');
  if (!box) { box = h('div', { id: 'js-error', class: 'banner crit', style: { position: 'fixed', left: '184px', bottom: '12px', right: '12px', zIndex: 70, whiteSpace: 'pre-wrap', fontFamily: 'var(--mono)' } }); document.body.append(box); }
  box.textContent = `Page error: ${e.message}\n${(e.stack || '').split('\n').slice(0, 3).join('\n')}`;
}
addEventListener('error', (e) => showError(e.error || new Error(e.message)));
addEventListener('unhandledrejection', (e) => showError(e.reason instanceof Error ? e.reason : new Error(String(e.reason))));

if (!hasToken()) {
  main.prepend(h('div', { class: 'banner crit' }, icon('alert'), h('div', {}, h('b', {}, 'No session token. '), 'Open the address the console printed when it started (it ends in ?token=…): every request to the console needs it.')));
}
stream({
  status: (st) => { S.conn = st; schedule(); },
  hello: (d) => {
    S.cfg = d.config; S.vehicles = d.vehicles || []; setHistory(d.history); S.events = d.events.slice(); S.lines = {};
    for (const [src, arr] of Object.entries(d.lines || {})) S.lines[src] = arr.slice();
    S.snap = d.snapshot; S.conn = 'up'; schedule();
  },
  state: (d) => { S.snap = d.snapshot; S.msgs++; S.lastMsgAt = performance.now(); if (d.snapshot && d.snapshot.source && d.snapshot.source.kind !== 'none') pushMetrics(d.snapshot.t, d.metrics); schedule(); },
  event: (e) => { addEvent(e); S.eventSeq++; schedule(); },
  line: (l) => { addLine(l); S.eventSeq++; schedule(); },
  reset: () => { S.hist = { t: [], s: {} }; S.events = []; S.lines = {}; schedule(); },
  error: (e) => showError(e),
});
select(location.hash.slice(1) || store.get('tab', 'mission'));
const st = new URLSearchParams(location.search).get('selftest') || (window.__selftest || '');
if (st) setTimeout(() => import('./selftest.js').then((m) => m.run(st, ctx)), 2500);       // a development aid: see selftest.js
setInterval(() => { if (S.conn === 'up' && performance.now() - S.lastMsgAt > 2500) { S.conn = 'down'; } schedule(); }, 1000);
