// The 3D viewer's page: the data, the world, the lenses, the controls.
//
// The page is made of a canvas (world.js draws on it), a top bar (the source, the clock, the lenses), two docks of panels that the current lens fills, and a bottom bar with the transport of a recording.
// A lens (js/lenses/*.js) is one way of looking at the same flight: it has `mount(app)` (build its panels and its 3D overlays), `update(app, pose, derived, dt)` every frame, and `unmount(app)`.
import * as THREE from '../vendor/three.module.js';
import { ViewerData, poseFromTruth } from './data.js';
import { World } from './world.js';
import { CAMERAS } from './cam.js';
import { derive } from './physics.js';
import { loadGltf } from './models/gltf.js';
import { DRESS } from './models/starship.js';
import { h, store, fmt } from './ui.js';
import { api } from '../../js/net.js';
import { toast } from '../../js/util.js';
import overview from './lenses/overview.js';
import aero from './lenses/aero.js';
import atmosphere from './lenses/atmosphere.js';
import propulsion from './lenses/propulsion.js';
import loads from './lenses/loads.js';
import attitude from './lenses/attitude.js';

const Q = new URLSearchParams(window.__query || location.search);

// ---- a console of the page's own, to the server's, for a browser that has none at hand (a headless one under test)
if (Q.get('debug')) {
  const tok = sessionStorage.getItem('tfc.token') || Q.get('token') || '';
  const send = (k, a) => { try { fetch('/api/viewer/log', { method: 'POST', headers: { 'X-TFC-Token': tok, 'Content-Type': 'application/json' }, body: JSON.stringify({ text: k + ' ' + a.map((x) => (x && x.stack) || (typeof x === 'object' ? JSON.stringify(x) : String(x))).join(' ') }) }); } catch { /* nothing */ } };
  for (const k of ['error', 'warn', 'log']) { const o = console[k].bind(console); console[k] = (...a) => { send(k, k === 'warn' ? [...a, new Error().stack.split('\n').slice(2, 5).join(' < ')] : a); o(...a); }; }
  window.addEventListener('error', (e) => send('window.error', [e.message, e.filename + ':' + e.lineno]));
  window.addEventListener('unhandledrejection', (e) => send('rejection', [e.reason]));
  window.__dbg = (...a) => send('dbg', a);
}

const LENSES = [overview, aero, atmosphere, propulsion, loads, attitude];

const canvas = document.getElementById('scene');
const ui = document.getElementById('ui');
const world = new World(canvas, { preserve: !!Q.get('shot') });
const data = new ViewerData();
world.imagery.refresh();
if (Q.get('img')) world.imagery.setMode(Q.get('img')); if (Q.get('night') === '0') world.imagery.setNight(false);
world.skip = { sky: Q.get('nosky'), near: Q.get('nonear'), env: Q.get('noenv') };

// remembered choices
const st = world.settings;
st.sunElev = store.get('v.sunEl', st.sunElev); st.sunBear = store.get('v.sunBear', st.sunBear); st.cloud = store.get('v.cloud', st.cloud);
st.plumes = store.get('v.plumes', true); st.trail = store.get('v.trail', true); st.pad = store.get('v.pad', true);

const app = {
  world, data, events: [], wall: 0, dt: 0.016, pose: null, d: null, lens: null, styleName: Q.get('style') || store.get('v.style', 'auto'),
  get spec() { return data.spec; }, get vehicle() { return world.vehicle; },
  left: h('aside', { id: 'v-left', class: 'v-dock', 'aria-label': 'Left panels' }), right: h('aside', { id: 'v-right', class: 'v-dock', 'aria-label': 'Right panels' }),
  models: [], gltf: Object.assign({ axis: '+y', fitLength: true, roll: 0, offset: 0, scale: 1, scene: null, name: null }, store.get('v.gltf', {}), { scene: null, name: null }),
  /** The model files in console/models/, as the server lists them. */
  async refreshModels() { try { const s = await api('/api/viewer/state'); this.models = s.models || []; } catch { /* the list stays as it was */ } },
  /** A generated style, or `gltf:NAME` for an imported file (loaded here, then built in). */
  async setStyle(v) {
    if (v.startsWith('gltf:')) {
      try { this.gltf.name = v.slice(5); this.gltf.scene = await loadGltf(this.gltf.name); } catch (e) { toast(`Could not load the model: ${e.message}`, 'crit', 9000); v = 'auto'; }
    }
    this.styleName = v; store.set('v.style', v); rebuildVehicle();
  },
  dress: Object.assign({}, DRESS.v3, store.get('v.dress', {})),
  saveDress() { store.set('v.dress', this.dress); },
  saveGltf() { const { axis, fitLength, roll, offset, scale } = this.gltf; store.set('v.gltf', { axis, fitLength, roll, offset, scale }); },
  rebuild() { rebuildVehicle(); },
  screenshot() { world.render(); const a = h('a', { href: canvas.toDataURL('image/png'), download: `tfc-viewer-${(app.pose ? app.pose.t : 0).toFixed(1)}.png` }); document.body.append(a); a.click(); a.remove(); },
  fullscreen() { if (document.fullscreenElement) document.exitFullscreen(); else document.documentElement.requestFullscreen && document.documentElement.requestFullscreen(); },
};
if (Q.get('gaxis')) app.gltf.axis = Q.get('gaxis');
window.__viewer = { world, data, app };

// ---- the top bar
const srcChip = h('button', { class: 'chip muted v-chip', id: 'v-src', type: 'button', title: 'Where the data comes from: click to open a flight, fly a vehicle or look at the options' }, h('i', { class: 'dot' }), h('span', {}, 'no data'));
const vehChip = h('span', { class: 'chip muted', id: 'v-veh' }, '—');
const clockEl = h('div', { id: 'v-clock', 'aria-live': 'off' }, 'T —');
const lensBar = h('nav', { id: 'v-lenses', role: 'tablist', 'aria-label': 'Lenses' }, LENSES.map((l) => h('button', { type: 'button', role: 'tab', dataset: { id: l.id }, 'aria-selected': 'false', onclick: () => setLens(l.id) }, l.label, h('kbd', {}, l.key))));
const top = h('header', { id: 'v-top' },
  h('div', { class: 'v-brand' }, h('svg', { viewBox: '0 0 32 32', 'aria-hidden': 'true' }, h('path', { d: 'M16 4l4 9v8l-4 5-4-5v-8z', fill: 'none', stroke: 'var(--accent)', 'stroke-width': 2.2, 'stroke-linejoin': 'round' })), h('span', {}, 'TFC ', h('small', {}, '3D viewer'))),
  srcChip, vehChip, clockEl, h('span', { class: 'v-spacer' }), lensBar, h('span', { class: 'v-spacer' }),
  h('a', { class: 'v-iconbtn', href: '/', target: '_blank', rel: 'noopener', title: 'The flight console', onclick: (e) => { e.preventDefault(); window.open('/?token=' + encodeURIComponent(sessionStorage.getItem('tfc.token') || ''), '_blank'); } }, 'Console ↗'),
  h('button', { class: 'v-iconbtn', type: 'button', id: 'v-hide', title: 'Hide or show the panels (H)', onclick: () => toggleUI() }, 'Panels'));
const bottom = h('div', { id: 'v-bottom' });
// the cameras: always on the screen, whatever the lens, with what the current one does said under it (they were a card of the overview lens, which made them invisible in the other five)
const camHint = h('div', { id: 'v-camhint' });
const camBar = h('div', { id: 'v-cams' }, h('nav', { 'aria-label': 'Camera', role: 'group' }, CAMERAS.map((c) => h('button', { type: 'button', dataset: { id: c.id }, 'aria-pressed': 'false', title: c.hint, onclick: () => { world.rig.set(c.id); store.set('v.cam', c.id); syncCamSeg(); } }, c.label))), camHint);
// the docks can be folded away one at a time (H hides everything); a narrow window starts with the right one folded
const dockBtn = (side, glyph) => h('button', { type: 'button', class: 'v-dockbtn', id: 'v-tg-' + side, 'aria-label': `Hide or show the ${side} panels`, title: `Hide or show the ${side} panels`, onclick: () => setDock(side, document.body.classList.contains('no-' + side)) }, glyph);
function setDock(side, show) { document.body.classList.toggle('no-' + side, !show); store.set('v.dock.' + side, show); const b = document.getElementById('v-tg-' + side); if (b) b.textContent = side === 'left' ? (show ? '‹' : '›') : (show ? '›' : '‹'); }
// what the page says when nothing has arrived: where the data can come from, and a button for each
const empty = h('div', { class: 'v-empty card v-card', hidden: true },
  h('header', {}, h('h3', {}, 'Nothing to show yet')),
  h('div', { class: 'body' },
    h('p', {}, 'The viewer draws what the simulator sends. Start the rig from the console and the launch appears here as it happens, or play a flight that is already flown:'),
    h('div', { class: 'v-row' },
      h('button', { class: 'btn go', type: 'button', onclick: async () => { try { const s = await api('/api/viewer/state'); const demo = (s.pose_files || []).find((f) => f.where === 'demo') || (s.pose_files || [])[0]; if (!demo) throw new Error('there is no recorded flight: fly a vehicle instead'); await api('/api/viewer/open', { name: demo.name }); } catch (e) { toast(e.message, 'warn'); } } }, 'Play the demo flight'),
      h('button', { class: 'btn', type: 'button', onclick: () => fly('starship') }, 'Fly the Starship-class vehicle'),
      h('button', { class: 'btn', type: 'button', onclick: () => openMenu() }, 'More…')),
    h('p', { class: 'note' }, 'Flying a vehicle runs the real flight software on it (tfc_fly, a few seconds) and plays the result. The live rig is started from the console\'s Rig tab.')));
ui.append(top, app.left, app.right, camBar, dockBtn('left', '‹'), dockBtn('right', '›'), bottom, empty);
setDock('left', store.get('v.dock.left', true)); setDock('right', store.get('v.dock.right', window.innerWidth >= 1280));
app.empty = empty;

function toggleUI() { document.body.classList.toggle('hideui'); }
function nextCamera() { const i = CAMERAS.findIndex((c) => c.id === world.rig.mode); const n = CAMERAS[(i + 1) % CAMERAS.length]; world.rig.set(n.id); store.set('v.cam', n.id); syncCamSeg(); }
function syncCamSeg() {
  const m = world.rig.mode;
  for (const b of camBar.querySelectorAll('button')) b.setAttribute('aria-pressed', b.dataset.id === m ? 'true' : 'false');
  const c = CAMERAS.find((x) => x.id === m); const t = c ? c.hint : ''; if (camHint.textContent !== t) camHint.textContent = t;
}

// ---- lenses
function setLens(id) {
  const next = LENSES.find((l) => l.id === id) || LENSES[0];
  if (app.lens) { try { app.lens.unmount(app); } catch (e) { console.error(e); } }
  app.lens = next; store.set('v.lens', next.id);
  for (const b of lensBar.children) b.setAttribute('aria-selected', b.dataset.id === next.id ? 'true' : 'false');
  app.left.replaceChildren(); app.right.replaceChildren();
  try { next.mount(app); } catch (e) { console.error('lens', next.id, e); toast('The ' + next.label + ' lens failed: ' + e.message, 'crit'); }
}

function rebuildVehicle() {
  const keep = app.lens;
  if (keep) { try { keep.unmount(app); } catch (e) { console.error(e); } app.lens = null; }
  world.setVehicle(data.spec, app.styleName, { gltf: app.styleName.startsWith('gltf:') ? app.gltf : null, dress: app.dress });
  if (keep) setLens(keep.id);
}
data.onSpec.push(() => rebuildVehicle());
data.onReset.push(() => { world.trail.clear(); world.smoke.clear(); app.events.length = 0; tracker.prev = null; tracker.liftoff = false; tracker.space = false; tracker.qpeak = 0; });

// ---- the events: what happened, from the poses
const tracker = { prev: null, qpeak: 0, qpeakT: 0, maxQDone: false, liftoff: false, space: false };
function track(pose) {
  const p = tracker.prev, t = pose.t, ev = (level, text) => app.events.push({ t, level, text });
  if (p && Math.abs(pose.tt - p.tt) < 5) {
    if (p.clamp && !pose.clamp && !tracker.liftoff) { tracker.liftoff = true; ev('ok', 'lift-off'); }
    for (let i = 0; i < app.spec.stages.length; i++) {
      const name = app.spec.stages[i].name || 'stage ' + (i + 1);
      if ((((pose.ign ?? 0) >> i) & 1) && !(((p.ign ?? 0) >> i) & 1)) ev('ok', `${name} ignited`);
      if (!(((pose.stg ?? 255) >> i) & 1) && (((p.stg ?? 255) >> i) & 1)) ev('info', `${name} separated`);
    }
    for (let i = 0; i < (app.spec.payloads || []).length; i++) if (!(((pose.pay ?? 255) >> i) & 1) && (((p.pay ?? 255) >> i) & 1)) ev('info', `${app.spec.payloads[i].name} jettisoned`);
    (pose.eng || []).forEach((f, k) => { if (f < 0 && !((p.eng || [])[k] < 0)) ev('crit', `engine ${k + 1} failed`); });
    if (pose.thr <= 1 && p.thr > 1 && tracker.liftoff) ev('info', 'engines off');
    if (!tracker.space && pose.alt > 100000) { tracker.space = true; ev('info', 'passed 100 km (the Karman line)'); }
    if (pose.qd > tracker.qpeak) { tracker.qpeak = pose.qd; tracker.qpeakT = t; tracker.maxQDone = false; }
    else if (!tracker.maxQDone && tracker.qpeak > 2000 && pose.qd < 0.9 * tracker.qpeak) { tracker.maxQDone = true; ev('info', `max-Q passed: ${(tracker.qpeak / 1000).toFixed(1)} kPa at T+${tracker.qpeakT.toFixed(1)} s`); }
    if (pose.crash && !p.crash) ev('crit', 'the vehicle was destroyed');
  } else if (p) { tracker.liftoff = false; }
  tracker.prev = pose;
}

// ---- the source menu
let menu = null;
async function openMenu() {
  if (menu) { menu.remove(); menu = null; return; }
  let state; try { state = await api('/api/viewer/state'); } catch (e) { toast(e.message, 'crit'); return; }
  const m = menu = h('div', { class: 'v-menu', role: 'dialog', 'aria-label': 'Source' });
  const close = () => { m.remove(); menu = null; };
  m.append(h('h4', {}, 'Now'), h('p', { class: 'note' }, describeSource(state)));
  if (state.pose_files.length) {
    m.append(h('h4', {}, 'Open a recorded flight'));
    for (const f of state.pose_files) m.append(h('button', { class: 'item', type: 'button', onclick: async () => { close(); try { await api('/api/viewer/open', { name: f.name, loop: false }); toast('Playing ' + f.name); } catch (e) { toast(e.message, 'crit'); } } }, f.name, h('small', {}, `${f.where}, ${(f.bytes / 1e6).toFixed(1)} MB`)));
  }
  m.append(h('h4', {}, 'Fly a vehicle with the real flight software'));
  if (!state.fly_available) m.append(h('p', { class: 'note' }, 'build/host/tfc_fly is not built (cmake --build build/host).'));
  else {
    for (const v of state.vehicles) m.append(h('button', { class: 'item', type: 'button', onclick: () => { close(); fly(v.name); } }, v.name, h('small', {}, (v.description || '').slice(0, 110))));
    m.append(h('p', { class: 'note' }, 'This runs tfc_fly (a few seconds, up to a minute for a long flight), then plays the result: every vehicle in vehicles/ can be seen this way, whatever the rig itself flies.'));
  }
  m.append(h('h4', {}, 'Live'), h('p', { class: 'note' }, `Poses from a running tfc_simd arrive on UDP port ${state.pose_port} (the rig the console starts is told it). They are shown as soon as they come.`));
  document.body.append(m);
  setTimeout(() => document.addEventListener('pointerdown', function away(e) { if (menu && !menu.contains(e.target) && !srcChip.contains(e.target)) close(); if (!menu) document.removeEventListener('pointerdown', away); }), 0);
}
srcChip.addEventListener('click', openMenu);

function describeSource(state) {
  const so = state.source || {};
  if (so.kind === 'pose-file') return `The recorded flight ${so.file}: ${fmt.t(so.start)} to ${fmt.t(so.end)}, ${so.state}.`;
  if (data.consoleSource && data.consoleSource.kind === 'replay') return 'The console is replaying a recording: ' + data.consoleSource.file + '.';
  if (data.live) return `Live: ${data.rateEst.toFixed(0)} poses a second from the simulator.`;
  if (data.truthSeen) return 'Only the console\'s 10 Hz telemetry is arriving (no poses: an older simulator): the vehicle is placed and turned, but there are no forces, no air and no per-engine state.';
  return 'No data yet. Start the rig from the console, or open a recorded flight, or fly a vehicle from this menu.';
}

async function fly(name) {
  toast(`Flying ${name} with the real flight software...`, 'info', 12000);
  try {
    const { job } = await api('/api/viewer/fly', { name, sensors: name === 'reference' ? 'platform' : 'vehicle', pad: 300 });
    for (let i = 0; i < 600; i++) {
      await new Promise((r) => setTimeout(r, 800));
      const j = await api('/api/job?id=' + job);
      if (j.state === 'done') { toast(`${name}: flown, playing it`, 'ok'); return; }
      if (j.state === 'failed') { toast(`${name}: ${j.error}`, 'crit', 12000); return; }
    }
  } catch (e) { toast(e.message, 'crit', 9000); }
}

// ---- the bottom bar: the transport of a pose file or of the console's bus replay, and a status line
const tb = {};
function buildBottom() {
  tb.play = h('button', { class: 'btn', type: 'button', style: { minWidth: '84px' }, onclick: () => control('toggle') }, 'Pause');
  tb.back = h('button', { class: 'btn', type: 'button', title: 'Back 10 s', onclick: () => control('seek', -10) }, '−10 s');
  tb.fwd = h('button', { class: 'btn', type: 'button', title: 'Forward 10 s', onclick: () => control('seek', 10) }, '+10 s');
  tb.range = h('input', { type: 'range', min: 0, max: 1000, value: 0, 'aria-label': 'Position in the recording' });
  tb.range.addEventListener('change', () => control('seekAbs', +tb.range.value / 1000)); tb.range.addEventListener('pointerdown', () => { tb.drag = true; }); tb.range.addEventListener('pointerup', () => { tb.drag = false; });
  tb.speed = h('select', { 'aria-label': 'Speed' }, [0.25, 0.5, 1, 2, 4, 8, 16].map((v) => h('option', { value: v, selected: v === 1 ? true : null }, v + '×')));
  tb.speed.addEventListener('change', () => control('speed', +tb.speed.value));
  tb.time = h('span', { class: 'v-info' }); tb.info = h('span', { class: 'v-info' });
  tb.group = h('div', { class: 'hide', style: { display: 'flex', gap: '10px', alignItems: 'center', flex: 1 } }, tb.play, tb.back, tb.fwd, tb.range, tb.time, tb.speed);
  bottom.append(tb.group, h('span', { class: 'v-spacer' }), tb.info);
}
async function control(action, v) {
  const so = data.source, cs = data.consoleSource;
  try {
    if (so && so.kind === 'pose-file') {
      if (action === 'toggle') await api('/api/viewer/control', { action: so.playing ? 'pause' : 'play' });
      else if (action === 'seek') await api('/api/viewer/control', { action: 'seek', value: Math.max(so.start, Math.min(so.end, so.pos + v)) });
      else if (action === 'seekAbs') await api('/api/viewer/control', { action: 'seek', value: so.start + v * (so.end - so.start) });
      else if (action === 'speed') await api('/api/viewer/control', { action: 'speed', value: v });
    } else if (cs && cs.kind === 'replay') {
      if (action === 'toggle') await api('/api/replay', { play: !cs.playing });
      else if (action === 'seek') await api('/api/replay', { seek: Math.max(cs.start, Math.min(cs.end, cs.pos + v)) });
      else if (action === 'seekAbs') await api('/api/replay', { seek: cs.start + v * (cs.end - cs.start) });
      else if (action === 'speed') await api('/api/replay', { speed: v });
    }
  } catch (e) { toast(e.message, 'warn'); }
}
function updateBottom() {
  const so = data.source, cs = data.consoleSource;
  const rec = so && so.kind === 'pose-file' ? so : cs && cs.kind === 'replay' ? cs : null;
  tb.group.classList.toggle('hide', !rec);
  if (rec) {
    tb.play.textContent = rec.playing ? 'Pause' : 'Play';
    if (!tb.drag) tb.range.value = Math.round(1000 * (rec.pos - rec.start) / Math.max(1e-6, rec.end - rec.start));
    tb.time.textContent = `${fmt.t(rec.pos)} / ${fmt.t(rec.end)}`;
    if (document.activeElement !== tb.speed) tb.speed.value = String(rec.speed);
  }
  const bits = [`${(1000 / Math.max(1, frameMs)).toFixed(0)} fps`];
  if (data.live) bits.push(`${data.rateEst.toFixed(0)} poses/s`); else if (data.truthSeen) bits.push('10 Hz truth only');
  bits.push(data.spec.name);
  tb.info.textContent = bits.join('   ');
}

// ---- the chips and the clock
function updateHeader(pose) {
  const so = data.source, cs = data.consoleSource;
  let kind = 'none', text = 'no data', cls = 'muted';
  if (so && so.kind === 'pose-file') { kind = 'file'; text = `recorded: ${so.file}`; cls = so.playing ? 'info' : 'warn'; }
  else if (cs && cs.kind === 'replay' && pose) { kind = 'replay'; text = `console replay: ${cs.file}`; cls = 'info'; }
  else if (data.live) { kind = 'live'; text = 'live simulator'; cls = 'ok'; }
  else if (pose && pose.truthOnly) { kind = 'truth'; text = 'telemetry only (10 Hz)'; cls = 'warn'; }
  else if (pose) { kind = 'held'; text = 'last state'; cls = 'warn'; }
  const wanted = `chip ${cls} v-chip`;
  if (srcChip.className !== wanted) srcChip.className = wanted;
  const span = srcChip.querySelector('span'); if (span.textContent !== text) span.textContent = text;
  srcChip.querySelector('.dot').className = 'dot ' + cls.replace('muted', '') + (kind === 'live' ? ' live' : '');
  if (vehChip.textContent !== data.spec.name) vehChip.textContent = data.spec.name;
  if (pose) {
    const t = fmt.t(pose.t); if (clockEl.textContent !== t) clockEl.textContent = t;
    clockEl.className = pose.t >= 0 && !pose.clamp ? 'fl' : 'cd';
  }
}

// ---- input
let drag = null;
canvas.addEventListener('pointerdown', (e) => { drag = { x: e.clientX, y: e.clientY, slide: e.button === 2 || e.shiftKey }; canvas.setPointerCapture(e.pointerId); canvas.classList.add('dragging'); });
canvas.addEventListener('pointermove', (e) => { if (!drag) return; world.rig.drag(e.clientX - drag.x, e.clientY - drag.y, drag.slide); drag.x = e.clientX; drag.y = e.clientY; syncCamSeg(); });
canvas.addEventListener('pointerup', () => { drag = null; canvas.classList.remove('dragging'); });
canvas.addEventListener('contextmenu', (e) => e.preventDefault());
canvas.addEventListener('dblclick', () => { world.rig.recenter(); syncCamSeg(); });
canvas.addEventListener('wheel', (e) => { e.preventDefault(); if (world.rig.mode === 'ground') world.rig.gzoom = Math.min(4, Math.max(0.12, world.rig.gzoom * Math.exp(e.deltaY * 0.001))); else world.rig.zoom(e.deltaY); }, { passive: false });
window.addEventListener('keydown', (e) => {
  if (/INPUT|SELECT|TEXTAREA/.test((document.activeElement && document.activeElement.tagName) || '')) return;
  const k = e.key.toLowerCase(), lens = LENSES.find((l) => l.key === e.key);
  if (lens) setLens(lens.id);
  else if (k === 'c') nextCamera();
  else if (k === 'h') toggleUI();
  else if (k === ' ') { e.preventDefault(); control('toggle'); }
  else if (k === 'f') app.fullscreen();
  else if (k === 'p') app.screenshot();
});

// ---- the frame loop
let last = performance.now(), frames = 0, loops = 0, frameMs = 16, shotDone = false;
const bootAt = performance.now();
function staticPose() {
  const spec = data.spec, alt = +(Q.get('alt') || 0), tilt = +(Q.get('tilt') || 0), thr = +(Q.get('thr') || 0);
  const cg = +(Q.get('cg') || spec.length * 0.45);
  const V = +(Q.get('speed') || 0), M = +(Q.get('mach') || 0), qd = +(Q.get('q') || 0), a = +(Q.get('alpha') || 0);
  const p = poseFromTruth({ alt, range: +(Q.get('range') || 0), tilt_p: tilt, tilt_y: 0, speed: V, thrust: thr * 7e7, engines_on: thr > 0 ? 1 : 0, stages_active: 255, gim_p: 0, gim_y: 0, mach: M, q: qd, mass: 4e6, ft: +(Q.get('tt') || 0) }, spec);
  p.cg = cg; p.cp = +(Q.get('cp') || spec.length * 0.9); p.eng = spec.engines.map(() => thr); p.thr = thr * 7e7; p.stg = +(Q.get('stg') || 3); p.clamp = alt < 5 ? 1 : 0; p.pay = 255; p.truthOnly = false;
  p.air = spec.airAt(alt).slice(0, 4); p.alpha = a; p.qd = qd; p.mach = M;
  p.vair = [V * Math.cos(a * Math.PI / 180), V * Math.sin(a * Math.PI / 180), 0]; p.spd = V;
  p.fae = [-qd * 70, qd * 70 * 2.5 * a * Math.PI / 180, 0]; p.fth = [p.thr, 0, 0]; p.m = 4e6;
  return p;
}
function loop(now) {
  const dt = Math.min(0.1, (now - last) / 1000); frameMs += ((now - last) - frameMs) * 0.05; last = now;
  app.wall = now / 1000; app.dt = dt;
  let pose = null;
  if (Q.get('static')) pose = staticPose();
  else { const t = data.clock.frame(data.buf, data.source, false); pose = t == null ? null : data.buf.at(t); }
  if (pose) {
    app.pose = pose;
    try {
      world.frame(pose, dt, app.wall, {});
      const d = app.d = derive(pose, data.spec);
      if (app.lens) app.lens.update(app, pose, d, dt);
      world.render(); frames++;
    } catch (e) { console.error('frame', e); }
    if (frames % 5 === 0 || !tracker.prev) track(pose);
  }
  updateHeader(pose); if (frames % 6 === 0) updateBottom();
  { const want = !pose && performance.now() - bootAt > 1500 && !Q.get('static'); if (empty.hidden === want) empty.hidden = !want; }
  loops++;
  if (Q.get('debug') && loops % 120 === 1) window.__dbg && window.__dbg('frames', frames, 'spec', data.haveSpec, 'status', data.status, 'poses', data.poseCount, 'pose', !!pose, 'ms', frameMs.toFixed(1), 'truthSeen', data.truthSeen, 'snap', !!data.snapshot, 'truth', data.snapshot && data.snapshot.truth ? JSON.stringify({ alive: data.snapshot.truth.alive, ft: data.snapshot.truth.ft }) : null, 'src', data.consoleSource && data.consoleSource.kind, 'deb', world.debris.length, 'stg', app.pose && app.pose.stg, 'ev', JSON.stringify(app.events.slice(-3).map((e) => e.text)));
  if (Q.get('shot') && !shotDone && frames > +(Q.get('shot')) && (data.haveSpec || Q.get('nospec'))) {
    shotDone = true;                                                       // a headless browser's screenshot has no WebGL canvas in it: this puts the picture on the page as an image
    const img = new Image(); img.src = world.snapshot(); img.style.cssText = 'position:fixed;inset:0;width:100%;height:100%;z-index:5;pointer-events:none';
    document.body.append(img); document.title = 'shot-ready';
  }
  requestAnimationFrame(loop);
}

// ---- go
buildBottom();
world.setVehicle(data.spec, app.styleName.startsWith('gltf:') ? 'auto' : app.styleName, { dress: app.dress });
if (Q.get('cam') || store.get('v.cam', null)) world.rig.set(Q.get('cam') || store.get('v.cam', 'orbit'));
syncCamSeg();
if (Q.get('gaz')) world.rig.gaz = +Q.get('gaz'); if (Q.get('gel')) world.rig.gel = +Q.get('gel'); if (Q.get('gzoom')) world.rig.gzoom = +Q.get('gzoom'); if (Q.get('gpos')) world.rig.gpos = Q.get('gpos').split(',').map(Number);
if (Q.get('yaw')) world.rig.yaw = +Q.get('yaw'); if (Q.get('pitch')) world.rig.pitch = +Q.get('pitch'); if (Q.get('dist')) world.rig.dist = +Q.get('dist'); if (Q.get('anchor')) world.rig.anchor = +Q.get('anchor');
if (Q.get('dbgmode')) world.sky.u.uDebug.value.x = +Q.get('dbgmode');
if (Q.get('el')) st.sunElev = +Q.get('el'); if (Q.get('bear')) st.sunBear = +Q.get('bear'); if (Q.get('cloud')) st.cloud = +Q.get('cloud');
if (Q.get('hideui')) document.body.classList.add('hideui');
setLens(Q.get('lens') || store.get('v.lens', 'overview'));
app.refreshModels().then(() => { if (app.styleName.startsWith('gltf:')) app.setStyle(app.styleName); else if (app.lens && app.lens.id === 'overview') setLens('overview'); });
data.start();
requestAnimationFrame(loop);
if (Q.get('selftest')) import('./selftest.js').then((m) => m.run(app, LENSES, setLens, world));

// ---- ?bench=1: what each choice of Earth imagery costs to draw. Each mode in turn: wait for its pictures, then draw the same frame again and again, forcing the GPU to finish each (a one-pixel read), and
// report the median and the 95th percentile of the time per frame. In a headless browser that is software rendering (a CPU doing a GPU's work): the *ratios* mean something, the milliseconds do not.
async function bench() {
  const say = (...a) => window.__dbg ? window.__dbg('bench', ...a) : console.log('bench', ...a);
  const gl = world.renderer.getContext(), px = new Uint8Array(4), next = () => new Promise((r) => setTimeout(r, 0));
  const modes = (Q.get('bench') === '1' ? 'procedural,earth-4k,earth-8k,site-4k,site-8k' : Q.get('bench')).split(',');
  await new Promise((r) => setTimeout(r, 3000));
  for (const m of modes) {
    const tLoad = performance.now();
    world.imagery.setMode(m);
    for (let i = 0; i < 400 && (world.imagery.applied === '' || world.imagery.busy); i++) await new Promise((r) => setTimeout(r, 50));
    const loadMs = Math.round(performance.now() - tLoad);
    const tFirst = performance.now(); world.render(); gl.readPixels(0, 0, 1, 1, gl.RGBA, gl.UNSIGNED_BYTE, px); const firstMs = Math.round(performance.now() - tFirst);      // the first frame with the new pictures: this is where they are uploaded to the GPU and their mip maps made               // from the choice to the pictures being on the planet: the fetch, the decode and the upload of what had not been loaded yet (pictures are kept: a later mode that reuses one pays nothing for it)
    const t = [];
    for (let i = 0; i < 100; i++) { const a = performance.now(); world.render(); gl.readPixels(0, 0, 1, 1, gl.RGBA, gl.UNSIGNED_BYTE, px); const b = performance.now(); if (i >= 10) t.push(b - a); await next(); }
    t.sort((x, y) => x - y);
    say(JSON.stringify({ mode: m, state: world.imagery.state, load_ms: loadMs, first_frame_ms: firstMs, median_ms: +t[t.length >> 1].toFixed(1), p95_ms: +t[Math.floor(t.length * 0.95)].toFixed(1), size: [world.renderer.domElement.width, world.renderer.domElement.height] }));
  }
  say('done');
}
if (Q.get('bench')) bench();
