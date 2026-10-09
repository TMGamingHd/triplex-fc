// Vehicle: choose a vehicle, change the numbers you are allowed to change, validate it with the simulator's own reader, fly a preview with the real flight software, and give the rig a vehicle.
import { h, setText, num, eng, toast, modal, icon, clear } from '../util.js';
import { api } from '../net.js';
import { Chart, Trajectory, legend } from '../charts.js';

let R = { name: 'reference', knobs: {}, base: {}, text: '' };

async function load(name, S) {
  try {
    const v = await api(`/api/vehicle?name=${encodeURIComponent(name)}`);
    R.name = name; R.base = v.knobs || {}; R.knobs = {}; R.text = v.text; R.where = v.where;
    R.editor.value = v.text; R.out.textContent = v.parse_error ? `Cannot be read: ${v.parse_error}` : 'Loaded. Validate, or fly a preview.';
    R.sheet.innerHTML = ''; knobRows(S); list(S); R.preview.innerHTML = '';
    setText(R.title, `${name}${v.where ? ' · ' + v.where : ''}`);
  } catch (e) { toast(e.message, 'crit'); }
}

function knobRows(S) {
  clear(R.sheet);
  for (const k of S.cfg.knobs) {
    const base = R.base[k.path] ?? k.default;
    const cur = R.knobs[k.path] ?? base;
    const range = h('input', { type: 'range', min: k.min, max: k.max, step: k.step, value: cur, 'aria-label': k.label });
    const box = h('input', { type: 'number', min: k.min, max: k.max, step: k.step, value: cur, style: { width: '82px' }, 'aria-label': `${k.label} value` });
    const mod = h('span', { class: 'chip warn hide' }, 'changed');
    const reset = h('button', { class: 'btn small ghost', title: 'Back to the file’s value', onclick: () => { delete R.knobs[k.path]; range.value = box.value = base; sync(); } }, '↺');
    const sync = () => { const v = +box.value; const changed = Math.abs(v - base) > 1e-9; if (changed) R.knobs[k.path] = v; else delete R.knobs[k.path]; mod.classList.toggle('hide', !changed); setText(R.nmod, Object.keys(R.knobs).length ? `${Object.keys(R.knobs).length} knob${Object.keys(R.knobs).length > 1 ? 's' : ''} changed from the file` : 'no knob changed'); };
    range.addEventListener('input', () => { box.value = range.value; sync(); }); box.addEventListener('input', () => { range.value = box.value; sync(); });
    if (R.knobs[k.path] !== undefined) mod.classList.remove('hide');
    R.sheet.append(h('div', { style: { display: 'grid', gridTemplateColumns: 'minmax(0, 1fr) auto', gap: '2px 8px', padding: '6px 0', borderBottom: '1px solid var(--line)' } },
      h('div', {}, h('b', { style: { fontSize: '13px' } }, k.label), ' ', h('span', { class: 'note' }, k.unit), ' ', mod), h('div', { style: { display: 'flex', gap: '4px', alignItems: 'center', justifySelf: 'end' } }, box, reset),
      h('div', { style: { gridColumn: '1 / 3' } }, range), h('div', { class: 'note', style: { gridColumn: '1 / 3' } }, k.doc, ` (file: ${base})`)));
  }
}

function list(S) {
  clear(R.list);
  for (const v of S.vehicles) {
    const active = S.snap && S.snap.vehicle && S.snap.vehicle.active.name === v.name;
    R.list.append(h('button', { class: 'btn ghost', style: { textAlign: 'left', display: 'block', width: '100%', whiteSpace: 'normal', height: 'auto', borderColor: v.name === R.name ? 'var(--accent)' : '' }, onclick: () => load(v.name, S) },
      h('div', { style: { display: 'flex', gap: '6px', alignItems: 'center' } }, h('b', {}, v.name), active ? h('span', { class: 'chip ok' }, 'rig') : null),
      h('div', { class: 'note', style: { fontWeight: 400, overflowWrap: 'anywhere' } }, v.where),
      h('div', { class: 'note', style: { fontWeight: 400 } }, (v.description || '').slice(0, 140))));
  }
}

function problems(out) {                                   // the reader's messages carry "(line N, column M)"
  const probs = [];
  for (const m of out.matchAll(/(.*)\(line (\d+), column (\d+)\)/g)) probs.push({ text: m[1].trim(), line: +m[2], col: +m[3] });
  return probs;
}
function gotoLine(line) {
  const ed = R.editor, lines = ed.value.split('\n');
  let start = 0; for (let i = 0; i < line - 1 && i < lines.length; i++) start += lines[i].length + 1;
  ed.focus(); ed.setSelectionRange(start, start + (lines[line - 1] || '').length);
  ed.scrollTop = Math.max(0, (line - 4) * 18);
}
const body = () => ({ text: R.editor.value, knobs: R.knobs });

async function validate() {
  R.out.textContent = 'Checking with tfc_fly --check…';
  try {
    const r = await api('/api/vehicle/check', body());
    R.out.textContent = r.output;
    R.out.className = `mono-box ${r.ok ? '' : 's-crit'}`;
    R.errs.innerHTML = '';
    for (const p of problems(r.output)) R.errs.append(h('button', { class: 'btn small warn', onclick: () => gotoLine(p.line) }, `line ${p.line}: ${p.text.replace(/^.*?: /, '').slice(0, 70)}`));
    toast(r.ok ? 'The simulator’s reader accepts this vehicle.' : 'The reader found problems.', r.ok ? 'ok' : 'warn');
  } catch (e) { toast(e.message, 'crit'); }
}

async function preview() {
  const sensors = R.sensors.value, pad = +R.pad.value;
  R.preview.innerHTML = ''; R.preview.append(h('div', { class: 'note' }, 'Flying the vehicle with the real flight software (tfc_fly)… a launcher takes a few seconds.'));
  let job;
  try { ({ job } = await api('/api/vehicle/preview', { ...body(), sensors, pad })); } catch (e) { R.preview.innerHTML = ''; toast(e.message, 'crit'); return; }
  for (let i = 0; i < 400; i++) {
    await new Promise((r) => setTimeout(r, 600));
    const j = await api(`/api/job?id=${job}`).catch(() => null);
    if (!j) continue;
    if (j.state === 'failed') { R.preview.innerHTML = ''; R.preview.append(h('div', { class: 'banner crit' }, j.error)); return; }
    if (j.state === 'done') return showPreview(j.result, j.seconds);
  }
}

function showPreview(res, secs) {
  R.preview.innerHTML = '';
  const c = res.columns;
  R.preview.append(h('div', { class: `banner ${res.ok ? 'ok' : 'warn'}` }, h('div', {}, h('b', {}, res.ok ? 'Flown: no crash, no Safe, finite. ' : 'Not a clean flight. '), `(computed in ${secs} s)`)), h('pre', { class: 'mono-box' }, res.output.split('\n').filter((l) => /Flight of|at the end|max-Q|attitude error|lift-off|stage|verdict/.test(l)).join('\n')));
  if (!c.t_s) return;
  const hist = { t: c.t_s, s: c };
  const T = c.t_s[c.t_s.length - 1] || 1;
  const mk = (series, title, opt = {}) => { const cv = h('canvas'); const ch = new Chart(cv, { height: 150, window: T, series, ...opt }); R.preview.append(h('div', { class: 'note', style: { marginTop: '10px' } }, h('b', {}, title)), cv, legend(series)); requestAnimationFrame(() => ch.draw(hist)); };
  const tc = h('canvas'); const tj = new Trajectory(tc, 240);
  R.preview.append(h('div', { class: 'note', style: { marginTop: '10px' } }, h('b', {}, 'Altitude against range')), tc);
  requestAnimationFrame(() => tj.draw({ trail: { range: c.range_m, alt: c.altitude_m }, now: null }));
  mk([{ key: 'altitude_m', label: 'altitude m', color: 'var(--accent)' }], 'Altitude');
  mk([{ key: 'speed_ms', label: 'speed m/s', color: 'var(--ok)' }], 'Speed');
  mk([{ key: 'dynamic_pressure_pa', label: 'q Pa', color: 'var(--warn)' }], 'Dynamic pressure');
  mk([{ key: 'tilt_pitch_deg', label: 'pitch tilt °', color: 'var(--info)' }, { key: 'err_pitch_deg', label: 'error vs program °', color: 'var(--crit)' }], 'Attitude', { zero: true });
  mk([{ key: 'mass_kg', label: 'mass kg', color: 'var(--nB)' }], 'Mass');
}

export default {
  id: 'vehicle', label: 'Vehicle', icon: 'vehicle',
  mount(root) {
    R.title = h('span', { class: 'note' }); R.nmod = h('span', { class: 'note' }, 'no knob changed');
    R.list = h('div', { class: 'stack', style: { gap: '6px' } });
    R.sheet = h('div');
    R.editor = h('textarea', { rows: 26, spellcheck: 'false', id: 'vehicle-json', 'aria-label': 'Vehicle file' });
    R.out = h('pre', { class: 'mono-box', style: { maxHeight: '260px' } }, 'Choose a vehicle.');
    R.errs = h('div', { class: 'btns', style: { marginTop: '6px' } });
    R.preview = h('div');
    R.sensors = h('select', { id: 'vehicle-sensors' }, [['platform', 'platform sensors (the rig)'], ['vehicle', 'the vehicle’s own sensors, accelerometer off']].map(([v, t]) => h('option', { value: v }, t)));
    R.pad = h('input', { type: 'number', value: 0, min: 0, step: 100, style: { width: '80px' }, 'aria-label': 'Pad frames' });
    R.active = h('div', { class: 'banner info' });
    R.saveName = h('input', { type: 'text', placeholder: 'save as…', style: { width: '150px' }, id: 'vehicle-save-name' });
    root.append(h('div', { class: 'stack' }, R.active,
      h('div', { class: 'grid', style: { gridTemplateColumns: 'repeat(auto-fit, minmax(min(100%, 330px), 1fr))' } },
        h('article', { class: 'card' }, h('header', {}, h('h3', {}, 'Vehicles')), h('div', { class: 'body' }, R.list, h('p', { class: 'note', style: { marginBottom: 0 } }, 'The examples in vehicles/ are never overwritten: a copy you save goes to the console’s own folder, as plain JSON (the // comments are not kept).'))),
        h('article', { class: 'card' }, h('header', {}, h('h3', {}, 'Parameters'), h('div', { class: 'tools' }, R.nmod)), h('div', { class: 'body' }, R.sheet)),
        h('article', { class: 'card' }, h('header', {}, h('h3', {}, 'The file'), h('div', { class: 'tools' }, R.title)), h('div', { class: 'body' }, R.editor,
          h('div', { class: 'btns', style: { marginTop: '10px' } }, h('button', { class: 'btn primary', id: 'vehicle-validate', onclick: validate }, 'Validate'), R.saveName, h('button', { class: 'btn', onclick: async () => { try { const r = await api('/api/vehicle/save', { name: R.saveName.value.trim(), ...body() }); toast(`Saved: ${r.saved}`, 'ok'); const v = await api('/api/vehicles'); window.__S.vehicles = v.vehicles; list(window.__S); } catch (e) { toast(e.message, 'warn', 8000); } } }, 'Save copy'), h('button', { class: 'btn', id: 'vehicle-use', onclick: useForRig }, 'Use for the rig')),
          R.errs, R.out))),
      h('article', { class: 'card' }, h('header', {}, h('h3', {}, 'Preview flight'), h('div', { class: 'tools' }, h('span', { class: 'note' }, 'tfc_fly: three flight functions and the real ACT logic fly it from T-zero'))),
        h('div', { class: 'body' }, h('div', { style: { display: 'flex', gap: '10px', alignItems: 'center', flexWrap: 'wrap' } }, R.sensors, h('label', { class: 'note' }, 'pad frames ', R.pad), h('button', { class: 'btn primary', id: 'vehicle-preview', onclick: preview }, icon('play'), 'Fly the preview')), R.preview))));
  },
  update(S) {
    window.__S = S;
    const s = S.snap; if (!s || !S.cfg) return;
    if (!R.built) { R.built = true; list(S); load('reference', S); }
    const a = s.vehicle && s.vehicle.active;
    R.active.innerHTML = '';
    if (a) R.active.append(icon('info'), h('div', {}, h('b', {}, 'The rig’s vehicle: '), a.name, Object.keys(a.knobs).length ? ` with ${Object.keys(a.knobs).length} knob${Object.keys(a.knobs).length > 1 ? 's' : ''} changed` : '', '. It takes effect when the rig is next started (Rig tab). ',
      h('span', { class: 'note' }, 'The flight computers in the rig carry the pitch program and gains of the reference vehicle, compiled in: another vehicle needs tfc_gen_tables --vehicle and a rebuild of the firmware, so the preview flies any vehicle, but the rig is only right for the reference one and its plant departures.')));
  },
};

async function useForRig() {
  const warn = R.name !== 'reference';
  if (warn) {
    const r = await modal({ title: 'Use this vehicle for the rig?', body: h('p', { style: { margin: 0 } }, 'The flight computers in the rig carry the reference vehicle’s tables. They will fly this vehicle with the wrong program and gains; the run shows how they cope, it does not show a designed controller. To fly it properly, generate its tables (tfc_gen_tables --vehicle FILE) and rebuild the firmware.'), buttons: [{ label: 'Cancel', value: 'no' }, { label: 'Use it anyway', kind: 'warn', value: 'yes' }] });
    if (r !== 'yes') return;
  }
  try { await api('/api/vehicle/active', { name: R.name, knobs: R.knobs }); toast(`The rig will fly ${R.name} the next time it starts.`, 'ok'); } catch (e) { toast(e.message, 'warn', 8000); }
}
