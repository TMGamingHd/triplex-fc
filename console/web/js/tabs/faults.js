// Faults: break the rig on purpose and watch what the system does about it. Three ways, by what is on the bench:
// the virtual peers (any of the 32 fault kinds on B and C, changed while they run), the processes of the virtual rig (kill, freeze, restart a node), and the injector's relays (the real thing, on the rig).
import { h, setText, toast, icon, NODES } from '../util.js';
import { api } from '../net.js';

let R = { params: {} };
const GROUPS = [
  ['Sensor values', ['bias', 'drift', 'spike', 'scale', 'noise', 'stuck', 'saturate', 'zero', 'clip', 'invert', 'swap', 'oscillate', 'repeat', 'bitflip', 'stuckbit']],
  ['Command and state', ['cmd_offset', 'cmdstuck', 'cmdinvert', 'digest']],
  ['Frames and timing', ['dropout', 'partial', 'corrupt', 'late', 'early', 'jitter', 'clockdrift', 'duplicate', 'replay', 'seqgap', 'seqstuck', 'reboot']],
  ['The bus', ['babble']],
];
const PRESETS = [
  ['B: gyro bias 3 dps', 'B:bias:mag=3'], ['B: gyro drifts away', 'B:drift:rate=0.05'], ['C: sensors freeze', 'C:stuck'], ['B: fail-silent', 'B:dropout'], ['C: bus babble', 'C:babble:n=5'],
  ['B: command off by 1°', 'B:cmd_offset:mag=1'], ['C: flaky link (1 frame in 3)', 'C:corrupt:period=3,duty=1,p=1'], ['B: reboots (50 frames)', 'B:reboot:down=50'], ['C: spikes (survivable)', 'C:spike:p=0.05'],
];

function specOf(node, kind, S) {
  const meta = S.cfg.faults.find((f) => f.kind === kind); if (!meta) return '';
  const opts = [];
  for (const [k, def] of Object.entries(meta.params)) { const v = R.params[k]?.value; if (v !== undefined && v !== '' && String(v) !== String(def)) opts.push(`${k}=${v}`); }
  const per = R.period.value, duty = R.duty.value;
  if (per) { opts.push(`period=${per}`); if (duty) opts.push(`duty=${duty}`); }
  return `${node}:${kind}${opts.length ? ':' + opts.join(',') : ''}`;
}

function buildParams(S) {
  const meta = S.cfg.faults.find((f) => f.kind === R.kind.value);
  R.paramBox.innerHTML = ''; R.params = {};
  if (!meta) return;
  for (const [k, def] of Object.entries(meta.params)) {
    const inp = k === 'sensor' ? h('select', {}, ['gyro', 'accel'].map((x) => h('option', { value: x }, x))) : h('input', { type: 'number', step: 'any' });
    inp.value = def; inp.addEventListener('input', () => updateSpec(S));
    R.params[k] = inp;
    R.paramBox.append(h('label', { class: 'field' }, k, inp));
  }
  setText(R.desc, `${meta.row}: ${meta.desc}`);
  const ex = S.cfg.expect[R.kind.value];
  setText(R.expect, ex ? `Expected (sim/README): caught by ${ex[0]}; mode afterwards ${ex[1]}.` : '');
  updateSpec(S);
}
function updateSpec(S) { setText(R.spec, specOf(R.node(), R.kind.value, S)); }

async function inject(spec, frames) {
  try { const rec = await api('/api/faults/add', { spec, frames: frames || null }); toast(`Injected: ${rec.spec}`, 'warn', 4000); }
  catch (e) { toast(e.message, 'crit', 8000); }
}

export default {
  id: 'faults', label: 'Faults', icon: 'faults',
  mount(root) {
    let cur = 'B';
    R.node = () => cur;
    R.seg = h('div', { class: 'seg', role: 'group', 'aria-label': 'Faulty node' }, ['B', 'C'].map((n) => h('button', { type: 'button', 'aria-pressed': n === cur ? 'true' : 'false', dataset: { v: n }, onclick: () => { cur = n; for (const b of R.seg.children) b.setAttribute('aria-pressed', b.dataset.v === n ? 'true' : 'false'); if (R.S) updateSpec(R.S); } }, `FC-${n}`)));
    R.kind = h('select', { id: 'fault-kind', 'aria-label': 'Fault kind' });
    R.kind.addEventListener('change', () => buildParams(R.S));
    R.paramBox = h('div', { class: 'grid', style: { gridTemplateColumns: 'repeat(auto-fill, minmax(110px, 1fr))' } });
    R.period = h('input', { type: 'number', min: 2, placeholder: 'every N frames', id: 'fault-period' }); R.duty = h('input', { type: 'number', min: 1, placeholder: 'active K', id: 'fault-duty' });
    for (const e of [R.period, R.duty]) e.addEventListener('input', () => R.S && updateSpec(R.S));
    R.frames = h('input', { type: 'number', min: 1, value: 300, id: 'fault-frames', style: { width: '90px' } });
    R.forever = h('input', { type: 'checkbox', checked: true, id: 'fault-forever' });
    R.desc = h('div', { class: 'note' }); R.expect = h('div', { class: 'note' }); R.spec = h('code', { class: 'mono' }, '');
    R.lab = h('div', { class: 'note' });
    R.table = h('tbody');
    R.proc = h('tbody');
    R.pico = h('div');
    R.presets = h('div', { class: 'btns' }, PRESETS.map(([t, spec]) => h('button', { class: 'btn small', onclick: () => inject(spec, null), dataset: { spec } }, t)));
    root.append(h('div', { class: 'stack' },
      h('div', { class: 'banner warn' }, icon('alert'), h('div', {}, h('b', {}, 'Test actions. '), 'Faults are injected on purpose, to measure what the system does. A node the fault gets latched out stays out until you readmit it on the Commands tab, whatever you do to the fault afterwards.')),
      h('article', { class: 'card' }, h('header', {}, h('h3', {}, 'Virtual peers: any fault kind on B or C'), h('div', { class: 'tools' }, R.lab)),
        h('div', { class: 'body' },
          h('div', { class: 'grid c-1-2' },
            h('div', { class: 'stack' },
              h('label', { class: 'field' }, 'Node', R.seg), h('label', { class: 'field' }, 'Kind', R.kind), R.desc, R.expect,
              h('div', { class: 'grid cols2' }, h('label', { class: 'field' }, 'Intermittent: period', R.period), h('label', { class: 'field' }, 'duty', R.duty)),
              h('div', { style: { display: 'flex', gap: '10px', alignItems: 'center' } }, h('label', { class: 'check', style: { padding: 0 } }, R.forever, h('span', {}, 'until cleared')), h('span', { class: 'note' }, 'or for'), R.frames, h('span', { class: 'note' }, 'frames (100 = 1 s)')),
              h('div', { class: 'mono-box' }, R.spec),
              h('div', { class: 'btns' }, h('button', { class: 'btn danger', id: 'fault-inject', onclick: () => inject(R.spec.textContent, R.forever.checked ? null : +R.frames.value) }, icon('faults'), 'Inject'))),
            h('div', { class: 'stack' }, h('div', { class: 'note' }, h('b', {}, 'Parameters')), R.paramBox, h('div', { class: 'sep' }), h('div', { class: 'note' }, h('b', {}, 'Scenarios')), R.presets)))),
      h('article', { class: 'card' }, h('header', {}, h('h3', {}, 'Faults in the scenario'), h('div', { class: 'tools' }, h('button', { class: 'btn small', id: 'fault-clear-all', onclick: async () => { try { await api('/api/faults/clear', { id: 'all' }); toast('All faults cleared', 'info'); } catch (e) { toast(e.message, 'warn'); } } }, 'Clear all'))),
        h('div', { class: 'body flush' }, h('table', {}, h('thead', {}, h('tr', {}, h('th', {}, '#'), h('th', {}, 'node'), h('th', {}, 'fault'), h('th', {}, 'window'), h('th', {}, 'detected (measured by this console)'), h('th', {}, ''))), R.table),
          h('div', { class: 'note', style: { padding: '8px 12px' } }, 'Detection is measured here: the first frame at which the computers (their heartbeats, or their own console) call the node latched, minus the frame the fault started in. The heartbeat view is sampled once per frame, so it can read one frame late.'))),
      h('div', { class: 'grid cols2' },
        h('article', { class: 'card' }, h('header', {}, h('h3', {}, 'Processes of the virtual rig'), h('div', { class: 'tools' }, h('span', { class: 'note' }, 'the live tests’ way of breaking a node'))), h('div', { class: 'body flush' }, h('table', {}, h('thead', {}, h('tr', {}, h('th', {}, 'process'), h('th', {}, 'state'), h('th', {}, 'test action'))), R.proc))),
        h('article', { class: 'card' }, h('header', {}, h('h3', {}, 'Injector (Pico relays)'), h('div', { class: 'tools' }, h('span', { class: 'chip muted', id: 'fault-pico-chip' }, 'not connected'))), h('div', { class: 'body' }, R.pico)))));
  },
  update(S) {
    R.S = S;
    const s = S.snap; if (!s || !S.cfg) return;
    if (!R.built) {
      R.built = true;
      for (const [g, kinds] of GROUPS) R.kind.append(h('optgroup', { label: g }, kinds.filter((k) => S.cfg.faults.some((f) => f.kind === k)).map((k) => h('option', { value: k }, k))));
      R.kind.value = 'bias'; buildParams(S);
    }
    const f = s.faults || {};
    R.lab.className = `chip ${f.available ? 'ok' : 'muted'}`; setText(R.lab, f.available ? 'fault lab running' : 'fault lab not running: start it on the Rig tab');
    for (const b of document.querySelectorAll('#pane-faults #fault-inject, #pane-faults [data-spec], #pane-faults #fault-clear-all')) b.disabled = !f.available;
    const key = JSON.stringify(f.table || []) + (f.frame != null ? Math.floor(f.frame / 25) : '');
    if (R.fkey !== key) {
      R.fkey = key; R.table.innerHTML = '';
      if (!(f.table || []).length) R.table.append(h('tr', {}, h('td', { colspan: 6, class: 'empty' }, 'No fault is in the scenario.')));
      for (const x of f.table || []) {
        const d = x.detected;
        R.table.append(h('tr', {}, h('td', { class: 'mono' }, x.id), h('td', {}, h('span', { class: `tag n${x.node}` }, x.node)), h('td', { class: 'mono', style: { fontSize: '12px' } }, x.spec),
          h('td', {}, h('span', { class: `chip ${x.active ? 'warn' : 'muted'}` }, x.active ? 'active' : 'ended'), ' ', h('span', { class: 'note' }, `from frame ${x.start}${x.end != null ? ' to ' + x.end : ''}`)),
          h('td', {}, d ? h('span', {}, h('b', { class: 's-ok' }, `+${d.frames_after} frames`), h('span', { class: 'note' }, ` (frame ${d.frame}, by the ${d.by}${d.reason ? ': ' + d.reason : ''})`)) : x.active ? h('span', { class: 'note' }, 'not detected (yet)') : h('span', { class: 'note' }, 'never latched')),
          h('td', {}, h('button', { class: 'btn small', onclick: async () => { try { await api('/api/faults/clear', { id: String(x.id) }); } catch (e) { toast(e.message, 'warn'); } } }, 'Clear'))));
      }
    }
    // processes
    const rg = s.rig || {}, procs = rg.procs || [];
    const pk = JSON.stringify(procs.map((p) => [p.name, p.state]));
    if (R.pk !== pk) {
      R.pk = pk; R.proc.innerHTML = '';
      if (!procs.length) R.proc.append(h('tr', {}, h('td', { colspan: 3, class: 'empty' }, 'No rig is running. Start one on the Rig tab.')));
      const act = (name, a, label, cls = '') => h('button', { class: `btn small ${cls}`, onclick: async () => { try { await api('/api/rig/proc', { name, action: a }); } catch (e) { toast(e.message, 'warn'); } } }, label);
      for (const p of procs) {
        const live = p.state === 'running', frozen = p.state === 'frozen';
        R.proc.append(h('tr', {}, h('td', {}, h('span', { class: `tag n${p.name}` }, p.name), ' ', h('span', { class: 'note' }, p.title)), h('td', {}, h('span', { class: `chip ${live ? 'ok' : frozen ? 'warn' : 'muted'}` }, p.state)),
          h('td', {}, h('div', { class: 'btns' }, live || frozen ? act(p.name, 'kill', 'Kill', 'danger') : null, live ? act(p.name, 'freeze', 'Freeze', 'warn') : null, frozen ? act(p.name, 'resume', 'Resume') : null, act(p.name, 'restart', 'Restart')))));
      }
    }
    // the injector
    const hw = s.hardware || {}, pico = hw.pico;
    const pc = document.getElementById('fault-pico-chip'); pc.className = `chip ${pico && pico.connected ? 'ok' : 'muted'}`; setText(pc, pico ? (pico.connected ? 'connected' : 'lost') : 'not connected');
    const pkey = JSON.stringify([!!pico, pico && pico.status && pico.status.relays]);
    if (R.pkey !== pkey) {
      R.pkey = pkey; R.pico.innerHTML = '';
      if (!pico) R.pico.append(h('p', { class: 'note', style: { marginTop: 0 } }, 'No Pico is connected. On the rig, connect it on the Rig tab: the injector cuts a node’s power with a relay for a time you choose (it ends by itself, so a crash of this console cannot leave a node cut). Not run on a board yet.'));
      else {
        const node = h('select', { id: 'pico-node' }, ['A', 'B', 'C', 'ACT'].map((n) => h('option', { value: n }, n)));
        const ms = h('input', { type: 'number', value: 3000, min: 1, max: 30000, step: 100, id: 'pico-ms', style: { width: '90px' } });
        R.pico.append(h('div', { style: { display: 'flex', gap: '8px', alignItems: 'center', flexWrap: 'wrap' } }, 'Cut', node, 'for', ms, 'ms', h('button', { class: 'btn danger', onclick: async () => { try { await api('/api/pico', { op: 'cut', node: node.value, ms: +ms.value }); } catch (e) { toast(e.message, 'warn'); } } }, 'Cut power'), h('button', { class: 'btn', onclick: async () => { try { await api('/api/pico', { op: 'restore', node: 'all' }); } catch (e) { toast(e.message, 'warn'); } } }, 'Restore all')),
          (R.picoText = h('p', { class: 'note' }, '')));
      }
    }
    if (R.picoText) setText(R.picoText, pico && pico.status ? pico.status.text : 'waiting for the Pico’s status…');
  },
};
