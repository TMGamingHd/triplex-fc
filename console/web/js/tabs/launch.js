// Launch: the checklist, the countdown, the sequence, and the one command that matters. The go/no-go is the launch checklist's own rule (tfc_peers/launch.py); the flight computers decide again.
import { h, setText, clock, num, store, modal, toast, icon, NODES } from '../util.js';
import { sequence } from '../derive.js';
import { sendCommand } from '../ops.js';
import { api } from '../net.js';

let R = {};
const MANUAL = [
  ['clear', 'Everyone is clear of the platform and nobody is within reach (real rig)'],
  ['estop', 'The E-stop is within the operator’s reach (real rig)'],
  ['disarm', 'The INJECTOR-DISARM switch is in the disarmed position, if fitted (real rig)'],
  ['consoles', 'The consoles have been read: no LATCHED OUT, no SAFE REQUESTED (Events tab)'],
];

const callApi = async (fn, ok) => { try { const r = await fn(); if (ok) toast(ok, 'ok', 3500); return r; } catch (e) { toast(e.message, 'warn', 9000); } };
const STATE = { ready: ['ok', 'flight computers ready'], 'needs-build': ['warn', 'flight computers to be built'], building: ['warn', 'building…'], failed: ['crit', 'build failed'], 'cannot-build': ['crit', 'cannot build here'], idle: ['muted', '…'] };

// The vehicle the rig flies. Choosing one builds the three flight computers with that vehicle's own pitch program and gains (console/tfc_console/rigbuild.py); the rig then starts them with the simulator flying the same vehicle.
function vehicleCard() {
  R.vsel = h('select', { id: 'rig-vehicle', 'aria-label': 'the vehicle the rig flies', onchange: () => { choose(R.vsel.value); } });
  R.vchip = h('span', { id: 'rv-state', class: 'chip muted' }, '—');
  R.vsteps = h('ol', { class: 'timeline', id: 'rv-steps' });
  R.vlog = h('pre', { class: 'mono note', id: 'rv-log', style: { maxHeight: '140px', overflow: 'auto', whiteSpace: 'pre-wrap', margin: '8px 0 0' } });
  R.vnote = h('p', { class: 'note', id: 'rv-note', style: { margin: '8px 0 0' } });
  R.vgo = h('button', { class: 'btn go', id: 'rv-start', onclick: () => callApi(() => api('/api/rig/start', { profile: 'closed-loop' }), 'Rig started: the flight computers are calibrating') }, icon('play'), h('span', { style: { whiteSpace: 'nowrap' } }, 'Start the rig'));
  R.vstop = h('button', { class: 'btn danger', id: 'rv-stop', onclick: () => callApi(() => api('/api/rig/stop', {}), 'Rig stopped') }, 'Stop the rig');
  R.vbuild = h('button', { class: 'btn primary', id: 'rv-build', onclick: () => choose(R.vsel.value) }, 'Build the flight computers');
  return h('article', { class: 'card', id: 'rig-vehicle-card' },
    h('header', {}, h('h3', {}, 'Vehicle on the rig'), h('div', { class: 'tools' }, R.vchip)),
    h('div', { class: 'body' },
      h('div', { class: 'btns', style: { alignItems: 'center', marginBottom: '8px' } }, R.vsel, R.vbuild),
      h('div', { class: 'btns', style: { flexWrap: 'wrap' } }, R.vgo, R.vstop, h('button', { class: 'btn', type: 'button', id: 'rv-3d', onclick: () => { window.open('/viewer/?token=' + encodeURIComponent(sessionStorage.getItem('tfc.token') || ''), '_blank'); } }, 'Open the 3D view')),
      R.vnote, R.vsteps, R.vlog));
}

async function choose(name) {
  const S = R.S, knobs = S.snap.vehicle && S.snap.vehicle.active.name === name ? S.snap.vehicle.active.knobs : {};
  await callApi(() => api('/api/rig/vehicle', { name, knobs }), `Vehicle: ${name}`);
}

function vehicleUpdate(S) {
  const s = S.snap, b = s.rigbuild || {}, act = (s.vehicle && s.vehicle.active) || { name: 'reference' }, rg = s.rig || {};
  const running = (rg.procs || []).some((p) => p.state === 'running' || p.state === 'frozen');
  const names = (S.vehicles || []).map((v) => v.name);
  const key = JSON.stringify([names, act.name]);
  if (R.vkey !== key) { R.vkey = key; R.vsel.replaceChildren(...names.map((n) => h('option', { value: n, selected: n === act.name }, n))); }
  if (document.activeElement !== R.vsel) R.vsel.value = act.name;
  const [cls, txt] = STATE[b.state] || STATE.idle;
  R.vchip.className = `chip ${cls}`; setText(R.vchip, txt + (b.state === 'building' ? ` ${b.seconds} s` : ''));
  R.vsel.disabled = running || b.state === 'building';
  R.vbuild.disabled = running || b.state === 'building' || b.state === 'ready' || b.state === 'cannot-build';
  R.vgo.disabled = !(b.state === 'ready') || running || !!rg.disabled;
  R.vstop.disabled = !running;
  setText(R.vnote, running ? `The rig is running with ${rg.profile === 'closed-loop' ? act.name : 'its own'} vehicle. Stop it to choose another.`
    : b.state === 'ready' ? (b.reference ? 'The reference vehicle: its flight computers are the committed build.' : `The flight computers for ${act.name} are built: their pitch program and gains were designed on this vehicle. Start the rig, wait for the three computers to calibrate, and launch.`)
    : b.state === 'needs-build' ? `${act.name}'s flight computers are not built yet. Building takes about half a minute and uses the real flight code.`
    : b.state === 'failed' ? (b.error || 'the build failed') : b.state === 'cannot-build' ? (b.error || '') : '');
  const sk = JSON.stringify([b.steps, (b.log || []).length]);
  if (R.vsk !== sk) {
    R.vsk = sk;
    R.vsteps.replaceChildren(...(b.state === 'building' || b.state === 'failed' ? (b.steps || []) : []).map((x) => h('li', { class: x.state }, h('span', { class: 'mark' }, x.state === 'done' ? '✓' : x.state === 'failed' ? '✗' : ''), h('div', {}, h('div', {}, x.name), h('div', { class: 'sub' }, x.detail || '')))));
    setText(R.vlog, b.state === 'failed' || b.state === 'building' ? (b.log || []).slice(-12).join('\n') : '');
    R.vlog.style.display = R.vlog.textContent ? '' : 'none';
  }
}

function checks() { return store.get('launch.checks', {}); }

async function confirmLaunch(S) {
  const unchecked = MANUAL.filter(([k]) => !checks()[k]);
  let input;
  const body = [
    h('p', { style: { margin: 0 } }, 'This sends an authenticated ', h('b', {}, 'ARM'), ', then the ', h('b', {}, 'EXECUTE'), ' 50 ms later, on ', h('b', { class: 'mono' }, S.snap.commands.iface || '?'), '. The sync master acts after its own go/no-go and starts a 10 s countdown, which you can scrub until T-zero.'),
    unchecked.length ? h('div', { class: 'banner warn' }, icon('alert'), h('div', {}, h('b', {}, 'Not ticked: '), unchecked.map(([, t]) => t).join('; '), '. Fine on the virtual rig; on the real rig do not launch.')) : null,
    h('label', { class: 'field' }, 'Type LAUNCH to confirm', input = h('input', { type: 'text', autocomplete: 'off', spellcheck: 'false', id: 'launch-confirm', placeholder: 'LAUNCH' })),
  ];
  const r = await modal({
    title: 'Launch: second confirmation', body, buttons: [{ label: 'Cancel', value: 'cancel' }, { label: 'ARM and LAUNCH', kind: 'go', value: 'go', id: 'launch-go', disabled: true }],
    onOpen: ({ foot }) => { const go = foot.querySelector('#launch-go'); input.addEventListener('input', () => { go.disabled = input.value.trim() !== 'LAUNCH'; }); input.focus(); },
  });
  if (r === 'go') await sendCommand({ op: 'launch', mode: 'armed', confirmed: true });
}

export default {
  id: 'launch', label: 'Launch', icon: 'launch',
  mount(root) {
    R.clock = h('div', { class: 'mono', style: { fontSize: '56px', fontWeight: 800, lineHeight: 1.05, textAlign: 'center' } }, '--:--.-');
    R.phase = h('div', { class: 'note', style: { textAlign: 'center', textTransform: 'uppercase', letterSpacing: '.12em' } });
    R.prog = h('i', { style: { width: '0%' } });
    R.big = h('div', { class: 'big-status hold' }, '—');
    R.reasons = h('ul', { class: 'note', style: { margin: '8px 0 0', paddingLeft: '18px' } });
    R.timeline = h('ol', { class: 'timeline' });
    R.gng = h('tbody');
    R.manual = h('div');
    for (const [k, t] of MANUAL) R.manual.append(h('label', { class: 'check' }, h('input', { type: 'checkbox', dataset: { k }, checked: !!checks()[k], onchange: (e) => { const c = checks(); c[k] = e.target.checked; store.set('launch.checks', c); } }), h('span', {}, t)));
    R.auto = h('div', { class: 'alerts' });
    R.launch = h('button', { class: 'btn go big', id: 'btn-launch', onclick: () => confirmLaunch(R.S) }, icon('launch'), h('span', {}, 'ARM and LAUNCH…'));
    R.scrub = h('button', { class: 'btn danger big', id: 'btn-scrub', onclick: async () => { await sendCommand({ op: 'scrub', mode: 'single' }); } }, 'SCRUB');
    R.why = h('div', { class: 'note' });
    R.cmdlog = h('div', { class: 'note' });
    R.cal = h('div', { class: 'stats' });
    const countdownCard = h('article', { class: 'card' }, h('header', {}, h('h3', {}, 'Countdown')), h('div', { class: 'body' }, R.clock, R.phase, h('div', { class: 'progress', style: { margin: '12px 0' } }, R.prog), R.big, R.reasons));
    const sequenceCard = h('article', { class: 'card' }, h('header', {}, h('h3', {}, 'Sequence')), h('div', { class: 'body' }, R.timeline));
    const controlCard = h('article', { class: 'card' },
      h('header', {}, h('h3', {}, 'Launch control'), h('div', { class: 'tools' }, h('span', { id: 'l-iface', class: 'chip muted' }, '—'))),
      h('div', { class: 'body' },
        h('div', { class: 'btns', style: { marginBottom: '12px' } }, R.launch, R.scrub), R.why,
        h('div', { class: 'sep' }),
        h('div', { class: 'note', style: { marginBottom: '4px' } }, h('b', {}, 'Before the command'), ' (the human half of P-S2-02; the first four are yours to tick):'), R.manual,
        h('div', { class: 'sep' }), R.auto, h('div', { class: 'sep' }), R.cmdlog));
    const gngTable = h('table', {}, h('thead', {}, h('tr', {}, h('th', { style: { width: '28px' } }), h('th', {}, 'Item'), h('th', {}, 'State'))), R.gng);
    const gngCard = h('article', { class: 'card' }, h('header', {}, h('h3', {}, 'Go / no-go'), h('div', { class: 'tools' }, h('span', { class: 'note' }, 'the checklist’s rule, evaluated by the console'))), h('div', { class: 'body flush' }, gngTable));
    const calCard = h('article', { class: 'card' }, h('header', {}, h('h3', {}, 'Pad calibration')),
      h('div', { class: 'body' }, R.cal, h('p', { class: 'note', style: { marginBottom: 0 } }, 'Each computer averages its own IMU for 10 s of rest, subtracts the bias, and reports ready. A platform that is moved on the pad restarts the calibration: that is the check.')));
    root.append(h('div', { class: 'grid c-1-2' }, h('div', { class: 'stack' }, vehicleCard(), countdownCard, sequenceCard), h('div', { class: 'stack' }, controlCard, gngCard, calCard)));
  },
  update(S) {
    R.S = S;
    const s = S.snap; if (!s) return;
    vehicleUpdate(S);
    const ph = s.phase;
    setText(R.clock, ph.name === 'countdown' ? `T-${clock(ph.t_minus_s)}` : ph.name === 'flight' ? `T+${clock(ph.flight_s)}` : ph.name === 'pad' ? 'PAD' : '--:--.-');
    R.clock.style.color = ph.name === 'countdown' ? 'var(--warn)' : ph.name === 'flight' ? 'var(--ok)' : 'var(--text)';
    setText(R.phase, { pad: 'on the pad: waiting for go', countdown: 'countdown running', flight: 'in flight', 'no-sync': 'no sync: nothing is running' }[ph.name]);
    R.prog.style.width = ph.name === 'countdown' ? `${((10 - ph.t_minus_s) / 10) * 100}%` : ph.name === 'flight' ? '100%' : '0%';
    const go = s.go_nogo.go;
    R.big.className = `big-status ${ph.name === 'flight' ? 'go' : go ? 'go' : ph.name === 'no-sync' ? 'hold' : 'nogo'}`;
    setText(R.big, ph.name === 'flight' ? 'LIFT-OFF' : ph.name === 'countdown' ? (go ? 'GO — COUNTING' : 'NO-GO — SCRUBS') : go ? 'GO FOR LAUNCH' : ph.name === 'no-sync' ? 'WAITING' : 'NO-GO');
    R.reasons.innerHTML = '';
    if (!go && ph.name !== 'flight') for (const r of s.go_nogo.reasons.slice(0, 6)) R.reasons.append(h('li', {}, r));
    // the sequence
    const sq = sequence(S), sk = JSON.stringify(sq);
    if (R.sk !== sk) {
      R.sk = sk; R.timeline.innerHTML = '';
      const t0 = (s.milestones || {}).t0;
      for (const x of sq) R.timeline.append(h('li', { class: x.state }, h('span', { class: 'mark' }, x.state === 'done' ? '✓' : x.state === 'failed' ? '✗' : ''), h('div', {}, h('div', {}, x.label), h('div', { class: 'sub' }, x.detail)), h('span', { class: 't' }, x.t != null && t0 != null ? `T${x.t - t0 >= 0 ? '+' : '-'}${Math.abs(x.t - t0).toFixed(1)}` : '')));
    }
    // the go/no-go table
    const gk = JSON.stringify(s.go_nogo.items.map((i) => [i.key, i.ok, i.detail]));
    if (R.gk !== gk) {
      R.gk = gk; R.gng.innerHTML = '';
      for (const it of s.go_nogo.items) R.gng.append(h('tr', {}, h('td', { class: it.ok ? 's-ok' : it.ok === false ? (it.required ? 's-crit' : 's-warn') : 's-muted', style: { fontWeight: 700 } }, it.ok ? '✓' : it.ok === false ? '✗' : '–'), h('td', {}, it.label, it.required ? '' : h('span', { class: 'note' }, '  (informs, does not decide)')), h('td', { class: 'mono' }, it.detail)));
    }
    // the controls
    const c = s.commands || {};
    const ic = document.getElementById('l-iface'); ic.className = `chip ${c.can_send ? 'ok' : 'muted'}`; setText(ic, c.can_send ? `sends on ${c.iface}` : 'cannot send');
    R.launch.disabled = !(c.can_send && go && ph.name === 'pad');
    R.scrub.disabled = !(c.can_send && ph.name === 'countdown');
    const mainNote = s.master && c.can_send ? ` Main computer: ${s.master}${s.master === 'A' ? '' : ' (A is gone)'}; it acts on the launch command, after its own go/no-go.` : '';
    setText(R.why, (!c.can_send ? 'This console is not attached to a live bus (a replay is read-only), so it cannot send a command.' : ph.name === 'flight' ? 'The vehicle has left the pad: a launch is over and a scrub is refused.' : ph.name === 'countdown' ? 'The countdown is running. SCRUB returns the vehicle to the pad (one click, no confirmation: it is the safe direction).' : go ? 'Every required item holds. The launch is a two-step command: you confirm it, then the console sends ARM and EXECUTE.' : 'NO-GO: the button stays disabled until the checklist passes.') + mainNote);
    // the automatic pre-launch items (steps 5 and 6 of P-S2-02)
    const bad = s.nodes.map((n) => ({ n, c: n.console || {} }));
    const consoleSeen = bad.some((b) => b.c.mode);
    const autos = [
      ['No bad frames on any computer (crc, seq, vote, digest = 0)', consoleSeen ? bad.every((b) => !(b.c.crc || b.c.seq || b.c.vote || b.c.digest)) : null, consoleSeen ? bad.map((b) => `${b.n.name}: crc ${b.c.crc ?? '–'} seq ${b.c.seq ?? '–'} vote ${b.c.vote ?? '–'} digest ${b.c.digest ?? '–'}`).join(' · ') : 'no node console attached: counters unknown'],
      ['The vehicle is clamped on the pad (altitude 0)', ph.name === 'flight' ? null : s.truth ? s.truth.clamped === 1 : s.sim ? s.sim.alt < 1 : null, ph.name === 'flight' ? 'not applicable: the vehicle has left the pad' : s.truth ? `altitude ${s.truth.alt.toFixed(1)} m` : s.sim ? `altitude ${s.sim.alt.toFixed(0)} m` : 'no simulator'],
      ['Every node is healthy in every other node’s view', s.nodes.every((n) => n.health === 'healthy'), s.nodes.map((n) => `${n.name} ${n.health}`).join(' · ')],
    ];
    const ak = JSON.stringify(autos);
    if (R.ak !== ak) { R.ak = ak; R.auto.innerHTML = ''; for (const [t, ok, d] of autos) R.auto.append(h('div', { class: `alert ${ok === false ? 'warn' : ''}` }, h('span', { class: `lv ${ok ? 's-ok' : ''}`, style: { width: '24px' } }, ok ? '✓' : ok === false ? '✗' : '–'), h('div', {}, h('div', {}, t), h('div', { class: 'note' }, d)))); }
    // what became of the last launch-related command
    const log = (c.log || []).filter((r) => ['launch', 'scrub'].includes(r.op)).slice(-1)[0];
    setText(R.cmdlog, log ? `Last: ${log.mode === 'armed' ? 'ARM + ' : ''}${log.op} — ${log.status}${log.responses.length ? ' (' + log.responses.map((r) => `${r.src}: ${r.result}`).join('; ') + ')' : ''}` : 'No launch command sent in this session.');
    // calibration
    const ck = JSON.stringify(s.nodes.map((n) => [n.ready, n.alive]));
    if (R.ck !== ck) { R.ck = ck; R.cal.innerHTML = ''; for (const n of s.nodes) R.cal.append(h('div', { class: `stat ${n.ready ? 'ok' : n.alive ? 'warn' : 'dim'}` }, h('div', { class: 'k' }, `FC-${n.name}`), h('div', { class: 'v' }, n.ready ? 'READY' : n.alive ? 'calibrating' : 'silent'))); }
  },
};
