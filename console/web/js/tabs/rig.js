// Rig: where the data comes from (a live interface or a recording), the virtual rig's processes, and the serial hardware for when the parts are on the bench.
import { h, setText, toast, icon, clear, NODES } from '../util.js';
import { api } from '../net.js';

let R = { src: null, srcAt: 0 };
const bytes = (n) => (n > 1e6 ? (n / 1e6).toFixed(1) + ' MB' : (n / 1e3).toFixed(0) + ' kB');
const call = async (fn, ok) => { try { const r = await fn(); if (ok) toast(ok, 'ok', 3500); return r; } catch (e) { toast(e.message, 'warn', 8000); } };

async function refreshSource() {
  R.srcAt = performance.now();
  try { R.src = await api('/api/source'); R.srcKey = null; } catch { /* the header already says the server is gone */ }
}

function profileCard(key, p, rig) {
  const running = rig.profile === key && rig.procs.some((x) => x.state === 'running' || x.state === 'frozen');
  const rows = rig.profile === key ? rig.procs : [];
  return h('article', { class: 'card', dataset: { profile: key } },
    h('header', {}, h('h3', {}, p.title), h('div', { class: 'tools' }, p.ready ? h('span', { class: 'chip ok' }, 'built') : h('span', { class: 'chip warn', title: p.missing.join(', ') }, `not built: ${p.missing.join(', ')}`))),
    h('div', { class: 'body' }, h('p', { class: 'note', style: { marginTop: 0 } }, p.doc),
      h('div', { class: 'btns', style: { marginBottom: '10px' } },
        h('button', { class: 'btn go', id: `rig-start-${key}`, disabled: !p.ready || (rig.procs.some((x) => x.state === 'running' || x.state === 'frozen')), onclick: () => call(() => api('/api/rig/start', { profile: key }), `Starting ${p.title}`) }, icon('play'), 'Start'),
        h('button', { class: 'btn danger', id: `rig-stop-${key}`, disabled: !running, onclick: () => call(() => api('/api/rig/stop', {}), 'Rig stopped') }, 'Stop all')),
      rows.length ? h('div', { class: 'tablewrap' }, h('table', {}, h('thead', {}, h('tr', {}, h('th', {}, 'process'), h('th', {}, 'state'), h('th', { class: 'num' }, 'pid'), h('th', { class: 'num' }, 'up'), h('th', { class: 'num' }, 'lines'), h('th', { class: 'num' }, 'restarts'))),
        h('tbody', {}, rows.map((x) => h('tr', {}, h('td', {}, h('span', { class: `tag n${x.name}` }, x.name), ' ', h('span', { class: 'note' }, x.title)), h('td', {}, h('span', { class: `chip ${x.state === 'running' ? 'ok' : x.state === 'frozen' ? 'warn' : x.state === 'exited' ? 'crit' : 'muted'}` }, x.state)),
          h('td', { class: 'num' }, x.pid ?? '—'), h('td', { class: 'num' }, x.uptime != null ? `${x.uptime.toFixed(0)} s` : '—'), h('td', { class: 'num' }, x.lines), h('td', { class: 'num' }, x.restarts)))))) : h('div', { class: 'note' }, `Processes: ${p.procs.join(', ')}`)));
}

export default {
  id: 'rig', label: 'Rig', icon: 'rig',
  mount(root) {
    R.source = h('div'); R.profiles = h('div', { class: 'grid cols2' }); R.hw = h('div'); R.msg = h('div', { class: 'note' });
    R.rigNote = h('div', { class: 'banner info' }, icon('info'), h('div', {}, 'Both profiles run on ', h('b', { class: 'mono' }, 'vcan0'), ' (create it once per boot with ', h('code', {}, 'sim/scripts/setup_vcan.sh'), '). The processes are children of this console and stop with it. Their consoles appear on the Events tab.'));
    root.append(h('div', { class: 'stack' }, R.rigNote,
      h('article', { class: 'card' }, h('header', {}, h('h3', {}, 'Data source'), h('div', { class: 'tools' }, h('button', { class: 'btn small', onclick: refreshSource }, '↻ refresh'))), h('div', { class: 'body' }, R.source)),
      R.profiles,
      h('article', { class: 'card' }, h('header', {}, h('h3', {}, 'Serial hardware'), h('div', { class: 'tools' }, h('span', { class: 'chip warn' }, 'not run on a board'))), h('div', { class: 'body' }, R.hw))));
  },
  update(S) {
    const s = S.snap; if (!s) return;
    if (!R.src || performance.now() - R.srcAt > 8000) refreshSource();
    // ---- the source
    const sk = JSON.stringify([R.src && R.src.interfaces, R.src && R.src.recordings.length, s.source && s.source.kind, s.source && s.source.iface, s.source && s.source.file, s.recording && true]);
    if (R.srcKey !== sk && R.src) {
      R.srcKey = sk; clear(R.source);
      const so = s.source || {};
      const ifs = R.src.interfaces;
      const sel = h('select', { id: 'rig-iface' }, [...new Set([...(ifs.length ? ifs : ['vcan0']), 'vcan0'])].map((i) => h('option', { value: i }, i)));
      if (so.iface) sel.value = so.iface;
      R.source.append(
        h('p', { style: { marginTop: 0 } }, 'Now: ', so.kind === 'live' ? h('b', {}, `live on ${so.iface}`) : so.kind === 'replay' ? h('b', {}, `replaying ${so.file}`) : h('b', {}, 'nothing'), so.message ? h('span', { class: 'note' }, ` (${so.message})`) : ''),
        h('div', { class: 'btns', style: { alignItems: 'center' } }, 'Attach to', sel, h('button', { class: 'btn primary', id: 'rig-connect', onclick: () => call(() => api('/api/source', { iface: sel.value }).then(refreshSource), `Attached to ${sel.value}`) }, 'Connect'), h('button', { class: 'btn', onclick: () => call(() => api('/api/source', { disconnect: true }).then(refreshSource)) }, 'Disconnect'),
          h('span', { class: 'note' }, ifs.length ? '' : 'No CAN interface found: run sim/scripts/setup_vcan.sh (needs sudo).')),
        h('div', { class: 'sep' }), h('div', { class: 'note', style: { marginBottom: '6px' } }, h('b', {}, 'Recordings'), ' (logs/ and console/demo/): the console’s own recordings carry the bus, the nodes’ consoles and the simulator’s telemetry; any candump -L log opens too.'),
        R.src.recordings.length ? h('table', {}, h('thead', {}, h('tr', {}, h('th', {}, 'file'), h('th', {}, 'where'), h('th', { class: 'num' }, 'size'), h('th', {}, 'with consoles'), h('th', {}, ''))), h('tbody', {}, R.src.recordings.map((r) => h('tr', {}, h('td', { class: 'mono', style: { fontSize: '12px' } }, r.name), h('td', {}, h('span', { class: 'chip muted' }, r.where)), h('td', { class: 'num' }, bytes(r.bytes)), h('td', {}, r.sidecar ? '✓' : '—'),
          h('td', {}, h('button', { class: 'btn small', onclick: () => call(() => api('/api/source', { replay: r.name }).then(refreshSource), `Replaying ${r.name}`) }, '▶ Replay')))))) : h('div', { class: 'empty' }, 'No recordings yet: press REC in the header while a live source is attached.'));
    }
    // ---- the rig's profiles
    const rg = s.rig || {};
    const pk = JSON.stringify([rg.profile, (rg.procs || []).map((p) => [p.name, p.state, Math.floor((p.uptime || 0) / 2), p.lines]), Object.keys(rg.profiles || {})]);
    if (R.pk !== pk) {
      R.pk = pk; clear(R.profiles);
      if (rg.disabled) R.profiles.append(h('div', { class: 'banner warn' }, 'The console was started with --no-rig: it can watch and command, not start processes.'));
      else for (const [k, p] of Object.entries(rg.profiles || {})) R.profiles.append(profileCard(k, p, rg));
    }
    // ---- the hardware
    const hw = s.hardware || {};
    const hk = JSON.stringify([hw.ports, hw.consoles, hw.supervisor && hw.supervisor.port, hw.pico && hw.pico.port, hw.pico && hw.pico.status && hw.pico.status.relays]);
    if (R.hk !== hk) {
      R.hk = hk; clear(R.hw);
      const ports = hw.ports || [];
      const portSel = () => h('select', {}, ports.length ? ports.map((p) => h('option', { value: p }, p)) : [h('option', { value: '' }, 'no serial port found')]);
      const row = (label, kind, role, state) => { const sel = portSel(); return h('tr', {}, h('td', {}, label), h('td', {}, state ? h('span', { class: `chip ${state.connected ? 'ok' : 'crit'}` }, `${state.connected ? 'connected' : 'lost'} · ${state.port}`) : h('span', { class: 'chip muted' }, 'not connected')), h('td', {}, sel),
        h('td', {}, h('div', { class: 'btns' }, h('button', { class: 'btn small', disabled: !ports.length, onclick: () => call(() => api('/api/hardware/connect', { kind, port: sel.value, role }), `${label} connected`) }, 'Connect'), state ? h('button', { class: 'btn small', onclick: () => call(() => api('/api/hardware/disconnect', { kind, role })) }, 'Disconnect') : null))); };
      R.hw.append(h('p', { class: 'note', style: { marginTop: 0 } }, 'The Nucleos’ ST-LINK virtual COM ports, the supervisor and the Pico appear as /dev/ttyACM*. A node’s console is read like a rig process’s: its lines feed the Events tab and the Voting tab’s counters. ', hw.note || ''),
        h('table', {}, h('thead', {}, h('tr', {}, h('th', {}, 'device'), h('th', {}, 'state'), h('th', {}, 'port'), h('th', {}, ''))), h('tbody', {},
          ['A', 'B', 'C', 'ACT'].map((n) => row(`Node ${n} console`, 'console', n, (hw.consoles || {})[n])), row('Supervisor (text)', 'supervisor', null, hw.supervisor), row('Pico (platform driver, injector)', 'pico', null, hw.pico))));
      if (hw.supervisor) {
        const verb = h('select', { id: 'sup-verb' }, (hw.verbs || []).map((v) => h('option', { value: v.verb }, v.verb)));
        const unit = h('select', { id: 'sup-unit' }, [h('option', { value: '' }, '—'), ...(hw.units || []).map((u) => h('option', { value: u }, u))]);
        R.hw.append(h('div', { class: 'sep' }), h('div', { class: 'note', style: { marginBottom: '6px' } }, h('b', {}, 'Supervisor commands'), ' (SUPERVISOR.md section 6; no authentication: its USB port is a physical port on the bench)'),
          h('div', { class: 'btns', style: { alignItems: 'center' } }, verb, unit, h('button', { class: 'btn warn', onclick: () => call(() => api('/api/supervisor', { line: `${verb.value} ${unit.value}`.trim() }), 'Sent') }, 'Send')));
      }
    }
  },
};
