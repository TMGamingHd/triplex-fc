// Bus: every id on the flight bus with its rate, its failures and its last frame, and a live monitor of decoded frames. The monitor is an observer: it prints what was on the wire and judges nothing.
import { h, setText, eng, store, download } from '../util.js';
import { api } from '../net.js';

let R = { after: 0, paused: false, rows: [] };
const BITS = 111;      // a classic CAN data frame with 8 data bytes, in bits, without stuffing (SOF, 11-bit id, control, 64 data, CRC, ACK, EOF, interframe)

async function poll() {
  if (performance.now() - (R.seen || 0) > 1500) return;      // the tab is not showing: update() is not being called
  try {
    const t = await api('/api/bus'); R.table = t;
    if (!R.paused) {
      const ids = R.filter.value.trim().replace(/\s+/g, '');
      const f = await api(`/api/frames?after=${R.after}&limit=250${ids ? '&ids=' + encodeURIComponent(ids) : ''}`);
      if (f.last < R.after) R.rows = [];            // the feed started over (a replay seeked)
      R.after = f.last;
      for (const fr of f.frames) R.rows.push(fr);
      if (R.rows.length > 600) R.rows.splice(0, R.rows.length - 600);
      render();
    }
    renderTable();
  } catch { /* the header says the server is gone */ }
}
function renderTable() {
  const t = R.table; if (!t) return;
  const body = R.tbody; body.innerHTML = '';
  const total = t.rows.reduce((a, r) => a + r.rate, 0);
  setText(R.load, `${total.toFixed(0)} frames/s, about ${((total * BITS) / 1e6 * 100).toFixed(0)} % of 1 Mbit/s (111 bits per frame, no stuffing: arithmetic, not a measurement)`);
  for (const r of t.rows) body.append(h('tr', { class: r.in_schedule ? '' : 's-warn', style: { cursor: 'pointer' }, title: 'Show only this id in the monitor (click again to clear)', onclick: () => { const hex = r.id.toString(16); R.filter.value = R.filter.value === hex ? '' : hex; R.rows = []; R.after = 0; } }, h('td', { class: 'mono' }, `0x${r.id.toString(16).toUpperCase().padStart(3, '0')}`), h('td', {}, r.name, r.in_schedule ? '' : h('span', { class: 'chip warn', style: { marginLeft: '6px' } }, 'out of schedule')), h('td', { class: `num ${r.expected && r.count > 0 && r.rate < 0.9 * r.expected ? 's-crit' : ''}`, title: r.expected ? `expected about ${r.expected} Hz` : 'no fixed rate' }, r.rate.toFixed(1)), h('td', { class: 'num s-muted' }, r.expected ? r.expected : '—'), h('td', { class: 'num' }, eng(r.count, 1)), h('td', { class: `num ${r.crc_bad ? 's-crit' : ''}` }, r.crc_bad),
    h('td', { class: 'num' }, r.age < 1 ? `${(r.age * 1000).toFixed(0)} ms` : `${r.age.toFixed(1)} s`), h('td', { class: 'mono', style: { fontSize: '11.5px' } }, r.hex)));
  R.nodes.innerHTML = '';
  for (const n of t.nodes) R.nodes.append(h('div', { class: `stat ${n.crc_bad || n.seq_gaps ? 'warn' : ''}` }, h('div', { class: 'k' }, `node ${n.name}`), h('div', { class: 'v', style: { fontSize: '14px' } }, `crc ${n.crc_bad} · seq ${n.seq_gaps}`)));
}
function render() {
  const mon = R.mon, stick = mon.scrollTop + mon.clientHeight >= mon.scrollHeight - 30;
  const q = R.search.value.trim().toLowerCase();
  mon.innerHTML = '';
  for (const r of R.rows) { if (q && !(r.text.toLowerCase().includes(q) || r.name.toLowerCase().includes(q))) continue; mon.append(h('div', { class: `log-line ${r.ok ? '' : 'crit'}` }, h('span', { class: 's-muted' }, r.t.toFixed(3).padStart(10) + '  '), h('span', { class: 's-muted' }, r.id.toString(16).toUpperCase().padStart(3, '0') + '  '), r.text)); }
  if (stick) mon.scrollTop = mon.scrollHeight;
}

export default {
  id: 'bus', label: 'Bus', icon: 'bus',
  mount(root) {
    R.tbody = h('tbody'); R.load = h('div', { class: 'note' }); R.nodes = h('div', { class: 'stats', style: { gridTemplateColumns: 'repeat(auto-fill, minmax(170px, 1fr))' } }); R.mon = h('div', { class: 'mono-box', style: { height: '360px', padding: 0 } });
    R.filter = h('input', { type: 'text', placeholder: 'ids to show, hex: 100,101,102', id: 'bus-ids', style: { width: '200px' } }); R.search = h('input', { type: 'text', placeholder: 'search the text', id: 'bus-search', style: { width: '160px' } });
    R.search.addEventListener('input', render); R.filter.addEventListener('change', () => { R.rows = []; R.after = 0; });
    R.pause = h('button', { class: 'btn small', onclick: () => { R.paused = !R.paused; setText(R.pause, R.paused ? '▶ Resume' : '❚❚ Pause'); } }, '❚❚ Pause');
    root.append(h('div', { class: 'stack' },
      h('article', { class: 'card' }, h('header', {}, h('h3', {}, 'Frames on the bus'), h('div', { class: 'tools' }, R.load)),
        h('div', { class: 'body' }, R.nodes), h('div', { class: 'body flush scroll', style: { maxHeight: '380px' } }, h('table', {}, h('thead', {}, h('tr', {}, h('th', {}, 'id'), h('th', {}, 'what'), h('th', { class: 'num' }, 'Hz'), h('th', { class: 'num' }, 'expected'), h('th', { class: 'num' }, 'frames'), h('th', { class: 'num' }, 'CRC bad'), h('th', { class: 'num' }, 'last'), h('th', {}, 'last data'))), R.tbody))),
      h('article', { class: 'card' }, h('header', {}, h('h3', {}, 'Monitor'), h('div', { class: 'tools' }, R.filter, R.search, R.pause, h('button', { class: 'btn small', onclick: () => download('bus-frames.txt', R.rows.map((r) => `${r.t.toFixed(4)} ${r.id.toString(16)} ${r.text}`).join('\n')) }, 'Save'))), h('div', { class: 'body' }, R.mon, h('p', { class: 'note', style: { marginBottom: 0 } }, 'Decoded with the same code the virtual peers use (tfc_peers/protocol.py). A frame that fails its CRC is shown as CRC-BAD with its raw bytes.')))));
    setInterval(poll, 500);
  },
  update() { R.seen = performance.now(); if (!R.started) { R.started = true; poll(); } },
};
