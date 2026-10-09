// Events: everything that has happened, in order, and the raw console of every node. The bus tells the outcome (a heartbeat changed); only the console tells the reason.
import { h, setText, store, download, NODES } from '../util.js';

let R = { open: new Set(), srcOn: store.get('events.src', null), minLevel: store.get('events.level', 'info'), tab: store.get('events.tab', 'A') };
const LV = { info: 0, ok: 1, warn: 2, crit: 3 };
const SRC = ['bus', 'A', 'B', 'C', 'ACT', 'SIM', 'SUP', 'PEERS', 'OP', 'CON'];
const NOISE = new Set(['boot', 'text', 'status', 'act-status', 'countdown', 'resync']);

function when(S, t) {
  const t0 = (S.snap.milestones || {}).t0;
  if (t0 != null) { const d = t - t0; return `T${d >= 0 ? '+' : '-'}${Math.abs(d).toFixed(1)}`; }
  const e0 = S.events.length ? S.events[0].t : t; const d = t - e0;
  return `+${Math.floor(d / 60)}:${(d % 60).toFixed(1).padStart(4, '0')}`;
}

function drawEvents(S) {
  const on = R.srcOn || SRC;
  const q = R.search.value.trim().toLowerCase();
  const rows = S.events.filter((e) => LV[e.level] >= LV[R.minLevel] && on.includes(e.src) && (R.noise.checked || !NOISE.has(e.kind)) && (!q || e.text.toLowerCase().includes(q) || e.kind.includes(q))).slice(-500);
  const key = rows.length + ':' + (rows.length ? rows[rows.length - 1].seq : 0) + R.minLevel + on.join() + q + R.noise.checked + [...R.open].join();
  if (R.ekey === key) return; R.ekey = key;
  const box = R.list, stick = box.scrollTop + box.clientHeight >= box.scrollHeight - 40;
  box.innerHTML = '';
  if (!rows.length) box.append(h('div', { class: 'empty' }, 'No event matches.'));
  for (const e of rows) {
    const open = R.open.has(e.seq);
    box.append(h('div', { class: `log-line ${e.level}`, style: { cursor: 'pointer' }, onclick: () => { open ? R.open.delete(e.seq) : R.open.add(e.seq); R.ekey = null; drawEvents(S); } },
      h('span', { class: 's-muted' }, when(S, e.t).padEnd(8) + ' '), h('span', { class: 's-muted' }, (e.frame != null ? 'f' + e.frame : '').padEnd(8) + ' '), h('span', { class: `tag n${e.src}`, style: { marginRight: '8px' } }, e.src), e.text.replace(/^\[\w+\] /, ''),
      open ? h('div', { class: 'mono-box', style: { margin: '4px 0 4px 0' } }, JSON.stringify({ kind: e.kind, level: e.level, node: e.node, t: e.t, frame: e.frame, fields: e.fields }, null, 1)) : null));
  }
  if (stick) box.scrollTop = box.scrollHeight;
  setText(R.count, `${rows.length} shown of ${S.events.length}`);
}

function drawConsole(S) {
  const srcs = Object.keys(S.lines);
  const key = srcs.join() + ':' + R.tab + ':' + srcs.map((k) => (S.lines[k].length ? S.lines[k][S.lines[k].length - 1].t : 0)).join() + R.hideStatus.checked;
  if (R.ckey === key) return; R.ckey = key;
  R.tabs.innerHTML = '';
  const all = [...new Set([...SRC.filter((x) => ['A', 'B', 'C', 'ACT', 'SIM', 'PEERS', 'SUP'].includes(x)), ...srcs])];
  if (!all.includes(R.tab)) R.tab = srcs[0] || 'A';
  for (const x of all) R.tabs.append(h('button', { type: 'button', 'aria-pressed': x === R.tab ? 'true' : 'false', onclick: () => { R.tab = x; store.set('events.tab', x); R.ckey = null; drawConsole(S); } }, x, S.lines[x] ? '' : ' ·'));
  const lines = (S.lines[R.tab] || []).filter((l) => !(R.hideStatus.checked && (l.kind === 'status' || l.kind === 'act-status'))).slice(-300);
  const box = R.con, stick = box.scrollTop + box.clientHeight >= box.scrollHeight - 40;
  box.innerHTML = '';
  if (!lines.length) box.append(h('div', { class: 'empty' }, S.lines[R.tab] ? 'Only status lines so far.' : `No console for ${R.tab}. A rig process, or a serial port attached on the Rig tab, provides one.`));
  for (const l of lines) box.append(h('div', { class: `log-line ${l.level}` }, l.text));
  if (stick) box.scrollTop = box.scrollHeight;
}

export default {
  id: 'events', label: 'Events', icon: 'events',
  mount(root) {
    R.list = h('div', { class: 'mono-box', style: { height: '62vh', padding: 0, whiteSpace: 'normal' } });
    R.search = h('input', { type: 'text', placeholder: 'search', id: 'ev-search', style: { width: '150px' } }); R.search.addEventListener('input', () => { R.ekey = null; });
    R.noise = h('input', { type: 'checkbox', id: 'ev-noise' }); R.noise.addEventListener('change', () => { R.ekey = null; });
    R.count = h('span', { class: 'note' });
    const lvl = h('div', { class: 'seg', role: 'group', 'aria-label': 'Lowest level shown' }, Object.keys(LV).map((k) => h('button', { type: 'button', 'aria-pressed': k === R.minLevel ? 'true' : 'false', onclick: (e) => { R.minLevel = k; store.set('events.level', k); R.ekey = null; for (const b of lvl.children) b.setAttribute('aria-pressed', b === e.currentTarget ? 'true' : 'false'); } }, k)));
    R.srcBox = h('div', { class: 'btns' }, SRC.map((x) => h('label', { class: 'chip', style: { cursor: 'pointer' } }, h('input', { type: 'checkbox', checked: !R.srcOn || R.srcOn.includes(x), style: { accentColor: 'var(--accent)' }, onchange: () => { R.srcOn = SRC.filter((s, i) => R.srcBox.children[i].querySelector('input').checked); store.set('events.src', R.srcOn); R.ekey = null; } }), h('span', { class: `tag n${x}` }, x))));
    R.tabs = h('div', { class: 'seg', role: 'group', 'aria-label': 'Console of' });
    R.con = h('div', { class: 'mono-box', style: { height: '62vh', padding: 0 } });
    R.hideStatus = h('input', { type: 'checkbox', checked: true, id: 'con-hide' }); R.hideStatus.addEventListener('change', () => { R.ckey = null; });
    root.append(h('div', { class: 'grid c-3-2' },
      h('article', { class: 'card' }, h('header', {}, h('h3', {}, 'Events'), h('div', { class: 'tools' }, R.count, R.search, h('button', { class: 'btn small', onclick: () => download('events.json', JSON.stringify(R.S.events, null, 1), 'application/json') }, 'Save'))),
        h('div', { class: 'body' }, h('div', { style: { display: 'flex', gap: '12px', flexWrap: 'wrap', alignItems: 'center', marginBottom: '8px' } }, lvl, R.srcBox, h('label', { class: 'check', style: { padding: 0 } }, R.noise, h('span', { class: 'note' }, 'boot, countdown and resync chatter'))), R.list)),
      h('article', { class: 'card' }, h('header', {}, h('h3', {}, 'Node consoles'), h('div', { class: 'tools' }, h('label', { class: 'check', style: { padding: 0 } }, R.hideStatus, h('span', { class: 'note' }, 'hide status lines')))),
        h('div', { class: 'body' }, h('div', { style: { marginBottom: '8px', overflowX: 'auto' } }, R.tabs), R.con))));
  },
  update(S) { R.S = S; if (!S.snap) return; drawEvents(S); drawConsole(S); },
};
