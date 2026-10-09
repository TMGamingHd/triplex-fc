// Voting: how the three computers' agreement is working. What the computers say (heartbeat views, ACT's vote status, their consoles) is shown as theirs; the deviation bars and the charts are the console's
// own reconstruction from the frames on the bus, compared frame by frame (same frame number), in units of the voter's tolerance.
import { h, svg, setText, num, NODES, store, levelOf } from '../util.js';
import { Chart, legend } from '../charts.js';
import { healthLevel } from '../derive.js';

let R = {};
const COL = { A: 'var(--nA)', B: 'var(--nB)', C: 'var(--nC)' };
const COUNTERS = [['crc', 'CRC'], ['seq', 'sequence'], ['missing', 'missing'], ['vote', 'vote'], ['digest', 'digest'], ['stuck', 'stuck'], ['oos', 'out-of-sched.'], ['tx_err', 'tx errors'], ['imu_err', 'IMU errors'], ['imu_stale', 'IMU stale'],
  ['bus_off', 'bus-off'], ['err_passive', 'err-passive'], ['sync_missed', 'SYNC missed'], ['resync_adopted', 'resync adopted'], ['resync_skipped', 'resync skipped'], ['far', 'resync far'], ['wcet_step', 'WCET step µs'], ['wcet_vote', 'WCET vote µs'], ['wcet_frame', 'WCET frame µs']];

function diagram() {
  const W = 600, Hh = 230;
  const root = svg('svg', { viewBox: `0 0 ${W} ${Hh}`, class: 'diagram', role: 'img', 'aria-label': 'The three computers, ACT\'s vote and its output' });
  R.box = {}; R.line = {};
  NODES.forEach((n, i) => {
    const y = 14 + i * 72;
    const g = svg('g', {});
    const rect = svg('rect', { x: 8, y, width: 150, height: 56, rx: 8, fill: 'var(--panel2)', stroke: COL[n], 'stroke-width': 2 });
    const t1 = svg('text', { x: 20, y: y + 22, fill: COL[n], 'font-weight': 700, 'font-size': 15 }, `FC-${n}`);
    const t2 = svg('text', { x: 20, y: y + 42, fill: 'var(--muted)', 'font-size': 12 }, '—');
    const dot = svg('circle', { cx: 142, cy: y + 16, r: 6, fill: 'var(--faint)' });
    g.append(rect, t1, t2, dot);
    root.append(g);
    R.box[n] = { rect, t2, dot, y };
    const line = svg('path', { d: `M158 ${y + 28} C 205 ${y + 28}, 205 115, 250 115`, fill: 'none', stroke: 'var(--faint)', 'stroke-width': 2.5 });
    root.append(line); R.line[n] = line;
  });
  R.vote = svg('rect', { x: 250, y: 66, width: 130, height: 100, rx: 10, fill: 'var(--panel3)', stroke: 'var(--line2)', 'stroke-width': 2 });
  R.vt1 = svg('text', { x: 315, y: 96, 'text-anchor': 'middle', fill: 'var(--text)', 'font-weight': 700, 'font-size': 15 }, 'ACT vote');
  R.vt2 = svg('text', { x: 315, y: 120, 'text-anchor': 'middle', fill: 'var(--muted)', 'font-size': 13 }, '—');
  R.vt3 = svg('text', { x: 315, y: 144, 'text-anchor': 'middle', fill: 'var(--muted)', 'font-size': 12 }, '');
  const out = svg('path', { d: 'M380 116 L450 116', stroke: 'var(--accent)', 'stroke-width': 3, fill: 'none', 'marker-end': 'url(#arr)' });
  const defs = svg('defs', {}, svg('marker', { id: 'arr', viewBox: '0 0 10 10', refX: 8, refY: 5, markerWidth: 7, markerHeight: 7, orient: 'auto-start-reverse' }, svg('path', { d: 'M0 0L10 5L0 10z', fill: 'var(--accent)' })));
  R.o1 = svg('text', { x: 458, y: 100, fill: 'var(--text)', 'font-size': 12, 'font-weight': 600 }, 'gimbal');
  R.o2 = svg('text', { x: 458, y: 118, fill: 'var(--muted)', 'font-size': 11, class: 'mono' }, '');
  R.o3 = svg('text', { x: 458, y: 134, fill: 'var(--muted)', 'font-size': 11, class: 'mono' }, '');
  root.append(defs, R.vote, R.vt1, R.vt2, R.vt3, out, R.o1, R.o2, R.o3);
  return root;
}

function lifecycle() {
  const W = 600, Hh = 190, root = svg('svg', { viewBox: `0 0 ${W} ${Hh}`, class: 'diagram', role: 'img', 'aria-label': 'The life cycle of a node: healthy, latched, probation, disabled' });
  root.append(svg('defs', {}, svg('marker', { id: 'ar2', viewBox: '0 0 10 10', refX: 9, refY: 5, markerWidth: 6, markerHeight: 6, orient: 'auto' }, svg('path', { d: 'M0 0L10 5L0 10z', fill: 'var(--muted)' }))));
  const box = { healthy: [20, 24, 'var(--ok)', 'Healthy'], latched: [440, 24, 'var(--warn)', 'Latched out'], probation: [230, 120, 'var(--info)', 'Probation'], disabled: [440, 120, 'var(--crit)', 'Disabled'] };
  for (const [, [x, y, c, t]] of Object.entries(box)) root.append(svg('rect', { x, y, width: 140, height: 50, rx: 10, fill: 'var(--panel2)', stroke: c, 'stroke-width': 2 }), svg('text', { x: x + 70, y: y + 31, 'text-anchor': 'middle', fill: c, 'font-weight': 700, 'font-size': 15 }, t));
  const arrow = (d, n, bx, by) => root.append(svg('path', { d, fill: 'none', stroke: 'var(--muted)', 'stroke-width': 1.8, 'marker-end': 'url(#ar2)' }), svg('circle', { cx: bx, cy: by, r: 9, fill: 'var(--panel3)', stroke: 'var(--muted)' }), svg('text', { x: bx, y: by + 4, 'text-anchor': 'middle', fill: 'var(--text)', 'font-size': 11, 'font-weight': 700 }, n));
  arrow('M160 49 L440 49', '1', 300, 49);
  arrow('M460 74 Q 430 105 345 128', '2', 428, 100);
  arrow('M230 150 Q 120 140 96 76', '3', 150, 130);
  arrow('M525 74 L525 120', '4', 525, 97);
  return root;
}

const LIFE = [
  ['1', 'Healthy to latched out', 'bad in 3 of the last 5 frames, or the leaky count reaches 3 (ADR-010, ADR-013). A latched node is out of the vote and stays out until the operator readmits it.'],
  ['2', 'Latched out to probation', 'after the dwell (0.5 s after a first transient-looking latch, else 2 s) and the operator’s reintegrate: a shadow vote against the healthy nodes, still not voting.'],
  ['3', 'Probation to healthy', '100 agreeing frames (300 after a repeat). A node that disagrees on probation goes back to latched out.'],
  ['4', 'Latched out to disabled', 'strikes reach 3 (2 when the cause is physical: stuck or intermittent). It needs “clear disabled” with an ARM, then a reintegrate.'],
];

// What ACT's status means next to the flight computers' mode, in words: the two count different things (ACT votes on the computers' commands, the computers vote on sensors), so they can honestly differ.
function actNote(s) {
  const a = s.act, r = s.redundancy;
  if (!a || !a.alive) return 'ACT is not sending its output.';
  const fcMode = s.nodes.filter((n) => n.heartbeat).map((n) => n.mode);
  const sensors = r.sensors, comps = r.computers;
  const parts = [];
  if (comps === 3) parts.push('ACT votes on all three computers’ commands (a median of three: one wrong command is out-voted).');
  else if (comps === 2) parts.push('ACT has two commands, so it can only compare them: it averages them when they agree and, if they differ, holds its last output and blames nobody. It is not a vote.');
  else if (comps === 1) parts.push('ACT has one command and nothing to check it against. It follows it, with no cross-check.');
  else parts.push('ACT has no command to follow.');
  if (fcMode.length && comps != null && sensors !== comps) parts.push(`The flight computers vote on sensors, and only ${sensors} of 3 are voting there (the others are latched out); ACT votes on commands, and ${comps} of 3 computers are still in that vote. A computer whose IMU was latched out still computes a good command, so the two counts differ. This is by design (docs/design/ACT_LOGIC.md).`);
  return parts.join(' ');
}

export default {
  id: 'voting', label: 'Voting', icon: 'voting',
  mount(root) {
    R.diagram = diagram(); R.life = lifecycle();
    R.actNote = h('p', { class: 'note', style: { margin: '8px 0 0' } });
    R.win = store.get('vote.window', 60);
    R.rows = h('div'); R.rowsHead = h('div', { class: 'devhead' }, h('span', {}, 'channel'), NODES.map((n) => h('span', { style: { color: COL[n] } }, `FC-${n}`)), h('span', {}, 'deviation from the median (green = within tolerance)'));
    const c1 = h('canvas'), c2 = h('canvas');
    R.chDev = new Chart(c1, { height: 170, window: R.win, yMin: 0, hlines: [{ y: 1, color: 'var(--crit)', label: 'tolerance', include: true }], series: NODES.map((n) => ({ key: `dev_${n.toLowerCase()}`, label: `FC-${n}`, color: COL[n] })) });
    R.chCmd = new Chart(c2, { height: 170, window: R.win, series: [...NODES.map((n) => ({ key: `cmd_p_${n.toLowerCase()}`, label: `FC-${n}`, color: COL[n], width: 1.4 })), { key: 'act_p', label: 'ACT out', color: 'var(--nACT)', width: 2.2, dash: [4, 3] }] });
    const winSeg = h('div', { class: 'seg', role: 'group', 'aria-label': 'Chart window' }, [30, 120, 300].map((w) => h('button', { type: 'button', 'aria-pressed': w === R.win ? 'true' : 'false', onclick: (e) => { R.win = w; store.set('vote.window', w); R.chDev.setWindow(w); R.chCmd.setWindow(w); for (const b of winSeg.children) b.setAttribute('aria-pressed', b === e.currentTarget ? 'true' : 'false'); } }, w < 60 ? `${w} s` : `${w / 60} min`)));
    R.views = h('table', { class: 'tight' }, h('thead', {}, h('tr', {}, h('th', { title: 'each row is one computer’s view of the three; the outlined chip is its view of itself' }, 'viewer ↓'), NODES.map((n) => h('th', { style: { color: COL[n] } }, `FC-${n}`)), h('th', {}, 'digest'), h('th', {}, 'strikes'), h('th', { title: 'the last ground-command counter this computer accepted' }, 'counter'))), h('tbody'));
    R.consensus = h('div', { class: 'stats', style: { marginBottom: '10px' } });
    R.latch = h('div', { class: 'note' });
    R.counters = h('table', {}, h('thead', {}, h('tr', {}, h('th', {}, 'counter'), NODES.map((n) => h('th', { class: 'num', style: { color: COL[n] } }, n)))), h('tbody'));
    R.counterNote = h('div', { class: 'note', style: { padding: '8px 12px' } });
    R.params = h('div', { class: 'stats params' });
    root.append(h('div', { class: 'stack' },
      h('div', { class: 'grid', style: { gridTemplateColumns: 'repeat(auto-fit, minmax(420px, 1fr))' } },
        h('article', { class: 'card' }, h('header', {}, h('h3', {}, 'ACT: the vote on the commands'), h('div', { class: 'tools' }, h('span', { class: 'note' }, 'what ACT says it is doing'))), h('div', { class: 'body' }, R.diagram, R.actNote)),
        h('article', { class: 'card' }, h('header', {}, h('h3', {}, 'Node life cycle'), h('div', { class: 'tools' }, h('span', { class: 'note' }, 'ADR-010, ADR-013'))), h('div', { class: 'body' }, R.life, h('div', { id: 'lc-where', class: 'legend', style: { margin: '6px 0' } }), h('ol', { class: 'note', style: { margin: 0, paddingLeft: '20px' } }, LIFE.map(([n, t, d]) => h('li', { value: n }, h('b', {}, t + ': '), d)))))),
      h('div', { class: 'grid', style: { gridTemplateColumns: 'repeat(auto-fit, minmax(500px, 1fr))' } },
        h('article', { class: 'card' }, h('header', {}, h('h3', {}, 'Agreement, channel by channel'), h('div', { class: 'tools' }, h('span', { class: 'note' }, 'the console’s reconstruction from the bus'))), h('div', { class: 'body' }, R.rowsHead, R.rows,
          h('p', { class: 'note', style: { marginBottom: 0 } }, 'Each frame’s samples are compared only with the other nodes’ samples of the same frame. A point outside the green band is a sample the voter would call out of tolerance; a node is latched only if that persists (3 of the last 5 frames, or the leaky count).'))),
        h('article', { class: 'card' }, h('header', {}, h('h3', {}, 'Over time'), h('div', { class: 'tools' }, winSeg)),
          h('div', { class: 'body' }, h('div', { class: 'note' }, 'Largest deviation of each node on any channel, since the last sample (1.0 = the tolerance)'), c1, legend(R.chDev.o.series), h('div', { class: 'sep' }), h('div', { class: 'note' }, 'Pitch command of each node, and ACT’s output'), c2, legend(R.chCmd.o.series)))),
      h('div', { class: 'grid', style: { gridTemplateColumns: 'repeat(auto-fit, minmax(min(100%, 560px), 1fr))' } },
        h('article', { class: 'card' }, h('header', {}, h('h3', {}, 'What the computers say about each other'), h('div', { class: 'tools' }, h('span', { class: 'note' }, 'heartbeats and state shares'))), h('div', { class: 'body' }, R.consensus, h('div', { class: 'tablewrap' }, R.views), R.latch)),
        h('article', { class: 'card' }, h('header', {}, h('h3', {}, 'Counters from the nodes’ consoles')), h('div', { class: 'body flush' }, h('div', { class: 'tablewrap' }, R.counters), R.counterNote))),
      h('article', { class: 'card' }, h('header', {}, h('h3', {}, 'The voter’s parameters'), h('div', { class: 'tools' }, h('span', { class: 'note' }, 'copies of core/ defaults, checked by a test'))), h('div', { class: 'body' }, R.params))));
  },
  update(S) {
    const s = S.snap; if (!s) return;
    // --- ACT diagram
    const a = s.act;
    for (const n of s.nodes) {
      const b = R.box[n.name], i = NODES.indexOf(n.name);
      const lvl = !n.seen ? 'muted' : !n.alive ? 'crit' : healthLevel(n.health);
      b.rect.setAttribute('stroke', COL[n.name]); b.dot.setAttribute('fill', `var(--${lvl === 'muted' ? 'faint' : lvl})`);
      setText(b.t2, !n.seen ? 'not on the bus' : !n.alive ? 'SILENT' : `${n.health} · ${n.master ? 'MAIN' : n.role || 'virtual peer'}`);
      const voted = a && a.voted[i], excl = a && a.excluded[i];
      const ln = R.line[n.name];
      ln.setAttribute('stroke', !n.alive ? 'var(--faint)' : excl ? 'var(--crit)' : voted ? 'var(--ok)' : 'var(--warn)');
      ln.setAttribute('stroke-dasharray', !n.alive || excl ? '5 4' : '');
    }
    if (a) {
      R.vote.setAttribute('stroke', a.alive ? (a.state === 1 ? 'var(--ok)' : a.state === 0 ? 'var(--info)' : 'var(--crit)') : 'var(--crit)');
      setText(R.vt2, a.vote_status); setText(R.vt3, `${a.alive ? a.state_name : 'SILENT'}${a.held ? ' · holding' : ''}`);
      setText(R.o2, `pitch ${a.pitch.toFixed(3)}°`); setText(R.o3, `yaw  ${a.yaw.toFixed(3)}°`);
    } else { setText(R.vt2, 'no ACT output'); setText(R.vt3, ''); setText(R.o2, ''); setText(R.o3, ''); }
    setText(R.actNote, actNote(s));
    // life cycle: where each node is
    const where = document.getElementById('lc-where');
    const wk = JSON.stringify(s.nodes.map((n) => n.health));
    if (R.wk !== wk) { R.wk = wk; where.innerHTML = ''; for (const n of s.nodes) where.append(h('span', { class: `chip ${n.alive ? healthLevel(n.health) : 'crit'}` }, h('b', { style: { color: COL[n.name] } }, `FC-${n.name}`), n.alive ? n.health : 'silent')); }
    // --- agreement rows
    const ch = s.vote.channels, cfg = S.cfg ? S.cfg.channels : [];
    if (!R.rowEls && cfg.length) {
      R.rowEls = cfg.map((c, k) => {
        const vals = NODES.map((n) => h('span', { class: 'mono', style: { fontSize: '12px' } }, '—'));
        const pts = NODES.map((n, i) => h('i', { class: `pt n${n}`, title: `FC-${n}`, style: { top: `${3 + i * 8}px` } }));
        const bar = h('div', { class: 'devbar' }, h('i', { class: 'band' }), h('i', { class: 'mid' }), pts);
        R.rows.append(h('div', { class: 'devrow' }, h('span', { style: { fontWeight: 600, fontSize: '13px' } }, c.name, h('span', { class: 'note' }, ` ${c.unit}`)), vals, bar));
        return { vals, pts };
      });
    }
    if (R.rowEls) ch.forEach((c, k) => {
      const e = R.rowEls[k];
      NODES.forEach((n, i) => {
        const v = c && c.vals[i];
        setText(e.vals[i], v == null ? '—' : v.toFixed(cfg[k].tol < 0.1 ? 4 : 2));
        const pt = e.pts[i];
        if (!c || v == null) { pt.style.display = 'none'; return; }
        pt.style.display = '';
        const z = c.dev[i] / cfg[k].tol;
        pt.style.left = `${50 + Math.max(-0.5, Math.min(0.5, z / 6)) * 100}%`;
        pt.classList.toggle('out', Math.abs(z) > 1);
      });
    });
    // --- charts
    R.chDev.draw(S.hist); R.chCmd.draw(S.hist);
    // --- views
    const tb = R.views.querySelector('tbody'); tb.innerHTML = '';
    s.views.forEach((row, o) => {
      const n = s.nodes[o];
      tb.append(h('tr', {}, h('td', { style: { color: COL[NODES[o]], fontWeight: 700 } }, `FC-${NODES[o]}`, n.alive ? '' : h('span', { class: 'note' }, ' (silent)')),
        row.map((v, sub) => h('td', {}, v == null ? '—' : h('span', { class: `chip ${levelOf(v)}${o === sub ? ' self' : ''}`, title: o === sub ? 'its own view of itself' : '' }, v))),
        h('td', { class: `mono${n.alive ? '' : ' stale'}` }, n.digest != null ? '0x' + n.digest.toString(16).padStart(4, '0') : '—'), h('td', { class: `mono${n.alive ? '' : ' stale'}`, title: 'latches on record against A · B · C' }, n.strikes ? n.strikes.join('·') : '—'), h('td', { class: `mono${n.alive ? '' : ' stale'}` }, n.cmd_counter ?? '—')));
    });
    const dig = s.nodes.filter((n) => n.alive && n.digest != null).map((n) => n.digest);
    const agree = dig.length > 1 && dig.every((d) => d === dig[0]);
    R.consensus.innerHTML = '';
    for (const n of s.nodes) R.consensus.append(h('div', { class: `stat ${n.alive ? healthLevel(n.health) : 'crit'}` }, h('div', { class: 'k' }, `FC-${n.name} per the others`), h('div', { class: 'v', style: { fontSize: '15px' } }, n.alive ? n.health : 'silent')));
    R.consensus.append(h('div', { class: `stat ${dig.length < 2 ? 'dim' : agree ? 'ok' : 'warn'}` }, h('div', { class: 'k' }, 'state digests'), h('div', { class: 'v', style: { fontSize: '15px' } }, dig.length < 2 ? '—' : agree ? 'agree' : 'DIFFER')));
    const lt = S.events.filter((e) => e.kind === 'latched-out' || e.kind === 'probation-failed' || e.kind === 'disabled' || e.kind === 'reintegrated').slice(-4).reverse();
    R.latch.innerHTML = '';
    R.latch.append(h('b', {}, 'Last judgements (from the nodes’ consoles): '), ...(lt.length ? lt.map((e) => h('div', { class: 'mono', style: { fontSize: '12px' } }, `f${e.frame ?? '?'}  ${e.text.replace(/^\[\w+\] /, '')}`)) : ['none yet, or no console attached.']));
    // --- counters
    const tbc = R.counters.querySelector('tbody'); tbc.innerHTML = '';
    const any = s.nodes.some((n) => n.console && n.console.mode);
    for (const [k, lbl] of COUNTERS) {
      const vals = s.nodes.map((n) => (n.console ? n.console[k] : undefined));
      const cls = (v, i) => (v ? (['crc', 'seq', 'missing', 'vote', 'digest', 'stuck', 'oos', 'bus_off'].includes(k) ? 's-crit' : k === 'far' || k === 'resync_skipped' ? 's-warn' : '') : '');
      tbc.append(h('tr', {}, h('td', {}, lbl), vals.map((v, i) => h('td', { class: `num ${cls(v, i)}${s.nodes[i].alive ? '' : ' stale'}`, title: s.nodes[i].alive ? '' : 'the last value before the node fell silent' }, v == null ? '—' : String(v)))));
    }
    setText(R.counterNote, any ? 'Each node’s own counters, from its last status line (every 100 frames).' : 'No node console is attached, so these are blank. Start the rig from the Rig tab, or attach the Nucleos’ serial consoles there.');
    // --- parameters (once)
    if (!R.pdone && S.cfg) {
      R.pdone = true;
      const st = (k, v) => h('div', { class: 'stat' }, h('div', { class: 'k' }, k), h('div', { class: 'v' }, v));
      for (const c of S.cfg.channels) R.params.append(st(`${c.name} tolerance`, `${c.tol} ${c.unit}`));
      R.params.append(st('ACT tolerance', `${S.cfg.act_tol}°`), st('latch when', `${S.cfg.persist[0]} of last ${S.cfg.persist[1]} frames`), st('strikes to disable', `${S.cfg.strikes_max[0]} (${S.cfg.strikes_max[1]} physical)`), st('ARM stays valid', `${S.cfg.arm_window_frames / 100} s`), st('counter window', `1 to ${S.cfg.command_window} ahead`));
    }
  },
};
