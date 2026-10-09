// Mission: the whole system at a glance: the three computers, ACT, the sync, what is wrong, how the flight is going, and what has happened.
import { h, setText, num, eng, ago, store, NODES } from '../util.js';
import { sequence, healthLevel } from '../derive.js';
import { Chart } from '../charts.js';

let R = {};
const kvRow = (k, id, tip) => [h('dt', { title: tip || k }, k), h('dd', { id })];
const $id = (x) => document.getElementById(x);

function nodeCard(n) {
  const id = (s) => `m-${n}-${s}`;
  return h('article', { class: `card nodecard n${n}` },
    h('header', {}, h('div', { class: 'title' }, h('b', {}, `FC-${n}`), h('span', { id: id('role'), class: 'chip muted' }, '—')), h('div', { class: 'tools' }, h('span', { id: id('health'), class: 'chip muted' }, 'not seen'))),
    h('div', { class: 'body' },
      h('dl', { class: 'kv' },
        kvRow('heartbeat', id('hb'), 'age of the last heartbeat'), kvRow('sees system as', id('mode'), 'the redundancy mode this computer reports'), kvRow('ready for launch', id('ready')), kvRow('release', id('rel'), 'first 16 bits of the source hash'),
        kvRow('resets', id('resets')), kvRow('strikes A·B·C', id('strikes'), 'latches on record against each node, as this computer holds them'),
        kvRow('frame WCET', id('wcet'), 'worst-case time of the frame, as the node reports it (wcet_frame). On native_sim it is not a measurement of a board'),
        kvRow('bad frames', id('bad'), 'CRC and sequence failures this computer has counted'), kvRow('frames missing', id('miss'), 'frames this computer expected and did not get')),
      h('div', { style: { marginTop: '8px' } }, h('button', { class: 'btn small ghost', onclick: () => { store.set('events.tab', n); R.ctx.go('events'); } }, 'its console →'))));
}

export default {
  id: 'mission', label: 'Mission', icon: 'mission',
  mount(root) {
    R.nodes = h('div', { class: 'nodegrid' },
      NODES.map(nodeCard),
      h('article', { class: 'card nodecard nACT' },
        h('header', {}, h('div', { class: 'title' }, h('b', {}, 'ACT')), h('div', { class: 'tools' }, h('span', { id: 'm-act-state', class: 'chip muted' }, '—'))),
        h('div', { class: 'body' }, h('dl', { class: 'kv' }, kvRow('vote status', 'm-act-vote'), kvRow('nodes in the vote', 'm-act-voted'), kvRow('excluded', 'm-act-excl'), kvRow('cause of Safe', 'm-act-cause'), kvRow('output pitch', 'm-act-p'), kvRow('output yaw', 'm-act-y'), kvRow('holding', 'm-act-held', 'ACT has no trustworthy vote this frame and repeats its last output'), kvRow('output frames', 'm-act-n'),
          kvRow('', 'm-act-pad')),
          h('div', { style: { marginTop: '8px' } }, h('button', { class: 'btn small ghost', onclick: () => { store.set('events.tab', 'ACT'); R.ctx.go('events'); } }, 'its console →')))));
    const bus = h('div', { class: 'strip', id: 'm-strip' },
      h('div', {}, h('span', {}, 'SYNC'), h('b', { id: 'm-sync-chip' }, '—')), h('div', {}, h('span', {}, 'frame'), h('b', { id: 'm-frame' }, '—')), h('div', {}, h('span', {}, 'rate'), h('b', { id: 'm-rate' }, '—')), h('div', {}, h('span', {}, 'mission frame'), h('b', { id: 'm-mf' }, '—')),
      h('div', {}, h('span', {}, 'sync restarts'), h('b', { id: 'm-restarts' }, '—')), h('div', {}, h('span', {}, 'frames received'), h('b', { id: 'm-rx' }, '—')), h('div', {}, h('span', {}, 'CRC failures'), h('b', { id: 'm-crc' }, '—')), h('div', {}, h('span', {}, 'out-of-schedule'), h('b', { id: 'm-oos' }, '—')));

    R.alerts = h('div', { class: 'alerts', 'aria-live': 'polite' });
    R.stats = h('div', { class: 'stats', style: { gridTemplateColumns: 'repeat(auto-fill, minmax(100px, 1fr))' } });
    R.seq = h('div', { style: { display: 'flex', gap: '6px', flexWrap: 'wrap' } });
    R.events = h('div', { class: 'scroll', style: { maxHeight: '300px' } });
    R.cAlt = new Chart(h('canvas'), { height: 96, window: 120, series: [{ key: 't_alt', label: 'altitude m', color: 'var(--accent)' }, { key: 'alt', label: 'bus', color: 'var(--faint)', dash: [3, 3] }] });
    R.cSpd = new Chart(h('canvas'), { height: 96, window: 120, series: [{ key: 't_speed', label: 'speed m/s', color: 'var(--ok)' }, { key: 'speed', label: 'bus', color: 'var(--faint)', dash: [3, 3] }] });
    root.append(h('div', { class: 'stack' },
      R.nodes, bus,
      h('div', { class: 'grid c-2-1' },
        h('div', { class: 'stack' },
          h('article', { class: 'card' }, h('header', {}, h('h3', {}, 'Launch sequence'), h('div', { class: 'tools' }, h('button', { class: 'btn small ghost', onclick: () => R.ctx.go('launch') }, 'Open Launch →'))), h('div', { class: 'body' }, R.seq)),
          h('article', { class: 'card' }, h('header', {}, h('h3', {}, 'The vehicle'), h('div', { class: 'tools' }, h('span', { id: 'm-fl-src', class: 'note' }), h('button', { class: 'btn small ghost', onclick: () => R.ctx.go('flight') }, 'Open Flight →'))),
            h('div', { class: 'body' }, R.stats, h('div', { class: 'grid cols2', style: { marginTop: '10px' } }, h('div', {}, h('div', { class: 'note' }, 'altitude, last 2 min'), R.cAlt.c), h('div', {}, h('div', { class: 'note' }, 'speed, last 2 min'), R.cSpd.c))))),
        h('div', { class: 'stack' },
          h('article', { class: 'card' }, h('header', {}, h('h3', {}, 'Alerts'), h('div', { class: 'tools' }, h('span', { id: 'm-alert-n', class: 'chip muted' }, '0'))), h('div', { class: 'body' }, R.alerts)),
          h('article', { class: 'card' }, h('header', {}, h('h3', {}, 'Latest events'), h('div', { class: 'tools' }, h('button', { class: 'btn small ghost', onclick: () => R.ctx.go('events') }, 'All events →'))), h('div', { class: 'body flush' }, R.events))))));
  },
  update(S, ctx) {
    R.ctx = ctx;
    const s = S.snap; if (!s) return;
    for (const n of s.nodes) {
      const id = (x) => $id(`m-${n.name}-${x}`);
      const hc = id('health'), role = id('role');
      const cells = ['hb', 'mode', 'ready', 'rel', 'resets', 'strikes', 'wcet', 'bad', 'miss'].map(id);
      if (!n.seen) { hc.className = 'chip muted'; setText(hc, 'not on the bus'); setText(role, '—'); for (const c of cells) { setText(c, '—'); c.classList.remove('stale'); } continue; }
      const lv = n.alive ? healthLevel(n.health) : 'crit';
      hc.className = `chip ${lv}`; setText(hc, n.alive ? n.health : 'SILENT');
      role.className = `chip ${n.role === 'hot' ? 'info' : 'muted'}`; setText(role, n.role ? n.role.toUpperCase() : 'VIRTUAL PEER');
      setText(id('hb'), n.heartbeat ? ago(n.hb_age) : n.alive ? 'none: samples only' : `silent ${ago(n.hb_age)}`);
      setText(id('mode'), n.mode_name);
      setText(id('ready'), n.ready == null ? '—' : n.ready ? 'READY' : 'calibrating');
      setText(id('rel'), n.release); setText(id('resets'), n.resets);
      setText(id('strikes'), n.strikes ? n.strikes.join(' · ') : '—');
      const c = n.console || {};
      setText(id('wcet'), c.wcet_frame != null ? `${(c.wcet_frame / 1000).toFixed(2)} ms` : '—');
      setText(id('bad'), `crc ${n.crc_bad} · seq ${n.seq_gaps}`);
      setText(id('miss'), c.missing != null ? c.missing : '—');
      const stale = !n.heartbeat && n.heartbeat !== undefined && n.seen && !n.alive;       // a silent node's numbers are the last it said: shown as such, not as current
      for (const x of ['mode', 'ready', 'strikes', 'wcet', 'bad', 'miss']) id(x).classList.toggle('stale', stale);
      id('mode').title = stale ? 'last reported before the node fell silent' : '';
    }
    const a = s.act, st = $id('m-act-state');
    if (!a) { st.className = 'chip muted'; setText(st, 'no output'); }
    else {
      st.className = `chip ${!a.alive ? 'crit' : a.state === 1 ? 'ok' : a.state === 0 ? 'info' : 'crit'}`; setText(st, a.alive ? a.state_name : 'SILENT');
      setText($id('m-act-vote'), a.vote_status); setText($id('m-act-voted'), NODES.filter((_, i) => a.voted[i]).join(' ') || 'none');
      setText($id('m-act-excl'), NODES.filter((_, i) => a.excluded[i]).join(' ') || 'none'); setText($id('m-act-cause'), a.cause);
      setText($id('m-act-p'), num(a.pitch, 3, '°')); setText($id('m-act-y'), num(a.yaw, 3, '°')); setText($id('m-act-held'), a.held ? 'YES' : 'no'); setText($id('m-act-n'), a.frames);
    }
    const sc = $id('m-sync-chip'); sc.className = s.sync.alive ? 's-ok' : s.frame == null ? 's-muted' : 's-crit'; setText(sc, s.sync.alive ? 'up' : s.frame == null ? 'none' : 'LOST');
    setText($id('m-frame'), s.frame ?? '—'); setText($id('m-rate'), s.frame_rate ? `${s.frame_rate.toFixed(2)} Hz` : '—'); setText($id('m-mf'), s.phase.mission_frame);
    setText($id('m-restarts'), s.sync.restarts); setText($id('m-rx'), eng(s.rx_total, 1)); setText($id('m-crc'), s.crc_bad_total);
    setText($id('m-oos'), Object.keys(s.oos).length ? Object.keys(s.oos).join(' ') : 'none');

    const al = s.alerts, key = JSON.stringify(al);
    if (R.alertKey !== key) {
      R.alertKey = key; R.alerts.innerHTML = '';
      if (!al.length) R.alerts.append(h('div', { class: 'empty' }, 'All nominal. Nothing the console can see is wrong.'));
      for (const x of al) R.alerts.append(h('div', { class: `alert ${x.level}` }, h('span', { class: 'lv' }, x.level === 'crit' ? 'CRIT' : x.level === 'warn' ? 'WARN' : 'INFO'), h('span', {}, x.text)));
      const c = $id('m-alert-n'); c.className = `chip ${al.some((x) => x.level === 'crit') ? 'crit' : al.length ? 'warn' : 'ok'}`; setText(c, al.length);
    }
    const sq = sequence(S), sk = JSON.stringify(sq.map((x) => [x.key, x.state, x.detail]));
    if (R.seqKey !== sk) {
      R.seqKey = sk; R.seq.innerHTML = '';
      for (const x of sq) R.seq.append(h('span', { class: `chip ${x.state === 'done' ? 'ok' : x.state === 'active' ? 'warn' : x.state === 'failed' ? 'crit' : 'muted'}`, title: x.detail }, x.state === 'done' ? '✓ ' : x.state === 'failed' ? '✗ ' : x.state === 'active' ? '▶ ' : '', x.label));
    }
    const t = s.truth, b = s.sim;
    setText($id('m-fl-src'), t ? 'simulator truth, 10 Hz' : b ? 'from the bus, 10 Hz' : 'no simulator');
    const rows = !t && !b ? [] : [
      ['Altitude', t ? t.alt : b.alt, 'm', 0], ['Range', t ? t.range : null, 'm', 0], ['Speed', t ? t.speed : b.speed, 'm/s', 0], ['Mach', t ? t.mach : null, '', 2], ['Dyn. pressure', t ? t.q : b.q, 'Pa', 0],
      ['Mass', t ? t.mass : b.mass, 'kg', 0], ['Thrust', t && !t.clamped ? t.thrust / 1000 : null, 'kN', 0], ['Pitch error', b ? b.err_p : null, '°', 2], ['Yaw error', b ? b.err_y : null, '°', 2],
      ['Engines on', t ? (t.clamped ? 'on the pad' : `${t.engines_on}/${t.engines}`) : b ? b.engines_on : null, '', 0],
    ];
    const rk = JSON.stringify(rows.map((r) => r[0]));
    if (R.statKey !== rk) { R.statKey = rk; R.stats.innerHTML = ''; R.statEls = rows.map((r) => { const v = h('div', { class: 'v' }); R.stats.append(h('div', { class: 'stat' }, h('div', { class: 'k' }, r[0]), v)); return v; }); if (!rows.length) R.stats.append(h('div', { class: 'empty' }, 'No simulator is running.')); }
    rows.forEach((r, i) => { const v = R.statEls[i], val = r[1]; const txt = val == null ? '—' : typeof val === 'string' ? val : (Math.abs(val) >= 10000 ? eng(val, 1) : val.toFixed(r[3])); v.textContent = ''; v.append(txt, h('small', {}, val == null ? '' : r[2])); });
    R.cAlt.draw(S.hist); R.cSpd.draw(S.hist);
    const ev = S.events.filter((e) => e.src !== 'bus' || ['countdown', 'scrub', 't-zero', 'sync-lost', 'sync-restart', 'node-silent', 'node-back', 'view', 'safe-request'].includes(e.kind)).slice(-40).reverse();
    const ek = ev.length ? ev[0].seq : 0;
    if (R.evKey !== ek) {
      R.evKey = ek; R.events.innerHTML = '';
      if (!ev.length) R.events.append(h('div', { class: 'empty' }, 'Nothing has happened yet.'));
      for (const e of ev.slice(0, 14)) R.events.append(h('div', { class: `log-line ${e.level}` }, h('span', { class: `tag n${e.src}`, style: { marginRight: '8px' } }, e.src), e.frame != null ? h('span', { class: 's-muted' }, `f${e.frame} `) : '', h('span', { class: 'clamp2', style: { display: 'inline' } }, e.text.replace(/^\[\w+\] /, ''))));
    }
  },
};
