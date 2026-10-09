// Mission: the whole system at a glance: the three computers, ACT, the sync, what is wrong, how the flight is going, and what has happened.
import { h, setText, num, eng, ago, clock, icon, NODES } from '../util.js';
import { sequence, healthLevel } from '../derive.js';

let R = {};
const kvRow = (k, id) => [h('dt', {}, k), h('dd', { id })];

function nodeCard(n) {
  const id = (s) => `m-${n}-${s}`;
  return h('article', { class: `card nodecard n${n}` },
    h('header', {}, h('div', { class: 'title' }, h('b', {}, `FC-${n}`), h('span', { id: id('role'), class: 'chip muted' }, '—')), h('div', { class: 'tools' }, h('span', { id: id('health'), class: 'chip muted' }, 'not seen'))),
    h('div', { class: 'body' },
      h('dl', { class: 'kv' },
        kvRow('heartbeat', id('hb')), kvRow('sees system as', id('mode')), kvRow('ready for launch', id('ready')), kvRow('release', id('rel')), kvRow('resets', id('resets')),
        kvRow('strikes A·B·C', id('strikes')), kvRow('frame time', id('wcet')), kvRow('bad frames', id('bad')))));
}

export default {
  id: 'mission', label: 'Mission', icon: 'mission',
  mount(root) {
    R.nodes = h('div', { class: 'grid', style: { gridTemplateColumns: 'repeat(auto-fit, minmax(215px, 1fr))' } },
      NODES.map(nodeCard),
      h('article', { class: 'card nodecard nACT' },
        h('header', {}, h('div', { class: 'title' }, h('b', {}, 'ACT'), h('span', { class: 'note' }, 'actuator node')), h('div', { class: 'tools' }, h('span', { id: 'm-act-state', class: 'chip muted' }, '—'))),
        h('div', { class: 'body' }, h('dl', { class: 'kv' }, kvRow('vote status', 'm-act-vote'), kvRow('nodes in the vote', 'm-act-voted'), kvRow('excluded', 'm-act-excl'), kvRow('cause of Safe', 'm-act-cause'), kvRow('output pitch', 'm-act-p'), kvRow('output yaw', 'm-act-y'), kvRow('holding', 'm-act-held'), kvRow('output frames', 'm-act-n')))),
      h('article', { class: 'card nodecard nbus' },
        h('header', {}, h('div', { class: 'title' }, h('b', {}, 'SYNC & BUS')), h('div', { class: 'tools' }, h('span', { id: 'm-sync-chip', class: 'chip muted' }, '—'))),
        h('div', { class: 'body' }, h('dl', { class: 'kv' }, kvRow('frame number', 'm-frame'), kvRow('frame rate', 'm-rate'), kvRow('mission frame', 'm-mf'), kvRow('sync restarts', 'm-restarts'), kvRow('frames received', 'm-rx'), kvRow('CRC failures', 'm-crc'), kvRow('out-of-schedule ids', 'm-oos')))));

    R.alerts = h('div', { class: 'alerts', 'aria-live': 'polite' });
    R.stats = h('div', { class: 'stats' });
    R.seq = h('div', { style: { display: 'flex', gap: '6px', flexWrap: 'wrap' } });
    R.events = h('div', { class: 'scroll', style: { maxHeight: '300px' } });
    root.append(h('div', { class: 'stack' },
      R.nodes,
      h('div', { class: 'grid c-2-1' },
        h('div', { class: 'stack' },
          h('article', { class: 'card' }, h('header', {}, h('h3', {}, 'Launch sequence'), h('div', { class: 'tools' }, h('button', { class: 'btn small ghost', onclick: () => R.ctx.go('launch') }, 'Open Launch →'))), h('div', { class: 'body' }, R.seq)),
          h('article', { class: 'card' }, h('header', {}, h('h3', {}, 'The vehicle'), h('div', { class: 'tools' }, h('span', { id: 'm-fl-src', class: 'note' }), h('button', { class: 'btn small ghost', onclick: () => R.ctx.go('flight') }, 'Open Flight →'))), h('div', { class: 'body' }, R.stats))),
        h('div', { class: 'stack' },
          h('article', { class: 'card' }, h('header', {}, h('h3', {}, 'Alerts'), h('div', { class: 'tools' }, h('span', { id: 'm-alert-n', class: 'chip muted' }, '0'))), h('div', { class: 'body' }, R.alerts)),
          h('article', { class: 'card' }, h('header', {}, h('h3', {}, 'Latest events'), h('div', { class: 'tools' }, h('button', { class: 'btn small ghost', onclick: () => R.ctx.go('events') }, 'All events →'))), h('div', { class: 'body flush' }, R.events))))));
  },
  update(S, ctx) {
    R.ctx = ctx;
    const s = S.snap; if (!s) return;
    const $id = (x) => document.getElementById(x);
    for (const n of s.nodes) {
      const id = (x) => $id(`m-${n.name}-${x}`);
      const hc = id('health'), role = id('role');
      if (!n.seen) { hc.className = 'chip muted'; setText(hc, 'not on the bus'); setText(role, '—'); for (const k of ['hb', 'mode', 'ready', 'rel', 'resets', 'strikes', 'wcet', 'bad']) setText(id(k), '—'); continue; }
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
      setText(id('bad'), `crc ${n.crc_bad} · seq ${n.seq_gaps}${c.missing ? ' · missing ' + c.missing : ''}`);
    }
    const a = s.act, st = $id('m-act-state');
    if (!a) { st.className = 'chip muted'; setText(st, 'no output'); }
    else {
      st.className = `chip ${!a.alive ? 'crit' : a.state === 1 ? 'ok' : a.state === 0 ? 'info' : 'crit'}`; setText(st, a.alive ? a.state_name : 'SILENT');
      setText($id('m-act-vote'), a.vote_status); setText($id('m-act-voted'), NODES.filter((_, i) => a.voted[i]).join(' ') || 'none');
      setText($id('m-act-excl'), NODES.filter((_, i) => a.excluded[i]).join(' ') || 'none'); setText($id('m-act-cause'), a.cause);
      setText($id('m-act-p'), num(a.pitch, 3, '°')); setText($id('m-act-y'), num(a.yaw, 3, '°')); setText($id('m-act-held'), a.held ? 'YES' : 'no'); setText($id('m-act-n'), a.frames);
    }
    const sc = $id('m-sync-chip'); sc.className = `chip ${s.sync.alive ? 'ok' : s.frame == null ? 'muted' : 'crit'}`; setText(sc, s.sync.alive ? 'SYNC up' : s.frame == null ? 'no SYNC' : 'SYNC LOST');
    setText($id('m-frame'), s.frame ?? '—'); setText($id('m-rate'), s.frame_rate ? `${s.frame_rate.toFixed(2)} Hz` : '—'); setText($id('m-mf'), s.phase.mission_frame);
    setText($id('m-restarts'), s.sync.restarts); setText($id('m-rx'), eng(s.rx_total, 1)); setText($id('m-crc'), s.crc_bad_total);
    setText($id('m-oos'), Object.keys(s.oos).length ? Object.keys(s.oos).join(' ') : 'none');

    // alerts
    const al = s.alerts;
    const key = JSON.stringify(al);
    if (R.alertKey !== key) {
      R.alertKey = key; R.alerts.innerHTML = '';
      if (!al.length) R.alerts.append(h('div', { class: 'empty' }, 'All nominal. Nothing the console can see is wrong.'));
      for (const x of al) R.alerts.append(h('div', { class: `alert ${x.level}` }, h('span', { class: 'lv' }, x.level === 'crit' ? 'CRIT' : x.level === 'warn' ? 'WARN' : 'INFO'), h('span', {}, x.text)));
      const c = $id('m-alert-n'); c.className = `chip ${al.some((x) => x.level === 'crit') ? 'crit' : al.length ? 'warn' : 'ok'}`; setText(c, al.length);
    }
    // the sequence as chips
    const sq = sequence(S), sk = JSON.stringify(sq.map((x) => [x.key, x.state, x.detail]));
    if (R.seqKey !== sk) {
      R.seqKey = sk; R.seq.innerHTML = '';
      for (const x of sq) R.seq.append(h('span', { class: `chip ${x.state === 'done' ? 'ok' : x.state === 'active' ? 'warn' : x.state === 'failed' ? 'crit' : 'muted'}`, title: x.detail }, x.state === 'done' ? '✓ ' : x.state === 'failed' ? '✗ ' : x.state === 'active' ? '▶ ' : '', x.label));
    }
    // the vehicle
    const t = s.truth, b = s.sim;
    setText($id('m-fl-src'), t ? 'simulator truth, 10 Hz' : b ? 'from the bus, 10 Hz' : 'no simulator');
    const rows = !t && !b ? [] : [
      ['Altitude', t ? t.alt : b.alt, 'm', 0], ['Range', t ? t.range : null, 'm', 0], ['Speed', t ? t.speed : b.speed, 'm/s', 0], ['Mach', t ? t.mach : null, '', 2], ['Dyn. pressure', (t ? t.q : b.q), 'Pa', 0],
      ['Mass', t ? t.mass : b.mass, 'kg', 0], ['Thrust', t ? t.thrust / 1000 : null, 'kN', 0], ['Pitch error', b ? b.err_p : null, '°', 2], ['Yaw error', b ? b.err_y : null, '°', 2],
      ['Engines on', t ? `${t.engines_on}/${t.engines}` : b ? b.engines_on : null, '', 0],
    ];
    const rk = JSON.stringify(rows.map((r) => r[0]));
    if (R.statKey !== rk) { R.statKey = rk; R.stats.innerHTML = ''; R.statEls = rows.map((r) => { const v = h('div', { class: 'v' }); R.stats.append(h('div', { class: 'stat' }, h('div', { class: 'k' }, r[0]), v)); return v; }); if (!rows.length) R.stats.append(h('div', { class: 'empty' }, 'No simulator is running.')); }
    rows.forEach((r, i) => { const v = R.statEls[i], val = r[1]; const txt = val == null ? '—' : typeof val === 'string' ? val : (Math.abs(val) >= 10000 ? eng(val, 1) : val.toFixed(r[3])); setText(v.firstChild || v, ''); v.textContent = ''; v.append(txt, h('small', {}, val == null ? '' : r[2])); });
    // events
    const ev = S.events.filter((e) => e.src !== 'bus' || ['countdown', 'scrub', 't-zero', 'sync-lost', 'sync-restart', 'node-silent', 'node-back', 'view', 'safe-request'].includes(e.kind)).slice(-40).reverse();
    const ek = ev.length ? ev[0].seq : 0;
    if (R.evKey !== ek) {
      R.evKey = ek; R.events.innerHTML = '';
      if (!ev.length) R.events.append(h('div', { class: 'empty' }, 'Nothing has happened yet.'));
      for (const e of ev.slice(0, 14)) R.events.append(h('div', { class: `log-line ${e.level}` }, h('span', { class: `tag n${e.src}`, style: { marginRight: '8px' } }, e.src), e.frame != null ? h('span', { class: 's-muted' }, `f${e.frame} `) : '', h('span', { class: 'clamp2', style: { display: 'inline' } }, e.text.replace(/^\[\w+\] /, ''))));
    }
  },
};
