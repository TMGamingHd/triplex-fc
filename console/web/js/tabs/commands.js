// Commands: the operator's console. Every command is an authenticated ground frame (ADR-019); the ones that can do harm are two steps (ARM, then EXECUTE) and ask for a confirmation here first. The flight computers
// enforce the rules again and answer on their consoles; the log below shows the answers.
import { h, setText, modal, toast, icon, NODES } from '../util.js';
import { suggestions } from '../derive.js';
import { sendCommand, keyNote } from '../ops.js';

let R = { sel: {} };

function targetControl(c, S) {
  const op = c.op;
  if (c.target === 'none') return h('span', { class: 'note' }, 'no target');
  if (c.target === 'phase') {
    const sel = h('select', { 'aria-label': 'Phase' }, S.cfg.phases.map((p) => h('option', { value: p.name }, `${p.n} · ${p.name} (${p.nominal} hot, min ${p.minimum})`)));
    sel.value = 'ascent'; R.sel[op] = () => sel.value; return sel;
  }
  const opts = [...NODES, ...(c.target === 'computer-or-imu' ? NODES.map((n) => `IMU-${n}`) : [])];
  let cur = R.sel[op + ':v'] || 'B';
  const seg = h('div', { class: 'seg', role: 'group', 'aria-label': `Target of ${c.label}` }, opts.map((o) => h('button', { type: 'button', 'aria-pressed': o === cur ? 'true' : 'false', dataset: { v: o }, onclick: (e) => { cur = o; R.sel[op + ':v'] = o; for (const b of seg.children) b.setAttribute('aria-pressed', b.dataset.v === o ? 'true' : 'false'); } }, o)));
  R.sel[op] = () => cur;
  return seg;
}

/** Would taking this computer out of the vote leave too few voters? (the interlock's first tier; the phase tiers are the computers' to enforce) */
function leaves(S, target) {
  const healthy = S.snap.nodes.filter((n) => n.alive && n.health === 'healthy').length;
  const isNode = NODES.includes(target);
  const n = S.snap.nodes.find((x) => x.name === target);
  return isNode && n && n.health === 'healthy' ? healthy - 1 : healthy;
}

async function run(c, S, mode, target) {
  const arm = c.arm === 'always' || mode === 'armed';
  const tgt = target ?? (R.sel[c.op] ? R.sel[c.op]() : '');
  if (arm) {
    const left = ['disable', 'warm'].includes(c.op) ? leaves(S, tgt) : null;
    const r = await modal({
      title: `${c.label}${tgt ? ' ' + tgt : ''}: confirm`,
      body: [h('p', { style: { margin: 0 } }, c.doc), h('div', { class: 'banner warn' }, icon('alert'), h('div', {}, 'This is a ', h('b', {}, 'two-step'), ' command: the console sends an ARM frame, then the EXECUTE frame 50 ms later. The flight computers accept the EXECUTE only with a matching ARM inside ', h('b', {}, `${S.cfg.arm_window_frames / 100} s`), '.',
        left != null ? h('div', {}, `After this, ${left} computer${left === 1 ? '' : 's'} would still be voting.`) : null))],
      buttons: [{ label: 'Cancel', value: 'no' }, { label: `ARM and send ${c.op}`, kind: 'danger', value: 'yes' }],
    });
    if (r !== 'yes') return;
    await sendCommand({ op: c.op, target: tgt, mode: 'armed', confirmed: true });
  } else await sendCommand({ op: c.op, target: tgt, mode: 'single' });
}

function card(c, S) {
  const tgt = targetControl(c, S);
  const arm = c.arm === 'always' ? h('span', { class: 'chip warn', title: 'always needs an ARM first' }, 'ARM') : c.arm === 'tiered' ? h('span', { class: 'chip info', title: 'ARM needed when it would leave fewer than the phase’s nominal number of voters' }, 'ARM if few voters left') : h('span', { class: 'chip muted' }, 'no ARM');
  const dangerous = ['launch', 'clear-safe', 'clear-disabled', 'disable'].includes(c.op);
  const btns = [h('button', { class: `btn ${dangerous ? 'warn' : ''}`, id: `cmd-${c.op}`, onclick: () => run(c, S, c.arm === 'always' ? 'armed' : 'single') }, c.arm === 'always' ? 'ARM + send…' : 'Send')];
  if (c.arm === 'tiered') btns.push(h('button', { class: 'btn warn', id: `cmd-${c.op}-arm`, onclick: () => run(c, S, 'armed') }, 'ARM + send…'));
  return h('article', { class: 'card', id: `cmdcard-${c.op}` }, h('header', {}, h('h3', {}, c.label), h('div', { class: 'tools' }, arm)), h('div', { class: 'body' }, h('p', { class: 'note', style: { marginTop: 0, minHeight: '54px' } }, c.doc), h('div', { style: { display: 'flex', gap: '8px', alignItems: 'center', flexWrap: 'wrap' } }, tgt, h('span', { class: 'spacer' }), h('div', { class: 'btns' }, btns))));
}

export default {
  id: 'commands', label: 'Commands', icon: 'commands',
  mount(root) {
    R.root = root;
    R.sug = h('div', { class: 'alerts' });
    R.grid = h('div', { class: 'grid', style: { gridTemplateColumns: 'repeat(auto-fill, minmax(330px, 1fr))' } });
    R.log = h('tbody');
    R.status = h('div', { class: 'banner info' });
    R.adv = h('details', {}, h('summary', {}, 'Advanced: test the authentication and the interlock'));
    root.append(h('div', { class: 'stack' }, R.status,
      h('article', { class: 'card' }, h('header', {}, h('h3', {}, 'Suggested now'), h('div', { class: 'tools' }, h('span', { class: 'note' }, 'from what the computers report'))), h('div', { class: 'body' }, R.sug)),
      R.grid,
      h('article', { class: 'card' }, h('header', {}, h('h3', {}, 'Command log'), h('div', { class: 'tools' }, h('span', { class: 'note' }, 'the answers are the nodes’ own, from their consoles'))), h('div', { class: 'body flush scroll' }, h('table', {}, h('thead', {}, h('tr', {}, h('th', {}, 'time'), h('th', {}, 'frame'), h('th', {}, 'command'), h('th', {}, 'frames sent (counter)'), h('th', {}, 'result'), h('th', {}, 'answers'))), R.log))),
      R.adv));
  },
  update(S, ctx) {
    const s = S.snap; if (!s || !S.cfg) return;
    if (!R.built) {
      R.built = true;
      for (const c of S.cfg.commands) R.grid.append(card(c, S));
      const modeSel = h('select', { id: 'adv-mode' }, [['forged', 'forged: a wrong tag (what a node without the key sends): refused without a trace'], ['arm', 'ARM only: the EXECUTE never follows'], ['execute', 'EXECUTE only: no ARM before it'], ['replay', 'replay: the last frame again (a stale counter)']].map(([v, t]) => h('option', { value: v }, t)));
      const opSel = h('select', { id: 'adv-op' }, S.cfg.commands.map((c) => h('option', { value: c.op }, c.op)));
      const tgt = h('input', { type: 'text', id: 'adv-target', value: 'B', size: 8, 'aria-label': 'Target' });
      R.adv.append(h('div', { class: 'banner warn', style: { margin: '8px 0' } }, icon('alert'), h('div', {}, h('b', {}, 'Test actions. '), 'These send frames that a correct operator never sends, to show the flight computers refuse them (fault-matrix rows on authentication and the interlock). A forged or replayed frame leaves no trace on the node’s console except a counter.')),
        h('div', { class: 'grid cols3' }, h('label', { class: 'field' }, 'Command', opSel), h('label', { class: 'field' }, 'Target', tgt), h('label', { class: 'field' }, 'Kind', modeSel)),
        h('div', { class: 'btns', style: { marginTop: '10px' } }, h('button', { class: 'btn warn', id: 'adv-send', onclick: async () => { await sendCommand({ op: opSel.value, target: tgt.value, mode: modeSel.value, confirmed: true, force: true }); } }, 'Send the test frame')));
    }
    const c = s.commands || {};
    R.status.className = `banner ${c.can_send ? 'info' : 'warn'}`;
    R.status.innerHTML = '';
    R.status.append(icon('info'), h('div', {}, c.can_send ? [`Commands go out on `, h('b', { class: 'mono' }, c.iface), '. ', keyNote, ' Counters are shared with the command line (tfc_peers command): a counter is never reused.'] : 'This console is not attached to a live bus (a replay is read-only), so it cannot send commands.'));
    // suggestions
    const sg = suggestions(S), key = JSON.stringify(sg);
    if (R.skey !== key) {
      R.skey = key; R.sug.innerHTML = '';
      if (!sg.length) R.sug.append(h('div', { class: 'empty' }, 'Nothing to do: no computer is latched, ACT is not in Safe, no countdown is running.'));
      for (const x of sg) R.sug.append(h('div', { class: `alert ${x.level}` }, h('span', { class: 'lv' }, x.level === 'crit' ? 'CRIT' : x.level === 'warn' ? 'WARN' : 'INFO'), h('span', { style: { flex: 1 } }, x.text),
        x.op ? h('button', { class: 'btn small', onclick: () => { const cmd = S.cfg.commands.find((q) => q.op === x.op); run(cmd, S, x.mode, x.target); } }, x.label) : null));
    }
    // the log
    const log = (c.log || []).slice().reverse(), lk = JSON.stringify(log.map((r) => [r.id, r.status, r.responses.length]));
    if (R.lkey !== lk) {
      R.lkey = lk; R.log.innerHTML = '';
      if (!log.length) R.log.append(h('tr', {}, h('td', { colspan: 6, class: 'empty' }, 'No command has been sent from this console yet.')));
      for (const r of log) R.log.append(h('tr', {}, h('td', { class: 'mono' }, new Date(r.wall * 1000).toTimeString().slice(0, 8)), h('td', { class: 'mono' }, r.frame ?? '—'), h('td', {}, h('b', {}, r.op), ' ', r.target, r.mode !== 'single' ? h('span', { class: 'chip info', style: { marginLeft: '6px' } }, r.mode) : ''),
        h('td', { class: 'mono' }, r.frames.map((f) => `${f.arm ? 'ARM' : f.forged ? 'forged' : 'exec'} ${f.counter}`).join(', ')),
        h('td', {}, h('span', { class: `chip ${r.status === 'accepted' ? 'ok' : r.status === 'refused' ? 'crit' : r.status === 'no answer' ? 'warn' : 'muted'}` }, r.status)),
        h('td', { class: 'mono', style: { fontSize: '12px' } }, r.responses.length ? r.responses.map((x) => `${x.src}${x.arm ? ' (ARM)' : ''}: ${x.result}`).join('\n') : '')));
    }
    for (const b of R.grid.querySelectorAll('button')) b.disabled = !c.can_send;
    const adv = document.getElementById('adv-send'); if (adv) adv.disabled = !c.can_send;
  },
};
