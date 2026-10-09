// A smoke test that runs inside the page against a live console: `?selftest=base` (any rig), `?selftest=faults` (the fault lab running) or `?selftest=launch` (the closed loop on the pad: it LAUNCHES, flies, and kills node B in flight). It clicks the real buttons, waits for the real answers
// from the real firmware, and prints PASS or FAIL for each step in a panel (and in the page title), so a headless browser can read the verdict from a screenshot. It sends only harmless commands
// (a no-op) and, in the fault lab, injects one fault and clears it. It is a development aid, not part of the console's function.
import { $, h } from './util.js';
import { S } from './state.js';

const out = h('pre', { id: 'selftest', style: { position: 'fixed', right: '10px', top: '66px', width: '520px', maxHeight: '86vh', overflow: 'auto', zIndex: 80, background: 'rgba(5,8,14,.96)', color: '#dbe3f1', border: '1px solid #4cc9f0', borderRadius: '8px', padding: '10px', font: '12px/1.45 monospace', whiteSpace: 'pre-wrap', margin: 0 } });
let fails = 0, passes = 0;
const log = (ok, msg) => { out.textContent += `${ok ? 'PASS' : 'FAIL'}  ${msg}\n`; ok ? passes++ : fails++; document.title = `SELFTEST ${fails ? 'FAIL' : 'ok'} ${passes}/${passes + fails}`; };
const sleep = (ms) => new Promise((r) => setTimeout(r, ms));
async function waitFor(fn, ms = 8000, step = 100) { const end = performance.now() + ms; while (performance.now() < end) { try { const v = fn(); if (v) return v; } catch { /* not yet */ } await sleep(step); } return null; }
const click = (sel) => { const el = $(sel); if (!el) throw new Error(`no ${sel}`); el.click(); return el; };
const tab = async (id) => { click(`#tab-${id}`); await sleep(250); };

export async function run(kind, ctx) {
  document.body.append(out);
  out.textContent = `selftest ${kind}\n`;
  try {
    // ---- every tab mounts and shows something
    for (const t of ctx.tabs) { await tab(t.id); const pane = $(`#pane-${t.id}`); log(pane.classList.contains('active') && pane.querySelectorAll('*').length > 15, `tab ${t.id} renders (${pane.querySelectorAll('*').length} elements)`); }
    log(!$('#js-error') && !$('#boot-error'), 'no page error so far');
    const s0 = S.snap;
    log(s0 && s0.source.kind === 'live', `live source: ${s0 && s0.source.iface}`);
    log(s0 && s0.sync.alive, `SYNC alive at ${s0 && s0.frame_rate} Hz`);
    log(s0.nodes.every((n) => n.alive), 'three nodes alive');
    // ---- keyboard
    document.dispatchEvent(new KeyboardEvent('keydown', { key: '3', bubbles: true })); await sleep(200);
    log($('#pane-voting').classList.contains('active'), 'key 3 selects Voting');
    // ---- theme
    const t0 = document.documentElement.dataset.theme; click('#theme-btn'); await sleep(100);
    log(document.documentElement.dataset.theme !== t0, 'theme toggles'); click('#theme-btn');
    // ---- the launch dialog (cancelled: nothing is sent)
    await tab('launch');
    const lb = $('#btn-launch');
    if (lb && !lb.disabled) {
      lb.click();
      const inp = await waitFor(() => $('#launch-confirm'), 2000);
      log(!!inp, 'the launch asks for a second confirmation');
      const go = $('#launch-go');
      log(go && go.disabled, 'the launch button in the dialog is disabled until LAUNCH is typed');
      inp.value = 'launch'; inp.dispatchEvent(new Event('input')); log($('#launch-go').disabled, 'lower-case "launch" does not enable it');
      inp.value = 'LAUNCH'; inp.dispatchEvent(new Event('input')); log(!$('#launch-go').disabled, 'typing LAUNCH enables it');
      document.dispatchEvent(new KeyboardEvent('keydown', { key: 'Escape' })); await sleep(200);
      log(!$('#launch-confirm'), 'Escape closes the dialog without sending');
      log(!((S.snap.commands.log || []).some((r) => r.op === 'launch')), 'no launch command was sent');
    } else log(true, `launch button not enabled (${S.snap.phase.name}, go=${S.snap.go_nogo.go}): dialog test skipped`);
    // ---- a no-op command through the page
    await tab('commands');
    const before = (S.snap.commands.log || []).length;
    click('#cmd-noop');
    const rec = await waitFor(() => { const l = S.snap.commands.log || []; return l.length > before && l[l.length - 1]; }, 4000);
    log(!!rec, 'the no-op was sent');
    const answered = rec && await waitFor(() => (S.snap.commands.log.find((r) => r.id === rec.id) || {}).status === 'accepted', 6000);
    log(!!answered, 'a flight computer answered the no-op: accepted (the real firmware, its own console)');
    // ---- the vehicle: the simulator's own reader
    await tab('vehicle');
    await waitFor(() => $('#vehicle-json') && $('#vehicle-json').value.length > 100, 5000);
    click('#vehicle-validate');
    const ok = await waitFor(() => /stage|engine|aerodynamics/.test($('#pane-vehicle pre').textContent), 6000);
    log(!!ok, 'the vehicle validates with tfc_fly --check');
    // ---- the bus and the events
    await tab('bus'); const rows = await waitFor(() => $('#pane-bus tbody tr') && $$rows('#pane-bus tbody tr') > 10, 4000); log(!!rows, 'the bus table lists the ids');
    await tab('events'); log($$rows('#pane-events .log-line') > 5, 'the event list has entries');
    if (kind === 'faults') await faults(ctx);
    if (kind === 'launch') await launch(ctx);
  } catch (e) { log(false, `exception: ${e.message}`); }
  log(!$('#js-error') && !$('#boot-error'), 'no page error at the end');
  out.textContent += `\nDONE ${passes} passed, ${fails} failed\n`;
  document.title = `SELFTEST ${fails ? 'FAIL' : 'PASS'} ${passes}/${passes + fails}`;
}
const $$rows = (sel) => document.querySelectorAll(sel).length;

async function faults() {
  await tab('faults');
  log(S.snap.faults.available, 'the fault lab is running');
  const kind = $('#fault-kind'); kind.value = 'bias'; kind.dispatchEvent(new Event('change')); await sleep(150);
  $('#fault-forever').checked = true;
  const spec = $('#pane-faults .mono-box').textContent;
  log(spec === 'B:bias', `the spec is ${spec}`);
  const f0 = S.snap.nodes[1].health;
  click('#fault-inject');
  const row = await waitFor(() => (S.snap.faults.table || []).length === 1 && S.snap.faults.table[0], 4000);
  log(!!row, `the fault is in the scenario: ${row && row.spec}`);
  const det = await waitFor(() => (S.snap.faults.table[0] || {}).detected, 6000);
  log(!!det && det.frames_after >= 0 && det.frames_after <= 8, `detected ${det ? '+' + det.frames_after + ' frames (' + det.by + (det.reason ? ': ' + det.reason : '') + ')' : 'never'}`);
  log(await waitFor(() => S.snap.nodes[1].health === 'latched', 3000) != null, `node B is latched in the others' view (was ${f0})`);
  log(await waitFor(() => Math.min(...S.snap.nodes.filter((n) => n.heartbeat).map((n) => n.mode)) === 2, 3000) != null, 'the system is in Duplex');
  click('#fault-clear-all');
  log(await waitFor(() => (S.snap.faults.table || []).length === 0, 4000) != null, 'the fault is cleared');
  // readmit B through the page: reintegrate B, then it must go through probation back to healthy
  await new Promise((r) => setTimeout(r, 2500));      // the dwell
  await tab('commands');
  const b = document.querySelector('#cmdcard-reintegrate .seg button[data-v="B"]'); if (b) b.click();
  const n0 = (S.snap.commands.log || []).length;
  click('#cmd-reintegrate');
  const rec = await waitFor(() => { const l = S.snap.commands.log || []; return l.length > n0 && l[l.length - 1]; }, 4000);
  log(!!rec && rec.target === 'B', 'reintegrate B was sent');
  log(await waitFor(() => (S.snap.commands.log.find((r) => r.id === rec.id) || {}).status === 'accepted', 6000) != null, 'the flight computers accepted reintegrate B');
  log(await waitFor(() => S.snap.nodes[1].health === 'probation' || S.snap.nodes[1].health === 'healthy', 4000) != null, 'node B is on probation or healthy again');
  log(await waitFor(() => S.snap.nodes.every((n) => n.health === 'healthy'), 9000) != null, 'all three healthy again after the probation: Triplex restored');
}

async function launch() {
  // the whole launch through the page: the dialog, the typed confirmation, the countdown, the flight, the trail, and the loss of a node in flight
  await tab('launch');
  const go = await waitFor(() => S.snap.go_nogo.go && !$('#btn-launch').disabled, 40000);
  log(!!go, 'the checklist is GO and the launch button is enabled');
  if (!go) return;
  click('#btn-launch');
  const inp = await waitFor(() => $('#launch-confirm'), 2000);
  inp.value = 'LAUNCH'; inp.dispatchEvent(new Event('input')); click('#launch-go');
  log(!!(await waitFor(() => S.snap.phase.name === 'countdown', 4000)), 'the countdown started');
  log(await waitFor(() => $('#clock-v').textContent.startsWith('T-'), 2000) != null, 'the header clock shows T-');
  log(await waitFor(() => (S.snap.commands.log.find((r) => r.op === 'launch') || {}).status === 'accepted', 6000) != null, 'the launch command is accepted by the flight computers');
  log(await waitFor(() => S.snap.phase.name === 'flight', 15000) != null, 'T-zero: the flight began');
  log(await waitFor(() => S.snap.truth && S.snap.truth.alt > 100, 15000) != null, 'the vehicle climbed past 100 m');
  await tab('flight');
  log(await waitFor(() => /T\+/.test($('#clock-v').textContent), 1000) != null, 'the header clock shows T+');
  await sleep(1500);
  log(document.querySelectorAll('#pane-flight canvas').length >= 9, 'the flight tab drew its charts');
  // the loss of a node, through the page
  await tab('faults');
  const kill = [...document.querySelectorAll('#pane-faults tbody button')].find((b) => b.textContent === 'Kill' && b.closest('tr').textContent.includes('flight computer B'));
  log(!!kill, 'the Faults tab offers Kill for flight computer B');
  if (kill) kill.click();
  log(await waitFor(() => S.snap.nodes[1].health === 'latched', 4000) != null, 'node B is latched');
  log(await waitFor(() => S.snap.act.excluded[1], 3000) != null, 'ACT excluded node B');
  log(await waitFor(() => !S.snap.nodes[1].alive, 3000) != null, 'node B is silent');
  await tab('mission');
  log(await waitFor(() => document.querySelector('#pane-mission .alert.crit'), 3000) != null, 'the Mission tab raises a critical alert');
  await tab('events');
  log(S.events.some((e) => /TEST ACTION: node B killed/.test(e.text)), 'the event log names the kill as a test action');
}
