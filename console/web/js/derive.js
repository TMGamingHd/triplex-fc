// What the page works out from the snapshot: the launch sequence, the suggestions for the operator. Pure functions of the state, so they can be read on their own.
import { NODES } from './util.js';

/** The launch sequence as steps: {key, label, state: done|active|pending|failed, detail, t (source seconds or null)}. */
export function sequence(S) {
  const s = S.snap;
  if (!s) return [];
  const ms = s.milestones || {};
  const nodes = s.nodes;
  const alive = nodes.filter((n) => n.alive);
  const step = (key, label, ok, detail, t = null, failed = false) => ({ key, label, state: failed ? 'failed' : ok ? 'done' : 'pending', detail, t });
  // a pre-launch step: once the countdown has begun it was met, and what is true now (a computer lost in flight) is not its business
  const pre = (key, label, okNow, detail) => step(key, label, okNow || past, past && !okNow ? 'met before the countdown' : detail);
  const out = [];
  const inFlight = s.phase.name === 'flight', inCd = s.phase.name === 'countdown';
  const past = inFlight || inCd;                               // once the countdown has begun, the earlier steps were met
  out.push(pre('sync', 'SYNC on the bus', s.sync.alive, s.sync.alive ? `frame ${s.frame}, ${s.frame_rate.toFixed(1)} Hz` : 'waiting for a sync master'));
  out.push(pre('nodes', 'Three flight computers up', alive.length === 3, `${alive.length} of 3 heartbeats`));
  const beating = nodes.filter((n) => n.heartbeat);
  out.push(pre('triplex', 'Triplex: all three voting', beating.length === 3 && beating.every((n) => n.mode === 3), beating.length ? beating.map((n) => `${n.name} ${n.mode_name}`).join(' · ') : '—'));
  const ready = nodes.filter((n) => n.heartbeat && n.ready).length;
  out.push(pre('ready', 'IMUs calibrated (10 s at rest)', ready === 3, `${ready} of 3 ready`));
  out.push(pre('act', 'ACT Nominal', !!(s.act && s.act.alive && s.act.state === 1), s.act ? s.act.state_name : 'no ACT output'));
  out.push(pre('go', 'GO for launch', s.go_nogo.go, s.go_nogo.go ? 'every required item holds' : (s.go_nogo.reasons[0] || '—')));
  const scrub = (S.events || []).slice().reverse().find((e) => e.kind === 'scrub' || e.kind === 'scrubbed');
  const scrubbed = scrub && !past && ms.countdown == null;
  out.push(step('countdown', 'Countdown T-10 s', past, inCd ? `T-${s.phase.t_minus_s.toFixed(1)} s` : scrubbed ? 'SCRUBBED: ' + scrub.text.replace(/^\[\w+\] /, '') : 'not started', ms.countdown ?? null, scrubbed));
  out.push(step('t0', 'T-zero', inFlight, inFlight ? `T+${s.phase.flight_s.toFixed(1)} s` : '—', ms.t0 ?? null));
  out.push(step('liftoff', 'Lift-off (the vehicle leaves the pad)', ms.liftoff != null || (s.truth && s.truth.clamped === 0 && inFlight), s.truth ? `altitude ${s.truth.alt.toFixed(0)} m` : s.sim ? `altitude ${s.sim.alt.toFixed(0)} m` : '—', ms.liftoff ?? null));
  out.push(step('maxq', 'Max-Q passed', ms['max-q'] != null, ms['max-q'] != null ? 'dynamic pressure past its peak' : s.sim && s.sim.q != null ? `q = ${(s.sim.q / 1000).toFixed(1)} kPa` : '—', ms['max-q'] ?? null));
  let activeSet = false;                                      // the first step not yet done is the one in progress
  for (const st of out) { if (st.state === 'pending' && !activeSet) { st.state = 'active'; activeSet = true; } }
  return out;
}

/** What the operator might want to do now, from what the computers report. */
export function suggestions(S) {
  const s = S.snap, out = [];
  if (!s) return out;
  for (const [i, n] of s.nodes.entries()) {
    if (!n.seen) continue;
    if (n.health === 'latched') out.push({ level: 'warn', text: n.alive ? `Node ${n.name} is latched out. Once the cause is gone, readmit it: it goes on probation and must agree for about a second.` : `Node ${n.name} is latched out and silent (it is not sending). Restore it first (power, or a restart on the Faults tab); it can only be readmitted once it sends data to be judged.`, op: n.alive ? 'reintegrate' : null, target: n.name, mode: 'single', label: `Reintegrate ${n.name}` });
    if (n.health === 'probation') out.push({ level: 'info', text: `Node ${n.name} is on probation (a shadow vote against the healthy nodes). It rejoins by itself if it agrees long enough.`, op: null });
    if (n.health === 'disabled') out.push({ level: 'crit', text: `Node ${n.name} is disabled for the run (too many strikes). Clearing it needs an ARM; it then needs a reintegrate.`, op: 'clear-disabled', target: n.name, mode: 'armed', label: `Clear disabled ${n.name}` });
    if (n.safe_requested) out.push({ level: 'crit', text: `Node ${n.name} has a Safe request up. Find the cause first (the Events tab has the console lines).`, op: null });
  }
  const latchedAlive = s.nodes.filter((n) => n.health === 'latched' && n.alive).map((n) => n.name);
  if (s.act && s.act.alive && s.act.excluded.some(Boolean) && !NODES.every((nm, i) => !s.act.excluded[i] || latchedAlive.includes(nm))) {     // when the same node is already offered above, one reintegrate does both
    const who = NODES.filter((_, i) => s.act.excluded[i]).join(', ');
    out.push({ level: 'warn', text: `ACT has excluded node ${who} from its vote and never readmits by itself (a node that was lost, or a peer that joined late). Once the cause is gone, reintegrate readmits what ACT excluded: the nodes must agree with the others again.`, op: 'reintegrate', target: NODES.find((_, i) => s.act.excluded[i]), mode: 'single', label: `Reintegrate ${who.split(',')[0]}` });
  }
  if (s.act && s.act.alive && s.act.state >= 2) out.push({ level: 'crit', text: `ACT is in ${s.act.state_name} (cause: ${s.act.cause}). Leaving Safe needs votes that have been trustworthy for a second, from two nodes, with no request or hardware line up.`, op: 'clear-safe', target: '', mode: 'armed', label: 'Clear Safe' });
  if (s.phase.name === 'countdown') out.push({ level: 'warn', text: 'A countdown is running. A scrub returns the vehicle to the pad (refused after T-zero).', op: 'scrub', target: '', mode: 'single', label: 'Scrub' });
  return out;
}

export const healthLevel = (h) => ({ healthy: 'ok', probation: 'warn', latched: 'warn', disabled: 'crit' }[h] || 'muted');
export const nodeNames = NODES;
