// Sending an operator command from the page, and following what became of it.
import { api } from './net.js';
import { S } from './state.js';
import { toast } from './util.js';

const watching = new Set();

/** body: {op, target, mode, confirmed, force}. Resolves with the server's record, or null after showing the reason it was refused. */
export async function sendCommand(body) {
  let rec;
  try { rec = await api('/api/command', body); }
  catch (e) { toast(e.message, e.status === 409 ? 'warn' : 'crit', 9000); return null; }
  const what = `${rec.mode === 'armed' ? 'ARM + ' : ''}${rec.op}${rec.target ? ' ' + rec.target : ''}`;
  toast(`Sent: ${what} (counter ${rec.frames.map((f) => f.counter).join(', ')}). Waiting for the nodes' answers…`, 'info', 3500);
  follow(rec.id, what);
  return rec;
}

/** Tells the operator the outcome once the nodes have answered (or have not). */
function follow(id, what) {
  if (watching.has(id)) return;
  watching.add(id);
  const t0 = performance.now();
  const tick = () => {
    const rec = (S.snap && S.snap.commands && S.snap.commands.log || []).find((r) => r.id === id);
    if (rec && rec.status !== 'sent' && rec.status !== 'answered') {
      watching.delete(id);
      const by = rec.responses.map((r) => `${r.src}: ${r.result}`).join(' · ');
      toast(rec.status === 'accepted' ? `${what}: accepted. ${by}` : rec.status === 'refused' ? `${what}: REFUSED. ${by}` : `${what}: no answer from any node's console. Attach the consoles (the Rig tab starts them) to see whether it was accepted; the heartbeats show its effect.`,
        rec.status === 'accepted' ? 'ok' : rec.status === 'refused' ? 'crit' : 'warn', 9000);
      return;
    }
    if (performance.now() - t0 > 12000) { watching.delete(id); return; }
    setTimeout(tick, 400);
  };
  setTimeout(tick, 400);
}

export const keyNote = 'Commands are signed with the ground key. The default is the PUBLIC bench key from the repository: set TFC_GROUND_KEY (32 hex digits) in the console’s and the flight computers’ environment before any real use.';
