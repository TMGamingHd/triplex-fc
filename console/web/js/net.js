// The connection to the console server: the session token, JSON calls, and the event stream with its history.
const params = new URLSearchParams(location.search);
let token = params.get('token') || sessionStorage.getItem('tfc.token') || '';
if (params.get('token')) { sessionStorage.setItem('tfc.token', token); if (!params.get('selftest')) history.replaceState(null, '', location.pathname + location.hash); }   // the token leaves the address bar
export const hasToken = () => !!token;

export async function api(path, body) {
  const opt = body === undefined ? { headers: { 'X-TFC-Token': token } } : { method: 'POST', headers: { 'X-TFC-Token': token, 'Content-Type': 'application/json' }, body: JSON.stringify(body) };
  let r;
  try { r = await fetch(path, opt); } catch (e) { throw new Error('the console server is not reachable'); }
  let data = null;
  try { data = await r.json(); } catch { /* no body */ }
  if (!r.ok) { const e = new Error((data && data.error) || `${r.status} ${r.statusText}`); e.status = r.status; throw e; }
  return data;
}

/** Subscribes to /api/stream. Handlers: hello, state, event, line, reset, status('up'|'down'). EventSource reconnects by itself. */
export function stream(h) {
  let es;
  const open = () => {
    es = new EventSource(`/api/stream?token=${encodeURIComponent(token)}`);
    es.onopen = () => h.status && h.status('up');
    es.onerror = () => h.status && h.status('down');
    for (const k of ['hello', 'state', 'event', 'line', 'reset']) {
      es.addEventListener(k, (e) => { try { h[k] && h[k](JSON.parse(e.data)); } catch (err) { console.error(k, err); h.error && h.error(err); } });
    }
  };
  open();
  return { close: () => es && es.close() };
}

/** `?hold=ms` in the address: the page does not finish loading for that long (a headless browser takes its screenshot at the load event, and the state arrives after it). */
const hold = +(params.get('hold') || 0);
if (hold > 0) { const img = new Image(); img.src = `/api/hold?ms=${hold}&token=${encodeURIComponent(token)}`; img.style.display = 'none'; document.addEventListener('DOMContentLoaded', () => document.body.append(img)); }
