// The page's one copy of what the server has said: the latest snapshot, the chart history, the event log, the console lines.
export const S = {
  snap: null, cfg: null, vehicles: [], conn: 'down', msgs: 0, lastMsgAt: 0,
  hist: { t: [], s: {} }, events: [], lines: {}, eventSeq: 0,
};
const KEEP_S = 900;                     // 15 minutes of history in the page

export function setHistory(cols) {
  S.hist = { t: cols.t.slice(), s: Object.fromEntries(Object.entries(cols.series).map(([k, v]) => [k, v.slice()])) };
}
export function pushMetrics(t, m) {
  const H = S.hist;
  if (H.t.length && t <= H.t[H.t.length - 1]) return;          // an old or repeated sample (the paused replay republishes its state)
  const n = H.t.length;
  H.t.push(t);
  for (const k of new Set([...Object.keys(H.s), ...Object.keys(m || {})])) {
    if (!H.s[k]) H.s[k] = new Array(n).fill(null);
    H.s[k].push(m && m[k] != null ? m[k] : null);
  }
  const cut = t - KEEP_S;
  if (H.t[0] < cut) {
    let i = 0; while (i < H.t.length && H.t[i] < cut) i++;
    H.t.splice(0, i); for (const k of Object.keys(H.s)) H.s[k].splice(0, i);
  }
}
export function addEvent(e) { S.events.push(e); if (S.events.length > 3000) S.events.splice(0, 500); }
export function addLine(l) { const a = (S.lines[l.src] ||= []); a.push(l); if (a.length > 1500) a.splice(0, 300); }
