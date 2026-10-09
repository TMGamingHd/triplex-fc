// Canvas charts: a time-series chart over the shared history, and the trajectory plot. No library; colours come from the page's CSS tokens so both themes work.
const cssCache = new Map();
export function cssColor(el, v) {
  if (!v) return '#888';
  const m = /^var\((--[\w-]+)\)$/.exec(v);
  if (!m) return v;
  const key = m[1] + (document.documentElement.dataset.theme || 'dark');
  if (!cssCache.has(key)) cssCache.set(key, getComputedStyle(document.documentElement).getPropertyValue(m[1]).trim() || '#888');
  return cssCache.get(key);
}
export const resetColorCache = () => cssCache.clear();

function niceStep(span, n) {
  const raw = span / Math.max(1, n);
  const p = Math.pow(10, Math.floor(Math.log10(raw)));
  const f = raw / p;
  return (f < 1.5 ? 1 : f < 3 ? 2 : f < 7 ? 5 : 10) * p;
}
function fmtTick(v, step) {
  const a = Math.abs(v);
  if (a >= 1e6) return (v / 1e6).toFixed(1) + 'M';
  if (a >= 1e4) return (v / 1e3).toFixed(a >= 1e5 ? 0 : 1) + 'k';
  const d = step >= 1 ? 0 : step >= 0.1 ? 1 : step >= 0.01 ? 2 : 3;
  return v.toFixed(d);
}
function lowerBound(arr, x) { let lo = 0, hi = arr.length; while (lo < hi) { const m = (lo + hi) >> 1; if (arr[m] < x) lo = m + 1; else hi = m; } return lo; }

export class Chart {
  /** o: {series:[{key,label,color,width,dash}], window:s, yMin, yMax, hlines:[{y,color,label,dash}], height:px, unit, zero:bool, markers:fn(hist)->[{t,label,color}]} */
  constructor(canvas, o) {
    this.c = canvas; this.o = Object.assign({ window: 60, height: 170, hlines: [], series: [] }, o);
    this.hover = null;
    canvas.classList.add('chart');
    canvas.style.height = this.o.height + 'px';
    canvas.addEventListener('mousemove', (e) => { const r = canvas.getBoundingClientRect(); this.hover = e.clientX - r.left; if (this.last) this.draw(this.last); });
    canvas.addEventListener('mouseleave', () => { this.hover = null; if (this.last) this.draw(this.last); });
  }
  setWindow(w) { this.o.window = w; if (this.last) this.draw(this.last); }
  draw(hist) {
    this.last = hist;
    const c = this.c, o = this.o;
    const dpr = window.devicePixelRatio || 1;
    const W = c.clientWidth, H = o.height;
    if (!W) return;
    if (c.width !== Math.round(W * dpr) || c.height !== Math.round(H * dpr)) { c.width = Math.round(W * dpr); c.height = Math.round(H * dpr); }
    const g = c.getContext('2d');
    g.setTransform(dpr, 0, 0, dpr, 0, 0);
    g.clearRect(0, 0, W, H);
    const L = 46, R = 8, T = 8, B = 20;
    const pw = W - L - R, ph = H - T - B;
    const col = (v) => cssColor(c, v);
    const text = col('var(--muted)'), line = col('var(--line)');
    const tEnd = hist.t.length ? hist.t[hist.t.length - 1] : 0;
    const t0 = tEnd - o.window;
    const i0 = Math.max(0, lowerBound(hist.t, t0) - 1);
    // y range
    let lo = Infinity, hi = -Infinity;
    for (const s of o.series) {
      const a = hist.s[s.key]; if (!a) continue;
      for (let i = i0; i < a.length; i++) { const v = a[i]; if (v == null) continue; if (v < lo) lo = v; if (v > hi) hi = v; }
    }
    for (const hl of o.hlines) { if (hl.include) { lo = Math.min(lo, hl.y); hi = Math.max(hi, hl.y); } }
    if (o.zero) { lo = Math.min(lo, 0); hi = Math.max(hi, 0); }
    if (!isFinite(lo)) { lo = 0; hi = 1; }
    if (o.yMin != null) lo = o.yMin;
    if (o.yMax != null) hi = o.yMax;
    if (hi - lo < 1e-9) { hi += 0.5; lo -= 0.5; }
    if (o.yMin == null) lo -= (hi - lo) * 0.06;
    if (o.yMax == null) hi += (hi - lo) * 0.06;
    const step = niceStep(hi - lo, Math.max(2, Math.floor(ph / 38)));
    const X = (t) => L + ((t - t0) / o.window) * pw;
    const Y = (v) => T + (1 - (v - lo) / (hi - lo)) * ph;
    g.font = '10px ' + getComputedStyle(c).getPropertyValue('--mono');
    g.textAlign = 'right'; g.textBaseline = 'middle';
    for (let v = Math.ceil(lo / step) * step; v <= hi + 1e-9; v += step) {
      const y = Y(v);
      g.strokeStyle = line; g.lineWidth = 1; g.beginPath(); g.moveTo(L, y + .5); g.lineTo(L + pw, y + .5); g.stroke();
      g.fillStyle = text; g.fillText(fmtTick(v, step), L - 5, y);
    }
    g.textAlign = 'center'; g.textBaseline = 'top'; g.fillStyle = text;
    for (const f of [0, 0.5, 1]) g.fillText(f === 1 ? 'now' : `-${Math.round(o.window * (1 - f))} s`, L + f * pw, H - B + 5);
    // horizontal reference lines
    for (const hl of o.hlines) {
      if (hl.y < lo || hl.y > hi) continue;
      g.strokeStyle = col(hl.color || 'var(--warn)'); g.lineWidth = 1; g.setLineDash(hl.dash || [5, 4]);
      g.beginPath(); g.moveTo(L, Y(hl.y) + .5); g.lineTo(L + pw, Y(hl.y) + .5); g.stroke(); g.setLineDash([]);
      if (hl.label) { g.fillStyle = col(hl.color || 'var(--warn)'); g.textAlign = 'right'; g.textBaseline = 'bottom'; g.fillText(hl.label, L + pw - 3, Y(hl.y) - 2); }
    }
    // markers (events)
    for (const m of (o.markers ? o.markers(hist) : [])) {
      if (m.t < t0 || m.t > tEnd) continue;
      g.strokeStyle = col(m.color || 'var(--info)'); g.setLineDash([2, 3]); g.beginPath(); g.moveTo(X(m.t) + .5, T); g.lineTo(X(m.t) + .5, T + ph); g.stroke(); g.setLineDash([]);
      g.fillStyle = col(m.color || 'var(--info)'); g.textAlign = 'left'; g.textBaseline = 'top'; g.fillText(m.label, X(m.t) + 3, T + 2);
    }
    // series
    g.save(); g.beginPath(); g.rect(L, T, pw, ph); g.clip();
    for (const s of o.series) {
      const a = hist.s[s.key]; if (!a) continue;
      g.strokeStyle = col(s.color); g.lineWidth = s.width || 1.6; g.setLineDash(s.dash || []); g.lineJoin = 'round';
      g.beginPath(); let pen = false;
      for (let i = i0; i < a.length; i++) {
        const v = a[i];
        if (v == null) { pen = false; continue; }
        const x = X(hist.t[i]), y = Y(v);
        if (!pen) { g.moveTo(x, y); pen = true; } else g.lineTo(x, y);
      }
      g.stroke(); g.setLineDash([]);
    }
    g.restore();
    // hover readout
    if (this.hover != null && this.hover >= L && this.hover <= L + pw) {
      const t = t0 + ((this.hover - L) / pw) * o.window;
      const i = Math.min(hist.t.length - 1, Math.max(0, lowerBound(hist.t, t)));
      g.strokeStyle = col('var(--faint)'); g.beginPath(); g.moveTo(this.hover + .5, T); g.lineTo(this.hover + .5, T + ph); g.stroke();
      const rows = o.series.map((s) => ({ s, v: (hist.s[s.key] || [])[i] })).filter((r) => r.v != null);
      if (rows.length) {
        g.textAlign = 'left'; g.textBaseline = 'top';
        const bw = 128, bh = 14 * rows.length + 6;
        const bx = this.hover + 10 + bw > W ? this.hover - bw - 10 : this.hover + 10;
        g.fillStyle = col('var(--panel3)'); g.globalAlpha = .94; g.fillRect(bx, T + 4, bw, bh); g.globalAlpha = 1;
        rows.forEach((r, k) => { g.fillStyle = col(r.s.color); g.fillRect(bx + 5, T + 10 + 14 * k, 8, 3); g.fillStyle = col('var(--text)'); g.fillText(`${r.s.label || r.s.key} ${fmtTick(r.v, Math.abs(r.v) > 100 ? 1 : 0.01)}`, bx + 18, T + 7 + 14 * k); });
        g.fillStyle = text; g.fillText(`${(t - tEnd).toFixed(1)} s`, bx + 5, T + bh + 6);
      }
    }
  }
}

/** Series legend as HTML (so it wraps and is selectable text). */
export function legend(series) {
  const el = document.createElement('div'); el.className = 'legend';
  for (const s of series) {
    const i = document.createElement('i'); i.style.background = `var(${(/var\((--[\w-]+)\)/.exec(s.color) || [])[1] || '--muted'})`;
    if (s.dash) i.style.opacity = .6;
    const span = document.createElement('span'); span.append(i, s.label || s.key); el.append(span);
  }
  return el;
}

/** Altitude against range: the nominal flight, the flown trail, event marks and the vehicle. */
export class Trajectory {
  constructor(canvas, height = 300) { this.c = canvas; this.h = height; canvas.classList.add('chart'); canvas.style.height = height + 'px'; }
  draw({ nominal, trail, marks, now }) {
    const c = this.c, dpr = window.devicePixelRatio || 1, W = c.clientWidth, H = this.h;
    if (!W) return;
    if (c.width !== Math.round(W * dpr) || c.height !== Math.round(H * dpr)) { c.width = Math.round(W * dpr); c.height = Math.round(H * dpr); }
    const g = c.getContext('2d'); g.setTransform(dpr, 0, 0, dpr, 0, 0); g.clearRect(0, 0, W, H);
    const col = (v) => cssColor(c, v);
    const L = 52, R = 14, T = 12, B = 26, pw = W - L - R, ph = H - T - B;
    let xmax = 1000, ymax = 1000;
    for (const s of [nominal, trail]) if (s) { for (let i = 0; i < s.range.length; i++) { xmax = Math.max(xmax, s.range[i]); ymax = Math.max(ymax, s.alt[i]); } }
    const xs = niceStep(xmax * 1.08, 5), ys = niceStep(ymax * 1.12, 5);
    const XM = Math.ceil((xmax * 1.04) / xs) * xs, YM = Math.ceil((ymax * 1.08) / ys) * ys;
    const X = (v) => L + (v / XM) * pw, Y = (v) => T + (1 - v / YM) * ph;
    g.font = '10px ' + getComputedStyle(c).getPropertyValue('--mono');
    g.textBaseline = 'middle'; g.textAlign = 'right';
    for (let v = 0; v <= YM + 1; v += ys) { g.strokeStyle = col('var(--line)'); g.beginPath(); g.moveTo(L, Y(v) + .5); g.lineTo(L + pw, Y(v) + .5); g.stroke(); g.fillStyle = col('var(--muted)'); g.fillText((v / 1000).toFixed(YM > 20000 ? 0 : 1), L - 5, Y(v)); }
    g.textAlign = 'center'; g.textBaseline = 'top';
    for (let v = 0; v <= XM + 1; v += xs) { g.strokeStyle = col('var(--line)'); g.beginPath(); g.moveTo(X(v) + .5, T); g.lineTo(X(v) + .5, T + ph); g.stroke(); g.fillStyle = col('var(--muted)'); g.fillText((v / 1000).toFixed(XM > 20000 ? 0 : 1), X(v), T + ph + 5); }
    g.fillStyle = col('var(--muted)'); g.textAlign = 'left'; g.fillText('altitude, km', 4, 0 + 1); g.textAlign = 'right'; g.fillText('range, km', W - 4, H - 12);
    const path = (s, color, w, dash) => { if (!s || !s.range.length) return; g.strokeStyle = col(color); g.lineWidth = w; g.setLineDash(dash || []); g.beginPath(); s.range.forEach((r, i) => (i ? g.lineTo(X(r), Y(s.alt[i])) : g.moveTo(X(r), Y(s.alt[i])))); g.stroke(); g.setLineDash([]); };
    path(nominal, 'var(--faint)', 1.6, [6, 4]);
    path(trail, 'var(--accent)', 2.4);
    for (const m of marks || []) { g.fillStyle = col(m.color || 'var(--info)'); g.beginPath(); g.arc(X(m.range), Y(m.alt), 4, 0, 7); g.fill(); g.fillStyle = col('var(--text)'); g.textAlign = 'left'; g.textBaseline = 'middle'; g.fillText(m.label, X(m.range) + 7, Y(m.alt) - 1); }
    if (now) {   // the vehicle: a triangle along its flight path
      const x = X(now.range), y = Y(now.alt);
      g.save(); g.translate(x, y); g.rotate(now.angle || 0); g.fillStyle = col('var(--ok)'); g.beginPath(); g.moveTo(0, -9); g.lineTo(5, 6); g.lineTo(-5, 6); g.closePath(); g.fill(); g.restore();
    }
  }
}
