// The parts of the CFD view that are not the flow itself: the map of the tables the flight flies on, with the flight's own path across it, and the colour legend of the section.
//
// The map is the answer to "does the flight use the CFD?": it draws one coefficient of the solver-made tables (`aero.cfd` in the vehicle file, tools/cfd/make_aerodb.py) over the Mach number and the angle of attack, the same
// bilinear interpolation the simulator's plant does (sim/vehicle/cfd_aero.hpp), and the flight's path across it from the history the page keeps. Where the map is empty (below the table's lowest Mach number, with the build-up
// fading in) the plant flies on the engineering build-up, and the map says so.
import { h } from './ui.js';

const FIELDS = [['cn', 'C_N', 'Normal force coefficient'], ['ca', 'C_A', 'Axial force coefficient (the pressure on the skin; the plant adds the friction and the base)'], ['cm', 'C_m', 'Pitching moment coefficient about the reference point']];

/** The table that belongs to the stages on the vehicle: the one that names exactly them (the stack's, the ship's, the booster's). */
export function tableFor(tables, mask) {
  const want = [0, 1, 2, 3, 4, 5, 6, 7].filter((i) => (mask >> i) & 1).join(',');
  return (tables || []).find((t) => (t.stages || []).slice().sort((a, b) => a - b).join(',') === want) || null;
}

/** The value of a table's field at (Mach, |alpha| in degrees): bilinear between the nodes, held at the edge of the table. */
export function sampleTable(t, key, mach, alpha) {
  const M = t.mach, A = t.alpha_deg, f = t[key]; if (!f) return NaN;
  const idx = (arr, x) => { if (x <= arr[0]) return [0, 0, 0]; const n = arr.length; if (x >= arr[n - 1]) return [n - 1, n - 1, 0]; let i = 0; while (arr[i + 1] < x) i++; return [i, i + 1, (x - arr[i]) / (arr[i + 1] - arr[i])]; };
  const [m0, m1, fm] = idx(M, mach), [a0, a1, fa] = idx(A, Math.abs(alpha));
  const v = (i, j) => f[i][j];
  return (v(m0, a0) * (1 - fa) + v(m0, a1) * fa) * (1 - fm) + (v(m1, a0) * (1 - fa) + v(m1, a1) * fa) * fm;
}

// a colour for a value from 0 to 1: dark blue, teal, yellow, white (it reads on a dark panel and in a screenshot)
const STOPS = [[0, [14, 22, 58]], [0.3, [38, 86, 150]], [0.55, [34, 160, 160]], [0.8, [236, 200, 70]], [1, [255, 245, 215]]];
const ramp = (x) => { x = Math.max(0, Math.min(1, x)); let i = 0; while (i < STOPS.length - 2 && STOPS[i + 1][0] < x) i++; const [a, ca] = STOPS[i], [b, cb] = STOPS[i + 1], f = (x - a) / (b - a); return ca.map((c, k) => Math.round(c + (cb[k] - c) * f)); };

/** The map as a canvas: the Mach number across (logarithmic), the angle of attack up, one coefficient in colour, the flight's path and its place now on it. */
export class AeroMap {
  constructor() {
    this.canvas = h('canvas', { class: 'v-aeromap', width: 300, height: 220, role: 'img', 'aria-label': 'The aerodynamic table and the flight across it' });
    this.field = 'cn'; this.cache = null; this.stamp = 0;
    this.readout = h('div', { class: 'v-mapread' }, '');
    this.seg = h('div', { class: 'seg', role: 'group', 'aria-label': 'coefficient' }, FIELDS.map(([k, t, hint]) => h('button', { type: 'button', title: hint, dataset: { v: k }, 'aria-pressed': String(k === this.field), onclick: () => { this.field = k; this.cache = null; for (const b of this.seg.children) b.setAttribute('aria-pressed', String(b.dataset.v === k)); } }, t)));
    this.el = h('div', { class: 'v-map' }, this.seg, this.canvas, this.readout);
  }

  /** `table`: the table for the stages on the vehicle, or null; `d`: the derived flight state; `hist`: the page's history (mach and alpha by time). */
  draw(table, d, hist, now) {
    if (now - this.stamp < 90) return; this.stamp = now;
    const c = this.canvas, dpr = Math.min(2, window.devicePixelRatio || 1), W = c.clientWidth || 300, H = 220;
    if (c.width !== Math.round(W * dpr)) { c.width = Math.round(W * dpr); c.height = Math.round(H * dpr); this.cache = null; }
    const g = c.getContext('2d'); g.setTransform(dpr, 0, 0, dpr, 0, 0);
    const L = 34, R = 8, T = 8, B = 24, pw = W - L - R, ph = H - T - B, M0 = 0.8, M1 = 24, lg = Math.log;
    const X = (m) => L + pw * (lg(Math.max(m, M0)) - lg(M0)) / (lg(M1) - lg(M0)), Yp = (a) => T + ph * (1 - Math.max(0, Math.min(180, a)) / 180);
    const css = (v) => getComputedStyle(document.documentElement).getPropertyValue(v).trim();
    g.clearRect(0, 0, W, H);
    if (!table) { g.fillStyle = css('--muted'); g.font = '12px system-ui'; g.fillText('No solver table for the stages on the vehicle.', L, T + 24); return; }
    const key = table[this.field] ? this.field : 'cn';
    if (!this.cache || this.cache.table !== table || this.cache.key !== key || this.cache.W !== W) {
      // the field, pixel by pixel, scaled to its own range
      const w = Math.round(pw * dpr), hh = Math.round(ph * dpr), img = new ImageData(w, hh), vals = new Float32Array(w * hh);
      let lo = Infinity, hi = -Infinity;
      for (let j = 0; j < hh; j++) for (let i = 0; i < w; i++) {
        const m = Math.exp(lg(M0) + (i + 0.5) / w * (lg(M1) - lg(M0))), a = 180 * (1 - (j + 0.5) / hh), v = sampleTable(table, key, m, a);
        vals[j * w + i] = v; if (m >= table.mach[0] && m <= table.mach[table.mach.length - 1]) { lo = Math.min(lo, v); hi = Math.max(hi, v); }
      }
      for (let n = 0; n < vals.length; n++) {
        const i = n % w, m = Math.exp(lg(M0) + (i + 0.5) / w * (lg(M1) - lg(M0))), [r, gg, b] = ramp((vals[n] - lo) / Math.max(1e-9, hi - lo)), inT = m >= table.mach[0] && m <= table.mach[table.mach.length - 1];
        img.data.set([r, gg, b, inT ? 255 : 0], 4 * n);
      }
      const off = document.createElement('canvas'); off.width = w; off.height = hh; off.getContext('2d').putImageData(img, 0, 0);
      this.cache = { table, key, W, off, lo, hi };
    }
    const ca = this.cache;
    // below the table: the engineering build-up flies, and the build-up fades into the table over the Mach numbers just below it
    g.fillStyle = css('--panel2'); g.fillRect(L, T, pw, ph);
    g.save(); g.beginPath(); g.rect(L, T, X(table.mach[0]) - L, ph); g.clip(); g.strokeStyle = css('--line2'); g.lineWidth = 1;
    for (let k = -ph; k < pw; k += 9) { g.beginPath(); g.moveTo(L + k, T + ph); g.lineTo(L + k + ph, T); g.stroke(); }
    g.restore();
    g.drawImage(ca.off, L, T, pw, ph);
    // the nodes the solver was run at
    g.strokeStyle = 'rgba(255,255,255,.22)'; g.fillStyle = 'rgba(255,255,255,.5)';
    for (const m of table.mach) for (const a of table.alpha_deg) { g.beginPath(); g.arc(X(m), Yp(a), 1.4, 0, 7); g.fill(); }
    // the axes
    g.fillStyle = css('--muted'); g.font = '10px system-ui'; g.textAlign = 'center';
    for (const m of [1, 2, 3, 5, 8, 12, 20]) { g.fillText(String(m), X(m), H - 10); g.fillRect(X(m), T + ph, 1, 3); }
    g.fillText('Mach', L + pw / 2, H - 0); g.textAlign = 'right';
    for (const a of [0, 45, 90, 135, 180]) g.fillText(a + '°', L - 5, Yp(a) + 3);
    g.textAlign = 'left'; g.fillStyle = css('--faint'); g.fillText('build-up', L + 3, T + 12);
    // the path, from the page's history: older is fainter
    const ms = hist.s.mach || [], as = hist.s.alpha || [], n = Math.min(ms.length, as.length), from = Math.max(0, n - 900);
    g.lineWidth = 1.6; g.lineJoin = 'round'; let prev = null;
    for (let i = from; i < n; i++) {
      if (ms[i] == null || as[i] == null) { prev = null; continue; }
      const p = [X(ms[i]), Yp(as[i])]; if (prev) { g.strokeStyle = `rgba(255,255,255,${(0.15 + 0.85 * (i - from) / Math.max(1, n - from)).toFixed(2)})`; g.beginPath(); g.moveTo(prev[0], prev[1]); g.lineTo(p[0], p[1]); g.stroke(); } prev = p;
    }
    // where the flight is now
    const px = X(d.M), py = Yp(d.alpha);
    g.strokeStyle = '#fff'; g.fillStyle = css('--bg'); g.lineWidth = 2; g.beginPath(); g.arc(px, py, 5, 0, 7); g.fill(); g.stroke();
    const inT = d.M >= table.mach[0], val = sampleTable(table, key, d.M, d.alpha);
    this.readout.textContent = inT ? `Now: Mach ${d.M.toFixed(2)}, ${d.alpha.toFixed(1)}°  →  ${FIELDS.find((f) => f[0] === key)[1]} = ${val.toFixed(3)} from the table` : `Now: Mach ${d.M.toFixed(2)}, ${d.alpha.toFixed(1)}°  →  below the table (Mach ${table.mach[0]}): the build-up flies`;
  }
}

/** The legend of the section: the colour bar with the numbers under it. `kind`: 'mach' (0 to `hi`), 'pressure' (a tenth to a hundred times the free stream's), 'schlieren'. */
export class FlowLegend {
  constructor(colormap) {
    this.colormap = colormap;
    this.badge = h('div', { class: 'v-lg-b' }, ''); this.title = h('div', { class: 'v-lg-t' }, ''); this.bar = h('canvas', { width: 280, height: 10 }); this.ticks = h('div', { class: 'v-lg-ticks' }); this.sub = h('div', { class: 'v-lg-s' }, '');
    this.el = h('div', { id: 'v-cfd-legend', hidden: true }, this.badge, this.title, this.bar, this.ticks, this.sub); this.key = '';
  }
  show(on) { this.el.hidden = !on; }
  set(kind, hi, mach0) {
    const key = kind + ':' + (kind === 'mach' ? hi.toFixed(1) : '');
    if (key === this.key) return; this.key = key;
    const g = this.bar.getContext('2d'), w = this.bar.width;
    for (let x = 0; x < w; x++) {
      let r, gg, b; if (kind === 'schlieren') { const v = Math.round(255 * (x / (w - 1))); r = gg = b = v; } else [r, gg, b] = this.colormap(x / (w - 1)).map((c) => Math.round(c * 255));
      g.fillStyle = `rgb(${r},${gg},${b})`; g.fillRect(x, 0, 1, 10);
    }
    let t, labels, sub;
    if (kind === 'mach') { t = 'Mach number'; const n = 5; labels = Array.from({ length: n + 1 }, (_, i) => (hi * i / n).toFixed(hi < 6 ? 1 : 0)); sub = 'blue: slow, behind the shock · red: fast'; }
    else if (kind === 'pressure') { t = 'Pressure over the free stream\'s'; labels = ['0.1×', '1×', '10×', '100×']; sub = 'the nose\'s shock and the shoulder\'s expansion'; }
    else { t = 'Density gradient (schlieren)'; labels = ['expansion', '', 'shock']; sub = 'bright: compression · dark: expansion'; }
    this.title.textContent = t; this.sub.textContent = sub;
    this.ticks.replaceChildren(...labels.map((l) => h('span', {}, l)));
  }
}
