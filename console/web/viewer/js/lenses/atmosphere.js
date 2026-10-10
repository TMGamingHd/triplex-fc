// Atmosphere: the air the vehicle is climbing through. Its pressure, density, temperature and the speed of sound at this altitude, the wind, the layers, how far the air is from a continuum, and the whole profile with the
// vehicle on it. The numbers are the simulator's own atmosphere (the profile it sent with the vehicle, dispersions included); the layers are drawn in the sky as faint shells.
import { Chart, legend } from '../../../js/charts.js';
import { h, stat, statGrid, card, seg, toggle, provenance, fmt, store } from '../ui.js';
import { us1976 } from '../data.js';
import { layerOf, viscosity } from '../physics.js';

const LAYERS = [[11, 'tropopause', [0.35, 0.6, 1.0]], [20, '20 km', [0.3, 0.8, 0.9]], [32, '32 km', [0.4, 0.9, 0.6]], [47, 'stratopause', [0.7, 0.95, 0.4]], [51, '51 km', [0.95, 0.85, 0.3]], [71, '71 km', [1.0, 0.65, 0.3]], [86, 'mesopause', [1.0, 0.45, 0.3]], [100, 'Karman line', [1.0, 0.3, 0.4]]];
const BANDS = [[0, 11, 'troposphere', '#4a7bd0'], [11, 47, 'stratosphere', '#3aa890'], [47, 86, 'mesosphere', '#c79a3a'], [86, 120, 'thermosphere', '#c0505a']];
const QTY = {
  T: { label: 'Temperature', unit: 'K', i: 0, log: false, lo: 150, hi: 330 }, p: { label: 'Pressure', unit: 'Pa', i: 1, log: true, lo: 1e-3, hi: 2e5 }, rho: { label: 'Density', unit: 'kg/m³', i: 2, log: true, lo: 1e-9, hi: 2 },
  a: { label: 'Speed of sound', unit: 'm/s', i: 3, log: false, lo: 260, hi: 360 }, wind: { label: 'Mean wind', unit: 'm/s', i: 4, log: false, lo: 0, hi: 60 },
};

export default {
  id: 'atmosphere', label: 'Atmosphere', key: '3',
  mount(app) {
    const S = (this.s = { q: store.get('atm.q', 'p'), shells: store.get('atm.shells', true) }), { world, spec } = app;
    const k = (l, u, hint) => stat(l, u, hint);
    S.st = { p: k('Pressure', 'Pa'), rho: k('Density', 'kg/m³'), T: k('Temperature', 'K'), a: k('Speed of sound', 'm/s'), wind: k('Mean wind', 'm/s', 'The simulator\'s mean wind profile at this altitude, times the scenario\'s wind scale'), mu: k('Viscosity', 'µPa·s'),
      lam: k('Mean free path', 'm', 'λ = (μ / p) √(π R T / 2): how far a molecule goes between collisions'), kn: k('Knudsen number', '', 'λ / vehicle diameter: below 0.01 the air is a continuum, above 10 it is free molecules'), H: k('Scale height', 'km', 'R T / g: the height over which the pressure falls by e'),
      pr: k('p / sea level', '%'), dev: k('Density vs standard', '%', 'The simulator\'s density against the 1976 standard atmosphere at this altitude: a dispersion shows here'), dT: k('Temperature vs standard', 'K') };
    const s = S.st;
    S.layer = h('div', { class: 'v-regime' }); S.regime = h('div', { class: 'v-verdict' });
    const left = [card('The air here', [statGrid(2, s.p, s.rho, s.T, s.a, s.wind, s.mu), S.layer]), card('How thin is it?', [statGrid(2, s.lam, s.kn, s.H, s.pr), S.regime]),
      card('Against the standard atmosphere', statGrid(2, s.dev, s.dT))];
    app.left.replaceChildren(...left);
    S.canvas = h('canvas', { class: 'v-profile', width: 320, height: 420 });
    S.qSeg = seg(Object.entries(QTY).map(([id, q]) => [id, { T: 'Temp', p: 'Pressure', rho: 'Density', a: 'Sound', wind: 'Wind' }[id], q.label]), S.q, (v) => { S.q = v; store.set('atm.q', v); }, 'quantity');
    S.charts = { p: new Chart(h('canvas'), { height: 100, window: 120, series: [{ key: 'p_amb', label: 'ambient pressure Pa', color: 'var(--info)' }] }), rho: new Chart(h('canvas'), { height: 100, window: 120, series: [{ key: 'rho', label: 'density kg/m³', color: 'var(--nC)' }] }),
      alt: new Chart(h('canvas'), { height: 100, window: 120, series: [{ key: 'alt', label: 'altitude m', color: 'var(--accent)' }] }) };
    const ch = (t, c) => card(t, [c.c, legend(c.o.series)]);
    app.right.replaceChildren(card('Profile, with the vehicle on it', [h('div', { class: 'v-row' }, S.qSeg), S.canvas, toggle('Show the layers in the sky', S.shells, (v) => { S.shells = v; store.set('atm.shells', v); }, 'Faint shells at the layer boundaries, brightest where seen edge-on')]),
      ch('Ambient pressure', S.charts.p), ch('Density', S.charts.rho), ch('Altitude', S.charts.alt),
      card('Where the numbers come from', [provenance('measured', 'pressure, density, temperature and the speed of sound at the vehicle, and the mean wind: the simulator\'s atmosphere (US 1976 standard, with the scenario\'s dispersions) sampled every kilometre.'),
        provenance('derived', 'viscosity (Sutherland), mean free path, Knudsen number and scale height: textbook relations on those.'), provenance('illustrative', 'the shells in the sky, the colours of the bands.')]));
    world.camera && (S.camSet = false);
  },
  update(app, pose, d, dt) {
    const S = this.s, s = S.st, { spec, world } = app, air = pose.air || us1976(pose.alt), [T, p, rho, a] = air;
    const wind = spec.airAt(pose.alt)[4], std = us1976(pose.alt);
    const mu = viscosity(T), lam = mu / p * Math.sqrt(Math.PI * 287.053 * T / 2), kn = lam / spec.diameter, H = 287.053 * T / 9.80665;
    s.p.set(p >= 100 ? p.toFixed(0) : p >= 1 ? p.toFixed(1) : p.toExponential(1)); s.rho.set(rho >= 0.01 ? rho.toFixed(4) : rho.toExponential(2)); s.T.set(T.toFixed(1)); s.a.set(a.toFixed(0)); s.wind.set(wind.toFixed(1)); s.mu.set((mu * 1e6).toFixed(2));
    s.lam.set(lam < 0.01 ? lam.toExponential(1) : lam.toFixed(lam < 1 ? 3 : 1)); s.kn.set(kn < 0.001 ? kn.toExponential(1) : kn.toFixed(kn < 1 ? 3 : 1)); s.H.set((H / 1000).toFixed(1)); s.pr.set((100 * p / 101325).toFixed(p / 101325 < 0.01 ? 4 : 2));
    const drho = 100 * (rho / std[2] - 1), dT = T - std[0];
    s.dev.set((drho >= 0 ? '+' : '') + drho.toFixed(1), Math.abs(drho) > 5 ? 'warn' : ''); s.dT.set((dT >= 0 ? '+' : '') + dT.toFixed(1), Math.abs(dT) > 5 ? 'warn' : '');
    const L = layerOf(pose.alt);
    S.layer.textContent = `${L.name}, ${L.lo} to ${L.hi} km; the vehicle is ${((pose.alt / 1000 - L.lo)).toFixed(1)} km into it.`;
    S.regime.className = 'v-verdict' + (kn > 0.1 ? ' warn' : ' ok');
    S.regime.textContent = kn < 0.01 ? 'Continuum: the air acts as a fluid, and the aerodynamic forces are the ones the aero lens shows.' : kn < 0.1 ? 'Slip flow: the continuum is starting to fail at the skin.' : kn < 10 ? 'Transitional: molecules and the surface hardly collide with each other; the continuum aerodynamics no longer hold.' : 'Free molecular flow: the air is individual molecules; there is no aerodynamics to speak of.';
    // the layers in the sky
    const u = world.sky.u, Rp = u.uRp.value;
    LAYERS.forEach(([km, , col], i) => { u.uShell.value[i].set(col[0] * 1.6, col[1] * 1.6, col[2] * 1.6, Rp + km); });
    u.uShellAlpha.value = S.shells ? 0.55 : 0;
    this.drawProfile(app, pose);
    for (const c of Object.values(S.charts)) c.draw(app.data.hist);
  },
  drawProfile(app, pose) {
    const S = this.s, c = S.canvas, spec = app.spec, q = QTY[S.q], ctx = c.getContext('2d');
    const dpr = window.devicePixelRatio || 1, W = c.clientWidth || 320, H = 420;
    if (c.width !== Math.round(W * dpr)) { c.width = Math.round(W * dpr); c.height = Math.round(H * dpr); }
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0); ctx.clearRect(0, 0, W, H);
    const css = (v) => getComputedStyle(document.documentElement).getPropertyValue(v).trim();
    const L = 48, R = 12, T = 10, B = 28, pw = W - L - R, ph = H - T - B, top = 120;
    const Y = (km) => T + ph * (1 - Math.min(km, top) / top);
    const val = (alt) => { const v = spec.airAt(alt * 1000); return v[q.i]; };
    const fx = (v) => { const t = q.log ? (Math.log10(Math.max(v, q.lo)) - Math.log10(q.lo)) / (Math.log10(q.hi) - Math.log10(q.lo)) : (v - q.lo) / (q.hi - q.lo); return L + pw * Math.min(1, Math.max(0, t)); };
    // the layer bands
    for (const [lo, hi, name, col] of BANDS) { ctx.fillStyle = col + '22'; ctx.fillRect(L, Y(hi), pw, Y(lo) - Y(hi)); ctx.fillStyle = col; ctx.font = '600 10px system-ui'; ctx.fillText(name, L + 4, Y(hi) + 11); }
    ctx.strokeStyle = css('--line'); ctx.lineWidth = 1; ctx.fillStyle = css('--muted'); ctx.font = '10px system-ui'; ctx.textAlign = 'right';
    for (const km of [0, 20, 40, 60, 80, 100, 120]) { ctx.beginPath(); ctx.moveTo(L, Y(km)); ctx.lineTo(L + pw, Y(km)); ctx.stroke(); ctx.fillText(km + ' km', L - 5, Y(km) + 3); }
    ctx.textAlign = 'center';
    const ticks = q.log ? [] : null;
    if (q.log) { for (let e = Math.ceil(Math.log10(q.lo)); e <= Math.floor(Math.log10(q.hi)); e += (q.hi / q.lo > 1e8 ? 2 : 1)) { const x = fx(Math.pow(10, e)); ctx.beginPath(); ctx.moveTo(x, T); ctx.lineTo(x, T + ph); ctx.stroke(); ctx.fillText('1e' + e, x, H - 14); } }
    else { for (let i = 0; i <= 4; i++) { const v = q.lo + (q.hi - q.lo) * i / 4, x = fx(v); ctx.beginPath(); ctx.moveTo(x, T); ctx.lineTo(x, T + ph); ctx.stroke(); ctx.fillText(v.toFixed(0), x, H - 14); } }
    ctx.fillText(`${q.label} (${q.unit})`, L + pw / 2, H - 2);
    // the standard atmosphere, dashed, and the simulator's, solid
    ctx.setLineDash([4, 3]); ctx.strokeStyle = css('--faint'); ctx.beginPath();
    for (let km = 0; km <= top; km += 1) { const std = us1976(km * 1000), v = q.i < 4 ? std[q.i] : null; if (v == null) break; const x = fx(v), y = Y(km); km === 0 ? ctx.moveTo(x, y) : ctx.lineTo(x, y); } ctx.stroke(); ctx.setLineDash([]);
    ctx.strokeStyle = css('--accent'); ctx.lineWidth = 2; ctx.beginPath();
    for (let km = 0; km <= top; km += 1) { const x = fx(val(km)), y = Y(km); km === 0 ? ctx.moveTo(x, y) : ctx.lineTo(x, y); } ctx.stroke();
    // the vehicle
    const alt = pose.alt / 1000, vy = Y(alt), vx = fx(val(alt));
    ctx.strokeStyle = css('--warn'); ctx.lineWidth = 1.2; ctx.setLineDash([2, 3]); ctx.beginPath(); ctx.moveTo(L, vy); ctx.lineTo(L + pw, vy); ctx.stroke(); ctx.setLineDash([]);
    ctx.fillStyle = css('--warn'); ctx.beginPath(); ctx.arc(vx, vy, 5, 0, 7); ctx.fill();
    ctx.textAlign = 'left'; ctx.font = '600 11px system-ui'; ctx.fillText(`${alt.toFixed(1)} km`, Math.min(vx + 9, L + pw - 56), Math.max(vy - 5, T + 24));
    ctx.textAlign = 'left';
  },
  unmount(app) { app.world.sky.u.uShellAlpha.value = 0; this.s = null; },
};
