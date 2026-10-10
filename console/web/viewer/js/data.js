// The viewer's data: the spec and the poses from the console's pose stream, the interpolation between them, a clock that plays them smoothly, and the fallback that makes a pose out of the console's 10 Hz
// telemetry when the simulator is too old to send poses. Nothing here draws.
//
// Frames. The simulator's inertial frame has +X up at the launch point, +Y downrange, +Z crossrange (right-handed). The scene's has +Y up, +X right, +Z toward the viewer. The map between them is the cyclic
// permutation  scene = (sim.Z, sim.X, sim.Y)  (determinant +1, so no mirror image), and the same map takes the vehicle's body axes (x along the axis, y, z lateral) to the model's (Y up the axis, Z, X).
import { api, hasToken } from '../../js/net.js';

export const toScene = (v) => [v[2], v[0], v[1]];                 // a vector of the simulator's frames in the scene's axes
export const quatToScene = (q) => [q[3], q[1], q[2], q[0]];       // (w, x, y, z) of the simulator -> (x, y, z, w) of the scene, the same permutation on the vector part
const RAD = Math.PI / 180;

/** The US 1976 standard atmosphere up to 86 km (the table of the simulator's own, atmosphere.hpp), for when no spec has arrived. Returns [T K, p Pa, rho, a m/s]. */
const LAYERS = [[0, 288.15, 101325, -0.0065], [11000, 216.65, 22632.06, 0], [20000, 216.65, 5474.889, 0.001], [32000, 228.65, 868.0187, 0.0028], [47000, 270.65, 110.9063, 0], [51000, 270.65, 66.93887, -0.0028], [71000, 214.65, 3.95642, -0.002]];
export function us1976(altM) {
  const h = Math.max(0, altM) * 6356766 / (6356766 + Math.max(0, altM));
  if (h > 84852) { const t = 186.87, p = 0.3734 * Math.exp(-(h - 84852) / 5500), rho = p / (287.053 * t); return [t, p, rho, Math.sqrt(1.4 * 287.053 * t)]; }
  let i = LAYERS.length - 1; while (h < LAYERS[i][0]) i--;
  const [hb, tb, pb, L] = LAYERS[i], dh = h - hb, g0 = 9.80665, R = 287.053;
  const t = tb + L * dh, p = L === 0 ? pb * Math.exp(-g0 * dh / (R * tb)) : pb * Math.pow(t / tb, -g0 / (L * R));
  return [t, p, p / (R * t), Math.sqrt(1.4 * R * t)];
}

/** The spec the simulator sent, made handy: the vehicle's parts, the planet, and the air by altitude (interpolated). */
export class Spec {
  constructor(raw) {
    this.raw = raw;
    this.name = raw.name || 'vehicle';
    const v = raw.vehicle || {};
    this.vehicle = v;
    this.stages = v.stages || [];
    this.engines = v.engines || [];
    this.payloads = v.payloads || [];
    this.fins = v.fins || [];
    this.planet = { radius: raw.radius_m || 6378137, rotation: raw.rotation_rad_s || 0, mu: raw.mu || 3.986004418e14, pole: raw.pole || [1, 0, 0] };
    this.air = raw.air || null;
    this.scenario = v.scenario || {};
    this.site = this.scenario.site || { latitude_deg: 28.5, azimuth_deg: 90 };
    // tanks in the order a pose lists them: stage by stage
    this.tanks = [];
    this.stages.forEach((st, si) => (st.tanks || []).forEach((t, ti) => this.tanks.push({ stage: si, index: ti, ...t })));
    this.diameter = this.referenceDiameter();
    this.length = this.totalLength();
  }
  referenceDiameter() {
    let d = 0;
    for (const st of this.stages) for (const s of st.sections || []) d = Math.max(d, s.d_aft_m || 0, s.d_fore_m || 0);
    for (const p of this.payloads) for (const s of p.sections || []) d = Math.max(d, s.d_aft_m || 0);
    return d || (this.vehicle.aero && this.vehicle.aero.diameter_m) || 1.8;
  }
  totalLength() {
    let top = 0;
    for (const st of this.stages) { top = Math.max(top, (st.x_start_m || 0) + (st.length_m || 0)); for (const s of st.sections || []) top = Math.max(top, s.x_start_m + s.length_m); }
    for (const p of this.payloads) for (const s of p.sections || []) top = Math.max(top, s.x_start_m + s.length_m);
    return top || 20;
  }
  /** The air at an altitude from the profile the simulator sampled (T K, p Pa, rho, a m/s, mean wind m/s). */
  airAt(alt) {
    const a = this.air;
    if (!a || !a.density) { const r = us1976(alt); return [r[0], r[1], r[2], r[3], 0]; }
    const step = a.step_m, n = a.density.length;
    const x = Math.max(0, alt) / step, i = Math.min(n - 2, Math.floor(x)), f = Math.min(1, x - i);
    if (x >= n - 1) { const k = n - 1, sc = Math.exp(-(alt - k * step) / 6000); return [a.temperature_k[k], a.pressure_pa[k] * sc, a.density[k] * sc, a.sound_ms[k], a.wind_ms[k]]; }
    const L = (arr) => arr[i] + (arr[i + 1] - arr[i]) * f;
    // density and pressure fall exponentially: interpolate their logarithms
    const LL = (arr) => Math.exp(Math.log(arr[i]) + (Math.log(arr[i + 1]) - Math.log(arr[i])) * f);
    return [L(a.temperature_k), LL(a.pressure_pa), LL(a.density), L(a.sound_ms), L(a.wind_ms)];
  }
}

/** The stand-in spec when the simulator sends none: the reference vehicle's size (a 1.8 m, 24 m single-stage rocket). Only its look; its numbers come from the poses. */
export function fallbackSpec() {
  return new Spec({
    name: 'reference vehicle (no spec received)', pole: [1, 0, 0], radius_m: 6378137, rotation_rad_s: 0, mu: 3.986004418e14,
    vehicle: {
      stages: [{ name: 'stage 1', x_start_m: 0, length_m: 24, radius_m: 0.9, tanks: [{ propellant_kg: 25000, x_bottom_m: 3, radius_m: 0.85, density_kg_m3: 900 }], sections: [{ kind: 'tube', x_start_m: 0, length_m: 20, d_aft_m: 1.8, d_fore_m: 1.8 }, { kind: 'nose', x_start_m: 20, length_m: 4, d_aft_m: 1.8, d_fore_m: 0, shape: 'ogive' }] }],
      engines: [0, 1, 2, 3, 4].map((i) => ({ stage: 0, position_m: [0, i === 0 ? 0 : 0.5 * Math.cos(i * Math.PI / 2), i === 0 ? 0 : 0.5 * Math.sin(i * Math.PI / 2)], thrust_vac_n: 120000, exit_area_m2: 0.12, isp_vac_s: 310, gimbal: true })),
      payloads: [], fins: [], scenario: {},
    },
  });
}

/* ------------------------------------------------------------------------------------------------------------------------------------------------ poses */
const lerp = (a, b, f) => a + (b - a) * f;
const lerpArr = (a, b, f) => a.map((x, i) => x + (b[i] - x) * f);

function slerp(a, b, f) {                           // quaternions as [w, x, y, z]
  let cos = a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
  const s = cos < 0 ? -1 : 1; cos *= s;
  if (cos > 0.9995) { const q = a.map((x, i) => x + (s * b[i] - x) * f); const n = Math.hypot(...q); return q.map((x) => x / n); }
  const th = Math.acos(Math.min(1, cos)), sn = Math.sin(th), wa = Math.sin((1 - f) * th) / sn, wb = Math.sin(f * th) / sn * s;
  return a.map((x, i) => x * wa + b[i] * wb);
}

/** One pose between two others. Position follows a cubic through both positions and velocities (the vehicle accelerates hard: a straight line would cut the corner), the attitude a slerp, every other number a line. */
export function between(p0, p1, f) {
  const out = {};
  for (const k of Object.keys(p1)) {
    const a = p0[k], b = p1[k];
    if (typeof b === 'number') out[k] = (typeof a === 'number') ? lerp(a, b, f) : b;
    else if (Array.isArray(b) && Array.isArray(a) && a.length === b.length && typeof b[0] === 'number') out[k] = lerpArr(a, b, f);
    else out[k] = b;
  }
  const h = p1.tt - p0.tt;
  if (h > 0 && p0.r && p1.r) {                      // Hermite
    const t = f, t2 = t * t, t3 = t2 * t, h00 = 2 * t3 - 3 * t2 + 1, h10 = t3 - 2 * t2 + t, h01 = -2 * t3 + 3 * t2, h11 = t3 - t2;
    out.r = p0.r.map((x, i) => h00 * x + h10 * h * p0.v[i] + h01 * p1.r[i] + h11 * h * p1.v[i]);
  }
  if (p0.q && p1.q) out.q = slerp(p0.q, p1.q, f);
  // discrete things take the later pose's value once it is more than half-way (the stage bits, the clamp, the ground flag)
  for (const k of ['stg', 'ign', 'pay', 'gnd', 'crash', 'clamp', 'act', 'fr']) if (k in p1) out[k] = f < 0.5 && k in p0 ? p0[k] : p1[k];
  return out;
}

/** The poses received, in order, and the sample at any time. */
export class PoseBuffer {
  constructor() { this.reset(); }
  reset() { this.poses = []; this.latest = null; this.version = 0; }
  push(p) {
    const a = this.poses;
    if (a.length && p.tt < a[a.length - 1].tt - 1e-6) { a.length = 0; this.version++; }   // time went backwards (a seek, a restart): the old poses are another flight
    a.push(p);
    if (a.length > 400) a.splice(0, a.length - 300);
    this.latest = p;
  }
  /** The pose at vehicle time t: interpolated inside the buffer, held a little beyond it. */
  at(t) {
    const a = this.poses, n = a.length;
    if (!n) return null;
    if (t <= a[0].tt) return a[0];
    if (t >= a[n - 1].tt) return a[n - 1];
    let lo = 0, hi = n - 1;
    while (hi - lo > 1) { const m = (lo + hi) >> 1; if (a[m].tt <= t) lo = m; else hi = m; }
    const f = (t - a[lo].tt) / ((a[hi].tt - a[lo].tt) || 1);
    return between(a[lo], a[hi], f);
  }
}

/** A clock that follows the poses smoothly: it advances in real time at the source's speed and is pulled gently to just behind the newest pose, so a late packet never shows as a jump. */
export class PlayClock {
  constructor() { this.t = 0; this.have = false; this.lastWall = performance.now(); this.rate = 1; this.lag = 0.1; }
  frame(buf, source, truthOnly) {
    const now = performance.now(), dt = Math.min(0.25, (now - this.lastWall) / 1000); this.lastWall = now;
    const last = buf.latest;
    if (!last) { this.have = false; return null; }
    const paused = source && source.kind !== 'live' && source.playing === false;
    const speed = source && source.speed ? source.speed : 1;
    this.rate = paused ? 0 : (source && source.kind !== 'live' ? speed : 1);
    const target = last.tt - this.lag * Math.max(1, this.rate) * (truthOnly ? 1.2 : 1);
    if (!this.have || Math.abs(target - this.t) > 1.5 + 0.5 * this.rate) { this.t = target; this.have = true; }
    else { this.t += dt * this.rate; if (!paused) this.t += (target - this.t) * Math.min(1, dt * 4); else this.t = Math.min(this.t, last.tt); }
    return this.t;
  }
}

/* ------------------------------------------------------------------------------------------------------------------------------------------------ the fallback */
/** A pose from the console's 10 Hz truth (altitude, range, tilts, speed, Mach, q, mass, thrust, propellant): the vehicle's place and the direction of its long axis, nothing more. Roll is zero, there are no forces and no per-engine state. */
export function poseFromTruth(d, spec) {
  const R = spec.planet.radius, th = (d.range || 0) / R, r = R + (d.alt || 0);
  const pos = [r * Math.cos(th), r * Math.sin(th), 0];
  const tp = (d.tilt_p || 0) * RAD, ty = (d.tilt_y || 0) * RAD;
  // the long axis in the launch frame: pitch toward downrange (+Y), yaw toward -Z, both measured from +X
  const ux = Math.cos(ty) * Math.cos(tp), uy = Math.cos(ty) * Math.sin(tp), uz = -Math.sin(ty);
  const nrm = Math.hypot(ux, uy, uz), u = [ux / nrm, uy / nrm, uz / nrm];
  // the shortest rotation from +X to u: q = (1 + x.u, x cross u) normalised
  const c = [0, -u[2], u[1]];                       // (1,0,0) x u
  let q = [1 + u[0], c[0], c[1], c[2]]; const qn = Math.hypot(...q);
  q = qn < 1e-9 ? [0, 0, 1, 0] : q.map((x) => x / qn);
  const air = spec.airAt(d.alt || 0);
  const sp = d.speed || 0, vdir = [Math.cos(Math.max(0, tp)), Math.sin(tp), 0];
  const live = { prop: d.prop || [], tank: [] };
  return {
    k: 'pose', t: d.ft || 0, tt: d.ft || 0, fr: d.frame || 0, r: pos, v: vdir.map((x) => x * sp), q, w: [0, 0, 0], alt: d.alt || 0, rng: d.range || 0, spd: sp, mach: d.mach || 0, qd: d.q || 0, alpha: 0,
    vair: [sp, 0, 0], wind: [0, 0, 0], air: air.slice(0, 4), m: d.mass || 0, cg: spec.length * 0.4, cp: spec.length * 0.7, cna: 2, ca: 0.3, dref: spec.diameter, fth: [d.thrust || 0, 0, 0], fae: [0, 0, 0], mth: [0, 0, 0], mae: [0, 0, 0],
    thr: d.thrust || 0, mdot: 0, eng: spec.engines.map(() => ((d.engines_on || 0) > 0 && (d.thrust || 0) > 0 ? 1 : 0)), gim: spec.stages.flatMap(() => [d.gim_p || 0, d.gim_y || 0]), prop: live.prop, tank: [],
    stg: d.stages_active ?? 1, ign: d.stages_ignited ?? 0, pay: 255, gnd: d.clamped ? 1 : 0, crash: d.crashed ? 1 : 0, clamp: d.clamped ? 1 : 0, tilt: [d.tilt_p || 0, d.tilt_y || 0], ref: [d.ref_p || 0, d.ref_y || 0], cmd: [0, 0], act: -1, truthOnly: true,
  };
}

/* ------------------------------------------------------------------------------------------------------------------------------------------------ the streams */
export class ViewerData {
  constructor() {
    this.spec = fallbackSpec(); this.haveSpec = false;
    this.buf = new PoseBuffer(); this.clock = new PlayClock();
    this.source = { kind: 'none' };
    this.snapshot = null;                           // the console's own state (the nodes, ACT, the mission clock), for the attitude lens and the header
    this.hist = { t: [], s: {} };                   // the charts' history, 10 per second of vehicle time
    this.onSpec = []; this.onReset = []; this.status = 'connecting';
    this.poseCount = 0; this.lastPoseWall = 0; this.rateEst = 0; this._rt = performance.now(); this._rn = 0;
    this.truthSeen = 0; this.consoleSource = null;
  }
  start() {
    if (!hasToken()) { this.status = 'no token'; return; }
    this._openPose();
    this._openMain();
  }
  _openPose() {
    const tok = new URLSearchParams(location.search).get('token') || sessionStorage.getItem('tfc.token') || '';
    const es = new EventSource(`/api/pose-stream?token=${encodeURIComponent(tok)}`);
    this.pes = es;
    es.onopen = () => { this.status = 'up'; };
    es.onerror = () => { this.status = 'down'; };
    es.addEventListener('hello', (e) => { const d = JSON.parse(e.data); if (d.source) this.source = d.source; if (d.spec) this._spec(d.spec); if (d.pose) this._pose(d.pose); });
    es.addEventListener('spec', (e) => this._spec(JSON.parse(e.data)));
    es.addEventListener('pose', (e) => this._pose(JSON.parse(e.data)));
    es.addEventListener('source', (e) => { this.source = JSON.parse(e.data); });
    es.addEventListener('reset', () => { this.buf.reset(); this.clock.have = false; this.hist = { t: [], s: {} }; for (const f of this.onReset) f(); });
  }
  _openMain() {
    const tok = new URLSearchParams(location.search).get('token') || sessionStorage.getItem('tfc.token') || '';
    const es = new EventSource(`/api/stream?token=${encodeURIComponent(tok)}`);
    const take = (e) => { try { const d = JSON.parse(e.data); const s = d.snapshot || null; if (s) { this.snapshot = s; this.consoleSource = s.source || null; this._truth(s.truth); } } catch { /* a bad message is skipped */ } };
    es.addEventListener('hello', take); es.addEventListener('state', take);
  }
  /** With no poses coming, the console's truth stands in for them. */
  _truth(t) {
    if (!t || !t.alive || !this.consoleSource) return;
    const fresh = this.lastPoseWall && performance.now() - this.lastPoseWall < 1500;
    if (fresh) return;                              // real poses are arriving
    if (t.ft === this._lastTruthFt) return;
    this._lastTruthFt = t.ft; this.truthSeen++;
    this._add(poseFromTruth(t, this.spec));
  }
  _spec(raw) {
    const key = raw.name + ':' + (raw.vehicle && raw.vehicle.engines ? raw.vehicle.engines.length : 0) + ':' + (raw.vehicle && raw.vehicle.stages ? raw.vehicle.stages.length : 0) + ':' + (raw.vehicle && raw.vehicle.stages && raw.vehicle.stages[0] ? raw.vehicle.stages[0].dry_mass_kg : 0);
    if (this.haveSpec && key === this._specKey) { this.spec.air = raw.air || this.spec.air; return; }
    this._specKey = key; this.haveSpec = true; this.spec = new Spec(raw);
    for (const f of this.onSpec) f(this.spec);
  }
  _pose(p) {
    this.lastPoseWall = performance.now(); this.poseCount++; this._rn++;
    const now = performance.now(); if (now - this._rt > 1000) { this.rateEst = this._rn * 1000 / (now - this._rt); this._rt = now; this._rn = 0; }
    this._add(p);
  }
  _add(p) {
    this.buf.push(p);
    const H = this.hist, t = p.tt;
    if (H.t.length && t < H.t[H.t.length - 1] - 1e-6) { this.hist = { t: [], s: {} }; return this._add(p); }
    if (!H.t.length || t - H.t[H.t.length - 1] >= 0.1) {
      const m = metrics(p);
      const n = H.t.length; H.t.push(t);
      for (const k of new Set([...Object.keys(H.s), ...Object.keys(m)])) { if (!H.s[k]) H.s[k] = new Array(n).fill(null); H.s[k].push(m[k] ?? null); }
      if (H.t.length > 9000) { H.t.splice(0, 1000); for (const k of Object.keys(H.s)) H.s[k].splice(0, 1000); }
    }
  }
  get live() { return this.lastPoseWall && performance.now() - this.lastPoseWall < 1500; }
}

/** The numbers the charts plot, from one pose. */
export function metrics(p) {
  const fa = p.fae || [0, 0, 0], ft = p.fth || [0, 0, 0], m = p.m || 1;
  const nx = (fa[0] + ft[0]) / m / 9.80665, nlat = Math.hypot(fa[1] + ft[1], fa[2] + ft[2]) / m / 9.80665;
  return {
    alt: p.alt, speed: p.spd, mach: p.mach, q: p.qd, alpha: p.alpha, qalpha: p.qd * Math.abs(p.alpha) * RAD, thrust: p.thr, mass: p.m, gx: nx, glat: nlat,
    static_margin: p.dref ? (p.cg - p.cp) / p.dref : 0, cp: p.cp, cg: p.cg, p_amb: p.air ? p.air[1] : null, rho: p.air ? p.air[2] : null, temp: p.air ? p.air[0] : null,
    tilt_p: p.tilt ? p.tilt[0] : null, ref_p: p.ref ? p.ref[0] : null, err_p: p.tilt && p.ref ? p.tilt[0] - p.ref[0] : null, err_y: p.tilt && p.ref ? p.tilt[1] - p.ref[1] : null,
    gim_p: p.gim ? p.gim[0] : null, gim_y: p.gim ? p.gim[1] : null, drag: -fa[0], normal: Math.hypot(fa[1], fa[2]), mdot: p.mdot, isp: p.mdot > 1 ? p.thr / (p.mdot * 9.80665) : null,
  };
}

export { api };
