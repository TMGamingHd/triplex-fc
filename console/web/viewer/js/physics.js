// The numbers the lenses show that are not in a pose as they are: formulas on the pose's own values. Each says in its comment which textbook relation it is, so that a number on the page can be traced to
// either the simulator (the pose) or one of these. Nothing here changes what the simulator did.
import { us1976 } from './data.js';

const G0 = 9.80665, GAMMA = 1.4, RAD = Math.PI / 180, DEG = 180 / Math.PI;

export const vlen = (v) => Math.hypot(v[0], v[1], v[2]);
export const vdot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
export const vscale = (a, s) => [a[0] * s, a[1] * s, a[2] * s];
export const vsub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
export const vadd = (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]];

/** Dynamic viscosity of air, Sutherland's law (Pa s). */
export const viscosity = (T) => 1.458e-6 * Math.pow(T, 1.5) / (T + 110.4);

/** Total pressure ratio p0/p of a flow at Mach M that is brought to rest: isentropically below Mach 1, through a normal shock above it (Rayleigh's pitot formula). */
export function pitotRatio(M) {
  if (M < 1) return Math.pow(1 + 0.2 * M * M, 3.5);
  const g = GAMMA;
  return Math.pow((g + 1) * (g + 1) * M * M / (4 * g * M * M - 2 * (g - 1)), g / (g - 1)) * (1 - g + 2 * g * M * M) / (g + 1);
}

/** The pressure coefficient at a stagnation point: (p0 - p) / q, with q = 0.5 gamma p M^2. 1 at low speed, about 1.8 at high Mach. */
export function cpStagnation(M) { if (M < 0.05) return 1; return (pitotRatio(M) - 1) / (0.5 * GAMMA * M * M); }

/** Which atmospheric layer an altitude is in (the 1976 standard's names), with its base and top in km. */
export function layerOf(altM) {
  const k = altM / 1000;
  if (k < 11) return { name: 'troposphere', lo: 0, hi: 11 };
  if (k < 20) return { name: 'lower stratosphere', lo: 11, hi: 20 };
  if (k < 47) return { name: 'stratosphere', lo: 20, hi: 47 };
  if (k < 51) return { name: 'stratopause', lo: 47, hi: 51 };
  if (k < 86) return { name: 'mesosphere', lo: 51, hi: 86 };
  if (k < 100) return { name: 'thermosphere (below the Karman line)', lo: 86, hi: 100 };
  return { name: 'thermosphere / space', lo: 100, hi: 1000 };
}

export function machRegime(M) {
  if (M < 0.3) return 'incompressible subsonic';
  if (M < 0.8) return 'subsonic';
  if (M < 1.2) return 'transonic';
  if (M < 3) return 'supersonic';
  if (M < 5) return 'high supersonic';
  return 'hypersonic';
}

/** The Taylor-Maccoll cone flow: the shock angle (rad) of an attached shock on a cone of half-angle `theta` (rad) at Mach M, found by integrating the ODE of the conical flow from the shock to the cone. null when the shock would be detached. */
export function coneShockAngle(M, theta) {
  const g = GAMMA;
  const f = (beta) => {
    // post-shock velocity (normalised by the stagnation speed) from the oblique-shock relations
    const Mn1 = M * Math.sin(beta);
    if (Mn1 <= 1) return null;
    const Mn2sq = (1 + 0.5 * (g - 1) * Mn1 * Mn1) / (g * Mn1 * Mn1 - 0.5 * (g - 1));
    const delta = Math.atan(2 / Math.tan(beta) * (Mn1 * Mn1 - 1) / (M * M * (g + Math.cos(2 * beta)) + 2));
    const M2 = Math.sqrt(Mn2sq) / Math.sin(beta - delta);
    const V = Math.pow(1 + 2 / ((g - 1) * M2 * M2), -0.5);               // V / Vmax
    let Vr = V * Math.cos(beta - delta), Vt = -V * Math.sin(beta - delta);
    let th = beta; const h = -0.0005;
    const k = (g - 1) / 2;
    const d = (vr, vt, t) => {
      const a = k * (1 - vr * vr - vt * vt);
      return [vt, (vt * vt * vr - a * (2 * vr + vt / Math.tan(t))) / (a - vt * vt)];
    };
    for (let i = 0; i < 6000 && Vt < 0; i++) {                              // RK4 in theta until the normal velocity vanishes: that is the cone's surface
      const k1 = d(Vr, Vt, th), k2 = d(Vr + 0.5 * h * k1[0], Vt + 0.5 * h * k1[1], th + 0.5 * h), k3 = d(Vr + 0.5 * h * k2[0], Vt + 0.5 * h * k2[1], th + 0.5 * h), k4 = d(Vr + h * k3[0], Vt + h * k3[1], th + h);
      Vr += h / 6 * (k1[0] + 2 * k2[0] + 2 * k3[0] + k4[0]); Vt += h / 6 * (k1[1] + 2 * k2[1] + 2 * k3[1] + k4[1]); th += h;
      if (th <= 0.001) return 0;
    }
    return th;
  };
  const mu = Math.asin(1 / M);
  let lo = mu + 1e-4, hi = Math.PI / 2 - 1e-3;
  const fl = f(lo), fh = f(hi);
  if (fl == null) return null;
  if (fh == null || fh < theta) return null;                                 // no attached solution at this cone angle
  for (let i = 0; i < 40; i++) { const mid = 0.5 * (lo + hi), fm = f(mid); if (fm == null) { lo = mid; continue; } if (fm < theta) lo = mid; else hi = mid; }
  return 0.5 * (lo + hi);
}

const _cone = new Map();
/** The cone shock angle with a cache (Mach to 0.05). */
export function coneShockCached(M, theta) {
  const key = Math.round(M * 20) + ':' + Math.round(theta * 400);
  if (!_cone.has(key)) _cone.set(key, coneShockAngle(Math.round(M * 20) / 20, theta));
  return _cone.get(key);
}

/** Everything the aerodynamics lens shows about one pose. */
export function derive(pose, spec) {
  const air = pose.air || us1976(pose.alt || 0), T = air[0], p = air[1], rho = air[2], a = air[3];
  const v = pose.vair || [pose.spd || 0, 0, 0], V = vlen(v) || 1e-9;
  const q = pose.qd != null ? pose.qd : 0.5 * rho * V * V;
  const M = pose.mach != null ? pose.mach : V / a;
  const S = Math.PI * 0.25 * (pose.dref || spec.diameter) ** 2;
  const fae = pose.fae || [0, 0, 0], vhat = [v[0] / V, v[1] / V, v[2] / V];
  const D = -vdot(fae, vhat), Lvec = vadd(fae, vscale(vhat, D)), L = vlen(Lvec);
  const alpha = Math.atan2(Math.hypot(v[1], v[2]), v[0]) * DEG;
  const phi = Math.atan2(v[2], v[1]) * DEG;                                  // the plane the angle of attack is in, from body +Y toward +Z
  const N = Math.hypot(fae[1], fae[2]);
  const T0 = T * (1 + 0.2 * M * M);
  const nose = (spec.noseRadius ?? 0.5);
  const heat = 1.7415e-4 * Math.sqrt(rho / nose) * V * V * V;                // Sutton-Graves: stagnation-point convective heat flux, W/m^2, for an assumed nose radius
  const Re = rho * V * (spec.length || 1) / viscosity(T);
  const dc = pose.dref || spec.diameter;
  return {
    T, p, rho, a, V, q, M, S, D, L, alpha, phi, N, T0, heat, Re,
    cn: q > 1 ? N / (q * S) : 0, ca: q > 1 ? -fae[0] / (q * S) : 0, cd: q > 1 ? D / (q * S) : 0, cl: q > 1 ? L / (q * S) : 0, ld: D > 1 ? L / D : 0,
    margin: dc ? (pose.cg - pose.cp) / dc : 0,                              // static margin in calibres: positive when the centre of pressure is behind the centre of gravity (the air turns the vehicle back into the wind: stable); negative: unstable, the controller holds it
    cpMax: cpStagnation(M), mu: M > 1 ? Math.asin(1 / M) * DEG : null, regime: machRegime(M), layer: layerOf(pose.alt || 0),
    fhat: vscale(vhat, -1),                                                  // the direction the air moves past the vehicle, in the body frame
    vhat, Lvec, thrustAxial: (pose.fth || [0, 0, 0])[0], gx: ((pose.fae || [0])[0] + (pose.fth || [0])[0]) / (pose.m || 1) / G0,
    glat: Math.hypot((pose.fae || [0, 0, 0])[1] + (pose.fth || [0, 0, 0])[1], (pose.fae || [0, 0, 0])[2] + (pose.fth || [0, 0, 0])[2]) / (pose.m || 1) / G0,
  };
}

export { G0, GAMMA, RAD, DEG };
