// A live flow solver on the graphics card: the axisymmetric Euler equations in the plane of the vehicle's axis and its radius, solved in the fragment shaders of WebGL2 (docs/design/CFD.md section 10).
//
// It is the scheme of the offline solver (sim/cfd, docs/design/CFD.md section 2) cut down to two dimensions: a finite-volume method on a uniform grid of cells, MUSCL reconstruction with the van Albada limiter,
// the HLLC flux with HLL where a strong shock lies along a face, two-stage Runge-Kutta at a local time step toward a steady state, the body cut out of the grid by a signed distance with ghost cells that take the
// mirror image of the fluid at their image points (the slip wall). The equations are those of a body of revolution at no angle of attack: with r the radius and (u, v) the axial and radial velocities,
//   d(rU)/dt + d(rF)/dx + d(rG)/dr = (0, 0, p, 0),  U = (rho, rho u, rho v, E).
// It runs a few thousand iterations a second, so that a flow settles in a second or two and follows the Mach number of the flight. It is a picture of the flow at *zero angle of attack*: the offline cases
// have the angle. Gas: calorically perfect, gamma 1.4; inviscid. The state lives in float textures: the page only draws it.
import * as THREE from '../vendor/three.module.js';
import { COLORMAP_GLSL } from './gfx.js';

const GAMMA = 1.4;

const VERT = /* glsl */`
out vec2 vUv;
void main() { vUv = uv; gl_Position = vec4(position.xy, 0.0, 1.0); }`;

const COMMON = /* glsl */`
precision highp float; precision highp int; precision highp sampler2D;
const float G = ${GAMMA.toFixed(1)};
uniform ivec2 uN;                 // cells in x and r
uniform float uH;                 // the cell, m
uniform vec4 uInf;                // the free stream: rho, rho u, rho v, E
uniform sampler2D uD;             // the signed distance to the body at the cell centres (m), negative inside
vec4 toPrim(vec4 U) { float r = U.x; vec2 w = U.yz / r; return vec4(r, w, (G - 1.0) * (U.w - 0.5 * r * dot(w, w))); }
vec4 toCons(vec4 W) { return vec4(W.x, W.x * W.y, W.x * W.z, W.w / (G - 1.0) + 0.5 * W.x * (W.y * W.y + W.z * W.z)); }
float sdf(ivec2 c) { return texelFetch(uD, c, 0).r; }
`;

// The flux across a face from the primitive states on its two sides: (rho, normal velocity, tangential velocity, p) in; (mass, normal momentum, tangential momentum, energy) out.
const FLUX = /* glsl */`
vec4 hllc(vec4 L, vec4 R) {
  float cl = sqrt(G * L.w / L.x), cr = sqrt(G * R.w / R.x);
  float El = L.w / (G - 1.0) + 0.5 * L.x * (L.y * L.y + L.z * L.z), Er = R.w / (G - 1.0) + 0.5 * R.x * (R.y * R.y + R.z * R.z);
  float sl = sqrt(L.x), sr = sqrt(R.x), inv = 1.0 / (sl + sr);
  float uh = (sl * L.y + sr * R.y) * inv, vh = (sl * L.z + sr * R.z) * inv;
  float Hl = (El + L.w) / L.x, Hr = (Er + R.w) / R.x, Hh = (sl * Hl + sr * Hr) * inv;
  float ch = sqrt(max(1e-12, (G - 1.0) * (Hh - 0.5 * (uh * uh + vh * vh))));
  float SL = min(L.y - cl, uh - ch), SR = max(R.y + cr, uh + ch);
  vec4 FL = vec4(L.x * L.y, L.x * L.y * L.y + L.w, L.x * L.y * L.z, (El + L.w) * L.y);
  vec4 FR = vec4(R.x * R.y, R.x * R.y * R.y + R.w, R.x * R.y * R.z, (Er + R.w) * R.y);
  if (SL >= 0.0) return FL;
  if (SR <= 0.0) return FR;
  vec4 UL = vec4(L.x, L.x * L.y, L.x * L.z, El), UR = vec4(R.x, R.x * R.y, R.x * R.z, Er);
  if (max(L.w, R.w) > 1.8 * min(L.w, R.w)) return (SR * FL - SL * FR + SL * SR * (UR - UL)) / (SR - SL);     // a strong shock on the face: HLL
  float Ss = (R.w - L.w + L.x * L.y * (SL - L.y) - R.x * R.y * (SR - R.y)) / (L.x * (SL - L.y) - R.x * (SR - R.y));
  if (Ss >= 0.0) {
    float f = L.x * (SL - L.y) / (SL - Ss);
    vec4 Us = vec4(f, f * Ss, f * L.z, f * (El / L.x + (Ss - L.y) * (Ss + L.w / (L.x * (SL - L.y)))));
    return FL + SL * (Us - UL);
  }
  float f = R.x * (SR - R.y) / (SR - Ss);
  vec4 Us = vec4(f, f * Ss, f * R.z, f * (Er / R.x + (Ss - R.y) * (Ss + R.w / (R.x * (SR - R.y)))));
  return FR + SR * (Us - UR);
}
float vanAlbada(float a, float b) { return a * b <= 0.0 ? 0.0 : a * b * (a + b) / (a * a + b * b + 1e-30); }
vec4 slope(vec4 a, vec4 b) { return vec4(vanAlbada(a.x, b.x), vanAlbada(a.y, b.y), vanAlbada(a.z, b.z), vanAlbada(a.w, b.w)); }
`;

// The pass that fills the ghost cells: each solid cell next to the fluid takes the fluid state at its image point with the velocity's normal component reversed.
const GHOST_FRAG = /* glsl */`
${COMMON}
uniform sampler2D uS;
in vec2 vUv; out vec4 outColor;
vec4 fetchS(ivec2 c) { return texelFetch(uS, c, 0); }
void main() {
  ivec2 c = ivec2(gl_FragCoord.xy);
  float d = sdf(c);
  if (d >= 0.0) { outColor = fetchS(c); return; }
  if (d < -3.2 * uH) { outColor = uInf; return; }
  // the normal from the gradient of the distance
  ivec2 e = ivec2(1, 0), f = ivec2(0, 1);
  vec2 n = vec2(sdf(min(c + e, uN - 1)) - sdf(max(c - e, ivec2(0))), sdf(min(c + f, uN - 1)) - sdf(max(c - f, ivec2(0))));
  n = length(n) > 1e-9 ? normalize(n) : vec2(1.0, 0.0);
  vec2 p = (vec2(c) + 0.5);                                   // the centre, in cells
  vec2 q = p - 2.0 * d / uH * n - 0.5;                        // the image point (the cell's distance is in metres) as a position among the centres
  ivec2 i0 = ivec2(floor(q)); vec2 fr = q - vec2(i0);
  vec4 acc = vec4(0.0); float wsum = 0.0;
  for (int k = 0; k < 4; k++) {
    ivec2 o = ivec2(k & 1, k >> 1); ivec2 cc = clamp(i0 + o, ivec2(0), uN - 1);
    float w = (o.x == 0 ? 1.0 - fr.x : fr.x) * (o.y == 0 ? 1.0 - fr.y : fr.y);
    if (sdf(cc) > 0.0 && w > 1e-6) { acc += w * toPrim(fetchS(cc)); wsum += w; }
  }
  if (wsum < 1e-6) {                                           // no fluid in the four: the nearest in the block round them
    float best = 1e9;
    for (int a = -1; a <= 2; a++) for (int b = -1; b <= 2; b++) {
      ivec2 cc = clamp(i0 + ivec2(a, b), ivec2(0), uN - 1);
      float dd = length(vec2(cc) + 0.5 - q - 0.5);
      if (sdf(cc) > 0.0 && dd < best) { best = dd; acc = toPrim(fetchS(cc)); wsum = 1.0; }
    }
  }
  if (wsum < 1e-6) { outColor = uInf; return; }
  vec4 W = acc / wsum;
  float vn = dot(W.yz, n);
  W.yz -= 2.0 * vn * n;                                        // the normal velocity reversed: no flow through the surface
  outColor = toCons(W);
}`;

// The pass that advances the fluid cells one stage: out = a U0 + b (S + dt R(G)), with R the residual from the ghost-filled state G and the local time step dt.
const UPDATE_FRAG = /* glsl */`
${COMMON}
${FLUX}
uniform sampler2D uG, uS, uU0;
uniform float uCfl, uA, uB;
in vec2 vUv; out vec4 outColor;
vec4 prim(ivec2 c) {                                           // the primitive state of a cell of the grid or of the boundary beyond it
  if (c.x >= uN.x) return toPrim(uInf);                         // x high: the inflow
  c.x = max(c.x, 0);                                            // x low: the outflow, as the cell next to it
  bool mirror = c.y < 0;
  if (mirror) c.y = -c.y - 1;                                   // the axis: the mirror image, the radial velocity reversed
  c.y = min(c.y, uN.y - 1);                                     // r high: the cell next to it
  vec4 W = toPrim(texelFetch(uG, c, 0));
  if (mirror) W.z = -W.z;
  return W;
}
vec4 mu(vec4 a, vec4 b, vec4 c) { return slope(b - a, c - b); }
void main() {
  ivec2 c = ivec2(gl_FragCoord.xy);
  if (sdf(c) < 0.0) { outColor = texelFetch(uG, c, 0); return; }
  vec4 Wm2 = prim(c + ivec2(-2, 0)), Wm1 = prim(c + ivec2(-1, 0)), W0 = prim(c), Wp1 = prim(c + ivec2(1, 0)), Wp2 = prim(c + ivec2(2, 0));
  vec4 Vm2 = prim(c + ivec2(0, -2)), Vm1 = prim(c + ivec2(0, -1)), Vp1 = prim(c + ivec2(0, 1)), Vp2 = prim(c + ivec2(0, 2));
  // x faces: low (between c-1 and c) and high (between c and c+1); the states reconstructed on each side, the normal velocity first
  vec4 sm1 = mu(Wm2, Wm1, W0), s0 = mu(Wm1, W0, Wp1), sp1 = mu(W0, Wp1, Wp2);
  vec4 aL = Wm1 + 0.5 * sm1, aR = W0 - 0.5 * s0, bL = W0 + 0.5 * s0, bR = Wp1 - 0.5 * sp1;
  if (aL.x <= 0.0 || aL.w <= 0.0 || aR.x <= 0.0 || aR.w <= 0.0) { aL = Wm1; aR = W0; }
  if (bL.x <= 0.0 || bL.w <= 0.0 || bR.x <= 0.0 || bR.w <= 0.0) { bL = W0; bR = Wp1; }
  // the fluxes in (rho, rho u, rho v, E): the normal is x, the normal velocity u (second), the tangential v (third)
  vec4 Fa = hllc(vec4(aL.x, aL.y, aL.z, aL.w), vec4(aR.x, aR.y, aR.z, aR.w));
  vec4 Fb = hllc(vec4(bL.x, bL.y, bL.z, bL.w), vec4(bR.x, bR.y, bR.z, bR.w));
  // r faces: the normal is r, so the normal velocity is v: permute (rho, v, u, p) in and the momenta out
  vec4 tm1 = mu(Vm2, Vm1, W0), t0 = mu(Vm1, W0, Vp1), tp1 = mu(W0, Vp1, Vp2);
  vec4 cL = Vm1 + 0.5 * tm1, cR = W0 - 0.5 * t0, dL = W0 + 0.5 * t0, dR = Vp1 - 0.5 * tp1;
  if (cL.x <= 0.0 || cL.w <= 0.0 || cR.x <= 0.0 || cR.w <= 0.0) { cL = Vm1; cR = W0; }
  if (dL.x <= 0.0 || dL.w <= 0.0 || dR.x <= 0.0 || dR.w <= 0.0) { dL = W0; dR = Vp1; }
  vec4 Gc = hllc(vec4(cL.x, cL.z, cL.y, cL.w), vec4(cR.x, cR.z, cR.y, cR.w));
  vec4 Gd = hllc(vec4(dL.x, dL.z, dL.y, dL.w), vec4(dR.x, dR.z, dR.y, dR.w));
  Gc = vec4(Gc.x, Gc.z, Gc.y, Gc.w); Gd = vec4(Gd.x, Gd.z, Gd.y, Gd.w);          // back to (mass, x momentum, r momentum, energy)
  float rc = (float(c.y) + 0.5) * uH, rlo = float(c.y) * uH, rhi = float(c.y + 1) * uH;
  vec4 R = -(Fb - Fa) / uH - (rhi * Gd - rlo * Gc) / (rc * uH);
  R.z += W0.w / rc;                                                              // the pressure's share of the radial momentum in the cylindrical form
  float cs = sqrt(G * W0.w / W0.x);
  float dt = uCfl * uH / (abs(W0.y) + abs(W0.z) + 2.0 * cs);
  vec4 Us = texelFetch(uS, c, 0) + dt * R;
  vec4 U = uA * texelFetch(uU0, c, 0) + uB * Us;
  // the floors: a density and a pressure that stay positive
  U.x = max(U.x, 1e-4 * uInf.x);
  float ke = 0.5 * (U.y * U.y + U.z * U.z) / U.x;
  U.w = max(U.w, ke + 1e-4 * uInf.w / (G * 0.5));
  outColor = U;
}`;

// The picture of the state on the section: Mach number, pressure or schlieren, over the plane through the axis (the radius runs both ways: the flow is symmetric about the axis).
const VIEW_VERT = /* glsl */`varying vec2 vUv; void main() { vUv = uv; gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0); }`;
const VIEW_FRAG = /* glsl */`
precision highp float; precision highp int; precision highp sampler2D;
${COLORMAP_GLSL}
varying vec2 vUv;
uniform sampler2D uState, uD;
uniform ivec2 uN;
uniform float uH, uMode, uOpacity, uMach, uPInf, uRhoInf, uMachHi;
const float G = ${GAMMA.toFixed(1)};
vec4 cell(ivec2 c) { c = clamp(c, ivec2(0), uN - 1); return texelFetch(uState, c, 0); }
float lnrho(ivec2 c) { return log(max(cell(c).x, 1e-6)); }
void main() {
  // vUv.y runs from -r_max to +r_max: the radius is |.|; vUv.x is x
  float xr = vUv.x * float(uN.x), rr = abs(vUv.y * 2.0 - 1.0) * float(uN.y);
  ivec2 c = ivec2(clamp(floor(vec2(xr, rr)), vec2(0.0), vec2(uN) - 1.0));
  if (texelFetch(uD, c, 0).r < 0.0) discard;
  vec4 U = cell(c); float rho = U.x; vec2 w = U.yz / rho; float p = (G - 1.0) * (U.w - 0.5 * rho * dot(w, w));
  float m = length(w) / sqrt(G * max(p, 1e-9) / rho);
  vec3 col; float a = uOpacity;
  if (uMode < 0.5) { col = cmap(m / uMachHi); a *= smoothstep(0.02, 0.14, abs(m - uMach) / max(uMach, 1.0)); }
  else if (uMode < 1.5) { float lp = log2(max(p / uPInf, 1e-3)); col = cmap((lp * 0.30103 + 1.0) / 3.0); a *= smoothstep(0.012, 0.09, abs(lp) * 0.30103); }
  else {
    vec2 g = vec2(lnrho(c + ivec2(1, 0)) - lnrho(c - ivec2(1, 0)), lnrho(c + ivec2(0, 1)) - lnrho(c - ivec2(0, 1))) / (2.0 * uH);
    g.y *= sign(vUv.y * 2.0 - 1.0);
    float strength = clamp(length(g) * 12.0, 0.0, 1.0);
    float s = 0.5 + 0.5 * clamp(g.x * 9.0 + g.y * 3.0, -1.0, 1.0);
    col = vec3(mix(0.02, 1.0, s)); a *= strength;
  }
  gl_FragColor = vec4(col, a);
  #include <tonemapping_fragment>
  #include <colorspace_fragment>
}`;

const rtOpts = { type: THREE.FloatType, format: THREE.RGBAFormat, minFilter: THREE.NearestFilter, magFilter: THREE.NearestFilter, depthBuffer: false, stencilBuffer: false, wrapS: THREE.ClampToEdgeWrapping, wrapT: THREE.ClampToEdgeWrapping };

/** The signed distance to a body of revolution (a profile [{x, r}] from the tail to the nose, closed on the axis) at the cell centres of a grid: x from x0, r from 0, n cells of h. Negative inside. */
export function bodyDistance(profile, x0, h, nx, nr) {
  const pts = profile.map((p) => [p.x, p.r]);
  const segs = [];
  for (let i = 0; i + 1 < pts.length; i++) segs.push([pts[i], pts[i + 1]]);
  if (pts[0][1] > 0) segs.push([[pts[0][0], 0], pts[0]]);
  if (pts[pts.length - 1][1] > 0) segs.push([pts[pts.length - 1], [pts[pts.length - 1][0], 0]]);
  const poly = pts.slice();
  if (pts[pts.length - 1][1] > 0) poly.push([pts[pts.length - 1][0], 0]);
  if (pts[0][1] > 0) poly.push([pts[0][0], 0]);
  const inside = (x, r) => {
    let inn = false;
    for (let i = 0; i < poly.length; i++) {
      const a = poly[i], b = poly[(i + 1) % poly.length];
      if ((a[1] > r) !== (b[1] > r)) { const xc = a[0] + (r - a[1]) * (b[0] - a[0]) / (b[1] - a[1]); if (x < xc) inn = !inn; }
    }
    return inn;
  };
  const out = new Float32Array(nx * nr);
  for (let j = 0; j < nr; j++) {
    const r = (j + 0.5) * h;
    for (let i = 0; i < nx; i++) {
      const x = x0 + (i + 0.5) * h;
      let best = 1e30;
      for (const [a, b] of segs) {
        const ex = b[0] - a[0], er = b[1] - a[1], l2 = ex * ex + er * er;
        let t = l2 > 0 ? ((x - a[0]) * ex + (r - a[1]) * er) / l2 : 0; t = Math.min(1, Math.max(0, t));
        const d = Math.hypot(x - (a[0] + t * ex), r - (a[1] + t * er)); if (d < best) best = d;
      }
      out[j * nx + i] = inside(x, r) ? -best : best;
    }
  }
  return out;
}

export class GpuFlow {
  /** `renderer`: the page's WebGLRenderer. `nx` cells along the body, `h` the cell in metres, `nr` cells in the radius. */
  constructor(renderer, { nx = 640, nr = 160, h = 0.25, x0 = 0 } = {}) {
    this.renderer = renderer; this.nx = nx; this.nr = nr; this.h = h; this.x0 = x0;
    this.ok = renderer.capabilities.isWebGL2 && renderer.extensions.has('EXT_color_buffer_float');
    if (!this.ok) return;
    const mk = () => new THREE.WebGLRenderTarget(nx, nr, rtOpts);
    this.rtA = mk(); this.rtA2 = mk(); this.rtB = mk(); this.rtG = mk();
    this.scene = new THREE.Scene(); this.cam = new THREE.OrthographicCamera(-1, 1, 1, -1, 0, 1);
    const quad = new THREE.PlaneGeometry(2, 2);
    this.dist = new THREE.DataTexture(new Float32Array(nx * nr).fill(1), nx, nr, THREE.RedFormat, THREE.FloatType);
    this.dist.minFilter = this.dist.magFilter = THREE.NearestFilter; this.dist.needsUpdate = true;
    const common = () => ({ uN: { value: new THREE.Vector2(nx, nr) }, uH: { value: h }, uInf: { value: new THREE.Vector4(1, 0, 0, 1) }, uD: { value: this.dist } });
    const mat = (frag, extra) => new THREE.ShaderMaterial({ glslVersion: THREE.GLSL3, vertexShader: VERT, fragmentShader: frag, uniforms: { ...common(), ...extra }, depthTest: false, depthWrite: false });
    this.ghostMat = mat(GHOST_FRAG, { uS: { value: null } });
    this.updMat = mat(UPDATE_FRAG, { uG: { value: null }, uS: { value: null }, uU0: { value: null }, uCfl: { value: 0.5 }, uA: { value: 0 }, uB: { value: 1 } });
    this.copyMat = new THREE.ShaderMaterial({ glslVersion: THREE.GLSL3, vertexShader: VERT, fragmentShader: 'precision highp float; precision highp sampler2D; uniform sampler2D t; in vec2 vUv; out vec4 o; void main() { o = texelFetch(t, ivec2(gl_FragCoord.xy), 0); }', uniforms: { t: { value: null } }, depthTest: false });
    this.mesh = new THREE.Mesh(quad, this.ghostMat); this.scene.add(this.mesh);
    this.mach = 3; this.iterations = 0; this.cfl = 0.5;
    this.setMach(3); this.reset();
  }

  /** The body: its distance to every cell centre, from an outline [{x, r}] (the grid starts at x0 on the axis). */
  setBody(profile, x0) {
    if (!this.ok) return;
    this.x0 = x0;
    const d = bodyDistance(profile, x0, this.h, this.nx, this.nr);
    this.dist.image.data.set(d); this.dist.needsUpdate = true;
    this.reset();
  }

  setMach(m) {
    if (!this.ok) return;
    const a = Math.sqrt(GAMMA), u = -m * a;                    // rho = 1, p = 1: the sound speed is sqrt(gamma); the air moves toward -x
    const E = 1 / (GAMMA - 1) + 0.5 * u * u;
    this.inf = new THREE.Vector4(1, u, 0, E);
    for (const mt of [this.ghostMat, this.updMat]) mt.uniforms.uInf.value.copy(this.inf);
    this.mach = m;
  }

  /** The whole grid at the free stream. */
  reset() {
    if (!this.ok) return;
    const d = new Float32Array(this.nx * this.nr * 4);
    for (let i = 0; i < this.nx * this.nr; i++) { d[4 * i] = this.inf.x; d[4 * i + 1] = this.inf.y; d[4 * i + 2] = this.inf.z; d[4 * i + 3] = this.inf.w; }
    const t = new THREE.DataTexture(d, this.nx, this.nr, THREE.RGBAFormat, THREE.FloatType); t.minFilter = t.magFilter = THREE.NearestFilter; t.needsUpdate = true;
    this.copyMat.uniforms.t.value = t;
    this.pass(this.copyMat, this.rtA);
    t.dispose();
    this.iterations = 0;
  }

  pass(material, target) {
    this.mesh.material = material;
    const r = this.renderer, old = r.getRenderTarget();
    r.setRenderTarget(target); r.render(this.scene, this.cam); r.setRenderTarget(old);
  }

  /** `n` iterations (two Runge-Kutta stages each: four passes). */
  step(n) {
    if (!this.ok) return;
    const r = this.renderer, auto = r.autoClear; r.autoClear = false;
    const g = this.ghostMat.uniforms, u = this.updMat.uniforms;
    u.uCfl.value = this.cfl;
    for (let k = 0; k < n; k++) {
      g.uS.value = this.rtA.texture; this.pass(this.ghostMat, this.rtG);                                                   // G = ghosts(A)
      u.uG.value = this.rtG.texture; u.uS.value = this.rtA.texture; u.uU0.value = this.rtA.texture; u.uA.value = 0; u.uB.value = 1; this.pass(this.updMat, this.rtB);   // B = A + dt R(G)
      g.uS.value = this.rtB.texture; this.pass(this.ghostMat, this.rtG);                                                   // G = ghosts(B)
      u.uG.value = this.rtG.texture; u.uS.value = this.rtB.texture; u.uU0.value = this.rtA.texture; u.uA.value = 0.5; u.uB.value = 0.5; this.pass(this.updMat, this.rtA2);   // A' = (A + B + dt R(G)) / 2
      [this.rtA, this.rtA2] = [this.rtA2, this.rtA];
    }
    this.iterations += n; r.autoClear = auto;
  }

  get texture() { return this.rtA.texture; }

  /** The state of every cell (rho, rho u, rho v, E) read back to the page: for a check. */
  read() {
    const d = new Float32Array(this.nx * this.nr * 4);
    this.renderer.readRenderTargetPixels(this.rtA, 0, 0, this.nx, this.nr, d);
    return d;
  }

  dispose() { if (!this.ok) return; for (const t of [this.rtA, this.rtA2, this.rtB, this.rtG, this.dist]) t.dispose(); this.ghostMat.dispose(); this.updMat.dispose(); }
}

/** The section that draws the state: a quad in the plane through the axis, `y` along the body from x0 and `z` across it (the radius both ways). */
export class GpuSection {
  constructor(group, flow) {
    this.flow = flow;
    this.mat = new THREE.ShaderMaterial({
      vertexShader: VIEW_VERT, fragmentShader: VIEW_FRAG, transparent: true, depthWrite: false, side: THREE.DoubleSide,
      uniforms: { uState: { value: null }, uD: { value: flow.dist }, uN: { value: new THREE.Vector2(flow.nx, flow.nr) }, uH: { value: flow.h }, uMode: { value: 0 }, uOpacity: { value: 0.9 }, uMach: { value: 3 }, uPInf: { value: 1 }, uRhoInf: { value: 1 }, uMachHi: { value: 5 } },
    });
    this.mat.uniforms.uN.value = new THREE.Vector2(flow.nx, flow.nr);
    const x0 = flow.x0, x1 = flow.x0 + flow.nx * flow.h, r = flow.nr * flow.h;
    const g = new THREE.BufferGeometry();
    g.setAttribute('position', new THREE.Float32BufferAttribute([0, x0, -r, 0, x1, -r, 0, x1, r, 0, x0, r], 3));
    g.setAttribute('uv', new THREE.Float32BufferAttribute([0, 0, 1, 0, 1, 1, 0, 1], 2));
    g.setIndex([0, 1, 2, 0, 2, 3]);
    this.quad = new THREE.Mesh(g, this.mat); this.quad.frustumCulled = false; this.quad.renderOrder = 3;
    this.pivot = new THREE.Group(); this.pivot.add(this.quad); group.add(this.pivot);
  }
  setVisible(v) { this.pivot.visible = v; }
  update({ mode, opacity, phiDeg }) {
    const u = this.mat.uniforms;
    u.uState.value = this.flow.texture; u.uD.value = this.flow.dist; u.uMode.value = mode; u.uOpacity.value = opacity; u.uMach.value = this.flow.mach; u.uMachHi.value = Math.max(2, 1.5 * this.flow.mach);
    this.pivot.rotation.y = (phiDeg || 0) * Math.PI / 180;
  }
  dispose() { this.pivot.removeFromParent(); this.mat.dispose(); this.quad.geometry.dispose(); }
}
