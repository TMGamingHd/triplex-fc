// The flow the Euler solver computed (tfc_cfd, docs/design/CFD.md), as the viewer shows it: a section through the flow in the vehicle's plane of symmetry (the Mach number, the pressure, or a schlieren picture, which is the
// gradient of the density the way a shadowgraph shows a shock), and the pressure on the skin. The files are the solver's own (console/cfd/, console/demo/); the viewer picks the one nearest the flight's Mach number
// and angle of attack. None of it is made up here and none of it is computed here: the page only decodes bytes to colours.
import * as THREE from '../vendor/three.module.js';
import { COLORMAP_GLSL } from './gfx.js';

const token = () => sessionStorage.getItem('tfc.token') || '';

/** The list of the flow fields on the server: {present, cases: [{name, vehicle, stage_from, mach, alpha_deg, ca_skin, cn, x_cp_m, cells, iterations, h_m, ...}]}. */
export async function listCases() {
  try {
    const r = await fetch('/api/viewer/cfd', { headers: { 'X-TFC-Token': token() } });
    if (r.ok) return await r.json();
  } catch { /* no server, no list */ }
  return { present: false, cases: [] };
}

/** Decodes a flow-field file: a JSON line (padded to a multiple of four bytes), the pressure coefficients of the skin (float32, station by station), then three volumes of 16-bit numbers, 0 for "inside the body"
 *  (the pressure ratio and the density ratio on log scales, the Mach number). */
export function parseCfd(buf) {
  const u8 = new Uint8Array(buf);
  const nl = u8.indexOf(10);
  const head = JSON.parse(new TextDecoder().decode(u8.subarray(0, nl)));
  const ns = head.surface.stations, na = head.surface.angles, v = head.volume, n = v.nx * v.ny * v.nz;
  let off = nl + 1;
  const cp = new Float32Array(buf.slice(off, off + ns * na * 4)); off += ns * na * 4;
  const u16 = new Uint16Array(buf.slice(off, off + 6 * n));
  const vol = { p: u16.subarray(0, n), rho: u16.subarray(n, 2 * n), mach: u16.subarray(2 * n, 3 * n) };
  return { head, cp, vol };
}

export async function loadCase(name) {
  const r = await fetch('/api/viewer/cfd/file?name=' + encodeURIComponent(name), { headers: { 'X-TFC-Token': token() } });
  if (!r.ok) throw new Error('cannot load ' + name + ': ' + r.status);
  return parseCfd(await r.arrayBuffer());
}

/** The case nearest a flight condition among those of a vehicle (and of the stages it still has): the Mach number in its logarithm and the angle of attack in tenths of a degree count the same. */
export function nearest(cases, vehicle, stageFrom, mach, alphaDeg) {
  let best = null, bd = Infinity;
  for (const c of cases) {
    if (vehicle && c.vehicle && c.vehicle !== vehicle) continue;
    if (c.stage_from != null && stageFrom != null && c.stage_from !== stageFrom) continue;
    const d = Math.abs(Math.log(Math.max(mach, 0.3) / Math.max(c.mach, 0.3))) * 3 + Math.abs(Math.abs(alphaDeg) - c.alpha_deg) / 6;
    if (d < bd) { bd = d; best = c; }
  }
  return best;
}

/* ------------------------------------------------------------------------------------------------------------------------------ the section through the flow */
const SLICE_VERT = /* glsl */`varying vec2 vUv; void main() { vUv = uv; gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0); }`;
const SLICE_FRAG = /* glsl */`
precision highp float;
${COLORMAP_GLSL}
varying vec2 vUv;
uniform sampler2D uTex;            // R: the Mach number, G: the pressure ratio, B: the density ratio, as floats in 0..1 (0 in A: inside the body)
uniform vec2 uTexel;               // the size of a texel in cells
uniform vec2 uLogP, uLogRho;       // the scales of the pressure and the density bytes
uniform float uMachHi, uMode, uOpacity, uH, uMach0;
float dec(float b, float lo, float hi) { return lo + (b * 65535.0 - 1.0) / 65534.0 * (hi - lo); }
float lnrho(vec2 uv) { return dec(texture2D(uTex, uv).b, uLogRho.x, uLogRho.y) * 0.6931472; }
void main() {
  vec4 t = texture2D(uTex, vUv);
  if (t.a < 0.5) discard;
  vec3 col; float a = uOpacity;
  if (uMode < 0.5) {                                              // the Mach number: blue is slow, red is fast; the undisturbed stream is left clear
    float m = dec(t.r, 0.0, uMachHi);
    col = cmap(m / uMachHi);
    a *= smoothstep(0.02, 0.14, abs(m - uMach0) / max(uMach0, 1.0));
  } else if (uMode < 1.5) {                                       // the pressure over the free stream's, on a logarithmic scale from a tenth to a hundred
    float lp = dec(t.g, uLogP.x, uLogP.y);
    col = cmap((lp * 0.30103 + 1.0) / 3.0);
    a *= smoothstep(0.012, 0.09, abs(lp) * 0.30103);
  } else {                                                        // schlieren: the gradient of the density, as a knife edge shows it (the shocks light, the expansions dark)
    vec2 d = uTexel;
    vec2 g = vec2(lnrho(vUv + vec2(d.x, 0.0)) - lnrho(vUv - vec2(d.x, 0.0)), lnrho(vUv + vec2(0.0, d.y)) - lnrho(vUv - vec2(0.0, d.y))) / (2.0 * uH);
    float strength = clamp(length(g) * 12.0, 0.0, 1.0);
    float s = 0.5 + 0.5 * clamp(g.x * 9.0 + g.y * 3.0, -1.0, 1.0);
    col = vec3(mix(0.02, 1.0, s)); a *= strength;
  }
  gl_FragColor = vec4(col, a);
  #include <tonemapping_fragment>
  #include <colorspace_fragment>
}`;

const SKIN_TEX = (cfd, bins = 512) => {
  // the skin's Cp on a grid of the body's length (u) and the angle round it (v): resampled from the solver's stations, which are not evenly spaced along the length
  const { head, cp } = cfd, ns = head.surface.stations, na = head.surface.angles, sx = head.surface.x, x0 = head.body.x0, x1 = head.body.x1;
  const data = new Float32Array(bins * na);
  let k = 0;
  for (let i = 0; i < bins; i++) {
    const x = x0 + (x1 - x0) * (i + 0.5) / bins;
    while (k + 2 < ns && sx[k + 1] < x) k++;
    const a = sx[k], b = sx[Math.min(k + 1, ns - 1)], f = b > a ? Math.min(1, Math.max(0, (x - a) / (b - a))) : 0;
    for (let j = 0; j < na; j++) data[j * bins + i] = cp[k * na + j] * (1 - f) + cp[Math.min(k + 1, ns - 1) * na + j] * f;
  }
  const t = new THREE.DataTexture(data, bins, na, THREE.RedFormat, THREE.FloatType);
  t.magFilter = THREE.LinearFilter; t.minFilter = THREE.LinearFilter; t.wrapS = t.wrapT = THREE.ClampToEdgeWrapping; t.needsUpdate = true;
  return t;
};

export class CfdView {
  constructor(group) {
    this.group = group;                                     // the lens's group, in the vehicle's own frame (y along the axis from the tail, z across the plane of the pitch, x the other way)
    this.cfd = null; this.name = ''; this.mode = 0; this.opacity = 0.9; this.k = 0;
    this.tex = new THREE.DataTexture(new Uint8Array(4), 1, 1); this.tex.needsUpdate = true;
    this.mat = new THREE.ShaderMaterial({
      vertexShader: SLICE_VERT, fragmentShader: SLICE_FRAG, transparent: true, depthWrite: false, side: THREE.DoubleSide,
      uniforms: { uTex: { value: this.tex }, uTexel: { value: new THREE.Vector2(1, 1) }, uLogP: { value: new THREE.Vector2(-3, 8) }, uLogRho: { value: new THREE.Vector2(-3, 6) }, uMachHi: { value: 5 }, uMode: { value: 0 }, uOpacity: { value: 0.9 }, uH: { value: 1 }, uMach0: { value: 3 } },
    });
    this.quad = new THREE.Mesh(new THREE.BufferGeometry(), this.mat); this.quad.frustumCulled = false; this.quad.renderOrder = 3; this.quad.visible = false;
    this.pivot = new THREE.Group(); this.pivot.add(this.quad); group.add(this.pivot);
    this.skinTex = null;
  }

  setVisible(v) { this.pivot.visible = v; }
  get nz() { return this.cfd ? this.cfd.head.volume.nz : 1; }

  /** Makes a case the one that is shown. */
  set(cfd, name) {
    this.cfd = cfd; this.name = name;
    const v = cfd.head.volume;
    // the quad: in the vehicle's frame the section is the plane through the axis (model y) and the pitch direction (model z): x = 0
    const x0 = v.x0, x1 = v.x0 + v.nx * v.h, y0 = v.y0, y1 = v.y0 + v.ny * v.h;
    const g = new THREE.BufferGeometry();
    g.setAttribute('position', new THREE.Float32BufferAttribute([0, x0, y0, 0, x1, y0, 0, x1, y1, 0, x0, y1], 3));
    g.setAttribute('uv', new THREE.Float32BufferAttribute([0, 0, 1, 0, 1, 1, 0, 1], 2));
    g.setIndex([0, 1, 2, 0, 2, 3]);
    this.quad.geometry.dispose(); this.quad.geometry = g;
    const u = this.mat.uniforms;
    u.uLogP.value.set(v.log2_p[0], v.log2_p[1]); u.uLogRho.value.set(v.log2_rho[0], v.log2_rho[1]); u.uMachHi.value = v.mach[1]; u.uH.value = v.h; u.uMach0.value = cfd.head.mach;
    u.uTexel.value.set(1 / v.nx, 1 / v.ny);
    if (this.skinTex) this.skinTex.dispose();
    this.skinTex = SKIN_TEX(cfd);
    this.slice(0);
  }

  /** The texture of the section at depth `k` (cells from the plane of symmetry). Cells inside the body take the colour of the fluid beside them so that the filtering does not darken the edge of the body. */
  slice(k) {
    if (!this.cfd) return;
    const v = this.cfd.head.volume, { vol } = this.cfd, nx = v.nx, ny = v.ny;
    this.k = Math.max(0, Math.min(v.nz - 1, Math.round(k)));
    const base = this.k * nx * ny, d = new Float32Array(nx * ny * 4);
    for (let i = 0; i < nx * ny; i++) {
      d[4 * i] = vol.mach[base + i] / 65535; d[4 * i + 1] = vol.p[base + i] / 65535; d[4 * i + 2] = vol.rho[base + i] / 65535; d[4 * i + 3] = vol.p[base + i] ? 1 : 0;
    }
    for (let pass = 0; pass < 3; pass++) {          // fill the body's cells with their neighbours' values (three cells deep), so that the filtering does not darken the edge of the body
      const src = d.slice();
      for (let j = 0; j < ny; j++) for (let i = 0; i < nx; i++) {
        const c = j * nx + i; if (src[4 * c + 3]) continue;
        let n = 0, r = 0, g = 0, b = 0;
        for (const [di, dj] of [[1, 0], [-1, 0], [0, 1], [0, -1]]) { const ii = i + di, jj = j + dj; if (ii < 0 || jj < 0 || ii >= nx || jj >= ny) continue; const q = jj * nx + ii; if (src[4 * q + 3] || (pass > 0 && src[4 * q] > 0)) { r += src[4 * q]; g += src[4 * q + 1]; b += src[4 * q + 2]; n++; } }
        if (n) { d[4 * c] = r / n; d[4 * c + 1] = g / n; d[4 * c + 2] = b / n; }
      }
    }
    const t = new THREE.DataTexture(d, nx, ny, THREE.RGBAFormat, THREE.FloatType);
    t.magFilter = THREE.LinearFilter; t.minFilter = THREE.LinearFilter; t.wrapS = t.wrapT = THREE.ClampToEdgeWrapping; t.needsUpdate = true;
    if (this.tex) this.tex.dispose();
    this.tex = t; this.mat.uniforms.uTex.value = t;
    this.quad.visible = true;
  }

  /** Per frame: the mode and opacity, and the turn of the section into the plane the flight's angle of attack is in (the case's windward side is its +y). */
  update({ mode, opacity, phiDeg }) {
    const u = this.mat.uniforms;
    u.uMode.value = mode; u.uOpacity.value = opacity;
    this.pivot.rotation.y = (phiDeg || 0) * Math.PI / 180;      // model y is the axis: from the pitch direction (z) toward the other lateral one (x)
  }

  dispose() { this.pivot.removeFromParent(); this.mat.dispose(); this.quad.geometry.dispose(); if (this.tex) this.tex.dispose(); if (this.skinTex) this.skinTex.dispose(); }
}
