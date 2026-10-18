// A check of the live flow solver (js/gpuflow.js) on the page itself: the flow over a cone at zero angle of attack is the axisymmetric problem of Taylor and Maccoll, whose answer the offline solver's
// gasdyn.hpp also gives (the shock angle and the surface pressure), so the shaders can be compared with it. The numbers go to the page and to the console server's log (?token=...).
import * as THREE from './vendor/three.module.js';
import { GpuFlow, bodyDistance } from './js/gpuflow.js';

const o = document.getElementById('o');
{ const img = new Image(); img.src = '/api/hold?ms=' + (new URLSearchParams(location.search).get('hold') || '15000') + '&token=' + (new URLSearchParams(location.search).get('token') || ''); document.body.append(img); }     // the screenshot waits for the load event: this holds it
const tok = new URLSearchParams(location.search).get('token') || '';
const say = (t) => { o.textContent += t + '\n'; try { fetch('/api/viewer/log', { method: 'POST', headers: { 'X-TFC-Token': tok, 'Content-Type': 'application/json' }, body: JSON.stringify({ text: 'gputest ' + t }) }); } catch { /* the page has it */ } };
window.addEventListener('error', (e) => say('WINERR ' + e.message + ' @' + e.filename + ':' + e.lineno));

const renderer = new THREE.WebGLRenderer({ canvas: document.getElementById('c') });
say('webgl2 ' + renderer.capabilities.isWebGL2 + ', color_buffer_float ' + renderer.extensions.has('EXT_color_buffer_float'));

const cases = [{ mach: 2.5, deg: 15, ps: 1.80519, beta: 28.4546 }, { mach: 5.0, deg: 10, ps: 2.30831, beta: 15.6083 }];
const q = new URLSearchParams(location.search);
const iters = +(q.get('iters') || 3000), h = +(q.get('h') || 0.03);
for (const c of cases) {
  const th = c.deg * Math.PI / 180, L = 4, rb = L * Math.tan(th);
  const x0 = -1.5, nx = Math.round((L + 3.0) / h), nr = Math.round(3.0 / h);
  const prof = [{ x: 0, r: 0 }, { x: 0, r: rb }, { x: L, r: 0 }];      // the nose is at the largest x (the air moves toward -x): a flat base at x = 0, the tip at x = L
  const flow = new GpuFlow(renderer, { nx, nr, h, x0 });
  if (!flow.ok) { say('no float rendering'); break; }
  flow.setMach(c.mach); flow.setBody(prof, x0);
  const t0 = performance.now();
  for (let k = 0; k < iters; k += 100) flow.step(100);
  const d = flow.read(); const ms = performance.now() - t0;                 // (reading back waits for the card: the time is of the whole run)
  const sd = bodyDistance(prof, x0, h, nx, nr), idx = (i, j) => j * nx + i;
  // the pressure on the first layer of fluid cells above the cone's side between 0.35 and 0.75 of its length, and the shock radius at 0.9 of the length
  let sum = 0, n = 0;
  for (let i = 0; i < nx; i++) { const x = x0 + (i + 0.5) * h, f = (L - x) / L; if (f < 0.35 || f > 0.75) continue; for (let j = 0; j < nr; j++) { const dd = sd[idx(i, j)]; if (dd > 0.5 * h && dd < 1.5 * h) { const U = [d[4 * idx(i, j)], d[4 * idx(i, j) + 1], d[4 * idx(i, j) + 2], d[4 * idx(i, j) + 3]]; const p = 0.4 * (U[3] - 0.5 * (U[1] * U[1] + U[2] * U[2]) / U[0]); sum += p; n++; break; } } }
  const is = Math.round((0.1 * L - x0) / h), xs = x0 + (is + 0.5) * h, along = L - xs;      // the section 0.9 of the length from the tip
  let rs = 0; for (let j = nr - 1; j >= 0; j--) { const k = 4 * idx(is, j), U = [d[k], d[k + 1], d[k + 2], d[k + 3]]; const p = 0.4 * (U[3] - 0.5 * (U[1] * U[1] + U[2] * U[2]) / U[0]); if (p > 1 + 0.5 * (c.ps - 1)) { rs = (j + 0.5) * h; break; } }
  const bad = d.some((v) => !Number.isFinite(v));
  say(`M ${c.mach} cone ${c.deg} deg, ${nx}x${nr} cells, ${iters} iterations in ${ms.toFixed(0)} ms: surface pressure ${(sum / n).toFixed(4)} (Taylor-Maccoll ${c.ps}, ${(100 * (sum / n / c.ps - 1)).toFixed(2)} %), shock angle ${(Math.atan(rs / along) * 180 / Math.PI).toFixed(2)} deg (${c.beta}), finite ${!bad}`);
  flow.dispose();
}
say('done');
