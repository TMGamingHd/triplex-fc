// What the flight does to the air and the ground: the exhaust plume of each running engine, the trail it leaves, the dust and steam that the engines blow up from the pad, and the stage that has been let go.
// All of it is a picture drawn from the simulator's numbers (the thrust fraction of each engine, the ambient pressure, the altitude, the wind); none of it is computed by the simulator, and each is labelled as such on the page.
import * as THREE from '../vendor/three.module.js';
import { puffTexture, glowTexture } from './models/common.js';

const PLUME_VERT = /* glsl */`
attribute float aS; attribute float aA;
uniform float uLen, uR0, uExp, uTime, uDiam;
varying float vS; varying vec3 vN; varying vec3 vV; varying float vA;
void main() {
  float s = aS;
  float grow = 1.0 + uExp * pow(s, 0.65);
  float diam = 1.0 + uDiam * 0.13 * sin(s * 26.0 - 1.0) * (1.0 - s);
  float w = uR0 * grow * diam * (1.0 - 0.15 * s * s * step(0.0, -uExp));
  vec3 p = vec3(cos(aA) * w, -s * uLen, sin(aA) * w);
  vN = normalize(normalMatrix * vec3(cos(aA), -uExp * uR0 / max(uLen, 1.0) * 0.5, sin(aA)));
  vec4 mv = modelViewMatrix * vec4(p, 1.0);
  vV = normalize(-mv.xyz); vS = s; vA = aA;
  gl_Position = projectionMatrix * mv;
}`;
const PLUME_FRAG = /* glsl */`
precision highp float;
uniform float uTime, uThr, uDens, uDiam, uSeed, uGain;
uniform vec3 uHot, uCool;
varying float vS; varying vec3 vN; varying vec3 vV; varying float vA;
float h(vec2 p) { return fract(sin(dot(p, vec2(127.1, 311.7))) * 43758.5453); }
float n2(vec2 p) { vec2 i = floor(p), f = fract(p); f = f * f * (3. - 2. * f); return mix(mix(h(i), h(i + vec2(1, 0)), f.x), mix(h(i + vec2(0, 1)), h(i + vec2(1, 1)), f.x), f.y); }
void main() {
  float s = vS;
  float fres = pow(abs(dot(normalize(vN), normalize(vV))), 1.1);
  float turb = n2(vec2(vA * 2.0 + uSeed, s * 7.0 - uTime * 9.0)) * 0.6 + n2(vec2(vA * 5.0, s * 15.0 - uTime * 17.0)) * 0.4;
  float core = exp(-s * 3.2);
  float diamonds = uDiam * pow(0.5 + 0.5 * cos(s * 26.0 - 1.0), 6.0) * (1.0 - s) * 1.6;
  float a = (core * 1.0 + (1.0 - s) * 0.5 + diamonds) * fres * (0.5 + 0.7 * turb) * smoothstep(1.0, 0.6, s) * uThr * uDens * uGain;
  vec3 col = mix(uCool, uHot, clamp(core * 1.5 + diamonds * .6, 0.0, 1.0)) * (1.0 + diamonds * 1.6);
  gl_FragColor = vec4(col * a, a * 0.5);
  #include <tonemapping_fragment>
  #include <colorspace_fragment>
}`;

let plumeGeo = null;
function plumeGeometry() {
  if (plumeGeo) return plumeGeo;
  const NS = 22, NA = 20, pos = [], aS = [], aA = [], idx = [];
  for (let i = 0; i <= NS; i++) for (let j = 0; j <= NA; j++) { pos.push(0, 0, 0); aS.push(i / NS); aA.push((j / NA) * Math.PI * 2); }
  for (let i = 0; i < NS; i++) for (let j = 0; j < NA; j++) { const a = i * (NA + 1) + j, b = a + 1, c = a + NA + 1, d = c + 1; idx.push(a, c, b, b, c, d); }
  plumeGeo = new THREE.BufferGeometry();
  plumeGeo.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3));
  plumeGeo.setAttribute('aS', new THREE.Float32BufferAttribute(aS, 1));
  plumeGeo.setAttribute('aA', new THREE.Float32BufferAttribute(aA, 1));
  plumeGeo.setIndex(idx);
  plumeGeo.boundingSphere = new THREE.Sphere(new THREE.Vector3(0, 0, 0), 1e4);
  return plumeGeo;
}

/** The exhaust of every engine of a vehicle model. Each plume is a flared tube of a shader: its length follows the thrust, its width the pressure of the air it leaves into (narrow where the air is dense, a wide soft fan where it is not). */
export class Plumes {
  constructor(model) {
    this.model = model; this.items = [];
    for (const en of model.engines) {
      const mat = new THREE.ShaderMaterial({
        vertexShader: PLUME_VERT, fragmentShader: PLUME_FRAG, transparent: true, depthWrite: false, blending: THREE.CustomBlending, blendEquation: THREE.AddEquation, blendSrc: THREE.OneFactor, blendDst: THREE.OneFactor, side: THREE.DoubleSide,
        uniforms: { uLen: { value: 10 }, uR0: { value: en.exitRadius }, uExp: { value: 0 }, uTime: { value: 0 }, uDiam: { value: 0 }, uThr: { value: 0 }, uDens: { value: 1 }, uSeed: { value: Math.random() * 50 }, uGain: { value: 1 }, uHot: { value: new THREE.Color(1.0, 0.86, 0.62) }, uCool: { value: new THREE.Color(1.0, 0.5, 0.2) } },
      });
      const mesh = new THREE.Mesh(plumeGeometry(), mat);
      mesh.position.y = -en.pivot; mesh.frustumCulled = false; mesh.renderOrder = 5; mesh.visible = false;
      en.holder.add(mesh);
      // a flare at the nozzle exit
      const glow = new THREE.Sprite(new THREE.SpriteMaterial({ map: glowTexture(), color: 0xffd9a8, blending: THREE.AdditiveBlending, depthWrite: false, transparent: true, opacity: 0 }));
      glow.position.y = -en.pivot - en.exitRadius * 0.3; glow.scale.setScalar(en.exitRadius * 6); glow.renderOrder = 6; en.holder.add(glow);
      this.items.push({ en, mesh, mat, glow });
    }
  }
  /** pose.eng: the thrust fraction of each engine (-1 for one that has failed); p_amb in Pa. `exitPressure` is an assumption (the simulator does not model the nozzle's flow): 70 kPa for a sea-level bell, 15 kPa for a vacuum one. */
  update(pose, tWall, opt = {}) {
    const eng = pose.eng || [], pa = pose.air ? pose.air[1] : 101325;
    const stgBits = pose.stg ?? 255;
    let nOn = 0; for (const it of this.items) if ((eng[it.en.index] ?? 0) > 0.02 && (stgBits & (1 << it.en.stage))) nOn++;
    const gain = 1.25 / Math.pow(Math.max(1, nOn), 0.9);
    for (const it of this.items) {
      const en = it.en, f = Math.max(0, eng[en.index] ?? 0);
      const on = f > 0.02 && (stgBits & (1 << en.stage)) !== 0;
      it.mesh.visible = on; it.glow.visible = on;
      if (!on) continue;
      const pe = en.vacuum ? 15000 : 70000, pr = pe / Math.max(pa, 1);
      const exp = THREE.MathUtils.clamp(Math.sqrt(pr) - 1.0, -0.3, 7.0);
      const u = it.mat.uniforms, re = en.exitRadius;
      u.uTime.value = tWall; u.uThr.value = Math.min(1.2, 0.35 + f); u.uExp.value = exp; u.uGain.value = gain;
      u.uLen.value = re * (10 + 14 * f) / (1 + 0.05 * Math.max(0, exp));
      u.uDiam.value = Math.max(0, 1 - Math.abs(Math.log(pr)) * 1.1);            // Mach diamonds show where the nozzle is nearly matched to the air
      u.uDens.value = 0.5 + 0.5 / (1 + 0.2 * Math.max(0, pr - 1));
      it.glow.material.opacity = Math.min(1, f * 0.9) * (opt.glow ?? 1) * Math.min(1, gain * 0.7) / (1 + 0.04 * Math.min(30, Math.max(0, pr - 1))); it.glow.scale.setScalar(re * (5 + 3 * f) * (1 + 0.3 * Math.min(exp, 3)));
    }
  }
  dispose() { for (const it of this.items) { it.mesh.removeFromParent(); it.glow.removeFromParent(); it.mat.dispose(); } }
}

/* ------------------------------------------------------------------------------------------------------------------------------ instanced billboards */
const BB_VERT = /* glsl */`
attribute vec3 aOff; attribute vec4 aData; attribute vec3 aTint;   // aData: size, alpha, rotation, frame
uniform vec3 uRight, uUp;
varying vec2 vUv; varying float vAlpha; varying vec3 vTint;
void main() {
  float c = cos(aData.z), s = sin(aData.z);
  vec2 q = position.xy * aData.x;
  vec2 r = vec2(q.x * c - q.y * s, q.x * s + q.y * c);
  vec3 p = aOff + uRight * r.x + uUp * r.y;
  vUv = uv; vAlpha = aData.y; vTint = aTint;
  gl_Position = projectionMatrix * viewMatrix * vec4(p, 1.0);
}`;
const BB_FRAG = /* glsl */`
precision highp float;
uniform sampler2D uMap; uniform vec3 uLight;
varying vec2 vUv; varying float vAlpha; varying vec3 vTint;
void main() {
  vec4 t = texture2D(uMap, vUv);
  float a = t.a * vAlpha;
  if (a < 0.003) discard;
  gl_FragColor = vec4(vTint * uLight, a);
  #include <tonemapping_fragment>
  #include <colorspace_fragment>
}`;

/** Many camera-facing quads in one draw call: smoke, dust, the vapour of a cloud. */
export class Billboards {
  constructor(n, map, { additive = false, order = 3 } = {}) {
    this.n = n; this.count = 0;
    const g = new THREE.InstancedBufferGeometry();
    const base = new THREE.PlaneGeometry(1, 1);
    g.index = base.index; g.setAttribute('position', base.attributes.position); g.setAttribute('uv', base.attributes.uv);
    this.off = new THREE.InstancedBufferAttribute(new Float32Array(n * 3), 3); this.data = new THREE.InstancedBufferAttribute(new Float32Array(n * 4), 4); this.tint = new THREE.InstancedBufferAttribute(new Float32Array(n * 3), 3);
    for (const a of [this.off, this.data, this.tint]) a.setUsage(THREE.DynamicDrawUsage);
    g.setAttribute('aOff', this.off); g.setAttribute('aData', this.data); g.setAttribute('aTint', this.tint);
    g.instanceCount = 0;
    this.mat = new THREE.ShaderMaterial({ vertexShader: BB_VERT, fragmentShader: BB_FRAG, transparent: true, depthWrite: false, blending: additive ? THREE.AdditiveBlending : THREE.NormalBlending,
      uniforms: { uRight: { value: new THREE.Vector3(1, 0, 0) }, uUp: { value: new THREE.Vector3(0, 1, 0) }, uMap: { value: map }, uLight: { value: new THREE.Vector3(1, 1, 1) } } });
    this.mesh = new THREE.Mesh(g, this.mat); this.mesh.frustumCulled = false; this.mesh.renderOrder = order;
    this.geo = g;
  }
  begin(camera) {
    const m = camera.matrixWorld.elements;
    this.mat.uniforms.uRight.value.set(m[0], m[1], m[2]); this.mat.uniforms.uUp.value.set(m[4], m[5], m[6]);
    this.count = 0;
  }
  add(x, y, z, size, alpha, rot, tr, tg, tb) {
    if (this.count >= this.n) return;
    const i = this.count++;
    this.off.setXYZ(i, x, y, z); this.data.setXYZW(i, size, alpha, rot, 0); this.tint.setXYZ(i, tr, tg, tb);
  }
  end() { this.geo.instanceCount = this.count; this.off.needsUpdate = this.data.needsUpdate = this.tint.needsUpdate = true; }
}

/* ------------------------------------------------------------------------------------------------------------------------------ the trail */
/** The exhaust trail behind the vehicle: points where the tail has been, drawn as a ribbon that widens and fades with age, drifting with the wind. It shows where the air is dense enough for the exhaust to show. */
export class Trail {
  constructor(max = 900) {
    this.max = max; this.pts = [];                    // {p: [x,y,z] planet-centred scene metres, t: vehicle time, w: initial width, a: initial alpha, wind: [x,y,z]}
    const g = new THREE.BufferGeometry();
    this.pos = new THREE.BufferAttribute(new Float32Array(max * 2 * 3), 3).setUsage(THREE.DynamicDrawUsage);
    this.col = new THREE.BufferAttribute(new Float32Array(max * 2 * 4), 4).setUsage(THREE.DynamicDrawUsage);
    const idx = []; for (let i = 0; i < max - 1; i++) { const a = 2 * i; idx.push(a, a + 1, a + 2, a + 1, a + 3, a + 2); }
    g.setAttribute('position', this.pos); g.setAttribute('color', this.col); g.setIndex(idx); g.setDrawRange(0, 0);
    g.boundingSphere = new THREE.Sphere(new THREE.Vector3(), 1e7);
    this.mesh = new THREE.Mesh(g, new THREE.MeshBasicMaterial({ vertexColors: true, transparent: true, depthWrite: false, side: THREE.DoubleSide, fog: false }));
    this.mesh.frustumCulled = false; this.mesh.renderOrder = 2; this.geo = g; this.lastT = -1e9;
  }
  clear() { this.pts.length = 0; this.lastT = -1e9; }
  emit(t, pos, width, alpha, wind) {
    if (t < this.lastT - 1e-6) this.clear();
    if (t - this.lastT < 0.05) return;
    this.lastT = t;
    this.pts.push({ p: pos.slice(), t, w: width, a: alpha, wind: wind.slice() });
    if (this.pts.length > this.max) this.pts.shift();
  }
  /** rS: the vehicle's place, planet-centred scene metres; camPos: the camera, relative to the vehicle. */
  update(t, rS, camPos, tint = [1, 1, 1], life = 40) {
    const n = this.pts.length;
    if (n < 2) { this.geo.setDrawRange(0, 0); return; }
    let k = 0;
    const P = this.pos.array, C = this.col.array;
    const v = new THREE.Vector3(), nxt = new THREE.Vector3(), dir = new THREE.Vector3(), side = new THREE.Vector3(), toCam = new THREE.Vector3();
    for (let i = 0; i < n; i++) {
      const q = this.pts[i], age = Math.max(0, t - q.t);
      if (age > life) continue;
      const drift = Math.min(age, 25) * 0.8;
      v.set(q.p[0] + q.wind[0] * drift - rS[0], q.p[1] + q.wind[1] * drift - rS[1], q.p[2] + q.wind[2] * drift - rS[2]);
      const j = Math.min(n - 1, i + 1), r = this.pts[j], ra = Math.max(0, t - r.t), rd = Math.min(ra, 25) * 0.8;
      nxt.set(r.p[0] + r.wind[0] * rd - rS[0], r.p[1] + r.wind[1] * rd - rS[1], r.p[2] + r.wind[2] * rd - rS[2]);
      if (j === i) dir.set(0, 1, 0); else dir.subVectors(nxt, v);
      if (dir.lengthSq() < 1e-9) dir.set(0, 1, 0);
      dir.normalize(); toCam.subVectors(camPos, v).normalize();
      side.crossVectors(dir, toCam); if (side.lengthSq() < 1e-8) side.set(1, 0, 0); side.normalize();
      const w = q.w * (1 + age * 0.55), fade = Math.pow(1 - age / life, 1.4) * q.a;
      P[6 * k] = v.x - side.x * w; P[6 * k + 1] = v.y - side.y * w; P[6 * k + 2] = v.z - side.z * w;
      P[6 * k + 3] = v.x + side.x * w; P[6 * k + 4] = v.y + side.y * w; P[6 * k + 5] = v.z + side.z * w;
      const e = 1 - Math.min(1, age / 1.2) * 0.0, al = fade * (0.28 + 0.25 * Math.exp(-age / 3));
      for (let s = 0; s < 2; s++) { C[8 * k + 4 * s] = tint[0]; C[8 * k + 4 * s + 1] = tint[1]; C[8 * k + 4 * s + 2] = tint[2]; C[8 * k + 4 * s + 3] = al * e * (s === 0 ? 0.7 : 0.7); }
      k++;
    }
    this.pos.needsUpdate = this.col.needsUpdate = true;
    // k points make k - 1 segments of 6 indices; the strip's last pair has no successor
    this.geo.setDrawRange(0, Math.max(0, (k - 1) * 6));
  }
}

/* ------------------------------------------------------------------------------------------------------------------------------ dust and steam at lift-off */
/** The cloud that the engines blow up from the pad: puffs born on the ground round the vehicle while it is low, spreading outward and growing. Positions are in the pad's frame (x crossrange, y up, z downrange). */
export class PadSmoke {
  constructor(n = 260) {
    this.n = n; this.p = []; this.bb = new Billboards(n, puffTexture(128, 5), { order: 4 }); this.acc = 0; this.rng = (() => { let s = 12345; return () => (s = (s * 1664525 + 1013904223) >>> 0) / 4294967296; })();
  }
  clear() { this.p.length = 0; this.acc = 0; }
  update(dt, t, st, camera, padToNear, light) {
    // st: {alt (m above the pad's mount), thrust (0..1), radius (the body radius), padRel: Vector3 of the pad in the near space, quat of the pad}
    if (st.alt < 260 && st.thrust > 0.05 && dt > 0) {
      this.acc += dt * (60 + 200 * st.thrust) * Math.max(0.15, 1 - st.alt / 260);
      while (this.acc >= 1) {
        this.acc -= 1;
        const a = this.rng() * Math.PI * 2, r0 = st.radius * (0.4 + 1.3 * this.rng()), sp = (8 + 38 * this.rng()) * (0.5 + st.thrust);
        const shade = 0.78 + 0.22 * this.rng();
        this.p.push({ x: Math.cos(a) * r0, y: 1 + 3 * this.rng(), z: Math.sin(a) * r0, vx: Math.cos(a) * sp, vy: 2 + 8 * this.rng(), vz: Math.sin(a) * sp, s0: 12 + 20 * this.rng(), age: 0, life: 10 + 14 * this.rng(), rot: this.rng() * 6.28, shade, warm: this.rng() < 0.4 });
      }
    }
    this.bb.begin(camera);
    const e = padToNear.elements;
    for (let i = this.p.length - 1; i >= 0; i--) {
      const q = this.p[i];
      q.age += dt; if (q.age > q.life) { this.p.splice(i, 1); continue; }
      const damp = Math.exp(-dt * 0.35);
      q.vx *= damp; q.vz *= damp; q.vy += (1 - q.y / 400) * 1.2 * dt; q.vy *= Math.exp(-dt * 0.15);
      q.x += q.vx * dt; q.y += q.vy * dt; q.z += q.vz * dt;
      const k = q.age / q.life, size = q.s0 * (1 + 5.5 * Math.sqrt(k)) * (1 + 0.3 * k), alpha = 0.55 * Math.pow(1 - k, 1.3) * Math.min(1, q.age * 1.5);
      const X = e[0] * q.x + e[4] * q.y + e[8] * q.z + e[12], Y = e[1] * q.x + e[5] * q.y + e[9] * q.z + e[13], Z = e[2] * q.x + e[6] * q.y + e[10] * q.z + e[14];
      const tintR = q.warm ? 1.0 : q.shade, tintG = q.warm ? 0.88 * q.shade : q.shade, tintB = q.warm ? 0.7 * q.shade : q.shade * 1.02;
      this.bb.add(X, Y, Z, size, alpha, q.rot + k * 0.4, tintR, tintG, tintB);
    }
    this.bb.mat.uniforms.uLight.value.set(light[0], light[1], light[2]);
    this.bb.end();
  }
}

/* ------------------------------------------------------------------------------------------------------------------------------ a stage that has been let go */
/** A stage after separation: carried on with the speed it had, pushed back a little, falling under gravity and slowed by the air, turning slowly. The simulator does not track what it lets go; this is the picture of it, not a result. */
export class Debris {
  constructor(group, rS, vS, qS, pushBack, tipoff, spec) {
    this.group = group; this.r = rS.slice(); this.v = vS.slice(); this.q = qS.clone(); this.spec = spec;
    const ax = new THREE.Vector3(0, 1, 0).applyQuaternion(qS);
    this.v = this.v.map((x, i) => x - pushBack * ax.getComponent(i));
    this.w = tipoff;                                // rad/s about the scene axes
    this.t = 0;
  }
  step(dt, mu, air) {
    const rn = Math.hypot(...this.r), g = mu / (rn * rn);
    const sp = Math.hypot(...this.v), cd = 0.7, area = Math.PI * this.spec.r * this.spec.r * 3, rho = air(rn);
    const drag = 0.5 * rho * sp * cd * area / Math.max(this.spec.m, 1000);
    for (let i = 0; i < 3; i++) { this.v[i] += (-g * this.r[i] / rn - drag * this.v[i]) * dt; this.r[i] += this.v[i] * dt; }
    const dq = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(...this.w).normalize(), Math.hypot(...this.w) * dt);
    this.q.premultiply(dq); this.t += dt;
  }
}
