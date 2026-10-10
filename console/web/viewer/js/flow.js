// The air around the vehicle, drawn from what the simulator says it is doing: the pressure on its skin, the streamlines past it, the shock in front of it.
//
// What each is, honestly (the page says the same). The pose gives the Mach number, the angle of attack, the air's pressure and temperature, and the force on the vehicle. The simulator does not solve the flow.
//   * The pressure map is the *modified Newtonian* estimate: the pressure coefficient on a surface that faces the wind is Cp_max cos^2 of the angle between its normal and the wind, and a small suction on the lee side;
//     Cp_max is the stagnation value for the Mach number (Rayleigh's pitot formula). A hypersonic method, used here at every speed: it says where the pressure is high, not how high below Mach 3.
//   * The streamlines are slender-body potential flow (a line of sources for the growth of the section, a doublet for the cross-flow) in the cross-section of every station, from the vehicle's own radius profile;
//     in supersonic flow the disturbance is confined to inside the shock.
//   * The shock is the attached conical shock of the Taylor-Maccoll solution for the nose's cone angle at the Mach number, tilted with the angle of attack; where no attached solution exists it is drawn as the Mach cone.
// None of it is a CFD result. A CFD field from a solver can replace any of them (docs/design/VIEWER.md, "True CFD later").
import * as THREE from '../vendor/three.module.js';
import { COLORMAP_GLSL, colormap } from './gfx.js';
import { coneShockCached, cpStagnation, DEG, RAD } from './physics.js';
import { Billboards } from './fx.js';
import { puffTexture } from './models/common.js';

/* ------------------------------------------------------------------------------------------------------------------------------ the pressure on the skin */
const CP_VERT = /* glsl */`
varying vec3 vN; varying vec3 vP;
void main() { vN = normalize(mat3(modelMatrix) * normal); vP = position; gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0); }`;
const CP_FRAG = /* glsl */`
precision highp float;
${COLORMAP_GLSL}
varying vec3 vN; varying vec3 vP;
uniform vec3 uF; uniform float uMach, uCpMax, uMode, uOpacity, uQ, uT, uT0, uTime, uBands;
void main() {
  vec3 n = normalize(vN);
  float c = -dot(n, uF);                                  // 1 where the surface faces the wind
  float cp = c > 0.0 ? uCpMax * c * c : -0.05 - 0.3 / (1.0 + 0.5 * uMach * uMach) * (1.0 - c * c * 0.0) * min(1.0, -c * 2.0 + 0.3);
  float t;
  if (uMode < 1.5) { t = cp < 0.0 ? 0.5 + 0.5 * cp / 0.5 : 0.5 + 0.5 * cp / max(uCpMax, 1.0); }
  else { float f = c > 0.0 ? sqrt(c) : 0.0; float Tw = uT + (uT0 - uT) * (0.9 * f + 0.08); t = (Tw - uT) / max(uT0 - uT, 1.0) * 0.5 + 0.5 * step(0.0, uT0 - uT - 1.0) * (Tw - uT) / max(uT0 - uT, 1.0); }
  vec3 col = cmap(clamp(t, 0.0, 1.0));
  float iso = uBands > 0.5 ? 0.12 * smoothstep(0.42, 0.5, abs(fract(t * 10.0) - 0.5)) : 0.0;   // contour lines
  col *= 1.0 - iso;
  float a = uOpacity * smoothstep(5.0, 400.0, uQ) * mix(0.35, 1.0, smoothstep(0.015, 0.2, abs(t - 0.5) * 2.0));
  gl_FragColor = vec4(col, a);
  #include <tonemapping_fragment>
  #include <colorspace_fragment>
}`;

export class CpSurface {
  constructor(model) {
    this.model = model;
    this.mat = new THREE.ShaderMaterial({
      vertexShader: CP_VERT, fragmentShader: CP_FRAG, transparent: true, depthWrite: false, polygonOffset: true, polygonOffsetFactor: -2, polygonOffsetUnits: -2,
      uniforms: { uF: { value: new THREE.Vector3(0, -1, 0) }, uMach: { value: 0 }, uCpMax: { value: 1 }, uMode: { value: 0 }, uOpacity: { value: 0.85 }, uQ: { value: 0 }, uT: { value: 250 }, uT0: { value: 300 }, uTime: { value: 0 }, uBands: { value: 1 } },
    });
    // an overlay on every surface of the vehicle (the body, the tiles, the fins, a model that was imported), as a child of the surface, so that it has the same place; it reads the normals in the world's axes
    this.meshes = [];
    const targets = [];
    const underEngine = (o) => { for (let p = o.parent; p; p = p.parent) if (p.name && p.name.startsWith('engine')) return true; return false; };
    const collect = (root) => root.traverse((o) => { if (o.isMesh && o.geometry && o.geometry.attributes.normal && !o.userData.noCp && !o.material.isShaderMaterial && !underEngine(o)) targets.push(o); });
    model.stages.forEach((s) => collect(s.group)); model.payloads.forEach((p) => collect(p.group));
    for (const t of targets) { const m = new THREE.Mesh(t.geometry, this.mat); m.renderOrder = 1; m.userData.noCp = true; t.add(m); this.meshes.push(m); }
    this.model = model;
    this.setVisible(false);
  }
  setVisible(v) { for (const m of this.meshes) m.visible = v; this.visible = v; }
  /** mode 0: pressure coefficient, 1: the same as static pressure (the legend says kPa), 2: recovery temperature */
  update(d, mode, opacity) {
    const u = this.mat.uniforms;
    this.model.root.updateWorldMatrix(true, false);
    u.uF.value.set(d.fhat[2], d.fhat[0], d.fhat[1]).transformDirection(this.model.root.matrixWorld);   // the body frame's (x, y, z) in the model's axes, then in the world's
    u.uMach.value = d.M; u.uCpMax.value = d.cpMax; u.uMode.value = mode; u.uOpacity.value = opacity; u.uQ.value = d.q; u.uT.value = d.T; u.uT0.value = d.T0;
  }
  dispose() { for (const m of this.meshes) m.removeFromParent(); this.mat.dispose(); }
}

/* ------------------------------------------------------------------------------------------------------------------------------ the streamlines */
const SL_VERT = /* glsl */`
attribute vec3 aColor; attribute float aDist;
varying vec3 vC; varying float vD;
void main() { vC = aColor; vD = aDist; gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0); }`;
const SL_FRAG = /* glsl */`
precision highp float;
uniform float uTime, uOpacity, uSpeed;
varying vec3 vC; varying float vD;
void main() {
  float s = fract(vD * 0.18 - uTime * uSpeed);
  float dash = 0.35 + 0.65 * smoothstep(0.0, 0.2, s) * (1.0 - smoothstep(0.55, 0.9, s));
  gl_FragColor = vec4(vC * (0.7 + 0.8 * dash), uOpacity * dash);
  #include <tonemapping_fragment>
  #include <colorspace_fragment>
}`;

/** The streamlines past the vehicle in its own (model) frame: recomputed when the wind direction, the Mach number or the stages on the vehicle change. */
export class Streamlines {
  constructor(model) {
    this.model = model; this.key = '';
    this.geo = new THREE.BufferGeometry();
    this.mat = new THREE.ShaderMaterial({ vertexShader: SL_VERT, fragmentShader: SL_FRAG, transparent: true, depthWrite: false, uniforms: { uTime: { value: 0 }, uOpacity: { value: 0.6 }, uSpeed: { value: 1 } } });
    this.obj = new THREE.LineSegments(this.geo, this.mat); this.obj.frustumCulled = false; this.obj.renderOrder = 8;
    model.root.add(this.obj);
    this.lastBuild = -1;
  }
  setVisible(v) { this.obj.visible = v; }
  /** Radius of the body at x (m from the tail) for the stages that are on the vehicle; 0 elsewhere. Also the slope. */
  profile(stgBits) {
    const segs = [];
    this.model.stages.forEach((s, i) => { if (stgBits & (1 << i)) for (let k = 0; k < s.profile.length - 1; k++) segs.push([s.profile[k].x, s.profile[k].r, s.profile[k + 1].x, s.profile[k + 1].r]); });
    segs.sort((a, b) => a[0] - b[0]);
    return segs;
  }
  rOf(segs, x) {
    for (let i = 0; i < segs.length; i++) { const s = segs[i]; if (x >= s[0] && x <= s[2]) { const dx = s[2] - s[0]; if (dx < 1e-9) return [s[1], 0]; const f = (x - s[0]) / dx; return [s[1] + (s[3] - s[1]) * f, (s[3] - s[1]) / dx]; } }
    return [0, 0];
  }
  update(d, stgBits, spec, tWall, wallDt) {
    this.mat.uniforms.uTime.value = tWall;
    if (!this.obj.visible) return;
    const key = [Math.round(d.fhat[0] * 400), Math.round(d.fhat[1] * 400), Math.round(d.fhat[2] * 400), Math.round(d.M * 20), stgBits].join(',');
    if (key === this.key || tWall - this.lastBuild < 0.1) return;
    this.key = key; this.lastBuild = tWall;
    this.build(d, stgBits, spec);
  }
  build(d, stgBits, spec) {
    const segs = this.profile(stgBits);
    if (!segs.length) { this.geo.setDrawRange(0, 0); return; }
    const x0 = segs[0][0], x1 = segs[segs.length - 1][2], L = x1 - x0, Rmax = Math.max(...segs.map((s) => Math.max(s[1], s[3])));
    const fb = d.fhat;                                          // the direction the air moves, body frame (x forward, y, z)
    const f = new THREE.Vector3(fb[2], fb[0], fb[1]).normalize(); // in the model's axes: Y along the vehicle
    const ax = new THREE.Vector3(0, 1, 0);
    // a basis of the plane the seeds are in: e1 along the vehicle's projection on it
    let e1 = ax.clone().sub(f.clone().multiplyScalar(ax.dot(f)));
    const proj = e1.length();
    if (proj < 1e-3) e1.set(1, 0, 0).sub(f.clone().multiplyScalar(f.x)); e1.normalize();
    const e2 = new THREE.Vector3().crossVectors(f, e1).normalize();
    const c = new THREE.Vector3(0, (x0 + x1) / 2, 0);
    const s0 = 0.58 * L * Math.max(proj, 0.15) + 3 * Rmax + 0.25 * L * (1 - Math.max(proj, 0.0));
    const nS = 17, tVals = [-2.4, -1.6, -1.15, -0.85, 0.85, 1.15, 1.6, 2.4, 0.0];
    const M = d.M, supersonic = M > 1.05;
    const tipY = x1;
    // the nose's cone angle, for the shock
    const noseAng = this.noseAngle(segs);
    const beta = supersonic ? (coneShockCached(M, Math.min(noseAng + d.alpha * RAD * 0.5, 1.2)) ?? Math.asin(1 / M)) : Math.PI / 2;
    const pos = [], col = [], dist = [];
    const u = new THREE.Vector3(), p = new THREE.Vector3(), tmp = new THREE.Vector3();
    const ds = Math.max(0.01 * L, 0.18), maxSteps = Math.ceil((2.4 * L + 2 * s0) / ds);
    const fieldAt = (P, out) => {
      // the flow at P: the stream plus the slender-body disturbance
      out.copy(f);
      const x = P.y, [R, Rx] = this.rOf(segs, x);
      if (R < 1e-3) return false;
      const rho = Math.hypot(P.x, P.z);
      if (rho < R * 1.0005) return true;                         // inside the body
      if (supersonic) { const reach = (tipY - x) * Math.tan(beta); if (rho > reach * 1.05 + 0.1) return false; }
      const w = f.y;
      const rx = P.x / rho, rz = P.z / rho;
      const uc = [f.x, f.z], ucr = uc[0] * rx + uc[1] * rz, r2 = (R * R) / (rho * rho);
      let dx = r2 * (uc[0] - 2 * ucr * rx), dz = r2 * (uc[1] - 2 * ucr * rz);
      const src = w * R * Rx / rho;                              // the section's growth pushes the air outward
      dx += src * rx; dz += src * rz;
      const fade = supersonic ? 0.85 : 1.0;
      out.x += dx * fade; out.z += dz * fade;
      return false;
    };
    for (let i = 0; i < nS; i++) {
      const s = (-0.55 + 1.1 * i / (nS - 1)) * L * Math.max(proj, 0.12);
      for (const t of tVals) {
        const tt = t * Rmax * (Math.abs(t) > 1.0 ? 1.0 : 1.0);
        p.copy(c).addScaledVector(e1, s).addScaledVector(e2, tt).addScaledVector(f, -s0);
        let dd = 0;
        const u2 = new THREE.Vector3();
        for (let k = 0; k < maxSteps; k++) {
          if (fieldAt(p, u)) break;                                // inside the body: the streamline ended at the surface
          const sp = u.length();
          if (sp < 0.05) break;
          u.divideScalar(sp);
          tmp.copy(p).addScaledVector(u, ds * 0.5);                // a second-order (midpoint) step
          if (fieldAt(tmp, u2)) break;
          u2.normalize();
          const next = p.clone().addScaledVector(u2, ds);
          // the colour: where the air is slow the pressure is high (Bernoulli: Cp = 1 - (speed ratio)^2)
          const cp = Math.max(-1.2, Math.min(cpStagnation(M), 1 - sp * sp));
          const tcol = cp < 0 ? 0.5 + 0.5 * cp / 0.6 : 0.5 + 0.5 * cp / Math.max(cpStagnation(M), 1);
          const rgb = colormap(tcol);
          pos.push(p.x, p.y, p.z, next.x, next.y, next.z); col.push(rgb[0], rgb[1], rgb[2], rgb[0], rgb[1], rgb[2]); dist.push(dd, dd + ds);
          p.copy(next); dd += ds;
          if (Math.abs(p.y - c.y) > L * 1.3 + s0 * 1.2) break;
        }
      }
    }
    this.geo.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3));
    this.geo.setAttribute('aColor', new THREE.Float32BufferAttribute(col, 3));
    this.geo.setAttribute('aDist', new THREE.Float32BufferAttribute(dist, 1));
    this.geo.setDrawRange(0, pos.length / 3);
    this.geo.boundingSphere = new THREE.Sphere(c.clone(), L * 3);
  }
  noseAngle(segs) {
    const tip = segs[segs.length - 1][2], back = Math.max(0.3, Math.min(6, 0.12 * (tip - segs[0][0])));
    // the radius a short way back from the tip: the half-angle of the cone that touches it
    const [r] = this.rOf(segs, tip - back);
    return Math.atan2(r, back);
  }
  dispose() { this.obj.removeFromParent(); this.geo.dispose(); this.mat.dispose(); }
}

/* ------------------------------------------------------------------------------------------------------------------------------ the shock */
const SHOCK_VERT = /* glsl */`
varying vec3 vN; varying vec3 vV; varying float vS;
attribute float aS;
void main() { vS = aS; vec4 mv = modelViewMatrix * vec4(position, 1.0); vN = normalize(normalMatrix * normal); vV = normalize(-mv.xyz); gl_Position = projectionMatrix * mv; }`;
const SHOCK_FRAG = /* glsl */`
precision highp float;
varying vec3 vN; varying vec3 vV; varying float vS;
uniform float uTime, uOpacity; uniform vec3 uColor;
void main() {
  float f = pow(1.0 - abs(dot(normalize(vN), normalize(vV))), 1.6);
  float ring = 0.5 + 0.5 * sin(vS * 60.0 - uTime * 5.0);
  float a = (0.10 + 0.55 * f) * (1.0 - vS) * (0.7 + 0.3 * ring) * uOpacity;
  gl_FragColor = vec4(uColor * (0.6 + 0.8 * f), a);
  #include <tonemapping_fragment>
  #include <colorspace_fragment>
}`;

/** The bow shock of a pointed nose in supersonic flight: an attached conical shock whose angle grows on the windward side and shrinks on the lee side with the angle of attack. */
export class Shock {
  constructor(model) {
    this.model = model;
    const NA = 48, NS = 14;
    this.NA = NA; this.NS = NS;
    const pos = new Float32Array((NA + 1) * (NS + 1) * 3), nrm = new Float32Array((NA + 1) * (NS + 1) * 3), aS = new Float32Array((NA + 1) * (NS + 1)), idx = [];
    for (let i = 0; i <= NS; i++) for (let j = 0; j <= NA; j++) aS[i * (NA + 1) + j] = i / NS;
    for (let i = 0; i < NS; i++) for (let j = 0; j < NA; j++) { const a = i * (NA + 1) + j, b = a + 1, c = a + NA + 1, e = c + 1; idx.push(a, c, b, b, c, e); }
    this.geo = new THREE.BufferGeometry();
    this.pos = new THREE.BufferAttribute(pos, 3); this.nrm = new THREE.BufferAttribute(nrm, 3);
    this.geo.setAttribute('position', this.pos); this.geo.setAttribute('normal', this.nrm); this.geo.setAttribute('aS', new THREE.BufferAttribute(aS, 1)); this.geo.setIndex(idx);
    this.mat = new THREE.ShaderMaterial({ vertexShader: SHOCK_VERT, fragmentShader: SHOCK_FRAG, transparent: true, depthWrite: false, side: THREE.DoubleSide, blending: THREE.AdditiveBlending, uniforms: { uTime: { value: 0 }, uOpacity: { value: 1 }, uColor: { value: new THREE.Vector3(0.45, 0.75, 1.0) } } });
    this.obj = new THREE.Mesh(this.geo, this.mat); this.obj.frustumCulled = false; this.obj.renderOrder = 7; this.obj.visible = false;
    model.root.add(this.obj);
    this.angle = null;
  }
  setVisible(v) { this.visible = v; if (!v) this.obj.visible = false; }
  update(d, tipY, noseAngle, L, tWall) {
    this.mat.uniforms.uTime.value = tWall;
    if (!this.visible || d.M < 1.08 || d.q < 200) { this.obj.visible = false; return; }
    this.obj.visible = true;
    const NA = this.NA, NS = this.NS, M = d.M, a = d.alpha * RAD;
    // the shock angle on each side of the vehicle: the cone's own angle plus or minus the angle of attack's share, for each azimuth
    const betas = [];
    for (let j = 0; j <= NA; j++) {
      const psi = (j / NA) * Math.PI * 2, phiW = Math.atan2(d.fhat[2] * -1, d.fhat[1] * -1);   // azimuth of the windward side: where the velocity's cross-component points
      const eff = Math.max(0.02, noseAngle + a * Math.cos(psi - phiW) * 0.9);
      betas.push(coneShockCached(M, Math.min(eff, 1.2)) ?? Math.asin(1 / M));
    }
    const Ls = L, P = this.pos.array, N = this.nrm.array;
    for (let i = 0; i <= NS; i++) {
      const s = i / NS, y = tipY - s * Ls;
      for (let j = 0; j <= NA; j++) {
        const psi = (j / NA) * Math.PI * 2, r = s * Ls * Math.tan(Math.min(betas[j], 1.45)), k = (i * (NA + 1) + j) * 3;
        P[k] = Math.cos(psi + 0) * r; P[k + 1] = y; P[k + 2] = Math.sin(psi) * r;
        N[k] = Math.cos(psi); N[k + 1] = 0.3; N[k + 2] = Math.sin(psi);
      }
    }
    this.pos.needsUpdate = true; this.nrm.needsUpdate = true;
    this.geo.boundingSphere = new THREE.Sphere(new THREE.Vector3(0, tipY - Ls / 2, 0), Ls * 2);
  }
  dispose() { this.obj.removeFromParent(); this.geo.dispose(); this.mat.dispose(); }
}

/* ------------------------------------------------------------------------------------------------------------------------------ the transonic cloud */
/** The Prandtl-Glauert cloud: where the air speeds up round the shoulder of the vehicle at about Mach 1 the pressure and the temperature drop, and in humid air a cloud of water drops forms in the dip. Humidity is assumed. */
export class Condensation {
  constructor(world) {
    this.world = world; this.bb = new Billboards(60, puffTexture(128, 21), { order: 6 }); world.nearScene.add(this.bb.mesh); this.ring = [];
    for (let i = 0; i < 24; i++) this.ring.push({ a: (i / 24) * 6.2832 + (i * 0.37), j: (Math.sin(i * 12.9898) * 43758.5453) % 1 });
  }
  update(d, shoulderY, radius, camera, light, on, tWall) {
    this.bb.begin(camera);
    const I = on ? Math.exp(-Math.pow((d.M - 1.0) / 0.1, 2)) * Math.max(0, Math.min(1, (10000 - (d.alt ?? 0)) / 8000)) * Math.min(1, d.q / 3000) : 0;
    if (I > 0.02) {
      const m = this.world.modelHolder, v = new THREE.Vector3();
      for (const r of this.ring) {
        const pr = radius * (1.05 + 0.25 * Math.abs(r.j));
        // the cloud sits on the shoulder, a ring that hugs the skin and sweeps back
        v.set(Math.cos(r.a) * pr, shoulderY - Math.abs(r.j) * radius * 1.6 - 0.5 * radius, Math.sin(r.a) * pr);
        m.localToWorld(v);
        this.bb.add(v.x, v.y, v.z, radius * (1.0 + 0.8 * Math.abs(r.j)), 0.55 * I, r.a, light[0], light[1], light[2]);
      }
    }
    this.bb.mat.uniforms.uLight.value.set(light[0], light[1], light[2]);
    this.bb.end();
  }
  dispose() { this.world.nearScene.remove(this.bb.mesh); }
}
