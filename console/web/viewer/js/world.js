// The world: the sky pass, then the near scene (the vehicle, the pad, the exhaust) drawn over it; the places and orientations of everything from one pose; the light the vehicle is lit with.
//
// Two scenes, because the numbers do not fit in one. The planet is 6371 km across and the vehicle is 9 m: in one scene with one depth buffer, 32-bit floats cannot put a bolt and a horizon in the same frame. So the sky,
// the atmosphere and the planet are one fragment shader that works from the camera's place above the planet's centre (env.js); and everything near is drawn in a scene whose origin is the vehicle's centre of gravity, in
// metres, so that what is within a kilometre of the camera has precision to a millimetre (the transforms are done in JavaScript's doubles and only the small offsets go to the GPU).
import { Imagery } from './imagery.js';
import * as THREE from '../vendor/three.module.js';
import { Sky, sunFromLocal } from './env.js';
import { CameraRig } from './cam.js';
import { VehicleModel, STYLES } from './models/vehicle.js';
import { Pad } from './pad.js';
import { Plumes, Trail, PadSmoke, Debris } from './fx.js';
import { toScene, quatToScene } from './data.js';

const _v = new THREE.Vector3(), _q = new THREE.Quaternion(), _m3 = new THREE.Matrix3(), _m4 = new THREE.Matrix4();

/** The sunlight that reaches a point (km from the planet's centre): the optical depth toward the sun through the same two-species atmosphere the sky shader uses. Returns an [r, g, b] transmittance. */
export function sunTransmittanceJS(pKm, sun, Rp, Ha) {
  const b = pKm.dot(sun), c = pKm.lengthSq() - Rp * Rp;
  if (b < 0 && c > 0 && b * b - c > 0) return [0, 0, 0];                 // the planet is in the way
  const Ra = Rp + Ha, cc = pKm.lengthSq() - Ra * Ra, dsc = b * b - cc;
  const tl = dsc > 0 ? -b + Math.sqrt(dsc) : 0, n = 12, dl = tl / n;
  let odR = 0, odM = 0;
  for (let j = 0; j < n; j++) { const t = (j + 0.5) * dl; const x = pKm.x + sun.x * t, y = pKm.y + sun.y * t, z = pKm.z + sun.z * t; const h = Math.max(0, Math.sqrt(x * x + y * y + z * z) - Rp); odR += Math.exp(-h / 8) * dl; odM += Math.exp(-h / 1.2) * dl; }
  return [Math.exp(-(5.8e-3 * odR + 21e-3 * 1.1 * odM)), Math.exp(-(13.5e-3 * odR + 21e-3 * 1.1 * odM)), Math.exp(-(33.1e-3 * odR + 21e-3 * 1.1 * odM))];
}

export class World {
  constructor(canvas, opt = {}) {
    this.canvas = canvas;
    this.renderer = new THREE.WebGLRenderer({ canvas, antialias: true, powerPreference: 'high-performance', preserveDrawingBuffer: !!opt.preserve });
    const r = this.renderer;
    r.autoClear = false; r.toneMapping = THREE.ACESFilmicToneMapping; r.toneMappingExposure = 0.45; r.outputColorSpace = THREE.SRGBColorSpace;
    this.quality = opt.quality ?? 1;
    r.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2) * (this.quality < 0.7 ? 0.7 : 1));
    this.nearScene = new THREE.Scene();
    this.camera = new THREE.PerspectiveCamera(40, 1, 0.3, 4.0e6);
    this.rig = new CameraRig(this.camera);
    this.sky = new Sky();
    this.imagery = new Imagery(this.sky, this.renderer);
    this.sun = new THREE.DirectionalLight(0xffffff, 1);
    this.nearScene.add(this.sun, this.sun.target);
    this.fill = new THREE.AmbientLight(0xffffff, 0.0);
    this.nearScene.add(this.fill);
    this.overlay = new THREE.Group(); this.overlay.name = 'overlays';          // the lenses' objects, in the vehicle's frame
    this.nearScene.add(this.overlay);
    this.worldOverlay = new THREE.Group(); this.nearScene.add(this.worldOverlay);   // and in the near scene's own frame
    this.settings = { sunElev: 28, sunBear: 135, cloud: 0.35, style: 'auto', stars: 1, trail: true, plumes: true, pad: true, atmosphereQuality: 0.6 };
    this.vehicle = null; this.pad = null; this.plumes = null; this.debris = []; this.padCG = null; this.last = { stg: null };
    this.trail = new Trail(); this.nearScene.add(this.trail.mesh);
    this.smoke = new PadSmoke(); this.nearScene.add(this.smoke.bb.mesh);
    this.vehicleGroup = new THREE.Group(); this.nearScene.add(this.vehicleGroup);   // rotated by the vehicle's attitude, the origin at its CG
    this.modelHolder = new THREE.Group(); this.vehicleGroup.add(this.modelHolder);  // the model's origin (the tail) is offset from the CG by -cg along the axis
    this.vehicleGroup.add(this.overlay);
    this.envRT = new THREE.WebGLCubeRenderTarget(128, { type: THREE.HalfFloatType, generateMipmaps: false });
    this.cubeCam = new THREE.CubeCamera(0.1, 100, this.envRT);
    this.pmrem = new THREE.PMREMGenerator(this.renderer);
    this.envScene = new THREE.Scene();
    this.envMat = null; this.envTimer = 1e9; this.envTex = null;
    this.buildEnvSky();
    this.time = { wall: 0 };
    this.rS = [0, 6.371e6, 0]; this.tt = 0; this.qs = new THREE.Quaternion();
    this.size = { w: 1, h: 1 };
    this.resize();
  }

  buildEnvSky() {
    // the sky as seen from a cube: the same fragment shader on the inside of a box whose vertices carry the direction
    const m = this.sky.mat;
    const vert = `varying vec2 vUv; varying vec3 vDirC; void main() { vUv = uv; vDirC = position; gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0); }`;
    const frag = m.fragmentShader.replace('varying vec2 vUv;', 'varying vec2 vUv; varying vec3 vDirC;').replace('vec3 rd = normalize(uCamRot * vec3(ndc.x * uTan.x, ndc.y * uTan.y, -1.0));', 'vec3 rd = normalize(vDirC);');
    this.envMat = new THREE.ShaderMaterial({ vertexShader: vert, fragmentShader: frag, uniforms: m.uniforms, side: THREE.BackSide, depthTest: false, depthWrite: false });
    this.envBox = new THREE.Mesh(new THREE.BoxGeometry(10, 10, 10), this.envMat);
    this.envScene.add(this.envBox);
  }

  resize() {
    const w = this.canvas.clientWidth || window.innerWidth, h = this.canvas.clientHeight || window.innerHeight;
    if (w === this.size.w && h === this.size.h) return;
    this.size = { w, h };
    this.renderer.setSize(w, h, false);
    this.camera.aspect = w / h; this.camera.updateProjectionMatrix();
  }

  /** The vehicle of a spec in a style, and the site it stands on. Called when a spec arrives or the style changes. */
  setVehicle(spec, style = 'auto', opts = {}) {
    this.disposeVehicle();
    this.spec = spec;
    this.vehicle = new VehicleModel(spec, style, opts);
    this.modelHolder.add(this.vehicle.root);
    this.plumes = new Plumes(this.vehicle);
    const L = spec.length, D = spec.diameter;
    this.mountH = Math.min(22, Math.max(2, D * 2.2)) + 0.4;
    if (this.pad) { this.nearScene.remove(this.pad.group); }
    this.pad = new Pad(L, D, this.mountH, 1600);
    this.nearScene.add(this.pad.group);
    this.padCG = null; this.trail.clear(); this.smoke.clear(); this.last.stg = null;
    for (const d of this.debris) this.nearScene.remove(d.group); this.debris = [];
    this.rig.firstFrame = true;
  }
  disposeVehicle() {
    if (this.plumes) { this.plumes.dispose(); this.plumes = null; }
    if (this.vehicle) { this.modelHolder.remove(this.vehicle.root); this.vehicle = null; }
  }

  /** Renders the sky into the environment cube at the vehicle's place and filters it for the vehicle's reflections and ambient light. */
  updateEnvironment() {
    if (this.skip?.env) return;
    const u = this.sky.u, keep = u.uCamPos.value.clone();
    const rKm = new THREE.Vector3(this.rS[0], this.rS[1], this.rS[2]).multiplyScalar(1e-3);
    u.uCamPos.value.copy(rKm);
    this.cubeCam.position.set(0, 0, 0);
    const q = u.uQuality.value; u.uQuality.value = 0.0;
    this.renderer.toneMapping = THREE.NoToneMapping;
    this.cubeCam.update(this.renderer, this.envScene);
    this.renderer.toneMapping = THREE.ACESFilmicToneMapping;
    u.uQuality.value = q; u.uCamPos.value.copy(keep);
    const tex = this.pmrem.fromCubemap(this.envRT.texture).texture;
    if (this.envTex) this.envTex.dispose();
    this.envTex = tex; this.nearScene.environment = tex;
  }

  /** One frame: place everything from the pose, light it, draw the sky and then the near scene. `ctx` carries what the page chose (camera, sun, lens). */
  frame(pose, dt, wall, ctx = {}) {
    this.resize();
    const s = this.settings;
    this.time.wall = wall;
    const spec = this.spec;
    const R = spec.planet.radius;
    if (this.padCG == null && pose) this.padCG = pose.cg;
    const rv = (R - (this.padCG ?? pose.cg) - this.mountH);                 // the surface the pad stands on: the vehicle's tail sits `mountH` above it, its CG `cg` above that
    const rS = toScene(pose.r); this.rS = rS; this.tt = pose.tt;
    const qs = new THREE.Quaternion(...quatToScene(pose.q)).normalize(); this.qs = qs;
    const pole = new THREE.Vector3(...toScene(spec.planet.pole)).normalize();
    const earthAngle = spec.planet.rotation * pose.tt;
    const padQ = new THREE.Quaternion().setFromAxisAngle(pole, earthAngle);
    const up0 = new THREE.Vector3(0, 1, 0), down0 = new THREE.Vector3(0, 0, 1);
    const up = new THREE.Vector3(rS[0], rS[1], rS[2]).normalize();
    const down = down0.clone().applyQuaternion(padQ);
    const padWorld = up0.clone().multiplyScalar(rv).applyQuaternion(padQ);
    const padRel = padWorld.clone().sub(new THREE.Vector3(...rS));
    const axis = new THREE.Vector3(0, 1, 0).applyQuaternion(qs);

    // the vehicle
    this.vehicleGroup.position.set(0, 0, 0); this.vehicleGroup.quaternion.copy(qs);
    this.modelHolder.position.set(0, -pose.cg, 0);
    if (this.vehicle) { this.vehicle.apply(pose); }
    if (this.plumes) { this.plumes.update(pose, wall, { glow: 1 }); }
    if (this.pad) {
      this.pad.group.position.copy(padRel); this.pad.group.quaternion.copy(padQ);
      this.pad.structures.visible = s.pad && (pose.alt < 25000);
      const g = this.pad.groundMat.uniforms;
      g.uTime.value = wall; g.uFade.value = 1 - THREE.MathUtils.smoothstep(pose.alt, 12000, 45000);
      this.pad.ground.visible = pose.alt < 50000;
    }

    // the camera
    const camCtx = { len: spec.length, diameter: spec.diameter, axis, up, down, padRel, cg: pose.cg, alt: pose.alt, dt, quat: qs };
    this.rig.update(camCtx);
    const camPos = this.camera.position;

    // light: the sun in the scene's axes (fixed in space), its colour after the atmosphere, the sky's light from the environment
    const sun = sunFromLocal(s.sunElev, s.sunBear);
    const rKm = new THREE.Vector3(rS[0], rS[1], rS[2]).multiplyScalar(1e-3);
    const Rp = rv * 1e-3, Ha = 100;
    const T = sunTransmittanceJS(rKm, sun, Rp, Ha);
    const SUNI = this.sky.u.uSunI.value;
    this.sun.color.setRGB(T[0], T[1], T[2]); this.sun.intensity = SUNI;
    this.sun.position.copy(sun).multiplyScalar(500); this.sun.target.position.set(0, 0, 0);
    this.sunDir = sun; this.sunT = T;
    // the environment is refreshed a few times a second, and at once when the vehicle moves a long way from where it was taken
    this.envTimer += dt;
    const moved = Math.abs(pose.alt - (this.envAlt ?? -1e9));
    if (this.envTimer > 0.6 || moved > 800 + 0.15 * Math.max(0, this.envAlt ?? 0)) { this.updateEnvironment(); this.envTimer = 0; this.envAlt = pose.alt; }
    this.nearScene.environmentIntensity = 1.0;
    if (this.pad) {
      const g = this.pad.groundMat.uniforms;
      const sl = sun.clone().applyQuaternion(padQ.clone().invert());          // the sun in the pad's frame
      g.uSun.value.copy(sl); g.uSunCol.value.set(T[0] * SUNI, T[1] * SUNI, T[2] * SUNI);
      const dayAmb = Math.max(0, Math.min(1, sun.dot(up) * 1.4 + 0.25));
      g.uAmb.value.set(0.07, 0.10, 0.16).multiplyScalar(SUNI * 0.5 * Math.exp(-pose.alt / 9000) * (0.15 + 0.85 * dayAmb) + 0.0001);
      g.uHaze.value.set(0.55, 0.70, 0.95).multiplyScalar(SUNI * 0.07 * (0.1 + 0.9 * dayAmb) * Math.exp(-pose.alt / 12000));
      g.uCam.value.copy(camPos).sub(this.pad.group.position);
      g.uCamAlt.value = Math.max(0, pose.alt - 20);
      g.uCoast.value = 1600;
      g.uImgOn.value = this.imagery.state && this.imagery.state !== 'procedural' ? 1 : 0;
    }

    // the sky
    const u = this.sky.u;
    const cw = new THREE.Vector3().copy(camPos).add(new THREE.Vector3(rS[0], rS[1], rS[2]));
    u.uCamPos.value.copy(cw).multiplyScalar(1e-3);
    _m4.copy(this.camera.matrixWorld); _m3.setFromMatrix4(_m4); u.uCamRot.value.copy(_m3);
    const t = Math.tan(this.camera.fov * Math.PI / 360); u.uTan.value.set(t * this.camera.aspect, t);
    u.uSun.value.copy(sun); u.uRp.value = Rp; u.uHa.value = Ha; u.uTime.value = wall; u.uCloud.value = s.cloud; u.uStars.value = s.stars; u.uQuality.value = s.atmosphereQuality * this.quality;
    const rot = new THREE.Quaternion().setFromAxisAngle(pole, -earthAngle); _m3.setFromMatrix4(_m4.makeRotationFromQuaternion(rot)); u.uPlanetRot.value.copy(_m3);
    u.uPole.value.copy(pole); u.uUp0.value.copy(up0); u.uDown0.value.copy(down0); u.uCoastKm.value = 1.6;
    this.imagery.sync(pole, up0).catch((e) => console.warn('imagery', e && e.message));

    // the exhaust trail, the pad's smoke, the stages let go
    const thrustFrac = pose.thr > 0 ? Math.min(1, pose.thr / (pose.m * 9.80665 * 1.6)) : 0;     // about 1 at the thrust-to-weight of a launcher on the pad
    const tailWorld = [rS[0] + axis.x * -pose.cg, rS[1] + axis.y * -pose.cg, rS[2] + axis.z * -pose.cg];
    const airRho = pose.air ? pose.air[2] : 1.2;
    if (s.trail && pose.thr > 0 && !pose.clamp) {
      const wnd = pose.wind ? toScene(pose.wind) : [0, 0, 0];
      this.trail.emit(pose.tt, tailWorld, spec.diameter * 0.55, Math.min(1, Math.pow(airRho / 1.2, 0.4)), wnd);
    }
    this.trail.update(pose.tt, rS, camPos, [0.95, 0.96, 1.0].map((c, i) => c * (0.8 + 0.25 * T[i])), 60);
    this.trailVisible = s.trail;
    this.trail.mesh.visible = s.trail;
    const padM = new THREE.Matrix4().compose(padRel, padQ, new THREE.Vector3(1, 1, 1));
    const light = [T[0] * 0.9 + 0.1, T[1] * 0.9 + 0.1, T[2] * 0.9 + 0.1].map((x) => x * Math.max(0.12, Math.min(1.1, sun.dot(up) * 1.1 + 0.3)));
    this.smoke.update(dt, pose.tt, { alt: pose.alt + pose.cg, thrust: pose.thr > 0 ? Math.min(1, thrustFrac * 1.5) : 0, radius: spec.diameter * 0.6 }, this.camera, padM, light);

    this.stepDebris(pose, dt, rS, qs, axis);
    this.last.stg = pose.stg;
  }

  stepDebris(pose, dt, rS, qs, axis) {
    const spec = this.spec, stg = pose.stg ?? 255;
    if (this.last.stg != null && this.vehicle) {
      for (let i = 0; i < spec.stages.length; i++) {
        const was = (this.last.stg >> i) & 1, now = (stg >> i) & 1;
        if (was && !now && dt > 0 && dt < 1) {
          const st = spec.stages[i], mid = (st.x_start_m || 0) + st.length_m * 0.45;
          const clone = this.vehicle.stages[i].group.clone(true);
          clone.traverse((o) => { if (o.material && o.material.isShaderMaterial) o.visible = false; if (o.isSprite) o.visible = false; });
          clone.visible = true;
          const holder = new THREE.Group(); clone.position.y = -mid; holder.add(clone);
          const r0 = [rS[0] + axis.x * (mid - pose.cg), rS[1] + axis.y * (mid - pose.cg), rS[2] + axis.z * (mid - pose.cg)];
          const deb = new Debris(holder, r0, toScene(pose.v), qs, st.separation_dv_ms || 1.0, new THREE.Vector3(0.05 + (st.tipoff_pitch_dps || 0) * 0.0175, 0.03, 0.04), { r: (st.radius_m || 1), m: st.dry_mass_kg || 10000 });
          this.nearScene.add(holder); this.debris.push(deb);
        }
      }
    }
    const R = spec.planet.radius, mu = spec.planet.mu;
    for (let k = this.debris.length - 1; k >= 0; k--) {
      const d = this.debris[k];
      d.step(Math.min(dt, 0.1), mu, (rn) => spec.airAt(rn - R)[2]);
      d.group.position.set(d.r[0] - rS[0], d.r[1] - rS[1], d.r[2] - rS[2]); d.group.quaternion.copy(d.q);
      if (Math.hypot(...d.r) < R - 50 || d.t > 600) { this.nearScene.remove(d.group); this.debris.splice(k, 1); }
    }
  }

  render() {
    const r = this.renderer;
    r.setRenderTarget(null);
    r.clear(true, true, true);
    if (!this.skip?.sky) r.render(this.sky.scene, this.sky.cam);
    r.clearDepth();
    if (!this.skip?.near) r.render(this.nearScene, this.camera);
  }
  /** a PNG of what is on the canvas now (for the page's screenshot button, and for tests: a headless browser does not composite a WebGL canvas into its own screenshots) */
  snapshot() { this.render(); return this.canvas.toDataURL('image/png'); }
}

export { STYLES };
