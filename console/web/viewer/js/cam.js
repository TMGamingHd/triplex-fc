// The cameras. Each is a rule that puts the eye somewhere and points it at something, given where the vehicle is and which way it points; the rig smooths what the rules say so that a change of camera, or a stage
// leaving, never jumps. Positions are in the near scene's space: metres, the vehicle's centre of gravity at the origin, the axes the simulator's inertial frame permuted into the scene's (data.js).
import * as THREE from '../vendor/three.module.js';

export const CAMERAS = [
  { id: 'orbit', label: 'Orbit', hint: 'Drag to turn around the vehicle, wheel to zoom, right-drag or shift-drag to slide along it' },
  { id: 'chase', label: 'Chase', hint: 'Behind and above the vehicle, following its attitude' },
  { id: 'track', label: 'Tracking', hint: 'A camera on the ground by the pad that follows the vehicle up, as on a broadcast' },
  { id: 'engines', label: 'Engines', hint: 'On the vehicle, looking at the engines and the plumes' },
  { id: 'nose', label: 'Nose', hint: 'On the vehicle, looking where it points' },
  { id: 'earth', label: 'Earth', hint: 'From the vehicle, looking down at the planet it is leaving' },
  { id: 'ground', label: 'Ground', hint: 'A person standing near the pad: drag to look around' },
  { id: 'wide', label: 'Wide', hint: 'A fixed point downrange of the pad, looking back at the whole flight' },
];

const _m = new THREE.Matrix4();

export class CameraRig {
  constructor(camera) {
    this.camera = camera;
    this.mode = 'orbit';
    this.yaw = 0.7; this.pitch = 0.18; this.dist = 1.9;          // orbit: radians from downrange, from the horizon, and the distance in vehicle lengths
    this.anchor = 0.45;                                          // where along the vehicle the orbit is centred, 0 at the tail, 1 at the nose
    this.slide = 0;
    this.pos = new THREE.Vector3(60, 30, 140); this.look = new THREE.Vector3(); this.up = new THREE.Vector3(0, 1, 0); this.fov = 40;
    this.smooth = true; this.firstFrame = true;
    this.chaseDir = new THREE.Vector3(0, 0, -1);
    this.gpos = [150, 2, -350]; this.gaz = 0.4; this.gel = 0.2; this.gfov = 60;
    this.focus = null;                                           // a lens can ask the camera to look at a point of the vehicle: {x: metres along the body, d: distance in m}
  }
  set(mode) { if (this.mode !== mode) { this.mode = mode; this.firstFrame = true; } }
  // input: a drag turns the orbit; the wheel zooms; the vertical drag with shift or the right button slides the point looked at along the vehicle
  drag(dx, dy, slide) {
    if (slide) { this.anchor = Math.min(1.2, Math.max(-0.2, this.anchor - dy * 0.002)); return; }
    if (this.mode === 'ground') { this.gaz -= dx * 0.004 * (this.gfov / 60); this.gel = Math.min(1.5, Math.max(-0.5, this.gel + dy * 0.004 * (this.gfov / 60))); return; }
    this.yaw -= dx * 0.005; this.pitch = Math.min(1.5, Math.max(-1.5, this.pitch + dy * 0.004));
    if (this.mode !== 'orbit') this.set('orbit');
  }
  zoom(f) { this.dist = Math.min(40, Math.max(0.06, this.dist * Math.exp(f * 0.0012))); }

  /** ctx: { len, diameter, axis (unit, scene), up (unit, scene: the local vertical), down (unit: downrange, horizontal), padRel (the pad's place relative to the vehicle), cg (m from the tail), alt, quat (the vehicle's), dt, lookFocus } */
  update(ctx) {
    const { len, axis, up, dt } = ctx;
    const h = ctx.down.clone().sub(up.clone().multiplyScalar(ctx.down.dot(up))).normalize();   // horizontal downrange
    const side = new THREE.Vector3().crossVectors(up, h).normalize();                          // to the left of downrange
    const tail = axis.clone().multiplyScalar(-ctx.cg);                                         // the vehicle's tail, from the CG
    const at = (frac) => tail.clone().addScaledVector(axis, frac * len);
    let pos, look, upv = up.clone(), fov = 38;
    const L = Math.max(len, 5);
    switch (this.mode) {
      case 'chase': {
        const want = axis.clone().multiplyScalar(-1).addScaledVector(up, 0.28).addScaledVector(side, 0.42).normalize();
        this.chaseDir.lerp(want, this.firstFrame ? 1 : 1 - Math.exp(-dt * 2.2)).normalize();
        look = at(0.55);
        pos = look.clone().addScaledVector(this.chaseDir, L * 1.45);
        fov = 42; break;
      }
      case 'track': {
        const p = ctx.padRel.clone().add(side.clone().multiplyScalar(-260)).addScaledVector(h, -420).addScaledVector(up, 14);   // beside and behind the pad
        pos = p; look = new THREE.Vector3(0, 0, 0);
        const d = pos.distanceTo(look);
        fov = THREE.MathUtils.clamp(2 * Math.atan(0.95 * L / Math.max(d, 1)) * 180 / Math.PI, 1.0, 55);
        upv = up.clone(); break;
      }
      case 'engines': {
        const tailP = tail.clone();
        pos = tailP.clone().addScaledVector(axis, -L * 0.30).add(this.lateral(axis, side, up).multiplyScalar(L * 0.30));
        look = tailP.clone().addScaledVector(axis, L * 0.02);
        upv = axis.clone(); fov = 55; break;
      }
      case 'nose': {
        const nose = at(1.0);
        pos = nose.clone().addScaledVector(axis, 2.0 + L * 0.01).add(this.lateral(axis, side, up).multiplyScalar(ctx.diameter * 0.5 + 1));
        look = nose.clone().addScaledVector(axis, L * 6);
        upv = this.lateral(axis, side, up).multiplyScalar(-1); fov = 70; break;
      }
      case 'earth': {
        look = at(0.5);
        pos = look.clone().addScaledVector(up, L * 0.16).addScaledVector(h, L * 0.15).addScaledVector(side, L * 1.3);
        const tgt = look.clone().addScaledVector(up, -L * 3.4);
        look = tgt.lerp(look, 0.4); fov = 62; upv = h.clone(); break;
      }
      case 'ground': {                              // an observer at the pad, looking where the user points (azimuth from downrange, elevation)
        pos = ctx.padRel.clone().addScaledVector(side, this.gpos[0]).addScaledVector(h, this.gpos[2]).addScaledVector(up, this.gpos[1]);
        const ce = Math.cos(this.gel), se = Math.sin(this.gel);
        const dir = h.clone().multiplyScalar(ce * Math.cos(this.gaz)).addScaledVector(side, ce * Math.sin(this.gaz)).addScaledVector(up, se);
        look = pos.clone().addScaledVector(dir, 100); fov = this.gfov; break;
      }
      case 'wide': {
        pos = ctx.padRel.clone().addScaledVector(h, 2600).addScaledVector(side, 900).addScaledVector(up, 160);
        look = new THREE.Vector3(0, 0, 0);
        const d = pos.distanceTo(look);
        fov = THREE.MathUtils.clamp(2 * Math.atan(1.5 * Math.max(L, ctx.alt * 0.9 + L) / Math.max(d, 1)) * 180 / Math.PI, 5, 70);
        if (ctx.alt > 8000) { pos = ctx.padRel.clone().addScaledVector(h, 2600 + ctx.alt * 1.6).addScaledVector(side, 900 + ctx.alt).addScaledVector(up, 160); fov = 40; }
        break;
      }
      default: {                                   // orbit, about the vehicle, in the local horizon's frame
        const f = this.focus ? this.focus : null;
        const aim = f ? tail.clone().addScaledVector(axis, f.x) : at(Math.min(1, Math.max(0, this.anchor)));
        aim.addScaledVector(axis, this.slide);
        const d = (f ? f.d : this.dist * L);
        const cp = Math.cos(this.pitch), sp = Math.sin(this.pitch);
        const dir = h.clone().multiplyScalar(cp * Math.cos(this.yaw)).addScaledVector(side, cp * Math.sin(this.yaw)).addScaledVector(up, sp);
        pos = aim.clone().addScaledVector(dir, d); look = aim; fov = f && f.fov ? f.fov : 40;
      }
    }
    const k = this.firstFrame || !this.smooth ? 1 : 1 - Math.exp(-dt * 12);
    this.pos.lerp(pos, this.mode === 'orbit' && !this.firstFrame ? Math.max(k, .5) : k);
    this.look.lerp(look, k); this.up.lerp(upv, k).normalize(); this.fov += (fov - this.fov) * (this.firstFrame ? 1 : 1 - Math.exp(-dt * 6));
    this.firstFrame = false;
    this.camera.position.copy(this.pos);
    _m.lookAt(this.pos, this.look, this.up);
    this.camera.quaternion.setFromRotationMatrix(_m);
    if (Math.abs(this.camera.fov - this.fov) > 1e-3) { this.camera.fov = this.fov; this.camera.updateProjectionMatrix(); }
    this.camera.updateMatrixWorld(true);
  }
  /** a direction at right angles to the body axis, stable as the vehicle turns (it keeps to the side of downrange) */
  lateral(axis, side, up) {
    const v = side.clone().sub(axis.clone().multiplyScalar(side.dot(axis)));
    if (v.lengthSq() < 1e-6) v.copy(up).sub(axis.clone().multiplyScalar(up.dot(axis)));
    return v.normalize();
  }
}
