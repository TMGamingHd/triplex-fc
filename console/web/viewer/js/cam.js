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
  { id: 'ground', label: 'Ground', hint: 'A person standing near the pad, looking at the vehicle: drag to look around, wheel to zoom, double-click to look at it again' },
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
    this.gpos = [150, 1.8, -350];                                // the observer, from the pad: metres to the left of downrange, up, and along downrange (negative: uprange of the pad)
    this.gaz = 0; this.gel = 0; this.gzoom = 1;                  // what the user added to the look at the vehicle: radians to the left and up, and the zoom
    this.focus = null;                                           // a lens can ask the camera to look at a point of the vehicle: {x: metres along the body, d: distance in m}
  }
  set(mode) { if (this.mode !== mode) { this.mode = mode; this.firstFrame = true; } }
  // input: a drag turns the orbit; the wheel zooms; the vertical drag with shift or the right button slides the point looked at along the vehicle
  drag(dx, dy, slide) {
    if (slide) { this.anchor = Math.min(1.2, Math.max(-0.2, this.anchor - dy * 0.002)); return; }
    if (this.mode === 'ground') { const k = 0.004 * Math.min(1, this.gzoom); this.gaz = Math.max(-Math.PI, Math.min(Math.PI, this.gaz + dx * k)); this.gel = Math.max(-0.6, Math.min(1.2, this.gel + dy * k)); return; }
    this.yaw -= dx * 0.005; this.pitch = Math.min(1.5, Math.max(-1.5, this.pitch + dy * 0.004));
    if (this.mode !== 'orbit') this.set('orbit');
  }
  /** Double-click: the ground camera looks at the vehicle again; the orbit goes back to where it starts. */
  recenter() { if (this.mode === 'ground') { this.gaz = 0; this.gel = 0; this.gzoom = 1; } else { this.yaw = 0.7; this.pitch = 0.18; this.dist = 1.9; this.anchor = 0.45; this.slide = 0; } }
  zoom(f) { this.dist = Math.min(40, Math.max(0.06, this.dist * Math.exp(f * 0.0012))); }

  /** ctx: { len, diameter, axis (unit, scene), up (unit, scene: the local vertical), down (unit: downrange, horizontal), padRel (the pad's place relative to the vehicle), cg (m from the tail), alt, quat (the vehicle's), dt, lookFocus } */
  update(ctx) {
    const { len, axis, up, dt } = ctx;
    const h = ctx.down.clone().sub(up.clone().multiplyScalar(ctx.down.dot(up))).normalize();   // horizontal downrange
    const side = new THREE.Vector3().crossVectors(up, h).normalize();                          // to the left of downrange
    const tail = axis.clone().multiplyScalar(-ctx.cg);                                         // the vehicle's tail, from the CG
    const at = (frac) => tail.clone().addScaledVector(axis, frac * len);
    let pos, look, upv = up.clone(), fov = 38, subject = 0;               // subject: the distance to what is being looked at, for the near plane
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
        fov = THREE.MathUtils.clamp(2 * Math.atan(0.95 * L / Math.max(d, 1)) * 180 / Math.PI, 1.0, 55); subject = d;
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
      case 'ground': {                              // a person at the pad who looks at the vehicle, whatever it does; a drag turns the view away from it, a double-click brings it back
        pos = ctx.padRel.clone().addScaledVector(side, this.gpos[0]).addScaledVector(h, this.gpos[2]).addScaledVector(up, this.gpos[1]);
        const aim = at(0.5), toV = aim.clone().sub(pos), d = Math.max(toV.length(), 1);
        const dir = toV.divideScalar(d);
        dir.applyAxisAngle(up, this.gaz);                                                     // to the left
        const lat = new THREE.Vector3().crossVectors(dir, up);
        if (lat.lengthSq() > 1e-8) dir.applyAxisAngle(lat.normalize(), this.gel);             // up
        look = pos.clone().addScaledVector(dir, 100);
        // the field of view keeps the vehicle in the picture as it leaves (a broadcast lens that zooms out the other way), the wheel scales it
        const R = Math.max(L * 1.2, 60);                                                      // the vehicle and its plume
        fov = THREE.MathUtils.clamp(Math.max(2 * Math.atan(R / d) * 180 / Math.PI, 1.0) * this.gzoom, 0.25, 100); subject = d;
        if (Math.abs(dir.dot(up)) > 0.96) upv = h.clone();                                    // looking straight up: the top of the picture is downrange
        break;
      }
      case 'wide': {
        pos = ctx.padRel.clone().addScaledVector(h, 2600).addScaledVector(side, 900).addScaledVector(up, 160);
        look = new THREE.Vector3(0, 0, 0);
        const d = pos.distanceTo(look);
        fov = THREE.MathUtils.clamp(2 * Math.atan(1.5 * Math.max(L, ctx.alt * 0.9 + L) / Math.max(d, 1)) * 180 / Math.PI, 5, 70); subject = d;
        if (ctx.alt > 8000) { pos = ctx.padRel.clone().addScaledVector(h, 2600 + ctx.alt * 1.6).addScaledVector(side, 900 + ctx.alt).addScaledVector(up, 160); fov = 40; subject = pos.distanceTo(look); }
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
    // a 24-bit depth buffer from 0.3 m to 4000 km resolves 40 m at 14 km, and a vehicle seen from there is a blob: the near plane follows the distance to what is looked at (0.2 %, at least 0.3 m: 29 m at 14 km, a hole of that radius round the camera)
    const near = THREE.MathUtils.clamp(0.002 * subject, 0.3, 600);
    if (Math.abs(this.camera.fov - this.fov) > 1e-3 || Math.abs(this.camera.near - near) > 1e-3 * near) { this.camera.fov = this.fov; this.camera.near = near; this.camera.updateProjectionMatrix(); }
    this.camera.updateMatrixWorld(true);
  }
  /** a direction at right angles to the body axis, stable as the vehicle turns (it keeps to the side of downrange) */
  lateral(axis, side, up) {
    const v = side.clone().sub(axis.clone().multiplyScalar(side.dot(axis)));
    if (v.lengthSq() < 1e-6) v.copy(up).sub(axis.clone().multiplyScalar(up.dot(axis)));
    return v.normalize();
  }
}
