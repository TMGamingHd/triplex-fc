// The launch site: the ground around the pad (land, a shore and the sea downrange of it), the launch mount the vehicle stands on, the tower beside it, the lightning masts and a tank farm. It is sized from the vehicle
// (a tower a little taller than the vehicle, a mount 2.2 diameters high), and stands in the pad's own frame: x crossrange, y up, z downrange (the scene's axes at T-zero).
// Drawn from the simulator's geometry only in this: where the vehicle's tail is and how big it is. The tower, the tanks and the shore are made up, and say so (docs/design/VIEWER.md).
import * as THREE from '../vendor/three.module.js';
import { bar } from './models/common.js';
import { NOISE_GLSL } from './env.js';

const GROUND_VERT = /* glsl */`
varying vec3 vW; varying vec3 vN;
void main() { vec4 w = modelMatrix * vec4(position, 1.0); vW = w.xyz; gl_Position = projectionMatrix * viewMatrix * w; }`;
const GROUND_FRAG = /* glsl */`
precision highp float;
${NOISE_GLSL}
varying vec3 vW;
uniform vec3 uSun; uniform vec3 uSunCol; uniform vec3 uAmb; uniform vec3 uHaze; uniform vec3 uCam; uniform float uCoast; uniform float uTime;
uniform float uFade; uniform float uMountR; uniform float uScale; uniform float uCamAlt; uniform float uImgOn;
uniform vec3 uPadPos;     // the pad centre in the ground's own coordinates (the ground is a child of the pad, so 0)
float hash(vec2 p) { return fract(sin(dot(p, vec2(127.1, 311.7))) * 43758.5453); }
float vn(vec2 p) { vec2 i = floor(p), f = fract(p); f = f * f * (3. - 2. * f); return mix(mix(hash(i), hash(i + vec2(1, 0)), f.x), mix(hash(i + vec2(0, 1)), hash(i + vec2(1, 1)), f.x), f.y); }
float fbm(vec2 p) { float a = .5, s = 0.; for (int i = 0; i < 6; i++) { s += a * vn(p); p = p * 2.03 + 7.1; a *= .5; } return s; }
void main() {
  vec2 g = vW.xz;                                       // x crossrange, z downrange, metres, from the pad
  // the shore line: the far sky shader draws it with 2.2 km of slow wander and a ripple of 0.35 km; the same noise (in km) is evaluated here
  float cz = g.x / 1000.0;
  float shoreKm = uCoast / 1000.0 + 2.2 * (vnoise(vec3(cz * .09, 1.3, 2.1)) - .5) + .35 * (vnoise(vec3(cz * 1.1, 4.0, .5)) - .5);
  float dx = g.y / 1000.0 - shoreKm;                    // km seaward of the shore
  float land = 1.0 - smoothstep(-.0004, .0004, dx);
  float d = length(g);
  vec3 N = vec3(0.0, 1.0, 0.0);
  vec3 col;
  float wave = 0.0;
  if (land > .5) {
    float n = fbm(g * .013), m = fbm(g * .22 + 3.0), fine = fbm(g * 2.1);
    vec3 scrub = mix(vec3(.17, .155, .10), vec3(.12, .14, .075), smoothstep(.35, .65, n));
    scrub = mix(scrub, vec3(.34, .29, .19), smoothstep(.55, .8, fbm(g * .006 + 9.0)) * .6);
    col = scrub * (.7 + .5 * m) * (.85 + .3 * fine);
    float sand = smoothstep(.22, 0.0, -dx) ;
    col = mix(col, vec3(.55, .5, .38) * (.85 + .3 * fine), sand);
    // the pad: a concrete apron, a darker ring where the flame falls, a road to it
    float apron = 1.0 - smoothstep(uMountR * 8.0, uMountR * 8.0 + 4.0, d);
    // the slabs of the apron: expansion joints every 12 m, a darker line, and a shade of its own for each slab
    vec2 sl = g / 12.0; vec2 jf = abs(fract(sl) - .5);
    float joint = 1.0 - smoothstep(.0, .012, .5 - max(jf.x, jf.y));
    float slabShade = .92 + .16 * hash(floor(sl));
    col = mix(col, vec3(.34, .33, .31) * (.85 + .25 * fine) * slabShade * (1.0 - .35 * joint), apron);
    float scorch = (1.0 - smoothstep(uMountR * 1.4, uMountR * 4.5, d)) * apron;
    col = mix(col, vec3(.09, .085, .08), scorch * .8);
    float road = (1.0 - smoothstep(5.0 * uScale, 7.0 * uScale, abs(g.x + 0.0))) * step(0.0, -g.y) * step(-3000.0, g.y);
    col = mix(col, vec3(.24, .235, .23), road * (1.0 - apron) * .9);
  } else {
    float deep = smoothstep(0.0, .5, dx);
    col = mix(vec3(.10, .35, .36), vec3(.01, .05, .11), deep);
    col = mix(col, vec3(.045, .13, .16), smoothstep(.2, .0, dx) * .0);
    vec2 w2 = g * .08 + vec2(uTime * .35, uTime * .22);
    float w = fbm(w2) + .5 * fbm(g * .3 - uTime * .4);
    N = normalize(vec3((fbm(w2 + vec2(.5, 0.)) - fbm(w2 - vec2(.5, 0.))) * .6, 1.0, (fbm(w2 + vec2(0., .5)) - fbm(w2 - vec2(0., .5))) * .6));
    wave = w;
    col *= .85 + .3 * w;
    // foam where the water is shallow
    float foam = smoothstep(.0006, .0, dx) * (.6 + .4 * fbm(g * .5 + uTime * .3)) + smoothstep(.004, .0008, dx) * .35 * fbm(g * .2 - uTime * .25);
    col = mix(col, vec3(.9), clamp(foam, 0.0, 1.0));
  }
  vec3 V = normalize(uCam - vW);
  float ndl = max(dot(N, uSun), 0.0);
  vec3 lit = col * (uSunCol * ndl / 3.14159 + uAmb);
  if (land < .5) {
    vec3 H = normalize(uSun + V);
    lit += uSunCol * pow(max(dot(N, H), 0.0), 380.0) * .9 + uAmb * pow(1.0 - max(dot(N, V), 0.0), 4.0) * .35;
  }
  float dist = length(uCam - vW);
  float fog = 1.0 - exp(-dist * 0.00007 * exp(-uCamAlt / 2500.0));      // the haze lives in the lowest kilometres: from high up there is less of it between the eye and the ground
  lit = mix(lit, uHaze, clamp(fog, 0.0, 1.0));
  float a = (1.0 - smoothstep(18000.0, 42000.0, dist)) * uFade;
  // with a picture of the real ground under the sky shader, this flat made-up plane is only the pad's own apron and what is round it: it gives way to the picture beyond a few hundred metres
  a *= mix(1.0, 1.0 - smoothstep(uMountR * 14.0, uMountR * 80.0, d), uImgOn);
  gl_FragColor = vec4(lit, a);
  #include <tonemapping_fragment>
  #include <colorspace_fragment>
}`;

export class Pad {
  /** vehicleLength and diameter size the site; mountHeight is where the vehicle's tail stands above the ground. */
  /** opts.second: the site of a second pad (no ground of its own: it stands on the first pad's); opts.armY: the height of the catch arms in metres (the mission's `arm_height_m`). */
  constructor(length, diameter, mountHeight, coastM = 1600, opts = {}) {
    this.opts = opts;
    this.group = new THREE.Group(); this.group.name = opts.second ? 'second pad' : 'pad';
    this.length = length; this.diameter = diameter; this.mountH = mountHeight;
    const scale = Math.max(0.12, Math.min(1, length / 123));
    this.scale = scale;
    if (opts.second) { this.structures = new THREE.Group(); this.group.add(this.structures); this.build(length, diameter, mountHeight, scale); return; }
    this.groundMat = new THREE.ShaderMaterial({
      vertexShader: GROUND_VERT, fragmentShader: GROUND_FRAG, transparent: true, depthWrite: true, side: THREE.DoubleSide,
      uniforms: { uSun: { value: new THREE.Vector3(0, 1, 0) }, uSunCol: { value: new THREE.Vector3(1, 1, 1) }, uAmb: { value: new THREE.Vector3(.1, .12, .2) }, uHaze: { value: new THREE.Vector3(.5, .6, .75) }, uCam: { value: new THREE.Vector3() },
        uCoast: { value: coastM }, uTime: { value: 0 }, uFade: { value: 1 }, uCamAlt: { value: 0 }, uImgOn: { value: 0 }, uMountR: { value: Math.max(2.5, diameter * 1.1) }, uScale: { value: Math.max(0.3, scale * 3) }, uPadPos: { value: new THREE.Vector3() } },
    });
    const gp = new THREE.PlaneGeometry(90000, 90000, 1, 1); gp.rotateX(-Math.PI / 2);
    this.ground = new THREE.Mesh(gp, this.groundMat); this.ground.position.y = 0; this.ground.renderOrder = 0; this.ground.frustumCulled = false;
    this.group.add(this.ground);
    this.structures = new THREE.Group(); this.group.add(this.structures);
    this.build(length, diameter, mountHeight, scale);
  }

  build(L, D, H, s) {
    const S = this.structures;
    const steel = new THREE.MeshStandardMaterial({ color: 0x6b7077, metalness: 0.7, roughness: 0.55 });
    const dkSteel = new THREE.MeshStandardMaterial({ color: 0x3d4248, metalness: 0.75, roughness: 0.5 });
    const concrete = new THREE.MeshStandardMaterial({ color: 0x8a8b88, metalness: 0.05, roughness: 0.9 });
    const white = new THREE.MeshStandardMaterial({ color: 0xdcdedf, metalness: 0.2, roughness: 0.55 });
    const orange = new THREE.MeshStandardMaterial({ color: 0xc3541b, metalness: 0.3, roughness: 0.55 });
    const yellow = new THREE.MeshStandardMaterial({ color: 0xd9a21b, metalness: 0.3, roughness: 0.55 });
    const beacon = new THREE.MeshBasicMaterial({ color: 0xff3b30 });
    const R = D / 2;
    this.clamps = []; this.qd = []; this.released = 0;
    if (!this.opts.second) {
      // ---- the launch mount ("stage 0"): a table on six legs over a flame trench, twenty hold-down clamps round the booster's tail, a deflector under it
      const ro = D * 1.3, ri = D * 0.68, tH = H - 0.4;                                                // the table's top
      const slab = new THREE.Mesh(new THREE.CylinderGeometry(D * 3.4, D * 3.6, 0.5, 48), concrete); slab.position.y = 0.25; S.add(slab);       // the foundation
      const tableProfile = [new THREE.Vector2(ri, 0), new THREE.Vector2(ro, 0), new THREE.Vector2(ro, D * 0.22), new THREE.Vector2(ri, D * 0.22), new THREE.Vector2(ri, 0)];
      const table = new THREE.Mesh(new THREE.LatheGeometry(tableProfile, 48), steel); table.position.y = tH - D * 0.22; S.add(table);
      const rim = new THREE.Mesh(new THREE.TorusGeometry(ro, 0.22, 8, 64), dkSteel); rim.rotation.x = Math.PI / 2; rim.position.y = tH; S.add(rim);
      const legR = ro * 0.84, legW = Math.max(0.5, D * 0.12), legH = tH - D * 0.22;
      const feet = [];
      for (let i = 0; i < 6; i++) {
        const a = (i / 6) * Math.PI * 2 + 0.3, lx = Math.cos(a) * legR, lz = Math.sin(a) * legR; feet.push([lx, lz]);
        for (const [dx, dz] of [[-1, -1], [1, -1], [1, 1], [-1, 1]]) S.add(bar(new THREE.Vector3(lx + dx * legW * 0.6, 0.4, lz + dz * legW * 0.6), new THREE.Vector3(lx + dx * legW * 0.45, legH, lz + dz * legW * 0.45), legW * 0.28, steel));
        for (let k = 0; k < 3; k++) { const y0 = 0.4 + (legH * k) / 3, y1 = 0.4 + (legH * (k + 1)) / 3; S.add(bar(new THREE.Vector3(lx - legW * 0.6, y0, lz - legW * 0.6), new THREE.Vector3(lx + legW * 0.45, y1, lz + legW * 0.45), legW * 0.12, dkSteel)); }
      }
      for (let i = 0; i < 6; i++) { const A = feet[i], B = feet[(i + 1) % 6]; S.add(bar(new THREE.Vector3(A[0], legH * 0.5, A[1]), new THREE.Vector3(B[0], legH * 0.5, B[1]), legW * 0.16, dkSteel)); }     // the ring beam between the legs
      const trench = new THREE.Mesh(new THREE.CylinderGeometry(D * 0.9, D * 1.3, 0.6, 40, 1, true), new THREE.MeshStandardMaterial({ color: 0x1d1c1b, roughness: 0.9, side: THREE.DoubleSide })); trench.position.y = 0.3; S.add(trench);
      const deflector = new THREE.Mesh(new THREE.ConeGeometry(D * 1.0, D * 0.55, 32), new THREE.MeshStandardMaterial({ color: 0x3b3a39, roughness: 0.8 })); deflector.position.y = D * 0.27 + 0.4; S.add(deflector);
      // the clamps: a base on the table, an arm that reaches over the booster's tail ring and a hook; they fall away outward when the vehicle is released
      for (let i = 0; i < 20; i++) {
        const a = (i / 20) * Math.PI * 2, holder = new THREE.Group(); holder.rotation.y = a; holder.position.set(0, tH, 0); S.add(holder);   // (local +z is radially out)
        const base = new THREE.Mesh(new THREE.BoxGeometry(0.9, 1.0, 1.0), dkSteel); base.position.set(0, 0.5, R + 1.1); holder.add(base);
        const pivot = new THREE.Group(); pivot.position.set(0, 1.0, R + 1.1); holder.add(pivot);
        const arm = new THREE.Mesh(new THREE.BoxGeometry(0.55, 0.45, 1.9), orange); arm.position.set(0, 0, -0.85); pivot.add(arm);
        const hook = new THREE.Mesh(new THREE.BoxGeometry(0.6, 0.6, 0.5), yellow); hook.position.set(0, -0.1, -1.7); pivot.add(hook);
        const ram = new THREE.Mesh(new THREE.CylinderGeometry(0.12, 0.12, 1.3, 8), steel); ram.rotation.x = Math.PI / 2 - 0.5; ram.position.set(0, 0.2, 0.0); pivot.add(ram);
        this.clamps.push({ pivot, delay: 0.25 * ((i * 7) % 20) / 20, e: 0 });
      }
    }
    // ---- the tower: four legs, bracing every few metres, an elevator, catwalks, two arms where the vehicle's top stage is, a lightning rod
    const TH = L * 1.19, tw = Math.max(3, L * 0.075), tx = Math.max(24, D * 3.4 + tw), tz = 0;
    const tower = new THREE.Group(); tower.position.set(this.opts.second ? 0 : tx, 0, this.opts.second ? tx : tz); S.add(tower);
    if (this.opts.second) tower.rotation.y = -Math.PI / 2;                      // a catch pad's tower is downrange of the stage, its arms either side of it: they take the two flank fins' pins (the stage's spine fin faces downrange)
    const col = Math.max(0.5, tw * 0.09), step = Math.max(2.5, tw * 0.8);
    for (const sx of [-1, 1]) for (const sz of [-1, 1]) tower.add(bar(new THREE.Vector3(sx * tw / 2, 0, sz * tw / 2), new THREE.Vector3(sx * tw / 2, TH, sz * tw / 2), col, steel));
    for (let y = 0; y < TH; y += step) {
      const y1 = Math.min(TH, y + step);
      for (const [a, b] of [[[-1, -1], [1, -1]], [[1, -1], [1, 1]], [[1, 1], [-1, 1]], [[-1, 1], [-1, -1]]]) {
        tower.add(bar(new THREE.Vector3(a[0] * tw / 2, y, a[1] * tw / 2), new THREE.Vector3(b[0] * tw / 2, y, b[1] * tw / 2), col * 0.55, steel));
        tower.add(bar(new THREE.Vector3(a[0] * tw / 2, y, a[1] * tw / 2), new THREE.Vector3(b[0] * tw / 2, y1, b[1] * tw / 2), col * 0.4, steel));
      }
    }
    for (let y = TH * 0.22; y < TH; y += TH * 0.22) { const deck = new THREE.Mesh(new THREE.BoxGeometry(tw * 1.5, 0.2, tw * 1.5), dkSteel); deck.position.y = y; tower.add(deck); }       // catwalk decks
    for (const sz of [-1, 1]) { const rail = new THREE.Mesh(new THREE.BoxGeometry(col * 0.6, TH, col * 0.6), orange); rail.position.set(-tw * 0.5 - col, TH / 2, sz * tw * 0.3); tower.add(rail); }   // the rails the carriage rides on
    const rod = new THREE.Mesh(new THREE.CylinderGeometry(0.15, 0.4, TH * 0.07, 8), steel); rod.position.y = TH + TH * 0.035; tower.add(rod);
    const cap = new THREE.Mesh(new THREE.BoxGeometry(tw * 1.2, tw * 0.5, tw * 1.2), white); cap.position.y = TH + tw * 0.15; tower.add(cap);
    const crane = new THREE.Mesh(new THREE.BoxGeometry(tw * 2.6, tw * 0.18, tw * 0.3), orange); crane.position.set(-tw * 0.9, TH + tw * 0.5, 0); tower.add(crane);
    // the arms (the catch arms of a tower that catches the booster): two truss beams on a carriage that rides up and down the tower, at the height the file gives (`armY`, the pins' height at the catch). They reach to the
    // vehicle's axis and pass either side of it, a pin's width outside the hull, with their top at the pins' height, and close on the pins when `setArms(1)` is asked; open, they swing out from the tower about their roots
    // (a stage comes down between them), the launch pad's wide, as the arms of a tower that is not about to catch anything are.
    const armLen = tx - tw * 0.5, armH = Math.max(1.2, D * 0.3), armW = Math.max(1.3, D * 0.28);
    const armY = this.opts.armY ?? L * 0.74, top = armY - 0.3;
    const dark = new THREE.MeshStandardMaterial({ color: 0x4a4f56, metalness: 0.7, roughness: 0.5 });
    this.arms = [];
    for (const side of [-1, 1]) {
      const arm = new THREE.Group(); arm.position.set(-tw * 0.5, top - armH / 2, side * (R + 1.2)); tower.add(arm);        // the root, on the tower's face; the arm swings about it
      const beam = new THREE.Mesh(new THREE.BoxGeometry(armLen, armH * 0.42, armW * 0.45), dark); beam.position.set(-armLen / 2, armH * 0.29, 0); arm.add(beam);        // the top chord
      const beam2 = new THREE.Mesh(new THREE.BoxGeometry(armLen, armH * 0.42, armW * 0.45), dark); beam2.position.set(-armLen / 2, -armH * 0.29, 0); arm.add(beam2);   // the bottom chord
      for (let i = 0; i <= 18; i++) { const x = -(armLen * i) / 18; arm.add(bar(new THREE.Vector3(x, -armH * 0.3, 0), new THREE.Vector3(x - armLen / 18, armH * 0.3 * (i % 2 ? -1 : 1), 0), armH * 0.07, steel)); }     // the diagonals
      const tip = new THREE.Mesh(new THREE.BoxGeometry(armW * 1.4, armH * 1.5, armW * 1.3), steel); tip.position.set(-armLen, 0.1, 0); arm.add(tip);
      const cup = new THREE.Mesh(new THREE.BoxGeometry(armW * 1.0, armH * 0.35, armW * 1.0), orange); cup.position.set(-armLen, armH * 0.9, 0); arm.add(cup);     // where the pin rests
      const pad = new THREE.Mesh(new THREE.BoxGeometry(armW * 0.9, armH * 0.12, armW * 0.8), new THREE.MeshStandardMaterial({ color: 0x1c1d1f, roughness: 0.95 })); pad.position.set(-armLen, armH * 1.1, 0); arm.add(pad);   // the pad's lining
      this.arms.push({ g: arm, side });
    }
    this.armClose = 0; this.armR = R; this.armOpen = this.opts.second ? 0.17 : 0.42;
    const car = new THREE.Mesh(new THREE.BoxGeometry(tw * 0.6, D * 1.6, tw * 1.05), orange); car.position.set(-tw * 0.1, armY - armH / 2, 0); tower.add(car);
    this.setArms(0, 10);
    S.traverse((o) => { if (o.isMesh) { o.castShadow = true; o.receiveShadow = true; } });
    if (!this.opts.second) {
      // the quick-disconnect arms: two booms from the tower to the vehicle (the booster's and the ship's propellant lines, power and data), that swing back from the vehicle when it is released
      for (const [fy, len] of [[0.52, 1.0], [0.86, 1.0]]) {
        const q = new THREE.Group(); q.position.set(-tw * 0.5, L * fy + H, tw * 0.28 * (fy > 0.7 ? -1 : 1)); tower.add(q);
        const bl = tx - tw * 0.5 - R + 0.2;
        const boom = new THREE.Mesh(new THREE.BoxGeometry(bl, 0.7, 0.7), yellow); boom.position.set(-bl / 2, 0, 0); q.add(boom);
        for (let i = 0; i < 8; i++) q.add(bar(new THREE.Vector3(-(bl * i) / 8, -0.9, 0), new THREE.Vector3(-(bl * (i + 0.5)) / 8, 0, 0), 0.08, dkSteel));
        const head = new THREE.Mesh(new THREE.CylinderGeometry(0.45, 0.45, 1.1, 14), dkSteel); head.rotation.z = Math.PI / 2; head.position.set(-bl, 0, 0); q.add(head);
        this.qd.push({ g: q, dir: fy > 0.7 ? -1 : 1 });
      }
      // lightning masts, a tank farm (spheres and vertical tanks) and a control building
      for (const [mx, mz] of [[-170, -150], [170, -150], [-170, 170], [170, 170]].map(([a, b]) => [a * Math.max(.4, s * 1.1), b * Math.max(.4, s * 1.1)])) {
        const mast = new THREE.Mesh(new THREE.CylinderGeometry(0.25, 0.7, L * 1.0, 8), steel); mast.position.set(mx, L * 0.5, mz); S.add(mast);
        const b = new THREE.Mesh(new THREE.SphereGeometry(0.7, 8, 6), beacon); b.position.set(mx, L * 1.0, mz); S.add(b);
      }
      for (let i = 0; i < 5; i++) {
        const r = Math.max(2.2, D * 0.55), h = r * 5;
        const tank = new THREE.Mesh(new THREE.CylinderGeometry(r, r, h, 24), white); tank.position.set(-D * 14 - i * r * 2.6, h / 2, -D * 9); S.add(tank);
        const dome = new THREE.Mesh(new THREE.SphereGeometry(r, 24, 12, 0, Math.PI * 2, 0, Math.PI / 2), white); dome.position.set(tank.position.x, h, tank.position.z); S.add(dome);
        for (const a of [0.6, 2.2, 3.9, 5.4]) S.add(bar(new THREE.Vector3(tank.position.x + Math.cos(a) * r, 0, tank.position.z + Math.sin(a) * r), new THREE.Vector3(tank.position.x + Math.cos(a) * r * 0.9, h * 0.35, tank.position.z + Math.sin(a) * r * 0.9), 0.12, steel));
      }
      for (let i = 0; i < 3; i++) {                                                      // spheres on their legs
        const r = Math.max(2.6, D * 0.65), sp = new THREE.Mesh(new THREE.SphereGeometry(r, 28, 18), white); sp.position.set(D * 16 + i * r * 2.7, r * 1.9, -D * 10); S.add(sp);
        for (let k = 0; k < 6; k++) { const a = (k / 6) * Math.PI * 2; S.add(bar(new THREE.Vector3(sp.position.x + Math.cos(a) * r * 0.8, 0, sp.position.z + Math.sin(a) * r * 0.8), new THREE.Vector3(sp.position.x + Math.cos(a) * r * 0.6, r * 1.3, sp.position.z + Math.sin(a) * r * 0.6), 0.14, steel)); }
      }
      const bld = new THREE.Mesh(new THREE.BoxGeometry(D * 4, D * 0.9, D * 2), concrete); bld.position.set(-D * 12, D * 0.45, D * 7); S.add(bld);
      const roof = new THREE.Mesh(new THREE.BoxGeometry(D * 4.1, D * 0.08, D * 2.1), dark); roof.position.set(-D * 12, D * 0.94, D * 7); S.add(roof);
    }
    if (this.opts.second) {                                  // the first pad's ground draws its apron and its scorch; a second pad has to bring its own
      const apronR = Math.max(2.5, D * 1.1) * 8;
      const apron = new THREE.Mesh(new THREE.CylinderGeometry(apronR, apronR, 0.12, 64), new THREE.MeshStandardMaterial({ color: 0x57554f, roughness: 0.95 })); apron.position.y = 0.06; S.add(apron);
      const scorch = new THREE.Mesh(new THREE.CylinderGeometry(D * 3.6, D * 3.6, 0.14, 48), new THREE.MeshStandardMaterial({ color: 0x1d1c1b, roughness: 1 })); scorch.position.y = 0.07; S.add(scorch);
      // the catch pad's own mount: a low ring of concrete with four steel stands that the stage is set down on if the arms do not hold it
      const ring = new THREE.Mesh(new THREE.TorusGeometry(R * 1.05, 0.5, 8, 48), concrete); ring.rotation.x = Math.PI / 2; ring.position.y = 0.4; S.add(ring);
      for (let k = 0; k < 4; k++) { const a = (k / 4) * Math.PI * 2 + Math.PI / 4; const st = new THREE.Mesh(new THREE.BoxGeometry(1.2, 1.4, 1.2), dkSteel); st.position.set(Math.cos(a) * R * 0.9, 0.7, Math.sin(a) * R * 0.9); S.add(st); }
    }
  }

  /** The hold-down clamps and the quick-disconnect arms: 1 when the vehicle has been released (they have fallen away / swung back), 0 when it is held; moved `dt` seconds' worth, in about a second and a half. */
  setRelease(target, dt = 0.016) {
    if (!this.clamps) return;
    const step = Math.min(dt, 0.1) / 1.2;
    this.released += Math.max(-step, Math.min(step, target - this.released));
    for (const c of this.clamps) {
      const e = Math.max(0, Math.min(1, (this.released - c.delay) / (1 - 0.25)));
      const k = e * e * (3 - 2 * e);
      c.pivot.rotation.x = -k * 1.75;                                    // the arm falls outward and down: its tip, which reaches in over the booster's tail, swings away
    }
    const q = Math.max(0, Math.min(1, (this.released - 0.2) / 0.8)), eq = q * q * (3 - 2 * q);
    for (const b of this.qd) b.g.rotation.y = b.dir * eq * 1.0;
  }

  /** The arms toward the vehicle (1: closed on the pins) or away (0: open, swung out about their roots), moved `dt` seconds' worth at about 0.4 per second. */
  setArms(target, dt = 0.016) {
    if (!this.arms) return;
    const step = Math.min(dt, 0.1) * 0.4;
    this.armClose += Math.max(-step, Math.min(step, target - this.armClose));
    const e = this.armClose * this.armClose * (3 - 2 * this.armClose);
    for (const a of this.arms) a.g.rotation.y = a.side * (1 - e) * this.armOpen;      // (an arm at +z swings toward +z as it opens)
  }
}
