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
    col = mix(col, vec3(.34, .33, .31) * (.85 + .25 * fine), apron);
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
  constructor(length, diameter, mountHeight, coastM = 1600) {
    this.group = new THREE.Group(); this.group.name = 'pad';
    this.length = length; this.diameter = diameter; this.mountH = mountHeight;
    const scale = Math.max(0.12, Math.min(1, length / 123));
    this.scale = scale;
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
    const concrete = new THREE.MeshStandardMaterial({ color: 0x8a8b88, metalness: 0.05, roughness: 0.9 });
    const white = new THREE.MeshStandardMaterial({ color: 0xdcdedf, metalness: 0.2, roughness: 0.55 });
    const orange = new THREE.MeshStandardMaterial({ color: 0xc3541b, metalness: 0.3, roughness: 0.55 });
    const beacon = new THREE.MeshBasicMaterial({ color: 0xff3b30 });
    // the launch mount: a ring table on legs over a flame trench
    const ro = D * 1.3, ri = D * 0.68;
    const tableProfile = [new THREE.Vector2(ri, 0), new THREE.Vector2(ro, 0), new THREE.Vector2(ro, D * 0.22), new THREE.Vector2(ri, D * 0.22), new THREE.Vector2(ri, 0)];
    const table = new THREE.Mesh(new THREE.LatheGeometry(tableProfile, 48), steel); table.position.y = H - D * 0.22 - 0.4; S.add(table);
    const legR = ro * 0.84, legW = Math.max(0.5, D * 0.14);
    for (let i = 0; i < 6; i++) { const a = (i / 6) * Math.PI * 2 + 0.3; const leg = new THREE.Mesh(new THREE.BoxGeometry(legW, H - D * 0.22 - 0.4, legW), steel); leg.position.set(Math.cos(a) * legR, (H - D * 0.22 - 0.4) / 2, Math.sin(a) * legR); S.add(leg); }
    for (let i = 0; i < 12; i++) { const a = (i / 12) * Math.PI * 2; const clamp = new THREE.Mesh(new THREE.BoxGeometry(D * 0.12, D * 0.14, D * 0.12), orange); clamp.position.set(Math.cos(a) * (D * 0.5 + D * 0.12), H - 0.2, Math.sin(a) * (D * 0.5 + D * 0.12)); S.add(clamp); }
    const trench = new THREE.Mesh(new THREE.CylinderGeometry(D * 0.9, D * 1.3, 0.6, 40, 1, true), new THREE.MeshStandardMaterial({ color: 0x1d1c1b, roughness: 0.9, side: THREE.DoubleSide })); trench.position.y = 0.3; S.add(trench);
    const deflector = new THREE.Mesh(new THREE.ConeGeometry(D * 1.0, D * 0.45, 32), new THREE.MeshStandardMaterial({ color: 0x3b3a39, roughness: 0.8 })); deflector.position.y = D * 0.22; S.add(deflector);
    // the tower: four legs, bracing every few metres, two arms where the vehicle's top stage is, a lightning rod
    const TH = L * 1.19, tw = Math.max(3, L * 0.075), tx = Math.max(24, D * 3.4 + tw), tz = 0;
    const tower = new THREE.Group(); tower.position.set(tx, 0, tz); S.add(tower);
    const col = Math.max(0.5, tw * 0.09), step = Math.max(2.5, tw * 0.8);
    for (const sx of [-1, 1]) for (const sz of [-1, 1]) tower.add(bar(new THREE.Vector3(sx * tw / 2, 0, sz * tw / 2), new THREE.Vector3(sx * tw / 2, TH, sz * tw / 2), col, steel));
    for (let y = 0; y < TH; y += step) {
      const y1 = Math.min(TH, y + step);
      for (const [a, b] of [[[-1, -1], [1, -1]], [[1, -1], [1, 1]], [[1, 1], [-1, 1]], [[-1, 1], [-1, -1]]]) {
        tower.add(bar(new THREE.Vector3(a[0] * tw / 2, y, a[1] * tw / 2), new THREE.Vector3(b[0] * tw / 2, y, b[1] * tw / 2), col * 0.55, steel));
        tower.add(bar(new THREE.Vector3(a[0] * tw / 2, y, a[1] * tw / 2), new THREE.Vector3(b[0] * tw / 2, y1, b[1] * tw / 2), col * 0.4, steel));
      }
    }
    const rod = new THREE.Mesh(new THREE.CylinderGeometry(0.15, 0.4, TH * 0.07, 8), steel); rod.position.y = TH + TH * 0.035; tower.add(rod);
    const cap = new THREE.Mesh(new THREE.BoxGeometry(tw * 1.2, tw * 0.5, tw * 1.2), white); cap.position.y = TH + tw * 0.15; tower.add(cap);
    // the arms (the catch arms of a tower that catches the booster): two beams that reach in to the vehicle at the height of its upper tank, on a carriage that rides up and down the tower
    const armLen = tx - D * 0.5 - tw * 0.5 - 0.6, armH = Math.max(1.2, D * 0.38), armW = Math.max(1.4, D * 0.3);
    const dark = new THREE.MeshStandardMaterial({ color: 0x4a4f56, metalness: 0.7, roughness: 0.5 });
    for (const side of [-1, 1]) {
      const arm = new THREE.Mesh(new THREE.BoxGeometry(armLen, armH, armW), dark);
      arm.position.set(-(armLen / 2) - tw * 0.5, L * 0.74 + side * D * 0.2, side * D * 0.62); tower.add(arm);
      const pad2 = new THREE.Mesh(new THREE.BoxGeometry(armW * 1.4, armH * 1.5, armW * 1.4), steel); pad2.position.set(-armLen - tw * 0.5, L * 0.74 + side * D * 0.2, side * D * 0.62); tower.add(pad2);
      for (let i = 1; i < 6; i++) { const rib = new THREE.Mesh(new THREE.BoxGeometry(armH * 0.25, armH * 1.25, armW * 1.12), steel); rib.position.set(-(armLen * i / 6) - tw * 0.5, L * 0.74 + side * D * 0.2, side * D * 0.62); tower.add(rib); }
    }
    const car = new THREE.Mesh(new THREE.BoxGeometry(tw * 0.6, D * 1.6, tw * 1.05), orange); car.position.set(-tw * 0.1, L * 0.74, 0); tower.add(car);
    // lightning masts and a tank farm
    for (const [mx, mz] of [[-170, -150], [170, -150], [-170, 170], [170, 170]].map(([a, b]) => [a * Math.max(.4, s * 1.1), b * Math.max(.4, s * 1.1)])) {
      const mast = new THREE.Mesh(new THREE.CylinderGeometry(0.25, 0.7, L * 1.0, 8), steel); mast.position.set(mx, L * 0.5, mz); S.add(mast);
      const b = new THREE.Mesh(new THREE.SphereGeometry(0.7, 8, 6), beacon); b.position.set(mx, L * 1.0, mz); S.add(b);
    }
    for (let i = 0; i < 5; i++) {
      const r = Math.max(2.2, D * 0.55), h = r * 5;
      const tank = new THREE.Mesh(new THREE.CylinderGeometry(r, r, h, 24), white); tank.position.set(-D * 14 - i * r * 2.6, h / 2, -D * 9); S.add(tank);
      const dome = new THREE.Mesh(new THREE.SphereGeometry(r, 24, 12, 0, Math.PI * 2, 0, Math.PI / 2), white); dome.position.set(tank.position.x, h, tank.position.z); S.add(dome);
    }
    const bld = new THREE.Mesh(new THREE.BoxGeometry(D * 4, D * 0.9, D * 2), concrete); bld.position.set(-D * 12, D * 0.45, D * 7); S.add(bld);
    const roof = new THREE.Mesh(new THREE.BoxGeometry(D * 4.1, D * 0.08, D * 2.1), dark); roof.position.set(-D * 12, D * 0.94, D * 7); S.add(roof);
  }
}
