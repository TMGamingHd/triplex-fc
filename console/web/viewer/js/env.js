// The world around the vehicle: the sky, the atmosphere, the planet and the stars, drawn by one fragment shader from the camera's place above the planet's centre; the ground around the launch pad; and the
// light the vehicle is lit with. It is the *far* scene: the vehicle and what is close to it are drawn after it, on top, with their own depth (main.js).
//
// What is physical here and what is made up. The atmosphere is single scattering by Rayleigh and Mie particles with exponential density (scale heights 8 and 1.2 km): the blue sky, the red sunset and the thin bright
// limb seen from space come out of it, and its thickness follows the simulator's planet. The planet's surface is *procedural*: a coast at the launch point, ocean downrange, land behind, clouds that move, ice at the
// poles. It is not a map of any real place (the viewer has no imagery to use offline); an imagery pack can replace it (docs/design/VIEWER.md).
import * as THREE from '../vendor/three.module.js';

const SKY_VERT = /* glsl */`
varying vec2 vUv;
void main() { vUv = uv; gl_Position = vec4(position.xy, 0.0, 1.0); }
`;

export const NOISE_GLSL = /* glsl */`
float hash31(vec3 p) { p = fract(p * .1031); p += dot(p, p.zyx + 31.32); return fract((p.x + p.y) * p.z); }
float vnoise(vec3 p) {
  vec3 i = floor(p), f = fract(p); f = f * f * (3.0 - 2.0 * f);
  return mix(mix(mix(hash31(i), hash31(i + vec3(1,0,0)), f.x), mix(hash31(i + vec3(0,1,0)), hash31(i + vec3(1,1,0)), f.x), f.y),
             mix(mix(hash31(i + vec3(0,0,1)), hash31(i + vec3(1,0,1)), f.x), mix(hash31(i + vec3(0,1,1)), hash31(i + vec3(1,1,1)), f.x), f.y), f.z);
}
`;

const SKY_FRAG = /* glsl */`
precision highp float;
varying vec2 vUv;
uniform vec3 uCamPos;          // km from the planet's centre, in scene axes
uniform mat3 uCamRot;          // camera to world
uniform vec2 uTan;             // tan(fov/2) * aspect, tan(fov/2)
uniform vec3 uSun;             // unit vector to the sun
uniform float uRp;             // the planet's radius, km
uniform float uHa;             // the top of the atmosphere above the surface, km
uniform mat3 uPlanetRot;       // world axes -> the planet's own (it turns)
uniform float uTime;           // s
uniform float uCloud;          // 0..1 cover
uniform float uSunI;
uniform float uStars;
uniform float uExposure;
uniform float uRayleighScale;  // 1 for Earth's air, 0 for a planet without
uniform vec3 uPole;            // the planet's axis of rotation, in its own frame
uniform vec3 uUp0;             // the launch point's direction in the planet's frame
uniform vec3 uDown0;           // downrange there
uniform float uQuality;        // 0 low .. 1 high: the number of samples
uniform float uCoastKm;        // where the shore is, km downrange of the pad
uniform vec3 uDebug;
uniform vec4 uShell[8];       // xyz: the colour of a shell of the atmosphere; w: its radius in km (0: none)
uniform float uShellAlpha;

const float PI = 3.14159265;
const vec3 BETA_R = vec3(5.8e-3, 13.5e-3, 33.1e-3);  // per km at sea level
const float BETA_M = 21e-3;
const float HR = 8.0;
const float HM = 1.2;
const float G_M = 0.76;

float hash11(float p) { p = fract(p * .1031); p *= p + 33.33; p *= p + p; return fract(p); }
${NOISE_GLSL}
vec3 hash33(vec3 p) { p = fract(p * vec3(.1031, .1030, .0973)); p += dot(p, p.yxz + 33.33); return fract((p.xxy + p.yxx) * p.zyx); }
float fbm(vec3 p, int oct) {
  float a = .5, s = 0.0;
  for (int i = 0; i < 8; i++) { if (i >= oct) break; s += a * vnoise(p); p = p * 2.03 + vec3(1.7, 9.2, 3.1); a *= .5; }
  return s;
}

// the two distances at which a ray meets a sphere about the origin (x > y: none)
vec2 raySphere(vec3 ro, vec3 rd, float r) {
  float b = dot(ro, rd), c = dot(ro, ro) - r * r, d = b * b - c;
  if (d < 0.0) return vec2(1.0, -1.0);
  d = sqrt(d); return vec2(-b - d, -b + d);
}

float phaseR(float mu) { return 3.0 / (16.0 * PI) * (1.0 + mu * mu); }
float phaseM(float mu) { float g = G_M, g2 = g * g; return 3.0 / (8.0 * PI) * (1.0 - g2) * (1.0 + mu * mu) / ((2.0 + g2) * pow(1.0 + g2 - 2.0 * g * mu, 1.5)); }

// the optical depth of the air from a point toward the sun, and whether the planet is in the way
vec3 sunOptical(vec3 p, out float blocked) {
  vec2 hp = raySphere(p, uSun, uRp);
  blocked = (hp.x < hp.y && hp.x > 0.0) ? 1.0 : 0.0;           // a miss is (1, -1): x > y
  float tl = raySphere(p, uSun, uRp + uHa).y;
  const int NL = 5;
  float dl = tl / float(NL), odR = 0.0, odM = 0.0;
  for (int j = 0; j < NL; j++) {
    vec3 pl = p + uSun * ((float(j) + .5) * dl);
    float h = max(length(pl) - uRp, 0.0);
    odR += exp(-h / HR) * dl; odM += exp(-h / HM) * dl;
  }
  return vec3(odR, odM, 0.0);
}

vec3 sunTransmittance(vec3 p) {
  float bl; vec3 od = sunOptical(p, bl);
  return (bl > 0.5 ? 0.0 : 1.0) * exp(-(BETA_R * od.x * uRayleighScale + BETA_M * 1.1 * od.y * uRayleighScale));
}

// the single-scattered light along a ray (ro, rd) from t0 to t1, and the fraction that gets through. The samples are spread so that the dense air gets most of them: near the eye when the eye is in the air,
// near the far end (the limb, the ground) when it is looking in from space.
vec3 atmosphere(vec3 ro, vec3 rd, float t0, float t1, float jit, out vec3 T) {
  T = vec3(1.0);
  if (t1 <= t0 || uRayleighScale <= 0.0) return vec3(0.0);
  int N = int(mix(18.0, 48.0, uQuality));
  float span = t1 - t0;
  bool inside = length(ro) < uRp + uHa;
  float k = inside ? 2.2 : 1.8;
  vec3 sumR = vec3(0.0), sumM = vec3(0.0);
  float odR = 0.0, odM = 0.0;
  float mu = dot(rd, uSun);
  float uPrev = 0.0;
  for (int i = 0; i < 48; i++) {
    if (i >= N) break;
    float u = (float(i) + jit) / float(N);
    float un = (float(i) + 1.0) / float(N);
    // t(u) = pow(u, k) (inside) or 1 - pow(1 - u, k) (outside); ds is the width of this sample's cell
    float a = inside ? pow(u, k) : 1.0 - pow(1.0 - u, k);
    float b = inside ? pow(un, k) : 1.0 - pow(1.0 - un, k);
    float ds = (b - (inside ? pow(float(i) / float(N), k) : 1.0 - pow(1.0 - float(i) / float(N), k))) * span;
    vec3 p = ro + rd * (t0 + a * span);
    float h = max(length(p) - uRp, 0.0);
    float hr = exp(-h / HR) * ds, hm = exp(-h / HM) * ds;
    odR += hr; odM += hm;
    float bl; vec3 od = sunOptical(p, bl);
    if (bl > 0.5) continue;
    vec3 tau = BETA_R * (odR + od.x) + BETA_M * 1.1 * (odM + od.y);
    vec3 att = exp(-tau * uRayleighScale);
    sumR += hr * att; sumM += hm * att;
  }
  T = exp(-(BETA_R * odR + BETA_M * 1.1 * odM) * uRayleighScale);
  return uSunI * (sumR * BETA_R * phaseR(mu) + sumM * BETA_M * phaseM(mu)) * uRayleighScale;
}

// ---- the surface -------------------------------------------------------------------------------------------------------------------
// the launch point's neighbourhood in km: x downrange, y crossrange, from the planet-fixed direction pp
vec2 localKm(vec3 pp) {
  float up = dot(pp, uUp0);
  vec3 cross0 = normalize(cross(uUp0, uDown0));
  return vec2(dot(pp, uDown0), dot(pp, cross0)) / max(up, 0.2) * uRp;
}

// 1: land, 0: sea, with the coast at the pad and the continents elsewhere
float landMask(vec3 pp, out float shore) {
  vec2 l = localKm(pp);
  float angDist = acos(clamp(dot(pp, uUp0), -1.0, 1.0)) * uRp;       // km from the pad
  float coast = uCoastKm + 2.2 * (vnoise(vec3(l.y * .09, 1.3, 2.1)) - .5) + .35 * (vnoise(vec3(l.y * 1.1, 4.0, .5)) - .5);
  float local = 1.0 - smoothstep(-.05, .05, l.x - coast);
  float cont = fbm(pp * 2.4 + vec3(3.1, 1.7, 8.2), 6);
  float globalLand = smoothstep(.50, .56, cont);
  float w = smoothstep(1800.0, 4200.0, angDist);
  float m = mix(local, globalLand, w);
  // a long thin shelf of shallow water off the pad's coast
  shore = (1.0 - w) * exp(-abs(l.x - coast) * .7);
  return m;
}

vec3 surface(vec3 pp, vec3 N, vec3 V, vec3 sunT, float ndl, out float isLand) {
  float shore;
  float land = landMask(pp, shore);
  isLand = land;
  float lat = abs(dot(pp, uPole));
  float rough = fbm(pp * 90.0, 5);
  float elev = fbm(pp * 18.0 + 5.0, 6);
  // land: green where it is wet and low, tan where dry and high, white where it is cold
  vec3 green = vec3(.08, .15, .05), tan = vec3(.30, .25, .15), rock = vec3(.2, .18, .16), sand = vec3(.55, .5, .38), snow = vec3(.85, .88, .92);
  vec3 lc = mix(green, tan, smoothstep(.42, .62, vnoise(pp * 11.0) * .6 + rough * .5));
  lc = mix(lc, rock, smoothstep(.62, .78, elev));
  vec2 l = localKm(pp);
  lc = mix(lc, sand, shore * smoothstep(-1.5, .2, l.x - uCoastKm) * .0 + shore * .6);
  lc *= .8 + .4 * rough;
  float ice = smoothstep(.78, .88, lat + .06 * (rough - .5));
  lc = mix(lc, snow, ice);
  // sea: dark blue in deep water, turquoise on the shelf, a glint of the sun
  vec3 deep = vec3(.006, .03, .09), shallow = vec3(.03, .17, .22);
  vec3 sc = mix(deep, shallow, clamp(shore * 1.4, 0.0, 1.0));
  sc = mix(sc, snow * .9, ice);
  vec3 alb = mix(sc, lc, land);
  vec3 H = normalize(uSun + V);
  vec3 nrm = normalize(N + (1.0 - land) * (1.0 - ice) * .02 * (vec3(vnoise(pp * 1400.0), vnoise(pp * 1500.0 + 3.0), vnoise(pp * 1300.0 + 7.0)) - .5));
  float spec = pow(max(dot(nrm, H), 0.0), 600.0) * (1.0 - land) * (1.0 - ice) * 6.0;
  vec3 skyAmb = vec3(.05, .07, .11) * (.25 + .75 * max(dot(N, uSun) * .5 + .5, 0.0));
  vec3 col = alb * (uSunI * sunT * ndl / PI + skyAmb * uSunI * .35 * length(sunT)) + spec * uSunI * sunT * .25;
  // the night side: a few lights on the land near the sea
  float night = 1.0 - smoothstep(-.12, .05, ndl);
  float lights = step(.80, vnoise(pp * 700.0) * .5 + vnoise(pp * 1900.0) * .5) * land * (1.0 - ice) * smoothstep(.45, .6, fbm(pp * 22.0 + 3.0, 4));
  col += vec3(1.0, .78, .45) * lights * night * 3.0;
  return col;
}

// the clouds: a thin layer 3 km up, drifting, thicker where the weather is
float cloudAt(vec3 pp) {
  float t = uTime * .00035;
  vec3 q = pp + vec3(t, 0.0, t * .6);
  float big = fbm(q * 5.0 + 4.0, 4);
  float cov = mix(.85, .35, uCloud) + .0;                                         // the threshold: less cover, higher threshold
  float c = fbm(q * 55.0 + vec3(1.0, 2.0, 3.0), 5) * .6 + big * .55;
  return smoothstep(cov, cov + .22, c);
}

vec3 stars(vec3 rd) {
  vec3 col = vec3(0.0);
  for (int L = 0; L < 3; L++) {
    float sc = L == 0 ? 55.0 : (L == 1 ? 130.0 : 280.0);
    vec3 q = rd * sc, id = floor(q), f = fract(q) - .5;
    vec3 off = (hash33(id) - .5) * .7;
    float h = hash31(id + float(L) * 17.0);
    float d = length(f - off);
    float mag = pow(h, 6.0);
    float s = (1.0 - smoothstep(0.0, .06 + .05 * mag, d)) * step(.86 - float(L) * .02, h) * (.25 + 1.6 * mag);
    vec3 tint = mix(vec3(.65, .78, 1.0), vec3(1.0, .82, .6), hash31(id + 3.0));
    col += tint * s;
  }
  // the Milky Way: a band of faint cloud around a tilted great circle
  vec3 gal = normalize(vec3(.3, .85, .45));
  float b = exp(-pow(dot(rd, gal) * 3.2, 2.0));
  float mw = b * (.15 + .85 * fbm(rd * 7.0 + 2.0, 5)) * (.5 + .5 * fbm(rd * 2.0, 3));
  col += vec3(.7, .75, 1.0) * mw * .35 + vec3(1.0, .85, .6) * b * b * fbm(rd * 3.0 + 9.0, 3) * .15;
  return col;
}

void main() {
  vec2 ndc = vUv * 2.0 - 1.0;
  vec3 rd = normalize(uCamRot * vec3(ndc.x * uTan.x, ndc.y * uTan.y, -1.0));
  vec3 ro = uCamPos;
  float jit = fract(52.9829189 * fract(dot(gl_FragCoord.xy, vec2(0.06711056, 0.00583715))));   // interleaved gradient noise: the same pattern every frame, which the eye does not see as noise
  float Ra = uRp + uHa;
  vec2 hs = raySphere(ro, rd, uRp);
  vec2 ha = raySphere(ro, rd, Ra);
  bool hitGround = hs.x < hs.y && hs.x > 0.0;
  float tg = hs.x > 0.0 ? hs.x : 0.0;
  float tEnd = hitGround ? tg : ha.y;
  float t0 = max(ha.x, 0.0);
  vec3 T;
  vec3 scat = atmosphere(ro, rd, t0, tEnd, jit, T);
  vec3 col = vec3(0.0);
  float mu = dot(rd, uSun);
  if (hitGround) {
    vec3 p = ro + rd * tg;
    vec3 N = normalize(p);
    vec3 pp = uPlanetRot * N;
    float ndl = max(dot(N, uSun), 0.0);
    vec3 sunT = sunTransmittance(p + N * .001);
    float land;
    vec3 gcol = surface(pp, N, -rd, sunT, ndl, land);
    // clouds between the ground and the eye
    float cloudA = 0.0; vec3 ccol = vec3(0.0);
    float Rc = uRp + 3.0;
    vec2 hc = raySphere(ro, rd, Rc);
    float tc = length(ro) > Rc ? hc.x : hc.y;
    if (hc.x < hc.y && tc > 0.0 && tc < tg + 1.0) {
      vec3 pc = ro + rd * tc; vec3 Nc = normalize(pc);
      vec3 ppc = uPlanetRot * Nc;
      cloudA = cloudAt(ppc) * (uCloud > 0.001 ? 1.0 : 0.0);
      float ndlc = dot(Nc, uSun);
      // the lit tops, the shadowed bases: brighter where the cloud is thinner toward the sun
      float toSun = cloudAt(normalize(ppc + (uPlanetRot * uSun) * .004));
      float lit = clamp(.55 + .9 * ndlc - .6 * (toSun - cloudA) * 0.0 - .35 * toSun * (1.0 - cloudA * .5), 0.0, 1.5);
      float below = length(ro) < Rc ? 1.0 : 0.0;
      vec3 sT = sunTransmittance(pc);
      ccol = vec3(1.0) * uSunI * (sT * max(ndlc, 0.0) / PI * (1.0 - .4 * below) * .95 + vec3(.06, .09, .15) * .6 * length(sT) * (.4 + .6 * clamp(ndlc + .3, 0.0, 1.0))) * (.75 + .25 * lit);
    }
    col = mix(gcol, ccol, cloudA);
    col = col * T + scat;
  } else {
    // the sky: the stars, the sun, and the scattered light between
    vec3 sky = vec3(0.0);
    float lum = dot(scat, vec3(.3, .6, .1));
    float dark = exp(-lum * 18.0);
    sky += stars(rd) * uStars * dark;
    float sd = smoothstep(.99996, .999985, mu);                       // the sun's disc, 0.53 degrees
    sky += vec3(1.0, .93, .8) * sd * uSunI * 120.0;
    // clouds in the sky (from below or from inside)
    float Rc = uRp + 3.0;
    if (length(ro) < Rc + 8.0) {
      vec2 hc = raySphere(ro, rd, Rc);
      float tc = length(ro) > Rc ? hc.x : hc.y;
      if (hc.x < hc.y && tc > 0.0 && tc < 300.0) {
        vec3 pc = ro + rd * tc; vec3 Nc = normalize(pc); vec3 ppc = uPlanetRot * Nc;
        float a = cloudAt(ppc);
        float ndlc = dot(Nc, uSun);
        vec3 sT = sunTransmittance(pc);
        vec3 cc = vec3(1.0) * uSunI * (sT * max(ndlc + .15, 0.0) / PI * .5 + vec3(.07, .1, .17) * .5 * length(sT));
        float fade = exp(-tc * .004);
        sky = mix(sky, cc, a * fade * float(uCloud > 0.001));
      }
    }
    col = sky * T + scat;
  }
  if (uShellAlpha > 0.001) {                                // the atmosphere's layers as faint shells, brighter where seen edge-on
    for (int i = 0; i < 8; i++) {
      float Rs = uShell[i].w;
      if (Rs <= 0.0) continue;
      vec2 hh = raySphere(ro, rd, Rs);
      if (hh.x >= hh.y) continue;
      float t = hh.x > 0.0 ? hh.x : hh.y;
      if (t <= 0.0 || (hitGround && t > tg)) continue;
      vec3 pp2 = ro + rd * t;
      float edge = 1.0 - abs(dot(rd, normalize(pp2)));
      float a = uShellAlpha * (0.05 + 0.75 * pow(edge, 3.0));
      col = mix(col, uShell[i].xyz, clamp(a, 0.0, 0.9));
    }
  }
  if (uDebug.x > 0.5) { if (uDebug.x < 1.5) col = scat * uExposure; else if (uDebug.x < 2.5) col = T; else if (uDebug.x < 3.5) col = vec3(hitGround ? 1.0 : 0.0, ha.y > ha.x ? 1.0 : 0.0, 0.0); else col = vec3(clamp((tEnd - t0) / 1000.0, 0.0, 1.0)); }
  col *= uExposure;
  gl_FragColor = vec4(col, 1.0);
  #include <tonemapping_fragment>
  #include <colorspace_fragment>
}
`;

/** Where the sun is, in the scene's axes, from its elevation and bearing at the launch point (bearing from the downrange direction, degrees). */
export function sunFromLocal(elevDeg, bearingDeg) {
  const el = elevDeg * Math.PI / 180, az = bearingDeg * Math.PI / 180;
  // local axes in the scene: up = +Y (the simulator's X), downrange = +Z (its Y), crossrange = +X (its Z)
  const h = Math.cos(el);
  return new THREE.Vector3(h * Math.sin(az), Math.sin(el), h * Math.cos(az)).normalize();
}

export class Sky {
  constructor() {
    this.mat = new THREE.ShaderMaterial({
      vertexShader: SKY_VERT, fragmentShader: SKY_FRAG, depthTest: false, depthWrite: false,
      uniforms: {
        uCamPos: { value: new THREE.Vector3(0, 6.371, 0) }, uCamRot: { value: new THREE.Matrix3() }, uTan: { value: new THREE.Vector2(1, 1) }, uSun: { value: new THREE.Vector3(0, 1, 0) },
        uRp: { value: 6371.0 }, uHa: { value: 100.0 }, uPlanetRot: { value: new THREE.Matrix3() }, uTime: { value: 0 }, uCloud: { value: 0.35 }, uSunI: { value: 20.0 }, uStars: { value: 1.0 },
        uExposure: { value: 1.0 }, uRayleighScale: { value: 1.0 }, uPole: { value: new THREE.Vector3(0, 1, 0) }, uUp0: { value: new THREE.Vector3(0, 1, 0) }, uDown0: { value: new THREE.Vector3(0, 0, 1) }, uQuality: { value: 0.6 },
        uCoastKm: { value: 1.6 }, uDebug: { value: new THREE.Vector3() }, uShell: { value: Array.from({ length: 8 }, () => new THREE.Vector4(0, 0, 0, 0)) }, uShellAlpha: { value: 0 },
      },
    });
    this.mesh = new THREE.Mesh(new THREE.PlaneGeometry(2, 2), this.mat);
    this.mesh.frustumCulled = false;
    this.scene = new THREE.Scene();
    this.scene.add(this.mesh);
    this.cam = new THREE.OrthographicCamera(-1, 1, 1, -1, 0, 1);
  }
  get u() { return this.mat.uniforms; }
}

/** How bright the daylight is at an altitude with the sun at an elevation (0: night or space, 1: a clear day at sea level). Drives the lights and the exposure of the near scene. */
export function daylight(altM, sunElevDeg) {
  const atmo = Math.exp(-Math.max(altM, 0) / 9000);                 // the sky's brightness falls with the air that scatters it
  const sun = Math.max(0, Math.sin(sunElevDeg * Math.PI / 180));
  const twilight = Math.min(1, Math.max(0, (sunElevDeg + 6) / 12));
  return { sky: atmo * twilight, sun: Math.pow(sun, 0.6) * (0.35 + 0.65 * Math.exp(-0.0 * altM)) };
}
