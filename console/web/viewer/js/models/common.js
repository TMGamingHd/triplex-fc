// What every model is made of: procedural textures (drawn on canvases, so there is nothing to download), a lathe for bodies of revolution, and the engine.
import * as THREE from '../../vendor/three.module.js';

/** A canvas texture from a drawing function. */
export function canvasTexture(w, h, draw, { repeat = [1, 1], color = false, aniso = 8 } = {}) {
  const c = document.createElement('canvas'); c.width = w; c.height = h;
  draw(c.getContext('2d'), w, h);
  const t = new THREE.CanvasTexture(c);
  t.wrapS = t.wrapT = THREE.RepeatWrapping; t.repeat.set(repeat[0], repeat[1]); t.anisotropy = aniso;
  if (color) t.colorSpace = THREE.SRGBColorSpace;
  return t;
}

function rand(seed) { let s = seed >>> 0; return () => { s = (s * 1664525 + 1013904223) >>> 0; return s / 4294967296; }; }

/** Stainless steel with the weld rings and seams of a rolled-and-welded tank: one ring per `period` metres along the body (the texture repeats in v), grain across it. As a bump and a roughness map. */
export function weldTextures(seed = 3) {
  const R = rand(seed);
  const bump = canvasTexture(256, 256, (g, w, h) => {
    g.fillStyle = '#808080'; g.fillRect(0, 0, w, h);
    const im = g.getImageData(0, 0, w, h);
    for (let i = 0; i < im.data.length; i += 4) { const n = (R() - .5) * 18; im.data[i] += n; im.data[i + 1] += n; im.data[i + 2] += n; }   // grain
    g.putImageData(im, 0, 0);
    // the horizontal weld: a ridge with a groove each side
    const y = 0;
    for (const [dy, v, hgt] of [[0, 230, 5], [-7, 70, 3], [7, 70, 3]]) { g.fillStyle = `rgb(${v},${v},${v})`; g.fillRect(0, ((y + dy) % h + h) % h - hgt / 2, w, hgt); g.fillRect(0, h + dy - hgt / 2, w, hgt); }
    // the vertical seams, a few per ring, a plate-width apart
    for (const x of [30, 100, 170, 235]) { g.fillStyle = 'rgb(165,165,165)'; g.fillRect(x, 0, 2, h); g.fillStyle = 'rgb(95,95,95)'; g.fillRect(x + 3, 0, 1.5, h); }
  }, { repeat: [1, 1] });
  const rough = canvasTexture(256, 256, (g, w, h) => {
    g.fillStyle = '#9a9a9a'; g.fillRect(0, 0, w, h);
    const im = g.getImageData(0, 0, w, h);
    for (let i = 0; i < im.data.length; i += 4) { const n = (R() - .5) * 40; im.data[i] += n; im.data[i + 1] += n; im.data[i + 2] += n; }
    g.putImageData(im, 0, 0);
    g.fillStyle = 'rgb(210,210,210)'; g.fillRect(0, -3, w, 6); g.fillRect(0, h - 3, w, 6);                 // a weld is rougher than the sheet
    for (let k = 0; k < 40; k++) { g.fillStyle = `rgba(${R() < .5 ? 60 : 200},${R() < .5 ? 60 : 200},${R() < .5 ? 60 : 200},.07)`; g.fillRect(R() * w, 0, 1 + R() * 6, h); }   // long streaks of the sheet's rolling
  });
  return { bump, rough };
}

/** The hexagonal ceramic tiles of a heat shield: black, a little glossy, a gap between them. As colour and bump. */
export function tileTextures() {
  const draw = (colour) => (g, w, h) => {
    g.fillStyle = colour ? '#2a2c2f' : '#303030'; g.fillRect(0, 0, w, h);
    const s = 32, hh = s * Math.sqrt(3) / 2;
    for (let row = -1; row < h / hh + 1; row++) for (let col = -1; col < w / (s * 1.5) + 1; col++) {
      const x = col * s * 1.5, y = row * hh * 2 + (col & 1 ? hh : 0);
      g.beginPath();
      for (let k = 0; k < 6; k++) { const a = Math.PI / 3 * k; g.lineTo(x + (s - 2.2) * Math.cos(a), y + (s - 2.2) * Math.sin(a)); }
      g.closePath();
      g.fillStyle = colour ? `rgb(${12 + ((row * 7 + col * 13) % 5) * 2},${13 + ((row * 5 + col * 11) % 5) * 2},${15 + ((row * 3 + col * 7) % 5) * 2})` : '#d8d8d8';
      g.fill();
    }
  };
  return { map: canvasTexture(512, 512, draw(true), { color: true }), bump: canvasTexture(512, 512, draw(false)) };
}

/** A soft round glow, for the flare at a nozzle and the puffs of smoke. */
export function glowTexture(size = 128, inner = 0.0, falloff = 2.2) {
  return canvasTexture(size, size, (g, w, h) => {
    const gr = g.createRadialGradient(w / 2, h / 2, w * inner, w / 2, h / 2, w / 2);
    gr.addColorStop(0, 'rgba(255,255,255,1)'); gr.addColorStop(.25, 'rgba(255,255,255,.55)'); gr.addColorStop(1, 'rgba(255,255,255,0)');
    g.fillStyle = gr; g.fillRect(0, 0, w, h);
  }, { aniso: 1 });
}

/** A cloud puff: noisy, round, soft. */
export function puffTexture(size = 128, seed = 11) {
  const R = rand(seed);
  return canvasTexture(size, size, (g, w, h) => {
    g.clearRect(0, 0, w, h);
    for (let k = 0; k < 60; k++) {
      const a = R() * Math.PI * 2, d = R() * w * .26, r = w * (.08 + R() * .16);
      const x = w / 2 + Math.cos(a) * d, y = h / 2 + Math.sin(a) * d;
      const gr = g.createRadialGradient(x, y, 0, x, y, r);
      gr.addColorStop(0, 'rgba(255,255,255,.35)'); gr.addColorStop(1, 'rgba(255,255,255,0)');
      g.fillStyle = gr; g.beginPath(); g.arc(x, y, r, 0, 7); g.fill();
    }
    const edge = g.createRadialGradient(w / 2, h / 2, w * .2, w / 2, h / 2, w / 2);
    edge.addColorStop(0, 'rgba(0,0,0,0)'); edge.addColorStop(1, 'rgba(0,0,0,1)');
    g.globalCompositeOperation = 'destination-out'; g.fillStyle = edge; g.fillRect(0, 0, w, h); g.globalCompositeOperation = 'source-over';
  }, { aniso: 1 });
}

/* ---------------------------------------------------------------------------------------------------------------------------------- a body of revolution */
/** The radius of a nose at a distance `t` from its tip (0 at the tip, 1 at the base), as a fraction of the base radius. */
export function noseRadius(shape, t, len, base) {
  t = Math.min(1, Math.max(0, t));
  switch (shape) {
    case 'ogive': { const rho = (base * base + len * len) / (2 * base); const x = t * len; return (Math.sqrt(Math.max(0, rho * rho - (len - x) * (len - x))) + base - rho) / base; }
    case 'parabola': return (2 * t - 0.5 * t * t) / 1.5;
    case 'ellipse': return Math.sqrt(Math.max(0, 1 - (1 - t) * (1 - t)));
    default: return t;
  }
}

/** The outline of a stage's outer body from its sections: [{x, r}], aft to forward, with the section kinds of the vehicle file (nose, tube, transition). */
export function outline(sections, n = 24) {
  const pts = [];
  for (const s of sections) {
    const x0 = s.x_start_m, L = s.length_m, ra = (s.d_aft_m || 0) / 2, rf = (s.d_fore_m || 0) / 2;
    if (s.kind === 'nose') {
      for (let i = 0; i <= n; i++) { const f = i / n; const t = 1 - f; const r = rf + (ra - rf) * noseRadius(s.shape, t, L, ra - rf || ra); pts.push({ x: x0 + f * L, r: i === n ? rf : r }); }
    } else if (s.kind === 'transition') {
      for (let i = 0; i <= 8; i++) pts.push({ x: x0 + (i / 8) * L, r: ra + (rf - ra) * (i / 8) });
    } else { pts.push({ x: x0, r: ra }, { x: x0 + L, r: ra }); }
  }
  return pts;
}

/** A lathe made by hand, so that the texture coordinate along the body is in metres (a weld every `ring` metres) and the normals follow the outline.
 *  profile: [{x, r}] (x is the axial distance: it becomes the model's Y), phiStart/phiLength limit it to part of the way round, colour(x) gives a vertex colour. */
export function lathe(profile, { segments = 64, phiStart = 0, phiLength = Math.PI * 2, ring = 1.8, colour = null, flip = false, uRepeat = 0 } = {}) {
  const n = profile.length, pos = [], nor = [], uv = [], col = [], idx = [];
  for (let i = 0; i < n; i++) {
    const a = profile[Math.max(0, i - 1)], b = profile[Math.min(n - 1, i + 1)];
    const dx = b.x - a.x, dr = b.r - a.r, len = Math.hypot(dx, dr) || 1;
    const ny = -dr / len, nr = dx / len;                               // the outward normal in the (r, x) plane
    for (let j = 0; j <= segments; j++) {
      const phi = phiStart + (j / segments) * phiLength, c = Math.cos(phi), s = Math.sin(phi);
      pos.push(profile[i].r * s, profile[i].x, profile[i].r * c);
      nor.push(nr * s * (flip ? -1 : 1), ny * (flip ? -1 : 1), nr * c * (flip ? -1 : 1));
      uv.push(uRepeat ? (j / segments) * phiLength * profile[0].r * uRepeat : j / segments * Math.max(1, Math.round(profile[0].r * Math.PI * 2 / 3.5)), profile[i].x / ring);
      if (colour) { const k = colour(profile[i].x); col.push(k[0], k[1], k[2]); }
    }
  }
  for (let i = 0; i < n - 1; i++) for (let j = 0; j < segments; j++) {
    const a = i * (segments + 1) + j, b = a + 1, c = a + segments + 1, d = c + 1;
    if (flip) idx.push(a, c, b, b, c, d); else idx.push(a, b, c, b, d, c);
  }
  const g = new THREE.BufferGeometry();
  g.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3));
  g.setAttribute('normal', new THREE.Float32BufferAttribute(nor, 3));
  g.setAttribute('uv', new THREE.Float32BufferAttribute(uv, 2));
  if (colour) g.setAttribute('color', new THREE.Float32BufferAttribute(col, 3));
  g.setIndex(idx);
  return g;
}

/* ---------------------------------------------------------------------------------------------------------------------------------- the engine */
const darkMetal = new THREE.MeshStandardMaterial({ color: 0x2a2623, metalness: 0.9, roughness: 0.42, side: THREE.DoubleSide });
const bellMat = new THREE.MeshStandardMaterial({ color: 0xffffff, vertexColors: true, metalness: 0.95, roughness: 0.34, side: THREE.DoubleSide });
const pumpMat = new THREE.MeshStandardMaterial({ color: 0x7c7f84, metalness: 0.9, roughness: 0.4 });

/** A rocket engine as seen from outside: a bell of exit radius from the nozzle's exit area, its chamber and turbopumps above it. The group's origin is the centre of the exit plane; the bell reaches up the +Y axis (the vehicle's forward). */
export function engineMesh(exitArea, vacuum = false) {
  const re = Math.sqrt(exitArea / Math.PI), eps = vacuum ? 80 : 34, rt = re / Math.sqrt(eps), Ln = re * (vacuum ? 3.1 : 2.3);
  const prof = [];
  const N = 22;
  for (let i = 0; i <= N; i++) { const s = i / N; prof.push({ x: s * Ln, r: rt + (re - rt) * Math.pow(1 - s, vacuum ? 0.62 : 0.52) }); }
  const g = lathe(prof, { segments: 28, colour: (x) => { const s = x / Ln; const k = 0.28 + 0.62 * s; return [k * 1.0, k * 0.86, k * 0.76]; }, ring: 100 });
  const grp = new THREE.Group();
  grp.add(new THREE.Mesh(g, bellMat));
  const cham = new THREE.Mesh(new THREE.CylinderGeometry(rt * 1.9, rt * 2.4, re * 0.9, 16), darkMetal); cham.position.y = Ln + re * 0.35; grp.add(cham);
  const pump = new THREE.Mesh(new THREE.CylinderGeometry(re * 0.38, re * 0.38, re * 0.9, 12), pumpMat); pump.position.set(re * 0.55, Ln + re * 0.2, 0); grp.add(pump);
  const pump2 = pump.clone(); pump2.position.set(-re * 0.5, Ln + re * 0.25, re * 0.3); pump2.scale.set(.8, 1, .8); grp.add(pump2);
  grp.userData = { exitRadius: re, length: Ln + re * 1.3 };
  return grp;
}

/** A box between two points, for lattice work (the tower, the fins, the mount). */
export function bar(a, b, w, mat) {
  const d = new THREE.Vector3().subVectors(b, a), L = d.length();
  const m = new THREE.Mesh(new THREE.BoxGeometry(w, L, w), mat);
  m.position.copy(a).addScaledVector(d, 0.5);
  m.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), d.normalize());
  return m;
}
