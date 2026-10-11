// Drawing aids shared by the lenses: text labels that stay the same size on the screen, arrows, the centre-of-gravity and centre-of-pressure marks, dimension lines and arcs.
import * as THREE from '../vendor/three.module.js';

/** A text label that is always the same size on the screen and always drawn on top. `setText` redraws it only when the text changes. */
export class Label {
  constructor({ text = '', color = '#ffffff', bg = 'rgba(8,12,20,.72)', size = 0.032, font = '600 22px system-ui, sans-serif', border = null } = {}) {
    this.canvas = document.createElement('canvas'); this.ctx = this.canvas.getContext('2d');
    this.tex = new THREE.CanvasTexture(this.canvas); this.tex.colorSpace = THREE.SRGBColorSpace; this.tex.anisotropy = 4;
    this.mat = new THREE.SpriteMaterial({ map: this.tex, depthTest: false, depthWrite: false, transparent: true, sizeAttenuation: false, toneMapped: false });
    this.sprite = new THREE.Sprite(this.mat); this.sprite.renderOrder = 100; this.sprite.frustumCulled = false;
    this.opt = { color, bg, size, font, border }; this.text = null; this.drawnAt = 0; this.setText(text);
  }
  setText(text, color) {
    if (text === this.text && (!color || color === this.opt.color)) return;
    const now = performance.now();
    if (this.nDraw >= 2 && now - this.drawnAt < 150) return;                  // a number that changes every frame is redrawn a few times a second, not sixty (a canvas upload each)
    this.drawnAt = now; this.nDraw = (this.nDraw || 0) + 1;
    if (color) this.opt.color = color;
    this.text = text;
    const g = this.ctx, o = this.opt, lines = String(text).split('\n');
    g.font = o.font;
    const w = Math.ceil(Math.max(...lines.map((l) => g.measureText(l).width))) + 20, lh = 28, h = lh * lines.length + 12;
    this.canvas.width = w; this.canvas.height = h;
    g.font = o.font; g.textBaseline = 'middle';
    g.fillStyle = o.bg; g.beginPath(); g.roundRect ? g.roundRect(0, 0, w, h, 8) : g.rect(0, 0, w, h); g.fill();
    if (o.border) { g.strokeStyle = o.border; g.lineWidth = 2; g.stroke(); }
    g.fillStyle = o.color;
    lines.forEach((l, i) => g.fillText(l, 10, 6 + lh * (i + 0.5)));
    this.tex.needsUpdate = true;
    this.sprite.scale.set(o.size * w / 40, o.size * h / 40, 1);                  // the height of one line is `size` of the screen's height
  }
  setAnchor(ax, ay) { this.sprite.center.set(ax, ay); this.base = [ax, ay]; return this; }
  get object() { return this.sprite; }
}

/** Pushes labels that would sit on top of one another apart, on the screen: each is first put back at the anchor its lens gave it, then, in the order given (the first keeps its place), moved up or down by whole label heights
 *  until it touches nothing already placed (at most three steps each way). A label that is hidden, or behind the camera, is left out. `W`, `H`: the size of the canvas in pixels. */
export function declutter(labels, camera, W, H, pad = 4) {
  const placed = [], v = new THREE.Vector3();
  const shown = (o) => { for (let n = o; n; n = n.parent) if (!n.visible) return false; return true; };
  for (const L of labels) {
    const sp = L.sprite;
    if (!sp || !shown(sp)) continue;
    const base = L.base || [sp.center.x, sp.center.y];
    sp.center.set(base[0], base[1]);
    sp.getWorldPosition(v).project(camera);
    if (v.z > 1 || v.z < -1) continue;
    const px = (v.x * 0.5 + 0.5) * W, py = (1 - (v.y * 0.5 + 0.5)) * H, w = sp.scale.x * H, h = sp.scale.y * H;
    const left = px - base[0] * w, top0 = py - (1 - base[1]) * h;
    const free = (t) => !placed.some((r) => left < r.x + r.w + pad && left + w + pad > r.x && t < r.y + r.h + pad && t + h + pad > r.y);
    let dy = 0;
    for (let k = 1; k <= 6 && !free(top0 + dy); k++) dy = (k % 2 ? 1 : -1) * Math.ceil(k / 2) * (h + pad);
    sp.center.y = base[1] + dy / h;                                         // the sprite's anchor moves down by dy pixels when its y grows by dy / h
    placed.push({ x: left, y: top0 + dy, w, h });
  }
}

/** An arrow from `origin` along `dir`, `length` long, with a head in proportion. */
export class Arrow {
  constructor(color = 0xffffff, { opacity = 1, depthTest = false, radius = 0.5 } = {}) {
    this.group = new THREE.Group();
    this.mat = new THREE.MeshBasicMaterial({ color, transparent: opacity < 1, opacity, depthTest, depthWrite: false, toneMapped: false });
    this.shaft = new THREE.Mesh(new THREE.CylinderGeometry(1, 1, 1, 12, 1), this.mat);
    this.head = new THREE.Mesh(new THREE.ConeGeometry(1, 1, 16, 1), this.mat);
    this.group.add(this.shaft, this.head);
    this.shaft.renderOrder = this.head.renderOrder = 90; this.radius = radius;
  }
  /** origin and dir as THREE.Vector3 (dir need not be unit); length in the group's units. */
  set(origin, dir, length, width) {
    const d = dir.clone().normalize();
    const w = width ?? Math.max(0.12, length * 0.025), headL = Math.min(length * 0.35, w * 6), shaftL = Math.max(0.01, length - headL);
    this.shaft.scale.set(w, shaftL, w); this.shaft.position.copy(origin).addScaledVector(d, shaftL / 2);
    this.head.scale.set(w * 2.4, headL, w * 2.4); this.head.position.copy(origin).addScaledVector(d, shaftL + headL / 2);
    const q = new THREE.Quaternion().setFromUnitVectors(new THREE.Vector3(0, 1, 0), d);
    this.shaft.quaternion.copy(q); this.head.quaternion.copy(q);
    this.group.visible = length > 1e-3;
  }
  get object() { return this.group; }
}

const _ringTex = {};
function markTexture(kind) {
  if (_ringTex[kind]) return _ringTex[kind];
  const c = document.createElement('canvas'); c.width = c.height = 128; const g = c.getContext('2d');
  g.translate(64, 64);
  if (kind === 'cg') {                                         // the quartered disc of a centre of mass: two black and two white quadrants in a circle
    g.fillStyle = '#ffffff'; g.beginPath(); g.arc(0, 0, 44, 0, Math.PI * 2); g.fill();
    g.fillStyle = '#10141c'; g.beginPath(); g.moveTo(0, 0); g.arc(0, 0, 44, 0, Math.PI / 2); g.fill(); g.beginPath(); g.moveTo(0, 0); g.arc(0, 0, 44, Math.PI, Math.PI * 1.5); g.fill();
    g.strokeStyle = '#10141c'; g.lineWidth = 6; g.beginPath(); g.arc(0, 0, 44, 0, Math.PI * 2); g.stroke();
    g.strokeStyle = '#3ddc97'; g.lineWidth = 5; g.beginPath(); g.arc(0, 0, 52, 0, Math.PI * 2); g.stroke();
  } else {                                                      // the centre of pressure: a ring with a cross
    g.strokeStyle = '#10141c'; g.lineWidth = 16; g.beginPath(); g.arc(0, 0, 40, 0, Math.PI * 2); g.stroke();
    g.strokeStyle = '#4cc9f0'; g.lineWidth = 9; g.beginPath(); g.arc(0, 0, 40, 0, Math.PI * 2); g.stroke();
    g.strokeStyle = '#4cc9f0'; g.lineWidth = 7; g.beginPath(); g.moveTo(-56, 0); g.lineTo(56, 0); g.moveTo(0, -56); g.lineTo(0, 56); g.stroke();
  }
  const t = new THREE.CanvasTexture(c); t.colorSpace = THREE.SRGBColorSpace;
  return (_ringTex[kind] = t);
}

/** A mark that stays the same size on the screen: 'cg' or 'cp'. */
export function mark(kind, size = 0.045) {
  const m = new THREE.Sprite(new THREE.SpriteMaterial({ map: markTexture(kind), depthTest: false, depthWrite: false, transparent: true, sizeAttenuation: false, toneMapped: false }));
  m.scale.set(size, size, 1); m.renderOrder = 110; m.frustumCulled = false;
  return m;
}

/** A line in 3D that is drawn on top. */
export function line(points, color = 0xffffff, { dashed = false, opacity = 1, depthTest = false } = {}) {
  const g = new THREE.BufferGeometry().setFromPoints(points);
  const m = dashed ? new THREE.LineDashedMaterial({ color, dashSize: 1, gapSize: 0.6, transparent: opacity < 1, opacity, depthTest, depthWrite: false, toneMapped: false })
    : new THREE.LineBasicMaterial({ color, transparent: opacity < 1, opacity, depthTest, depthWrite: false, toneMapped: false });
  const l = new THREE.Line(g, m); l.renderOrder = 95; l.frustumCulled = false;
  if (dashed) l.computeLineDistances();
  return l;
}

/** Update a line's points in place. */
export function setLine(l, points) {
  const a = l.geometry.attributes.position;
  if (!a || a.count !== points.length) { l.geometry.dispose(); l.geometry = new THREE.BufferGeometry().setFromPoints(points); } else { points.forEach((p, i) => a.setXYZ(i, p.x, p.y, p.z)); a.needsUpdate = true; }
  l.geometry.computeBoundingSphere();
  if (l.material.isLineDashedMaterial) l.computeLineDistances();
}

/** A colour map for scalar fields: the full spectrum of "turbo" (Mikhailov's polynomial fit), dark blue for the lowest values through cyan, green and yellow to red for the highest: every value has its own colour, unlike a
 *  blue-white-red ramp, whose middle is white. Used for the pressure on the skin, the streamlines' speed and the legends. Returns [r, g, b] 0..1 for t in 0..1. */
export function colormap(t) {
  const x = Math.min(1, Math.max(0, t)), x2 = x * x, x3 = x2 * x, x4 = x2 * x2, x5 = x4 * x;
  const c = (v) => Math.min(1, Math.max(0, v));
  return [
    c(0.13572138 + 4.61539260 * x - 42.66032258 * x2 + 132.13108234 * x3 - 152.94239396 * x4 + 59.28637943 * x5),
    c(0.09140261 + 2.19418839 * x + 4.84296658 * x2 - 14.18503333 * x3 + 4.27729857 * x4 + 2.82956604 * x5),
    c(0.10667330 + 12.64194608 * x - 60.58204836 * x2 + 110.36276771 * x3 - 89.90310912 * x4 + 27.34824973 * x5),
  ];
}
export const COLORMAP_GLSL = /* glsl */`
vec3 cmap(float x) {
  x = clamp(x, 0.0, 1.0);
  const vec4 kR = vec4(0.13572138, 4.61539260, -42.66032258, 132.13108234), kG = vec4(0.09140261, 2.19418839, 4.84296658, -14.18503333), kB = vec4(0.10667330, 12.64194608, -60.58204836, 110.36276771);
  const vec2 kR2 = vec2(-152.94239396, 59.28637943), kG2 = vec2(4.27729857, 2.82956604), kB2 = vec2(-89.90310912, 27.34824973);
  vec4 v4 = vec4(1.0, x, x * x, x * x * x); vec2 v2 = v4.zw * v4.z;
  return clamp(vec3(dot(v4, kR) + dot(v2, kR2), dot(v4, kG) + dot(v2, kG2), dot(v4, kB) + dot(v2, kB2)), 0.0, 1.0);
}
`;

/** The legend bar for the colour map as a canvas the page can place. */
export function legendCanvas(w = 220, h = 12) {
  const c = document.createElement('canvas'); c.width = w; c.height = h; const g = c.getContext('2d');
  for (let x = 0; x < w; x++) { const [r, gg, b] = colormap(x / (w - 1)); g.fillStyle = `rgb(${r * 255 | 0},${gg * 255 | 0},${b * 255 | 0})`; g.fillRect(x, 0, 1, h); }
  c.style.width = '100%'; c.style.height = h + 'px'; c.style.borderRadius = '3px';
  return c;
}
