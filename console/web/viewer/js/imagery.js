// The Earth imagery of the sky shader: which pictures there are (the manifest of console/imagery/, written by `console/tfc-imagery`), which of them are on, and the numbers the shader needs to put them on the
// planet (the longitude and latitude of the pad). With no pack, or with the choice "Procedural", the planet is the one env.js draws by itself.
import * as THREE from '../vendor/three.module.js';
import { store } from './ui.js';

export const MODES = [
  ['procedural', 'Procedural', 'no pictures: the coast at the pad and a made-up planet, the cheapest'],
  ['earth-4k', 'Earth 4K', 'Blue Marble, 9.8 km a pixel'],
  ['earth-8k', 'Earth 8K', 'Blue Marble, 4.9 km a pixel'],
  ['site-4k', 'Earth 4K + the launch site', 'with Landsat patches of 108 m and 27 m a pixel round the pad'],
  ['site-8k', 'Earth 8K + the launch site', 'the most detailed'],
];

const blank = () => { const t = new THREE.DataTexture(new Uint8Array([0, 0, 0, 255]), 1, 1); t.needsUpdate = true; return t; };

export class Imagery {
  constructor(sky, renderer) {
    this.sky = sky; this.renderer = renderer;
    this.manifest = { layers: [], sites: {}, present: false };
    this.mode = store.get('v.img', 'auto');          // 'auto': the best the pack has, when there is a pack
    this.night = store.get('v.imgNight', true);
    this.site = null;                                 // chosen by the latitude of the pad
    this.tex = new Map();                             // layer id -> texture
    this.loading = new Map();
    this.applied = '';
    this.blank = blank();
    const u = sky.u;
    u.uTexDay.value = u.uTexNight.value = u.uTexReg.value = u.uTexLoc.value = this.blank;
  }

  async refresh() {
    const tok = sessionStorage.getItem('tfc.token') || '';
    try {
      const r = await fetch('/api/viewer/imagery', { headers: { 'X-TFC-Token': tok } });
      if (r.ok) this.manifest = await r.json();
    } catch { /* no pack: the procedural planet */ }
    this.applied = '';
  }

  layer(id) { return this.manifest.layers.find((l) => l.id === id); }
  get present() { return !!this.manifest.present; }
  /** What "auto" means: the most detailed picture set there is for the site. */
  resolve() {
    if (!this.present) return 'procedural';
    if (this.mode !== 'auto') return MODES.some((m) => m[0] === this.mode) ? this.mode : 'procedural';
    const hasSite = this.site && this.layer(`site-${this.site}-local`);
    return hasSite ? 'site-8k' : (this.layer('earth-8k') ? 'earth-8k' : 'earth-4k');
  }
  setMode(m) { this.mode = m; store.set('v.imgNight', this.night); store.set('v.img', m); this.applied = ''; }
  setNight(on) { this.night = on; store.set('v.imgNight', on); this.applied = ''; }

  async texture(id) {
    if (this.tex.has(id)) return this.tex.get(id);
    if (this.loading.has(id)) return this.loading.get(id);
    const l = this.layer(id);
    if (!l) return null;
    const max = this.renderer.capabilities.maxTextureSize;
    if (l.px[0] > max) return null;
    const tok = sessionStorage.getItem('tfc.token') || '';
    const p = (async () => {
      const r = await fetch('/api/viewer/imagery/file?name=' + encodeURIComponent(l.file), { headers: { 'X-TFC-Token': tok } });
      if (!r.ok) throw new Error(`${l.file}: ${r.status}`);
      const bmp = await createImageBitmap(await r.blob(), { imageOrientation: 'flipY', premultiplyAlpha: 'none', colorSpaceConversion: 'none' });
      const t = new THREE.Texture(bmp);
      t.flipY = false; t.colorSpace = THREE.SRGBColorSpace; t.generateMipmaps = true; t.minFilter = THREE.LinearMipmapLinearFilter; t.magFilter = THREE.LinearFilter;
      t.anisotropy = Math.min(8, this.renderer.capabilities.getMaxAnisotropy());
      t.wrapS = l.kind === 'patch' ? THREE.ClampToEdgeWrapping : THREE.RepeatWrapping; t.wrapT = THREE.ClampToEdgeWrapping;
      t.needsUpdate = true;
      this.tex.set(id, t);
      return t;
    })().catch((e) => { console.warn('imagery', e.message); return null; }).finally(() => this.loading.delete(id));
    this.loading.set(id, p);
    return p;
  }

  /** Called every frame with the planet's pole and the pad's direction (scene axes, planet frame): sets the shader's numbers, and starts the pictures loading when the choice has changed. */
  async sync(pole, up0) {
    const u = this.sky.u;
    const latSim = Math.asin(Math.max(-1, Math.min(1, pole.dot(up0))));
    const eqx = up0.clone().addScaledVector(pole, -pole.dot(up0)); if (eqx.lengthSq() < 1e-9) eqx.set(1, 0, 0); eqx.normalize();
    u.uEqX.value.copy(eqx); u.uEqY.value.copy(new THREE.Vector3().crossVectors(pole, eqx));
    const sites = this.manifest.sites || {};
    if (!this.site || this.siteFor !== latSim.toFixed(4)) {
      this.siteFor = latSim.toFixed(4);
      let best = null, bd = 1e9;
      for (const [k, s] of Object.entries(sites)) { const d = Math.abs(s.lat * Math.PI / 180 - latSim); if (d < bd) { bd = d; best = k; } }
      this.site = best; this.applied = '';
    }
    const s = sites[this.site];
    if (s) u.uLonLat0.value.set(s.lon * Math.PI / 180, (s.lat * Math.PI / 180) - latSim);
    const mode = this.resolve(), key = mode + this.night + this.site + this.manifest.layers.length;
    if (key === this.applied || this.busy) return;
    this.busy = true;
    try {
      const want = mode === 'procedural' ? null : mode.endsWith('8k') && this.layer('earth-8k') ? 'earth-8k' : 'earth-4k';
      const day = want ? await this.texture(want) : null;
      const night = want && this.night && this.layer('night-4k') ? await this.texture('night-4k') : null;
      let patches = 0, reg = null, loc = null;
      if (want && mode.startsWith('site')) {
        reg = await this.texture(`site-${this.site}-regional`); loc = await this.texture(`site-${this.site}-local`);
        if (reg) { patches = 1; const b = this.layer(`site-${this.site}-regional`).box; u.uBoxReg.value.set(...b.map((v) => v * Math.PI / 180)); u.uTexReg.value = reg; }
        if (reg && loc) { patches = 2; const b = this.layer(`site-${this.site}-local`).box; u.uBoxLoc.value.set(...b.map((v) => v * Math.PI / 180)); u.uTexLoc.value = loc; }
      }
      u.uTexDay.value = day || this.blank; u.uTexNight.value = night || this.blank;
      const finest = patches === 2 ? this.layer(`site-${this.site}-local`).texel_km : patches === 1 ? this.layer(`site-${this.site}-regional`).texel_km : want ? this.layer(want).texel_km : 10;
      u.uImg.value.set(day ? 1 : 0, night ? 1 : 0, patches, finest);
      this.applied = key; this.state = day ? mode : 'procedural';
    } finally { this.busy = false; }
  }

  /** A sentence for the page: what is drawn and where it comes from. */
  describe() {
    if (!this.present) return 'No imagery pack: the procedural planet. Run console/tfc-imagery to download one.';
    const m = this.state || this.resolve();
    if (m === 'procedural') return 'The procedural planet (a made-up coast and continents).';
    const l = this.layer(m.endsWith('8k') ? 'earth-8k' : 'earth-4k') || {};
    const site = this.site && this.manifest.sites[this.site];
    return `${l.what || 'Earth'}${m.startsWith('site') && site ? `; Landsat patches round ${site.name}` : ''}. NASA GIBS, public domain.`;
  }
}
