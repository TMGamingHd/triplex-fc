// Overview: the whole flight at a glance. The vehicle, the sky it is in, the numbers that matter (altitude, speed, Mach, dynamic pressure, mass, thrust, acceleration), the stages and what is left in them,
// and the events as they happen. The camera, the light and the look of the scene are chosen here.
import { Chart, legend } from '../../../js/charts.js';
import { h, setText, stat, statGrid, card, seg, toggle, slider, select, fmt, store, provenance } from '../ui.js';
import { CAMERAS } from '../cam.js';
import { STYLES } from '../models/vehicle.js';
import { DRESS } from '../models/starship.js';
import { MODES } from '../imagery.js';
import { AXES } from '../models/gltf.js';

export default {
  id: 'overview', label: 'Overview', key: '1',
  mount(app) {
    const { world, spec } = app, S = (this.s = {});
    S.stats = {
      alt: stat('Altitude', 'km'), speed: stat('Speed', 'm/s', 'The inertial speed'), mach: stat('Mach', ''), q: stat('Dynamic pressure', 'kPa'),
      mass: stat('Mass', 't'), thrust: stat('Thrust', 'MN'), gx: stat('Axial accel.', 'g', '(thrust + aerodynamic axial force) / (mass g₀)'), twr: stat('Thrust / weight', ''),
      rng: stat('Downrange', 'km'), vert: stat('Vertical speed', 'm/s'), tilt: stat('Tilt from vertical', '°'), alpha: stat('Angle of attack', '°'),
    };
    const s = S.stats;
    S.stages = h('div', { class: 'v-stages' });
    S.events = h('ol', { class: 'v-events' });
    app.left.replaceChildren(
      card('Flight', statGrid(3, s.alt, s.speed, s.mach, s.q, s.mass, s.thrust, s.gx, s.twr, s.rng)),
      card('Attitude and path', statGrid(3, s.vert, s.tilt, s.alpha)),
      card('Stages', S.stages),
      card('Events', S.events),
    );
    S.charts = {
      alt: new Chart(h('canvas'), { height: 110, window: 120, series: [{ key: 'alt', label: 'altitude m', color: 'var(--accent)' }] }),
      speed: new Chart(h('canvas'), { height: 110, window: 120, series: [{ key: 'speed', label: 'speed m/s', color: 'var(--ok)' }] }),
      q: new Chart(h('canvas'), { height: 110, window: 120, series: [{ key: 'q', label: 'q Pa', color: 'var(--warn)' }] }),
      thrust: new Chart(h('canvas'), { height: 110, window: 120, series: [{ key: 'thrust', label: 'thrust N', color: 'var(--crit)' }] }),
    };
    const ch = (t, c) => card(t, [c.c, legend(c.o.series)]);
    const st = world.settings;
    const camSeg = seg(CAMERAS.map((c) => [c.id, c.label, c.hint]), world.rig.mode, (v) => { world.rig.set(v); store.set('v.cam', v); }, 'camera');
    S.camSeg = camSeg;
    const styles = select([...STYLES.map((x) => [x.id, x.label]), ...app.models.map((m) => ['gltf:' + m.name, 'Imported: ' + m.name])], app.styleName, (v) => app.setStyle(v), 'model');
    const gl = app.gltf, again = () => { app.saveGltf(); app.rebuild(); };
    S.import = h('div', { class: app.styleName.startsWith('gltf:') ? '' : 'hide', style: { display: app.styleName.startsWith('gltf:') ? '' : 'none' } },
      h('div', { class: 'v-row' }, h('span', { class: 'v-lbl' }, 'Nose along'), select(AXES, gl.axis, (v) => { gl.axis = v; again(); }, 'the vehicle\'s axis in the file')),
      toggle('Scale to the vehicle file\'s length', gl.fitLength, (v) => { gl.fitLength = v; again(); }), slider('Roll', -180, 180, 5, gl.roll, (v) => { gl.roll = v; app.saveGltf(); }, (v) => v + '°'), slider('Move along', -20, 20, 0.5, gl.offset, (v) => { gl.offset = v; app.saveGltf(); }, (v) => v + ' m'),
      h('button', { class: 'btn', type: 'button', onclick: again }, 'Apply'), h('p', { class: 'note' }, 'Nodes named stage1, stage2… go with that stage and leave when it separates; the engines, plumes and every lens overlay come from the vehicle file, not from the picture.'));
    // the Starship's details: a choice of the viewer (the vehicle file has no grid fins, the simulator's aerodynamics do not use them): Block 3 as reported by default, Block 2 one click away
    const dr = app.dress, redo = () => { app.saveDress(); app.rebuild(); };
    S.dress = h('div', { id: 'v-dress' },
      h('div', { class: 'v-row' }, h('span', { class: 'v-lbl' }, 'Version'), seg(Object.entries(DRESS).map(([k, v]) => [k, v.name]), Object.entries(DRESS).find(([, v]) => v.fins === dr.fins && v.finScale === dr.finScale && v.finDrop === dr.finDrop)?.[0] ?? '', (k) => { Object.assign(dr, DRESS[k]); redo(); }, 'Starship version')),
      slider('Grid fins', 0, 6, 1, dr.fins, (v) => { dr.fins = v; }, (v) => v + ''), slider('Fin size', 0.6, 2, 0.02, dr.finScale, (v) => { dr.finScale = v; }, (v) => '×' + v.toFixed(2)),
      slider('Fins below the ring', 0, 14, 0.5, dr.finDrop, (v) => { dr.finDrop = v; }, (v) => v + ' m'), slider('Flap size', 0.6, 1.6, 0.05, dr.flapScale, (v) => { dr.flapScale = v; }, (v) => '×' + v.toFixed(2)),
      slider('Hot-stage ring', 1, 6, 0.1, dr.ringHeight, (v) => { dr.ringHeight = v; }, (v) => v.toFixed(1) + ' m'), toggle('Catch pins', dr.pins, (v) => { dr.pins = v; }),
      h('button', { class: 'btn', type: 'button', id: 'v-dress-apply', onclick: redo }, 'Apply'),
      h('p', { class: 'note' }, 'Block 3 as reported: three grid fins, each 50 % larger in area than Block 2\'s (×1.22 in each direction here), set lower on the booster. These are a picture of public descriptions: the vehicle file, and so the physics, does not have them.'));
    // the Earth: pictures of the real planet from NASA (a pack the user downloads), or the procedural planet
    const im = world.imagery;
    S.imgNote = h('p', { class: 'note', id: 'v-img-note' }, im.describe());
    S.earth = h('div', { id: 'v-earth' },
      im.present ? [h('div', { class: 'v-row' }, h('span', { class: 'v-lbl' }, 'Imagery'), select([['auto', 'Best there is'], ...MODES.map(([k, t]) => [k, t])], im.mode, (v) => { im.setMode(v); setTimeout(() => setText(S.imgNote, im.describe()), 400); }, 'Earth imagery')),
        toggle('Lights of the night side', im.night, (v) => im.setNight(v)), S.imgNote,
        h('p', { class: 'note' }, 'A picture of the ground, not terrain: it is of the year 2000 (Landsat) or a monthly composite (Blue Marble), and the sea in it is lit by the viewer\'s own sun. The sky, the air and the clouds are drawn over it as before.')]
        : [h('p', { class: 'note' }, 'The planet is procedural: a coast at the pad and made-up continents. For pictures of the real Earth (NASA, public domain, about 13 MB) run ', h('code', {}, 'console/tfc-imagery'), ' once, then reload this page.')]);
    const look = [
      h('div', { class: 'v-row' }, camSeg),
      h('p', { class: 'note v-camhint' }, 'Drag to turn, wheel to zoom, right-drag to slide along the vehicle. Keys: 1–6 lenses, C next camera, Space pause, H hide the panels.'),
      h('div', { class: 'v-row' }, h('span', { class: 'v-lbl' }, 'Earth')), S.earth,
      h('div', { class: 'v-row' }, h('span', { class: 'v-lbl' }, 'Model'), styles), S.import, (app.styleName === 'starship' || (app.styleName === 'auto' && /starship/i.test(app.spec.name || ''))) ? S.dress : null,
      slider('Sun elevation', -10, 90, 1, st.sunElev, (v) => { st.sunElev = v; store.set('v.sunEl', v); }, (v) => v + '°'),
      slider('Sun bearing', 0, 359, 1, st.sunBear, (v) => { st.sunBear = v; store.set('v.sunBear', v); }, (v) => v + '°'),
      slider('Cloud cover', 0, 1, 0.05, st.cloud, (v) => { st.cloud = v; store.set('v.cloud', v); }, (v) => Math.round(v * 100) + ' %'),
      toggle('Exhaust plumes', st.plumes, (v) => { st.plumes = v; world.plumes && (world.plumes.items.forEach((i) => (i.mesh.parent.visible = true))); store.set('v.plumes', v); }),
      toggle('Exhaust trail', st.trail, (v) => { st.trail = v; store.set('v.trail', v); }), toggle('Launch site', st.pad, (v) => { st.pad = v; store.set('v.pad', v); }),
      h('div', { class: 'v-row' }, h('button', { class: 'btn', type: 'button', onclick: () => app.screenshot() }, 'Save a picture'), h('button', { class: 'btn', type: 'button', onclick: () => app.fullscreen() }, 'Full screen')),
    ];
    app.right.replaceChildren(card('Camera and light', look), ch('Altitude', S.charts.alt), ch('Speed', S.charts.speed), ch('Dynamic pressure', S.charts.q), ch('Thrust', S.charts.thrust),
      card('Where the numbers come from', [provenance('measured', 'every number here is the simulator\'s own state at that instant.'), provenance('illustrative', 'the sky, the ground, the pad, the exhaust and the trail are pictures drawn round it; a stage that has left is carried on by the viewer, which the simulator does not track.')]));
  },
  update(app, pose, d) {
    const S = this.s, s = S.stats, spec = app.spec;
    const vert = (pose.v[0] * pose.r[0] + pose.v[1] * pose.r[1] + pose.v[2] * pose.r[2]) / Math.hypot(...pose.r);
    const wt = pose.m * 9.80665;
    s.alt.set((pose.alt / 1000).toFixed(pose.alt < 100000 ? 2 : 1)); s.speed.set(pose.spd.toFixed(0)); s.mach.set(pose.mach.toFixed(2)); s.q.set((pose.qd / 1000).toFixed(1));
    s.mass.set((pose.m / 1000).toFixed(0)); s.thrust.set((pose.thr / 1e6).toFixed(1)); s.gx.set(d.gx.toFixed(2)); s.twr.set((pose.thr / wt).toFixed(2));
    s.rng.set((pose.rng / 1000).toFixed(1)); s.vert.set(vert.toFixed(0)); s.tilt.set(Math.hypot(pose.tilt[0], pose.tilt[1]).toFixed(1)); s.alpha.set(d.alpha.toFixed(1));
    // the stages: propellant left, engines running
    const el = S.stages;
    if (el.childElementCount !== spec.stages.length) {
      el.replaceChildren(...spec.stages.map((st, i) => h('div', { class: 'v-stage', dataset: { i } },
        h('div', { class: 'v-stage-h' }, h('b', {}, st.name || `stage ${i + 1}`), h('span', { class: 'v-stage-state' })), h('div', { class: 'v-bar' }, h('i')), h('div', { class: 'v-stage-n' }))));
    }
    spec.stages.forEach((st, i) => {
      const row = el.children[i], cap = (st.tanks || []).reduce((a, t) => a + t.propellant_kg, 0), left = pose.prop ? pose.prop[i] : 0, on = !!((pose.stg ?? 255) & (1 << i)), ign = !!((pose.ign ?? 0) & (1 << i));
      const engines = spec.engines.map((e, k) => [e, k]).filter(([e]) => e.stage === i), running = engines.filter(([, k]) => (pose.eng || [])[k] > 0.02).length, failed = engines.filter(([, k]) => (pose.eng || [])[k] < 0).length;
      row.classList.toggle('gone', !on);
      row.querySelector('.v-stage-state').textContent = !on ? 'separated' : running ? 'burning' : ign ? 'burnt out' : 'waiting';
      row.querySelector('.v-bar i').style.width = (on && cap ? Math.max(0, Math.min(100, 100 * left / cap)) : 0).toFixed(1) + '%';
      row.querySelector('.v-stage-n').textContent = on ? `${fmt.eng(left)} kg of ${fmt.eng(cap)} kg; ${running} of ${engines.length} engines${failed ? `, ${failed} failed` : ''}` : '';
    });
    // the events
    if (S.evCount !== app.events.length) {
      S.evCount = app.events.length;
      S.events.replaceChildren(...app.events.slice(-12).reverse().map((e) => h('li', { class: e.level }, h('span', { class: 'mono' }, fmt.t(e.t)), ' ', e.text)));
    }
    for (const c of Object.values(S.charts)) c.draw(app.data.hist);
  },
  unmount() { this.s = null; },
};
