// A smoke test that runs in the page itself (`?selftest=1`, the way the console has one): it goes through every lens, every camera and every generated model style on whatever flight is being shown, and checks that
// nothing threw, that the lens drew its panels, and that a frame was rendered after each change. It prints the result on the page, because a headless browser has no console to read and its screenshot is the only
// way out; the result is also in the page's title (`selftest ok` or `selftest FAILED`).
import { CAMERAS } from './cam.js';
import { STYLES } from './models/vehicle.js';
import * as THREE from '../vendor/three.module.js';
import { sameVehicle } from './cfd.js';
import { gimbalAngles } from './gyro.js';
import { Director } from './director.js';
import { callName } from './lenses/broadcast.js';
import { sampleTable, tableFor } from './cfdui.js';

export async function run(app, lenses, setLens, world) {
  const out = []; let fails = 0;
  const errors = [];
  const keep = console.error; console.error = (...a) => { errors.push(a.map((x) => (x && x.stack) || String(x)).join(' ')); keep(...a); };
  window.addEventListener('error', (e) => errors.push(e.message));
  const frame = () => new Promise((r) => requestAnimationFrame(() => requestAnimationFrame(r)));
  const check = (ok, text) => { out.push(`${ok ? 'ok  ' : 'FAIL'}  ${text}`); if (!ok) fails++; };
  const settle = async (ms = 400) => { await new Promise((r) => setTimeout(r, ms)); await frame(); };
  await settle(1500);
  check(!!app.pose, 'a pose is being shown');
  check(world.vehicle && world.vehicle.engines.length === app.spec.engines.length, `the model has the vehicle file's ${app.spec.engines.length} engines`);
  for (const l of lenses) {
    const before = errors.length;
    setLens(l.id); await settle();
    check(errors.length === before, `lens ${l.id}: no error while it mounts and draws`);
    if (l.id === 'cast') {                                                                  // the broadcast has no docks: its readouts are the stage blocks and the clock
      const nums = [...document.querySelectorAll('#cast .cast-v b')].filter((e) => /[0-9]/.test(e.textContent)).length;
      check(!!document.getElementById('cast') && nums >= 2, `lens ${l.id}: the stage blocks show ${nums} numbers`);
      check(/^T[+−] \d\d:\d\d:\d\d$/.test(document.querySelector('#cast .cast-clock').textContent), `lens ${l.id}: the clock reads ${document.querySelector('#cast .cast-clock').textContent}`);
      check([...document.querySelectorAll('#cast')].every((e) => !/NaN|Infinity|undefined/.test(e.textContent)), `lens ${l.id}: no NaN, Infinity or undefined on the screen`);
      if (app.auto) { app.setAuto(false); const b = document.querySelector('#v-follow button[data-id="main"]'); if (b) b.click(); await settle(500); }      // (the first visit to the broadcast turns the director on: the checks that follow are about the main body)
      continue;
    }
    check(app.left.textContent.length > 60 && app.right.textContent.length > 60, `lens ${l.id}: both docks have text`);
    const nums = [...app.left.querySelectorAll('.stat .v')].filter((e) => /[0-9]/.test(e.textContent)).length;
    check(nums >= 4, `lens ${l.id}: ${nums} readouts have a number`);
    check([...app.left.querySelectorAll('.stat .v')].every((e) => !/NaN|Infinity|undefined/.test(e.textContent)), `lens ${l.id}: no NaN, Infinity or undefined in a readout`);
  }
  for (const c of CAMERAS) {
    const before = errors.length;
    world.rig.set(c.id); await settle(250);
    check(errors.length === before && Number.isFinite(world.camera.position.x) && Number.isFinite(world.camera.fov), `camera ${c.id}: a finite eye`);
  }
  // the cameras that are meant to show the vehicle do: it projects inside the picture, in front of the eye (the ground camera once looked at a fixed direction and the vehicle was nowhere in it)
  for (const id of ['orbit', 'chase', 'track', 'ground', 'wide']) {
    document.querySelector(`#v-cams button[data-id="${id}"]`).click(); await settle(450);                // through the bar, as a person does
    const v = new THREE.Vector3(0, 0, 0).project(world.camera);
    check(Math.abs(v.x) <= 1 && Math.abs(v.y) <= 1 && v.z < 1 && v.z > -1, `camera ${id}: the vehicle is inside the picture (${v.x.toFixed(2)}, ${v.y.toFixed(2)})`);
    const pressed = document.querySelector('#v-cams button[aria-pressed="true"]');
    check(pressed && pressed.dataset.id === id, `camera ${id}: the camera bar shows it as the current one`);
  }
  // a vehicle too small to see is circled, one that fills the picture is not
  world.rig.set('orbit'); await settle(300);
  check(document.getElementById('v-locator').hidden, 'the locator is hidden when the vehicle is big on the screen');
  // the cutaway: every skin material is clipped by the vehicle's plane, the inside appears and goes away again
  {
    const was = world.settings.inside;
    world.settings.inside = true; await settle(300);
    check(world.vehicle.inside && world.vehicle.hull.length > 0 && world.vehicle.hull.every((m) => m.clippingPlanes && m.clippingPlanes.length === 1), `the cutaway clips the skin (${world.vehicle.hull.length} materials)`);
    check(world.vehicle.interior.tanks.length === app.spec.tanks.length && world.vehicle.interior.parts.some((p) => p.visible), `and shows the inside: ${app.spec.tanks.length} tanks`);
    const n = world.vehicle.interior.tanks.filter((q) => q.liquid.visible).length;
    check(n > 0, `liquid in ${n} tanks, at the levels the pose gives`);
    world.settings.inside = false; await settle(200);
    check(!world.vehicle.inside && world.vehicle.hull.every((m) => !m.clippingPlanes), 'and the skin is whole again');
    world.settings.inside = was;
  }
  // the surfaces of the vehicle file are in the model (the flaps hinged on the axis, the grid fins), and they follow the deflection and the deployed bit in the pose
  const drawn = (app.spec.surfaces || []).filter((x) => x.kind === 'grid_fin' || x.axial_hinge);
  check(world.vehicle.flaps.length === drawn.length, `the model has the vehicle file's ${drawn.length} surfaces (flaps and grid fins)`);
  const flap = world.vehicle.flaps.find((f) => f.pivot);
  if (flap) {
    const all = (v) => (app.spec.surfaces || []).map(() => v);
    world.vehicle.apply({ ...app.pose, surf: all(40) }); const out = flap.pivot.rotation.y;
    world.vehicle.apply({ ...app.pose, surf: all(90) }); const stood = flap.pivot.rotation.y;
    check(Math.abs(Math.abs(out) - 40 * Math.PI / 180) < 1e-9 && Math.abs(Math.abs(stood) - Math.PI / 2) < 1e-9, `a flap turns about its hinge with the simulator's deflection (${(out * 180 / Math.PI).toFixed(0)} deg, then ${(stood * 180 / Math.PI).toFixed(0)} deg)`);
  }
  const fin = world.vehicle.flaps.find((f) => f.kind === 'grid_fin');
  if (fin) {
    const bits = (v) => (app.spec.surfaces || []).reduce((a, x, i) => a | ((v ? 1 : 0) << i), 0);
    for (let k = 0; k < 90; k++) world.vehicle.apply({ ...app.pose, sdep: bits(true) });                 // 90 calls: the fold takes 1.5 s of wall time at 60 per second, so the call count and not the clock decide here
    const folded0 = fin.fold;
    for (let k = 0; k < 20; k++) fin.set(0, false, 0.1);                                                 // 2 s of folding, in steps of the 0.1 s a call may take at most
    check(fin.fold > folded0 || fin.fold === 1, 'a grid fin folds when the pose says it is not deployed');
    for (let k = 0; k < 20; k++) fin.set(0, true, 0.1);
    check(fin.fold === 0, 'and deploys again when it is');
  }
  // a mission with another body (the stage that returns): it is drawn as a vehicle of its own, the viewer can follow it, the lenses then measure it and not the stack, and the pad it returns to is there
  const rawNow = app.data ? app.data.buf.latest : null;
  if (rawNow && rawNow.others && rawNow.others.length) {
    const names = rawNow.others.map((o) => o.name);
    check(names.every((n) => world.bodies.has(n)), `the other bodies (${names.join(', ')}) are in the scene`);
    const ext = app.spec.extent(app.pose.stg, app.pose.pay);
    check(ext.length < app.spec.length - 1, `after the separation the lenses measure what is left (${ext.length.toFixed(1)} m of ${app.spec.length.toFixed(1)} m)`);
    check(!!world.catchPad, 'the mission has a pad of its own for the stage that returns');
    const btn = document.querySelector(`#v-follow button[data-id="${names[0]}"]`);
    check(!!btn, 'the follow bar offers the other body');
    if (btn) {
      btn.click(); await settle(900);
      check(app.spec.stages.length === 1 && /booster|stage/i.test(app.spec.name), `following ${names[0]}: the spec is that stage's alone (${app.spec.name})`);
      const v = new THREE.Vector3(0, 0, 0).project(world.camera);
      check(Math.abs(v.x) <= 1 && Math.abs(v.y) <= 1, `following ${names[0]}: it is inside the picture`);
      check(world.vehicle.engines.length === app.spec.engines.length, `following ${names[0]}: the model has its ${app.spec.engines.length} engines`);
      const own = app.d && Number.isFinite(app.d.q);
      check(own, 'the lenses work on the followed body');
      document.querySelector('#v-follow button[data-id="main"]').click(); await settle(700);
      check(app.spec === app.data.spec, 'following the main vehicle again');
    }
  }
  // the Aerodynamics lens's four views, and in the CFD one the flow the solver computed and the live solver: no error, the cards have numbers, the live solver has run
  {
    setLens('aero'); await settle(300);
    const aero = lenses.find((l) => l.id === 'aero'), S = aero.s, was = S.opts.view;
    for (const v of ['flow', 'pressure', 'stability', 'cfd']) {
      const before = errors.length;
      S.opts.view = v; aero.applyView(app, false); await settle(500);
      check(errors.length === before, `aerodynamics view ${v}: no error`);
    }
    // at a Mach number with air and a case near it, and no more than this: the checks do not need a library, only that what is there works
    S.opts.view = 'cfd'; S.opts.live = true; S.opts.csec = true; aero.applyView(app, false); await settle(900);
    const cases = S.cfdState.list.length;
    check(S.cfdState.list !== undefined, `the CFD view asked the server for flow fields (${cases} listed)`);
    if (app.d && app.d.air && app.d.M > 1.25) {
      check(!!S.live && (S.live.ok === false || S.live.iterations > 0), `the live solver ${S.live && S.live.ok ? 'ran ' + S.live.iterations + ' iterations at Mach ' + S.live.mach.toFixed(2) : 'is not available here'}`);
      if (S.live && S.live.ok) {
        const d = S.live.read(); const finite = d.every((x) => Number.isFinite(x)); const rho = []; for (let i = 0; i < d.length; i += 4) rho.push(d[i]);
        check(finite && Math.min(...rho) > 0, 'the live solver\'s state is finite and its density positive everywhere');
      }
    }
    const first = (() => { const m = app.pose.stg ?? 255; for (let i = 0; i < 8; i++) if (m & (1 << i)) return i; return 0; })();
    if (S.cfdState.list.some((c) => sameVehicle(app.spec.name, c.vehicle) && (c.stage_from == null || c.stage_from === first))) { await settle(1500); check(!!S.cfd.cfd, `a flow field was loaded (${S.cfd.name})`); check(S.cfdStats.cells.el === undefined || /[0-9]/.test(app.left.querySelector('#v-aero-cfdcard')?.textContent || ''), 'the solver card has numbers'); }
    S.opts.view = was; aero.applyView(app, false); await settle(300);
  }
  // the gyro: the three angles of the gimbal set rebuild the attitude they were taken from (for attitudes all round, the vertical included), and agree with the simulator's own tilt
  {
    let worst = 0, seed = 7; const rnd = () => (seed = (seed * 1664525 + 1013904223) % 4294967296) / 4294967296;
    for (let i = 0; i < 300; i++) {
      const q = new THREE.Quaternion(rnd() - 0.5, rnd() - 0.5, rnd() - 0.5, rnd() - 0.5 + 1e-3).normalize();
      const a = gimbalAngles(q, 0), back = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 1, 0), a.lean).multiply(new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1, 0, 0), a.tilt)).multiply(new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 1, 0), a.roll));
      worst = Math.max(worst, 1 - Math.abs(back.dot(q)));
    }
    check(worst < 1e-9, `gimbalAngles: 300 attitudes rebuilt from lean, tilt and roll (worst 1 - |q.q'| = ${worst.toExponential(1)})`);
    const vert = gimbalAngles(new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 1, 0), 0.7), 1.2);
    check(Math.abs(vert.tilt) < 1e-9 && Math.abs(vert.lean - 1.2) < 1e-12 && Math.abs(vert.lean + vert.roll - 0.7) < 1e-9, 'gimbalAngles: a vehicle standing up keeps its last lean, and the lean and the roll together are its turn about the vertical');
    if (app.pose && !app.pose.truthOnly && app.pose.tilt) {
      const { world } = app, q = world.localQ.clone().invert().multiply(world.qs), a = gimbalAngles(q, 0), tp = app.pose.tilt[0] * Math.PI / 180, ty = app.pose.tilt[1] * Math.PI / 180;
      const sim = Math.acos(Math.cos(tp) * Math.cos(ty)) * 180 / Math.PI, mine = a.tilt * 180 / Math.PI, curve = Math.abs(app.pose.rng || 0) / app.spec.planet.radius * 180 / Math.PI;
      check(Math.abs(sim - mine) <= curve + 0.7, `the gyro's tilt (${mine.toFixed(1)} deg from the local vertical) agrees with the simulator's (${sim.toFixed(1)} deg from the launch vertical) within the planet's curvature over the range (${curve.toFixed(1)} deg)`);
    }
  }
  // the director: the pad, the climb, the separation and the stage coming home each get their shot, and a shot is held
  {
    const D = new Director(), P = (o) => ({ t: 0, alt: 0, thr: 1, clamp: 0, stg: 3, others: [], ...o });
    check(D.shot(P({ t: -5, clamp: 1 })).cam === 'ground', 'director: the pad is the ground camera');
    check(D.shot(P({ t: 30, alt: 8000 })).cam === 'track', 'director: the climb is the tracking camera');
    D.shot(P({ t: 120, alt: 60000, stg: 2, others: [{ name: 'booster', stage: 0, pose: { alt: 62000, thr: 1 } }] }));
    check(D.shot(P({ t: 121, alt: 60000, stg: 2, others: [{ name: 'booster', stage: 0, pose: { alt: 62000, thr: 1 } }] })).cam === 'chase', 'director: just after a separation it is a chase view');
    const hold = D.shot(P({ t: 123, alt: 61000, stg: 2, others: [{ name: 'booster', stage: 0, pose: { alt: 5000, thr: 0 } }] }));
    check(hold.cam === 'chase' && !hold.cut, 'director: a shot is held for a few seconds, whatever the numbers do');
    const home = D.shot(P({ t: 300, alt: 150000, stg: 2, others: [{ name: 'booster', stage: 0, pose: { alt: 900, thr: 1 } }] }));
    check(home.cam === 'ground' && home.focus === 'booster' && home.cut, 'director: the stage in its last kilometre is watched from the ground');
  }
  check(callName('lift-off') === 'LIFTOFF' && callName('booster separated') === 'BOOSTER SEPARATION' && callName('max-Q passed: 28.1 kPa at T+72.0 s') === 'MAX Q' && callName('ship ignited') === 'SHIP IGNITION' && callName('booster caught') === 'BOOSTER CAUGHT', 'the events are called by their names on the broadcast');
  {                                                       // the table map reads a table the way the plant does: at a node it is the node, between two it is between
    const tb = tableFor(app.spec.vehicle && app.spec.vehicle.aero && app.spec.vehicle.aero.cfd, 3);
    if (tb) {
      const m = tb.mach, a = tb.alpha_deg;
      check(Math.abs(sampleTable(tb, 'cn', m[1], a[2]) - tb.cn[1][2]) < 1e-12, 'the table map: at a node it is the node\'s value');
      const mid = sampleTable(tb, 'cn', 0.5 * (m[1] + m[2]), a[2]), lo = Math.min(tb.cn[1][2], tb.cn[2][2]), hi = Math.max(tb.cn[1][2], tb.cn[2][2]);
      check(mid >= lo - 1e-12 && mid <= hi + 1e-12 && Math.abs(mid - 0.5 * (tb.cn[1][2] + tb.cn[2][2])) < 1e-12, 'the table map: halfway between two Mach nodes it is halfway between their values');
      check(sampleTable(tb, 'cn', 0.5, a[2]) === tb.cn[0][2] && sampleTable(tb, 'cn', 99, a[2]) === tb.cn[m.length - 1][2], 'the table map: held at the first and last Mach number, as the plant holds it');
    }
  }
  // a card folds when its header is clicked, and unfolds again
  setLens('aero'); await settle(300);
  const cardEl = app.right.querySelector('.v-card'); const was = cardEl.classList.contains('collapsed');
  cardEl.querySelector('header').click(); const now = cardEl.classList.contains('collapsed'); cardEl.querySelector('header').click();
  check(was !== now && cardEl.classList.contains('collapsed') === was, 'a card folds and unfolds when its header is clicked');
  world.rig.set('orbit');
  for (const s of STYLES) {
    const before = errors.length;
    app.styleName = s.id; app.rebuild(); await settle(500);
    check(errors.length === before && world.vehicle && world.vehicle.root.children.length > 0, `model style ${s.id}: built and drawn`);
  }
  app.styleName = 'auto'; app.rebuild(); await settle(300);
  setLens('overview'); await settle(200);
  check(errors.length === 0, `no error at all in the whole run${errors.length ? ': ' + errors[0] : ''}`);
  console.error = keep;
  const text = `${fails ? 'selftest FAILED' : 'selftest ok'}: ${out.length - fails} of ${out.length} checks\n${out.join('\n')}`;
  const pre = document.createElement('pre');
  pre.id = 'selftest'; pre.style.cssText = 'position:fixed;left:360px;top:60px;z-index:80;margin:0;padding:12px;background:rgba(0,0,0,.88);color:#cfe;font:12px/1.35 monospace;max-height:calc(100vh - 80px);overflow:hidden;border:1px solid #3a6;border-radius:8px;pointer-events:none';
  pre.textContent = text; document.body.append(pre); document.title = fails ? 'selftest FAILED' : 'selftest ok';
  if (window.__dbg) window.__dbg('selftest', fails ? 'FAILED' : 'ok', `${out.length - fails} of ${out.length}`, out.filter((l) => l.startsWith('FAIL')).join(' | '), out.slice(-8).join(' | '));      // with ?debug=1 the server's log has the verdict too
  return { fails, text };
}
