// A smoke test that runs in the page itself (`?selftest=1`, the way the console has one): it goes through every lens, every camera and every generated model style on whatever flight is being shown, and checks that
// nothing threw, that the lens drew its panels, and that a frame was rendered after each change. It prints the result on the page, because a headless browser has no console to read and its screenshot is the only
// way out; the result is also in the page's title (`selftest ok` or `selftest FAILED`).
import { CAMERAS } from './cam.js';
import { STYLES } from './models/vehicle.js';
import * as THREE from '../vendor/three.module.js';

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
  // the flaps of the vehicle file are in the model, and they follow the deflection in the pose
  const flaps = (app.spec.surfaces || []).filter((x) => x.kind === 'flap');
  check(world.vehicle.flaps.length === flaps.length, `the model has the vehicle file's ${flaps.length} flaps`);
  if (flaps.length) {
    const f = world.vehicle.flaps[0];
    world.vehicle.apply({ ...app.pose, surf: flaps.map(() => 40) }); const out = f.pivot.rotation.x;
    world.vehicle.apply({ ...app.pose, surf: flaps.map(() => 0) }); const flush = f.pivot.rotation.x;
    check(Math.abs(out) > 0.5 && Math.abs(flush) < 1e-9, `a flap turns with the simulator's deflection (${(out * 180 / Math.PI).toFixed(0)} deg out, ${(flush * 180 / Math.PI).toFixed(0)} deg flush)`);
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
  return { fails, text };
}
