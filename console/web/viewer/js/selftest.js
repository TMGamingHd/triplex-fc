// A smoke test that runs in the page itself (`?selftest=1`, the way the console has one): it goes through every lens, every camera and every generated model style on whatever flight is being shown, and checks that
// nothing threw, that the lens drew its panels, and that a frame was rendered after each change. It prints the result on the page, because a headless browser has no console to read and its screenshot is the only
// way out; the result is also in the page's title (`selftest ok` or `selftest FAILED`).
import { CAMERAS } from './cam.js';
import { STYLES } from './models/vehicle.js';

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
