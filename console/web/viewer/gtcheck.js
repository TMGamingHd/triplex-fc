// Parses modules in a headless browser, which has no console to read: a syntax error in an ES module does not reach window.onerror, so each is import()ed and the message printed on the page.
const o = document.getElementById('o');
const q = new URLSearchParams(location.search);
{ const img = new Image(); img.src = '/api/hold?ms=6000&token=' + (q.get('token') || ''); document.body.append(img); }
const list = (q.get('m') || './js/gpuflow.js,./js/cfd.js,./js/flow.js,./js/physics.js,./js/lenses/aero.js,./js/lenses/atmosphere.js,./js/selftest.js,./js/data.js').split(',');
for (const m of list) { try { await import(m); o.textContent += 'ok ' + m + '\n'; } catch (e) { o.textContent += 'FAIL ' + m + ': ' + e.message + ' ' + (e.stack || '').slice(0, 300) + '\n'; } }
