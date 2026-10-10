const o = document.getElementById('o');
window.addEventListener('error', (e) => { o.textContent += 'WINERR ' + e.message + ' @' + e.filename + ':' + e.lineno + '\n'; });
console.error = (...a) => { o.textContent += 'console.error ' + a.map(String).join(' ') + '\n'; };
console.warn = (...a) => { o.textContent += 'console.warn ' + a.map(String).join(' ').slice(0, 600) + '\n'; };
try { await import('./js/main.js'); o.textContent += 'main ok\n'; } catch (e) { o.textContent += 'FAIL main: ' + e.message + '\n' + (e.stack || '') + '\n'; }
await new Promise((r) => setTimeout(r, 2500));
o.textContent += 'frames? ' + (window.__viewer ? 'viewer present, haveSpec=' + window.__viewer.data.haveSpec + ' status=' + window.__viewer.data.status + ' polls=' + window.__viewer.data.poseCount : 'no viewer') + '\n';
const img = new Image(); img.src = '/api/hold?ms=500&token=test'; document.body.append(img);
