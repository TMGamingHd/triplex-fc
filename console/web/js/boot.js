// Loaded first, as a classic script: if a module fails to load (a syntax error, a missing file) the page would be blank and the reason invisible, so this puts it on the page.
(function () {
  window.__query = location.search;                          // net.js takes the token out of the address bar; a page that reads its own options from it needs the whole address as it was opened
  function show(msg) {
    var box = document.getElementById('boot-error');
    if (!box) {
      box = document.createElement('pre');
      box.id = 'boot-error';
      box.style.cssText = 'position:fixed;inset:70px 16px auto 16px;z-index:99;background:#3a0f18;color:#ffd7dd;border:1px solid #ff5d73;border-radius:8px;padding:12px;font:12px/1.4 monospace;white-space:pre-wrap;max-height:60vh;overflow:auto';
      (document.body || document.documentElement).appendChild(box);
    }
    box.textContent += msg + '\n';
  }
  window.addEventListener('error', function (e) {
    if (e.target && e.target.tagName === 'SCRIPT') { show('A script failed to load or parse: ' + (e.target.src || 'inline') + ' (the browser console says which file and line)'); return; }
    if (e.target && e.target !== window) return;           // a resource that failed (an image, a stylesheet) is not a script error
    show('Script error: ' + (e.message || 'failed to load a module') + (e.filename ? '\n  at ' + e.filename + ':' + e.lineno + ':' + e.colno : '')); }, true);
  window.addEventListener('unhandledrejection', function (e) { show('Unhandled: ' + (e.reason && (e.reason.stack || e.reason.message) || e.reason)); });
  // `?hold=ms`: the page does not finish loading for that long. A headless browser takes its screenshot at the load event; a module graph and a stream take longer than that to show anything.
  try {
    var q = new URLSearchParams(location.search), ms = +(q.get('hold') || 0), tok = q.get('token') || sessionStorage.getItem('tfc.token') || '';
    if (ms > 0) { var im = new Image(); im.style.display = 'none'; im.src = '/api/hold?ms=' + ms + '&token=' + encodeURIComponent(tok); document.documentElement.appendChild(im); }
  } catch (e) { /* no hold */ }
})();
