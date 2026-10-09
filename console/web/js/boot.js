// Loaded first, as a classic script: if a module fails to load (a syntax error, a missing file) the page would be blank and the reason invisible, so this puts it on the page.
(function () {
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
    if (e.target && e.target !== window) return;           // a resource that failed (an image, a stylesheet) is not a script error
    show('Script error: ' + (e.message || 'failed to load a module') + (e.filename ? '\n  at ' + e.filename + ':' + e.lineno + ':' + e.colno : '')); }, true);
  window.addEventListener('unhandledrejection', function (e) { show('Unhandled: ' + (e.reason && (e.reason.stack || e.reason.message) || e.reason)); });
})();
