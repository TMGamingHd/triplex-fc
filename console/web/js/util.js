// Small helpers shared by every tab: DOM building, number and time formatting, toasts, modals, icons.
export const $ = (sel, root = document) => root.querySelector(sel);
export const $$ = (sel, root = document) => Array.from(root.querySelectorAll(sel));

/** h('div', {class:'x', onclick:fn, dataset:{a:1}}, 'text', child, ...) */
export function h(tag, attrs, ...kids) {
  const el = document.createElement(tag);
  for (const [k, v] of Object.entries(attrs || {})) {
    if (v == null || v === false) continue;
    if (k === 'class') el.className = v;
    else if (k === 'text') el.textContent = v;
    else if (k === 'html') el.innerHTML = v;          // only ever given strings this code made itself
    else if (k === 'style' && typeof v === 'object') Object.assign(el.style, v);
    else if (k === 'dataset') Object.assign(el.dataset, v);
    else if (k.startsWith('on') && typeof v === 'function') el.addEventListener(k.slice(2), v);
    else if (v === true) el.setAttribute(k, '');
    else el.setAttribute(k, v);
  }
  for (const kid of kids.flat()) {
    if (kid == null || kid === false) continue;
    el.append(kid.nodeType ? kid : document.createTextNode(String(kid)));
  }
  return el;
}

export function svg(tag, attrs, ...kids) {
  const el = document.createElementNS('http://www.w3.org/2000/svg', tag);
  for (const [k, v] of Object.entries(attrs || {})) if (v != null) el.setAttribute(k, v);
  for (const kid of kids.flat()) if (kid != null) el.append(kid.nodeType ? kid : document.createTextNode(String(kid)));
  return el;
}

export function setText(el, v) { const s = v == null ? '—' : String(v); if (el.textContent !== s) el.textContent = s; }
export function setClass(el, cls, on) { el.classList.toggle(cls, !!on); }
export function clear(el) { while (el.firstChild) el.removeChild(el.firstChild); }

export const NODES = ['A', 'B', 'C'];
export const num = (v, d = 1, unit = '') => (v == null || Number.isNaN(v) ? '—' : Number(v).toFixed(d) + unit);
export function eng(v, d = 1) {                       // 12345 -> "12.3k"
  if (v == null || Number.isNaN(v)) return '—';
  const a = Math.abs(v);
  if (a >= 1e6) return (v / 1e6).toFixed(d) + 'M';
  if (a >= 1e4) return (v / 1e3).toFixed(d) + 'k';
  return Number(v).toFixed(a >= 100 ? 0 : d);
}
export function clock(sec) {                          // seconds -> "01:23.4"
  if (sec == null) return '--:--.-';
  const s = Math.max(0, sec);
  const m = Math.floor(s / 60);
  return String(m).padStart(2, '0') + ':' + (s - m * 60).toFixed(1).padStart(4, '0');
}
export const ago = (s) => (s == null ? '—' : s < 1 ? `${Math.round(s * 1000)} ms` : s < 100 ? `${s.toFixed(1)} s` : `${Math.round(s)} s`);
export function hms(wall) { const d = new Date(wall * 1000); return d.toTimeString().slice(0, 8); }

export function levelOf(health) {                      // a node's health word -> a status colour name
  return { healthy: 'ok', probation: 'warn', latched: 'warn', disabled: 'crit', unknown: 'muted' }[health] || 'muted';
}

/* ---------- toasts ---------- */
export function toast(msg, level = 'info', ms = 5200) {
  const box = $('#toasts');
  const el = h('div', { class: `toast ${level}`, role: level === 'crit' || level === 'warn' ? 'alert' : 'status' }, msg);
  box.append(el);
  setTimeout(() => el.remove(), ms);
  while (box.children.length > 5) box.firstChild.remove();
}

/* ---------- modal ---------- */
export function modal({ title, body, buttons, onOpen }) {
  return new Promise((resolve) => {
    const close = (v) => { bg.remove(); document.removeEventListener('keydown', onKey); resolve(v); };
    const onKey = (e) => { if (e.key === 'Escape') close(null); };
    const foot = h('footer', {}, buttons.map((b) => h('button', { class: `btn ${b.kind || ''}`, id: b.id, onclick: () => close(b.value ?? b.label), disabled: b.disabled }, b.label)));
    const bg = h('div', { class: 'modal-bg', onclick: (e) => { if (e.target === bg) close(null); } },
      h('div', { class: 'modal', role: 'dialog', 'aria-modal': 'true', 'aria-label': title }, h('header', {}, title), h('div', { class: 'body' }, body), foot));
    document.body.append(bg);
    document.addEventListener('keydown', onKey);
    if (onOpen) onOpen({ root: bg, close, foot });
  });
}

/* ---------- icons (24x24 line icons) ---------- */
const P = {
  mission: 'M12 3l2.5 6H21l-5.2 4 2 6.5L12 15.5 6.2 19.5l2-6.5L3 9h6.5z',
  launch: 'M12 2c3 3 4.5 7 4 11l-4 4-4-4c-.5-4 1-8 4-11zM9 17l-3 4M15 17l3 4M12 8.5a1.5 1.5 0 100 .01',
  voting: 'M4 6h16M4 12h16M4 18h16M8 3v6M16 9v6M10 15v6',
  flight: 'M3 20l6-6 4 3 8-12M3 20h18',
  commands: 'M4 5h16v14H4zM8 10l3 2-3 2M13 15h4',
  faults: 'M13 2L4 14h7l-1 8 9-12h-7z',
  vehicle: 'M12 2l3 5v8l-3 3-3-3V7zM9 12l-4 4v3l4-2M15 12l4 4v3l-4-2',
  rig: 'M4 6h16v5H4zM4 13h16v5H4zM7 8.5h.01M7 15.5h.01',
  bus: 'M3 12h18M7 12V7M12 12V5M17 12V8M7 12v5M12 12v7M17 12v4',
  events: 'M5 4h14M5 9h14M5 14h9M5 19h6',
  alert: 'M12 3l10 18H2zM12 10v5M12 18v.01',
  info: 'M12 12m-9 0a9 9 0 1018 0 9 9 0 10-18 0M12 11v6M12 7.5v.01',
  check: 'M5 13l4 4 10-10',
  x: 'M6 6l12 12M18 6L6 18',
  play: 'M7 4l13 8-13 8z', pause: 'M7 4h4v16H7zM13 4h4v16h-4z',
};
export function icon(name, cls = '') {
  const s = svg('svg', { viewBox: '0 0 24 24', fill: 'none', stroke: 'currentColor', 'stroke-width': 1.8, 'stroke-linecap': 'round', 'stroke-linejoin': 'round', class: cls, 'aria-hidden': 'true' }, svg('path', { d: P[name] || P.info }));
  return s;
}

/* ---------- download ---------- */
export function download(name, text, type = 'text/plain') {
  const a = h('a', { href: URL.createObjectURL(new Blob([text], { type })), download: name });
  document.body.append(a); a.click(); setTimeout(() => { URL.revokeObjectURL(a.href); a.remove(); }, 500);
}

export const store = {                                  // a localStorage that never throws (private windows, blocked storage)
  get(k, d) { try { const v = localStorage.getItem('tfc.' + k); return v == null ? d : JSON.parse(v); } catch { return d; } },
  set(k, v) { try { localStorage.setItem('tfc.' + k, JSON.stringify(v)); } catch { /* not remembered */ } },
};
