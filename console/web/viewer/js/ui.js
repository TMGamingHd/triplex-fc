// Small building blocks for the viewer's panels, in the console's own look (its tokens and its .card, .stat, .seg classes): a stat tile that updates in place, a segmented control, a toggle, a slider.
import { h, setText, store } from '../../js/util.js';

export { h, setText, store };

/** A labelled number: `set(text)` changes only what changed. level: '', 'ok', 'warn', 'crit', 'dim'. */
export function stat(label, unit = '', hint = '') {
  const v = h('div', { class: 'v' }), u = unit ? h('small', {}, unit) : null;
  const el = h('div', { class: 'stat', title: hint }, h('div', { class: 'k' }, label), h('div', { class: 'v-row' }, v));
  const val = h('span'); v.append(val); if (u) v.append(u);
  return { el, set(text, level = '') { setText(val, text); if (el.dataset.level !== level) { el.dataset.level = level; el.className = 'stat' + (level ? ' ' + level : ''); } } };
}

export const statGrid = (cols, ...stats) => h('div', { class: 'stats', style: { gridTemplateColumns: `repeat(${cols}, minmax(0, 1fr))` } }, stats.map((s) => s.el));

/** A card with a title, in a dock. */
export function card(title, body, tools) {
  return h('article', { class: 'card v-card' }, h('header', {}, h('h3', {}, title), tools ? h('div', { class: 'tools' }, tools) : null), h('div', { class: 'body' }, body));
}

/** A segmented control: options [[value, label, hint]], current value, onchange(value). */
export function seg(options, value, onchange, label = '') {
  const root = h('div', { class: 'seg', role: 'group', 'aria-label': label });
  const set = (v) => { for (const b of root.children) b.setAttribute('aria-pressed', b.dataset.v === String(v) ? 'true' : 'false'); };
  for (const [v, text, hint] of options) root.append(h('button', { type: 'button', dataset: { v }, title: hint || '', 'aria-pressed': String(v) === String(value) ? 'true' : 'false', onclick: () => { set(v); onchange(v); } }, text));
  return Object.assign(root, { setValue: set });
}

/** A checkbox that reads as a switch: returns the element and `.checked`. */
export function toggle(label, value, onchange, hint = '') {
  const input = h('input', { type: 'checkbox', checked: value ? true : null });
  input.checked = !!value;
  input.addEventListener('change', () => onchange(input.checked));
  const el = h('label', { class: 'v-toggle', title: hint }, input, h('span', {}, label));
  return Object.assign(el, { input });
}

export function slider(label, min, max, step, value, onchange, fmt = (v) => v) {
  const out = h('output', {}, fmt(value));
  const input = h('input', { type: 'range', min, max, step, value });
  input.addEventListener('input', () => { out.textContent = fmt(+input.value); onchange(+input.value); });
  return Object.assign(h('label', { class: 'v-slider' }, h('span', {}, label), input, out), { input });
}

export function select(options, value, onchange, label = '') {
  const s = h('select', { 'aria-label': label }, options.map(([v, t]) => h('option', { value: v, selected: String(v) === String(value) ? true : null }, t)));
  s.addEventListener('change', () => onchange(s.value));
  return s;
}

/** A line of text that is marked as a measurement, a formula or an illustration, as the project's rule is that a number says where it came from. */
export function provenance(kind, text) {
  const label = { measured: 'from the simulator', derived: 'derived by a formula', illustrative: 'illustration' }[kind] || kind;
  return h('p', { class: 'v-prov ' + kind }, h('b', {}, label + ': '), text);
}

export const fmt = {
  n(v, d = 1) { return v == null || !Number.isFinite(v) ? '—' : Number(v).toFixed(d); },
  eng(v, d = 1) { if (v == null || !Number.isFinite(v)) return '—'; const a = Math.abs(v); if (a >= 1e9) return (v / 1e9).toFixed(d) + 'G'; if (a >= 1e6) return (v / 1e6).toFixed(d) + 'M'; if (a >= 1e4) return (v / 1e3).toFixed(d) + 'k'; return a >= 100 ? v.toFixed(0) : v.toFixed(d); },
  km(m, d = 1) { return m == null ? '—' : (m / 1000).toFixed(d); },
  t(sec) { if (sec == null) return '--:--.-'; const s = sec < 0 ? '−' : '+'; const a = Math.abs(sec); const m = Math.floor(a / 60); return `T${s}${String(m).padStart(2, '0')}:${(a - m * 60).toFixed(1).padStart(4, '0')}`; },
};
