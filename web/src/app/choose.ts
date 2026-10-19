/* **What a ⌘-click had under it**, listed to choose among: the drawing's figures and
 * points, then an object's edges and faces nearest the eye first — an edge behind the material
 * too, said so — then the objects whole, as the core ordered them (`workspace::under`).  The
 * row the pointer or the keyboard is on is lit where it is drawn, as the pointer lights what it
 * is over on the canvas, and the one chosen is taken as a click on it would be.  The view says
 * what is there (`SketchView.onChoose`); this is only the list. */
import { view } from './shell.js';
import { hitName } from './view.js';
import type { Hit } from './view.js';

/** The list open, if one is, and what closes it. */
let open: { el: HTMLElement; away: (e: Event) => void } | null = null;

/** List `hits` at a canvas point.  `add` is the ⌘-click's shift: the one chosen joins the
 *  selection rather than replacing it, as a shift-click's does — and so does one chosen with
 *  shift held. */
export function onChoose(hits: Hit[], at: [number, number], add: boolean): void {
  closeChooser();
  const wrap = document.getElementById('canvas-wrap') as HTMLElement;
  const el = document.createElement('div');
  el.className = 'chooser';
  el.setAttribute('role', 'menu');
  const head = document.createElement('div');
  head.className = 'chooser-head';
  head.textContent = `${hits.length} under the pointer`;
  el.append(head);
  const rows = hits.map((hit) => {
    const { kind, name, hidden } = hitName(view.doc, hit);
    const row = document.createElement('button');
    row.type = 'button';
    row.setAttribute('role', 'menuitem');
    const cell = (cls: string, text: string) => {
      const span = document.createElement('span');
      span.className = cls;
      span.textContent = text;
      row.append(span);
    };
    cell('kind', kind);
    cell('name', name);
    if (hidden) cell('note', 'hidden');
    row.addEventListener('pointerenter', () => row.focus());
    row.addEventListener('focus', () => view.hoverHit(hit));
    row.addEventListener('click', (e) => {
      closeChooser();
      view.takeHit(hit, add || e.shiftKey);
    });
    el.append(row);
    return row;
  });
  el.addEventListener('pointerleave', () => view.hoverHit(null));
  el.addEventListener('keydown', (e) => {
    // the list's own keys: the page's (Escape backing out of a tool, a tool's letter) wait
    e.stopPropagation();
    const k = rows.indexOf(document.activeElement as HTMLButtonElement);
    if (e.key === 'Escape') {
      e.preventDefault();
      closeChooser();
    } else if (e.key === 'ArrowDown' || e.key === 'ArrowUp') {
      e.preventDefault();
      const step = e.key === 'ArrowDown' ? 1 : -1;
      rows[(k + step + rows.length) % rows.length].focus();
    }
  });
  // where the click was, kept inside the viewport
  wrap.append(el);
  const x = Math.max(4, Math.min(at[0] + 8, wrap.clientWidth - el.offsetWidth - 4));
  const y = Math.max(4, Math.min(at[1] + 8, wrap.clientHeight - el.offsetHeight - 4));
  el.style.left = `${x}px`;
  el.style.top = `${y}px`;
  // a press anywhere else, or the wheel, is done with it
  const away = (e: Event) => {
    if (!el.contains(e.target as Node)) closeChooser();
  };
  document.addEventListener('pointerdown', away, true);
  document.addEventListener('wheel', away, true);
  open = { el, away };
  rows[0]?.focus();
}

/** Close the list, if it is open, and put out what its row lit.  Whether one was open. */
export function closeChooser(): boolean {
  if (!open) return false;
  const { el, away } = open;
  open = null;
  document.removeEventListener('pointerdown', away, true);
  document.removeEventListener('wheel', away, true);
  el.remove();
  view.hoverHit(null);
  return true;
}
