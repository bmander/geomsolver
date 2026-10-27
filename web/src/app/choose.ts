/* **What a ⌘-click had under it**, listed to choose among: the drawing's figures and
 * points, then an object's edges and faces nearest the eye first — an edge behind the material
 * too, said so — then the objects whole, as the core ordered them (`workspace::under`).  The
 * row the pointer or the keyboard is on is lit where it is drawn, as the pointer lights what it
 * is over on the canvas, and the one chosen is taken as a click on it would be.  The view says
 * what is there (`SketchView.onChoose`); this is only the list.  A right-click's menu is such a
 * list too (`onContextMenu`), of what can be done with the one thing it was on. */
import type { SourceSpan } from '../core/program.js';
import type { Row } from '../core/outline.js';
import { view } from './shell.js';
import { hide, hideableRow, partToOpen, rowOf } from './hidden.js';
import { bodyOf, hitName } from './view.js';
import type { Hit } from './view.js';

/** The list open, if one is, and what closes it. */
let open: { el: HTMLElement; away: (e: Event) => void } | null = null;

/** Where a right-click's ways to what it was on lead, as the page wires them (`onWays`): its
 *  statement in the program editor, and its row in the model outline. */
let toSource: (span: SourceSpan) => void = () => {};
let toRow: (row: Row) => void = () => {};

/** Be told how to go to a statement and to an outline row — the panel's, which this list does
 *  not hold. */
export function onWays(source: (span: SourceSpan) => void, row: (row: Row) => void): void {
  toSource = source;
  toRow = row;
}

/** List `hits` at a canvas point.  `add` is the ⌘-click's shift: the one chosen joins the
 *  selection rather than replacing it, as a shift-click's does — and so does one chosen with
 *  shift held. */
export function onChoose(hits: Hit[], at: [number, number], add: boolean): void {
  openList(`${hits.length} under the pointer`, hits.map((hit) => {
    const { kind, name, hidden } = hitName(view.doc, hit);
    return {
      cells: [['kind', kind], ['name', name], ...(hidden ? [['note', 'hidden'] as Cell] : [])],
      hit,
      take: (e: MouseEvent) => view.takeHit(hit, add || e.shiftKey),
    };
  }), at);
}

/** **A right-click's menu** on what it was on, `hit` (an object's face or edge standing for the
 *  object): Show definition, the program editor's caret on the statement that made it; Show in
 *  model viewer, the outline opened to its row; Hide, that row hidden; Open, a closed part's
 *  object's part opened, and Close part while one is (#679); and Show all while anything is
 *  hidden.  With none to offer nothing opens; on something that cannot be hidden, it
 *  says so. */
export function onContextMenu(hit: Hit | null, at: [number, number]): void {
  closeChooser();
  const items: Item[] = [];
  const whole: Hit | null = hit?.kind === 'solid' ? { kind: 'solid', pick: bodyOf(hit.pick) } : hit;
  const target = !whole ? null : whole.kind === 'entity' ? whole.ent : whole.pick.index;
  const made = target === null ? null : rowOf(view, target);
  const span = made?.span;
  if (span) {
    items.push({ cells: [['verb', 'Show definition']], hit: whole, take: () => toSource(span) });
  }
  if (made) {
    items.push({ cells: [['verb', 'Show in model viewer']], hit: whole,
                 take: () => toRow(made) });
  }
  const row = target === null ? null : hideableRow(view, target);
  if (row && target !== null) {
    items.push({ cells: [['verb', 'Hide'], ['name', row.label]], hit: whole,
                 take: () => { hide(view, target); } });
  }
  // a closed part's object opens the part (#679), as a double-click on it does; one open closes
  const part = target === null ? null : partToOpen(view, target);
  if (part) {
    items.push({ cells: [['verb', 'Open'], ['name', part.label]], hit: whole,
                 take: () => { view.openPart(part); } });
  }
  if (view.opened !== null) {
    items.push({ cells: [['verb', 'Close part'], ['note', 'Esc']], hit: null,
                 take: () => { view.closePart(); } });
  }
  if (view.hiddenRows.size) {
    items.push({ cells: [['verb', 'Show all'], ['note', '⇧I']], hit: null,
                 take: () => view.showAll() });
  }
  if (whole && !row) view.onStatus(`${hitName(view.doc, whole).name} cannot be hidden`);
  if (!items.length) return;
  const named = whole && hitName(view.doc, whole);
  openList(named ? `${named.kind} ${named.name}` : null, items, at);
}

/** A row's text, by the class each span is drawn in. */
type Cell = [string, string];

/** One row of a list: its text, what it lights while it is pointed at, and what choosing it does. */
interface Item {
  cells: Cell[];
  hit: Hit | null;
  take: (e: MouseEvent) => void;
}

/** Open a list of `items` under a `head` at a canvas point, kept inside the viewport: the row
 *  pointed at or reached by the arrows lit where it is drawn, a press anywhere else or the wheel
 *  done with it. */
function openList(head: string | null, items: Item[], at: [number, number]): void {
  closeChooser();
  const wrap = document.getElementById('canvas-wrap') as HTMLElement;
  const el = document.createElement('div');
  el.className = 'chooser';
  el.setAttribute('role', 'menu');
  if (head !== null) {
    const top = document.createElement('div');
    top.className = 'chooser-head';
    top.textContent = head;
    el.append(top);
  }
  const rows = items.map(({ cells, hit, take }) => {
    const row = document.createElement('button');
    row.type = 'button';
    row.setAttribute('role', 'menuitem');
    for (const [cls, text] of cells) {
      const span = document.createElement('span');
      span.className = cls;
      span.textContent = text;
      row.append(span);
    }
    row.addEventListener('pointerenter', () => row.focus());
    row.addEventListener('focus', () => view.hoverHit(hit));
    row.addEventListener('click', (e) => {
      closeChooser();
      take(e);
    });
    el.append(row);
    return row;
  });
  el.addEventListener('pointerleave', () => view.hoverHit(null));
  // a right-click on the list is the list's, not the browser's menu over it
  el.addEventListener('contextmenu', (e) => e.preventDefault());
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
