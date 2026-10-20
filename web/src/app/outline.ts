/* **The model outline pane** (#435): every instance the document makes and its members, as a
 * tree under the project's files, styled as that tree is — for finding the way round a large
 * document and taking things out of the way while working.
 *
 * The tree is the core's (`overview::outline`), and what a row does is `hidden.ts`'s: an eye that
 * hides or shows it, a click that selects, a hover that lights, a double-click that frames, and a
 * way to its statement in the program.  This is only the DOM.  It is rebuilt when the outline
 * changes — its rows' shape, not their spans, which every splice moves — and children are made
 * when a row is first opened, so a ring of many copies, or the V-twin, stays cheap. */
import type { Row } from '../core/outline.js';
import {
  frameRow, hasEye, hoverRow, isHidden, partlyHidden, rowKey, rowSelected, selectRow,
  selectedKeys, selectionRow, toggleRow, walk,
} from './hidden.js';
import { bindPaneToggle } from './project-explorer.js';
import type { SketchView } from './view.js';

/** A row's shape, which decides whether the tree is rebuilt: everything but where it is written. */
function shapeOf(rows: readonly Row[]): string {
  const bare = (r: Row): unknown => [r.label, r.kind, r.path, r.ents, r.solid ?? -1, !!r.operand,
    r.children.map(bare)];
  return JSON.stringify(rows.map(bare));
}

/** The eye a row shows: open, shut (hidden, or under a hidden row), or half (some row under it
 *  hidden) — drawn, since no glyph reads the same in every face. */
type Eye = 'open' | 'shut' | 'half';

const SVG = 'http://www.w3.org/2000/svg';

/** An eye in the tree's ink: its lids, a pupil — filled open, half filled half — and a stroke
 *  through it shut. */
function eyeIcon(state: Eye): SVGSVGElement {
  const svg = document.createElementNS(SVG, 'svg');
  svg.setAttribute('viewBox', '0 0 24 24');
  svg.setAttribute('aria-hidden', 'true');
  const path = (d: string, cls: string): void => {
    const p = document.createElementNS(SVG, 'path');
    p.setAttribute('d', d);
    p.setAttribute('class', cls);
    svg.append(p);
  };
  const lid = 'M2 12s3.6-7 10-7 10 7 10 7-3.6 7-10 7S2 12 2 12z';
  if (state === 'half') {
    // the left half open, the right half faint: some of what is under it is hidden
    path(lid, 'lid faint');
    path('M12 5C5.6 5 2 12 2 12s3.6 7 10 7', 'lid');
    path('M12 9a3 3 0 0 0 0 6z', 'pupil');
  } else {
    path(lid, 'lid');
  }
  if (state === 'open') path('M12 9a3 3 0 1 0 0 6 3 3 0 0 0 0-6z', 'pupil');
  if (state === 'shut') path('M4 20 20 4', 'lid');
  return svg;
}

/** One row as drawn: its element, its children's container (made when first opened), and the
 *  rows above it. */
interface Drawn {
  item: HTMLElement;
  row: HTMLElement;
  eye: HTMLButtonElement | null;
  group: HTMLElement | null;
  above: Row[];
}

export class OutlinePane {
  private shape = '';
  /** The outline as last read, so a refresh with nothing new to read only repaints. */
  private rows: Row[] | null = null;
  /** The rows as last read, in tree order — a drawn row's `data-i` indexes it, so an edit that
   *  moved only spans rebinds them without touching the DOM — and each row's place in it. */
  private flat: Row[] = [];
  private index = new Map<Row, number>();
  /** What the tree was last opened to show (`reveal`), so a band sweeping the drawing does not
   *  open it again on every move. */
  private revealed: Row | null = null;
  private readonly drawn = new Map<number, Drawn>();
  /** The rows open, by key, kept across a rebuild. */
  private readonly open = new Set<string>();

  constructor(private readonly pane: HTMLElement, private readonly tree: HTMLElement,
    private readonly view: SketchView, private readonly toSource: (row: Row) => void) {
    bindPaneToggle(pane, tree, pane.querySelector<HTMLButtonElement>('#outline-toggle')!,
      'the model outline', () => this.refresh());
    pane.querySelector('#outline-isolate')?.addEventListener('click', () => view.isolate());
    pane.querySelector('#outline-show-all')?.addEventListener('click', () => view.showAll());
    // **I isolates and ⇧I shows all** while the tree has the keys: the plain letter is
    // Coincident's everywhere else
    tree.addEventListener('keydown', (e) => {
      if (e.metaKey || e.ctrlKey || e.altKey || e.key.toLowerCase() !== 'i') return;
      e.preventDefault();
      e.stopPropagation();
      if (e.shiftKey) view.showAll();
      else view.isolate();
    });
  }

  /** Bring the tree up to the document: rebuilt where the outline's shape changed, its rows
   *  rebound where only their spans moved, and its eyes and marks repainted either way. */
  refresh(): void {
    if (this.tree.hidden) return;
    let rows: Row[];
    try {
      rows = this.view.outline();
    } catch {
      rows = [];
    }
    if (rows !== this.rows) {
      this.rows = rows;
      this.revealed = null;
      this.flat = [...walk(rows)].map(([r]) => r);
      this.index = new Map(this.flat.map((r, i) => [r, i]));
      const shape = shapeOf(rows);
      if (shape !== this.shape) {
        this.shape = shape;
        this.rebuild(rows);
      }
    }
    this.paint();
  }

  /** The eyes, the dimming under a hidden row and the rows marked current, on every row drawn. */
  paint(): void {
    const selected = selectedKeys(this.view);
    for (const [i, d] of this.drawn) {
      const row = this.flat[i];
      if (!row) continue;
      const own = this.view.hiddenRows.has(rowKey(row));
      const under = !own && isHidden(this.view, row, d.above);
      const half = !own && !under && partlyHidden(this.view, row);
      d.row.classList.toggle('hidden', own);
      d.row.classList.toggle('dimmed', under);
      if (d.eye) {
        const state: Eye = own || under ? 'shut' : half ? 'half' : 'open';
        if (d.eye.dataset.state !== state) {
          d.eye.dataset.state = state;
          d.eye.replaceChildren(eyeIcon(state));
        }
        d.eye.setAttribute('aria-pressed', String(own || under));
        d.eye.title = own || under ? 'Show' : 'Hide';
      }
      if (rowSelected(row, selected)) d.row.setAttribute('aria-current', 'true');
      else d.row.removeAttribute('aria-current');
    }
  }

  /** **Workspace to tree**: open the tree to the row of what is selected, and bring it on screen. */
  reveal(): void {
    if (this.tree.hidden) return;
    this.refresh();
    const at = selectionRow(this.view);
    if (at?.row === this.revealed) return;
    this.revealed = at?.row ?? null;
    if (!at) return;
    for (const r of at.above) this.expand(this.indexOf(r), false);
    this.paint();
    this.drawn.get(this.indexOf(at.row))?.row.scrollIntoView({ block: 'nearest' });
  }

  private indexOf(row: Row): number {
    return this.index.get(row) ?? -1;
  }

  /** Listen on a row's control for what it does to the row it now stands for. */
  private on(el: HTMLElement, type: string, i: number, fn: (r: Row, e: Event) => void): void {
    el.addEventListener(type, (e) => {
      const r = this.flat[i];
      if (r) fn(r, e);
    });
  }

  private rebuild(rows: readonly Row[]): void {
    this.drawn.clear();
    this.tree.replaceChildren(...rows.map((r) => this.item(r, [])));
  }

  /** One row's element, its children made now if it is open. */
  private item(row: Row, above: Row[]): HTMLElement {
    const i = this.indexOf(row);
    const item = document.createElement('div');
    item.className = 'outline-item';
    item.setAttribute('role', 'treeitem');
    const line = document.createElement('div');
    line.className = `outline-row${row.operand ? ' operand' : ''}`;
    line.dataset.i = String(i);
    line.style.setProperty('--depth', String(above.length));
    const kids = row.children.length > 0;
    const twist = document.createElement('button');
    twist.type = 'button';
    twist.className = 'outline-twist';
    if (kids) {
      twist.append(this.pane.querySelector('.project-disclosure')!.cloneNode(true));
      twist.setAttribute('aria-label', 'Expand');
      twist.addEventListener('click', () => this.toggleOpen(i));
    } else {
      twist.disabled = true;
      twist.tabIndex = -1;
    }
    let eye: HTMLButtonElement | null = null;
    if (hasEye(row, above)) {
      eye = document.createElement('button');
      eye.type = 'button';
      eye.className = 'outline-eye';
      this.on(eye, 'click', i, (r, e) => {
        e.stopPropagation();
        toggleRow(this.view, r);
      });
    }
    const label = document.createElement('button');
    label.type = 'button';
    label.className = `outline-label kind-${row.kind}`;
    label.textContent = row.label;
    label.title = row.path || row.label;
    this.on(label, 'click', i, (r, e) => selectRow(this.view, r, (e as MouseEvent).shiftKey));
    this.on(label, 'dblclick', i, (r) => frameRow(this.view, r));
    label.addEventListener('pointerenter', () => hoverRow(this.view, this.flat[i] ?? null));
    label.addEventListener('pointerleave', () => hoverRow(this.view, null));
    line.append(twist, ...(eye ? [eye] : [document.createElement('span')]), label);
    if (row.span) {
      const go = document.createElement('button');
      go.type = 'button';
      go.className = 'outline-source';
      go.textContent = '↗';
      go.title = 'Go to source';
      go.setAttribute('aria-label', `Go to the source of ${row.label}`);
      this.on(go, 'click', i, (r) => this.toSource(r));
      line.append(go);
    }
    if (eye) line.classList.add('has-eye');
    item.append(line);
    const d: Drawn = { item, row: line, eye, group: null, above };
    this.drawn.set(i, d);
    if (kids) {
      item.setAttribute('aria-expanded', 'false');
      if (this.open.has(rowKey(row))) this.expand(i, false);
    }
    return item;
  }

  private toggleOpen(i: number): void {
    const d = this.drawn.get(i);
    const row = this.flat[i];
    if (!d || !row) return;
    if (d.item.getAttribute('aria-expanded') === 'true') {
      d.item.setAttribute('aria-expanded', 'false');
      if (d.group) d.group.hidden = true;
      this.open.delete(rowKey(row));
      this.revealed = null;          // folded away: the next selection opens it again
    } else {
      this.expand(i);
    }
  }

  /** Open row `i`, making its children the first time — and painting them, unless the caller
   *  opens several and paints once after. */
  private expand(i: number, paint = true): void {
    const d = this.drawn.get(i);
    const row = this.flat[i];
    if (!d || !row || !row.children.length) return;
    this.open.add(rowKey(row));
    d.item.setAttribute('aria-expanded', 'true');
    if (!d.group) {
      d.group = document.createElement('div');
      d.group.className = 'outline-group';
      d.group.setAttribute('role', 'group');
      const above = [...d.above, row];
      d.group.append(...row.children.map((c) => this.item(c, above)));
      d.item.append(d.group);
      if (paint) this.paint();
    }
    d.group.hidden = false;
  }
}
