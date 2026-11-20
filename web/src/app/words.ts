/* **The Words menu** — one button over every word the document can be written with, and every
 * command of the app beside them.
 *
 * The words are the core's (`Document.lexicon`): the language's own, the standard library's, the
 * modules the document links and the document's own components and relation words, each with the
 * slots it takes.  Nothing here knows what a word means; it lists, searches and hands the chosen
 * one to the argument box (`wordbox.ts`).  The commands — the drawing tools, open, export, undo,
 * the views — are the app's, under one branch of their own, since they write no source.
 *
 * A word or a command dragged out of the menu (or starred) is kept on the **palette**, the strip
 * beside the button, in the order it was put there; the palette, the tree's arrangement and what
 * was used lately are this browser's, kept in `localStorage` where it lets us. */
import type { Slot, Word } from '../core/lexicon.js';
import { kindMarkup } from './outline.js';

/** A command of the app: what it is called, the branch it is listed under, its key. */
export interface Command {
  id: string;
  label: string;
  branch: string;
  key?: string;
  title?: string;
  run: () => void;
  /** A setting it turns on and off, ticked as it stands. */
  checked?: () => boolean;
  /** A tool it takes up, pressed while held. */
  pressed?: () => boolean;
}

type Entry = { kind: 'word'; word: Word } | { kind: 'command'; cmd: Command };

/** How the tree is arranged: by the module a word is read from, or by what it is for. */
type Arrangement = 'module' | 'yields';

const STORE = 'gcs.words';

interface Kept {
  palette: string[];
  tree: Arrangement;
  recent: string[];
}

function load(fallback: string[]): Kept {
  try {
    const raw = JSON.parse(localStorage.getItem(STORE) ?? 'null') as Partial<Kept> | null;
    if (raw && Array.isArray(raw.palette)) {
      return {
        palette: raw.palette.filter((x) => typeof x === 'string'),
        tree: raw.tree === 'yields' ? 'yields' : 'module',
        recent: Array.isArray(raw.recent) ? raw.recent.slice(0, 8) : [],
      };
    }
  } catch { /* storage refused or garbled: start as new */ }
  return { palette: fallback, tree: 'module', recent: [] };
}

/* -- glyphs: what a slot takes, at a glance ------------------------------------------------ */

/* The kinds are drawn as the model outline draws them (`outline.ts`); these are the rest: a
 * number's, a place's, a command's and the star that keeps one on the palette. */
const SVG = (body: string) =>
  `<svg class="glyph" viewBox="0 0 16 16" aria-hidden="true">${body}</svg>`;
const GLYPH: Record<string, string> = {
  Length: '<path d="M2 4v8M14 4v8M2.6 8h10.8M5 6L2.8 8 5 10M11 6l2.2 2L11 10"/>',
  Angle: '<path d="M2.5 13h11M2.5 13L10.5 3M9.2 13a6.7 6.7 0 0 0-2.5-5.2"/>',
  place: '<path d="M8 2v4M8 10v4M2 8h4M10 8h4"/>',
  app: '<rect x="2.5" y="3" width="11" height="10" rx="1.6"/><path d="M2.5 6h11"/>',
  pin: '<path d="M8 2.2l1.8 3.7 4 .6-2.9 2.8.7 4L8 11.4l-3.6 1.9.7-4L2.2 6.5l4-.6z"/>',
};
const LETTER: Record<string, string> = {
  Scalar: '#', Int: 'n', Bool: '?', word: 'ab', expr: 'ƒ', vector: '(,)', name: ':=', set: '{}',
};
/** The outline's name for a kind the lexicon names otherwise. */
const OUTLINE: Record<string, string> = { any: 'entity', path: 'curve' };

/** The glyph a kind or a type is shown by. */
export function glyph(of: string): string {
  const kind = kindMarkup(OUTLINE[of] ?? of, 'glyph kind');
  if (kind) return kind;
  if (GLYPH[of]) return SVG(GLYPH[of]);
  const t = LETTER[of] ?? '·';
  return SVG(`<text x="8" y="11.5" text-anchor="middle" font-size="${t.length > 2 ? 6.5 : 9}" `
    + `fill="currentColor" stroke="none">${t}</text>`);
}

/** What a slot is shown as: what it is clicked with, else what is typed. */
export function slotGlyph(s: Slot): string {
  const paths = ['line', 'arc', 'circle', 'spline', 'curve'];
  if (s.kinds.length > 1 && s.kinds.every((k) => paths.includes(k))) return glyph('path');
  return glyph(s.kinds[0] ?? s.type ?? 'expr');
}

/** Which of the three inks a slot is drawn in: clicked on the drawing, a number, a word chosen. */
export function slotCategory(s: Slot): 'pick' | 'value' | 'choice' | 'name' {
  if (s.role === 'name') return 'name';
  if (s.kinds.length) return 'pick';
  if (s.words.length) return 'choice';
  return 'value';
}

/** A kind's or a slot's type said, `point, line or axis`. */
export function said(xs: string[]): string {
  if (xs.length <= 1) return xs[0] ?? '';
  return `${xs.slice(0, -1).join(', ')} or ${xs[xs.length - 1]}`;
}

const ENTITY: Record<string, string> = { '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' };
/** Text as markup reads it. */
export const esc = (s: string) => s.replace(/[&<>"]/g, (c) => ENTITY[c]);

/** A word's statement in small: the word, and a glyph per slot it needs. */
function signature(w: Word): string {
  const need = (s: Slot) => !s.optional && s.role !== 'name';
  const g = (s: Slot) => {
    const takes = said(s.kinds.length ? s.kinds : [s.type ?? 'expr']);
    const title = s.key ? `${s.key}: ${takes}` : takes;
    return `<span class="g ${slotCategory(s)}" title="${esc(title)}">${slotGlyph(s)}</span>`;
  };
  const word = `<b>${esc(w.form === 'value' ? ':=' : bare(w.word))}</b>`;
  const inner = w.slots.filter((s) => need(s) && !['left', 'right', 'operand'].includes(s.role));
  const more = w.slots.some((s) => s.optional && s.role !== 'name');
  const parens = inner.length || more
    ? `(${inner.map(g).join('')}${more ? '<span class="more">…</span>' : ''})` : '';
  const left = w.slots.find((s) => s.role === 'left');
  const right = w.slots.find((s) => s.role === 'right' || s.role === 'operand');
  return `${left ? g(left) : ''}${word}${parens}${right ? g(right) : ''}`;
}

/** A word as it is read in a list: `right_of`, `Prism` — its module said beside it. */
const bare = (word: string) => word.replace(/^std\.\w+\./, '');

/* -- the menu and the palette ------------------------------------------------------------- */

export interface WordsHost {
  /** Every word the document can be written with. */
  words(): Word[];
  /** The words that take what is selected, and so fill their slots from it. */
  fitting(): Word[];
  /** What is selected, said: `l, m`. */
  selection(): string;
  /** Take a word up. */
  use(w: Word): void;
}

export class Words {
  private kept: Kept;
  private node: string | null = null;
  private hi = 0;
  private shown: Entry[] = [];
  private readonly search: HTMLInputElement;
  private readonly tree: HTMLElement;
  private readonly items: HTMLElement;
  private readonly arrange: HTMLElement;
  /** A drag out of the menu or along the strip, while one is going: its ghost. */
  private dragging: HTMLElement | null = null;

  constructor(private readonly button: HTMLButtonElement, private readonly menu: HTMLElement,
              private readonly strip: HTMLElement, private readonly host: WordsHost,
              private readonly commands: Command[], seed: string[]) {
    this.kept = load(seed);
    menu.innerHTML = `<div class="words-head"><input type="search" id="words-search"
      autocomplete="off" spellcheck="false"
      placeholder="Search every word — tangent, face, std.place, export…"
      aria-label="Search every word and command">
      <div class="seg" role="group" aria-label="Arrange by">
      <button type="button" data-tree="module">By module</button>
      <button type="button" data-tree="yields">By what it makes</button></div></div>
      <div class="words-body"><nav class="words-tree" aria-label="Groups"></nav>
      <div class="words-items" role="listbox" aria-label="Words"></div></div>
      <div class="words-foot"><span><kbd>↵</kbd> use</span><span>drag a word to the palette to
      keep it</span><span>☆ keeps it too</span><span><kbd>esc</kbd> close</span></div>`;
    this.search = menu.querySelector('input')!;
    this.tree = menu.querySelector('.words-tree')!;
    this.items = menu.querySelector('.words-items')!;
    this.arrange = menu.querySelector('.seg')!;
    button.addEventListener('click', () => (this.isOpen ? this.close() : this.open()));
    this.search.addEventListener('input', () => { this.hi = 0; this.render(); });
    this.search.addEventListener('keydown', (e) => this.key(e));
    this.arrange.addEventListener('click', (e) => {
      const b = (e.target as HTMLElement).closest('button');
      if (!b) return;
      this.kept.tree = b.dataset.tree === 'yields' ? 'yields' : 'module';
      this.node = null;
      this.save();
      this.render();
      this.search.focus();
    });
    this.tree.addEventListener('click', (e) => {
      const b = (e.target as HTMLElement).closest<HTMLButtonElement>('button[data-node]');
      if (!b) return;
      this.search.value = '';
      this.node = b.dataset.node!;
      this.hi = 0;
      this.render();
      this.search.focus();
    });
    this.items.addEventListener('click', (e) => {
      const row = (e.target as HTMLElement).closest<HTMLElement>('[data-id]');
      if (!row || this.swallow) return;
      if ((e.target as HTMLElement).closest('.keep')) {
        this.keep(row.dataset.id!);
        return;
      }
      this.choose(row.dataset.id!);
    });
    this.items.addEventListener('pointerdown', (e) => this.dragFromMenu(e));
    strip.addEventListener('pointerdown', (e) => this.dragOnStrip(e));
    document.addEventListener('pointerdown', (e) => {
      const t = e.target as Node;
      if (this.isOpen && !menu.contains(t) && !button.contains(t)) this.close();
    });
    this.renderStrip();
  }

  get isOpen(): boolean {
    return !this.menu.hidden;
  }

  /** Open the menu, its search holding `q` — on what fits the selection where something is
   *  selected that a word takes. */
  open(q = ''): void {
    this.menu.hidden = false;
    this.button.setAttribute('aria-expanded', 'true');
    this.search.value = q;
    this.hi = 0;
    if (!q && this.fitting().length) this.node = '@fits';
    this.render();
    this.search.focus();
  }

  close(): boolean {
    if (!this.isOpen) return false;
    this.menu.hidden = true;
    this.button.setAttribute('aria-expanded', 'false');
    this.search.blur();
    return true;
  }

  /** The palette's tools pressed as the view holds them, and its settings ticked. */
  refresh(): void {
    for (const b of this.strip.querySelectorAll<HTMLElement>('[data-id]')) {
      const c = this.command(b.dataset.id!);
      if (c?.pressed) b.setAttribute('aria-pressed', String(c.pressed()));
    }
    const n = this.fitting().length;
    this.button.querySelector('.fits')!.textContent = n ? `${n} fit` : '';
  }

  /* -- what there is -------------------------------------------------------------------- */

  private entries(): Entry[] {
    return [
      ...this.host.words().map((word): Entry => ({ kind: 'word', word })),
      ...this.commands.map((cmd): Entry => ({ kind: 'command', cmd })),
    ];
  }

  private idOf(e: Entry): string {
    return e.kind === 'word' ? `w:${e.word.id}` : `c:${e.cmd.id}`;
  }

  private find(id: string): Entry | null {
    if (id.startsWith('c:')) {
      const cmd = this.command(id);
      return cmd ? { kind: 'command', cmd } : null;
    }
    const word = this.host.words().find((w) => `w:${w.id}` === id);
    return word ? { kind: 'word', word } : null;
  }

  private command(id: string): Command | undefined {
    return this.commands.find((c) => `c:${c.id}` === id);
  }

  private fitting(): Entry[] {
    return this.host.fitting().map((word): Entry => ({ kind: 'word', word }));
  }

  /** Where an entry stands in the tree: its branch, and the twig under it. */
  private place(e: Entry): [string, string] {
    if (e.kind === 'command') return ['App', e.cmd.branch];
    const w = e.word;
    const mod = w.module === '' ? 'language' : w.module === '.' ? 'This document' : w.module;
    if (this.kept.tree === 'yields') return [w.module === '.' ? mod : w.group, mod];
    if (w.module === 'std') return ['std', 'datums'];
    if (w.module.startsWith('std.')) return ['std', w.module.slice(4)];
    return [mod, w.group];
  }

  /** What an entry is called in a list, and its glyph. */
  private labelOf(e: Entry): string {
    return e.kind === 'word' ? bare(e.word.word) : e.cmd.label;
  }

  private glyphOf(e: Entry): string {
    if (e.kind === 'command') return glyph('app');
    const w = e.word;
    return glyph(w.makes ?? w.slots.find((s) => s.kinds.length)?.kinds[0] ?? 'expr');
  }

  /* -- drawing it ----------------------------------------------------------------------- */

  private render(): void {
    for (const b of this.arrange.querySelectorAll('button')) {
      b.setAttribute('aria-pressed', String(b.dataset.tree === this.kept.tree));
    }
    const all = this.entries();
    const branches = new Map<string, Map<string, number>>();
    for (const e of all) {
      const [top, twig] = this.place(e);
      const m = branches.get(top) ?? new Map<string, number>();
      m.set(twig, (m.get(twig) ?? 0) + 1);
      branches.set(top, m);
    }
    const order = [...branches.keys()];
    if (this.kept.tree === 'yields') {
      const rank = ['Geometry', 'Relations', 'Dimensions', 'Placement', 'Faces & solids', 'Sets',
        'Rounds', 'Motion', 'Stations', 'Parts', 'Values', 'This document', 'App'];
      order.sort((a, b) => (rank.indexOf(a) + 1 || 99) - (rank.indexOf(b) + 1 || 99));
    } else {
      order.sort((a, b) => rankModule(a) - rankModule(b) || a.localeCompare(b));
    }
    const fits = this.fitting();
    if (this.node === '@fits' && !fits.length) this.node = null;
    if (!this.node || (!this.node.startsWith('@') && !branches.has(this.node.split('/')[0]))) {
      this.node = order[0];
    }
    const q = this.search.value.trim().toLowerCase();
    const n = (node: string, label: string, count: number, cls = '') => {
      const on = !q && this.node === node;
      return `<button type="button" class="${cls}${on ? ' on' : ''}" data-node="${esc(node)}"`
        + ` aria-current="${on}"><span>${esc(label)}</span>`
        + `<span class="n">${count}</span></button>`;
    };
    let t = '';
    if (fits.length) t += n('@fits', 'Fits selection', fits.length, 'fits');
    if (this.kept.recent.length) t += n('@recent', 'Recent', this.kept.recent.length);
    t += n('@all', 'Everything', all.length) + '<div class="rule"></div>';
    for (const top of order) {
      const twigs = branches.get(top)!;
      t += n(top, top, [...twigs.values()].reduce((a, b) => a + b, 0));
      if (!q && this.node.split('/')[0] === top && twigs.size > 1) {
        for (const [twig, c] of twigs) t += n(`${top}/${twig}`, twig, c, 'twig');
      }
    }
    this.tree.innerHTML = t;

    let html = '';
    let list: Entry[];
    const crumb = (e: Entry) => (e.kind === 'command' ? 'app' : e.word.module || 'language');
    if (q) {
      const toks = q.split(/\s+/);
      list = all.map((e): [number, Entry] | null => {
        const name = (e.kind === 'word' ? e.word.word : e.cmd.label).toLowerCase();
        const shown = this.labelOf(e).toLowerCase();
        const hay = e.kind === 'word'
          ? `${e.word.id} ${e.word.module} ${e.word.group} ${e.word.doc} ${e.word.slots
            .map((s) => `${s.key} ${s.kinds.join(' ')} ${s.type ?? ''}`).join(' ')}`.toLowerCase()
          : `${e.cmd.label} ${e.cmd.branch} ${e.cmd.title ?? ''}`.toLowerCase();
        if (!toks.every((k) => hay.includes(k) || name.includes(k))) return null;
        const score = shown === q ? 0 : shown.startsWith(q) ? 1 : name.includes(q) ? 2 : 3;
        return [score, e];
      }).filter((x): x is [number, Entry] => !!x)
        .sort((a, b) => a[0] - b[0]).map((x) => x[1]);
      html = list.length
        ? `<div class="head">${list.length} for “${esc(q)}”</div>`
          + list.map((e, i) => this.row(e, crumb(e), i)).join('')
        : `<p class="empty">Nothing matches “${esc(q)}”. Words are found by name, module, what
           they take and what they do.</p>`;
    } else {
      const [top, twig] = this.node.split('/');
      if (this.node === '@fits') list = fits;
      else if (this.node === '@recent') {
        list = this.kept.recent.map((id) => this.find(id)).filter((e): e is Entry => !!e);
      }
      else if (this.node === '@all') list = all;
      else {
        list = all.filter((e) => {
          const [a, b] = this.place(e);
          return a === top && (!twig || b === twig);
        });
      }
      const twigs = branches.get(top);
      if (!this.node.startsWith('@') && !twig && twigs && twigs.size > 1) {
        const grouped: Entry[] = [];
        for (const tw of twigs.keys()) {
          html += `<div class="head">${esc(top)} ▸ ${esc(tw)}</div>`;
          for (const e of list.filter((x) => this.place(x)[1] === tw)) {
            html += this.row(e, '', grouped.length);
            grouped.push(e);
          }
        }
        list = grouped;
      } else {
        const title = this.node === '@fits' ? `Words that take ${this.host.selection()}`
          : this.node === '@recent' ? 'Used lately'
          : this.node === '@all' ? 'Every word and command'
          : this.node.replace('/', ' ▸ ');
        html = `<div class="head">${esc(title)}</div>${list.map((e, i) =>
          this.row(e, this.node!.startsWith('@') ? crumb(e) : '', i)).join('')}`;
      }
    }
    this.shown = list;
    this.items.innerHTML = html;
    this.items.querySelector('.hi')?.scrollIntoView({ block: 'nearest' });
  }

  private row(e: Entry, crumb: string, i: number): string {
    const id = this.idOf(e);
    const kept = this.kept.palette.includes(id);
    const sig = e.kind === 'word' ? signature(e.word)
      : `<span class="g">${glyph('app')}</span><span>${esc(e.cmd.label)}</span>`
        + (e.cmd.key ? `<kbd>${esc(e.cmd.key.toUpperCase())}</kbd>` : '')
        + (e.cmd.checked?.() ? '<span class="tick">✓</span>' : '');
    const doc = e.kind === 'word' ? e.word.doc : e.cmd.title ?? '';
    const cls = `row${i === this.hi ? ' hi' : ''}${e.kind === 'command' ? ' cmd' : ''}`;
    const name = e.kind === 'word' ? e.word.id : e.cmd.label;
    return `<div class="${cls}" role="option"`
      + ` data-id="${esc(id)}" aria-selected="${i === this.hi}"><span class="sig">${sig}</span>`
      + `<span class="crumb">${esc(crumb)}</span>`
      + `<button type="button" class="keep" aria-pressed="${kept}"`
      + ` title="${kept ? 'On the palette' : 'Keep on the palette'}"`
      + ` aria-label="Keep ${esc(name)} on the palette">${glyph('pin')}</button>`
      + (doc ? `<span class="doc">${esc(doc)}</span>` : '') + '</div>';
  }

  private key(e: KeyboardEvent): void {
    if (e.key === 'Escape') { e.preventDefault(); this.close(); return; }
    if (e.key === 'ArrowDown' || e.key === 'ArrowUp') {
      e.preventDefault();
      const by = e.key === 'ArrowDown' ? 1 : -1;
      this.hi = Math.max(0, Math.min(this.shown.length - 1, this.hi + by));
      for (const [i, r] of [...this.items.querySelectorAll('.row')].entries()) {
        r.classList.toggle('hi', i === this.hi);
        r.setAttribute('aria-selected', String(i === this.hi));
      }
      this.items.querySelector('.hi')?.scrollIntoView({ block: 'nearest' });
      return;
    }
    if (e.key === 'Enter') {
      e.preventDefault();
      const pick = this.shown[this.hi];
      if (pick) this.choose(this.idOf(pick));
    }
  }

  /** Use an entry: a command runs, a word is taken up. */
  choose(id: string): void {
    const e = this.find(id);
    if (!e) return;
    this.close();
    this.kept.recent = [id, ...this.kept.recent.filter((x) => x !== id)].slice(0, 8);
    this.save();
    if (e.kind === 'command') e.cmd.run();
    else this.host.use(e.word);
    this.refresh();
  }

  /* -- the palette ---------------------------------------------------------------------- */

  /** Keep an entry on the palette, at `at` (its end if not said), or take it off where it is. */
  keep(id: string, at?: number): void {
    const p = this.kept.palette;
    const was = p.indexOf(id);
    if (was >= 0 && at === undefined) {
      p.splice(was, 1);
    } else {
      if (was >= 0) p.splice(was, 1);
      p.splice(Math.min(at ?? p.length, p.length), 0, id);
    }
    this.save();
    this.renderStrip();
    if (this.isOpen) this.render();
  }

  private renderStrip(): void {
    this.strip.replaceChildren(...this.kept.palette.map((id) => {
      const e = this.find(id);
      const b = document.createElement('button');
      b.type = 'button';
      b.className = 'chip';
      b.dataset.id = id;
      if (!e) {
        // a word the document no longer has, kept for when it has it again
        b.classList.add('gone');
        b.innerHTML = `${glyph('any')}<span>${esc(id.slice(2))}</span>`
          + '<i class="x" title="Take off the palette">×</i>';
        return b;
      }
      if (e.kind === 'command') b.classList.add('cmd');
      b.title = e.kind === 'word' ? `${e.word.id} — ${e.word.doc}` : [
        `${e.cmd.label}${e.cmd.key ? ` (${e.cmd.key.toUpperCase()})` : ''}`, e.cmd.title,
      ].filter(Boolean).join(' — ');
      b.innerHTML = `${this.glyphOf(e)}<span>${esc(this.labelOf(e))}</span>`
        + '<i class="x" title="Take off the palette">×</i>';
      return b;
    }));
    this.refresh();
  }

  private save(): void {
    try { localStorage.setItem(STORE, JSON.stringify(this.kept)); } catch { /* not kept: fine */ }
  }

  /** Where along the strip a drop at `x` lands. */
  private slotAt(x: number): number {
    const chips = [...this.strip.querySelectorAll<HTMLElement>('.chip:not(.lifted)')];
    const i = chips.findIndex((c) => {
      const r = c.getBoundingClientRect();
      return x < r.left + r.width / 2;
    });
    return i < 0 ? chips.length : i;
  }

  private overStrip(x: number, y: number): boolean {
    const r = this.strip.getBoundingClientRect();
    return x >= r.left - 24 && x <= r.right + 120 && y >= r.top - 24 && y <= r.bottom + 24;
  }

  private swallow = false;

  /** A row of the menu carried out to the palette: a press that moves picks it up. */
  private dragFromMenu(e: PointerEvent): void {
    const row = (e.target as HTMLElement).closest<HTMLElement>('.row[data-id]');
    if (!row || e.button !== 0 || e.pointerType === 'touch') return;
    if ((e.target as HTMLElement).closest('.keep')) return;
    this.carry(e, row.dataset.id!, row, null);
  }

  /** A chip pressed: a click uses it, its × takes it off, a drag moves it along the strip. */
  private dragOnStrip(e: PointerEvent): void {
    const chip = (e.target as HTMLElement).closest<HTMLElement>('.chip[data-id]');
    if (!chip || e.button !== 0) return;
    const id = chip.dataset.id!;
    if ((e.target as HTMLElement).closest('.x')) {
      e.preventDefault();
      this.keep(id);
      return;
    }
    this.carry(e, id, chip, this.kept.palette.indexOf(id), () => this.choose(id));
  }

  private carry(e: PointerEvent, id: string, from: HTMLElement, at: number | null,
                click?: () => void): void {
    const [x0, y0] = [e.clientX, e.clientY];
    from.setPointerCapture(e.pointerId);
    const move = (ev: PointerEvent) => {
      if (!this.dragging) {
        if (Math.hypot(ev.clientX - x0, ev.clientY - y0) < 6) return;
        const ghost = document.createElement('div');
        ghost.className = 'chip ghost';
        const entry = this.find(id);
        ghost.innerHTML = !entry ? esc(id)
          : `${this.glyphOf(entry)}<span>${esc(this.labelOf(entry))}</span>`;
        document.body.append(ghost);
        this.dragging = ghost;
        this.menu.classList.add('carrying');
        if (at !== null) from.classList.add('lifted');
      }
      this.dragging.style.left = `${ev.clientX}px`;
      this.dragging.style.top = `${ev.clientY}px`;
      this.strip.classList.toggle('drop', this.overStrip(ev.clientX, ev.clientY));
    };
    const up = (ev: PointerEvent) => {
      from.removeEventListener('pointermove', move);
      from.removeEventListener('pointerup', up);
      from.removeEventListener('pointercancel', up);
      const ghost = this.dragging;
      if (!ghost) { if (ev.type === 'pointerup') click?.(); return; }
      this.dragging = null;
      ghost.remove();
      from.classList.remove('lifted');
      this.menu.classList.remove('carrying');
      this.strip.classList.remove('drop');
      this.swallow = true;
      setTimeout(() => { this.swallow = false; }, 0);
      if (ev.type !== 'pointerup') return;
      if (this.overStrip(ev.clientX, ev.clientY)) this.keep(id, this.slotAt(ev.clientX));
      else if (at !== null) this.keep(id);           // dragged off the strip: taken off it
    };
    from.addEventListener('pointermove', move);
    from.addEventListener('pointerup', up);
    from.addEventListener('pointercancel', up);
  }
}

/** The language first, the library next, a document's own words, other modules, the app last. */
function rankModule(top: string): number {
  return top === 'language' ? 0 : top === 'std' ? 1 : top === 'This document' ? 2
    : top === 'App' ? 9 : 5;
}
