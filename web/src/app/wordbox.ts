/* **The argument box**: a word in hand, written as the statement it will make, with a chip for
 * each slot in the order the statement writes them — `[p] distance([80mm], side: […]) [q]`.
 *
 * A chip says what it takes by its glyph and its ink, and in its placeholder — and is
 * *armed* by a click on it, by Tab, or by the box itself as each slot is filled.  While a slot that
 * is filled by a click is armed, the workspace takes only what it would take: the `word` tool is
 * down, `SketchView.wordPick` answers which hits are wanted, everything else is dimmed, and a click
 * on what it takes fills it (a click on nothing places a new point, for a slot that makes one).  The
 * program's names fill it as well, typed into the chip.
 *
 * What the words take is the core's (`Document.lexicon`), and so is the statement written:
 * `Document.writeWord` composes it, brings in the module it needs and holds it to the elaborator,
 * whose refusal is said in the status line with the box left as it was. */
import type { Slot, Word, WordRequest } from '../core/lexicon.js';
import { Arc, Circle, Line, angleBetween, distanceBetween, toWire } from '../core/model.js';
import type { Primitive } from '../core/model.js';
import { snapOrNew } from './tools.js';
import type { Hit, SketchView } from './view.js';
import { said, slotCategory, slotGlyph } from './words.js';

/** What a hit is called in the program, and the kind it is as the lexicon names kinds. */
export function hitNamed(v: SketchView, h: Hit): { name: string; kind: string } | null {
  if (h.kind === 'entity') {
    const name = v.doc.nameOf(h.ent);
    return name ? { name, kind: toWire(h.ent.kind) } : null;
  }
  if (h.pick.edge) return null;
  return h.pick.face ? { name: h.pick.face, kind: 'face' } : { name: h.pick.name, kind: 'solid' };
}

/** Whether a slot that takes `kinds` takes a thing of kind `k`. */
function takesKind(kinds: readonly string[], k: string): boolean {
  return kinds.includes('any') || kinds.includes(k) || (k === 'plane' && kinds.includes('frame'));
}

/** What is selected, as hits: the drawing's entities, then the objects' parts. */
function selectedHits(v: SketchView): Hit[] {
  return [
    ...v.selected.map((ent): Hit => ({ kind: 'entity', ent })),
    ...v.selectedSolids.map((pick): Hit => ({ kind: 'solid', pick })),
  ];
}

/** Each selected thing a slot of `w` it fills, in order — every one of them, or `null`. */
function fillFrom(v: SketchView, w: Word): [number, string, string][] | null {
  const hits = selectedHits(v);
  if (!hits.length) return null;
  const out: [number, string, string][] = [];
  for (const h of hits) {
    const named = hitNamed(v, h);
    if (!named) return null;
    const i = w.slots.findIndex((s, k) => s.kinds.length && !s.optional
      && (s.many || !out.some(([j]) => j === k)) && takesKind(s.kinds, named.kind));
    if (i < 0) return null;
    out.push([i, named.name, named.kind]);
  }
  return out;
}

/** Whether `w` takes every selected thing, each in a slot of its own. */
export function fitsSelection(v: SketchView, w: Word): boolean {
  return fillFrom(v, w) !== null;
}

const esc = (s: string) => s.replace(/[&<>"]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c]!));

/** A number as a dimension writes it: six significant digits, no trailing zeros. */
const num = (x: number) => String(Number(x.toPrecision(6)));

export class WordBox {
  word: Word | null = null;
  /** Each slot's text, by index: a name, a number, a word, or names joined by commas. */
  private vals: string[] = [];
  /** The kind of what fills each pick slot, where a click filled it — what a pair narrows by. */
  private kinds: (string | null)[] = [];
  private armed = -1;
  /** Whether the optional slots are shown, rather than folded into `+n`. */
  private open = false;
  /** Slots a number was put in for, from the drawing, and not yet looked at. */
  private measured = new Set<number>();
  /** Why the core would not write it, said under the statement until the next edit. */
  private refused = '';

  constructor(private readonly el: HTMLElement, private readonly v: SketchView) {
    el.addEventListener('pointerdown', (e) => {
      const t = e.target as HTMLElement;
      const chip = t.closest<HTMLElement>('.slot[data-i]');
      if (chip && !t.closest('input, select')) {
        e.preventDefault();
        this.arm(Number(chip.dataset.i));
      }
    });
    el.addEventListener('click', (e) => {
      const t = e.target as HTMLElement;
      if (t.closest('.write')) this.write();
      else if (t.closest('.discard')) this.cancel();
      else if (t.closest('.more')) { this.open = true; this.render(); }
    });
    el.addEventListener('focusin', (e) => {
      const chip = (e.target as HTMLElement).closest<HTMLElement>('.slot[data-i]');
      const i = chip ? Number(chip.dataset.i) : -1;
      if (i >= 0 && i !== this.armed) this.arm(i, false);
    });
    el.addEventListener('input', (e) => {
      const f = e.target as HTMLInputElement | HTMLSelectElement;
      const chip = f.closest<HTMLElement>('.slot[data-i]');
      if (!chip || !this.word) return;
      const i = Number(chip.dataset.i);
      this.vals[i] = f.value;
      this.kinds[i] = null;
      this.measured.delete(i);
      this.unrefuse();
      chip.classList.toggle('filled', !!f.value.trim());
      if (f instanceof HTMLInputElement) f.size = Math.max(4, f.value.length + 1);
      this.say();
    });
    el.addEventListener('change', (e) => {
      if ((e.target as HTMLElement).tagName === 'SELECT') this.advance();
    });
    el.addEventListener('keydown', (e) => this.key(e));
  }

  get active(): boolean {
    return this.word !== null;
  }

  /** **Take a word up**: its slots filled from what is selected, where the selection fits, and the
   *  first slot still empty armed. */
  start(w: Word): void {
    this.word = w;
    this.vals = w.slots.map((s) => s.default ?? '');
    this.kinds = w.slots.map(() => null);
    this.measured.clear();
    this.open = false;
    this.refused = '';
    for (const [i, name, kind] of fillFrom(this.v, w) ?? []) {
      this.vals[i] = this.vals[i] && w.slots[i].many ? `${this.vals[i]}, ${name}` : name;
      this.kinds[i] = kind;
    }
    this.el.hidden = false;
    this.advance(true);
  }

  /** Put the word down, writing nothing. */
  cancel(): void {
    if (!this.word) return;
    this.word = null;
    this.armed = -1;
    this.el.hidden = true;
    this.el.replaceChildren();
    this.v.wordPick = null;
    if (this.v.tool === 'word') this.v.setTool('select');
    this.v.onStatus('');
  }

  /* -- the slots -------------------------------------------------------------------------- */

  private filled(i: number): boolean {
    return !!this.vals[i]?.trim();
  }

  /** The selectors the operands chosen settle the word by only when given (`tangent` between
   *  two lines, `at:`), which those slots need filled. */
  private needs(): string[] {
    const w = this.word!;
    const [l, r] = [w.slots.findIndex((s) => s.role === 'left'), w.slots.findIndex((s) => s.role === 'right')];
    const [a, b] = [this.kinds[l], this.kinds[r]];
    if (!a || !b) return [];
    return w.pairs.find(([x, y]) => x === a && y === b)?.[2] ?? [];
  }

  private required(i: number): boolean {
    const s = this.word!.slots[i];
    return (!s.optional || this.needs().includes(s.key)) && s.role !== 'name' && s.default === null;
  }

  /** Something changed since the core refused it: the refusal is about what was. */
  private unrefuse(): void {
    if (!this.refused) return;
    this.refused = '';
    this.el.querySelector('.refused')?.remove();
  }

  private complete(): boolean {
    return this.word!.slots.every((_, i) => !this.required(i) || this.filled(i));
  }

  /** What a pick slot takes now: its kinds, narrowed by the other operand where a built-in
   *  word's pairs say which go together. */
  private takes(i: number): string[] {
    const w = this.word!;
    const s = w.slots[i];
    if (!w.pairs.length || (s.role !== 'left' && s.role !== 'right')) return s.kinds;
    const other = w.slots.findIndex((x) => x.role === (s.role === 'left' ? 'right' : 'left'));
    const k = this.kinds[other];
    if (!k) return s.kinds;
    const side = s.role === 'left' ? 0 : 1;
    return [...new Set(w.pairs.filter((p) => p[1 - side] === k).map((p) => p[side]))];
  }

  /** Arm the slot `i`: its chip lit and focused, and the workspace taking what it takes. */
  arm(i: number, focus = true): void {
    const w = this.word;
    if (!w || i < 0 || i >= w.slots.length) return;
    this.armed = i;
    this.unrefuse();
    if (w.slots[i].optional) this.open = true;
    const s = w.slots[i];
    const takes = this.takes(i);
    if (takes.length || s.mints) {
      this.v.wordPick = {
        mints: s.mints,
        accepts: (h) => {
          const n = hitNamed(this.v, h);
          return !!n && takesKind(takes, n.kind) && !this.uses(n.name, i);
        },
        take: (h, sp) => this.take(i, h, sp),
      };
      if (this.v.tool !== 'word') this.v.setTool('word');
    } else {
      this.v.wordPick = null;
      if (this.v.tool === 'word') this.v.setTool('select');
    }
    this.render(focus);
    this.v.draw();
  }

  /** Whether `name` already fills a slot other than `i` — one thing is not both operands. */
  private uses(name: string, i: number): boolean {
    return this.vals.some((v, k) => k !== i && this.word!.slots[k].kinds.length
      && v.split(',').map((x) => x.trim()).includes(name));
  }

  /** A click on the workspace for slot `i`: the thing hit, or a point placed where there was
   *  nothing. */
  private take(i: number, h: Hit | null, sp: [number, number]): void {
    const w = this.word;
    if (!w) return;
    let named = h ? hitNamed(this.v, h) : null;
    if (!named) {
      const made = this.mint(sp);
      if (!made) return;
      named = { name: made, kind: 'point' };
      // a point is the whole of what the `point` word makes
      if (w.id === 'point') {
        this.v.onStatus(`point ${made}`);
        this.start(w);
        return;
      }
    }
    const s = w.slots[i];
    this.vals[i] = s.many && this.filled(i) ? `${this.vals[i]}, ${named.name}` : named.name;
    this.kinds[i] = named.kind;
    this.unrefuse();
    if (s.many) {
      this.render(true);
      this.v.draw();
      return;
    }
    this.advance();
  }

  /** A new point where the click was, in space where the eye sees it, its name the one the
   *  document gives it. */
  private mint(sp: [number, number]): string | null {
    const v = this.v;
    v.pushUndo();
    const p = snapOrNew(v, sp);
    v.releasePlan();
    v.afterEdit();
    return v.doc.nameOf(p) ?? null;
  }

  /** On to the next slot that wants filling — a pick slot first, then a number — or, with all
   *  filled, the box ready to write. */
  private advance(first = false): void {
    const w = this.word!;
    const empty = (i: number) => this.required(i) && !this.filled(i);
    let i = w.slots.findIndex((s, k) => empty(k) && s.kinds.length > 0);
    if (i < 0) {
      this.measure();
      i = w.slots.findIndex((_, k) => empty(k) || this.measured.has(k));
    }
    if (i >= 0) {
      this.measured.delete(i);
      this.arm(i, !first || w.slots[i].kinds.length === 0);
      return;
    }
    this.armed = -1;
    this.v.wordPick = null;
    if (this.v.tool === 'word') this.v.setTool('select');
    this.render(true);
    this.v.draw();
  }

  /** A dimension's number put in from the drawing as it stands, once its operands are known —
   *  a length, a radius, an angle — selected in its chip so typing replaces it. */
  private measure(): void {
    const w = this.word!;
    const vi = w.slots.findIndex((s) => s.role === 'value');
    if (vi < 0 || this.filled(vi) || !['distance', 'radius', 'length', 'angle'].includes(w.id)) return;
    const ent = (role: string): Primitive | null => {
      const i = w.slots.findIndex((s) => s.role === role);
      return i < 0 ? null : this.v.doc.entity(this.vals[i]?.trim() ?? '') ?? null;
    };
    let x: number | null = null;
    let unit = '';
    if (w.id === 'distance') {
      const [a, b] = [ent('left'), ent('right')];
      if (a && b && !(a instanceof Circle || a instanceof Arc)) x = Math.abs(distanceBetween(a, b));
    } else if (w.id === 'radius') {
      const c = ent('operand');
      if (c instanceof Circle || c instanceof Arc) x = Math.abs(c.radius.value);
    } else if (w.id === 'length') {
      const l = ent('operand');
      if (l instanceof Line) x = distanceBetween(l.p1, l.p2);
    } else {
      const [a, b] = [ent('left'), ent('right')];
      if (a instanceof Line && b instanceof Line) {
        x = ((angleBetween(a, b) * 180 / Math.PI) % 360 + 360) % 360;
        unit = 'deg';
      }
    }
    if (x === null || !Number.isFinite(x)) return;
    this.vals[vi] = `${num(x)}${unit}`;
    this.measured.add(vi);
  }

  /* -- writing it ------------------------------------------------------------------------- */

  private request(): WordRequest {
    const w = this.word!;
    const args: Record<string, string> = {};
    w.slots.forEach((s, i) => { if (this.filled(i)) args[s.key] = this.vals[i].trim(); });
    // drawn in space, as every tool draws: a word declares no plane
    return { id: w.id, args };
  }

  /** **Write the word**: the core composes the statement and holds it to the elaborator.  A line
   *  goes on from where it ended, so a run of clicks draws a chain of them; any other word drawn
   *  is taken up again empty, and a relation is put down. */
  write(): void {
    const w = this.word;
    if (!w) return;
    if (!this.complete()) {
      const i = w.slots.findIndex((_, k) => this.required(k) && !this.filled(k));
      this.arm(i);
      this.refuse(`${w.word} still needs ${this.need(w.slots[i])}`);
      return;
    }
    const r = this.request();
    const said = this.v.doc.wordText(r).text ?? w.word;
    // what the core will not write is said here, beside what it is about, and the box stays
    const e = this.v.doc.writeWord(r);
    if (e.refused) return this.refuse(e.refused);
    if (!this.v.apply(e, `wrote ${said}`)) return;
    if (w.form === 'declare' && w.id !== 'face' && w.id !== 'solid') {
      const end = w.id === 'line' ? r.args.p2 : undefined;
      this.start(w);
      if (end) {
        const p1 = w.slots.findIndex((s) => s.key === 'p1');
        this.vals[p1] = end;
        this.kinds[p1] = 'point';
        this.advance();
      }
      return;
    }
    this.cancel();
    this.v.onStatus(`wrote ${said}`);
  }

  /** What a slot is asked for, in a phrase: `a point, line or plane for p`. */
  private need(s: Slot): string {
    const what = s.kinds.length ? said(this.takes(this.word!.slots.indexOf(s)))
      : s.words.length ? said(s.words) : s.type === 'place' ? 'a place' : s.type ?? 'a value';
    return s.key ? `${what} for ${s.key}` : what;
  }

  private key(e: KeyboardEvent): void {
    if (e.key === 'Escape') {
      e.preventDefault();
      e.stopPropagation();
      this.cancel();
    } else if (e.key === 'Enter') {
      e.preventDefault();
      e.stopPropagation();
      const s = this.word?.slots[this.armed];
      if (this.complete() && (!s?.many || this.filled(this.armed))) this.write();
      else this.advance();
    }
  }

  /** The keys the box takes while a word is in hand and the focus is on the workspace: Enter
   *  writes it (or ends a slot of several), Escape puts it down.  Whether the key was taken. */
  handleKey(e: KeyboardEvent): boolean {
    if (!this.word || (e.key !== 'Enter' && e.key !== 'Escape')) return false;
    this.key(e);
    return true;
  }

  /* -- drawing it ------------------------------------------------------------------------- */

  /** Say why it was not written, under the statement, until something in the box changes. */
  private refuse(why: string): void {
    this.refused = why;
    this.render();
  }

  /** Whether it can be written yet, on its button. */
  private say(): void {
    const ok = this.el.querySelector('.write');
    ok?.classList.toggle('ready', this.complete());
  }

  private chip(i: number): HTMLElement {
    const w = this.word!;
    const s = w.slots[i];
    const cat = slotCategory(s);
    const wrap = document.createElement('span');
    wrap.className = `slotwrap${s.optional && !this.filled(i) && !this.needs().includes(s.key) ? ' dim' : ''}`;
    const chip = document.createElement('span');
    chip.className = `slot ${cat}${this.filled(i) ? ' filled' : ''}${this.armed === i ? ' armed' : ''}`;
    chip.dataset.i = String(i);
    const id = `wordbox-${i}`;
    const val = this.vals[i] ?? '';
    const label = `${s.key || w.word}: ${this.need(s).replace(/ for .*/, '')}${s.optional ? ' (optional)' : ''}`;
    let field: string;
    if (cat === 'choice' && !s.kinds.length) {
      field = `<select id="${id}" aria-label="${esc(label)}"><option value="">${esc(s.key)}</option>`
        + s.words.map((x) => `<option${x === val ? ' selected' : ''}>${esc(x)}</option>`).join('') + '</select>';
    } else {
      const ph = cat === 'name' ? 'name' : s.key || (s.type === 'place' ? 'place' : 'value');
      const list = s.words.length ? ` list="${id}-words"` : '';
      field = `<input id="${id}" value="${esc(val)}" placeholder="${esc(ph)}" size="${Math.max(4, Math.max(val.length, ph.length) + 1)}"`
        + ` autocomplete="off" spellcheck="false" aria-label="${esc(label)}"${list}>`
        + (s.words.length ? `<datalist id="${id}-words">${s.words.map((x) => `<option value="${esc(x)}">`).join('')}</datalist>` : '');
    }
    chip.innerHTML = (cat === 'name' ? '' : slotGlyph(s)) + field;
    wrap.append(chip);
    return wrap;
  }

  private render(focus = false): void {
    const w = this.word;
    if (!w) return;
    const sent = document.createElement('div');
    sent.className = 'sentence';
    const lit = (t: string, cls = '') => {
      const s = document.createElement('span');
      s.className = `lit${cls ? ` ${cls}` : ''}`;
      s.textContent = t;
      sent.append(s);
    };
    const word = (t: string) => lit(t, 'word');
    const at = (role: string) => w.slots.map((s, i) => [s, i] as const).filter(([s]) => s.role === role);
    const shown = (i: number) => this.open || !w.slots[i].optional || this.filled(i) || this.armed === i
      || this.needs().includes(w.slots[i].key);
    const name = at('name')[0];
    const holes = w.slots.map((s, i) => [s, i] as const)
      .filter(([s]) => ['value', 'arg', 'option', 'pin'].includes(s.role));
    const parens = (close = ')') => {
      const inside = holes.filter(([s, i]) => (w.form !== 'param' || s.role !== 'option') && shown(i));
      const hidden = holes.filter(([, i]) => !shown(i)).length;
      if (!inside.length && !hidden) return;
      lit('(');
      inside.forEach(([s, i], k) => {
        if (k) lit(', ');
        if (s.role === 'pin') lit(`${s.key} ==`);
        else if (s.role === 'option' && !(w.form === 'declare' && w.id === 'face' && s.key === 'edges')) lit(`${s.key}:`);
        sent.append(this.chip(i));
      });
      if (hidden) {
        const b = document.createElement('button');
        b.type = 'button';
        b.className = 'more';
        b.textContent = `+${hidden}`;
        b.title = `Options: ${holes.filter(([, i]) => !shown(i)).map(([s]) => s.key).join(', ')}`;
        sent.append(b);
      }
      lit(close);
    };
    const named = () => { if (name) { sent.append(this.chip(name[1])); lit(':='); } };
    switch (w.form) {
      case 'infix':
        for (const [, i] of at('left')) sent.append(this.chip(i));
        word(w.word);
        parens();
        for (const [, i] of at('right')) sent.append(this.chip(i));
        break;
      case 'prefix':
        word(w.word);
        parens();
        for (const [, i] of at('operand')) sent.append(this.chip(i));
        break;
      case 'call':
        word(w.word);
        parens();
        break;
      case 'instance': {
        named();
        const solid = w.makes === 'solid';
        if (solid) word('solid(');
        word(w.word);
        parens(solid ? '))' : ')');
        break;
      }
      case 'param': {
        word('param');
        if (name) sent.append(this.chip(name[1]));
        lit(':');
        for (const [, i] of at('option')) sent.append(this.chip(i));
        lit(':=');
        for (const [, i] of at('value')) sent.append(this.chip(i));
        break;
      }
      case 'value':
        named();
        for (const [, i] of at('value')) sent.append(this.chip(i));
        break;
      default: {
        named();
        word(w.word);
        parens();
        const seeds = at('seed');
        if (w.id === 'point') break;
        if (seeds.some(([, i]) => shown(i))) {
          word('hint(');
          seeds.filter(([, i]) => shown(i)).forEach(([s, i], k) => {
            if (k) lit(', ');
            if (s.key) lit(`${s.key}:`);
            sent.append(this.chip(i));
          });
          lit(')');
        } else if (seeds.length) {
          const b = document.createElement('button');
          b.type = 'button';
          b.className = 'more';
          b.textContent = `+${seeds.length}`;
          b.title = `Seeds: ${seeds.map(([s]) => s.key).join(', ')}`;
          sent.append(b);
        }
      }
    }
    this.el.innerHTML = `<div class="box-row"><div class="sentence-host"></div>`
      + `<div class="acts"><button type="button" class="write${this.complete() ? ' ready' : ''}"`
      + ` title="Write it into the program (↵)">write <kbd>↵</kbd></button>`
      + `<button type="button" class="discard" aria-label="Put the word down (esc)"`
      + ` title="Put the word down (esc)">×</button></div></div>`
      + (this.refused ? `<div class="refused" role="alert">${esc(this.refused)}</div>` : '');
    this.el.querySelector('.sentence-host')!.replaceWith(sent);
    sent.title = w.doc || w.id;
    this.say();
    if (focus && this.armed >= 0) {
      const f = this.el.querySelector<HTMLInputElement | HTMLSelectElement>(`#wordbox-${this.armed}`);
      if (f) {
        f.focus({ preventScroll: true });
        if (f instanceof HTMLInputElement && this.vals[this.armed]) f.select();
      }
    }
  }
}

