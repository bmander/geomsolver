/* **The argument box**: a word in hand, written as the statement it will make, with a chip for
 * each slot in the order the statement writes them — `[p] distance([80mm], side: […]) [q]`.
 *
 * A chip says what it takes — a glyph and a caption, `point, line or plane`, `Length` — and is
 * *armed* by a click on it, by Tab, or by the box itself as each slot is filled.  The `word` tool
 * is down for as long as a word is in hand; while a slot that is filled by a click is armed,
 * `SketchView.wordPick` answers which hits it takes, the workspace takes only those, everything
 * else is dimmed, and a click on one fills it (a click on nothing places a new point, for a slot
 * that makes one).  The program's names fill it as well, typed into the chip.
 *
 * What the words take is the core's (`Document.lexicon`), and so is the statement written:
 * `Document.writeWord` composes it, brings in the module it needs and holds it to the elaborator,
 * whose refusal is said in the status line with the box left as it was. */
import { fmt } from '../core/io.js';
import type { Slot, Word, WordRequest } from '../core/lexicon.js';
import { Arc, Circle, Line, angleBetween, distanceBetween, toWire } from '../core/model.js';
import type { Primitive } from '../core/model.js';
import { drawableHere, snapOrNew } from './tools.js';
import type { Hit, SketchView } from './view.js';
import { esc, glyph, said, slotCategory, slotGlyph } from './words.js';

/** What a hit is called in the program, and the kind it is as the lexicon names kinds — an edge,
 *  which no word takes, nothing. */
export function hitNamed(v: SketchView, h: Hit): { name: string; kind: string } | null {
  if (h.kind === 'solid') {
    if (h.pick.edge) return null;
    return h.pick.face ? { name: h.pick.face, kind: 'face' } : { name: h.pick.name, kind: 'solid' };
  }
  const name = v.doc.nameOf(h.ent);
  return name ? { name, kind: toWire(h.ent.kind) } : null;
}

/** Whether a slot that takes `kinds` takes a thing of kind `k`. */
function takesKind(kinds: readonly string[], k: string): boolean {
  return kinds.includes('any') || kinds.includes(k);
}

/** What is selected, as hits: the drawing's entities, then the objects' parts. */
export function selectedHits(v: SketchView): Hit[] {
  return [
    ...v.selected.map((ent): Hit => ({ kind: 'entity', ent })),
    ...v.selectedSolids.map((pick): Hit => ({ kind: 'solid', pick })),
  ];
}

/** The selection named, `[name, kind]` each — or `null` where something selected has no name. */
export function selectionNamed(v: SketchView): [string, string][] | null {
  const named = selectedHits(v).map((h) => hitNamed(v, h));
  return named.every((n) => n) ? named.map((n) => [n!.name, n!.kind]) : null;
}

/** Each selected thing a slot of `w` it fills, in order — every one of them, or `null`. */
function fillFrom(w: Word, sel: [string, string][] | null): [number, string, string][] | null {
  if (!sel?.length) return null;
  const out: [number, string, string][] = [];
  for (const [name, kind] of sel) {
    const i = w.slots.findIndex((s, k) => s.kinds.length && !s.optional
      && (s.many || !out.some(([j]) => j === k)) && takesKind(s.kinds, kind));
    if (i < 0) return null;
    out.push([i, name, kind]);
  }
  return out;
}

/** Whether `w` takes every thing of the selection `sel` named, each in a slot of its own. */
export function fitsSelection(sel: [string, string][] | null, w: Word): boolean {
  return fillFrom(w, sel) !== null;
}

/** The dimension words whose number is put in as the drawing measures it, by what they read. */
const MEASURED = ['distance', 'radius', 'length', 'angle'];

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

  /** **Take a word up**: the `word` tool down, the word's slots filled from what is selected
   *  where the selection fits, and the first slot still empty armed. */
  start(w: Word): void {
    this.word = w;
    this.vals = w.slots.map((s) => s.default ?? '');
    this.kinds = w.slots.map(() => null);
    this.measured.clear();
    this.open = false;
    for (const [i, name, kind] of fillFrom(w, selectionNamed(this.v)) ?? []) {
      this.vals[i] = this.vals[i] && w.slots[i].many ? `${this.vals[i]}, ${name}` : name;
      this.kinds[i] = kind;
    }
    this.el.hidden = false;
    if (this.v.tool !== 'word') this.v.setTool('word');
    this.advance(true);
  }

  /** Put the word down, writing nothing: the tool goes with it. */
  cancel(): void {
    if (!this.word) return;
    this.word = null;
    this.armed = -1;
    this.el.hidden = true;
    this.el.replaceChildren();
    if (this.v.tool === 'word') this.v.setTool('select');
  }

  /* -- the slots -------------------------------------------------------------------------- */

  private filled(i: number): boolean {
    return !!this.vals[i]?.trim();
  }

  /** The selectors the operands chosen settle the word by only when given (`tangent` between
   *  two lines, `at:`), which those slots need filled. */
  private needs(): string[] {
    const w = this.word!;
    const at = (role: string) => this.kinds[w.slots.findIndex((s) => s.role === role)];
    const [a, b] = [at('left'), at('right')];
    if (!a || !b) return [];
    return w.pairs.find(([x, y]) => x === a && y === b)?.[2] ?? [];
  }

  private required(i: number): boolean {
    const s = this.word!.slots[i];
    return (!s.optional || this.needs().includes(s.key)) && s.role !== 'name' && s.default === null;
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

  /** Arm the slot `i`: its chip lit and focused, and — for a slot a click fills — the workspace
   *  taking what it takes.  Whether a hit is taken is asked of every entity at every paint, so it
   *  is worked out once per hit and kept while the slot is armed. */
  arm(i: number, focus = true): void {
    const w = this.word;
    if (!w || i < 0 || i >= w.slots.length) return;
    this.armed = i;
    if (w.slots[i].optional) this.open = true;
    const s = w.slots[i];
    const takes = this.takes(i);
    const used = new Set(this.vals.flatMap((val, k) => k !== i && w.slots[k].kinds.length
      ? val.split(',').map((x) => x.trim()) : []));
    const known = new Map<unknown, boolean>();
    this.v.wordPick = takes.length || s.mints ? {
      mints: s.mints,
      accepts: (h) => {
        const key = h.kind === 'entity' ? h.ent
          : `${h.pick.index} ${h.pick.face} ${h.pick.edge ?? ''}`;
        let ok = known.get(key);
        if (ok === undefined) {
          const n = hitNamed(this.v, h);
          ok = !!n && takesKind(takes, n.kind) && !used.has(n.name);
          known.set(key, ok);
        }
        return ok;
      },
      take: (h, sp) => this.take(i, h, sp),
    } : null;
    this.render(focus);
    this.v.draw();
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
      if (w.slots[i].type === 'place') {
        this.v.onStatus(`point ${made}`);
        this.start(w);
        return;
      }
    }
    const s = w.slots[i];
    this.vals[i] = s.many && this.filled(i) ? `${this.vals[i]}, ${named.name}` : named.name;
    this.kinds[i] = named.kind;
    if (s.many) this.arm(i);
    else this.advance();
  }

  /** A new point where the click was, drawn on the plane being drawn on (or in space), its name
   *  the one the document gives it. */
  private mint(sp: [number, number]): string | null {
    const v = this.v;
    if (!drawableHere(v)) return null;
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
    this.measured.delete(i);
    if (i >= 0) {
      this.arm(i, !first || w.slots[i].kinds.length === 0);
      return;
    }
    this.armed = -1;
    this.v.wordPick = null;
    this.render(true);
    this.v.draw();
  }

  /** A dimension's number put in from the drawing as it stands, once its operands are known —
   *  a length, a radius, an angle — selected in its chip so typing replaces it. */
  private measure(): void {
    const w = this.word!;
    const vi = w.slots.findIndex((s) => s.role === 'value');
    if (vi < 0 || this.filled(vi) || !MEASURED.includes(w.id)) return;
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
    this.vals[vi] = `${fmt(x, 6)}${unit}`;
    this.measured.add(vi);
  }

  /* -- writing it ------------------------------------------------------------------------- */

  private request(): WordRequest {
    const w = this.word!;
    const args: Record<string, string> = {};
    w.slots.forEach((s, i) => { if (this.filled(i)) args[s.key] = this.vals[i].trim(); });
    return { id: w.id, args, ...(w.drawn && this.v.planeName ? { plane: this.v.planeName } : {}) };
  }

  /** **Write the word**: the core composes the statement and holds it to the elaborator.  A line
   *  goes on from where it ended, so a run of clicks draws a chain of them; any other word drawn
   *  on a plane is taken up again empty, and anything else is put down. */
  write(): void {
    const w = this.word;
    if (!w) return;
    const r = this.request();
    const said = this.v.doc.wordText(r);
    if (said.refused || !this.complete()) {
      const i = w.slots.findIndex((_, k) => this.required(k) && !this.filled(k));
      this.v.onStatus(said.refused ?? `${w.word} still needs ${this.need(i)}`);
      this.arm(i);
      return;
    }
    if (w.drawn && !this.v.ensurePlane()) return;
    if (!this.v.apply(this.v.doc.writeWord(r), `wrote ${said.text}`)) return;
    if (!w.drawn) {
      this.cancel();
      this.v.onStatus(`wrote ${said.text}`);
      return;
    }
    this.start(w);
    const p1 = w.slots.findIndex((s) => s.key === 'p1');
    if (w.id === 'line' && r.args.p2) {
      this.vals[p1] = r.args.p2;
      this.kinds[p1] = 'point';
      this.advance();
    }
  }

  /** What slot `i` takes, said: `a point, line or plane`, `Length`. */
  private what(i: number): string {
    const s = this.word!.slots[i];
    return s.kinds.length ? said(this.takes(i)) : s.words.length ? said(s.words)
      : s.type === 'place' ? 'a place' : s.type ?? 'a value';
  }

  /** What slot `i` is asked for: `a point, line or plane for p`. */
  private need(i: number): string {
    const key = this.word!.slots[i].key;
    return key ? `${this.what(i)} for ${key}` : this.what(i);
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

  /** The prompt under the sentence: what the armed slot wants, said. */
  private prompt(): string {
    const w = this.word!;
    if (this.armed < 0) {
      return this.complete() ? 'Ready: <kbd>↵</kbd> writes it, <kbd>esc</kbd> puts it down.' : '';
    }
    const s = w.slots[this.armed];
    const key = `<b>${esc(s.key || w.word)}</b>`;
    switch (slotCategory(s)) {
      case 'pick': {
        if (s.type === 'place') return 'Click where the point goes.';
        const many = s.many ? ', one after another; <kbd>↵</kbd> when that is all' : '';
        const mint = s.mints ? ', or empty space for a new point' : '';
        return `Click ${esc(this.what(this.armed))} for ${key}${mint}${many}, or type its name.`;
      }
      case 'choice': return `Choose ${key}.`;
      case 'name': return 'The name it is given — left empty, the next one free.';
      default: {
        const now = this.filled(this.armed) ? ' — shown as the drawing measures it now' : '';
        const what = s.type === 'expr' ? 'a number or a name' : `a ${s.type}`;
        return `Type ${esc(what)} for ${key}${now}.`;
      }
    }
  }

  /** The write button ready and the prompt said, as the slots stand. */
  private say(): void {
    this.el.querySelector('.write')?.classList.toggle('ready', this.complete());
    const p = this.el.querySelector('.prompt');
    if (p) p.innerHTML = this.prompt();
  }

  private chip(i: number): HTMLElement {
    const w = this.word!;
    const s = w.slots[i];
    const cat = slotCategory(s);
    const wrap = document.createElement('span');
    const optional = s.optional && !this.needs().includes(s.key);
    wrap.className = `slotwrap${optional && !this.filled(i) ? ' dim' : ''}`;
    const chip = document.createElement('span');
    chip.className = `slot ${cat}`;
    chip.classList.toggle('filled', this.filled(i));
    chip.classList.toggle('armed', this.armed === i);
    chip.dataset.i = String(i);
    const id = `wordbox-${i}`;
    const val = this.vals[i] ?? '';
    const label = `${s.key || w.word}: ${this.what(i)}${s.optional ? ' (optional)' : ''}`;
    let field: string;
    if (cat === 'choice') {
      const option = (x: string) => `<option${x === val ? ' selected' : ''}>${esc(x)}</option>`;
      field = `<select id="${id}" aria-label="${esc(label)}">`
        + `<option value="">${esc(s.key)}</option>${s.words.map(option).join('')}</select>`;
    } else {
      const ph = cat === 'name' ? 'name' : s.key || (s.type === 'place' ? 'place' : 'value');
      const list = s.words.length ? ` list="${id}-words"` : '';
      field = `<input id="${id}" value="${esc(val)}" placeholder="${esc(ph)}"`
        + ` size="${Math.max(4, Math.max(val.length, ph.length) + 1)}"`
        + ` autocomplete="off" spellcheck="false" aria-label="${esc(label)}"${list}>`
        + (s.words.length ? `<datalist id="${id}-words">`
          + s.words.map((x) => `<option value="${esc(x)}">`).join('') + '</datalist>' : '');
    }
    chip.innerHTML = (cat === 'name' ? '' : slotGlyph(s)) + field;
    const cap = document.createElement('span');
    cap.className = 'cap';
    cap.textContent = cat === 'name' ? 'new name'
      : [s.kinds.length ? short(this.takes(i)) : s.words.length ? short(s.words) : s.type ?? '',
         s.mints && 'or new', s.many && 'several', optional && 'optional']
        .filter(Boolean).join(' · ');
    wrap.append(chip, cap);
    return wrap;
  }

  /** The `+n` that shows what is folded away. */
  private more(keys: string[]): HTMLElement {
    const b = document.createElement('button');
    b.type = 'button';
    b.className = 'more';
    b.textContent = `+${keys.length}`;
    b.title = `Also: ${keys.join(', ')}`;
    return b;
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
    const indexed = w.slots.map((s, i) => [s, i] as const);
    const at = (role: string) => indexed.filter(([s]) => s.role === role);
    const shown = (i: number) => this.open || !w.slots[i].optional || this.filled(i)
      || this.armed === i || this.needs().includes(w.slots[i].key);
    const name = at('name')[0];
    const holes = indexed.filter(([s]) => ['value', 'arg', 'option', 'pin'].includes(s.role));
    const parens = (close = ')') => {
      // a param's type is written before `:=`, not in parentheses
      const typed = (s: Slot) => w.form === 'param' && s.role === 'option';
      const inside = holes.filter(([s, i]) => shown(i) && !typed(s));
      const hidden = holes.filter(([, i]) => !shown(i)).map(([s]) => s.key);
      if (!inside.length && !hidden.length) return;
      lit('(');
      inside.forEach(([s, i], k) => {
        if (k) lit(', ');
        if (s.role === 'pin') lit(`${s.key} ==`);
        else if (s.role === 'option') lit(`${s.key}:`);
        sent.append(this.chip(i));
      });
      if (hidden.length) sent.append(this.more(hidden));
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
      case 'param':
        word('param');
        if (name) sent.append(this.chip(name[1]));
        lit(':');
        for (const [, i] of at('option')) sent.append(this.chip(i));
        lit(':=');
        for (const [, i] of at('value')) sent.append(this.chip(i));
        break;
      case 'value':
        named();
        for (const [, i] of at('value')) sent.append(this.chip(i));
        break;
      default: {
        named();
        word(w.word);
        parens();
        // a point's place is its seed, filled by the click that places it: no clause to show
        const seeds = at('seed').filter(([s]) => s.type !== 'place');
        if (seeds.some(([, i]) => shown(i))) {
          word('hint(');
          seeds.filter(([, i]) => shown(i)).forEach(([s, i], k) => {
            if (k) lit(', ');
            if (s.key) lit(`${s.key}:`);
            sent.append(this.chip(i));
          });
          lit(')');
        } else if (seeds.length) {
          sent.append(this.more(seeds.map(([s]) => s.key)));
        }
      }
    }
    this.el.innerHTML = '<div class="box-row">'
      + `<span class="grip" aria-hidden="true">${glyph('any')}</span>`
      + '<div class="sentence-host"></div><div class="acts"><button type="button" class="write"'
      + ' title="Write it into the program (↵)">write <kbd>↵</kbd></button>'
      + '<button type="button" class="discard" aria-label="Put the word down (esc)"'
      + ' title="Put the word down (esc)">×</button></div></div>'
      + '<div class="prompt" role="status"></div>';
    this.el.querySelector('.sentence-host')!.replaceWith(sent);
    this.el.querySelector('.grip')?.setAttribute('title', w.doc || w.id);
    this.say();
    if (focus && this.armed >= 0) {
      const f = this.el.querySelector<HTMLInputElement | HTMLSelectElement>(
        `#wordbox-${this.armed}`);
      if (f) {
        f.focus({ preventScroll: true });
        if (f instanceof HTMLInputElement && this.vals[this.armed]) f.select();
      }
    }
  }
}

/** Kinds said short, for a caption: `point, line +3`. */
function short(xs: string[]): string {
  return xs.length > 3 ? `${xs.slice(0, 2).join(', ')} +${xs.length - 2}` : said(xs);
}
