/* The other places the name under the caret is named, lit — what every code editor does
 * (VS Code's occurrences highlight, JetBrains' usages of the element at the caret), and the first
 * feature on the core's name index (`core/program.ts::symbols`).
 *
 * Conventions taken from those editors, since a person arrives with them in their hands:
 *   * the caret *in* a name, or just past its end (`rim|`), means the name; a selection means it
 *     only where it is exactly the name (a double-click), and otherwise nothing is lit;
 *   * every place it is named is lit, the one under the caret included, and where it is
 *     *given* — declared, bound, a formal — lit more strongly than where it is read;
 *   * after a short pause, so arrowing through text does not flicker, and gone at once on an
 *     edit, whose offsets the old answer no longer fits;
 *   * F7 and ⇧F7 step the caret to the next and the previous place (VS Code's keys), round the
 *     end of the file.
 *
 * And one of this app's, whose text and drawing are one thing: the caret *put* on a name — by a
 * click, an arrow, F7, never by typing — is told to `onName`, which selects what it names in the
 * workspace (`names.ts`).
 *
 * Which word is a name and which declaration it names is the core's — this module only asks, and
 * says nothing about Solvent.  The editor knows nothing of either: it is handed marks in a layer
 * of their own, beside the picked statement's. */
import type { CodeEditor, Mark } from './editor.js';
import type { Occurrence, Symbols } from '../core/program.js';

/** The editor's layer the occurrences are marked in. */
const LAYER = 'occurrences';
/** How long the caret rests before the names are lit: long enough that arrowing through a line
 *  repaints nothing, short enough that a click reads as answered. */
const PAUSE = 120;
/** A character a name may hold — the lexer's `ident_char`. */
const WORD = /[\p{L}\p{N}_]/u;

/** The occurrence a caret or selection means (module header): the one it is in or stands just
 *  past, or the one it selects exactly. */
export function meant(index: Symbols, lo: number, hi: number): Occurrence | null {
  const o = index.at(lo);
  if (!o) return null;
  return lo === hi || (o.lo === lo && o.hi === hi) ? o : null;
}

/** What to mark for a caret or selection at `lo`..`hi`: every place the name it means is named,
 *  `occurrence-def` where it is given and `occurrence` where it is read — none where it means no
 *  name. */
export function occurrenceMarks(index: Symbols, lo: number, hi: number): Mark[] {
  const o = meant(index, lo, hi);
  if (!o) return [];
  return index.of(o.symbol)
    .map((x) => ({ lo: x.lo, hi: x.hi, cls: x.def ? 'occurrence-def' : 'occurrence' }));
}

/** Where `F7` (`step` 1) or `⇧F7` (-1) puts the caret from `lo`..`hi`: the start of the next or
 *  the previous place the name there is named, round the end of the file, or null where the caret
 *  means no name. */
export function stepTo(index: Symbols, lo: number, hi: number, step: 1 | -1): Occurrence | null {
  const o = meant(index, lo, hi);
  if (!o) return null;
  const all = index.of(o.symbol);
  const k = all.indexOf(o);
  return all[(k + step + all.length) % all.length];
}

/** The occurrences of the name under an editor's caret, lit (module header). */
export class Occurrences {
  private index: { text: string; symbols: Symbols } | null = null;
  private indexer: ((text: string) => Symbols) | null = null;
  private timer: ReturnType<typeof setTimeout> | undefined;
  /** The caret was put where it rests, not carried there by typing: what `onName` is told of. */
  private put = false;
  /** Told of the name the caret was put on, once it rests there, with the index it is read in. */
  onName: ((o: Occurrence, index: Symbols) => void) | null = null;

  constructor(private readonly ed: CodeEditor) {
    // an edit has already let go of what was lit (the editor drops a layer the text moved
    // under); either way the name is asked again once the caret rests
    ed.onCaret((edited) => { this.put = !edited; this.later(); });
    ed.box.addEventListener('keydown', (e) => {
      if (e.key !== 'F7' || e.metaKey || e.ctrlKey || e.altKey) return;
      if (this.step(e.shiftKey ? -1 : 1)) e.preventDefault();
    });
  }

  /** What reads the names of the text shown — the core's index for a program — or null where the
   *  text is not one (a drawing, a README), which lights nothing. */
  set names(indexer: ((text: string) => Symbols) | null) {
    if (indexer === this.indexer) return;
    this.indexer = indexer;
    this.index = null;
    this.put = false;
    this.later();
  }

  /** Light the name under the caret now, rather than after the pause. */
  update(): void {
    clearTimeout(this.timer);
    this.timer = undefined;
    const { lo, hi } = this.ed.caret;
    // no word against the caret, no name: the text is not indexed for it, which is most of the
    // pauses in typing — after a space, a bracket, a line's end
    const text = this.ed.text;
    const index = (WORD.test(text[lo - 1] ?? '') || WORD.test(text[lo] ?? '')) && this.symbols();
    this.ed.setMarks(index ? occurrenceMarks(index, lo, hi) : [], LAYER);
    const o = index && this.put && meant(index, lo, hi);
    this.put = false;
    if (o && index) this.onName?.(o, index);
  }

  /** Put the caret on the next (`1`) or previous (`-1`) place the name under it is named; whether
   *  there was a name to step along. */
  step(step: 1 | -1): boolean {
    const index = this.symbols();
    const { lo, hi } = this.ed.caret;
    const to = index && stepTo(index, lo, hi, step);
    if (!to) return false;
    this.ed.select(to.lo);
    this.put = true;
    this.update();
    return true;
  }

  private later(): void {
    clearTimeout(this.timer);
    this.timer = setTimeout(() => this.update(), PAUSE);
  }

  /** The index of the text shown, asked of the core once per text. */
  private symbols(): Symbols | null {
    if (!this.indexer) return null;
    const text = this.ed.text;
    if (this.index?.text !== text) {
      // a core that cannot read the text lights nothing; it is no reason to stop typing
      try { this.index = { text, symbols: this.indexer(text) }; } catch { return null; }
    }
    return this.index.symbols;
  }
}
