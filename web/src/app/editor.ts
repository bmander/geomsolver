/* A box you type code into, with the code coloured.
 *
 * The browser has no such control.  A `<textarea>` takes typing and gives you the caret, undo,
 * autoscroll and every key the platform defines, but it cannot colour a word of what it holds; a
 * `contenteditable` can be coloured and gets all of that wrong.  So the source is drawn **twice**:
 * a `<pre>` behind, carrying the colour, and the textarea in front with its own glyphs made
 * invisible and only its caret showing.
 *
 * Everything hard about that is one sentence: **the two layers must put every character in exactly
 * the same place**, or the caret sits somewhere other than the text it is in front of.  Sharing
 * the CSS is necessary and not sufficient, because the two do not lay out the *same* text — the
 * box has one uninterrupted run per line where the copy has one element per coloured run.  Three
 * things follow, and none of them is a nicety:
 *
 *   * the line height is a whole number of pixels (a fractional line box rounds differently in a
 *     textarea's internal layout than in a `<pre>`, a fifth of a pixel a line, a whole line out by
 *     the bottom of a long file);
 *   * kerning and ligatures are off (they form across a run, and the runs differ);
 *   * a coloured run may change the colour and **never the face** — an italic or a bold is a
 *     different font, whose advances the regular face makes no promise about, and the box has no
 *     spans to match it with.
 *
 * The first two live in `app.css` beside the shared metrics.  The third is a rule about every
 * class this ever renders, which is why it is written here as well as there.
 *
 * A fourth thing does not belong in a stylesheet at all: the copy is *moved* to follow the box's
 * scrolling and never scrolled itself — see `follow()`.
 *
 * None of that can be seen by a unit test, because it is a question about what a font and a
 * layout engine do.  `tools/overlay.mjs` drives a real browser and checks it: same metrics, the
 * colouring moves no glyph off where the plain text puts it, and the copy follows the box to both
 * ends of the file.
 *
 * This module knows nothing about Solvent, the sketch, or the shell: it is handed the text and a
 * function that says which runs of it are what.  That is what lets the harness build the very same
 * control the app builds, rather than a second one that might not have the bug. */

/** A run of the text and what it is.  `cls` becomes the element's class and the stylesheet says
 *  what it looks like — see the face rule above. */
/** A stretch of the text, by string index. */
export interface Extent { lo: number; hi: number }

export interface Run extends Extent { cls: string }

/** Past this the colouring is dropped and the text is shown plain: one span per run stops being
 *  free long before a textarea's own limit, and a file this long is not one anybody is reading a
 *  word of.  Typing stays instant, which is the thing that must not be traded. */
const MAX_COLOUR = 200_000;

export class CodeEditor {
  /** What takes the typing.  Callers own the selection and the key handling through it; the
   *  colouring behind it is this class's business alone. */
  readonly box: HTMLTextAreaElement;
  private readonly copy: HTMLPreElement;
  private readonly colour: (text: string) => Run[];
  /** A range a caller has asked to be marked.  What it *means* is the caller's business; this
   *  class knows only that those characters carry one more class than their neighbours.
   *
   *  Marked with a class and **never with a weight**.  The box in front of this copy is one
   *  face throughout; a bold here would advance its glyphs differently from the box's, and
   *  every character after it on the line would sit beside the one the caret is on.  See the
   *  four rules in this file's header — this is the third of them. */
  private lit: Extent | null = null;
  /** The last colouring, against the text it was made from.  Colouring crosses the ABI and
   *  re-lexes the document, and a repaint that only moved the *mark* has the same text and so
   *  the same runs — which used to be paid for again anyway. */
  private lastRuns: { text: string; runs: Run[] } | null = null;

  /** Build the two layers inside `host`, which is expected to be positioned (the stylesheet makes
   *  `#pcode` so).  The copy goes first so the box paints over it and takes the clicks. */
  constructor(host: HTMLElement, colour: (text: string) => Run[]) {
    this.colour = colour;
    this.copy = document.createElement('pre');
    this.copy.className = 'code-copy';
    this.copy.setAttribute('aria-hidden', 'true');
    this.box = document.createElement('textarea');
    this.box.className = 'code-box';
    for (const [k, v] of Object.entries({
      spellcheck: 'false', autocapitalize: 'off', autocorrect: 'off', autocomplete: 'off',
      wrap: 'off',
    })) {
      this.box.setAttribute(k, v);
    }
    host.replaceChildren(this.copy, this.box);
    // the copy has no scrollbars of its own — it is moved by the box, which is the only one that
    // knows where the text has got to
    this.box.addEventListener('scroll', () => this.follow());
  }

  get text(): string {
    return this.box.value;
  }

  /** Put text in, colour it, and leave the caret and the scroll where they were.  The caller
   *  decides whether that is the right thing; this only promises not to move them itself.
   *
   *  `lit` comes with the text because it usually moves with it: a splice shifts every offset
   *  after it, so setting the two apart would repaint once against the old mark and again
   *  against the new one. */
  setText(text: string, lit?: Extent | null): void {
    if (lit !== undefined) this.lit = lit;
    const { selectionStart, selectionEnd, scrollTop, scrollLeft } = this.box;
    this.box.value = text;
    this.repaint();
    this.box.setSelectionRange(selectionStart, selectionEnd);
    this.box.scrollTop = scrollTop;
    this.box.scrollLeft = scrollLeft;
    this.follow();
  }

  /** Mark a range, or nothing.  Repaints only when it actually moved, so a gesture that keeps
   *  landing on the same range costs one paint and not sixty. */
  setLit(range: Extent | null): void {
    const a = this.lit, b = range;
    if (a && b ? a.lo === b.lo && a.hi === b.hi : a === b) return;
    this.lit = range;
    this.repaint();
  }

  /** Colour what is in the box now — after typing, or after the rules changed. */
  repaint(): void {
    const text = this.text;
    const out = document.createDocumentFragment();
    const lit = this.lit;
    /** One piece of one colour: a bare string where it has nothing to say, a span where it has. */
    const piece = (a: number, b: number, cls: string): void => {
      if (b <= a) return;
      if (!cls) {
        out.append(text.slice(a, b));
        return;
      }
      const span = document.createElement('span');
      span.className = cls;
      span.textContent = text.slice(a, b);
      out.append(span);
    };
    /* One stretch of text, cut where the mark starts and ends so it covers the punctuation
     * *between* coloured runs too — a statement is spans and gaps, and half a marked statement
     * would look like a colouring bug.  Nothing marked puts both cuts at the end, and so does a
     * range lying outside this stretch, which is how those cases need no branch of their own. */
    const put = (from: number, to: number, cls: string): void => {
      const cut = (i: number): number => Math.min(Math.max(i, from), to);
      const [lo, hi] = lit ? [cut(lit.lo), cut(lit.hi)] : [to, to];
      piece(from, lo, cls);
      piece(lo, hi, cls ? `${cls} lit` : 'lit');
      piece(hi, to, cls);
    };
    let at = 0;
    if (text.length <= MAX_COLOUR) {
      for (const r of this.runs(text)) {
        put(at, r.lo, '');
        put(r.lo, r.hi, r.cls);
        at = r.hi;
      }
    }
    // the tail, and a newline past it: a `pre` drops the last one, and the copy has to be exactly
    // as tall as the box in front of it or the two scroll apart at the bottom
    put(at, text.length, '');
    out.append('\n');
    this.copy.replaceChildren(out);
    this.follow();
  }

  /** The colouring of this text, from last time where the text has not changed since. */
  private runs(text: string): Run[] {
    if (this.lastRuns?.text !== text) this.lastRuns = { text, runs: this.colour(text) };
    return this.lastRuns.runs;
  }

  /** One line, in pixels, as the stylesheet has it — asked for rather than written down, so a
   *  caller scrolling by lines cannot disagree with the layer that draws them. */
  get lineHeight(): number {
    return parseFloat(getComputedStyle(this.box).lineHeight) || 0;
  }

  /** Put `line` a few lines below the top, without taking the focus from wherever it is. */
  scrollToLine(line: number, margin = 4): void {
    this.box.scrollTop = Math.max(0, (line - margin) * this.lineHeight);
    this.follow();
  }

  /** Bring the copy to where the box is scrolled to.
   *
   *  By *moving* it, not by scrolling it.  Scrolling the copy looks like the obvious thing and is
   *  the bug this class exists to not have: the box carries scrollbars and the copy does not, so
   *  the box's client area is shorter by a scrollbar's width and its scroll range is longer by the
   *  same — and `copy.scrollTop = box.scrollTop` then *clamps* at the bottom of a long file,
   *  leaving the text a scrollbar's width above the caret sitting in it.  A translation has no
   *  range to clamp against, so the two cannot come apart however far down the file they are. */
  private follow(): void {
    this.copy.style.transform = `translate(${-this.box.scrollLeft}px, ${-this.box.scrollTop}px)`;
  }
}
