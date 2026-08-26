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
export interface Run { cls: string; lo: number; hi: number }

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
  /** The range the panel is pointing at — where the selected thing was written.
   *
   *  Marked with a class and **never with a weight**.  The box in front of this copy is one
   *  face throughout; a bold here would advance its glyphs differently from the box's, and
   *  every character after it on the line would sit beside the one the caret is on.  See the
   *  four rules in this file's header — this is the third of them. */
  private lit: [number, number] | null = null;

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
   *  decides whether that is the right thing; this only promises not to move them itself. */
  setText(text: string): void {
    const { selectionStart, selectionEnd, scrollTop, scrollLeft } = this.box;
    this.box.value = text;
    this.repaint();
    this.box.setSelectionRange(selectionStart, selectionEnd);
    this.box.scrollTop = scrollTop;
    this.box.scrollLeft = scrollLeft;
    this.follow();
  }

  /** Colour what is in the box now — after typing, or after the rules changed. */
  /** Point the copy at a range, or at nothing.  Repaints only when it actually moved, so a
   *  selection gesture that keeps landing on the same statement costs one paint and not sixty. */
  setLit(range: [number, number] | null): void {
    const a = this.lit, b = range;
    if (a === b || (a && b && a[0] === b[0] && a[1] === b[1])) return;
    this.lit = range;
    this.repaint();
  }

  repaint(): void {
    const text = this.text;
    const out = document.createDocumentFragment();
    const lit = this.lit;
    /* One stretch of text, split where the lit range starts and ends so the mark covers the
     * punctuation *between* coloured runs too — a statement is spans and gaps, and half a
     * marked statement would look like a colouring bug. */
    const put = (from: number, to: number, cls: string): void => {
      if (to <= from) return;
      const clamp = (i: number): number => Math.min(Math.max(i, from), to);
      const cuts = lit ? [from, clamp(lit[0]), clamp(lit[1]), to] : [from, to];
      for (let i = 0; i + 1 < cuts.length; i += 1) {
        const [a, b] = [cuts[i], cuts[i + 1]];
        if (b <= a) continue;
        const on = !!lit && a >= lit[0] && b <= lit[1];
        if (!cls && !on) {
          out.append(text.slice(a, b));
          continue;
        }
        const span = document.createElement('span');
        span.className = on ? (cls ? `${cls} lit` : 'lit') : cls;
        span.textContent = text.slice(a, b);
        out.append(span);
      }
    };
    let at = 0;
    if (text.length <= MAX_COLOUR) {
      for (const r of this.colour(text)) {
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
