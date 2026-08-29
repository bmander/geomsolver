/* The document, which is a program.
 *
 * A `Document` is one elaboration: the source somebody wrote, the drawing it makes, what was wrong
 * with it, and where each part of it was written.  It owns a core handle, so it is disposed like a
 * sketch — and it owns the *sketch* too, since the drawing is what the program came to and the two
 * cannot outlive each other usefully.
 *
 * Every edit verb here returns an `Edit` and **changes nothing**: an edit is a new text, and a new
 * text is a new `Document`.  `apply` is the one place a document is replaced, so there is exactly
 * one seam where the drawing can disagree with the source, and it is a single function.
 *
 * `kind` is the core's reading of what an edit costs, and the front end never guesses it: a bare
 * number is `numeric` (the topology cannot have moved, so a compiled plan survives), a number that
 * names anything is `structural`, because a name nothing defines is a free variable and a free
 * variable is a column.
 */
import { Constraint } from './constraints.js';
import { Kind, KINDS, Primitive, Sketch } from './model.js';
import { core, lastError, takeJson, takeStr, withJson, withStr } from './wasm.js';

/** Where a statement sits in the program text, plus the line and column the core worked out, so
 *  nothing here scans the text a second time.
 *
 *  `lo`/`hi` index the **string**: every offset the core reports is brought onto the text the
 *  browser holds at the one seam it crosses (see `Offsets`), so nothing downstream has to remember
 *  which of the two units it is holding.  A caller slices with them and hands them to
 *  `setSelectionRange`. */
export interface Span extends SourceSpan { line: number; col: number }

/** A stretch of the program text, by string index — what a source-map entry pins down.  Not a
 *  `Span`: that also carries the line and column the core worked out for a diagnostic, which a
 *  map entry has no use for. */
export interface SourceSpan { lo: number; hi: number }

export interface Diagnostic extends Span {
  severity: 'error' | 'warning' | 'note';
  code: string;
  message: string;
}

/** Which statement made which part of the drawing, and the other way round.  Offsets into the
 *  same text the panel holds, so a click on either end finds the other. */
export interface SourceMap {
  /** `name` is absent for an **anonymous** element: the source calls it nothing, and the core
   *  withholds the key it elaborated under (that key is its statement's offset, so a selection
   *  carried on one could land on a different entity after an edit above it).  `lo`/`hi` still
   *  say where it was written. */
  entities: { kind: string; index: number; name?: string; lo: number; hi: number }[];
  constraints: { id: number; lo: number; hi: number }[];
}

/** What an edit costs. */
export type EditKind = 'structural' | 'numeric' | 'none';

/** A proposed new source.  Nothing has happened yet: `text` is what the document would say. */
export interface Edit {
  text: string;
  kind: EditKind;
  /** The names the edit minted, in the order it made them. */
  names: string[];
  /** Set when the core declined, and says why.  `text` is then unchanged. */
  refused: string | null;
}

/** What `gcs_elab_report` hands back.  The offsets in it are the core's **bytes**; `Document.adopt`
 *  is the only thing that reads them, and brings them onto the string as it goes. */
interface Report {
  ok: boolean;
  diagnostics: Diagnostic[];
  entities: [string, number, string, number, number][];
  constraints: [number, number, number][];
}

function edit(handle: number): Edit {
  if (!handle) throw new Error(lastError() || 'the edit could not be made');
  return takeJson<Edit>(handle);
}

export class Document {
  /** The source.  Replaced in place only by `retext`, for an edit the core called `numeric`. */
  text: string;
  readonly sketch: Sketch;
  readonly ok: boolean;
  diagnostics: Diagnostic[];
  map: SourceMap;
  private h: number;
  private byName = new Map<string, Primitive>();

  private constructor(h: number, text: string) {
    const c = core();
    this.h = h;
    this.text = text;
    const r = takeJson<Report>(c.gcs_elab_report(h));
    const sh = c.gcs_elab_take_sketch(h);
    if (!sh) {
      c.gcs_elab_free(h);
      this.h = 0;
      throw new Error('the program built no drawing at all');
    }
    this.sketch = new Sketch(sh);
    this.ok = r.ok;
    this.diagnostics = [];
    this.map = { entities: [], constraints: [] };
    this.adopt(r);
  }

  /** Take a report: the offsets onto the string, the map, and the names it gives the drawing.
   *
   *  The one place a report is read, so the conversion happens exactly once and a diagnostic, a
   *  statement's span and a coloured run are all in the same units by the time anyone sees them. */
  private adopt(r: Report): void {
    const at = new Offsets(this.text);
    this.diagnostics = r.diagnostics.map((d) => ({ ...d, lo: at.at(d.lo), hi: at.at(d.hi) }));
    this.map = {
      // an anonymous element comes back with an empty name; it becomes *absent* here, at the
      // one seam a report is read, so every later reader — `byName`, `nameOf`'s type, the
      // selection carried across a re-elaboration — inherits the rule rather than each
      // remembering to test for `''`
      entities: r.entities.map(([kind, index, name, lo, hi]) =>
        ({ kind, index, name: name || undefined, lo: at.at(lo), hi: at.at(hi) })),
      constraints: r.constraints.map(([id, lo, hi]) => ({ id, lo: at.at(lo), hi: at.at(hi) })),
    };
    this.byName.clear();
    for (const e of this.map.entities) {
      const of = e.name ? this.entityOf(e) : undefined;
      if (of) this.byName.set(e.name!, of);
    }
  }

  /** Elaborate a source.  Never throws for a *program* error — a program with one bad line comes
   *  back with the other twenty drawn and the diagnostics beside them, because a panel has to show
   *  the drawing *and* the error.  Whether to adopt it is the caller's, from `ok`. */
  static read(text: string): Document {
    const h = withStr(text, (p, n) => core().gcs_program_elaborate(p, n));
    if (!h) throw new Error(lastError() || 'could not read the program');
    return new Document(h, text);
  }

  /** Take the text of a `numeric` edit without rebuilding the drawing.
   *
   *  That is the whole value of the classification: a re-elaboration is a new sketch, and a new
   *  sketch is a lost plan, a lost compiled system and a lost selection — which is what would make
   *  editing a dimension stop being instant.  The spans still follow, so the next edit splices in
   *  the right place.  False when the core would rather be re-elaborated. */
  retext(text: string): boolean {
    const ok = withStr(text, (p, n) => core().gcs_elab_retext(this.h, p, n)) !== 0;
    if (ok) this.text = text;
    return ok;
  }

  dispose(): void {
    if (this.h) core().gcs_elab_free(this.h);
    this.h = 0;
    this.sketch.dispose();
    this.byName.clear();
  }

  /** The entity a name in the source stands for, in *this* elaboration.  How a selection crosses
   *  from one document to the next: a proxy dies with its sketch, a name does not. */
  entity(name: string): Primitive | undefined {
    return this.byName.get(name);
  }

  /** The entity a source-map entry stands for.  Every entry carries the kind and the index, so
   *  this resolves an **anonymous** element as readily as a named one — which `entity` cannot,
   *  there being no name to ask by.  Within one elaboration it is the lookup to use; only a
   *  selection *crossing* elaborations needs the name. */
  entityOf(entry: { kind: string; index: number }): Primitive | undefined {
    if (!KINDS.includes(entry.kind as Kind)) return undefined;
    return this.sketch.entities(entry.kind as Kind)[entry.index];
  }

  /** What the source map says about a part of the drawing: the name it was declared under and
   *  where that declaration sits.  The one walk behind `nameOf` and `spanOf`, so "which entry is
   *  this entity's" is asked in one place however many things come to want the answer. */
  private entryOf(p: Primitive): SourceMap['entities'][number] | undefined {
    return this.map.entities.find((e) => e.kind === p.kind && e.index === p.index);
  }

  /** Every name this elaboration gave a part of the drawing.  Several names may reach one entity —
   *  a port bound to an actual is two names and one thing — so this is a list, not a lookup. */
  nameOf(p: Primitive): string | undefined {
    return this.entryOf(p)?.name;
  }

  /** Where a part of the drawing was written down, and where a constraint was.
   *
   *  **The document answers this, not its readers.**  A span is a fact about the source, and the
   *  source map is this class's; a caller that walked `map` itself would be the second reader of
   *  a table with one owner, and the fourth such walk is what made these two worth having. */
  spanOf(p: Primitive): SourceSpan | undefined {
    return this.entryOf(p);
  }

  spanOfConstraint(id: number): SourceSpan | undefined {
    return this.map.constraints.find((c) => c.id === id);
  }

  /* -- the edit verbs ------------------------------------------------------------------- */

  /** Put where the drawing *is* back into the seeds it started from — the one edit a solve makes.
   *  A seed written as an expression, or one statement standing for many instances, is left
   *  alone: `kind` is then `none` and the text is unchanged. */
  commitSeeds(sk: Sketch = this.sketch): Edit {
    return edit(core().gcs_elab_commit_seeds(this.h, sk.handle));
  }

  /** Bring the source into step with a drawing a gesture changed: what the sketch has that this
   *  elaboration did not gets a statement appended, what it has lost loses one, and the seeds are
   *  committed.  A splice — so a component, a cycle and every comment survive a line drawn beside
   *  them.
   *
   *  **It applies itself, and the drawing is not rebuilt.**  Nothing about the drawing changed
   *  here; it had already changed, and the source is only catching up.  So every proxy the caller
   *  is holding stays good — which is the state a multi-click tool is in between its clicks — and
   *  `kind` is a report rather than an instruction. */
  reconcile(sk: Sketch = this.sketch): Edit {
    const e = edit(core().gcs_elab_reconcile(this.h, sk.handle));
    if (!e.refused && e.kind !== 'none') {
      this.text = e.text;
      this.remap();
    }
    return e;
  }

  /** The names this elaboration now gives the drawing, after it took an edit of its own.  Read
   *  against `this.text`, which `reconcile` has already moved on to the spliced source. */
  private remap(): void {
    this.adopt(takeJson<Report>(core().gcs_elab_report(this.h)));
  }

  addPoint(x: number, y: number): Edit {
    return edit(core().gcs_elab_add_point(this.h, x, y));
  }

  /** A `Rectangle` component instance — and the component's definition, the first time. */
  addRectangle(w: number, h: number): Edit {
    return edit(core().gcs_elab_add_rectangle(this.h, w, h));
  }

  /** An entity over names already in the source. */
  addEntity(kind: string, args: string[], seed: number[] = []): Edit {
    return edit(
      withJson({ kind, args, seed }, (p, n) => core().gcs_elab_add_entity(this.h, p, n)),
    );
  }

  /** One constraint, in the record shape both bindings already build — but with entities named
   *  the way the document names them, rather than indexed into a sketch about to be replaced. */
  addRelation(type: string, args: unknown[]): Edit {
    return edit(withJson({ type, args }, (p, n) => core().gcs_elab_add_relation(this.h, p, n)));
  }

  /** Take out the statements that declare these, and every statement that named one. */
  remove(entities: Iterable<Primitive> = [], constraints: Iterable<Constraint> = []): Edit {
    const ents = [...entities].map((e) => e.ref);
    const cons = [...constraints].map((c) => c.id);
    return edit(
      withJson({ entities: ents, constraints: cons }, (p, n) =>
        core().gcs_elab_remove(this.h, p, n),
      ),
    );
  }

  /** Rewrite a dimension's number, as written — an expression stays an expression. */
  setDimension(id: number, attr: string, text: string): Edit {
    return edit(
      withStr(attr, (ap, an) =>
        withStr(text, (tp, tn) => core().gcs_elab_set_dimension(this.h, id, ap, an, tp, tn)),
      ),
    );
  }
}

/** A run of the source and what it is.  `cls` is the core's own name for the class — a stylesheet
 *  says what it looks like, and nothing here says what it means.
 *
 *  `lo`/`hi` index the **string**, not the core's bytes: a caller here slices a JS string with
 *  them, and one that had to remember which of the two it was holding would get it wrong. */
export interface Run { cls: string; lo: number; hi: number }

/** Colour a program.  The classified runs only, in order: whatever falls between two of them is
 *  ordinary text, so a caller writes the gaps out plainly and never has to describe whitespace.
 *
 *  A function of the *text*, not of a `Document`: the program being looked at is usually the one
 *  half-typed, which does not elaborate.  The core's own scan, so what a colour says a word is and
 *  what the parser makes of it are the same answer. */
export function highlight(text: string): Run[] {
  const runs = withStr(text, (p, n) => core().gcs_program_highlight(p, n));
  if (!runs) throw new Error(lastError() || 'the program could not be coloured');
  const at = new Offsets(text);
  return takeJson<[string, number, number][]>(runs)
    .map(([cls, lo, hi]) => ({ cls, lo: at.at(lo), hi: at.at(hi) }));
}

/** Byte offsets, as the core counts them, onto the string the browser holds.
 *
 *  The core measures a source in UTF-8 bytes and a JS string is UTF-16 code units.  The two agree
 *  exactly while the text is ASCII and part company at the first character that is not — an em
 *  dash in a comment, a `π` in one — after which every offset is long by however many extra bytes
 *  have gone by.  `gear.sv` has an em dash in its second line, so this is not a corner case: left
 *  unconverted, clicking a diagnostic selects the wrong words and a statement lights the wrong
 *  part of the drawing.
 *
 *  **This is the one seam.**  Every offset the core reports crosses it — the highlighting, the
 *  diagnostics and the source map alike — so no consumer has to know the difference, and no two of
 *  them can disagree about which unit they hold.
 *
 *  A program that is all ASCII builds nothing and answers in one comparison, which is the usual
 *  case.  Otherwise one walk records where each wider-than-one-byte character ends, and a lookup
 *  is a binary search of that — so offsets need not arrive in any order, which the source map's
 *  do not. */
class Offsets {
  /** Byte offset just past each character whose UTF-8 length differs from its UTF-16 length. */
  private readonly ends: number[] = [];
  /** The string index just past that same character. */
  private readonly units: number[] = [];

  constructor(text: string) {
    let byte = 0, unit = 0;
    while (unit < text.length) {
      const c = text.codePointAt(unit) as number;
      const bytes = c < 0x80 ? 1 : c < 0x800 ? 2 : c < 0x10000 ? 3 : 4;
      const wide = c < 0x10000 ? 1 : 2;
      byte += bytes;
      unit += wide;
      if (bytes !== wide) {
        this.ends.push(byte);
        this.units.push(unit);
      }
    }
  }

  /** True when the two agree everywhere, so nothing has to be converted at all. */
  get plain(): boolean {
    return this.ends.length === 0;
  }

  at(off: number): number {
    if (this.plain) return off;
    // the last wide character that ends at or before `off`; everything after it is one-to-one
    let lo = 0, hi = this.ends.length;
    while (lo < hi) {
      const mid = (lo + hi) >> 1;
      if (this.ends[mid] <= off) lo = mid + 1;
      else hi = mid;
    }
    return lo === 0 ? off : this.units[lo - 1] + (off - this.ends[lo - 1]);
  }
}

/** The canonical program for a sketch — the lift, and how a JSON document becomes a source one. */
export function fromSketch(sk: Sketch): string {
  return takeStr(core().gcs_sketch_to_program(sk.handle));
}
