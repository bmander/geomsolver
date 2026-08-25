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

/** Where a statement sits in the program text — byte offsets, plus the line and column the core
 *  worked out, so nothing here scans the text a second time. */
export interface Span { lo: number; hi: number; line: number; col: number }

export interface Diagnostic extends Span {
  severity: 'error' | 'warning' | 'note';
  code: string;
  message: string;
}

/** Which statement made which part of the drawing, and the other way round.  Byte offsets into
 *  the same text the panel holds, so a click on either end finds the other. */
export interface SourceMap {
  entities: { kind: string; index: number; name: string; lo: number; hi: number }[];
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

function edit(handle: number): Edit {
  if (!handle) throw new Error(lastError() || 'the edit could not be made');
  return takeJson<Edit>(handle);
}

export class Document {
  /** The source.  Replaced in place only by `retext`, for an edit the core called `numeric`. */
  text: string;
  readonly sketch: Sketch;
  readonly ok: boolean;
  readonly diagnostics: Diagnostic[];
  map: SourceMap;
  private h: number;
  private byName = new Map<string, Primitive>();

  private constructor(h: number, text: string) {
    const c = core();
    this.h = h;
    this.text = text;
    const r = takeJson<{
      ok: boolean;
      diagnostics: Diagnostic[];
      entities: [string, number, string, number, number][];
      constraints: [number, number, number][];
    }>(c.gcs_elab_report(h));
    const sh = c.gcs_elab_take_sketch(h);
    if (!sh) {
      c.gcs_elab_free(h);
      this.h = 0;
      throw new Error('the program built no drawing at all');
    }
    this.sketch = new Sketch(sh);
    this.ok = r.ok;
    this.diagnostics = r.diagnostics;
    this.map = {
      entities: r.entities.map(([kind, index, name, lo, hi]) => ({ kind, index, name, lo, hi })),
      constraints: r.constraints.map(([id, lo, hi]) => ({ id, lo, hi })),
    };
    for (const e of this.map.entities) {
      if (!KINDS.includes(e.kind as Kind)) continue;
      const of = this.sketch.entities(e.kind as Kind)[e.index];
      if (of) this.byName.set(e.name, of);
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

  /** Every name this elaboration gave a part of the drawing.  Several names may reach one entity —
   *  a port bound to an actual is two names and one thing — so this is a list, not a lookup. */
  nameOf(p: Primitive): string | undefined {
    for (const e of this.map.entities) {
      if (e.kind === p.kind && e.index === p.index) return e.name;
    }
    return undefined;
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

  /** The names this elaboration now gives the drawing, after it took an edit of its own. */
  private remap(): void {
    const r = takeJson<{
      entities: [string, number, string, number, number][];
      constraints: [number, number, number][];
    }>(core().gcs_elab_report(this.h));
    this.map = {
      entities: r.entities.map(([kind, index, name, lo, hi]) => ({ kind, index, name, lo, hi })),
      constraints: r.constraints.map(([id, lo, hi]) => ({ id, lo, hi })),
    };
    this.byName.clear();
    for (const e of this.map.entities) {
      if (!KINDS.includes(e.kind as Kind)) continue;
      const of = this.sketch.entities(e.kind as Kind)[e.index];
      if (of) this.byName.set(e.name, of);
    }
  }

  addPoint(x: number, y: number): Edit {
    return edit(core().gcs_elab_add_point(this.h, x, y));
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

/** The canonical program for a sketch — the lift, and how a JSON document becomes a source one. */
export function fromSketch(sk: Sketch): string {
  return takeStr(core().gcs_sketch_to_program(sk.handle));
}
