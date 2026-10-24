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
 * names anything is `structural`, because the name may be an unknown, and an unknown is a
 * column.
 */
import type { Level, Pictured, ScenarioMark } from './scenario.js';
import { Constraint } from './constraints.js';
import { Kind, KINDS, Plane, Primitive, Sketch, fromWire } from './model.js';
import type { Way } from './model.js';
import type { Row } from './outline.js';
import { core, lastError, takeJson, takeStr, withBuf, withJson, withStr } from './wasm.js';

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

/** How a face is swept (Solvent §6.9): along its normal (`depth`, or `from` and `to`), through a
 *  body, or turned `about` a line or an axis in its plane. */
export interface SolidSpec extends SweepSpec {
  face: string;
  name?: string;
}

/** A sweep alone, each extent the text written. */
export interface SweepSpec {
  depth?: string;
  from?: string;
  to?: string;
  through?: string;
  about?: string;
  sweep?: string;
  sense?: Sense;
}

/** Which way a revolution turns about its axis: right-handed (`ccw`, the default) or not. */
export type Sense = 'cw' | 'ccw';

export type BodyWord = 'union' | 'cut' | 'common';


/** A region of the drawing (`Document.regionAt`): the edges of its outer loop and of each hole in
 *  walk order, by name, and the rings — outer first — in the plane's own coordinates. */
export interface Region {
  outer: string[];
  holes: string[][];
  rings: [number, number][][];
}

/** A requirement of a load scenario, answered at the drawing as it stands (#510): what it is of as
 *  written, its grade (`claim`, `driving`, …), its value in the document's units and whether it
 *  holds — both null where it has no answer — and the core's sentence for it. */
export interface ScenarioAnswer {
  /** Its requirement's place among the scenario's, as `ScenarioMark.index` counts them. */
  index: number;
  requirement: string;
  grade: string;
  value: number | null;
  holds: boolean | null;
  text: string;
}

/** What a load scenario says (`report::scenarios_json`): its words — a motion its supports leave
 *  free, an edge where a `safety` it states is unbounded — and each requirement's answer. */
export interface Scenario {
  name: string;
  words: string[];
  answers: ScenarioAnswer[];
}

/** A proposed new source.  Nothing has happened yet: `text` is what the document would say. */
export interface Edit {
  text: string;
  kind: EditKind;
  /** The names the edit minted, in the order it made them. */
  names: string[];
  /** Set when the core declined, and says why.  `text` is then unchanged. */
  refused: string | null;
  /** What a removal was asked to take and left standing — what a component made, which has no
   *  statement of its own to delete. */
  kept: number;
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
  /** The core handle, for a call that takes the elaboration beside the sketch. */
  get handle(): number {
    return this.h;
  }
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
        ({ kind: fromWire(kind), index, name: name || undefined, lo: at.at(lo), hi: at.at(hi) })),
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

  /** Whether it is still held: not yet let go (`dispose`). */
  get live(): boolean {
    return this.h !== 0;
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

  /** **What each load scenario says** of the drawing as it stands, in the core's words: every
   *  requirement judged — a claim kept or refuted — over a mesh of the solved drawing.  Read
   *  again after each solve; empty for a document with no scenario. */
  scenarios(): Scenario[] {
    return takeJson<Scenario[]>(core().gcs_elab_scenarios_json(this.h, this.sketch.handle)) ?? [];
  }

  /** **What every load scenario says, and how the drawing moves under scenario `drawn`** — its
   *  index among `scenarios()`, or −1 for none: the answers `scenarios()` gives, and the picture
   *  read with them, each body analysed once for both.  Seconds for a solid read in three
   *  dimensions: what the mesh worker asks (`scenario-view.ts`), never the page's own thread. */
  scenariosPictured(drawn: number): Pictured {
    return takeJson<Pictured>(
      core().gcs_elab_scenarios_pictured_json(this.h, this.sketch.handle, drawn))
      ?? { scenarios: [], picture: null, measures: [] };
  }

  /** **How many masses and volumes the document states** (#510, `mass(10kg) arm`): whether there
   *  is anything to read beside its scenarios.  No analysis. */
  measuresStated(): number {
    return core().gcs_elab_measures_stated(this.h);
  }

  /** **Every load scenario read a level at a time** (`ScenarioLevels`), coarsest first, scenario
   *  `drawn` (its index, −1 for none) pictured at each: what the worker steps, posting each level
   *  as it comes.  The last level is `scenariosPictured(drawn)`'s reading. */
  scenarioLevels(drawn: number): ScenarioLevels {
    return ScenarioLevels.create(this.h, this.sketch.handle, drawn);
  }

  /** **The load scenarios the document states**, by name in the order written, and the one each
   *  extends: what a chooser lists, with no analysis. */
  scenarioNames(): { name: string; extends: string | null }[] {
    return takeJson<{ name: string; extends: string | null }[]>(
      core().gcs_elab_scenario_names_json(this.h)) ?? [];
  }

  /** **Where scenario `k` falls on the drawing** as it stands: a mark for each of its lines, in
   *  the order written, every place and direction the core's.  No analysis: read on every edit. */
  scenarioMarks(k: number): ScenarioMark[] {
    const marks = takeJson<ScenarioMark[]>(
      core().gcs_elab_scenario_marks_json(this.h, this.sketch.handle, k)) ?? [];
    // the core's spans are bytes of the text, brought onto the string
    const at = new Offsets(this.text);
    for (const m of marks) if (m.span) m.span = [at.at(m.span[0]), at.at(m.span[1])];
    return marks;
  }

  /** **The solids this document names**, in the order it names them.
   *
   *  Read off the source map rather than through a proxy, because a solid *has* no proxy: it is
   *  not on the sheet, owns no parameter and is picked by nothing there (§6.9).  What a viewer
   *  wants of one is its name and the index `mesh()` takes, and both are in the map already. */
  solids(): { name: string; index: number }[] {
    const out: { name: string; index: number }[] = [];
    for (const e of this.map.entities) {
      if (e.kind === 'solid' && e.name) out.push({ name: e.name, index: e.index });
    }
    return out;
  }

  /** Implicit generated surfaces; evaluate the defining equation with envelopeSample. */
  envelopes(): { name: string; index: number }[] {
    return this.map.entities.filter(e => e.kind === 'envelope' && e.name)
      .map(e => ({ name: e.name!, index: e.index }));
  }

  /** Named faces: planar profiles. */
  faces(): { name: string; index: number }[] {
    return this.map.entities.filter(e => e.kind === 'face' && e.name)
      .map(e => ({ name: e.name!, index: e.index }));
  }

  /** Named frames that move (#383), evaluated through `frameSample` in world coordinates. */
  movingFrames(): { name: string; index: number }[] {
    return this.map.entities.filter(e => e.kind === 'frame' && e.name
      && this.sketch.frames[e.index]?.moves)
      .map(e => ({ name: e.name!, index: e.index }));
  }

  /** Named analytic surfaces, evaluated through surfaceSample rather than a drawing proxy. */
  surfaces(): { name: string; index: number }[] {
    return this.map.entities.filter(e => e.kind === 'surface' && e.name)
      .map(e => ({ name: e.name!, index: e.index }));
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

  /** Where statement `stmt` of this text was written — a claim about solids, which makes no
   *  constraint to be found by — or nothing where the text has no such statement (one a module
   *  states).  The core's bytes brought onto the string, as every report's are. */
  spanOfStatement(stmt: number): SourceSpan | undefined {
    const span = withBuf(2, 4, (b) =>
      core().gcs_elab_stmt_span(this.h, stmt, b.ptr) < 0 ? undefined : [b.i32[0], b.i32[1]]);
    if (!span) return undefined;
    const at = new Offsets(this.text);
    return { lo: at.at(span[0]), hi: at.at(span[1]) };
  }

  /** **The model outline** (#435): every instance this document makes and its members, as the
   *  core arranges them (`gcs_elab_outline_json`), each span brought onto the string. */
  outline(): Row[] {
    type Wire = Omit<Row, 'span' | 'children'> & { span?: [number, number]; children: Wire[] };
    const at = new Offsets(this.text);
    const row = (w: Wire): Row => {
      const { span, children, ...rest } = w;
      return { ...rest, ...(span ? { span: { lo: at.at(span[0]), hi: at.at(span[1]) } } : {}),
               children: children.map(row) };
    };
    return takeJson<Wire[]>(core().gcs_elab_outline_json(this.h, this.sketch.handle)).map(row);
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

  /** `use NAME` as a line of the document's own — nothing when it already says it. */
  addUse(name: string): Edit {
    return edit(withStr(name, (p, n) => core().gcs_elab_add_use(this.h, p, n)));
  }

  /** A `Rectangle` component instance — and the component's definition, the first time. */
  /** `plane` draws the instance in that view — `r0: Rectangle(…) in top` — and the empty
   *  string is the page. */
  addRectangle(w: number, h: number, plane = ''): Edit {
    return edit(withStr(plane, (p, n) => core().gcs_elab_add_rectangle(this.h, w, h, p, n)));
  }

  /** An entity over names already in the source.  A plane is over the two axes or lines `args`
   *  names — `plane(u: a, v: b)` — and also takes the `name` the statement is to be given,
   *  refused if taken. */
  addEntity(kind: string, args: string[], seed: number[] = [], name?: string): Edit {
    return edit(
      withJson({ kind, args, seed, name }, (p, n) => core().gcs_elab_add_entity(this.h, p, n)),
    );
  }

  /** One constraint, in the record shape both bindings already build — but with entities named
   *  the way the document names them, rather than indexed into a sketch about to be replaced. */
  addRelation(type: string, args: unknown[]): Edit {
    return edit(withJson({ type, args }, (p, n) => core().gcs_elab_add_relation(this.h, p, n)));
  }

  /** A face over drawn edges, `edges` in walk order.  A hole that is one circle is named as it
   *  stands; any other loop is written first as a face of its own, since a face is a hole as it
   *  stands.  `names` is the face, then each hole face. */
  addFace(edges: string[], holes: string[][] = [], name?: string): Edit {
    return edit(withJson({ edges, holes, name }, (p, n) => core().gcs_elab_add_face(this.h, p, n)));
  }

  /** A solid swept from a face, each extent the text written (`'20'`, `'20mm'`, `'90deg'`). */
  addSolid(spec: SolidSpec): Edit {
    return edit(withJson(spec, (p, n) => core().gcs_elab_add_solid(this.h, p, n)));
  }

  /** A plane or an axis dropped by a button, constrained to what `from` names (the selection)
   *  by the core's table, and seeded off the live drawing; `current` is the plane being drawn
   *  on, which a datum with nothing selected is set off. */
  addDatum(kind: 'plane' | 'axis', from: string[], current?: string, name?: string): Edit {
    return edit(withJson({ kind, from, current, name },
      (p, n) => core().gcs_elab_add_datum(this.h, this.sketch.handle, p, n)));
  }

  /** **Sketch on a face**: a plane lying on face `face` (its path, `b0.near`) of object `solid`
   *  (its index), its origin where the eye's ray met it (`at`, in space) or the face's middle,
   *  touching the surface the face lies on there (`b0.near.surface`) — so it follows the solid.
   *  `names[0]` is the plane. */
  sketchOnFace(solid: number, face: string, at?: [number, number, number], name?: string): Edit {
    return edit(withJson({ face, at, name },
      (p, n) => core().gcs_elab_sketch_on_face(this.h, this.sketch.handle, solid, p, n)));
  }

  /** The plane already lying on face `face` of object `solid`, by name — the profile's own for a
   *  cap on it — or null. */
  facePlane(solid: number, face: string): string | null {
    return takeJson<string | null>(withStr(face,
      (p, n) => core().gcs_elab_face_plane_json(this.h, this.sketch.handle, solid, p, n)));
  }

  /** **Round or chamfer** edges of object `solid` (its index): whole faces (every edge of each)
   *  and edges (the two faces meeting there), by the paths its mesh names them by — or with
   *  `whole` the body's every edge — written as the library's word over face paths at `size`
   *  (the radius, or the flat's leg, as typed). */
  addRounds(solid: number, word: 'round' | 'chamfer', faces: string[], edges: [string, string][],
            size: string, whole = false): Edit {
    return edit(withJson({ word, faces, edges, whole, size },
      (p, n) => core().gcs_elab_add_rounds(this.h, this.sketch.handle, solid, p, n)));
  }

  /** **Extrude** the region a click found — its loops by name — as one splice: the face, the
   *  solid swept from it at a round default depth (or `through` the body), and with a body the
   *  body rule.  `names` is the solid, then the faces. */
  extrude(outer: string[], holes: string[][],
          with_?: { word: BodyWord; body: string; through?: boolean }): Edit {
    const req = { outer, holes, word: with_?.word, body: with_?.body, through: with_?.through };
    return edit(withJson(req, (p, n) => core().gcs_elab_extrude(this.h, this.sketch.handle, p, n)));
  }

  /** **Revolve** the region a click found — its loops by name — about the line or axis `about`
   *  (a name: `ax`, `std.z`, `v0.u`) as one splice: the face, the solid turned a whole turn about
   *  it, and with a body the body rule.  `names` is the solid, then the faces. */
  revolve(outer: string[], holes: string[][], about: string,
          with_?: { word: BodyWord; body: string }): Edit {
    const req = { outer, holes, about, word: with_?.word, body: with_?.body };
    return edit(withJson(req, (p, n) => core().gcs_elab_revolve(this.h, p, n)));
  }

  /** Rewrite how solid `solid` (its index) is swept, over the same face: an extrusion's
   *  thickness the way it runs (`{ way, thick }`, the core writing `depth:` or `from:`/`to:`), or
   *  a sweep outright. */
  setSweep(solid: number, spec: SweepSpec | { way: Way; thick: string }): Edit {
    return edit(withJson(spec, (p, n) => core().gcs_elab_set_sweep(this.h, solid, p, n)));
  }

  /** Rewrite how far revolution `solid` (its index) turns about what it turns about: `angle` in
   *  degrees as written — a number signed, negative clockwise, a whole turn written as none; an
   *  expression the way it already turns. */
  setTurn(solid: number, angle: string): Edit {
    return edit(withStr(angle, (p, n) => core().gcs_elab_set_turn(this.h, solid, p, n)));
  }

  /** The body rule: `what union body`, `what cut body`, `what common body` — a statement for each
   *  solid in `what`, in one splice (a Boolean gesture's tools). */
  addBodyWord(word: BodyWord, what: string[], body: string): Edit {
    return edit(
      withJson({ word, what, body }, (p, n) => core().gcs_elab_add_body_word(this.h, p, n)),
    );
  }

  /** The region of the drawing on `plane` (null the page) around `(x, y)` in its coordinates:
   *  the outer loop's edges and each hole's, by the names a face is written with, and their
   *  rings to wash — or null where no loop encloses it, or `{ refused }` with the cause. */
  regionAt(plane: Plane | null, x: number, y: number, unit: number): Region | { refused: string } | null {
    const i = plane ? plane.index : -1;
    return takeJson(core().gcs_elab_region_json(this.h, this.sketch.handle, i, x, y, unit));
  }

  /** Take out the statements that declare these, and every statement that named one.  An entity
   *  is anything with a `[kind, index]` ref — a proxy, or a solid, which has none. */
  remove(entities: Iterable<{ ref: readonly [string, number] }> = [],
         constraints: Iterable<Constraint> = []): Edit {
    const ents = [...entities].map((e) => e.ref);
    const cons = [...constraints].map((c) => c.id);
    return edit(
      // the live sketch goes with it: which constraints name an entity their text never
      // spells (a projection's planes) is the model's to say, and the elaboration's own
      // sketch was taken out into this one
      withJson({ entities: ents, constraints: cons }, (p, n) =>
        core().gcs_elab_remove(this.h, this.sketch.handle, p, n),
      ),
    );
  }

  /** Package the selection as a component of the document, drawn again by one instance of it:
   *  the statements that make it move into `component NAME(…) { … }`, what they read from
   *  outside handed in as formals, and what stays outside names them through the instance.
   *  `name` is the component's, else a fresh `Part`; the edit's `names` are the component and
   *  then the instance. */
  makeComponent(entities: Iterable<{ ref: readonly [string, number] }> = [],
                constraints: Iterable<Constraint> = [], name?: string): Edit {
    const ents = [...entities].map((e) => e.ref);
    const cons = [...constraints].map((c) => c.id);
    return edit(
      withJson({ entities: ents, constraints: cons, name: name ?? '' }, (p, n) =>
        core().gcs_elab_make_component(this.h, this.sketch.handle, p, n),
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
  return coloured(text, (p, n) => core().gcs_program_highlight(p, n));
}

/** What gave a symbol its name — the core's `syntax::SymbolKind`, by its word. */
export type SymbolKind = 'name' | 'param' | 'formal' | 'binder' | 'component' | 'word' | 'module'
  | 'import' | 'member' | 'unknown';

/** One thing a text names.  `parent` is what a member is read of (`l` for `l.p1`), else -1. */
export interface Symbol { name: string; kind: SymbolKind; parent: number }

/** One place the text names a symbol: `lo`/`hi` index the **string**, as a `Run`'s do; `def` where
 *  the name is given (declared, bound, a formal) rather than read. */
export interface Occurrence { lo: number; hi: number; symbol: number; def: boolean }

/** **The names a text gives and reads** (`syntax::symbols`) — what an editor's features stand on:
 *  the word under the caret, the declaration it names, and every other place it is named.  The
 *  core resolves every name; this only holds the answer, crossed onto the string, and looks it up.
 *
 *  A function of the *text*, as the colouring is: the program being looked at is usually the one
 *  half-typed, and its names are still worth lighting. */
export class Symbols {
  constructor(readonly symbols: Symbol[], readonly occurrences: Occurrence[]) {}

  /** The occurrence a caret at string index `at` is in, or stands just past — `rim|` means `rim`,
   *  as in every editor (the rule `Symbols::at` states in the core). */
  at(at: number): Occurrence | null {
    const os = this.occurrences;
    const o = os[partition(os.length, (k) => os[k].hi < at)];
    return o && o.lo <= at ? o : null;
  }

  /** Every place `symbol` is named, in text order. */
  of(symbol: number): Occurrence[] {
    return this.occurrences.filter((o) => o.symbol === symbol);
  }
}

/** Index a program's names.  An empty index where the text is past what the core reads. */
export function symbols(text: string): Symbols {
  type Wire = { symbols: [string, SymbolKind, number][];
    occurrences: [number, number, number, number][] };
  return crossed<Wire, Symbols>(text, (p, n) => core().gcs_program_symbols(p, n), (wire, at) =>
    new Symbols(
      wire.symbols.map(([name, kind, parent]) => ({ name, kind, parent })),
      wire.occurrences.map(([lo, hi, symbol, def]) =>
        ({ lo: at.at(lo), hi: at.at(hi), symbol, def: def !== 0 })),
    ));
}

/** A colouring entry's runs, crossed onto the string the browser holds — the program's here, and
 *  a drawing's (`drawing.highlight`) through the same seam. */
export function coloured(text: string, entry: (p: number, n: number) => number): Run[] {
  return crossed<[string, number, number][], Run[]>(text, entry, (runs, at) =>
    runs.map(([cls, lo, hi]) => ({ cls, lo: at.at(lo), hi: at.at(hi) })));
}

/** What an entry reading a text answers, its offsets crossed onto the string by `cross`. */
function crossed<W, R>(text: string, entry: (p: number, n: number) => number,
  cross: (wire: W, at: Offsets) => R): R {
  const h = withStr(text, entry);
  if (!h) throw new Error(lastError() || 'the text could not be read');
  return cross(takeJson<W>(h), new Offsets(text));
}

/** The first of `0..n` for which `before` is false, `before` holding of a prefix — where a sorted
 *  array is cut. */
export function partition(n: number, before: (k: number) => boolean): number {
  let lo = 0, hi = n;
  while (lo < hi) {
    const mid = (lo + hi) >> 1;
    if (before(mid)) lo = mid + 1;
    else hi = mid;
  }
  return lo;
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
 *  names, the diagnostics and the source map alike — so no consumer has to know the difference, and
 *  no two of them can disagree about which unit they hold.
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
    const lo = partition(this.ends.length, (k) => this.ends[k] <= off);
    return lo === 0 ? off : this.units[lo - 1] + (off - this.ends[lo - 1]);
  }
}

/** The canonical program for a sketch — the lift, and how a JSON document becomes a source one. */
export function fromSketch(sk: Sketch): string {
  return takeStr(core().gcs_sketch_to_program(sk.handle));
}

/** **A document's load scenarios being read a level at a time** (`report::Levels`): the drawing
 *  and its scenarios are copied in, so it outlives the document it was read from. */
export class ScenarioLevels {
  private constructor(private h: number) {}

  static create(doc: number, sk: number, drawn: number): ScenarioLevels {
    const h = core().gcs_elab_scenario_levels_new(doc, sk, drawn);
    if (!h) throw new Error(lastError() || 'could not start reading the scenarios');
    return new ScenarioLevels(h);
  }

  /** The next level read, or null once every level is; a reading the core could not make
   *  throws its error. */
  next(): Level | null {
    const p = core().gcs_scenario_levels_next(this.h);
    if (!p) throw new Error(lastError() || 'the scenarios could not be read');
    return takeJson<Level | null>(p) ?? null;
  }

  dispose(): void {
    if (this.h) core().gcs_scenario_levels_free(this.h);
    this.h = 0;
  }
}
