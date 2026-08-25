/* JSON (de)serialization of sketches, and the rebuild operations: deletion, copy, paste.
 *
 * The document format lives in the core; this module only moves strings. */
import { Constraint, ENTITY_KINDS } from './constraints.js';
import { Primitive, Sketch, expand } from './model.js';
import { core, lastError, takeJson, takeStr, withJson, withStr } from './wasm.js';

export type Ref = [string, number];

export interface SketchJSON {
  version: number;
  points: { x: number; y: number; fixed: boolean }[];
  lines: { p1: number; p2: number; construction: boolean }[];
  circles: { center: number; r: number; fixed: boolean; construction: boolean }[];
  arcs: { center: number; start: number; end: number; r: number; fixed: boolean; construction: boolean }[];
  constraints: { type: string; args: unknown[] }[];
  branches: Record<string, number>;
}

export function dumps(sk: Sketch, space = -1): string {
  return takeStr(core().gcs_sketch_to_json(sk.handle, space));
}

export function loads(s: string): Sketch {
  const h = withStr(s, (p, n) => core().gcs_sketch_from_json(p, n));
  if (!h) throw new Error(lastError() || 'bad sketch document');
  return new Sketch(h);
}

export function toJSON(sk: Sketch): SketchJSON {
  return JSON.parse(dumps(sk)) as SketchJSON;
}

export function fromJSON(d: SketchJSON): Sketch {
  return loads(JSON.stringify(d));
}

/** Copy of the sketch with the given entities/constraints removed, plus everything that depends
 *  on a removed entity.  Deletion by rebuild — simple, and keeps the invariants trivially true. */
export function without(sk: Sketch, entities: Iterable<Primitive> = [],
                        constraints: Iterable<Constraint> = []): Sketch {
  const ents = [...entities].map((e) => e.ref);
  const cids = [...constraints].filter((c) => c.id >= 0).map((c) => c.id);
  return withJson(ents, (ep, en) => withJson(cids, (cp, cn) =>
    new Sketch(core().gcs_without(sk.handle, ep, en, cp, cn))));
}

/** The selection as a sketch of its own: the entities picked, the points that define them and
 *  the constraints all of whose entities came along.  It is an ordinary sketch, so a clipboard
 *  is a document — `dumps` it and it saves, `loads` it and it pastes. */
export function copy(sk: Sketch, entities: Iterable<Primitive>): Sketch {
  const ents = [...entities].map((e) => e.ref);
  return withJson(ents, (p, n) => new Sketch(core().gcs_copy(sk.handle, p, n)));
}

/** Add everything in `clip` to `sk`, moved by (dx, dy), and return what that made — so the
 *  caller can select the copy it just pasted. */
export function paste(sk: Sketch, clip: Sketch, dx: number, dy: number): Primitive[] {
  const made = takeJson<Ref[]>(core().gcs_paste(sk.handle, clip.handle, dx, dy)) ?? [];
  sk.touch();   // the pasted constraints arrived behind the proxy's back
  const of: Record<string, () => Primitive[]> = {
    point: () => sk.points, line: () => sk.lines, circle: () => sk.circles, arc: () => sk.arcs,
  };
  return made.flatMap(([kind, i]) => {
    const e = of[kind]?.()[i];
    return e ? [e] : [];
  });
}

/** (kind, index) lookup — the short entity labels the lists and reports use. */
export class Index {
  constructor(_sk?: Sketch) {}

  ref(e: Primitive): Ref {
    return e.ref;
  }

  name(e: Primitive): string {
    return e.name;
  }
}

/** Human-readable one-liner: `Distance(P0, P1, 80)`; angles shown in degrees. */
export function describe(c: Constraint, _ix?: Index | Sketch): string {
  return c.describe();
}

/** Python's %g-style formatting: `sig` significant digits, trailing zeros dropped. */
export function fmt(v: number, sig = 4): string {
  return takeStr(core().gcs_fmt_g(v, sig));
}

export { ENTITY_KINDS, expand };

/* -- Solvent: the program a sketch is written as ---------------------------------- */

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

/** What a program came to: the drawing it makes, what was wrong with it, and where each part of
 *  it was written.  `sketch` is null only when nothing at all could be built. */
export interface Elaboration {
  sketch: Sketch | null;
  ok: boolean;
  diagnostics: Diagnostic[];
  map: SourceMap;
}

/** The canonical program for a sketch. */
export function toProgram(sk: Sketch): string {
  return takeStr(core().gcs_sketch_to_program(sk.handle));
}

/** Read a program and elaborate it.
 *
 *  Never throws: a program with one bad line still comes back with the other twenty drawn and the
 *  diagnostics beside them, because a panel has to show the drawing *and* the error.  Whether to
 *  adopt the result is the caller's, from `ok`. */
export function fromProgram(text: string): Elaboration {
  const c = core();
  const h = withStr(text, (p, n) => c.gcs_program_elaborate(p, n));
  if (!h) throw new Error(lastError() || 'could not read the program');
  try {
    const r = takeJson<{
      ok: boolean;
      diagnostics: Diagnostic[];
      entities: [string, number, string, number, number][];
      constraints: [number, number, number][];
    }>(c.gcs_elab_report(h));
    const sh = c.gcs_elab_take_sketch(h);
    return {
      sketch: sh ? new Sketch(sh) : null,
      ok: r.ok,
      diagnostics: r.diagnostics,
      map: {
        entities: r.entities.map(([kind, index, name, lo, hi]) => ({ kind, index, name, lo, hi })),
        constraints: r.constraints.map(([id, lo, hi]) => ({ id, lo, hi })),
      },
    };
  } finally {
    c.gcs_elab_free(h);
  }
}
