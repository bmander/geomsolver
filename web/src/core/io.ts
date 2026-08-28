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
  lines: { p1: number; p2: number; class: string[] }[];
  circles: { center: number; r: number; fixed: boolean; class: string[] }[];
  arcs: { center: number; start: number; end: number; r: number; fixed: boolean; class: string[] }[];
  constraints: { type: string; args: unknown[] }[];
  branches: Record<string, number>;
  /** What the document's numbers are in — `null` for one in drawing units (Solvent §3.3). */
  unit: string | null;
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
