/* JSON (de)serialization of sketches, and deletion by rebuild.
 *
 * The document format lives in the core; this module only moves strings. */
import { Constraint, ENTITY_KINDS } from './constraints.js';
import { Primitive, Sketch, expand } from './model.js';
import { core, lastError, takeStr, withJson, withStr } from './wasm.js';

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
