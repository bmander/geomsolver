/* JSON (de)serialization of sketches.
 *
 * Entities are referenced by [kind, index] into the sketch's ordered lists; constraints
 * serialize their constructor arguments per `Constraint.spec`.  Intrinsic constraints are
 * not stored — the primitives recreate them. */
import { CONSTRAINT_TYPES, Constraint, ENTITY_KINDS } from './constraints.js';
import { Kind, Primitive, Sketch, expand } from './model.js';

export type Ref = [Kind, number];
export interface SketchJSON {
  version: number;
  points: { x: number; y: number; fixed: boolean }[];
  lines: { p1: number; p2: number; construction: boolean }[];
  circles: { center: number; r: number; fixed: boolean; construction: boolean }[];
  arcs: { center: number; start: number; end: number; r: number; fixed: boolean; construction: boolean }[];
  constraints: { type: string; args: unknown[] }[];
  branches: Record<string, number>;
}

/** (kind, index) lookup by identity. */
export class Index {
  private of = new Map<Primitive, Ref>();

  constructor(sk: Sketch) {
    for (const kind of ['point', 'line', 'circle', 'arc'] as Kind[]) {
      sk.entities(kind).forEach((e, i) => this.of.set(e, [kind, i]));
    }
  }

  ref(e: Primitive): Ref {
    const r = this.of.get(e);
    if (!r) throw new Error('entity not in this sketch');
    return r;
  }

  name(e: Primitive): string {
    const [kind, i] = this.ref(e);
    return `${kind[0].toUpperCase()}${i}`;
  }
}

export function toJSON(sk: Sketch): SketchJSON {
  const ix = new Index(sk);
  return {
    version: 1,
    points: sk.points.map((p) => ({ x: p.x.value, y: p.y.value, fixed: p.isFixed })),
    lines: sk.lines.map((l) => ({ p1: ix.ref(l.p1)[1], p2: ix.ref(l.p2)[1], construction: l.construction })),
    circles: sk.circles.map((c) => ({
      center: ix.ref(c.center)[1], r: c.radius.value, fixed: c.radius.fixed, construction: c.construction,
    })),
    arcs: sk.arcs.map((a) => ({
      center: ix.ref(a.center)[1], start: ix.ref(a.start)[1], end: ix.ref(a.end)[1],
      r: a.radius.value, fixed: a.radius.fixed, construction: a.construction,
    })),
    // userConstraints() is exactly "what the user added": no intrinsic ones (the primitives
    // recreate those) and no soft ones (a drag target or a RadiusDrag's pull would come back
    // as a real dimension, since `soft` is not part of the JSON)
    constraints: sk.userConstraints().map((c) => ({
      type: c.typeName,
      args: c.args().map((v, i) => (ENTITY_KINDS.has(c.spec[i][1]) ? ix.ref(v as Primitive) : v)),
    })),
    branches: Object.fromEntries(sk.branches),
  };
}

export function fromJSON(d: SketchJSON): Sketch {
  const sk = new Sketch();
  d.points.forEach((p, i) => sk.point(p.x, p.y, !!p.fixed, `p${i}`));
  for (const l of d.lines) {
    const pair = l as unknown as [number, number];        // v1 stored a bare pair
    const ln = Array.isArray(pair) ? sk.line(sk.points[pair[0]], sk.points[pair[1]])
      : sk.line(sk.points[l.p1], sk.points[l.p2]);
    ln.construction = !Array.isArray(pair) && !!l.construction;
  }
  for (const c of d.circles) {
    const circ = sk.circle(sk.points[c.center], c.r);
    circ.radius.fixed = !!c.fixed;
    circ.construction = !!c.construction;
  }
  for (const a of d.arcs) {
    const arc = sk.arc(sk.points[a.center], sk.points[a.start], sk.points[a.end]);
    arc.radius.value = a.r;
    arc.radius.fixed = !!a.fixed;
    arc.construction = !!a.construction;
  }
  for (const c of d.constraints) {
    const T = CONSTRAINT_TYPES[c.type];
    if (!T) throw new Error(`unknown constraint type: ${c.type}`);
    const args = c.args.map((v, i) => {
      const kind = T.spec[i][1];
      if (!ENTITY_KINDS.has(kind)) return v;
      const [k, idx] = v as Ref;
      return sk.entities(k)[idx];
    });
    sk.add(new (T as unknown as new (...a: unknown[]) => Constraint)(...args));
  }
  sk.branches = new Map(Object.entries(d.branches ?? {}).map(([k, v]) => [k, Number(v)]));
  return sk;
}

export const dumps = (sk: Sketch, space = 1): string => JSON.stringify(toJSON(sk), null, space);
export const loads = (s: string): Sketch => fromJSON(JSON.parse(s) as SketchJSON);

/** Copy of the sketch with the given entities/constraints removed, plus everything that
 *  depends on a removed entity.  Deletion by rebuild — simple, and keeps Sketch's invariants
 *  trivially true. */
export function without(sk: Sketch, entities: Iterable<Primitive> = [], constraints: Iterable<Constraint> = []): Sketch {
  const dead = new Set(entities);
  const deadC = new Set(constraints);
  const alive = (e: Primitive): boolean => !dead.has(e) && !e.children.some((ch) => dead.has(ch));
  const tmp = new Sketch();
  tmp.points = sk.points.filter(alive);
  tmp.lines = sk.lines.filter(alive);
  tmp.circles = sk.circles.filter(alive);
  tmp.arcs = sk.arcs.filter(alive);
  tmp.constraints = sk.userConstraints().filter(
    (c) => !deadC.has(c) && !expand(c.entities()).some((e) => dead.has(e)),
  );
  return fromJSON(toJSON(tmp));
}

/** Human-readable one-liner: `Distance(P0, P1, 80)`; angles shown in degrees. */
export function describe(c: Constraint, ix: Index | Sketch): string {
  const index = ix instanceof Sketch ? new Index(ix) : ix;
  const parts = c.args().map((v, i) => {
    const kind = c.spec[i][1];
    if (ENTITY_KINDS.has(kind)) return index.name(v as Primitive);
    if (kind === 'angle') return `${fmt((v as number) * 180 / Math.PI, 3)}°`;
    if (kind === 'length' || kind === 'float') return fmt(v as number, 4);
    return String(v);
  });
  return `${c.typeName}(${parts.join(', ')})`;
}

/** Python's %g-style formatting: `sig` significant digits, trailing zeros dropped. */
export function fmt(v: number, sig = 4): string {
  if (!isFinite(v)) return String(v);
  if (v === 0) return '0';
  const a = Math.abs(v);
  if (a >= 10 ** sig || a < 1e-4) return Number(v.toPrecision(sig)).toExponential();
  return String(Number(v.toPrecision(sig)));
}
