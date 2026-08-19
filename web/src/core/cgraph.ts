/* Constraint graph for decomposition (Stage 3).
 *
 * Fudos-Hoffmann work on geometric elements with 2 DOF each in the plane — points and
 * (infinite) lines — joined by valency-1 constraints: point-point distance, point-line
 * signed distance (0 = incidence), line-line angle.  This module maps a Sketch onto that:
 *
 *  * points are contracted by Coincident (one element per equivalence class);
 *  * a Line becomes a line element the first time a supported constraint refers to it (its
 *    endpoints are incident); lines nobody refers to are passive and get no element;
 *  * a circle/arc whose radius is known (Radius, a fixed param, or an EqualRadius chain to
 *    a known one) contributes distance edges: point-on-circle -> dist(centre, point) = r,
 *    line tangency -> signed dist(centre, line) = +-r, arc-endpoint tangency -> a virtual
 *    radius line through centre and endpoint, perpendicular to the tangent line;
 *  * angle-type constraints (Horizontal/Vertical against a ground x-axis, Parallel,
 *    Perpendicular, Angle) are direction relations, not rigid pairs — they contribute one
 *    equation when clusters merge.  Fixed points and the x-axis form the ground elements;
 *  * everything else is listed as `unsupported` and left to the numeric residual step.
 */
import * as C from './constraints.js';
import { Constraint } from './constraints.js';
import { UnionFind } from './graph.js';
import { Arc, Circle, Line, Param, Point, Sketch } from './model.js';

export type ElKind = 'P' | 'L' | 'V';
/** A geometric element, encoded as "kind:index" so it can key a Map cheaply. */
export type El = string;

export const el = (kind: ElKind, idx: number): El => `${kind}:${idx}`;
export const elKind = (e: El): ElKind => e[0] as ElKind;
export const elIdx = (e: El): number => Number(e.slice(2));
/** Python's (kind, idx) tuple order: L < P < V, then by index. */
export const cmpEl = (a: El, b: El): number =>
  a[0] === b[0] ? elIdx(a) - elIdx(b) : (a[0] < b[0] ? -1 : 1);
export const sortEls = (es: Iterable<El>): El[] => [...es].sort(cmpEl);

/** Ground line y = 0 (normal (0,1), c = 0). */
export const X_AXIS: El = el('L', -1);

/** A valency-1 constraint between two elements: 'PP' distance, 'PL' signed distance
 *  (0 = incidence).  `value()` reads the constraint on every execution, so edits and drags
 *  replay without recompiling. */
export interface Edge {
  kind: 'PP' | 'PL';
  a: El;
  b: El;
  value: () => number;
  source: Constraint | null;   /* null for implicit edges (line endpoints, virtual radii) */
}

/** dir(b) = dir(a) + phi (normals: n_b = rot(phi) n_a).  phi is the branch (mod pi) nearest
 *  the current geometry at build time — a chirality-like choice. */
export interface DirRelation {
  a: El;
  b: El;
  phi: number;
  source: Constraint;
}

export class ConstraintGraph {
  pointOf = new Map<Point, number>();
  members: Point[][] = [];
  lines: Line[] = [];
  lineOf = new Map<Line, number>();
  virtual: [El, El][] = [];
  edges: Edge[] = [];
  dirs: DirRelation[] = [];
  unsupported: Constraint[] = [];
  groundPoints: number[] = [];
  knownRadius = new Map<Param, number>();
  passive: Line[] = [];
  readonly X_AXIS = X_AXIS;
  private pointIndexCache: Map<Point, number> | null = null;

  constructor(readonly sketch: Sketch) {}

  P(p: Point): El {
    return el('P', this.pointOf.get(p)!);
  }

  /** Line element for `ln`, registered on first use. */
  L(ln: Line): El {
    let i = this.lineOf.get(ln);
    if (i === undefined) {
      i = this.lines.length;
      this.lineOf.set(ln, i);
      this.lines.push(ln);
    }
    return el('L', i);
  }

  /** Document-stable index of a point element: the smallest sketch index among the Points of
   *  its coincidence class (element numbering itself depends on how the graph was built). */
  pointIndex(e: El): number {
    if (!this.pointIndexCache) {
      this.pointIndexCache = new Map();
      this.sketch.points.forEach((p, i) => this.pointIndexCache!.set(p, i));
    }
    let best = Infinity;
    for (const p of this.members[elIdx(e)]) best = Math.min(best, this.pointIndexCache.get(p)!);
    return best;
  }

  /** Line element through two point elements (an arc's radius at an endpoint). */
  virtualLine(a: El, b: El): El {
    this.virtual.push([a, b]);
    return el('V', this.virtual.length - 1);
  }

  get nPoints(): number {
    return this.members.length;
  }

  get elements(): El[] {
    return [
      ...this.members.map((_, i) => el('P', i)),
      ...this.lines.map((_, i) => el('L', i)),
      ...this.virtual.map((_, i) => el('V', i)),
      X_AXIS,
    ];
  }

  summary(): string {
    return `${this.nPoints} point elements, ${this.lines.length} lines (+${this.passive.length} passive, `
      + `${this.virtual.length} virtual), ${this.edges.length} edges, ${this.dirs.length} direction relations `
      + `(${this.unsupported.length} unsupported constraints), ${this.groundPoints.length} ground points`;
  }
}

/** Points contracted by Coincident: (Point -> class index, class -> member Points). */
export function coincidentClasses(sketch: Sketch): { of: Map<Point, number>; members: Point[][] } {
  const pts = sketch.points;
  const idx = new Map<Point, number>();
  pts.forEach((p, i) => idx.set(p, i));
  const uf = new UnionFind(pts.length);
  for (const c of sketch.constraints)
    if (c instanceof C.Coincident) uf.union(idx.get(c.p)!, idx.get(c.q)!);
  const { label, count } = uf.labels();
  const members: Point[][] = Array.from({ length: count }, () => []);
  const of = new Map<Point, number>();
  pts.forEach((p, i) => { members[label[i]].push(p); of.set(p, label[i]); });
  return { of, members };
}

/** Radius Param -> value for every radius fixed, dimensioned by Radius, or EqualRadius-chained
 *  to one. */
export function knownRadii(sketch: Sketch): Map<Param, number> {
  const rounds: (Circle | Arc)[] = [...sketch.circles, ...sketch.arcs];
  const radii = rounds.map((c) => c.radius);
  const ridx = new Map<Param, number>();
  radii.forEach((r, i) => ridx.set(r, i));
  const uf = new UnionFind(radii.length);
  for (const c of sketch.constraints)
    if (c instanceof C.EqualRadius) uf.union(ridx.get(c.c1.radius)!, ridx.get(c.c2.radius)!);
  const known = new Map<number, number>();
  for (const c of sketch.constraints)          // after all unions, so class roots are final
    if (c instanceof C.Radius) known.set(uf.find(ridx.get(c.circle.radius)!), c.r);
  for (const r of radii) {
    const root = uf.find(ridx.get(r)!);
    if (r.fixed && !known.has(root)) known.set(root, r.value);
  }
  const out = new Map<Param, number>();
  for (const r of radii) {
    const root = uf.find(ridx.get(r)!);
    if (known.has(root)) out.set(r, known.get(root)!);
  }
  return out;
}

export function build(sketch: Sketch): ConstraintGraph {
  const g = new ConstraintGraph(sketch);
  const cc = coincidentClasses(sketch);
  g.pointOf = cc.of;
  g.members = cc.members;
  g.groundPoints = g.members.map((ms, k) => (ms.some((p) => p.isFixed) ? k : -1)).filter((k) => k >= 0);
  g.knownRadius = knownRadii(sketch);
  const kr = g.knownRadius;
  const rad = (c: Circle | Arc): number | undefined => kr.get(c.radius);
  const zero = () => 0;

  for (const c of sketch.constraints) {
    if (c.soft || c instanceof C.Coincident || c instanceof C.Radius) continue;  // contracted / absorbed
    if (c instanceof C.Distance) {
      g.edges.push({ kind: 'PP', a: g.P(c.p), b: g.P(c.q), value: () => c.d, source: c });
    } else if (c instanceof C.PointOnLine) {
      g.edges.push({ kind: 'PL', a: g.P(c.p), b: g.L(c.line), value: zero, source: c });
    } else if (c instanceof C.Horizontal) {
      g.dirs.push({ a: X_AXIS, b: g.L(c.line), phi: branch([0, 1], lineNormal(c.line), 0), source: c });
    } else if (c instanceof C.Vertical) {
      g.dirs.push({ a: X_AXIS, b: g.L(c.line), phi: branch([0, 1], lineNormal(c.line), Math.PI / 2), source: c });
    } else if (c instanceof C.Parallel || c instanceof C.Perpendicular || c instanceof C.Angle) {
      const target = c instanceof C.Parallel ? 0
        : c instanceof C.Perpendicular ? Math.PI / 2 : (c as C.Angle).theta;
      g.dirs.push({
        a: g.L(c.l1), b: g.L(c.l2),
        phi: branch(lineNormal(c.l1), lineNormal(c.l2), target), source: c,
      });
    } else if (c instanceof C.PointOnCircle && rad(c.circle) !== undefined) {
      const r = c.circle.radius;
      g.edges.push({ kind: 'PP', a: g.P(c.circle.center), b: g.P(c.p), value: () => kr.get(r)!, source: c });
    } else if (c instanceof C.TangentLineCircle && rad(c.circle) !== undefined) {
      // signed distance from centre to line = side*r (our line normal n = (-dy, dx)/|d|
      // gives n.(c - a) = cross(d, c - a)/|d|, which is the kernel's residual)
      const r = c.circle.radius;
      const side = c.side;
      g.edges.push({ kind: 'PL', a: g.P(c.circle.center), b: g.L(c.line), value: () => side * kr.get(r)!, source: c });
    } else if (c instanceof C.TangentArcLine && rad(c.arc) !== undefined) {
      // tangent at endpoint p <=> radius c-p perpendicular to the line (p on the line and
      // |c - p| = r come from elsewhere)
      const pt = c.at === 'start' ? c.arc.start : c.arc.end;
      const cen = g.P(c.arc.center), pe = g.P(pt);
      const R = g.virtualLine(cen, pe);
      g.edges.push({ kind: 'PL', a: cen, b: R, value: zero, source: null });
      g.edges.push({ kind: 'PL', a: pe, b: R, value: zero, source: null });
      const nR = normalOf(c.arc.center.x.value, c.arc.center.y.value, pt.x.value, pt.y.value);
      g.dirs.push({ a: g.L(c.line), b: R, phi: branch(lineNormal(c.line), nR, Math.PI / 2), source: c });
    } else if (c instanceof C.EqualRadius && rad(c.c1) !== undefined) {
      // absorbed into the known radii
    } else {
      g.unsupported.push(c);
    }
  }
  // endpoints lie on their (registered) line — implicit incidences; the rest are passive
  for (const ln of [...g.lines]) {
    for (const p of [ln.p1, ln.p2]) g.edges.push({ kind: 'PL', a: g.P(p), b: g.L(ln), value: zero, source: null });
  }
  g.passive = sketch.lines.filter((ln) => !g.lineOf.has(ln));
  return g;
}

/* -- line helpers ----------------------------------------------------------- */

/** (nx, ny, c) for the line a->b: n is the unit left normal and n.X = c on the line. */
export function normalOf(ax: number, ay: number, bx: number, by: number): [number, number, number] {
  const dx = bx - ax, dy = by - ay;
  const L = Math.hypot(dx, dy) || 1;
  const nx = -dy / L, ny = dx / L;
  return [nx, ny, nx * ax + ny * ay];
}

export function lineNormal(ln: Line): [number, number, number] {
  return normalOf(ln.p1.x.value, ln.p1.y.value, ln.p2.x.value, ln.p2.y.value);
}

/** Angle from normal n1 to normal n2 on the branch of `target` (mod pi) nearest the current
 *  geometry. */
export function branch(n1: readonly number[], n2: readonly number[], target: number): number {
  const cur = Math.atan2(n1[0] * n2[1] - n1[1] * n2[0], n1[0] * n2[0] + n1[1] * n2[1]);
  const k = Math.round((cur - target) / Math.PI);
  return target + k * Math.PI;
}

/** IEEE remainder: the value r with |r| <= |y|/2 and x - r a multiple of y. */
export function remainder(x: number, y: number): number {
  const q = Math.round(x / y);
  return x - q * y;
}
