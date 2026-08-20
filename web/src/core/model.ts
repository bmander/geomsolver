/* Parameters, primitives and the Sketch container.
 *
 * Every scalar degree of freedom is a `Param`.  Primitives are thin bundles of Params;
 * the Sketch owns the ordered list of Params (its parameter vector) and the ordered list
 * of Constraints.  Ordering is deterministic by construction — insertion order, never
 * hashing — so identical edits give identical solves.
 */
import { Rng } from './rng.js';
import type { Constraint } from './constraints.js';

/** (xmin, ymin, xmax, ymax) */
export type Box = [number, number, number, number];

export class Param {
  constructor(
    public value: number,
    public fixed = false,
    public index = -1,
    public name = '',
  ) {}
}

export type Kind = 'point' | 'line' | 'circle' | 'arc';

export class Point {
  readonly kind = 'point' as const;
  constructor(readonly x: Param, readonly y: Param) {}

  get children(): Point[] { return []; }
  get params(): Param[] { return [this.x, this.y]; }
  get xy(): [number, number] { return [this.x.value, this.y.value]; }
  get isFixed(): boolean { return this.x.fixed && this.y.fixed; }

  fix(fixed = true): void {
    this.x.fixed = fixed;
    this.y.fixed = fixed;
  }

  bounds(): Box {
    return [this.x.value, this.y.value, this.x.value, this.y.value];
  }
}

export class Line {
  readonly kind = 'line' as const;
  /** Reference geometry: drawn dashed, constrains like any other. */
  construction = false;
  constructor(readonly p1: Point, readonly p2: Point) {}

  get children(): Point[] { return [this.p1, this.p2]; }
  get params(): Param[] { return [...this.p1.params, ...this.p2.params]; }

  direction(): [number, number] {
    return [this.p2.x.value - this.p1.x.value, this.p2.y.value - this.p1.y.value];
  }

  length(): number {
    const [dx, dy] = this.direction();
    return Math.hypot(dx, dy);
  }

  bounds(): Box {
    return [
      Math.min(this.p1.x.value, this.p2.x.value), Math.min(this.p1.y.value, this.p2.y.value),
      Math.max(this.p1.x.value, this.p2.x.value), Math.max(this.p1.y.value, this.p2.y.value),
    ];
  }
}

export class Circle {
  readonly kind = 'circle' as const;
  construction = false;
  constructor(readonly center: Point, readonly radius: Param) {}

  get children(): Point[] { return [this.center]; }
  get params(): Param[] { return [...this.center.params, this.radius]; }

  bounds(): Box {
    const [cx, cy] = this.center.xy;
    const r = Math.abs(this.radius.value);
    return [cx - r, cy - r, cx + r, cy + r];
  }
}

/** CCW arc from `start` to `end` about `center`.  The radius is its own Param so Circle
 *  and Arc share every radius-based constraint; the two intrinsic constraints
 *  |start-center|^2 = r^2 and |end-center|^2 = r^2 are added by `Sketch.arc`. */
export class Arc {
  readonly kind = 'arc' as const;
  construction = false;
  constructor(readonly center: Point, readonly start: Point, readonly end: Point, readonly radius: Param) {}

  get children(): Point[] { return [this.center, this.start, this.end]; }
  get params(): Param[] {
    return [...this.center.params, ...this.start.params, ...this.end.params, this.radius];
  }

  angles(): [number, number] {
    const [cx, cy] = this.center.xy;
    const a0 = Math.atan2(this.start.y.value - cy, this.start.x.value - cx);
    let a1 = Math.atan2(this.end.y.value - cy, this.end.x.value - cx);
    if (a1 <= a0) a1 += 2 * Math.PI;
    return [a0, a1];
  }

  /** The points that bound the drawn sweep: its two ends, plus every quarter-turn direction
   *  the sweep passes through.  Endpoints alone would under-report an arc that bulges past
   *  them. */
  extremes(): [number, number][] {
    const [cx, cy] = this.center.xy;
    const r = Math.abs(this.radius.value);
    const [a0, a1] = this.angles();
    const at = (th: number): [number, number] => [cx + r * Math.cos(th), cy + r * Math.sin(th)];
    const out: [number, number][] = [at(a0), at(a1)];
    const quarter = Math.PI / 2;
    for (let k = Math.ceil(a0 / quarter); k * quarter < a1; k++) out.push(at(k * quarter));
    return out;
  }

  bounds(): Box {
    let x0 = Infinity, y0 = Infinity, x1 = -Infinity, y1 = -Infinity;
    for (const [x, y] of this.extremes()) {
      x0 = Math.min(x0, x); y0 = Math.min(y0, y);
      x1 = Math.max(x1, x); y1 = Math.max(y1, y);
    }
    return [x0, y0, x1, y1];
  }
}

export type Primitive = Point | Line | Circle | Arc;

/** The CCW arc through three points: centre, radius, and the sweep from `a0` to `a1` that
 *  passes through the third point.  `swapped` is true when that sweep runs from the *second*
 *  given point to the first. */
export interface ThreePointArc {
  cx: number;
  cy: number;
  r: number;
  a0: number;
  a1: number;
  swapped: boolean;
}

/** Arc from (ax, ay) to (bx, by) passing through (cx, cy) — the circumcircle of the three,
 *  plus the sweep direction that actually contains the third point.  null if they are
 *  collinear (the test is on the sine of the angle, so it is scale-free). */
export function threePointArc(ax: number, ay: number, bx: number, by: number,
                              cx: number, cy: number, tol = 1e-9): ThreePointArc | null {
  const ux = bx - ax, uy = by - ay;
  const vx = cx - ax, vy = cy - ay;
  const cross = ux * vy - uy * vx;
  if (Math.abs(cross) <= tol * Math.hypot(ux, uy) * Math.hypot(vx, vy)) return null;
  const d = 2 * cross;
  const u2 = ux * ux + uy * uy, v2 = vx * vx + vy * vy;
  const ox = ax + (vy * u2 - uy * v2) / d;
  const oy = ay + (ux * v2 - vx * u2) / d;
  const r = Math.hypot(ax - ox, ay - oy);
  const ta = Math.atan2(ay - oy, ax - ox);
  const tb = Math.atan2(by - oy, bx - ox);
  const sweep = (th: number): number => ((th - ta) % (2 * Math.PI) + 2 * Math.PI) % (2 * Math.PI);
  const toB = sweep(tb), toC = sweep(Math.atan2(cy - oy, cx - ox));
  return toC < toB                                  // the third point is on the a -> b sweep
    ? { cx: ox, cy: oy, r, a0: ta, a1: ta + toB, swapped: false }
    : { cx: ox, cy: oy, r, a0: tb, a1: tb + (2 * Math.PI - toB), swapped: true };
}

/** Signed perpendicular offset from the *infinite* line through `ln`, positive to the left of
 *  its direction — the convention `PointLineDistance` and `ParallelDistance` use, so the UI can
 *  prefill their prompts with the value the sketch already shows, sign included.
 *
 *  A degenerate line has no side; it gives Infinity rather than a silent zero. */
export function signedPointToLine(px: number, py: number, ln: Line): number {
  const [ax, ay] = ln.p1.xy;
  const [dx, dy] = ln.direction();
  const length = Math.hypot(dx, dy);
  if (length === 0) return Infinity;
  return (dx * (py - ay) - dy * (px - ax)) / length;
}

/** Perpendicular distance from a point to the *infinite* line through `ln`. */
function pointToLine(px: number, py: number, ln: Line): number {
  const [dx, dy] = ln.direction();
  if (dx === 0 && dy === 0) return Math.hypot(px - ln.p1.x.value, py - ln.p1.y.value);
  return Math.abs(signedPointToLine(px, py, ln));
}

const MEASURE_ORDER: Record<Kind, number> = { point: 0, line: 1, circle: 2, arc: 2 };

const isRound = (e: Primitive): e is Circle | Arc => e instanceof Circle || e instanceof Arc;

/** Shortest distance between two entities, as a sketcher measures it.
 *
 *  Lines are treated as infinite — perpendicular distance to one, the gap between two
 *  parallel ones — because that is what dimensioning them means; two lines that cross are 0
 *  apart.  Arcs are measured as the whole circle they lie on, which is exact for the common
 *  cases and an over-estimate of the true gap only when the nearest approach is off the swept
 *  part. */
export function distanceBetween(first: Primitive, second: Primitive): number {
  // ordered so the simpler kind is `a`, which halves the cases and lets the checker narrow
  const [a, b] = MEASURE_ORDER[first.kind] > MEASURE_ORDER[second.kind]
    ? [second, first] : [first, second];
  if (a instanceof Point) {
    if (b instanceof Point) return Math.hypot(a.x.value - b.x.value, a.y.value - b.y.value);
    if (b instanceof Line) return pointToLine(a.x.value, a.y.value, b);
    if (!isRound(b)) return 0;
    const [cx, cy] = b.center.xy;
    return Math.abs(Math.hypot(a.x.value - cx, a.y.value - cy) - Math.abs(b.radius.value));
  }
  if (a instanceof Line) {
    if (b instanceof Line) {
      const [d1, d2] = [a.direction(), b.direction()];
      const cross = d1[0] * d2[1] - d1[1] * d2[0];
      if (Math.abs(cross) > 1e-9 * Math.hypot(...d1) * Math.hypot(...d2)) return 0;
      return pointToLine(b.p1.x.value, b.p1.y.value, a);
    }
    if (!isRound(b)) return 0;                               // ordered above: b is a round
    return Math.max(0, pointToLine(b.center.x.value, b.center.y.value, a) - Math.abs(b.radius.value));
  }
  return isRound(a) && isRound(b) ? roundGap(a, b) : 0;       // ordered above: both are rounds
}

/** Gap between two circles: outside each other, or one inside the other; overlapping give 0. */
function roundGap(a: Circle | Arc, b: Circle | Arc): number {
  const [ax, ay] = a.center.xy, [bx, by] = b.center.xy;
  const gap = Math.hypot(ax - bx, ay - by);
  const r1 = Math.abs(a.radius.value), r2 = Math.abs(b.radius.value);
  return Math.max(0, gap - r1 - r2, Math.abs(r1 - r2) - gap);
}

/** Entities plus their sub-entities (a line's endpoints, an arc's centre and ends). */
export function expand(ents: Iterable<Primitive>): Primitive[] {
  const out: Primitive[] = [];
  for (const e of ents) {
    out.push(e);
    out.push(...e.children);
  }
  return out;
}

export class Sketch {
  params: Param[] = [];
  constraints: Constraint[] = [];
  points: Point[] = [];
  lines: Line[] = [];
  circles: Circle[] = [];
  arcs: Arc[] = [];
  /** Recorded root choices (Stage 5), persisted with the document. */
  branches: Map<string, number> = new Map();

  // -- construction -------------------------------------------------------

  param(value: number, fixed = false, name = ''): Param {
    const p = new Param(value, fixed, this.params.length, name);
    this.params.push(p);
    return p;
  }

  point(x: number, y: number, fixed = false, name = ''): Point {
    const pt = new Point(this.param(x, fixed, `${name}.x`), this.param(y, fixed, `${name}.y`));
    this.points.push(pt);
    return pt;
  }

  line(p1: Point, p2: Point): Line {
    const ln = new Line(p1, p2);
    this.lines.push(ln);
    return ln;
  }

  lineXY(x1: number, y1: number, x2: number, y2: number, name = ''): Line {
    return this.line(this.point(x1, y1, false, `${name}.p1`), this.point(x2, y2, false, `${name}.p2`));
  }

  circle(center: Point, radius: number, name = ''): Circle {
    const c = new Circle(center, this.param(radius, false, `${name}.r`));
    this.circles.push(c);
    return c;
  }

  arc(center: Point, start: Point, end: Point, name = '', PointOnCircleCls?: PointOnCircleCtor): Arc {
    const [cx, cy] = center.xy;
    const r = Math.hypot(start.x.value - cx, start.y.value - cy);
    const a = new Arc(center, start, end, this.param(r, false, `${name}.r`));
    this.arcs.push(a);
    const POC = PointOnCircleCls ?? arcIntrinsicFactory;
    if (!POC) throw new Error('Sketch.arc needs the PointOnCircle constructor');
    this.add(POC(start, a, true), POC(end, a, true));   // intrinsic: endpoints at the radius
    return a;
  }

  /** Arc from `start` to `end` bulging through `through` — the three-point construction.
   *  Creates the centre point; null if the three are collinear. */
  arcThrough(start: Point, end: Point, through: [number, number], name = ''): Arc | null {
    const g = threePointArc(...start.xy, ...end.xy, ...through);
    if (!g) return null;
    const centre = this.point(g.cx, g.cy, false, `${name}.c`);
    const [a, b] = g.swapped ? [end, start] : [start, end];
    return this.arc(centre, a, b, name);
  }

  /** Four lines round the corners (x0, y0) and (x1, y1), sharing corner points, with three
   *  perpendicular constraints.
   *
   *  Three, not four: the fourth follows (l3 ⟂ l2 ⟂ l1 ⟂ l0 already forces l3 ⟂ l0), so
   *  adding it would make every rectangle over-constrained by one equation.  What is left is
   *  the 5 DOF a rectangle has — position, rotation, width, height.  `a` is an existing
   *  point, so a rectangle can start on geometry that is already there. */
  rectangle(a: Point, x1: number, y1: number, name = ''): Line[] {
    const [x0, y0] = a.xy;
    const corners = [
      a, this.point(x1, y0, false, `${name}.b`),
      this.point(x1, y1, false, `${name}.c`), this.point(x0, y1, false, `${name}.d`),
    ];
    const lines = corners.map((c, i) => this.line(c, corners[(i + 1) % 4]));
    for (let i = 0; i < 3; i++) this.add(makePerpendicular(lines[i], lines[i + 1]));
    return lines;
  }

  /** `rectangle` starting from a fresh corner point — the `lineXY` of rectangles. */
  rectangleXY(x0: number, y0: number, x1: number, y1: number, name = ''): Line[] {
    return this.rectangle(this.point(x0, y0, false, `${name}.a`), x1, y1, name);
  }

  add(...constraints: Constraint[]): void {
    this.constraints.push(...constraints);
  }

  remove(constraint: Constraint): void {
    const i = this.constraints.indexOf(constraint);
    if (i >= 0) this.constraints.splice(i, 1);
  }

  // -- parameter vector ---------------------------------------------------

  getX(): Float64Array {
    const x = new Float64Array(this.params.length);
    for (let i = 0; i < this.params.length; i++) x[i] = this.params[i].value;
    return x;
  }

  setX(x: ArrayLike<number>): void {
    for (let i = 0; i < this.params.length; i++) this.params[i].value = x[i];
  }

  freeIndices(): Int32Array {
    const out: number[] = [];
    for (let i = 0; i < this.params.length; i++) {
      this.params[i].index = i;
      if (!this.params[i].fixed) out.push(i);
    }
    return Int32Array.from(out);
  }

  nResiduals(): number {
    return this.constraints.reduce((s, c) => s + c.nResiduals, 0);
  }

  /** Constraints the user added (excludes intrinsic and soft/transient ones). */
  userConstraints(): Constraint[] {
    return this.constraints.filter((c) => !(c.intrinsic || c.soft));
  }

  /** Everything that must be satisfied (excludes soft ones such as drag targets). */
  hardConstraints(): Constraint[] {
    return this.constraints.filter((c) => !c.soft);
  }

  entities(kind: Kind): Primitive[] {
    return kind === 'point' ? this.points : kind === 'line' ? this.lines
      : kind === 'circle' ? this.circles : this.arcs;
  }

  /** Every entity, in creation order per kind. */
  primitives(): Primitive[] {
    return [...this.points, ...this.lines, ...this.circles, ...this.arcs];
  }

  /** (xmin, ymin, xmax, ymax) over all points.
   *
   *  Points only, deliberately: `extent()` is built on this, and `extent()` scales the
   *  solver's residual tolerances (`System.scale = max(1, extent)²`), the violated-constraint
   *  threshold, the witness perturbation and the drag continuation step.  Folding radii in
   *  would loosen every tolerance quadratically on a circle-heavy sketch and coarsen the very
   *  drag increments Stage 5 uses to track a branch.  It costs little: the kernels that are
   *  quadratic in r all put a point on or against the curve, so the points already track the
   *  radius.  Only a lone circle with nothing on it falls back to extent 1 — the tightest
   *  tolerance, which errs safe.  For what is drawn, use `drawnBounds()`. */
  bbox(): Box {
    if (!this.points.length) return [0, 0, 1, 1];
    let x0 = Infinity, y0 = Infinity, x1 = -Infinity, y1 = -Infinity;
    for (const p of this.points) {
      x0 = Math.min(x0, p.x.value); x1 = Math.max(x1, p.x.value);
      y0 = Math.min(y0, p.y.value); y1 = Math.max(y1, p.y.value);
    }
    return [x0, y0, x1, y1];
  }

  /** Bounds of everything drawn, curves included — what a "fit the view" wants.  A circle or
   *  arc reaches past its centre, so a points-only box clips it. */
  drawnBounds(): Box {
    let x0 = Infinity, y0 = Infinity, x1 = -Infinity, y1 = -Infinity;
    let any = false;
    for (const e of this.primitives()) {
      const b = e.bounds();
      any = true;
      x0 = Math.min(x0, b[0]); y0 = Math.min(y0, b[1]);
      x1 = Math.max(x1, b[2]); y1 = Math.max(y1, b[3]);
    }
    return any ? [x0, y0, x1, y1] : this.bbox();
  }

  /** Seeded Gaussian noise on every free parameter (warm starts, witness construction). */
  perturb(sigma: number, seed = 0): void {
    const rng = new Rng(seed);
    for (const p of this.params) if (!p.fixed) p.value += rng.normal(0, sigma);
  }

  /** Characteristic length of the sketch (tolerances, drag weights). */
  extent(): number {
    const [x0, y0, x1, y1] = this.bbox();
    return Math.max(x1 - x0, y1 - y0, 1);
  }

  nearestPoint(x: number, y: number): { point: Point | null; dist: number } {
    let best: Point | null = null;
    let bd = Infinity;
    for (const p of this.points) {
      const d = Math.hypot(p.x.value - x, p.y.value - y);
      if (d < bd) { best = p; bd = d; }
    }
    return { point: best, dist: bd };
  }
}

/* `Sketch.arc` needs PointOnCircle, which lives in constraints.ts and imports this module
 * for its types.  Registering the constructor here keeps the runtime dependency one-way. */
type PointOnCircleCtor = (p: Point, circle: Circle | Arc, intrinsic: boolean) => Constraint;
type PerpendicularCtor = (l1: Line, l2: Line) => Constraint;
let arcIntrinsicFactory: PointOnCircleCtor | null = null;
let perpendicularFactory: PerpendicularCtor | null = null;

export function registerArcIntrinsic(f: PointOnCircleCtor): void {
  arcIntrinsicFactory = f;
}

export function registerPerpendicular(f: PerpendicularCtor): void {
  perpendicularFactory = f;
}

function makePerpendicular(l1: Line, l2: Line): Constraint {
  if (!perpendicularFactory) throw new Error('Sketch.rectangle needs the Perpendicular constructor');
  return perpendicularFactory(l1, l2);
}
