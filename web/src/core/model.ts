/* Parameters, primitives and the Sketch container.
 *
 * Every scalar degree of freedom is a `Param`.  Primitives are thin bundles of Params;
 * the Sketch owns the ordered list of Params (its parameter vector) and the ordered list
 * of Constraints.  Ordering is deterministic by construction — insertion order, never
 * hashing — so identical edits give identical solves.
 */
import { Rng } from './rng.js';
import type { Constraint } from './constraints.js';

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
}

export class Line {
  readonly kind = 'line' as const;
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
}

export class Circle {
  readonly kind = 'circle' as const;
  constructor(readonly center: Point, readonly radius: Param) {}

  get children(): Point[] { return [this.center]; }
  get params(): Param[] { return [...this.center.params, this.radius]; }
}

/** CCW arc from `start` to `end` about `center`.  The radius is its own Param so Circle
 *  and Arc share every radius-based constraint; the two intrinsic constraints
 *  |start-center|^2 = r^2 and |end-center|^2 = r^2 are added by `Sketch.arc`. */
export class Arc {
  readonly kind = 'arc' as const;
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
}

export type Primitive = Point | Line | Circle | Arc;

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

  bbox(): [number, number, number, number] {
    if (!this.points.length) return [0, 0, 1, 1];
    let x0 = Infinity, y0 = Infinity, x1 = -Infinity, y1 = -Infinity;
    for (const p of this.points) {
      x0 = Math.min(x0, p.x.value); x1 = Math.max(x1, p.x.value);
      y0 = Math.min(y0, p.y.value); y1 = Math.max(y1, p.y.value);
    }
    return [x0, y0, x1, y1];
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
let arcIntrinsicFactory: PointOnCircleCtor | null = null;

export function registerArcIntrinsic(f: PointOnCircleCtor): void {
  arcIntrinsicFactory = f;
}
