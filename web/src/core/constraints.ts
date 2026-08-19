/* Constraint types: entities -> a local parameter tuple, constants and a kernel.
 *
 * Each class declares
 *   * `kernelId` — its vectorized residual/Jacobian kernel in the C core,
 *   * `params`   — the ordered Params the kernel's columns refer to,
 *   * `consts()` — the per-constraint constants (dimension values, chirality flags),
 *   * `spec`     — its constructor arguments as (attribute, kind) pairs; serialization,
 *                  the constraint list, value editing and the toolbar applier all read it.
 *
 * Residual forms follow the program: distance uses |p-q|^2 - d^2 (no sqrt), parallel is a
 * 2x2 determinant, angle a dot/cross combination, tangency a signed distance minus the
 * radius with a chirality flag fixed at construction.
 */
import { K, KERNELS } from './kernels.js';
import { Arc, Circle, Line, Param, Point, registerArcIntrinsic, registerPerpendicular } from './model.js';

export type SpecKind =
  | 'point' | 'line' | 'circle' | 'arc' | 'circle_or_arc'
  | 'length' | 'angle' | 'float' | 'int' | 'str' | 'bool';

export const ENTITY_KINDS: ReadonlySet<string> = new Set(['point', 'line', 'circle', 'arc', 'circle_or_arc']);
export const DIMENSION_KINDS: ReadonlySet<string> = new Set(['length', 'angle']);

export type Spec = readonly (readonly [string, SpecKind])[];
export type Entity = Point | Line | Circle | Arc;

export abstract class Constraint {
  abstract readonly kernelId: K;
  params: Param[] = [];
  soft = false;
  /** Implied by a primitive's definition (an arc's endpoints sit at its radius). */
  intrinsic = false;

  get spec(): Spec {
    return (this.constructor as typeof Constraint & { spec: Spec }).spec;
  }

  get typeName(): string {
    return (this.constructor as { ctorName?: string } & Function).name;
  }

  get nResiduals(): number {
    return KERNELS[this.kernelId].nRes;
  }

  consts(): number[] {
    return [];
  }

  /** Entities this constraint references directly, in spec order. */
  entities(): Entity[] {
    const self = this as unknown as Record<string, unknown>;
    return this.spec.filter(([, k]) => ENTITY_KINDS.has(k)).map(([n]) => self[n] as Entity);
  }

  /** Constructor arguments in spec order (round-trips through `new Type(...args)`). */
  args(): unknown[] {
    const self = this as unknown as Record<string, unknown>;
    return this.spec.map(([n]) => self[n]);
  }

  localValues(): Float64Array {
    const v = new Float64Array(this.params.length);
    for (let i = 0; i < this.params.length; i++) v[i] = this.params[i].value;
    return v;
  }

  /** The (attribute, kind) pairs of this constraint's dimension values. */
  dimensions(): (readonly [string, SpecKind])[] {
    return this.spec.filter(([, k]) => DIMENSION_KINDS.has(k));
  }
}

/* -- point / point ---------------------------------------------------------- */

export class Coincident extends Constraint {
  readonly kernelId = K.Coincident;
  static readonly spec: Spec = [['p', 'point'], ['q', 'point']];
  constructor(readonly p: Point, readonly q: Point) {
    super();
    this.params = [...p.params, ...q.params];
  }
}

/** |p - q|^2 - d^2 = 0. */
export class Distance extends Constraint {
  readonly kernelId = K.Distance;
  static readonly spec: Spec = [['p', 'point'], ['q', 'point'], ['d', 'length']];
  d: number;
  constructor(readonly p: Point, readonly q: Point, d: number) {
    super();
    this.d = d;
    this.params = [...p.params, ...q.params];
  }
  override consts(): number[] { return [this.d]; }
}

export class Midpoint extends Constraint {
  readonly kernelId = K.Midpoint;
  static readonly spec: Spec = [['p', 'point'], ['line', 'line']];
  constructor(readonly p: Point, readonly line: Line) {
    super();
    this.params = [...p.params, ...line.params];
  }
}

/** Soft constraint pulling `p` toward a (mutable) target; the drag. */
export class DragTarget extends Constraint {
  readonly kernelId = K.Drag;
  static readonly spec: Spec = [['p', 'point'], ['tx', 'float'], ['ty', 'float'], ['weight', 'float']];
  tx: number;
  ty: number;
  weight: number;
  constructor(readonly p: Point, tx: number, ty: number, weight = 1.0) {
    super();
    this.tx = tx; this.ty = ty; this.weight = weight;
    this.soft = true;
    this.params = [...p.params];
  }
  setTarget(tx: number, ty: number): void { this.tx = tx; this.ty = ty; }
  override consts(): number[] { return [this.tx, this.ty, this.weight]; }
}

/* -- line orientation ------------------------------------------------------- */

export class Horizontal extends Constraint {
  readonly kernelId = K.Horizontal;
  static readonly spec: Spec = [['line', 'line']];
  constructor(readonly line: Line) {
    super();
    this.params = [...line.params];
  }
}

export class Vertical extends Constraint {
  readonly kernelId = K.Vertical;
  static readonly spec: Spec = [['line', 'line']];
  constructor(readonly line: Line) {
    super();
    this.params = [...line.params];
  }
}

abstract class TwoLine extends Constraint {
  static readonly spec: Spec = [['l1', 'line'], ['l2', 'line']];
  constructor(readonly l1: Line, readonly l2: Line) {
    super();
    this.params = [...l1.params, ...l2.params];
  }
}

/** d1 x d2 = 0. */
export class Parallel extends TwoLine {
  readonly kernelId = K.Parallel;
  static override readonly spec: Spec = [['l1', 'line'], ['l2', 'line']];
}

/** d1 . d2 = 0. */
export class Perpendicular extends TwoLine {
  readonly kernelId = K.Perpendicular;
  static override readonly spec: Spec = [['l1', 'line'], ['l2', 'line']];
}

/** CCW angle from l1 to l2 equals theta (mod pi): dot*sin - cross*cos = 0. */
export class Angle extends TwoLine {
  readonly kernelId = K.Angle;
  static override readonly spec: Spec = [['l1', 'line'], ['l2', 'line'], ['theta', 'angle']];
  theta: number;
  constructor(l1: Line, l2: Line, theta: number) {
    super(l1, l2);
    this.theta = theta;
  }
  override consts(): number[] { return [Math.sin(this.theta), Math.cos(this.theta)]; }
}

/** |d1|^2 - |d2|^2 = 0. */
export class EqualLength extends TwoLine {
  readonly kernelId = K.EqualLength;
  static override readonly spec: Spec = [['l1', 'line'], ['l2', 'line']];
}

/* -- incidence -------------------------------------------------------------- */

/** (b-a) x (p-a) = 0. */
export class PointOnLine extends Constraint {
  readonly kernelId = K.PointOnLine;
  static readonly spec: Spec = [['p', 'point'], ['line', 'line']];
  constructor(readonly p: Point, readonly line: Line) {
    super();
    this.params = [...p.params, ...line.params];
  }
}

/** |p - c|^2 - r^2 = 0. */
export class PointOnCircle extends Constraint {
  readonly kernelId = K.PointOnCircle;
  static readonly spec: Spec = [['p', 'point'], ['circle', 'circle_or_arc']];
  constructor(readonly p: Point, readonly circle: Circle | Arc, intrinsic = false) {
    super();
    this.params = [...p.params, ...circle.center.params, circle.radius];
    this.intrinsic = intrinsic;
  }
}

/* -- radii ------------------------------------------------------------------ */

export class Radius extends Constraint {
  readonly kernelId = K.Radius;
  static readonly spec: Spec = [['circle', 'circle_or_arc'], ['r', 'length']];
  r: number;
  constructor(readonly circle: Circle | Arc, r: number) {
    super();
    this.r = r;
    this.params = [circle.radius];
  }
  override consts(): number[] { return [this.r]; }
}

export class EqualRadius extends Constraint {
  readonly kernelId = K.EqualRadius;
  static readonly spec: Spec = [['c1', 'circle_or_arc'], ['c2', 'circle_or_arc']];
  constructor(readonly c1: Circle | Arc, readonly c2: Circle | Arc) {
    super();
    this.params = [c1.radius, c2.radius];
  }
}

/* -- tangency --------------------------------------------------------------- */

/** Signed distance from the centre to the line equals +-r.  `side` is a chirality flag;
 *  when omitted it is read off the current geometry, so the solver keeps the circle on the
 *  side it already is. */
export class TangentLineCircle extends Constraint {
  readonly kernelId = K.TangentLineCircle;
  static readonly spec: Spec = [['line', 'line'], ['circle', 'circle_or_arc'], ['side', 'int']];
  side: number;
  constructor(readonly line: Line, readonly circle: Circle | Arc, side: number | null = null) {
    super();
    this.params = [...line.params, ...circle.center.params, circle.radius];
    if (side === null || side === undefined) {
      const v = this.localValues();
      const dx = v[2] - v[0], dy = v[3] - v[1], wx = v[4] - v[0], wy = v[5] - v[1];
      side = dx * wy - dy * wx >= 0 ? 1 : -1;
    }
    this.side = side;
  }
  override consts(): number[] { return [this.side]; }
}

/** |c1 - c2|^2 - (r1 +- r2)^2 = 0 (external: +, internal: -). */
export class TangentCircleCircle extends Constraint {
  readonly kernelId = K.TangentCircleCircle;
  static readonly spec: Spec = [['c1', 'circle_or_arc'], ['c2', 'circle_or_arc'], ['external', 'bool']];
  external: boolean;
  constructor(readonly c1: Circle | Arc, readonly c2: Circle | Arc, external = true) {
    super();
    this.external = external;
    this.params = [...c1.center.params, c1.radius, ...c2.center.params, c2.radius];
  }
  override consts(): number[] { return [this.external ? 1 : -1]; }
}

/** The line is tangent to the arc at the arc's `at` endpoint: (p - c).(b - a) = 0.
 *  Pair this with a Coincident between the arc endpoint and a line endpoint (the fillet). */
export class TangentArcLine extends Constraint {
  readonly kernelId = K.TangentArcLine;
  static readonly spec: Spec = [['arc', 'arc'], ['line', 'line'], ['at', 'str']];
  constructor(readonly arc: Arc, readonly line: Line, readonly at: 'start' | 'end') {
    super();
    const p = at === 'start' ? arc.start : arc.end;
    this.params = [...p.params, ...arc.center.params, ...line.params];
  }
}

/** p and q mirror each other across `line`: their midpoint is on it and p->q crosses it at a
 *  right angle.  Two residuals, and the line itself is free to move. */
export class Symmetric extends Constraint {
  readonly kernelId = K.Symmetric;
  static readonly spec: Spec = [['p', 'point'], ['q', 'point'], ['line', 'line']];
  constructor(readonly p: Point, readonly q: Point, readonly line: Line) {
    super();
    this.params = [...p.params, ...q.params, ...line.params];
  }
}

/* -- registry --------------------------------------------------------------- */

export type ConstraintCtor = (new (...args: never[]) => Constraint) & { spec: Spec };

export const CONSTRAINT_TYPES: Record<string, ConstraintCtor> = {
  Coincident, Distance, Midpoint, DragTarget, Horizontal, Vertical, Parallel, Perpendicular,
  Angle, EqualLength, PointOnLine, PointOnCircle, Radius, EqualRadius, TangentLineCircle,
  TangentCircleCircle, TangentArcLine, Symmetric,
} as unknown as Record<string, ConstraintCtor>;

registerArcIntrinsic((p, circle, intrinsic) => new PointOnCircle(p, circle, intrinsic));
registerPerpendicular((l1, l2) => new Perpendicular(l1, l2));
