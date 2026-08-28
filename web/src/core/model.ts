/* Parameters, primitives and the Sketch container — proxies over the Rust model.
 *
 * A `Param` is an index, an entity is a `(kind, index)` pair and a constraint is a
 * document-stable id; the objects here are interned per sketch, so `===` means what it always
 * did while the data itself lives in the core.  Nothing is mirrored: every read goes through
 * the ABI.
 */
import { Buf, core, lastError, takeJson, takeStr, withBuf, withJson, withStr } from './wasm.js';
import type { Constraint } from './constraints.js';
// The constraint classes are generated from the core's registry and register themselves with
// `initCore`; loading them here means every consumer of the model gets them, in any import order.
import './constraints.js';

/** (xmin, ymin, xmax, ymax) */
export type Box = [number, number, number, number];

export type Kind = 'point' | 'line' | 'circle' | 'arc' | 'spline' | 'ellipse' | 'curve'
  | 'frame';
export const KINDS: Kind[] =
  ['point', 'line', 'circle', 'arc', 'spline', 'ellipse', 'curve', 'frame'];
export const KIND_ID: Record<Kind, number> =
  { point: 0, line: 1, circle: 2, arc: 3, spline: 4, ellipse: 5, curve: 6, frame: 7 };

export class Param {
  constructor(readonly sketch: Sketch, readonly index: number) {}

  get value(): number {
    return core().gcs_param_value(this.sketch.handle, this.index);
  }

  set value(v: number) {
    core().gcs_param_set_value(this.sketch.handle, this.index, v);
  }

  get fixed(): boolean {
    return core().gcs_param_fixed(this.sketch.handle, this.index) !== 0;
  }

  set fixed(v: boolean) {
    core().gcs_param_set_fixed(this.sketch.handle, this.index, v ? 1 : 0);
  }

  get name(): string {
    return takeStr(core().gcs_param_name(this.sketch.handle, this.index));
  }
}

export abstract class Entity {
  abstract readonly kind: Kind;

  constructor(readonly sketch: Sketch, readonly index: number) {}

  get kindId(): number {
    return KIND_ID[this.kind];
  }

  /** `['point', 3]` — the reference form the document and the ABI use. */
  get ref(): [Kind, number] {
    return [this.kind, this.index];
  }

  /** `P0` / `L3` / `C1` / `A2`. */
  get name(): string {
    return takeStr(core().gcs_entity_name(this.kindId, this.index));
  }

  /** How many indices `params`/`children` have to make room for.  Every kind but a spline has a
   *  width fixed by its shape; a spline's is its control polygon, so only it pays to ask. */
  protected get slots(): number {
    return 8;
  }

  get params(): Param[] {
    return withBuf(this.slots, 4, (b) => {
      const n = core().gcs_entity_params(this.sketch.handle, this.kindId, this.index, b.ptr);
      return [...b.i32.subarray(0, n)].map((i) => this.sketch.paramAt(i));
    });
  }

  get children(): Point[] {
    if (this.kind === 'point') return [];
    return withBuf(this.slots, 4, (b) => {
      const n = core().gcs_entity_points(this.sketch.handle, this.kindId, this.index, b.ptr);
      const pts = this.sketch.points;
      return [...b.i32.subarray(0, n)].map((i) => pts[i]);
    });
  }

  bounds(): Box {
    return withBuf(4, 8, (b) => {
      core().gcs_entity_bounds(this.sketch.handle, this.kindId, this.index, b.ptr);
      const v = b.f64;
      return [v[0], v[1], v[2], v[3]] as Box;
    });
  }
}

/** What an entity is drawn with: dash pattern, stroke weight and ink, all in **screen pixels**
 *  — a dashed line does not change its dash pattern when you zoom.  Resolved in the core from
 *  the document's style sheet; nothing here knows what a class is. */
export interface Style {
  dash: number[];
  width: number | null;
  color: string | null;
}

/** Everything with a stroke carries *classes*, and a style sheet says what a class looks like.
 *  Presentation, and nothing the core computes reads it: the binding surfaces it and the app
 *  strokes it. */
abstract class Styled extends Entity {
  /** The classes it carries, in written order — later wins on a conflicting property. */
  get classes(): string[] {
    return takeJson<string[]>(
      core().gcs_entity_class(this.sketch.handle, this.kindId, this.index),
    );
  }

  hasClass(name: string): boolean {
    return this.classes.includes(name);
  }

  setClass(name: string, on: boolean): void {
    withStr(name, (p, n) =>
      core().gcs_entity_set_class(this.sketch.handle, this.kindId, this.index, p, n, on ? 1 : 0));
  }

  /** What it is *drawn with*, cascaded: **the core resolves and the front end strokes**, the
   *  same seam callout layout and curve tessellation sit on, so every front end draws one
   *  drawing alike. */
  get style(): Style {
    return takeJson<Style>(core().gcs_entity_style(this.sketch.handle, this.kindId, this.index));
  }
}

/** Points a curve's polyline is expected to need; enough for any ordinary curve at any ordinary
 *  zoom, and only a miss costs a second tessellation. */
const POLYLINE_CAP = 512;

export class Point extends Entity {
  readonly kind = 'point' as const;

  get x(): Param {
    return this.params[0];
  }

  get y(): Param {
    return this.params[1];
  }

  get xy(): [number, number] {
    const p = this.params;
    return [p[0].value, p[1].value];
  }

  get isFixed(): boolean {
    const p = this.params;
    return p[0].fixed && p[1].fixed;
  }

  fix(fixed = true): void {
    for (const p of this.params) p.fixed = fixed;
  }
}

export class Line extends Styled {
  readonly kind = 'line' as const;

  get p1(): Point {
    return this.children[0];
  }

  get p2(): Point {
    return this.children[1];
  }

  direction(): [number, number] {
    const [ax, ay] = this.p1.xy;
    const [bx, by] = this.p2.xy;
    return [bx - ax, by - ay];
  }

  length(): number {
    const [dx, dy] = this.direction();
    return Math.hypot(dx, dy);
  }
}

export class Circle extends Styled {
  readonly kind = 'circle' as const;

  get center(): Point {
    return this.children[0];
  }

  get radius(): Param {
    return this.sketch.paramAt(
      core().gcs_entity_radius_param(this.sketch.handle, this.kindId, this.index));
  }
}

/** CCW arc from `start` to `end` about `center`.  The radius is its own Param so Circle and Arc
 *  share every radius-based constraint; the two intrinsic constraints |start-center|² = r² and
 *  |end-center|² = r² are added by `Sketch.arc`. */
export class Arc extends Styled {
  readonly kind = 'arc' as const;

  get center(): Point {
    return this.children[0];
  }

  get start(): Point {
    return this.children[1];
  }

  get end(): Point {
    return this.children[2];
  }

  get radius(): Param {
    return this.sketch.paramAt(
      core().gcs_entity_radius_param(this.sketch.handle, this.kindId, this.index));
  }

  angles(): [number, number] {
    return withBuf(2, 8, (b) => {
      core().gcs_arc_angles(this.sketch.handle, this.index, b.ptr);
      const v = b.f64;
      return [v[0], v[1]] as [number, number];
    });
  }
}

/** A cubic B-spline over an ordered control polygon.
 *
 *  The control points are ordinary sketch Points, so they drag, snap and take constraints like
 *  any others.  Everything about the curve itself — where a parameter lands, the polyline it is
 *  drawn as, the distance to it — is computed in the core: nothing here evaluates a basis
 *  function, exactly as nothing here lays out a dimension callout. */
export class Spline extends Styled {
  readonly kind = 'spline' as const;

  /** A control polygon is as long as it is, so this is the one kind whose width follows the
   *  document — every other kind keeps the fixed buffer and never asks for a count. */
  protected override get slots(): number {
    return Math.max(8, 2 * this.sketch.pointCount);
  }

  get ctrl(): Point[] {
    return this.children;
  }

  get knots(): number[] {
    return withBuf(this.slots + 8, 8, (b) => {
      const n = core().gcs_spline_knots(this.sketch.handle, this.index, b.ptr);
      return [...b.f64.subarray(0, n)];
    });
  }

  /** The parameter interval the curve is drawn over. */
  get domain(): [number, number] {
    return withBuf(2, 8, (b) => {
      core().gcs_spline_domain(this.sketch.handle, this.index, b.ptr);
      const v = b.f64;
      return [v[0], v[1]] as [number, number];
    });
  }

  /** C(t), C'(t) and C''(t). */
  eval(t: number): { p: [number, number]; d1: [number, number]; d2: [number, number] } {
    return withBuf(6, 8, (b) => {
      core().gcs_spline_eval(this.sketch.handle, this.index, t, b.ptr);
      const v = b.f64;
      return { p: [v[0], v[1]], d1: [v[2], v[3]], d2: [v[4], v[5]] };
    });
  }

  pointAt(t: number): [number, number] {
    return this.eval(t).p;
  }

  /** The curve as a polyline, refined until a chord strays less than a fraction of a pixel from
   *  it.  `unit` is the world length of one screen pixel, as everywhere else in the drawing. */
  polyline(unit: number): [number, number][] {
    // One tessellation in almost every case.  The core reports how many points it wanted, so a
    // buffer that was too small costs a second pass and nothing else — asking the length first
    // would tessellate the curve twice, every time.
    const read = (cap: number): [number, number][] | number => withBuf(2 * cap, 8, (b) => {
      const need = core().gcs_spline_polyline(this.sketch.handle, this.index, unit, b.ptr, cap);
      if (need > cap) return need;
      const v = b.f64;
      const out: [number, number][] = [];
      for (let i = 0; i < need; i++) out.push([v[2 * i], v[2 * i + 1]]);
      return out;
    });
    const first = read(POLYLINE_CAP);
    return typeof first === 'number' ? (read(first) as [number, number][]) : first;
  }

  /** Give the curve one more control point at `t`, without changing its shape.  Every contact
   *  keeps its parameter and its place; null if `t` is not a place a knot can go. */
  insertControl(t: number): Point | null {
    const i = core().gcs_spline_insert_control(this.sketch.handle, this.index, t);
    return i < 0 ? null : this.sketch.points[i];
  }

  /** The parameter of the nearest curve point, and how far that is — the pick test. */
  closest(x: number, y: number): { t: number; distance: number } {
    return withBuf(2, 8, (b) => {
      core().gcs_spline_closest(this.sketch.handle, this.index, x, y, b.ptr);
      const v = b.f64;
      return { t: v[0], distance: v[1] };
    });
  }
}

/** Centre, one end of the major axis, and a minor radius of its own.  Five numbers — the 5 DOF
 *  an ellipse has — so unlike an arc it carries no intrinsic constraint; the major point is a
 *  real rim point and drags, snaps and constrains like any other. */
export class Ellipse extends Styled {
  readonly kind = 'ellipse' as const;

  get center(): Point {
    return this.children[0];
  }

  get major(): Point {
    return this.children[1];
  }

  get minor(): Param {
    return this.sketch.paramAt(
      core().gcs_entity_radius_param(this.sketch.handle, this.kindId, this.index));
  }
}

/** A curve written in the language: `C(u)` as a pair of expressions over the geometry it is
 *  drawn from.  It owns no coordinates — it *is* its expressions — so it moves exactly when its
 *  arguments do, and the core lays out the polyline the front end strokes. */
export class Curve extends Styled {
  readonly kind = 'curve' as const;

  /** As many arguments as its family takes, of whatever kinds. */
  protected override get slots(): number {
    return Math.max(8, 2 * this.sketch.pointCount);
  }

  /** The curve as a polyline, laid out by the core.  Asked once for the count, then once for
   *  the points — the same buffer-size-then-retry a control polygon uses. */
  polyline(): [number, number][] {
    const c = core();
    const n = c.gcs_curve_polyline(this.sketch.handle, this.index, 0, 0);
    if (n <= 0) return [];
    return withBuf(2 * n, 8, (b) => {
      c.gcs_curve_polyline(this.sketch.handle, this.index, b.ptr, n);
      const v = b.f64;
      const out: [number, number][] = [];
      for (let i = 0; i < n; i++) out.push([v[2 * i], v[2 * i + 1]]);
      return out;
    });
  }
}

/** An origin, a point it is pointed at, and a unit rotor slaved to the chord between them by
 *  two intrinsic constraints — a datum other statements measure from, adding no freedom beyond
 *  its two points.  A trace block reads its bearing as `f.angle`. */
export class Frame extends Styled {
  readonly kind = 'frame' as const;

  get origin(): Point {
    return this.children[0];
  }

  get toward(): Point {
    return this.children[1];
  }

  /** The rotor `(c, s)`, held to the unit circle by its intrinsic constraint. */
  get rotor(): [Param, Param] {
    const p = this.params;
    return [p[4], p[5]];
  }
}

export type Primitive = Point | Line | Circle | Arc | Spline | Ellipse | Curve | Frame;

const CLASSES =
  { point: Point, line: Line, circle: Circle, arc: Arc, spline: Spline,
    ellipse: Ellipse, curve: Curve, frame: Frame } as const;

/** The CCW arc through three points: centre, radius, and the sweep that passes through the
 *  third point.  `swapped` is true when that sweep runs from the *second* given point. */
export interface ThreePointArc {
  cx: number;
  cy: number;
  r: number;
  a0: number;
  a1: number;
  swapped: boolean;
}

/** null if the three points are collinear (the test is on the sine of the angle, so it is
 *  scale-free). */
export function threePointArc(ax: number, ay: number, bx: number, by: number,
                              cx: number, cy: number): ThreePointArc | null {
  return withBuf(6, 8, (b) => {
    if (!core().gcs_three_point_arc(ax, ay, bx, by, cx, cy, b.ptr)) return null;
    const v = b.f64;
    return { cx: v[0], cy: v[1], r: v[2], a0: v[3], a1: v[4], swapped: v[5] !== 0 };
  });
}

export class Sketch {
  readonly handle: number;
  private params_: Param[] = [];
  private ents: Record<Kind, Entity[]> =
    { point: [], line: [], circle: [], arc: [], spline: [], ellipse: [], curve: [],
      frame: [] };
  private cons: Constraint[] = [];
  /** Constraint id → its proxy, so identity survives every round trip. */
  readonly byId = new Map<number, Constraint>();
  private dirty = true;

  constructor(handle?: number) {
    this.handle = handle ?? core().gcs_sketch_new();
  }

  dispose(): void {
    core().gcs_sketch_free(this.handle);
  }

  // -- interning ----------------------------------------------------------

  private counts(): Int32Array {
    // sized by the core, not by a number written here: a buffer short by one entity kind is an
    // overflow rather than a truncation, and it surfaces as a crash somewhere else entirely
    return withBuf(core().gcs_counts_len(), 4, (b) => {
      core().gcs_sketch_counts(this.handle, b.ptr);
      return b.i32.slice();
    });
  }

  paramAt(i: number): Param {
    while (this.params_.length <= i) this.params_.push(new Param(this, this.params_.length));
    return this.params_[i];
  }

  private list<T extends Entity>(kind: Kind, n: number): T[] {
    const lst = this.ents[kind];
    const Cls = CLASSES[kind];
    while (lst.length < n) lst.push(new Cls(this, lst.length));
    lst.length = n;
    return lst as T[];
  }

  /** The constraint list changed in the core. */
  touch(): void {
    this.dirty = true;
  }

  /** Bring the proxies up to date with the core if anything has changed since they last were —
   *  what a proxy does before handing out a value, since an expression's number can move with
   *  an edit made elsewhere. */
  sync(): void {
    this.syncConstraints();
  }

  private syncConstraints(): void {
    if (!this.dirty) return;
    this.dirty = false;
    const records = takeJsonRecords(core().gcs_constraints_json(this.handle));
    this.cons = records.map((rec) => {
      let c = this.byId.get(rec.id);
      if (!c) {
        c = fromRecord(this, rec);
        this.byId.set(rec.id, c);
      } else {
        // the core is the authority on a value: a dimension written as an expression changes
        // when a name it reads does, with nothing said to this proxy
        c.absorb(this, rec);
      }
      return c;
    });
  }

  // -- construction -------------------------------------------------------

  point(x: number, y: number, fixed = false, name = ''): Point {
    const i = withStr(name, (p, n) => core().gcs_sketch_point(this.handle, x, y, fixed ? 1 : 0, p, n));
    return this.points[i];
  }

  line(p1: Point, p2: Point): Line {
    // the index first: `this.lines[...]` would evaluate the getter before the core has the line
    const i = core().gcs_sketch_line(this.handle, p1.index, p2.index);
    return this.lines[i];
  }

  lineXY(x1: number, y1: number, x2: number, y2: number, name = ''): Line {
    return this.line(this.point(x1, y1, false, `${name}.p1`), this.point(x2, y2, false, `${name}.p2`));
  }

  circle(center: Point, radius: number, name = ''): Circle {
    const i = withStr(name, (p, n) => core().gcs_sketch_circle(this.handle, center.index, radius, p, n));
    return this.circles[i];
  }

  /** An ellipse about `center` whose major axis ends at `major`, with minor radius `b`. */
  ellipse(center: Point, major: Point, b: number, name = ''): Ellipse {
    const i = withStr(name, (p, n) =>
      core().gcs_sketch_ellipse(this.handle, center.index, major.index, b, p, n));
    return this.ellipses[i];
  }

  /** A frame at `origin` pointed at `toward`. */
  frame(origin: Point, toward: Point, name = ''): Frame {
    const i = withStr(name, (p, n) =>
      core().gcs_sketch_frame(this.handle, origin.index, toward.index, p, n));
    this.touch();     // the rotor's two intrinsic constraints came with it
    return this.frames[i];
  }

  /** A cubic B-spline over `ctrl`.  null when there are too few control points for a cubic, or
   *  the knot vector given does not fit them. */
  spline(ctrl: Point[], knots?: number[]): Spline | null {
    const i = withBuf(Math.max(1, ctrl.length), 4, (b) => {
      b.i32.set(ctrl.map((p) => p.index));
      if (!knots) return core().gcs_sketch_spline(this.handle, b.ptr, ctrl.length);
      return withBuf(Math.max(1, knots.length), 8, (k) => {
        k.f64.set(knots);
        return core().gcs_sketch_spline_knots(
          this.handle, b.ptr, ctrl.length, k.ptr, knots.length);
      });
    });
    return i < 0 ? null : this.splines[i];
  }

  /** A cubic B-spline through `pts`, in order.  The control points are computed, not clicked —
   *  the same bargain `arcThrough` strikes.
   *
   *  `hold[i]`, where given, is a Point the place came from rather than empty space: the curve
   *  is held to it by a `PointOnSpline` pinned at the parameter the fit chose, so a curve fitted
   *  to constrained points is itself fully constrained.  null if there are too few points for a
   *  cubic, or they give no parameterisation. */
  splineThrough(pts: readonly (readonly [number, number])[],
                hold: readonly (Point | null)[] = []): Spline | null {
    const n = Math.max(1, pts.length);
    const i = withBuf(2 * n, 8, (b) => {
      b.f64.set(pts.flatMap((p) => [p[0], p[1]]));
      return withBuf(n, 4, (h) => {
        h.i32.set(pts.map((_, k) => hold[k]?.index ?? -1));
        return core().gcs_sketch_spline_through(this.handle, b.ptr, pts.length, h.ptr);
      });
    });
    return i < 0 ? null : this.splines[i];
  }

  arc(center: Point, start: Point, end: Point, name = ''): Arc {
    const i = withStr(name, (p, n) =>
      core().gcs_sketch_arc(this.handle, center.index, start.index, end.index, p, n));
    this.touch();     // the two intrinsic PointOnCircle constraints came with it
    return this.arcs[i];
  }

  /** Arc from `start` to `end` bulging through `through`; null if the three are collinear. */
  arcThrough(start: Point, end: Point, through: [number, number], name = ''): Arc | null {
    const i = withStr(name, (p, n) =>
      core().gcs_sketch_arc_through(this.handle, start.index, end.index, through[0], through[1], p, n));
    if (i < 0) return null;
    this.touch();
    return this.arcs[i];
  }

  /** Four lines round the corners, sharing corner points, with three perpendiculars — the fourth
   *  follows, so adding it would over-constrain every rectangle by one equation. */
  rectangle(a: Point, x1: number, y1: number, name = ''): Line[] {
    const out = withBuf(4, 4, (b) => {
      withStr(name, (p, n) => core().gcs_sketch_rectangle(this.handle, a.index, x1, y1, p, n, b.ptr));
      return [...b.i32];
    });
    this.touch();
    return out.map((i) => this.lines[i]);
  }

  rectangleXY(x0: number, y0: number, x1: number, y1: number, name = ''): Line[] {
    return this.rectangle(this.point(x0, y0, false, `${name}.a`), x1, y1, name);
  }

  add(...constraints: Constraint[]): void {
    for (const c of constraints) c.bind(this);
    this.touch();
  }

  remove(constraint: Constraint): void {
    if (constraint.id < 0) return;
    core().gcs_constraint_remove(this.handle, constraint.id);
    this.byId.delete(constraint.id);
    constraint.unbind();
    this.touch();
  }

  // -- lists --------------------------------------------------------------

  get params(): Param[] {
    const n = this.counts()[0];
    if (n) this.paramAt(n - 1);
    return this.params_.slice(0, n);
  }

  get points(): Point[] {
    return this.list<Point>('point', this.counts()[1]);
  }

  get lines(): Line[] {
    return this.list<Line>('line', this.counts()[2]);
  }

  get circles(): Circle[] {
    return this.list<Circle>('circle', this.counts()[3]);
  }

  get arcs(): Arc[] {
    return this.list<Arc>('arc', this.counts()[4]);
  }

  get splines(): Spline[] {
    return this.list<Spline>('spline', this.counts()[6]);
  }

  get ellipses(): Ellipse[] {
    return this.list<Ellipse>('ellipse', this.counts()[7]);
  }

  get curves(): Curve[] {
    return this.list<Curve>('curve', this.counts()[8]);
  }

  get frames(): Frame[] {
    return this.list<Frame>('frame', this.counts()[9]);
  }

  /** How many points the document has — the size a control-polygon buffer has to allow for. */
  get pointCount(): number {
    return this.counts()[1];
  }

  get constraints(): Constraint[] {
    this.syncConstraints();
    return this.cons.slice();
  }

  set constraints(cs: Constraint[]) {
    withJson(cs.filter((c) => c.id >= 0).map((c) => c.id),
             (p, n) => core().gcs_sketch_set_constraints(this.handle, p, n));
    this.touch();
  }

  entities(kind: Kind): Primitive[] {
    return (kind === 'point' ? this.points : kind === 'line' ? this.lines
      : kind === 'circle' ? this.circles : kind === 'spline' ? this.splines
      : kind === 'ellipse' ? this.ellipses
      : kind === 'curve' ? this.curves
      : kind === 'frame' ? this.frames : this.arcs) as Primitive[];
  }

  /** Every entity, in creation order per kind. */
  primitives(): Primitive[] {
    return [...this.points, ...this.lines, ...this.circles, ...this.arcs, ...this.splines,
            ...this.ellipses, ...this.frames];
  }

  /** Constraints the user added (excludes intrinsic and soft/transient ones). */
  userConstraints(): Constraint[] {
    return this.constraints.filter((c) => !(c.intrinsic || c.soft));
  }

  /** Everything that must be satisfied (excludes soft ones such as drag targets). */
  hardConstraints(): Constraint[] {
    return this.constraints.filter((c) => !c.soft);
  }

  constraintById(id: number): Constraint | undefined {
    this.syncConstraints();
    return this.byId.get(id);
  }

  // -- parameter vector ---------------------------------------------------

  getX(): Float64Array {
    const n = this.counts()[0];
    return withBuf(Math.max(n, 1), 8, (b) => {
      core().gcs_sketch_get_x(this.handle, b.ptr);
      return b.f64.slice(0, n);
    });
  }

  /** Write the parameter vector.  A vector of the wrong length belongs to some other sketch, and
   *  is refused rather than written as far as it goes. */
  setX(x: ArrayLike<number>): void {
    const ok = withBuf(Math.max(x.length, 1), 8, (b) => {
      b.set(x);
      return core().gcs_sketch_set_x(this.handle, b.ptr, x.length) === 0;
    });
    if (!ok) throw new Error(lastError() || 'setX: wrong length');
  }

  freeIndices(): Int32Array {
    const out: number[] = [];
    this.params.forEach((p, i) => {
      if (!p.fixed) out.push(i);
    });
    return Int32Array.from(out);
  }

  nResiduals(): number {
    return core().gcs_sketch_n_residuals(this.handle);
  }

  // -- geometry -----------------------------------------------------------

  /** (xmin, ymin, xmax, ymax) over all points.  Points only, deliberately: `extent()` is built on
   *  this, and `extent()` scales the solver's tolerances, the witness perturbation and the drag
   *  continuation step.  For what is drawn, use `drawnBounds()`. */
  bbox(): Box {
    return this.boundsOf(0);
  }

  drawnBounds(): Box {
    return this.boundsOf(1);
  }

  private boundsOf(drawn: number): Box {
    return withBuf(4, 8, (b) => {
      core().gcs_sketch_bounds(this.handle, drawn, b.ptr);
      const v = b.f64;
      return [v[0], v[1], v[2], v[3]] as Box;
    });
  }

  extent(): number {
    return core().gcs_sketch_extent(this.handle);
  }

  /** What a compiled plan or System depends on: which entities exist, which constraints (by id,
   *  so swapping one Distance for another shows up — counts and type names alone do not) and
   *  which params are fixed.  A cache over compiled artefacts keys on this. */
  topologyKey(): string {
    return takeStr(core().gcs_sketch_topology_key(this.handle));
  }

  /** Seeded Gaussian noise on every free parameter (warm starts, witness construction). */
  perturb(sigma: number, seed = 0): void {
    core().gcs_sketch_perturb(this.handle, sigma, seed >>> 0);
  }

  /** What a click at (x, y) picks: the nearest entity whose *drawn* figure comes within `tol`,
   *  a world length — so a front end passes `PICK_PX * unit` and keeps no geometry of its own.
   *  The core measures against what it drew, which is what makes clicking a thing and
   *  constraining it agree about where it is. */
  pick(x: number, y: number, tol: number): Primitive | null {
    return withBuf(2, 8, (b) => {
      if (!core().gcs_sketch_pick(this.handle, x, y, tol, b.ptr)) return null;
      return this.entities(KINDS[b.f64[0]])[b.f64[1]];
    });
  }

  nearestPoint(x: number, y: number): { point: Point | null; dist: number } {
    return withBuf(1, 8, (b) => {
      const i = core().gcs_sketch_nearest_point(this.handle, x, y, b.ptr);
      return { point: i >= 0 ? this.points[i] : null, dist: b.f64[0] };
    });
  }

  // -- document state -----------------------------------------------------

  /** Recorded root choices (Stage 5), persisted with the document. */
  get branches(): Map<string, number> {
    const o = takeJsonObj(core().gcs_branches_json(this.handle));
    return new Map(Object.entries(o).map(([k, v]) => [k, Number(v)]));
  }

  set branches(b: Map<string, number>) {
    withJson(Object.fromEntries(b), (p, n) => core().gcs_branches_set_json(this.handle, p, n));
  }

  clone(): Sketch {
    return new Sketch(core().gcs_sketch_clone(this.handle));
  }
}

/** Signed perpendicular offset from the *infinite* line, positive to the left of its direction;
 *  Infinity when the line is degenerate. */
export function signedPointToLine(px: number, py: number, ln: Line): number {
  return core().gcs_signed_point_to_line(ln.sketch.handle, px, py, ln.index);
}

/** Signed CCW angle from line `a` to line `b`, in radians — what an `Angle` constraint's value
 *  means, and what a dimension dialog offers as the current value. */
export function angleBetween(a: Line, b: Line): number {
  return core().gcs_angle_between(a.sketch.handle, a.index, b.index);
}

/** The point at distance `r` from (cx, cy) towards (tx, ty).  The centre-start-end arc
 *  construction: the third click gives a direction, and the radius comes from the second.
 *  Null when the target is the centre, which names no direction. */
export function onRadius(cx: number, cy: number, tx: number, ty: number, r: number)
  : [number, number] | null {
  return withBuf(2, 8, (b) => (core().gcs_on_radius(cx, cy, tx, ty, r, b.ptr)
    ? [b.f64[0], b.f64[1]] as [number, number]
    : null));
}

/** The minor radius that puts the rim of the ellipse (centre c, major end m) through (tx, ty)
 *  — the ellipse tool's third click, and where a rim drag holds the rim to the cursor.  Null
 *  when centre and major end coincide, which names no axis. */
export function ellipseMinor(cx: number, cy: number, mx: number, my: number,
                             tx: number, ty: number): number | null {
  const b = core().gcs_ellipse_minor(cx, cy, mx, my, tx, ty);
  return b < 0 ? null : b;
}

/** Shortest distance between two entities, as a sketcher measures it: lines are infinite, arcs
 *  are the whole circle they lie on. */
export function distanceBetween(a: Primitive, b: Primitive): number {
  return core().gcs_distance_between(a.sketch.handle, a.kindId, a.index, b.kindId, b.index);
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

/* `Sketch` needs to materialize constraint proxies, which live in constraints.ts and import this
 * module for their types.  Registering the factory here keeps the runtime dependency one-way. */
export interface ConstraintRecord {
  id: number;
  type: string;
  args: unknown[];
  soft: boolean;
  intrinsic: boolean;
  /** a `claim` (Solvent §9.7): expected to add no rank, judged by the diagnosis, never solved */
  claim: boolean;
  /** attribute → the expression text behind it, for a dimension written as one */
  exprs?: Record<string, string>;
}

type Factory = (sk: Sketch, rec: ConstraintRecord) => Constraint;
let factory: Factory | null = null;

export function registerConstraintFactory(f: Factory): void {
  factory = f;
}

function fromRecord(sk: Sketch, rec: ConstraintRecord): Constraint {
  if (!factory) throw new Error('constraints.js was not loaded');
  return factory(sk, rec);
}

function takeJsonRecords(handle: number): ConstraintRecord[] {
  const s = takeStr(handle);
  return s ? (JSON.parse(s) as ConstraintRecord[]) : [];
}

function takeJsonObj(handle: number): Record<string, unknown> {
  const s = takeStr(handle);
  return s ? (JSON.parse(s) as Record<string, unknown>) : {};
}

export { Buf };
