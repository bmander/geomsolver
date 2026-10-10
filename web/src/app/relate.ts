/* What a constraint button reads off the selection: the selection sorted into bins, which
 * class's slots it fills, and the constraints it states — apart from the bar itself, which
 * toasts and adds, so the reading is tested without a page. */
import * as C from '../core/constraints.js';
import { Constraint, ENTITY_KINDS } from '../core/constraints.js';
import { Arc, Axis, Circle, Curve, Line, Plane, Point, Spline } from '../core/model.js';
import type { Primitive } from '../core/model.js';

export type Sel = {
  pts: Point[]; lines: Line[]; circles: (Circle | Arc)[]; splines: Spline[];
  curves: Curve[]; planes: Plane[]; axes: Axis[];
};
type Bin = keyof Sel;
/** A spec slot's bin, or `rays`: a direction, which a line or an axis fills. */
type Slot = Bin | 'rays';

export function sortSelection(s: readonly Primitive[]): Sel {
  return {
    pts: s.filter((e): e is Point => e instanceof Point),
    lines: s.filter((e): e is Line => e instanceof Line),
    circles: s.filter((e): e is Circle | Arc => e instanceof Circle || e instanceof Arc),
    splines: s.filter((e): e is Spline => e instanceof Spline),
    curves: s.filter((e): e is Curve => e instanceof Curve),
    planes: s.filter((e): e is Plane => e instanceof Plane),
    axes: s.filter((e): e is Axis => e instanceof Axis),
  };
}

/** The bin of the selection a spec slot of this kind is filled from — `null` for a slot that is
 *  not an entity (a number, a selector, a contact's own parameter, which the core seeds), and
 *  for an ordinate's direction, which a word names and the core reads off the view. */
function binOf(kind: string): Slot | null {
  if (kind === 'along') return null;
  if (kind === 'point') return 'pts';
  if (kind === 'line') return 'lines';
  if (kind === 'spline') return 'splines';
  if (kind === 'curve') return 'curves';
  if (kind === 'plane') return 'planes';
  if (kind === 'axis') return 'axes';
  if (kind === 'direction') return 'rays';
  return ENTITY_KINDS.has(kind) ? 'circles' : null;   // a circle or an arc, whichever is picked
}

export const BINS: Bin[] = ['pts', 'lines', 'circles', 'splines', 'curves', 'planes', 'axes'];
const SLOTS: Slot[] = [...BINS, 'rays'];
const BIN_WORD: Record<Slot, string> = {
  pts: 'point(s)', lines: 'line(s)', circles: 'circle(s)/arc(s)', splines: 'spline(s)',
  curves: 'curve(s)', planes: 'plane(s)', axes: 'axis/axes', rays: 'line(s) or axis/axes',
};

/** How many of each slot a class wants — counted off its spec, the one statement of it. */
function wanted(cls: C.ConstraintCtor): Record<Slot, number> {
  const n = Object.fromEntries(SLOTS.map((b) => [b, 0])) as Record<Slot, number>;
  for (const [, kind] of cls.spec) {
    const b = binOf(kind);
    if (b) n[b]++;
  }
  return n;
}

/** What a class wants, as a toast says it: `2 point(s), 1 plane(s)`. */
export function wantedText(cls: C.ConstraintCtor): string {
  const w = wanted(cls);
  return SLOTS.filter((b) => w[b]).map((b) => `${w[b]} ${BIN_WORD[b]}`).join(', ');
}

/** A class on one line alone applies to every selected line at once — level and plumb. */
function perLine(w: Record<Slot, number>): boolean {
  return w.lines === 1 && SLOTS.every((b) => b === 'lines' || w[b] === 0);
}

/** Whether the selection is what the class wants: each bin exactly, the lines and axes left
 *  over from their own slots exactly filling its rays. */
export function fitsSelection(cls: C.ConstraintCtor, s: Sel): boolean {
  const w = wanted(cls);
  if (perLine(w)) return s.lines.length >= 1 && BINS.every((b) => b === 'lines' || !s[b].length);
  const spare = (b: 'lines' | 'axes') => s[b].length - w[b];
  return BINS.every((b) => b === 'lines' || b === 'axes' || s[b].length === w[b])
    && spare('lines') >= 0 && spare('axes') >= 0 && spare('lines') + spare('axes') === w.rays;
}

/** The constraints a class states over a selection it fits, the entities in spec order — one
 *  per selected line for a class on one line alone. */
export function stated(cls: C.ConstraintCtor, s: Sel): Constraint[] {
  const w = wanted(cls);
  const each = perLine(w);
  return (each ? s.lines : [null]).map((ln) => {
    const args: unknown[] = [];
    const taken = Object.fromEntries(BINS.map((b) => [b, 0])) as Record<Bin, number>;
    // a ray takes the lines its own slots leave, then the axes
    const rays = [...s.lines.slice(w.lines), ...s.axes.slice(w.axes)];
    for (const [, kind] of cls.spec) {
      const b = binOf(kind);
      // a `param` slot is not an entity: it is left out, and the core seeds it off the geometry
      if (!b) continue;
      args.push(b === 'rays' ? rays.shift() : b === 'lines' && each ? ln : s[b][taken[b]++]);
    }
    return new (cls as unknown as new (...a: unknown[]) => Constraint)(...args);
  });
}

/* What each button that reads the selection may mean, by class name, in the order tried: the
 * first the selection fits is the one meant, so a pair of lines on a page states the 2D kind
 * before the one in space. */

/** "These touch": two points meet, a point on a line, circle, arc or curve — and in space a
 *  point, a line or an axis on a plane or an axis, an axis on an axis. */
export const INCIDENCE = [
  'Coincident', 'PointOnLine', 'PointOnCircle', 'PointOnSpline', 'PointOnCurve',
  'PointOnPlane', 'LineOnPlane', 'AxisOnPlane', 'PointOnAxis', 'LineOnAxis', 'AxisCoincident',
] as const;
/** Two lines on a page — or in space lines, axes and planes in any pair the language relates. */
export const PARALLEL = ['Parallel', 'Parallel3', 'AxisParallelPlane', 'PlaneParallel'] as const;
export const PERPENDICULAR = ['Perpendicular', 'Perpendicular3', 'AxisPerpendicularPlane'] as const;

/** The first of these classes the selection fits — one button, several meanings. */
export function firstFit(names: readonly string[], s: Sel): C.ConstraintCtor | undefined {
  return names.map((n) => C.type(n)).find((cls) => fitsSelection(cls, s));
}
