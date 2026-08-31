/* The constraints bar: what each button states, and how it reads the selection to find out.
 * A button says what it is *for*, not which class it makes — one "these touch" button, one
 * "put a number on this" button — so the selection decides which constraint that comes to. */
import * as C from '../core/constraints.js';
import { Constraint, ENTITY_KINDS } from '../core/constraints.js';
import {
  Arc, Circle, Ellipse, Line, Plane, Point, Spline, angleBetween, distanceBetween,
  signedPointToLine,
} from '../core/model.js';
import { view } from './shell.js';
import { ToolbarButton, toast } from './ui.js';
import type { DimAlt } from './view.js';

/* constraints whose arguments are just entities:
 * (label, class, points, lines, circles/arcs, shortcut, splines, ellipses) */
type Simple = [string, C.ConstraintCtor, number, number, number, string?, number?, number?];
const SIMPLE: Simple[] = [
  ['Parallel', C.Parallel, 0, 2, 0, 'b'],
  ['Perpendicular', C.Perpendicular, 0, 2, 0, '⇧l'],
  ['Midpoint', C.Midpoint, 1, 1, 0, '⇧m'],
];
/* Level and plumb read the selection too.  A pair of points says exactly what a line through
 * them would — that the segment between them is level — and wanting that without drawing the
 * line is the common case: two corners of a shape that share no edge. */
const LEVEL: Simple[] = [
  ['Horizontal', C.Horizontal, 0, 1, 0],
  ['Horizontal', C.HorizontalPoints, 2, 0, 0],
  ['Vertical', C.Vertical, 0, 1, 0],
  ['Vertical', C.VerticalPoints, 2, 0, 0],
];

function cLevel(label: 'Horizontal' | 'Vertical'): void {
  const { pts, lines, circles } = sel();
  const hit = LEVEL.find(([l, , nPts, nLines, nCirc]) =>
    l === label && circles.length === nCirc && pts.length === nPts
    && (nLines === 1 ? lines.length >= 1 : lines.length === nLines));
  if (!need(!!hit, 'one or more lines, or two points')) return;
  applySimple(hit as Simple);
}

/* One "these touch" button.  Which incidence it means is the selection's business, not the
 * user's: two points meet, a point sits on a line, a point sits on a circle or arc. */
const INCIDENCE: Simple[] = [
  ['Coincident', C.Coincident, 2, 0, 0],
  ['On line', C.PointOnLine, 1, 1, 0],
  ['On circle', C.PointOnCircle, 1, 0, 1],
  // a curve is one more row, not a branch: `applySimple` fills the spec's slots by kind, and
  // the contact's hidden parameter is not an entity slot so it is left for the core to seed
  ['On curve', C.PointOnSpline, 1, 0, 0, undefined, 1],
  ['On ellipse', C.PointOnEllipse, 1, 0, 0, undefined, 0, 1],
];
/* The constraints bar, in an order that interleaves the dimensioned constraints with the
 * entity-only ones.  `key` is both the chip printed on the button and the token the keyboard
 * handler matches — '⇧l' prints as ⇧L and fires on shift-L — so a button and its shortcut
 * cannot drift apart. */
export const CONSTRAINT_BUTTONS: ToolbarButton[] = [
  { label: 'Coincident', key: 'i', onClick: () => cCoincident(),
    title: 'Two points meet · a point on a line · a point on a circle, arc, curve or ellipse' },
  { label: 'Dimension', key: 'd', onClick: () => cDimension(),
    title: 'Put a number on the selection, then place it and type · a length, a radius, an '
         + 'offset, a ring · on two points, above them is the run and beside them the rise '
         + '· two lines take their gap when parallel and their angle when not' },
  { label: 'Horizontal', key: 'h', onClick: () => cLevel('Horizontal'),
    title: 'Level: one or more lines, or a pair of points with no line between them' },
  { label: 'Vertical', key: 'v', onClick: () => cLevel('Vertical'),
    title: 'Plumb: one or more lines, or a pair of points with no line between them' },
  ...SIMPLE.map((c): ToolbarButton => ({ label: c[0], key: c[5], onClick: () => applySimple(c) })),
  { label: 'Equal', key: 'e', onClick: () => cEqual() },
  { label: 'Tangent', key: 't', onClick: () => cTangent(),
    title: 'A line or a circle tangent to a circle/arc · a line tangent to a curve or ellipse '
         + '· a circle taking a curve\'s or an ellipse\'s own radius where it touches' },
  { label: 'Symmetric', key: '⇧q', onClick: () => cSymmetric() },
  { label: 'Project', key: 'j', onClick: () => cProject(),
    title: 'Two points, each drawn in a view, are images of one point in space: what they '
         + 'share along the fold line between their views agrees' },
  { label: 'Fix', key: 'f', onClick: () => view.toggleFixSelected() },
];

/* -- selection helpers -------------------------------------------------------- */

function sel(): {
  pts: Point[]; lines: Line[]; circles: (Circle | Arc)[]; splines: Spline[];
  ellipses: Ellipse[]; planes: Plane[];
} {
  const s = view.selected;
  return {
    pts: s.filter((e): e is Point => e instanceof Point),
    lines: s.filter((e): e is Line => e instanceof Line),
    circles: s.filter((e): e is Circle | Arc => e instanceof Circle || e instanceof Arc),
    splines: s.filter((e): e is Spline => e instanceof Spline),
    ellipses: s.filter((e): e is Ellipse => e instanceof Ellipse),
    planes: s.filter((e): e is Plane => e instanceof Plane),
  };
}

function need(ok: boolean, what: string): boolean {
  if (!ok) toast(`select ${what} first`);
  return ok;
}

/** Generic applier: checks the selection has the required counts and passes the entities in
 *  spec order.  Single-line constraints (Horizontal/Vertical) apply to every selected line. */
function applySimple([, cls, nPts, nLines, nCirc, , nSpl = 0, nEll = 0]: Simple): void {
  const { pts, lines, circles, splines, ellipses, planes } = sel();
  const perLine = nPts === 0 && nLines === 1 && nCirc === 0;
  const ok = pts.length === nPts && circles.length === nCirc && splines.length === nSpl
    && ellipses.length === nEll
    && (perLine ? lines.length >= 1 : lines.length === nLines);
  const what = ([[nPts, 'point(s)'], [nLines, 'line(s)'], [nCirc, 'circle(s)/arc(s)'],
                 [nSpl, 'curve(s)'], [nEll, 'ellipse(s)']] as const)
    .filter(([n]) => n).map(([n, w]) => `${n} ${w}`).join(', ');
  if (!need(ok, what)) return;
  const made = (perLine ? lines : [null]).map((ln) => {
    const args: unknown[] = [];
    let pi = 0, li = 0, ci = 0, si = 0, ei = 0, vi = 0;
    for (const [, kind] of cls.spec) {
      if (kind === 'point') args.push(pts[pi++]);
      else if (kind === 'line') args.push(perLine ? ln : lines[li++]);
      else if (kind === 'spline') args.push(splines[si++]);
      else if (kind === 'ellipse') args.push(ellipses[ei++]);
      else if (kind === 'plane') args.push(planes[vi++]);   // never a circle, whatever is picked
      else if (ENTITY_KINDS.has(kind)) args.push(circles[ci++]);
      // a `param` slot is not an entity: it is left out, and the core seeds it off the geometry
    }
    return new (cls as unknown as new (...a: unknown[]) => Constraint)(...args);
  });
  view.addConstraints(...made);
}

/** The single incidence button: read the selection and pick the constraint that fits it. */
function cCoincident(): void {
  const { pts, lines, circles, splines, ellipses } = sel();
  const hit = INCIDENCE.find(([, , nPts, nLines, nCirc, , nSpl = 0, nEll = 0]) =>
    pts.length === nPts && lines.length === nLines && circles.length === nCirc
    && splines.length === nSpl && ellipses.length === nEll);
  if (!need(!!hit,
            'two points, a point and a line, or a point and a circle/arc/curve/ellipse')) return;
  applySimple(hit as Simple);
}

/** One of the three dimensions between two points, stated at what the sketch measures now.
 *  The pair is ordered so the number reads positive: a run or a rise is signed from the first
 *  point to the second, so which of the two comes first is what its sign says. */
function pairDim(kind: string, a: Point, b: Point): Constraint {
  // a run measures x and a rise measures y; a length has no axis of its own, and no sign
  const axis = kind === 'HorizontalDistance' ? 0 : kind === 'VerticalDistance' ? 1 : null;
  const [p, q] = axis !== null && b.xy[axis] < a.xy[axis] ? [b, a] : [a, b];
  const v = axis !== null ? q.xy[axis] - p.xy[axis] : distanceBetween(p, q);
  return C.build(kind, [p, q, v]);
}

/** Put a dimension on the selection — the one path all six dimension buttons take.
 *
 *  It is stated at once, at what it measures now, and its number is opened on the drawing
 *  where it will stay: no box in the middle of the screen, and nothing to accept before the
 *  drawing shows what was asked for.  `alt` is what else the same selection could have meant,
 *  which is then settled by where the number is put.
 *
 *  Nothing here asks what the selection is already dimensioned by: a second dimension is
 *  written like a first, and what comes of it is the diagnosis's to say — see
 *  `edit::applyConstraints`. */
export function dimension(cs: Constraint[], alt: DimAlt | null = null): void {
  showCallouts();
  view.startDimension(cs, true, alt);
}

/** Open a dimension the drawing already carries: it is in the sketch, so there is nothing to
 *  add and nothing to place, only the number to write.  `dimbox::editValue` is the way in —
 *  the constraint list's double-click and a callout's both land there, and it is what checks
 *  there is a number to edit at all. */
export function editDimension(c: Constraint): void {
  showCallouts();
  view.startDimension([c], false, null);
}

/** A number is written where it is read, so there has to be somewhere to write it: asking for
 *  a dimension with the callouts turned off turns them back on rather than refusing. */
function showCallouts(): void {
  if (view.showDimensions) return;
  view.showDimensions = true;
  toast('dimensions turned back on — a number is edited on the drawing');
}

/** The one dimension button: what it puts a number on is the selection's business.  Two points
 *  or a single line take a length — or the run or the rise between them, whichever the number
 *  is put where — a point and a line a signed offset, a circle its radius and two of them the
 *  ring between them, and two lines take the gap between them when they are parallel, the angle
 *  at their corner when they are not. */
function cDimension(): void {
  let { pts } = sel();
  const { lines, circles } = sel();
  if (!pts.length && lines.length === 2) {
    const [a, b] = lines;
    if (!need(a.length() > 0 && b.length() > 0, 'lines with two distinct endpoints')) return;
    // "Parallel" here means the sketch makes them so, not that they merely look it: a solved
    // Parallel sits within a residual scaled to the sketch's extent, which on a short line is
    // a few ten-thousandths of a radian.  Anything looser is a corner, and what a drawing
    // dimensions on a corner is its angle.  The angle is the core's, so the test is on the
    // same number the constraint would be given.
    return Math.abs(Math.sin(angleBetween(a, b))) <= 1e-3 ? cParallelDistance(a, b)
                                                          : cAngle(a, b);
  }
  if (pts.length === 1 && lines.length === 1) return cPointLineDistance(pts[0], lines[0]);
  // a dimension *between* two circles is the ring; on any other number of them it is the
  // radius they are each to have
  if (!pts.length && !lines.length && circles.length) {
    return circles.length === 2 ? cAnnularDistance(circles[0], circles[1]) : cRadius(circles);
  }
  if (!pts.length && lines.length === 1) pts = [lines[0].p1, lines[0].p2];
  if (!need(pts.length === 2, 'two points, one line, a point and a line, two lines, '
                            + 'or one or more circles/arcs')) return;
  const [a, b] = pts;
  dimension([pairDim('Distance', a, b)], { a, b, make: (k: string) => pairDim(k, a, b) });
}

/** Two parallel lines: dimension the gap between them.  It does not make them parallel — the
 *  caller has already established that they are, and sent the other case to Angle, because a
 *  "gap" between converging lines pins one endpoint's offset and reads as arbitrary. */
function cParallelDistance(l1: Line, l2: Line): void {
  const cur = signedPointToLine(l2.p1.x.value, l2.p1.y.value, l1);
  dimension([new C.ParallelDistance(l1, l2, cur)]);
}

/** A point and a line: dimension the point's perpendicular offset, signed so negating it moves
 *  the point across.  Measured to the infinite line — the foot may fall off the end of the
 *  segment, which is what a drawing means by "distance to this edge". */
function cPointLineDistance(p: Point, line: Line): void {
  if (!need(line.length() > 0, 'a line with two distinct endpoints')) return;
  if (!need(p !== line.p1 && p !== line.p2, 'a point that is not an endpoint of the line')) return;
  dimension([new C.PointLineDistance(p, line, signedPointToLine(p.x.value, p.y.value, line))]);
}

/** Two circles or arcs: dimension the annulus between them.  Like the parallel gap it sizes
 *  the ring without centring it, so say so when the centres are not already together. */
function cAnnularDistance(c1: Circle | Arc, c2: Circle | Arc): void {
  dimension([new C.AnnularDistance(c1, c2, Math.abs(c2.radius.value) - Math.abs(c1.radius.value))]);
  if (distanceBetween(c1.center, c2.center) > 1e-9) {
    toast('the ring is dimensioned, but these circles are not concentric — add Coincident on their centres');
  }
}

/** Two lines that meet at a corner: dimension the corner. */
function cAngle(l1: Line, l2: Line): void {
  // the constructor takes radians, as the kernels do; only what a person writes is in degrees
  dimension([new C.Angle(l1, l2, angleBetween(l1, l2))]);
}

/** An equality set: every selected line the same length, or every selected circle/arc the
 *  same radius.  n entities need n-1 constraints, all against the first — a star rather than
 *  a cycle, since closing the loop would make one equation redundant (which is exactly what
 *  `polygon_chain` does on purpose, to have a redundancy the graph cannot see). */
function cEqual(): void {
  const { lines, circles } = sel();
  if (lines.length >= 2 && !circles.length) {
    view.addConstraints(...lines.slice(1).map((l) => new C.EqualLength(lines[0], l)));
  } else if (circles.length >= 2 && !lines.length) {
    view.addConstraints(...circles.slice(1).map((c) => new C.EqualRadius(circles[0], c)));
  } else {
    need(false, 'two or more lines, or two or more circles/arcs (not a mix)');
  }
}

/** Two points are images of one point in space.  Which two views they are in is not asked:
 *  the core reads it off the points, and refuses — in its own words, which become the status
 *  line — a point on no view, two in one view, or two views that are parallel. */
function cProject(): void {
  const { pts } = sel();
  // exactly two points and nothing else: `selected.length` is what says "nothing else", so a
  // plane picked alongside them is refused here rather than silently ignored
  if (!need(pts.length === 2 && view.selected.length === 2,
            'two points, one in each of two views')) return;
  view.addConstraints(new C.Project(pts[0], pts[1]));
}

/** Two points mirrored across a line — pick the two points and the axis. */
function cSymmetric(): void {
  const { pts, lines } = sel();
  if (!need(pts.length === 2 && lines.length === 1, 'two points and a line (the mirror axis)')) return;
  view.addConstraints(new C.Symmetric(pts[0], pts[1], lines[0]));
}

function cTangent(): void {
  const { lines, circles, splines, ellipses } = sel();
  /* The two parametric families take the same pair of contacts, for the same reasons, so they
   * are one table and not two blocks: against a line it is a tangency — one constraint owning
   * one parameter, since split in half it would be a point on the rim and a direction somewhere
   * else on it — and against a circle it is a curvature, which says not just the direction there
   * but how hard it turns, so the circle becomes that rim's own radius where it touches. */
  const RIMS: [Spline[] | Ellipse[], C.ConstraintCtor, C.ConstraintCtor, string][] = [
    [splines, C.SplineTangentLine, C.SplineCurvature, 'a curve'],
    [ellipses, C.EllipseTangentLine, C.EllipseCurvature, 'an ellipse'],
  ];
  const picked = RIMS.filter(([rims]) => rims.length);
  if (picked.length) {
    const [rims, tangent, curvature, what] = picked[0];
    const make = (cls: C.ConstraintCtor, other: Line | Circle | Arc): void => view.addConstraints(
      new (cls as unknown as new (...a: unknown[]) => Constraint)(rims[0], other));
    // exactly one rim, and exactly one thing for it to touch
    if (picked.length === 1 && rims.length === 1) {
      if (lines.length === 1 && !circles.length) return make(tangent, lines[0]);
      if (circles.length === 1 && !lines.length) return make(curvature, circles[0]);
    }
    need(false, `${what} and either a line or a circle`);
    return;
  }
  if (lines.length === 1 && circles.length === 1) {
    const ln = lines[0], cc = circles[0];
    if (cc instanceof Arc) {
      const ends = new Set([ln.p1, ln.p2]);
      if (ends.has(cc.start)) return view.addConstraints(new C.TangentArcLine(cc, ln, 'start'));
      if (ends.has(cc.end)) return view.addConstraints(new C.TangentArcLine(cc, ln, 'end'));
    } else {
      // a line end the user has already put on this circle is where the tangency will land, so
      // it is stated *at* that point — the pair (PointOnCircle, TangentLineCircle) says the
      // same thing as a double root, rank-deficient at every solution.  Whether the sketch
      // already says a point is on the circle is the core's question, the same one the
      // constraints bar asks before offering to edit a dimension.
      const on = (p: Point) => C.stating(view.sketch, new C.PointOnCircle(p, cc)) !== null;
      if (on(ln.p1)) return view.addConstraints(new C.TangentLineCircleAt(ln, cc, 'p1'));
      if (on(ln.p2)) return view.addConstraints(new C.TangentLineCircleAt(ln, cc, 'p2'));
    }
    view.addConstraints(new C.TangentLineCircle(ln, cc));
  } else if (circles.length === 2 && !lines.length) {
    // the sense is left out: the core reads it off the geometry, the same rule everywhere
    view.addConstraints(new C.TangentCircleCircle(circles[0], circles[1]));
  } else {
    need(false, 'a line and a circle/arc/curve, a curve and a circle, or two circles/arcs');
  }
}

/** One circle or arc takes its radius; several are opened together and all take the number
 *  that is typed, whichever of their callouts it is typed on. */
function cRadius(circles: (Circle | Arc)[]): void {
  dimension(circles.map((cc) => new C.Radius(cc, Math.abs(cc.radius.value))));
}
