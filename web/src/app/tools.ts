/* The drawing tools: what a click means while one is down, and how a run of clicks ends.
 * A tool that makes geometry as it goes takes its undo snapshot on the first click of a run;
 * the fit tool makes nothing until it finishes and takes its own there. */
import * as C from '../core/constraints.js';
import { Axis, Line, Point, distanceBetween, onRadius } from '../core/model.js';
import type { Primitive } from '../core/model.js';
import type { AxisOn, Edit } from '../core/program.js';
import { PICK_PX } from './view.js';
import type { Place, SketchView, Tool } from './view.js';

/** What the datum tools ask for, said when one is taken up: they draw nothing until it is
 *  given, so the status line is the only place that says what a click will do. */
const HINTS: Partial<Record<Tool, string>> = {
  plane: 'click the line or axis the plane runs along, then the one that says which way is up '
    + 'in it',
  axis: 'click a line for an axis along it, or a point — then another point, or the plane the '
    + 'axis stands square to',
};

export function setTool(v: SketchView, tool: Tool): void {
  v.tool = tool;
  v.pending = [];
  v.pendingFit = [];
  if (tool !== 'plane') v.planeSpec = null;      // armed for one plane, and it was not drawn
  v.firstPick = null;
  v.canvas.classList.toggle('select', tool === 'select');
  v.canvas.style.cursor = '';                // drop any hover affordance
  v.onTool(tool);
  const hint = HINTS[tool];
  if (hint) v.onStatus(hint);
  v.draw();
}

/** Turn the places the fit tool has collected into a curve through them.
 *
 *  Those places are construction input, not sketch points — the same bargain the three-point
 *  arc strikes with its third click.  What comes back is an ordinary curve with an ordinary
 *  control polygon, so everything that edits a drawn one edits this one; a user who wants it
 *  to *keep* passing through somewhere says so with a point and the Coincident button. */
/** Commit whichever curve tool is collecting — Enter, for the two tools whose click count is
 *  not known in advance.  Which one it is lives here, with the pending state. */
export function finishCurve(v: SketchView): void {
  if (v.tool === 'spline') finishSpline(v);
  else if (v.tool === 'splinefit') finishSplineFit(v);
}

/** Every point minted since there were `n0` is drawn in the current plane.  Called at the two
 *  places a tool mints one — a click that found nothing to snap to, and a fitted curve's
 *  control polygon — so the membership is in the sketch before the source catches up, and
 *  `reconcile` writes the `in` clause.  A snapped point already exists and is left where it is:
 *  it was drawn somewhere on purpose. */
export function joinPlane(v: SketchView, n0: number): void {
  if (!v.plane) return;
  for (const p of v.sketch.points.slice(n0)) p.plane = v.plane;
}

export function finishSplineFit(v: SketchView): void {
  if (v.tool !== 'splinefit') return;
  const min = C.curveInfo().minCtrl;
  if (v.pendingFit.length < min) {
    v.onStatus(`a curve needs ${min} points; ${v.pendingFit.length} placed`);
    return;
  }
  v.pushUndo();
  // A click that landed on a point meant that point, not a place that happens to be under it:
  // the curve should *stay* through it when either is moved.  The core makes those contacts,
  // because it is the one that knows where along the curve each point ended up — and pinning
  // that is what leaves a curve fitted to constrained points fully constrained.
  const n0 = v.sketch.points.length;
  const made = v.sketch.splineThrough(v.pendingFit.map((f) => f.at),
                                         v.pendingFit.map((f) => f.on));
  if (!made) {
    v.dropUndo();
    v.onStatus('no curve passes through those points — are any of them on top of another?');
    return;
  }
  joinPlane(v, n0);                    // the control polygon is minted, so it is drawn here too
  const held = v.pendingFit.filter((f) => f.on).length;
  if (held) {
    v.onStatus(`curve through ${v.pendingFit.length} points, ${held} held`);
  }
  v.pendingFit = [];
  v.releasePlan();
  v.afterEdit();
}

/** Turn the control points the spline tool has collected into a curve.  Fewer than a cubic
 *  needs is not an error: the points stay, so one more click finishes it. */
export function finishSpline(v: SketchView): void {
  if (v.tool !== 'spline') return;
  const min = C.curveInfo().minCtrl;
  if (v.pending.length < min) {
    v.onStatus(`a curve needs ${min} control points; ${v.pending.length} placed`);
    return;
  }
  if (!v.sketch.spline(v.pending)) {
    v.onStatus('those control points do not make a curve');
    return;
  }
  v.pending = [];
  v.releasePlan();
  v.afterEdit();
}

/** Escape, in stages: stop a DOF animation, then drop the points the active tool has
 *  collected so far, then leave the tool for Select.  Repeated presses always end up
 *  somewhere calmer rather than doing nothing. */
export function cancelTool(v: SketchView): void {
  if (v.anim) {
    v.stopAnimation();
    return;
  }
  // a first click a datum tool is holding: let it go and keep the tool, as a curve's points are
  if (v.pending.length || v.pendingFit.length || v.firstPick) {
    v.pending = [];
    v.pendingFit = [];
    v.firstPick = null;
    v.draw();
    return;
  }
  if (v.tool !== 'select') setTool(v, 'select');
}
export function snapOrNew(v: SketchView, sp: [number, number]): Point {
  const on = v.pickPoint(sp[0], sp[1]);
  if (on) return on;
  const n0 = v.sketch.points.length;
  const p = v.sketch.point(...v.s2w(sp[0], sp[1]));
  joinPlane(v, n0);
  return p;
}

/** Where a click asks for, and the Point it landed on if it landed on one.  Both curve tools
 *  and the three-point arc share this rule: a click that found a point *meant* that point, so
 *  the geometry should be held to it rather than merely built near it. */
export function pickPlace(v: SketchView, sp: [number, number]): Place {
  const on = v.pickPoint(sp[0], sp[1]);
  return { at: on ? on.xy : v.s2w(sp[0], sp[1]), on };
}

/** Seed a fresh `Rectangle` instance's corners where the gesture drew them.  The dragged span
 *  is already a solution of the component — the sides measure w and h and the corners are
 *  square — so the solve keeps it there.  The pose lives in the session only: an instance's
 *  geometry is written in the component's terms, and a pose written there would be a pose on
 *  every instance — the bargain every component interior strikes. */
function placeRectangle(
  v: SketchView,
  name: string,
  x0: number,
  y0: number,
  x1: number,
  y1: number,
): void {
  const corners: [string, number, number][] = [
    ['l1.p1', x0, y0],
    ['l1.p2', x1, y0],
    ['l2.p2', x1, y1],
    ['l3.p2', x0, y1],
  ];
  for (const [part, x, y] of corners) seedNamed(v, `${name}.${part}`, x, y);
  v.afterEdit();
}

/** Put the point the source calls `name` at (x, y) — how a tool that wrote a statement with
 *  its children implicit then says where they go, the elaboration having named them by their
 *  dotted path.  A name that reaches no point is left alone. */
export function seedNamed(v: SketchView, name: string, x: number, y: number): void {
  const p = v.doc.entity(name);
  if (p instanceof Point) {
    p.x.value = x;
    p.y.value = y;
  }
}

/** Write a fresh plane over the two lines picked — `plane(u: a, v: b)`, right along the first and
 *  up along what is left of the second — then make it the plane being drawn in.  Where it
 *  stands is its origin's, which floats until something places it. */
/** What a datum tool's click picked, and the name the source writes it by: refused with the
 *  cause when it is a line, an axis or a point the source calls nothing.  `undefined` name for
 *  anything else picked, or nothing. */
function namedPick(v: SketchView, sp: [number, number], what: string)
  : { picked: Primitive | null; name?: string } | null {
  const picked = v.pick(sp[0], sp[1]);
  const name = picked ? v.doc.nameOf(picked) : undefined;
  if (picked && !name && (picked instanceof Line || picked instanceof Axis
                          || picked instanceof Point)) {
    v.onStatus(`that ${picked.kind} has no name in the source to write the ${what} over: name it `
      + 'first');
    return null;
  }
  return { picked, name };
}

/** Take an edit a datum tool computed and select what it made. */
function takeMade(v: SketchView, e: Edit, what: string): void {
  if (!v.apply(e, `${what} ${e.names[0]}`)) return;
  const made = v.doc.entity(e.names[0]);
  if (made) v.selected = [made];     // a plane selected alone is the current one: the setter
  v.onSelect();
  v.onChanged();
  v.draw();
}

/** The plane tool's two clicks pick lines or axes already drawn and named: the one the plane runs
 *  along, then the one that says which way is up in it.  Armed for one plane, and put down after
 *  it: a second would reuse a name it was given. */
function planeClick(v: SketchView, sp: [number, number]): void {
  const at = namedPick(v, sp, 'plane');
  if (!at) return;
  const { picked, name } = at;
  if (!(picked instanceof Line || picked instanceof Axis) || !name) {
    v.onStatus('a plane is picked over two lines or axes: click one');
    return;
  }
  const first = v.firstPick;
  if (!first) {
    v.firstPick = name;
    v.onStatus(`the plane runs along ${name}: now click the line or axis that says which way `
      + 'is up');
    v.draw();
  } else if (first === name) {
    v.onStatus('a plane needs two lines or axes: click another');
  } else {
    v.firstPick = null;
    takeMade(v, v.doc.addEntity('plane', [first, name], [], v.planeSpec?.name), 'plane');
    setTool(v, 'select');
  }
}

/** The axis tool: a line is an axis along it at once; a point waits for a second click — another
 *  point, for the axis through both, or anywhere else, for the axis through it square to the
 *  plane there (the pane under the pointer, else the one being drawn on).  The tool stays down
 *  for the next. */
function axisClick(v: SketchView, sp: [number, number]): void {
  const at = namedPick(v, sp, 'axis');
  if (!at) return;
  const { picked, name } = at;
  const from = v.firstPick;
  let on: AxisOn;
  if (!from) {
    if (picked instanceof Point && name) {
      v.firstPick = name;
      v.onStatus(`the axis passes through ${name}: now click another point, or the plane it `
        + 'stands square to');
      v.draw();
      return;
    }
    if (!(picked instanceof Line && name)) {
      v.onStatus('an axis runs along a line or through a point: click one');
      return;
    }
    on = { line: name };
  } else if (picked instanceof Point && name) {
    if (name === from) {
      v.onStatus('an axis through two points needs another: click it, or the plane instead');
      return;
    }
    on = { points: [from, name] };
  } else {
    const plane = v.paneAt(sp[0], sp[1]) ?? (v.plane ? v.doc.nameOf(v.plane) : undefined);
    if (!plane) {
      v.onStatus('there is no plane here for the axis to stand square to: click a pane, or a '
        + 'point');
      return;
    }
    on = { plane, point: from };
  }
  v.firstPick = null;
  takeMade(v, v.doc.addAxis(on), 'axis');
}

export function toolClick(v: SketchView, sp: [number, number]): void {
  // a standard plane chosen before the document had it is brought in by the first press: the
  // document gains `use std`, and what the press makes is drawn on the plane it now has
  // the plane tool picks lines where they are seen, and draws nothing on a plane
  if (v.tool === 'plane') {
    planeClick(v, sp);
    return;
  }
  // and so does the axis tool: it picks what is drawn and writes a statement over it
  if (v.tool === 'axis') {
    axisClick(v, sp);
    return;
  }
  if (!v.ensurePlane()) return;
  // a plane seen edge on has no place on it under the pointer (`toolView` is the one read)
  if (!v.viewCam().readable()) {
    v.onStatus(`${v.planeName} is seen edge on — turn the view, or choose the plane again to face it`);
    return;
  }
  const sk = v.sketch;
  // Tools that make geometry as they go take their snapshot on the first click of a run.  The
  // fit tool makes nothing until it finishes and takes its own there, so pushing here would
  // leave an undo entry — and a whole document serialised — per click that changed nothing.
  // The two tools that write a statement (`rect`, `plane`) take theirs in `apply`.
  if (v.tool !== 'splinefit' && v.tool !== 'rect' && !v.pending.length) {
    v.pushUndo();
  }
  if (v.tool === 'point') {
    snapOrNew(v, sp);
  } else if (v.tool === 'line') {
    const p = snapOrNew(v, sp);
    if (v.pending.length && p !== v.pending[v.pending.length - 1]) sk.line(v.pending[v.pending.length - 1], p);
    v.pending = [p];                            // continue the polyline
  } else if (v.tool === 'rect') {
    // a rectangle is a *component instance*: the gesture measures width and height, the
    // document gains `rN := Rectangle(w: …, h: …)` — and the component itself, the first time —
    // and the instance is then seeded where the gesture drew it.  The first click is a place,
    // not a point: the statement's only numbers are the two lengths.
    if (!v.pendingFit.length) {
      v.pendingFit = [pickPlace(v, sp)];
    } else {
      const [x0, y0] = v.pendingFit[0].at;
      const [x1, y1] = v.s2w(sp[0], sp[1]);
      v.pendingFit = [];
      // the instance joins the current plane whole — `in top` on its one statement —
      // reaching every point the component makes, which per-point membership cannot
      const e = v.doc.addRectangle(
        Math.abs(x1 - x0) || 1,
        Math.abs(y1 - y0) || 1,
        (v.plane && v.doc.nameOf(v.plane)) || '',
      );
      if (v.apply(e, `${e.names[0]}: Rectangle`)) {
        placeRectangle(v, e.names[0], x0, y0, x1, y1);
      }
    }
  } else if (v.tool === 'circle') {
    if (!v.pending.length) {
      v.pending = [snapOrNew(v, sp)];
    } else {
      const [x, y] = v.s2w(sp[0], sp[1]);
      const c = v.pending[0];
      sk.circle(c, Math.hypot(x - c.x.value, y - c.y.value) || 1);
      v.pending = [];
    }
  } else if (v.tool === 'arc3') {
    // two endpoints, then a point the arc must pass through.  That third click is
    // construction input, not a sketch point — the circumcircle is what it produces.
    if (v.pending.length < 2) {
      const p = snapOrNew(v, sp);
      if (!v.pending.length || p !== v.pending[0]) v.pending.push(p);
    } else {
      // the snap indicator is painted for every drawing tool, so honour it here: landing
      // the third click on a real point means the arc should stay on it, not merely start
      // out near it
      const [a, b] = v.pending;
      const { at, on } = pickPlace(v, sp);
      // the centre is minted by the *core* — the circumcircle's, not a click's — so the
      // bracket goes round the construction rather than round a `snapOrNew`: an arc whose
      // ends were in the view and whose centre was on the page is a straddling statement
      // no `in` clause can say
      const n0 = sk.points.length;
      const arc = sk.arcThrough(a, b, at);
      joinPlane(v, n0);
      if (!arc) {
        v.onStatus('those three points are collinear — pick a point off the chord');
        return;                                    // keep the two ends, let them try again
      }
      if (on) sk.add(new C.PointOnCircle(on, arc));
      v.pending = [];
    }
  } else if (v.tool === 'splinefit') {
    // places the curve must pass through, not points of the sketch: snapping still works, but
    // what is recorded is where, so the tool leaves nothing behind if it is abandoned
    const place = pickPlace(v, sp);
    const last = v.pendingFit[v.pendingFit.length - 1];
    const near = last
      && Math.hypot(place.at[0] - last.at[0], place.at[1] - last.at[1]) < v.world(PICK_PX);
    if (near) {
      finishSplineFit(v);
      return;
    }
    v.pendingFit.push(place);
    const min = C.curveInfo().minCtrl;
    const held = v.pendingFit.filter((f) => f.on).length;
    v.onStatus(v.pendingFit.length < min
      ? `${min - v.pendingFit.length} more point(s) for a curve`
      : `Enter to finish${held ? `; ${held} held by a point` : ''}`);
    v.draw();
    return;
  } else if (v.tool === 'spline') {
    // a control polygon is as long as the user wants it, so the tool collects points until
    // it is finished — Enter, or a click back on the last one
    const p = snapOrNew(v, sp);
    if (p === v.pending[v.pending.length - 1]) {
      finishSpline(v);
      return;
    }
    v.pending.push(p);
    const min = C.curveInfo().minCtrl;
    v.onStatus(v.pending.length < min
      ? `${min - v.pending.length} more control point(s) for a curve`
      : 'Enter, or click the last point again, to finish the curve');
    v.draw();
    return;                                        // still collecting: nothing to solve yet
  } else if (v.tool === 'arc') {
    const existing = v.pickPoint(sp[0], sp[1]) !== null;
    v.pending.push(snapOrNew(v, sp));
    if (v.pending.length === 3) {
      const [cpt, s, en] = v.pending;
      if (new Set([cpt, s, en]).size === 3) {
        if (!existing) {                           // freshly placed end point: put it on the radius
          const q = onRadius(...cpt.xy, ...en.xy, distanceBetween(cpt, s));
          if (q) [en.x.value, en.y.value] = q;
        }
        sk.arc(cpt, s, en);
      }
      v.pending = [];
    }
  }
  v.releasePlan();
  v.afterEdit();
}
