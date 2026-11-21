/* The drawing tools: what a click means while one is down, and how a run of clicks ends.
 * A tool that makes geometry as it goes takes its undo snapshot on the first click of a run;
 * the fit tool makes nothing until it finishes and takes its own there. */
import * as C from '../core/constraints.js';
import * as io from '../core/io.js';
import { Line, Point, distanceBetween, onRadius } from '../core/model.js';
import { eyeFrame } from '../core/workspace.js';
import { bodiesPicked, onePicked } from './solids.js';
import { applyHidden, toolKinds } from './hidden.js';
import { combines, picksAs, roundsEdges } from './view.js';
import type { Place, SketchView, Tool } from './view.js';

export function setTool(v: SketchView, tool: Tool): void {
  const shown = v.kinds | toolKinds(v.tool);
  v.tool = tool;
  // **a tool shows the kinds it needs while it is down** (#678), and they go with it
  if ((v.kinds | toolKinds(tool)) !== shown) {
    applyHidden(v);
    v.onHidden();
  }
  v.pending = [];
  v.pendingFit = [];
  v.hoverRegion = null;
  v.pendingRevolve = null;
  v.highlight = [];
  v.hovered = null;                          // what the next move lights is the new tool's
  v.endRound();                              // an edge tool's number goes with it
  // the selection is where a picking tool starts: a Boolean's bodies, an edge tool's parts of
  // one object
  const takes = picksAs(tool);
  if (takes === 'bodies') bodiesPicked(v);
  else if (takes === 'parts') onePicked(v);
  v.endExtent();                             // an extrusion being sized is done with the tool
  v.canvas.classList.toggle('select', tool === 'select');
  v.canvas.style.cursor = '';                // drop any hover affordance
  v.onTool(tool);
  v.draw();
}

/** Turn the places the fit tool has collected into a curve through them.
 *
 *  Those places are construction input, not sketch points — the same bargain the three-point
 *  arc strikes with its third click.  What comes back is an ordinary curve with an ordinary
 *  control polygon, so everything that edits a drawn one edits this one; a user who wants it
 *  to *keep* passing through somewhere says so with a point and the Coincident button. */
/** Commit whichever tool is collecting — Enter, for the tools whose click count is not known in
 *  advance: the two curve tools, and a Boolean's tools.  Which one it is lives here, with the
 *  pending state. */
export function finishCurve(v: SketchView): void {
  if (v.tool === 'spline') finishSpline(v);
  else if (v.tool === 'splinefit') finishSplineFit(v);
  else if (combines(v.tool)) v.finishCombine();
  else if (roundsEdges(v.tool)) v.finishRound();
}

/** Every point minted since there were `n0` is drawn in the tool's plane, where it draws in
 *  one (`toolPlane`).  Called at the two places a tool mints one — a click that found nothing
 *  to snap to, and a fitted curve's control polygon — so the membership is in the sketch before
 *  the source catches up, and `reconcile` writes the `in` clause.  A snapped point already
 *  exists and is left where it is: it was drawn somewhere on purpose. */
export function joinPlane(v: SketchView, n0: number): void {
  const pl = v.toolPlane;
  if (!pl) return;
  for (const p of v.sketch.points.slice(n0)) p.plane = pl;
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
  // over points in space the elaborator draws it as a curve in space (`splines_in_space`), which
  // the live sketch's spline is not: written, it is read again
  const inSpace = v.pending.some((p) => v.inSpace(p));
  v.pending = [];
  v.releasePlan();
  v.afterEdit();
  if (inSpace) v.rereadSource();
}

/** Escape, in stages: stop a DOF animation, then drop the points the active tool has
 *  collected so far, then leave the tool for Select.  Repeated presses always end up
 *  somewhere calmer rather than doing nothing. */
export function cancelTool(v: SketchView): void {
  if (v.anim) {
    v.stopAnimation();
    return;
  }
  if (v.pending.length || v.pendingFit.length) {
    v.pending = [];
    v.pendingFit = [];
    v.draw();
    return;
  }
  // a region waiting for the line to turn it about: let it go, the tool still down
  if (v.pendingRevolve) {
    v.pendingRevolve = null;
    v.highlight = [];
    v.draw();
    return;
  }
  // a Boolean's bodies: its tools let go, then its target, the tool still down
  if (combines(v.tool) && v.selectedSolids.length) {
    const [target, ...tools] = v.selectedSolids;
    if (tools.length) v.pickSolid(target);
    else v.selected = [];
    v.onSelect();
    v.onChanged();
    v.draw();
    return;
  }
  // what an edge tool picked: let go, the tool still down
  if (roundsEdges(v.tool) && v.selectedSolids.length) {
    v.selected = [];
    v.endRound();
    v.tellRound();
    v.onSelect();
    v.onChanged();
    v.draw();
    return;
  }
  // an extrusion being sized: done sizing, the tool still down for the next
  if (v.liveExtent) {
    v.endExtent();
    v.draw();
    return;
  }
  if (v.tool !== 'select') setTool(v, 'select');
  // nothing else to back out of: the part open is closed (#679)
  else v.closePart();
}
/** **Whether a tool still draws in a plane.**  Every tool draws in space, what it mints standing
 *  in no plane where the eye sees the click — but these, until the core can draw them there: a
 *  three-point arc and a curve fitted through places in space, and a rectangle.  They draw in
 *  the standard plane the eye is most square to (`SketchView.toolPlane`). */
export function drawsInPlane(tool: Tool): boolean {
  return tool === 'rect' || tool === 'arc3' || tool === 'splinefit';
}

/** The point a tool's click lands on, if it lands on one it can draw from: any, in space — but
 *  a tool that draws in a plane does not take a point in space, which a circle, a three-point
 *  arc or a rectangle in that plane could not be drawn about. */
function snapTarget(v: SketchView, sp: [number, number]): Point | null {
  const on = v.pickPoint(sp[0], sp[1]);
  return on && v.toolPlane && v.inSpace(on) ? null : on;
}

export function snapOrNew(v: SketchView, sp: [number, number]): Point {
  const on = snapTarget(v, sp);
  if (on) return on;
  // in no plane: in space, where the eye sees the click
  if (!v.toolPlane) {
    return v.sketch.pointSeen(v.orbit.az, v.orbit.el, ...v.s2w(sp[0], sp[1]), v.viewCam().pixel);
  }
  const n0 = v.sketch.points.length;
  const p = v.sketch.point(...v.place(sp[0], sp[1]));
  joinPlane(v, n0);
  return p;
}

/** Where a click asks for, and the Point it landed on if it landed on one.  Both curve tools
 *  and the three-point arc share this rule: a click that found a point *meant* that point, so
 *  the geometry should be held to it rather than merely built near it. */
export function pickPlace(v: SketchView, sp: [number, number]): Place {
  const on = snapTarget(v, sp);
  return { at: on ? on.xy : v.place(sp[0], sp[1]), on };
}

/** Seed a fresh `Rectangle` instance's corners where the gesture drew them.  The dragged span
 *  is already a solution of the component — the sides measure w and h, the corners are square
 *  and the first side is level — so the solve keeps it there.  `corner` is the point the
 *  instance is given, the drawing's own, so its place is the document's to keep; the other
 *  three are the instance's, written in the component's terms. */
function placeRectangle(
  v: SketchView,
  name: string,
  corner: string,
  x0: number,
  y0: number,
  x1: number,
  y1: number,
): void {
  const corners: [string, number, number][] = [
    ['l1.p2', x1, y0],
    ['l2.p2', x1, y1],
    ['l3.p2', x0, y1],
  ];
  seedNamed(v, corner, x0, y0);
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

/** A press with a tool down.  `alt` and `shift` are the press's modifiers: an extrusion or a
 *  revolution with a body selected cuts it with ⌥, and an extrusion cuts through all of it with
 *  ⌥⇧. */
export function toolClick(v: SketchView, sp: [number, number],
                          mods: { alt?: boolean; shift?: boolean } = {}): void {
  if (takenElsewhere(v, sp, mods)) return;
  // Tools that make geometry as they go take their snapshot on the first click of a run.  The
  // fit tool makes nothing until it finishes and takes its own there, so pushing here would
  // leave an undo entry — and a whole document serialised — per click that changed nothing.
  // The two tools that write a statement (`rect`, `plane`) take theirs in `apply`.
  if (v.tool !== 'splinefit' && v.tool !== 'rect' && !v.pending.length) {
    v.pushUndo();
  }
  if (!drawClick(v, sp)) return;
  v.releasePlan();
  v.afterEdit();
}

/** A press the drawing tools never see — a Boolean's, an edge tool's, a sweep's, or one refused
 *  for want of a plane to draw in.  Whether it was taken. */
function takenElsewhere(v: SketchView, sp: [number, number],
                        mods: { alt?: boolean; shift?: boolean }): boolean {
  // a word's slot armed: the nearest thing it takes fills it — or, where it places a point, a
  // click on nothing does
  if (v.tool === 'word') {
    const pick = v.wordPick;
    if (!pick) return true;
    const hit = v.hitsAt(sp[0], sp[1]).find((h) => !('hidden' in h && h.hidden) && pick.accepts(h));
    if (hit || pick.mints) pick.take(hit ?? null, sp);
    return true;
  }
  // a Boolean picks bodies where the eye sees them, on no plane
  if (combines(v.tool)) {
    v.combineAt(sp[0], sp[1]);
    return true;
  }
  // and an edge tool its edges and faces
  if (roundsEdges(v.tool)) {
    v.roundAt(sp[0], sp[1]);
    return true;
  }
  // an extrusion writes its statements through `apply`, which takes its own undo step
  if (v.tool === 'extrude') {
    v.extrudeAt(sp[0], sp[1], mods.alt, mods.alt && mods.shift);
    return true;
  }
  if (v.tool === 'revolve') {
    v.revolveAt(sp[0], sp[1], mods.alt);
    return true;
  }
  // a tool that cannot draw in space yet draws in the standard plane the eye is most square to,
  // brought in by the first press where the document does not say `use std` yet
  if (!v.ensureToolPlane()) return true;
  return false;
}

/** A drawing tool's click.  Whether the drawing changed and is to be solved: a tool still
 *  collecting clicks, or one that refused the click, says not. */
function drawClick(v: SketchView, sp: [number, number]): boolean {
  switch (v.tool) {
    case 'point':
      snapOrNew(v, sp);
      return true;
    case 'line':
      lineClick(v, sp);
      return true;
    case 'rect':
      rectClick(v, sp);
      return true;
    case 'circle':
      return circleClick(v, sp);
    case 'arc3':
      return arc3Click(v, sp);
    case 'splinefit':
      splineFitClick(v, sp);
      return false;
    case 'spline':
      splineClick(v, sp);
      return false;                                // still collecting: nothing to solve yet
    case 'arc':
      return arcClick(v, sp);
    default:
      return true;
  }
}

function lineClick(v: SketchView, sp: [number, number]): void {
  const p = snapOrNew(v, sp);
  if (v.pending.length && p !== v.pending[v.pending.length - 1]) {
    v.sketch.line(v.pending[v.pending.length - 1], p);
  }
  v.pending = [p];                                 // continue the polyline
}

/** A rectangle is a *component instance*: the gesture measures width and height, the document
 *  gains a corner and `rN := Rectangle(pN, w: …, h: …)` — and the component itself, the first
 *  time — and the instance is then seeded where the gesture drew it.  The first click is a
 *  place, not a point: the statement's only numbers are the two lengths. */
function rectClick(v: SketchView, sp: [number, number]): void {
  if (!v.pendingFit.length) {
    v.pendingFit = [pickPlace(v, sp)];
    return;
  }
  const [x0, y0] = v.pendingFit[0].at;
  const [x1, y1] = v.place(sp[0], sp[1]);
  v.pendingFit = [];
  // the instance joins the tool's plane whole — `in top` on its one statement —
  // reaching every point the component makes, which per-point membership cannot
  const e = v.doc.addRectangle(
    v.placeLength(Math.abs(x1 - x0)) || 1,
    v.placeLength(Math.abs(y1 - y0)) || 1,
    v.facingPlane(),
  );
  if (v.apply(e, `${e.names[0]}: Rectangle`)) {
    placeRectangle(v, e.names[0], e.names[1], x0, y0, x1, y1);
  }
}

/** Centre, then a point on it.  In space the circle stands square to the eye's line of sight,
 *  its radius as the eye sees it, and is written — `circle(center: c, normal: hint(dir: (…)))`,
 *  an axis of its own seeded along the sight, free for its constraints to turn — the elaborator
 *  drawing it in the plane through its centre (`circles_in_space`).  False where it was written:
 *  the edit took its own solve. */
function circleClick(v: SketchView, sp: [number, number]): boolean {
  if (!v.pending.length) {
    v.pending = [snapOrNew(v, sp)];
    return true;
  }
  const [x, y] = v.s2w(sp[0], sp[1]);
  const c = v.pending[0];
  v.pending = [];
  if (!v.toolPlane) {
    // where the eye sees the centre, on the same picture plane the click is read on
    const [cx, cy] = v.s2w(...v.seen(c));
    const r = v.placeLength(Math.hypot(x - cx, y - cy)) || 1;
    return !writeInSpace(v, 'circle', { center: c },
                         { normal: `hint(dir: (${sight(v).join(', ')}))`, r: String(r) });
  }
  v.sketch.circle(c, v.placeLength(Math.hypot(x - c.x.value, y - c.y.value)) || 1);
  return true;
}

/** The eye's line of sight, toward it, to the digits a direction is written with. */
function sight(v: SketchView): [number, number, number] {
  return eyeFrame(v.orbit.az, v.orbit.el).toward.map((x) => io.placed(x, 1e-4)) as
    [number, number, number];
}

/** Two endpoints, then a point the arc must pass through.  That third click is construction
 *  input, not a sketch point — the circumcircle is what it produces.  False where the three
 *  are collinear: the two ends are kept, to try again. */
function arc3Click(v: SketchView, sp: [number, number]): boolean {
  const sk = v.sketch;
  if (v.pending.length < 2) {
    const p = snapOrNew(v, sp);
    if (!v.pending.length || p !== v.pending[0]) v.pending.push(p);
    return true;
  }
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
    return false;                                  // keep the two ends, let them try again
  }
  if (on) sk.add(new C.PointOnCircle(on, arc));
  v.pending = [];
  return true;
}

/** Places the curve must pass through, not points of the sketch: snapping still works, but
 *  what is recorded is where, so the tool leaves nothing behind if it is abandoned. */
function splineFitClick(v: SketchView, sp: [number, number]): void {
  const place = pickPlace(v, sp);
  const last = v.pendingFit[v.pendingFit.length - 1];
  const near = last
    && Math.hypot(place.at[0] - last.at[0], place.at[1] - last.at[1]) < v.world(v.pickPx);
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
}

/** A control polygon is as long as the user wants it, so the tool collects points until it is
 *  finished — Enter, or a click back on the last one. */
function splineClick(v: SketchView, sp: [number, number]): void {
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
}

/** Centre, start, end.  In a plane the arc is drawn at once, its end put on the radius; in space
 *  it is written — `arc(center: c, start: s, end: e)`, no `in` — and the elaborator draws it in
 *  the plane through its three points, the short way round (`arcs_in_space`), its end held to
 *  the radius by the arc's own rows.  False where the arc was written: the edit took its own
 *  solve. */
function arcClick(v: SketchView, sp: [number, number]): boolean {
  const existing = snapTarget(v, sp) !== null;
  v.pending.push(snapOrNew(v, sp));
  if (v.pending.length !== 3) return true;
  const [cpt, s, en] = v.pending;
  v.pending = [];
  if (new Set([cpt, s, en]).size !== 3) return true;
  if (!v.toolPlane) {
    // a freshly placed end point put on the radius, as in a plane: the solve then moves nothing
    if (!existing) v.sketch.ontoRadius(cpt, s, en);
    return !writeInSpace(v, 'arc', { center: cpt, start: s, end: en });
  }
  if (!existing) {                                 // freshly placed end point: put it on the radius
    const q = onRadius(...cpt.xy, ...en.xy, distanceBetween(cpt, s));
    if (q) [en.x.value, en.y.value] = q;
  }
  v.sketch.arc(cpt, s, en);
  return true;
}

/** **A figure over points in space, written as its statement**: the word `id` over `points`, by
 *  slot, and `args` beside them — the points brought into the source first, so the statement can
 *  name them, and the figure's edit folded into the undo step the tool's first click took.  The
 *  elaborator draws it in the plane it mints (`arcs_in_space`, `circles_in_space`).  Whether it
 *  was written. */
function writeInSpace(v: SketchView, id: string, points: Record<string, Point>,
                      args: Record<string, string> = {}): boolean {
  v.syncSource();
  const named = Object.entries(points).map(([slot, p]) => [slot, v.doc.nameOf(p)] as const);
  if (named.some(([, n]) => !n)) {
    v.onStatus(`the ${id}'s points have no names in the source to write it over`);
    return false;
  }
  const r = { id, args: { ...Object.fromEntries(named), ...args } as Record<string, string> };
  if (!v.apply(v.doc.writeWord(r))) return false;
  v.dropUndo();                    // one step with the points: the tool's first click took it
  return true;
}
