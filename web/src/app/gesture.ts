/* The pointer: which gesture a press starts, and what each one does while it lasts.  One
 * pointer owns the gesture until it lets go — a second finger starting one on top would drop
 * the live gesture on the floor, leaking the core's drag handle — and every gesture owns its
 * own state, so the pointer handlers stay two lines each. */
import * as dim from '../core/callout.js';
import { Constraint } from '../core/constraints.js';
import { PlanDrag } from '../core/decompose.js';
import {
  Arc, Box, Circle, Ellipse, Param, Point, Primitive, Spline, ellipseMinor,
} from '../core/model.js';
import { RadiusDrag } from '../core/system.js';
import { moveDimension, placeDimension } from './dimension.js';
import { COL } from './paint.js';
import { insertControl } from './edit.js';
import { cancelTool, toolClick } from './tools.js';
import { bodyAt, grabBody, grabHandle, handleAt } from './underlay.js';
import type { SketchView } from './view.js';

/** One pointer gesture in progress.  `move` gets canvas coordinates; `end` and `paint` are
 *  optional because pan needs neither and the rubber band needs both. */
export interface Gesture {
  move(sp: [number, number]): void;
  /** Finish and commit whatever the gesture produced. */
  end?(): void;
  /** Drop it without committing — the sketch was replaced underneath. */
  abandon?(): void;
  paint?(ctx: CanvasRenderingContext2D): void;
  /** Nothing about the sketch or the selection changed, so releasing needs no refresh. */
  transient?: boolean;
  /** The geometry moved, so the diagnosis no longer describes the pose on screen. */
  movedGeometry?: boolean;
}

export function local(v: SketchView, e: MouseEvent): [number, number] {
  const r = v.canvas.getBoundingClientRect();
  return [e.clientX - r.left, e.clientY - r.top];
}

export function bindEvents(v: SketchView): void {
  const cv = v.canvas;
  // One pointer owns the gesture until it lets go.  A second finger starting one on top would
  // drop the live gesture on the floor: its `end` never runs, so the core's drag handle leaks
  // and the soft drag target it added stays in the sketch, quietly compromising every later
  // solve.  And a gesture can end without a `pointerup` — a cancelled touch, or capture lost
  // to a system gesture — so those have to finish it too.
  cv.addEventListener('pointerdown', (e) => {
    if (v.gesture) return;
    cv.setPointerCapture(e.pointerId);
    v.gesturePointer = e.pointerId;
    onPointerDown(v, e);
  });
  cv.addEventListener('pointermove', (e) => {
    if (v.gesture && e.pointerId !== v.gesturePointer) return;
    onPointerMove(v, e);
  });
  const finish = (e: PointerEvent): void => {
    if (v.gesturePointer !== null && e.pointerId !== v.gesturePointer) return;
    v.gesturePointer = null;
    onPointerUp(v);
  };
  cv.addEventListener('pointerup', (e) => {
    if (v.gesturePointer === null || e.pointerId === v.gesturePointer) {
      cv.releasePointerCapture(e.pointerId);
    }
    finish(e);
  });
  cv.addEventListener('pointercancel', finish);
  cv.addEventListener('lostpointercapture', finish);
  cv.addEventListener('dblclick', (e) => {
    // the same gesture the constraint list uses: double-click a dimension, type a new number
    const sp = local(v, e);
    const c = v.pickCallout(...sp);
    if (c) {
      v.onEditConstraint(c);
      return;
    }
    // and on a curve it asks for another handle where you clicked: the insertion is
    // shape-preserving, so nothing moves — the new control point comes out selected, ready
    // to be dragged, which is the whole reason you asked for it.  Only while selecting: with
    // a tool down a double-click is two clicks of that tool, and talking over it would put a
    // knot in whatever curve happened to be under the second one.
    if (v.tool !== 'select') return;
    const hit = v.pick(...sp);
    if (hit instanceof Spline) insertControl(v, hit, ...v.s2w(sp[0], sp[1]));
  });
  cv.addEventListener('contextmenu', (e) => e.preventDefault());
  cv.addEventListener('wheel', (e) => {
    e.preventDefault();
    const [sx, sy] = local(v, e);
    const f = 1.0015 ** (-e.deltaY * (e.deltaMode === 1 ? 16 : 1));
    v.cam.zoomAt(sx, sy, f);       // about the cursor, so what is under it stays under it
    v.draw();
  }, { passive: false });
}

export function onPointerDown(v: SketchView, e: PointerEvent): void {
  v.stopAnimation();
  const sp = local(v, e);
  v.cursor = sp;
  if (e.button === 1 || e.button === 2) {
    if (v.pending.length) cancelTool(v);
    else v.gesture = panGesture(v, sp);
    return;
  }
  if (e.button !== 0) return;
  // a dimension still following the pointer: this click is what plants it.  The default is
  // refused so the focus stays in its editor — the number is still being typed
  if (v.liveDim?.placing) {
    e.preventDefault();
    placeDimension(v);
    return;
  }
  if (v.tool !== 'select') {
    toolClick(v, sp);
    return;
  }
  // a number still being written, on a pair that can be measured three ways: taking hold of
  // it goes on choosing which, exactly as it did before the click that planted it
  const live = v.liveDim;
  if (live?.alt && v.pickCallout(sp[0], sp[1]) === live.targets[0]) {
    e.preventDefault();                       // the focus stays in the editor
    live.placing = true;                      // carried again, so held still again
    v.gesture = {
      transient: true,
      move: (at) => moveDimension(v, at),
      end: () => placeDimension(v),       // and solved again when it lands
    };
    return;
  }
  const at = whatIsAt(v, sp);
  // a corner of the traced picture: it sizes and turns the picture about its centre
  if (at.kind === 'handle') {
    v.gesture = grabHandle(v, at.corner);
    return;
  }
  // a dimension: the press selects the constraint, and dragging moves the callout rather
  // than the sketch
  if (at.kind === 'callout') {
    v.selected = [];
    v.onPickConstraint(at.callout);
    v.gesture = calloutGesture(v, at.callout, sp);
    v.draw();
    return;
  }
  if (at.kind === 'image') {
    v.pickImage();
    v.gesture = grabBody(v, sp);
    v.onSelect();
    v.onChanged();          // as every other press does: the window and the readout follow it
    v.draw();
    return;
  }
  v.dropImage();
  const ent = at.kind === 'entity' ? at.ent : null;
  if (!ent) {
    // nothing under the cursor: start a rubber band.  A press with no drag still just
    // clears the selection, because an empty box selects nothing.
    if (!e.shiftKey) v.selected = [];
    v.gesture = bandGesture(v, sp);
  } else if (e.shiftKey) {
    const i = v.selected.indexOf(ent);
    if (i >= 0) v.selected.splice(i, 1);
    else v.selected.push(ent);
  } else {
    if (!v.selected.includes(ent)) v.selected = [ent];
    if (ent instanceof Point && canMove(v, ent)) {
      v.pushUndo();
      // on the sketch's own plan, compiled once per topology: the drag starts at once
      const drag = new PlanDrag(v.sketch, ent, ...v.s2w(sp[0], sp[1]), null, 0.05,
                                v.plan());
      v.gesture = pointGesture(v, drag);
    } else if (isResizable(v, ent)) {
      v.pushUndo();
      v.gesture = radiusGesture(v, new RadiusDrag(v.sketch, ent, Math.abs(scalarOf(ent).value)));
    }
  }
  v.onSelect();
  v.onChanged();
  v.draw();
}
export function onPointerMove(v: SketchView, e: PointerEvent): void {
  const sp = local(v, e);
  v.cursor = sp;
  if (v.liveDim?.placing) moveDimension(v, sp);
  else if (v.gesture) v.gesture.move(sp);
  else hover(v, sp);
  v.draw();
}

export function onPointerUp(v: SketchView): void {
  endGesture(v);
}

/** The gesture finished: let it commit, then refresh — unless it changed nothing (a pan
 *  moves the camera, not the sketch). */
export function endGesture(v: SketchView): void {
  const g = v.gesture;
  if (!g) return;
  v.gesture = null;
  g.end?.();
  if (g.movedGeometry) {
    v.staleDiagnosis = true;
    // **the gesture becomes a source edit.**  One writeback, here, at the moment it is let go:
    // during the drag the text is stale and that is correct — a drag is one edit, not sixty.
    v.syncSeeds();
  }
  if (!g.transient) v.onChanged();
  v.draw();
}

/** The sketch is being replaced under a live gesture: drop it without committing, since
 *  `end` would write into a document the gesture never ran on. */
export function abandonGesture(v: SketchView): void {
  const g = v.gesture;
  v.gesture = null;
  v.gesturePointer = null;
  g?.abandon?.();
}

/* -- the gestures.  Each owns its own state; `paint` is for the ones that draw. -- */

export function panGesture(v: SketchView, from: [number, number]): Gesture {
  let last = from;
  return {
    transient: true,                 // the camera moved, not the sketch
    move: (sp) => {
      v.cam.panBy(sp[0] - last[0], sp[1] - last[1]);
      last = sp;
    },
  };
}

export function pointGesture(v: SketchView, drag: PlanDrag): Gesture {
  let reported = 0;
  return {
    movedGeometry: true,
    move: (sp) => {
      v.lastResult = drag.move(...v.s2w(sp[0], sp[1]));
      if (drag.flips.length > reported) {          // only announce new ones
        reported = drag.flips.length;
        v.onStatus(`⚠ solution branch flipped in ${reported} triangle(s) during this drag`);
      }
      v.onDragFrame();
    },
    end: () => {
      drag.end();
      const b = drag.sketch.branches;                 // document state: merge, then write back
      for (const [k, v] of drag.branches()) b.set(k, v);
      drag.sketch.branches = b;
    },
    abandon: () => {
      drag.end();              // disposes the drag's own handles; no branches committed
    },
  };
}

export function radiusGesture(v: SketchView, drag: RadiusDrag): Gesture {
  return {
    movedGeometry: true,
    abandon: () => drag.end(),
    move: (sp) => {
      const [wx, wy] = v.s2w(sp[0], sp[1]);
      const e = drag.circle;
      // a circle's rim follows the cursor at its distance from the centre; an ellipse's by the
      // minor radius that puts the rim through the cursor, which is the same question the tool's
      // third click asked the core
      const r = e instanceof Ellipse
        ? ellipseMinor(...e.center.xy, ...e.major.xy, wx, wy) ?? Math.abs(e.minor.value)
        : Math.hypot(wx - e.center.x.value, wy - e.center.y.value);
      v.lastResult = drag.move(r);
      v.onDragFrame();
    },
    end: () => drag.end(),
  };
}

/** Move a dimension's callout.  The number and its line go where they are put — where the
 *  layout would have placed it is only a first guess — so this is what settles a crowded
 *  drawing.  The placement is document state, so it undoes and saves with everything else; a
 *  press that never moves leaves nothing behind, and puts nothing on the undo stack. */
export function calloutGesture(v: SketchView, c: Constraint, from: [number, number]): Gesture {
  const grip = dim.grab(v.sketch, v.unit, c.id, ...v.s2w(from[0], from[1]));
  let moved = false;
  return {
    transient: true,               // the annotation moved, not the geometry
    move: (sp) => {
      if (!grip) return;
      if (!moved) {                // the first movement is the edit; a bare click is not one
        moved = true;
        v.pushUndo();
      }
      dim.drag(v.sketch, c.id, ...v.s2w(sp[0], sp[1]), grip);
    },
    // where a callout sits is document state, so the drag is a source edit and has to be
    // written down — once, at the release, the same bargain `syncSeeds` strikes for a point
    end: () => { if (moved) v.syncSource(); },
  };
}

export function bandGesture(v: SketchView, from: [number, number]): Gesture {
  const base = [...v.selected];
  let to = from;                     // the gesture owns both corners, so paint reads no globals
  // nothing moves during a selection drag, so the extents are computed once, not per frame
  const extents = v.sketch.primitives().map((e) => [e, e.bounds()] as const);
  return {
    move: (sp) => {
      to = sp;
      // live preview: the canvas shows what would be selected, the status line the count
      v.selected = [...new Set([...base, ...boxContents(v, extents, from, sp)])];
      v.onSelect();
      v.onDragFrame();
    },
    paint: (ctx) => {
      const [x0, y0] = from, [x1, y1] = to;
      ctx.save();
      ctx.setLineDash([5, 4]);
      ctx.strokeStyle = COL.sel;
      ctx.fillStyle = COL.bandFill;
      ctx.lineWidth = 1;
      const rx = Math.min(x0, x1), ry = Math.min(y0, y1);
      ctx.fillRect(rx, ry, Math.abs(x1 - x0), Math.abs(y1 - y0));
      ctx.strokeRect(rx + 0.5, ry + 0.5, Math.abs(x1 - x0), Math.abs(y1 - y0));
      ctx.restore();
    },
  };
}

/** Entities lying entirely inside the box — "window" selection.  "All of it is inside" is
 *  exactly "its bounds are inside", so the caller asks the model for each primitive's extent
 *  (a line's two endpoints, a circle's rim, an arc's sweep) once per gesture. */
export function boxContents(v: SketchView, extents: readonly (readonly [Primitive, Box])[],
                            from: [number, number], to: [number, number]): Primitive[] {
  const a = v.s2w(from[0], from[1]);
  const b = v.s2w(to[0], to[1]);
  const x0 = Math.min(a[0], b[0]), x1 = Math.max(a[0], b[0]);
  const y0 = Math.min(a[1], b[1]), y1 = Math.max(a[1], b[1]);
  return extents.filter(([, bb]) => bb[0] >= x0 && bb[1] >= y0 && bb[2] <= x1 && bb[3] <= y1)
    .map(([e]) => e);
}

/** What is under the pointer, in the order a press offers itself to things.
 *
 *  The order is the whole content of this function and it is stated **once**: a corner handle,
 *  then a dimension callout, then the geometry, then the traced picture, then nothing.  Handles
 *  and callouts are painted over everything and so outrank it; the **drawing outranks the
 *  picture**, so a line lying across a photograph is what a click on that line picks, and the
 *  picture answers only where the geometry did not.
 *
 *  Both readers — the press and the cursor it promises — ask this rather than each walking the
 *  list, because two copies of the order are two orders the moment one of them is edited, and
 *  the failure is a cursor that offers what a click does not do. */
type Target =
  | { kind: 'handle'; corner: number }
  | { kind: 'callout'; callout: Constraint }
  | { kind: 'entity'; ent: Primitive }
  | { kind: 'image' }
  | { kind: 'none' };

function whatIsAt(v: SketchView, sp: [number, number]): Target {
  const corner = handleAt(v, sp);
  if (corner >= 0) return { kind: 'handle', corner };
  const callout = v.pickCallout(sp[0], sp[1]);
  if (callout) return { kind: 'callout', callout };
  const ent = v.pick(sp[0], sp[1]);
  if (ent) return { kind: 'entity', ent };
  if (bodyAt(v, sp)) return { kind: 'image' };
  return { kind: 'none' };
}

/** Cursor affordance: what a press here would grab. */
export function hover(v: SketchView, sp: [number, number]): void {
  if (v.tool !== 'select') return;
  const at = whatIsAt(v, sp);
  v.canvas.style.cursor =
      at.kind === 'handle' ? (at.corner % 2 ? 'nesw-resize' : 'nwse-resize')
    : at.kind === 'callout' || at.kind === 'image' ? 'move'
    : at.kind === 'entity' && at.ent instanceof Point && canMove(v, at.ent) ? 'grab'
    : at.kind === 'entity' && isResizable(v, at.ent) ? 'ew-resize'
    : '';
}

/** Can any part of this entity actually move?  `Diagnosis.underParams` is the Jacobian
 *  null space (or the structural under-block above NUMERIC_MAX), so the cursor promises
 *  only what the solver will deliver rather than reading `fixed` and offering to drag
 *  something a constraint has pinned.
 *
 *  The question is per entity, not per parameter — a point pinned in x but free in y still
 *  slides.  And the null space belongs to the pose it was computed at, so a gesture that
 *  moved geometry marks it stale: we then fall back to "yes", because refusing needs
 *  positive knowledge and a hint should never be a lie in the strict direction. */
export function canMove(v: SketchView, e: Primitive): boolean {
  const free = e.params.filter((p) => !p.fixed);
  if (!free.length) return false;
  if (!v.diagnosis || v.staleDiagnosis) return true;
  if (v.movable?.owner !== v.diagnosis) {
    v.movable = { owner: v.diagnosis, set: new Set(v.diagnosis.underParams) };
  }
  return free.some((p) => v.movable!.set.has(p));
}

/** The one scalar a rim drag moves: a circle's or arc's radius, an ellipse's minor radius. */
export function scalarOf(e: Circle | Arc | Ellipse): Param {
  return e instanceof Ellipse ? e.minor : e.radius;
}

/** A circle, arc or ellipse whose one scalar is free to follow the cursor. */
export function isResizable(v: SketchView, e: Primitive | null): e is Circle | Arc | Ellipse {
  return (e instanceof Circle || e instanceof Arc || e instanceof Ellipse)
    && !scalarOf(e).fixed && (!v.diagnosis || v.staleDiagnosis || canMove(v, e));
}
