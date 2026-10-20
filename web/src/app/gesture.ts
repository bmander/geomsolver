/* The pointer: which gesture a press starts, and what each one does while it lasts.  One
 * pointer owns the gesture until it lets go — a second finger starting one on top would drop
 * the live gesture on the floor, leaking the core's drag handle — and every gesture owns its
 * own state, so the pointer handlers stay two lines each. */
import * as dim from '../core/callout.js';
import { Constraint } from '../core/constraints.js';
import { PlanDrag } from '../core/decompose.js';
import { Scrub } from '../core/parameters.js';
import {
  Arc, Circle, Param, Plane, Point, Primitive, Spline,
} from '../core/model.js';
import { RadiusDrag, TurnDrag } from '../core/system.js';
import { overlappingSeen } from '../core/workspace.js';
import type { View } from '../core/workspace.js';
import { moveDimension, placeDimension } from './dimension.js';
import { COL } from './paint.js';
import { insertControl } from './edit.js';
import { cancelTool, toolClick } from './tools.js';
import { bodyAt, grabBody, grabHandle, handleAt } from './underlay.js';
import { combinePick, roundPick, roundTarget } from './solids.js';
import { bodyOf, picksAs, roundsEdges, sweepsRegion, toggled } from './view.js';
import type { Hit, SketchView, SolidPick } from './view.js';

/** One pointer gesture in progress.  `move` gets canvas coordinates; `end` and `paint` are
 *  optional because pan needs neither and the rubber band needs both. */
export interface Gesture {
  /** The pointer moved; `mods` says what it held, for a gesture a modifier changes. */
  move(sp: [number, number], mods?: { shift: boolean }): void;
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
  const touch = touchNavigation(v);
  bindPointer(v, touch);
  cv.addEventListener('dblclick', (e) => onDoubleClick(v, e));
  cv.addEventListener('contextmenu', (e) => e.preventDefault());
  cv.addEventListener('wheel', (e) => onWheel(v, e), { passive: false });
  bindSafariPinch(v, touch);
}

/** The pointer's own events, a finger's handed to `touch`.
 *
 *  One pointer owns the gesture until it lets go.  A second finger starting one on top would
 *  drop the live gesture on the floor: its `end` never runs, so the core's drag handle leaks
 *  and the soft drag target it added stays in the sketch, quietly compromising every later
 *  solve.  And a gesture can end without a `pointerup` — a cancelled touch, or capture lost
 *  to a system gesture — so those have to finish it too.  A finger is `touch`'s: a second one
 *  is a pinch, which it settles the first one's gesture to start. */
function bindPointer(v: SketchView, touch: TouchNavigation): void {
  const cv = v.canvas;
  cv.addEventListener('pointerdown', (e) => {
    v.finger = e.pointerType === 'touch';
    if (e.pointerType === 'touch') {
      touch.down(e);
      return;
    }
    if (v.gesture) return;
    cv.setPointerCapture(e.pointerId);
    v.gesturePointer = e.pointerId;
    onPointerDown(v, e);
  });
  cv.addEventListener('pointermove', (e) => {
    v.finger = e.pointerType === 'touch';
    if (e.pointerType === 'touch') {
      touch.move(e);
      return;
    }
    if (v.gesture && e.pointerId !== v.gesturePointer) return;
    onPointerMove(v, e);
  });
  const finish = (e: PointerEvent): void => {
    if (e.pointerType === 'touch') {
      touch.up(e);
      return;
    }
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
  // the pointer gone, nothing is under it: what it lit goes out
  cv.addEventListener('pointerleave', () => {
    if (v.gesture || !v.hovered) return;
    v.hovered = null;
    v.draw();
  });
}

/** A double-click: a dimension's number edited, a handle put on a curve, a face drawn on, or a
 *  plane chosen. */
function onDoubleClick(v: SketchView, e: MouseEvent): void {
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
  if (hit instanceof Spline) {
    insertControl(v, hit, ...v.s2wIn(v.viewOf(hit.ctrl[0]), ...sp));
    return;
  }
  // on a solid's face, the face drawn on (#162 F5): the plane lying there chosen, or written —
  // a pane's frame lies beneath a solid, as a press finds it
  const frame = hit instanceof Plane ? hit : null;
  const face = hit && !frame ? null : v.solidAt(...sp);
  if (face) {
    v.sketchOnFace(face);
    return;
  }
  // on a pane's frame, which picks its plane, that plane chosen as a press inside it would be
  const named = frame ? v.doc.nameOf(frame) : null;
  if (named && v.planeChoices().includes(named)) {
    v.choosePlane(named, false);
    return;
  }
  // and on a pane, with nothing drawn under the pointer, it is that plane chosen: the next thing
  // is drawn there.  The eye stays where it is — the pane was double-clicked where it was seen
  const pane = hit && !frame ? null : v.paneAt(...sp);
  if (pane) v.choosePlane(pane, false);
}

/** The wheel.  A pinch on a trackpad comes as a wheel with ctrl held, in small steps; two
 *  fingers slid across one, as a scroll, which slides the workspace as a pan does; a wheel's
 *  notches zoom.  All about the cursor, so what is under it stays under it. */
function onWheel(v: SketchView, e: WheelEvent): void {
  e.preventDefault();
  const [sx, sy] = local(v, e);
  if (e.ctrlKey) {
    const d = Math.max(-PINCH_STEP_MAX, Math.min(PINCH_STEP_MAX, e.deltaY * wheelUnit(e)));
    v.cam.zoomAt(sx, sy, Math.exp(-d * PINCH_PER_PX));
  } else if (scrolled(e)) {
    v.cam.panBy(-e.deltaX, -e.deltaY);
  } else {
    v.cam.zoomAt(sx, sy, 1.0015 ** (-e.deltaY * wheelUnit(e)));
  }
  v.draw();
}

/** Safari tells a trackpad's pinch as a gesture with a running scale and no wheel.  On a touch
 *  screen it tells a pinch so too, beside the fingers' own events, which `touch` reads instead. */
function bindSafariPinch(v: SketchView, touch: TouchNavigation): void {
  const cv = v.canvas;
  let scale = 1;
  const pinched = (e: Event): SafariGesture | null => {
    e.preventDefault();               // or the page zooms
    return touch.fingers() ? null : e as SafariGesture;
  };
  cv.addEventListener('gesturestart', (e) => {
    if (pinched(e)) scale = 1;
  });
  cv.addEventListener('gesturechange', (e) => {
    const g = pinched(e);
    if (!g || !(g.scale > 0)) return;
    v.cam.zoomAt(...local(v, g), g.scale / scale);
    scale = g.scale;
    v.draw();
  });
  cv.addEventListener('gestureend', pinched);
}

/** What `touchNavigation` hands the pointer's events to. */
type TouchNavigation = ReturnType<typeof touchNavigation>;

/** Safari's `GestureEvent`, which no other browser has and so no type library declares. */
type SafariGesture = MouseEvent & { scale: number };

/** How fast a trackpad's pinch zooms, per pixel of its wheel delta, and the most one event may
 *  count: ctrl and a mouse's wheel come as the same event, a notch as a hundred pixels. */
const PINCH_PER_PX = 0.01;
const PINCH_STEP_MAX = 25;

/** A wheel's delta in pixels per unit: Firefox counts a mouse's notches in lines. */
function wheelUnit(e: WheelEvent): number {
  return e.deltaMode === 1 ? 16 : 1;
}

/** Is this wheel event two fingers sliding on a trackpad rather than a mouse's wheel turning?
 *  Nothing says so outright, so it is read off what a wheel does not do: count in pixels and
 *  move sideways — or, where the browser keeps the legacy `wheelDeltaY`, report it as exactly
 *  three times the delta, which a trackpad does and a wheel's notches do not. */
function scrolled(e: WheelEvent): boolean {
  if (e.deltaMode !== 0) return false;
  if (e.deltaX !== 0) return true;
  const legacy = (e as WheelEvent & { wheelDeltaY?: number }).wheelDeltaY;
  return legacy !== undefined && legacy !== 0 && legacy === -3 * e.deltaY;
}

/** How long a lone finger is held before its press is the canvas's, and how far it may wander
 *  meanwhile: a second finger down in that time makes the two a pinch, so the first one's press
 *  neither clicked a tool nor started a drag. */
const TOUCH_HOLD_MS = 120;
const TOUCH_SLOP_PX = 8;

/** Fingers on the glass.  One alone is a pointer like any other — once it has been held or
 *  moved long enough to be sure no second is coming, or lifted as a tap.  Two are a pinch: their
 *  middle carries the workspace with it and their spread zooms it about that middle.  Camera only,
 *  as a pan is; a gesture the first finger started is settled (ended, never dropped: see
 *  `bindEvents`) when the second comes down, and after a pinch a finger left down does nothing
 *  until it is lifted. */
function touchNavigation(v: SketchView) {
  const cv = v.canvas;
  const fingers = new Map<number, [number, number]>();   // every finger down, where it is now
  let held: { e: PointerEvent; at: [number, number]; timer: ReturnType<typeof setTimeout> } | null
    = null;
  let pinch: { mid: [number, number]; span: number } | null = null;

  const spread = (): { mid: [number, number]; span: number } => {
    const [a, b] = [...fingers.values()];
    return {
      mid: [(a[0] + b[0]) / 2, (a[1] + b[1]) / 2],
      span: Math.hypot(b[0] - a[0], b[1] - a[1]),
    };
  };
  /** The lone finger is sure to be one: its press goes to the canvas as a mouse's would. */
  const press = (): void => {
    if (!held) return;
    const { e, timer } = held;
    held = null;
    clearTimeout(timer);
    if (v.gesture) return;
    v.gesturePointer = e.pointerId;
    onPointerDown(v, e);
  };
  return {
    fingers: (): number => fingers.size,
    down(e: PointerEvent): void {
      const at = local(v, e);
      fingers.set(e.pointerId, at);
      cv.setPointerCapture(e.pointerId);
      if (pinch) {
        pinch = spread();               // a finger joined: carry on from where they all are
        return;
      }
      if (fingers.size === 1) {
        if (!v.gesture) held = { e, at, timer: setTimeout(press, TOUCH_HOLD_MS) };
        return;
      }
      // a second finger: a pinch, unless a mouse owns the live gesture
      const owner = v.gesturePointer;
      if (v.gesture && (owner === null || !fingers.has(owner))) return;
      if (held) {
        clearTimeout(held.timer);
        held = null;
      }
      if (owner !== null && fingers.has(owner)) {
        v.gesturePointer = null;
        endGesture(v);
      }
      pinch = spread();
    },
    move(e: PointerEvent): void {
      if (!fingers.has(e.pointerId)) return;
      const at = local(v, e);
      fingers.set(e.pointerId, at);
      if (pinch) {
        const now = spread();
        v.cam.panBy(now.mid[0] - pinch.mid[0], now.mid[1] - pinch.mid[1]);
        if (pinch.span > 0 && now.span > 0) v.cam.zoomAt(...now.mid, now.span / pinch.span);
        pinch = now;
        v.draw();
        return;
      }
      if (held?.e.pointerId === e.pointerId) {
        if (Math.hypot(at[0] - held.at[0], at[1] - held.at[1]) < TOUCH_SLOP_PX) return;
        press();
      }
      if (e.pointerId === v.gesturePointer) onPointerMove(v, e);
    },
    up(e: PointerEvent): void {
      if (!fingers.delete(e.pointerId)) return;
      if (held?.e.pointerId === e.pointerId) press();   // a tap: the press, then its release
      if (pinch) {
        pinch = fingers.size >= 2 ? spread() : null;
        return;
      }
      if (e.pointerId !== v.gesturePointer) return;
      v.gesturePointer = null;
      onPointerUp(v);
    },
  };
}


export function onPointerDown(v: SketchView, e: PointerEvent): void {
  v.stopAnimation();
  const sp = local(v, e);
  v.cursor = sp;
  if (e.button !== 0) {
    cameraPress(v, e, sp);
    return;
  }
  if (primaryTaken(v, e, sp)) return;
  selectPress(v, e, sp);
}

/** The camera's buttons, as a CAD tool has them: the right turns the workspace about, the middle
 *  (or the right with shift) slides it; with a tool's points half collected the right backs
 *  out.  Any other button does nothing. */
function cameraPress(v: SketchView, e: PointerEvent, sp: [number, number]): void {
  if (e.button === 2 && v.pending.length) {
    cancelTool(v);
    return;
  }
  if (e.button === 1 || (e.button === 2 && e.shiftKey)) {
    v.gesture = panGesture(v, sp);
    return;
  }
  if (e.button === 2) {
    v.gesture = orbitGesture(v, sp);
  }
}

/** A primary press taken before the select tool sees it: a dimension planted, an orbit, an
 *  extent's handle grabbed, or a tool's click.  Whether it was taken. */
function primaryTaken(v: SketchView, e: PointerEvent, sp: [number, number]): boolean {
  // a dimension still following the pointer: this click is what plants it.  The default is
  // refused so the focus stays in its editor — the number is still being typed
  if (v.liveDim?.placing) {
    e.preventDefault();
    placeDimension(v);
    return true;
  }
  // in orbit mode the primary button turns the workspace, whatever is under it
  if (v.orbitMode) {
    v.gesture = orbitGesture(v, sp);
    return true;
  }
  // the handle of an extrusion, a revolution or an offset plane being sized: taking hold of it
  // sizes it, and a press anywhere else is done with an offset plane's
  if (onExtentArrow(v, sp)) {
    v.gesture = extentGesture(v);
    return true;
  }
  if (v.liveExtent?.kind === 'offset') v.endExtent();
  // ⌘-click (Ctrl off a Mac): everything under the pointer, to choose among
  if ((e.metaKey || e.ctrlKey) && picksAs(v.tool)) {
    e.preventDefault();
    chooseAt(v, sp, e.shiftKey);
    return true;
  }
  if (v.tool !== 'select') {
    // an edge tool's number keeps the keyboard across the clicks that pick more
    if (roundsEdges(v.tool)) e.preventDefault();
    toolClick(v, sp, { alt: e.altKey, shift: e.shiftKey });
    return true;
  }
  return false;
}

/** A primary press with the select tool: what is under it picked, and a gesture started on it. */
function selectPress(v: SketchView, e: PointerEvent, sp: [number, number]): void {
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
  if (pressedApart(v, e, sp, at)) return;
  v.dropImage();
  const ent = at.kind === 'entity' ? at.ent : null;
  if (!ent) {
    // nothing under the cursor: start a rubber band.  A press with no drag still just
    // clears the selection, because an empty box selects nothing.
    if (!e.shiftKey) v.selected = [];
    v.gesture = bandGesture(v, sp);
  } else if (e.shiftKey) {
    // a shift-click toggles it in the selection; a shift-drag turns the body
    v.gesture = toggleOrTurn(v, sp, ent);
  } else if (ent instanceof Plane && !canMove(v, ent)) {
    // a plane held where it stands is picked by a click on its pane's frame — which runs just
    // outside what is drawn on it, so a press there that moves is the band it would have been
    // on bare canvas, taking what it touches in place of the plane
    v.selected = [ent];
    v.gesture = bandGesture(v, sp, []);
  } else {
    if (!v.selected.includes(ent)) v.selected = [ent];
    dragEntity(v, sp, ent);
  }
  v.onSelect();
  v.onChanged();
  v.draw();
}

/** A press on what is not the drawing's: a corner of the traced picture, a dimension, the
 *  picture itself or a solid.  Whether it was one. */
function pressedApart(v: SketchView, e: PointerEvent, sp: [number, number], at: Target): boolean {
  // a corner of the traced picture: it sizes and turns the picture about its centre
  if (at.kind === 'handle') {
    v.gesture = grabHandle(v, at.corner);
    return true;
  }
  // a dimension: the press selects the constraint, and dragging moves the callout rather
  // than the sketch
  if (at.kind === 'callout') {
    v.selected = [];
    v.onPickConstraint(at.callout);
    v.gesture = calloutGesture(v, at.callout, sp);
    v.draw();
    return true;
  }
  if (at.kind === 'image') {
    v.pickImage();
    v.gesture = grabBody(v, sp);
    v.onSelect();
    v.onChanged();          // as every other press does: the window and the readout follow it
    v.draw();
    return true;
  }
  if (at.kind === 'solid') {
    v.pickSolid(at.pick, e.shiftKey);
    v.onSelect();
    v.onChanged();
    v.draw();
    return true;
  }
  return false;
}

/** **A ⌘-click**: everything the eye's ray reaches under the pointer that the tool down could take
 *  (`SketchView.hitsAt`) — the one there is taken as a click on it, several are handed to the
 *  shell to list (`onChoose`), and where there is nothing, it says so. */
export function chooseAt(v: SketchView, sp: [number, number], add: boolean): void {
  const hits = v.hitsAt(sp[0], sp[1]);
  if (hits.length === 1) takeHit(v, hits[0], add);
  else if (hits.length) v.onChoose(hits, sp, add);
  else v.onStatus('nothing is under the pointer here to choose');
}

/** **Take a hit as a click on it would**, with the tool down: selected (with `add`, toggled in the
 *  selection), an edge tool's pick, a Boolean's body.  A row chosen from a ⌘-click's list is taken
 *  so; nothing is dragged. */
export function takeHit(v: SketchView, hit: Hit, add: boolean): void {
  v.hovered = null;
  const takes = picksAs(v.tool);
  if (hit.kind === 'solid' && takes === 'parts') {
    roundPick(v, hit.pick);
    return;
  }
  if (hit.kind === 'solid' && takes === 'bodies') {
    combinePick(v, bodyOf(hit.pick));
    return;
  }
  if (takes !== 'all') return;
  // assigned, never grown in place: the setter is where the other selections are let go
  if (hit.kind === 'solid') v.pickSolid(hit.pick, add);
  else v.selected = toggled(v.selected, hit.ent, add);
  v.onSelect();
  v.onChanged();
  v.draw();
}

/** The drag a press on an entity starts, where it can move at all. */
function dragEntity(v: SketchView, sp: [number, number], ent: Primitive): void {
  // a point of a drawing that runs over a parameter is dragged **along its march** (#398 L): the
  // drawing driven by where its point is put, as a crank is turned by its pin
  if (ent instanceof Point && v.marches.some((m) => m.followed)) {
    const space = v.inSpace(ent);
    const view = v.viewOf(ent);
    if (space || readable(v, view)) {
      const read = (at: [number, number]) => space ? v.eye(...at) : v.placeIn(view, ...at);
      const taken = takeScrub(v, ent, space);
      if (taken instanceof Scrub) {
        v.gesture = scrubGesture(v, taken, read);
        return;
      }
      // shown at an instant, nothing else of the drawing moves: what is drawn there is not
      // where it is drawn
      if (v.instant?.at != null) {
        v.onStatus(`${v.doc.nameOf(ent) ?? 'that point'}: ${taken}`);
        return;
      }
    }
  }
  // a point in space stands in no view, so it is dragged **where the eye sees it**: across the
  // picture plane, its depth along the line of sight left to what else holds it
  if (ent instanceof Point && v.inSpace(ent)) {
    if (canMove(v, ent)) {
      v.pushUndo();
      const drag = new PlanDrag(v.sketch, ent, ...v.eye(...sp), null, 0.05, v.plan(), v.orbit);
      v.gesture = pointGesture(v, drag, (at) => v.eye(...at));
    }
  // a drag moves a thing **in its own plane**: the pointer is read off the view it stands in,
  // so a point on the side plane slides along the side plane however the eye is turned
  } else if (ent instanceof Point && canMove(v, ent) && readable(v, v.viewOf(ent))) {
    v.pushUndo();
    const view = v.viewOf(ent);
    // on the sketch's own plan, compiled once per topology: the drag starts at once
    const drag = new PlanDrag(v.sketch, ent, ...v.placeIn(view, ...sp), null, 0.05, v.plan());
    v.gesture = pointGesture(v, drag, (at) => v.placeIn(view, ...at));
  // a plane is dragged by its pane as a point in space is, where the eye sees it: the core
  // takes hold of its origin and moves where the plane stands, carrying it without turning it.
  // The pane is grabbed where it was pressed, so it follows the pointer from there
  } else if (ent instanceof Plane && canMove(v, ent)) {
    v.pushUndo();
    const [px, py] = v.eye(...sp);
    const [ox, oy] = v.eye(...v.seen(ent.origin));
    const drag = new PlanDrag(v.sketch, ent.origin, ox, oy, null, 0.05, v.plan(), v.orbit);
    v.gesture = pointGesture(v, drag, (at) => {
      const [x, y] = v.eye(...at);
      return [x + ox - px, y + oy - py];
    });
  } else if (isResizable(v, ent) && readable(v, v.viewOf(ent.center))) {
    v.pushUndo();
    v.gesture = radiusGesture(v, new RadiusDrag(v.sketch, ent, Math.abs(scalarOf(ent).value)),
                              v.viewOf(ent.center));
  }
}
export function onPointerMove(v: SketchView, e: PointerEvent): void {
  const sp = local(v, e);
  v.cursor = sp;
  if (v.liveDim?.placing) moveDimension(v, sp);
  else if (v.gesture) v.gesture.move(sp, { shift: e.shiftKey });
  else {
    hover(v, sp);
    if (v.orbitMode) return;   // nothing a hover paints is offered in orbit mode
  }
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

/** How far the eye swings per screen pixel dragged, and how near the pole an elevation may come:
 *  at exactly ±90° the bearing means nothing and the workspace would spin about nothing. */
const ORBIT_PER_PX = 0.008;
const EL_MAX = Math.PI / 2 - 1e-3;

/** Turn the workspace about: a horizontal drag swings the eye round it, a vertical one lowers
 *  it — the pointer pushes the scene about as if it were held.  Camera-only — the two angles
 *  are view state and nothing in the document moves — so it is `transient` exactly as a pan is. */
export function orbitGesture(v: SketchView, from: [number, number]): Gesture {
  let last = from;
  return {
    transient: true,
    move: (sp) => {
      const o = v.orbit;
      v.orbit = {
        az: o.az + (sp[0] - last[0]) * ORBIT_PER_PX,
        // dragging *down* lowers the eye: the pointer pushes the scene about as if it were held,
        // rather than steering a camera pointed at it
        el: Math.min(EL_MAX, Math.max(-EL_MAX, o.el - (sp[1] - last[1]) * ORBIT_PER_PX)),
      };
      last = sp;
    },
  };
}

/** How far the pointer goes before a shift-press on a figure is a turn rather than a click. */
const TURN_SLOP_PX = 3;

/** **A shift-press on a figure**: let go where it was pressed, it is a click, toggling the figure
 *  in the selection; moved, it **turns the body** — the selection, where the figure is in it,
 *  else the figure alone, taken as the selection.  The core says where it turns (`TurnDrag`): in
 *  the plane it is drawn in, about its middle as the pointer sweeps round it; or, with a plane,
 *  a frame, an axis or a point in space in it, in space, about its middle (a lone plane's own
 *  origin) as a ball held there is pushed round — the way an unconstrained plane is stood where
 *  it is wanted.  On the frame of a plane nothing of which can turn, a press that moves is the
 *  band it would have been on bare canvas, the plane toggled in its base. */
function toggleOrTurn(v: SketchView, from: [number, number], ent: Primitive): Gesture {
  let g: Gesture | null = null;
  return {
    move: (sp) => {
      if (!g) {
        if (Math.hypot(sp[0] - from[0], sp[1] - from[1]) < TURN_SLOP_PX) return;
        g = taken(v, from, ent);
      }
      g.move(sp);
    },
    end: () => {
      if (g) {
        g.end?.();
        return;
      }
      // assigned, never grown in place: the setter is where the other selections are let go
      v.selected = toggled(v.selected, ent, true);
      v.onSelect();
    },
    abandon: () => g?.abandon?.(),
    get movedGeometry() { return !!g?.movedGeometry; },
  };
}

/** What a shift-press on `ent` that moved takes up: the turn, else a held plane's band, else
 *  nothing, said why. */
function taken(v: SketchView, from: [number, number], ent: Primitive): Gesture {
  const picked = v.selected.includes(ent);
  const turn = turnGesture(v, picked ? [ent, ...v.selected.filter((e) => e !== ent)] : [ent], from);
  if (turn) {
    if (!picked) {
      v.selected = [ent];
      v.onSelect();
    }
    return turn;
  }
  if (ent instanceof Plane) {
    const base = v.selected;
    v.selected = toggled(base, ent, true);
    v.onSelect();
    return bandGesture(v, from, base);
  }
  v.onStatus('nothing there is free to turn');
  return { transient: true, move: () => {} };
}

/** Turn `body` as the pointer moves from `from`, or `null` where nothing of it is free to turn
 *  or the plane it turns in is seen edge on (said so). */
function turnGesture(v: SketchView, body: Primitive[], from: [number, number]): Gesture | null {
  const drag = new TurnDrag(v.sketch, body);
  if (!drag.carries || (!drag.inSpace && !readable(v, drag.plane))) {
    drag.end();
    return null;
  }
  v.pushUndo();
  let last = from;
  return {
    movedGeometry: true,
    move: (sp) => {
      // in space pushed round as the workspace is orbited, the pointer's pixels turning it alike;
      // in its plane, by the angle the pointer swept round its middle there
      v.lastResult = drag.inSpace
        ? drag.turnSeen(v.orbit, (sp[0] - last[0]) * ORBIT_PER_PX, (sp[1] - last[1]) * ORBIT_PER_PX)
        : drag.sweep(v.s2wIn(drag.plane, ...last), v.s2wIn(drag.plane, ...sp));
      last = sp;
      v.onDragFrame();
    },
    end: () => drag.end(),
    abandon: () => drag.end(),
  };
}

/** Can a place on this view be read off the pointer?  Not when it is seen edge on, where a pixel
 *  spans the whole plane; the status line says so, since a drag that does nothing is a mystery. */
function readable(v: SketchView, view: View): boolean {
  if (v.camOf(view)?.readable()) return true;
  v.onStatus('that plane is seen edge on — turn the view to move things on it');
  return false;
}

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

/** Drag a point, `read` turning a canvas point into what the drag is handed: a place on the
 *  point's view, or on the eye's picture plane for a point in space. */
export function pointGesture(v: SketchView, drag: PlanDrag,
                             read: (sp: [number, number]) => [number, number]): Gesture {
  let reported = 0;
  return {
    movedGeometry: true,
    move: (sp) => {
      v.lastResult = drag.move(...read(sp));
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

/** The scrub of `p` along the first followed parameter it moves under — the one shown first —
 *  or why there is none. */
function takeScrub(v: SketchView, p: Point, space: boolean): Scrub | string {
  const shown = v.instant?.name;
  const names = v.marches.filter((m) => m.followed).map((m) => m.name)
    .sort((a, b) => Number(b === shown) - Number(a === shown));
  let why = '';
  for (const name of names) {
    const s = Scrub.take(v.sketch, name, p, space ? v.orbit : null);
    if (s instanceof Scrub) return s;
    why ||= s.refused;
  }
  return why;
}

/** Drag a point along the march (#398 L): each move stands the drawing at the instant its point
 *  comes nearest the pointer.  What is shown moves and the document does not, so it is
 *  `transient` — nothing is written back, and nothing goes on the undo stack. */
export function scrubGesture(v: SketchView, scrub: Scrub,
                             read: (sp: [number, number]) => [number, number]): Gesture {
  return {
    transient: true,
    move: (sp) => {
      const r = scrub.move(...read(sp));
      if ('at' in r) v.scrubbedTo(scrub.name, r.at);
      else v.onStatus(r.refused);
      v.onDragFrame();
    },
    end: () => scrub.end(),
    abandon: () => scrub.end(),
  };
}

export function radiusGesture(v: SketchView, drag: RadiusDrag, view: View): Gesture {
  return {
    movedGeometry: true,
    abandon: () => drag.end(),
    move: (sp) => {
      const [wx, wy] = v.s2wIn(view, ...sp);
      const e = drag.circle;
      // a circle's rim follows the cursor at its distance from the centre, to the digits the
      // zoom justifies — and a rim within half a pixel of it keeps what it measured, not zero
      const w = Math.hypot(wx - e.center.x.value, wy - e.center.y.value);
      const r = v.placeLength(w, view) || w;
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
  // a callout is laid out on the page of the view its dimension is in, and moved there
  const view = v.calloutView(c);
  const grip = dim.grab(v.sketch, v.unit, c.id, ...v.s2wIn(view, ...from));
  let moved = false;
  return {
    transient: true,               // the annotation moved, not the geometry
    move: (sp) => {
      if (!grip) return;
      if (!moved) {                // the first movement is the edit; a bare click is not one
        moved = true;
        v.pushUndo();
      }
      dim.drag(v.sketch, c.id, ...v.s2wIn(view, ...sp), grip);
    },
    // where a callout sits is document state, so the drag is a source edit and has to be
    // written down — once, at the release, the same bargain `syncSeeds` strikes for a point
    end: () => { if (moved) v.syncSource(); },
  };
}

export function bandGesture(v: SketchView, from: [number, number],
                            base: Primitive[] = v.selected): Gesture {
  base = [...base];
  let to = from;                     // the gesture owns both corners, so paint reads no globals
  return {
    move: (sp) => {
      to = sp;
      // live preview: the canvas shows what would be selected, the status line the count
      v.selected = [...new Set([...base, ...boxContents(v, from, sp)])];
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

/** Entities the box touches — "crossing" selection, a point inside it or a stroke through it —
 *  asked of the core where the eye sees each figure, so a band drawn over one plane takes nothing
 *  standing on another. */
export function boxContents(v: SketchView, from: [number, number], to: [number, number]): Primitive[] {
  const { az, el } = v.orbit;
  return overlappingSeen(v.sketch, v.unit, az, el, v.cam.s2w(...from), v.cam.s2w(...to))
    .filter((e) => !(e instanceof Plane));
}

/** What is under the pointer, in the order a press offers itself to things.
 *
 *  The order is the whole content of this function and it is stated **once**: a corner handle,
 *  then a dimension callout, then the geometry, then an object's face, then the traced picture,
 *  then a pane's frame, then nothing.  Handles and callouts are painted over everything and so
 *  outrank it; the **drawing outranks the picture**, so a line lying across a photograph is what a
 *  click on that line picks, and the picture answers only where the geometry did not.  A pane's
 *  frame runs just outside what is drawn on its plane, so it answers only where nothing else
 *  does.
 *
 *  Both readers — the press and the cursor it promises — ask this rather than each walking the
 *  list, because two copies of the order are two orders the moment one of them is edited, and
 *  the failure is a cursor that offers what a click does not do. */
type Target =
  | { kind: 'handle'; corner: number }
  | { kind: 'callout'; callout: Constraint }
  | { kind: 'entity'; ent: Primitive }
  | { kind: 'solid'; pick: SolidPick }
  | { kind: 'image' }
  | { kind: 'none' };

function whatIsAt(v: SketchView, sp: [number, number]): Target {
  const corner = handleAt(v, sp);
  if (corner >= 0) return { kind: 'handle', corner };
  const callout = v.pickCallout(sp[0], sp[1]);
  if (callout) return { kind: 'callout', callout };
  const ent = v.pick(sp[0], sp[1]);
  if (ent && !(ent instanceof Plane)) return { kind: 'entity', ent };
  // beneath the drawing: an object's edge the eye sees within reach, else its face, where the eye
  // sees it — the smaller target first, as a point outranks a line
  const pick = v.edgeAt(sp[0], sp[1]) ?? v.solidAt(sp[0], sp[1]);
  if (pick) return { kind: 'solid', pick };
  if (bodyAt(v, sp)) return { kind: 'image' };
  // and beneath everything, a pane's frame: a plane
  if (ent) return { kind: 'entity', ent };
  return { kind: 'none' };
}

/** Whether a canvas point is on the tip of the handle of what is being sized. */
function onExtentArrow(v: SketchView, sp: [number, number]): boolean {
  const tip = v.extentArrow()?.tip;
  return !!tip && Math.hypot(tip[0] - sp[0], tip[1] - sp[1]) < v.pickPx + 4;
}

/** Dragging an extrusion's or an offset plane's arrow, or a revolution's handle: it is sized where
 *  the eye sees the pointer along its normal (an extrusion both ways with ⇧) or round its turn,
 *  and written when it is let go — one edit, joining the undo step that made it. */
function extentGesture(v: SketchView): Gesture {
  return {
    transient: true,
    move: (at, mods) => v.sizeExtent(at[0], at[1], mods?.shift),
    end: () => {
      const why = v.commitExtent();
      if (why) v.onStatus(why);
    },
  };
}

/** **What a press here would take, streamed as the pointer moves**: lit where it is drawn
 *  (`SketchView.hovered`) — the drawing's figure or point, an object's edge, face or body — and
 *  the cursor's affordance for what a press would grab. */
export function hover(v: SketchView, sp: [number, number]): void {
  v.hovered = null;
  if (v.orbitMode) {
    v.canvas.style.cursor = 'grab';
    return;
  }
  // the Extrude and Revolve tools show what a click would sweep, and offer its handle; with a
  // region chosen, Revolve lights the line or axis a click would turn it about
  if (sweepsRegion(v.tool)) {
    const grab = onExtentArrow(v, sp);
    if (v.pendingRevolve) {
      const e = grab ? null : v.pick(sp[0], sp[1]);
      v.highlight = e && v.turnsAbout(e) ? [e] : [];
    } else {
      v.hoverRegion = grab ? null : v.regionAt(sp[0], sp[1]);
    }
    v.canvas.style.cursor = grab ? 'grab' : '';
    return;
  }
  // an edge tool lights the edge or face a click would pick, a Boolean the body
  const takes = picksAs(v.tool);
  if (takes === 'parts' || takes === 'bodies') {
    const pick = takes === 'parts' ? roundTarget(v, sp[0], sp[1]) : v.solidAt(sp[0], sp[1]);
    v.hovered = pick && { kind: 'solid', pick: takes === 'bodies' ? bodyOf(pick) : pick };
    v.canvas.style.cursor = pick ? 'pointer' : '';
    return;
  }
  if (v.tool !== 'select') return;
  if (onExtentArrow(v, sp)) {
    v.canvas.style.cursor = 'grab';
    return;
  }
  const at = whatIsAt(v, sp);
  v.hovered = at.kind === 'entity' ? { kind: 'entity', ent: at.ent }
    : at.kind === 'solid' ? { kind: 'solid', pick: at.pick } : null;
  v.canvas.style.cursor =
      at.kind === 'handle' ? (at.corner % 2 ? 'nesw-resize' : 'nwse-resize')
    : at.kind === 'callout' || at.kind === 'image' ? 'move'
    : at.kind === 'entity' && (at.ent instanceof Point || at.ent instanceof Plane)
      && canMove(v, at.ent) ? 'grab'
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

/** The one scalar a rim drag moves: a circle's or arc's radius. */
export function scalarOf(e: Circle | Arc): Param {
  return e.radius;
}

/** A circle or arc whose radius is free to follow the cursor. */
export function isResizable(v: SketchView, e: Primitive | null): e is Circle | Arc {
  return (e instanceof Circle || e instanceof Arc)
    && !scalarOf(e).fixed && (!v.diagnosis || v.staleDiagnosis || canMove(v, e));
}
