/* The sketch view: the object the whole front end holds, and the state every part of it reads
 * — the camera, the selection, the tool, the compiled plan and the last diagnosis.  What is
 * *done* to it lives in the modules beside this one: `paint` strokes it, `gesture` drives it
 * from the pointer, `tools` draws into it, `dimension` writes a number on it and `edit` changes
 * the document.  They take the view as their first argument, so this file stays the state and
 * the seams — the history, the solve, and the two one-line questions everything else asks:
 * where a world place is on the canvas (`camera`) and what is under the cursor (the core).
 * Neither answer is worked out here: the camera is the front end's only linear algebra and the
 * geometry is all the core's, which is what keeps the two apart.
 *
 * **The view is the workspace**: one scene in space, every sketch standing on its own plane and
 * solids drawn under it by three.js (`box3d.ts`).  Each plane's page reaches the screen through
 * one affine map (`core/workspace.ts`, composed by `camera.ts`), so the painter, the tools and the
 * gestures read and write page coordinates as they always did — through the camera of the view
 * they are working in — and what is under the pointer is asked of the core where the eye sees it.
 *
 * Every mutation funnels through `afterEdit`, which re-solves (when auto-solve is on),
 * re-diagnoses and notifies the shell exactly once. */
import { Box3D } from './box3d.js';
import { objects } from '../core/mesh.js';
import { FieldPreview, type Refining } from './field-preview.js';
import * as io from '../core/io.js';
import * as dim from '../core/callout.js';
import { Constraint } from '../core/constraints.js';
import { PlanResult, PlanSolver, asSolveResult } from '../core/decompose.js';
import { Diagnosis, diagnose } from '../core/diagnose.js';
import { Param, Plane, Point, Primitive, Sketch } from '../core/model.js';
import { Document, Edit, fromSketch } from '../core/program.js';
import { Method, SolveResult, System } from '../core/system.js';
import type { Item } from '../core/overview.js';
import { Motion, WitnessReport, analyze } from '../core/witness.js';
import { Camera, ViewCam } from './camera.js';
import * as edit from './edit.js';
import * as dimension from './dimension.js';
import { abandonGesture, bindEvents } from './gesture.js';
import type { Gesture } from './gesture.js';
import type { DimAlt, LiveDim } from './dimension.js';
import { paint } from './paint.js';
import * as tools from './tools.js';
import * as underlay from './underlay.js';
import type { Bitmap, Underlay } from './underlay.js';
import {
  NOWHERE, PAGE, boundsSeen, calloutSeen, lookOf, maps, nearestSeen, ofView, panesSeen, pickSeen,
  pickSolidSeen, placeOf, spacePoints, workspace,
} from '../core/workspace.js';
import type { View, Workspace } from '../core/workspace.js';

/** A solid picked in the workspace: which (its name, and its index in the current sketch) and the
 *  face the click landed on, named where it was made. */
export interface SolidPick {
  name: string;
  index: number;
  face: string;
}

/* A dimension being written belongs to `dimension`, but it is the view a caller holds, so the
 * two types are published from here as well. */
export type { DimAlt, LiveDim } from './dimension.js';

export const PICK_PX = 8;
/** The same tolerance for a finger: a fingertip covers some 7–10 mm of glass, and the thing it
 *  means is anywhere under it — so a tap reaches as far as half the 44-pixel target a touch
 *  screen's guidelines ask of a control.  Nearest still wins within it. */
export const TOUCH_PICK_PX = 22;

/** The planes every workspace offers, whether or not the document has them yet — a CAD part's
 *  origin planes.  `std.front` is the page itself seen in space, so geometry drawn on it with no
 *  `use std` stays on the page; the other two bring the library in the first time they are drawn
 *  on (`ensurePlane`). */
export const STANDARD_PLANES = ['std.front', 'std.side', 'std.top'] as const;

/** A document of nothing but `use std`, read once: where the standard planes are for a document
 *  that does not have them yet, so the eye can be turned to one before it is drawn on. */
let stdDoc: Document | null = null;
function standardDocument(): Document {
  return stdDoc ??= Document.read('use std\n');
}

/** The eye a flat drawing opens with — the front, square on — and the three-quarter view a
 *  document with solids or geometry off the front plane opens with. */
const FRONT = { az: -Math.PI / 2, el: 0 };
const THREE_QUARTER = { az: -Math.PI / 3, el: Math.PI / 6 };
/** How long the eye takes to swing square on to a chosen plane. */
const LOOK_MS = 260;

const ANIM_DT = 0.03;        // seconds per animation tick
const ANIM_PERIOD = 2.0;     // seconds spent on each degree of freedom

interface Animation {
  modes: Motion[];
  /** The sketch it started on: the animation borrows that sketch's values and puts them back,
   *  and neither the tick nor the restore may touch a sketch that replaced it. */
  sketch: Sketch;
  x0: Float64Array;
  free: Int32Array;
  amp: number;
  labels: string[];
  t: number;
  showing: number;
}

/** A place a click asked for, and the Point it came from if it came from one. */
export interface Place {
  at: [number, number];
  on: Point | null;
}

export type Tool =
  'select' | 'point' | 'line' | 'rect' | 'circle' | 'arc' | 'arc3' | 'spline' | 'splinefit'
  | 'plane';

/** What the plane tool is armed with: the name the statement is to be given, if any.  Which way
 *  the plane faces is the two lines its clicks pick. */
export interface PlaneSpec {
  name?: string;
}


export class SketchView {
  /** **The document.**  The source somebody wrote, the drawing it elaborates to, and where each
   *  part of the drawing was written.  Every edit is an edit of this text; the drawing is what
   *  the text came to, and is replaced whole whenever the text changes structurally. */
  doc: Document;
  /** Where the drawing sits on the canvas — and the whole of the front end's linear algebra:
   *  every world/screen conversion in `app/` goes through it (see `camera.ts`). */
  readonly cam = new Camera();
  tool: Tool = 'select';
  method: Method = 'dogleg';
  autoSolve = true;
  usePlan = false;
  colorByState = true;
  /** Paint the dimensioned constraints on the drawing as callouts.  View state, like the
   *  colouring: a document change leaves it as it was.  The model canvas's callouts are the
   *  editor's, and separate from what a `.svd` asks to be annotated on paper. */
  showDimensions = true;
  /** **Where the eye stands**, in radians: a bearing about the vertical and an elevation above
   *  the horizon.  The whole of how the workspace is looked at — the three.js camera is set from
   *  it (`box3d.ts`) and the core projects each plane for it (`core/workspace.ts`).  `az = -π/2,
   *  el = 0` is the front seen square on, where a flat drawing opens.  View state: never saved,
   *  exported, solved or undone, and a document change leaves it where it was. */
  orbit = { ...FRONT };
  /** A picture to trace over, in world coordinates — see `underlay.ts`.  Handled like anything
   *  else on the canvas (clicked, dragged, deleted) but **not document state**: it is scenery,
   *  and only its frame answers a press until it is selected, which is what lets the drawing be
   *  made straight through it. */
  underlay: Underlay | null = null;

  /** What is selected.  Written through a setter on purpose: selecting geometry is the other
   *  half of the picture's exclusivity, and there are eleven sites that assign this.  Enforced
   *  here, paste, a rubber band and a press on an entity all inherit the rule; enforced at each
   *  of them, it is a rule stated nowhere they can see and the failure is silent — a Delete
   *  that takes the photograph instead of what was just pasted. */
  get selected(): Primitive[] { return this._selected; }
  set selected(prims: Primitive[]) {
    this._selected = prims;
    // assigning the drawing's selection, even to nothing, lets the solids go: a press on a
    // callout or on the constraint list selects that instead
    this._solids = [];
    if (prims.length) this.dropImage();
    // picking a view is choosing where to draw: a plane selected on its own becomes the current
    // one, and stays so past the selection — the point drawn next is in it
    if (prims.length === 1 && prims[0] instanceof Plane) this.plane = prims[0];
  }
  private _selected: Primitive[] = [];
  /** The solids selected, each by the face the click landed on — the third selection, exclusive
   *  with the other two for the picture's reason: a solid is not a `Primitive`, nothing
   *  constrains one beside a point, and holding both would leave Delete ambiguous. */
  get selectedSolids(): readonly SolidPick[] { return this._solids; }
  private _solids: SolidPick[] = [];
  /** **The current plane**: the view every point a tool mints is drawn in, or null for the
   *  page.  A proxy of the current sketch, carried across a re-elaboration by name like the
   *  selection, and let go when the name no longer resolves — a deleted plane, or a load. */
  plane: Plane | null = null;
  /** A standard plane chosen to draw on that the document does not have yet — it says no
   *  `use std` — named until the first press of a tool brings it in (`ensurePlane`). */
  pendingPlane: string | null = null;
  /** What the plane tool will write when its two clicks land. */
  planeSpec: PlaneSpec | null = null;
  /** The line the plane tool's first click picked, by name: the one the plane runs along. */
  planeAxis: string | null = null;
  highlight: Primitive[] = [];
  pending: Point[] = [];
  /** Where the fit tool has been told the curve must pass, before there is a curve.  Places
   *  rather than Points, so the tool leaves nothing in the sketch if it is abandoned; one that
   *  came from a Point becomes a constraint once there is a curve to name. */
  pendingFit: Place[] = [];
  diagnosis: Diagnosis | null = null;
  /** The constraint the shell has the focus on, so its callout can say so. */
  litConstraint: Constraint | null = null;
  lastResult: SolveResult | null = null;
  lastPlan: PlanResult | null = null;

  onChanged: () => void = () => {};
  /** The active tool changed (including when Escape backs out of one). */
  onTool: (tool: Tool) => void = () => {};
  /** A pointer interaction changed the canvas selection — so the canvas now has the focus,
   *  and whatever else was focused (a constraint row) no longer does. */
  onSelect: () => void = () => {};
  /** The source changed — a structural edit, a seed writeback, an undo.  What the panel is
   *  wired to, and **never** `onDragFrame`: a drag writes the source once, when it is let go. */
  onProgram: () => void = () => {};
  /** A different document was opened, so source navigation starts at its main file. */
  onLoad: () => void = () => {};
  /** Per gesture frame: the sketch's structure and the constraint list are unchanged, so
   *  only the status line needs updating (a drag's numbers, a band's selection count). */
  onDragFrame: () => void = () => {};
  onStatus: (msg: string) => void = () => {};
  /** The swept objects refining in the background, as each frame of their meshing arrives. */
  onRefine: (refining: Refining[]) => void = () => {};
  /** A dimension callout was clicked: the shell puts the focus on that constraint. */
  onPickConstraint: (c: Constraint) => void = () => {};
  /** What is held changed, and nothing else did — the line that names it is the whole of the
   *  consequence.  Narrower than `onChanged`, which re-reads every constraint and every
   *  expression out of the core and re-renders the program overlay: fading the traced picture
   *  is a keypress that auto-repeats, and it moves one number in one string. */
  onPicked: () => void = () => {};
  /** And double-clicked: the shell opens its value for editing. */
  onEditConstraint: (c: Constraint) => void = () => {};
  /** A dimension is being written, and this is where its number is on screen — the shell puts
   *  an editor there.  Called again whenever the callout moves or turns into another kind, and
   *  with nulls when there is no longer one being written. */
  onDimension: (live: LiveDim | null, at: [number, number] | null) => void = () => {};

  /** The dimension being written, if any — see `startDimension`. */
  liveDim: LiveDim | null = null;

  /* -- the view's own workings.  What is not `private` here is read by the modules beside
   * this file — they are as much the view as this class is, and TypeScript has no way to say
   * "private to these five files".  Nothing outside `app/` touches them. */

  ctx: CanvasRenderingContext2D;
  /** Where the pointer last was, in canvas coordinates. */
  cursor: [number, number] = [0, 0];
  /** Both histories hold **program text**: `undoStack` the states an edit moved away from,
   *  `redoStack` the ones an undo moved away from.  Text, so undo restores what somebody wrote
   *  — and so a caller that snapshots before it knows whether the edit will happen must
   *  snapshot `source`, never a serialised sketch: `Document.read` does not throw on a program
   *  error, so the wrong kind of string comes back as an empty drawing rather than a refusal. */
  private undoStack: string[] = [];
  private redoStack: string[] = [];
  private planSolver: PlanSolver | null = null;
  private planKey = '';
  /** The topology `lastSystem` was compiled from, so a stale one is not diagnosed against. */
  private systemKey = '';
  private lastSystem: System | null = null;
  private witness: WitnessReport | null = null;
  private witnessFor: Diagnosis | null = null;
  anim: Animation | null = null;
  private animTimer = 0;
  /** The last copy, as a sketch of its own.  A clipboard is a document, which is what lets one
   *  outlive the selection — and the sketch — it came from. */
  clipboard: Sketch | null = null;
  /** How many times the clipboard has been pasted since it was filled, so pastes cascade
   *  instead of landing on each other. */
  pastes = 0;
  /** The one gesture in progress, if any — pan, point drag, radius drag or rubber band.
   *  One field rather than four means a new gesture cannot be forgotten in `setSketch`, and
   *  the pointer handlers stay two lines each. */
  gesture: Gesture | null = null;
  /** The pointer that owns `gesture`; a second one is ignored until it lets go. */
  gesturePointer: number | null = null;
  /** Whether the last pointer over the canvas was a finger — what the pick tolerance is sized
   *  to (`pickPx`).  Set by every press and move, so a pen or a mouse taking over takes it back. */
  finger = false;
  /** `underParams` as a set, rebuilt when the diagnosis it came from is replaced. */
  movable: { owner: Diagnosis; set: Set<Param> } | null = null;
  /** A gesture moved geometry, so the null space no longer describes the pose on screen. */
  staleDiagnosis = false;
  private frame = 0;
  /** The eye swinging square on to a plane: its timer, so a second choice replaces the first. */
  private looking = 0;
  /** Swept solids' surfaces, refined in a worker and drawn as they arrive. */
  private readonly fields = new FieldPreview((error) => this.fieldArrived(error), (r) => this.onRefine(r));
  /** A source sync is running: `swap` re-enters `afterEdit`, and the second pass has nothing
   *  left to write.  One flag rather than a subtle argument about termination. */
  private syncing = false;

  constructor(readonly canvas: HTMLCanvasElement, doc: Document,
              readonly boxCanvas: HTMLCanvasElement | null = null) {
    this.doc = doc;
    this.box3d = new Box3D(boxCanvas);
    this.ctx = canvas.getContext('2d')!;
    this.drawOnFront();
    bindEvents(this);
  }

  /** There is no page: a document nothing has been chosen on is drawn on the front, brought in by
   *  the first press of a tool where the document does not say `use std` yet (`ensurePlane`). */
  private drawOnFront(): void {
    if (this.plane || this.pendingPlane) return;
    const front = this.doc.entity('std.front');
    if (front instanceof Plane) this.plane = front;
    else this.pendingPlane = 'std.front';
  }

  /** The drawing the source came to.  Read everywhere and written nowhere: a new drawing is a
   *  new elaboration, which is `setProgram`. */
  get sketch(): Sketch { return this.doc.sketch; }

  /** The source.  This is the document — what Save writes, what undo remembers, and what the
   *  panel shows. */
  get source(): string { return this.doc.text; }

  // -- coordinates ---------------------------------------------------------

  /* Two cameras.  `cam` is the eye's: a similarity over the picture plane the core projects the
   * workspace onto, which zooms and pans.  A **view's** camera is that composed with the view's
   * own map (`ViewCam`), and is what turns page coordinates into pixels — so `w2s` and `s2w` read
   * whichever view is being worked in: the one the painter is drawing (`inView`), else the
   * current plane, which is where a tool puts what it makes. */

  w2s(x: number, y: number): [number, number] { return this.viewCam().w2s(x, y); }

  s2w(sx: number, sy: number): [number, number] { return this.viewCam().s2w(sx, sy); }

  /** A page length in screen pixels, in the view being worked in. */
  len(w: number): number { return this.viewCam().len(w); }

  /** The eye length of one screen pixel — what the core sizes annotation and pick tolerances
   *  through, all of which it measures on the eye's picture plane. */
  get unit(): number { return this.cam.unit; }

  /** A screen length as the eye length it stands for. */
  world(px: number): number { return this.cam.world(px); }

  get width(): number { return this.canvas.clientWidth; }
  get height(): number { return this.canvas.clientHeight; }

  /** What of the workspace does not depend on the eye: the view each thing stands in, each view's
   *  place and the eye square on to it.  Asked once an edit — `afterEdit` forgets it, and a new
   *  drawing is a new sketch. */
  workspace(): Workspace {
    const c = this.wsCache;
    if (c && c.sketch === this.sketch) return c.ws;
    const ws = workspace(this.sketch);
    this.wsCache = { sketch: this.sketch, ws };
    return ws;
  }
  private wsCache: { sketch: Sketch; ws: Workspace } | null = null;

  /** Every view's camera, the page first: the core's map of it for the orbit, composed with the
   *  eye's camera.  Built once a frame — `draw` forgets them, since a drag can move a plane's own
   *  points and so its map — and asked of per point. */
  private cams(): ViewCam[] {
    const { az, el } = this.orbit;
    const k = this.cam;
    const c = this.camCache;
    if (c && c.sketch === this.sketch && c.az === az && c.el === el && c.scale === k.scale
        && c.x === k.originX && c.y === k.originY) return c.cams;
    const cams = maps(this.sketch, az, el).map((m) => k.through(m));
    const space = spacePoints(this.sketch, az, el);
    const eye = k.through([1, 0, 0, 0, 1, 0]);
    this.camCache = { sketch: this.sketch, az, el, scale: k.scale, x: k.originX, y: k.originY, cams,
                      space, eye };
    return cams;
  }
  private camCache: { sketch: Sketch; az: number; el: number; scale: number; x: number; y: number;
                      cams: ViewCam[]; space: ([number, number] | null)[]; eye: ViewCam } | null = null;

  /** The view the painter is drawing in, while it is — see `inView`. */
  private drawing: ViewCam | null = null;

  /** The plane being drawn on: the current plane's view, else the page's. */
  get activeView(): View {
    return this.plane ? this.plane.index : PAGE;
  }

  /** The view a tool reads its clicks off: the plane being drawn on. */
  get toolView(): View {
    return this.activeView;
  }

  /** A view's camera, or null for a figure that stands in no one view. */
  camOf(view: View): ViewCam | null {
    return ofView(this.cams(), view);
  }

  /** The camera of the view being worked in: the painter's, else the tool's. */
  viewCam(): ViewCam {
    return this.drawing ?? this.camOf(this.toolView) ?? this.cams()[0];
  }

  /** The view a point stands in. */
  viewOf(p: Point): View {
    return this.workspace().views.point[p.index] ?? PAGE;
  }

  /** The view an entity is drawn in — a point's own, a figure's, or `NOWHERE` for one whose points
   *  stand in views apart in space (a projector between two views is drawn end to end instead). */
  viewOfEntity(e: Primitive): View {
    const table = (this.workspace().views as Record<string, View[] | undefined>)[e.kind];
    return table?.[e.index] ?? PAGE;
  }

  /** Where a point is seen, whichever view it stands in — a point in space where the core says the
   *  eye sees it, through the eye's own camera. */
  seen(p: Point): [number, number] {
    this.cams();
    const at = this.camCache!.space[p.index];
    if (at) return this.camCache!.eye.w2s(...at);
    return (this.camOf(this.viewOf(p)) ?? this.viewCam()).w2s(...p.xy);
  }

  /** Where a canvas point is on a view's page — where the eye's ray through it meets that plane. */
  s2wIn(view: View, sx: number, sy: number): [number, number] {
    return (this.camOf(view) ?? this.viewCam()).s2w(sx, sy);
  }

  /** Run `fn` in a view: every `w2s`, `s2w` and `len` inside it is that view's.  Nothing runs for
   *  a figure that stands in no one view, which has nowhere to be drawn. */
  inView<T>(view: View, fn: () => T): T | undefined {
    const cam = this.camOf(view);
    return cam ? this.withCam(cam, fn) : undefined;
  }

  /** Run `fn` through a view's camera already in hand. */
  withCam<T>(cam: ViewCam, fn: () => T): T {
    const outer = this.drawing;
    this.drawing = cam;
    try {
      return fn();
    } finally {
      this.drawing = outer;
    }
  }

  /** Whether a point stands in space, in no view: seen where it is, dragged where it is seen. */
  inSpace(p: Point): boolean {
    this.cams();
    return this.camCache!.space[p.index] != null;
  }

  /** A canvas point on the eye's picture plane, which is where the core asks what is there. */
  eye(sx: number, sy: number): [number, number] {
    return this.cam.s2w(sx, sy);
  }

  /** **Show the solid's surfaces in the box**, not only its edges.
   *
   *  View state, `underlay`'s rule and the orbit's: a drawing is the same drawing whether or not
   *  you are looking at it filled in, so this is never saved, exported, solved or undone.
   *
   *  It is on by default, and it was not while the box was strokes on a flat canvas: there a
   *  surface cost the boundary of every solid and a painter's order that is only very nearly
   *  right.  A depth buffer settles the order per pixel and a mesh is uploaded once, so the
   *  object may as well be shown as an object.  Off is still worth having — a wireframe is how
   *  you see the far side of a part — and is what the toggle means. */
  showSolid = true;

  /** **The workspace's three.js renderer.**  Built once and kept, because a WebGL context is a scarce
   *  thing a browser hands out and dropping one per toggle would eventually get none. */
  readonly box3d: Box3D;

  setShowSolid(on: boolean): void {
    if (this.showSolid === on) return;
    this.showSolid = on;
    this.draw();               // the flat scene does not carry it: the box's own build does
  }

  /** The entity a scene item is drawn from, where one is — the one decode of the wire's
   *  `kind`/`index` pair, so the painter and the picks read it one way. */
  entityOf(it: Omit<Item, 'pts' | 'shade'>): Primitive | undefined {
    return it.kind !== undefined && it.index !== undefined
      ? this.doc.entityOf({ kind: it.kind, index: it.index }) : undefined;
  }

  /** The view a scene item belongs to, as a `Plane` — the other decode of the wire, for the pane
   *  `box3d` bolds as the plane being drawn on. */
  planeOf(it: Omit<Item, 'pts' | 'shade'>): Plane | null {
    const p = it.plane !== undefined ? this.doc.entityOf({ kind: 'plane', index: it.plane }) : null;
    return p instanceof Plane ? p : null;
  }

  /** End everything in flight, uncommitted: a gesture, a wobble animation, a dimension being
   *  carried, a tool's half-collected clicks, and the remembered workspace.  Called before the
   *  drawing is replaced (`swap`) and when a project's paper is shown (`pauseEditing`), so the
   *  list of things that can be mid-way is written once — a tool's
   *  pending points are proxies that die with the sketch, and a curve fit finished after a load
   *  would hand the new sketch the old one's points. */
  private settle(): void {
    this.stopAnimation();             // first: it restores into the sketch it started on
    abandonGesture(this);             // dropped, not ended: `end` would commit into what follows
    if (this.liveDim) this.endDimension(false);
    this.pending = [];
    this.pendingFit = [];
    this.planeSpec = null;
    this.planeAxis = null;
  }

  /** Frame everything the workspace shows — figures and solids, as the eye now sees them. */
  fit(): void {
    const { az, el } = this.orbit;
    // a first guess at the scale from the page, so the extent below is tessellated at a zoom near
    // the one it will be shown at rather than whatever the last document was left at
    if (this.sketch.points.length) {
      this.cam.fitTo([...this.sketch.drawnBounds()] as [number, number, number, number],
                     this.width, this.height);
    }
    const b = boundsSeen(this.sketch, this.unit, az, el);
    if (b) this.cam.fitTo(b, this.width, this.height);
    this.draw();
  }

  // -- sketch mutation -----------------------------------------------------

  /** Show a different document.  Loading one is not a step along the history — there is no
   *  future to return to any more — so it drops the redo stack; stepping uses `swap`.
   *
   *  A program that will not elaborate leaves the drawing alone and says so: a half-written
   *  source is a thing somebody is in the middle of typing, not a reason to lose their work. */
  setProgram(text: string, fit = true): boolean {
    return this.reread(text, fit, false);
  }

  /** Project navigation starts a separate edit history; undo must never cross file boundaries. */
  openProjectFile(text: string): boolean {
    if (!this.reread(text, true, false)) return false;
    this.undoStack = [];
    return true;
  }

  /** Finish transient interactions before showing a project's paper preview. */
  pauseEditing(): void { this.settle(); }

  /** **A new document**, and the whole of what that means.  Every way of getting one — File ▸
   *  New, Open, a test case — comes through here, so the intention is spelled once: the
   *  outgoing document goes on the undo stack as one step (so a load is undoable, and ⌘Z after
   *  it cannot land on an older state of a drawing that is gone), whatever was in flight is
   *  settled, the selection is not carried (it belonged to the other drawing), the box is left
   *  if the new drawing has no view to show, and the camera is refitted.  `setProgram` is the
   *  *edit* of the same shape — the program panel replacing the text — which keeps the history
   *  its own way and is not this. */
  load(text: string, fit = true): boolean {
    this.pushUndo();
    if (!this.reread(text, fit, false)) {
      this.dropUndo();
      return false;
    }
    this.onLoad();
    return true;
  }

  /** File ▸ New: a fresh sheet that says `use std`, so the first thing drawn is drawn on the
   *  front and has `std.origin` to be placed against. */
  newDocument(): void {
    this.load('use std\n');
  }

  /** Read a source afresh and make it the document — the one place `Document.read` is called, so
   *  a program that will not elaborate is refused in exactly one way.  `carry` says whether the
   *  selection is the same drawing's (an edit) or another's (a load). */
  private reread(text: string, fit: boolean, carry: boolean): boolean {
    let next: Document;
    try {
      next = Document.read(text);
    } catch {
      this.onStatus('the program could not be read');
      return false;
    }
    this.redoStack = [];
    this.swap(next, fit, carry);
    return true;
  }

  /** Adopt a sketch built some other way — an example, a JSON file, a fresh sheet — by lifting
   *  it into the program it is written as.  **The one migration seam**: past it, everything is
   *  a document.  The sketch is consumed. */
  setSketch(sk: Sketch, fit = true): void {
    let text: string;
    try {
      text = fromSketch(sk);
    } finally {
      sk.dispose();
    }
    this.load(text, fit);
  }

  /** Adopt an elaboration already in hand — the seam every structural edit goes through, so
   *  there is exactly one place where the drawing is replaced. */
  private swap(next: Document, fit: boolean, carry = false): void {
    this.settle();                    // before the swap: nothing in flight may reach the new sketch
    const held = carry ? this.namesOf(this.selected) : [];
    const heldSolids = carry ? this._solids : [];
    const heldPlane = carry && this.plane ? this.doc.nameOf(this.plane) : undefined;
    const old = this.doc;
    this.doc = next;
    // the outgoing elaboration owns a core sketch, and a wasm heap only grows
    if (old !== next) old.dispose();
    // the current plane crosses by name too, and only if the name still reaches a plane: an
    // edit keeps it, deleting it or loading another document lets it go
    // the selection first: its setter arms the current plane when a lone plane is picked, and
    // what the swap carries is the answer — otherwise a plane still selected but deliberately
    // *not* current (`choosePlane`) would be re-armed by the rebind
    this.selected = carry ? this.rebind(held) : [];
    // a solid crosses by name too; the face path is the source's words and needs no rebinding
    if (heldSolids.length) {
      const solids = new Map(this.doc.solids().map((s) => [s.name, s.index]));
      this._solids = heldSolids.flatMap((s) => {
        const index = solids.get(s.name);
        return index === undefined ? [] : [{ ...s, index }];
      });
    }
    const again = heldPlane ? this.doc.entity(heldPlane) : undefined;
    this.plane = again instanceof Plane ? again : null;
    this.drawOnFront();
    // a standard plane chosen before the document had it: brought in by the edit just taken
    if (this.pendingPlane) {
      const wanted = this.doc.entity(this.pendingPlane);
      if (wanted instanceof Plane) {
        this.plane = wanted;
        this.pendingPlane = null;
      } else if (!carry) {
        this.pendingPlane = null;   // another document: it chooses its own
      }
    }
    // a new document is looked at the way it is drawn: a flat one square on, one with solids or
    // with geometry off the front plane from three quarters
    if (fit && !carry) this.orbit = this.homeOrbit();
    this.highlight = [];
    this.litConstraint = null;
    this.pastes = 0;              // a fresh sheet: the next paste starts its cascade over
    this.releasePlan();
    this.afterEdit();
    this.onProgram();
    if (fit) this.fit();      // after the solve: loading a case can move the geometry a long way
  }

  /* -- a selection across a re-elaboration --------------------------------
   *
   * A proxy is interned on `(kind, index)` in one `Sketch`, so it dies with the elaboration that
   * made it.  A **name** does not: it is what the source calls the thing, and the source is what
   * survives an edit.  So a selection crosses by name, and the lookup is the source map's — the
   * front end does no indexing of its own. */

  private namesOf(ps: Primitive[]): string[] {
    return ps.map((p) => this.doc.nameOf(p)).filter((n): n is string => !!n);
  }

  private rebind(names: string[]): Primitive[] {
    return names.map((n) => this.doc.entity(n)).filter((p): p is Primitive => !!p);
  }

  /** Apply an edit the core computed.  `structural` re-elaborates and carries the selection
   *  across by name; `numeric` and `none` leave the drawing standing, because the core has said
   *  the topology cannot have moved and a compiled plan is still good. */
  apply(e: Edit, what?: string): boolean {
    if (e.refused) {
      this.onStatus(e.refused);
      return false;
    }
    if (e.kind === 'none') return false;
    this.pushUndo();
    if (!this.take(e.text, e.kind === 'numeric')) {
      this.dropUndo();
      return false;
    }
    if (what) this.onStatus(what);
    return true;
  }

  /** Take an elaboration already read as the document — for a caller that had to read it to
   *  know whether to take it at all (a dimension's text that does not elaborate is refused and
   *  left in its editor).  The selection crosses by name, as after any structural edit. */
  takeDocument(next: Document): void {
    this.redoStack = [];
    this.swap(next, false, true);
  }

  /** Elaborate the source again, as it stands — the selection crossing by name. */
  rereadSource(): boolean {
    return this.reread(this.source, false, true);
  }

  /** Take a new source as the document.
   *
   *  `numeric` is the core's word that the topology cannot have moved, so the drawing stands and
   *  the compiled plan and the selection with it — which is what keeps editing a dimension
   *  instant.  When the core would rather be re-elaborated it says so by refusing `retext`, and
   *  re-reading is always correct and only slower. */
  private take(text: string, numeric: boolean): boolean {
    if (numeric && this.doc.retext(text)) {
      this.redoStack = [];
      this.afterEdit();
      this.onProgram();
      return true;
    }
    return this.reread(text, false, true);
  }

  /** Bring the source back into step with a drawing a gesture changed.
   *
   *  A tool draws by mutating the elaborated sketch — that is how it gets to snap and solve with
   *  the pointer still down — so this is where the document catches up: a splice appending what
   *  was drawn, taking out what was deleted, and committing the seeds.  Everything somebody wrote
   *  is left alone, which is the difference between a document and a print-out of the drawing.
   *
   *  Safe mid-tool: `reconcile` extends the elaboration rather than replacing it, so a chaining
   *  tool's half-built polyline keeps the proxies it is holding.  Held off only while a *drag* is
   *  live (`syncSeeds` is the seam for that, once, at release) and while a dimension is being
   *  carried, which is a thing being said rather than a thing yet done. */
  syncSource(): void {
    if (this.syncing || this.anim || this.gesture || this.liveDim) return;
    this.syncing = true;
    try {
      const e = this.doc.reconcile();
      if (e.refused) return this.onStatus(e.refused);
      if (e.kind !== 'none') this.onProgram();
    } finally {
      this.syncing = false;
    }
  }

  /** Put where the drawing *is* back into the seeds it came from.  Run at the end of a gesture,
   *  never per frame: during a drag the text is stale, and that is correct — a drag is one edit,
   *  at the moment it is let go. */
  syncSeeds(): void {
    if (this.anim) return;      // a wobble is not where the drawing is; freezing it in would lie
    const e = this.doc.commitSeeds();
    if (e.kind === 'none') return;
    if (this.doc.retext(e.text)) return this.onProgram();
    // the core would rather be re-elaborated.  Re-read rather than drop it: dropping would leave
    // the source quietly describing where the drawing *was*, which is the one failure this whole
    // design exists to prevent.  No solve here — the drag has already done it, and `endGesture`
    // is about to tell the shell.
    this.reread(e.text, false, true);
  }


  /** Remember a state to come back to — the current one, or an earlier one a caller took a
   *  snapshot of before it knew whether the edit would come to anything.  A state is program
   *  text, so undo is exact: it restores what somebody wrote, comments and all. */
  pushUndo(state: string = this.source): void {
    this.undoStack.push(state);
    if (this.undoStack.length > 100) this.undoStack.shift();
    this.redoStack = [];              // a fresh edit is a new branch: the old future is gone
  }

  /** The edit that snapshot was taken for came to nothing, so take the snapshot back —
   *  for an edit that cannot know whether it will happen until it has tried. */
  dropUndo(): void {
    this.undoStack.pop();
  }

  undo(): void { this.step(this.undoStack, this.redoStack, 'undo'); }

  redo(): void { this.step(this.redoStack, this.undoStack, 'redo'); }

  /** One step along the history.  The state being left goes onto the other stack, so the two
   *  are mirror images and the pair walks the same line in both directions. */
  private step(from: string[], to: string[], what: string): void {
    const s = from.pop();
    if (!s) return this.onStatus(`nothing to ${what}`);
    let next: Document;
    try {
      next = Document.read(s);
    } catch {
      return this.onStatus(`could not ${what}`);
    }
    to.push(this.source);
    this.swap(next, false);
    this.onStatus(what);
  }

  /** Free the compiled plan.  A live point drag holds the core's pointer to it, so the gesture
   *  goes first — freeing a plan out from under one would leave the drag stepping freed memory,
   *  and on wasm that aborts rather than raising.  Every caller gets this, so no caller has to
   *  remember the order. */
  releasePlan(): void {
    if (this.planSolver && this.gesture) abandonGesture(this);
    if (this.lastSystem === this.planSolver?.system) this.lastSystem = null;
    this.planSolver?.dispose();
    this.planSolver = null;
    this.planKey = '';
  }

  /** The plan solver owns its System; anything else we compiled here is ours to free. */
  private releaseSystem(): void {
    if (this.lastSystem && this.lastSystem !== this.planSolver?.system) this.lastSystem.dispose();
    this.lastSystem = null;
  }

  /** The decomposition plan, compiled once per topology and replayed for dimension edits and
   *  drags. */
  plan(): PlanSolver {
    const key = this.sketch.topologyKey();
    if (!this.planSolver || key !== this.planKey) {
      this.releasePlan();
      this.planSolver = new PlanSolver(this.sketch, true);
      this.planKey = key;
    }
    return this.planSolver;
  }

  /** One solve by the selected path; keeps the compiled System for the diagnosis that follows. */
  private solveOnce(): SolveResult {
    this.releaseSystem();
    this.systemKey = this.sketch.topologyKey();
    if (this.usePlan && this.sketch.constraints.length) {
      const ps = this.plan();
      this.lastPlan = ps.solve(1e-9, true, this.method);     // reads and records sketch.branches
      this.lastSystem = ps.system;                           // borrowed, not ours to dispose
      return asSolveResult(this.lastPlan);
    }
    this.lastPlan = null;
    const sys = new System(this.sketch);
    this.lastSystem = sys;
    return sys.solve({ method: this.method });
  }

  solveNow(): SolveResult {
    this.lastResult = this.solveOnce();
    this.onChanged();
    this.draw();
    return this.lastResult;
  }

  private rediagnose(system: System | null): void {
    this.diagnosis = this.sketch.constraints.length ? diagnose(this.sketch, { system }) : null;
    this.staleDiagnosis = false;
  }

  /** Every mutation ends here.  A failed solve leaves the last good geometry on screen: the
   *  failure is reported by the diagnosis, not by exploded geometry (which would also mislead
   *  the conflict search). */
  afterEdit(): SolveResult | null {
    this.wsCache = null;
    // A dimension still being laid down is being *said*, not solved, and the drawing holds
    // still under the number while somebody decides where to put it and which one it is: no
    // solve, and no re-diagnosis either — nothing changes colour, no banner appears and
    // disappears under the pointer, and the constraint list does not rebuild on every kind the
    // number passes through.  The click that plants it is when all of it happens, once, and
    // when what it came to is reported — see `dimension.placeDimension`.
    if (this.liveDim?.placing) {
      this.draw();
      return this.lastResult;
    }
    if (this.autoSolve) {
      const xBefore = this.sketch.getX();
      this.lastResult = this.solveOnce();
      if (!this.lastResult.success) this.sketch.setX(xBefore);
    }
    // with auto-solve off nothing has recompiled since the edit, so the System we still hold was
    // built from a sketch that no longer exists: diagnosing against it names dead constraints
    const fresh = this.systemKey === this.sketch.topologyKey();
    if (!fresh) this.releaseSystem();
    this.rediagnose(fresh ? this.lastSystem : null);
    this.syncSource();        // the drawing changed, so the document has to say what it now says
    this.fields.start(this.doc);
    this.onChanged();
    this.draw();
    return this.lastResult;
  }

  /** How much finer than the preview's swept solids are refined (Options ▸ mesh fineness): view
   *  state, never the document's, so it is neither saved nor undone. */
  get meshFineness(): number { return this.fields.fineness; }
  set meshFineness(f: number) { this.fields.setFineness(f, this.doc); }

  /** A swept solid's surface arrived from the worker: the same sketch now shows more, so what was
   *  drawn from it is drawn again. */
  private fieldArrived(error?: string): void {
    this.box3d.invalidate();
    if (error) this.onStatus(error);
    this.draw();
  }

  // -- Stage 4: witness analysis and DOF animation --------------------------

  /** Witness analysis of the current sketch, cached until the next edit.  The structural
   *  diagnosis is already in hand, so this only adds the Jacobian work. */
  witnessReport(): WitnessReport | null {
    const d = this.diagnosis;
    if (!d) return null;
    if (this.witnessFor !== d) {
      this.witnessFor = d;
      this.witness = analyze(this.sketch);
    }
    return this.witness;
  }

  /** Animate the remaining internal DOFs (each null-space mode in turn); false if none. */
  startAnimation(): boolean {
    this.stopAnimation();             // a second click would otherwise leak the running interval
    const rep = this.witnessReport();
    const modes = rep ? rep.motions.filter((m) => !m.rigid) : [];
    if (!modes.length) return false;
    this.anim = {
      modes,
      sketch: this.sketch,
      x0: this.sketch.getX(),
      free: this.sketch.freeIndices(),
      amp: 0.06 * this.sketch.extent(),
      labels: modes.map((m) => m.moving.slice(0, 8).map((p) => p.name).join(', ')),
      t: 0,
      showing: -1,
    };
    this.animTimer = window.setInterval(() => this.animTick(), ANIM_DT * 1000);
    return true;
  }

  stopAnimation(): void {
    if (!this.anim) return;
    clearInterval(this.animTimer);
    this.animTimer = 0;
    const { x0, sketch } = this.anim;
    this.anim = null;
    sketch.setX(x0);                  // the sketch it started on, whatever is on screen now
    this.draw();
  }

  private animTick(): void {
    const a = this.anim;
    if (!a || a.sketch !== this.sketch) return;
    a.t += ANIM_DT;
    const k = Math.floor(a.t / ANIM_PERIOD) % a.modes.length;
    const phase = Math.sin((2 * Math.PI * (a.t % ANIM_PERIOD)) / ANIM_PERIOD);
    const x = Float64Array.from(a.x0);
    const v = a.modes[k].velocity;
    for (let i = 0; i < a.free.length; i++) x[a.free[i]] += a.amp * phase * v[i];
    this.sketch.setX(x);
    if (k !== a.showing) {
      a.showing = k;
      this.onStatus(`DOF ${k + 1}/${a.modes.length}: ${a.labels[k]}`);
    }
    this.draw();
  }

  stateOf(e: Primitive): string {
    return this.diagnosis?.entityState.get(e) ?? 'well';
  }
  // -- hit testing ---------------------------------------------------------

  /* The core's answers, asked where the eye sees things: the camera turns the click into a place
   * on the picture plane and the pixel tolerance into a length there, and every figure is measured
   * standing on its own plane.  Views that lie on top of one another on the page are nowhere near
   * one another in space, which is why none of this is asked of page coordinates any more. */

  /** How near, in pixels, a press must be to what it picks: a finger's reach or a cursor's. */
  get pickPx(): number { return this.finger ? TOUCH_PICK_PX : PICK_PX; }

  pickPoint(sx: number, sy: number, tol = this.pickPx): Point | null {
    const { az, el } = this.orbit;
    const { point, dist } = nearestSeen(this.sketch, az, el, ...this.eye(sx, sy));
    return point && dist < this.world(tol) ? point : null;
  }

  pick(sx: number, sy: number): Primitive | null {
    const { az, el } = this.orbit;
    return pickSeen(this.sketch, this.unit, az, el, ...this.eye(sx, sy), this.world(this.pickPx));
  }

  /** The object face under the canvas point, nearest the eye, and the solid it is a face of. */
  solidAt(sx: number, sy: number): SolidPick | null {
    const { az, el } = this.orbit;
    const hit = pickSolidSeen(this.sketch, az, el, ...this.eye(sx, sy));
    if (!hit) return null;
    const name = this.doc.solids().find((s) => s.index === hit.solid)?.name;
    return name ? { name, index: hit.solid, face: hit.face } : null;
  }

  /** Select a solid, or with `add` toggle it in the selection.  It lets the drawing's selection
   *  and the picture go, as selecting either of them lets the solids go. */
  pickSolid(pick: SolidPick, add = false): void {
    const others = this._solids.filter((s) => s.index !== pick.index);
    this._solids = !add ? [pick] : others.length < this._solids.length ? others : [...others, pick];
    this._selected = [];
    this.dropImage();
  }

  /** The plane whose pane is under the canvas point, nearest the eye, as the chooser names it — a
   *  pane is the plane, so double-clicking one is choosing it.  Of panes standing in one place
   *  (`std.up` on `std.front`), the one the chooser offers. */
  paneAt(sx: number, sy: number): string | null {
    const { az, el } = this.orbit;
    const offered = this.planeChoices();
    for (const pl of panesSeen(this.sketch, az, el, ...this.eye(sx, sy))) {
      const name = this.doc.nameOf(pl);
      if (name && offered.includes(name)) return name;
    }
    return null;
  }

  // -- painting ------------------------------------------------------------

  draw(): void {
    this.camCache = null;     // a plane's own points may have moved, and its map with them
    if (this.frame) return;
    this.frame = requestAnimationFrame(() => { this.frame = 0; paint(this); });
  }

  resize(): void {
    const dpr = window.devicePixelRatio || 1;
    this.canvas.width = Math.round(this.width * dpr);
    this.canvas.height = Math.round(this.height * dpr);
    this.ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    // Coalesce resize with the solve and fit that follow it during startup. Painting
    // synchronously here projects the unsolved solid, only to discard it immediately.
    this.draw();
  }

  /** The dimension whose callout is under the cursor, if any.  Callouts are painted over the
   *  geometry and are what a click on them means, so they are picked before it.  The core does
   *  the test against the same layout it drew — including what a point in reach outranks — so
   *  what is picked is what is on screen. */
  pickCallout(sx: number, sy: number): Constraint | null {
    if (!this.showDimensions) return null;
    const { az, el } = this.orbit;
    const hit = calloutSeen(this.sketch, this.unit, az, el, ...this.eye(sx, sy), this.pickPx);
    const c = hit && (this.sketch.constraintById(hit.id) ?? null);
    // only a callout that is drawn answers a press
    return c && (c === this.litConstraint || this.showsCallouts(hit.view)) ? c : null;
  }

  /** Whether a view's callouts are shown: those of the plane being drawn on, as a CAD tool shows
   *  the dimensions of the sketch being edited — and of any view standing in the same place, as
   *  the page and `std.front` do.  The rest of the workspace's dimensions would be a thicket from
   *  any one eye; the dimension being written or focused is shown wherever it is. */
  showsCallouts(view: View): boolean {
    const ws = this.workspace();
    return view !== NOWHERE && placeOf(ws, view) === placeOf(ws, this.activeView);
  }

  /** The view a dimension's callout is laid out in — where a press on it is read, and where it
   *  is dragged to. */
  calloutView(c: Constraint): View {
    const k = dim.callouts(this.sketch, this.unit, [c.id]).items[0];
    return k ? k.view : NOWHERE;
  }

  /* -- what the shell asks the view to do ---------------------------------
   *
   * The work is next door; these are the names the rest of the app knows it by, so a caller
   * holds one object and never has to know which module a verb lives in. */

  setTool(tool: Tool): void { tools.setTool(this, tool); }

  /* The picture traced over, if there is one.  It is view state and not document state, so
   * none of this goes near `afterEdit`, the undo stack or the source — a repaint is the whole
   * of the consequence. */

  /** Put a picture in the middle of the view, replacing any that was there.  It arrives
   *  selected, so the handles that place it are there to be used at once. */
  traceImage(image: Bitmap, name: string, url: string | null = null): void {
    underlay.release(this.underlay);
    this.underlay = underlay.place(this, image, name, url);
    // select is where a picture is handled, and it arrives selected — so the tool comes with
    // it rather than being set by whichever caller happened to ask.  A drawing tool would
    // leave it `picked` with nothing on the canvas willing to answer for it.
    this.setTool('select');
    this.pickImage();
    this.onStatus(`tracing ${name} — drag it to place it, drag a corner to size and turn it, `
                  + 'Delete to remove it');
    this.onChanged();
    this.draw();
  }

  /** Select the picture.  The two selections are exclusive: a photograph is not a `Primitive`
   *  and cannot be constrained or dimensioned beside one, so holding both would only leave
   *  Delete ambiguous. */
  pickImage(): void {
    if (!this.underlay) return;
    this.underlay.picked = true;
    this.selected = [];
  }

  /** And the other way: anything that selects geometry lets the picture go. */
  dropImage(): void {
    if (this.underlay) this.underlay.picked = false;
  }

  /** Take it away again.  The sentence is here and not at the callers: the menu item and the
   *  Delete key remove the same picture, and a removal reported two ways (or, as it was, one
   *  way and silently) is two removals as far as anyone reading the status line is concerned. */
  removeImage(): void {
    const gone = this.underlay?.name;
    underlay.release(this.underlay);
    this.underlay = null;
    if (gone) this.onStatus(`removed ${gone}`);
    this.onChanged();
    this.draw();
  }

  /** Fade it, or bring it back — `by` is added to the opacity and the result is kept on 0…1.
   *  What it came to is read off the status line, which is where what is picked is named; it is
   *  not also toasted, since two places saying one number is two places to keep in step. */
  fadeImage(by: number): void {
    const u = this.underlay;
    if (!u) return;
    u.opacity = Math.min(1, Math.max(0, u.opacity + by));
    this.onPicked();
    this.draw();
  }
  cancelTool(): void { tools.cancelTool(this); }
  finishCurve(): void { tools.finishCurve(this); }
  finishSplineFit(): void { tools.finishSplineFit(this); }

  /** Arm the plane tool: the next two clicks say where the view sits, and this says what it is. */
  insertPlane(spec: PlaneSpec): void {
    this.planeSpec = spec;
    this.setTool('plane');
  }

  /** **The planes a sketch can be drawn on**, by name, for the workspace's chooser: the three
   *  standard ones first, whether or not the document has them yet, then every other plane the
   *  document names (`std.up` is the front plane turned, so it is not offered twice). */
  planeChoices(): string[] {
    const ws = this.workspace();
    const names = new Set<string>(STANDARD_PLANES);
    // where the standard planes the chooser offers stand: a standard datum standing there too is
    // one of them turned (`std.up` is the front), and is not offered twice
    const offered = new Set<View>([PAGE]);
    for (const n of STANDARD_PLANES) {
      const pl = this.doc.entity(n);
      if (pl instanceof Plane) offered.add(placeOf(ws, pl.index));
    }
    for (const pl of this.sketch.planes) {
      const n = this.doc.nameOf(pl);
      if (!n || (n.startsWith('std.') && !names.has(n) && offered.has(placeOf(ws, pl.index)))) continue;
      names.add(n);
    }
    return [...names];
  }

  /** The name of the plane being drawn on, as the chooser shows it. */
  get planeName(): string {
    if (this.pendingPlane) return this.pendingPlane;
    return (this.plane && this.doc.nameOf(this.plane)) || 'std.front';
  }

  /** **Sketch on a plane**: make it the one the next thing is drawn in, and turn the eye square
   *  on to it unless `look` is false (a double-click on its pane leaves the eye where it is).  A
   *  standard plane the document does not have yet is remembered by name and brought in by the
   *  first press of a tool (`ensurePlane`), so choosing one writes nothing. */
  choosePlane(name: string, look = true): void {
    const found = this.doc.entity(name);
    if (found instanceof Plane) {
      this.plane = found;
      this.pendingPlane = null;
    } else if ((STANDARD_PLANES as readonly string[]).includes(name)) {
      this.plane = null;
      this.pendingPlane = name;
    } else {
      return this.onStatus(`there is no plane ${name}`);
    }
    // the plane is where the next thing goes, not a thing picked: a selected plane would open the
    // constraints window over the drawing about to be made
    if (this.selected.some((e) => e instanceof Plane)) {
      this.selected = this.selected.filter((e) => !(e instanceof Plane));
    }
    if (look) this.lookAt(this.lookFor(name));
    this.onStatus(`drawing on ${name}`);
    this.onChanged();
  }

  /** The eye square on to a named plane: this document's own where it has the plane, else the
   *  standard library's, read once from a document of nothing but `use std`. */
  private lookFor(name: string): { az: number; el: number } {
    const here = this.doc.entity(name);
    if (here instanceof Plane) return lookOf(this.workspace(), here.index) ?? { ...this.orbit };
    if (name === 'std.front') return lookOf(this.workspace(), PAGE) ?? { ...this.orbit };
    const std = standardDocument();
    const pl = std.entity(name);
    return (pl instanceof Plane && lookOf(workspace(std.sketch), pl.index)) || { ...this.orbit };
  }

  /** Swing the eye to `to`, the short way round, and redraw as it goes.  The camera's own
   *  similarity is left where it is, so what was in the middle of the screen stays there. */
  lookAt(to: { az: number; el: number }): void {
    // where there is no clock to animate by (a test's view), the eye is simply there
    const clock = typeof window !== 'undefined' && typeof window.setInterval === 'function';
    if (clock) window.clearInterval(this.looking);
    const from = { ...this.orbit };
    let daz = to.az - from.az;
    daz -= 2 * Math.PI * Math.round(daz / (2 * Math.PI));
    if (!clock) {
      this.orbit = { az: from.az + daz, el: to.el };
      this.draw();
      return;
    }
    const t0 = performance.now();
    const step = (): void => {
      const t = Math.min(1, (performance.now() - t0) / LOOK_MS);
      const k = t * t * (3 - 2 * t);
      this.orbit = { az: from.az + daz * k, el: from.el + (to.el - from.el) * k };
      this.draw();
      if (t >= 1) {
        window.clearInterval(this.looking);
        this.looking = 0;
      }
    };
    this.looking = window.setInterval(step, 16);
    step();
  }

  /** Bring a chosen standard plane into the document before anything is drawn on it: the
   *  document gains `use std` (one undoable edit) and so the plane, which becomes current.
   *  True when the plane is ready to be drawn on. */
  ensurePlane(): boolean {
    if (!this.pendingPlane) return true;
    const name = this.pendingPlane;
    if (!this.apply(this.doc.addUse('std'), `use std, for ${name}`)) return false;
    const pl = this.doc.entity(name);
    if (!(pl instanceof Plane)) {
      this.onStatus(`${name} is not in the standard library`);
      return false;
    }
    this.plane = pl;
    this.pendingPlane = null;
    return true;
  }

  /** How a document opens: square on to the front when everything is drawn there, and from three
   *  quarters when it has a solid or anything stands on another plane. */
  private homeOrbit(): { az: number; el: number } {
    const ws = this.workspace();
    // anything standing elsewhere than the page's place — or across places, a projector — but a
    // plane's own origin, which stands wherever its plane does whether or not it is drawn on
    const origins = new Set(this.sketch.planes.map((pl) => pl.origin.index));
    const off = Object.entries(ws.views).some(([kind, vs]) =>
      (vs as View[]).some((v, i) => !(kind === 'point' && origins.has(i)) && placeOf(ws, v) !== PAGE));
    return off || objects(this.sketch).length ? { ...THREE_QUARTER } : { ...FRONT };
  }

  startDimension(targets: Constraint[], fresh: boolean, alt: DimAlt | null): boolean {
    return dimension.startDimension(this, targets, fresh, alt);
  }
  endDimension(commit: boolean): void { dimension.endDimension(this, commit); }
  rewriteDimension(text: string): string | null { return dimension.rewriteDimension(this, text); }

  addConstraints(...cs: Constraint[]): void { edit.addConstraints(this, ...cs); }
  removeConstraint(c: Constraint): void { edit.removeConstraint(this, c); }
  deleteSelected(): void { edit.deleteSelected(this); }
  toggleConstructionSelected(): void { edit.toggleConstructionSelected(this); }
  toggleFixSelected(): void { edit.toggleFixSelected(this); }
  copySelected(): number { return edit.copySelected(this); }
  cutSelected(): number { return edit.cutSelected(this); }
  pasteClipboard(): number { return edit.pasteClipboard(this); }
  resetCallouts(c?: Constraint | null): number { return edit.resetCallouts(this, c); }
}
