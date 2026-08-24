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
 * Every mutation funnels through `afterEdit`, which re-solves (when auto-solve is on),
 * re-diagnoses and notifies the shell exactly once. */
import * as io from '../core/io.js';
import * as dim from '../core/callout.js';
import { Constraint } from '../core/constraints.js';
import { PlanResult, PlanSolver, asSolveResult } from '../core/decompose.js';
import { Diagnosis, diagnose } from '../core/diagnose.js';
import { Param, Point, Primitive, Sketch } from '../core/model.js';
import { Method, SolveResult, System } from '../core/system.js';
import { Motion, WitnessReport, analyze } from '../core/witness.js';
import { Camera } from './camera.js';
import * as edit from './edit.js';
import * as dimension from './dimension.js';
import { abandonGesture, bindEvents } from './gesture.js';
import type { Gesture } from './gesture.js';
import type { DimAlt, LiveDim } from './dimension.js';
import { paint } from './paint.js';
import * as tools from './tools.js';

/* A dimension being written belongs to `dimension`, but it is the view a caller holds, so the
 * two types are published from here as well. */
export type { DimAlt, LiveDim } from './dimension.js';

export const PICK_PX = 8;

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
  | 'ellipse';



export class SketchView {
  sketch: Sketch;
  /** Where the drawing sits on the canvas — and the whole of the front end's linear algebra:
   *  every world/screen conversion in `app/` goes through it (see `camera.ts`). */
  readonly cam = new Camera();
  tool: Tool = 'select';
  method: Method = 'dogleg';
  autoSolve = true;
  usePlan = false;
  colorByState = true;
  /** Paint the dimensioned constraints on the drawing as callouts. */
  showDimensions = true;

  selected: Primitive[] = [];
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
  /** Per gesture frame: the sketch's structure and the constraint list are unchanged, so
   *  only the status line needs updating (a drag's numbers, a band's selection count). */
  onDragFrame: () => void = () => {};
  onStatus: (msg: string) => void = () => {};
  /** A dimension callout was clicked: the shell puts the focus on that constraint. */
  onPickConstraint: (c: Constraint) => void = () => {};
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
  /** Both histories hold serialised sketches: `undoStack` the states an edit moved away from,
   *  `redoStack` the ones an undo moved away from. */
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
  /** `underParams` as a set, rebuilt when the diagnosis it came from is replaced. */
  movable: { owner: Diagnosis; set: Set<Param> } | null = null;
  /** A gesture moved geometry, so the null space no longer describes the pose on screen. */
  staleDiagnosis = false;
  private frame = 0;

  constructor(readonly canvas: HTMLCanvasElement, sketch: Sketch) {
    this.sketch = sketch;
    this.ctx = canvas.getContext('2d')!;
    bindEvents(this);
  }

  // -- coordinates ---------------------------------------------------------

  /* The camera's own verbs, on the object everyone holds — moving it is asked of `cam`. */

  w2s(x: number, y: number): [number, number] { return this.cam.w2s(x, y); }

  s2w(sx: number, sy: number): [number, number] { return this.cam.s2w(sx, sy); }

  /** A world length in screen pixels. */
  len(w: number): number { return this.cam.len(w); }

  /** The world length of one screen pixel — what the core sizes annotation and pick
   *  tolerances through. */
  get unit(): number { return this.cam.unit; }

  /** A screen length as the world length it stands for — what a tolerance in pixels is worth
   *  where the geometry lives, since every measurement is the core's and made out there. */
  world(px: number): number { return this.cam.world(px); }

  get width(): number { return this.canvas.clientWidth; }
  get height(): number { return this.canvas.clientHeight; }

  fit(): void {
    if (!this.sketch.points.length) return;
    this.cam.fitTo(this.sketch.drawnBounds(), this.width, this.height);
    this.draw();
  }

  // -- sketch mutation -----------------------------------------------------

  /** Show a different sketch.  Loading one is not a step along the history — there is no
   *  future to return to any more — so it drops the redo stack; stepping uses `swap`. */
  setSketch(sk: Sketch, fit = true): void {
    this.redoStack = [];
    this.swap(sk, fit);
  }

  private swap(sk: Sketch, fit: boolean): void {
    this.stopAnimation();             // before the swap: it restores into the sketch it started on
    abandonGesture(this);            // before the swap: `end` would commit into the new sketch
    this.liveDim = null;              // and a dimension half-written belongs to the old document
    this.onDimension(null, null);
    this.sketch = sk;
    this.selected = [];
    this.highlight = [];
    this.litConstraint = null;
    this.pastes = 0;              // a fresh sheet: the next paste starts its cascade over
    this.pending = [];
    this.releasePlan();
    this.afterEdit();
    if (fit) this.fit();      // after the solve: loading a case can move the geometry a long way
  }

  /** Remember a state to come back to — the current one, or an earlier one a caller took a
   *  snapshot of before it knew whether the edit would come to anything. */
  pushUndo(state: string = io.dumps(this.sketch)): void {
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
    to.push(io.dumps(this.sketch));
    this.swap(io.loads(s), false);
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
    this.onChanged();
    this.draw();
    return this.lastResult;
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

  /* Both of these are the core's answer to a question asked in world coordinates: the camera
   * turns the click into a place and the pixel tolerance into a length, and the geometry
   * happens out there.  It is what makes clicking a thing and constraining it agree about
   * where it is — the pick measures the same figure the core drew. */

  pickPoint(sx: number, sy: number, tol = PICK_PX): Point | null {
    const { point, dist } = this.sketch.nearestPoint(...this.s2w(sx, sy));
    return point && dist < this.world(tol) ? point : null;
  }

  pick(sx: number, sy: number): Primitive | null {
    return this.sketch.pick(...this.s2w(sx, sy), this.world(PICK_PX));
  }

  // -- painting ------------------------------------------------------------

  draw(): void {
    if (this.frame) return;
    this.frame = requestAnimationFrame(() => { this.frame = 0; paint(this); });
  }

  resize(): void {
    const dpr = window.devicePixelRatio || 1;
    this.canvas.width = Math.round(this.width * dpr);
    this.canvas.height = Math.round(this.height * dpr);
    this.ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    paint(this);
  }

  /** The dimension whose callout is under the cursor, if any.  Callouts are painted over the
   *  geometry and are what a click on them means, so they are picked before it.  The core does
   *  the test against the same layout it drew — including what a point in reach outranks — so
   *  what is picked is what is on screen. */
  pickCallout(sx: number, sy: number): Constraint | null {
    if (!this.showDimensions) return null;
    const id = dim.pick(this.sketch, this.unit, ...this.s2w(sx, sy), PICK_PX);
    return id < 0 ? null : this.sketch.constraintById(id) ?? null;
  }

  /* -- what the shell asks the view to do ---------------------------------
   *
   * The work is next door; these are the names the rest of the app knows it by, so a caller
   * holds one object and never has to know which module a verb lives in. */

  setTool(tool: Tool): void { tools.setTool(this, tool); }
  cancelTool(): void { tools.cancelTool(this); }
  finishCurve(): void { tools.finishCurve(this); }
  finishSplineFit(): void { tools.finishSplineFit(this); }

  startDimension(targets: Constraint[], fresh: boolean, alt: DimAlt | null): boolean {
    return dimension.startDimension(this, targets, fresh, alt);
  }
  endDimension(commit: boolean): void { dimension.endDimension(this, commit); }

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
