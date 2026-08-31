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
import { Param, Plane, Point, Primitive, Sketch } from '../core/model.js';
import { Attitude, Document, Edit, fromSketch } from '../core/program.js';
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
import * as underlay from './underlay.js';
import type { Bitmap, Underlay } from './underlay.js';

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
  | 'ellipse' | 'plane';

/** What the plane tool is armed with: the name the statement is to be given, if any, and the
 *  attitude — as text, since the statement spells it and the core reads it.  Where the plane
 *  sits on the page is the two clicks' business. */
export interface PlaneSpec {
  name?: string;
  attitude: Attitude | null;
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
  /** Paint the dimensioned constraints on the drawing as callouts. */
  showDimensions = true;
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
    if (prims.length) this.dropImage();
    // picking a view is choosing where to draw: a plane selected on its own becomes the current
    // one, and stays so past the selection — the point drawn next is in it
    if (prims.length === 1 && prims[0] instanceof Plane) this.plane = prims[0];
  }
  private _selected: Primitive[] = [];
  /** **The current plane**: the view every point a tool mints is drawn in, or null for the
   *  page.  A proxy of the current sketch, carried across a re-elaboration by name like the
   *  selection, and let go when the name no longer resolves — a deleted plane, or a load. */
  plane: Plane | null = null;
  /** What the plane tool will write when its two clicks land. */
  planeSpec: PlaneSpec | null = null;
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
  /** Per gesture frame: the sketch's structure and the constraint list are unchanged, so
   *  only the status line needs updating (a drag's numbers, a band's selection count). */
  onDragFrame: () => void = () => {};
  onStatus: (msg: string) => void = () => {};
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
  /** `underParams` as a set, rebuilt when the diagnosis it came from is replaced. */
  movable: { owner: Diagnosis; set: Set<Param> } | null = null;
  /** A gesture moved geometry, so the null space no longer describes the pose on screen. */
  staleDiagnosis = false;
  private frame = 0;
  /** A source sync is running: `swap` re-enters `afterEdit`, and the second pass has nothing
   *  left to write.  One flag rather than a subtle argument about termination. */
  private syncing = false;

  constructor(readonly canvas: HTMLCanvasElement, doc: Document) {
    this.doc = doc;
    this.ctx = canvas.getContext('2d')!;
    bindEvents(this);
  }

  /** The drawing the source came to.  Read everywhere and written nowhere: a new drawing is a
   *  new elaboration, which is `setProgram`. */
  get sketch(): Sketch { return this.doc.sketch; }

  /** The source.  This is the document — what Save writes, what undo remembers, and what the
   *  panel shows. */
  get source(): string { return this.doc.text; }

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

  /** Show a different document.  Loading one is not a step along the history — there is no
   *  future to return to any more — so it drops the redo stack; stepping uses `swap`.
   *
   *  A program that will not elaborate leaves the drawing alone and says so: a half-written
   *  source is a thing somebody is in the middle of typing, not a reason to lose their work. */
  setProgram(text: string, fit = true): boolean {
    return this.reread(text, fit, false);
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
    this.setProgram(text, fit);
  }

  /** Adopt an elaboration already in hand — the seam every structural edit goes through, so
   *  there is exactly one place where the drawing is replaced. */
  private swap(next: Document, fit: boolean, carry = false): void {
    this.stopAnimation();             // before the swap: it restores into the sketch it started on
    abandonGesture(this);            // before the swap: `end` would commit into the new sketch
    this.liveDim = null;              // and a dimension half-written belongs to the old document
    this.onDimension(null, null);
    const held = carry ? this.namesOf(this.selected) : [];
    const heldPlane = carry && this.plane ? this.doc.nameOf(this.plane) : undefined;
    const old = this.doc;
    this.doc = next;
    // the outgoing elaboration owns a core sketch, and a wasm heap only grows
    if (old !== next) old.dispose();
    // the current plane crosses by name too, and only if the name still reaches a plane: an
    // edit keeps it, deleting it or loading another document lets it go
    // the selection first: its setter arms the current plane when a lone plane is picked, and
    // what the swap carries is the answer — otherwise a plane still selected but deliberately
    // *not* current (`drawOnPage`) would be re-armed by the rebind
    this.selected = carry ? this.rebind(held) : [];
    const again = heldPlane ? this.doc.entity(heldPlane) : undefined;
    this.plane = again instanceof Plane ? again : null;
    this.highlight = [];
    this.litConstraint = null;
    this.pastes = 0;              // a fresh sheet: the next paste starts its cascade over
    this.pending = [];
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

  /** Draw the next things on the page rather than in a view.  Nothing in the document changes:
   *  the current plane is where a *future* point goes, so this is a repaint and a status line. */
  drawOnPage(): void {
    // and the view stops being the subject: left selected, the setter would arm it again as
    // the next press or re-elaboration went through `selected`
    if (this.selected.some((e) => e instanceof Plane)) {
      this.selected = this.selected.filter((e) => !(e instanceof Plane));
    }
    this.plane = null;
    this.onStatus('drawing on the page');
    this.onChanged();
    this.draw();
  }

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
