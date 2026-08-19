/* The sketch view: world/screen mapping, canvas painting, hit testing, the drawing tools and
 * the drag.  Every mutation funnels through `afterEdit`, which re-solves (when auto-solve is
 * on), re-diagnoses and notifies the shell exactly once. */
import * as io from '../core/io.js';
import * as C from '../core/constraints.js';
import { Constraint } from '../core/constraints.js';
import { PlanDrag, PlanResult, PlanSolver, asSolveResult, pppTriangles } from '../core/decompose.js';
import { Diagnosis, diagnose } from '../core/diagnose.js';
import { Arc, Circle, Line, Param, Point, Primitive, Sketch, threePointArc } from '../core/model.js';
import { Method, RadiusDrag, SolveResult, System, Triangle } from '../core/system.js';
import { Motion, WitnessReport, analyze, movingParams } from '../core/witness.js';

const PICK_PX = 8;

const COL = {
  bg: '#fafafa',
  axis: '#dddddd',
  line: '#1f77b4',
  circle: '#2ca02c',
  arc: '#ff7f0e',
  point: '#222222',
  fixed: '#d62728',
  sel: '#e377c2',
  preview: '#999999',
  highlight: '#9467bd',
  conflict: '#b3001b',
  bandFill: 'rgba(227, 119, 194, 0.10)',
};
/* entity colouring by constraint state (FreeCAD-style, but from the DM decomposition and the
 * conflict set rather than from a guess) */
const COL_STATE: Record<string, string> = {
  well: '#2ca02c', under: '#e69500', over: '#d62728', conflict: '#d62728',
};

const ANIM_DT = 0.03;        // seconds per animation tick
const ANIM_PERIOD = 2.0;     // seconds spent on each degree of freedom

interface Animation {
  modes: Motion[];
  x0: Float64Array;
  free: Int32Array;
  amp: number;
  labels: string[];
  t: number;
  showing: number;
}

export type Tool = 'select' | 'point' | 'line' | 'circle' | 'arc' | 'arc3';

export class SketchView {
  sketch: Sketch;
  scale = 6;
  originX = 80;
  originY = 500;
  tool: Tool = 'select';
  method: Method = 'dogleg';
  autoSolve = true;
  usePlan = false;
  colorByState = true;

  selected: Primitive[] = [];
  highlight: Primitive[] = [];
  pending: Point[] = [];
  diagnosis: Diagnosis | null = null;
  lastResult: SolveResult | null = null;
  lastPlan: PlanResult | null = null;

  onChanged: () => void = () => {};
  /** The active tool changed (including when Escape backs out of one). */
  onTool: (tool: Tool) => void = () => {};
  /** Per gesture frame: the sketch's structure and the constraint list are unchanged, so
   *  only the status line needs updating (a drag's numbers, a band's selection count). */
  onDragFrame: () => void = () => {};
  onStatus: (msg: string) => void = () => {};

  private ctx: CanvasRenderingContext2D;
  private cursor: [number, number] = [0, 0];
  private undoStack: string[] = [];
  private planSolver: PlanSolver | null = null;
  private planKey = '';
  private lastSystem: System | null = null;
  private witness: WitnessReport | null = null;
  private witnessFor: Diagnosis | null = null;
  private anim: Animation | null = null;
  private animTimer = 0;
  /** The one gesture in progress, if any — pan, point drag, radius drag or rubber band.
   *  One field rather than four means a new gesture cannot be forgotten in `setSketch`, and
   *  the pointer handlers stay two lines each. */
  private gesture: Gesture | null = null;
  /** `underParams` as a set, rebuilt when the diagnosis it came from is replaced. */
  private movable: { owner: Diagnosis; set: Set<Param> } | null = null;
  /** A gesture moved geometry, so the null space no longer describes the pose on screen. */
  private staleDiagnosis = false;
  private frame = 0;

  constructor(readonly canvas: HTMLCanvasElement, sketch: Sketch) {
    this.sketch = sketch;
    this.ctx = canvas.getContext('2d')!;
    this.bindEvents();
  }

  // -- coordinates ---------------------------------------------------------

  w2s(x: number, y: number): [number, number] {
    return [this.originX + x * this.scale, this.originY - y * this.scale];
  }

  s2w(sx: number, sy: number): [number, number] {
    return [(sx - this.originX) / this.scale, (this.originY - sy) / this.scale];
  }

  get width(): number { return this.canvas.clientWidth; }
  get height(): number { return this.canvas.clientHeight; }

  fit(): void {
    if (!this.sketch.points.length) return;
    const [x0, y0, x1, y1] = this.sketch.drawnBounds();
    this.scale = 0.8 * Math.min(this.width / (x1 - x0 || 1), this.height / (y1 - y0 || 1));
    const cx = (x0 + x1) / 2, cy = (y0 + y1) / 2;
    this.originX = this.width / 2 - cx * this.scale;
    this.originY = this.height / 2 + cy * this.scale;
    this.draw();
  }

  // -- sketch mutation -----------------------------------------------------

  setSketch(sk: Sketch, fit = true): void {
    this.abandonGesture();            // before the swap: `end` would commit into the new sketch
    this.sketch = sk;
    this.selected = [];
    this.highlight = [];
    this.pending = [];
    this.releasePlan();
    this.afterEdit();
    if (fit) this.fit();      // after the solve: loading a case can move the geometry a long way
  }

  pushUndo(): void {
    this.undoStack.push(io.dumps(this.sketch));
    if (this.undoStack.length > 100) this.undoStack.shift();
  }

  undo(): void {
    const s = this.undoStack.pop();
    if (!s) return;
    this.setSketch(io.loads(s), false);
    this.onStatus('undo');
  }

  private releasePlan(): void {
    this.planSolver?.dispose();
    this.planSolver = null;
    this.planKey = '';
  }

  /** The plan solver owns its System; anything else we compiled here is ours to free. */
  private releaseSystem(): void {
    if (this.lastSystem && this.lastSystem !== this.planSolver?.system) this.lastSystem.dispose();
    this.lastSystem = null;
  }

  /** The decomposition plan, compiled once per topology (constraints, entities, fixed flags)
   *  and replayed for dimension edits and drags. */
  plan(): PlanSolver {
    const sk = this.sketch;
    const key = [
      sk.constraints.length, sk.points.length, sk.lines.length, sk.circles.length, sk.arcs.length,
      sk.constraints.map((c) => c.typeName).join(','),
      sk.params.map((p) => (p.fixed ? 1 : 0)).join(''),
    ].join('|');
    if (!this.planSolver || key !== this.planKey) {
      this.releasePlan();
      this.planSolver = new PlanSolver(sk, true);
      this.planKey = key;
    }
    return this.planSolver;
  }

  /** One solve by the selected path; keeps the compiled System for the diagnosis that follows. */
  private solveOnce(): SolveResult {
    this.releaseSystem();
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
    if (this.autoSolve) {
      const xBefore = this.sketch.getX();
      this.lastResult = this.solveOnce();
      if (!this.lastResult.success) this.sketch.setX(xBefore);
    }
    this.rediagnose(this.lastSystem);
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
      this.witness = analyze(this.sketch, null, { overIds: new Set(d.over) });
    }
    return this.witness;
  }

  /** Animate the remaining internal DOFs (each null-space mode in turn); false if none. */
  startAnimation(): boolean {
    const rep = this.witnessReport();
    const modes = rep ? rep.motions.filter((m) => !m.rigid) : [];
    if (!modes.length) return false;
    this.anim = {
      modes,
      x0: this.sketch.getX(),
      free: this.sketch.freeIndices(),
      amp: 0.06 * this.sketch.extent(),
      labels: modes.map((m) => movingParams(m).slice(0, 8).map((p) => p.name).join(', ')),
      t: 0,
      showing: -1,
    };
    this.animTimer = window.setInterval(() => this.animTick(), ANIM_DT * 1000);
    this.onStatus(`animating ${modes.length} remaining DOF (click or Esc to stop)`);
    return true;
  }

  stopAnimation(): void {
    if (!this.anim) return;
    clearInterval(this.animTimer);
    this.sketch.setX(this.anim.x0);
    this.anim = null;
    this.draw();
  }

  private animTick(): void {
    const a = this.anim;
    if (!a) return;
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

  /** Add one or more constraints as a single edit: one undo entry, one solve, one
   *  diagnosis.  A multi-entity action (an equality set, Horizontal over several lines)
   *  is one thing the user did, so it should take one Ctrl+Z to undo. */
  addConstraints(...cs: Constraint[]): void {
    if (!cs.length) return;
    this.pushUndo();
    this.sketch.add(...cs);
    const res = this.afterEdit();
    const d = this.diagnosis;
    const st = d?.status ?? 'well';
    const kinds = [...new Set(cs.map((c) => c.typeName))].join(' + ');
    const what = cs.length === 1 ? kinds : `${cs.length} × ${kinds}`;
    const why = st === 'conflict' && d?.conflicts?.length
      ? ` — CONFLICT, remove one of: ${d.conflicts.map((k) => io.describe(k, this.sketch)).join(', ')}`
      : st === 'over' ? ' — redundant (consistent) with existing constraints'
      : res && !res.success ? ' — solver did NOT converge' : '';
    this.onStatus(`added ${what}${why}`);
  }

  removeConstraint(c: Constraint): void {
    this.pushUndo();
    this.sketch.remove(c);
    this.afterEdit();
  }

  deleteSelected(): void {
    if (!this.selected.length) return;
    this.pushUndo();
    const n = this.selected.length;
    this.setSketch(io.without(this.sketch, this.selected), false);
    this.onStatus(`deleted ${n} entities`);
  }

  toggleFixSelected(): void {
    const pts = this.selected.filter((e): e is Point => e instanceof Point);
    if (!pts.length) return;
    this.pushUndo();
    const allFixed = pts.every((p) => p.isFixed);
    for (const p of pts) p.fix(!allFixed);
    this.afterEdit();
  }

  // -- hit testing ---------------------------------------------------------

  private pickPoint(sx: number, sy: number, tol = PICK_PX): Point | null {
    const { point, dist } = this.sketch.nearestPoint(...this.s2w(sx, sy));
    return point && dist * this.scale < tol ? point : null;
  }

  pick(sx: number, sy: number): Primitive | null {
    const p = this.pickPoint(sx, sy);
    if (p) return p;
    let best: Primitive | null = null;
    let bd = PICK_PX;
    for (const ln of this.sketch.lines) {
      const d = segDist([sx, sy], this.w2s(...ln.p1.xy), this.w2s(...ln.p2.xy));
      if (d < bd) { best = ln; bd = d; }
    }
    for (const c of this.sketch.circles) {
      const cs = this.w2s(...c.center.xy);
      const d = Math.abs(Math.hypot(sx - cs[0], sy - cs[1]) - Math.abs(c.radius.value) * this.scale);
      if (d < bd) { best = c; bd = d; }
    }
    for (const a of this.sketch.arcs) {
      const cs = this.w2s(...a.center.xy);
      const d = Math.abs(Math.hypot(sx - cs[0], sy - cs[1]) - Math.abs(a.radius.value) * this.scale);
      if (d >= bd) continue;
      let ang = Math.atan2(-(sy - cs[1]), sx - cs[0]);
      const [a0, a1] = a.angles();
      while (ang < a0) ang += 2 * Math.PI;
      if (ang <= a1) { best = a; bd = d; }
    }
    return best;
  }

  // -- painting ------------------------------------------------------------

  draw(): void {
    if (this.frame) return;
    this.frame = requestAnimationFrame(() => { this.frame = 0; this.paint(); });
  }

  resize(): void {
    const dpr = window.devicePixelRatio || 1;
    this.canvas.width = Math.round(this.width * dpr);
    this.canvas.height = Math.round(this.height * dpr);
    this.ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    this.paint();
  }

  private paint(): void {
    const ctx = this.ctx;
    const w = this.width, h = this.height;
    ctx.save();
    ctx.fillStyle = COL.bg;
    ctx.fillRect(0, 0, w, h);
    ctx.lineCap = 'round';
    const [ox, oy] = this.w2s(0, 0);
    ctx.strokeStyle = COL.axis;
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(0, oy + 0.5); ctx.lineTo(w, oy + 0.5);
    ctx.moveTo(ox + 0.5, 0); ctx.lineTo(ox + 0.5, h);
    ctx.stroke();

    const sk = this.sketch;
    const sel = new Set(this.selected);
    const hl = new Set(this.highlight);
    const strokeFor = (base: string, ent: Primitive, lw = 1.8): [string, number] => {
      if (sel.has(ent)) return [COL.sel, lw + 1.5];
      if (hl.has(ent)) return [COL.highlight, lw + 1];
      return [this.colorByState ? COL_STATE[this.stateOf(ent)] : base, lw];
    };

    for (const ln of sk.lines) {
      const [col, lw] = strokeFor(COL.line, ln);
      ctx.strokeStyle = col;
      ctx.lineWidth = lw;
      ctx.beginPath();
      ctx.moveTo(...this.w2s(...ln.p1.xy));
      ctx.lineTo(...this.w2s(...ln.p2.xy));
      ctx.stroke();
    }
    for (const c of sk.circles) {
      const [col, lw] = strokeFor(COL.circle, c);
      ctx.strokeStyle = col;
      ctx.lineWidth = lw;
      const [cx, cy] = this.w2s(...c.center.xy);
      ctx.beginPath();
      ctx.arc(cx, cy, Math.abs(c.radius.value) * this.scale, 0, 2 * Math.PI);
      ctx.stroke();
    }
    for (const a of sk.arcs) {
      const [col, lw] = strokeFor(COL.arc, a);
      ctx.strokeStyle = col;
      ctx.lineWidth = lw;
      this.arcPath(a.center.xy, Math.abs(a.radius.value) * this.scale, ...a.angles());
      ctx.stroke();
    }
    if (this.pending.length) this.paintPreview();
    if (this.diagnosis?.conflicts?.length) this.paintConflicts();

    for (const p of sk.points) {
      const [sx, sy] = this.w2s(...p.xy);
      const col = sel.has(p) ? COL.sel : hl.has(p) ? COL.highlight : p.isFixed ? COL.fixed
        : this.colorByState ? COL_STATE[this.stateOf(p)] : COL.point;
      ctx.fillStyle = col;
      if (p.isFixed) {
        ctx.fillRect(sx - 4, sy - 4, 8, 8);
      } else {
        ctx.beginPath();
        ctx.arc(sx, sy, 3.5, 0, 2 * Math.PI);
        ctx.fill();
      }
    }
    this.gesture?.paint?.(ctx);
    if (this.tool !== 'select') {                 // snap indicator
      const sp = this.pickPoint(...this.cursor);
      if (sp) {
        const [sx, sy] = this.w2s(...sp.xy);
        ctx.strokeStyle = COL.sel;
        ctx.lineWidth = 1.5;
        ctx.beginPath();
        ctx.arc(sx, sy, 7, 0, 2 * Math.PI);
        ctx.stroke();
      }
    }
    ctx.restore();
  }

  /** CCW world arc from a0 to a1; screen y is flipped, so the canvas angles are negated and
   *  the sweep runs counterclockwise in canvas terms. */
  private arcPath(centerXY: [number, number], r: number, a0: number, a1: number): void {
    const [cx, cy] = this.w2s(...centerXY);
    this.ctx.beginPath();
    this.ctx.arc(cx, cy, r, -a0, -a1, true);
  }

  /** Dashed red halo on every entity a culprit constraint references, and a label at each
   *  culprit's anchor — the culprits are what to remove, as opposed to geometry that merely
   *  turned red because it touches them. */
  private paintConflicts(): void {
    const ctx = this.ctx;
    const d = this.diagnosis!;
    const used = new Map<string, number>();
    ctx.save();
    ctx.setLineDash([7, 5]);
    ctx.lineWidth = 5;
    ctx.strokeStyle = COL.conflict;
    ctx.font = 'bold 13px system-ui, sans-serif';
    for (const c of d.conflicts ?? []) {
      const xs: number[] = [], ys: number[] = [];
      for (const e of c.entities()) {
        if (e instanceof Point) {
          const [sx, sy] = this.w2s(...e.xy);
          ctx.beginPath(); ctx.arc(sx, sy, 9, 0, 2 * Math.PI); ctx.stroke();
          xs.push(e.x.value); ys.push(e.y.value);
        } else if (e instanceof Line) {
          ctx.beginPath();
          ctx.moveTo(...this.w2s(...e.p1.xy));
          ctx.lineTo(...this.w2s(...e.p2.xy));
          ctx.stroke();
          xs.push(e.p1.x.value, e.p2.x.value);
          ys.push(e.p1.y.value, e.p2.y.value);
        } else if (e instanceof Circle) {
          const [sx, sy] = this.w2s(...e.center.xy);
          ctx.beginPath(); ctx.arc(sx, sy, Math.abs(e.radius.value) * this.scale, 0, 2 * Math.PI); ctx.stroke();
          xs.push(e.center.x.value); ys.push(e.center.y.value + e.radius.value);
        } else if (e instanceof Arc) {
          const [a0, a1] = e.angles();
          this.arcPath(e.center.xy, Math.abs(e.radius.value) * this.scale, a0, a1);
          ctx.stroke();
          const am = 0.5 * (a0 + a1);
          xs.push(e.center.x.value + e.radius.value * Math.cos(am));
          ys.push(e.center.y.value + e.radius.value * Math.sin(am));
        }
      }
      if (!xs.length) continue;
      const [ax, ay] = this.w2s(xs.reduce((a, b) => a + b, 0) / xs.length,
                                ys.reduce((a, b) => a + b, 0) / ys.length);
      const cell = `${Math.floor(ax / 40)},${Math.floor(ay / 40)}`;
      const n = used.get(cell) ?? 0;
      used.set(cell, n + 1);
      ctx.save();
      ctx.setLineDash([]);
      ctx.fillStyle = COL.conflict;
      ctx.fillText(`✗ ${io.describe(c, this.sketch)}`, ax + 8, ay - 8 - 18 * n);
      ctx.restore();
    }
    ctx.restore();
  }

  private paintPreview(): void {
    const ctx = this.ctx;
    ctx.save();
    ctx.setLineDash([5, 4]);
    ctx.strokeStyle = COL.preview;
    ctx.lineWidth = 1;
    const p0 = this.w2s(...this.pending[0].xy);
    const cur = this.cursor;
    /** A dashed line from the last point placed to the cursor. */
    const rubber = (): void => {
      ctx.beginPath();
      ctx.moveTo(...this.w2s(...this.pending[this.pending.length - 1].xy));
      ctx.lineTo(cur[0], cur[1]);
      ctx.stroke();
    };
    if (this.tool === 'line') {
      rubber();
    } else if (this.tool === 'circle') {
      ctx.beginPath();
      ctx.arc(p0[0], p0[1], Math.hypot(cur[0] - p0[0], cur[1] - p0[1]), 0, 2 * Math.PI);
      ctx.stroke();
    } else if (this.tool === 'arc3') {
      const g = this.pending.length === 2
        ? threePointArc(...this.pending[0].xy, ...this.pending[1].xy, ...this.s2w(cur[0], cur[1]))
        : null;
      if (g) {
        this.arcPath([g.cx, g.cy], g.r * this.scale, g.a0, g.a1);
        ctx.stroke();
      } else {
        rubber();                                    // one end so far, or a collinear cursor
      }
    } else if (this.tool === 'arc') {
      if (this.pending.length === 1) {
        rubber();
      } else {
        const ps = this.w2s(...this.pending[1].xy);
        const a0 = Math.atan2(-(ps[1] - p0[1]), ps[0] - p0[0]);
        let a1 = Math.atan2(-(cur[1] - p0[1]), cur[0] - p0[0]);
        if (a1 <= a0) a1 += 2 * Math.PI;
        const r = Math.hypot(ps[0] - p0[0], ps[1] - p0[1]);
        ctx.beginPath();
        ctx.arc(p0[0], p0[1], r, -a0, -a1, true);
        ctx.stroke();
      }
    }
    ctx.restore();
  }

  // -- interaction ---------------------------------------------------------

  setTool(tool: Tool): void {
    this.tool = tool;
    this.pending = [];
    this.canvas.classList.toggle('select', tool === 'select');
    this.canvas.style.cursor = '';                // drop any hover affordance
    this.onTool(tool);
    this.onStatus(`tool: ${tool}`);
    this.draw();
  }

  /** Escape, in stages: stop a DOF animation, then drop the points the active tool has
   *  collected so far, then leave the tool for Select.  Repeated presses always end up
   *  somewhere calmer rather than doing nothing. */
  cancelTool(): void {
    if (this.anim) {
      this.stopAnimation();
      return;
    }
    if (this.pending.length) {
      this.pending = [];
      this.onStatus(`${this.tool}: cancelled — Esc again for Select`);
      this.draw();
      return;
    }
    if (this.tool !== 'select') this.setTool('select');
  }

  private local(e: PointerEvent | WheelEvent): [number, number] {
    const r = this.canvas.getBoundingClientRect();
    return [e.clientX - r.left, e.clientY - r.top];
  }

  private bindEvents(): void {
    const cv = this.canvas;
    cv.addEventListener('pointerdown', (e) => {
      cv.setPointerCapture(e.pointerId);
      this.onPointerDown(e);
    });
    cv.addEventListener('pointermove', (e) => this.onPointerMove(e));
    cv.addEventListener('pointerup', (e) => {
      cv.releasePointerCapture(e.pointerId);
      this.onPointerUp();
    });
    cv.addEventListener('contextmenu', (e) => e.preventDefault());
    cv.addEventListener('wheel', (e) => {
      e.preventDefault();
      const [sx, sy] = this.local(e);
      const f = 1.0015 ** (-e.deltaY * (e.deltaMode === 1 ? 16 : 1));
      this.originX = sx + (this.originX - sx) * f;
      this.originY = sy + (this.originY - sy) * f;
      this.scale *= f;
      this.draw();
    }, { passive: false });
  }

  private onPointerDown(e: PointerEvent): void {
    this.stopAnimation();
    const sp = this.local(e);
    this.cursor = sp;
    if (e.button === 1 || e.button === 2) {
      if (this.pending.length) this.cancelTool();
      else this.gesture = this.panGesture(sp);
      return;
    }
    if (e.button !== 0) return;
    if (this.tool !== 'select') {
      this.toolClick(sp);
      return;
    }
    const ent = this.pick(sp[0], sp[1]);
    if (!ent) {
      // nothing under the cursor: start a rubber band.  A press with no drag still just
      // clears the selection, because an empty box selects nothing.
      if (!e.shiftKey) this.selected = [];
      this.gesture = this.bandGesture(sp);
    } else if (e.shiftKey) {
      const i = this.selected.indexOf(ent);
      if (i >= 0) this.selected.splice(i, 1);
      else this.selected.push(ent);
    } else {
      if (!this.selected.includes(ent)) this.selected = [ent];
      if (ent instanceof Point && this.canMove(ent)) {
        this.pushUndo();
        const guards: Triangle[] | null = this.planSolver ? pppTriangles(this.planSolver.plan) : null;
        this.gesture = this.pointGesture(new PlanDrag(this.sketch, ent, ...this.s2w(sp[0], sp[1]), guards));
      } else if (this.isResizable(ent)) {
        this.pushUndo();
        this.gesture = this.radiusGesture(new RadiusDrag(this.sketch, ent, Math.abs(ent.radius.value)));
      } else if (!this.canMove(ent)) {
        this.onStatus(`${ent.kind} is fully constrained — nothing here is free to move`);
      }
    }
    this.onChanged();
    this.draw();
  }

  private snapOrNew(sp: [number, number]): Point {
    return this.pickPoint(sp[0], sp[1]) ?? this.sketch.point(...this.s2w(sp[0], sp[1]));
  }

  private toolClick(sp: [number, number]): void {
    const sk = this.sketch;
    if (!this.pending.length) this.pushUndo();
    if (this.tool === 'point') {
      this.snapOrNew(sp);
    } else if (this.tool === 'line') {
      const p = this.snapOrNew(sp);
      if (this.pending.length && p !== this.pending[this.pending.length - 1]) sk.line(this.pending[this.pending.length - 1], p);
      this.pending = [p];                            // continue the polyline
    } else if (this.tool === 'circle') {
      if (!this.pending.length) {
        this.pending = [this.snapOrNew(sp)];
      } else {
        const [x, y] = this.s2w(sp[0], sp[1]);
        const c = this.pending[0];
        sk.circle(c, Math.hypot(x - c.x.value, y - c.y.value) || 1);
        this.pending = [];
      }
    } else if (this.tool === 'arc3') {
      // two endpoints, then a point the arc must pass through.  That third click is
      // construction input, not a sketch point — the circumcircle is what it produces.
      if (this.pending.length < 2) {
        const p = this.snapOrNew(sp);
        if (!this.pending.length || p !== this.pending[0]) this.pending.push(p);
      } else {
        // the snap indicator is painted for every drawing tool, so honour it here: landing
        // the third click on a real point means the arc should stay on it, not merely start
        // out near it
        const [a, b] = this.pending;
        const on = this.pickPoint(sp[0], sp[1]);
        const arc = sk.arcThrough(a, b, on ? on.xy : this.s2w(sp[0], sp[1]));
        if (!arc) {
          this.onStatus('those three points are collinear — pick a point off the chord');
          return;                                    // keep the two ends, let them try again
        }
        if (on) sk.add(new C.PointOnCircle(on, arc));
        this.pending = [];
      }
    } else if (this.tool === 'arc') {
      const existing = this.pickPoint(sp[0], sp[1]) !== null;
      this.pending.push(this.snapOrNew(sp));
      if (this.pending.length === 3) {
        const [cpt, s, en] = this.pending;
        if (new Set([cpt, s, en]).size === 3) {
          if (!existing) {                           // freshly placed end point: put it on the radius
            const r = Math.hypot(s.x.value - cpt.x.value, s.y.value - cpt.y.value);
            const ang = Math.atan2(en.y.value - cpt.y.value, en.x.value - cpt.x.value);
            en.x.value = cpt.x.value + r * Math.cos(ang);
            en.y.value = cpt.y.value + r * Math.sin(ang);
          }
          sk.arc(cpt, s, en);
        }
        this.pending = [];
      }
    }
    this.releasePlan();
    this.afterEdit();
  }

  private onPointerMove(e: PointerEvent): void {
    const sp = this.local(e);
    this.cursor = sp;
    if (this.gesture) this.gesture.move(sp);
    else this.hover(sp);
    this.draw();
  }

  private onPointerUp(): void {
    this.endGesture();
  }

  /** The gesture finished: let it commit, then refresh — unless it changed nothing (a pan
   *  moves the camera, not the sketch). */
  private endGesture(): void {
    const g = this.gesture;
    if (!g) return;
    this.gesture = null;
    g.end?.();
    if (g.movedGeometry) this.staleDiagnosis = true;
    if (!g.transient) this.onChanged();
    this.draw();
  }

  /** The sketch is being replaced under a live gesture: drop it without committing, since
   *  `end` would write into a document the gesture never ran on. */
  private abandonGesture(): void {
    const g = this.gesture;
    this.gesture = null;
    g?.abandon?.();
  }

  /* -- the gestures.  Each owns its own state; `paint` is for the ones that draw. -- */

  private panGesture(from: [number, number]): Gesture {
    let last = from;
    return {
      transient: true,                 // the camera moved, not the sketch
      move: (sp) => {
        this.originX += sp[0] - last[0];
        this.originY += sp[1] - last[1];
        last = sp;
      },
    };
  }

  private pointGesture(drag: PlanDrag): Gesture {
    let reported = 0;
    return {
      movedGeometry: true,
      move: (sp) => {
        this.lastResult = drag.move(...this.s2w(sp[0], sp[1]));
        if (drag.flips.length > reported) {          // only announce new ones
          reported = drag.flips.length;
          this.onStatus(`⚠ solution branch flipped in ${reported} triangle(s) during this drag`);
        }
        this.onDragFrame();
      },
      end: () => {
        drag.end();
        for (const [k, v] of drag.branches()) drag.sketch.branches.set(k, v);
        this.releasePlan();      // the drag's plan pinned the dragged point; recompile lazily
      },
      abandon: () => {
        drag.end();              // disposes the compiled systems; no branches committed
        this.releasePlan();
      },
    };
  }

  private radiusGesture(drag: RadiusDrag): Gesture {
    return {
      movedGeometry: true,
      abandon: () => drag.end(),
      move: (sp) => {
        const [wx, wy] = this.s2w(sp[0], sp[1]);
        const c = drag.circle.center;
        this.lastResult = drag.move(Math.hypot(wx - c.x.value, wy - c.y.value));
        this.onDragFrame();
      },
      end: () => drag.end(),
    };
  }

  private bandGesture(from: [number, number]): Gesture {
    const base = [...this.selected];
    let to = from;                     // the gesture owns both corners, so paint reads no globals
    return {
      move: (sp) => {
        to = sp;
        // live preview: the canvas shows what would be selected, the status line the count
        this.selected = [...new Set([...base, ...this.boxContents(from, sp)])];
        this.onDragFrame();
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

  /** Entities lying entirely inside the box — "window" selection.  "All of it is inside"
   *  is exactly "its bounds are inside", so this asks the model rather than re-deriving each
   *  primitive's extent (a line's two endpoints, a circle's rim, an arc's sweep). */
  private boxContents(from: [number, number], to: [number, number]): Primitive[] {
    const a = this.s2w(from[0], from[1]);
    const b = this.s2w(to[0], to[1]);
    const x0 = Math.min(a[0], b[0]), x1 = Math.max(a[0], b[0]);
    const y0 = Math.min(a[1], b[1]), y1 = Math.max(a[1], b[1]);
    return this.sketch.primitives().filter((e) => {
      const bb = e.bounds();
      return bb[0] >= x0 && bb[1] >= y0 && bb[2] <= x1 && bb[3] <= y1;
    });
  }

  /** Cursor affordance: what a press here would grab. */
  private hover(sp: [number, number]): void {
    if (this.tool !== 'select') return;
    const ent = this.pick(sp[0], sp[1]);
    this.canvas.style.cursor = ent instanceof Point && this.canMove(ent) ? 'grab'
      : this.isResizable(ent) ? 'ew-resize' : '';
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
  private canMove(e: Primitive): boolean {
    const free = e.params.filter((p) => !p.fixed);
    if (!free.length) return false;
    if (!this.diagnosis || this.staleDiagnosis) return true;
    if (this.movable?.owner !== this.diagnosis) {
      this.movable = { owner: this.diagnosis, set: new Set(this.diagnosis.underParams) };
    }
    return free.some((p) => this.movable!.set.has(p));
  }

  /** A circle or arc whose radius is free to follow the cursor. */
  private isResizable(e: Primitive | null): e is Circle | Arc {
    return (e instanceof Circle || e instanceof Arc) && !e.radius.fixed
      && (!this.diagnosis || this.staleDiagnosis || this.canMove(e));
  }
}

/** One pointer gesture in progress.  `move` gets canvas coordinates; `end` and `paint` are
 *  optional because pan needs neither and the rubber band needs both. */
interface Gesture {
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

function dist(a: [number, number], b: [number, number]): number {
  return Math.hypot(a[0] - b[0], a[1] - b[1]);
}

function segDist(p: [number, number], a: [number, number], b: [number, number]): number {
  const vx = b[0] - a[0], vy = b[1] - a[1];
  const L2 = vx * vx + vy * vy;
  if (L2 === 0) return dist(p, a);
  let t = ((p[0] - a[0]) * vx + (p[1] - a[1]) * vy) / L2;
  t = Math.max(0, Math.min(1, t));
  return dist(p, [a[0] + t * vx, a[1] + t * vy]);
}
