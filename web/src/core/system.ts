/* Compile a sketch to a flat evaluation plan, evaluate r(z) and J(z), solve.
 *
 * `System` groups the sketch's constraints by kernel type into blocks — pure arrays of
 * (kernel id, global parameter indices, constants) — and hands them to the C core, which
 * owns the residual/Jacobian loop, the sparsity structure and the solve iteration.  This
 * compile-to-plan boundary is the architectural seam: the object model stays here, the
 * numbers live in WebAssembly.
 */
import { Constraint, DragTarget, Radius } from './constraints.js';
import { K, KERNELS } from './kernels.js';
import { Arc, Circle, Point, Sketch } from './model.js';
import { Buf, IBuf, core, readU8 } from './wasm.js';

export type Method = 'dogleg' | 'lm';
export const METHODS: Method[] = ['dogleg', 'lm'];

const STATUS: Record<number, string> = {
  0: 'residual tolerance reached',
  1: 'step size below xtol',
  2: 'gradient below gtol',
  3: 'trust region collapsed / damping exhausted',
  4: 'max iterations reached',
  [-1]: 'failed',
};

export interface SolveResult {
  success: boolean;
  status: number;
  message: string;
  residualNorm: number;   /* over all residuals, soft ones included */
  maxResidual: number;    /* over hard residuals only — what "solved" means */
  nfev: number;
  njev: number;
  timeS: number;
  method: string;
  iterations: number;
  rank: number | null;    /* numerical rank of J at the solution (dense path) */
}

export interface Block {
  kernelId: K;
  constraints: Constraint[];
  gidx: Int32Array;       /* (n * nPar) global parameter index per local column */
}

/** Seconds, monotonic where available — the one timer the core uses. */
export const now = (): number =>
  (typeof performance !== 'undefined' ? performance.now() : Date.now()) / 1000;

export class System {
  readonly sketch: Sketch;
  readonly constraints: Constraint[];
  readonly blocks: Block[] = [];
  readonly free: Int32Array;
  readonly nFree: number;
  readonly colOf: Int32Array;
  readonly nRes: number;
  readonly extent: number;
  readonly scale: number;
  readonly hard: Uint8Array;

  private handle_: number;
  private slotOf = new Map<Constraint, [number, number]>();
  private bx: Buf;
  private bz: Buf;
  private br: Buf;
  private bJ: Buf | null = null;
  private bInfo: IBuf;
  private bConst: Buf;
  private bScratch: Buf | null = null;
  private disposed = false;

  /** The C handle.  Every entry point goes through here so a use-after-dispose throws
   *  instead of calling into freed heap. */
  private get handle(): number {
    if (this.disposed) throw new Error('System used after dispose()');
    return this.handle_;
  }

  constructor(sketch: Sketch) {
    this.sketch = sketch;
    this.constraints = [...sketch.constraints];   // snapshot: the plan is fixed at compile time
    const n = sketch.params.length;
    this.free = sketch.freeIndices();
    this.nFree = this.free.length;
    this.colOf = new Int32Array(n).fill(-1);
    for (let i = 0; i < this.nFree; i++) this.colOf[this.free[i]] = i;
    this.extent = sketch.extent();
    this.scale = Math.max(1, this.extent) ** 2;   // residual units for squared distances

    // group by kernel id, then sketch order — deterministic
    const byKernel = new Map<number, Constraint[]>();
    for (const c of this.constraints) {
      let l = byKernel.get(c.kernelId);
      if (!l) byKernel.set(c.kernelId, (l = []));
      l.push(c);
    }
    const kids = [...byKernel.keys()].sort((a, b) => a - b);
    const kernelId: number[] = [];
    const count: number[] = [];
    const gidxAll: number[] = [];
    const constsAll: number[] = [];
    const soft: number[] = [];
    let row0 = 0;
    for (const kid of kids) {
      const cs = byKernel.get(kid)!;
      const k = KERNELS[kid];
      const gidx = new Int32Array(cs.length * k.nPar);
      for (let i = 0; i < cs.length; i++) {
        const ps = cs[i].params;
        for (let c = 0; c < k.nPar; c++) gidx[i * k.nPar + c] = ps[c].index;
        this.slotOf.set(cs[i], [this.blocks.length, i]);
        if (k.nConst) constsAll.push(...cs[i].consts());
        soft.push(cs[i].soft ? 1 : 0);
      }
      this.blocks.push({ kernelId: kid, constraints: cs, gidx });
      gidxAll.push(...gidx);
      kernelId.push(kid);
      count.push(cs.length);
      row0 += cs.length * k.nRes;
    }
    this.nRes = row0;

    const x0 = sketch.getX();
    const bufs = [
      new Buf(n).set(x0),
      new IBuf(this.nFree).set(this.free),
      new IBuf(kernelId.length).set(kernelId),
      new IBuf(count.length).set(count),
      new IBuf(gidxAll.length).set(gidxAll),
      new Buf(constsAll.length).set(constsAll),
      new IBuf(soft.length).set(soft),
    ] as const;
    this.handle_ = core()._gcs_system_new(
      n, bufs[0].ptr, this.nFree, bufs[1].ptr, kernelId.length,
      bufs[2].ptr, bufs[3].ptr, bufs[4].ptr, bufs[5].ptr, bufs[6].ptr,
    );
    for (const b of bufs) b.release();

    this.bx = new Buf(n);
    this.bz = new Buf(this.nFree);
    this.br = new Buf(this.nRes);
    this.bInfo = new IBuf(5);
    this.bConst = new Buf(constsAll.length);
    this.hard = readU8(core()._gcs_system_hard(this.handle), this.nRes);
  }

  dispose(): void {
    if (this.disposed) return;
    core()._gcs_system_free(this.handle_);
    this.disposed = true;
    this.handle_ = 0;
    this.bx.release(); this.bz.release(); this.br.release(); this.bInfo.release(); this.bConst.release();
    this.bJ?.release();
    this.bScratch?.release();
  }

  // -- constants -----------------------------------------------------------

  /** Push a constraint's (mutated) constants into the compiled plan — a moving drag target
   *  or an edited dimension.  Topology is unchanged, so no recompile. */
  updateConsts(c: Constraint): void {
    const slot = this.slotOf.get(c);
    if (!slot) return;
    const vals = c.consts();
    if (!vals.length) return;
    if (!this.bScratch || this.bScratch.len < vals.length) {
      this.bScratch?.release();
      this.bScratch = new Buf(vals.length);       // reused: this runs once per drag frame
    }
    this.bScratch.view.set(vals);
    core()._gcs_system_set_consts(this.handle, slot[0], slot[1], this.bScratch.ptr);
  }

  /** Re-read every constraint's constants (after arbitrary dimension edits). */
  refreshConsts(): void {
    const all: number[] = [];
    for (const blk of this.blocks) {
      if (!KERNELS[blk.kernelId].nConst) continue;
      for (const c of blk.constraints) all.push(...c.consts());
    }
    if (!all.length) return;
    this.bConst.view.set(all);
    core()._gcs_system_set_all_consts(this.handle, this.bConst.ptr);
  }

  // -- evaluation ----------------------------------------------------------

  /** Free values of the current sketch geometry (also refreshes the core's copy of x). */
  z0(): Float64Array {
    this.bx.set(this.sketch.getX());
    core()._gcs_system_set_x(this.handle, this.bx.ptr);
    core()._gcs_system_get_z(this.handle, this.bz.ptr);
    return this.bz.copy();
  }

  fullX(z: ArrayLike<number>): Float64Array {
    this.bz.set(z);
    core()._gcs_system_full_x(this.handle, this.bz.ptr, this.bx.ptr);
    return this.bx.copy();
  }

  residuals(z: ArrayLike<number>): Float64Array {
    this.bz.set(z);
    core()._gcs_system_residuals(this.handle, this.bz.ptr, this.br.ptr);
    return this.br.copy();
  }

  jacobianDense(z: ArrayLike<number>): { rows: number; cols: number; data: Float64Array } {
    if (!this.bJ) this.bJ = new Buf(this.nRes * this.nFree);
    this.bz.set(z);
    core()._gcs_system_jacobian_dense(this.handle, this.bz.ptr, this.bJ.ptr);
    return { rows: this.nRes, cols: this.nFree, data: this.bJ.copy() };
  }

  /** max |r| over hard rows at z (default: the current sketch values) — what "solved" means. */
  maxHardResidual(z?: ArrayLike<number>): number {
    this.bz.set(z ?? this.z0());
    return core()._gcs_system_max_hard_residual(this.handle, this.bz.ptr);
  }

  /** max |residual| per constraint, from one vectorized evaluation. */
  constraintErrors(z?: ArrayLike<number>): Map<Constraint, number> {
    const out = new Buf(this.constraints.length);
    try {
      this.bz.set(z ?? this.z0());
      core()._gcs_system_constraint_errors(this.handle, this.bz.ptr, out.ptr);
      const v = out.view;
      const m = new Map<Constraint, number>();
      let i = 0;
      for (const blk of this.blocks) for (const c of blk.constraints) m.set(c, v[i++]);
      return m;
    } finally {
      out.release();
    }
  }

  /** Numerical rank of the Jacobian at z — the workhorse of Stage 2/4 diagnosis. */
  rank(z?: ArrayLike<number>, rcond = 1e-10, hardOnly = false): number {
    this.bz.set(z ?? this.z0());
    return core()._gcs_system_rank(this.handle, this.bz.ptr, rcond, hardOnly ? 1 : 0);
  }

  /** Structural Jacobian as a bipartite graph: adj[row] = sorted free columns with a
   *  structural nonzero, plus row -> owning constraint.  The public surface for diagnosis
   *  and decomposition, derived from the compiled blocks so it stays in step with what the
   *  solver actually evaluates.  Soft rows (drag targets) are never part of it. */
  structure(): { adj: number[][]; rowC: Constraint[] } {
    const adj: number[][] = [];
    const rowC: Constraint[] = [];
    for (const blk of this.blocks) {
      const k = KERNELS[blk.kernelId];
      for (let i = 0; i < blk.constraints.length; i++) {
        const c = blk.constraints[i];
        if (c.soft) continue;
        const cols = new Set<number>();
        for (let t = 0; t < k.nPar; t++) {
          const col = this.colOf[blk.gidx[i * k.nPar + t]];
          if (col >= 0) cols.add(col);
        }
        const sorted = [...cols].sort((a, b) => a - b);
        for (let t = 0; t < k.nRes; t++) { adj.push(sorted); rowC.push(c); }
      }
    }
    return { adj, rowC };
  }

  // -- solving --------------------------------------------------------------

  solve(opts: {
    method?: Method;
    tol?: number;          /* relative to extent^2 */
    maxNfev?: number;
    writeback?: boolean;
    maxIter?: number;
    dense?: boolean | null;
  } = {}): SolveResult {
    const method = opts.method ?? 'dogleg';
    const tol = opts.tol ?? 1e-14;
    const maxIter = opts.maxIter ?? 100;
    const writeback = opts.writeback ?? true;
    const t0 = now();
    this.z0();                                    // leaves the free values in bz
    // dense < 0 lets the C core pick by size; that threshold lives in one place, in newton.c
    const dense = opts.dense === undefined || opts.dense === null ? -1 : Number(opts.dense);
    core()._gcs_system_solve(
      this.handle, method === 'lm' ? 1 : 0,
      tol * this.scale, 1e-12, 1e-16 * this.scale,
      maxIter, opts.maxNfev ?? 0, dense, this.bz.ptr, this.bInfo.ptr,
    );
    const info = this.bInfo.view;
    if (writeback) {
      core()._gcs_system_get_x(this.handle, this.bx.ptr);   // the solve wrote z back into x
      this.sketch.setX(this.bx.view);
    }
    core()._gcs_system_residuals(this.handle, this.bz.ptr, this.br.ptr);
    return this.result(info, this.br.view, now() - t0, method);
  }

  /** Build the result from the core's info block and the residual still sitting in `br`. */
  private result(info: Int32Array, r: Float64Array, timeS: number, method: string): SolveResult {
    let n2 = 0, mx = 0;
    for (let i = 0; i < r.length; i++) {
      n2 += r[i] * r[i];
      if (this.hard[i]) { const a = Math.abs(r[i]); if (a > mx) mx = a; }
    }
    const [status, nfev, njev, iterations, rank] = info;
    return {
      success: status >= 0 && mx < 1e-6 * this.scale,
      status,
      message: STATUS[status] ?? 'unknown',
      residualNorm: Math.sqrt(n2),
      maxResidual: mx,
      nfev, njev, timeS, method, iterations,
      rank: rank < 0 ? null : rank,
    };
  }
}

/** One-shot: compile and solve, writing the result back into the sketch. */
export function solve(sketch: Sketch, opts: Parameters<System['solve']>[0] = {}): SolveResult {
  const s = new System(sketch);
  try {
    return s.solve(opts);
  } finally {
    s.dispose();
  }
}

export type Triangle = [Point, Point, Point];

/** Twice the signed area of (a, b, c) — the order-type invariant the drag guards. */
export function orientation(a: Point, b: Point, c: Point): number {
  return (b.x.value - a.x.value) * (c.y.value - a.y.value) - (b.y.value - a.y.value) * (c.x.value - a.x.value);
}

/** Continuation path from (x0,y0) to (x1,y1): waypoints no farther apart than maxStep, so a
 *  solution tracks its branch instead of teleporting across it.  Always at least one point. */
export function increments(x0: number, y0: number, x1: number, y1: number, maxStep: number): [number, number][] {
  const n = Math.max(1, Math.ceil(Math.hypot(x1 - x0, y1 - y0) / maxStep));
  const out: [number, number][] = [];
  for (let i = 1; i <= n; i++) out.push([x0 + ((x1 - x0) * i) / n, y0 + ((y1 - y0) * i) / n]);
  return out;
}

/** The pull/polish protocol every interactive drag shares.
 *
 *  A soft constraint pulls the geometry toward what the cursor asks for; the hard
 *  constraints are then polished on their own so they hold exactly.  Both systems are
 *  compiled once, at drag start, and reused for every move — dragging never re-analyses the
 *  sketch.  The compile order is load-bearing: `polish` must be built before the soft target
 *  joins the sketch, so it contains the hard constraints only. */
abstract class PullPolish<T extends Constraint> {
  readonly PULL_ITER = 4;   // the pull is a soft compromise; polish makes it exact
  readonly POLISH_ITER = 20;

  readonly polish: System;
  readonly pull: System;
  active = true;

  constructor(readonly sketch: Sketch, readonly target: T, readonly method: Method) {
    this.polish = new System(sketch);
    sketch.add(target);
    this.pull = new System(sketch);
  }

  /** One frame: push the target's new value in, pull, then make the hard ones exact. */
  protected pullPolish(): SolveResult {
    this.pull.updateConsts(this.target);
    this.pull.solve({ method: this.method, maxIter: this.PULL_ITER });
    return this.polish.solve({ method: this.method, maxIter: this.POLISH_ITER });
  }

  end(): void {
    if (!this.active) return;
    this.sketch.remove(this.target);
    this.active = false;
    this.pull.dispose();
    this.polish.dispose();
  }
}

/** Interactive drag of one point: pull toward the cursor, then polish.
 *
 *  Stage 5 robustness: continuation (a far cursor jump is taken in increments so the solution
 *  tracks its homotopy branch) and order-type guards (a step that would flip a guarded
 *  triangle's orientation is retried with smaller increments, and an unavoidable flip is
 *  recorded and flagged). */
export class Drag extends PullPolish<DragTarget> {
  guards: Triangle[];
  flips: Triangle[] = [];
  private signs: boolean[];
  private maxStep: number;
  private lastGood: Float64Array;

  constructor(
    sketch: Sketch,
    readonly point: Point,
    x: number,
    y: number,
    method: Method = 'dogleg',
    weight = 1.0,
    guards: Triangle[] | null = null,
    maxStepRel = 0.05,
  ) {
    super(sketch, new DragTarget(point, x, y, weight), method);
    this.guards = guards ?? [];
    this.maxStep = maxStepRel * Math.max(1, sketch.extent());
    this.signs = this.guards.map((t) => orientation(t[0], t[1], t[2]) >= 0);
    this.lastGood = sketch.getX();
  }

  private step(x: number, y: number): SolveResult {
    this.target.setTarget(x, y);
    return this.pullPolish();
  }

  private flipped(): number[] {
    const out: number[] = [];
    for (let i = 0; i < this.guards.length; i++) {
      const t = this.guards[i];
      if ((orientation(t[0], t[1], t[2]) >= 0) !== this.signs[i]) out.push(i);
    }
    return out;
  }

  /** One increment that would flip a guard: bisect the remaining interval from the last good
   *  state, keeping whatever prefix stays on the branch, within a sub-step budget. */
  private damped(tx: number, ty: number, budget: number): [SolveResult, number] {
    let res = this.step(tx, ty);
    while (this.flipped().length && budget > 0) {
      this.sketch.setX(this.lastGood);
      const [bx, by] = this.point.xy;
      res = this.step((bx + tx) / 2, (by + ty) / 2);
      budget--;
      if (this.flipped().length) continue;      // the flip is in the first half: bisect that
      this.lastGood = this.sketch.getX();
      res = this.step(tx, ty);                  // first half was clean: try the rest again
      budget--;
    }
    return [res, budget];
  }

  move(x: number, y: number): SolveResult {
    const t0 = now();
    const nFlips = this.flips.length;
    let budget = 12;                            // cap the sub-steps a single frame may spend
    const [px, py] = this.point.xy;
    this.lastGood = this.sketch.getX();
    let res = this.step(px, py);
    for (const [tx, ty] of increments(px, py, x, y, this.maxStep)) {
      res = this.step(tx, ty);
      if (this.guards.length && this.flipped().length) {
        [res, budget] = this.damped(tx, ty, budget);
        for (const k of this.flipped()) {       // unavoidable: accept, record, flag
          this.signs[k] = !this.signs[k];
          this.flips.push(this.guards[k]);
        }
      }
      this.lastGood = this.sketch.getX();
    }
    res.timeS = now() - t0;
    if (this.flips.length > nFlips) res.message = `order-type flip in ${this.flips.length - nFlips} triangle(s)`;
    return res;
  }
}

/** A `Radius` that does not have to hold: its residual is already exactly r - target, so the
 *  scalar pull needs no kernel of its own. */
function softRadius(circle: Circle | Arc, r: number): Radius {
  const target = new Radius(circle, r);
  target.soft = true;
  return target;
}

/** Interactive drag of a circle's or arc's radius — the scalar counterpart of `Drag`.
 *
 *  A radius that is fixed or dimensioned simply does not move: the polish wins, exactly as a
 *  point drag compromises on an over-constrained sketch.  An `EqualRadius` chain is a relation
 *  rather than a dimension, so the whole chain resizes together.  (The web app additionally
 *  refuses to *start* such a drag, using the diagnosis; that is a UI choice, not a property of
 *  this class.) */
export class RadiusDrag extends PullPolish<Radius> {
  constructor(sketch: Sketch, readonly circle: Circle | Arc, r: number, method: Method = 'dogleg') {
    super(sketch, softRadius(circle, r), method);
  }

  move(r: number): SolveResult {
    const t0 = now();
    this.target.r = Math.max(r, 1e-9);         // a radius through zero would flip the geometry
    const res = this.pullPolish();
    res.timeS = now() - t0;
    return res;
  }
}
