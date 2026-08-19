/* Compile a sketch to a flat evaluation plan, evaluate r(z) and J(z), solve.
 *
 * `System` groups the sketch's constraints by kernel type into blocks — pure arrays of
 * (kernel id, global parameter indices, constants) — and hands them to the C core, which
 * owns the residual/Jacobian loop, the sparsity structure and the solve iteration.  This
 * compile-to-plan boundary is the architectural seam: the object model stays here, the
 * numbers live in WebAssembly.
 */
import { Constraint, DragTarget } from './constraints.js';
import { K, KERNELS } from './kernels.js';
import { Point, Sketch } from './model.js';
import { Buf, IBuf, core, readI32, readU8 } from './wasm.js';

export type Method = 'dogleg' | 'lm';
export const METHODS: Method[] = ['dogleg', 'lm'];

/** Free parameters up to which the dense Jacobian path (exact minimum-norm step and a
 *  rank) is used; above it the sparse normal equations take over.  Mirrors the C default. */
export const DENSE_MAX = 120;

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
  row0: number;
}

const now = (): number => (typeof performance !== 'undefined' ? performance.now() : Date.now()) / 1000;

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

  private handle: number;
  private slotOf = new Map<Constraint, [number, number]>();
  private bx: Buf;
  private bz: Buf;
  private br: Buf;
  private bJ: Buf | null = null;
  private bInfo: IBuf;
  private bConst: Buf;
  private disposed = false;

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
      this.blocks.push({ kernelId: kid, constraints: cs, gidx, row0 });
      gidxAll.push(...gidx);
      kernelId.push(kid);
      count.push(cs.length);
      row0 += cs.length * k.nRes;
    }
    this.nRes = row0;

    const x0 = sketch.getX();
    const bufs = [
      new Buf(Math.max(n, 1)).set(x0),
      new IBuf(Math.max(this.nFree, 1)).set(this.free),
      new IBuf(Math.max(kernelId.length, 1)).set(kernelId),
      new IBuf(Math.max(count.length, 1)).set(count),
      new IBuf(Math.max(gidxAll.length, 1)).set(gidxAll),
      new Buf(Math.max(constsAll.length, 1)).set(constsAll),
      new IBuf(Math.max(soft.length, 1)).set(soft),
    ] as const;
    this.handle = core()._gcs_system_new(
      n, bufs[0].ptr, this.nFree, bufs[1].ptr, kernelId.length,
      bufs[2].ptr, bufs[3].ptr, bufs[4].ptr, bufs[5].ptr, bufs[6].ptr,
    );
    for (const b of bufs) b.release();

    this.bx = new Buf(Math.max(n, 1));
    this.bz = new Buf(Math.max(this.nFree, 1));
    this.br = new Buf(Math.max(this.nRes, 1));
    this.bInfo = new IBuf(5);
    this.bConst = new Buf(Math.max(constsAll.length, 1));
    this.hard = readU8(core()._gcs_system_hard(this.handle), Math.max(this.nRes, 1)).subarray(0, this.nRes);
  }

  dispose(): void {
    if (this.disposed) return;
    this.disposed = true;
    core()._gcs_system_free(this.handle);
    this.bx.release(); this.bz.release(); this.br.release(); this.bInfo.release(); this.bConst.release();
    if (this.bJ) this.bJ.release();
    this.handle = 0;
  }

  // -- constants -----------------------------------------------------------

  /** Push a constraint's (mutated) constants into the compiled plan — a moving drag target
   *  or an edited dimension.  Topology is unchanged, so no recompile. */
  updateConsts(c: Constraint): void {
    const slot = this.slotOf.get(c);
    if (!slot) return;
    const vals = c.consts();
    if (!vals.length) return;
    const b = new Buf(vals.length).set(vals);
    core()._gcs_system_set_consts(this.handle, slot[0], slot[1], b.ptr);
    b.release();
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
    if (!this.bJ) this.bJ = new Buf(Math.max(this.nRes * this.nFree, 1));
    this.bz.set(z);
    core()._gcs_system_jacobian_dense(this.handle, this.bz.ptr, this.bJ.ptr);
    return { rows: this.nRes, cols: this.nFree, data: this.bJ.copy() };
  }

  csrIndptr(): Int32Array {
    return readI32(core()._gcs_system_csr_indptr(this.handle), this.nRes + 1);
  }

  csrIndices(): Int32Array {
    return readI32(core()._gcs_system_csr_indices(this.handle), core()._gcs_system_nnz(this.handle));
  }

  /** max |r| over hard rows at z (default: the current sketch values) — what "solved" means. */
  maxHardResidual(z?: ArrayLike<number>): number {
    this.bz.set(z ?? this.z0());
    return core()._gcs_system_max_hard_residual(this.handle, this.bz.ptr);
  }

  /** max |residual| per constraint, from one vectorized evaluation. */
  constraintErrors(z?: ArrayLike<number>): Map<Constraint, number> {
    const nCons = this.constraints.length;
    const out = new Buf(Math.max(nCons, 1));
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

  /** First residual row of a constraint. */
  rowOf(c: Constraint): number {
    const slot = this.slotOf.get(c);
    if (!slot) return -1;
    const blk = this.blocks[slot[0]];
    return blk.row0 + slot[1] * KERNELS[blk.kernelId].nRes;
  }

  /** Structural Jacobian as a bipartite graph: adj[row] = sorted free columns with a
   *  structural nonzero, plus row -> owning constraint.  The public surface for diagnosis
   *  and decomposition, derived from the compiled blocks so it stays in step with what the
   *  solver actually evaluates. */
  structure(hardOnly = true): { adj: number[][]; rowC: Constraint[] } {
    const adj: number[][] = [];
    const rowC: Constraint[] = [];
    for (const blk of this.blocks) {
      const k = KERNELS[blk.kernelId];
      for (let i = 0; i < blk.constraints.length; i++) {
        const c = blk.constraints[i];
        if (hardOnly && c.soft) continue;
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
    const z = this.z0();
    if (this.nFree === 0 || this.nRes === 0) {
      const r = this.residuals(z);
      let n2 = 0;
      for (let i = 0; i < r.length; i++) n2 += r[i] * r[i];
      return this.result(0, Math.sqrt(n2), this.maxHard(r), 1, 0, 0, now() - t0, method, null);
    }
    const dense = opts.dense === undefined || opts.dense === null ? this.nFree <= DENSE_MAX : opts.dense;
    this.bz.set(z);
    core()._gcs_system_solve(
      this.handle, method === 'lm' ? 1 : 0,
      tol * this.scale, 1e-12, 1e-16 * this.scale,
      maxIter, opts.maxNfev ?? 0, dense ? 1 : 0, this.bz.ptr, this.bInfo.ptr,
    );
    const info = this.bInfo.copy();
    const zs = this.bz.copy();
    if (writeback) this.sketch.setX(this.fullX(zs));
    const r = this.residuals(zs);
    let nrm = 0;
    for (let i = 0; i < r.length; i++) nrm += r[i] * r[i];
    return this.result(info[0], Math.sqrt(nrm), this.maxHard(r), info[1], info[2], info[3],
                       now() - t0, method, info[4] < 0 ? null : info[4]);
  }

  private maxHard(r: Float64Array): number {
    let mx = 0;
    for (let i = 0; i < r.length; i++) if (this.hard[i]) { const a = Math.abs(r[i]); if (a > mx) mx = a; }
    return mx;
  }

  private result(status: number, nrm: number, mx: number, nfev: number, njev: number,
                 iterations: number, timeS: number, method: string, rank: number | null): SolveResult {
    return {
      success: status >= 0 && mx < 1e-6 * this.scale,
      status,
      message: STATUS[status] ?? 'unknown',
      residualNorm: nrm,
      maxResidual: mx,
      nfev, njev, timeS, method, iterations, rank,
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

/** Interactive drag of one point: pull toward the cursor with a soft target, then polish
 *  with the hard constraints only so they hold exactly.
 *
 *  Both systems are compiled once at drag start and reused for every move — dragging never
 *  re-analyses the sketch.  Stage 5 robustness: continuation (a far cursor jump is taken in
 *  increments so the solution tracks its homotopy branch) and order-type guards (a step that
 *  would flip a guarded triangle's orientation is retried with smaller increments, and an
 *  unavoidable flip is recorded and flagged). */
export class Drag {
  readonly PULL_ITER = 4;
  readonly POLISH_ITER = 20;

  readonly target: DragTarget;
  readonly polish: System;
  readonly pull: System;
  active = true;
  guards: Triangle[];
  flips: Triangle[] = [];
  private signs: boolean[];
  private maxStep: number;
  private lastGood: Float64Array;

  constructor(
    readonly sketch: Sketch,
    readonly point: Point,
    x: number,
    y: number,
    readonly method: Method = 'dogleg',
    weight = 1.0,
    guards: Triangle[] | null = null,
    maxStepRel = 0.05,
  ) {
    this.polish = new System(sketch);
    this.target = new DragTarget(point, x, y, weight);
    sketch.add(this.target);
    this.pull = new System(sketch);
    this.guards = guards ?? [];
    this.maxStep = maxStepRel * Math.max(1, sketch.extent());
    this.signs = this.guards.map((t) => orientation(t[0], t[1], t[2]) >= 0);
    this.lastGood = sketch.getX();
  }

  private step(x: number, y: number): SolveResult {
    this.target.setTarget(x, y);
    this.pull.updateConsts(this.target);
    this.pull.solve({ method: this.method, maxIter: this.PULL_ITER });
    return this.polish.solve({ method: this.method, maxIter: this.POLISH_ITER });
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

  end(): void {
    if (this.active) {
      this.sketch.remove(this.target);
      this.active = false;
      this.pull.dispose();
      this.polish.dispose();
    }
  }
}
