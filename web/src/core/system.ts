/* The compiled system, solving, and interactive dragging.
 *
 * `System` is the compile-once / evaluate-many seam: it owns a handle to the core's evaluation
 * plan, so the object model never enters the hot loop.  Call `dispose()` when you drop one (the
 * drags, the plan solver and diagnosis all do).
 */
import { Constraint } from './constraints.js';
import { Arc, Circle, KIND_ID, Point, Sketch } from './model.js';
import { core, takeJson, takeStr, withBuf } from './wasm.js';

export type Method = 'dogleg' | 'lm';
export const METHODS: Method[] = ['dogleg', 'lm'];
const METHOD_ID: Record<Method, number> = { dogleg: 0, lm: 1 };

/** Free params up to which J is dense (exact minimum-norm step + rank); sparse above. */
export const DENSE_MAX = 120;

export type Triangle = [Point, Point, Point];

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

/** Seconds, monotonic where available — the one timer the front end uses. */
export const now = (): number =>
  (typeof performance !== 'undefined' ? performance.now() : Date.now()) / 1000;

function readResult(out: Float64Array, message: string, method: string, t0: number): SolveResult {
  return {
    success: out[0] !== 0,
    status: out[1],
    message,
    residualNorm: out[2],
    maxResidual: out[3],
    nfev: out[4],
    njev: out[5],
    iterations: out[6],
    rank: out[7] < 0 ? null : out[7],
    timeS: now() - t0,
    method,
  };
}

export interface Structure {
  adj: number[][];
  rowC: Constraint[];
}

export class System {
  readonly nRes: number;
  readonly nFree: number;
  readonly nnz: number;
  /** Constraints the plan was compiled from — not the live sketch's count once edited. */
  readonly nConstraints: number;
  readonly scale: number;
  readonly extent: number;
  readonly hard: Uint8Array;
  readonly free: Int32Array;

  private handle_: number;
  private owned: boolean;
  private disposed = false;

  constructor(readonly sketch: Sketch, handle?: number, owned = true) {
    this.handle_ = handle ?? core().gcs_system_new(sketch.handle);
    this.owned = owned;
    const c = core();
    this.nRes = c.gcs_system_n_res(this.handle_);
    this.nFree = c.gcs_system_n_free(this.handle_);
    this.nnz = c.gcs_system_nnz(this.handle_);
    this.nConstraints = c.gcs_system_n_constraints(this.handle_);
    this.scale = c.gcs_system_scale(this.handle_);
    this.extent = sketch.extent();
    this.hard = withBuf(Math.max(this.nRes, 1), 1, (b) => {
      c.gcs_system_hard(this.handle_, b.ptr);
      return b.u8.slice(0, this.nRes);
    });
    this.free = withBuf(Math.max(this.nFree, 1), 4, (b) => {
      c.gcs_system_free_indices(this.handle_, b.ptr);
      return b.i32.slice(0, this.nFree);
    });
  }

  /** The core handle.  Every entry point goes through here so a use-after-dispose throws
   *  instead of calling into freed heap. */
  get handle(): number {
    if (this.disposed) throw new Error('System used after dispose()');
    return this.handle_;
  }

  dispose(): void {
    if (this.disposed) return;
    this.disposed = true;
    if (this.owned) core().gcs_system_free(this.handle_);
    this.handle_ = 0;
  }

  // -- constants -----------------------------------------------------------

  /** Push a constraint's (mutated) constants into the compiled plan — a moving drag target or an
   *  edited dimension.  Topology is unchanged, so no recompile. */
  updateConsts(c: Constraint): void {
    core().gcs_system_update_consts(this.handle, this.sketch.handle, c.id);
  }

  /** Re-read every constraint's constants (after arbitrary dimension edits). */
  refreshConsts(): void {
    core().gcs_system_refresh_consts(this.handle, this.sketch.handle);
  }

  /** First residual row of a constraint. */
  rowOf(c: Constraint): number {
    return core().gcs_system_row_of(this.handle, c.id);
  }

  // -- evaluation ----------------------------------------------------------

  /** Free values of the current sketch geometry (also refreshes the core's copy of x). */
  z0(): Float64Array {
    return withBuf(Math.max(this.nFree, 1), 8, (b) => {
      core().gcs_system_z0(this.handle, this.sketch.handle, b.ptr);
      return b.f64.slice(0, this.nFree);
    });
  }

  residuals(z: ArrayLike<number>): Float64Array {
    return withBuf(Math.max(this.nFree, 1), 8, (zb) =>
      withBuf(Math.max(this.nRes, 1), 8, (rb) => {
        zb.set(z);
        core().gcs_system_residuals(this.handle, zb.ptr, rb.ptr);
        return rb.f64.slice(0, this.nRes);
      }));
  }

  jacobianDense(z: ArrayLike<number>): { rows: number; cols: number; data: Float64Array } {
    const n = Math.max(this.nRes * this.nFree, 1);
    return withBuf(Math.max(this.nFree, 1), 8, (zb) => withBuf(n, 8, (jb) => {
      zb.set(z);
      core().gcs_system_jacobian_dense(this.handle, zb.ptr, jb.ptr);
      return { rows: this.nRes, cols: this.nFree, data: jb.f64.slice(0, this.nRes * this.nFree) };
    }));
  }

  /** The sparse Jacobian in CSR; the structure is fixed at compile time. */
  csr(z: ArrayLike<number>): { data: Float64Array; indices: Int32Array; indptr: Int32Array } {
    return withBuf(Math.max(this.nFree, 1), 8, (zb) =>
      withBuf(this.nRes + 1, 4, (ip) => withBuf(Math.max(this.nnz, 1), 4, (ix) =>
        withBuf(Math.max(this.nnz, 1), 8, (d) => {
          zb.set(z);
          core().gcs_system_csr_structure(this.handle, ip.ptr, ix.ptr);
          core().gcs_system_csr_data(this.handle, zb.ptr, d.ptr);
          return {
            data: d.f64.slice(0, this.nnz),
            indices: ix.i32.slice(0, this.nnz),
            indptr: ip.i32.slice(),
          };
        }))));
  }

  /** max |r| over hard rows at the current sketch values — what "solved" means. */
  maxHardResidual(): number {
    return core().gcs_system_max_hard_residual(this.handle, this.sketch.handle);
  }

  /** max |residual| per constraint, from one vectorized evaluation. */
  constraintErrors(): Map<Constraint, number> {
    const n = this.nConstraints;
    return withBuf(Math.max(n, 1), 4, (ids) => withBuf(Math.max(n, 1), 8, (vals) => {
      const m = core().gcs_system_constraint_errors(this.handle, this.sketch.handle,
                                                    ids.ptr, vals.ptr, n);
      const out = new Map<Constraint, number>();
      const ia = ids.i32, va = vals.f64;
      for (let i = 0; i < m; i++) {
        const c = this.sketch.constraintById(ia[i]);
        if (c) out.set(c, va[i]);
      }
      return out;
    }));
  }

  /** Numerical rank of the Jacobian — the workhorse of Stage 2/4 diagnosis. */
  rank(rcond = 1e-10, hardOnly = false): number {
    return core().gcs_system_rank(this.handle, this.sketch.handle, rcond, hardOnly ? 1 : 0);
  }

  /** Structural Jacobian as a bipartite graph: adj[row] = sorted free columns with a structural
   *  nonzero, plus row → owning constraint.  Soft rows are never part of it. */
  structure(): Structure {
    const d = takeJson<{ adj: number[][]; rowC: number[] }>(
      core().gcs_system_structure_json(this.handle));
    const rowC = d.rowC.map((id) => this.sketch.constraintById(id)!);
    return { adj: d.adj, rowC };
  }

  // -- solving --------------------------------------------------------------

  solve(opts: {
    method?: Method;
    tol?: number;          /* relative to extent² */
    maxNfev?: number;
    writeback?: boolean;
    maxIter?: number;
    dense?: boolean | null;
  } = {}): SolveResult {
    const method = opts.method ?? 'dogleg';
    const t0 = now();
    return withBuf(8, 8, (b) => {
      const msg = takeStr(core().gcs_system_solve(
        this.handle, this.sketch.handle, METHOD_ID[method], opts.tol ?? 1e-14,
        opts.maxIter ?? 100, opts.maxNfev ?? 0,
        opts.dense === undefined || opts.dense === null ? -1 : Number(opts.dense),
        opts.writeback === false ? 0 : 1, b.ptr));
      return readResult(b.f64, msg, method, t0);
    });
  }
}

/** One-shot: compile and solve, writing the result back into the sketch. */
export function solve(sketch: Sketch, opts: Parameters<System['solve']>[0] = {}): SolveResult {
  const method = opts.method ?? 'dogleg';
  const t0 = now();
  return withBuf(8, 8, (b) => {
    const msg = takeStr(core().gcs_solve(
      sketch.handle, METHOD_ID[method], opts.tol ?? 1e-14, opts.maxIter ?? 100,
      opts.maxNfev ?? 0,
      opts.dense === undefined || opts.dense === null ? -1 : Number(opts.dense), b.ptr));
    return readResult(b.f64, msg, method, t0);
  });
}

/** Twice the signed area of (a, b, c) — the order-type invariant the drag guards. */
export function orientation(a: Point, b: Point, c: Point): number {
  const [ax, ay] = a.xy, [bx, by] = b.xy, [cx, cy] = c.xy;
  return (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
}

/** Continuation path from (x0,y0) to (x1,y1): waypoints no farther apart than maxStep, so a
 *  solution tracks its branch instead of teleporting across it.  Always at least one point. */
export function increments(x0: number, y0: number, x1: number, y1: number,
                           maxStep: number): [number, number][] {
  const n = Math.max(1, Math.ceil(Math.hypot(x1 - x0, y1 - y0) / maxStep));
  const out: [number, number][] = [];
  for (let i = 1; i <= n; i++) out.push([x0 + ((x1 - x0) * i) / n, y0 + ((y1 - y0) * i) / n]);
  return out;
}

export function guardBuffer<T>(guards: Triangle[] | null | undefined,
                               fn: (ptr: number, n: number) => T): T {
  if (!guards || !guards.length) return fn(0, guards ? 0 : -1);
  return withBuf(3 * guards.length, 4, (b) => {
    const v = b.i32;
    guards.forEach((t, i) => {
      v[3 * i] = t[0].index;
      v[3 * i + 1] = t[1].index;
      v[3 * i + 2] = t[2].index;
    });
    return fn(b.ptr, guards.length);
  });
}

export function readFlips(sketch: Sketch, list: (d: number, out: number) => number,
                          handle: number, n: number): Triangle[] {
  if (n <= 0) return [];
  return withBuf(3 * n, 4, (b) => {
    list(handle, b.ptr);
    const v = b.i32;
    const pts = sketch.points;
    const out: Triangle[] = [];
    for (let i = 0; i < n; i++) out.push([pts[v[3 * i]], pts[v[3 * i + 1]], pts[v[3 * i + 2]]]);
    return out;
  });
}

/** Interactive drag of one point: pull toward the cursor, then polish.
 *
 *  Stage 5 robustness: continuation (a far cursor jump is taken in increments so the solution
 *  tracks its homotopy branch) and order-type guards (a step that would flip a guarded triangle's
 *  orientation is retried with smaller increments; an unavoidable flip is recorded and flagged). */
export class Drag {
  private handle: number;
  active = true;

  constructor(
    readonly sketch: Sketch,
    readonly point: Point,
    x: number,
    y: number,
    readonly method: Method = 'dogleg',
    weight = 1.0,
    readonly guards: Triangle[] = [],
    maxStepRel = 0.05,
  ) {
    this.handle = guardBuffer(guards, (ptr, n) => core().gcs_drag_new(
      sketch.handle, point.index, x, y, METHOD_ID[method], weight, ptr, Math.max(n, 0),
      maxStepRel));
    sketch.touch();
  }

  get flips(): Triangle[] {
    const c = core();
    return readFlips(this.sketch, (d, o) => c.gcs_drag_flip_list(d, o), this.handle,
                     c.gcs_drag_flips(this.handle));
  }

  move(x: number, y: number): SolveResult {
    const t0 = now();
    return withBuf(8, 8, (b) => {
      const msg = takeStr(core().gcs_drag_move(this.handle, this.sketch.handle, x, y, b.ptr));
      return readResult(b.f64, msg, this.method, t0);
    });
  }

  end(): void {
    if (!this.active) return;
    this.active = false;
    core().gcs_drag_end(this.handle, this.sketch.handle);
    this.sketch.touch();
    core().gcs_drag_free(this.handle);
    this.handle = 0;
  }
}

/** Interactive drag of a circle's or arc's radius — the scalar counterpart of `Drag`.
 *
 *  A radius that is fixed or dimensioned simply does not move: the polish wins, exactly as a point
 *  drag compromises on an over-constrained sketch.  An `EqualRadius` chain is a relation rather
 *  than a dimension, so the whole chain resizes together. */
export class RadiusDrag {
  private handle: number;
  active = true;

  constructor(readonly sketch: Sketch, readonly circle: Circle | Arc, r: number,
              readonly method: Method = 'dogleg') {
    this.handle = core().gcs_radius_drag_new(sketch.handle, KIND_ID[circle.kind], circle.index,
                                             r, METHOD_ID[method]);
    sketch.touch();
  }

  move(r: number): SolveResult {
    const t0 = now();
    return withBuf(8, 8, (b) => {
      const msg = takeStr(core().gcs_radius_drag_move(this.handle, this.sketch.handle, r, b.ptr));
      return readResult(b.f64, msg, this.method, t0);
    });
  }

  end(): void {
    if (!this.active) return;
    this.active = false;
    core().gcs_radius_drag_end(this.handle, this.sketch.handle);
    this.sketch.touch();
    core().gcs_radius_drag_free(this.handle);
    this.handle = 0;
  }
}
