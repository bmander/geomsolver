/* Stage 3 — cluster merging (Fudos–Hoffmann, generalised) → plan → replay.
 *
 * Decomposition and replay run in the core.  `PlanSolver` compiles once per topology and replays
 * the plan on every solve; `PlanDrag` is the DCM-style drag that never re-analyses the graph. */
import { Constraint } from './constraints.js';
import { Point, Sketch } from './model.js';
import {
  Drag, Method, SolveResult, System, Triangle, guardBuffer, now, readFlips,
} from './system.js';
import { core, takeJson, takeStr, withBuf } from './wasm.js';

const METHOD_ID: Record<Method, number> = { dogleg: 0, lm: 1 };

/** A geometric element of the F–H graph: a point class, a line, or a virtual radius line. */
export type El = ['P' | 'L' | 'V', number];

/** One merge, lowered for replay.  `ppp` is the (x, y, z) closed-form construction and `branch`
 *  its ±1 chirality; `key` is the document-stable identity that persists. */
export interface Step {
  ids: number[];
  ppp: [El, El, El] | null;
  branch: number | null;
  key: string | null;
  nPairs: number;
  nDpairs: number;
}

export interface Plan {
  leaves: number;
  steps: Step[];
  roots: number[];
  fullyDecomposed: boolean;
  stickyBranches: boolean;
  summary: string;
  pppTriangles: [number, number, number][];
}

export interface Graph {
  nPoints: number;
  members: number[][];
  lines: number[];
  virtuals: [El, El][];
  edges: { kind: 'PP' | 'PL'; a: El; b: El; source: number | null }[];
  dirs: { a: El; b: El; phi: number; source: number }[];
  unsupported: number[];
  knownRadius: Record<string, number>;
  groundPoints: number[];
  passive: number[];
  summary: string;
}

export interface PlanResult {
  success: boolean;
  maxResidual: number;
  fellBack: boolean;
  timeS: number;
  nSteps: number;
  numeric: SolveResult | null;
  plan: Plan;
}

/** The same outcome in the solver's common result type (method `plan` or the fallback's). */
export function asSolveResult(pr: PlanResult): SolveResult {
  if (pr.numeric) return pr.numeric;
  return {
    success: pr.success, status: 0, message: 'plan',
    residualNorm: pr.maxResidual, maxResidual: pr.maxResidual,
    nfev: pr.nSteps, njev: 0, timeS: pr.timeS, method: 'plan', iterations: 0, rank: null,
  };
}

/** The F–H constraint graph of a sketch (elements, edges, direction relations). */
export function buildGraph(sketch: Sketch): Graph {
  return takeJson<Graph>(core().gcs_graph_json(sketch.handle));
}

/** Compile once per topology (graph + decomposition + a System for verification); `solve` replays
 *  the plan and falls back to the numeric core when the residual says the plan did not (fully)
 *  determine the sketch. */
export class PlanSolver {
  private handle: number;
  /** Owned by this PlanSolver — borrow it (diagnosis does), never dispose it. */
  readonly system: System;

  constructor(readonly sketch: Sketch, sticky = false) {
    this.handle = core().gcs_plan_solver_new(sketch.handle, sticky ? 1 : 0);
    this.system = new System(sketch, core().gcs_plan_solver_system(this.handle), false);
  }

  dispose(): void {
    if (!this.handle) return;
    this.system.dispose();
    core().gcs_plan_solver_free(this.handle);
    this.handle = 0;
  }

  get plan(): Plan {
    return takeJson<Plan>(core().gcs_plan_solver_plan_json(this.handle));
  }

  get graph(): Graph {
    return takeJson<Graph>(core().gcs_plan_solver_graph_json(this.handle));
  }

  set stickyBranches(v: boolean) {
    core().gcs_plan_solver_sticky(this.handle, v ? 1 : 0);
  }

  /** The point element (coincidence class) a sketch point belongs to. */
  pointElement(p: Point): number {
    return core().gcs_plan_solver_point_element(this.handle, p.index);
  }

  /** Flip the root of every closed-form construction that places `p`; returns how many.  Root
   *  choices are document state — `solve` reads them back. */
  flip(p: Point): number {
    return core().gcs_plan_solver_flip(this.handle, this.sketch.handle, p.index);
  }

  /** Replay the plan on the current geometry (no solving, no fallback). */
  execute(): void {
    core().gcs_plan_solver_execute(this.handle, this.sketch.handle);
  }

  branches(): Map<string, number> {
    const out = new Map<string, number>();
    for (const st of this.plan.steps) {
      if (st.key !== null && st.branch !== null) out.set(st.key, st.branch);
    }
    return out;
  }

  /** The closed-form merges' triangles — the order-type invariants a numeric drag guards. */
  pppTriangles(): Triangle[] {
    const pts = this.sketch.points;
    return this.plan.pppTriangles.map(([a, b, c]) => [pts[a], pts[b], pts[c]] as Triangle);
  }

  /** (index, step) of every merge that places `p`: closed-form ones where it is the constructed
   *  apex, else numeric merges that share it. */
  stepsPlacing(p: Point): [number, Step][] {
    const steps = this.plan.steps;
    const idxs = withBuf(Math.max(steps.length, 1), 4, (b) => {
      const n = core().gcs_plan_steps_placing(this.handle, p.index, b.ptr);
      return [...b.i32.subarray(0, n)];
    });
    return idxs.map((i) => [i, steps[i]] as [number, Step]);
  }

  solve(tol = 1e-9, fallback = true, method: Method = 'dogleg'): PlanResult {
    const t0 = now();
    const d = takeJson<{ success: boolean; maxResidual: number; fellBack: boolean;
                         nSteps: number; numeric: SolveResult | null }>(
      core().gcs_plan_solver_solve(this.handle, this.sketch.handle, tol, fallback ? 1 : 0,
                                   METHOD_ID[method]));
    return { ...d, timeS: now() - t0, plan: this.plan };
  }
}

/** The closed-form triangles of a freshly decomposed sketch. */
export function pppTriangles(solver: PlanSolver): Triangle[] {
  return solver.pppTriangles();
}

/** DCM-style drag: the dragged point joins the ground and the cached plan replays per frame — no
 *  graph analysis while dragging, recorded roots are sticky, and under-constrained roots move
 *  least.  If the plan cannot determine the sketch with the point pinned (fully constrained
 *  sketches, unsupported constraints) the numeric pull/polish `Drag` takes over. */
export class PlanDrag {
  private handle: number;
  active = true;

  constructor(readonly sketch: Sketch, readonly point: Point, x: number, y: number,
              guards: Triangle[] | null = null, maxStepRel = 0.05) {
    this.handle = guardBuffer(guards, (ptr, n) =>
      core().gcs_plan_drag_new(sketch.handle, point.index, x, y, ptr, n, maxStepRel));
    sketch.touch();
  }

  /** True while the cached plan is driving the drag (false once it handed over). */
  get usable(): boolean {
    return core().gcs_plan_drag_usable(this.handle) !== 0;
  }

  /** Truthy once the numeric drag has taken over. */
  get numeric(): boolean {
    return !this.usable;
  }

  get flips(): Triangle[] {
    const c = core();
    return readFlips(this.sketch, (d, o) => c.gcs_plan_drag_flip_list(d, o), this.handle,
                     c.gcs_plan_drag_flips(this.handle));
  }

  /** Order-type invariants for the numeric path — computed at most once per drag. */
  guardTriangles(): Triangle[] {
    return withBuf(3 * 4096, 4, (b) => {
      const n = core().gcs_plan_drag_guards(this.handle, this.sketch.handle, b.ptr);
      const v = b.i32;
      const pts = this.sketch.points;
      const out: Triangle[] = [];
      for (let i = 0; i < n; i++) out.push([pts[v[3 * i]], pts[v[3 * i + 1]], pts[v[3 * i + 2]]]);
      return out;
    });
  }

  branches(): Map<string, number> {
    const o = takeJson<Record<string, number>>(core().gcs_plan_drag_branches_json(this.handle));
    return new Map(Object.entries(o ?? {}));
  }

  move(x: number, y: number): SolveResult {
    const t0 = now();
    return withBuf(8, 8, (b) => {
      const msg = takeStr(core().gcs_plan_drag_move(this.handle, this.sketch.handle, x, y, b.ptr));
      const v = b.f64;
      return {
        success: v[0] !== 0, status: v[1], message: msg,
        residualNorm: v[2], maxResidual: v[3], nfev: v[4], njev: v[5],
        iterations: v[6], rank: v[7] < 0 ? null : v[7], timeS: now() - t0, method: 'plan',
      };
    });
  }

  end(): void {
    if (!this.active) return;
    this.active = false;
    core().gcs_plan_drag_end(this.handle, this.sketch.handle);
    this.sketch.touch();
    core().gcs_plan_drag_free(this.handle);
    this.handle = 0;
  }
}

export { Drag, System };
export type { Constraint };
