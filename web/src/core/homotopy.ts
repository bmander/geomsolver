/* Stage 5 — homotopy continuation to enumerate the real solutions of one small merge.
 *
 * "We can show you the other solutions": the tracking runs in the core; this module presents the
 * alternatives and applies one. */
import { Point } from './model.js';
import { PlanSolver } from './decompose.js';
import { core, takeJson, withJson } from './wasm.js';

export interface Alternative {
  /** Transform (theta, tx, ty) per movable cluster, relative to the current leaves. */
  u: number[];
  /** ‖w − w_identity‖: 0 for the root the sketch is on. */
  distance: number;
  /** Where a requested point element would land. */
  location: [number, number] | null;
  isCurrent: boolean;
}

export const isCurrent = (a: Alternative): boolean => a.isCurrent;

export interface EnumerateOptions {
  locate?: Point | null;
  seed?: number;
  maxPaths?: number;
}

/** Real solutions of the merge at `stepIndex` (the current one first).  Empty if the merge is not
 *  isolated (under-determined) or too large. */
export function enumerateStep(solver: PlanSolver, stepIndex: number,
                              opts: EnumerateOptions = {}): Alternative[] {
  return takeJson<Alternative[]>(core().gcs_enumerate_step_json(
    (solver as unknown as { handle: number }).handle, solver.sketch.handle, stepIndex,
    opts.locate ? opts.locate.index : -1, (opts.seed ?? 0) >>> 0, opts.maxPaths ?? 256)) ?? [];
}

/** Put the sketch on this root, then replay the whole plan so dependent geometry follows. */
export function applyAlternative(solver: PlanSolver, stepIndex: number, alt: Alternative): void {
  withJson({ u: alt.u, distance: alt.distance }, (p, n) => core().gcs_apply_alternative(
    (solver as unknown as { handle: number }).handle, solver.sketch.handle, stepIndex, p, n));
}
