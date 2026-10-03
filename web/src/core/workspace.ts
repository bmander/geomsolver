/* The workspace: every sketch standing on its own plane, seen by one orthographic eye.
 *
 * **The core projects and the front end composes.**  Each view's page coordinates reach the eye's
 * picture plane through one affine map (`gcs-core/src/overview/workspace.rs`), so the app draws a
 * sketch on a tilted plane by composing that map with its own 2D camera, and turns a click back
 * into a place on a plane by inverting a 2×2 — `app/camera.ts`'s whole job.  Every question about
 * what is under the pointer is asked here, of the figures where the eye sees them: views that lie
 * on top of one another on the page are nowhere near one another in space. */
import { KINDS, Point, Primitive, Sketch } from './model.js';
import type { Kind } from './model.js';
import { core, takeJson, withBuf } from './wasm.js';

/** A view's page onto the eye's picture plane: `(x, y) ↦ (m0·x + m1·y + m2, m3·x + m4·y + m5)`. */
export type Map = [number, number, number, number, number, number];

/** -1 the page, a plane's index, -2 none (the entity straddles views that are apart in space). */
export type View = number;

export const PAGE: View = -1;
export const NOWHERE: View = -2;

export interface Workspace {
  /** The page's map first, then every plane's in order: `map(view)` reads it. */
  maps: Map[];
  /** The eye square on to each view, `[az, el]` in radians, in the same order. */
  looks: [number, number][];
  /** The view every point and every drawn entity of each kind stands in, by index. */
  views: {
    point: View[]; line: View[]; circle: View[]; arc: View[]; spline: View[]; curve: View[];
  };
}

/** The workspace from the eye at bearing `az` and elevation `el` (radians). */
export function workspace(sk: Sketch, az: number, el: number): Workspace {
  return takeJson<Workspace>(core().gcs_workspace_json(sk.handle, az, el));
}

/** A view's map out of a workspace report. */
export function mapOf(ws: Workspace, view: View): Map | null {
  return view === NOWHERE ? null : ws.maps[view + 1] ?? null;
}

/** What a click at `(x, y)` on the eye's picture plane picks within `tol` (an eye length);
 *  `unit` is the eye length of one screen pixel. */
export function pickSeen(sk: Sketch, unit: number, az: number, el: number,
                         x: number, y: number, tol: number): Primitive | null {
  return withBuf(2, 8, (b) => {
    if (!core().gcs_workspace_pick(sk.handle, unit, az, el, x, y, tol, b.ptr)) return null;
    return sk.entities(KINDS[b.f64[0]])[b.f64[1]] ?? null;
  });
}

/** The point nearest `(x, y)` on the eye's picture plane, and how far it is. */
export function nearestSeen(sk: Sketch, az: number, el: number,
                            x: number, y: number): { point: Point | null; dist: number } {
  return withBuf(1, 8, (b) => {
    const i = core().gcs_workspace_nearest_point(sk.handle, az, el, x, y, b.ptr);
    return { point: i >= 0 ? sk.points[i] : null, dist: b.f64[0] };
  });
}

/** The entities a rubber band between two places on the eye's picture plane holds whole. */
export function insideSeen(sk: Sketch, unit: number, az: number, el: number,
                           a: [number, number], b: [number, number]): Primitive[] {
  const hits = takeJson<{ kind: string; index: number }[]>(
    core().gcs_workspace_inside_json(sk.handle, unit, az, el, a[0], a[1], b[0], b[1]));
  return hits
    .map((h) => (KINDS as string[]).includes(h.kind)
      ? sk.entities(h.kind as Kind)[h.index] : undefined)
    .filter((e): e is Primitive => !!e);
}

/** The dimension whose callout `(x, y)` on the eye's picture plane lands on, or -1. */
export function calloutSeen(sk: Sketch, unit: number, az: number, el: number,
                            x: number, y: number, tolPx: number): number {
  return core().gcs_workspace_callout_pick(sk.handle, unit, az, el, x, y, tolPx);
}

/** The extent of everything shown, on the eye's picture plane — figures and solids — as
 *  `[xmin, ymin, xmax, ymax]`, or null when nothing is drawn. */
export function boundsSeen(sk: Sketch, unit: number, az: number,
                           el: number): [number, number, number, number] | null {
  return withBuf(4, 8, (b) => core().gcs_workspace_bounds(sk.handle, unit, az, el, b.ptr)
    ? [b.f64[0], b.f64[1], b.f64[2], b.f64[3]] : null);
}
