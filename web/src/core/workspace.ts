/* The workspace: every sketch standing on its own plane, seen by one orthographic eye.
 *
 * **The core projects and the front end composes.**  Each view's page coordinates reach the eye's
 * picture plane through one affine map (`gcs-core/src/overview/workspace.rs`), so the app draws a
 * sketch on a tilted plane by composing that map with its own 2D camera, and turns a click back
 * into a place on a plane by inverting a 2×2 — `app/camera.ts`'s whole job.  Every question about
 * what is under the pointer is asked here, of the figures where the eye sees them: views that lie
 * on top of one another on the page are nowhere near one another in space. */
import { KINDS, Plane, Point, Primitive, Sketch } from './model.js';
import type { Kind } from './model.js';
import { core, takeJson, withBuf } from './wasm.js';

/** A view's page onto the eye's picture plane: `(x, y) ↦ (m0·x + m1·y + m2, m3·x + m4·y + m5)`. */
export type Map = [number, number, number, number, number, number];

/** -1 the page, a plane's index, -2 none (the entity straddles views that are apart in space). */
export type View = number;

export const PAGE: View = -1;
export const NOWHERE: View = -2;

/** What of the workspace does not depend on the eye, so is asked once an edit.  Every table over
 *  views is the page first, then each plane in order (`slot`). */
export interface Workspace {
  /** Each view's place: the first view standing on the same plane in space.  Two views are one
   *  place — the page and `std.front`, or `std.up` turned on it — exactly when these agree. */
  places: View[];
  /** The eye square on to each view, `[az, el]` in radians. */
  looks: [number, number][];
  /** The view every point and every drawn entity of each kind stands in, by index. */
  views: {
    point: View[]; line: View[]; circle: View[]; arc: View[]; spline: View[]; curve: View[];
  };
}

/** Where a view sits in a table over views. */
function slot(view: View): number {
  return view + 1;
}

/** The workspace's eye-free half. */
export function workspace(sk: Sketch): Workspace {
  return takeJson<Workspace>(core().gcs_workspace_json(sk.handle));
}

/** A view's place, or `NOWHERE` for a view that is none. */
export function placeOf(ws: Workspace, view: View): View {
  return view === NOWHERE ? NOWHERE : ws.places[slot(view)] ?? NOWHERE;
}

/** The eye square on to a view. */
export function lookOf(ws: Workspace, view: View): { az: number; el: number } | null {
  const l = view === NOWHERE ? undefined : ws.looks[slot(view)];
  return l ? { az: l[0], el: l[1] } : null;
}

/** Every view's map as the eye at bearing `az` and elevation `el` (radians) sees it, the page
 *  first — asked per frame, so through a buffer and not JSON. */
export function maps(sk: Sketch, az: number, el: number): Map[] {
  const cap = sk.planes.length + 1;
  return withBuf(6 * cap, 8, (b) => {
    const n = Math.min(core().gcs_workspace_maps(sk.handle, az, el, b.ptr, cap), cap);
    return Array.from({ length: Math.max(n, 0) },
                      (_, k) => Array.from(b.f64.subarray(6 * k, 6 * k + 6)) as Map);
  });
}

/** Where the eye at bearing `az` and elevation `el` sees each point in space, by point index, on
 *  its picture plane; `null` for a point drawn in a plane, which its view's map places.  Asked per
 *  frame beside `maps`. */
export function spacePoints(sk: Sketch, az: number, el: number): ([number, number] | null)[] {
  const cap = sk.points.length;
  if (cap === 0) return [];
  return withBuf(2 * cap, 8, (b) => {
    const n = Math.min(core().gcs_workspace_space_points(sk.handle, az, el, b.ptr, cap), cap);
    return Array.from({ length: Math.max(n, 0) }, (_, k) => {
      const [x, y] = [b.f64[2 * k], b.f64[2 * k + 1]];
      return Number.isNaN(x) ? null : [x, y] as [number, number];
    });
  });
}

/** A view's entry in a table over views — `maps`, or anything built from it. */
export function ofView<T>(all: readonly T[], view: View): T | null {
  return view === NOWHERE ? null : all[slot(view)] ?? null;
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

/** A face of an object the eye's ray meets: the solid (an index into the sketch's solids), the
 *  face's path where it was made — an operand's, perhaps (`bore.wall` on `body`) — and how far
 *  toward the viewer it stands. */
export interface SolidHit {
  solid: number;
  face: string;
  depth: number;
}

/** The object face the eye's ray through `(x, y)` on the picture plane meets nearest the viewer
 *  — what a click on a solid picks — or null. */
export function pickSolidSeen(sk: Sketch, az: number, el: number,
                              x: number, y: number): SolidHit | null {
  return takeJson<SolidHit | null>(core().gcs_workspace_pick_solid_json(sk.handle, az, el, x, y));
}

/** The planes whose panes `(x, y)` on the eye's picture plane falls inside, nearest the eye
 *  first. */
export function panesSeen(sk: Sketch, az: number, el: number, x: number, y: number): Plane[] {
  const cap = sk.planes.length;
  if (cap === 0) return [];
  return withBuf(cap, 8, (b) => {
    const n = Math.min(core().gcs_workspace_panes_at(sk.handle, az, el, x, y, b.ptr, cap), cap);
    return Array.from(b.f64.subarray(0, Math.max(n, 0)), (i) => sk.planes[i]);
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
  const hits = takeJson<[string, number][]>(
    core().gcs_workspace_inside_json(sk.handle, unit, az, el, a[0], a[1], b[0], b[1]));
  return hits
    .map(([kind, index]) => (KINDS as string[]).includes(kind)
      ? sk.entities(kind as Kind)[index] : undefined)
    .filter((e): e is Primitive => !!e);
}

/** The dimension whose callout `(x, y)` on the eye's picture plane lands on, and the view its
 *  figure is laid out in — or null. */
export function calloutSeen(sk: Sketch, unit: number, az: number, el: number,
                            x: number, y: number, tolPx: number): { id: number; view: View } | null {
  return withBuf(1, 4, (b) => {
    const id = core().gcs_workspace_callout_pick(sk.handle, unit, az, el, x, y, tolPx, b.ptr);
    return id < 0 ? null : { id, view: b.i32[0] };
  });
}

/** The extent of everything shown, on the eye's picture plane — figures and solids — as
 *  `[xmin, ymin, xmax, ymax]`, or null when nothing is drawn. */
export function boundsSeen(sk: Sketch, unit: number, az: number,
                           el: number): [number, number, number, number] | null {
  return withBuf(4, 8, (b) => core().gcs_workspace_bounds(sk.handle, unit, az, el, b.ptr)
    ? [b.f64[0], b.f64[1], b.f64[2], b.f64[3]] : null);
}
