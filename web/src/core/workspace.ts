/* The workspace: every sketch standing on its own plane, seen by one orthographic eye.
 *
 * **The core projects and the front end composes.**  Each view's page coordinates reach the eye's
 * picture plane through one affine map (`gcs-core/src/overview/workspace.rs`), so the app draws a
 * sketch on a tilted plane by composing that map with its own 2D camera, and turns a click back
 * into a place on a plane by inverting a 2×2 — `app/camera.ts`'s whole job.  Every question about
 * what is under the pointer is asked here, of the figures where the eye sees them: views that lie
 * on top of one another on the page are nowhere near one another in space. */
import { KINDS, Plane, Point, Primitive, Sketch, fromWire, kindOf } from './model.js';
import type { Kind, Way } from './model.js';
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

/** The eye at bearing `az` and elevation `el` (radians): its picture plane's `right` and `up`,
 *  and `toward` the viewer — the core's one basis (`workspace::eye_frame`), which a camera of the
 *  front end's own is set from so that it sees what the picking sees. */
export interface Eye {
  right: [number, number, number];
  up: [number, number, number];
  toward: [number, number, number];
}

/** The eye at bearing `az` and elevation `el` (radians). */
export function eyeFrame(az: number, el: number): Eye {
  return withBuf(9, 8, (b) => {
    core().gcs_workspace_eye(az, el, b.ptr);
    const v = (k: number) => Array.from(b.f64.subarray(3 * k, 3 * k + 3)) as
      [number, number, number];
    return { right: v(0), up: v(1), toward: v(2) };
  });
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
    const kind = kindOf(b.f64[0]);
    return kind ? sk.entities(kind)[b.f64[1]] ?? null : null;
  });
}

/** A face of an object the eye's ray meets: the solid (an index into the sketch's solids), the
 *  face's path where it was made — an operand's, perhaps (`bore.wall` on `body`) — and how far
 *  toward the viewer it stands. */
export interface SolidHit {
  solid: number;
  face: string;
  depth: number;
  /** Where in space the ray met the face. */
  at: [number, number, number];
}

/** The object face the eye's ray through `(x, y)` on the picture plane meets nearest the viewer
 *  — what a click on a solid picks — or null. */
export function pickSolidSeen(sk: Sketch, az: number, el: number,
                              x: number, y: number): SolidHit | null {
  return takeJson<SolidHit | null>(core().gcs_workspace_pick_solid_json(sk.handle, az, el, x, y));
}

/** An edge of an object the eye sees under the pointer: the object, the two faces meeting there
 *  (by the paths its mesh names them by, in order) and the place on it nearest the pointer. */
export interface EdgeHit {
  solid: number;
  faces: [string, string];
  at: [number, number, number];
}

/** The edge of an object nearest `(x, y)` on the picture plane, within `tol` there, that the eye
 *  sees — an edge behind the material is not picked through it — or null. */
export function pickEdgeSeen(sk: Sketch, az: number, el: number, x: number, y: number,
                             tol: number): EdgeHit | null {
  return takeJson<EdgeHit | null>(
    core().gcs_workspace_pick_edge_json(sk.handle, az, el, x, y, tol));
}

/** One thing the eye's ray reaches under the pointer: something drawn (a figure, a point, an
 *  axis, a pane's frame), an edge of an object — seen, or behind its material — a face it
 *  crosses, or an object whole. */
export type Under =
  | { kind: 'drawn'; ent: Primitive }
  | { kind: 'edge'; solid: number; faces: [string, string]; at: [number, number, number];
      seen: boolean }
  | { kind: 'face'; solid: number; face: string; at: [number, number, number] }
  | { kind: 'object'; solid: number };

/** **Everything under the pointer** at `(x, y)` on the picture plane, within `tol` there for what
 *  is drawn and for edges: the drawing first, then the objects' edges and faces nearest the eye
 *  first, then the objects (`workspace::under`) — what a click there might have meant. */
export function underSeen(sk: Sketch, unit: number, az: number, el: number, x: number, y: number,
                          tol: number): Under[] {
  type Wire = Exclude<Under, { kind: 'drawn' }> | { kind: 'drawn'; id: number; index: number };
  const wire = takeJson<Wire[]>(
    core().gcs_workspace_under_json(sk.handle, unit, az, el, x, y, tol));
  return wire.flatMap((u): Under[] => {
    if (u.kind !== 'drawn') return [u];
    const kind = kindOf(u.id);
    const ent = kind ? sk.entities(kind)[u.index] : undefined;
    return ent ? [{ kind: 'drawn', ent }] : [];
  });
}

/** Where two faces of an object meet: the faces, and the stretches of it in space, each six
 *  numbers (from, to). */
export interface SolidEdge {
  faces: [string, string];
  segs: number[][];
}

/** Object `solid`'s edges. */
export function solidEdges(sk: Sketch, solid: number): SolidEdge[] {
  return takeJson<SolidEdge[] | null>(core().gcs_workspace_solid_edges_json(sk.handle, solid)) ?? [];
}

/** The size a round or chamfer on object `solid` starts from, or null. */
export function roundSize(sk: Sketch, solid: number): number | null {
  const size = core().gcs_workspace_round_size(sk.handle, solid);
  return Number.isFinite(size) ? size : null;
}

/** The arrow a prism is sized by, as the eye sees it, and what it says: from its face's middle to
 *  its far end along the face's normal — no tip where the eye looks down the normal — which way
 *  it runs (null for extents none of the three ways say), and how thick it is. */
export interface ExtrudeHandle {
  base: [number, number];
  tip: [number, number] | null;
  way: Way | null;
  thick: number;
}

/** The arrow prism `solid` is sized by, on the eye's picture plane — or null for no prism. */
export function extrudeHandle(sk: Sketch, az: number, el: number,
                              solid: number): ExtrudeHandle | null {
  return takeJson<ExtrudeHandle | null>(
    core().gcs_workspace_extrude_handle_json(sk.handle, az, el, solid));
}

/** How far along prism `solid`'s normal the point under `(x, y)` on the eye's picture plane is —
 *  what dragging its arrow there sizes it to — or NaN. */
export function extentAt(sk: Sketch, az: number, el: number, solid: number,
                         x: number, y: number): number {
  return core().gcs_workspace_extent_at(sk.handle, az, el, solid, x, y);
}

/** The handle a revolution is sized by, as the eye sees it: its profile's middle, the arc that
 *  middle turns through (a whole circle for a whole turn), its end — none where the eye looks
 *  along the turning plane — and how far it turns, in degrees, positive right-handed about the
 *  axis. */
export interface TurnHandle {
  base: [number, number];
  arc: [number, number][];
  tip: [number, number] | null;
  turn: number;
}

/** The handle revolution `solid` is sized by, on the eye's picture plane — or null for none. */
export function turnHandle(sk: Sketch, az: number, el: number, solid: number): TurnHandle | null {
  return takeJson<TurnHandle | null>(core().gcs_workspace_turn_handle_json(sk.handle, az, el, solid));
}

/** How far revolution `solid` turns where its handle is dragged to `(x, y)` on the eye's picture
 *  plane, in signed degrees, the whole turns round nearest `near` — or NaN. */
export function turnAt(sk: Sketch, az: number, el: number, solid: number,
                       x: number, y: number, near: number): number {
  return core().gcs_workspace_turn_at(sk.handle, az, el, solid, x, y, near);
}

/** The arrow an offset plane is sized by, as the eye sees it: from the plane it is set off along
 *  that plane's normal by the distance — no tip where the eye looks down the normal. */
export interface OffsetHandle {
  base: [number, number];
  tip: [number, number] | null;
  distance: number;
}

/** The arrow plane distance `id` (a constraint id) is sized by — or null for any other. */
export function offsetHandle(sk: Sketch, az: number, el: number, id: number): OffsetHandle | null {
  return takeJson<OffsetHandle | null>(core().gcs_workspace_offset_handle_json(sk.handle, az, el, id));
}

/** What dragging plane distance `id`'s arrow to `(x, y)` sizes it to, or NaN. */
export function offsetAt(sk: Sketch, az: number, el: number, id: number,
                         x: number, y: number): number {
  return core().gcs_workspace_offset_at(sk.handle, az, el, id, x, y);
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

/** The entities a rubber band between two places on the eye's picture plane touches. */
export function overlappingSeen(sk: Sketch, unit: number, az: number, el: number,
                                a: [number, number], b: [number, number]): Primitive[] {
  const hits = takeJson<[string, number][]>(
    core().gcs_workspace_overlapping_json(sk.handle, unit, az, el, a[0], a[1], b[0], b[1]));
  return hits
    .map(([wire, index]) => {
      const kind = fromWire(wire);
      return (KINDS as string[]).includes(kind) ? sk.entities(kind as Kind)[index] : undefined;
    })
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
