/* **A section**: the workspace cut by a plane, the side its normal points to taken away.
 *
 * The core owns all of it (`Sketch::cut`, `EvaluatedSolid::section`): which side of the plane a
 * point is on, where the plane meets each object's surface and the face the cut leaves.  The cut
 * is view state on the sketch, as what the outline hides is, so the core's picks pass by what it
 * took away; a host sets it on every drawing it is handed.  What crosses here is a plane's frame
 * going in, and coming out the plane a renderer clips by and each object's cut in its own
 * coordinates, placed by the matrix that places the object. */
import { Sketch } from './model.js';
import { core, lastError, withBuf } from './wasm.js';

type Vec3 = [number, number, number];

/** A plane's frame in the world, as `Plane.frame3` reads it. */
export interface Frame3 {
  u: Vec3;
  v: Vec3;
  o: Vec3;
}

/** What a section is: the plane it cuts by, kept the other way where `flip`, moved `offset` (a
 *  length in the drawing's unit) along the plane's own normal. */
export interface Cut {
  frame: Frame3;
  flip: boolean;
  offset: number;
}

/** Section the workspace by `cut`, or by nothing.  Throws for a frame that spans no plane. */
export function setCut(sk: Sketch, cut: Cut | null): void {
  if (!cut) {
    core().gcs_sketch_set_cut(sk.handle, 0, 0, 0);
    return;
  }
  const { u, v, o } = cut.frame;
  withBuf(9, 8, (b) => {
    b.f64.set([...u, ...v, ...o]);
    if (core().gcs_sketch_set_cut(sk.handle, b.ptr, cut.flip ? 1 : 0, cut.offset) < 0) {
      throw new Error(lastError());
    }
  });
}

/** The section as a renderer clips by — a unit normal toward what is kept and a constant, three's
 *  `Plane` — or null where the workspace is not sectioned. */
export function clipPlane(sk: Sketch): [number, number, number, number] | null {
  return withBuf(4, 8, (b) => {
    if (core().gcs_sketch_cut(sk.handle, b.ptr) !== 4) return null;
    const x = b.f64;
    return [x[0], x[1], x[2], x[3]];
  });
}

/** How far the objects shown reach along `frame`'s normal from it, least and most: the range a
 *  section's offset sweeps them over.  Null where no object evaluates. */
export function reachAlong(sk: Sketch, frame: Frame3): [number, number] | null {
  // the frame in, and the reach out after it
  return withBuf(11, 8, (b) => {
    b.f64.set([...frame.u, ...frame.v, ...frame.o]);
    if (core().gcs_workspace_reach_along(sk.handle, b.ptr, b.ptr + 9 * 8) !== 2) return null;
    return [b.f64[9], b.f64[10]];
  });
}

/** One object cut, in its own coordinates (`own`'s): the face the cut leaves, nine doubles a
 *  triangle, and where the cut meets its surface, six doubles a segment. */
export interface Sectioned {
  face: Float64Array;
  outline: Float64Array;
}

/** Object `idx` cut by the workspace's section, refined to `unit` as `own` is: null where the
 *  workspace is not sectioned.  Throws with the core's reason where it does not evaluate. */
export function sectioned(sk: Sketch, idx: number, unit: number): Sectioned | null {
  const n = core().gcs_solid_section(sk.handle, idx, unit, 0, 0);
  if (n < 0) throw new Error(lastError());
  if (n === 0) return null;
  return withBuf(n, 8, (b) => {
    if (core().gcs_solid_section(sk.handle, idx, unit, b.ptr, n) !== n) {
      throw new Error('the section does not fit the buffer it was given');
    }
    const all = Float64Array.from(b.f64.subarray(0, n));
    const faces = all[0];
    return { face: all.subarray(1, 1 + faces), outline: all.subarray(1 + faces) };
  });
}
