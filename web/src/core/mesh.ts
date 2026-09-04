/* A solid's mesh: what a printer takes and what a viewer takes, which are the same thing.
 *
 * **Welded**, so every edge has its partner — a boundary evaluation cuts neighbouring facets by
 * different planes and leaves a vertex partway along an edge the other still spans whole, which
 * is a T-junction and which a strict validator refuses.  And **grouped by face**, so a viewer
 * shades a bore's wall as one round surface and selecting it names `body.bore.wall` rather than
 * a triangle.
 *
 * Numbers come across as flat buffers and the table of faces as JSON — the division
 * `gcs_entity_params` already draws: tens of thousands of numbers all of one kind go in a
 * buffer, and the ragged small thing goes in JSON.  All three calls read one memoised answer. */
import { Sketch } from './model.js';
import { core, takeJson, withBuf } from './wasm.js';

/** One face of the object, as a run of triangles into the mesh's buffers. */
export interface Face {
  /** `body.bore.wall` — what the document calls it. */
  path: string;
  /** The first triangle and how many: multiply by 9 for an index into `positions`. */
  start: number;
  count: number;
  /** A tessellation of something curved: shade it smooth.  Absent for a flat face, which wants
   *  flat shading so its corners stay corners. */
  smooth?: boolean;
}

export interface Mesh {
  /** Nine doubles a triangle: three vertices, in the order `faces` names. */
  positions: Float64Array;
  /** Nine more, a normal per vertex. */
  normals: Float64Array;
  faces: Face[];
}

/** The mesh of solid `idx`, refined to `unit` — the world length of one screen pixel, or
 *  `REPORT_UNIT` for a file that should not depend on the zoom. */
export function mesh(sk: Sketch, idx: number, unit: number): Mesh {
  const faces = takeJson<Face[]>(core().gcs_solid_faces_json(sk.handle, idx, unit));
  const n = faces.reduce((a, f) => a + f.count, 0) * 9;
  const positions = readInto(n, (p, cap) => core().gcs_solid_mesh(sk.handle, idx, unit, p, cap));
  const normals = readInto(n, (p, cap) => core().gcs_solid_normals(sk.handle, idx, unit, p, cap));
  return { positions, normals, faces };
}

/** A flat `f64` buffer out of the core, sized by what the face table already said — so the
 *  buffer is never guessed at and the "too small" return can only mean the drawing moved under
 *  us, which a copy out of the core's heap would not survive anyway. */
function readInto(n: number, fill: (ptr: number, cap: number) => number): Float64Array {
  if (n === 0) return new Float64Array(0);
  return withBuf(n, 8, (b) => {
    const got = fill(b.ptr, n);
    if (got < 0) throw new Error('the mesh does not fit the buffer it was given');
    return new Float64Array(b.f64.subarray(0, got));
  });
}
