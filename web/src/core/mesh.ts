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
import type { BodyWord } from './program.js';
import { core, lastError, takeBytes, takeJson, withBuf, withJson } from './wasm.js';

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
 *  `REPORT_UNIT` for a file that should not depend on the zoom.  Throws with the core's reason
 *  where the solid does not evaluate (a revolution whose axis crosses its profile). */
export function mesh(sk: Sketch, idx: number, unit: number): Mesh {
  const faces = takeJson<Face[] | null>(core().gcs_solid_faces_json(sk.handle, idx, unit));
  if (!faces) throw new Error(lastError());
  const n = faces.reduce((a, f) => a + f.count, 0) * 9;
  const positions = readInto(n, (p, cap) => core().gcs_solid_mesh(sk.handle, idx, unit, p, cap));
  const normals = readInto(n, (p, cap) => core().gcs_solid_normals(sk.handle, idx, unit, p, cap));
  return { positions, normals, faces };
}

/** **A solid in its own coordinates, as a renderer draws it** (#412, #674): its mesh over shared
 *  vertices — a vertex once a face, where the triangles of a face meet there, so a flat face keeps
 *  its corners and a round one shades smooth — and the edges a CAD viewer inks, before its
 *  placement stands it in the world.  A solid that has only moved since it was evaluated is the
 *  same evaluation, and the solids of one binding are one geometry (`Placement.geometry`), so a
 *  host builds one geometry of it and moves it by `placement` alone. */
export interface Drawn {
  /** Three a vertex. */
  positions: Float32Array;
  normals: Float32Array;
  /** The face each vertex is of: an index into `faces`. */
  face: Uint32Array;
  /** Three vertices a triangle, in the order `faces` names: face `f`'s triangles are the run
   *  `f.start` to `f.start + f.count`, so its indices are three times that. */
  index: Uint32Array;
  /** The edges always inked — where two faces meet at a corner, or the surface ends — six
   *  numbers a segment. */
  sharp: Float32Array;
  /** Where two faces meet tangent (a round's run-out): drawn lighter, or not at all. */
  tangent: Float32Array;
  faces: Face[];
}

/** Solid `idx` in its own coordinates as a renderer draws it, refined to `unit` as `mesh` is.
 *  Throws with the core's reason where it does not evaluate. */
export function drawn(sk: Sketch, idx: number, unit: number): Drawn {
  const faces = takeJson<Face[] | null>(core().gcs_solid_faces_json(sk.handle, idx, unit));
  if (!faces) throw new Error(lastError());
  const handle = core().gcs_solid_drawn(sk.handle, idx, unit);
  if (!handle) throw new Error(lastError());
  // one buffer, every item four bytes: the counts, then each part in turn (`gcs_solid_drawn`)
  const bytes = takeBytes(handle);
  const [nv, nt, ns, nk] = new Uint32Array(bytes.buffer, bytes.byteOffset, 5);
  let at = bytes.byteOffset + 20;
  const take = <T>(make: new (b: ArrayBuffer, o: number, n: number) => T, n: number): T => {
    const out = new make(bytes.buffer as ArrayBuffer, at, n);
    at += 4 * n;
    return out;
  };
  return {
    positions: take(Float32Array, 3 * nv),
    normals: take(Float32Array, 3 * nv),
    face: take(Uint32Array, nv),
    index: take(Uint32Array, 3 * nt),
    sharp: take(Float32Array, 6 * ns),
    tangent: take(Float32Array, 6 * nk),
    faces,
  };
}

/** Solid `idx`'s faces as its mesh names them, and the run of triangles each is: what an object
 *  drawn from another's geometry (a peer of its binding) calls its own.  Throws with the core's
 *  reason where it does not evaluate. */
export function faces(sk: Sketch, idx: number, unit: number): Face[] {
  const faces = takeJson<Face[] | null>(core().gcs_solid_faces_json(sk.handle, idx, unit));
  if (!faces) throw new Error(lastError());
  return faces;
}

/** Where a solid stands: which evaluation it is (`shape`, the same number exactly while it is the
 *  same shape, however it has moved), whose triangles it has (`geometry`, the same for every
 *  solid of one binding, #566: one evaluation renamed for each instance), and the 4×4 matrix,
 *  column-major, that places its own coordinates in the world. */
export interface Placement {
  shape: number;
  geometry: number;
  matrix: number[];
}

/** Where solid `idx` stands.  With `built`, only where the core already has its shape — moved or
 *  not — and null where it would have to evaluate it: what a gesture asks, which must not build a
 *  solid every frame.  Throws where the solid does not evaluate. */
export function placement(sk: Sketch, idx: number, unit: number, built = false): Placement | null {
  return withBuf(18, 8, (b) => {
    const got = core().gcs_solid_placement(sk.handle, idx, unit, built ? 1 : 0, b.ptr, 18);
    if (got === 0) return null;
    if (got < 0) throw new Error(lastError());
    const x = Array.from(b.f64.subarray(0, 18));
    return { shape: x[0], geometry: x[1], matrix: x.slice(2) };
  });
}

/** **What a Boolean would make of `target`** (indices into the sketch's solids): its mesh with
 *  `tools` joined by `word`, as the statements would make it, written nowhere — the core's
 *  `Sketch::preview_body`.  Throws with the cause where it is refused: where the statements
 *  would be (a tool that is the target or made of it), or a body with a swept operand, whose
 *  surface is meshed elsewhere and never for a preview. */
export function preview(sk: Sketch, target: number, word: BodyWord, tools: number[]): Mesh {
  const m = takeJson<{ faces: Face[]; positions: number[]; normals: number[] } | null>(
    withJson({ target, word, tools }, (p, n) => core().gcs_solid_preview_json(sk.handle, p, n)));
  if (!m) throw new Error(lastError());
  return { faces: m.faces, positions: Float64Array.from(m.positions),
           normals: Float64Array.from(m.normals) };
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

/** The objects a document has: the solids nothing else is made of.  A bore is a hole in a part,
 *  not a part beside it, and this is the rule that says so — the same one the glass box shows. */
export function objects(sk: Sketch): { name: string; index: number }[] {
  return takeJson<{ name: string; index: number }[]>(core().gcs_solid_objects_json(sk.handle));
}

/** The objects the workspace draws: `objects`, less what the model outline hid (#435). */
export function shownObjects(sk: Sketch): { name: string; index: number }[] {
  return takeJson<{ name: string; index: number }[]>(core().gcs_solid_shown_json(sk.handle));
}

/** A material the document states (#510 P3), `steel := material(E: 200GPa, …)`: each property
 *  it states in SI under its own name (pascals, kilograms a cubic metre), the core's words for
 *  it, and the solids made of it (`solid(…) of steel`). */
export interface Material {
  name: string;
  E?: number;
  nu?: number;
  rho?: number;
  yield?: number;
  text: string;
  solids: number[];
}

/** The materials the document states, and what is made of each. */
export function materials(sk: Sketch): Material[] {
  return takeJson<Material[]>(core().gcs_materials_json(sk.handle)) ?? [];
}

/** The objects that are not drawn, and why: each one that does not evaluate, with the core's
 *  reason (`` `b1`: the revolution axis crosses the profile ``).  A swept object still being
 *  meshed in a worker is not one. */
export function unbuilt(sk: Sketch): { solid: number; name: string; reason: string }[] {
  return takeJson<{ solid: number; name: string; reason: string }[]>(
    core().gcs_solid_unbuilt_json(sk.handle));
}

/** **The objects as a binary glTF.**  Every face of every object is a named node, so a viewer's
 *  outliner is the document's own tree of names; positions are in metres where the document names
 *  a unit, as the glTF spec requires.
 *
 *  `idx` of −1 asks for every object, and a `unit` of 0 lets the core choose one suited to the
 *  object rather than to a report. */
export function glb(sk: Sketch, idx = -1, unit = 0): Uint8Array {
  const handle = core().gcs_solid_glb(sk.handle, idx, unit);
  if (!handle) throw new Error(lastError());
  return takeBytes(handle);
}

/** **One object as binary STL**: what a printer takes.  Welded, so every edge has its partner —
 *  which a boundary evaluation does not give on its own and a strict validator refuses without.
 *  An STL carries triangles and nothing else: no face is named in it and no unit recorded. */
export function stl(sk: Sketch, idx: number, unit = 0): Uint8Array {
  const handle = core().gcs_solid_stl(sk.handle, idx, unit);
  if (!handle) throw new Error(lastError());
  return takeBytes(handle);
}

/** **One object's exact export by the core's own kernel** (`gcs_core::brep::export`, the functions
 *  `solventc` calls, so a file is one file wherever it is written): its STEP file, or its STL held
 *  to `tolerance` millimetres (0 for the gross bars). A body with swept cuts of the generating class
 *  is admitted and built as one sector patterned, and its STL judged against its material field:
 *  seconds, not milliseconds — which is why the app asks for it in its mesh worker. Throws with
 *  the export's refusal, the stage named. */
export function exact(sk: Sketch, idx: number, kind: 'step' | 'stl', tolerance = 0): Uint8Array {
  const handle = core().gcs_solid_exact(sk.handle, idx, kind === 'step' ? 0 : 1, tolerance);
  if (!handle) throw new Error(lastError());
  return takeBytes(handle);
}

/** **A surface still being refined, as it stands**, as binary STL: coarse, maybe open — a look at
 *  the part before the refinement has finished, never a part to print. */
export function stlPreview(sk: Sketch, idx: number, unit = 0): Uint8Array {
  const handle = core().gcs_solid_stl_preview(sk.handle, idx, unit);
  if (!handle) throw new Error(lastError());
  return takeBytes(handle);
}

/** Whether a solid's surface is still being refined (a swept solid meshed in the background):
 *  throws where the solid has no surface to ask about yet, with the core's reason. */
export function provisional(sk: Sketch, idx: number): boolean {
  const r = core().gcs_solid_provisional(sk.handle, idx);
  if (r < 0) throw new Error(lastError());
  return r === 1;
}
