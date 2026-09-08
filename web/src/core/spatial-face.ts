import { Sketch } from './model.js';
import type { SeamTolerance } from './seam.js';
import type { VertexTolerance } from './vertex.js';
import { core, lastError, withBuf } from './wasm.js';

type Parameters = [number, number, number];

/** Ordered directed uses, validated against shared vertex and support identities. */
export function spatialFaceLoop(sk: Sketch, index: number): { edge: number; reversed: boolean }[] {
  const n = core().gcs_spatial_face_loop(sk.handle, index, 0, 0);
  if (n < 0) throw new Error(lastError() || 'Face boundary is invalid');
  return withBuf(n*2, 4, b => {
    if (core().gcs_spatial_face_loop(sk.handle, index, b.ptr, n) !== n) {
      throw new Error(lastError() || 'Face boundary changed');
    }
    return Array.from({ length: n }, (_, i) => ({ edge: b.i32[2*i], reversed: b.i32[2*i+1] < 0 }));
  });
}

/** Witness triples are indexed by spatial vertex ID. Fraction follows this face's
 * directed edge use. The core returns the shared position and this support's chart. */
export function faceBoundarySample(sk: Sketch, index: number, edge: number, fraction: number,
  vertices: Parameters[], tolerance: SeamTolerance & VertexTolerance, maxIterations = 100):
  { position: Parameters; parameters: Parameters; incidenceError: number } {
  const offset = vertices.length*3;
  return withBuf(offset+7, 8, b => {
    vertices.forEach((p, i) => b.f64.set(p, i*3));
    if (!core().gcs_face_boundary_sample(sk.handle, index, edge, fraction, b.ptr, vertices.length,
      tolerance.position, tolerance.normal, tolerance.axis, tolerance.normalVelocity,
      tolerance.incidence, tolerance.trim, maxIterations, b.ptr+offset*8)) {
      throw new Error(lastError() || 'Face boundary evaluation failed');
    }
    const p = b.f64;
    return { position: [p[offset], p[offset+1], p[offset+2]],
      parameters: [p[offset+3], p[offset+4], p[offset+5]], incidenceError: p[offset+6] };
  });
}
