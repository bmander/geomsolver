import { Sketch } from './model.js';
import type { EnvelopeSample } from './envelope.js';
import { core, lastError, withBuf } from './wasm.js';

/** The envelope's chart; the boundary equation restricts it to an implicit curve. */
export function boundarySeamDomain(sk: Sketch, index: number, axisTolerance: number):
  [[number, number], [number, number], [number, number]] {
  return withBuf(6, 8, b => {
    if (!core().gcs_boundary_seam_domain(sk.handle, index, axisTolerance, b.ptr)) {
      throw new Error(lastError() || 'Boundary seam domain evaluation failed');
    }
    const p = b.f64;
    return [[p[0], p[1]], [p[2], p[3]], [p[4], p[5]]];
  });
}

export function boundarySeamSample(sk: Sketch, index: number, u: number, v: number, roll: number,
  tolerance: { axis: number; normalVelocity: number; incidence: number; trim: number }): EnvelopeSample {
  return withBuf(10, 8, b => {
    if (!core().gcs_boundary_seam_sample(sk.handle, index, u, v, roll, tolerance.axis,
      tolerance.normalVelocity, tolerance.incidence, tolerance.trim, b.ptr)) {
      throw new Error(lastError() || 'Boundary seam evaluation failed');
    }
    const p = b.f64;
    return { position: [p[0], p[1], p[2]], normal: [p[3], p[4], p[5]],
      velocity: [p[6], p[7], p[8]], normalVelocity: p[9] };
  });
}

export interface SeamTolerance {
  /** Model length tolerance at the common source vertex. */
  position: number;
  /** Difference of unit normals, allowing either orientation of the tangent plane. */
  normal: number;
  /** Model length tolerance for recognizing axis edges of clipping solids. */
  axis: number;
}

/** First source chart; u is fixed at the shared vertex. The last pair is roll radians. */
export function seamDomain(sk: Sketch, index: number, tolerance: SeamTolerance):
  [[number, number], [number, number], [number, number]] {
  return withBuf(6, 8, b => {
    if (!core().gcs_seam_domain(sk.handle, index, tolerance.position,
      tolerance.normal, tolerance.axis, b.ptr)) {
      throw new Error(lastError() || 'Seam domain evaluation failed');
    }
    const p = b.f64;
    return [[p[0], p[1]], [p[2], p[3]], [p[4], p[5]]];
  });
}

export function seamSample(sk: Sketch, index: number, u: number, v: number, roll: number,
  tolerance: SeamTolerance & { normalVelocity: number; trim: number }): EnvelopeSample {
  return withBuf(10, 8, b => {
    if (!core().gcs_seam_sample(sk.handle, index, u, v, roll, tolerance.position,
      tolerance.normal, tolerance.axis, tolerance.normalVelocity, tolerance.trim, b.ptr)) {
      throw new Error(lastError() || 'Seam evaluation failed');
    }
    const p = b.f64;
    return { position: [p[0], p[1], p[2]], normal: [p[3], p[4], p[5]],
      velocity: [p[6], p[7], p[8]], normalVelocity: p[9] };
  });
}
