import { Sketch } from './model.js';
import type { SeamTolerance } from './seam.js';
import type { VertexTolerance } from './vertex.js';
import { core, lastError, withBuf } from './wasm.js';

type Parameters = [number, number, number];

/** Endpoint witnesses use the vertices' canonical charts. The returned parameters
 * use the edge's seam chart. Fraction is linear signed distance along the declared
 * line, not an interpolated spatial polyline or a globally certified branch. */
export function edgeSample(sk: Sketch, index: number, endpoints: [Parameters, Parameters], fraction: number,
  tolerance: SeamTolerance & VertexTolerance, maxIterations = 100):
  { position: [number, number, number]; parameters: Parameters } {
  return withBuf(12, 8, b => {
    b.f64.set(endpoints[0], 0);
    b.f64.set(endpoints[1], 3);
    if (!core().gcs_edge_sample(sk.handle, index, b.ptr, fraction, tolerance.position,
      tolerance.normal, tolerance.axis, tolerance.normalVelocity, tolerance.incidence,
      tolerance.trim, maxIterations, b.ptr+6*8)) {
      throw new Error(lastError() || 'Edge evaluation failed');
    }
    const p = b.f64;
    return { position: [p[6], p[7], p[8]], parameters: [p[9], p[10], p[11]] };
  });
}
