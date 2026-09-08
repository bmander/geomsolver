import { Sketch } from './model.js';
import type { SeamTolerance } from './seam.js';
import { core, lastError, withBuf } from './wasm.js';

export interface VertexTolerance {
  axis: number;
  normalVelocity: number;
  incidence: number;
  trim: number;
}

type Domain = [[number, number], [number, number], [number, number]];
type Position = [number, number, number];

/** Original generating chart; this is a local search box, not a uniqueness certificate. */
export function boundaryVertexDomain(sk: Sketch, index: number, axisTolerance: number): Domain {
  return withBuf(6, 8, b => {
    if (!core().gcs_boundary_vertex_domain(sk.handle, index, axisTolerance, b.ptr)) {
      throw new Error(lastError() || 'Boundary vertex domain evaluation failed');
    }
    const p = b.f64;
    return [[p[0], p[1]], [p[2], p[3]], [p[4], p[5]]];
  });
}

export function boundaryVertexPosition(sk: Sketch, index: number, u: number, v: number, roll: number,
  tolerance: VertexTolerance): Position {
  return withBuf(3, 8, b => {
    if (!core().gcs_boundary_vertex_position(sk.handle, index, u, v, roll, tolerance.axis,
      tolerance.normalVelocity, tolerance.incidence, tolerance.trim, b.ptr)) {
      throw new Error(lastError() || 'Boundary vertex evaluation failed');
    }
    const p = b.f64;
    return [p[0], p[1], p[2]];
  });
}

/** The junction's first-face chart, with its shared endpoint u fixed structurally. */
export function junctionVertexDomain(sk: Sketch, index: number, tolerance: SeamTolerance): Domain {
  return withBuf(6, 8, b => {
    if (!core().gcs_junction_vertex_domain(sk.handle, index, tolerance.position,
      tolerance.normal, tolerance.axis, b.ptr)) {
      throw new Error(lastError() || 'Junction vertex domain evaluation failed');
    }
    const p = b.f64;
    return [[p[0], p[1]], [p[2], p[3]], [p[4], p[5]]];
  });
}

export function junctionVertexPosition(sk: Sketch, index: number, u: number, v: number, roll: number,
  tolerance: SeamTolerance & VertexTolerance): Position {
  return withBuf(3, 8, b => {
    if (!core().gcs_junction_vertex_position(sk.handle, index, u, v, roll, tolerance.position,
      tolerance.normal, tolerance.axis, tolerance.normalVelocity, tolerance.incidence, tolerance.trim, b.ptr)) {
      throw new Error(lastError() || 'Junction vertex evaluation failed');
    }
    const p = b.f64;
    return [p[0], p[1], p[2]];
  });
}
