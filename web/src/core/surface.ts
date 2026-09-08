/** Exact model surfaces. Positions and both parameter derivatives come from the core. */
import { Sketch } from './model.js';
import { core, lastError, withBuf } from './wasm.js';

export interface SurfaceSample {
  position: [number, number, number];
  du: [number, number, number];
  dv: [number, number, number];
}

/** Outward-rounded profile bounds before revolution. These enclose the analytic
 * snapshot coefficients, not a swept envelope, source solve error or export error. */
export function generatingProfileBounds(sk: Sketch, index: number, u: [number, number]):
  { position: [[number, number], [number, number], [number, number]];
    du: [[number, number], [number, number], [number, number]] } {
  return withBuf(12, 8, b => {
    if (!core().gcs_surface_profile_bounds(sk.handle, index, u[0], u[1], b.ptr)) {
      throw new Error(lastError() || 'Generating profile bounds failed');
    }
    const p = b.f64;
    return { position: [[p[0], p[1]], [p[2], p[3]], [p[4], p[5]]],
      du: [[p[6], p[7]], [p[8], p[9]], [p[10], p[11]]] };
  });
}

/** Bounds in the original source chart; naming a span does not renumber its points. */
export function surfaceDomain(sk: Sketch, index: number): [[number, number], [number, number]] {
  return withBuf(4, 8, b => {
    if (!core().gcs_surface_domain(sk.handle, index, b.ptr)) {
      throw new Error(lastError() || 'Surface domain evaluation failed');
    }
    const p = b.f64;
    return [[p[0], p[1]], [p[2], p[3]]];
  });
}

export function surfaceSample(sk: Sketch, index: number, u: number, v: number): SurfaceSample {
  return withBuf(9, 8, b => {
    if (!core().gcs_surface_sample(sk.handle, index, u, v, b.ptr)) {
      throw new Error(lastError() || 'Surface evaluation failed');
    }
    const p = b.f64;
    return { position: [p[0], p[1], p[2]], du: [p[3], p[4], p[5]], dv: [p[6], p[7], p[8]] };
  });
}

/** A point on the finite patch and the equation of its continued supporting surface. */
export interface SurfaceProjection {
  point: [number, number, number];
  normal: [number, number, number];
  parameters: [number, number];
  signedResidual: number;
  /** Distance to point; small proves incidence, but this is not a global minimum distance. */
  incidenceError: number;
}

export function surfaceProjection(sk: Sketch, index: number,
  point: [number, number, number]): SurfaceProjection {
  return withBuf(10, 8, b => {
    if (!core().gcs_surface_project(sk.handle, index, ...point, b.ptr)) {
      throw new Error(lastError() || 'Surface projection failed');
    }
    const p = b.f64;
    return { point: [p[0], p[1], p[2]], normal: [p[3], p[4], p[5]],
      parameters: [p[6], p[7]], signedResidual: p[8], incidenceError: p[9] };
  });
}
