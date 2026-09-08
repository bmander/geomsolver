/** Retained source points; material trims and envelope incidence are checked in the core. */
import { Sketch } from './model.js';
import type { SurfaceSample } from './surface.js';
import type { EnvelopeSample } from './envelope.js';
import { core, lastError, withBuf } from './wasm.js';

export interface PatchTolerance {
  /** Absolute length for recognizing solved edges on the revolution axis. */
  axis: number;
  /** Absolute length for the material-boundary band. */
  trim: number;
}

export function patchSurfaceSample(sk: Sketch, index: number, u: number, v: number,
  tolerance: PatchTolerance): SurfaceSample {
  return withBuf(9, 8, b => {
    if (!core().gcs_patch_surface_sample(sk.handle, index, u, v,
      tolerance.axis, tolerance.trim, b.ptr)) {
      throw new Error(lastError() || 'Patch evaluation failed');
    }
    const p = b.f64;
    return { position: [p[0], p[1], p[2]], du: [p[3], p[4], p[5]], dv: [p[6], p[7], p[8]] };
  });
}

export function patchEnvelopeSample(sk: Sketch, index: number, u: number, v: number,
  rollRadians: number, tolerance: PatchTolerance & { normalVelocity: number }): EnvelopeSample {
  return withBuf(10, 8, b => {
    if (!core().gcs_patch_envelope_sample(sk.handle, index, u, v, rollRadians,
      tolerance.axis, tolerance.normalVelocity, tolerance.trim, b.ptr)) {
      throw new Error(lastError() || 'Patch evaluation failed');
    }
    const p = b.f64;
    return { position: [p[0], p[1], p[2]], normal: [p[3], p[4], p[5]],
      velocity: [p[6], p[7], p[8]], normalVelocity: p[9] };
  });
}
