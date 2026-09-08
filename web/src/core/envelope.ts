/** Trials of the implicit envelope equation. A trial with nonzero residual is off the locus. */
import { Sketch } from './model.js';
import { core, lastError, withBuf } from './wasm.js';

export interface EnvelopeSample {
  position: [number, number, number];
  normal: [number, number, number];
  velocity: [number, number, number];
  /** Normal velocity per radian of the shared roll; the envelope is its zero set. */
  normalVelocity: number;
}

/** Original-source u and v bounds, followed by the declared roll interval in radians. */
export function envelopeDomain(sk: Sketch, index: number):
  [[number, number], [number, number], [number, number]] {
  return withBuf(6, 8, b => {
    if (!core().gcs_envelope_domain(sk.handle, index, b.ptr)) {
      throw new Error(lastError() || 'Envelope domain evaluation failed');
    }
    const p = b.f64;
    return [[p[0], p[1]], [p[2], p[3]], [p[4], p[5]]];
  });
}

export function envelopeSample(sk: Sketch, index: number, u: number, v: number,
  rollRadians: number): EnvelopeSample {
  return withBuf(10, 8, b => {
    if (!core().gcs_envelope_sample(sk.handle, index, u, v, rollRadians, b.ptr)) {
      throw new Error(lastError() || 'Envelope evaluation failed');
    }
    const p = b.f64;
    return { position: [p[0], p[1], p[2]], normal: [p[3], p[4], p[5]],
      velocity: [p[6], p[7], p[8]], normalVelocity: p[9] };
  });
}
