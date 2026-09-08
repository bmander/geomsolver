/** Named rigid motion: the parameter is radians, velocity is distance per radian. */
import { Sketch } from './model.js';
import { core, lastError, withBuf } from './wasm.js';

export interface MotionSample {
  position: [number, number, number];
  velocity: [number, number, number];
}

export function motionSample(sk: Sketch, index: number, angleRadians: number,
  point: [number, number, number]): MotionSample {
  return withBuf(6, 8, b => {
    if (!core().gcs_motion_sample(sk.handle, index, angleRadians, ...point, b.ptr)) {
      throw new Error(lastError() || 'Motion evaluation failed');
    }
    const p = b.f64;
    return { position: [p[0], p[1], p[2]], velocity: [p[3], p[4], p[5]] };
  });
}
