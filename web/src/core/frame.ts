/** A frame's motion (#383): its joint carrying a point, the parameter in radians and the velocity
 *  a distance per radian of it. */
import { Sketch } from './model.js';
import { core, lastError, withBuf } from './wasm.js';

export interface FrameSample {
  position: [number, number, number];
  velocity: [number, number, number];
}

/** Where frame `index`'s joint carries `point` at the roll `angleRadians`, and how fast. */
export function frameSample(sk: Sketch, index: number, angleRadians: number,
  point: [number, number, number]): FrameSample {
  return withBuf(6, 8, b => {
    if (!core().gcs_frame_sample(sk.handle, index, angleRadians, ...point, b.ptr)) {
      throw new Error(lastError() || 'Frame evaluation failed');
    }
    const p = b.f64;
    return { position: [p[0], p[1], p[2]], velocity: [p[3], p[4], p[5]] };
  });
}
