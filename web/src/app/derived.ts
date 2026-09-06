/* A derived picture is geometry in page coordinates. Moving the camera only transforms those
 * coordinates; it must not reclassify the solid's edges and hidden lines on every frame. */
import { derived, derivedInputs } from '../core/derived.js';
import type { Drawn } from '../core/derived.js';
import type { Sketch } from '../core/model.js';

/** Stable display detail levels, at least as fine as the requested pixel length. Half-octave
 * bands limit extra refinement to sqrt(2) and let small camera-fit/zoom changes reuse geometry.
 * Exporters continue to request their exact output tolerance directly from the core. */
export function displayUnit(unit: number): number {
  if (!(unit > 0) || !Number.isFinite(unit)) return unit;
  return Math.min(unit, 2 ** (Math.floor(Math.log2(unit) * 2) / 2));
}

interface Cached {
  sketch: Sketch;
  inputs: Float64Array;
  style: number;
  unit: number;
  items: Drawn[];
}

/** Keep the last projection through a pan or zoom. Finer curves are requested once the camera
 *  rests; edits and moving geometry refresh immediately. The exact export path still asks the
 *  core directly at its output resolution. */
export class DerivedDrawing {
  private cached: Cached | null = null;
  private refinement: ReturnType<typeof setTimeout> | null = null;

  constructor(private readonly redraw: () => void) {}

  /** Structural edits can change a solid without changing any coordinate (its depth, for
   *  example). The view's edit and document-replacement seams clear the picture explicitly. */
  clear(): void {
    this.cancelRefinement();
    this.cached = null;
  }

  read(sketch: Sketch, unit: number): Drawn[] {
    this.cancelRefinement();
    const c = this.cached;
    const inputs = derivedInputs(sketch, c?.inputs.length), style = sketch.styleEpoch;
    // A solve can move stationary coordinates by roundoff. Inputs also include unit vectors,
    // so scale the noise allowance by the largest input magnitude and a millionth of a pixel
    // at either resolution. Compare against the stored picture,
    // never the preceding frame, so a sequence of small real moves eventually refreshes it.
    const scale = inputs.reduce((m, v) => Math.max(m, Math.abs(v)), 1);
    const noise = Math.min(1, unit, c?.unit ?? unit) * 1e-6 / scale;
    if (!c || c.sketch !== sketch || c.style !== style || c.inputs.length !== inputs.length
        || inputs.some((v, i) => !(Math.abs(v - c.inputs[i]) <= noise))) {
      const detail = displayUnit(unit);
      const items = derived(sketch, detail);
      this.cached = { sketch, inputs, style, unit: detail, items };
      return items;
    }
    // Zooming out can keep the finer picture indefinitely. Zooming in transforms the same
    // polylines during the gesture, then replaces them at the final screen tolerance. Only
    // one refinement can be pending, and it never retains a sketch that a load disposed of.
    if (unit < c.unit && c.items.length) {
      this.refinement = setTimeout(() => {
        this.refinement = null;
        this.cached = null;
        this.redraw();
      }, 150);
    }
    return c.items;
  }

  private cancelRefinement(): void {
    if (this.refinement !== null) clearTimeout(this.refinement);
    this.refinement = null;
  }
}
