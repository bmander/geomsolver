/* The pictures a document asks of its solids: `view(body) in right`, `section(body, at: mid) in
 * front` (Solvent §6.11).
 *
 * **The core projects and the front end strokes**, the seam `callout.ts` sits on.  What arrives
 * is polylines in 2D *world* coordinates with the ink already resolved, so the canvas maps them
 * through the camera exactly as it maps a callout's figure, and no 3D arithmetic — and no rule
 * about what a hidden line looks like — exists above the ABI. */
import { Sketch } from './model.js';
import { core, takeJson } from './wasm.js';

/** How a stroke is inked, resolved by the core's own style cascade — so a document's
 *  `style .hidden { … }` rule reaches a derived view with nothing added on this side. */
export interface Ink {
  color?: string;
  width?: number;
  dash?: number[];
}

export interface Drawn {
  /** The polyline, in 2D world coordinates. */
  pts: [number, number][];
  /** Which `view`/`section` statement asked for it. */
  of: number;
  /** The solid it is a picture of, and which face of it this stroke bounds. */
  solid: string;
  path: string;
  stroke: Ink;
  /** The material stands between this line and the eye.  Absent where it does not. */
  hidden?: boolean;
  /** A silhouette rather than a corner — a round surface turning away, not an edge of the
   *  design.  Absent where it is a corner. */
  silhouette?: boolean;
}

/** Every picture the document asked for, at the zoom it is being looked at: `unit` is the world
 *  length of one screen pixel, which is what refines a round surface. */
export function derived(sk: Sketch, unit: number): Drawn[] {
  return takeJson<Drawn[]>(core().gcs_derived_json(sk.handle, unit));
}
