/* The drawing as the box it was unfolded from: every view standing on its own plane in space,
 * and the object the views are of reconstructed between them.
 *
 * **The core folds and projects; the front end strokes.**  A scene arrives as 2D *world*
 * polylines — already lifted into space, orbited and flattened by `gcs-core/src/overview.rs` —
 * so the camera maps them to the screen exactly as it maps a callout's figure or a plane's
 * glyph, and the app keeps the one piece of linear algebra it has ever had (`app/camera.ts`, a
 * 2D similarity).  This module is a proxy over that one call, the shape `callout.ts` has. */
import { Sketch } from './model.js';
import { core, takeJson } from './wasm.js';

/** What a scene item *is*, so a front end can ink them apart: a pane of the glass box, a plane's
 *  own axes at its origin, a view's geometry standing on its plane, or an edge of the
 *  reconstructed object. */
export type Part = 'face' | 'axis' | 'drawn' | 'solid' | 'shell';

export interface Item {
  part: Part;
  /** The polyline, in 2D world coordinates. */
  pts: [number, number][];
  /** What it is drawn from, where an entity owns it — `Document.entityOf` resolves it, so style
   *  and selection are read off the drawing exactly as they are on the sheet.  Absent for a part
   *  of the scene no entity owns. */
  kind?: string;
  index?: number;
  /** The index of the plane it belongs to — a pane's and its axes' own, and the plane a drawn
   *  polyline stands in.  Absent for the reconstructed object, which is of no one view.  This is
   *  what "go to the view I double-clicked" reads, so the app works out for itself which view a
   *  thing is in nowhere. */
  plane?: number;
  /** For a `shell` face, how squarely it faces the light, 0 to 1 — a *number*, because which
   *  tone that is is this side's chrome and the geometry is the core's.  Absent for a line. */
  shade?: number;
}

export interface Scene {
  items: Item[];
  /** (xmin, ymin, xmax, ymax) over everything in it — what a "fit to screen" wants, since the
   *  box and the sheet have unrelated coordinates. */
  bounds: [number, number, number, number];
}

/** The scene as seen from the orbit (`az`, `el`), both in radians.  `unit` is the world length
 *  of one screen pixel, which is what refines a curve to the zoom it is being looked at. */
/** `shaded` asks for the solid's surfaces as well as its edges: back-face culled, and **far
 *  first**, so painting them in the order they arrive puts the near ones over the far ones. */
export function overview(
  sk: Sketch,
  unit: number,
  az: number,
  el: number,
  shaded = false,
): Scene {
  return takeJson<Scene>(core().gcs_overview_json(sk.handle, unit, az, el, shaded ? 1 : 0));
}
