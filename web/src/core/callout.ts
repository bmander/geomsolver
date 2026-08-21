/* Dimension callouts: the drafting figure the core lays out for every dimensioned constraint.
 *
 * Extension lines, a dimension line between two arrowheads, a radial leader, an angular arc and
 * the number beside it are all geometry, so all of it comes from `gcs-core/src/callout.rs` in
 * world coordinates.  This module is a proxy over that one call; the view maps the points to the
 * screen with the same transform it draws the sketch through. */
import { Sketch } from './model.js';
import { core, takeJson, withBuf } from './wasm.js';

export type Pt = [number, number];
export type Seg = [Pt, Pt];

export interface CalloutArc {
  c: Pt;
  r: number;
  /** The sweep runs counterclockwise when `a1 > a0` — the sign of the angle constrained. */
  a0: number;
  a1: number;
}

/** `at` is the tip and `dir` the direction it points, so the head fills the arrow length back
 *  along `-dir`. */
export interface Arrow {
  at: Pt;
  dir: Pt;
}

export interface Callout {
  /** The constraint this dimension states — what clicking it selects. */
  id: number;
  text: string;
  /** The centre of the label's box, the direction it reads in, and the box's four corners. */
  anchor: Pt;
  angle: number;
  label: Pt[];
  /** The dimension line itself, and the extension/witness lines a drawing puts in dashes. */
  solid: Seg[];
  thin: Seg[];
  arcs: CalloutArc[];
  arrows: Arrow[];
}

export interface Callouts {
  /** The size the layout reserved for the text, in screen pixels, and the length and barb
   *  half-width (as a fraction of that length) of an arrowhead.  Drawing the heads at any other
   *  shape would make a second front end invent its own drafting style. */
  font: number;
  arrow: number;
  barb: number;
  items: Callout[];
}

/** Every dimension in the sketch, drawn.  `unit` is the world length of one screen pixel, which
 *  is what makes the stand-offs, arrowheads and characters come out the same size at any zoom. */
export function callouts(sk: Sketch, unit: number): Callouts {
  return takeJson<Callouts>(core().gcs_callouts_json(sk.handle, unit));
}

/** The constraint whose callout the world point (x, y) lands on, within `tolPx` screen pixels.
 *  Asking the core rather than shipping the hit geometry with every frame keeps the payload to
 *  what is actually painted — and it is the same layout either way. */
export function pick(sk: Sketch, unit: number, x: number, y: number, tolPx: number): number {
  return core().gcs_callout_pick(sk.handle, unit, x, y, tolPx);
}

/** Take hold of a callout at a world point: the two numbers to hand back to `drag` for the rest
 *  of the gesture, so the callout moves with the pointer instead of jumping to it. */
export function grab(sk: Sketch, unit: number, id: number, x: number, y: number)
  : [number, number] | null {
  return withBuf(2, 8, (b) => (core().gcs_callout_grab(sk.handle, id, unit, x, y, b.ptr)
    ? [b.f64[0], b.f64[1]] as [number, number]
    : null));
}

/** Move a callout so the point it was grabbed at follows the pointer to (x, y). */
export function drag(sk: Sketch, id: number, x: number, y: number,
                     grip: readonly [number, number]): void {
  core().gcs_callout_drag(sk.handle, id, x, y, grip[0], grip[1]);
}

/** Put a callout back wherever the layout would have put it; true if it had been moved at all,
 *  so a caller can tell an edit from a no-op. */
export function reset(sk: Sketch, id: number): boolean {
  return core().gcs_callout_reset(sk.handle, id) !== 0;
}
