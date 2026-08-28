/* A picture placed in the drawing's world, to trace over.
 *
 * **The image is not part of the document.**  It is scaffolding for the person drawing — a
 * photograph, a scan, a screenshot of the thing being digitized — and the drawing is what they
 * make *over* it.  So it lives on the view beside the camera and the tool, not in the `Sketch`
 * and not in the program text: nothing about it is solved, saved, diagnosed or undone, and
 * nothing that changes the document changes it.  Undoing a line must not move your photograph.
 *
 * Its placement is a **similarity in world coordinates** — a centre, a uniform scale and a
 * rotation — which is the same shape `camera.ts` is, and for the same reason: the picture sits
 * *in* the drawing, so it pans and zooms with it, which is the whole of what makes tracing work.
 * The arithmetic below is in world units and image pixels; every screen conversion goes through
 * the camera, so this file writes no minus sign in front of a y either.
 *
 * The geometry is separated from the drawing on purpose.  `corners`, `contains` and `transform`
 * are pure functions of the placement and can be exercised without a browser; only `paint` and
 * the loader need one.
 */
import { PICK_PX } from './view.js';
import type { Gesture } from './gesture.js';
import type { SketchView } from './view.js';

/** What the canvas will draw, and the two numbers the placement is written in terms of.  An
 *  `HTMLImageElement` is one; so is anything a test hands over. */
export interface Bitmap {
  readonly width: number;
  readonly height: number;
}

/** A picture, and where it sits in the world. */
export interface Underlay {
  image: Bitmap;
  /** What it was called, for the status line. */
  name: string;
  /** Where the image's centre is, in world units. */
  x: number;
  y: number;
  /** World units per image pixel — uniform, so the picture is never squashed. */
  scale: number;
  /** Counterclockwise from the world's +x, in radians, as every world angle here is. */
  angle: number;
  /** 0…1.  A tracing sheet is faded, or the drawing does not read over it. */
  opacity: number;
  /** Where an object URL must be released when this is replaced — `null` for one that has none
   *  (a test's stub).  The loader is the only thing that makes them and this is the only thing
   *  that lets them go, so a session that traces a dozen photographs keeps none of them. */
  url: string | null;
}

/** How faded a fresh underlay is: enough that the geometry drawn over it reads, and enough that
 *  the picture underneath still does. */
export const OPACITY = 0.55;

/** How much of the visible world a fresh underlay's longer side spans.  Not the whole of it: a
 *  picture you cannot see the edges of is one you cannot take hold of. */
const FIT = 0.6;

/** The image's own half-width and half-height, in image pixels. */
function half(u: Underlay): [number, number] {
  return [u.image.width / 2, u.image.height / 2];
}

/** A point of the image, in world coordinates.  `(px, py)` is measured from the image's centre
 *  in image pixels, with y **down** as an image counts its rows — so the corner `(-w/2, -h/2)`
 *  is the top-left one, and the picture comes out the way up it was taken. */
export function toWorld(u: Underlay, px: number, py: number): [number, number] {
  const [c, s] = [Math.cos(u.angle), Math.sin(u.angle)];
  const [x, y] = [px * u.scale, -py * u.scale];
  return [u.x + x * c - y * s, u.y + x * s + y * c];
}

/** The inverse: a world place as the image pixel it falls on, measured from the centre. */
export function toImage(u: Underlay, wx: number, wy: number): [number, number] {
  const [c, s] = [Math.cos(u.angle), Math.sin(u.angle)];
  const [dx, dy] = [wx - u.x, wy - u.y];
  const [x, y] = [dx * c + dy * s, -dx * s + dy * c];
  return [x / u.scale, -y / u.scale];
}

/** The four corners in world coordinates, in image order: top-left, top-right, bottom-right,
 *  bottom-left.  The order matters to the handles — each remembers which corner it is, so
 *  taking hold of one does not spin the picture to meet the pointer. */
export function corners(u: Underlay): [number, number][] {
  const [hw, hh] = half(u);
  return ([[-hw, -hh], [hw, -hh], [hw, hh], [-hw, hh]] as [number, number][])
    .map(([px, py]) => toWorld(u, px, py));
}

/** Whether a world place falls on the picture. */
export function contains(u: Underlay, wx: number, wy: number): boolean {
  const [hw, hh] = half(u);
  const [px, py] = toImage(u, wx, wy);
  return Math.abs(px) <= hw && Math.abs(py) <= hh;
}

/** Put a freshly loaded picture in the middle of what is being looked at, at a size that leaves
 *  its edges in view.  Where it goes after that is the user's. */
export function place(v: SketchView, image: Bitmap, name: string, url: string | null): Underlay {
  const [x, y] = v.s2w(v.width / 2, v.height / 2);
  const span = v.cam.world(Math.min(v.width, v.height)) * FIT;
  const scale = span / Math.max(image.width, image.height, 1);
  return { image, name, x, y, scale, angle: 0, opacity: OPACITY, url };
}

/** Let go of the browser resource a picture was holding.  Called wherever one is replaced or
 *  removed, so the two paths cannot disagree about whose job it was. */
export function release(u: Underlay | null): void {
  if (u?.url) URL.revokeObjectURL(u.url);
}

/* -- taking hold of it ------------------------------------------------------------- */

/** Which corner a press is on, or `-1`.  Measured in **screen** pixels: a handle is drawn at a
 *  constant size whatever the zoom, so where it *is* is a fact about the canvas.  (The rule that
 *  a pick tolerance travels as a world length is about the core's geometry, which measures out
 *  where the drawing is; a handle is the app's own chrome and never leaves this file.) */
function handleAt(v: SketchView, u: Underlay, sp: [number, number]): number {
  let best = -1;
  let near = PICK_PX + HANDLE;
  corners(u).forEach((c, i) => {
    const [sx, sy] = v.w2s(c[0], c[1]);
    const d = Math.hypot(sx - sp[0], sy - sp[1]);
    if (d < near) {
      near = d;
      best = i;
    }
  });
  return best;
}

/** Half the side of a corner handle, in screen pixels. */
export const HANDLE = 5;

/** The gesture a press starts on the picture, or `null` where it lands on neither a handle nor
 *  the picture itself — which is what lets a press beside it go on being a press beside it.
 *
 * **A corner does both.**  Dragging one sets the scale from how far the pointer is from the
 * centre and the angle from which way it lies, keeping the centre still — so one handle places
 * and turns the picture with no modifier key and no second mode, which is what a person tracing
 * actually wants to do.  The corner's own bearing is subtracted, so taking hold of one does not
 * snap the picture round to meet the pointer.
 *
 * Nothing here touches the undo stack: the undo stack is program text, and the picture is not
 * in the program. */
export function grab(v: SketchView, sp: [number, number]): Gesture | null {
  const u = v.underlay;
  if (!u) return null;
  const at = v.s2w(sp[0], sp[1]);
  const i = handleAt(v, u, sp);
  if (i >= 0) {
    const [hw, hh] = half(u);
    const local = [[-hw, -hh], [hw, -hh], [hw, hh], [-hw, hh]][i];
    // the corner's distance from the centre in image pixels, and the bearing it sits at in the
    // *unrotated* picture — the two numbers a drag turns into a scale and an angle
    const reach = Math.hypot(local[0], local[1]) || 1;
    const own = Math.atan2(-local[1], local[0]);
    return {
      transient: true,
      move: (to) => {
        const [wx, wy] = v.s2w(to[0], to[1]);
        const [dx, dy] = [wx - u.x, wy - u.y];
        const d = Math.hypot(dx, dy);
        if (d > 1e-9) {
          u.scale = d / reach;
          u.angle = Math.atan2(dy, dx) - own;
        }
        v.draw();
      },
    };
  }
  if (!contains(u, at[0], at[1])) return null;
  // the picture itself: it follows the pointer, keeping the place that was grabbed under it
  const from: [number, number] = [u.x - at[0], u.y - at[1]];
  return {
    transient: true,
    move: (to) => {
      const [wx, wy] = v.s2w(to[0], to[1]);
      u.x = wx + from[0];
      u.y = wy + from[1];
      v.draw();
    },
  };
}

/* -- drawing it -------------------------------------------------------------------- */

/** The picture, under everything.  Called before the axes, so they read over it.
 *
 * The transform is the camera's composed with the placement's, and it is assembled out of the
 * camera's own answers — where the centre is (`w2s`), how a world angle reads on the canvas
 * (`cam.angle`) and how long a world length is (`cam.len`) — rather than out of `scale` and a
 * minus sign, which is what keeps the front end's linear algebra in one file.
 *
 * There is no flip to undo.  The camera's is what makes the world's +y point up the screen, and
 * an image whose rows run down maps onto that the right way up already: image row 0 is the top
 * of the picture and the top of the picture is what appears at the top. */
export function paintUnderlay(v: SketchView): void {
  const u = v.underlay;
  if (!u || u.opacity <= 0) return;
  const ctx = v.ctx;
  const k = v.cam.len(u.scale);        // screen pixels per image pixel
  if (!(k > 0) || !isFinite(k)) return;
  ctx.save();
  ctx.globalAlpha = u.opacity;
  const [sx, sy] = v.w2s(u.x, u.y);
  ctx.translate(sx, sy);
  ctx.rotate(v.cam.angle(u.angle));
  ctx.scale(k, k);
  ctx.drawImage(u.image as CanvasImageSource, -u.image.width / 2, -u.image.height / 2);
  ctx.restore();
}

/** The frame and its four handles, over everything — drawn only while the image tool is down,
 *  which is the whole of what that tool is: with any other tool the picture is scenery and a
 *  press goes straight through it to the drawing. */
export function paintHandles(v: SketchView, frame: string, ink: string): void {
  const u = v.underlay;
  if (!u) return;
  const ctx = v.ctx;
  const pts = corners(u).map(([x, y]) => v.w2s(x, y));
  ctx.save();
  ctx.setLineDash([]);
  ctx.lineWidth = 1;
  ctx.strokeStyle = frame;
  ctx.beginPath();
  pts.forEach(([x, y], i) => (i ? ctx.lineTo(x, y) : ctx.moveTo(x, y)));
  ctx.closePath();
  ctx.stroke();
  ctx.fillStyle = ink;
  for (const [x, y] of pts) {
    ctx.fillRect(x - HANDLE, y - HANDLE, HANDLE * 2, HANDLE * 2);
  }
  ctx.restore();
}
