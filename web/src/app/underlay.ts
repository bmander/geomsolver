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
 * **It is handled the way everything else on the canvas is** — clicked to select it, dragged to
 * move it, Delete to take it away — under the ordinary select tool, with no mode of its own.
 * Two rules keep that from getting in the way of the tracing it exists for:
 *
 *   * **The drawing outranks the picture.**  A press is offered to the geometry first, so a
 *     line lying across the picture is what a click on that line picks.
 *   * **Only its frame is clickable, never its interior.**  Nothing in this drawing is picked by
 *     an area — a circle is picked by its rim and not by the disc inside it — so the picture is
 *     not either.  Its edge is where you take hold of it and its middle is where you draw, which
 *     is exactly the division tracing wants.  Once it *is* selected the whole of it drags, so
 *     placing it is not a fight with a two-pixel border: selected, you are handling the picture;
 *     click away and you are drawing again.
 *
 * The geometry is separated from the drawing on purpose.  `corners`, `contains` and the two
 * coordinate maps are pure functions of the placement and can be exercised without a browser;
 * only `paint` and the loader need one.
 */
import { PICK_PX } from './view.js';
import { COL, polyPath } from './paint.js';
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
  /** Selected.  Its own flag and not a place in `SketchView.selected`, which holds `Primitive`s
   *  — things the core knows about, that can be constrained, dimensioned, diagnosed and named.
   *  A photograph is none of those, and putting it in that list would mean answering for it at
   *  every seam that reads one.  The two are kept exclusive instead: selecting either clears
   *  the other, so what Delete takes is never in doubt. */
  picked: boolean;
  /** Where an object URL must be released when this is replaced — `null` for one that has none
   *  (a test's stub).  The loader is the only thing that makes them and this is the only thing
   *  that lets them go, so a session that traces a dozen photographs keeps none of them. */
  url: string | null;
}

/** How faded a fresh underlay is: enough that the geometry drawn over it reads, and enough that
 *  the picture underneath still does. */
const OPACITY = 0.55;

/** Half the side of a corner handle, in screen pixels. */
const HANDLE = 5;

/** How much of the visible world a fresh underlay's longer side spans.  Not the whole of it: a
 *  picture you cannot see the edges of is one you cannot take hold of. */
const FIT = 0.6;

/** The image's own half-width and half-height, in image pixels. */
function half(u: Underlay): [number, number] {
  return [u.image.width / 2, u.image.height / 2];
}

/** The four corners of the image, in image pixels from its centre, in image order. */
function box(u: Underlay): [number, number][] {
  const [hw, hh] = half(u);
  return [[-hw, -hh], [hw, -hh], [hw, hh], [-hw, hh]];
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
  return box(u).map(([px, py]) => toWorld(u, px, py));
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
  const span = v.world(Math.min(v.width, v.height)) * FIT;
  const scale = span / Math.max(image.width, image.height, 1);
  return { image, name, x, y, scale, angle: 0, opacity: OPACITY, picked: false, url };
}

/** Let go of the browser resource a picture was holding.  Called wherever one is replaced or
 *  removed, so the two paths cannot disagree about whose job it was. */
export function release(u: Underlay | null): void {
  if (u?.url) URL.revokeObjectURL(u.url);
}

/* -- what a press lands on ---------------------------------------------------------
 *
 * All of it is measured in **screen** pixels.  A frame and its handles are drawn at a constant
 * size whatever the zoom, so where they *are* is a fact about the canvas.  (The rule that a pick
 * tolerance travels as a world length is about the core's geometry, which measures out where the
 * drawing is; this is the app's own chrome and never leaves this file.) */

/** Which corner handle a press is on, or `-1`.  Handles exist only while the picture is
 *  selected, so this answers `-1` the rest of the time. */
export function handleAt(v: SketchView, sp: [number, number]): number {
  const u = v.underlay;
  if (!u?.picked) return -1;
  return corners(u).findIndex(([x, y]) => {
    const s = v.w2s(x, y);
    return Math.hypot(s[0] - sp[0], s[1] - sp[1]) < PICK_PX + HANDLE;
  });
}

/** Whether a press would take hold of the picture itself: anywhere on it once it is selected,
 *  and only within a pick tolerance of its frame while it is not. */
export function bodyAt(v: SketchView, sp: [number, number]): boolean {
  const u = v.underlay;
  if (!u) return false;
  const at = v.s2w(sp[0], sp[1]);
  if (u.picked) return contains(u, at[0], at[1]);
  const pts = corners(u).map(([x, y]) => v.w2s(x, y));
  return pts.some((p, i) => segmentDistance(sp, p, pts[(i + 1) % 4]) <= PICK_PX);
}

/** Distance from a point to a segment, in whatever units all three are in. */
function segmentDistance(p: [number, number], a: [number, number], b: [number, number]): number {
  const [vx, vy] = [b[0] - a[0], b[1] - a[1]];
  const len2 = vx * vx + vy * vy;
  const t = len2 > 0 ? Math.max(0, Math.min(1, ((p[0] - a[0]) * vx + (p[1] - a[1]) * vy) / len2)) : 0;
  return Math.hypot(p[0] - (a[0] + t * vx), p[1] - (a[1] + t * vy));
}

/* -- taking hold of it -------------------------------------------------------------- */

/** Drag a corner: it sizes **and** turns the picture, about a centre that stays put.
 *
 *  One handle doing both is deliberate.  The scale comes from how far the pointer is from the
 *  centre and the angle from which way it lies, with the corner's own bearing subtracted so that
 *  taking hold of one does not snap the picture round to meet the pointer — the corner you
 *  grabbed simply stays under it.  No modifier key, and no second mode to be in.
 *
 *  Nothing here touches the undo stack: the undo stack is program text, and the picture is not
 *  in the program. */
export function grabHandle(v: SketchView, i: number): Gesture {
  const u = v.underlay!;
  const local = box(u)[i];
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

/** Drag the picture: it follows the pointer, keeping the place that was grabbed under it. */
export function grabBody(v: SketchView, sp: [number, number]): Gesture {
  const u = v.underlay!;
  const at = v.s2w(sp[0], sp[1]);
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
  const k = v.len(u.scale);            // screen pixels per image pixel
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

/** Its frame, over everything — and the four corner handles once it is selected.
 *
 *  The frame is drawn whether or not it is selected, because it is the *only* part of the
 *  picture a press can take hold of unselected: an affordance you cannot see is one nobody
 *  finds.  Selected and hovered recolour it exactly as they recolour a line, so the picture
 *  answers a pointer the way the rest of the canvas does.
 *
 *  Hovered is **derived here from `v.cursor`**, the way the snap indicator is, rather than
 *  stored on the picture and written from the move handler: a flag written in one place and
 *  read in another goes stale the moment something returns early — changing tool with the
 *  pointer over the frame left it lit, promising an affordance that no longer answers. */
export function paintFrame(v: SketchView): void {
  const u = v.underlay;
  if (!u) return;
  const ctx = v.ctx;
  const world = corners(u);
  const hover = v.tool === 'select'
    && (handleAt(v, v.cursor) >= 0 || bodyAt(v, v.cursor));
  const col = u.picked ? COL.sel : hover ? COL.highlight : COL.imageFrame;
  ctx.save();
  ctx.setLineDash(u.picked ? [] : [6, 4]);
  ctx.lineWidth = u.picked ? 2 : 1;
  ctx.strokeStyle = col;
  polyPath(v, world);
  ctx.closePath();
  ctx.stroke();
  if (u.picked) {
    ctx.setLineDash([]);
    ctx.fillStyle = col;
    for (const [wx, wy] of world) {
      const [x, y] = v.w2s(wx, wy);
      ctx.fillRect(x - HANDLE, y - HANDLE, HANDLE * 2, HANDLE * 2);
    }
  }
  ctx.restore();
}
