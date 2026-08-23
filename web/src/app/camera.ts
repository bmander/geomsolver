/* The camera: where the drawing sits on the canvas, and the only arithmetic in the front end
 * that turns one space into the other.
 *
 * It is a similarity — a uniform scale, a translation, and the flip that comes of the canvas
 * putting y downwards — so it carries lengths and angles faithfully: a distance in world units
 * is the same distance in pixels divided by `scale`, whichever way round it is measured.  That
 * is what lets every *geometric* question be asked of the core out in world coordinates, and
 * leaves this file the whole of the front end's linear algebra. */

/** A world box, as the model reports one: (xmin, ymin, xmax, ymax). */
export type Box = readonly [number, number, number, number];

export class Camera {
  /** Screen pixels per world unit. */
  scale = 6;
  /** Where world (0, 0) falls on the canvas. */
  originX = 80;
  originY = 500;

  /** A world point on the canvas. */
  w2s(x: number, y: number): [number, number] {
    return [this.originX + x * this.scale, this.originY - y * this.scale];
  }

  /** A canvas point in the world. */
  s2w(sx: number, sy: number): [number, number] {
    return [(sx - this.originX) / this.scale, (this.originY - sy) / this.scale];
  }

  /** A world *direction* on the canvas: the map without its translation, which is where the
   *  flipped y axis lives.  The length is carried across unchanged, so a unit direction stays
   *  one — the callers that want pixels ask for them by scaling afterwards. */
  dir(dx: number, dy: number): [number, number] {
    return [dx, -dy];
  }

  /** A world angle (counterclockwise from +x) as the canvas measures one — the same turn seen
   *  in a mirror, because the canvas's y points the other way. */
  angle(a: number): number {
    return -a;
  }

  /** A world length in screen pixels. */
  len(w: number): number {
    return w * this.scale;
  }

  /** A screen length as the world length it stands for — the inverse of `len`, and how a
   *  tolerance in pixels reaches the core, which measures out where the geometry is. */
  world(px: number): number {
    return px / this.scale;
  }

  /** The world length of one screen pixel — what the core sizes annotation and pick tolerances
   *  through: `world(1)`, named because that is how the core's arguments read. */
  get unit(): number {
    return this.world(1);
  }

  /** Drag the drawing by a screen offset. */
  panBy(dx: number, dy: number): void {
    this.originX += dx;
    this.originY += dy;
  }

  /** Zoom by `f` about a point on the canvas, which therefore stays under the pointer. */
  zoomAt(sx: number, sy: number, f: number): void {
    this.originX = sx + (this.originX - sx) * f;
    this.originY = sy + (this.originY - sy) * f;
    this.scale *= f;
  }

  /** Put a world box on a canvas of this size, centred, with a margin around it. */
  fitTo(box: Box, width: number, height: number, margin = 0.8): void {
    const [x0, y0, x1, y1] = box;
    this.scale = margin * Math.min(width / (x1 - x0 || 1), height / (y1 - y0 || 1));
    const cx = (x0 + x1) / 2, cy = (y0 + y1) / 2;
    this.originX = width / 2 - cx * this.scale;
    this.originY = height / 2 + cy * this.scale;
  }
}
