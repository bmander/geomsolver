/* The camera: where the drawing sits on the canvas, and the only arithmetic in the front end
 * that turns one space into the other.
 *
 * Two layers, both 2D.  `Camera` is a similarity — a uniform scale, a translation, and the flip
 * that comes of the canvas putting y downwards — over the **eye's picture plane**, the flat
 * picture the core makes of the workspace for the orbit (`core/workspace.ts`).  It carries lengths
 * and angles faithfully, so a tolerance in pixels is one eye length whichever way it is measured.
 * `ViewCam` is one view's page seen through it: the core hands over the affine map of that view's
 * page onto the eye's picture plane, and composing the two is a 2×3 matrix — so a sketch on a
 * tilted plane is drawn, and a click turned back into a place on it, by 2D linear algebra alone.
 * The 3D arithmetic stays in the core, where the map came from. */

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

  /** A view's page seen through this camera: the core's map of the page onto the eye's picture
   *  plane (`m`), followed by this similarity. */
  through(m: readonly number[]): ViewCam {
    const k = this.scale;
    return new ViewCam([
      k * m[0], k * m[1], this.originX + k * m[2],
      -k * m[3], -k * m[4], this.originY - k * m[5],
    ]);
  }
}

/** **One view's page on the canvas**: `(x, y) ↦ (a·x + b·y + c, d·x + e·y + f)` in screen pixels.
 *
 *  Affine and not a similarity, because a plane seen at a slant is foreshortened: a circle drawn on
 *  it is an ellipse and its two axes are scaled differently.  So it answers what a similarity
 *  answers — where a place is, which way a direction points, how long a length looks — each in the
 *  form that stays true under a shear, and says when it cannot be inverted: a plane seen edge on
 *  has no place on it under the pointer. */
export class ViewCam {
  constructor(readonly m: readonly [number, number, number, number, number, number]) {}

  /** A page point on the canvas. */
  w2s(x: number, y: number): [number, number] {
    const m = this.m;
    return [m[0] * x + m[1] * y + m[2], m[3] * x + m[4] * y + m[5]];
  }

  /** How much of a pixel's area one unit of page area covers — zero for a plane seen edge on. */
  get det(): number {
    return this.m[0] * this.m[4] - this.m[1] * this.m[3];
  }

  /** Whether a place on this view can be read off the canvas: not when it is foreshortened more
   *  than `limit` times — about 1/cos of the angle it is turned from square on, so 12 is some 85°
   *  and the default only a view seen as good as exactly edge on. */
  readable(limit = 1e6): boolean {
    const d = Math.abs(this.det);
    const k = Math.max(Math.hypot(this.m[0], this.m[3]), Math.hypot(this.m[1], this.m[4]));
    return d > 0 && k * k / d < limit;
  }

  /** A canvas point on the view's page — where the eye's ray through it meets the plane. */
  s2w(sx: number, sy: number): [number, number] {
    const m = this.m;
    const d = this.det;
    const [x, y] = [sx - m[2], sy - m[5]];
    // `+ 0` turns a negative zero into the zero a source would write
    return [(m[4] * x - m[1] * y) / d + 0, (-m[3] * x + m[0] * y) / d + 0];
  }

  /** A page *direction* on the canvas, as a unit vector: which way it points on screen, and
   *  nothing about how long it looks.  Zero where the direction is seen end on. */
  dir(dx: number, dy: number): [number, number] {
    const m = this.m;
    const [x, y] = [m[0] * dx + m[1] * dy, m[3] * dx + m[4] * dy];
    const l = Math.hypot(x, y);
    return l > 0 ? [x / l, y / l] : [0, 0];
  }

  /** A page angle (counterclockwise from +x) as the canvas measures one. */
  angle(a: number): number {
    const [x, y] = this.dir(Math.cos(a), Math.sin(a));
    return Math.atan2(y, x);
  }

  /** A page length in screen pixels — the mean of how much the view stretches each way, which
   *  is exact where the view is seen square on. */
  len(w: number): number {
    return w * Math.sqrt(Math.abs(this.det));
  }

  /** A screen length as the page length it stands for — the inverse of `len`. */
  world(px: number): number {
    const k = Math.sqrt(Math.abs(this.det));
    return k > 0 ? px / k : Infinity;
  }

  /** This map as the canvas's own transform, in the canvas's argument order — so a path can be
   *  built in page units (an arc, a picture) and stroked afterwards in pixels. */
  transform(ctx: CanvasRenderingContext2D): void {
    const m = this.m;
    ctx.transform(m[0], m[3], m[1], m[4], m[2], m[5]);
  }
}
