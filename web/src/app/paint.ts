/* What the canvas shows: the sketch itself, the dimension callouts over it, the conflict
 * halos and the tool preview.  Everything here reads the view and strokes; nothing here
 * changes the document.  The figures a dimension is made of are laid out by the core in world
 * coordinates — this only maps them to the screen. */
import * as io from '../core/io.js';
import * as dim from '../core/callout.js';
import type { Pt, Seg } from '../core/callout.js';
import {
  Arc, Circle, Ellipse, Line, Plane, Point, Primitive, Spline, Style, ellipseMinor, onRadius,
  threePointArc,
} from '../core/model.js';
import { tellDimension } from './dimension.js';
import { paintFrame, paintUnderlay } from './underlay.js';
import type { SketchView } from './view.js';

export const COL = {
  bg: '#fafafa',
  axis: '#dddddd',
  line: '#1f77b4',
  circle: '#2ca02c',
  arc: '#ff7f0e',
  spline: '#8c564b',
  ellipse: '#17becf',
  point: '#222222',
  fixed: '#d62728',
  sel: '#e377c2',
  preview: '#999999',
  highlight: '#9467bd',
  conflict: '#b3001b',
  bandFill: 'rgba(227, 119, 194, 0.10)',
  /** The traced picture's frame, unselected — chrome, so the same grey a preview is drawn in:
   *  neither is part of the drawing.  Hovered and selected it takes the canvas's own two
   *  colours, so the picture answers a pointer the way everything else does. */
  imageFrame: '#999999',
  /** A plane's chord, should the sheet say nothing — the base sheet's `.plane` always does,
   *  so this is the same dead fallback a callout's ink has. */
  plane: '#8a8a8a',
};
const PLANE_FONT = '11px system-ui, sans-serif';
/* entity colouring by constraint state (FreeCAD-style, but from the DM decomposition and the
 * conflict set rather than from a guess) */
const COL_STATE: Record<string, string> = {
  well: '#2ca02c', under: '#e69500', over: '#d62728', conflict: '#d62728',
};

export function paint(v: SketchView): void {
  const ctx = v.ctx;
  const w = v.width, h = v.height;
  ctx.save();
  ctx.fillStyle = COL.bg;
  ctx.fillRect(0, 0, w, h);
  // the picture being traced, under everything: the axes and the drawing read over it
  paintUnderlay(v);
  ctx.lineCap = 'round';
  const [ox, oy] = v.w2s(0, 0);
  ctx.strokeStyle = COL.axis;
  ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.moveTo(0, oy + 0.5); ctx.lineTo(w, oy + 0.5);
  ctx.moveTo(ox + 0.5, 0); ctx.lineTo(ox + 0.5, h);
  ctx.stroke();

  const sk = v.sketch;
  const sel = new Set(v.selected);
  const hl = new Set(v.highlight);
  /* What to stroke an entity with.  The *document's* half — dash, weight, ink — is resolved in
   * the core from its style sheet and arrives as a `Style`; the app's own chrome — selection,
   * highlight, colour-by-state — is layered over it here, because that is a view toggle and not
   * a statement in the document.  `paint` knows what a class is nowhere. */
  const strokeFor = (base: string, ent: Primitive, st?: Style): [string, number] => {
    const lw = st?.width ?? 1.8;   // the other copy is `svg::PLAIN_PX`; the two must agree
    if (sel.has(ent)) return [COL.sel, lw + 1.5];
    if (hl.has(ent)) return [COL.highlight, lw + 1];
    if (v.colorByState) return [COL_STATE[v.stateOf(ent)], lw];
    return [st?.color ?? base, lw];
  };

  for (const ln of sk.lines) {
    const [col, lw] = strokeFor(COL.line, ln, ln.style);
    ctx.strokeStyle = col;
    ctx.lineWidth = lw;
    ctx.setLineDash(ln.style.dash);
    ctx.beginPath();
    ctx.moveTo(...v.w2s(...ln.p1.xy));
    ctx.lineTo(...v.w2s(...ln.p2.xy));
    ctx.stroke();
  }
  for (const c of sk.circles) {
    const [col, lw] = strokeFor(COL.circle, c, c.style);
    ctx.strokeStyle = col;
    ctx.lineWidth = lw;
    ctx.setLineDash(c.style.dash);
    circlePath(v, c.center.xy, Math.abs(c.radius.value));
    ctx.stroke();
  }
  for (const a of sk.arcs) {
    const [col, lw] = strokeFor(COL.arc, a, a.style);
    ctx.strokeStyle = col;
    ctx.lineWidth = lw;
    ctx.setLineDash(a.style.dash);
    arcPath(v, a.center.xy, Math.abs(a.radius.value), ...a.angles());
    ctx.stroke();
  }
  for (const el of sk.ellipses) {
    const [col, lw] = strokeFor(COL.ellipse, el, el.style);
    ctx.strokeStyle = col;
    ctx.lineWidth = lw;
    ctx.setLineDash(el.style.dash);
    ellipsePath(v, el.center.xy, el.major.xy, Math.abs(el.minor.value));
    ctx.stroke();
  }
  // curves written in the language: the core lays out the polyline, exactly as it does for a
  // B-spline, so the front end strokes what it is handed and evaluates no expression of its own
  for (const cv of sk.curves) {
    const [col, lw] = strokeFor(COL.spline, cv, cv.style);
    ctx.strokeStyle = col;
    ctx.lineWidth = lw;
    ctx.setLineDash(cv.style.dash);
    polyPath(v, cv.polyline());
    ctx.stroke();
    ctx.setLineDash([]);
  }
  for (const sp of sk.splines) {
    const [col, lw] = strokeFor(COL.spline, sp, sp.style);
    // the curve arrives as a polyline already refined to this zoom: `unit` is the world
    // length of one screen pixel, the same number the callouts are laid out against, so the
    // front end strokes what the core hands it and never evaluates a basis function
    ctx.strokeStyle = col;
    ctx.lineWidth = lw;
    ctx.setLineDash(sp.style.dash);
    polyPath(v, sp.polyline(v.unit));
    ctx.stroke();
    // the control polygon, only while the curve or one of its points is in play: it is how
    // the shape is edited, and clutter the rest of the time
    const live = sel.has(sp) || hl.has(sp)
      || sp.ctrl.some((p) => sel.has(p) || hl.has(p));
    if (live) {
      ctx.save();
      ctx.strokeStyle = COL.preview;
      ctx.lineWidth = 1;
      ctx.setLineDash([4, 3]);
      polyPath(v, sp.ctrl.map((p) => p.xy));
      ctx.stroke();
      ctx.restore();
    }
  }
  // a plane is its chord — origin to toward, the sheet's dashed light ink — with a tick out of
  // the origin along the chord's normal and its name beside it, upright, since the page is
  // read upright whichever way the view is turned.  Its two points are drawn as points below;
  // nothing marks the points drawn *in* it, which read as ordinary geometry
  for (const pl of sk.planes) {
    // one read: `style` crosses the ABI and comes back as JSON, and it is the same answer twice
    const st = pl.style;
    const [col, lw] = strokeFor(COL.plane, pl, st);
    // the figure is the core's, laid out at `unit` like a callout's: the chord, then the tick.
    // Nothing here derives it — the tick's direction is the frame's y-axis and its length is a
    // screen constant, and both are said in `plane::glyph`
    const [chord, tick] = pl.glyph(v.unit);
    ctx.strokeStyle = col;
    ctx.lineWidth = lw;
    ctx.setLineDash(st.dash);
    ctx.beginPath();
    ctx.moveTo(...v.w2s(...chord[0]));
    ctx.lineTo(...v.w2s(...chord[1]));
    ctx.stroke();
    ctx.setLineDash([]);
    ctx.beginPath();
    ctx.moveTo(...v.w2s(...tick[0]));
    ctx.lineTo(...v.w2s(...tick[1]));
    ctx.stroke();
    // the name is the app's: a `Sketch` holds no source names, which is the one part of the
    // glyph the core cannot say
    const [tx, ty] = v.w2s(...tick[1]);
    ctx.fillStyle = col;
    ctx.font = PLANE_FONT;
    ctx.textAlign = 'left';
    ctx.textBaseline = 'bottom';
    ctx.fillText(v.doc.nameOf(pl) ?? pl.name, tx + 3, ty - 3);
  }
  ctx.setLineDash([]);
  if (v.pending.length || v.pendingFit.length) paintPreview(v);
  if (v.diagnosis?.conflicts?.length) paintConflicts(v);

  for (const p of sk.points) {
    const [sx, sy] = v.w2s(...p.xy);
    const col = sel.has(p) ? COL.sel : hl.has(p) ? COL.highlight : p.isFixed ? COL.fixed
      : v.colorByState ? COL_STATE[v.stateOf(p)] : COL.point;
    ctx.fillStyle = col;
    if (p.isFixed) {
      ctx.fillRect(sx - 4, sy - 4, 8, 8);
    } else {
      ctx.beginPath();
      ctx.arc(sx, sy, 3.5, 0, 2 * Math.PI);
      ctx.fill();
    }
  }
  paintCallouts(v);
  // the traced picture's frame, over everything: dashed grey while it is scenery, since that
  // edge is the only part of it a press takes hold of and an affordance you cannot see is one
  // nobody finds — and the canvas's own selected/hovered colours otherwise
  paintFrame(v);
  v.gesture?.paint?.(ctx);
  if (v.tool !== 'select') {                 // snap indicator
    const sp = v.pickPoint(...v.cursor);
    if (sp) {
      const [sx, sy] = v.w2s(...sp.xy);
      ctx.strokeStyle = COL.sel;
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      ctx.arc(sx, sy, 7, 0, 2 * Math.PI);
      ctx.stroke();
    }
  }
  ctx.restore();
}

/** The dimensions, as a drawing states them.
 *
 *  The whole figure — where a dimension line stands off, which side of the shape a leader
 *  comes out on, how a short span puts its arrowheads outside — is laid out by the core in
 *  world coordinates; here it is only mapped to the screen and stroked.  Two passes, because
 *  every label clears the background behind itself: one dimension's number must not rub out
 *  the next one's line. */
export function paintCallouts(v: SketchView): void {
  if (!v.showDimensions) return;
  const ctx = v.ctx;
  const cs = dim.callouts(v.sketch, v.unit);
  const conflicts = new Set(v.diagnosis?.conflicts ?? []);
  const lit = v.litConstraint;
  /* A callout's *figure* is geometry, laid out by the core so every front end agrees where it
   * is; the ink it is stroked in is presentation, and every callout in a document shares it —
   * which is what a class is.  So the sheet says it, and this asks the sheet.  What stays on
   * the dimension's own statement is the one pair of numbers that is about that statement
   * alone: where somebody dragged this callout (spec §13.1).  Three lookups a repaint. */
  const inkDim = v.sketch.styleNamed('dimension');
  // a reference dimension *is* a dimension, so it is drawn with both classes: the shared rule,
  // and then the one that says how it differs.  Asked for `reference` alone it would miss
  // whatever the document said about `.dimension`, and a sheet that recoloured its callouts
  // would recolour half of them.
  const inkRef = v.sketch.styleNamed('dimension reference');
  const extension = v.sketch.styleNamed('extension');
  // the colour rule reaches for a constraint by id, so it runs once per callout rather than
  // once per callout per pass
  const painted = cs.items.map((k) => {
    const c = v.sketch.constraintById(k.id);
    const ink = c?.claim ? inkRef : inkDim;
    const col = c && conflicts.has(c) ? COL.conflict
      : c && c === lit ? COL.highlight
      : ink.color ?? COL.point;   // the base sheet always states one, so the fallback is dead
    return { k, col, lw: ink.width ?? 1 };
  });
  const path = (segs: Seg[]): void => {
    ctx.beginPath();
    for (const [a, b] of segs) {
      ctx.moveTo(...v.w2s(a[0], a[1]));
      ctx.lineTo(...v.w2s(b[0], b[1]));
    }
    ctx.stroke();
  };

  ctx.save();
  ctx.lineCap = 'butt';
  for (const { k, col, lw } of painted) {
    ctx.strokeStyle = ctx.fillStyle = col;
    ctx.setLineDash(extension.dash);
    ctx.lineWidth = extension.width ?? lw;   // `callout::ink` composes the thin lines this way
    path(k.thin);
    ctx.setLineDash([]);
    ctx.lineWidth = lw;
    path(k.solid);
    for (const a of k.arcs) {
      arcPath(v, a.c, a.r, a.a0, a.a1, a.a1 > a.a0);
      ctx.stroke();
    }
    for (const a of k.arrows) paintArrow(v, a.at, a.dir, cs.arrow, cs.barb);
  }
  ctx.font = `${cs.font}px system-ui, sans-serif`;
  ctx.textAlign = 'center';
  ctx.textBaseline = 'middle';
  for (const { k, col } of painted) {
    ctx.fillStyle = COL.bg;
    polyPath(v, k.label);
    ctx.closePath();
    ctx.fill();
    ctx.save();
    ctx.translate(...v.w2s(k.anchor[0], k.anchor[1]));
    ctx.rotate(-k.angle);      // the layout turns counterclockwise; the canvas turns the other
    ctx.fillStyle = col;       // way, because its y axis points down
    ctx.fillText(k.text, 0, 0);
    ctx.restore();
  }
  ctx.restore();
  if (v.liveDim) tellDimension(v, cs.items);   // the layout this frame already made
}

/** A solid head: the tip at `at`, pointing along `dir`, filling `size` screen px back, with
 *  barbs `barb` of that half-width.  Both numbers come from the layout, so the drawing style
 *  is the core's and not this front end's. */
export function paintArrow(v: SketchView, at: Pt, dir: Pt, size: number, barb: number): void {
  const ctx = v.ctx;
  const [tx, ty] = v.w2s(at[0], at[1]);
  const [dx, dy] = v.cam.dir(dir[0], dir[1]);
  const [bx, by] = [tx - dx * size, ty - dy * size];
  const [px, py] = [-dy * size * barb, dx * size * barb];
  ctx.beginPath();
  ctx.moveTo(tx, ty);
  ctx.lineTo(bx + px, by + py);
  ctx.lineTo(bx - px, by - py);
  ctx.closePath();
  ctx.fill();
}
/** A CCW world arc, from a0 to a1 about `centerXY` with world radius `r`.  Everything here is
 *  in world terms and the camera turns all of it — including the angles, which run the other
 *  way on a canvas whose y points down, so the sweep is counterclockwise in canvas terms. */
export function arcPath(v: SketchView, centerXY: readonly [number, number], r: number,
                        a0: number, a1: number, ccw = true): void {
  const [cx, cy] = v.w2s(...centerXY);
  v.ctx.beginPath();
  v.ctx.arc(cx, cy, v.len(r), v.cam.angle(a0), v.cam.angle(a1), ccw);
}

/** The whole circle of world radius `r` about a world centre. */
export function circlePath(v: SketchView, centerXY: readonly [number, number], r: number): void {
  const [cx, cy] = v.w2s(...centerXY);
  v.ctx.beginPath();
  v.ctx.arc(cx, cy, v.len(r), 0, 2 * Math.PI);
}

/** The whole rim of an ellipse, from the same five numbers the model keeps: the centre, the
 *  major-axis endpoint, and the minor radius.  The camera is a similarity, so the axis lengths
 *  scale together and the rotation is one world angle turned the canvas's way. */
export function ellipsePath(v: SketchView, centerXY: readonly [number, number],
                            majorXY: readonly [number, number], b: number): void {
  const [cx, cy] = v.w2s(...centerXY);
  const [ux, uy] = [majorXY[0] - centerXY[0], majorXY[1] - centerXY[1]];
  v.ctx.beginPath();
  v.ctx.ellipse(cx, cy, v.len(Math.hypot(ux, uy)), v.len(Math.abs(b)),
                v.cam.angle(Math.atan2(uy, ux)), 0, 2 * Math.PI);
}

/** Dashed red halo on every entity a culprit constraint references, and a label at each
 *  culprit's anchor — the culprits are what to remove, as opposed to geometry that merely
 *  turned red because it touches them. */
export function paintConflicts(v: SketchView): void {
  const ctx = v.ctx;
  const d = v.diagnosis!;
  const used = new Map<string, number>();
  ctx.save();
  ctx.setLineDash([7, 5]);
  ctx.lineWidth = 5;
  ctx.strokeStyle = COL.conflict;
  ctx.font = 'bold 13px system-ui, sans-serif';
  for (const c of d.conflicts ?? []) {
    const xs: number[] = [], ys: number[] = [];
    for (const e of c.entities()) {
      if (e instanceof Point) {
        const [sx, sy] = v.w2s(...e.xy);
        ctx.beginPath(); ctx.arc(sx, sy, 9, 0, 2 * Math.PI); ctx.stroke();
        xs.push(e.x.value); ys.push(e.y.value);
      } else if (e instanceof Line) {
        ctx.beginPath();
        ctx.moveTo(...v.w2s(...e.p1.xy));
        ctx.lineTo(...v.w2s(...e.p2.xy));
        ctx.stroke();
        xs.push(e.p1.x.value, e.p2.x.value);
        ys.push(e.p1.y.value, e.p2.y.value);
      } else if (e instanceof Circle) {
        circlePath(v, e.center.xy, Math.abs(e.radius.value));
        ctx.stroke();
        xs.push(e.center.x.value); ys.push(e.center.y.value + e.radius.value);
      } else if (e instanceof Arc) {
        const [a0, a1] = e.angles();
        arcPath(v, e.center.xy, Math.abs(e.radius.value), a0, a1);
        ctx.stroke();
        const am = 0.5 * (a0 + a1);
        xs.push(e.center.x.value + e.radius.value * Math.cos(am));
        ys.push(e.center.y.value + e.radius.value * Math.sin(am));
      } else if (e instanceof Spline) {
        polyPath(v, e.polyline(v.unit));
        ctx.stroke();
        const [t0, t1] = e.domain;
        const [mx, my] = e.pointAt(0.5 * (t0 + t1));
        xs.push(mx);
        ys.push(my);
      } else if (e instanceof Ellipse) {
        ellipsePath(v, e.center.xy, e.major.xy, Math.abs(e.minor.value));
        ctx.stroke();
        xs.push(e.major.x.value);
        ys.push(e.major.y.value);
      } else if (e instanceof Plane) {
        ctx.beginPath();
        ctx.moveTo(...v.w2s(...e.origin.xy));
        ctx.lineTo(...v.w2s(...e.toward.xy));
        ctx.stroke();
        xs.push(e.origin.x.value);
        ys.push(e.origin.y.value);
      }
    }
    if (!xs.length) continue;
    const [ax, ay] = v.w2s(xs.reduce((a, b) => a + b, 0) / xs.length,
                           ys.reduce((a, b) => a + b, 0) / ys.length);
    const cell = `${Math.floor(ax / 40)},${Math.floor(ay / 40)}`;
    const n = used.get(cell) ?? 0;
    used.set(cell, n + 1);
    ctx.save();
    ctx.setLineDash([]);
    ctx.fillStyle = COL.conflict;
    ctx.fillText(`✗ ${io.describe(c, v.doc)}`, ax + 8, ay - 8 - 18 * n);
    ctx.restore();
  }
  ctx.restore();
}

export function paintPreview(v: SketchView): void {
  const ctx = v.ctx;
  ctx.save();
  ctx.setLineDash([5, 4]);
  ctx.strokeStyle = COL.preview;
  ctx.lineWidth = 1;
  const cur = v.cursor;
  // the fit tool collects places rather than points, so `pending` may be empty here
  const p0 = v.pending.length ? v.w2s(...v.pending[0].xy) : ([0, 0] as [number, number]);
  /** A dashed line from the last point placed to the cursor. */
  const rubber = (): void => {
    ctx.beginPath();
    ctx.moveTo(...v.w2s(...v.pending[v.pending.length - 1].xy));
    ctx.lineTo(cur[0], cur[1]);
    ctx.stroke();
  };
  if (v.tool === 'splinefit') {
    // the places given so far, joined in order, and a band to the cursor.  Not the fitted
    // curve: that is a solve, and a preview that lags the cursor is worse than an honest one
    polyPath(v, v.pendingFit.map((f) => f.at));
    ctx.lineTo(cur[0], cur[1]);
    ctx.stroke();
    for (const f of v.pendingFit) {
      // a place that landed on a real point is drawn filled: it is the one the finished curve
      // will be *held* to, not merely fitted through
      const [sx, sy] = v.w2s(f.at[0], f.at[1]);
      ctx.beginPath();
      ctx.arc(sx, sy, 3, 0, 2 * Math.PI);
      if (f.on) {
        ctx.fillStyle = COL.preview;
        ctx.fill();
      } else {
        ctx.stroke();
      }
    }
  } else if (v.tool === 'line') {
    rubber();
  } else if (v.tool === 'spline') {
    // the control polygon so far, then a rubber band to the cursor: what is being placed is
    // the polygon, and the curve only exists once there are enough points for a cubic
    polyPath(v, v.pending.map((p) => p.xy));
    ctx.stroke();
    rubber();
  } else if (v.tool === 'rect') {
    // the first click is a place rather than a point, so the band starts from `pendingFit`
    const a = v.pendingFit.length ? v.w2s(...v.pendingFit[0].at) : p0;
    ctx.strokeRect(a[0], a[1], cur[0] - a[0], cur[1] - a[1]);
  } else if (v.tool === 'plane') {
    // the chord being laid down, from the first place to the cursor
    const a = v.pendingFit.length ? v.w2s(...v.pendingFit[0].at) : p0;
    ctx.beginPath();
    ctx.moveTo(a[0], a[1]);
    ctx.lineTo(cur[0], cur[1]);
    ctx.stroke();
  } else if (v.tool === 'circle') {
    const c = v.pending[0].xy;
    const w = v.s2w(cur[0], cur[1]);
    circlePath(v, c, Math.hypot(w[0] - c[0], w[1] - c[1]));
    ctx.stroke();
  } else if (v.tool === 'ellipse') {
    if (v.pending.length === 1) {
      rubber();                                    // the major axis being laid down
    } else {
      // the same rule the third click will apply: the rim passes through the cursor
      const c = v.pending[0].xy;
      const m = v.pending[1].xy;
      const b = ellipseMinor(...c, ...m, ...v.s2w(cur[0], cur[1]));
      if (b !== null) {
        ellipsePath(v, c, m, b);
        ctx.stroke();
      }
    }
  } else if (v.tool === 'arc3') {
    const g = v.pending.length === 2
      ? threePointArc(...v.pending[0].xy, ...v.pending[1].xy, ...v.s2w(cur[0], cur[1]))
      : null;
    if (g) {
      arcPath(v, [g.cx, g.cy], g.r, g.a0, g.a1);
      ctx.stroke();
    } else {
      rubber();                                    // one end so far, or a collinear cursor
    }
  } else if (v.tool === 'arc') {
    if (v.pending.length === 1) {
      rubber();
    } else {
      // the same rule the third click will apply: the cursor gives a direction, the second
      // point the radius
      const [cx, cy] = v.pending[0].xy;
      const [ex, ey] = v.pending[1].xy;
      const rw = Math.hypot(ex - cx, ey - cy);
      const q = onRadius(cx, cy, ...v.s2w(cur[0], cur[1]), rw);
      if (q) {
        // the sweep is measured where the arc will be, the same way the model measures one
        const a0 = Math.atan2(ey - cy, ex - cx);
        let a1 = Math.atan2(q[1] - cy, q[0] - cx);
        if (a1 <= a0) a1 += 2 * Math.PI;
        arcPath(v, v.pending[0].xy, rw, a0, a1);
        ctx.stroke();
      }
    }
  }
  ctx.restore();
}

/** A world-coordinate polyline as a screen path — a curve's tessellation, a control polygon,
 *  a callout label's box. */
export function polyPath(v: SketchView, pts: readonly (readonly [number, number])[]): void {
  const ctx = v.ctx;
  ctx.beginPath();
  pts.forEach((p, i) => {
    const s = v.w2s(p[0], p[1]);
    if (i) ctx.lineTo(s[0], s[1]);
    else ctx.moveTo(s[0], s[1]);
  });
}
