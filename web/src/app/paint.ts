/* What the canvas shows: the sketch itself, the dimension callouts over it, the conflict
 * halos and the tool preview.  Everything here reads the view and strokes; nothing here
 * changes the document.  The figures a dimension is made of are laid out by the core in world
 * coordinates — this only maps them to the screen. */
import * as io from '../core/io.js';
import * as dim from '../core/callout.js';
import type { Pt, Seg } from '../core/callout.js';
import {
  Arc, Circle, Line, Plane, Point, Primitive, Spline, Style, onRadius,
  threePointArc,
} from '../core/model.js';
import { tellDimension } from './dimension.js';
import { PAGE } from '../core/workspace.js';
import { paintFrame, paintUnderlay } from './underlay.js';
import type { SketchView } from './view.js';

export const COL = {
  bg: '#fafafa',
  axis: '#dddddd',
  line: '#1f77b4',
  circle: '#2ca02c',
  arc: '#ff7f0e',
  spline: '#8c564b',
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
/** How foreshortened a view may be before its callouts are left out: a dimension on a plane
 *  seen within about five degrees of edge on is a squashed line of text along a line, and says
 *  nothing a reader can use.  The geometry is still drawn — it is where the plane is. */
const EDGE_ON = 12;
/* entity colouring by constraint state (FreeCAD-style, but from the DM decomposition and the
 * conflict set rather than from a guess) */
const COL_STATE: Record<string, string> = {
  well: '#2ca02c', under: '#e69500', over: '#d62728', conflict: '#d62728',
};

/* What to stroke an entity with.  The *document's* half — dash, weight, ink — is resolved in the
 * core from its style sheet and arrives as a `Style`; the app's own chrome — selection,
 * highlight, colour-by-state — is layered over it here, because that is a view toggle and not a
 * statement in the document.  `paint` knows what a class is nowhere.
 *
 * Module-level rather than a closure so the three.js layer inks by the same rule (`chromeOf`): a
 * thing picked is lit the same way wherever it is drawn. */
function strokeOf(v: SketchView, sel: Set<Primitive>, hl: Set<Primitive>,
                  base: string, ent: Primitive, st?: Style): [string, number] {
  const lw = st?.width ?? 1.8;   // the other copy is `svg::PLAIN_PX`; the two must agree
  const chrome = chromeOf(sel, hl, ent);
  if (chrome) return [chrome[0], lw + chrome[1]];
  if (v.colorByState) return [COL_STATE[v.stateOf(ent)], lw];
  return [st?.color ?? base, lw];
}

/** The app's own layer over the document's ink — selected, then highlighted — as a colour and
 *  the weight it adds, or null where the document's ink shows through.  The one statement of
 *  that order, read by the sheet's stroke and by the box's datum marks alike. */
export function chromeOf(sel: Set<Primitive>, hl: Set<Primitive>, ent: Primitive): [string, number] | null {
  if (sel.has(ent)) return [COL.sel, 1.5];
  if (hl.has(ent)) return [COL.highlight, 1];
  return null;
}

export function paint(v: SketchView): void {
  const ctx = v.ctx;
  const w = v.width, h = v.height;
  // this canvas is the upper of two and must be see-through: the ground, the panes and the
  // solids are three.js's, one canvas down, and every sketch is stroked over them here
  ctx.clearRect(0, 0, w, h);
  v.box3d.paint(v);
  ctx.save();
  // the picture being traced, under the drawing: it lies on the page plane
  paintUnderlay(v);
  ctx.lineCap = 'round';

  const sk = v.sketch;
  const sel = new Set(v.selected);
  const hl = new Set(v.highlight);
  const strokeFor = (base: string, ent: Primitive, st?: Style): [string, number] =>
    strokeOf(v, sel, hl, base, ent, st);

  // **every figure in its own view**: a line from where each end is seen (a projector between
  // two views runs between them), and anything round, or a curve, through the camera of the one
  // view it stands in
  for (const ln of sk.lines) {
    const st = ln.style;
    if (st.hidden) continue;
    const [col, lw] = strokeFor(COL.line, ln, st);
    ctx.strokeStyle = col;
    ctx.lineWidth = lw;
    ctx.setLineDash(st.dash);
    ctx.beginPath();
    ctx.moveTo(...v.seen(ln.p1));
    ctx.lineTo(...v.seen(ln.p2));
    ctx.stroke();
  }
  for (const c of sk.circles) {
    const st = c.style;
    if (st.hidden) continue;
    const [col, lw] = strokeFor(COL.circle, c, st);
    v.inView(v.viewOfEntity(c), () => {
      ctx.strokeStyle = col;
      ctx.lineWidth = lw;
      ctx.setLineDash(st.dash);
      circlePath(v, c.center.xy, Math.abs(c.radius.value));
      ctx.stroke();
    });
  }
  for (const a of sk.arcs) {
    const st = a.style;
    if (st.hidden) continue;
    const [col, lw] = strokeFor(COL.arc, a, st);
    v.inView(v.viewOfEntity(a), () => {
      ctx.strokeStyle = col;
      ctx.lineWidth = lw;
      ctx.setLineDash(st.dash);
      arcPath(v, a.center.xy, Math.abs(a.radius.value), ...a.angles());
      ctx.stroke();
    });
  }
  // curves written in the language: the core lays out the polyline, exactly as it does for a
  // B-spline, so the front end strokes what it is handed and evaluates no expression of its own
  for (const cv of sk.curves) {
    const st = cv.style;
    if (st.hidden) continue;
    const [col, lw] = strokeFor(COL.spline, cv, st);
    v.inView(v.viewOfEntity(cv), () => {
      ctx.strokeStyle = col;
      ctx.lineWidth = lw;
      ctx.setLineDash(st.dash);
      polyPath(v, cv.polyline());
      ctx.stroke();
      ctx.setLineDash([]);
    });
  }
  for (const sp of sk.splines) {
    const st = sp.style;
    if (st.hidden) continue;
    const [col, lw] = strokeFor(COL.spline, sp, st);
    v.inView(v.viewOfEntity(sp), () => {
      // the curve arrives as a polyline already refined to this zoom: `unit` is the eye length of
      // one screen pixel, the same number the callouts are laid out against, so the front end
      // strokes what the core hands it and never evaluates a basis function
      ctx.strokeStyle = col;
      ctx.lineWidth = lw;
      ctx.setLineDash(st.dash);
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
    });
  }
  // a plane is not stroked here: it is a pane of the workspace, drawn in space below, and the
  // plane being drawn on is named in the chooser over the viewport
  ctx.setLineDash([]);
  // a tool's preview is where its clicks are read: the plane being drawn on, or — for the plane
  // tool, whose two points are layout — the page
  if (v.pending.length || v.pendingFit.length) {
    v.inView(v.tool === 'plane' ? PAGE : v.activeView, () => paintPreview(v));
  }
  if (v.diagnosis?.conflicts?.length) paintConflicts(v);

  // one read for every point: a point's style is the sheet's `.point` rule and nothing else
  const hidePoints = sk.styleNamed('point').hidden;
  for (const p of sk.points) {
    if (hidePoints && !sel.has(p) && !hl.has(p)) continue;
    const [sx, sy] = v.seen(p);
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
      const [sx, sy] = v.seen(sp);
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
  if (!v.showDimensions && !v.liveDim) return;
  const ctx = v.ctx;
  const cs = dim.callouts(v.sketch, v.unit,
    v.showDimensions ? undefined : v.liveDim!.targets.map((c) => c.id));
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
  const live = new Set(v.liveDim?.targets.map((c) => c.id) ?? []);
  const shown = cs.items.filter((k) => live.has(k.id) || k.id === lit?.id || v.showsCallouts(k.view));
  const painted = shown.map((k) => {
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
  // each figure is laid out on the page of the view its dimension is in, and drawn through that
  // view's camera; one whose points stand in views apart in space has nowhere to be drawn
  for (const { k, col, lw } of painted) {
    v.inView(k.view, () => {
      if (!v.viewCam().readable(EDGE_ON)) return;
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
    });
  }
  ctx.font = `${cs.font}px system-ui, sans-serif`;
  ctx.textAlign = 'center';
  ctx.textBaseline = 'middle';
  for (const { k, col } of painted) {
    v.inView(k.view, () => {
      if (!v.viewCam().readable(EDGE_ON)) return;
      ctx.fillStyle = COL.bg;
      polyPath(v, k.label);
      ctx.closePath();
      ctx.fill();
      ctx.save();
      ctx.translate(...v.w2s(k.anchor[0], k.anchor[1]));
      // the number reads along its line as the view turns it, and never upside down
      let turn = v.viewCam().angle(k.angle);
      if (Math.abs(turn) > Math.PI / 2) turn -= Math.sign(turn) * Math.PI;
      ctx.rotate(turn);
      ctx.fillStyle = col;
      ctx.fillText(k.text, 0, 0);
      ctx.restore();
    });
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
  const [dx, dy] = v.viewCam().dir(dir[0], dir[1]);
  const [bx, by] = [tx - dx * size, ty - dy * size];
  const [px, py] = [-dy * size * barb, dx * size * barb];
  ctx.beginPath();
  ctx.moveTo(tx, ty);
  ctx.lineTo(bx + px, by + py);
  ctx.lineTo(bx - px, by - py);
  ctx.closePath();
  ctx.fill();
}
/** A page arc, from a0 to a1 about `centerXY` with page radius `r` — counterclockwise on the
 *  page where `ccw`.  The path is built **in page units** under the view's own transform and
 *  stroked after it is let go, so a circle on a tilted plane is the ellipse it is seen as while
 *  the line drawing it stays a constant width in pixels. */
export function arcPath(v: SketchView, centerXY: readonly [number, number], r: number,
                        a0: number, a1: number, ccw = true): void {
  const ctx = v.ctx;
  ctx.beginPath();
  ctx.save();
  v.viewCam().transform(ctx);
  // on the page an angle grows counterclockwise, which is the canvas's own clockwise sense once
  // the transform has put the page's y up
  ctx.arc(centerXY[0], centerXY[1], r, a0, a1, !ccw);
  ctx.restore();
}

/** The whole circle of page radius `r` about a page centre. */
export function circlePath(v: SketchView, centerXY: readonly [number, number], r: number): void {
  arcPath(v, centerXY, r, 0, 2 * Math.PI);
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
    // where each halo is seen, on screen, so the label sits among them whichever views they are in
    const xs: number[] = [], ys: number[] = [];
    const at = (s: [number, number]): void => { xs.push(s[0]); ys.push(s[1]); };
    for (const e of c.entities()) {
      if (e instanceof Point) {
        const [sx, sy] = v.seen(e);
        ctx.beginPath(); ctx.arc(sx, sy, 9, 0, 2 * Math.PI); ctx.stroke();
        at([sx, sy]);
      } else if (e instanceof Line) {
        ctx.beginPath();
        ctx.moveTo(...v.seen(e.p1));
        ctx.lineTo(...v.seen(e.p2));
        ctx.stroke();
        at(v.seen(e.p1));
        at(v.seen(e.p2));
      } else if (e instanceof Circle) {
        v.inView(v.viewOfEntity(e), () => {
          circlePath(v, e.center.xy, Math.abs(e.radius.value));
          ctx.stroke();
          at(v.w2s(e.center.x.value, e.center.y.value + e.radius.value));
        });
      } else if (e instanceof Arc) {
        v.inView(v.viewOfEntity(e), () => {
          const [a0, a1] = e.angles();
          arcPath(v, e.center.xy, Math.abs(e.radius.value), a0, a1);
          ctx.stroke();
          const am = 0.5 * (a0 + a1);
          at(v.w2s(e.center.x.value + e.radius.value * Math.cos(am),
                   e.center.y.value + e.radius.value * Math.sin(am)));
        });
      } else if (e instanceof Spline) {
        v.inView(v.viewOfEntity(e), () => {
          polyPath(v, e.polyline(v.unit));
          ctx.stroke();
          const [t0, t1] = e.domain;
          at(v.w2s(...e.pointAt(0.5 * (t0 + t1))));
        });
      } else if (e instanceof Plane) {
        // a plane is a pane of the workspace, so it is marked at its origin
        const [sx, sy] = v.seen(e.origin);
        ctx.beginPath(); ctx.arc(sx, sy, 9, 0, 2 * Math.PI); ctx.stroke();
        at([sx, sy]);
      }
    }
    if (!xs.length) continue;
    const [ax, ay] = [xs.reduce((a, b) => a + b, 0) / xs.length,
                      ys.reduce((a, b) => a + b, 0) / ys.length];
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
  const p0 = v.pending.length ? v.seen(v.pending[0]) : ([0, 0] as [number, number]);
  /** A dashed line from the last point placed to the cursor. */
  const rubber = (): void => {
    ctx.beginPath();
    ctx.moveTo(...v.seen(v.pending[v.pending.length - 1]));
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
