/* What the canvas shows: the sketch itself, the dimension callouts over it, the conflict
 * halos and the tool preview.  Everything here reads the view and strokes; nothing here
 * changes the document.  The figures a dimension is made of are laid out by the core in world
 * coordinates — this only maps them to the screen. */
import * as io from '../core/io.js';
import * as dim from '../core/callout.js';
import type { Pt, Seg } from '../core/callout.js';
import {
  Arc, Circle, Line, Point, Primitive, Spline, onRadius, threePointArc,
} from '../core/model.js';
import { tellDimension } from './dimension.js';
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
  dim: '#0f6f7a',
};
/* entity colouring by constraint state (FreeCAD-style, but from the DM decomposition and the
 * conflict set rather than from a guess) */
const COL_STATE: Record<string, string> = {
  well: '#2ca02c', under: '#e69500', over: '#d62728', conflict: '#d62728',
};

/** Reference geometry is drawn dashed; the dashes are screen px, so they do not zoom. */
const CONSTRUCTION_DASH = [7, 4];

/** A dimension's extension and witness lines: finer dashes than construction geometry, since
 *  they are annotation rather than something the sketch is made of. */
const WITNESS_DASH = [4, 3];

export function paint(v: SketchView): void {
  const ctx = v.ctx;
  const w = v.width, h = v.height;
  ctx.save();
  ctx.fillStyle = COL.bg;
  ctx.fillRect(0, 0, w, h);
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
  const strokeFor = (base: string, ent: Primitive, lw = 1.8): [string, number] => {
    if (sel.has(ent)) return [COL.sel, lw + 1.5];
    if (hl.has(ent)) return [COL.highlight, lw + 1];
    return [v.colorByState ? COL_STATE[v.stateOf(ent)] : base, lw];
  };

  for (const ln of sk.lines) {
    const [col, lw] = strokeFor(COL.line, ln);
    ctx.strokeStyle = col;
    ctx.lineWidth = lw;
    ctx.setLineDash(ln.construction ? CONSTRUCTION_DASH : []);
    ctx.beginPath();
    ctx.moveTo(...v.w2s(...ln.p1.xy));
    ctx.lineTo(...v.w2s(...ln.p2.xy));
    ctx.stroke();
  }
  for (const c of sk.circles) {
    const [col, lw] = strokeFor(COL.circle, c);
    ctx.strokeStyle = col;
    ctx.lineWidth = lw;
    ctx.setLineDash(c.construction ? CONSTRUCTION_DASH : []);
    const [cx, cy] = v.w2s(...c.center.xy);
    ctx.beginPath();
    ctx.arc(cx, cy, Math.abs(c.radius.value) * v.scale, 0, 2 * Math.PI);
    ctx.stroke();
  }
  for (const a of sk.arcs) {
    const [col, lw] = strokeFor(COL.arc, a);
    ctx.strokeStyle = col;
    ctx.lineWidth = lw;
    ctx.setLineDash(a.construction ? CONSTRUCTION_DASH : []);
    arcPath(v, a.center.xy, Math.abs(a.radius.value) * v.scale, ...a.angles());
    ctx.stroke();
  }
  for (const sp of sk.splines) {
    const [col, lw] = strokeFor(COL.spline, sp);
    // the curve arrives as a polyline already refined to this zoom: `unit` is the world
    // length of one screen pixel, the same number the callouts are laid out against, so the
    // front end strokes what the core hands it and never evaluates a basis function
    ctx.strokeStyle = col;
    ctx.lineWidth = lw;
    ctx.setLineDash(sp.construction ? CONSTRUCTION_DASH : []);
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
  // the colour rule reaches for a constraint by id, so it runs once per callout rather than
  // once per callout per pass
  const painted = cs.items.map((k) => {
    const c = v.sketch.constraintById(k.id);
    const col = c && conflicts.has(c) ? COL.conflict
      : c && c === lit ? COL.highlight : COL.dim;
    return { k, col };
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
  ctx.lineWidth = 1;
  for (const { k, col } of painted) {
    ctx.strokeStyle = ctx.fillStyle = col;
    ctx.setLineDash(WITNESS_DASH);
    path(k.thin);
    ctx.setLineDash([]);
    path(k.solid);
    for (const a of k.arcs) {
      arcPath(v, a.c, a.r * v.scale, a.a0, a.a1, a.a1 > a.a0);
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
  const [dx, dy] = [dir[0], -dir[1]];            // world direction, on a screen with y down
  const [bx, by] = [tx - dx * size, ty - dy * size];
  const [px, py] = [-dy * size * barb, dx * size * barb];
  ctx.beginPath();
  ctx.moveTo(tx, ty);
  ctx.lineTo(bx + px, by + py);
  ctx.lineTo(bx - px, by - py);
  ctx.closePath();
  ctx.fill();
}
/** CCW world arc from a0 to a1; screen y is flipped, so the canvas angles are negated and
 *  the sweep runs counterclockwise in canvas terms. */
export function arcPath(v: SketchView, centerXY: [number, number], r: number, a0: number,
                        a1: number, ccw = true): void {
  const [cx, cy] = v.w2s(...centerXY);
  v.ctx.beginPath();
  v.ctx.arc(cx, cy, r, -a0, -a1, ccw);
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
        const [sx, sy] = v.w2s(...e.center.xy);
        ctx.beginPath(); ctx.arc(sx, sy, Math.abs(e.radius.value) * v.scale, 0, 2 * Math.PI); ctx.stroke();
        xs.push(e.center.x.value); ys.push(e.center.y.value + e.radius.value);
      } else if (e instanceof Arc) {
        const [a0, a1] = e.angles();
        arcPath(v, e.center.xy, Math.abs(e.radius.value) * v.scale, a0, a1);
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
    ctx.fillText(`✗ ${io.describe(c, v.sketch)}`, ax + 8, ay - 8 - 18 * n);
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
    ctx.strokeRect(p0[0], p0[1], cur[0] - p0[0], cur[1] - p0[1]);
  } else if (v.tool === 'circle') {
    ctx.beginPath();
    ctx.arc(p0[0], p0[1], Math.hypot(cur[0] - p0[0], cur[1] - p0[1]), 0, 2 * Math.PI);
    ctx.stroke();
  } else if (v.tool === 'arc3') {
    const g = v.pending.length === 2
      ? threePointArc(...v.pending[0].xy, ...v.pending[1].xy, ...v.s2w(cur[0], cur[1]))
      : null;
    if (g) {
      arcPath(v, [g.cx, g.cy], g.r * v.scale, g.a0, g.a1);
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
      const [sx2, sy2] = v.pending[1].xy;
      const rw = Math.hypot(sx2 - cx, sy2 - cy);
      const q = onRadius(cx, cy, ...v.s2w(cur[0], cur[1]), rw);
      if (q) {
        const ps = v.w2s(sx2, sy2), pe = v.w2s(...q);
        const a0 = Math.atan2(-(ps[1] - p0[1]), ps[0] - p0[0]);
        let a1 = Math.atan2(-(pe[1] - p0[1]), pe[0] - p0[0]);
        if (a1 <= a0) a1 += 2 * Math.PI;
        arcPath(v, v.pending[0].xy, rw * v.scale, a0, a1);
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
