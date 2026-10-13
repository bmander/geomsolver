/* Sizing what a solid tool or a datum just made, by its arrow or its number: an extrusion's
 * thickness, a revolution's turn, an offset plane's distance.  Each kind is one row of `SIZERS`
 * — where its arrow is, how a drag sizes it (a preview on the sketch) and how it is written into
 * the source — so the view asks the row and never switches on the kind.  The arrows and the
 * readings along them are the core's (`core/workspace.ts`); this moves numbers and writes text. */
import type { Constraint } from '../core/constraints.js';
import { Plane } from '../core/model.js';
import type { Way } from '../core/model.js';
import { Document } from '../core/program.js';
import type { Edit } from '../core/program.js';
import {
  extentAt, extrudeHandle, offsetAt, offsetHandle, turnAt, turnHandle,
} from '../core/workspace.js';
import type { ExtrudeHandle, TurnHandle } from '../core/workspace.js';
import { readWhole } from './dimension.js';
import type { SketchView } from './view.js';

/** A thing being sized by its arrow: an extrusion (#162 F2) — which solid, which way it runs
 *  from its face (the core's `Way`: behind it, in front of it, both ways) and how thick, in all —
 *  a revolution (F3), which solid and how far it turns, in degrees, negative clockwise about its
 *  axis — or an offset plane (F5), by the plane's name, and how far it is set off.  Open from the
 *  gesture that made it until the tool is put down or the next click; carried across an edit by
 *  name. */
export type LiveExtent =
  | { kind: 'extrusion'; solid: string; way: Way; d: number }
  | { kind: 'turn'; solid: string; d: number }
  | { kind: 'offset'; plane: string; d: number };

/** A sizing handle on the eye's picture plane or on screen: where it starts, the tip a press takes
 *  hold of (none where it cannot be read) and, for a turn, the arc it runs along. */
export interface Handle {
  base: [number, number];
  tip: [number, number] | null;
  arc?: [number, number][];
}

/** One kind of thing sized: its arrow on the eye's picture plane (null when it is gone), a drag
 *  to where the eye sees a canvas point (`both` ways, for an extrusion), and its number written
 *  — at the drag's, or `text` as typed — null when written, else why not. */
interface Sizer<L extends LiveExtent> {
  handle(v: SketchView, live: L): Handle | null;
  size(v: SketchView, live: L, sx: number, sy: number, both: boolean): void;
  commit(v: SketchView, live: L, text?: string): string | null;
}

type Of<K extends LiveExtent['kind']> = Extract<LiveExtent, { kind: K }>;

const SIZERS: { [K in LiveExtent['kind']]: Sizer<Of<K>> } = {
  extrusion: {
    handle: (v, live) => extrudeHandleOf(v, live.solid),
    // in front of its face or behind by the side the point is on, both ways with `both`
    // (`setPrism`)
    size(v, live, sx, sy, both) {
      const i = solidIndex(v, live.solid);
      if (i === undefined) return;
      const { az, el } = v.orbit;
      const t = extentAt(v.sketch, az, el, i, ...v.eye(sx, sy));
      if (!Number.isFinite(t) || Math.abs(t) < 1e-9) return;
      // two significant figures: a drag says roughly how much, the number box says exactly
      const r = Number(Math.abs(t).toPrecision(2));
      const next: LiveExtent = both ? { ...live, way: 'both', d: 2 * r }
        : { ...live, way: t > 0 ? 'front' : 'behind', d: r };
      if (next.way === live.way && next.d === live.d) return;
      v.liveExtent = next;
      v.sketch.setPrism(i, next.way, next.d);
      tellExtent(v);
      v.draw();
    },
    // the thickness over the same face and solid, the way it runs; the core writes the sweep
    commit(v, live, text) {
      const i = solidIndex(v, live.solid);
      if (i === undefined) return 'nothing is being extruded';
      const thick = text ?? String(live.d);
      return resized(v, v.doc.setSweep(i, { way: live.way, thick }), () => Number(thick));
    },
  },
  turn: {
    handle: (v, live) => turnHandleOf(v, live.solid),
    // on its turning plane, in whole degrees, signed: carried back past its start it turns the
    // other way (`setTurn`)
    size(v, live, sx, sy) {
      const i = solidIndex(v, live.solid);
      if (i === undefined) return;
      const { az, el } = v.orbit;
      const t = Math.round(turnAt(v.sketch, az, el, i, ...v.eye(sx, sy), live.d));
      if (!Number.isFinite(t) || t === 0 || t === live.d) return;
      v.liveExtent = { ...live, d: t };
      v.sketch.setTurn(i, t);
      tellExtent(v);
      v.draw();
    },
    // degrees as the core reads them (a number signed, negative clockwise), and the turn it now
    // has read back
    commit(v, live, text) {
      const i = solidIndex(v, live.solid);
      if (i === undefined) return 'nothing is being revolved';
      return resized(v, v.doc.setTurn(i, text ?? String(live.d)),
        () => turnHandleOf(v, live.solid)?.turn);
    },
  },
  offset: {
    handle(v, live) {
      const c = offsetOf(v, live.plane);
      const { az, el } = v.orbit;
      return c ? offsetHandle(v.sketch, az, el, c.id) : null;
    },
    // along the normal it is set off by — either side — previewed on the sketch by a solve,
    // written when the drag lets go
    size(v, live, sx, sy) {
      const o = offsetSlot(v, live);
      if (!o) return;
      const { c, attr } = o;
      const { az, el } = v.orbit;
      const t = offsetAt(v.sketch, az, el, c.id, ...v.eye(sx, sy));
      if (!Number.isFinite(t) || Math.abs(t) < 1e-9) return;
      const d = Number(t.toPrecision(2));
      if (d === live.d) return;
      v.liveExtent = { ...live, d };
      c.setDimension(attr, String(d));
      // the plane moves by a solve; the lists and the source wait for the release
      v.lastResult = v.solveOnce();
      tellExtent(v);
      v.draw();
    },
    // into its statement, joining the plane's own undo step
    commit(v, live, text) {
      const o = offsetSlot(v, live);
      if (!o) return 'no plane is being set off';
      const { c, attr } = o;
      const d = text ?? String(live.d);
      const e = v.doc.setDimension(c.id, attr, d);
      if (e.refused) return e.refused;
      let numeric = e.kind === 'numeric';
      try {
        if (numeric) numeric = c.setDimension(attr, d) === null;
      } catch {
        numeric = false;
      }
      if (numeric && v.doc.retext(e.text)) {
        v.afterEdit();
        v.onProgram();
      } else {
        const next = readWhole(e.text);
        if (typeof next === 'string') return next;
        v.takeDocument(next);
      }
      const n = Number(d);
      if (v.liveExtent && Number.isFinite(n)) v.liveExtent = { ...v.liveExtent, d: n };
      tellExtent(v);
      v.draw();
      return null;
    },
  },
};

/** The row of `SIZERS` for what is being sized. */
function sizerOf(live: LiveExtent): Sizer<LiveExtent> {
  return SIZERS[live.kind] as Sizer<LiveExtent>;
}

/** A solid's index in the current sketch, by the name the source gives it. */
export function solidIndex(v: SketchView, name: string): number | undefined {
  return v.doc.solids().find((s) => s.name === name)?.index;
}

/** The arrow prism `name` is sized by, on the eye's picture plane, or null. */
export function extrudeHandleOf(v: SketchView, name: string): ExtrudeHandle | null {
  const i = solidIndex(v, name);
  const { az, el } = v.orbit;
  return i === undefined ? null : extrudeHandle(v.sketch, az, el, i);
}

/** The handle revolution `name` is sized by, on the eye's picture plane, or null. */
export function turnHandleOf(v: SketchView, name: string): TurnHandle | null {
  const i = solidIndex(v, name);
  const { az, el } = v.orbit;
  return i === undefined ? null : turnHandle(v.sketch, az, el, i);
}

/** The plane distance an offset plane is set off by: `P distance(d) plane`. */
export function offsetOf(v: SketchView, plane: string): Constraint | undefined {
  const p = v.doc.entity(plane);
  if (!(p instanceof Plane)) return undefined;
  return v.sketch.constraints.find((c) => {
    const set = c.typeName === 'PlaneDistance' ? c.entities()[1] : undefined;
    return set instanceof Plane && set.index === p.index;
  });
}

/** The offset plane's distance and the slot its number is in — or null. */
function offsetSlot(v: SketchView, live: Of<'offset'>): { c: Constraint; attr: string } | null {
  const c = offsetOf(v, live.plane);
  const attr = c?.dimensions()[0]?.[0];
  return c && attr ? { c, attr } : null;
}

/** The arrow of what is being sized, on the eye's picture plane: an extrusion's, a revolution's
 *  (with the arc it turns along) or an offset plane's.  Null when nothing is, or it is gone. */
export function liveHandle(v: SketchView): Handle | null {
  const live = v.liveExtent;
  return live ? sizerOf(live).handle(v, live) : null;
}

/** The arrow what is being sized is dragged by, on screen: an extrusion's from its face's
 *  middle to its far end, an offset plane's from the plane it is set off — no tip where the eye
 *  looks down the normal — and a revolution's round the arc its profile's middle turns
 *  through, no tip where the eye looks along that.  Null when nothing is being sized. */
export function extentArrow(v: SketchView): Handle | null {
  const h = liveHandle(v);
  if (!h) return null;
  const eye = v.eyeCam();
  return {
    base: eye.w2s(...h.base), tip: h.tip && eye.w2s(...h.tip),
    arc: h.arc?.map((p) => eye.w2s(...p)),
  };
}

/** Size what is being sized to where the eye sees a canvas point — an extrusion along its
 *  normal, a revolution round its turn, an offset plane along the normal it is set off by — a
 *  preview, written when the drag lets go (`commitExtent`). */
export function sizeExtent(v: SketchView, sx: number, sy: number, both = false): void {
  const live = v.liveExtent;
  if (live) sizerOf(live).size(v, live, sx, sy, both);
}

/** Write what is being sized into the source — at the drag's number, or at `text` as typed —
 *  joining the undo step that made it.  Null when written, else why not. */
export function commitExtent(v: SketchView, text?: string): string | null {
  const live = v.liveExtent;
  return live ? sizerOf(live).commit(v, live, text) : 'nothing is being sized';
}

/** A sizing edit taken — the document read again, carrying what is being sized across by
 *  name (`swap`) and the edit into the gesture's undo step — and its number `d` now, read off
 *  what was written.  Null when taken, else why not. */
function resized(v: SketchView, e: Edit, d: () => number | undefined): string | null {
  if (e.refused) return e.refused;
  v.takeDocument(Document.read(e.text));
  const n = d();
  if (v.liveExtent && n !== undefined && Number.isFinite(n)) {
    v.liveExtent = { ...v.liveExtent, d: n };
  }
  tellExtent(v);
  v.draw();
  return null;
}

/** Stop sizing: what was last written stands. */
export function endExtent(v: SketchView): void {
  if (!v.liveExtent) return;
  v.liveExtent = null;
  v.onExtent(null, null);
}

/** Tell the shell where the number of what is being sized is: at the arrow's tip, or its base
 *  where there is no tip to hold. */
export function tellExtent(v: SketchView): void {
  const live = v.liveExtent;
  if (!live) return v.onExtent(null, null);
  const a = extentArrow(v);
  v.onExtent(live, a ? a.tip ?? a.base : null);
}
