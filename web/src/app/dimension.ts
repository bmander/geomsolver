/* Writing a dimension: it is stated at once, at what it measures, and its number is edited on
 * the drawing where it will be read.  Where it is put is which dimension it is, so a pair that
 * can be measured three ways swaps one constraint for another as the pointer moves; nothing is
 * solved while it is being carried, and nothing reaches the undo stack until it is accepted. */
import * as dim from '../core/callout.js';
import type { Callout } from '../core/callout.js';
import { Constraint } from '../core/constraints.js';
import { Point } from '../core/model.js';
import { applyConstraints, reportAdded } from './edit.js';
import type { SketchView } from './view.js';

/** The other dimensions a selection could have taken, and how to say the same thing as one of
 *  them.  Two points can be measured three ways — the length between them, the run, or the rise
 *  — and which one is meant is decided by where the number is put, so the alternatives have to
 *  outlive the click that started the dimension. */
export interface DimAlt {
  a: Point;
  b: Point;
  make(kind: string): Constraint;
}

/** A dimension being written: the constraints the number will land on, whether writing it is
 *  what put them there, what else it could have been, and the document as it was before any of
 *  it — which is what Escape puts back. */
export interface LiveDim {
  targets: Constraint[];
  /** They were stated to write this number, so refusing it takes them back out.  False when the
   *  number being written is one the drawing already carried. */
  fresh: boolean;
  alt: DimAlt | null;
  /** The **program text** before the gesture, which is what the undo stack holds. */
  before: string;
  /** The theorems the sketch already held, so that when the dimension lands the report can say
   *  which ones it brought with it: the sketch is not diagnosed while the number is carried. */
  impliedBefore: Set<Constraint>;
  /** Still following the pointer: the click that plants it is what ends this. */
  placing: boolean;
}

/** Start writing a dimension: `fresh` says whether the constraints are being stated now, at
 *  what they measure — in which case the callout follows the pointer until a click plants it,
 *  and on a point pair (`alt`) where it is put decides *which* of the three dimensions it
 *  states — or are already on the drawing and only their number is being written.  A stated
 *  one goes on the drawing where it will stay rather than into a box in the middle of the
 *  screen.  Nothing reaches the undo stack until the number is accepted; Escape takes the
 *  whole thing back out.
 *
 *  False if the constraints could not be added, in which case there is nothing to write. */
export function startDimension(v: SketchView, targets: Constraint[], fresh: boolean,
                               alt: DimAlt | null): boolean {
  endDimension(v, false);
  if (!targets.length) return false;
  const before = v.source;
  const impliedBefore = new Set(v.diagnosis?.implied ?? []);
  // the record goes in first: stating the constraint is part of the gesture, so it must not
  // solve or diagnose either — the sketch it lands in is the one the pointer is still
  // choosing over
  v.liveDim = { targets, fresh, alt, before, impliedBefore, placing: fresh };
  if (fresh && !applyConstraints(v, ...targets).length) {
    v.liveDim = null;
    return false;
  }
  v.litConstraint = targets[0];
  if (v.liveDim.placing) {
    v.onStatus('place the dimension, then type its number — Enter to accept, Esc to take '
                + 'it back');
  }
  if (v.liveDim.placing) moveDimension(v, v.cursor);
  v.draw();
  return true;
}

/** The number follows the pointer while it is being placed, and on a point pair it changes
 *  what it says as it goes: a dimension line stands off across what it measures, so putting
 *  the number above a pair asks for the run and out to the side asks for the rise.  The rule
 *  is the core's — the same one that then draws the figure. */
export function moveDimension(v: SketchView, sp: [number, number]): void {
  const live = v.liveDim;
  if (!live) return;
  const at = v.s2w(sp[0], sp[1]);
  if (live.alt) retarget(v, live, at);
  const c = live.targets[0];
  if (c.id >= 0) dim.drag(v.sketch, c.id, at[0], at[1], [0, 0]);
}

/** Say the same thing as a different kind of dimension, because the number was put where that
 *  is what it means.  The old one goes and the new one takes its place: it is the same edit,
 *  still unfinished, so it makes no undo entry of its own. */
export function retarget(v: SketchView, live: LiveDim, at: [number, number]): void {
  const { a, b, make } = live.alt!;
  const want = dim.pairDimension([a.x.value, a.y.value], [b.x.value, b.y.value], at);
  const was = live.targets[0];
  if (was.typeName === want) return;
  v.sketch.remove(was);
  const c = make(want);
  v.sketch.add(c);
  live.targets[0] = c;
  v.litConstraint = c;
  v.afterEdit();          // a different constraint: the list, the DOF and the diagnosis move
}

/** The click that plants it: the number stops following the pointer and stays where it was
 *  put, a placement like any other — and *now* the sketch is solved, once, because now there
 *  is something settled to solve.  What it says is still open: the editor stays up until the
 *  number is accepted. */
export function placeDimension(v: SketchView): void {
  const live = v.liveDim;
  if (!live?.placing) return;
  live.placing = false;
  const res = v.afterEdit();
  if (live.fresh) reportAdded(v, live.targets, 0, live.impliedBefore, res);
}

/** Done writing.  Accepted, what was there before goes on the undo stack, so the constraint,
 *  where it was put and what it says are one step back together; refused, the constraints that
 *  were added to say it come out again and nothing happened at all — a number that was only
 *  being *edited* stays where it was found, since refusing an edit must not delete it. */
export function endDimension(v: SketchView, commit: boolean): void {
  const live = v.liveDim;
  if (!live) return;
  v.liveDim = null;
  v.litConstraint = null;
  if (commit) {
    v.pushUndo(live.before);
  } else if (live.fresh) {
    for (const c of live.targets) v.sketch.remove(c);
  }
  v.afterEdit();
  v.onDimension(null, null);
}

/** Where the live dimension's number was just painted, so the shell can put its editor on
 *  it.  Told off the paint's own layout rather than one of its own: the editor then sits
 *  exactly where the label is, and a placement gesture lays the callouts out once a frame
 *  instead of twice.  Clamped into the canvas — a callout can be laid out off the edge, and
 *  an editor nobody can see is worse than one a little out of place. */
export function tellDimension(v: SketchView, items: Callout[]): void {
  const live = v.liveDim;
  if (!live) return;
  const k = items.find((i) => i.id === live.targets[0].id);
  if (!k) return v.onDimension(live, null);
  const [x, y] = v.w2s(k.anchor[0], k.anchor[1]);
  const pad = 24;
  v.onDimension(live, [Math.min(Math.max(x, pad), v.width - pad),
                       Math.min(Math.max(y, pad), v.height - pad)]);
}
