/* Editing the document: adding and removing constraints, and what the selection can be told to
 * do — delete, mark as construction, copy, cut, paste, fix.  Each is one thing the user did, so
 * each is one undo entry, one solve and one diagnosis, and each says what came of it. */
import * as io from '../core/io.js';
import * as dim from '../core/callout.js';
import { Constraint, sameConstraint } from '../core/constraints.js';
import { Arc, Circle, Line, Point, Spline } from '../core/model.js';
import { SolveResult } from '../core/system.js';
import type { SketchView } from './view.js';

/** How far a paste lands from what it was copied from, in screen px — far enough to see, near
 *  enough to drag.  Screen-constant, so a paste looks the same at any zoom, and successive
 *  pastes cascade rather than pile up. */
const PASTE_PX = 24;

/** Add one or more constraints as a single edit: one undo entry, one solve, one
 *  diagnosis.  A multi-entity action (an equality set, Horizontal over several lines)
 *  is one thing the user did, so it should take one Ctrl+Z to undo. */
/** Add constraints, silently dropping any that repeat one the sketch already has.
 *
 *  A duplicate is pure cost: it adds equations without adding rank, and the structural
 *  check cannot see that, so it lurks until an unrelated edit tips its block into a
 *  spurious over-constrained report — a long way from the click that caused it. */
export function addConstraints(v: SketchView, ...cs: Constraint[]): void {
  if (cs.length) v.pushUndo();
  applyConstraints(v, ...cs);
}

/** The same, without the undo entry: for an edit that is still being made, and so does not
 *  know yet whether it will come to anything.  Returns the ones that were actually added. */
export function applyConstraints(v: SketchView, ...cs: Constraint[]): Constraint[] {
  if (!cs.length) return [];
  const have = v.sketch.userConstraints();
  const fresh: Constraint[] = [];
  for (const c of cs) {                        // ...against this batch too, not just the sketch
    if (!have.some((e) => sameConstraint(e, c)) && !fresh.some((e) => sameConstraint(e, c))) {
      fresh.push(c);
    }
  }
  if (!fresh.length) {
    const kinds = [...new Set(cs.map((c) => c.typeName))].join(' + ');
    v.onStatus(`${kinds} is already on this selection — nothing added`);
    return [];
  }
  const skipped = cs.length - fresh.length;
  cs = fresh;
  const impliedBefore = new Set(v.diagnosis?.implied ?? []);
  v.sketch.add(...cs);
  const res = v.afterEdit();
  // a dimension still being carried has not been judged — `afterEdit` did not diagnose it —
  // so what it came to is reported when it lands instead of now
  if (!v.liveDim?.placing) reportAdded(v, cs, skipped, impliedBefore, res);
  return cs;
}

/** What the sketch made of what was just added.  Said once, from a fresh diagnosis: which is
 *  why a dimension being placed waits until it lands to hear it. */
export function reportAdded(v: SketchView, cs: Constraint[], skipped: number,
                            impliedBefore: Set<Constraint>, res: SolveResult | null): void {
  const d = v.diagnosis;
  const st = d?.status ?? 'well';
  const kinds = [...new Set(cs.map((c) => c.typeName))].join(' + ');
  const what = cs.length === 1 ? kinds : `${cs.length} × ${kinds}`;
  const dup = skipped ? ` (${skipped} duplicate${skipped > 1 ? 's' : ''} skipped)` : '';
  // a new theorem is worth a word (the user may not have seen it coming), in the register of
  // a remark rather than a warning: the sketch is consistent and nothing needs doing
  const newlyImplied = (d?.implied ?? []).filter((k) => !impliedBefore.has(k));
  const why = st === 'conflict' && d?.conflicts?.length
    ? ` — CONFLICT, remove one of: ${d.conflicts.map((k) => io.describe(k, v.sketch)).join(', ')}`
    : st === 'over' ? ' — redundant (consistent) with existing constraints'
    : res && !res.success ? ' — solver did NOT converge'
    : newlyImplied.length
    ? ` — consistent; ${newlyImplied.map((k) => io.describe(k, v.sketch)).join(', ')} now follow from the rest`
    : '';
  v.onStatus(`added ${what}${dup}${why}`);
}

export function removeConstraint(v: SketchView, c: Constraint): void {
  v.pushUndo();
  v.sketch.remove(c);
  v.afterEdit();
}
export function deleteSelected(v: SketchView): void {
  if (!v.selected.length) return;
  v.pushUndo();
  const n = v.selected.length;
  v.setSketch(io.without(v.sketch, v.selected), false);
  v.onStatus(`deleted ${n} entities`);
}

/** Flip reference/normal on the selected lines, circles and arcs. */
export function toggleConstructionSelected(v: SketchView): void {
  const ents = v.selected.filter(
    (e): e is Line | Circle | Arc => e instanceof Line || e instanceof Circle || e instanceof Arc,
  );
  if (!ents.length) {
    v.onStatus('select line(s), circle(s) or arc(s) to toggle construction geometry');
    return;
  }
  v.pushUndo();
  const all = ents.every((e) => e.construction);
  for (const e of ents) e.construction = !all;
  v.onStatus(`${ents.length} entit${ents.length === 1 ? 'y' : 'ies'} `
    + `${all ? 'back to normal geometry' : 'marked as construction'}`);
  v.onChanged();
  v.draw();
}

/** Copy the selection.  Returns what went onto the clipboard, so the caller can say so; a
 *  selection nothing came of leaves the previous clipboard alone. */
export function copySelected(v: SketchView): number {
  if (!v.selected.length) return 0;
  const clip = io.copy(v.sketch, v.selected);
  const n = clip.primitives().length;
  if (!n) {
    clip.dispose();
    return 0;
  }
  v.clipboard?.dispose();
  v.clipboard = clip;
  v.pastes = 0;
  return n;
}

/** Copy the selection and take it out of the sketch. */
export function cutSelected(v: SketchView): number {
  const n = copySelected(v);
  if (n) deleteSelected(v);
  return n;
}

/** Paste the clipboard, nudged clear of whatever it landed on and left selected, so it can be
 *  dragged where it belongs straight away.  The copy is independent: it brings its own
 *  constraints and is joined to nothing. */
export function pasteClipboard(v: SketchView): number {
  const clip = v.clipboard;
  if (!clip?.primitives().length) return 0;
  v.pushUndo();
  const d = (PASTE_PX * ++v.pastes) / v.scale;
  const made = io.paste(v.sketch, clip, d, -d);
  v.selected = made;
  v.litConstraint = null;
  v.highlight = [];
  v.releasePlan();
  v.afterEdit();
  v.onSelect();
  return made.length;
}

/** Put dimension callouts back where the layout would place them: one of them, or all of
 *  them.  A drawing that has been rearranged by hand and then edited into a mess needs a way
 *  back, and this is it. */
export function resetCallouts(v: SketchView, c?: Constraint | null): number {
  const cs = c ? [c] : v.sketch.userConstraints();
  const before = io.dumps(v.sketch);
  const n = cs.filter((k) => dim.reset(v.sketch, k.id)).length;
  if (!n) return 0;             // nothing was out of place: no edit, and no history entry
  v.pushUndo(before);
  v.draw();
  return n;
}

export function toggleFixSelected(v: SketchView): void {
  const pts = v.selected.filter((e): e is Point => e instanceof Point);
  if (!pts.length) return;
  v.pushUndo();
  const allFixed = pts.every((p) => p.isFixed);
  for (const p of pts) p.fix(!allFixed);
  v.afterEdit();
}

/** Another control point on `curve`, nearest (x, y).  The curve does not move: this is knot
 *  insertion, so every contact keeps its parameter and its place, and the sketch stays solved.
 *  It is `closest` that turns the click into a parameter — the same projection the pick test
 *  used to decide the click was on this curve at all. */
export function insertControl(v: SketchView, curve: Spline, x: number, y: number): void {
  v.pushUndo();
  const made = curve.insertControl(curve.closest(x, y).t);
  if (!made) {
    v.dropUndo();
    v.onStatus('no room for another control point there');
    return;
  }
  v.selected = [made];
  v.releasePlan();
  v.afterEdit();
  v.onSelect();
  v.onStatus(`${curve.name} now has ${curve.ctrl.length} control points`);
}
