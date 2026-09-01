/* Editing the document: adding and removing constraints, and what the selection can be told to
 * do — delete, mark as reference geometry, copy, cut, paste, fix.  Each is one thing the user did, so
 * each is one undo entry, one solve and one diagnosis, and each says what came of it. */
import * as io from '../core/io.js';
import * as dim from '../core/callout.js';
import { Constraint, sameConstraint } from '../core/constraints.js';
import { Arc, Circle, Ellipse, Line, Point, Spline } from '../core/model.js';
import { SolveResult } from '../core/system.js';
import type { SketchView } from './view.js';

/** How far a paste lands from what it was copied from, in screen px — far enough to see, near
 *  enough to drag.  Screen-constant, so a paste looks the same at any zoom, and successive
 *  pastes cascade rather than pile up. */
const PASTE_PX = 24;

/** Add one or more constraints as a single edit: one undo entry, one solve, one
 *  diagnosis.  A multi-entity action (an equality set, Horizontal over several lines)
 *  is one thing the user did, so it should take one Ctrl+Z to undo. */
export function addConstraints(v: SketchView, ...cs: Constraint[]): void {
  if (!cs.length || !v.mayEdit()) return;
  v.pushUndo();
  // nothing added — every one a repeat, or the core refused it — is nothing to come back from
  if (!applyConstraints(v, ...cs).length) v.dropUndo();
}

/** The same, without the undo entry: for an edit that is still being made, and so does not
 *  know yet whether it will come to anything.  Returns the ones that were actually added.
 *
 *  A *relation* that repeats one the sketch already has is dropped: it says nothing the sketch
 *  does not already say, and it adds equations without adding rank, which the structural check
 *  cannot see — so it lurks until an unrelated edit tips its block into a spurious
 *  over-constrained report, a long way from the click that caused it.
 *
 *  A *dimension* is not dropped, even an identical one.  It states a number, and whether a
 *  second number on the same feature is redundant or a contradiction is what the solve and the
 *  diagnosis are for: it comes back as `over` with both named, which is something the drawing
 *  can show and the user can act on.  Refusing it here would decide that quietly instead. */
export function applyConstraints(v: SketchView, ...cs: Constraint[]): Constraint[] {
  if (!cs.length) return [];
  const have = v.sketch.userConstraints();
  const fresh: Constraint[] = [];
  for (const c of cs) {                        // ...against this batch too, not just the sketch
    const dup = !c.dimensions().length
      && (have.some((e) => sameConstraint(e, c)) || fresh.some((e) => sameConstraint(e, c)));
    if (!dup) fresh.push(c);
  }
  if (!fresh.length) {
    const kinds = [...new Set(cs.map((c) => c.typeName))].join(' + ');
    v.onStatus(`${kinds} is already on this selection — nothing added`);
    return [];
  }
  const skipped = cs.length - fresh.length;
  cs = fresh;
  const impliedBefore = new Set(v.diagnosis?.implied ?? []);
  // the core may refuse one outright — a `project` between two points of one view, or of a
  // point on no view — and its message is the whole of what there is to say.  Whatever of the
  // batch was bound before the refusal comes back out, so a refused edit is no edit at all.
  try {
    v.sketch.add(...cs);
  } catch (err) {
    for (const c of cs) if (c.id >= 0 && c.sketch === v.sketch) v.sketch.remove(c);
    v.onStatus((err as Error).message);
    return [];
  }
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
    ? ` — CONFLICT, remove one of: ${d.conflicts.map((k) => io.describe(k, v.doc)).join(', ')}`
    : st === 'over' ? ' — redundant (consistent) with existing constraints'
    : res && !res.success ? ' — solver did NOT converge'
    : newlyImplied.length
    ? ` — consistent; ${newlyImplied.map((k) => io.describe(k, v.doc)).join(', ')} now follow from the rest`
    : '';
  v.onStatus(`added ${what}${dup}${why}`);
}

export function removeConstraint(v: SketchView, c: Constraint): void {
  if (!v.mayEdit()) return;
  v.pushUndo();
  v.sketch.remove(c);
  v.afterEdit();
}
/** Delete what is selected — as a **source edit**: the statements that declare them go, and every
 *  statement that named one goes with them, which is the rule `io::without` follows on a sketch
 *  said about text instead.  A rebuild would have been simpler and would have flattened the
 *  document to a list of points on the first deletion; this leaves everything else written.
 *
 *  Refused when what is selected came out of a component: there is no one statement to delete, and
 *  taking the component out is a far larger edit than the gesture asked for. */
export function deleteSelected(v: SketchView): void {
  // the traced picture, when that is what is selected — which is the only time it can be, the
  // two selections being exclusive.  It is not in the document, so this is not an `apply`:
  // nothing is spliced, nothing is solved and there is nothing for undo to come back to.
  if (v.underlay?.picked) {
    v.removeImage();          // which says so itself, so the menu item and this agree
    return;
  }
  if (!v.selected.length) return;
  const n = v.selected.length;
  if (v.apply(v.doc.remove(v.selected), `deleted ${n} entities`)) v.onSelect();
}

/** Flip reference/normal on the selected lines, circles, arcs and ellipses. */
export function toggleConstructionSelected(v: SketchView): void {
  if (!v.mayEdit()) return;
  const ents = v.selected.filter(
    (e): e is Line | Circle | Arc | Ellipse =>
      e instanceof Line || e instanceof Circle || e instanceof Arc || e instanceof Ellipse,
  );
  if (!ents.length) {
    v.onStatus('select line(s), circle(s), arc(s) or ellipse(s) to toggle reference geometry');
    return;
  }
  v.pushUndo();
  // `construction` is a *class* now, and the base sheet is what draws it dashed: the toggle
  // sets and clears the name, and knows nothing about what it looks like
  const all = ents.every((e) => e.hasClass('construction'));
  for (const e of ents) e.setClass('construction', !all);
  v.onStatus(`${ents.length} entit${ents.length === 1 ? 'y' : 'ies'} `
    + `${all ? 'back to normal geometry' : 'marked as construction'}`);
  v.syncSource();      // a flag is document state, so the source has to say so
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
  if (!v.mayEdit()) return 0;
  const clip = v.clipboard;
  if (!clip?.primitives().length) return 0;
  v.pushUndo();
  const d = v.world(PASTE_PX * ++v.pastes);
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
  if (!v.mayEdit()) return 0;
  const cs = c ? [c] : v.sketch.userConstraints();
  const before = v.source;
  const n = cs.filter((k) => dim.reset(v.sketch, k.id)).length;
  if (!n) return 0;             // nothing was out of place: no edit, and no history entry
  v.pushUndo(before);
  v.draw();
  return n;
}

export function toggleFixSelected(v: SketchView): void {
  if (!v.mayEdit()) return;
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
