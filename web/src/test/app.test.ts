/* The app layer: `SketchView`'s gesture and animation lifecycles, against a stubbed canvas.
 *
 * These are the parts of the front end that own core handles — a drag, a compiled plan, an
 * interval — and the bugs worth a test here are the ones where a handle outlives what it was made
 * for. */
import assert from 'node:assert/strict';
import test from 'node:test';

import * as C from '../core/constraints.js';
import { Constraint } from '../core/constraints.js';
import * as examples from '../core/examples.js';
import * as io from '../core/io.js';
import { Sketch } from '../core/model.js';
import { Document, fromSketch } from '../core/program.js';
import type { Diagnosis } from '../core/diagnose.js';
import { callouts } from '../core/callout.js';
import { PlanDrag } from '../core/decompose.js';
import { solve } from '../core/system.js';
import { DimAlt, SketchView } from '../app/view.js';
import { initCore } from '../core/wasm.js';
import { fakeCanvas, pointer } from './canvas.js';

// the view schedules its repaints; nothing is being looked at, so run them inline
(globalThis as { requestAnimationFrame?: unknown }).requestAnimationFrame ??=
  (fn: FrameRequestCallback) => { fn(0); return 0; };
(globalThis as { cancelAnimationFrame?: unknown }).cancelAnimationFrame ??= () => {};

await initCore();

/** A fixed base with one free apex, over-determined once the apex is pinned — so a drag on it
 *  takes the numeric path, with a core handle of its own to keep alive and to free. */
function pinnedApex(): Sketch {
  const sk = new Sketch();
  const a = sk.point(0, 0, true), b = sk.point(10, 0, true), c = sk.point(5, 4);
  sk.add(new C.Distance(a, c, 6.4), new C.Distance(b, c, 6.4));
  return sk;
}

/** A view on a sketch built by hand — lifted into the program it is written as, because that is
 *  what a view holds now.  What the view draws is the *elaboration*, a different sketch with the
 *  same numbers in the same order, so a test may still measure through the one it built but must
 *  compare identity against `view.sketch`. */
function viewOn(sk: Sketch): SketchView {
  const view = new SketchView(fakeCanvas(), Document.read(fromSketch(sk)));
  view.autoSolve = false;
  return view;
}


test('a second pointer does not take over a live drag', () => {
  const sk = pinnedApex();
  const view = viewOn(sk);
  const cv = view.canvas as ReturnType<typeof fakeCanvas>;
  const apex = view.sketch.points[2];
  const [sx, sy] = view.w2s(...apex.xy);

  cv.fire('pointerdown', pointer(sx, sy, { pointerId: 1 }));
  assert.deepEqual(view.selected, [apex]);
  assert.equal(PlanDrag.live, 1, 'the drag should have a live handle');

  // a second finger, far from anything: on its own that would clear the selection and start a
  // rubber band, dropping the live drag with its core handle and its target still in the sketch
  cv.fire('pointerdown', pointer(sx + 300, sy + 300, { pointerId: 2 }));
  cv.fire('pointermove', pointer(sx + 320, sy + 320, { pointerId: 2 }));
  cv.fire('pointerup', pointer(sx + 320, sy + 320, { pointerId: 2 }));
  assert.deepEqual(view.selected, [apex], 'the second pointer took over');
  assert.equal(PlanDrag.live, 1, 'the first drag was dropped without ending');

  cv.fire('pointerup', pointer(sx, sy, { pointerId: 1 }));
  assert.equal(PlanDrag.live, 0, 'ending the drag has to free its handle');
});

test('a cancelled pointer ends the drag it owned', () => {
  const sk = pinnedApex();
  const view = viewOn(sk);
  const cv = view.canvas as ReturnType<typeof fakeCanvas>;
  const [sx, sy] = view.w2s(...sk.points[2].xy);

  cv.fire('pointerdown', pointer(sx, sy, { pointerId: 1 }));
  assert.equal(PlanDrag.live, 1);
  cv.fire('pointercancel', pointer(sx, sy, { pointerId: 1 }));
  assert.equal(PlanDrag.live, 0, 'a cancelled touch left the drag behind');

  // and the view is usable afterwards: a fresh press starts a fresh drag
  cv.fire('pointerdown', pointer(sx, sy, { pointerId: 3 }));
  assert.equal(PlanDrag.live, 1);
  cv.fire('pointerup', pointer(sx, sy, { pointerId: 3 }));
  assert.equal(PlanDrag.live, 0);
});

test('losing pointer capture ends the drag too', () => {
  const sk = pinnedApex();
  const view = viewOn(sk);
  const cv = view.canvas as ReturnType<typeof fakeCanvas>;
  const [sx, sy] = view.w2s(...sk.points[2].xy);

  cv.fire('pointerdown', pointer(sx, sy, { pointerId: 1 }));
  assert.equal(PlanDrag.live, 1);
  cv.fire('lostpointercapture', pointer(sx, sy, { pointerId: 1 }));
  assert.equal(PlanDrag.live, 0);
});

/* -- dimension callouts ---------------------------------------------------------------- */

/** A dimensioned span, drawn: two points 60 apart with a Distance on them. */
function dimensioned(): Sketch {
  const sk = new Sketch();
  const a = sk.point(0, 0, true), b = sk.point(60, 0);
  sk.add(new C.Distance(a, b, 60));
  return sk;
}

test('the drawing calls out every dimension it has', () => {
  const sk = dimensioned();
  const view = viewOn(sk);
  view.draw();                     // paint them too, so the painter is exercised as well
  const cs = callouts(sk, view.unit);
  assert.equal(cs.items.length, 1);
  assert.equal(cs.items[0].text, '60');
  assert.equal(cs.items[0].id, view.sketch.userConstraints()[0].id);
  assert.ok(cs.items[0].solid.length >= 1 && cs.items[0].arrows.length === 2);
  assert.ok(cs.font > 0 && cs.arrow > 0 && cs.barb > 0, 'the drawing style comes from the core');
});

test('clicking a callout picks its constraint instead of the geometry', () => {
  const sk = dimensioned();
  const view = viewOn(sk);
  const cv = view.canvas as ReturnType<typeof fakeCanvas>;
  view.draw();
  const picked: Constraint[] = [];
  view.onPickConstraint = (c) => picked.push(c);

  const [ax, ay] = view.w2s(...sk.points[0].xy);
  cv.fire('pointerdown', pointer(ax, ay - 30));   // on the dimension line, above the span
  cv.fire('pointerup', pointer(ax, ay - 30));
  assert.deepEqual(picked, [view.sketch.userConstraints()[0]]);
  assert.deepEqual(view.selected, [], 'a dimension is not part of the geometry selection');
});

test('a click that misses every callout still reaches the sketch', () => {
  const sk = dimensioned();
  const view = viewOn(sk);
  const cv = view.canvas as ReturnType<typeof fakeCanvas>;
  view.draw();
  let picked = 0;
  view.onPickConstraint = () => { picked += 1; };

  const [bx, by] = view.w2s(...sk.points[1].xy);
  cv.fire('pointerdown', pointer(bx, by));
  cv.fire('pointerup', pointer(bx, by));
  assert.equal(picked, 0);
  assert.deepEqual(view.selected, [view.sketch.points[1]]);
});

test("a radius's leader does not shadow the centre it comes out of", () => {
  // the figure runs from the centre to the rim, so the one point a circle has lies on the
  // callout: a press there means the point, which is the thing the next constraint is about
  const built = new Sketch();
  const circle0 = built.circle(built.point(0, 0), 20);
  built.add(new C.Radius(circle0, 20));
  const view = viewOn(built);
  const c = view.sketch.points[0];
  const cv = view.canvas as ReturnType<typeof fakeCanvas>;
  view.draw();
  let picked = 0;
  view.onPickConstraint = () => { picked += 1; };

  const [cx, cy] = view.w2s(...c.xy);
  assert.equal(view.pickCallout(cx, cy), null, 'the callout claimed the centre');
  cv.fire('pointerdown', pointer(cx, cy));
  cv.fire('pointerup', pointer(cx, cy));
  assert.equal(picked, 0);
  assert.deepEqual(view.selected, [c]);

  // and the callout is still there to be taken hold of, out along the leader
  const k = callouts(view.sketch, view.unit).items[0];
  assert.ok(view.pickCallout(...view.w2s(k.arrows[0].at[0], k.arrows[0].at[1])));
});

test('double-clicking a callout opens its value', () => {
  const sk = dimensioned();
  const view = viewOn(sk);
  const cv = view.canvas as ReturnType<typeof fakeCanvas>;
  view.draw();
  const edited: Constraint[] = [];
  view.onEditConstraint = (c) => edited.push(c);

  const [ax, ay] = view.w2s(...sk.points[0].xy);
  cv.fire('dblclick', { clientX: ax, clientY: ay - 30 });
  assert.deepEqual(edited, [view.sketch.userConstraints()[0]]);
  cv.fire('dblclick', { clientX: ax, clientY: ay + 200 });
  assert.equal(edited.length, 1, 'a double-click on empty canvas edits nothing');
});

test('switching the dimensions off leaves nothing to click on', () => {
  const sk = dimensioned();
  const view = viewOn(sk);
  view.showDimensions = false;
  view.draw();
  const [ax, ay] = view.w2s(...sk.points[0].xy);
  assert.equal(view.pickCallout(ax, ay - 30), null);
});

/** Where the dimension line sits, in screen y — what dragging a linear callout moves. */
function dimY(view: SketchView, sk: Sketch): number {
  const k = callouts(sk, view.unit).items[0];
  return view.w2s(k.solid[0][0][0], k.solid[0][0][1])[1];
}

test('dragging a callout moves it, and it stays moved', () => {
  const sk = dimensioned();
  const view = viewOn(sk);
  const cv = view.canvas as ReturnType<typeof fakeCanvas>;
  view.draw();
  const before = dimY(view, view.sketch);
  const [ax] = view.w2s(...view.sketch.points[0].xy);

  cv.fire('pointerdown', pointer(ax, before));      // take hold of the dimension line
  cv.fire('pointermove', pointer(ax, before - 60));
  cv.fire('pointerup', pointer(ax, before - 60));
  const after = dimY(view, view.sketch);
  assert.ok(Math.abs(after - (before - 60)) < 1, `${before} → ${after}`);

  // it is the document that remembers, so it survives a re-solve and a round trip
  view.solveNow();
  assert.ok(Math.abs(dimY(view, view.sketch) - after) < 1e-6, 'a solve moved the callout');
  const reloaded = io.loads(io.dumps(view.sketch));
  const view2 = viewOn(reloaded);
  Object.assign(view2.cam, view.cam);       // the same camera, so the same screen positions
  assert.ok(Math.abs(dimY(view2, reloaded) - after) < 1e-6, 'the placement did not save');
});

test('a callout follows the point it was grabbed at, not the pointer', () => {
  const sk = dimensioned();
  const view = viewOn(sk);
  const cv = view.canvas as ReturnType<typeof fakeCanvas>;
  view.draw();
  const before = dimY(view, view.sketch);
  const [ax] = view.w2s(...view.sketch.points[0].xy);

  // press 6px below the line, then move 40px up: the line should move 40, not 46
  cv.fire('pointerdown', pointer(ax, before + 6));
  cv.fire('pointermove', pointer(ax, before + 6 - 40));
  cv.fire('pointerup', pointer(ax, before + 6 - 40));
  assert.ok(Math.abs(dimY(view, view.sketch) - (before - 40)) < 1,
            'the callout jumped to the pointer');
});

test('a click on a callout that never moves is not an edit', () => {
  const sk = dimensioned();
  const view = viewOn(sk);
  const cv = view.canvas as ReturnType<typeof fakeCanvas>;
  view.draw();
  const before = dimY(view, sk);
  const [ax] = view.w2s(...sk.points[0].xy);

  cv.fire('pointerdown', pointer(ax, before));
  cv.fire('pointerup', pointer(ax, before));
  assert.equal(dimY(view, sk), before);
  view.undo();
  assert.equal(dimY(view, sk), before, 'the click went onto the undo stack');
});

test('re-placing puts a dragged callout back', () => {
  const sk = dimensioned();
  const view = viewOn(sk);
  const cv = view.canvas as ReturnType<typeof fakeCanvas>;
  view.draw();
  const before = dimY(view, view.sketch);
  const [ax] = view.w2s(...view.sketch.points[0].xy);
  cv.fire('pointerdown', pointer(ax, before));
  cv.fire('pointermove', pointer(ax, before - 60));
  cv.fire('pointerup', pointer(ax, before - 60));
  assert.notEqual(dimY(view, view.sketch), before);

  // the drag is document state, so it reached the source rather than living in the sketch alone
  assert.ok(/at \(/.test(view.source), `the drag was written down: ${view.source}`);
  const dragged = dimY(view, view.sketch);
  view.setProgram(view.source, false);
  assert.ok(Math.abs(dimY(view, view.sketch) - dragged) < 1e-9, 'and survives a re-elaboration');

  view.resetCallouts();
  assert.ok(Math.abs(dimY(view, view.sketch) - before) < 1e-9);

  // and undoing it keeps the drawing: the snapshot it takes is program text, so undo restores a
  // document — where a serialised sketch would come back from `Document.read` as an empty one
  const points = view.sketch.points.length;
  view.undo();
  assert.equal(view.sketch.points.length, points, 'undo kept the drawing');
  assert.ok(view.source.startsWith('point'), 'undo restored the source, not a dump');
});

/* -- copy and paste -------------------------------------------------------------------- */

/** Two points and a line between them, with a length on it. */
function oneLine(): Sketch {
  const sk = new Sketch();
  const a = sk.point(0, 0, true), b = sk.point(60, 0);
  sk.line(a, b);
  sk.add(new C.Distance(a, b, 60));
  return sk;
}

test('copying a line takes its points and its dimension', () => {
  const sk = oneLine();
  const view = viewOn(sk);
  view.selected = [view.sketch.lines[0]];
  assert.equal(view.copySelected(), 3, 'the line and its two ends');

  assert.equal(view.pasteClipboard(), 3);
  assert.equal(view.sketch.points.length, 4);
  assert.equal(view.sketch.lines.length, 2);
  assert.equal(view.sketch.userConstraints().length, 2, 'the copy brought its own Distance');
});

test('a paste is selected, and lands clear of what it came from', () => {
  const sk = oneLine();
  const view = viewOn(sk);
  view.selected = [view.sketch.lines[0]];
  view.copySelected();
  view.pasteClipboard();

  assert.deepEqual(view.selected, [view.sketch.points[2], view.sketch.points[3], view.sketch.lines[1]]);
  const [x0, y0] = view.sketch.points[0].xy;
  const [x1, y1] = view.sketch.points[2].xy;
  assert.ok(x1 > x0 && y1 < y0, `the copy should be nudged clear: ${x1},${y1} vs ${x0},${y0}`);
});

test('successive pastes cascade instead of piling up', () => {
  const sk = oneLine();
  const view = viewOn(sk);
  view.selected = [view.sketch.lines[0]];
  view.copySelected();
  view.pasteClipboard();
  const first = view.sketch.points[2].xy;
  view.pasteClipboard();
  const second = view.sketch.points[4].xy;
  assert.notDeepEqual(second, first, 'the second paste landed on the first');
  assert.ok(second[0] > first[0]);
});

test('a pasted copy is independent of the original', () => {
  const sk = oneLine();
  const view = viewOn(sk);
  view.selected = [view.sketch.lines[0]];
  view.copySelected();
  view.pasteClipboard();

  // the pasted Distance names the pasted points and nothing else
  const pasted = view.sketch.userConstraints()[1];
  assert.deepEqual(pasted.entities(), [view.sketch.points[2], view.sketch.points[3]]);
});

test('copying nothing leaves the clipboard as it was', () => {
  const sk = oneLine();
  const view = viewOn(sk);
  view.selected = [view.sketch.lines[0]];
  assert.equal(view.copySelected(), 3);
  view.selected = [];
  assert.equal(view.copySelected(), 0, 'an empty selection is not a copy');
  assert.equal(view.pasteClipboard(), 3, 'the earlier copy should still be there');
});

test('pasting with an empty clipboard changes nothing', () => {
  const sk = oneLine();
  const view = viewOn(sk);
  const before = io.dumps(sk);
  assert.equal(view.pasteClipboard(), 0);
  assert.equal(io.dumps(sk), before);
  view.undo();
  assert.equal(io.dumps(sk), before, 'the no-op went onto the undo stack');
});

test('a paste undoes in one step', () => {
  const sk = oneLine();
  const view = viewOn(sk);
  const before = io.dumps(sk);
  view.selected = [view.sketch.lines[0]];
  view.copySelected();
  view.pasteClipboard();
  assert.notEqual(io.dumps(view.sketch), before);
  view.undo();
  assert.equal(io.dumps(view.sketch), before);
});

test('cut takes the selection out and keeps it', () => {
  const sk = oneLine();
  const view = viewOn(sk);
  view.selected = [view.sketch.lines[0]];
  assert.equal(view.cutSelected(), 3);
  assert.equal(view.sketch.lines.length, 0, 'the line should be gone');
  assert.equal(view.pasteClipboard(), 3, 'and still on the clipboard');
  assert.equal(view.sketch.lines.length, 1);
});

test('the clipboard outlives the sketch it came from', () => {
  const sk = oneLine();
  const view = viewOn(sk);
  view.selected = [view.sketch.lines[0]];
  view.copySelected();
  view.setSketch(new Sketch());          // a fresh sheet
  assert.equal(view.pasteClipboard(), 3);
  assert.equal(view.sketch.lines.length, 1);
  assert.equal(view.sketch.userConstraints().length, 1);
});

/* -- the spline fit tool ------------------------------------------------------- */

/** Click the fit tool at each screen position, then finish. */
function fitThrough(view: SketchView, at: [number, number][]): void {
  const cv = view.canvas as ReturnType<typeof fakeCanvas>;
  view.setTool('splinefit');
  for (const [x, y] of at) {
    cv.fire('pointerdown', pointer(x, y));
    cv.fire('pointerup', pointer(x, y));
  }
  view.finishSplineFit();
}

test('a curve fitted through free clicks leaves no points behind and no constraints', () => {
  const view = viewOn(new Sketch());
  fitThrough(view, [[100, 100], [200, 60], [300, 160], [400, 80], [500, 140]]);
  const sk = view.sketch;
  assert.equal(view.sketch.splines.length, 1);
  assert.equal(view.sketch.points.length, 5, 'the control polygon, and nothing else');
  assert.deepEqual(view.sketch.points.map((p) => p.index), view.sketch.splines[0].ctrl.map((p) => p.index));
  assert.equal(view.sketch.userConstraints().length, 0, 'a free click is a place, not a promise');
});

test('a fit click that lands on a point holds the curve to it', () => {
  const sk = new Sketch();
  const view = viewOn(sk);
  // two points already in the sketch, at screen positions the tool will snap to
  const [ax, ay] = view.s2w(200, 60);
  const [bx, by] = view.s2w(400, 80);
  const a = view.sketch.point(ax, ay), b = view.sketch.point(bx, by);
  fitThrough(view, [[100, 100], [200, 60], [300, 160], [400, 80], [500, 140]]);

  assert.equal(view.sketch.splines.length, 1);
  const curve = view.sketch.splines[0];
  const held = view.sketch.userConstraints().filter((c) => c.typeName === 'PointOnSpline');
  assert.equal(held.length, 2, 'both snapped clicks became constraints');
  assert.deepEqual(held.map((c) => (c.args[0] as { index: number }).index).sort(),
                   [a.index, b.index].sort());
  // the snapped points are not control points: they were already in the sketch
  assert.equal(curve.ctrl.length, 5);
  assert.ok(!curve.ctrl.some((p) => p === a || p === b));
  // and the curve already passes through them, so the constraints hold with nothing to solve
  for (const p of [a, b]) assert.ok(curve.closest(p.x.value, p.y.value).distance < 1e-9);
  assert.ok(solve(sk).success);
  for (const p of [a, b]) assert.ok(curve.closest(p.x.value, p.y.value).distance < 1e-9);
});

test('an abandoned fit leaves the sketch untouched', () => {
  const sk = new Sketch();
  const view = viewOn(sk);
  const before = io.dumps(sk);
  const cv = view.canvas as ReturnType<typeof fakeCanvas>;
  view.setTool('splinefit');
  for (const [x, y] of [[100, 100], [200, 60], [300, 160]] as [number, number][]) {
    cv.fire('pointerdown', pointer(x, y));
    cv.fire('pointerup', pointer(x, y));
  }
  view.cancelTool();
  assert.equal(io.dumps(view.sketch), before, 'the tool left something behind');
});

/* -- dimensioning something already dimensioned -------------------------------- */

test('the core says which constraint states the same relation', () => {
  // a question the core answers; the front end no longer asks it before writing a dimension,
  // since what a second number on a pair comes to is the diagnosis's reading and not a button's
  const sk = new Sketch();
  const a = sk.point(0, 0), b = sk.point(10, 0);
  const first = new C.Distance(a, b, 80);
  sk.add(first);
  assert.equal(C.stating(sk, new C.Distance(a, b, 60)), first, 'a different number, one relation');
  assert.equal(C.stating(sk, new C.Distance(b, a, 60)), first, 'and either way round');
  assert.equal(C.stating(sk, new C.Distance(a, sk.point(0, 10), 80)), null, 'a different pair');
  assert.equal(C.stating(sk, new C.Horizontal(sk.line(a, b))), null, 'a different type');
  sk.dispose();
});

test('a repeated relation is dropped, a repeated dimension is not', () => {
  const built = new Sketch();
  built.line(built.point(0, 0, true), built.point(60, 0));
  const view = viewOn(built);
  const [a, b] = view.sketch.points;
  const line = view.sketch.lines[0];
  view.addConstraints(new C.Horizontal(line));
  view.addConstraints(new C.Horizontal(line));
  assert.equal(view.sketch.userConstraints().length, 1, 'the same relation twice says nothing new');

  // the same number twice is a claim about the drawing, so it is written and then judged
  const d1 = new C.Distance(a, b, 60), d2 = new C.Distance(a, b, 60);
  view.addConstraints(d1);
  view.addConstraints(d2);
  assert.equal(view.sketch.userConstraints().length, 3, 'the second dimension was refused');
  assert.equal(view.diagnosis?.status, 'over', 'nobody said the sketch was over-constrained');
  const over = view.diagnosis?.over ?? [];
  assert.ok(over.includes(d1) && over.includes(d2), 'and did not name what to choose between');
});

/* -- writing a dimension --------------------------------------------------------------- */

/** Two free points on a diagonal, and the three dimensions they could take — the same
 *  alternatives the Dimension button builds. */
function pairToDimension(): { view: SketchView; sk: Sketch; alt: DimAlt } {
  const built = new Sketch();
  built.point(0, 0, true);
  built.point(40, 40);
  const view = viewOn(built);
  const sk = view.sketch;
  const [a, b] = sk.points;
  const make = (kind: string): Constraint => {
    const v = kind === 'HorizontalDistance' ? b.x.value - a.x.value
            : kind === 'VerticalDistance' ? b.y.value - a.y.value
            : Math.hypot(b.x.value - a.x.value, b.y.value - a.y.value);
    return C.build(kind, [a, b, v]);
  };
  return { view, sk, alt: { a, b, make } };
}

test('where the number is put is which dimension it is', () => {
  const { view, sk, alt } = pairToDimension();
  const cv = view.canvas as ReturnType<typeof fakeCanvas>;
  const first = alt.make('Distance');
  assert.ok(view.startDimension([first], true, alt));
  assert.equal(sk.userConstraints().length, 1, 'stated at once, not after a dialog');

  const at = (dx: number, dy: number): [number, number] => view.w2s(20 + dx, 20 + dy);
  const kind = (): string => view.liveDim!.targets[0].typeName;
  cv.fire('pointermove', pointer(...at(-30, 30)));      // across the pair: its own length
  assert.equal(kind(), 'Distance');
  cv.fire('pointermove', pointer(...at(0, 40)));        // above it: the run
  assert.equal(kind(), 'HorizontalDistance');
  assert.equal(view.sketch.userConstraints().length, 1, 'the old one should have gone');
  assert.equal((view.liveDim!.targets[0] as unknown as { d: number }).d, 40);
  cv.fire('pointermove', pointer(...at(40, 0)));        // out to the side: the rise
  assert.equal(kind(), 'VerticalDistance');

  // the number goes where it is put: the callout's placement follows the pointer
  const k = callouts(sk, view.unit).items[0];
  const [sx, sy] = view.w2s(...k.anchor);
  const [px, py] = at(40, 0);
  assert.ok(Math.hypot(sx - px, sy - py) < 30, `the callout stayed behind: ${sx}, ${sy}`);
});

test('a dimension being written is one edit, and Escape takes all of it back', () => {
  const { view, sk, alt } = pairToDimension();
  const cv = view.canvas as ReturnType<typeof fakeCanvas>;
  const before = io.dumps(sk);
  const first = alt.make('Distance');
  view.startDimension([first], true, alt);
  cv.fire('pointermove', pointer(...view.w2s(20, 60)));
  view.endDimension(false);
  assert.equal(sk.userConstraints().length, 0, 'the constraint should have come back out');
  assert.equal(io.dumps(sk), before, 'and its placement with it');
  assert.equal(view.liveDim, null);

  // accepted, it is one step back — the constraint, where it was put and what it says together
  const c = alt.make('Distance');
  view.startDimension([c], true, alt);
  cv.fire('pointermove', pointer(...view.w2s(20, 60)));
  view.endDimension(true);
  assert.equal(sk.userConstraints().length, 1);
  const points = view.sketch.points.length;
  view.undo();
  assert.equal(view.sketch.userConstraints().length, 0);
  // the drawing comes back, rather than the undo blanking it: the snapshot a dimension takes is
  // program text like every other, and a serialised sketch fed to `Document.read` would come
  // back as an empty document rather than as a refusal
  assert.equal(view.sketch.points.length, points, 'undo kept the drawing');
  assert.ok(view.source.startsWith('point'), `undo restored the source, not a dump: ${view.source.slice(0, 40)}`);
});

test('a click plants the number, and the pointer stops carrying it', () => {
  const { view, sk, alt } = pairToDimension();
  const cv = view.canvas as ReturnType<typeof fakeCanvas>;
  const c = alt.make('Distance');
  view.startDimension([c], true, alt);
  cv.fire('pointermove', pointer(...view.w2s(-20, 20)));
  const kind = view.liveDim!.targets[0].typeName;
  cv.fire('pointerdown', pointer(...view.w2s(-20, 20)));
  cv.fire('pointerup', pointer(...view.w2s(-20, 20)));
  assert.equal(view.liveDim!.placing, false);
  const where = callouts(sk, view.unit).items[0].anchor;

  // moving on now leaves it where it was put, and does not turn it into a different dimension
  cv.fire('pointermove', pointer(...view.w2s(20, 60)));
  assert.equal(view.liveDim!.targets[0].typeName, kind);
  assert.deepEqual(callouts(sk, view.unit).items[0].anchor, where);

  // but it is still being written, so taking hold of it goes on choosing which one it is
  const on = view.w2s(...callouts(sk, view.unit).items[0].anchor);
  cv.fire('pointerdown', pointer(...on));
  cv.fire('pointermove', pointer(...view.w2s(20, 60)));
  assert.equal(view.liveDim!.targets[0].typeName, 'HorizontalDistance');
  cv.fire('pointerup', pointer(...view.w2s(20, 60)));
  view.endDimension(true);
});

test('a pair with a length on it can still be given its run', () => {
  const { view, sk, alt } = pairToDimension();
  const cv = view.canvas as ReturnType<typeof fakeCanvas>;
  const first = alt.make('Distance');
  view.startDimension([first], true, alt);
  cv.fire('pointermove', pointer(...view.w2s(-10, 50)));    // across the pair: its own length
  view.endDimension(true);
  assert.equal(sk.userConstraints().length, 1);

  // asking again writes a second dimension rather than reopening the first, so the pointer
  // still gets to say which of the three it is — the run, which is a fact the length is not
  const second = alt.make('Distance');
  assert.ok(view.startDimension([second], true, alt), 'a second dimension was refused');
  cv.fire('pointermove', pointer(...view.w2s(20, 60)));     // above it: the run
  assert.equal(view.liveDim!.targets[0].typeName, 'HorizontalDistance');
  view.endDimension(true);
  assert.deepEqual(sk.userConstraints().map((c) => c.typeName).sort(),
                   ['Distance', 'HorizontalDistance']);
});

test('a dimension already on the drawing is opened, not stated twice', () => {
  const { view, sk, alt } = pairToDimension();
  const c = alt.make('Distance');
  view.startDimension([c], true, alt);
  view.endDimension(true);
  const there = sk.userConstraints()[0];

  // what `commands::editDimension` does: a target that is already in the sketch, nothing fresh
  assert.ok(view.startDimension([there], false, null));
  assert.equal(sk.userConstraints().length, 1);
  assert.equal(view.liveDim!.placing, false, 'an existing dimension is not being placed');
  view.endDimension(false);
  assert.equal(sk.userConstraints().length, 1, 'refusing an edit must not remove it');
});

test('nothing is solved or judged while a dimension is being laid down', () => {
  // a sketch with an unsatisfied constraint waiting in it, so any solve is visible: the free
  // point moves out to 50 the moment one runs
  const built = new Sketch();
  const a0 = built.point(0, 0, true);
  built.point(40, 40);
  const c0 = built.point(10, 0);
  built.add(new C.Distance(a0, c0, 50));
  const view = new SketchView(fakeCanvas(), Document.read(fromSketch(built)));  // auto-solve on
  const [a, b, c] = view.sketch.points;
  const cv = view.canvas as ReturnType<typeof fakeCanvas>;
  const said: string[] = [];
  view.onStatus = (m) => said.push(m);
  let refreshes = 0;
  view.onChanged = () => { refreshes += 1; };
  const still = c.xy;
  const judged = (): Diagnosis | null => view.diagnosis;
  const make = (kind: string): Constraint => C.build(kind, [a, b, kind === 'HorizontalDistance'
    ? b.x.value - a.x.value : kind === 'VerticalDistance' ? b.y.value - a.y.value
    : Math.hypot(b.x.value - a.x.value, b.y.value - a.y.value)]);

  const first = make('Distance');
  const was = judged();
  view.startDimension([first], true, { a, b, make });
  assert.deepEqual(c.xy, still, 'stating the dimension solved the sketch');
  cv.fire('pointermove', pointer(...view.w2s(20, 60)));
  cv.fire('pointermove', pointer(...view.w2s(60, 20)));
  assert.deepEqual(c.xy, still, 'carrying the number about solved the sketch');
  // nor is it judged while it is carried: no re-diagnosis, so nothing changes colour and the
  // banner does not come and go under the pointer
  assert.equal(was, judged(), 'the sketch was re-diagnosed while the number was carried');
  assert.equal(refreshes, 0, 'the shell was told to rebuild while the number was carried');
  assert.ok(!said.some((m) => m.startsWith('added ')), 'it was reported before it landed');

  // the click that plants it is when it takes effect
  cv.fire('pointerdown', pointer(...view.w2s(60, 20)));
  cv.fire('pointerup', pointer(...view.w2s(60, 20)));
  assert.ok(Math.abs(c.xy[0] - 50) < 1e-6, `the plant did not solve: ${c.xy}`);
  assert.ok(said.some((m) => m.startsWith('added ')), `what it came to was never said: ${said}`);

  // and the editor is still open: what it says has not been settled by planting it
  assert.ok(view.liveDim && !view.liveDim.placing);
  view.endDimension(true);
  built.dispose();
});

/* -- the two layers ------------------------------------------------------------------------
 *
 * The camera is the front end's whole linear algebra and the core is its whole geometry, so
 * what these check is the seam between them: a click becomes a world place and a pixel
 * tolerance becomes a world length, and the answer comes back from the core. */

test('the camera carries a length whichever way it is measured', () => {
  const view = viewOn(new Sketch());
  view.cam.zoomAt(300, 200, 1.7);            // an ordinary pan-and-zoom, not the default pose
  view.cam.panBy(-40, 25);
  const [sx, sy] = view.w2s(3, -7);
  const back = view.s2w(sx, sy);
  assert.ok(Math.hypot(back[0] - 3, back[1] + 7) < 1e-9, 'w2s and s2w are inverses');
  assert.ok(Math.abs(view.len(view.world(12)) - 12) < 1e-9, 'len and world are inverses');
  // a similarity carries lengths, which is what lets a pick tolerance travel in world units
  const [ax, ay] = view.w2s(0, 0);
  const [bx, by] = view.w2s(5, 12);
  assert.ok(Math.abs(Math.hypot(bx - ax, by - ay) - view.len(13)) < 1e-9);
  // and turns angles into the canvas's, which run the other way
  assert.equal(view.cam.dir(1, 2)[1], -2);
  assert.equal(view.cam.angle(Math.PI / 4), -Math.PI / 4);
});

test('picking measures what is drawn, and does it in the core', () => {
  const built = new Sketch();
  built.line(built.point(0, 0), built.point(10, 0));
  const view = viewOn(built);
  const b = view.sketch.points[1];
  const line = view.sketch.lines[0];
  const at = (x: number, y: number): [number, number] => view.w2s(x, y);
  assert.equal(view.pick(...at(5, 0)), line);
  assert.equal(view.pick(...at(5, 0.2)), line, 'within a few pixels of the segment');
  // the infinite line a dimension would measure to reaches out here; what is drawn does not
  assert.equal(view.pick(...at(30, 0)), null);
  // a point within reach wins over the edge it is an end of
  assert.equal(view.pick(...at(9.95, 0)), b);
  // the tolerance is in pixels, so it covers less of the drawing the further in you zoom
  const off = view.world(6);                 // six pixels off the segment
  assert.equal(view.pick(...at(5, off)), line);
  view.cam.zoomAt(0, 0, 4);
  assert.equal(view.pick(...at(5, off)), null, 'zoomed in, the same place is well clear of it');
});

/* -- the source is the document -------------------------------------------------------- */

const ANNOTATED = `\
// a base, and this comment must survive every gesture
point a at (0, 0)
point b at (100, 0)
line ab(a, b)      // the base
horizontal(ab)
ground(a)
`;

function docView(text: string): SketchView {
  const view = new SketchView(fakeCanvas(), Document.read(text));
  view.autoSolve = false;
  return view;
}

test('a gesture is a source edit, and leaves everything else written', () => {
  const view = docView(ANNOTATED);
  assert.ok(view.doc.ok, JSON.stringify(view.doc.diagnostics));
  const cv = view.canvas as ReturnType<typeof fakeCanvas>;
  view.setTool('line');
  cv.fire('pointerdown', pointer(...view.w2s(0, 60)));
  cv.fire('pointerup', pointer(...view.w2s(0, 60)));
  cv.fire('pointerdown', pointer(...view.w2s(80, 60)));
  cv.fire('pointerup', pointer(...view.w2s(80, 60)));

  assert.equal(view.sketch.lines.length, 2, 'the line was drawn');
  for (const line of ANNOTATED.split('\n').filter((l) => l.trim())) {
    assert.ok(view.source.includes(line), `the gesture rewrote: ${line}\n${view.source}`);
  }
  assert.ok(/\nline\s+l0\(p0, p1\)/.test(view.source), view.source);
  // and the source is a document: reading it back gives the same drawing
  const again = Document.read(view.source);
  assert.equal(io.dumps(again.sketch, 1), io.dumps(view.sketch, 1));
  again.dispose();
});

test('a drag writes its seeds back, once, and nothing else', () => {
  const view = docView(ANNOTATED);
  const cv = view.canvas as ReturnType<typeof fakeCanvas>;
  const b = view.sketch.points[1];
  const [sx, sy] = view.w2s(...b.xy);
  let wrote = 0;
  view.onProgram = () => { wrote += 1; };

  cv.fire('pointerdown', pointer(sx, sy));
  for (let i = 1; i <= 5; i++) cv.fire('pointermove', pointer(sx + 4 * i, sy - 4 * i));
  assert.equal(wrote, 0, 'the source was written while the pointer was down');
  const sketchDuring = view.sketch;
  cv.fire('pointerup', pointer(sx + 20, sy - 20));

  assert.equal(view.sketch, sketchDuring, 'a drag must not re-elaborate the drawing');
  assert.equal(wrote, 1, 'a drag is one edit');
  assert.ok(view.source.includes('// a base, and this comment must survive every gesture'));
  assert.ok(view.source.includes('line ab(a, b)      // the base'));
  assert.ok(!view.source.includes('point b at (100, 0)'), `the seed did not move:\n${view.source}`);
  assert.ok(/point b at \(/.test(view.source), 'and it is still a seed');
});

test('deleting takes the statements that named it, and leaves the comments', () => {
  const view = docView(ANNOTATED);
  view.selected = [view.sketch.points[1]];
  view.deleteSelected();
  assert.ok(!view.source.includes('point b at'), view.source);
  assert.ok(!view.source.includes('line ab'), 'the line that named it went too');
  assert.ok(!view.source.includes('horizontal(ab)'), 'and the constraint on that line');
  assert.ok(view.source.includes('// a base, and this comment must survive every gesture'));
  assert.ok(view.source.includes('ground(a)'));
  assert.equal(view.sketch.points.length, 1);
});

test('undo is the source, so it comes back word for word', () => {
  const view = docView(ANNOTATED);
  view.selected = [view.sketch.points[1]];
  view.deleteSelected();
  view.undo();
  assert.equal(view.source, ANNOTATED, 'undo restored a print-out instead of the document');
  assert.equal(view.sketch.points.length, 2);
});

test('a gesture beside a component leaves the component written', () => {
  const view = docView(examples.source('gear'));
  assert.ok(view.doc.ok, JSON.stringify(view.doc.diagnostics.slice(0, 3)));
  const before = view.sketch.points.length;
  const cv = view.canvas as ReturnType<typeof fakeCanvas>;
  view.setTool('point');
  cv.fire('pointerdown', pointer(...view.w2s(200, 0)));
  cv.fire('pointerup', pointer(...view.w2s(200, 0)));

  assert.equal(view.sketch.points.length, before + 1);
  assert.ok(view.source.includes('curve involute(c: circle, phase: Angle)(u) ='));
  assert.ok(view.source.includes('component Flank('));
  assert.ok(view.source.includes('cycle N as i {'));
  assert.ok(view.source.includes('g: Gear(N: 30, m: 3, phi: 25, ded: 1)'));
  assert.ok(/point\s+p0 hint at \(200, 0\)/.test(view.source), view.source);
});
