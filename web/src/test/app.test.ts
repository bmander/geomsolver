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
import { Plane, Point, Primitive, Sketch } from '../core/model.js';
import { Document, fromSketch } from '../core/program.js';
import type { Diagnosis } from '../core/diagnose.js';
import { callouts } from '../core/callout.js';
import { PlanDrag } from '../core/decompose.js';
import { solve } from '../core/system.js';
import { DimAlt, SketchView } from '../app/view.js';
import { threeViews } from '../app/tools.js';
import { contains, corners, toImage, toWorld } from '../app/underlay.js';
import type { Bitmap } from '../app/underlay.js';
import type { Item } from '../core/overview.js';
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
point a hint(x: 0, y: 0)
point b hint(x: 100, y: 0)
line ab(a, b)      // the base
horizontal ab
ground a
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
  assert.ok(!view.source.includes('point b hint(x: 100, y: 0)'), `the seed did not move:\n${view.source}`);
  assert.ok(/point b hint\(/.test(view.source), 'and it is still a seed');
});

test('deleting takes the statements that named it, and leaves the comments', () => {
  const view = docView(ANNOTATED);
  view.selected = [view.sketch.points[1]];
  view.deleteSelected();
  assert.ok(!view.source.includes('point b at'), view.source);
  assert.ok(!view.source.includes('line ab'), 'the line that named it went too');
  assert.ok(!view.source.includes('horizontal ab'), 'and the constraint on that line');
  assert.ok(view.source.includes('// a base, and this comment must survive every gesture'));
  assert.ok(view.source.includes('ground a'));
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
  assert.ok(/point\s+p0 hint\(x: 200, y: 0\)/.test(view.source), view.source);
});


/* -- views: the current plane, membership and projection ----------------------------------
 *
 * A plane is where the next point goes, and that is view state until a point is drawn — at
 * which moment it is document state, written as the point's `in` clause.  What is worth a
 * test is that seam: the membership reaches the source through the same reconcile every
 * gesture goes through, the current plane crosses a re-elaboration the way the selection
 * does, and a projection the core refuses leaves nothing behind. */

const VIEWS = `\
point o hint(x: 0, y: 0)
point q hint(x: 40, y: 0)
point o2 hint(x: 0, y: 80)
point q2 hint(x: 40, y: 80)
plane front(origin: o, toward: q)
plane top(origin: o2, toward: q2, from: front, fold: 0deg)
ground o
ground q
ground o2
ground q2
`;

function planeNamed(view: SketchView, name: string): Plane {
  const p = view.doc.entity(name);
  assert.ok(p instanceof Plane, `${name} is a plane`);
  return p;
}

function pointNamed(view: SketchView, name: string): Point {
  const p = view.doc.entity(name);
  assert.ok(p instanceof Point, `${name} is a point`);
  return p;
}

/** A click with a drawing tool down. */
function click(view: SketchView, x: number, y: number): void {
  const cv = view.canvas as ReturnType<typeof fakeCanvas>;
  cv.fire('pointerdown', pointer(...view.w2s(x, y)));
  cv.fire('pointerup', pointer(...view.w2s(x, y)));
}

test('the current plane flows into a fresh point, and a snapped point stays where it was', () => {
  const view = docView(VIEWS);
  assert.ok(view.doc.ok, JSON.stringify(view.doc.diagnostics));
  const front = planeNamed(view, 'front');
  view.selected = [front];
  assert.equal(view.plane, front, 'a plane selected on its own is the one being drawn in');
  view.selected = [];
  assert.equal(view.plane, front, 'and stays so past the selection');

  view.setTool('point');
  click(view, 20, 10);
  assert.ok(/point\s+p0 hint\(x: 20, y: 10\) in front/.test(view.source), view.source);
  assert.equal(pointNamed(view, 'p0').plane, front);
  // a click on the page's own datum snaps to it and does not pull it into the view
  click(view, 0, 0);
  assert.equal(view.sketch.points.length, 5, 'the click snapped rather than minting');
  assert.ok(view.source.includes('point o hint(x: 0, y: 0)\n'), view.source);
  assert.equal(pointNamed(view, 'o').plane, null);
  // and back on the page, the next point carries no clause
  view.drawOnPage();
  click(view, 25, 15);
  assert.ok(/point\s+p1 hint\(x: 25, y: 15\)\n/.test(view.source), view.source);
});

test('a projection is one constraint, and the source says `project`', () => {
  const view = docView(`${VIEWS}point a in front hint(x: 10, y: 5)\n`
                       + 'point b in top hint(x: 10, y: 100)\n');
  assert.ok(view.doc.ok, JSON.stringify(view.doc.diagnostics));
  view.addConstraints(new C.Project(pointNamed(view, 'a'), pointNamed(view, 'b')));
  const cs = view.sketch.userConstraints();
  assert.equal(cs.length, 1);
  assert.equal(cs[0].typeName, 'Project');
  assert.equal(cs[0].entities().length, 4, 'the core filled the two views in');
  assert.ok(/\na project b\n/.test(view.source), view.source);
  assert.ok(!view.source.includes('project('), `the views are never spelled:\n${view.source}`);
  view.undo();
  assert.equal(view.sketch.userConstraints().length, 0, 'and it is one step back');
});

test('a refused projection changes nothing, says why, and leaves nothing to undo', () => {
  const view = docView(`${VIEWS}point a in front hint(x: 10, y: 5)\n`
                       + 'point b in front hint(x: 20, y: 5)\npoint c hint(x: 30, y: 30)\n');
  const said: string[] = [];
  view.onStatus = (m) => said.push(m);
  const before = view.source;
  view.addConstraints(new C.Project(pointNamed(view, 'a'), pointNamed(view, 'b')));
  assert.equal(view.sketch.userConstraints().length, 0, 'two points of one view');
  assert.ok(said.some((m) => /itself/.test(m)), said.join('\n'));
  view.addConstraints(new C.Project(pointNamed(view, 'a'), pointNamed(view, 'c')));
  assert.equal(view.sketch.userConstraints().length, 0, 'a point on no view');
  assert.ok(said.some((m) => /on no plane/.test(m)), said.join('\n'));
  assert.equal(view.source, before, 'the source did not move');
  view.undo();
  assert.equal(said[said.length - 1], 'nothing to undo');
});

test('the current plane survives an edit, goes with its deletion, and is dropped by a load', () => {
  // a projection into the view about to go: its statement never names the plane, so whether
  // it goes too is the model's to say — through the live sketch, which the elaboration's
  // own was taken out into
  const view = docView(`${VIEWS}point a hint(x: 5, y: 5) in front\npoint b hint(x: 5, y: 85) in top\na project b\n`);
  view.plane = planeNamed(view, 'top');
  // a structural edit re-elaborates the drawing: the plane comes across by name
  assert.ok(view.apply(view.doc.addPoint(1, 2)));
  assert.ok(view.plane instanceof Plane, 'the plane was lost to the re-elaboration');
  assert.equal(view.plane.sketch, view.sketch, 'and is the new drawing\'s');
  assert.equal(view.doc.nameOf(view.plane), 'top');
  // deleting it takes the clauses and the statement; there is no view left to draw in
  view.selected = [view.plane];
  view.deleteSelected();
  assert.equal(view.plane, null);
  assert.ok(!view.source.includes('plane top'), view.source);
  assert.ok(view.source.includes('ground o2'), 'its points stay');
  assert.ok(!view.source.includes('project'), `the projection went with it: ${view.source}`);
  assert.ok(view.source.includes('point b hint(x: 5, y: 85)\n'), `the clause came out: ${view.source}`);
  assert.equal(view.sketch.userConstraints().length, 0);
  // and a load is another drawing's, whatever it happens to call things
  view.plane = planeNamed(view, 'front');
  view.setProgram(VIEWS);
  assert.equal(view.plane, null);
});

test('the plane tool writes the statement, seeds its points, and makes it current', () => {
  const view = docView(VIEWS);
  view.insertPlane({ name: 'aux', attitude: { from: 'front', fold: '30deg' } });
  assert.equal(view.tool, 'plane');
  click(view, 60, 0);
  assert.equal(view.sketch.planes.length, 2, 'the first click is a place, not a plane');
  click(view, 100, 0);
  assert.equal(view.sketch.planes.length, 3);
  const aux = planeNamed(view, 'aux');
  assert.equal(view.plane, aux, 'the new view is the one being drawn in');
  assert.deepEqual(view.selected, [aux]);
  assert.equal(view.tool, 'select', 'armed for one plane, and put down after it');
  assert.deepEqual(aux.origin.xy, [60, 0]);
  assert.deepEqual(aux.toward.xy, [100, 0]);
  // one statement, its attitude kept and its two points seeded in the one list — seeded *in
  // the statement*, so the frame was read off the chord that was clicked
  const line = view.source.split('\n').find((l) => /^plane\s+aux\(/.test(l));
  assert.ok(line, view.source);
  assert.ok(line.includes('origin: hint(x: 60, y: 0)'), line);
  assert.ok(line.includes('toward: hint(x: 100, y: 0)'), line);
  assert.ok(line.includes('from: front, fold: 30deg'), line);
  assert.equal(line.split('(').length - 1, line.split(')').length - 1, 'balanced');
  assert.ok(Math.abs(aux.rotor[0].value - 1) < 1e-12, 'the rotor is the chord\'s');
  // Enter after the first click points the view to the right
  view.insertPlane({ attitude: null });
  click(view, 0, -60);
  view.finishCurve();
  const v = planeNamed(view, 'v0');
  assert.deepEqual(v.toward.xy, [40, -60]);
  assert.equal(view.plane, v);
});

test('three views land where the table puts them, and stay there through the solve', () => {
  // auto-solve on: what this guards is the *solve* after the edit.  A plane is a frame whose
  // rotor and chord length are read off the chord at elaboration, so a pose written into the
  // points afterwards left both stale and the solve collapsed `toward` onto `origin`
  const view = new SketchView(fakeCanvas(), Document.read('point o hint(x: 0, y: 0)\nground o\n'));
  assert.ok(threeViews(view));
  const at = (name: string, x: number, y: number): void => {
    const p = pointNamed(view, name);
    assert.ok(Math.hypot(p.xy[0] - x, p.xy[1] - y) < 1e-6, `${name} at ${p.xy}, not (${x}, ${y})`);
  };
  at('front.origin', 0, 0);
  at('front.toward', 40, 0);
  at('top.origin', 0, 80);
  at('top.toward', 40, 80);
  at('right.origin', 120, 0);
  at('right.toward', 120, -40);
  // the right view is turned a quarter clockwise: its rotor says so
  const [c, s] = planeNamed(view, 'right').rotor;
  assert.ok(Math.abs(c.value) < 1e-9 && Math.abs(s.value + 1) < 1e-9,
            `rotor ${c.value}, ${s.value}`);
  assert.ok(view.lastResult?.success, 'the layout solved');
  assert.equal(view.plane, planeNamed(view, 'front'), 'drawing in the front');
  // one edit: the three statements, their seeds, and the five relations, all written
  assert.ok(view.source.includes('plane   right(origin: hint(x: 120, y: 0), toward: hint(x: 120, '
                                 + 'y: -40), from: front, fold: -90deg)'), view.source);
  assert.ok(view.source.includes('front.origin vertical top.origin'), view.source);
  const kinds = view.sketch.userConstraints().map((c) => c.typeName).sort();
  assert.deepEqual(kinds, ['HorizontalPoints', 'HorizontalPoints', 'HorizontalPoints',
                           'VerticalPoints', 'VerticalPoints']);
  view.undo();
  assert.equal(view.sketch.planes.length, 0, 'and one step back');
  assert.equal(view.plane, null);
});

/* -- the traced picture ------------------------------------------------------------
 *
 * The placement is a similarity in world coordinates, so all of it can be checked without a
 * browser: a `Bitmap` is two numbers, and the fake canvas swallows the one `drawImage` call.
 * What is worth a test is what a person doing the tracing would notice — that the picture is
 * handled like everything else on the canvas, that the drawing still outranks it, and that none
 * of it reaches the document. */

/** A picture, without one.  Everything below is a fact about the placement, not the pixels. */
function bitmap(width = 400, height = 300): Bitmap {
  return { width, height };
}

/** A view with a picture on it, dropped again so the tests start where a user would: with it
 *  on the canvas and nothing selected.  (`traceImage` leaves it selected, which is its own
 *  case below.) */
function traced(text = ANNOTATED): SketchView {
  const view = docView(text);
  view.traceImage(bitmap(), 'photo.png');
  view.dropImage();
  return view;
}

/** Press, move and release on the canvas, as the pointer handlers see it. */
function drag(view: SketchView, from: [number, number], to: [number, number]): void {
  const cv = view.canvas as ReturnType<typeof fakeCanvas>;
  cv.fire('pointerdown', pointer(...from));
  cv.fire('pointermove', pointer(...to));
  cv.fire('pointerup', pointer(...to));
}

/** A screen point a given number of pixels along the picture's top edge, which is the frame —
 *  the only part of it a press takes hold of while it is not selected. */
function onFrame(view: SketchView): [number, number] {
  const [tl, tr] = corners(view.underlay!).map(([x, y]) => view.w2s(x, y));
  return [(tl[0] + tr[0]) / 2, (tl[1] + tr[1]) / 2];
}

test('a traced picture lands in the middle of the view, edges in sight, and selected', () => {
  const view = docView(ANNOTATED);
  view.traceImage(bitmap(), 'photo.png');
  const u = view.underlay!;
  assert.deepEqual([u.x, u.y], view.s2w(view.width / 2, view.height / 2));
  assert.equal(u.angle, 0);
  assert.ok(u.picked, 'the handles that place it should be there to be used at once');
  // the longer side spans some but not all of the shorter side of the canvas
  const span = view.len(u.scale * u.image.width);
  assert.ok(span > 0.3 * view.height && span < view.height, `spanned ${span}px`);
  // on it, and only just: a corner is the edge of it, so what is measured is either side
  assert.ok(contains(u, u.x, u.y));
  const [cx, cy] = corners(u)[0];
  const in3 = u.scale * 3;
  assert.ok(contains(u, cx + in3, cy - in3), 'three pixels inside the top-left corner is not on it');
  assert.ok(!contains(u, cx - in3, cy + in3), 'three pixels outside it is');
});

test('world and image coordinates are inverses, turned and scaled or not', () => {
  const view = traced();
  const u = view.underlay!;
  u.angle = 0.7;
  u.scale = 0.42;
  for (const [px, py] of [[0, 0], [37, -11], [-200, 150]]) {
    const back = toImage(u, ...toWorld(u, px, py));
    assert.ok(Math.hypot(back[0] - px, back[1] - py) < 1e-9, `${back} for ${[px, py]}`);
  }
  // and the corners come back in image order: the first is the top-left one, which at a
  // rotation of zero is up and to the left of the centre — the picture is not mirrored
  u.angle = 0;
  const [tl, tr, , bl] = corners(u);
  assert.ok(tl[0] < u.x && tl[1] > u.y, `top-left at ${tl}`);
  assert.ok(tr[0] > u.x && tr[1] > u.y, `top-right at ${tr}`);
  assert.ok(bl[0] < u.x && bl[1] < u.y, `bottom-left at ${bl}`);
});

test('unselected, only its frame answers a press — the middle of it is where you draw', () => {
  const view = traced();
  const u = view.underlay!;
  const middle = view.w2s(u.x, u.y);
  const before = view.sketch.points.length;

  // a click in the middle of it selects nothing and starts a band, exactly as bare canvas does
  drag(view, middle, [middle[0] + 40, middle[1] + 40]);
  assert.ok(!u.picked, 'clicking through it took hold of it');

  // and a drawing tool puts a point down on top of it
  view.setTool('point');
  drag(view, middle, middle);
  assert.equal(view.sketch.points.length, before + 1);
  assert.ok(!u.picked);
  view.setTool('select');

  // its frame, though, is a click target like any other
  drag(view, onFrame(view), onFrame(view));
  assert.ok(u.picked, 'the frame is the handle, and it did not answer');
});

test('selecting the picture and selecting geometry are exclusive', () => {
  const view = traced();
  const u = view.underlay!;
  const a = view.sketch.points[0];
  view.selected = [a];

  drag(view, onFrame(view), onFrame(view));
  assert.ok(u.picked);
  assert.deepEqual(view.selected, [], 'a photograph is not a Primitive and cannot join the list');

  // and back: taking hold of a point lets the picture go
  const on = view.w2s(...a.xy);
  drag(view, on, on);
  assert.deepEqual(view.selected, [a]);
  assert.ok(!u.picked);

  // and it is the *assignment* that lets it go, not the gesture that happened to make one —
  // paste, a rubber band and the constraint list all write this field and none of them can be
  // expected to remember the picture.  Delete would otherwise take the photograph instead.
  view.pickImage();
  assert.ok(u.picked);
  view.selected = [a];
  assert.ok(!u.picked, 'selecting geometry by any route should let the picture go');
});

test('the drawing outranks the picture: a line across it is what a click on the line picks', () => {
  const view = traced();
  const u = view.underlay!;
  const ln = view.sketch.lines[0];
  // put the picture over the line, so the two are under the same pixel
  const mid: [number, number] = [(ln.p1.xy[0] + ln.p2.xy[0]) / 2, (ln.p1.xy[1] + ln.p2.xy[1]) / 2];
  [u.x, u.y] = mid;
  const on = view.w2s(...mid);

  drag(view, on, on);
  assert.deepEqual(view.selected, [ln], 'the picture swallowed a click meant for the drawing');
  assert.ok(!u.picked);
});

test('dragging the picture keeps the place that was grabbed under the pointer', () => {
  const view = traced();
  const u = view.underlay!;
  view.pickImage();                      // selected, so the whole of it drags
  const from = view.w2s(u.x + 3, u.y - 2);
  const held = toImage(u, ...view.s2w(...from));
  const size = u.scale;

  drag(view, from, [from[0] + 55, from[1] - 30]);
  const now = toImage(u, ...view.s2w(from[0] + 55, from[1] - 30));
  assert.ok(Math.hypot(now[0] - held[0], now[1] - held[1]) < 1e-6,
            `the picture slipped: ${held} became ${now}`);
  assert.equal(u.scale, size, 'a move is not a resize');
  assert.equal(u.angle, 0, 'and not a rotation');
});

test('a press on the frame selects and moves it in the one gesture', () => {
  const view = traced();
  const u = view.underlay!;
  const from = onFrame(view);
  const was: [number, number] = [u.x, u.y];

  drag(view, from, [from[0] + 40, from[1]]);
  assert.ok(u.picked);
  assert.ok(Math.abs(u.x - was[0] - view.world(40)) < 1e-9, 'it did not follow the pointer');
  assert.equal(u.y, was[1]);
});

test('dragging a corner sizes and turns it about a centre that stays put', () => {
  const view = traced();
  const u = view.underlay!;
  view.pickImage();                      // handles exist only while it is selected
  const centre: [number, number] = [u.x, u.y];
  const grip = view.w2s(...corners(u)[0]);
  const to: [number, number] = [grip[0] - 60, grip[1] + 25];

  drag(view, grip, to);
  assert.deepEqual([u.x, u.y], centre, 'the centre moved');
  assert.ok(u.angle !== 0, 'it did not turn');
  // the corner that was taken hold of is where the pointer left it — which is the whole of
  // what makes one handle do both jobs without a mode
  const at = view.w2s(...corners(u)[0]);
  assert.ok(Math.hypot(at[0] - to[0], at[1] - to[1]) < 1e-6, `corner at ${at}, pointer at ${to}`);
  // and the scale stayed uniform: the two sides keep the ratio the pixels have
  const side = (a: number[], b: number[]): number => Math.hypot(b[0] - a[0], b[1] - a[1]);
  const [tl, tr, , bl] = corners(u);
  assert.ok(Math.abs(side(tl, tr) / side(tl, bl) - u.image.width / u.image.height) < 1e-9,
            'the picture was squashed');
});

test('a corner handle is not there to be grabbed until the picture is selected', () => {
  const view = traced();
  const u = view.underlay!;
  const grip = view.w2s(...corners(u)[0]);
  const was = { scale: u.scale, angle: u.angle };

  drag(view, grip, [grip[0] - 60, grip[1] + 25]);
  assert.deepEqual({ scale: u.scale, angle: u.angle }, was, 'an unselected corner resized it');
});

test('Delete takes the picture when the picture is what is selected, and only then', () => {
  const view = traced();
  view.selected = [view.sketch.points[0]];
  const points = view.sketch.points.length;
  view.deleteSelected();
  assert.ok(view.underlay, 'deleting a point took the photograph with it');
  assert.equal(view.sketch.points.length, points - 1);

  view.pickImage();
  const source = view.source;
  view.deleteSelected();
  assert.equal(view.underlay, null);
  assert.equal(view.source, source, 'removing the picture spliced the document');
});

test('the picture is not in the document: nothing it does is written, saved or undone', () => {
  const view = traced();
  const u = view.underlay!;
  const source = view.source;
  view.pickImage();
  const on = view.w2s(u.x, u.y);

  drag(view, on, [on[0] + 90, on[1] + 15]);
  assert.notEqual(u.x, view.s2w(...on)[0], 'the test moved nothing');
  assert.equal(view.source, source, 'a traced picture wrote itself into the document');
  view.undo();
  assert.equal(view.source, source, 'undo stepped over a document edit that never happened');
  assert.ok(view.underlay, 'and undo took the photograph away');
  assert.ok(!io.dumps(view.sketch).includes('photo.png'));
});

test('fading is kept on the scale, from either end', () => {
  const view = traced();
  const u = view.underlay!;
  for (let i = 0; i < 20; i++) view.fadeImage(-0.1);
  assert.equal(u.opacity, 0);
  for (let i = 0; i < 20; i++) view.fadeImage(0.1);
  assert.equal(u.opacity, 1);
});

test('an arc drawn in a view puts its core-minted centre in the view too', () => {
  const view = docView(VIEWS);
  view.plane = planeNamed(view, 'top');
  view.setTool('arc3');
  click(view, 0, 60);
  click(view, 40, 60);
  click(view, 20, 75);            // the third click makes the circumcircle, and its centre
  assert.equal(view.sketch.arcs.length, 1);
  const arc = view.sketch.arcs[0];
  // the centre is minted inside the core, after the two ends were joined: left on the page it
  // is a straddling statement no `in` clause can say, and the source stops tracking the drawing
  assert.equal(arc.children.length, 3, 'centre, start and end');
  for (const p of arc.children) {
    assert.equal(p.plane, view.plane, 'every point of the arc is in the view');
  }
  view.afterEdit();
  assert.ok(view.source.includes(' in top'), view.source);
  // and the source keeps up: a second sync is not refused
  const before = view.source;
  view.syncSource();
  assert.equal(view.source, before);
});

test('drawing on the page stays on the page across a re-elaboration', () => {
  const view = docView(VIEWS);
  const top = planeNamed(view, 'top');
  view.selected = [top];
  assert.equal(view.plane, top);
  view.drawOnPage();
  assert.equal(view.plane, null);
  assert.ok(!view.selected.includes(top), 'the view stops being the subject');
  // a structural edit re-elaborates and rebinds the selection: the plane must not come back
  assert.ok(view.apply(view.doc.addPoint(3, 4)));
  assert.equal(view.plane, null, 'the rebind re-armed the current plane');
  view.setTool('point');
  click(view, 12, 12);
  assert.equal(view.sketch.points.at(-1)?.plane, null, 'and the next point is on the page');
});

/* -- the overview: the same document, folded back into the glass box --------------------
 *
 * Everything about the scene itself — where a view stands in space, what reconstructs, how it
 * projects — is the core's and is tested there.  What is worth a test here is the *mode*: that
 * it is view state and so survives a document change, that nothing in it edits, and that a
 * press picks the entity whose edge is under it. */

/** Two views with one edge of an object drawn in each, tied corner to corner — the least a box
 *  can be folded from. */
const BOXED = `${VIEWS}point af in front hint(x: 10, y: 10)
point bf in front hint(x: 30, y: 20)
point a2 in top hint(x: 10, y: 100)
point b2 in top hint(x: 30, y: 100)
line lf(af, bf)
line lt(a2, b2)
af project a2
bf project b2
`;

function boxed(): SketchView {
  const view = docView(BOXED);
  assert.ok(view.doc.ok, JSON.stringify(view.doc.diagnostics));
  view.setOverview(true);
  return view;
}

/** A screen point in the middle of one of the scene's `drawn` segments, and the entity that
 *  segment is drawn from — so a press can be aimed without this test knowing any 3D. */
function onEdge(view: SketchView): [[number, number], Primitive] {
  const it = view.scene().items.find((i) => i.part === 'drawn' && i.pts.length > 1);
  assert.ok(it, 'the scene has a view\'s own geometry in it');
  const ent = view.entityOf(it);
  assert.ok(ent, 'and it names what it is drawn from');
  return [midOf(view, it), ent];
}

/** The screen midpoint of an item's first segment. */
function midOf(view: SketchView, it: Item): [number, number] {
  const [a, b] = [view.w2s(...it.pts[0]), view.w2s(...it.pts[1])];
  return [(a[0] + b[0]) / 2, (a[1] + b[1]) / 2];
}

/** A screen point on the edge of one of the scene's panes, and the plane that pane is of. */
function onPaneEdge(view: SketchView): [[number, number], Plane] {
  const it = view.scene().items.find((i) => i.part === 'face');
  assert.ok(it, 'the scene has a pane in it');
  const plane = view.planeOf(it);
  assert.ok(plane, 'and it names the view it is of');
  return [midOf(view, it), plane];
}

test('the overview is view state, so it outlives the document it was opened on', () => {
  const view = boxed();
  const orbit = { ...view.orbit };
  // the drawing is read-only in the box but the *source* is not: typing in the program panel
  // re-elaborates the drawing under it, and the mode has to survive that swap
  const n = view.sketch.points.length;
  view.setProgram(`${BOXED}point extra hint(x: 1, y: 2)\n`);
  assert.equal(view.sketch.points.length, n + 1, 'a structural edit re-elaborates the drawing');
  assert.equal(view.overview, true, 'the mode survived the swap');
  assert.deepEqual(view.orbit, orbit, 'and so did the orbit');
  view.setProgram(VIEWS);
  assert.equal(view.overview, true, 'and a load is a document, not a camera');
  // the scene is asked afresh, so it is the *new* document's
  assert.ok(view.scene().items.every((i) => i.part !== 'solid'), 'nothing left to reconstruct');
});

test('a new document is one undo step, and takes nothing in flight with it', () => {
  const view = docView(examples.source('rect_fillets'));
  const before = view.source;
  // half a spline fit, a plane armed: state that points into the sketch about to be replaced
  view.setTool('splinefit');
  click(view, 30, 30);
  click(view, 60, 40);
  assert.equal(view.pendingFit.length, 2, 'two places collected');
  view.newDocument();
  assert.notEqual(view.source, before, 'a fresh sheet');
  assert.deepEqual(view.pendingFit, [], 'the fit did not come along');
  assert.deepEqual(view.pending, []);
  assert.equal(view.planeSpec, null);
  // ⌘Z is the drawing that was replaced — the *last* state of it, not an older one — and ⌘⇧Z
  // is the fresh sheet again
  view.undo();
  assert.equal(view.source, before, 'undo restores the document New replaced');
  view.redo();
  assert.notEqual(view.source, before, 'and redo is the new one again');
  // a test case loaded from the menu goes the same way
  view.load(examples.source('bracket'));
  assert.ok(view.sketch.planes.length, 'the bracket');
  view.undo();
  assert.notEqual(view.source, before);
  assert.equal(view.sketch.planes.length, 0, 'back to the sheet it replaced');
});

test('the box exists only where there are views', () => {
  // a drawing with no plane has nothing to fold: ⌘B stays on the sheet and says why
  const view = docView('point p hint(x: 0, y: 0)\nground p\n');
  let said = '';
  view.onStatus = (s) => { said = s; };
  view.setOverview(true);
  assert.equal(view.overview, false, 'no box without a view');
  assert.match(said, /no views/);
  // and File ▸ New from inside the box comes back to the sheet, where the tools work — left in
  // the box, a plane-less document is a tilted empty sheet on which every click does nothing
  const boxedView = boxed();
  boxedView.setProgram('point p hint(x: 0, y: 0)\nground p\n');
  assert.equal(boxedView.overview, false, 'a document with no plane has no box');
  boxedView.setTool('point');
  const n = boxedView.sketch.points.length;
  click(boxedView, 40, 40);
  assert.equal(boxedView.sketch.points.length, n + 1, 'and drawing works again');
});

test('nothing in the overview edits: the tools and the drags are gated off in one place', () => {
  const view = boxed();
  const before = view.source;
  const n = view.sketch.points.length;
  view.setTool('point');
  click(view, 20, 20);
  assert.equal(view.sketch.points.length, n, 'the point tool minted nothing');
  // a press on an edge would be a drag on the sheet; here it takes no undo state and moves no
  // geometry, so the source is what it was
  const poses = view.sketch.points.map((p) => [...p.xy]);
  const [at] = onEdge(view);
  drag(view, at, [at[0] + 40, at[1] + 25]);
  assert.equal(view.source, before, 'the source did not move');
  assert.deepEqual(view.sketch.points.map((p) => [...p.xy]), poses, 'and neither did the drawing');
  view.undo();
  assert.equal(view.source, before, 'there was nothing on the stack to undo');
});

test('a press in the overview picks the edge under it, and then orbits', () => {
  const view = boxed();
  const [at, ent] = onEdge(view);
  const cv = view.canvas as ReturnType<typeof fakeCanvas>;
  cv.fire('pointerdown', pointer(...at));
  assert.deepEqual(view.selected, [ent], 'the entity the edge is drawn from');
  const az = view.orbit.az, el = view.orbit.el;
  cv.fire('pointermove', pointer(at[0] + 50, at[1] + 30));
  cv.fire('pointerup', pointer(at[0] + 50, at[1] + 30));
  assert.ok(view.orbit.az > az, 'a rightward drag swung the eye round');
  assert.ok(view.orbit.el < el, 'and a downward one lowered it: the pointer pushes the box');
  assert.deepEqual(view.selected, [ent], 'the selection is the press\'s, not the drag\'s');
  // and a press on nothing clears it without touching the document
  const before = view.source;
  cv.fire('pointerdown', pointer(5, 5));
  cv.fire('pointerup', pointer(5, 5));
  assert.deepEqual(view.selected, []);
  assert.equal(view.source, before);
});

test('double-clicking a view in the box goes to it, and the box shows every plane', () => {
  const view = boxed();
  // every plane is a pane with its own axes, drawn in or not — which is what makes a view
  // something you can go to before anything has been drawn in it
  const scene = view.scene();
  const planes = view.sketch.planes.length;
  assert.equal(scene.items.filter((i) => i.part === 'face').length, planes, 'a pane per plane');
  assert.equal(scene.items.filter((i) => i.part === 'axis').length, 2 * planes, 'an x and a y');

  // aimed at a pane's own outline: a double-click anywhere that belongs to a view goes to it
  const [edge, want] = onPaneEdge(view);
  const cv = view.canvas as ReturnType<typeof fakeCanvas>;
  cv.fire('dblclick', { clientX: edge[0], clientY: edge[1] });
  assert.equal(view.overview, false, 'it left the box');
  assert.equal(view.plane, want, 'on the view that was double-clicked, where a tool now draws');
  // armed, not selected: a selected plane opens the constraints window over the drawing
  assert.deepEqual(view.selected, [], 'and nothing is selected');
});

test('a pane bolds when the pointer is on its edge, and lets go when it leaves', () => {
  const view = boxed();
  const cv = view.canvas as ReturnType<typeof fakeCanvas>;
  const [edge, plane] = onPaneEdge(view);
  const face = view.scene().items.find((i) => i.part === 'face')!;
  cv.fire('pointermove', pointer(...edge));
  assert.equal(view.hoverPlane, plane, 'the pane whose edge the pointer is on');
  // its *interior* is not a target — nothing on this canvas is picked by an area — so a point
  // well inside the same pane holds nothing
  const mid = face.pts.slice(0, 4).reduce((m, p) => [m[0] + p[0] / 4, m[1] + p[1] / 4], [0, 0]);
  cv.fire('pointermove', pointer(...view.w2s(mid[0], mid[1])));
  assert.equal(view.hoverPlane, null, 'the middle of a pane is where you draw, not what you grab');
  // and leaving the box lets it go rather than holding a proxy the next document will not know
  cv.fire('pointermove', pointer(...edge));
  assert.ok(view.hoverPlane);
  view.setOverview(false);
  assert.equal(view.hoverPlane, null);
});

test('every verb that reaches the document without a pointer is refused in the box', () => {
  const view = boxed();
  const before = view.source;
  const cv = view.canvas as ReturnType<typeof fakeCanvas>;
  // an edge selected in the box is a real entity: Delete on it must not splice the source
  const [at, ent] = onEdge(view);
  cv.fire('pointerdown', pointer(...at));
  cv.fire('pointerup', pointer(...at));
  assert.deepEqual(view.selected, [ent]);
  view.deleteSelected();
  assert.equal(view.source, before, 'Delete did nothing');
  assert.ok(view.copySelected() >= 1, 'copying is reading, and still allowed');
  assert.equal(view.pasteClipboard(), 0, 'pasting is not');
  assert.equal(view.source, before);
  view.toggleConstructionSelected();
  assert.equal(view.source, before, 'and neither is the class toggle');
  const n = view.sketch.userConstraints().length;
  const [p, q] = view.sketch.points;
  view.addConstraints(new C.HorizontalPoints(p, q));
  assert.equal(view.sketch.userConstraints().length, n, 'the constraints bar adds nothing');
  assert.equal(view.startDimension([new C.Distance(p, q, 10)], true, null), false, 'nor a dimension');
  view.undo();
  assert.equal(view.source, before, 'and there was nothing on the stack to undo');
});

test('leaving or entering the box abandons the gesture in flight', () => {
  const view = docView(BOXED);
  view.fit();
  const cv = view.canvas as ReturnType<typeof fakeCanvas>;
  const before = view.source;
  // press and hold a free point on the sheet, then flip into the box while holding
  const pt = view.sketch.points.find((p) => !p.isFixed)!;
  const at = view.w2s(...pt.xy);
  cv.fire('pointerdown', pointer(...at));
  assert.ok(view.gesture, 'a drag is live');
  view.setOverview(true);
  assert.equal(view.gesture, null, 'and is dropped at the seam, uncommitted');
  cv.fire('pointermove', pointer(at[0] + 80, at[1] + 60));
  cv.fire('pointerup', pointer(at[0] + 80, at[1] + 60));
  assert.equal(view.source, before, 'so the release writes nothing');
  // and the other way: an orbit does not go on turning the sheet
  cv.fire('pointerdown', pointer(5, 5));
  assert.ok(view.gesture, 'an orbit is live');
  const az = view.orbit.az;
  view.setOverview(false);
  assert.equal(view.gesture, null);
  cv.fire('pointermove', pointer(100, 5));
  cv.fire('pointerup', pointer(100, 5));
  assert.equal(view.orbit.az, az, 'the orbit stayed where it was');
  assert.equal(view.source, before);
});

test('a click on a pane selects nothing and arms nothing; the hover lets go off the canvas', () => {
  const view = boxed();
  const cv = view.canvas as ReturnType<typeof fakeCanvas>;
  const [edge] = onPaneEdge(view);
  const plane = view.plane;
  cv.fire('pointerdown', pointer(...edge));
  cv.fire('pointerup', pointer(...edge));
  assert.deepEqual(view.selected, [], 'a pane is not a thing to select');
  assert.equal(view.plane, plane, 'and a click did not change where the next point goes');
  cv.fire('pointermove', pointer(...edge));
  assert.ok(view.hoverPlane, 'the pointer on its edge bolds it');
  cv.fire('pointerleave', {});
  assert.equal(view.hoverPlane, null, 'and off the canvas it lets go');
  // back on the sheet the box's hand is not left on the cursor
  view.setOverview(false);
  assert.equal(view.canvas.style.cursor, '');
});

test('a fit in the overview frames the scene, not the sheet', () => {
  const view = docView(BOXED);
  view.fit();                         // the sheet's own bounds
  view.setOverview(true);             // which refits, the two spaces being unrelated
  const b = view.scene().bounds;
  const mid = view.w2s((b[0] + b[2]) / 2, (b[1] + b[3]) / 2);
  assert.ok(Math.abs(mid[0] - view.width / 2) < 1e-6 && Math.abs(mid[1] - view.height / 2) < 1e-6,
            `the scene is centred: ${mid}`);
  const [x0, y0] = view.w2s(b[0], b[1]), [x1, y1] = view.w2s(b[2], b[3]);
  assert.ok(Math.min(x0, x1) >= 0 && Math.max(x0, x1) <= view.width, 'and it is all on screen');
  assert.ok(Math.min(y0, y1) >= 0 && Math.max(y0, y1) <= view.height);
  // and back on the sheet the drawing is framed again
  view.setOverview(false);
  const d = view.sketch.drawnBounds();
  const c = view.w2s((d[0] + d[2]) / 2, (d[1] + d[3]) / 2);
  assert.ok(Math.abs(c[0] - view.width / 2) < 1e-6 && Math.abs(c[1] - view.height / 2) < 1e-6,
            `the drawing is centred: ${c}`);
});
