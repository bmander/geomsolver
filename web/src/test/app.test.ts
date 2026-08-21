/* The app layer: `SketchView`'s gesture and animation lifecycles, against a stubbed canvas.
 *
 * These are the parts of the front end that own core handles — a drag, a compiled plan, an
 * interval — and the bugs worth a test here are the ones where a handle outlives what it was made
 * for. */
import assert from 'node:assert/strict';
import test from 'node:test';

import * as C from '../core/constraints.js';
import { Constraint } from '../core/constraints.js';
import * as io from '../core/io.js';
import { Sketch } from '../core/model.js';
import { callouts } from '../core/callout.js';
import { PlanDrag } from '../core/decompose.js';
import { SketchView } from '../app/view.js';
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

function viewOn(sk: Sketch): SketchView {
  const view = new SketchView(fakeCanvas(), sk);
  view.autoSolve = false;
  return view;
}


test('a second pointer does not take over a live drag', () => {
  const sk = pinnedApex();
  const view = viewOn(sk);
  const cv = view.canvas as ReturnType<typeof fakeCanvas>;
  const apex = sk.points[2];
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
  const cs = callouts(sk, 1 / view.scale);
  assert.equal(cs.items.length, 1);
  assert.equal(cs.items[0].text, '60');
  assert.equal(cs.items[0].id, sk.userConstraints()[0].id);
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
  assert.deepEqual(picked, [sk.userConstraints()[0]]);
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
  assert.deepEqual(view.selected, [sk.points[1]]);
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
  assert.deepEqual(edited, [sk.userConstraints()[0]]);
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
  const k = callouts(sk, 1 / view.scale).items[0];
  return view.w2s(k.solid[0][0][0], k.solid[0][0][1])[1];
}

test('dragging a callout moves it, and it stays moved', () => {
  const sk = dimensioned();
  const view = viewOn(sk);
  const cv = view.canvas as ReturnType<typeof fakeCanvas>;
  view.draw();
  const before = dimY(view, sk);
  const [ax, ay] = view.w2s(...sk.points[0].xy);

  cv.fire('pointerdown', pointer(ax, before));      // take hold of the dimension line
  cv.fire('pointermove', pointer(ax, before - 60));
  cv.fire('pointerup', pointer(ax, before - 60));
  const after = dimY(view, sk);
  assert.ok(Math.abs(after - (before - 60)) < 1, `${before} → ${after}`);

  // it is the sketch that remembers, so it survives a re-solve and a round trip
  view.solveNow();
  assert.ok(Math.abs(dimY(view, sk) - after) < 1e-6, 'a solve moved the callout');
  const reloaded = io.loads(io.dumps(sk));
  const view2 = viewOn(reloaded);
  view2.scale = view.scale;
  view2.originX = view.originX;
  view2.originY = view.originY;
  assert.ok(Math.abs(dimY(view2, reloaded) - after) < 1e-6, 'the placement did not save');
});

test('a callout follows the point it was grabbed at, not the pointer', () => {
  const sk = dimensioned();
  const view = viewOn(sk);
  const cv = view.canvas as ReturnType<typeof fakeCanvas>;
  view.draw();
  const before = dimY(view, sk);
  const [ax] = view.w2s(...sk.points[0].xy);

  // press 6px below the line, then move 40px up: the line should move 40, not 46
  cv.fire('pointerdown', pointer(ax, before + 6));
  cv.fire('pointermove', pointer(ax, before + 6 - 40));
  cv.fire('pointerup', pointer(ax, before + 6 - 40));
  assert.ok(Math.abs(dimY(view, sk) - (before - 40)) < 1, 'the callout jumped to the pointer');
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
  const before = dimY(view, sk);
  const [ax] = view.w2s(...sk.points[0].xy);
  cv.fire('pointerdown', pointer(ax, before));
  cv.fire('pointermove', pointer(ax, before - 60));
  cv.fire('pointerup', pointer(ax, before - 60));
  assert.notEqual(dimY(view, sk), before);

  view.resetCallouts();
  assert.ok(Math.abs(dimY(view, sk) - before) < 1e-9);
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
  view.selected = [sk.lines[0]];
  assert.equal(view.copySelected(), 3, 'the line and its two ends');

  assert.equal(view.pasteClipboard(), 3);
  assert.equal(sk.points.length, 4);
  assert.equal(sk.lines.length, 2);
  assert.equal(sk.userConstraints().length, 2, 'the copy brought its own Distance');
});

test('a paste is selected, and lands clear of what it came from', () => {
  const sk = oneLine();
  const view = viewOn(sk);
  view.selected = [sk.lines[0]];
  view.copySelected();
  view.pasteClipboard();

  assert.deepEqual(view.selected, [sk.points[2], sk.points[3], sk.lines[1]]);
  const [x0, y0] = sk.points[0].xy;
  const [x1, y1] = sk.points[2].xy;
  assert.ok(x1 > x0 && y1 < y0, `the copy should be nudged clear: ${x1},${y1} vs ${x0},${y0}`);
});

test('successive pastes cascade instead of piling up', () => {
  const sk = oneLine();
  const view = viewOn(sk);
  view.selected = [sk.lines[0]];
  view.copySelected();
  view.pasteClipboard();
  const first = sk.points[2].xy;
  view.pasteClipboard();
  const second = sk.points[4].xy;
  assert.notDeepEqual(second, first, 'the second paste landed on the first');
  assert.ok(second[0] > first[0]);
});

test('a pasted copy is independent of the original', () => {
  const sk = oneLine();
  const view = viewOn(sk);
  view.selected = [sk.lines[0]];
  view.copySelected();
  view.pasteClipboard();

  // the pasted Distance names the pasted points and nothing else
  const pasted = sk.userConstraints()[1];
  assert.deepEqual(pasted.entities(), [sk.points[2], sk.points[3]]);
});

test('copying nothing leaves the clipboard as it was', () => {
  const sk = oneLine();
  const view = viewOn(sk);
  view.selected = [sk.lines[0]];
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
  view.selected = [sk.lines[0]];
  view.copySelected();
  view.pasteClipboard();
  assert.notEqual(io.dumps(view.sketch), before);
  view.undo();
  assert.equal(io.dumps(view.sketch), before);
});

test('cut takes the selection out and keeps it', () => {
  const sk = oneLine();
  const view = viewOn(sk);
  view.selected = [sk.lines[0]];
  assert.equal(view.cutSelected(), 3);
  assert.equal(view.sketch.lines.length, 0, 'the line should be gone');
  assert.equal(view.pasteClipboard(), 3, 'and still on the clipboard');
  assert.equal(view.sketch.lines.length, 1);
});

test('the clipboard outlives the sketch it came from', () => {
  const sk = oneLine();
  const view = viewOn(sk);
  view.selected = [sk.lines[0]];
  view.copySelected();
  view.setSketch(new Sketch());          // a fresh sheet
  assert.equal(view.pasteClipboard(), 3);
  assert.equal(view.sketch.lines.length, 1);
  assert.equal(view.sketch.userConstraints().length, 1);
});
