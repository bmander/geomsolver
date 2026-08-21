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
import { SketchView } from '../app/view.js';
import { initCore } from '../core/wasm.js';
import { fakeCanvas, pointer } from './canvas.js';

// the view schedules its repaints; nothing is being looked at, so run them inline
(globalThis as { requestAnimationFrame?: unknown }).requestAnimationFrame ??=
  (fn: FrameRequestCallback) => { fn(0); return 0; };
(globalThis as { cancelAnimationFrame?: unknown }).cancelAnimationFrame ??= () => {};

await initCore();

/** A fixed base with one free apex, over-determined once the apex is pinned — so a drag on it
 *  takes the numeric path, which adds a soft drag target to the sketch. */
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

const softCount = (sk: Sketch): number => sk.constraints.filter((c) => c.soft).length;

test('a second pointer does not take over a live drag', () => {
  const sk = pinnedApex();
  const view = viewOn(sk);
  const cv = view.canvas as ReturnType<typeof fakeCanvas>;
  const apex = sk.points[2];
  const [sx, sy] = view.w2s(...apex.xy);

  cv.fire('pointerdown', pointer(sx, sy, { pointerId: 1 }));
  assert.deepEqual(view.selected, [apex]);
  assert.equal(softCount(sk), 1, 'the drag should have added its target');

  // a second finger, far from anything: on its own that would clear the selection and start a
  // rubber band, dropping the live drag with its core handle and its target still in the sketch
  cv.fire('pointerdown', pointer(sx + 300, sy + 300, { pointerId: 2 }));
  cv.fire('pointermove', pointer(sx + 320, sy + 320, { pointerId: 2 }));
  cv.fire('pointerup', pointer(sx + 320, sy + 320, { pointerId: 2 }));
  assert.deepEqual(view.selected, [apex], 'the second pointer took over');
  assert.equal(softCount(sk), 1, 'the first drag was dropped without ending');

  cv.fire('pointerup', pointer(sx, sy, { pointerId: 1 }));
  assert.equal(softCount(sk), 0, 'ending the drag has to take its target with it');
});

test('a cancelled pointer ends the drag it owned', () => {
  const sk = pinnedApex();
  const view = viewOn(sk);
  const cv = view.canvas as ReturnType<typeof fakeCanvas>;
  const [sx, sy] = view.w2s(...sk.points[2].xy);

  cv.fire('pointerdown', pointer(sx, sy, { pointerId: 1 }));
  assert.equal(softCount(sk), 1);
  cv.fire('pointercancel', pointer(sx, sy, { pointerId: 1 }));
  assert.equal(softCount(sk), 0, 'a cancelled touch left the drag target behind');

  // and the view is usable afterwards: a fresh press starts a fresh drag
  cv.fire('pointerdown', pointer(sx, sy, { pointerId: 3 }));
  assert.equal(softCount(sk), 1);
  cv.fire('pointerup', pointer(sx, sy, { pointerId: 3 }));
  assert.equal(softCount(sk), 0);
});

test('losing pointer capture ends the drag too', () => {
  const sk = pinnedApex();
  const view = viewOn(sk);
  const cv = view.canvas as ReturnType<typeof fakeCanvas>;
  const [sx, sy] = view.w2s(...sk.points[2].xy);

  cv.fire('pointerdown', pointer(sx, sy, { pointerId: 1 }));
  assert.equal(softCount(sk), 1);
  cv.fire('lostpointercapture', pointer(sx, sy, { pointerId: 1 }));
  assert.equal(softCount(sk), 0);
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
