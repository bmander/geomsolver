/* The app layer: `SketchView`'s gesture and animation lifecycles, against a stubbed canvas.
 *
 * These are the parts of the front end that own core handles — a drag, a compiled plan, an
 * interval — and the bugs worth a test here are the ones where a handle outlives what it was made
 * for. */
import assert from 'node:assert/strict';
import test from 'node:test';

import * as C from '../core/constraints.js';
import { Sketch } from '../core/model.js';
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
