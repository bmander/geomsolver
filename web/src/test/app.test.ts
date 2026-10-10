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
import { Plane, Point, Sketch } from '../core/model.js';
import { Document, fromSketch } from '../core/program.js';
import type { Diagnosis } from '../core/diagnose.js';
import { callouts, pairOf } from '../core/callout.js';
import type { PairDimension } from '../core/callout.js';
import { PlanDrag } from '../core/decompose.js';
import { solve } from '../core/system.js';
import { DimAlt, SketchView } from '../app/view.js';
import { contains, corners, toImage, toWorld } from '../app/underlay.js';
import type { Bitmap } from '../app/underlay.js';
import { initCore } from '../core/wasm.js';
import { fakeCanvas, pointer } from './canvas.js';
import { paintCallouts } from '../app/paint.js';

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

const PROJECTED_CIRCLE = 'use std\nin std.front {\no := point\nrim := circle(center: o) hint(r: 10)\n}\n'
  + 'body := solid(face(rim), depth: 8)\n';

test('project file navigation isolates undo and clears pending model interactions', (t) => {
  const v = new SketchView(fakeCanvas(), Document.read('a := point hint(x: 10)\n'));
  t.after(() => { v.doc.dispose(); });
  v.pushUndo();
  v.setProgram('a := point hint(x: 20)\n');
  v.setTool('line');
  v.pending = [v.sketch.points[0]];
  v.pauseEditing();
  assert.equal(v.pending.length, 0);
  let loads = 0;
  v.onLoad = () => { loads++; };
  const other = 'b := point hint(x: 30)\n';
  assert.equal(v.openProjectFile(other), true);
  assert.equal(loads, 0, 'switching files must not reset the containing project');
  v.undo();
  assert.equal(v.source, other, 'undo cannot restore another file into this one');
  v.pushUndo();
  v.setProgram('b := point hint(x: 40)\n');
  v.undo();
  assert.equal(v.source, other, 'edits within the new file remain undoable');
});

test('the model canvas calls out its dimensions, and off shows only the one being edited', (t) => {
  t.mock.method(globalThis, 'requestAnimationFrame', () => 1);
  const v = new SketchView(fakeCanvas(), Document.read(examples.source('rect_fillets')));
  t.after(() => { v.doc.dispose(); });
  const painted: string[] = [];
  v.ctx = new Proxy(v.ctx, { get: (target, key) => key === 'fillText'
    ? (text: string) => painted.push(text) : Reflect.get(target, key) });
  const all = callouts(v.sketch, v.unit).items;
  assert.ok(all.length > 1);
  assert.equal(v.showDimensions, true, 'callouts are on by default');
  paintCallouts(v);
  assert.equal(painted.length, all.length);

  v.showDimensions = false;
  painted.length = 0;
  paintCallouts(v);
  assert.equal(painted.length, 0);
  const target = v.sketch.constraintById(all[0].id)!;
  assert.ok(v.startDimension([target], false, null));
  paintCallouts(v);
  assert.deepEqual(painted, [all[0].text]);
  assert.equal(v.showDimensions, false, 'editing does not switch them all back on');
  assert.deepEqual(callouts(v.sketch, v.unit, []).items, []);

  v.load(examples.source('square'));
  assert.equal(v.showDimensions, false, 'the choice is view state and survives a load');
});

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

test('a point in space is dragged where the eye sees it', (t) => {
  const view = new SketchView(fakeCanvas(), Document.read('l := line\n'));
  t.after(() => { view.doc.dispose(); });
  view.orbit = { az: 0.6, el: 0.4 };                   // three quarters: depth is not the page's y
  const cv = view.canvas as ReturnType<typeof fakeCanvas>;
  const end = view.sketch.points[1];
  assert.ok(view.inSpace(end), 'a line no `in` reaches stands in space');
  const [sx, sy] = view.seen(end);
  const before = view.source;

  cv.fire('pointerdown', pointer(sx, sy, { pointerId: 1 }));
  assert.equal(PlanDrag.live, 1, 'a press on a point in space starts a drag');
  cv.fire('pointermove', pointer(sx + 40, sy + 25, { pointerId: 1 }));
  const [ex, ey] = view.seen(end);
  assert.ok(Math.hypot(ex - sx - 40, ey - sy - 25) < 1e-6, `seen at (${ex}, ${ey})`);
  cv.fire('pointerup', pointer(sx + 40, sy + 25, { pointerId: 1 }));
  assert.equal(PlanDrag.live, 0);
  assert.notEqual(view.source, before, 'the drag is written back as the end\'s seeds');
  const [fx, fy] = view.seen(view.sketch.points[1]);
  assert.ok(Math.hypot(fx - sx - 40, fy - sy - 25) < 1e-6, 'and the source says where it went');
});

/* -- navigating: pinch, trackpad ------------------------------------------------------- */

const finger = (x: number, y: number, id: number): Record<string, unknown> =>
  pointer(x, y, { pointerId: id, pointerType: 'touch' });

test('two fingers pinch the workspace about their middle and carry it with them', () => {
  const sk = pinnedApex();
  const view = viewOn(sk);
  view.setTool('point');                     // a press would add a point: a pinch must not
  const cv = view.canvas as ReturnType<typeof fakeCanvas>;
  const before = view.source;
  const scale = view.cam.scale;
  const anchor = view.cam.s2w(400, 300);

  cv.fire('pointerdown', finger(350, 300, 1));
  cv.fire('pointerdown', finger(450, 300, 2));
  cv.fire('pointermove', finger(300, 300, 1));
  cv.fire('pointermove', finger(500, 300, 2));
  assert.ok(Math.abs(view.cam.scale / scale - 2) < 1e-12, 'twice the spread is twice the zoom');
  const still = view.cam.s2w(400, 300);
  assert.ok(Math.hypot(still[0] - anchor[0], still[1] - anchor[1]) < 1e-9,
    'what is between the fingers stays between them');

  cv.fire('pointermove', finger(330, 340, 1));
  cv.fire('pointermove', finger(530, 340, 2));
  const carried = view.cam.s2w(430, 340);
  assert.ok(Math.hypot(carried[0] - anchor[0], carried[1] - anchor[1]) < 1e-9,
    'and moving them together pans');

  cv.fire('pointerup', finger(330, 340, 1));
  cv.fire('pointermove', finger(560, 360, 2));   // the finger left down does nothing
  cv.fire('pointerup', finger(560, 360, 2));
  assert.equal(view.source, before, 'the first finger\'s press was the pinch\'s, not the tool\'s');
  assert.equal(PlanDrag.live, 0);
  assert.equal(view.gesturePointer, null);
});

test('a lone finger is a pointer: it drags, and a second finger settles the drag', () => {
  const sk = pinnedApex();
  const view = viewOn(sk);
  const cv = view.canvas as ReturnType<typeof fakeCanvas>;
  const apex = view.sketch.points[2];
  const [sx, sy] = view.w2s(...apex.xy);

  cv.fire('pointerdown', finger(sx, sy, 1));
  assert.equal(PlanDrag.live, 0, 'held back until it is sure to be one finger');
  cv.fire('pointermove', finger(sx + 20, sy, 1));
  assert.equal(PlanDrag.live, 1, 'moved past the slop, the press is the drag\'s');
  assert.deepEqual(view.selected, [apex]);

  const scale = view.cam.scale;
  cv.fire('pointerdown', finger(sx + 200, sy, 2));
  assert.equal(PlanDrag.live, 0, 'the second finger ended the drag, freeing its handle');
  cv.fire('pointermove', finger(sx + 300, sy, 2));
  assert.ok(view.cam.scale > scale, 'and the two pinch');
  cv.fire('pointerup', finger(sx + 20, sy, 1));
  cv.fire('pointerup', finger(sx + 300, sy, 2));
  assert.equal(PlanDrag.live, 0);
});

test('a tap is a click', () => {
  const sk = pinnedApex();
  const view = viewOn(sk);
  const cv = view.canvas as ReturnType<typeof fakeCanvas>;
  const apex = view.sketch.points[2];
  const [sx, sy] = view.w2s(...apex.xy);
  cv.fire('pointerdown', finger(sx, sy, 1));
  cv.fire('pointerup', finger(sx, sy, 1));
  assert.deepEqual(view.selected, [apex]);
  assert.equal(PlanDrag.live, 0);
  assert.equal(view.gesturePointer, null);
});

test('a trackpad pinches by ctrl and its wheel, and pans by scrolling', () => {
  const view = viewOn(pinnedApex());
  const cv = view.canvas as ReturnType<typeof fakeCanvas>;
  const wheel = (init: Record<string, unknown>): void => cv.fire('wheel',
    { clientX: 400, clientY: 300, deltaX: 0, deltaY: 0, deltaMode: 0, ctrlKey: false, ...init });
  const at = (): [number, number] => view.cam.s2w(400, 300);

  const scale = view.cam.scale, anchor = at();
  wheel({ ctrlKey: true, deltaY: -10 });
  assert.ok(view.cam.scale > scale, 'pinching out zooms in');
  assert.ok(Math.hypot(at()[0] - anchor[0], at()[1] - anchor[1]) < 1e-9, 'about the cursor');

  const zoomed = view.cam.scale, x = view.cam.originX, y = view.cam.originY;
  wheel({ deltaX: 12, deltaY: 5 });
  assert.equal(view.cam.scale, zoomed, 'a scroll does not zoom');
  assert.deepEqual([view.cam.originX, view.cam.originY], [x - 12, y - 5]);
  wheel({ deltaY: 7, wheelDeltaY: -21 });       // straight down, as only a trackpad reports it
  assert.deepEqual([view.cam.originX, view.cam.originY], [x - 12, y - 12]);

  wheel({ deltaY: -100, wheelDeltaY: 120 });    // a wheel's notch still zooms
  assert.ok(view.cam.scale > zoomed);
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
  const cs = callouts(view.sketch, view.unit);
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

  // Preview placements are session state; .sv only records geometry.
  assert.ok(!/at \(/.test(view.source), `no presentation in the model: ${view.source}`);

  view.resetCallouts();
  assert.ok(Math.abs(dimY(view, view.sketch) - before) < 1e-9);
  view.setProgram(view.source, false);
  assert.ok(Math.abs(dimY(view, view.sketch) - before) < 1e-9);

  // and undoing it keeps the drawing: the snapshot it takes is program text, so undo restores a
  // document — where a serialised sketch would come back from `Document.read` as an empty one
  const points = view.sketch.points.length;
  view.undo();
  assert.equal(view.sketch.points.length, points, 'undo kept the drawing');
  assert.ok(/^\w+ := point/.test(view.source), 'undo restored the source, not a dump');
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
  assert.equal(view.sketch.points.length, 4 + 1, 'and the origin of the plane they are drawn in');
  assert.equal(view.sketch.lines.length, 2);
  assert.equal(view.sketch.userConstraints().length, 2, 'the copy brought its own Distance');
});

test('a paste is selected, and lands clear of what it came from', () => {
  const sk = oneLine();
  const view = viewOn(sk);
  view.selected = [view.sketch.lines[0]];
  view.copySelected();
  view.pasteClipboard();

  // the pasted two, after the original's two and the origin of the plane they are drawn in
  const [p3, p4] = view.sketch.points.slice(-2);
  assert.deepEqual(view.selected, [p3, p4, view.sketch.lines[1]]);
  const [x0, y0] = view.sketch.points[0].xy;
  const [x1, y1] = p3.xy;
  assert.ok(x1 > x0 && y1 < y0, `the copy should be nudged clear: ${x1},${y1} vs ${x0},${y0}`);
});

test('successive pastes cascade instead of piling up', () => {
  const sk = oneLine();
  const view = viewOn(sk);
  view.selected = [view.sketch.lines[0]];
  view.copySelected();
  view.pasteClipboard();
  const first = view.sketch.points[3].xy;
  view.pasteClipboard();
  const second = view.sketch.points[5].xy;
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
  assert.deepEqual(pasted.entities(), view.sketch.points.slice(-2));
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
  const before = io.dumps(view.sketch);
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

/** The points a drawing has of its own: not the standard datums' `use std` brings. */
function ownPoints(view: SketchView): Point[] {
  return view.sketch.points.filter((p) => !view.doc.nameOf(p)?.startsWith('std.'));
}

test('a curve fitted through free clicks leaves no points behind and no constraints', () => {
  const view = docView('use std\n');
  fitThrough(view, [[100, 100], [200, 60], [300, 160], [400, 80], [500, 140]]);
  assert.equal(view.sketch.splines.length, 1);
  assert.equal(ownPoints(view).length, 5, 'the control polygon, and nothing else');
  assert.deepEqual(ownPoints(view).map((p) => p.index), view.sketch.splines[0].ctrl.map((p) => p.index));
  assert.equal(view.sketch.userConstraints().length, 0, 'a free click is a place, not a promise');
});

test('a fit click that lands on a point holds the curve to it', () => {
  const view = docView('use std\n');
  // two points already in the sketch, drawn on the front at screen positions the tool will snap to
  const [ax, ay] = view.s2w(200, 60);
  const [bx, by] = view.s2w(400, 80);
  const a = view.sketch.point(ax, ay), b = view.sketch.point(bx, by);
  a.plane = view.plane;
  b.plane = view.plane;
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
  assert.ok(solve(view.sketch).success);
  for (const p of [a, b]) assert.ok(curve.closest(p.x.value, p.y.value).distance < 1e-9);
});

test('an abandoned fit leaves the sketch untouched', () => {
  const view = docView('use std\n');
  const before = io.dumps(view.sketch);
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
  return { view, sk, alt: { a, b, make: pairMaker(a, b) } };
}

/** The three dimensions of a pair, as `commands::pairDim` states them: the length, and the run
 *  and the rise — an ordinate along the view's `x` and `y`. */
function pairMaker(a: Point, b: Point): (kind: PairDimension) => Constraint {
  return (kind) => kind === 'length'
    ? C.build('Distance', [a, b, Math.hypot(b.x.value - a.x.value, b.y.value - a.y.value)])
    : C.build('Ordinate', [a, b, null, kind === 'run' ? b.x.value - a.x.value
      : b.y.value - a.y.value, kind === 'run' ? 'x' : 'y']);
}

test('where the number is put is which dimension it is', () => {
  const { view, sk, alt } = pairToDimension();
  const cv = view.canvas as ReturnType<typeof fakeCanvas>;
  const first = alt.make('length');
  assert.ok(view.startDimension([first], true, alt));
  assert.equal(sk.userConstraints().length, 1, 'stated at once, not after a dialog');

  const at = (dx: number, dy: number): [number, number] => view.w2s(20 + dx, 20 + dy);
  const kind = (): PairDimension | null => pairOf(view.liveDim!.targets[0]);
  cv.fire('pointermove', pointer(...at(-30, 30)));      // across the pair: its own length
  assert.equal(kind(), 'length');
  cv.fire('pointermove', pointer(...at(0, 40)));        // above it: the run
  assert.equal(kind(), 'run');
  assert.equal(view.sketch.userConstraints().length, 1, 'the old one should have gone');
  assert.equal((view.liveDim!.targets[0] as unknown as { d: number }).d, 40);
  cv.fire('pointermove', pointer(...at(40, 0)));        // out to the side: the rise
  assert.equal(kind(), 'rise');

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
  const first = alt.make('length');
  view.startDimension([first], true, alt);
  cv.fire('pointermove', pointer(...view.w2s(20, 60)));
  view.endDimension(false);
  assert.equal(sk.userConstraints().length, 0, 'the constraint should have come back out');
  assert.equal(io.dumps(sk), before, 'and its placement with it');
  assert.equal(view.liveDim, null);

  // accepted, it is one step back — the constraint, where it was put and what it says together
  const c = alt.make('length');
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
  assert.ok(/^\w+ := point/.test(view.source), `undo restored the source, not a dump: ${view.source.slice(0, 40)}`);
});

test('a click plants the number, and the pointer stops carrying it', () => {
  const { view, sk, alt } = pairToDimension();
  const cv = view.canvas as ReturnType<typeof fakeCanvas>;
  const c = alt.make('length');
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
  assert.equal(pairOf(view.liveDim!.targets[0]), 'run');
  cv.fire('pointerup', pointer(...view.w2s(20, 60)));
  view.endDimension(true);
});

test('a pair with a length on it can still be given its run', () => {
  const { view, sk, alt } = pairToDimension();
  const cv = view.canvas as ReturnType<typeof fakeCanvas>;
  const first = alt.make('length');
  view.startDimension([first], true, alt);
  cv.fire('pointermove', pointer(...view.w2s(-10, 50)));    // across the pair: its own length
  view.endDimension(true);
  assert.equal(sk.userConstraints().length, 1);

  // asking again writes a second dimension rather than reopening the first, so the pointer
  // still gets to say which of the three it is — the run, which is a fact the length is not
  const second = alt.make('length');
  assert.ok(view.startDimension([second], true, alt), 'a second dimension was refused');
  cv.fire('pointermove', pointer(...view.w2s(20, 60)));     // above it: the run
  assert.equal(pairOf(view.liveDim!.targets[0]), 'run');
  view.endDimension(true);
  assert.deepEqual(sk.userConstraints().map((c) => pairOf(c)).sort(), ['length', 'run']);
});

test('a dimension already on the drawing is opened, not stated twice', () => {
  const { view, sk, alt } = pairToDimension();
  const c = alt.make('length');
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
  const make = pairMaker(a, b);

  const first = make('length');
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
  const [dx, dy] = view.viewCam().dir(1, 2);
  assert.ok(Math.abs(dx - 1 / Math.sqrt(5)) < 1e-12 && Math.abs(dy + 2 / Math.sqrt(5)) < 1e-12);
  assert.ok(Math.abs(view.viewCam().angle(Math.PI / 4) + Math.PI / 4) < 1e-12);
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
use std
in std.front {
a := point
b := point hint((100, 0))
ab := line(a, b)      // the base
horizontal ab
fix((0, 0)) a
}
`;

function docView(text: string): SketchView {
  const view = new SketchView(fakeCanvas(), Document.read(text));
  view.showDimensions = true;
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
  assert.ok(/\nl0 := line\(p0, p1\)/.test(view.source), view.source);
  // and the source is a document: reading it back gives the same drawing — the same points where
  // they were, drawn where they were (renumbered: what was written comes before what `use std`
  // brings, where the gesture's points came after it)
  const again = Document.read(view.source);
  assert.ok(again.ok, JSON.stringify(again.diagnostics));
  for (const n of ['a', 'b', 'p0', 'p1']) {
    const [p, q] = [pointNamed(view, n), again.entity(n) as Point];
    assert.deepEqual(q.xy, p.xy, n);
    assert.equal(again.nameOf(q.plane!), view.doc.nameOf(p.plane!), n);
  }
  assert.equal(again.sketch.lines.length, view.sketch.lines.length);
  assert.equal(again.sketch.userConstraints().length, view.sketch.userConstraints().length);
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
  assert.ok(view.source.includes('ab := line(a, b)      // the base'));
  assert.ok(!view.source.includes('b := point hint((100, 0))'), `the seed did not move:\n${view.source}`);
  assert.ok(/b := point hint\(/.test(view.source), 'and it is still a seed');
});

test('deleting takes the statements that named it, and leaves the comments', () => {
  const view = docView(ANNOTATED);
  view.selected = [view.sketch.points[1]];
  view.deleteSelected();
  assert.ok(!view.source.includes('b := point'), view.source);
  assert.ok(!view.source.includes('ab := line'), 'the line that named it went too');
  assert.ok(!view.source.includes('horizontal ab'), 'and the constraint on that line');
  assert.ok(view.source.includes('// a base, and this comment must survive every gesture'));
  assert.ok(view.source.includes('fix((0, 0)) a'));
  assert.equal(view.sketch.points.length, 1 + 5, 'a, and the five the standard datums bring');
});

test('undo is the source, so it comes back word for word', () => {
  const view = docView(ANNOTATED);
  view.selected = [view.sketch.points[1]];
  view.deleteSelected();
  view.undo();
  assert.equal(view.source, ANNOTATED, 'undo restored a print-out instead of the document');
  assert.equal(view.sketch.points.length, 2 + 5);
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
  assert.ok(view.source.includes('component Involute(c: circle, phase: Angle, u: Angle) {'));
  assert.ok(view.source.includes('component Flank('));
  assert.ok(view.source.includes('ring N about center {'));
  assert.ok(view.source.includes('g := Gear(N: 30, m: 3, phi: 25, ded: 1)'));
  assert.ok(/p0 := point hint\(\(200, 0\)\)/.test(view.source), view.source);
});


/* -- views: the current plane, membership and projection ----------------------------------
 *
 * A plane is where the next point goes, and that is view state until a point is drawn — at
 * which moment it is document state, written as the point's `in` clause.  What is worth a
 * test is that seam: the membership reaches the source through the same reconcile every
 * gesture goes through, the current plane crosses a re-elaboration the way the selection
 * does, and a projection the core refuses leaves nothing behind. */

/** Two planes of the document's own, the front's and the top's, and a point drawn in
 *  `std.front` away from either plane's origin. */
const VIEWS = `\
use std
front := plane(u: std.x, v: std.z)
fix(origin == (0, 0, 0)) front
top := plane(u: std.x, v: std.y)
fix(origin == (0, 0, 0)) top
o := point in std.front
fix((30, 30)) o
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
  assert.ok(/p0 := point hint\(\(20, 10\)\) in front/.test(view.source), view.source);
  assert.equal(pointNamed(view, 'p0').plane, front);
  // a click on a point of another plane standing there snaps to it, and does not pull it in
  const n = view.sketch.points.length;
  click(view, 30, 30);
  assert.equal(view.sketch.points.length, n, 'the click snapped rather than minting');
  assert.ok(view.source.includes('o := point in std.front\n'), view.source);
  assert.equal(view.doc.nameOf(pointNamed(view, 'o').plane!), 'std.front');
  // and back on the front, the next point is drawn there
  view.choosePlane('std.front');
  click(view, 25, 15);
  assert.ok(/p1 := point hint\(\(25, 15\)\) in std\.front\n/.test(view.source), view.source);
});

test('a projection is one constraint, and the source says `project`', () => {
  const view = docView(`${VIEWS}a := point in front hint((10, 5))\n`
                       + 'b := point in top hint((10, 100))\n');
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
  const view = docView(`${VIEWS}a := point in front hint((10, 5))\n`
                       + 'b := point in front hint((20, 5))\nc := point hint((30, 30))\n');
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
  const view = docView(`${VIEWS}a := point hint((5, 5)) in front\nb := point hint((5, 85)) in top\na project b\n`);
  view.plane = planeNamed(view, 'top');
  // a structural edit re-elaborates the drawing: the plane comes across by name
  assert.ok(view.apply(view.doc.addPoint(1, 2)));
  assert.ok(view.plane instanceof Plane, 'the plane was lost to the re-elaboration');
  assert.equal(view.plane.sketch, view.sketch, 'and is the new drawing\'s');
  assert.equal(view.doc.nameOf(view.plane), 'top');
  // deleting it takes the clauses and the statement; what is drawn next goes on the front
  view.selected = [view.plane];
  view.deleteSelected();
  assert.equal(view.doc.nameOf(view.plane!), 'std.front');
  assert.ok(!view.source.includes('top := plane'), view.source);
  assert.ok(view.source.includes('front := plane'), 'the other plane stays');
  assert.ok(!view.source.includes('project'), `the projection went with it: ${view.source}`);
  assert.ok(view.source.includes('b := point hint((5, 85))\n'), `the clause came out: ${view.source}`);
  assert.equal(view.sketch.userConstraints().length, 0);
  // and a load is another drawing's, whatever it happens to call things: it is drawn on the front
  view.plane = planeNamed(view, 'front');
  view.setProgram(VIEWS);
  assert.equal(view.doc.nameOf(view.plane!), 'std.front');
});

test('the plane tool picks two lines, writes the plane over them, and makes it current', () => {
  const view = docView(`${VIEWS}in std.front {\nab := line(hint((0, 0)), hint((40, 0)))\n`
                       + 'ac := line(hint((0, 0)), hint((0, 30)))\n}\n');
  assert.ok(view.doc.ok, JSON.stringify(view.doc.diagnostics));
  view.choosePlane('std.front');
  const said: string[] = [];
  view.onStatus = (m) => said.push(m);
  view.insertPlane({ name: 'aux' });
  assert.equal(view.tool, 'plane');
  // a click on nothing is no line
  click(view, 20, 20);
  assert.ok(said.some((m) => /click a line/.test(m)), said.join('\n'));
  const planes = view.sketch.planes.length;
  click(view, 20, 0);
  assert.equal(view.sketch.planes.length, planes, 'the first line is half a plane');
  click(view, 0, 15);
  assert.equal(view.sketch.planes.length, planes + 1);
  const aux = planeNamed(view, 'aux');
  assert.equal(view.plane, aux, 'the new plane is the one being drawn in');
  assert.deepEqual(view.selected, [aux]);
  assert.equal(view.tool, 'select', 'armed for one plane, and put down after it');
  assert.ok(view.source.includes('aux := plane(u: ab, v: ac)'), view.source);
  // right along `ab`, up along `ac`: the front's own attitude
  const { u, v } = aux.basis;
  assert.ok(Math.hypot(u[0] - 1, u[1], u[2]) < 1e-9 && Math.hypot(v[0], v[1], v[2] - 1) < 1e-9,
            `${u} ${v}`);
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

const BLOCK = `unit mm
use std
in std.top {
a := point hint((0, 0))
b := point hint((20, 0))
c := point hint((20, 10))
d := point hint((0, 10))
ab := line(a, b)
bc := line(b, c)
cd := line(c, d)
da := line(d, a)
}
block := solid(face(ab, bc, cd, da), depth: 5)
`;

/** The canvas point where the eye, from the view's orbit, sees a point in space. */
function seenAt(view: SketchView, x: [number, number, number]): [number, number] {
  const { az, el } = view.orbit;
  const right = [-Math.sin(az), Math.cos(az), 0];
  const up = [-Math.cos(az) * Math.sin(el), -Math.sin(az) * Math.sin(el), Math.cos(el)];
  const dot = (a: number[]) => a[0] * x[0] + a[1] * x[1] + a[2] * x[2];
  return [view.cam.originX + dot(right) * view.cam.scale,
          view.cam.originY - dot(up) * view.cam.scale];
}

test('a click on a solid selects it by the face it lands on, apart from the drawing', () => {
  const view = docView(BLOCK);
  assert.ok(view.doc.ok, JSON.stringify(view.doc.diagnostics));
  view.orbit = { az: 0.6, el: 0.5 };
  const top = seenAt(view, [10, 5, 0]);
  drag(view, top, top);
  assert.equal(view.selectedSolids.length, 1);
  const [pick] = view.selectedSolids;
  assert.equal(pick.name, 'block');
  assert.ok(pick.face.startsWith('block.'), pick.face);
  assert.deepEqual(view.selected, []);
  // the drawing outranks what is beneath it, and taking hold of it lets the solid go
  const edge = seenAt(view, [10, 0, 0]);
  drag(view, edge, edge);
  assert.deepEqual(view.selected, [view.doc.entity('ab')]);
  assert.deepEqual(view.selectedSolids, []);
  // and back: picking the solid lets the drawing go; shift toggles it out again
  drag(view, top, top);
  assert.deepEqual(view.selected, []);
  assert.equal(view.selectedSolids.length, 1);
  const cv = view.canvas as ReturnType<typeof fakeCanvas>;
  cv.fire('pointerdown', pointer(...top, { shiftKey: true }));
  cv.fire('pointerup', pointer(...top, { shiftKey: true }));
  assert.deepEqual(view.selectedSolids, []);
  // assigning the drawing's selection by any route lets it go, even to nothing — a press on a
  // callout or a constraint's row selects that instead
  drag(view, top, top);
  view.selected = [];
  assert.deepEqual(view.selectedSolids, []);
  // a press on nothing lets it go too
  drag(view, top, top);
  const away = seenAt(view, [10, 80, 0]);
  drag(view, away, away);
  assert.deepEqual(view.selectedSolids, []);
});

test('a solid selected crosses an edit by name, and Delete takes its statement out', () => {
  const view = docView(BLOCK);
  view.orbit = { az: 0.6, el: 0.5 };
  const top = seenAt(view, [10, 5, 0]);
  drag(view, top, top);
  assert.ok(view.apply(view.doc.addEntity('line', ['a', 'c'])));
  assert.deepEqual(view.selectedSolids.map((s) => s.name), ['block'], 'still held, by name');
  view.deleteSelected();
  assert.ok(!view.source.includes('block :='), view.source);
  assert.deepEqual(view.selectedSolids, []);
  assert.ok(view.doc.ok, JSON.stringify(view.doc.diagnostics));
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
  view.choosePlane('top');               // which turns the eye to face it, where it can be drawn on
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

test('drawing on the front stays on the front across a re-elaboration', () => {
  const view = docView(VIEWS);
  const top = planeNamed(view, 'top');
  view.selected = [top];
  assert.equal(view.plane, top);
  view.choosePlane('std.front');
  assert.equal(view.doc.nameOf(view.plane!), 'std.front');
  assert.ok(!view.selected.includes(top), 'the view stops being the subject');
  // a structural edit re-elaborates and rebinds the selection: the plane must not come back
  assert.ok(view.apply(view.doc.addPoint(3, 4)));
  assert.equal(view.doc.nameOf(view.plane!), 'std.front', 'the rebind re-armed the old plane');
  view.setTool('point');
  click(view, 12, 12);
  assert.equal(view.doc.nameOf(view.sketch.points.at(-1)!.plane!), 'std.front',
               'and the next point is on the front');
});

/* -- the workspace: every sketch on its own plane, seen by one eye ---------------------------
 *
 * Where a view stands in space and how the eye sees it is the core's, and tested there
 * (`workspace.rs`).  What is worth a test here is the view's side of it: the plane chooser's two
 * halves — where the next thing goes, and where the eye turns — a standard plane coming in on its
 * first use, a drag that stays in its own plane, the camera's buttons and what a fit frames. */

const FRONT = { az: -Math.PI / 2, el: 0 };

function closeTo(a: readonly number[], b: readonly number[], tol = 1e-6): boolean {
  return a.length === b.length && a.every((x, i) => Math.abs(x - b[i]) < tol);
}

test('a flat drawing opens square on to the front, and one with a solid from three quarters', () => {
  const view = docView(examples.source('rect_fillets'));
  view.load(examples.source('rect_fillets'));
  assert.ok(closeTo([view.orbit.az, view.orbit.el], [FRONT.az, FRONT.el]), JSON.stringify(view.orbit));
  view.load(PROJECTED_CIRCLE);
  assert.ok(view.orbit.el > 0.1, `a solid is looked at from above the horizon: ${view.orbit.el}`);
  view.doc.dispose();
});

test('the chooser offers the standard planes and the document\'s own, and turns the eye to one', () => {
  const view = docView(VIEWS);
  assert.deepEqual(view.planeChoices(), ['std.front', 'std.side', 'std.top', 'front', 'top']);
  assert.equal(view.planeName, 'std.front', 'a document is drawn on the front until told');
  view.choosePlane('top');
  assert.equal(view.plane, planeNamed(view, 'top'));
  assert.equal(view.planeName, 'top');
  assert.ok(Math.abs(view.orbit.el - Math.PI / 2) < 1e-9, `straight down on it: ${view.orbit.el}`);
  // the next point is drawn where the pointer is, on the top plane
  view.setTool('point');
  click(view, 20, 110);
  assert.ok(/p0 := point hint\(\(20(\.\d+)?, 110(\.\d+)?\)\) in top/.test(view.source), view.source);
  view.choosePlane('std.front');
  assert.equal(view.doc.nameOf(view.plane!), 'std.front');
  assert.ok(closeTo([view.orbit.el], [0]), 'and square on to it again');
  view.choosePlane('nowhere');
  assert.equal(view.doc.nameOf(view.plane!), 'std.front', 'a name that is no plane changes nothing');
});

test('a standard plane the document does not have comes in with `use std` on the first press', () => {
  const view = docView('w := 100\n');
  view.choosePlane('std.side');
  assert.equal(view.pendingPlane, 'std.side');
  assert.equal(view.source, 'w := 100\n', 'choosing a plane writes nothing');
  assert.ok(closeTo([view.orbit.az, view.orbit.el], [0, 0]), 'looking at the side from +x');
  view.setTool('point');
  click(view, 10, 20);
  assert.equal(view.pendingPlane, null);
  assert.equal(view.doc.nameOf(view.plane!), 'std.side');
  assert.ok(view.source.startsWith('use std\n'), view.source);
  // a second point: the first keeps its clause, and nothing of the library is copied in
  click(view, 30, 5);
  assert.equal(view.source.match(/ in std\.side/g)?.length, 2, view.source);
  assert.ok(!view.source.includes('fix('), view.source);
  // and undo takes the whole of it back, the `use` included
  view.undo();
  view.undo();
  view.undo();
  assert.equal(view.source, 'w := 100\n');
});

test('double-clicking a pane draws on its plane', () => {
  const view = docView('use std\nt := point in std.top hint((40, 40))\n');
  const cv = view.canvas as ReturnType<typeof fakeCanvas>;
  const top = planeNamed(view, 'std.top');
  view.orbit = { az: -Math.PI / 3, el: 1.2 };       // from above, the front and side all but edge on
  const at = view.inView(top.index, () => view.w2s(30, 30))!;
  // with a tool down a double-click is two of its clicks, and chooses nothing
  view.setTool('line');
  cv.fire('dblclick', { clientX: at[0], clientY: at[1] });
  assert.equal(view.planeName, 'std.front');
  view.setTool('select');
  cv.fire('dblclick', { clientX: at[0], clientY: at[1] });
  assert.equal(view.plane, top);
  assert.ok(closeTo([view.orbit.az, view.orbit.el], [-Math.PI / 3, 1.2]), 'the eye stays put');
  // and off every pane, nothing changes
  view.choosePlane('std.front', false);
  const far = view.inView(top.index, () => view.w2s(400, -400))!;
  cv.fire('dblclick', { clientX: far[0], clientY: far[1] });
  assert.equal(view.doc.nameOf(view.plane!), 'std.front');
});

test('a point on a plane seen at an angle drags along that plane', () => {
  const view = docView(`${VIEWS}a := point in top hint((10, 120))\n`);
  view.orbit = { az: -Math.PI / 3, el: Math.PI / 6 };
  const a = pointNamed(view, 'a');
  const at = view.seen(a);
  const to: [number, number] = [at[0] + 30, at[1] - 10];
  drag(view, at, to);
  const moved = pointNamed(view, 'a');
  // where the pointer let go, read on the top plane, is where the point now is
  const want = view.inView(view.viewOf(moved), () => view.s2w(...to))!;
  assert.ok(closeTo(moved.xy, want, 1e-4), `${moved.xy} ${want}`);
  assert.ok(closeTo(view.seen(moved), to, 1e-3), 'and it is seen under the pointer');
  assert.equal(moved.plane, planeNamed(view, 'top'));
});

test('a plane seen edge on takes no point: the press says why', () => {
  const view = docView(VIEWS);
  view.plane = planeNamed(view, 'top');     // current, but the eye is still square on the front
  let said = '';
  view.onStatus = (m) => { said = m; };
  view.setTool('point');
  const n = view.sketch.points.length;
  const cv = view.canvas as ReturnType<typeof fakeCanvas>;
  cv.fire('pointerdown', pointer(400, 300));
  cv.fire('pointerup', pointer(400, 300));
  assert.equal(view.sketch.points.length, n);
  assert.ok(said.includes('edge on'), said);
});

test('the right button turns the workspace and the middle one slides it; neither edits', () => {
  const view = docView(examples.source('rect_fillets'));
  const before = view.source;
  const cv = view.canvas as ReturnType<typeof fakeCanvas>;
  const orbit = { ...view.orbit };
  cv.fire('pointerdown', pointer(400, 300, { button: 2, buttons: 2 }));
  cv.fire('pointermove', pointer(460, 280, { button: 2, buttons: 2 }));
  cv.fire('pointerup', pointer(460, 280, { button: 2, buttons: 0 }));
  assert.ok(!closeTo([view.orbit.az, view.orbit.el], [orbit.az, orbit.el]), 'the eye turned');
  const x0 = view.cam.originX;
  cv.fire('pointerdown', pointer(400, 300, { button: 1, buttons: 4 }));
  cv.fire('pointermove', pointer(430, 300, { button: 1, buttons: 4 }));
  cv.fire('pointerup', pointer(430, 300, { button: 1, buttons: 0 }));
  assert.equal(view.cam.originX, x0 + 30, 'the middle button slides it');
  assert.equal(view.source, before, 'and nothing in the document moved');
});

test('a click picks what is seen under it, though another view lies there on the page', () => {
  const view = docView('use std\nf := point hint((10, 10)) in std.front\nfix((10, 10)) f\n'
    + 's := point hint((10, 10)) in std.side\nfix((10, 10)) s\n');
  view.orbit = { az: -Math.PI / 4, el: Math.PI / 6 };
  const f = pointNamed(view, 'f'), s = pointNamed(view, 's');
  assert.notDeepEqual(view.seen(f), view.seen(s), 'one place on the page, two in space');
  assert.equal(view.pick(...view.seen(f)), f);
  assert.equal(view.pick(...view.seen(s)), s);
  // and a band round one of them takes that one alone
  const cv = view.canvas as ReturnType<typeof fakeCanvas>;
  const [x, y] = view.seen(s);
  cv.fire('pointerdown', pointer(x - 6, y - 6));
  cv.fire('pointermove', pointer(x + 6, y + 6));
  cv.fire('pointerup', pointer(x + 6, y + 6));
  assert.deepEqual(view.selected, [s]);
});

test('a fit frames everything the workspace shows, solids included', () => {
  const view = docView(examples.source('solid_flange'));
  assert.ok(solve(view.sketch).success);
  view.orbit = { ...FRONT };
  view.fit();
  // the section draws only the half right of the axis; the turned part reaches as far left
  const axis = view.seen(pointNamed(view, 'std.origin'));
  const rim = view.seen(pointNamed(view, 'b'));
  const reach = rim[0] - axis[0];
  assert.ok(reach > 0);
  assert.ok(axis[0] - reach >= 0, `the far side of the turned part is on screen: ${axis[0] - reach}`);
  assert.ok(rim[0] <= view.width);
  view.doc.dispose();
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
  assert.equal(view.source, 'use std\n', 'back to the sheet it replaced');
});

/** The spatial demos the menu offers as files: each opens and solves, and the one freedom the
 *  sphere demo leaves — a point of the side view on the ball — drags, staying on the ball. */
test('the spatial demos open, and the point on the ball drags', async () => {
  const { readFile } = await import('node:fs/promises');
  const files: Record<string, string> =
    JSON.parse(await readFile(new URL('../examples/sources.json', import.meta.url), 'utf8'));
  for (const name of ['skew_axes', 'hypoid_pitch_cones', 'sphere_cone_cylinder']) {
    const view = docView(files[`${name}.sv`]);
    assert.ok(view.doc.ok, `${name}: ${JSON.stringify(view.doc.diagnostics)}`);
    assert.ok(solve(view.sketch).success, `${name} solves`);
    if (name !== 'sphere_cone_cylinder') continue;
    // the side view is `std.side`, the plane x = 0, its x the world's y and up as up; the ball
    // is 12 about (-8, 0, 38).  Looked at from three quarters, the point is dragged on the side
    // view's plane, wherever it is seen
    view.orbit = { az: -Math.PI / 4, el: Math.PI / 6 };
    const pb = pointNamed(view, 'pb');
    const onBall = ([x, y]: [number, number]) => Math.hypot(8, x, y - 38);
    const before = pb.xy;
    assert.ok(Math.abs(onBall(before) - 12) < 1e-6, `pb on the ball: ${onBall(before)}`);
    const at = view.seen(pb);
    drag(view, at, [at[0] + 10, at[1] - 25]);
    const after = pointNamed(view, 'pb').xy;
    assert.notDeepEqual(after, before, 'the point moved');
    assert.ok(Math.abs(onBall(after) - 12) < 1e-6, `and is still on the ball: ${onBall(after)}`);
  }
});

test('a dimension over a param opens as written, and an edit of it goes into the source', (t) => {
  t.mock.method(globalThis, 'requestAnimationFrame', () => 1);
  const v = new SketchView(fakeCanvas(), Document.read(examples.source('rect_fillets')));
  t.after(() => { v.doc.dispose(); });
  const width = (): Constraint => v.sketch.userConstraints()
    .find((c) => c.typeName === 'Distance' && c.written === 'w')!;
  const c = width();
  assert.ok(c, 'the width is written `w`, and the record says so');
  assert.equal(c.d, 100, 'and holds the number the param came to');

  // an expression over the other param: the source says it, and the drawing holds to it
  assert.ok(v.startDimension([c], false, null));
  assert.equal(v.rewriteDimension('2 * h'), null);
  assert.equal(v.liveDim, null, 'accepting it ends the dimension');
  assert.match(v.source, /l1 distance\(2 \* h\) r2/);
  const now = v.sketch.userConstraints().find((k) => k.written === '2 * h');
  assert.ok(now, 'drawn as written');
  assert.equal(now.d, 120, 'h is the param, not an unknown of the sketch');
  v.undo();
  assert.match(v.source, /l1 distance\(w\) r2/, 'one undo step back');

  // a text that does not elaborate is refused, and the dimension stays open to correct it
  assert.ok(v.startDimension([width()], false, null));
  assert.notEqual(v.rewriteDimension('2 *'), null);
  assert.ok(v.liveDim, 'still being edited');
  assert.match(v.source, /l1 distance\(w\) r2/);
  v.endDimension(false);
});

test('a point in space is drawn and picked where it stands, whichever way the eye looks', () => {
  const view = docView('use std\np := point hint((10, 20, 30))\n'
    + 'in std.front {\n  f := point\n  fix((10, 30)) f\n}\n'
    + 'in std.top {\n  t := point\n  fix((10, 20)) t\n}\n');
  const named = (n: string) => view.doc.entity(n) as Point;
  const [p, f, t] = [named('p'), named('f'), named('t')];
  // square on to the front, p is seen where its x and z are; from above, where its x and y are
  view.choosePlane('std.front');
  assert.ok(closeTo(view.seen(p), view.seen(f)), `${view.seen(p)} vs ${view.seen(f)}`);
  view.choosePlane('std.top');
  assert.ok(closeTo(view.seen(p), view.seen(t)), `${view.seen(p)} vs ${view.seen(t)}`);
  // from the side the three stand apart, and a click on p picks p
  view.choosePlane('std.side');
  assert.equal(view.pick(...view.seen(p)), p, 'a click on it picks it');
  view.doc.dispose();
});
