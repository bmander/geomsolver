/* What the app tests share: a view on a document over the stubbed canvas, and the pointer gestures
 * a test makes on one — a click on the page, with modifiers held, where the eye sees a place in
 * space, and a double-click on the canvas. */
import assert from 'node:assert/strict';

import { Document } from '../core/program.js';
import { eyeFrame } from '../core/workspace.js';
import { SketchView } from '../app/view.js';
import { walk } from '../app/hidden.js';
import type { Row } from '../core/outline.js';
import { canvasOf, fakeCanvas, pointer } from './canvas.js';

/** A view on a Solvent document, its callouts shown and nothing solved behind the test's back.
 *  The document must elaborate: a test about a gesture is no test on a drawing that is not
 *  there. */
export function docView(text: string): SketchView {
  const view = new SketchView(fakeCanvas(), Document.read(text));
  view.showDimensions = true;
  view.autoSolve = false;
  assert.ok(view.doc.ok, JSON.stringify(view.doc.diagnostics));
  return view;
}

/** A click with a drawing tool down. */
export function click(view: SketchView, x: number, y: number): void {
  clickWith(view, x, y, {});
}

/** A press and release at a place on the page, with modifiers held. */
export function clickWith(view: SketchView, x: number, y: number,
                          mods: Record<string, boolean>): void {
  const cv = canvasOf(view);
  cv.fire('pointerdown', pointer(...view.w2s(x, y), mods));
  cv.fire('pointerup', pointer(...view.w2s(x, y), mods));
}

/** The canvas point where the eye, from the view's orbit, sees a point in space. */
export function seenAt(view: SketchView, x: [number, number, number]): [number, number] {
  const { right, up } = eyeFrame(view.orbit.az, view.orbit.el);
  const dot = (a: number[]) => a[0] * x[0] + a[1] * x[1] + a[2] * x[2];
  return [view.cam.originX + dot(right) * view.cam.scale,
          view.cam.originY - dot(up) * view.cam.scale];
}

/** A click where the eye sees a place in space. */
export function clickSeen(view: SketchView, x: [number, number, number]): void {
  const cv = canvasOf(view);
  cv.fire('pointerdown', pointer(...seenAt(view, x)));
  cv.fire('pointerup', pointer(...seenAt(view, x)));
}

/** A double-click at a canvas point. */
export function dblclickAt(view: SketchView, at: [number, number]): void {
  canvasOf(view).fire('dblclick', { clientX: at[0], clientY: at[1] });
}

/** The outline's row carried by `path`. */
export function row(v: SketchView, path: string): Row {
  for (const [r] of walk(v.outline())) if (r.path === path) return r;
  throw new Error(`no row ${path}`);
}
