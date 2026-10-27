/* The parts as the 3D box draws them (#674–#677): an object one mesh and one line set, drawn
 * from one indexed geometry its binding's objects share; each in its part's colour — the
 * palette's, the same for every placement of a part and across a reload with other numbers, or
 * the drawing's beside the model (`plates/plates.svd`); and what is selected or under the
 * pointer tinted over that colour, never painted in place of it. */
import assert from 'node:assert/strict';
import { readFile } from 'node:fs/promises';
import test from 'node:test';

import { objects } from '../core/mesh.js';
import { Document } from '../core/program.js';
import { initCore } from '../core/wasm.js';
import { SketchView } from '../app/view.js';
import { fakeCanvas } from './canvas.js';

(globalThis as { requestAnimationFrame?: unknown }).requestAnimationFrame ??=
  (fn: FrameRequestCallback) => { fn(0); return 0; };
(globalThis as { cancelAnimationFrame?: unknown }).cancelAnimationFrame ??= () => {};

await initCore();
const files: Record<string, string> =
  JSON.parse(await readFile(new URL('../examples/sources.json', import.meta.url), 'utf8'));
const PLATES = files['plates/plates.sv'];

/** A view on `text`, solved, as the page opens it. */
function opened(text: string): SketchView {
  const view = new SketchView(fakeCanvas(), Document.read(text));
  assert.ok(view.doc.ok, JSON.stringify(view.doc.diagnostics));
  view.afterEdit();
  return view;
}

/** The index of the object `name`. */
function solid(view: SketchView, name: string): number {
  const o = objects(view.sketch).find((s) => s.name === name);
  assert.ok(o, `no object ${name}`);
  return o.index;
}

interface Held { geometry: number; mesh: { geometry: { index: { count: number } | null } } | null;
                 edges: unknown | null }

test('an object is one mesh and one line set, drawn from a geometry its binding shares', (t) => {
  assert.ok(PLATES, 'plates/plates.sv is packed');
  const view = opened(PLATES);
  t.after(() => view.doc.dispose());
  const box = view.box3d;
  box.sync(view);
  const placed = (box as unknown as { placed: Map<number, Held> }).placed;
  assert.equal(placed.size, 3, 'three objects');
  for (const [, held] of placed) {
    assert.ok(held.mesh, 'its faces, one mesh');
    assert.ok(held.edges, 'its edges, one line set');
    assert.equal(held.mesh.geometry.index!.count / 3, 12, 'a plate is twelve triangles, indexed');
  }
  const [one, two, three] = ['one.body', 'two.body', 'three.body'].map((n) => placed.get(solid(view, n))!);
  assert.equal(one.geometry, two.geometry, 'two placements of one part, one geometry');
  assert.notEqual(one.geometry, three.geometry);
  assert.equal(box.stats.built, 2, 'two geometries built for three objects');
  // a wireframe: the edges alone
  view.setShowSolid(false);
  box.sync(view);
  for (const [, held] of placed) assert.ok(!held.mesh && held.edges, 'edges, no faces');
});

test('every placement of a part is one colour, the palette\'s or the drawing\'s', (t) => {
  const view = opened(PLATES);
  t.after(() => view.doc.dispose());
  const box = view.box3d;
  box.sync(view);
  const colour = (v: SketchView, n: string) => v.box3d.colours().get(solid(v, n));
  const palette = ['one.body', 'two.body', 'three.body'].map((n) => colour(view, n));
  assert.ok(palette.every((c) => /^#[0-9a-f]{6}$/.test(c ?? '')), JSON.stringify(palette));
  assert.equal(palette[0], palette[1], 'one part, one colour');
  assert.notEqual(palette[0], palette[2], 'neighbouring parts differ');
  // read again with the third plate wider: a numeric edit moves no part's colour
  const wider = opened(PLATES.replace('Plate(w: 40mm', 'Plate(w: 44mm'));
  t.after(() => wider.doc.dispose());
  wider.box3d.sync(wider);
  assert.deepEqual(['one.body', 'two.body', 'three.body'].map((n) => colour(wider, n)), palette);
  // the drawing beside the model names two parts' colours
  view.partDrawing = { text: files['plates/plates.svd'], source: 'plates/plates.svd', files,
                       model: 'plates/plates.sv' };
  assert.equal(view.colourError(), null);
  box.sync(view);
  assert.deepEqual(['one.body', 'two.body', 'three.body'].map((n) => colour(view, n)),
                   ['#e8a33d', '#e8a33d', '#4f7cac']);
  // a drawing that does not read leaves the palette's, and says why
  view.partDrawing = { ...view.partDrawing, text: 'model m from "plates.sv"\nstyle m.one { fill: no }\n' };
  assert.match(view.colourError() ?? '', /fill must be #RGB or #RRGGBB/);
  box.sync(view);
  assert.deepEqual(['one.body', 'two.body', 'three.body'].map((n) => colour(view, n)), palette);
});

test('a face selected is tinted over its part\'s colour, not painted in place of it', (t) => {
  const view = opened(PLATES);
  t.after(() => view.doc.dispose());
  const box = view.box3d;
  box.sync(view);
  const one = solid(view, 'one.body');
  const before = box.colours().get(one);
  const faces = (box as unknown as { placed: Map<number, { faces: { path: string }[] }> })
    .placed.get(one)!.faces;
  const face = faces[2].path;
  view.pickSolid({ name: 'one.body', index: one, face });
  box.sync(view);
  assert.equal(box.colours().get(one), before, 'the part keeps its colour');
  const tints = box.tints().get(one);
  assert.ok(tints, 'the object is tinted');
  assert.equal(tints.filter(Boolean).length, 1, 'one face of it');
  assert.ok(tints[2], `the face picked, ${face}`);
  view.pickSolid({ name: 'one.body', index: one, face: '' });
  box.sync(view);
  assert.ok(box.tints().get(one)?.every((x) => x), 'the body whole, every face');
});
