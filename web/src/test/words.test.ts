/* The Words menu's ground: the lexicon as the binding reads it, a word written through the
 * document, and the `word` tool — a slot of the argument box armed — taking from the workspace
 * only what the slot takes.  The core's own gate is `rust/gcs-core/tests/lexicon.rs`. */
import assert from 'node:assert/strict';
import test from 'node:test';

import { Line, Point } from '../core/model.js';
import { initCore } from '../core/wasm.js';
import type { Hit } from '../app/view.js';
import { fitsSelection, hitNamed, selectionNamed } from '../app/wordbox.js';
import { clickSeen, docView } from './views.js';

(globalThis as { requestAnimationFrame?: unknown }).requestAnimationFrame ??=
  (fn: FrameRequestCallback) => { fn(0); return 0; };
(globalThis as { cancelAnimationFrame?: unknown }).cancelAnimationFrame ??= () => {};

await initCore();

/** Two lines apart and a circle on the front plane, nothing holding them.  The front's `u` is
 *  the world's x and its `v` the world's z. */
const DOC = 'use std\nin std.front {\np := point hint((0, 0))\nq := point hint((40, 0))\n'
  + 'l := line(p, q)\nr := point hint((0, 30))\ns := point hint((40, 30))\nm := line(r, s)\n'
  + 'c := point hint((80, 60))\nk := circle(center: c) hint(r: 10)\n}\n';

test('the lexicon is every word, with what each takes', () => {
  const v = docView(DOC);
  const words = v.doc.lexicon();
  const word = (id: string) => words.find((w) => w.id === id)!;
  for (const id of ['line', 'distance', 'tangent', 'std.place.right_of', 'std.solids.Prism']) {
    assert.ok(word(id), id);
  }
  const d = word('distance');
  assert.equal(d.form, 'infix');
  assert.deepEqual(d.slots.find((s) => s.role === 'value')?.type, 'Length');
  // a pair the elaborator settles only with a selector says which
  assert.deepEqual(word('tangent').pairs.find(([a, b]) => a === 'line' && b === 'line')?.[2], ['at']);
  // read once per text
  assert.equal(v.doc.lexicon(), words);
});

test('a word written is the statement the core composes, its module brought in', () => {
  const v = docView(DOC);
  const r = { id: 'std.place.right_of', args: { b: 'q', a: 'p', d: '40' } };
  assert.deepEqual(v.doc.wordText(r), { text: 'q right_of(d: 40) p' });
  assert.ok(v.apply(v.doc.writeWord(r)));
  assert.match(v.source, /^use std\.place \(right_of\)$/m);
  assert.match(v.source, /^q right_of\(d: 40\) p$/m);
  // what is not said yet is said, and nothing is written
  const short = v.doc.wordText({ id: 'distance', args: { p: 'p' } });
  assert.equal(short.refused, 'distance still needs `value`');
  const refused = v.doc.writeWord({ id: 'radius', args: { circle: 'l', value: '5' } });
  assert.ok(refused.refused);
});

test('an armed slot takes only what it takes, and a click on nothing places a point', () => {
  const v = docView(DOC);
  const taken: (string | null)[] = [];
  v.setTool('word');
  v.wordPick = {
    mints: false,
    accepts: (h: Hit) => hitNamed(v, h)?.kind === 'circle',
    take: (h) => taken.push(h && hitNamed(v, h)!.name),
  };
  clickSeen(v, [20, 0, 0]);           // on line l: not a circle
  clickSeen(v, [90, 0, 60]);          // on the circle's rim
  assert.deepEqual(taken, ['k']);
  // a slot that places a point is handed the click on nothing
  v.wordPick = { mints: true, accepts: () => false, take: (h) => taken.push(h ? 'hit' : 'nothing') };
  clickSeen(v, [200, 0, 200]);
  assert.deepEqual(taken, ['k', 'nothing']);
  v.wordPick = null;
  v.setTool('select');
});

test('a word fits the selection where it takes every selected thing', () => {
  const v = docView(DOC);
  const words = v.doc.lexicon();
  const word = (id: string) => words.find((w) => w.id === id)!;
  const l = v.doc.entity('l'), m = v.doc.entity('m');
  assert.ok(l instanceof Line && m instanceof Line);
  v.selected = [l, m];
  assert.ok(fitsSelection(selectionNamed(v), word('parallel')));
  assert.ok(fitsSelection(selectionNamed(v), word('equal')));
  assert.ok(!fitsSelection(selectionNamed(v), word('radius')), 'a radius takes one circle, not two lines');
  const p = v.doc.entity('p');
  assert.ok(p instanceof Point);
  v.selected = [p, l];
  assert.ok(fitsSelection(selectionNamed(v), word('distance')));
  assert.ok(!fitsSelection(selectionNamed(v), word('parallel')));
});
