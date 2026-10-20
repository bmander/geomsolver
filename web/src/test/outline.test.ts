/* The model outline (#435): the document's instances as a tree, and hidden as view state — carried
 * across an edit and an undo by path, let go by a load, never written to the source.  Isolate and
 * Show all, a row framed by double-click, and a selection made in the workspace opening the tree
 * to its row. */
import assert from 'node:assert/strict';
import test from 'node:test';

import type { Row } from '../core/outline.js';
import { Point, fromWire } from '../core/model.js';
import type { Primitive } from '../core/model.js';
import { frameRow, isHidden, rowKey, selectRow, selectionRow, walk } from '../app/hidden.js';
import type { SketchView } from '../app/view.js';
import { initCore } from '../core/wasm.js';
import { docView } from './views.js';

(globalThis as { requestAnimationFrame?: unknown }).requestAnimationFrame ??=
  (fn: FrameRequestCallback) => { fn(0); return 0; };
(globalThis as { cancelAnimationFrame?: unknown }).cancelAnimationFrame ??= () => {};

await initCore();

/** A component instance, a ring, an object made of two features and `use std` — the core's gate
 *  document (`tests/outline.rs`). */
const DOC = 'unit mm\nuse std\n'
  + 'component Pin(c: point) {\n  q := point hint((3, 0))\n  c distance(3) q\n  arm := line(c, q)\n}\n'
  + 'in std.front {\n  hub := point\n  fix((0, 0)) hub\n'
  + '  ring 3 about hub {\n    tip := point hint((10, 1))\n    hub distance(10) tip\n'
  + '    chord := line(tip, next.tip)\n  }\n'
  + '  pin := Pin(hub)\n  o := point hint((40, 0))\n  fix((40, 0)) o\n'
  + '  rim := circle(center: o) hint(r: 20)\n  h := circle(center: o) hint(r: 5)\n  line(o, hub)\n'
  + '  seg := line(hint((0, -30)), hint((10, -30)))\n}\n'
  + 'stock := solid(face(rim), depth: 8)\nbore := solid(face(h), depth: 8)\nbore cut stock\n';

/** The row carried by `path`. */
function row(v: SketchView, path: string): Row {
  for (const [r] of walk(v.outline())) if (r.path === path) return r;
  throw new Error(`no row ${path}`);
}

/** What the row carried by `path` made — by the outline's own name for it, `tip[1]` for a copy
 *  of a ring's, which the source map files as `#9.1.tip`. */
function ent(v: SketchView, path: string): Primitive {
  const [kind, index] = row(v, path).ents[0];
  const e = v.doc.entityOf({ kind: fromWire(kind), index });
  assert.ok(e, path);
  return e;
}

test('the outline lists the document as it is written, std last', (t) => {
  const v = docView(DOC);
  t.after(() => v.doc.dispose());
  const rows = v.outline();
  assert.deepEqual(rows.map((r) => r.label),
    ['hub', '{tip, chord}[0‥2]', 'pin : Pin', 'o', 'rim', 'h', 'line', 'seg', 'stock', 'std']);
  const pin = row(v, 'pin');
  assert.equal(pin.component, 'Pin');
  // a span is the string's, ready for the editor's caret
  assert.ok(pin.span);
  assert.equal(v.source.slice(pin.span.lo, pin.span.hi), 'pin := Pin(hub)');
  assert.equal(rows[9].span, undefined, "std's datums are not the document's to go to");
});

test('a hidden member stays hidden by path across an edit and its undo', (t) => {
  const v = docView(DOC);
  t.after(() => v.doc.dispose());
  v.fit();
  const q = ent(v, 'pin.q') as Point;
  assert.equal(v.pick(...v.seen(q)), q);
  // hiding something selected deselects it
  v.selected = [q];
  v.setRowHidden(row(v, 'pin'), true);
  assert.deepEqual(v.selected, []);
  assert.ok(v.hiddenEnts.has(q) && v.hiddenEnts.has(ent(v, 'pin.arm')));
  assert.ok(!v.hiddenEnts.has(ent(v, 'hub')), 'what the instance reads is not what it made');
  assert.notEqual(v.pick(...v.seen(q)), q, 'hidden is not picked');
  const text = v.source;

  // an edit elsewhere: a new drawing, the member hidden on it by its path
  assert.ok(v.apply(v.doc.addPoint(80, 80)));
  const edited = v.source;
  assert.ok(edited.length > text.length);
  assert.ok(v.hiddenEnts.has(ent(v, 'pin.q')), 'carried across the edit');
  assert.ok(!v.source.includes('hidden'), 'never written to the source');
  // and its undo
  v.undo();
  assert.ok(v.source.length < edited.length, 'the point is gone again');
  assert.ok(v.hiddenRows.has('pin'));
  const q2 = ent(v, 'pin.q') as Point;
  assert.ok(v.hiddenEnts.has(q2), 'carried across the undo');
  assert.notEqual(v.pick(...v.seen(q2)), q2);

  // selecting a hidden row from the tree shows it, and it alone
  selectRow(v, row(v, 'pin.q'));
  assert.deepEqual(v.selected, [q2]);
  assert.ok(!v.hiddenEnts.has(q2));
  assert.ok(v.hiddenEnts.has(ent(v, 'pin.arm')), 'its sibling stays hidden');
  assert.ok(!isHidden(v, row(v, 'pin')));
});

test('Isolate hides all but the selection and what made it; Show all shows it again', (t) => {
  const v = docView(DOC);
  t.after(() => v.doc.dispose());
  // nothing selected: refused, saying why
  let said = '';
  v.onStatus = (m) => { said = m; };
  assert.equal(v.isolate(), false);
  assert.match(said, /select/);

  const tip = ent(v, 'tip[1]');
  v.selected = [tip];
  assert.ok(v.isolate());
  assert.ok(!v.hiddenEnts.has(tip));
  for (const gone of ['hub', 'o', 'rim', 'tip[0]', 'tip[2]', 'chord[0]', 'pin.q', 'std.front']) {
    assert.ok(v.hiddenEnts.has(ent(v, gone)), gone);
  }
  assert.ok(v.hiddenSolids.size > 0, 'the object too');
  // the ring and the copy that made the selection are shown, the other copies not
  assert.ok(!isHidden(v, row(v, 'tip,chord')));
  assert.ok(!isHidden(v, row(v, 'tip,chord[1]')));
  assert.ok(isHidden(v, row(v, 'tip,chord[0]')));
  assert.deepEqual(v.selected, [tip], 'what is isolated stays selected');

  v.showAll();
  assert.equal(v.hiddenRows.size, 0);
  assert.equal(v.hiddenEnts.size, 0);
  assert.equal(v.hiddenSolids.size, 0);
});

test('a hidden object leaves the box, and comes back', (t) => {
  const v = docView(DOC);
  t.after(() => v.doc.dispose());
  const box = v.box3d;
  const placed = () => (box as unknown as { placed: Map<number, unknown> }).placed.size;
  box.sync(v);
  assert.equal(placed(), 1);
  v.setRowHidden(row(v, 'stock'), true);
  box.sync(v);
  assert.equal(placed(), 0);
  v.setRowHidden(row(v, 'stock'), false);
  box.sync(v);
  assert.equal(placed(), 1);
});

test('double-click frames a row: its subtree alone', (t) => {
  const v = docView(DOC);
  t.after(() => v.doc.dispose());
  v.fit();
  const whole = v.unit;
  assert.ok(frameRow(v, row(v, 'pin')));
  assert.ok(v.unit < whole / 4, `zoomed in on the pin: ${v.unit} against ${whole}`);
  for (const name of ['hub', 'pin.q']) {
    const [x, y] = v.seen(ent(v, name) as Point);
    assert.ok(x >= 0 && x <= v.width && y >= 0 && y <= v.height, `${name} on screen`);
  }
  const [far] = v.seen(ent(v, 'o') as Point);
  assert.ok(far > v.width, 'what the row did not make is not framed');
});

test('a selection in the workspace opens the tree to its row', (t) => {
  const v = docView(DOC);
  t.after(() => v.doc.dispose());
  v.selected = [ent(v, 'chord[2]')];
  const at = selectionRow(v);
  assert.ok(at);
  assert.equal(at.row.path, 'chord[2]');
  assert.deepEqual(at.above.map((r) => r.label), ['{tip, chord}[0‥2]', '{tip, chord}[2]']);
  // an object picked opens to the object's row
  const stock = v.solidPick(row(v, 'stock').solid!);
  assert.ok(stock);
  v.pickSolid(stock);
  assert.equal(selectionRow(v)?.row.path, 'stock');
  assert.equal(rowKey(selectionRow(v)!.row), 'stock');
});

test('choosing a hidden plane shows it', (t) => {
  const v = docView(DOC);
  t.after(() => v.doc.dispose());
  v.setRowHidden(row(v, 'std'), true);
  assert.ok(v.hiddenEnts.has(ent(v, 'std.front')));
  v.choosePlane('std.front');
  assert.ok(!v.hiddenEnts.has(ent(v, 'std.front')));
  assert.ok(v.hiddenEnts.has(ent(v, 'std.top')), 'the rest of the group stays hidden');
});

test('a load lets what was hidden go', (t) => {
  const v = docView(DOC);
  t.after(() => v.doc.dispose());
  v.setRowHidden(row(v, 'pin'), true);
  v.setRowHidden(row(v, 'stock'), true);
  assert.ok(v.hiddenRows.size === 2);
  assert.ok(v.load(DOC));
  assert.equal(v.hiddenRows.size, 0);
  assert.equal(v.hiddenEnts.size, 0);
  assert.equal(v.hiddenSolids.size, 0);
  assert.equal(v.pick(...v.seen(ent(v, 'pin.q') as Point)), ent(v, 'pin.q'));
});
