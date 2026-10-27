/* What the workspace shows: View ▸ Show's kinds (#678) — each turned off leaves the box and the
 * canvas, is not picked, is shown again by a tool that needs it, by choosing it to draw on or from
 * the outline, and is view state carried across an edit — and an assembly opened on its parts
 * (#679): what a part draws to make its object out of the way until the part is opened, the rest
 * of the assembly dimmed while it is, Esc or a click on nothing closing it. */
import assert from 'node:assert/strict';
import test from 'node:test';

import { callouts } from '../core/callout.js';
import { KIND_BIT, Line, Plane, Point, fromWire } from '../core/model.js';
import type { Primitive, ShownKind } from '../core/model.js';
import type { Row } from '../core/outline.js';
import { overview3 } from '../core/overview.js';
import {
  isHidden, partToOpen, partlyHidden, selectRow, setRowHidden, shows,
} from '../app/hidden.js';
import { paint } from '../app/paint.js';
import type { SketchView } from '../app/view.js';
import { initCore } from '../core/wasm.js';
import { canvasOf, pointer } from './canvas.js';
import { dblclickAt, docView, row, seenAt } from './views.js';

(globalThis as { requestAnimationFrame?: unknown }).requestAnimationFrame ??=
  (fn: FrameRequestCallback) => { fn(0); return 0; };
(globalThis as { cancelAnimationFrame?: unknown }).cancelAnimationFrame ??= () => {};

await initCore();

/** A line and a circle on the front, a fixed point and a free one, a dimension, an axis standing
 *  up beside them and a disc made of the circle — the core's gate (`tests/workspace.rs`). */
const KINDS = 'unit mm\nuse std\nin std.front {\n'
  + '  a := point hint((0, 0))\n  fix((0, 0)) a\n  b := point hint((40, 0))\n  ab := line(a, b)\n'
  + '  horizontal ab\n  a distance(40) b\n'
  + '  o := point hint((20, 30))\n  fix((20, 30)) o\n  rim := circle(center: o) hint(r: 10)\n}\n'
  + 't := axis hint(dir: (0, 0, 1))\nfix(dir == (0, 0, 1), origin == (80, 0, 0)) t\n'
  + 'disc := solid(face(rim), depth: 5)\n';

/** An assembly: two blocks, each a part drawing its outline and a dimension to make its object,
 *  and the root's own rod on the front. */
const ASSEMBLY = 'unit mm\nuse std\n'
  + 'component Block(x: Length) {\n'
  + '  a := point hint((x, 0))\n  b := point hint((x + 20mm, 0))\n'
  + '  c := point hint((x + 20mm, 10))\n  d := point hint((x, 10))\n'
  + '  fix((x, 0)) a\n  fix((x, 10)) d\n'
  + '  ab := line(a, b)\n  bc := line(b, c)\n  cd := line(c, d)\n  da := line(d, a)\n'
  + '  horizontal ab\n  vertical bc\n  a distance(20) b\n  b distance(10) c\n'
  + '  sec := face(ab, bc, cd, da)\n  body := solid(sec, depth: 5)\n}\n'
  + 'in std.top {\n  left := Block(x: 0mm)\n  right := Block(x: 40mm)\n}\n'
  + 'in std.front {\n  o := point\n  fix((0, 0)) o\n  q := point hint((10, 10))\n'
  + '  rod := line(o, q)\n}\n';

/** The entity the document calls `name`. */
function ent(v: SketchView, name: string): Primitive {
  const e = v.doc.entity(name);
  assert.ok(e, name);
  return e;
}

/** The objects the box holds, and the planes its panes are. */
function box(v: SketchView): { placed: number[]; panes: (number | undefined)[] } {
  v.box3d.sync(v);
  const b = v.box3d as unknown as { placed: Map<number, unknown>; panes: { it: { plane?: number } }[] };
  return { placed: [...b.placed.keys()], panes: b.panes.map((p) => p.it.plane) };
}

/** Every call the canvas took while the view painted, by name, with its arguments. */
function painted(v: SketchView): [string, unknown[]][] {
  const calls: [string, unknown[]][] = [];
  const real = v.ctx;
  const ctx: unknown = new Proxy({}, {
    get: (_t, k) => (k === 'measureText' ? () => ({ width: 8 })
      : (...args: unknown[]) => { calls.push([String(k), args]); }),
    set: () => true,
  });
  v.ctx = ctx as CanvasRenderingContext2D;
  try {
    paint(v);
  } finally {
    v.ctx = real;
  }
  return calls;
}

/** The fixed marks painted: an 8-pixel square each. */
const marks = (calls: [string, unknown[]][]): number =>
  calls.filter(([k, a]) => k === 'fillRect' && a[2] === 8 && a[3] === 8).length;

test('View ▸ Show: each kind turned off leaves the box and the canvas, and is not picked', (t) => {
  const v = docView(KINDS);
  t.after(() => v.doc.dispose());
  v.fit();
  const [a, ab, rim, axis] = ['a', 'ab', 'rim', 't'].map((n) => ent(v, n));
  const front = ent(v, 'std.front') as Plane;
  const disc = v.doc.solids().find((s) => s.name === 'disc')!.index;
  const mid = (l: Line): [number, number] => {
    const [p, q] = [v.seen(l.p1), v.seen(l.p2)];
    return [(p[0] + q[0]) / 2, (p[1] + q[1]) / 2];
  };
  const onAxis = seenAt(v, [80, 0, 20]);
  // all of it shown
  assert.equal(v.pick(...v.seen(a as Point)), a);
  assert.equal(v.pick(...mid(ab as Line)), ab);
  assert.equal(v.pick(...onAxis), axis);
  assert.deepEqual(box(v).placed, [disc]);
  assert.ok(box(v).panes.includes(front.index));
  const fixed = marks(painted(v));
  assert.ok(fixed > 0, 'the fixed points marked');
  assert.ok(callouts(v.sketch, v.unit).items.length > 0);

  const off = (kind: ShownKind) => {
    v.setKind(kind, false);
    assert.ok(!v.kindOn(kind) && !shows(v, kind), kind);
    assert.equal(v.kinds & KIND_BIT[kind], 0);
  };
  const on = (kind: ShownKind) => v.setKind(kind, true);

  off('objects');
  assert.deepEqual(box(v).placed, [], 'the object leaves the box');
  assert.ok(v.hiddenSolids.has(disc));
  assert.equal(v.solidAt(...seenAt(v, [24, -2, 33])), null, 'and is not picked');
  on('objects');
  assert.deepEqual(box(v).placed, [disc]);

  off('planes');
  assert.ok(!box(v).panes.includes(front.index), 'no pane');
  assert.ok(v.hiddenEnts.has(front));
  on('planes');

  off('axes');
  assert.ok(v.hiddenEnts.has(axis));
  assert.notEqual(v.pick(...onAxis), axis);
  assert.ok(!overview3(v.sketch, 0, 'datums').some((it) => it.kind === 'axis'));
  on('axes');

  off('sketches');
  for (const e of [a, ab, rim]) assert.ok(v.hiddenEnts.has(e));
  assert.notEqual(v.pick(...mid(ab as Line)), ab);
  assert.equal(marks(painted(v)), 0, 'nor its points');
  assert.ok(!painted(v).some(([k]) => k === 'arc'), 'no point is drawn');
  on('sketches');

  off('points');
  assert.ok(v.hiddenEnts.has(a) && !v.hiddenEnts.has(ab));
  assert.equal(v.pick(...v.seen(a as Point)), ab, 'the line under the point is what is picked');
  assert.equal(marks(painted(v)), 0);
  on('points');

  off('fixed');
  assert.equal(marks(painted(v)), 0, 'the fixed points drawn as any other');
  assert.equal(v.pick(...v.seen(a as Point)), a, 'and still picked');
  on('fixed');

  off('dimensions');
  assert.equal(v.showDimensions, false);
  assert.equal(callouts(v.sketch, v.unit).items.length, 0);
  assert.equal(v.pickCallout(...v.seen(a as Point)), null);
  on('dimensions');
  assert.ok(callouts(v.sketch, v.unit).items.length > 0);
  assert.equal(v.hiddenEnts.size, 0);
});

test('a tool shows the kinds it needs while it is down', (t) => {
  const v = docView(KINDS);
  t.after(() => v.doc.dispose());
  v.fit();
  const a = ent(v, 'a') as Point;
  v.setKind('points', false);
  v.setKind('objects', false);
  assert.ok(v.hiddenEnts.has(a));
  // a drawing tool snaps to the points
  v.setTool('line');
  assert.ok(!v.hiddenEnts.has(a));
  assert.equal(v.pickPoint(...v.seen(a)), a);
  // a Boolean takes objects
  v.setTool('union');
  assert.equal(v.hiddenSolids.size, 0);
  assert.ok(v.hiddenEnts.has(a));
  v.setTool('select');
  assert.ok(v.hiddenEnts.has(a), 'and they go with it');
  assert.equal([...v.hiddenSolids].length, 1);
  assert.ok(!v.kindOn('points'), 'View ▸ Show is as it was');
});

test('a plane chosen to draw on, or a row chosen from the outline, is shown whatever its kind',
     (t) => {
  const v = docView(KINDS);
  t.after(() => v.doc.dispose());
  v.setKind('planes', false);
  const front = ent(v, 'std.front');
  assert.ok(v.hiddenEnts.has(front) && v.hiddenEnts.has(ent(v, 'std.top')));
  v.choosePlane('std.front');
  assert.ok(!v.hiddenEnts.has(front), 'the plane drawn on is shown');
  assert.ok(v.hiddenEnts.has(ent(v, 'std.top')), 'the others are not');
  // a row of a kind not shown, chosen from the outline: that one thing
  v.setKind('sketches', false);
  const ab = ent(v, 'ab');
  assert.ok(v.hiddenEnts.has(ab));
  selectRow(v, row(v, 'ab'));
  assert.deepEqual(v.selected, [ab]);
  assert.ok(!v.hiddenEnts.has(ab));
  assert.ok(v.hiddenEnts.has(ent(v, 'rim')));
  // a kind turned on or off lets them go
  v.setKind('axes', false);
  assert.ok(v.hiddenEnts.has(ab) && v.hiddenEnts.has(front));
});

test('the kinds shown are view state: carried across an edit and its undo, never written', (t) => {
  const v = docView(KINDS);
  t.after(() => v.doc.dispose());
  v.setKind('axes', false);
  const text = v.source;
  assert.ok(v.apply(v.doc.addPoint(80, 80)));
  assert.ok(v.hiddenEnts.has(ent(v, 't')), 'carried across the edit');
  const edited = v.source;
  assert.notEqual(edited, text);
  v.undo();
  assert.notEqual(v.source, edited, 'the point is gone again');
  assert.ok(v.hiddenEnts.has(ent(v, 't')), 'and the undo');
  assert.ok(!v.source.includes('axes'), 'never written to the source');
  assert.ok(v.load(KINDS));
  assert.ok(!v.kindOn('axes') && v.hiddenEnts.has(ent(v, 't')), 'kept across a load, as the orbit is');
});

test('an assembly opens on its parts: a part\'s insides are out of the way, the root\'s drawn',
     (t) => {
  const v = docView(ASSEMBLY);
  t.after(() => v.doc.dispose());
  assert.ok(v.onParts);
  const inside = ['left.a', 'left.ab', 'left.da', 'right.cd', 'right.d'].map((n) => ent(v, n));
  for (const [i, e] of inside.entries()) assert.ok(v.hiddenEnts.has(e), String(i));
  for (const n of ['o', 'q', 'rod', 'std.front']) assert.ok(!v.hiddenEnts.has(ent(v, n)), n);
  // the objects are drawn, the parts' dimensions are not
  const solids = v.doc.solids().filter((s) => s.name.endsWith('.body')).map((s) => s.index);
  assert.equal(solids.length, 2);
  assert.deepEqual(box(v).placed.sort(), [...solids].sort());
  assert.equal(callouts(v.sketch, v.unit).items.length, 0);
  // the outline says so: a part's members shut, the part itself half
  assert.ok(isHidden(v, row(v, 'left.ab')));
  assert.ok(!isHidden(v, row(v, 'left')) && partlyHidden(v, row(v, 'left')));
  assert.ok(!isHidden(v, row(v, 'left.body')), 'its object is not');
  // and the datums are drawn small: sized to the objects, not the workspace
  const front = (ent(v, 'std.front') as Plane).index;
  const pane = overview3(v.sketch, 0, 'datums').find((it) => it.part === 'face' && it.plane === front)!;
  const xs = pane.pts.map((p) => p[0]);
  assert.ok(Math.max(...xs) - Math.min(...xs) < 60, `the front's pane: ${xs}`);
  // Inside parts draws them all, and the datums the size they always were
  v.setCloseParts(false);
  assert.ok(!v.onParts);
  for (const e of inside) assert.ok(!v.hiddenEnts.has(e));
  assert.ok(callouts(v.sketch, v.unit).items.length > 0);
});

test('a document with no parts opens as it always did', (t) => {
  const v = docView(KINDS);
  t.after(() => v.doc.dispose());
  assert.ok(!v.onParts);
  assert.equal(v.hiddenEnts.size, 0);
});

test('Open shows a part\'s insides and only its own, dims the rest; Esc closes it', (t) => {
  const v = docView(ASSEMBLY);
  t.after(() => v.doc.dispose());
  v.fit();
  const left = row(v, 'left');
  const [la, rab, rod] = ['left.a', 'right.ab', 'rod'].map((n) => ent(v, n));
  const leftBody = row(v, 'left.body').solid!;
  const rightBody = row(v, 'right.body').solid!;
  assert.equal(partToOpen(v, leftBody), left);
  assert.ok(v.openPart(left));
  assert.equal(v.opened, 'left');
  assert.ok(!v.hiddenEnts.has(la), 'its sketch drawn');
  assert.ok(v.hiddenEnts.has(rab), 'and only its own');
  // its dimensions too, and only its own
  const ids = new Set(callouts(v.sketch, v.unit).items.map((k) => k.id));
  assert.ok(ids.size > 0);
  for (const id of ids) {
    const c = v.sketch.constraintById(id)!;
    assert.ok(c.entities().every((e) => v.inside!.ents.has(e)), c.typeName);
  }
  // the rest of the assembly is dimmed: the other part's object and the root's drawing
  assert.ok(v.inside!.solids.has(leftBody) && !v.inside!.solids.has(rightBody));
  assert.ok(v.inside!.ents.has(la) && !v.inside!.ents.has(rod));
  assert.equal(partToOpen(v, leftBody), null, 'open already');
  // carried across an edit by its path
  assert.ok(v.apply(v.doc.addPoint(80, 80)));
  assert.equal(v.opened, 'left');
  assert.ok(!v.hiddenEnts.has(ent(v, 'left.a')));
  // Esc closes it
  v.cancelTool();
  assert.equal(v.opened, null);
  assert.ok(v.hiddenEnts.has(ent(v, 'left.a')));
  assert.equal(v.inside, null);
});

test('a part opens by a double-click on its object, and closes by a click on nothing', (t) => {
  const v = docView(ASSEMBLY);
  t.after(() => v.doc.dispose());
  v.orbit = { az: 0.6, el: 0.5 };
  v.fit();
  // the right block's middle, from three quarters
  const at = seenAt(v, [50, 5, -2.5]);
  const hit = v.solidAt(...at);
  assert.ok(hit, 'the right block is under the pointer');
  assert.equal(hit.index, row(v, 'right.body').solid);
  dblclickAt(v, at);
  assert.equal(v.opened, 'right');
  assert.ok(!v.hiddenEnts.has(ent(v, 'right.ab')));
  // a double-click on the part open draws on its face, as it always did
  assert.equal(partToOpen(v, hit.index), null);
  // a click on nothing closes it
  const cv = canvasOf(v);
  cv.fire('pointerdown', pointer(3, 3));
  cv.fire('pointerup', pointer(3, 3));
  assert.equal(v.opened, null);
  assert.ok(v.hiddenEnts.has(ent(v, 'right.ab')));
});

test('selecting a member row opens its part; the eye shows a row whatever closes it', (t) => {
  const v = docView(ASSEMBLY);
  t.after(() => v.doc.dispose());
  const lab = ent(v, 'left.ab');
  selectRow(v, row(v, 'left.ab'));
  assert.equal(v.opened, 'left');
  assert.deepEqual(v.selected, [lab]);
  v.closePart();
  // the outline's eye overrides the rule: a member of a closed part shown
  assert.ok(isHidden(v, row(v, 'right.ab')));
  setRowHidden(v, row(v, 'right.ab'), false);
  assert.ok(!isHidden(v, row(v, 'right.ab')));
  assert.ok(!v.hiddenEnts.has(ent(v, 'right.ab')));
  assert.ok(v.hiddenEnts.has(ent(v, 'right.bc')), 'its siblings stay closed');
  // and either way: a root row hidden by its eye
  setRowHidden(v, row(v, 'rod'), true);
  assert.ok(v.hiddenEnts.has(ent(v, 'rod')));
  // a load lets the part opened and the rows shown go
  v.openPart(row(v, 'left'));
  assert.ok(v.load(ASSEMBLY));
  assert.equal(v.opened, null);
  assert.equal(v.shownRows.size, 0);
  assert.ok(v.hiddenEnts.has(ent(v, 'right.ab')));
});

test('the parts\' rows are what the core lists them as', (t) => {
  const v = docView(ASSEMBLY);
  t.after(() => v.doc.dispose());
  const kinds = (r: Row) => r.children.map((c) => c.kind);
  assert.ok(kinds(row(v, 'left')).includes('solid'), JSON.stringify(kinds(row(v, 'left'))));
  const [kind, index] = row(v, 'left.a').ents[0];
  assert.ok(v.doc.entityOf({ kind: fromWire(kind), index }) instanceof Point);
});
