/* The drawing and the text as one thing (`app/names.ts`): what is selected in the workspace lit
 * wherever the text names it, and a name in the text selecting what it names. */
import assert from 'node:assert/strict';
import test from 'node:test';

import { Document, symbols } from '../core/program.js';
import type { Row } from '../core/outline.js';
import { Names, selectRows, selectionMarks } from '../app/names.js';
import { SketchView } from '../app/view.js';
import { initCore } from '../core/wasm.js';
import { fakeCanvas } from './canvas.js';

(globalThis as { requestAnimationFrame?: unknown }).requestAnimationFrame ??=
  (fn: FrameRequestCallback) => { fn(0); return 0; };
(globalThis as { cancelAnimationFrame?: unknown }).cancelAnimationFrame ??= () => {};

await initCore();

const RUNG = 'component Rung(a: point, b: point, len: Length) {\n'
  + '  mid := point\n  a distance(len) mid\n}\n'
  + 'p := point\nq := point\nt0 := Rung(p, q, len: 50)\nt1 := Rung(q, p, len: 40)\n'
  + 't0.mid distance(3) q\n';

const BODY = 'use std\nin std.front {\no := point\nrim := circle(center: o) hint(r: 10)\n}\n'
  + 'body := solid(face(rim), depth: 8)\n';

function viewOf(text: string): SketchView {
  const v = new SketchView(fakeCanvas(), Document.read(text));
  v.autoSolve = false;
  return v;
}

const namesOf = (v: SketchView) => new Names(v.doc.text, symbols(v.doc.text), v.outline());

/** `text` with the marks written into it: `«given»`, `‹read›`. */
function marked(text: string, marks: { lo: number; hi: number; cls: string }[]): string {
  let out = '', end = 0;
  for (const m of marks) {
    const [open, close] = m.cls === 'occurrence-def' ? ['«', '»'] : ['‹', '›'];
    out += text.slice(end, m.lo) + open + text.slice(m.lo, m.hi) + close;
    end = m.hi;
  }
  return out + text.slice(end);
}

/** The rows the name starting at string index `at` stands for, by path. */
function rowsAt(n: Names, at: number): string[] {
  const o = n.index.at(at);
  assert.ok(o, `a name at ${at}`);
  return n.rowsAt(o).map((r: Row) => r.path);
}

test('a name stands for the rows it is written as a part of', () => {
  const v = viewOf(RUNG);
  const n = namesOf(v);
  assert.deepEqual(rowsAt(n, RUNG.indexOf('p := ')), ['p']);
  assert.deepEqual(rowsAt(n, RUNG.indexOf('p, len: 40')), ['p'], 'read as given');
  // the `mid` the component declares, and reads in its body, is every instance's
  assert.deepEqual(rowsAt(n, RUNG.indexOf('mid :=')), ['t0.mid', 't1.mid']);
  assert.deepEqual(rowsAt(n, RUNG.indexOf('mid\n}')), ['t0.mid', 't1.mid']);
  // read through an instance it is that instance's
  assert.deepEqual(rowsAt(n, RUNG.indexOf('t0.mid') + 3), ['t0.mid']);
  assert.deepEqual(rowsAt(n, RUNG.indexOf('t0 :=')), ['t0'], 'an instance is its row');
  assert.deepEqual(rowsAt(n, RUNG.indexOf('len)')), [], 'a formal number makes nothing');
  v.doc.dispose();
});

test('a module datum is the member its whole path reads', () => {
  const v = viewOf(BODY);
  const n = namesOf(v);
  assert.deepEqual(rowsAt(n, BODY.indexOf('front')), ['std.front']);
  assert.deepEqual(rowsAt(n, BODY.indexOf('body')), ['body']);
  v.doc.dispose();
});

test('what is selected is lit wherever the text names it', () => {
  const v = viewOf(RUNG);
  const n = namesOf(v);
  v.selected = [v.doc.entity('t0.mid')!];
  assert.equal(marked(RUNG, selectionMarks(v, n)),
    'component Rung(a: point, b: point, len: Length) {\n'
    + '  «mid» := point\n  a distance(len) ‹mid›\n}\n'
    + 'p := point\nq := point\nt0 := Rung(p, q, len: 50)\nt1 := Rung(q, p, len: 40)\n'
    + 't0.‹mid› distance(3) q\n');
  v.selected = [v.doc.entity('t1.mid')!, v.doc.entity('q')!];
  assert.equal(marked(RUNG, selectionMarks(v, n)),
    'component Rung(a: point, b: point, len: Length) {\n'
    + '  «mid» := point\n  a distance(len) ‹mid›\n}\n'
    + 'p := point\n«q» := point\nt0 := Rung(p, ‹q›, len: 50)\nt1 := Rung(‹q›, p, len: 40)\n'
    + 't0.mid distance(3) ‹q›\n', 't0.mid is not t1\'s');
  v.selected = [];
  assert.deepEqual(selectionMarks(v, n), []);
  v.doc.dispose();
});

test('a body selected is lit where it is named', () => {
  const v = viewOf(BODY);
  const n = namesOf(v);
  v.pickSolid(v.solidPick(v.doc.solids()[0].index)!);
  assert.equal(marked(BODY, selectionMarks(v, n)),
    BODY.replace('body :=', '«body» :='));
  v.doc.dispose();
});

test('a name in the text selects what it names, and only once', () => {
  const v = viewOf(RUNG);
  const n = namesOf(v);
  const at = (i: number) => n.rowsAt(n.index.at(i)!);
  assert.equal(selectRows(v, at(RUNG.indexOf('t0.mid') + 3)), true);
  assert.deepEqual(v.selected, [v.doc.entity('t0.mid')]);
  assert.equal(selectRows(v, at(RUNG.indexOf('t0.mid') + 3)), false, 'already selected');
  // the declaration in the component selects what each instance made of it
  assert.equal(selectRows(v, at(RUNG.indexOf('mid :='))), true);
  assert.deepEqual(new Set(v.selected), new Set([v.doc.entity('t0.mid'), v.doc.entity('t1.mid')]));
  // an instance is every member it made
  selectRows(v, at(RUNG.indexOf('t1 :=')));
  assert.deepEqual(v.selected, [v.doc.entity('t1.mid')]);
  // a name that makes nothing leaves the selection where it was
  assert.equal(selectRows(v, at(RUNG.indexOf('len)'))), false);
  assert.deepEqual(v.selected, [v.doc.entity('t1.mid')]);
  v.doc.dispose();
});

test('a body named in the text is selected whole', () => {
  const v = viewOf(BODY);
  const n = namesOf(v);
  v.selected = [v.doc.entity('o')!];
  assert.equal(selectRows(v, n.rowsAt(n.index.at(BODY.lastIndexOf('body'))!)), true);
  assert.deepEqual(v.selected, []);
  assert.deepEqual(v.selectedSolids.map((s) => [s.name, s.face]), [['body', '']]);
  // and a point named after it takes the selection back to the drawing's
  selectRows(v, n.rowsAt(n.index.at(BODY.indexOf('o :='))!));
  assert.deepEqual(v.selected, [v.doc.entity('o')]);
  assert.equal(v.selectedSolids.length, 0);
  v.doc.dispose();
});
