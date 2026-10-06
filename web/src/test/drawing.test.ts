import assert from 'node:assert/strict';
import test from 'node:test';
import { initCore } from '../core/wasm.js';
import * as drawing from '../core/drawing.js';

await initCore();

const model = 'unit mm\nuse std\nin std.front {\na := point\nfix((0, 0)) a\n'
  + 'b := point\nfix((20, 0)) b\nbar := line(a,b)\n}\n';
const source = 'model m from "../models/part.sv" use "../styles/ink.svd"\n'
  + 'sheet front { size A4 sketch v(m) at (40mm,50mm) '
  + 'measure distance(m.bar.p1,m.b) in v offset 6mm }\n'
  + 'sheet detail { size (100mm,80mm) scale 2 sketch v(m) at (20mm,40mm) }';
const files = {
  'models/part.sv': model,
  'styles/ink.svd': 'use "base.svd" style m.bar { color: #123456 }',
  'styles/base.svd': 'style m.bar { width: 2 }',
};

test('drawing WASM binding resolves relative files and renders selected sheets', () => {
  assert.deepEqual(drawing.info(source), {
    models: ['../models/part.sv'], imports: ['../styles/ink.svd'], sheets: ['front', 'detail'],
  });
  const before = JSON.stringify(files);
  const front = drawing.render(source, 'drawings/part.svd', files, 'front');
  assert.ok(front.includes('width="210mm"') && front.includes('#123456'));
  assert.ok(front.includes('>20</text>'));
  const detail = drawing.render(source, 'drawings/part.svd', files, 'detail');
  assert.ok(detail.includes('width="100mm"'));
  assert.notEqual(front, detail);
  assert.equal(JSON.stringify(files), before);
  assert.throws(() => drawing.render(source, 'drawings/part.svd', files), /select a sheet/);
  assert.throws(() => drawing.render(source, 'drawings/part.svd', {}, 'front'), /cannot load/);
  assert.throws(() => drawing.render(source.replace('m.b)', 'm.missing)'),
    'drawings/part.svd', files, 'front'), /not a point/);
});

test('a drawing renders only the dimensions it explicitly requests', () => {
  const part = `${model}param width := 20mm\na distance(width) b\n`
    + 'in std.front {\nc := circle(center: a) hint(r: 5)\n}\nparam r := 5mm\nradius(r) c\n';
  const render = (request: string) => drawing.render('model m from "part.sv" '
    + `sheet s { sketch v(m) at (30mm,40mm) ${request} }`,
    'part.svd', {'part.sv': part});
  assert.ok(!render('').includes('<text'));
  const one = render('dimension m.width in v');
  assert.equal(one.match(/<text /g)?.length, 1);
  assert.ok(one.includes('>width</text>'), one);
  assert.equal(render('dimensions in v').match(/<text /g)?.length, 2);
});
