import assert from 'node:assert/strict';
import { readFile } from 'node:fs/promises';
import test from 'node:test';
import { initCore } from '../core/wasm.js';
import * as examples from '../core/examples.js';
import * as drawings from '../core/drawing.js';
import * as remote from '../app/remote.js';

await initCore();
const packed = await readFile(new URL('../examples/sources.json', import.meta.url), 'utf8');
const files: Record<string, string> = JSON.parse(packed);

test('every menu example has a main drawing and resolves through the static source bundle', async (t) => {
  t.mock.method(globalThis, 'fetch', async (path: string | URL | Request) =>
    new Response(String(path) === 'dist/examples/sources.json' ? packed : '',
      { status: String(path) === 'dist/examples/sources.json' ? 200 : 404 }));
  for (const example of examples.cases()) {
    const bundle = await remote.drawing(example.key);
    assert.equal(bundle.source, `${example.key}.svd`);
    const doc = drawings.info(bundle.files[bundle.source]);
    assert.ok(doc.sheets.length, example.key);
    for (const model of doc.models) assert.ok(bundle.files[model], `${example.key}: ${model}`);
  }
  const vtwin = await remote.drawing('vtwin');
  assert.ok(vtwin.files['vtwin/frame.sv']);
  assert.ok(vtwin.files['vtwin/throttle.sv']);
  assert.ok(!vtwin.files['engine.svd'], 'the bundle contains only this drawing and its dependencies');
});

test('the demo server can override the main drawing and transitive model dependencies', async (t) => {
  const fresh: Record<string, string> = { ...files,
    'vtwin.svd': 'model m from "vtwin.sv" sheet main { sketch v(m) at (20,30) }',
    'vtwin/dims.sv': `${files['vtwin/dims.sv']}\n// fresh source\n`,
  };
  t.mock.method(globalThis, 'fetch', async (path: string | URL | Request) => {
    const key = String(path);
    const text = key === 'dist/examples/sources.json' ? packed : fresh[key.replace(/^examples\//, '')];
    return new Response(text ?? '', { status: text === undefined ? 404 : 200 });
  });
  const bundle = await remote.drawing('vtwin');
  assert.equal(bundle.files['vtwin.svd'], fresh['vtwin.svd']);
  assert.equal(bundle.files['vtwin/dims.sv'], fresh['vtwin/dims.sv']);
});

test('parameterized example routes retain their generated model and authored drawing', async (t) => {
  t.mock.method(globalThis, 'fetch', async (path: string | URL | Request) =>
    new Response(String(path) === 'dist/examples/sources.json' ? packed : '',
      { status: String(path) === 'dist/examples/sources.json' ? 200 : 404 }));
  const bundle = await remote.drawing('rect_fillets:80:40:5');
  assert.equal(bundle.source, 'rect_fillets.svd');
  const svg = drawings.render(bundle.files[bundle.source], bundle.source, bundle.files);
  assert.ok(svg.includes('>80</text>') && svg.includes('>40</text>'));
  await assert.rejects(remote.drawing('does_not_exist'), /cannot load example drawing/);
});
