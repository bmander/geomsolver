import assert from 'node:assert/strict';
import { readFile } from 'node:fs/promises';
import test from 'node:test';
import { initCore } from '../core/wasm.js';
import * as drawings from '../core/drawing.js';
import { Document } from '../core/program.js';
import * as modules from '../core/modules.js';
import { solve } from '../core/system.js';
import { exampleCases } from '../app/example-catalog.js';
import * as remote from '../app/remote.js';

await initCore();
const packed = await readFile(new URL('../examples/sources.json', import.meta.url), 'utf8');
const files: Record<string, string> = JSON.parse(packed);
const staticFetch = async (path: string | URL | Request) =>
  new Response(String(path) === 'dist/examples/sources.json' ? packed : '',
    { status: String(path) === 'dist/examples/sources.json' ? 200 : 404 });

test('a bare cylinder preview accepts a standard datum without placement boilerplate', () => {
  const text = 'unit mm\nuse std\nuse components.dims\nuse components.cylinder\n'
    + 'preview { components.cylinder.Cylinder(std.front, fw: 12mm, dims: components.dims.vtwin_dims) }\n';
  const doc = Document.read(text);
  try {
    assert.ok(doc.ok, JSON.stringify(doc.diagnostics));
    assert.ok(solve(doc.sketch).success);
    assert.ok(doc.map.entities.some((e) => e.kind === 'solid'));
    assert.equal(doc.text, text);
    assert.ok(doc.map.entities.every((e) => !e.name?.includes('#')));
  } finally { doc.dispose(); }
});

test('menu examples open files or directories with one V-twin choice', async (t) => {
  t.mock.method(globalThis, 'fetch', staticFetch);
  const cases = exampleCases();
  assert.equal(cases.filter((c) => c.key.startsWith('vtwin')).length, 1);
  for (const example of cases) {
    const bundle = await remote.drawing(example.key);
    const target = example.target;
    assert.equal(bundle.source, target.kind === 'file' ? target.path : `${target.path}/${target.entry}`);
    // a project may open on its model, whose solids the glass box shows, rather than a sheet
    if (bundle.source.endsWith('.svd')) assert.ok(drawings.info(bundle.files[bundle.source]).sheets.length, example.key);
    else assert.ok(bundle.files[bundle.source], example.key);
  }
  const gears = await remote.drawing('spiral_bevel');
  assert.equal(gears.directory, 'spiral_bevel');
  for (const name of ['configuration', 'design', 'layout', 'members', 'generation', 'views',
    'pitch/gear', 'blank/member', 'blank/ends', 'crown/tooth', 'crown/mate', 'crown/mate_section',
    'crown/reach', 'crown/space', 'crown/relief']) {
    assert.ok(gears.files[`spiral_bevel/${name}.sv`], name);
  }
  // the text beside the sources is part of the project, shown and never elaborated
  assert.match(gears.files['spiral_bevel/README.md'], /^# /);
  const readme = await remote.drawing('spiral_bevel', 'spiral_bevel/README.md');
  assert.equal(readme.source, 'spiral_bevel/README.md');
  assert.equal(readme.entry, 'spiral_bevel/gears.sv');
  const vtwin = await remote.drawing('vtwin');
  assert.equal(vtwin.directory, 'vtwin');
  assert.ok(vtwin.files['vtwin/components/frame.sv']);
  for (const name of ['assembly', 'cylinder', 'plate', 'piston', 'disc', 'flywheel', 'throttle']) {
    assert.ok(vtwin.files[`vtwin/${name}.svd`]);
    const model = name === 'assembly' ? name : `components/${name === 'plate' ? 'frame' : name}`;
    assert.ok(vtwin.files[`vtwin/${model}.sv`]);
    if (name !== 'assembly') assert.ok(!(`vtwin/${name}.sv` in vtwin.files));
  }
  assert.ok(!vtwin.files['engine.svd']);
  const drill = await remote.drawing('twist_drill');
  assert.equal(drill.source, 'twist_drill/drill.sv');
  for (const name of ['drill.sv', 'drill.svd', 'configuration.sv', 'wheel.sv', 'point.sv']) {
    assert.ok(drill.files[`twist_drill/${name}`], name);
  }
  assert.ok(drawings.info(drill.files['twist_drill/drill.svd']).sheets.length);
});

test('a component preview uses edited project dependencies when opened directly', async (t) => {
  t.mock.method(globalThis, 'fetch', staticFetch);
  const path = 'vtwin/components/cylinder.sv';
  const bundle = await remote.drawing(path);
  assert.equal(bundle.source, path);
  assert.ok(bundle.files['vtwin/components/dims.sv']);
  const edited: Record<string, string> = { ...bundle.files,
    'vtwin/components/dims.sv': bundle.files['vtwin/components/dims.sv'].replace(
      'fwA := trapz + traph + 3mm', 'fwA := trapz + traph + 4mm'),
  };
  modules.provideProject(path, edited);
  try {
    assert.equal(modules.source('components.dims'), edited['vtwin/components/dims.sv']);
    const doc = Document.read(edited[path]);
    try {
      assert.ok(doc.ok, JSON.stringify(doc.diagnostics));
      assert.ok(doc.text.includes('preview {'));
    } finally { doc.dispose(); }
    const drawing = files['vtwin/cylinder.svd'];
    assert.ok(drawings.render(drawing, 'vtwin/cylinder.svd', edited).includes('<svg'));
  } finally { modules.forget(); }
});

test('every V-twin component preview opens and solves from the project files', () => {
  for (const name of ['cylinder', 'piston', 'disc', 'flywheel', 'throttle', 'frame', 'bank', 'crank', 'side_view']) {
    const path = `vtwin/components/${name}.sv`;
    assert.ok(files[path].includes('\npreview {'), path);
    modules.provideProject(path, files);
    try {
      const doc = Document.read(files[path]);
      try {
        assert.ok(doc.ok, `${path}: ${JSON.stringify(doc.diagnostics)}`);
        assert.ok(solve(doc.sketch).success, path);
        assert.ok(doc.map.entities.some((e) => e.kind === 'line'), path);
      } finally { doc.dispose(); }
    } finally { modules.forget(); }
  }
});

test('live directory listings discover files added after the static build', async (t) => {
  const fresh: Record<string, string> = { ...files,
    'vtwin/assembly.svd': 'model m from "assembly.sv" sheet main { sketch v(m) at (20,30) }',
    'vtwin/components/dims.sv': `${files['vtwin/components/dims.sv']}\n// fresh source\n`,
    'vtwin/extra.sv': 'extra := point\n',
  };
  t.mock.method(globalThis, 'fetch', async (path: string | URL | Request) => {
    const key = String(path);
    const text = key === 'dist/examples/sources.json' ? packed
      : key === 'examples/index.json' ? JSON.stringify(Object.keys(fresh))
      : fresh[key.replace(/^examples\//, '')];
    return new Response(text ?? '', { status: text === undefined ? 404 : 200 });
  });
  const bundle = await remote.drawing('vtwin');
  assert.equal(bundle.files['vtwin/assembly.svd'], fresh['vtwin/assembly.svd']);
  assert.equal(bundle.files['vtwin/components/dims.sv'], fresh['vtwin/components/dims.sv']);
  assert.equal(bundle.files['vtwin/extra.sv'], 'extra := point\n');
});

test('file routes, old part links, and selected directory files resolve to the right source', async (t) => {
  t.mock.method(globalThis, 'fetch', staticFetch);
  const model = await remote.drawing('rect_fillets.sv');
  assert.equal(model.directory, undefined);
  const doc = Document.read(model.files[model.source]);
  try { assert.ok(doc.ok); } finally { doc.dispose(); }
  const old = await remote.drawing('vtwin_piston');
  assert.equal(old.key, 'vtwin');
  assert.equal(old.source, 'vtwin/piston.svd');
  assert.equal((await remote.drawing('vtwin', 'vtwin/cylinder.svd')).source, 'vtwin/cylinder.svd');
  assert.equal((await remote.drawing('vtwin', 'missing.sv')).source, 'vtwin/assembly.svd');
  await assert.rejects(remote.drawing('../outside.sv'), /unknown example/);
});

test('parameterized file examples retain their generated models', async (t) => {
  t.mock.method(globalThis, 'fetch', staticFetch);
  const bundle = await remote.drawing('rect_fillets:80:40:5');
  const svg = drawings.render(bundle.files[bundle.source], bundle.source, bundle.files);
  assert.ok(svg.includes('>80</text>') && svg.includes('>40</text>'));
  await assert.rejects(remote.drawing('does_not_exist'), /unknown example/);
});

test('V-twin models resolve against their own directory without the compiled module library', async (t) => {
  t.mock.method(globalThis, 'fetch', staticFetch);
  const bundle = await remote.drawing('vtwin');
  const text = bundle.files['vtwin/assembly.sv'];
  modules.forget();
  try {
    const loaded = await modules.link(text, async (path) => bundle.files[`vtwin/${path}`] ?? null);
    assert.ok(loaded.includes('components.frame'));
    assert.ok(loaded.includes('components.dims'));
    const doc = Document.read(text);
    try { assert.ok(doc.ok, JSON.stringify(doc.diagnostics)); } finally { doc.dispose(); }
  } finally { modules.forget(); }
});
