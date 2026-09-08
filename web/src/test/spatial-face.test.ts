import assert from 'node:assert/strict';
import { readFile } from 'node:fs/promises';
import test from 'node:test';
import { Document } from '../core/program.js';
import { faceBoundarySample, spatialFaceLoop } from '../core/spatial-face.js';
import { solve } from '../core/system.js';
import { initCore } from '../core/wasm.js';

await initCore();

test('oriented spatial faces share edge positions and use their own support charts through the ABI', async () => {
  const source = await readFile(new URL('../../../rust/gcs-core/tests/fixtures/spatial_faces.sv', import.meta.url), 'utf8');
  const doc = Document.read(source);
  try {
    assert.ok(doc.ok, JSON.stringify(doc.diagnostics));
    assert.ok(solve(doc.sketch).success);
    const lower = doc.faces().find(f => f.name === 'lower_face')!.index;
    const upper = doc.faces().find(f => f.name === 'upper_face')!.index;
    const a = spatialFaceLoop(doc.sketch, lower), b = spatialFaceLoop(doc.sketch, upper);
    assert.equal(a.length, 4);
    assert.equal(b.length, 4);
    assert.equal(a[1].edge, b[3].edge);
    assert.equal(a[1].reversed, false);
    assert.equal(b[3].reversed, true);
    const heights: Record<string, [number, number, boolean]> = {
      bl: [.98, .1, false], br: [.98, .2, false], ml: [1, .1, false],
      mr: [1, .2, false], tl: [1.02, .1, true], tr: [1.02, .2, true],
    };
    const vertices: [number, number, number][] = doc.vertices().map(v => {
      const h = heights[v.name];
      if (!h) return [NaN, NaN, NaN]; // Unused witnesses need not be supplied.
      return [h[2] ? h[0]-1 : h[0], 0, Math.acos((h[0]*h[0]-1+2*Math.cos(h[1]))/(2*h[0]))];
    });
    const tolerance = { position: 1e-9, normal: 1e-9, axis: 1e-10,
      normalVelocity: 1e-9, incidence: 1e-9, trim: 1e-9 };
    for (const fraction of [0, .25, .5, .75, 1]) {
      const p = faceBoundarySample(doc.sketch, lower, 1, fraction, vertices, tolerance);
      const q = faceBoundarySample(doc.sketch, upper, 3, 1-fraction, vertices, tolerance);
      assert.deepEqual(p.position, q.position);
      assert.equal(p.parameters[0], 1);
      assert.equal(q.parameters[0], 0);
      const z = (1-fraction)*Math.cos(.1)+fraction*Math.cos(.2);
      [3, -Math.sqrt(1-z*z), z].forEach((v, i) => assert.ok(Math.abs(v-p.position[i]) < 1e-8));
      assert.ok(p.incidenceError <= tolerance.incidence && q.incidenceError <= tolerance.incidence);
    }
    assert.throws(() => spatialFaceLoop(doc.sketch, 999), /no such face/);
    assert.throws(() => spatialFaceLoop(doc.sketch, 0), /named support/);
    assert.throws(() => faceBoundarySample(doc.sketch, lower, 0, .5, [], tolerance), /missing.*witness/);
    assert.throws(() => faceBoundarySample(doc.sketch, upper, 3, -Number.MIN_VALUE, vertices, tolerance), /OutsideDomain/);
    assert.throws(() => faceBoundarySample(doc.sketch, upper, 3, 0, vertices, tolerance, 0), /InvalidOptions/);
    assert.throws(() => faceBoundarySample(doc.sketch, upper, 3, .5, [...vertices, [0, 0, 0]], tolerance), /witness count/);
  } finally { doc.dispose(); }
});
