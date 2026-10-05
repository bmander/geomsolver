import assert from 'node:assert/strict';
import test from 'node:test';
import { Document } from '../core/program.js';
import { surfaceSample, surfaceProjection, generatingProfileBounds } from '../core/surface.js';
import { solve } from '../core/system.js';
import { initCore } from '../core/wasm.js';

await initCore();

test('a named analytic surface evaluates the solved model through the ABI', () => {
  const doc = Document.read(`unit mm
use std
in std.front {
o := point
q := point
c := point
fix(x == 0, y == 0) o
fix(x == 0, y == 1) q
fix(x == 3, y == 0) c
ax := line(o,q)
meridian := circle(center: c)
radius(1mm) meridian
}
ring := solid(face(meridian),about: ax)
wall := surface(ring,meridian)
`);
  try {
    assert.ok(doc.ok, JSON.stringify(doc.diagnostics));
    assert.ok(solve(doc.sketch).success);
    assert.deepEqual(doc.surfaces(), [{ name: 'wall', index: 0 }]);
    const sample = surfaceSample(doc.sketch, 0, 0, .25);
    assert.ok(Math.abs(sample.position[0]) < 1e-10);
    assert.ok(Math.abs(sample.position[1] - 4) < 1e-10);
    assert.ok(Math.abs(sample.du[2] - 2 * Math.PI) < 1e-10);
    assert.ok(Math.abs(sample.dv[0] + 8 * Math.PI) < 1e-10);
    assert.throws(() => surfaceSample(doc.sketch, 0, -1, 0), /OutsideDomain/);
    assert.throws(() => surfaceSample(doc.sketch, 10, 0, 0), /no such surface/);
    const on = surfaceProjection(doc.sketch, 0, sample.position);
    assert.ok(on.incidenceError < 1e-10);
    assert.ok(Math.abs(on.signedResidual) < 1e-10);
    const outside = surfaceProjection(doc.sketch, 0, [0, 4.1, 0]);
    assert.ok(Math.abs(outside.incidenceError - .1) < 1e-10);
    assert.ok(Math.abs(outside.signedResidual + .1) < 1e-10);
    assert.throws(() => surfaceProjection(doc.sketch, 0, [NaN, 0, 0]), /NonFinite/);
    const bounds = generatingProfileBounds(doc.sketch, 0, [.1, .2]);
    for (const u of [.1, .15, .2]) {
      const p = surfaceSample(doc.sketch, 0, u, 0);
      p.position.forEach((x, i) => assert.ok(bounds.position[i][0] <= x && x <= bounds.position[i][1]));
      p.du.forEach((x, i) => assert.ok(bounds.du[i][0] <= x && x <= bounds.du[i][1]));
    }
    assert.throws(() => generatingProfileBounds(doc.sketch, 0, [.2, .1]), /InvalidBounds/);
    assert.throws(() => generatingProfileBounds(doc.sketch, 0, [-.1, .1]), /OutsideDomain/);
    assert.throws(() => generatingProfileBounds(doc.sketch, 99, [0, 1]), /no such surface/);
  } finally { doc.dispose(); }
});
