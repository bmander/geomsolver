import assert from 'node:assert/strict';
import test from 'node:test';
import { Document } from '../core/program.js';
import { patchSurfaceSample, patchEnvelopeSample } from '../core/patch.js';
import { solve } from '../core/system.js';
import { initCore } from '../core/wasm.js';

await initCore();

test('patch sampling requires both the declared material side and source incidence', () => {
  const doc = Document.read(`unit mm
point o hint(x: 0,y: 0)
point q hint(x: 0,y: 1)
point x hint(x: 1,y: 0)
point c hint(x: 3,y: 0)
point b hint(x: 0,y: -3.5)
point t hint(x: 0,y: 3.5)
ground o
ground q
ground x
ground c
ground b
ground t
line axis(o,q)
line spin_axis(o,x)
circle meridian(center: c)
radius(1mm) meridian
solid ring(face(meridian),about: axis)
surface wall(ring,meridian)
arc rim(center: o,start: b,end: t)
radius(3.5mm) rim
line diameter(t,b)
solid limit(face(rim,diameter),about: diameter)
motion roll(about: spin_axis)
envelope generated(wall,under: roll,from: -20deg,to: 20deg)
patch bounded(wall,inside: limit)
patch tooth(generated,inside: limit)
`);
  try {
    assert.ok(doc.ok, JSON.stringify(doc.diagnostics));
    assert.ok(solve(doc.sketch).success);
    assert.deepEqual(doc.patches(), [{ name: 'bounded', index: 0 }, { name: 'tooth', index: 1 }]);
    const tolerance = { axis: 1e-9, trim: 1e-9, normalVelocity: 1e-9 };
    const p = patchSurfaceSample(doc.sketch, 0, .25, 0, tolerance);
    assert.ok(Math.abs(p.position[0]-3) < 1e-9);
    assert.ok(Math.abs(p.position[2]-1) < 1e-9);
    assert.throws(() => patchSurfaceSample(doc.sketch, 0, 0, 0, tolerance), /OutsideDomain/);
    const c = patchEnvelopeSample(doc.sketch, 1, .25, 0, .1, tolerance);
    assert.ok(Math.abs(c.normalVelocity) < 1e-9);
    assert.throws(() => patchEnvelopeSample(doc.sketch, 1, .25, .1, .1, tolerance), /OutsideDomain/);
    assert.throws(() => patchSurfaceSample(doc.sketch, 99, 0, 0, tolerance), /no such patch/);
  } finally { doc.dispose(); }
});
