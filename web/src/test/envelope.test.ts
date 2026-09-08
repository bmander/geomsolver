import assert from 'node:assert/strict';
import test from 'node:test';
import { Document } from '../core/program.js';
import { envelopeSample, envelopeDomain } from '../core/envelope.js';
import { surfaceDomain } from '../core/surface.js';
import { solve } from '../core/system.js';
import { initCore } from '../core/wasm.js';

await initCore();

test('a named envelope exposes its defining residual and enforces the declared roll domain', () => {
  const doc = Document.read(`unit mm
point o hint(x: 0,y: 0)
point q hint(x: 0,y: 1)
point x hint(x: 1,y: 0)
point c hint(x: 3,y: 0)
ground o
ground q
ground x
ground c
line axis(o,q)
line spin_axis(o,x)
circle meridian(center: c)
radius(1mm) meridian
solid ring(face(meridian),about: axis)
surface wall(ring,meridian)
motion roll(about: spin_axis)
envelope generated(wall,under: roll,from: -20deg,to: 20deg)
surface half(ring,meridian,from: 180deg,to: 360deg)
envelope limited(half,under: roll,from: -20deg,to: 20deg)
`);
  try {
    assert.ok(doc.ok, JSON.stringify(doc.diagnostics));
    assert.ok(solve(doc.sketch).success);
    assert.deepEqual(doc.envelopes(), [{ name: 'generated', index: 0 }, { name: 'limited', index: 1 }]);
    assert.deepEqual(surfaceDomain(doc.sketch, 1), [[0, 1], [.5, 1]]);
    const domain = envelopeDomain(doc.sketch, 1);
    assert.deepEqual(domain.slice(0, 2), [[0, 1], [.5, 1]]);
    assert.ok(Math.abs(domain[2][0]+Math.PI/9) < 1e-12);
    assert.ok(Math.abs(domain[2][1]-Math.PI/9) < 1e-12);
    assert.throws(() => envelopeSample(doc.sketch, 1, .125, 0, .1), /OutsideDomain/);
    assert.ok(Math.abs(envelopeSample(doc.sketch, 1, .125, .5, .1).normalVelocity) < 1e-10);
    const on = envelopeSample(doc.sketch, 0, .125, 0, .1);
    assert.ok(Math.abs(on.normalVelocity) < 1e-10);
    const h = Math.SQRT1_2;
    [3+h, -h*Math.sin(.1), h*Math.cos(.1)].forEach((v, i) =>
      assert.ok(Math.abs(v-on.position[i]) < 1e-10));
    const off = envelopeSample(doc.sketch, 0, .125, .1, .1);
    assert.ok(Math.abs(off.normalVelocity) > 1);
    assert.throws(() => envelopeSample(doc.sketch, 0, .1, .1, 1), /OutsideDomain/);
    assert.throws(() => envelopeSample(doc.sketch, 99, .1, .1, 0), /no such envelope/);
  } finally { doc.dispose(); }
});
