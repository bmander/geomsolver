import assert from 'node:assert/strict';
import test from 'node:test';
import { Document } from '../core/program.js';
import { patchSurfaceSample, patchEnvelopeSample } from '../core/patch.js';
import { solve } from '../core/system.js';
import { initCore } from '../core/wasm.js';

await initCore();

test('patch sampling requires both the declared material side and source incidence', () => {
  const doc = Document.read(`unit mm
use std
in std.front {
o := point
q := point
x := point
c := point
b := point
t := point
fix(x == 0, y == 0) o
fix(x == 0, y == 1) q
fix(x == 1, y == 0) x
fix(x == 3, y == 0) c
fix(x == 0, y == -3.5) b
fix(x == 0, y == 3.5) t
axis := line(o,q)
spin_axis := line(o,x)
meridian := circle(center: c)
radius(1mm) meridian
rim := arc(center: o,start: b,end: t)
radius(3.5mm) rim
diameter := line(t,b)
}
ring := solid(face(meridian),about: axis)
wall := surface(ring,meridian)
limit := solid(face(rim,diameter),about: diameter)
roll := motion(about: spin_axis)
generated := envelope(wall,under: roll,from: -20deg,to: 20deg)
bounded := patch(wall,inside: limit)
tooth := patch(generated,inside: limit)
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
