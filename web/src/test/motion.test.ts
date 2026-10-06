import assert from 'node:assert/strict';
import test from 'node:test';
import { Document } from '../core/program.js';
import { motionSample } from '../core/motion.js';
import { solve } from '../core/system.js';
import { initCore } from '../core/wasm.js';

await initCore();

test('a named relative motion returns world position and exact velocity through the ABI', () => {
  const doc = Document.read(`unit mm
use std
in std.front {
a := point
b := point
fix((2, 0)) a
fix((2, 1)) b
ax := line(a,b)
}
relative := motion(turn,relative_to: observer)
turn := motion(about: ax,ratio: 3,phase: 90deg)
observer := motion(about: ax,ratio: 1)
`);
  try {
    assert.ok(doc.ok, JSON.stringify(doc.diagnostics));
    assert.ok(solve(doc.sketch).success);
    const motions = doc.motions();
    assert.equal(motions.length, 3);
    const index = motions.find(m => m.name === 'relative')!.index;
    const sample = motionSample(doc.sketch, index, 0, [3, 0, 0]);
    const near = (a: number[], b: number[]) => a.forEach((v, i) => assert.ok(Math.abs(v-b[i]) < 1e-10));
    near(sample.position, [2, 1, 0]);
    near(sample.velocity, [-2, 0, 0]);
    assert.throws(() => motionSample(doc.sketch, 999, 0, [0, 0, 0]), /no such motion/);
    assert.throws(() => motionSample(doc.sketch, index, NaN, [0, 0, 0]), /finite/);
    assert.throws(() => motionSample(doc.sketch, index, 0, [Infinity, 0, 0]), /finite/);
  } finally { doc.dispose(); }
});

test('a ratio measured off the drawing is read through the ABI once the drawing is solved', () => {
  const doc = Document.read(`unit mm
use std
in std.front {
a := point
b := point
fix((0, 0)) a
fix((0, 1)) b
ax := line(a,b)
c := point
d := point hint((25, -2))
fix((0, -2)) c
big := horizontal line(c,d)
c distance(30mm) d
e := point
f := point hint((12, -4))
fix((0, -4)) e
small := horizontal line(e,f)
e distance(10mm) f
}
turn := motion(about: ax, ratio: length(big) / length(small))
`);
  try {
    assert.ok(doc.ok, JSON.stringify(doc.diagnostics));
    assert.ok(solve(doc.sketch).success);
    const index = doc.motions().find(m => m.name === 'turn')!.index;
    // a point a unit off the axis moves at the ratio: 30 / 10 as solved, not 25 / 12 as seeded
    const v = motionSample(doc.sketch, index, 0, [1, 0, 0]).velocity;
    assert.ok(Math.abs(Math.hypot(...v) - 3) < 1e-9, JSON.stringify(v));
  } finally { doc.dispose(); }
});

test('a measurement where a number is needed before the solve is refused as E107', () => {
  const doc = Document.read(`unit mm
c := point hint((0, -2))
d := point hint((25, -2))
big := line(c,d)
k := length(big)
`);
  try {
    assert.ok(doc.diagnostics.some(d => d.code === 'E107'), JSON.stringify(doc.diagnostics));
  } finally { doc.dispose(); }
});
