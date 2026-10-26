import assert from 'node:assert/strict';
import test from 'node:test';
import { Document } from '../core/program.js';
import { frameSample } from '../core/frame.js';
import { solve } from '../core/system.js';
import { initCore } from '../core/wasm.js';

await initCore();

test('a frame turning returns world position and exact velocity through the ABI', () => {
  const doc = Document.read(`unit mm
use std
use std.motion (turns)
in std.front {
a := point
b := point
fix((2, 0)) a
fix((2, 1)) b
ax := line(a,b)
}
turn := frame
turn turns(3) ax
still := frame
`);
  try {
    assert.ok(doc.ok, JSON.stringify(doc.diagnostics));
    assert.ok(solve(doc.sketch).success);
    const moving = doc.movingFrames();
    assert.deepEqual(moving.map(m => m.name), ['turn'], 'a frame at rest is no motion');
    const index = moving[0].index;
    // a third of a quarter turn of the roll turns it a quarter: (3, 0, 0) about the line through
    // (2, 0, 0) along z goes to (2, 1, 0), moving at 3 a radian
    const sample = frameSample(doc.sketch, index, Math.PI / 6, [3, 0, 0]);
    const near = (a: number[], b: number[]) => a.forEach((v, i) => assert.ok(Math.abs(v-b[i]) < 1e-10, `${a} ${b}`));
    near(sample.position, [2, 1, 0]);
    near(sample.velocity, [-3, 0, 0]);
    assert.throws(() => frameSample(doc.sketch, 999, 0, [0, 0, 0]), /no such frame/);
    assert.throws(() => frameSample(doc.sketch, index, NaN, [0, 0, 0]), /finite/);
    assert.throws(() => frameSample(doc.sketch, index, 0, [Infinity, 0, 0]), /finite/);
    const still = doc.entity('still')!;
    assert.throws(() => frameSample(doc.sketch, still.index, 0, [0, 0, 0]), /does not move/);
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
