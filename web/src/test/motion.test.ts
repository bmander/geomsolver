import assert from 'node:assert/strict';
import test from 'node:test';
import { Document } from '../core/program.js';
import { motionSample } from '../core/motion.js';
import { solve } from '../core/system.js';
import { initCore } from '../core/wasm.js';

await initCore();

test('a named relative motion returns world position and exact velocity through the ABI', () => {
  const doc = Document.read(`unit mm
point a hint(x: 2,y: 0)
point b hint(x: 2,y: 1)
ground a
ground b
line axis(a,b)
motion relative(turn,relative_to: observer)
motion turn(about: axis,ratio: 3,phase: 90deg)
motion observer(about: axis,ratio: 1)
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
