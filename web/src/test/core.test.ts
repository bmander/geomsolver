/* The TypeScript binding against the Rust core — the same properties the Python suite asserts:
 * the compiled plan, both solvers, structural diagnosis, the decomposition plan, witness
 * analysis, homotopy enumeration, dragging and JSON I/O.
 *
 * There is one implementation of all of it now; these tests check that the binding reaches it
 * faithfully, and that the ABI the two sides share stays in step. */
import assert from 'node:assert/strict';
import test from 'node:test';

import * as C from '../core/constraints.js';
import * as examples from '../core/examples.js';
import * as graph from '../core/graph.js';
import * as io from '../core/io.js';
import { PlanDrag, PlanSolver, buildGraph } from '../core/decompose.js';
import {
  diagnose, distanceRigidity, minimalConflictSet, violatedConstraints,
} from '../core/diagnose.js';
import { checkSketch } from '../core/fdcheck.js';
import { enumerateStep } from '../core/homotopy.js';
import { Sketch } from '../core/model.js';
import { Drag, RadiusDrag, System, solve } from '../core/system.js';
import { analyze } from '../core/witness.js';
import { core, initCore } from '../core/wasm.js';

await initCore();

const allSatisfied = (sk: Sketch): boolean => {
  const s = new System(sk);
  try {
    return violatedConstraints(s).length === 0;
  } finally {
    s.dispose();
  }
};

const num = (v: unknown): number => Number(v);

/* -- graph algorithms ------------------------------------------------------- */

test('hopcroft-karp perfect and deficient matchings', () => {
  assert.equal(graph.hopcroftKarp([[0, 1], [1, 2], [2, 0]], 3).mateL.filter((m) => m >= 0).length, 3);
  assert.equal(graph.hopcroftKarp([[0], [0], [0]], 1).mateL.filter((m) => m >= 0).length, 1);
});

test('Dulmage-Mendelsohn blocks', () => {
  const dm = graph.dulmageMendelsohn([[0], [0], [1, 2]], 3);
  assert.deepEqual(dm.overRows.sort(), [0, 1]);
  assert.deepEqual(dm.overCols, [0]);
  assert.deepEqual(dm.underRows, [2]);
  assert.deepEqual(dm.underCols.sort(), [1, 2]);
  assert.equal(dm.nRedundant, 1);
  assert.equal(dm.nFree, 1);
  assert.equal(dm.rank, 2);
});

test('(2,3) pebble game basics', () => {
  assert.equal(graph.pebbleGame(3, [[0, 1], [1, 2], [2, 0]]).dof, 0);
  assert.equal(graph.pebbleGame(4, [[0, 1], [1, 2], [2, 3], [3, 0]]).dof, 1);
  const k4 = graph.pebbleGame(4, [[0, 1], [1, 2], [2, 3], [3, 0], [0, 2], [1, 3]]);
  assert.equal(k4.dof, 0);
  assert.deepEqual(k4.redundant, [5]);
  const bow = graph.pebbleGame(5, [[0, 1], [1, 2], [2, 0], [2, 3], [3, 4], [4, 2]]);
  assert.equal(bow.dof, 1);
  assert.deepEqual(bow.components.map((c) => [...c].sort((a, b) => a - b)).sort((a, b) => a[0] - b[0]),
                   [[0, 1, 2], [2, 3, 4]]);
});

test('pebble game recognises random Laman graphs', () => {
  for (let seed = 0; seed < 6; seed++) {
    const n = 4 + seed;
    const edges = examples.hennebergEdges(n, seed + 1);
    assert.equal(edges.length, 2 * n - 3);
    const res = graph.pebbleGame(n, edges);
    assert.equal(res.dof, 0, `seed ${seed}`);
    assert.equal(res.redundant.length, 0);
    assert.deepEqual(res.components.map((c) => c.length), [n]);
    assert.equal(graph.pebbleGame(n, edges.slice(1)).dof, 1);
  }
});

/* -- solving ---------------------------------------------------------------- */

for (const name of Object.keys(examples.EXAMPLES)) {
  for (const method of ['dogleg', 'lm'] as const) {
    test(`solve ${name} with ${method}`, () => {
      const sk = examples.EXAMPLES[name]();
      sk.perturb(1.0, 3);
      const res = solve(sk, { method });
      assert.ok(res.success, `${name}/${method}: max|r| = ${res.maxResidual}`);
      assert.ok(allSatisfied(sk));
    });
  }
}

test('a large truss solves on either Jacobian path', () => {
  for (const dense of [false, true]) {
    const sk = examples.truss(60);
    const sys = new System(sk);
    try {
      sk.perturb(0.4, 1);
      const res = sys.solve({ dense });
      assert.ok(res.success, `dense=${dense}: max|r| = ${res.maxResidual}`);
    } finally {
      sys.dispose();
    }
  }
});

test('a disposed System refuses to be used again', () => {
  const sys = new System(examples.truss(3));
  sys.dispose();
  assert.throws(() => sys.solve({}), /after dispose/);
});

test('a fully constrained sketch has rank equal to its free parameter count', () => {
  const sk = examples.rectFillets();
  const sys = new System(sk);
  try {
    assert.equal(sys.rank(1e-10, true), sys.nFree);
  } finally {
    sys.dispose();
  }
});

test('every example Jacobian agrees with finite differences', () => {
  for (const name of Object.keys(examples.EXAMPLES)) {
    assert.ok(checkSketch(examples.EXAMPLES[name]()) >= 0, name);
  }
});

/* -- diagnosis -------------------------------------------------------------- */

test('well-constrained examples diagnose clean', () => {
  for (const name of ['rect_fillets', 'slotted_link', 'truss']) {
    const d = diagnose(examples.EXAMPLES[name]());
    assert.equal(d.status, 'well', name);
    assert.equal(d.dof, 0, name);
    assert.equal(d.nRedundant, 0, name);
    assert.deepEqual(d.warnings, [], name);
    assert.ok([...d.entityState.values()].every((s) => s === 'well'), name);
  }
});

test('two contradicting widths give a two-constraint conflict set', () => {
  const sk = examples.rectFillets();
  const extra = new C.Distance(sk.lines[0].p1, sk.lines[0].p2, 50);
  sk.add(extra);
  solve(sk);
  const d = diagnose(sk);
  assert.equal(d.status, 'conflict');
  assert.equal(d.nRedundant, 1);
  const width = sk.constraints.find((c) => c instanceof C.Distance && num(c.d) === 80)!;
  assert.deepEqual(new Set(d.conflicts ?? []), new Set([extra, width]));
  assert.equal(d.entityState.get(sk.lines[0]), 'conflict');
});

test('a redundant but consistent member is over, not conflict', () => {
  const sk = examples.truss(4);
  const p = sk.points[0], q = sk.points[2];
  sk.add(new C.Distance(p, q, Math.hypot(p.x.value - q.x.value, p.y.value - q.y.value)));
  const d = diagnose(sk);
  assert.equal(d.status, 'over');
  assert.equal(d.nRedundant, 1);
  assert.equal(d.violated.length, 0);
  assert.equal(d.conflicts, null);
  assert.equal(d.redundantDistances.length, 1);
});

test('under-constrained slot reports the free parameters the null space sees', () => {
  const sk = examples.slottedLink();
  sk.constraints = sk.constraints.filter((c) => !(c instanceof C.Distance));
  const d = diagnose(sk);
  assert.equal(d.status, 'under');
  assert.equal(d.dof, 1);
  assert.deepEqual(new Set(d.underParams.map((p) => p.name)), new Set(['c2.x', 't2.x', 'b1.x']));
  assert.ok(['c2.x', 'c2.y'].every((n) => d.structuralUnderParams.some((p) => p.name === n)));
  assert.deepEqual(d.components.map((c) => c.dof).sort(), [0, 0, 1]);
  assert.equal(d.entityState.get(sk.points[1]), 'under');
  assert.equal(d.entityState.get(sk.points[0]), 'well');
});

test('the null space pins the left side of an undimensioned rectangle', () => {
  const sk = examples.rectFillets();
  const c = sk.constraints.find((k) => k instanceof C.Distance && num(k.d) === 80)!;
  sk.remove(c);
  const d = diagnose(sk);
  assert.equal(d.dof, 1);
  assert.deepEqual(new Set(d.underParams.map((p) => p.name)),
                   new Set(['b2.x', 'r1.x', 'r2.x', 't1.x', 'c_br.x', 'c_tr.x']));
  const ents = [...sk.lines, ...sk.arcs];
  assert.equal(d.entityState.get(ents[3]), 'well');     // left edge
  assert.equal(d.entityState.get(ents[6]), 'well');
  assert.equal(d.entityState.get(ents[7]), 'well');
  assert.equal(d.entityState.get(ents[0]), 'under');    // bottom, right, top
  assert.equal(d.entityState.get(ents[1]), 'under');
  assert.equal(d.entityState.get(ents[2]), 'under');
});

test('a theorem-type dependency is logged', () => {
  const d = diagnose(examples.polygonChain(8));
  assert.notEqual(d.numericRank, null);
  assert.equal(d.numericRank, d.structuralRank - 1);
  assert.ok(d.warnings.length);
});

test('minimal conflict set of an impossible triangle', () => {
  const sk = examples.impossibleTriangle();
  solve(sk);
  const d = diagnose(sk);
  assert.equal(d.nRedundant, 0);           // the graph sees nothing wrong...
  assert.equal(d.status, 'conflict');      // ...but the numbers do
  const conf = minimalConflictSet(sk);
  assert.ok(conf.length >= 2 && conf.length <= 3);
  assert.ok(conf.every((x) => x instanceof C.Distance));
});

test('distance rigidity merges coincident points', () => {
  const empty = distanceRigidity(examples.polygonChain(5));
  assert.deepEqual(empty.clusters, []);
  assert.deepEqual(empty.redundant, []);
  const sk = new Sketch();
  const l1 = sk.lineXY(0, 0, 10, 0);
  const l2 = sk.lineXY(10, 0, 5, 8);
  const l3 = sk.lineXY(5, 8, 0, 0);
  sk.add(new C.Coincident(l1.p2, l2.p1), new C.Coincident(l2.p2, l3.p1),
         new C.Coincident(l3.p2, l1.p1));
  for (const l of [l1, l2, l3]) sk.add(new C.Distance(l.p1, l.p2, l.length()));
  const { clusters, redundant } = distanceRigidity(sk);
  assert.equal(clusters.length, 1);
  assert.equal(clusters[0].length, 6);
  assert.equal(redundant.length, 0);
});

test('the culprit member is found on a large truss', () => {
  const sk = examples.truss(30);
  const bad = new C.Distance(sk.points[0], sk.points[3], 999);
  sk.add(bad);
  const d = diagnose(sk);
  assert.equal(d.status, 'conflict');
  assert.ok((d.conflicts ?? []).includes(bad));
});

/* -- decomposition ----------------------------------------------------------- */

test('the constraint graph maps the examples', () => {
  let g = buildGraph(examples.rectFillets());
  assert.equal(g.nPoints, 12);
  assert.equal(g.lines.length, 4);
  assert.equal(g.unsupported.length, 0);
  assert.equal(g.virtuals.length, 8);         // one radius line per arc-endpoint tangency
  assert.equal(g.dirs.length, 4 + 8);         // H/V plus the tangency perpendiculars
  g = buildGraph(examples.truss());
  assert.equal(g.passive.length, 30);
  assert.equal(g.lines.length, 1);            // only the horizontal member is an element
  g = buildGraph(examples.polygonChain(6));
  assert.equal(g.unsupported.length, 6);      // EqualLength is not an F-H constraint
});

for (const name of ['rect_fillets', 'slotted_link', 'truss']) {
  test(`${name} fully decomposes and replays exactly`, () => {
    const sk = examples.EXAMPLES[name]();
    const ps = new PlanSolver(sk);
    try {
      assert.ok(ps.plan.fullyDecomposed, ps.plan.summary);
      for (let seed = 0; seed < 3; seed++) {
        sk.perturb(2.0, seed);
        const r = ps.solve(1e-9, false);
        assert.ok(r.success && !r.fellBack && r.maxResidual < 1e-8,
                  `${name} seed ${seed}: max|r| = ${r.maxResidual}`);
      }
      if (name === 'rect_fillets') {
        const d = sk.constraints.find((c) => c instanceof C.Distance && num(c.d) === 80)!;
        d.setValue('d', 120);                 // dimensions are read live: replay, no recompile
        const r = ps.solve(1e-9, false);
        assert.ok(r.success);
        assert.ok(Math.abs(Math.max(...sk.points.map((p) => p.x.value)) - 140) < 1e-6);
      }
    } finally {
      ps.dispose();
    }
  });
}

test('the two distance-to-a-line constraints are PL elements', () => {
  const sk = new Sketch();
  const base = sk.line(sk.point(0, 0, true), sk.point(10, 0, true));
  const other = sk.line(sk.point(1, 3), sk.point(12, 9));
  const p = sk.point(4, 9);
  sk.add(new C.Parallel(base, other), new C.ParallelDistance(base, other, 5),
         new C.Distance(base.p1, other.p1, 5), new C.Distance(other.p1, other.p2, 10),
         new C.PointLineDistance(p, base, 3), new C.Distance(base.p1, p, 5));
  assert.equal(buildGraph(sk).unsupported.length, 0);
  const ps = new PlanSolver(sk);
  try {
    assert.ok(ps.plan.fullyDecomposed, ps.plan.summary);
    const r = ps.solve(1e-9, false);
    assert.ok(r.success && r.maxResidual < 1e-8);
    assert.ok(Math.abs(other.p2.y.value - 5) < 1e-7 && Math.abs(p.y.value - 3) < 1e-7);
  } finally {
    ps.dispose();
  }
});

test('unsupported constraints fall back to the numeric core', () => {
  const sk = examples.polygonChain(8);
  const ps = new PlanSolver(sk);
  try {
    assert.ok(!ps.plan.fullyDecomposed);
    sk.perturb(2.0, 0);
    const r = ps.solve();
    assert.ok(r.success && r.fellBack);
  } finally {
    ps.dispose();
  }
});

test('chirality flags follow the current geometry', () => {
  const sk = new Sketch();
  const a = sk.point(0, 0, true), b = sk.point(10, 0, true);
  const c = sk.point(5, 4);
  sk.add(new C.Distance(a, c, 6), new C.Distance(b, c, 6));
  const ps = new PlanSolver(sk);
  try {
    assert.ok(ps.plan.fullyDecomposed);
    let r = ps.solve(1e-9, false);
    assert.ok(r.success && c.y.value > 0);
    const up = ps.plan.steps.filter((s) => s.ppp).map((s) => s.branch);
    c.y.value = -4;                            // flip the sketch to the other root
    r = ps.solve(1e-9, false);
    assert.ok(r.success && c.y.value < 0);
    assert.ok(up.length);
    assert.deepEqual(ps.plan.steps.filter((s) => s.ppp).map((s) => s.branch), up.map((s) => -s!));
    ps.stickyBranches = true;                  // the recorded root wins even if the sketch moved
    c.y.value = 4;
    ps.solve(1e-9, false);
    assert.ok(c.y.value < 0);
  } finally {
    ps.dispose();
  }
});

test('K3,3 needs a core merge and still decomposes', () => {
  const sk = examples.k33();
  const ps = new PlanSolver(sk);
  try {
    assert.ok(ps.plan.fullyDecomposed, ps.plan.summary);
    assert.ok(Math.max(...ps.plan.steps.map((s) => s.ids.length)) >= 4);
    sk.perturb(1.0, 1);
    const r = ps.solve(1e-9, false);
    assert.ok(r.success && r.maxResidual < 1e-8, `max|r| = ${r.maxResidual}`);
  } finally {
    ps.dispose();
  }
});

test('random Laman frameworks decompose fully', () => {
  for (let seed = 0; seed < 8; seed++) {
    const sk = examples.laman(4 + (seed % 9), 500 + seed);
    const ps = new PlanSolver(sk);
    try {
      assert.ok(ps.plan.fullyDecomposed, `seed ${seed}: ${ps.plan.summary}`);
      sk.perturb(1.0, seed);
      const r = ps.solve(1e-9, false);
      assert.ok(r.success && r.maxResidual < 1e-8, `seed ${seed}: max|r| = ${r.maxResidual}`);
    } finally {
      ps.dispose();
    }
  }
});

test('direction classes: a parallel pair is not a rigid pair', () => {
  const sk = new Sketch();
  const l1 = sk.line(sk.point(0, 0, true), sk.point(10, 0, true));
  const l2 = sk.line(sk.point(0, 5), sk.point(10, 5));
  const l3 = sk.line(sk.point(3, 5), sk.point(3, 12));
  sk.add(new C.Parallel(l1, l2), new C.Distance(l1.p1, l2.p1, 5), new C.Vertical(l3),
         new C.Coincident(l3.p1, l2.p1), new C.Distance(l3.p1, l3.p2, 7),
         new C.Distance(l2.p1, l2.p2, 10));
  const ps = new PlanSolver(sk);
  try {
    sk.perturb(1.0, 0);
    assert.ok(ps.solve().success);
    assert.ok(allSatisfied(sk));
  } finally {
    ps.dispose();
  }
});

test('the plan and the numeric path agree', () => {
  for (const name of Object.keys(examples.EXAMPLES)) {
    for (let seed = 0; seed < 3; seed++) {
      const a = examples.EXAMPLES[name](), b = examples.EXAMPLES[name]();
      a.perturb(1.0, seed);
      b.perturb(1.0, seed);
      const ps = new PlanSolver(a);
      try {
        assert.ok(ps.solve().success, name);
      } finally {
        ps.dispose();
      }
      assert.ok(solve(b).success, name);
      assert.ok(allSatisfied(a) && allSatisfied(b), name);
      if (name !== 'polygon_chain') {
        const xa = a.getX(), xb = b.getX();
        for (let i = 0; i < xa.length; i++) assert.ok(Math.abs(xa[i] - xb[i]) < 1e-4, `${name}[${i}]`);
      }
    }
  }
});

/* -- witness ----------------------------------------------------------------- */

test('the witness sees the concurrent-altitudes dependency', () => {
  const sk = examples.altitudes();
  solve(sk);
  const w = analyze(sk);
  assert.ok(w.dependencies.length >= 1, 'expected a dependent incidence');
  assert.ok(w.dependencies.some((d) => d.theorem));
  assert.ok(w.dependencies[0].impliedBy.length >= 1);
});

test('a floating rigid truss has exactly three rigid-body motions', () => {
  const sk = examples.trussFloating(6);
  const w = analyze(sk);
  assert.equal(w.motions.filter((m) => m.rigid).length, 3);
  assert.equal(w.motions.filter((m) => !m.rigid).length, 0);
});

test('witness analysis of the equal-length ring finds the cycle redundancy', () => {
  const sk = examples.polygonChain(6);
  const w = analyze(sk);
  assert.ok(w.dependencies.length >= 1);
});

/* -- homotopy ----------------------------------------------------------------- */

test('homotopy enumerates alternative roots of a triangle merge', () => {
  const sk = new Sketch();
  const a = sk.point(0, 0, true), b = sk.point(10, 0, true);
  const c = sk.point(5, 4);
  sk.add(new C.Distance(a, c, 6), new C.Distance(b, c, 6));
  const ps = new PlanSolver(sk);
  try {
    ps.solve(1e-9, false);
    const idx = ps.plan.steps.findIndex((s) => s.ppp !== null);
    assert.ok(idx >= 0);
    const alts = enumerateStep(ps, idx);
    assert.ok(alts.length >= 2, `expected both roots, got ${alts.length}`);
    assert.ok(alts[0].distance < 1e-6);
  } finally {
    ps.dispose();
  }
});

/* -- dragging ------------------------------------------------------------------ */

test('numeric drag keeps the constraints satisfied', () => {
  const sk = examples.rectFillets();
  solve(sk);
  const sys = new System(sk);
  const p = sk.lines[1].p2;
  const d = new Drag(sk, p, p.x.value, p.y.value);
  try {
    for (let i = 0; i < 8; i++) {
      const r = d.move(p.x.value + 3, p.y.value + 1);
      assert.ok(sys.maxRelativeResidual() < 1e-6, `frame ${i}: ${r.maxResidual}`);
    }
  } finally {
    d.end();
    sys.dispose();
  }
});

test('dragging the edge of a free circle changes its radius', () => {
  const sk = new Sketch();
  const c = sk.circle(sk.point(0, 0, true), 10);
  const d = new RadiusDrag(sk, c, c.radius.value);
  try {
    for (const target of [25, 4, 12.5]) {
      assert.ok(d.move(target).success);
      assert.ok(Math.abs(c.radius.value - target) < 1e-6, `${c.radius.value} != ${target}`);
    }
  } finally {
    d.end();
  }
  assert.ok(!sk.constraints.some((x) => x.soft));
});

test('a dimensioned radius does not follow the cursor', () => {
  const sk = new Sketch();
  const c = sk.circle(sk.point(0, 0, true), 10);
  sk.add(new C.Radius(c, 10));
  const d = new RadiusDrag(sk, c, c.radius.value);
  try {
    d.move(30);
  } finally {
    d.end();
  }
  assert.ok(Math.abs(c.radius.value - 10) < 1e-6);
});

test('resizing an arc carries its endpoints', () => {
  const sk = new Sketch();
  const centre = sk.point(0, 0, true);
  const arc = sk.arc(centre, sk.point(10, 0), sk.point(0, 10));
  solve(sk);
  const d = new RadiusDrag(sk, arc, arc.radius.value);
  try {
    assert.ok(d.move(17).success);
  } finally {
    d.end();
  }
  assert.ok(Math.abs(arc.radius.value - 17) < 1e-6);
  for (const p of [arc.start, arc.end]) {
    assert.ok(Math.abs(Math.hypot(p.x.value, p.y.value) - 17) < 1e-6);
  }
});

test('resizing one circle of an EqualRadius pair carries the other', () => {
  const sk = new Sketch();
  const a = sk.circle(sk.point(0, 0, true), 10);
  const b = sk.circle(sk.point(40, 0, true), 10);
  sk.add(new C.EqualRadius(a, b));
  const d = new RadiusDrag(sk, a, a.radius.value);
  try {
    assert.ok(d.move(18).success);
  } finally {
    d.end();
  }
  assert.ok(Math.abs(a.radius.value - 18) < 1e-6);
  assert.ok(Math.abs(b.radius.value - 18) < 1e-6);
});

test('plan drag replays the cached plan', () => {
  const sk = examples.truss(8);
  solve(sk);
  const p = sk.points[10];
  const d = new PlanDrag(sk, p, p.x.value, p.y.value);
  try {
    for (let i = 0; i < 6; i++) {
      const r = d.move(p.x.value + 2, p.y.value + 2);
      assert.ok(r.success, `frame ${i}`);
    }
    assert.ok(allSatisfied(sk));
  } finally {
    d.end();
  }
});

/* -- I/O ------------------------------------------------------------------------ */

test('JSON round-trips every example', () => {
  for (const name of Object.keys(examples.EXAMPLES)) {
    const sk = examples.EXAMPLES[name]();
    const s = io.dumps(sk);
    const back = io.loads(s);
    assert.equal(io.dumps(back), s, name);
    assert.equal(back.constraints.length, sk.constraints.length, name);
    assert.ok(solve(back).success, name);
  }
});

test('removing an arc centre removes the arc and its tangencies', () => {
  const sk = examples.rectFillets();
  const nArcC = sk.constraints.filter((c) => c instanceof C.TangentArcLine).length;
  const back = io.without(sk, [sk.arcs[0].center]);
  assert.equal(back.arcs.length, 3);
  assert.equal(back.constraints.filter((c) => c instanceof C.TangentArcLine).length, nArcC - 2);
  io.dumps(back);          // every kept constraint references only live entities
});

test('every constraint type declares a spec that reconstructs it', () => {
  for (const [name, T] of Object.entries(C.CONSTRAINT_TYPES)) {
    assert.ok(T.spec.length > 0, name);
    assert.equal(T.defaults.length, T.spec.length, name);
  }
});

test('a live drag never reaches the document', () => {
  const sk = examples.slottedLink();
  const n = io.toJSON(sk).constraints.length;
  const d = new Drag(sk, sk.points[1], 1, 2);
  const r = new RadiusDrag(sk, sk.circles[0], 9);
  try {
    assert.equal(io.toJSON(sk).constraints.length, n);
    assert.equal(io.loads(io.dumps(sk)).constraints.length, sk.constraints.length - 2);
  } finally {
    r.end();
    d.end();
  }
});

test('a soft radius is not a known dimension', () => {
  const sk = new Sketch();
  const c = sk.circle(sk.point(0, 0, true), 10);
  const d = new RadiusDrag(sk, c, 10);
  try {
    assert.equal(Object.keys(buildGraph(sk).knownRadius).length, 0);
  } finally {
    d.end();
  }
});

test('window selection is exactly "the entity bounds are inside the box"', () => {
  const sk = new Sketch();
  const centre = sk.point(0, 0);
  const arc = sk.arc(centre, sk.point(5, 0), sk.point(-5, 0));   // half turn through the top
  const inside = (box: number[], b: number[]): boolean =>
    b[0] >= box[0] && b[1] >= box[1] && b[2] <= box[2] && b[3] <= box[3];
  assert.ok(inside([-6, -1, 6, 6], arc.bounds()));               // the bulge fits
  assert.ok(!inside([-6, -1, 6, 4], arc.bounds()));              // ...and is what excludes it
});

test('drawn bounds cover curves, not just points', () => {
  const sk = new Sketch();
  sk.circle(sk.point(0, 0), 10);
  assert.deepEqual(sk.bbox(), [0, 0, 0, 0]);
  assert.deepEqual(sk.drawnBounds(), [-10, -10, 10, 10]);

  const sk2 = new Sketch();
  const centre = sk2.point(0, 0);
  const arc = sk2.arc(centre, sk2.point(5, 0), sk2.point(0, 5));
  const close = (b: number[], want: number[]): void => {
    b.forEach((v, i) => assert.ok(Math.abs(v - want[i]) < 1e-9, `${b} != ${want}`));
  };
  close(arc.bounds(), [0, 0, 5, 5]);                 // a quarter turn: the ends bound it
  arc.end.x.value = -5; arc.end.y.value = 0;
  close(arc.bounds(), [-5, 0, 5, 5]);                // a half turn bulges through the top
  arc.end.x.value = 5; arc.end.y.value = -1e-12;
  close(arc.bounds(), [-5, -5, 5, 5]);               // nearly the whole circle
});

test('a three-point arc sweeps through the third point', () => {
  const sk = new Sketch();
  const a = sk.point(-5, 0), b = sk.point(5, 0);
  const up = sk.arcThrough(a, b, [0, 5])!;
  assert.ok(Math.hypot(...up.center.xy) < 1e-12);
  assert.ok(Math.abs(up.radius.value - 5) < 1e-12);
  assert.equal(up.start, b);
  assert.equal(up.end, a);
  const [a0, a1] = up.angles();
  assert.ok(Math.abs(a0) < 1e-12 && Math.abs(a1 - Math.PI) < 1e-12);

  const sk2 = new Sketch();
  const c = sk2.point(-5, 0), d = sk2.point(5, 0);
  const down = sk2.arcThrough(c, d, [0, -5])!;         // same chord, other side
  assert.equal(down.start, c);
  assert.equal(down.end, d);
  assert.ok(Math.abs(down.radius.value - 5) < 1e-12);
  const [b0, b1] = down.angles();
  assert.ok(Math.abs(b0 - Math.PI) < 1e-12 && Math.abs(b1 - 2 * Math.PI) < 1e-12);
  for (const arc of [up, down]) {
    for (const p of [arc.start, arc.end]) {
      const dist = Math.hypot(p.x.value - arc.center.x.value, p.y.value - arc.center.y.value);
      assert.ok(Math.abs(dist - arc.radius.value) < 1e-12);
    }
  }
});

test('a three-point arc refuses collinear input', () => {
  const sk = new Sketch();
  const a = sk.point(0, 0), b = sk.point(10, 0);
  const n = sk.points.length;
  assert.equal(sk.arcThrough(a, b, [5, 0]), null);
  assert.equal(sk.arcThrough(a, b, [20, 1e-12]), null);  // scale-free, not an absolute epsilon
  assert.equal(sk.points.length, n);                     // nothing was created
  assert.ok(sk.arcThrough(a, b, [5, 0.01]));             // a real, very flat arc is fine
});

test('DOF counts what can actually move, not what the matching sees', () => {
  const sk = examples.altitudes();            // the altitudes concur: only the numbers see it
  solve(sk);
  const d = diagnose(sk);
  assert.equal(d.geometricDependency, 1);
  assert.equal(d.structuralDof, 2);           // what the matching alone believes
  assert.equal(d.dof, 3);                     // what is actually free to move
  assert.equal(d.dof, d.nParams - (d.numericRank ?? 0));
  assert.ok(d.underParams.length >= d.dof);   // and dragging agrees with the count
});

test('a dependency with nothing to remove is not called over-constrained', () => {
  const sk = new Sketch();
  const a = sk.point(0, 0);
  const centre = sk.point(10, 0);
  const line = sk.line(a, centre);
  const arc = sk.arc(centre, sk.point(13, 4), sk.point(13, -4));
  sk.add(new C.Symmetric(arc.start, arc.end, line));
  solve(sk);
  const d = diagnose(sk);
  assert.equal(d.geometricDependency, 1);       // the deficiency is real...
  assert.equal(d.nRedundant, 1);
  assert.deepEqual(d.over, []);                 // ...but nothing is removable
  assert.equal(d.status, 'under');
  assert.ok(d.dof > 0);
});

test('redundancy the matching cannot see is counted and named', () => {
  const sk = examples.altitudes();
  solve(sk);
  const d = diagnose(sk);
  assert.equal(d.structuralNRedundant, 0);       // what the matching alone believes
  assert.equal(d.nRedundant, 1);
  assert.equal(d.status, 'over');
  const named = new Set(d.over.map((c) => io.describe(c)));
  assert.deepEqual([...named].sort(), [
    'Perpendicular(L3, L1)', 'Perpendicular(L4, L2)', 'Perpendicular(L5, L0)',
    'PointOnLine(P6, L3)', 'PointOnLine(P6, L4)', 'PointOnLine(P6, L5)',
  ]);
});

test('DOF is unchanged when the two ranks agree', () => {
  const sk = examples.rectFillets();
  solve(sk);
  const d = diagnose(sk);
  assert.equal(d.geometricDependency, 0);
  assert.equal(d.dof, 0);
  assert.equal(d.structuralDof, 0);
  assert.equal(d.status, 'well');
  assert.equal(d.underParams.length, 0);
});

test('sameConstraint matches exact repeats', () => {
  const sk = new Sketch();
  const p = sk.point(0, 0), q = sk.point(3, 0), r = sk.point(0, 4);
  const line = sk.line(p, q);
  assert.ok(C.sameConstraint(new C.Coincident(p, q), new C.Coincident(p, q)));
  assert.ok(!C.sameConstraint(new C.Coincident(p, q), new C.Coincident(p, r)));
  assert.ok(!C.sameConstraint(new C.Coincident(p, q), new C.Midpoint(p, line)));
  assert.ok(C.sameConstraint(new C.Distance(p, q, 5), new C.Distance(p, q, 5)));
  assert.ok(!C.sameConstraint(new C.Distance(p, q, 5), new C.Distance(p, q, 6)));  // a conflict
  assert.ok(C.sameConstraint(new C.Symmetric(p, q, line), new C.Symmetric(p, q, line)));
});

test('sameConstraint sees through a swapped pair, but only where that is a no-op', () => {
  const sk = new Sketch();
  const p = sk.point(0, 0), q = sk.point(3, 0);
  const l1 = sk.line(p, q), l2 = sk.line(sk.point(0, 5), sk.point(3, 5));
  const c1 = sk.circle(p, 2), c2 = sk.circle(q, 3);
  assert.ok(C.sameConstraint(new C.Coincident(p, q), new C.Coincident(q, p)));
  assert.ok(C.sameConstraint(new C.Parallel(l1, l2), new C.Parallel(l2, l1)));
  assert.ok(C.sameConstraint(new C.Perpendicular(l1, l2), new C.Perpendicular(l2, l1)));
  assert.ok(C.sameConstraint(new C.EqualLength(l1, l2), new C.EqualLength(l2, l1)));
  assert.ok(C.sameConstraint(new C.EqualRadius(c1, c2), new C.EqualRadius(c2, c1)));
  assert.ok(C.sameConstraint(new C.Distance(p, q, 5), new C.Distance(q, p, 5)));
  assert.ok(C.sameConstraint(new C.Symmetric(p, q, l1), new C.Symmetric(q, p, l1)));
  // the first argument is the reference for these, so a swap means something else
  assert.ok(!C.sameConstraint(new C.Angle(l1, l2, 0.7), new C.Angle(l2, l1, 0.7)));
  assert.ok(!C.sameConstraint(new C.ParallelDistance(l1, l2, 4), new C.ParallelDistance(l2, l1, 4)));
  assert.ok(!C.sameConstraint(new C.AnnularDistance(c1, c2, 1), new C.AnnularDistance(c2, c1, 1)));
});

test('annular distance sets the ring thickness', () => {
  const sk = new Sketch();
  const c = sk.point(0, 0, true);
  const inner = sk.circle(c, 10), outer = sk.circle(c, 12);
  sk.add(new C.Radius(inner, 10), new C.AnnularDistance(inner, outer, 3));
  assert.ok(solve(sk).success);
  assert.ok(Math.abs(outer.radius.value - 13) < 1e-9);
});

test('annular distance is signed and drives either circle', () => {
  const sk = new Sketch();
  const c = sk.point(0, 0, true);
  const inner = sk.circle(c, 10), outer = sk.circle(c, 12);
  sk.add(new C.Radius(outer, 20), new C.AnnularDistance(inner, outer, -4));
  assert.ok(solve(sk).success);
  assert.ok(Math.abs(inner.radius.value - 24) < 1e-9);   // negative d flips which is outer
});

test('annular distance carries a known radius along a chain of rings', () => {
  const sk = new Sketch();
  const c = sk.point(0, 0, true);
  const inner = sk.circle(c, 10), mid = sk.circle(c, 13), outer = sk.circle(c, 15);
  sk.add(new C.Radius(inner, 10), new C.AnnularDistance(inner, mid, 3),
         new C.AnnularDistance(mid, outer, 2));
  const g = buildGraph(sk);
  assert.equal(g.unsupported.length, 0);
  assert.equal(g.knownRadius[String(mid.radius.index)], 13);   // resolves from the one dimension
  assert.equal(g.knownRadius[String(outer.radius.index)], 15);
});

test('point-line distance offsets a point from a line', () => {
  const sk = new Sketch();
  const base = sk.line(sk.point(0, 0, true), sk.point(10, 0, true));
  const p = sk.point(4, 9);
  sk.add(new C.PointLineDistance(p, base, 3));
  assert.ok(solve(sk).success);
  assert.ok(Math.abs(p.y.value - 3) < 1e-7);          // left of a +x base is +y
  assert.ok(Math.abs(p.x.value - 4) < 1e-7);          // it slides only perpendicular
});

test('the sign of a point-line distance picks the side', () => {
  const sk = new Sketch();
  const base = sk.line(sk.point(0, 0, true), sk.point(10, 0, true));
  const p = sk.point(4, 9);
  sk.add(new C.PointLineDistance(p, base, -3));
  assert.ok(solve(sk).success);
  assert.ok(Math.abs(p.y.value + 3) < 1e-7);
});

test('point-line distance measures to the infinite line', () => {
  const sk = new Sketch();
  const base = sk.line(sk.point(0, 0, true), sk.point(1, 0, true));
  const p = sk.point(50, 9);
  sk.add(new C.PointLineDistance(p, base, 3));
  assert.ok(solve(sk).success);
  assert.ok(Math.abs(p.y.value - 3) < 1e-7 && Math.abs(p.x.value - 50) < 1e-7);
});

test('parallel distance dimensions the gap between parallel lines', () => {
  const sk = new Sketch();
  const base = sk.line(sk.point(0, 0, true), sk.point(10, 0, true));
  const other = sk.line(sk.point(1, 3), sk.point(12, 9));
  sk.add(new C.Parallel(base, other), new C.ParallelDistance(base, other, 5));
  assert.ok(solve(sk).success);
  for (const p of [other.p1, other.p2]) assert.ok(Math.abs(p.y.value - 5) < 1e-7);
});

test('parallel distance does not itself make lines parallel', () => {
  const sk = new Sketch();
  const base = sk.line(sk.point(0, 0, true), sk.point(10, 0, true));
  const other = sk.line(sk.point(1, 3), sk.point(12, 9));
  sk.add(new C.ParallelDistance(base, other, 5));
  assert.ok(solve(sk).success);
  assert.ok(Math.abs(other.p1.y.value - 5) < 1e-7);
  const [d1, d2] = [base.direction(), other.direction()];
  assert.ok(Math.abs(d1[0] * d2[1] - d1[1] * d2[0]) > 1e-6);   // still skew
});

test('the sign of a parallel distance picks the side', () => {
  const sk = new Sketch();
  const base = sk.line(sk.point(0, 0, true), sk.point(10, 0, true));
  const other = sk.line(sk.point(1, 3), sk.point(12, 9));
  sk.add(new C.Parallel(base, other), new C.ParallelDistance(base, other, -5));
  assert.ok(solve(sk).success);
  assert.ok(Math.abs(other.p1.y.value + 5) < 1e-7);
});

test('a rectangle is rigid up to its five degrees of freedom', () => {
  const sk = new Sketch();
  const lines = sk.rectangleXY(0, 0, 40, 25);
  assert.equal(lines.length, 4);
  assert.equal(sk.points.length, 4);                 // corners are shared, not duplicated
  const d = diagnose(sk);
  assert.equal(d.nRedundant, 0);
  assert.equal(d.dof, 5);                            // position, rotation, width, height
  sk.perturb(3.0, 1);
  assert.ok(solve(sk).success);
  for (let i = 0; i < 4; i++) {
    const u = lines[i].direction(), v = lines[(i + 1) % 4].direction();
    assert.ok(Math.abs(u[0] * v[0] + u[1] * v[1]) < 1e-6, `corner ${i}`);
  }
});

test('symmetric mirrors two points about a line', () => {
  const sk = new Sketch();
  const a = sk.point(0, 0, true), b = sk.point(0, 10, true);    // the axis: x = 0
  const p = sk.point(-3, 4), q = sk.point(9, 1);
  sk.add(new C.Symmetric(p, q, sk.line(a, b)));
  assert.ok(solve(sk).success);
  assert.ok(Math.abs(p.x.value + q.x.value) < 1e-9);
  assert.ok(Math.abs(p.y.value - q.y.value) < 1e-9);
});

test('the construction flag round-trips', () => {
  const sk = examples.slottedLink();
  sk.lines[0].construction = true;
  sk.arcs[0].construction = true;
  sk.circles[0].construction = true;
  const s = io.dumps(sk);
  const back = io.loads(s);
  assert.deepEqual(back.lines.map((l) => l.construction), sk.lines.map((l) => l.construction));
  assert.deepEqual(back.arcs.map((a) => a.construction), sk.arcs.map((a) => a.construction));
  assert.deepEqual(back.circles.map((c) => c.construction), sk.circles.map((c) => c.construction));
  assert.equal(io.dumps(back), s);
});

test('describe reads the spec', () => {
  const sk = examples.rectFillets();
  const d = sk.constraints.find((c) => c instanceof C.Distance)!;
  assert.match(io.describe(d), /^Distance\(P\d+, P\d+, 80\)$/);
});

test('deleting an entity removes what depends on it', () => {
  const sk = examples.truss(4);
  const before = sk.constraints.length;
  const back = io.without(sk, [sk.points[0]]);
  assert.equal(back.points.length, sk.points.length - 1);
  assert.ok(back.constraints.length < before);
  assert.ok(solve(back).success);
});

/* -- the ABI between the binding and the core ----------------------------------- */

test('constraint errors are sized from the plan, not the live sketch', () => {
  // The core reports one entry per *compiled* constraint; sizing from the live sketch's count
  // writes past the end of the buffers as soon as the sketch is edited.
  const sk = new Sketch();
  const a = sk.point(0, 0), b = sk.point(10, 0), c = sk.point(10, 10);
  const d1 = new C.Distance(a, b, 10), d2 = new C.Distance(b, c, 10);
  sk.add(d1, d2);
  const sys = new System(sk);
  try {
    assert.equal(sys.nConstraints, 2);
    sk.remove(d2);
    assert.equal(sys.constraintErrors().size, 1);   // d2's proxy is detached, d1's is not
    assert.equal(sys.nConstraints, 2);              // the plan did not recompile
  } finally {
    sys.dispose();
  }
});

test('a dangling reference in a document is an error, not a trap', () => {
  // A document is untrusted input; a bad index has to come back as a thrown error with the
  // wasm instance still usable afterwards.
  for (const bad of [
    '{"points":[{"x":0,"y":0}],"arcs":[{"center":7,"start":0,"end":0,"r":1}]}',
    '{"points":[{"x":0,"y":0}],"lines":[{"p1":0,"p2":4}]}',
    '{"points":[{"x":0,"y":0}],"circles":[{"center":-1,"r":1}]}',
    '{"points":[{"x":0,"y":0}],"constraints":[{"type":"Horizontal","args":[["line",0]]}]}',
  ]) {
    assert.throws(() => io.loads(bad), /out of range/);
  }
  const sk = io.loads('{"points":[{"x":1,"y":2}]}');
  assert.deepEqual(sk.points[0].xy, [1, 2]);
});

test('rowOf an uncompiled constraint is -1, not a trap', () => {
  const sk = new Sketch();
  const a = sk.point(0, 0), b = sk.point(10, 0);
  sk.line(a, b);
  const d = new C.Distance(a, b, 10);
  sk.add(d);
  const sys = new System(sk);
  try {
    assert.ok(sys.rowOf(d) >= 0);
    const later = new C.Horizontal(sk.lines[0]);
    sk.add(later);
    assert.equal(sys.rowOf(later), -1);
  } finally {
    sys.dispose();
  }
});

test('every function the Abi interface declares is exported by the module', async () => {
  // one hand-kept list describes the boundary now (the `Abi` interface); this turns a
  // rename in Rust from a runtime failure into a test failure
  const { readFileSync } = await import('node:fs');
  const { fileURLToPath } = await import('node:url');
  const src = readFileSync(fileURLToPath(new URL('../../src/core/wasm.ts', import.meta.url)), 'utf8');
  const decl = src.slice(src.indexOf('export interface Abi'), src.indexOf('let abi:'));
  const names = [...new Set(decl.match(/\bgcs_\w+/g) ?? [])];
  assert.ok(names.length > 80, `expected the full API, found ${names.length}`);
  const m = core() as unknown as Record<string, unknown>;
  for (const n of names) assert.equal(typeof m[n], 'function', `${n} is declared but not exported`);
});

test('the registry the binding generates its classes from matches the kernels', () => {
  const reg = C.REGISTRY();
  assert.equal(reg.kernels.length, core().gcs_kernel_count());
  assert.equal(reg.types.length, Object.keys(C.CONSTRAINT_TYPES).length);
  for (const t of reg.types) {
    assert.ok(t.kernel >= 0 && t.kernel < reg.kernels.length, t.name);
    assert.equal(C.CONSTRAINT_TYPES[t.name].kernelId, t.kernel);
  }
});
