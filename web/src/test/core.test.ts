/* The TypeScript front end against the WebAssembly core — the same properties the Python
 * suite asserts: kernels and the compiled plan, both solvers, structural diagnosis, the
 * decomposition plan, witness analysis, homotopy enumeration, dragging and JSON I/O. */
import assert from 'node:assert/strict';
import test from 'node:test';

import * as C from '../core/constraints.js';
import * as examples from '../core/examples.js';
import * as graph from '../core/graph.js';
import * as io from '../core/io.js';
import { build } from '../core/cgraph.js';
import { PlanDrag, PlanSolver, decompose } from '../core/decompose.js';
import { diagnose, distanceRigidity, minimalConflictSet, violatedConstraints } from '../core/diagnose.js';
import { enumerateStep } from '../core/homotopy.js';
import { Sketch } from '../core/model.js';
import { Rng } from '../core/rng.js';
import { Drag, System, solve } from '../core/system.js';
import { analyze } from '../core/witness.js';
import { initCore } from '../core/wasm.js';

await initCore();

const allSatisfied = (sk: Sketch): boolean => {
  const s = new System(sk);
  try {
    return violatedConstraints(s).length === 0;
  } finally {
    s.dispose();
  }
};

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
    const rng = new Rng(seed);
    const n = 4 + rng.int(11);
    const edges = examples.hennebergEdges(n, rng);
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

test('solve on a large truss uses the sparse path', () => {
  const sk = examples.truss(60);
  const sys = new System(sk);
  try {
    assert.ok(sys.nFree > 120);
    sk.perturb(0.4, 1);
    const res = sys.solve({});
    assert.ok(res.success, `max|r| = ${res.maxResidual}`);
  } finally {
    sys.dispose();
  }
});

test('a fully constrained sketch has rank equal to its free parameter count', () => {
  const sk = examples.rectFillets();
  const sys = new System(sk);
  try {
    assert.equal(sys.rank(undefined, 1e-10, true), sys.nFree);
  } finally {
    sys.dispose();
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
  const width = sk.constraints.find((c) => c instanceof C.Distance && c.d === 80)!;
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
  const c = sk.constraints.find((k) => k instanceof C.Distance && k.d === 80)!;
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
  sk.add(new C.Coincident(l1.p2, l2.p1), new C.Coincident(l2.p2, l3.p1), new C.Coincident(l3.p2, l1.p1));
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
  let g = build(examples.rectFillets());
  assert.equal(g.nPoints, 12);
  assert.equal(g.lines.length, 4);
  assert.equal(g.unsupported.length, 0);
  assert.equal(g.virtual.length, 8);          // one radius line per arc-endpoint tangency
  assert.equal(g.dirs.length, 4 + 8);         // H/V plus the tangency perpendiculars
  g = build(examples.truss());
  assert.equal(g.passive.length, 30);
  assert.equal(g.lines.length, 1);            // only the horizontal member is an element
  g = build(examples.polygonChain(6));
  assert.equal(g.unsupported.length, 6);      // EqualLength is not an F-H constraint
});

for (const name of ['rect_fillets', 'slotted_link', 'truss']) {
  test(`${name} fully decomposes and replays exactly`, () => {
    const sk = examples.EXAMPLES[name]();
    const ps = new PlanSolver(sk);
    try {
      assert.ok(ps.plan.fullyDecomposed, ps.plan.summary());
      for (let seed = 0; seed < 3; seed++) {
        sk.perturb(2.0, seed);
        const r = ps.solve(1e-9, false);
        assert.ok(r.success && !r.fellBack && r.maxResidual < 1e-8,
                  `${name} seed ${seed}: max|r| = ${r.maxResidual}`);
      }
      if (name === 'rect_fillets') {
        const d = sk.constraints.find((c) => c instanceof C.Distance && c.d === 80) as C.Distance;
        d.d = 120;                            // dimensions are read live: replay, no recompile
        const r = ps.solve(1e-9, false);
        assert.ok(r.success);
        assert.ok(Math.abs(Math.max(...sk.points.map((p) => p.x.value)) - 140) < 1e-6);
      }
    } finally {
      ps.dispose();
    }
  });
}

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
    ps.plan.stickyBranches = true;             // the recorded root wins even if the sketch moved
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
    assert.ok(ps.plan.fullyDecomposed, ps.plan.summary());
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
    const rng = new Rng(500 + seed);
    const n = 4 + rng.int(9);
    const edges = examples.hennebergEdges(n, rng);
    const sk = new Sketch();
    const pts = Array.from({ length: n }, () => sk.point(rng.uniform(0, 50), rng.uniform(0, 50)));
    pts[0].fix();
    for (const [a, b] of edges) {
      sk.add(new C.Distance(pts[a], pts[b],
        Math.hypot(pts[a].x.value - pts[b].x.value, pts[a].y.value - pts[b].y.value)));
    }
    sk.add(new C.Horizontal(sk.line(pts[0], pts[1])));
    const ps = new PlanSolver(sk);
    try {
      assert.ok(ps.plan.fullyDecomposed, `seed ${seed}: ${ps.plan.summary()}`);
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
         new C.Coincident(l3.p1, l2.p1), new C.Distance(l3.p1, l3.p2, 7), new C.Distance(l2.p1, l2.p2, 10));
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
    const alts = enumerateStep(ps.plan, idx);
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
  const p = sk.lines[1].p2;
  const d = new Drag(sk, p, p.x.value, p.y.value);
  try {
    for (let i = 0; i < 8; i++) {
      const r = d.move(p.x.value + 3, p.y.value + 1);
      assert.ok(r.maxResidual < 1e-6 * d.polish.scale, `frame ${i}: ${r.maxResidual}`);
    }
  } finally {
    d.end();
  }
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
    // a second dump is byte-identical: the document round-trips, not the parameter order
    // (rebuilding creates every point before the arc radii, so the vector is laid out differently)
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
  }
});

test('describe reads the spec', () => {
  const sk = examples.rectFillets();
  const ix = new io.Index(sk);
  const d = sk.constraints.find((c) => c instanceof C.Distance) as C.Distance;
  assert.match(io.describe(d, ix), /^Distance\(P\d+, P\d+, 80\)$/);
});

test('deleting an entity removes what depends on it', () => {
  const sk = examples.truss(4);
  const before = sk.constraints.length;
  const back = io.without(sk, [sk.points[0]]);
  assert.equal(back.points.length, sk.points.length - 1);
  assert.ok(back.constraints.length < before);
  assert.ok(solve(back).success);
});
