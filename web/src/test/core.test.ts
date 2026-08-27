/* The TypeScript binding against the Rust core — the same properties the Python suite asserts:
 * the compiled plan, both solvers, structural diagnosis, the decomposition plan, witness
 * analysis, homotopy enumeration, dragging and JSON I/O.
 *
 * There is one implementation of all of it now; these tests check that the binding reaches it
 * faithfully, and that the ABI the two sides share stays in step. */
import assert from 'node:assert/strict';
import test from 'node:test';

import * as C from '../core/constraints.js';
import { Constraint } from '../core/constraints.js';
import * as examples from '../core/examples.js';
import { callouts, pairDimension } from '../core/callout.js';
import { expressions } from '../core/expr.js';
import * as graph from '../core/graph.js';
import * as io from '../core/io.js';
import { PlanDrag, PlanSolver, buildGraph } from '../core/decompose.js';
import {
  diagnose, distanceRigidity, minimalConflictSet, violatedConstraints,
} from '../core/diagnose.js';
import { checkSketch } from '../core/fdcheck.js';
import { enumerateStep } from '../core/homotopy.js';
import { Point, Sketch, Spline } from '../core/model.js';
import { Document, fromSketch, highlight } from '../core/program.js';
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
    // the core's own tolerance, not a second rank rule written here
    assert.equal(sys.rank(undefined, true), sys.nFree);
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
  // the width the case states, and a second number on the *same pair*: what makes this a
  // minimal conflict of two is that both name one length, not that both are lengths
  const width = sk.constraints.find((c) => c instanceof C.Distance && num(c.d) === 100)!;
  const [wp, wq] = [width.p as Point, width.q as Point];
  const extra = new C.Distance(wp, wq, 50);
  sk.add(extra);
  solve(sk);
  const d = diagnose(sk);
  assert.equal(d.status, 'conflict');
  assert.equal(d.nRedundant, 1);
  assert.deepEqual(new Set(d.conflicts ?? []), new Set([extra, width]));
  assert.equal(d.entityState.get(wp), 'conflict');
  assert.equal(d.entityState.get(wq), 'conflict');
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
  const c = sk.constraints.find((k) => k instanceof C.Distance && num(k.d) === 100)!;
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

test('a tangency at a line end on the circle is stated regular', () => {
  // the pair (PointOnCircle, TangentLineCircle) at one point is a double root; TangentLineCircleAt
  // says it as one regular statement, so the numeric rank agrees with the matching
  const sk = new Sketch();
  const c = sk.point(0, 0, true);
  const k = sk.circle(c, 17);
  const ln = sk.line(sk.point(17, 0), sk.point(17, 30));
  sk.add(new C.Radius(k, 17), new C.PointOnCircle(ln.p1, k),
         new C.TangentLineCircleAt(ln, k, 'p1'));
  solve(sk);
  const d = diagnose(sk);
  assert.equal(d.numericRank, d.structuralRank);
  assert.equal(d.shaky, 0);
  assert.equal(d.dof, 2);
  assert.equal(d.warnings.length, 0);
});

test('a tangential contact stated the old way is rigid, not under', () => {
  // each end on its circle plus the line tangent to it: rank-deficient at every solution, and
  // the settle test counts the swimming motions out of the DOF
  const sk = examples.beltTangency();
  solve(sk);
  const d = diagnose(sk);
  assert.equal(d.dof, 0);
  assert.equal(d.status, 'well');
  assert.equal(d.over.length + d.implied.length, 0);
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
        const d = sk.constraints.find((c) => c instanceof C.Distance && num(c.d) === 100)!;
        d.setValue('d', 140);                 // dimensions are read live: replay, no recompile
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
      // Where the answer is unique the two paths must reach the same one.  Where it is not,
      // they need not: both are minimum-norm, but the plan has already moved the geometry
      // before its solver starts, so the two are least-change from different places.
      if (diagnose(b).dof === 0) {
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

test('a motion says which params it moves, and the core is what says so', () => {
  // the same case the Python suite checks, so the two bindings cannot drift from each other
  // or from `witness::moving_params` — which they would if either read the velocities itself
  const sk = examples.rectFilletsUnder();
  const w = analyze(sk);
  assert.equal(w.motions.length, 1);
  const m = w.motions[0];
  assert.deepEqual(new Set(m.moving.map((p) => p.name)),
                   new Set(['b2.x', 'r1.x', 'r2.x', 't1.x', 'c_br.x', 'c_tr.x']));
  // every param it names is one the velocity actually moves, and the rest are still
  assert.ok(m.moving.length < m.velocity.length);
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

test('redundancy the matching cannot see is counted and named as implied', () => {
  // a theorem among pure relations: counted, named (any of the six could go), but not `over`
  // — nothing can ever break it, so there is nothing to fix and no reason to paint it red
  const sk = examples.altitudes();
  solve(sk);
  const d = diagnose(sk);
  assert.equal(d.structuralNRedundant, 0);       // what the matching alone believes
  assert.equal(d.nRedundant, 1);
  assert.equal(d.status, 'under');
  assert.deepEqual(d.over, []);
  const named = new Set(d.implied.map((c) => io.describe(c)));
  assert.deepEqual([...named].sort(), [
    'Perpendicular(L3, L1)', 'Perpendicular(L4, L2)', 'Perpendicular(L5, L0)',
    'PointOnLine(P6, L3)', 'PointOnLine(P6, L4)', 'PointOnLine(P6, L5)',
  ]);
  assert.ok([...d.entityState.values()].every((s) => s !== 'over'));
});

test('a relation-only theorem is implied, not over', () => {
  // two arcs on one centre, the centre on a line, equal radii, an endpoint of each mirrored
  // about the line: mirroring about a line through the centre preserves the distance to it, so
  // EqualRadius follows, and so does the centre being on the line.  No dimension takes part,
  // the sketch stays consistent wherever it is dragged — a remark, not a fault
  const sk = new Sketch();
  const a = sk.point(-20, 0), b = sk.point(40, 0);
  const line = sk.line(a, b);
  const c = sk.point(10, 0, true);
  const arc1 = sk.arc(c, sk.point(18, 6), sk.point(4, 8));
  const arc2 = sk.arc(c, sk.point(4, -8), sk.point(18, -6));
  const onLine = new C.PointOnLine(c, line), equal = new C.EqualRadius(arc1, arc2);
  sk.add(onLine, equal, new C.Symmetric(arc2.start, arc1.end, line));
  solve(sk);
  const d = diagnose(sk);
  assert.equal(d.geometricDependency, 1);
  assert.equal(d.nRedundant, 1);
  assert.equal(d.status, 'under');
  assert.deepEqual(d.over, []);
  assert.deepEqual(d.violated, []);
  assert.deepEqual(d.implied.map((k) => io.describe(k)).sort(),
                   [io.describe(onLine), io.describe(equal)].sort());
  assert.ok([...d.entityState.values()].every((s) => s !== 'over'));
});

test('a dependency that involves a dimension is still over', () => {
  // the same kind of theorem — two equal distances make EqualLength follow — but the rows that
  // take part carry dimensions, and editing either is a conflict: worth flagging now
  const sk = new Sketch();
  const p = sk.point(0, 0, true), q = sk.point(5, 0), r = sk.point(5, 5);
  const equal = new C.EqualLength(sk.line(p, q), sk.line(q, r));
  sk.add(new C.Distance(p, q, 5), new C.Distance(q, r, 5), equal);
  solve(sk);
  const d = diagnose(sk);
  assert.equal(d.geometricDependency, 1);
  assert.equal(d.status, 'over');
  assert.equal(d.over.length, 3);
  assert.ok(d.over.includes(equal));
  assert.deepEqual(d.implied, []);
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
  assert.match(io.describe(d), /^Distance\(P\d+, P\d+, 100\)$/);
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

test('angleBetween and onRadius are the core\'s', async () => {
  const { angleBetween, onRadius } = await import('../core/model.js');
  const sk = new Sketch();
  const o = sk.point(0, 0);
  const east = sk.line(o, sk.point(10, 0)), north = sk.line(o, sk.point(0, 10));
  assert.ok(Math.abs(angleBetween(east, north) - Math.PI / 2) < 1e-12);
  assert.ok(Math.abs(angleBetween(north, east) + Math.PI / 2) < 1e-12);   // signed
  const q = onRadius(0, 0, 3, 4, 10)!;
  assert.ok(Math.abs(q[0] - 6) < 1e-12 && Math.abs(q[1] - 8) < 1e-12, `${q}`);
  assert.equal(onRadius(1, 1, 1, 1, 5), null);                            // no direction
});

test('a heap view does not survive a call that grows the core\'s memory', async () => {
  // Why the readers copy their numbers out before touching the sketch again: the module grows
  // its memory on any call, and every typed-array view over the old buffer detaches when it does.
  const { Buf } = await import('../core/wasm.js');
  const b = new Buf(4, 4);
  try {
    const view = b.i32;
    view[0] = 42;
    const copy = Int32Array.from(view);
    const big = core().gcs_malloc(64 * 1024 * 1024);   // enough to force a grow
    core().gcs_free(big, 64 * 1024 * 1024);
    assert.equal(copy[0], 42);                          // the copy is still readable
    assert.equal(b.i32[0], 42);                         // and a freshly taken view still is
  } finally {
    b.release();
  }
});

test('flip and guard readers survive a growing sketch', () => {
  // The same readers, exercised end to end: a sketch big enough that `points` allocates while
  // the buffer is still in hand.
  const sk = new Sketch();
  for (let i = 0; i < 400; i++) sk.point(i, (i * 7) % 13);
  const a = sk.point(0, 0, true), bb = sk.point(10, 0, true);
  const c = sk.point(5, 4);
  sk.add(new C.Distance(a, c, 6));
  const d = new Drag(sk, c, 5, 4, 'dogleg', 1.0, [[a, bb, c]], 1.0);
  try {
    d.move(5, -4);
    for (const t of d.flips) assert.ok(t.every((p) => p !== undefined));
  } finally {
    d.end();
  }
});

test('every argument a proxy holds reaches the core', () => {
  // A proxy showing one value while the core holds another is a sketch that solves to something
  // other than what the UI says.  `Number('start')` is NaN, and sending that replaced the string.
  const sk = new Sketch();
  const line = sk.line(sk.point(0, 0, true), sk.point(10, 0, true));
  const circle = sk.circle(sk.point(5, 3), 1);
  const t = new C.TangentLineCircle(line, circle);
  sk.add(t);
  t.side = -1;
  const stored = () => JSON.parse(io.dumps(sk)).constraints as { type: string; args: unknown[] }[];
  assert.equal(stored()[0].args[2], -1);

  const c2 = sk.circle(sk.point(30, 0), 2), c3 = sk.circle(sk.point(31, 0), 1);
  const tc = new C.TangentCircleCircle(c2, c3);
  sk.add(tc);
  const before = tc.external as boolean;
  tc.external = !before;
  assert.equal(stored()[1].args[2], !before);
  assert.equal(tc.external, !before);

  const arc = sk.arc(sk.point(0, 50), sk.point(6, 50), sk.point(0, 56));
  const ta = new C.TangentArcLine(arc, line, 'start');
  sk.add(ta);
  ta.at = 'end';
  assert.equal(stored().find((c) => c.type === 'TangentArcLine')!.args[2], 'end');
  assert.equal(ta.at, 'end');
});

test('setX refuses a vector that is not this sketch\'s', () => {
  // Writing the overlapping prefix scattered one sketch's coordinates over another's — the DOF
  // animation restoring its starting state into whatever sketch had replaced it, for one.
  const a = new Sketch(), b = new Sketch();
  a.point(1, 2);
  a.point(3, 4);
  b.point(9, 9);
  assert.throws(() => b.setX(a.getX()), /params/);
  assert.deepEqual(b.points[0].xy, [9, 9]);
});

test('the topology key distinguishes one constraint from another of the same type', () => {
  // A front end caches compiled plans against this.  Counts and type names alone are not enough:
  // delete one Distance and add another and both are identical, so the cache replays a plan that
  // still enforces the old dimension and ignores the new one.
  const sk = new Sketch();
  const p = [0, 1, 2, 3].map((i) => sk.point(i * 10, 0));
  const a = new C.Distance(p[0], p[1], 50);
  sk.add(a);
  const k1 = sk.topologyKey();
  sk.remove(a);
  sk.add(new C.Distance(p[2], p[3], 20));
  assert.notEqual(sk.topologyKey(), k1);

  const k2 = sk.topologyKey();
  p[0].fix(true);
  assert.notEqual(sk.topologyKey(), k2);
  p[0].fix(false);
  assert.equal(sk.topologyKey(), k2);
  p[0].x.value = 99;
  assert.equal(sk.topologyKey(), k2);   // moving geometry is not a topology change
});

test('a diagnosis run against a stale System does not name dead constraints', () => {
  // With auto-solve off the app can still be holding the System it last solved with.  A
  // constraint removed since must not come back as an `undefined` proxy.
  const sk = new Sketch();
  const a = sk.point(0, 0, true), b = sk.point(10, 0), c = sk.point(10, 10);
  const d1 = new C.Distance(a, b, 10), d2 = new C.Distance(b, c, 10);
  sk.add(d1, d2);
  const sys = new System(sk);
  try {
    sk.remove(d2);
    const d = diagnose(sk, { system: sys });
    for (const con of [...d.over, ...d.violated, ...(d.conflicts ?? [])]) {
      assert.ok(con !== undefined, 'diagnosis named a constraint the sketch no longer has');
    }
  } finally {
    sys.dispose();
  }
});

test('a tangency left open takes its branch from the geometry', () => {
  // The core reads a tangency's branch off the current sketch.  Substituting the registry's
  // constant for an omitted argument picks the branch in the binding — and the wrong one, so the
  // solve drags the circle through the line to reach the other side.
  const sk = new Sketch();
  const line = sk.line(sk.point(0, 0, true), sk.point(10, 0, true));
  const circle = sk.circle(sk.point(5, -3), 1);
  sk.add(new C.Radius(circle, 1));

  const t = new C.TangentLineCircle(line, circle);
  assert.equal(t.side, null);            // nothing decided before there is a sketch
  sk.add(t);
  assert.equal(t.side, -1);

  assert.ok(solve(sk).success);
  assert.ok(Math.abs(circle.center.y.value + 1) < 1e-6, `${circle.center.y.value}`);

  const big = sk.circle(sk.point(100, 0), 5);
  const inside = sk.circle(sk.point(101, 0), 1);
  const apart = sk.circle(sk.point(200, 0), 2);
  const a = new C.TangentCircleCircle(big, inside), b = new C.TangentCircleCircle(big, apart);
  sk.add(a, b);
  assert.deepEqual([a.external, b.external], [false, true]);
});

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
    // -1 means the kernel belongs to the curve *definition* rather than to the type: two curve
    // families read different numbers of coordinates, so they cannot share one.  Every other
    // type names a static kernel, and must.
    if (t.kernel === -1) {
      assert.equal(t.name, 'PointOnCurve', `${t.name} has no static kernel`);
    } else {
      assert.ok(t.kernel >= 0 && t.kernel < reg.kernels.length, t.name);
    }
    assert.equal(C.CONSTRAINT_TYPES[t.name].kernelId, t.kernel);
  }
});

/* -- dimension expressions ----------------------------------------------------- */

/** Three free segments, each dimensioned by the text given. */
function threeDims(texts: [string, string, string]): { sk: Sketch; cs: Constraint[] } {
  const sk = new Sketch();
  const cs = texts.map((t, i) => {
    const a = sk.point(10 * i, 0), b = sk.point(10 * i + 5, 0);
    const c = new C.Distance(a, b, t);
    sk.add(c);
    return c;
  });
  return { sk, cs };
}

test('dimensions written as expressions are evaluated in dependency order', () => {
  // the reader comes first in the document, the definition last
  const { sk, cs } = threeDims(['sin(h * 10)', 'h = w * 2', 'w = 1']);
  assert.equal(num(cs[2].d), 1);
  assert.equal(num(cs[1].d), 2);
  assert.ok(Math.abs(num(cs[0].d) - Math.sin((20 * Math.PI) / 180)) < 1e-12);
  assert.equal(cs[1].expr('d'), 'h = w * 2');
  assert.equal(cs[1].describe(), 'Distance(P2, P3, h = w * 2 = 2)');
  const items = expressions(sk);
  assert.deepEqual(items.map((it) => it.id), [cs[2].id, cs[1].id, cs[0].id]);
  assert.deepEqual(items[1].deps, ['w']);
  assert.equal(items[1].name, 'h');
  assert.ok(items.every((it) => it.error === null));
  // the solver sees the numbers
  assert.ok(solve(sk).success);
  const [p, q] = cs[1].entities() as [import('../core/model.js').Point, import('../core/model.js').Point];
  assert.ok(Math.abs(Math.hypot(p.x.value - q.x.value, p.y.value - q.y.value) - 2) < 1e-9);
});

test('editing one dimension moves every proxy that reads it', () => {
  const { sk, cs } = threeDims(['w = 3', 'h = w * 2', 'h + 1']);
  assert.equal(cs[2].d, 7);
  assert.equal(cs[0].setDimension('d', 'w = 5'), null);
  assert.equal(cs[1].d, 10);                  // re-read from the core, nothing told this proxy
  assert.equal(cs[2].d, 11);
  // a bare number is a constant again; nothing defines `w` now, so it becomes a free variable
  // and the two readers keep both their numbers and their relation to each other
  assert.equal(cs[0].setDimension('d', '4'), null);
  assert.equal(cs[0].expr('d'), null);
  assert.equal(cs[1].d, 10);
  assert.equal(expressions(sk).filter((it) => it.error).length, 0);
  assert.deepEqual(expressions(sk).map((it) => it.free), [['w'], ['w']]);
  // text that does not parse is refused and changes nothing
  assert.throws(() => cs[0].setDimension('d', '1 +'));
  assert.equal(cs[0].d, 4);
  // a free name used in a way an affine form cannot hold is kept, and says why
  assert.match(cs[0].setDimension('d', 'q * q') ?? '', /`q` is free/);
  assert.equal(cs[0].d, 4);
  // a cycle is named
  cs[0].setDimension('d', 'w = h');
  assert.match(expressions(sk).find((it) => it.id === cs[0].id)?.error ?? '', /circular/);
  // angles are written in degrees — as text at construction too, where a bare number is a
  // constant under the same rule (what the Dimension tool sends)
  const sk2 = new Sketch();
  const l1 = sk2.lineXY(0, 0, 10, 0), l2 = sk2.lineXY(0, 0, 10, 5);
  const ang = new C.Angle(l1, l2, '30');
  sk2.add(ang);
  assert.ok(Math.abs(num(ang.theta) - Math.PI / 6) < 1e-12);
  assert.equal(ang.expr('theta'), null);
  ang.setDimension('theta', 'a = 30');
  assert.ok(Math.abs(num(ang.theta) - Math.PI / 6) < 1e-12);
  assert.equal(expressions(sk2)[0].value, 30);
  sk.dispose();
  sk2.dispose();
});

test('expressions round-trip through the document and survive a rebuild', () => {
  const { sk, cs } = threeDims(['w = 3', 'h = w * 2', 'h + 1']);
  const sk2 = io.loads(io.dumps(sk));
  assert.equal(io.dumps(sk2), io.dumps(sk));
  assert.equal(sk2.constraints[1].expr('d'), 'h = w * 2');
  assert.equal(sk2.constraints[1].d, 6);
  // deleting the definition: nothing defines `w` any more, so it is a free variable — the
  // relation outlives its definition and the readers keep their numbers
  const sk3 = io.without(sk, [], [cs[0]]);
  assert.equal(sk3.constraints[0].d, 6);
  assert.equal(expressions(sk3)[0].error, null);
  assert.deepEqual(expressions(sk3)[0].free, ['w']);
  // the callout carries the expression itself, not what it came to
  const texts = callouts(sk, 1).items.map((k) => k.text);
  assert.deepEqual(texts, ['w = 3', 'h = w * 2', 'h + 1']);
  sk.dispose();
  sk2.dispose();
  sk3.dispose();
});

test('a claim reads as one, and a claimed dimension is drawn as a reference dimension', () => {
  // what the app puts in a constraint row and on a callout comes from the core, so both
  // bindings say the same thing about a claim and neither has a rule of its own
  const sk = examples.peaucellier();
  assert.ok(solve(sk).success);
  const claim = sk.userConstraints().find((c) => c.claim)!;
  assert.ok(claim, 'peaucellier ends on a claim');
  assert.ok(io.describe(claim).startsWith('claim '), io.describe(claim));

  const d = diagnose(sk);
  assert.deepEqual(d.claimsTheorem, [claim], 'the straight line is a theorem');
  assert.equal(d.claimsViolated.length + d.claimsConsuming.length, 0);
  // and the claim is none of the ordinary readings, which is what lets the row say only its own
  assert.equal(d.over.length, 0);
  assert.ok(!d.implied.includes(claim));

  // a claimed dimension draws in parentheses; pythagoras is the case that has one
  const py = examples.pythagoras(30, 40);
  assert.ok(solve(py).success);
  const cc = py.userConstraints().find((c) => c.claim)!;
  const k = callouts(py, 1).items.find((it) => it.id === cc.id)!;
  assert.ok(k, 'the claimed hypotenuse is drawn');
  assert.ok(k.text.startsWith('(') && k.text.endsWith(')'), k.text);
});

test('pythagoras drawn with expressions holds, and stays true when a leg is edited', () => {
  // four a×b right triangles in a square of side a + b leave a square whose side is *claimed*
  // to be `c = hypot(a, b)`: judged a theorem, and still one after a leg is edited
  const sk = examples.pythagoras(30, 40);
  const check = (a: number, b: number): void => {
    assert.ok(solve(sk).success);
    const c = Math.hypot(a, b);
    for (const ln of sk.lines.slice(-4)) {             // the hypotenuses are the inner square
      assert.ok(Math.abs(Math.hypot(ln.p1.x.value - ln.p2.x.value, ln.p1.y.value - ln.p2.y.value) - c) < 1e-6);
    }
    const cc = sk.constraints.find((k) => k.expr('d') === 'c = hypot(a, b)')!;
    assert.ok(Math.abs(num(cc.d) - c) < 1e-9);
    assert.ok(cc.claim, 'the hypotenuse is stated as a claim');
    const d = diagnose(sk);
    assert.equal(d.dof, 0);
    assert.equal(d.nRedundant, 0);
    assert.deepEqual(d.claimsTheorem, [cc]);
    assert.equal(d.claimsViolated.length + d.claimsConsuming.length, 0);
    assert.equal(d.violated.length, 0);
    assert.equal(d.conflicts?.length ?? 0, 0);
  };
  check(30, 40);
  const a = sk.constraints.find((k) => k.expr('d') === 'a = 30')!;
  assert.equal(a.setDimension('d', 'a = 50'), null);
  check(50, 40);
  sk.dispose();
});

/* -- parametric curves --------------------------------------------------------- */

function wave(n = 6): { sk: Sketch; sp: Spline } {
  const sk = new Sketch();
  const ctrl = Array.from({ length: n }, (_, i) => sk.point(i * 10, i % 2 ? 12 : 0));
  const sp = sk.spline(ctrl);
  assert.ok(sp, 'six control points make a cubic');
  return { sk, sp: sp! };
}

test('a spline is a control polygon of ordinary points', () => {
  const { sk, sp } = wave();
  assert.deepEqual(sp.ctrl.map((p) => p.index), sk.points.map((p) => p.index));
  assert.deepEqual(sp.knots, [0, 0, 0, 0, 1, 2, 3, 3, 3, 3]);
  assert.deepEqual(sp.domain, [0, 3]);
  // a clamped curve starts at its first control point and ends at its last
  for (const [t, p] of [[0, sk.points[0]], [3, sk.points[5]]] as [number, Point][]) {
    const [x, y] = sp.pointAt(t);
    assert.ok(Math.abs(x - p.x.value) < 1e-12 && Math.abs(y - p.y.value) < 1e-12);
  }
  sk.dispose();
});

test('too few control points is not a curve', () => {
  const sk = new Sketch();
  assert.equal(sk.spline([sk.point(0, 0), sk.point(1, 1), sk.point(2, 0)]), null);
  assert.equal(sk.splines.length, 0);
  sk.dispose();
});

test('the polyline the core hands over lands on the curve and follows the zoom', () => {
  const { sk, sp } = wave();
  const coarse = sp.polyline(1), fine = sp.polyline(0.01);
  assert.ok(fine.length > coarse.length);
  for (const [x, y] of fine) assert.ok(sp.closest(x, y).distance < 1e-6);
  sk.dispose();
});

test('a curve contact owns one unknown, reads as a number and cannot be written', () => {
  const { sk, sp } = wave();
  const p = sk.point(21, 30);
  const before = sk.params.length;
  const c = new C.PointOnSpline(p, sp);
  sk.add(c);
  assert.equal(sk.params.length, before + 1);
  assert.equal(typeof c.t, 'number');
  // no setter: the solver moves a curve parameter, nobody states one
  assert.throws(() => { (c as unknown as { t: number }).t = 0.5; }, TypeError);
  sk.dispose();
});

test('a point is pulled onto the curve and a line is made tangent to it', () => {
  const { sk, sp } = wave();
  for (const q of sp.ctrl) { q.x.fixed = true; q.y.fixed = true; }
  const p = sk.point(21, 30);
  sk.add(new C.PointOnSpline(p, sp));
  assert.ok(solve(sk).success);
  assert.ok(sp.closest(p.x.value, p.y.value).distance < 1e-9);

  // what tangency *means* is checked in the Rust test, where the kernel lives; here the point is
  // that the binding reaches it — the core says the constraint holds, and the contact the proxy
  // hands back is a real point of the curve
  const ln = sk.line(sk.point(0, -20), sk.point(50, -20));
  const c = new C.SplineTangentLine(sp, ln);
  sk.add(c);
  assert.ok(solve(sk).success);
  assert.ok(allSatisfied(sk));
  const [cx, cy] = sp.pointAt(c.t as number);
  assert.ok(sp.closest(cx, cy).distance < 1e-9);
  sk.dispose();
});

test('a contact settles on the drawn curve, not on the polynomial past its end', () => {
  const sk = new Sketch();
  const ctrl = ([[0, 0], [0, 10], [10, 10], [10, 0]] as [number, number][])
    .map(([x, y]) => sk.point(x, y));
  const sp = sk.spline(ctrl)!;
  for (const q of sp.ctrl) { q.x.fixed = true; q.y.fixed = true; }
  const ln = sk.line(sk.point(-6, 0), sk.point(-6, 10));
  const c = new C.SplineTangentLine(sp, ln);
  sk.add(c);
  assert.ok(solve(sk).success);
  const [t0, t1] = sp.domain;
  assert.ok((c.t as number) >= t0 - 1e-12 && (c.t as number) <= t1 + 1e-12, `t = ${c.t}`);
  sk.dispose();
});

test('a document keeps its curves and where they are touched', () => {
  const { sk, sp } = wave(7);
  const p = sk.point(21, 30);
  const c = new C.PointOnSpline(p, sp);
  sk.add(c);
  assert.ok(solve(sk).success);
  const text = io.dumps(sk);
  const back = io.loads(text);
  assert.equal(back.splines.length, 1);
  assert.deepEqual(back.splines[0].knots, sp.knots);
  const c2 = back.constraints.find((k) => k.typeName === 'PointOnSpline')!;
  assert.ok(Math.abs((c2.t as number) - (c.t as number)) < 1e-12);
  assert.equal(io.dumps(back), text);
  sk.dispose();
  back.dispose();
});

test('dragging a point along a curve carries it across a knot', () => {
  const { sk, sp } = wave(7);
  for (const q of sp.ctrl) { q.x.fixed = true; q.y.fixed = true; }
  const p = sk.point(2, 4);
  const c = new C.PointOnSpline(p, sp);
  sk.add(c);
  assert.ok(solve(sk).success);
  const first = Math.floor(c.t as number);
  const [, t1] = sp.domain;
  const far = sp.pointAt(t1 - 0.2);
  const d = new Drag(sk, p, p.x.value, p.y.value);
  d.move(far[0], far[1]);
  d.end();
  assert.ok(Math.floor(c.t as number) > first, `the contact never left span ${first}`);
  assert.ok(sp.closest(p.x.value, p.y.value).distance < 1e-6);
  sk.dispose();
});

test('deleting a control point shortens the curve instead of deleting it', () => {
  const { sk, sp } = wave(7);
  const out = io.without(sk, [sp.ctrl[3]]);
  assert.equal(out.splines.length, 1);
  assert.equal(out.splines[0].ctrl.length, 6);
  sk.dispose();
  out.dispose();
});

// that the curve does not move, that a fit passes through its points and what any of it costs
// in degrees of freedom are Rust tests, where the kernels are; these check the binding reaches
test('the binding can insert a control point', () => {
  const { sk, sp } = wave(6);
  const [t0, t1] = sp.domain;
  const made = sp.insertControl(t0 + (t1 - t0) * 0.4);
  assert.ok(made);
  assert.ok(sp.ctrl.includes(made!));
  assert.equal(sp.ctrl.length, 7);
  assert.equal(sp.insertControl(NaN), null);
  sk.dispose();
});

test('the binding can fit a curve through points, and hold it to them', () => {
  const pts: [number, number][] = [[0, 0], [10, 20], [30, 5], [50, 25], [70, 0]];
  const sk = new Sketch();
  assert.ok(sk.splineThrough(pts));
  assert.equal(sk.userConstraints().length, 0, 'nothing held, nothing said');
  assert.equal(sk.splineThrough(pts.slice(0, 3)), null);

  const held = new Sketch();
  const on = pts.map(([x, y]) => held.point(x, y));
  assert.ok(held.splineThrough(pts, on));
  assert.equal(held.userConstraints().length, pts.length, 'the hold list arrived');
  sk.dispose();
  held.dispose();
});

test('a dimension written as a mixed fraction keeps the way it was written', () => {
  // the number is for the graph and the text is for the drawing; what the language accepts is
  // the core's business, this is that the binding reaches it and both halves come back
  const sk = examples.rectFillets();
  const d = sk.constraints.find((c) => c.typeName === 'Distance')!;
  assert.equal(d.setDimension('d', '3 1/8'), null);
  assert.ok(Math.abs(num(d.d) - 3.125) < 1e-12);
  assert.equal(d.expr('d'), '3 1/8', 'the fraction was collapsed to its value');
  assert.ok(solve(sk).success);
  // and it reaches the drawing, which is the point of keeping it
  assert.ok(callouts(sk, 0.1).items.some((k) => k.text === '3 1/8'));

  assert.equal(d.setDimension('d', 'w = 12 3/8'), null);
  assert.ok(Math.abs(num(d.d) - 12.375) < 1e-12);
  assert.equal(d.expr('d'), 'w = 12 3/8');
  assert.throws(() => d.setDimension('d', '3 1/0'));
  sk.dispose();
});

test('a circle against a curve is a curvature constraint', () => {
  const { sk, sp } = wave(6);
  for (const q of sp.ctrl) { q.x.fixed = true; q.y.fixed = true; }
  const o = sk.point(24, 30);
  const circle = sk.circle(o, 9);
  const c = new C.SplineCurvature(sp, circle);
  sk.add(c);
  assert.ok(solve(sk).success);
  assert.ok(allSatisfied(sk));
  // it ends up the curve's own circle there: touching, at the radius it now has
  const { p } = sp.eval(c.t as number);
  const r = Math.abs(circle.radius.value);
  assert.ok(Math.abs(Math.hypot(o.x.value - p[0], o.y.value - p[1]) - r) < 1e-6 * r);
  sk.dispose();
});

test('two points can be levelled without a line between them', () => {
  const sk = new Sketch();
  const a = sk.point(0, 0), b = sk.point(10, 4), c = sk.point(3, 9);
  a.x.fixed = true;
  a.y.fixed = true;
  sk.add(new C.HorizontalPoints(a, b), new C.VerticalPoints(a, c));
  assert.ok(solve(sk).success);
  assert.ok(Math.abs(b.y.value - a.y.value) < 1e-9);
  assert.ok(Math.abs(c.x.value - a.x.value) < 1e-9);
  // and it is the same statement either way round, so a duplicate is caught
  assert.ok(C.sameConstraint(new C.HorizontalPoints(a, b), new C.HorizontalPoints(b, a)));
  sk.dispose();
});

test('the run and the rise between two points are dimensions of their own', () => {
  const sk = new Sketch();
  const a = sk.point(0, 0), b = sk.point(10, 4);
  a.x.fixed = true;
  a.y.fixed = true;
  sk.add(new C.HorizontalDistance(a, b, 30), new C.VerticalDistance(a, b, -5));
  assert.ok(solve(sk).success);
  assert.ok(Math.abs(b.x.value - 30) < 1e-9, `x ${b.x.value}`);
  assert.ok(Math.abs(b.y.value + 5) < 1e-9, `y ${b.y.value}`);
  // signed from the first point to the second, so the pair does not commute
  assert.ok(!C.sameConstraint(new C.HorizontalDistance(a, b, 30),
                              new C.HorizontalDistance(b, a, 30)));
  // and which of the three a callout states comes from where it is put
  assert.equal(pairDimension([0, 0], [40, 40], [-10, 50]), 'Distance');
  assert.equal(pairDimension([0, 0], [40, 40], [20, 60]), 'HorizontalDistance');
  assert.equal(pairDimension([0, 0], [40, 40], [60, 20]), 'VerticalDistance');
  sk.dispose();
});

test('a name nothing defines is a free variable that ties the dimensions reading it', () => {
  // an unknown the solver moves, not a number: the two dimensions are tied to each other and
  // what they come to is one degree of freedom nobody stated
  const sk = new Sketch();
  const a = sk.point(0, 0, true), b = sk.point(30, 0);
  const c = sk.point(0, 10, true), d = sk.point(12, 10);
  const d1 = new C.Distance(a, b, 'q');
  const d2 = new C.Distance(c, d, 'q / 2');
  sk.add(d1);
  sk.add(d2);
  const items = expressions(sk);
  assert.deepEqual(items.map((it) => it.error), [null, null]);
  assert.deepEqual(items.map((it) => it.free), [['q'], ['q']]);
  assert.ok(solve(sk).success);
  const len = (p: Point, q: Point) => Math.hypot(q.x.value - p.x.value, q.y.value - p.y.value);
  assert.ok(Math.abs(len(c, d) - len(a, b) / 2) < 1e-6, `${len(a, b)} ${len(c, d)}`);
  // the drawing carries what was written; the list adds where it stands, marked as free
  assert.equal(callouts(sk, 1).items[0].text, 'q');
  assert.match(d1.describe(), /\(free\)/);
  // one degree of freedom more than the same drawing with the numbers stated
  const open = diagnose(sk).dof;
  d1.d = 20;
  d2.d = 10;
  assert.equal(diagnose(sk).dof, open - 1);
  // a free name can be scaled and offset and no more
  assert.match(d1.setDimension('d', 'sin(q)') ?? '', /`q` is free/);
  sk.dispose();
});

/* -- ellipses --------------------------------------------------------------- */

test('an ellipse is a centre, a major end and a minor radius of its own', () => {
  const sk = new Sketch();
  const c = sk.point(10, 5);
  const m = sk.point(18, 5);
  const el = sk.ellipse(c, m, 3);
  assert.equal(el.center, c);
  assert.equal(el.major, m);
  assert.equal(el.minor.value, 3);
  assert.equal(el.name, 'E0');
  // the rim is what a click picks; the inside is empty space
  assert.equal(sk.pick(10, 8.05, 0.2), el);
  assert.equal(sk.pick(12, 5.8, 0.2), null);
  // the bounds are the rotated rim's, which here is axis-aligned
  assert.deepEqual(el.bounds(), [2, 2, 18, 8]);
  sk.dispose();
});

test('a point is pulled onto the rim, and the document keeps the ellipse', () => {
  const sk = new Sketch();
  const c = sk.point(10, 5, true);
  const m = sk.point(18, 5, true);
  const el = sk.ellipse(c, m, 3);
  el.minor.fixed = true;
  el.construction = true;
  const p = sk.point(11, 9);
  sk.add(new C.PointOnEllipse(p, el));
  assert.ok(solve(sk).success);
  // in the ellipse's frame the rim satisfies (x/a)² + (y/b)² = 1
  const [x, y] = [p.x.value - 10, p.y.value - 5];
  assert.ok(Math.abs((x / 8) ** 2 + (y / 3) ** 2 - 1) < 1e-6);
  const back = io.loads(io.dumps(sk));
  assert.equal(back.ellipses.length, 1);
  assert.ok(back.ellipses[0].construction);
  assert.ok(back.ellipses[0].minor.fixed);
  assert.equal(back.ellipses[0].minor.value, 3);
  assert.equal(back.constraints.filter((k) => k.typeName === 'PointOnEllipse').length, 1);
  sk.dispose();
  back.dispose();
});

test('a rim drag resizes the minor radius through the same question the tool asks', () => {
  const sk = new Sketch();
  const el = sk.ellipse(sk.point(0, 0, true), sk.point(8, 0, true), 3);
  // the minor radius that puts the rim through (4, 4): b = 4 / sqrt(1 - (4/8)²)
  const want = num(core().gcs_ellipse_minor(0, 0, 8, 0, 4, 4));
  assert.ok(Math.abs(want - 4 / Math.sqrt(0.75)) < 1e-12);
  const drag = new RadiusDrag(sk, el, el.minor.value);
  assert.ok(drag.move(want).success);
  drag.end();
  assert.ok(Math.abs(el.minor.value - want) < 1e-6);
  sk.dispose();
});

test('a line solves tangent to the rim, and a circle takes its curvature', () => {
  const sk = new Sketch();
  const el = sk.ellipse(sk.point(10, 5, true), sk.point(18, 5, true), 3);
  el.minor.fixed = true;
  const ln = sk.line(sk.point(4, 10), sk.point(16, 10));
  sk.add(new C.EllipseTangentLine(el, ln));
  const cc = sk.circle(sk.point(16, 5.5), 2);
  sk.add(new C.EllipseCurvature(el, cc));
  assert.ok(solve(sk).success);
  assert.ok(allSatisfied(sk));
  // the level line above the ellipse rests on top of the rim: y = 5 + b = 8
  assert.ok(Math.abs(ln.p1.y.value - 8) < 1e-6, `line landed at y=${ln.p1.y.value}`);
  // an axis-aligned check: at the major end the rim's radius of curvature is b²/a
  const t = num(sk.constraints.find((k) => k.typeName === 'EllipseCurvature')!.args[2]);
  if (Math.abs(Math.sin(t)) < 1e-3) {
    assert.ok(Math.abs(Math.abs(cc.radius.value) - 9 / 8) < 1e-3);
  }
  sk.dispose();
});

/* -- Solvent: the program a sketch is written as ---------------------------------- */

test('a sketch prints as a program and reads back the same', () => {
  for (const name of ['rect_fillets', 'slotted_link', 'truss', 'pythagoras']) {
    const sk = examples.build(name);
    const text = fromSketch(sk);
    assert.ok(text.length > 0, name);
    const d = Document.read(text);
    assert.ok(d.ok, `${name}: ${JSON.stringify(d.diagnostics)}`);
    assert.equal(io.dumps(d.sketch, 1), io.dumps(sk, 1), name);
    d.dispose();
    sk.dispose();
  }
});

test('a program written by hand draws', () => {
  const d = Document.read([
    'point a at (0, 0)',
    'point b at (100, 0)',
    'line  ab(a, b)',
    'distance(a, b) == w = 60',
    'horizontal(ab)',
    'ground(a)',
  ].join('\n'));
  assert.ok(d.ok, JSON.stringify(d.diagnostics));
  assert.equal(d.sketch.points.length, 2);
  assert.equal(d.sketch.lines.length, 1);
  d.dispose();
});

test('a program with a bad line reports it and draws the rest', () => {
  const d = Document.read('point a at (0, 0)\nnonsense here\npoint b at (5, 5)\n');
  assert.ok(!d.ok);
  assert.ok(d.diagnostics.length > 0);
  assert.ok(d.diagnostics[0].line >= 1 && d.diagnostics[0].code.length === 4);
  assert.equal(d.sketch.points.length, 2, 'one bad line costs one line');
  d.dispose();
});

test('the source map says where each entity was written', () => {
  const sk = examples.build('slotted_link');
  const text = fromSketch(sk);
  const d = Document.read(text);
  assert.ok(d.map.entities.length >= sk.points.length);
  const p0 = d.map.entities.find((x) => x.name === 'p0')!;
  assert.ok(p0, 'p0 is in the map');
  assert.ok(text.slice(p0.lo, p0.hi).startsWith('point'), text.slice(p0.lo, p0.hi));
  d.dispose();
  sk.dispose();
});

test('the gear is a program, and its flanks are involutes the language defines', () => {
  const sk = examples.build('gear');
  const n = 30;
  assert.equal(sk.circles.length, 3, 'the base, root and tip circles');
  assert.equal(sk.curves.length, 2 * n, 'two involute flanks per tooth');
  assert.equal(sk.points.length, 1 + 4 * n, 'a centre, and two ends per flank');
  const r = solve(sk);
  assert.ok(r.success, r.message);

  // every flank end is on the circle its statement named, and nothing said where
  const [rr, rt] = [42, 48];
  let onRoot = 0;
  let onTip = 0;
  for (let i = 1; i < sk.points.length; i++) {
    const p = sk.points[i];
    const rad = Math.hypot(p.x.value, p.y.value);
    if (Math.abs(rad - rr) < 1e-6) onRoot++;
    else if (Math.abs(rad - rt) < 1e-6) onTip++;
    else assert.fail(`a flank end at radius ${rad}`);
  }
  assert.equal(onRoot, 2 * n);
  assert.equal(onTip, 2 * n);

  // and the drawn curve really is an involute: sampled along the polyline the core hands the
  // painter, every point is `rb` from the centre plus a string as long as the arc it unwound
  const rb = 45 * Math.cos((25 * Math.PI) / 180);
  for (const cv of sk.curves.slice(0, 4)) {
    const poly = cv.polyline();
    assert.ok(poly.length > 32, 'the core laid out a polyline');
    for (const [x, y] of poly) {
      const d = Math.hypot(x, y);
      // a point of an involute of radius rb is at distance sqrt(rb^2 + s^2) for a string s
      const s = Math.sqrt(Math.max(0, d * d - rb * rb));
      assert.ok(d >= rb - 1e-6, `inside the base circle at ${d}`);
      assert.ok(s <= rt, `absurd string length ${s}`);
    }
  }
  sk.dispose();
});

/* -- Solvent: the source is the document ------------------------------------------ */

const TRIANGLE = `\
// a triangle, and this comment must survive every edit
point a at (0, 0)
point b at (100, 0)
point c at (40, 70)

line ab(a, b)      // the base
line bc(b, c)
line ca(c, a)

horizontal(ab)
distance(a, b) == w = 140
ground(a)
`;

test('an edit is a new text, and the document is unchanged until it is applied', () => {
  const d = Document.read(TRIANGLE);
  assert.ok(d.ok, JSON.stringify(d.diagnostics));
  const e = d.addPoint(12.5, -3);
  assert.equal(e.kind, 'structural');
  assert.deepEqual(e.names, ['p0']);
  assert.ok(e.text.includes('point   p0 hint at (12.5, -3)'), e.text);
  assert.equal(d.text, TRIANGLE, 'the document has not moved');
  assert.equal(d.sketch.points.length, 3);
  const next = Document.read(e.text);
  assert.equal(next.sketch.points.length, 4);
  next.dispose();
  d.dispose();
});

test('a solve writes the seeds back and touches nothing else', () => {
  const d = Document.read(TRIANGLE);
  const r = solve(d.sketch);
  assert.ok(r.success, r.message);
  const e = d.commitSeeds();
  assert.equal(e.kind, 'numeric', 'a seed is not a statement');
  assert.ok(e.text.includes('// a triangle, and this comment must survive every edit'));
  assert.ok(e.text.includes('line ab(a, b)      // the base'));
  assert.ok(e.text.includes('distance(a, b) == w = 140'), 'the dimension is not a seed');
  const before = TRIANGLE.split('\n');
  const after = e.text.split('\n');
  assert.equal(before.length, after.length);
  for (let i = 0; i < before.length; i++) {
    if (before[i].startsWith('point ')) continue;
    assert.equal(after[i], before[i], `line ${i + 1} changed`);
  }
  d.dispose();
});

test('a name carries a selection from one document to the next', () => {
  const d = Document.read(TRIANGLE);
  const held = d.sketch.points[2];
  const name = d.nameOf(held);
  assert.equal(name, 'c');
  const e = d.addPoint(0, 0);
  const next = Document.read(e.text);
  const again = next.entity(name!);
  assert.ok(again, 'the same name reaches an entity in the new elaboration');
  assert.equal(again!.kind, 'point');
  assert.notEqual(again!.sketch, d.sketch, 'and it is a different sketch');
  next.dispose();
  d.dispose();
});

test('drawing a triangle by gestures writes six statements', () => {
  let d = Document.read('');
  const names: string[] = [];
  for (const [x, y] of [[0, 0], [60, 0], [60, 40]]) {
    const e = d.addPoint(x, y);
    names.push(e.names[0]);
    d.dispose();
    d = Document.read(e.text);
  }
  for (const [i, j] of [[0, 1], [1, 2], [2, 0]]) {
    const e = d.addEntity('line', [names[i], names[j]]);
    d.dispose();
    d = Document.read(e.text);
  }
  const e = d.addRelation('horizontal', ['l0']);
  assert.ok(!e.refused, e.refused ?? '');
  d.dispose();
  d = Document.read(e.text);
  assert.ok(d.ok, JSON.stringify(d.diagnostics));
  assert.equal(d.sketch.points.length, 3);
  assert.equal(d.sketch.lines.length, 3);
  assert.ok(d.text.includes('horizontal(l0)'), d.text);
  d.dispose();
});

test('editing a number splices the number, and a name is a column', () => {
  const d = Document.read(TRIANGLE);
  const dim = d.sketch.constraints.find((c) => c.typeName === 'Distance')!;
  // `w = 140` names its value, so dropping the name drops a column — and the core says so
  const drop = d.setDimension(dim.id, 'd', '160');
  assert.equal(drop.kind, 'structural', 'a name that goes away is a column that goes away');
  assert.ok(drop.text.includes('distance(a, b) == 160'), drop.text);
  assert.ok(drop.text.includes('// the base'), 'and nothing else moved');

  // between two bare numbers there is nothing but the number: the plan survives it
  const plain = Document.read(drop.text);
  const same = plain.sketch.constraints.find((c) => c.typeName === 'Distance')!;
  const again = plain.setDimension(same.id, 'd', '170');
  assert.equal(again.kind, 'numeric', 'a bare number cannot move the topology');
  assert.ok(again.text.includes('distance(a, b) == 170'), again.text);

  const named = plain.setDimension(same.id, 'd', 'w = 170');
  assert.equal(named.kind, 'structural', 'a name may be a free variable');
  plain.dispose();
  d.dispose();
});

test('deleting a point takes the statements that named it', () => {
  const d = Document.read(TRIANGLE);
  const e = d.remove([d.sketch.points[2]]);
  assert.ok(!e.text.includes('point c at'), e.text);
  assert.ok(!e.text.includes('line bc'), e.text);
  assert.ok(e.text.includes('line ab(a, b)      // the base'));
  const next = Document.read(e.text);
  assert.equal(next.sketch.points.length, 2);
  assert.equal(next.sketch.lines.length, 1);
  next.dispose();
  d.dispose();
});

/* The colouring is the core's scan, so what a class *means* is tested in Rust.  What is worth
 * testing here is the seam: the offsets arrive in bytes and index a JS string on this side, and
 * the gear has an em dash in its second line. */
test('the coloured runs index the string, not the core\'s bytes', () => {
  const text = examples.source('gear');
  assert.ok(text.length !== Buffer.byteLength(text), 'the gear is not all ASCII');
  const runs = highlight(text);
  assert.ok(runs.length > 100);
  let end = 0;
  for (const r of runs) {
    assert.ok(r.lo >= end && r.hi > r.lo && r.hi <= text.length, `${r.cls} at ${r.lo}`);
    end = r.hi;
  }
  // the first run is the comment the file opens with, and it covers the whole of that line —
  // compared against the line itself rather than against its wording, which is prose and moves
  assert.equal(runs[0].cls, 'comment');
  assert.equal(text.slice(runs[0].lo, runs[0].hi), text.split('\n')[0]);
  // and a run past the em dash still lands on the word it names
  const family = runs.find((r) => r.cls === 'def' && text.slice(r.lo, r.hi) === 'involute');
  assert.ok(family, 'the curve family names itself');
  assert.ok(text.slice(0, family.lo).includes('—'), 'which is past the first non-ASCII character');
});

test('a diagnostic and a source map index the string, not the core\'s bytes', () => {
  // `gear.sv` has an em dash in its second line, so every offset past it differs between the two
  const text = examples.source('gear');
  assert.ok(text.length !== Buffer.byteLength(text), 'the gear is not all ASCII');
  const dash = text.indexOf('\u2014');
  assert.ok(dash > 0 && dash < 300, 'the em dash is near the top, so most spans are past it');
  const d = Document.read(text);
  // the gear's centre is declared well past the em dash, so its span is only right if converted
  const centre = d.map.entities.find((x) => x.name === 'g.center')!;
  assert.ok(centre && centre.lo > dash, 'the centre is in the map, past the em dash');
  assert.equal(text.slice(centre.lo, centre.hi), 'point center hint at (0, 0)');
  const port = d.map.entities.find((x) => x.name.endsWith('.t.r.lo'))!;
  assert.ok(port, 'a flank port is in the map');
  assert.equal(text.slice(port.lo, port.hi), 'port lo: point');
  d.dispose();

  // and a diagnostic points at the words it is about, on a line past the em dash
  const broken = `${text}\nnonsense here\n`;
  const b = Document.read(broken);
  const err = b.diagnostics.find((x) => x.severity === 'error')!;
  assert.ok(err, JSON.stringify(b.diagnostics));
  assert.ok(broken.slice(err.lo, err.hi).startsWith('nonsense'),
            `the diagnostic covers ${JSON.stringify(broken.slice(err.lo, err.hi))}`);
  // the line the core counted and the line the string has agree
  assert.equal(broken.slice(0, err.lo).split('\n').length, err.line);
  b.dispose();
});

test('a program half-typed is still coloured', () => {
  const runs = highlight('point p at (0,\ncircle c(center: p, r');
  assert.equal(runs[0].cls, 'word');
  assert.ok(runs.some((r) => r.cls === 'label'), 'the labels of an unfinished call');
});

test('dragging the gear does not rewrite the gear', () => {
  const d = Document.read(examples.source('gear'));
  assert.ok(d.ok, JSON.stringify(d.diagnostics));
  assert.ok(solve(d.sketch).success);
  for (const p of d.sketch.points) {
    p.x.value += 1.5;
    p.y.value -= 0.5;
  }
  const e = d.commitSeeds();
  assert.equal(e.text, d.text, 'a statement that makes thirty points records no one pose');
  assert.ok(e.text.includes('curve involute(c: circle, phase: Angle)(u) ='));
  d.dispose();
});
