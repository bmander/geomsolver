/* Reference sketches used by the tests, the benchmarks and the app's case library. */
import {
  Coincident, Distance, EqualLength, EqualRadius, Horizontal, Parallel, Perpendicular,
  PointOnLine, Radius, TangentArcLine, Vertical,
} from './constraints.js';
import { Point, Sketch } from './model.js';
import { Rng } from './rng.js';

/** Rectangle w x h with four equal fillets of radius r.  Fully constrained (0 DOF). */
export function rectFillets(w = 100, h = 60, r = 10, jitter = 0): Sketch {
  const sk = new Sketch();
  const rng = new Rng(0);
  const P = (x: number, y: number, name: string): Point =>
    sk.point(x + rng.uniform(-jitter, jitter), y + rng.uniform(-jitter, jitter), false, name);

  const bottom = sk.line(P(r, 0, 'b1'), P(w - r, 0, 'b2'));
  const right = sk.line(P(w, r, 'r1'), P(w, h - r, 'r2'));
  const top = sk.line(P(w - r, h, 't1'), P(r, h, 't2'));
  const left = sk.line(P(0, h - r, 'l1'), P(0, r, 'l2'));
  // arcs share endpoints with the lines (CCW start -> end)
  const aBr = sk.arc(P(w - r, r, 'c_br'), bottom.p2, right.p1, 'a_br');
  const aTr = sk.arc(P(w - r, h - r, 'c_tr'), right.p2, top.p1, 'a_tr');
  const aTl = sk.arc(P(r, h - r, 'c_tl'), top.p2, left.p1, 'a_tl');
  const aBl = sk.arc(P(r, r, 'c_bl'), left.p2, bottom.p1, 'a_bl');

  sk.add(new Horizontal(bottom), new Horizontal(top), new Vertical(left), new Vertical(right));
  for (const [arc, lIn, lOut] of [[aBr, bottom, right], [aTr, right, top], [aTl, top, left], [aBl, left, bottom]] as const) {
    sk.add(new TangentArcLine(arc, lIn, 'start'), new TangentArcLine(arc, lOut, 'end'));
  }
  sk.add(new EqualRadius(aBr, aTr), new EqualRadius(aBr, aTl), new EqualRadius(aBr, aBl));
  sk.add(new Radius(aBl, r));
  sk.add(new Distance(bottom.p1, bottom.p2, w - 2 * r), new Distance(left.p1, left.p2, h - 2 * r));
  aBl.center.fix();
  return sk;
}

/** Obround slot with two concentric holes.  Fully constrained (0 DOF). */
export function slottedLink(length = 80, r = 15, holeR = 6): Sketch {
  const sk = new Sketch();
  const c1 = sk.point(0, 0, false, 'c1');
  const c2 = sk.point(length, 0, false, 'c2');
  const top = sk.line(sk.point(0, r, false, 't1'), sk.point(length, r, false, 't2'));
  const bottom = sk.line(sk.point(length, -r, false, 'b1'), sk.point(0, -r, false, 'b2'));
  const aRight = sk.arc(c2, bottom.p1, top.p2, 'a_r');
  const aLeft = sk.arc(c1, top.p1, bottom.p2, 'a_l');
  const h1 = sk.circle(c1, holeR, 'h1');
  const h2 = sk.circle(c2, holeR, 'h2');
  sk.add(
    new TangentArcLine(aRight, bottom, 'start'), new TangentArcLine(aRight, top, 'end'),
    new TangentArcLine(aLeft, top, 'start'), new TangentArcLine(aLeft, bottom, 'end'),
    new EqualRadius(aLeft, aRight), new Radius(aLeft, r),
    new Radius(h1, holeR), new Radius(h2, holeR),
    new Distance(c1, c2, length), new Horizontal(top),
  );
  c1.fix();
  return sk;
}

/** Warren-style truss: bays+1 bottom nodes, bays top nodes, ~4*bays members.  With dims every
 *  member gets a length constraint -> rigid, 0 DOF after fixing the first node and making the
 *  first chord horizontal.  bays = 8 gives 17 points + 31 lines. */
export function truss(bays = 8, span = 20, height = 15, dims = true): Sketch {
  const sk = new Sketch();
  const bot = Array.from({ length: bays + 1 }, (_, i) => sk.point(i * span, 0, false, `b${i}`));
  const top = Array.from({ length: bays }, (_, i) => sk.point((i + 0.5) * span, height, false, `t${i}`));
  const members = [];
  for (let i = 0; i < bays; i++) {
    members.push(sk.line(bot[i], bot[i + 1]));
    members.push(sk.line(bot[i], top[i]));
    members.push(sk.line(top[i], bot[i + 1]));
    if (i + 1 < bays) members.push(sk.line(top[i], top[i + 1]));
  }
  if (dims) for (const m of members) sk.add(new Distance(m.p1, m.p2, m.length()));
  sk.add(new Horizontal(members[0]));
  bot[0].fix();
  return sk;
}

/** Under-constrained: a closed n-gon of equal-length edges via Coincident joints.  The
 *  EqualLength cycle is deliberately closed, so one equation is redundant-but-consistent. */
export function polygonChain(n = 12, radius = 50): Sketch {
  const sk = new Sketch();
  const lines = [];
  for (let i = 0; i < n; i++) {
    const a0 = (2 * Math.PI * i) / n, a1 = (2 * Math.PI * (i + 1)) / n;
    lines.push(sk.lineXY(radius * Math.cos(a0), radius * Math.sin(a0),
                         radius * Math.cos(a1), radius * Math.sin(a1), `e${i}`));
  }
  for (let i = 0; i < n; i++) {
    sk.add(new Coincident(lines[i].p2, lines[(i + 1) % n].p1));
    sk.add(new EqualLength(lines[i], lines[(i + 1) % n]));
  }
  lines[0].p1.fix();
  return sk;
}

/** Random Laman graph on n >= 2 vertices by Henneberg I (add a vertex and 2 edges) and II
 *  (subdivide an edge and connect to a third vertex) moves — minimally rigid by construction. */
export function hennebergEdges(n: number, rng: Rng): [number, number][] {
  const edges: [number, number][] = [[0, 1]];
  for (let v = 2; v < n; v++) {
    if (v === 2 || rng.next() < 0.6) {                    // type I
      const [a, b] = rng.sample(v, 2);
      edges.push([v, a], [v, b]);
    } else {                                              // type II
      const i = rng.int(edges.length);
      const [a, b] = edges.splice(i, 1)[0];
      const cands = Array.from({ length: v }, (_, w) => w).filter((w) => w !== a && w !== b);
      const c = rng.choice(cands);
      edges.push([v, a], [v, b], [v, c]);
    }
  }
  return edges;
}

/** Random minimally rigid framework with a horizontal member and a fixed node — fully
 *  constrained; Henneberg-II moves make some of them non-tree-decomposable. */
export function laman(n = 10, seed = 0, ground = true): Sketch {
  const rng = new Rng(seed);
  const sk = new Sketch();
  const pts = Array.from({ length: n }, (_, i) => sk.point(rng.uniform(0, 60), rng.uniform(0, 60), false, `n${i}`));
  for (const [a, b] of hennebergEdges(n, rng)) {
    sk.add(new Distance(pts[a], pts[b], Math.hypot(pts[a].x.value - pts[b].x.value, pts[a].y.value - pts[b].y.value)));
  }
  if (ground) {
    pts[0].fix();
    sk.add(new Horizontal(sk.line(pts[0], pts[1])));
  }
  return sk;
}

/** K3,3 bar framework: minimally rigid but triangle-free — no pair/triple cluster merge
 *  applies, so the decomposition must isolate it as one core. */
export function k33(seed = 3): Sketch {
  const rng = new Rng(seed);
  const sk = new Sketch();
  const pts = Array.from({ length: 6 }, (_, i) => sk.point(rng.uniform(0, 40), rng.uniform(0, 40), false, `k${i}`));
  pts[0].fix();
  for (let a = 0; a < 3; a++) {
    for (let b = 3; b < 6; b++) {
      sk.add(new Distance(pts[a], pts[b],
        Math.hypot(pts[a].x.value - pts[b].x.value, pts[a].y.value - pts[b].y.value)));
    }
  }
  sk.add(new Horizontal(sk.line(pts[0], pts[3])));
  return sk;
}

/** Fillet rectangle with a second, contradicting width dimension (80 vs 50). */
export function rectFilletsConflict(): Sketch {
  const sk = rectFillets();
  sk.add(new Distance(sk.lines[0].p1, sk.lines[0].p2, 50));
  return sk;
}

/** Fillet rectangle without its width dimension: the right side slides (1 DOF). */
export function rectFilletsUnder(): Sketch {
  const sk = rectFillets();
  const c = sk.constraints.find((k) => k instanceof Distance && k.d === 80);
  if (c) sk.remove(c);
  return sk;
}

/** Truss with an extra, consistent member: structurally over-constrained but satisfiable. */
export function trussRedundant(): Sketch {
  const sk = truss(6);
  const p = sk.points[0], q = sk.points[2];
  sk.add(new Distance(p, q, Math.hypot(p.x.value - q.x.value, p.y.value - q.y.value)));
  return sk;
}

/** Truss with an impossible member length (999 between nearby nodes). */
export function trussConflict(): Sketch {
  const sk = truss(6);
  sk.add(new Distance(sk.points[0], sk.points[3], 999));
  return sk;
}

/** Rigid truss with nothing fixed: a free rigid body (3 DOF) — drag it around. */
export function trussFloating(bays = 8): Sketch {
  const sk = truss(bays);
  for (const p of sk.params) p.fixed = false;
  sk.constraints = sk.constraints.filter((c) => !(c instanceof Horizontal));
  return sk;
}

/** Structurally fine, geometrically impossible: sides 10, 1, 1 (the triangle inequality). */
export function impossibleTriangle(): Sketch {
  const sk = new Sketch();
  const a = sk.point(0, 0, true, 'a'), b = sk.point(10, 0, false, 'b'), c = sk.point(5, 5, false, 'c');
  sk.add(new Distance(a, b, 10), new Distance(b, c, 1), new Distance(a, c, 1), new Horizontal(sk.line(a, b)));
  return sk;
}

/** Fixed triangle, three altitudes and a point on all three: structurally the third incidence
 *  looks independent, but the altitudes concur — a theorem-type dependency only the witness
 *  configuration method sees. */
export function altitudes(): Sketch {
  const sk = new Sketch();
  const A = sk.point(0, 0, true, 'A'), B = sk.point(40, 0, true, 'B'), C = sk.point(15, 30, true, 'C');
  const ab = sk.line(A, B), bc = sk.line(B, C), ca = sk.line(C, A);
  const QA = sk.point(15, 5, false, 'QA'), QB = sk.point(20, 10, false, 'QB'), QC = sk.point(15, -5, false, 'QC');
  const altA = sk.line(A, QA), altB = sk.line(B, QB), altC = sk.line(C, QC);
  sk.add(new Perpendicular(altA, bc), new Perpendicular(altB, ca), new Perpendicular(altC, ab));
  const P = sk.point(15, 8, false, 'P');
  sk.add(new PointOnLine(P, altA), new PointOnLine(P, altB), new PointOnLine(P, altC));
  return sk;
}

/** Parallel / perpendicular / vertical lines with a few distances — exercises direction classes. */
export function parallels(): Sketch {
  const sk = new Sketch();
  const base = sk.line(sk.point(0, 0, true, 'o'), sk.point(40, 0, true, 'e'));
  const l2 = sk.line(sk.point(0, 15, false, 'a'), sk.point(40, 15, false, 'b'));
  const l3 = sk.line(sk.point(10, 15, false, 'c'), sk.point(10, 35, false, 'd'));
  const l4 = sk.line(sk.point(10, 35, false, 'f'), sk.point(30, 30, false, 'g'));
  sk.add(
    new Parallel(base, l2), new Distance(base.p1, l2.p1, 15), new Vertical(l3), new Coincident(l3.p1, l2.p1),
    new Distance(l3.p1, l3.p2, 20), new Distance(l2.p1, l2.p2, 40), new Perpendicular(l3, l4),
    new Coincident(l4.p1, l3.p2), new Distance(l4.p1, l4.p2, 20),
  );
  return sk;
}

export const EXAMPLES: Record<string, () => Sketch> = {
  rect_fillets: () => rectFillets(),
  slotted_link: () => slottedLink(),
  truss: () => truss(),
  polygon_chain: () => polygonChain(),
};

/** The case library shown in the app: name -> (factory, one-line description). */
export const CASES: [string, () => Sketch, string][] = [
  ['Rectangle with fillets', () => rectFillets(), 'fully constrained; tangent arcs, equal radii, two dimensions'],
  ['Slotted link', () => slottedLink(), 'obround slot with two holes; fully constrained'],
  ['Truss (8 bays)', () => truss(8), '~30-entity Warren truss, every member dimensioned'],
  ['Truss (50 bays)', () => truss(50), '300 entities — drag a node'],
  ['Truss (200 bays)', () => truss(200), '1200 entities — solver/plan timing'],
  ['Truss, floating', () => trussFloating(8), 'rigid body with nothing fixed: 3 DOF, drag it around'],
  ['Polygon chain (12)', () => polygonChain(12), "under-constrained equal-length ring; the EqualLength cycle is a redundancy the graph can't see"],
  ['Rect, missing width', rectFilletsUnder, 'under-constrained: the right side slides (null-space colouring)'],
  ['Rect, conflicting width', rectFilletsConflict, 'conflict: two contradicting width dimensions'],
  ['Truss, redundant member', trussRedundant, 'structurally over-constrained but consistent (amber)'],
  ['Truss, impossible member', trussConflict, 'conflict: a 999-long member; the minimal conflict set is a path plus it'],
  ['Impossible triangle', impossibleTriangle, 'structurally fine, geometrically impossible (triangle inequality)'],
  ['K3,3 framework', () => k33(), 'rigid but triangle-free: the decomposition needs a core merge'],
  ['Random Laman #0', () => laman(10, 0), 'Henneberg-built minimally rigid framework'],
  ['Random Laman #1', () => laman(12, 1), 'Henneberg-built; may need a core (Henneberg II)'],
  ['Concurrent altitudes', altitudes, 'theorem-type dependency: the third incidence is implied (Diagnose -> witness); 3 DOF to animate'],
  ['Parallels & perpendiculars', parallels, 'direction classes: parallel/perpendicular/vertical (1 DOF left: slide along the base)'],
];
