"""Reference sketches used by tests, benchmarks and the canvas."""

from __future__ import annotations

import math
import random
from collections.abc import Callable

import numpy as np

from gcs.constraints import (
    Coincident, Distance, EqualLength, EqualRadius, Horizontal, Parallel, Perpendicular, PointOnLine,
    Radius, TangentArcLine, Vertical,
)
from gcs.model import Point, Sketch


def perturb(sk: Sketch, sigma: float, seed: int = 0) -> None:
    """Add seeded Gaussian noise to every free parameter (tests/benchmarks warm-start from here)."""
    sk.perturb(sigma, seed)


def rect_fillets(w: float = 100.0, h: float = 60.0, r: float = 10.0, perturb: float = 0.0) -> Sketch:
    """Rectangle w×h with four equal fillets of radius r. Fully constrained (0 DOF)."""
    sk = Sketch()
    rng = random.Random(0)

    def P(x: float, y: float, name: str) -> Point:  # noqa: N802
        return sk.point(x + rng.uniform(-perturb, perturb), y + rng.uniform(-perturb, perturb), name=name)

    bottom = sk.line(P(r, 0, "b1"), P(w - r, 0, "b2"))
    right = sk.line(P(w, r, "r1"), P(w, h - r, "r2"))
    top = sk.line(P(w - r, h, "t1"), P(r, h, "t2"))
    left = sk.line(P(0, h - r, "l1"), P(0, r, "l2"))
    # arcs share endpoints with the lines (CCW start -> end)
    a_br = sk.arc(P(w - r, r, "c_br"), bottom.p2, right.p1, name="a_br")
    a_tr = sk.arc(P(w - r, h - r, "c_tr"), right.p2, top.p1, name="a_tr")
    a_tl = sk.arc(P(r, h - r, "c_tl"), top.p2, left.p1, name="a_tl")
    a_bl = sk.arc(P(r, r, "c_bl"), left.p2, bottom.p1, name="a_bl")

    sk.add(Horizontal(bottom), Horizontal(top), Vertical(left), Vertical(right))
    for arc, l_in, l_out in ((a_br, bottom, right), (a_tr, right, top), (a_tl, top, left), (a_bl, left, bottom)):
        sk.add(TangentArcLine(arc, l_in, "start"), TangentArcLine(arc, l_out, "end"))
    sk.add(EqualRadius(a_br, a_tr), EqualRadius(a_br, a_tl), EqualRadius(a_br, a_bl))
    sk.add(Radius(a_bl, r))
    sk.add(Distance(bottom.p1, bottom.p2, w - 2 * r), Distance(left.p1, left.p2, h - 2 * r))
    a_bl.center.fix()
    return sk


def slotted_link(length: float = 80.0, r: float = 15.0, hole_r: float = 6.0) -> Sketch:
    """Obround slot with two concentric holes. Fully constrained (0 DOF)."""
    sk = Sketch()
    c1 = sk.point(0, 0, name="c1")
    c2 = sk.point(length, 0, name="c2")
    top = sk.line(sk.point(0, r, name="t1"), sk.point(length, r, name="t2"))
    bottom = sk.line(sk.point(length, -r, name="b1"), sk.point(0, -r, name="b2"))
    a_right = sk.arc(c2, bottom.p1, top.p2, name="a_r")
    a_left = sk.arc(c1, top.p1, bottom.p2, name="a_l")
    h1 = sk.circle(c1, hole_r, name="h1")
    h2 = sk.circle(c2, hole_r, name="h2")
    sk.add(
        TangentArcLine(a_right, bottom, "start"), TangentArcLine(a_right, top, "end"),
        TangentArcLine(a_left, top, "start"), TangentArcLine(a_left, bottom, "end"),
        EqualRadius(a_left, a_right), Radius(a_left, r),
        Radius(h1, hole_r), Radius(h2, hole_r),
        Distance(c1, c2, length), Horizontal(top),
    )
    c1.fix()
    return sk


def truss(bays: int = 8, span: float = 20.0, height: float = 15.0, dims: bool = True) -> Sketch:
    """Warren-style truss: bays+1 bottom nodes, bays top nodes, ~4·bays members.

    With dims=True every member gets a length constraint → rigid, 0 DOF after
    fixing the first node and making the first chord horizontal.  bays=8 gives
    17 points + 31 lines (the "~30-entity" exit criterion).
    """
    sk = Sketch()
    bot = [sk.point(i * span, 0, name=f"b{i}") for i in range(bays + 1)]
    top = [sk.point((i + 0.5) * span, height, name=f"t{i}") for i in range(bays)]
    members = []
    for i in range(bays):
        members.append(sk.line(bot[i], bot[i + 1]))
        members.append(sk.line(bot[i], top[i]))
        members.append(sk.line(top[i], bot[i + 1]))
        if i + 1 < bays:
            members.append(sk.line(top[i], top[i + 1]))
    if dims:
        for m in members:
            sk.add(Distance(m.p1, m.p2, m.length()))
    sk.add(Horizontal(members[0]))
    bot[0].fix()
    return sk


def polygon_chain(n: int = 12, radius: float = 50.0) -> Sketch:
    """Under-constrained: closed n-gon of equal-length edges via Coincident joints.

    Deliberately closes the EqualLength cycle (e0=e1, ..., e_{n-1}=e0), so one
    equation is redundant-but-consistent — a Stage 2 diagnosis test case."""
    sk = Sketch()
    lines = []
    for i in range(n):
        a0, a1 = 2 * math.pi * i / n, 2 * math.pi * (i + 1) / n
        lines.append(sk.line_xy(radius * math.cos(a0), radius * math.sin(a0),
                                radius * math.cos(a1), radius * math.sin(a1), name=f"e{i}"))
    for i in range(n):
        sk.add(Coincident(lines[i].p2, lines[(i + 1) % n].p1))
        sk.add(EqualLength(lines[i], lines[(i + 1) % n]))
    lines[0].p1.fix()
    return sk


def henneberg_edges(n: int, rng: random.Random) -> list[tuple[int, int]]:
    """Random Laman graph on n >= 2 vertices by Henneberg I (add vertex + 2 edges) and
    II (subdivide an edge and connect to a third vertex) moves — minimally rigid by construction."""
    edges = [(0, 1)]
    for v in range(2, n):
        if v == 2 or rng.random() < 0.6:  # type I
            a, b = rng.sample(range(v), 2)
            edges += [(v, a), (v, b)]
        else:  # type II
            i = rng.randrange(len(edges))
            a, b = edges.pop(i)
            c = rng.choice([w for w in range(v) if w not in (a, b)])
            edges += [(v, a), (v, b), (v, c)]
    return edges


def laman(n: int = 10, seed: int = 0, ground: bool = True) -> Sketch:
    """Random minimally rigid framework (Henneberg construction) with a horizontal member and a
    fixed node — fully constrained; Henneberg-II moves make some of them non-tree-decomposable."""
    rng = random.Random(seed)
    sk = Sketch()
    pts = [sk.point(rng.uniform(0, 60), rng.uniform(0, 60), name=f"n{i}") for i in range(n)]
    for a, b in henneberg_edges(n, rng):
        sk.add(Distance(pts[a], pts[b], math.dist(pts[a].xy, pts[b].xy)))
    if ground:
        pts[0].fix()
        sk.add(Horizontal(sk.line(pts[0], pts[1])))
    return sk


def k33(seed: int = 3) -> Sketch:
    """K3,3 bar framework: minimally rigid but triangle-free — no pair/triple cluster merge
    applies, the decomposition must isolate it as one core."""
    rng = random.Random(seed)
    sk = Sketch()
    pts = [sk.point(rng.uniform(0, 40), rng.uniform(0, 40), name=f"k{i}") for i in range(6)]
    pts[0].fix()
    for a in range(3):
        for b in range(3, 6):
            sk.add(Distance(pts[a], pts[b], math.dist(pts[a].xy, pts[b].xy)))
    sk.add(Horizontal(sk.line(pts[0], pts[3])))
    return sk


def rect_fillets_conflict() -> Sketch:
    """Fillet rectangle with a second, contradicting width dimension (80 vs 50)."""
    sk = rect_fillets()
    sk.add(Distance(sk.lines[0].p1, sk.lines[0].p2, 50))
    return sk


def rect_fillets_under() -> Sketch:
    """Fillet rectangle without its width dimension: the right side slides (1 DOF)."""
    sk = rect_fillets()
    sk.remove(next(c for c in sk.constraints if isinstance(c, Distance) and c.d == 80))
    return sk


def truss_redundant() -> Sketch:
    """Truss with an extra, consistent member: structurally over-constrained but satisfiable."""
    sk = truss(6)
    p, q = sk.points[0], sk.points[2]
    sk.add(Distance(p, q, math.dist(p.xy, q.xy)))
    return sk


def truss_conflict() -> Sketch:
    """Truss with an impossible member length (999 between nearby nodes)."""
    sk = truss(6)
    sk.add(Distance(sk.points[0], sk.points[3], 999))
    return sk


def truss_floating(bays: int = 8) -> Sketch:
    """Rigid truss with nothing fixed: a free rigid body (3 DOF) — drag it around."""
    sk = truss(bays)
    for prm in sk.params:
        prm.fixed = False
    sk.constraints = [c for c in sk.constraints if not isinstance(c, Horizontal)]
    return sk


def impossible_triangle() -> Sketch:
    """Structurally fine, geometrically impossible: sides 10, 1, 1 (triangle inequality)."""
    sk = Sketch()
    a, b, c = sk.point(0, 0, fixed=True, name="a"), sk.point(10, 0, name="b"), sk.point(5, 5, name="c")
    sk.add(Distance(a, b, 10), Distance(b, c, 1), Distance(a, c, 1), Horizontal(sk.line(a, b)))
    return sk


def altitudes() -> Sketch:
    """Fixed triangle, three altitudes and a point on all three: structurally the third incidence
    looks independent, but the altitudes concur — a theorem-type dependency only the witness sees."""
    sk = Sketch()
    A, B, Cc = sk.point(0, 0, fixed=True, name="A"), sk.point(40, 0, fixed=True, name="B"), sk.point(15, 30, fixed=True, name="C")
    ab, bc, ca = sk.line(A, B), sk.line(B, Cc), sk.line(Cc, A)
    QA, QB, QC = sk.point(15, 5, name="QA"), sk.point(20, 10, name="QB"), sk.point(15, -5, name="QC")
    altA, altB, altC = sk.line(A, QA), sk.line(B, QB), sk.line(Cc, QC)
    sk.add(Perpendicular(altA, bc), Perpendicular(altB, ca), Perpendicular(altC, ab))
    P = sk.point(15, 8, name="P")
    sk.add(PointOnLine(P, altA), PointOnLine(P, altB), PointOnLine(P, altC))
    return sk


def parallels() -> Sketch:
    """Parallel / perpendicular / vertical lines with a few distances — exercises direction classes."""
    sk = Sketch()
    base = sk.line(sk.point(0, 0, fixed=True, name="o"), sk.point(40, 0, fixed=True, name="e"))
    l2 = sk.line(sk.point(0, 15, name="a"), sk.point(40, 15, name="b"))
    l3 = sk.line(sk.point(10, 15, name="c"), sk.point(10, 35, name="d"))
    l4 = sk.line(sk.point(10, 35, name="f"), sk.point(30, 30, name="g"))
    sk.add(Parallel(base, l2), Distance(base.p1, l2.p1, 15), Vertical(l3), Coincident(l3.p1, l2.p1),
           Distance(l3.p1, l3.p2, 20), Distance(l2.p1, l2.p2, 40), Perpendicular(l3, l4),
           Coincident(l4.p1, l3.p2), Distance(l4.p1, l4.p2, 20))
    return sk


EXAMPLES: dict[str, Callable[..., Sketch]] = {
    "rect_fillets": rect_fillets,
    "slotted_link": slotted_link,
    "truss": truss,
    "polygon_chain": polygon_chain,
}

# The case library shown in the app's dropdown: name → (factory, one-line description).
CASES: dict[str, tuple[Callable[[], Sketch], str]] = {
    "Rectangle with fillets": (rect_fillets, "fully constrained; tangent arcs, equal radii, two dimensions"),
    "Slotted link": (slotted_link, "obround slot with two holes; fully constrained"),
    "Truss (8 bays)": (lambda: truss(8), "~30-entity Warren truss, every member dimensioned"),
    "Truss (50 bays)": (lambda: truss(50), "300 entities — drag a node"),
    "Truss (200 bays)": (lambda: truss(200), "1200 entities — solver/plan timing"),
    "Truss, floating": (lambda: truss_floating(8), "rigid body with nothing fixed: 3 DOF, drag it around"),
    "Polygon chain (12)": (lambda: polygon_chain(12), "under-constrained equal-length ring; the EqualLength cycle is a redundancy the graph can't see"),
    "Rect, missing width": (rect_fillets_under, "under-constrained: the right side slides (null-space colouring)"),
    "Rect, conflicting width": (rect_fillets_conflict, "conflict: two contradicting width dimensions"),
    "Truss, redundant member": (truss_redundant, "structurally over-constrained but consistent (amber)"),
    "Truss, impossible member": (truss_conflict, "conflict: a 999-long member; minimal conflict set is a path + it"),
    "Impossible triangle": (impossible_triangle, "structurally fine, geometrically impossible (triangle inequality)"),
    "K3,3 framework": (k33, "rigid but triangle-free: decomposition needs a core merge"),
    "Random Laman #0": (lambda: laman(10, 0), "Henneberg-built minimally rigid framework"),
    "Random Laman #1": (lambda: laman(12, 1), "Henneberg-built; may need a core (Henneberg II)"),
    "Random Laman #7": (lambda: laman(11, 507), "needs a 9-cluster core"),
    "Concurrent altitudes": (altitudes, "theorem-type dependency: the third incidence is implied (Diagnose → witness); 3 DOF to animate"),
    "Parallels & perpendiculars": (parallels, "direction classes: parallel/perpendicular/vertical (1 DOF left: slide along the base)"),
}
