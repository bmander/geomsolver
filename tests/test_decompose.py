"""Stage 3: cluster decomposition — the graph, the plan, and replay against the numeric path."""

from __future__ import annotations

import math

import numpy as np
import pytest

from gcs import constraints as C
from gcs import examples, solve
from gcs.decompose import PlanSolver, build_graph
from gcs.model import Sketch


def test_graph_maps_examples() -> None:
    g = build_graph(examples.rect_fillets())
    assert g["nPoints"] == 12 and len(g["lines"]) == 4 and not g["unsupported"]
    assert len(g["virtuals"]) == 8            # one radius line per arc-endpoint tangency
    assert len(g["dirs"]) == 4 + 8            # H/V + tangency perpendiculars
    g = build_graph(examples.truss())
    assert len(g["passive"]) == 30 and len(g["lines"]) == 1   # only the horizontal member
    g = build_graph(examples.polygon_chain(6))
    assert len(g["unsupported"]) == 6         # EqualLength is not an F–H constraint


@pytest.mark.parametrize("name", ["rect_fillets", "slotted_link", "truss"])
def test_examples_fully_decompose_and_replay_exactly(name: str) -> None:
    sk = examples.EXAMPLES[name]()
    ps = PlanSolver(sk)
    assert ps.plan.fully_decomposed, ps.plan.summary
    for seed in range(3):
        examples.perturb(sk, 2.0, seed=seed)
        r = ps.solve(fallback=False)
        assert r.success and not r.fell_back and r.max_residual < 1e-8, r
    # constraint values are read live: change a dimension, replay without recompiling
    if name == "rect_fillets":
        d = next(c for c in sk.constraints if isinstance(c, C.Distance) and c.d == 80)
        d.d = 120
        r = ps.solve(fallback=False)
        assert r.success and max(p.x.value for p in sk.points) == pytest.approx(140.0, abs=1e-6)


def test_point_line_distance_is_a_pl_element() -> None:
    """It is the same F–H element as PointOnLine, just with a non-zero distance, so a point placed
    by one distance to a point and one to a line still decomposes."""
    sk = Sketch()
    base = sk.line(sk.point(0, 0, fixed=True), sk.point(10, 0, fixed=True))
    p = sk.point(4, 9)
    sk.add(C.PointLineDistance(p, base, 3.0), C.Distance(base.p1, p, 5.0))
    g = build_graph(sk)
    assert not g["unsupported"]
    assert sum(e["kind"] == "PL" for e in g["edges"]) == 3   # + the two endpoints
    ps = PlanSolver(sk)
    assert ps.plan.fully_decomposed, ps.plan.summary
    r = ps.solve(fallback=False)
    assert r.success and r.max_residual < 1e-8
    assert p.y.value == pytest.approx(3.0, abs=1e-7) and p.x.value == pytest.approx(4.0, abs=1e-7)


def test_annular_distance_carries_a_known_radius() -> None:
    """Like EqualRadius it is absorbed into the known-radius map rather than becoming an element,
    so a ring dimensioned on one circle still places geometry on the other."""
    sk = Sketch()
    c = sk.point(0, 0, fixed=True)
    inner, mid, outer = sk.circle(c, 10.0), sk.circle(c, 13.0), sk.circle(c, 15.0)
    sk.add(C.Radius(inner, 10.0), C.AnnularDistance(inner, mid, 3.0),
           C.AnnularDistance(mid, outer, 2.0))
    g = build_graph(sk)
    assert not g["unsupported"]
    kr = g["knownRadius"]                    # the chain resolves from the one dimension
    assert kr[str(mid.radius.index)] == pytest.approx(13.0)
    assert kr[str(outer.radius.index)] == pytest.approx(15.0)


def test_parallel_distance_is_a_pl_element() -> None:
    """One residual makes it the same PL element as PointLineDistance, so a gap dimension no
    longer forces the whole sketch onto the numeric fallback."""
    sk = Sketch()
    base = sk.line(sk.point(0, 0, fixed=True), sk.point(10, 0, fixed=True))
    other = sk.line(sk.point(1, 3), sk.point(12, 9))
    sk.add(C.Parallel(base, other), C.ParallelDistance(base, other, 5.0),
           C.Distance(base.p1, other.p1, 5.0), C.Distance(other.p1, other.p2, 10.0))
    assert not build_graph(sk)["unsupported"]
    ps = PlanSolver(sk)
    assert ps.plan.fully_decomposed, ps.plan.summary
    r = ps.solve(fallback=False)
    assert r.success and r.max_residual < 1e-8
    for p in (other.p1, other.p2):
        assert p.y.value == pytest.approx(5.0, abs=1e-7)


def test_unsupported_constraints_fall_back_to_numeric() -> None:
    sk = examples.polygon_chain(8)
    ps = PlanSolver(sk)
    assert not ps.plan.fully_decomposed
    examples.perturb(sk, 2.0)
    r = ps.solve()
    assert r.success and r.fell_back


def test_chirality_flags_follow_the_current_geometry() -> None:
    """A triangle merge has two roots; the replay keeps the one the sketch is on."""
    sk = Sketch()
    a, b = sk.point(0, 0, fixed=True), sk.point(10, 0, fixed=True)
    c = sk.point(5, 4)
    sk.add(C.Distance(a, c, 6), C.Distance(b, c, 6))
    ps = PlanSolver(sk)
    assert ps.plan.fully_decomposed
    r = ps.solve(fallback=False)
    assert r.success and c.y.value > 0
    signs_up = [st.branch for st in ps.plan.steps if st.ppp is not None]
    c.y.value = -4                                       # flip the sketch to the other root
    r = ps.solve(fallback=False)
    assert r.success and c.y.value < 0
    assert signs_up
    assert [st.branch for st in ps.plan.steps if st.ppp is not None] == [-s for s in signs_up]
    # sticky branches: the recorded root wins even if the sketch moved
    ps.sticky_branches = True
    c.y.value = 4
    ps.solve(fallback=False)
    assert c.y.value < 0


def test_k33_needs_a_core_and_decomposes() -> None:
    """K3,3 is minimally rigid but has no triangles: no pair/triple merge applies, so the Stage-3b
    core search must merge (all of) it in one numeric step."""
    sk = examples.k33()
    ps = PlanSolver(sk)
    assert ps.plan.fully_decomposed
    assert max(len(st.ids) for st in ps.plan.steps) >= 4      # a core merge happened
    examples.perturb(sk, 1.0, seed=1)
    r = ps.solve(fallback=False)
    assert r.success and r.max_residual < 1e-8


@pytest.mark.parametrize("seed", range(8))
def test_laman_frameworks_decompose_fully(seed: int) -> None:
    sk = examples.laman(4 + seed % 9, 500 + seed)
    ps = PlanSolver(sk)
    # Henneberg-II moves can create non-tree-decomposable cores: Stage 3b merges those as one
    # numeric step, so every Laman framework decomposes fully
    assert ps.plan.fully_decomposed, ps.plan.summary
    examples.perturb(sk, 1.0, seed=seed)
    r = ps.solve(fallback=False)
    assert r.success and r.max_residual < 1e-8


def test_direction_classes_parallel_and_perpendicular() -> None:
    """A parallel pair is not a rigid pair (free separation) — the plan must not pin it."""
    sk = Sketch()
    l1 = sk.line(sk.point(0, 0, fixed=True), sk.point(10, 0, fixed=True))
    l2 = sk.line(sk.point(0, 5), sk.point(10, 5))
    l3 = sk.line(sk.point(3, 5), sk.point(3, 12))
    sk.add(C.Parallel(l1, l2), C.Distance(l1.p1, l2.p1, 5), C.Vertical(l3),
           C.Coincident(l3.p1, l2.p1), C.Distance(l3.p1, l3.p2, 7), C.Distance(l2.p1, l2.p2, 10))
    ps = PlanSolver(sk)
    examples.perturb(sk, 1.0)
    r = ps.solve()
    assert r.success
    for c in sk.constraints:
        assert c.error() < 1e-6


# -- regression: plan path vs monolithic numeric path ---------------------------

def _all_satisfied(sk: Sketch) -> bool:
    from gcs.diagnose import violated_constraints
    from gcs.solve import System

    return not violated_constraints(System(sk))


@pytest.mark.parametrize("name", list(examples.EXAMPLES))
def test_plan_and_numeric_agree(name: str) -> None:
    for seed in range(3):
        a, b = examples.EXAMPLES[name](), examples.EXAMPLES[name]()
        examples.perturb(a, 1.0, seed=seed)
        examples.perturb(b, 1.0, seed=seed)
        ra = PlanSolver(a).solve()
        rb = solve(b)
        assert ra.success and rb.success
        assert _all_satisfied(a) and _all_satisfied(b)
        if name != "polygon_chain":     # fully constrained: same solution
            np.testing.assert_allclose(a.get_x(), b.get_x(), atol=1e-5)
