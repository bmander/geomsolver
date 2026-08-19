"""Stage 3a: constraint graph, cluster merging, plan execution, plan-vs-numeric regression."""

import math
import random

import numpy as np
import pytest

from gcs import constraints as C
from gcs import examples, solve
from gcs.cgraph import build
from gcs.decompose import PlanSolver, decompose, execute
from gcs.model import Sketch
from tests.test_diagnose import henneberg


def test_graph_maps_examples() -> None:
    g = build(examples.rect_fillets())
    assert g.n_points == 12 and len(g.lines) == 4 and not g.unsupported
    assert len(g.virtual) == 8            # one radius line per arc-endpoint tangency
    assert len(g.dirs) == 4 + 8           # H/V + tangency perpendiculars
    g = build(examples.truss())
    assert len(g.passive) == 30 and len(g.lines) == 1   # only the horizontal member is an element
    g = build(examples.polygon_chain(6))
    assert len(g.unsupported) == 6        # EqualLength is not an F–H constraint


@pytest.mark.parametrize("name", ["rect_fillets", "slotted_link", "truss"])
def test_examples_fully_decompose_and_replay_exactly(name: str) -> None:
    sk = examples.EXAMPLES[name]()
    ps = PlanSolver(sk)
    assert ps.plan.fully_decomposed, ps.plan.summary()
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
    signs_up = dict(ps.plan.chirality)
    c.y.value = -4                                       # flip the sketch to the other root
    r = ps.solve(fallback=False)
    assert r.success and c.y.value < 0
    assert signs_up and all(ps.plan.chirality[k] == -v for k, v in signs_up.items())


def test_k33_needs_a_core_and_decomposes() -> None:
    """K3,3 is minimally rigid but has no triangles: no pair/triple merge applies, so the
    Stage-3b core search must merge (all of) it in one numeric step."""
    rng = random.Random(3)
    sk = Sketch()
    pts = [sk.point(rng.uniform(0, 30), rng.uniform(0, 30)) for _ in range(6)]
    pts[0].fix()
    for a in range(3):
        for b in range(3, 6):
            sk.add(C.Distance(pts[a], pts[b], math.dist(pts[a].xy, pts[b].xy)))
    sk.add(C.Horizontal(sk.line(pts[0], pts[3])))
    ps = PlanSolver(sk)
    assert ps.plan.fully_decomposed
    assert max(len(st.ids) for st in ps.plan.steps) >= 4      # a core merge happened
    examples.perturb(sk, 1.0, seed=1)
    r = ps.solve(fallback=False)
    assert r.success and r.max_residual < 1e-8


@pytest.mark.parametrize("seed", range(8))
def test_laman_frameworks_decompose_fully(seed: int) -> None:
    rng = random.Random(500 + seed)
    n = rng.randint(4, 12)
    edges = henneberg(n, rng)
    sk = Sketch()
    pts = [sk.point(rng.uniform(0, 50), rng.uniform(0, 50)) for _ in range(n)]
    pts[0].fix()
    for a, b in edges:
        sk.add(C.Distance(pts[a], pts[b], math.dist(pts[a].xy, pts[b].xy)))
    sk.add(C.Horizontal(sk.line(pts[0], pts[1])))       # remove the rotation DOF
    ps = PlanSolver(sk)
    # Henneberg-II moves can create non-tree-decomposable cores: Stage 3b merges those as one
    # numeric step, so every Laman framework decomposes fully
    assert ps.plan.fully_decomposed, ps.plan.summary()
    examples.perturb(sk, 1.0, seed=seed)
    r = ps.solve(fallback=False)
    assert r.success and r.max_residual < 1e-8


def test_direction_classes_parallel_and_perpendicular() -> None:
    """Parallel pair is not a rigid pair (free separation) — the plan must not pin it."""
    sk = Sketch()
    l1 = sk.line(sk.point(0, 0, fixed=True), sk.point(10, 0, fixed=True))
    l2 = sk.line(sk.point(0, 5), sk.point(10, 5))
    l3 = sk.line(sk.point(3, 5), sk.point(3, 12))
    sk.add(C.Parallel(l1, l2), C.Distance(l1.p1, l2.p1, 5), C.Vertical(l3), C.Coincident(l3.p1, l2.p1),
           C.Distance(l3.p1, l3.p2, 7), C.Distance(l2.p1, l2.p2, 10))
    ps = PlanSolver(sk)
    examples.perturb(sk, 1.0)
    r = ps.solve()
    assert r.success
    for c in sk.constraints:
        assert c.error() < 1e-6


# -- regression: plan path vs monolithic numeric path ---------------------------

def _all_satisfied(sk: Sketch, tol: float = 1e-6) -> bool:
    return all(c.error() < tol for c in sk.hard_constraints())


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
