"""Stage 2: structural diagnosis — matching/DM, pebble game, conflict sets, Laman property test."""

import math
import random

import numpy as np
import pytest

from gcs import constraints as C
from gcs import examples, graph, solve
from gcs.diagnose import diagnose, distance_rigidity, minimal_conflict_set
from gcs.model import Sketch


# -- graph algorithms -------------------------------------------------------

def test_hopcroft_karp_perfect_and_deficient() -> None:
    assert sum(m >= 0 for m in graph.hopcroft_karp([[0, 1], [1, 2], [2, 0]], 3)[0]) == 3
    assert sum(m >= 0 for m in graph.hopcroft_karp([[0], [0], [0]], 1)[0]) == 1


def test_dm_blocks() -> None:
    # rows 0,1 both depend only on col 0 -> over block; cols 1,2 share one row -> under block
    dm = graph.dulmage_mendelsohn([[0], [0], [1, 2]], 3)
    assert set(dm.over_rows) == {0, 1} and dm.over_cols == [0]
    assert dm.under_rows == [2] and set(dm.under_cols) == {1, 2}
    assert dm.n_redundant == 1 and dm.n_free == 1 and dm.rank == 2


def test_pebble_game_basics() -> None:
    assert graph.pebble_game(3, [(0, 1), (1, 2), (2, 0)]).is_rigid()
    assert graph.pebble_game(4, [(0, 1), (1, 2), (2, 3), (3, 0)]).dof == 1
    k4 = graph.pebble_game(4, [(0, 1), (1, 2), (2, 3), (3, 0), (0, 2), (1, 3)])
    assert k4.is_rigid() and len(k4.redundant) == 1
    bow = graph.pebble_game(5, [(0, 1), (1, 2), (2, 0), (2, 3), (3, 4), (4, 2)])
    assert bow.dof == 1 and sorted(map(sorted, bow.components)) == [[0, 1, 2], [2, 3, 4]]


def henneberg(n: int, rng: random.Random) -> list[tuple[int, int]]:
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


@pytest.mark.parametrize("seed", range(6))
def test_pebble_game_recognises_laman_graphs(seed: int) -> None:
    rng = random.Random(seed)
    n = rng.randint(4, 14)
    edges = henneberg(n, rng)
    assert len(edges) == 2 * n - 3
    res = graph.pebble_game(n, edges)
    assert res.is_rigid() and not res.redundant
    assert res.components == [frozenset(range(n))]
    # any extra edge is redundant
    extra = next((a, b) for a in range(n) for b in range(a + 1, n) if (a, b) not in edges and (b, a) not in edges)
    res2 = graph.pebble_game(n, edges + [extra])
    assert res2.redundant == [extra] and res2.is_rigid()
    # removing one edge leaves 1 DOF
    assert graph.pebble_game(n, edges[1:]).dof == 1


@pytest.mark.parametrize("seed", range(4))
def test_laman_framework_solves_and_agrees_with_pebble_game(seed: int) -> None:
    """Property test from the plan: random Laman graph, random lengths from a random
    realization → solver finds a realization; pebble game says rigid; DM says 3 DOF (rigid motions)."""
    rng = random.Random(100 + seed)
    n = rng.randint(4, 10)
    edges = henneberg(n, rng)
    sk = Sketch()
    pts = [sk.point(rng.uniform(0, 50), rng.uniform(0, 50)) for _ in range(n)]
    for a, b in edges:
        sk.add(C.Distance(pts[a], pts[b], math.dist(pts[a].xy, pts[b].xy)))
    d0 = diagnose(sk)
    assert len(d0.rigid_clusters) == 1 and len(d0.rigid_clusters[0]) == n
    assert d0.dof == 3 and d0.n_redundant == 0     # rigid body: 2 translations + rotation
    examples.perturb(sk, 1.0, seed=seed)
    res = solve(sk)
    assert res.success and res.max_residual < 1e-8


# -- sketch-level diagnosis ---------------------------------------------------

def test_well_constrained_examples() -> None:
    for name in ("rect_fillets", "slotted_link", "truss"):
        d = diagnose(examples.EXAMPLES[name]())
        assert d.status == "well" and d.dof == 0 and d.n_redundant == 0 and not d.warnings, name
        assert all(s == "well" for s in d.entity_state.values())


def test_conflict_set_is_the_two_distances() -> None:
    sk = examples.rect_fillets()
    extra = C.Distance(sk.lines[0].p1, sk.lines[0].p2, 50)  # contradicts the 80 width
    sk.add(extra)
    solve(sk)
    d = diagnose(sk)
    assert d.status == "conflict"
    assert d.n_redundant == 1
    width = next(c for c in sk.constraints if isinstance(c, C.Distance) and c.d == 80)
    assert set(map(id, d.conflicts or [])) == {id(extra), id(width)}
    assert d.entity_state[id(sk.lines[0])] == "conflict"


def test_redundant_but_consistent_is_over_not_conflict() -> None:
    sk = examples.truss(4)
    p, q = sk.points[0], sk.points[2]
    sk.add(C.Distance(p, q, math.dist(p.xy, q.xy)))
    d = diagnose(sk)
    assert d.status == "over" and d.n_redundant == 1 and not d.violated and not d.conflicts
    assert len(d.redundant_distances) == 1


def test_under_constrained_reports_free_params_and_components() -> None:
    sk = examples.slotted_link()
    sk.constraints = [c for c in sk.constraints if not isinstance(c, C.Distance)]
    d = diagnose(sk)
    assert d.status == "under" and d.dof == 1
    names = {p.name for p in d.under_params}
    assert {"c2.x", "c2.y"} <= names and "c1.x" not in names
    assert sorted(c.dof for c in d.components) == [0, 0, 1]
    assert d.entity_state[id(sk.points[1])] == "under" and d.entity_state[id(sk.points[0])] == "well"


def test_theorem_type_dependency_is_logged() -> None:
    d = diagnose(examples.polygon_chain(8))
    assert d.numeric_rank is not None and d.numeric_rank == d.structural_rank - 1
    assert d.warnings


def test_minimal_conflict_set_infeasible_triangle() -> None:
    """Structurally well-determined but geometrically impossible (triangle inequality)."""
    sk = Sketch()
    a, b, c = sk.point(0, 0, fixed=True), sk.point(10, 0), sk.point(5, 5)
    sk.add(C.Distance(a, b, 10), C.Distance(b, c, 1), C.Distance(a, c, 1), C.Horizontal(sk.line(a, b)))
    solve(sk)
    d = diagnose(sk)
    assert d.n_redundant == 0            # the graph sees nothing wrong...
    assert d.status == "conflict"        # ...but the numbers do
    conf = minimal_conflict_set(sk)
    assert 2 <= len(conf) <= 3 and all(isinstance(x, C.Distance) for x in conf)


def test_distance_rigidity_merges_coincident_points() -> None:
    sk = examples.polygon_chain(5)   # 5 edges via Coincident, no Distance → no edges
    assert distance_rigidity(sk) == ([], [])
    sk = Sketch()
    l1 = sk.line_xy(0, 0, 10, 0)
    l2 = sk.line_xy(10, 0, 5, 8)
    l3 = sk.line_xy(5, 8, 0, 0)
    sk.add(C.Coincident(l1.p2, l2.p1), C.Coincident(l2.p2, l3.p1), C.Coincident(l3.p2, l1.p1))
    for l in (l1, l2, l3):
        sk.add(C.Distance(l.p1, l.p2, l.length()))
    clusters, red = distance_rigidity(sk)
    assert len(clusters) == 1 and len(clusters[0]) == 6 and not red
