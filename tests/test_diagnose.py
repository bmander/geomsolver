"""Stage 2: structural diagnosis — matching/DM, pebble game, conflict sets, Laman property test."""

from __future__ import annotations

import math

import pytest

from gcs import constraints as C
from gcs import examples, graph, io, solve
from gcs.diagnose import diagnose, distance_rigidity, minimal_conflict_set
from gcs.examples import henneberg_edges as henneberg
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
    assert k4.is_rigid() and k4.redundant == [5]   # (1,3) is the 6th edge: dependent on the rest
    bow = graph.pebble_game(5, [(0, 1), (1, 2), (2, 0), (2, 3), (3, 4), (4, 2)])
    assert bow.dof == 1 and sorted(map(sorted, bow.components)) == [[0, 1, 2], [2, 3, 4]]


@pytest.mark.parametrize("seed", range(6))
def test_pebble_game_recognises_laman_graphs(seed: int) -> None:
    n = 4 + seed
    edges = henneberg(n, seed + 1)
    assert len(edges) == 2 * n - 3
    res = graph.pebble_game(n, edges)
    assert res.is_rigid() and not res.redundant
    assert res.components == [frozenset(range(n))]
    # any extra edge is redundant
    extra = next((a, b) for a in range(n) for b in range(a + 1, n)
                 if (a, b) not in edges and (b, a) not in edges)
    res2 = graph.pebble_game(n, edges + [extra])
    assert res2.redundant == [len(edges)] and res2.is_rigid()
    # removing one edge leaves 1 DOF
    assert graph.pebble_game(n, edges[1:]).dof == 1


@pytest.mark.parametrize("seed", range(4))
def test_laman_framework_solves_and_agrees_with_pebble_game(seed: int) -> None:
    """Property test from the plan: random Laman graph, random lengths from a random realization →
    the solver finds a realization; the pebble game says rigid; DM says 3 DOF (rigid motions)."""
    n = 4 + seed
    sk = examples.laman(n, 100 + seed)
    # `laman` grounds the framework; strip that to see the bare rigid body
    sk.constraints = [c for c in sk.constraints if not isinstance(c, C.Horizontal)]
    for p in sk.params:
        p.fixed = False
    d0 = diagnose(sk)
    assert len(d0.rigid_clusters) == 1 and len(d0.rigid_clusters[0]) == n
    assert d0.dof == 3 and d0.n_redundant == 0     # rigid body: 2 translations + rotation
    examples.perturb(sk, 1.0, seed=seed)
    res = solve(sk)
    assert res.success and res.max_residual < 1e-8


# -- sketch-level diagnosis ---------------------------------------------------

def test_dof_counts_what_can_actually_move_not_what_the_matching_sees() -> None:
    """A matching cannot tell that two equations say the same thing — it counts both and calls the
    sketch rigid while the geometry still moves.  `dof` therefore reports the numeric rank when the
    cross-check ran; `structural_dof` keeps the matching's generous answer."""
    sk = examples.altitudes()          # the altitudes concur: a dependency only the numbers see
    solve(sk)
    d = diagnose(sk)
    assert d.geometric_dependency == 1
    assert d.structural_dof == 2                       # what the matching alone believes
    assert d.dof == 3                                  # what is actually free to move
    assert d.dof == d.n_params - d.numeric_rank
    assert len(d.under_params) >= d.dof                # and dragging agrees with the count


def test_redundancy_the_matching_cannot_see_is_counted_and_named_as_implied() -> None:
    """The altitudes concur, so one of the six constraints is implied by the other five.  The
    matching sees six independent equations and calls the sketch merely under-constrained; the
    numeric rank sees the dependency, and the report names every constraint that could go.  But
    it is a theorem among pure relations — nothing can ever break it — so they are `implied`, not
    `over`: the sketch stays merely under-constrained, and nothing is painted as a fault."""
    sk = examples.altitudes()
    solve(sk)
    d = diagnose(sk)
    assert d.structural_n_redundant == 0        # what the matching alone believes
    assert d.n_redundant == 1 and d.status == "under"
    assert d.over == []
    named = {io.describe(c) for c in d.implied}
    assert named == {"Perpendicular(L3, L1)", "Perpendicular(L4, L2)", "Perpendicular(L5, L0)",
                     "PointOnLine(P6, L3)", "PointOnLine(P6, L4)", "PointOnLine(P6, L5)"}
    assert all(st != "over" for st in d.entity_state.values())
    # the same set the Stage-4 witness reaches independently
    w = diagnose(sk, witness=True).witness
    assert w is not None and w.dependencies
    dep = w.dependencies[0]
    assert {io.describe(c) for c in [dep.constraint, *dep.implied_by]} <= named


def test_a_relation_only_theorem_is_implied_not_over() -> None:
    """Two arcs on one centre, the centre on a line, equal radii, an endpoint of each mirrored
    about the line.  Mirroring about a line through the centre preserves the distance to it, so
    EqualRadius follows — and so does the centre being on the line (the chord's perpendicular
    bisector).  Each is wholly implied and neither carries a dimension: the user can drag the
    sketch anywhere and it stays consistent, so the report remarks on it rather than flagging it."""
    sk = Sketch()
    a, b = sk.point(-20, 0), sk.point(40, 0)
    line = sk.line(a, b)
    c = sk.point(10, 0, fixed=True)
    arc1 = sk.arc(c, sk.point(18, 6), sk.point(4, 8))
    arc2 = sk.arc(c, sk.point(4, -8), sk.point(18, -6))
    on_line, equal = C.PointOnLine(c, line), C.EqualRadius(arc1, arc2)
    sk.add(on_line, equal, C.Symmetric(arc2.start, arc1.end, line))
    solve(sk)
    d = diagnose(sk)
    assert d.geometric_dependency == 1 and d.n_redundant == 1
    assert d.status == "under" and d.over == [] and not d.violated
    assert {io.describe(k) for k in d.implied} == {io.describe(on_line), io.describe(equal)}
    assert all(st != "over" for st in d.entity_state.values())


def test_a_dependency_that_involves_a_dimension_is_still_over() -> None:
    """The same kind of theorem — two equal distances make EqualLength follow — but the rows
    that take part carry dimensions, and editing either is a conflict.  Worth flagging now."""
    sk = Sketch()
    p, q, r = sk.point(0, 0, fixed=True), sk.point(5, 0), sk.point(5, 5)
    equal = C.EqualLength(sk.line(p, q), sk.line(q, r))
    sk.add(C.Distance(p, q, 5), C.Distance(q, r, 5), equal)
    solve(sk)
    d = diagnose(sk)
    assert d.geometric_dependency == 1 and d.status == "over"
    assert len(d.over) == 3 and equal in d.over
    assert d.implied == []


def test_a_dependency_with_nothing_to_remove_is_not_called_over_constrained() -> None:
    """An arc centred on a line endpoint, its two endpoints mirrored about that line.  The two
    intrinsic radius equations plus "the chord is perpendicular to the line" already force "the
    chord's midpoint is on the line", so one of `Symmetric`'s two residuals is implied — a real
    rank deficiency.  But `Symmetric` still carries the perpendicularity, and the intrinsic
    equations cannot be deleted, so there is nothing to tell the user to remove."""
    sk = Sketch()
    a = sk.point(0.0, 0.0)
    centre = sk.point(10.0, 0.0)
    line = sk.line(a, centre)
    arc = sk.arc(centre, sk.point(13.0, 4.0), sk.point(13.0, -4.0))
    sk.add(C.Symmetric(arc.start, arc.end, line))
    solve(sk)
    d = diagnose(sk)
    assert d.geometric_dependency == 1 and d.n_redundant == 1     # the deficiency is real...
    assert d.over == []                                           # ...but nothing is removable
    assert d.status == "under" and d.dof > 0
    assert all(st != "over" for st in d.entity_state.values())


def test_a_wholly_implied_constraint_is_still_named() -> None:
    """The other side of the same test: when a constraint really is redundant on its own, it has
    to be named — otherwise the check above would just silence every report."""
    sk = examples.truss(4)
    p, q = sk.points[0], sk.points[2]
    extra = C.Distance(p, q, math.dist(p.xy, q.xy))
    sk.add(extra)
    solve(sk)
    d = diagnose(sk)
    assert d.status == "over" and d.n_redundant == 1
    assert any(c is extra for c in d.over)


def test_the_named_culprits_use_the_system_row_order_not_the_sketch_order() -> None:
    """`System` compiles to a plan that batches rows by kernel, so row i is not the i-th
    constraint.  Mapping a dependent row back through the sketch's own ordering names the wrong
    constraints — this pins the mapping to `structure()`, which is the authority."""
    from gcs.solve import System

    sk = Sketch()                                  # interleave two kernels so batching reorders
    a, b, c = sk.point(0, 0, fixed=True), sk.point(3, 0), sk.point(0, 4)
    sk.add(C.Distance(a, b, 3.0), C.Coincident(b, c), C.Distance(a, c, 4.0))
    _, row_c = System(sk).structure()
    naive = [k for k in sk.hard_constraints() for _ in range(k.n_residuals)]
    assert not all(x is y for x, y in zip(row_c, naive)), "kernel batching should reorder rows"
    assert {type(k).__name__ for k in row_c} == {"Distance", "Coincident"}


def test_dof_is_unchanged_when_the_two_ranks_agree() -> None:
    """The common case: no theorem-type dependency, so nothing about the report moves."""
    sk = examples.rect_fillets()
    solve(sk)
    d = diagnose(sk)
    assert d.geometric_dependency == 0
    assert d.dof == d.structural_dof == 0 and d.status == "well"
    assert not d.under_params


def test_well_constrained_examples() -> None:
    for name in ("rect_fillets", "slotted_link", "truss"):
        d = diagnose(examples.EXAMPLES[name]())
        assert d.status == "well" and d.dof == 0 and d.n_redundant == 0 and not d.warnings, name
        assert all(s == "well" for s in d.entity_state.values())


def test_conflict_set_is_the_two_distances() -> None:
    sk = examples.rect_fillets()
    # the width the case states, and a second number on the *same pair*: what makes this a
    # minimal conflict of two is that both name one length, not that both are lengths
    width = next(c for c in sk.constraints if isinstance(c, C.Distance) and c.d == 100)
    extra = C.Distance(width.p, width.q, 50)
    sk.add(extra)
    solve(sk)
    d = diagnose(sk)
    assert d.status == "conflict"
    assert d.n_redundant == 1
    assert set(d.conflicts or []) == {extra, width}
    assert d.entity_state[width.p] == "conflict"
    assert d.entity_state[width.q] == "conflict"


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
    # numeric (null-space) view: only the x-coordinates of the right-hand side can move
    assert {p.name for p in d.under_params} == {"c2.x", "t2.x", "b1.x"}
    # structural (DM) view is generous: it also lists the y's and the left arc endpoints
    assert {"c2.x", "c2.y"} <= {p.name for p in d.structural_under_params}
    assert sorted(c.dof for c in d.components) == [0, 0, 1]
    assert d.entity_state[sk.points[1]] == "under" and d.entity_state[sk.points[0]] == "well"


def test_null_space_pins_left_side_of_undimensioned_rect() -> None:
    """Remove the width: geometrically the fixed lower-left arc, the left edge and the upper-left
    arc stay pinned; only the right side slides.  Structural analysis can't see that (tangent
    equations mention the far endpoint), the null space can."""
    sk = examples.rect_fillets()
    sk.remove(next(c for c in sk.constraints if isinstance(c, C.Distance) and c.d == 100))
    d = diagnose(sk)
    assert d.dof == 1
    assert {p.name for p in d.under_params} == {"b2.x", "r1.x", "r2.x", "t1.x", "c_br.x", "c_tr.x"}
    st = {i: d.entity_state[e] for i, e in enumerate([*sk.lines, *sk.arcs])}
    assert st[3] == "well" and st[6] == "well" and st[7] == "well"      # left edge, A2, A3
    assert st[0] == "under" and st[1] == "under" and st[2] == "under"   # bottom, right, top


def test_theorem_type_dependency_is_logged() -> None:
    d = diagnose(examples.polygon_chain(8))
    assert d.numeric_rank is not None and d.numeric_rank == d.structural_rank - 1
    assert d.warnings


def test_minimal_conflict_set_infeasible_triangle() -> None:
    """Structurally well-determined but geometrically impossible (triangle inequality)."""
    sk = examples.impossible_triangle()
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
    for ln in (l1, l2, l3):
        sk.add(C.Distance(ln.p1, ln.p2, ln.length()))
    clusters, red = distance_rigidity(sk)
    assert len(clusters) == 1 and len(clusters[0]) == 6 and not red


def test_conflict_set_on_large_truss_from_good_geometry() -> None:
    """The culprit is found when diagnosing from the last good geometry (what the app does)."""
    sk = examples.truss(30)
    bad = C.Distance(sk.points[0], sk.points[3], 999)
    sk.add(bad)
    d = diagnose(sk)                       # geometry untouched: the truss is still sane
    assert d.status == "conflict" and bad in (d.conflicts or [])
    # a mild but real conflict: the whole bay cycle is the minimal infeasible set
    sk = examples.truss(30)
    bad = C.Distance(sk.points[0], sk.points[3], 21)
    sk.add(bad)
    d = diagnose(sk)
    assert bad in (d.conflicts or []) and 3 <= len(d.conflicts or []) <= 13


def test_under_params_are_per_axis_not_per_point() -> None:
    """A point sliding along a vertical line has its y free and its x pinned.  Anything asking
    "can this point move?" has to look at the whole point, not one coordinate."""
    sk = Sketch()
    a = sk.point(0, 0, fixed=True)
    b = sk.point(0, 10, fixed=True)
    p = sk.point(0, 4)
    sk.add(C.PointOnLine(p, sk.line(a, b)))
    d = diagnose(sk)
    names = {q.name for q in d.under_params}
    assert p.y in d.under_params and p.x not in d.under_params, names
