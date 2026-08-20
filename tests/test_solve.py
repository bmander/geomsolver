import numpy as np
import pytest

from gcs import Coincident, Distance, DragTarget, Sketch, examples, solve
from gcs import constraints as C
from gcs.examples import perturb as _perturb
from gcs.solve import METHODS, Drag, System


@pytest.mark.parametrize("method", METHODS)
@pytest.mark.parametrize("name", list(examples.EXAMPLES))
def test_examples_solve_from_perturbed_start(name: str, method: str) -> None:
    sk = examples.EXAMPLES[name]()
    _perturb(sk, 2.0)
    res = solve(sk, method=method)
    assert res.success, res
    assert res.max_residual < 1e-8
    for c in sk.constraints:
        assert c.error() < 1e-6


def test_rect_fillets_geometry() -> None:
    sk = examples.rect_fillets(100, 60, 10)
    _perturb(sk, 3.0)
    solve(sk)
    xs = [p.x.value for p in sk.points]
    ys = [p.y.value for p in sk.points]
    assert max(xs) == pytest.approx(100.0, abs=1e-6)
    assert max(ys) == pytest.approx(60.0, abs=1e-6)
    assert min(xs) == pytest.approx(0.0, abs=1e-6)
    for a in sk.arcs:
        assert a.radius.value == pytest.approx(10.0, abs=1e-6)


def test_deterministic() -> None:
    def run() -> np.ndarray:
        sk = examples.truss()
        _perturb(sk, 1.5, seed=3)
        solve(sk, method="dogleg")
        return sk.get_x()

    a, b = run(), run()
    assert np.array_equal(a, b), "same sketch + same edit must give bit-identical results"


def test_underconstrained_moves_minimally() -> None:
    """Two free points, one distance constraint: solver should split the correction."""
    sk = Sketch()
    p, q = sk.point(0, 0), sk.point(12, 0)
    sk.add(Distance(p, q, 10))
    res = solve(sk)
    assert res.success
    assert p.x.value == pytest.approx(1.0, abs=1e-6)
    assert q.x.value == pytest.approx(11.0, abs=1e-6)


def test_drag_reuses_compiled_system() -> None:
    sk = examples.slotted_link()
    # free the anchor so the whole link can translate/rotate under drag
    for prm in sk.params:
        prm.fixed = False
    tgt = sk.lines[0].p2
    drag = DragTarget(tgt, *tgt.xy, weight=0.1)
    sk.add(drag)
    sys_ = System(sk)
    for i in range(1, 6):
        drag.set_target(80 + i, 15 + 2 * i)
        sys_.update_consts(drag)   # the compiled plan holds its own copy of the constants
        res = sys_.solve(method="dogleg")
        assert res.success
    assert tgt.x.value == pytest.approx(85, abs=1e-5)
    assert tgt.y.value == pytest.approx(25, abs=1e-5)
    # hard constraints still hold
    for c in sk.constraints:
        if c is not drag:
            assert c.error() < 1e-6


def test_drag_helper_pull_then_polish() -> None:
    """Dragging a fully-constrained point: hard constraints hold exactly, drag target removed on end."""
    sk = examples.rect_fillets()
    n = len(sk.constraints)
    p = sk.lines[0].p2
    d = Drag(sk, p, *p.xy)
    for i in range(1, 4):
        res = d.move(p.xy[0] + i, p.xy[1] + i)
        assert res.success and res.max_residual < 1e-8
    d.end()
    assert len(sk.constraints) == n
    for c in sk.constraints:
        assert c.error() < 1e-6


def test_overconstrained_conflict_reports_failure() -> None:
    sk = Sketch()
    p, q = sk.point(0, 0, fixed=True), sk.point(5, 0)
    sk.add(Distance(p, q, 10), Distance(p, q, 4))
    res = solve(sk)
    assert not res.success
    assert res.max_residual > 1.0


def test_coincident_chain() -> None:
    sk = examples.polygon_chain(6)
    _perturb(sk, 4.0)
    res = solve(sk, method="dogleg")
    assert res.success
    for c in sk.constraints:
        if isinstance(c, Coincident):
            assert c.error() < 1e-8


def test_symmetric_mirrors_two_points_about_a_line() -> None:
    """The midpoint lands on the axis and the connecting segment crosses it squarely."""
    sk = Sketch()
    a, b = sk.point(0, 0, fixed=True), sk.point(0, 10, fixed=True)     # the axis: x = 0
    p, q = sk.point(-3, 4), sk.point(9, 1)
    sk.add(C.Symmetric(p, q, sk.line(a, b)))
    assert solve(sk).success
    assert p.x.value == pytest.approx(-q.x.value, abs=1e-9)
    assert p.y.value == pytest.approx(q.y.value, abs=1e-9)


def test_symmetric_works_about_a_free_axis() -> None:
    """The axis is geometry too: mirror about a diagonal and the constraint still holds."""
    sk = Sketch()
    a, b = sk.point(0, 0, fixed=True), sk.point(10, 10, fixed=True)    # the axis: y = x
    p, q = sk.point(6, 1), sk.point(0, 5)
    p.fix()
    sk.add(C.Symmetric(p, q, sk.line(a, b)))
    assert solve(sk).success
    assert (q.x.value, q.y.value) == pytest.approx((1.0, 6.0), abs=1e-9)


def test_parallel_distance_dimensions_the_gap_between_parallel_lines() -> None:
    """With the lines parallel by other means, one residual pins the gap."""
    sk = Sketch()
    base = sk.line(sk.point(0, 0, fixed=True), sk.point(10, 0, fixed=True))
    other = sk.line(sk.point(1, 3), sk.point(12, 9))
    sk.add(C.Parallel(base, other), C.ParallelDistance(base, other, 5.0))
    assert solve(sk).success
    for p in (other.p1, other.p2):
        assert p.y.value == pytest.approx(5.0, abs=1e-7)       # left of a +x base is +y


def test_parallel_distance_does_not_itself_make_lines_parallel() -> None:
    """It dimensions a gap, it does not create the parallelism — that is `Parallel`'s job, and
    bundling the two duplicated a constraint most sketches already imply."""
    sk = Sketch()
    base = sk.line(sk.point(0, 0, fixed=True), sk.point(10, 0, fixed=True))
    other = sk.line(sk.point(1, 3), sk.point(12, 9))
    sk.add(C.ParallelDistance(base, other, 5.0))
    assert solve(sk).success
    assert other.p1.y.value == pytest.approx(5.0, abs=1e-7)    # the anchored endpoint moved
    d1, d2 = base.direction(), other.direction()
    assert abs(d1[0] * d2[1] - d1[1] * d2[0]) > 1e-6           # the lines are still skew


def test_parallel_distance_sign_picks_the_side() -> None:
    sk = Sketch()
    base = sk.line(sk.point(0, 0, fixed=True), sk.point(10, 0, fixed=True))
    other = sk.line(sk.point(1, 3), sk.point(12, 9))
    sk.add(C.Parallel(base, other), C.ParallelDistance(base, other, -5.0))
    assert solve(sk).success
    assert other.p1.y.value == pytest.approx(-5.0, abs=1e-7)


def test_same_constraint_matches_exact_repeats() -> None:
    """The check that keeps a duplicate out of a sketch.  A duplicate adds equations without
    adding rank, and the structural matching cannot see it — the bug this exists to prevent."""
    sk = Sketch()
    p, q, r = sk.point(0, 0), sk.point(3, 0), sk.point(0, 4)
    line = sk.line(p, q)
    assert C.same_constraint(C.Coincident(p, q), C.Coincident(p, q))
    assert not C.same_constraint(C.Coincident(p, q), C.Coincident(p, r))
    assert not C.same_constraint(C.Coincident(p, q), C.Midpoint(p, line))
    assert C.same_constraint(C.Distance(p, q, 5.0), C.Distance(p, q, 5.0))
    assert not C.same_constraint(C.Distance(p, q, 5.0), C.Distance(p, q, 6.0))  # that is a conflict
    assert C.same_constraint(C.Symmetric(p, q, line), C.Symmetric(p, q, line))


def test_same_constraint_sees_through_a_swapped_pair() -> None:
    """Picking the pair in the other order means the same relation — but only where swapping
    really is a no-op, which is why it is a per-type flag and not a blanket rule."""
    sk = Sketch()
    p, q = sk.point(0, 0), sk.point(3, 0)
    l1, l2 = sk.line(p, q), sk.line(sk.point(0, 5), sk.point(3, 5))
    c1, c2 = sk.circle(p, 2.0), sk.circle(q, 3.0)
    assert C.same_constraint(C.Coincident(p, q), C.Coincident(q, p))
    assert C.same_constraint(C.Parallel(l1, l2), C.Parallel(l2, l1))
    assert C.same_constraint(C.Perpendicular(l1, l2), C.Perpendicular(l2, l1))
    assert C.same_constraint(C.EqualLength(l1, l2), C.EqualLength(l2, l1))
    assert C.same_constraint(C.EqualRadius(c1, c2), C.EqualRadius(c2, c1))
    assert C.same_constraint(C.Distance(p, q, 5.0), C.Distance(q, p, 5.0))
    assert C.same_constraint(C.Symmetric(p, q, l1), C.Symmetric(q, p, l1))
    # ...and the ones where the first argument is the reference, so a swap is a different thing
    assert not C.same_constraint(C.Angle(l1, l2, 0.7), C.Angle(l2, l1, 0.7))
    assert not C.same_constraint(C.ParallelDistance(l1, l2, 4.0), C.ParallelDistance(l2, l1, 4.0))
    assert not C.same_constraint(C.AnnularDistance(c1, c2, 1.0), C.AnnularDistance(c2, c1, 1.0))


def test_same_constraint_covers_every_type_reflexively() -> None:
    """Whatever `spec` a new type declares, it must at least recognise itself — the guard is
    spec-driven so that adding a constraint type cannot silently opt out of it."""
    from tests.test_jacobians import all_constraints

    for a, b in zip(all_constraints(0), all_constraints(1)):
        assert not C.same_constraint(a, b)          # different sketches: different entities
    for c in all_constraints(0):
        assert C.same_constraint(c, c)


def test_annular_distance_sets_the_ring_thickness() -> None:
    sk = Sketch()
    c = sk.point(0, 0, fixed=True)
    inner, outer = sk.circle(c, 10.0), sk.circle(c, 12.0)
    sk.add(C.Radius(inner, 10.0), C.AnnularDistance(inner, outer, 3.0))
    assert solve(sk).success
    assert outer.radius.value == pytest.approx(13.0, abs=1e-9)


def test_annular_distance_is_signed_and_drives_either_circle() -> None:
    """Dimension the outer one and the inner follows — nothing distinguishes the two ends."""
    sk = Sketch()
    c = sk.point(0, 0, fixed=True)
    inner, outer = sk.circle(c, 10.0), sk.circle(c, 12.0)
    sk.add(C.Radius(outer, 20.0), C.AnnularDistance(inner, outer, -4.0))
    assert solve(sk).success
    assert inner.radius.value == pytest.approx(24.0, abs=1e-9)   # negative d flips which is outer


def test_annular_distance_does_not_itself_make_the_circles_concentric() -> None:
    """It dimensions the radii only; `Coincident` on the centres is what centres them."""
    sk = Sketch()
    a, b = sk.point(0, 0, fixed=True), sk.point(7, 0)
    inner, outer = sk.circle(a, 10.0), sk.circle(b, 12.0)
    sk.add(C.Radius(inner, 10.0), C.AnnularDistance(inner, outer, 3.0))
    assert solve(sk).success
    assert outer.radius.value == pytest.approx(13.0, abs=1e-9)
    assert b.x.value == pytest.approx(7.0, abs=1e-9)             # the centre never moved


def test_point_line_distance_offsets_a_point_from_a_line() -> None:
    sk = Sketch()
    base = sk.line(sk.point(0, 0, fixed=True), sk.point(10, 0, fixed=True))
    p = sk.point(4, 9)
    sk.add(C.PointLineDistance(p, base, 3.0))
    assert solve(sk).success
    assert p.y.value == pytest.approx(3.0, abs=1e-7)           # left of a +x base is +y
    assert p.x.value == pytest.approx(4.0, abs=1e-7)           # it slides only perpendicular


def test_point_line_distance_sign_picks_the_side() -> None:
    sk = Sketch()
    base = sk.line(sk.point(0, 0, fixed=True), sk.point(10, 0, fixed=True))
    p = sk.point(4, 9)
    sk.add(C.PointLineDistance(p, base, -3.0))
    assert solve(sk).success
    assert p.y.value == pytest.approx(-3.0, abs=1e-7)


def test_point_line_distance_measures_to_the_infinite_line() -> None:
    """The foot of the perpendicular may fall outside the segment — that is the dimension a
    drawing means by "distance to this edge"."""
    sk = Sketch()
    base = sk.line(sk.point(0, 0, fixed=True), sk.point(1, 0, fixed=True))
    p = sk.point(50, 9)
    sk.add(C.PointLineDistance(p, base, 3.0))
    assert solve(sk).success
    assert p.y.value == pytest.approx(3.0, abs=1e-7)
    assert p.x.value == pytest.approx(50.0, abs=1e-7)


def test_point_line_distance_moves_the_line_when_the_point_is_fixed() -> None:
    """Nothing distinguishes the two ends of the constraint: with the point pinned the line
    swings instead."""
    sk = Sketch()
    p = sk.point(0, 0, fixed=True)
    line = sk.line(sk.point(-5, 4), sk.point(5, 4))
    sk.add(C.Horizontal(line), C.PointLineDistance(p, line, -2.0))
    assert solve(sk).success
    for q in (line.p1, line.p2):
        assert q.y.value == pytest.approx(2.0, abs=1e-7)       # p is 2 to the right of +x, i.e. below


def test_parallel_distance_alongside_parallel_is_not_redundant() -> None:
    """The pairing a real sketch uses: the lines are parallel through some other chain, and
    the dimension adds exactly one equation on top of it."""
    from gcs.diagnose import diagnose

    sk = Sketch()
    base = sk.line(sk.point(0, 0, fixed=True), sk.point(10, 0, fixed=True))
    other = sk.line(sk.point(1, 3), sk.point(12, 9))
    sk.add(C.Parallel(base, other), C.ParallelDistance(base, other, 5.0))
    d = diagnose(sk)
    assert d.n_redundant == 0
    assert d.geometric_dependency == 0
