import numpy as np
import pytest

from gcs import Coincident, Distance, DragTarget, Sketch, examples, solve
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
