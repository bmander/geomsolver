"""Every constraint's analytic Jacobian must agree with finite differences at random points."""

from __future__ import annotations

import math

import numpy as np
import pytest

from gcs import constraints as C
from gcs import examples
from gcs.constraints import CONSTRAINT_TYPES
from gcs.fdcheck import check_constraint, check_sketch
from gcs.model import Sketch
from gcs.solve import System


def _sketch_with_stuff(seed: int):
    rng = np.random.default_rng(seed)
    sk = Sketch()
    r = lambda: float(rng.uniform(-10, 10))  # noqa: E731
    p, q, s = sk.point(r(), r()), sk.point(r(), r()), sk.point(r(), r())
    l1 = sk.line(sk.point(r(), r()), sk.point(r(), r()))
    l2 = sk.line(sk.point(r(), r()), sk.point(r(), r()))
    c1 = sk.circle(sk.point(r(), r()), abs(r()) + 1)
    c2 = sk.circle(sk.point(r(), r()), abs(r()) + 1)
    a = sk.arc(sk.point(r(), r()), sk.point(r(), r()), sk.point(r(), r()))
    sp = sk.spline([sk.point(r(), r()) for _ in range(6)])
    assert sp is not None
    el = sk.ellipse(sk.point(r(), r()), sk.point(r(), r()), abs(r()) + 1)
    return sk, p, q, s, l1, l2, c1, c2, a, sp, el


def all_constraints(seed: int):
    """One instance of every constraint type, on a sketch of every kind of entity."""
    sk, p, q, s, l1, l2, c1, c2, a, sp, el = _sketch_with_stuff(seed)
    return [
        C.Coincident(p, q), C.Distance(p, q, 3.0), C.Midpoint(p, l1), C.DragTarget(p, 1, 2, 0.3),
        C.Horizontal(l1), C.Vertical(l1), C.Parallel(l1, l2), C.Perpendicular(l1, l2),
        C.HorizontalPoints(p, q), C.VerticalPoints(p, q),
        C.HorizontalDistance(p, q, 2.5), C.VerticalDistance(p, q, -1.5),
        C.Angle(l1, l2, 0.7), C.EqualLength(l1, l2), C.PointOnLine(p, l1), C.PointOnCircle(p, c1),
        C.PointOnCircle(p, a), C.Radius(c1, 2.0), C.EqualRadius(c1, a),
        C.TangentLineCircle(l1, c1), C.TangentLineCircle(l1, c1, -1),
        C.TangentCircleCircle(c1, c2, True), C.TangentCircleCircle(c1, c2, False),
        C.TangentArcLine(a, l1, "start"), C.TangentArcLine(a, l2, "end"),
        C.TangentLineCircleAt(l1, c1, "p1"), C.TangentLineCircleAt(l2, c2, "p2"),
        C.Symmetric(p, q, l1), C.ParallelDistance(l1, l2, 4.0),
        C.PointLineDistance(p, l1, 4.0),
        C.AnnularDistance(c1, a, 1.5),
        C.PointOnSpline(p, sp), C.SplineTangentLine(sp, l1),
        C.SplineCurvature(sp, c1), C.SplineCurvature(sp, a),
        C.PointOnEllipse(p, el), C.PointOnEllipse(q, el),
        C.EllipseTangentLine(el, l1), C.EllipseTangentLine(el, l2),
        C.EllipseCurvature(el, c1), C.EllipseCurvature(el, a),
    ]


@pytest.mark.parametrize("seed", range(5))
def test_every_constraint_jacobian(seed: int) -> None:
    for c in all_constraints(seed):
        err = check_constraint(c)
        assert math.isfinite(err)


def test_every_constraint_type_is_covered() -> None:
    """The registry is the authority: a new type in the core shows up here and must be exercised.

    Except one whose kernel belongs to a *curve definition* rather than to its type — there is no
    fixture for it until the binding can declare a curve family, and it says so by publishing a
    kernel of -1.  Keying on that rather than on a name means a second such type is covered by
    this rule too, and anything else still has to be exercised.
    """
    covered = {type(c).__name__ for c in all_constraints(0)}
    want = {n for n, t in CONSTRAINT_TYPES.items() if t.kernel_id >= 0}
    assert covered == want


def test_scalar_kernel_matches_the_compiled_system() -> None:
    """The one-row view of a kernel and the vectorized block must agree."""
    sk = examples.rect_fillets()
    s = System(sk)
    z = s.z0()
    r = s.residuals(z)
    for c in sk.constraints:
        off = s.row_of(c)
        np.testing.assert_allclose(r[off : off + c.n_residuals],
                                   c.residual(c.local_values()), atol=1e-12)
    s.dispose()


def test_shared_param_assembly() -> None:
    """A param used twice in one constraint (l1 and l2 share a point) must sum contributions."""
    sk = Sketch()
    a, b, c = sk.point(0, 0), sk.point(1, 0.2), sk.point(2, 1)
    l1, l2 = sk.line(a, b), sk.line(b, c)
    sk.add(C.Perpendicular(l1, l2), C.EqualLength(l1, l2), C.Angle(l1, l2, 0.3))
    check_sketch(sk)


@pytest.mark.parametrize("name", list(examples.EXAMPLES))
def test_example_sketch_jacobians(name: str) -> None:
    check_sketch(examples.EXAMPLES[name]())


def test_fixed_params_dropped_from_jacobian() -> None:
    sk = examples.rect_fillets()
    sys_ = System(sk)
    assert sys_.n_free == len(sk.params) - 2
    assert sys_.jacobian_dense(sys_.z0()).shape == (sk.n_residuals(), sys_.n_free)
    sys_.dispose()
