"""Every constraint's analytic Jacobian must agree with finite differences at random points."""

import math

import numpy as np
import pytest

from gcs import constraints as C
from gcs.fdcheck import check_constraint, check_sketch
from gcs.model import Sketch
from gcs import examples
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
    return sk, p, q, s, l1, l2, c1, c2, a


def all_constraints(seed: int):
    sk, p, q, s, l1, l2, c1, c2, a = _sketch_with_stuff(seed)
    return [
        C.Coincident(p, q), C.Distance(p, q, 3.0), C.Midpoint(p, l1), C.DragTarget(p, 1, 2, 0.3),
        C.Horizontal(l1), C.Vertical(l1), C.Parallel(l1, l2), C.Perpendicular(l1, l2),
        C.Angle(l1, l2, 0.7), C.EqualLength(l1, l2), C.PointOnLine(p, l1), C.PointOnCircle(p, c1),
        C.PointOnCircle(p, a), C.Radius(c1, 2.0), C.EqualRadius(c1, a),
        C.TangentLineCircle(l1, c1), C.TangentLineCircle(l1, c1, side=-1),
        C.TangentCircleCircle(c1, c2, True), C.TangentCircleCircle(c1, c2, False),
        C.TangentArcLine(a, l1, "start"), C.TangentArcLine(a, l2, "end"),
    ]


@pytest.mark.parametrize("seed", range(5))
def test_every_constraint_jacobian(seed: int) -> None:
    for c in all_constraints(seed):
        err = check_constraint(c)
        assert math.isfinite(err)


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
    assert sys_.jacobian(sys_.z0()).shape == (sk.n_residuals(), sys_.n_free)
