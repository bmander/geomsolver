import math

import pytest

from gcs import examples, io, solve
from gcs import constraints as C
from gcs.model import Sketch


def test_roundtrip_all_examples() -> None:
    for name, make in examples.EXAMPLES.items():
        sk = make()
        s = io.dumps(sk)
        sk2 = io.loads(s)
        assert io.dumps(sk2) == s, name
        assert len(sk2.constraints) == len(sk.constraints)
        assert solve(sk2).success


def test_without_removes_dependents() -> None:
    sk = examples.rect_fillets()
    n_arc_c = sum(isinstance(c, C.TangentArcLine) for c in sk.constraints)
    sk2 = io.without(sk, entities=[sk.arcs[0].center])
    assert len(sk2.arcs) == 3
    assert sum(isinstance(c, C.TangentArcLine) for c in sk2.constraints) == n_arc_c - 2
    io.dumps(sk2)  # every kept constraint references only live entities


def test_without_line_keeps_points() -> None:
    sk = examples.truss(3)
    n_pts = len(sk.points)
    sk2 = io.without(sk, entities=[sk.lines[0]])
    assert len(sk2.points) == n_pts
    assert len(sk2.lines) == len(sk.lines) - 1


def test_every_constraint_type_has_spec_and_roundtrips() -> None:
    """Every concrete Constraint subclass declares a spec that reconstructs it."""
    from tests.test_jacobians import all_constraints

    seen = set()
    for c in all_constraints(0):
        seen.add(type(c))
        assert c.spec, type(c)
        c2 = type(c)(*c.args())
        assert c2.args() == c.args()
    assert seen >= {t for t in io.BY_NAME.values() if not t.__name__.startswith("_") and t is not C.Constraint} - {C._TwoLine}


def test_describe() -> None:
    sk = examples.rect_fillets()
    assert io.describe(sk.constraints[-1], sk) == "Distance(P6, P7, 40)"


def test_a_live_drag_never_reaches_the_document() -> None:
    """`soft` is not part of the JSON, so a soft constraint saved mid-drag would come back
    as a real one — a DragTarget as geometry, a RadiusDrag's pull as a dimension the user
    never typed.  Snapshots (undo) go through the same path."""
    from gcs.solve import Drag, RadiusDrag

    sk = examples.slotted_link()
    n = len(io.to_dict(sk)["constraints"])
    d = Drag(sk, sk.points[1], 1.0, 2.0)
    r = RadiusDrag(sk, sk.circles[0], 9.0)
    try:
        assert len(io.to_dict(sk)["constraints"]) == n      # to_dict only ever adds
        assert len(io.loads(io.dumps(sk)).constraints) == len(sk.constraints) - 2
    finally:
        r.end()
        d.end()


def test_a_soft_radius_is_not_a_known_dimension() -> None:
    """The decomposition must not treat a RadiusDrag's pull as a dimensioned radius: that
    would change which clusters are rigid while the user is mid-drag."""
    from gcs.cgraph import known_radii
    from gcs.solve import RadiusDrag

    sk = Sketch()
    c = sk.circle(sk.point(0, 0, fixed=True), 10.0)
    d = RadiusDrag(sk, c, 10.0)
    try:
        assert known_radii(sk) == {}
    finally:
        d.end()


def test_drawn_bounds_covers_curves_not_just_points() -> None:
    """`bbox` is points-only (it defines `extent`, and through it the solver's residual
    scale); `drawn_bounds` is what a "fit the view" wants — a circle reaches past its
    centre, and an arc past its endpoints."""
    sk = Sketch()
    sk.circle(sk.point(0, 0), 10.0)
    assert sk.bbox() == (0.0, 0.0, 0.0, 0.0)
    assert sk.drawn_bounds() == (-10.0, -10.0, 10.0, 10.0)

    sk2 = Sketch()
    c = sk2.point(0, 0)
    arc = sk2.arc(c, sk2.point(5, 0), sk2.point(0, 5))       # a quarter turn, no bulge past the ends
    assert arc.bounds() == pytest.approx((0.0, 0.0, 5.0, 5.0), abs=1e-12)
    arc.end.x.value, arc.end.y.value = -5.0, 0.0             # now a half turn through the top
    assert arc.bounds() == pytest.approx((-5.0, 0.0, 5.0, 5.0), abs=1e-12)
    arc.end.x.value, arc.end.y.value = 5.0, -1e-12           # nearly the full circle
    assert arc.bounds() == pytest.approx((-5.0, -5.0, 5.0, 5.0), abs=1e-9)


def test_three_point_arc_takes_the_sweep_through_the_third_point() -> None:
    """Start, end, and a point the arc must pass through: the circumcircle plus the sweep
    direction that actually contains that point."""
    sk = Sketch()
    a, b = sk.point(-5, 0), sk.point(5, 0)
    up = sk.arc_through(a, b, (0.0, 5.0))
    assert up is not None
    assert up.center.xy == pytest.approx((0.0, 0.0), abs=1e-12)
    assert up.radius.value == pytest.approx(5.0, abs=1e-12)
    # CCW from a=(-5,0) would sweep under, so a top-bulging arc has to start at b
    assert (up.start, up.end) == (b, a)
    a0, a1 = up.angles()
    assert a0 == pytest.approx(0.0, abs=1e-12) and a1 == pytest.approx(math.pi, abs=1e-12)

    sk2 = Sketch()
    c, d = sk2.point(-5, 0), sk2.point(5, 0)
    down = sk2.arc_through(c, d, (0.0, -5.0))              # same chord, other side
    assert down is not None
    assert down.center.xy == pytest.approx((0.0, 0.0), abs=1e-12)
    assert (down.start, down.end) == (c, d)                # CCW from c sweeps under, as wanted
    b0, b1 = down.angles()
    assert b0 == pytest.approx(math.pi, abs=1e-12) and b1 == pytest.approx(2 * math.pi, abs=1e-12)

    # the arc it builds is consistent: both endpoints sit at the radius
    for arc in (up, down):
        for p in (arc.start, arc.end):
            assert math.dist(p.xy, arc.center.xy) == pytest.approx(arc.radius.value, abs=1e-12)


def test_three_point_arc_refuses_collinear_input() -> None:
    sk = Sketch()
    a, b = sk.point(0, 0), sk.point(10, 0)
    n = len(sk.points)
    assert sk.arc_through(a, b, (5.0, 0.0)) is None
    assert sk.arc_through(a, b, (20.0, 1e-12)) is None     # the test is scale-free, not absolute
    assert len(sk.points) == n                             # nothing was created
    assert sk.arc_through(a, b, (5.0, 0.01)) is not None   # a real, very flat arc is fine
