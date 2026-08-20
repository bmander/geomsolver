"""Document I/O, deletion by rebuild, and the model's own geometry helpers."""

from __future__ import annotations

import math

import pytest

from gcs import constraints as C
from gcs import examples, io, solve
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
    """Every constraint type declares a spec that reconstructs it."""
    from tests.test_jacobians import all_constraints

    seen = set()
    for c in all_constraints(0):
        seen.add(type(c))
        assert c.spec, type(c)
        c2 = type(c)(*c.args())
        assert c2.args() == c.args()
    assert seen == set(io.BY_NAME.values())


def test_describe() -> None:
    sk = examples.rect_fillets()
    assert io.describe(sk.constraints[-1], sk) == "Distance(P6, P7, 40)"


def test_a_live_drag_never_reaches_the_document() -> None:
    """`soft` is not part of the JSON, so a soft constraint saved mid-drag would come back as a
    real one — a DragTarget as geometry, a RadiusDrag's pull as a dimension the user never typed.
    Snapshots (undo) go through the same path."""
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
    """The decomposition must not treat a RadiusDrag's pull as a dimensioned radius: that would
    change which clusters are rigid while the user is mid-drag."""
    from gcs.decompose import build_graph
    from gcs.solve import RadiusDrag

    sk = Sketch()
    c = sk.circle(sk.point(0, 0, fixed=True), 10.0)
    d = RadiusDrag(sk, c, 10.0)
    try:
        assert build_graph(sk)["knownRadius"] == {}
    finally:
        d.end()


def test_drawn_bounds_covers_curves_not_just_points() -> None:
    """`bbox` is points-only (it defines `extent`, and through it the solver's residual scale);
    `drawn_bounds` is what a "fit the view" wants."""
    sk = Sketch()
    sk.circle(sk.point(0, 0), 10.0)
    assert sk.bbox() == (0.0, 0.0, 0.0, 0.0)
    assert sk.drawn_bounds() == (-10.0, -10.0, 10.0, 10.0)

    sk2 = Sketch()
    c = sk2.point(0, 0)
    arc = sk2.arc(c, sk2.point(5, 0), sk2.point(0, 5))       # a quarter turn, no bulge past ends
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


def test_rectangle_is_rigid_up_to_its_five_degrees_of_freedom() -> None:
    """Three perpendiculars, not four: the fourth follows from the other three, so adding it
    would leave every rectangle over-constrained by one equation."""
    from gcs.diagnose import diagnose

    sk = Sketch()
    lines = sk.rectangle_xy(0, 0, 40, 25)
    assert len(lines) == 4
    assert len(sk.points) == 4                      # corners are shared, not duplicated
    d = diagnose(sk)
    assert d.n_redundant == 0
    assert d.dof == 5                               # position, rotation, width, height
    sk.perturb(3.0, seed=1)
    assert solve(sk).success
    for i in range(4):
        u = lines[i].direction()
        v = lines[(i + 1) % 4].direction()
        assert u[0] * v[0] + u[1] * v[1] == pytest.approx(0.0, abs=1e-6)


def test_construction_flag_round_trips() -> None:
    sk = examples.slotted_link()
    sk.lines[0].construction = True
    sk.arcs[0].construction = True
    sk.circles[0].construction = True
    back = io.loads(io.dumps(sk))
    assert [l.construction for l in back.lines] == [l.construction for l in sk.lines]
    assert [a.construction for a in back.arcs] == [a.construction for a in sk.arcs]
    assert [c.construction for c in back.circles] == [c.construction for c in sk.circles]
    assert io.dumps(back) == io.dumps(sk)


def test_distance_between_covers_every_pair_of_kinds() -> None:
    from gcs.model import distance_between as dist

    sk = Sketch()
    o, p = sk.point(0, 0), sk.point(3, 4)
    horiz = sk.line(sk.point(0, 10), sk.point(20, 10))          # y = 10
    slant = sk.line(sk.point(0, 0), sk.point(10, 10))           # y = x, crosses horiz
    para = sk.line(sk.point(-5, 16), sk.point(5, 16))           # y = 16, parallel to horiz
    c1 = sk.circle(sk.point(0, 0), 2.0)
    c2 = sk.circle(sk.point(10, 0), 3.0)
    inner = sk.circle(sk.point(0, 0), 0.5)

    assert dist(o, p) == pytest.approx(5.0)
    assert dist(p, o) == pytest.approx(5.0)                     # symmetric
    assert dist(o, horiz) == pytest.approx(10.0)                # perpendicular to the line
    assert dist(horiz, o) == pytest.approx(10.0)
    assert dist(o, c2) == pytest.approx(7.0)                    # to the curve, not the centre
    assert dist(sk.point(10, 0), c2) == pytest.approx(3.0)      # from inside, still the curve
    assert dist(horiz, para) == pytest.approx(6.0)              # parallel gap
    assert dist(horiz, slant) == pytest.approx(0.0)             # crossing lines meet
    assert dist(horiz, c1) == pytest.approx(8.0)                # line to circle
    assert dist(slant, c1) == pytest.approx(0.0)                # the line cuts it
    assert dist(c1, c2) == pytest.approx(5.0)                   # outside each other
    assert dist(c1, inner) == pytest.approx(1.5)                # one inside the other
    assert dist(c1, sk.circle(sk.point(3, 0), 2.0)) == pytest.approx(0.0)   # overlapping


def test_a_dangling_reference_is_an_error_not_a_crash() -> None:
    """A document is untrusted input: a bad index has to come back as an exception, with the
    interpreter still standing."""
    for bad in [
        '{"points":[{"x":0,"y":0}],"arcs":[{"center":7,"start":0,"end":0,"r":1}]}',
        '{"points":[{"x":0,"y":0}],"lines":[{"p1":0,"p2":4}]}',
        '{"points":[{"x":0,"y":0}],"circles":[{"center":-1,"r":1}]}',
        '{"points":[{"x":0,"y":0}],'
        '"constraints":[{"type":"Horizontal","args":[["line",0]]}]}',
    ]:
        with pytest.raises(ValueError, match="out of range"):
            io.loads(bad)
    assert io.loads('{"points":[{"x":1,"y":2}]}').points[0].xy == (1.0, 2.0)


def test_a_core_panic_comes_back_as_an_error() -> None:
    """`guard` in the FFI is the panic boundary: a bad index sets the last error and returns a
    neutral value, rather than aborting the host process."""
    from gcs import _ffi
    from gcs._ffi import lib

    sk = Sketch()
    sk.point(0, 0)
    v = lib.gcs_param_value(sk._h, 99)
    assert math.isnan(v)
    assert "out of range" in _ffi.last_error()
    assert sk.points[0].xy == (0.0, 0.0)          # the sketch is untouched and still usable


def test_topology_key_distinguishes_one_constraint_from_another_of_the_same_type() -> None:
    """A front end caches compiled plans against this.  Counts and type names alone are not
    enough: delete one Distance and add another and both are identical."""
    sk = Sketch()
    p = [sk.point(i * 10, 0) for i in range(4)]
    a = C.Distance(p[0], p[1], 50)
    sk.add(a)
    k1 = sk.topology_key()
    sk.remove(a)
    sk.add(C.Distance(p[2], p[3], 20))
    assert sk.topology_key() != k1

    k2 = sk.topology_key()
    p[0].fix(True)
    assert sk.topology_key() != k2
    p[0].fix(False)
    assert sk.topology_key() == k2
    p[0].x.value = 99
    assert sk.topology_key() == k2       # moving geometry is not a topology change
