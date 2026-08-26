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
    # a type whose kernel belongs to a curve definition has no fixture yet — see
    # `test_every_constraint_type_is_covered` for why that is keyed on the kernel and not a name
    assert seen == {t for t in io.BY_NAME.values() if t.kernel_id >= 0}


def test_describe() -> None:
    sk = examples.rect_fillets()
    # the height, stated across the rectangle: `distance(t1, b2) == h`
    assert io.describe(sk.constraints[-1], sk) == "Distance(P4, P1, 60)"


def test_callouts_cover_every_dimension() -> None:
    """Every constraint carrying a Length or an Angle comes back as a drafting figure."""
    for name, make in examples.EXAMPLES.items():
        sk = make()
        cs = io.callouts(sk, 0.2)
        dims = [c for c in sk.user_constraints() if c.dimensions()]
        assert len(cs["items"]) == len(dims), name
        assert cs["font"] > 0 and cs["arrow"] > 0 and cs["barb"] > 0, name
        assert {k["id"] for k in cs["items"]} == {c._id for c in dims}, name
        for k in cs["items"]:
            assert k["text"], name
            assert k["solid"] or k["arcs"], name       # something for the number to ride on
            assert all(math.isfinite(v) for v in k["anchor"]), name


def test_callouts_are_screen_constant() -> None:
    """`unit` is the world length of a pixel, so halving the zoom doubles the stand-off."""
    sk = Sketch()
    a, b = sk.point(0, 0, fixed=True), sk.point(60, 0)
    sk.add(C.Distance(a, b, 60))
    off = [io.callouts(sk, u)["items"][0]["solid"][0][0][1] for u in (1.0, 2.0)]
    assert off[1] == pytest.approx(2 * off[0])


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


def test_set_x_refuses_a_vector_that_is_not_this_sketchs() -> None:
    a, b = Sketch(), Sketch()
    a.point(1, 2)
    a.point(3, 4)
    b.point(9, 9)
    with pytest.raises(ValueError):
        b.set_x(a.get_x())
    assert b.points[0].xy == (9.0, 9.0)


def test_angle_between_and_on_radius_are_the_core_s() -> None:
    """Two pieces of geometry the front ends were each doing for themselves: the current angle a
    dimension dialog offers, and where the third click of a centre-start-end arc lands."""
    from gcs.model import angle_between, on_radius

    sk = Sketch()
    o = sk.point(0, 0)
    east = sk.line(o, sk.point(10, 0))
    north = sk.line(o, sk.point(0, 10))
    assert angle_between(east, north) == pytest.approx(math.pi / 2)
    assert angle_between(north, east) == pytest.approx(-math.pi / 2)     # signed

    assert on_radius(0, 0, 3, 4, 10) == pytest.approx((6.0, 8.0))
    assert on_radius(1, 1, 1, 1, 5) is None                              # no direction


def test_callout_placements_survive_a_save() -> None:
    """A dimension dragged into place stays there through a save, and can be put back."""
    sk = examples.rect_fillets()
    dim = next(c for c in sk.user_constraints() if isinstance(c, C.Radius))

    def anchor(s: Sketch, c: C.Constraint) -> list[float]:
        return next(k for k in io.callouts(s, 0.2)["items"] if k["id"] == c._id)["anchor"]

    home = anchor(sk, dim)
    # take hold of the callout where it is, and put it down a good way off
    grip = io.callout_grab(dim, 0.2, home[0], home[1])
    assert grip is not None
    assert io.callout_drag(dim, home[0] + 30.0, home[1] + 25.0, grip)
    moved = anchor(sk, dim)
    assert moved != pytest.approx(home, abs=1e-6)

    sk2 = io.loads(io.dumps(sk))
    dim2 = next(c for c in sk2.user_constraints() if isinstance(c, C.Radius))
    assert anchor(sk2, dim2) == pytest.approx(moved, abs=1e-9)

    io.callout_reset(dim)
    assert anchor(sk, dim) == pytest.approx(home, abs=1e-9)


def test_callout_pick_finds_the_dimension_under_a_point() -> None:
    """The tolerance is in screen pixels, like every other size crossing this seam."""
    sk = examples.rect_fillets()
    for k in io.callouts(sk, 0.2)["items"]:
        hit = io.callout_pick(sk, 0.2, k["anchor"][0], k["anchor"][1], 4.0)
        assert hit is not None and hit._id == k["id"]
    assert io.callout_pick(sk, 0.2, -1e4, -1e4, 4.0) is None


def test_copy_and_paste_round_trip() -> None:
    """A clipboard is a sketch: what a copy keeps is what deleting the rest would have kept."""
    sk = examples.rect_fillets()
    picked = [sk.lines[0], sk.arcs[0]]
    clip = io.copy(sk, picked)
    rest = [e for e in sk.primitives() if e not in io.expand(picked)]
    assert io.dumps(clip) == io.dumps(io.without(sk, entities=rest))

    n_pts, n_cons = len(sk.points), len(sk.user_constraints())
    made = io.paste(sk, clip, 5.0, -3.0)
    assert len(made) == len(clip.primitives())
    assert len(sk.points) == n_pts + len(clip.points)
    assert len(sk.user_constraints()) == n_cons + len(clip.user_constraints())
    # moved by exactly the offset asked for, and independent of what it was copied from
    assert sk.points[n_pts].xy == pytest.approx((clip.points[0].x.value + 5.0,
                                                 clip.points[0].y.value - 3.0))
    assert solve(sk).success


def test_two_points_can_be_levelled_without_a_line() -> None:
    """The binding reaches the point-pair forms; what they mean is the Rust test's business."""
    sk = Sketch()
    a = sk.point(0.0, 0.0, fixed=True)
    b = sk.point(10.0, 4.0)
    c = sk.point(3.0, 9.0)
    sk.add(C.HorizontalPoints(a, b), C.VerticalPoints(a, c))
    assert solve(sk).success
    assert b.xy[1] == pytest.approx(a.xy[1])
    assert c.xy[0] == pytest.approx(a.xy[0])
    assert io.dumps(io.loads(io.dumps(sk))) == io.dumps(sk)


def test_the_run_and_the_rise_are_dimensions_of_their_own() -> None:
    """The binding reaches them, and the rule that says which one a callout states."""
    sk = Sketch()
    a = sk.point(0.0, 0.0, fixed=True)
    b = sk.point(10.0, 4.0)
    sk.add(C.HorizontalDistance(a, b, 30.0), C.VerticalDistance(a, b, -5.0))
    assert solve(sk).success
    assert b.xy == pytest.approx((30.0, -5.0))
    assert io.dumps(io.loads(io.dumps(sk))) == io.dumps(sk)
    # where the number is put is which of the three dimensions it is
    assert io.pair_dimension((0, 0), (40, 40), (-10, 50)) == "Distance"
    assert io.pair_dimension((0, 0), (40, 40), (20, 60)) == "HorizontalDistance"
    assert io.pair_dimension((0, 0), (40, 40), (60, 20)) == "VerticalDistance"
