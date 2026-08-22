"""Parametric curves through the binding: the proxy, the contacts, and the document."""

from __future__ import annotations

import math

import pytest

from gcs import constraints as C
from gcs import examples, io, solve
from gcs.model import Sketch, Spline


def wave(n: int = 6) -> tuple[Sketch, Spline]:
    sk = Sketch()
    ctrl = [sk.point(i * 10.0, 12.0 if i % 2 else 0.0) for i in range(n)]
    sp = sk.spline(ctrl)
    assert sp is not None
    return sk, sp


def test_a_spline_is_a_control_polygon_of_ordinary_points() -> None:
    sk, sp = wave()
    assert [p.index for p in sp.ctrl] == [p.index for p in sk.points]
    assert sp.knots == (0.0, 0.0, 0.0, 0.0, 1.0, 2.0, 3.0, 3.0, 3.0, 3.0)
    assert sp.domain == (0.0, 3.0)
    # a clamped curve starts at its first control point and ends at its last
    assert sp.point_at(0.0) == pytest.approx(sk.points[0].xy)
    assert sp.point_at(3.0) == pytest.approx(sk.points[-1].xy)


def test_too_few_control_points_is_not_a_curve() -> None:
    sk = Sketch()
    assert sk.spline([sk.point(0.0, 0.0), sk.point(1.0, 1.0), sk.point(2.0, 0.0)]) is None
    assert sk.splines == []


def test_the_polyline_lands_on_the_curve_and_follows_the_zoom() -> None:
    _, sp = wave()
    coarse, fine = sp.polyline(1.0), sp.polyline(0.01)
    assert len(fine) > len(coarse)
    for x, y in fine:
        assert sp.closest(x, y)[1] < 1e-6


def test_a_contact_owns_one_unknown_and_reads_as_a_number() -> None:
    sk, sp = wave()
    p = sk.point(21.0, 30.0)
    c = C.PointOnSpline(p, sp)
    n_before = len(sk.params)
    sk.add(c)
    assert len(sk.params) == n_before + 1
    assert c.owned_params() == ["t"]
    assert isinstance(c.t, float)
    with pytest.raises(AttributeError):
        c.t = 0.5          # type: ignore[misc]  — the solver moves it, nobody states it


def test_a_point_is_pulled_onto_the_curve() -> None:
    sk, sp = wave()
    for q in sp.ctrl:
        q.fix()
    p = sk.point(21.0, 30.0)
    sk.add(C.PointOnSpline(p, sp))
    assert solve(sk).success
    assert sp.closest(*p.xy)[1] < 1e-9


def test_a_line_is_made_tangent_to_the_curve() -> None:
    """What tangency *means* is checked in the Rust test, where the kernel lives; here the point
    is that the binding reaches it — the core says the constraint holds, and the contact the
    proxy hands back is a real point of the curve."""
    sk, sp = wave()
    for q in sp.ctrl:
        q.fix()
    ln = sk.line(sk.point(0.0, -20.0), sk.point(50.0, -20.0))
    c = C.SplineTangentLine(sp, ln)
    sk.add(c)
    assert solve(sk).success
    assert c.error() < 1e-6
    assert sp.closest(*sp.point_at(c.t))[1] < 1e-9


def test_a_contact_stays_on_the_drawn_curve() -> None:
    """The basis is a polynomial and evaluates past the knot vector; a tangency out there is a
    perfectly good solution of the equations and a nonsense answer."""
    sk = Sketch()
    ctrl = [sk.point(*xy) for xy in [(0.0, 0.0), (0.0, 10.0), (10.0, 10.0), (10.0, 0.0)]]
    sp = sk.spline(ctrl)
    assert sp is not None
    for q in sp.ctrl:
        q.fix()
    ln = sk.line(sk.point(-6.0, 0.0), sk.point(-6.0, 10.0))
    c = C.SplineTangentLine(sp, ln)
    sk.add(c)
    assert solve(sk).success
    t0, t1 = sp.domain
    assert t0 - 1e-12 <= c.t <= t1 + 1e-12


def test_a_document_keeps_its_curves_and_where_they_are_touched() -> None:
    sk, sp = wave(7)
    p = sk.point(21.0, 30.0)
    c = C.PointOnSpline(p, sp)
    sk.add(c)
    assert solve(sk).success
    text = io.dumps(sk)
    back = io.loads(text)
    assert len(back.splines) == 1
    assert back.splines[0].knots == sp.knots
    (c2,) = [x for x in back.constraints if type(x).__name__ == "PointOnSpline"]
    assert c2.t == pytest.approx(c.t)
    assert io.dumps(back) == text


def test_copying_a_curve_takes_its_contacts() -> None:
    sk, sp = wave(5)
    p = sk.point(21.0, 30.0)
    sk.add(C.PointOnSpline(p, sp))
    clip = io.copy(sk, [sp, p])
    assert len(clip.splines) == 1
    assert [type(c).__name__ for c in clip.constraints] == ["PointOnSpline"]


def test_the_follower_example_solves_and_diagnoses() -> None:
    sk = examples.spline_follower()
    assert solve(sk).success
    from gcs.diagnose import diagnose

    d = diagnose(sk)
    assert not d.conflicts
    assert d.dof > 0            # a curve carries its own shape: there is plenty left to drag


def test_deleting_a_control_point_shortens_the_curve() -> None:
    sk, sp = wave(7)
    victim = sp.ctrl[3]
    out = io.without(sk, entities=[victim])
    assert len(out.splines) == 1
    assert len(out.splines[0].ctrl) == 6


def test_inserting_a_control_point_does_not_move_the_curve() -> None:
    sk, sp = wave(6)
    t0, t1 = sp.domain
    before = [sp.point_at(t0 + (t1 - t0) * k / 40) for k in range(41)]
    made = sp.insert_control(t0 + (t1 - t0) * 0.4)
    assert made is not None
    assert len(sp.ctrl) == 7
    assert sp.domain == (t0, t1)
    for k, (x, y) in enumerate(before):
        gx, gy = sp.point_at(t0 + (t1 - t0) * k / 40)
        assert math.hypot(gx - x, gy - y) < 1e-9


def test_a_curve_through_points_passes_through_them() -> None:
    pts = [(0.0, 0.0), (10.0, 20.0), (30.0, 5.0), (50.0, 25.0), (70.0, 0.0)]
    sk = Sketch()
    sp = sk.spline_through(pts)
    assert sp is not None
    assert len(sp.ctrl) == len(pts)
    for x, y in pts:
        assert sp.closest(x, y)[1] < 1e-9
    assert sk.spline_through(pts[:3]) is None


def test_a_curve_fitted_to_constrained_points_is_fully_constrained() -> None:
    from gcs.diagnose import diagnose

    pts = [(0.0, 0.0), (10.0, 20.0), (30.0, 5.0), (50.0, 25.0), (70.0, 0.0)]
    sk = Sketch()
    held = [sk.point(x, y, fixed=True) for x, y in pts]
    sp = sk.spline_through(pts, held)
    assert sp is not None
    d = diagnose(sk)
    assert d.dof == 0
    assert d.n_redundant == 0
    assert solve(sk).success
    for x, y in pts:
        assert sp.closest(x, y)[1] < 1e-9
    # a fit that holds nothing keeps the freedom of its own control polygon
    free = Sketch()
    assert free.spline_through(pts) is not None
    assert diagnose(free).dof == 2 * len(pts)


def test_a_pin_survives_the_document() -> None:
    from gcs.diagnose import diagnose

    pts = [(0.0, 0.0), (10.0, 20.0), (30.0, 5.0), (50.0, 25.0)]
    sk = Sketch()
    sk.spline_through(pts, [sk.point(x, y, fixed=True) for x, y in pts])
    text = io.dumps(sk)
    back = io.loads(text)
    assert diagnose(back).dof == 0, "the pins were lost on the way through the document"
    assert io.dumps(back) == text
