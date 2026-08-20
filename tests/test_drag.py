"""Stage 5 torture suite: recorded drag trajectories must keep constraints satisfied, move
continuously (no solution jumps), keep/flag chirality, and branches must survive save/load."""

from __future__ import annotations

import math

import numpy as np
import pytest

from gcs import constraints as C
from gcs import examples, io
from gcs.decompose import PlanDrag, PlanSolver
from gcs.model import Point, Sketch
from gcs.solve import Drag, RadiusDrag, System, solve


def circle_path(cx: float, cy: float, r: float, n: int = 40) -> list[tuple[float, float]]:
    return [(cx + r * math.cos(2 * math.pi * i / n), cy + r * math.sin(2 * math.pi * i / n))
            for i in range(n + 1)]


def run_trajectory(sk: Sketch, p: Point, path: list[tuple[float, float]],
                   jump_factor: float = 10.0, plan: bool = True) -> dict:
    """Drag p along path; assert constraints hold after every frame and nothing teleports."""
    sys_ = System(sk)
    drag = PlanDrag(sk, p, *p.xy) if plan else Drag(sk, p, *p.xy)
    prev = sk.get_x()
    max_ratio = 0.0
    for (x, y) in path:
        cursor_step = math.hypot(x - p.xy[0], y - p.xy[1])
        res = drag.move(x, y)
        assert res.success, res
        sys_.refresh_consts()
        assert sys_.max_hard_residual() <= 1e-6 * sys_.scale
        now = sk.get_x()
        moved = float(np.abs(now - prev).max())
        if cursor_step > 1e-9:
            max_ratio = max(max_ratio, moved / cursor_step)
            assert moved <= jump_factor * cursor_step + 1e-9, \
                f"jump: moved {moved:.3g} for cursor step {cursor_step:.3g}"
        prev = now
    on_plan = plan and drag.usable
    flips = list(drag.flips)
    drag.end()
    sys_.dispose()
    return {"max_ratio": max_ratio, "flips": flips, "plan": on_plan}


def test_floating_truss_rides_along_with_the_cursor() -> None:
    sk = examples.truss_floating(6)
    p = sk.points[3]
    info = run_trajectory(sk, p, circle_path(*p.xy, 15.0), jump_factor=8.0)
    assert info["plan"] and not info["flips"]


def test_under_constrained_rect_slides() -> None:
    sk = examples.rect_fillets_under()
    p = sk.lines[0].p2
    x0, y0 = p.xy
    path = [(x0 + dx, y0) for dx in list(np.linspace(0, 40, 20)) + list(np.linspace(40, -20, 30))]
    info = run_trajectory(sk, p, path)
    assert not info["flips"]
    assert p.x.value == pytest.approx(x0 - 20, abs=1e-6)


def test_fully_constrained_point_stays_put_without_jumping() -> None:
    sk = examples.rect_fillets()
    p = sk.lines[0].p2
    x0 = sk.get_x()
    run_trajectory(sk, p, circle_path(*p.xy, 20.0))
    np.testing.assert_allclose(sk.get_x(), x0, atol=1e-6)


def test_polygon_vertex_far_drag_is_continuous() -> None:
    sk = examples.polygon_chain(10)
    p = sk.points[5]
    path = [(p.xy[0] + 60 * t, p.xy[1] + 30 * t) for t in np.linspace(0, 1, 12)]
    info = run_trajectory(sk, p, path, jump_factor=6.0)
    assert not info["plan"]     # EqualLength is not decomposable: numeric path with continuation


def test_fully_constrained_apex_never_jumps_across_the_base() -> None:
    """Dragging a rigid triangle's apex 'through' the base must not teleport it to the mirror
    root: the point is fully constrained, it stays on its branch (flipping is an explicit action,
    not a drag side effect)."""
    sk = Sketch()
    a, b = sk.point(0, 0, fixed=True), sk.point(10, 0, fixed=True)
    c = sk.point(5, 4)
    sk.add(C.Distance(a, c, 6), C.Distance(b, c, 6))
    d = PlanDrag(sk, c, *c.xy)
    # pinning the apex over-determines the sketch: the numeric path with guards takes over
    assert d.numeric and d.guard_triangles()
    ys = []
    for y in np.linspace(4, -4, 17):
        d.move(5, float(y))
        ys.append(c.y.value)
    flips = list(d.flips)
    d.end()
    assert not flips and min(ys) > 3.0 and max(ys) < 3.5


def test_guard_flags_an_unavoidable_crossing() -> None:
    """A point free on a circle dragged through a guarded triangle's base: the orientation must
    change; continuation/damping cannot avoid it, so the flip is recorded and flagged."""
    sk = Sketch()
    a, b = sk.point(0, 0, fixed=True), sk.point(10, 0, fixed=True)
    c = sk.point(5, 4)
    sk.add(C.Distance(a, c, 6))
    d = Drag(sk, c, *c.xy, guards=[(a, b, c)])
    msgs = []
    for y in np.linspace(4, -4, 9):
        msgs.append(d.move(5, float(y)).message)
    flips = list(d.flips)
    d.end()
    assert flips == [(a, b, c)] and any("flip" in m for m in msgs)
    assert c.y.value < 0


def test_branches_survive_save_load_and_replay_sticky() -> None:
    sk = Sketch()
    a, b = sk.point(0, 0, fixed=True), sk.point(10, 0, fixed=True)
    c = sk.point(5, 4)
    sk.add(C.Distance(a, c, 6), C.Distance(b, c, 6))
    ps = PlanSolver(sk, sticky=True)
    ps.solve()
    ps.flip(c)
    ps.solve()
    assert c.y.value < 0
    sk.update_branches(ps.branches())
    sk2 = io.loads(io.dumps(sk))
    assert sk2.branches == sk.branches
    sk2.points[2].y.value = 4.0                                 # sketch moved to the other side...
    PlanSolver(sk2, sticky=True).solve()
    assert sk2.points[2].y.value < 0                            # ...the recorded root wins


def test_continuation_subdivides_large_moves() -> None:
    sk = examples.truss_floating(4)
    p = sk.points[2]
    d = PlanDrag(sk, p, *p.xy)
    res = d.move(p.xy[0] + 200, p.xy[1])                       # far beyond one increment
    d.end()
    assert res.success and res.nfev > 1     # nfev = number of increments on the plan path


def test_flip_survives_a_later_solve_by_the_same_cached_plan() -> None:
    """Root choices are document state: a plan cached per topology must not replay the old branch
    after a flip (the app keeps one PlanSolver per topology and re-solves on every edit)."""
    sk = Sketch()
    a, b = sk.point(0, 0, fixed=True), sk.point(10, 0, fixed=True)
    c = sk.point(5, 4)
    sk.add(C.Distance(a, c, 6), C.Distance(b, c, 6))
    ps = PlanSolver(sk, sticky=True)          # the cached solver
    ps.solve()
    assert c.y.value > 0
    assert ps.flip(c) == 1
    ps.solve()
    assert c.y.value < 0 and sk.branches
    for _ in range(3):                        # every later solve keeps the chosen root
        ps.solve()
        assert c.y.value < 0


def test_radius_drag_resizes_a_free_circle() -> None:
    """Dragging the edge of an unconstrained circle changes its radius."""
    sk = Sketch()
    c = sk.circle(sk.point(0, 0, fixed=True), 10.0)
    d = RadiusDrag(sk, c, c.radius.value)
    try:
        for target in (25.0, 4.0, 12.5):
            res = d.move(target)
            assert res.success
            assert c.radius.value == pytest.approx(target, abs=1e-6)
    finally:
        d.end()
    assert not any(x.soft for x in sk.constraints)


def test_radius_drag_leaves_a_dimensioned_circle_alone() -> None:
    """A dimensioned radius does not follow the cursor — the polish wins, as with points."""
    sk = Sketch()
    c = sk.circle(sk.point(0, 0, fixed=True), 10.0)
    sk.add(C.Radius(c, 10.0))
    d = RadiusDrag(sk, c, c.radius.value)
    try:
        d.move(30.0)
    finally:
        d.end()
    assert c.radius.value == pytest.approx(10.0, abs=1e-6)


def test_radius_drag_carries_the_geometry_that_depends_on_it() -> None:
    """An arc's endpoints sit at its radius (intrinsic PointOnCircle), so resizing the arc has to
    move them with it."""
    sk = Sketch()
    c = sk.point(0, 0, fixed=True)
    arc = sk.arc(c, sk.point(10, 0), sk.point(0, 10))
    solve(sk)
    d = RadiusDrag(sk, arc, arc.radius.value)
    try:
        assert d.move(17.0).success
    finally:
        d.end()
    assert arc.radius.value == pytest.approx(17.0, abs=1e-6)
    for p in (arc.start, arc.end):
        assert math.dist(p.xy, arc.center.xy) == pytest.approx(17.0, abs=1e-6)


def test_radius_drag_respects_an_equal_radius_chain() -> None:
    """EqualRadius makes the chain move together — the pull is soft, the chain is not."""
    sk = Sketch()
    a = sk.circle(sk.point(0, 0, fixed=True), 10.0)
    b = sk.circle(sk.point(40, 0, fixed=True), 10.0)
    sk.add(C.EqualRadius(a, b))
    d = RadiusDrag(sk, a, a.radius.value)
    try:
        assert d.move(18.0).success
    finally:
        d.end()
    assert a.radius.value == pytest.approx(18.0, abs=1e-6)
    assert b.radius.value == pytest.approx(18.0, abs=1e-6)
