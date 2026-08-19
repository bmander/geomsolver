"""Reference sketches used by tests, benchmarks and the canvas."""

from __future__ import annotations

import math
import random
from collections.abc import Callable

import numpy as np

from gcs.constraints import (
    Coincident, Distance, EqualLength, EqualRadius, Horizontal, Radius, TangentArcLine, Vertical,
)
from gcs.model import Point, Sketch


def perturb(sk: Sketch, sigma: float, seed: int = 0) -> None:
    """Add seeded Gaussian noise to every free parameter (tests/benchmarks warm-start from here)."""
    rng = np.random.default_rng(seed)
    x = sk.get_x()
    free = sk.free_indices()
    x[free] += rng.normal(0, sigma, len(free))
    sk.set_x(x)


def rect_fillets(w: float = 100.0, h: float = 60.0, r: float = 10.0, perturb: float = 0.0) -> Sketch:
    """Rectangle w×h with four equal fillets of radius r. Fully constrained (0 DOF)."""
    sk = Sketch()
    rng = random.Random(0)

    def P(x: float, y: float, name: str) -> Point:  # noqa: N802
        return sk.point(x + rng.uniform(-perturb, perturb), y + rng.uniform(-perturb, perturb), name=name)

    bottom = sk.line(P(r, 0, "b1"), P(w - r, 0, "b2"))
    right = sk.line(P(w, r, "r1"), P(w, h - r, "r2"))
    top = sk.line(P(w - r, h, "t1"), P(r, h, "t2"))
    left = sk.line(P(0, h - r, "l1"), P(0, r, "l2"))
    # arcs share endpoints with the lines (CCW start -> end)
    a_br = sk.arc(P(w - r, r, "c_br"), bottom.p2, right.p1, name="a_br")
    a_tr = sk.arc(P(w - r, h - r, "c_tr"), right.p2, top.p1, name="a_tr")
    a_tl = sk.arc(P(r, h - r, "c_tl"), top.p2, left.p1, name="a_tl")
    a_bl = sk.arc(P(r, r, "c_bl"), left.p2, bottom.p1, name="a_bl")

    sk.add(Horizontal(bottom), Horizontal(top), Vertical(left), Vertical(right))
    for arc, l_in, l_out in ((a_br, bottom, right), (a_tr, right, top), (a_tl, top, left), (a_bl, left, bottom)):
        sk.add(TangentArcLine(arc, l_in, "start"), TangentArcLine(arc, l_out, "end"))
    sk.add(EqualRadius(a_br, a_tr), EqualRadius(a_br, a_tl), EqualRadius(a_br, a_bl))
    sk.add(Radius(a_bl, r))
    sk.add(Distance(bottom.p1, bottom.p2, w - 2 * r), Distance(left.p1, left.p2, h - 2 * r))
    a_bl.center.fix()
    return sk


def slotted_link(length: float = 80.0, r: float = 15.0, hole_r: float = 6.0) -> Sketch:
    """Obround slot with two concentric holes. Fully constrained (0 DOF)."""
    sk = Sketch()
    c1 = sk.point(0, 0, name="c1")
    c2 = sk.point(length, 0, name="c2")
    top = sk.line(sk.point(0, r, name="t1"), sk.point(length, r, name="t2"))
    bottom = sk.line(sk.point(length, -r, name="b1"), sk.point(0, -r, name="b2"))
    a_right = sk.arc(c2, bottom.p1, top.p2, name="a_r")
    a_left = sk.arc(c1, top.p1, bottom.p2, name="a_l")
    h1 = sk.circle(c1, hole_r, name="h1")
    h2 = sk.circle(c2, hole_r, name="h2")
    sk.add(
        TangentArcLine(a_right, bottom, "start"), TangentArcLine(a_right, top, "end"),
        TangentArcLine(a_left, top, "start"), TangentArcLine(a_left, bottom, "end"),
        EqualRadius(a_left, a_right), Radius(a_left, r),
        Radius(h1, hole_r), Radius(h2, hole_r),
        Distance(c1, c2, length), Horizontal(top),
    )
    c1.fix()
    return sk


def truss(bays: int = 8, span: float = 20.0, height: float = 15.0, dims: bool = True) -> Sketch:
    """Warren-style truss: bays+1 bottom nodes, bays top nodes, ~4·bays members.

    With dims=True every member gets a length constraint → rigid, 0 DOF after
    fixing the first node and making the first chord horizontal.  bays=8 gives
    17 points + 31 lines (the "~30-entity" exit criterion).
    """
    sk = Sketch()
    bot = [sk.point(i * span, 0, name=f"b{i}") for i in range(bays + 1)]
    top = [sk.point((i + 0.5) * span, height, name=f"t{i}") for i in range(bays)]
    members = []
    for i in range(bays):
        members.append(sk.line(bot[i], bot[i + 1]))
        members.append(sk.line(bot[i], top[i]))
        members.append(sk.line(top[i], bot[i + 1]))
        if i + 1 < bays:
            members.append(sk.line(top[i], top[i + 1]))
    if dims:
        for m in members:
            sk.add(Distance(m.p1, m.p2, m.length()))
    sk.add(Horizontal(members[0]))
    bot[0].fix()
    return sk


def polygon_chain(n: int = 12, radius: float = 50.0) -> Sketch:
    """Under-constrained: closed n-gon of equal-length edges via Coincident joints.

    Deliberately closes the EqualLength cycle (e0=e1, ..., e_{n-1}=e0), so one
    equation is redundant-but-consistent — a Stage 2 diagnosis test case."""
    sk = Sketch()
    lines = []
    for i in range(n):
        a0, a1 = 2 * math.pi * i / n, 2 * math.pi * (i + 1) / n
        lines.append(sk.line_xy(radius * math.cos(a0), radius * math.sin(a0),
                                radius * math.cos(a1), radius * math.sin(a1), name=f"e{i}"))
    for i in range(n):
        sk.add(Coincident(lines[i].p2, lines[(i + 1) % n].p1))
        sk.add(EqualLength(lines[i], lines[(i + 1) % n]))
    lines[0].p1.fix()
    return sk


EXAMPLES: dict[str, Callable[..., Sketch]] = {
    "rect_fillets": rect_fillets,
    "slotted_link": slotted_link,
    "truss": truss,
    "polygon_chain": polygon_chain,
}
