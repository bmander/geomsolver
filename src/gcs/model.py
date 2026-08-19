"""Parameters, primitives and the Sketch container.

Every scalar degree of freedom is a `Param`.  Primitives are thin bundles of
Params; the Sketch owns the ordered list of Params (its parameter vector) and
the ordered list of Constraints.  Ordering is deterministic by construction —
insertion order, never hashing — so identical edits give bit-identical solves.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Iterable, NamedTuple, Sequence

import numpy as np
import numpy.typing as npt

if TYPE_CHECKING:
    from gcs.constraints import Constraint

Vec = npt.NDArray[np.float64]
Box = tuple[float, float, float, float]      # (xmin, ymin, xmax, ymax)


def _union(boxes: Iterable[Box]) -> Box | None:
    lo_x, lo_y, hi_x, hi_y = math.inf, math.inf, -math.inf, -math.inf
    for x0, y0, x1, y1 in boxes:
        lo_x, lo_y = min(lo_x, x0), min(lo_y, y0)
        hi_x, hi_y = max(hi_x, x1), max(hi_y, y1)
    return None if lo_x is math.inf or lo_x > hi_x else (lo_x, lo_y, hi_x, hi_y)


@dataclass(eq=False)
class Param:
    """One scalar unknown. `index` is its slot in the sketch's parameter vector."""

    value: float
    fixed: bool = False
    index: int = -1
    name: str = ""

    def __repr__(self) -> str:
        f = " fixed" if self.fixed else ""
        return f"Param({self.name or self.index}={self.value:.6g}{f})"


@dataclass(eq=False)
class Point:
    x: Param
    y: Param

    kind = "point"
    children: tuple[Point, ...] = ()

    @property
    def params(self) -> tuple[Param, ...]:
        return (self.x, self.y)

    @property
    def xy(self) -> tuple[float, float]:
        return (self.x.value, self.y.value)

    @property
    def is_fixed(self) -> bool:
        return self.x.fixed and self.y.fixed

    def fix(self, fixed: bool = True) -> None:
        self.x.fixed = fixed
        self.y.fixed = fixed

    def bounds(self) -> Box:
        return (self.x.value, self.y.value, self.x.value, self.y.value)


@dataclass(eq=False)
class Line:
    p1: Point
    p2: Point
    construction: bool = False   # reference geometry: drawn dashed, constrains like any other

    kind = "line"

    @property
    def children(self) -> tuple[Point, ...]:
        return (self.p1, self.p2)

    @property
    def params(self) -> tuple[Param, ...]:
        return self.p1.params + self.p2.params

    def direction(self) -> tuple[float, float]:
        return (self.p2.x.value - self.p1.x.value, self.p2.y.value - self.p1.y.value)

    def length(self) -> float:
        return math.hypot(*self.direction())

    def bounds(self) -> Box:
        return (min(self.p1.x.value, self.p2.x.value), min(self.p1.y.value, self.p2.y.value),
                max(self.p1.x.value, self.p2.x.value), max(self.p1.y.value, self.p2.y.value))


@dataclass(eq=False)
class Circle:
    center: Point
    radius: Param
    construction: bool = False

    kind = "circle"

    @property
    def children(self) -> tuple[Point, ...]:
        return (self.center,)

    @property
    def params(self) -> tuple[Param, ...]:
        return self.center.params + (self.radius,)

    def bounds(self) -> Box:
        cx, cy = self.center.xy
        r = abs(self.radius.value)
        return (cx - r, cy - r, cx + r, cy + r)


@dataclass(eq=False)
class Arc:
    """CCW arc from `start` to `end` about `center` with radius `radius`.

    Storing the radius as its own Param (rather than deriving it) lets Circle
    and Arc share every radius-based constraint.  The two intrinsic constraints
    |start-center|² = r² and |end-center|² = r² are added by Sketch.arc().
    Net DOF: 7 params - 2 = 5.
    """

    center: Point
    start: Point
    end: Point
    radius: Param
    construction: bool = False

    kind = "arc"

    @property
    def children(self) -> tuple[Point, ...]:
        return (self.center, self.start, self.end)

    @property
    def params(self) -> tuple[Param, ...]:
        return self.center.params + self.start.params + self.end.params + (self.radius,)

    def angles(self) -> tuple[float, float]:
        cx, cy = self.center.xy
        a0 = math.atan2(self.start.y.value - cy, self.start.x.value - cx)
        a1 = math.atan2(self.end.y.value - cy, self.end.x.value - cx)
        if a1 <= a0:
            a1 += 2 * math.pi
        return a0, a1

    def extremes(self) -> list[tuple[float, float]]:
        """The points that bound the drawn sweep: its two ends, plus every quarter-turn
        direction the sweep passes through.  Endpoints alone would under-report an arc that
        bulges past them."""
        cx, cy = self.center.xy
        r = abs(self.radius.value)
        a0, a1 = self.angles()
        def at(th: float) -> tuple[float, float]:
            return (cx + r * math.cos(th), cy + r * math.sin(th))

        out = [at(a0), at(a1)]
        quarter = math.pi / 2
        k = math.ceil(a0 / quarter)
        while k * quarter < a1:
            out.append(at(k * quarter))
            k += 1
        return out

    def bounds(self) -> Box:
        xs = [p[0] for p in self.extremes()]
        ys = [p[1] for p in self.extremes()]
        return (min(xs), min(ys), max(xs), max(ys))


Primitive = Point | Line | Circle | Arc


class ThreePointArc(NamedTuple):
    """The CCW arc through three points: centre, radius, and the sweep from `a0` to `a1`
    that passes through the third point.  `swapped` is True when that sweep runs from the
    *second* given point to the first."""

    cx: float
    cy: float
    r: float
    a0: float
    a1: float
    swapped: bool


def three_point_arc(ax: float, ay: float, bx: float, by: float,
                    cx: float, cy: float, tol: float = 1e-9) -> ThreePointArc | None:
    """Arc from (ax, ay) to (bx, by) passing through (cx, cy) — the circumcircle of the
    three, plus the sweep direction that actually contains the third point.  None if they
    are collinear (the test is on the sine of the angle, so it is scale-free)."""
    ux, uy = bx - ax, by - ay
    vx, vy = cx - ax, cy - ay
    cross = ux * vy - uy * vx
    if abs(cross) <= tol * math.hypot(ux, uy) * math.hypot(vx, vy):
        return None
    d = 2 * cross
    u2, v2 = ux * ux + uy * uy, vx * vx + vy * vy
    ox = ax + (vy * u2 - uy * v2) / d
    oy = ay + (ux * v2 - vx * u2) / d
    r = math.hypot(ax - ox, ay - oy)
    ta = math.atan2(ay - oy, ax - ox)
    tb = math.atan2(by - oy, bx - ox)

    def sweep(th: float) -> float:
        return (th - ta) % (2 * math.pi)

    to_b, to_c = sweep(tb), sweep(math.atan2(cy - oy, cx - ox))
    if to_c < to_b:                                    # the third point is on the a → b sweep
        return ThreePointArc(ox, oy, r, ta, ta + to_b, False)
    return ThreePointArc(ox, oy, r, tb, tb + (2 * math.pi - to_b), True)


def expand(ents: Iterable[Primitive]) -> list[Primitive]:
    """Entities plus their sub-entities (a line's endpoints, an arc's centre/ends)."""
    out: list[Primitive] = []
    for e in ents:
        out.append(e)
        out.extend(e.children)
    return out


@dataclass(eq=False)
class Sketch:
    params: list[Param] = field(default_factory=list)
    constraints: list[Constraint] = field(default_factory=list)
    points: list[Point] = field(default_factory=list)
    lines: list[Line] = field(default_factory=list)
    circles: list[Circle] = field(default_factory=list)
    arcs: list[Arc] = field(default_factory=list)
    branches: dict[str, int] = field(default_factory=dict)   # recorded root choices (Stage 5), persisted

    # -- construction -------------------------------------------------------

    def param(self, value: float, *, fixed: bool = False, name: str = "") -> Param:
        p = Param(float(value), fixed, len(self.params), name)
        self.params.append(p)
        return p

    def point(self, x: float, y: float, *, fixed: bool = False, name: str = "") -> Point:
        pt = Point(self.param(x, fixed=fixed, name=f"{name}.x"), self.param(y, fixed=fixed, name=f"{name}.y"))
        self.points.append(pt)
        return pt

    def line(self, p1: Point, p2: Point) -> Line:
        ln = Line(p1, p2)
        self.lines.append(ln)
        return ln

    def line_xy(self, x1: float, y1: float, x2: float, y2: float, name: str = "") -> Line:
        return self.line(self.point(x1, y1, name=f"{name}.p1"), self.point(x2, y2, name=f"{name}.p2"))

    def circle(self, center: Point, radius: float, name: str = "") -> Circle:
        c = Circle(center, self.param(radius, name=f"{name}.r"))
        self.circles.append(c)
        return c

    def arc(self, center: Point, start: Point, end: Point, name: str = "") -> Arc:
        from gcs.constraints import PointOnCircle

        cx, cy = center.xy
        r = math.hypot(start.x.value - cx, start.y.value - cy)
        a = Arc(center, start, end, self.param(r, name=f"{name}.r"))
        self.arcs.append(a)
        # intrinsic: both endpoints lie at the arc's radius
        self.add(PointOnCircle(start, a, intrinsic=True), PointOnCircle(end, a, intrinsic=True))
        return a

    def arc_through(self, start: Point, end: Point, through: tuple[float, float],
                    name: str = "") -> Arc | None:
        """Arc from `start` to `end` bulging through `through` — the three-point
        construction.  Creates the centre point; None if the three are collinear."""
        g = three_point_arc(*start.xy, *end.xy, *through)
        if g is None:
            return None
        centre = self.point(g.cx, g.cy, name=f"{name}.c")
        a, b = (end, start) if g.swapped else (start, end)
        return self.arc(centre, a, b, name=name)

    def rectangle(self, a: Point, x1: float, y1: float, name: str = "") -> list[Line]:
        """Four lines from corner `a` to the opposite corner (x1, y1), sharing corner points,
        with three perpendicular constraints.  `a` is an existing point, so a rectangle can
        start on geometry that is already there.

        Three perpendiculars, not four: the fourth follows (l3 ⟂ l2 ⟂ l1 ⟂ l0 already forces
        l3 ⟂ l0), so adding it would leave every rectangle over-constrained by one equation.
        What is left is the 5 DOF a rectangle has — position, rotation, width, height."""
        from gcs.constraints import Perpendicular

        x0, y0 = a.xy
        corners = [a, self.point(x1, y0, name=f"{name}.b"),
                   self.point(x1, y1, name=f"{name}.c"), self.point(x0, y1, name=f"{name}.d")]
        lines = [self.line(corners[i], corners[(i + 1) % 4]) for i in range(4)]
        self.add(*(Perpendicular(lines[i], lines[i + 1]) for i in range(3)))
        return lines

    def rectangle_xy(self, x0: float, y0: float, x1: float, y1: float, name: str = "") -> list[Line]:
        """`rectangle` starting from a fresh corner point — the `line_xy` of rectangles."""
        return self.rectangle(self.point(x0, y0, name=f"{name}.a"), x1, y1, name)

    def add(self, *constraints: Constraint) -> None:
        self.constraints.extend(constraints)

    def remove(self, constraint: Constraint) -> None:
        self.constraints.remove(constraint)

    # -- parameter vector ---------------------------------------------------

    def get_x(self) -> Vec:
        return np.array([p.value for p in self.params], dtype=np.float64)

    def set_x(self, x: Vec) -> None:
        for p, v in zip(self.params, x, strict=True):
            p.value = float(v)

    def free_indices(self) -> npt.NDArray[np.intp]:
        return np.array([p.index for p in self.params if not p.fixed], dtype=np.intp)

    def n_residuals(self) -> int:
        return sum(c.n_residuals for c in self.constraints)

    def user_constraints(self) -> list[Constraint]:
        """Constraints the user added (excludes intrinsic and soft/transient ones)."""
        return [c for c in self.constraints if not (c.intrinsic or c.soft)]

    def hard_constraints(self) -> list[Constraint]:
        """Everything that must be satisfied (excludes soft/transient ones such as drag targets)."""
        return [c for c in self.constraints if not c.soft]

    def entities(self, kind: str) -> Sequence[Primitive]:
        lists: dict[str, Sequence[Primitive]] = {"point": self.points, "line": self.lines,
                                                 "circle": self.circles, "arc": self.arcs}
        return lists[kind]

    def bbox(self) -> Box:
        """(xmin, ymin, xmax, ymax) over all points; unit box at the origin if empty.
        Points only — this is what `extent()` (and through it the solver's residual scale)
        is defined on.  For what is actually drawn, use `drawn_bounds()`."""
        if not self.points:
            return (0.0, 0.0, 1.0, 1.0)
        xs = [p.x.value for p in self.points]
        ys = [p.y.value for p in self.points]
        return (min(xs), min(ys), max(xs), max(ys))

    def drawn_bounds(self) -> Box:
        """Bounds of everything drawn, curves included — what a "fit the view" wants.  A
        circle or arc reaches past its centre, so a points-only box clips it."""
        ents: list[Primitive] = [*self.points, *self.lines, *self.circles, *self.arcs]
        return _union(e.bounds() for e in ents) or self.bbox()

    def perturb(self, sigma: float, seed: int = 0) -> None:
        """Add seeded Gaussian noise to every free parameter (warm starts, witness construction)."""
        rng = np.random.default_rng(seed)
        x = self.get_x()
        free = self.free_indices()
        x[free] += rng.normal(0, sigma, len(free))
        self.set_x(x)

    def extent(self) -> float:
        """Characteristic length of the sketch (for tolerances / drag weights)."""
        x0, y0, x1, y1 = self.bbox()
        return max(x1 - x0, y1 - y0, 1.0)

    def nearest_point(self, x: float, y: float) -> tuple[Point | None, float]:
        """Closest point to (x, y) and its distance (None, inf if the sketch has no points)."""
        best, bd = None, math.inf
        for p in self.points:
            d = math.hypot(p.x.value - x, p.y.value - y)
            if d < bd:
                best, bd = p, d
        return best, bd
