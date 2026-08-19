"""Parameters, primitives and the Sketch container.

Every scalar degree of freedom is a `Param`.  Primitives are thin bundles of
Params; the Sketch owns the ordered list of Params (its parameter vector) and
the ordered list of Constraints.  Ordering is deterministic by construction —
insertion order, never hashing — so identical edits give bit-identical solves.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Iterable, Sequence

import numpy as np
import numpy.typing as npt

if TYPE_CHECKING:
    from gcs.constraints import Constraint

Vec = npt.NDArray[np.float64]


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


@dataclass(eq=False)
class Line:
    p1: Point
    p2: Point

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


@dataclass(eq=False)
class Circle:
    center: Point
    radius: Param

    kind = "circle"

    @property
    def children(self) -> tuple[Point, ...]:
        return (self.center,)

    @property
    def params(self) -> tuple[Param, ...]:
        return self.center.params + (self.radius,)


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


Primitive = Point | Line | Circle | Arc


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

    def bbox(self) -> tuple[float, float, float, float]:
        """(xmin, ymin, xmax, ymax) over all points; unit box at the origin if empty."""
        if not self.points:
            return (0.0, 0.0, 1.0, 1.0)
        xs = [p.x.value for p in self.points]
        ys = [p.y.value for p in self.points]
        return (min(xs), min(ys), max(xs), max(ys))

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
