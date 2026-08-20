"""Parameters, primitives and the Sketch container — proxies over the Rust model.

A `Param` is an index, an entity is a `(kind, index)` pair and a constraint is a document-stable
id; the objects here are interned per sketch, so `is` and `id()` mean what they always did while
the data itself lives in the core.  Nothing is mirrored: every read goes through the ABI.
"""

from __future__ import annotations

import math
from typing import TYPE_CHECKING, Any, Iterable, NamedTuple, Sequence

import numpy as np

from gcs import _ffi
from gcs._ffi import Vec, lib

if TYPE_CHECKING:
    from gcs.constraints import Constraint

Box = tuple[float, float, float, float]  # (xmin, ymin, xmax, ymax)

KIND_ID = {"point": 0, "line": 1, "circle": 2, "arc": 3}
KINDS = ("point", "line", "circle", "arc")


class Param:
    """One scalar unknown.  `index` is its slot in the sketch's parameter vector."""

    __slots__ = ("sketch", "index")

    def __init__(self, sketch: Sketch, index: int) -> None:
        self.sketch = sketch
        self.index = index

    @property
    def value(self) -> float:
        return float(lib.gcs_param_value(self.sketch._h, self.index))

    @value.setter
    def value(self, v: float) -> None:
        lib.gcs_param_set_value(self.sketch._h, self.index, float(v))

    @property
    def fixed(self) -> bool:
        return bool(lib.gcs_param_fixed(self.sketch._h, self.index))

    @fixed.setter
    def fixed(self, v: bool) -> None:
        lib.gcs_param_set_fixed(self.sketch._h, self.index, 1 if v else 0)

    @property
    def name(self) -> str:
        return _ffi.take_str(lib.gcs_param_name(self.sketch._h, self.index))

    def __repr__(self) -> str:
        f = " fixed" if self.fixed else ""
        return f"Param({self.name or self.index}={self.value:.6g}{f})"


class Entity:
    """A primitive: its kind and its index in the sketch's ordered list for that kind."""

    kind: str = ""
    __slots__ = ("sketch", "index")

    def __init__(self, sketch: Sketch, index: int) -> None:
        self.sketch = sketch
        self.index = index

    @property
    def _k(self) -> int:
        return KIND_ID[self.kind]

    @property
    def ref(self) -> list[Any]:
        return [self.kind, self.index]

    @property
    def params(self) -> tuple[Param, ...]:
        buf = _ffi.i32(8)
        n = lib.gcs_entity_params(self.sketch._h, self._k, self.index, _ffi.pi(buf))
        return tuple(self.sketch.param_at(int(i)) for i in buf[:n])

    @property
    def children(self) -> tuple[Point, ...]:
        if self.kind == "point":
            return ()
        buf = _ffi.i32(4)
        n = lib.gcs_entity_points(self.sketch._h, self._k, self.index, _ffi.pi(buf))
        return tuple(self.sketch.points[int(i)] for i in buf[:n])

    def bounds(self) -> Box:
        out = _ffi.f64(4)
        lib.gcs_entity_bounds(self.sketch._h, self._k, self.index, _ffi.pf(out))
        return (float(out[0]), float(out[1]), float(out[2]), float(out[3]))

    def __repr__(self) -> str:
        return f"{type(self).__name__}({self.index})"


class _Constructible(Entity):
    """Line, Circle and Arc carry the construction (reference geometry) flag."""

    __slots__ = ()

    @property
    def construction(self) -> bool:
        return bool(lib.gcs_entity_construction(self.sketch._h, self._k, self.index))

    @construction.setter
    def construction(self, v: bool) -> None:
        lib.gcs_entity_set_construction(self.sketch._h, self._k, self.index, 1 if v else 0)


class Point(Entity):
    kind = "point"
    __slots__ = ()

    @property
    def x(self) -> Param:
        return self.params[0]

    @property
    def y(self) -> Param:
        return self.params[1]

    @property
    def xy(self) -> tuple[float, float]:
        p = self.params
        return (p[0].value, p[1].value)

    @property
    def is_fixed(self) -> bool:
        p = self.params
        return p[0].fixed and p[1].fixed

    def fix(self, fixed: bool = True) -> None:
        for p in self.params:
            p.fixed = fixed


class Line(_Constructible):
    kind = "line"
    __slots__ = ()

    @property
    def p1(self) -> Point:
        return self.children[0]

    @property
    def p2(self) -> Point:
        return self.children[1]

    def direction(self) -> tuple[float, float]:
        (ax, ay), (bx, by) = self.p1.xy, self.p2.xy
        return (bx - ax, by - ay)

    def length(self) -> float:
        return math.hypot(*self.direction())


class Circle(_Constructible):
    kind = "circle"
    __slots__ = ()

    @property
    def center(self) -> Point:
        return self.children[0]

    @property
    def radius(self) -> Param:
        i = lib.gcs_entity_radius_param(self.sketch._h, self._k, self.index)
        return self.sketch.param_at(int(i))


class Arc(_Constructible):
    """CCW arc from `start` to `end` about `center`.  The radius is its own Param so Circle and
    Arc share every radius-based constraint; the two intrinsic constraints |start-center|² = r²
    and |end-center|² = r² are added by `Sketch.arc`."""

    kind = "arc"
    __slots__ = ()

    @property
    def center(self) -> Point:
        return self.children[0]

    @property
    def start(self) -> Point:
        return self.children[1]

    @property
    def end(self) -> Point:
        return self.children[2]

    @property
    def radius(self) -> Param:
        i = lib.gcs_entity_radius_param(self.sketch._h, self._k, self.index)
        return self.sketch.param_at(int(i))

    def angles(self) -> tuple[float, float]:
        out = _ffi.f64(2)
        lib.gcs_arc_angles(self.sketch._h, self.index, _ffi.pf(out))
        return (float(out[0]), float(out[1]))


Primitive = Point | Line | Circle | Arc
_CLASSES: dict[str, type[Entity]] = {"point": Point, "line": Line, "circle": Circle, "arc": Arc}


class ThreePointArc(NamedTuple):
    cx: float
    cy: float
    r: float
    a0: float
    a1: float
    swapped: bool


def three_point_arc(ax: float, ay: float, bx: float, by: float, cx: float, cy: float,
                    tol: float = 1e-9) -> ThreePointArc | None:
    """The circumcircle of three points plus the sweep direction that contains the third.
    None if they are collinear (the test is on the sine of the angle, so it is scale-free)."""
    out = _ffi.f64(6)
    ok = lib.gcs_three_point_arc(ax, ay, bx, by, cx, cy, _ffi.pf(out))
    if not ok:
        return None
    return ThreePointArc(float(out[0]), float(out[1]), float(out[2]),
                         float(out[3]), float(out[4]), bool(out[5]))


class Sketch:
    """The ordered parameter vector and constraint list, owned by the core."""

    def __init__(self, handle: Any = None) -> None:
        self._h = handle if handle is not None else lib.gcs_sketch_new()
        self._params: list[Param] = []
        self._ents: dict[str, list[Any]] = {k: [] for k in KINDS}
        self._cons: list[Constraint] = []
        self._by_id: dict[int, Constraint] = {}
        self._cdirty = True

    def __del__(self) -> None:  # pragma: no cover - interpreter shutdown ordering
        try:
            lib.gcs_sketch_free(self._h)
        except Exception:
            pass

    # -- interning ----------------------------------------------------------

    def _counts(self) -> list[int]:
        buf = _ffi.i32(6)
        lib.gcs_sketch_counts(self._h, _ffi.pi(buf))
        return [int(v) for v in buf]

    def param_at(self, i: int) -> Param:
        while len(self._params) <= i:
            self._params.append(Param(self, len(self._params)))
        return self._params[i]

    def _entities(self, kind: str, n: int) -> list[Any]:
        lst = self._ents[kind]
        cls = _CLASSES[kind]
        while len(lst) < n:
            lst.append(cls(self, len(lst)))
        del lst[n:]
        return lst

    def _sync_constraints(self) -> None:
        if not self._cdirty:
            return
        from gcs.constraints import from_record

        self._cdirty = False
        records = _ffi.take_json(lib.gcs_constraints_json(self._h)) or []
        out: list[Constraint] = []
        for rec in records:
            c = self._by_id.get(rec["id"])
            if c is None:
                c = from_record(self, rec)
                self._by_id[rec["id"]] = c
            out.append(c)
        self._cons = out

    def touch(self) -> None:
        """The constraint list changed in the core."""
        self._cdirty = True

    # -- construction -------------------------------------------------------

    def point(self, x: float, y: float, *, fixed: bool = False, name: str = "") -> Point:
        p, n = _ffi.send(name)
        i = lib.gcs_sketch_point(self._h, float(x), float(y), 1 if fixed else 0, p, n)
        return self.points[int(i)]

    def line(self, p1: Point, p2: Point) -> Line:
        i = lib.gcs_sketch_line(self._h, p1.index, p2.index)
        return self.lines[int(i)]

    def line_xy(self, x1: float, y1: float, x2: float, y2: float, name: str = "") -> Line:
        return self.line(self.point(x1, y1, name=f"{name}.p1"),
                         self.point(x2, y2, name=f"{name}.p2"))

    def circle(self, center: Point, radius: float, name: str = "") -> Circle:
        p, n = _ffi.send(name)
        i = lib.gcs_sketch_circle(self._h, center.index, float(radius), p, n)
        return self.circles[int(i)]

    def arc(self, center: Point, start: Point, end: Point, name: str = "") -> Arc:
        p, n = _ffi.send(name)
        i = lib.gcs_sketch_arc(self._h, center.index, start.index, end.index, p, n)
        self.touch()  # the two intrinsic PointOnCircle constraints came with it
        return self.arcs[int(i)]

    def arc_through(self, start: Point, end: Point, through: tuple[float, float],
                    name: str = "") -> Arc | None:
        """Arc from `start` to `end` bulging through `through`.  None if the three are collinear."""
        p, n = _ffi.send(name)
        i = lib.gcs_sketch_arc_through(self._h, start.index, end.index,
                                       float(through[0]), float(through[1]), p, n)
        if i < 0:
            return None
        self.touch()
        return self.arcs[int(i)]

    def rectangle(self, a: Point, x1: float, y1: float, name: str = "") -> list[Line]:
        """Four lines round the corners, sharing corner points, with three perpendiculars — the
        fourth follows, so adding it would over-constrain every rectangle by one equation."""
        p, n = _ffi.send(name)
        out = _ffi.i32(4)
        lib.gcs_sketch_rectangle(self._h, a.index, float(x1), float(y1), p, n, _ffi.pi(out))
        self.touch()
        return [self.lines[int(i)] for i in out]

    def rectangle_xy(self, x0: float, y0: float, x1: float, y1: float, name: str = "") -> list[Line]:
        return self.rectangle(self.point(x0, y0, name=f"{name}.a"), x1, y1, name)

    def add(self, *constraints: Constraint) -> None:
        for c in constraints:
            c._bind(self)
        self.touch()

    def remove(self, constraint: Constraint) -> None:
        if constraint._id >= 0:
            lib.gcs_constraint_remove(self._h, constraint._id)
            self._by_id.pop(constraint._id, None)
            constraint._id = -1
            self.touch()

    # -- lists --------------------------------------------------------------

    @property
    def params(self) -> list[Param]:
        n = self._counts()[0]
        self.param_at(max(n - 1, 0))
        return self._params[:n]

    @property
    def points(self) -> list[Point]:
        return self._entities("point", self._counts()[1])

    @property
    def lines(self) -> list[Line]:
        return self._entities("line", self._counts()[2])

    @property
    def circles(self) -> list[Circle]:
        return self._entities("circle", self._counts()[3])

    @property
    def arcs(self) -> list[Arc]:
        return self._entities("arc", self._counts()[4])

    @property
    def constraints(self) -> list[Constraint]:
        self._sync_constraints()
        return list(self._cons)

    @constraints.setter
    def constraints(self, cs: Sequence[Constraint]) -> None:
        p, n = _ffi.send_json([c._id for c in cs if c._id >= 0])
        lib.gcs_sketch_set_constraints(self._h, p, n)
        self.touch()

    def entities(self, kind: str) -> Sequence[Primitive]:
        if kind == "point":
            return self.points
        if kind == "line":
            return self.lines
        if kind == "circle":
            return self.circles
        return self.arcs

    def primitives(self) -> list[Primitive]:
        return [*self.points, *self.lines, *self.circles, *self.arcs]

    def user_constraints(self) -> list[Constraint]:
        """What the user added: no intrinsic ones, no soft ones."""
        return [c for c in self.constraints if not (c.intrinsic or c.soft)]

    def hard_constraints(self) -> list[Constraint]:
        return [c for c in self.constraints if not c.soft]

    def constraint_by_id(self, cid: int) -> Constraint | None:
        self._sync_constraints()
        return self._by_id.get(cid)

    # -- parameter vector ---------------------------------------------------

    def get_x(self) -> Vec:
        out = _ffi.f64(self._counts()[0])
        lib.gcs_sketch_get_x(self._h, _ffi.pf(out))
        return out

    def set_x(self, x: Any) -> None:
        a = _ffi.as_f64(x)
        lib.gcs_sketch_set_x(self._h, _ffi.pf(a), len(a))

    def free_indices(self) -> Any:
        return np.array([p.index for p in self.params if not p.fixed], dtype=np.intp)

    def n_residuals(self) -> int:
        return int(lib.gcs_sketch_n_residuals(self._h))

    # -- geometry -----------------------------------------------------------

    def bbox(self) -> Box:
        out = _ffi.f64(4)
        lib.gcs_sketch_bounds(self._h, 0, _ffi.pf(out))
        return (float(out[0]), float(out[1]), float(out[2]), float(out[3]))

    def drawn_bounds(self) -> Box:
        out = _ffi.f64(4)
        lib.gcs_sketch_bounds(self._h, 1, _ffi.pf(out))
        return (float(out[0]), float(out[1]), float(out[2]), float(out[3]))

    def extent(self) -> float:
        return float(lib.gcs_sketch_extent(self._h))

    def perturb(self, sigma: float, seed: int = 0) -> None:
        lib.gcs_sketch_perturb(self._h, float(sigma), int(seed) & 0xFFFFFFFF)

    def nearest_point(self, x: float, y: float) -> tuple[Point | None, float]:
        out = _ffi.f64(1)
        i = lib.gcs_sketch_nearest_point(self._h, float(x), float(y), _ffi.pf(out))
        return (self.points[int(i)] if i >= 0 else None, float(out[0]))

    # -- document state -----------------------------------------------------

    @property
    def branches(self) -> dict[str, int]:
        return {k: int(v) for k, v in
                (_ffi.take_json(lib.gcs_branches_json(self._h)) or {}).items()}

    @branches.setter
    def branches(self, b: dict[str, int]) -> None:
        p, n = _ffi.send_json({k: int(v) for k, v in b.items()})
        lib.gcs_branches_set_json(self._h, p, n)

    def update_branches(self, b: dict[str, int]) -> None:
        merged = self.branches
        merged.update(b)
        self.branches = merged

    def copy(self) -> Sketch:
        return Sketch(lib.gcs_sketch_clone(self._h))


def signed_point_to_line(p: tuple[float, float], ln: Line) -> float:
    """Signed perpendicular offset from the *infinite* line, positive to its left; inf when the
    line is degenerate."""
    return float(lib.gcs_signed_point_to_line(ln.sketch._h, float(p[0]), float(p[1]), ln.index))


def distance_between(a: Primitive, b: Primitive) -> float:
    """Shortest distance between two entities, as a sketcher measures it."""
    return float(lib.gcs_distance_between(a.sketch._h, KIND_ID[a.kind], a.index,
                                          KIND_ID[b.kind], b.index))


def expand(ents: Iterable[Primitive]) -> list[Primitive]:
    """Entities plus their sub-entities."""
    out: list[Primitive] = []
    for e in ents:
        out.append(e)
        out.extend(e.children)
    return out
