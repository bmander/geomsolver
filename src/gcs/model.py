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

KIND_ID = {"point": 0, "line": 1, "circle": 2, "arc": 3, "spline": 4}
KINDS = ("point", "line", "circle", "arc", "spline")


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
    def _slots(self) -> int:
        """How many indices `params`/`children` have to make room for.  Every kind but a spline
        has a width fixed by its shape; a spline's is its control polygon, so only it asks."""
        return 8

    @property
    def params(self) -> tuple[Param, ...]:
        buf = _ffi.i32(self._slots)
        n = lib.gcs_entity_params(self.sketch._h, self._k, self.index, _ffi.pi(buf))
        return tuple(self.sketch.param_at(int(i)) for i in buf[:n])

    @property
    def children(self) -> tuple[Point, ...]:
        if self.kind == "point":
            return ()
        buf = _ffi.i32(self._slots)
        n = lib.gcs_entity_points(self.sketch._h, self._k, self.index, _ffi.pi(buf))
        pts = self.sketch.points
        return tuple(pts[int(i)] for i in buf[:n])

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


#: Points a curve's polyline is expected to need; enough for any ordinary curve at any ordinary
#: zoom, and only a miss costs a second tessellation.
POLYLINE_CAP = 512


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


class Spline(_Constructible):
    """A cubic B-spline over an ordered control polygon.

    The control points are ordinary sketch Points, so they drag, snap and take constraints like
    any others.  Everything about the curve itself — where a parameter lands, the polyline it is
    drawn as, the distance to it — is computed in the core; nothing here evaluates a basis
    function."""

    kind = "spline"
    __slots__ = ()

    @property
    def _slots(self) -> int:
        """A control polygon is as long as it is, so this is the one kind whose width follows
        the document — every other kind keeps the fixed buffer and never asks for a count."""
        return max(8, 2 * len(self.sketch.points))

    @property
    def ctrl(self) -> tuple[Point, ...]:
        return self.children

    @property
    def knots(self) -> tuple[float, ...]:
        buf = _ffi.f64(self._slots + 8)
        n = lib.gcs_spline_knots(self.sketch._h, self.index, _ffi.pf(buf))
        return tuple(float(v) for v in buf[:n])

    @property
    def domain(self) -> tuple[float, float]:
        """The parameter interval the curve is drawn over."""
        out = _ffi.f64(2)
        lib.gcs_spline_domain(self.sketch._h, self.index, _ffi.pf(out))
        return (float(out[0]), float(out[1]))

    def eval(self, t: float) -> tuple[tuple[float, float], tuple[float, float],
                                      tuple[float, float]]:
        """C(t), C'(t) and C''(t)."""
        out = _ffi.f64(6)
        lib.gcs_spline_eval(self.sketch._h, self.index, float(t), _ffi.pf(out))
        return ((float(out[0]), float(out[1])), (float(out[2]), float(out[3])),
                (float(out[4]), float(out[5])))

    def point_at(self, t: float) -> tuple[float, float]:
        return self.eval(t)[0]

    def polyline(self, unit: float = 0.01) -> list[tuple[float, float]]:
        """The curve refined until a chord strays less than a fraction of a pixel from it.
        `unit` is the world length of one screen pixel, as everywhere else in the drawing.

        One tessellation in almost every case: the core reports how many points it wanted, so a
        buffer that was too small costs a second pass and nothing else."""
        def read(cap: int) -> tuple[int, Vec]:
            buf = _ffi.f64(2 * cap)
            need = int(lib.gcs_spline_polyline(self.sketch._h, self.index, float(unit),
                                               _ffi.pf(buf), cap))
            return need, buf
        cap = POLYLINE_CAP
        need, buf = read(cap)
        if need > cap:
            cap = need
            need, buf = read(cap)
        return [(float(buf[2 * i]), float(buf[2 * i + 1])) for i in range(max(0, need))]

    def insert_control(self, t: float) -> Point | None:
        """Give the curve one more control point at `t`, without changing its shape.  Every
        contact keeps its parameter and its place; None if `t` is not a place a knot can go."""
        i = int(lib.gcs_spline_insert_control(self.sketch._h, self.index, float(t)))
        return self.sketch.points[i] if i >= 0 else None

    def closest(self, x: float, y: float) -> tuple[float, float]:
        """The parameter of the nearest curve point, and how far that is."""
        out = _ffi.f64(2)
        lib.gcs_spline_closest(self.sketch._h, self.index, float(x), float(y), _ffi.pf(out))
        return (float(out[0]), float(out[1]))


_CLASSES: dict[str, type[Entity]] = {"point": Point, "line": Line, "circle": Circle,
                                     "arc": Arc, "spline": Spline}

Primitive = Point | Line | Circle | Arc | Spline


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
        buf = _ffi.i32(7)
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
            else:
                # the core is the authority on a value: a dimension written as an expression
                # changes when a name it reads does, with nothing said to this proxy
                c._absorb(self, rec)
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

    def spline(self, ctrl: Sequence[Point],
               knots: Sequence[float] | None = None) -> Spline | None:
        """A cubic B-spline over `ctrl`.  `None` if there are too few control points for a cubic,
        or the knot vector given does not fit them."""
        ids = _ffi.i32(max(1, len(ctrl)))
        for k, p in enumerate(ctrl):
            ids[k] = p.index
        if knots is None:
            i = lib.gcs_sketch_spline(self._h, _ffi.pi(ids), len(ctrl))
        else:
            ks = _ffi.f64(max(1, len(knots)))
            for k, v in enumerate(knots):
                ks[k] = float(v)
            i = lib.gcs_sketch_spline_knots(self._h, _ffi.pi(ids), len(ctrl),
                                            _ffi.pf(ks), len(knots))
        if i < 0:
            return None
        return self.splines[int(i)]

    def spline_through(self, pts: Sequence[tuple[float, float]],
                       hold: Sequence[Point | None] | None = None) -> Spline | None:
        """A cubic B-spline through `pts`, in order.  The control points are computed, not given
        — the same bargain `arc_through` strikes.

        `hold[i]`, where given, is a Point the place came from rather than empty space: the curve
        is held to it by a `PointOnSpline` pinned at the parameter the fit chose, so a curve
        fitted to constrained points is itself fully constrained.  `None` if there are too few
        points for a cubic, or they give no parameterisation."""
        n = max(1, len(pts))
        buf = _ffi.f64(2 * n)
        for k, (x, y) in enumerate(pts):
            buf[2 * k], buf[2 * k + 1] = float(x), float(y)
        held = None
        if hold and any(h is not None for h in hold):
            held = _ffi.i32(n)
            for k in range(len(pts)):
                h = hold[k] if k < len(hold) else None
                held[k] = h.index if h is not None else -1
        i = lib.gcs_sketch_spline_through(self._h, _ffi.pf(buf), len(pts),
                                          _ffi.pi(held) if held is not None else None)
        return self.splines[int(i)] if i >= 0 else None

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
    def splines(self) -> list[Spline]:
        return self._entities("spline", self._counts()[6])

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
        if kind == "spline":
            return self.splines
        return self.arcs

    def primitives(self) -> list[Primitive]:
        return [*self.points, *self.lines, *self.circles, *self.arcs, *self.splines]

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
        """Write the parameter vector.  A vector of the wrong length belongs to some other
        sketch, and is refused rather than written as far as it goes."""
        a = _ffi.as_f64(x)
        if lib.gcs_sketch_set_x(self._h, _ffi.pf(a), len(a)) != 0:
            raise ValueError(_ffi.last_error() or "set_x: wrong length")

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

    def topology_key(self) -> str:
        """What a compiled plan or System depends on: which entities exist, which constraints
        (by id) and which params are fixed.  A cache over compiled artefacts keys on this."""
        return _ffi.take_str(lib.gcs_sketch_topology_key(self._h))

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


def angle_between(a: Line, b: Line) -> float:
    """Signed CCW angle from line `a` to line `b`, in radians — what an `Angle` constraint's
    value means, and what a dimension dialog offers as the current value."""
    return float(lib.gcs_angle_between(a.sketch._h, a.index, b.index))


def on_radius(cx: float, cy: float, tx: float, ty: float, r: float) -> tuple[float, float] | None:
    """The point at distance `r` from (cx, cy) towards (tx, ty).  The centre-start-end arc
    construction: the third click gives a direction, and the radius comes from the second.
    None when the target is the centre, which names no direction."""
    out = _ffi.f64(2)
    if not lib.gcs_on_radius(float(cx), float(cy), float(tx), float(ty), float(r), _ffi.pf(out)):
        return None
    return float(out[0]), float(out[1])


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
