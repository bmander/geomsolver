"""Constraint types — generated from the core's own registry.

The core declares every type's `spec` (its constructor arguments as (attribute, kind) pairs); this
module turns that declaration into Python classes, so adding a constraint type in Rust makes it
appear here with its attributes, its JSON form and its value editing, with nothing to change.

A constraint can be built before it belongs to a sketch (the reference API allowed that, and the
tests rely on it): until `Sketch.add` binds it, its arguments live locally; afterwards every read
and write goes through the core.
"""

from __future__ import annotations

from contextlib import contextmanager
from typing import Any, ClassVar, Iterator, Sequence

from gcs import _ffi
from gcs._ffi import Vec, lib
from gcs.model import Entity, Sketch

ENTITY_KINDS = frozenset({"point", "line", "circle", "arc", "circle_or_arc"})
DIMENSION_KINDS = frozenset({"length", "angle"})

_REGISTRY: dict[str, Any] = _ffi.take_json(lib.gcs_registry_json())
KERNEL_NAMES: list[str] = [k["name"] for k in _REGISTRY["kernels"]]


class Constraint:
    """Base of every constraint type.  Subclasses are generated from the registry below."""

    spec: ClassVar[tuple[tuple[str, str], ...]] = ()
    defaults: ClassVar[tuple[Any, ...]] = ()
    commutative: ClassVar[bool] = False
    soft_by_default: ClassVar[bool] = False
    kernel_id: ClassVar[int] = -1
    kernel_name: ClassVar[str] = ""

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        names = [n for n, _ in self.spec]
        if len(args) > len(names):
            raise TypeError(f"{type(self).__name__} takes {len(names)} arguments")
        vals: list[Any] = list(args) + [None] * (len(names) - len(args))
        for k, v in kwargs.items():
            if k not in names:
                raise TypeError(f"{type(self).__name__} has no argument {k!r}")
            vals[names.index(k)] = v
        # an omitted value takes the core's own default (a drag weight of 1, an external
        # tangency, a tangency side read off the geometry)
        vals = [self.defaults[i] if v is None else v for i, v in enumerate(vals)]
        self._args: list[Any] = [self._coerce(v, kind) for v, (_, kind) in zip(vals, self.spec)]
        self._sketch: Sketch | None = None
        self._id: int = -1
        self.soft = self.soft_by_default
        self.intrinsic = False

    # -- values -------------------------------------------------------------

    @staticmethod
    def _coerce(v: Any, kind: str) -> Any:
        if v is None or kind in ENTITY_KINDS or kind in ("str", "bool"):
            return v
        if kind == "int":
            return int(v)
        return float(v)

    def args(self) -> list[Any]:
        """Constructor arguments in spec order (round-trips through `type(self)(*args)`)."""
        return list(self._args)

    def entities(self) -> list[Entity]:
        """Entities this constraint references directly, in spec order."""
        return [v for v, (_, k) in zip(self._args, self.spec) if k in ENTITY_KINDS]

    def dimensions(self) -> list[tuple[str, str]]:
        return [(n, k) for n, k in self.spec if k in DIMENSION_KINDS]

    @property
    def n_residuals(self) -> int:
        return int(_REGISTRY["kernels"][self.kernel_id]["nRes"])

    @property
    def sketch(self) -> Sketch:
        if self._sketch is not None:
            return self._sketch
        for e in self.entities():
            if isinstance(e, Entity):
                return e.sketch
        raise ValueError(f"{type(self).__name__} references no sketch")

    # -- binding ------------------------------------------------------------

    def to_record(self) -> dict[str, Any]:
        out: list[Any] = []
        for v, (_, kind) in zip(self._args, self.spec):
            out.append(v.ref if kind in ENTITY_KINDS and isinstance(v, Entity) else v)
        return {"type": type(self).__name__, "args": out,
                "soft": self.soft, "intrinsic": self.intrinsic}

    def _bind(self, sk: Sketch) -> None:
        if self._id >= 0 and self._sketch is sk:
            return
        p, n = _ffi.send_json(self.to_record())
        cid = int(lib.gcs_constraint_add(sk._h, p, n))
        if cid < 0:
            raise ValueError(_ffi.last_error() or "constraint rejected")
        self._sketch = sk
        self._id = cid
        sk._by_id[cid] = self
        # the core may have filled in a value we left to the geometry (a tangency's side)
        rec = _ffi.take_json(lib.gcs_constraint_json(sk._h, cid))
        if rec:
            self._absorb(sk, rec)

    def _absorb(self, sk: Sketch, rec: dict[str, Any]) -> None:
        self._sketch = sk
        self._id = int(rec["id"])
        self.soft = bool(rec["soft"])
        self.intrinsic = bool(rec["intrinsic"])
        self._args = [_from_json(sk, v, kind) for v, (_, kind) in zip(rec["args"], self.spec)]

    def _set_value(self, name: str, v: Any) -> None:
        i = next(k for k, (n, _) in enumerate(self.spec) if n == name)
        kind = self.spec[i][1]
        self._args[i] = self._coerce(v, kind)
        if self._id >= 0 and self._sketch is not None and kind not in ENTITY_KINDS:
            if kind in ("length", "angle", "float", "int"):
                p, n = _ffi.send(name)
                lib.gcs_constraint_set_num(self._sketch._h, self._id, p, n, float(self._args[i]))

    # -- evaluation ---------------------------------------------------------

    def local_values(self) -> Vec:
        """The values of the params the kernel's columns refer to."""
        with _bound(self) as (sk, cid):
            out = _ffi.f64(_MAX_PAR)
            n = lib.gcs_constraint_local_values(sk._h, cid, _ffi.pf(out))
            return out[:n].copy()

    def param_indices(self) -> list[int]:
        with _bound(self) as (sk, cid):
            out = _ffi.i32(_MAX_PAR)
            n = lib.gcs_constraint_params(sk._h, cid, _ffi.pi(out))
            return [int(i) for i in out[:n]]

    def residual(self, v: Any) -> Vec:
        return self._eval(v)[0]

    def jacobian(self, v: Any) -> Vec:
        """n_residuals x n_params, as the kernel computes it."""
        return self._eval(v)[1]

    def _eval(self, v: Any) -> tuple[Vec, Vec]:
        vals = _ffi.as_f64(v)
        n_res = self.n_residuals
        with _bound(self) as (sk, cid):
            r = _ffi.f64(n_res)
            j = _ffi.f64(n_res * _MAX_PAR)
            n_par = lib.gcs_constraint_eval(sk._h, cid, _ffi.pf(vals), _ffi.pf(r), _ffi.pf(j))
            return r.copy(), j[: n_res * n_par].reshape(n_res, n_par).copy()

    def error(self) -> float:
        """Current residual norm (convenience for reporting)."""
        with _bound(self) as (sk, cid):
            return float(lib.gcs_constraint_error(sk._h, cid))

    def describe(self) -> str:
        with _bound(self) as (sk, cid):
            return _ffi.take_str(lib.gcs_describe(sk._h, cid))

    def __repr__(self) -> str:
        return f"{type(self).__name__}(n={self.n_residuals})"


def _from_json(sk: Sketch, v: Any, kind: str) -> Any:
    if kind in ENTITY_KINDS and isinstance(v, list):
        return sk.entities(v[0])[v[1]]
    return v


_MAX_PAR = 16  # the widest kernel takes 8; a little headroom costs nothing


@contextmanager
def _bound(c: Constraint) -> Iterator[tuple[Sketch, int]]:
    """Evaluate against the core.  A constraint the user has not added yet is placed in its
    entities' sketch for the call and taken out again — the reference API allowed building one
    standalone, and the tests exercise exactly that."""
    if c._id >= 0 and c._sketch is not None:
        yield c._sketch, c._id
        return
    sk = c.sketch
    temp = type(c)(*c.args())
    temp.soft, temp.intrinsic = c.soft, c.intrinsic
    temp._bind(sk)
    try:
        # a value the core filled in (a tangency's side) belongs to the original too
        c._args = list(temp._args)
        yield sk, temp._id
    finally:
        sk.remove(temp)


def same_constraint(a: Constraint, b: Constraint) -> bool:
    """True when two constraints say exactly the same thing: same type, the same entities in the
    same roles, the same values.  `commutative` types also match with their first two entities
    swapped, since picking the pair in the other order means the same relation."""
    if type(a) is not type(b):
        return False

    def match(swap: bool) -> bool:
        order = list(range(len(a.spec)))
        if swap:
            ents = [i for i, (_, k) in enumerate(a.spec) if k in ENTITY_KINDS]
            if len(ents) < 2:
                return False
            order[ents[0]], order[ents[1]] = order[ents[1]], order[ents[0]]
        av, bv = a.args(), b.args()
        for i, (_, kind) in enumerate(a.spec):
            x, y = av[i], bv[order[i]]
            if (x is not y) if kind in ENTITY_KINDS else (x != y):
                return False
        return True

    return match(False) or (a.commutative and match(True))


# ---------------------------------------------------------------------------
# Generated types


def _make(entry: dict[str, Any]) -> type[Constraint]:
    name = entry["name"]
    spec = tuple((n, k) for n, k in entry["spec"])
    ns: dict[str, Any] = {
        "spec": spec,
        "defaults": tuple(entry["defaults"]),
        "soft_by_default": bool(entry["soft"]),
        "commutative": bool(entry["commutative"]),
        "kernel_id": int(entry["kernel"]),
        "kernel_name": KERNEL_NAMES[int(entry["kernel"])],
        "__doc__": f"{name}{tuple(n for n, _ in spec)}",
    }
    for i, (attr, kind) in enumerate(spec):
        def getter(self: Constraint, _i: int = i) -> Any:
            return self._args[_i]

        def setter(self: Constraint, v: Any, _n: str = attr) -> None:
            self._set_value(_n, v)

        ns[attr] = property(getter, setter)
    return type(name, (Constraint,), ns)


CONSTRAINT_TYPES: dict[str, type[Constraint]] = {}
for _entry in _REGISTRY["types"]:
    CONSTRAINT_TYPES[_entry["name"]] = _make(_entry)
globals().update(CONSTRAINT_TYPES)

Coincident = CONSTRAINT_TYPES["Coincident"]
Distance = CONSTRAINT_TYPES["Distance"]
Midpoint = CONSTRAINT_TYPES["Midpoint"]
DragTarget = CONSTRAINT_TYPES["DragTarget"]
Horizontal = CONSTRAINT_TYPES["Horizontal"]
Vertical = CONSTRAINT_TYPES["Vertical"]
Parallel = CONSTRAINT_TYPES["Parallel"]
Perpendicular = CONSTRAINT_TYPES["Perpendicular"]
Angle = CONSTRAINT_TYPES["Angle"]
ParallelDistance = CONSTRAINT_TYPES["ParallelDistance"]
EqualLength = CONSTRAINT_TYPES["EqualLength"]
PointOnLine = CONSTRAINT_TYPES["PointOnLine"]
PointLineDistance = CONSTRAINT_TYPES["PointLineDistance"]
PointOnCircle = CONSTRAINT_TYPES["PointOnCircle"]
Radius = CONSTRAINT_TYPES["Radius"]
EqualRadius = CONSTRAINT_TYPES["EqualRadius"]
AnnularDistance = CONSTRAINT_TYPES["AnnularDistance"]
TangentLineCircle = CONSTRAINT_TYPES["TangentLineCircle"]
TangentCircleCircle = CONSTRAINT_TYPES["TangentCircleCircle"]
TangentArcLine = CONSTRAINT_TYPES["TangentArcLine"]
Symmetric = CONSTRAINT_TYPES["Symmetric"]


def _drag_set_target(self: Constraint, tx: float, ty: float) -> None:
    self._args[1], self._args[2] = float(tx), float(ty)
    if self._id >= 0 and self._sketch is not None:
        lib.gcs_constraint_set_target(self._sketch._h, self._id, float(tx), float(ty))


DragTarget.set_target = _drag_set_target  # type: ignore[attr-defined]


def from_record(sk: Sketch, rec: dict[str, Any]) -> Constraint:
    """Materialize the proxy for a constraint the core already holds."""
    cls = CONSTRAINT_TYPES[rec["type"]]
    c = cls.__new__(cls)
    c._args = []
    c.soft = False
    c.intrinsic = False
    c._sketch = sk
    c._id = -1
    c._absorb(sk, rec)
    return c


def build(sk: Sketch, type_name: str, args: Sequence[Any]) -> Constraint:
    """Construct and add a constraint by type name — the generic path the UI applier uses."""
    c = CONSTRAINT_TYPES[type_name](*args)
    sk.add(c)
    return c


__all__ = [
    "CONSTRAINT_TYPES", "Constraint", "DIMENSION_KINDS", "ENTITY_KINDS", "build", "from_record",
    "same_constraint", *CONSTRAINT_TYPES,
]
