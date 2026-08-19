"""JSON (de)serialization of sketches.

Entities are referenced by [kind, index] into the sketch's ordered lists;
constraints serialize their constructor arguments per `Constraint.spec`.
Intrinsic constraints are not stored — the primitives recreate them.
"""

from __future__ import annotations

import json
import math
from collections.abc import Iterable
from typing import Any

from gcs.constraints import ENTITY_KINDS, Constraint
from gcs.model import Primitive, Sketch, expand

BY_NAME: dict[str, type[Constraint]] = {}


def _register(cls: type[Constraint]) -> None:
    BY_NAME[cls.__name__] = cls
    for sub in cls.__subclasses__():
        _register(sub)


_register(Constraint)


class Index:
    """(kind, index) lookup by identity — O(1) instead of list.index per reference."""

    def __init__(self, sk: Sketch) -> None:
        self.of: dict[int, tuple[str, int]] = {}
        for kind in ("point", "line", "circle", "arc"):
            for i, e in enumerate(sk.entities(kind)):
                self.of[id(e)] = (kind, i)

    def ref(self, e: Primitive) -> list[Any]:
        return list(self.of[id(e)])

    def name(self, e: Primitive) -> str:
        kind, i = self.of[id(e)]
        return f"{kind[0].upper()}{i}"


def to_dict(sk: Sketch) -> dict[str, Any]:
    ix = Index(sk)
    return {
        "version": 1,
        "points": [{"x": p.x.value, "y": p.y.value, "fixed": p.is_fixed} for p in sk.points],
        "lines": [[ix.ref(l.p1)[1], ix.ref(l.p2)[1]] for l in sk.lines],
        "circles": [{"center": ix.ref(c.center)[1], "r": c.radius.value, "fixed": c.radius.fixed} for c in sk.circles],
        "arcs": [{"center": ix.ref(a.center)[1], "start": ix.ref(a.start)[1], "end": ix.ref(a.end)[1],
                  "r": a.radius.value, "fixed": a.radius.fixed} for a in sk.arcs],
        "constraints": [
            {"type": type(c).__name__,
             "args": [ix.ref(v) if kind in ENTITY_KINDS else v for v, (_, kind) in zip(c.args(), c.spec, strict=True)]}
            for c in sk.constraints if not c.intrinsic
        ],
        "branches": dict(sk.branches),
    }


def from_dict(d: dict[str, Any]) -> Sketch:
    sk = Sketch()
    for i, p in enumerate(d["points"]):
        sk.point(p["x"], p["y"], fixed=bool(p.get("fixed", False)), name=f"p{i}")
    for a, b in d["lines"]:
        sk.line(sk.points[a], sk.points[b])
    for c in d["circles"]:
        circ = sk.circle(sk.points[c["center"]], c["r"])
        circ.radius.fixed = bool(c.get("fixed", False))
    for a in d["arcs"]:
        arc = sk.arc(sk.points[a["center"]], sk.points[a["start"]], sk.points[a["end"]])
        arc.radius.value = float(a["r"])
        arc.radius.fixed = bool(a.get("fixed", False))
    for c in d["constraints"]:
        t = BY_NAME[c["type"]]
        args = [sk.entities(v[0])[v[1]] if kind in ENTITY_KINDS else v
                for v, (_, kind) in zip(c["args"], t.spec, strict=True)]
        sk.add(t(*args))
    sk.branches = {k: int(v) for k, v in d.get("branches", {}).items()}
    return sk


def dumps(sk: Sketch, **kw: Any) -> str:
    return json.dumps(to_dict(sk), **kw)


def loads(s: str) -> Sketch:
    return from_dict(json.loads(s))


def save(sk: Sketch, path: str) -> None:
    with open(path, "w") as f:
        json.dump(to_dict(sk), f, indent=1)


def load(path: str) -> Sketch:
    with open(path) as f:
        return from_dict(json.load(f))


def without(sk: Sketch, entities: Iterable[Primitive] = (), constraints: Iterable[Constraint] = ()) -> Sketch:
    """Copy of the sketch with the given entities/constraints removed, plus every
    entity or constraint that depends on a removed entity.  Deletion by rebuild —
    simple, and keeps Sketch's invariants trivially true."""
    dead = {id(e) for e in entities}
    dead_c = {id(c) for c in constraints}

    def alive(e: Primitive) -> bool:
        return id(e) not in dead and not any(id(ch) in dead for ch in e.children)

    tmp = Sketch()
    tmp.points = [p for p in sk.points if alive(p)]
    tmp.lines = [l for l in sk.lines if alive(l)]
    tmp.circles = [c for c in sk.circles if alive(c)]
    tmp.arcs = [a for a in sk.arcs if alive(a)]
    tmp.constraints = [c for c in sk.constraints if id(c) not in dead_c and not c.intrinsic
                       and all(id(e) not in dead for e in expand(c.entities()))]
    return from_dict(to_dict(tmp))


def describe(c: Constraint, ix: Index | Sketch) -> str:
    """Human-readable one-liner: `Distance(P0, P1, 80)`; angles shown in degrees.
    Pass a prebuilt `Index` when describing many constraints of one sketch."""
    if isinstance(ix, Sketch):
        ix = Index(ix)
    parts = []
    for v, (_, kind) in zip(c.args(), c.spec, strict=True):
        if kind in ENTITY_KINDS:
            parts.append(ix.name(v))
        elif kind == "angle":
            parts.append(f"{math.degrees(v):.3g}°")
        elif kind in ("length", "float"):
            parts.append(f"{v:.4g}")
        else:
            parts.append(str(v))
    return f"{type(c).__name__}({', '.join(parts)})"

