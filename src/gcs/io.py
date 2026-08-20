"""JSON (de)serialization of sketches, and deletion by rebuild.

The document format lives in the core; this module only moves strings.
"""

from __future__ import annotations

import json
from typing import Any, Iterable

from gcs import _ffi
from gcs._ffi import lib
from gcs.constraints import Constraint, ENTITY_KINDS
from gcs.model import Primitive, Sketch, expand  # noqa: F401  (expand is part of the public API)


def to_dict(sk: Sketch) -> dict[str, Any]:
    d: dict[str, Any] = json.loads(_ffi.take_str(lib.gcs_sketch_to_json(sk._h, -1)))
    return d


def from_dict(d: dict[str, Any]) -> Sketch:
    return loads(json.dumps(d))


def dumps(sk: Sketch, indent: int | None = None) -> str:
    return _ffi.take_str(lib.gcs_sketch_to_json(sk._h, -1 if indent is None else int(indent)))


def loads(s: str) -> Sketch:
    p, n = _ffi.send(s)
    h = lib.gcs_sketch_from_json(p, n)
    if not h:
        raise ValueError(_ffi.last_error() or "bad sketch document")
    return Sketch(h)


def save(sk: Sketch, path: str) -> None:
    with open(path, "w") as f:
        f.write(dumps(sk, indent=1))


def load(path: str) -> Sketch:
    with open(path) as f:
        return loads(f.read())


def without(sk: Sketch, entities: Iterable[Primitive] = (),
            constraints: Iterable[Constraint] = ()) -> Sketch:
    """Copy of the sketch with the given entities/constraints removed, plus everything that
    depends on a removed entity.  Deletion by rebuild — simple, and keeps the invariants
    trivially true."""
    ep, en = _ffi.send_json([e.ref for e in entities])
    cp, cn = _ffi.send_json([c._id for c in constraints if c._id >= 0])
    return Sketch(lib.gcs_without(sk._h, ep, en, cp, cn))


def describe(c: Constraint, ix: Sketch | None = None) -> str:
    """Human-readable one-liner: `Distance(P0, P1, 80)`; angles shown in degrees."""
    return c.describe()


def fmt(v: float, sig: int = 4) -> str:
    """Python's %g-style formatting: `sig` significant digits, trailing zeros dropped."""
    return _ffi.take_str(lib.gcs_fmt_g(float(v), int(sig)))


def name_of(e: Primitive) -> str:
    """`P0` / `L3` / `C1` / `A2` — the short label the constraint list uses."""
    return f"{e.kind[0].upper()}{e.index}"


BY_NAME = {}  # populated below from the registry, for compatibility with the reference API
from gcs.constraints import CONSTRAINT_TYPES as _TYPES  # noqa: E402

BY_NAME.update(_TYPES)

__all__ = ["BY_NAME", "describe", "dumps", "fmt", "from_dict", "load", "loads", "name_of",
           "save", "to_dict", "without", "ENTITY_KINDS", "expand"]
