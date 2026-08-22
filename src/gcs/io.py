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


def copy(sk: Sketch, entities: Iterable[Primitive]) -> Sketch:
    """The selection as a sketch of its own: the entities picked, the points that define them
    and the constraints all of whose entities came along.  It is an ordinary sketch, so a
    clipboard is a document — `dumps` it and it saves, `loads` it and it pastes."""
    p, n = _ffi.send_json([e.ref for e in entities])
    return Sketch(lib.gcs_copy(sk._h, p, n))


def paste(sk: Sketch, clip: Sketch, dx: float = 0.0, dy: float = 0.0) -> list[Primitive]:
    """Add everything in `clip` to `sk`, moved by (dx, dy), and return what that made."""
    made: list[list[Any]] = _ffi.take_json(lib.gcs_paste(sk._h, clip._h, float(dx), float(dy)))
    sk.touch()    # the pasted constraints arrived behind the proxy's back
    of: dict[str, list[Primitive]] = {
        "point": list(sk.points), "line": list(sk.lines),
        "circle": list(sk.circles), "arc": list(sk.arcs), "spline": list(sk.splines),
    }
    return [of[kind][i] for kind, i in made or []]


def describe(c: Constraint, ix: Sketch | None = None) -> str:
    """Human-readable one-liner: `Distance(P0, P1, 80)`; angles shown in degrees."""
    return c.describe()


def callouts(sk: Sketch, unit: float = 1.0) -> dict[str, Any]:
    """Every dimensioned constraint as a drafting figure, in world coordinates.

    Extension lines, a dimension line between two arrowheads, a radial leader, an angular arc
    and the number beside it are all geometry, so the whole layout comes from the core.  `unit`
    is the world length of one screen pixel: pass it and the stand-offs, arrowheads and
    characters come out the same size at any zoom.  The result carries `font`, `arrow` and
    `barb` (the size and shape the layout reserved, in pixels) and `items`, one per dimension.
    """
    d: dict[str, Any] = _ffi.take_json(lib.gcs_callouts_json(sk._h, float(unit)))
    return d


def callout_pick(sk: Sketch, unit: float, x: float, y: float,
                 tol_px: float) -> Constraint | None:
    """The dimension whose callout the world point (x, y) lands on, within `tol_px` pixels."""
    cid = int(lib.gcs_callout_pick(sk._h, float(unit), float(x), float(y), float(tol_px)))
    return None if cid < 0 else sk.constraint_by_id(cid)


def callout_grab(c: Constraint, unit: float, x: float, y: float) -> tuple[float, float] | None:
    """Take hold of `c`'s callout at a world point: the grip to hand back to `callout_drag` for
    the rest of the gesture, so the callout moves with the pointer instead of jumping to it."""
    out = _ffi.f64(2)
    ok = lib.gcs_callout_grab(c.sketch._h, c._id, float(unit), float(x), float(y), _ffi.pf(out))
    return (float(out[0]), float(out[1])) if ok else None


def callout_drag(c: Constraint, x: float, y: float, grip: tuple[float, float]) -> bool:
    """Move `c`'s callout so the point it was grabbed at follows the pointer to (x, y)."""
    return bool(lib.gcs_callout_drag(c.sketch._h, c._id, float(x), float(y),
                                     float(grip[0]), float(grip[1])))


def callout_reset(c: Constraint) -> bool:
    """Put `c`'s callout back wherever the layout would have put it; True if it had been moved
    at all, so a caller can tell an edit from a no-op."""
    return bool(lib.gcs_callout_reset(c.sketch._h, c._id))


def fmt(v: float, sig: int = 4) -> str:
    """Python's %g-style formatting: `sig` significant digits, trailing zeros dropped."""
    return _ffi.take_str(lib.gcs_fmt_g(float(v), int(sig)))


def name_of(e: Primitive) -> str:
    """`P0` / `L3` / `C1` / `A2` — the short label the constraint list uses."""
    return f"{e.kind[0].upper()}{e.index}"


BY_NAME = {}  # populated below from the registry, for compatibility with the reference API
from gcs.constraints import CONSTRAINT_TYPES as _TYPES  # noqa: E402

BY_NAME.update(_TYPES)

__all__ = ["BY_NAME", "callout_drag", "callout_grab", "callout_pick", "callout_reset",
           "callouts", "copy", "describe", "dumps", "fmt", "from_dict", "load", "loads",
           "name_of", "paste", "save", "to_dict", "without", "ENTITY_KINDS", "expand"]
