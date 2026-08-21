"""Reference sketches used by the tests, the benchmarks and the app's case library.

The sketches themselves are built in the core; this is the lookup and the case list.
"""

from __future__ import annotations

from typing import Any, Callable

from gcs import _ffi
from gcs._ffi import lib
from gcs.model import Sketch


def build(name: str) -> Sketch:
    p, n = _ffi.send(name)
    h = lib.gcs_example(p, n)
    if not h:
        raise KeyError(_ffi.last_error() or name)
    return Sketch(h)


def rect_fillets(w: float = 100.0, h: float = 60.0, r: float = 10.0) -> Sketch:
    return build(f"rect_fillets:{w}:{h}:{r}")


def slotted_link(length: float = 80.0, r: float = 15.0, hole_r: float = 6.0) -> Sketch:
    return build("slotted_link")


def truss(bays: int = 8) -> Sketch:
    return build(f"truss:{bays}")


def polygon_chain(n: int = 12) -> Sketch:
    return build(f"polygon_chain:{n}")


def rect_fillets_conflict() -> Sketch:
    return build("rect_fillets_conflict")


def rect_fillets_under() -> Sketch:
    return build("rect_fillets_under")


def truss_redundant() -> Sketch:
    return build("truss_redundant")


def truss_conflict() -> Sketch:
    return build("truss_conflict")


def truss_floating(bays: int = 8) -> Sketch:
    return build(f"truss_floating:{bays}")


def zigzag(n: int = 32, copies: int = 1) -> Sketch:
    """`copies` separate staircases of `n` points, every segment alternately vertical and
    horizontal and nothing else: the sketch that finds any cost going with the document rather
    than with the figure being dragged."""
    return build(f"zigzag:{n}:{copies}")


def impossible_triangle() -> Sketch:
    return build("impossible_triangle")


def altitudes() -> Sketch:
    return build("altitudes")


def parallels() -> Sketch:
    return build("parallels")


def pythagoras(a: float = 30.0, b: float = 40.0) -> Sketch:
    """The graphical proof of the Pythagorean theorem: four a×b right triangles in a square of
    side a + b leave a square of side c, dimensioned `c = hypot(a, b)` — redundant and
    consistent."""
    return build(f"pythagoras:{a}:{b}")


def k33(seed: int = 3) -> Sketch:
    return build(f"k33:{seed}")


def laman(n: int = 10, seed: int = 0) -> Sketch:
    return build(f"laman:{n}:{seed}")


def perturb(sk: Sketch, sigma: float, seed: int = 0) -> None:
    sk.perturb(sigma, seed)


def henneberg_edges(n: int, seed: int = 0) -> list[tuple[int, int]]:
    """A random Laman graph on n vertices by Henneberg I/II moves — minimally rigid by
    construction, and the generator the property tests use."""
    e = _ffi.take_json(lib.gcs_henneberg_edges_json(int(n), seed & 0xFFFFFFFF)) or []
    return [(a, b) for a, b in e]


EXAMPLES: dict[str, Callable[[], Sketch]] = {
    "rect_fillets": rect_fillets,
    "slotted_link": slotted_link,
    "truss": truss,
    "polygon_chain": polygon_chain,
}


def cases() -> list[dict[str, Any]]:
    """The case library shown in the app: label, key and a one-line description."""
    return _ffi.take_json(lib.gcs_cases_json()) or []


__all__ = ["EXAMPLES", "altitudes", "henneberg_edges", "build", "cases", "impossible_triangle", "k33", "laman", "pythagoras",
           "parallels", "perturb", "polygon_chain", "rect_fillets", "rect_fillets_conflict",
           "rect_fillets_under", "slotted_link", "truss", "truss_conflict", "truss_floating",
           "truss_redundant", "zigzag"]
