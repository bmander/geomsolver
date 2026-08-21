"""Dimension expressions: `w = 1`, `h = w * 2`, `sin(h * 10)`.

A dimension may be written as an arithmetic expression naming values other dimensions define;
the core parses them, orders them by what they read and evaluates them — see `gcs_core::expr`
for the language (trigonometry in degrees, like every angle here).  This module only reads the
report.
"""

from __future__ import annotations

from dataclasses import dataclass

from gcs import _ffi
from gcs._ffi import lib
from gcs.model import Sketch


@dataclass
class ExprItem:
    """One expression in the document, after evaluation."""

    id: int
    """The constraint it is an argument of, and which argument."""
    attr: str
    text: str
    name: str | None
    """The name it defines, if any."""
    value: float
    """Its value in the units a person reads (degrees for an angle) — the last one it evaluated
    to, when `error` is set."""
    deps: list[str]
    """The names it reads."""
    error: str | None


def expressions(sk: Sketch) -> list[ExprItem]:
    """Every expression in the sketch, evaluated, in evaluation order: each after the names it
    reads, earliest in the document first among the ready ones, the ones on a cycle last."""
    items = _ffi.take_json(lib.gcs_exprs_json(sk._h)) or []
    return [ExprItem(int(it["id"]), str(it["attr"]), str(it["text"]), it["name"],
                     float(it["value"]), list(it["deps"]), it["error"]) for it in items]


__all__ = ["ExprItem", "expressions"]
