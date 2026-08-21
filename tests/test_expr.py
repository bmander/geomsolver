"""Dimension expressions through the Python binding: the core evaluates them in dependency
order, every proxy that reads a name sees the change, and the text travels with the document."""

from __future__ import annotations

import math

import pytest

from gcs import constraints as C
from gcs import io, solve
from gcs.expr import expressions
from gcs.model import Sketch


def three(texts: tuple[str, str, str]) -> tuple[Sketch, list[C.Constraint]]:
    """Three free segments, each dimensioned by the text given."""
    sk = Sketch()
    cs = []
    for i, t in enumerate(texts):
        a, b = sk.point(10 * i, 0), sk.point(10 * i + 5, 0)
        c = C.Distance(a, b, t)
        sk.add(c)
        cs.append(c)
    return sk, cs


def test_evaluated_in_dependency_order() -> None:
    # the reader comes first in the document, the definition last
    sk, cs = three(("sin(h * 10)", "h = w * 2", "w = 1"))
    assert cs[2].d == 1 and cs[1].d == 2
    assert cs[0].d == pytest.approx(math.sin(math.radians(20)))
    assert cs[1].expr("d") == "h = w * 2"
    assert cs[1].describe() == "Distance(P2, P3, h = w * 2 = 2)"
    items = expressions(sk)
    assert [it.id for it in items] == [cs[2]._id, cs[1]._id, cs[0]._id]
    assert items[1].deps == ["w"] and items[1].name == "h"
    assert all(it.error is None for it in items)
    assert solve(sk).success
    p, q = cs[1].entities()
    assert math.hypot(p.x.value - q.x.value, p.y.value - q.y.value) == pytest.approx(2)  # type: ignore[attr-defined]


def test_editing_one_dimension_moves_every_reader() -> None:
    sk, cs = three(("w = 3", "h = w * 2", "h + 1"))
    assert cs[2].d == 7
    assert cs[0].set_dimension("d", "w = 5") is None
    assert cs[1].d == 10 and cs[2].d == 11       # re-read from the core
    # a bare number is a constant again; the readers say so and keep their numbers
    assert cs[0].set_dimension("d", "4") is None
    assert cs[0].expr("d") is None and cs[1].d == 10
    bad = [it for it in expressions(sk) if it.error]
    assert [it.error for it in bad] == ["`w` is not defined", "`h` could not be evaluated"]
    # text that does not parse is refused and changes nothing
    with pytest.raises(ValueError):
        cs[0].set_dimension("d", "1 +")
    assert cs[0].d == 4
    # text that reads a name nothing defines is kept, and says why
    assert "`q` is not defined" in (cs[0].set_dimension("d", "q * 2") or "")
    assert cs[0].d == 4
    # a cycle is named
    cs[0].set_dimension("d", "w = h")
    mine = next(it for it in expressions(sk) if it.id == cs[0]._id)
    assert mine.error == "circular: w → h → w"
    # a plain set of the number drops the expression
    cs[1].d = 9
    assert cs[1].expr("d") is None


def test_angles_are_written_in_degrees() -> None:
    sk = Sketch()
    l1, l2 = sk.line_xy(0, 0, 10, 0), sk.line_xy(0, 0, 10, 5)
    ang = C.Angle(l1, l2, "a = 30")
    sk.add(ang)
    assert ang.theta == pytest.approx(math.radians(30))
    assert expressions(sk)[0].value == 30
    ang.set_dimension("theta", "45")
    assert ang.theta == pytest.approx(math.radians(45)) and ang.expr("theta") is None


def test_documents_and_rebuilds_keep_the_text() -> None:
    sk, cs = three(("w = 3", "h = w * 2", "h + 1"))
    s = io.dumps(sk)
    assert '"expr":"h = w * 2"' in s
    sk2 = io.loads(s)
    assert io.dumps(sk2) == s
    assert sk2.constraints[1].expr("d") == "h = w * 2" and sk2.constraints[1].d == 6
    # deleting the definition: readers keep their numbers and say what is missing
    sk3 = io.without(sk, constraints=[cs[0]])
    assert sk3.constraints[0].d == 6
    assert expressions(sk3)[0].error == "`w` is not defined"
    # the callouts print the name and value, or a leading = for an unnamed expression
    assert [k["text"] for k in io.callouts(sk, 1.0)["items"]] == ["w=3", "h=6", "=7"]
