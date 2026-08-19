"""Headless smoke test of the Qt sketcher (skipped if PySide6 is unavailable)."""

import os

import pytest

pytest.importorskip("PySide6")
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtCore import QEvent, Qt  # noqa: E402
from PySide6.QtGui import QMouseEvent  # noqa: E402
from PySide6.QtWidgets import QApplication  # noqa: E402

from gcs import constraints as C  # noqa: E402
from gcs.app import MainWindow  # noqa: E402
from gcs.examples import EXAMPLES  # noqa: E402
from gcs.model import Line, Sketch  # noqa: E402


@pytest.fixture(scope="module")
def qapp() -> QApplication:
    return QApplication.instance() or QApplication([])


def _mouse(v, kind, x, y, button=Qt.MouseButton.LeftButton, mods=Qt.KeyboardModifier.NoModifier):  # type: ignore[no-untyped-def]
    sp = v.w2s(x, y)
    if kind == "press":
        v.mousePressEvent(QMouseEvent(QEvent.Type.MouseButtonPress, sp, sp, button, button, mods))
    elif kind == "release":
        v.mouseReleaseEvent(QMouseEvent(QEvent.Type.MouseButtonRelease, sp, sp, button, Qt.MouseButton.NoButton, mods))
    else:
        v.mouseMoveEvent(QMouseEvent(QEvent.Type.MouseMove, sp, sp, Qt.MouseButton.NoButton, button, mods))


def _click(v, x, y, mods=Qt.KeyboardModifier.NoModifier):  # type: ignore[no-untyped-def]
    _mouse(v, "press", x, y, mods=mods)
    _mouse(v, "release", x, y, mods=mods)


def test_draw_constrain_drag_undo(qapp: QApplication) -> None:
    w = MainWindow(Sketch())
    w.resize(1000, 700)
    w.show()
    v = w.view
    v.set_tool("line")
    for xy in ((0, 0), (30, 0), (15, 20), (0, 0)):
        _click(v, *xy)
    v.cancel_tool()
    assert (len(v.sketch.lines), len(v.sketch.points)) == (3, 3)
    v.set_tool("arc")
    for xy in ((50, 0), (60, 0), (50, 12)):
        _click(v, *xy)
    assert len(v.sketch.arcs) == 1 and v.sketch.arcs[0].radius.value == pytest.approx(10)

    v.set_tool("select")
    _click(v, 15, 0)
    assert isinstance(v.selected[0], Line)
    w._apply(C.Horizontal, 0, 1, 0)
    _click(v, 0, 0)
    v.toggle_fix_selected()
    _click(v, 30, 0)
    _click(v, 15, 20, mods=Qt.KeyboardModifier.ShiftModifier)
    assert len(v.selected) == 2
    v.add_constraint(C.Distance(v.selected[0], v.selected[1], 25))
    v.add_constraint(C.Distance(v.sketch.points[0], v.sketch.points[1], 30))

    apex = v.sketch.points[2]
    _mouse(v, "press", *apex.xy)
    assert v.drag is not None
    for i in range(10):
        _mouse(v, "move", 15 + i, 20 + i)
    _mouse(v, "release", 0, 0)
    assert v.drag is None
    assert v.last_result is not None and v.last_result.success
    for c in v.sketch.constraints:
        assert c.error() < 1e-6

    n = len(v.sketch.constraints)
    v.undo()
    assert len(v.sketch.constraints) == n
    v.selected = [v.sketch.arcs[0]]
    v.delete_selected()
    assert not v.sketch.arcs
    v.undo()
    assert len(v.sketch.arcs) == 1
    w.refresh()
    assert w.clist.count() == 3


def test_examples_render_and_tangent_action(qapp: QApplication) -> None:
    w = MainWindow(Sketch())
    w.show()
    v = w.view
    for name in EXAMPLES:
        v.set_sketch(EXAMPLES[name]())
        v.repaint()
    sk = EXAMPLES["rect_fillets"]()
    v.set_sketch(sk)
    v.selected = [sk.arcs[0], sk.lines[0]]
    w.c_tangent()
    assert isinstance(sk.constraints[-1], C.TangentArcLine)
    v.selected = [sk.points[0], sk.points[1]]
    w._apply(C.Coincident, 2, 0, 0)
    assert v.last_result is not None and not v.last_result.success  # conflicts with the width dimension


def test_selection_highlights_constraint_rows(qapp: QApplication) -> None:
    from gcs.app import COL_ROW_SEL

    w = MainWindow(EXAMPLES["rect_fillets"]())
    w.show()
    v = w.view
    v.selected = [v.sketch.lines[0]]  # bottom line: Horizontal, 2 tangents, width Distance (via endpoints)
    w.refresh()
    rows = [w.clist.item(i) for i in range(w.clist.count())]
    hit = [r for r in rows if r.background().color() == COL_ROW_SEL]
    assert {type(r.data(Qt.ItemDataRole.UserRole)).__name__ for r in hit} == {"Horizontal", "TangentArcLine", "Distance"}
    v.selected = []
    w.refresh()
    assert not any(w.clist.item(i).background().color() == COL_ROW_SEL for i in range(w.clist.count()))


def test_delete_key_removes_selected_constraint(qapp: QApplication) -> None:
    w = MainWindow(EXAMPLES["rect_fillets"]())
    w.show()
    n = len(w.view.sketch.constraints)
    w.clist.setCurrentRow(0)
    w.clist.setFocus()
    qapp.processEvents()
    target = w.clist.currentItem().data(Qt.ItemDataRole.UserRole)
    w.clist.setFocus()
    if not w.clist.hasFocus():  # offscreen platform may not grant focus; call the handler's branch directly
        w.delete_constraint()
    else:
        w.delete_pressed()
    assert len(w.view.sketch.constraints) == n - 1
    assert target not in w.view.sketch.constraints
    assert w.clist.currentRow() == -1 and w.view.highlight == []  # nothing auto-selected/highlighted
    # with focus on the view and nothing selected, Delete is a no-op
    w.view.setFocus()
    w.view.selected = []
    n = len(w.view.sketch.constraints)
    w.delete_pressed()
    assert len(w.view.sketch.constraints) == n


def test_diagnosis_drives_colours_and_dialog_text(qapp: QApplication) -> None:
    from gcs.app import COL_STATE

    w = MainWindow(EXAMPLES["rect_fillets"]())
    w.show()
    v = w.view
    assert v.diagnosis is not None and v.diagnosis.status == "well"
    assert all(v.state_of(e) == "well" for e in v.sketch.points)
    # add a conflicting width → conflict state on the bottom line, message names the culprits
    l0 = v.sketch.lines[0]
    v.add_constraint(C.Distance(l0.p1, l0.p2, 50))
    assert v.diagnosis is not None and v.diagnosis.status == "conflict"
    assert v.state_of(l0) == "conflict"
    assert COL_STATE[v.state_of(l0)] == COL_STATE["conflict"]
    assert "CONFLICT" in w.perm_status.text()
    v.undo()
    assert v.diagnosis is not None and v.diagnosis.status == "well"
    # remove a dimension → under
    dist = next(c for c in v.sketch.constraints if isinstance(c, C.Distance))
    v.remove_constraint(dist)
    assert v.diagnosis is not None and v.diagnosis.status == "under" and v.diagnosis.dof == 1
    assert any(v.state_of(e) == "under" for e in v.sketch.points)
