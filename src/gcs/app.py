"""Desktop sketcher (PySide6).

    python -m gcs.app [sketch.json]

Draw points / lines / circles / arcs, select entities and apply constraints
from the toolbar, drag points and watch the solver keep everything satisfied.
Tools:  S select · P point · L line · C circle · A arc · Esc cancel
        F fix/unfix point · Del delete · Ctrl+Z undo · wheel zoom · right/middle-drag pan
"""

from __future__ import annotations

import math
import sys
from collections import deque
from dataclasses import dataclass
from typing import Any

import numpy as np
import numpy.typing as npt
from PySide6.QtCore import QPointF, QRectF, Qt, QTimer, Signal
from PySide6.QtGui import (
    QAction, QActionGroup, QColor, QKeySequence, QMouseEvent, QPainter, QPaintEvent, QPen, QWheelEvent,
)
from PySide6.QtWidgets import (
    QApplication, QComboBox, QDialog, QDialogButtonBox, QDockWidget, QFileDialog, QHBoxLayout, QInputDialog,
    QLabel, QListWidget, QListWidgetItem, QMainWindow, QMessageBox, QPlainTextEdit, QPushButton, QToolBar,
    QVBoxLayout, QWidget,
)

from gcs import constraints as C
from gcs import io
from gcs.constraints import ENTITY_KINDS, Constraint
from gcs.decompose import PlanDrag, PlanResult, PlanSolver, ppp_triangles
from gcs.diagnose import Diagnosis, diagnose
from gcs.homotopy import apply_alternative, enumerate_step
from gcs.examples import CASES, EXAMPLES
from gcs.model import Arc, Circle, Line, Point, Primitive, Sketch, Vec, expand
from gcs.solve import METHODS, Method, SolveResult, System
from gcs.witness import Motion, WitnessReport, analyze

PICK_PX = 8.0

COL_BG = QColor("#fafafa")
COL_AXIS = QColor("#dddddd")
COL_LINE = QColor("#1f77b4")
COL_CIRC = QColor("#2ca02c")
COL_ARC = QColor("#ff7f0e")
COL_PT = QColor("#222222")
COL_FIXED = QColor("#d62728")
COL_SEL = QColor("#e377c2")
COL_PREVIEW = QColor("#999999")
COL_HL = QColor("#9467bd")
COL_ROW_SEL = QColor("#fce4f3")   # list rows whose constraint touches a selected entity
COL_ROW_TEXT = QColor("#000000")
# entity colouring by constraint state (FreeCAD-style, but from the DM decomposition + conflict set)
COL_STATE = {"well": QColor("#2ca02c"), "under": QColor("#e69500"), "over": QColor("#d62728"),
             "conflict": QColor("#d62728")}
COL_ROW_OVER = QColor("#e69500")
COL_CONFLICT_GLYPH = QColor("#b3001b")


_ANIM_DT = 0.03            # seconds per animation tick (also the timer interval)
_ANIM_PERIOD = 2.0         # seconds spent on each degree of freedom


@dataclass
class _Animation:
    """Everything a DOF-animation tick needs, computed once when it starts."""

    modes: list[Motion]
    x0: Vec
    free: npt.NDArray[np.intp]
    amp: float
    labels: list[str]
    t: float = 0.0
    showing: int = -1


class SketchView(QWidget):
    """World ↔ screen mapping, painting, hit-testing and the drawing tools."""

    changed = Signal()          # sketch topology/values changed (refresh lists, status)
    status = Signal(str)        # transient message

    def __init__(self, sketch: Sketch) -> None:
        super().__init__()
        self.sketch = sketch
        self.scale = 6.0            # px per world unit
        self.origin = QPointF(80, 500)  # screen position of world (0,0)
        self.tool = "select"
        self.method: Method = "dogleg"
        self.auto_solve = True
        self.use_plan = False                   # Stage 3: decomposition plan (numeric fallback) for solves
        self.last_plan: PlanResult | None = None
        self._plan_solver: PlanSolver | None = None
        self._plan_key: object = None
        self._flips_reported = 0
        # Stage 4: witness analysis (cached per diagnosis) and DOF animation
        self._witness: WitnessReport | None = None
        self._witness_for: Diagnosis | None = None
        self.anim_timer = QTimer(self)
        self.anim_timer.timeout.connect(self._anim_tick)
        self.anim: _Animation | None = None
        self.pending: list[Point] = []          # points clicked so far in a drawing tool
        self.cursor_s = QPointF(0, 0)           # screen coords of cursor
        self.selected: list[Primitive] = []
        self.highlight: list[Primitive] = []    # entities of the constraint selected in the list
        self.undo_stack: deque[str] = deque(maxlen=100)
        self.last_result: SolveResult | None = None
        self.diagnosis: Diagnosis | None = None
        self.color_by_state = True
        self._rediagnose(None)
        self.drag: PlanDrag | None = None
        self.pan_last: QPointF | None = None
        self.setMouseTracking(True)
        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)
        self.setMinimumSize(400, 300)

    # -- coordinates --------------------------------------------------------

    def w2s(self, x: float, y: float) -> QPointF:
        return QPointF(self.origin.x() + x * self.scale, self.origin.y() - y * self.scale)

    def s2w(self, p: QPointF) -> tuple[float, float]:
        return ((p.x() - self.origin.x()) / self.scale, (self.origin.y() - p.y()) / self.scale)

    def fit(self) -> None:
        if not self.sketch.points:
            return
        x0, y0, x1, y1 = self.sketch.bbox()
        self.scale = 0.8 * min(self.width() / (x1 - x0 or 1.0), self.height() / (y1 - y0 or 1.0))
        cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
        self.origin = QPointF(self.width() / 2 - cx * self.scale, self.height() / 2 + cy * self.scale)
        self.update()

    # -- sketch mutation ----------------------------------------------------

    def set_sketch(self, sk: Sketch, *, fit: bool = True) -> None:
        self.sketch = sk
        self.selected, self.highlight, self.pending = [], [], []
        self.drag = None
        self._after_edit()
        if fit:
            self.fit()      # after the solve: loading a case can move the geometry a long way

    def push_undo(self) -> None:
        self.undo_stack.append(io.dumps(self.sketch))

    def undo(self) -> None:
        if self.undo_stack:
            self.set_sketch(io.loads(self.undo_stack.pop()), fit=False)
            self.status.emit("undo")

    def _plan(self) -> PlanSolver:
        """The decomposition plan, compiled once per topology (constraints, entities, fixed flags)
        and replayed for dimension edits / drags."""
        sk = self.sketch
        key = (tuple(id(c) for c in sk.constraints), tuple(p.fixed for p in sk.params),
               len(sk.points), len(sk.lines), len(sk.circles), len(sk.arcs))
        if self._plan_solver is None or key != self._plan_key:
            self._plan_solver, self._plan_key = PlanSolver(sk, sticky=True), key
        return self._plan_solver

    def _solve(self) -> tuple[SolveResult, System]:
        """One solve by the selected path; returns the result and the compiled System."""
        if self.use_plan and self.sketch.constraints:
            ps = self._plan()
            self.last_plan = ps.solve(method=self.method)     # reads/records sketch.branches
            return self.last_plan.as_solve_result(), ps.system
        self.last_plan = None
        system = System(self.sketch)
        return system.solve(method=self.method), system

    def solve_now(self) -> SolveResult:
        self.last_result, _ = self._solve()
        self.changed.emit()
        self.update()
        return self.last_result

    def _rediagnose(self, system: System | None) -> None:
        self.diagnosis = diagnose(self.sketch, system=system) if self.sketch.constraints else None

    def _after_edit(self) -> SolveResult | None:
        """Every mutation ends here: re-solve (if auto), re-diagnose, then notify listeners once.
        A failed solve leaves the last good geometry on screen (the failure is reported by
        the diagnosis, not by exploded geometry — which would also mislead the conflict search)."""
        system: System | None = None
        if self.auto_solve:
            x_before = self.sketch.get_x()
            self.last_result, system = self._solve()
            if not self.last_result.success:
                self.sketch.set_x(x_before)
        self._rediagnose(system)   # reuses the compiled system when we have one
        self.changed.emit()
        self.update()
        return self.last_result

    # -- Stage 4: witness analysis and DOF animation --------------------------

    def witness_report(self) -> WitnessReport | None:
        """Witness analysis of the current sketch, cached until the next edit.  The structural
        diagnosis is already in hand, so this only adds the Jacobian work — no second diagnose."""
        d = self.diagnosis
        if d is None:
            return None
        if self._witness_for is not d:
            self._witness_for = d
            self._witness = analyze(self.sketch, over_ids=frozenset(id(c) for c in d.over))
        return self._witness

    def start_animation(self) -> bool:
        """Animate the remaining internal DOFs (each null-space mode in turn); False if none."""
        rep = self.witness_report()
        modes = [m for m in rep.motions if not m.rigid] if rep is not None else []
        if not modes:
            return False
        # everything the ticks need, computed once
        self.anim = _Animation(modes, self.sketch.get_x(), self.sketch.free_indices(),
                               0.06 * self.sketch.extent(),
                               [", ".join(p.name for p in m.moving_params()[:8]) for m in modes])
        self.anim_timer.start(int(_ANIM_DT * 1000))
        self.status.emit(f"animating {len(modes)} remaining DOF (click or Esc to stop)")
        return True

    def stop_animation(self) -> None:
        self.anim_timer.stop()
        if self.anim is not None:
            self.sketch.set_x(self.anim.x0)
            self.anim = None
            self.update()

    def _anim_tick(self) -> None:
        a = self.anim
        if a is None:
            return
        a.t += _ANIM_DT
        k = int(a.t // _ANIM_PERIOD) % len(a.modes)
        phase = math.sin(2 * math.pi * (a.t % _ANIM_PERIOD) / _ANIM_PERIOD)
        x = a.x0.copy()
        x[a.free] += a.amp * phase * a.modes[k].velocity
        self.sketch.set_x(x)
        if k != a.showing:
            a.showing = k
            self.status.emit(f"DOF {k + 1}/{len(a.modes)}: {a.labels[k]}")
        self.update()

    def state_of(self, e: Primitive) -> str:
        return self.diagnosis.entity_state.get(id(e), "well") if self.diagnosis else "well"

    def add_constraint(self, c: Constraint) -> None:
        self.push_undo()
        self.sketch.add(c)
        res = self._after_edit()
        d = self.diagnosis
        st = d.status if d is not None else "well"
        if st == "conflict" and d is not None and d.conflicts:
            self.status.emit(f"added {type(c).__name__} — CONFLICT, remove one of: "
                             + ", ".join(io.describe(k, self.sketch) for k in d.conflicts))
        elif st == "over":
            self.status.emit(f"added {type(c).__name__} — redundant (consistent) with existing constraints")
        elif res is not None and not res.success:
            self.status.emit(f"added {type(c).__name__} — solver did NOT converge")
        else:
            self.status.emit(f"added {type(c).__name__}")

    def remove_constraint(self, c: Constraint) -> None:
        self.push_undo()
        self.sketch.remove(c)
        self._after_edit()

    def delete_selected(self) -> None:
        if not self.selected:
            return
        self.push_undo()
        n = len(self.selected)
        self.set_sketch(io.without(self.sketch, entities=self.selected), fit=False)
        self.status.emit(f"deleted {n} entities")

    def toggle_fix_selected(self) -> None:
        pts = [e for e in self.selected if isinstance(e, Point)]
        if not pts:
            return
        self.push_undo()
        all_fixed = all(p.is_fixed for p in pts)
        for p in pts:
            p.fix(not all_fixed)
        self._after_edit()

    # -- hit testing --------------------------------------------------------

    def _pick_point(self, sp: QPointF, tol: float = PICK_PX) -> Point | None:
        p, d = self.sketch.nearest_point(*self.s2w(sp))
        return p if d * self.scale < tol else None

    def pick(self, sp: QPointF) -> Primitive | None:
        p = self._pick_point(sp)
        if p is not None:
            return p
        best: Primitive | None = None
        bd = PICK_PX
        for ln in self.sketch.lines:
            d = _seg_dist(sp, self.w2s(*ln.p1.xy), self.w2s(*ln.p2.xy))
            if d < bd:
                best, bd = ln, d
        for c in self.sketch.circles:
            d = abs(_dist(sp, self.w2s(*c.center.xy)) - abs(c.radius.value) * self.scale)
            if d < bd:
                best, bd = c, d
        for a in self.sketch.arcs:
            cs = self.w2s(*a.center.xy)
            d = abs(_dist(sp, cs) - abs(a.radius.value) * self.scale)
            if d < bd:
                ang = math.atan2(-(sp.y() - cs.y()), sp.x() - cs.x())
                a0, a1 = a.angles()
                while ang < a0:
                    ang += 2 * math.pi
                if ang <= a1:
                    best, bd = a, d
        return best

    # -- painting -----------------------------------------------------------

    def paintEvent(self, _e: QPaintEvent) -> None:  # noqa: N802
        qp = QPainter(self)
        qp.setRenderHint(QPainter.RenderHint.Antialiasing)
        qp.fillRect(self.rect(), COL_BG)
        qp.setPen(QPen(COL_AXIS, 1))
        o = self.w2s(0, 0)
        qp.drawLine(QPointF(0, o.y()), QPointF(self.width(), o.y()))
        qp.drawLine(QPointF(o.x(), 0), QPointF(o.x(), self.height()))
        sk = self.sketch
        sel, hl = set(self.selected), set(self.highlight)

        def pen(col: QColor, ent: Primitive, w: float = 1.8) -> QPen:
            if ent in sel:
                return QPen(COL_SEL, w + 1.5)
            if ent in hl:
                return QPen(COL_HL, w + 1.0)
            return QPen(COL_STATE[self.state_of(ent)] if self.color_by_state else col, w)

        for ln in sk.lines:
            qp.setPen(pen(COL_LINE, ln))
            qp.drawLine(self.w2s(*ln.p1.xy), self.w2s(*ln.p2.xy))
        for c in sk.circles:
            qp.setPen(pen(COL_CIRC, c))
            r = abs(c.radius.value) * self.scale
            qp.drawEllipse(self.w2s(*c.center.xy), r, r)
        for a in sk.arcs:
            qp.setPen(pen(COL_ARC, a))
            _draw_arc(qp, self.w2s(*a.center.xy), abs(a.radius.value) * self.scale, *a.angles())
        if self.pending:
            self._paint_preview(qp)
        if self.diagnosis is not None and self.diagnosis.conflicts:
            self._paint_conflicts(qp)
        for p in sk.points:
            s = self.w2s(*p.xy)
            col = (COL_SEL if p in sel else COL_HL if p in hl else COL_FIXED if p.is_fixed
                   else COL_STATE[self.state_of(p)] if self.color_by_state else COL_PT)
            qp.setPen(QPen(col, 1))
            qp.setBrush(col)
            if p.is_fixed:
                qp.drawRect(QRectF(s.x() - 4, s.y() - 4, 8, 8))
            else:
                qp.drawEllipse(s, 3.5, 3.5)
        if self.tool != "select":  # snap indicator
            sp = self._pick_point(self.cursor_s)
            if sp is not None:
                qp.setPen(QPen(COL_SEL, 1.5))
                qp.setBrush(Qt.BrushStyle.NoBrush)
                qp.drawEllipse(self.w2s(*sp.xy), 7, 7)
        qp.end()

    def _paint_conflicts(self, qp: QPainter) -> None:
        """Dashed red halo on every entity referenced by a culprit constraint, and a ✗
        glyph at each culprit's anchor — the culprits are what to remove, as opposed to
        geometry that merely turned red because it touches them."""
        assert self.diagnosis is not None
        halo = QPen(COL_CONFLICT_GLYPH, 6, Qt.PenStyle.DashLine)
        qp.setBrush(Qt.BrushStyle.NoBrush)
        used: dict[tuple[int, int], int] = {}   # anchor cell → labels already placed there (stack them)
        for c in self.diagnosis.conflicts or []:
            xs: list[float] = []
            ys: list[float] = []
            for e in c.entities():
                qp.setPen(halo)
                if isinstance(e, Point):
                    qp.drawEllipse(self.w2s(*e.xy), 9, 9)
                    xs.append(e.x.value); ys.append(e.y.value)
                elif isinstance(e, Line):
                    qp.drawLine(self.w2s(*e.p1.xy), self.w2s(*e.p2.xy))
                    xs += [e.p1.x.value, e.p2.x.value]; ys += [e.p1.y.value, e.p2.y.value]
                elif isinstance(e, Circle):
                    r = abs(e.radius.value) * self.scale
                    qp.drawEllipse(self.w2s(*e.center.xy), r, r)
                    xs.append(e.center.x.value); ys.append(e.center.y.value + e.radius.value)
                elif isinstance(e, Arc):
                    _draw_arc(qp, self.w2s(*e.center.xy), abs(e.radius.value) * self.scale, *e.angles())
                    a0, a1 = e.angles()
                    am = 0.5 * (a0 + a1)
                    xs.append(e.center.x.value + e.radius.value * math.cos(am))
                    ys.append(e.center.y.value + e.radius.value * math.sin(am))
            if xs:
                anchor = self.w2s(sum(xs) / len(xs), sum(ys) / len(ys))
                cell = (int(anchor.x() // 40), int(anchor.y() // 40))
                n = used.get(cell, 0)
                used[cell] = n + 1
                f = qp.font()
                f.setPointSize(13)
                f.setBold(True)
                qp.setFont(f)
                qp.setPen(QPen(COL_CONFLICT_GLYPH, 1))
                qp.drawText(anchor + QPointF(8, -8 - 18 * n), f"✗ {io.describe(c, self.sketch)}")

    def _paint_preview(self, qp: QPainter) -> None:
        qp.setPen(QPen(COL_PREVIEW, 1, Qt.PenStyle.DashLine))
        p0 = self.w2s(*self.pending[0].xy)
        cur = self.cursor_s
        if self.tool == "line":
            qp.drawLine(self.w2s(*self.pending[-1].xy), cur)
        elif self.tool == "circle":
            r = _dist(p0, cur)
            qp.drawEllipse(p0, r, r)
        elif self.tool == "arc":
            if len(self.pending) == 1:
                qp.drawLine(p0, cur)
            else:
                ps = self.w2s(*self.pending[1].xy)
                a0 = math.atan2(-(ps.y() - p0.y()), ps.x() - p0.x())
                a1 = math.atan2(-(cur.y() - p0.y()), cur.x() - p0.x())
                _draw_arc(qp, p0, _dist(p0, ps), a0, a1 if a1 > a0 else a1 + 2 * math.pi)

    # -- mouse / keyboard ---------------------------------------------------

    def mousePressEvent(self, e: QMouseEvent) -> None:  # noqa: N802
        self.stop_animation()
        sp = e.position()
        self.cursor_s = sp
        if e.button() in (Qt.MouseButton.MiddleButton, Qt.MouseButton.RightButton):
            if self.pending:
                self.cancel_tool()
            else:
                self.pan_last = sp
            return
        if e.button() != Qt.MouseButton.LeftButton:
            return
        if self.tool != "select":
            self._tool_click(sp)
            return
        ent = self.pick(sp)
        shift = bool(e.modifiers() & Qt.KeyboardModifier.ShiftModifier)
        if ent is None:
            if not shift:
                self.selected = []
        elif shift:
            if ent in self.selected:
                self.selected.remove(ent)
            else:
                self.selected.append(ent)
        else:
            if ent not in self.selected:
                self.selected = [ent]
            if isinstance(ent, Point) and not ent.is_fixed:
                self.push_undo()
                cached = self._plan_solver
                guards = ppp_triangles(cached.plan) if cached is not None else None
                self.drag = PlanDrag(self.sketch, ent, *self.s2w(sp), guards=guards)
                self._flips_reported = 0
        self.changed.emit()
        self.update()

    def _snap_or_new(self, sp: QPointF) -> Point:
        return self._pick_point(sp) or self.sketch.point(*self.s2w(sp))

    def _tool_click(self, sp: QPointF) -> None:
        sk = self.sketch
        if not self.pending:
            self.push_undo()
        if self.tool == "point":
            self._snap_or_new(sp)
        elif self.tool == "line":
            p = self._snap_or_new(sp)
            if self.pending and p is not self.pending[-1]:
                sk.line(self.pending[-1], p)
            self.pending = [p]  # continue the polyline
        elif self.tool == "circle":
            if not self.pending:
                self.pending = [self._snap_or_new(sp)]
            else:
                x, y = self.s2w(sp)
                c = self.pending[0]
                sk.circle(c, math.hypot(x - c.x.value, y - c.y.value) or 1.0)
                self.pending = []
        elif self.tool == "arc":
            existing = self._pick_point(sp) is not None
            self.pending.append(self._snap_or_new(sp))
            if len(self.pending) == 3:
                cpt, s, en = self.pending
                if len({id(cpt), id(s), id(en)}) == 3:
                    if not existing:  # freshly placed end point: put it exactly on the radius
                        r = math.hypot(s.x.value - cpt.x.value, s.y.value - cpt.y.value)
                        ang = math.atan2(en.y.value - cpt.y.value, en.x.value - cpt.x.value)
                        en.x.value, en.y.value = cpt.x.value + r * math.cos(ang), cpt.y.value + r * math.sin(ang)
                    sk.arc(cpt, s, en)
                self.pending = []
        self._after_edit()  # (an arc ending on an existing point is reconciled by the solve)

    def cancel_tool(self) -> None:
        self.stop_animation()
        self.pending = []
        self.update()

    def set_tool(self, tool: str) -> None:
        self.tool = tool
        self.pending = []
        self.status.emit(f"tool: {tool}")
        self.update()

    def mouseMoveEvent(self, e: QMouseEvent) -> None:  # noqa: N802
        sp = e.position()
        self.cursor_s = sp
        if self.pan_last is not None:
            self.origin += sp - self.pan_last
            self.pan_last = sp
        elif self.drag is not None:
            self.last_result = self.drag.move(*self.s2w(sp))
            if len(self.drag.flips) > self._flips_reported:      # only announce new ones
                self._flips_reported = len(self.drag.flips)
                self.status.emit(f"⚠ solution branch flipped in {self._flips_reported} triangle(s) during this drag")
            self.changed.emit()
        self.update()

    def mouseReleaseEvent(self, _e: QMouseEvent) -> None:  # noqa: N802
        self.pan_last = None
        if self.drag is not None:
            self.drag.end()
            self.sketch.branches.update(self.drag.branches())
            self.drag = None
            self.changed.emit()
            self.update()

    def wheelEvent(self, e: QWheelEvent) -> None:  # noqa: N802
        f = 1.0015 ** e.angleDelta().y()
        sp = e.position()
        self.origin = sp + (self.origin - sp) * f
        self.scale *= f
        self.update()


def _draw_arc(qp: QPainter, center: QPointF, r: float, a0: float, a1: float) -> None:
    """CCW arc from angle a0 to a1 (radians, math convention) — Qt takes 1/16 degrees."""
    qp.drawArc(QRectF(center.x() - r, center.y() - r, 2 * r, 2 * r),
               int(math.degrees(a0) * 16), int(math.degrees(a1 - a0) * 16))


def _dist(a: QPointF, b: QPointF) -> float:
    return math.hypot(a.x() - b.x(), a.y() - b.y())


def _seg_dist(p: QPointF, a: QPointF, b: QPointF) -> float:
    ab = b - a
    L2 = ab.x() ** 2 + ab.y() ** 2
    t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((p.x() - a.x()) * ab.x() + (p.y() - a.y()) * ab.y()) / L2))
    return _dist(p, a + ab * t)


# ---------------------------------------------------------------------------


class MainWindow(QMainWindow):
    def __init__(self, sketch: Sketch | None = None) -> None:
        super().__init__()
        self.setWindowTitle("gcs sketcher")
        self.resize(1200, 800)
        self.view = SketchView(sketch or Sketch())
        central = QWidget()
        lay = QVBoxLayout(central)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(0)
        self.banner = QWidget()
        self.banner.setStyleSheet("background:#fdecea; border-bottom:1px solid #d62728;")
        bl = QHBoxLayout(self.banner)
        bl.setContentsMargins(10, 4, 10, 4)
        self.banner_text = QLabel()
        self.banner_text.setStyleSheet("color:#8a0000; font-weight:bold;")
        self.banner_text.setWordWrap(True)
        bl.addWidget(self.banner_text, 1)
        self.banner_select = QPushButton("Select culprits")
        self.banner_select.clicked.connect(self.select_conflicts)
        bl.addWidget(self.banner_select)
        self.banner_details = QPushButton("Details…")
        self.banner_details.clicked.connect(self.show_diagnosis)
        bl.addWidget(self.banner_details)
        self.banner.hide()
        lay.addWidget(self.banner)
        lay.addWidget(self.view, 1)
        self.setCentralWidget(central)
        self.perm_status = QLabel()
        self.statusBar().addPermanentWidget(self.perm_status)
        self._rows: list[Constraint] = []   # constraints currently shown in the list, in order
        self._build_menus()
        self._build_toolbar()
        self._build_dock()
        self.view.changed.connect(self.refresh)
        self.view.status.connect(lambda m: self.statusBar().showMessage(m, 4000))
        self.refresh()

    # -- ui construction ----------------------------------------------------

    def _act(self, text: str, slot: Any, key: str | None = None, checkable: bool = False) -> QAction:
        a = QAction(text, self)
        if key:
            a.setShortcut(QKeySequence(key))
        a.setCheckable(checkable)
        a.triggered.connect(slot)
        return a

    def _build_menus(self) -> None:
        mb = self.menuBar()
        fm = mb.addMenu("&File")
        fm.addAction(self._act("&New", self.new_sketch, "Ctrl+N"))
        fm.addAction(self._act("&Open…", self.open_sketch, "Ctrl+O"))
        fm.addAction(self._act("&Save As…", self.save_sketch, "Ctrl+S"))
        fm.addSeparator()
        ex = fm.addMenu("&Examples")
        for name in CASES:
            ex.addAction(self._act(name, lambda _=False, n=name: self.load_case(n)))
        fm.addSeparator()
        fm.addAction(self._act("&Quit", self.close, "Ctrl+Q"))

        em = mb.addMenu("&Edit")
        em.addAction(self._act("&Undo", self.view.undo, "Ctrl+Z"))
        em.addAction(self._act("&Delete selected", self.delete_pressed, "Backspace"))
        em.addAction(self._act("Delete selected (Del)", self.delete_pressed, "Delete"))
        em.addAction(self._act("Toggle &fix on selected points", self.view.toggle_fix_selected, "F"))
        em.addAction(self._act("Select &all", self.select_all, "Ctrl+A"))
        em.addAction(self._act("Flip solution &branch", self.flip_branch, "Ctrl+F"))
        em.addAction(self._act("Alternative &solutions for selected point…", self.alternatives, "Ctrl+Shift+F"))

        vm = mb.addMenu("&View")
        vm.addAction(self._act("&Fit", self.view.fit, "Home"))
        col = self._act("Colour by &constraint state", self.toggle_colour, checkable=True)
        col.setChecked(True)
        vm.addAction(col)
        vm.addAction(self._act("&Animate remaining DOF", self.animate_dof, "Ctrl+M"))

        sm = mb.addMenu("&Solve")
        sm.addAction(self._act("Solve &now", self.view.solve_now, "Return"))
        sm.addAction(self._act("&Diagnose…", self.show_diagnosis, "D"))
        auto = self._act("&Auto-solve", self.toggle_auto, checkable=True)
        auto.setChecked(True)
        sm.addAction(auto)
        plan = self._act("Use decomposition &plan (Stage 3)", self.toggle_plan, checkable=True)
        sm.addAction(plan)
        sm.addSeparator()
        grp = QActionGroup(self)
        for m in METHODS:
            a = self._act(m, lambda _=False, mm=m: self.set_method(mm), checkable=True)
            a.setChecked(m == self.view.method)
            grp.addAction(a)
            sm.addAction(a)

    def _build_toolbar(self) -> None:
        tb = QToolBar("Tools")
        tb.setMovable(False)
        self.addToolBar(tb)
        self.tool_group = QActionGroup(self)
        for text, tool, key in (("Select", "select", "S"), ("Point", "point", "P"), ("Line", "line", "L"),
                                ("Circle", "circle", "C"), ("Arc", "arc", "A")):
            a = self._act(text, lambda _=False, t=tool: self.view.set_tool(t), key, checkable=True)
            a.setChecked(tool == "select")
            self.tool_group.addAction(a)
            tb.addAction(a)
        tb.addAction(self._act("Cancel", self.view.cancel_tool, "Escape"))
        tb.addSeparator()
        tb.addWidget(QLabel(" Case: "))
        self.case_box = QComboBox()
        self.case_box.setMinimumWidth(240)
        self.case_box.addItem("— load a test case —")
        for name, (_, desc) in CASES.items():
            self.case_box.addItem(name)
            self.case_box.setItemData(self.case_box.count() - 1, desc, Qt.ItemDataRole.ToolTipRole)
        self.case_box.activated.connect(self._case_chosen)
        tb.addWidget(self.case_box)

        self.addToolBarBreak()
        cb = QToolBar("Constraints")
        cb.setMovable(False)
        self.addToolBar(cb)
        # simple constraints: (label, class, needed points, lines, circles/arcs, apply to each line?)
        simple: list[tuple[str, type[Constraint], int, int, int]] = [
            ("Coincident", C.Coincident, 2, 0, 0), ("Horizontal", C.Horizontal, 0, 1, 0),
            ("Vertical", C.Vertical, 0, 1, 0), ("Parallel", C.Parallel, 0, 2, 0),
            ("Perpendicular", C.Perpendicular, 0, 2, 0), ("On line", C.PointOnLine, 1, 1, 0),
            ("Midpoint", C.Midpoint, 1, 1, 0), ("On circle", C.PointOnCircle, 1, 0, 1),
        ]
        actions: list[tuple[str, Any]] = [
            (label, lambda _=False, cls=cls, n=(np_, nl, nc): self._apply(cls, *n)) for label, cls, np_, nl, nc in simple
        ]
        actions[1:1] = [("Distance", self.c_distance)]
        actions += [("Angle", self.c_angle), ("Equal", self.c_equal), ("Tangent", self.c_tangent),
                    ("Radius", self.c_radius), ("Fix", self.view.toggle_fix_selected)]
        for text, fn in actions:
            cb.addAction(self._act(text, fn))

    def _build_dock(self) -> None:
        dock = QDockWidget("Constraints", self)
        dock.setFeatures(QDockWidget.DockWidgetFeature.NoDockWidgetFeatures)
        self.clist = QListWidget()
        self.clist.currentItemChanged.connect(self.on_constraint_selected)
        self.clist.itemDoubleClicked.connect(self.edit_constraint_value)
        dock.setWidget(self.clist)
        self.addDockWidget(Qt.DockWidgetArea.RightDockWidgetArea, dock)

    # -- refresh ------------------------------------------------------------

    def refresh(self) -> None:
        """Sync the constraint list and status bar with the sketch.

        Rows are rebuilt only when the set of user constraints changed; during a
        drag (every mouse move) we just recolour rows and update the status text."""
        sk = self.view.sketch
        rows = sk.user_constraints()
        if rows != self._rows:
            self._rebuild_rows(rows)
        sel_direct = set(self.view.selected)
        sel_all = set(expand(self.view.selected))
        d = self.view.diagnosis
        bad_ids = {id(c) for c in [*(d.conflicts or []), *d.violated]} if d else set()
        over_ids = {id(c) for c in d.over} if d else set()
        culprit_ids = {id(c) for c in (d.conflicts or [])} if d else set()
        for i, c in enumerate(rows):
            item = self.clist.item(i)
            base = item.data(Qt.ItemDataRole.UserRole + 1)
            if id(c) in culprit_ids:
                item.setText(f"✗ {base}")
                item.setForeground(COL_CONFLICT_GLYPH)
            elif id(c) in bad_ids:
                item.setText(base)
                item.setForeground(COL_FIXED)
            elif id(c) in over_ids:
                item.setText(f"≈ {base}")
                item.setForeground(COL_ROW_OVER)
            else:
                item.setText(base)
                item.setForeground(COL_ROW_TEXT)
            hit = bool(sel_all) and (any(e in sel_all for e in c.entities())
                                     or any(e in sel_direct for e in expand(c.entities())))
            item.setBackground(COL_ROW_SEL if hit else Qt.GlobalColor.transparent)
            f = item.font()
            f.setBold(hit)
            item.setFont(f)
        r = self.view.last_result
        msg = (f"points {len(sk.points)}  lines {len(sk.lines)}  circles {len(sk.circles)}  arcs {len(sk.arcs)}   "
               f"| params {len(sk.params)} (free {len(sk.free_indices())})  equations {sk.n_residuals()}")
        if d is not None:
            msg += f"  DOF {d.dof}"
            if d.n_redundant:
                msg += f"  redundant {d.n_redundant}"
            if d.status == "conflict":
                msg += "  ⚠ CONFLICT"
            elif d.numeric_rank is not None and d.numeric_rank < d.structural_rank:
                msg += "  ⚠ geometric dependency"
            elif d.numeric_rank is None and d.warnings:
                msg += "  (structural only)"
        msg += f"   | selected {len(self.view.selected)}"
        if r is not None:
            msg += (f"   | {'solved' if r.success else 'NOT CONVERGED'}  max|r|={r.max_residual:.1e}  "
                    f"{r.time_s * 1e3:.1f} ms  nfev={r.nfev}  {r.method}")
        pr = self.view.last_plan
        if pr is not None:
            msg += f"   | plan: {pr.plan.summary()}{' (fell back)' if pr.fell_back else ''}"
        self._refresh_banner(d)
        title = "gcs sketcher"
        if d is not None and d.status == "conflict":
            title += "  —  ⚠ conflicting constraints"
        elif r is not None and not r.success:
            title += "  —  ⚠ not converged"
        self.setWindowTitle(title)
        self.perm_status.setText(msg)

    def _refresh_banner(self, d: Diagnosis | None) -> None:
        if d is None or d.status not in ("conflict", "over"):
            self.banner.hide()
            return
        ix = io.Index(self.view.sketch)
        if d.status == "conflict":
            if d.conflicts:
                names = ", ".join(io.describe(c, ix) for c in d.conflicts)
                self.banner_text.setText(f"⚠ Conflicting constraints — remove one of: {names}")
            else:
                self.banner_text.setText(f"⚠ {len(d.violated)} constraint(s) cannot be satisfied")
            self.banner.setStyleSheet("background:#fdecea; border-bottom:1px solid #d62728;")
            self.banner_text.setStyleSheet("color:#8a0000; font-weight:bold;")
        else:
            names = ", ".join(io.describe(c, ix) for c in d.over)
            self.banner_text.setText(f"⚠ {d.n_redundant} redundant equation(s) (consistent, but over-constrained) among: {names}")
            self.banner.setStyleSheet("background:#fff4e0; border-bottom:1px solid #e69500;")
            self.banner_text.setStyleSheet("color:#7a4a00; font-weight:bold;")
        self.banner_select.setVisible(bool(d.conflicts) or d.status == "over")
        self.banner.show()

    def select_conflicts(self) -> None:
        """Select the culprit rows in the list and their entities in the view."""
        d = self.view.diagnosis
        if d is None:
            return
        culprits = d.conflicts or d.over
        ids = {id(c) for c in culprits}
        self.clist.clearSelection()
        for i in range(self.clist.count()):
            item = self.clist.item(i)
            item.setSelected(id(item.data(Qt.ItemDataRole.UserRole)) in ids)
        first = next((i for i in range(self.clist.count())
                      if id(self.clist.item(i).data(Qt.ItemDataRole.UserRole)) in ids), -1)
        if first >= 0:
            self.clist.setCurrentRow(first)
        self.view.selected = list(dict.fromkeys(e for c in culprits for e in c.entities()))
        self.view.update()
        self.refresh()

    def _rebuild_rows(self, rows: list[Constraint]) -> None:
        cur = self.clist.currentItem().data(Qt.ItemDataRole.UserRole) if self.clist.currentItem() else None
        ix = io.Index(self.view.sketch)
        self.clist.blockSignals(True)
        self.clist.clear()
        for c in rows:
            text = io.describe(c, ix)
            item = QListWidgetItem(text)
            item.setData(Qt.ItemDataRole.UserRole, c)
            item.setData(Qt.ItemDataRole.UserRole + 1, text)   # base text (prefixes are added by refresh)
            self.clist.addItem(item)
            if c is cur:
                self.clist.setCurrentItem(item)
        self.clist.blockSignals(False)
        self._rows = list(rows)

    # -- file ---------------------------------------------------------------

    def new_sketch(self) -> None:
        self.view.set_sketch(Sketch())

    def load_case(self, name: str) -> None:
        make, desc = CASES[name]
        self.view.set_sketch(make())
        i = self.case_box.findText(name)
        if i >= 0:
            self.case_box.setCurrentIndex(i)
        self.statusBar().showMessage(f"{name}: {desc}", 8000)

    def _case_chosen(self, index: int) -> None:
        if index > 0:
            self.load_case(self.case_box.itemText(index))

    def open_sketch(self) -> None:
        path, _ = QFileDialog.getOpenFileName(self, "Open sketch", "", "Sketch JSON (*.json)")
        if path:
            try:
                self.view.set_sketch(io.load(path))
            except Exception as ex:  # noqa: BLE001
                QMessageBox.critical(self, "Open failed", str(ex))

    def save_sketch(self) -> None:
        path, _ = QFileDialog.getSaveFileName(self, "Save sketch", "sketch.json", "Sketch JSON (*.json)")
        if path:
            io.save(self.view.sketch, path)
            self.statusBar().showMessage(f"saved {path}", 4000)

    def select_all(self) -> None:
        sk = self.view.sketch
        self.view.selected = [*sk.points, *sk.lines, *sk.circles, *sk.arcs]
        self.view.update()
        self.refresh()

    def toggle_auto(self, on: bool) -> None:
        self.view.auto_solve = on

    def toggle_plan(self, on: bool) -> None:
        self.view.use_plan = on
        self.view.solve_now()

    def flip_branch(self) -> None:
        """Stage 5 root selection: a selected tangency row toggles its inside/outside flag; a
        selected point flips the closed-form constructions that place it (the other circle–circle
        intersection), recorded in the sketch's branches and replayed sticky."""
        item = self.clist.currentItem()
        c = item.data(Qt.ItemDataRole.UserRole) if item is not None and self.clist.hasFocus() else None
        if isinstance(c, C.TangentLineCircle):
            self.view.push_undo()
            c.side = -c.side
            self.view._after_edit()
            self.statusBar().showMessage(f"flipped tangency side of {io.describe(c, self.view.sketch)}", 4000)
            return
        if isinstance(c, C.TangentCircleCircle):
            self.view.push_undo()
            c.external = not c.external
            self.view._after_edit()
            self.statusBar().showMessage(f"flipped {'external' if c.external else 'internal'} tangency", 4000)
            return
        pts = [e for e in self.view.selected if isinstance(e, Point)]
        if not pts:
            self.statusBar().showMessage("select a point (or a tangency row) to flip its solution branch", 4000)
            return
        sk = self.view.sketch
        ps = self.view._plan()                    # the plan cached for this topology
        self.view.push_undo()
        n = sum(ps.flip(ps.graph.P(p)) for p in pts)
        if not n:
            self.statusBar().showMessage("no closed-form construction places the selected point(s)", 4000)
            return
        r = ps.solve()
        self.view._after_edit()
        self.statusBar().showMessage(f"flipped {n} construction(s)" + ("" if r.success else " — the other root is not reachable here"), 5000)

    def alternatives(self) -> None:
        """Stage 5: enumerate the real solutions of the construction that places the selected
        point (homotopy continuation on its merge system) and let the user pick one."""
        pts = [e for e in self.view.selected if isinstance(e, Point)]
        if len(pts) != 1:
            self.statusBar().showMessage("select exactly one point", 4000)
            return
        sk = self.view.sketch
        ps = self.view._plan()                    # the plan cached for this topology
        el = ps.graph.P(pts[0])
        placing = ps.plan.steps_placing(el)
        if not placing:
            self.statusBar().showMessage("no construction places that point (under-constrained or not decomposable)", 5000)
            return
        idx = placing[0][0]
        QApplication.setOverrideCursor(Qt.CursorShape.WaitCursor)
        try:
            alts = enumerate_step(ps.plan, idx, locate=el)
        finally:
            QApplication.restoreOverrideCursor()
        if len(alts) < 2:
            self.statusBar().showMessage(f"{len(alts)} real solution(s) for this construction — nothing to choose", 5000)
            return
        labels = [("● current — " if a.is_current else "")
                  + (f"point at ({a.location[0]:.3g}, {a.location[1]:.3g})" if a.location is not None
                     else f"distance {a.distance:.3g}")
                  for a in alts]
        choice, ok = QInputDialog.getItem(self, "Alternative solutions",
                                          f"{len(alts)} real solutions of this construction:", labels, 0, False)
        if not ok:
            return
        alt = alts[labels.index(choice)]
        if alt.is_current:
            return
        self.view.push_undo()
        apply_alternative(ps.plan, idx, alt)
        res = self.view._after_edit()
        # a root of the isolated merge system is not always reachable through a whole-plan
        # replay (the leaves are re-derived from the new geometry, and the surrounding merges
        # may pull it back) — say so rather than leaving an unexplained conflict on screen
        ok = res is None or res.success
        self.statusBar().showMessage("switched to the chosen solution" if ok else
                                     "that root is not reachable from here — the replay could not "
                                     "keep it (Ctrl+Z to go back)", 6000)

    def animate_dof(self) -> None:
        if not self.view.start_animation():
            self.statusBar().showMessage("no remaining internal DOF to animate", 4000)

    def toggle_colour(self, on: bool) -> None:
        self.view.color_by_state = on
        self.view.update()

    def show_diagnosis(self) -> None:
        d = self.view.diagnosis
        if d is None:
            QMessageBox.information(self, "Diagnosis", "No constraints.")
            return
        sk = self.view.sketch
        ix = io.Index(sk)
        lines = [d.summary(), ""]
        if d.conflicts:
            lines += ["Conflict — remove one of:"] + [f"   ✗ {io.describe(c, ix)}" for c in d.conflicts] + [""]
        if d.over:
            lines += [f"Structurally redundant block ({d.n_redundant} equation(s) too many):"]
            lines += [f"   • {io.describe(c, ix)}" for c in d.over] + [""]
        if d.under_params:
            names = sorted(ix.name(e) for kind in ("point", "circle", "arc") for e in sk.entities(kind)
                           if d.entity_state[id(e)] == "under")
            lines += [f"Under-constrained ({d.dof} DOF): {', '.join(names)}", ""]
        if len(d.components) > 1:
            lines += ["Components: " + ", ".join(f"{len(c.params)} params / DOF {c.dof}" for c in d.components), ""]
        big = [c for c in d.rigid_clusters if len(c) >= 3]
        if big:
            lines += ["Rigid clusters (distance graph): " + "; ".join(
                "{" + ", ".join(sorted(ix.name(p) for p in c)) + "}" for c in big), ""]
        rep = self.view.witness_report()
        if rep is not None:
            lines += [f"Witness analysis: {rep.summary()}"]
            for dep in rep.dependencies:
                lines += [f"   ⟂ {io.describe(dep.constraint, ix)} is implied by "
                          + ", ".join(io.describe(c, ix) for c in dep.implied_by[:6])
                          + ("  [theorem-type: invisible to structural analysis]" if dep.theorem else "")]
            internal = [m for m in rep.motions if not m.rigid]
            for i, m in enumerate(internal[:8]):
                lines += [f"   DOF {i + 1}: " + ", ".join(p.name for p in m.moving_params()[:10])]
            if internal:
                lines += ["   (View → Animate remaining DOF, Ctrl+M)"]
            lines += [""]
        lines += d.warnings
        pr = self.view.last_plan
        if pr is not None:
            lines += ["", f"Decomposition: {pr.plan.summary()}" + (" — numeric fallback used" if pr.fell_back else "")]
        self._show_report("Diagnosis", "\n".join(lines))

    def _show_report(self, title: str, text: str) -> None:
        """Scrollable read-only report, sized to its content but never taller than the screen
        (a QMessageBox grows unboundedly and pushes its button off-screen)."""
        dlg = QDialog(self)
        dlg.setWindowTitle(title)
        lay = QVBoxLayout(dlg)
        box = QPlainTextEdit(text)
        box.setReadOnly(True)
        box.setLineWrapMode(QPlainTextEdit.LineWrapMode.NoWrap)
        f = box.font()
        f.setFamily("Menlo")
        box.setFont(f)
        lay.addWidget(box)
        buttons = QDialogButtonBox(QDialogButtonBox.StandardButton.Close)
        buttons.rejected.connect(dlg.reject)
        lay.addWidget(buttons)
        screen = (self.screen() or QApplication.primaryScreen()).availableGeometry()
        fm = box.fontMetrics()                          # monospace (set above), so this measures right
        lines = text.split("\n")
        width = min(int(screen.width() * 0.9), max(520, fm.horizontalAdvance(max(lines, key=len)) + 60))
        height = min(int(screen.height() * 0.85), fm.lineSpacing() * (len(lines) + 2) + 90)
        dlg.resize(width, height)
        dlg.exec()

    def set_method(self, m: Method) -> None:
        self.view.method = m
        self.view.solve_now()

    # -- constraint list ----------------------------------------------------

    def on_constraint_selected(self, item: QListWidgetItem | None, _prev: Any = None) -> None:
        self.view.highlight = [] if item is None else expand(item.data(Qt.ItemDataRole.UserRole).entities())
        self.view.update()

    def delete_pressed(self) -> None:
        """Delete / Backspace: the constraint list has focus → remove that constraint;
        otherwise delete the selected entities in the view."""
        if self.clist.hasFocus() and self.clist.currentItem() is not None:
            self.delete_constraint()
        else:
            self.view.delete_selected()

    def delete_constraint(self) -> None:
        item = self.clist.currentItem()
        if item is None:
            return
        c = item.data(Qt.ItemDataRole.UserRole)
        self.view.remove_constraint(c)
        # no auto-selection of the next row: a selected row highlights its entities in
        # purple, which reads as "constrained" if it appears uninvited
        self.clist.setCurrentRow(-1)
        self.view.highlight = []
        self.view.update()
        self.statusBar().showMessage(f"removed {type(c).__name__}", 4000)

    def edit_constraint_value(self, item: QListWidgetItem) -> None:
        """Double-click: edit the constraint's dimension (first length/angle field in its spec)."""
        c = item.data(Qt.ItemDataRole.UserRole)
        for attr, kind in c.spec:
            if kind in ("length", "angle"):
                deg = kind == "angle"
                label = f"{type(c).__name__} {attr}" + (" (deg)" if deg else "")
                cur = math.degrees(getattr(c, attr)) if deg else getattr(c, attr)
                v, ok = QInputDialog.getDouble(self, label, label, cur, -1e9, 1e9, 4)
                if ok:
                    self.view.push_undo()
                    setattr(c, attr, math.radians(v) if deg else v)
                    self._rows = []  # force row text rebuild
                    self.view._after_edit()
                return

    # -- constraint actions (operate on the current selection) ---------------

    def _sel(self) -> tuple[list[Point], list[Line], list[Circle | Arc]]:
        s = self.view.selected
        return ([e for e in s if isinstance(e, Point)], [e for e in s if isinstance(e, Line)],
                [e for e in s if isinstance(e, (Circle, Arc))])

    def _need(self, ok: bool, what: str) -> bool:
        if not ok:
            self.statusBar().showMessage(f"select {what} first", 4000)
        return ok

    def _apply(self, cls: type[Constraint], n_pts: int, n_lines: int, n_circ: int) -> None:
        """Generic applier for constraints whose arguments are just entities: checks the
        selection has the required counts and passes them in spec order (points, lines, circles).
        Single-line constraints (Horizontal/Vertical) apply to every selected line."""
        p, l, c = self._sel()
        per_line = n_pts == 0 and n_lines == 1 and n_circ == 0
        ok = len(p) == n_pts and len(c) == n_circ and (len(l) >= 1 if per_line else len(l) == n_lines)
        what = ", ".join(f"{n} {w}" for n, w in ((n_pts, "point(s)"), (n_lines, "line(s)"), (n_circ, "circle(s)/arc(s)")) if n)
        if not self._need(ok, what):
            return
        for ln in (l if per_line else [None]):
            args: list[Any] = []
            pi = li = ci = 0
            for _, kind in cls.spec:
                if kind == "point":
                    args.append(p[pi]); pi += 1
                elif kind == "line":
                    args.append(ln if per_line else l[li]); li += 1
                elif kind in ENTITY_KINDS:
                    args.append(c[ci]); ci += 1
            self.view.add_constraint(cls(*args))

    def c_distance(self) -> None:
        p, l, _ = self._sel()
        if len(p) == 0 and len(l) == 1:
            p = [l[0].p1, l[0].p2]
        if self._need(len(p) == 2, "two points (or one line)"):
            cur = math.hypot(p[0].x.value - p[1].x.value, p[0].y.value - p[1].y.value)
            v, ok = QInputDialog.getDouble(self, "Distance", "Distance", cur, 0, 1e9, 4)
            if ok:
                self.view.add_constraint(C.Distance(p[0], p[1], v))

    def c_angle(self) -> None:
        _, l, _ = self._sel()
        if self._need(len(l) == 2, "two lines"):
            d1, d2 = l[0].direction(), l[1].direction()
            cur = math.degrees(math.atan2(d1[0] * d2[1] - d1[1] * d2[0], d1[0] * d2[0] + d1[1] * d2[1]))
            v, ok = QInputDialog.getDouble(self, "Angle", "Angle from first to second line (deg)", cur, -360, 360, 3)
            if ok:
                self.view.add_constraint(C.Angle(l[0], l[1], math.radians(v)))

    def c_equal(self) -> None:
        _, l, c = self._sel()
        if len(l) == 2:
            self.view.add_constraint(C.EqualLength(l[0], l[1]))
        elif len(c) == 2:
            self.view.add_constraint(C.EqualRadius(c[0], c[1]))
        else:
            self._need(False, "two lines or two circles/arcs")

    def c_tangent(self) -> None:
        _, l, c = self._sel()
        if len(l) == 1 and len(c) == 1:
            ln, cc = l[0], c[0]
            if isinstance(cc, Arc):
                ends = {ln.p1, ln.p2}
                if cc.start in ends:
                    self.view.add_constraint(C.TangentArcLine(cc, ln, "start"))
                    return
                if cc.end in ends:
                    self.view.add_constraint(C.TangentArcLine(cc, ln, "end"))
                    return
            self.view.add_constraint(C.TangentLineCircle(ln, cc))
        elif len(c) == 2 and not l:
            a, b = c
            d = math.hypot(a.center.x.value - b.center.x.value, a.center.y.value - b.center.y.value)
            self.view.add_constraint(C.TangentCircleCircle(a, b, external=d > max(abs(a.radius.value), abs(b.radius.value))))
        else:
            self._need(False, "a line and a circle/arc, or two circles/arcs")

    def c_radius(self) -> None:
        _, _, c = self._sel()
        if self._need(len(c) >= 1, "circle(s)/arc(s)"):
            v, ok = QInputDialog.getDouble(self, "Radius", "Radius", abs(c[0].radius.value), 0, 1e9, 4)
            if ok:
                for cc in c:
                    self.view.add_constraint(C.Radius(cc, v))


def main(argv: list[str] | None = None) -> int:
    argv = sys.argv if argv is None else argv
    app = QApplication(argv)
    sk = io.load(argv[1]) if len(argv) > 1 else EXAMPLES["rect_fillets"]()
    win = MainWindow(sk)
    win.show()
    win.view.fit()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
