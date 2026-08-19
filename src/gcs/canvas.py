"""Dead-simple interactive testbed: matplotlib canvas with click-drag of points.

    python -m gcs.canvas rect_fillets
    python -m gcs.canvas truss --bays 8
    python -m gcs.canvas polygon_chain

Dragging = a temporary soft `DragTarget` constraint on the picked point,
re-solved from the warm start on every mouse move.  Keys: r = reset,
m = cycle solver method, f = toggle fix on nearest point, d = print DOF/state.
"""

from __future__ import annotations

import argparse
import math
from collections import deque
from collections.abc import Callable
from typing import Any

import matplotlib
import matplotlib.pyplot as plt
from matplotlib.backend_bases import KeyEvent, MouseEvent
from matplotlib.lines import Line2D
from matplotlib.patches import Arc as MplArc
from matplotlib.patches import Circle as MplCircle

from gcs.examples import EXAMPLES
from gcs.model import Point, Sketch
from gcs.solve import METHODS, Drag, Method


class Canvas:
    def __init__(self, make_sketch: Callable[[], Sketch], method: Method = "dogleg", drag_weight: float = 1.0) -> None:
        self.make_sketch = make_sketch
        self.sketch: Sketch = make_sketch()
        self.method_i = METHODS.index(method)
        self.drag_weight = drag_weight
        self.drag: Drag | None = None
        self.times: deque[float] = deque(maxlen=30)
        self.last: str = ""

        self.fig, self.ax = plt.subplots(figsize=(9, 7))
        self.ax.set_aspect("equal")
        self.fig.canvas.mpl_connect("button_press_event", self.on_press)
        self.fig.canvas.mpl_connect("motion_notify_event", self.on_motion)
        self.fig.canvas.mpl_connect("button_release_event", self.on_release)
        self.fig.canvas.mpl_connect("key_press_event", self.on_key)
        self.fig.canvas.mpl_connect("draw_event", self._on_draw)
        self.bg: Any = None
        self._refresh_topology()
        self._build_artists()
        self._fit_view()

    # -- artists ------------------------------------------------------------

    def _build_artists(self) -> None:
        for a in getattr(self, "artists", []):
            a.remove()
        self.artists: list[Any] = []
        sk = self.sketch
        self.line_art: list[Line2D] = [self.ax.add_line(Line2D([], [], color="C0", lw=1.6, animated=True)) for _ in sk.lines]
        self.circ_art: list[MplCircle] = [MplCircle((0, 0), 1, fill=False, color="C2", lw=1.6, animated=True) for _ in sk.circles]
        self.arc_art: list[MplArc] = [MplArc((0, 0), 2, 2, color="C1", lw=1.6, animated=True) for _ in sk.arcs]
        for patch in (*self.circ_art, *self.arc_art):
            self.ax.add_patch(patch)
        self.pt_art = self.ax.add_line(Line2D([], [], ls="", marker="o", ms=5, color="k", animated=True))
        self.fixed_art = self.ax.add_line(Line2D([], [], ls="", marker="s", ms=7, color="C3", animated=True))
        self.text = self.ax.text(0.01, 0.99, "", transform=self.ax.transAxes, va="top", ha="left",
                                 family="monospace", fontsize=9, animated=True)
        self.artists = [*self.line_art, *self.circ_art, *self.arc_art, self.pt_art, self.fixed_art, self.text]
        self._update_artists()

    def _update_artists(self) -> None:
        sk = self.sketch
        for art, ln in zip(self.line_art, sk.lines, strict=True):
            art.set_data([ln.p1.x.value, ln.p2.x.value], [ln.p1.y.value, ln.p2.y.value])
        for circ, c in zip(self.circ_art, sk.circles, strict=True):
            circ.set_center(c.center.xy)
            circ.set_radius(abs(c.radius.value))
        for arc, a in zip(self.arc_art, sk.arcs, strict=True):
            a0, a1 = a.angles()
            arc.set_center(a.center.xy)
            arc.set_width(2 * abs(a.radius.value))
            arc.set_height(2 * abs(a.radius.value))
            arc.theta1, arc.theta2 = math.degrees(a0), math.degrees(a1)
        free = [p for p in sk.points if not p.is_fixed]
        fixed = [p for p in sk.points if p.is_fixed]
        self.pt_art.set_data([p.x.value for p in free], [p.y.value for p in free])
        self.fixed_art.set_data([p.x.value for p in fixed], [p.y.value for p in fixed])
        avg = sum(self.times) / len(self.times) * 1e3 if self.times else 0.0
        self.text.set_text(f"method={METHODS[self.method_i]}  {self.topo}  solve={avg:.1f} ms avg  {self.last}")

    def _refresh_topology(self) -> None:
        """Topology-only counts; recomputed on structural edits, not per frame."""
        sk = self.sketch
        self.topo = f"params={len(sk.params)} res={sk.n_residuals()} dof={sk.dof()}"

    def _fit_view(self) -> None:
        x0, y0, x1, y1 = self.sketch.bbox()
        pad = 0.15 * self.sketch.extent()
        self.ax.set_xlim(x0 - pad, x1 + pad)
        self.ax.set_ylim(y0 - pad, y1 + pad)
        self.fig.canvas.draw_idle()

    def _on_draw(self, _evt: Any) -> None:
        canvas: Any = self.fig.canvas
        self.bg = canvas.copy_from_bbox(self.fig.bbox)
        self._blit()

    def _blit(self) -> None:
        if self.bg is None:
            return
        canvas: Any = self.fig.canvas
        canvas.restore_region(self.bg)
        self._update_artists()
        for a in self.artists:
            self.ax.draw_artist(a)
        self.fig.canvas.blit(self.fig.bbox)
        self.fig.canvas.flush_events()

    # -- interaction --------------------------------------------------------

    def _nearest_point(self, x: float, y: float) -> Point | None:
        best, bd = self.sketch.nearest_point(x, y)
        x0, x1 = self.ax.get_xlim()
        return best if bd < 0.03 * (x1 - x0) else None

    def on_press(self, evt: MouseEvent) -> None:
        if evt.inaxes is not self.ax or evt.xdata is None or evt.ydata is None or evt.button != 1:
            return
        p = self._nearest_point(evt.xdata, evt.ydata)
        if p is None or p.is_fixed:
            return
        self.drag = Drag(self.sketch, p, evt.xdata, evt.ydata, METHODS[self.method_i], self.drag_weight)

    def on_motion(self, evt: MouseEvent) -> None:
        if self.drag is None or evt.xdata is None or evt.ydata is None:
            return
        res = self.drag.move(evt.xdata, evt.ydata)
        self.times.append(res.time_s)
        self.last = f"|r|={res.max_residual:.1e} nfev={res.nfev}" + ("" if res.success else "  !! NOT CONVERGED")
        self._blit()

    def on_release(self, _evt: MouseEvent) -> None:
        if self.drag is not None:
            self.drag.end()
            self.drag = None
            self._blit()

    def on_key(self, evt: KeyEvent) -> None:
        if evt.key == "r":
            self.sketch = self.make_sketch()
            self.times.clear()
            self._refresh_topology()
            self._build_artists()
            self._fit_view()
        elif evt.key == "m":
            self.method_i = (self.method_i + 1) % len(METHODS)
            self.times.clear()
            self._blit()
        elif evt.key == "f" and evt.xdata is not None and evt.ydata is not None:
            p = self._nearest_point(evt.xdata, evt.ydata)
            if p is not None:
                p.fix(not p.is_fixed)
                self._refresh_topology()
                self._blit()
        elif evt.key == "d":
            sk = self.sketch
            worst = max(sk.constraints, key=lambda c: c.error(), default=None)
            print(f"params={len(sk.params)} free={len(sk.free_indices())} residuals={sk.n_residuals()} "
                  f"dof={sk.dof()} worst={worst} err={worst.error() if worst else 0:.2e}")

    def show(self) -> None:
        plt.show()


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("example", choices=sorted(EXAMPLES), nargs="?", default="rect_fillets")
    ap.add_argument("--method", choices=METHODS, default="dogleg")
    ap.add_argument("--bays", type=int, default=8, help="truss size")
    ap.add_argument("--n", type=int, default=12, help="polygon_chain size")
    ap.add_argument("--free", action="store_true", help="unfix all anchors so the sketch can float")
    ap.add_argument("--backend", default=None)
    args = ap.parse_args()
    if args.backend:
        matplotlib.use(args.backend)

    def make() -> Sketch:
        sk: Sketch
        if args.example == "truss":
            sk = EXAMPLES["truss"](bays=args.bays)
        elif args.example == "polygon_chain":
            sk = EXAMPLES["polygon_chain"](n=args.n)
        else:
            sk = EXAMPLES[args.example]()
        if args.free:
            for p in sk.params:
                p.fixed = False
        return sk

    Canvas(make, method=args.method).show()


if __name__ == "__main__":
    main()
