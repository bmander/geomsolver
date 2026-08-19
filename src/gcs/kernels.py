"""Vectorized residual/Jacobian kernels — one per constraint *type*.

A kernel evaluates n constraints of the same type at once:
    res(V, K) -> R,  V: (n, k) local parameter values, K: (n, m) constants, R: (n, n_res)
    jac(V, K) -> J,  J: (n, n_res, k)
The compiled `System` calls each kernel once per evaluation with every
constraint of that type stacked, so the Python-per-constraint overhead of
Stage 0 disappears.  The scalar `Constraint.residual/jacobian` API is a
one-row view of these same functions, so the FD checker still covers them.

Column conventions (local parameter order) are documented per kernel; they
match the `params` tuples built by the Constraint classes.
"""

from __future__ import annotations

from collections.abc import Callable
from typing import NamedTuple

import numpy as np
import numpy.typing as npt

Arr = npt.NDArray[np.float64]


class Kernel(NamedTuple):
    name: str
    n_res: int
    n_par: int
    n_const: int
    res: Callable[[Arr, Arr], Arr]
    jac: Callable[[Arr, Arr], Arr]
    const_jac: Arr | None = None   # set when J is the same for every instance (linear constraints)


def _const_jac(J: Arr) -> Callable[[Arr, Arr], Arr]:
    def jac(V: Arr, K: Arr) -> Arr:
        return np.broadcast_to(J, (V.shape[0],) + J.shape)

    return jac


def _row1(*cols: Arr) -> Arr:
    """Stack 1-D column arrays into a (n, 1, k) Jacobian for single-residual kernels."""
    return np.stack(cols, axis=1)[:, None, :]


# -- point / point ----------------------------------------------------------

def _coincident_res(V: Arr, K: Arr) -> Arr:                # (px,py,qx,qy)
    return np.stack([V[:, 0] - V[:, 2], V[:, 1] - V[:, 3]], axis=1)


coincident = Kernel("coincident", 2, 4, 0, _coincident_res,
                    _const_jac(np.array([[1.0, 0, -1, 0], [0, 1.0, 0, -1]])), np.array([[1.0, 0, -1, 0], [0, 1.0, 0, -1]]))


def _distance_res(V: Arr, K: Arr) -> Arr:                  # (px,py,qx,qy) K=(d,)
    dx, dy = V[:, 0] - V[:, 2], V[:, 1] - V[:, 3]
    return (dx * dx + dy * dy - K[:, 0] ** 2)[:, None]


def _distance_jac(V: Arr, K: Arr) -> Arr:
    dx, dy = V[:, 0] - V[:, 2], V[:, 1] - V[:, 3]
    return _row1(2 * dx, 2 * dy, -2 * dx, -2 * dy)


distance = Kernel("distance", 1, 4, 1, _distance_res, _distance_jac)


def _midpoint_res(V: Arr, K: Arr) -> Arr:                  # (px,py,ax,ay,bx,by)
    return np.stack([2 * V[:, 0] - V[:, 2] - V[:, 4], 2 * V[:, 1] - V[:, 3] - V[:, 5]], axis=1)


midpoint = Kernel("midpoint", 2, 6, 0, _midpoint_res,
                  _const_jac(np.array([[2.0, 0, -1, 0, -1, 0], [0, 2.0, 0, -1, 0, -1]])), np.array([[2.0, 0, -1, 0, -1, 0], [0, 2.0, 0, -1, 0, -1]]))


def _drag_res(V: Arr, K: Arr) -> Arr:                      # (px,py) K=(tx,ty,w)
    return K[:, 2:3] * np.stack([V[:, 0] - K[:, 0], V[:, 1] - K[:, 1]], axis=1)


def _drag_jac(V: Arr, K: Arr) -> Arr:
    return K[:, 2][:, None, None] * np.eye(2)[None]


drag = Kernel("drag", 2, 2, 3, _drag_res, _drag_jac)


# -- line orientation -------------------------------------------------------

def _horizontal_res(V: Arr, K: Arr) -> Arr:                # (ax,ay,bx,by)
    return (V[:, 1] - V[:, 3])[:, None]


horizontal = Kernel("horizontal", 1, 4, 0, _horizontal_res, _const_jac(np.array([[0.0, 1, 0, -1]])), np.array([[0.0, 1, 0, -1]]))


def _vertical_res(V: Arr, K: Arr) -> Arr:
    return (V[:, 0] - V[:, 2])[:, None]


vertical = Kernel("vertical", 1, 4, 0, _vertical_res, _const_jac(np.array([[1.0, 0, -1, 0]])), np.array([[1.0, 0, -1, 0]]))


def _dirs(V: Arr) -> tuple[Arr, Arr, Arr, Arr]:            # (a1x,a1y,b1x,b1y,a2x,a2y,b2x,b2y)
    return V[:, 2] - V[:, 0], V[:, 3] - V[:, 1], V[:, 6] - V[:, 4], V[:, 7] - V[:, 5]


def _cross(V: Arr) -> Arr:
    d1x, d1y, d2x, d2y = _dirs(V)
    return d1x * d2y - d1y * d2x


def _dot(V: Arr) -> Arr:
    d1x, d1y, d2x, d2y = _dirs(V)
    return d1x * d2x + d1y * d2y


def _cross_jac(V: Arr) -> Arr:
    d1x, d1y, d2x, d2y = _dirs(V)
    return _row1(-d2y, d2x, d2y, -d2x, d1y, -d1x, -d1y, d1x)


def _dot_jac(V: Arr) -> Arr:
    d1x, d1y, d2x, d2y = _dirs(V)
    return _row1(-d2x, -d2y, d2x, d2y, -d1x, -d1y, d1x, d1y)


parallel = Kernel("parallel", 1, 8, 0, lambda V, K: _cross(V)[:, None], lambda V, K: _cross_jac(V))
perpendicular = Kernel("perpendicular", 1, 8, 0, lambda V, K: _dot(V)[:, None], lambda V, K: _dot_jac(V))


def _angle_res(V: Arr, K: Arr) -> Arr:                     # K=(sinθ, cosθ)
    return (_dot(V) * K[:, 0] - _cross(V) * K[:, 1])[:, None]


def _angle_jac(V: Arr, K: Arr) -> Arr:
    return _dot_jac(V) * K[:, 0][:, None, None] - _cross_jac(V) * K[:, 1][:, None, None]


angle = Kernel("angle", 1, 8, 2, _angle_res, _angle_jac)


def _equal_length_res(V: Arr, K: Arr) -> Arr:
    d1x, d1y, d2x, d2y = _dirs(V)
    return (d1x * d1x + d1y * d1y - d2x * d2x - d2y * d2y)[:, None]


def _equal_length_jac(V: Arr, K: Arr) -> Arr:
    d1x, d1y, d2x, d2y = _dirs(V)
    return 2 * _row1(-d1x, -d1y, d1x, d1y, d2x, d2y, -d2x, -d2y)


equal_length = Kernel("equal_length", 1, 8, 0, _equal_length_res, _equal_length_jac)


# -- incidence --------------------------------------------------------------

def _point_on_line_res(V: Arr, K: Arr) -> Arr:             # (px,py,ax,ay,bx,by)
    dx, dy = V[:, 4] - V[:, 2], V[:, 5] - V[:, 3]
    wx, wy = V[:, 0] - V[:, 2], V[:, 1] - V[:, 3]
    return (dx * wy - dy * wx)[:, None]


def _point_on_line_jac(V: Arr, K: Arr) -> Arr:
    dx, dy = V[:, 4] - V[:, 2], V[:, 5] - V[:, 3]
    wx, wy = V[:, 0] - V[:, 2], V[:, 1] - V[:, 3]
    return _row1(-dy, dx, dy - wy, wx - dx, wy, -wx)


point_on_line = Kernel("point_on_line", 1, 6, 0, _point_on_line_res, _point_on_line_jac)


def _point_on_circle_res(V: Arr, K: Arr) -> Arr:           # (px,py,cx,cy,r)
    ux, uy = V[:, 0] - V[:, 2], V[:, 1] - V[:, 3]
    return (ux * ux + uy * uy - V[:, 4] ** 2)[:, None]


def _point_on_circle_jac(V: Arr, K: Arr) -> Arr:
    ux, uy = V[:, 0] - V[:, 2], V[:, 1] - V[:, 3]
    return _row1(2 * ux, 2 * uy, -2 * ux, -2 * uy, -2 * V[:, 4])


point_on_circle = Kernel("point_on_circle", 1, 5, 0, _point_on_circle_res, _point_on_circle_jac)


# -- radii ------------------------------------------------------------------

radius = Kernel("radius", 1, 1, 1, lambda V, K: (V[:, 0] - K[:, 0])[:, None], _const_jac(np.array([[1.0]])), np.array([[1.0]]))
equal_radius = Kernel("equal_radius", 1, 2, 0, lambda V, K: (V[:, 0] - V[:, 1])[:, None],
                      _const_jac(np.array([[1.0, -1.0]])), np.array([[1.0, -1.0]]))


# -- tangency ---------------------------------------------------------------

def _tlc_parts(V: Arr) -> tuple[Arr, Arr, Arr, Arr, Arr, Arr]:   # (ax,ay,bx,by,cx,cy,r)
    dx, dy = V[:, 2] - V[:, 0], V[:, 3] - V[:, 1]
    wx, wy = V[:, 4] - V[:, 0], V[:, 5] - V[:, 1]
    L = np.hypot(dx, dy)
    C = dx * wy - dy * wx
    return dx, dy, wx, wy, L, C


def _tangent_line_circle_res(V: Arr, K: Arr) -> Arr:       # K=(side,)
    _, _, _, _, L, C = _tlc_parts(V)
    return (C / L - K[:, 0] * V[:, 6])[:, None]


def _tangent_line_circle_jac(V: Arr, K: Arr) -> Arr:
    dx, dy, wx, wy, L, C = _tlc_parts(V)
    z = np.zeros_like(L)
    dC = np.stack([dy - wy, wx - dx, wy, -wx, -dy, dx, z], axis=1)
    dL = np.stack([-dx / L, -dy / L, dx / L, dy / L, z, z, z], axis=1)
    J = dC / L[:, None] - (C / (L * L))[:, None] * dL
    J[:, 6] = -K[:, 0]
    return J[:, None, :]


tangent_line_circle = Kernel("tangent_line_circle", 1, 7, 1, _tangent_line_circle_res, _tangent_line_circle_jac)


def _tangent_circle_circle_res(V: Arr, K: Arr) -> Arr:     # (c1x,c1y,r1,c2x,c2y,r2) K=(sign,) +1 external
    ux, uy = V[:, 0] - V[:, 3], V[:, 1] - V[:, 4]
    R = V[:, 2] + K[:, 0] * V[:, 5]
    return (ux * ux + uy * uy - R * R)[:, None]


def _tangent_circle_circle_jac(V: Arr, K: Arr) -> Arr:
    ux, uy = V[:, 0] - V[:, 3], V[:, 1] - V[:, 4]
    R = V[:, 2] + K[:, 0] * V[:, 5]
    return _row1(2 * ux, 2 * uy, -2 * R, -2 * ux, -2 * uy, -2 * R * K[:, 0])


tangent_circle_circle = Kernel("tangent_circle_circle", 1, 6, 1, _tangent_circle_circle_res, _tangent_circle_circle_jac)


def _tangent_arc_line_res(V: Arr, K: Arr) -> Arr:          # (px,py,cx,cy,ax,ay,bx,by)
    ux, uy = V[:, 0] - V[:, 2], V[:, 1] - V[:, 3]
    dx, dy = V[:, 6] - V[:, 4], V[:, 7] - V[:, 5]
    return (ux * dx + uy * dy)[:, None]


def _tangent_arc_line_jac(V: Arr, K: Arr) -> Arr:
    ux, uy = V[:, 0] - V[:, 2], V[:, 1] - V[:, 3]
    dx, dy = V[:, 6] - V[:, 4], V[:, 7] - V[:, 5]
    return _row1(dx, dy, -dx, -dy, -ux, -uy, ux, uy)


tangent_arc_line = Kernel("tangent_arc_line", 1, 8, 0, _tangent_arc_line_res, _tangent_arc_line_jac)


KERNELS: tuple[Kernel, ...] = (
    coincident, distance, midpoint, drag, horizontal, vertical, parallel, perpendicular, angle,
    equal_length, point_on_line, point_on_circle, radius, equal_radius, tangent_line_circle,
    tangent_circle_circle, tangent_arc_line,
)
KERNEL_ID: dict[str, int] = {k.name: i for i, k in enumerate(KERNELS)}
