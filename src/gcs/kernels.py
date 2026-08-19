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
ArrLike = npt.ArrayLike


class Kernel(NamedTuple):
    name: str
    n_res: int
    n_par: int
    n_const: int
    res: Callable[[Arr, Arr], Arr]
    jac: Callable[[Arr, Arr], Arr]
    const_jac: Arr | None = None   # set when J is the same for every instance (linear constraints)


KERNELS: list[Kernel] = []          # registration order == kernel id
KERNEL_ID: dict[str, int] = {}


def _broadcast_jac(J: Arr) -> Callable[[Arr, Arr], Arr]:
    def jac(V: Arr, K: Arr) -> Arr:
        return np.broadcast_to(J, (V.shape[0],) + J.shape)

    return jac


def kernel(name: str, n_res: int, n_par: int, n_const: int, res: Callable[[Arr, Arr], Arr],
           jac: Callable[[Arr, Arr], Arr] | None = None, const_jac: ArrLike | None = None) -> Kernel:
    """Register a kernel.  Pass `const_jac` (and no `jac`) when the Jacobian is constant."""
    if const_jac is not None:
        const_jac = np.asarray(const_jac, dtype=np.float64)
        jac = _broadcast_jac(const_jac)
    assert jac is not None
    k = Kernel(name, n_res, n_par, n_const, res, jac, const_jac)
    KERNEL_ID[name] = len(KERNELS)
    KERNELS.append(k)
    return k


def linear_kernel(name: str, J: ArrLike) -> Kernel:
    """Kernel for a linear constraint r = J·v: residual, Jacobian and shape all derive from J."""
    J = np.asarray(J, dtype=np.float64)
    return kernel(name, J.shape[0], J.shape[1], 0, lambda V, K: V @ J.T, const_jac=J)


def _row1(*cols: Arr) -> Arr:
    """Stack 1-D column arrays into a (n, 1, k) Jacobian for single-residual kernels."""
    return np.stack(cols, axis=1)[:, None, :]


# -- point / point ----------------------------------------------------------

coincident = linear_kernel("coincident", [[1, 0, -1, 0], [0, 1, 0, -1]])          # (px,py,qx,qy)


def _distance_res(V: Arr, K: Arr) -> Arr:                  # (px,py,qx,qy) K=(d,)
    dx, dy = V[:, 0] - V[:, 2], V[:, 1] - V[:, 3]
    return (dx * dx + dy * dy - K[:, 0] ** 2)[:, None]


def _distance_jac(V: Arr, K: Arr) -> Arr:
    J = np.empty((V.shape[0], 1, 4))
    d = 2 * (V[:, 0:2] - V[:, 2:4])
    J[:, 0, 0:2] = d
    J[:, 0, 2:4] = -d
    return J


distance = kernel("distance", 1, 4, 1, _distance_res, _distance_jac)

midpoint = linear_kernel("midpoint", [[2, 0, -1, 0, -1, 0], [0, 2, 0, -1, 0, -1]])   # (px,py,ax,ay,bx,by)


def _drag_res(V: Arr, K: Arr) -> Arr:                      # (px,py) K=(tx,ty,w)
    return K[:, 2:3] * np.stack([V[:, 0] - K[:, 0], V[:, 1] - K[:, 1]], axis=1)


def _drag_jac(V: Arr, K: Arr) -> Arr:
    return K[:, 2][:, None, None] * np.eye(2)[None]


drag = kernel("drag", 2, 2, 3, _drag_res, _drag_jac)


# -- line orientation -------------------------------------------------------

horizontal = linear_kernel("horizontal", [[0, 1, 0, -1]])   # (ax,ay,bx,by): ay − by
vertical = linear_kernel("vertical", [[1, 0, -1, 0]])       # ax − bx


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


parallel = kernel("parallel", 1, 8, 0, lambda V, K: _cross(V)[:, None], lambda V, K: _cross_jac(V))
perpendicular = kernel("perpendicular", 1, 8, 0, lambda V, K: _dot(V)[:, None], lambda V, K: _dot_jac(V))


def _angle_res(V: Arr, K: Arr) -> Arr:                     # K=(sinθ, cosθ)
    return (_dot(V) * K[:, 0] - _cross(V) * K[:, 1])[:, None]


def _angle_jac(V: Arr, K: Arr) -> Arr:
    return _dot_jac(V) * K[:, 0][:, None, None] - _cross_jac(V) * K[:, 1][:, None, None]


angle = kernel("angle", 1, 8, 2, _angle_res, _angle_jac)


def _equal_length_res(V: Arr, K: Arr) -> Arr:
    d1x, d1y, d2x, d2y = _dirs(V)
    return (d1x * d1x + d1y * d1y - d2x * d2x - d2y * d2y)[:, None]


def _equal_length_jac(V: Arr, K: Arr) -> Arr:
    d1x, d1y, d2x, d2y = _dirs(V)
    return 2 * _row1(-d1x, -d1y, d1x, d1y, d2x, d2y, -d2x, -d2y)


equal_length = kernel("equal_length", 1, 8, 0, _equal_length_res, _equal_length_jac)


# -- incidence --------------------------------------------------------------

def _point_on_line_res(V: Arr, K: Arr) -> Arr:             # (px,py,ax,ay,bx,by)
    dx, dy = V[:, 4] - V[:, 2], V[:, 5] - V[:, 3]
    wx, wy = V[:, 0] - V[:, 2], V[:, 1] - V[:, 3]
    return (dx * wy - dy * wx)[:, None]


def _point_on_line_jac(V: Arr, K: Arr) -> Arr:
    dx, dy = V[:, 4] - V[:, 2], V[:, 5] - V[:, 3]
    wx, wy = V[:, 0] - V[:, 2], V[:, 1] - V[:, 3]
    return _row1(-dy, dx, dy - wy, wx - dx, wy, -wx)


point_on_line = kernel("point_on_line", 1, 6, 0, _point_on_line_res, _point_on_line_jac)


def _point_on_circle_res(V: Arr, K: Arr) -> Arr:           # (px,py,cx,cy,r)
    ux, uy = V[:, 0] - V[:, 2], V[:, 1] - V[:, 3]
    return (ux * ux + uy * uy - V[:, 4] ** 2)[:, None]


def _point_on_circle_jac(V: Arr, K: Arr) -> Arr:
    ux, uy = V[:, 0] - V[:, 2], V[:, 1] - V[:, 3]
    return _row1(2 * ux, 2 * uy, -2 * ux, -2 * uy, -2 * V[:, 4])


point_on_circle = kernel("point_on_circle", 1, 5, 0, _point_on_circle_res, _point_on_circle_jac)


# -- radii ------------------------------------------------------------------

radius = kernel("radius", 1, 1, 1, lambda V, K: (V[:, 0] - K[:, 0])[:, None], const_jac=[[1.0]])
equal_radius = linear_kernel("equal_radius", [[1, -1]])


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


tangent_line_circle = kernel("tangent_line_circle", 1, 7, 1, _tangent_line_circle_res, _tangent_line_circle_jac)


def _tangent_circle_circle_res(V: Arr, K: Arr) -> Arr:     # (c1x,c1y,r1,c2x,c2y,r2) K=(sign,) +1 external
    ux, uy = V[:, 0] - V[:, 3], V[:, 1] - V[:, 4]
    R = V[:, 2] + K[:, 0] * V[:, 5]
    return (ux * ux + uy * uy - R * R)[:, None]


def _tangent_circle_circle_jac(V: Arr, K: Arr) -> Arr:
    ux, uy = V[:, 0] - V[:, 3], V[:, 1] - V[:, 4]
    R = V[:, 2] + K[:, 0] * V[:, 5]
    return _row1(2 * ux, 2 * uy, -2 * R, -2 * ux, -2 * uy, -2 * R * K[:, 0])


tangent_circle_circle = kernel("tangent_circle_circle", 1, 6, 1, _tangent_circle_circle_res, _tangent_circle_circle_jac)


def _tangent_arc_line_res(V: Arr, K: Arr) -> Arr:          # (px,py,cx,cy,ax,ay,bx,by)
    ux, uy = V[:, 0] - V[:, 2], V[:, 1] - V[:, 3]
    dx, dy = V[:, 6] - V[:, 4], V[:, 7] - V[:, 5]
    return (ux * dx + uy * dy)[:, None]


def _tangent_arc_line_jac(V: Arr, K: Arr) -> Arr:
    ux, uy = V[:, 0] - V[:, 2], V[:, 1] - V[:, 3]
    dx, dy = V[:, 6] - V[:, 4], V[:, 7] - V[:, 5]
    return _row1(dx, dy, -dx, -dy, -ux, -uy, ux, uy)


tangent_arc_line = kernel("tangent_arc_line", 1, 8, 0, _tangent_arc_line_res, _tangent_arc_line_jac)
