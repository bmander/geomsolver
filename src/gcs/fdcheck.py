"""Finite-difference verification of analytic Jacobians.

Kept as a first-class module because it stays useful forever: every new
constraint type, every port to C, gets checked against this.
"""

from __future__ import annotations

from collections.abc import Callable

import numpy as np

from gcs.constraints import Constraint
from gcs.model import Sketch, Vec
from gcs.solve import System


def fd_jacobian(f: Callable[[Vec], Vec], v: Vec, h: float = 1e-6) -> Vec:
    """Central differences, one column per input."""
    v = np.asarray(v, dtype=np.float64)
    f0 = f(v)
    J = np.empty((f0.size, v.size))
    for j in range(v.size):
        e = np.zeros_like(v)
        e[j] = h
        J[:, j] = (f(v + e) - f(v - e)) / (2 * h)
    return J


def _assert_close(Ja: Vec, Jn: Vec, rtol: float, atol: float, label: str) -> float:
    err = float(np.max(np.abs(Ja - Jn))) if Ja.size else 0.0
    scale = float(np.max(np.abs(Jn))) if Jn.size else 0.0
    if err > atol + rtol * scale:
        raise AssertionError(f"{label}: Jacobian mismatch, max err {err:.3e}\nanalytic=\n{Ja}\nfd=\n{Jn}")
    return err


def check_constraint(c: Constraint, v: Vec | None = None, rtol: float = 1e-6, atol: float = 1e-7) -> float:
    """Return the max abs error between analytic and FD Jacobian; raise if too large."""
    v = c.local_values() if v is None else np.asarray(v, dtype=np.float64)
    Ja = np.asarray(c.jacobian(v), dtype=np.float64)
    assert Ja.shape == (c.n_residuals, len(c.params)), f"{c}: jacobian shape {Ja.shape}"
    return _assert_close(Ja, fd_jacobian(c.residual, v), rtol, atol, repr(c))


def check_sketch(sketch: Sketch, rtol: float = 1e-6, atol: float = 1e-6) -> float:
    """Check the assembled Jacobian of a whole sketch against FD."""
    sys_ = System(sketch)
    z = sys_.z0()
    return _assert_close(sys_.jacobian_dense(z), fd_jacobian(sys_.residuals, z), rtol, atol, "sketch")
