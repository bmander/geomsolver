"""Finite-difference verification of analytic Jacobians.

Kept as a first-class module because it stays useful forever: every new constraint type gets
checked against it.  The check itself runs in the core, so Python and the browser see the same
numbers.
"""

from __future__ import annotations

from gcs._ffi import lib
from gcs.constraints import Constraint, _bound
from gcs.model import Sketch


def check_constraint(c: Constraint, rtol: float = 1e-6, atol: float = 1e-7) -> float:
    """Max abs error between analytic and FD Jacobian; raises if too large."""
    with _bound(c) as (sk, cid):
        err = float(lib.gcs_check_constraint(sk._h, cid, rtol, atol))
    if err < 0:
        from gcs import _ffi
        raise AssertionError(_ffi.last_error() or f"{c!r}: Jacobian mismatch")
    return err


def check_sketch(sketch: Sketch, rtol: float = 1e-6, atol: float = 1e-6) -> float:
    """Check the assembled Jacobian of a whole sketch against finite differences."""
    err = float(lib.gcs_check_sketch(sketch._h, rtol, atol))
    if err < 0:
        from gcs import _ffi
        raise AssertionError(_ffi.last_error() or "sketch: Jacobian mismatch")
    return err


__all__ = ["check_constraint", "check_sketch"]
