"""Loading the Rust core and moving numbers across the boundary.

`gcs` is a thin binding: every algorithm lives in `rust/gcs-core`, reached through the flat C ABI
in `rust/gcs-ffi`.  This module is the only place that knows about pointers — nothing above it
does.  Ragged results (diagnosis, plans, constraint lists) cross as JSON; hot-path numbers cross
as ctypes buffers over numpy arrays.
"""

from __future__ import annotations

import ctypes as ct
import json
import platform
from pathlib import Path
from typing import Any

import numpy as np
import numpy.typing as npt

Vec = npt.NDArray[np.float64]
IVec = npt.NDArray[np.int32]

_EXT = {"Darwin": ".dylib", "Windows": ".dll"}.get(platform.system(), ".so")
_ROOT = Path(__file__).resolve().parents[2]
_CANDIDATES = [
    _ROOT / "build" / f"libgcs{_EXT}",
    _ROOT / "rust" / "target" / "release" / f"libgcs{_EXT}",
    _ROOT / "rust" / "target" / "debug" / f"libgcs{_EXT}",
]

F64 = ct.POINTER(ct.c_double)
I32 = ct.POINTER(ct.c_int32)
U8 = ct.POINTER(ct.c_uint8)
PTR = ct.c_void_p
STR = ct.c_void_p  # a length-prefixed string block owned by the core


def _find_library() -> Path:
    for p in _CANDIDATES:
        if p.exists():
            return p
    raise OSError(
        f"gcs core not built: none of {[str(p) for p in _CANDIDATES]} exist.  Run `make`."
    )


# (name, restype, argtypes)
_SIGNATURES: list[tuple[str, Any, list[Any]]] = [
    # memory / strings
    ("gcs_malloc", PTR, [ct.c_size_t]),
    ("gcs_free", None, [PTR, ct.c_size_t]),
    ("gcs_str_len", ct.c_uint32, [STR]),
    ("gcs_str_ptr", PTR, [STR]),
    ("gcs_str_free", None, [STR]),
    ("gcs_last_error", STR, []),
    # metadata
    ("gcs_registry_json", STR, []),
    ("gcs_kernel_count", ct.c_int32, []),
    ("gcs_version", STR, []),
    # sketch
    ("gcs_sketch_new", PTR, []),
    ("gcs_sketch_free", None, [PTR]),
    ("gcs_sketch_clone", PTR, [PTR]),
    ("gcs_sketch_from_json", PTR, [PTR, ct.c_size_t]),
    ("gcs_sketch_to_json", STR, [PTR, ct.c_int32]),
    ("gcs_sketch_counts", None, [PTR, I32]),
    ("gcs_sketch_point", ct.c_int32, [PTR, ct.c_double, ct.c_double, ct.c_int32, PTR, ct.c_size_t]),
    ("gcs_sketch_line", ct.c_int32, [PTR, ct.c_int32, ct.c_int32]),
    ("gcs_sketch_circle", ct.c_int32, [PTR, ct.c_int32, ct.c_double, PTR, ct.c_size_t]),
    ("gcs_sketch_arc", ct.c_int32, [PTR, ct.c_int32, ct.c_int32, ct.c_int32, PTR, ct.c_size_t]),
    ("gcs_sketch_arc_through", ct.c_int32,
     [PTR, ct.c_int32, ct.c_int32, ct.c_double, ct.c_double, PTR, ct.c_size_t]),
    ("gcs_sketch_spline", ct.c_int32, [PTR, I32, ct.c_size_t]),
    ("gcs_sketch_spline_knots", ct.c_int32, [PTR, I32, ct.c_size_t, F64, ct.c_size_t]),
    ("gcs_sketch_spline_through", ct.c_int32, [PTR, F64, ct.c_size_t]),
    ("gcs_sketch_rectangle", None,
     [PTR, ct.c_int32, ct.c_double, ct.c_double, PTR, ct.c_size_t, I32]),
    ("gcs_sketch_get_x", None, [PTR, F64]),
    ("gcs_sketch_set_x", ct.c_int32, [PTR, F64, ct.c_size_t]),
    ("gcs_sketch_perturb", None, [PTR, ct.c_double, ct.c_uint32]),
    ("gcs_sketch_topology_key", STR, [PTR]),
    ("gcs_sketch_extent", ct.c_double, [PTR]),
    ("gcs_sketch_bounds", None, [PTR, ct.c_int32, F64]),
    ("gcs_sketch_nearest_point", ct.c_int32, [PTR, ct.c_double, ct.c_double, F64]),
    ("gcs_sketch_n_residuals", ct.c_int32, [PTR]),
    ("gcs_sketch_set_constraints", None, [PTR, PTR, ct.c_size_t]),
    # params
    ("gcs_param_value", ct.c_double, [PTR, ct.c_int32]),
    ("gcs_param_set_value", None, [PTR, ct.c_int32, ct.c_double]),
    ("gcs_param_fixed", ct.c_int32, [PTR, ct.c_int32]),
    ("gcs_param_set_fixed", None, [PTR, ct.c_int32, ct.c_int32]),
    ("gcs_param_name", STR, [PTR, ct.c_int32]),
    # entities
    ("gcs_entity_params", ct.c_int32, [PTR, ct.c_int32, ct.c_int32, I32]),
    ("gcs_entity_points", ct.c_int32, [PTR, ct.c_int32, ct.c_int32, I32]),
    ("gcs_entity_radius_param", ct.c_int32, [PTR, ct.c_int32, ct.c_int32]),
    ("gcs_entity_construction", ct.c_int32, [PTR, ct.c_int32, ct.c_int32]),
    ("gcs_entity_set_construction", None, [PTR, ct.c_int32, ct.c_int32, ct.c_int32]),
    ("gcs_entity_bounds", None, [PTR, ct.c_int32, ct.c_int32, F64]),
    ("gcs_entity_name", STR, [ct.c_int32, ct.c_int32]),
    ("gcs_distance_between", ct.c_double, [PTR, ct.c_int32, ct.c_int32, ct.c_int32, ct.c_int32]),
    ("gcs_orientation", ct.c_double, [PTR, ct.c_int32, ct.c_int32, ct.c_int32]),
    ("gcs_angle_between", ct.c_double, [PTR, ct.c_int32, ct.c_int32]),
    ("gcs_on_radius", ct.c_int32,
     [ct.c_double, ct.c_double, ct.c_double, ct.c_double, ct.c_double, F64]),
    ("gcs_signed_point_to_line", ct.c_double, [PTR, ct.c_double, ct.c_double, ct.c_int32]),
    ("gcs_arc_angles", None, [PTR, ct.c_int32, F64]),
    ("gcs_spline_knots", ct.c_int32, [PTR, ct.c_int32, F64]),
    ("gcs_spline_domain", None, [PTR, ct.c_int32, F64]),
    ("gcs_spline_eval", None, [PTR, ct.c_int32, ct.c_double, F64]),
    ("gcs_spline_polyline", ct.c_int32, [PTR, ct.c_int32, ct.c_double, F64, ct.c_int32]),
    ("gcs_spline_closest", None, [PTR, ct.c_int32, ct.c_double, ct.c_double, F64]),
    ("gcs_spline_insert_control", ct.c_int32, [PTR, ct.c_int32, ct.c_double]),
    ("gcs_three_point_arc", ct.c_int32,
     [ct.c_double] * 6 + [F64]),
    # constraints
    ("gcs_constraint_add", ct.c_int32, [PTR, PTR, ct.c_size_t]),
    ("gcs_constraint_remove", None, [PTR, ct.c_int32]),
    ("gcs_constraints_json", STR, [PTR]),
    ("gcs_constraint_json", STR, [PTR, ct.c_int32]),
    ("gcs_constraint_set_num", ct.c_int32, [PTR, ct.c_int32, PTR, ct.c_size_t, ct.c_double]),
    ("gcs_constraint_set_str", ct.c_int32,
     [PTR, ct.c_int32, PTR, ct.c_size_t, PTR, ct.c_size_t]),
    ("gcs_constraint_set_dimension", ct.c_int32,
     [PTR, ct.c_int32, PTR, ct.c_size_t, PTR, ct.c_size_t]),
    ("gcs_exprs_json", STR, [PTR]),
    ("gcs_constraint_set_target", None, [PTR, ct.c_int32, ct.c_double, ct.c_double]),
    ("gcs_constraint_error", ct.c_double, [PTR, ct.c_int32]),
    ("gcs_constraint_params", ct.c_int32, [PTR, ct.c_int32, I32]),
    ("gcs_constraint_local_values", ct.c_int32, [PTR, ct.c_int32, F64]),
    ("gcs_constraint_eval", ct.c_int32, [PTR, ct.c_int32, F64, F64, F64]),
    ("gcs_same_constraint", ct.c_int32,
     [PTR, PTR, ct.c_size_t, PTR, ct.c_size_t]),
    ("gcs_same_constraint", ct.c_int32,
     [PTR, PTR, ct.c_size_t, PTR, ct.c_size_t]),
    ("gcs_constraint_duplicate", ct.c_int32, [PTR, PTR, ct.c_size_t]),
    ("gcs_describe", STR, [PTR, ct.c_int32]),
    ("gcs_callouts_json", STR, [PTR, ct.c_double]),
    ("gcs_callout_pick", ct.c_int32,
     [PTR, ct.c_double, ct.c_double, ct.c_double, ct.c_double]),
    ("gcs_callout_grab", ct.c_int32,
     [PTR, ct.c_int32, ct.c_double, ct.c_double, ct.c_double, F64]),
    ("gcs_callout_drag", ct.c_int32,
     [PTR, ct.c_int32, ct.c_double, ct.c_double, ct.c_double, ct.c_double]),
    ("gcs_callout_reset", ct.c_int32, [PTR, ct.c_int32]),
    ("gcs_fmt_g", STR, [ct.c_double, ct.c_int32]),
    # branches / io / examples
    ("gcs_branches_json", STR, [PTR]),
    ("gcs_branches_set_json", None, [PTR, PTR, ct.c_size_t]),
    ("gcs_without", PTR, [PTR, PTR, ct.c_size_t, PTR, ct.c_size_t]),
    ("gcs_copy", PTR, [PTR, PTR, ct.c_size_t]),
    ("gcs_paste", STR, [PTR, PTR, ct.c_double, ct.c_double]),
    ("gcs_example", PTR, [PTR, ct.c_size_t]),
    ("gcs_cases_json", STR, []),
    # system
    ("gcs_system_new", PTR, [PTR]),
    ("gcs_system_free", None, [PTR]),
    ("gcs_system_n_res", ct.c_int32, [PTR]),
    ("gcs_system_n_free", ct.c_int32, [PTR]),
    ("gcs_system_nnz", ct.c_int32, [PTR]),
    ("gcs_system_scale", ct.c_double, [PTR]),
    ("gcs_system_hard", None, [PTR, U8]),
    ("gcs_system_z0", None, [PTR, PTR, F64]),
    ("gcs_system_residuals", None, [PTR, F64, F64]),
    ("gcs_system_jacobian_dense", None, [PTR, F64, F64]),
    ("gcs_system_csr_structure", None, [PTR, I32, I32]),
    ("gcs_system_csr_data", None, [PTR, F64, F64]),
    ("gcs_system_max_hard_residual", ct.c_double, [PTR, PTR]),
    ("gcs_system_constraint_errors", ct.c_int32, [PTR, PTR, I32, F64, ct.c_int32]),
    ("gcs_system_n_constraints", ct.c_int32, [PTR]),
    ("gcs_system_max_relative_residual", ct.c_double, [PTR, PTR]),
    ("gcs_system_rank", ct.c_int32, [PTR, PTR, ct.c_double, ct.c_int32]),
    ("gcs_system_update_consts", None, [PTR, PTR, ct.c_int32]),
    ("gcs_system_refresh_consts", None, [PTR, PTR]),
    ("gcs_system_structure_json", STR, [PTR]),
    ("gcs_system_free_indices", None, [PTR, I32]),
    ("gcs_system_row_of", ct.c_int32, [PTR, ct.c_int32]),
    ("gcs_system_solve", STR,
     [PTR, PTR, ct.c_int32, ct.c_double, ct.c_int32, ct.c_int32, ct.c_int32, ct.c_int32, F64]),
    ("gcs_solve", STR,
     [PTR, ct.c_int32, ct.c_double, ct.c_int32, ct.c_int32, ct.c_int32, F64]),
    ("gcs_status_message", STR, [ct.c_int32]),
    # linear algebra
    ("gcs_min_norm_lstsq", ct.c_int32,
     [ct.c_int32, ct.c_int32, ct.c_int32, F64, F64, ct.c_double, F64]),
    ("gcs_rrqr", ct.c_int32, [ct.c_int32, ct.c_int32, F64, ct.c_double, I32]),
    ("gcs_svd", ct.c_int32, [ct.c_int32, ct.c_int32, F64, F64, F64, F64]),
    ("gcs_rank_nullspace", ct.c_int32, [ct.c_int32, ct.c_int32, F64, ct.c_double, F64, F64]),
    ("gcs_lu_solve", ct.c_int32, [ct.c_int32, F64, F64]),
    # fd check
    ("gcs_check_sketch", ct.c_double, [PTR, ct.c_double, ct.c_double]),
    ("gcs_check_constraint", ct.c_double, [PTR, ct.c_int32, ct.c_double, ct.c_double]),
    # pure graph algorithms
    ("gcs_hopcroft_karp_json", STR, [PTR, ct.c_size_t, ct.c_int32]),
    ("gcs_dulmage_mendelsohn_json", STR, [PTR, ct.c_size_t, ct.c_int32]),
    ("gcs_pebble_game_json", STR, [ct.c_int32, PTR, ct.c_size_t]),
    ("gcs_bipartite_components_json", STR, [PTR, ct.c_size_t, ct.c_int32]),
    ("gcs_henneberg_edges_json", STR, [ct.c_int32, ct.c_uint32]),
    # diagnosis / witness
    ("gcs_diagnose_json", STR, [PTR, PTR, ct.c_size_t]),
    ("gcs_diagnose_with_json", STR, [PTR, PTR, PTR, ct.c_size_t]),
    ("gcs_minimal_conflict_set_json", STR, [PTR, PTR, ct.c_size_t, ct.c_double]),
    ("gcs_violated_json", STR, [PTR, ct.c_double]),
    ("gcs_distance_rigidity_json", STR, [PTR]),
    ("gcs_witness_json", STR, [PTR, ct.c_uint32]),
    ("gcs_make_witness", None, [PTR, ct.c_uint32, F64]),
    # decomposition
    ("gcs_graph_json", STR, [PTR]),
    ("gcs_plan_solver_new", PTR, [PTR, ct.c_int32]),
    ("gcs_plan_solver_free", None, [PTR]),
    ("gcs_plan_solver_system", PTR, [PTR]),
    ("gcs_plan_solver_plan_json", STR, [PTR]),
    ("gcs_plan_solver_graph_json", STR, [PTR]),
    ("gcs_plan_solver_solve", STR, [PTR, PTR, ct.c_double, ct.c_int32, ct.c_int32]),
    ("gcs_plan_solver_flip", ct.c_int32, [PTR, PTR, ct.c_int32]),
    ("gcs_plan_solver_sticky", None, [PTR, ct.c_int32]),
    ("gcs_plan_solver_execute", None, [PTR, PTR]),
    ("gcs_plan_solver_point_element", ct.c_int32, [PTR, ct.c_int32]),
    ("gcs_ppp_triangles", ct.c_int32, [PTR, I32]),
    ("gcs_plan_steps_placing", ct.c_int32, [PTR, ct.c_int32, I32]),
    # homotopy
    ("gcs_enumerate_step_json", STR,
     [PTR, PTR, ct.c_int32, ct.c_int32, ct.c_uint32, ct.c_int32]),
    ("gcs_apply_alternative", None, [PTR, PTR, ct.c_int32, PTR, ct.c_size_t]),
    # drags
    ("gcs_drag_new", PTR,
     [PTR, ct.c_int32, ct.c_double, ct.c_double, ct.c_int32, ct.c_double, I32, ct.c_int32,
      ct.c_double]),
    ("gcs_drag_move", STR, [PTR, PTR, ct.c_double, ct.c_double, F64]),
    ("gcs_drag_end", None, [PTR, PTR]),
    ("gcs_drag_free", None, [PTR]),
    ("gcs_drag_flips", ct.c_int32, [PTR]),
    ("gcs_drag_flip_list", ct.c_int32, [PTR, I32]),
    ("gcs_radius_drag_new", PTR, [PTR, ct.c_int32, ct.c_int32, ct.c_double, ct.c_int32]),
    ("gcs_radius_drag_move", STR, [PTR, PTR, ct.c_double, F64]),
    ("gcs_radius_drag_end", None, [PTR, PTR]),
    ("gcs_radius_drag_free", None, [PTR]),
    ("gcs_plan_drag_new", PTR,
     [PTR, PTR, ct.c_int32, ct.c_double, ct.c_double, I32, ct.c_int32, ct.c_double]),
    ("gcs_plan_drag_move", STR, [PTR, PTR, ct.c_double, ct.c_double, F64]),
    ("gcs_plan_drag_usable", ct.c_int32, [PTR]),
    ("gcs_plan_drag_flips", ct.c_int32, [PTR]),
    ("gcs_plan_drag_flip_list", ct.c_int32, [PTR, I32]),
    ("gcs_plan_drag_branches_json", STR, [PTR]),
    ("gcs_plan_drag_guards", ct.c_int32, [PTR, PTR, I32]),
    ("gcs_plan_drag_end", None, [PTR, PTR]),
    ("gcs_plan_drag_free", None, [PTR]),
]


def _load() -> ct.CDLL:
    lib = ct.CDLL(str(_find_library()))
    for name, restype, argtypes in _SIGNATURES:
        fn = getattr(lib, name)
        fn.restype = restype
        fn.argtypes = argtypes
    return lib


lib = _load()


# -- strings ----------------------------------------------------------------

def take_str(handle: int | None) -> str:
    """Consume a length-prefixed string block from the core."""
    if not handle:
        return ""
    n = lib.gcs_str_len(handle)
    p = lib.gcs_str_ptr(handle)
    out = ct.string_at(p, n).decode() if n else ""
    lib.gcs_str_free(handle)
    return out


def take_json(handle: int | None) -> Any:
    s = take_str(handle)
    return json.loads(s) if s else None


class Bytes:
    """A UTF-8 argument: the pointer/length pair the ABI expects."""

    __slots__ = ("buf", "ptr", "len")

    def __init__(self, s: str) -> None:
        self.buf = s.encode()
        self.ptr = ct.cast(ct.c_char_p(self.buf), PTR)
        self.len = len(self.buf)


def send(s: str) -> tuple[Any, int]:
    b = Bytes(s)
    return b.ptr, b.len


def send_json(obj: Any) -> tuple[Any, int]:
    return send(json.dumps(obj))


def last_error() -> str:
    return take_str(lib.gcs_last_error())


# -- array views ------------------------------------------------------------

def f64(n: int) -> Vec:
    return np.zeros(n, dtype=np.float64)


def i32(n: int) -> IVec:
    return np.zeros(n, dtype=np.int32)


def pf(a: Vec) -> Any:
    return a.ctypes.data_as(F64)


def pi(a: IVec) -> Any:
    return a.ctypes.data_as(I32)


def pu8(a: npt.NDArray[np.uint8]) -> Any:
    return a.ctypes.data_as(U8)


def as_f64(a: Any) -> Vec:
    return np.ascontiguousarray(a, dtype=np.float64)


def as_i32(a: Any) -> IVec:
    return np.ascontiguousarray(a, dtype=np.int32)
