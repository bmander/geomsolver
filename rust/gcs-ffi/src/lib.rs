//! gcs — the flat C ABI over `gcs-core`.
//!
//! One library, built twice: the web app instantiates it compiled to WebAssembly, and the native
//! `cdylib` is the released C ABI for anything else that speaks C.  Everything above the numbers
//! lives in Rust; a binding only marshals.
//!
//! Conventions
//!   * handles are opaque pointers, freed by their `*_free`;
//!   * hot-path numbers cross as caller-owned `f64`/`i32` buffers;
//!   * everything ragged (diagnosis, plans, constraint lists) crosses as UTF-8 JSON in a
//!     length-prefixed block: `gcs_str_len` / `gcs_str_ptr` / `gcs_str_free`;
//!   * a function returning a handle returns null on failure, with the reason in
//!     `gcs_last_error()`.

#![allow(clippy::missing_safety_doc)]

use gcs_core::callout;
use gcs_core::cgraph::{self, El};
use gcs_core::constraints::Constraint;
use gcs_core::curve;
use gcs_core::decompose::{self, Plan, PlanDrag, PlanSolver};
use gcs_core::diagnose::{self, DiagnoseOptions};
use gcs_core::expr;
use gcs_core::homotopy::{self, EnumerateOptions};
use gcs_core::io;
use gcs_core::json::{self, Json};
use gcs_core::model::{self, EntKind, EntRef, Param, Sketch};
use gcs_core::newton::{self, Method};
use gcs_core::program::{self, Elaborated};
use gcs_core::report;
use gcs_core::syntax;
use gcs_core::solve::{Drag, RadiusDrag, SolveOpts, SolveResult, Triangle};
use gcs_core::system::System;
use gcs_core::{examples, fdcheck, kernels, linalg, witness};
use std::alloc::{alloc, dealloc, Layout};
use std::cell::RefCell;
use std::collections::BTreeMap;

thread_local! {
    static LAST_ERROR: RefCell<String> = const { RefCell::new(String::new()) };
}

fn set_error(msg: impl Into<String>) {
    LAST_ERROR.with(|e| *e.borrow_mut() = msg.into());
}

/// The panic boundary.  Every entry point runs inside this: a panic in the core (a bad index, a
/// broken invariant) becomes `gcs_last_error()` and a neutral return value instead of an abort
/// that would take the host process with it.  `wasm32-unknown-unknown`'s panic strategy is abort,
/// so on the web the core's own bounds checks are the defence and this is belt to that braces.
fn guard<T>(fallback: T, f: impl FnOnce() -> T) -> T {
    match std::panic::catch_unwind(std::panic::AssertUnwindSafe(f)) {
        Ok(v) => v,
        Err(e) => {
            let msg = e
                .downcast_ref::<&str>()
                .map(|s| (*s).to_string())
                .or_else(|| e.downcast_ref::<String>().cloned())
                .unwrap_or_else(|| "panic".to_string());
            set_error(format!("internal error: {msg}"));
            fallback
        }
    }
}

/* -- raw memory ------------------------------------------------------------ */

fn layout(size: usize) -> Layout {
    Layout::from_size_align(size.max(1), 8).unwrap()
}

/// Raw heap block for callers that need to hand buffers in (the WebAssembly binding).
#[no_mangle]
pub extern "C" fn gcs_malloc(size: usize) -> *mut u8 {
    unsafe { alloc(layout(size)) }
}

#[no_mangle]
pub unsafe extern "C" fn gcs_free(ptr: *mut u8, size: usize) {
    if !ptr.is_null() {
        dealloc(ptr, layout(size));
    }
}

/// A UTF-8 string as `[u32 length][bytes]`, owned by the caller until `gcs_str_free`.
fn out_str(s: String) -> *mut u8 {
    let bytes = s.into_bytes();
    let total = 4 + bytes.len();
    unsafe {
        let p = alloc(layout(total));
        (p as *mut u32).write_unaligned(bytes.len() as u32);
        std::ptr::copy_nonoverlapping(bytes.as_ptr(), p.add(4), bytes.len());
        p
    }
}

fn out_json(v: Json) -> *mut u8 {
    out_str(v.dump(None))
}

#[no_mangle]
pub unsafe extern "C" fn gcs_str_len(p: *const u8) -> u32 {
    if p.is_null() {
        0
    } else {
        (p as *const u32).read_unaligned()
    }
}

#[no_mangle]
pub unsafe extern "C" fn gcs_str_ptr(p: *const u8) -> *const u8 {
    if p.is_null() {
        p
    } else {
        p.add(4)
    }
}

#[no_mangle]
pub unsafe extern "C" fn gcs_str_free(p: *mut u8) {
    if !p.is_null() {
        let len = (p as *const u32).read_unaligned() as usize;
        dealloc(p, layout(4 + len));
    }
}

#[no_mangle]
pub extern "C" fn gcs_last_error() -> *mut u8 {
    out_str(LAST_ERROR.with(|e| e.borrow().clone()))
}

unsafe fn as_str<'a>(ptr: *const u8, len: usize) -> &'a str {
    if ptr.is_null() || len == 0 {
        return "";
    }
    std::str::from_utf8(std::slice::from_raw_parts(ptr, len)).unwrap_or("")
}

unsafe fn as_json(ptr: *const u8, len: usize) -> Json {
    json::parse(as_str(ptr, len)).unwrap_or(Json::Null)
}

/// `[[kind, idx], ...]` as the entities it names, skipping any the model does not know.
unsafe fn ent_list(p: *const u8, len: usize) -> Vec<EntRef> {
    as_json(p, len)
        .arr()
        .iter()
        .filter_map(|v| {
            let a = v.arr();
            EntKind::parse(a.first().map(|x| x.as_str()).unwrap_or(""))
                .map(|k| EntRef::new(k, a.get(1).map(|x| x.as_i64()).unwrap_or(0) as usize))
        })
        .collect()
}

unsafe fn sk<'a>(h: *mut Sketch) -> &'a mut Sketch {
    &mut *h
}

/// A Param by index, with the index checked.  A binding that hands us a stale index gets a
/// caught panic and `gcs_last_error()`, not whatever happens to be past the end of the vector.
fn param(s: &Sketch, i: i32) -> &Param {
    let n = s.params.len();
    s.params.get(i as usize).unwrap_or_else(|| panic!("param index {i} out of range (0..{n})"))
}

fn param_mut(s: &mut Sketch, i: i32) -> &mut Param {
    let n = s.params.len();
    s.params.get_mut(i as usize).unwrap_or_else(|| panic!("param index {i} out of range (0..{n})"))
}

/* -- writing into caller buffers ------------------------------------------- */

/// Copy a slice into a caller-owned buffer.  Every hot-path result crosses this way, and writing
/// the loop out at each of them is a chance to get the length or the pointer arithmetic wrong.
unsafe fn write<T: Copy>(out: *mut T, src: &[T]) {
    if out.is_null() {
        return;
    }
    std::ptr::copy_nonoverlapping(src.as_ptr(), out, src.len());
}

/// The same, mapping each element on the way (`bool` flags, `usize` indices).
unsafe fn write_map<S, T>(out: *mut T, src: &[S], f: impl Fn(&S) -> T) {
    if out.is_null() {
        return;
    }
    for (i, v) in src.iter().enumerate() {
        *out.add(i) = f(v);
    }
}

/* -- library metadata ------------------------------------------------------ */

/// The constraint-type and kernel registry — everything a front end needs to be generic.
#[no_mangle]
pub extern "C" fn gcs_registry_json() -> *mut u8 {
    guard(std::ptr::null_mut(), move || {
        out_json(report::registry_json())
    })
}

#[no_mangle]
pub extern "C" fn gcs_kernel_count() -> i32 {
    guard(-1, move || {
        kernels::N_KERNELS as i32
    })
}

#[no_mangle]
pub extern "C" fn gcs_version() -> *mut u8 {
    guard(std::ptr::null_mut(), move || {
        out_str(env!("CARGO_PKG_VERSION").to_string())
    })
}

/* -- sketch ---------------------------------------------------------------- */

#[no_mangle]
pub extern "C" fn gcs_sketch_new() -> *mut Sketch {
    guard(std::ptr::null_mut(), move || {
        Box::into_raw(Box::new(Sketch::new()))
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_sketch_free(h: *mut Sketch) {
    guard((), move || {
        if !h.is_null() {
            drop(Box::from_raw(h));
        }
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_sketch_clone(h: *mut Sketch) -> *mut Sketch {
    guard(std::ptr::null_mut(), move || {
        Box::into_raw(Box::new(sk(h).clone()))
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_sketch_from_json(ptr: *const u8, len: usize) -> *mut Sketch {
    guard(std::ptr::null_mut(), move || {
        match io::loads(as_str(ptr, len)) {
            Ok(s) => Box::into_raw(Box::new(s)),
            Err(e) => {
                set_error(e);
                std::ptr::null_mut()
            }
        }
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_sketch_to_json(h: *mut Sketch, indent: i32) -> *mut u8 {
    guard(std::ptr::null_mut(), move || {
        let ind = if indent < 0 { None } else { Some(indent as usize) };
        out_str(io::dumps(sk(h), ind))
    })
}

/// How many integers `gcs_sketch_counts` writes.
///
/// Asked rather than assumed: a binding that hard-codes the width writes past its buffer the day
/// a new entity kind is added, and the damage shows up as a crash somewhere else entirely.
#[no_mangle]
pub unsafe extern "C" fn gcs_counts_len() -> i32 {
    N_COUNTS as i32
}

const N_COUNTS: usize = 10;

#[no_mangle]
pub unsafe extern "C" fn gcs_sketch_counts(h: *mut Sketch, out: *mut i32) {
    guard((), move || {
        let s = sk(h);
        let v = [
            s.params.len(),
            s.points.len(),
            s.lines.len(),
            s.circles.len(),
            s.arcs.len(),
            s.constraints.len(),
            s.splines.len(),
            s.ellipses.len(),
            s.curves.len(),
            // appended, never inserted: the positions above are what the bindings hard-code
            s.frames.len(),
        ];
        debug_assert_eq!(v.len(), N_COUNTS, "gcs_counts_len is what callers size their buffer by");
        for (i, x) in v.iter().enumerate() {
            *out.add(i) = *x as i32;
        }
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_sketch_point(
    h: *mut Sketch,
    x: f64,
    y: f64,
    fixed: i32,
    name: *const u8,
    name_len: usize,
) -> i32 {
    guard(-1, move || {
        sk(h).point(x, y, fixed != 0, as_str(name, name_len)) as i32
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_sketch_line(h: *mut Sketch, p1: i32, p2: i32) -> i32 {
    guard(-1, move || {
        sk(h).line(p1 as usize, p2 as usize) as i32
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_sketch_circle(
    h: *mut Sketch,
    center: i32,
    r: f64,
    name: *const u8,
    name_len: usize,
) -> i32 {
    guard(-1, move || {
        sk(h).circle(center as usize, r, as_str(name, name_len)) as i32
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_sketch_arc(
    h: *mut Sketch,
    center: i32,
    start: i32,
    end: i32,
    name: *const u8,
    name_len: usize,
) -> i32 {
    guard(-1, move || {
        sk(h).arc(center as usize, start as usize, end as usize, as_str(name, name_len)) as i32
    })
}

/// An ellipse about `center` whose major axis ends at `major`, with minor radius `b`.
#[no_mangle]
pub unsafe extern "C" fn gcs_sketch_ellipse(
    h: *mut Sketch,
    center: i32,
    major: i32,
    b: f64,
    name: *const u8,
    name_len: usize,
) -> i32 {
    guard(-1, move || {
        sk(h).ellipse(center as usize, major as usize, b, as_str(name, name_len)) as i32
    })
}

/// A frame at `origin` pointed at `toward` — the rotor and its two intrinsic constraints come
/// with it, exactly as an arc's endpoint incidences come with the arc.
#[no_mangle]
pub unsafe extern "C" fn gcs_sketch_frame(
    h: *mut Sketch,
    origin: i32,
    toward: i32,
    name: *const u8,
    name_len: usize,
) -> i32 {
    guard(-1, move || {
        sk(h).frame(origin as usize, toward as usize, as_str(name, name_len)) as i32
    })
}

/// The minor radius that puts the rim of the ellipse (centre c, major end m) through (tx, ty)
/// — the ellipse tool's third click, and where a rim drag holds the rim to the cursor.
/// Negative when centre and major end coincide, which names no axis.
#[no_mangle]
pub extern "C" fn gcs_ellipse_minor(
    cx: f64,
    cy: f64,
    mx: f64,
    my: f64,
    tx: f64,
    ty: f64,
) -> f64 {
    guard(-1.0, move || {
        gcs_core::ellipse::minor_to(cx, cy, mx, my, tx, ty).unwrap_or(-1.0)
    })
}

/// A cubic B-spline over `n` control points, with the clamped uniform knot vector; -1 when
/// there are too few of them for a cubic.
#[no_mangle]
pub unsafe extern "C" fn gcs_sketch_spline(h: *mut Sketch, ctrl: *const i32, n: usize) -> i32 {
    guard(-1, move || {
        let ids: Vec<usize> =
            std::slice::from_raw_parts(ctrl, n).iter().map(|&i| i as usize).collect();
        sk(h).spline(&ids).map(|i| i as i32).unwrap_or(-1)
    })
}

/// A cubic B-spline through `n` (x, y) pairs, in order — the control points are computed.
///
/// `hold` is `n` point indices, `-1` for a place that came from empty space; the rest become
/// `PointOnSpline` contacts pinned at the parameter the fit chose, so a curve fitted to
/// constrained points is itself fully constrained.  Pass null to hold none of them.
/// -1 when there are too few for a cubic, or they give no parameterisation.
#[no_mangle]
pub unsafe extern "C" fn gcs_sketch_spline_through(
    h: *mut Sketch,
    pts: *const f64,
    n: usize,
    hold: *const i32,
) -> i32 {
    guard(-1, move || {
        let v = std::slice::from_raw_parts(pts, 2 * n);
        let q: Vec<(f64, f64)> = (0..n).map(|i| (v[2 * i], v[2 * i + 1])).collect();
        let held: Vec<Option<usize>> = if hold.is_null() {
            vec![None; n]
        } else {
            std::slice::from_raw_parts(hold, n)
                .iter()
                .map(|&i| if i < 0 { None } else { Some(i as usize) })
                .collect()
        };
        sk(h).spline_through_held(&q, &held).map(|i| i as i32).unwrap_or(-1)
    })
}

/// A cubic B-spline with a knot vector of its own; -1 when it does not fit the control polygon.
#[no_mangle]
pub unsafe extern "C" fn gcs_sketch_spline_knots(
    h: *mut Sketch,
    ctrl: *const i32,
    n: usize,
    knots: *const f64,
    nk: usize,
) -> i32 {
    guard(-1, move || {
        let ids: Vec<usize> =
            std::slice::from_raw_parts(ctrl, n).iter().map(|&i| i as usize).collect();
        let ks = std::slice::from_raw_parts(knots, nk).to_vec();
        sk(h).spline_with(&ids, Some(ks)).map(|i| i as i32).unwrap_or(-1)
    })
}

/// A spline's knot vector; returns how many were written.
#[no_mangle]
pub unsafe extern "C" fn gcs_spline_knots(h: *mut Sketch, idx: i32, out: *mut f64) -> i32 {
    guard(-1, move || {
        let k = &sk(h).splines[idx as usize].knots;
        for (i, v) in k.iter().enumerate() {
            *out.add(i) = *v;
        }
        k.len() as i32
    })
}

/// The parameter interval a spline is drawn over (2 doubles).
#[no_mangle]
pub unsafe extern "C" fn gcs_spline_domain(h: *mut Sketch, idx: i32, out: *mut f64) {
    guard((), move || {
        let (a, b) = curve::domain(sk(h), idx as usize);
        *out = a;
        *out.add(1) = b;
    })
}

/// C(t), C'(t) and C''(t) at one parameter (6 doubles).
#[no_mangle]
pub unsafe extern "C" fn gcs_spline_eval(h: *mut Sketch, idx: i32, t: f64, out: *mut f64) {
    guard((), move || {
        let f = curve::eval(sk(h), idx as usize, t);
        let v = [f.p.0, f.p.1, f.d1.0, f.d1.1, f.d2.0, f.d2.1];
        for (i, x) in v.iter().enumerate() {
            *out.add(i) = *x;
        }
    })
}

/// The curve as a polyline, refined to `unit` (the world length of one screen pixel) exactly as
/// the callouts are: the core lays the figure out and the front end strokes what it is handed.
///
/// Returns how many points the curve *needs*, and writes x, y pairs for as many as `cap` allows.
/// A caller that guessed big enough is done in one call; one that did not sees a number larger
/// than its `cap` and calls again — which beats asking the length first, since that would
/// tessellate the curve twice on every call rather than on the rare miss.
#[no_mangle]
pub unsafe extern "C" fn gcs_spline_polyline(
    h: *mut Sketch,
    idx: i32,
    unit: f64,
    out: *mut f64,
    cap: i32,
) -> i32 {
    guard(-1, move || {
        let pts = curve::tessellate(sk(h), idx as usize, unit);
        for (i, p) in pts.iter().take(cap.max(0) as usize).enumerate() {
            *out.add(2 * i) = p.0;
            *out.add(2 * i + 1) = p.1;
        }
        pts.len() as i32
    })
}

/// Give the curve one more control point at `t`, without changing its shape — Boehm's knot
/// insertion.  Returns the new control Point's index, or -1 if `t` is not a place a knot can go.
#[no_mangle]
pub unsafe extern "C" fn gcs_spline_insert_control(h: *mut Sketch, idx: i32, t: f64) -> i32 {
    guard(-1, move || {
        curve::insert_control(sk(h), idx as usize, t).map(|p| p as i32).unwrap_or(-1)
    })
}

/// The parameter of the curve point nearest (x, y), and how far that is (2 doubles) — the pick
/// test, so a front end never converts a tolerance or evaluates a basis function.
#[no_mangle]
pub unsafe extern "C" fn gcs_spline_closest(
    h: *mut Sketch,
    idx: i32,
    x: f64,
    y: f64,
    out: *mut f64,
) {
    guard((), move || {
        let (t, d) = curve::closest(sk(h), idx as usize, x, y);
        *out = t;
        *out.add(1) = d;
    })
}

/// The three-point construction; -1 when the three points are collinear.
#[no_mangle]
pub unsafe extern "C" fn gcs_sketch_arc_through(
    h: *mut Sketch,
    start: i32,
    end: i32,
    tx: f64,
    ty: f64,
    name: *const u8,
    name_len: usize,
) -> i32 {
    guard(-1, move || {
        sk(h)
            .arc_through(start as usize, end as usize, (tx, ty), as_str(name, name_len))
            .map(|i| i as i32)
            .unwrap_or(-1)
    })
}

/// Four lines and three perpendiculars; writes the line indices into `out` (4 ints).
#[no_mangle]
pub unsafe extern "C" fn gcs_sketch_rectangle(
    h: *mut Sketch,
    a: i32,
    x1: f64,
    y1: f64,
    name: *const u8,
    name_len: usize,
    out: *mut i32,
) {
    guard((), move || {
        let lines = sk(h).rectangle(a as usize, x1, y1, as_str(name, name_len));
        for (i, l) in lines.iter().enumerate() {
            *out.add(i) = *l as i32;
        }
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_sketch_get_x(h: *mut Sketch, out: *mut f64) {
    guard((), move || {
        let s = sk(h);
        for (i, p) in s.params.iter().enumerate() {
            *out.add(i) = p.value;
        }
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_sketch_set_x(h: *mut Sketch, x: *const f64, n: usize) -> i32 {
    guard(-1, move || {
        let s = sk(h);
        if s.set_x(std::slice::from_raw_parts(x, n)) {
            0
        } else {
            set_error(format!("set_x: {n} values for {} params", s.params.len()));
            -1
        }
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_sketch_perturb(h: *mut Sketch, sigma: f64, seed: u32) {
    guard((), move || {
        sk(h).perturb(sigma, seed);
    })
}

/// What a compiled plan or System depends on — entity counts, constraint ids and types, fixed
/// flags.  A front end caching a compiled artefact keys on this.
#[no_mangle]
pub unsafe extern "C" fn gcs_sketch_topology_key(h: *mut Sketch) -> *mut u8 {
    guard(std::ptr::null_mut(), move || out_str(sk(h).topology_key()))
}

#[no_mangle]
pub unsafe extern "C" fn gcs_sketch_extent(h: *mut Sketch) -> f64 {
    guard(f64::NAN, move || {
        sk(h).extent()
    })
}

/// `bbox` when `drawn == 0`, `drawn_bounds` otherwise; writes 4 doubles.
#[no_mangle]
pub unsafe extern "C" fn gcs_sketch_bounds(h: *mut Sketch, drawn: i32, out: *mut f64) {
    guard((), move || {
        let s = sk(h);
        let b = if drawn != 0 { s.drawn_bounds() } else { s.bbox() };
        *out = b.0;
        *out.add(1) = b.1;
        *out.add(2) = b.2;
        *out.add(3) = b.3;
    })
}

/// Index of the nearest point and its distance (index -1 when the sketch has none).
#[no_mangle]
pub unsafe extern "C" fn gcs_sketch_nearest_point(
    h: *mut Sketch,
    x: f64,
    y: f64,
    out_dist: *mut f64,
) -> i32 {
    guard(-1, move || {
        let (i, d) = sk(h).nearest_point(x, y);
        *out_dist = d;
        i.map(|v| v as i32).unwrap_or(-1)
    })
}

/// What a click at (x, y) picks within `tol` (a world length): writes [kind, index] and
/// returns 1, or returns 0 when nothing drawn is within reach.
#[no_mangle]
pub unsafe extern "C" fn gcs_sketch_pick(
    h: *mut Sketch,
    x: f64,
    y: f64,
    tol: f64,
    out: *mut f64,
) -> i32 {
    guard(0, move || match model::pick(sk(h), x, y, tol) {
        Some(e) => {
            write(out, &[kind_id(e.kind) as f64, e.i() as f64]);
            1
        }
        None => 0,
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_sketch_n_residuals(h: *mut Sketch) -> i32 {
    guard(-1, move || {
        sk(h).n_residuals() as i32
    })
}

/* -- params ---------------------------------------------------------------- */

#[no_mangle]
pub unsafe extern "C" fn gcs_param_value(h: *mut Sketch, i: i32) -> f64 {
    guard(f64::NAN, move || {
        param(sk(h), i).value
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_param_set_value(h: *mut Sketch, i: i32, v: f64) {
    guard((), move || {
        param_mut(sk(h), i).value = v;
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_param_fixed(h: *mut Sketch, i: i32) -> i32 {
    guard(-1, move || {
        param(sk(h), i).fixed as i32
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_param_set_fixed(h: *mut Sketch, i: i32, fixed: i32) {
    guard((), move || {
        param_mut(sk(h), i).fixed = fixed != 0;
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_param_name(h: *mut Sketch, i: i32) -> *mut u8 {
    guard(std::ptr::null_mut(), move || {
        out_str(param(sk(h), i).name.clone())
    })
}

/* -- entities -------------------------------------------------------------- */

fn kind_id(k: EntKind) -> i32 {
    match k {
        EntKind::Point => 0,
        EntKind::Line => 1,
        EntKind::Circle => 2,
        EntKind::Arc => 3,
        EntKind::Spline => 4,
        EntKind::Ellipse => 5,
        EntKind::Curve => 6,
        EntKind::Frame => 7,
    }
}

fn ent(kind: i32, idx: i32) -> EntRef {
    let k = match kind {
        0 => EntKind::Point,
        1 => EntKind::Line,
        2 => EntKind::Circle,
        3 => EntKind::Arc,
        5 => EntKind::Ellipse,
        6 => EntKind::Curve,
        7 => EntKind::Frame,
        _ => EntKind::Spline,
    };
    EntRef::new(k, idx as usize)
}

/// Every Param index of an entity, in the model's canonical order; returns how many were written.
#[no_mangle]
pub unsafe extern "C" fn gcs_entity_params(
    h: *mut Sketch,
    kind: i32,
    idx: i32,
    out: *mut i32,
) -> i32 {
    guard(-1, move || {
        let ps = sk(h).entity_params(ent(kind, idx));
        for (i, p) in ps.iter().enumerate() {
            *out.add(i) = *p as i32;
        }
        ps.len() as i32
    })
}

/// The point indices an entity is built from: line → (p1, p2); circle → (centre); arc →
/// (centre, start, end); ellipse → (centre, major end).  Returns how many were written.
#[no_mangle]
pub unsafe extern "C" fn gcs_entity_points(
    h: *mut Sketch,
    kind: i32,
    idx: i32,
    out: *mut i32,
) -> i32 {
    guard(-1, move || {
        let s = sk(h);
        let v: Vec<usize> = match kind {
            1 => vec![s.lines[idx as usize].p1 as usize, s.lines[idx as usize].p2 as usize],
            2 => vec![s.circles[idx as usize].center as usize],
            3 => {
                let a = &s.arcs[idx as usize];
                vec![a.center as usize, a.start as usize, a.end as usize]
            }
            4 => s.splines[idx as usize].ctrl.iter().map(|&c| c as usize).collect(),
            5 => {
                let e = &s.ellipses[idx as usize];
                vec![e.center as usize, e.major as usize]
            }
            7 => {
                let f = &s.frames[idx as usize];
                vec![f.origin as usize, f.toward as usize]
            }
            _ => vec![idx as usize],
        };
        for (i, p) in v.iter().enumerate() {
            *out.add(i) = *p as i32;
        }
        v.len() as i32
    })
}

/// The radius Param index of a circle or arc, or an ellipse's minor radius (-1 otherwise).
#[no_mangle]
pub unsafe extern "C" fn gcs_entity_radius_param(h: *mut Sketch, kind: i32, idx: i32) -> i32 {
    guard(-1, move || {
        if kind != 2 && kind != 3 && kind != 5 {
            return -1;
        }
        sk(h).round_radius(ent(kind, idx)) as i32
    })
}

/// The classes an entity carries, as a JSON array of strings.
#[no_mangle]
pub unsafe extern "C" fn gcs_entity_class(h: *mut Sketch, kind: i32, idx: i32) -> *mut u8 {
    guard(std::ptr::null_mut(), move || {
        let c = sk(h).class_of(ent(kind, idx));
        out_json(Json::Arr(c.0.into_iter().map(Json::Str).collect()))
    })
}

/// Give an entity a class, or take one away.
#[no_mangle]
pub unsafe extern "C" fn gcs_entity_set_class(
    h: *mut Sketch,
    kind: i32,
    idx: i32,
    name: *const u8,
    name_len: usize,
    on: i32,
) {
    guard((), move || {
        sk(h).set_class(ent(kind, idx), as_str(name, name_len), on != 0);
    })
}

/// What an entity is *drawn with*: dash, width and colour, resolved from the base sheet under
/// the document's (`style.rs`).  The core resolves and a front end strokes what it is handed —
/// the same seam callout layout and curve tessellation sit on.
#[no_mangle]
pub unsafe extern "C" fn gcs_entity_style(h: *mut Sketch, kind: i32, idx: i32) -> *mut u8 {
    guard(std::ptr::null_mut(), move || out_json(style_json(&sk(h).style_of(ent(kind, idx)))))
}

/// Every resolved style in one document, in `primitives()` order per kind, plus the sheet's own
/// epoch.  One call a repaint, rather than one a shape: presentation almost never changes and
/// geometry changes every frame, and this is what lets the first be read at the first's rate.
#[no_mangle]
pub unsafe extern "C" fn gcs_styles_json(h: *mut Sketch) -> *mut u8 {
    guard(std::ptr::null_mut(), move || {
        let s = sk(h);
        let of = |k: EntKind| -> Json {
            Json::Arr(
                (0..s.count(k))
                    .map(|i| style_json(&s.style_of(EntRef::new(k, i))))
                    .collect(),
            )
        };
        out_json(json::object([
            ("epoch", Json::Int(s.style_epoch as i64)),
            ("line", of(EntKind::Line)),
            ("circle", of(EntKind::Circle)),
            ("arc", of(EntKind::Arc)),
            ("spline", of(EntKind::Spline)),
            ("ellipse", of(EntKind::Ellipse)),
            ("frame", of(EntKind::Frame)),
            ("curve", of(EntKind::Curve)),
        ]))
    })
}

/// Bumped whenever the sheet or a class changes, so a caller may cache the table above.
#[no_mangle]
pub unsafe extern "C" fn gcs_style_epoch(h: *mut Sketch) -> i32 {
    guard(-1, move || sk(h).style_epoch as i32)
}

fn style_json(s: &gcs_core::style::Style) -> Json {
    json::object([
        (
            "dash",
            Json::Arr(s.dash.clone().unwrap_or_default().into_iter().map(Json::Num).collect()),
        ),
        ("width", s.width.map(Json::Num).unwrap_or(Json::Null)),
        ("color", s.color.clone().map(Json::Str).unwrap_or(Json::Null)),
    ])
}

/// Bounding box of one entity (4 doubles).
#[no_mangle]
pub unsafe extern "C" fn gcs_entity_bounds(h: *mut Sketch, kind: i32, idx: i32, out: *mut f64) {
    guard((), move || {
        let b = sk(h).bounds(ent(kind, idx));
        *out = b.0;
        *out.add(1) = b.1;
        *out.add(2) = b.2;
        *out.add(3) = b.3;
    })
}

/// `P0` / `L3` / `C1` / `A2` — the short label the constraint list and reports use.
#[no_mangle]
pub extern "C" fn gcs_entity_name(kind: i32, idx: i32) -> *mut u8 {
    guard(std::ptr::null_mut(), move || {
        out_str(io::entity_name(ent(kind, idx)))
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_distance_between(
    h: *mut Sketch,
    ka: i32,
    ia: i32,
    kb: i32,
    ib: i32,
) -> f64 {
    guard(f64::NAN, move || {
        model::distance_between(sk(h), ent(ka, ia), ent(kb, ib))
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_signed_point_to_line(
    h: *mut Sketch,
    x: f64,
    y: f64,
    line: i32,
) -> f64 {
    guard(f64::NAN, move || {
        model::signed_point_to_line(sk(h), x, y, line as usize)
    })
}

/// The arc's CCW sweep (2 doubles).
/// Signed CCW angle from line `a` to line `b`, in radians.
#[no_mangle]
pub unsafe extern "C" fn gcs_angle_between(h: *mut Sketch, a: i32, b: i32) -> f64 {
    guard(f64::NAN, move || {
        model::angle_between(sk(h), EntRef::line(a as usize), EntRef::line(b as usize))
    })
}

/// The point at distance `r` from (cx, cy) towards (tx, ty); 0 if the target is the centre.
#[no_mangle]
pub unsafe extern "C" fn gcs_on_radius(
    cx: f64,
    cy: f64,
    tx: f64,
    ty: f64,
    r: f64,
    out: *mut f64,
) -> i32 {
    guard(0, move || match model::on_radius(cx, cy, tx, ty, r) {
        Some((x, y)) => {
            write(out, &[x, y]);
            1
        }
        None => 0,
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_arc_angles(h: *mut Sketch, idx: i32, out: *mut f64) {
    guard((), move || {
        let (a0, a1) = sk(h).arc_angles(idx as usize);
        *out = a0;
        *out.add(1) = a1;
    })
}

/// The circumcircle construction: writes (cx, cy, r, a0, a1, swapped); 0 when collinear.
#[no_mangle]
pub unsafe extern "C" fn gcs_three_point_arc(
    ax: f64,
    ay: f64,
    bx: f64,
    by: f64,
    cx: f64,
    cy: f64,
    out: *mut f64,
) -> i32 {
    guard(-1, move || {
        match model::three_point_arc(ax, ay, bx, by, cx, cy, 1e-9) {
            None => 0,
            Some(g) => {
                let v = [g.cx, g.cy, g.r, g.a0, g.a1, if g.swapped { 1.0 } else { 0.0 }];
                for (i, x) in v.iter().enumerate() {
                    *out.add(i) = *x;
                }
                1
            }
        }
    })
}

/* -- constraints ----------------------------------------------------------- */

/// Add a constraint from its JSON form; returns its document-stable id, or -1 on error.
#[no_mangle]
pub unsafe extern "C" fn gcs_constraint_add(
    h: *mut Sketch,
    ptr: *const u8,
    len: usize,
) -> i32 {
    guard(-1, move || {
        let s = sk(h);
        let v = as_json(ptr, len);
        match report::constraint_from_json(s, &v) {
            Ok(c) => s.add(c) as i32,
            Err(e) => {
                set_error(e);
                -1
            }
        }
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_constraint_remove(h: *mut Sketch, id: i32) {
    guard((), move || {
        sk(h).remove(id as u32);
    })
}

/// Keep exactly these constraint ids, in this order — the one bulk edit the model allows
/// (a filtered constraint list, as diagnosis trials and the UI's delete perform).
#[no_mangle]
pub unsafe extern "C" fn gcs_sketch_set_constraints(
    h: *mut Sketch,
    ptr: *const u8,
    len: usize,
) {
    guard((), move || {
        let s = sk(h);
        let v = as_json(ptr, len);
        let ids: Vec<u32> = v.arr().iter().map(|x| x.as_i64() as u32).collect();
        let mut by_id: BTreeMap<u32, Constraint> =
            s.constraints.drain(..).map(|c| (c.id, c)).collect();
        for id in ids {
            if let Some(c) = by_id.remove(&id) {
                s.constraints.push(c);
            }
        }
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_constraints_json(h: *mut Sketch) -> *mut u8 {
    guard(std::ptr::null_mut(), move || {
        out_json(report::constraints_json(sk(h)))
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_constraint_json(h: *mut Sketch, id: i32) -> *mut u8 {
    guard(std::ptr::null_mut(), move || {
        let s = sk(h);
        match s.constraint(id as u32) {
            Some(c) => out_json(report::constraint_json(s, c)),
            None => out_json(Json::Null),
        }
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_constraint_set_num(
    h: *mut Sketch,
    id: i32,
    name: *const u8,
    name_len: usize,
    v: f64,
) -> i32 {
    guard(-1, move || {
        let n = as_str(name, name_len).to_string();
        match sk(h).constraint(id as u32) {
            None => {
                set_error(format!("no constraint {id}"));
                -1
            }
            Some(_) => {
                if sk(h).set_constraint_num(id as u32, &n, v) {
                    0
                } else {
                    set_error(format!("{n} is not an argument a number can set"));
                    -1
                }
            }
        }
    })
}

/// Set a string argument by name (an arc tangency's end).
#[no_mangle]
pub unsafe extern "C" fn gcs_constraint_set_str(
    h: *mut Sketch,
    id: i32,
    name: *const u8,
    name_len: usize,
    v: *const u8,
    v_len: usize,
) -> i32 {
    guard(-1, move || {
        let n = as_str(name, name_len).to_string();
        let val = as_str(v, v_len).to_string();
        match sk(h).constraint_mut(id as u32) {
            None => {
                set_error(format!("no constraint {id}"));
                -1
            }
            Some(c) => {
                if c.set_str(&n, &val) {
                    0
                } else {
                    set_error(format!("{n} is not a string argument"));
                    -1
                }
            }
        }
    })
}

/// Write a dimension from text: a bare number is a constant, anything else an expression
/// (`w = 1`, `h = w * 2`), evaluated with the rest of the document's — see `gcs_core::expr`.
/// 0 when it was stored and computed; 1 when it was stored but could not be computed yet (a name
/// nothing defines), with why in `gcs_last_error()`; -1 when it was rejected (it does not parse,
/// or `name` is no dimension) and nothing changed.
#[no_mangle]
pub unsafe extern "C" fn gcs_constraint_set_dimension(
    h: *mut Sketch,
    id: i32,
    name: *const u8,
    name_len: usize,
    text: *const u8,
    text_len: usize,
) -> i32 {
    guard(-1, move || {
        let n = as_str(name, name_len).to_string();
        let t = as_str(text, text_len).to_string();
        match expr::set_dimension(sk(h), id as u32, &n, &t) {
            Ok(None) => 0,
            Ok(Some(why)) => {
                set_error(why);
                1
            }
            Err(e) => {
                set_error(e);
                -1
            }
        }
    })
}

/// Every dimension expression in the document, evaluated and in evaluation order: each with its
/// constraint id and attribute, its text, the name it defines, its value (degrees for an angle),
/// the names it reads and its error if it could not be computed.
#[no_mangle]
pub unsafe extern "C" fn gcs_exprs_json(h: *mut Sketch) -> *mut u8 {
    guard(std::ptr::null_mut(), move || out_json(report::exprs_json(sk(h))))
}

/// A drag target's (tx, ty) — the one mutation the hot path performs.
#[no_mangle]
pub unsafe extern "C" fn gcs_constraint_set_target(h: *mut Sketch, id: i32, tx: f64, ty: f64) {
    guard((), move || {
        match sk(h).constraint_mut(id as u32) {
            None => set_error(format!("no constraint {id}")),
            Some(c) => {
                let name = c.type_name();
                if !c.set_target(tx, ty) {
                    set_error(format!("{name} has no drag target"));
                }
            }
        }
    })
}

/// The global Param indices the kernel's columns refer to; returns how many.
#[no_mangle]
pub unsafe extern "C" fn gcs_constraint_params(h: *mut Sketch, id: i32, out: *mut i32) -> i32 {
    guard(-1, move || {
        let s = sk(h);
        let Some(c) = s.constraint(id as u32) else { return 0 };
        let ps = c.params(s);
        for (i, p) in ps.iter().enumerate() {
            *out.add(i) = *p as i32;
        }
        ps.len() as i32
    })
}

/// The current values of those params; returns how many.
#[no_mangle]
pub unsafe extern "C" fn gcs_constraint_local_values(
    h: *mut Sketch,
    id: i32,
    out: *mut f64,
) -> i32 {
    guard(-1, move || {
        let s = sk(h);
        let Some(c) = s.constraint(id as u32) else { return 0 };
        let v = c.local_values(s);
        for (i, x) in v.iter().enumerate() {
            *out.add(i) = *x;
        }
        v.len() as i32
    })
}

/// One row of the kernel at arbitrary local values: `n_res` residuals and the `n_res * n_par`
/// Jacobian.  Returns `n_par`.
#[no_mangle]
pub unsafe extern "C" fn gcs_constraint_eval(
    h: *mut Sketch,
    id: i32,
    v: *const f64,
    r_out: *mut f64,
    j_out: *mut f64,
) -> i32 {
    guard(-1, move || {
        let s = sk(h);
        let Some(c) = s.constraint(id as u32) else { return 0 };
        let k = kernels::kernel_by_id(c.kernel_id());
        let vals = std::slice::from_raw_parts(v, k.n_par);
        let r = c.residual(s, vals);
        let j = c.jacobian(s, vals);
        for (i, x) in r.iter().enumerate() {
            *r_out.add(i) = *x;
        }
        for (i, x) in j.iter().enumerate() {
            *j_out.add(i) = *x;
        }
        k.n_par as i32
    })
}

/// The first residual row of a constraint in a compiled system.
#[no_mangle]
pub unsafe extern "C" fn gcs_system_row_of(s: *mut System, id: i32) -> i32 {
    guard(-1, move || {
        (*s).row_of(id as u32).map_or(-1, |r| r as i32)
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_constraint_error(h: *mut Sketch, id: i32) -> f64 {
    guard(f64::NAN, move || {
        let s = sk(h);
        s.constraint(id as u32).map(|c| c.error(s)).unwrap_or(0.0)
    })
}

/// Whether two constraint records say exactly the same thing — same type, same entities in the
/// same roles, same values, up to the swap a commutative type allows.  The rule lives in the
/// core; both bindings ask here rather than each keeping a copy of it.
#[no_mangle]
pub unsafe extern "C" fn gcs_same_constraint(
    h: *mut Sketch,
    a_ptr: *const u8,
    a_len: usize,
    b_ptr: *const u8,
    b_len: usize,
) -> i32 {
    guard(-1, move || {
        let s = sk(h);
        let (av, bv) = (as_json(a_ptr, a_len), as_json(b_ptr, b_len));
        let (Ok(a), Ok(b)) = (
            report::constraint_from_json(s, &av),
            report::constraint_from_json(s, &bv),
        ) else {
            return -1;
        };
        gcs_core::constraints::same_constraint(&a, &b) as i32
    })
}

/// The id of a constraint already stating the same *relation* as the one described — the same
/// type on the same entities, whatever number it states — or -1.  What an *edit* of a dimension
/// would land on, for a caller that wants to offer one.
#[no_mangle]
pub unsafe extern "C" fn gcs_constraint_stating(
    h: *mut Sketch,
    ptr: *const u8,
    len: usize,
) -> i32 {
    guard(-1, move || {
        let s = sk(h);
        let v = as_json(ptr, len);
        let Ok(c) = report::constraint_from_json(s, &v) else { return -1 };
        s.constraints
            .iter()
            .filter(|k| !(k.intrinsic || k.soft))
            .find(|k| gcs_core::constraints::same_relation(k, &c))
            .map(|k| k.id as i32)
            .unwrap_or(-1)
    })
}

/// The id of an existing constraint that says exactly the same thing, or -1.
#[no_mangle]
pub unsafe extern "C" fn gcs_constraint_duplicate(
    h: *mut Sketch,
    ptr: *const u8,
    len: usize,
) -> i32 {
    guard(-1, move || {
        let s = sk(h);
        let v = as_json(ptr, len);
        let Ok(c) = report::constraint_from_json(s, &v) else { return -1 };
        s.constraints
            .iter()
            .find(|k| gcs_core::constraints::same_constraint(k, &c))
            .map(|k| k.id as i32)
            .unwrap_or(-1)
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_describe(h: *mut Sketch, id: i32) -> *mut u8 {
    guard(std::ptr::null_mut(), move || {
        let s = sk(h);
        out_str(s.constraint(id as u32).map(io::describe).unwrap_or_default())
    })
}

/// The dimension callouts for the whole sketch.  `unit` is the world length of one screen pixel;
/// the layout is screen-constant through it.
#[no_mangle]
pub unsafe extern "C" fn gcs_callouts_json(h: *mut Sketch, unit: f64) -> *mut u8 {
    guard(std::ptr::null_mut(), move || {
        out_json(report::callouts_json(sk(h), unit))
    })
}

/// The dimension whose callout the world point (x, y) lands on, within `tol_px` screen pixels of
/// it, or -1.
#[no_mangle]
pub unsafe extern "C" fn gcs_callout_pick(h: *mut Sketch, unit: f64, x: f64, y: f64,
                                          tol_px: f64) -> i32 {
    guard(-1, move || {
        callout::pick(sk(h), unit, (x, y), tol_px).map_or(-1, |id| id as i32)
    })
}

/// Take hold of dimension `id`'s callout at the world point (x, y): writes the two numbers the
/// caller hands back to `gcs_callout_drag` for the rest of the gesture, so the callout moves with
/// the pointer rather than jumping to it.  0 if the dimension has no callout to grab.
#[no_mangle]
pub unsafe extern "C" fn gcs_callout_grab(h: *mut Sketch, id: i32, unit: f64, x: f64, y: f64,
                                          out: *mut f64) -> i32 {
    guard(0, move || match callout::grab(sk(h), unit, id as u32, (x, y)) {
        Some((a, b)) => {
            let o = std::slice::from_raw_parts_mut(out, 2);
            o[0] = a;
            o[1] = b;
            1
        }
        None => 0,
    })
}

/// Move dimension `id`'s callout so the point it was grabbed at follows the pointer to (x, y).
#[no_mangle]
pub unsafe extern "C" fn gcs_callout_drag(h: *mut Sketch, id: i32, x: f64, y: f64,
                                          gu: f64, gv: f64) -> i32 {
    guard(0, move || callout::drag(sk(h), id as u32, (x, y), (gu, gv)) as i32)
}

/// Which of the three dimensions between two points a callout dropped at (px, py) states: a
/// length, the run between them or the rise.  The answer is an index into the registry's type
/// list, so a front end names it the way it names every other constraint type; -1 if the core
/// has no such type, which cannot happen.
#[no_mangle]
pub unsafe extern "C" fn gcs_dimension_pair_kind(
    ax: f64,
    ay: f64,
    bx: f64,
    by: f64,
    px: f64,
    py: f64,
) -> i32 {
    guard(-1, move || {
        let k = callout::pair_dimension((ax, ay), (bx, by), (px, py));
        gcs_core::constraints::ALL_KINDS.iter().position(|&e| e == k).map_or(-1, |i| i as i32)
    })
}

/// Put dimension `id`'s callout back wherever the layout would have put it; 1 if it had been
/// moved at all.
#[no_mangle]
pub unsafe extern "C" fn gcs_callout_reset(h: *mut Sketch, id: i32) -> i32 {
    guard(0, move || callout::reset(sk(h), id as u32) as i32)
}

#[no_mangle]
pub extern "C" fn gcs_fmt_g(v: f64, sig: i32) -> *mut u8 {
    guard(std::ptr::null_mut(), move || {
        out_str(json::fmt_g(v, sig.max(1) as usize))
    })
}

/* -- branches (document state) --------------------------------------------- */

#[no_mangle]
pub unsafe extern "C" fn gcs_branches_json(h: *mut Sketch) -> *mut u8 {
    guard(std::ptr::null_mut(), move || {
        let s = sk(h);
        out_json(Json::Obj(
            s.branches.iter().map(|(k, &v)| (k.clone(), Json::Int(v as i64))).collect(),
        ))
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_branches_set_json(h: *mut Sketch, ptr: *const u8, len: usize) {
    guard((), move || {
        let s = sk(h);
        s.branches.clear();
        if let Json::Obj(kv) = as_json(ptr, len) {
            for (k, v) in kv {
                s.branches.insert(k, v.as_i64() as i32);
            }
        }
    })
}

/* -- io -------------------------------------------------------------------- */

/// Copy of the sketch without `entities` (`[[kind, idx], ...]`) and `constraints` (`[id, ...]`).
#[no_mangle]
pub unsafe extern "C" fn gcs_without(
    h: *mut Sketch,
    ents: *const u8,
    ents_len: usize,
    cids: *const u8,
    cids_len: usize,
) -> *mut Sketch {
    guard(std::ptr::null_mut(), move || {
        let s = sk(h);
        let cv = as_json(cids, cids_len);
        let constraints: Vec<u32> = cv.arr().iter().map(|v| v.as_i64() as u32).collect();
        Box::into_raw(Box::new(io::without(s, &ent_list(ents, ents_len), &constraints)))
    })
}

/// A fresh sketch holding just `entities` (`[[kind, idx], ...]`), the points that define them and
/// the constraints all of whose entities came along — what a copy puts on the clipboard.
#[no_mangle]
pub unsafe extern "C" fn gcs_copy(h: *mut Sketch, ents: *const u8, ents_len: usize) -> *mut Sketch {
    guard(std::ptr::null_mut(), move || {
        Box::into_raw(Box::new(io::copy(sk(h), &ent_list(ents, ents_len))))
    })
}

/// Add everything in `clip` to the sketch, moved by (dx, dy).  Returns what it made, as
/// `[[kind, idx], ...]` in the order the clipboard held them.
#[no_mangle]
pub unsafe extern "C" fn gcs_paste(h: *mut Sketch, clip: *mut Sketch, dx: f64,
                                   dy: f64) -> *mut u8 {
    guard(std::ptr::null_mut(), move || {
        let made = io::paste(sk(h), &*clip, dx, dy);
        out_json(Json::Arr(made.into_iter().map(report::ent_json).collect()))
    })
}

/* -- examples -------------------------------------------------------------- */

/// The source of a case written as a document, or null for one that is a function.
#[no_mangle]
pub unsafe extern "C" fn gcs_example_source(name: *const u8, len: usize) -> *mut u8 {
    guard(std::ptr::null_mut(), move || match examples::source(&as_str(name, len)) {
        Some(src) => out_str(src.to_string()),
        None => std::ptr::null_mut(),
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_example(name: *const u8, len: usize) -> *mut Sketch {
    guard(std::ptr::null_mut(), move || {
        match examples::case(as_str(name, len)) {
            Some(s) => Box::into_raw(Box::new(s)),
            None => {
                set_error(format!("unknown example: {}", as_str(name, len)));
                std::ptr::null_mut()
            }
        }
    })
}

#[no_mangle]
pub extern "C" fn gcs_cases_json() -> *mut u8 {
    guard(std::ptr::null_mut(), move || {
        out_json(Json::Arr(
            examples::CASES
                .iter()
                .map(|(label, key, desc)| {
                    json::object([
                        ("label", (*label).into()),
                        ("key", (*key).into()),
                        ("description", (*desc).into()),
                    ])
                })
                .collect(),
        ))
    })
}

/* -- system ---------------------------------------------------------------- */

#[no_mangle]
pub unsafe extern "C" fn gcs_system_new(h: *mut Sketch) -> *mut System {
    guard(std::ptr::null_mut(), move || {
        Box::into_raw(Box::new(System::new(sk(h))))
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_system_free(s: *mut System) {
    guard((), move || {
        if !s.is_null() {
            drop(Box::from_raw(s));
        }
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_system_n_res(s: *mut System) -> i32 {
    guard(-1, move || {
        (*s).n_res as i32
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_system_n_free(s: *mut System) -> i32 {
    guard(-1, move || {
        (*s).n_free as i32
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_system_nnz(s: *mut System) -> i32 {
    guard(-1, move || {
        (*s).nnz as i32
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_system_scale(s: *mut System) -> f64 {
    guard(f64::NAN, move || {
        (*s).scale
    })
}

/// One flag per residual row: rows that must be satisfied.
#[no_mangle]
pub unsafe extern "C" fn gcs_system_hard(s: *mut System, out: *mut u8) {
    guard((), move || {
        write_map(out, &(*s).hard, |&b| b as u8);
    })
}

/// The free values of the current sketch geometry (also refreshes the core's copy of x).
#[no_mangle]
pub unsafe extern "C" fn gcs_system_z0(s: *mut System, h: *mut Sketch, out: *mut f64) {
    guard((), move || {
        let z = (*s).z0(sk(h));
        write(out, &z);
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_system_residuals(s: *mut System, z: *const f64, out: *mut f64) {
    guard((), move || {
        let sys = &mut *s;
        let zz = std::slice::from_raw_parts(z, sys.n_free);
        let r = sys.residuals(zz);
        write(out, &r);
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_system_jacobian_dense(s: *mut System, z: *const f64, out: *mut f64) {
    guard((), move || {
        let sys = &mut *s;
        let zz = std::slice::from_raw_parts(z, sys.n_free);
        let j = sys.jacobian_dense(zz);
        write(out, &j.data);
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_system_csr_structure(
    s: *mut System,
    indptr: *mut i32,
    indices: *mut i32,
) {
    guard((), move || {
        let sys = &*s;
        write(indptr, &sys.csr_indptr);
        write(indices, &sys.csr_indices);
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_system_csr_data(s: *mut System, z: *const f64, out: *mut f64) {
    guard((), move || {
        let sys = &mut *s;
        let zz = std::slice::from_raw_parts(z, sys.n_free);
        let d = sys.compute_csr(zz).to_vec();
        write(out, &d);
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_system_max_hard_residual(
    s: *mut System,
    h: *mut Sketch,
) -> f64 {
    guard(f64::NAN, move || {
        let sys = &mut *s;
        let z = sys.z0(sk(h));
        sys.max_hard_residual(&z)
    })
}

/// max |residual| per constraint, in block order; `ids` receives the matching constraint ids.
#[no_mangle]
pub unsafe extern "C" fn gcs_system_constraint_errors(
    s: *mut System,
    h: *mut Sketch,
    ids: *mut i32,
    out: *mut f64,
    cap: i32,
) -> i32 {
    guard(-1, move || {
        let sys = &mut *s;
        let z = sys.z0(sk(h));
        let e = sys.constraint_errors(&z);
        // one entry per *compiled* constraint, which is not the live sketch's count once the
        // sketch has been edited: never write past what the caller sized
        let n = e.len().min(cap.max(0) as usize);
        if n < e.len() {
            set_error(format!("constraint_errors: {} entries need {} slots", e.len(), e.len()));
        }
        write(out, &e[..n]);
        write_map(ids, &sys.cids[..n], |&c| c as i32);
        n as i32
    })
}

/// max |residual| / that row's units over the hard rows — dimensionless, so one threshold judges
/// every kernel.  This, not `gcs_system_max_hard_residual`, is what "solved" means.
#[no_mangle]
pub unsafe extern "C" fn gcs_system_max_relative_residual(s: *mut System, h: *mut Sketch) -> f64 {
    guard(f64::NAN, move || {
        let sys = &mut *s;
        let z = sys.z0(sk(h));
        sys.max_relative_residual(&z)
    })
}

/// How many constraints the plan was compiled from — the size `gcs_system_constraint_errors`
/// needs, which is the live sketch's count only until the sketch is edited.
#[no_mangle]
pub unsafe extern "C" fn gcs_system_n_constraints(s: *mut System) -> i32 {
    guard(-1, move || (*s).n_constraints() as i32)
}

/// Numerical rank of the Jacobian at the sketch's current values; `tol` is absolute and
/// dimensionless (`gcs_rank_tol()` is the diagnosis's own).
#[no_mangle]
pub unsafe extern "C" fn gcs_system_rank(
    s: *mut System,
    h: *mut Sketch,
    tol: f64,
    hard_only: i32,
) -> i32 {
    guard(-1, move || {
        let sys = &mut *s;
        let z = sys.z0(sk(h));
        sys.rank(&z, tol, hard_only != 0) as i32
    })
}

/// The tolerance a rank is judged at — `system::RANK_TOL`, so a binding's default is the
/// core's and not a second number.
#[no_mangle]
pub unsafe extern "C" fn gcs_rank_tol() -> f64 {
    gcs_core::system::RANK_TOL
}

/// The hard rows of the Jacobian at the sketch's current values with their units divided out
/// — the matrix every rank and null space in the core is judged on (`System::conditioned`).
/// Row-major, one row per hard row in `gcs_system_structure_json`'s order, `n_free` columns;
/// returns the row count.  A getter, so a test can check a verdict against the matrix the
/// core actually judged.
#[no_mangle]
pub unsafe extern "C" fn gcs_system_conditioned(s: *mut System, h: *mut Sketch, out: *mut f64) -> i32 {
    guard(-1, move || {
        let sys = &mut *s;
        let z = sys.z0(sk(h));
        let c = sys.conditioned(&z);
        write(out, &c.as_mat().data);
        c.rows() as i32
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_system_update_consts(s: *mut System, h: *mut Sketch, id: i32) {
    guard((), move || {
        (*s).update_consts(sk(h), id as u32);
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_system_refresh_consts(s: *mut System, h: *mut Sketch) {
    guard((), move || {
        (*s).refresh_consts(sk(h));
    })
}

/// The structural Jacobian as a bipartite graph, plus row → constraint id.
#[no_mangle]
pub unsafe extern "C" fn gcs_system_structure_json(s: *mut System) -> *mut u8 {
    guard(std::ptr::null_mut(), move || {
        let (adj, row_c) = (*s).structure();
        out_json(json::object([
            (
                "adj",
                Json::Arr(
                    adj.iter()
                        .map(|r| Json::Arr(r.iter().map(|&c| Json::Int(c as i64)).collect()))
                        .collect(),
                ),
            ),
            ("rowC", Json::Arr(row_c.iter().map(|&c| Json::Int(c as i64)).collect())),
            ("nFree", Json::Int((*s).n_free as i64)),
        ]))
    })
}

/// Free-parameter index → sketch Param index.
#[no_mangle]
pub unsafe extern "C" fn gcs_system_free_indices(s: *mut System, out: *mut i32) {
    guard((), move || {
        write(out, &(*s).free);
    })
}

/* -- solving --------------------------------------------------------------- */

/// `[success, status, residualNorm, maxResidual, nfev, njev, iterations, rank]` (rank -1 = none).
fn write_result(r: &SolveResult, out: *mut f64) {
    let v = [
        r.success as i32 as f64,
        r.status as f64,
        r.residual_norm,
        r.max_residual,
        r.nfev as f64,
        r.njev as f64,
        r.iterations as f64,
        r.rank.map(|x| x as f64).unwrap_or(-1.0),
    ];
    unsafe {
        for (i, x) in v.iter().enumerate() {
            *out.add(i) = *x;
        }
    }
}

fn opts_from(method: i32, tol: f64, max_iter: i32, max_nfev: i32, dense: i32, writeback: i32) -> SolveOpts {
    SolveOpts {
        method: if method == 1 { Method::Lm } else { Method::DogLeg },
        tol,
        max_nfev,
        writeback: writeback != 0,
        max_iter,
        dense: if dense < 0 { None } else { Some(dense != 0) },
        ..SolveOpts::default()
    }
}

/// Solve on an already-compiled system.  Returns the status message.
#[no_mangle]
#[allow(clippy::too_many_arguments)]
pub unsafe extern "C" fn gcs_system_solve(
    s: *mut System,
    h: *mut Sketch,
    method: i32,
    tol: f64,
    max_iter: i32,
    max_nfev: i32,
    dense: i32,
    writeback: i32,
    out: *mut f64,
) -> *mut u8 {
    guard(std::ptr::null_mut(), move || {
        let r = (*s).solve(sk(h), opts_from(method, tol, max_iter, max_nfev, dense, writeback));
        write_result(&r, out);
        out_str(r.message)
    })
}

/// One-shot: compile and solve.
#[no_mangle]
#[allow(clippy::too_many_arguments)]
pub unsafe extern "C" fn gcs_solve(
    h: *mut Sketch,
    method: i32,
    tol: f64,
    max_iter: i32,
    max_nfev: i32,
    dense: i32,
    out: *mut f64,
) -> *mut u8 {
    guard(std::ptr::null_mut(), move || {
        let r = gcs_core::solve::solve(sk(h), opts_from(method, tol, max_iter, max_nfev, dense, 1));
        write_result(&r, out);
        out_str(r.message)
    })
}

#[no_mangle]
pub extern "C" fn gcs_status_message(status: i32) -> *mut u8 {
    guard(std::ptr::null_mut(), move || {
        out_str(newton::status_message(status).to_string())
    })
}

/* -- dense linear algebra (exposed so the bindings can be checked) ---------- */

#[no_mangle]
pub unsafe extern "C" fn gcs_min_norm_lstsq(
    m: i32,
    n: i32,
    nrhs: i32,
    a: *const f64,
    b: *const f64,
    rcond: f64,
    x: *mut f64,
) -> i32 {
    guard(-1, move || {
        let (m, n, nrhs) = (m as usize, n as usize, nrhs as usize);
        let am = linalg::Mat::from_vec(m, n, std::slice::from_raw_parts(a, m * n).to_vec());
        let bm = linalg::Mat::from_vec(m, nrhs, std::slice::from_raw_parts(b, m * nrhs).to_vec());
        let (xm, rank) = linalg::min_norm_lstsq(&am, &bm, rcond);
        write(x, &xm.data);
        rank as i32
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_rrqr(
    m: i32,
    n: i32,
    a: *const f64,
    rcond: f64,
    piv: *mut i32,
) -> i32 {
    guard(-1, move || {
        let (m, n) = (m as usize, n as usize);
        let am = linalg::Mat::from_vec(m, n, std::slice::from_raw_parts(a, m * n).to_vec());
        let (rank, p) = linalg::rrqr(&am, rcond);
        if !piv.is_null() {
            write(piv, &p);
        }
        rank as i32
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_svd(
    m: i32,
    n: i32,
    a: *const f64,
    u: *mut f64,
    s: *mut f64,
    vt: *mut f64,
) -> i32 {
    guard(-1, move || {
        let (mm, nn) = (m as usize, n as usize);
        let am = linalg::Mat::from_vec(mm, nn, std::slice::from_raw_parts(a, mm * nn).to_vec());
        let d = linalg::svd(&am, !u.is_null());
        write(s, &d.s);
        write(vt, &d.vt.data);
        if !u.is_null() {
            write(u, &d.u.data);
        }
        if d.converged {
            0
        } else {
            set_error("svd: the QR sweeps did not converge".to_string());
            -1
        }
    })
}

/// Numerical rank and null space from one SVD; `n_out` is written with `n - rank` columns.
#[no_mangle]
pub unsafe extern "C" fn gcs_rank_nullspace(
    m: i32,
    n: i32,
    a: *const f64,
    rcond: f64,
    n_out: *mut f64,
    s_out: *mut f64,
) -> i32 {
    guard(-1, move || {
        let (mm, nn) = (m as usize, n as usize);
        let am = linalg::Mat::from_vec(mm, nn, std::slice::from_raw_parts(a, mm * nn).to_vec());
        let rn = linalg::rank_and_nullspace(&am, rcond);
        write(n_out, &rn.null().data);
        if !s_out.is_null() {
            write(s_out, &rn.s);
        }
        if !rn.converged {
            set_error("rank_nullspace: the SVD did not converge".to_string());
            return -1;
        }
        rn.rank as i32
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_lu_solve(n: i32, a: *mut f64, b: *mut f64) -> i32 {
    guard(-1, move || {
        let n = n as usize;
        let aa = std::slice::from_raw_parts_mut(a, n * n);
        let bb = std::slice::from_raw_parts_mut(b, n);
        linalg::lu_solve(n, aa, bb) as i32 - 1
    })
}

/* -- finite-difference check ----------------------------------------------- */

/// Max |analytic − FD| over the whole sketch Jacobian; negative on mismatch.
#[no_mangle]
pub unsafe extern "C" fn gcs_check_sketch(h: *mut Sketch, rtol: f64, atol: f64) -> f64 {
    guard(f64::NAN, move || {
        match fdcheck::check_sketch(sk(h), rtol, atol) {
            Ok(v) => v,
            Err(e) => {
                set_error(e);
                -1.0
            }
        }
    })
}

/// Max |analytic − FD| for one constraint; negative on mismatch.
#[no_mangle]
pub unsafe extern "C" fn gcs_check_constraint(
    h: *mut Sketch,
    id: i32,
    rtol: f64,
    atol: f64,
) -> f64 {
    guard(f64::NAN, move || {
        let s = sk(h);
        let Some(c) = s.constraint(id as u32) else { return -1.0 };
        match fdcheck::check_constraint(s, c, rtol, atol) {
            Ok(v) => v,
            Err(e) => {
                set_error(e);
                -1.0
            }
        }
    })
}

/* -- diagnosis ------------------------------------------------------------- */

fn diagnose_options(v: &Json) -> DiagnoseOptions {
    let mut o = DiagnoseOptions::default();
    if let Some(x) = v.get("numeric") {
        if !matches!(x, Json::Null) {
            o.numeric = Some(x.as_bool());
        }
    }
    if let Some(x) = v.get("conflicts") {
        if !matches!(x, Json::Null) {
            o.conflicts = Some(x.as_bool());
        }
    }
    if let Some(x) = v.get("witness") {
        o.witness = x.as_bool();
    }
    if let Some(x) = v.get("tol") {
        o.tol = x.as_f64();
    }
    if let Some(x) = v.get("numericMax") {
        o.numeric_max = x.as_i64() as usize;
    }
    o
}

#[no_mangle]
pub unsafe extern "C" fn gcs_diagnose_json(
    h: *mut Sketch,
    opts: *const u8,
    opts_len: usize,
) -> *mut u8 {
    guard(std::ptr::null_mut(), move || {
        let s = sk(h);
        let o = diagnose_options(&as_json(opts, opts_len));
        let d = diagnose::diagnose(s, o);
        out_json(report::diagnosis_json(s, &d))
    })
}

/// Diagnose reusing an already-compiled system (what the app does after a solve).
#[no_mangle]
pub unsafe extern "C" fn gcs_diagnose_with_json(
    h: *mut Sketch,
    sys: *mut System,
    opts: *const u8,
    opts_len: usize,
) -> *mut u8 {
    guard(std::ptr::null_mut(), move || {
        let s = sk(h);
        let o = diagnose_options(&as_json(opts, opts_len));
        let d = diagnose::diagnose_with(s, &mut *sys, o);
        out_json(report::diagnosis_json(s, &d))
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_minimal_conflict_set_json(
    h: *mut Sketch,
    cands: *const u8,
    cands_len: usize,
    tol: f64,
) -> *mut u8 {
    guard(std::ptr::null_mut(), move || {
        let s = sk(h);
        let cv = as_json(cands, cands_len);
        let list: Vec<u32> = cv.arr().iter().map(|v| v.as_i64() as u32).collect();
        let out = diagnose::minimal_conflict_set(
            s,
            if matches!(cv, Json::Null) { None } else { Some(&list) },
            tol,
            Method::DogLeg,
            60,
        );
        out_json(Json::Arr(out.iter().map(|&c| Json::Int(c as i64)).collect()))
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_violated_json(h: *mut Sketch, tol: f64) -> *mut u8 {
    guard(std::ptr::null_mut(), move || {
        let s = sk(h);
        let mut sys = System::new(s);
        let v = diagnose::violated_constraints(s, &mut sys, tol);
        out_json(Json::Arr(v.iter().map(|&c| Json::Int(c as i64)).collect()))
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_distance_rigidity_json(h: *mut Sketch) -> *mut u8 {
    guard(std::ptr::null_mut(), move || {
        let (clusters, redundant) = diagnose::distance_rigidity(sk(h));
        out_json(json::object([
            (
                "clusters",
                Json::Arr(
                    clusters
                        .iter()
                        .map(|c| Json::Arr(c.iter().map(|&p| Json::Int(p as i64)).collect()))
                        .collect(),
                ),
            ),
            ("redundant", Json::Arr(redundant.iter().map(|&c| Json::Int(c as i64)).collect())),
        ]))
    })
}

/* -- pure graph algorithms -------------------------------------------------- */

fn adj_from(v: &Json) -> Vec<Vec<usize>> {
    v.arr()
        .iter()
        .map(|r| r.arr().iter().map(|c| c.as_i64() as usize).collect())
        .collect()
}

fn i_arr(v: &[i32]) -> Json {
    Json::Arr(v.iter().map(|&x| Json::Int(x as i64)).collect())
}

fn u_arr(v: &[usize]) -> Json {
    Json::Arr(v.iter().map(|&x| Json::Int(x as i64)).collect())
}

/// Maximum bipartite matching: `{mateL, mateR}` with -1 for unmatched.
#[no_mangle]
pub unsafe extern "C" fn gcs_hopcroft_karp_json(
    adj: *const u8,
    adj_len: usize,
    n_right: i32,
) -> *mut u8 {
    guard(std::ptr::null_mut(), move || {
        let a = adj_from(&as_json(adj, adj_len));
        let (l, r) = gcs_core::graph::hopcroft_karp(&a, n_right as usize);
        out_json(json::object([("mateL", i_arr(&l)), ("mateR", i_arr(&r))]))
    })
}

/// The coarse Dulmage–Mendelsohn decomposition.
#[no_mangle]
pub unsafe extern "C" fn gcs_dulmage_mendelsohn_json(
    adj: *const u8,
    adj_len: usize,
    n_cols: i32,
) -> *mut u8 {
    guard(std::ptr::null_mut(), move || {
        let a = adj_from(&as_json(adj, adj_len));
        let d = gcs_core::graph::dulmage_mendelsohn(&a, n_cols as usize);
        out_json(json::object([
            ("mateRow", i_arr(&d.mate_row)),
            ("mateCol", i_arr(&d.mate_col)),
            ("overRows", u_arr(&d.over_rows)),
            ("overCols", u_arr(&d.over_cols)),
            ("underRows", u_arr(&d.under_rows)),
            ("underCols", u_arr(&d.under_cols)),
            ("wellRows", u_arr(&d.well_rows)),
            ("wellCols", u_arr(&d.well_cols)),
            ("rank", Json::Int(d.rank as i64)),
            ("nRedundant", Json::Int(d.n_redundant)),
            ("nFree", Json::Int(d.n_free)),
        ]))
    })
}

/// The (2,3) pebble game on `n` vertices with `[[u, v], ...]` edges.
#[no_mangle]
pub unsafe extern "C" fn gcs_pebble_game_json(
    n: i32,
    edges: *const u8,
    edges_len: usize,
) -> *mut u8 {
    guard(std::ptr::null_mut(), move || {
        let e: Vec<(usize, usize)> = as_json(edges, edges_len)
            .arr()
            .iter()
            .map(|p| {
                let a = p.arr();
                (a[0].as_i64() as usize, a[1].as_i64() as usize)
            })
            .collect();
        let r = gcs_core::graph::pebble_game(n as usize, &e);
        out_json(json::object([
            ("independent", u_arr(&r.independent)),
            ("redundant", u_arr(&r.redundant)),
            ("components", Json::Arr(r.components.iter().map(|c| u_arr(c)).collect())),
            ("dof", Json::Int(r.dof as i64)),
            ("isRigid", Json::Bool(r.is_rigid())),
        ]))
    })
}

/// Connected components of a bipartite graph.
#[no_mangle]
pub unsafe extern "C" fn gcs_bipartite_components_json(
    adj: *const u8,
    adj_len: usize,
    n_cols: i32,
) -> *mut u8 {
    guard(std::ptr::null_mut(), move || {
        let a = adj_from(&as_json(adj, adj_len));
        let c = gcs_core::graph::bipartite_components(&a, n_cols as usize);
        out_json(json::object([
            ("compRow", u_arr(&c.comp_row)),
            ("compCol", u_arr(&c.comp_col)),
            ("count", Json::Int(c.count as i64)),
        ]))
    })
}

/// A random Laman graph's edges by Henneberg construction — the property-test generator.
#[no_mangle]
pub extern "C" fn gcs_henneberg_edges_json(n: i32, seed: u32) -> *mut u8 {
    guard(std::ptr::null_mut(), move || {
        let mut rng = gcs_core::rng::Rng::new(seed);
        let e = gcs_core::fixtures::henneberg_edges(n.max(2) as usize, &mut rng);
        out_json(Json::Arr(
            e.iter()
                .map(|&(a, b)| Json::Arr(vec![Json::Int(a as i64), Json::Int(b as i64)]))
                .collect(),
        ))
    })
}

/* -- witness --------------------------------------------------------------- */

#[no_mangle]
pub unsafe extern "C" fn gcs_witness_json(h: *mut Sketch, seed: u32) -> *mut u8 {
    guard(std::ptr::null_mut(), move || {
        let s = sk(h);
        let w = witness::analyze(s, None, seed);
        out_json(report::witness_json(s, &w))
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_make_witness(h: *mut Sketch, seed: u32, out: *mut f64) {
    guard((), move || {
        let x = witness::make_witness(sk(h), seed, 0.05, 1e-8);
        write(out, &x);
    })
}

/* -- decomposition --------------------------------------------------------- */

#[no_mangle]
pub unsafe extern "C" fn gcs_graph_json(h: *mut Sketch) -> *mut u8 {
    guard(std::ptr::null_mut(), move || {
        out_json(report::graph_json(&cgraph::build(sk(h))))
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_plan_solver_new(h: *mut Sketch, sticky: i32) -> *mut PlanSolver {
    guard(std::ptr::null_mut(), move || {
        Box::into_raw(Box::new(PlanSolver::new(sk(h), sticky != 0)))
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_plan_solver_free(p: *mut PlanSolver) {
    guard((), move || {
        if !p.is_null() {
            drop(Box::from_raw(p));
        }
    })
}

/// The verification System the plan solver owns — borrowed, never freed by the caller.
#[no_mangle]
pub unsafe extern "C" fn gcs_plan_solver_system(p: *mut PlanSolver) -> *mut System {
    guard(std::ptr::null_mut(), move || {
        &mut (*p).system as *mut System
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_plan_solver_plan_json(p: *mut PlanSolver) -> *mut u8 {
    guard(std::ptr::null_mut(), move || {
        out_json(report::plan_json(&(*p).plan))
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_plan_solver_graph_json(p: *mut PlanSolver) -> *mut u8 {
    guard(std::ptr::null_mut(), move || {
        out_json(report::graph_json(&(*p).plan.graph))
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_plan_solver_solve(
    p: *mut PlanSolver,
    h: *mut Sketch,
    tol: f64,
    fallback: i32,
    method: i32,
) -> *mut u8 {
    guard(std::ptr::null_mut(), move || {
        let m = if method == 1 { Method::Lm } else { Method::DogLeg };
        let r = (*p).solve(sk(h), tol, fallback != 0, m);
        out_json(report::plan_result_json(&r))
    })
}

/// Flip the closed-form constructions that place a point element; returns how many.
#[no_mangle]
pub unsafe extern "C" fn gcs_plan_solver_flip(
    p: *mut PlanSolver,
    h: *mut Sketch,
    point: i32,
) -> i32 {
    guard(-1, move || {
        let s = sk(h);
        let ps = &mut *p;
        let cls = ps.plan.graph.point_of[point as usize];
        ps.flip(s, El::p(cls)) as i32
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_plan_solver_sticky(p: *mut PlanSolver, sticky: i32) {
    guard((), move || {
        (*p).plan.sticky_branches = sticky != 0;
    })
}

/// Replay the plan on the current geometry (no solving, no fallback).
#[no_mangle]
pub unsafe extern "C" fn gcs_plan_solver_execute(p: *mut PlanSolver, h: *mut Sketch) {
    guard((), move || {
        decompose::execute(&mut (*p).plan, sk(h), None);
    })
}

/// The point element index of a sketch point (its coincidence class), or -1.
#[no_mangle]
pub unsafe extern "C" fn gcs_plan_solver_point_element(p: *mut PlanSolver, point: i32) -> i32 {
    guard(-1, move || {
        let ps = &*p;
        ps.plan.graph.point_of.get(point as usize).map(|&c| c as i32).unwrap_or(-1)
    })
}

/// The merges that place a point: closed-form ones where it is the constructed apex, else the
/// numeric merges that share it.  Writes step indices; returns how many.
#[no_mangle]
pub unsafe extern "C" fn gcs_plan_steps_placing(
    p: *mut PlanSolver,
    point: i32,
    out: *mut i32,
) -> i32 {
    guard(-1, move || {
        let ps = &*p;
        let Some(&cls) = ps.plan.graph.point_of.get(point as usize) else { return 0 };
        let idxs = ps.plan.steps_placing(El::p(cls));
        write_map(out, &idxs, |&s| s as i32);
        idxs.len() as i32
    })
}

/* -- homotopy -------------------------------------------------------------- */

#[no_mangle]
pub unsafe extern "C" fn gcs_enumerate_step_json(
    p: *mut PlanSolver,
    h: *mut Sketch,
    step: i32,
    locate_point: i32,
    seed: u32,
    max_paths: i32,
) -> *mut u8 {
    guard(std::ptr::null_mut(), move || {
        let s = sk(h);
        let ps = &mut *p;
        let locate = if locate_point < 0 {
            None
        } else {
            ps.plan.graph.point_of.get(locate_point as usize).map(|&c| El::p(c))
        };
        let alts = homotopy::enumerate_step(
            &mut ps.plan,
            s,
            step as usize,
            EnumerateOptions {
                locate,
                seed,
                max_paths: max_paths.max(1) as usize,
                ..Default::default()
            },
        );
        out_json(report::alternatives_json(&alts))
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_apply_alternative(
    p: *mut PlanSolver,
    h: *mut Sketch,
    step: i32,
    alt: *const u8,
    alt_len: usize,
) {
    guard((), move || {
        let a = report::alternative_from_json(&as_json(alt, alt_len));
        homotopy::apply_alternative(&mut (*p).plan, sk(h), step as usize, &a);
    })
}

/* -- drags ----------------------------------------------------------------- */

unsafe fn guards_from(ptr: *const i32, n: i32) -> Vec<Triangle> {
    if ptr.is_null() || n <= 0 {
        return Vec::new();
    }
    let v = std::slice::from_raw_parts(ptr, 3 * n as usize);
    (0..n as usize)
        .map(|i| (v[3 * i] as usize, v[3 * i + 1] as usize, v[3 * i + 2] as usize))
        .collect()
}

#[no_mangle]
#[allow(clippy::too_many_arguments)]
pub unsafe extern "C" fn gcs_drag_new(
    h: *mut Sketch,
    point: i32,
    x: f64,
    y: f64,
    method: i32,
    weight: f64,
    guards: *const i32,
    n_guards: i32,
    max_step_rel: f64,
) -> *mut Drag {
    guard(std::ptr::null_mut(), move || {
        let m = if method == 1 { Method::Lm } else { Method::DogLeg };
        Box::into_raw(Box::new(Drag::new(
            sk(h),
            point as usize,
            x,
            y,
            m,
            weight,
            guards_from(guards, n_guards),
            max_step_rel,
        )))
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_drag_move(
    d: *mut Drag,
    h: *mut Sketch,
    x: f64,
    y: f64,
    out: *mut f64,
) -> *mut u8 {
    guard(std::ptr::null_mut(), move || {
        let r = (*d).move_to(sk(h), x, y);
        write_result(&r, out);
        out_str(r.message)
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_drag_end(d: *mut Drag, h: *mut Sketch) {
    guard((), move || {
        (*d).end(sk(h));
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_drag_free(d: *mut Drag) {
    guard((), move || {
        if !d.is_null() {
            drop(Box::from_raw(d));
        }
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_drag_flips(d: *mut Drag) -> i32 {
    guard(-1, move || {
        (*d).flips.len() as i32
    })
}

/// The triangles whose orientation flipped, 3 point indices each; returns how many.
#[no_mangle]
pub unsafe extern "C" fn gcs_drag_flip_list(d: *mut Drag, out: *mut i32) -> i32 {
    guard(-1, move || {
        let f = &(*d).flips;
        for (i, t) in f.iter().enumerate() {
            *out.add(3 * i) = t.0 as i32;
            *out.add(3 * i + 1) = t.1 as i32;
            *out.add(3 * i + 2) = t.2 as i32;
        }
        f.len() as i32
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_radius_drag_new(
    h: *mut Sketch,
    kind: i32,
    idx: i32,
    r: f64,
    method: i32,
) -> *mut RadiusDrag {
    guard(std::ptr::null_mut(), move || {
        let m = if method == 1 { Method::Lm } else { Method::DogLeg };
        Box::into_raw(Box::new(RadiusDrag::new(sk(h), ent(kind, idx), r, m)))
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_radius_drag_move(
    d: *mut RadiusDrag,
    h: *mut Sketch,
    r: f64,
    out: *mut f64,
) -> *mut u8 {
    guard(std::ptr::null_mut(), move || {
        let res = (*d).move_to(sk(h), r);
        write_result(&res, out);
        out_str(res.message)
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_radius_drag_end(d: *mut RadiusDrag, h: *mut Sketch) {
    guard((), move || {
        (*d).end(sk(h));
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_radius_drag_free(d: *mut RadiusDrag) {
    guard((), move || {
        if !d.is_null() {
            drop(Box::from_raw(d));
        }
    })
}

/// A `PlanDrag` and the plan it was made on, if any: the caller keeps that plan alive for as long
/// as the drag, and it comes back to the core with every call.
pub struct PlanDragH {
    d: PlanDrag,
    ps: *mut PlanSolver,
}

/// The plan the drag was made on, handed back to the core with every call.
unsafe fn plan_of<'a>(h: *mut PlanDragH) -> Option<&'a Plan> {
    (*h).ps.as_ref().map(|p| &p.plan)
}

/// `ps` may be null: the drag then makes a plan of its own over the dragged point's part.
#[no_mangle]
#[allow(clippy::too_many_arguments)]
pub unsafe extern "C" fn gcs_plan_drag_new(
    h: *mut Sketch,
    ps: *mut PlanSolver,
    point: i32,
    x: f64,
    y: f64,
    guards: *const i32,
    n_guards: i32,
    max_step_rel: f64,
) -> *mut PlanDragH {
    guard(std::ptr::null_mut(), move || {
        let g = if n_guards < 0 { None } else { Some(guards_from(guards, n_guards)) };
        let d = if ps.is_null() {
            PlanDrag::new(sk(h), point as usize, x, y, g, max_step_rel)
        } else {
            PlanDrag::on(sk(h), &mut *ps, point as usize, x, y, g, max_step_rel)
        };
        Box::into_raw(Box::new(PlanDragH { d, ps }))
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_plan_drag_move(
    d: *mut PlanDragH,
    h: *mut Sketch,
    x: f64,
    y: f64,
    out: *mut f64,
) -> *mut u8 {
    guard(std::ptr::null_mut(), move || {
        let plan = plan_of(d);
        let r = (*d).d.move_to(sk(h), plan, x, y);
        write_result(&r, out);
        out_str(r.message)
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_plan_drag_usable(d: *mut PlanDragH) -> i32 {
    guard(-1, move || {
        (*d).d.usable() as i32
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_plan_drag_flips(d: *mut PlanDragH) -> i32 {
    guard(-1, move || {
        (*d).d.flips().len() as i32
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_plan_drag_flip_list(d: *mut PlanDragH, out: *mut i32) -> i32 {
    guard(-1, move || {
        let f = (*d).d.flips();
        for (i, t) in f.iter().enumerate() {
            *out.add(3 * i) = t.0 as i32;
            *out.add(3 * i + 1) = t.1 as i32;
            *out.add(3 * i + 2) = t.2 as i32;
        }
        f.len() as i32
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_plan_drag_branches_json(d: *mut PlanDragH) -> *mut u8 {
    guard(std::ptr::null_mut(), move || {
        let plan = plan_of(d);
        let b: BTreeMap<String, i32> = (*d).d.branches(plan);
        out_json(Json::Obj(b.into_iter().map(|(k, v)| (k, Json::Int(v as i64))).collect()))
    })
}

/// The order-type triangles a numeric fallback would guard (3 point indices each).
#[no_mangle]
pub unsafe extern "C" fn gcs_plan_drag_guards(
    d: *mut PlanDragH,
    _h: *mut Sketch,
    out: *mut i32,
) -> i32 {
    guard(-1, move || {
        let plan = plan_of(d);
        let g = (*d).d.guard_triangles(plan);
        for (i, t) in g.iter().enumerate() {
            *out.add(3 * i) = t.0 as i32;
            *out.add(3 * i + 1) = t.1 as i32;
            *out.add(3 * i + 2) = t.2 as i32;
        }
        g.len() as i32
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_plan_drag_end(d: *mut PlanDragH, _h: *mut Sketch) {
    guard((), move || {
        (*d).d.end();
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_plan_drag_free(d: *mut PlanDragH) {
    guard((), move || {
        if !d.is_null() {
            drop(Box::from_raw(d));
        }
    })
}

/// The closed-form triangles of a plan (3 point indices each); returns how many.
#[no_mangle]
pub unsafe extern "C" fn gcs_ppp_triangles(p: *mut PlanSolver, out: *mut i32) -> i32 {
    guard(-1, move || {
        let t = decompose::ppp_triangles(&(*p).plan);
        for (i, x) in t.iter().enumerate() {
            *out.add(3 * i) = x.0 as i32;
            *out.add(3 * i + 1) = x.1 as i32;
            *out.add(3 * i + 2) = x.2 as i32;
        }
        t.len() as i32
    })
}

/* -- Solvent: the program a sketch is written as ----------------------------------- */

/// Read and elaborate a program.
///
/// Never null except when there is no memory: a program full of errors still comes back, with
/// whatever geometry it could build and the diagnostics beside it, because a panel has to show
/// the drawing *and* the error.  Whether to adopt the result is the caller's, from the report.
#[no_mangle]
pub unsafe extern "C" fn gcs_program_elaborate(ptr: *const u8, len: usize) -> *mut Elaborated {
    guard(std::ptr::null_mut(), move || {
        let src = as_str(ptr, len);
        let (prog, errs) = syntax::parse(src);
        let mut e = program::elaborate(&prog);
        // the parser's complaints come first, in the order they were found: they are about the
        // text, and everything after them is about a text that was already wrong
        let mut all: Vec<program::Diag> = errs
            .iter()
            .map(|s| program::Diag {
                code: program::Code::E100,
                span: s.span,
                stmt: None,
                message: s.message.clone(),
            })
            .collect();
        all.append(&mut e.diags);
        e.diags = all;
        Box::into_raw(Box::new(e))
    })
}

/// Colour a program: `[[class, lo, hi], …]`, in order, over the classified runs only.
///
/// A function of the *text* and nothing else, so a panel colours what somebody is halfway through
/// typing — which never elaborates and is exactly the program being looked at.
#[no_mangle]
pub unsafe extern "C" fn gcs_program_highlight(ptr: *const u8, len: usize) -> *mut u8 {
    guard(std::ptr::null_mut(), move || {
        let runs: Vec<Json> = syntax::highlight(as_str(ptr, len))
            .into_iter()
            .map(|(t, s)| {
                Json::Arr(vec![
                    Json::Str(t.as_str().to_string()),
                    Json::Int(s.lo as i64),
                    Json::Int(s.hi as i64),
                ])
            })
            .collect();
        out_json(Json::Arr(runs))
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_elab_free(h: *mut Elaborated) {
    if !h.is_null() {
        drop(Box::from_raw(h));
    }
}

/// Take the sketch out.  The caller owns it and frees it with `gcs_sketch_free`; a second call
/// returns null, because there was only ever one.
#[no_mangle]
pub unsafe extern "C" fn gcs_elab_take_sketch(h: *mut Elaborated) -> *mut Sketch {
    guard(std::ptr::null_mut(), move || {
        let e = &mut *h;
        match e.taken {
            true => std::ptr::null_mut(),
            false => {
                e.taken = true;
                Box::into_raw(Box::new(std::mem::take(&mut e.sketch)))
            }
        }
    })
}

/// The text the program was read from — which a splice may have moved on.
#[no_mangle]
pub unsafe extern "C" fn gcs_elab_text(h: *mut Elaborated) -> *mut u8 {
    guard(std::ptr::null_mut(), move || out_str((*h).text().to_string()))
}

/// `{"ok": bool, "diagnostics": [{severity, code, message, lo, hi, line, col}], "map": {...}}`
///
/// Line and column are computed here, so no binding scans the text a second time and the two
/// cannot disagree about where line 12 is.
#[no_mangle]
pub unsafe extern "C" fn gcs_elab_report(h: *mut Elaborated) -> *mut u8 {
    guard(std::ptr::null_mut(), move || {
        let e = &*h;
        let diags: Vec<Json> = e
            .diags
            .iter()
            .map(|d| {
                let (line, col) = d.at(e.text());
                json::object([
                    ("severity", Json::Str(format!("{:?}", d.severity()).to_lowercase())),
                    ("code", Json::Str(d.code.as_str().to_string())),
                    ("message", Json::Str(d.message.clone())),
                    ("lo", Json::Int(d.span.lo as i64)),
                    ("hi", Json::Int(d.span.hi as i64)),
                    ("line", Json::Int(line as i64)),
                    ("col", Json::Int(col as i64)),
                ])
            })
            .collect();
        let ents: Vec<Json> = e
            .map
            .of_entity
            .iter()
            .map(|(r, site)| {
                Json::Arr(vec![
                    Json::Str(r.kind.as_str().to_string()),
                    Json::Int(r.idx as i64),
                    Json::Str(
                        e.map.names.get(r).and_then(|v| v.first()).cloned().unwrap_or_default(),
                    ),
                    Json::Int(site.span.lo as i64),
                    Json::Int(site.span.hi as i64),
                ])
            })
            .collect();
        let cons: Vec<Json> = e
            .map
            .of_constraint
            .iter()
            .map(|(id, site)| {
                Json::Arr(vec![
                    Json::Int(*id as i64),
                    Json::Int(site.span.lo as i64),
                    Json::Int(site.span.hi as i64),
                ])
            })
            .collect();
        out_json(json::object([
            ("ok", Json::Bool(e.ok())),
            ("diagnostics", Json::Arr(diags)),
            ("entities", Json::Arr(ents)),
            ("constraints", Json::Arr(cons)),
        ]))
    })
}

/* -- editing the source ------------------------------------------------------------
 *
 * Every one of these returns `{text, kind, names, refused}` and changes nothing: the caller
 * applies the text it gets back by elaborating it, which is the only place a document is ever
 * replaced.  `kind` says what that costs — `numeric` means only numbers a solve may move
 * changed, so a compiled plan survives.
 */

fn out_edit(e: gcs_core::edit::Edit) -> *mut u8 {
    use gcs_core::edit::Kind;
    out_json(json::object([
        ("text", Json::Str(e.text)),
        (
            "kind",
            Json::Str(
                match e.kind {
                    Kind::Structural => "structural",
                    Kind::Numeric => "numeric",
                    Kind::None => "none",
                }
                .to_string(),
            ),
        ),
        ("names", Json::Arr(e.names.into_iter().map(Json::Str).collect())),
        ("refused", e.refused.map(Json::Str).unwrap_or(Json::Null)),
    ]))
}

/// Put a solved sketch's coordinates back into the seeds they came from.
#[no_mangle]
pub unsafe extern "C" fn gcs_elab_commit_seeds(h: *mut Elaborated, s: *mut Sketch) -> *mut u8 {
    guard(std::ptr::null_mut(), move || {
        let e = &*h;
        out_edit(gcs_core::edit::commit_seeds(e, sk(s), &e.program))
    })
}

/// The source after a gesture mutated the drawing: what is new gets a statement, what is gone
/// loses one, and every seed is committed.
#[no_mangle]
pub unsafe extern "C" fn gcs_elab_reconcile(h: *mut Elaborated, s: *mut Sketch) -> *mut u8 {
    guard(std::ptr::null_mut(), move || out_edit(gcs_core::edit::reconcile(&mut *h, sk(s))))
}

#[no_mangle]
pub unsafe extern "C" fn gcs_elab_add_point(h: *mut Elaborated, x: f64, y: f64) -> *mut u8 {
    guard(std::ptr::null_mut(), move || {
        out_edit(gcs_core::edit::add_point(&(*h).program, x, y))
    })
}

/// `{"kind": "line", "args": ["p0", "p1"], "seed": [..]}`
#[no_mangle]
pub unsafe extern "C" fn gcs_elab_add_entity(
    h: *mut Elaborated,
    ptr: *const u8,
    len: usize,
) -> *mut u8 {
    guard(std::ptr::null_mut(), move || {
        let v = as_json(ptr, len);
        let Some(kind) = EntKind::parse(v.get("kind").map(|k| k.as_str()).unwrap_or("")) else {
            set_error("unknown entity kind");
            return std::ptr::null_mut();
        };
        let args: Vec<String> = v
            .get("args")
            .map(|a| a.arr().iter().map(|x| x.as_str().to_string()).collect())
            .unwrap_or_default();
        let seed: Vec<f64> =
            v.get("seed").map(|a| a.arr().iter().map(|x| x.as_f64()).collect()).unwrap_or_default();
        out_edit(gcs_core::edit::add_entity(&(*h).program, kind, &args, &seed))
    })
}

/// One constraint, in `report::constraint_from_json`'s shape but with entities by *name* — the
/// document's own way of saying which, rather than an index into a sketch it is about to replace.
#[no_mangle]
pub unsafe extern "C" fn gcs_elab_add_relation(
    h: *mut Elaborated,
    ptr: *const u8,
    len: usize,
) -> *mut u8 {
    guard(std::ptr::null_mut(), move || {
        let v = as_json(ptr, len);
        // either spelling: `Horizontal`, the record shape both bindings build, or `horizontal`,
        // the way a statement says it.  One name for one thing, written twice.
        let name = v.get("type").map(|x| x.as_str().to_string()).unwrap_or_default();
        let kind = gcs_core::constraints::CKind::from_name(&name)
            .or_else(|| gcs_core::constraints::CKind::from_name(&gcs_core::syntax::camel(&name)));
        let Some(kind) = kind else {
            set_error(format!("unknown constraint type: {name}"));
            return std::ptr::null_mut();
        };
        let spec = kind.spec();
        let raw = v.get("args").cloned().unwrap_or(Json::Arr(Vec::new()));
        let raw = raw.arr();
        let mut args: Vec<Option<gcs_core::syntax::Arg>> = vec![None; spec.len()];
        for (i, (_, k)) in spec.iter().enumerate() {
            let Some(a) = raw.get(i) else { continue };
            if matches!(a, Json::Null) {
                continue; // left out: the core reads it off the geometry
            }
            args[i] = Some(if k.is_entity() {
                gcs_core::syntax::Arg::Ref(gcs_core::syntax::Ref::new(a.as_str().to_string()))
            } else if k.is_dimension() {
                gcs_core::syntax::Arg::Dim {
                    text: a.as_str().to_string(),
                    span: Default::default(),
                }
            } else {
                match a {
                    Json::Bool(b) => gcs_core::syntax::Arg::Bool(*b),
                    Json::Str(s) => gcs_core::syntax::Arg::Word(s.clone()),
                    Json::Int(n) => gcs_core::syntax::Arg::Int(*n),
                    other => gcs_core::syntax::Arg::Num(other.as_f64()),
                }
            });
        }
        out_edit(gcs_core::edit::add_relation(
            &(*h).program,
            gcs_core::syntax::Relation {
                kind,
                args,
                place: None,
                place_span: gcs_core::syntax::Span::default(),
                poly: None,
                // the app states constraints; a claim is written in the program panel
                claim: false,
            },
        ))
    })
}

/// `{"entities": [["point", 3], ...], "constraints": [id, ...]}`
#[no_mangle]
pub unsafe extern "C" fn gcs_elab_remove(
    h: *mut Elaborated,
    ptr: *const u8,
    len: usize,
) -> *mut u8 {
    guard(std::ptr::null_mut(), move || {
        let v = as_json(ptr, len);
        let ents: Vec<EntRef> = v
            .get("entities")
            .map(|a| {
                a.arr()
                    .iter()
                    .filter_map(|r| {
                        let p = r.arr();
                        (p.len() == 2)
                            .then(|| EntKind::parse(p[0].as_str()))
                            .flatten()
                            .map(|k| EntRef::new(k, p[1].as_i64() as usize))
                    })
                    .collect()
            })
            .unwrap_or_default();
        let cons: Vec<u32> = v
            .get("constraints")
            .map(|a| a.arr().iter().map(|x| x.as_i64() as u32).collect())
            .unwrap_or_default();
        let e = &*h;
        out_edit(gcs_core::edit::remove(e, &e.program, &ents, &cons))
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_elab_set_dimension(
    h: *mut Elaborated,
    cid: u32,
    ap: *const u8,
    an: usize,
    tp: *const u8,
    tn: usize,
) -> *mut u8 {
    guard(std::ptr::null_mut(), move || {
        let e = &*h;
        out_edit(gcs_core::edit::set_dimension(
            e,
            &e.program,
            cid,
            as_str(ap, an),
            as_str(tp, tn),
        ))
    })
}

/// Take a new source that says the same statements, keeping the drawing.  0 when it cannot —
/// the caller then elaborates, which is always correct and only slower.
#[no_mangle]
pub unsafe extern "C" fn gcs_elab_retext(h: *mut Elaborated, p: *const u8, n: usize) -> i32 {
    guard(0, move || (*h).retext(&as_str(p, n)) as i32)
}

/// One curve as a polyline, over the interval that curve is drawn on.
///
/// The core lays it out: a curve is geometry, so where it goes is the core's answer and a front
/// end only strokes what it is handed — the same bargain a dimension callout and a B-spline
/// already strike.  Writes `2n` doubles and returns `n`; when `cap` is too small it writes
/// nothing and still returns the count it wanted, so a caller sizes its buffer by asking once.
#[no_mangle]
pub unsafe extern "C" fn gcs_curve_polyline(
    h: *mut Sketch,
    idx: i32,
    out: *mut f64,
    cap: i32,
) -> i32 {
    guard(-1, move || {
        let s = sk(h);
        let i = idx as usize;
        if i >= s.curves.len() {
            return -1;
        }
        let pts = s.curve_polyline(i);
        if (pts.len() as i32) <= cap && !out.is_null() {
            for (k, (x, y)) in pts.iter().enumerate() {
                *out.add(2 * k) = *x;
                *out.add(2 * k + 1) = *y;
            }
        }
        pts.len() as i32
    })
}

/// A sketch as a program — the lift, and the whole of the migration: every document ever saved
/// becomes text through this.
#[no_mangle]
pub unsafe extern "C" fn gcs_sketch_to_program(h: *mut Sketch) -> *mut u8 {
    guard(std::ptr::null_mut(), move || out_str(program::dumps(sk(h))))
}
