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

mod sketch;
mod entities;
mod constraints;
mod solids;
mod workspace;
mod documents;
mod systems;
mod analysis;
mod drags;
mod parameters;
mod solvent;
mod editing;

/* Every entry point is reached from the crate root, as it was before the split: the rlib's
 * tests and a Rust caller name `gcs::gcs_sketch_new`, never the module it is written in. */
pub use sketch::*;
pub use entities::*;
pub use constraints::*;
pub use solids::*;
pub use workspace::*;
pub use documents::*;
pub use systems::*;
pub use analysis::*;
pub use drags::*;
pub use parameters::*;
pub use solvent::*;
pub use editing::*;

use gcs_core::callout;
use gcs_core::cgraph::{self, El};
use gcs_core::constraints::Constraint;
use gcs_core::curve;
use gcs_core::decompose::{self, Plan, PlanDrag, PlanSolver};
use gcs_core::diagnose::{self, DiagnoseOptions};
use gcs_core::expr;
use gcs_core::homotopy::{self, EnumerateOptions};
use gcs_core::graft;
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

/// Bytes out of the core, under the same length-prefixed handle a string uses — the convention
/// was always bytes, and only the reader on the far side assumed they were characters.
fn out_bytes(bytes: Vec<u8>) -> *mut u8 {
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

/// An entry point whose answer is JSON: `f`'s, written out, or null with the reason where the
/// core panicked — `guard` and `out_json` together, the shape of most of the ragged answers.
fn json_out(f: impl FnOnce() -> Json) -> *mut u8 {
    guard(std::ptr::null_mut(), move || out_json(f()))
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

/// `{ents: [[kind, idx]…], solids: [idx…]}` — a part of the drawing as the outline names it
/// (`report::outline_json`), every kind spelled as `EntKind::as_str` spells it — as the entities
/// and the solids, skipping a kind the model does not know.
unsafe fn part_of(p: *const u8, len: usize) -> (Vec<EntRef>, Vec<usize>) {
    let j = as_json(p, len);
    let ents = j.get("ents").map(|v| v.arr()).unwrap_or(&[]).iter().filter_map(|v| {
        let a = v.arr();
        let word = a.first().map(|x| x.as_str()).unwrap_or("");
        let kind = EntKind::ALL.into_iter().find(|k| k.as_str() == word)?;
        Some(EntRef::new(kind, a.get(1)?.as_i64().max(0) as usize))
    }).collect();
    let solids = j.get("solids").map(|v| v.arr()).unwrap_or(&[]).iter()
        .map(|v| v.as_i64()).filter(|&i| i >= 0).map(|i| i as usize).collect();
    (ents, solids)
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

/* -- indices across the ABI ------------------------------------------------ */

/// An index as a binding passes it (`i32`): what it counts to, or `None` for a negative one.  A
/// negative index names nothing — never the first thing, which `max(0)` read it as.
fn index(i: i32) -> Option<usize> {
    usize::try_from(i).ok()
}

/// The solid `idx` names in `s`, or `None` with the reason in `gcs_last_error()`.  Every entry
/// point taking a solid of a sketch reads its index here, so a negative one and one past the
/// drawing's solids are refused alike, each by its entry point's neutral return.
fn solid_at(s: &Sketch, idx: i32) -> Option<usize> {
    let i = index(idx).filter(|&i| i < s.solids.len());
    if i.is_none() {
        set_error(format!("{idx} names no solid of this drawing (it has {})", s.solids.len()));
    }
    i
}

/* -- library metadata ------------------------------------------------------ */

/// The constraint-type and kernel registry — everything a front end needs to be generic.
#[no_mangle]
pub extern "C" fn gcs_registry_json() -> *mut u8 {
    json_out(move || {
        report::registry_json()
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
