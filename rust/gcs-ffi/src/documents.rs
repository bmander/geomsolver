//! Document state that travels: branches, the rebuild walk, and the examples.

use super::*;

/// A place the pointer gave, to the digits a pixel of `px` page units justifies
/// (`edit::placed`).
#[no_mangle]
pub extern "C" fn gcs_placed(x: f64, px: f64) -> f64 {
    guard(x, move || gcs_core::edit::placed(x, px))
}

/* -- branches (document state) --------------------------------------------- */

#[no_mangle]
pub unsafe extern "C" fn gcs_branches_json(h: *mut Sketch) -> *mut u8 {
    json_out(move || {
        let s = sk(h);
        Json::Obj(
            s.branches.iter().map(|(k, &v)| (k.clone(), Json::Int(v as i64))).collect(),
        )
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
        Box::into_raw(Box::new(graft::without(s, &ent_list(ents, ents_len), &constraints)))
    })
}

/// A fresh sketch holding just `entities` (`[[kind, idx], ...]`), the points that define them and
/// the constraints all of whose entities came along — what a copy puts on the clipboard.
#[no_mangle]
pub unsafe extern "C" fn gcs_copy(h: *mut Sketch, ents: *const u8, ents_len: usize) -> *mut Sketch {
    guard(std::ptr::null_mut(), move || {
        Box::into_raw(Box::new(graft::copy(sk(h), &ent_list(ents, ents_len))))
    })
}

/// Add everything in `clip` to the sketch, moved by (dx, dy).  Returns what it made, as
/// `[[kind, idx], ...]` in the order the clipboard held them.
#[no_mangle]
pub unsafe extern "C" fn gcs_paste(h: *mut Sketch, clip: *mut Sketch, dx: f64,
                                   dy: f64) -> *mut u8 {
    json_out(move || {
        let made = graft::paste(sk(h), &*clip, dx, dy);
        Json::Arr(made.into_iter().map(report::ent_json).collect())
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
    json_out(move || {
        Json::Arr(
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
        )
    })
}
