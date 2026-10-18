//! Parameters that run (#398 L): what a timeline asks — the declared parameters, a march, the
//! drawing stood at an instant and sent home, and a point dragged along its track.

use super::*;
use gcs_core::follow::{self, Scrub};

/// The parameters the drawing declares (`report::parameters_json`).
#[no_mangle]
pub unsafe extern "C" fn gcs_parameters_json(h: *mut Sketch) -> *mut u8 {
    json_out(move || report::parameters_json(sk(h)))
}

/// How the drawing runs over its parameter `name` (`report::parameter_json`): it is followed
/// first, and the march remembered on the sketch.
#[no_mangle]
pub unsafe extern "C" fn gcs_parameter_json(h: *mut Sketch, np: *const u8, nn: usize) -> *mut u8 {
    json_out(move || report::parameter_json(sk(h), as_str(np, nn)))
}

/// Stand the drawing at `t` of `name` (`follow::stand`): `""`, or why it is left at home.
#[no_mangle]
pub unsafe extern "C" fn gcs_parameter_stand(h: *mut Sketch, np: *const u8, nn: usize, t: f64)
    -> *mut u8 {
    guard(std::ptr::null_mut(), move || {
        out_str(follow::stand(sk(h), as_str(np, nn), t).err().unwrap_or_default())
    })
}

/// Send the drawing home (`follow::home`).
#[no_mangle]
pub unsafe extern "C" fn gcs_parameter_home(h: *mut Sketch) {
    guard((), move || follow::home(sk(h)))
}

/// `{"at": t}` for an instant of `name` as written (`follow::instant`), or `{"error": why}`.
#[no_mangle]
pub unsafe extern "C" fn gcs_parameter_instant(
    h: *mut Sketch,
    np: *const u8,
    nn: usize,
    tp: *const u8,
    tn: usize,
) -> *mut u8 {
    json_out(move || match follow::instant(sk(h), as_str(np, nn), as_str(tp, tn)) {
        Ok(t) => json::object([("at", t.into())]),
        Err(e) => json::object([("error", e.into())]),
    })
}

/// A value of `name` as a reader reads it (`follow::reading`): `129.838deg`.
#[no_mangle]
pub unsafe extern "C" fn gcs_parameter_reading(h: *mut Sketch, np: *const u8, nn: usize, v: f64)
    -> *mut u8 {
    guard(std::ptr::null_mut(), move || {
        let s = sk(h);
        out_str(s.parameters.get(as_str(np, nn))
            .map(|over| follow::reading(over, v, s.units)).unwrap_or_default())
    })
}

/// Take hold of `point` to drag it along the march of `name` (`follow::Scrub`): read in its own
/// plane, or where the eye at bearing `az` and elevation `el` (radians) sees it when `seen` is
/// set.  Null, with why in `gcs_last_error()`, where it does not move as the parameter runs.
#[no_mangle]
#[allow(clippy::too_many_arguments)]
pub unsafe extern "C" fn gcs_scrub_new(
    h: *mut Sketch,
    np: *const u8,
    nn: usize,
    point: i32,
    seen: i32,
    az: f64,
    el: f64,
) -> *mut Scrub {
    guard(std::ptr::null_mut(), move || {
        let eye = (seen != 0).then_some((az, el));
        let Some(p) = index(point) else {
            set_error(format!("{point} names no point"));
            return std::ptr::null_mut();
        };
        match Scrub::new(sk(h), as_str(np, nn), p, eye) {
            Ok(s) => Box::into_raw(Box::new(s)),
            Err(e) => {
                set_error(e);
                std::ptr::null_mut()
            }
        }
    })
}

/// The pointer at (`x`, `y`): the instant found, and the drawing stood there.  NaN, with why in
/// `gcs_last_error()`, where it could not be stood.
#[no_mangle]
pub unsafe extern "C" fn gcs_scrub_move(s: *mut Scrub, h: *mut Sketch, x: f64, y: f64) -> f64 {
    guard(f64::NAN, move || {
        let s = &mut *s;
        let t = s.to(x, y);
        match follow::stand(sk(h), &s.name, t) {
            Ok(()) => t,
            Err(e) => {
                set_error(e);
                f64::NAN
            }
        }
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_scrub_free(s: *mut Scrub) {
    guard((), move || {
        if !s.is_null() {
            drop(Box::from_raw(s));
        }
    })
}
