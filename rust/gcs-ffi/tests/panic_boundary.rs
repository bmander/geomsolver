//! The ABI is a panic boundary, and this is the only test of it.
//!
//! `guard` wraps every entry point so a core panic — a bad index, a broken invariant — becomes
//! `gcs_last_error()` and a neutral return instead of an abort that would take the host process
//! with it.  That needs `panic = "unwind"` in the release profile, which is why the profile says
//! so; `wasm32-unknown-unknown` aborts whatever the profile says, so this can only be checked on
//! the native target and can never move to the web suite.

use gcs::*;

/// `gcs_last_error()` as a `String`.  The ABI hands back `[u32 len][bytes]`, owned by the caller.
fn last_error() -> String {
    unsafe {
        let p = gcs_last_error();
        let len = gcs_str_len(p) as usize;
        let s = std::str::from_utf8(std::slice::from_raw_parts(gcs_str_ptr(p), len))
            .unwrap_or("")
            .to_string();
        gcs_str_free(p);
        s
    }
}

#[test]
fn a_core_panic_comes_back_as_an_error() {
    unsafe {
        let h = gcs_sketch_new();
        assert_eq!(gcs_sketch_point(h, 2.0, 3.0, 0, std::ptr::null(), 0), 0);

        let v = gcs_param_value(h, 99);
        assert!(v.is_nan(), "a panicking entry point returns its neutral value, got {v}");
        let err = last_error();
        assert!(err.contains("out of range"), "the panic's message survives: {err:?}");

        // and the sketch is untouched and still usable
        assert_eq!(gcs_param_value(h, 0), 2.0);
        assert_eq!(gcs_param_value(h, 1), 3.0);
        assert_eq!(gcs_sketch_point(h, 5.0, 7.0, 0, std::ptr::null(), 0), 1);
        assert_eq!(gcs_param_value(h, 2), 5.0);

        gcs_sketch_free(h);
    }
}

/// A neutral return is not the same as a *silent* one: an entry point that fails without
/// panicking sets the error too, and the next successful call leaves the old text alone rather
/// than clearing it — a caller reads the error because a return value told it to.
#[test]
fn a_rejected_call_reports_why() {
    unsafe {
        let h = gcs_sketch_new();
        let bad = [0.0f64; 3];
        assert_eq!(gcs_sketch_set_x(h, bad.as_ptr(), bad.len()), -1);
        let err = last_error();
        assert!(err.contains("set_x"), "{err:?}");
        gcs_sketch_free(h);
    }
}
