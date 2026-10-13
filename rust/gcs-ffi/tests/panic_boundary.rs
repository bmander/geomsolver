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

#[test]
fn invalid_solid_exports_report_geometry_errors_without_panicking() {
    for src in [
        include_str!("../../gcs-core/tests/fixtures/solid_issue50/08-invalid_face_bowtie.sv"),
        include_str!("../../gcs-core/tests/fixtures/solid_issue50/09-invalid_revolve_crossing.sv"),
    ] {
        unsafe {
            let e = gcs_program_elaborate(src.as_ptr(), src.len());
            let sk = gcs_elab_take_sketch(e);
            assert!(gcs_solid_stl(sk, 0, 0.0).is_null());
            assert!(!last_error().is_empty());
            assert!(!last_error().contains("internal error"));
            assert!(gcs_solid_glb(sk, 0, 0.0).is_null());
            gcs_sketch_free(sk);
            gcs_elab_free(e);
        }
    }
}

/// A string the ABI handed back, freed.
fn take(p: *mut u8) -> String {
    unsafe {
        let len = gcs_str_len(p) as usize;
        let s = std::str::from_utf8(std::slice::from_raw_parts(gcs_str_ptr(p), len))
            .unwrap_or("")
            .to_string();
        gcs_str_free(p);
        s
    }
}

/// A negative solid index names no solid — never solid 0, which `max(0)` once read it as — and
/// neither does one past the drawing's solids: every entry point taking a solid refuses both
/// alike, by its neutral return with the reason.
#[test]
fn a_solid_index_out_of_range_names_no_solid() {
    let src = "unit mm\nuse std\nin std.front {\n\
        a := point hint((0, 0))\nb := point hint((60, 0))\nc := point hint((60, 40))\n\
        d := point hint((0, 40))\nab := line(a, b)\nbc := line(b, c)\ncd := line(c, d)\n\
        da := line(d, a)\n\
        fix((0, 0)) a\nfix((60, 0)) b\nfix((60, 40)) c\nfix((0, 40)) d\n}\n\
        block := solid(face(ab, bc, cd, da), depth: 5)\n";
    unsafe {
        let e = gcs_program_elaborate(src.as_ptr(), src.len());
        assert!(!e.is_null(), "{}", last_error());
        let sk = gcs_elab_take_sketch(e);
        let (az, el) = (0.6, 0.4);
        // solid 0 is there, and every entry point reads it
        let stl = gcs_solid_stl(sk, 0, 0.0);
        assert!(!stl.is_null(), "{}", last_error());
        gcs_str_free(stl);
        assert!(gcs_solid_mesh_unit(sk, 0) > 0.0);
        assert_ne!(take(gcs_workspace_extrude_handle_json(sk, az, el, 0)), "null");
        let all = gcs_solid_glb(sk, -1, 0.0);
        assert!(!all.is_null(), "-1 asks a .glb for every object: {}", last_error());
        gcs_str_free(all);
        for idx in [-1, -7, 1] {
            let refused = |what: &str| {
                let err = last_error();
                assert!(err.contains("names no solid"), "{what}({idx}): {err:?}");
            };
            assert!(gcs_solid_stl(sk, idx, 0.0).is_null());
            refused("stl");
            assert!(gcs_solid_stl_preview(sk, idx, 0.0).is_null());
            refused("stl_preview");
            assert!(gcs_solid_exact(sk, idx, 0, 0.0).is_null());
            refused("exact");
            assert!(gcs_solid_faces_json(sk, idx, 0.0).is_null());
            refused("faces_json");
            assert_eq!(gcs_solid_mesh(sk, idx, 0.0, std::ptr::null_mut(), 0), -1);
            refused("mesh");
            assert_eq!(gcs_solid_provisional(sk, idx), -1);
            refused("provisional");
            assert_eq!(gcs_solid_mesh_unit(sk, idx), 0.0);
            refused("mesh_unit");
            assert!(gcs_exact_builder_new(sk, idx).is_null());
            refused("exact_builder_new");
            assert!(gcs_workspace_extent_at(sk, az, el, idx, 0.0, 0.0).is_nan());
            assert!(gcs_workspace_turn_at(sk, az, el, idx, 0.0, 0.0, 0.0).is_nan());
            assert_eq!(take(gcs_workspace_extrude_handle_json(sk, az, el, idx)), "null");
            assert_eq!(take(gcs_workspace_turn_handle_json(sk, az, el, idx)), "null");
            assert_eq!(gcs_sketch_set_prism(sk, idx, 0, 9.0), 0);
            if idx < 0 {
                assert!(gcs_solid_glb(sk, 1, 0.0).is_null());
                refused("glb");
                let how = r#"{"way": "behind", "thick": "9"}"#;
                assert!(gcs_elab_set_sweep(e, idx, how.as_ptr(), how.len()).is_null());
                refused("set_sweep");
                assert!(gcs_elab_set_turn(e, idx, "90".as_ptr(), 2).is_null());
                refused("set_turn");
            }
        }
        // and solid 0 was left as it was by every one of them
        assert_ne!(take(gcs_workspace_extrude_handle_json(sk, az, el, 0)), "null");
        gcs_sketch_free(sk);
        gcs_elab_free(e);
    }
}
