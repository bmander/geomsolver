//! A material through the ABI (#510 P3): a document stating one, elaborated by
//! `gcs_program_elaborate`, hands a binding its materials and the solids made of each
//! (`gcs_materials_json`), and the sketch's JSON (`gcs_sketch_to_json`, `gcs_sketch_from_json`)
//! carries them across and back unchanged.

use gcs::*;
use gcs_core::json::{parse, Json};

/// A string block the ABI handed back — `[u32 len][bytes]`, owned by the caller — as a `String`.
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

const DOC: &str = "\
unit in
use std
steel := material(E: 200GPa, nu: 0.3, rho: 7850kg / 1m^3, yield: 250MPa)
in std.top {
  rim := radius(1in) circle(center: std.top.origin)
  disc := solid(face(rim), depth: 0.25in) of steel
}
";

fn materials(h: *mut gcs_core::model::Sketch) -> Json {
    parse(&take(unsafe { gcs_materials_json(h) })).expect("materials are JSON")
}

#[test]
fn a_material_crosses_the_abi_and_comes_back() {
    unsafe {
        let e = gcs_program_elaborate(DOC.as_ptr(), DOC.len());
        assert!(!e.is_null());
        let h = gcs_elab_take_sketch(e);
        gcs_elab_free(e);
        let ms = materials(h);
        let [m] = ms.arr() else { panic!("one material: {}", ms.dump(None)) };
        assert_eq!(m.get("name").map(Json::as_str), Some("steel"));
        // in SI whatever the document's unit: an inch document's steel is 200GPa
        let e_pa = m.get("E").map(Json::as_f64).unwrap();
        assert!((e_pa - 200e9).abs() <= 1e-12 * 200e9, "{e_pa}");
        assert_eq!(m.get("text").map(Json::as_str),
                   Some("steel: E 200GPa, nu 0.3, rho 7850kg/m^3, yield 250MPa, of disc"));
        assert_eq!(m.get("solids").map(|s| s.arr().len()), Some(1));
        // the sketch's JSON across and back: the same materials (its solids are the source's to
        // say, and the JSON carries none, so none is made of one there)
        let json = take(gcs_sketch_to_json(h, -1));
        let back = gcs_sketch_from_json(json.as_ptr(), json.len());
        assert!(!back.is_null());
        let again = materials(back);
        let [n] = again.arr() else { panic!("one material: {}", again.dump(None)) };
        for k in ["name", "E", "nu", "rho", "yield"] {
            assert_eq!(n.get(k).map(|v| v.dump(None)), m.get(k).map(|v| v.dump(None)), "{k}");
        }
        assert_eq!(take(gcs_sketch_to_json(back, -1)), json);
        // a document whose material no property admits is refused, with the core's reason
        let bad = json.replace("\"nu\":0.3", "\"nu\":0.7");
        assert_ne!(bad, json, "{json}");
        assert!(gcs_sketch_from_json(bad.as_ptr(), bad.len()).is_null());
        assert!(take(gcs_last_error()).contains("Poisson's ratio"));
        gcs_sketch_free(back);
        gcs_sketch_free(h);
    }
}
