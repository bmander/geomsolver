//! The solver used through the ABI with no Solvent text anywhere (#409).
//!
//! A host that is not the app — a C program, another language's binding — builds a sketch from
//! entities and constraint records, solves it and asks what is wrong with it, and never writes a
//! document.  The language is one way to make a sketch, not the only way in, so the published
//! surface is checked here to be enough on its own: points and lines by `gcs_sketch_*`,
//! constraints by the records `report::registry_json` describes, then `gcs_solve` and
//! `gcs_diagnose_json`.

use gcs::*;
use gcs_core::json::{parse, Json};

/// A string the ABI hands back, `[u32 len][bytes]`, read and freed.
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

/// Add the constraint `record` says, as a binding does: its id, or the refusal's text.
fn add(h: *mut gcs_core::model::Sketch, record: &str) -> Result<i32, String> {
    let id = unsafe { gcs_constraint_add(h, record.as_ptr(), record.len()) };
    if id < 0 { Err(take(gcs_last_error())) } else { Ok(id) }
}

fn diagnosis(h: *mut gcs_core::model::Sketch) -> Json {
    parse(&take(unsafe { gcs_diagnose_json(h, b"{}".as_ptr(), 2) })).expect("diagnosis is JSON")
}

fn ids(v: Option<&Json>) -> Vec<i64> {
    let mut ids: Vec<i64> =
        v.map(|v| v.arr().iter().map(Json::as_i64).collect()).unwrap_or_default();
    ids.sort();
    ids
}

/// A right triangle with legs 30 and 40, started from rough guesses: one point fixed, one leg
/// horizontal, the legs perpendicular, two distances.  It solves to the triangle, the diagnosis
/// finds it well-constrained, and a hypotenuse the legs already decide — stated wrong, as 51 —
/// is a conflict the diagnosis names with the constraints it is against.
#[test]
fn a_sketch_built_from_records_solves_and_is_diagnosed() {
    unsafe {
        let h = gcs_sketch_new();
        let a = gcs_sketch_point(h, 0.0, 0.0, 1, b"a".as_ptr(), 1);
        let b = gcs_sketch_point(h, 25.0, 3.0, 0, b"b".as_ptr(), 1);
        let c = gcs_sketch_point(h, 2.0, 33.0, 0, b"c".as_ptr(), 1);
        let ab = gcs_sketch_line(h, a, b);
        let ac = gcs_sketch_line(h, a, c);
        let mut stated = vec![
            add(h, &format!(r#"{{"type":"Horizontal","args":[["line",{ab}]]}}"#)).unwrap(),
            add(h, &format!(r#"{{"type":"Perpendicular","args":[["line",{ab}],["line",{ac}]]}}"#))
                .unwrap(),
            add(h, &format!(r#"{{"type":"Distance","args":[["point",{a}],["point",{b}],30]}}"#))
                .unwrap(),
            add(h, &format!(r#"{{"type":"Distance","args":[["point",{a}],["point",{c}],40]}}"#))
                .unwrap(),
        ];

        let mut result = [0.0f64; 9];
        take(gcs_solve(h, 0, 1e-10, 100, 1000, -1, result.as_mut_ptr()));
        assert_eq!(result[0], 1.0, "the solve succeeds");
        assert!(result[3] < 1e-9, "max residual {}", result[3]);
        let mut x = vec![0.0f64; 6];
        gcs_sketch_get_x(h, x.as_mut_ptr());
        let expect = [0.0, 0.0, 30.0, 0.0, 0.0, 40.0];
        for (got, want) in x.iter().zip(expect) {
            assert!((got - want).abs() < 1e-8, "{x:?}");
        }

        let d = diagnosis(h);
        assert_eq!(d.get("dof").map(Json::as_i64), Some(0));
        assert!(ids(d.get("over")).is_empty());
        for e in d.get("entityState").expect("every entity's state").arr() {
            assert_eq!(e.arr()[2].as_str(), "well", "{e:?}");
        }

        stated.push(
            add(h, &format!(r#"{{"type":"Distance","args":[["point",{b}],["point",{c}],51]}}"#))
                .unwrap(),
        );
        take(gcs_solve(h, 0, 1e-10, 100, 1000, -1, result.as_mut_ptr()));
        assert_eq!(result[0], 0.0, "a triangle with sides 30, 40 and 51 at a right angle is none");
        let d = diagnosis(h);
        // the horizontal only places the figure: the conflict is the other four
        stated.remove(0);
        stated.sort();
        let stated: Vec<i64> = stated.iter().map(|&i| i as i64).collect();
        assert_eq!(ids(d.get("over")), stated);
        assert_eq!(ids(d.get("conflicts")), stated);

        gcs_sketch_free(h);
    }
}

/// A record the registry does not describe is refused with the reason, as a document's
/// statement would be: the text-free door checks what it is handed as the language does.
#[test]
fn a_bad_record_is_refused_with_its_reason() {
    unsafe {
        let h = gcs_sketch_new();
        let p = gcs_sketch_point(h, 0.0, 0.0, 0, std::ptr::null(), 0);
        let q = gcs_sketch_point(h, 1.0, 0.0, 0, std::ptr::null(), 0);
        let unknown = add(h, r#"{"type":"Nearby","args":[]}"#).unwrap_err();
        assert!(unknown.contains("unknown constraint type"), "{unknown}");
        let wrong = add(h, &format!(r#"{{"type":"Horizontal","args":[["point",{p}]]}}"#));
        assert!(wrong.unwrap_err().contains("does not take"));
        let past =
            add(h, &format!(r#"{{"type":"Distance","args":[["point",{p}],["point",9],1]}}"#));
        assert!(past.unwrap_err().contains("out of range"));
        assert!(add(h, &format!(r#"{{"type":"Distance","args":[["point",{p}],["point",{q}],1]}}"#))
            .is_ok());
        gcs_sketch_free(h);
    }
}
