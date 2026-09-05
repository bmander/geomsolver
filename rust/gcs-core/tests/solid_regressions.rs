//! Grounded reductions from issue #50, plus controls that distinguish the failure modes.
use gcs_core::model::SolidDef;
use gcs_core::solve::{solve, SolveOpts};
use gcs_core::{mesh, program, report, solid, syntax};
use std::collections::BTreeMap;

fn source(number: usize) -> String {
    let dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures/solid_issue50");
    let prefix = format!("{number:02}-");
    let path = std::fs::read_dir(dir)
        .unwrap()
        .map(|e| e.unwrap().path())
        .find(|p| {
            p.file_name()
                .unwrap()
                .to_str()
                .unwrap()
                .starts_with(&prefix)
        })
        .unwrap();
    std::fs::read_to_string(path).unwrap()
}

fn read(src: &str) -> program::Elaborated {
    let (p, errors) = syntax::parse(src);
    assert!(errors.is_empty(), "{errors:?}");
    let mut e = program::elaborate(&p);
    assert!(e.ok(), "{:?}", e.diags);
    assert!(solve(&mut e.sketch, SolveOpts::default()).success);
    e
}

fn positions(src: &str) -> BTreeMap<String, f64> {
    let e = read(src);
    let errors = program::solid_diagnostics(&e.sketch, &e.map);
    assert!(errors.is_empty(), "{errors:?}");
    report::positions(&e.sketch, &e.map).into_iter().collect()
}

fn near(got: f64, want: f64, relative: f64) {
    assert!(
        (got - want).abs() <= want.abs().max(1e-30) * relative,
        "got {got}, expected {want} within {relative} relative"
    );
}

fn closed(positions: &[f64]) {
    let mut edges = BTreeMap::new();
    let key = |p: &[f64]| {
        p.iter()
            .map(|&v| if v == 0.0 { 0 } else { v.to_bits() })
            .collect::<Vec<_>>()
    };
    for t in positions.chunks_exact(9) {
        let v = [key(&t[0..3]), key(&t[3..6]), key(&t[6..9])];
        for i in 0..3 {
            *edges
                .entry((v[i].clone(), v[(i + 1) % 3].clone()))
                .or_insert(0) += 1;
        }
    }
    assert!(!edges.is_empty());
    for ((a, b), n) in &edges {
        assert_eq!(edges.get(&(b.clone(), a.clone())), Some(n));
    }
}

#[test]
fn translated_bore_evaluates_without_recursing_on_its_own_plane() {
    let src = source(2);
    let e = read(&src);
    let si = e.map.ent_named("result").unwrap().i();
    let p: BTreeMap<_, _> = report::positions(&e.sketch, &e.map).into_iter().collect();
    let origin = positions(
        &src.replace("1000000000", "0")
            .replace("1000000010", "10")
            .replace("1000000005", "5"),
    );
    near(p["result.volume"], origin["result.volume"], 1e-8);
    closed(&e.sketch.solid_mesh(si, 0.0).positions);
    assert!(
        mesh::checked_stl(&e.sketch.solid_boundary(si, 0.0), "result")
            .unwrap_err()
            .contains("float32 STL")
    );
    // World coordinates around 1e9 cannot encode this 10 mm object in float32 STL;
    // the native f64 boundary and report are the geometry checked here.
}

#[test]
fn mate_ordinates_are_sorted_and_subtraction_reverses_the_floor() {
    let reversed = positions(&source(3));
    let ordered = positions(&source(3).replace("from: 0mm, to: -2mm", "from: -2mm, to: 0mm"));
    assert_eq!(reversed["result.bounds.y0"], -2.0);
    assert_eq!(reversed["result.bounds.y1"], 0.0);
    assert_eq!(reversed, ordered);
    let floor = positions(&source(4));
    assert_eq!(floor["result.bounds.y0"], 2.0);
    assert_eq!(floor["result.bounds.y1"], 3.0);
}

#[test]
fn a_derived_plane_inherits_its_mated_parents_final_origin() {
    let p = positions(&source(5));
    assert_eq!(p["result.bounds.y0"], -5.0);
    assert_eq!(p["result.bounds.y1"], -4.0);
    let src = source(5)
        .replace(
            "plane child(",
            "plane middle(origin: o, toward: q, from: back, offset: 1mm)\nplane child(",
        )
        .replace("from: back, offset: 3mm", "from: middle, offset: 2mm");
    let deeper = positions(&src);
    assert_eq!(deeper["result.bounds.y0"], -5.0);
}

#[test]
fn body_reports_keep_every_operand_path() {
    let src = source(6);
    let p = positions(&src);
    assert_eq!(p["result.stock.near.area"], 91.0);
    assert_eq!(p["result.boss.near.area"], 9.0);
    assert_eq!(p["result.volume"], 518.0);
    assert_eq!(p["result.area"], 424.0);
    let face_area: f64 = p
        .iter()
        .filter(|(key, _)| {
            key.starts_with("result.") && key.ends_with(".area") && key.as_str() != "result.area"
        })
        .map(|(_, value)| value)
        .sum();
    assert_eq!(face_area, p["result.area"]);
    let p = positions(&format!("{src}\nsolid outer(result)\n"));
    assert_eq!(p["outer.result.stock.near.area"], 91.0);
    assert_eq!(p["outer.result.boss.near.area"], 9.0);
}

#[test]
fn containment_tests_enclosed_voids_as_well_as_the_exterior() {
    use gcs_core::constraints::SolidWord;
    for void in [true, false] {
        let src = if void {
            source(7)
        } else {
            source(7).replace("void cut shell", "")
        };
        let e = read(&src);
        let a = e.map.ent_named("result").unwrap().i();
        let b = e.map.ent_named("shell").unwrap().i();
        assert_eq!(
            gcs_core::clear::judge(&e.sketch, SolidWord::Inside, a, b, 0.0, solid::REPORT_UNIT)
                .holds(),
            Some(!void)
        );
    }
}

#[test]
fn invalid_solved_profiles_and_revolution_axes_are_diagnosed() {
    for src in [
        source(8),
        source(9),
        source(9).replace("x: 5, y: 10", "x: 5, y: 0"),
    ] {
        let e = read(&src);
        let errors = program::solid_diagnostics(&e.sketch, &e.map);
        assert!(!errors.is_empty());
        assert!(errors.iter().all(|d| d.span.hi > d.span.lo));
        assert!(!report::positions(&e.sketch, &e.map)
            .iter()
            .any(|(key, _)| key == "result.volume"));
    }
    for coords in [
        vec![(0.0, 0.0), (5.0, 0.0), (10.0, 0.0)],
        vec![(0.0, 0.0); 3],
        vec![
            (0.0, 0.0),
            (10.0, 0.0),
            (10.0, 10.0),
            (5.0, 10.0),
            (5.0, 15.0),
            (5.0, 10.0),
            (0.0, 10.0),
        ],
    ] {
        let mut src = String::from("unit mm\n");
        for (i, (x, y)) in coords.iter().enumerate() {
            src += &format!("point p{i} hint(x: {x}, y: {y})\nground p{i}\n");
        }
        src += &format!(
            "face f({}, -> close)\nsolid result(f, depth: 5mm)\n",
            (0..coords.len())
                .map(|i| format!("p{i}"))
                .collect::<Vec<_>>()
                .join(", ")
        );
        let e = read(&src);
        assert!(!program::solid_diagnostics(&e.sketch, &e.map).is_empty());
    }
}

#[test]
fn nonexistent_and_removed_mate_faces_are_refused() {
    let (p, _) = syntax::parse(&source(10));
    let e = program::elaborate(&p);
    assert!(e.errors().any(|d| d.code == program::Code::E082));
    let removed = source(4)
        .replace("from: -3mm, to: 0mm", "from: -6mm, to: 0mm")
        .replace("x: 2,", "x: 0,")
        .replace("y: 2)", "y: 0)")
        .replace("x: 8,", "x: 10,")
        .replace("y: 8)", "y: 10)")
        .replace("body.tool.far", "body.stock.near");
    let e = read(&removed);
    let errors = program::solid_diagnostics(&e.sketch, &e.map);
    assert!(
        errors.iter().any(|d| d.code == program::Code::E082),
        "{errors:?}"
    );
}

#[test]
fn acyclic_body_nesting_has_no_fake_cycle_or_empty_term_limit() {
    let p = positions(&source(11));
    assert_eq!(p["result.volume"], 1000.0);
    let mut e = read(&source(11));
    let mut last = e.map.ent_named("result").unwrap().i();
    for i in 0..512 {
        last = e.sketch.solid(
            SolidDef::Body {
                stock: last as u32,
                on: vec![],
                through: vec![],
            },
            &format!("wrap{i}"),
        );
    }
    assert_eq!(
        mesh::volume(&e.sketch.solid_boundary(last, solid::REPORT_UNIT)),
        1000.0
    );
}

#[test]
fn depth_is_a_positive_magnitude_even_when_it_is_an_expression() {
    for depth in ["-5mm", "2mm - 7mm", "0mm"] {
        let (p, _) = syntax::parse(&source(12).replace("-5mm", depth));
        let e = program::elaborate(&p);
        assert!(
            e.errors().any(|d| d.message.contains("positive magnitude")),
            "{:?}",
            e.diags
        );
    }
    assert_eq!(
        positions(&source(12).replace("depth: -5mm", "from: 0mm, to: 5mm"))["result.volume"],
        500.0
    );
    let src = source(12)
        .replace("unit mm", "unit mm\nparam d = -5mm")
        .replace("depth: -5mm", "depth: d");
    let (p, _) = syntax::parse(&src);
    assert!(!program::elaborate(&p).ok());
}

#[test]
fn geometry_validation_uses_the_solved_shape_instead_of_its_hint() {
    // p2 starts at p1. Its dimensions move it to the fourth corner before validation runs.
    let e = read("unit mm\npoint p0 hint(x: 0, y: 0)\nground p0\npoint p1 hint(x: 10, y: 0)\nground p1\npoint p2 hint(x: 10, y: 0)\npoint p3 hint(x: 0, y: 10)\nground p3\np0 distance(10mm, along: x) p2\np0 distance(10mm, along: y) p2\nface f(p0,p1,p2,p3, -> close)\nsolid result(f, depth: 5mm)\n");
    assert!(program::solid_diagnostics(&e.sketch, &e.map).is_empty());
    near(
        report::positions(&e.sketch, &e.map)
            .into_iter()
            .find(|(n, _)| n == "result.volume")
            .unwrap()
            .1,
        500.0,
        1e-7,
    );
}

#[test]
fn small_profiles_remain_closed_regions() {
    for (i, expected, tolerance) in [
        (13, 2.4e-16, 1e-10),
        (14, std::f64::consts::PI * 1e-12 * 5.0, 0.002),
    ] {
        let e = read(&source(i));
        let si = e.map.ent_named("result").unwrap().i();
        let pieces = e.sketch.solid_boundary(si, solid::REPORT_UNIT);
        near(mesh::volume(&pieces), expected, tolerance);
        closed(&e.sketch.solid_mesh(si, 0.0).positions);
        assert!(mesh::area(&pieces) > 0.0);
    }
}

#[test]
fn translated_small_prism_volume_uses_a_local_reference() {
    let p = positions(&source(15));
    near(p["result.volume"], 0.00024, 1e-8);
    let e = read(&source(15));
    let si = e.map.ent_named("result").unwrap().i();
    closed(&e.sketch.solid_mesh(si, 0.0).positions);
}
