//! Issue #51: frozen inputs plus geometric and output-contract checks independent of the bash runner.
use gcs_core::{
    clear, constraints::SolidWord, diagnose, gltf, hidden, json::Json, mesh, program, solid, solve,
    syntax,
};

fn source(name: &str) -> String {
    std::fs::read_to_string(
        std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("tests/fixtures/solid_issue51")
            .join(format!("{name}.sv")),
    )
    .unwrap()
}
fn read(src: &str) -> program::Elaborated {
    let (p, errors) = syntax::parse(src);
    assert!(errors.is_empty(), "{errors:?}");
    let mut e = program::elaborate(&p);
    assert!(e.ok(), "{:?}", e.diags);
    assert!(solve::solve(&mut e.sketch, solve::SolveOpts::default()).success);
    e
}
fn case(name: &str) -> program::Elaborated {
    read(&source(name))
}
fn si(e: &program::Elaborated, name: &str) -> usize {
    e.map.ent_named(name).unwrap().i()
}
fn close(a: f64, b: f64) {
    assert!((a - b).abs() <= b.abs().max(1e-30) * 1e-7, "{a} != {b}");
}
fn claim(e: &program::Elaborated, word: SolidWord, gap: f64) -> clear::Verdict {
    clear::judge(
        &e.sketch,
        word,
        si(e, "result"),
        si(e, "other"),
        gap,
        solid::REPORT_UNIT,
    )
}

#[test]
fn crossing_bars_are_material_interference_even_without_contained_vertices() {
    let e = case("cross_clear");
    let v = claim(&e, SolidWord::Clear, 0.1);
    assert_eq!(v.holds, Some(false));
    close(v.measured, -1.0);
    close(v.tolerance, 0.0);
    // The answer is symmetric and ignores an unrelated point.
    let reversed = clear::judge(
        &e.sketch,
        SolidWord::Clear,
        si(&e, "other"),
        si(&e, "result"),
        0.1,
        solid::REPORT_UNIT,
    );
    close(reversed.measured, v.measured);
    let unrelated = read(
        &(source("cross_clear") + "point remote hint(x:1000000000,y:1000000000)\nground remote\n"),
    );
    close(claim(&unrelated, SolidWord::Clear, 0.1).measured, -1.0);
    close(
        claim(&case("cross_clear_control"), SolidWord::Clear, 0.1).measured,
        2.0,
    );
}

#[test]
fn penetration_is_a_geometric_thickness_with_a_reported_error_bound() {
    let e = case("penetration_10");
    let v = claim(&e, SolidWord::Clear, 0.0);
    close(v.measured, -5.0);
    close(v.tolerance, 0.0);
    assert_eq!(v.holds, Some(false));
    let equal = read(
        &source("penetration_10")
            .replace("x: 5,", "x: 0,")
            .replace("x: 15,", "x: 10,"),
    );
    close(claim(&equal, SolidWord::Clear, 0.0).measured, -10.0);
}

#[test]
fn unrelated_points_do_not_change_boolean_boundaries_or_containment() {
    for name in ["boolean_extent_0", "boolean_extent_1000000000.0"] {
        let e = case(name);
        close(
            mesh::volume(
                &e.sketch
                    .solid_boundary(si(&e, "result"), solid::REPORT_UNIT),
            ),
            928.0,
        );
    }
    for name in ["containment_extent_0", "containment_extent_1000000000"] {
        assert_eq!(
            claim(&case(name), SolidWord::Inside, 0.0).holds,
            Some(false)
        );
    }
    // Also exercise a cache miss after unrelated geometry changes.
    let mut e = case("boolean_extent_0");
    let i = si(&e, "result");
    let before = e.sketch.solid_boundary(i, solid::REPORT_UNIT);
    e.sketch.point(1e12, 1e12, true, "remote");
    e.sketch.solid_cache.borrow_mut().clear();
    let after = e.sketch.solid_boundary(i, solid::REPORT_UNIT);
    close(mesh::volume(&before), mesh::volume(&after));
}

#[test]
fn negative_gaps_cannot_override_inside_or_disjointness() {
    let e = case("negative_fits_outside");
    assert_eq!(claim(&e, SolidWord::Fits, -2.0).holds, Some(false));
    let e = case("penetration_10");
    assert_eq!(claim(&e, SolidWord::Clear, -100.0).holds, Some(false));
}

#[test]
fn solid_claim_arguments_are_checked_instead_of_discarded() {
    for src in [
        source("arguments_inside(1mm)"),
        source("arguments_clear(1mm,2mm)"),
        source("arguments_clear(1mm,2mm)").replace("clear(1mm,2mm)", "fits(1mm,2mm)"),
    ] {
        let (p, errors) = syntax::parse(&src);
        assert!(errors.is_empty());
        let e = program::elaborate(&p);
        assert!(e.errors().any(|d| d.code == program::Code::E040));
    }
    assert_eq!(
        diagnose::judge_solids(&case("claim_no_arguments_control").sketch)[0].holds,
        Some(true)
    );
}

#[test]
fn failed_sweep_poses_are_disclosed_and_cannot_certify_clearance() {
    let e = case("sweep_impossible");
    let v = diagnose::judge_solids(&e.sketch).remove(0);
    assert_eq!(v.samples, 37);
    assert_eq!(v.failed_samples.len(), 37);
    assert_eq!(v.holds, None);
    assert!(v.measured.is_nan());
    assert_eq!(v.worst, None);
    let e = read(&source("sweep_impossible").replace("(2mm,3mm)", "(0.1mm,2mm)"));
    let v = diagnose::judge_solids(&e.sketch).remove(0);
    assert!(!v.failed_samples.is_empty() && v.failed_samples.len() < 37);
    assert_eq!(v.holds, None);
    assert!(v.measured.is_finite());
}

#[test]
fn sweep_bounds_use_the_inferred_variable_dimension_and_user_units() {
    let (p, _) = syntax::parse(&source("sweep_dimensional_error"));
    assert!(program::elaborate(&p)
        .errors()
        .any(|d| d.code == program::Code::E103));
    let src = source("sweep_possible")
        .replace(
            "circle c(center:o)",
            "point q hint(x:1,y:0)\nground q\ncircle c(center:o)",
        )
        .replace(
            "o distance(reach,along:x) p",
            "line datum(o,q)\nline radial(o,p)\ndatum angle(reach) radial",
        )
        .replace("(0.1mm,0.9mm)", "(10deg,30deg)");
    let e = read(&src);
    let sweep = e.sketch.solid_claims[0].over.as_ref().unwrap();
    close(sweep.from, 10.0);
    close(sweep.to, 30.0);
    let v = diagnose::judge_solids(&e.sketch).remove(0);
    assert!(v.failed_samples.is_empty());
    assert_eq!(v.holds, Some(true));
}

#[test]
fn sampling_method_and_actual_attempt_count_are_serialized() {
    let mut e = case("sampling_disclosure");
    let d = diagnose::diagnose(&mut e.sketch, Default::default());
    let j = gcs_core::report::diagnosis_json(&e.sketch, &d);
    let c = &j.get("solidClaims").unwrap().arr()[0];
    assert_eq!(c.get("method").unwrap().as_str(), "sampling");
    assert_eq!(c.get("samples").unwrap().as_i64(), 37);
    assert!(c.get("failedSamples").unwrap().arr().is_empty());
    let e = case("single_pose_control");
    let v = diagnose::judge_solids(&e.sketch).remove(0);
    assert_eq!(v.samples, 0);
}

#[test]
fn input_edge_names_cannot_steal_sweep_cap_names() {
    for name in ["near", "far"] {
        let e = read(&source("cap_collision").replace("near", name));
        let errors = program::solid_diagnostics(&e.sketch, &e.map);
        assert!(
            errors.iter().any(|d| d.message.contains("collides")),
            "{errors:?}"
        );
    }
    let e = case("cap_collision_control");
    assert!(program::solid_diagnostics(&e.sketch, &e.map).is_empty());
    let src=source("cap_collision_control").replace("bottom","start")
        .replace("solid result(f,depth: 2mm)","point ax0 hint(x:-1,y:0)\nground ax0\npoint ax1 hint(x:-1,y:5)\nground ax1\nline axis(ax0,ax1)\nsolid result(f,about:axis,sweep:90deg)");
    let e = read(&src);
    assert!(!program::solid_diagnostics(&e.sketch, &e.map).is_empty());
}

#[test]
fn tiny_revolutions_keep_relative_angular_accuracy() {
    let e = case("small_revolve_1e-06");
    let got = mesh::volume(
        &e.sketch
            .solid_boundary(si(&e, "result"), solid::REPORT_UNIT),
    );
    let expected = std::f64::consts::TAU * 12.0 * 24.0 * 1e-18;
    assert!(
        (got / expected - 1.0).abs() < 0.002,
        "{got} versus {expected}"
    );
    let m = e.sketch.solid_mesh(si(&e, "result"), solid::REPORT_UNIT);
    assert!(m.positions.len() / 9 > 100);
}

#[test]
fn a_thin_occluder_hides_the_same_edges_as_a_thick_one() {
    for name in ["hidden_thin_0.1", "hidden_thin_1"] {
        let e = case(name);
        let strokes = hidden::layout(&e.sketch, solid::REPORT_UNIT);
        let boss: Vec<_> = strokes
            .iter()
            .filter(|s| s.path.starts_with("boss."))
            .collect();
        assert_eq!(boss.len(), 4);
        assert!(boss.iter().all(|s| s.hidden));
    }
}

#[test]
fn a_section_removes_geometry_in_front_and_keeps_geometry_behind() {
    for (name, expected) in [("section_0.5", 0), ("section_3", 4)] {
        let e = case(name);
        let strokes = hidden::layout(&e.sketch, solid::REPORT_UNIT);
        assert_eq!(
            strokes
                .iter()
                .filter(|s| s.path.starts_with("boss."))
                .count(),
            expected
        );
        assert_eq!(
            strokes
                .iter()
                .filter(|s| s.path.starts_with("stock."))
                .count(),
            4
        );
    }
}

#[test]
fn derived_strokes_are_invariant_under_sheet_translation() {
    let a = case("view_layout_0");
    let b = case("view_layout_100000000.0");
    let a = hidden::layout(&a.sketch, solid::REPORT_UNIT);
    let b = hidden::layout(&b.sketch, solid::REPORT_UNIT);
    assert_eq!(a.len(), 4);
    assert_eq!(b.len(), 4);
    for (a, b) in a.iter().zip(&b) {
        assert_eq!(a.hidden, b.hidden);
        assert_eq!(a.pts.len(), b.pts.len());
        for (a, b) in a.pts.iter().zip(&b.pts) {
            assert!((b.0 - a.0 - 1e8).abs() < 1e-6);
            assert!((b.1 - a.1 - 1e8).abs() < 1e-6);
        }
    }
}

#[test]
fn dimensions_only_describe_surviving_round_features() {
    for name in ["ghost_dimension_through", "ghost_dimension_none"] {
        let e = case(name);
        let dims = hidden::generated(&e.sketch, solid::REPORT_UNIT);
        assert_eq!(dims.len(), 2);
        assert!(dims.iter().all(|(_, d)| !d.round));
    }
    let e = read(&source("ghost_dimension_through").replace("x: 20, y: 20", "x: 5, y: 5"));
    let dims = hidden::generated(&e.sketch, solid::REPORT_UNIT);
    assert_eq!(
        dims.iter()
            .filter(|(_, d)| d.round && d.value == 4.0)
            .count(),
        1
    );
}

#[test]
fn glb_preserves_local_triangles_and_world_placement_at_large_coordinates() {
    for name in ["glb_collapse", "glb_control"] {
        let e = case(name);
        let i = si(&e, "result");
        let m = e.sketch.solid_mesh(i, solid::REPORT_UNIT);
        let (doc, bytes) = gltf::build(&[("result".into(), m.clone())], 0.001, Some("mm"));
        let translation = doc.get("nodes").unwrap().arr()[0]
            .get("translation")
            .unwrap()
            .arr();
        let translation = [
            translation[0].as_f64(),
            translation[1].as_f64(),
            translation[2].as_f64(),
        ];
        let mut points = Vec::new();
        for i in 0..m.positions.len() / 3 {
            let p = std::array::from_fn::<_, 3, _>(|k| {
                f32::from_le_bytes(
                    bytes[(i * 3 + k) * 4..(i * 3 + k + 1) * 4]
                        .try_into()
                        .unwrap(),
                ) as f64
            });
            for k in 0..3 {
                assert!((p[k] + translation[k] - m.positions[i * 3 + k] * 0.001).abs() < 1e-9);
            }
            points.push(p);
        }
        for t in points.chunks_exact(3) {
            let a = std::array::from_fn::<_, 3, _>(|k| t[1][k] - t[0][k]);
            let b = std::array::from_fn::<_, 3, _>(|k| t[2][k] - t[0][k]);
            let cross = [
                a[1] * b[2] - a[2] * b[1],
                a[2] * b[0] - a[0] * b[2],
                a[0] * b[1] - a[1] * b[0],
            ];
            assert!(cross.iter().any(|&x| x != 0.0));
        }
        for a in doc.get("accessors").unwrap().arr().iter().step_by(2) {
            let off = a.get("byteOffset").unwrap().as_i64() as usize;
            let count = a.get("count").unwrap().as_i64() as usize;
            for k in 0..3 {
                let vals = (0..count).map(|i| {
                    f32::from_le_bytes(
                        bytes[off + i * 12 + k * 4..off + i * 12 + k * 4 + 4]
                            .try_into()
                            .unwrap(),
                    ) as f64
                });
                let min = vals.clone().fold(f64::INFINITY, f64::min);
                let max = vals.fold(f64::NEG_INFINITY, f64::max);
                assert_eq!(a.get("min").unwrap().arr()[k], Json::Num(min));
                assert_eq!(a.get("max").unwrap().arr()[k], Json::Num(max));
            }
        }
    }
}

fn box_source(name: &str, x: f64, y: f64, w: f64, h: f64, lo: f64, hi: f64) -> String {
    let mut src = String::new();
    for (i, (x, y)) in [(x, y), (x + w, y), (x + w, y + h), (x, y + h)]
        .into_iter()
        .enumerate()
    {
        src += &format!("point {name}p{i} hint(x:{x},y:{y})\nground {name}p{i}\n");
    }
    src += &format!(
        "solid {name}(face({name}p0,{name}p1,{name}p2,{name}p3,-> close),from:{lo}mm,to:{hi}mm)\n"
    );
    src
}

#[test]
fn interference_checks_respect_voids_and_disconnected_material() {
    let src = source("boolean_extent_0") + &box_source("other", 3.0, 3.0, 1.0, 1.0, -5.5, -4.5);
    let e = read(&src);
    let v = claim(&e, SolidWord::Clear, 0.1);
    assert_eq!(v.holds, Some(true));
    close(v.measured, 0.5);
    let src = "unit mm\n".to_string()
        + &box_source("left", 0.0, 0.0, 1.0, 1.0, -1.0, 0.0)
        + &box_source("right", 8.0, 0.0, 1.0, 1.0, -1.0, 0.0)
        + "solid result(left)\nright on result\nsolid other(result)\n";
    let e = read(&src);
    let v = claim(&e, SolidWord::Clear, 0.0);
    assert_eq!(v.holds, Some(false));
    assert!((-v.measured - 1.0).abs() <= v.tolerance + 1e-7);
    assert!(v.tolerance < 0.001, "{:?}", v);
}

#[test]
fn a_section_clips_crossing_edges_and_removes_occlusion_by_discarded_material() {
    // A cut through the plate still has a square section. The boss ahead of it must not
    // occlude any retained edge, and a cut behind the whole part is empty.
    for (cut, count) in [(-0.5, 4), (-2.0, 0)] {
        let e = read(&source("section_0.5").replace("offset:0.5mm", &format!("offset:{cut}mm")));
        let strokes = hidden::layout(&e.sketch, solid::REPORT_UNIT);
        assert_eq!(strokes.len(), count, "cut {cut}: {strokes:?}");
        assert!(strokes.iter().all(|s| !s.hidden));
    }
}
