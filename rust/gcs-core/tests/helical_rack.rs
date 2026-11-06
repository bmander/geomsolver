//! **A helical rack cuts a helical gear as a screw's sheet** (issue #826): `rack_cut`'s gear with
//! its rack's tooth leaned 15° about the radius, an undrafted prism extruded square to its leaned
//! plane, so it crosses the blank aslant as a helical rack's teeth do.  The prism is the same set
//! slid along its own extrusion, so its section by any plane square to the blank's axis is the
//! base's section slid along the rack's line, and the roll carries a section slid along its line to
//! its own envelope turned about the axis: the sweep's side is the transverse envelope carried up a
//! helix of `tan β / r` a millimetre, as a helical gear's flank is its transverse involute carried up
//! its helix.  It is built so (`brep::swept::extruded::Screwed`), not traced, and its STEP says
//! the flank as one net of a few poles along the helix, cut by the blank's faces along iso lines.
//!
//! What is removed is checked against arithmetic, as `rack_cut`'s is: every section square to the
//! axis is the transverse section's space turned, so the space is the blank's thickness times the
//! area the transverse tooth (the leaned tooth's `v` stretched by `1 / cos β`) removes.

use gcs_core::solid::admission::{admit_body, Options};

use crate::common::build;
use crate::rack_cut::{gear, rack_space, SPACE};
use std::f64::consts::PI;

/// How far the rack's tooth leans (degrees).
const LEAN: f64 = 15.;

/// `rack_cut`'s tooth space with its tooth leaned `LEAN` about the radius: drawn in the plane over
/// the page's `x` and the page's `v` tipped toward the page's normal, extruded square to that plane
/// from 9 under it to 3 over, past the blank (6 thick, behind the page) by more than the lean
/// carries a cap's corner.
fn leaned(src: &str) -> String {
    let (s, c) = LEAN.to_radians().sin_cos();
    let tooth = "\
t0 := point hint((18, -0.84))
t1 := point hint((18, 0.84))
t2 := point hint((24, 3.03))
t3 := point hint((24, -3.03))
fix((18, -0.842856)) t0
fix((18, 0.842856)) t1
fix((24, 3.026666)) t2
fix((24, -3.026666)) t3
}
tooth := face(t0, t1, t2, t3, -> close)
construction rack_tooth := solid(std.solids.Prism(tooth, from: -8mm, to: 2mm))
";
    assert!(src.contains(tooth));
    src.replace(tooth, &format!("\
}}
lean := axis
fix(dir == (0, {:.15}, {c:.15})) lean
inclined := plane(u: std.x, v: lean)
fix(origin == (0, 0, 0)) inclined
in inclined {{
t0 := point hint((18, -0.84))
t1 := point hint((18, 0.84))
t2 := point hint((24, 3.03))
t3 := point hint((24, -3.03))
fix((18, -0.842856)) t0
fix((18, 0.842856)) t1
fix((24, 3.026666)) t2
fix((24, -3.026666)) t3
}}
tooth := face(t0, t1, t2, t3, -> close)
construction rack_tooth := solid(std.solids.Prism(tooth, from: -9mm, to: 3mm))
", -s))
}

/// The transverse tooth, clockwise: `rack_cut`'s with its coordinate along the rack's line
/// stretched by `1 / cos β`, as a plane square to the blank's axis cuts the leaned prism.
fn transverse() -> [(f64, f64); 4] {
    let c = LEAN.to_radians().cos();
    [(18., -0.842856 / c), (18., 0.842856 / c), (24., 3.026666 / c), (24., -3.026666 / c)]
}

/// **The helical rack's sheet is its transverse envelope carried up a helix**, not traced, and
/// agrees with the traced sheet: every contact the trace found in the blank (each a root of the
/// contact equation, exact) lies on it within its own fit, with its normal.
#[test]
fn a_helical_racks_sheet_is_its_transverse_envelope_screwed() {
    use gcs_core::brep::sweep::{self, sheet::feet_near, Say};
    let mut e = build(&leaned(SPACE));
    let r = gcs_core::solve::solve(&mut e.sketch, gcs_core::solve::SolveOpts::default());
    assert!(r.success, "{}", r.message);
    let gear = e.map.ent_named("gear").unwrap().i();
    let admission = admit_body(&e.sketch, gear, &Options::default()).unwrap_or_else(|e| panic!("refused: {e}"));
    let recipe = gcs_core::solid::cad::recipe_static(&e.sketch, gear).unwrap();
    let lines = std::sync::Mutex::new(Vec::<String>::new());
    let say = Say { stage: &|l: &str| { eprintln!("{l}"); lines.lock().unwrap().push(l.to_string()) }, mark: &|_| {} };
    let refused = |r: gcs_core::solid::export::ExportRefusal| -> ! { panic!("refused at {:?}: {}", r.stage, r.message) };
    let prepared = sweep::prepare(&e.sketch, gear, &recipe, &admission, &say).unwrap_or_else(|r| refused(r));
    let built = sweep::sheet(&prepared, 0, None, &say).unwrap_or_else(|r| refused(r));
    assert!(lines.lock().unwrap().iter().any(|l| l.contains("carried up its helix")), "the sheet was not screwed");
    // turned tan β / r a millimetre up, one way or the other as the axis is read: the helix of the
    // pitch circle's lead
    let want = (LEAN.to_radians().tan() / 20.).to_degrees();
    assert!(lines.lock().unwrap().iter().any(|l| l.contains(&format!("{want:.4} degrees a millimetre up"))),
            "the helix is not tan β / r, {want:.4} degrees a millimetre");
    let traced = sweep::traced(&prepared, 0, None, &say).unwrap_or_else(|r| refused(r));
    // the blank in the export's millimetres: radius 22 about y, from 0 to 6 along it
    let inside = |p: &[f64; 3]| p[0].hypot(p[2]) < 22. && p[1] > 0. && p[1] < 6.;
    let angle = |a: [f64; 3], b: [f64; 3]| (a[0] * b[0] + a[1] * b[1] + a[2] * b[2]).abs().min(1.).acos().to_degrees();
    let (points, normals): (Vec<[f64; 3]>, Vec<[f64; 3]>) = traced.sheet.points.iter().zip(&traced.sheet.normals)
        .chain(traced.sheet.withheld.iter().zip(&traced.sheet.withheld_normals))
        .filter(|(p, _)| inside(p)).map(|(p, n)| (*p, *n)).unzip();
    assert!(points.len() > 500, "only {} traced contacts in the blank", points.len());
    let feet = feet_near(&built.face, &points, &vec![[f64::NAN; 2]; points.len()], 0.);
    let (mut gap, mut turn) = (0f64, 0f64);
    for (foot, n) in feet.iter().zip(&normals) {
        let (m, d) = foot.expect("a foot on the sheet");
        (gap, turn) = (gap.max(d), turn.max(angle(m, *n)));
    }
    eprintln!("{} traced contacts within {gap:.2e} mm of the screwed sheet, normals within {turn:.2e} degrees (its error {:.2e})",
        points.len(), built.error);
    assert!(built.error < 1e-4, "the screwed sheet is {:.2e} mm off its curve", built.error);
    assert!(gap <= built.error + 1e-7 && turn < 1e-2,
            "the traced contacts lie {gap:.2e} mm and {turn:.2e} degrees off the screwed sheet");
    // its normals are the helicoid's, not the section's: leaned out of the plane square to the axis
    let lean = built.sheet.normals.iter().map(|n| n[1].abs()).fold(0., f64::max);
    assert!(lean > 0.1, "the sheet's normals lie square to the axis (at most {lean:.3} along it)");
}

/// **The tooth space is built**: what it removes is the transverse space's area by arithmetic times
/// the blank's thickness, to the screwed sheet's fit, and its STL is written only where the
/// material field agrees with it.
#[test]
fn a_helical_racks_tooth_space_is_built() {
    use gcs_core::brep::{export, sweep::Say};
    let mut e = build(&leaned(SPACE));
    let r = gcs_core::solve::solve(&mut e.sketch, gcs_core::solve::SolveOpts::default());
    assert!(r.success, "{}", r.message);
    let gear = e.map.ent_named("gear").unwrap().i();
    let say = Say { stage: &|l: &str| eprintln!("{l}"), mark: &|_| {} };
    let exact = export::exact(&e.sketch, gear, None, None, &say).unwrap_or_else(|r| panic!("refused at {:?}: {}", r.stage, r.message));
    assert!(exact.swept && exact.pattern.is_none());
    let removed = PI * 22. * 22. * 6. - gcs_core::brep::props::volume(&exact.solid);
    let want = 6. * rack_space(&transverse(), 20., [18., 22.], [-PI / 3., PI / 3.], 400, 4000);
    eprintln!("removed {removed:.5} mm³, the space being {want:.5}");
    assert!((removed - want).abs() < 0.01, "removed {removed:.5} mm³, the space being {want:.5}");
    export::stl(&e.sketch, gear, &exact, None, &say).unwrap_or_else(|r| panic!("the STL refused at {:?}: {}", r.stage, r.message));
}

/// **A helical gear's STEP is about its spur twin's size** (#826, where it was 8× it): the whole
/// 20-tooth gear, built as one sector patterned, written and read back; each flank a net of a few
/// poles along its helix, and each blank face's cut of it the net's iso line, a line in the
/// face's parameters.  Measured against the same gear cut by the unleaned rack.
#[test]
fn a_helical_gears_step_is_about_its_spur_twins_size() {
    use gcs_core::brep::{export, sweep::Say};
    let say = Say { stage: &|l: &str| eprintln!("{l}"), mark: &|_| {} };
    let written = |src: &str| -> (export::Exact, String) {
        let mut e = build(src);
        let r = gcs_core::solve::solve(&mut e.sketch, gcs_core::solve::SolveOpts::default());
        assert!(r.success, "{}", r.message);
        let gear = e.map.ent_named("gear").unwrap().i();
        let exact = export::exact(&e.sketch, gear, None, None, &say).unwrap_or_else(|r| panic!("refused at {:?}: {}", r.stage, r.message));
        let text = export::step(&exact, "gear", None, &say).unwrap_or_else(|r| panic!("the STEP refused at {:?}: {}", r.stage, r.message));
        export::stl(&e.sketch, gear, &exact, None, &say).unwrap_or_else(|r| panic!("the STL refused at {:?}: {}", r.stage, r.message));
        (exact, text)
    };
    let (helical, text) = written(&leaned(&gear(20)));
    let (_, spur) = written(&gear(20));
    assert!(helical.pattern.is_some(), "built whole");
    let removed = PI * (22. * 22. - 6. * 6.) * 6. - gcs_core::brep::props::volume(&helical.solid);
    let want = 20. * 6. * rack_space(&transverse(), 20., [18., 22.], [-PI / 3., PI / 3.], 400, 4000);
    assert!((removed - want).abs() < 0.2, "removed {removed:.5} mm³, the spaces being {want:.5}");
    let entities = |t: &str| t.lines().filter(|l| l.starts_with('#')).count();
    let (h, s) = (entities(&text), entities(&spur));
    eprintln!("the helical gear's STEP: {h} entities, {} bytes; its spur twin's: {s}, {} bytes", text.len(), spur.len());
    assert!(h <= 2 * s, "the helical gear's STEP has {h} entities, its spur twin's {s}");
    // the flanks' nets few poles along the helix, and no curve of many hundred poles
    for l in text.lines().filter(|l| l.contains("B_SPLINE_SURFACE_WITH_KNOTS('',")) {
        let inner = &l[l.find("((").unwrap() + 1..];
        let columns = inner.split("),(").next().unwrap().split(',').count();
        assert!(columns <= 16, "a flank's net has {columns} poles along its helix: {}", &l[..l.len().min(200)]);
    }
    let longest = text.lines().filter(|l| l.contains("B_SPLINE_CURVE_WITH_KNOTS('',")).map(|l| {
        let inner = l.split_once("B_SPLINE_CURVE_WITH_KNOTS('',").unwrap().1;
        inner.split_once(",(").map_or(0, |(_, rest)| rest.split(')').next().unwrap().split(',').count())
    }).max().unwrap_or(0);
    assert!(longest < 120, "a curve of {longest} poles");
}
