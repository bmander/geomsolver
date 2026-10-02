//! Directed guides unify a repeated section and an explicit two-section transition.
use gcs_core::{
    io,
    model::EntRef,
    program,
    solid::{ApproximationPolicy::Report, WorldPoint},
    syntax,
};

fn read(src: &str) -> program::Elaborated {
    let (p, errors) = syntax::parse(src);
    assert!(errors.is_empty(), "{errors:?}");
    let e = program::elaborate(&p);
    assert!(e.ok(), "{:?}", e.diags);
    e
}

const SECTIONS: &str = "unit mm
o := point hint(x: 0, y: 0)
q := point hint(x: 1, y: 0)
front := plane(origin: o, toward: q)
back := plane(origin: o, toward: q, from: front, offset: -20mm)
top := plane(origin: o, toward: q, from: front, fold: 0deg)
in top {
  a := point hint(x: 0, y: 0)
  b := point hint(x: 0, y: 20)
  guide := line(a, b)
}
in front {
  c := point hint(x: 0, y: 0)
  outer := circle(center: c) hint(r: 10)
  inner := circle(center: c) hint(r: 5)
  section := face(outer, holes: inner)
}
in back {
  ec := point hint(x: 0, y: 0)
  eo := circle(center: ec) hint(r: 6)
  ei := circle(center: ec) hint(r: 3)
  end_section := face(eo, holes: ei)
}
";

fn evaluated(e: &program::Elaborated, name: &str) -> std::rc::Rc<gcs_core::solid::EvaluatedSolid> {
    e.sketch
        .evaluated_solid(e.map.ent_named(name).unwrap().i(), Report)
        .unwrap()
}

#[test]
fn sweep_and_loft_preserve_holes_caps_and_closed_meshes() {
    let e = read(&format!("{SECTIONS}sweep := solid(section, along: guide)\nloft := solid(section, end_section, along: guide)\n"));
    for (name, want) in [
        ("sweep", 1500.0 * std::f64::consts::PI),
        ("loft", 980.0 * std::f64::consts::PI),
    ] {
        let s = evaluated(&e, name);
        assert!(
            (s.volume() - want).abs() / want < 0.003,
            "{name}: {} != {want}",
            s.volume()
        );
        assert!(!s.contains_world(WorldPoint([0.0, 10.0, 0.0])));
        assert!(s.contains_world(WorldPoint([7.0, 10.0, 0.0])));
        for face in ["start", "end", "outer", "inner"] {
            assert!(
                s.surviving_faces().contains(&format!("{name}.{face}")),
                "{face}"
            );
        }
        assert_eq!(super::solid::unpaired(s.mesh()), 0, "{name}");
        assert!(s.stl().unwrap().len() > 84);
    }
}

#[test]
fn guide_and_end_section_changes_invalidate_cache_and_copy_retains_dependencies() {
    let mut e = read(&format!(
        "{SECTIONS}body := solid(section, end_section, along: guide)\n"
    ));
    let before = evaluated(&e, "body").volume();
    let r = e.sketch.circles[e.map.ent_named("eo").unwrap().i()].radius as usize;
    e.sketch.params[r].value = 8.0;
    assert!(evaluated(&e, "body").volume() > before);
    let copied = io::copy(&e.sketch, &[e.map.ent_named("body").unwrap()]);
    assert_eq!(copied.solids.len(), 1);
    assert!(
        (copied.evaluated_solid(0, Report).unwrap().volume() - evaluated(&e, "body").volume())
            .abs()
            < 1e-7
    );
    let mut dest = gcs_core::model::Sketch::new();
    io::paste(&mut dest, &copied, 13.0, 27.0);
    assert!(
        (dest.evaluated_solid(0, Report).unwrap().volume() - evaluated(&e, "body").volume()).abs()
            < 1e-7
    );
    // Guide placement is a dependency even though the solid owns no solver parameters.
    let b = e.map.ent_named("b").unwrap().i();
    let py = e.sketch.point_params(b)[1] as usize;
    e.sketch.params[py].value = 30.0;
    assert!(e
        .sketch
        .evaluated_solid(e.map.ent_named("body").unwrap().i(), Report)
        .unwrap_err()
        .contains("end section"));
}

#[test]
fn malformed_guides_and_mismatched_sections_are_diagnosed() {
    for (old, new, needle) in [
        ("y: 20", "y: 0", "distinct"),
        ("offset: -20mm", "offset: -21mm", "end section"),
        ("guide := line(a, b)", "guide := line(b, a)", "start section"),
        (
            "end_section := face(eo, holes: ei)",
            "end_section := face(eo)",
            "numbers of holes",
        ),
    ] {
        let src =
            format!("{SECTIONS}body := solid(section, end_section, along: guide)\n").replace(old, new);
        let (p, err) = syntax::parse(&src);
        assert!(err.is_empty());
        let e = program::elaborate(&p);
        let errors = program::solid_diagnostics(&e.sketch, &e.map);
        assert!(
            errors.iter().any(|d| d.message.contains(needle)),
            "{needle}: {errors:?}"
        );
    }
    for tail in [
        ", depth: 3mm",
        ", through: body",
        ", about: guide",
        ", along: guide",
    ] {
        let (_, errs) = syntax::parse(&format!(
            "{SECTIONS}body := solid(section, along: guide{tail})\n"
        ));
        assert!(!errs.is_empty());
    }
}

#[test]
fn arc_sweep_follows_solved_radius_and_transports_the_profile() {
    let src = "unit mm
o := point hint(x: 0, y: 0)
q := point hint(x: 1, y: 0)
front := plane(origin: o, toward: q)
top := plane(origin: o, toward: q, from: front, fold: 0deg)
in top {
 center := point hint(x: 0, y: 0)
 a := point hint(x: 30, y: 0)
 b := point hint(x: 0, y: 30)
 guide := arc(center: center, start: a, end: b) hint(r: 30)
}
in front {
 c := point hint(x: 30, y: 0)
 outer := circle(center: c) hint(r: 5)
 inner := circle(center: c) hint(r: 3)
 section := face(outer, holes: inner)
}
body := solid(section, along: guide)
";
    let mut e = read(src);
    let s = e
        .sketch
        .evaluated_solid(0, gcs_core::solid::ApproximationPolicy::View { unit: 0.2 })
        .unwrap();
    let want = 0.5 * std::f64::consts::PI * 30.0 * 16.0 * std::f64::consts::PI;
    assert!(
        (s.volume() - want).abs() / want < 0.004,
        "{} != {want}",
        s.volume()
    );
    assert_eq!(super::solid::unpaired(s.mesh()), 0);
    let k = 0.5_f64.sqrt();
    assert!(!s.contains_world(WorldPoint([30.0 * k, 30.0 * k, 0.0])));
    assert!(s.contains_world(WorldPoint([34.0 * k, 34.0 * k, 0.0])));
    let g = e.map.ent_named("guide").unwrap().i();
    e.sketch.params[e.sketch.arcs[g].radius as usize].value = 31.0;
    assert!(e.sketch.evaluated_solid(0, Report).is_err());
    assert!(io::copy(&read(src).sketch, &[EntRef::solid(0)])
        .evaluated_solid(0, gcs_core::solid::ApproximationPolicy::View { unit: 0.2 })
        .is_ok());
}

#[test]
fn reversed_and_rotated_guides_keep_the_same_loft_and_source_round_trips() {
    let src = format!("{SECTIONS}body := solid(end_section, section, along: guide)\n")
        .replace("guide := line(a, b)", "guide := line(b, a)");
    let mut e = read(&src);
    let want = evaluated(&e, "body").volume();
    let (mut p, errors) = syntax::parse("body := solid(end_section, section, along: guide)\n");
    assert!(errors.is_empty());
    let printed = syntax::render_flat(&mut p).unwrap();
    let rebuilt = format!(
        "{}{printed}",
        SECTIONS.replace("guide := line(a, b)", "guide := line(b, a)")
    );
    assert!((evaluated(&read(&rebuilt), "body").volume() - want).abs() < 1e-7);
    let committed = gcs_core::edit::commit_seeds(&e, &e.sketch, &e.program);
    assert!(committed
        .text
        .contains("body := solid(end_section, section, along: guide)"));
    let k = 0.5_f64.sqrt();
    let rotate = |p: [f64; 3]| [p[0], k * (p[1] - p[2]), k * (p[1] + p[2])];
    for p in 0..e.sketch.planes.len() {
        let mut b = e.sketch.basis(p);
        b.u = rotate(b.u);
        b.v = rotate(b.v);
        b.o = rotate(b.o);
        for (i, delta) in [100.0, -70.0, 50.0].iter().enumerate() {
            b.o[i] += delta;
        }
        e.sketch.set_basis(p, b);
    }
    let s = evaluated(&e, "body");
    assert!((s.volume() - want).abs() < 1e-6);
    assert_eq!(super::solid::unpaired(s.mesh()), 0);
}

#[test]
fn a_straight_sweep_resizes_with_the_line_and_can_be_a_boolean_operand() {
    let src=format!("{SECTIONS}sweep := solid(section, along: guide)\ncutter := solid(face(outer), depth: 10mm)\nbody := solid(sweep)\ncutter cut body\n");
    let mut e = read(&src);
    let sweep = evaluated(&e, "sweep").volume();
    assert!((evaluated(&e, "body").volume() - sweep * 0.5).abs() < 1e-6);
    let b = e.map.ent_named("b").unwrap().i();
    let by = e.sketch.point_params(b)[1] as usize;
    e.sketch.params[by].value = 30.0;
    assert!((evaluated(&e, "sweep").volume() - sweep * 1.5).abs() < 1e-6);
    let copy = io::without(&e.sketch, &[e.map.ent_named("guide").unwrap()], &[]);
    assert!(copy
        .solids
        .iter()
        .all(|s| s.name != "body" && s.name != "sweep"));
}

#[test]
fn an_explicit_arc_end_section_changes_size_in_the_transported_frame() {
    let src = "unit mm
o := point hint(x: 0, y: 0)
q := point hint(x: 1, y: 0)
front := plane(origin: o, toward: q)
top := plane(origin: o, toward: q, from: front, fold: 0deg)
side := plane(origin: o, toward: q, u: (0,1,0), v: (0,0,1))
in top {
 center := point hint(x: 0, y: 0)
 a := point hint(x: 30, y: 0)
 b := point hint(x: 0, y: 30)
 guide := arc(center: center, start: a, end: b) hint(r: 30)
}
in front {
 c := point hint(x: 30, y: 0)
 outer := circle(center: c) hint(r: 5)
 section := face(outer)
}
in side {
 ec := point hint(x: 30, y: 0)
 eo := circle(center: ec) hint(r: 4)
 end_section := face(eo)
}
body := solid(section, end_section, along: guide)
";
    let e = read(src);
    let s = e
        .sketch
        .evaluated_solid(0, gcs_core::solid::ApproximationPolicy::View { unit: 0.1 })
        .unwrap();
    let want = 305.0 * std::f64::consts::PI.powi(2);
    assert!(
        (s.volume() - want).abs() / want < 0.004,
        "{} != {want}",
        s.volume()
    );
    assert_eq!(super::solid::unpaired(s.mesh()), 0);
}

#[test]
fn guide_constructor_references_follow_components_and_inline_faces() {
    let src = format!(
        "{SECTIONS}
component Duct(section: face, guide: line) {{ body := solid(section, along: guide) }}
x := Duct(section, guide)
inline := solid(face(outer, holes: inner), along: guide)
"
    );
    let e = read(&src);
    assert!((evaluated(&e, "x.body").volume() - evaluated(&e, "inline").volume()).abs() < 1e-7);
    for (args, needle) in [
        (
            "section, end_section, section, along: guide",
            "optional end face",
        ),
        ("section, along: outer", "directed line or circular arc"),
        ("section, along: absent", "no such guide"),
    ] {
        let (p, errs) = syntax::parse(&format!("{SECTIONS}bad := solid({args})\n"));
        assert!(errs.is_empty());
        let e = program::elaborate(&p);
        assert!(
            e.diags.iter().any(|d| d.message.contains(needle)),
            "{:?}",
            e.diags
        );
    }
}

#[test]
fn concave_sections_with_multiple_holes_have_closed_caps() {
    let src = format!(
        "{}in front {{
p0 := point hint(x: 0, y: 0)
p1 := point hint(x: 10, y: 0)
p2 := point hint(x: 10, y: 5)
p3 := point hint(x: 5, y: 5)
p4 := point hint(x: 5, y: 10)
p5 := point hint(x: 0, y: 10)
h0 := point hint(x: 7, y: 2)
h1 := point hint(x: 2, y: 7)
hole0 := circle(center: h0) hint(r: 1)
hole1 := circle(center: h1) hint(r: 1)
outline := (e0 := line(p0, p1)) -> (e1 := line(p1, p2)) -> (e2 := line(p2, p3)) -> (e3 := line(p3, p4)) -> (e4 := line(p4, p5)) -> (e5 := line(p5, p0)) -> close
section := face(outline, holes: hole0, hole1)
}}
body := solid(section, along: guide)
",
        SECTIONS.split("in front {").next().unwrap()
    );
    let e = read(&src);
    let s = evaluated(&e, "body");
    let want = (75.0 - 2.0 * std::f64::consts::PI) * 20.0;
    assert!((s.volume() - want).abs() / want < 0.001);
    for (p, inside) in [
        ([7.0, 10.0, 7.0], false),
        ([7.0, 10.0, 2.0], false),
        ([2.0, 10.0, 7.0], false),
        ([2.0, 10.0, 2.0], true),
    ] {
        assert_eq!(s.contains_world(WorldPoint(p)), inside);
    }
    assert_eq!(super::solid::unpaired(s.mesh()), 0);
    assert!(s.stl().unwrap().len() > 84);
}

#[test]
fn strip_caps_publish_the_complete_outer_and_inner_rims() {
    let e = read(&format!("{SECTIONS}body := solid(section, along: guide)\n"));
    let s = evaluated(&e, "body");
    let length: f64 = s
        .edges()
        .iter()
        .filter(|edge| !edge.smooth && edge.a[1].abs() < 1e-8 && edge.b[1].abs() < 1e-8)
        .map(|edge| {
            (0..3)
                .map(|k| (edge.b[k] - edge.a[k]).powi(2))
                .sum::<f64>()
                .sqrt()
        })
        .sum();
    let want = 2.0 * std::f64::consts::PI * (10.0 + 5.0);
    assert!((length - want).abs() < 0.02, "cap rims: {length} != {want}");
}

#[test]
fn an_opposite_end_plane_normal_does_not_twist_circular_sections() {
    let mut e = read(&format!(
        "{SECTIONS}body := solid(section, end_section, along: guide)\n"
    ));
    let before = evaluated(&e, "body").volume();
    let p = e.map.ent_named("back").unwrap().i();
    let (sn, cs) = 97.0_f64.to_radians().sin_cos();
    let b = e.sketch.basis(p);
    e.sketch.set_basis(p, gcs_core::plane::Basis { u: [cs, 0.0, sn], v: [sn, 0.0, -cs], ..b });
    let after = evaluated(&e, "body");
    assert!((after.volume() - before).abs() < 1e-7);
    assert_eq!(super::solid::unpaired(after.mesh()), 0);
}

/// Each start edge joins the end edge written in its place, in both kernels: an end section written
/// the other way round is refused by the faceted kernel and the exact one alike, rather than lofted
/// by one and refused by the other.
#[test]
fn an_end_section_written_the_other_way_round_is_refused_by_both_kernels() {
    let square = |order: &str| format!("unit mm
o := point hint(x: 0, y: 0)
q := point hint(x: 1, y: 0)
ground o
ground q
front := plane(origin: o, toward: q)
back := plane(origin: o, toward: q, from: front, offset: -20mm)
top := plane(origin: o, toward: q, from: front, fold: 0deg)
in top {{
  a := point hint(x: 0, y: 0)
  b := point hint(x: 0, y: 20)
  ground a
  ground b
  guide := line(a, b)
}}
in front {{
  s0 := point hint(x: -5, y: -5)
  s1 := point hint(x: 5, y: -5)
  s2 := point hint(x: 5, y: 5)
  s3 := point hint(x: -5, y: 5)
  ground s0
  ground s1
  ground s2
  ground s3
}}
in back {{
  e0 := point hint(x: -3, y: -3)
  e1 := point hint(x: 3, y: -3)
  e2 := point hint(x: 3, y: 3)
  e3 := point hint(x: -3, y: 3)
  ground e0
  ground e1
  ground e2
  ground e3
}}
body := solid(face(s0, s1, s2, s3, -> close), face({order}, -> close), along: guide)
");
    let exact = |e: &program::Elaborated| {
        let mut sk = e.sketch.clone();
        gcs_core::solve::solve(&mut sk, gcs_core::solve::SolveOpts::default());
        gcs_core::solid::cad::recipe(&sk, 0).and_then(|r| gcs_core::brep::recipe::build(&r))
    };
    // written alike, both build the frustum
    let alike = read(&square("e0, e1, e2, e3"));
    assert!(evaluated(&alike, "body").volume() > 0.);
    assert!(exact(&alike).is_ok());
    // written the other way round, both refuse it, saying why
    let reversed = read(&square("e0, e3, e2, e1"));
    let faceted = reversed.sketch.evaluated_solid(reversed.map.ent_named("body").unwrap().i(), Report);
    assert!(faceted.as_ref().is_err_and(|e| e.contains("corresponding order")), "{:?}", faceted.map(|s| s.volume()));
    let e = exact(&reversed).unwrap_err();
    assert!(e.contains("corresponding order"), "{e}");
}
