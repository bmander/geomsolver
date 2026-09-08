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
point o hint(x: 0, y: 0)
point q hint(x: 1, y: 0)
plane front(origin: o, toward: q)
plane back(origin: o, toward: q, from: front, offset: -20mm)
plane top(origin: o, toward: q, from: front, fold: 0deg)
in top {
  point a hint(x: 0, y: 0)
  point b hint(x: 0, y: 20)
  line guide(a, b)
}
in front {
  point c hint(x: 0, y: 0)
  circle outer(center: c) hint(r: 10)
  circle inner(center: c) hint(r: 5)
  face section(outer, holes: inner)
}
in back {
  point ec hint(x: 0, y: 0)
  circle eo(center: ec) hint(r: 6)
  circle ei(center: ec) hint(r: 3)
  face end_section(eo, holes: ei)
}
";

fn evaluated(e: &program::Elaborated, name: &str) -> std::rc::Rc<gcs_core::solid::EvaluatedSolid> {
    e.sketch
        .evaluated_solid(e.map.ent_named(name).unwrap().i(), Report)
        .unwrap()
}

#[test]
fn sweep_and_loft_preserve_holes_caps_and_closed_meshes() {
    let e = read(&format!("{SECTIONS}solid sweep(section, along: guide)\nsolid loft(section, end_section, along: guide)\n"));
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
        "{SECTIONS}solid body(section, end_section, along: guide)\n"
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
        ("line guide(a, b)", "line guide(b, a)", "start section"),
        (
            "face end_section(eo, holes: ei)",
            "face end_section(eo)",
            "numbers of holes",
        ),
    ] {
        let src =
            format!("{SECTIONS}solid body(section, end_section, along: guide)\n").replace(old, new);
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
            "{SECTIONS}solid body(section, along: guide{tail})\n"
        ));
        assert!(!errs.is_empty());
    }
}

#[test]
fn arc_sweep_follows_solved_radius_and_transports_the_profile() {
    let src = "unit mm
point o hint(x: 0, y: 0)
point q hint(x: 1, y: 0)
plane front(origin: o, toward: q)
plane top(origin: o, toward: q, from: front, fold: 0deg)
in top {
 point center hint(x: 0, y: 0)
 point a hint(x: 30, y: 0)
 point b hint(x: 0, y: 30)
 arc guide(center: center, start: a, end: b) hint(r: 30)
}
in front {
 point c hint(x: 30, y: 0)
 circle outer(center: c) hint(r: 5)
 circle inner(center: c) hint(r: 3)
 face section(outer, holes: inner)
}
solid body(section, along: guide)
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
    let src = format!("{SECTIONS}solid body(end_section, section, along: guide)\n")
        .replace("line guide(a, b)", "line guide(b, a)");
    let mut e = read(&src);
    let want = evaluated(&e, "body").volume();
    let (mut p, errors) = syntax::parse("solid body(end_section, section, along: guide)\n");
    assert!(errors.is_empty());
    let printed = syntax::render_flat(&mut p).unwrap();
    let rebuilt = format!(
        "{}{printed}",
        SECTIONS.replace("line guide(a, b)", "line guide(b, a)")
    );
    assert!((evaluated(&read(&rebuilt), "body").volume() - want).abs() < 1e-7);
    let committed = gcs_core::edit::commit_seeds(&e, &e.sketch, &e.program);
    assert!(committed
        .text
        .contains("solid body(end_section, section, along: guide)"));
    let k = 0.5_f64.sqrt();
    let rotate = |p: [f64; 3]| [p[0], k * (p[1] - p[2]), k * (p[1] + p[2])];
    for p in &mut e.sketch.planes {
        p.basis.u = rotate(p.basis.u);
        p.basis.v = rotate(p.basis.v);
        p.basis.o = rotate(p.basis.o);
        for (i, delta) in [100.0, -70.0, 50.0].iter().enumerate() {
            p.basis.o[i] += delta;
        }
    }
    let s = evaluated(&e, "body");
    assert!((s.volume() - want).abs() < 1e-6);
    assert_eq!(super::solid::unpaired(s.mesh()), 0);
}

#[test]
fn a_straight_sweep_resizes_with_the_line_and_can_be_a_boolean_operand() {
    let src=format!("{SECTIONS}solid sweep(section, along: guide)\nsolid cutter(face(outer), depth: 10mm)\nsolid body(sweep)\ncutter cut body\n");
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
point o hint(x: 0, y: 0)
point q hint(x: 1, y: 0)
plane front(origin: o, toward: q)
plane top(origin: o, toward: q, from: front, fold: 0deg)
plane side(origin: o, toward: q, u: (0,1,0), v: (0,0,1))
in top {
 point center hint(x: 0, y: 0)
 point a hint(x: 30, y: 0)
 point b hint(x: 0, y: 30)
 arc guide(center: center, start: a, end: b) hint(r: 30)
}
in front {
 point c hint(x: 30, y: 0)
 circle outer(center: c) hint(r: 5)
 face section(outer)
}
in side {
 point ec hint(x: 30, y: 0)
 circle eo(center: ec) hint(r: 4)
 face end_section(eo)
}
solid body(section, end_section, along: guide)
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
component Duct(section: face, guide: line) {{ solid body(section, along: guide) }}
x: Duct(section, guide)
solid inline(face(outer, holes: inner), along: guide)
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
        let (p, errs) = syntax::parse(&format!("{SECTIONS}solid bad({args})\n"));
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
point p0 hint(x: 0, y: 0)
point p1 hint(x: 10, y: 0)
point p2 hint(x: 10, y: 5)
point p3 hint(x: 5, y: 5)
point p4 hint(x: 5, y: 10)
point p5 hint(x: 0, y: 10)
point h0 hint(x: 7, y: 2)
point h1 hint(x: 2, y: 7)
circle hole0(center: h0) hint(r: 1)
circle hole1(center: h1) hint(r: 1)
outline = line e0(p0, p1) -> line e1(p1, p2) -> line e2(p2, p3) -> line e3(p3, p4) -> line e4(p4, p5) -> line e5(p5, p0) -> close
face section(outline, holes: hole0, hole1)
}}
solid body(section, along: guide)
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
    let e = read(&format!("{SECTIONS}solid body(section, along: guide)\n"));
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
        "{SECTIONS}solid body(section, end_section, along: guide)\n"
    ));
    let before = evaluated(&e, "body").volume();
    let p = e.map.ent_named("back").unwrap().i();
    let (sn, cs) = 97.0_f64.to_radians().sin_cos();
    e.sketch.planes[p].basis.u = [cs, 0.0, sn];
    e.sketch.planes[p].basis.v = [sn, 0.0, -cs];
    let after = evaluated(&e, "body");
    assert!((after.volume() - before).abs() < 1e-7);
    assert_eq!(super::solid::unpaired(after.mesh()), 0);
}
