//! Signed datum coordinates retain the shared solve and independent plane membership.
use gcs_core::{callout, diagnose, io, library, program, solve, syntax};
use gcs_core::constraints::CKind;
use gcs_core::model::Sketch;

fn build(src: &str) -> program::Elaborated {
    let (p, errors, linked) = library::parse_linked(src);
    assert!(errors.is_empty() && linked.is_empty(), "{errors:?} {linked:?}");
    let e = program::elaborate(&p);
    assert!(e.ok(), "{:?}", e.diags);
    e
}

fn solved(sk: &mut Sketch) {
    let r = solve::solve(sk, Default::default());
    assert!(r.success, "{}", r.message);
}

fn close(a: (f64, f64), b: (f64, f64)) {
    assert!((a.0 - b.0).hypot(a.1 - b.1) < 1e-6, "{a:?} != {b:?}");
}

#[test]
fn standard_datums_are_shared_fixed_and_only_built_when_used() {
    let unused = build("use std\np := point\n");
    assert_eq!(unused.sketch.points.len(), 1);
    assert!(unused.sketch.planes.is_empty());
    let explicit = build("use std\nstd := std.StandardDatums()\na := point hint(x: 3, y: 4)\na distance(3, along: u) std.front\na distance(4, along: v) std.front\n");
    assert_eq!(explicit.sketch.planes.len(), 2, "an explicit std binding is not duplicated");
    let mut e = build("unit mm\nuse std\n\
        a := point hint(x: 3, y: 4)\na distance(3mm, along: u) std.front\na distance(4mm, along: v) std.front\n\
        b := point hint(x: -4, y: 3)\nb distance(3mm, along: u) std.up\nb distance(4mm, along: v) std.up\n\
        c := point hint(x: 6, y: 8)\nc distance(6mm, along: u) std.front\nc distance(8mm, along: v) std.front\n");
    solved(&mut e.sketch);
    assert_eq!(e.sketch.planes.len(), 2);
    assert_eq!(e.sketch.points.len(), 6);
    close(e.sketch.point_xy(e.map.ent_named("a").unwrap().i()), (3.0, 4.0));
    close(e.sketch.point_xy(e.map.ent_named("b").unwrap().i()), (-4.0, 3.0));
    assert_eq!(diagnose::diagnose(&mut e.sketch, Default::default()).dof, 0);
    assert!((0..e.sketch.points.len()).all(|p| e.sketch.plane_of(p).is_none()));
    let sk = e.sketch.clone();
    let edit = gcs_core::edit::reconcile(&mut e, &sk);
    assert_eq!(edit.kind, gcs_core::edit::Kind::None, "{}", edit.text);
    assert!(!edit.text.contains("fix("), "library datums must not be copied into model source");
}

#[test]
fn standard_datums_work_in_hints_children_and_explicit_membership() {
    let mut e = build("unit mm\nuse std\n\
        a := point\n\
        fix(x == 0mm, y == 1mm) a\naxis := line(std.front.origin, std.front.toward)\n\
        b := point in std.front\nb distance(2mm, along: u) std.front\nb distance(3mm, along: v) std.front\n");
    solved(&mut e.sketch);
    close(e.sketch.point_xy(e.map.ent_named("a").unwrap().i()), (0.0, 1.0));
    let front = e.map.ent_named("std.front").unwrap().i();
    assert_eq!(e.sketch.plane_of(e.map.ent_named("b").unwrap().i()), Some(front));
}

#[test]
fn unnamed_component_calls_keep_their_instances_distinct_and_source_intact() {
    let src = "unit mm\nuse std\ncomponent Spoke(f: plane) {\n\
        tip := point hint(x: f.origin.x + 5mm * f.c, y: f.origin.y + 5mm * f.s)\n\
        axis := line(f.origin, f.toward)\ntip on axis\nf.origin distance(5mm) tip\n}\n\
        preview {\nSpoke(std.front)\nSpoke(std.up)\n}\n";
    let mut e = build(src);
    solved(&mut e.sketch);
    assert_eq!(e.sketch.points.len(), 5);
    assert_eq!(diagnose::diagnose(&mut e.sketch, Default::default()).dof, 0);
    assert!(e.map.names.values().flatten().all(|n| !n.contains('#')));
    let instances: Vec<_> = e.program.root().body.iter().filter(|s|
        matches!(s.kind, syntax::StmtKind::Instance(_))).collect();
    assert_eq!(instances.len(), 2);
    for st in instances {
        let mut text = String::new();
        syntax::write_stmt_to(&mut text, &st.kind).unwrap();
        assert!(text.starts_with("Spoke("), "{text}");
        assert!(!text.contains('#'));
    }
    let sketch = e.sketch.clone();
    let edit = gcs_core::edit::reconcile(&mut e, &sketch);
    assert_eq!(edit.text, src);
    e.sketch.add(gcs_core::constraints::Constraint::distance(
        gcs_core::model::EntRef::point(3), gcs_core::model::EntRef::point(4), 5.0));
    let sketch = e.sketch.clone();
    let edit = gcs_core::edit::reconcile(&mut e, &sketch);
    assert!(edit.refused.as_deref().is_some_and(|s| s.contains("component call")),
        "{:?}: {}", edit.refused, edit.text);
    assert_eq!(edit.text, src);
}

#[test]
fn signed_coordinates_follow_a_moving_datum_even_from_the_wrong_quadrant() {
    let mut e = build("unit mm\no := point\nq := point\nfix(x == 10, y == 20) o\nfix(x == 14, y == 23) q\nf := plane(origin: o, toward: q)\np := point hint(x: 100, y: -200)\np distance(-5mm, along: u) f\np distance(2mm, along: v) f\n");
    let p = e.map.ent_named("p").unwrap().i();
    solved(&mut e.sketch);
    close(e.sketch.point_xy(p), (4.8, 18.6));
    let q = e.map.ent_named("q").unwrap().i();
    let [x, y] = e.sketch.point_params(q);
    e.sketch.params[x as usize].value = 7.0;
    e.sketch.params[y as usize].value = 24.0;
    solved(&mut e.sketch);
    close(e.sketch.point_xy(p), (11.4, 14.8));
    assert_eq!(diagnose::diagnose(&mut e.sketch, Default::default()).dof, 0);
    let back = io::loads(&io::dumps(&e.sketch, None)).unwrap();
    assert_eq!(back.constraints.iter().filter(|c| matches!(c.kind, CKind::CoordinateU | CKind::CoordinateV)).count(), 2);
    let c = e.sketch.constraints.iter().find(|c| c.kind == CKind::CoordinateU).unwrap();
    let Some(callout::Frame::Linear { d, .. }) = callout::frame(&e.sketch, c) else { panic!("no ordinate callout") };
    close(d, (-0.6, 0.8));
}

#[test]
fn an_external_constraint_moves_the_datum_through_an_aliased_component_point() {
    let mut e = build("unit mm\ncomponent Offset(f: plane, p: point) {\np distance(3mm, along: u) f\np distance(-2mm, along: v) f\n}\no := point hint(x: 10, y: 20)\nq := point hint(x: 14, y: 20)\naxis := line(o, q)\nhorizontal axis\no distance(4mm, along: x) q\nf := plane(origin: o, toward: q)\np := point\nfix(x == 23, y == 28) p\ni := Offset(f, p)\n");
    solved(&mut e.sketch);
    close(e.sketch.point_xy(e.map.ent_named("o").unwrap().i()), (20.0, 30.0));
    assert_eq!(e.sketch.points.len(), 3, "the argument is an alias");
    assert_eq!(diagnose::diagnose(&mut e.sketch, Default::default()).dof, 0);
}

#[test]
fn both_ordinates_can_be_solved_as_independent_dimensions() {
    let mut e = build("unit mm\ncomponent Read(f: plane, p: point, u: Length, v: Length) {\np distance(u, along: u) f\np distance(v, along: v) f\n}\no := point\nq := point\np := point\nfix(x == 10, y == 20) o\nfix(x == 14, y == 23) q\nfix(x == 4.8, y == 18.6) p\nf := plane(origin: o, toward: q)\nr := Read(f, p)\n");
    super::common::fd_jacobian(&e.sketch, 1e-6);
    solved(&mut e.sketch);
    let value = |name: &str| e.sketch.params[e.sketch.free_vars[name] as usize].value;
    assert!((value("r.u") + 5.0).abs() < 1e-6);
    assert!((value("r.v") - 2.0).abs() < 1e-6);
}

#[test]
fn datum_seeds_and_membership_pass_through_nested_instances() {
    let mut e = build("unit mm\nuse std\ncomponent Probe(f: plane) {\np := point hint(x: f.origin.x - 5mm * f.c - 2mm * f.s, y: f.origin.y - 5mm * f.s + 2mm * f.c)\np distance(-5mm, along: u) f\np distance(2mm, along: v) f\n}\ncomponent Part(f: plane) { probe := Probe(f) }\no := point\nq := point\nfix(x == 10, y == 20) o\nfix(x == 14, y == 23) q\ndatum := plane(origin: o, toward: q)\nview := plane\ni := Part(datum) in view\n");
    let p = e.map.ent_named("i.probe.p").unwrap().i();
    close(e.sketch.point_xy(p), (4.8, 18.6));
    let view = e.map.ent_named("view").unwrap().i();
    assert_eq!(e.sketch.points[p].plane, Some(view as u32));
    assert_eq!(e.sketch.points[e.map.ent_named("o").unwrap().i()].plane, None);
    solved(&mut e.sketch);
    close(e.sketch.point_xy(p), (4.8, 18.6));
}

#[test]
fn zero_ordinates_are_regular_and_claims_remain_assertions() {
    let mut e = build("unit mm\no := point\nq := point\nfix(x == 10, y == 20) o\nfix(x == 14, y == 23) q\nf := plane(origin: o, toward: q)\np := point\np distance(0mm, along: u) f\np distance(0mm, along: v) f\nclaim p distance(0mm, along: u) f\n");
    solved(&mut e.sketch);
    close(e.sketch.point_xy(e.map.ent_named("p").unwrap().i()), (10.0, 20.0));
    assert_eq!(diagnose::diagnose(&mut e.sketch, Default::default()).status, diagnose::State::Well);
}

#[test]
fn datum_axes_require_a_datum_and_lengths() {
    for relation in ["p distance(2mm, along: u) q", "p distance(2mm, along: x) f", "p distance(2deg, along: u) f", "p distance(2mm, along: z) f"] {
        let source = format!("unit mm\np := point\nq := point\nf := plane\n{relation}\n");
        let (p, errs) = syntax::parse(&source);
        assert!(errs.is_empty(), "{errs:?}");
        assert!(!program::elaborate(&p).ok(), "accepted {relation}");
    }
}

#[test]
fn a_datum_child_remains_a_point_through_aliases_and_face_corners() {
    let e = build("unit mm\ncomponent Inner(p: point) {\nq := point hint(x: p.x + 1mm, y: p.y + 2mm)\nborder := line(p, q)\n}\ncomponent Outer(f: plane) {\ni := Inner(f.origin)\ntriangle := face(f.origin, i.q, f.toward, -> close)\n}\no := point hint(x: 10, y: 20)\nt := point hint(x: 14, y: 23)\nf := plane(origin: o, toward: t)\nx := Outer(f)\n");
    let edge = &e.sketch.lines[e.map.ent_named("x.i.border").unwrap().i()];
    assert_eq!(edge.p1 as usize, e.map.ent_named("o").unwrap().i());
    close(e.sketch.point_xy(edge.p2 as usize), (11.0, 22.0));
    assert_eq!(e.sketch.points.len(), 3);
    assert_eq!(e.sketch.faces.len(), 1);
}

#[test]
fn coordinate_placement_is_not_a_standard_library_component() {
    let (p, errors, linked) = library::parse_linked("use std\np := Loc(std.front, u: 3mm, v: 4mm)\n");
    assert!(errors.is_empty() && linked.is_empty());
    let e = program::elaborate(&p);
    assert!(!e.ok(), "the removed placement helper must not remain callable");
}

#[test]
fn vtwin_datums_follow_a_complete_crank_turn() {
    let mut e = build(gcs_core::examples::source("vtwin").unwrap());
    solved(&mut e.sketch);
    let theta = e.sketch.free_vars["crank.theta"] as usize;
    e.sketch.params[theta].fixed = true;
    let point = |e: &program::Elaborated, name: &str| {
        e.sketch.point_xy(e.map.ent_named(name).unwrap().i())
    };
    for step in 0..=8 {
        e.sketch.params[theta].value = 180.0 + step as f64 * 45.0;
        solved(&mut e.sketch);
        let pin = point(&e, "crank.pin");
        for (bank, pivot) in [("bankR", "plate.r.piv"), ("bankL", "plate.l.piv")] {
            let pivot = point(&e, pivot);
            let (dx, dy) = (pivot.0 - pin.0, pivot.1 - pin.1);
            let length = dx.hypot(dy);
            let (c, s) = (dx / length, dy / length);
            let crown = (pin.0 + 46.0 * c, pin.1 + 46.0 * s);
            close(point(&e, &format!("{bank}.crown")), crown);
            // Cylinder mouth corner: eight below the pivot and twelve to its left.
            close(point(&e, &format!("{bank}.cyl.k_bl")),
                  (pivot.0 - 8.0 * c - 12.0 * s, pivot.1 - 8.0 * s + 12.0 * c));
            // The left flank starts fourteen down from the crown, 2.5 left of the rod.
            close(point(&e, &format!("{bank}.pis.ra")),
                  (crown.0 - 14.0 * c - 2.5 * s, crown.1 - 14.0 * s + 2.5 * c));
        }
    }
}

// Edit the design or hardware module as the file pane does; callers keep passing the same group.
fn vtwin_variant(src: &str, edits: &[(&str, &str, &str)]) -> program::Elaborated {
    let (mut p, errors) = syntax::parse(src);
    assert!(errors.is_empty(), "{errors:?}");
    let linked = gcs_core::modules::link(&mut p, &mut |name| {
        let mut text = library::resolve(name)?;
        for &(module, before, after) in edits {
            if name == module {
                assert!(text.contains(before), "{module}: missing {before}");
                text = text.replacen(before, after, 1);
            }
        }
        Some(text)
    });
    assert!(linked.is_empty(), "{linked:?}");
    let mut e = program::elaborate(&p);
    assert!(e.ok(), "{:?}", e.diags);
    solved(&mut e.sketch);
    e
}

#[test]
fn vtwin_cylinder_follows_piston_travel_and_wall_thickness() {
    let src = "unit mm\nuse std\nuse components.dims\nuse components.bank\n\
        bottom_pin := point hint(x: 0mm, y: -components.dims.R)\nstd.origin vertical bottom_pin\nstd.origin distance(components.dims.R) bottom_pin\n\
        top_pin := point hint(x: 0mm, y: components.dims.R)\nstd.origin vertical top_pin\nstd.origin distance(components.dims.R) top_pin\n\
        pivot := point hint(x: 0mm, y: components.dims.H)\nstd.origin vertical pivot\nstd.origin distance(components.dims.H) pivot\n\
        bottom := components.bank.Bank(bottom_pin, pivot, fw: components.dims.fwA, dim: 0, dims: components.dims.vtwin_dims)\n\
        top := components.bank.Bank(top_pin, pivot, fw: components.dims.fwA, dim: 0, dims: components.dims.vtwin_dims)\n";
    for (before, after, wall) in [
        ("R := 10mm", "R := 12mm", 4.0),
        ("L := 46mm", "L := 49mm", 4.0),
        ("ph := 14mm", "ph := 16mm", 4.0),
        ("D := 16mm", "D := 18mm", 4.0),
        ("wall := 4mm", "wall := 5mm", 5.0),
    ] {
        let e = vtwin_variant(src, &[("components.dims", before, after)]);
        let at = |name| e.sketch.point_xy(e.map.ent_named(name).unwrap().i());
        // At the two ends of travel, the skirt meets the mouth and the crown clears the head.
        close(at("bottom.pis.s0"), at("bottom.cyl.m0"));
        let crown = at("top.crown");
        close(at("top.cyl.hx"), (crown.0, crown.1 + 2.0));
        let inner = at("top.cyl.b_tr");
        close(at("top.cyl.k_tr"), (inner.0 + wall, inner.1 + wall));
    }
}

#[test]
fn vtwin_hardware_changes_update_pockets_bores_and_seal_grooves() {
    let e = vtwin_variant(gcs_core::examples::source("vtwin").unwrap(), &[
        ("hardware", "hexbolt14_h := 4.4mm", "hexbolt14_h := 5.4mm"),
        ("hardware", "brg608_w := 7mm", "brg608_w := 8mm"),
        ("hardware", "clevis14_d := 6.35mm", "clevis14_d := 6.85mm"),
        ("hardware", "clevis14_head_d := 9.7mm", "clevis14_head_d := 10.7mm"),
        ("hardware", "clevis14_head_t := 2.3mm", "clevis14_head_t := 3.3mm"),
        ("hardware", "oring014_cs := 1.78mm", "oring014_cs := 2mm"),
        ("hardware", "oring010_cs := 1.78mm", "oring010_cs := 2mm"),
        ("components.dims", "rbar := 5mm", "rbar := 6mm"),
    ]);
    let at = |name| e.sketch.point_xy(e.map.ent_named(name).unwrap().i());
    let radius = |name| e.sketch.radius_value(e.map.ent_named(name).unwrap());
    let extent = |name| {
        let solid = &e.sketch.solids[e.map.ent_named(name).unwrap().i()];
        let gcs_core::model::SolidDef::Prism { from, to, .. } = &solid.def else { panic!("{name}") };
        (from.value, to.value)
    };
    let near = |a: f64, b: f64| assert!((a - b).abs() < 1e-7, "{a} != {b}");

    // The pivot head still fits its slot, with the original wall left between slot and bore.
    let trap = extent("bankL.cyl.trap");
    near(trap.1 - trap.0, 6.0);
    near(trap.1, -11.0); // bore surface is at -8mm, with 3mm of material before it
    let bearing = extent("plate.blank.bboss");
    let bearing_pocket = extent("plate.blank.bpkt");
    near(bearing.1 - bearing.0, 18.0);
    near(bearing.1 - bearing_pocket.1, 1.5);
    near(radius("crank.disc.ph"), 3.5);
    near(radius("bankL.pis.hole"), 3.5);
    // The clevis pocket follows the larger head in both the disc and the hardware side view.
    near(radius("crank.disc.pkt"), 5.6);
    let pocket = extent("crank.disc.pinpkt");
    near(pocket.1 - pocket.0, 4.0);
    near(at("side.disc.b").0 - at("side.head.a").0, 4.0);
    // Barrel and inlet keep their 0.2mm diametral running clearance.
    near(radius("plate.thr.barrel"), 6.0);
    near(radius("plate.inlet.tbore") - radius("plate.thr.barrel"), 0.1);
    // Both grooves follow the newly selected 2mm-section rings and the shared seal rule.
    near(at("plate.thr.seal1.p").0 - at("plate.thr.section_center").0, 4.24);
    near((at("plate.thr.seal1.p").1 - at("plate.thr.seal2.p").1).abs(), 2.7);
    near((at("bankL.pis.g1R").0 - at("bankL.pis.g1L").0).hypot(
        at("bankL.pis.g1R").1 - at("bankL.pis.g1L").1), 12.48);
}
