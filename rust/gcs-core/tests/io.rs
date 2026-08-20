use gcs_core::constraints::{Arg, CKind, Constraint};
use gcs_core::diagnose::{diagnose, DiagnoseOptions};
use gcs_core::examples;
use gcs_core::io;
use gcs_core::json::fmt_g;
use gcs_core::model::{distance_between, EntRef, Sketch};
use gcs_core::solve::{solve, SolveOpts};

#[test]
fn round_trip_all_examples() {
    for name in examples::EXAMPLES {
        let sk = examples::example(name).unwrap();
        let s = io::dumps(&sk, Some(1));
        let mut sk2 = io::loads(&s).unwrap();
        assert_eq!(io::dumps(&sk2, Some(1)), s, "{name}");
        assert_eq!(sk2.constraints.len(), sk.constraints.len(), "{name}");
        assert!(solve(&mut sk2, SolveOpts::default()).success, "{name}");
    }
}

#[test]
fn without_removes_dependents() {
    let sk = examples::rect_fillets(100.0, 60.0, 10.0, 0.0);
    let n_arc_c = sk.constraints.iter().filter(|c| c.kind == CKind::TangentArcLine).count();
    let centre = EntRef::point(sk.arcs[0].center as usize);
    let sk2 = io::without(&sk, &[centre], &[]);
    assert_eq!(sk2.arcs.len(), 3);
    assert_eq!(
        sk2.constraints.iter().filter(|c| c.kind == CKind::TangentArcLine).count(),
        n_arc_c - 2
    );
    io::dumps(&sk2, Some(1)); // every kept constraint references only live entities
}

#[test]
fn without_a_line_keeps_its_points() {
    let sk = examples::truss(3, 20.0, 15.0, true);
    let n_pts = sk.points.len();
    let sk2 = io::without(&sk, &[EntRef::line(0)], &[]);
    assert_eq!(sk2.points.len(), n_pts);
    assert_eq!(sk2.lines.len(), sk.lines.len() - 1);
}

#[test]
fn describe_matches_the_reference_form() {
    let sk = examples::rect_fillets(100.0, 60.0, 10.0, 0.0);
    let last = sk.constraints.last().unwrap();
    assert_eq!(io::describe(last), "Distance(P6, P7, 40)");
    assert_eq!(fmt_g(80.0, 4), "80");
    assert_eq!(fmt_g(0.5, 4), "0.5");
    assert_eq!(fmt_g(1234567.0, 4), "1.235e+06");
    assert_eq!(fmt_g(90.0, 3), "90");
}

#[test]
fn a_live_drag_never_reaches_the_document() {
    use gcs_core::newton::Method;
    use gcs_core::solve::{Drag, RadiusDrag};
    let mut sk = examples::slotted_link(80.0, 15.0, 6.0);
    let n = sk.user_constraints().len();
    let mut d = Drag::new(&mut sk, 1, 1.0, 2.0, Method::DogLeg, 1.0, Vec::new(), 0.05);
    let mut r = RadiusDrag::new(&mut sk, EntRef::circle(0), 9.0, Method::DogLeg);
    assert_eq!(sk.user_constraints().len(), n);
    assert_eq!(io::loads(&io::dumps(&sk, None)).unwrap().constraints.len(), sk.constraints.len() - 2);
    r.end(&mut sk);
    d.end(&mut sk);
}

#[test]
fn a_soft_radius_is_not_a_known_dimension() {
    use gcs_core::cgraph::known_radii;
    use gcs_core::newton::Method;
    use gcs_core::solve::RadiusDrag;
    let mut sk = Sketch::new();
    let c = sk.point(0.0, 0.0, true, "c");
    let circ = sk.circle(c, 10.0, "c1");
    let mut d = RadiusDrag::new(&mut sk, EntRef::circle(circ), 10.0, Method::DogLeg);
    assert!(known_radii(&sk).is_empty());
    d.end(&mut sk);
}

#[test]
fn drawn_bounds_covers_curves_not_just_points() {
    let mut sk = Sketch::new();
    let c = sk.point(0.0, 0.0, false, "c");
    sk.circle(c, 10.0, "c1");
    assert_eq!(sk.bbox(), (0.0, 0.0, 0.0, 0.0));
    assert_eq!(sk.drawn_bounds(), (-10.0, -10.0, 10.0, 10.0));

    let mut sk2 = Sketch::new();
    let c = sk2.point(0.0, 0.0, false, "c");
    let s = sk2.point(5.0, 0.0, false, "s");
    let e = sk2.point(0.0, 5.0, false, "e");
    let arc = sk2.arc(c, s, e, "a");
    let b = sk2.bounds(EntRef::arc(arc));
    assert!((b.0).abs() < 1e-12 && (b.2 - 5.0).abs() < 1e-12 && (b.3 - 5.0).abs() < 1e-12);
    let (ex, ey) = (sk2.points[e].x as usize, sk2.points[e].y as usize);
    sk2.params[ex].value = -5.0;
    sk2.params[ey].value = 0.0; // now a half turn through the top
    let b = sk2.bounds(EntRef::arc(arc));
    assert!((b.0 + 5.0).abs() < 1e-12 && (b.1).abs() < 1e-12 && (b.3 - 5.0).abs() < 1e-12);
}

#[test]
fn three_point_arc_takes_the_sweep_through_the_third_point() {
    let mut sk = Sketch::new();
    let a = sk.point(-5.0, 0.0, false, "a");
    let b = sk.point(5.0, 0.0, false, "b");
    let up = sk.arc_through(a, b, (0.0, 5.0), "up").unwrap();
    let (cx, cy) = sk.point_xy(sk.arcs[up].center as usize);
    assert!(cx.abs() < 1e-12 && cy.abs() < 1e-12);
    assert!((sk.params[sk.arcs[up].radius as usize].value - 5.0).abs() < 1e-12);
    // CCW from a = (-5, 0) would sweep under, so a top-bulging arc has to start at b
    assert_eq!((sk.arcs[up].start as usize, sk.arcs[up].end as usize), (b, a));
    let (a0, a1) = sk.arc_angles(up);
    assert!(a0.abs() < 1e-12 && (a1 - std::f64::consts::PI).abs() < 1e-12);

    let mut sk2 = Sketch::new();
    let c = sk2.point(-5.0, 0.0, false, "c");
    let d = sk2.point(5.0, 0.0, false, "d");
    let down = sk2.arc_through(c, d, (0.0, -5.0), "dn").unwrap();
    assert_eq!((sk2.arcs[down].start as usize, sk2.arcs[down].end as usize), (c, d));
}

#[test]
fn three_point_arc_refuses_collinear_input() {
    let mut sk = Sketch::new();
    let a = sk.point(0.0, 0.0, false, "a");
    let b = sk.point(10.0, 0.0, false, "b");
    let n = sk.points.len();
    assert!(sk.arc_through(a, b, (5.0, 0.0), "x").is_none());
    assert!(sk.arc_through(a, b, (20.0, 1e-12), "x").is_none());
    assert_eq!(sk.points.len(), n); // nothing was created
    assert!(sk.arc_through(a, b, (5.0, 0.01), "x").is_some());
}

#[test]
fn a_rectangle_is_rigid_up_to_its_five_degrees_of_freedom() {
    let mut sk = Sketch::new();
    let lines = sk.rectangle_xy(0.0, 0.0, 40.0, 25.0, "r");
    assert_eq!(lines.len(), 4);
    assert_eq!(sk.points.len(), 4); // corners are shared, not duplicated
    let d = diagnose(&mut sk, DiagnoseOptions::default());
    assert_eq!(d.n_redundant, 0);
    assert_eq!(d.dof, 5); // position, rotation, width, height
    sk.perturb(3.0, 1);
    assert!(solve(&mut sk, SolveOpts::default()).success);
    for i in 0..4 {
        let u = sk.line_dir(lines[i]);
        let v = sk.line_dir(lines[(i + 1) % 4]);
        assert!((u.0 * v.0 + u.1 * v.1).abs() < 1e-6);
    }
}

#[test]
fn construction_flags_round_trip() {
    let mut sk = examples::slotted_link(80.0, 15.0, 6.0);
    sk.lines[0].construction = true;
    sk.arcs[0].construction = true;
    sk.circles[0].construction = true;
    let back = io::loads(&io::dumps(&sk, Some(1))).unwrap();
    assert_eq!(
        back.lines.iter().map(|l| l.construction).collect::<Vec<_>>(),
        sk.lines.iter().map(|l| l.construction).collect::<Vec<_>>()
    );
    assert_eq!(io::dumps(&back, Some(1)), io::dumps(&sk, Some(1)));
}

#[test]
fn distance_between_covers_every_pair_of_kinds() {
    let mut sk = Sketch::new();
    let o = sk.point(0.0, 0.0, false, "o");
    let p = sk.point(3.0, 4.0, false, "p");
    let horiz = sk.line_xy(0.0, 10.0, 20.0, 10.0, "h"); // y = 10
    let slant = sk.line_xy(0.0, 0.0, 10.0, 10.0, "s"); // y = x, crosses horiz
    let para = sk.line_xy(-5.0, 16.0, 5.0, 16.0, "pa"); // y = 16
    let cc = sk.point(0.0, 0.0, false, "cc");
    let c1 = sk.circle(cc, 2.0, "c1");
    let c2c = sk.point(10.0, 0.0, false, "c2c");
    let c2 = sk.circle(c2c, 3.0, "c2");
    let ic = sk.point(0.0, 0.0, false, "ic");
    let inner = sk.circle(ic, 0.5, "in");
    let ov = sk.point(3.0, 0.0, false, "ov");
    let over = sk.circle(ov, 2.0, "ov");
    let d = |a: EntRef, b: EntRef| distance_between(&sk, a, b);
    let close = |a: f64, b: f64| (a - b).abs() < 1e-9;
    assert!(close(d(EntRef::point(o), EntRef::point(p)), 5.0));
    assert!(close(d(EntRef::point(p), EntRef::point(o)), 5.0));
    assert!(close(d(EntRef::point(o), EntRef::line(horiz)), 10.0));
    assert!(close(d(EntRef::line(horiz), EntRef::point(o)), 10.0));
    assert!(close(d(EntRef::point(o), EntRef::circle(c2)), 7.0));
    assert!(close(d(EntRef::line(horiz), EntRef::line(para)), 6.0));
    assert!(close(d(EntRef::line(horiz), EntRef::line(slant)), 0.0));
    assert!(close(d(EntRef::line(horiz), EntRef::circle(c1)), 8.0));
    assert!(close(d(EntRef::line(slant), EntRef::circle(c1)), 0.0));
    assert!(close(d(EntRef::circle(c1), EntRef::circle(c2)), 5.0));
    assert!(close(d(EntRef::circle(c1), EntRef::circle(inner)), 1.5));
    assert!(close(d(EntRef::circle(c1), EntRef::circle(over)), 0.0));
}

#[test]
fn every_constraint_type_round_trips_through_its_spec() {
    for kind in gcs_core::constraints::ALL_KINDS {
        let spec = kind.spec();
        assert!(!spec.is_empty(), "{kind:?}");
        // a synthetic instance, argument kinds only — enough to prove the spec drives I/O
        let args: Vec<Arg> = spec
            .iter()
            .map(|(_, k)| match k {
                x if x.is_entity() => Arg::Ent(EntRef::point(0)),
                gcs_core::constraints::SpecKind::Int => Arg::Int(1),
                gcs_core::constraints::SpecKind::Bool => Arg::Bool(true),
                gcs_core::constraints::SpecKind::Str => Arg::Str("start".into()),
                _ => Arg::Num(1.5),
            })
            .collect();
        let c = Constraint::new(kind, args.clone());
        assert_eq!(c.args, args);
        assert_eq!(CKind::from_name(kind.name()), Some(kind));
    }
}

/// A document is untrusted input.  Every stored index is checked on load, so a hand-edited or
/// truncated file is an `Err` the caller can show — not an out-of-bounds index inside the model.
#[test]
fn a_dangling_reference_is_an_error_not_a_panic() {
    let bad = [
        r#"{"points":[{"x":0,"y":0}],"arcs":[{"center":7,"start":0,"end":0,"r":1}]}"#,
        r#"{"points":[{"x":0,"y":0}],"lines":[{"p1":0,"p2":4}]}"#,
        r#"{"points":[{"x":0,"y":0}],"lines":[[0,4]]}"#,
        r#"{"points":[{"x":0,"y":0}],"circles":[{"center":-1,"r":1}]}"#,
        r#"{"points":[{"x":0,"y":0}],"constraints":[{"type":"Horizontal","args":[["line",0]]}]}"#,
        r#"{"points":[{"x":0,"y":0},{"x":1,"y":0}],
            "constraints":[{"type":"Distance","args":[["point",0],["point",9],1]}]}"#,
    ];
    for s in bad {
        let e = io::loads(s).unwrap_err();
        assert!(e.contains("out of range"), "{s} gave {e}");
    }
}

#[test]
fn a_load_that_is_in_range_still_works() {
    let sk = io::loads(
        r#"{"points":[{"x":0,"y":0},{"x":3,"y":4}],"lines":[{"p1":0,"p2":1}],
            "constraints":[{"type":"Distance","args":[["point",0],["point",1],5]}]}"#,
    )
    .unwrap();
    assert_eq!(sk.points.len(), 2);
    assert_eq!(sk.lines.len(), 1);
    assert_eq!(sk.user_constraints().len(), 1);
}

/// `row_of` is asked about ids that need not be in the plan (a constraint added after compile).
#[test]
fn row_of_an_uncompiled_constraint_is_none() {
    let mut sk = Sketch::new();
    let a = sk.point(0.0, 0.0, false, "a");
    let b = sk.point(10.0, 0.0, false, "b");
    sk.line(a, b);
    let d = sk.add(Constraint::distance(EntRef::point(a), EntRef::point(b), 10.0));
    let sys = gcs_core::system::System::new(&sk);
    assert!(sys.row_of(d).is_some());
    let later = sk.add(Constraint::new(CKind::Horizontal, vec![Arg::Ent(EntRef::line(0))]));
    assert_eq!(sys.row_of(later), None);
}

/// Only a `DragTarget` has a target; the shorter argument lists must not be written past.
#[test]
fn set_target_refuses_a_constraint_without_one() {
    let mut c = Constraint::new(CKind::Radius, vec![Arg::Ent(EntRef::circle(0)), Arg::Num(5.0)]);
    assert!(!c.set_target(1.0, 2.0));
    assert_eq!(c.args.len(), 2);
    let mut d = Constraint::drag_target(EntRef::point(0), 1.0, 2.0, 1.0);
    assert!(d.set_target(3.0, 4.0));
    assert_eq!(d.consts()[0], 3.0);
}
