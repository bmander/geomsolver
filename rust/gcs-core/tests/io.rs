use gcs_core::constraints::{same_constraint, Arg, CKind, Constraint};
use gcs_core::diagnose::{diagnose, DiagnoseOptions};
use gcs_core::examples;
use gcs_core::io;
use gcs_core::json::{fmt_g, Json};
use gcs_core::model::{self, angle_between, distance_between, on_radius, EntKind, EntRef, Sketch};
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
    // the height, which the case states across the rectangle: `distance(t1, b2) == h`
    let last = sk.constraints.last().unwrap();
    assert_eq!(io::describe(last), "Distance(P4, P1, 60)");
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
    assert_eq!(d.consts(&Sketch::new())[0], 3.0);
}

/// A front end caches compiled plans against this.  Counts and type names alone are not enough:
/// deleting one `Distance` and adding another leaves both identical, and the cache then replays a
/// plan that still enforces the old dimension and ignores the new one.
#[test]
fn the_topology_key_distinguishes_one_constraint_from_another_of_the_same_type() {
    let mut sk = Sketch::new();
    for i in 0..4 {
        sk.point(i as f64 * 10.0, 0.0, false, &format!("p{i}"));
    }
    let a = sk.add(Constraint::distance(EntRef::point(0), EntRef::point(1), 50.0));
    let k1 = sk.topology_key();

    sk.remove(a);
    sk.add(Constraint::distance(EntRef::point(2), EntRef::point(3), 20.0));
    assert_ne!(sk.topology_key(), k1, "swapping one Distance for another kept the key");

    // and it does move with the things a compiled plan actually depends on
    let k2 = sk.topology_key();
    sk.fix_point(0, true);
    assert_ne!(sk.topology_key(), k2);
    sk.fix_point(0, false);
    assert_eq!(sk.topology_key(), k2);
    let (x, y) = sk.point_xy(0);
    sk.set_x(&{
        let mut v = sk.get_x();
        v[0] = x + 5.0;
        v[1] = y + 5.0;
        v
    });
    assert_eq!(sk.topology_key(), k2, "moving geometry is not a topology change");
}

/// A parameter vector of the wrong length belongs to some other sketch.  Writing the overlapping
/// prefix scattered one sketch's coordinates over another's — the DOF animation restoring its
/// starting state into whatever sketch had replaced it, for one.
#[test]
fn set_x_refuses_a_vector_that_is_not_this_sketchs() {
    let mut a = Sketch::new();
    a.point(1.0, 2.0, false, "a");
    a.point(3.0, 4.0, false, "b");
    let mut b = Sketch::new();
    b.point(9.0, 9.0, false, "only");

    let xa = a.get_x();
    assert!(!b.set_x(&xa), "a two-point vector was accepted by a one-point sketch");
    assert_eq!(b.point_xy(0), (9.0, 9.0));
    assert!(b.set_x(&[7.0, 8.0]));
    assert_eq!(b.point_xy(0), (7.0, 8.0));
}

/// The duplicate rule is the core's, and both bindings now ask it through the ABI rather than
/// keeping a copy.  Whatever `spec` a new type declares, it has to at least recognise itself.
#[test]
fn same_constraint_recognises_every_type_reflexively() {
    let mut sk = Sketch::new();
    let p = sk.point(0.0, 0.0, false, "p");
    let q = sk.point(10.0, 0.0, false, "q");
    let l1 = sk.line(p, q);
    let l2 = sk.line(q, p);
    let c1 = sk.circle(p, 2.0, "c1");
    let c2 = sk.circle(q, 3.0, "c2");

    let pairs: Vec<(Constraint, Constraint, bool)> = vec![
        // commutative: the same relation with the pair picked the other way round
        (
            Constraint::coincident(EntRef::point(p), EntRef::point(q)),
            Constraint::coincident(EntRef::point(q), EntRef::point(p)),
            true,
        ),
        (
            Constraint::distance(EntRef::point(p), EntRef::point(q), 5.0),
            Constraint::distance(EntRef::point(q), EntRef::point(p), 5.0),
            true,
        ),
        (
            Constraint::two_line(CKind::Parallel, EntRef::line(l1), EntRef::line(l2)),
            Constraint::two_line(CKind::Parallel, EntRef::line(l2), EntRef::line(l1)),
            true,
        ),
        // not commutative: the first argument is the reference
        (
            Constraint::new(
                CKind::AnnularDistance,
                vec![Arg::Ent(EntRef::circle(c1)), Arg::Ent(EntRef::circle(c2)), Arg::Num(1.0)],
            ),
            Constraint::new(
                CKind::AnnularDistance,
                vec![Arg::Ent(EntRef::circle(c2)), Arg::Ent(EntRef::circle(c1)), Arg::Num(1.0)],
            ),
            false,
        ),
        // a different dimension is a conflict, not a duplicate
        (
            Constraint::distance(EntRef::point(p), EntRef::point(q), 5.0),
            Constraint::distance(EntRef::point(p), EntRef::point(q), 6.0),
            false,
        ),
    ];
    for (a, b, want) in pairs {
        assert!(same_constraint(&a, &a), "{:?} does not recognise itself", a.kind);
        assert_eq!(same_constraint(&a, &b), want, "{:?}", a.kind);
        assert_eq!(same_constraint(&b, &a), want, "{:?} (reversed)", a.kind);
    }
}

/// Both bindings rebuild their whole constraint list from these records after every edit, and
/// they read identity and arguments.  Anything else in there is work per constraint per edit that
/// nobody asked for — a formatted description, or an `error` that evaluates the kernel.
#[test]
fn the_constraint_record_carries_identity_and_arguments_only() {
    let sk = examples::rect_fillets(100.0, 60.0, 10.0, 0.0);
    let Json::Arr(recs) = gcs_core::report::constraints_json(&sk) else { panic!("not an array") };
    assert!(!recs.is_empty());
    for rec in &recs {
        let Json::Obj(kv) = rec else { panic!("not an object") };
        let keys: Vec<&str> = kv.iter().map(|(k, _)| k.as_str()).collect();
        assert_eq!(keys, ["id", "type", "args", "soft", "intrinsic"], "{keys:?}");
    }
    // and what was dropped is still reachable for the one constraint someone is looking at
    let c = &sk.constraints[0];
    assert!(!io::describe(c).is_empty());
    assert!(c.error(&sk).is_finite());
}

/// Two pieces of geometry the front ends were each doing for themselves: the current angle a
/// dimension dialog offers, and where the third click of a centre–start–end arc lands.
#[test]
fn angle_between_and_on_radius_are_the_core_s() {
    let mut sk = Sketch::new();
    let o = sk.point(0.0, 0.0, false, "o");
    let e = sk.point(10.0, 0.0, false, "e");
    let n = sk.point(0.0, 10.0, false, "n");
    let east = sk.line(o, e);
    let north = sk.line(o, n);
    let a = angle_between(&sk, EntRef::line(east), EntRef::line(north));
    assert!((a - std::f64::consts::FRAC_PI_2).abs() < 1e-12, "{a}");
    let back = angle_between(&sk, EntRef::line(north), EntRef::line(east));
    assert!((back + std::f64::consts::FRAC_PI_2).abs() < 1e-12, "{back}"); // signed, so CW

    // the third click gives a direction; the radius comes from the second point
    let (x, y) = on_radius(0.0, 0.0, 3.0, 4.0, 10.0).unwrap();
    assert!((x - 6.0).abs() < 1e-12 && (y - 8.0).abs() < 1e-12, "{:?}", (x, y));
    assert_eq!(on_radius(1.0, 1.0, 1.0, 1.0, 5.0), None); // the centre names no direction
}

/* -- copy and paste ------------------------------------------------------------------ */

#[test]
fn copy_takes_the_points_that_define_what_was_picked() {
    let sk = examples::rect_fillets(100.0, 60.0, 10.0, 0.0);
    // one line, nothing else: its two endpoints have to come with it or it is not a line
    let clip = io::copy(&sk, &[EntRef::line(0)]);
    assert_eq!(clip.lines.len(), 1);
    assert_eq!(clip.points.len(), 2);
    assert!(clip.circles.is_empty() && clip.arcs.is_empty());
}

#[test]
fn copy_keeps_a_constraint_only_when_both_ends_came() {
    let mut sk = Sketch::new();
    let a = sk.point(0.0, 0.0, false, "");
    let b = sk.point(10.0, 0.0, false, "");
    let c = sk.point(10.0, 8.0, false, "");
    let l1 = sk.line(a, b);
    let l2 = sk.line(b, c);
    sk.add(Constraint::distance(EntRef::point(a), EntRef::point(b), 10.0));
    sk.add(Constraint::one_line(CKind::Horizontal, EntRef::line(l1)));
    sk.add(Constraint::two_line(CKind::Perpendicular, EntRef::line(l1), EntRef::line(l2)));

    let clip = io::copy(&sk, &[EntRef::line(l1)]);
    let kinds: Vec<CKind> = clip.user_constraints().iter().map(|c| c.kind).collect();
    // Distance and Horizontal live entirely on l1; Perpendicular reaches l2, which did not come
    assert_eq!(kinds, vec![CKind::Distance, CKind::Horizontal]);

    let both = io::copy(&sk, &[EntRef::line(l1), EntRef::line(l2)]);
    assert_eq!(both.user_constraints().len(), 3);
}

#[test]
fn copy_is_the_other_half_of_deleting_the_rest() {
    // the two operations share one rule, so what a copy keeps is what deleting everything else
    // would have kept — checked on a sketch with arcs, tangencies and dimensions on it
    let sk = examples::rect_fillets(100.0, 60.0, 10.0, 0.0);
    let picked = [EntRef::line(0), EntRef::arc(0)];
    let rest: Vec<EntRef> = sk
        .primitives()
        .into_iter()
        .filter(|e| !model::expand(&sk, &picked).contains(e))
        .collect();
    assert_eq!(io::dumps(&io::copy(&sk, &picked), Some(1)),
               io::dumps(&io::without(&sk, &rest, &[]), Some(1)));
}

#[test]
fn copy_carries_the_flags_and_the_dimension_placements() {
    let mut sk = Sketch::new();
    let a = sk.point(0.0, 0.0, true, "");
    let b = sk.point(10.0, 0.0, false, "");
    let l = sk.line(a, b);
    sk.lines[l].construction = true;
    let id = sk.add(Constraint::distance(EntRef::point(a), EntRef::point(b), 10.0));
    sk.placements.insert(id, (1.5, -7.0));

    let clip = io::copy(&sk, &[EntRef::line(l)]);
    assert!(clip.point_fixed(0), "a fixed point stays fixed");
    assert!(clip.lines[0].construction, "reference geometry stays reference geometry");
    let kept = clip.user_constraints()[0].id;
    assert_eq!(clip.placements.get(&kept), Some(&(1.5, -7.0)));
}

#[test]
fn paste_lands_beside_what_was_copied_and_brings_its_constraints() {
    let mut sk = examples::rect_fillets(100.0, 60.0, 10.0, 0.0);
    let before = (sk.points.len(), sk.lines.len(), sk.arcs.len(), sk.user_constraints().len());
    let clip = io::copy(&sk, &sk.primitives());
    let made = io::paste(&mut sk, &clip, 5.0, -3.0);

    assert_eq!(sk.points.len(), 2 * before.0);
    assert_eq!(sk.lines.len(), 2 * before.1);
    assert_eq!(sk.arcs.len(), 2 * before.2);
    assert_eq!(sk.user_constraints().len(), 2 * before.3);
    // the new entities come back in clipboard order, so the caller can select what it pasted
    assert_eq!(made.len(), clip.primitives().len());
    assert!(made.iter().all(|e| match e.kind {
        EntKind::Point => e.i() >= before.0,
        EntKind::Line => e.i() >= before.1,
        _ => true,
    }));
    // moved by exactly the offset asked for
    let (x0, y0) = sk.point_xy(0);
    let (x1, y1) = sk.point_xy(before.0);
    assert!((x1 - x0 - 5.0).abs() < 1e-9 && (y1 - y0 + 3.0).abs() < 1e-9, "{x1} {y1}");
    // and the copy holds together on its own
    assert!(solve(&mut sk, SolveOpts::default()).success);
}

#[test]
fn a_pasted_copy_is_independent_of_the_original() {
    let mut sk = Sketch::new();
    let a = sk.point(0.0, 0.0, false, "");
    let b = sk.point(10.0, 0.0, false, "");
    sk.add(Constraint::distance(EntRef::point(a), EntRef::point(b), 10.0));
    let clip = io::copy(&sk, &sk.primitives());
    io::paste(&mut sk, &clip, 0.0, 20.0);

    // the pasted Distance names the pasted points and nothing else
    let pasted = &sk.user_constraints()[1];
    assert_eq!(pasted.entities(), vec![EntRef::point(2), EntRef::point(3)]);
    // moving the original leaves the copy where it is
    let mut x = sk.get_x();
    x[0] += 100.0;
    sk.set_x(&x);
    assert_eq!(sk.point_xy(2), (0.0, 20.0));
}

#[test]
fn pasting_twice_gives_two_copies() {
    let mut sk = Sketch::new();
    let a = sk.point(0.0, 0.0, false, "");
    let b = sk.point(10.0, 0.0, false, "");
    sk.add(Constraint::distance(EntRef::point(a), EntRef::point(b), 10.0));
    let clip = io::copy(&sk, &sk.primitives());
    io::paste(&mut sk, &clip, 0.0, 20.0);
    io::paste(&mut sk, &clip, 0.0, 40.0);
    assert_eq!(sk.points.len(), 6);
    assert_eq!(sk.user_constraints().len(), 3);
    assert_eq!(sk.point_xy(4), (0.0, 40.0));
    io::dumps(&sk, Some(1)); // every constraint still references a live entity
}

#[test]
fn copying_nothing_gives_an_empty_sketch_and_pastes_as_nothing() {
    let mut sk = examples::rect_fillets(100.0, 60.0, 10.0, 0.0);
    let before = io::dumps(&sk, Some(1));
    let clip = io::copy(&sk, &[]);
    assert!(clip.primitives().is_empty() && clip.constraints.is_empty());
    assert!(io::paste(&mut sk, &clip, 5.0, 5.0).is_empty());
    assert_eq!(io::dumps(&sk, Some(1)), before);
}

#[test]
fn a_clipboard_is_a_document() {
    // the fragment is an ordinary sketch, so it saves, loads and pastes like one — which is what
    // makes a copied selection something you can keep
    let sk = examples::slotted_link(80.0, 15.0, 6.0);
    let clip = io::copy(&sk, &sk.primitives());
    let text = io::dumps(&clip, Some(1));
    let reloaded = io::loads(&text).unwrap();
    assert_eq!(io::dumps(&reloaded, Some(1)), text);

    let mut fresh = Sketch::new();
    io::paste(&mut fresh, &reloaded, 0.0, 0.0);
    assert_eq!(fresh.points.len(), sk.points.len());
    assert_eq!(fresh.user_constraints().len(), sk.user_constraints().len());
}

/// The part around a point is what a drag of it can move: reached through shared points and
/// constraints, and stopping at fixed entities — a wall comes along (a constraint naming it keeps
/// all of its entities) but nothing is reached through it.  What it exchanges by point index
/// crosses through its maps, and writing back moves only what it holds.
#[test]
fn a_part_stops_at_a_wall_and_writes_back_only_itself() {
    use gcs_core::decompose::branch_key;
    use gcs_core::io::Part;
    let mut sk = Sketch::new();
    let a = sk.point(0.0, 0.0, false, "a");
    let w = sk.point(10.0, 0.0, true, "w");
    let b = sk.point(20.0, 0.0, false, "b");
    let c = sk.point(30.0, 5.0, false, "c");
    let lone = sk.point(50.0, 50.0, false, "lone");
    let l1 = sk.line(a, w);
    sk.line(w, b);
    sk.add(Constraint::one_line(CKind::Horizontal, EntRef::line(l1)));
    sk.add(Constraint::distance(EntRef::point(b), EntRef::point(c), 7.0));
    sk.branches.insert(branch_key([w, b, c]), -1);

    let left = Part::around(&sk, EntRef::point(a));
    assert_eq!((left.sketch.points.len(), left.sketch.lines.len()), (2, 1));
    assert_eq!(left.sketch.user_constraints().len(), 1);
    assert!(left.sketch.point_fixed(left.point_in(w).unwrap()));
    assert_eq!(left.point_in(b), None);
    assert_eq!(left.point_out(left.point_in(a).unwrap()), a);
    assert!(left.sketch.branches.is_empty());

    let mut right = Part::around(&sk, EntRef::point(b));
    assert_eq!((right.sketch.points.len(), right.sketch.lines.len()), (3, 1));
    assert_eq!(right.sketch.user_constraints()[0].kind, CKind::Distance);
    assert_eq!(right.point_in(a), None);
    assert_eq!(right.point_in(lone), None);
    assert_eq!(right.sketch.lines[0].p2 as usize, right.point_in(b).unwrap());
    assert_eq!(right.triangle_in((a, b, c)), None);
    let t = right.triangle_in((w, b, c)).unwrap();
    assert_eq!(right.triangle_out(t), (w, b, c));
    assert_eq!(right.sketch.branches.len(), 1);
    assert_eq!(right.branches_out(&right.sketch.branches), sk.branches);

    let pb = right.point_in(b).unwrap();
    let px = right.sketch.points[pb].x as usize;
    right.sketch.params[px].value = 99.0;
    let before = sk.get_x();
    right.write_back(&mut sk);
    assert_eq!(sk.point_xy(b).0, 99.0);
    let bx = sk.points[b].x as usize;
    for (i, (p, q)) in before.iter().zip(sk.get_x()).enumerate() {
        assert!(i == bx || *p == q, "param {i} moved");
    }
    assert_eq!(sk.points.len(), 5, "the document was restructured");
}

#[test]
fn a_relation_is_the_same_whatever_number_it_states() {
    use gcs_core::constraints::same_relation;
    let mut sk = Sketch::new();
    let a = sk.point(0.0, 0.0, false, "a");
    let b = sk.point(10.0, 0.0, false, "b");
    let c = sk.point(0.0, 10.0, false, "c");
    let (pa, pb, pc) = (EntRef::point(a), EntRef::point(b), EntRef::point(c));

    let d80 = Constraint::distance(pa, pb, 80.0);
    let d60 = Constraint::distance(pa, pb, 60.0);
    assert!(same_relation(&d80, &d60), "two widths on one pair are one relation");
    assert!(!same_constraint(&d80, &d60), "but not the same constraint — the numbers differ");
    // commutative, like `same_constraint`
    assert!(same_relation(&d80, &Constraint::distance(pb, pa, 1.0)));
    // a different pair is a different relation, and so is a different type
    assert!(!same_relation(&d80, &Constraint::distance(pa, pc, 80.0)));
    assert!(!same_relation(&d80, &Constraint::new(CKind::Coincident,
                                                  vec![Arg::Ent(pa), Arg::Ent(pb)])));

    // a flag is part of what a constraint says, so it still separates two of them
    let l = sk.line(a, b);
    let circle = sk.circle(c, 4.0, "c0");
    let (le, ce) = (EntRef::line(l), EntRef::circle(circle));
    let left = Constraint::tangent_line_circle(&sk, le, ce, Some(1));
    let right = Constraint::tangent_line_circle(&sk, le, ce, Some(-1));
    assert!(!same_relation(&left, &right), "the two sides are two relations");
}

#[test]
fn two_points_can_be_levelled_without_a_line_between_them() {
    let mut sk = Sketch::new();
    let a = sk.point(0.0, 0.0, true, "a");
    let b = sk.point(10.0, 4.0, false, "b");
    let c = sk.point(3.0, 9.0, false, "c");
    let (pa, pb, pc) = (EntRef::point(a), EntRef::point(b), EntRef::point(c));
    sk.add(Constraint::new(CKind::HorizontalPoints, vec![Arg::Ent(pa), Arg::Ent(pb)]));
    sk.add(Constraint::new(CKind::VerticalPoints, vec![Arg::Ent(pa), Arg::Ent(pc)]));
    assert!(solve(&mut sk, SolveOpts::default()).success);
    assert!((sk.point_xy(b).1 - sk.point_xy(a).1).abs() < 1e-9, "not level");
    assert!((sk.point_xy(c).0 - sk.point_xy(a).0).abs() < 1e-9, "not plumb");
    // one equation each, as for a levelled line
    assert_eq!(diagnose(&mut sk, DiagnoseOptions::default()).dof, 4 - 2);
}

#[test]
fn levelling_a_pair_is_the_same_statement_either_way_round() {
    let mut sk = Sketch::new();
    let a = sk.point(0.0, 0.0, false, "a");
    let b = sk.point(10.0, 4.0, false, "b");
    let (pa, pb) = (EntRef::point(a), EntRef::point(b));
    let ab = Constraint::new(CKind::HorizontalPoints, vec![Arg::Ent(pa), Arg::Ent(pb)]);
    let ba = Constraint::new(CKind::HorizontalPoints, vec![Arg::Ent(pb), Arg::Ent(pa)]);
    assert!(same_constraint(&ab, &ba), "a duplicate the other way round is still a duplicate");
    // and it is not the same as the plumb one
    let v = Constraint::new(CKind::VerticalPoints, vec![Arg::Ent(pa), Arg::Ent(pb)]);
    assert!(!same_constraint(&ab, &v));
}

#[test]
fn a_levelled_pair_decomposes_like_a_levelled_line() {
    use gcs_core::cgraph;
    // it must not fall to the numeric residue: a drag touching one would then cost the document
    let mut sk = Sketch::new();
    let a = sk.point(0.0, 0.0, true, "a");
    let b = sk.point(10.0, 4.0, false, "b");
    sk.add(Constraint::new(
        CKind::HorizontalPoints,
        vec![Arg::Ent(EntRef::point(a)), Arg::Ent(EntRef::point(b))],
    ));
    sk.add(Constraint::distance(EntRef::point(a), EntRef::point(b), 10.0));
    let g = cgraph::build(&sk);
    assert!(g.unsupported.is_empty(), "the levelled pair went to the numeric fallback");
    let mut ps = gcs_core::decompose::PlanSolver::new(&sk, false);
    assert!(ps.plan.fully_decomposed(), "{}", ps.plan.summary());
    let r = ps.solve(&mut sk, 1e-9, false, gcs_core::newton::Method::DogLeg);
    assert!(r.success && !r.fell_back, "{r:?}");
    assert!((sk.point_xy(b).1).abs() < 1e-9);
    assert!((sk.point_xy(b).0 - 10.0).abs() < 1e-9);
}
