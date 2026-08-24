//! Dimension callouts: the drafting figure every dimensioned constraint gets on the drawing.

use gcs_core::callout::{drag, grab, layout, pick, reset, Callout, CalloutKind, Seg};
use gcs_core::constraints::{Arg, CKind, Constraint, ALL_KINDS};
use gcs_core::examples;
use gcs_core::io;
use gcs_core::model::{EntRef, Sketch};
use gcs_core::solve::{solve, SolveOpts};
use std::f64::consts::PI;

/// One sketch with every kind of dimension on it: a slot with a shoulder, and two rings.
///
/// Distance across the slot, PointLineDistance from the free point to its floor,
/// ParallelDistance between floor and ceiling, Angle at the shoulder, Radius on the outer
/// circle, AnnularDistance between the two, and the run and rise from the free point to the
/// shoulder.
fn all_dimensions() -> Sketch {
    let mut sk = Sketch::new();
    let a = sk.point(0.0, 0.0, true, "");
    let b = sk.point(80.0, 0.0, false, "");
    let c = sk.point(80.0, 40.0, false, "");
    let d = sk.point(0.0, 40.0, false, "");
    let e = sk.point(30.0, 25.0, false, "");
    let floor = sk.line(a, b);
    let riser = sk.line(b, c);
    let ceil = sk.line(d, c);
    let ctr = sk.point(140.0, 20.0, false, "");
    let outer = sk.circle(ctr, 25.0, "");
    let inner = sk.circle(ctr, 15.0, "");
    let (lf, lr, lc) = (EntRef::line(floor), EntRef::line(riser), EntRef::line(ceil));
    let (co, ci) = (EntRef::circle(outer), EntRef::circle(inner));
    sk.add(Constraint::distance(EntRef::point(a), EntRef::point(b), 80.0));
    sk.add(Constraint::new(
        CKind::PointLineDistance,
        vec![Arg::Ent(EntRef::point(e)), Arg::Ent(lf), Arg::Num(25.0)],
    ));
    sk.add(Constraint::new(
        CKind::ParallelDistance,
        vec![Arg::Ent(lf), Arg::Ent(lc), Arg::Num(40.0)],
    ));
    sk.add(Constraint::new(
        CKind::Angle,
        vec![Arg::Ent(lf), Arg::Ent(lr), Arg::Num(PI / 2.0)],
    ));
    sk.add(Constraint::radius(co, 25.0));
    sk.add(Constraint::new(
        CKind::AnnularDistance,
        vec![Arg::Ent(ci), Arg::Ent(co), Arg::Num(10.0)],
    ));
    // the run and the rise from the free point up to the shoulder
    sk.add(Constraint::new(
        CKind::HorizontalDistance,
        vec![Arg::Ent(EntRef::point(e)), Arg::Ent(EntRef::point(c)), Arg::Num(50.0)],
    ));
    sk.add(Constraint::new(
        CKind::VerticalDistance,
        vec![Arg::Ent(EntRef::point(e)), Arg::Ent(EntRef::point(c)), Arg::Num(15.0)],
    ));
    sk
}

/// One dimensioned span, with the constraint's id to hand.
fn spanned() -> (Sketch, u32) {
    let mut sk = Sketch::new();
    let a = sk.point(0.0, 0.0, true, "");
    let b = sk.point(60.0, 0.0, false, "");
    let id = sk.add(Constraint::distance(EntRef::point(a), EntRef::point(b), 60.0));
    (sk, id)
}

/// How far a linear callout's dimension line stands off what it measures.
fn stand_off(k: &Callout) -> f64 {
    k.solid[0].0 .1.abs()
}

fn finite(p: (f64, f64)) -> bool {
    p.0.is_finite() && p.1.is_finite()
}

/// Every segment, arc and anchor a callout hands over is a real place on the drawing.
fn sane(k: &Callout) {
    assert!(!k.text.is_empty(), "a dimension with no number");
    assert!(finite(k.anchor), "{}: anchor {:?}", k.text, k.anchor);
    assert!(k.label.iter().all(|&p| finite(p)), "{}: label box", k.text);
    assert!(k.angle.is_finite(), "{}: angle", k.text);
    for s in k.solid.iter().chain(&k.thin) {
        assert!(finite(s.0) && finite(s.1), "{}: segment {:?}", k.text, s);
    }
    for a in &k.arcs {
        assert!(finite(a.c) && a.r.is_finite() && a.r > 0.0, "{}: arc", k.text);
        assert!(a.a0.is_finite() && a.a1.is_finite(), "{}: arc angles", k.text);
    }
    for a in &k.arrows {
        assert!(finite(a.at), "{}: arrow tip", k.text);
        assert!((a.dir.0.hypot(a.dir.1) - 1.0).abs() < 1e-9, "{}: arrow dir not a unit", k.text);
    }
    // there is something to see, and so something to click on
    assert!(!k.solid.is_empty() || !k.arcs.is_empty(), "{}: nothing drawn", k.text);
}

#[test]
fn every_dimension_is_drawn() {
    // live fire: the fixture holds one constraint of every kind that carries a number, and each
    // one has to come back as a figure.  `Pen::one` matches `CKind` exhaustively, so a new type
    // stops the build there; this is the other half — that the arm someone wrote actually draws.
    let mut drawn: Vec<CKind> = sk_kinds(&all_dimensions());
    drawn.sort();
    let mut want: Vec<CKind> = ALL_KINDS.iter().copied().filter(|k| k.has_dimension()).collect();
    want.sort();
    assert_eq!(drawn, want, "the fixture is missing a dimensioned kind");

    let sk = all_dimensions();
    let ks = layout(&sk, 1.0);
    assert_eq!(ks.len(), sk.constraints.len(), "one callout per dimension");
    for k in &ks {
        sane(k);
    }
    // the number on each is the one `describe` would print
    assert!(ks.iter().any(|k| k.text == "80"), "{:?}", texts(&ks));
    assert!(ks.iter().any(|k| k.text == "R25"), "{:?}", texts(&ks));
    assert!(ks.iter().any(|k| k.text == "90°"), "{:?}", texts(&ks));
}

fn texts(ks: &[Callout]) -> Vec<&str> {
    ks.iter().map(|k| k.text.as_str()).collect()
}

fn sk_kinds(sk: &Sketch) -> Vec<CKind> {
    sk.constraints.iter().map(|c| c.kind).collect()
}

#[test]
fn a_callout_belongs_to_its_constraint() {
    let sk = all_dimensions();
    let ks = layout(&sk, 1.0);
    for k in &ks {
        assert!(sk.constraint(k.id).is_some(), "callout {} names no constraint", k.id);
    }
}

#[test]
fn heads_face_each_other_across_the_measurement() {
    let (sk, _) = spanned();
    let ks = layout(&sk, 1.0);
    let k = &ks[0];
    assert_eq!(k.arrows.len(), 2);
    // the dimension line runs the length of what it measures, and each head sits on one end of
    // it pointing outward along it
    let d = k.solid[0];
    let span = (d.1 .0 - d.0 .0).hypot(d.1 .1 - d.0 .1);
    assert!((span - 60.0).abs() < 1e-9, "span {span}");
    assert!((k.arrows[0].dir.0 + 1.0).abs() < 1e-9, "{:?}", k.arrows[0].dir);
    assert!((k.arrows[1].dir.0 - 1.0).abs() < 1e-9, "{:?}", k.arrows[1].dir);
    // and it stands off the segment it measures rather than lying on top of it
    assert!(d.0 .1.abs() > 10.0, "the dimension line is on the geometry");
}

#[test]
fn a_short_span_puts_the_heads_outside() {
    let mut sk = Sketch::new();
    let a = sk.point(0.0, 0.0, true, "");
    let b = sk.point(2.0, 0.0, false, "");
    sk.add(Constraint::distance(EntRef::point(a), EntRef::point(b), 2.0));
    let k = &layout(&sk, 1.0)[0];
    // heads turned inward, and the number moved out past the far end rather than shrunk
    assert!((k.arrows[0].dir.0 - 1.0).abs() < 1e-9, "{:?}", k.arrows[0].dir);
    assert!((k.arrows[1].dir.0 + 1.0).abs() < 1e-9, "{:?}", k.arrows[1].dir);
    assert!(k.anchor.0 > 2.0, "the number should clear the span: {:?}", k.anchor);
}

#[test]
fn the_layout_is_screen_constant() {
    let (sk, _) = spanned();
    let one = layout(&sk, 1.0);
    let two = layout(&sk, 2.0);
    // half the zoom, twice the world length of a pixel — so a stand-off that is a fixed number
    // of pixels is twice as far out in world terms, and comes out the same size on screen
    let off = |ks: &[Callout]| stand_off(&ks[0]);
    assert!((off(&two) - 2.0 * off(&one)).abs() < 1e-9, "{} vs {}", off(&one), off(&two));
    // what it measures does not move
    let span = |ks: &[Callout]| ks[0].solid[0].1 .0 - ks[0].solid[0].0 .0;
    assert!((span(&one) - span(&two)).abs() < 1e-9);
}

#[test]
fn dimensions_over_the_same_span_stack() {
    let (mut sk, _) = spanned();
    let (a, b) = (EntRef::point(0), EntRef::point(1));
    sk.add(Constraint::distance(a, b, 60.0));
    let ks = layout(&sk, 1.0);
    assert!(stand_off(&ks[1]) > stand_off(&ks[0]) + 5.0,
            "{} vs {}", stand_off(&ks[0]), stand_off(&ks[1]));
}

#[test]
fn dimensions_side_by_side_on_one_line_do_not() {
    // a chord with a dimension on every bay is a row: they share the line but not any stretch
    // of it, so nothing has to move out of anything's way
    let mut sk = Sketch::new();
    let p: Vec<usize> = (0..4).map(|i| sk.point(20.0 * i as f64, 0.0, i == 0, "")).collect();
    for i in 0..3 {
        sk.add(Constraint::distance(EntRef::point(p[i]), EntRef::point(p[i + 1]), 20.0));
    }
    let ks = layout(&sk, 1.0);
    assert!(ks.iter().all(|k| (stand_off(k) - stand_off(&ks[0])).abs() < 1e-9),
            "a staircase, not a row");
}

#[test]
fn the_two_sides_of_a_line_are_different_lanes() {
    // a rectangle's floor and its ceiling run in the same direction; the dimension under one
    // and the one over the other are not in each other's way
    let mut sk = Sketch::new();
    let a = sk.point(0.0, 0.0, true, "");
    let b = sk.point(60.0, 0.0, false, "");
    let c = sk.point(0.0, 40.0, false, "");
    let d = sk.point(60.0, 40.0, false, "");
    sk.add(Constraint::distance(EntRef::point(a), EntRef::point(b), 60.0));
    sk.add(Constraint::distance(EntRef::point(c), EntRef::point(d), 60.0));
    let ks = layout(&sk, 1.0);
    // one stands off below the floor, the other above the ceiling — neither pushed out
    assert!(ks[0].solid[0].0 .1 < 0.0, "{:?}", ks[0].solid[0]);
    assert!(ks[1].solid[0].0 .1 > 40.0, "{:?}", ks[1].solid[0]);
    let out = |k: &Callout, base: f64| (k.solid[0].0 .1 - base).abs();
    assert!((out(&ks[0], 0.0) - out(&ks[1], 40.0)).abs() < 1e-9, "one of them was bumped");
}

#[test]
fn an_angle_arc_sweeps_the_angle_it_names() {
    let mut sk = Sketch::new();
    let o = sk.point(0.0, 0.0, true, "");
    let a = sk.point(50.0, 0.0, false, "");
    let b = sk.point(30.0, 30.0, false, "");
    let l1 = sk.line(o, a);
    let l2 = sk.line(o, b);
    sk.add(Constraint::new(
        CKind::Angle,
        vec![Arg::Ent(EntRef::line(l1)), Arg::Ent(EntRef::line(l2)), Arg::Num(PI / 4.0)],
    ));
    assert!(solve(&mut sk, SolveOpts::default()).success);
    let k = &layout(&sk, 1.0)[0];
    assert_eq!(k.kind, CalloutKind::Angular);
    let arc = k.arcs[0];
    assert!(arc.c.0.abs() < 1e-6 && arc.c.1.abs() < 1e-6, "corner {:?}", arc.c);
    assert!((arc.a1 - arc.a0 - PI / 4.0).abs() < 1e-6, "sweep {}", arc.a1 - arc.a0);
    // the label sits outside the arc, where a drawing puts it
    let r = (k.anchor.0 - arc.c.0).hypot(k.anchor.1 - arc.c.1);
    assert!(r > arc.r, "the number is inside the arc");
}

#[test]
fn a_negative_angle_sweeps_the_other_way() {
    let mut sk = Sketch::new();
    let o = sk.point(0.0, 0.0, true, "");
    let a = sk.point(50.0, 0.0, false, "");
    let b = sk.point(30.0, -30.0, false, "");
    let l1 = sk.line(o, a);
    let l2 = sk.line(o, b);
    sk.add(Constraint::new(
        CKind::Angle,
        vec![Arg::Ent(EntRef::line(l1)), Arg::Ent(EntRef::line(l2)), Arg::Num(-PI / 4.0)],
    ));
    assert!(solve(&mut sk, SolveOpts::default()).success);
    let arc = layout(&sk, 1.0)[0].arcs[0];
    assert!(arc.a1 < arc.a0, "a clockwise angle should sweep back: {} {}", arc.a0, arc.a1);
}

#[test]
fn parallel_lines_get_a_note_rather_than_an_arc() {
    // an Angle whose lines have gone parallel has its corner at infinity — the number still has
    // to appear, so it comes back as a leader with no arc
    let mut sk = Sketch::new();
    let a = sk.point(0.0, 0.0, true, "");
    let b = sk.point(50.0, 0.0, true, "");
    let c = sk.point(0.0, 20.0, true, "");
    let d = sk.point(50.0, 20.0, true, "");
    let l1 = sk.line(a, b);
    let l2 = sk.line(c, d);
    sk.add(Constraint::new(
        CKind::Angle,
        vec![Arg::Ent(EntRef::line(l1)), Arg::Ent(EntRef::line(l2)), Arg::Num(0.0)],
    ));
    let k = &layout(&sk, 1.0)[0];
    assert!(k.arcs.is_empty());
    assert_eq!(k.text, "0°");
    sane(k);
}

#[test]
fn a_radius_head_lands_on_the_rim() {
    let mut sk = Sketch::new();
    let ctr = sk.point(10.0, 10.0, true, "");
    let c = sk.circle(ctr, 25.0, "");
    sk.add(Constraint::radius(EntRef::circle(c), 25.0));
    let k = &layout(&sk, 1.0)[0];
    assert_eq!(k.kind, CalloutKind::Radial);
    let tip = k.arrows[0].at;
    let r = (tip.0 - 10.0).hypot(tip.1 - 10.0);
    assert!((r - 25.0).abs() < 1e-9, "the head is at {r}, not on the rim");
    // the leader comes out of the centre, and the label clears the shape
    let anchor_r = (k.anchor.0 - 10.0).hypot(k.anchor.1 - 10.0);
    assert!(anchor_r > 25.0, "the number is inside the circle");
}

#[test]
fn a_radius_leader_stays_on_the_arc_it_measures() {
    let mut sk = Sketch::new();
    let ctr = sk.point(0.0, 0.0, true, "");
    let s = sk.point(20.0, 0.0, false, "");
    let e = sk.point(0.0, 20.0, false, "");
    let a = sk.arc(ctr, s, e, "");
    sk.add(Constraint::radius(EntRef::arc(a), 20.0));
    let ks = layout(&sk, 1.0);
    // the arc's own definition is not a dimension, so only the radius is drawn
    assert_eq!(ks.len(), 1, "{:?}", texts(&ks));
    let tip = ks[0].arrows[0].at;
    let th = tip.1.atan2(tip.0);
    let (a0, a1) = sk.arc_angles(a);
    assert!(th > a0 && th < a1, "the head is off the drawn sweep: {th} not in {a0}..{a1}");
}

#[test]
fn concentric_leaders_do_not_land_on_each_other() {
    let mut sk = Sketch::new();
    let ctr = sk.point(0.0, 0.0, true, "");
    let outer = sk.circle(ctr, 25.0, "");
    let inner = sk.circle(ctr, 15.0, "");
    sk.add(Constraint::radius(EntRef::circle(outer), 25.0));
    sk.add(Constraint::radius(EntRef::circle(inner), 15.0));
    let ks = layout(&sk, 1.0);
    let dir = |k: &Callout| {
        let t = k.arrows[0].at;
        t.1.atan2(t.0)
    };
    assert!((dir(&ks[0]) - dir(&ks[1])).abs() > 0.1, "the two leaders came out together");
}

#[test]
fn a_point_line_offset_is_measured_on_the_perpendicular() {
    let mut sk = Sketch::new();
    let a = sk.point(0.0, 0.0, true, "");
    let b = sk.point(100.0, 0.0, true, "");
    let p = sk.point(30.0, 12.0, false, "");
    let l = sk.line(a, b);
    sk.add(Constraint::new(
        CKind::PointLineDistance,
        vec![Arg::Ent(EntRef::point(p)), Arg::Ent(EntRef::line(l)), Arg::Num(12.0)],
    ));
    let k = &layout(&sk, 1.0)[0];
    let d = k.solid[0];
    // foot to point, right where it is measured — one end on the line, the other on the point
    assert!(d.0 .1.abs() < 1e-9 && (d.0 .0 - 30.0).abs() < 1e-9, "{:?}", d.0);
    assert!((d.1 .0 - 30.0).abs() < 1e-9 && (d.1 .1 - 12.0).abs() < 1e-9, "{:?}", d.1);
}

#[test]
fn a_foot_off_the_end_gets_a_witness_line() {
    let mut sk = Sketch::new();
    let a = sk.point(0.0, 0.0, true, "");
    let b = sk.point(20.0, 0.0, true, "");
    let p = sk.point(60.0, 12.0, false, "");
    let l = sk.line(a, b);
    sk.add(Constraint::new(
        CKind::PointLineDistance,
        vec![Arg::Ent(EntRef::point(p)), Arg::Ent(EntRef::line(l)), Arg::Num(12.0)],
    ));
    let k = &layout(&sk, 1.0)[0];
    // the distance is to the infinite line, so the stretch of it that is not drawn is shown
    assert!(
        k.thin.iter().any(|s| (s.0 .0 - 20.0).abs() < 1e-9 && (s.1 .0 - 60.0).abs() < 1e-9),
        "{:?}",
        k.thin
    );
}

#[test]
fn a_parallel_gap_crosses_between_the_two_lines() {
    let mut sk = Sketch::new();
    let a = sk.point(0.0, 0.0, true, "");
    let b = sk.point(50.0, 0.0, true, "");
    let c = sk.point(0.0, 18.0, true, "");
    let d = sk.point(50.0, 18.0, true, "");
    let l1 = sk.line(a, b);
    let l2 = sk.line(c, d);
    sk.add(Constraint::new(
        CKind::ParallelDistance,
        vec![Arg::Ent(EntRef::line(l1)), Arg::Ent(EntRef::line(l2)), Arg::Num(18.0)],
    ));
    let k = &layout(&sk, 1.0)[0];
    let s = k.solid[0];
    assert!(s.0 .1.abs() < 1e-9, "one end belongs on the first line: {:?}", s.0);
    assert!((s.1 .1 - 18.0).abs() < 1e-9, "the other on the second: {:?}", s.1);
    assert!((s.0 .0 - 25.0).abs() < 1e-6, "it should cross where they overlap: {:?}", s.0);
}

#[test]
fn a_ring_is_measured_out_along_one_ray() {
    let mut sk = Sketch::new();
    let ctr = sk.point(0.0, 0.0, true, "");
    let inner = sk.circle(ctr, 15.0, "");
    let outer = sk.circle(ctr, 25.0, "");
    sk.add(Constraint::new(
        CKind::AnnularDistance,
        vec![Arg::Ent(EntRef::circle(inner)), Arg::Ent(EntRef::circle(outer)), Arg::Num(10.0)],
    ));
    let k = &layout(&sk, 1.0)[0];
    let s = k.solid[0];
    let (r0, r1) = (s.0 .0.hypot(s.0 .1), s.1 .0.hypot(s.1 .1));
    assert!((r0 - 15.0).abs() < 1e-9 && (r1 - 25.0).abs() < 1e-9, "{r0} {r1}");
    // and out along the same ray, so the segment drawn *is* the thickness
    let gap = (s.1 .0 - s.0 .0).hypot(s.1 .1 - s.0 .1);
    assert!((gap - 10.0).abs() < 1e-9, "gap {gap}");
}

#[test]
fn soft_and_intrinsic_constraints_are_not_dimensions() {
    let mut sk = Sketch::new();
    let ctr = sk.point(0.0, 0.0, true, "");
    let s = sk.point(20.0, 0.0, false, "");
    let e = sk.point(0.0, 20.0, false, "");
    sk.arc(ctr, s, e, ""); // brings two intrinsic PointOnCircle constraints with it
    sk.add(Constraint::drag_target(EntRef::point(s), 21.0, 1.0, 1.0));
    assert!(sk.constraints.len() >= 3);
    assert!(layout(&sk, 1.0).is_empty(), "only dimensions are called out");
}

#[test]
fn a_zero_length_distance_still_shows_its_number() {
    let mut sk = Sketch::new();
    let a = sk.point(10.0, 10.0, true, "");
    let b = sk.point(10.0, 10.0, false, "");
    sk.add(Constraint::distance(EntRef::point(a), EntRef::point(b), 5.0));
    let ks = layout(&sk, 1.0);
    assert_eq!(ks.len(), 1);
    assert_eq!(ks[0].text, "5");
    sane(&ks[0]);
}

#[test]
fn every_example_lays_out() {
    for name in examples::EXAMPLES {
        let sk = examples::example(name).unwrap();
        let ks = layout(&sk, 0.2);
        let dims = sk
            .constraints
            .iter()
            .filter(|c| !c.soft && !c.intrinsic && c.kind.has_dimension())
            .count();
        assert_eq!(ks.len(), dims, "{name}");
        for k in &ks {
            sane(k);
        }
        // an angular callout has an arc unless its lines went parallel, in which case it is a
        // note with nothing but a leader
        for k in ks.iter().filter(|k| k.kind == CalloutKind::Angular) {
            assert!(!k.arcs.is_empty() || k.thin.is_empty(), "{name}");
        }
    }
}

#[test]
fn the_layout_survives_a_degenerate_unit() {
    let sk = all_dimensions();
    for unit in [0.0, -1.0, f64::NAN, f64::INFINITY] {
        let ks = layout(&sk, unit);
        assert_eq!(ks.len(), sk.constraints.len(), "unit {unit}");
        for k in &ks {
            sane(k);
        }
    }
}

/* -- moving a callout about ---------------------------------------------------------- */

fn dim_line(sk: &Sketch) -> Seg {
    layout(sk, 1.0)[0].solid[0]
}

#[test]
fn a_dragged_callout_goes_where_it_is_put() {
    let (mut sk, id) = spanned();
    let start = dim_line(&sk).0;
    // take hold of the dimension line and pull it the other side of what it measures
    let grip = grab(&sk, 1.0, id, start).unwrap();
    assert!(drag(&mut sk, id, (start.0, -25.0), grip));
    let moved = dim_line(&sk).0;
    assert!((moved.1 + 25.0).abs() < 1e-9, "{:?}", moved);
    assert!((moved.0 - start.0).abs() < 1e-9, "it should not have slid along: {:?}", moved);
}

#[test]
fn a_grip_keeps_the_callout_under_the_hand() {
    let (mut sk, id) = spanned();
    let line = dim_line(&sk);
    // grab 7 above the line, then let go 30 lower: the line moves 30, not 37
    let held = (line.0 .0, line.0 .1 + 7.0);
    let grip = grab(&sk, 1.0, id, held).unwrap();
    drag(&mut sk, id, (held.0, held.1 - 30.0), grip);
    assert!((dim_line(&sk).0 .1 - (line.0 .1 - 30.0)).abs() < 1e-9);
}

#[test]
fn a_placement_holds_still_while_the_sketch_moves() {
    // the placement is written in a frame that follows the geometry, so a dimension dragged
    // clear of the drawing stays clear of it as the drawing is dragged around
    let (mut sk, id) = spanned();
    let grip = grab(&sk, 1.0, id, dim_line(&sk).0).unwrap();
    drag(&mut sk, id, (10.0, 40.0), grip);
    let before = layout(&sk, 1.0)[0].place;
    let (line, anchor) = (dim_line(&sk), layout(&sk, 1.0)[0].anchor);
    let mut x = sk.get_x();
    for i in (0..x.len()).step_by(2) {
        x[i] += 100.0;                  // slide the whole span sideways
    }
    sk.set_x(&x);
    assert_eq!(layout(&sk, 1.0)[0].place, before, "the placement moved with the geometry");
    let (line2, anchor2) = (dim_line(&sk), layout(&sk, 1.0)[0].anchor);
    assert!((line2.0 .0 - line.0 .0 - 100.0).abs() < 1e-9, "{:?} → {:?}", line.0, line2.0);
    assert!((line2.0 .1 - line.0 .1).abs() < 1e-9);
    assert!((anchor2.0 - anchor.0 - 100.0).abs() < 1e-9, "the number stayed behind");
}

#[test]
fn re_placing_puts_a_callout_back() {
    let (mut sk, id) = spanned();
    let auto = dim_line(&sk);
    let grip = grab(&sk, 1.0, id, auto.0).unwrap();
    drag(&mut sk, id, (0.0, -40.0), grip);
    assert!(sk.placements.contains_key(&id));
    assert!(reset(&mut sk, id));
    assert!(sk.placements.is_empty());
    assert!(!reset(&mut sk, id), "a callout that was never moved is not an edit");
    assert!((dim_line(&sk).0 .1 - auto.0 .1).abs() < 1e-9);
}

#[test]
fn a_deleted_dimension_takes_its_placement_with_it() {
    let (mut sk, id) = spanned();
    let grip = grab(&sk, 1.0, id, dim_line(&sk).0).unwrap();
    drag(&mut sk, id, (0.0, -40.0), grip);
    sk.remove(id);
    assert!(sk.placements.is_empty(), "a placement outlived its constraint");
}

#[test]
fn placements_survive_a_save() {
    let mut sk = all_dimensions();
    let ids: Vec<u32> = sk.constraints.iter().map(|c| c.id).collect();
    // move every one of them somewhere of its own
    for (i, &id) in ids.iter().enumerate() {
        let k = layout(&sk, 1.0).into_iter().find(|k| k.id == id).unwrap();
        sk.placements.insert(id, (k.place.0 + 0.1 * (i as f64 + 1.0), k.place.1 * 1.3 + 1.0));
    }
    let before: Vec<(f64, f64)> = layout(&sk, 1.0).iter().map(|k| k.place).collect();
    let text = io::dumps(&sk, Some(1));
    let sk2 = io::loads(&text).unwrap();
    assert_eq!(sk2.placements.len(), sk.placements.len());
    let after: Vec<(f64, f64)> = layout(&sk2, 1.0).iter().map(|k| k.place).collect();
    assert_eq!(before, after);
    assert_eq!(io::dumps(&sk2, Some(1)), text, "a second round trip should be a no-op");
}

#[test]
fn a_placement_follows_its_dimension_through_a_deletion() {
    // `without` rebuilds the sketch, so every constraint is re-added under a fresh id — the
    // placements have to be carried across with them, not left pointing at the old ids
    let mut sk = all_dimensions();
    let radius = sk.constraints.iter().find(|c| c.kind == CKind::Radius).unwrap().id;
    sk.placements.insert(radius, (0.9, 44.0));
    let sk2 = io::without(&sk, &[EntRef::point(4)], &[]); // the free point, and its offset
    let kept = sk2.constraints.iter().find(|c| c.kind == CKind::Radius).unwrap().id;
    assert_eq!(sk2.placements.get(&kept), Some(&(0.9, 44.0)));
    assert_eq!(sk2.placements.len(), 1);
}

#[test]
fn a_click_picks_the_callout_it_lands_on() {
    let sk = all_dimensions();
    for k in layout(&sk, 1.0) {
        assert_eq!(pick(&sk, 1.0, k.anchor, 1.0), Some(k.id), "{} by its number", k.text);
        // an angular dimension is all arc, so it has no straight line to click on
        if let Some(s) = k.solid.first() {
            let midpoint = (0.5 * (s.0 .0 + s.1 .0), 0.5 * (s.0 .1 + s.1 .1));
            assert_eq!(pick(&sk, 1.0, midpoint, 1.0), Some(k.id), "{} by its line", k.text);
        }
        for a in &k.arcs {
            let on = (a.c.0 + a.r * a.a0.cos(), a.c.1 + a.r * a.a0.sin());
            assert_eq!(pick(&sk, 1.0, on, 1.0), Some(k.id), "{} by its arc", k.text);
        }
    }
    assert_eq!(pick(&sk, 1.0, (-500.0, -500.0), 1.0), None);
}

/// A radius runs its leader out of the centre, so the figure passes straight through the one
/// point a circle has.  The point is the smaller target and the thing most verbs are about, so
/// it wins there — but not under the number itself, which is painted solid.
#[test]
fn a_point_outranks_the_figure_drawn_over_it() {
    let mut sk = Sketch::new();
    let ctr = sk.point(0.0, 0.0, false, "");
    let circle = sk.circle(ctr, 25.0, "");
    let id = sk.add(Constraint::radius(EntRef::circle(circle), 25.0));
    let k = layout(&sk, 1.0).into_iter().next().unwrap();

    // out along the leader the callout is picked, and at the centre it is not
    let leader = k.solid[0];
    let along = (0.5 * (leader.0 .0 + leader.1 .0), 0.5 * (leader.0 .1 + leader.1 .1));
    assert_eq!(pick(&sk, 1.0, along, 6.0), Some(id));
    assert_eq!(pick(&sk, 1.0, (0.0, 0.0), 6.0), None, "the leader shadowed its own centre");
    assert_eq!(pick(&sk, 1.0, (7.0, 0.0), 6.0), Some(id), "a point out of reach vetoed it");

    // and where the number is put over a point, the number is what is there to be clicked
    sk.placements.insert(id, (0.0, 40.0));
    let k = layout(&sk, 1.0).into_iter().next().unwrap();
    let under = sk.point(k.anchor.0, k.anchor.1, false, "");
    assert_eq!(sk.nearest_point(k.anchor.0, k.anchor.1).0, Some(under));
    assert_eq!(pick(&sk, 1.0, k.anchor, 6.0), Some(id), "a point behind the label hid it");
}

#[test]
fn grabbing_a_constraint_that_has_no_callout_is_not_a_trap() {
    let mut sk = Sketch::new();
    let a = sk.point(0.0, 0.0, true, "");
    let b = sk.point(10.0, 0.0, false, "");
    let id = sk.add(Constraint::coincident(EntRef::point(a), EntRef::point(b)));
    assert_eq!(grab(&sk, 1.0, id, (0.0, 0.0)), None);
    assert!(!drag(&mut sk, id, (0.0, 0.0), (0.0, 0.0)));
    assert_eq!(grab(&sk, 1.0, 9999, (0.0, 0.0)), None);
    assert!(!drag(&mut sk, 9999, (0.0, 0.0), (0.0, 0.0)));
    assert!(!reset(&mut sk, 9999));
}

/// Two points, dimensioned three ways: which one it is comes from where the number is put.
#[test]
fn where_a_dimension_is_put_is_which_dimension_it_is() {
    use gcs_core::callout::pair_dimension;
    let (a, b) = ((0.0, 0.0), (40.0, 40.0));   // a diagonal pair, so all three are reachable
    let mid = (20.0, 20.0);
    let out = |dx: f64, dy: f64| (mid.0 + dx, mid.1 + dy);
    // across the pair is the length it already looked like it wanted
    assert_eq!(pair_dimension(a, b, out(-30.0, 30.0)), CKind::Distance);
    assert_eq!(pair_dimension(a, b, out(30.0, -30.0)), CKind::Distance, "and on the other side");
    // above or below it, the dimension line lies along the page's x: the run between them
    assert_eq!(pair_dimension(a, b, out(0.0, 40.0)), CKind::HorizontalDistance);
    assert_eq!(pair_dimension(a, b, out(3.0, -40.0)), CKind::HorizontalDistance);
    // out to either side, along y: the rise
    assert_eq!(pair_dimension(a, b, out(40.0, 0.0)), CKind::VerticalDistance);
    assert_eq!(pair_dimension(a, b, out(-40.0, -3.0)), CKind::VerticalDistance);
    // the borders are the bisectors, and nothing degenerate is an error
    assert_eq!(pair_dimension(a, b, mid), CKind::Distance, "nowhere in particular");
    assert_eq!(pair_dimension(a, a, out(0.0, 40.0)), CKind::Distance, "no pair to measure");
    // a level pair reads the same either way, so the length wins the tie
    assert_eq!(pair_dimension((0.0, 0.0), (40.0, 0.0), (20.0, 30.0)), CKind::Distance);
    assert_eq!(pair_dimension((0.0, 0.0), (40.0, 0.0), (60.0, 0.0)), CKind::VerticalDistance);
}

/// A run is drawn along the page, whatever the pair is doing: the dimension line is horizontal
/// and each extension line reaches its own point, however far apart across it the two are.
#[test]
fn a_run_is_drawn_along_the_page_and_reaches_both_points() {
    let mut sk = Sketch::new();
    let a = sk.point(0.0, 0.0, true, "");
    let b = sk.point(30.0, 20.0, false, "");
    let (pa, pb) = (EntRef::point(a), EntRef::point(b));
    let run = sk.add(Constraint::new(CKind::HorizontalDistance, vec![Arg::Ent(pa), Arg::Ent(pb),
                                                                     Arg::Num(30.0)]));
    let rise = sk.add(Constraint::new(CKind::VerticalDistance, vec![Arg::Ent(pa), Arg::Ent(pb),
                                                                    Arg::Num(20.0)]));
    let of = |sk: &Sketch, id: u32| layout(sk, 1.0).into_iter().find(|k| k.id == id).unwrap();

    let k = of(&sk, run);
    assert_eq!(k.text, "30");
    let Seg(p, q) = k.solid[0];
    assert!((p.1 - q.1).abs() < 1e-9, "the dimension line is not level: {p:?} {q:?}");
    assert!((p.0 - 0.0).abs() < 1e-9 && (q.0 - 30.0).abs() < 1e-9, "heads not over the points");
    // an extension line springs from each point and ends past the dimension line
    for pt in [(0.0, 0.0), (30.0, 20.0)] {
        assert!(k.thin.iter().any(|s| (s.0 .0 - pt.0).abs() < 1e-9 && (s.0 .1 - pt.1).abs() < 6.0),
                "nothing reaches {pt:?}: {:?}", k.thin);
    }

    let k = of(&sk, rise);
    assert_eq!(k.text, "20");
    let Seg(p, q) = k.solid[0];
    assert!((p.0 - q.0).abs() < 1e-9, "the dimension line is not plumb");
    assert!((p.1 - 0.0).abs() < 1e-9 && (q.1 - 20.0).abs() < 1e-9);

    // and they hold what they say: the run is set to 50 and only x moves
    sk.constraints.iter_mut().find(|c| c.id == run).unwrap().args[2] = Arg::Num(50.0);
    assert!(solve(&mut sk, SolveOpts::default()).success);
    assert!((sk.point_xy(b).0 - 50.0).abs() < 1e-9, "{:?}", sk.point_xy(b));
    assert!((sk.point_xy(b).1 - 20.0).abs() < 1e-9, "the rise moved with the run");
}
