//! Corner rounds (#576): `A -> round(r: R) B` rounds the corner where two lines of a chain meet.
//!
//! The joint mints the arc, a tangency at each end and the virtual corner on both lines, and the
//! elaborator turns it the way its corner turns, seeds it from the corner and states its branch —
//! so a rounded outline is written as its corners and solves with no seed of the round's own.
//! What is held here: each spelling is the longhand it is short for (the shipped `rounds/slot.sv`
//! pocket, a solid and its round to the last bit of volume), the branch the walk chooses (convex
//! and concave, one root), and a round that does not fit said by name.

use crate::common::gestures;
use crate::common::{build, dof, ent, refused, solved};
use gcs_core::model::Sketch;

/// A rectangle held at its corners, each corner rounded: the corners are the only numbers.
const RECT: &str = "\
use std
in std.front {
a := point
b := point
c := point
d := point
fix((0, 0)) a
fix((100, 0)) b
fix((100, 60)) c
fix((0, 60)) d
(bottom := line(a, b)) -> (br := round(r: 10)) (right := line(b, c)) -> (tr := round(r: 10))
  (top := line(c, d)) -> (tl := round(r: 10)) (left := line(d, a)) -> (bl := round(r: 10)) close
outline := face(bottom, br, right, tr, top, tl, left, bl)
}
";

/// The arcs of a solved sketch, each as (centre, start, end, radius).
fn arcs(sk: &Sketch) -> Vec<((f64, f64), (f64, f64), (f64, f64), f64)> {
    sk.arcs.iter().map(|a| (sk.point_xy(a.center as usize), sk.point_xy(a.start as usize),
                            sk.point_xy(a.end as usize), sk.params[a.radius as usize].value))
        .collect()
}

fn at(sk: &Sketch, p: u32) -> (f64, f64) {
    sk.point_xy(p as usize)
}

fn near(a: (f64, f64), b: (f64, f64)) -> bool {
    (a.0 - b.0).abs() < 1e-7 && (a.1 - b.1).abs() < 1e-7
}

/// How far an arc turns from its start to its end, counter-clockwise, in degrees.
fn sweep(c: (f64, f64), s: (f64, f64), e: (f64, f64)) -> f64 {
    let a = (s.1 - c.1).atan2(s.0 - c.0);
    let b = (e.1 - c.1).atan2(e.0 - c.0);
    (b - a).rem_euclid(std::f64::consts::TAU).to_degrees()
}

#[test]
fn a_rounded_rectangle_is_written_as_its_corners() {
    let e = build(RECT);
    let mut sk = solved(&e);
    assert_eq!(dof(&mut sk), 0);
    let want = [((90.0, 10.0), (90.0, 0.0), (100.0, 10.0)), ((90.0, 50.0), (100.0, 50.0),
        (90.0, 60.0)), ((10.0, 50.0), (10.0, 60.0), (0.0, 50.0)), ((10.0, 10.0), (0.0, 10.0),
        (10.0, 0.0))];
    for ((c, s, t, r), (wc, ws, wt)) in arcs(&sk).into_iter().zip(want) {
        assert!(near(c, wc) && near(s, ws) && near(t, wt), "{c:?} {s:?} {t:?}");
        assert!((r - 10.0).abs() < 1e-9);
        assert!((sweep(c, s, t) - 90.0).abs() < 1e-6);
    }
    // each line runs from one round to the next, and the corner it was written to stays on it
    let bottom = &sk.lines[ent(&e, "bottom").i()];
    assert!(near(at(&sk, bottom.p1), (10.0, 0.0)) && near(at(&sk, bottom.p2), (90.0, 0.0)));
    // the face walks the rounds: the rectangle less a square's corners, plus the quarter discs
    let area = 100.0 * 60.0 - 4.0 * 100.0 + std::f64::consts::PI * 100.0;
    let src = format!("{RECT}pad := solid(outline, depth: 5)\n");
    let g = gestures::build(&src);
    let v = gestures::volume(&g, "pad");
    assert!((v - 5.0 * area).abs() <= 1e-6 * v, "{v} against {}", 5.0 * area);
}

#[test]
fn every_round_states_its_arc_its_tangencies_and_its_corner() {
    let e = build(RECT);
    let said = crate::common::said(&e);
    for want in ["radius(10) br", "br tangent(at: bottom.p2) bottom",
                 "br tangent(at: right.p1) right", "b coincident bottom", "b coincident right"] {
        assert!(said.iter().any(|s| s == want), "{want} not in {said:#?}");
    }
    // three root choices a round: its centre inside both lines and its own chord
    assert_eq!(e.sketch.orientations.len(), 12);
}

/// An L-bracket's outline, its one inside corner filleted: the concave corner the walk turns
/// right at, so the arc runs clockwise as walked and its centre is outside the material.
const BRACKET: &str = "\
use std
in std.front {
p0 := point
p1 := point
p2 := point
p3 := point
p4 := point
p5 := point
fix((0, 0)) p0
fix((60, 0)) p1
fix((60, 10)) p2
fix((10, 10)) p3
fix((10, 50)) p4
fix((0, 50)) p5
(a := line(p0, p1)) -> (b := line(p1, p2)) -> (c := line(p2, p3)) -> (fillet := round(r: 5))
  (d := line(p3, p4)) -> (e := line(p4, p5)) -> (f := line(p5, p0)) -> close
outline := face(a, b, c, fillet, d, e, f)
}
";

#[test]
fn an_inside_fillet_turns_the_way_its_corner_does() {
    let e = build(BRACKET);
    let mut sk = solved(&e);
    assert_eq!(dof(&mut sk), 0);
    let k = ent(&e, "fillet").i();
    let arc = &sk.arcs[k];
    // drawn from its end: counter-clockwise from the point on `d` round to the point on `c`
    let (c, s, t) = (at(&sk, arc.center), at(&sk, arc.start), at(&sk, arc.end));
    assert!(near(c, (15.0, 15.0)), "{c:?}");
    assert!(near(s, (10.0, 15.0)) && near(t, (15.0, 10.0)), "{s:?} {t:?}");
    assert!((sweep(c, s, t) - 90.0).abs() < 1e-6);
    assert!(sk.walked_back.contains(&(k as u32)));
    // and the names are as the walk wrote them: its start on `c`, where the walk enters it
    let src = format!("{BRACKET}q := point\nq coincident fillet.start\n");
    let g = build(&src);
    let sk = solved(&g);
    // `q` stands in space, drawn where the arc's start is in the front plane
    let q = sk.world_point(ent(&g, "q").i());
    assert!(near((q[0], q[2]), (15.0, 10.0)), "{q:?}");
    // the bracket's area: the L, and the fillet's web between the corner's square and its disc
    let area = 60.0 * 10.0 + 10.0 * 40.0 + 25.0 - std::f64::consts::PI * 25.0 / 4.0;
    let g = gestures::build(&format!("{BRACKET}pad := solid(outline, depth: 2)\n"));
    let v = gestures::volume(&g, "pad");
    assert!((v - 2.0 * area).abs() <= 1e-6 * v, "{v} against {}", 2.0 * area);
}

/// `rounds/slot.sv` with its pocket written as four corners rounded at the joints — the shipped
/// longhand's lines, arcs and centres, said by the corners alone.
fn slot_by_joints() -> String {
    let src = std::fs::read_to_string(concat!(env!("CARGO_MANIFEST_DIR"),
                                              "/../examples/rounds/slot.sv")).unwrap();
    let from = src.find("  k0 := point").unwrap();
    let to = src.find("  mouth := face").unwrap();
    let to = to + src[to..].find('\n').unwrap() + 1;
    let pocket = "\
  k0 := point
  k1 := point
  k2 := point
  k3 := point
  fix((48, 17)) k0
  fix((68, 17)) k1
  fix((68, 33)) k2
  fix((48, 33)) k3
  (bottom := line(k0, k1)) -> (corner1 := round(r: 4)) (right := line(k1, k2)) ->
    (corner2 := round(r: 4)) (upper := line(k2, k3)) -> (corner3 := round(r: 4))
    (left := line(k3, k0)) -> (corner0 := round(r: 4)) close
  mouth := face(bottom, corner1, right, corner2, upper, corner3, left, corner0)
";
    format!("{}{pocket}{}", &src[..from], &src[to..])
}

#[test]
fn a_slot_rounded_at_its_joints_is_the_slot_written_longhand() {
    let src = std::fs::read_to_string(concat!(env!("CARGO_MANIFEST_DIR"),
                                              "/../examples/rounds/slot.sv")).unwrap();
    let longhand = gestures::build(&src);
    let joints = gestures::build(&slot_by_joints());
    for name in ["pocket", "part"] {
        let (a, b) = (gestures::volume(&longhand, name), gestures::volume(&joints, name));
        assert!((a - b).abs() <= 1e-9 * a, "{name}: {b} against the longhand's {a}");
    }
}

#[test]
fn a_drafted_prism_sweeps_the_rounds_as_cones() {
    let draft = |src: &str| src.replace("pocket := solid(mouth, from: 1mm, to: -6mm)",
                                        "pocket := solid(mouth, from: 1mm, to: -6mm, draft: 4deg)");
    let src = std::fs::read_to_string(concat!(env!("CARGO_MANIFEST_DIR"),
                                              "/../examples/rounds/slot.sv")).unwrap();
    let longhand = gestures::build(&draft(&src));
    let joints = gestures::build(&draft(&slot_by_joints()));
    let (a, b) = (gestures::volume(&longhand, "pocket"), gestures::volume(&joints, "pocket"));
    assert!((a - b).abs() <= 1e-9 * a, "{b} against the longhand's {a}");
}

#[test]
fn a_named_chain_walks_its_rounds() {
    // its rounds unnamed: the chain walks each all the same
    let src = RECT.replace("(bottom := line(a, b))", "outline := (bottom := line(a, b))")
        .replace("(tr := round(r: 10))", "round(r: 10)")
        .replace("(tl := round(r: 10))", "round(r: 10)")
        .replace("outline := face(bottom, br, right, tr, top, tl, left, bl)\n", "");
    let g = gestures::build(&format!("{src}pad := solid(outline, depth: 5)\n"));
    let area = 100.0 * 60.0 - 4.0 * 100.0 + std::f64::consts::PI * 100.0;
    let v = gestures::volume(&g, "pad");
    assert!((v - 5.0 * area).abs() <= 1e-6 * v, "{v} against {}", 5.0 * area);
    // and one turning right at a round walks it from the end it is drawn to
    let src = BRACKET.replace("(a := line(p0, p1))", "outline := (a := line(p0, p1))")
        .replace("outline := face(a, b, c, fillet, d, e, f)\n", "");
    let g = gestures::build(&format!("{src}pad := solid(outline, depth: 2)\n"));
    let area = 60.0 * 10.0 + 10.0 * 40.0 + 25.0 - std::f64::consts::PI * 25.0 / 4.0;
    let v = gestures::volume(&g, "pad");
    assert!((v - 2.0 * area).abs() <= 1e-6 * v, "{v} against {}", 2.0 * area);
}

/// The radiator valve's star handle of #576: four arms, each a tip rounded and a notch
/// filleted, written as its corners — no seed but theirs, no branch chosen by hand.
const STAR: &str = "\
use std

component Star(hub: point, span: Length, waist: Length, tip_r: Length, fillet_r: Length) {
  ring 4 about hub {
    tip := point hint((span / 2, 0))
    notch := point hint((waist / 3, waist / 3))
    hub distance(waist / 2) notch
    hub distance(span / 2) tip
    line(notch, tip) equal line(notch, next.tip)
    (side := line(tip, notch)) -> (fillet := round(r: fillet_r))
      (next_side := line(notch, next.tip)) -> (tip_round := round(r: tip_r))
  }
  hub level(v) tip[0]
  outline := face(side[0], fillet[0], next_side[0], tip_round[0], side[1], fillet[1],
    next_side[1], tip_round[1], side[2], fillet[2], next_side[2], tip_round[2], side[3],
    fillet[3], next_side[3], tip_round[3])
}

in std.front {
  hub := point
  fix((0, 0)) hub
  s := Star(hub, span: 80, waist: 20, tip_r: 4, fillet_r: 3)
}
handle := solid(s.outline, depth: 12, draft: 2deg)
";

#[test]
fn the_star_handle_solves_on_one_root() {
    let e = build(STAR);
    let mut sk = solved(&e);
    assert_eq!(dof(&mut sk), 0);
    for (i, (c, s, t, r)) in arcs(&sk).into_iter().enumerate() {
        // the notches' fillets are concave (drawn from their end), the tips' convex; every arc
        // less than a half turn as drawn, its centre where the corner puts it
        let (want_r, at_hub) = if i % 2 == 0 { (3.0, 10.0 + 3.0 / (57.0f64).to_radians().sin()) }
                               else { (4.0, 0.0) };
        assert!((r - want_r).abs() < 1e-9);
        assert!(sweep(c, s, t) < 180.0, "arc {i}: {}", sweep(c, s, t));
        if i % 2 == 0 {
            assert!(sk.walked_back.contains(&(i as u32)));
            let d = c.0.hypot(c.1);
            assert!(d > 10.0 && (d - at_hub).abs() < 0.5, "{d}");
        }
    }
    let classes = gcs_core::boundary::classify(&e, &sk);
    let star = classes.iter().find(|c| c.name == "Star").unwrap();
    let roots = star.analysis.as_ref().unwrap().roots.unwrap();
    assert_eq!(roots.admitted, 1, "{roots:?}");
    let g = gestures::build(STAR);
    assert!(gestures::volume(&g, "handle") > 0.0);
}

#[test]
fn a_round_too_large_for_its_line_is_said_by_name() {
    // r 12 on a 20 × 16 rectangle: the short sides take 24 of their 16
    let src = "\
use std
in std.front {
a := point
b := point
c := point
d := point
fix((0, 0)) a
fix((20, 0)) b
fix((20, 16)) c
fix((0, 16)) d
(bottom := line(a, b)) -> (br := round(r: 9)) (right := line(b, c)) -> (tr := round(r: 9))
  (top := line(c, d)) -> (left := line(d, a)) -> close
}
";
    let e = build(src);
    let mut sk = e.sketch.clone();
    let _ = gcs_core::solve::solve(&mut sk, Default::default());
    let d = gcs_core::diagnose::diagnose(&mut sk, Default::default());
    assert!(d.warnings.iter().any(|w| w == "`br` (r 9mm) and `tr` (r 9mm) overlap on `right`: \
        they set back 9mm and 9mm from corners 16mm apart"), "{:#?}", d.warnings);
    // one round alone on a line: the largest radius the line takes
    let src = "\
use std
in std.front {
a := point
b := point
c := point
fix((0, 0)) a
fix((40, 0)) b
fix((40, 6)) c
(bottom := line(a, b)) -> (k := round(r: 8)) (up := line(b, c)) -> (back := line(c, a)) -> close
}
";
    let e = build(src);
    let mut sk = e.sketch.clone();
    let _ = gcs_core::solve::solve(&mut sk, Default::default());
    let d = gcs_core::diagnose::diagnose(&mut sk, Default::default());
    assert!(d.warnings.iter().any(|w| w == "`k` (r 8mm) runs past `up`'s far end; it fits up \
        to r 6mm"), "{:#?}", d.warnings);
}

#[test]
fn what_a_round_cannot_round_is_refused() {
    // a corner its lines run straight through
    refused("use std\nin std.front {\na := point\nb := point\nc := point\nfix((0, 0)) a\n\
             fix((10, 0)) b\nfix((20, 0)) c\n(l := line(a, b)) -> (k := round(r: 2)) \
             (m := line(b, c))\n}\n", "E040", "`k` rounds no corner", "(k := round(r: 2))");
}

#[test]
fn a_round_names_its_corner_once() {
    let (_, errs) = crate::common::parse("in std.front {\na := point\nb := point\nc := point\n\
        d := point\n(l := line(a, b)) -> round(r: 2) (m := line(c, d))\n}\n");
    assert!(errs.iter().any(|e| e.message.starts_with("the round's corner is named twice")),
            "{errs:?}");
    let (_, errs) = crate::common::parse("in std.front {\na := point\nb := point\n\
        (l := line(a, b)) -> round(r: 2) equal (m := line(b, a))\n}\n");
    assert!(errs.iter().any(|e| e.message.starts_with("a round is the whole of its joint")),
            "{errs:?}");
    let (_, errs) = crate::common::parse("in std.front {\na := point\nb := point\n\
        (l := line(a, b)) -> round() (m := line(b, a))\n}\n");
    assert!(errs.iter().any(|e| e.message.starts_with("a round says its radius")), "{errs:?}");
    let (_, errs) = crate::common::parse("in std.front {\na := point\nb := point\n\
        (l := line(a, b)) -> round(r: 2) (k := arc(center: a))\n}\n");
    assert!(errs.iter().any(|e| e.message == "a round rounds the corner where two lines meet, \
        and this is an arc"), "{errs:?}");
}

#[test]
fn a_repeat_cannot_round_into_its_next_copy() {
    let src = "use std\nin std.front {\nrepeat 3 {\n  (s := line) -> round(r: 1)\n}\n}\n";
    let (prog, errs) = crate::common::parse(src);
    assert!(errs.is_empty(), "{errs:?}");
    let e = gcs_core::program::elaborate(&prog);
    assert!(e.diags.iter().any(|d| d.message.starts_with("a round at the end of a `repeat`")),
            "{:?}", e.diags.iter().map(|d| &d.message).collect::<Vec<_>>());
}

#[test]
fn a_corner_nobody_wrote_still_rounds() {
    // the lines declared elsewhere, each ending at a point of its own: the round between them
    // touches each there, at the corner where they would cross, which no point keeps
    let src = "\
use std
in std.front {
a := point
b := point hint((20, 0))
c := point hint((30, 10))
d := point
fix((0, 0)) a
fix((30, 40)) d
b level(v) a
c level(u) d
l := line(a, b)
m := line(c, d)
l -> (k := round(r: 5)) m
}
";
    let e = build(src);
    let mut sk = solved(&e);
    assert_eq!(dof(&mut sk), 0);
    assert!(near(sk.point_xy(ent(&e, "b").i()), (25.0, 0.0)));
    assert!(near(sk.point_xy(ent(&e, "c").i()), (30.0, 5.0)));
    assert!(near(sk.point_xy(sk.arcs[ent(&e, "k").i()].center as usize), (25.0, 5.0)));
    assert_eq!(e.sketch.rounds[0].corner, None);
}

#[test]
fn a_rounds_radius_is_edited_where_it_is_written_and_its_rows_go_only_with_it() {
    let e = build(RECT);
    let br = gcs_core::model::EntRef::arc(ent(&e, "br").i());
    let radius = e.sketch.user_constraints().into_iter()
        .find(|c| c.kind == gcs_core::constraints::CKind::Radius && c.args[0].ent() == br)
        .unwrap();
    let edit = gcs_core::edit::set_dimension(&e, &e.program, radius.id, "r", "12");
    assert!(edit.refused.is_none(), "{:?}", edit.refused);
    assert!(edit.text.contains("(br := round(r: 12)) (right"), "{}", edit.text);
    // a tangency the round states is the round's, and not deletable on its own
    let tangency = e.sketch.user_constraints().into_iter()
        .find(|c| c.kind != gcs_core::constraints::CKind::Radius
            && c.args.iter().any(|a| matches!(a, gcs_core::constraints::Arg::Ent(x) if *x == br)))
        .unwrap();
    let out = gcs_core::edit::remove(&e, &e.program, &e.sketch, &[], &[tangency.id]);
    assert!(out.refused.is_some(), "{}", out.text);
}


#[test]
fn the_shipped_handle_is_dimensioned_over_its_rounds() {
    let src = std::fs::read_to_string(concat!(env!("CARGO_MANIFEST_DIR"),
                                              "/../examples/star_handle.sv")).unwrap();
    let e = build(&src);
    let mut sk = solved(&e);
    assert_eq!(dof(&mut sk), 0);
    // across the tips' rounded ends and across the fillets, each at its arc's middle
    for (k, want) in [("tip_end[0]", 74.2 / 2.0), ("waist_end[0]", 25.8 / 2.0)] {
        let (x, y) = sk.point_xy(crate::common::at_path(&e, k).i());
        assert!((x.hypot(y) - want).abs() < 1e-9, "{k}: {}", x.hypot(y));
    }
    let g = gestures::build(&src);
    assert!(gestures::volume(&g, "handle") > 0.0);
}
