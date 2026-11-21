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
use std.solids
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

/// What the diagnosis of `src` warns of, solved as far as it will go.
fn warnings(src: &str) -> Vec<String> {
    let mut sk = build(src).sketch;
    let _ = gcs_core::solve::solve(&mut sk, Default::default());
    gcs_core::diagnose::diagnose(&mut sk, Default::default()).warnings
}

/// Whether `body`, drawn in the front plane, is refused by the parser with a message opening so.
fn refused_where_written(body: &str, opening: &str) {
    let (_, errs) = crate::common::parse(&format!("in std.front {{\n{body}}}\n"));
    assert!(errs.iter().any(|e| e.message.starts_with(opening)), "{opening}: {errs:?}");
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
    let src = format!("{RECT}pad := solid(std.solids.Prism(outline, from: -5, to: 0))\n");
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
use std.solids
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
    assert!(sk.arc_reversed(k));
    // and the names are as the walk wrote them: its start on `c`, where the walk enters it
    let src = format!("{BRACKET}q := point\nq coincident fillet.start\n");
    let g = build(&src);
    let sk = solved(&g);
    // and its turn runs as walked: `t == 0` where the walk enters it, on `c`
    let at_c = build(&BRACKET.replace("outline :=", "w := point\nw coincident(t == 0.25) fillet\n\
        outline :="));
    let skw = solved(&at_c);
    let w = skw.point_xy(ent(&at_c, "w").i());
    let quarter = std::f64::consts::FRAC_PI_8;
    assert!(near(w, (15.0 - 5.0 * quarter.sin(), 15.0 - 5.0 * quarter.cos())), "{w:?}");
    // `q` stands in space, drawn where the arc's start is in the front plane
    let q = sk.world_point(ent(&g, "q").i());
    assert!(near((q[0], q[2]), (15.0, 10.0)), "{q:?}");
    // the bracket's area: the L, and the fillet's web between the corner's square and its disc
    let area = 60.0 * 10.0 + 10.0 * 40.0 + 25.0 - std::f64::consts::PI * 25.0 / 4.0;
    let g = gestures::build(&format!("{BRACKET}pad := solid(std.solids.Prism(outline, from: -2, to: 0))\n"));
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
    let draft = |src: &str| src.replace("pocket := solid(std.solids.Prism(mouth, from: -6mm, to: 1mm))",
                                        "pocket := solid(std.solids.Drafted(mouth, from: -6mm, to: 1mm, draft: 4deg))");
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
    let g = gestures::build(&format!("{src}pad := solid(std.solids.Prism(outline, from: -5, to: 0))\n"));
    let area = 100.0 * 60.0 - 4.0 * 100.0 + std::f64::consts::PI * 100.0;
    let v = gestures::volume(&g, "pad");
    assert!((v - 5.0 * area).abs() <= 1e-6 * v, "{v} against {}", 5.0 * area);
    // and one turning right at a round walks it from the end it is drawn to
    let src = BRACKET.replace("(a := line(p0, p1))", "outline := (a := line(p0, p1))")
        .replace("outline := face(a, b, c, fillet, d, e, f)\n", "");
    let g = gestures::build(&format!("{src}pad := solid(std.solids.Prism(outline, from: -2, to: 0))\n"));
    let area = 60.0 * 10.0 + 10.0 * 40.0 + 25.0 - std::f64::consts::PI * 25.0 / 4.0;
    let v = gestures::volume(&g, "pad");
    assert!((v - 2.0 * area).abs() <= 1e-6 * v, "{v} against {}", 2.0 * area);
}

/// The radiator valve's star handle of #576: four arms, each a tip rounded and a notch
/// filleted, written as its corners — no seed but theirs, no branch chosen by hand.
const STAR: &str = "\
use std
use std.solids

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
handle := solid(std.solids.Drafted(s.outline, from: -12, to: 0, draft: 2deg))
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
            assert!(sk.arc_reversed(i));
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
    // r 9 at two corners of a 20 × 16 rectangle: the short side between them takes 18 of its 16
    let src = "\
use std
use std.solids
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
    let said = warnings(src);
    assert!(said.iter().any(|w| w == "`br` (r 9mm) and `tr` (r 9mm) overlap on `right`: \
        they set back 9mm and 9mm from corners 16mm apart"), "{said:#?}");
    // one round alone on a line: the largest radius the line takes
    let src = "\
use std
use std.solids
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
    let said = warnings(src);
    assert!(said.iter().any(|w| w == "`k` (r 8mm) runs past `up`'s far end; it fits up \
        to r 6mm"), "{said:#?}");
}

#[test]
fn what_a_round_cannot_round_is_refused() {
    // a corner its lines run straight through
    refused("use std
use std.solids\nin std.front {\na := point\nb := point\nc := point\nfix((0, 0)) a\n\
             fix((10, 0)) b\nfix((20, 0)) c\n(l := line(a, b)) -> (k := round(r: 2)) \
             (m := line(b, c))\n}\n", "E040", "`k` rounds no corner", "(k := round(r: 2))");
}

#[test]
fn a_round_is_refused_where_it_is_written() {
    let points = "a := point\nb := point\nc := point\nd := point\n";
    refused_where_written(&format!("{points}(l := line(a, b)) -> round(r: 2) (m := line(c, d))\n"),
                          "the round's corner is named twice");
    refused_where_written(&format!("{points}(l := line(a, b)) -> round(r: 2) equal \
                                    (m := line(b, a))\n"), "a round is the whole of its joint");
    refused_where_written(&format!("{points}(l := line(a, b)) -> round() (m := line(b, a))\n"),
                          "a round says its radius");
    refused_where_written(&format!("{points}(l := line(a, b)) -> round(r: 2) \
                                    (k := circle(center: a))\n"),
                          "a corner joins lines and arcs; a circle has no ends to thread");
    // an arc beside a round is walked from its start, as written: one written to be walked from
    // its end names the corner at its other end
    refused_where_written(&format!("{points}(l := line(a, b)) -> round(r: 2) \
                                    (k := arc(center: c, start: d, end: b))\n"),
                          "the round's corner is named twice: `l` leaves at `b` and `k` arrives \
                           at `d`: a round walks an arc beside it from its start, and `k` is \
                           written to be walked from its end");
}

#[test]
fn a_repeat_cannot_round_into_its_next_copy() {
    let src = "use std\nuse std.solids\nin std.front {\nrepeat 3 {\n  (s := line) -> round(r: 1)\n}\n}\n";
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
use std.solids
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

/// **A round beside an arc declared elsewhere** (#948): `re -> round(r: 18mm) -> tl`, `re` an
/// arc declared before it.  The parser cannot know `re`'s kind, so it writes the round's arc from
/// `re.p2`, which an arc has none of: said at line 1, column 1, then each row of the round over
/// an arc never built, by its key.  Now said once, at the round, by the name written there, and
/// nothing of the round stated — an arc declared in the chain is rounded (#962).
#[test]
fn a_round_from_an_arc_declared_elsewhere_is_said_once_at_the_round() {
    let src = "unit mm\nuse std\nin std.top {\n  o := point\n  fix((0, 0)) o\n\
        p1 := point hint((80, -60))\n  p2 := point hint((80, 60))\n  fix(y == -60mm) p1\n\
        fix(y == 60mm) p2\n  p3 := point\n  fix((-80, 60)) p3\n\
        re := arc(center: o, start: p1, end: p2) hint(r: 100)\n  radius(100mm) re\n\
        tl := line(p2, p3)\n  re -> round(r: 18mm) -> tl\n}\n";
    let (prog, errs) = crate::common::parse(src);
    assert!(errs.is_empty(), "{errs:?}");
    let e = gcs_core::program::elaborate(&prog);
    let said: Vec<_> = e.errors().map(|d| (d.at(e.text()).0, d.message.as_str())).collect();
    assert_eq!(said, [(15, "a round names the ends of a side it is not declared with as a \
        line's, and `re` is an arc declared apart from the chain: declare it in the chain, `(…) \
        -> round(…) (re := arc(…))`, where the round reads its ends")]);
    // and a circle, which has no ends to round between
    let src = src.replace("re := arc(center: o, start: p1, end: p2) hint(r: 100)",
                          "re := circle(center: o) hint(r: 100)");
    let (prog, errs) = crate::common::parse(&src);
    assert!(errs.is_empty(), "{errs:?}");
    let e = gcs_core::program::elaborate(&prog);
    assert!(e.errors().any(|d| d.message == "a round rounds the corner where two lines or arcs \
        meet, and `re` is a circle: round a curve's edge by a round of the solid it bounds"),
        "{:?}", e.errors().map(|d| &d.message).collect::<Vec<_>>());
}

// ---- a round beside an arc (#962) ---------------------------------------------------------------

/// One edge of a loop, for the area it bounds: a segment, or an arc about a centre from one point
/// to another, counter-clockwise or not.
enum Edge {
    Seg((f64, f64), (f64, f64)),
    Arc((f64, f64), (f64, f64), (f64, f64), bool),
}

/// The area a loop of edges bounds, `∮ (x dy − y dx) / 2`: a segment's `½ A × B`, an arc's
/// `½ (C × (B − A) + r² φ)`, `φ` its signed turn — worked out here, never asked of the core.
fn loop_area(edges: &[Edge]) -> f64 {
    let cross = |a: (f64, f64), b: (f64, f64)| a.0 * b.1 - a.1 * b.0;
    edges.iter().map(|e| match *e {
        Edge::Seg(a, b) => cross(a, b) / 2.0,
        Edge::Arc(c, a, b, ccw) => {
            let (u, v) = ((a.0 - c.0, a.1 - c.1), (b.0 - c.0, b.1 - c.1));
            let turn = (v.1.atan2(v.0) - u.1.atan2(u.0)).rem_euclid(std::f64::consts::TAU);
            let phi = if ccw { turn } else { turn - std::f64::consts::TAU };
            let r2 = u.0 * u.0 + u.1 * u.1;
            (cross(c, (b.0 - a.0, b.1 - a.1)) + r2 * phi) / 2.0
        }
    }).sum()
}

fn close9(a: (f64, f64), b: (f64, f64)) -> bool {
    (a.0 - b.0).abs() < 1e-9 && (a.1 - b.1).abs() < 1e-9
}

fn turned(p: (f64, f64), by: f64) -> (f64, f64) {
    let (s, c) = by.sin_cos();
    (c * p.0 - s * p.1, s * p.0 + c * p.1)
}

/// A window 80 wide on a sill, its jambs square to the sill and an arch of R50 springing from
/// them, its centre 20 above the sill on the centre line (`sag`: an arch of R50 sagging, its
/// centre 80 above, said to turn clockwise), each corner where a jamb meets the arch rounded at
/// `r` — the whole turned by `by` about the sill's middle.  Walked counter-clockwise: a left turn
/// at each corner, the round inside the arch's circle where it bulges and outside where it sags.
fn window(r: f64, sag: bool, by: f64) -> String {
    let at = |p: (f64, f64)| {
        let q = turned(p, by);
        format!("({}, {})", q.0, q.1)
    };
    let (o, sense) = if sag { ((0.0, 80.0), ", sense: cw") } else { ((0.0, 20.0), "") };
    format!("\
use std
use std.solids
in std.front {{
a := point
b := point
o := point
fix({}) a
fix({}) b
fix({}) o
c := point hint({})
d := point hint({})
(bottom := line(a, b)) -> (right := line(b, c)) -> (k1 := round(r: {r}))
  (top := arc(center: o, start: c, end: d{sense})) -> (k2 := round(r: {r})) (left := line(d, a)) ->
  close
right perpendicular bottom
left perpendicular bottom
radius(50) top
outline := face(bottom, right, k1, top, k2, left)
}}
pad := solid(std.solids.Prism(outline, from: -5, to: 0))
", at((-40.0, 0.0)), at((40.0, 0.0)), at(o), at((40.0, 50.0)), at((-40.0, 50.0)))
}

/// The round of radius `r` at the window's right corner, worked out here: its centre on the
/// jamb's offset `x = 40 − r` and on the arch's circle shrunk by `r` (grown, where it sags), the
/// root by the corner — `(centre, its point on the jamb, its point on the arch)`.
fn window_round(r: f64, sag: bool) -> ((f64, f64), (f64, f64), (f64, f64)) {
    let kx = 40.0 - r;
    let (oy, g) = if sag { (80.0, 50.0 + r) } else { (20.0, 50.0 - r) };
    let ky = if sag { oy - (g * g - kx * kx).sqrt() } else { oy + (g * g - kx * kx).sqrt() };
    let s = 50.0 / g;
    ((kx, ky), (40.0, ky), (kx * s, oy + (ky - oy) * s))
}

/// The window's area with both its corners rounded at `r` (`window_round`, and its mirror).
fn window_area(r: f64, sag: bool) -> f64 {
    let (k, s, e) = window_round(r, sag);
    let m = |p: (f64, f64)| (-p.0, p.1);
    let o = if sag { (0.0, 80.0) } else { (0.0, 20.0) };
    loop_area(&[Edge::Seg((-40.0, 0.0), (40.0, 0.0)), Edge::Seg((40.0, 0.0), s),
                Edge::Arc(k, s, e, true), Edge::Arc(o, e, m(e), !sag),
                Edge::Arc(m(k), m(e), m(s), true), Edge::Seg(m(s), (-40.0, 0.0))])
}

/// Each round of a solved window to its closed form, turned by `by`.
fn window_held(e: &gcs_core::program::Elaborated, sk: &Sketch, r: f64, sag: bool, by: f64) {
    let (k, s, t) = window_round(r, sag);
    let m = |p: (f64, f64)| (-p.0, p.1);
    for (name, (k, on_jamb, on_arch), jamb) in [("k1", (k, s, t), "right"),
                                                 ("k2", (m(k), m(s), m(t)), "left")] {
        let arc = &sk.arcs[ent(e, name).i()];
        let c = at(sk, arc.center);
        assert!(close9(c, turned(k, by)), "{name} centre {c:?} against {:?}", turned(k, by));
        assert!((sk.params[arc.radius as usize].value - r).abs() < 1e-9);
        // its ends, as written: on the side arrived along, then the one left along
        let (from, to) = if name == "k1" { (on_jamb, on_arch) } else { (on_arch, on_jamb) };
        let (s, t) = match sk.arc_reversed(ent(e, name).i()) {
            false => (arc.start, arc.end),
            true => (arc.end, arc.start),
        };
        assert!(close9(at(sk, s), turned(from, by)) && close9(at(sk, t), turned(to, by)),
                "{name}: {:?} {:?}", at(sk, s), at(sk, t));
        // and the jamb ends at it
        let l = &sk.lines[ent(e, jamb).i()];
        let p = if name == "k1" { l.p2 } else { l.p1 };
        assert!(close9(at(sk, p), turned(on_jamb, by)));
    }
    // the arch runs between the two rounds
    let top = &sk.arcs[ent(e, "top").i()];
    let ends = [at(sk, top.start), at(sk, top.end)];
    for want in [t, m(t)].map(|p| turned(p, by)) {
        assert!(ends.iter().any(|&q| close9(q, want)), "{ends:?} against {want:?}");
    }
}

#[test]
fn a_round_between_a_line_and_an_arc_touches_both_inside_the_corner() {
    for (r, sag) in [(5.0, false), (12.5, false), (5.0, true), (12.5, true)] {
        let src = window(r, sag, 0.0);
        let e = build(&src);
        let mut sk = solved(&e);
        assert_eq!(dof(&mut sk), 0, "r {r}, sag {sag}");
        window_held(&e, &sk, r, sag, 0.0);
        // a left turn at each corner, so each round drawn as written
        assert!(!sk.arc_reversed(ent(&e, "k1").i()) && !sk.arc_reversed(ent(&e, "k2").i()));
        // a tangency to the arch is the one row at the round's end: the two centres and it on
        // one line
        let said = crate::common::said(&e);
        assert!(said.iter().any(|s| s == "k1 tangent(at: top.start) top"), "{said:#?}");
        assert!(said.iter().any(|s| s == "c coincident top"), "{said:#?}");
        // the window less its corners' webs, extruded 5
        let g = gestures::build(&src);
        let (v, want) = (gestures::volume(&g, "pad"), 5.0 * window_area(r, sag));
        assert!((v - want).abs() <= 1e-9 * want, "r {r}, sag {sag}: {v} against {want}");
    }
}

#[test]
fn a_round_beside_an_arc_turned_or_walked_the_other_way_is_the_same_round() {
    // the whole window turned a third of a right angle
    let by = std::f64::consts::FRAC_PI_6;
    for sag in [false, true] {
        let e = build(&window(5.0, sag, by));
        let mut sk = solved(&e);
        assert_eq!(dof(&mut sk), 0);
        window_held(&e, &sk, 5.0, sag, by);
    }
    // the window walked clockwise — its mirror, as drawn: a right turn at each corner, so each
    // round drawn from its end, and the arch said to turn clockwise from the left jamb's head
    let src = "\
use std
use std.solids
in std.front {
a := point
b := point
o := point
fix((-40, 0)) a
fix((40, 0)) b
fix((0, 20)) o
c := point hint((40, 50))
d := point hint((-40, 50))
(bottom := line(b, a)) -> (left := line(a, d)) -> (k2 := round(r: 5))
  (top := arc(center: o, start: d, end: c, sense: cw)) -> (k1 := round(r: 5))
  (right := line(c, b)) -> close
right perpendicular bottom
left perpendicular bottom
radius(50) top
outline := face(bottom, left, k2, top, k1, right)
}
pad := solid(std.solids.Prism(outline, from: -5, to: 0))
";
    let e = build(src);
    let mut sk = solved(&e);
    assert_eq!(dof(&mut sk), 0);
    assert!(sk.arc_reversed(ent(&e, "k1").i()) && sk.arc_reversed(ent(&e, "k2").i()));
    let (k, s, t) = window_round(5.0, false);
    for (name, c, from, to) in [("k1", k, t, s), ("k2", (-k.0, k.1), (-s.0, s.1), (-t.0, t.1))] {
        let arc = &sk.arcs[ent(&e, name).i()];
        assert!(close9(at(&sk, arc.center), c), "{name}");
        // drawn from its end: its model start is the end as written
        assert!(close9(at(&sk, arc.end), from) && close9(at(&sk, arc.start), to), "{name}");
        assert!(sweep(c, at(&sk, arc.start), at(&sk, arc.end)) < 180.0);
    }
    let g = gestures::build(src);
    let (v, want) = (gestures::volume(&g, "pad"), 5.0 * window_area(5.0, false));
    assert!((v - want).abs() <= 1e-9 * want, "{v} against {want}");
}

#[test]
fn a_round_closes_a_loop_onto_a_line_after_an_arc_and_a_named_chain_walks_it() {
    // the arch the last link, its round the closing corner's; the chain named and the rounds
    // unnamed, and walked all the same
    let src = "\
use std
use std.solids
in std.front {
a := point
b := point
o := point
fix((-40, 0)) a
fix((40, 0)) b
fix((0, 20)) o
c := point hint((40, 50))
d := point hint((-40, 50))
outline := (left := line(d, a)) -> (bottom := line(a, b)) -> (right := line(b, c)) ->
  round(r: 5) (top := arc(center: o, start: c, end: d)) -> round(r: 5) close
right perpendicular bottom
left perpendicular bottom
radius(50) top
}
pad := solid(std.solids.Prism(outline, from: -5, to: 0))
";
    let e = build(src);
    let mut sk = solved(&e);
    assert_eq!(dof(&mut sk), 0);
    let (_, s, t) = window_round(5.0, false);
    let top = &sk.arcs[ent(&e, "top").i()];
    assert!(close9(at(&sk, top.start), t) && close9(at(&sk, top.end), (-t.0, t.1)));
    let left = &sk.lines[ent(&e, "left").i()];
    assert!(close9(at(&sk, left.p1), (-s.0, s.1)));
    let g = gestures::build(src);
    let (v, want) = (gestures::volume(&g, "pad"), 5.0 * window_area(5.0, false));
    assert!((v - want).abs() <= 1e-9 * want, "{v} against {want}");
}

/// Four arcs of R40 bulging between four corners 30 from a hub, each corner where one arc runs
/// into the next rounded at `r` — a round between two arcs, the ring's last joint rounding into
/// the next copy.
fn cushion(r: f64) -> String {
    format!("\
use std
use std.solids
in std.front {{
hub := point
fix((0, 0)) hub
ring 4 about hub {{
  p := point hint((30, 0))
  c := point hint((-9, -9))
  hub distance(30) p
  radius(40) edge
  (edge := arc(center: c, start: p, end: next.p)) -> (k := round(r: {r}))
}}
hub level(v) p[0]
outline := face(edge[0], k[0], edge[1], k[1], edge[2], k[2], edge[3], k[3])
}}
pad := solid(std.solids.Prism(outline, from: -2, to: 0))
")
}

#[test]
fn a_round_between_two_arcs_rounds_a_ring_into_its_next_copy() {
    let r = 3.0;
    let src = cushion(r);
    let e = build(&src);
    let mut sk = solved(&e);
    assert_eq!(dof(&mut sk), 0);
    // the first edge's centre on the hub's side of its chord from (30, 0) to (0, 30), 40 from
    // each; the round at (0, 30) on the line of symmetry, 40 − r from that centre
    let c0 = 15.0 - 575f64.sqrt();
    let k = (0.0, c0 + ((40.0 - r).powi(2) - c0 * c0).sqrt());
    let on = |c: (f64, f64)| {
        let s = 40.0 / (40.0 - r);
        (c.0 + (k.0 - c.0) * s, c.1 + (k.1 - c.1) * s)
    };
    let (c1, c0p) = ((-c0, c0), (c0, c0));
    let (arrive, leave) = (on(c0p), on(c1));
    let half = std::f64::consts::FRAC_PI_2;
    for i in 0..4 {
        let by = half * i as f64;
        let arc = &sk.arcs[crate::common::at_path(&e, &format!("k[{i}]")).i()];
        assert!(close9(at(&sk, arc.center), turned(k, by)), "k[{i}] {:?}", at(&sk, arc.center));
        assert!(close9(at(&sk, arc.start), turned(arrive, by))
                && close9(at(&sk, arc.end), turned(leave, by)), "k[{i}]");
        assert!(!sk.arc_reversed(crate::common::at_path(&e, &format!("k[{i}]")).i()));
    }
    let mut edges = Vec::new();
    for i in 0..4 {
        let by = half * i as f64;
        let start = turned(leave, -half);
        edges.push(Edge::Arc(turned(c0p, by), turned(start, by), turned(arrive, by), true));
        edges.push(Edge::Arc(turned(k, by), turned(arrive, by), turned(leave, by), true));
    }
    let g = gestures::build(&src);
    let (v, want) = (gestures::volume(&g, "pad"), 2.0 * loop_area(&edges));
    assert!((v - want).abs() <= 1e-9 * want, "{v} against {want}");
}

#[test]
fn a_round_between_an_arc_and_one_turning_the_other_way_is_outside_one_and_inside_the_other() {
    // R30 about the origin walked counter-clockwise into R40 about (60, 0) walked clockwise,
    // meeting above the axis: a right turn, the round outside the first circle and inside the
    // second — its centre where the first grown by r and the second shrunk by r cross, above
    let src = "\
use std
in std.front {
o1 := point
o2 := point
s := point
t := point
fix((0, 0)) o1
fix((60, 0)) o2
fix((30, 0)) s
fix((100, 0)) t
p := point hint((24, 18))
(lobe := arc(center: o1, start: s, end: p)) -> (k := round(r: 5))
  (waist := arc(center: o2, start: p, end: t, sense: cw))
}
";
    let e = build(src);
    let mut sk = solved(&e);
    assert_eq!(dof(&mut sk), 0);
    let kc = (30.0, 325f64.sqrt());
    let on_lobe = (kc.0 * 30.0 / 35.0, kc.1 * 30.0 / 35.0);
    let on_waist = (60.0 + (kc.0 - 60.0) * 40.0 / 35.0, kc.1 * 40.0 / 35.0);
    let k = ent(&e, "k").i();
    let arc = &sk.arcs[k];
    assert!(sk.arc_reversed(k));
    assert!(close9(at(&sk, arc.center), kc), "{:?}", at(&sk, arc.center));
    // drawn from its end: its model start the end as written, on the waist
    assert!(close9(at(&sk, arc.end), on_lobe) && close9(at(&sk, arc.start), on_waist));
    // the corner stays on both circles
    let p = at(&sk, ent(&e, "p").i() as u32);
    let x = (900.0 - 1600.0 + 3600.0) / 120.0;
    assert!(close9(p, (x, (900.0 - x * x).sqrt())), "{p:?}");
}

#[test]
fn a_corner_an_arc_turns_toward_holds_a_round_only_so_large() {
    use gcs_core::model::{rounded, RoundSide};
    // a line arriving at (16, −12) on a circle of R20 about the origin, turning onto it
    // counter-clockwise: the round inside the circle, which the corner holds up to (ρ + h)/2 =
    // (20 + 12)/2 = 16, h the centre's height above the line
    let (p, a) = ((16.0, -12.0), RoundSide::Line((-16.0, -12.0)));
    let b = RoundSide::Arc { center: (0.0, 0.0), radius: 20.0, ccw: true };
    let under = rounded(p, a, b, 15.9).unwrap();
    let kx = (4.1f64.powi(2) - 3.9f64.powi(2)).sqrt();
    assert!(close9(under.center, (kx, 3.9)), "{:?}", under.center);
    assert!(close9(under.start, (kx, -12.0)));
    assert!(close9(under.end, (kx * 20.0 / 4.1, 3.9 * 20.0 / 4.1)));
    assert_eq!(under.most, Some(16.0));
    assert_eq!(under.inside, [false, true]);
    // at it, the round touches the line square below the centre and the circle at its top
    let at_most = rounded(p, a, b, 16.0).unwrap();
    assert!(close9(at_most.center, (0.0, 4.0)) && close9(at_most.end, (0.0, 20.0)));
    // over it, placed where the corner holds it, and said so
    assert_eq!(rounded(p, a, b, 17.0).unwrap().center, at_most.center);
    // two arcs of R60 about (∓20, 0) meeting at the top, both turning toward the round: it holds
    // up to (ρ₁ + ρ₂ − D)/2 = 40, a round about the origin
    let apex = (0.0, 3200f64.sqrt());
    let (l, r) = (RoundSide::Arc { center: (-20.0, 0.0), radius: 60.0, ccw: true },
                  RoundSide::Arc { center: (20.0, 0.0), radius: 60.0, ccw: true });
    let five = rounded(apex, l, r, 5.0).unwrap();
    assert!(close9(five.center, (0.0, 2625f64.sqrt())));
    let s = 60.0 / 55.0;
    assert!(close9(five.start, (-20.0 + 20.0 * s, 2625f64.sqrt() * s)));
    assert_eq!(five.most, Some(40.0));
    assert!(close9(rounded(apex, l, r, 40.0).unwrap().center, (0.0, 0.0)));
    // the same corner walked the other way: the right turn, the same round
    let (l2, r2) = (RoundSide::Arc { center: (20.0, 0.0), radius: 60.0, ccw: false },
                    RoundSide::Arc { center: (-20.0, 0.0), radius: 60.0, ccw: false });
    let back = rounded(apex, l2, r2, 5.0).unwrap();
    assert_eq!(back.turn, -1);
    assert!(close9(back.center, five.center) && close9(back.end, five.start));
}

#[test]
fn a_round_too_large_beside_an_arc_is_said_by_name() {
    // r 42 at both of the window's corners: their stretches of the arch overlap
    let said = warnings(&window(42.0, false, 0.0));
    assert!(said.iter().any(|w| w.starts_with("`k1` (r 42mm) and `k2` (r 42mm) overlap on \
        `top`")), "{said:#?}");
    // one round alone, past the arch's far end: the largest radius that touches it there, the
    // round tangent to the jamb and to the arch at `d`, 80 / 1.8
    let one = window(44.9, false, 0.0).replace("(top := arc(center: o, start: c, end: d)) -> \
        (k2 := round(r: 44.9)) (left := line(d, a))", "(top := arc(center: o, start: c, end: d)) \
        -> (left := line(d, a))").replace(", k2, left)", ", left)");
    let said = warnings(&one);
    assert!(said.iter().any(|w| w == "`k1` (r 44.9mm) runs past `top`'s far end; it fits up to \
        r 44.44mm"), "{said:#?}");
    // a bowl the line turns into, more than its corner holds; and just under, nothing said
    let bowl = |r: f64| format!("\
use std
use std.solids
in std.front {{
a := point
b := point
o := point hint((0, 1))
fix((-16, -12)) a
fix((16, -12)) b
(flat := line(a, b)) -> (k := round(r: {r})) (bowl := arc(center: o, start: b, end: a)) -> close
radius(20) bowl
}}
");
    // — the radius as written, and what the corner holds read off the drawing the solve that
    // could not hold it left, near the 16 it holds as drawn (`rounded` holds that bound exactly)
    let said = warnings(&bowl(17.0));
    let opening = "`k` (r 17mm) is more than the corner of `flat` and `bowl` holds: `bowl` turns \
        toward it there; it fits up to r ";
    let fits = said.iter().find_map(|w| w.strip_prefix(opening)?.strip_suffix("mm")?
        .parse::<f64>().ok());
    assert!(fits.is_some_and(|f| (f - 16.0).abs() < 1.0), "{said:#?}");
    let e = build(&bowl(15.9));
    let mut sk = solved(&e);
    assert_eq!(dof(&mut sk), 0);
    let kx = (4.1f64.powi(2) - 3.9f64.powi(2)).sqrt();
    assert!(close9(at(&sk, sk.arcs[ent(&e, "k").i()].center), (kx, 3.9)));
    assert!(warnings(&bowl(15.9)).iter().all(|w| !w.contains("`k`")));
}

#[test]
fn the_shipped_arched_window_rounds_its_jambs_into_its_arch() {
    let src = std::fs::read_to_string(concat!(env!("CARGO_MANIFEST_DIR"),
                                              "/../examples/arched_window.sv")).unwrap();
    let e = build(&src);
    let mut sk = solved(&e);
    assert_eq!(dof(&mut sk), 0);
    // the arch's centre on the centre line below the springing, the round 12 in from the jamb
    // and 97.5 − 12 from that centre
    let h0 = 120.0 - (97.5f64.powi(2) - 75.0f64.powi(2)).sqrt();
    let k = (63.0, h0 + (85.5f64.powi(2) - 63.0f64.powi(2)).sqrt());
    let s = (75.0, k.1);
    let t = (k.0 * 97.5 / 85.5, h0 + (k.1 - h0) * 97.5 / 85.5);
    let round = &sk.arcs[ent(&e, "round_r").i()];
    assert!(close9(at(&sk, round.center), k), "{:?}", at(&sk, round.center));
    assert!(close9(at(&sk, round.start), s) && close9(at(&sk, round.end), t));
    let m = |p: (f64, f64)| (-p.0, p.1);
    let area = loop_area(&[Edge::Seg((-75.0, 0.0), (75.0, 0.0)), Edge::Seg((75.0, 0.0), s),
                           Edge::Arc(k, s, t, true), Edge::Arc((0.0, h0), t, m(t), true),
                           Edge::Arc(m(k), m(t), m(s), true), Edge::Seg(m(s), (-75.0, 0.0))]);
    let g = gestures::build(&src);
    let (v, want) = (gestures::volume(&g, "panel"), 20.0 * (300.0 * 300.0 - area));
    assert!((v - want).abs() <= 1e-9 * want, "{v} against {want}");
}
