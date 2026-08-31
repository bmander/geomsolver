//! A plane is a frame with an attitude in space, a point may be `in` one, and `project` says
//! two such points are images of one point — descriptive geometry on one sheet (§6.7).
use gcs_core::constraints::{CKind, Constraint};
use gcs_core::model::{pick, EntRef, Sketch};
use gcs_core::plane::{fold_line, Basis};
use gcs_core::solve::{solve, SolveOpts};
use gcs_core::{diagnose, io};
use std::f64::consts::FRAC_PI_2;

fn near(a: f64, b: f64) -> bool {
    (a - b).abs() < 1e-8
}

/// The frame's fixture with an attitude: chord (4, 3), so the rotor is the 3-4-5 triangle's.
fn with_plane() -> (Sketch, usize) {
    let mut sk = Sketch::new();
    let o = sk.point(10.0, 5.0, false, "o");
    let t = sk.point(14.0, 8.0, false, "t");
    let p = sk.plane(o, t, Basis::page(), "front");
    (sk, p)
}

#[test]
fn the_rotor_and_intrinsics_mirror_a_frame() {
    let (mut sk, p) = with_plane();
    let f = &sk.planes[p].frame;
    assert!((sk.params[f.c as usize].value - 0.8).abs() < 1e-12);
    assert!((sk.params[f.s as usize].value - 0.6).abs() < 1e-12);
    assert!((sk.params[f.c as usize].scale - 5.0).abs() < 1e-12);
    assert_eq!(sk.constraints.len(), 2);
    assert!(sk.constraints.iter().all(|c| c.intrinsic));
    assert_eq!(sk.constraints[0].kind, CKind::FrameUnit);
    assert_eq!(sk.constraints[1].kind, CKind::FrameAlign);
    assert!(sk.user_constraints().is_empty());
    let rp = sk.constraints[1].args[1].param() as usize;
    assert!((sk.params[rp].value - 5.0).abs() < 1e-12);
    assert_eq!(sk.planes[p].basis, Basis::page());
    let d = diagnose::diagnose(&mut sk, Default::default());
    assert_eq!(d.dof, 4, "two free points, and the rotor slaved to them");
}

#[test]
fn the_fold_convention_and_the_fold_line() {
    let near = |a: [f64; 3], b: [f64; 3]| (0..3).all(|i| (a[i] - b[i]).abs() < 1e-12);
    let top = Basis::page().fold(0.0);
    assert!(near(top.u, [1.0, 0.0, 0.0]) && near(top.v, [0.0, 1.0, 0.0]));
    assert!(near(top.normal(), [0.0, 0.0, 1.0]), "the top view looks down from +z");
    let right = Basis::page().fold(-FRAC_PI_2);
    assert!(near(right.u, [0.0, 0.0, -1.0]) && near(right.v, [0.0, 1.0, 0.0]));
    assert!(near(right.normal(), [1.0, 0.0, 0.0]), "and the right view from +x");
    // an explicit basis is orthonormalised, and a pair spanning no plane is refused
    let b = Basis::explicit([2.0, 0.0, 0.0], [1.0, 0.0, 3.0]).unwrap();
    assert!(near(b.u, [1.0, 0.0, 0.0]) && near(b.v, [0.0, 0.0, 1.0]));
    assert!(Basis::explicit([1.0, 0.0, 0.0], [-2.0, 0.0, 0.0]).is_none());
    assert!(Basis::explicit([0.0, 0.0, 0.0], [0.0, 0.0, 1.0]).is_none(), "and a zero vector");
    // the front and the top share the page's x-axis, which is `u` in both
    let (da, db) = fold_line(&Basis::page(), &top).unwrap();
    assert!((da[0].abs() - 1.0).abs() < 1e-12 && da[1].abs() < 1e-12);
    assert!((da[0] - db[0]).abs() < 1e-12 && db[1].abs() < 1e-12);
    assert!(fold_line(&Basis::page(), &Basis::page()).is_none());
}

/// Front, top and right views in the standard third-angle layout — top above the front, right
/// beside it with its frame turned so z is up — and one point seen in all three.  Width agrees
/// front↔top, height front↔right, depth top↔right: the projector rule, three times.
fn three_views() -> (Sketch, [usize; 3], [usize; 3]) {
    let mut sk = Sketch::new();
    let datum = |sk: &mut Sketch, o: (f64, f64), t: (f64, f64), b: Basis, n: &str| {
        let oi = sk.point(o.0, o.1, true, &format!("{n}.o"));
        let ti = sk.point(t.0, t.1, true, &format!("{n}.t"));
        sk.plane(oi, ti, b, n)
    };
    let front = datum(&mut sk, (0.0, 0.0), (1.0, 0.0), Basis::page(), "front");
    let top = datum(&mut sk, (0.0, 100.0), (1.0, 100.0), Basis::page().fold(0.0), "top");
    let right =
        datum(&mut sk, (150.0, 0.0), (150.0, -1.0), Basis::page().fold(-FRAC_PI_2), "right");
    // the images of X = (30, 20, 40): the front's is stated, the other two are unknowns
    let pf = sk.point(30.0, 40.0, true, "pf");
    let pt = sk.point(20.0, 110.0, false, "pt");
    let pr = sk.point(160.0, 30.0, false, "pr");
    sk.set_plane(pf, Some(front));
    sk.set_plane(pt, Some(top));
    sk.set_plane(pr, Some(right));
    (sk, [front, top, right], [pf, pt, pr])
}

#[test]
fn three_views_agree_on_a_point() {
    let (mut sk, _, [pf, pt, pr]) = three_views();
    let (pfe, pte, pre) = (EntRef::point(pf), EntRef::point(pt), EntRef::point(pr));
    for (a, b) in [(pfe, pte), (pfe, pre), (pte, pre)] {
        let c = Constraint::project(&sk, a, b).unwrap();
        assert_eq!(c.entities().len(), 4, "the planes ride in the constraint");
        sk.add(c);
    }
    let d = diagnose::diagnose(&mut sk, Default::default());
    assert_eq!(d.dof, 1, "three rows over four unknowns: the depth is free");
    // say the depth in the top view and everything else follows
    let py = sk.point_params(pt)[1] as usize;
    sk.params[py].value = 120.0;
    sk.params[py].fixed = true;
    let r = solve(&mut sk, SolveOpts::default());
    assert!(r.success, "{}", r.message);
    let (tx, _) = sk.point_xy(pt);
    let (rx, ry) = sk.point_xy(pr);
    assert!(near(tx, 30.0), "width agrees front↔top: {tx}");
    assert!(near(ry, 40.0), "height agrees front↔right: {ry}");
    assert!(near(rx, 170.0), "depth agrees top↔right: {rx}");
}

/// An auxiliary view folded from the top at the bearing of a slanted edge sees the edge at its
/// true length — the reason a draughtsman draws one.  The auxiliary images are placed by
/// projection alone, four rows over four unknowns.
#[test]
fn an_auxiliary_view_shows_true_length() {
    let (c30, s30) = (30f64.to_radians().cos(), 30f64.to_radians().sin());
    // A = (0, 0, 0), B = (40 cos 30°, 40 sin 30°, 30): length 50, foreshortened in every
    // principal view
    let mut sk = Sketch::new();
    let datum = |sk: &mut Sketch, o: (f64, f64), b: Basis, n: &str| {
        let oi = sk.point(o.0, o.1, true, &format!("{n}.o"));
        let ti = sk.point(o.0 + 1.0, o.1, true, &format!("{n}.t"));
        sk.plane(oi, ti, b, n)
    };
    let front = datum(&mut sk, (0.0, 0.0), Basis::page(), "front");
    let top = datum(&mut sk, (0.0, 100.0), Basis::page().fold(0.0), "top");
    let aux = datum(&mut sk, (200.0, 0.0), Basis::page().fold(0.0).fold(30f64.to_radians()), "aux");
    let af = sk.point(0.0, 0.0, true, "af");
    let bf = sk.point(40.0 * c30, 30.0, true, "bf");
    let at = sk.point(0.0, 100.0, true, "at");
    let bt = sk.point(40.0 * c30, 100.0 + 40.0 * s30, true, "bt");
    let aa = sk.point(205.0, 3.0, false, "aa");
    let ba = sk.point(230.0, -20.0, false, "ba");
    for (p, pl) in [(af, front), (bf, front), (at, top), (bt, top), (aa, aux), (ba, aux)] {
        sk.set_plane(p, Some(pl));
    }
    for (a, b) in [(at, aa), (af, aa), (bt, ba), (bf, ba)] {
        let c = Constraint::project(&sk, EntRef::point(a), EntRef::point(b)).unwrap();
        sk.add(c);
    }
    let r = solve(&mut sk, SolveOpts::default());
    assert!(r.success, "{}", r.message);
    let (ax, ay) = sk.point_xy(aa);
    let (bx, by) = sk.point_xy(ba);
    assert!(near((bx - ax).hypot(by - ay), 50.0), "true length: {}", (bx - ax).hypot(by - ay));
    assert!(near(ax, 200.0) && near(ay, 0.0), "A sits at the auxiliary origin: {ax}, {ay}");
}

#[test]
fn the_refusals() {
    let (mut sk, [front, top, _], [pf, pt, _]) = three_views();
    let loose = sk.point(5.0, 5.0, false, "loose");
    let e = |r: Result<Constraint, String>| r.err().unwrap_or_default();
    let m = e(Constraint::project(&sk, EntRef::point(pf), EntRef::point(loose)));
    assert!(m.contains("no plane"), "{m}");
    let other = sk.point(7.0, 7.0, false, "other");
    sk.set_plane(other, Some(front));
    let m = e(Constraint::project(&sk, EntRef::point(pf), EntRef::point(other)));
    assert!(m.contains("relates nothing to itself"), "{m}");
    // a second plane with the front's own basis is parallel to it
    let o = sk.point(300.0, 0.0, true, "o2");
    let t = sk.point(301.0, 0.0, true, "t2");
    let front2 = sk.plane(o, t, Basis::page(), "front2");
    sk.set_plane(other, Some(front2));
    let m = e(Constraint::project(&sk, EntRef::point(pf), EntRef::point(other)));
    assert!(m.contains("parallel"), "{m}");
    // and the pair that does relate still does
    assert!(Constraint::project(&sk, EntRef::point(pf), EntRef::point(pt)).is_ok());
    let _ = top;
    // the same three through a document, as `Err` and never a panic
    let doc = |planes: &str, memberships: [&str; 2]| {
        format!(
            "{{\"version\":1,\"points\":[{{\"x\":0,\"y\":0{}}},{{\"x\":1,\"y\":1{}}},\
             {{\"x\":0,\"y\":0,\"fixed\":true}},{{\"x\":1,\"y\":0,\"fixed\":true}}],\
             \"planes\":[{planes}],\
             \"constraints\":[{{\"type\":\"Project\",\
             \"args\":[[\"point\",0],[\"point\",1],null,null]}}]}}",
            memberships[0], memberships[1]
        )
    };
    let page = "{\"origin\":2,\"toward\":3,\"u\":[1,0,0],\"v\":[0,0,1]}";
    let top_p = "{\"origin\":2,\"toward\":3,\"u\":[1,0,0],\"v\":[0,1,0]}";
    let m = io::loads(&doc(page, ["", ""])).err().unwrap();
    assert!(m.contains("no plane"), "{m}");
    let m = io::loads(&doc(page, [",\"plane\":0", ",\"plane\":0"])).err().unwrap();
    assert!(m.contains("itself"), "{m}");
    let two = format!("{page},{page}");
    let m = io::loads(&doc(&two, [",\"plane\":0", ",\"plane\":1"])).err().unwrap();
    assert!(m.contains("parallel"), "{m}");
    let two = format!("{page},{top_p}");
    let sk = io::loads(&doc(&two, [",\"plane\":0", ",\"plane\":1"])).unwrap();
    assert_eq!(sk.user_constraints().len(), 1);
    let m = io::loads(&doc(&two, [",\"plane\":0", ",\"plane\":7"])).err().unwrap();
    assert!(m.contains("out of range"), "{m}");
}

#[test]
fn round_trips_through_json_and_the_graft() {
    let (mut sk, [front, top, right], [pf, pt, pr]) = three_views();
    let c = Constraint::project(&sk, EntRef::point(pf), EntRef::point(pt)).unwrap();
    let id = sk.add(c);
    sk.set_class(EntRef::plane(top), "section", true);
    let text = io::dumps(&sk, Some(1));
    assert!(!text.contains("FrameUnit"), "an intrinsic is never stored");
    let back = io::loads(&text).unwrap();
    assert_eq!(back.planes.len(), 3);
    assert_eq!(back.planes[right].basis, sk.planes[right].basis);
    assert_eq!(back.plane_of(pf), Some(front));
    assert_eq!(back.plane_of(pt), Some(top));
    assert_eq!(back.plane_of(pr), Some(right));
    assert!(back.class_of(EntRef::plane(top)).0.contains(&"section".to_string()));
    assert_eq!(back.user_constraints().len(), 1);
    assert_eq!(back.user_constraints()[0].entities().len(), 4);
    assert_eq!(io::dumps(&back, Some(1)), text);
    // a copy of everything keeps the projection; a copy of one view keeps its memberships
    let all = io::copy(&sk, &sk.primitives());
    assert_eq!(all.user_constraints().len(), 1);
    let one = io::copy(&sk, &[EntRef::plane(top), EntRef::point(pt)]);
    assert_eq!(one.planes.len(), 1);
    assert_eq!(one.plane_of(one.points.len() - 1), Some(0));
    assert!(one.user_constraints().is_empty(), "the other plane did not come");
    // deleting a plane drops the projection and clears the memberships that named it
    let less = io::without(&sk, &[EntRef::plane(top)], &[]);
    assert_eq!(less.planes.len(), 2);
    assert!(less.user_constraints().is_empty());
    assert!(less.points.iter().all(|p| p.plane.map_or(true, |q| (q as usize) < 2)));
    assert_eq!(less.points.iter().filter(|p| p.plane.is_none()).count(), 1 + 6);
    // deleting a point drops the projection and keeps the planes
    let less = io::without(&sk, &[EntRef::point(pf)], &[]);
    assert_eq!(less.planes.len(), 3);
    assert!(less.user_constraints().is_empty());
    let _ = id;
}

#[test]
fn a_claimed_projection_is_judged_and_moves_nothing() {
    let (mut sk, _, [pf, pt, _]) = three_views();
    let mut c = Constraint::project(&sk, EntRef::point(pf), EntRef::point(pt)).unwrap();
    c.claim = true;
    let id = sk.add(c);
    // the top image sits at x = 20 where the front says 30: a counterexample, left where it is
    let r = solve(&mut sk, SolveOpts::default());
    assert!(r.success, "{}", r.message);
    assert!(near(sk.point_xy(pt).0, 20.0), "a claim moves nothing");
    let d = diagnose::diagnose(&mut sk, Default::default());
    assert_eq!(d.claims_violated, vec![id]);
    assert!(d.violated.is_empty() && d.conflicts.is_none());
}

#[test]
fn the_drag_part_reaches_the_planes() {
    let (mut sk, [front, top, _], [pf, pt, _]) = three_views();
    // the front image is fixed in the fixture; free it, so the part is reached through it
    sk.fix_point(pf, false);
    let c = Constraint::project(&sk, EntRef::point(pf), EntRef::point(pt)).unwrap();
    sk.add(c);
    let part = io::Part::around(&sk, EntRef::point(pt));
    assert_eq!(part.sketch.planes.len(), 2, "both planes come with the projection");
    assert_eq!(part.sketch.user_constraints().len(), 1);
    let _ = (front, top);
}

/// The case in the library: the auxiliary view of the bracket's inclined face is placed by
/// projection alone, and comes out the true-size rectangle the face is.
#[test]
fn the_bracket_shows_its_incline_true_size() {
    let (prog, errs) = gcs_core::syntax::parse(gcs_core::examples::BRACKET);
    assert!(errs.is_empty(), "{errs:?}");
    let e = gcs_core::program::elaborate(&prog);
    assert!(e.ok(), "{:?}", e.errors().map(|d| d.message.clone()).collect::<Vec<_>>());
    let mut sk = e.sketch.clone();
    let r = solve(&mut sk, SolveOpts::default());
    assert!(r.success, "{}", r.message);
    let at = |n: &str| sk.point_xy(e.map.ent_named(n).unwrap().i());
    let d = |a: (f64, f64), b: (f64, f64)| (a.0 - b.0).hypot(a.1 - b.1);
    assert!(near(d(at("Fa"), at("Ea")), 15f64.hypot(10.0)), "{}", d(at("Fa"), at("Ea")));
    assert!(near(d(at("Fa"), at("F2a")), 30.0), "{}", d(at("Fa"), at("F2a")));
    assert!(near(d(at("Ea"), at("E2a")), 30.0));
    // and the right view's heights are the front's, its depths the top's
    assert!(near(at("Er").1, 40.0) && near(at("Cr").1, 15.0) && near(at("Fr").1, 30.0));
    assert!(near(at("A2r").0 - at("Ar").0, 30.0));
    assert_eq!(diagnose::diagnose(&mut sk, Default::default()).dof, 0);
}

#[test]
fn a_plane_is_picked_on_its_chord_and_a_point_wins() {
    let (sk, p) = with_plane();
    let o = sk.planes[p].frame.origin as usize;
    assert_eq!(pick(&sk, 12.0, 6.5, 0.5), Some(EntRef::plane(p)), "the chord's midpoint");
    assert_eq!(pick(&sk, 10.0, 5.0, 0.5), Some(EntRef::point(o)), "the origin is a point");
    assert_eq!(pick(&sk, 12.0, 9.0, 0.5), None);
}

/// A document is untrusted input, and `wasm32-unknown-unknown` aborts rather than unwinding —
/// so an entity argument whose *kind* is not the one its slot names must be refused where it is
/// read.  Every reader past that point indexes the list the spec names (a projection's planes
/// reach `sk.planes`), which for a line would be an out-of-bounds panic.
#[test]
fn a_projection_pointed_at_the_wrong_kind_is_refused() {
    let doc = |args: &str| {
        format!(
            "{{\"version\":1,\"points\":[{{\"x\":0,\"y\":0}},{{\"x\":1,\"y\":1}}],\
             \"lines\":[{{\"p1\":0,\"p2\":1}},{{\"p1\":0,\"p2\":1}}],\"planes\":[],\
             \"constraints\":[{{\"type\":\"Project\",\"args\":{args}}}]}}"
        )
    };
    for args in [
        "[[\"point\",0],[\"point\",1],[\"line\",0],[\"line\",1]]",
        "[[\"point\",0],[\"point\",1],[\"line\",0],[\"line\",0]]",
        "[[\"line\",0],[\"point\",1],null,null]",
    ] {
        let m = io::loads(&doc(args)).err().unwrap_or_default();
        assert!(m.contains("does not take"), "expected a kind refusal, got {m:?}");
    }
    // and an index the sketch does not have is still out of range, not a panic
    let m = io::loads(&doc("[[\"point\",0],[\"point\",1],[\"plane\",9],[\"plane\",0]]"))
        .err()
        .unwrap_or_default();
    assert!(m.contains("out of range"), "{m}");
}

/// The datum glyph is the core's, and one figure: `svg.rs` and the canvas both stroke what
/// `plane::glyph` hands them, so the exported picture and the drawn one cannot come apart —
/// which they had, at the tick's length.
#[test]
fn the_glyph_is_laid_out_once_and_screen_constant() {
    let (sk, p) = with_plane();
    let unit = 0.5;                                   // half a world unit to the screen pixel
    let [(o, t), (o2, tick)] = gcs_core::plane::glyph(&sk, p, unit);
    assert_eq!(o, sk.point_xy(sk.planes[p].frame.origin as usize));
    assert_eq!(t, sk.point_xy(sk.planes[p].frame.toward as usize));
    assert_eq!(o2, o, "the tick comes out of the origin");
    // perpendicular to the chord, and a screen-constant length
    let (cx, cy) = (t.0 - o.0, t.1 - o.1);
    let (tx, ty) = (tick.0 - o.0, tick.1 - o.1);
    assert!(near(cx * tx + cy * ty, 0.0), "the tick is the frame's y-axis");
    assert!(near(tx.hypot(ty), gcs_core::plane::TICK_PX * unit));
    // and it is drawn: the SVG export strokes two segments for the plane
    let svg = gcs_core::svg::render(&sk, 400.0);
    assert!(svg.contains("<line"), "{svg}");
}
