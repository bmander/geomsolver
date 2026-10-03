//! The workspace (`overview::workspace`): every view standing on its own plane, seen by one
//! orthographic eye, with what is under the pointer asked where the eye sees it — because views
//! that lie on top of one another on the page are nowhere near one another in space.
use gcs_core::model::{EntKind, EntRef, Sketch};
use gcs_core::overview::workspace::{apply, inside, look_at, nearest_point, pick, Projection};
use gcs_core::{callout, library, program};
use std::f64::consts::{FRAC_PI_2, PI};

fn build(src: &str) -> program::Elaborated {
    let (p, errors, linked) = library::parse_linked(src);
    assert!(errors.is_empty() && linked.is_empty(), "{errors:?} {linked:?}");
    let mut e = program::elaborate(&p);
    assert!(e.ok(), "{:?}", e.diags);
    assert!(gcs_core::solve::solve(&mut e.sketch, Default::default()).success);
    e
}

fn close(a: (f64, f64), b: (f64, f64)) {
    assert!((a.0 - b.0).hypot(a.1 - b.1) < 1e-9, "{a:?} != {b:?}");
}

/// The eye's picture-plane coordinates of a point in space, written out from `overview::eye`'s
/// convention: the viewer at (cos az cos el, sin az cos el, sin el), looking at the origin.
fn seen(x: [f64; 3], az: f64, el: f64) -> (f64, f64) {
    let right = [-az.sin(), az.cos(), 0.0];
    let up = [-az.cos() * el.sin(), -az.sin() * el.sin(), el.cos()];
    let dot = |a: [f64; 3], b: [f64; 3]| a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    (dot(right, x), dot(up, x))
}

fn plane(e: &program::Elaborated, name: &str) -> usize {
    e.map.ent_named(name).unwrap().i()
}

const VIEWS: &str = "unit mm\nuse std\n\
    f := point hint(x: 10, y: 4) in std.front\nfix(x == 10, y == 4) f\n\
    s := point hint(x: 10, y: 4) in std.side\nfix(x == 10, y == 4) s\n\
    t := point hint(x: 10, y: 4) in std.top\nfix(x == 10, y == 4) t\n\
    p := point hint(x: 10, y: 4)\nfix(x == 10, y == 4) p\n";

/// From the front, seen square on, the page is the picture: the map is the identity.
#[test]
fn the_page_seen_from_the_front_is_the_picture() {
    let e = build(VIEWS);
    let proj = Projection::new(&e.sketch, -FRAC_PI_2, 0.0);
    let m = proj.map(None);
    for (a, b) in m.iter().zip([1.0, 0.0, 0.0, 0.0, 1.0, 0.0]) {
        assert!((a - b).abs() < 1e-12, "{m:?}");
    }
}

/// Every view's map is its page placement, its lift and the eye, multiplied out: the same place
/// `Basis::lift` stands a point at, seen from anywhere.
#[test]
fn a_map_is_the_lift_seen_by_the_eye() {
    let e = build(VIEWS);
    let sk = &e.sketch;
    // side: page x runs along +y and page y up +z; top: page x along +x and page y along +y
    let space = [("f", [10.0, 0.0, 4.0]), ("s", [0.0, 10.0, 4.0]), ("t", [10.0, 4.0, 0.0]),
                 ("p", [10.0, 0.0, 4.0])];
    for (az, el) in [(-FRAC_PI_2, 0.0), (0.3, 0.4), (-2.0, -0.7), (1.0, 1.2)] {
        let proj = Projection::new(sk, az, el);
        for (name, x) in space {
            let i = e.map.ent_named(name).unwrap().i();
            close(proj.point(sk, i), seen(x, az, el));
            close(apply(proj.map(proj.view_of(i)), sk.point_xy(i)), seen(x, az, el));
        }
    }
}

/// `look_at` turns the eye square on to a plane, with the plane's `u` to the right: its map is
/// then a rotation-free similarity of the page — the plane seen as it is drawn.
#[test]
fn the_eye_looks_square_on_to_each_standard_plane() {
    let e = build(VIEWS);
    let sk = &e.sketch;
    let (az, el) = look_at(&sk.basis(plane(&e, "std.front")));
    assert!((az + FRAC_PI_2).abs() < 1e-12 && el.abs() < 1e-12, "{az} {el}");
    let (az, el) = look_at(&sk.basis(plane(&e, "std.side")));
    assert!(az.abs() < 1e-12 && el.abs() < 1e-12, "{az} {el}");
    let (_, el) = look_at(&sk.basis(plane(&e, "std.top")));
    assert!((el - FRAC_PI_2).abs() < 1e-12, "{el}");
    for name in ["std.front", "std.side", "std.top"] {
        let i = plane(&e, name);
        let (az, el) = look_at(&sk.basis(i));
        let m = *Projection::new(sk, az, el).map(Some(i));
        // the page's x runs right and its y up, unturned and unstretched
        for (a, b) in m[..2].iter().chain(&m[3..5]).zip([1.0, 0.0, 0.0, 1.0]) {
            assert!((a - b).abs() < 1e-12, "{name}: {m:?}");
        }
    }
    // and the bearing is free only at the poles: anywhere else the normal decides it
    let (az, _) = look_at(&gcs_core::plane::Basis::page().fold(PI / 3.0));
    assert!(az.is_finite());
}

/// Four points with the same page coordinates in four views: on the page they are one spot, and
/// in space three of them are not — so a pick is asked where the eye sees them.
#[test]
fn a_pick_tells_apart_views_that_overlap_on_the_page() {
    let e = build(VIEWS);
    let sk = &e.sketch;
    let named = |n: &str| e.map.ent_named(n).unwrap();
    let (az, el) = (0.6, 0.5);
    let proj = Projection::new(sk, az, el);
    for (name, x) in [("s", [0.0, 10.0, 4.0]), ("t", [10.0, 4.0, 0.0])] {
        let at = seen(x, az, el);
        assert_eq!(pick(sk, &proj, at, 0.5, 0.01), Some(named(name)), "{name}");
        assert_eq!(nearest_point(sk, &proj, at).0, Some(named(name).i()));
    }
    // the page and std.front are one place in space, so either is the answer there
    let got = pick(sk, &proj, seen([10.0, 0.0, 4.0], az, el), 0.5, 0.01).unwrap();
    assert!(got == named("f") || got == named("p"), "{got:?}");
    // a band round where the side's point is seen holds it and nothing of the other views
    let (cx, cy) = seen([0.0, 10.0, 4.0], az, el);
    let held = inside(sk, &proj, (cx - 0.5, cy - 0.5), (cx + 0.5, cy + 0.5), 0.01);
    assert_eq!(held, vec![named("s")]);
}

/// A line drawn on a plane is picked along its figure there, and a projector between two views
/// is drawn — and picked — between where its two ends stand.
#[test]
fn lines_are_picked_where_their_ends_stand() {
    let e = build(&format!("{VIEWS}l := line(s, t)\nm := line(hint(x: 0, y: 0), hint(x: 0, y: 8)) in std.side\n\
        fix(x == 0, y == 0) m.p1\nfix(x == 0, y == 8) m.p2\n"));
    let sk = &e.sketch;
    let (az, el) = (0.6, 0.5);
    let proj = Projection::new(sk, az, el);
    let mid = |a: [f64; 3], b: [f64; 3]| [(a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0, (a[2] + b[2]) / 2.0];
    let l = e.map.ent_named("l").unwrap();
    assert_eq!(pick(sk, &proj, seen(mid([0.0, 10.0, 4.0], [10.0, 4.0, 0.0]), az, el), 0.2, 0.01), Some(l));
    // `m` stands on the side plane from its origin up page y: world z from 0 to 8
    let m = e.map.ent_named("m").unwrap();
    assert_eq!(m.kind, EntKind::Line);
    assert_eq!(pick(sk, &proj, seen([0.0, 0.0, 4.0], az, el), 0.2, 0.01), Some(m));
}

/// A circle on a tilted plane is seen as an ellipse, and picked on its rim there, not where the
/// page would put it.
#[test]
fn a_circle_on_a_tilted_plane_is_picked_on_its_rim() {
    let e = build("unit mm\nuse std\nc := circle(center: o) hint(r: 5) in std.top\no := point in std.top\n\
        fix(x == 0, y == 0) o\nfix(r == 5) c\n");
    let sk = &e.sketch;
    let (az, el) = (0.6, 0.5);
    let proj = Projection::new(sk, az, el);
    let c = e.map.ent_named("c").unwrap();
    // on the rim in space: (5, 0, 0) on the top plane
    assert_eq!(pick(sk, &proj, seen([5.0, 0.0, 0.0], az, el), 0.1, 0.01), Some(c));
    // where the page alone would have put the rim, seen from here, is off it
    let off = seen([0.0, 0.0, 5.0], az, el);
    assert_ne!(pick(sk, &proj, off, 0.1, 0.01), Some(c));
}

/// A dimension's callout is laid out in its view and picked where the eye sees its number.
#[test]
fn a_callout_is_picked_where_the_eye_sees_it() {
    let e = build("unit mm\nuse std\na := point in std.side\nb := point hint(x: 30, y: 0) in std.side\n\
        fix(x == 0, y == 0) a\na distance(30mm) b\nhorizontal line(a, b)\n");
    let sk = &e.sketch;
    let unit = 0.05;
    let k = &callout::layout(sk, unit)[0];
    let id = k.id;
    // the label's middle, in the side's page, then seen
    let mid = (
        k.label.iter().map(|p| p.0).sum::<f64>() / 4.0,
        k.label.iter().map(|p| p.1).sum::<f64>() / 4.0,
    );
    for (az, el) in [(0.0, 0.0), (0.6, 0.5)] {
        let proj = Projection::new(sk, az, el);
        let at = apply(proj.map(Some(plane(&e, "std.side"))), mid);
        let side = Some(plane(&e, "std.side"));
        assert_eq!(callout::pick_seen(sk, unit, &proj, at, 8.0), Some((id, side)), "({az}, {el})");
        // the page reading of the same spot, which is where the front plane would be, misses
        let wrong = apply(proj.map(None), mid);
        if (wrong.0 - at.0).hypot(wrong.1 - at.1) > 1.0 {
            assert_eq!(callout::pick_seen(sk, unit, &proj, wrong, 8.0), None);
        }
    }
}

/// The workspace report says each view's place and each entity's view by the wire's codes.
#[test]
fn the_report_says_each_view() {
    let e = build(&format!("{VIEWS}l := line(s, t)\n"));
    let sk: &Sketch = &e.sketch;
    let parsed = gcs_core::json::parse(&gcs_core::report::workspace_json(sk).dump(None)).unwrap();
    assert_eq!(parsed.get("places").unwrap().arr().len(), 1 + sk.planes.len());
    assert_eq!(parsed.get("looks").unwrap().arr().len(), 1 + sk.planes.len());
    let lines = parsed.get("views").unwrap().get("line").unwrap().arr();
    // the projector between the side and the top stands in no one view
    let l = e.map.ent_named("l").unwrap();
    assert_eq!(lines[l.i()].as_i64(), -2);
    let points = parsed.get("views").unwrap().get("point").unwrap().arr();
    let s = e.map.ent_named("s").unwrap();
    assert_eq!(points[s.i()].as_i64(), plane(&e, "std.side") as i64);
    let p = e.map.ent_named("p").unwrap();
    assert_eq!(points[p.i()].as_i64(), -1);
    let _ = EntRef::point(0);
}

/// A view's place is the plane it stands on in space, judged there and not by any eye: `std.up`
/// is the page turned, so one place with it, and a plane stood off the front along its normal is
/// another place — though from straight in front the two look the same.
#[test]
fn a_place_is_a_plane_in_space() {
    let e = build("unit mm\nuse std\no := point\nfix(x == 0, y == 0) o\nq := point\nfix(x == 1, y == 0) q\n\
        back := plane(origin: o, toward: q, from: std.front, offset: 10mm)\n");
    let views = gcs_core::overview::workspace::Views::new(&e.sketch);
    let (front, up, back) = (plane(&e, "std.front"), plane(&e, "std.up"), plane(&e, "back"));
    assert_eq!(views.place(None), None, "the page is its own first place");
    assert_eq!(views.place(Some(front)), None, "std.front is the page");
    assert_eq!(views.place(Some(up)), None, "std.up is the page turned");
    assert_eq!(views.place(Some(back)), Some(back), "stood off is elsewhere");
    let proj = Projection::new(&e.sketch, -FRAC_PI_2, 0.0);
    let (m, n) = (proj.map(Some(back)), proj.map(None));
    assert!(m.iter().zip(n).all(|(a, b)| (a - b).abs() < 1e-12), "though from in front it looks the same");
}

/// A fit frames what is shown, solids included: the flange's section draws only the half of it
/// right of the axis, and the turned part reaches as far the other way.
#[test]
fn the_bounds_reach_past_a_turned_section() {
    let e = build(gcs_core::examples::source("solid_flange").unwrap());
    let proj = Projection::new(&e.sketch, -FRAC_PI_2, 0.0);
    let (x0, _, x1, _) = gcs_core::overview::workspace::bounds(&e.sketch, &proj, 0.05).unwrap();
    assert!(x0 <= -31.9 && x1 >= 31.9, "{x0} {x1}");
}
