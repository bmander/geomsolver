//! The workspace (`overview::workspace`): every view standing on its own plane, seen by one
//! orthographic eye, with what is under the pointer asked where the eye sees it — because views
//! that lie on top of one another on the page are nowhere near one another in space.
use gcs_core::model::{EntKind, EntRef, Sketch};
use gcs_core::overview::workspace::{apply, look_at, nearest_point, overlapping, panes_at, pick, Projection};
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

/// A point in each standard plane, and one in space.
const VIEWS: &str = "unit mm\nuse std\n\
    f := point hint((10, 4)) in std.front\nfix((10, 4)) f\n\
    s := point hint((10, 4)) in std.side\nfix((10, 4)) s\n\
    t := point hint((10, 4)) in std.top\nfix((10, 4)) t\n\
    p := point hint((10, 4))\nfix((10, 4, 0)) p\n";

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
    // side: page x runs along +y and page y up +z; top: page x along +x and page y along +y;
    // `p` stands in space where its three numbers say, in no view
    let space = [("f", [10.0, 0.0, 4.0]), ("s", [0.0, 10.0, 4.0]), ("t", [10.0, 4.0, 0.0]),
                 ("p", [10.0, 4.0, 0.0])];
    for (az, el) in [(-FRAC_PI_2, 0.0), (0.3, 0.4), (-2.0, -0.7), (1.0, 1.2)] {
        let proj = Projection::new(sk, az, el);
        for (name, x) in space {
            let i = e.map.ent_named(name).unwrap().i();
            close(proj.point(sk, i), seen(x, az, el));
            if name != "p" {
                close(apply(proj.map(proj.view_of(i)), sk.point_xy(i)), seen(x, az, el));
            }
        }
        let p = e.map.ent_named("p").unwrap().i();
        let (x, y) = gcs_core::overview::workspace::space_points(sk, az, el)[p].unwrap();
        close((x, y), seen([10.0, 4.0, 0.0], az, el));
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
    let tilted = gcs_core::plane::Basis::explicit([1.0, 0.0, 0.0], [0.0, (PI / 3.0).cos(), (PI / 3.0).sin()]);
    let (az, _) = look_at(&tilted.unwrap());
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
    let held = overlapping(sk, &proj, (cx - 0.5, cy - 0.5), (cx + 0.5, cy + 0.5), 0.01);
    assert_eq!(held, vec![named("s")]);
}

/// A band takes whatever it touches — "crossing" selection — and only what is drawn: a line
/// through it with both ends outside, a circle whose rim passes through it, never a circle round
/// it, nor a line beside it.
#[test]
fn a_band_takes_what_it_touches() {
    let e = build("unit mm\nuse std\nin std.front {\n\
        a := point hint((0, 0))\nfix((0, 0)) a\nb := point hint((20, 0))\nfix((20, 0)) b\n\
        across := line(a, b)\n\
        c := point hint((0, 10))\nfix((0, 10)) c\nd := point hint((20, 10))\nfix((20, 10)) d\n\
        beside := line(c, d)\n\
        o := point hint((50, 0))\nfix((50, 0)) o\nring := circle(center: o) hint(r: 10)\n\
        radius(10) ring\n}\n");
    let sk = &e.sketch;
    let named = |n: &str| e.map.ent_named(n).unwrap();
    let proj = Projection::new(sk, -FRAC_PI_2, 0.0);
    // over the middle of `across`, short of `beside` and either end
    assert_eq!(overlapping(sk, &proj, (8.0, -2.0), (12.0, 2.0), 0.01), vec![named("across")]);
    // the same box drawn from its other corner is the same box
    assert_eq!(overlapping(sk, &proj, (12.0, 2.0), (8.0, -2.0), 0.01), vec![named("across")]);
    // across the circle's rim, inside its centre's reach: the rim and the centre
    let mut held = overlapping(sk, &proj, (48.0, -2.0), (62.0, 2.0), 0.01);
    held.sort();
    let mut want = vec![named("o"), named("ring")];
    want.sort();
    assert_eq!(held, want);
    // wholly inside the rim, off the centre: nothing is drawn there
    assert!(overlapping(sk, &proj, (53.0, 3.0), (55.0, 5.0), 0.01).is_empty());
}

/// A pane answers over its whole face, the nearest the eye first: seen from above and in front,
/// a place on the top pane in front of the front pane is the top's, with the front's behind it.
#[test]
fn a_pane_is_found_where_the_eye_sees_it_nearest_first() {
    let e = build(VIEWS);
    let sk = &e.sketch;
    let (top, front, side) = (plane(&e, "std.top"), plane(&e, "std.front"), plane(&e, "std.side"));
    let (az, el) = (-FRAC_PI_2, 0.5);
    let proj = Projection::new(sk, az, el);
    // the top pane in front of the front: (5, -0.3) lies off the front plane's y = 0, toward the eye
    let hits = panes_at(sk, &proj, seen([5.0, -0.3, 0.0], az, el));
    assert_eq!(hits.first(), Some(&top), "{hits:?}");
    assert!(hits.contains(&front), "the front behind it: {hits:?}");
    // a place on the front above the top: the front's place first (std.up stands there too)
    let hits = panes_at(sk, &proj, seen([5.0, 0.0, 3.0], az, el));
    let place = |i: usize| proj.views.place(Some(i));
    assert_eq!(place(hits[0]), place(front), "{hits:?}");
    // the side is seen edge on from the front, and is no place to land
    assert!(!hits.contains(&side), "{hits:?}");
    // far off every pane
    assert!(panes_at(sk, &proj, (1e4, 1e4)).is_empty());
}

/// A line drawn on a plane is picked along its figure there, and a projector between two views
/// is drawn — and picked — between where its two ends stand.
#[test]
fn lines_are_picked_where_their_ends_stand() {
    let e = build(&format!("{VIEWS}l := line(s, t)\nm := line(hint((0, 0)), hint((0, 8))) in std.side\n\
        fix((0, 0)) m.p1\nfix((0, 8)) m.p2\n"));
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
        fix((0, 0)) o\nfix(r == 5) c\n");
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
    let e = build("unit mm\nuse std\na := point in std.side\nb := point hint((30, 0)) in std.side\n\
        fix((0, 0)) a\na distance(30mm) b\nhorizontal line(a, b)\n");
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
    // a point in space stands in no view: it is seen where it is (`space_points`)
    let p = e.map.ent_named("p").unwrap();
    assert_eq!(points[p.i()].as_i64(), -2);
    let _ = EntRef::point(0);
}

/// A view's place is the plane it stands on in space, judged there and not by any eye: `std.up`
/// is the page turned, so one place with it, and a plane stood off the front along its normal is
/// another place — though from straight in front the two look the same.
#[test]
fn a_place_is_a_plane_in_space() {
    // the front stood off along its normal, which is -y
    let e = build("unit mm\nuse std\nback := plane\nfix(origin == (0, -10, 0)) back\nfix(dir == (1, 0, 0)) back.u\nfix(dir == (0, 0, 1)) back.v\n");
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

/// A curve owns no point, so the view it is drawn in is what it is written over's: an envelope of
/// a point drawn in `std.top`, and an envelope of that envelope, stand in `std.top` — drawn flat
/// in the plane at z = 0 — and not on the page the front stands for (the Wankel's bore and flank).
#[test]
fn a_curve_is_drawn_in_the_view_of_what_it_is_written_over() {
    let e = build("unit mm
use std
in std.top {
  centre := point
  fix((0, 0)) centre
  hub := point
  fix((15, 0)) hub
  apex := point
  fix((120, 0)) apex
}
counter := motion(about: centre, ratio: -1)
spin := motion(about: hub, ratio: -2 / 3)
rotor_turn := motion(spin, relative_to: counter)
housing_turn := motion(counter, relative_to: spin)
bore := envelope(apex, under: rotor_turn, from: 0deg, to: 1080deg)
flank := envelope(bore, under: housing_turn, from: 10deg, to: 170deg)
");
    let top = plane(&e, "std.top");
    let views = gcs_core::overview::workspace::Views::new(&e.sketch);
    let scene = gcs_core::overview::scene3d(&e.sketch, 0.1);
    for name in ["bore", "flank"] {
        let c = e.map.ent_named(name).unwrap();
        assert_eq!(views.entity_view(&e.sketch, c), Ok(Some(top)), "{name}");
        let drawn: Vec<_> = scene.iter().filter(|it| it.of == Some(c)).collect();
        assert!(!drawn.is_empty(), "{name} is drawn");
        for it in drawn {
            assert_eq!(it.in_plane, Some(EntRef::plane(top)), "{name}");
            assert!(it.pts.iter().all(|p| p[2].abs() < 1e-9), "{name} lies in the plane z = 0");
            assert!(it.pts.iter().any(|p| p[1].abs() > 1.0), "{name} reaches off the x axis");
        }
    }
}

/// A 20 × 10 block on `std.top`, 5 deep below it, and a second one under it — seen from above,
/// the upper hides the lower — and a third with a bore cut through it, at x = 40.
const BLOCKS: &str = "unit mm\nuse std\n\
    component Slab(x: Length) {\n\
      a := point hint((x, 0))\nb := point hint((x + 20mm, 0))\n\
      c := point hint((x + 20mm, 10))\nd := point hint((x, 10))\n\
      ab := line(a, b)\nbc := line(b, c)\ncd := line(c, d)\nda := line(d, a)\n\
      sec := face(ab, bc, cd, da)\n\
    }\n\
    in std.top {\nupper := Slab(x: 0mm)\nlower := Slab(x: 0mm)\nholed := Slab(x: 40mm)\n\
    o := point hint((50, 5))\nhole := circle(center: o) hint(r: 3)\n}\n\
    top := solid(upper.sec, depth: 5)\n\
    bottom := solid(lower.sec, from: -30, to: -20)\n\
    stock := solid(holed.sec, depth: 5)\nbody := solid(stock)\n\
    bore := solid(face(hole), through: body)\nbore cut body\n";

#[test]
fn a_click_on_a_solid_picks_the_face_nearest_the_eye() {
    use gcs_core::overview::workspace::pick_solid;
    let e = build(BLOCKS);
    let sk = &e.sketch;
    let solid = |n: &str| e.map.ent_named(n).unwrap().i();
    let (az, el) = (0.6, 0.5);
    let proj = Projection::new(sk, az, el);
    // the middle of the top block's upper face: the ray meets it there, before the lower block
    let hit = pick_solid(sk, &proj, seen([10.0, 5.0, 0.0], az, el)).expect("the top block");
    assert_eq!(hit.solid, solid("top"));
    let toward = [az.cos() * el.cos(), az.sin() * el.cos(), el.sin()];
    let expected = 10.0 * toward[0] + 5.0 * toward[1];
    assert!((hit.depth - expected).abs() < 1e-9, "{} {expected}", hit.depth);
    assert!(hit.face.starts_with("top."), "{}", hit.face);
    // past every object, nothing
    assert_eq!(pick_solid(sk, &proj, seen([10.0, 80.0, 0.0], az, el)), None);
    // a face of a body is named where it was made — the stock's — and the object is the body
    let hit = pick_solid(sk, &proj, seen([43.0, 5.0, 0.0], az, el)).expect("the body");
    assert_eq!(hit.solid, solid("body"));
    assert!(hit.face.starts_with("stock."), "{}", hit.face);
    // and from overhead, down the bore, the ray passes through the hole
    let overhead = Projection::new(sk, az, 1.5);
    assert_eq!(pick_solid(sk, &overhead, seen([50.0, 5.0, 0.0], az, 1.5)), None);
    // the box beneath: the lower block alone, from below
    let under = Projection::new(sk, az, -0.5);
    let hit = pick_solid(sk, &under, seen([10.0, 5.0, -30.0], az, -0.5)).expect("the bottom block");
    assert_eq!(hit.solid, solid("bottom"));
}

/// Axes on a drawing that reaches 50 out: a line along part of `std.x`, and an axis parallel to
/// `std.y` through (10, 0, 10) — seen end on from the front.
const AXES: &str = "unit mm\nuse std\nin std.front {\n\
    a := point hint((5, 0))\nb := point hint((25, 0))\nfar := point hint((40, 30))\n\
    on_x := line(a, b)\nfix((5, 0)) a\nfix((25, 0)) b\nfix((40, 30)) far\n}\n\
    t := axis hint(dir: (0, 1, 0))\nfix(dir == (0, 1, 0), origin == (10, 0, 10)) t\n";

/// An axis is picked along the line the box draws it as, under anything drawn, never end on.
#[test]
fn an_axis_is_picked_where_it_is_drawn_and_the_drawing_outranks_it() {
    let e = build(AXES);
    let sk = &e.sketch;
    let axis = |n: &str| Some(e.map.ent_named(n).unwrap());
    let (az, el) = (0.6, 0.5);
    let proj = Projection::new(sk, az, el);
    // along std.x past the drawn line: the axis; on the line: the line
    assert_eq!(pick(sk, &proj, seen([35.0, 0.0, 0.0], az, el), 0.5, 0.01), axis("std.x"));
    assert_eq!(pick(sk, &proj, seen([15.0, 0.0, 0.0], az, el), 0.5, 0.01), axis("on_x"));
    // the drawn axis, from three quarters
    assert_eq!(pick(sk, &proj, seen([10.0, 20.0, 10.0], az, el), 0.5, 0.01), axis("t"));
    // from the front it is seen end on, a dot: not picked
    let front = Projection::new(sk, -FRAC_PI_2, 0.0);
    assert_eq!(pick(sk, &front, seen([10.0, 0.0, 10.0], -FRAC_PI_2, 0.0), 0.5, 0.01), None);
}

/// The box asks for the scene in layers: the datums every frame a drag moves them, the objects'
/// creases once the pose settles.  Each layer is its part of the whole scene and nothing else.
#[test]
fn the_scene_comes_in_a_datum_layer_and_an_object_layer() {
    use gcs_core::overview::{scene3d, scene3d_of, Layer};
    let e = build(BLOCKS);
    let sk = &e.sketch;
    let part = |it: &gcs_core::overview::Item3| format!("{:?}", it.what);
    let all = scene3d(sk, 0.0);
    let datums = scene3d_of(sk, 0.0, Layer::Datums);
    let objects = scene3d_of(sk, 0.0, Layer::Objects);
    assert!(datums.iter().any(|it| part(it) == "Face") && datums.iter().any(|it| part(it) == "Axis"));
    // the axes in space are datums; the sheet's own drawing and the creases are not
    assert!(datums.iter().all(|it| part(it) != "Solid"));
    assert!(datums.iter().filter(|it| part(it) == "Drawn").all(|it| it.of.unwrap().kind == EntKind::Axis));
    assert!(!objects.is_empty() && objects.iter().all(|it| part(it) == "Solid"));
    let creases = |s: &[gcs_core::overview::Item3]| s.iter().filter(|it| part(it) == "Solid").count();
    assert_eq!(creases(&objects), creases(&all));
}
