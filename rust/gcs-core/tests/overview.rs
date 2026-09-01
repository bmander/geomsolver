//! The drawing folded back into the box it was unfolded from (§6.7): each view on its own plane,
//! and the object reconstructed from the views that see it.
use gcs_core::constraints::Constraint;
use gcs_core::model::{EntRef, Sketch};
use gcs_core::overview::{corners, drawable, scene, Part};
use gcs_core::plane::Basis;
use std::f64::consts::FRAC_PI_2;

fn near(a: f64, b: f64) -> bool {
    (a - b).abs() < 1e-8
}

/// The three-view fixture `tests/plane.rs` reconstructs its point from: front, top and right in
/// the standard third-angle layout, and the images of X = (30, 20, 40) placed exactly.
fn three_views() -> (Sketch, [usize; 3]) {
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
    // the images of X = (30, 20, 40), each where its own view sees it
    let pf = sk.point(30.0, 40.0, true, "pf");
    let pt = sk.point(30.0, 120.0, true, "pt");
    let pr = sk.point(170.0, 40.0, true, "pr");
    sk.set_plane(pf, Some(front));
    sk.set_plane(pt, Some(top));
    sk.set_plane(pr, Some(right));
    for (a, b) in [(pf, pt), (pf, pr), (pt, pr)] {
        let c = Constraint::project(&sk, EntRef::point(a), EntRef::point(b)).unwrap();
        sk.add(c);
    }
    (sk, [pf, pt, pr])
}

/// **The claim the whole overview rests on**: three images of one corner put it back where it
/// came from, exactly, with no solving.
#[test]
fn three_views_reconstruct_the_corner_they_are_of() {
    let (sk, [pf, pt, pr]) = three_views();
    let cs = corners(&sk);
    // three projections over the corner, and the three plane origins pairwise
    let of = |a: usize, b: usize| {
        cs.iter().find(|c| c.images == [a, b] || c.images == [b, a]).map(|c| c.at)
    };
    for (a, b) in [(pf, pt), (pf, pr), (pt, pr)] {
        let x = of(a, b).unwrap_or_else(|| panic!("{a} and {b} place no corner"));
        assert!(near(x[0], 30.0) && near(x[1], 20.0) && near(x[2], 40.0), "{x:?}");
    }
    // every pair of views agrees on where the one corner is — to arithmetic, not to the bit:
    // each pair is its own least-squares solve
    let (a, b) = (of(pf, pt).unwrap(), of(pt, pr).unwrap());
    assert!((0..3).all(|i| near(a[i], b[i])), "{a:?} vs {b:?}");
}

/// A point seen in one view is a *ray*, not a place — rank 2, and left standing on its plane.
#[test]
fn one_view_places_nothing() {
    let mut sk = Sketch::new();
    let o = sk.point(0.0, 0.0, true, "o");
    let t = sk.point(1.0, 0.0, true, "t");
    let front = sk.plane(o, t, Basis::page(), "front");
    let p = sk.point(5.0, 7.0, false, "p");
    sk.set_plane(p, Some(front));
    assert!(corners(&sk).is_empty(), "one view cannot say where a point is");

    // two images on *parallel* planes are two rays in one direction, and still say nothing:
    // `Project` refuses the pair, so nothing ties them and no class forms
    let o2 = sk.point(50.0, 0.0, true, "o2");
    let t2 = sk.point(51.0, 0.0, true, "t2");
    let front2 = sk.plane(o2, t2, Basis::page(), "front2");
    let q = sk.point(55.0, 7.0, false, "q");
    sk.set_plane(q, Some(front2));
    assert!(Constraint::project(&sk, EntRef::point(p), EntRef::point(q)).is_err());
    // the two planes' origins are still a pair, and being parallel they place nothing
    assert!(corners(&sk).is_empty());
}

/// A claim is judged and never solved for, so it relates nothing — including here.
#[test]
fn a_claimed_projection_ties_no_images() {
    let (mut sk, [pf, pt, _]) = three_views();
    sk.constraints.clear();
    let mut c = Constraint::project(&sk, EntRef::point(pf), EntRef::point(pt)).unwrap();
    c.claim = true;
    sk.add(c);
    // the origins still pair; what must not appear is a corner made by the claim
    assert!(
        !corners(&sk).iter().any(|c| c.images.contains(&pf) && c.images.contains(&pt)),
        "a claim welds no two images into one corner",
    );
}

/// The projection is orthographic: looked at a view square on, the box flattens to exactly the
/// picture that view draws, and parallel stays parallel from anywhere.
#[test]
fn the_projection_is_orthographic() {
    let (mut sk, [pf, _, _]) = three_views();
    let qf = sk.point(60.0, 90.0, true, "qf");
    sk.set_plane(qf, Some(0));
    let lf = sk.line(pf, qf);

    // the front plane's viewer stands at −y, which is bearing −90° at no elevation; there the
    // screen axes are exactly the page's own (right = x, up = z), so a front-view point comes
    // out at the view coordinates the draughtsman measured
    let s = scene(&sk, 1.0, -FRAC_PI_2, 0.0);
    let line = s.items.iter().find(|i| i.of == Some(EntRef::line(lf)) && i.what == Part::Drawn);
    let pts = &line.expect("the front view's line is in the scene").pts;
    for (&got, want) in pts.iter().zip([(30.0, 40.0), (60.0, 90.0)]) {
        assert!(near(got.0, want.0) && near(got.1, want.1), "{got:?} is not {want:?}");
    }
    // and it is a rigid picture of that view: the length is carried faithfully
    assert!(near((pts[1].0 - pts[0].0).hypot(pts[1].1 - pts[0].1), 30f64.hypot(50.0)));

    // from a general angle, a face's opposite sides stay parallel — which is what "orthographic"
    // buys and perspective would take away
    let s = scene(&sk, 1.0, 0.7, 0.4);
    let faces: Vec<_> = s.items.iter().filter(|i| i.what == Part::Face).collect();
    assert_eq!(faces.len(), 3, "one pane per view");
    for face in faces {
        let p = &face.pts;
        let (e0, e2) = ((p[1].0 - p[0].0, p[1].1 - p[0].1), (p[3].0 - p[2].0, p[3].1 - p[2].1));
        assert!(near(e0.0 * e2.1 - e0.1 * e2.0, 0.0), "opposite sides stayed parallel");
    }
}

/// The scene carries the three kinds apart, and the solid appears only where the views agree.
#[test]
fn the_scene_holds_the_box_and_the_solid() {
    let (mut sk, [pf, pt, _]) = three_views();
    // a second corner, and a line between the two in each of the front and top views
    let qf = sk.point(60.0, 40.0, true, "qf");
    let qt = sk.point(60.0, 120.0, true, "qt");
    sk.set_plane(qf, Some(0));
    sk.set_plane(qt, Some(1));
    sk.add(Constraint::project(&sk, EntRef::point(qf), EntRef::point(qt)).unwrap());
    let lf = sk.line(pf, qf);
    let _lt = sk.line(pt, qt);

    let s = scene(&sk, 1.0, 0.6, 0.3);
    let solid: Vec<_> = s.items.iter().filter(|i| i.what == Part::Solid).collect();
    // **one** edge: the front and the top draw the same edge of the object, and the object has
    // it once however many views agree on it
    assert_eq!(solid.len(), 1, "one edge of the object, not one per view that draws it");
    let drawn: Vec<_> = s.items.iter().filter(|i| i.what == Part::Drawn).collect();
    assert_eq!(drawn.len(), 2, "and each line still stands on its own plane");
    assert!(drawn.iter().any(|i| i.of == Some(EntRef::line(lf))), "items carry their entity");
    // the object's edge is 30 long in space, whatever the views measured
    let e = solid[0];
    let (a, b) = (e.pts[0], e.pts[1]);
    assert!((a.0 - b.0).hypot(a.1 - b.1) <= 30.0 + 1e-9, "a projection never lengthens");
}

/// A round kind has no polyline anywhere else in the core; `drawable` gives one, closed, and
/// refined against the same screen flatness a curve is.
#[test]
fn drawable_tessellates_the_round_kinds() {
    let mut sk = Sketch::new();
    let c = sk.point(0.0, 0.0, false, "c");
    let circle = sk.circle(c, 50.0, "c0");
    let fine = drawable(&sk, EntRef::circle(circle), 0.05);
    let coarse = drawable(&sk, EntRef::circle(circle), 2.0);
    assert_eq!(fine.len(), 1);
    assert_eq!(fine[0].first(), fine[0].last(), "a rim is closed");
    assert!(fine[0].len() > coarse[0].len(), "zoomed in, it is refined further");
    for &(x, y) in &fine[0] {
        assert!(near(x.hypot(y), 50.0), "every point is on the rim");
    }
    // and the sagitta rule holds: no chord strays more than the flatness from the rim
    let mid = |a: (f64, f64), b: (f64, f64)| ((a.0 + b.0) / 2.0, (a.1 + b.1) / 2.0);
    for w in fine[0].windows(2) {
        let m = mid(w[0], w[1]);
        assert!(50.0 - m.0.hypot(m.1) < 0.05 * 0.3 + 1e-9);
    }
}

/// The whole worked case: `bracket.sv` folds up, every corner it dimensions is placed, and the
/// inclined face comes out flat in space.
#[test]
fn the_bracket_folds_up() {
    let (prog, errs) = gcs_core::syntax::parse(gcs_core::examples::BRACKET);
    assert!(errs.is_empty(), "{errs:?}");
    let e = gcs_core::program::elaborate(&prog);
    assert!(e.ok());
    let mut sk = e.sketch.clone();
    assert!(gcs_core::solve::solve(&mut sk, Default::default()).success);
    let cs = corners(&sk);
    let of = |n: &str| {
        let i = e.map.ent_named(n).unwrap().i();
        cs.iter().find(|c| c.images.contains(&i)).map(|c| c.at)
    };

    // the front view's corners, reconstructed from the views that see them
    let a = of("Af").expect("A is in three views");
    let b = of("Bf").expect("B is in three views");
    assert!(near((b[0] - a[0]).hypot(b[1] - a[1]), 60.0), "the 60 width, in space");
    // the inclined face: F, E and their far twins are coplanar, and the face is 15 × 30
    let (fa, ea) = (of("Ff").unwrap(), of("Ef").unwrap());
    let rise = ((ea[0] - fa[0]).powi(2) + (ea[1] - fa[1]).powi(2) + (ea[2] - fa[2]).powi(2)).sqrt();
    assert!(near(rise, 15f64.hypot(10.0)), "the incline's true length: {rise}");
    let s = scene(&sk, 0.5, 0.6, 0.3);
    assert!(s.items.iter().any(|i| i.what == Part::Solid), "the object is drawn");
    assert!(s.items.iter().filter(|i| i.what == Part::Face).count() == 4, "four panes");
    assert!(s.bounds.2 > s.bounds.0 && s.bounds.3 > s.bounds.1);
}

/// **Every plane is a pane, drawn in or not** — a view is a place to draw, and one that did not
/// show until something was in it could not be gone to.  Each carries its own axes at its
/// origin, so the box says which way that view's coordinates run.
#[test]
fn every_plane_is_a_pane_with_its_own_axes() {
    let (mut sk, _) = three_views();
    // a fourth view with nothing whatever drawn in it
    let o = sk.point(-80.0, 60.0, true, "aux.o");
    let t = sk.point(-79.0, 61.0, true, "aux.t");
    let aux = sk.plane(o, t, Basis::page().fold(0.7), "aux");

    let unit = 0.5;
    let s = scene(&sk, unit, -FRAC_PI_2, 0.0);
    assert_eq!(s.items.iter().filter(|i| i.what == Part::Face).count(), 4, "a pane per plane");
    for i in 0..4 {
        let of = Some(EntRef::plane(i));
        let arms: Vec<_> =
            s.items.iter().filter(|it| it.what == Part::Axis && it.in_plane == of).collect();
        assert_eq!(arms.len(), 2, "plane {i} has an x and a y");
        assert!(arms[0].pts.len() == 2 && arms[1].pts.len() == 2);
        // each axis runs right across the pane: the projection is affine, so the x arm is
        // exactly as long as the pane's own x side, and the y arm as its y side.  That is the
        // sheet's axes folded up, and not a tick that vanishes into whatever the view drew
        let face = s.items.iter().find(|it| it.what == Part::Face && it.in_plane == of);
        let pts = &face.expect("a pane").pts;
        let len = |a: (f64, f64), b: (f64, f64)| (a.0 - b.0).hypot(a.1 - b.1);
        assert!(near(len(arms[0].pts[0], arms[0].pts[1]), len(pts[0], pts[1])), "x spans the pane");
        assert!(near(len(arms[1].pts[0], arms[1].pts[1]), len(pts[1], pts[2])), "y spans the pane");
        // the empty view's pane is a rectangle like any other, and not nothing
        assert_eq!(pts.len(), 5, "a closed rectangle");
    }
    // the pane contains its own origin, so the two axes cross on it rather than running along
    // an edge: looked at the front view square on, the scene's coordinates are that view's own
    let front = s.items.iter().find(|it| it.what == Part::Face && it.in_plane == Some(EntRef::plane(0)));
    let p = &front.expect("the front pane").pts;
    let (lo, hi) = p.iter().fold(((f64::MAX, f64::MAX), (f64::MIN, f64::MIN)), |(l, h), q| {
        ((l.0.min(q.0), l.1.min(q.1)), (h.0.max(q.0), h.1.max(q.1)))
    });
    assert!(lo.0 <= 0.0 && hi.0 >= 0.0 && lo.1 <= 0.0 && hi.1 >= 0.0, "the origin is on the pane");

    // and every item says which view it belongs to — except the object, which is of them all
    for it in &s.items {
        assert_eq!(it.in_plane.is_none(), it.what == Part::Solid, "{:?}", it.what);
    }
    let _ = aux;
}

/// The plane origins are corners with no projection stated — and a datum's own points are
/// members of no view, so they must be read in the view they place, or `Insert ▸ Three views`
/// (which stamps nothing) would fold up into three panes with no shared corner.
#[test]
fn the_origins_pair_without_being_stamped() {
    let (sk, _) = three_views();   // `datum` stamps neither origin nor toward
    let cs = corners(&sk);
    let origins: Vec<usize> = (0..3).map(|i| sk.planes[i].frame.origin as usize).collect();
    for i in 0..3 {
        for j in i + 1..3 {
            let c = cs.iter().find(|c| c.images == [origins[i], origins[j]]);
            let at = c.unwrap_or_else(|| panic!("origins {i} and {j} pair")).at;
            assert!(at.iter().all(|&x| near(x, 0.0)), "at the one shared origin: {at:?}");
        }
    }
}

/// An edge is found whichever way round its two `project` statements were written: a corner's
/// images are ordered by plane, never by the statement.
#[test]
fn an_edge_is_found_whatever_order_its_projections_were_written() {
    let (mut sk, [pf, pt, _]) = three_views();
    let qf = sk.point(60.0, 40.0, true, "qf");
    let qt = sk.point(60.0, 120.0, true, "qt");
    sk.set_plane(qf, Some(0));
    sk.set_plane(qt, Some(1));
    // the other corner was stated `pf project pt`; this one names the top view first
    sk.add(Constraint::project(&sk, EntRef::point(qt), EntRef::point(qf)).unwrap());
    sk.line(pf, qf);
    sk.line(pt, qt);
    let s = scene(&sk, 1.0, 0.6, 0.3);
    assert_eq!(s.items.iter().filter(|i| i.what == Part::Solid).count(), 1, "the edge P–Q");
}

/// One edge however many views agree on it, and "agree" is to the solve's tolerance: each
/// corner is its own least-squares answer, and an image off by less than the solver can see
/// must not split one edge into two.
#[test]
fn an_edge_three_views_agree_on_is_drawn_once() {
    let (mut sk, [pf, pt, pr]) = three_views();
    // Q = (60, 20, 40): its three images, the front one nudged by less than a solve resolves
    let qf = sk.point(60.0 + 1e-7, 40.0, true, "qf");
    let qt = sk.point(60.0, 120.0, true, "qt");
    let qr = sk.point(170.0, 40.0, true, "qr");
    sk.set_plane(qf, Some(0));
    sk.set_plane(qt, Some(1));
    sk.set_plane(qr, Some(2));
    for (a, b) in [(qf, qt), (qf, qr), (qt, qr)] {
        sk.add(Constraint::project(&sk, EntRef::point(a), EntRef::point(b)).unwrap());
    }
    for (a, b) in [(pf, qf), (pt, qt), (pr, qr)] {
        sk.line(a, b);
    }
    let s = scene(&sk, 1.0, 0.6, 0.3);
    assert_eq!(s.items.iter().filter(|i| i.what == Part::Solid).count(), 1, "one edge, not three");
}

/// A line between two views — a projector — stands with each end where that end is, and belongs
/// to no view; it is not a stroke lifted off the page from the world origin.
#[test]
fn a_line_between_two_views_stands_with_each_end_where_it_is() {
    let (mut sk, [pf, pt, _]) = three_views();
    let qf = sk.point(60.0, 90.0, true, "qf");
    let qt = sk.point(60.0, 120.0, true, "qt");
    sk.set_plane(qf, Some(0));
    sk.set_plane(qt, Some(1));
    let lf = sk.line(pf, qf);
    let lt = sk.line(pt, qt);
    let proj = sk.line(pf, pt);      // front to top
    let s = scene(&sk, 1.0, 0.6, 0.3);
    let drawn = |l: usize| s.items.iter().find(|i| i.of == Some(EntRef::line(l)) && i.what == Part::Drawn);
    let (f, t, x) = (drawn(lf).unwrap(), drawn(lt).unwrap(), drawn(proj).unwrap());
    assert!(x.in_plane.is_none(), "of no one view");
    // its ends are exactly where the front and top views put those points
    assert!(near(x.pts[0].0, f.pts[0].0) && near(x.pts[0].1, f.pts[0].1), "the front end");
    assert!(near(x.pts[1].0, t.pts[0].0) && near(x.pts[1].1, t.pts[0].1), "the top end");
}

/// A view holding only its own origin — `bracket.sv`'s idiom, before its body is written — is a
/// pane like any other, and so is one whose points lie along one line.
#[test]
fn a_view_with_only_its_origin_is_still_a_pane() {
    let mut sk = Sketch::new();
    let o = sk.point(0.0, 0.0, true, "o");
    let t = sk.point(40.0, 0.0, true, "t");
    let front = sk.plane(o, t, Basis::page(), "front");
    sk.set_plane(o, Some(front));
    // and something elsewhere, so the sketch has an extent to size the pane off
    sk.point(200.0, 100.0, true, "far");
    let s = scene(&sk, 1.0, -FRAC_PI_2, 0.0);
    let face = s.items.iter().find(|i| i.what == Part::Face).expect("a pane");
    let side = |a: (f64, f64), b: (f64, f64)| (a.0 - b.0).hypot(a.1 - b.1);
    assert!(side(face.pts[0], face.pts[1]) > 10.0, "wide: {:?}", face.pts);
    assert!(side(face.pts[1], face.pts[2]) > 10.0, "and tall");
    // two points along x only: still a pane with height
    let a = sk.point(10.0, 0.0, true, "a");
    sk.set_plane(a, Some(front));
    let s = scene(&sk, 1.0, -FRAC_PI_2, 0.0);
    let face = s.items.iter().find(|i| i.what == Part::Face).expect("a pane");
    assert!(side(face.pts[1], face.pts[2]) > 10.0, "a line of points is not a strip");
}

/// Two views within a degree of parallel are one view, and a corner seen in them is a ray —
/// not a point flung across the page by a residual the near-singular solve amplified.
#[test]
fn nearly_parallel_views_place_nothing() {
    let mut sk = Sketch::new();
    let o = sk.point(0.0, 0.0, true, "o");
    let t = sk.point(1.0, 0.0, true, "t");
    let front = sk.plane(o, t, Basis::page(), "front");
    let o2 = sk.point(100.0, 0.0, true, "o2");
    let t2 = sk.point(101.0, 0.0, true, "t2");
    let tilted = Basis::explicit([1.0, 0.0, 0.0], [0.0, 0.00001, 1.0]).unwrap();
    let near_front = sk.plane(o2, t2, tilted, "near_front");
    let p = sk.point(30.0, 40.0, true, "p");
    let q = sk.point(130.0, 40.01, true, "q");   // an image off by a hundredth
    sk.set_plane(p, Some(front));
    sk.set_plane(q, Some(near_front));
    let c = Constraint::project(&sk, EntRef::point(p), EntRef::point(q)).unwrap();
    sk.add(c);
    assert!(
        !corners(&sk).iter().any(|c| c.images.contains(&p)),
        "a view a hundred-thousandth of a radian off the front is not a second view",
    );
}
