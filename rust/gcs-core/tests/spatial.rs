//! A plane over two axes and a place in space (`docs/planes-plan.md`), and the hidden points in
//! space a spatial relation reads: what a plane's axes and origin cost in freedoms, that none of
//! it moves a number until a solve moves the plane, and the relations in space over them.
use gcs_core::constraints::CKind;
use gcs_core::model::{EntRef, Sketch};
use gcs_core::plane::Basis;
use gcs_core::space::cross;
use gcs_core::solve::{solve, SolveOpts};
use gcs_core::{diagnose, io};

fn close3(a: [f64; 3], b: [f64; 3], tol: f64) -> bool {
    (0..3).all(|i| (a[i] - b[i]).abs() < tol)
}

fn bits(b: Basis) -> Vec<u64> {
    [b.u, b.v, b.o].concat().iter().map(|x| x.to_bits()).collect()
}

/// The front plane stood `d` toward the viewer (its normal is −y).
fn front(d: f64) -> Basis {
    Basis { o: [0.0, -d, 0.0], ..Basis::page() }
}

/// The top plane (`u = x`, `v = y`, the normal +z) at height `h`.
fn top(h: f64) -> Basis {
    Basis { u: [1.0, 0.0, 0.0], v: [0.0, 1.0, 0.0], o: [0.0, 0.0, h] }
}

/// A plane turned off every axis and stood off the origin, over two free axes, with two points
/// drawn in it.
fn turned() -> (Sketch, usize, [usize; 2]) {
    let mut sk = Sketch::new();
    let u = sk.axis([0.8, 0.3, -0.2], "u");
    let v = sk.axis([-0.1, 0.4, 0.9], "w");
    let pl = sk.plane(u, v, [2.0, -1.0, 3.5], "side");
    let a = sk.point(20.0, 3.0, false, "a");
    let c = sk.point(-4.0, 12.0, false, "c");
    sk.set_plane(a, Some(pl));
    sk.set_plane(c, Some(pl));
    (sk, pl, [a, c])
}

#[test]
fn a_plane_stands_on_its_rays() {
    let (sk, pl, [a, _]) = turned();
    let b = sk.basis(pl);
    let n = |x: [f64; 3]| gcs_core::space::norm(x);
    // right along the first axis, up what is left of the second, out their cross product
    let du = [0.8, 0.3, -0.2].map(|x: f64| x / n([0.8, 0.3, -0.2]));
    assert!(close3(b.u, du, 1e-15));
    assert!(gcs_core::space::dot(b.u, b.v).abs() < 1e-15 && (n(b.v) - 1.0).abs() < 1e-15);
    let w = [-0.1, 0.4, 0.9];
    assert!(gcs_core::space::dot(cross(b.u, b.v), w).abs() < 1e-15, "the second axis in it");
    assert_eq!(b.o, [2.0, -1.0, 3.5]);
    // a point drawn at (20, 3) stands at o + 20u + 3v, and its origin at o
    assert!(close3(sk.world_point(a), b.lift(20.0, 3.0), 1e-12));
    assert!(close3(sk.world_point(sk.planes[pl].origin as usize), b.o, 1e-15));
}

#[test]
fn a_free_plane_counts_its_freedoms() {
    let (mut sk, _, [a, c]) = turned();
    let dof = |sk: &mut Sketch| diagnose::diagnose(sk, Default::default()).dof;
    // two points in the plane, two directions (each on its sphere), and where the plane stands;
    // the second axis's lean along the first is no freedom of the plane, but it is a parameter
    // nothing states
    let stated = dof(&mut sk);
    assert_eq!(stated, 4 + 2 + 2 + 3);
    sk.lift_point(a).unwrap();
    assert_eq!(dof(&mut sk), stated, "a hidden point is held where its plane point lifts");
    sk.lift_point(c).unwrap();
    assert_eq!(dof(&mut sk), stated);
    // a point in space is its own lift: no new parameter and no row
    let before = sk.params.len();
    let s = sk.point3([1.0, 2.0, 3.0], false, "s");
    let k = sk.lift_point(s).unwrap();
    assert_eq!(sk.params.len(), before + 3);
    assert_eq!(sk.lifted(s), [1.0, 2.0, 3.0]);
    assert_eq!(sk.lifts[k].x[2], sk.points[s].z.unwrap());
    assert_eq!(dof(&mut sk), stated + 3);
}

#[test]
fn a_lift_follows_its_plane() {
    let (mut sk, pl, [a, c]) = turned();
    let la = sk.lift_point(a).unwrap();
    assert_eq!(sk.constraints.last().unwrap().kind, CKind::Lift);
    assert!(sk.constraints.last().unwrap().intrinsic);
    assert_eq!(sk.lift_point(a), Some(la), "one hidden point per plane point");
    let at = |sk: &Sketch, k: usize| sk.lifts[k].x.map(|p| sk.params[p as usize].value);
    assert_eq!(at(&sk, la), sk.world_point(a), "seeded at the lift");
    let lc = sk.lift_point(c).unwrap();
    // a point of a 2D sketch has no lift
    let flat = sk.point(1.0, 1.0, false, "flat");
    assert_eq!(sk.lift_point(flat), None);
    // turn the first axis by hand, hold everything drawn, and solve: the hidden points follow
    let u = sk.axes[sk.planes[pl].u as usize].d;
    let turned = [0.2f64, 0.9, -0.3];
    let n = gcs_core::space::norm(turned);
    for k in 0..3 {
        sk.params[u[k] as usize].value = turned[k] / n;
        sk.params[u[k] as usize].fixed = true;
    }
    for p in sk.axes[sk.planes[pl].v as usize].d.into_iter().chain(sk.planes[pl].o) {
        sk.params[p as usize].fixed = true;
    }
    sk.fix_point(a, true);
    sk.fix_point(c, true);
    let r = solve(&mut sk, SolveOpts::default());
    assert!(r.success, "{}", r.message);
    for (k, p) in [(la, a), (lc, c)] {
        let want = sk.world_point(p);
        assert!(close3(at(&sk, k), want, 1e-9), "{:?} {:?}", at(&sk, k), want);
    }
    assert!(close3(sk.basis(pl).u, turned.map(|x| x / n), 1e-15));
}

#[test]
fn a_plane_round_trips_through_json_and_a_copy() {
    let (mut sk, pl, [a, _]) = turned();
    sk.lift_point(a).unwrap();
    let held = sk.fixed_plane(top(1.25), "top");
    let text = io::dumps(&sk, None);
    assert!(!text.contains("AxisUnit") && !text.contains("\"Lift\"") && !text.contains("lift"),
            "an intrinsic row and a hidden point are never stored");
    let back = io::loads(&text).unwrap();
    assert_eq!(io::dumps(&back, None), text);
    for p in [pl, held] {
        assert_eq!(bits(back.basis(p)), bits(sk.basis(p)), "plane {p}");
        assert_eq!(back.plane_fixed(p), sk.plane_fixed(p));
    }
    assert!(back.plane_fixed(held) && !back.plane_fixed(pl));
    assert_eq!(back.constraints.iter().filter(|c| c.kind == CKind::AxisUnit).count(), 4);
    assert!(back.lifts.is_empty(), "re-minted by whatever reads it, not by the reader");
    // a copy carries the plane, its axes and the hidden point
    let copy = io::copy(&sk, &[EntRef::plane(pl), EntRef::plane(held), EntRef::point(a)]);
    for p in [pl, held] {
        assert_eq!(bits(copy.basis(p)), bits(sk.basis(p)));
    }
    assert_eq!(copy.lifts.len(), 1);
}

#[test]
fn a_drag_part_carries_the_plane_and_its_hidden_points() {
    let (mut sk, pl, [a, _]) = turned();
    sk.lift_point(a).unwrap();
    let part = io::Part::around(&sk, EntRef::point(a));
    assert_eq!(part.sketch.planes.len(), 1, "the plane came");
    assert_eq!(part.sketch.lifts.len(), 1);
    assert_eq!(bits(part.sketch.basis(0)), bits(sk.basis(pl)));
    // a move in the part reaches the document's axis
    let mut part = part;
    let u0 = part.sketch.axes[part.sketch.planes[0].u as usize].d[0] as usize;
    part.sketch.params[u0].value = 0.25;
    part.write_back(&mut sk);
    let u = sk.axes[sk.planes[pl].u as usize].d[0] as usize;
    assert_eq!(sk.params[u].value, 0.25);
}

/* -- the relations in space, through the Rust API --------------------------------------------- */

use gcs_core::constraints::{Arg, Constraint};
use gcs_core::diagnose::Diagnosis;

fn drawn(sk: &mut Sketch, v: usize, x: f64, y: f64, name: &str) -> usize {
    let p = sk.point(x, y, false, name);
    sk.set_plane(p, Some(v));
    p
}

fn exact() -> SolveOpts {
    SolveOpts { acceptance_tol: 1e-12, ..SolveOpts::default() }
}

fn ledger(sk: &mut Sketch) -> Diagnosis {
    diagnose::diagnose(sk, Default::default())
}

fn relation(sk: &Sketch, kind: CKind, ents: &[EntRef], value: Option<f64>) -> Constraint {
    Constraint::in_space(sk, kind, ents, value).unwrap_or_else(|e| panic!("{kind:?}: {e}"))
}

/// **A regular tetrahedron.**  The base is an equilateral triangle drawn on the front plane; the
/// apex is drawn in a second plane over two free axes, standing wherever the solve puts it, and
/// stated a true length `a` from each corner.  Nothing says where the apex is in space but those
/// three lengths, and the height that comes back is the closed form, `a·√(2/3)`.
#[test]
fn a_regular_tetrahedron_stands_at_its_height() {
    let a = 30.0;
    let mut sk = Sketch::new();
    let page = sk.fixed_plane(Basis::page(), "page");
    let b0 = drawn(&mut sk, page, 0.0, 0.0, "b0");
    let b1 = drawn(&mut sk, page, a + 2.0, 1.0, "b1");
    let b2 = drawn(&mut sk, page, 0.4 * a, 0.9 * a, "b2");
    sk.fix_point(b0, true);
    for (p, q) in [(b0, b1), (b1, b2), (b2, b0)] {
        sk.add(Constraint::distance(EntRef::point(p), EntRef::point(q), a));
    }
    sk.add(Constraint::new(CKind::HorizontalPoints, vec![
        Arg::Ent(EntRef::point(b0)), Arg::Ent(EntRef::point(b1)),
    ]));
    assert_eq!(ledger(&mut sk).dof, 0, "the base is determined");
    // the apex's plane: two free axes and a free place
    let (u, w) = (sk.axis([1.0, 0.3, 0.2], "u"), sk.axis([0.1, -0.6, 0.8], "w"));
    let slant = sk.plane(u, w, [3.0, -4.0, 5.0], "slant");
    let apex = drawn(&mut sk, slant, 10.0, 8.0, "apex");
    assert_eq!(ledger(&mut sk).dof, 2 + 2 + 2 + 3, "the apex's two, the axes' and the place");
    for b in [b0, b1, b2] {
        sk.add(relation(&sk, CKind::Distance3, &[EntRef::point(apex), EntRef::point(b)], Some(a)));
    }
    assert_eq!(sk.lifts.len(), 4, "the hidden points are minted by the add, one per point");
    let d = ledger(&mut sk);
    // three lengths place a point in space up to its mirror; the six left are the plane's own
    // gauge about the apex — turning it about the point three ways, sliding the point in it, and
    // the second axis's lean along the first
    assert_eq!(d.dof, 6, "{d:?}");
    assert!(d.over.is_empty() && d.implied.is_empty());
    let r = solve(&mut sk, exact());
    assert!(r.success, "{}", r.message);
    // the front plane is y = 0, so the height is the apex's distance from it
    let h = sk.world_point(apex)[1].abs();
    let want = a * (2.0f64 / 3.0).sqrt();
    assert!((h - want).abs() < 1e-9 * want, "height {h} against {want}");
    assert!(close3(sk.lifted(apex), sk.world_point(apex), 1e-9));
    for b in [b0, b1, b2] {
        let d = gcs_core::space::distance(sk.world_point(apex), sk.world_point(b));
        assert!((d - a).abs() < 1e-9 * a);
    }
    for r in [u, w] {
        let d = sk.axes[r].d.map(|k| sk.params[k as usize].value);
        assert!((gcs_core::space::norm(d) - 1.0).abs() < 1e-12, "an axis stays a direction");
    }
}

/// The same tetrahedron with its apex a **point in space**: three numbers, and the three lengths
/// leave nothing but the mirror.
#[test]
fn a_point_in_space_stands_at_its_height() {
    let a = 30.0;
    let mut sk = Sketch::new();
    let page = sk.fixed_plane(Basis::page(), "page");
    let b0 = drawn(&mut sk, page, 0.0, 0.0, "b0");
    let b1 = drawn(&mut sk, page, a, 0.0, "b1");
    let b2 = drawn(&mut sk, page, 0.5 * a, 0.75f64.sqrt() * a, "b2");
    for b in [b0, b1, b2] {
        sk.fix_point(b, true);
    }
    let apex = sk.point3([12.0, -10.0, 9.0], false, "apex");
    for b in [b0, b1, b2] {
        sk.add(relation(&sk, CKind::Distance3, &[EntRef::point(apex), EntRef::point(b)], Some(a)));
    }
    assert_eq!(ledger(&mut sk).dof, 0);
    let r = solve(&mut sk, exact());
    assert!(r.success, "{}", r.message);
    let want = a * (2.0f64 / 3.0).sqrt();
    let p = sk.world_point(apex);
    assert!((-p[1] - want).abs() < 1e-9 * want, "{p:?}: the side the seed stood on");
    assert_eq!(p, sk.lifted(apex), "its own lift");
}

/// Two lines drawn in two stated planes — one on the front plane, one on a top plane `h` above
/// the origin — with the angle between them and their common-perpendicular distance stated.  The
/// answer is worked out by hand: the top line's bearing from the angle, then its offset from the
/// distance.
#[test]
fn two_skew_lines_meet_their_closed_form() {
    let (h, e, theta) = (6.0, 3.0, 60f64.to_radians());
    let mut sk = Sketch::new();
    let page = sk.fixed_plane(Basis::page(), "page");
    let top = sk.fixed_plane(top(h), "top");
    // on the front plane, (0, 0) to (10, 10): in space from the origin along (1, 0, 1)
    let (a, b) = (drawn(&mut sk, page, 0.0, 0.0, "a"), drawn(&mut sk, page, 10.0, 10.0, "b"));
    sk.fix_point(a, true);
    sk.fix_point(b, true);
    // on the top plane (z = h), from x = 5 somewhere along y, ten long
    let c = drawn(&mut sk, top, 5.0, 2.0, "c");
    let phi0 = 40f64.to_radians();
    let d = drawn(&mut sk, top, 5.0 + 10.0 * phi0.cos(), 2.0 + 10.0 * phi0.sin(), "d");
    let cx = sk.points[c].x as usize;
    sk.params[cx].fixed = true;
    let (l1, l2) = (sk.line(a, b), sk.line(c, d));
    let (e1, e2) = (EntRef::line(l1), EntRef::line(l2));
    sk.add(Constraint::distance(EntRef::point(c), EntRef::point(d), 10.0));
    let dist = relation(&sk, CKind::LineLine3, &[e1, e2], Some(e));
    let sign = match dist.args[3] { Arg::Int(s) => s as f64, _ => unreachable!() };
    assert_eq!(sign, 1.0, "the side the seed stands on");
    sk.add(dist);
    sk.add(relation(&sk, CKind::Angle3, &[e1, e2], Some(theta)));
    assert_eq!(ledger(&mut sk).dof, 0);
    let r = solve(&mut sk, exact());
    assert!(r.success, "{}", r.message);
    let ((x0, y0), (x1, y1)) = (sk.point_xy(c), sk.point_xy(d));
    let phi = (y1 - y0).atan2(x1 - x0);
    // cos θ = (1, 0, 1)/√2 · (cos φ, sin φ, 0)
    assert!((phi.cos() - 2f64.sqrt() * theta.cos()).abs() < 1e-10, "{}", phi.to_degrees());
    // s = (e₁ × e₂)·(C − A)/|e₁ × e₂| = (−5 sin φ + y cos φ + h sin φ) / √(1 + sin² φ)
    let (sn, cs) = phi.sin_cos();
    let want = (sign * e * (1.0 + sn * sn).sqrt() + (5.0 - h) * sn) / cs;
    assert!((y0 - want).abs() < 1e-9, "c.y {y0} against {want}");
    // and an independent reading of both off the points in space
    let w = |p: usize| sk.world_point(p);
    let (u1, u2) = (gcs_core::space::sub(w(b), w(a)), gcs_core::space::sub(w(d), w(c)));
    let m = cross(u1, u2);
    let got = gcs_core::space::dot(m, gcs_core::space::sub(w(c), w(a))) / gcs_core::space::norm(m);
    assert!((got - e).abs() < 1e-9);
    let cosv = gcs_core::space::dot(u1, u2)
        / (gcs_core::space::norm(u1) * gcs_core::space::norm(u2));
    assert!((cosv - theta.cos()).abs() < 1e-10);

    // a claim in space is judged and never acts: a true one is a theorem, a false one violated
    let before = (sk.params.len(), sk.n_residuals(), ledger(&mut sk).dof);
    let ac = gcs_core::space::distance(sk.world_point(a), sk.world_point(c));
    let mut yes = relation(&sk, CKind::Distance3, &[EntRef::point(a), EntRef::point(c)], Some(ac));
    yes.claim = true;
    let yes = sk.add(yes);
    let mut no = relation(&sk, CKind::Perpendicular3, &[e1, e2], None);
    no.claim = true;
    let no = sk.add(no);
    let d = ledger(&mut sk);
    assert_eq!((sk.params.len(), sk.n_residuals(), d.dof), before, "a claim adds nothing");
    assert_eq!(d.claims_theorem, vec![yes]);
    assert_eq!(d.claims_violated, vec![no]);
}

/// A skew distance between lines that are parallel as drawn has no common perpendicular, and a
/// point of a 2D sketch has no place in space: both are refused where they are stated.
#[test]
fn a_relation_in_space_refuses_what_it_cannot_read() {
    let mut sk = Sketch::new();
    let page = sk.fixed_plane(Basis::page(), "page");
    let top = sk.fixed_plane(top(4.0), "top");
    let (a, b) = (drawn(&mut sk, page, 0.0, 0.0, "a"), drawn(&mut sk, page, 10.0, 0.0, "b"));
    let (c, d) = (drawn(&mut sk, top, 0.0, 3.0, "c"), drawn(&mut sk, top, 7.0, 3.0, "d"));
    let (l1, l2) = (EntRef::line(sk.line(a, b)), EntRef::line(sk.line(c, d)));
    let e = Constraint::in_space(&sk, CKind::LineLine3, &[l1, l2], Some(2.0)).unwrap_err();
    assert!(e.contains("parallel"), "{e}");
    let loose = sk.point(1.0, 1.0, false, "loose");
    let e = Constraint::in_space(&sk, CKind::Distance3, &[EntRef::point(loose), EntRef::point(a)],
                                 Some(2.0)).unwrap_err();
    assert!(e.contains("2D sketch"), "{e}");
    let e = Constraint::in_space(&sk, CKind::Distance3, &[EntRef::point(c), EntRef::point(a)],
                                 Some(-2.0)).unwrap_err();
    assert!(e.contains("magnitude"), "{e}");
}

/// Two lines in two planes held parallel in space: the front plane's line runs along (2, 0, 1),
/// and a plane parallel to it stood off it can carry a line that way — ten long from a held end,
/// it can only end at `c + 10·(2, 1)/√5`.  Perpendicular from the top plane, whose lines all run
/// level, the second line can only run straight across: `c + (0, 10)`.
#[test]
fn parallel_and_perpendicular_in_space() {
    for kind in [CKind::Parallel3, CKind::Perpendicular3] {
        let mut sk = Sketch::new();
        let page = sk.fixed_plane(Basis::page(), "page");
        let other = match kind {
            CKind::Parallel3 => front(7.0),
            _ => top(5.0),
        };
        let v = sk.fixed_plane(other, "v");
        let (a, b) = (drawn(&mut sk, page, 0.0, 0.0, "a"), drawn(&mut sk, page, 10.0, 5.0, "b"));
        sk.fix_point(a, true);
        sk.fix_point(b, true);
        let c = drawn(&mut sk, v, 3.0, 4.0, "c");
        sk.fix_point(c, true);
        let seed = if kind == CKind::Parallel3 { 20f64 } else { 70f64 }.to_radians();
        let d = drawn(&mut sk, v, 3.0 + 10.0 * seed.cos(), 4.0 + 10.0 * seed.sin(), "d");
        let (l1, l2) = (EntRef::line(sk.line(a, b)), EntRef::line(sk.line(c, d)));
        sk.add(Constraint::distance(EntRef::point(c), EntRef::point(d), 10.0));
        sk.add(relation(&sk, kind, &[l1, l2], None));
        assert_eq!(ledger(&mut sk).dof, 0, "{kind:?}");
        // the interactive acceptance: the perpendicular stops a few ulps short of 1e-12 on the
        // squared length's row, a stall of the step test and not of the geometry
        let r = solve(&mut sk, SolveOpts::default());
        assert!(r.success, "{kind:?}: {}", r.message);
        let want = match kind {
            CKind::Parallel3 => (3.0 + 20.0 / 5f64.sqrt(), 4.0 + 10.0 / 5f64.sqrt()),
            _ => (3.0, 14.0),
        };
        let got = sk.point_xy(d);
        assert!((got.0 - want.0).abs() < 1e-9 && (got.1 - want.1).abs() < 1e-9,
                "{kind:?}: {got:?} against {want:?}");
    }
}

/// A point on a plane, over a stated plane and then over one whose place is solved for.
#[test]
fn a_point_on_a_plane_in_space() {
    let h = 6.0;
    let mut sk = Sketch::new();
    let page = sk.fixed_plane(Basis::page(), "page");
    let top = sk.fixed_plane(top(h), "top");
    // drawn on the front plane at x = 3, so it stands at (3, 0, y): on z = h exactly at y = h
    let p = drawn(&mut sk, page, 3.0, 1.0, "p");
    let px = sk.points[p].x as usize;
    sk.params[px].fixed = true;
    sk.add(relation(&sk, CKind::PointOnPlane, &[EntRef::point(p), EntRef::plane(top)], None));
    assert_eq!(ledger(&mut sk).dof, 0);
    let r = solve(&mut sk, exact());
    assert!(r.success, "{}", r.message);
    assert!((sk.point_xy(p).1 - h).abs() < 1e-10);
    // now hold the point and let the plane's height go, its axes' with it (they pass through its
    // origin): the plane comes to the point
    sk.fix_point(p, true);
    let py = sk.points[p].y as usize;
    sk.params[py].value = 2.5;
    let oz = sk.planes[top].o[2] as usize;
    sk.params[oz].fixed = false;
    for r in [sk.planes[top].u, sk.planes[top].v] {
        let az = sk.axes[r as usize].a[2] as usize;
        sk.params[az].fixed = false;
    }
    assert_eq!(ledger(&mut sk).dof, 0);
    let r = solve(&mut sk, exact());
    assert!(r.success, "{}", r.message);
    assert!((sk.params[oz].value - 2.5).abs() < 1e-10);
    assert!((sk.basis(top).along_normal() - 2.5).abs() < 1e-10);
}

/// A point on a circle drawn in another plane: the circle is in the top plane at height `h`,
/// about its origin with radius `r`; a point drawn on the front plane stands at (x, 0, y), so it
/// can only be at `(r, h)` — on the circle's plane, and `r` out from its centre.
#[test]
fn a_point_on_a_circle_in_space() {
    let (h, rad) = (4.0, 7.5);
    for solved in [false, true] {
        let mut sk = Sketch::new();
        let page = sk.fixed_plane(Basis::page(), "page");
        let top = sk.fixed_plane(top(h), "top");
        if solved {
            // a place the solve may move, held by a statement rather than a `fix`
            let oz = sk.planes[top].o[2] as usize;
            sk.params[oz].fixed = false;
            let origin = EntRef::point(sk.planes[top].origin as usize);
            let held = sk.point3([0.0, 0.0, h], true, "held");
            sk.add(relation(&sk, CKind::Coincident3, &[origin, EntRef::point(held)], None));
        }
        let centre = drawn(&mut sk, top, 0.0, 0.0, "centre");
        sk.fix_point(centre, true);
        let k = sk.circle(centre, rad, "k");
        let rp = sk.circles[k].radius as usize;
        sk.params[rp].fixed = true;
        let p = drawn(&mut sk, page, 6.0, 3.0, "p");
        sk.add(relation(&sk, CKind::PointOnCircle3, &[EntRef::point(p), EntRef::circle(k)], None));
        assert_eq!(ledger(&mut sk).dof, 0, "solved: {solved}");
        let r = solve(&mut sk, exact());
        assert!(r.success, "{}", r.message);
        let got = sk.point_xy(p);
        assert!((got.0 - rad).abs() < 1e-9 && (got.1 - h).abs() < 1e-9, "{got:?}");
    }
}

/// The same point in space, drawn in two planes: on the front plane (x, 0, y) and on the top
/// plane (x′, y′, h), with the front one's x held — so the front point is at `(3, h)` and the top
/// one at `(3, 0)`.
#[test]
fn one_point_in_two_planes_is_coincident_in_space() {
    let h = 5.0;
    let mut sk = Sketch::new();
    let page = sk.fixed_plane(Basis::page(), "page");
    let top = sk.fixed_plane(top(h), "top");
    let p = drawn(&mut sk, page, 3.0, 1.0, "p");
    let q = drawn(&mut sk, top, 2.0, 1.5, "q");
    let px = sk.points[p].x as usize;
    sk.params[px].fixed = true;
    sk.add(relation(&sk, CKind::Coincident3, &[EntRef::point(p), EntRef::point(q)], None));
    assert_eq!(ledger(&mut sk).dof, 0);
    let r = solve(&mut sk, exact());
    assert!(r.success, "{}", r.message);
    let (gp, gq) = (sk.point_xy(p), sk.point_xy(q));
    assert!((gp.0 - 3.0).abs() < 1e-10 && (gp.1 - h).abs() < 1e-10, "{gp:?}");
    assert!((gq.0 - 3.0).abs() < 1e-10 && gq.1.abs() < 1e-10, "{gq:?}");
}

/// A relation in space is a document statement like any other: it writes, reads back and copies
/// with its hidden points re-minted by the add, and a skew distance keeps the side it was read on.
#[test]
fn a_relation_in_space_round_trips() {
    let mut sk = Sketch::new();
    let page = sk.fixed_plane(Basis::page(), "page");
    let top = sk.fixed_plane(top(3.0), "top");
    let (a, b) = (drawn(&mut sk, page, 0.0, 0.0, "a"), drawn(&mut sk, page, 10.0, 10.0, "b"));
    let (c, d) = (drawn(&mut sk, top, 5.0, 9.0, "c"), drawn(&mut sk, top, 8.0, -2.0, "d"));
    let s = sk.point3([1.0, -2.0, 3.0], false, "s");
    let (l1, l2) = (EntRef::line(sk.line(a, b)), EntRef::line(sk.line(c, d)));
    let dist = relation(&sk, CKind::LineLine3, &[l1, l2], Some(2.0));
    let sign = dist.args[3].clone();
    sk.add(dist);
    sk.add(relation(&sk, CKind::Distance3, &[EntRef::point(a), EntRef::point(d)], Some(12.0)));
    sk.add(relation(&sk, CKind::Distance3, &[EntRef::point(s), EntRef::point(c)], Some(4.0)));
    let text = io::dumps(&sk, None);
    let back = io::loads(&text).unwrap();
    assert_eq!(io::dumps(&back, None), text);
    assert_eq!(back.lifts.len(), sk.lifts.len(), "the hidden points are the add's, again");
    assert_eq!(back.points[s].z.map(|z| back.params[z as usize].value), Some(3.0));
    let ll = back.constraints.iter().find(|c| c.kind == CKind::LineLine3).unwrap();
    assert_eq!(ll.args[3], sign);
    let copy = io::copy(&sk, &[l1, l2]);
    assert_eq!(copy.constraints.iter().filter(|c| c.kind == CKind::LineLine3).count(), 1);
    assert_eq!(copy.lifts.len(), 4);
}
