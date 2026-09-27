//! A view whose attitude is solved rather than stated, and the hidden points in space a spatial
//! relation will read (`docs/spatial-constraints-plan.md`, P1a): the quaternion helpers, the
//! unknowns a freed view mints, what they cost in freedoms, and that none of it moves a number
//! until a solve moves the view.
use gcs_core::constraints::CKind;
use gcs_core::model::{EntRef, Sketch};
use gcs_core::plane::{from_quat, lift_q, quat_matrix, quat_mul, quat_rotate, to_quat, Basis, Quat};
use gcs_core::space::cross;
use gcs_core::rng::Rng;
use gcs_core::solve::{solve, SolveOpts};
use gcs_core::{diagnose, io};

fn close3(a: [f64; 3], b: [f64; 3], tol: f64) -> bool {
    (0..3).all(|i| (a[i] - b[i]).abs() < tol)
}

fn bits(b: Basis) -> Vec<u64> {
    [b.u, b.v, b.o].concat().iter().map(|x| x.to_bits()).collect()
}

/// Bases worth turning into quaternions: the page and its folds (each pivot branch of the
/// conversion is reached among them), and explicit ones at random.
fn bases() -> Vec<Basis> {
    let right = Basis::page().fold(-std::f64::consts::FRAC_PI_2);
    let mut out = vec![Basis::page(), Basis::page().fold(0.0), right];
    for k in 0..12 {
        out.push(Basis::page().fold(0.5 * k as f64 - 3.0).fold(0.3 * k as f64));
    }
    let mut rng = Rng::new(7);
    for _ in 0..40 {
        let r = |rng: &mut Rng| [0; 3].map(|_| rng.uniform(-1.0, 1.0));
        if let Some(b) = Basis::explicit(r(&mut rng), r(&mut rng)) {
            out.push(b);
        }
    }
    out
}

#[test]
fn a_basis_and_its_quaternion_are_one_attitude() {
    for b in bases() {
        let q = to_quat(&b);
        assert!((q.iter().map(|x| x * x).sum::<f64>() - 1.0).abs() < 1e-14, "unit: {q:?}");
        assert!(q[0] >= 0.0, "the canonical sign");
        let back = from_quat(q, b.o).unwrap();
        assert!(close3(back.u, b.u, 1e-14) && close3(back.v, b.v, 1e-14), "{b:?} -> {back:?}");
        // and the rotation carries the page's axes onto the view's, the normal included
        assert!(close3(quat_rotate(q, [0.0, 0.0, 1.0]).unwrap(), b.normal(), 1e-14));
        // only the direction of q is read: a q off the unit sphere names the same view
        let long = q.map(|x| 3.0 * x);
        let b3 = from_quat(long, b.o).unwrap();
        assert!(close3(b3.u, b.u, 1e-14) && close3(b3.v, b.v, 1e-14));
        let gap = to_quat(&back).iter().zip(&q).map(|(a, b)| (a - b).abs()).fold(0.0, f64::max);
        assert!(gap < 1e-14);
    }
    assert!(quat_matrix([0.0; 4]).is_none() && from_quat([0.0; 4], [0.0; 3]).is_none());
}

#[test]
fn a_product_is_the_rotations_composed() {
    let bs = bases();
    for w in bs.windows(2) {
        let (a, b) = (to_quat(&w[0]), to_quat(&w[1]));
        let ab: Quat = quat_mul(a, b);
        for x in [[1.0, 0.0, 0.0], [0.3, -2.0, 0.7]] {
            let two = quat_rotate(a, quat_rotate(b, x).unwrap()).unwrap();
            assert!(close3(quat_rotate(ab, x).unwrap(), two, 1e-13));
        }
    }
}

#[test]
fn the_lift_and_its_derivative() {
    let mut rng = Rng::new(3);
    for _ in 0..20 {
        let q: Quat = [0; 4].map(|_| rng.uniform(-1.0, 1.0));
        let w = [0; 3].map(|_| rng.uniform(-5.0, 5.0));
        let (l, r, dq) = lift_q(q, w).unwrap();
        assert!(close3(l, quat_rotate(q, w).unwrap(), 1e-12));
        // R is a rotation: its columns orthonormal and right-handed
        let col = |j: usize| [r[0][j], r[1][j], r[2][j]];
        assert!(close3(cross(col(0), col(1)), col(2), 1e-12));
        for k in 0..4 {
            let h = 1e-6;
            let (mut qp, mut qm) = (q, q);
            qp[k] += h;
            qm[k] -= h;
            let (lp, lm) = (lift_q(qp, w).unwrap().0, lift_q(qm, w).unwrap().0);
            for i in 0..3 {
                assert!(((lp[i] - lm[i]) / (2.0 * h) - dq[i][k]).abs() < 1e-6);
            }
        }
        // no component along q itself: its length is quat_unit's business
        for i in 0..3 {
            assert!((0..4).map(|k| dq[i][k] * q[k]).sum::<f64>().abs() < 1e-12);
        }
    }
}

/// A view turned and stood off in space, with two points drawn in it.
fn turned() -> (Sketch, usize, [usize; 2]) {
    let mut sk = Sketch::new();
    let o = sk.point(10.0, 5.0, false, "o");
    let t = sk.point(14.0, 8.0, false, "t");
    let b = Basis::page().fold(-0.7).fold(0.4).offset(2.5);
    let b = Basis { o: [b.o[0] + 0.3 * b.u[0], b.o[1] + 0.3 * b.u[1], b.o[2] + 0.3 * b.u[2]], ..b };
    let v = sk.plane(o, t, b, "side");
    let a = sk.point(20.0, 3.0, false, "a");
    let c = sk.point(-4.0, 12.0, false, "c");
    sk.set_plane(a, Some(v));
    sk.set_plane(c, Some(v));
    (sk, v, [a, c])
}

#[test]
fn freeing_a_view_moves_nothing() {
    let (mut sk, v, _) = turned();
    let before = bits(sk.basis(v));
    let values: Vec<u64> = sk.params.iter().map(|p| p.value.to_bits()).collect();
    sk.free_attitude(v);
    let att = sk.planes[v].att.clone().expect("minted");
    assert_eq!(bits(sk.basis(v)), before, "the stored basis, to the bit");
    // the numbers that were there are untouched; five were added, all free
    assert_eq!(sk.params.len(), values.len() + 5);
    for (p, &x) in sk.params.iter().zip(&values) {
        assert_eq!(p.value.to_bits(), x);
    }
    assert!(att.q.iter().chain([&att.d]).all(|&k| !sk.params[k as usize].fixed));
    // one intrinsic row, never the user's
    let c = sk.constraints.last().unwrap();
    assert_eq!(c.kind, CKind::QuatUnit);
    assert!(c.intrinsic && sk.user_constraints().is_empty());
    // what the unknowns say is the stored basis, to the solve's resolution
    let b = sk.basis(v);
    let q = att.q.map(|k| sk.params[k as usize].value);
    let d = sk.params[att.d as usize].value;
    let o = quat_rotate(q, [att.ab[0], att.ab[1], d]).unwrap();
    let r = from_quat(q, o).unwrap();
    assert!(close3(r.u, b.u, 1e-14) && close3(r.v, b.v, 1e-14) && close3(r.o, b.o, 1e-14));
    // freed twice is minted once
    sk.free_attitude(v);
    assert_eq!(sk.constraints.iter().filter(|c| c.kind == CKind::QuatUnit).count(), 1);
    // held and let go, and nothing moves either way
    sk.fix_attitude(v, true);
    assert!(att.q.iter().chain([&att.d]).all(|&k| sk.params[k as usize].fixed));
    assert_eq!(bits(sk.basis(v)), before);
    sk.fix_attitude(v, false);
    assert_eq!(bits(sk.basis(v)), before);
    // a solve of a drawing already solved leaves the view where it stands
    let r = solve(&mut sk, SolveOpts::default());
    assert!(r.success, "{}", r.message);
    let after = sk.basis(v);
    assert!(close3(after.u, b.u, 1e-12) && close3(after.v, b.v, 1e-12));
    assert!(close3(after.o, b.o, 1e-12));
}

#[test]
fn a_free_view_is_four_freedoms_and_a_lift_is_none() {
    let (mut sk, v, [a, c]) = turned();
    let dof = |sk: &mut Sketch| diagnose::diagnose(sk, Default::default()).dof;
    let stated = dof(&mut sk);
    assert_eq!(stated, 8, "the datum's two points and the two drawn in it");
    sk.lift_point(a).unwrap();
    assert_eq!(dof(&mut sk), stated, "a hidden point is held where its view point lifts");
    sk.free_attitude(v);
    assert_eq!(dof(&mut sk), stated + 4, "three turns and one offset");
    sk.lift_point(c).unwrap();
    assert_eq!(dof(&mut sk), stated + 4);
    sk.fix_attitude(v, true);
    assert_eq!(dof(&mut sk), stated, "a held view is a stated one");
}

#[test]
fn a_lift_follows_its_view() {
    let (mut sk, v, [a, c]) = turned();
    // a lift written before the view was freed changes twin with it, keeping its id
    let la = sk.lift_point(a).unwrap();
    let id = sk.constraints.last().unwrap().id;
    assert_eq!(sk.constraints.last().unwrap().kind, CKind::LiftFixed);
    assert_eq!(sk.lift_point(a), Some(la), "one hidden point per view point");
    let at = |sk: &Sketch, k: usize| sk.lifts[k].x.map(|p| sk.params[p as usize].value);
    assert_eq!(at(&sk, la), sk.world_point(a), "seeded at the lift");
    sk.free_attitude(v);
    let lift = sk.constraints.iter().find(|x| x.id == id).unwrap();
    assert_eq!(lift.kind, CKind::Lift);
    let lc = sk.lift_point(c).unwrap();
    assert_eq!(sk.constraints.last().unwrap().kind, CKind::Lift);
    // a point on no view has no lift yet
    let page = sk.point(1.0, 1.0, false, "page");
    assert_eq!(sk.lift_point(page), None);
    // turn the view by hand, hold it and the drawing, and solve: the hidden points follow
    let att = sk.planes[v].att.clone().unwrap();
    let q = to_quat(&sk.basis(v));
    let turn: Quat = [0.9f64.cos(), 0.0, 0.9f64.sin(), 0.0];
    let q2 = quat_mul(turn, q);
    for k in 0..4 {
        sk.params[att.q[k] as usize].value = q2[k];
    }
    sk.params[att.d as usize].value = -4.0;
    sk.fix_attitude(v, true);
    for p in 0..4 {
        sk.fix_point(p, true);
    }
    let r = solve(&mut sk, SolveOpts::default());
    assert!(r.success, "{}", r.message);
    for (k, p) in [(la, a), (lc, c)] {
        let want = sk.world_point(p);
        assert!(close3(at(&sk, k), want, 1e-9), "{:?} {:?}", at(&sk, k), want);
    }
    // and the view it was turned to is what `basis` reads, offset included
    let b = sk.basis(v);
    assert!((b.along_normal() + 4.0).abs() < 1e-12);
    assert!(close3(b.u, quat_rotate(q2, [1.0, 0.0, 0.0]).unwrap(), 1e-15));
    // the stated basis turned the same way lifts the same point
    let pv = gcs_core::plane::in_view(
        sk.params[sk.planes[v].frame.c as usize].value,
        sk.params[sk.planes[v].frame.s as usize].value,
        sk.point_xy(0),
        sk.point_xy(a),
    );
    assert!(close3(b.lift(pv.0, pv.1), sk.world_point(a), 1e-12));
    let _ = EntRef::plane(v);
}

#[test]
fn a_solved_view_round_trips_through_json_and_a_copy() {
    let (mut sk, v, [a, _]) = turned();
    sk.lift_point(a).unwrap();
    sk.free_attitude(v);
    // one view left at rest and one moved: the first reads its stored basis, the second its
    // unknowns, and both must come back to the bit
    let (o2, t2) = (sk.point(0.0, 0.0, false, "o2"), sk.point(3.0, 0.0, false, "t2"));
    let w = sk.plane(o2, t2, Basis::page().fold(0.2), "top");
    sk.free_attitude(w);
    let att = sk.planes[w].att.clone().unwrap();
    for (k, x) in [0.8, 0.1, -0.3, 0.5].iter().enumerate() {
        sk.params[att.q[k] as usize].value = *x;
    }
    sk.params[att.d as usize].value = 1.25;
    sk.params[att.d as usize].fixed = true;
    let text = io::dumps(&sk, None);
    assert!(text.contains("\"att\""));
    assert!(!text.contains("QuatUnit") && !text.contains("Lift") && !text.contains("lift"),
            "an intrinsic row and a hidden point are never stored");
    let back = io::loads(&text).unwrap();
    assert_eq!(io::dumps(&back, None), text);
    for p in [v, w] {
        assert_eq!(bits(back.basis(p)), bits(sk.basis(p)), "plane {p}");
        assert!(back.planes[p].att.is_some());
    }
    let batt = back.planes[w].att.clone().unwrap();
    assert!(back.params[batt.d as usize].fixed && !back.params[batt.q[0] as usize].fixed);
    assert_eq!(back.constraints.iter().filter(|c| c.kind == CKind::QuatUnit).count(), 2);
    assert!(back.lifts.is_empty(), "re-minted by whatever reads it, not by the reader");
    // a copy carries the unknowns and the hidden point, and a stated document writes as it did
    let copy = io::copy(&sk, &[EntRef::plane(v), EntRef::plane(w), EntRef::point(a)]);
    for p in [v, w] {
        assert_eq!(bits(copy.basis(p)), bits(sk.basis(p)));
    }
    assert_eq!(copy.lifts.len(), 1);
    let (plain, _, _) = turned();
    assert!(!io::dumps(&plain, None).contains("att"));
}

#[test]
fn a_drag_part_carries_the_view_and_its_hidden_points() {
    let (mut sk, v, [a, _]) = turned();
    sk.lift_point(a).unwrap();
    sk.free_attitude(v);
    let part = io::Part::around(&sk, EntRef::point(a));
    let pv = part.sketch.planes.iter().position(|p| p.att.is_some()).expect("the view came");
    assert_eq!(part.sketch.lifts.len(), 1);
    assert_eq!(bits(part.sketch.basis(pv)), bits(sk.basis(v)));
    // a move in the part reaches the document's unknowns
    let mut part = part;
    let q0 = part.sketch.planes[pv].att.as_ref().unwrap().q[0] as usize;
    part.sketch.params[q0].value = 0.25;
    part.write_back(&mut sk);
    let att = sk.planes[v].att.clone().unwrap();
    assert_eq!(sk.params[att.q[0] as usize].value, 0.25);
}

/* -- P1b: the relations in space ------------------------------------------------------------- */

use gcs_core::constraints::{Arg, Constraint};
use gcs_core::diagnose::Diagnosis;

/// A view with its datum held at the page's origin, so a point drawn in it at page `(x, y)` is at
/// view coordinates `(x, y)` and stands in space at `basis.lift(x, y)`.
fn view(sk: &mut Sketch, b: Basis, name: &str) -> usize {
    let o = sk.point(0.0, 0.0, true, &format!("{name}.o"));
    let t = sk.point(1.0, 0.0, true, &format!("{name}.t"));
    sk.plane(o, t, b, name)
}

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

/// **A regular tetrahedron.**  The base is an equilateral triangle drawn on the page; the apex is
/// drawn in a second view whose attitude is left to the solve, and stated a true length `a` from
/// each corner.  Nothing says where the apex is in space but those three lengths, and the height
/// that comes back is the closed form, `a·√(2/3)`.
#[test]
fn a_regular_tetrahedron_stands_at_its_height() {
    let a = 30.0;
    let mut sk = Sketch::new();
    let page = view(&mut sk, Basis::page(), "page");
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
    // the apex's view: folded off the page and then let go, so the solve may turn and stand it
    // off wherever the three lengths want the apex
    let slant = view(&mut sk, Basis::page().fold(0.9), "slant");
    let apex = drawn(&mut sk, slant, 10.0, 8.0, "apex");
    assert_eq!(ledger(&mut sk).dof, 2, "the base is determined; the apex has its two in the view");
    sk.free_attitude(slant);
    assert_eq!(ledger(&mut sk).dof, 6, "and its view three turns and an offset");
    for b in [b0, b1, b2] {
        sk.add(relation(&sk, CKind::Distance3, &[EntRef::point(apex), EntRef::point(b)], Some(a)));
    }
    assert_eq!(sk.lifts.len(), 4, "the hidden points are minted by the add, one per point");
    let d = ledger(&mut sk);
    // three lengths place a point in space up to its mirror; the three left are the view's own
    // gauge about the apex — turning the view about the point, and sliding the point in it
    assert_eq!(d.dof, 3, "{d:?}");
    assert!(d.over.is_empty() && d.implied.is_empty());
    let r = solve(&mut sk, exact());
    assert!(r.success, "{}", r.message);
    // the page is the plane y = 0, so the height is the apex's distance from it
    let h = sk.world_point(apex)[1].abs();
    let want = a * (2.0f64 / 3.0).sqrt();
    assert!((h - want).abs() < 1e-9 * want, "height {h} against {want}");
    // and the hidden point is the drawn one, stood up by the view as it was solved
    assert!(close3(sk.lifted(apex), sk.world_point(apex), 1e-9));
    for b in [b0, b1, b2] {
        let d = gcs_core::space::distance(sk.world_point(apex), sk.world_point(b));
        assert!((d - a).abs() < 1e-9 * a);
    }
    let att = sk.planes[slant].att.clone().unwrap();
    let q = att.q.map(|k| sk.params[k as usize].value);
    assert!((q.iter().map(|x| x * x).sum::<f64>() - 1.0).abs() < 1e-12);
}

/// Two lines drawn in two stated views — one on the page, one on a top view stood `h` above the
/// origin — with the angle between them and their common-perpendicular distance stated.  The
/// answer is worked out by hand: the top line's bearing from the angle, then its offset from the
/// distance.
#[test]
fn two_skew_lines_meet_their_closed_form() {
    let (h, e, theta) = (6.0, 3.0, 60f64.to_radians());
    let mut sk = Sketch::new();
    let page = view(&mut sk, Basis::page(), "page");
    let top = view(&mut sk, Basis::page().fold(0.0).offset(h), "top");
    // on the page, (0, 0) to (10, 10): in space from the origin along (1, 0, 1)
    let (a, b) = (drawn(&mut sk, page, 0.0, 0.0, "a"), drawn(&mut sk, page, 10.0, 10.0, "b"));
    sk.fix_point(a, true);
    sk.fix_point(b, true);
    // on the top view (the plane z = h), from x = 5 somewhere along y, ten long
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
/// point on no view has no place in space: both are refused where they are stated.
#[test]
fn a_relation_in_space_refuses_what_it_cannot_read() {
    let mut sk = Sketch::new();
    let page = view(&mut sk, Basis::page(), "page");
    let top = view(&mut sk, Basis::page().fold(0.0).offset(4.0), "top");
    let (a, b) = (drawn(&mut sk, page, 0.0, 0.0, "a"), drawn(&mut sk, page, 10.0, 0.0, "b"));
    let (c, d) = (drawn(&mut sk, top, 0.0, 3.0, "c"), drawn(&mut sk, top, 7.0, 3.0, "d"));
    let (l1, l2) = (EntRef::line(sk.line(a, b)), EntRef::line(sk.line(c, d)));
    let e = Constraint::in_space(&sk, CKind::LineLine3, &[l1, l2], Some(2.0)).unwrap_err();
    assert!(e.contains("parallel"), "{e}");
    let loose = sk.point(1.0, 1.0, false, "loose");
    let e = Constraint::in_space(&sk, CKind::Distance3, &[EntRef::point(loose), EntRef::point(a)],
                                 Some(2.0)).unwrap_err();
    assert!(e.contains("no view"), "{e}");
    let e = Constraint::in_space(&sk, CKind::Distance3, &[EntRef::point(c), EntRef::point(a)],
                                 Some(-2.0)).unwrap_err();
    assert!(e.contains("magnitude"), "{e}");
}

/// Two lines in two views held parallel in space: the page's line runs along (2, 0, 1), and a
/// view parallel to the page stood off it can carry a line that way — ten long from a held end,
/// it can only end at `c + 10·(2, 1)/√5`.  Perpendicular from the top view, whose lines all run
/// level, the second line can only run straight across: `c + (0, 10)`.
#[test]
fn parallel_and_perpendicular_in_space() {
    for kind in [CKind::Parallel3, CKind::Perpendicular3] {
        let mut sk = Sketch::new();
        let page = view(&mut sk, Basis::page(), "page");
        let other = match kind {
            CKind::Parallel3 => Basis::page().offset(7.0),
            _ => Basis::page().fold(0.0).offset(5.0),
        };
        let v = view(&mut sk, other, "v");
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

/// A point on a plane, over a stated plane and then over a solved one; and the statement changes
/// twin with its plane, keeping its id.
#[test]
fn a_point_on_a_plane_in_space() {
    let h = 6.0;
    let mut sk = Sketch::new();
    let page = view(&mut sk, Basis::page(), "page");
    let top = view(&mut sk, Basis::page().fold(0.0).offset(h), "top");
    // drawn on the page at x = 3, so it stands at (3, 0, y): on the plane z = h exactly at y = h
    let p = drawn(&mut sk, page, 3.0, 1.0, "p");
    let px = sk.points[p].x as usize;
    sk.params[px].fixed = true;
    let id = sk.add(relation(&sk, CKind::PointOnPlane, &[EntRef::point(p), EntRef::plane(top)],
                             None));
    let kind = |sk: &Sketch| sk.constraint(id).unwrap().kind;
    assert_eq!(kind(&sk), CKind::PointOnPlaneFixed, "a stated plane's twin");
    assert_eq!(ledger(&mut sk).dof, 0);
    let r = solve(&mut sk, exact());
    assert!(r.success, "{}", r.message);
    assert!((sk.point_xy(p).1 - h).abs() < 1e-10);
    // now hold the point and let the plane's offset go: the plane comes to the point
    sk.fix_point(p, true);
    let py = sk.points[p].y as usize;
    sk.params[py].value = 2.5;
    sk.free_attitude(top);
    assert_eq!(kind(&sk), CKind::PointOnPlane, "and the solved one's, the same statement");
    sk.fix_attitude(top, true);
    let att = sk.planes[top].att.clone().unwrap();
    sk.params[att.d as usize].fixed = false;
    assert_eq!(ledger(&mut sk).dof, 0);
    let r = solve(&mut sk, exact());
    assert!(r.success, "{}", r.message);
    assert!((sk.params[att.d as usize].value - 2.5).abs() < 1e-10);
    assert!((sk.basis(top).along_normal() - 2.5).abs() < 1e-10);
}

/// A point on a circle drawn in another view: the circle is in the top view at height `h`, about
/// the origin with radius `r`; a point drawn on the page stands at (x, 0, y), so it can only be
/// at `(r, h)` — on the circle's plane, and `r` out from its centre.
#[test]
fn a_point_on_a_circle_in_space() {
    let (h, rad) = (4.0, 7.5);
    for solved in [false, true] {
        let mut sk = Sketch::new();
        let page = view(&mut sk, Basis::page(), "page");
        let top = view(&mut sk, Basis::page().fold(0.0).offset(h), "top");
        let centre = drawn(&mut sk, top, 0.0, 0.0, "centre");
        sk.fix_point(centre, true);
        let k = sk.circle(centre, rad, "k");
        let rp = sk.circles[k].radius as usize;
        sk.params[rp].fixed = true;
        if solved {
            sk.free_attitude(top);
            sk.fix_attitude(top, true);
        }
        let p = drawn(&mut sk, page, 6.0, 3.0, "p");
        let id = sk.add(relation(&sk, CKind::PointOnCircle3, &[EntRef::point(p), EntRef::circle(k)],
                                 None));
        let want = if solved { CKind::PointOnCircle3 } else { CKind::PointOnCircle3Fixed };
        assert_eq!(sk.constraint(id).unwrap().kind, want);
        assert_eq!(ledger(&mut sk).dof, 0, "solved: {solved}");
        let r = solve(&mut sk, exact());
        assert!(r.success, "{}", r.message);
        let got = sk.point_xy(p);
        assert!((got.0 - rad).abs() < 1e-9 && (got.1 - h).abs() < 1e-9, "{got:?}");
    }
}

/// The same point in space, drawn in two views: on the page (x, 0, y) and on the top view
/// (x′, y′, h), with the page's x held — so the page point is at `(3, h)` and the top one at
/// `(3, 0)`.
#[test]
fn one_point_in_two_views_is_coincident_in_space() {
    let h = 5.0;
    let mut sk = Sketch::new();
    let page = view(&mut sk, Basis::page(), "page");
    let top = view(&mut sk, Basis::page().fold(0.0).offset(h), "top");
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
    let page = view(&mut sk, Basis::page(), "page");
    let top = view(&mut sk, Basis::page().fold(0.0).offset(3.0), "top");
    let (a, b) = (drawn(&mut sk, page, 0.0, 0.0, "a"), drawn(&mut sk, page, 10.0, 10.0, "b"));
    let (c, d) = (drawn(&mut sk, top, 5.0, 9.0, "c"), drawn(&mut sk, top, 8.0, -2.0, "d"));
    let (l1, l2) = (EntRef::line(sk.line(a, b)), EntRef::line(sk.line(c, d)));
    let dist = relation(&sk, CKind::LineLine3, &[l1, l2], Some(2.0));
    let sign = dist.args[3].clone();
    sk.add(dist);
    sk.add(relation(&sk, CKind::Distance3, &[EntRef::point(a), EntRef::point(d)], Some(12.0)));
    let text = io::dumps(&sk, None);
    let back = io::loads(&text).unwrap();
    assert_eq!(io::dumps(&back, None), text);
    assert_eq!(back.lifts.len(), sk.lifts.len(), "the hidden points are the add's, again");
    let ll = back.constraints.iter().find(|c| c.kind == CKind::LineLine3).unwrap();
    assert_eq!(ll.args[3], sign);
    let copy = io::copy(&sk, &[l1, l2]);
    assert_eq!(copy.constraints.iter().filter(|c| c.kind == CKind::LineLine3).count(), 1);
    assert_eq!(copy.lifts.len(), 4);
}
