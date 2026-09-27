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
