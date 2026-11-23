//! **A sheet's inverse of a point on it is that point's parameters, and a net's offset at a pole
//! is the offset of its pole** (docs/contract/brep.md, `geom::net_inverse`, `Surface::Offset`;
//! #993 stage 1): a rational net has no index of its spans, and its foot was the first Newton's
//! method settled from a pole's Greville point — a foot on the domain's edge among them, a whole
//! turn of a turned skew line read 0.3 mm off a point lying on it. Held on rational nets of whole
//! turns — a skew line's (a hyperboloid), a sphere's meridian's (two rows of poles) — at points
//! spread over them, by the seam and the poles, read cold; and a ball's net offset is the ball.
use gcs_core::brep::geom::{Curve, Frame, Surface, V};
use gcs_core::brep::nurbs::{BSpline, Net};
use gcs_core::brep::step_read::geometry::turned;
use gcs_core::space::{distance, norm};
use std::f64::consts::{FRAC_1_SQRT_2, PI, TAU};
use std::sync::Arc;

/// A curve turned once about the world's `z` as its rational net.
fn turned_net(c: BSpline) -> Arc<Net> { Arc::new(turned(&Frame::WORLD, &Curve::BSpline(Arc::new(c)), [0., TAU], 0.).unwrap()) }

/// A skew line, `(3, 0, 0) + t (0, 1, 2)` over `t ∈ [−4, 4]`, turned: a hyperboloid of one sheet.
fn hyperboloid() -> Arc<Net> {
    turned_net(BSpline::new(1, vec![0., 0., 1., 1.], vec![[3., -4., -8.], [3., 4., 8.]]).unwrap())
}

/// The half circle of radius `r` from the south pole to the north, two rational quarters, turned:
/// a ball's net, its first and last rows of poles each one point.
fn ball(r: f64) -> Arc<Net> {
    let poles = vec![[0., 0., -r], [r, 0., -r], [r, 0., 0.], [r, 0., r], [0., 0., r]];
    turned_net(BSpline::rational(2, vec![0., 0., 0., 0.5, 0.5, 1., 1., 1.], poles, vec![1., FRAC_1_SQRT_2, 1., FRAC_1_SQRT_2, 1.]).unwrap())
}

/// The parameters points are read at: a grid across the domain, its ends and the seam's sides a
/// hair in, by the poles too.
fn spread(n: &Net) -> Vec<[f64; 2]> {
    let [[u0, u1], [v0, v1]] = n.domain();
    let at = |a: f64, z: f64| -> Vec<f64> {
        let w = z - a;
        let mut out: Vec<f64> = (0..=12).map(|k| a + w * k as f64 / 12.).collect();
        out.extend([1e-9, 1e-6, 1e-3, 0.0371, 0.5137, 0.9629, 1. - 1e-3, 1. - 1e-6, 1. - 1e-9].map(|f| a + w * f));
        out
    };
    at(u0, u1).iter().flat_map(|&u| at(v0, v1).into_iter().map(move |v| [u, v])).collect()
}

/// **A point on a rational net is read back to itself**: `Surface::inverse` of each point spread
/// over the sheet, read in an order that leaves no warm start near it, is a foot within 1e-9 of it.
#[test]
fn a_point_on_a_rational_net_is_read_back_to_itself() {
    for (name, n) in [("hyperboloid", hyperboloid()), ("ball", ball(5.))] {
        let s = Surface::BSpline(Frame::WORLD, n.clone());
        let uvs = spread(&n);
        let mut worst = (0f64, [0.; 2]);
        // each point read after one far across the sheet, so the last foot is no start near it
        for (k, &uv) in uvs.iter().enumerate() {
            let far = uvs[(k + uvs.len() / 2) % uvs.len()];
            let _ = s.inverse(n.point(far[0], far[1]));
            let p = n.point(uv[0], uv[1]);
            let back = s.inverse(p);
            let off = distance(n.point(back[0], back[1]), p);
            if off > worst.0 { worst = (off, uv); }
        }
        eprintln!("{name}: {} points, the farthest read {:e} off at {:?}", uvs.len(), worst.0, worst.1);
        assert!(worst.0 <= 1e-9, "{name}: a point at {:?} on it read {:e} off", worst.1, worst.0);
    }
}

/// **A ball's net offset is the ball of the radius grown**: `S + d N` at every point of it, its
/// poles too — where the normal is the limit of the normals along the rows approaching it — stands
/// `r + d` from the centre, along the ball's own normal there.
#[test]
fn a_balls_net_offset_is_a_ball() {
    let (r, d) = (5., 1.5);
    let n = ball(r);
    let s = Surface::Offset(Frame::WORLD, n.clone(), d);
    let [[u0, u1], [v0, v1]] = n.domain();
    let mut worst = 0f64;
    for i in 0..=16 {
        for j in 0..=16 {
            let uv = [u0 + (u1 - u0) * i as f64 / 16., v0 + (v1 - v0) * j as f64 / 16.];
            let q = s.point(uv);
            let p = n.point(uv[0], uv[1]);
            let out: V = std::array::from_fn(|k| p[k] * (r + d) / r);
            worst = worst.max(distance(q, out));
            assert!((norm(q) - (r + d)).abs() <= 1e-9, "{uv:?}: {q:?} stands {} from the centre", norm(q));
        }
    }
    assert!(worst <= 1e-9, "an offset point {worst:e} off the ball's");
    // the poles themselves: the south pole grown to (0, 0, −(r + d)), the north to (0, 0, r + d)
    for (v, z) in [(v0, -(r + d)), (v1, r + d)] {
        for u in [u0, 0.3 * PI, u1] {
            let (q, _, qv) = s.d1([u, v]);
            assert!(distance(q, [0., 0., z]) <= 1e-9, "the pole at v = {v}, u = {u}: {q:?}");
            // its rate along the meridian the ball's grown with it (carried to the pole, O(h²))
            let (_, _, pv) = n.d1(u, v);
            let grown: V = std::array::from_fn(|k| pv[k] * (r + d) / r);
            assert!(distance(qv, grown) <= 1e-6 * norm(grown), "the pole's rate at u = {u}: {qv:?}, the ball's grown {grown:?}");
        }
    }
}

/// **A point's foot on a sheet, sought near a hint, is the same whatever was sought before**
/// (`Surface::inverse_near`; benchy's face #27487, #993 stage 1). A sheet folded back on itself, its
/// two layers 2e-4 apart — as a fitted sheet's crumpled corner comes back within 1e-4 of its own
/// point — and a point lying on the lower layer, sought from a hint on the upper: Newton's method
/// settles on the upper, 2e-4 off, and the whole sheet is searched. That search started from the
/// last foot the thread found and kept it where it lay within a thousandth of the point's size of
/// the sheet, so the point read 2e-4 off on the upper layer after a point there was asked and on
/// itself otherwise: benchy's corner vertex, read on itself by one edge's walk and 9.5e-5 off by
/// the next's, broke its loop in parameters. It reads on itself both ways.
#[test]
fn a_points_foot_near_a_hint_does_not_depend_on_what_was_asked_before() {
    // a U turn in `v` (x out to 2 and back at a height of 2e-4), carried along `y` in `u`
    let h = 2e-4;
    let row = |y: f64| vec![[0., y, 0.], [2., y, 0.], [2.5, y, 0.5 * h], [2., y, h], [0., y, h]];
    let n = Arc::new(Net {du: 1, dv: 2, uknots: vec![0., 0., 1., 1.], vknots: vec![0., 0., 0., 0.4, 0.6, 1., 1., 1.],
        poles: vec![row(0.), row(1.)], weights: None});
    n.check().unwrap();
    let s = Surface::BSpline(Frame::WORLD, n.clone());
    // the point on the lower layer, the hint above it on the upper
    let (below, above) = ([0.5, 0.1], [0.5, 0.9]);
    let (p, q) = (n.point(below[0], below[1]), n.point(above[0], above[1]));
    assert!(distance(p, q) > 0.5 * h && distance(p, q) < 1.5 * h, "the layers stand {:e} apart", distance(p, q));
    // sought after the upper layer's own point, its foot the thread's last, and again after the
    // lower's
    let up = n.point(0.5, 0.92);
    let read = s.inverse(up);
    assert!(distance(n.point(read[0], read[1]), up) <= 1e-12, "the upper layer's point read off it");
    let warm = s.inverse_near(p, above);
    let _ = s.inverse(n.point(0.5, 0.08));
    let after = s.inverse_near(p, above);
    for (when, uv) in [("after a point on the upper layer", warm), ("after a point on the lower", after)] {
        let off = distance(n.point(uv[0], uv[1]), p);
        assert!(off <= 1e-12, "{when}: the point read {off:e} off, at {uv:?}");
    }
}
