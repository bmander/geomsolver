//! Stage 5 torture suite: recorded drag trajectories must keep constraints satisfied, move
//! continuously (no solution jumps), keep/flag chirality, and branches must survive save/load.
use gcs_core::constraints::{CKind, Constraint};
use gcs_core::decompose::{PlanDrag, PlanSolver};
use gcs_core::examples;
use gcs_core::io;
use gcs_core::model::{EntRef, Sketch};
use gcs_core::newton::Method;
use gcs_core::solve::{solve, Drag, RadiusDrag, SolveOpts};
use gcs_core::system::System;

fn circle_path(cx: f64, cy: f64, r: f64, n: usize) -> Vec<(f64, f64)> {
    (0..=n)
        .map(|i| {
            let t = 2.0 * std::f64::consts::PI * i as f64 / n as f64;
            (cx + r * t.cos(), cy + r * t.sin())
        })
        .collect()
}

struct TrajectoryInfo {
    flips: usize,
    on_plan: bool,
}

/// Drag `p` along `path`; assert constraints hold after every frame and nothing teleports.
fn run_trajectory(
    sk: &mut Sketch,
    p: usize,
    path: &[(f64, f64)],
    jump_factor: f64,
) -> TrajectoryInfo {
    let mut sys = System::new(sk);
    let (x0, y0) = sk.point_xy(p);
    let mut drag = PlanDrag::new(sk, p, x0, y0, None, 0.05);
    let mut prev = sk.get_x();
    for &(x, y) in path {
        let (px, py) = sk.point_xy(p);
        let cursor_step = (x - px).hypot(y - py);
        let res = drag.move_to(sk, x, y);
        assert!(res.success, "{res:?}");
        sys.refresh_consts(sk);
        let z = sys.z0(sk);
        assert!(sys.max_hard_residual(&z) <= 1e-6 * sys.scale);
        let now = sk.get_x();
        let moved =
            now.iter().zip(&prev).map(|(a, b)| (a - b).abs()).fold(0.0f64, f64::max);
        if cursor_step > 1e-9 {
            assert!(
                moved <= jump_factor * cursor_step + 1e-9,
                "jump: moved {moved:.3e} for cursor step {cursor_step:.3e}"
            );
        }
        prev = now;
    }
    let info = TrajectoryInfo { flips: drag.flips().len(), on_plan: drag.usable() };
    drag.end(sk);
    info
}

#[test]
fn a_floating_truss_rides_along_with_the_cursor() {
    let mut sk = examples::truss_floating(6);
    let p = 3;
    let (x, y) = sk.point_xy(p);
    let info = run_trajectory(&mut sk, p, &circle_path(x, y, 15.0, 40), 8.0);
    assert!(info.on_plan && info.flips == 0);
}

#[test]
fn an_under_constrained_rect_slides() {
    let mut sk = examples::rect_fillets_under();
    let p = sk.lines[0].p2 as usize;
    let (x0, y0) = sk.point_xy(p);
    let mut path: Vec<(f64, f64)> =
        (0..20).map(|i| (x0 + 40.0 * i as f64 / 19.0, y0)).collect();
    path.extend((0..30).map(|i| (x0 + 40.0 - 60.0 * i as f64 / 29.0, y0)));
    let info = run_trajectory(&mut sk, p, &path, 10.0);
    assert_eq!(info.flips, 0);
    assert!((sk.point_xy(p).0 - (x0 - 20.0)).abs() < 1e-6);
}

#[test]
fn a_fully_constrained_point_stays_put_without_jumping() {
    let mut sk = examples::rect_fillets(100.0, 60.0, 10.0, 0.0);
    let p = sk.lines[0].p2 as usize;
    let x0 = sk.get_x();
    let (x, y) = sk.point_xy(p);
    run_trajectory(&mut sk, p, &circle_path(x, y, 20.0, 40), 10.0);
    for (a, b) in sk.get_x().iter().zip(&x0) {
        assert!((a - b).abs() < 1e-6);
    }
}

#[test]
fn a_polygon_vertex_far_drag_is_continuous() {
    let mut sk = examples::polygon_chain(10, 50.0);
    let p = 5;
    let (x, y) = sk.point_xy(p);
    let path: Vec<(f64, f64)> = (0..12)
        .map(|i| {
            let t = i as f64 / 11.0;
            (x + 60.0 * t, y + 30.0 * t)
        })
        .collect();
    let info = run_trajectory(&mut sk, p, &path, 6.0);
    // EqualLength is not decomposable: the numeric path with continuation takes over
    assert!(!info.on_plan);
}

#[test]
fn a_fully_constrained_apex_never_jumps_across_the_base() {
    let mut sk = Sketch::new();
    let a = sk.point(0.0, 0.0, true, "a");
    let b = sk.point(10.0, 0.0, true, "b");
    let c = sk.point(5.0, 4.0, false, "c");
    sk.add(Constraint::distance(EntRef::point(a), EntRef::point(c), 6.0));
    sk.add(Constraint::distance(EntRef::point(b), EntRef::point(c), 6.0));
    let (x, y) = sk.point_xy(c);
    let mut d = PlanDrag::new(&mut sk, c, x, y, None, 0.05);
    // pinning the apex over-determines the sketch: the numeric path with guards takes over
    assert!(d.numeric.is_some());
    assert!(!d.numeric.as_ref().unwrap().guards.is_empty());
    let mut ys = Vec::new();
    for i in 0..17 {
        let yy = 4.0 - 8.0 * i as f64 / 16.0;
        d.move_to(&mut sk, 5.0, yy);
        ys.push(sk.point_xy(c).1);
    }
    d.end(&mut sk);
    assert!(d.flips().is_empty());
    assert!(ys.iter().cloned().fold(f64::INFINITY, f64::min) > 3.0);
    assert!(ys.iter().cloned().fold(f64::NEG_INFINITY, f64::max) < 3.5);
}

#[test]
fn a_guard_flags_an_unavoidable_crossing() {
    let mut sk = Sketch::new();
    let a = sk.point(0.0, 0.0, true, "a");
    let b = sk.point(10.0, 0.0, true, "b");
    let c = sk.point(5.0, 4.0, false, "c");
    sk.add(Constraint::distance(EntRef::point(a), EntRef::point(c), 6.0));
    let (x, y) = sk.point_xy(c);
    let mut d = Drag::new(&mut sk, c, x, y, Method::DogLeg, 1.0, vec![(a, b, c)], 0.05);
    let mut msgs = Vec::new();
    for i in 0..9 {
        let yy = 4.0 - 8.0 * i as f64 / 8.0;
        msgs.push(d.move_to(&mut sk, 5.0, yy).message);
    }
    d.end(&mut sk);
    assert_eq!(d.flips, vec![(a, b, c)]);
    assert!(msgs.iter().any(|m| m.contains("flip")));
    assert!(sk.point_xy(c).1 < 0.0);
}

#[test]
fn branches_survive_save_load_and_replay_sticky() {
    let mut sk = Sketch::new();
    let a = sk.point(0.0, 0.0, true, "a");
    let b = sk.point(10.0, 0.0, true, "b");
    let c = sk.point(5.0, 4.0, false, "c");
    sk.add(Constraint::distance(EntRef::point(a), EntRef::point(c), 6.0));
    sk.add(Constraint::distance(EntRef::point(b), EntRef::point(c), 6.0));
    let mut ps = PlanSolver::new(&sk, true);
    ps.solve(&mut sk, 1e-9, true, Method::DogLeg);
    let el = gcs_core::cgraph::El::p(ps.plan.graph.point_of[c]);
    ps.flip(&mut sk, el);
    ps.solve(&mut sk, 1e-9, true, Method::DogLeg);
    assert!(sk.point_xy(c).1 < 0.0);
    let mut sk2 = io::loads(&io::dumps(&sk, Some(1))).unwrap();
    assert_eq!(sk2.branches, sk.branches);
    let cy = sk2.points[2].y as usize;
    sk2.params[cy].value = 4.0; // sketch moved to the other side...
    PlanSolver::new(&sk2, true).solve(&mut sk2, 1e-9, true, Method::DogLeg);
    assert!(sk2.point_xy(2).1 < 0.0); // ...the recorded root wins
}

#[test]
fn continuation_subdivides_large_moves() {
    let mut sk = examples::truss_floating(4);
    let p = 2;
    let (x, y) = sk.point_xy(p);
    let mut d = PlanDrag::new(&mut sk, p, x, y, None, 0.05);
    let res = d.move_to(&mut sk, x + 200.0, y); // far beyond one increment
    d.end(&mut sk);
    assert!(res.success && res.nfev > 1);
}

#[test]
fn radius_drag_resizes_a_free_circle() {
    let mut sk = Sketch::new();
    let o = sk.point(0.0, 0.0, true, "o");
    let c = sk.circle(o, 10.0, "c");
    let mut d = RadiusDrag::new(&mut sk, EntRef::circle(c), 10.0, Method::DogLeg);
    for target in [25.0, 4.0, 12.5] {
        let res = d.move_to(&mut sk, target);
        assert!(res.success);
        assert!((sk.radius_value(EntRef::circle(c)) - target).abs() < 1e-6);
    }
    d.end(&mut sk);
    assert!(!sk.constraints.iter().any(|c| c.soft));
}

#[test]
fn radius_drag_leaves_a_dimensioned_circle_alone() {
    let mut sk = Sketch::new();
    let o = sk.point(0.0, 0.0, true, "o");
    let c = sk.circle(o, 10.0, "c");
    sk.add(Constraint::radius(EntRef::circle(c), 10.0));
    let mut d = RadiusDrag::new(&mut sk, EntRef::circle(c), 10.0, Method::DogLeg);
    d.move_to(&mut sk, 30.0);
    d.end(&mut sk);
    assert!((sk.radius_value(EntRef::circle(c)) - 10.0).abs() < 1e-6);
}

#[test]
fn radius_drag_carries_the_geometry_that_depends_on_it() {
    let mut sk = Sketch::new();
    let o = sk.point(0.0, 0.0, true, "o");
    let s = sk.point(10.0, 0.0, false, "s");
    let e = sk.point(0.0, 10.0, false, "e");
    let arc = sk.arc(o, s, e, "a");
    solve(&mut sk, SolveOpts::default());
    let mut d = RadiusDrag::new(&mut sk, EntRef::arc(arc), 10.0, Method::DogLeg);
    assert!(d.move_to(&mut sk, 17.0).success);
    d.end(&mut sk);
    assert!((sk.radius_value(EntRef::arc(arc)) - 17.0).abs() < 1e-6);
    for p in [s, e] {
        let (px, py) = sk.point_xy(p);
        let (cx, cy) = sk.point_xy(o);
        assert!(((px - cx).hypot(py - cy) - 17.0).abs() < 1e-6);
    }
}

#[test]
fn radius_drag_respects_an_equal_radius_chain() {
    let mut sk = Sketch::new();
    let o1 = sk.point(0.0, 0.0, true, "o1");
    let o2 = sk.point(40.0, 0.0, true, "o2");
    let a = sk.circle(o1, 10.0, "a");
    let b = sk.circle(o2, 10.0, "b");
    sk.add(Constraint::new(
        CKind::EqualRadius,
        vec![
            gcs_core::constraints::Arg::Ent(EntRef::circle(a)),
            gcs_core::constraints::Arg::Ent(EntRef::circle(b)),
        ],
    ));
    let mut d = RadiusDrag::new(&mut sk, EntRef::circle(a), 10.0, Method::DogLeg);
    assert!(d.move_to(&mut sk, 18.0).success);
    d.end(&mut sk);
    assert!((sk.radius_value(EntRef::circle(a)) - 18.0).abs() < 1e-6);
    assert!((sk.radius_value(EntRef::circle(b)) - 18.0).abs() < 1e-6);
}

/// When a cursor jump would flip a guard, the drag bisects the interval to keep as much of it as
/// stays on the branch.  Bisecting means halving the *suspect* end each time: re-testing the same
/// midpoint just spends the sub-step budget, and the point lands wherever that one midpoint put
/// it — a long way past the crossing it was supposed to stop at.
#[test]
fn a_flip_in_the_first_half_is_bisected_not_re_tested() {
    let mut sk = Sketch::new();
    let a = sk.point(0.0, 0.0, true, "a");
    let b = sk.point(10.0, 0.0, true, "b");
    let c = sk.point(5.9875, 0.3873, false, "c"); // on the circle |ac| = 6, just above a→b
    sk.add(Constraint::distance(EntRef::point(a), EntRef::point(c), 6.0));

    let (x, y) = sk.point_xy(c);
    // one frame, one increment, and the midpoint of it is already across the guard
    let mut d = Drag::new(&mut sk, c, x, y, Method::DogLeg, 1.0, vec![(a, b, c)], 10.0);
    d.move_to(&mut sk, 6.0, -20.0);
    d.end(&mut sk);

    assert_eq!(d.flips, vec![(a, b, c)], "the crossing is unavoidable and has to be reported");
    let (cx, cy) = sk.point_xy(c);
    assert!(cy < 0.0, "it did not cross at all: {:?}", (cx, cy));
    assert!(cy > -1.0, "it overshot the crossing: {:?}", (cx, cy));
}
