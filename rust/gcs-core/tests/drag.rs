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
    drag.end();
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
    let mut d = PlanDrag::new(&sk, c, x, y, None, 0.05);
    // the apex is determined by its two distances: it is part of the ground's rigid body, and
    // the drag starts from the solved configuration and then has nothing it may move
    assert!(d.usable());
    let mut ys = Vec::new();
    for i in 0..17 {
        let yy = 4.0 - 8.0 * i as f64 / 16.0;
        let r = d.move_to(&mut sk, 5.0, yy);
        assert!(r.success && r.message.contains("held"), "{r:?}");
        ys.push(sk.point_xy(c).1);
    }
    d.end();
    assert!(d.flips().is_empty());
    let y_solved = (36.0f64 - 25.0).sqrt();
    assert!(ys.iter().all(|&v| (v - y_solved).abs() < 1e-9), "{ys:?}");
}

/// A drag moves the plan's roots as rigid bodies, and only the ones it has to: on a chain of
/// levelled segments the dragged corner, and the two next to it sliding along their own lines.
/// Everything further along is not computed, let alone written — the cost of a frame is the
/// region, not the chain.
#[test]
fn a_drag_on_a_levelled_chain_moves_three_corners_and_leaves_the_rest_untouched() {
    let n = 64;
    let mut sk = examples::zigzag(n, 1);
    let p = n / 2;
    let (x0, y0) = sk.point_xy(p);
    let before = sk.get_x();
    let mut d = PlanDrag::new(&sk, p, x0, y0, None, 0.05);
    assert!(d.usable());
    let r = d.move_to(&mut sk, x0 + 1.5, y0 - 2.5);
    assert!(r.success && r.message == "plan-drag", "{r:?}");
    assert_eq!(sk.point_xy(p), (x0 + 1.5, y0 - 2.5));
    d.end();
    let after = sk.get_x();
    for k in 0..n {
        let [ix, iy] = sk.point_params(k);
        let moved = before[ix as usize] != after[ix as usize] || before[iy as usize] != after[iy as usize];
        // the neighbours slide: one along the vertical line into p, one along the horizontal
        assert_eq!(moved, (k as i64 - p as i64).abs() <= 1, "point {k}");
    }
    // and the constraints hold exactly
    let mut sys = System::new(&sk);
    let z = sys.z0(&sk);
    assert!(sys.max_relative_residual(&z) < 1e-9);
}

/// A point on a circle about a fixed centre is a body with one degree of freedom: pulled away
/// from the circle it goes round it, as near the cursor as it may, and says so.
#[test]
fn a_point_held_on_a_circle_goes_round_it() {
    let mut sk = Sketch::new();
    let c = sk.point(0.0, 0.0, true, "c");
    let p = sk.point(10.0, 0.0, false, "p");
    sk.add(Constraint::distance(EntRef::point(c), EntRef::point(p), 10.0));
    let mut d = PlanDrag::new(&sk, p, 10.0, 0.0, None, 1.0);
    assert!(d.usable());
    let r = d.move_to(&mut sk, 0.0, 20.0);
    assert!(r.success && r.message.contains("held"), "{r:?}");
    let (x, y) = sk.point_xy(p);
    assert!((x.hypot(y) - 10.0).abs() < 1e-9 && y > 9.0, "{:?}", (x, y));
    d.end();
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
    let mut d = PlanDrag::new(&sk, p, x, y, None, 0.05);
    let res = d.move_to(&mut sk, x + 200.0, y); // far beyond one increment
    d.end();
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

/// A drag is built on the dragged point's part of the document — what is connected to it — and
/// solves, decomposes and writes only that.  Dragging a point of the middle of three separate
/// staircases is, parameter for parameter, dragging a lone staircase: the other two are never
/// touched, and the systems are the lone one's size.
#[test]
fn a_drag_costs_the_figure_not_the_document() {
    let n = 16;
    let mut one = examples::zigzag(n, 1);
    let mut three = examples::zigzag(n, 3);
    let (p1, p3) = (n / 2, n + n / 2); // the same point of the lone chain and of the middle one
    let dx = three.point_xy(p3).0 - one.point_xy(p1).0;
    let (x0, y0) = one.point_xy(p1);
    let before = three.get_x();
    let mut d1 = PlanDrag::new(&one, p1, x0, y0, None, 0.05);
    let mut d3 = PlanDrag::new(&three, p3, x0 + dx, y0, None, 0.05);
    assert_eq!(d3.part.sketch.points.len(), n);
    assert_eq!(d3.solver.system.n_free, d1.solver.system.n_free);
    assert_eq!(d3.solver.plan.steps.len(), d1.solver.plan.steps.len());
    for i in 1..=12 {
        let t = i as f64 * 0.4;
        let (x, y) = (x0 + 6.0 * t.cos(), y0 + 4.0 * t.sin());
        let r1 = d1.move_to(&mut one, x, y);
        let r3 = d3.move_to(&mut three, x + dx, y);
        assert_eq!(r1.success, r3.success);
        for k in 0..n {
            let (ax, ay) = one.point_xy(k);
            let (bx, by) = three.point_xy(n + k);
            assert!((ax + dx - bx).abs() < 1e-9 && (ay - by).abs() < 1e-9, "point {k} frame {i}");
        }
    }
    d1.end();
    d3.end();
    let after = three.get_x();
    let middle: Vec<u32> = (n..2 * n).flat_map(|k| three.point_params(k)).collect();
    for (i, (a, b)) in before.iter().zip(&after).enumerate() {
        if !middle.contains(&(i as u32)) {
            assert_eq!(a, b, "param {i} outside the dragged figure was written");
        }
    }
    assert!(three.constraints.len() == 3 * (n - 1), "the document itself was restructured");
}

/// A body nothing holds rides along with the cursor: it slides, it does not spin about its own
/// centre — turning is the dear way to move a point, taken only when an anchor leaves no other.
#[test]
fn a_free_body_slides_rather_than_spins() {
    let mut sk = examples::truss_floating(6);
    let p = 3;
    let (x, y) = sk.point_xy(p);
    let before = sk.get_x();
    let mut d = PlanDrag::new(&sk, p, x, y, None, 1.0);
    assert!(d.usable());
    d.move_to(&mut sk, x + 10.0, y + 4.0);
    d.end();
    let after = sk.get_x();
    for k in 0..sk.points.len() {
        let [ix, iy] = sk.point_params(k);
        let (dx, dy) = (after[ix as usize] - before[ix as usize], after[iy as usize] - before[iy as usize]);
        assert!((dx - 10.0).abs() < 0.2 && (dy - 4.0).abs() < 0.2, "point {k} moved by {:?}", (dx, dy));
    }
}

/// The wave never re-reads what it moved: a long drag that goes round and round leaves the
/// constraints as exactly satisfied as it found them, and stays on the plan throughout.
#[test]
fn a_long_drag_does_not_drift() {
    let mut sk = examples::zigzag(32, 1);
    let p = 16;
    let (x, y) = sk.point_xy(p);
    let mut d = PlanDrag::new(&sk, p, x, y, None, 0.05);
    for i in 0..3000 {
        let a = 0.05 * i as f64;
        let r = d.move_to(&mut sk, x + 2.0 * a.cos(), y + 1.5 * (1.7 * a).sin());
        assert!(r.success && r.message == "plan-drag", "frame {i}: {r:?}");
    }
    assert!(d.usable());
    d.end();
    let mut sys = System::new(&sk);
    let z = sys.z0(&sk);
    assert!(sys.max_relative_residual(&z) < 1e-10, "{:e}", sys.max_relative_residual(&z));
}
