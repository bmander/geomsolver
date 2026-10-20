//! **A body turned whole** (`TurnDrag`, a shift-drag in the page): the selection rotated rigidly —
//! in the plane it is drawn in, or in space when a plane, a frame, an axis or a point in space is
//! in it — then polished, and the turn written back into the seeds.
use gcs_core::fmath;
use gcs_core::model::{EntRef, Sketch};
use gcs_core::solve::{solve, SolveOpts, TurnDrag, Turning};

fn close(a: [f64; 3], b: [f64; 3], tol: f64) -> bool {
    a.iter().zip(b).all(|(x, y)| (x - y).abs() < tol)
}

/// **A plane nothing holds turns in space about its own origin**: a drag to the right, seen
/// square on to its normal, swings it about the eye's up — a quarter turn carries its `u` from x
/// to y — and it stays where it stands, what is drawn on it carried with it; the turn is written
/// into its axes' `hint(dir: …)`, and reads back to the same attitude.
#[test]
fn a_free_plane_turns_about_its_origin_in_space() {
    let e = crate::common::read(
        "use std\nv0 := plane(u: hint(dir: (1, 0, 0)), v: hint(dir: (0, 0, 1))) \
         hint(origin: (10, 0, 0))\np := point hint((5, 5)) in v0\n",
    );
    let mut sk = e.sketch.clone();
    solve(&mut sk, SolveOpts::default());
    let v0 = crate::common::ent(&e, "v0").i();
    let p = crate::common::ent(&e, "p").i();
    let at = sk.world_point(p);
    let mut d = TurnDrag::new(&mut sk, &[EntRef::frame(v0)]);
    assert_eq!(d.turning, Turning::InSpace);
    assert!(close(d.pivot, [10.0, 0.0, 0.0], 1e-12), "pivot {:?}", d.pivot);
    assert!(d.carries(&sk) > 0);
    // the eye at bearing 0 stands out along x: its right is y, its up z
    let quarter = std::f64::consts::FRAC_PI_2;
    let steps = 9;
    for _ in 0..steps {
        let r = d.turn_seen(&mut sk, (0.0, 0.0), quarter / steps as f64, 0.0);
        assert!(r.success, "{r:?}");
    }
    let b = sk.basis(v0);
    assert!(close(b.o, [10.0, 0.0, 0.0], 1e-9), "the plane moved: {:?}", b.o);
    assert!(close(b.u, [0.0, 1.0, 0.0], 1e-9), "u {:?}", b.u);
    assert!(close(b.v, [0.0, 0.0, 1.0], 1e-9), "v {:?}", b.v);
    // p, 5 along u and 5 up from the origin, is carried round with it
    let turned = [10.0 - at[1], at[0] - 10.0, at[2]];
    assert!(close(sk.world_point(p), turned, 1e-9), "{:?} not {turned:?}", sk.world_point(p));
    let out = gcs_core::edit::commit_seeds(&e, &sk, &e.program);
    let back = crate::common::read(&out.text);
    let now = back.sketch.basis(crate::common::ent(&back, "v0").i());
    assert!(close(now.u, b.u, 1e-6) && close(now.v, b.v, 1e-6), "{now:?}\n{}", out.text);
}

/// **A plane held where it stands still turns there**: its origin `fix`ed, a drag down seen from
/// the front tips it about the eye's right, the near side going down.
#[test]
fn a_plane_held_at_its_origin_tips_about_the_eyes_right() {
    let e = crate::common::read(
        "use std\nv0 := plane(u: hint(dir: (0, 1, 0)), v: hint(dir: (0, 0, 1))) \
         hint(origin: (0, 0, 0))\nfix(origin == (0, 0, 0)) v0\n",
    );
    let mut sk = e.sketch.clone();
    solve(&mut sk, SolveOpts::default());
    let v0 = crate::common::ent(&e, "v0").i();
    let n0 = sk.basis(v0).normal();
    assert!(close(n0, [1.0, 0.0, 0.0], 1e-12), "facing the eye: {n0:?}");
    let mut d = TurnDrag::new(&mut sk, &[EntRef::frame(v0)]);
    let r = d.turn_seen(&mut sk, (0.0, 0.0), 0.0, 0.3);
    assert!(r.success, "{r:?}");
    let n = sk.basis(v0).normal();
    // the near side (its normal, toward the eye) swings down: about y, by 0.3
    let (s, c) = (fmath::sin(0.3), fmath::cos(0.3));
    assert!(close(n, [c, 0.0, -s], 1e-9), "{n:?}");
    assert!(close(sk.basis(v0).o, [0.0; 3], 1e-12));
}

/// **A figure drawn in a plane turns in that plane**, about the middle of its points: a triangle's
/// three lines turned a quarter keep every side, each corner a quarter round the middle.
#[test]
fn a_figure_turns_in_its_plane_about_its_middle() {
    let e = crate::common::read(
        "use std\nin std.front {\n  a := point hint((0, 0))\n  b := point hint((6, 0))\n  \
         c := point hint((0, 3))\n  ab := line(a, b)\n  bc := line(b, c)\n  ca := line(c, a)\n  \
         a distance(6) b\n  c distance(3) a\n}\n",
    );
    let mut sk = e.sketch.clone();
    solve(&mut sk, SolveOpts::default());
    let pts = ["a", "b", "c"].map(|n| crate::common::ent(&e, n).i());
    let was = pts.map(|p| sk.point_xy(p));
    let body = ["ab", "bc", "ca"].map(|n| crate::common::ent(&e, n));
    let mut d = TurnDrag::new(&mut sk, &body);
    let front = crate::common::ent(&e, "std.front").i();
    assert_eq!(d.turning, Turning::InPlane(Some(front)));
    let m = d.pivot;
    assert!(close(m, [2.0, 1.0, 0.0], 1e-12), "{m:?}");
    // the pointer swept a quarter round the middle, in many small steps
    let n = 24;
    let on = |t: f64| (m[0] + 4.0 * fmath::cos(t), m[1] + 4.0 * fmath::sin(t));
    for i in 0..n {
        let t = |k: usize| std::f64::consts::FRAC_PI_2 * k as f64 / n as f64;
        let r = d.sweep(&mut sk, on(t(i)), on(t(i + 1)));
        assert!(r.success, "{r:?}");
    }
    assert!((d.angle() - std::f64::consts::FRAC_PI_2).abs() < 1e-12, "{}", d.angle());
    for (p, w) in pts.into_iter().zip(was) {
        let want = (m[0] - (w.1 - m[1]), m[1] + (w.0 - m[0]));
        let got = sk.point_xy(p);
        assert!((got.0 - want.0).abs() < 1e-9 && (got.1 - want.1).abs() < 1e-9, "{got:?} {want:?}");
    }
}

/// **A body that holds a point turns about it**: a line with one end `fix`ed swings round that
/// end, its length kept.
#[test]
fn a_line_held_at_one_end_turns_about_that_end() {
    let e = crate::common::read(
        "use std\nin std.front {\n  a := point hint((1, 2))\n  b := point hint((5, 2))\n  \
         fix((1, 2)) a\n  l := line(a, b)\n}\n",
    );
    let mut sk = e.sketch.clone();
    solve(&mut sk, SolveOpts::default());
    let b = crate::common::ent(&e, "b").i();
    let mut d = TurnDrag::new(&mut sk, &[crate::common::ent(&e, "l")]);
    assert!(close(d.pivot, [1.0, 2.0, 0.0], 1e-12), "{:?}", d.pivot);
    let r = d.turn_in_plane(&mut sk, std::f64::consts::FRAC_PI_2);
    assert!(r.success, "{r:?}");
    let got = sk.point_xy(b);
    assert!((got.0 - 1.0).abs() < 1e-9 && (got.1 - 6.0).abs() < 1e-9, "{got:?}");
}

/// **Points in space turn in space** about their middle; a body nothing of which is free carries
/// nothing, and is left where it is.
#[test]
fn points_in_space_turn_about_their_middle_and_a_held_body_carries_nothing() {
    let e = crate::common::read(
        "a := point hint((0, 0, 0))\nb := point hint((4, 0, 0))\nl := line(a, b)\n\
         c := point hint((9, 9, 9))\nfix((9, 9, 9)) c\n",
    );
    let mut sk: Sketch = e.sketch.clone();
    solve(&mut sk, SolveOpts::default());
    let [a, b] = ["a", "b"].map(|n| crate::common::ent(&e, n).i());
    let mut d = TurnDrag::new(&mut sk, &[crate::common::ent(&e, "l")]);
    assert_eq!(d.turning, Turning::InSpace);
    assert!(close(d.pivot, [2.0, 0.0, 0.0], 1e-12));
    // seen from the front, a drag to the right turns it about the eye's up, z
    let r = d.turn_seen(&mut sk, (0.0, 0.0), std::f64::consts::FRAC_PI_2, 0.0);
    assert!(r.success, "{r:?}");
    let (pa, pb) = (sk.world_point(a), sk.world_point(b));
    assert!(close(pa, [2.0, -2.0, 0.0], 1e-9) && close(pb, [2.0, 2.0, 0.0], 1e-9), "{pa:?} {pb:?}");
    let c = crate::common::ent(&e, "c");
    let held = TurnDrag::new(&mut sk, &[c]);
    assert_eq!(held.carries(&sk), 0);
}

