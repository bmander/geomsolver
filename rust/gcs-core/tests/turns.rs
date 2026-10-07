//! A ring's turned copies in the model (`model/turns.rs`, issue #96): a copy's numbers are its
//! representative's turned about the ring's centre — derived, owning no column — and every
//! seam that solves, diagnoses, renumbers or saves a sketch carries that.  Built by hand, so
//! nothing here depends on the language.

use crate::common::fd_jacobian;
use gcs_core::constraints::{Arg, CKind, Constraint};
use gcs_core::diagnose::{diagnose, DiagnoseOptions};
use gcs_core::io::{self, Part};
use gcs_core::model::{EntKind, EntRef, Sketch, Turn};
use gcs_core::solve::{solve, SolveOpts};
use gcs_core::system::System;

fn pt(sk: &Sketch, i: usize) -> (f64, f64) {
    let p = &sk.points[i];
    (sk.params[p.x as usize].value, sk.params[p.y as usize].value)
}

/// `q` turned `k` of `n` steps about `c`, worked out here.
fn turned(c: (f64, f64), q: (f64, f64), k: u32, n: u32) -> (f64, f64) {
    let th = std::f64::consts::TAU * k as f64 / n as f64;
    let (dx, dy) = (q.0 - c.0, q.1 - c.1);
    (c.0 + th.cos() * dx - th.sin() * dy, c.1 + th.sin() * dx + th.cos() * dy)
}

fn close(a: (f64, f64), b: (f64, f64)) -> bool {
    (a.0 - b.0).abs() < 1e-9 && (a.1 - b.1).abs() < 1e-9
}

/// A free centre, a representative and its two turns (a ring of 3), and a free point `q`.
fn three() -> (Sketch, usize, usize, [usize; 2], usize) {
    let mut sk = Sketch::new();
    let c = sk.point(1.0, 2.0, false, "c");
    let p = sk.point(9.0, 3.0, false, "p");
    let p1 = sk.point(0.0, 0.0, false, "p1");
    let p2 = sk.point(0.0, 0.0, false, "p2");
    let q = sk.point(-4.0, 6.0, false, "q");
    for (k, copy) in [(1, p1), (2, p2)] {
        sk.turns.push(Turn {
            copy: EntRef::point(copy),
            rep: EntRef::point(p),
            about: EntRef::point(c),
            k,
            n: 3,
        });
    }
    sk.settle_turns();
    (sk, c, p, [p1, p2], q)
}

#[test]
fn a_copy_is_its_representative_turned_and_owns_no_column() {
    let (sk, c, p, [p1, p2], _) = three();
    assert!(close(pt(&sk, p1), turned(pt(&sk, c), pt(&sk, p), 1, 3)));
    assert!(close(pt(&sk, p2), turned(pt(&sk, c), pt(&sk, p), 2, 3)));
    // c, p and q are the unknowns: the two copies are worked out from them
    assert_eq!(System::new(&sk).n_free, 6);
    let mut moved = sk.clone();
    let mut x = moved.get_x();
    x[moved.points[p].x as usize] += 5.0;
    moved.set_x(&x);
    assert!(close(pt(&moved, p2), turned(pt(&moved, c), pt(&moved, p), 2, 3)), "set_x settles");
}

#[test]
fn a_row_on_a_copy_is_differentiated_through_the_turn() {
    let (mut sk, c, p, [p1, p2], q) = three();
    let e = |i| Arg::Ent(EntRef::point(i));
    sk.add(Constraint::new(CKind::Distance, vec![e(p1), e(q), Arg::Num(7.0)]));
    sk.add(Constraint::new(CKind::Distance, vec![e(p2), e(q), Arg::Num(12.0)]));
    sk.add(Constraint::new(CKind::Distance, vec![e(c), e(p2), Arg::Num(8.0)]));
    fd_jacobian(&sk, 1e-6);
    let r = solve(&mut sk, SolveOpts::default());
    assert!(r.success, "{r:?}");
    let d = |a: (f64, f64), b: (f64, f64)| (a.0 - b.0).hypot(a.1 - b.1);
    assert!((d(pt(&sk, p1), pt(&sk, q)) - 7.0).abs() < 1e-7);
    assert!((d(pt(&sk, p2), pt(&sk, q)) - 12.0).abs() < 1e-7);
    // the centre's distance to a copy is its distance to the representative
    assert!((d(pt(&sk, c), pt(&sk, p)) - 8.0).abs() < 1e-7);
    assert!(close(pt(&sk, p1), turned(pt(&sk, c), pt(&sk, p), 1, 3)), "still a turn");
    // 6 unknowns, 3 rows
    let dg = diagnose(&mut sk, DiagnoseOptions::default());
    assert_eq!((dg.n_params, dg.dof), (6, 3));
}

#[test]
fn a_turned_arc_states_its_ends_once() {
    let mut sk = Sketch::new();
    let c = sk.point(0.0, 0.0, true, "c");
    let (a, b) = (sk.point(5.0, 0.0, false, "a"), sk.point(0.0, 5.0, false, "b"));
    let arc = sk.arc(c, a, b, "k");
    let (a1, b1) = (sk.point(0.0, 0.0, false, "a1"), sk.point(0.0, 0.0, false, "b1"));
    let arc1 = sk.arc(c, a1, b1, "k1");
    let ring = |copy, rep| Turn { copy, rep, about: EntRef::point(c), k: 1, n: 4 };
    sk.turns.push(ring(EntRef::point(a1), EntRef::point(a)));
    sk.turns.push(ring(EntRef::point(b1), EntRef::point(b)));
    sk.turns.push(ring(EntRef::arc(arc1), EntRef::arc(arc)));
    sk.settle_turns();
    // the representative's two intrinsic rows; the copy's are their images and are not compiled
    let sys = System::new(&sk);
    assert_eq!((sys.n_free, sys.n_res), (5, 2));
    let r = sk.params[sk.arcs[arc].radius as usize].value;
    assert_eq!(sk.params[sk.arcs[arc1].radius as usize].value, r);
}

#[test]
fn a_turn_about_a_held_axis_turns_a_point_in_space() {
    let mut sk = Sketch::new();
    let t = sk.axis([0.0, 0.0, 1.0], "t");
    for &p in &sk.axes[t].d {
        sk.params[p as usize].fixed = true;
    }
    let p = sk.point(3.0, 0.0, false, "p");
    sk.give_place(p, 2.0);
    let p1 = sk.point(0.0, 0.0, false, "p1");
    sk.give_place(p1, 0.0);
    let about = EntRef::new(EntKind::Axis, t);
    sk.turns.push(Turn { copy: EntRef::point(p1), rep: EntRef::point(p), about, k: 1, n: 4 });
    sk.settle_turns();
    let v = |i: u32| sk.params[i as usize].value;
    let q = &sk.points[p1];
    assert!((v(q.x) - 0.0).abs() < 1e-12 && (v(q.y) - 3.0).abs() < 1e-12);
    assert_eq!(v(q.z.unwrap()), 2.0);
    fd_jacobian(&sk, 1e-6);
}

#[test]
fn turns_are_saved_and_renumbered() {
    let (mut sk, _, p, [p1, p2], q) = three();
    let e = |i| Arg::Ent(EntRef::point(i));
    sk.add(Constraint::new(CKind::Distance, vec![e(p1), e(q), Arg::Num(7.0)]));
    let back = io::loads(&io::dumps(&sk, None)).unwrap();
    assert_eq!(back.turns, sk.turns);
    assert_eq!(System::new(&back).n_free, 6);
    // deleting what a copy is not worked out from keeps the turn, renumbered
    let kept = io::without(&sk, &[EntRef::point(q)], &[]);
    assert_eq!(kept.turns.len(), 2);
    assert_eq!(kept.turns[0].copy, EntRef::point(p1 - 0));
    // deleting the representative leaves its copies standing free where they are
    let freed = io::without(&sk, &[EntRef::point(p)], &[]);
    assert!(freed.turns.is_empty());
    assert_eq!(pt(&freed, p2 - 1), pt(&sk, p2));
}

#[test]
fn a_drag_part_reaching_a_copy_reaches_what_it_turns_with() {
    let (mut sk, _, _, [p1, _], q) = three();
    let e = |i| Arg::Ent(EntRef::point(i));
    sk.add(Constraint::new(CKind::Distance, vec![e(p1), e(q), Arg::Num(7.0)]));
    let part = Part::around(&sk, EntRef::point(q));
    // q, its copy p1 — and through the turn the centre, the representative and the other copy
    assert_eq!(part.sketch.points.len(), 5);
    assert_eq!(part.sketch.turns.len(), 2);
    assert_eq!(System::new(&part.sketch).n_free, 6);
}

#[test]
fn a_malformed_turn_is_refused_on_load() {
    let (sk, ..) = three();
    let text = io::dumps(&sk, None).replace("\"k\":1", "\"k\":3");
    assert!(io::loads(&text).is_err());
}
