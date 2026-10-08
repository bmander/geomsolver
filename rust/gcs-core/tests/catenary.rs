//! The acceptance cases of #121: a rope stated as what it is — a curve of fixed length between
//! two points that minimises its potential energy — held to the closed-form catenary, reported a
//! minimum, and re-hung as an end is dragged; and Dido's problem, a fixed length enclosing the
//! most area against a line, held to the circular arc.

use gcs_core::program::{elaborate, Elaborated};
use gcs_core::solve::{solve, SolveOpts};
use crate::common::parse;
use crate::minimize::off_catenary;

fn solved(src: &str) -> Elaborated {
    let (prog, errs) = parse(src);
    assert!(errs.is_empty(), "{:?}", errs.iter().map(|e| &e.message).collect::<Vec<_>>());
    let mut e = elaborate(&prog);
    let d: Vec<String> = e.diags.iter().map(|d| format!("{}: {}", d.code.as_str(), d.message)).collect();
    assert!(e.ok(), "{d:?}");
    let r = solve(&mut e.sketch, SolveOpts::default());
    assert!(r.success, "{}", r.message);
    e
}

pub const ROPE: &str = "\
use std
in std.front {
  a := point
  b := point
  fix((0, 0)) a
  fix((100, 0)) b
  rope := spline(a, b)
  length(150) rope
}
minimize integral(p.y over p in rope)
";

/// The free rope hangs as the catenary `a cosh((x − 50)/a) + c` through its ends, its interior
/// found from a seed that only knew the chord and the length.
#[test]
fn a_free_rope_hangs_as_the_catenary() {
    let e = solved(ROPE);
    let sp = &e.sketch.splines[0];
    assert!(sp.free);
    assert_eq!(sp.ctrl.len(), gcs_core::variational::FREE_CTRL);
    let off = off_catenary(&e.sketch, 0, 150.0);
    eprintln!("off {off}");
    assert!(off < 1e-2, "{off}");
}

/// The verdict on the hanging rope is a minimum — and on the same rope stood on its head, a
/// standing arch where the height is greatest, a maximum.
#[test]
fn the_rope_is_a_minimum_and_the_arch_a_maximum() {
    let mut e = solved(ROPE);
    let d = gcs_core::diagnose::diagnose(&mut e.sketch, gcs_core::diagnose::DiagnoseOptions::default());
    let names: Vec<&str> = d.extrema.iter().map(|(_, v)| v.name()).collect();
    assert_eq!(names, ["minimum"]);
    let mut arch = solved(&ROPE.replace("minimize", "maximize"));
    let d = gcs_core::diagnose::diagnose(&mut arch.sketch, gcs_core::diagnose::DiagnoseOptions::default());
    let names: Vec<&str> = d.extrema.iter().map(|(_, v)| v.name()).collect();
    assert_eq!(names, ["maximum"]);
    // and the report says each is what its statement asked for
    let asked = |e: &Elaborated, d: &gcs_core::diagnose::Diagnosis| {
        gcs_core::report::diagnosis_json(&e.sketch, d).dump(None)
    };
    assert!(asked(&arch, &d).contains(r#""extrema":[["#) && asked(&arch, &d).contains(r#","maximum",true]"#), "{}", asked(&arch, &d));
    // the arch is the rope turned over the chord
    for (i, &p) in e.sketch.splines[0].ctrl.iter().enumerate() {
        let (x, y) = e.sketch.point_xy(p as usize);
        let (ax, ay) = arch.sketch.point_xy(arch.sketch.splines[0].ctrl[i] as usize);
        assert!((x - ax).abs() < 1e-6 && (y + ay).abs() < 1e-6, "{i}: ({x}, {y}) against ({ax}, {ay})");
    }
}

/// An end let go and dragged: every frame the rope hangs again from where the end is, as the
/// catenary through the new ends of the same length.
#[test]
fn dragging_an_end_rehangs_the_rope() {
    let mut e = solved(&ROPE.replace("fix((100, 0)) b", "b := point hint((100, 0))").replace("  b := point\n", ""));
    let b = e.sketch.splines[0].ctrl[gcs_core::variational::FREE_CTRL - 1] as usize;
    let mut drag = gcs_core::decompose::PlanDrag::new(&e.sketch, b, 100.0, 0.0, None, 0.05);
    for k in 1..=10 {
        let x = 100.0 - 2.0 * k as f64;
        let r = drag.move_to(&mut e.sketch, None, x, 0.0);
        assert!(r.success, "frame {k}: {r:?}");
        let (bx, by) = e.sketch.point_xy(b);
        assert!((bx - x).abs() < 1e-6 && by.abs() < 1e-6, "frame {k}: b at ({bx}, {by})");
    }
    drag.end();
    // the ends now 80 apart: the catenary of length 150 over that span, centred on its middle
    let (lo, hi) = (e.sketch.point_xy(0), e.sketch.point_xy(b));
    let span = hi.0 - lo.0;
    let (mut a_lo, mut a_hi): (f64, f64) = (1.0, 1e4);
    for _ in 0..200 {
        let a = 0.5 * (a_lo + a_hi);
        if 2.0 * a * (span / 2.0 / a).sinh() > 150.0 { a_lo = a } else { a_hi = a }
    }
    let a = 0.5 * (a_lo + a_hi);
    let c = -a * (span / 2.0 / a).cosh();
    let (t0, t1) = gcs_core::curve::domain(&e.sketch, 0);
    let off = (0..=200)
        .map(|k| {
            let (x, y) = gcs_core::curve::point_at(&e.sketch, 0, t0 + (t1 - t0) * k as f64 / 200.0);
            (y - (a * ((x - span / 2.0) / a).cosh() + c)).abs()
        })
        .fold(0.0, f64::max);
    // as near as sixteen points come to so deep a sag — and exactly where a rope hung there from
    // the start comes to rest, so the drag carried nothing of where it began
    assert!(off < 5e-2, "{off}");
    let fresh = solved(&ROPE.replace("fix((100, 0)) b", "fix((80, 0)) b"));
    for (i, &p) in e.sketch.splines[0].ctrl.iter().enumerate() {
        let (x, y) = e.sketch.point_xy(p as usize);
        let (fx, fy) = fresh.sketch.point_xy(fresh.sketch.splines[0].ctrl[i] as usize);
        assert!((x - fx).hypot(y - fy) < 1e-6, "{i}: ({x}, {y}) against ({fx}, {fy})");
    }
}

/// **Dido's problem**: of every curve of length 130 standing on a chord of 100, the one enclosing
/// the most area with it — `∮ (x dy − y dx) / 2`, the chord adding nothing on `y = 0` — is the
/// arc of the circle through the ends with that length, `R sin α = 50`, `2Rα = 130`.
#[test]
fn didos_curve_is_the_circular_arc() {
    let src = ROPE.replace("length(150) rope", "length(130) rope").replace(
        "minimize integral(p.y over p in rope)",
        "maximize integral((p.x * t.y - p.y * t.x) / 2 over (p, t) in rope)",
    );
    let mut e = solved(&src);
    let (mut lo, mut hi): (f64, f64) = (1e-6, std::f64::consts::PI - 1e-9);
    for _ in 0..200 {
        let al = 0.5 * (lo + hi);
        if 130.0 * al.sin() / (2.0 * al) > 50.0 { lo = al } else { hi = al }
    }
    let al = 0.5 * (lo + hi);
    let r = 65.0 / al;
    // the arc bulges below the chord: walked from a to b, that is the side the area is counted on
    let cy = r * al.cos();
    let (t0, t1) = gcs_core::curve::domain(&e.sketch, 0);
    let off = (0..=400)
        .map(|k| {
            let (x, y) = gcs_core::curve::point_at(&e.sketch, 0, t0 + (t1 - t0) * k as f64 / 400.0);
            ((x - 50.0).hypot(y - cy) - r).abs()
        })
        .fold(0.0, f64::max);
    assert!(off < 1e-3, "{off}");
    let d = gcs_core::diagnose::diagnose(&mut e.sketch, gcs_core::diagnose::DiagnoseOptions::default());
    let names: Vec<&str> = d.extrema.iter().map(|(_, v)| v.name()).collect();
    assert_eq!(names, ["maximum"]);
}

/// `std.hangs(L) rope` is the longhand in one word: the same rope, the same verdict, and a rope
/// deleted takes the word's use with it.
#[test]
fn the_library_word_hangs_the_same_rope() {
    let longhand = solved(ROPE);
    let src = ROPE
        .replace("use std\n", "use std (hangs)\n")
        .replace("  length(150) rope\n", "  hangs(L: 150) rope\n")
        .replace("minimize integral(p.y over p in rope)\n", "");
    let mut e = solved(&src);
    for (i, &p) in e.sketch.splines[0].ctrl.iter().enumerate() {
        let (x, y) = e.sketch.point_xy(p as usize);
        let (lx, ly) = longhand.sketch.point_xy(longhand.sketch.splines[0].ctrl[i] as usize);
        assert!((x - lx).hypot(y - ly) < 1e-9, "{i}");
    }
    let d = gcs_core::diagnose::diagnose(&mut e.sketch, gcs_core::diagnose::DiagnoseOptions::default());
    let names: Vec<&str> = d.extrema.iter().map(|(_, v)| v.name()).collect();
    assert_eq!(names, ["minimum"]);
    let out = gcs_core::edit::remove(&e, &e.program, &e.sketch, &[gcs_core::model::EntRef::spline(0)], &[]);
    assert!(!out.text.contains("hangs(L: 150) rope"), "{}", out.text);
}

/// A free curve's interior is its statement's, nameless and never written: after a solve, a
/// gesture's reconcile and a seed writeback leave the source as it was.
#[test]
fn a_free_curve_writes_nothing_back() {
    let mut e = solved(ROPE);
    let sk = e.sketch.clone();
    let out = gcs_core::edit::reconcile(&mut e, &sk);
    assert_eq!(out.text, ROPE, "reconcile wrote {}", out.text);
    let seeds = gcs_core::edit::commit_seeds(&e, &e.sketch, &e.program);
    assert_eq!(seeds.text, ROPE, "a writeback wrote {}", seeds.text);
}

/// An end dragged is a seed like any other: its `hint(…)` is rewritten where it moved to, and the
/// free curve's interior beside it is still not written.
#[test]
fn a_dragged_end_writes_its_hint() {
    let src = ROPE.replace("  b := point\n", "  b := point hint((100, 0))\n").replace("  fix((100, 0)) b\n", "");
    let mut e = solved(&src);
    let b = e.sketch.splines[0].ctrl[gcs_core::variational::FREE_CTRL - 1] as usize;
    let mut drag = gcs_core::decompose::PlanDrag::new(&e.sketch, b, 100.0, 0.0, None, 0.05);
    let r = drag.move_to(&mut e.sketch, None, 90.0, -10.0);
    assert!(r.success, "{r:?}");
    drag.end();
    let seeds = gcs_core::edit::commit_seeds(&e, &e.sketch, &e.program);
    assert!(seeds.text.contains("b := point hint((90, -10))"), "{}", seeds.text);
    assert_eq!(seeds.text.matches("hint").count(), 1, "{}", seeds.text);
}

/// The rope with a line drawn beside it, held or free, and made tangent to it.
fn with_line(fixed: bool, at: [(f64, f64); 2]) -> (Elaborated, usize, usize) {
    use gcs_core::constraints::Constraint;
    use gcs_core::model::EntRef;
    let mut e = solved(ROPE);
    let sk = &mut e.sketch;
    let p = sk.point(at[0].0, at[0].1, fixed, "lp");
    let q = sk.point(at[1].0, at[1].1, fixed, "lq");
    let l = sk.line(p, q);
    let c = Constraint::spline_tangent_line(sk, EntRef::spline(0), EntRef::line(l));
    sk.add(c);
    (e, p, q)
}

/// **A line drawn tangent to the rope moves to touch it**: the line is free, so the tangency is
/// the drawing's to satisfy and presses on nothing — the rope hangs as the catenary, the line
/// keeps the freedoms the tangency leaves it, and the rope is still a minimum.
#[test]
fn a_free_line_drawn_tangent_to_the_rope_moves_to_it() {
    let (mut e, p, q) = with_line(false, [(20.0, -40.0), (90.0, -45.0)]);
    let r = gcs_core::solve::solve(&mut e.sketch, gcs_core::solve::SolveOpts::default());
    assert!(r.success, "{}", r.message);
    let off = off_catenary(&e.sketch, 0, 150.0);
    assert!(off < 1e-2, "{off}");
    let d = gcs_core::diagnose::diagnose(&mut e.sketch, gcs_core::diagnose::DiagnoseOptions::default());
    // the line's four coordinates less the one the tangency takes
    assert_eq!(d.dof, 3, "{d:?}");
    assert_eq!(d.extrema.iter().map(|(_, v)| v.name()).collect::<Vec<_>>(), ["minimum"]);
    // and it touches: the curve's nearest point to the line is on it
    let (a, b) = (e.sketch.point_xy(p), e.sketch.point_xy(q));
    let (t0, t1) = gcs_core::curve::domain(&e.sketch, 0);
    let gap = (0..=2000)
        .map(|k| {
            let (x, y) = gcs_core::curve::point_at(&e.sketch, 0, t0 + (t1 - t0) * k as f64 / 2000.0);
            ((b.0 - a.0) * (y - a.1) - (b.1 - a.1) * (x - a.0)) / (b.0 - a.0).hypot(b.1 - a.1)
        })
        .map(f64::abs)
        .fold(f64::INFINITY, f64::min);
    assert!(gap < 1e-3, "{gap}");
}

/// **A held line presses the rope**: nothing else can move to satisfy the tangency, so it is a
/// contact of the energy, with a multiplier of its own — the rope's lowest point is lifted onto a
/// level line a little above where it would hang, or pulled down to one a little below, at DOF 0.
#[test]
fn a_held_line_tangent_to_the_rope_lifts_or_lowers_it() {
    let (a, c) = crate::minimize::catenary(150.0);
    for dy in [1.0, -1.0] {
        let level = a + c + dy;
        let (mut e, _, _) = with_line(true, [(20.0, level), (90.0, level)]);
        let r = gcs_core::solve::solve(&mut e.sketch, gcs_core::solve::SolveOpts::default());
        assert!(r.success, "{dy}: {}", r.message);
        let (t0, t1) = gcs_core::curve::domain(&e.sketch, 0);
        let lowest = (0..=2000)
            .map(|k| gcs_core::curve::point_at(&e.sketch, 0, t0 + (t1 - t0) * k as f64 / 2000.0).1)
            .fold(f64::INFINITY, f64::min);
        assert!((lowest - level).abs() < 1e-3, "{dy}: lowest {lowest} against {level}");
        let d = gcs_core::diagnose::diagnose(&mut e.sketch, gcs_core::diagnose::DiagnoseOptions::default());
        assert_eq!(d.dof, 0, "{dy}: {d:?}");
    }
}

/// A held line the rope cannot be bent to touch from where it hangs is a solve that fails and
/// says so — and the diagnosis, which then searches for a conflict among subsets of the
/// constraints, never reads an energy whose rows it has set aside.
#[test]
fn a_rope_that_cannot_touch_is_diagnosed_not_crashed() {
    let (mut e, _, _) = with_line(true, [(20.0, -40.0), (90.0, -45.0)]);
    let _ = gcs_core::solve::solve(&mut e.sketch, gcs_core::solve::SolveOpts::default());
    let _ = gcs_core::diagnose::diagnose(&mut e.sketch, gcs_core::diagnose::DiagnoseOptions::default());
}

/// The same, written: `rope tangent floor` in the document elaborates, solves, and leaves the rope
/// the catenary with the line moved onto it.
#[test]
fn a_written_tangent_to_the_rope_elaborates_and_solves() {
    let src = ROPE.replace(
        "  length(150) rope\n",
        "  length(150) rope\n  f0 := point hint((20, -40))\n  f1 := point hint((90, -45))\n  floor := line(f0, f1)\n  rope tangent floor\n",
    );
    let e = solved(&src);
    let off = off_catenary(&e.sketch, 0, 150.0);
    assert!(off < 1e-2, "{off}");
}
