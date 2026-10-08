//! The acceptance cases of #121: a rope stated as what it is — a curve of fixed length between
//! two points that minimises its potential energy — held to the closed-form catenary, reported a
//! minimum, and re-hung as an end is dragged; and Dido's problem, a fixed length enclosing the
//! most area against a line, held to the circular arc.

use gcs_core::constraints::CKind;
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
