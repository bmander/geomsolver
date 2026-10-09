//! **A bound chooses the root** (§9.6, #109): `distance(>= d)`, `distance(<= d)`, `distance(in:
//! (a, b))` and `p inside P` / `p outside P` add no row.  Read on the solution, they say which of
//! the equations' solutions is meant, and a solve that lands on another is steered to it — or
//! fails, since a solver may not report a solution violating one.
use gcs_core::diagnose::{diagnose, DiagnoseOptions};
use gcs_core::program::Elaborated;
use gcs_core::solve::{solve, SolveOpts, BOUND_BROKEN};
use gcs_core::system::System;

use crate::common::{ent, read, refused};

/// A point on two circles about `a` and `b`, five from each: a root above the line through them
/// and a root below, seeded below.
const CROSSING: &str = "unit mm\nuse std\nin std.front {\n  a := point hint((0, 0))\n  \
                        b := point hint((6, 0))\n  p := point hint((3, -2))\n}\nfix((0, 0)) a\n\
                        fix((6, 0)) b\np distance(5) a\np distance(5) b\n";

/// Where `p` stands in space.
fn p_at(e: &Elaborated) -> [f64; 3] {
    e.sketch.world_point(ent(e, "p").i())
}

fn solved(src: &str) -> Elaborated {
    let mut e = read(src);
    let r = solve(&mut e.sketch, SolveOpts::default());
    assert!(r.success, "{}\n{src}", r.message);
    e
}

/// Unbounded, the seed picks the root below; bounded above, the solve is steered to the other.
#[test]
fn a_bound_chooses_the_root_the_seed_did_not() {
    let below = solved(CROSSING);
    assert!((p_at(&below)[2] + 4.0).abs() < 1e-9, "{:?}", p_at(&below));
    let upward = ["a distance(>= 0, along: up) p", "p outside std.top",
                  "a distance(in: (1, 9), along: up) p"];
    for bound in upward {
        let e = solved(&format!("{CROSSING}{bound}\n"));
        let at = p_at(&e);
        assert!((at[0] - 3.0).abs() < 1e-9 && (at[2] - 4.0).abs() < 1e-9, "{bound}: {at:?}");
    }
    // and the other way, from a seed above
    let above = CROSSING.replace("hint((3, -2))", "hint((3, 2))");
    for bound in ["a distance(<= 0, along: up) p", "p inside std.top"] {
        let e = solved(&format!("{above}{bound}\n"));
        assert!((p_at(&e)[2] + 4.0).abs() < 1e-9, "{bound}: {:?}", p_at(&e));
    }
}

/// A bound adds no row: the same unknowns, the same equations and the same freedoms as the
/// drawing without it — and a drawing whose bound holds solves to the same bits.
#[test]
fn a_bound_adds_no_row() {
    let plain = read(CROSSING);
    let bounded = read(&format!("{CROSSING}a distance(<= 0, along: up) p\n"));
    let count = |e: &Elaborated| {
        let sys = System::new(&e.sketch);
        (sys.hard_rows().len(), sys.n_free)
    };
    assert_eq!(count(&plain), count(&bounded));
    let (mut a, mut b) = (plain, bounded);
    solve(&mut a.sketch, SolveOpts::default());
    solve(&mut b.sketch, SolveOpts::default());
    let bits =
        |e: &Elaborated| e.sketch.params.iter().map(|p| p.value.to_bits()).collect::<Vec<_>>();
    assert_eq!(bits(&a), bits(&b));
    assert_eq!(diagnose(&mut a.sketch, DiagnoseOptions::default()).dof,
               diagnose(&mut b.sketch, DiagnoseOptions::default()).dof);
}

/// A bound no root keeps fails the solve, and the diagnosis says which; one the solution stands
/// on holds, and is told as active.
#[test]
fn a_bound_no_root_keeps_fails_the_solve() {
    let mut e = read(&format!("{CROSSING}a distance(>= 10, along: up) p\n"));
    let r = solve(&mut e.sketch, SolveOpts::default());
    assert!(!r.success && r.status == BOUND_BROKEN, "{r:?}");
    assert!(r.message.contains("distance(>= 10"), "{}", r.message);
    let d = diagnose(&mut e.sketch, DiagnoseOptions::default());
    assert_eq!(d.bounds_violated.len(), 1);
    let mut e = solved(&format!("{CROSSING}a distance(>= 4, along: up) p\n"));
    let d = diagnose(&mut e.sketch, DiagnoseOptions::default());
    assert!(d.bounds_violated.is_empty() && d.bounds_active.len() == 1, "{d:?}");
}

/// What a bound may be said of, and how.
#[test]
fn a_bound_is_said_of_a_measure() {
    let open = format!("{CROSSING}a distance(> 0, along: up) p\n");
    let (_, errs, _) = gcs_core::library::parse_linked(&open);
    assert!(errs.iter().any(|e| e.message.contains("closed")), "{errs:?}");
    refused(&format!("{CROSSING}c := circle(center: a) hint(r: 2) in std.front\nradius(>= 3) c\n"),
            "E040", "a bound is read off", ">=");
    refused(&format!("{CROSSING}claim a distance(>= 0, along: up) p\n"),
            "E040", "claim", ">=");
    refused(&format!("{CROSSING}a distance(in: (5, 1), along: up) p\n"),
            "E040", "low end", "in:");
    refused(&format!("{CROSSING}param w: Length hint(1)\na distance(>= w, along: up) p\n"),
            "E040", "a bound may not bind", "a distance(>= w, along: up) p");
    refused(&format!("{CROSSING}a outside b\n"), "E040", "`outside` says", "a outside b");
}

/// The source, the program and the document all keep a bound as the bound it is.
#[test]
fn a_bound_is_kept() {
    for bound in ["a distance(>= 0, along: up) p", "a distance(in: (1, 9), along: up) p",
                  "p distance(<= 6) b"] {
        let src = format!("{CROSSING}{bound}\n");
        let e = solved(&src);
        let c = e.sketch.constraints.iter().find(|c| c.bound.is_some()).expect("bounded");
        let lifted = gcs_core::program::to_program(&e.sketch).text().to_string();
        let said = gcs_core::io::describe(c);
        assert!(said.contains(&bound[bound.find('(').unwrap()..bound.find(')').unwrap()]),
                "{bound}: {said}");
        let again = solved(&lifted);
        let b2 = again.sketch.constraints.iter().find(|c| c.bound.is_some()).expect("lifted");
        assert_eq!((b2.kind, b2.bound), (c.kind, c.bound), "{lifted}");
        let doc = gcs_core::io::from_json(&gcs_core::io::to_json(&e.sketch)).unwrap();
        let b3 = doc.constraints.iter().find(|c| c.bound.is_some()).expect("in the document");
        assert_eq!(b3.bound, c.bound);
    }
}

/// A bound in a set's body is a region, which is slice 2 (#145, F1): a point is put on a set by
/// `coincident`, and `inside` a set is not yet said.
#[test]
fn inside_a_set_is_not_yet_said() {
    refused(&format!("{CROSSING}ball := {{ q | q distance(5) a }}\np inside ball\n"),
            "E040", "set", "ball");
}
