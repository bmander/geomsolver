//! **A plane tangent to a cone at a point** (§6.21, #145): `P tangent(at: m) K` says `P` is the
//! cone's tangent plane at `m`.  Every tangent plane of a cone holds the generator through the
//! point, so where the apex is on `P` the tangency is one condition, not two — stated as one row
//! (`CKind::TangentPlaneCone`), with the apex and `m` put on `P` only where the drawing does not
//! draw them there.  Regular either way: the diagnosis finds no dependency.
use gcs_core::constraints::CKind;
use gcs_core::diagnose::{diagnose, DiagnoseOptions, State};
use gcs_core::io;
use gcs_core::program::Elaborated;
use gcs_core::solve::{solve, SolveOpts};
use gcs_core::space::{cross, dot, sub};

use crate::common::{ent, read, refused};

const SHIPPED: &str = include_str!("../../examples/hypoid_pitch_cones.sv");

/// `SHIPPED` with each of `edits` made, every one of them found.
fn edited(edits: &[(&str, &str)]) -> String {
    edits.iter().fold(SHIPPED.to_string(), |s, (from, to)| {
        assert!(s.contains(from), "no `{from}` in the shipped example");
        s.replacen(from, to, 1)
    })
}

fn solved(src: &str) -> Elaborated {
    let mut e = read(src);
    let r = solve(&mut e.sketch, SolveOpts::default());
    assert!(r.success, "{}", r.message);
    e
}

fn kinds(e: &Elaborated) -> Vec<CKind> {
    e.sketch.user_constraints().iter().map(|c| c.kind).collect()
}

/// Regular and determined: no dependency only the numbers can see, nothing implied or over.
fn regular(e: &mut Elaborated) {
    let d = diagnose(&mut e.sketch, DiagnoseOptions::default());
    assert_eq!((d.dof, d.status), (0, State::Well));
    assert_eq!(d.geometric_dependency, 0);
    assert!(d.implied.is_empty() && d.over.is_empty(), "{:?} {:?}", d.implied, d.over);
}

/// The plane touches the cone about `apex → tip` at `m`: square to the generator there and to
/// the direction round the axis.
fn touches(e: &Elaborated, plane: &str, apex: &str, tip: &str, m: &str) {
    let sk = &e.sketch;
    let at = |n: &str| sk.world_point(ent(e, n).i());
    let (a, b, m) = (at(apex), at(tip), at(m));
    let n = sk.basis(ent(e, plane).i()).normal();
    let (g, d) = (sub(m, a), sub(b, a));
    let round = cross(d, g);
    let unit = |v: [f64; 3]| dot(v, v).sqrt();
    assert!(dot(n, g).abs() < 1e-9 * unit(g), "the plane holds the generator");
    assert!(dot(n, round).abs() < 1e-9 * unit(round), "and the direction round the axis");
}

/// **The hypoid's pitch cones with no view** (#145): the shipped example draws the gear's axis
/// in its axial plane `G`, square to the pitch plane along the generator; saying instead that
/// the pitch plane touches the gear's cone at M, with the axis in space, is the same pair —
/// solved to the same apexes and axes — and as determined, with no dependency.
#[test]
fn the_pitch_plane_touches_the_gear_cone_with_no_view() {
    let without = edited(&[
        ("G := plane(u: std.z, v: std.y)\nfix(origin == (0, 0, 0)) G\n", ""),
        (
            "gax := line(hint((110.85, 0)), hint((50.85, 103.9))) in G",
            "ga := point hint((0, 0, 110.85))\ngb := point hint((0, 103.9, 50.85))\n\
             gax := line(ga, gb)\nfix(x == 0) ga",
        ),
        ("gax.p1 coincident std.front\n", ""),
        ("M coincident gc\n", "M coincident gc\nstd.front tangent(at: M) gc\n"),
    ]);
    let shipped = solved(SHIPPED);
    let mut touched = solved(&without);
    assert_eq!(kinds(&touched).iter().filter(|k| **k == CKind::TangentPlaneCone).count(), 1);
    let pairs = [("gax.p1", "ga"), ("gax.p2", "gb"), ("pax.p1", "pax.p1"), ("pax.p2", "pax.p2"),
                 ("M", "M")];
    for (n, m) in pairs {
        let (a, b) = (
            shipped.sketch.world_point(ent(&shipped, n).i()),
            touched.sketch.world_point(ent(&touched, m).i()),
        );
        assert!(a.iter().zip(&b).all(|(x, y)| (x - y).abs() < 1e-9), "{n}: {a:?} {b:?}");
    }
    touches(&touched, "std.front", "ga", "gb", "M");
    regular(&mut touched);
    let said: Vec<String> = touched.sketch.user_constraints().iter()
        .map(|c| io::describe_with(c, &|x| touched.map.name_of(x).cloned())).collect();
    assert!(said.contains(&"std.front tangent(at: M) gc".to_string()), "{said:?}");
}

/// A cone over the top plane with its apex `O`, touched at `M` — `apex` how the apex is said.
fn over_top(apex: &str) -> String {
    format!(
        "unit mm\nuse std\n{apex}\nM := point in std.top\nfix((40, 0)) M\n\
         T := point hint((20, 0, -30))\ngax := line(O, T)\ndistance(60) gax\n\
         gc := std.Cone(gax, half: 30deg)\nM coincident gc\nstd.top tangent(at: M) gc\n"
    )
}

/// The apex and the point touched at, drawn in the plane, are on it already: the tangency is the
/// one row.  An apex standing in space is put on the plane by a row beside it — the same cone.
#[test]
fn what_the_drawing_draws_on_the_plane_is_not_said_again() {
    let mut drawn = solved(&over_top("O := point in std.top\nfix((0, 0)) O"));
    let mut standing = solved(&over_top("O := point hint((0, 0, 5))\nfix(x == 0, y == 0) O"));
    assert_eq!(kinds(&drawn).iter().filter(|k| **k == CKind::PointOnPlane).count(), 0);
    assert_eq!(kinds(&standing).iter().filter(|k| **k == CKind::PointOnPlane).count(), 1);
    for e in [&drawn, &standing] {
        touches(e, "std.top", "O", "T", "M");
    }
    let tip = |e: &Elaborated| e.sketch.world_point(ent(e, "T").i());
    let (a, b) = (tip(&drawn), tip(&standing));
    assert!(a.iter().zip(&b).all(|(x, y)| (x - y).abs() < 1e-9), "{a:?} {b:?}");
    regular(&mut drawn);
    regular(&mut standing);
}

/// What a plane touches is a cone, at a point the word names, and only a set's use says so.
#[test]
fn a_plane_touches_a_cone_at_a_point() {
    let base = "unit mm\nuse std\nO := point in std.top\nfix((0, 0)) O\n\
                M := point in std.top\nfix((40, 0)) M\nT := point hint((20, 0, -30))\n\
                gax := line(O, T)\n\
                gc := std.Cone(gax, half: 30deg)\nball := std.Sphere(O, r: 40)\n";
    let src = |s: &str| format!("{base}{s}\n");
    refused(&src("std.top tangent(at: M) ball"), "E040", "to a cone", "tangent(at: M)");
    refused(&src("std.top tangent(at: M) gax"), "E040", "relates a plane to a set",
            "std.top tangent(at: M) gax");
    refused(&src("gax tangent(at: M) gc"), "E040", "touched at a point by a plane",
            "tangent(at: M)");
}
