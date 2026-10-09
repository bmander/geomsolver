//! **A plane tangent to a set at a point** (§6.21, #148): `P tangent(at: m) S` says `P` is the
//! set's tangent plane at `m` — the body differentiated at `m` along each of `P`'s axes, whatever
//! the set.  Nothing here knows a shape: where the drawing already says what the body says along
//! the plane (a cone's apex drawn on it, a cylinder's axis parallel to it), one derivative is what
//! the other says, and the system's rank finds that — `Diagnosis::expected`, said nowhere.
use gcs_core::diagnose::{diagnose, DiagnoseOptions, Diagnosis, State};
use gcs_core::io;
use gcs_core::model::Toward;
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

/// The rows a plane's tangency states as derivatives along the plane's axes.
fn along_axes(e: &Elaborated) -> usize {
    let sk = &e.sketch;
    sk.user_constraints().iter()
        .filter(|c| c.along.is_some_and(|d| matches!(sk.duals[d].toward, Toward::Axis(_))))
        .count()
}

/// Determined, with nothing to say: no dependency the author wrote, nothing over.  What the
/// tangency makes dependent of its own rows comes back for the caller to read.
fn determined(e: &mut Elaborated) -> Diagnosis {
    let d = diagnose(&mut e.sketch, DiagnoseOptions::default());
    assert_eq!((d.dof, d.status), (0, State::Well), "{:?}", d.warnings);
    assert!(d.implied.is_empty() && d.over.is_empty(), "{:?} {:?}", d.implied, d.over);
    assert!(d.warnings.is_empty(), "{:?}", d.warnings);
    d
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
/// in its axial plane `G`, square to the pitch plane along the generator; saying instead that the
/// pitch plane touches the gear's cone at M, with the axis in space, is the same pair — solved
/// to the same apexes and axes — and as determined.  The apex is put on the plane by the
/// tangency, which no other statement repeats: no dependency at all.
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
    assert_eq!(along_axes(&touched), 2);
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
    assert!(determined(&mut touched).expected.is_empty());
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

/// **The apex drawn in the plane** is on it already, and a cone's body is stationary along its
/// generator: the tangency's two derivatives along the plane are one and its multiple there.
/// The rank finds it and says nothing — expected, not implied, not over.  The apex standing in
/// space is put on the plane by the tangency: no dependency.  The same cone either way.
#[test]
fn what_the_drawing_says_along_the_plane_is_found_by_the_rank() {
    let mut drawn = solved(&over_top("O := point in std.top\nfix((0, 0)) O"));
    let mut standing = solved(&over_top("O := point hint((0, 0, 5))\nfix(x == 0, y == 0) O"));
    for e in [&drawn, &standing] {
        assert_eq!(along_axes(e), 2);
        touches(e, "std.top", "O", "T", "M");
    }
    let tip = |e: &Elaborated| e.sketch.world_point(ent(e, "T").i());
    let (a, b) = (tip(&drawn), tip(&standing));
    assert!(a.iter().zip(&b).all(|(x, y)| (x - y).abs() < 1e-9), "{a:?} {b:?}");
    assert_eq!(determined(&mut drawn).expected.len(), 1);
    assert!(determined(&mut standing).expected.is_empty());
}

/// A sphere has no direction its body is stationary along: a plane tangent to one states two
/// conditions, independent — the centre straight off the plane from the point.
#[test]
fn a_plane_touches_a_sphere() {
    let mut e = solved("unit mm\nuse std\nM := point in std.top\nfix((40, 0)) M\n\
        C := point hint((35, 5, 18))\nball := std.Sphere(C, r: 20)\nM coincident ball\n\
        std.top tangent(at: M) ball\n");
    let c = e.sketch.world_point(ent(&e, "C").i());
    assert!((c[0] - 40.0).abs() < 1e-9 && c[1].abs() < 1e-9 && (c[2] - 20.0).abs() < 1e-9, "{c:?}");
    assert!(determined(&mut e).expected.is_empty());
}

/// A cylinder's body is stationary along its axis, and an axis drawn parallel to the plane makes
/// one of the two derivatives what the other says — found by the rank, as for the cone.
#[test]
fn a_plane_touches_a_cylinder_lying_along_it() {
    let mut e = solved("unit mm\nuse std\nux := axis\n\
        fix(dir == (1, 0, 0), origin == (0, 0, 20)) ux\nvy := axis\n\
        fix(dir == (0, 1, 0), origin == (0, 0, 20)) vy\nup := plane(u: ux, v: vy)\n\
        A := point in up\nfix((0, 0)) A\n\
        B := point hint((50, 10)) in up\nax := line(A, B)\ndistance(60) ax\n\
        M := point in std.top\nfix((40, 0)) M\nshaft := std.Cylinder(ax, r: hint(15))\n\
        M coincident shaft\nstd.top tangent(at: M) shaft\n");
    let b = e.sketch.world_point(ent(&e, "B").i());
    assert!(b[1].abs() < 1e-9 && (b[0] - 60.0).abs() < 1e-9, "the axis right above M: {b:?}");
    assert_eq!(determined(&mut e).expected.len(), 1);
}

/// What touches a set at a point is a plane, at a point the word names.
#[test]
fn a_plane_touches_a_set_at_a_point() {
    let base = "unit mm\nuse std\nO := point in std.top\nfix((0, 0)) O\n\
                M := point in std.top\nfix((40, 0)) M\nT := point hint((20, 0, -30))\n\
                gax := line(O, T)\ngc := std.Cone(gax, half: 30deg)\n";
    let src = |s: &str| format!("{base}{s}\n");
    refused(&src("gax tangent(at: M) gc"), "E040", "touched at a point by a plane",
            "tangent(at: M)");
    refused(&src("std.top tangent gc"), "E040", "which the word names", "tangent");
    refused(&src("std.top tangent(at: M) gax"), "E040", "does not relate a plane to a line",
            "tangent(at: M)");
}
