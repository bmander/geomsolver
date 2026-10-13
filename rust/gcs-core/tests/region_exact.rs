//! **Every region an exact B-rep** (#238): a region whose sum of distances is a conic about the
//! line through its points is folded into its meridian (the egg an ellipse turned, the Voronoi
//! cell a plane, a hyperboloid's lobe a hyperbola), and one whose terms share no axis is built as
//! the common of its terms' primitives bounded past its support (the bicylinder). Each is held to
//! the region read point by point before it stands, and here to its closed form.
use gcs_core::brep::{export, props, sweep::Say};
use gcs_core::brep::topo::Brep;
use gcs_core::program::Elaborated;
use gcs_core::solve::{solve, SolveOpts};
use std::f64::consts::PI;

use crate::common::{ent, read};

fn solved(src: &str) -> Elaborated {
    let mut e = read(src);
    let r = solve(&mut e.sketch, SolveOpts::default());
    assert!(r.success, "{}\n{src}", r.message);
    e
}

/// The origin, and lines from it along x and z.
const AXES: &str = "unit mm\nuse std\nc := point hint((0, 0, 0))\nfix((0, 0, 0)) c\n\
                    xe := point hint((1, 0, 0))\nfix((1, 0, 0)) xe\nze := point hint((0, 0, 1))\n\
                    fix((0, 0, 1)) ze\nxl := line(c, xe)\nzl := line(c, ze)\n";

/// Solid `name` of `e` built exactly, written as STEP and as STL by this kernel (each read back
/// and checked as `solventc` checks it), and its B-rep — never the field's surface.
fn exported(e: &Elaborated, name: &str) -> Brep {
    let s = ent(e, name).i();
    let say = Say { stage: &|_: &str| {}, mark: &|_| {} };
    let exact = export::exact(&e.sketch, s, None, None, &say).unwrap_or_else(|r| panic!("{name}: {r:?}"));
    export::step(&exact, name, None, &say).unwrap();
    let stl = export::stl(&e.sketch, s, &exact, None, &say).unwrap();
    gcs_core::mesh::stl_shells(&stl).unwrap();
    assert!(e.sketch.exact_solid(s).is_ok(), "{name}: the app's exact solid");
    exact.solid
}

fn close(got: f64, want: f64, tol: f64) {
    assert!((got - want).abs() <= tol * want, "{got} against {want}");
}

/// An ellipsoid of revolution by its foci, 3 either side of the origin along x, the sum of the
/// distances at most 10: its meridian a half ellipse of semi-axes 5 and 4, two conic arcs turned.
#[test]
fn an_ellipsoid_by_its_foci_is_an_ellipse_turned() {
    let e = solved(&format!("{AXES}f1 := point hint((3, 0, 0))\nfix((3, 0, 0)) f1\n\
                             f2 := point hint((-3, 0, 0))\nfix((-3, 0, 0)) f2\n\
                             egg := {{ p | distance(p, f1) + distance(p, f2) <= 10mm }}\n\
                             s := solid(egg)\n"));
    let s = ent(&e, "s").i();
    assert!(!gcs_core::solid::region::in_general(&e.sketch, s));
    let m = gcs_core::solid::region::meridian(&e.sketch, s).unwrap();
    assert!(m.region.loops.iter().flatten()
        .any(|st| matches!(st.seg, gcs_core::brep::planar::Seg::Conic { .. })));
    let b = exported(&e, "s");
    close(props::volume(&b), 4.0 / 3.0 * PI * 5.0 * 4.0 * 4.0, 1e-9);
    // the bound turned round, the ellipsoid taken out of a ball about it
    let e = solved(&format!("{AXES}f1 := point hint((3, 0, 0))\nfix((3, 0, 0)) f1\n\
                             f2 := point hint((-3, 0, 0))\nfix((-3, 0, 0)) f2\n\
                             ball := std.Sphere(c, r: 8)\n\
                             shell := {{ p | p inside ball; distance(p, f1) + distance(p, f2) >= 10mm }}\n\
                             s := solid(shell)\n"));
    let b = exported(&e, "s");
    close(props::volume(&b), 4.0 / 3.0 * PI * (512.0 - 80.0), 1e-9);
}

/// A Voronoi cell within a ball: the points nearer one centre than the other, the half of the ball
/// on its side of the plane midway between them.
#[test]
fn a_voronoi_cell_is_a_ball_halved_by_a_plane() {
    let e = solved(&format!("{AXES}a := point hint((2, 0, 0))\nfix((2, 0, 0)) a\n\
                             b := point hint((-2, 0, 0))\nfix((-2, 0, 0)) b\n\
                             ball := std.Sphere(c, r: 5)\n\
                             near_a := {{ p | p inside ball; distance(p, a) <= distance(p, b) }}\n\
                             near_b := {{ p | p inside ball; distance(p, a) >= distance(p, b) }}\n\
                             sa := solid(near_a)\nsb := solid(near_b)\n"));
    for (name, x) in [("sa", 1.0), ("sb", -1.0)] {
        let s = ent(&e, name).i();
        assert!(!gcs_core::solid::region::in_general(&e.sketch, s), "{name}: {:?}",
                gcs_core::solid::region::meridian(&e.sketch, s).err());
        let b = exported(&e, name);
        close(props::volume(&b), 2.0 / 3.0 * PI * 125.0, 1e-9);
        let field = gcs_core::solid::MaterialField::read(&e.sketch, s, 1e-10).unwrap();
        assert!(field.side([x, 0.5, 0.5]) < 0.0 && field.side([-x, 0.5, 0.5]) > 0.0, "{name}");
        // on the axis past its cap the field reads how far off it is: the sector its quarter arc
        // bounds is bounded by no side along the axis (a meridian lies where `r ≥ 0`)
        for (past, off) in [(6.0, 1.0), (7.0, 2.0)] {
            let v = field.side([x * past, 0.0, 0.0]);
            assert!((v - off).abs() < 1e-9, "{name}: {v} at {}", x * past);
        }
    }
}

/// A hyperboloid's lobe: the points nearer the focus at 3 than the one at −3 by at least 2 — a
/// sheet of semi-axes 1 and √8 about the first — within a ball of 6 about the origin.  Its volume
/// in two quadratures closed: the lobe's `π ∫ 8 (z² − 1) dz` from its vertex to where it meets the
/// sphere, the ball's cap `π ∫ (36 − z²) dz` past that.
#[test]
fn a_hyperboloids_lobe_is_a_hyperbola_turned() {
    let e = solved(&format!("{AXES}f1 := point hint((3, 0, 0))\nfix((3, 0, 0)) f1\n\
                             f2 := point hint((-3, 0, 0))\nfix((-3, 0, 0)) f2\n\
                             ball := std.Sphere(c, r: 6)\n\
                             lobe := {{ p | p inside ball; distance(p, f2) - distance(p, f1) >= 2mm }}\n\
                             s := solid(lobe)\n"));
    let s = ent(&e, "s").i();
    assert!(!gcs_core::solid::region::in_general(&e.sketch, s), "{:?}",
            gcs_core::solid::region::meridian(&e.sketch, s).err());
    let b = exported(&e, "s");
    let meet = (44.0f64 / 9.0).sqrt();
    let lobe = PI * 8.0 * ((meet.powi(3) / 3.0 - meet) - (1.0 / 3.0 - 1.0));
    let cap = PI * ((36.0 * 6.0 - 72.0) - (36.0 * meet - meet.powi(3) / 3.0));
    close(props::volume(&b), lobe + cap, 1e-9);
}

/// A sum of distances from points off one line, or alike in weight but more than two, is no conic
/// about an axis: still read point by point.
#[test]
fn a_sum_no_conic_folds_stays_the_fields() {
    let e = solved(&format!("{AXES}f1 := point hint((3, 0, 0))\nfix((3, 0, 0)) f1\n\
                             f2 := point hint((-3, 0, 0))\nfix((-3, 0, 0)) f2\n\
                             f3 := point hint((0, 3, 0))\nfix((0, 3, 0)) f3\n\
                             three := {{ p | distance(p, f1) + distance(p, f2) + distance(p, f3) <= 15mm }}\n\
                             s := solid(three)\n\
                             oval := {{ p | distance(p, f1) + 2 * distance(p, f2) <= 15mm }}\n\
                             t := solid(oval)\n"));
    for name in ["s", "t"] {
        assert!(gcs_core::solid::region::in_general(&e.sketch, ent(&e, name).i()), "{name}");
    }
}

/// Steinmetz's bicylinder: two cylinders of radius 2 crossing square, neither bounded but by the
/// other — no axis they share, so built as the common of the two, each long enough to pass the
/// box the region lies in.  Its volume `16 r³ / 3`; and a ball cored by a cylinder off its centre,
/// the ball less the cylinder.
#[test]
fn two_crossing_cylinders_are_their_common() {
    let e = solved(&format!("{AXES}a := std.Cylinder(xl, r: 2)\nb := std.Cylinder(zl, r: 2)\n\
                             both := {{ p | p inside a; p inside b }}\nbi := solid(both)\n\
                             o := point hint((2, 0, 0))\nfix((2, 0, 0)) o\n\
                             ball := std.Sphere(o, r: 5)\nbore := std.Cylinder(zl, r: 1)\n\
                             bored := {{ p | p inside ball; p outside bore }}\ncored := solid(bored)\n"));
    let bi = ent(&e, "bi").i();
    assert!(gcs_core::solid::region::in_general(&e.sketch, bi));
    assert!(matches!(gcs_core::solid::bounded::built(&e.sketch, bi), Some(Ok(_))));
    let b = exported(&e, "bi");
    close(props::volume(&b), 16.0 * 8.0 / 3.0, 1e-9);
    // each face named by the set it came from
    let mut names: Vec<_> = b.faces.iter().map(|f| gcs_core::brep::recipe::named_parts(&f.name).1.to_string())
        .collect();
    names.sort();
    names.dedup();
    assert_eq!(names, ["a", "b"]);
    // the ball less the bore through it, off its centre by 2: `4/3 π (R² − ρ²)^{3/2}` taken by a
    // bore through the centre, here the ball less the cylinder's slice of it by quadrature
    let b = exported(&e, "cored");
    let (r, rho, off) = (5.0f64, 1.0f64, 2.0f64);
    let n = 20000;
    // the bore's disc at distance `s` from the ball's centre (`s² = off² + t² + 2 off t cos φ`),
    // each of its points' chord through the ball `2 √(R² − s²)`
    let taken: f64 = (0..n).map(|i| {
        let t = rho * (i as f64 + 0.5) / n as f64;
        let m = 400;
        (0..m).map(|j| {
            let phi = 2.0 * PI * (j as f64 + 0.5) / m as f64;
            let s2 = off * off + t * t + 2.0 * off * t * phi.cos();
            2.0 * (r * r - s2).max(0.0).sqrt()
        }).sum::<f64>() * 2.0 * PI / m as f64 * t * rho / n as f64
    }).sum();
    close(props::volume(&b), 4.0 / 3.0 * PI * r.powi(3) - taken, 1e-7);
}

/// `rounds/regions.sv`: the egg, the bicylinder and the cell, each built exactly — none is the
/// field's.
#[test]
fn every_region_of_the_example_is_exact() {
    let e = solved(include_str!("../../examples/rounds/regions.sv"));
    let (a, b) = (5.0f64, 4.0f64);
    for (name, want) in [("egg", 4.0 / 3.0 * PI * a * b * b), ("bicylinder", 16.0 * 27.0 / 3.0),
                         ("cell", 2.0 / 3.0 * PI * 125.0)] {
        let s = ent(&e, name).i();
        assert!(!gcs_core::solid::region::in_general(&e.sketch, s)
                || matches!(gcs_core::solid::bounded::built(&e.sketch, s), Some(Ok(_))), "{name}");
        close(props::volume(&exported(&e, name)), want, 1e-9);
    }
}
