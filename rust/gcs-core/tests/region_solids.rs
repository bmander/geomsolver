//! **A region is a solid** (§6.9, §6.21; #145 F2/F3): `solid(R)` is the points inside the set
//! `R` — its body applied to a hidden probe, each bound it puts there a term — built as one
//! revolution of its meridian region: exactly, as a field, and as facets.  Held to closed forms.
use gcs_core::mesh;
use gcs_core::program::Elaborated;
use gcs_core::solid::MaterialField;
use gcs_core::solve::{solve, SolveOpts};
use std::f64::consts::PI;

use crate::common::{ent, read, refused};

fn solved(src: &str) -> Elaborated {
    let mut e = read(src);
    let r = solve(&mut e.sketch, SolveOpts::default());
    assert!(r.success, "{}\n{src}", r.message);
    e
}

/// The three readings of the solid the document calls `name`: its exact volume, its faceted
/// volume, and its field's sign at each of `points` (negative inside).
fn readings(e: &Elaborated, name: &str, points: &[[f64; 3]]) -> (f64, f64, Vec<f64>) {
    let s = ent(e, name).i();
    let x = e.sketch.exact_solid(s).unwrap_or_else(|m| panic!("`{name}`: {m}"));
    let exact = gcs_core::brep::props::volume(&x.brep);
    let faceted = mesh::volume(&e.sketch.solid_boundary(s, 0.0));
    let field =
        MaterialField::read(&e.sketch, s, 1e-10).unwrap_or_else(|m| panic!("`{name}`: {m}"));
    (exact, faceted, points.iter().map(|&p| field.side(p)).collect())
}

/// Held to `want` exactly, to its facets' sagitta, and in and out where `inside` and `outside`
/// say.
fn holds(e: &Elaborated, name: &str, want: f64, inside: &[[f64; 3]], outside: &[[f64; 3]]) {
    let points: Vec<[f64; 3]> = inside.iter().chain(outside).copied().collect();
    let (exact, faceted, sides) = readings(e, name, &points);
    assert!((exact - want).abs() <= 1e-9 * want, "`{name}` exactly: {exact} against {want}");
    assert!((faceted - want).abs() <= 2e-2 * want, "`{name}` faceted: {faceted} against {want}");
    for (k, (p, side)) in points.iter().zip(sides).enumerate() {
        assert_eq!(side < 0.0, k < inside.len(), "`{name}` at {p:?}: {side}");
    }
}

/// A point held at the origin, the axis up from it, and a plane at height 10 square to it.
const FRAME: &str = "unit mm\nuse std\nc := point hint((0, 0, 0))\nfix((0, 0, 0)) c\n\
                     t := point hint((0, 0, 1))\nfix((0, 0, 1)) t\nup := line(c, t)\n\
                     lu := axis hint(dir: (1, 0, 0))\nlv := axis hint(dir: (0, 1, 0))\n\
                     fix(dir == (1, 0, 0), origin == (0, 0, 10)) lu\n\
                     fix(dir == (0, 1, 0), origin == (0, 0, 10)) lv\nlid := plane(u: lu, v: lv)\n";

#[test]
fn a_ball_and_a_shell() {
    let e = solved(&format!("{FRAME}big := std.Sphere(c, r: 5)\nsmall := std.Sphere(c, r: 3)\n\
                             ball := solid(big)\nskin := {{ p | p inside big; p outside small }}\n\
                             shell := solid(skin)\n"));
    let ball = 4.0 / 3.0 * PI * 125.0;
    holds(&e, "ball", ball, &[[0.0, 0.0, 0.0], [4.0, 0.0, 0.0]], &[[6.0, 0.0, 0.0]]);
    holds(&e, "shell", 4.0 / 3.0 * PI * (125.0 - 27.0), &[[0.0, 4.0, 0.0]],
          &[[0.0, 0.0, 0.0], [0.0, 0.0, 6.0]]);
}

/// A cylinder between two planes, and a cone under one: their closed forms.
#[test]
fn a_capped_cylinder_and_a_cone() {
    let e = solved(&format!("{FRAME}tube := std.Cylinder(up, r: 2)\n\
                             funnel := std.Cone(up, half: 30deg)\n\
                             can := {{ p | p inside tube; p outside std.top; p inside lid }}\n\
                             cup := {{ p | p inside funnel; p inside lid }}\n\
                             tin := solid(can)\ncone := solid(cup)\n"));
    let outside = [[3.0, 0.0, 5.0], [0.0, 0.0, 11.0], [0.0, 0.0, -1.0]];
    holds(&e, "tin", PI * 4.0 * 10.0, &[[1.0, 0.0, 5.0]], &outside);
    let r = 10.0 * (30f64).to_radians().tan();
    let outside = [[4.0, 0.0, 5.0], [0.0, 0.0, -1.0]];
    holds(&e, "cone", PI * r * r * 10.0 / 3.0, &[[0.0, 0.0, 5.0]], &outside);
}

/// A cone met with a ball about its apex: a spherical sector, `2/3 π r³ (1 - cos h)`.
#[test]
fn a_spherical_sector() {
    let e = solved(&format!("{FRAME}funnel := std.Cone(up, half: 30deg)\n\
                             big := std.Sphere(c, r: 5)\n\
                             sector := {{ p | p inside funnel; p inside big }}\n\
                             piece := solid(sector)\n"));
    let want = 2.0 / 3.0 * PI * 125.0 * (1.0 - (30f64).to_radians().cos());
    holds(&e, "piece", want, &[[0.0, 0.0, 3.0]], &[[0.5, 0.0, 6.0], [3.0, 0.0, 1.0]]);
}

/// A region solid is a solid like any: a body over it takes cuts.
#[test]
fn a_region_solid_is_material() {
    let e = solved(&format!("{FRAME}big := std.Sphere(c, r: 5)\nsmall := std.Sphere(c, r: 3)\n\
                             ball := solid(big)\ncore := solid(small)\nbody := solid(ball)\n\
                             core cut body\n"));
    holds(&e, "body", 4.0 / 3.0 * PI * (125.0 - 27.0), &[[4.0, 0.0, 0.0]], &[[0.0, 0.0, 0.0]]);
}

/// What a region solid may be made of: judged where it is evaluated, on the solved drawing — a
/// cylinder without ends, a ball off the axis.
#[test]
fn a_region_solid_is_bounded_on_one_axis() {
    let refused_as = |src: String, name: &str, why: &str| {
        let e = solved(&src);
        let m = gcs_core::solid::validate(&e.sketch, ent(&e, name).i()).expect_err(name);
        assert!(m.contains(why), "{m}");
    };
    let rod = format!("{FRAME}tube := std.Cylinder(up, r: 2)\nrod := solid(tube)\n");
    refused_as(rod, "rod", "no end");
    refused_as(format!("{FRAME}d := point hint((1, 0, 0))\nfix((1, 0, 0)) d\n\
                        tube := std.Cylinder(up, r: 2)\naway := std.Sphere(d, r: 1)\n\
                        odd := {{ p | p inside tube; p inside away }}\nbad := solid(odd)\n"),
               "bad", "off the region's axis");
    let rail = format!("{FRAME}rail := {{ p | p coincident up; p distance(5) c }}\n\
                        r := solid(rail)\n");
    refused(&rail, "E040", "bounds nothing", "r := solid(rail)");
}

/// Two sets with declarations of their own in one body (two cones, each drawing the line from its
/// apex to the point): each use is applied under a prefix of its own, or the second cone's line
/// is named as the first's and never built.
#[test]
fn two_cones_in_one_region() {
    let e = solved(&format!("{FRAME}wide := std.Cone(up, half: 30deg)\n\
                             narrow := std.Cone(up, half: 15deg)\n\
                             shade := {{ p | p inside wide; p outside narrow; p inside lid }}\n\
                             lamp := solid(shade)\n"));
    let (t30, t15) = ((30f64).to_radians().tan(), (15f64).to_radians().tan());
    let want = PI * 1000.0 * (t30 * t30 - t15 * t15) / 3.0;
    holds(&e, "lamp", want, &[[4.0, 0.0, 9.0]], &[[0.5, 0.0, 9.0], [7.0, 0.0, 9.0]]);
}
