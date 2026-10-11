//! **A point's distance from a set, and offsets** (§6.21, #145 F4): `p distance(d, side:
//! outside) S` is the signed distance from `S`'s surface, `std.Offset(S, d)` the set `d` outside
//! it, `p inside(d) S` / `p outside(d) S` that side of it grown by `d`.  A distance's set moves
//! its number, a region solid's cone is its exact offset — a cone, its apex rounded by a ball
//! where it grows — and a drawn point stands at its foot on the set.  Held to closed forms.
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

/// A point held at the origin, the axis up from it, a plane at height 10 square to it, a cone
/// of half 30° opening up from the origin and a cylinder of radius 6 about the axis.
const FRAME: &str = "unit mm\nuse std\nc := point hint((0, 0, 0))\nfix((0, 0, 0)) c\n\
                     t := point hint((0, 0, 1))\nfix((0, 0, 1)) t\nup := line(c, t)\n\
                     lu := axis hint(dir: (1, 0, 0))\nlv := axis hint(dir: (0, 1, 0))\n\
                     fix(dir == (1, 0, 0), origin == (0, 0, 10)) lu\n\
                     fix(dir == (0, 1, 0), origin == (0, 0, 10)) lv\nlid := plane(u: lu, v: lv)\n\
                     funnel := std.Cone(up, half: 30deg)\ntube := std.Cylinder(up, r: 6)\n";

/// `name`'s exact volume against `want`, and its field's sign at `inside` and `outside`.
fn holds(e: &Elaborated, name: &str, want: f64, inside: &[[f64; 3]], outside: &[[f64; 3]]) {
    let s = ent(e, name).i();
    let x = e.sketch.exact_solid(s).unwrap_or_else(|m| panic!("`{name}`: {m}"));
    let exact = gcs_core::brep::props::volume(&x.brep);
    assert!((exact - want).abs() <= 1e-9 * want, "`{name}` exactly: {exact} against {want}");
    let field =
        MaterialField::read(&e.sketch, s, 1e-10).unwrap_or_else(|m| panic!("`{name}`: {m}"));
    for p in inside {
        assert!(field.side(*p) < 0.0, "`{name}` at {p:?}: {}", field.side(*p));
    }
    for p in outside {
        assert!(field.side(*p) > 0.0, "`{name}` at {p:?}: {}", field.side(*p));
    }
}

fn tan30() -> f64 {
    (30f64).to_radians().tan()
}

/// A cone grown by 1 stands 1 / sin 30° = 2 further back: between the planes through its apex
/// and the lid, the frustum of the cone from -2.
#[test]
fn a_cone_grown_is_the_cone_further_back() {
    let e = solved(&format!("{FRAME}cup := {{ p | p inside(1) funnel; p outside std.top; \
                             p inside lid }}\nglass := solid(cup)\n"));
    let want = PI * tan30() * tan30() / 3.0 * (12f64.powi(3) - 2f64.powi(3));
    // at height 5 the surface is 5 tan 30° out, and the offset 1 / cos 30° further
    let edge = 5.0 * tan30() + 1.0 / (30f64).to_radians().cos();
    holds(&e, "glass", want, &[[edge - 0.05, 0.0, 5.0], [0.5, 0.0, 0.5]],
          &[[edge + 0.05, 0.0, 5.0], [0.5, 0.0, -0.5]]);
}

/// Grown, the apex is rounded: within 1 of the apex is within 1 of the cone, so the ball of
/// radius 1 about it is inside, whole; below it, where the nearest point is the apex, the
/// grown cone is that ball's.
#[test]
fn a_grown_cone_is_rounded_at_its_apex() {
    let e = solved(&format!("{FRAME}small := std.Sphere(c, r: 1)\n\
                             bead := {{ p | p inside(1) funnel; p inside small }}\n\
                             glow := {{ p | p inside(1) funnel; p inside lid }}\n\
                             b := solid(bead)\ng := solid(glow)\n"));
    holds(&e, "b", 4.0 / 3.0 * PI, &[[0.3, 0.0, -0.8]], &[[0.3, 0.0, -1.1]]);
    let field = MaterialField::read(&e.sketch, ent(&e, "g").i(), 1e-10).unwrap();
    // nearest the apex, below the generator's square: in within 1 of it, out beyond (and off
    // the axis, where a revolution's field reads no further than to it)
    for (p, inside) in [([0.2, 0.0, -0.95], true), ([0.2, 0.0, -1.05], false),
                        ([0.6, 0.0, -0.7], true), ([0.7, 0.0, -0.8], false)] {
        assert_eq!(field.side(p) < 0.0, inside, "{p:?}: {}", field.side(p));
    }
}

/// `outside(1)` is within the cone's outside grown by 1: outside the cone shrunk by 1, whose
/// apex stands 2 further along — so a cylinder less it is the cylinder less that cone.
#[test]
fn a_cone_shrunk_is_the_cone_further_along() {
    let e = solved(&format!("{FRAME}sleeve := {{ p | p inside tube; p outside std.top; \
                             p inside lid; p outside(1) funnel }}\ns := solid(sleeve)\n"));
    let want = PI * 36.0 * 10.0 - PI * tan30() * tan30() / 3.0 * 8f64.powi(3);
    holds(&e, "s", want, &[[0.3, 0.0, 1.5], [3.0, 0.0, 5.0]], &[[0.1, 0.0, 2.5], [1.0, 0.0, 9.0]]);
}

/// A cone wider than square is all but the cone the other way: grown, that one shrinks.
#[test]
fn a_wide_cone_grown() {
    let e = solved(&format!("{FRAME}wide := std.Cone(up, half: 120deg)\n\
                             big := std.Sphere(c, r: 5)\n\
                             cap := {{ p | p inside(1) wide; p inside big }}\nw := solid(cap)\n"));
    let field = MaterialField::read(&e.sketch, ent(&e, "w").i(), 1e-10).unwrap();
    // the other way's apex 1 / sin 60° below
    let apex = -1.0 / (60f64).to_radians().sin();
    for (p, inside) in [([0.1, 0.0, apex + 0.05], true), ([0.1, 0.0, apex - 0.3], false),
                        ([0.5, 0.0, 4.0], true), ([3.0, 0.0, -1.0], true)] {
        assert_eq!(field.side(p) < 0.0, inside, "{p:?}: {}", field.side(p));
    }
}

/// The set offset is a set (`std.Offset`): inside it is inside(d), and its number may be an
/// unknown the drawing solves.
#[test]
fn an_offset_is_a_set() {
    let e = solved(&format!("{FRAME}tip := std.Offset(funnel, d: 1)\n\
                             cup := {{ p | p inside tip; p outside std.top; p inside lid }}\n\
                             glass := solid(cup)\n"));
    let want = PI * tan30() * tan30() / 3.0 * (12f64.powi(3) - 2f64.powi(3));
    holds(&e, "glass", want, &[[0.5, 0.0, 0.5]], &[[0.5, 0.0, -0.5]]);
    // grown by an unknown a held point sets: 2, so the cone from -4
    let e = solved(&format!("{FRAME}param g: Length hint(1mm)\nq := point hint((0, 0, -2))\n\
                             fix((0, 0, -2)) q\nq distance(g) c\n\
                             cup := {{ p | p inside(g) funnel; p outside std.top; p inside lid }}\n\
                             glass := solid(cup)\n"));
    let want = PI * tan30() * tan30() / 3.0 * (14f64.powi(3) - 4f64.powi(3));
    holds(&e, "glass", want, &[[0.5, 0.0, 0.5]], &[[5.0, 0.0, 1.0]]);
}

/// A distance's set moves its number: a ball and a cylinder grown, a plane's side moved.
#[test]
fn a_distance_set_moves_its_number() {
    let e = solved(&format!("{FRAME}big := std.Sphere(c, r: 3)\nsmall := std.Sphere(c, r: 2)\n\
                             grown := {{ p | p inside(2) big }}\n\
                             skin := {{ p | p inside big; p outside(1) small }}\n\
                             can := {{ p | p inside(1) tube; p outside(2) std.top; \
                             p inside(2) lid }}\n\
                             g := solid(grown)\nk := solid(skin)\ntin := solid(can)\n"));
    holds(&e, "g", 4.0 / 3.0 * PI * 125.0, &[[4.9, 0.0, 0.0]], &[[5.1, 0.0, 0.0]]);
    holds(&e, "k", 4.0 / 3.0 * PI * (27.0 - 1.0), &[[1.5, 0.0, 0.0]], &[[0.9, 0.0, 0.0]]);
    holds(&e, "tin", PI * 49.0 * 14.0, &[[6.9, 0.0, -1.9], [1.0, 0.0, 11.9]],
          &[[7.1, 0.0, 5.0], [1.0, 0.0, -2.1], [1.0, 0.0, 12.1]]);
}

/// A bound on the distance inside is the other way on the distance outside: within 1 of a ball's
/// surface on its inside is outside the ball 1 smaller — and in a region, a shell.
#[test]
fn a_bound_inside_turns_round() {
    let e = solved(&format!("{FRAME}big := std.Sphere(c, r: 3)\n\
                             rind := {{ p | p inside big; p distance(<= 1, side: inside) big }}\n\
                             r := solid(rind)\n"));
    holds(&e, "r", 4.0 / 3.0 * PI * (27.0 - 8.0), &[[2.5, 0.0, 0.0]], &[[1.5, 0.0, 0.0]]);
}

/// A plane's side grown by a distance chooses a drawn point's root as its side does: the axis
/// meets a ball of radius 1 at heights 1 and -1, and within `std.top`'s side grown by 2 both
/// stand, so the seed's is kept — where the side alone carries it across.
#[test]
fn a_planes_side_grown_chooses_a_root() {
    for (said, seed, want) in [
        ("p inside(2) std.top", 0.8, 1.0),
        ("p outside(2) std.top", -0.8, -1.0),
        ("p inside std.top", 0.8, -1.0),
        ("p outside std.top", -0.8, 1.0),
    ] {
        let e = solved(&format!("{FRAME}ball := std.Sphere(c, r: 1)\n\
                                 p := point hint((0, 0, {seed}))\nfix(x == 0, y == 0) p\n\
                                 p coincident ball\n{said}\n"));
        let z = e.sketch.world_point(ent(&e, "p").i())[2];
        assert!((z - want).abs() <= 1e-9, "{said}: {z}");
    }
}

/// A point at a distance from a ball: its centre's distance moved.
#[test]
fn a_point_at_a_distance_from_a_ball() {
    for (side, want) in [("outside", 15.0), ("inside", 5.0)] {
        let e = solved(&format!("{FRAME}ball := std.Sphere(c, r: 10)\np := point hint((12, 0, 0))\n\
                                 fix(y == 0, z == 0) p\np distance(5, side: {side}) ball\n"));
        let x = e.sketch.world_point(ent(&e, "p").i())[0];
        assert!((x - want).abs() <= 1e-9, "{side}: {x}");
    }
}

/// A point at a distance from a cone, by its foot on it: at height 5 the surface stands 5 tan
/// 30° out, and a point 1 from it 1 / cos 30° further, or nearer — standing in space, drawn in
/// a view, or put on the cone's offset.
#[test]
fn a_point_at_a_distance_from_a_cone() {
    let surface = 5.0 * tan30();
    let off = 1.0 / (30f64).to_radians().cos();
    let point = [
        "p := point hint((4, 0, 5))\nfix(y == 0, z == 5) p\n",
        "in std.front {\n  p := point hint((4, 5))\n}\nfix(y == 5) p\n",
    ];
    for p in point {
        for (said, want) in [
            ("p distance(1, side: outside) funnel", surface + off),
            ("p distance(1, side: inside) funnel", surface - off),
            ("p coincident std.Offset(funnel, d: 1)", surface + off),
        ] {
            let src = format!("{FRAME}{p}{said}\n");
            let mut e = solved(&src);
            let x = e.sketch.world_point(ent(&e, "p").i());
            assert!((x[0] - want).abs() <= 1e-9 && x[2] == 5.0, "{said}: {x:?}\n{src}");
            let d = gcs_core::diagnose::diagnose(&mut e.sketch, Default::default());
            assert_eq!(d.dof, 0, "{said}: {}", gcs_core::diagnose::summary(&d));
        }
    }
}

/// A cone whose half is an unknown a held point on it sets: the side the point at a distance
/// stands on is a bound read against the half as solved (§9.6), and the half is left unseeded so
/// its row seeds it from the pose.
#[test]
fn a_point_at_a_distance_from_a_cone_its_point_sizes() {
    let q = format!("q := point hint((5, 0, 10))\nfix(({}, 0, 10)) q\n", 10.0 * tan30());
    for (side, want) in [("outside", 1.0), ("inside", -1.0)] {
        let src = format!("{FRAME}{q}k := std.Cone(up)\nq coincident k\n\
                           p := point hint((4, 0, 5))\nfix(y == 0, z == 5) p\n\
                           p distance(1, side: {side}) k\n");
        let e = solved(&src);
        let x = e.sketch.world_point(ent(&e, "p").i())[0];
        let at = 5.0 * tan30() + want / (30f64).to_radians().cos();
        assert!((x - at).abs() <= 1e-9, "{side}: {x} against {at}");
    }
}

/// What a distance from a set cannot say, each refused at its word.
#[test]
fn what_a_distance_from_a_set_refuses() {
    let point = "p := point hint((4, 0, 5))\nfix(y == 0, z == 5) p\n";
    // each refused at its word, with its parentheses as the parser spans them
    for (said, why, at) in [
        ("p distance(1) funnel", "two sides", "distance(1)"),
        ("p distance(0, side: outside) funnel", "is on it", "distance(0, side: outside)"),
        ("p distance(<= 1, side: outside) funnel", "chooses among roots",
         "distance(<= 1, side: outside)"),
        ("p inside(1) funnel", "chooses among roots", "inside(1)"),
        ("p distance(in: (1, 2), side: outside) funnel", "one side of a number",
         "distance(in: (1, 2), side: outside)"),
        ("claim p distance(1, side: outside) funnel", "claim cannot add", "distance"),
        ("r := { q | q inside funnel; q inside lid }\np distance(1, side: outside) r",
         "is a region", "distance(1, side: outside)"),
        ("ring := { q | q coincident lid; q distance(3) c }\np distance(1, side: outside) ring",
         "says 2 things", "distance(1, side: outside)"),
        ("q := point hint((1, 0, 1))\nl := line(q, p)\nl tangent std.Offset(funnel, d: 1)",
         "derivative cannot move", "tangent"),
    ] {
        refused(&format!("{FRAME}{point}{said}\n"), "E040", why, at);
    }
    let region = "r := { q | q inside funnel; q inside lid }\nbad := { q | q inside(1) r }\n\
                  b := solid(bad)\n";
    refused(&format!("{FRAME}{region}"), "E040", "is a region", "bad");
}
