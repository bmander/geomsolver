//! **A face along a direction** (#763): `when a parallel l` holds where every point of face `a`
//! has `l`'s direction in its tangent plane — a plane holding it, a cylinder about it, a prism's
//! side swept along it — and `when a perpendicular l` where `a` is a plane square to it, as the
//! relation words say of a plane and an axis.  Read where the drawing is seeded, as `meets` is,
//! and again where it is solved.  So the edges a round takes are picked by a block over the faces
//! written in Solvent, not by an option of the word: two faces along `std.z` meet in lines along
//! it, the edges in plan.
use std::f64::consts::PI;

use gcs_core::brep::topo::Brep;
use gcs_core::brep::{export, props, sweep::Say};
use gcs_core::program::Elaborated;

use crate::common::lenient::read;
use crate::common::{close, ent, text::solved};

const HUB: &str = include_str!("../../examples/rounds/hub.sv");

/// The block over the faces in `hub.sv`, as the example writes it.
const ALONG: &str = "when f parallel std.z and g parallel std.z {";

/// Solid `name` of `e` built and written as STEP and as STL by this kernel (the STL read back as
/// closed shells), and its B-rep.
fn exported(e: &Elaborated, name: &str) -> Brep {
    let body = ent(e, name).i();
    let say = Say { stage: &|_: &str| {}, mark: &|_| {} };
    let exact = export::exact(&e.sketch, body, None, None, &say)
        .unwrap_or_else(|r| panic!("{name}: {r:?}"));
    export::step(&exact, name, None, &say).unwrap();
    let stl = export::stl(&e.sketch, body, &exact, None, &say).unwrap();
    gcs_core::mesh::stl_shells(&stl).unwrap();
    exact.solid
}

/// How many of a B-rep's faces lie on each kind of surface.
fn kinds(b: &Brep) -> std::collections::BTreeMap<&'static str, usize> {
    let mut out = std::collections::BTreeMap::new();
    for f in &b.faces {
        *out.entry(f.surface.kind()).or_default() += 1;
    }
    out
}

/// The section of the fill between a line `w` off a circle's centre, the material on the centre's
/// side, and the circle of radius `big`, filled by a ball of radius `r` outside both: the
/// quadrilateral of the ball's centre, its touch on the line, the corner and the circle's centre,
/// less the circle's sector from the corner to the ball's touch and the ball's own sector.
fn line_beside_circle(w: f64, big: f64, r: f64) -> f64 {
    let s = ((big + r) * (big + r) - (w + r) * (w + r)).sqrt();
    let k = (big * big - w * w).sqrt();
    // about the circle's centre: the ball's centre, its touch on the line, the corner
    let (c, t, q) = ([-s, w + r], [-s, w], [-k, w]);
    let quad = 0.5 * ((c[0] * t[1] - t[0] * c[1]) + (t[0] * q[1] - q[0] * t[1])
        + (q[0] * 0.0 - 0.0 * q[1]) + (0.0 * c[1] - c[0] * 0.0)).abs();
    let angle = |u: [f64; 2], v: [f64; 2]| {
        (u[0] * v[1] - u[1] * v[0]).atan2(u[0] * v[0] + u[1] * v[1]).abs()
    };
    let at_centre = angle(q, c);
    let at_ball = angle([0.0, -r], [-c[0], -c[1]]);
    quad - 0.5 * big * big * at_centre - 0.5 * r * r * at_ball
}

/// `hub.sv`'s body with nothing rounded: the bar 40 × 8 × 3 and the post of radius 6 and height
/// 10, less what they share, the half disc's band `|y| < 4` through the bar's depth.
fn unrounded() -> f64 {
    let shared = 4.0 * 20f64.sqrt() + 36.0 * (2.0f64 / 3.0).asin();
    40.0 * 8.0 * 3.0 + PI * 36.0 * 10.0 - 3.0 * shared
}

/// **The edges in plan** (#763): the bar's sides filled into the post's wall through the bar's
/// depth — two prisms of the section between a line and a circle — and the arc where the post
/// rises from the bar's top left sharp, so no torus: the closed form's volume to 1e-9.
#[test]
fn a_bars_sides_are_filled_into_the_post_and_its_top_left_sharp() {
    assert!(HUB.contains(ALONG));
    let e = solved(HUB);
    let b = exported(&e, "hub");
    close(props::volume(&b), unrounded() + 2.0 * 3.0 * line_beside_circle(4.0, 6.0, 3.0));
    let k = kinds(&b);
    assert_eq!(k.get("torus"), None, "{k:?}");
    assert_eq!(k.get("cylinder"), Some(&3), "the post's wall and a fill on each side: {k:?}");
}

/// The same faces picked by what they are not — the bar's top and floor and the post's caps left
/// out by identity — make the same two fills; rounding the bar against the post outright fills
/// the arc too, so more.
#[test]
fn the_same_edges_by_name_and_all_of_them() {
    let by_name = HUB.replace(ALONG,
        "when f != hub.bar.near and f != hub.bar.far and g == hub.post.k {");
    let named = exported(&solved(&by_name), "hub");
    let along = exported(&solved(HUB), "hub");
    close(props::volume(&named), props::volume(&along));
    let every = format!("{}hub.bar round(r: r) hub.post\n", &HUB[..HUB.find("repeat f").unwrap()]);
    let all = exported(&solved(&every), "hub");
    assert!(props::volume(&all) > props::volume(&along) + 1.0, "the arc filled too");
    assert!(kinds(&all).contains_key("torus"));
}

/// `perpendicular` keeps the bar's top and floor, square to the axis: against the post's wall the
/// top makes the arc's fill, a ring's stretch about the post, and the floor — the post's own floor
/// going on flat into it — no edge; so no fill along the sides.
#[test]
fn the_arc_alone_by_the_faces_square_to_the_axis() {
    let src = HUB.replace(ALONG, "when f perpendicular std.z and g parallel std.z {");
    let b = exported(&solved(&src), "hub");
    let k = kinds(&b);
    assert_eq!(k.get("torus"), Some(&1), "{k:?}");
    assert_eq!(k.get("cylinder"), Some(&1), "the post's wall alone: {k:?}");
    assert!(props::volume(&b) > unrounded() + 1.0);
}

/// **Against the instance's z** (#763): the bar and its post drawn in a component placed in the
/// front plane, where the component's `std.top` is the world's front plane and its `std.z` the
/// frame's normal, the world's `y`.  The sides run along that and are filled as in the world's
/// top plane; read as the world's z, they would run along nothing and nothing would be filled.
#[test]
fn std_z_in_a_component_is_its_frames() {
    let open = HUB.find("r := 3mm").unwrap();
    let body = &HUB[HUB.find("in std.top").unwrap()..];
    let src = format!(
        "{}component Keeper(r: Length) {{\n{body}}}\nkeeper := Keeper(r: 3mm) in std.front\n",
        &HUB[..open]);
    let b = exported(&solved(&src), "keeper.hub");
    close(props::volume(&b), unrounded() + 2.0 * 3.0 * line_beside_circle(4.0, 6.0, 3.0));
}

/// A line drawn is a direction too: one up the front plane runs along the post as `std.z` does,
/// and the bar's sides are filled; one across it runs along the bar's sides but not the post's
/// wall, and nothing is.
#[test]
fn a_line_drawn_is_a_direction_too() {
    let src = HUB.replace(ALONG, "when f parallel up and g parallel up {")
        .replace("in std.top {", "in std.front {\n  u0 := point\n  u1 := point\n  \
                  fix((0, 0)) u0\n  fix((0, 1)) u1\n  up := line(u0, u1)\n}\nin std.top {");
    let b = exported(&solved(&src), "hub");
    close(props::volume(&b), unrounded() + 2.0 * 3.0 * line_beside_circle(4.0, 6.0, 3.0));
    let across = src.replace("fix((0, 1)) u1", "fix((1, 0)) u1");
    let b = exported(&solved(&across), "hub");
    close(props::volume(&b), unrounded());
}

/// What a `when` holds a face against is a direction drawn, and what it holds is a face: anything
/// else is E103 saying which (a name standing for nothing E101), and makes nothing.
#[test]
fn a_face_along_a_direction_or_e103() {
    let refused = |cond: &str, code: &str, words: &str| {
        let (e, d) = read(&HUB.replace(ALONG, &format!("when {cond} {{")));
        assert!(d.iter().any(|m| m.starts_with(code) && m.contains(words)),
                "{cond}: expected {code} saying {words:?}: {d:?}");
        let b = exported(&{
            let mut e = e;
            assert!(gcs_core::solve::solve(&mut e.sketch, Default::default()).success);
            e
        }, "hub");
        close(props::volume(&b), unrounded());
    };
    refused("f parallel g", "E103", "is no direction drawn");
    refused("o parallel std.z", "E103", "`o` is no face");
    refused("f parallel nothing", "E101", "`nothing` is not a number here");
    // read as two names, which `decide` refuses
    let mm = gcs_core::units::Units::with_length("mm").unwrap();
    let c = gcs_core::expr::parse_condition_in("f perpendicular std.z", mm).unwrap();
    assert_eq!(c, gcs_core::expr::Cond::Along(gcs_core::expr::Along::Perpendicular,
                                              "f".into(), "std.z".into()));
    assert!(gcs_core::expr::parse_condition_in("f parallel 3", mm).is_err(),
            "a name, not a number");
    assert!(gcs_core::expr::decide(&c, &Default::default()).is_err());
}

/// **Read where the drawing is seeded, said where it solves** (#763): the faces held against a
/// line drawn up the front plane, its far end seeded leaning and held along `std.z`.  Where the
/// drawing is seeded the post's wall does not run along the line, so the `when` leaves the fills
/// out; where it solves it does — said at the block, rather than built with those edges left
/// sharp.  Seeded upright, the same document builds its fills and says nothing.  A plane seeded
/// leaning is settled before the faces are named, so a deck held flat builds them as the top
/// plane does.
#[test]
fn a_face_read_off_the_direction_where_it_is_seeded_is_said_where_it_solves() {
    let filled = unrounded() + 2.0 * 3.0 * line_beside_circle(4.0, 6.0, 3.0);
    let up = |seed: &str| HUB.replace(ALONG, "when f parallel up and g parallel up {")
        .replace("in std.top {", &format!("in std.front {{\n  u0 := point\n  \
                  u1 := point hint(({seed}))\n  fix((0, 0)) u0\n  u0 distance(1mm) u1\n  \
                  up := line(u0, u1)\n  up parallel std.z\n}}\nin std.top {{"));
    let said = |src: &str| {
        let e = solved(src);
        let d = gcs_core::program::solid_diagnostics(&e.sketch, &e.map, &e.program);
        (props::volume(&exported(&e, "hub")), d)
    };
    let (v, upright) = said(&up("0, 1"));
    assert!(upright.is_empty(), "{upright:?}");
    close(v, filled);
    let (v, leaning) = said(&up("0.3, 1"));
    close(v, unrounded());
    assert!(leaning.iter().any(|d| d.code.as_str() == "E080"
        && d.message.contains("is not `parallel up` where the drawing is seeded, but where it \
                               solves")
        && d.message.contains("`hub`'s face")), "{leaning:?}");
    let deck = HUB.replace("in std.top {", "du := axis hint(dir: (1, 0, 0))\n\
        dv := axis hint(dir: (0, 1, 0.3))\ndu parallel std.x\ndv parallel std.y\n\
        deck := plane(u: du, v: dv)\nfix(origin == (0mm, 0mm, 0mm)) deck\nin deck {");
    let (v, flat) = said(&deck);
    assert!(flat.is_empty(), "{flat:?}");
    close(v, filled);
}
