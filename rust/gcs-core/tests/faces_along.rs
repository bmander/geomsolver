//! **A face along a direction** (#763): `when a parallel l` holds where every point of face `a`
//! has `l`'s direction in its tangent plane — a plane holding it, a cylinder about it, a prism's
//! side swept along it — and `when a perpendicular l` where `a` is a plane square to it, as the
//! relation words say of a plane and an axis.  Read where the drawing is seeded, as `meets` is,
//! and again where it solves where a block comes out the other way there.  So the edges a round
//! takes are picked by a block over the faces written in Solvent, not by an option of the word:
//! two faces along `std.z` meet in lines along it, the edges in plan.
use std::f64::consts::PI;

use gcs_core::brep::props;

use crate::common::lenient::read;
use crate::common::rounds::{exported, kinds, line_beside_circle};
use crate::common::{close, text::solved};

const HUB: &str = include_str!("../../examples/rounds/hub.sv");

/// The block over the faces in `hub.sv`, as the example writes it.
const ALONG: &str = "when f parallel std.z and g parallel std.z {";

/// `hub.sv` with its faces held against a line `up` drawn up the front plane from the origin, its
/// far end `u1` drawn by `far` — and the block's condition `cond`.
fn against_up(cond: &str, far: &str) -> String {
    HUB.replace(ALONG, &format!("when {cond} {{")).replace("in std.top {", &format!(
        "in std.front {{\n  u0 := point\n  fix((0, 0)) u0\n  {far}\n  up := line(u0, u1)\n}}\n\
         in std.top {{"))
}

/// `hub.sv`'s body with nothing rounded: the bar 40 × 8 × 3 and the post of radius 6 and height
/// 10, less what they share, the half disc's band `|y| < 4` through the bar's depth.
fn unrounded() -> f64 {
    let shared = 4.0 * 20f64.sqrt() + 36.0 * (2.0f64 / 3.0).asin();
    40.0 * 8.0 * 3.0 + PI * 36.0 * 10.0 - 3.0 * shared
}

/// The same with its sides filled into the post through the bar's depth: two prisms 3 deep of the
/// section between a line 4 off the post's axis and its wall, a ball of 3.
fn filled() -> f64 {
    unrounded() + 2.0 * 3.0 * line_beside_circle(4.0, 6.0, 3.0)
}

/// **The edges in plan** (#763): the bar's sides filled into the post's wall through the bar's
/// depth — two prisms of the section between a line and a circle — and the arc where the post
/// rises from the bar's top left sharp, so no torus: the closed form's volume to 1e-9.
#[test]
fn a_bars_sides_are_filled_into_the_post_and_its_top_left_sharp() {
    assert!(HUB.contains(ALONG));
    let e = solved(HUB);
    let b = exported(&e, "hub");
    close(props::volume(&b), filled());
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
    close(props::volume(&b), filled());
}

/// A line drawn is a direction too: one up the front plane runs along the post as `std.z` does,
/// and the bar's sides are filled; one across it runs along the bar's sides but not the post's
/// wall, and nothing is.
#[test]
fn a_line_drawn_is_a_direction_too() {
    let along = |far: &str| {
        let src = against_up("f parallel up and g parallel up", far);
        props::volume(&exported(&solved(&src), "hub"))
    };
    close(along("u1 := point\n  fix((0, 1)) u1"), filled());
    close(along("u1 := point\n  fix((1, 0)) u1"), unrounded());
}

/// What a `when` holds a face against is a direction drawn, and what it holds is a face: anything
/// else is E103 saying which (a name standing for nothing E101), and makes nothing.
#[test]
fn a_face_along_a_direction_or_e103() {
    let refused = |cond: &str, code: &str, words: &str| {
        let (mut e, d) = read(&HUB.replace(ALONG, &format!("when {cond} {{")));
        assert!(d.iter().any(|m| m.starts_with(code) && m.contains(words)),
                "{cond}: expected {code} saying {words:?}: {d:?}");
        assert!(gcs_core::solve::solve(&mut e.sketch, Default::default()).success);
        close(props::volume(&exported(&e, "hub")), unrounded());
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

/// **Read where the drawing solves** (#763, as `meets` is, #761): the faces held against a line
/// drawn up the front plane, its far end seeded leaning and held along `std.z`.  Where the drawing
/// is seeded the post's wall does not run along the line, so the `when` leaves the fills out; where
/// it solves the block comes out the other way, so the faces are read again there and the fills
/// are built, as they are seeded upright, and nothing is said.  A block whose condition fails
/// however its faces are read — the bar's own top, which runs along nothing, beside a face that
/// turns — makes nothing either way.  A plane seeded leaning is settled before the faces are named,
/// so a deck held flat builds them as the top plane does.
#[test]
fn a_face_read_off_the_direction_where_it_is_seeded_is_read_where_it_solves() {
    let up = |cond: &str, seed: &str| against_up(cond, &format!(
        "u1 := point hint(({seed}))\n  u0 distance(1mm) u1\n  up parallel std.z"));
    let said = |src: &str| {
        let e = solved(src);
        let d = gcs_core::program::solid_diagnostics(&e.sketch, &e.map, &e.program);
        (props::volume(&exported(&e, "hub")), d)
    };
    let both = "f parallel up and g parallel up";
    for seed in ["0, 1", "0.3, 1"] {
        let (v, d) = said(&up(both, seed));
        assert!(d.is_empty(), "{seed}: {d:?}");
        close(v, filled());
    }
    let (v, never) = said(&up("f parallel up and g parallel up and f == hub.bar.near", "0.3, 1"));
    assert!(never.is_empty(), "{never:?}");
    close(v, unrounded());
    let deck = HUB.replace("in std.top {", "du := axis hint(dir: (1, 0, 0))\n\
        dv := axis hint(dir: (0, 1, 0.3))\ndu parallel std.x\ndv parallel std.y\n\
        deck := plane(u: du, v: dv)\nfix(origin == (0mm, 0mm, 0mm)) deck\nin deck {");
    let (v, flat) = said(&deck);
    assert!(flat.is_empty(), "{flat:?}");
    close(v, filled());
}

/// **A `when` that reads both** (#763 with #761): a pin of radius 3 standing on a plate 20 × 20,
/// its centre held at the plate's middle and seeded far off it, a point made for each face of the
/// plate the pin's wall — along `std.z` — does not meet.  Where the drawing is seeded the wall
/// meets none of the six, and where it solves it meets the plate's top: decided again there with
/// both its reads, the block comes out the other way for the top, so the faces are read where the
/// drawing solves and five points are made, as seeded on the plate.
#[test]
fn a_when_over_faces_met_and_faces_along_is_decided_where_the_drawing_solves() {
    let pin = |at: &str| format!("unit mm\nuse std\nuse std.solids (union)\nuse std.place (coords)\n\
        in std.top {{\na := point hint((0, 0))\nb := point hint((20, 0))\n\
        c := point hint((20, 20))\nd := point hint((0, 20))\na coords(du: 0mm, dv: 0mm) std.top\n\
        b coords(du: 20mm, dv: 0mm) std.top\nc coords(du: 20mm, dv: 20mm) std.top\n\
        d coords(du: 0mm, dv: 20mm) std.top\no := point hint(({at}))\n\
        o coords(du: 10mm, dv: 10mm) std.top\nboss := circle(center: o) hint(r: 3)\n\
        radius(3mm) boss\n}}\n\
        plate := solid(std.solids.Prism(face(a, b, c, d, -> close), from: 0mm, to: 5mm))\n\
        pin := solid(std.solids.Prism(face(boss), from: 0mm, to: 10mm))\n\
        whole := solid(plate)\npin union whole\n\
        repeat f in whole.pin {{\n  repeat g in whole.plate {{\n    \
        when not (g meets f) and f parallel std.z {{\n      m := point in std.top\n      \
        fix((0, 0)) m\n    }}\n  }}\n}}\n");
    let made = |at: &str| {
        let e = solved(&pin(at));
        let d = gcs_core::program::solid_diagnostics(&e.sketch, &e.map, &e.program);
        assert!(d.is_empty(), "{at}: {d:?}");
        (0..e.sketch.points.len())
            .filter_map(|i| e.map.name_of(gcs_core::model::EntRef::point(i)))
            .filter(|n| n.as_str() == "m" || n.ends_with(".m"))
            .count()
    };
    assert_eq!(made("10, 10"), 5);
    assert_eq!(made("60, 60"), 5);
}
