//! **Rounds that know of one another** ([0.149]): a word's use named (`rim := A round(r) B`), its
//! faces under the name; a round that names another's faces by that name reading the body as the
//! other leaves it (the stratum in layers, `solid::faces::layer`); and a ball or a flat standing
//! against a face's smooth surface (`F.smooth`), so a chamfer's leg runs on across the seam where
//! the face goes on tangent into a round.
use gcs_core::brep::{export, props, sweep::Say};
use gcs_core::program::{Code, Elaborated};
use gcs_core::solid::MaterialField;
use std::f64::consts::PI;

use crate::common::{ent, parse, text::solved};

/// A disc of radius `r` drawn about the front plane's origin and swept 20 deep (y from 0 to 20),
/// `rim` its wall, `near` and `far` its caps; then `rest`.
fn disc(r: f64, rest: &str) -> String {
    format!("unit mm\nuse std (round, chamfer)\nin std.front {{\n  o := point\n  fix((0, 0)) o\n  \
             rim := circle(center: o) hint(r: {r})\n  radius({r}mm) rim\n}}\n\
             plate := solid(face(rim), depth: 20mm)\n{rest}")
}

/// Solid `name` of `e` built exactly, and its volume.
fn volume(e: &Elaborated, name: &str) -> f64 {
    let say = Say { stage: &|_: &str| {}, mark: &|_| {} };
    let exact = export::exact(&e.sketch, ent(e, name).i(), None, None, &say)
        .unwrap_or_else(|r| panic!("{name}: {r:?}"));
    props::volume(&exact.solid)
}

/// Solid `name`'s faces and their areas, as the report names them under it.
fn areas(e: &Elaborated, name: &str) -> Vec<(String, f64)> {
    let want = format!("{name}.");
    gcs_core::report::positions_where(&e.sketch, &e.map, &|n| n == name).into_iter()
        .filter_map(|(k, v)| k.strip_prefix(&want)?.strip_suffix(".area").map(|f| (f.to_string(), v)))
        .collect()
}

/// The area of face `face` among `areas`, or nothing.
fn area(areas: &[(String, f64)], face: &str) -> f64 {
    areas.iter().find(|(f, _)| f == face).map_or(0.0, |(_, a)| *a)
}

/// The quarter torus a ball of radius `r` rounds a disc's rim of radius `big` with (Pappus: the
/// quarter arc times the path of its centroid).
fn rim_torus(big: f64, r: f64) -> f64 {
    PI * PI * r * (big - r) + 2.0 * PI * r * r
}

/// Whether `src` is refused with `code`, saying `says`.
fn refused(src: &str, code: Code, says: &str) {
    let (prog, errs) = parse(src);
    assert!(errs.is_empty(), "{errs:?}");
    let e = gcs_core::program::elaborate(&prog);
    assert!(e.diags.iter().any(|d| d.code == code && d.message.contains(says)),
            "wanted {code:?} saying {says:?}: {:?}", e.diags);
}

/// **A named round's faces are its own**: two rounds of one rim, each named, are two faces under
/// their names, each the quarter torus its radius makes; written bare, the two read as one name.
#[test]
fn named_rounds_keep_their_faces_apart() {
    let e = solved(&disc(33.54, "top := plate.rim round(r: 3mm) plate.far\n\
                                  bottom := plate.rim round(r: 2mm) plate.near\n"));
    let a = areas(&e, "plate");
    let (top, bottom) = (area(&a, "top.taken.round"), area(&a, "bottom.taken.round"));
    assert!((top - rim_torus(33.54, 3.0)).abs() < 1e-4 * top, "{a:?}");
    assert!((bottom - rim_torus(33.54, 2.0)).abs() < 1e-4 * bottom, "{a:?}");
    let bare = solved(&disc(33.54, "plate.rim round(r: 3mm) plate.far\nplate.rim round(r: 2mm) plate.near\n"));
    let both = area(&areas(&bare, "plate"), "taken.round");
    assert!((both - top - bottom).abs() < 1e-4 * both, "{both} against {top} + {bottom}");
}

/// **A name is a use's of a word a file defines**: a built-in relation makes nothing to name, and
/// a name is one thing where it is written.
#[test]
fn a_name_is_a_defined_words_use_and_one_thing() {
    // a word the language has is read as the declaration it would name, and refused there
    assert!(!parse("a := point\nb := point\nh := a horizontal b\n").1.is_empty());
    refused(&disc(30.0, "top := plate.rim round(r: 3mm) plate.far\ntop := plate.rim round(r: 2mm) plate.near\n"),
            Code::E001, "declared twice");
    refused(&disc(30.0, "top := point\ntop := plate.rim round(r: 3mm) plate.far\n"), Code::E001, "declared twice");
}

/// The area a ball of radius `r` cannot reach at a convex edge of material angle `alpha`, and the
/// distance of its centroid from the edge along the bisector: the kite between the edge, the
/// ball's two touching points and its centre, less the ball's sector there.
fn sliver(r: f64, alpha: f64) -> (f64, f64) {
    let (h, phi) = (alpha / 2.0, PI - alpha);
    let (centre, touch) = (r / h.sin(), r / h.tan());
    let kite = (r * touch, (centre + touch * h.cos()) / 3.0);
    let sector = (r * r * phi / 2.0, centre - 4.0 * r * (phi / 2.0).sin() / (3.0 * phi));
    let a = kite.0 - sector.0;
    (a, (kite.0 * kite.1 - sector.0 * sector.1) / a)
}

/// **A round over the faces another makes reads the body that one leaves** (`rounds/composed.sv`):
/// a disc of 30 chamfered 4 at its far rim, the chamfer named `bevel`, and its two edges — 135°,
/// one where the cone meets the wall at (30, 16), one where it meets the far cap at (26, 20) —
/// rounded 1 by a statement naming it.  The disc less the chamfer's ring less the two slivers,
/// each by Pappus; and the same as the hand's intermediate, `part := solid(plate)`, rounded.
#[test]
fn a_round_over_a_chamfers_edges_reads_the_body_the_chamfer_leaves() {
    let e = solved(include_str!("../../examples/rounds/composed.sv"));
    let got = volume(&e, "plate");
    let ring = 2.0 * PI * (30.0 - 4.0 / 3.0) * 8.0;
    let (a, along) = sliver(1.0, 0.75 * PI);
    // each edge's bisector into the material: (−cos 22.5°, …) at the wall's, (−sin 22.5°, …) at the cap's
    let (wall, cap) = (30.0 - along * (PI / 8.0).cos(), 26.0 - along * (PI / 8.0).sin());
    let want = PI * 900.0 * 20.0 - ring - a * 2.0 * PI * (wall + cap);
    assert!((got - want).abs() <= 1e-9 * want, "{got} against {want}");
    let faces = areas(&e, "plate");
    assert!(area(&faces, "taken.round") > 0.0 && area(&faces, "bevel.taken.flat") > 0.0, "{faces:?}");
    let by_hand = solved(&disc(30.0, "bevel := plate.rim chamfer(d: 4mm) plate.far\npart := solid(plate)\n\
                                      part.plate.bevel round(r: 1mm) part.plate\n"));
    let hand = volume(&by_hand, "part");
    assert!((got - hand).abs() <= 1e-9 * want, "{got} against the intermediate's {hand}");
}

/// **Rounds of one body that name nothing of each other read it before either**: a round of the
/// near and far rims beside a named chamfer of the far one reads the plate unchamfered, so it
/// rounds no edge the chamfer made — at the far rim what it would take the chamfer took already —
/// and the plate is less the chamfer's ring and the near rim's corner (Pappus).
#[test]
fn a_round_naming_nothing_reads_the_body_before_the_others() {
    let e = solved(&disc(30.0, "bevel := plate.rim chamfer(d: 4mm) plate.far\n\
                                plate.rim round(r: 1mm) plate.near\nplate.rim round(r: 1mm) plate.far\n"));
    let got = volume(&e, "plate");
    let ring = 2.0 * PI * (30.0 - 4.0 / 3.0) * 8.0;
    let inset = (10.0 - 3.0 * PI) / (3.0 * (4.0 - PI));
    let near = (1.0 - PI / 4.0) * 2.0 * PI * (30.0 - inset);
    let want = PI * 900.0 * 20.0 - ring - near;
    assert!((got - want).abs() <= 1e-9 * want, "{got} against {want}");
}

/// **Uses that name each other are refused**: neither can act on the body the other leaves.  (The
/// library's words reach another's faces only through the faces they pair, so two of them naming
/// each other make nothing for the other to name; a word whose body reads a face path past its
/// operand does.)
#[test]
fn uses_that_name_each_other_are_refused() {
    let shave = "A shave(d) B := {\n  near := { p | p inside A; p distance(<= d) B }\n  \
                 shaved := solid(near)\n  shaved cut A.body\n}\n";
    let src = disc(30.0, "x := plate.y shave(d: 1mm) plate.far\ny := plate.x shave(d: 1mm) plate.far\n");
    refused(&src.replacen("in std.front", &format!("{shave}in std.front"), 1), Code::E041,
            "each name the faces the other makes");
}

/// A plate of 40, its far rim rounded 4 (the far face flat out to 36, then a torus), a boss of 25
/// about (10, 0) standing on it from y = 20 to 30 — its foot reaching 35, a millimetre short of the
/// round — and the boss's foot chamfered 2 against the plate's far face.
const SPILL: &str = "unit mm\nuse std (round, chamfer)\nin std.front {\n  o := point\n  fix((0, 0)) o\n  \
    rim := circle(center: o) hint(r: 40)\n  radius(40mm) rim\n  q := point\n  fix((10, 0)) q\n  \
    foot := circle(center: q) hint(r: 25)\n  radius(25mm) foot\n}\n\
    plate := solid(face(rim), depth: 20mm)\nplate.rim round(r: 4mm) plate.far\n\
    boss := solid(face(foot), from: -30mm, to: -20mm)\npart := solid(plate)\nboss union part\n\
    part.boss.foot chamfer(d: 2mm) part.plate.far\n";

/// **A chamfer's leg runs on across a seam** (`F.smooth`): 2 out from the boss's foot is past
/// where the far face goes on tangent into the rim's round, so the flat lands on the round, and
/// what it adds fills down to it — a point between the far face's plane and the round, 1.5 out
/// from the boss's wall, is material, where a flat standing on the plane run on into the air left
/// a ledge over a gap.  Beside the boss away from the rim, where the leg stays on the flat, the
/// chamfer is the flat's.
#[test]
fn a_chamfers_leg_runs_on_across_a_seam_into_a_round() {
    let e = solved(SPILL);
    let field = MaterialField::read(&e.sketch, ent(&e, "part").i(), 1e-10).unwrap();
    // over the round at x = 36.5, its surface at 16 + √(16 − 0.25) ≈ 19.969
    for p in [[36.5, 19.985, 0.0], [36.5, 20.2, 0.0]] {
        assert!(field.side(p) < 0.0, "{p:?} is the chamfer's");
    }
    for p in [[36.5, 20.6, 0.0], [37.2, 19.9, 0.0]] {
        assert!(field.side(p) > 0.0, "{p:?} is beyond it");
    }
    // on the far side of the boss, the flat chamfer: 1 out, up to 1 above the face
    assert!(field.side([-16.0, 20.9, 0.0]) < 0.0 && field.side([-16.0, 21.1, 0.0]) > 0.0);
}

/// **Where a face goes on tangent and the leg stays short of the seam, nothing changes**: a boss
/// standing well inside the round, chamfered, built exactly — the flat's ring by Pappus beside the
/// plate, the boss and the rim's round.
#[test]
fn a_chamfer_short_of_the_seam_is_the_flats() {
    let src = SPILL.replace("radius(25mm) foot", "radius(20mm) foot").replace("hint(r: 25)", "hint(r: 20)");
    let e = solved(&src);
    let got = volume(&e, "part");
    let corner = 16.0 * (1.0 - PI / 4.0);
    // the corner a quarter ball leaves, its centroid (10 − 3π)/(3(4 − π)) of the radius in from
    // the rim along each face
    let inset = 4.0 * (10.0 - 3.0 * PI) / (3.0 * (4.0 - PI));
    let rim = corner * 2.0 * PI * (40.0 - inset);
    let ring = 2.0 * PI * (20.0 + 2.0 / 3.0) * 2.0;
    let want = PI * 1600.0 * 20.0 - rim + PI * 400.0 * 10.0 + ring;
    assert!((got - want).abs() <= 1e-9 * want, "{got} against {want}");
}

/// The user's part (#419): a plate of 33.54 its far rim rounded 3, a boss of radius `r` drawn in a
/// plane of its own standing on the far face, its foot chamfered 2 against the far face — out past
/// the seam onto the round, the region the field's (`lowering::seams_beside`).
fn spilled(r: f64) -> String {
    format!("use std (round, chamfer)\n\
             c0 := circle(center: std.front.origin) hint(r: 33.54) in std.front\nf0 := face(c0)\n\
             b0 := solid(f0, depth: 20)\nb0.c0 round(r: 3) b0.far\n\
             v0 := plane(u: hint(dir: (1, 0, 0)), v: hint(dir: (0, 0, 1))) \
             hint(origin: (-0.110911353, 20, 8.93063091))\n\
             v0.origin coincident b0.far.surface\nv0 tangent(at: v0.origin) b0.far.surface\n\
             p0 := point hint((-8.4, -6.7)) in v0\nc1 := circle(center: p0) hint(r: {r}) in v0\n\
             f1 := face(c1)\nb1 := solid(f1, depth: 10)\nb0 union b1\nb1.b0.far chamfer(d: 2) b1.c1\n")
}

/// The volume a closed triangle mesh holds.
fn mesh_volume(s: &gcs_core::solid::FieldSurface) -> f64 {
    s.triangles.iter().map(|t| {
        let [a, b, c] = t.map(|i| s.vertices[i as usize]);
        (a[0] * (b[1] * c[2] - b[2] * c[1]) - a[1] * (b[0] * c[2] - b[2] * c[0])
            + a[2] * (b[0] * c[1] - b[1] * c[0])) / 6.0
    }).sum()
}

/// **An operand built exactly is read by its own boundary** (#419): the rounded plate is its
/// B-rep in the field, not its prism less its round's region, so no sheet of nothing lies on the
/// far face's plane over the round for the spilled chamfer's flat to cross, and the part meshes.
/// What it holds is the plate and the boss built exactly and some of the chamfer's ring: more
/// than the two, less than the two and the whole ring a flat on the plane would make.  Just over
/// the round, within the chamfer's reach, is material.
fn spilled_meshes(r: f64) {
    let e = solved(&spilled(r));
    let i = ent(&e, "b1").i();
    let surface = gcs_core::solid::FieldMesher::new(&e.sketch, i).unwrap().finish()
        .unwrap_or_else(|why| panic!("r = {r}: {why}"));
    assert!(!surface.provisional);
    let got = mesh_volume(&surface);
    let plate = volume(&e, "b0");
    let boss = PI * r * r * 10.0;
    let ring = 2.0 * PI * (r + 2.0 / 3.0) * 2.0;
    assert!(got > plate + boss && got < plate + boss + ring, "r = {r}: {got} against {plate} + {boss} + {ring}");
    // out from the plate's axis through the boss's centre, 1.5 past its wall: over the round,
    // whose surface there is 17 + √(9 − (ρ − 30.54)²), and under the chamfer's flat
    let field = MaterialField::read(&e.sketch, i, 1e-10).unwrap();
    let c = e.sketch.world_point(ent(&e, "p0").i());
    let out = c[0].hypot(c[2]);
    let rho = out + r + 1.5;
    let round = 17.0 + (9.0 - (rho - 30.54) * (rho - 30.54)).sqrt();
    assert!(round < 20.0 - 0.1, "r = {rho}: over the round");
    let at = |y: f64| [c[0] / out * rho, y, c[2] / out * rho];
    for y in [0.5 * (round + 20.0), 20.1] {
        assert!(field.side(at(y)) < 0.0, "r = {r}: {:?} is the chamfer's", at(y));
    }
    assert!(field.side(at(round - 0.1)) < 0.0 && field.side(at(21.5)) > 0.0);
}

#[test]
fn a_chamfer_spilled_onto_a_rounded_plate_meshes() {
    spilled_meshes(21.7);
}

#[test]
#[cfg_attr(not(feature = "slow"), ignore = "slow tier, a mesh for each boss")]
fn a_chamfer_spilled_onto_a_rounded_plate_meshes_at_every_radius() {
    for r in [21.0, 21.4, 21.6] {
        spilled_meshes(r);
    }
}
