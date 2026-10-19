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

/// The user's part (#419, `rounds/spill.sv`) with a boss of radius `r`: a plate of 33.54 its far
/// rim rounded 3, a boss drawn in a plane of its own standing on the far face, its foot chamfered
/// 2 against the far face — out past the seam onto the round, the region the field's
/// (`lowering::seams_beside`).
fn spilled(r: f64) -> String {
    let src = include_str!("../../examples/rounds/spill.sv");
    assert!(src.contains("hint(r: 21.7)"));
    src.replace("hint(r: 21.7)", &format!("hint(r: {r})"))
}

/// **The reach measured, not doubled** (#422's rung 1): a boss of 19.5 — its foot 8.8 from the
/// plate's axis, its chamfer's leg 2 out from it reaching 30.3, short of the seam at 30.54 though
/// within twice the leg of it — is built exactly: the flat's ring by Pappus beside the plate
/// and the boss, as the table built it before any seam was read.
#[test]
fn a_chamfer_whose_leg_stays_short_of_the_seam_is_built() {
    let r = 19.5;
    let e = solved(&spilled(r));
    let got = volume(&e, "boss");
    let ring = 2.0 * PI * (r + 2.0 / 3.0) * 2.0;
    let want = volume(&e, "plate") + PI * r * r * 10.0 + ring;
    assert!((got - want).abs() <= 1e-9 * want, "{got} against {want}");
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
    let i = ent(&e, "boss").i();
    let surface = gcs_core::solid::FieldMesher::new(&e.sketch, i).unwrap().finish()
        .unwrap_or_else(|why| panic!("r = {r}: {why}"));
    assert!(!surface.provisional);
    let got = mesh_volume(&surface);
    let plate = volume(&e, "plate");
    let boss = PI * r * r * 10.0;
    let ring = 2.0 * PI * (r + 2.0 / 3.0) * 2.0;
    assert!(got > plate + boss && got < plate + boss + ring, "r = {r}: {got} against {plate} + {boss} + {ring}");
    // out from the plate's axis through the boss's centre, 1.5 past its wall: over the round,
    // whose surface there is 17 + √(9 − (ρ − 30.54)²), and under the chamfer's flat
    let field = MaterialField::read(&e.sketch, i, 1e-10).unwrap();
    let c = e.sketch.world_point(ent(&e, "centre").i());
    let out = c[0].hypot(c[2]);
    let rho = out + r + 1.5;
    let round = 17.0 + (9.0 - (rho - 30.54) * (rho - 30.54)).sqrt();
    assert!(round < 20.0 - 0.05, "r = {r}: ρ = {rho} is over the round");
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

/// `rounds/ledge.sv` (#422): a block 40 × 24 × 12 (y from 0 to 12), its far face's north edge
/// rounded 3 — the far face flat to z = 21, then the round, a cylinder about the line y = 9,
/// z = 21 — and a rib 20 long, 4.5 wide and 6 tall standing on the far face along x, its north
/// wall at z = 19.5; then `rest` in place of its chamfer.
fn ledge(rest: &str) -> String {
    let src = include_str!("../../examples/rounds/ledge.sv");
    let chamfer = "part.rib.rn chamfer(d: d) part.block.far\n";
    assert!(src.contains(chamfer));
    src.replace(chamfer, rest)
}

/// `∫ f` over `[a, b]` by Simpson's rule on `n` (even) stretches.
fn simpson(f: impl Fn(f64) -> f64, a: f64, b: f64, n: usize) -> f64 {
    let h = (b - a) / n as f64;
    (0..=n).map(|k| {
        let w = if k == 0 || k == n { 1.0 } else if k % 2 == 1 { 4.0 } else { 2.0 };
        w * f(a + k as f64 * h)
    }).sum::<f64>() * h / 3.0
}

/// The block rounded and the rib, built exactly.
fn ledge_part() -> f64 {
    let corner = 9.0 * (1.0 - PI / 4.0);
    40.0 * 24.0 * 12.0 - 40.0 * corner + 20.0 * 4.5 * 6.0
}

/// **A chamfer whose leg spills onto a round, along a straight edge** (#422's rung 4): the rib's
/// north foot chamfered 2 — its leg on the far face reaches z = 21.5, past the seam at 21, onto the
/// round.  In the section the flat is the line `y + z = 33.5` over the face, to where it crosses
/// the round's normal at the seam (z = 21, y = 12.5), then the conic `|q − c| = 24.5 − z` over the
/// round (`c` its axis), down to the round at z = 21.5: built exactly, a prism of that section.
#[test]
fn a_chamfer_spilled_onto_a_round_along_a_straight_edge_is_built() {
    let e = solved(&ledge("part.rib.rn chamfer(d: d) part.block.far\n"));
    let got = volume(&e, "part");
    let trapezoid = 1.5 * (2.0 + 0.5) / 2.0;
    let over = simpson(|z| ((24.5 - z) * (24.5 - z) - (z - 21.0) * (z - 21.0)).sqrt()
        - (9.0 - (z - 21.0) * (z - 21.0)).sqrt(), 21.0, 21.5, 20000);
    let want = ledge_part() + 20.0 * (trapezoid + over);
    assert!((got - want).abs() <= 1e-9 * want, "{got} against {want}");
}

/// **A round whose ball spills onto a round, along a straight edge** (#422's rung 4): the rib's
/// north foot rounded 2 — the ball beside the wall stands at z = 21.5, past the seam, so it touches
/// the round (its centre 5 from the round's axis, at y = 9 + √24.75) and not the far face.
#[test]
fn a_round_spilled_onto_a_round_along_a_straight_edge_is_built() {
    let e = solved(&ledge("part.rib.rn round(r: 2mm) part.block.far\n"));
    let got = volume(&e, "part");
    let yc = 9.0 + 24.75f64.sqrt();
    // the ball's lower arc over the chain: the face to z = 21, the round to the touch at z = 21.3
    let chain = |z: f64| if z <= 21.0 { 12.0 } else { 9.0 + (9.0 - (z - 21.0) * (z - 21.0)).sqrt() };
    let area = simpson(|z| yc - (4.0 - (z - 21.5) * (z - 21.5)).sqrt() - chain(z), 19.5, 21.0, 20000)
        + simpson(|z| yc - (4.0 - (z - 21.5) * (z - 21.5)).sqrt() - chain(z), 21.0, 21.3, 20000);
    let want = ledge_part() + 20.0 * area;
    assert!((got - want).abs() <= 1e-9 * want, "{got} against {want}");
}

/// **A cut whose leg spills onto a round** (#422's rung 4, convex): a pocket 20 × 4.5 × 4 sunk in
/// the far face, its north wall at z = 19.5, its rim chamfered 2 — the leg on the far face past the
/// seam, onto the round.  Within the material the flat is `y = z − 9.5` under the face, to the
/// round's normal at the seam, then `|q − c| = z − 18.5` under the round, up to it at z = 21.5.
#[test]
fn a_chamfer_cut_spilled_onto_a_round_along_a_straight_edge_is_built() {
    let src = ledge("").replace("rib := solid(ribbed, from: -18mm, to: -12mm)\npart := solid(block)\nrib union part\n",
        "pocket := solid(ribbed, from: -12mm, to: -8mm)\npart := solid(block)\npocket cut part\n")
        + "part.pocket.rn chamfer(d: 2mm) part.block.far\n";
    let e = solved(&src);
    let got = volume(&e, "part");
    let under = 1.875 + simpson(|z| (9.0 - (z - 21.0) * (z - 21.0)).sqrt()
        - ((z - 18.5) * (z - 18.5) - (z - 21.0) * (z - 21.0)).sqrt(), 21.0, 21.5, 20000);
    let corner = 9.0 * (1.0 - PI / 4.0);
    let want = 40.0 * 24.0 * 12.0 - 40.0 * corner - 20.0 * 4.5 * 4.0 - 20.0 * under;
    assert!((got - want).abs() <= 1e-9 * want, "{got} against {want}");
}

/// The same chamfer with the faces named the other way round, and mirrored: the rib by the south
/// edge, the south edge rounded.
#[test]
fn a_chamfer_spilled_onto_a_round_is_built_whichever_face_goes_on() {
    let src = ledge("part.block.far chamfer(d: 2mm) part.rib.rs\n").replace("block.north round", "block.south round")
        .replace("fix((10, 15)) r0", "fix((10, 4.5)) r0").replace("fix((30, 15)) r1", "fix((30, 4.5)) r1")
        .replace("fix((30, 19.5)) r2", "fix((30, 9)) r2").replace("fix((10, 19.5)) r3", "fix((10, 9)) r3");
    let e = solved(&src);
    let got = volume(&e, "part");
    let over = simpson(|z| ((24.5 - z) * (24.5 - z) - (z - 21.0) * (z - 21.0)).sqrt()
        - (9.0 - (z - 21.0) * (z - 21.0)).sqrt(), 21.0, 21.5, 20000);
    let want = ledge_part() + 20.0 * (1.875 + over);
    assert!((got - want).abs() <= 1e-9 * want, "{got} against {want}");
}

/// A plate of 30 its far rim rounded 3 (the far face flat to 27, then a torus about its axis), a
/// boss of 26 about the same axis standing on it from y = 20 to 30, and the boss's foot treated by
/// `how` against the far face — 2 out from the boss, past the seam at 27.
fn coaxial(how: &str) -> String {
    format!("unit mm\nuse std (round, chamfer)\nin std.front {{\n  o := point\n  fix((0, 0)) o\n  \
        rim := circle(center: o) hint(r: 30)\n  radius(30mm) rim\n  foot := circle(center: o) hint(r: 26)\n  \
        radius(26mm) foot\n}}\nplate := solid(face(rim), depth: 20mm)\nplate.rim round(r: 3mm) plate.far\n\
        boss := solid(face(foot), from: -30mm, to: -20mm)\npart := solid(plate)\nboss union part\n\
        part.boss.foot {how} part.plate.far\n")
}

/// **A chamfer spilled onto a round about one axis** (#422's rung 2, coaxial): in the meridian the
/// flat is the line `y = 48 − ρ` over the far face to the torus's normal at the seam (ρ = 27,
/// y = 21), then `|q − c| = 31 − ρ` over the torus (`c` its tube's centre) down to it at ρ = 28,
/// turned about the axis: Pappus over that section.
#[test]
fn a_chamfer_spilled_onto_a_round_about_one_axis_is_built() {
    let e = solved(&coaxial("chamfer(d: 2mm)"));
    let got = volume(&e, "part");
    let tube = |rho: f64| 17.0 + (9.0 - (rho - 27.0) * (rho - 27.0)).sqrt();
    let moment = simpson(|rho| rho * (28.0 - rho), 26.0, 27.0, 20000)
        + simpson(|rho| rho * (17.0 + ((31.0 - rho) * (31.0 - rho) - (rho - 27.0) * (rho - 27.0)).sqrt() - tube(rho)),
            27.0, 28.0, 20000);
    let want = volume(&e, "plate") + PI * 26.0 * 26.0 * 10.0 + 2.0 * PI * moment;
    assert!((got - want).abs() <= 1e-9 * want, "{got} against {want}");
}

/// **A round spilled onto a round about one axis** (#422's rung 3, coaxial): the ball of 2 beside
/// the boss stands at ρ = 28, past the seam, so it touches the torus (its centre 5 from the tube's,
/// at y = 17 + √24), turned about the axis.
#[test]
fn a_round_spilled_onto_a_round_about_one_axis_is_built() {
    let e = solved(&coaxial("round(r: 2mm)"));
    let got = volume(&e, "part");
    let yc = 17.0 + 24.0f64.sqrt();
    let touch = 27.0 + 3.0 / 5.0;
    let chain = |rho: f64| if rho <= 27.0 { 20.0 } else { 17.0 + (9.0 - (rho - 27.0) * (rho - 27.0)).sqrt() };
    let ball = |rho: f64| yc - (4.0 - (rho - 28.0) * (rho - 28.0)).sqrt();
    let moment = simpson(|rho| rho * (ball(rho) - chain(rho)), 26.0, 27.0, 20000)
        + simpson(|rho| rho * (ball(rho) - chain(rho)), 27.0, touch, 20000);
    let want = volume(&e, "plate") + PI * 26.0 * 26.0 * 10.0 + 2.0 * PI * moment;
    assert!((got - want).abs() <= 1e-9 * want, "{got} against {want}");
}


/// The spilled chamfer's fill (#422), worked out from `std.FlatFill`'s own words over the part of
/// `rounds/spill.sv` with a boss of `r`: in each half-plane through the boss's axis, between the
/// chain (the far face's plane, then the rim's torus past the seam at 30.54 from the plate's axis)
/// and where the distances from the chain and from the boss's wall, each over 2, sum to one —
/// that height found by bisection, the area's moment about the axis by Gauss–Legendre between the
/// wall, where the chain changes piece and the leg — turned about the axis by the trapezoid rule
/// (each half-plane's moment periodic in its angle).  Nothing of the core is asked.
fn spilled_fill(r: f64) -> f64 {
    // the boss's axis on the far face, from `spill.sv`'s plane and its centre in it
    let (cx, cz) = (-0.110911353 - 8.4, 8.93063091 - 6.7);
    let (seam, tube, below) = (30.54, 3.0, 17.0);
    let chain_distance = |rho: f64, y: f64| if rho <= seam { y - 20.0 } else { (rho - seam).hypot(y - below) - tube };
    let chain_height = |rho: f64| if rho <= seam { Some(20.0) } else {
        let d = rho - seam;
        (d < tube).then(|| below + (tube * tube - d * d).sqrt())
    };
    let (nodes, weights): (Vec<f64>, Vec<f64>) = gauss(16);
    let section = |phi: f64| -> f64 {
        let (s, c) = phi.sin_cos();
        let rho_of = |rb: f64| (cx + rb * c).hypot(cz + rb * s);
        // how high the fill stands over the chain `rb` from the axis
        let height = |rb: f64| -> f64 {
            let rho = rho_of(rb);
            let Some(base) = chain_height(rho) else { return 0.0 };
            let over = |y: f64| chain_distance(rho, y) / 2.0 + (rb - r) / 2.0 - 1.0;
            if over(base) >= 0.0 {
                return 0.0;
            }
            let (mut lo, mut hi) = (base, base + 3.0);
            for _ in 0..100 {
                let m = 0.5 * (lo + hi);
                if over(m) > 0.0 { hi = m } else { lo = m }
            }
            0.5 * (lo + hi) - base
        };
        // the leg: where the height comes to nothing; and where the chain changes piece
        let (mut lo, mut hi) = (r, r + 2.5);
        for _ in 0..100 {
            let m = 0.5 * (lo + hi);
            if height(m) > 0.0 { lo = m } else { hi = m }
        }
        let leg = 0.5 * (lo + hi);
        let mut breaks = vec![r, leg];
        // |c + rb u| = seam: rb² + 2 rb (c·u) + |c|² − seam² = 0
        let (b, k) = (cx * c + cz * s, cx * cx + cz * cz - seam * seam);
        let root = -b + (b * b - k).sqrt();
        if root > r && root < leg {
            breaks.insert(1, root);
        }
        breaks.windows(2).map(|w| {
            let n = 32;
            (0..n).map(|j| {
                let (a, z) = (w[0] + (w[1] - w[0]) * j as f64 / n as f64, w[0] + (w[1] - w[0]) * (j + 1) as f64 / n as f64);
                nodes.iter().zip(&weights).map(|(&x, &wt)| {
                    let rb = 0.5 * (a + z) + 0.5 * (z - a) * x;
                    0.5 * (z - a) * wt * rb * height(rb)
                }).sum::<f64>()
            }).sum::<f64>()
        }).sum()
    };
    let n = 2880;
    (0..n).map(|k| section(std::f64::consts::TAU * k as f64 / n as f64)).sum::<f64>() * std::f64::consts::TAU / n as f64
}

/// Gauss–Legendre's nodes and weights on `[-1, 1]`, by Newton's steps on the Legendre polynomial.
fn gauss(n: usize) -> (Vec<f64>, Vec<f64>) {
    let (mut xs, mut ws) = (Vec::new(), Vec::new());
    for i in 0..n {
        let mut x = (PI * (i as f64 + 0.75) / (n as f64 + 0.5)).cos();
        let mut dp = 0.0;
        for _ in 0..100 {
            let (mut p0, mut p1) = (1.0, x);
            for k in 2..=n {
                let p2 = ((2 * k - 1) as f64 * x * p1 - (k - 1) as f64 * p0) / k as f64;
                p0 = p1;
                p1 = p2;
            }
            dp = n as f64 * (x * p1 - p0) / (x * x - 1.0);
            let step = p1 / dp;
            x -= step;
            if step.abs() < 1e-16 {
                break;
            }
        }
        xs.push(x);
        ws.push(2.0 / ((1.0 - x * x) * dp * dp));
    }
    (xs, ws)
}

/// The spilled part built exactly (#422's rung 2): the chamfer's ring kept short of the seam's
/// normal surface — the cylinder through the seam along the far face's normal — glued there to the
/// flat beyond it, against the round; its fill the quadrature's (`spilled_fill`) to a millionth.
fn spilled_built(r: f64) {
    let e = solved(&spilled(r));
    let got = volume(&e, "boss") - volume(&e, "plate") - PI * r * r * 10.0;
    let want = spilled_fill(r);
    assert!((got - want).abs() <= 1e-6 * want, "r = {r}: {got} against {want}");
}

#[test]
fn a_chamfer_spilled_onto_a_rounded_plate_is_built_exactly() {
    spilled_built(21.7);
}

#[test]
#[cfg_attr(not(feature = "slow"), ignore = "slow tier, a quadrature for each boss")]
fn a_chamfer_spilled_onto_a_rounded_plate_is_built_exactly_at_every_radius() {
    for k in 0..7 {
        spilled_built(21.0 + 0.1 * k as f64);
    }
}

/// **The exact part and the field's mesh agree** (#422): the mesh's volume within what its own
/// triangles stand off the field's surface (the field's distance at each triangle's middle, over
/// its area) of the part built exactly.
#[test]
fn a_spilled_chamfer_built_exactly_agrees_with_the_fields_mesh() {
    let e = solved(&spilled(21.7));
    let i = ent(&e, "boss").i();
    let surface = gcs_core::solid::FieldMesher::new(&e.sketch, i).unwrap().finish().unwrap();
    let field = MaterialField::read(&e.sketch, i, 1e-10).unwrap();
    let off: f64 = surface.triangles.iter().map(|t| {
        let [a, b, c] = t.map(|k| surface.vertices[k as usize]);
        let (u, v) = ([b[0] - a[0], b[1] - a[1], b[2] - a[2]], [c[0] - a[0], c[1] - a[1], c[2] - a[2]]);
        let n = [u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]];
        let area = 0.5 * (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();
        let middle = [0, 1, 2].map(|k| (a[k] + b[k] + c[k]) / 3.0);
        area * field.side(middle).abs()
    }).sum();
    let (exact, meshed) = (volume(&e, "boss"), mesh_volume(&surface));
    assert!((exact - meshed).abs() <= 1.5 * off, "{exact} against the mesh's {meshed}, off by {off} at most");
}

/// `rounds/spill.sv` with a boss of `r`, its foot rounded `ball` in place of the chamfer.
fn spilled_round(r: f64, ball: f64) -> String {
    spilled(r).replace("boss.plate.far chamfer(d: 2mm) boss.foot", &format!("boss.plate.far round(r: {ball}mm) boss.foot"))
}

/// The spilled round's fill (#422's rung 3), worked out from `std.Fill`'s own words over the part
/// of `rounds/spill.sv` with a boss of `r` rounded `ball`: a point beside the edge — above the
/// chain (the far face, then the rim's torus past the seam at 30.54), outside the boss's wall — is
/// the fill's where no ball of that radius whose centre stands at least `ball` from the chain and
/// from the wall reaches it.  The nearest such centre is the point of the chain's offset or the
/// wall's straight off it, where that one stands far enough from the other, or else a point of the
/// curve where both offsets meet (the centre touching both: in the half-plane through the boss's
/// axis at each angle, `ball` above the chain), found by Newton's steps along it.  In each
/// half-plane the fill's height over the chain found by bisection and its moment about the axis by
/// Gauss–Legendre between the wall (read as the square of the way from it, where the ball's arc
/// meets the wall tangent), where the chain changes piece and where the ball touches it;
/// over the angles where the ball touches the round (between where its centre's foot crosses the
/// seam) and a tenth of a radian either side by Gauss–Legendre, the plain ring's section
/// (`root_ring`) elsewhere.  Nothing of the core is asked.
fn spilled_round_fill(r: f64, ball: f64) -> f64 {
    let (cx, cz) = (-0.110911353 - 8.4, 8.93063091 - 6.7);
    let (seam, tube, below) = (30.54, 3.0, 17.0);
    let rho_at = |phi: f64, rb: f64| { let (s, c) = phi.sin_cos(); (cx + rb * c).hypot(cz + rb * s) };
    let point = |phi: f64, rb: f64, y: f64| { let (s, c) = phi.sin_cos(); [cx + rb * c, y, cz + rb * s] };
    // the chain's distance and its unit normal at a point; and the chain's height under it
    let chain = |p: [f64; 3]| -> (f64, [f64; 3]) {
        let rho = p[0].hypot(p[2]);
        if rho <= seam { return (p[1] - 20.0, [0.0, 1.0, 0.0]) }
        let (dr, dy) = (rho - seam, p[1] - below);
        let d = dr.hypot(dy);
        (d - tube, [p[0] / rho * dr / d, dy / d, p[2] / rho * dr / d])
    };
    let chain_height = |rho: f64| if rho <= seam { Some(20.0) } else {
        let d = rho - seam;
        (d < tube).then(|| below + (tube * tube - d * d).sqrt())
    };
    let wall = |p: [f64; 3]| -> (f64, [f64; 3]) {
        let (dx, dz) = (p[0] - cx, p[2] - cz);
        let rb = dx.hypot(dz);
        (rb - r, [dx / rb, 0.0, dz / rb])
    };
    // the centre touching both, in the half-plane at `phi`
    let spine = |phi: f64| -> [f64; 3] {
        let rho = rho_at(phi, r + ball);
        let y = if rho <= seam { 20.0 + ball } else { below + ((tube + ball).powi(2) - (rho - seam).powi(2)).sqrt() };
        point(phi, r + ball, y)
    };
    let dist = |a: [f64; 3], b: [f64; 3]| ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2) + (a[2] - b[2]).powi(2)).sqrt();
    // the distance from `p` (in the half-plane at `phi`) to the nearest centre
    let reach = |phi: f64, p: [f64; 3]| -> f64 {
        let (dc, nc) = chain(p);
        let (dw, nw) = wall(p);
        if dc >= ball && dw >= ball {
            return 0.0;
        }
        let mut best = f64::INFINITY;
        if dc < ball {
            let c = [0, 1, 2].map(|k| p[k] + (ball - dc) * nc[k]);
            if wall(c).0 >= ball { best = best.min(ball - dc) }
        }
        if dw < ball {
            let c = [0, 1, 2].map(|k| p[k] + (ball - dw) * nw[k]);
            if chain(c).0 >= ball { best = best.min(ball - dw) }
        }
        // the nearest point of the spine: Newton's steps on the distance's derivative
        let f = |t: f64| dist(p, spine(t));
        let (mut t, h) = (phi, 1e-5);
        for _ in 0..30 {
            let (d1, d2) = ((f(t + h) - f(t - h)) / (2.0 * h), (f(t + h) - 2.0 * f(t) + f(t - h)) / (h * h));
            if !(d2 > 0.0) { break }
            let step = d1 / d2;
            t -= step;
            if step.abs() < 1e-13 { break }
        }
        best.min(f(t))
    };
    let (nodes, weights) = gauss(8);
    let gl = |a: f64, z: f64, n: usize, g: &dyn Fn(f64) -> f64| -> f64 {
        (0..n).map(|j| {
            let (lo, hi) = (a + (z - a) * j as f64 / n as f64, a + (z - a) * (j + 1) as f64 / n as f64);
            nodes.iter().zip(&weights).map(|(&x, &w)| 0.5 * (hi - lo) * w * g(0.5 * (lo + hi) + 0.5 * (hi - lo) * x)).sum::<f64>()
        }).sum()
    };
    let section = |phi: f64| -> f64 {
        let height = |rb: f64| -> f64 {
            let Some(base) = chain_height(rho_at(phi, rb)) else { return 0.0 };
            let inside = |y: f64| reach(phi, point(phi, rb, y)) >= ball;
            if !inside(base + 1e-12) {
                return 0.0;
            }
            let (mut lo, mut hi) = (base, base + 2.0 * ball);
            for _ in 0..60 {
                let m = 0.5 * (lo + hi);
                if inside(m) { lo = m } else { hi = m }
            }
            0.5 * (lo + hi) - base
        };
        let (mut lo, mut hi) = (r, r + 2.0 * ball);
        for _ in 0..60 {
            let m = 0.5 * (lo + hi);
            if height(m) > 0.0 { lo = m } else { hi = m }
        }
        let touch = 0.5 * (lo + hi);
        let mut breaks = vec![r, touch];
        let (s, c) = phi.sin_cos();
        let (b, k) = (cx * c + cz * s, cx * cx + cz * cz - seam * seam);
        let root = -b + (b * b - k).sqrt();
        if root > r && root < touch {
            breaks.insert(1, root);
        }
        // (the ball's arc meets the wall tangent, its height a square root of the way from it: read
        // as `rb = r + s²` from the wall)
        let first = breaks[1] - r;
        gl(0.0, 1.0, 4, &|t| { let rb = r + first * t * t; 2.0 * first * t * rb * height(rb) })
            + breaks[1..].windows(2).map(|w| gl(w[0], w[1], 4, &|rb| rb * height(rb))).sum::<f64>()
    };
    // where the ball's centre's foot crosses the seam: |c + (r + ball) u| = seam
    let (d, rc) = (cx.hypot(cz), r + ball);
    let half = ((seam * seam - d * d - rc * rc) / (2.0 * d * rc)).acos();
    let toward = (-cz).atan2(-cx) + PI;
    let wedge = [toward - half - 0.1, toward + half + 0.1];
    let ring = crate::common::rounds::root_ring(r, ball) / (2.0 * PI);
    ring * (2.0 * PI - (wedge[1] - wedge[0]))
        + gl(wedge[0], toward - half, 4, &section) + gl(toward - half, toward + half, 64, &section)
        + gl(toward + half, wedge[1], 4, &section)
}

/// The spilled round built exactly (#422's rung 3): the ring a stretch between where its ball
/// touches the seam, glued at both ends to the ball rolled against the rim's torus between;
/// its fill the quadrature's (`spilled_round_fill`) to a millionth.
fn spilled_round_built(r: f64) {
    let e = solved(&spilled_round(r, 2.0));
    let got = volume(&e, "boss") - volume(&e, "plate") - PI * r * r * 10.0;
    let want = spilled_round_fill(r, 2.0);
    assert!((got - want).abs() <= 1e-6 * want, "r = {r}: {got} against {want}");
}

#[test]
fn a_round_spilled_onto_a_rounded_plate_is_built_exactly() {
    spilled_round_built(21.7);
}

#[test]
#[cfg_attr(not(feature = "slow"), ignore = "slow tier, a quadrature for each boss")]
fn a_round_spilled_onto_a_rounded_plate_is_built_exactly_at_every_radius() {
    for k in 0..7 {
        spilled_round_built(21.0 + 0.1 * k as f64);
    }
}

/// A block drawn on the side plane (y across, z up) and 40 long in x: its top at z = 10 out to
/// y = 20, then a slope down at 45° to y = 30, that edge rounded 2 — the top going on tangent into
/// the round at y = 20 − 2 tan 22.5°, the round into the slope — and a rib 20 long on the top, its
/// front wall at y = 17, 6 tall; then `rest`.
fn sloped(rest: &str) -> String {
    format!("unit mm\nuse std (round, chamfer)\nin std.side {{\n  b0 := point\n  b1 := point\n  b2 := point\n  \
        b3 := point\n  fix((0, 0)) b0\n  fix((30, 0)) b1\n  fix((20, 10)) b2\n  fix((0, 10)) b3\n  \
        (bottom := line(b0, b1)) -> (slope := line(b1, b2)) -> (top := line(b2, b3)) -> (back := line(b3, b0)) -> close\n  \
        outline := face(bottom, slope, top, back)\n  r0 := point\n  r1 := point\n  r2 := point\n  r3 := point\n  \
        fix((12, 10)) r0\n  fix((17, 10)) r1\n  fix((17, 16)) r2\n  fix((12, 16)) r3\n  \
        (rb := line(r0, r1)) -> (rf := line(r1, r2)) -> (rt := line(r2, r3)) -> (rk := line(r3, r0)) -> close\n  \
        ribbed := face(rb, rf, rt, rk)\n}}\nblock := solid(outline, from: 0mm, to: 40mm)\nblock.top round(r: 2mm) block.slope\n\
        rib := solid(ribbed, from: 10mm, to: 30mm)\npart := solid(block)\nrib union part\n{rest}")
}

/// **A flat across a chain of three faces** (#422's rung 5): the rib's front foot chamfered 4 — its
/// leg runs from the top across the round onto the slope (to y = 21).  In the section the flat is
/// each face's sum of distances with the wall in turn — a line over the top, a parabola's arc over
/// the round (`|q − c| = 23 − y`), a line over the slope — changing face on the chain's normal at
/// each seam: a prism of five strokes over the chain's three, against a quadrature of the section
/// read by the chain's nearest face.
#[test]
fn a_chamfer_spilled_across_a_chain_of_three_faces_is_built() {
    let e = solved(&sloped("part.rib.rf chamfer(d: 4mm) part.block.top\n"));
    let got = volume(&e, "part");
    let t = (PI / 8.0).tan();
    let (y1, c) = (20.0 - 2.0 * t, [20.0 - 2.0 * t, 8.0]);
    let y2 = c[0] + 2.0 * (PI / 4.0).sin();
    let chain = |y: f64| if y <= y1 { 10.0 } else if y <= y2 { c[1] + (4.0 - (y - c[0]).powi(2)).sqrt() } else { 30.0 - y };
    // the distance from the chain's nearest face, above it
    let distance = |y: f64, z: f64| {
        let (dy, dz) = (y - c[0], z - c[1]);
        if y <= y1 { z - 10.0 } else if dy <= dz { dy.hypot(dz) - 2.0 } else { (y + z - 30.0) / 2f64.sqrt() }
    };
    let top = |y: f64| {
        let (mut lo, mut hi) = (chain(y), chain(y) + 5.0);
        for _ in 0..100 {
            let m = 0.5 * (lo + hi);
            if distance(y, m) / 4.0 + (y - 17.0) / 4.0 > 1.0 { hi = m } else { lo = m }
        }
        0.5 * (lo + hi)
    };
    // where the flat crosses the round's normal at its second seam
    let (mut lo, mut hi) = (y1, 21.0);
    for _ in 0..100 {
        let m = 0.5 * (lo + hi);
        if top(m) - c[1] > m - c[0] { lo = m } else { hi = m }
    }
    let yx = 0.5 * (lo + hi);
    let (nodes, weights) = gauss(16);
    let mut breaks = vec![17.0, y1, yx, y2, 21.0];
    breaks.sort_by(f64::total_cmp);
    let area: f64 = breaks.windows(2).map(|w| (0..8).map(|j| {
        let (a, z) = (w[0] + (w[1] - w[0]) * j as f64 / 8.0, w[0] + (w[1] - w[0]) * (j + 1) as f64 / 8.0);
        nodes.iter().zip(&weights).map(|(&x, &wt)| { let y = 0.5 * (a + z) + 0.5 * (z - a) * x; 0.5 * (z - a) * wt * (top(y) - chain(y)) })
            .sum::<f64>()
    }).sum::<f64>()).sum();
    let want = volume(&e, "block") + 5.0 * 6.0 * 20.0 + 20.0 * area;
    assert!((got - want).abs() <= 1e-9 * want, "{got} against {want}");
}

/// A wedge 40 along x and 24 deep (y from −24 to 0), its top sloping from 20 high at x = 0 down to
/// 10 at x = 40, the slope's low edge rounded 3 — the slope going on tangent into the round along a
/// line — and a boss of 6 standing up through the slope about (30, −12), so its foot on the slope
/// is an ellipse no line or coaxial circle carries; then `rest`.
fn tilted(rest: &str) -> String {
    format!("unit mm\nuse std (round, chamfer)\nin std.front {{\n  w0 := point\n  w1 := point\n  w2 := point\n  \
        w3 := point\n  fix((0, 0)) w0\n  fix((40, 0)) w1\n  fix((40, 10)) w2\n  fix((0, 20)) w3\n  \
        (wb := line(w0, w1)) -> (we := line(w1, w2)) -> (wt := line(w2, w3)) -> (ww := line(w3, w0)) -> close\n  \
        side_f := face(wb, we, wt, ww)\n}}\nin std.top {{\n  bc := point\n  fix((30, -12)) bc\n  \
        boss_k := circle(center: bc) hint(r: 6)\n  radius(6mm) boss_k\n}}\n\
        wedge := solid(side_f, from: 0mm, to: 24mm)\nwedge.we round(r: 3mm) wedge.wt\n\
        boss := solid(face(boss_k), from: 0mm, to: 30mm)\npart := solid(wedge)\nboss union part\n{rest}")
}

/// The tilted chamfer's fill (#422's rung 5), worked out from `std.FlatFill`'s own words: about the
/// boss's axis (upright through (30, −12)), between the chain — the slope `z = 20 − x/4`, then past
/// its seam at x ≈ 37.73 the round of 3 about the line through (37, 7.66) along y — and where the
/// distances from the chain's nearest face and from the boss's wall, each over 2, sum to one: at
/// each angle and distance from the axis (between the wall and 2 out from it, the leg) that height
/// found by bisection, the moment by Gauss–Legendre split where the chain changes face, and the
/// angles likewise split where that change comes within the leg.  Nothing of the core is asked.
fn tilted_fill() -> f64 {
    let n1 = { let k = 1.0 / 17f64.sqrt(); [k, 4.0 * k] };
    // the round's centre, 3 inside both the slope and the east face
    let cx = 37.0;
    let cz = 10.0 + (-3.0 + 3.0 * n1[0]) / n1[1];
    let seam = [cx + 3.0 * n1[0], cz + 3.0 * n1[1]];
    let along = [n1[1], -n1[0]];
    let chain = |x: f64| if x <= seam[0] { 20.0 - x / 4.0 } else { cz + (9.0 - (x - cx).powi(2)).sqrt() };
    let distance = |x: f64, z: f64| {
        if (x - seam[0]) * along[0] + (z - seam[1]) * along[1] <= 0.0 {
            (x - 40.0) * n1[0] + (z - 10.0) * n1[1]
        } else {
            (x - cx).hypot(z - cz) - 3.0
        }
    };
    let (nodes, weights) = gauss(16);
    let gl = |a: f64, z: f64, n: usize, g: &dyn Fn(f64) -> f64| -> f64 {
        (0..n).map(|j| {
            let (lo, hi) = (a + (z - a) * j as f64 / n as f64, a + (z - a) * (j + 1) as f64 / n as f64);
            nodes.iter().zip(&weights).map(|(&t, &w)| 0.5 * (hi - lo) * w * g(0.5 * (lo + hi) + 0.5 * (hi - lo) * t)).sum::<f64>()
        }).sum()
    };
    let section = |phi: f64| -> f64 {
        let c = phi.cos();
        let height = |rho: f64| {
            let x = 30.0 + rho * c;
            let base = chain(x);
            let over = |z: f64| distance(x, z) / 2.0 + (rho - 6.0) / 2.0 - 1.0;
            if over(base) >= 0.0 {
                return 0.0;
            }
            let (mut lo, mut hi) = (base, base + 3.0);
            for _ in 0..100 {
                let m = 0.5 * (lo + hi);
                if over(m) > 0.0 { hi = m } else { lo = m }
            }
            0.5 * (lo + hi) - base
        };
        let mut breaks = vec![6.0, 8.0];
        let kink = (seam[0] - 30.0) / c;
        if c > 0.0 && kink > 6.0 && kink < 8.0 {
            breaks.insert(1, kink);
        }
        breaks.windows(2).map(|w| gl(w[0], w[1], 4, &|rho| rho * height(rho))).sum()
    };
    let turn = ((seam[0] - 30.0) / 8.0).acos();
    gl(-turn, turn, 16, &section) + gl(turn, 2.0 * PI - turn, 32, &section)
}

/// **A flat along a traced edge spilled onto a round** (#422's rung 5): the boss's foot on the
/// wedge's slope chamfered 2 — its leg runs past the seam where the slope goes on into the round of
/// its low edge.  The foot is an ellipse, so the flat is cut along it, fitted (`fillet::flat_along`),
/// and kept short of the seam's normal surface — the plane through the seam along the slope's
/// normal — beyond which the flat against the round, sectioned square to the foot between where
/// its leg on the slope crosses the seam, is glued to it: its fill the quadrature's to 1e-8.
#[test]
fn a_chamfer_along_a_traced_edge_spilled_onto_a_round_is_built() {
    let e = solved(&tilted("part.boss chamfer(d: 2mm) part.wedge.wt\n"));
    let bare = solved(&tilted(""));
    let got = volume(&e, "part") - volume(&bare, "part");
    let want = tilted_fill();
    assert!((got - want).abs() <= 1e-8 * want, "{got} against {want}");
}

/// The tilted round's fill (#422's rung 5), worked out from `std.Fill`'s own words, as
/// `spilled_round_fill` is: about the boss's axis (upright through (30, −12)), above the chain (the
/// slope, then past its seam the round of 3 about the line through (37, 7.66) along y) and outside
/// the wall, a point is the fill's where no ball of `ball` whose centre stands at least that far
/// from the chain's nearest face and from the wall reaches it — the nearest centre straight off one
/// face where it stands far enough from the other, or a point of the curve where both offsets meet
/// (`ball` above the chain on the cylinder of `6 + ball` about the axis: the slope's offset, then
/// the round's), found by Newton's steps.
/// The fill's height at each angle and distance from the axis by bisection, its moment by
/// Gauss–Legendre between the wall (read as the square of the way from it), where the chain changes
/// face and where the ball touches it; the angles split where the centre's path crosses the seam's
/// offset.  Nothing of the core is asked.
fn tilted_round_fill(ball: f64) -> f64 {
    let n1 = { let k = 1.0 / 17f64.sqrt(); [k, 4.0 * k] };
    let (cx, cz) = (37.0, 10.0 + (-3.0 + 3.0 * n1[0]) / n1[1]);
    let seam = [cx + 3.0 * n1[0], cz + 3.0 * n1[1]];
    let along = [n1[1], -n1[0]];
    let point = |phi: f64, rho: f64, z: f64| { let (s, c) = phi.sin_cos(); [30.0 + rho * c, -12.0 + rho * s, z] };
    // the chain's nearest face's distance and its unit normal at a point; the chain's height
    let chain = |p: [f64; 3]| -> (f64, [f64; 3]) {
        let (x, z) = (p[0], p[2]);
        if (x - seam[0]) * along[0] + (z - seam[1]) * along[1] <= 0.0 {
            ((x - 40.0) * n1[0] + (z - 10.0) * n1[1], [n1[0], 0.0, n1[1]])
        } else {
            let d = (x - cx).hypot(z - cz);
            (d - 3.0, [(x - cx) / d, 0.0, (z - cz) / d])
        }
    };
    let chain_height = |x: f64| if x <= seam[0] { 20.0 - x / 4.0 } else { cz + (9.0 - (x - cx).powi(2)).max(0.0).sqrt() };
    let wall = |p: [f64; 3]| -> (f64, [f64; 3]) {
        let (dx, dy) = (p[0] - 30.0, p[1] + 12.0);
        let rho = dx.hypot(dy);
        (rho - 6.0, [dx / rho, dy / rho, 0.0])
    };
    // the centre touching both at the angle `phi`: `ball` above the chain, `6 + ball` from the axis
    // (the slope's offset short of the seam's, the round's past it)
    let spine = |phi: f64| -> [f64; 3] {
        let x = point(phi, 6.0 + ball, 0.0)[0];
        let z = if x <= seam[0] + ball * n1[0] { 10.0 + (ball - (x - 40.0) * n1[0]) / n1[1] }
            else { cz + ((3.0 + ball).powi(2) - (x - cx).powi(2)).sqrt() };
        point(phi, 6.0 + ball, z)
    };
    let dist = |a: [f64; 3], b: [f64; 3]| ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2) + (a[2] - b[2]).powi(2)).sqrt();
    let reach = |phi: f64, p: [f64; 3]| -> f64 {
        let (dc, nc) = chain(p);
        let (dw, nw) = wall(p);
        if dc >= ball && dw >= ball {
            return 0.0;
        }
        let mut best = f64::INFINITY;
        if dc < ball {
            let c = [0, 1, 2].map(|k| p[k] + (ball - dc) * nc[k]);
            if wall(c).0 >= ball { best = best.min(ball - dc) }
        }
        if dw < ball {
            let c = [0, 1, 2].map(|k| p[k] + (ball - dw) * nw[k]);
            if chain(c).0 >= ball { best = best.min(ball - dw) }
        }
        let f = |t: f64| dist(p, spine(t));
        let (mut t, h) = (phi, 1e-5);
        for _ in 0..30 {
            let (d1, d2) = ((f(t + h) - f(t - h)) / (2.0 * h), (f(t + h) - 2.0 * f(t) + f(t - h)) / (h * h));
            if !(d2 > 0.0) { break }
            let step = d1 / d2;
            t -= step;
            if step.abs() < 1e-13 { break }
        }
        best.min(f(t))
    };
    let (nodes, weights) = gauss(8);
    let gl = |a: f64, z: f64, n: usize, g: &dyn Fn(f64) -> f64| -> f64 {
        (0..n).map(|j| {
            let (lo, hi) = (a + (z - a) * j as f64 / n as f64, a + (z - a) * (j + 1) as f64 / n as f64);
            nodes.iter().zip(&weights).map(|(&x, &w)| 0.5 * (hi - lo) * w * g(0.5 * (lo + hi) + 0.5 * (hi - lo) * x)).sum::<f64>()
        }).sum()
    };
    let section = |phi: f64| -> f64 {
        let height = |rho: f64| -> f64 {
            let base = chain_height(point(phi, rho, 0.0)[0]);
            let inside = |z: f64| reach(phi, point(phi, rho, z)) >= ball;
            if !inside(base + 1e-12) {
                return 0.0;
            }
            let (mut lo, mut hi) = (base, base + 3.0 * ball);
            for _ in 0..60 {
                let m = 0.5 * (lo + hi);
                if inside(m) { lo = m } else { hi = m }
            }
            0.5 * (lo + hi) - base
        };
        let (mut lo, mut hi) = (6.0, 6.0 + 3.0 * ball);
        for _ in 0..60 {
            let m = 0.5 * (lo + hi);
            if height(m) > 0.0 { lo = m } else { hi = m }
        }
        let touch = 0.5 * (lo + hi);
        let mut breaks = vec![6.0, touch];
        let kink = (seam[0] - 30.0) / phi.cos();
        if phi.cos() > 0.0 && kink > 6.0 && kink < touch {
            breaks.insert(1, kink);
        }
        let first = breaks[1] - 6.0;
        gl(0.0, 1.0, 4, &|t| { let rho = 6.0 + first * t * t; 2.0 * first * t * rho * height(rho) })
            + breaks[1..].windows(2).map(|w| gl(w[0], w[1], 4, &|rho| rho * height(rho))).sum::<f64>()
    };
    // where the centre's path crosses the seam's offset: 30 + (6 + ball) cos φ on the offset seam
    let turn = ((seam[0] + ball * n1[0] - 30.0) / (6.0 + ball)).acos();
    gl(-turn, turn, 8, &section) + gl(turn, 2.0 * PI - turn, 12, &section)
}

/// **A round along a traced edge spilled onto a round** (#422's rung 5): the boss's foot on the
/// wedge's slope rounded 3 — beside the slope's low edge its ball touches the round past the seam.
/// The ball is rolled along the foot (`fillet::roll`) with the stretch between the planes of its
/// arc where it touches the seam cut away, and there the ball rolled against the round, sectioned
/// square to the foot, glued at both: its fill the quadrature's to a millionth.
#[test]
fn a_round_along_a_traced_edge_spilled_onto_a_round_is_built() {
    let e = solved(&tilted("part.boss round(r: 3mm) part.wedge.wt\n"));
    let bare = solved(&tilted(""));
    let got = volume(&e, "part") - volume(&bare, "part");
    let want = tilted_round_fill(3.0);
    assert!((got - want).abs() <= 1e-6 * want, "{got} against {want}");
}
