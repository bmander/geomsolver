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
/// and the boss.  One of 21.7, whose leg lands on the round, stays the field's.
#[test]
fn a_chamfer_whose_leg_stays_short_of_the_seam_is_built() {
    let r = 19.5;
    let e = solved(&spilled(r));
    let got = volume(&e, "boss");
    let ring = 2.0 * PI * (r + 2.0 / 3.0) * 2.0;
    let want = volume(&e, "plate") + PI * r * r * 10.0 + ring;
    assert!((got - want).abs() <= 1e-9 * want, "{got} against {want}");
    let e = solved(&spilled(21.7));
    let say = Say { stage: &|_: &str| {}, mark: &|_| {} };
    let refused = export::exact(&e.sketch, ent(&e, "boss").i(), None, None, &say).err()
        .map(|r| format!("{r:?}")).unwrap_or_default();
    assert!(refused.contains("reaches across the seam"), "{refused}");
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

/// A block 40 × 24 × 12 (y from 0 to 12), its far face's north edge rounded 3 — the far face flat
/// to z = 21, then the round, a cylinder about the line y = 9, z = 21 — and a rib 20 long, 4.5 wide
/// and 6 tall standing on the far face along x, its north wall at z = 19.5; then `rest`.
fn ledge(rest: &str) -> String {
    format!("unit mm\nuse std (round, chamfer)\nin std.front {{\n  b0 := point\n  b1 := point\n  b2 := point\n  \
        b3 := point\n  fix((0, 0)) b0\n  fix((40, 0)) b1\n  fix((40, 24)) b2\n  fix((0, 24)) b3\n  \
        (south := line(b0, b1)) -> (east := line(b1, b2)) -> (north := line(b2, b3)) -> (west := line(b3, b0)) -> close\n  \
        outline := face(south, east, north, west)\n  r0 := point\n  r1 := point\n  r2 := point\n  r3 := point\n  \
        fix((10, 15)) r0\n  fix((30, 15)) r1\n  fix((30, 19.5)) r2\n  fix((10, 19.5)) r3\n  \
        (rs := line(r0, r1)) -> (re := line(r1, r2)) -> (rn := line(r2, r3)) -> (rw := line(r3, r0)) -> close\n  \
        ribbed := face(rs, re, rn, rw)\n}}\nblock := solid(outline, depth: 12mm)\nblock.north round(r: 3mm) block.far\n\
        rib := solid(ribbed, from: -18mm, to: -12mm)\npart := solid(block)\nrib union part\n{rest}")
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
    let e = solved(&ledge("part.rib.rn chamfer(d: 2mm) part.block.far\n"));
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
