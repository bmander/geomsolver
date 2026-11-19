//! **A round over a rounded body** (#879): a box's four upright edges rounded, then its rims
//! rounded or chamfered round the corners' arcs as one tangent run — the faces a round makes
//! listed to a block by the names the body's boundary has (`round`, `flat`), which faces meet read
//! in the layer the round that asks stands in, and the kernel's meeting of two rounds tangent to
//! one face ending at the point where both touch it.  Each volume is the closed form's to 1e-9.
use gcs_core::brep::props;
use gcs_core::program::Code;
use std::f64::consts::PI;

use crate::common::linked::build as elaborated;
use crate::common::rounds::{exact, exported, kinds, square_run};
use crate::common::{close, text::solved};

/// A box 50 × 25 × 10 drawn on the top plane, its sides `ab`, `bc`, `cd`, `da`, its caps `near`
/// (z = 10) and `far` (z = 0).
const BOX: &str = "unit mm\nuse std\nuse std.solids\nuse std.rounds (round, chamfer)\n\
    in std.top {\n  loop := step(u: 50mm, v: 0mm) (ab := line) -> step(v: 25mm) (bc := line) ->\n\
    step(u: 0mm) (cd := line) -> step(v: 0mm) (da := line) -> close\n}\n\
    box := solid(std.solids.Prism(loop, from: 0mm, to: 10mm))\n";

/// The four upright edges rounded 5, the statement in the block given `name` (`vert := `) or none.
fn upright(name: &str) -> String {
    format!("repeat f in box {{ repeat g in box {{ when f parallel std.z and g parallel std.z \
             and f != g {{ {name}f round(r: 5mm) g }} }} }}\n")
}

/// The box less its four corners rounded 5.
const CORNERED: f64 = 50.0 * 25.0 * 10.0 - (4.0 - PI) * 25.0 * 10.0;
/// The straight edges of a rim of the box with its corners rounded 5.
const STRAIGHT: f64 = 2.0 * (40.0 + 15.0);

/// What a ball of radius `r` cannot reach along a rim of the cornered box (`whole`), or round its
/// four arcs alone: its section along the straight edges and turned once about an arc's axis
/// (Pappus: the section's centroid stands `(10 − 3π) / (3 (4 − π)) r` in from each face).
fn rounded(r: f64, whole: bool) -> f64 {
    let inset = (10.0 - 3.0 * PI) / (3.0 * (4.0 - PI)) * r;
    square_run(r) * (2.0 * PI * (5.0 - inset) + if whole { STRAIGHT } else { 0.0 })
}

/// What a flat `d` along each face takes of a rim of the cornered box: its triangle the same.
fn chamfered(d: f64) -> f64 {
    0.5 * d * d * (2.0 * PI * (5.0 - d / 3.0) + STRAIGHT)
}

/// **The rim follows the corners' rounds** (`rounds/rim.sv`): the top rim rounded 1 and the bottom
/// rim chamfered 1 on a solid of the box whose upright edges are rounded 5 — a run along each
/// straight edge and a quarter ring round each arc, glued at the seams.  Ten planes (the caps,
/// the sides, the chamfer's four flats), eight cylinders (the corners, the round's four runs),
/// four tori and four cones round the arcs.
#[test]
fn a_rim_rounded_over_rounded_corners_is_one_tangent_run() {
    let e = solved(include_str!("../../examples/rounds/rim.sv"));
    assert!(e.diags.is_empty(), "{:?}", e.diags);
    let b = exported(&e, "part");
    close(props::volume(&b), CORNERED - rounded(1.0, true) - chamfered(1.0));
    assert_eq!(kinds(&b), [("cone", 4), ("cylinder", 8), ("plane", 10), ("torus", 4)].into());
}

/// **The top and the bottom alike, a round and a chamfer alike**: each of the four on its own.
#[test]
fn a_rim_over_rounded_corners_is_built_on_either_cap() {
    for cap in ["near", "far"] {
        for (word, taken) in [("round(r: 1mm)", rounded(1.0, true)),
                              ("chamfer(d: 1mm)", chamfered(1.0))] {
            let e = solved(&format!("{BOX}{}part := solid(box)\npart.box.{cap} {word} part.box\n",
                                    upright("")));
            assert!(e.diags.is_empty(), "{cap} {word}: {:?}", e.diags);
            close(props::volume(&exact(&e, "part")), CORNERED - taken);
        }
    }
}

/// **A round over the face a round made** ([0.149]): the corners' round named, and the top
/// rounded against its faces by a statement that names it — named itself or not — is the four
/// quarter rings, each ending square at the seams where a side goes on.  The round's face is
/// listed as the body's boundary names it, `vert.taken.round`: named by its set (`run`), a block
/// over `box.vert` paired the top with a face the body has not, and made nothing.
#[test]
fn a_round_over_a_named_rounds_faces_rounds_their_edges() {
    for rim in ["rim := ", ""] {
        for (cap, word, taken) in [("near", "round(r: 1mm)", rounded(1.0, false)),
                                   ("far", "chamfer(d: 1mm)", 0.5 * 2.0 * PI * (5.0 - 1.0 / 3.0))] {
            let e = elaborated(&format!("{BOX}{}{rim}box.{cap} {word} box.vert\n",
                                        upright("vert := ")));
            assert!(e.diags.is_empty(), "{rim}{cap} {word}: {:?}", e.diags);
            close(props::volume(&exact(&e, "box")), CORNERED - taken);
        }
    }
}

/// **Which faces meet is read where the round that asks reads them** (#879): `ab` and `bc` meet
/// in the box the corner's round reads, and no longer in the one a round over its face reads.
/// Read off the finished body alone, the corner's `when b meets a` came out false once the second
/// round stood over it, the corner was not made, and the elaboration went round until it gave up
/// with the second round naming nothing (W116).  One corner, and a rim over its face.
#[test]
fn faces_meet_in_the_layer_of_the_round_that_asks() {
    let e = elaborated(&format!("{BOX}vert := box.ab round(r: 5mm) box.bc\n\
                                 rim := box.near round(r: 1mm) box.vert\n"));
    assert!(e.diags.is_empty(), "{:?}", e.diags);
    // one corner of the four the arcs alone take round
    let arc = rounded(1.0, false) / 4.0;
    close(props::volume(&exact(&e, "box")), 12500.0 - square_run(5.0) * 10.0 - arc);
    // each round's faces under its own name, a name once however many faces bear it
    let listed = gcs_core::solid::faces::face_paths(&e.sketch, crate::common::ent(&e, "box").i());
    let names: Vec<String> = listed.iter().map(|f| f.text()).collect();
    for want in ["vert.taken.round", "rim.taken.round"] {
        assert_eq!(names.iter().filter(|n| *n == want).count(), 1, "{names:?}");
    }
}

/// **A path that names no face still says so** (W116), with a round's faces among those it could
/// have named.
#[test]
fn a_round_over_no_face_is_said() {
    let e = elaborated(&format!("{BOX}vert := box.ab round(r: 5mm) box.bc\n\
                                 rim := box.near round(r: 1mm) box.verts\n"));
    let said: Vec<&str> = e.diags.iter().filter(|d| d.code == Code::W116)
        .map(|d| d.message.as_str()).collect();
    let names = |m: &str| m.contains("`box.verts` names no face of `box`") && m.contains("`vert`");
    assert!(matches!(said[..], [m] if names(m)),
            "{:?}", e.diags);
}

/// **Rounds that name nothing of each other each read the body before either, on either cap**
/// (#879): a rim's run and a corner's round then only cross at the box's sharp corner — the
/// rim's cylinder and the corner's meet in a curve from the point where both touch a side to the
/// point where the rim's touches the cap.  The kernel's march stepped across the first and ended
/// past it, off the corner's face; the curve ran through that vertex twice, one stretch of it was
/// never laid, and two cells joined across it: the top was refused where the bottom, its steps
/// falling otherwise, was built.  Now both are, and to one volume.
#[test]
fn rounds_that_cross_at_a_corner_are_built_on_either_cap() {
    for word in ["round(r: 1mm)", "chamfer(d: 1mm)"] {
        let volume = |cap: &str| {
            let e = solved(&format!("{BOX}{}box.{cap} {word} box\n", upright("")));
            props::volume(&exact(&e, "box"))
        };
        let (near, far) = (volume("near"), volume("far"));
        close(near, far);
        // short of the box with its corners rounded, by less than the rim taken whole
        assert!(near < CORNERED && near > CORNERED - 4.0 * rounded(1.0, true), "{word}: {near}");
    }
}

/// **A round that does not build is said by its body and its label** (#879): its solid is named
/// by the use of the word that made it (`#17.0.….taken`), which no document wrote, so the recipe
/// says what a failure is to call it.
#[test]
fn a_rounds_node_says_its_body_and_its_label() {
    let e = solved(&format!("{BOX}vert := box.ab round(r: 5mm) box.bc\n\
                             box.near round(r: 1mm) box.cd\n"));
    let body = crate::common::ent(&e, "box").i();
    let recipe = gcs_core::solid::cad::recipe(&e.sketch, body).unwrap();
    let said: Vec<String> = gcs_core::brep::recipe::Recipe::read(&recipe).unwrap().nodes.iter()
        .filter(|n| n.said.is_some()).map(|n| n.called()).collect();
    assert_eq!(said, ["`box`'s round or chamfer `vert.taken`", "`box`'s round or chamfer `taken`"]);
}

/// A bar 40 × 20 and a rib across its far side (30 to 60, 17 to 23, its far side `e2`), each 10
/// tall on the top plane, `second` unioned into a solid of `first`, and the rib's far top edge
/// rounded 5 over the solid they leave.
fn bar_and_rib(first: &str, second: &str) -> String {
    format!("unit mm\nuse std\nuse std.solids\nuse std.solids (union)\nuse std.rounds (round)\n\
             in std.top {{\n  long := step(u: 40mm, v: 0mm) (ab := line) -> \
             step(v: 20mm) (bc := line) -> step(u: 0mm) (cd := line) -> \
             step(v: 0mm) (da := line) -> close\n  t0 := point\n  fix((30, 17)) t0\n  \
             t1 := point\n  fix((60, 17)) t1\n  t2 := point\n  fix((60, 23)) t2\n  \
             t3 := point\n  fix((30, 23)) t3\n  \
             (e0 := line(t0, t1)) -> (e1 := line(t1, t2)) -> (e2 := line(t2, t3)) -> \
             (e3 := line(t3, t0)) -> close\n  tab := face(e0, e1, e2, e3)\n}}\n\
             bar := solid(std.solids.Prism(long, from: 0mm, to: 10mm))\n\
             rib := solid(std.solids.Prism(tab, from: 0mm, to: 10mm))\n\
             plate := solid({first})\n{second} union plate\n\
             part := solid(plate)\npart.plate.rib.near round(r: 5mm) part.plate.rib.e2\n")
}

/// **A section stands against a surface, not the faces it is named by** (#952): a plate's top in
/// the pieces the union of its operands leaves it, each named by its own, is one plane to a round,
/// so a ball whose touch lands past the edge between two pieces stands against the same plane
/// there.  The rib unioned into the bar: of the top, the bar's piece holds where both are, and the
/// rib's is three wide beside its far side over the bar, so the round along that side touches the
/// top past the edge between them over a third of its length — refused as a seam into the next
/// face of a chain, one not along the edge, where nothing goes on but the plane itself.  The plate
/// less the run's square section over the rib's 30, either way round the union.
#[test]
fn a_round_over_a_top_in_pieces_stands_against_one_plane() {
    let want = (40.0 * 20.0 + 10.0 * 3.0 + 20.0 * 6.0) * 10.0 - 30.0 * square_run(5.0);
    for (first, second) in [("bar", "rib"), ("rib", "bar")] {
        let e = solved(&bar_and_rib(first, second));
        assert!(e.diags.is_empty(), "{first} first: {:?}", e.diags);
        close(props::volume(&exported(&e, "part")), want);
    }
}

/// A disc of 50 and a lobe of 20 whose centre stands 40 out (`core` and `ear`), each 10 tall on
/// the top plane, `second` unioned into a solid of `first`; their upright edges rounded 10 where
/// they meet, and the lobe's top rim rounded 5 over the solid they leave.
fn disc_and_lobe(first: &str, second: &str) -> String {
    format!("unit mm\nuse std\nuse std.solids\nuse std.solids (union)\nuse std.rounds (round)\n\
             in std.top {{\n  o := point\n  fix((0, 0)) o\n  \
             big := circle(center: o) hint(r: 50)\n  radius(50mm) big\n  h := point\n  \
             fix((40, 0)) h\n  lobe := circle(center: h) hint(r: 20)\n  \
             radius(20mm) lobe\n}}\n\
             core := solid(std.solids.Prism(face(big), from: 0mm, to: 10mm))\n\
             ear := solid(std.solids.Prism(face(lobe), from: 0mm, to: 10mm))\n\
             plate := solid({first})\n{second} union plate\n\
             repeat f in plate {{ repeat g in plate {{ when f parallel std.z and g parallel std.z \
             and f != g {{ f round(r: 10mm) g }} }} }}\n\
             part := solid(plate)\npart.plate.ear.near round(r: 5mm) part.plate.ear\n")
}

/// **A rim over a plate of pieces** (#952, a lobed flange's): the lobe unioned into the disc
/// leaves its piece of the top a crescent, narrower than the ball's reach where the rounds of 10
/// run into the rim, so the rim's ring touches the top past the arc where the disc's piece begins
/// — refused as a seam running only part of the ring, though both pieces are one plane.  The same
/// solid as the disc unioned into the lobe, whose piece of the top is its whole disc, reached
/// nowhere past: one volume, and faces of the same kinds.
#[test]
fn a_rim_over_a_plate_of_pieces_is_the_rim_over_one_top() {
    let built = |first: &str, second: &str| {
        let e = solved(&disc_and_lobe(first, second));
        assert!(e.diags.is_empty(), "{first} first: {:?}", e.diags);
        exported(&e, "part")
    };
    let (crescent, whole) = (built("core", "ear"), built("ear", "core"));
    close(props::volume(&crescent), props::volume(&whole));
    assert_eq!(kinds(&crescent), kinds(&whole));
}
