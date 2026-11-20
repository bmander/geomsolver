//! **Where edges of both turns meet at a vertex** (#962's 8c, "all edges R1.5"): a stock block's
//! step, a pocket's rim, a boss's foot.  At such a vertex one edge is alone in its turn — a step's
//! floor meeting its wall (concave) between two convex edges of the block's side, a boss's upright
//! (convex) between two concave edges of its foot — and the ball rolled along every edge leaves a
//! smooth surface only where that edge is rounded first and the other two are rounded on the body
//! it leaves, carried round its round tangent: the side's round goes on round the step's fill as a
//! ring about the fill's axis (a torus of `2r` about it), the foot's fill round the upright's round
//! the same.  That is what `std.rounds`' words already say, read in that order — the lone edge
//! rounded by one statement and every edge by another on a solid of what it leaves — and nothing
//! new is built: a fill's cap where it ends on a face is a face of the body, listed, so the second
//! statement rounds the fill's rim on that face as it rounds the face's own edges.  Each volume is
//! its closed form's to 1e-9, each solid exported with no sliver.
use gcs_core::brep::{export, props, sweep::Say};
use std::f64::consts::PI;

use crate::common::rounds::square_run;
use crate::common::{close, ent, text::solved};

/// The solid `name` built exactly, and exported: its STEP written, and no face of it thinner than
/// the export's tolerance.
fn exported(src: &str, name: &str) -> f64 {
    let e = solved(src);
    assert!(e.diags.is_empty(), "{:?}", e.diags);
    let say = Say { stage: &|_: &str| {}, mark: &|_| {} };
    let body = ent(&e, name).i();
    let exact = export::exact(&e.sketch, body, None, None, &say).unwrap_or_else(|r| panic!("{name}: {r:?}"));
    if let Some(s) = export::slivers(&e.sketch, body, &exact, None) {
        panic!("{}", s.message);
    }
    export::step(&exact, name, None, &say).unwrap();
    props::volume(&exact.solid)
}

/// How far in from each face the centroid of a round's section stands.
fn inset(r: f64) -> f64 { (10.0 - 3.0 * PI) / (3.0 * (4.0 - PI)) * r }

/// A prism `long` long of a profile all of whose corners are right angles — `convex` of them
/// convex, `concave` reflex — of area `area` and perimeter `perimeter`, every edge rounded `r`:
/// the profile's corners rounded (a convex one less the round's section, a reflex one filled by
/// it), and each end's rim rounded all round the rounded profile — a run along each straight
/// stretch, a ring round each arc (Pappus: the section's centroid turned about the arc's axis, in
/// at a convex corner, where the ring is the corner's ball, and out at a reflex one, where it is a
/// torus of `2r` about the fill's axis).
fn rounded_prism(area: f64, perimeter: f64, long: f64, [convex, concave]: [f64; 2], r: f64) -> f64 {
    let (s, d) = (square_run(r), inset(r));
    let section = area - (convex - concave) * s;
    let straight = perimeter - 2.0 * r * (convex + concave);
    let arcs = 0.5 * PI * (convex * (r - d) + concave * (r + d));
    section * long - 2.0 * s * (straight + arcs)
}

/// `rounded_prism` at any angles and radii: each of the profile's corners its angle on the side
/// the round stands (the material's at a convex one, the void's at a reflex one), whether it is
/// convex, and the radius it is rounded by; each end's rim rounded `rim` all round the rounded
/// profile.  A corner of angle `θ` rounded `ρ` takes or fills `ρ²(cot(θ/2) − (π − θ)/2)` of the
/// section and `2ρ cot(θ/2)` of the perimeter's straight stretches, and the rim's section turns
/// through `π − θ` about its axis, `ρ ∓ d` from it.
fn rounded_profile(area: f64, perimeter: f64, long: f64, corners: &[(f64, bool, f64)], rim: f64) -> f64 {
    let (s, d) = (square_run(rim), inset(rim));
    let cot = |t: f64| 1.0 / (0.5 * t).tan();
    let (mut section, mut straight, mut arcs) = (area, perimeter, 0.0);
    for &(t, convex, rho) in corners {
        let corner = rho * rho * (cot(t) - 0.5 * (PI - t));
        section += if convex { -corner } else { corner };
        straight -= 2.0 * rho * cot(t);
        arcs += (PI - t) * if convex { rho - d } else { rho + d };
    }
    section * long - 2.0 * s * (straight + arcs)
}

/// A part drawn in the front plane through `outline` (its corners, counter-clockwise, the faces
/// it leaves named `e0`, `e1`, …), 50 deep, its concave edge between faces `e2` and `e3` filled
/// `first` and then every edge rounded `rest` on the body that leaves.
fn profiled(outline: &[[f64; 2]], first: f64, rest: f64) -> String {
    let n = outline.len();
    let steps: Vec<String> = (1..=n).map(|k| {
        let [u, v] = outline[k % n];
        format!("step(u: {u}mm, v: {v}mm) (e{} := line)", k - 1)
    }).collect();
    format!("unit mm\nuse std\nuse std.solids\nuse std.rounds (round)\nin std.front {{\n  outline := {} -> close\n}}\n\
        part := solid(std.solids.Prism(outline, from: -25mm, to: 25mm))\n\
        inside := part.e2 round(r: {first}mm) part.e3\nwhole := solid(part)\nround(r: {rest}mm) whole\n",
        steps.join(" ->\n    "))
}

/// **A step whose wall leans**, at 45° to its floor, and **a fill of its own radius**: the vertex
/// where the side meets the floor and the wall is still one of a concave edge between two convex
/// ones, the side's round turned through the fill's 45° round its axis.  The fill 2.5 and every
/// other edge 1, and the fill 1 and the rest 2.5 (the side's ring about the fill's axis `3.5` out
/// either way).
#[test]
fn a_leaning_step_and_a_fill_of_its_own_radius_ring_the_side_round_the_fill() {
    let q = 0.5 * PI;
    // the stepped profile: (40, 10) to (10, 10) the floor, (10, 10) to (0, 20) the wall leaning
    let leaning = [[-40.0, 0.0], [40.0, 0.0], [40.0, 10.0], [10.0, 10.0], [0.0, 20.0], [-40.0, 20.0]];
    let (area, perimeter) = (80.0 * 20.0 - 30.0 * 10.0 - 50.0, 80.0 + 10.0 + 30.0 + 200f64.sqrt() + 40.0 + 20.0);
    for (first, rest) in [(2.5, 1.0), (1.0, 2.5)] {
        let corners = [(q, true, rest), (q, true, rest), (0.75 * PI, false, first), (0.75 * PI, true, rest),
                       (q, true, rest), (q, true, rest)];
        let want = rounded_profile(area, perimeter, 50.0, &corners, rest);
        close(exported(&profiled(&leaning, first, rest), "whole"), want);
    }
}

/// A block 80 by 50 by 20 drawn on the top plane about the origin, `cuts` (a statement each, over
/// `part`) made in it, then `rest`.
fn block(cuts: &str, rest: &str) -> String {
    format!("unit mm\nuse std\nuse std.solids (cut, union)\nuse std.rounds (round, chamfer)\n\
        in std.top {{\n  outline := step(u: -40mm, v: -25mm) line -> step(u: 40mm) line -> step(v: 25mm) line ->\n\
        \x20   step(u: -40mm) line -> close\n}}\n\
        block := solid(std.solids.Prism(outline, from: 0mm, to: 20mm))\npart := solid(block)\n{cuts}{rest}")
}

/// A step cut across one end of the block, `from` to `to` along it, 10 deep.
fn step(from: f64, to: f64) -> String {
    format!("in std.top {{\n  notch := step(u: {from}mm, v: -26mm) line -> step(u: {to}mm) line -> step(v: 26mm) line ->\n\
        \x20   step(u: {from}mm) line -> close\n}}\n\
        cutter := solid(std.solids.Prism(notch, from: 10mm, to: 21mm))\ncutter cut part\n")
}

/// The step's fill first, then every edge on the body it leaves.
const LAYERED: &str = "inside := round(r: 1.5mm) part.cutter\nwhole := solid(part)\nround(r: 1.5mm) whole\n";

/// **A step across the block, every edge rounded**: its floor meets its wall in a concave edge
/// between two convex edges of each side, and the side's round goes on round the fill as a ring:
/// the L's profile rounded at its five convex corners and filled at its reflex one, and each side's
/// rim rounded all round it.  The step at either end, and moved along the block.
#[test]
fn a_step_rounded_all_over_rings_its_sides_round_the_fill() {
    let r = 1.5;
    for (from, to) in [(0.0_f64, 41.0_f64), (20.0, 41.0), (-41.0, -20.0)] {
        let over = to.min(40.0) - from.max(-40.0);
        let area = 80.0 * 20.0 - over * 10.0;
        let want = rounded_prism(area, 2.0 * (80.0 + 20.0), 50.0, [5.0, 1.0], r);
        close(exported(&block(&step(from, to), LAYERED), "whole"), want);
    }
}

/// **A corner's edge ends where its face goes on tangent** (#962): the block's corner at the
/// step's far end rounds the side's edge with the floor, which ends where the floor goes on into
/// the fill — no face there, so square.  Read as no end at all, the corner's run went on along the
/// side past the fill and under the step's high end, a groove 1.5 deep through the field (and not
/// through the exact build, which a probe at the cap where the run ends found at one placing of
/// the step in five).  The field holds the high end's material beside the side, below the floor.
#[test]
fn a_corner_rounds_its_edge_no_further_than_where_its_face_goes_on_tangent() {
    let e = solved(&block(&step(20.0, 41.0), LAYERED));
    let field = gcs_core::solid::MaterialField::read(&e.sketch, ent(&e, "whole").i(), 1e-10).unwrap();
    for x in [-30.0, 0.0, 10.0, 19.0] {
        for y in [-24.97, 24.97] {
            assert!(field.side([x, y, 9.24]) < 0.0, "({x}, {y}, 9.24) is material");
        }
    }
    // and the side's rim round the fill, a ring, takes what lies past the floor's end beside it
    assert!(field.side([21.0, 24.97, 9.24]) > 0.0);
}

/// **A step and two holes beside it**: a bore through the block's high end and a blind hole 6 deep
/// in it, every edge rounded on the body the step's fill leaves.  A hole's rims meet no other edge:
/// the top's rim of each a ring taken about its axis (the section's centroid out from it), the
/// blind hole's floor a ring filled (in toward it).
#[test]
fn holes_beside_a_step_are_rounded_with_it() {
    let (r, s, d) = (1.5, square_run(1.5), inset(1.5));
    let holes = "in std.top {\n  c1 := point\n  fix((-20, -10)) c1\n  bore := circle(center: c1) hint(r: 5)\n\
        \x20 radius(5mm) bore\n  c2 := point\n  fix((-20, 12)) c2\n  blind := circle(center: c2) hint(r: 4)\n\
        \x20 radius(4mm) blind\n}\n\
        drill := solid(std.solids.Prism(face(bore), from: -1mm, to: 21mm))\n\
        spot := solid(std.solids.Prism(face(blind), from: 14mm, to: 21mm))\ndrill cut part\nspot cut part\n";
    let stepped = rounded_prism(80.0 * 20.0 - 40.0 * 10.0, 2.0 * (80.0 + 20.0), 50.0, [5.0, 1.0], r);
    let bore = PI * 25.0 * 20.0 + 2.0 * 2.0 * PI * s * (5.0 + d);
    let blind = PI * 16.0 * 6.0 + 4.0 * PI * s * d;
    close(exported(&block(&(step(0.0, 41.0) + holes), LAYERED), "whole"), stepped - bore - blind);
}

/// A rectangle 40 by 20 about the block's middle, `name`d, a prism of it `from` to `to`.
fn middle(name: &str, from: f64, to: f64) -> String {
    format!("in std.top {{\n  {name}_at := step(u: -20mm, v: -10mm) line -> step(u: 20mm) line -> step(v: 10mm) line ->\n\
        \x20   step(u: -20mm) line -> close\n}}\n\
        {name} := solid(std.solids.Prism({name}_at, from: {from}mm, to: {to}mm))\n")
}

/// What a rectangle `l` by `w` rounded `r` at its corners, `deep` deep, leaves when its own edges
/// are rounded first (its uprights, and its far rim where it stands on or sinks into a face) and
/// its near rim on the body that leaves: the prism of the rounded rectangle, less the far rim's
/// section run round it (its centroid in at each corner, where the corner is the ball's), more
/// the near rim's (out at each corner, a torus of `2r` about the upright's axis) — the same along
/// the straight stretches, so `4π s d` more.
fn rounded_feature(l: f64, w: f64, deep: f64, r: f64) -> f64 {
    (l * w - 4.0 * square_run(r)) * deep + 4.0 * PI * square_run(r) * inset(r)
}

/// **A pocket rounded all over**: each upright where two of its walls meet is concave, between two
/// convex edges of the top at the rim — the pocket's own edges are filled first (uprights, floor
/// edges, the floor's corners), and the top's rim on the body that leaves goes on round each
/// upright's fill, a ring about it.  The block rounded all over less the pocket's rounded void.
#[test]
fn a_pocket_rounded_all_over_rings_its_rim_round_the_uprights() {
    let r = 1.5;
    let cuts = format!("{}pocket cut part\n", middle("pocket", 10.0, 21.0));
    let rest = "inside := round(r: 1.5mm) part.pocket\nwhole := solid(part)\nround(r: 1.5mm) whole\n";
    let want = crate::common::rounds::rounded_box([80.0, 50.0, 20.0], r) - rounded_feature(40.0, 20.0, 10.0, r);
    close(exported(&block(&cuts, rest), "whole"), want);
}

/// **A boss rounded all over**: each upright where two of its sides meet is convex, between two
/// concave edges of its foot — the boss's own edges are rounded first (uprights, top rim, the
/// top's corners), and the foot on the body that leaves is filled round each upright's round, a
/// ring about it.  The plate rounded all over with the boss's rounded solid on it.
#[test]
fn a_boss_rounded_all_over_fills_its_foot_round_the_uprights() {
    let r = 1.5;
    let src = format!("unit mm\nuse std\nuse std.solids (union)\nuse std.rounds (round)\n\
        in std.top {{\n  outline := step(u: -40mm, v: -25mm) line -> step(u: 40mm) line -> step(v: 25mm) line ->\n\
        \x20   step(u: -40mm) line -> close\n}}\n\
        plate := solid(std.solids.Prism(outline, from: 0mm, to: 10mm))\npart := solid(plate)\n{}boss union part\n\
        outside := round(r: 1.5mm) part.boss\nwhole := solid(part)\nround(r: 1.5mm) whole\n",
        middle("boss", 10.0, 20.0));
    let want = crate::common::rounds::rounded_box([80.0, 50.0, 10.0], r) + rounded_feature(40.0, 20.0, 10.0, r);
    close(exported(&src, "whole"), want);
}

/// **`rounds/stock.sv`**, the general note on a stock block with a step and a pocket: the stepped
/// profile rounded all round, each side's rim round it, less the pocket's rounded void.
#[test]
fn the_stock_block_has_every_edge_rounded() {
    let r = 1.5;
    let want = rounded_prism(80.0 * 20.0 - 20.0 * 10.0, 2.0 * (80.0 + 20.0), 50.0, [5.0, 1.0], r)
        - rounded_feature(30.0, 24.0, 8.0, r);
    close(exported(include_str!("../../examples/rounds/stock.sv"), "whole"), want);
}

/// **An edge the general note cannot build is said by the faces it lies between**: a rod of
/// radius 3 rounded all over by 4, its rims a ball wider than the rod holds.  The round no
/// document named (a word's use, `#….taken`) is said with the two faces of the part it rounds,
/// in the author's names, so the statement can leave that edge out.
#[test]
fn an_edge_the_general_note_cannot_build_is_said_by_its_faces() {
    let e = solved("unit mm\nuse std\nuse std.solids\nuse std.rounds (round)\nin std.top {\n  o := point\n  fix((0, 0)) o\n\
        \x20 rim := circle(center: o) hint(r: 3)\n  radius(3mm) rim\n}\n\
        rod := solid(std.solids.Prism(face(rim), from: 0mm, to: 10mm))\nround(r: 4mm) rod\n");
    let say = Say { stage: &|_: &str| {}, mark: &|_| {} };
    let why = format!("{:?}", export::exact(&e.sketch, ent(&e, "rod").i(), None, None, &say).err().expect("refused"));
    assert!(why.contains("(where `rod.") && why.contains("` and `rod.") && why.contains("` meet)"), "{why}");
}

/// **A corner whose face is in pieces of one name**: two steps cut down 10 along one end and one
/// side of the block, the second cut splitting the first one's floor in two pieces, both
/// `part.cut1.far`.  The block's corner at the first step's far end rounds the floor's edge with
/// the end along both pieces — the edge goes on across their seam, so the field reads its ends
/// where the reference's run ends, and the corner is held within those, not within the one
/// piece's (whose seam, where a straight run ends on no face, ends it square): built, not "a ball
/// larger than the edges it rounds".  Rounded all over in one statement the part is refused, as
/// ever, at a vertex of both turns.
#[test]
fn a_corner_whose_face_is_in_pieces_of_one_name_is_held_within_the_references_run() {
    let cuts = "in std.top {\n  first := step(u: 0mm, v: -26mm) (w1 := line) -> step(u: 41mm) line -> step(v: 26mm) line ->\n\
        \x20   step(u: 0mm) line -> close\n\
        \x20 second := step(u: 41mm, v: 0mm) (w2 := line) -> step(v: 26mm) line -> step(u: -41mm) line ->\n\
        \x20   step(v: 0mm) line -> close\n}\n\
        cut1 := solid(std.solids.Prism(first, from: 10mm, to: 21mm))\n\
        cut2 := solid(std.solids.Prism(second, from: 10mm, to: 21mm))\ncut1 cut part\ncut2 cut part\n";
    let e = solved(&block(cuts, "round(r: 1.5mm) part\n"));
    let say = Say { stage: &|_: &str| {}, mark: &|_| {} };
    let why = format!("{:?}", export::exact(&e.sketch, ent(&e, "part").i(), None, None, &say).err().expect("refused"));
    assert!(why.contains("edges of both turns meet where") && !why.contains("a ball larger"), "{why}");
}

/// **Every edge as one statement**: each round reads the sharp block, so at each corner of the
/// step the side's rounds and the step's fill touch along a line — refused by the export, naming
/// the vertex, the faces there with each edge's turn, and the edge to round first.
#[test]
fn every_edge_in_one_statement_names_the_vertex_of_both_turns() {
    let refused = |word: &str| {
        let e = solved(&block(&step(0.0, 41.0), &format!("{word} part\n")));
        let say = Say { stage: &|_: &str| {}, mark: &|_| {} };
        format!("{:?}", export::exact(&e.sketch, ent(&e, "part").i(), None, None, &say).err().expect("refused"))
    };
    let at = ["touches itself along a line", "edges of both turns meet where", "in a concave edge",
              "in a convex edge", "`part.cutter.far`", "`part.block."];
    let why = refused("round(r: 1.5mm)");
    for said in at.iter().chain(&["(`part.cutter.", "round(r: …) part.cutter.", "whole := solid(part)"]) {
        assert!(why.contains(said), "{said}: {why}");
    }
    // a chamfer's flats are no tangent chain, and the two statements would chamfer the first
    // one's flat again: said as not said yet
    let why = refused("chamfer(d: 1.5mm)");
    for said in at.iter().chain(&["A chamfer of every edge at such a vertex is not said by the words yet"]) {
        assert!(why.contains(said), "{said}: {why}");
    }
}
