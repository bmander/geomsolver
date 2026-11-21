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

use crate::common::rounds::{exported_volume, inset, refused, square_run};
use crate::common::{close, ent, text::solved};

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
        close(exported_volume(&profiled(&leaning, first, rest), "whole"), want);
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
        close(exported_volume(&block(&step(from, to), LAYERED), "whole"), want);
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
    let whole = exported_volume(&block(&(step(0.0, 41.0) + holes), LAYERED), "whole");
    close(whole, stepped - bore - blind);
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
    close(exported_volume(&block(&cuts, rest), "whole"), want);
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
    close(exported_volume(&src, "whole"), want);
    // the uprights alone first, the edge alone in its turn at the foot: the top's rim then goes
    // round each upright's round as the foot does — a ring whose ball is as wide as its circle,
    // the corner's ball — and the body is the same
    let alone = src.replace("outside := round(r: 1.5mm) part.boss\n",
        "repeat f in part.boss { repeat g in part.boss { when f parallel std.z and g parallel std.z and f != g {\n\
        \x20 f round(r: 1.5mm) g } } }\n");
    close(exported_volume(&alone, "whole"), want);
}

/// Each vertex's own part of a body every edge of which is rounded `r` at right angles — what is
/// left once each edge has taken (convex) or given (concave) `s` along its whole length — read off
/// three closed forms: the box (eight corners of three convex edges), the step (two of a concave
/// edge between convex ones, ten of three convex) and the boss (four of a convex edge between
/// concave ones, twelve of three convex).  Each vertex's rounding is its own, whatever the edges'
/// lengths (each more than `2r`), so a body's volume is the sharp one's, less `s` along every
/// convex edge, more along every concave one, plus these at its vertices.
fn vertex_parts(r: f64) -> [f64; 3] {
    let s = square_run(r);
    let ccc = (crate::common::rounds::rounded_box([80.0, 50.0, 20.0], r) - 80000.0 + 600.0 * s) / 8.0;
    let ccv = (rounded_prism(1200.0, 200.0, 50.0, [5.0, 1.0], r) - 60000.0 + 650.0 * s - 50.0 * s - 10.0 * ccc) / 2.0;
    let cvv = (crate::common::rounds::rounded_box([80.0, 50.0, 10.0], r) + rounded_feature(40.0, 20.0, 10.0, r)
        - 48000.0 + 720.0 * s - 120.0 * s - 12.0 * ccc) / 4.0;
    [ccc, ccv, cvv]
}

/// **Two steps of two depths**, one down 10 across the block's end and one down 6 along its side
/// (#962's reviewer's): each step's floor into its wall is alone at each end, and the two meet
/// nowhere, so each step's own edges and then every edge, each step's statement named.  Edges: 690
/// convex, 90 concave; twelve vertices of three convex edges, four of a concave between convex.
/// Written bare, the two steps' fills leave their faces under one name (`filled.round`, and
/// `filled.near` on the block's side and on the first step's wall), read as one face in pieces
/// on their several surfaces: refused, saying so and to name the statements.
#[test]
fn two_steps_of_two_depths_round_in_two_statements() {
    let s = square_run(1.5);
    let [ccc, ccv, _] = vertex_parts(1.5);
    let cuts = format!("{}in std.top {{\n  notch2 := step(u: -41mm, v: 5mm) line -> step(u: 41mm) line -> \
        step(v: 26mm) line ->\n    step(u: -41mm) line -> close\n}}\n\
        step2 := solid(std.solids.Prism(notch2, from: 14mm, to: 21mm))\nstep2 cut part\n", step(0.0, 41.0));
    let rest = "first := round(r: 1.5mm) part.cutter\nsecond := round(r: 1.5mm) part.step2\n\
        whole := solid(part)\nround(r: 1.5mm) whole\n";
    let want = 55200.0 - 690.0 * s + 90.0 * s + 12.0 * ccc + 4.0 * ccv;
    close(exported_volume(&block(&cuts, rest), "whole"), want);
    let bare = rest.replace("first := ", "").replace("second := ", "");
    let e = solved(&block(&cuts, &bare));
    let say = Say { stage: &|_: &str| {}, mark: &|_| {} };
    match export::exact(&e.sketch, ent(&e, "whole").i(), None, None, &say) {
        Ok(x) => close(props::volume(&x.solid), want),
        Err(why) => assert!(why.message.contains("the faces of rounds or chamfers of several statements written \
            bare under one name") && why.message.contains("name each of those statements"), "{}", why.message),
    }
}

/// **A boss beside a step**: a boss 10 square and 6 tall standing on the step's floor, clear of
/// its wall — the step's fill and the boss's own edges first, then every edge.  The rounded step,
/// and the boss's prism less `s` along its convex edges, more along its foot, with four corners of
/// three convex edges and four of a convex edge between concave ones.
#[test]
fn a_boss_beside_a_step_rounds_in_two_statements() {
    let (r, s) = (1.5, square_run(1.5));
    let [ccc, _, cvv] = vertex_parts(r);
    let cuts = step(0.0, 41.0) + "in std.top {\n  pad := step(u: 15mm, v: -5mm) line -> step(u: 25mm) line -> \
        step(v: 5mm) line ->\n    step(u: 15mm) line -> close\n}\n\
        boss := solid(std.solids.Prism(pad, from: 10mm, to: 16mm))\nbody := solid(part)\nboss union body\n";
    // (the boss stands on the stepped part: a cut of a body takes its unions too)
    let rest = "inside := round(r: 1.5mm) body.part.cutter\noutside := round(r: 1.5mm) body.boss\n\
        whole := solid(body)\nround(r: 1.5mm) whole\n";
    let stepped = rounded_prism(1200.0, 200.0, 50.0, [5.0, 1.0], r);
    let want = stepped + 600.0 - (24.0 + 40.0) * s + 40.0 * s + 4.0 * ccc + 4.0 * cvv;
    close(exported_volume(&block(&cuts, rest), "whole"), want);
}

/// A pocket 40 by 20, 10 deep, in the block's top, its plan corners arcs of `big`, every edge of
/// the part rounded `r` by one statement: the pocket's walls go on tangent round its corners, so
/// no vertex of it has edges of both turns.
fn rounded_pocket(big: f64, r: f64) -> String {
    let (x, y) = (20.0, 10.0);
    let pts = [[-x, -y], [x, -y], [x, y], [-x, y]];
    let mut s = String::from("in std.top {\n");
    for (k, [u, v]) in pts.iter().enumerate() {
        s += &format!("  q{k} := point\n  fix(({u}, {v})) q{k}\n");
    }
    let chain: Vec<String> = (0..4).map(|k| format!("(w{k} := line(q{k}, q{})) -> (c{} := round(r: {big}mm))",
        (k + 1) % 4, (k + 1) % 4)).collect();
    let names: Vec<String> = (0..4).map(|k| format!("w{k}, c{}", (k + 1) % 4)).collect();
    s += &format!("  {} close\n  opening := face({})\n}}\n", chain.join(" "), names.join(", "));
    block(&(s + "pocket := solid(std.solids.Prism(opening, from: 10mm, to: 21mm))\npocket cut part\n"),
        &format!("round(r: {r}mm) part\n"))
}

/// **"All edges" over a pocket with rounded corners**, the commonest real part: corners of 3 and
/// of exactly the round's 1.5 (the floor's ring a ball as wide as its corner, its torus closed
/// onto the corner's axis, item 1), each in one statement — the block rounded all over less the
/// pocket's void: its rounded plan the depth deep, its floor's fill turned in round each corner and
/// its rim's round out, `4π s d` more along the four quarter turns.  With corners of 1 the floor's
/// ball is wider than the corner: refused, naming the corner's radius and the faces it lies
/// between.
#[test]
fn a_pocket_with_rounded_corners_is_rounded_all_over_in_one_statement() {
    let (r, s, d) = (1.5, square_run(1.5), inset(1.5));
    for big in [3.0, 1.5] {
        let void = (40.0 * 20.0 - (4.0 - PI) * big * big) * 10.0 + 4.0 * PI * s * d;
        let want = crate::common::rounds::rounded_box([80.0, 50.0, 20.0], r) - void;
        close(exported_volume(&rounded_pocket(big, r), "part"), want);
    }
    let e = solved(&rounded_pocket(1.0, r));
    let why = refused(&e, "part");
    assert!(why.contains("of radius 1mm where the round runs and tighter than the ball") && why.contains("(where `part."),
        "{why}");
}

/// **A bore's rims through a thin plate**: a plate 40 square and `thick`, a bore of 5 through it,
/// its rims on both faces rounded 1.5.  Each rim takes what a ball touching the face and the bore
/// cannot reach; the ball need stand within neither the other face, so where the plate is thinner
/// than `2r` the two rims' takes overlap, and the bore's wall is gone between them: the two tori
/// cross in a circle halfway through.  The volume: the plate less the bore less the takes' union,
/// by symmetry twice the bottom one's from the face to halfway (or to its whole height), turned
/// about the axis — a quadrature in the angle about the take's ball, its integrand smooth.
#[test]
fn a_bores_rims_through_a_plate_thinner_than_both_rounds_cross_halfway() {
    let (r, big) = (1.5_f64, 5.0_f64);
    for thick in [3.2_f64, 2.6] {
        let src = format!("unit mm\nuse std\nuse std.solids (cut)\nuse std.rounds (round)\nin std.top {{\n\
            \x20 plate_at := step(u: -20mm, v: -20mm) line -> step(u: 20mm) line -> step(v: 20mm) line ->\n\
            \x20   step(u: -20mm) line -> close\n  o := point\n  fix((0, 0)) o\n  bore := circle(center: o) hint(r: 5)\n\
            \x20 radius(5mm) bore\n}}\nplate := solid(std.solids.Prism(plate_at, from: 0mm, to: {thick}mm))\n\
            drill := solid(std.solids.Prism(face(bore), from: -1mm, to: {}mm))\npart := solid(plate)\ndrill cut part\n\
            part.drill round(r: 1.5mm) part\n", thick + 1.0);
        // the bottom take at height z: out to `big + b(z)`, b = r − √(r² − (r − z)²); with r − z
        // = r sin φ, b = r(1 − cos φ) and dz = −r cos φ dφ, from φ = π/2 (the face) down to where
        // z reaches halfway or r
        let top = (r - (0.5 * thick).min(r)) / r;
        let from = top.asin();
        let take: f64 = crate::common::rounds::gauss_legendre(40).iter().map(|&(x, w)| {
            let phi = from + (0.5 * PI - from) * 0.5 * (x + 1.0);
            let b = r * (1.0 - phi.cos());
            w * 0.5 * (0.5 * PI - from) * (2.0 * big * b + b * b) * r * phi.cos()
        }).sum();
        let want = (1600.0 - PI * big * big) * thick - 2.0 * PI * take;
        close(exported_volume(&src, "part"), want);
    }
}

/// **`rounds/stock.sv`**, the general note on a stock block with a step and a pocket: the stepped
/// profile rounded all round, each side's rim round it, less the pocket's rounded void.
#[test]
fn the_stock_block_has_every_edge_rounded() {
    let r = 1.5;
    let want = rounded_prism(80.0 * 20.0 - 20.0 * 10.0, 2.0 * (80.0 + 20.0), 50.0, [5.0, 1.0], r)
        - rounded_feature(30.0, 24.0, 8.0, r);
    close(exported_volume(include_str!("../../examples/rounds/stock.sv"), "whole"), want);
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
    let why = refused(&e, "rod");
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
    let why = refused(&e, "part");
    assert!(why.contains("edges of both turns meet where") && !why.contains("a ball larger"), "{why}");
}

/// **Every edge as one statement**: each round reads the sharp block, so at each corner of the
/// step the side's rounds and the step's fill touch along a line — refused by the export, naming
/// the vertex, the faces there with each edge's turn, and the edge to round first.
#[test]
fn every_edge_in_one_statement_names_the_vertex_of_both_turns() {
    let refused = |word: &str| {
        let src = block(&step(0.0, 41.0), &format!("{word} part\n"));
        crate::common::rounds::refused(&solved(&src), "part")
    };
    let at = ["touches itself along a line", "edges of both turns meet where", "in a concave edge",
              "in a convex edge", "`part.cutter.far`", "`part.block."];
    let why = refused("round(r: 1.5mm)");
    for said in at.iter().chain(&["(`inside := part.cutter.", "round(r: …) part.cutter.", "whole := solid(part)"]) {
        assert!(why.contains(said), "{said}: {why}");
    }
    // a chamfer's flats are no tangent chain, and the two statements would chamfer the first
    // one's flat again: said as not said yet
    let why = refused("chamfer(d: 1.5mm)");
    for said in at.iter().chain(&["A chamfer of every edge at such a vertex is not said by the words yet"]) {
        assert!(why.contains(said), "{said}: {why}");
    }
}

/// A flange plate 80 × 68 × 10 on a rib `rib` (its `y` extent) drawn in the front plane, whose
/// edge leaves the flange's end edge at `(−40, 0)` down to `(x1, −deep)` — `x1` past `−40` a slope
/// under the flange, short of it an overhang — the flange's end and the slope met by `word`;
/// `mirror` draws it all reflected across `x = 0` (the flange's other end).
fn ribbed(x1: f64, deep: f64, rib: [f64; 2], word: &str, mirror: bool) -> String {
    let s = if mirror { -1.0 } else { 1.0 };
    let mut pts = [[-40.0, 0.0], [x1, -deep], [40.0, -deep], [40.0, 0.0]].map(|[u, v]| [s * u, v]);
    let mut names = ["slope", "base", "back", "top"];
    if mirror {
        pts.reverse();
        names = ["back", "base", "slope", "top"];
    }
    let points: String = pts.iter().enumerate()
        .map(|(i, [u, v])| format!("  w{i} := point\n  fix(({u}, {v})) w{i}\n")).collect();
    let lines: String = (0..4).map(|i| format!("  {} := line(w{i}, w{})\n", names[i], (i + 1) % 4)).collect();
    format!("unit mm\nuse std\nuse std.solids\nuse std.solids (union)\nuse std.rounds (round, chamfer)\n\
        in std.top {{\n  a := point\n  b := point\n  c := point\n  d := point\n  fix((-40, -34)) a\n\
        \x20 fix((40, -34)) b\n  fix((40, 34)) c\n  fix((-40, 34)) d\n  s0 := line(a, b)\n  s1 := line(b, c)\n\
        \x20 s2 := line(c, d)\n  s3 := line(d, a)\n  outline := face(s0, s1, s2, s3)\n}}\n\
        flange := solid(std.solids.Prism(outline, from: 0mm, to: 10mm))\n\
        in std.front {{\n{points}{lines}  wf := face(slope, base, back, top)\n}}\n\
        rib := solid(std.solids.Prism(wf, from: {}mm, to: {}mm))\npart := solid(flange)\nrib union part\n\
        sqend := part.flange.{} {word} part.rib.slope\n", rib[0], rib[1], if mirror { "s1" } else { "s3" })
}

/// **A round whose edge ends where its faces' neighbours meet in an edge of the other turn**
/// (#996 item 4): a rib's sloped edge starting at a flange's end edge, the corner between the
/// flange's end and the slope rounded or chamfered.  At each end of that edge the flange's
/// underside meets the rib's side in a concave edge, and the underside's plane holds the rounded
/// edge itself, past its end: it stands at no distance along the run and ends none of it, so the
/// run ends square on the rib's side, its section whole — the flange's corner above the
/// underside taken with the rib's below, a cap a few hundredths wide left on the side's plane
/// where the flange goes on.  Read as a cut along the run (the underside's material side), the
/// rib's part of the section was left and the flange's taken, and the two touched along a line
/// at each end (convex), or the fill was halved (an overhang, concave).  The slope at 10° to 80°
/// off the end's plane, overhanging 30° and 60° (a fill); round and chamfer; the round 6 and 12,
/// as wide as the rib is thick; the rib flush with the flange's side, so one end is ordinary;
/// mirrored: each the prism's closed form to 1e-9 and exported with no line or point it touches
/// itself at.
#[test]
fn a_round_ending_where_a_rib_meets_a_flange_ends_square_on_the_ribs_side() {
    // the flange, and the rib's trapezoid `deep` down from x = −40 to `x1` and up to x = 40
    let body = |x1: f64, deep: f64, wide: f64| 80.0 * 68.0 * 10.0 + 0.5 * (80.0 + 40.0 - x1) * deep * wide;
    // the section a round of `r` takes or fills at a corner of `t` on its side, and a chamfer of
    // legs `d` each measured from the other face
    let round = |t: f64, r: f64| r * r * (1.0 / (0.5 * t).tan() - 0.5 * (PI - t));
    let flat = |t: f64, d: f64| 0.5 * d * d / t.sin();
    let mut cases: Vec<(f64, f64, [f64; 2], &str, bool, f64)> = Vec::new();
    for deg in [10.0_f64, 30.0, 45.0, 60.0, 80.0, -30.0, -60.0] {
        let a = deg.to_radians();
        let deep = if deg > 0.0 { (70.0 / a.tan()).min(60.0) } else { 60.0 };
        let x1 = -40.0 + deep * a.tan();
        // the material's angle at a slope, the void's at an overhang: each `π − |a|`
        let (t, sign) = (PI - a.abs(), if deg > 0.0 { -1.0 } else { 1.0 });
        cases.push((x1, deep, [-6.0, 6.0], "round(r: 1mm)", false, body(x1, deep, 12.0) + sign * 12.0 * round(t, 1.0)));
        cases.push((x1, deep, [-6.0, 6.0], "chamfer(d: 1mm)", false, body(x1, deep, 12.0) + sign * 12.0 * flat(t, 1.0)));
    }
    // the issue's: the slope from (−40, 0) to (0, −60)
    let t = PI - (2.0_f64 / 3.0).atan();
    for (rib, wide) in [([-6.0, 6.0], 12.0), ([-6.0, 34.0], 40.0), ([-34.0, 6.0], 40.0)] {
        cases.push((0.0, 60.0, rib, "round(r: 1mm)", false, body(0.0, 60.0, wide) - wide * round(t, 1.0)));
    }
    for r in [6.0, 12.0] {
        cases.push((0.0, 60.0, [-6.0, 6.0], if r == 6.0 { "round(r: 6mm)" } else { "round(r: 12mm)" }, false,
            body(0.0, 60.0, 12.0) - 12.0 * round(t, r)));
    }
    for word in ["round(r: 1mm)", "chamfer(d: 1mm)"] {
        let taken = if word.starts_with("round") { round(t, 1.0) } else { flat(t, 1.0) };
        cases.push((0.0, 60.0, [-6.0, 6.0], word, true, body(0.0, 60.0, 12.0) - 12.0 * taken));
    }
    // built and written as STEP, touching itself nowhere (at 10° the cap above the underside is
    // the set's own, 3.8 µm wide, which the export warns of and writes)
    let say = Say { stage: &|_: &str| {}, mark: &|_| {} };
    for (x1, deep, rib, word, mirror, want) in cases {
        let e = solved(&ribbed(x1, deep, rib, word, mirror));
        assert!(e.diags.is_empty(), "{:?}", e.diags);
        let exact = export::exact(&e.sketch, ent(&e, "part").i(), None, None, &say)
            .unwrap_or_else(|r| panic!("{word} to ({x1}, -{deep}) over {rib:?}, mirrored {mirror}: {r:?}"));
        assert!(exact.solid.self_touching(1e-9).is_empty() && exact.solid.pinches().is_empty());
        export::step(&exact, "part", None, &say).unwrap();
        let got = props::volume(&exact.solid);
        eprintln!("{word} to ({x1}, -{deep}) over {rib:?}{}: {got:.9} against {want:.9}", if mirror { " mirrored" } else { "" });
        close(got, want);
    }
}

/// A recipe of a square plate 2 × 2 × 1 with a quarter cut from it by the body's own statement
/// (`written`, none where empty) and quarters taken by rounds or chamfers (`late`, in order), each
/// a box over `[x0, x1] × [y0, y1]` through the plate's thickness; the late terms said as a round.
fn quartered(written: &[[f64; 4]], late: &[[f64; 4]]) -> gcs_core::brep::recipe::Recipe {
    use gcs_core::brep::recipe::{Body, Edge, Named, Node, Recipe, Section, Stroke};
    let square = |[x0, x1, y0, y1]: [f64; 4]| {
        let p = [[x0, y0, 0.0], [x1, y0, 0.0], [x1, y1, 0.0], [x0, y1, 0.0]];
        let edges = (0..4).map(|k| Edge { name: format!("s{k}"), shape: Stroke::Line { start: p[k], end: p[(k + 1) % 4] } });
        Node::Prism { profile: Section { origin: [0.0; 3], normal: [0.0, 0.0, 1.0], loops: vec![edges.collect()] },
            from: -0.5, to: 1.5, draft: None }
    };
    let mut nodes = vec![Named { id: 0, name: "plate".into(), said: None, node: match square([0.0, 2.0, 0.0, 2.0]) {
        Node::Prism { profile, .. } => Node::Prism { profile, from: 0.0, to: 1.0, draft: None },
        _ => unreachable!(),
    } }];
    let mut body = Body { stock: 0, ..Default::default() };
    for (k, &q) in written.iter().chain(late).enumerate() {
        let id = 1 + k as i64;
        let late = k >= written.len();
        nodes.push(Named { id, name: format!("q{k}"), node: square(q),
            said: late.then(|| format!("`part`'s round or chamfer `r{k}.taken`")) });
        if late { body.cut_late.push(id) } else { body.cut.push(id) }
    }
    let root = 1 + (written.len() + late.len()) as i64;
    nodes.push(Named { id: root, name: "part".into(), said: None, node: Node::Body(body) });
    Recipe { root, nodes }
}

/// **No round or chamfer leaves the body touching itself alone** (#996 item 4): a round whose
/// material, read off the body, meets only the body's own along a line — here a late term taking
/// the quarter of a plate diagonal to one the body's own cut took, so the two quarters left touch
/// along the plate's middle line — is refused where it is applied, by its name and the line, not
/// built and handed on to a `--where`, a later round or an export.  Two rounds whose materials
/// meet so (the two quarters taken by two late terms, as rounds of separate statements reading
/// the body before each other at a vertex of both turns) are built still, the line held twice,
/// and said by the export.
#[test]
fn a_round_alone_touching_the_body_along_a_line_is_refused_by_its_name() {
    let (a, b) = ([1.0, 3.0, -1.0, 1.0], [-1.0, 1.0, 1.0, 3.0]);
    let why = gcs_core::brep::recipe::build(&quartered(&[a], &[b])).expect_err("a round alone touching the body");
    assert!(why.contains("`part`'s round or chamfer `r1.taken`") && why.contains("touching itself along a line")
        && why.contains("[1.0000, 1.0000,"), "{why}");
    let built = gcs_core::brep::recipe::build(&quartered(&[], &[a, b])).expect("two rounds meeting");
    assert!(!built.self_touching(1e-9).is_empty());
    close(props::volume(&built), 2.0);
    // the body's own cuts alone leave what they leave, and a late term touching nothing new builds
    assert!(gcs_core::brep::recipe::build(&quartered(&[a, b], &[])).is_ok());
    let clear = gcs_core::brep::recipe::build(&quartered(&[a], &[[-1.0, 0.5, 1.5, 3.0]])).expect("a round clear of the line");
    assert!(clear.self_touching(1e-9).is_empty());
}
