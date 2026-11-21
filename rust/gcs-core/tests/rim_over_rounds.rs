//! **A rim over rounded uprights whose outline turns both ways** (#994 item 2c, fixture 150): a
//! block's upright edges rounded, then its rims rounded on a solid of the body that leaves
//! (`whole := solid(part)`), the documented order for a vertex where edges of both turns meet.
//! Where the outline has a reflex corner — an L, a block with a tab — its sides, the convex
//! uprights' rounds and the reflex uprights' fills are one smooth chain turning both ways, so the
//! rim's ball centres are read about the chain's nearest face (`conditions::about_nearest`); read
//! through that face alone, a rim rounded to the uprights' own radius, whose centres close onto
//! each convex upright's axis, never settled there and the gate refused the corner's quarter ball
//! as leaving out what the field reads.  Each volume is the closed form's to 1e-9: the outline's
//! prism, less each convex upright's run and plus each reflex one's, less the rim's section along
//! the straight sides and turned about each upright's axis (Pappus: `R ∓ inset` from it).
use gcs_core::program::Code;
use std::f64::consts::PI;

use crate::common::close;
use crate::common::rounds::{exported_volume, inset, refused, square_run};
use crate::common::text::solved;

/// A rectangle, an L (one reflex corner) and the fixture's block with a tab (two), each outline
/// counter-clockwise on the top plane.
const RECT: &[[f64; 2]] = &[[0.0, 0.0], [50.0, 0.0], [50.0, 30.0], [0.0, 30.0]];
const ELL: &[[f64; 2]] = &[[0.0, 0.0], [50.0, 0.0], [50.0, 20.0], [20.0, 20.0], [20.0, 40.0], [0.0, 40.0]];
const TAB: &[[f64; 2]] = &[[317.5, -100.75], [317.5, 54.15], [332.5, 54.15], [332.5, 64.15], [317.5, 64.15],
    [317.5, 100.75], [200.0, 100.75], [200.0, -100.75]];

/// The outline as a prism 40 tall (z from −20 to 20), its upright edges rounded `ru` (`vs`), and
/// `rim` stated over `whole := solid(part)`.
fn block(outline: &[[f64; 2]], ru: f64, rim: &str) -> String {
    let mut s = String::from("unit mm\nuse std\nuse std.solids\nuse std.rounds (round)\nin std.top {\n");
    for (k, [x, y]) in outline.iter().enumerate() {
        s += &format!("  q{k} := point\n  fix(({x}, {y})) q{k}\n");
    }
    let names: Vec<String> = (0..outline.len()).map(|k| format!("q{k}")).collect();
    s + &format!("  outline := face({}, -> close)\n}}\n\
        stock := solid(std.solids.Prism(outline, from: -20mm, to: 20mm))\npart := solid(stock)\n\
        repeat f in part.stock {{ repeat g in part.stock {{ when f parallel std.z and g parallel std.z \
        and f != g {{\n  vs := f round(r: {ru}mm) g }} }} }}\nwhole := solid(part)\n{rim}\n", names.join(", "))
}

/// Both rims rounded `r`: each face square to the uprights (the caps and the fills' caps on them).
fn both_rims(r: f64) -> String {
    format!("repeat a in whole {{ when a parallel std.x and a parallel std.y {{ a round(r: {r}mm) whole }} }}")
}

/// The outline's area, its perimeter, and how many of its corners are convex and reflex.
fn measures(outline: &[[f64; 2]]) -> (f64, f64, usize, usize) {
    let n = outline.len();
    let (mut area, mut perimeter, mut convex) = (0.0, 0.0, 0);
    for k in 0..n {
        let ([x0, y0], [x1, y1], [x2, y2]) = (outline[k], outline[(k + 1) % n], outline[(k + 2) % n]);
        area += 0.5 * (x0 * y1 - x1 * y0);
        perimeter += (x1 - x0).hypot(y1 - y0);
        convex += usize::from((x1 - x0) * (y2 - y1) - (y1 - y0) * (x2 - x1) > 0.0);
    }
    (area, perimeter, convex, n - convex)
}

/// The closed form: the prism 40 tall, its right-angled uprights rounded `ru` — a convex one's run
/// taken, a reflex one's filled — and `rims` of its rims rounded `r` all the way round: the run's
/// section along the straight sides, and turned a quarter about each upright's axis, its centroid
/// `ru − inset` from a convex one's and `ru + inset` from a reflex one's.
fn volume(outline: &[[f64; 2]], ru: f64, r: f64, rims: f64) -> f64 {
    let (area, perimeter, convex, reflex) = measures(outline);
    let (convex, reflex) = (convex as f64, reflex as f64);
    let s = square_run(r);
    let rim = s * (perimeter - 2.0 * ru * (convex + reflex))
        + 0.5 * PI * s * (convex * (ru - inset(r)) + reflex * (ru + inset(r)));
    40.0 * (area - (convex - reflex) * square_run(ru)) - rims * rim
}

/// Both rims of `outline` over its uprights rounded 1.5, rounded `r`: built, and the closed form.
fn both_rims_built(outline: &[[f64; 2]], r: f64) {
    close(exported_volume(&block(outline, 1.5, &both_rims(r)), "whole"), volume(outline, 1.5, r, 2.0));
}

/// **Both rims over the rounded uprights, built**: the rectangle (one chain bulging toward the
/// ball), the L and the tab (chains turning both ways), the rim just under and at the uprights'
/// radius — a spindle torus round each convex upright, at it a quarter ball, its centres on the
/// upright's axis — and smaller; round each reflex upright a torus of `ru + r`.
#[test]
fn both_rims_over_a_rectangles_rounded_uprights() {
    for r in [1.0, 1.49, 1.5] {
        both_rims_built(RECT, r);
    }
}

/// The L, one reflex corner: read about the nearest face, its chain turning both ways.
#[test]
fn both_rims_over_an_ls_rounded_uprights() {
    for r in [1.0, 1.5] {
        both_rims_built(ELL, r);
    }
}

/// The fixture's tab, two reflex corners, at the uprights' radius and under it.
#[test]
fn both_rims_over_a_tabs_rounded_uprights() {
    for r in [1.49, 1.5] {
        both_rims_built(TAB, r);
    }
}

/// **The general note, its second statement** (`round(r) whole`, every edge of the body the
/// uprights' round leaves): the rims and every edge a fill's cap makes, the same body.
#[test]
fn every_edge_over_rounded_uprights_turning_both_ways() {
    let got = exported_volume(&block(TAB, 1.5, "round(r: 1.5mm) whole"), "whole");
    close(got, volume(TAB, 1.5, 1.5, 2.0));
}

/// **The fixture's statement** (`whole.part.stock.far round(r: 1.5mm) whole`), on either cap:
/// the rim all the way round.  A reflex upright's fill ends flush on the cap's plane in a cap of
/// its own name (`vs.filled.far`, `lowering::CAPS`), which has grown the top: `stock.far` of
/// `whole` is the top with the caps lying on it (`BodyFaces::with_caps`), so the rim goes on round
/// each fill's arc as round each convex upright's.  Named with the caps too, the same rim: the cap
/// and the top are one face, never paired as two.
fn one_rim(outline: &[[f64; 2]], cap: &str) {
    let rim = format!("whole.part.stock.{cap} round(r: 1.5mm) whole");
    close(exported_volume(&block(outline, 1.5, &rim), "whole"), volume(outline, 1.5, 1.5, 1.0));
    let rim = format!("{rim}\nwhole.part.vs.filled.{cap} round(r: 1.5mm) whole");
    close(exported_volume(&block(outline, 1.5, &rim), "whole"), volume(outline, 1.5, 1.5, 1.0));
}

/// The fixture itself, its far rim (#994's repro).
#[test]
fn the_fixtures_far_rim_goes_round_the_fills_caps() {
    one_rim(TAB, "far");
}

/// The L's near rim.
#[test]
fn an_ls_near_rim_goes_round_the_fills_cap() {
    one_rim(ELL, "near");
}

/// **A chamfer the same**: the L's top chamfered 1 all the way round over its rounded uprights —
/// the flat's triangle along the sides and turned about each upright's axis, its centroid
/// `ru ∓ d/3` from it.
#[test]
fn an_ls_rim_chamfered_goes_round_the_fills_cap() {
    let src = block(ELL, 1.5, "whole.part.stock.far chamfer(d: 1mm) whole").replace("(round)", "(round, chamfer)");
    let (_, perimeter, convex, reflex) = measures(ELL);
    let flat = 0.5 * (perimeter - 3.0 * (convex + reflex) as f64)
        + 0.25 * PI * (convex as f64 * (1.5 - 1.0 / 3.0) + reflex as f64 * (1.5 + 1.0 / 3.0));
    close(exported_volume(&src, "whole"), volume(ELL, 1.5, 0.0, 0.0) - flat);
}

/// **A cap is still a face of its own name**: rounded alone against the fill it ends, the rim is
/// the quarter torus of `ru + inset` round each reflex upright, the sides' edges with the top
/// sharp.
#[test]
fn a_fills_cap_named_alone_is_itself() {
    let (_, _, _, reflex) = measures(ELL);
    let ring = 0.5 * PI * square_run(1.5) * (1.5 + inset(1.5));
    let src = block(ELL, 1.5, "whole.part.vs.filled.far round(r: 1.5mm) whole.part.vs.filled.round");
    close(exported_volume(&src, "whole"), volume(ELL, 1.5, 0.0, 0.0) - reflex as f64 * ring);
}

/// **A rim wider than the uprights' round is refused, saying so**: no ball of it stands on the
/// round's side of a convex upright's round, whichever way the rest of the outline turns.
#[test]
fn a_rim_wider_than_the_uprights_is_refused_naming_their_radius() {
    for outline in [RECT, ELL, TAB] {
        let e = solved(&block(outline, 1.5, &both_rims(2.0)));
        let why = refused(&e, "whole");
        assert!(why.contains("its ball, of radius 2mm, is wider than the arc it rolls round, of radius 1.5mm \
            where the round runs"), "{why}");
    }
}

/// **The uprights and a rim in one statement** are refused where the outline turns back: at a
/// reflex upright's foot the fill and the rim's two runs each read the sharp body, and the fill
/// and the block's uncut material touch along the upright's line for the run's depth — the
/// three-face words (`Corner`, `CornerFill`) say only a corner of one turn — said at the vertex
/// with the two statements to write.
#[test]
fn uprights_and_a_rim_in_one_statement_are_refused_at_the_reflex_corner() {
    for outline in [ELL, TAB] {
        let src = block(outline, 1.5, "").replace("  vs := f round(r: 1.5mm) g } } }\nwhole := solid(part)\n",
            "  f round(r: 1.5mm) g } } }\npart.stock.far round(r: 1.5mm) part.stock\n");
        let e = solved(&src);
        assert!(e.diags.iter().all(|d| d.code != Code::E080), "{:?}", e.diags);
        let why = refused(&e, "part");
        for want in ["touches itself along a line", "edges of both turns meet where", "part.stock.far",
                     "in a concave edge", "whole := solid(part)"] {
            assert!(why.contains(want), "{want}: {why}");
        }
    }
}
