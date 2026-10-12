//! **Rounds and chamfers built exactly** (#194, #183's L7): a region over a body's faces that an
//! entry of the lowering table recognises — by its terms, never by the word that wrote them — is
//! built as its pieces (`solid::lowering`): a prism of its section along a straight edge between
//! planes, a ball's cell where three square faces meet. Each is held to its own field before it
//! stands; a body taking them is built and exported by this kernel, and its volume is the closed
//! form's to 1e-9. A region no entry builds is refused by name and stays the field's.
use gcs_core::brep::{export, props, sweep::Say};
use gcs_core::brep::topo::Brep;
use gcs_core::program::Elaborated;
use gcs_core::solid::lowering::{self, Piece};
use gcs_core::solid::MaterialField;
use gcs_core::solve::{solve, SolveOpts};
use std::f64::consts::PI;

use crate::common::{ent, read};

fn solved(src: &str) -> Elaborated {
    let mut e = read(src);
    let r = solve(&mut e.sketch, SolveOpts::default());
    assert!(r.success, "{}\n{src}", r.message);
    e
}

/// A block 40 × 24 × 12 drawn on the front plane (x, z), its near face at y = 0, standing 12 deep.
const BLOCK: &str = "unit mm\nuse std (round, chamfer)\nin std.front {\n\
    b0 := point\nb1 := point\nb2 := point\nb3 := point\n\
    fix((0, 0)) b0\nfix((40, 0)) b1\nfix((40, 24)) b2\nfix((0, 24)) b3\n\
    (south := line(b0, b1)) -> (east := line(b1, b2)) -> (north := line(b2, b3)) -> \
    (west := line(b3, b0)) -> close\n\
    outline := face(south, east, north, west)\n}\n\
    block := solid(outline, depth: 12mm)\n";

/// Solid `name` of `e` built, written as STEP and as STL by this kernel (each read back and
/// checked as `solventc` checks it), and its B-rep.
fn exported(e: &Elaborated, name: &str) -> Brep {
    let body = ent(e, name).i();
    let say = Say { stage: &|_: &str| {}, mark: &|_| {} };
    let exact = export::exact(&e.sketch, body, None, None, &say).unwrap_or_else(|r| panic!("{name}: {r:?}"));
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

fn close(got: f64, want: f64) {
    assert!((got - want).abs() <= 1e-9 * want, "{got} against {want}");
}

/// The regions of a drawing an entry lowers, and those it lowers to nothing.
fn lowered(e: &Elaborated) -> (usize, usize) {
    let (mut built, mut empty) = (0, 0);
    for i in 0..e.sketch.solids.len() {
        match lowering::lowered(&e.sketch, i) {
            Some(Ok(p)) if p.is_empty() => empty += 1,
            Some(Ok(_)) => built += 1,
            Some(Err(why)) => panic!("`{}`: {why}", e.sketch.solids[i].name),
            None => {}
        }
    }
    (built, empty)
}

/// `block.sv`: its near rim's four runs mitred at the corners (each a prism whose face is a
/// cylinder), its far rim's four flats; the near face's run against the far face, and every fill
/// at a convex edge, hold nowhere. The volume is the block less the four runs' sections along the
/// rim, less what two runs crossing at each corner take twice (`∫ s(z)² dz`, `(5/3 − π/2) r³`), and
/// the same for the flats (`d³/3` each corner).
#[test]
fn the_blocks_rims_are_built_exactly() {
    let e = solved(include_str!("../../examples/rounds/block.sv"));
    assert_eq!(lowered(&e), (8, 12));
    let b = exported(&e, "block");
    let (r, d, rim) = (2.0, 1.0, 2.0 * (40.0 + 24.0));
    let near = (1.0 - PI / 4.0) * r * r * rim - 4.0 * (5.0 / 3.0 - PI / 2.0) * r * r * r;
    let far = d * d / 2.0 * rim - 4.0 * d * d * d / 3.0;
    close(props::volume(&b), 40.0 * 24.0 * 12.0 - near - far);
    assert_eq!(kinds(&b), [("cylinder", 4), ("plane", 10)].into());
}

/// **Recognised by the set, not the word**: `by_hand.sv`'s own `Rolled`, written in the file, is
/// the same set as `std.Run`, so the same entry builds it.
#[test]
fn a_round_written_by_hand_is_built_by_the_same_entry() {
    let e = solved(include_str!("../../examples/rounds/by_hand.sv"));
    assert_eq!(lowered(&e), (1, 0));
    let b = exported(&e, "block");
    close(props::volume(&b), 40.0 * 24.0 * 12.0 - (1.0 - PI / 4.0) * 9.0 * 40.0);
}

/// X2's block rounded all over: twelve runs, each cut short at the cell of the ball standing at
/// either end and joined to it there — Steiner's formula, a cylinder face per edge and a sphere's
/// patch per corner.
#[test]
fn a_block_rounded_all_over_is_steiners_exactly() {
    let e = solved(&format!("{BLOCK}round(r: 2mm) block\n"));
    let b = exported(&e, "block");
    let r = 2.0;
    let (a, w, h) = (40.0 - 2.0 * r, 24.0 - 2.0 * r, 12.0 - 2.0 * r);
    let want = a * w * h + 2.0 * r * (a * w + w * h + a * h) + PI * r * r * (a + w + h) + 4.0 * PI * r * r * r / 3.0;
    close(props::volume(&b), want);
    assert_eq!(kinds(&b), [("cylinder", 12), ("plane", 6), ("sphere", 8)].into());
}

/// X3's block chamfered all over: three flats crossing at each corner, `2d²(l + w + h) − 6d³`.
#[test]
fn a_block_chamfered_all_over_exactly() {
    let e = solved(&format!("{BLOCK}chamfer(d: 1mm) block\n"));
    let b = exported(&e, "block");
    close(props::volume(&b), 40.0 * 24.0 * 12.0 - (2.0 * 76.0 - 6.0));
    assert_eq!(kinds(&b), [("plane", 18)].into());
}

/// `pocket.sv`: a pocket rounded inside — every edge among its faces concave, filled after the
/// cut that makes it (the stratum), its corners each a ball's cell filled too.
#[test]
fn a_pocket_rounded_inside_is_filled_exactly() {
    let e = solved(include_str!("../../examples/rounds/pocket.sv"));
    let b = exported(&e, "dish");
    let pr = 1.5;
    let run = (1.0 - PI / 4.0) * pr * pr;
    let fill = 4.0 * run * (6.0 - pr) + run * (2.0 * (30.0 - 2.0 * pr) + 2.0 * (20.0 - 2.0 * pr))
        + 4.0 * (pr * pr * pr - PI * pr * pr * pr / 6.0);
    close(props::volume(&b), 60.0 * 40.0 * 10.0 - 30.0 * 20.0 * 6.0 + fill);
}

/// A flat with unequal legs (`std.Flat` named directly): `la` along the near face from the south
/// face's surface, `lb` along the south face from the near face's.
#[test]
fn a_flat_with_unequal_legs() {
    let src = BLOCK.replace("use std (round, chamfer)", "use std");
    let e = solved(&format!("{src}flat := std.Flat(block.near, block.south, la: 1mm, lb: 3mm)\n\
        taken := solid(flat)\ntaken cut block\n"));
    let b = exported(&e, "block");
    close(props::volume(&b), 40.0 * 24.0 * 12.0 - 0.5 * 1.0 * 3.0 * 40.0);
    let field = MaterialField::read(&e.sketch, ent(&e, "block").i(), 1e-10).unwrap();
    // the near face (y = 0) loses 1 up from the south face (z = 0), the south face 3 in from the near
    assert!(field.side([20.0, 0.5, 0.4]) > 0.0 && field.side([20.0, 2.5, 0.4]) < 0.0);
}

/// **The gate's reading, wider**: each lowered region's pieces and its field read point by point
/// agree everywhere about a corner of the rounded block — its runs, its ball's cell, and beyond.
#[test]
fn a_corners_pieces_are_its_field() {
    let e = solved(&format!("{BLOCK}round(r: 2mm) block\n"));
    let sk = &e.sketch;
    let corner = (0..sk.solids.len()).find(|&i| {
        matches!(lowering::lowered(sk, i), Some(Ok(p)) if p.iter().any(|q| matches!(q, Piece::Corner(_))))
    }).expect("a corner lowered");
    let pieces = lowering::lowered(sk, corner).unwrap().unwrap();
    assert_eq!(pieces.len(), 4, "{pieces:?}");
    let Some(Piece::Corner(c)) = pieces.last() else { panic!() };
    let b = lowering::assemble(pieces.iter()).brep(1.0).unwrap();
    let loc = gcs_core::brep::query::Located::owned(b, 1e-9);
    let field = MaterialField::read(sk, corner, 1e-10).unwrap();
    let mut k = 0x2545f4914f6cdd1du64;
    let mut next = || { k ^= k << 13; k ^= k >> 7; k ^= k << 17; (k >> 11) as f64 / (1u64 << 53) as f64 };
    let mut read = 0;
    for _ in 0..3000 {
        // within 6 of the corner's vertex, either side of every face
        let p: [f64; 3] = std::array::from_fn(|j| c.vertex[j] + 12.0 * next() - 6.0);
        let place = loc.solid_place(p);
        let f = field.side(p);
        if place == gcs_core::brep::query::Place::On || f.abs() < 1e-6 {
            continue;
        }
        assert_eq!(place == gcs_core::brep::query::Place::In, f < 0.0, "at {p:?}: {f}");
        read += 1;
    }
    assert!(read > 2500, "{read}");
}

/// `boss.sv`: a boss's root, where its wall meets the plate — a concave circle, filled by the
/// section turned once about the boss's axis (a torus's face); Pappus.
#[test]
fn a_bosss_root_is_a_ring_exactly() {
    let e = solved(include_str!("../../examples/rounds/boss.sv"));
    let b = exported(&e, "pad");
    let (big, r) = (8.0, 3.0);
    let moment = r * r * (big + r / 2.0) - PI * r * r / 4.0 * (big + r - 4.0 * r / (3.0 * PI));
    close(props::volume(&b), 60.0 * 40.0 * 10.0 + PI * big * big * 20.0 + 2.0 * PI * moment);
    assert_eq!(kinds(&b), [("cylinder", 1), ("plane", 7), ("torus", 1)].into());
}

/// **A run-out** (#110): a run cut off by a face its edge ends on at a slant. The block's east side
/// leans in (its top 10 short of its foot) and its west side leans out (10 past), and only its
/// near face's south edge is rounded. At the east end the face reaches into the run's prism and
/// cuts it off: the length kept at height `z` above the edge is `40 − (10/24)·z`, so the run takes
/// its section's area along 40 less `(10/24)·z̄` of it, `z̄ = r(10 − 3π)/(12 − 3π)` its centroid's
/// height. At the west end the face leans away, and the run stops square at the edge's end, as
/// the slab its nearest point of the meeting must be on says.
#[test]
fn a_run_cut_off_at_a_slant_by_the_face_it_ends_on() {
    let src = BLOCK.replace("fix((40, 24)) b2", "fix((30, 24)) b2").replace("fix((0, 24)) b3", "fix((-10, 24)) b3");
    let e = solved(&format!("{src}block.near round(r: 2mm) block.south\n"));
    assert_eq!(lowered(&e), (1, 1));
    let b = exported(&e, "block");
    let r = 2.0;
    let area = (1.0 - PI / 4.0) * r * r;
    let zbar = r * (10.0 - 3.0 * PI) / (12.0 - 3.0 * PI);
    close(props::volume(&b), 40.0 * 24.0 * 12.0 - area * 40.0 + 10.0 / 24.0 * zbar * area);
    assert_eq!(kinds(&b), [("cylinder", 1), ("plane", 7)].into());
}

/// The leaning block's whole near rim: each of the four runs cut off at a slant where it meets the
/// next side, and the runs unioned at the corners (mitres). Held to the field by the gate; its
/// volume between the block's and the block's less every run's whole prism.
#[test]
fn a_leaning_rim_runs_out_at_every_corner() {
    let src = BLOCK.replace("fix((40, 24)) b2", "fix((30, 24)) b2");
    let e = solved(&format!("{src}block.near round(r: 2mm) block\n"));
    assert_eq!(lowered(&e), (4, 6));
    let b = exported(&e, "block");
    let (r, rim) = (2.0, 40.0 + 26.0 + 30.0 + 24.0);
    let whole = (40.0 + 30.0) / 2.0 * 24.0 * 12.0;
    let v = props::volume(&b);
    assert!(v < whole && v > whole - (1.0 - PI / 4.0) * r * r * rim, "{v}");
    assert_eq!(kinds(&b), [("cylinder", 4), ("plane", 6)].into());
}

/// **What no entry builds stays the field's**: the leaning block rounded all over — at the corners
/// of its leaning side three faces meet not square to one another, which no entry builds yet. The
/// export is refused, naming why, and the field still reads it.
#[test]
fn a_round_no_entry_builds_is_refused_by_name() {
    let src = BLOCK.replace("fix((40, 24)) b2", "fix((30, 24)) b2");
    let e = solved(&format!("{src}round(r: 2mm) block\n"));
    let body = ent(&e, "block").i();
    let say = Say { stage: &|_: &str| {}, mark: &|_| {} };
    let refusal = export::exact(&e.sketch, body, None, None, &say).err().expect("refused");
    assert!(format!("{refusal:?}").contains("a corner whose faces are not square to one another: no exact \
        construction is made of it yet"), "{refusal:?}");
    let field = MaterialField::read(&e.sketch, body, 1e-10).unwrap();
    assert!(field.side([20.0, 0.2, 0.2]) > 0.0 && field.side([20.0, 6.0, 6.0]) < 0.0);
}

/// A round section's area beside a square corner, `(1 − π/4)r²`, and how far its centroid stands
/// off each face, `r(10 − 3π)/(12 − 3π)`.
fn spandrel(r: f64) -> (f64, f64) {
    ((1.0 - PI / 4.0) * r * r, r * (10.0 - 3.0 * PI) / (12.0 - 3.0 * PI))
}

/// `slot.sv`: rounds along chains of edges meeting tangent. Where a flat side goes on into a half
/// circle there is no corner, so nothing ends either round there: the boss's foot is filled along
/// its two flat sides (prisms) and round its two ends (stretches of rings, a half turn each), each
/// glued to the next at the section they share; the pocket's rim likewise, four quarter turns.
/// Pappus for the rings: the section's area along its centroid's circle, `R + z̄` from the axis.
#[test]
fn a_slots_rounds_go_on_round_its_tangent_ends() {
    let e = solved(include_str!("../../examples/rounds/slot.sv"));
    assert_eq!(lowered(&e).0, 12);
    let b = exported(&e, "part");
    let ((foot, fz), (rim, rz)) = (spandrel(2.0), spandrel(1.0));
    let boss = (18.0 * 12.0 + PI * 36.0) * 8.0;
    let pocket = (20.0 * 16.0 - 4.0 * (16.0 - 4.0 * PI)) * 6.0;
    let want = 80.0 * 50.0 * 10.0 + boss - pocket + foot * (2.0 * 18.0 + 2.0 * PI * (6.0 + fz))
        - rim * (2.0 * 12.0 + 2.0 * 8.0 + 2.0 * PI * (4.0 + rz));
    close(props::volume(&b), want);
    assert_eq!(kinds(&b).get("torus"), Some(&6));
}

/// `pad.sv`: the boss's root a ring, two of the plate's near edges each a run cut off flush at the
/// plate's sides, and the bore drilled through after — the rounds read the faces the body has
/// before it.
#[test]
fn the_pads_rounds_are_built_exactly() {
    let e = solved(include_str!("../../examples/rounds/pad.sv"));
    let b = exported(&e, "pad");
    let ((root, rz), (edge, _)) = (spandrel(3.0), spandrel(2.0));
    let want = 60.0 * 40.0 * 10.0 + PI * 64.0 * 20.0 - PI * 16.0 * 30.0 + root * 2.0 * PI * (8.0 + rz)
        - 2.0 * 40.0 * edge;
    close(props::volume(&b), want);
    assert_eq!(kinds(&b), [("cylinder", 4), ("plane", 7), ("torus", 1)].into());
}

/// **No term left unread**: the hand-written round with one side moved off its face
/// (`p inside(1mm) a`) is no set of the table's — every term must be one an entry reads — so it is
/// not built, and its export says no entry builds it.
#[test]
fn a_set_with_a_term_no_entry_reads_is_not_lowered() {
    let src = include_str!("../../examples/rounds/by_hand.sv").replacen("  p inside a\n", "  p inside(1mm) a\n", 1);
    assert_ne!(src, include_str!("../../examples/rounds/by_hand.sv"));
    let e = solved(&src);
    assert_eq!(lowered(&e), (0, 0));
    let say = Say { stage: &|_: &str| {}, mark: &|_| {} };
    let refusal = export::exact(&e.sketch, ent(&e, "block").i(), None, None, &say).err().expect("refused");
    assert!(format!("{refusal:?}").contains("no entry of the lowering table builds"), "{refusal:?}");
}
