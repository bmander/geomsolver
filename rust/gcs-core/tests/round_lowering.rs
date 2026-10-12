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
use crate::fillet_definition::{beside_cylinder, root_ring, square_run};

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
/// leans in (its top 10 short of its foot) and its west side leans out (5 past), and only its near
/// face's south edge is rounded. The run is carried on past each end of its edge and cut off there
/// by the face it ends on, however it stands (#195). At the east end the face reaches into the
/// run's prism: the length kept at height `z` above the edge is short of 40 by `(10/24)·z`. At the
/// west end it leans away, and the run carries on past the edge's end, `(5/24)·z` at height `z`.
/// So the run takes its section's area along 40, less `(10/24 − 5/24)·z̄` of it, `z̄ = r(10 −
/// 3π)/(12 − 3π)` its centroid's height.
#[test]
fn a_run_cut_off_at_a_slant_by_the_face_it_ends_on() {
    let src = BLOCK.replace("fix((40, 24)) b2", "fix((30, 24)) b2").replace("fix((0, 24)) b3", "fix((-5, 24)) b3");
    let e = solved(&format!("{src}block.near round(r: 2mm) block.south\n"));
    assert_eq!(lowered(&e), (1, 1));
    let b = exported(&e, "block");
    let (area, zbar) = spandrel(2.0);
    let want = (40.0 + 35.0) / 2.0 * 24.0 * 12.0 - area * 40.0 + (10.0 - 5.0) / 24.0 * zbar * area;
    close(props::volume(&b), want);
    // running out flush into the west face, it leaves no sliver of the near face there
    assert_eq!(kinds(&b), [("cylinder", 1), ("plane", 6)].into());
    // the app builds a solid about its own origin (`cad::shifted`): the planes the run is kept
    // behind move with the rest (a cut's volume cannot tell, its prism past a face cutting air)
    let recipe = gcs_core::solid::cad::recipe(&e.sketch, ent(&e, "block").i()).unwrap();
    let by = [5.0, 7.0, 11.0];
    let planes = |r: &gcs_core::json::Json| -> Vec<[f64; 3]> {
        r.get("nodes").unwrap().arr().iter().filter_map(|n| n.get("pieces")).flat_map(|p| p.arr().iter())
            .filter_map(|p| p.get("within")).flat_map(|w| w.arr().iter())
            .map(|h| { let a = h.get("at").unwrap().arr(); [a[0].as_f64(), a[1].as_f64(), a[2].as_f64()] })
            .collect()
    };
    let (before, after) = (planes(&recipe), planes(&gcs_core::solid::cad::shifted(&recipe, by)));
    assert_eq!(before.len(), 2);
    assert!(before.iter().zip(&after).all(|(p, q)| (0..3).all(|k| (p[k] - by[k] - q[k]).abs() < 1e-12)));
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
    assert!(v < whole && v > whole - square_run(r) * rim, "{v}");
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

/// A round section's area beside a square corner (`square_run`), and how far its centroid stands
/// off each face, `r(10 − 3π)/(12 − 3π)`. (Turned about an axis `R` from the corner, Pappus:
/// `fillet_definition::root_ring`.)
fn spandrel(r: f64) -> (f64, f64) {
    (square_run(r), r * (10.0 - 3.0 * PI) / (12.0 - 3.0 * PI))
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
    let (foot, rim) = (square_run(2.0), square_run(1.0));
    let boss = (18.0 * 12.0 + PI * 36.0) * 8.0;
    let pocket = (20.0 * 16.0 - 4.0 * (16.0 - 4.0 * PI)) * 6.0;
    let want = 80.0 * 50.0 * 10.0 + boss - pocket + foot * 2.0 * 18.0 + root_ring(6.0, 2.0)
        - rim * (2.0 * 12.0 + 2.0 * 8.0) - root_ring(4.0, 1.0);
    close(props::volume(&b), want);
    assert_eq!(kinds(&b).get("torus"), Some(&6));
}

/// `pad.sv`: the boss's root a ring, two of the plate's near edges each a run cut off flush at the
/// plate's sides, and the bore drilled through after — the rounds read the faces the body has
/// before it.
#[test]
fn the_pads_rounds_are_built_exactly() {
    let e = solved(include_str!("../../examples/rounds/pad.sv"));
    // the app's exact solid, built about the pad's own origin (its first corner, off the world's):
    // the pieces move with the rest (`solventc`'s export of it is the slow tier's)
    let x = e.sketch.exact_solid(ent(&e, "pad").i()).unwrap();
    let want = 60.0 * 40.0 * 10.0 + PI * 64.0 * 20.0 - PI * 16.0 * 30.0 + root_ring(8.0, 3.0)
        - 2.0 * 40.0 * square_run(2.0);
    close(props::volume(&x.brep) / x.mm.powi(3), want);
    assert_eq!(kinds(&x.brep), [("cylinder", 4), ("plane", 7), ("torus", 1)].into());
}

/// `rail.sv`: the rod splits the plate's top in two, so the round is over a face in pieces — a run
/// along each line where a piece meets the rod. In the section the rod is a circle: a prism of the
/// line, the ball's arc and the rod's arc, running out flush at the plate's ends.
#[test]
fn a_rods_sides_are_filled_where_it_splits_the_plates_top() {
    let e = solved(include_str!("../../examples/rounds/rail.sv"));
    let b = exported(&e, "rail");
    close(props::volume(&b), 60.0 * 10.0 * 40.0 + PI * 36.0 / 2.0 * 40.0 + 2.0 * 40.0 * beside_cylinder(6.0, 2.0));
    assert_eq!(kinds(&b).get("cylinder"), Some(&3));
}

/// The same rod cut as a groove, its rims along the plate's top rounded: convex, so the same word
/// takes them away — the ball within the plate and outside the groove, its section the fill's
/// mirror. (Rounding its rims at the plate's ends too mitres a run against a ring's stretch on one
/// cylinder, which this kernel's union does not yet resolve.)
#[test]
fn a_grooves_rims_are_rounded_away() {
    let src = include_str!("../../examples/rounds/rail.sv").replace("rod union rail", "rod cut rail")
        .replace("round(r: r) rail.plate\n", "round(r: r) rail.plate.cd\n");
    assert!(src.contains("rail.plate.cd"));
    let e = solved(&src);
    let b = exported(&e, "rail");
    close(props::volume(&b), 60.0 * 10.0 * 40.0 - PI * 36.0 / 2.0 * 40.0 - 2.0 * 40.0 * beside_cylinder(6.0, 2.0));
}

/// The groove's rim at the plate's near face: a half circle, its round a stretch of a ring ending
/// at the plate's top — a plane through the groove's axis, standing where the turn stops, so the
/// sector already ends there. In its meridian the corner is square: the spandrel, turned a half
/// turn about the axis at `R + z̄` (Pappus).
#[test]
fn a_rings_stretch_ends_on_a_plane_through_its_axis() {
    let src = include_str!("../../examples/rounds/rail.sv").replace("rod union rail", "rod cut rail")
        .replace("round(r: r) rail.plate\n", "round(r: r) rail.plate.near\n");
    let e = solved(&src);
    let b = exported(&e, "rail");
    close(props::volume(&b), 60.0 * 10.0 * 40.0 - PI * 36.0 / 2.0 * 40.0 - root_ring(6.0, 2.0) / 2.0);
    assert_eq!(kinds(&b).get("torus"), Some(&1));
}

/// `knob.sv` (#83's knob): the rod's root a ring between a plane and a cylinder, its neck one
/// between a cylinder and a sphere — in the meridian the sphere is a circle, the neck's section
/// the rod's line, the rolled ball's arc and the ball's own arc (`fillet_meridian::knob_rounded`).
#[test]
fn a_knobs_neck_turns_a_section_with_a_curved_side() {
    let e = solved(include_str!("../../examples/rounds/knob.sv"));
    let b = exported(&e, "knob");
    close(props::volume(&b), crate::fillet_meridian::knob_rounded([0.0, 44.0], 8.0, 4.0, 1.5));
    assert_eq!(kinds(&b).get("torus"), Some(&2));
}

/// Of a cylinder of radius `small` standing on the axis of one of radius `big` (square to it), the
/// volume inside the bigger: `∫ 2·√(small² − y²)·√(big² − y²) dy`, with `y = small·sin θ` a smooth
/// periodic integrand, which the midpoint rule over its period takes to roundoff.
fn inside_crossing(big: f64, small: f64) -> f64 {
    let n = 64;
    let mut total = 0.0;
    for k in 0..n {
        // the midpoint rule over a whole period of a smooth periodic integrand is spectral
        let t = -PI / 2.0 + PI * (k as f64 + 0.5) / n as f64;
        let (sn, cs) = t.sin_cos();
        total += 2.0 * small * small * cs * cs * (big * big - small * small * sn * sn).sqrt();
    }
    total * PI / n as f64
}

/// The volume is held to the rolled pieces' closed form — #85's quadrature
/// (`fillet_definition::rolled`) — to 1e-6, as #194 asks of a rolled piece.
fn near_quadrature(got: f64, want: f64) {
    assert!((got - want).abs() <= 1e-6 * want, "{got} against {want}");
}

/// `tee.sv` (#85's tee): a branch pipe standing square on a main pipe, the crotch filled. No line
/// or circle carries where they meet: the ball rolls along a traced loop (`brep::fillet::roll`),
/// against the body as its other terms make it, its canal fitted.
#[test]
fn a_tees_crotch_is_rolled() {
    let e = solved(include_str!("../../examples/rounds/tee.sv"));
    assert_eq!(lowered(&e).0, 1);
    let b = exported(&e, "tee");
    let want = PI * 100.0 * 60.0 + PI * 36.0 * 25.0 - inside_crossing(10.0, 6.0)
        + crate::fillet_definition::tee_crotch(10.0, 6.0, 2.0);
    near_quadrature(props::volume(&b), want);
}

/// `bore.sv` (#85's bore): a bore drilled square into a solid pipe's wall, its rim rounded away —
/// convex, along a saddle, rolled against the pipe as the cut leaves it (the stratum).
#[test]
fn a_bores_rim_on_a_pipe_is_rolled_away() {
    let e = solved(include_str!("../../examples/rounds/bore.sv"));
    assert_eq!(lowered(&e).0, 1);
    let b = exported(&e, "pipe");
    let want = PI * 100.0 * 60.0 - inside_crossing(10.0, 6.0) - crate::fillet_definition::bore_rim(10.0, 6.0, 1.0);
    near_quadrature(props::volume(&b), want);
}

/// Of a disc of radius `r`, the segment beyond a chord `d` from its centre: its area, and how far
/// its centroid stands from the centre.
fn segment(r: f64, d: f64) -> (f64, f64) {
    let half = (d / r).acos();
    let area = r * r * half - d * (r * r - d * d).sqrt();
    (area, 2.0 * (r * r - d * d).powf(1.5) / (3.0 * area))
}

/// `runout.sv` (#110's run-outs): a boss's root against a sloped top, ending obliquely at the
/// wedge's near face, and a hole's rim on another's slope, breaking out of its low end — each a
/// ball rolled along a traced run, carried on past each end and cut off by the face there (#195).
/// Each is held to #85's quadrature cut off by the same plane (`fillet_definition::sloped`):
///
/// - the lug: the wedge, plus the boss above the slope (all of it, less the part of its disc short
///   of the near face under the slope, at its centroid's height 15), plus the root;
/// - the vent: the block less the hole's disc short of the low end, under the slope (at its
///   centroid's height), less the rim.
///
/// With the boss centred on the near face, the face cuts the root square: half a whole boss's root.
#[test]
fn rounds_that_run_out_onto_a_face_are_rolled_and_cut_off() {
    use crate::fillet_definition::{sloped, sloped_root, RIM, ROOT};
    let src = include_str!("../../examples/rounds/runout.sv");
    let e = solved(src);
    assert_eq!(lowered(&e).0, 2);
    let wedge = 40.0 * 15.0 * 24.0;
    let (seg, _) = segment(6.0, 3.0);
    let lug = wedge + PI * 36.0 * 30.0 - 15.0 * (PI * 36.0 - seg)
        + sloped(6.0, [20.0, -3.0], [20.0, 0.25], 2.0, ROOT, Some(([0.0, 1.0, 0.0], 0.0)));
    near_quadrature(props::volume(&exported(&e, "lug")), lug);
    let (seg, off) = segment(4.0, 2.0);
    let short = PI * 16.0 - seg;
    let x = (98.0 * PI * 16.0 - (98.0 + off) * seg) / short;
    let vent = wedge - (35.0 - 0.25 * x) * short
        - sloped(4.0, [98.0, -12.0], [35.0, 0.25], 1.0, RIM, Some(([1.0, 0.0, 0.0], 100.0)));
    near_quadrature(props::volume(&exported(&e, "vent")), vent);
    let centred = solved(&src.replace("fix((20, -3)) bc", "fix((20, 0)) bc"));
    let b = exported(&centred, "lug");
    let want = wedge + PI * 36.0 * 30.0 - 15.0 * PI * 18.0 + sloped_root(6.0, [20.0, 0.0], [20.0, 0.25], 2.0) / 2.0;
    near_quadrature(props::volume(&b), want);
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
