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

/// **Corners not square** (#218's 3): the leaning block rounded all over. Where its leaning side
/// meets the top and the bottom, three faces meet at a slant; the ball touching all three stands
/// `r` from each (a 3×3 solve), and its cell — the trihedron cut by the three sections through it
/// square to the edges — less the ball is the corner's piece, each run cut short at its section
/// there. The body is the shrunk prism grown by the ball (Steiner): `V + S r + r² Σ L(π − θ)/2 +
/// 4πr³/3` over the prism of the trapezoid inset by `r`, `h − 2r` deep.
#[test]
fn a_leaning_block_rounded_all_over_is_steiners_exactly() {
    let src = BLOCK.replace("fix((40, 24)) b2", "fix((30, 24)) b2");
    let e = solved(&format!("{src}round(r: 2mm) block\n"));
    let b = exported(&e, "block");
    let r = 2.0;
    // the east side `24x + 10z = 960` moved in by `r` (its normal 26 long), the others square
    let east = |z: f64| (960.0 - 26.0 * r - 10.0 * z) / 24.0;
    let (lo, hi) = (r, 24.0 - r);
    let (bottom, top) = (east(lo) - r, east(hi) - r);
    let area = (bottom + top) / 2.0 * (hi - lo);
    let slant = ((bottom - top).powi(2) + (hi - lo).powi(2)).sqrt();
    let (perimeter, h) = (bottom + top + (hi - lo) + slant, 12.0 - 2.0 * r);
    close(props::volume(&b), steiner(area, perimeter, h, r));
    assert_eq!(kinds(&b), [("cylinder", 12), ("plane", 6), ("sphere", 8)].into());
}

/// Of two discs of radii `a` and `b`, centres `d` apart, the lens they share.
fn lens(a: f64, b: f64, d: f64) -> f64 {
    let half = |a: f64, b: f64| ((d * d + a * a - b * b) / (2.0 * d * a)).acos();
    a * a * half(a, b) + b * b * half(b, a)
        - 0.5 * ((-d + a + b) * (d + a - b) * (d - a + b) * (d + a + b)).sqrt()
}

/// Of two discs of radii `a` and `b`, centres `d` apart, the fill a ball of radius `r` outside
/// both leaves at one of the cusps where their circles cross: the triangle of the three centres
/// less each disc's sector in it, the half lens in it counted once.
fn cusp_fill(a: f64, b: f64, d: f64, r: f64) -> f64 {
    let (sa, sb) = (a + r, b + r);
    let alpha = ((d * d + sa * sa - sb * sb) / (2.0 * d * sa)).acos();
    let beta = ((d * d + sb * sb - sa * sa) / (2.0 * d * sb)).acos();
    let gamma = PI - alpha - beta;
    let triangle = 0.5 * d * sa * alpha.sin();
    triangle - 0.5 * (a * a * alpha + b * b * beta + r * r * gamma) + lens(a, b, d) / 2.0
}

/// **A round between two curved faces** (#218's 5), `twin.sv`: two parallel rods overlapping,
/// made one, the two straight seams where their walls meet filled. In the section both walls are
/// circles, and the ball stands where the two circles grown by `r` meet: each seam's fill a prism
/// of two arcs and the ball's.
#[test]
fn a_seam_between_two_rods_is_filled_exactly() {
    let e = solved(include_str!("../../examples/rounds/twin.sv"));
    assert_eq!(lowered(&e).0, 1);
    let b = exported(&e, "bar");
    let (big, small, d, r) = (10.0, 8.0, 14.0, 2.0);
    let section = PI * (big * big + small * small) - lens(big, small, d) + 2.0 * cusp_fill(big, small, d, r);
    close(props::volume(&b), section * 40.0);
    // (the union leaves the caps in pieces)
    assert_eq!(kinds(&b).keys().copied().collect::<Vec<_>>(), ["cylinder", "plane"]);
}

/// The same rods as two bores through a block, overlapping: the cusps where the bores' walls meet
/// are convex, so the same word rounds them away — the ball within the material, outside both
/// bores.
#[test]
fn a_cusp_between_two_bores_is_rounded_away() {
    let e = solved("unit mm\nuse std (round)\nin std.front {\n\
        a := point\nb := point\nc := point\nd := point\n\
        fix((-20, -20)) a\nfix((30, -20)) b\nfix((30, 20)) c\nfix((-20, 20)) d\n\
        (ab := line(a, b)) -> (bc := line(b, c)) -> (cd := line(c, d)) -> (da := line(d, a)) -> close\n\
        c1 := point\nc2 := point\nfix((0, 0)) c1\nfix((14, 0)) c2\n\
        big := circle(center: c1) hint(r: 10)\nsmall := circle(center: c2) hint(r: 8)\n\
        radius(10mm) big\nradius(8mm) small\n}\n\
        block := solid(face(ab, bc, cd, da), from: 0mm, to: 40mm)\n\
        left := solid(face(big), from: 0mm, to: 40mm)\nright := solid(face(small), from: 0mm, to: 40mm)\n\
        left cut block\nright cut block\nblock.right.small round(r: 2mm) block.left.big\n");
    let b = exported(&e, "block");
    // at each cusp the material outside both bores that the ball cannot reach: the same figure
    // as the twin bar's fill
    let (big, small, d, r) = (10.0, 8.0, 14.0, 2.0);
    let bores = PI * (big * big + small * small) - lens(big, small, d);
    close(props::volume(&b), (50.0 * 40.0 - bores - 2.0 * cusp_fill(big, small, d, r)) * 40.0);
}

/// **What no entry builds stays the field's**: the twin bar's seams chamfered. Between two circles
/// a flat's sum of distances is a Cartesian oval, no conic, which no entry builds: the export is
/// refused, naming why, and the field still reads it.
#[test]
fn a_round_no_entry_builds_is_refused_by_name() {
    let src = include_str!("../../examples/rounds/twin.sv").replace("use std (round)", "use std (chamfer)")
        .replace("bar.right.small round(r: r) bar.left.big", "bar.right.small chamfer(d: 1mm) bar.left.big");
    let e = solved(&src);
    let body = ent(&e, "bar").i();
    let say = Say { stage: &|_: &str| {}, mark: &|_| {} };
    let refusal = export::exact(&e.sketch, body, None, None, &say).err().expect("refused");
    assert!(format!("{refusal:?}").contains("a flat between two curved faces is no line or conic: no exact \
        construction is made of it yet"), "{refusal:?}");
    let field = MaterialField::read(&e.sketch, body, 1e-10).unwrap();
    // (the front plane's normal, toward the viewer, is −y: the bar stands at y in (−40, 0))
    assert!(field.side([0.0, -20.0, 0.0]) < 0.0 && field.side([7.0, -20.0, 20.0]) > 0.0);
}

/// **A chamfer beside a curved face** (#218's 6): `rail.sv`'s fills chamfered instead. In the
/// section the rod is a circle about `c` on the plate's top, and the flat — `d_rod + d_top = d` —
/// is the conic `(ρ − 6) + ρ sin θ = d` about `c`, from the top `d` out along it to the rod `d` up:
/// `ρ(θ) = (6 + d)/(1 + sin θ)` for `0 ≤ θ ≤ asin(d/6)`, its area beside the rod `∫ (ρ² − 36)/2 dθ`
/// in closed form (`t = tan(θ/2)`, `∫ dθ/(1 + sin θ)² = −2/u + 2/u² − 4/(3u³)` with `u = 1 + t`).
/// Built as a prism over a rational quadratic, an extrusion face.
#[test]
fn a_chamfer_beside_a_rod_is_a_conic_prism() {
    let src = include_str!("../../examples/rounds/rail.sv").replace("use std (round)", "use std (round, chamfer)")
        .replace("rail.rod round(r: r) rail.plate", "rail.rod chamfer(d: 2mm) rail.plate");
    let e = solved(&src);
    assert_eq!(lowered(&e).0, 1);
    let b = exported(&e, "rail");
    let (r, d): (f64, f64) = (6.0, 2.0);
    let end = (d / r).asin();
    let anti = |theta: f64| {
        let u = 1.0 + (theta / 2.0).tan();
        -2.0 / u + 2.0 / (u * u) - 4.0 / (3.0 * u * u * u)
    };
    let area = 0.5 * (r + d) * (r + d) * (anti(end) - anti(0.0)) - 0.5 * r * r * end;
    close(props::volume(&b), 60.0 * 10.0 * 40.0 + PI * r * r / 2.0 * 40.0 + 2.0 * 40.0 * area);
    assert_eq!(kinds(&b).get("extrusion"), Some(&2), "{:?}", kinds(&b));
}

/// Over the section a round of radius `r` takes beside a square corner, `∫ f(u, h) du`: `u` the
/// depth in from one face, `h` the section's height there (`f` the integrand per `du`, its height
/// in it). `u = r(1 − cos t)` makes it smooth for Simpson, to roundoff at a few thousand steps.
fn over_round(r: f64, f: impl Fn(f64, f64) -> f64) -> f64 {
    crate::brep::simpson(0.0, PI / 2.0, 4000, |t: f64| f(r * (1.0 - t.cos()), r - r * t.sin()) * r * t.sin())
}

/// Steiner's formula for a prism of `area`, `perimeter` and depth `h`, convex, grown by a ball of
/// radius `r`: `A h + (2A + p h) r + π r² (h + p/2) + 4πr³/3`.
fn steiner(area: f64, perimeter: f64, h: f64, r: f64) -> f64 {
    area * h + (2.0 * area + perimeter * h) * r + PI * r * r * (h + perimeter / 2.0) + 4.0 * PI * r * r * r / 3.0
}

/// `dshaft.sv` with `round(r: r) shaft` in place of its round all over.
fn dshaft(round: &str) -> Elaborated {
    let src = include_str!("../../examples/rounds/dshaft.sv");
    assert!(src.contains("round(r: r) shaft\n"));
    solved(&src.replace("round(r: r) shaft\n", &format!("{round}\n")))
}

/// **A run cut off by a curved face** (#218's 2, #110), `post.sv`: the plate's top edge along its
/// near side runs into the post standing there, whose axis the edge passes through. Each run is
/// carried on into the post and kept outside its wall: at depth `u` in from the side, the run's
/// section stands `30 − √(64 − u²)` long, so it takes `∫ h(u) (30 − √(64 − u²)) du` over the round's
/// section (`h` its height at `u`; `u = r(1 − cos t)` makes it smooth for Simpson).
#[test]
fn a_run_cut_off_by_a_post_it_runs_into() {
    let e = solved(include_str!("../../examples/rounds/post.sv"));
    assert_eq!(lowered(&e).0, 1);
    let b = exported(&e, "stand");
    let (r, big) = (2.0, 8.0);
    let run = over_round(r, |u, h| h * (30.0 - (big * big - u * u).sqrt()));
    let want = 60.0 * 40.0 * 10.0 + PI * big * big * 30.0 - PI * big * big / 2.0 * 10.0 - 2.0 * run;
    close(props::volume(&b), want);
    // the app builds a solid about its own origin (`cad::shifted`): the post's wall the runs are
    // cut off by moves with the rest
    let recipe = gcs_core::solid::cad::recipe(&e.sketch, ent(&e, "stand").i()).unwrap();
    let by = [5.0, 7.0, 11.0];
    let walls = |r: &gcs_core::json::Json| -> Vec<[f64; 3]> {
        r.get("nodes").unwrap().arr().iter().filter_map(|n| n.get("pieces")).flat_map(|p| p.arr().iter())
            .filter_map(|p| p.get("kept")).flat_map(|w| w.arr().iter())
            .map(|h| { let a = h.get("origin").unwrap().arr(); [a[0].as_f64(), a[1].as_f64(), a[2].as_f64()] })
            .collect()
    };
    let (before, after) = (walls(&recipe), walls(&gcs_core::solid::cad::shifted(&recipe, by)));
    assert_eq!(before.len(), 2);
    assert!(before.iter().zip(&after).all(|(p, q)| (0..3).all(|k| (p[k] - by[k] - q[k]).abs() < 1e-12)));
}

/// The same within a curved face: a D-shaped shaft (a round of 10 about its axis with a flat 6
/// from it), the flat's top edge rounded. The run ends on the round at both ends, from inside:
/// carried on out of the shaft past its wall and kept within it. At depth `u` in from the flat its
/// section is the chord `2√(100 − (6 − u)²)` long.
#[test]
fn a_run_cut_off_at_both_ends_by_the_round_it_lies_within() {
    let e = dshaft("shaft.near round(r: r) shaft.flat");
    assert_eq!(lowered(&e).0, 1);
    let b = exported(&e, "shaft");
    let (r, big, c): (f64, f64, f64) = (2.0, 10.0, 6.0);
    let run = over_round(r, |u, h| h * 2.0 * (big * big - (c - u) * (c - u)).sqrt());
    close(props::volume(&b), (PI * big * big - segment(big, c).0) * 20.0 - run);
}

/// **A ring's stretch cut off by a plane at its end** (#218's 2): the D-shaped shaft's top rim
/// rounded — round its round, from one end of the flat to the other. The flat is no plane through
/// the axis, so the stretch is carried on round past each end and kept within the flat. A point
/// of the section `ρ = 10 − u` from the axis is kept over the turn where `ρ cos θ <= 6`, `2π −
/// 2 acos(6/ρ)` of it: the section turned through that much at each radius.
#[test]
fn a_rings_stretch_cut_off_by_a_flat_at_its_ends() {
    let e = dshaft("shaft.near round(r: r) shaft.rim");
    assert_eq!(lowered(&e).0, 1);
    let b = exported(&e, "shaft");
    let (r, big, c): (f64, f64, f64) = (2.0, 10.0, 6.0);
    let ring = over_round(r, |u, h| h * (big - u) * (2.0 * PI - 2.0 * (c / (big - u)).acos()));
    close(props::volume(&b), (PI * big * big - segment(big, c).0) * 20.0 - ring);
}

/// **Corners where a curved face meets two planes** (#218's 4), `dshaft.sv`: the D-shaped shaft
/// rounded all over. At each of its four corners its flat, its round and a cap meet; the ball touching all three
/// stands where Newton's steps put it, and its cell is the cell over the three surfaces — a patch of
/// the round among them — the flat's runs, the rims' rings and the seams' runs each cut short at
/// its section there. The D is convex, so the body is the D shrunk by `r`, `h − 2r` deep, grown by
/// the ball (Steiner, as for a polygon): `A h + (2A + p h) r + π r² (h + p/2) + 4πr³/3`.
#[test]
fn a_d_shaft_rounded_all_over_is_steiners_exactly() {
    let e = solved(include_str!("../../examples/rounds/dshaft.sv"));
    let b = exported(&e, "shaft");
    let r: f64 = 2.0;
    let (big, c) = (10.0 - r, 6.0 - r);
    let area = PI * big * big - segment(big, c).0;
    let perimeter = big * (2.0 * PI - 2.0 * (c / big).acos()) + 2.0 * (big * big - c * c).sqrt();
    close(props::volume(&b), steiner(area, perimeter, 20.0 - 2.0 * r, r));
}

/// **A ring's stretch cut off by a curved face** (#218's 2): the twin bar's big rod's rim at one
/// end rounded — round the rod from one cusp to the other, ending on the small rod's wall at both.
/// Carried on round into the small rod and kept outside it: a point of the section `ρ = 10 − u` from
/// the big rod's axis is kept over the turn outside the small rod's disc, all but `2 acos((ρ² + 14²
/// − 8²)/(28ρ))` of it.
#[test]
fn a_rings_stretch_cut_off_by_a_cylinder_at_its_ends() {
    let src = include_str!("../../examples/rounds/twin.sv")
        .replace("bar.right.small round(r: r) bar.left.big", "bar.left.near round(r: 1mm) bar.left.big");
    let e = solved(&src);
    assert_eq!(lowered(&e).0, 1);
    let b = exported(&e, "bar");
    let (r, big, small, d): (f64, f64, f64, f64) = (1.0, 10.0, 8.0, 14.0);
    let ring = over_round(r, |u, h| {
        let rho = big - u;
        h * rho * (2.0 * PI - 2.0 * ((rho * rho + d * d - small * small) / (2.0 * rho * d)).acos())
    });
    close(props::volume(&b), (PI * (big * big + small * small) - lens(big, small, d)) * 40.0 - ring);
}

/// And within a ball: a quarter of a ball of 10 (its half disc turned a quarter turn), the edge
/// where its two flats meet rounded. The edge runs through the centre from pole to pole, ending on
/// the sphere at both: the run carried on out of the ball and kept within it. At `(a, b)` from the
/// edge its section is the chord `2√(100 − a² − b²)` long; over the section's height at `a` that
/// is `[b√(A² − b²) + A² asin(b/A)]` from 0, `A² = 100 − a²`.
#[test]
fn a_run_cut_off_by_the_ball_it_lies_within() {
    let e = solved("unit mm\nuse std (round)\nin std.front {\n\
        a0 := point\na1 := point\nfix((0, -10)) a0\nfix((0, 10)) a1\nspindle := line(a0, a1)\n\
        c0 := point\nfix((0, 0)) c0\nshell := arc(center: c0, start: a0, end: a1) hint(r: 10)\n\
        shut := line(a1, a0)\n}\n\
        wedge := solid(face(shell, shut), about: spindle, sweep: 90deg)\n\
        wedge.start round(r: 2mm) wedge.end\n");
    assert_eq!(lowered(&e).0, 1);
    let b = exported(&e, "wedge");
    let (r, big): (f64, f64) = (2.0, 10.0);
    let run = over_round(r, |a, h| {
        let aa = big * big - a * a;
        h * (aa - h * h).sqrt() + aa * (h / aa.sqrt()).asin()
    });
    close(props::volume(&b), PI * big * big * big / 3.0 - run);
    assert_eq!(kinds(&b).get("sphere"), Some(&1), "{:?}", kinds(&b));
}

/// `knob.sv` chamfered instead (#218's 6, about an axis): the root's flat between the disc's top
/// and the rod is a cone (its triangle turned, Pappus), and the neck's between the rod and the
/// ball the conic `(x − 4) + (|q − c| − 8) = d` of the meridian turned — in polar about the
/// ball's centre `ρ = (12 + d)/(1 + cos φ)` out to where it meets the rod and the ball, from the
/// ball (`ρ = 8`) or the rod (`ρ = 4 / cos φ`), whichever stands farther; turned, Pappus over
/// `∫∫ x ρ dρ dφ`.
#[test]
fn a_knobs_neck_chamfered_is_a_conic_turned() {
    let src = include_str!("../../examples/rounds/knob.sv").replace("use std (round)", "use std (chamfer)")
        .replace("knob.rod round(r: 2mm) knob.disc", "knob.rod chamfer(d: 2mm) knob.disc")
        .replace("knob.rod round(r: 1.5mm) knob.ball", "knob.rod chamfer(d: 1.5mm) knob.ball");
    let e = solved(&src);
    let b = exported(&e, "knob");
    let (rod, big, d) = (4.0, 8.0, 1.5);
    let unrounded = crate::fillet_meridian::knob_rounded([0.0, 44.0], big, rod, 1.5)
        - root_ring(rod, 2.0) - crate::fillet_meridian::knob_neck([0.0, 44.0], big, rod, 1.5);
    let root = 2.0 * PI * (rod + 2.0 / 3.0) * 2.0;
    let hi = |phi: f64| (big + rod + d) / (1.0 + phi.cos());
    let moment = |phi: f64, lo: f64| (hi(phi).powi(3) - lo.powi(3)) / 3.0 * phi.cos();
    // below the ball's equator: from the conic meeting the rod, through the edge, to its meeting the
    // ball
    let (from, edge, to) = (-(rod / (big + d)).acos(), -(rod / big).acos(), -((rod + d) / big).acos());
    let neck = crate::brep::simpson(from, edge, 4000, |phi| moment(phi, rod / phi.cos()))
        + crate::brep::simpson(edge, to, 4000, |phi| moment(phi, big));
    close(props::volume(&b), unrounded + root + 2.0 * PI * neck);
    assert_eq!(kinds(&b).get("cone"), Some(&1), "{:?}", kinds(&b));
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

/// **A run mitred against a ring's stretch on one cylinder** (#218's 1): the groove's rims rounded
/// all round — along the plate's top (two runs) and at its ends (two half rings). At each corner a
/// run and a stretch meet, each with a face on the groove's cylinder; their union is found (the
/// faces they share on one cylinder, the line where a run's round touches the groove taken once)
/// up to where the two rounds meet, which ends where both touch the groove — three surfaces
/// tangent at one point, which this kernel's trace does not follow (#58): refused, naming that.
#[test]
fn a_grooves_rims_rounded_all_round_are_refused_where_three_surfaces_touch() {
    let src = include_str!("../../examples/rounds/rail.sv").replace("rod union rail", "rod cut rail");
    let e = solved(&src);
    assert_eq!(lowered(&e).0, 3);
    let say = Say { stage: &|_: &str| {}, mark: &|_| {} };
    let refusal = export::exact(&e.sketch, ent(&e, "rail").i(), None, None, &say).err().expect("refused");
    assert!(format!("{refusal:?}").contains("a cylinder and a torus meet at 0.00°")
        && format!("{refusal:?}").contains("nearly touching: their meeting is ill-conditioned there"), "{refusal:?}");
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

/// Solid `name`'s exact B-rep as the drawing reads it (`Sketch::exact_solid`), held to its own
/// field: every probe `offset` inside its boundary material by the field, every one `offset`
/// outside not (`agreement`, by area); and its volume (model units).
fn agrees_with_field(e: &Elaborated, name: &str, offset: f64) -> f64 {
    let body = ent(e, name).i();
    let exact = e.sketch.exact_solid(body).unwrap_or_else(|why| panic!("{name}: {why}"));
    let b = &exact.brep;
    let offset = offset * exact.mm;
    let m = gcs_core::brep::mesh::mesh(b, offset / 20.0, gcs_core::brep::export::ANGULAR).unwrap();
    let pts: Vec<[f64; 3]> = m.pts.iter().map(|&q| std::array::from_fn(|k| q[k] / exact.mm + exact.origin[k])).collect();
    let field = MaterialField::read(&e.sketch, body, 1e-10).unwrap();
    let offset = offset / exact.mm;
    let options = gcs_core::solid::agreement::Options {
        offset, confirm: offset / 4.0, value_tolerance: offset / 5.0, triangles: 2000, ..Default::default()
    };
    let report = gcs_core::solid::agreement::of_triangles_parallel(&pts, &m.tris, &field, 1, &options).unwrap();
    assert!(report.agrees(), "{name}: {} of {} probes disagree: {:?}", report.disagreements.len(), report.probes,
        report.disagreements);
    props::volume(b) / exact.mm.powi(3)
}

/// The export's refusal of solid `name`, as text.
fn refused(e: &Elaborated, name: &str) -> String {
    let say = Say { stage: &|_: &str| {}, mark: &|_| {} };
    format!("{:?}", export::exact(&e.sketch, ent(e, name).i(), None, None, &say).err().expect("refused"))
}

/// **A corner of a face in pieces** (#218's 4): `post.sv`'s stand rounded all over. The post
/// splits the plate's front side in two, so a corner where the plate's top, its front and an end
/// meet takes the piece of the front standing at that vertex. Where the plate's convex top edge
/// runs into the post's concave root, the rounds — each read off the stand before the others —
/// leave material touching itself along a line (#218, from #197): built as a boundary holding
/// that line twice, agreeing with the field, and refused by the export, which no manifold file
/// describes.
#[test]
fn a_corner_of_a_face_in_pieces_is_rounded() {
    let src = include_str!("../../examples/rounds/post.sv")
        .replace("stand.plate.near round(r: r) stand.plate.ab", "round(r: 1mm) stand");
    let e = solved(&src);
    lowered(&e);
    assert!(agrees_with_field(&e, "stand", 0.01) > 0.0);
    let why = refused(&e, "stand");
    assert!(why.contains("touches itself along a line") && why.contains("no manifold file describes it"), "{why}");
}

/// **A vertex where convex and concave rounds meet** (#218): `twin.sv`'s bar rounded all over.
/// Where the seam between the rods (concave, filled) meets a cap whose rims are convex (taken),
/// each round is read off the bar before the others, so the seam's fill and the rims' cuts leave
/// material touching itself along a short line at each end of the seam: built, the line held
/// twice, agreeing with the field, and refused by the export, which no manifold file describes.
#[test]
fn rounds_meeting_at_a_vertex_of_both_kinds_touch_along_a_line() {
    let src = include_str!("../../examples/rounds/twin.sv")
        .replace("bar.right.small round(r: r) bar.left.big", "round(r: 1mm) bar");
    let e = solved(&src);
    lowered(&e);
    assert!(agrees_with_field(&e, "bar", 0.01) > 0.0);
    let why = refused(&e, "bar");
    assert!(why.contains("touches itself along a line") && why.contains("no manifold file describes it"), "{why}");
}

/// Two overlapping bores through a round slab 40 across and 10 deep.
const BORES: &str = "unit mm\nuse std (round)\nin std.front {\n\
    o := point\nfix((5, 0)) o\nrim := circle(center: o) hint(r: 20)\nradius(20mm) rim\n\
    c1 := point\nc2 := point\nfix((0, 0)) c1\nfix((14, 0)) c2\n\
    big := circle(center: c1) hint(r: 10)\nsmall := circle(center: c2) hint(r: 8)\n\
    radius(10mm) big\nradius(8mm) small\n}\n\
    block := solid(face(rim), from: 0mm, to: 10mm)\n\
    left := solid(face(big), from: 0mm, to: 10mm)\nright := solid(face(small), from: 0mm, to: 10mm)\n\
    left cut block\nright cut block\n";

/// **A corner where two curved faces meet** (#218's 4): the two bores' slab rounded all over.
/// Where the cusp between the bores reaches a face of the block, three convex edges meet — the
/// cusp (a line between two cylinders) and each bore's rim (a circle where a plane meets a
/// cylinder) — and the ball touching the face and both walls rounds the corner: its cell over a
/// plane and two cylinders, the cusp's run and each rim's stretch of a ring cut short at its
/// sections there and glued (a ring's stretch to the corner at its own end of it, never to one
/// standing on another stretch of the same circle). Built, exported, and held to the field.
#[test]
fn a_corner_where_two_bores_meet_a_face_is_rounded() {
    let e = solved(&format!("{BORES}round(r: 1mm) block\n"));
    lowered(&e);
    let b = exported(&e, "block");
    let k = kinds(&b);
    assert!(k.get("sphere").is_some_and(|&n| n >= 4) && k.contains_key("torus"), "{k:?}");
    agrees_with_field(&e, "block", 0.01);
}
