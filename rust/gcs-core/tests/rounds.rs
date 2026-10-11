//! **Rounds and chamfers written in Solvent** (#183, step 6): `round` and `chamfer` are words of
//! `rust/lib/std.sv` over the language's own features — faces as sets, the distance from a set,
//! a bound on measurements, where an edge ends, the material's angle, repetition over faces and
//! words that make solids — evaluated by the field, and held to X2's closed forms.
use gcs_core::program::Elaborated;
use gcs_core::solid::MaterialField;
use gcs_core::solve::{solve, SolveOpts};
use std::f64::consts::PI;

use crate::common::{ent, read};
use crate::region_general::field_volume;

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

/// The volume a statement takes from (or gives to) solid `name`, within a box: the field's before
/// less after.
fn change(stmt: &str, name: &str, lo: [f64; 3], hi: [f64; 3], h: f64) -> f64 {
    let before = solved(BLOCK);
    let after = solved(&format!("{BLOCK}{stmt}\n"));
    let volume = |e: &Elaborated| {
        field_volume(&MaterialField::read(&e.sketch, ent(e, name).i(), 1e-10).unwrap(), lo, hi, h)
    };
    volume(&before) - volume(&after)
}

/// The stack's mitre: a convex polygon's prism with one cap's rim rounded alone loses `∫δ` along
/// its perimeter less `∫δ²` at each corner (`cots` the sum of `cot(θ/2)` over its angles).
fn mitred(perimeter: f64, cots: f64, r: f64) -> f64 {
    (1.0 - PI / 4.0) * r * r * perimeter - (5.0 / 3.0 - PI / 2.0) * r * r * r * cots
}

/// X2's lid: the near face's rim rounded alone, the other edges sharp — runs crossing at each
/// corner in a mitre.
#[test]
fn a_rim_rounded_alone_is_mitred() {
    let r = 2.0;
    let got = change("block.near round(r: 2mm) block", "block", [-1.0, -3.0, -1.0], [41.0, 3.0, 25.0], r / 32.0);
    let want = mitred(2.0 * (40.0 + 24.0), 4.0, r);
    assert!((got - want).abs() < 2e-3 * want, "{got} against {want}");
}

/// A box rounded on every edge, its corners spherical (Steiner).
fn rounded_box([l, w, h]: [f64; 3], r: f64) -> f64 {
    let (a, b, c) = (l - 2.0 * r, w - 2.0 * r, h - 2.0 * r);
    a * b * c + 2.0 * r * (a * b + b * c + a * c) + PI * r * r * (a + b + c)
        + 4.0 * PI * r * r * r / 3.0
}

/// X2's block rounded all over: twelve runs, and a corner term where three meet.
#[test]
#[cfg_attr(not(feature = "slow"), ignore = "slow tier: every edge of a block, by the field")]
fn a_block_rounded_all_over_is_steiners() {
    let r = 2.0;
    let got = change("round(r: 2mm) block", "block", [-1.0, -13.0, -1.0], [41.0, 13.0, 25.0], r / 16.0);
    let want = 40.0 * 24.0 * 12.0 - rounded_box([40.0, 24.0, 12.0], r);
    assert!((got - want).abs() < 2e-3 * want, "{got} against {want}");
}

/// A tray 60 × 40 × 10 with a pocket 30 × 20 cut 6 into its near face, and a plate the same with
/// a boss of radius 8 standing 20 on it — each a body over its operands' faces.
const PARTS: &str = "unit mm\nuse std (round, chamfer)\nin std.front {\n\
    t0 := point\nt1 := point\nt2 := point\nt3 := point\n\
    fix((0, 0)) t0\nfix((60, 0)) t1\nfix((60, 40)) t2\nfix((0, 40)) t3\n\
    (ts := line(t0, t1)) -> (te := line(t1, t2)) -> (tn := line(t2, t3)) -> (tw := line(t3, t0)) -> close\n\
    outline := face(ts, te, tn, tw)\n\
    k0 := point\nk1 := point\nk2 := point\nk3 := point\n\
    fix((15, 10)) k0\nfix((45, 10)) k1\nfix((45, 30)) k2\nfix((15, 30)) k3\n\
    (ks := line(k0, k1)) -> (ke := line(k1, k2)) -> (kn := line(k2, k3)) -> (kw := line(k3, k0)) -> close\n\
    opening := face(ks, ke, kn, kw)\n\
    c := point\nfix((30, 20)) c\nrim := circle(center: c) hint(r: 8)\nradius(8mm) rim\n}\n\
    tray := solid(outline, depth: 10mm)\npocket := solid(opening, from: -6mm, to: 1mm)\n\
    dish := solid(tray)\npocket cut dish\n\
    plate := solid(outline, depth: 10mm)\nboss := solid(face(rim), from: 0mm, to: 20mm)\n\
    pad := solid(plate)\nboss union pad\n";

fn part_change(stmt: &str, name: &str, lo: [f64; 3], hi: [f64; 3], h: f64) -> f64 {
    let before = solved(PARTS);
    let after = solved(&format!("{PARTS}{stmt}\n"));
    let volume = |e: &Elaborated| {
        field_volume(&MaterialField::read(&e.sketch, ent(e, name).i(), 1e-10).unwrap(), lo, hi, h)
    };
    volume(&before) - volume(&after)
}

/// X2's pocket rounded inside: every edge among the pocket's faces is concave, filled, and its
/// corners are each a ball's patch filled too.
#[test]
#[cfg_attr(not(feature = "slow"), ignore = "slow tier: a pocket rounded inside, by the field")]
fn a_pocket_rounded_inside_is_filled() {
    let pr = 1.5;
    let run = (1.0 - PI / 4.0) * pr * pr;
    let want = 4.0 * run * (6.0 - pr) + run * (2.0 * (30.0 - 2.0 * pr) + 2.0 * (20.0 - 2.0 * pr))
        + 4.0 * (pr * pr * pr - PI * pr * pr * pr / 6.0);
    let got = -part_change("round(r: 1.5mm) dish.pocket", "dish", [14.0, -1.0, 9.0], [46.0, 7.0, 31.0], pr / 16.0);
    assert!((got - want).abs() < 2e-3 * want, "{got} against {want}");
}

/// The stack's ring a ball of radius `r` fills where a cylinder of radius `big` stands on a plane
/// (Pappus).
fn root_ring(big: f64, r: f64) -> f64 {
    let moment = r * r * (big + r / 2.0) - PI * r * r / 4.0 * (big + r - 4.0 * r / (3.0 * PI));
    2.0 * PI * moment
}

/// A boss's root, where its wall meets the plate: a concave circle, filled by a torus's ring.
#[test]
#[cfg_attr(not(feature = "slow"), ignore = "slow tier: a boss's root, by the field")]
fn a_bosss_root_is_a_ring() {
    let r = 3.0;
    let got = -part_change("pad.boss round(r: 3mm) pad.plate", "pad", [17.0, -5.0, 7.0], [43.0, 1.0, 33.0], r / 32.0);
    let want = root_ring(8.0, r);
    assert!((got - want).abs() < 2e-3 * want, "{got} against {want}");
}

/// X3's block chamfered all over: three flats meet at each corner, `2d²(l + w + h) − 6d³`.
#[test]
#[cfg_attr(not(feature = "slow"), ignore = "slow tier: every edge of a block chamfered, by the field")]
fn a_block_chamfered_all_over() {
    let d = 1.0;
    let got = change("chamfer(d: 1mm) block", "block", [-1.0, -13.0, -1.0], [41.0, 13.0, 25.0], d / 16.0);
    let want = 2.0 * d * d * (40.0 + 24.0 + 12.0) - 6.0 * d * d * d;
    assert!((got - want).abs() < 2e-3 * want, "{got} against {want}");
}

/// A disc of radius 10 and 4 thick about z, as a region: within a cylinder, and within 2 of the
/// plane through its middle each way.
const DISC: &str = "unit mm\nuse std\n\
    c := point hint((0, 0, 0))\nfix((0, 0, 0)) c\nze := point hint((0, 0, 1))\nfix((0, 0, 1)) ze\n\
    zl := line(c, ze)\n\
    tx := axis hint(dir: (1, 0, 0))\nty := axis hint(dir: (0, 1, 0))\n\
    fix(dir == (1, 0, 0), origin == (0, 0, 2)) tx\nfix(dir == (0, 1, 0), origin == (0, 0, 2)) ty\n\
    mid := plane(u: tx, v: ty)\n\
    wall := std.Cylinder(zl, r: 10)\n\
    top := { q | q distance(<= 2mm, along: n) mid }\nbottom := { q | q distance(>= -2mm, along: n) mid }\n";

/// **In a region's body** (#183 X3): a round or a chamfer at the disc's top rim is one more bound
/// on its point — the ring a ball of 2 cannot reach (Pappus: the corner less the quarter disc,
/// its centroid `r/(6(1 − π/4))` out from the ball's centre, `2π((1 − π/4)r²(R − r) + r³/6)`), and the
/// triangle a flat 1 each way takes (`π(R − d/3)d²`).
#[test]
fn a_regions_rim_rounded_and_chamfered_in_its_body() {
    let e = solved(&format!("{DISC}\
        disc := {{ p | p inside wall; p inside top; p inside bottom }}\n\
        rounded := {{ p | p inside wall; p inside top; p inside bottom; p inside std.Rounded(wall, top, r: 2mm) }}\n\
        chamfered := {{ p | p inside wall; p inside top; p inside bottom; \
                          p inside std.Chamfered(wall, top, la: 1mm, lb: 1mm) }}\n\
        sd := solid(disc)\nsr := solid(rounded)\nsc := solid(chamfered)\n"));
    let volume = |name: &str| {
        let f = MaterialField::read(&e.sketch, ent(&e, name).i(), 1e-10).unwrap();
        field_volume(&f, [-11.0, -11.0, -1.0], [11.0, 11.0, 5.0], 1.0 / 16.0)
    };
    let disc = volume("sd");
    assert!((disc - PI * 400.0).abs() < 1e-3 * PI * 400.0, "{disc}");
    let (big, r, d) = (10.0, 2.0, 1.0);
    let ring = 2.0 * PI * ((1.0 - PI / 4.0) * r * r * (big - r) + r * r * r / 6.0);
    let flat = PI * (big - d / 3.0) * d * d;
    for (name, want) in [("sr", ring), ("sc", flat)] {
        let got = disc - volume(name);
        assert!((got - want).abs() < 3e-3 * want, "{name}: {got} against {want}");
    }
}


/// **Lowered** (#183 step 7): the disc's round and chamfer, each at the corner of two of its own
/// bounds, are built in its meridian (`brep::corners`, X1) — its exact solid's volume the closed
/// form to 1e-9 — and the field of the meridian agrees with the region read point by point.
#[test]
fn a_regions_round_and_chamfer_lower_into_its_meridian() {
    let e = solved(&format!("{DISC}\
        rounded := {{ p | p inside wall; p inside top; p inside bottom; p inside std.Rounded(wall, top, r: 2mm) }}\n\
        chamfered := {{ p | p inside wall; p inside top; p inside bottom; \
                          p inside std.Chamfered(wall, top, la: 1mm, lb: 1mm) }}\n\
        sr := solid(rounded)\nsc := solid(chamfered)\n"));
    let (big, r, d) = (10.0, 2.0, 1.0);
    let disc = PI * big * big * 4.0;
    let ring = 2.0 * PI * ((1.0 - PI / 4.0) * r * r * (big - r) + r * r * r / 6.0);
    let flat = PI * (big - d / 3.0) * d * d;
    for (name, want) in [("sr", disc - ring), ("sc", disc - flat)] {
        let i = ent(&e, name).i();
        assert!(gcs_core::solid::region::meridian(&e.sketch, i).is_ok(), "{name}");
        assert!(!gcs_core::solid::region::in_general(&e.sketch, i), "{name}");
        let ex = e.sketch.exact_solid(i).unwrap();
        let got = gcs_core::brep::props::volume(&ex.brep) / ex.mm.powi(3);
        assert!((got - want).abs() <= 1e-9 * want, "{name}: {got} against {want}");
        // the lowering builds what the set says: the meridian's field and the point-by-point
        // reading agree wherever a point is not on the boundary
        let lowered = MaterialField::read(&e.sketch, i, 1e-10).unwrap();
        let general = gcs_core::solid::region::general(&e.sketch, i).unwrap();
        let mut k = 0x2545f4914f6cdd1du64;
        let mut next = || { k ^= k << 13; k ^= k >> 7; k ^= k << 17; (k >> 11) as f64 / (1u64 << 53) as f64 };
        for _ in 0..4000 {
            // the rim's neighbourhood, where the two differ if anywhere
            let (phi, rho, z) = (next() * 2.0 * PI, 7.0 + 4.0 * next(), 1.0 + 4.0 * next());
            let p = [rho * phi.cos(), rho * phi.sin(), z];
            let g = general.value(p);
            if g.abs() > 1e-6 {
                assert_eq!(lowered.side(p) < 0.0, g < 0.0, "{name} at {p:?}: {g}");
            }
        }
    }
}
