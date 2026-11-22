//! **A band that tapers to nothing on a plate, its root filleted and its own edges rounded** (#994
//! item 2d, fixture 142).  A band stands on a plate's face, unioned onto it, and thins to nothing
//! at its ends: its top comes down onto the plate.  The fill where the band's side meets the plate
//! runs along the side's foot to where the top closes down onto it, cut off there by the top (kept
//! outside its surface, `crease.ends`): the fillet's section is clipped by the top progressively
//! until nothing of it is left, and where the top runs on tangent into the plate the crease's own
//! line only touches it, so the run ends square at the toe (`run_piece`'s `halfway`, the chord
//! nothing).  The plate's smooth chain goes on tangent into the band's top, and its reading at the
//! chain's rim was wrong (`BodyFaces::nearest_face`): beside the band, past the rim where its top
//! meets its side, the top's side was read, within, where the plate (or the disc) the side rises
//! from is the surface; and in the side's own plane, where the gate probes a round's leg, the
//! nearest face's — read now as the region holds its point, just past the side or just within.
//! The band's own top edge rounded ends at each toe on the plate and is kept over it (`crease.ends`,
//! `BodyFaces::ends_at`), where it was cut on square into the plate under the toe.
//!
//! The family, each drawn whole at DOF 0, exported (STEP and STL, read back) with no face thinner
//! than the export's tolerance, and its volume the closed form's to 1e-9: a band whose top runs on
//! tangent into the plate (concave arcs off the plate joined by a convex one) in three
//! proportions, filleted 1, 3 and 8 (its height 11 to 14) on one side and on both; the same band
//! on a disc, filleted under and over its height (a ring about the disc's axis, the fill standing
//! beside the band past its top where the radius is the larger); a band whose top meets the plate
//! at an angle, a circular segment and a wedge, under and over their heights; and the band's own
//! top edge rounded, on the plate and on the disc.  On the disc a round over what the band's
//! thickness at its seams allows is refused, by its faces and what is true to do.
use gcs_core::brep::{export, props, sweep::Say};
use std::f64::consts::PI;

use crate::common::rounds::gauss_legendre;
use crate::common::{close, ent, text::{dof, solved}};

const HEAD: &str = "unit mm\nuse std\nuse std.solids\nuse std.solids (union)\nuse std.rounds (round)\n";

/// The plate `[-50, 50] × [-30, 30] × [0, 10]` and the band on its top, the outline `section` in
/// the front plane (`x`, `z`) carried from `y = −10` to `10`; `word` after.
fn on_plate(section: &str, word: &str) -> String {
    format!("{HEAD}in std.top {{\n  a := point\n  fix((-50, -30)) a\n  b := point\n  fix((50, -30)) b\n  \
        c := point\n  fix((50, 30)) c\n  d := point\n  fix((-50, 30)) d\n  j0 := line(a, b)\n  j1 := line(b, c)\n  \
        j2 := line(c, d)\n  j3 := line(d, a)\n  blk := face(j0, j1, j2, j3)\n}}\nin std.front {{\n{section}}}\n\
        plate := solid(std.solids.Prism(blk, from: 0mm, to: 10mm))\n\
        band := solid(std.solids.Prism(sg, from: -10mm, to: 10mm))\npart := solid(plate)\nband union part\n{word}\n")
}

/// Points named and fixed where they stand in the front plane.
fn points(ps: &[(&str, [f64; 2])]) -> String {
    ps.iter().map(|(n, [x, z])| format!("  {n} := point\n  fix(({x:.17}, {z:.17})) {n}\n")).collect()
}

/// The tapered band: from its toes at `(±30, 10)` concave arcs of `rho` tangent to the plate, about
/// `(±30, 10 + rho)`, run up into a convex arc of `big` about `(0, z0)` tangent to both.  Its
/// numbers: `z0`, the tangent points' `x` (`±tx`) and `z`.
fn lens_numbers(rho: f64, big: f64) -> (f64, f64, f64) {
    let z0 = 10.0 + rho - ((rho + big).powi(2) - 900.0).sqrt();
    let k = rho / (rho + big);
    (z0, 30.0 - 30.0 * k, 10.0 + rho + k * (z0 - 10.0 - rho))
}

fn lens(rho: f64, big: f64, word: &str) -> String {
    let (z0, tx, tz) = lens_numbers(rho, big);
    let ps = points(&[("p", [-30.0, 10.0]), ("q", [30.0, 10.0]), ("cl", [-30.0, 10.0 + rho]), ("cr", [30.0, 10.0 + rho]),
        ("ct", [0.0, z0]), ("t1", [-tx, tz]), ("t2", [tx, tz])]);
    on_plate(&format!("{ps}  k0 := line(p, q)\n  a1 := arc(center: cl, start: p, end: t1) hint(r: {rho})\n  \
        a2 := arc(center: ct, start: t2, end: t1) hint(r: {big})\n  a3 := arc(center: cr, start: t2, end: q) hint(r: {rho})\n  \
        sg := face(k0, a1, a2, a3)\n"), word)
}

/// The tapered band's section's area: under the convex arc and the two concave ones, over the plate.
fn lens_area(rho: f64, big: f64) -> f64 {
    let (z0, tx, _) = lens_numbers(rho, big);
    // `∫₀ˢ √(a² − u²) du`
    let f = |s: f64, a: f64| 0.5 * (s * (a * a - s * s).sqrt() + a * a * (s / a).asin());
    let foot = 30.0 - tx;
    2.0 * (rho * foot - f(foot, rho)) + 2.0 * ((z0 - 10.0) * tx + f(tx, big))
}

/// A circular segment over the plate, its chord from `(−30, 10)` to `(30, 10)` and its height `h`.
fn segment(h: f64, word: &str) -> String {
    let big = (900.0 + h * h) / (2.0 * h);
    let ps = points(&[("p", [-30.0, 10.0]), ("q", [30.0, 10.0]), ("o", [0.0, 10.0 + h - big])]);
    on_plate(&format!("{ps}  k0 := line(p, q)\n  k1 := arc(center: o, start: q, end: p) hint(r: {big})\n  \
        sg := face(k0, k1)\n"), word)
}

/// A wedge over the plate, rising from nothing at `x = −30` to `h` at `x = 30`.
fn wedge(h: f64, word: &str) -> String {
    let ps = points(&[("p", [-30.0, 10.0]), ("q", [30.0, 10.0]), ("s", [30.0, 10.0 + h])]);
    on_plate(&format!("{ps}  k0 := line(p, q)\n  k1 := line(q, s)\n  k2 := line(s, p)\n  sg := face(k0, k1, k2)\n"), word)
}

/// The tapered band on a disc of 30 about the `y` axis, `y` in `[−20, 20]`: from its toes at `90° ±
/// alpha` concave arcs of `rho` tangent to the disc run up into a convex arc about `(0, z0)`.  Its
/// numbers: the left toe, the concave arc's centre, the tangent point and the convex arc's radius.
fn disc_numbers(alpha: f64, rho: f64, z0: f64) -> ([f64; 2], [f64; 2], [f64; 2], f64) {
    let a = (90.0 + alpha).to_radians();
    let toe = [30.0 * a.cos(), 30.0 * a.sin()];
    let ca = [(30.0 + rho) * a.cos(), (30.0 + rho) * a.sin()];
    let d = (ca[0] * ca[0] + (ca[1] - z0).powi(2)).sqrt();
    let t = [ca[0] - rho * ca[0] / d, ca[1] + rho * (z0 - ca[1]) / d];
    (toe, ca, t, d - rho)
}

fn disc_band(alpha: f64, rho: f64, z0: f64, word: &str) -> String {
    let (toe, ca, t, big) = disc_numbers(alpha, rho, z0);
    let m = |[x, z]: [f64; 2]| [-x, z];
    let ps = points(&[("o", [0.0, 0.0]), ("ta", toe), ("tb", m(toe)), ("ca", ca), ("cb", m(ca)), ("ct", [0.0, z0]),
        ("t1", t), ("t2", m(t))]);
    format!("{HEAD}in std.front {{\n{ps}  eo := circle(center: o) hint(r: 30)\n  fix(r == 30mm) eo\n  \
        a1 := arc(center: ca, start: ta, end: t1) hint(r: {rho})\n  a2 := arc(center: ct, start: t2, end: t1) hint(r: {big})\n  \
        a3 := arc(center: cb, start: t2, end: tb) hint(r: {rho})\n  k := line(ta, tb)\n  sg := face(k, a3, a2, a1)\n}}\n\
        disc := solid(std.solids.Prism(face(eo), from: -20mm, to: 20mm))\n\
        band := solid(std.solids.Prism(sg, from: -10mm, to: 10mm))\npart := solid(disc)\nband union part\n{word}\n")
}

/// `∫₀^top g(t) dt` by Gauss–Legendre in `φ`, `t = top (1 − cos φ) / 2`: a root at either end of
/// the interval (`√t` at the floor, `√(top − t)` where a top closes) made smooth.
fn over(top: f64, g: impl Fn(f64) -> f64) -> f64 {
    gauss_legendre(80).into_iter().map(|(x, w)| {
        let phi = 0.5 * PI * (1.0 + x);
        0.5 * PI * w * g(0.5 * top * (1.0 - phi.cos())) * 0.5 * top * phi.sin()
    }).sum()
}

/// A round's section's width at `t` from one face, measured along the other: `r − √(2rt − t²)`.
fn width(r: f64, t: f64) -> f64 { r - (2.0 * r * t - t * t).max(0.0).sqrt() }

/// A fill of radius `r` at the foot of a band's side on a plate: each line of its section `t` over
/// the plate as long as `along(t)` (nothing where that is not positive), up to `top`.
fn fill(r: f64, top: f64, along: impl Fn(f64) -> f64) -> f64 {
    over(r.min(top), |t| width(r, t) * along(t).max(0.0))
}

/// The plate unrounded with a band of section `area` on it.
fn bare(area: f64) -> f64 { 60000.0 + 20.0 * area }

/// The tapered band's foot filled `r`: the line `t` over the plate kept outside both concave arcs'
/// cylinders, `60 − 2√(2ρt − t²)` long, square at the toes where the crease only touches them.
fn lens_fill(rho: f64, r: f64) -> f64 { fill(r, r, |t| 60.0 - 2.0 * (2.0 * rho * t - t * t).max(0.0).sqrt()) }

/// The tapered band's top edge along one side rounded `r`: the section's line `t` in from the top,
/// along the top's offset by `t` — the concave arcs' at `ρ + t`, the convex one's at `R − t` —
/// kept over the plate: on each concave arc's offset from the crown down to where it meets the
/// plate, `acos(ρ / (ρ + t))` short of the toe's turn.
fn lens_round(rho: f64, big: f64, r: f64) -> f64 {
    let (z0, tx, tz) = lens_numbers(rho, big);
    let (foot, crown) = (((30.0 - tx) / rho).asin(), 2.0 * tx.atan2(tz - z0));
    over(r, |t| width(r, t) * (2.0 * (rho + t) * (foot - (rho / (rho + t)).acos()).max(0.0) + (big - t) * crown))
}

/// The wedge's slope edge along one side rounded `r`: the section's line `t` in from the slope,
/// from where it meets the plate to the wedge's end `x = 30` (none deeper than the slope's drop
/// along its length, `60 sin α`).
fn wedge_round(h: f64, r: f64) -> f64 {
    let a = h.atan2(60.0);
    over(r.min(60.0 * a.sin()), |t| width(r, t) * (60.0 - t / a.sin()) / a.cos())
}

/// The segment's arc edge along one side rounded `r`: the ring's section, its line `t` in from the
/// arc at `q = R − t` from the axis, turned over the plate, `2 acos(c / q)` (`c` the axis's depth
/// under the plate's face; none deeper than the segment's height).
fn segment_round(h: f64, r: f64) -> f64 {
    let big = (900.0 + h * h) / (2.0 * h);
    let c = big - h;
    over(r.min(h), |t| { let q = big - t; width(r, t) * 2.0 * q * (c / q).min(1.0).acos() })
}

/// The disc's band's foot filled `r`: the ring's section, its line `t` out from the disc turned
/// from toe to toe less where it stands within either concave arc's cylinder.
fn disc_fill(alpha: f64, rho: f64, r: f64) -> f64 {
    let spread = (2.0 * alpha).to_radians();
    let d = 30.0 + rho;
    over(r, |t| {
        let q = 30.0 + t;
        let within = ((q * q + d * d - rho * rho) / (2.0 * q * d)).min(1.0).acos();
        width(r, t) * q * (spread - 2.0 * within).max(0.0)
    })
}

/// The disc and its band unrounded: the band's section less the disc's segment under its chord.
fn disc_bare(alpha: f64, rho: f64, z0: f64) -> f64 {
    let (toe, _, t, big) = disc_numbers(alpha, rho, z0);
    // the polygon of the toes and the tangent points, less each concave arc's segment (bowing into
    // it) and with the convex one's (bowing out)
    let seg = |rad: f64, chord: f64| { let th = 2.0 * (0.5 * chord / rad).asin(); 0.5 * rad * rad * (th - th.sin()) };
    let dist = |a: [f64; 2], b: [f64; 2]| ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2)).sqrt();
    let poly = [toe, t, [-t[0], t[1]], [-toe[0], toe[1]]];
    let shoelace: f64 = 0.5 * (0..4).map(|i| { let (a, b) = (poly[i], poly[(i + 1) % 4]); a[0] * b[1] - b[0] * a[1] })
        .sum::<f64>().abs();
    // the concave arcs bow into the polygon, the convex one out of it
    let band = shoelace - 2.0 * seg(rho, dist(toe, t)) + seg(big, dist(t, [-t[0], t[1]]));
    let under = seg(30.0, 2.0 * toe[0].abs());
    PI * 900.0 * 40.0 + 20.0 * (band - under)
}

/// The disc's band's top edge along one side rounded `r`, short of the seams: the crown's run
/// whole, its section's line `t` in along the crown's offset at `R − t`; each foot arc's along its
/// offset at `ρ + t` about its centre `C`, from the seam (the way from `C` to the crown's centre)
/// to where it meets the disc's circle — turned back from the toe (the way from `C` to the disc's
/// centre) by `acos(((ρ + t)² + |C|² − 30²) / (2 (ρ + t) |C|))`.
fn disc_round(alpha: f64, rho: f64, z0: f64, r: f64) -> f64 {
    let (_, ca, t1, big) = disc_numbers(alpha, rho, z0);
    let crown = 2.0 * (-t1[0]).atan2(t1[1] - z0);
    let reach = (ca[0] * ca[0] + ca[1] * ca[1]).sqrt();
    // the foot arc's whole turn: between the ways from its centre to the disc's and the crown's
    let way = |p: [f64; 2]| [p[0] - ca[0], p[1] - ca[1]];
    let (u, v) = (way([0.0, 0.0]), way([0.0, z0]));
    let foot = ((u[0] * v[0] + u[1] * v[1]) / (reach * (v[0] * v[0] + v[1] * v[1]).sqrt())).acos();
    over(r, |t| {
        let q = rho + t;
        let back = ((q * q + reach * reach - 900.0) / (2.0 * q * reach)).min(1.0).acos();
        width(r, t) * ((big - t) * crown + 2.0 * q * (foot - back))
    })
}

/// Solid `name` of `src` built exactly, exported as STEP and STL (each read back), no face of it
/// thinner than the export's tolerance; its volume.
fn built(src: &str, name: &str) -> f64 {
    assert_eq!(dof(src), 0, "{src}");
    let e = solved(src);
    let say = Say { stage: &|_: &str| {}, mark: &|_| {} };
    let body = ent(&e, name).i();
    let exact = export::exact(&e.sketch, body, None, None, &say).unwrap_or_else(|r| panic!("{name}: {r:?}\n{src}"));
    if let Some(s) = export::slivers(&e.sketch, body, &exact, None) {
        panic!("{}\n{src}", s.message);
    }
    export::step(&exact, name, None, &say).unwrap();
    let stl = export::stl(&e.sketch, body, &exact, None, &say).unwrap();
    gcs_core::mesh::stl_shells(&stl).unwrap();
    props::volume(&exact.solid)
}

/// **The fixture's root fillet** where the band's top runs on tangent into the plate (#994: "its
/// exact construction leaves out what the field reads beside its edge" at the taper, and before
/// that "a run its curved end does not cut off"): three proportions of foot and crown, filleted
/// under and over the band's height (11 to 14).
#[test]
fn a_root_fillet_runs_out_where_a_band_tapers_tangent_into_its_plate() {
    for (rho, big) in [(20.0, 20.0), (40.0, 10.0), (10.0, 30.0)] {
        for r in [1.0, 3.0, 8.0] {
            let src = lens(rho, big, &format!("part.plate.near round(r: {r}mm) part.band.near"));
            close(built(&src, "part"), bare(lens_area(rho, big)) + lens_fill(rho, r));
        }
    }
}

/// Both sides' feet filled, each its own run to the toes.
#[test]
fn a_tapered_bands_feet_are_filled_on_both_sides() {
    for r in [1.0, 8.0] {
        let src = lens(20.0, 20.0, &format!("part.plate.near round(r: {r}mm) part.band"));
        close(built(&src, "part"), bare(lens_area(20.0, 20.0)) + 2.0 * lens_fill(20.0, r));
    }
}

/// **The fixture's own setting**: the band on a curved face, a disc, its foot a ring about the
/// disc's axis cut off by the concave arcs it tapers into, filleted under and over its height
/// (4.9): over it the fill stands beside the band past its top, the side's plane read as the set
/// says — the round's ball against the side's whole plane.
#[test]
fn a_root_fillet_runs_out_where_a_band_tapers_tangent_into_a_disc() {
    for r in [2.0, 3.0, 8.0] {
        let src = disc_band(40.0, 15.0, 15.0, &format!("part.disc.eo round(r: {r}mm) part.band.near"));
        close(built(&src, "part"), disc_bare(40.0, 15.0, 15.0) + disc_fill(40.0, 15.0, r));
    }
}

/// A band whose top meets the plate at an angle — a circular segment, its fill kept within the
/// arc's cylinder, and a wedge, kept below its slope — under and over its height: what built
/// before, held to its closed form.
#[test]
fn a_root_fillet_ends_where_a_band_meets_its_plate_at_an_angle() {
    for h in [2.0f64, 8.0] {
        let big = (900.0 + h * h) / (2.0 * h);
        let c = big - h;
        let area = big * big * (30.0 / big).asin() - 30.0 * c;
        for r in [1.0, 3.0, 10.0] {
            let src = segment(h, &format!("part.plate.near round(r: {r}mm) part.band.near"));
            let along = |t: f64| 2.0 * (big * big - (c + t).powi(2)).max(0.0).sqrt();
            close(built(&src, "part"), bare(area) + fill(r, h, along));
        }
    }
    for h in [2.0, 6.0] {
        for r in [1.0, 3.0, 8.0] {
            let src = wedge(h, &format!("part.plate.near round(r: {r}mm) part.band.near"));
            close(built(&src, "part"), bare(30.0 * h) + fill(r, h, |t| 60.0 * (1.0 - t / h)));
        }
    }
}

/// **The band's own edges rounded after the union** (#994: "fails the same way"; the crown's run
/// over 8 disagreed with the field): along its top, from toe to toe, the chain's offset swept, cut
/// off at the plate each toe ends on and kept over it (`crease.ends`) — the plate is no plane
/// turning back over the run's faces, which stand wholly on its outside, so it bounds the run
/// (`BodyFaces::ends_at`); ended square at the toe, the run cut a groove into the plate under it.
#[test]
fn a_tapered_bands_top_edge_is_rounded_to_its_toes_and_no_further() {
    for (rho, big, r) in [(20.0, 20.0, 1.0), (20.0, 20.0, 3.0), (20.0, 20.0, 8.0), (10.0, 30.0, 2.0), (40.0, 10.0, 2.0)] {
        let src = lens(rho, big, &format!("part.band.near round(r: {r}mm) part.band"));
        close(built(&src, "part"), bare(lens_area(rho, big)) - lens_round(rho, big, r));
    }
}

/// The same where the top meets the plate at an angle: the wedge's slope and the segment's arc
/// rounded under and over the band's height, each cut off at the plate.
#[test]
fn a_bands_top_edge_meeting_its_plate_at_an_angle_is_kept_over_it() {
    for (h, r) in [(2.0, 1.0), (2.0, 8.0), (6.0, 1.0), (6.0, 8.0)] {
        let src = wedge(h, &format!("part.band.near round(r: {r}mm) part.band.k2"));
        close(built(&src, "part"), bare(30.0 * h) - wedge_round(h, r));
    }
    for (h, r) in [(2.0f64, 1.0), (2.0, 10.0), (8.0, 3.0), (8.0, 10.0)] {
        let big = (900.0 + h * h) / (2.0 * h);
        let area = big * big * (30.0 / big).asin() - 30.0 * (big - h);
        let src = segment(h, &format!("part.band.near round(r: {r}mm) part.band.k1"));
        close(built(&src, "part"), bare(area) - segment_round(h, r));
    }
}

/// **The band's top edge where it stands on the disc**: its foot arcs are short, the band at their
/// seams with the crown only 0.86 off the disc.  A round under what that allows (0.5 and 0.9) is
/// built: the crown's run whole and each foot arc's ring cut off at the disc and kept outside it,
/// each line of its section `t` in turned from the seam to where the foot arc's offset meets the
/// disc's circle.
#[test]
fn a_tapered_bands_top_edge_on_a_disc_is_rounded_short_of_its_seams() {
    for r in [0.5, 0.9] {
        let src = disc_band(40.0, 15.0, 15.0, &format!("part.band.near round(r: {r}mm) part.band"));
        close(built(&src, "part"), disc_bare(40.0, 15.0, 15.0) - disc_round(40.0, 15.0, 15.0, r));
    }
}

/// Over it the disc, which each foot arc runs on tangent into and its run ends on, reaches back over
/// the section the foot arc's round shares with the crown's at their seam: refused, by its faces, by
/// how far the seam stands off the disc, and by what is true to do — the disc cannot be moved
/// (#994: "Move `disc.eo` clear of the seam" said of a face the band stands on).  At 1 the gate
/// refused the crown's run before ("disagrees with the field"): the field read a point in the band's
/// side's plane, where the gate probes the round's leg, by the disc beside the band, not the crown.
#[test]
fn a_tapered_bands_top_edge_on_a_disc_is_refused_where_the_disc_reaches_over_its_seam() {
    for r in [1.0, 2.0] {
        let src = disc_band(40.0, 15.0, 15.0, &format!("part.band.near round(r: {r}mm) part.band"));
        let why = crate::common::rounds::refused(&solved(&src), "part");
        assert!(why.contains("into `band.a2` across their seam") && why.contains("`band.a3` runs on tangent into `disc.eo`")
            && why.contains("the seam stands 0.86mm off `disc.eo`") && why.contains("A smaller round, or a longer `band.a3`")
            && !why.contains("Move"), "{why}");
    }
}
