//! **A point in space held to a generated surface** (issue #70, part 1).  A prism's side
//! generating under a motion that keeps the prism's view sweeps the surface its edge's planar
//! envelope is extruded into, square to the view, so the drawing holds it: `flank :=
//! envelope(side, under: cutting, …)` over `side := surface(rack_tooth, edge: rack_flank)` is that
//! envelope, marked an extrusion (`CurveE::extrusion`), and a point drawn in another view is `on`
//! it by its place in the prism's view (`CKind::PointOnExtrusion`, its kernel the definition's).
//!
//! The fixture is `rack_cut.rs`'s tooth space drawn in `std.front`; the point stands in a view
//! parallel to the page's horizontal, 1.9 above the gear's centre — across the flank of the tooth
//! space — and 3 off the page along the extrusion.  What it lands on is checked against the
//! swept solid's material field, not the curve it was solved against.

use crate::common::{ent, fd_jacobian};
use gcs_core::{library, program};
use gcs_core::solve::{solve, SolveOpts};

/// `src`, its `use std` linked, elaborated with no error.
fn build(src: &str) -> program::Elaborated {
    let (p, errors, linked) = library::parse_linked(src);
    assert!(errors.is_empty() && linked.is_empty(), "{errors:?} {linked:?}");
    let e = program::elaborate(&p);
    assert!(e.ok(), "{:?}", e.errors().map(|d| (d.code.as_str(), &d.message)).collect::<Vec<_>>());
    e
}

/// The tooth space's drawing: its blank's rim, the rack's pitch line and tooth, the upper flank
/// named.
const DRAWN: &str = "\
  o := point
  c := circle(center: o) hint(r: 22)
  s0 := point hint(x: 20, y: 0)
  s1 := point hint(x: 20, y: 10)
  slide := line(s0, s1)
  t0 := point hint(x: 18, y: -0.84)
  t1 := point hint(x: 18, y: 0.84)
  t2 := point hint(x: 24, y: 3.03)
  t3 := point hint(x: 24, y: -3.03)
  rack_flank := line(t1, t2)
";

/// What it states and makes: the numbers, the blank, the rack's tooth and its sweep, and the
/// flank's surface — a prism's side — and its envelope.
const MADE: &str = "\
fix(x == 0, y == 0) o
radius(22mm) c
blank := solid(face(c), depth: 6mm)
fix(x == 20, y == 0) s0
fix(x == 20, y == 10) s1
fix(x == 18, y == -0.842856) t0
fix(x == 18, y == 0.842856) t1
fix(x == 24, y == 3.026666) t2
fix(x == 24, y == -3.026666) t3
tooth := face(t0, t1, rack_flank, t3, -> close)
construction rack_tooth := solid(tooth, from: -8mm, to: 2mm)
turn := motion(about: o, ratio: 1)
rack := motion(along: slide, advance: 2 * pi * 20mm)
cutting := motion(rack, relative_to: turn)
construction space := solid(rack_tooth, under: cutting, from: -60deg, to: 60deg)
side := surface(rack_tooth, edge: rack_flank)
flank := envelope(side, under: cutting, from: -30deg, to: 30deg)
";

/// The tooth space drawn in `std.front`.
fn rack() -> String {
    format!("unit mm\nuse std\nin std.front {{\n{DRAWN}}}\n{MADE}")
}

/// A plane square to the front, through its horizontal 1.9 above `o` (its `u` the front's `x`,
/// its `v` the front's normal reversed), and a point drawn in it at `along` along the extrusion.
fn across(along: f64) -> String {
    format!("{}\
cut := plane
fix(x == 0, y == 0, z == 1.9) cut
fix(x == 1, y == 0, z == 0) cut.u
fix(x == 0, y == 1, z == 0) cut.v
p := point in cut hint(x: 20, y: {along})
fix(y == {along}) p
p coincident flank
", rack())
}

/// **A point drawn in another view lands on the generated surface.**  Its lift stands 1.9 above
/// the page's horizontal through `o` and `along` off the page, and there the rack's sweep — the
/// swept solid, read as a material field — changes sign within a micron of it: the point is on
/// the tooth space's flank in space, though it is drawn in neither the prism's view nor the
/// sweep's.
/// Moved along the extrusion it stays at the same place in the page: what holds it is a surface.
#[test]
fn a_point_in_space_lands_on_the_generated_flank() {
    let mut seen = Vec::new();
    for along in [3., 5.] {
        let mut e = build(&across(along));
        let r = solve(&mut e.sketch, SolveOpts::default());
        assert!(r.success, "{}", r.message);
        let p = ent(&e, "p").i();
        let w = e.sketch.lifted(p);
        // the page is the world's xz, its normal -y: the point's place is (x, 1.9) on the page,
        // `along` off it
        assert!((w[2] - 1.9).abs() < 1e-9 && (w[1] - along).abs() < 1e-9, "the point stands at {w:?}");
        assert!(w[0] > 18. && w[0] < 22., "the point is not in the tooth space's flank: {w:?}");
        let space = ent(&e, "space").i();
        let field = gcs_core::solid::MaterialField::read(&e.sketch, space, 1e-10).unwrap();
        let (inner, outer) = (field.side([w[0] - 1e-6, w[1], w[2]]), field.side([w[0] + 1e-6, w[1], w[2]]));
        assert!(inner * outer < 0., "the sweep's field reads {inner} and {outer} either side of the point");
        seen.push(w);
    }
    assert!((seen[0][0] - seen[1][0]).abs() < 1e-9, "moved along the extrusion, the point moved across it: {seen:?}");
}

/// The contact's Jacobian is the system's derivative: the point's lift, the roll and the curve's
/// columns against central differences of the assembled residuals.
#[test]
fn the_extrusion_contact_has_its_own_jacobian() {
    let mut e = build(&across(3.));
    let r = solve(&mut e.sketch, SolveOpts::default());
    assert!(r.success, "{}", r.message);
    let c = e.sketch.constraints.iter().find(|c| c.kind == gcs_core::constraints::CKind::PointOnExtrusion);
    assert!(c.is_some(), "`p on flank` is not a point on the extrusion");
    fd_jacobian(&e.sketch, 1e-5);
}
