//! **Draft** (#245): `solid(f, from: a, to: b, draft: θ)` tapers a prism's sides from the plane its
//! face is drawn in, the parting plane — each section the face moved `|h| tan θ` into its
//! material, so the sides lean away from the plane either side of it, as a forging's come out of
//! its die. A straight side stays a plane and a circle's a cone, so the exact kernel builds it
//! (two lofts glued at the parting plane), and each face keeps the name it has undrafted. Held
//! here: a drafted block's volume and a drafted disc's to their closed forms by every evaluator
//! (the exact kernel to 1e-9, the facets exactly where the sides are planes, the field by its
//! reading of the tilted sides), the names, a hole growing as the outside shrinks, and what is
//! refused and why.
use gcs_core::brep::{export, props, sweep::Say};
use gcs_core::program::Elaborated;
use gcs_core::solid::{ApproximationPolicy::Report, MaterialField};
use std::f64::consts::PI;

use crate::common::{ent, text::solved};

/// A 20 × 30 block in the front plane (x along u, z along v; its normal is −y), swept `sweep`,
/// with whatever `more` adds.
fn block(sweep: &str, more: &str) -> String {
    format!("unit mm
use std
in std.front {{
  a := point
  b := point
  c := point
  d := point
  fix((0, 0)) a
  fix((20, 0)) b
  fix((20, 30)) c
  fix((0, 30)) d
  ab := line(a, b)
  bc := line(b, c)
  cd := line(c, d)
  da := line(d, a)
  f := face(ab, bc, cd, da)
{more}}}
blk := solid(f, {sweep})
")
}

/// The exact kernel's solid `name`, or why it is not built.
fn exact(e: &Elaborated, name: &str) -> Result<gcs_core::brep::topo::Brep, String> {
    let say = Say { stage: &|_: &str| {}, mark: &|_| {} };
    export::exact(&e.sketch, ent(e, name).i(), None, None, &say).map(|x| x.solid).map_err(|r| r.message)
}

/// The volume between ordinates `h0` and `h1` of a `w` × `l` rectangle drafted by `t` (a tangent):
/// the section at `h` is `(w − 2|h| t)(l − 2|h| t)`.
fn rectangle(w: f64, l: f64, t: f64, h0: f64, h1: f64) -> f64 {
    // ∫₀^H (w − 2ht)(l − 2ht) dh
    let up = |h: f64| w * l * h - (w + l) * t * h * h + 4.0 / 3.0 * t * t * h * h * h;
    let side = |h: f64| if h >= 0.0 { up(h) } else { -up(-h) };
    side(h1) - side(h0)
}

fn close(what: &str, got: f64, want: f64, rel: f64) {
    eprintln!("{what}: {got:.12} against {want:.12}");
    assert!((got - want).abs() <= rel * want.abs(), "{what}: {got} against {want}");
}

/// A block drafted 3° each side of its parting plane: the exact kernel's volume is the closed
/// form's to 1e-9, and so are the facets' (its sides are planes); the field reads each tilted side
/// as its own distance.
#[test]
fn a_drafted_block_is_its_closed_form() {
    let t = 3f64.to_radians().tan();
    for (sweep, h0, h1) in [("from: -5, to: 5, draft: 3deg", -5.0, 5.0),
                            ("from: 2, to: 7, draft: 3deg", 2.0, 7.0),
                            ("depth: 6, draft: 3deg", -6.0, 0.0)] {
        let e = solved(&block(sweep, ""));
        let want = rectangle(20.0, 30.0, t, h0, h1);
        close(&format!("{sweep}, exactly"), props::volume(&exact(&e, "blk").unwrap()), want, 1e-9);
        let read = e.sketch.evaluated_solid(ent(&e, "blk").i(), Report).unwrap();
        close(&format!("{sweep}, as the app reads it"), read.volume(), want, 1e-9);
    }
    // the field: at ordinate 3 along the front's normal (y = −3), the side `ab` (z = 0 at the
    // parting plane) has come in by 3 tan 3°, and the field there is its distance
    let e = solved(&block("from: -5, to: 5, draft: 3deg", ""));
    // and the facet term, its sides planes, holds the same points on the same sides
    let field = MaterialField::read(&e.sketch, ent(&e, "blk").i(), 1e-10).unwrap();
    let facets = gcs_core::solid::resolve(&e.sketch, ent(&e, "blk").i(), gcs_core::solid::REPORT_UNIT);
    for (y, dz) in [(-3.0f64, 0.05), (-3.0, -0.05), (4.0, 0.02), (4.0, -0.02), (0.0, -0.1), (0.0, 0.1)] {
        let z = y.abs() * t + dz;
        let want = -dz * 3f64.to_radians().cos();
        let got = field.side([10.0, y, z]);
        assert!((got - want).abs() <= 1e-9, "at y {y}, z {z}: {got} against {want}");
        assert_eq!(facets.inside([10.0, y, z]), dz > 0.0, "the facets at y {y}, z {z}");
    }
}

/// Every face keeps the name it has undrafted (a Boolean never renames, and a draft is no
/// Boolean): the caps `near` and `far`, a side per edge — two pieces of one name where it crosses
/// the parting plane, none where it does not.
#[test]
fn a_drafted_prisms_faces_keep_their_names() {
    for (sweep, pieces) in [("from: -5, to: 5, draft: 3deg", 2), ("from: 0, to: 5, draft: 3deg", 1)] {
        let e = solved(&block(sweep, ""));
        let b = exact(&e, "blk").unwrap();
        let named = |n: &str| b.faces.iter().filter(|f| f.name.ends_with(n)).count();
        for side in ["ab", "bc", "cd", "da"] {
            assert_eq!(named(side), pieces, "{sweep}: `{side}` in {:?}",
                       b.faces.iter().map(|f| &f.name).collect::<Vec<_>>());
        }
        assert_eq!((named("near"), named("far")), (1, 1), "{sweep}");
        assert_eq!(b.faces.len(), 2 + 4 * pieces, "{sweep}");
        let facets = e.sketch.evaluated_solid(ent(&e, "blk").i(), Report).unwrap();
        for face in ["near", "far", "ab", "bc", "cd", "da"] {
            assert!(facets.surviving_faces().contains(&format!("blk.{face}")), "{face}");
        }
    }
}

/// A disc drafted from its mid-plane is two frustums of cones, `π h (R² + R r + r²) / 3` each;
/// one with a bore drafted the same way loses a bore that widens as the disc narrows.
#[test]
fn a_drafted_disc_is_two_frustums_and_its_bore_widens() {
    let t = 5f64.to_radians().tan();
    // a frustum `h` high from radius `r0`, its radius growing `taper` a unit along
    let cone = |r0: f64, h: f64, taper: f64| { let r1 = r0 + h * taper; PI * h * (r0 * r0 + r0 * r1 + r1 * r1) / 3.0 };
    let disc = |holes: &str| format!("unit mm
use std
in std.front {{
  o := point
  fix((0, 0)) o
  rim := circle(center: o) hint(r: 20)
  bore := circle(center: o) hint(r: 6)
  radius(20) rim
  radius(6) bore
  f := face(rim{holes})
}}
disc := solid(f, from: -4, to: 3, draft: 5deg)
");
    let e = solved(&disc(""));
    let want = cone(20.0, 4.0, -t) + cone(20.0, 3.0, -t);
    close("the disc, exactly", props::volume(&exact(&e, "disc").unwrap()), want, 1e-9);
    // the app reads the exact solid, and the facet term's polygons lie within the report's
    // sagitta of the cones
    let read = e.sketch.evaluated_solid(ent(&e, "disc").i(), Report).unwrap();
    close("the disc, as the app reads it", read.volume(), want, 1e-9);
    let facets = gcs_core::solid::resolve(&e.sketch, ent(&e, "disc").i(), gcs_core::solid::REPORT_UNIT);
    let r = 20.0 - 2.0 * t;
    assert!(facets.inside([r - 0.01, 2.0, 0.0]) && !facets.inside([r + 0.01, 2.0, 0.0]));
    let e = solved(&disc(", holes: bore"));
    let want = want - cone(6.0, 4.0, t) - cone(6.0, 3.0, t);
    close("the bored disc, exactly", props::volume(&exact(&e, "disc").unwrap()), want, 1e-9);
    // the field: at y = 2 (ordinate −2) the bore's wall stands at 6 + 2 tan 5° and the rim's at
    // 20 − 2 tan 5°, each read as its cone's distance
    let field = MaterialField::read(&e.sketch, ent(&e, "disc").i(), 1e-10).unwrap();
    let cos = 5f64.to_radians().cos();
    let facets = gcs_core::solid::resolve(&e.sketch, ent(&e, "disc").i(), gcs_core::solid::REPORT_UNIT);
    for (r, sign) in [(6.0 + 2.0 * t, 1.0), (20.0 - 2.0 * t, -1.0)] {
        for dx in [0.01, -0.01] {
            let (got, want) = (field.side([r + dx, 2.0, 0.0]), -sign * dx * cos);
            assert!((got - want).abs() <= 1e-9, "at {}: {got} against {want}", r + dx);
            assert_eq!(facets.inside([r + dx, 2.0, 0.0]), want < 0.0, "the facets at {}", r + dx);
        }
    }
}

/// What a draft cannot say, and where it is said: on a turned solid (the parser), a right angle or
/// more (the elaborator, E080), a draft taking a side to nothing within the prism's reach (where
/// it is evaluated), an arc meeting a side at an angle (the exact kernel: its cone and the side's
/// plane meet along a curve) and a side of a drafted prism sketched on.
#[test]
fn what_a_draft_cannot_say_is_refused() {
    let (_, errs) = crate::common::parse(&block("about: ab, draft: 3deg", ""));
    assert!(errs.iter().any(|e| e.message.contains("`draft:` tapers a prism's sides")), "{errs:?}");
    let (p, errs) = crate::common::parse(&block("from: 0, to: 5, draft: 90deg", ""));
    assert!(errs.is_empty(), "{errs:?}");
    let e = gcs_core::program::elaborate(&p);
    assert!(e.diags.iter().any(|d| d.code.as_str() == "E080" && d.message.contains("right angle")),
            "{:?}", e.diags);
    // 20 wide, 2 × 10 tan 50° > 20: the bottom and top sides meet before the cap
    let e = solved(&block("from: 0, to: 10, draft: 50deg", ""));
    let why = e.sketch.evaluated_solid(ent(&e, "blk").i(), Report).unwrap_err();
    assert!(why.contains("to nothing"), "{why}");
    // a slot's round end, meeting its sides tangent, is built: its cone touches each side's
    // plane along a line
    let e = solved("unit mm
use std
in std.front {
  a := point
  b := point
  c := point
  d := point
  m := point
  fix((0, -5)) a
  fix((20, -5)) b
  fix((20, 5)) c
  fix((0, 5)) d
  fix((20, 0)) m
  low := line(a, b)
  end := arc(center: m, start: b, end: c)
  high := line(c, d)
  back := line(d, a)
  f := face(low, end, high, back)
}
s := solid(f, from: -2, to: 2, draft: 4deg)
");
    let t = 4f64.to_radians().tan();
    // the section at `h`: the 20 × 10 rectangle less 2|h|t round its three straight sides, and
    // the half disc of radius 5 − |h|t
    let section = |h: f64| { let k = h.abs() * t; (20.0 - k) * (10.0 - 2.0 * k) + PI * (5.0 - k).powi(2) / 2.0 };
    // a quadratic in h each side: Simpson's rule is exact
    let half = (section(0.0) + 4.0 * section(1.0) + section(2.0)) * 2.0 / 6.0;
    close("the drafted slot, exactly", props::volume(&exact(&e, "s").unwrap()), 2.0 * half, 1e-9);
    // a quarter disc's arc meets its two radii square: refused by name
    let e = solved("unit mm
use std
in std.front {
  o := point
  a := point
  b := point
  fix((0, 0)) o
  fix((10, 0)) a
  fix((0, 10)) b
  q := arc(center: o, start: a, end: b)
  l1 := line(b, o)
  l2 := line(o, a)
  f := face(q, l1, l2)
}
s := solid(f, from: -2, to: 2, draft: 4deg)
");
    let why = exact(&e, "s").unwrap_err();
    assert!(why.contains("at an angle where one is an arc"), "{why}");
}

/// A drafted side leans off its face's plane, so it is no set to sketch on; a cap still is.
#[test]
fn a_drafted_side_is_no_set_to_sketch_on() {
    let src = block("from: -5, to: 5, draft: 3deg", "")
        + "p := point\np coincident blk.near.surface\n";
    solved(&src);
    let (p, _) = crate::common::parse(&(block("from: -5, to: 5, draft: 3deg", "")
        + "p := point\np coincident blk.ab.surface\n"));
    let e = gcs_core::program::elaborate(&p);
    assert!(e.diags.iter().any(|d| d.message.contains("drafted side")), "{:?}", e.diags);
}

/// Sizing a drafted prism by its arrow rewrites its extent and keeps its draft: the gesture says
/// how thick, never how it tapers.
#[test]
fn resizing_a_drafted_prism_keeps_its_draft() {
    use gcs_core::edit::{self, SolidSweep};
    let e = solved(&block("from: -5, to: 5, draft: 3deg", ""));
    let how = SolidSweep { depth: Some("4".into()), ..Default::default() };
    let out = edit::set_sweep(&e, &e.program, ent(&e, "blk").i(), &how);
    assert_eq!(out.refused, None);
    assert!(out.text.contains("blk := solid(f, depth: 4, draft: 3deg)"), "{}", out.text);
    let e = solved(&out.text);
    let want = rectangle(20.0, 30.0, 3f64.to_radians().tan(), -4.0, 0.0);
    close("resized", props::volume(&exact(&e, "blk").unwrap()), want, 1e-9);
}
