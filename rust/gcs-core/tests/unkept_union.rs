//! A union the body keeps none of (#961 item 6, W119): a body is its stock, plus its unions, less
//! its cuts, within its commons, whatever order the statements stand in — so a union lying wholly
//! in what the cuts take is not in the body at all, and the statement that wrote it is told so,
//! naming the union, the body and what took it.  The body is as the rule makes it: said, not
//! changed.  The test is the containment `every q in u: q in what the cuts take`, read off the
//! arrangement's cells: a partly taken union says nothing, one flush with a cut's wall is wholly
//! taken, one a hair wider is not.
use gcs_core::program::{self, Code, Diag, Severity};
use gcs_core::solve::{solve, SolveOpts};

/// A ring (r 50, 0 to 20) bored through (`hole`, r 30), and a hub (r `hub`, `from` to `to`)
/// unioned into it, the union written after the cut or before it.
fn ring(hub: f64, from: f64, to: f64, union_first: bool) -> String {
    let mut src = format!("\
unit mm
use std
use std.solids
use std.solids (cut, union)
in std.top {{
  o := point
  fix((0, 0)) o
  outer := circle(center: o) hint(r: 50)
  radius(50mm) outer
  bore := circle(center: o) hint(r: 30)
  radius(30mm) bore
  web := circle(center: o) hint(r: {hub})
  radius({hub}mm) web
}}
ring := solid(std.solids.Prism(face(outer), from: 0mm, to: 20mm))
hole := solid(std.solids.Through(face(bore)))
hub := solid(std.solids.Prism(face(web), from: {from}mm, to: {to}mm))
part := solid(ring)
");
    let (cut, union) = ("hole cut part\n", "hub union part\n");
    src.push_str(if union_first { union } else { cut });
    src.push_str(if union_first { cut } else { union });
    src
}

/// The document's diagnostics of its solids, each with its line, solved first; and the volume
/// (mm³) of `part` as built.
fn said(src: &str) -> (Vec<(Diag, u32)>, f64) {
    let mut e = crate::common::build(src);
    solve(&mut e.sketch, SolveOpts::default());
    let diags = program::solid_diagnostics(&e.sketch, &e.map, &e.program).into_iter()
        .map(|d| { let line = d.at(e.text()).0; (d, line) }).collect();
    let part = e.sketch.solid_called("part").expect("a part");
    let x = e.sketch.exact_solid(part).expect("the part builds");
    (diags, gcs_core::brep::props::volume(&x.brep))
}

/// Only the W119s.
fn unkept(diags: &[(Diag, u32)]) -> Vec<(&str, u32)> {
    diags.iter().filter(|(d, _)| d.code == Code::W119)
        .map(|(d, l)| (d.message.as_str(), *l)).collect()
}

const TAIL: &str = "a body is its stock and what is in `union` with it, less what cuts it and \
                    within what it is in `common` with, whatever order its statements stand in — ";

/// The ring bored through, as the body rule makes it whatever is unioned inside the bore.
fn bored() -> f64 {
    std::f64::consts::PI * (50.0f64.powi(2) - 30.0f64.powi(2)) * 20.0
}

#[test]
fn a_hub_wholly_in_the_bore_is_said_at_its_union_whatever_the_order() {
    let want = format!("`hub` lies wholly in what `hole` cuts from `part`, so `part` keeps none \
                        of it: {TAIL}a `union` written after a cut puts nothing back");
    for union_first in [false, true] {
        let (diags, volume) = said(&ring(20.0, 5.0, 15.0, union_first));
        // the statement `hub union part`: the last line, or the one before it
        let line = if union_first { 19 } else { 20 };
        assert_eq!(unkept(&diags), vec![(want.as_str(), line)], "{diags:#?}");
        let d = &diags.iter().find(|(d, _)| d.code == Code::W119).unwrap().0;
        assert_eq!(d.severity(), Severity::Warning);
        assert!(d.stmt.is_some(), "said at its statement: {d:?}");
        // a warning, never the body changed: the part is the plain ring bored
        assert!(diags.iter().all(|(d, _)| d.severity() != Severity::Error), "{diags:#?}");
        assert!((volume - bored()).abs() < 1e-6 * bored(), "{volume} against {}", bored());
    }
}

#[test]
fn a_hub_partly_cut_or_held_beyond_the_bore_says_nothing() {
    // a hub out to r 40 keeps its ring 30 to 40, and one to r 35 keeps 30 to 35 (material the
    // ring has already: the union is redundant there, but kept — not lost)
    for hub in [40.0, 35.0] {
        let (diags, _) = said(&ring(hub, 5.0, 15.0, false));
        assert_eq!(unkept(&diags), vec![], "r {hub}: {diags:#?}");
    }
}

#[test]
fn a_hub_flush_with_the_bores_wall_is_wholly_cut_and_one_a_hair_wider_is_not() {
    // its side on the bore's wall: no point of it lies beyond the wall, so nothing of it is kept
    let (diags, volume) = said(&ring(30.0, 5.0, 15.0, false));
    assert_eq!(unkept(&diags).len(), 1, "{diags:#?}");
    assert!((volume - bored()).abs() < 1e-6 * bored(), "{volume}");
    // a micrometre wider keeps a skin a micrometre thick: no tolerance swallows it
    let (diags, _) = said(&ring(30.001, 5.0, 15.0, false));
    assert_eq!(unkept(&diags), vec![], "{diags:#?}");
}

#[test]
fn a_hub_standing_proud_of_a_through_bore_is_cut_through_it() {
    // a prism through everything spans the body it cuts, the hub's height included: the hub up
    // to 25 above a ring to 20 is still wholly in it
    let (diags, volume) = said(&ring(20.0, 5.0, 25.0, false));
    assert_eq!(unkept(&diags).len(), 1, "{diags:#?}");
    assert!((volume - bored()).abs() < 1e-6 * bored(), "{volume}");
}

/// A ring cut by two bores of its own heights, one below 10 and one above, `hub` unioned across
/// both; and a square boss beside, a lug outside and a pin on top, each unioned too.
fn two_bores(hub_from: f64, hub_to: f64) -> String {
    format!("\
unit mm
use std
use std.solids
use std.solids (cut, union)
in std.top {{
  o := point
  fix((0, 0)) o
  outer := circle(center: o) hint(r: 50)
  radius(50mm) outer
  bore := circle(center: o) hint(r: 30)
  radius(30mm) bore
  web := circle(center: o) hint(r: 20)
  radius(20mm) web
  a := point
  b := point
  c := point
  d := point
  fix((-10, 32)) a
  fix((10, 32)) b
  fix((10, 45)) c
  fix((-10, 45)) d
  sq := face(a, b, c, d, -> close)
  l := point
  fix((60, 0)) l
  lugc := circle(center: l) hint(r: 15)
  radius(15mm) lugc
}}
ring := solid(std.solids.Prism(face(outer), from: 0mm, to: 20mm))
lower := solid(std.solids.Prism(face(bore), from: -1mm, to: 10mm))
upper := solid(std.solids.Prism(face(bore), from: 10mm, to: 21mm))
hub := solid(std.solids.Prism(face(web), from: {hub_from}mm, to: {hub_to}mm))
boss := solid(std.solids.Prism(sq, from: 15mm, to: 30mm))
lug := solid(std.solids.Prism(face(lugc), from: 0mm, to: 20mm))
part := solid(ring)
boss union part
lower cut part
hub union part
upper cut part
lug union part
")
}

#[test]
fn a_union_taken_by_two_cuts_together_names_both_and_no_other_union() {
    let (diags, _) = said(&two_bores(5.0, 15.0));
    let want = format!("`hub` lies wholly in what `lower` and `upper` cut from `part`, so `part` \
                        keeps none of it: {TAIL}a `union` written after a cut puts nothing back");
    // the boss (beside the bore, standing proud) and the lug keep their material: unnamed
    assert_eq!(unkept(&diags), vec![(want.as_str(), 37)], "{diags:#?}");
    // a hub reaching below the lower bore keeps what is under it
    let (diags, _) = said(&two_bores(-5.0, 15.0));
    assert_eq!(unkept(&diags), vec![], "{diags:#?}");
}

#[test]
fn a_union_outside_what_the_body_is_kept_in_common_with_is_said() {
    let src = |x: f64| format!("\
unit mm
use std
use std.solids
use std.solids (union, common)
in std.top {{
  o := point
  fix((0, 0)) o
  outer := circle(center: o) hint(r: 50)
  radius(50mm) outer
  l := point
  fix(({x}, 0)) l
  lugc := circle(center: l) hint(r: 5)
  radius(5mm) lugc
  k := circle(center: o) hint(r: 52)
  radius(52mm) k
}}
ring := solid(std.solids.Prism(face(outer), from: 0mm, to: 20mm))
lug := solid(std.solids.Prism(face(lugc), from: 0mm, to: 20mm))
keep := solid(std.solids.Prism(face(k), from: -1mm, to: 21mm))
part := solid(ring)
lug union part
keep common part
");
    let (diags, _) = said(&src(60.0));
    let want = format!("`lug` lies wholly outside `keep`, which `part` is in `common` with, so \
                        `part` keeps none of it: {TAIL}a `union` written after a `common` is kept \
                        within it too");
    assert_eq!(unkept(&diags), vec![(want.as_str(), 21)], "{diags:#?}");
    // straddling the common's wall, it keeps what is inside
    let (diags, _) = said(&src(55.0));
    assert_eq!(unkept(&diags), vec![], "{diags:#?}");
}

#[test]
fn a_union_with_no_cut_nor_common_reads_nothing() {
    // a boss wholly inside the plate: redundant, its material the body's as stated
    let src = "\
unit mm
use std
use std.solids
use std.solids (union)
in std.top {
  o := point
  fix((0, 0)) o
  outer := circle(center: o) hint(r: 50)
  radius(50mm) outer
  web := circle(center: o) hint(r: 20)
  radius(20mm) web
}
ring := solid(std.solids.Prism(face(outer), from: 0mm, to: 20mm))
hub := solid(std.solids.Prism(face(web), from: 5mm, to: 15mm))
part := solid(ring)
hub union part
";
    let mut e = crate::common::build(src);
    solve(&mut e.sketch, SolveOpts::default());
    let part = e.sketch.solid_called("part").unwrap();
    assert_eq!(gcs_core::solid::unkept(&e.sketch, part), Ok(vec![]));
    let diags = program::solid_diagnostics(&e.sketch, &e.map, &e.program);
    assert!(diags.iter().all(|d| d.code != Code::W119), "{diags:#?}");
}

#[test]
fn a_hub_put_on_a_body_bored_already_fills_the_bore_and_says_nothing() {
    // the remedy the warning points at: the bore cut in a body of its own, the hub on that one
    let src = ring(20.0, 5.0, 15.0, false).replace(
        "part := solid(ring)\nhole cut part\nhub union part\n",
        "bored := solid(ring)\nhole cut bored\npart := solid(bored)\nhub union part\n");
    let (diags, volume) = said(&src);
    assert_eq!(unkept(&diags), vec![], "{diags:#?}");
    let hub = std::f64::consts::PI * 20.0f64.powi(2) * 10.0;
    assert!((volume - bored() - hub).abs() < 1e-6 * bored(), "{volume} against {}", bored() + hub);
}
