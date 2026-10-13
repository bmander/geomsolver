//! A point at a fraction of a line's run or of an arc's turn (#248, lofts L0 finding 2):
//! `m coincident(t == 0.25) g`.  The contact owns its place `t`, as a spline's does — a line's
//! `m = p1 + t·(p2 − p1)`, an arc's `t` of the way round from `start` to `end` by angle — pinned,
//! shared through a declared unknown, or left free and seeded where the point stands; across
//! views and for a point in space the same, read in space; in a traced body pinned over the swept
//! formal.  Held against closed forms worked out here, and against the station probes that used
//! to be handed the guide's length and sweep (`docs/lofts/l0/stations/{reducer,elbow}.sv`).

use gcs_core::constraints::CKind;
use gcs_core::diagnose::{diagnose, DiagnoseOptions};
use gcs_core::program::{elaborate, Elaborated};
use gcs_core::solve::{solve, SolveOpts};

use crate::common::{build, ent, parse};

/// `src` elaborated and solved, or the test fails saying why.
fn solved(src: &str) -> Elaborated {
    let mut e = build(src);
    let r = solve(&mut e.sketch, SolveOpts::default());
    assert!(r.success, "{}\n{src}", r.message);
    e
}

/// Where the point the document calls `n` stands in space.
fn at(e: &Elaborated, n: &str) -> [f64; 3] {
    e.sketch.world_point(ent(e, n).i())
}

fn near(got: [f64; 3], want: [f64; 3], what: &str) {
    let off = (0..3).map(|i| (got[i] - want[i]).abs()).fold(0.0, f64::max);
    assert!(off < 1e-9, "{what}: {got:?} against {want:?}");
}

/// The constraints of `kind` the drawing holds.
fn of_kind(e: &Elaborated, kind: CKind) -> usize {
    e.sketch.constraints.iter().filter(|c| c.kind == kind).count()
}

/// The place the one contact of `kind` owns, solved.
fn place(e: &Elaborated, kind: CKind) -> f64 {
    let c = e.sketch.constraints.iter().find(|c| c.kind == kind).expect("a contact");
    let (_, t) = c.fraction_contact().expect("added, so its place is a Param");
    e.sketch.params[t as usize].value
}

fn dof(e: &mut Elaborated) -> i64 {
    diagnose(&mut e.sketch, DiagnoseOptions::default()).dof as i64
}

/// A quarter arc of radius 20 about (5, −3), from 30° to 120°.
const QUARTER: &str = "\
  c := point
  fix((5mm, -3mm)) c
  e := point hint((22mm, 7mm))
  q := point hint((-5mm, 14mm))
  k := arc(center: c, start: e, end: q)
  radius(20mm) k
  sa := line(c, e)
  sb := line(c, q)
  xa := point hint((15mm, -3mm))
  ref := line(c, xa)
  horizontal ref
  ref angle(30deg) sa
  ref angle(120deg) sb
";

/// The point at the bearing `deg` on the quarter arc.
fn on_quarter(deg: f64) -> [f64; 3] {
    let r = deg.to_radians();
    [5.0 + 20.0 * r.cos(), -3.0 + 20.0 * r.sin(), 0.0]
}

/// **A quarter of the way along a line** is `p1 + (p2 − p1) / 4`, and the statement is its own
/// kind: `t` written is what makes the contact a fraction, so one without it stays `PointOnLine`
/// and pays no column.
#[test]
fn a_point_a_quarter_of_the_way_along_a_line() {
    let e = solved("\
unit mm
use std
in std.top {
  a := point
  b := point
  fix((0mm, 0mm)) a
  fix((40mm, 20mm)) b
  g := line(a, b)
  m := point
  m coincident(t == 0.25) g
  n := point hint((10mm, 30mm))
  n coincident g
}
");
    near(at(&e, "m"), [10.0, 5.0, 0.0], "m");
    assert_eq!(of_kind(&e, CKind::PointAlongLine), 1);
    assert_eq!(of_kind(&e, CKind::PointOnLine), 1, "no `t`, no fraction");
    assert_eq!(place(&e, CKind::PointAlongLine), 0.25);
}

/// **Halfway round a quarter arc** is the bearing midway between its ends, whatever its centre
/// and wherever it starts.
#[test]
fn a_point_halfway_round_a_quarter_arc() {
    let e = solved(&format!("unit mm\nuse std\nin std.top {{\n{QUARTER}  m := point\n  m coincident(t == 0.5) k\n  n := point\n  n coincident(t == 1) k\n}}\n"));
    near(at(&e, "m"), on_quarter(75.0), "m");
    near(at(&e, "n"), on_quarter(120.0), "n at the end");
    assert_eq!(of_kind(&e, CKind::PointAlongArc), 2);
}

/// **Two points at one fraction of two lines** through one declared unknown are similar
/// triangles: the unknown is one place both contacts own, and a distance between the points
/// settles it.  Legs 40 and 30 make the hypotenuse 50, so a gap of 13 is the fraction 0.26.
#[test]
fn two_lines_sharing_one_fraction_are_similar_triangles() {
    let mut e = solved("\
unit mm
use std
param s: Scalar hint(0.4)
in std.top {
  o := point
  fix((0mm, 0mm)) o
  a := point
  b := point
  fix((40mm, 0mm)) a
  fix((0mm, 30mm)) b
  g := line(o, a)
  h := line(o, b)
  m := point hint((15mm, 1mm))
  n := point hint((1mm, 10mm))
  m coincident(t == s) g
  n coincident(t == s) h
  m distance(13mm) n
}
");
    let p = e.sketch.shared["s"].param;
    assert_eq!(e.sketch.constraints.iter().filter(|c| c.owns(p)).count(), 2, "one unknown, two owners");
    let s = e.sketch.params[p as usize].value;
    assert!((s - 0.26).abs() < 1e-9, "s = {s}");
    near(at(&e, "m"), [10.4, 0.0, 0.0], "m");
    near(at(&e, "n"), [0.0, 7.8, 0.0], "n");
    assert_eq!(dof(&mut e), 0);
}

/// A fraction is one place along any run: a line's and an arc's may share it, but not a spline's
/// parameter, whose interval is the spline's own.
#[test]
fn a_fraction_is_shared_by_runs_and_not_by_a_spline() {
    let e = solved(&format!("unit mm\nuse std\nparam s: Scalar hint(0.5)\nin std.top {{\n{QUARTER}  a := point\n  fix((0mm, 0mm)) a\n  b := point\n  fix((40mm, 0mm)) b\n  g := line(a, b)\n  m := point\n  m coincident(t == s) g\n  n := point\n  n coincident(t == s) k\n  m distance(10mm, along: right) b\n}}\n"));
    // m stands 10 short of b: three quarters of the way along, and so round the arc
    near(at(&e, "n"), on_quarter(30.0 + 0.75 * 90.0), "n");
    let (prog, errs) = parse("\
unit mm
use std
param s: Scalar hint(0.5)
in std.top {
  a := point
  b := point hint((40mm, 0mm))
  g := line(a, b)
  p0 := point hint((0mm, 0mm))
  p1 := point hint((10mm, 10mm))
  p2 := point hint((20mm, 0mm))
  p3 := point hint((30mm, 10mm))
  sp := spline(p0, p1, p2, p3)
  m := point
  m coincident(t == s) g
  n := point
  n coincident(t == s) sp
}
");
    assert!(errs.is_empty(), "{errs:?}");
    let e = elaborate(&prog);
    assert!(e.errors().any(|d| d.message.contains("cannot share it")), "{:?}",
        e.errors().map(|d| &d.message).collect::<Vec<_>>());
}

/// **Unpinned, `t` is the contact's own unknown**, seeded where the point stands: the point keeps
/// the one freedom a point on a line has, and its place is read as the fraction it comes to.
#[test]
fn an_unpinned_fraction_is_the_points_own_unknown() {
    let mut e = solved("\
unit mm
use std
in std.top {
  a := point
  b := point
  fix((0mm, 0mm)) a
  fix((20mm, 0mm)) b
  g := line(a, b)
  m := point hint((5mm, 2mm))
  m coincident g hint(t: 0.6)
}
");
    assert_eq!(of_kind(&e, CKind::PointAlongLine), 1, "a seed for `t` is `t` written");
    assert_eq!(dof(&mut e), 1, "a point on a line slides along it");
    let m = at(&e, "m");
    assert!(m[1].abs() < 1e-9, "on the line: {m:?}");
    assert!((place(&e, CKind::PointAlongLine) - m[0] / 20.0).abs() < 1e-9, "its place is where it is");
    // and seeded off its own place, a fraction starts where the point stands
    let e = build("\
unit mm
use std
in std.top {
  a := point
  b := point
  fix((0mm, 0mm)) a
  fix((20mm, 0mm)) b
  g := line(a, b)
  m := point hint((15mm, 3mm))
  m coincident(t == s) g
}
param s: Scalar
");
    let p = e.sketch.shared["s"].param;
    assert!((e.sketch.params[p as usize].value - 0.75).abs() < 1e-12, "the foot of (15, 3)");
}

/// **An arc's fraction is held to the arc**: seeded past its end it is brought back onto it before
/// the solve, as a spline's contact is to its knots, and left free to find its place.
#[test]
fn an_arc_fraction_seeded_off_the_arc_is_brought_onto_it() {
    let e = solved("\
unit mm
use std
in std.top {
  c := point
  fix((0mm, 0mm)) c
  e := point
  fix((10mm, 0mm)) e
  q := point
  fix((0mm, 10mm)) q
  k := arc(center: c, start: e, end: q)
  n := point hint((5mm, 8mm))
  n coincident k hint(t: 1.7)
  fix(x == 5mm) n
}
");
    let t = place(&e, CKind::PointAlongArc);
    assert!((t - 2.0 / 3.0).abs() < 1e-9, "at 60° of a quarter turn: {t}");
    near(at(&e, "n"), [5.0, 75f64.sqrt(), 0.0], "n");
}

/// **In space the same**: a point in space and a point drawn in another view at a fraction of a
/// line and an arc drawn in `std.top` read the relation in space, the place carried over.
#[test]
fn a_fraction_reads_in_space_across_views() {
    let e = solved("\
unit mm
use std
in std.top {
  c := point
  fix((0mm, 0mm)) c
  e := point
  fix((10mm, 0mm)) e
  q := point
  fix((0mm, 10mm)) q
  k := arc(center: c, start: e, end: q)
  a := point
  fix((20mm, 20mm)) a
  g := line(c, a)
}
m := point hint((1mm, 2mm, 3mm))
m coincident(t == 0.5) k
n := point hint((1mm, 2mm, 3mm))
n coincident(t == 0.25) g
in std.front {
  f := point hint((3mm, 1mm))
  f coincident(t == 0) g
}
");
    let h = 50f64.sqrt();
    near(at(&e, "m"), [h, h, 0.0], "m");
    near(at(&e, "n"), [5.0, 5.0, 0.0], "n");
    near(at(&e, "f"), [0.0, 0.0, 0.0], "f");
    assert_eq!(of_kind(&e, CKind::PointAlongArc3), 1);
    assert_eq!(of_kind(&e, CKind::PointAlongLine3), 2);
}

/// **In a traced body** the place is the block's to state over the swept formal, read through a
/// tape as a dimension over `u` is: here the point runs `u²` of the way along its line.
#[test]
fn a_traced_point_runs_along_its_line_as_its_pin_says() {
    for (u, want) in [(0.5, [10.0, 5.0, 0.0]), (0.8, [25.6, 12.8, 0.0])] {
        let e = solved(&format!("\
unit mm
use std
component Slider(a: point, b: point, u: Scalar) {{
  g := line(a, b)
  m := point
  m coincident(t == u * u) g
}}
in std.top {{
  p := point
  fix((0mm, 0mm)) p
  q := point
  fix((40mm, 20mm)) q
  k := point
}}
path := Slider(p, q).m over u in (0, 1)
k coincident(t == {u}) path
"));
        near(at(&e, "k"), want, &format!("u = {u}"));
    }
}

/// A traced body states every number it holds: a fraction left to the solve is refused there.
#[test]
fn a_trace_infers_no_fraction() {
    let (prog, errs) = parse("\
unit mm
use std
component Slider(a: point, b: point, u: Scalar) {
  g := line(a, b)
  m := point
  m coincident g hint(t: 0.5)
  a distance(u) m
}
in std.top {
  p := point
  q := point
}
path := Slider(p, q).m over u in (0mm, 1mm)
");
    assert!(errs.is_empty(), "{errs:?}");
    let e = elaborate(&prog);
    assert!(e.errors().any(|d| d.message.contains("`t` must be pinned in a trace block")),
        "{:?}", e.errors().map(|d| &d.message).collect::<Vec<_>>());
}

/// A fraction travels: the program lifted from the drawing and its JSON keep the kind and the
/// pinned place.
#[test]
fn a_fraction_is_kept_by_the_lift_and_the_document() {
    let e = solved(&format!("unit mm\nuse std\nin std.top {{\n{QUARTER}  m := point\n  m coincident(t == 0.5) k\n  a := point\n  fix((0mm, 0mm)) a\n  g := line(a, c)\n  n := point\n  n coincident(t == 0.25) g\n}}\n"));
    let lifted = gcs_core::program::to_program(&e.sketch).text().to_string();
    let again = solved(&lifted);
    for kind in [CKind::PointAlongArc, CKind::PointAlongLine] {
        assert_eq!(of_kind(&again, kind), 1, "{kind:?} in\n{lifted}");
        assert_eq!(place(&again, kind), place(&e, kind), "{lifted}");
    }
    let doc = gcs_core::io::from_json(&gcs_core::io::to_json(&e.sketch)).unwrap();
    for kind in [CKind::PointAlongArc, CKind::PointAlongLine] {
        let c = doc.constraints.iter().find(|c| c.kind == kind).expect("in the document");
        let (_, t) = c.fraction_contact().unwrap();
        assert!(doc.params[t as usize].fixed, "pinned");
    }
}

/* -- the station probes (`docs/lofts/l0/stations`), handed no run and no sweep -------------- */

const LOFTS: &str = include_str!("../../../docs/lofts/l0/stations/lofts.sv");

/// A probe linked as `solventc` links it: `lofts` beside it, then the library.
fn probe(src: &str) -> Elaborated {
    let (mut prog, errs) = parse(src);
    assert!(errs.is_empty(), "{errs:?}");
    let linked = gcs_core::modules::link(&mut prog, &mut |name| match name {
        "lofts" => Some(LOFTS.to_string()),
        _ => gcs_core::library::resolve(name),
    });
    assert!(linked.is_empty(), "{:?}", linked.iter().map(|d| &d.message).collect::<Vec<_>>());
    let mut e = elaborate(&prog);
    assert!(e.ok(), "{:?}", e.errors().map(|d| &d.message).collect::<Vec<_>>());
    let r = solve(&mut e.sketch, SolveOpts::default());
    assert!(r.success, "{}", r.message);
    e
}

fn station(e: &Elaborated) -> [f64; 3] {
    e.sketch.world_point(crate::common::at_path(e, "body.st.m").i())
}

/// The reducer's station halfway up its 40 mm guide, the guide's length passed nowhere.
#[test]
fn the_reducers_station_needs_no_run() {
    let src = include_str!("../../../docs/lofts/l0/stations/reducer.sv");
    assert!(!src.contains("run:") && !LOFTS.contains("run:"), "no run passed");
    let mut e = probe(src);
    near(station(&e), [0.0, 20.0, 0.0], "the station");
    assert_eq!(dof(&mut e), 0);
}

/// The elbow's station halfway round its quarter bend of radius 30, the sweep passed nowhere and
/// the station's point in space.
#[test]
fn the_elbows_station_needs_no_sweep() {
    let src = include_str!("../../../docs/lofts/l0/stations/elbow.sv");
    assert!(!src.contains("sweep:") && !LOFTS.contains("sweep:"), "no sweep passed");
    let mut e = probe(src);
    let h = 30.0 / 2f64.sqrt();
    near(station(&e), [h, h, 0.0], "the station");
    assert_eq!(dof(&mut e), 0);
}
