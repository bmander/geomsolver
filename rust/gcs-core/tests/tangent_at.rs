//! **Any path, and the tangent at a point of it** (#251, lofts L0 finding 5).
//!
//! `path` is a formal type: a line, an arc, a circle, a spline or a curve, what the call gives it
//! judged by its kind (E103). `g tangent(at: m) l` is the line along `g` where a relation already
//! holds `m` on it: of a spline or a curve one row reading the place of the contact holding `m`
//! (#70's shared place, never a second unknown), of a circle or an arc the radius to `m` square
//! to the line, of a line `parallel`. So one station word stands over any guide (`std.On`,
//! drafted in L0's `pending.sv`, removed at L8), each station solved at DOF 0 with every equation
//! independent — where the tangency pinned beside the contact restates the point.

use crate::common::{read, refused};
use gcs_core::constraints::CKind;
use gcs_core::diagnose::{diagnose, DiagnoseOptions};
use gcs_core::program::Elaborated;
use gcs_core::solve::{solve, SolveOpts};

/// The guides: a cubic Bézier, an arc of radius 30 and a straight line, in `std.top`.
const GUIDES: &str = "\
unit mm
use std
in std.top {
  k0 := point
  k1 := point
  k2 := point
  k3 := point
  fix((0mm, 0mm)) k0
  fix((0mm, 20mm)) k1
  fix((20mm, 30mm)) k2
  fix((40mm, 30mm)) k3
  g := spline(k0, k1, k2, k3)
  c := point
  e := point
  fix((0mm, -50mm)) c
  fix((30mm, -50mm)) e
  a := arc(center: c, start: e) hint(r: 30)
  a.end distance(40mm) e
  l0 := point
  l1 := point
  fix((-20mm, 10mm)) l0
  fix((-20mm, 60mm)) l1
  l := line(l0, l1)
}
";

/// `std.On` as #251's spelling said it of a point drawn in the guide's view (#248, #249 were its
/// own issues): the plane square to any path `g` where `m` is on it.
const ON: &str = "\
component On(g: path, view: plane, s: Scalar) {
  in view {
    m := point
    m coincident(t == s) g
    q := point
    tl := line(m, q)
  }
  g tangent(at: m) tl
  length(10mm) tl
}
";

/// Solved, then what the diagnosis says: its freedom, and how many equations say nothing the
/// others do not, by the numeric rank.
fn solved(e: &mut Elaborated) -> (i64, usize) {
    let r = solve(&mut e.sketch, SolveOpts::default());
    assert!(r.success, "{}", r.message);
    let d = diagnose(&mut e.sketch, DiagnoseOptions::default());
    (d.dof, d.n_equations - d.numeric_rank.unwrap())
}

fn xy(e: &Elaborated, n: &str) -> (f64, f64) {
    e.sketch.point_xy(e.map.ent_named(n).unwrap_or_else(|| panic!("no `{n}`")).i())
}

/// `q − m` along `(tx, ty)`, to the solve's tolerance.
fn along(e: &Elaborated, m: &str, q: &str, (tx, ty): (f64, f64)) {
    let (m, q) = (xy(e, m), xy(e, q));
    let (dx, dy) = (q.0 - m.0, q.1 - m.1);
    let cross = (dx * ty - dy * tx) / (dx.hypot(dy) * tx.hypot(ty));
    assert!(cross.abs() < 1e-9, "({dx}, {dy}) is not along ({tx}, {ty})");
}

/// **One station word over a line, an arc and a spline**, each placed at its station, each
/// solved at DOF 0 with every equation independent; the tangent at each is the guide's own.
#[test]
fn one_station_word_stands_over_any_path() {
    let src = format!(
        "{GUIDES}{ON}st := On(g, std.top, s: 0.5)\nsa := On(a, std.top, s: 0.5)\n\
         sl := On(l, std.top, s: 0.5)\n"
    );
    let mut e = read(&src);
    assert_eq!(solved(&mut e), (0, 0));
    // the Bézier at t = ½ is (12.5, 22.5), its tangent (60, 40)
    let m = xy(&e, "st.m");
    assert!((m.0 - 12.5).abs() < 1e-9 && (m.1 - 22.5).abs() < 1e-9, "{m:?}");
    along(&e, "st.m", "st.q", (60.0, 40.0));
    // the arc's tangent is square to its radius
    let (c, m) = (xy(&e, "c"), xy(&e, "sa.m"));
    along(&e, "sa.m", "sa.q", (-(m.1 - c.1), m.0 - c.0));
    // and the line's is the line
    along(&e, "sl.m", "sl.q", (0.0, 1.0));
    let kinds: Vec<CKind> = e.sketch.constraints.iter().map(|c| c.kind).collect();
    for k in [CKind::Rate, CKind::TangentCircleAt, CKind::Parallel] {
        assert!(kinds.contains(&k), "{k:?} in {kinds:?}");
    }
}

/// The issue's own station: the point pinned at `t == ½` and the tangent at it — what
/// `stations/spline_station.sv` read as DOF 1 and a dependency, spelled `g tangent(t == s) tl`.
#[test]
fn the_tangent_at_a_pinned_place_is_regular_where_the_pinned_tangency_is_not() {
    let station = |tangency: &str| {
        format!(
            "{GUIDES}in std.top {{\n  m := point\n  m coincident(t == 0.5) g\n  q := point\n  \
             tl := line(m, q)\n  {tangency}\n  length(10mm) tl\n}}\n"
        )
    };
    let mut e = read(&station("g tangent(at: m) tl"));
    assert_eq!(solved(&mut e), (0, 0));
    along(&e, "m", "q", (60.0, 40.0));
    let mut e = read(&station("g tangent(t == 0.5) tl"));
    let (_, dependent) = solved(&mut e);
    assert_eq!(dependent, 1, "the pinned tangency restates the point");
}

/// **A line and a curve touch whichever is written first**: `tl tangent g` is `g tangent tl`,
/// and `tl tangent(at: m) g` is `g tangent(at: m) tl` — one kind each, the curve first.
#[test]
fn a_tangency_between_a_line_and_a_curve_reads_either_way_round() {
    let kinds = |tangency: &str| {
        let src = format!(
            "{GUIDES}in std.top {{\n  m := point hint((12, 22))\n  m coincident g\n  \
             q := point hint((20, 28))\n  tl := line(m, q)\n  {tangency}\n}}\n"
        );
        let e = read(&src);
        let at = |k: CKind| e.sketch.constraints.iter().filter(|c| c.kind == k).count();
        (at(CKind::SplineTangentLine), at(CKind::Rate))
    };
    assert_eq!(kinds("g tangent tl"), (1, 0));
    assert_eq!(kinds("tl tangent g"), (1, 0));
    assert_eq!(kinds("g tangent(at: m) tl"), (0, 1));
    assert_eq!(kinds("tl tangent(at: m) g"), (0, 1));
}

/// The tangent's place is the contact's, whichever is stated first, and travels so: described
/// as written, lifted to a program that elaborates to the same drawing, and through JSON.
#[test]
fn the_tangent_at_a_point_travels_with_its_contact() {
    let src = format!(
        "{GUIDES}in std.top {{\n  m := point hint((12, 22))\n  q := point hint((20, 28))\n  \
         tl := line(m, q)\n  g tangent(at: m) tl\n  m coincident g\n  length(10mm) tl\n  \
         k0 distance(22.5mm, along: up) m\n}}\n"
    );
    let mut e = read(&src);
    let shared = |sk: &gcs_core::model::Sketch| {
        let owned: Vec<u32> = sk.constraints.iter().flat_map(|c| c.aux_params()).collect();
        owned.len() == 2 && owned[0] == owned[1]
    };
    assert!(shared(&e.sketch));
    assert_eq!(solved(&mut e), (0, 0));
    let said: Vec<String> = e
        .sketch
        .user_constraints()
        .iter()
        .map(|c| gcs_core::io::describe_with(c, &|x| e.map.name_of(x).cloned()))
        .collect();
    assert!(said.iter().any(|s| s == "g tangent(at: m) tl"), "{said:?}");
    // a sketch read back from its JSON shares the place again
    let mut back = gcs_core::io::loads(&gcs_core::io::dumps(&e.sketch, None)).unwrap();
    assert!(shared(&back));
    assert!(solve(&mut back, SolveOpts::default()).success);
    // and a lifted program states it as written, and elaborates to the drawing it came from
    // (every name its own: `m` and `q` are the ninth and tenth points, `g` the first spline)
    let lifted = gcs_core::program::to_program(&e.sketch).text().to_string();
    assert!(lifted.contains("s0 tangent(at: p8) l1"), "{lifted}");
    let mut again = read(&lifted);
    assert!(shared(&again.sketch));
    assert_eq!(solved(&mut again), (0, 0));
    along(&again, "p8", "p9", (60.0, 40.0));
}

/// A `path` is given something a point runs along; anything else is refused at the argument.
#[test]
fn a_path_is_given_something_to_run_along() {
    let src = format!("{GUIDES}{ON}st := On(k0, std.top, s: 0.5)\n");
    refused(&src, "E103", "`g` is a path", "k0");
}

/// The tangent at a point reads the contact holding the point: with none, it says which to write.
#[test]
fn a_tangent_at_a_point_no_contact_holds_is_refused() {
    let src = format!(
        "{GUIDES}in std.top {{\n  m := point hint((12, 22))\n  q := point\n  tl := line(m, q)\n  \
         g tangent(at: m) tl\n}}\n"
    );
    refused(&src, "E040", "state `m coincident g`", "g tangent(at: m) tl");
}

/// **`length` measures any path** (#269): a word over `g: path` states `g`'s length whatever it
/// is given — a line's the distance between its ends, an arc's along itself, a spline's along the
/// whole of it — where through 0.89 a line's was `distance` and the word could not be written.
#[test]
fn length_of_any_path() {
    const PATHS: &str = "\
unit mm
use std
in std.top {
  l0 := point
  fix((0mm, 0mm)) l0
  l1 := point hint((30, 5))
  l := line(l0, l1)
  c := point
  e := point
  fix((100mm, 0mm)) c
  fix((130mm, 0mm)) e
  a := arc(center: c, start: e) hint(r: 30)
  k0 := point
  fix((0mm, 50mm)) k0
  k1 := point hint((0, 70))
  k2 := point hint((20, 80))
  k3 := point hint((40, 80))
  g := spline(k0, k1, k2, k3)
}
";
    // a relation word, which settles by what it is given — and not a definition, whose rows
    // determine what it makes and say nothing of what it is given (#562)
    refused(&format!("{PATHS}component Span(g: path, d: Length) {{\n  length(d) g\n}}\n\
                      sl := Span(l, d: 40mm)\n"), "E109", "a fact about what it is given",
            "length(d) g");
    let uses = [
        "stretch(d) g := length(d) g\nstretch(d: 40mm) l\nstretch(d: 40mm) a\nstretch(d: 80mm) g\n",
    ];
    for u in uses {
        let mut e = read(&format!("{PATHS}{u}"));
        let kinds: Vec<CKind> = e.sketch.constraints.iter().map(|c| c.kind).collect();
        for k in [CKind::Distance, CKind::ArcLength, CKind::SplineLength] {
            assert!(kinds.contains(&k), "{k:?} in {kinds:?}\n{u}");
        }
        let r = solve(&mut e.sketch, SolveOpts::default());
        assert!(r.success, "{}\n{u}", r.message);
        let (p, q) = (xy(&e, "l0"), xy(&e, "l1"));
        assert!(((q.0 - p.0).hypot(q.1 - p.1) - 40.0).abs() < 1e-9, "{p:?} {q:?}\n{u}");
        // the arc's end, 40 along a radius of 30 counter-clockwise from its start at angle 0
        let (c, m) = (xy(&e, "c"), xy(&e, "a.end"));
        let turn = (m.1 - c.1).atan2(m.0 - c.0).rem_euclid(std::f64::consts::TAU);
        assert!((30.0 * turn - 40.0).abs() < 1e-9, "{turn}\n{u}");
    }
}
