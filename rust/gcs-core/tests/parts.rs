//! **Parts** (#461, `docs/parts-plan.md` §3): `part Name(…) { … }` is a component declared a
//! function of what it is given, and the elaborator holds it to that over each instance — every
//! unknown it makes reached by its own rows, its inputs held (PT2, **E108**), and no row about its
//! operands alone (PT3, **E109**) — and then solves it flat, as any component.  `part` is a word
//! only where a definition's shape follows it.
use gcs_core::program::Elaborated;

use crate::common::{at, read};

/// `src` parsed and linked, and elaborated whatever it says.
fn elaborated(src: &str) -> Elaborated {
    crate::common::linked::build(src)
}

/// The errors of `src`, as `(code, the text they are shown at, message)`.
fn errors(src: &str) -> Vec<(String, String, String)> {
    let e = elaborated(src);
    e.errors().map(|d| (d.code.as_str().to_string(), d.span.slice(e.text()).to_string(),
        d.message.clone())).collect()
}

/// The common tangent of two circles, the left one from the first to the second, as a part:
/// two points of its own and four rows, the root chosen by `ccw` — the direction paper's noun.
const TANGENT: &str = "\
unit mm
use std
part CommonTangent(a: circle, b: circle) {
  in a.center.plane {
    s := point hint(at: a.center, toward: b.center, by: 0.1, turn: 90deg)
    e := point hint(at: b.center, toward: a.center, by: 0.1, turn: -90deg)
    run := line(s, e)
    s coincident a
    e coincident b
    run tangent(at: p1) a
    run tangent(at: p2) b
    ccw(a.center, b.center, s)
    ccw(a.center, b.center, e)
  }
}
in std.front {
  o1 := point
  o2 := point
  fix((0, 0)) o1
  fix((120, 0)) o2
  small := circle(center: o1)
  large := circle(center: o2)
  radius(20mm) small
  radius(45mm) large
}
upper := CommonTangent(small, large)
lower := CommonTangent(large, small)
";

/// **A part is a component, held to a promise**: one whose rows place everything it makes from
/// what it is given elaborates as a component does and is solved flat, both runs where the closed
/// form puts them — the small rim's tangent point at `r₁ (sin φ, cos φ)` above and below, with
/// `sin φ = (r₂ − r₁) / c`.
#[test]
fn a_part_whose_rows_place_what_it_makes_is_a_component() {
    let e = read(TANGENT);
    assert!(e.diags.is_empty(), "{:?}", e.diags);
    let mut e = e;
    assert!(gcs_core::solve::solve(&mut e.sketch, Default::default()).success);
    let s = (45.0_f64 - 20.0) / 120.0;
    let want = [-20.0 * s, 20.0 * (1.0 - s * s).sqrt()];
    for (n, sign) in [("upper.s", 1.0), ("lower.e", -1.0)] {
        let p = at(&e, n);
        assert!((p[0] - want[0]).abs() < 1e-7 && (p[2] - sign * want[1]).abs() < 1e-7,
            "{n}: {p:?} against {want:?}");
    }
}

/// **PT2: what a part makes, its own rows place** (E108): a tip held only at a distance from its
/// pin turns about it, and the part is refused at its definition naming the tip — once, however
/// many instances there are — with the two remedies.
#[test]
fn a_part_that_leaves_a_freedom_is_refused_naming_it() {
    let got = errors("\
unit mm
use std
part Loose(o: point) {
  in o.plane {
    tip := point hint((30, 0))
    o distance(30mm) tip
  }
}
in std.front {
  o := point
  fix((0, 0)) o
}
a := Loose(o)
b := Loose(o)
");
    assert_eq!(got.len(), 1, "{got:?}");
    let (code, shown, message) = &got[0];
    assert_eq!((code.as_str(), shown.as_str()), ("E108", "Loose"));
    assert!(message.contains("moving `a.tip`") && message.contains("make the freedom a formal"),
        "{message}");
}

/// **A freedom is exported as a formal** (FR2): the crank's angle, a formal the call leaves
/// unbound, is an input, so the part whose rows place its tip from it stands — while the same
/// body that leaves the angle to an instance inside it, unbound there, owns that number, and is
/// refused for leaving it free.
#[test]
fn a_freedom_is_a_formal_and_not_a_number_of_the_body() {
    let crank = |head: &str, angle: &str, call: &str| format!("\
unit mm
use std
component Turned(datum: line, arm: line, t: Angle) {{
  datum angle(t) arm
}}
part Crank(o: point, datum: line{head}) {{
  in o.plane {{
    tip := point hint((20, 20))
    arm := line(o, tip)
    o distance(30mm) tip
  }}
  turn := Turned(datum, arm, t: {angle})
}}
in std.front {{
  o := point
  fix((0, 0)) o
  d := line(o, hint((10, 0)))
  fix((10, 0)) d.p2
}}
c := {call}
");
    let given = read(&crank(", t: Angle", "t", "Crank(o, d, t: hint(30deg))"));
    assert!(given.diags.is_empty(), "{:?}", given.diags);
    let own = errors(&crank("", "hint(30deg)", "Crank(o, d)"));
    assert!(own.iter().any(|(code, shown, _)| code == "E108" && shown == "Crank"), "{own:?}");
}

/// **PT3: no fact about its operands alone** (E109): a row between two of the part's operands
/// is about what it is given — said at the row, which belongs where they are given — and a row
/// past what its own unknowns take is said so too.
#[test]
fn a_part_stating_a_fact_about_its_operands_is_refused_at_the_row() {
    let fused = errors("\
unit mm
use std
part Apex(a: point, b: point) {
  in a.plane {
    m := point hint((3, 4))
    m distance(5mm) a
    m distance(5mm) b
    a distance(6mm) b
  }
}
in std.front {
  o := point
  fix((0, 0)) o
  q := point hint((6, 0))
  fix(y == 0) q
}
x := Apex(o, q)
");
    assert_eq!(fused.len(), 1, "{fused:?}");
    assert_eq!((fused[0].0.as_str(), fused[0].1.as_str()), ("E109", "a distance(6mm) b"));
    assert!(fused[0].2.contains("a fact about what it is given"), "{}", fused[0].2);
    let over = errors("\
unit mm
use std
part Apex(a: point, b: point) {
  in a.plane {
    m := point hint((3, 4))
    m distance(5mm) a
    m distance(5mm) b
    m distance(5mm, along: up) a
  }
}
in std.front {
  o := point
  fix((0, 0)) o
  q := point
  fix((6, 0)) q
}
x := Apex(o, q)
");
    assert!(over.iter().any(|(code, _, m)| code == "E109" && m.contains("past what its own unknowns take")),
        "{over:?}");
}

/// **A parent's statement over a part's output is the parent's** (IO2): it is no row of the
/// part, so a part placed from outside is still refused for what it leaves free — while a part
/// whose outputs a parent relates further is accepted.
#[test]
fn a_parent_cannot_complete_a_part_and_may_relate_its_outputs() {
    let src = |part_rows: &str| format!("\
unit mm
use std
part Arm(o: point) {{
  in o.plane {{
    tip := point hint((30, 0))
    o distance(30mm) tip
    {part_rows}
  }}
}}
in std.front {{
  o := point
  fix((0, 0)) o
  r := point hint((40, 0))
}}
a := Arm(o)
in std.front {{
  r distance(10mm) a.tip
  fix(y == 0) r
}}
");
    let placed_outside = errors(&src(""));
    assert!(placed_outside.iter().any(|(code, _, m)| code == "E108" && m.contains("`a.tip`")),
        "{placed_outside:?}");
    let e = read(&src("fix(y == 0) tip"));
    assert!(e.diags.is_empty(), "{:?}", e.diags);
}

/// **`part` is a word only before a definition's shape** — `part Name(…) {` — so a document may
/// still call a thing `part`, read its members and apply a word to it.
#[test]
fn part_stays_a_name() {
    let e = read("\
unit mm
use std (horizontal)
a above(d) b := a distance(d, along: up) b
in std.front {
  part := point hint((0, 5))
  q := point
  fix((0, 0)) q
  part horizontal q
  q above(5mm) part
}
");
    assert!(e.diags.is_empty(), "{:?}", e.diags);
    let parsed = gcs_core::library::parse_linked("component C(p: point) { }\npart D(p: point) { }\n").0;
    let parts: Vec<bool> = parsed.components.iter().filter(|c| c.name.is_some()).map(|c| c.part).collect();
    assert_eq!(parts, [false, true]);
}

/// The diagnosis of `src` once solved, read through its parts (#461 D1–D3).
fn reading(src: &str) -> gcs_core::parts::Reading {
    let e = read(src);
    let mut sk = e.sketch.clone();
    let r = gcs_core::solve::solve(&mut sk, Default::default());
    let settled = r.settled();
    let d = gcs_core::diagnose::diagnose(&mut sk,
        gcs_core::diagnose::DiagnoseOptions { settled, ..Default::default() });
    gcs_core::parts::read(&e, &sk, &d, settled)
}

/// An arm whose tip a parent holds 10 from a point 45 out: the arm places its tip 30 along from
/// its pin, so the parent's statement is the one that cannot hold.
const HELD_FROM_OUTSIDE: &str = "\
unit mm
use std (horizontal)
part Arm(o: point) {
  in o.plane {
    tip := point hint((30, 0))
    o distance(30mm) tip
    o horizontal tip
  }
}
in std.front {
  o := point
  fix((0, 0)) o
  r := point hint((40, 0))
  fix((45, 0)) r
}
a := Arm(o)
in std.front {
  r distance(10mm) a.tip
}
";

/// **D1: an over-constraint through a part's outputs is the parent's.**  The conflict holds the
/// arm's row and the parent's; the reading names the parent's as the one to remove, the part's as
/// the reason, and what the part is given — and does not say the part failed as well.
#[test]
fn an_over_constraint_through_a_part_is_said_as_the_parents() {
    let r = reading(HELD_FROM_OUTSIDE);
    assert_eq!((r.instances, r.definitions), (1, 1));
    assert_eq!(r.over, ["`r distance(10mm) a.tip` over-constrains part `a` (`Arm`): what it makes \
        is placed by what it is given (`o`), so remove this, not `o distance(30mm) a.tip`"]);
    assert!(r.unmet.is_empty(), "{:?}", r.unmet);
}

/// **D2: a part that does not hold is named, with what it is given.**  A point 5 from two points
/// 12 apart has no pose: the solve stops with the part's rows unmet, and the reading names the
/// part, its rows and where its operands stood.
#[test]
fn a_part_that_does_not_hold_is_named_with_its_inputs() {
    let r = reading("\
unit mm
use std
part Apex(a: point, b: point) {
  in a.plane {
    m := point hint((5, 0.5))
    m distance(5mm) a
    m distance(5mm) b
  }
}
in std.front {
  o := point
  fix((0, 0)) o
  q := point
  fix((12, 0)) q
}
x := Apex(o, q)
");
    assert!(r.over.is_empty(), "{:?}", r.over);
    assert_eq!(r.unmet.len(), 1, "{:?}", r.unmet);
    assert!(r.unmet[0].starts_with("part `x` (`Apex`) does not hold where the solve stopped") &&
        r.unmet[0].contains("`o` at (0, 0, 0), `q` at (12, 0, 0)"), "{}", r.unmet[0]);
}

/// **D3: a part singular at what it is given.**  Its rows reach both numbers of its point, so it
/// is a part; but with its two operands one point its two rows are one, and at the solved pose
/// they place one number of two — said, with the rank.  Apart, it is not.
#[test]
fn a_part_singular_at_its_inputs_is_said_with_its_rank() {
    let apex = |q: &str| format!("\
unit mm
use std
part Apex(a: point, b: point) {{
  in a.plane {{
    m := point hint((3, 4))
    m distance(5mm) a
    m distance(5mm) b
  }}
}}
in std.front {{
  o := point
  fix((0, 0)) o
  q := point
  fix({q}) q
}}
x := Apex(o, q)
");
    let one = reading(&apex("(0, 0)"));
    assert_eq!(one.singular.len(), 1, "{one:?}");
    assert!(one.singular[0].contains("its rows place 1 of its 2 own unknowns"), "{}", one.singular[0]);
    assert!(reading(&apex("(6, 0)")).singular.is_empty());
}

/// **PT5, which root a part takes is stated** (W207, by `--lint`): a point 5 from two points 6
/// apart has two poses; the part that leaves the choice to its seed is named, and the ones that
/// state it, with `ccw` or with a bound (the hub motor's slot corners, ahead of the axle), are not.
/// Nor is an axis held along a line that says which way it runs (#535), where one that does not
/// is named: a point 5 along it stands on the line's side or the other.
#[test]
fn a_part_that_leaves_its_root_to_its_seeds_is_named() {
    let e = read("\
unit mm
use std
part Apex(a: point, b: point) {
  in a.plane {
    m := point hint((3, 4))
    m distance(5mm) a
    m distance(5mm) b
  }
}
part Chosen(a: point, b: point) {
  in a.plane {
    m := point hint((3, 4))
    m distance(5mm) a
    m distance(5mm) b
    ccw(a, b, m)
  }
}
part Bounded(a: point, b: point) {
  in a.plane {
    m := point hint((3, 4))
    m distance(5mm) a
    m distance(5mm) b
    a distance(>= 0, along: v) m
  }
}
in std.front {
  o := point
  fix((0, 0)) o
  q := point
  fix((6, 0)) q
}
part Unsensed(a: point, b: point) {
  l := line(a, b)
  t := axis hint(dir: (0.3, 0.9, 0.3))
  t parallel l
  a coincident t
  m := point hint((3, 3, 3))
  m coincident t
  a distance(5mm, along: t) m
}
part Sensed(a: point, b: point) {
  l := line(a, b)
  t := axis hint(dir: (0.3, 0.9, 0.3))
  t parallel(sense: same) l
  a coincident t
  m := point hint((3, 3, 3))
  m coincident t
  a distance(5mm, along: t) m
}
x := Apex(o, q)
y := Chosen(o, q)
z := Bounded(o, q)
w := Unsensed(o, q)
v := Sensed(o, q)
");
    let mut sk = e.sketch.clone();
    assert!(gcs_core::solve::solve(&mut sk, Default::default()).success);
    let said: Vec<(String, String)> = gcs_core::parts::roots(&e, &sk).iter()
        .map(|d| (d.code.as_str().to_string(), d.span.slice(e.text()).to_string()))
        .collect();
    assert_eq!(said, [("W207".to_string(), "Apex".to_string()),
                      ("W207".to_string(), "Unsensed".to_string())]);
}

/// **The engine states every root its parts take** (#503): `--lint` names none of its parts, and
/// each choice it states holds through the turn — at the dead centres of both revolutions of the
/// cycle, where the pins stand in line with the bores, and between them.
#[test]
fn the_engine_states_the_root_of_every_part() {
    let engine = |cycle: &str| {
        let (mut prog, errs) = gcs_core::syntax::parse(gcs_core::examples::source("engine").unwrap());
        let linked = gcs_core::modules::link(&mut prog, &mut |name| {
            let text = gcs_core::library::resolve(name)?;
            Some(match name {
                "engine.dims" => text.replace("cycle := 400deg", &format!("cycle := {cycle}")),
                _ => text,
            })
        });
        assert!(errs.is_empty() && linked.is_empty(), "{errs:?} {linked:?}");
        let e = gcs_core::program::elaborate(&prog);
        assert!(e.ok(), "{:?}", e.diags);
        let mut sk = e.sketch.clone();
        let r = gcs_core::solve::solve(&mut sk, Default::default());
        assert!(r.success, "at {cycle}: {}", r.message);
        assert!(gcs_core::solve::orientations_broken(&sk).is_empty(), "at {cycle}");
        (e, sk)
    };
    let (e, sk) = engine("400deg");
    let said: Vec<String> = gcs_core::parts::roots(&e, &sk).iter().map(|d| d.message.clone())
        .collect();
    assert!(said.is_empty(), "{said:#?}");
    for cycle in ["0deg", "90deg", "180deg", "360deg", "540deg", "630deg"] {
        engine(cycle);
    }
}

/// **What a lift reads is its point's and its plane's** (#501): a point drawn in a plane and read
/// in space gets a hidden point, held to it by a row over the plane's axes and origin, and the
/// plane's axes are numbers of their own.  None of that is the instance's that happened to read
/// it.  `Mark` measures along a frame it turns through its operands, so their hidden points are
/// read; `two` turns its frame through `one`'s mark, which `one` lifted for itself, and `disc`
/// draws in a frame the parent turned, its own frame turned in that one.  Every instance is a
/// part: the parent's frame is placed by the parent's rows, `one`'s mark by `one`'s.
#[test]
fn a_part_reading_its_operands_in_space_is_given_their_lifts() {
    let e = read("\
unit mm
use std
part Mark(a: point, b: point) {
  in a.plane {
    t := std.Turned(a, b)
    m := point hint(at: t.axes, (4mm, 1mm))
    m distance(4mm, along: u) t.axes
    m distance(1mm, along: v) t.axes
  }
}
part Disc(f: group) {
  in f.axes {
    se := point hint((3mm, -4mm))
    f.axes.origin distance(3mm, along: u) se
    se distance(4mm, along: v) f.axes.origin
    screw := Mark(f.axes.origin, se)
  }
}
in std.front {
  o := point
  fix((0, 0)) o
  q := point
  fix((10, 10)) q
  f := std.Turned(o, q)
}
one := Mark(o, q)
two := Mark(one.m, o)
disc := Disc(f)
");
    assert!(e.diags.is_empty(), "{:?}", e.diags.iter().map(|d| &d.message).collect::<Vec<_>>());
    let mut e = e;
    assert!(gcs_core::solve::solve(&mut e.sketch, Default::default()).success);
    // `one`'s mark 4 along the line from `o` to `q` and 1 to its left
    let (c, s) = (0.5_f64.sqrt(), 0.5_f64.sqrt());
    let m = at(&e, "one.m");
    assert!((m[0] - (4.0 * c - s)).abs() < 1e-9 && (m[2] - (4.0 * s + c)).abs() < 1e-9, "{m:?}");
}
