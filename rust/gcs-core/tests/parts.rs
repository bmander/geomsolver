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
