//! **A document's values, read** (#596): `ratio := teeth / 12 * 4`, worked out while elaborating,
//! is carried out of the walk by its absolute name (`Elaborated::values`) and said in the unit it
//! reads in (`report::value_text`): `torque = 0.0864 N·m` in every document's lengths.  And **a
//! claim over values** (`claim torque >= 2Nm`) is judged there, before the solve — a theorem or
//! refuted, never consuming — or refused saying why it cannot be decided.

use crate::common::linked::build as elaborated;
use gcs_core::program::{Code, Elaborated};
use gcs_core::report::{value_claim_text, value_text};
use gcs_core::syntax::{parse, StmtKind};
use gcs_core::units::Dim;

fn ok(src: &str) -> Elaborated {
    let e = elaborated(src);
    assert!(e.ok(), "{:?}", e.errors().map(|d| d.message.clone()).collect::<Vec<_>>());
    e
}

/// What `name` is said to be, or `None` where the document names no number there.
fn said(e: &Elaborated, name: &str) -> Option<String> {
    e.values.get(name).map(|v| value_text(e.sketch.units, name, v))
}

/// The issue's drive table, and a point so there is a drawing.
const DRIVE: &str = "\
use std
teeth := 48
ratio := teeth / 12 * 4
torque := 0.006Nm * ratio * 0.9
w := 60
area := 12mm * 10mm
spin := 30rpm
inertia := 2kg * (1m)^2
dims := {bore: 16mm, depth: 2 * 8mm}
in std.front {
  a := point
  fix((0, 0)) a
}
";

/// **A value is said in its unit**: a plain number bare, a length and an area in the document's
/// unit, an angle's rate in degrees a second, and a design table's figures in SI — by the unit's
/// name where SI has one, by its bases where not — the same figure whatever the document's
/// lengths are in.
#[test]
fn a_documents_values_are_said_in_their_units() {
    let units = [("", "mm"), ("unit mm\n", "mm"), ("unit m\n", "m"), ("unit in\n", "in")];
    for (unit, length) in units {
        let e = ok(&format!("{unit}{DRIVE}"));
        let v = &e.values["ratio"];
        assert_eq!((v.value, v.dim), (16.0, Dim::SCALAR), "{unit:?}");
        for (name, want) in [
            ("ratio", "ratio = 16".to_string()),
            ("torque", "torque = 0.0864 N·m".to_string()),
            ("w", "w = 60".to_string()),
            ("area", format!("area = {} {length}^2", fig(120.0 * mm(length).powi(2)))),
            ("spin", "spin = 180 deg/s".to_string()),
            ("inertia", "inertia = 2 m^2·kg".to_string()),
            ("dims.bore", format!("dims.bore = {} {length}", fig(16.0 * mm(length)))),
        ] {
            assert_eq!(said(&e, name).as_deref(), Some(want.as_str()), "{unit:?} {name}");
        }
    }
}

/// A number to the figures a value is said to.
fn fig(v: f64) -> String {
    gcs_core::json::fmt_g(v, gcs_core::expr::READING_SIG)
}

/// How many of the document's lengths a millimetre is.
fn mm(length: &str) -> f64 {
    match length {
        "mm" => 1.0,
        "m" => 1e-3,
        _ => 1.0 / 25.4,
    }
}

/// **Only a number is a value**: one written over an unknown of the drawing is the solve's to
/// give, a parameter's axis runs, and neither is here; an instance's numbers are, under its name.
#[test]
fn a_value_is_a_number_the_walk_worked_out() {
    let e = ok("\
use std
param beta: Angle hint(30deg)
k := beta * 2
component Tab(n: Scalar) {
  m := n * 2
  q := point
  fix((n, 0)) q
}
in std.front {
  t1 := Tab(n: 3)
  a := point
  fix((0, 0)) a
  l := line(a, hint((10, 5)))
  g := horizontal line(a, hint((10, 0)))
  g angle(beta) l
}
");
    assert_eq!(said(&e, "k"), None);
    assert_eq!(said(&e, "beta"), None);
    assert_eq!(said(&e, "t1.m").as_deref(), Some("t1.m = 6"));
}

/// **A claim over values is judged before the solve**: each a theorem or refuted, said with the
/// values it reads — an instance's by their absolute names, once per instance — and an equality
/// kept within a hair of rounding, as a requirement is.
#[test]
fn a_claim_over_values_is_a_theorem_or_refuted() {
    let e = ok("\
use std
teeth := 48
ratio := teeth / 12 * 4
torque := 0.006Nm * ratio * 0.9
claim torque >= 2Nm
claim ratio == 16
claim torque == 0.0864Nm
claim 2 * ratio > 30 and not (teeth < 12)
component Tab(n: Scalar) {
  m := n * 2
  claim m >= 10
  q := point
  fix((n, 0)) q
}
in std.front {
  t1 := Tab(n: 3)
  t2 := Tab(n: 30)
}
");
    let units = e.sketch.units;
    let texts: Vec<String> = e.value_claims.iter().map(|c| value_claim_text(units, c)).collect();
    assert_eq!(texts, [
        "claim torque >= 2Nm — refuted: torque = 0.0864 N·m",
        "claim ratio == 16 — theorem: ratio = 16",
        "claim torque == 0.0864Nm — theorem: torque = 0.0864 N·m",
        "claim 2 * ratio > 30 and not (teeth < 12) — theorem: ratio = 16, teeth = 48",
        "claim m >= 10 — refuted: t1.m = 6",
        "claim m >= 10 — theorem: t2.m = 60",
    ]);
    assert_eq!(e.value_claims.iter().map(|c| c.holds).collect::<Vec<_>>(),
               [false, true, true, true, false, true]);
}

/// **A claim that cannot be decided says why, where it stands**: one reading an unknown of the
/// drawing is E040 (a relation claims what the solve gives), a name that is no number E101, two
/// dimensions compared E103.
#[test]
fn a_claim_over_values_that_cannot_be_decided_says_why() {
    for (claim, code, words) in [
        ("claim k > 0deg", Code::E040, "`k` reads `beta`, an unknown of the drawing"),
        ("claim beta > 0deg", Code::E040, "`beta` is an unknown of the drawing"),
        ("claim nothing > 0", Code::E101, "`nothing` is not a number here"),
        ("claim w > 10deg", Code::E103, "Scalar cannot be compared with Angle"),
    ] {
        let e = elaborated(&format!("\
use std
param beta: Angle hint(30deg)
k := beta * 2
w := 60
{claim}
in std.front {{
  a := point
  fix((0, 0)) a
  l := line(a, hint((10, 5)))
  g := horizontal line(a, hint((10, 0)))
  g angle(beta) l
}}
"));
        let d = e.errors().find(|d| d.code == code)
            .unwrap_or_else(|| panic!("{claim}: {:?}", e.errors().collect::<Vec<_>>()));
        assert!(d.message.contains(words), "{claim}: {}", d.message);
        assert!(e.value_claims.is_empty(), "{claim}");
    }
}

/// **A claim over values is told from a relation claim** by what follows its first reference —
/// arithmetic or a comparison where a relation has its word — and is printed back as written.
#[test]
fn a_claim_over_values_parses_apart_from_a_relation() {
    let (p, errs) = parse("\
w := 60
claim w >= 10
claim (w + 1) * 2 != 3
claim sqrt(w) > 7
claim -w < 0
l := line
claim horizontal l
claim l.p1 coincident l.p2
");
    assert!(errs.is_empty(), "{errs:?}");
    let root = &p.root().body;
    let texts: Vec<&str> = root.iter().filter_map(|st| match &st.kind {
        StmtKind::ValueClaim(c) => Some(c.text.as_str()),
        _ => None,
    }).collect();
    assert_eq!(texts, ["w >= 10", "(w + 1) * 2 != 3", "sqrt(w) > 7", "-w < 0"]);
    let relations = root.iter().filter(|st| matches!(&st.kind, StmtKind::Relation(r) if r.claim));
    assert_eq!(relations.count(), 2);
    let mut printed = String::new();
    gcs_core::syntax::write_stmt_to(&mut printed, &root[1].kind).unwrap();
    assert_eq!(printed, "claim w >= 10");
}
