//! **Load scenarios** (#510 P4): `scenario NAME { … }` parses and elaborates into a
//! `model::Scenario` — its supports, loads and requirements, each target resolved by the
//! selectors every operand is resolved by and each number held to what it is — and adds nothing
//! to the drawing: the sketch a document with scenarios elaborates to is the one it does with
//! them deleted.
use gcs_core::model::{
    Applied, EntKind, FaceRef, Grade, Hold, LoadKind, LoadValue, Output, Scenario,
};
use gcs_core::expr::Cmp;
use gcs_core::program::{elaborate, Elaborated};
use gcs_core::syntax::{parse, StmtKind};

/// A plate 60 × 20 × `t`, with a point at its tip and an axis pointing down, and an unknown `u`,
/// the tip's distance from a corner.
const PLATE: &str = "\
unit mm
use std
t := 4mm
param u: Length hint(50mm)
in std.front {
  b0 := point
  b1 := point
  b2 := point
  b3 := point
  fix((0, 0)) b0
  fix((60, 0)) b1
  fix((60, 20)) b2
  fix((0, 20)) b3
  (south := line(b0, b1)) -> (east := line(b1, b2)) -> (north := line(b2, b3)) -> \
(west := line(b3, b0)) -> close
  outline := face(south, east, north, west)
  tip := point
  fix((60, 10)) tip
  b0 distance(u) tip
}
down := axis hint(dir: (0, -1, 0))
plate := solid(outline, depth: t)
";

/// The issue's three scenarios, over the plate.
const CASES: &str = "\
scenario lift {
  fixed plate.west
  load tip (0, -500N, 0)
  deflection(tip, along: down) <= 0.5mm
  safety(plate) == 2
}
scenario overload: lift {
  load tip 1kN along: down
  pressure plate.near 2MPa
  claim safety(plate) >= 1.25
}
scenario transport {
  pinned b0
  pinned b1 along: down
  accelerate plate (0, 0, -3 * 9.81m / 1s^2)
  claim mode(plate, 1) >= 120Hz
}
";

/// Parsed and linked, as a host reads it, and elaborated whatever it says.
fn read(src: &str) -> Elaborated {
    let (p, errs, linked) = gcs_core::library::parse_linked(src);
    assert!(errs.is_empty() && linked.is_empty(), "{errs:?} {linked:?}");
    elaborate(&p)
}

/// The errors a document elaborates with, each as its code (`syntax` for the parser's) and
/// message.
fn errors(src: &str) -> Vec<(String, String)> {
    let (p, errs, _) = gcs_core::library::parse_linked(src);
    let mut out: Vec<_> =
        errs.into_iter().map(|e| ("syntax".to_string(), e.message)).collect();
    let e = elaborate(&p);
    out.extend(e.errors().map(|d| (d.code.as_str().to_string(), d.message.clone())));
    out
}

fn scenario<'a>(e: &'a Elaborated, name: &str) -> &'a Scenario {
    &e.scenarios.iter().find(|s| s.scenario.name == name).expect("stated").scenario
}

fn ent(e: &Elaborated, name: &str) -> gcs_core::model::EntRef {
    crate::common::ent(e, name)
}

#[test]
fn a_scenario_holds_its_supports_loads_and_requirements() {
    let e = read(&format!("{PLATE}{CASES}"));
    assert!(e.ok(), "{:?}", e.errors().map(|d| &d.message).collect::<Vec<_>>());
    let names: Vec<&str> = e.scenarios.iter().map(|s| s.scenario.name.as_str()).collect();
    assert_eq!(names, ["lift", "overload", "transport"]);
    let (plate, tip, down) = (ent(&e, "plate"), ent(&e, "tip"), ent(&e, "down"));
    assert_eq!((plate.kind, tip.kind, down.kind), (EntKind::Solid, EntKind::Point, EntKind::Axis));
    let face = |path: &str| Applied::Face(FaceRef {
        solid: plate.i() as u32,
        path: vec![path.to_string()],
        surface: false,
        smooth: false,
    });

    let lift = scenario(&e, "lift");
    assert_eq!(lift.extends, None);
    assert_eq!(lift.supports.len(), 1);
    assert_eq!((lift.supports[0].hold, &lift.supports[0].at), (Hold::Fixed, &face("west")));
    // a force is kg·mm/s² in a millimetre document: 500 N is 5·10⁵ of them
    assert_eq!(lift.loads.len(), 1);
    assert_eq!(lift.loads[0].kind, LoadKind::Force);
    assert_eq!(lift.loads[0].at, Applied::Ent(tip));
    assert_eq!(lift.loads[0].value, LoadValue::Vector([0.0, -5e5, 0.0]));
    let [defl, safety] = &lift.requirements[..] else { panic!("{:?}", lift.requirements) };
    assert_eq!((defl.output, defl.of.clone(), defl.along), (Output::Deflection, Applied::Ent(tip), Some(down)));
    assert_eq!((defl.compare, defl.value, defl.grade()), (Cmp::Le, 0.5, Grade::Bound));
    assert_eq!((safety.output, safety.of.clone()), (Output::Safety, Applied::Ent(plate)));
    assert_eq!((safety.value, safety.grade()), (2.0, Grade::Driving));

    // an extended scenario's supports and loads are its base's, then its own; its requirements
    // are its own alone
    let over = scenario(&e, "overload");
    assert_eq!(over.extends, Some(0));
    assert_eq!(over.supports, lift.supports);
    assert_eq!(over.loads.len(), 3);
    assert_eq!(over.loads[0], lift.loads[0]);
    assert_eq!(over.loads[1].value, LoadValue::Along { magnitude: 1e6, along: down });
    assert_eq!(over.loads[2].kind, LoadKind::Pressure);
    assert_eq!(over.loads[2].at, face("near"));
    // a pascal is kg/(m·s²), and a megapascal 10³ kg/(mm·s²)
    assert_eq!(over.loads[2].value, LoadValue::Scalar(2e3));
    assert_eq!(over.requirements.len(), 1);
    assert_eq!((over.requirements[0].grade(), over.requirements[0].value), (Grade::Claim, 1.25));
    let stated = &e.scenarios[1];
    assert_eq!(stated.loads.iter().map(|l| l.stmt == e.scenarios[0].stmt).collect::<Vec<_>>(),
               [true, false, false], "the inherited load is written in its base");

    let transport = scenario(&e, "transport");
    let holds: Vec<_> = transport.supports.iter().map(|s| (s.hold, s.at.clone(), s.along)).collect();
    assert_eq!(holds, [
        (Hold::Pinned, Applied::Ent(ent(&e, "b0")), None),
        (Hold::Pinned, Applied::Ent(ent(&e, "b1")), Some(down)),
    ]);
    let LoadValue::Vector([0.0, 0.0, az]) = transport.loads[0].value else { panic!() };
    assert!((az + 3.0 * 9810.0).abs() < 1e-9, "{az} mm/s²");
    let [mode] = &transport.requirements[..] else { panic!() };
    assert_eq!((mode.output, mode.count, mode.value), (Output::Mode, Some(1), 120.0));
}

/// **A scenario adds nothing to the drawing**: no entity, no parameter, no row, and the drawing
/// solves as it does without one.
#[test]
fn a_scenario_adds_nothing_to_the_drawing() {
    let bare = read(PLATE);
    let with = read(&format!("{PLATE}{CASES}"));
    assert!(bare.ok() && with.ok());
    assert!(bare.scenarios.is_empty());
    let (a, b) = (&bare.sketch, &with.sketch);
    assert_eq!(a.params.len(), b.params.len());
    assert_eq!(a.constraints.len(), b.constraints.len());
    assert_eq!(a.points.len(), b.points.len());
    assert_eq!(a.solids.len(), b.solids.len());
    assert_eq!(gcs_core::io::dumps(a, None), gcs_core::io::dumps(b, None));
}

/// A scenario's names are the document's: a `param` written into its numbers, and a scenario
/// read in a `preview` as at the top.
#[test]
fn a_scenario_reads_the_documents_numbers() {
    let e = read(&format!("{PLATE}w := 250N\npreview {{\n  scenario half {{\n    \
                           load tip (0, -2 * w, 0)\n    claim deflection(tip) <= t / 8\n  }}\n}}\n"));
    assert!(e.ok(), "{:?}", e.errors().map(|d| &d.message).collect::<Vec<_>>());
    let half = scenario(&e, "half");
    assert_eq!(half.loads[0].value, LoadValue::Vector([0.0, -5e5, 0.0]));
    assert_eq!(half.requirements[0].value, 0.5);
    // `u` is an unknown of the drawing, and a requirement compares with a number it states
    let free = errors(&format!("{PLATE}scenario s {{\n  claim deflection(tip) <= u / 8\n}}\n"));
    assert!(free.iter().any(|(c, m)| c == "E040" && m.contains("unknown")), "{free:?}");
}

#[test]
fn a_scenario_prints_as_it_is_written() {
    let (p, errs) = parse(CASES);
    assert!(errs.is_empty());
    let mut out = String::new();
    for st in &p.root().body {
        assert!(matches!(st.kind, StmtKind::Scenario(_)));
        gcs_core::syntax::write_stmt_to(&mut out, &st.kind).expect("printable");
        out.push('\n');
    }
    assert_eq!(out, CASES);
}

/// What a scenario refuses, each where it is written.
#[test]
fn what_a_scenario_refuses() {
    let said = |body: &str| errors(&format!("{PLATE}scenario s {{\n{body}\n}}\n"));
    let has = |got: &[(String, String)], code: &str, words: &str| {
        assert!(got.iter().any(|(c, m)| c == code && m.contains(words)), "{code} {words}: {got:?}");
    };
    // syntax
    has(&said("  patch := point"), "syntax", "adds no geometry");
    has(&said("  tip horizontal b0"), "syntax", "a scenario holds");
    has(&said("  load tip (0, -5N)"), "syntax", "(x, y, z)");
    has(&said("  load tip 5N"), "syntax", "has a direction");
    has(&said("  pressure plate.near 1MPa along: down"), "syntax", "takes no `along:`");
    has(&said("  claim mode(plate) >= 1Hz"), "syntax", "counted from 1");
    has(&said("  claim safety(plate, along: down) >= 1"), "syntax", "what it is of alone");
    has(&said("  claim safety(plate) > 1"), "syntax", "closed");
    has(&said("  fixed tip at: b0"), "syntax", "`along:`");
    // the elaboration's
    has(&said("  load tip (0, -5mm, 0)"), "E103", "is Length");
    has(&said("  claim deflection(tip) <= 2N"), "E103", "`deflection` is Length");
    has(&said("  claim mode(plate, 1.5) >= 1Hz"), "E103", "counts from 1");
    has(&said("  pressure tip 1MPa"), "E040", "on a face");
    has(&said("  fixed std.front"), "E040", "is a plane");
    has(&said("  fixed tip along: tip"), "E040", "names a direction");
    has(&said("  fixed nowhere"), "E101", "nowhere");
    let twice = errors(&format!("{PLATE}scenario s {{\n  fixed tip\n}}\nscenario s {{\n  fixed tip\n}}\n"));
    has(&twice, "E001", "stated already");
    let base = errors(&format!("{PLATE}scenario a: b {{\n  fixed tip\n}}\nscenario b {{\n  fixed tip\n}}\n"));
    has(&base, "E101", "written above it");
    // until an optimiser keeps them together (#510 S4), one scenario's requirements drive
    let two = errors(&format!(
        "{PLATE}scenario a {{\n  safety(plate) == 2\n}}\nscenario b {{\n  safety(plate) == 3\n}}\n"
    ));
    has(&two, "E106", "drives dimensions already");
    // at the top of a document alone
    let inside = errors(&format!(
        "{PLATE}component C(p: point) {{\n  scenario s {{\n    fixed p\n  }}\n}}\n"
    ));
    has(&inside, "syntax", "top of a document");
    let block = errors(&format!("{PLATE}when 1 > 0 {{\n  scenario s {{\n    fixed tip\n  }}\n}}\n"));
    has(&block, "syntax", "top of a document");
    // and `scenario` is no name
    assert!(!gcs_core::syntax::is_name("scenario"));
}

/// The words of a scenario are tinted where they stand: `scenario` and the name it gives, a
/// line's word, and an output as a relation.
#[test]
fn a_scenario_is_tinted_as_it_reads() {
    use gcs_core::syntax::{highlight, Tint};
    let src = "scenario lift {\n  fixed plate.west\n  claim mode(plate, 1) >= 120Hz\n  \
               safety(plate) == 2\n}\n";
    let tint = |word: &str| {
        let lo = src.find(word).expect("written") as u32;
        highlight(src).into_iter().find(|(_, s)| s.lo == lo).map(|(t, _)| t)
    };
    assert_eq!(tint("scenario"), Some(Tint::Word));
    assert_eq!(tint("lift"), Some(Tint::Def));
    assert_eq!(tint("fixed"), Some(Tint::Word));
    assert_eq!(tint("claim"), Some(Tint::Word));
    assert_eq!(tint("mode"), Some(Tint::Relation));
    assert_eq!(tint("safety"), Some(Tint::Relation));
    // and only in a scenario: elsewhere `mode(…)` is the call it reads as
    let call = "m := mode(plate)\nload x\n";
    let runs = highlight(call);
    let at = |w: &str| call.find(w).expect("written") as u32;
    assert_eq!(runs.iter().find(|(_, s)| s.lo == at("mode")).map(|(t, _)| *t), Some(Tint::Type));
    assert!(!runs.iter().any(|(t, s)| s.lo == at("load") && *t == Tint::Word));
}
