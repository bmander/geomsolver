//! `k minimizes …` as the language states it (#121, #144): what an energy is over, how several
//! add, what it reads, what is refused, and how it is edited and printed.

use gcs_core::constraints::CKind;
use gcs_core::model::{EntKind, EntRef};
use crate::catenary::{read, samples, solved, ROPE};
use crate::common::parse;

fn same(a: &[(f64, f64)], b: &[(f64, f64)]) -> f64 {
    a.iter().zip(b).map(|(p, q)| (p.0 - q.0).hypot(p.1 - q.1)).fold(0.0, f64::max)
}

/// Two statements over one rope are one energy, its terms added: the two halves hang the rope as
/// the whole does.
#[test]
fn two_energies_over_one_curve_add() {
    let whole = solved(ROPE);
    let halves = ROPE.replace(
        "rope minimizes integral(p.y over p)",
        "rope minimizes 0.5 * integral(p.y over p)\nrope minimizes integral(p.y / 2 over p)",
    );
    let e = solved(&halves);
    assert_eq!(e.sketch.constraints.iter().filter(|c| c.kind == CKind::Stationary).count(), 2);
    assert!(same(&samples(&whole.sketch, 0), &samples(&e.sketch, 0)) < 1e-9);
}

/// Numbers in scope are written into the integrand, under whatever name the point is given.
#[test]
fn an_integrand_reads_the_scope() {
    let whole = solved(ROPE);
    let e = solved(&ROPE.replace("rope minimizes integral(p.y over p)", "h := 7\nrope minimizes integral(q.y - h over q)"));
    // a constant added to the height is the length's multiplier moved, not the shape
    assert!(same(&samples(&whole.sketch, 0), &samples(&e.sketch, 0)) < 1e-9);
}

/// An end held in one coordinate keeps the other: the energy closes none of the ends' freedoms.
#[test]
fn free_ends_keep_their_freedom() {
    let mut e = solved(&ROPE.replace("fix((100, 0)) b\n", "fix(y == 0) b\n").replace("  b := point\n", "  b := point hint((100, 0))\n"));
    let dg = gcs_core::diagnose::diagnose(&mut e.sketch, gcs_core::diagnose::DiagnoseOptions::default());
    assert_eq!(dg.dof, 1, "{dg:?}");
}

/// What an energy cannot be over, or read, is refused where it is written.
#[test]
fn energies_refused() {
    let refused = |src: &str, code: &str| {
        let (_, d) = read(src);
        assert!(d.iter().any(|m| m.starts_with(code)), "{src}\n{d:?}");
    };
    // a circle, and a spline with written control points, have no shape for an energy to state
    refused(&ROPE.replace("}\nrope minimizes", "o := point\nc := circle(o) hint(r: 5)\n}\nc minimizes"), "E040");
    refused(
        &ROPE.replace("  rope := curve(a, b)\n  length(150) rope\n", "  m := point hint((50, -40))\n  m2 := point hint((60, -40))\n  rope := spline(a, m, m2, b)\n"),
        "E040",
    );
    // nor a curve nothing declares
    refused(&ROPE.replace("rope minimizes", "cord minimizes"), "E101");
    // a free curve with no energy has no shape
    refused(&ROPE.replace("rope minimizes integral(p.y over p)\n", ""), "E040");
    // a name no scope declares
    refused(&ROPE.replace("p.y over", "p.y + h over"), "E101");
    // and a point's own field that is not one
    refused(&ROPE.replace("p.y over", "p.z over"), "E101");
    // nor, where the document names a unit, is an angle a power of length
    let mm = format!("unit mm\n{ROPE}");
    refused(&mm.replace("p.y over", "atan2(p.y, p.x) * 1deg over"), "E103");
    refused(&mm.replace("p.y over", "p.y + 1 over"), "E103");
    // and a free curve is drawn in a plane
    refused(&ROPE.replace("in std.front {\n", "{\n").replace("use std\n", ""), "E0");
}

/// Deleting the rope takes its energy with it, as it takes its length; deleting the energy leaves
/// the rope, with no shape until one is stated again.
#[test]
fn an_energy_goes_with_its_curve() {
    let e = solved(ROPE);
    let out = gcs_core::edit::remove(&e, &e.program, &e.sketch, &[EntRef::new(EntKind::Curve, 0)], &[]);
    assert!(out.refused.is_none(), "{:?}", out.refused);
    assert!(!out.text.contains("minimizes") && !out.text.contains("length("), "{}", out.text);
    let stat = e.sketch.constraints.iter().find(|c| c.kind == CKind::Stationary).unwrap().id;
    let out = gcs_core::edit::remove(&e, &e.program, &e.sketch, &[], &[stat]);
    assert!(!out.text.contains("minimizes") && out.text.contains("length(150) rope"), "{}", out.text);
}

/// The statement prints as it was written, so a document lifted from the drawing keeps it.
#[test]
fn an_energy_prints_as_written() {
    let src = "rope minimizes 2 * integral(p.y over p) - integral((p.x * t.y) over (p, t))";
    let (prog, errs) = parse(src);
    assert!(errs.is_empty(), "{:?}", errs.iter().map(|e| &e.message).collect::<Vec<_>>());
    let st = &prog.root().body[0];
    let mut out = String::new();
    gcs_core::syntax::write_stmt_to(&mut out, &st.kind).unwrap();
    assert_eq!(out, src);
}

/// The word stands after the curve it is about, a dotted one too, and is no name.
#[test]
fn an_energy_is_said_of_its_curve() {
    let src = "inst.rope maximizes integral(p.x over p)";
    let (prog, errs) = parse(src);
    assert!(errs.is_empty(), "{:?}", errs.iter().map(|e| &e.message).collect::<Vec<_>>());
    let mut out = String::new();
    gcs_core::syntax::write_stmt_to(&mut out, &prog.root().body[0].kind).unwrap();
    assert_eq!(out, src);
    assert!(!gcs_core::syntax::is_name("minimizes") && !gcs_core::syntax::is_name("maximizes"));
    assert!(!parse("minimize integral(p.y over p)").1.is_empty());
}
