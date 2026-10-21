//! A claim with witnesses of its own (§9.7, #460): `claim { … }` declares geometry made only to
//! state the claim, and that geometry restricts nothing.
//!
//! The case is the issue's belt: two pulleys, the upper and lower runs tangent to both, and the
//! claim that the runs meet on the line of centres.  Stating it takes a point on both runs —
//! which at equal radii, where the runs are parallel, cannot exist.  Written loose, that point
//! makes a valid drive unsolvable; written in the claim's block it is solved after the drive,
//! the drive held, and the claim is refuted while the drive solves as it always did.

use gcs_core::diagnose::{diagnose, DiagnoseOptions, Diagnosis, State};
use gcs_core::model::Sketch;
use gcs_core::program::Elaborated;

/// The drive, the second pulley's radius written in (or left free, `""`).
fn belt(rb: &str) -> String {
    let radius = if rb.is_empty() { String::new() } else { format!("  radius({rb}) cb\n") };
    format!(
        "\
unit mm
use std
component Run(view: plane, a: circle, b: circle, up: Scalar) {{
  in view {{
    s := point hint((a.center.x, a.center.y + up * a.r))
    e := point hint((b.center.x, b.center.y + up * b.r))
    span := line(s, e)
    s coincident a
    e coincident b
    span tangent(at: p1) a
    span tangent(at: p2) b
  }}
}}
in std.front {{
  a := point
  b := point
  fix((0, 0)) a
  fix((120, 0)) b
  ca := circle(center: a)
  cb := circle(center: b) hint(r: 30)
  radius(20mm) ca
{radius}  centres := line(a, b)
}}
upper := Run(std.front, ca, cb, up: 1)
lower := Run(std.front, ca, cb, up: -1)
"
    )
}

/// The claim, its witness in its block.
const WITNESSED: &str = "\
in std.front {
  claim {
    x := point
    x coincident upper.span
    x coincident lower.span
    x coincident centres
  }
}
";

/// The same three relations, stated: the construction the issue warns of.
const LOOSE: &str = "\
in std.front {
  x := point hint((-96, 0))
  x coincident upper.span
  x coincident lower.span
  x coincident centres
}
";

fn elaborated(src: &str) -> Elaborated {
    let (prog, errs) = crate::common::parse(src);
    assert!(errs.is_empty(), "does not parse: {errs:?}");
    gcs_core::program::elaborate(&prog)
}

/// Elaborated, solved (settled) and diagnosed.
fn judged(src: &str) -> (Elaborated, Sketch, Diagnosis) {
    let e = elaborated(src);
    assert!(
        e.ok(),
        "does not elaborate: {:?}",
        e.errors().map(|d| (d.code.as_str(), d.message.clone())).collect::<Vec<_>>()
    );
    let mut sk = e.sketch.clone();
    let r = gcs_core::solve::solve(&mut sk, gcs_core::solve::SolveOpts::default());
    assert!(r.success && r.settled(), "does not solve: {}", r.message);
    let d = diagnose(&mut sk, DiagnoseOptions::default());
    (e, sk, d)
}

fn xy(e: &Elaborated, sk: &Sketch, name: &str) -> (f64, f64) {
    let p = e.map.ent_named(name).unwrap_or_else(|| panic!("no `{name}`"));
    sk.point_xy(p.i())
}

/// The block's statements, by id.
fn block(sk: &Sketch) -> Vec<u32> {
    sk.constraints.iter().filter(|c| c.witness.is_some()).map(|c| c.id).collect()
}

/// What the drawing says of itself, the claims left out.
fn ledger(d: &Diagnosis) -> (usize, usize, usize, i64, State, Vec<u32>, Vec<u32>) {
    (d.n_params, d.n_equations, d.structural_rank, d.dof, d.status, d.over.clone(),
     d.implied.clone())
}

#[test]
fn unequal_pulleys_meet_on_the_line_of_centres() {
    let (e, sk, d) = judged(&format!("{}{WITNESSED}", belt("45mm")));
    let rows = block(&sk);
    assert_eq!(rows.len(), 3, "the block's three relations");
    assert_eq!(d.claims_theorem, rows, "the block is one claim, and the drive implies it");
    assert!(d.claims_violated.is_empty() && d.claims_consuming.is_empty());
    // the witness was solved for, from no seed of its own, to where the runs meet: the external
    // centre of similitude, 120 · 20 / (20 − 45) from `a`
    let (x, y) = xy(&e, &sk, "x");
    assert!((x + 96.0).abs() < 1e-6 && y.abs() < 1e-6, "the witness stands at ({x}, {y})");
}

#[test]
fn a_witnessed_claim_is_a_theorem_over_the_drawings_freedom() {
    // the second radius left free: the runs meet on the line of centres at every radius but
    // the first's, so the claim is implied, not merely true at the radius the seed gave
    let (_, sk, d) = judged(&format!("{}{WITNESSED}", belt("")));
    assert_eq!((d.dof, d.status), (1, State::Under), "the witness is no freedom of the drawing");
    assert_eq!(d.claims_theorem, block(&sk));
}

#[test]
fn equal_pulleys_solve_and_the_claim_is_refuted() {
    // the runs are parallel: no point lies on both.  The drive solves as it does with no claim,
    // to the same numbers, and the claim is refuted — never a conflict, never unsolved
    let (e0, bare, d0) = judged(&belt("20mm"));
    let (e, sk, d) = judged(&format!("{}{WITNESSED}", belt("20mm")));
    assert_eq!(ledger(&d), ledger(&d0), "the block changed what the drawing says of itself");
    assert_eq!(d.status, State::Well);
    assert_eq!(d.claims_violated, block(&sk));
    assert!(d.claims_theorem.is_empty() && d.claims_consuming.is_empty());
    assert!(d.conflicts.is_none() && d.violated.is_empty());
    for name in ["upper.s", "upper.e", "lower.s", "lower.e"] {
        assert_eq!(xy(&e, &sk, name), xy(&e0, &bare, name), "`{name}` moved");
    }
    let (sx, sy) = xy(&e, &sk, "upper.s");
    assert!(sx.abs() < 1e-9 && (sy - 20.0).abs() < 1e-9, "the contact is at ({sx}, {sy})");
}

#[test]
fn a_witness_restricts_nothing_its_loose_construction_would() {
    // stated loose, the witness is two unknowns under three equations: the drawing reads the
    // surplus as its own (one relation implied by the others)
    let (_, _, loose) = judged(&format!("{}{LOOSE}", belt("45mm")));
    assert_eq!(loose.n_equations, loose.n_params + 1, "three rows on the witness's two unknowns");
    assert!(!loose.implied.is_empty() || !loose.over.is_empty(), "{:?}", ledger(&loose));
    // in its block, the drawing is exactly the drawing with the block deleted
    let (_, _, bare) = judged(&belt("45mm"));
    let (_, _, d) = judged(&format!("{}{WITNESSED}", belt("45mm")));
    assert_eq!(ledger(&d), ledger(&bare));
    assert!(d.over.is_empty() && d.implied.is_empty());
}

#[test]
fn a_witness_the_pose_happens_to_allow_is_consuming() {
    // nothing holds `c` on the rail — the seed put it there.  A point on the rail where `c` is
    // exists at this solution, and would not once `c` moved: the claim claims too much
    let (_, sk, d) = judged(
        "\
use std
in std.front {
  a := point
  b := point hint((10, 0))
  fix((0, 0)) a
  fix((10, 0)) b
  rail := line(a, b)
  c := point hint((4, 0))
  claim {
    w := point hint((1, 1))
    w coincident rail
    w coincident c
  }
}
",
    );
    assert_eq!((d.dof, d.status), (2, State::Under), "`c` keeps both its freedoms");
    assert_eq!(d.claims_consuming, block(&sk));
    assert!(d.claims_theorem.is_empty() && d.claims_violated.is_empty());
}

#[test]
fn a_witness_is_its_claims_alone() {
    // a statement outside the block naming the witness would make it part of the drawing again
    let e = elaborated(&format!(
        "{}in std.front {{\n  x coincident a\n}}\n",
        format!("{}{WITNESSED}", belt("45mm"))
    ));
    let errs: Vec<(&str, String)> =
        e.errors().map(|d| (d.code.as_str(), d.message.clone())).collect();
    assert_eq!(errs.len(), 1, "{errs:?}");
    assert_eq!(errs[0].0, "E040");
    assert!(errs[0].1.contains("`x` is a claim's witness"), "{errs:?}");
    // and a block holds declarations and relations, nothing that adds to the drawing
    let (_, errs) = crate::common::parse("use std\nclaim {\n  repeat 2 { p := point }\n}\n");
    assert!(errs.iter().any(|e| e.message.contains("declares its witnesses")), "{errs:?}");
}

#[test]
fn a_precondition_is_a_when_over_the_numbers() {
    // "unequal radii: the runs meet on the line of centres", read over the drawing's numbers:
    // at equal radii the claim is not made at all
    let conditional = |rb: &str| {
        let claim = WITNESSED.replace("  claim {", "  when rb != 20mm {\n  claim {")
            .replace("\n  }\n}", "\n  }\n  }\n}");
        format!("{}rb := {rb}\n{claim}", belt("rb"))
    };
    let (_, sk, d) = judged(&conditional("20mm"));
    assert!(block(&sk).is_empty(), "no claim is made");
    assert!(d.claims_theorem.is_empty() && d.claims_violated.is_empty());
    let (_, sk, d) = judged(&conditional("45mm"));
    assert_eq!(d.claims_theorem, block(&sk));
    assert_eq!(block(&sk).len(), 3);
}

#[test]
fn a_witness_travels_like_any_flag() {
    let (_, sk, _) = judged(&format!("{}{WITNESSED}", belt("45mm")));
    let back = gcs_core::io::loads(&gcs_core::io::dumps(&sk, None)).unwrap();
    assert_eq!(block(&back), block(&sk), "the rows keep their block");
    assert_eq!(back.roles.values().filter(|r| r.witness).count(), 1, "the point stays a witness");
    let kept = gcs_core::graft::without(&sk, &[], &[]);
    assert_eq!(kept.constraints.iter().filter(|c| c.witness.is_some()).count(), 3);
    // and the lifted program writes the block back around its statements, never loose, where
    // they would restrict the drawing: read again, it is the same claim, judged the same
    let text = gcs_core::program::to_program(&sk).text().to_string();
    assert!(text.contains("claim {\n"), "{text}");
    let (_, again, d) = judged(&text);
    assert_eq!(block(&again).len(), 3, "{text}");
    assert_eq!(d.claims_theorem, block(&again), "{text}");
}
