//! Issue #54: derive verdicts from validated arguments, geometry and complete sampled coverage.
use gcs_core::{
    clear,
    diagnose::{self, SolidOutcome},
    json::{self, Json},
    model::SolidRequirement,
    program, report, syntax,
};

fn fixture() -> String {
    include_str!("fixtures/solid_issue51/sweep_possible.sv").into()
}
fn read(src: &str) -> program::Elaborated {
    let (p, errors) = syntax::parse(src);
    assert!(errors.is_empty(), "{errors:?}");
    let e = program::elaborate(&p);
    assert!(e.ok(), "{:?}", e.diags);
    e
}
fn verdict(src: &str) -> diagnose::SolidVerdict {
    let mut e = read(src);
    assert!(gcs_core::solve::solve(&mut e.sketch, Default::default()).success);
    diagnose::judge_solids(&e.sketch).remove(0)
}

#[test]
fn interval_and_length_constructors_reject_nonfinite_or_inverted_values() {
    for (lo, hi) in [(1.0, 0.0), (f64::NAN, 1.0), (0.0, f64::INFINITY), (f64::NEG_INFINITY, 0.0)] {
        assert!(clear::Interval::new(lo, hi).is_err());
    }
    let i = clear::Interval::new(-f64::MAX, f64::MAX).unwrap();
    assert_eq!(i.midpoint(), 0.0);
    assert_eq!(i.uncertainty(), f64::MAX);
    let rounded = clear::Interval::around(1e100, 1e-100).unwrap();
    assert!(rounded.lower() < 1e100 && rounded.upper() > 1e100);
    assert!(clear::Interval::around(f64::MAX, 1.0).is_err());
    assert!(clear::Interval::around(1.0, -1.0).is_err());
    let smallest = f64::from_bits(1);
    assert!(clear::Interval::new(0.0, smallest).unwrap().uncertainty() > 0.0);
    assert_eq!(clear::Interval::new(smallest, smallest).unwrap().midpoint(), smallest);
    for value in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        assert!(gcs_core::model::Length::new(value).is_err());
    }
}

#[test]
fn parsing_preserves_only_complete_valid_variants() {
    let src = fixture();
    for form in [
        "inside(1mm)",
        "inside(1mm,2mm)",
        "fits",
        "fits(1mm,2mm)",
        "clear(1deg)",
        "clear(1mm,2mm)",
        "clear(1e309mm)",
        "clear(true)",
    ] {
        let (p, errors) = syntax::parse(&src.replace("clear(1mm)", form));
        let e = program::elaborate(&p);
        assert!(!errors.is_empty() || !e.ok(), "accepted {form}");
        assert!(e.sketch.solid_claims.is_empty(), "valid claim created for {form}");
    }
    for bounds in ["(10deg,30deg)", "(0.1mm,30deg)", "(1e309mm,2mm)"] {
        let (p, errors) = syntax::parse(&src.replace("(0.1mm,0.9mm)", bounds));
        let e = program::elaborate(&p);
        assert!(!errors.is_empty() || !e.ok());
        assert!(e.sketch.solid_claims.is_empty());
    }
    let e = read(&src);
    assert!(
        matches!(e.sketch.solid_claims[0].requirement(), SolidRequirement::Clear { gap } if gap.value() == 1.0)
    );
    let e = read(&src.replace("clear(1mm)", "inside"));
    assert!(matches!(e.sketch.solid_claims[0].requirement(), SolidRequirement::Inside));
}

#[test]
fn coverage_derives_success_partial_failure_and_counterexamples() {
    let src = fixture();
    for (bounds, gap, expected) in [
        ("(0.1mm,0.9mm)", "1mm", SolidOutcome::SampledSuccess),
        ("(0.1mm,0.9mm)", "4mm", SolidOutcome::SampledSuccess), // exact planar equality
        ("(0.1mm,0.9mm)", "5mm", SolidOutcome::Refuted),
        ("(0.1mm,2mm)", "1mm", SolidOutcome::Indeterminate),
        ("(0.1mm,2mm)", "5mm", SolidOutcome::Refuted),
        ("(2mm,3mm)", "1mm", SolidOutcome::Indeterminate),
    ] {
        let v = verdict(
            &src.replace("(0.1mm,0.9mm)", bounds).replace("clear(1mm)", &format!("clear({gap})")),
        );
        assert_eq!(v.outcome(), expected, "{bounds}, {gap}: {v:?}");
        assert_eq!(v.poses().len(), 37);
        assert_eq!(v.valid_samples().count() + v.failures().count(), 37);
        assert!(v.failures().all(|p| !p.evaluation().unwrap_err().is_empty()));
        let j = report::solid_claim_json(&v);
        let encoded = j.dump(None);
        assert_eq!(json::parse(&encoded).unwrap().dump(None), encoded);
        assert_eq!(j.get("text").unwrap().as_str(), report::solid_claim_text(&v));
        assert_eq!(j.get("continuousProof"), Some(&Json::Bool(false)));
        if expected == SolidOutcome::Refuted {
            let witness = v.counterexample().unwrap();
            assert_eq!(witness.holds(), Some(false));
            assert_eq!(v.worst(), witness.parameter());
            assert!(report::solid_claim_text(&v).contains("counterexample at"));
        }
        if v.valid_samples().count() == 0 {
            assert!(v.measured().is_none() && v.measurement().is_none() && v.worst().is_none());
            assert_eq!(j.get("tolerance"), Some(&Json::Null));
            assert_eq!(j.get("counterexample"), Some(&Json::Null));
            let text = report::solid_claim_text(&v);
            assert!(text.contains("no solved valid poses"));
            assert_eq!(text.matches("unresolved at").count(), 37);
        }
    }
}

fn round_claim(form: &str) -> String {
    format!("unit mm\npoint a hint(x:0,y:0)\nground a\ncircle ac(center:a)\nradius(1mm) ac\npoint b hint(x:2,y:0)\nground b\ncircle bc(center:b)\nradius(1mm) bc\nface af(ac)\nface bf(bc)\nsolid result(af,depth:1mm)\nsolid other(bf,depth:1mm)\nclaim result {form} other\n")
}

#[test]
fn unresolved_predicates_and_spacing_are_independent_of_negative_gaps() {
    for (form, expected) in [("clear(-100mm)", None), ("clear(1mm)", Some(false))] {
        let v = verdict(&round_claim(form));
        assert_eq!(v.holds(), expected);
        let e = v.poses()[0].evaluation().unwrap();
        assert!(matches!(e.predicate(), clear::Predicate::Unresolved(_)));
        let i = e.measurement().interval().unwrap();
        assert!(i.lower() < 0.0 && i.upper() > 0.0);
    }
    // Equal curved solids share a boundary: faceting cannot certify containment at contact.
    for form in ["inside", "fits(-100mm)"] {
        let v = verdict(&round_claim(form).replace("x:2,y:0", "x:0,y:0"));
        assert_eq!(v.holds(), None);
        assert!(report::solid_claim_text(&v).contains("containment"));
    }
    // A sampled, valid but uncertain pose also prevents sampled success.
    let prefix = fixture().split("point resultfp0").next().unwrap().to_string();
    let src = prefix
        + &round_claim("clear(-100mm)").replace("unit mm\n", "").replace(
            "claim result clear(-100mm) other",
            "claim over reach in (0.1mm,0.9mm) { result clear(-100mm) other }",
        );
    let v = verdict(&src);
    assert_eq!(v.outcome(), SolidOutcome::Indeterminate);
    assert_eq!(v.valid_samples().count(), 37);
    assert_eq!(v.failures().count(), 0);
}

#[test]
fn finite_extreme_sweep_interpolation_preserves_endpoints_and_units() {
    for (from, to) in [
        (-f64::MAX, f64::MAX),
        (f64::MAX, f64::MAX),
        (f64::MAX, f64::MAX / 2.0),
        (f64::MAX, -f64::MAX),
    ] {
        let src = fixture().replace("(0.1mm,0.9mm)", &format!("({from}mm,{to}mm)"));
        let e = read(&src);
        let sw = e.sketch.solid_claims[0].over().unwrap();
        assert_eq!(sw.sample(0, 36), Some(from));
        assert_eq!(sw.sample(36, 36), Some(to));
        for k in 0..=36 {
            let t = sw.sample(k, 36).unwrap();
            assert!(t.is_finite() && t >= from.min(to) && t <= from.max(to));
        }
    }
    let e = read(&fixture().replace("unit mm", "unit cm"));
    let sw = e.sketch.solid_claims[0].over().unwrap();
    assert!((sw.from() - 0.01).abs() < 1e-12);
    assert!((sw.to() - 0.09).abs() < 1e-12);
}

#[test]
fn invalid_geometry_and_invalid_legacy_arguments_have_explicit_failures() {
    let mut e = read(&fixture());
    let a = e.map.ent_named("result").unwrap().i();
    let b = e.map.ent_named("other").unwrap().i();
    let invalid =
        clear::judge(&e.sketch, gcs_core::constraints::SolidWord::Clear, a, b, f64::NAN, 0.0);
    assert!(invalid.evidence().is_err());
    assert_eq!(invalid.holds(), None);
    assert!(invalid.measured().is_none());
    if let gcs_core::model::SolidDef::Prism { from, to, .. } = &mut e.sketch.solids[a].def {
        to.value = from.value;
    }
    let v = diagnose::judge_solids(&e.sketch).remove(0);
    assert_eq!(v.failures().count(), 37);
    assert!(v.failures().all(|p| p.evaluation().unwrap_err().contains("distinct")));
    assert_eq!(v.outcome(), SolidOutcome::Indeterminate);
}

#[test]
fn empty_boundaries_are_explicitly_unbounded_and_cannot_certify() {
    let src = fixture()
        .replace("claim over", "solid removed(result)\nresult through removed\nclaim over")
        .replace("{ result clear(1mm) other }", "{ removed clear(1mm) other }");
    let v = verdict(&src);
    assert_eq!(v.outcome(), SolidOutcome::Indeterminate);
    assert_eq!(v.valid_samples().count(), 37);
    assert!(matches!(v.measurement(), Some(clear::Measurement::Unbounded)));
    assert!(v.measured().is_none() && v.tolerance().is_none());
    let j = report::solid_claim_json(&v);
    assert_eq!(j.get("measurement").unwrap().get("kind").unwrap().as_str(), "unbounded");
    assert!(json::parse(&j.dump(None)).is_ok());
}

#[test]
fn angular_coverage_and_failure_reports_keep_degrees() {
    let src = fixture()
        .replace("circle c(center:o)", "point q hint(x:1,y:0)\nground q\ncircle c(center:o)")
        .replace(
            "o distance(reach,along:x) p",
            "line datum(o,q)\nline radial(o,p)\ndatum angle(reach) radial",
        )
        .replace("(0.1mm,0.9mm)", "(10deg,30deg)");
    let v = verdict(&src);
    assert_eq!(v.outcome(), SolidOutcome::SampledSuccess);
    assert_eq!(v.poses().first().unwrap().parameter(), Some(10.0));
    assert_eq!(v.poses().last().unwrap().parameter(), Some(30.0));
    let j = report::solid_claim_json(&v);
    assert_eq!(j.get("coverage").unwrap().get("units").unwrap().as_str(), "degrees");
    assert!(report::solid_claim_text(&v).contains("[10deg, 30deg]"));
}

#[test]
fn a_counterexample_refutes_a_sweep_with_uncertain_contact_and_successful_poses() {
    let src = round_claim("clear(-100mm)")
        .replace("ground b\n", "a distance(reach,along:x) b\na distance(0mm,along:y) b\n")
        .replace(
            "claim result clear(-100mm) other",
            "claim over reach in (1mm,3mm) { result clear(-100mm) other }",
        );
    let v = verdict(&src);
    assert_eq!(v.outcome(), SolidOutcome::Refuted);
    assert_eq!(v.valid_samples().count(), 37);
    assert_eq!(v.poses()[18].parameter(), Some(2.0));
    assert_eq!(v.poses()[18].holds(), None);
    assert_eq!(v.poses().last().unwrap().holds(), Some(true));
    assert_eq!(v.counterexample().unwrap().holds(), Some(false));
    assert_eq!(v.worst(), v.counterexample().unwrap().parameter());
}

#[test]
fn a_single_pose_claim_cannot_silently_solve_a_different_pose() {
    let mut e = read(&round_claim("clear(1mm)").replace("x:2,y:0", "x:20,y:0"));
    assert!(gcs_core::solve::solve(&mut e.sketch, Default::default()).success);
    let radius = e.sketch.circles[0].radius as usize;
    e.sketch.params[radius].value = 5.0; // Valid geometry, but violates radius(1mm).
    let before: Vec<_> = e.sketch.params.iter().map(|p| p.value).collect();
    let v = diagnose::judge_solids(&e.sketch).remove(0);
    assert_eq!(v.outcome(), SolidOutcome::Indeterminate);
    assert!(v.poses()[0].evaluation().unwrap_err().contains("current pose"));
    assert!(v.measured().is_none());
    assert_eq!(e.sketch.params.iter().map(|p| p.value).collect::<Vec<_>>(), before);
    assert!(gcs_core::solve::solve(&mut e.sketch, Default::default()).success);
    assert_eq!(diagnose::judge_solids(&e.sketch)[0].outcome(), SolidOutcome::Holds);
}

#[test]
fn reported_midpoint_and_uncertainty_enclose_the_interval() {
    for (lower, upper) in [
        (1.0, 1.0f64.next_up()),
        (f64::from_bits(1), f64::from_bits(3)),
        (-f64::MAX, f64::MAX),
        (f64::MAX.next_down(), f64::MAX),
        (-1.0, 2.0),
    ] {
        let i = clear::Interval::new(lower, upper).unwrap();
        let m = i.midpoint();
        let r = i.uncertainty();
        assert!(lower <= m && m <= upper);
        assert!(r.is_finite() && r >= m - lower && r >= upper - m);
    }
}
