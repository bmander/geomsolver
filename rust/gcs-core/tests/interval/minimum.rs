use super::*;
use gcs_core::interval::minimum::{self,Options,Status};

fn options() -> Options { Options {value_tolerance:1e-8,max_evaluations:10000} }
fn min(a: I,b: I) -> I {
    interval(a.bounds()[0].min(b.bounds()[0]),a.bounds()[1].min(b.bounds()[1]))
}

#[test]
fn continuous_sphere_translation_includes_endpoint_and_interior_minima() {
    // A unit sphere translated from x=0 to x=1. Use squared implicit distance:
    // (x-t)^2+y^2-1, whose sign defines the sphere but whose values are not distances.
    for x in [-0.5,0.,0.375,1.,1.5] {
        for y in [0.,0.75,1.25] {
            let found = minimum::enclose(interval(0.,1.),|t|
                t.sub(point(x))?.square()?.add(point(y*y))?.sub(I::ONE),options()).unwrap();
            let expected = (x-x.clamp(0.,1.)).powi(2)+y*y-1.;
            assert_eq!(found.status,Status::Converged);
            assert!(found.value.contains(expected),"{x} {y}: {found:?}, expected {expected}");
            assert!(found.evaluations <= options().max_evaluations);
            let witness = (x-found.witness).powi(2)+y*y-1.;
            assert!(witness <= found.value.bounds()[1]+1e-14);
        }
    }
}

#[test]
fn a_narrow_competing_minimum_cannot_be_discarded_between_samples() {
    let field = |t: I| -> Result<I,Error> {
        let broad = t.sub(point(0.2))?.square()?.add(point(0.1))?;
        let narrow = t.sub(point(0.71337))?.mul(point(1000.))?.square()?.sub(point(0.001))?;
        Ok(min(broad,narrow))
    };
    // Every coarse stamp says outside. The complete interval still contains a cut.
    for i in 0..=100 {
        assert!(field(point(i as f64/100.)).unwrap().bounds()[0] > 0.);
    }
    let limited = minimum::enclose(interval(0.,1.),field,Options {max_evaluations:4,..options()}).unwrap();
    assert_eq!(limited.status,Status::BudgetExhausted);
    assert!(limited.value.contains(-0.001));
    assert!(limited.value.bounds()[1] > 0.);
    let found = minimum::enclose(interval(0.,1.),field,options()).unwrap();
    assert_eq!(found.status,Status::Converged);
    assert!(found.value.contains(-0.001));
    assert!(found.value.bounds()[1] < 0.);
    assert!((found.witness-0.71337).abs() < 1e-6);
    let limited = minimum::enclose_outside(interval(0.,1.),field,
        Options {max_evaluations:4,..options()},I::ZERO).unwrap();
    assert_eq!(limited.status,Status::BudgetExhausted);
    assert!(limited.value.contains(-0.001) && limited.value.bounds()[1] > 0.);
    let separated = minimum::enclose_outside(interval(0.,1.),field,options(),I::ZERO).unwrap();
    assert_eq!(separated.status,Status::Separated);
    assert!(separated.value.contains(-0.001) && separated.value.bounds()[1] < 0.);
    assert!(separated.evaluations < found.evaluations);
}

#[test]
fn band_separation_keeps_global_bounds_and_is_distinct_from_convergence() {
    let field = |t: I| t.sub(point(0.37))?.square()?.add(point(2.));
    let fine = minimum::enclose(interval(0.,1.),field,options()).unwrap();
    for band in [interval(-1.,1.),interval(3.,4.)] {
        let found = minimum::enclose_outside(interval(0.,1.),field,options(),band).unwrap();
        assert_eq!(found.status,Status::Separated);
        assert_eq!(found.evaluations,4);
        assert!(found.value.bounds()[0] <= fine.value.bounds()[0] && found.value.bounds()[1] >= fine.value.bounds()[1]);
        assert!(found.value.bounds()[1]-found.value.bounds()[0] > options().value_tolerance);
    }
    // Contact with either band endpoint is unresolved, not strict separation.
    for band in [interval(1.,2.),interval(3.,4.)] {
        let found = minimum::enclose_outside(interval(0.,1.),|_| Ok::<_,()>(interval(2.,3.)),
            Options {max_evaluations:4,..options()},band).unwrap();
        assert_eq!(found.status,Status::BudgetExhausted);
    }
    let zero = minimum::enclose_outside(interval(0.,1.),|_| Ok::<_,()>(I::ZERO),options(),I::ZERO).unwrap();
    assert_eq!(zero.status,Status::Converged); assert_eq!(zero.value,I::ZERO);
    assert_eq!(minimum::enclose_outside(interval(0.,1.),|_| Err::<I,_>("missing enclosure"),options(),I::ZERO)
        .unwrap_err(),minimum::Error::Oracle("missing enclosure"));
}

#[test]
fn unresolved_oracle_uncertainty_and_unrepresentable_subdivision_stay_explicit() {
    for domain in [point(1.),interval(1.,1f64.next_up())] {
        let found = minimum::enclose(domain,|_| Ok::<_,()>(interval(-1.,1.)),options()).unwrap();
        assert_eq!(found.status,Status::ResolutionLimit);
        assert_eq!(found.value,interval(-1.,1.));
    }
    let found = minimum::enclose(interval(-f64::MAX,f64::MAX),|_| Ok::<_,()>(I::ZERO),options()).unwrap();
    assert_eq!(found.status,Status::Converged);
    assert_eq!(found.value,I::ZERO);
    for budget in [4,5,7,8,15] {
        let found = minimum::enclose(interval(0.,1.),|_| Ok::<_,()>(interval(-1.,1.)),
            Options {max_evaluations:budget,..options()}).unwrap();
        assert_eq!(found.status,Status::BudgetExhausted);
        assert!(found.evaluations <= budget);
        assert_eq!(found.value,interval(-1.,1.));
    }
}

#[test]
fn invalid_controls_and_detectable_oracle_errors_fail_closed() {
    for tolerance in [0.,-1.,f64::NAN,f64::INFINITY] {
        let bad = minimum::enclose(interval(0.,1.),|_| -> Result<I,()> { panic!("invalid options") },
            Options {value_tolerance:tolerance,..options()});
        assert_eq!(bad.unwrap_err(),minimum::Error::InvalidOptions);
    }
    assert_eq!(minimum::enclose(interval(0.,1.),|_| Ok::<_,()>(I::ZERO),
        Options {max_evaluations:3,..options()}).unwrap_err(),minimum::Error::InvalidOptions);
    assert_eq!(minimum::enclose(interval(0.,1.),|_| Err::<I,_>("missing enclosure"),options())
        .unwrap_err(),minimum::Error::Oracle("missing enclosure"));
    assert_eq!(minimum::enclose(interval(0.,1.),|t| Ok::<_,()>(
        if t.bounds()[0] == t.bounds()[1] { point(2.) } else { interval(0.,1.) }),options())
        .unwrap_err(),minimum::Error::InconsistentBounds);
}
