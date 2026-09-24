use super::*;
use super::sweeps::{rotation,sphere,options,includes};
use gcs_core::{interval::minimum::{Options,Status},solid::{MaterialField,SweptField,SweepError}};

fn torus() -> MaterialField {
    SweptField::new(sphere(),rotation(),I::new(-4.,4.).unwrap()).into()
}
fn ball(radius: f64) -> MaterialField {
    SpatialField::from(RevolvedField::new(F::disk([0.;2],radius).unwrap(),[0.;3],[0.,0.,1.]).unwrap()).into()
}
fn torus_value(p: [f64;3]) -> f64 { (p[0].hypot(p[1])-3.).hypot(p[2])-1. }
fn ball_value(p: [f64;3]) -> f64 { p[0].hypot(p[1]).hypot(p[2])-3.5 }

#[test]
fn composed_swept_material_matches_independent_ball_and_torus_booleans() {
    let a = ball(3.5); let b = torus();
    let mut union = a.clone().union(b.clone()).unwrap().evaluator(128);
    let mut intersection = a.clone().intersection(b.clone()).unwrap().evaluator(128);
    let mut difference = a.difference(b).unwrap().evaluator(128);
    for p in [[0.;3],[3.,0.,0.],[4.1,0.,0.],[2.,1.,0.2],[0.,-2.8,0.4],[1.,0.,4.]] {
        let a = ball_value(p); let b = torus_value(p);
        for (field,expected) in [(&mut union,a.min(b)),(&mut intersection,a.max(b)),(&mut difference,a.max(-b))] {
            let query = field.bounds(point(p),options()).unwrap();
            includes(query.value,expected);
            assert_eq!(query.sweeps.len(),1);
            assert_eq!(query.sweeps[0].minimum.status,Status::Converged);
        }
    }
    let p = [I::new(2.7999,2.8001).unwrap(),I::new(-0.0001,0.0001).unwrap(),I::new(0.3999,0.4001).unwrap()];
    let query = difference.bounds(p,Options {value_tolerance:0.001,..options()}).unwrap();
    assert_eq!(query.sweeps[0].minimum.status,Status::Converged);
    for x in [2.7999,2.8,2.8001] { for y in [-0.0001,0.,0.0001] { for z in [0.3999,0.4,0.4001] {
        let p = [x,y,z]; includes(query.value,ball_value(p).max(-torus_value(p)));
    } } }
}

#[test]
fn shared_sweeps_keep_transformed_boxes_evidence_and_budgets_distinct() {
    let family = rotation();
    let sweep = MaterialField::from(SweptField::new(sphere(),family.clone(),I::new(-0.25,0.25).unwrap()));
    let rotated = sweep.clone().transformed(&family,3.).unwrap();
    let field = sweep.clone().union(rotated).unwrap();
    let mut cached = field.evaluator(32); let mut uncached = field.evaluator(0);
    let p = point([3.,0.,0.]);
    let mut observations = vec![0;2];
    let a = cached.bounds_with_observer(p,options(),|query,_,_| observations[query] += 1).unwrap();
    let b = uncached.bounds(p,options()).unwrap();
    assert_eq!(a.sweeps.len(),2); assert_eq!(a.value,b.value);
    assert_ne!(a.sweeps[0].point_box,a.sweeps[1].point_box);
    assert!(a.sweeps[0].minimum.value.bounds()[1] < 0.);
    assert!(a.sweeps[1].minimum.value.bounds()[0] > 0.);
    includes(a.sweeps[0].minimum.value,-1.);
    // The moved arc occupies [2.75,3.25] radians. Its nearest center to
    // (3,0,0) is the lower endpoint, independently of the interval transform.
    includes(a.sweeps[1].minimum.value,(3.-3.*2.75_f64.cos()).hypot(3.*2.75_f64.sin())-1.);
    for (i,(a,b)) in a.sweeps.iter().zip(&b.sweeps).enumerate() {
        assert_eq!(a.minimum.value,b.minimum.value);
        assert_eq!(a.minimum.witness,b.minimum.witness);
        assert_eq!(a.minimum.evaluations,b.minimum.evaluations);
        assert_eq!(a.minimum.evaluations,observations[i]);
    }
    assert_eq!(cached.cached_poses(),32); assert_eq!(uncached.cached_poses(),0);
    cached.clear_cache(); assert_eq!(cached.cached_poses(),0);
    let coarse = cached.bounds(p,Options {max_evaluations:4,..options()}).unwrap();
    assert_eq!(coarse.sweeps.len(),2);
    assert!(coarse.sweeps.iter().all(|s| s.minimum.status == Status::BudgetExhausted));
    assert!(coarse.value.bounds()[0] <= a.value.bounds()[0] && coarse.value.bounds()[1] >= a.value.bounds()[1]);
    let again = cached.bounds(p,options()).unwrap(); assert_eq!(again.value,a.value);

    // 2^63 expression visits must reduce to one unique node/box sweep query.
    let mut dag = sweep;
    for _ in 1..64 { dag = dag.clone().intersection(dag).unwrap(); }
    let query = dag.evaluator(0).bounds(p,options()).unwrap();
    assert_eq!(query.sweeps.len(),1);
    assert_eq!(query.value,a.sweeps[0].minimum.value);
    assert!(dag.clone().union(dag.clone()).is_err());
    assert!(dag.transformed(&family,0.).is_err());
}

#[test]
fn material_cancellation_and_failed_operands_do_not_become_silent_solids() {
    let a = torus();
    let mut empty = a.clone().difference(a).unwrap().evaluator(64);
    for p in [[0.;3],[3.,0.,0.],[4.,0.,0.]] {
        let query = empty.bounds(point(p),options()).unwrap();
        includes(query.value,torus_value(p).abs());
        assert_eq!(query.sweeps.len(),1);
        assert!(query.value.bounds()[1] >= 0.); // no established material in A-A
    }
    // A failed leaf is still an error even if another operand could fix the sign.
    let invalid = MaterialField::from(SweptField::new(sphere(),rotation(),I::point(f64::MAX).unwrap()));
    let mut field = ball(100.).union(invalid).unwrap().evaluator(0);
    assert_eq!(field.bounds(point([0.;3]),options()).unwrap_err(),SweepError::Oracle(Error::Overflow));
    let mut field = torus().evaluator(0);
    assert_eq!(field.bounds(point([0.;3]),Options {max_evaluations:3,..options()}).unwrap_err(),SweepError::InvalidOptions);
    assert_eq!(field.bounds(point([f64::MAX;3]),options()).unwrap_err(),SweepError::Oracle(Error::Overflow));
    assert!(torus().transformed(&rotation(),f64::NAN).is_err());
}

#[test]
fn classification_bands_follow_cut_signs_and_shared_query_identity() {
    let sweep = torus();
    let field = ball(3.5).difference(sweep.clone()).unwrap().union(sweep.clone()).unwrap();
    let band = I::new(-0.8,-0.6).unwrap();
    let mut evaluator = field.evaluator(128);
    let p = point([3.3,0.,0.]);
    let fine = evaluator.bounds(p,options()).unwrap();
    let mut observations = [0;2];
    let fast = evaluator.bounds_outside_with_observer(p,band,options(),|query,_,_| {
        observations[query] += 1;
    }).unwrap();
    for (s,n) in fast.sweeps.iter().zip(observations) { assert_eq!(s.minimum.evaluations,n); }
    includes(fast.value,-0.7);
    assert!(fast.value.bounds()[0] <= fine.value.bounds()[0] && fast.value.bounds()[1] >= fine.value.bounds()[1]);
    assert_eq!(fast.sweeps.len(),2);
    assert_eq!(fast.sweeps[0].point_box,fast.sweeps[1].point_box);
    assert_eq!(fast.sweeps[0].separation_band,Some(band.neg()));
    assert_eq!(fast.sweeps[1].separation_band,Some(band));
    assert_eq!(fast.sweeps[0].minimum.status,Status::Separated);
    assert_eq!(fast.sweeps[1].minimum.status,Status::Converged);
    assert!(fast.sweeps[0].minimum.evaluations < fast.sweeps[1].minimum.evaluations);
    let shared = sweep.clone().union(sweep).unwrap().evaluator(0).bounds_outside(p,I::ZERO,options()).unwrap();
    assert_eq!(shared.sweeps.len(),1);
    assert!(shared.value.bounds()[1] < 0.);
    let invalid = MaterialField::from(SweptField::new(sphere(),rotation(),I::point(f64::MAX).unwrap()));
    assert_eq!(ball(100.).union(invalid).unwrap().evaluator(0).bounds_outside(p,I::ZERO,options()).unwrap_err(),
        SweepError::Oracle(Error::Overflow));
}
