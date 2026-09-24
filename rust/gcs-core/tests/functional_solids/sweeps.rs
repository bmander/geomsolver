use super::*;
use gcs_core::{interval::minimum::{Options,Status},motion::Family,
    solid::{SweptField,SweepError},syntax,program,solve};

pub(super) fn rotation() -> Family {
    let (source,errors) = syntax::parse("point a hint(x: 0,y: 0)\npoint b hint(x: 0,y: 1)\n\
        ground a\nground b\nline axis(a,b)\nmotion turn(about: axis)\n");
    assert!(errors.is_empty());
    let mut model = program::elaborate(&source); assert!(model.ok());
    assert!(solve::solve(&mut model.sketch,Default::default()).success);
    Family::read(&model.sketch,0).unwrap()
}

pub(super) fn sphere() -> SpatialField {
    SpatialField::from(RevolvedField::new(F::disk([0.;2],1.).unwrap(),[3.,0.,0.],[0.,0.,1.]).unwrap())
}
pub(super) fn options() -> Options { Options {value_tolerance:1e-4,max_evaluations:50000} }
pub(super) fn includes(b: I,value: f64) {
    assert!(b.bounds()[0] <= value+1e-12 && b.bounds()[1] >= value-1e-12,"{b:?} excludes {value}");
}

#[test]
fn sphere_orbits_bound_a_complete_torus_and_finite_arc_end_caps() {
    // [-4,4] contains a full mathematical turn without treating binary64 pi
    // as an exact full-turn endpoint. Multiple covering passes form one union.
    let full = SweptField::new(sphere(),rotation(),I::new(-4.,4.).unwrap());
    let partial = SweptField::new(sphere(),rotation(),I::new(-1.,1.).unwrap());
    let mut a = full.evaluator(1024); let mut b = partial.evaluator(1024);
    for p in [[3_f64,0.,0.],[0.,3.,0.],[-3.,0.,0.],[0.;3],[4.2,0.,0.3],[2.,1.,2.]] {
        let got = a.bounds(point(p),options()).unwrap();
        assert_eq!(got.status,Status::Converged);
        includes(got.value,(p[0].hypot(p[1])-3.).hypot(p[2])-1.);
        let angle = p[1].atan2(p[0]).clamp(-1.,1.);
        let expected = (p[0]-3.*angle.cos()).hypot(p[1]-3.*angle.sin()).hypot(p[2])-1.;
        let got = b.bounds(point(p),options()).unwrap();
        assert_eq!(got.status,Status::Converged);
        includes(got.value,expected);
    }
}

#[test]
fn sweep_boxes_budgets_cache_caps_and_observers_preserve_enclosures() {
    let sweep = SweptField::new(sphere(),rotation(),I::new(-4.,4.).unwrap());
    let mut cached = sweep.evaluator(64); let mut uncached = sweep.evaluator(0);
    let p = point([3.,1.,0.]);
    let mut observed = 0;
    let a = cached.bounds_with_observer(p,options(),|d,_| {
        assert!(d.bounds()[0] >= -4. && d.bounds()[1] <= 4.); observed += 1;
    }).unwrap();
    let b = uncached.bounds(p,options()).unwrap();
    assert_eq!(a.value,b.value); assert_eq!(a.witness,b.witness);
    assert_eq!(a.evaluations,b.evaluations); assert_eq!(observed,a.evaluations);
    assert_eq!(cached.cached_poses(),64); assert_eq!(uncached.cached_poses(),0);
    cached.clear_cache(); assert_eq!(cached.cached_poses(),0);
    let coarse = cached.bounds(p,Options {max_evaluations:4,..options()}).unwrap();
    assert_eq!(coarse.status,Status::BudgetExhausted);
    assert!(coarse.value.bounds()[0] <= a.value.bounds()[0] && coarse.value.bounds()[1] >= a.value.bounds()[1]);
    let box_p = [I::new(2.9999,3.0001).unwrap(),I::new(-0.0001,0.0001).unwrap(),I::new(-0.0001,0.0001).unwrap()];
    let got = cached.bounds(box_p,Options {value_tolerance:0.001,..options()}).unwrap();
    assert_eq!(got.status,Status::Converged); assert!(got.value.bounds()[1] < 0.);
    for x in [2.9999_f64,3.,3.0001] { for y in [-0.0001_f64,0.,0.0001] { for z in [-0.0001_f64,0.,0.0001] {
        includes(got.value,(x.hypot(y)-3.).hypot(z)-1.);
    } } }
}

#[test]
fn fixed_sweep_domains_and_failures_remain_explicit() {
    let family = rotation();
    let domain = I::point(0.25).unwrap();
    let source = sphere();
    let placed = source.clone().transformed(&family,0.25).unwrap();
    let sweep = SweptField::new(source,family,domain);
    assert_eq!(sweep.domain(),domain);
    let mut query = sweep.evaluator(1); assert_eq!(query.domain(),domain);
    let p = point([4.,1.,0.]);
    let got = query.bounds(p,options()).unwrap();
    assert_eq!(got.status,Status::Converged);
    let reference = placed.bounds(p).unwrap();
    assert!(got.value.bounds()[0] <= reference.bounds()[0] && got.value.bounds()[1] >= reference.bounds()[1]);
    assert_eq!(query.bounds(p,Options {max_evaluations:3,..options()}).unwrap_err(),SweepError::InvalidOptions);
    let invalid = SweptField::new(sphere(),rotation(),I::point(f64::MAX).unwrap());
    assert_eq!(invalid.evaluator(0).bounds(p,options()).unwrap_err(),SweepError::Oracle(Error::Overflow));
    assert_eq!(query.bounds(point([f64::MAX;3]),options()).unwrap_err(),SweepError::Oracle(Error::Overflow));
}
