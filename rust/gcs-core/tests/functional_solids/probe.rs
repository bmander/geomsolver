use super::*;
use super::sweeps::{rotation,sphere,options};
use gcs_core::{interval::minimum::{Options,Status},solid::{MaterialField,SweptField,ProbeState,SweepError}};

fn swept(domain: [f64;2]) -> MaterialField {
    SweptField::new(sphere(),rotation(),I::new(domain[0],domain[1]).unwrap()).into()
}
fn ball(radius: f64) -> MaterialField {
    SpatialField::from(RevolvedField::new(F::disk([0.;2],radius).unwrap(),[0.;3],[0.,0.,1.]).unwrap()).into()
}

#[test]
fn whole_motion_brackets_torus_sides_and_rejects_a_covered_end_cap() {
    let mut full = swept([-4.,4.]).evaluator(1024);
    for (p,n,state) in [([4.,0.,0.],[1.,0.,0.],ProbeState::OutwardBracket),
        ([4.,0.,0.],[-1.,0.,0.],ProbeState::InwardBracket),
        ([3.,0.,0.],[1.,0.,0.],ProbeState::InteriorBall),
        ([0.;3],[1.,0.,0.],ProbeState::ExteriorBall)] {
        let result = full.probe(point(p),n,0.01,options()).unwrap();
        assert_eq!(result.state,state,"{p:?}: {result:?}");
        assert_eq!(result.center.sweeps[0].domain.bounds(),[-4.,4.]);
    }
    // Sphere center (3,0,0), radius 1, at the end of an arc [-1,0]. The forward
    // spherical cap at (3,1,0) is real for that finite sweep, but is swallowed
    // when the center travels around the full circle: sqrt(10)-3 < 1.
    let mut partial = swept([-1.,0.]).evaluator(1024);
    let p = point([3.,1.,0.]);
    assert_eq!(partial.probe(p,[0.,1.,0.],0.01,options()).unwrap().state,ProbeState::OutwardBracket);
    let covered = full.probe(p,[0.,1.,0.],0.01,options()).unwrap();
    assert_eq!(covered.state,ProbeState::InteriorBall);
    assert!(covered.sides.is_none());
    assert!(covered.center.sweeps[0].minimum.witness > 0.);
}

#[test]
fn candidate_boxes_and_boolean_cut_orientation_keep_the_same_contract() {
    let torus = swept([-4.,4.]);
    let mut removal = torus.clone().evaluator(1024);
    let mut body = ball(3.5).difference(torus).unwrap().evaluator(1024);
    let p = [I::new(1.999,2.001).unwrap(),I::new(-0.001,0.001).unwrap(),I::new(-0.001,0.001).unwrap()];
    assert_eq!(removal.probe(p,[-1.,0.,0.],0.01,options()).unwrap().state,ProbeState::OutwardBracket);
    assert_eq!(body.probe(p,[-1.,0.,0.],0.01,options()).unwrap().state,ProbeState::InwardBracket);
    // Normalization and offsets enclose the mathematical unit direction, even
    // when its input scale is far outside the safe range for an unscaled square.
    let mut sphere = ball(2.).evaluator(0);
    for length in [1e-300,1.,1e300] {
        let r = sphere.probe(point([2.,0.,0.]),[length,0.,0.],0.01,options()).unwrap();
        assert_eq!(r.state,ProbeState::OutwardBracket);
        assert!(r.side_boxes[0][0].contains(1.99) && r.side_boxes[1][0].contains(2.01));
    }
}

#[test]
fn field_zero_and_budget_exhaustion_do_not_become_boundary_claims() {
    let sphere = ball(2.);
    let mut empty = sphere.clone().difference(sphere).unwrap().evaluator(0);
    assert_eq!(empty.probe(point([2.,0.,0.]),[1.,0.,0.],0.01,options()).unwrap().state,ProbeState::Unresolved);
    let half = |sign| SpatialField::from(RevolvedField::new(
        F::half_plane([0.;2],[0.,sign]).unwrap(),[0.;3],[0.,0.,1.]).unwrap());
    let mut whole = MaterialField::from(half(1.).union(half(-1.)).unwrap()).evaluator(0);
    assert_eq!(whole.probe(point([0.;3]),[0.,0.,1.],0.01,options()).unwrap().state,ProbeState::Unresolved);
    let mut torus = swept([-4.,4.]).evaluator(0);
    let coarse = torus.probe(point([4.,0.,0.]),[1.,0.,0.],0.01,
        Options {max_evaluations:4,..options()}).unwrap();
    assert_eq!(coarse.state,ProbeState::Unresolved);
    assert!(coarse.center.sweeps.iter().any(|q| q.minimum.status == Status::BudgetExhausted));
    let fine = torus.probe(point([4.,0.,0.]),[1.,0.,0.],0.01,options()).unwrap();
    assert_eq!(fine.state,ProbeState::OutwardBracket);
    for (normal,distance,opts) in [([0.;3],0.01,options()),([1.,0.,0.],0.,options()),
        ([f64::NAN,0.,0.],0.01,options()),([1.,0.,0.],0.01,Options {max_evaluations:0,..options()})] {
        assert!(matches!(whole.probe(point([0.;3]),normal,distance,opts),Err(SweepError::InvalidOptions)));
    }
}
