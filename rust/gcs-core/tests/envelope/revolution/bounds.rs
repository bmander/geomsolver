use super::*;
use gcs_core::interval::{Interval as I,Error as IntervalError};

#[test]
fn full_revolved_source_boxes_enclose_positions_tangents_and_poles() {
    let e = read();
    let placement = Motion::rotation([1.,2.,3.],0.7,0.).unwrap()
        .then(Motion::translation([3.,-4.,2.],[0.;3]).unwrap());
    for name in ["outer","outer_round","inner","inner_round"] {
        for (surface,axis) in [(patch(&e,name),[0.,0.,1.]),
            (patch(&e,name).placed(placement),placement.vector([0.,0.,1.]))] {
            let length = axis[0].hypot(axis[1]).hypot(axis[2]);
            let axis = axis.map(|v| v/length);
            for (ur,vr) in [([0.,1.],[0.,1.]),([0.13,0.17],[0.31,0.33]),([1.,1.],[1.,1.])] {
                let b = surface.bounds(I::new(ur[0],ur[1]).unwrap(),I::new(vr[0],vr[1]).unwrap()).unwrap();
                for i in 0..=4 { for j in 0..=4 {
                    let u = ur[0]+(ur[1]-ur[0])*i as f64/4.; let v = vr[0]+(vr[1]-vr[0])*j as f64/4.;
                    let p = surface.at(u,v).unwrap();
                    for (actual,bounds) in [(p.position,b.position),(p.du,b.du),(p.dv,b.dv),
                        ([p.du[1]*p.dv[2]-p.du[2]*p.dv[1],p.du[2]*p.dv[0]-p.du[0]*p.dv[2],
                            p.du[0]*p.dv[1]-p.du[1]*p.dv[0]],b.normal)] {
                        for k in 0..3 { assert!(bounds[k].contains(actual[k]),"{name}: {} not in {:?}",actual[k],bounds[k]); }
                    }
                    // These known cone/torus normals rotate once about world Z
                    // (or the independently placed axis) as v traverses [0,1].
                    let n = [p.du[1]*p.dv[2]-p.du[2]*p.dv[1],p.du[2]*p.dv[0]-p.du[0]*p.dv[2],p.du[0]*p.dv[1]-p.du[1]*p.dv[0]];
                    let dn = [axis[1]*n[2]-axis[2]*n[1],axis[2]*n[0]-axis[0]*n[2],axis[0]*n[1]-axis[1]*n[0]];
                    for k in 0..3 { assert!(b.normal_dv[k].contains(dn[k]*std::f64::consts::TAU)); }
                } }
            }
            assert_eq!(surface.bounds(I::new(-0.1,0.).unwrap(),I::ZERO).unwrap_err(),IntervalError::OutsideDomain);
            assert_eq!(surface.bounds(I::ZERO,I::new(0.,1.1).unwrap()).unwrap_err(),IntervalError::OutsideDomain);
        }
    }
}
