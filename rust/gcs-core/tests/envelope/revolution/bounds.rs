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
                    let cross = |a: [f64;3],b: [f64;3]| [a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];
                    let moment = cross(p.position,n);
                    let dn = dn.map(|x| x*std::f64::consts::TAU);
                    let a = cross(p.dv,n); let c = cross(p.position,dn);
                    let norm = |v: [f64;3]| v[0].hypot(v[1]).hypot(v[2]);
                    // The independent reference uses rounded Cartesian samples.
                    // Its cross products can leave cancellation residues for an
                    // exactly zero moment; allow a small term-scaled roundoff
                    // band in this comparison, not in the production enclosures.
                    let errors = [norm(p.position)*norm(n),norm(p.dv)*norm(n)+norm(p.position)*norm(dn)]
                        .map(|s| 64.*f64::EPSILON*s);
                    for k in 0..3 {
                        for (actual,bound,error) in [(moment[k],b.moment[k],errors[0]),(a[k]+c[k],b.moment_dv[k],errors[1])] {
                            let [lo,hi] = bound.bounds();
                            assert!(actual >= lo-error && actual <= hi+error,
                                "{name} u={u} v={v} axis={k}: {actual} vs {bound:?}, reference roundoff {error}");
                        }
                    }
                } }
            }
            assert_eq!(surface.bounds(I::new(-0.1,0.).unwrap(),I::ZERO).unwrap_err(),IntervalError::OutsideDomain);
            assert_eq!(surface.bounds(I::ZERO,I::new(0.,1.1).unwrap()).unwrap_err(),IntervalError::OutsideDomain);
        }
    }
}

#[test]
fn local_angular_charts_cross_only_full_revolution_seams() {
    use gcs_core::model::{Sense,SolidDef};
    let mut e = read();
    let si = e.map.ent_named("crown").unwrap().i();
    let placement = Motion::rotation([1.,2.,3.],0.7,0.).unwrap()
        .then(Motion::translation([3.,-4.,2.],[0.;3]).unwrap());
    for sense in [Sense::Cw,Sense::Ccw] {
        let SolidDef::Revolve {sense:s,..} = &mut e.sketch.solids[si].def else { unreachable!() };
        *s = sense;
        let source = patch(&e,"outer_round").placed(placement);
        assert!(source.is_periodic());
        assert!(source.at(0.5,-0.1).is_err(),"ordinary source coordinates stay restricted");
        for range in [[-0.2,0.2],[0.8,1.2]] {
            let chart = source.angular_chart(range).unwrap();
            let bounds = chart.bounds(I::new(0.3,0.7).unwrap(),I::new(range[0],range[1]).unwrap()).unwrap();
            for u in [0.3,0.5,0.7] { for i in 0..=16 {
                let v = range[0]+(range[1]-range[0])*i as f64/16.;
                let p = chart.at(u,v).unwrap();
                let q = source.at(u,v.rem_euclid(1.)).unwrap();
                near(p.position,q.position,1e-12);
                near(p.du,q.du,1e-12);
                near(p.dv,q.dv,1e-12);
                for (actual,b) in [(p.position,bounds.position),(p.du,bounds.du),(p.dv,bounds.dv)] {
                    for k in 0..3 { assert!(b[k].contains(actual[k])); }
                }
            } }
            // A rotating translation direction drives one isolated contact
            // continuously across this seam. Its unwrapped coordinate must not
            // jump to the other end of the canonical revolution interval.
            let seam = if range[0] < 0. { 0. } else { 1. };
            let sign = if sense == Sense::Cw { -1. } else { 1. };
            for theta in [-0.2_f64,0.,0.2] {
                let motion = Motion::translation([0.;3],placement.vector([theta.sin(),theta.cos(),0.])).unwrap();
                let roots = chart.contacts(0.5,motion,1e-9).unwrap();
                assert_eq!(roots.len(),1);
                assert!((roots[0].v-(seam-theta/(sign*std::f64::consts::TAU))).abs() < 1e-12);
                assert!(roots[0].contact.normal_velocity.abs() < 1e-9);
                assert!(source.contacts(0.5,motion,1e-9).unwrap().iter().any(|q|
                    q.contact.position.iter().zip(roots[0].contact.position).all(|(a,b)| (a-b).abs() < 1e-11)));
            }
        }
        for range in [[-0.3,0.1],[0.9,1.3],[-0.2,1.2],[0.4,0.2],[f64::NAN,0.2]] {
            assert!(source.angular_chart(range).is_err());
        }
        let restricted = source.angular_chart([0.2,0.8]).unwrap();
        assert!(!restricted.is_periodic());
        assert!(restricted.angular_chart([0.1,0.9]).is_err());
    }
    let SolidDef::Revolve {sweep,..} = &mut e.sketch.solids[si].def else { unreachable!() };
    sweep.value = std::f64::consts::PI;
    let partial = patch(&e,"outer_round");
    assert!(!partial.is_periodic());
    assert_eq!(partial.angular_chart_domain(),[0.,1.]);
    assert!(partial.angular_chart([-0.1,0.1]).is_err());
    assert!(partial.angular_chart([0.9,1.1]).is_err());
}
