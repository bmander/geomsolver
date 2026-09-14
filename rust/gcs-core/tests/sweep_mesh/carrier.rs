//! Analytic carrier identity and interval-regular maps; no fixture atlas in production.
use super::{harness,creases,tools,motions};
use gcs_core::solid::{SweepContacts,swept_boundary::arrangement::carrier::{
    Correspondence,Band,Options,Error,Limit}};
fn read(source: &str) -> SweepContacts {
    let e = harness::read(source);
    SweepContacts::read(&e.sketch,harness::solid(&e,"swept"),1e-10).unwrap()
}
fn bands(s: &SweepContacts) -> [Band;2] {
    [Band {edge:0,domain:[[0.01,0.1],s.domain()]},
        Band {edge:0,domain:[[0.9,0.99],s.domain()]}]
}
fn options() -> Options { Options {max_depth:16,max_cells:4096} }
fn partition(c: &Correspondence<'_>) {
    for (i,band) in c.bands().iter().enumerate() {
        let boxes: Vec<_> = c.regions().iter().filter(|r| r.band() == i).map(|r| r.domain())
            .chain(c.unresolved().iter().filter(|r| r.band == i).map(|r| r.domain)).collect();
        let area = |b: [[f64;2];2]| (b[0][1]-b[0][0])*(b[1][1]-b[1][0]);
        assert!((boxes.iter().map(|&b| area(b)).sum::<f64>()-area(band.domain)).abs() < 1e-12);
        for (j,a) in boxes.iter().enumerate() {
            for k in 0..2 { assert!(a[k][0] >= band.domain[k][0] && a[k][1] <= band.domain[k][1]); }
            for b in &boxes[j+1..] {
                assert!((0..2).any(|k| a[k][1] <= b[k][0] || b[k][1] <= a[k][0]));
            }
        }
    }
}
#[test]
fn two_rim_bands_have_one_analytic_carrier_and_distinct_inverse_maps() {
    let s = read(&creases::tumbling_cylinder());
    let c = Correspondence::new(&s,&bands(&s),options()).unwrap();
    assert_eq!(c.centre(),[3.,0.,0.]); assert_eq!(c.radius_terms(),[0.,1.,1.]);
    assert!(c.unresolved().is_empty(),"{:?}",c.unresolved());
    assert_eq!(c.regions().len(),2); partition(&c);
    assert_eq!(c.regions()[0].orientation(),-c.regions()[1].orientation());
    for r in c.regions() { assert!(r.derivatives().iter().all(|d| !d.contains(0.))); }
    // These source generators meet on y=0 at different times. The nominal
    // sphere and inverse angular relation are a judge only, not construction input.
    for t in [0.015,0.03,0.05,0.08] {
        let q = s.source_at(gcs_core::solid::sweep_source::SourcePoint {
            source:gcs_core::solid::sweep_source::Source::Edge(0),parameters:[t,0.]},0.).unwrap();
        let roll = (q.position[1]/q.position[2]).atan();
        for seed in [[0.91,-0.4],[0.98,0.4]] {
            let mapped = c.transfer(0,1,[t,roll],seed,1e-10).unwrap();
            let a = &mapped[0]; let b = &mapped[1];
            assert!(harness::distance(a.evaluation.position,b.evaluation.position) < 1e-10);
            assert!((b.native[0]-(1.-t)).abs() < 1e-10);
            assert!((b.native[1]+roll).abs() < 1e-10);
            assert!(a.evaluation.position[1].abs() < 1e-12);
            let p = a.evaluation.position;
            assert!(((p[0]-3.).powi(2)+p[1]*p[1]+p[2]*p[2]-2.).abs() < 1e-12);
            for m in mapped {
                let q = s.source_at(m.source,m.native[1]).unwrap();
                assert_eq!(m.evaluation.position,q.position);
                assert_eq!(m.evaluation.faces.len(),2);
                assert!(harness::distance(q.faces[0].position,q.faces[1].position) < 1e-12);
            }
        }
    }
}
#[test]
fn mapping_is_stable_under_consumer_order_and_motion_rate() {
    for (rate,phase) in [(1.,0.),(-1.,17.),(0.5,-9.)] {
        let motion = motions::TUMBLE.replace("motion turn(about: tumbler)",
            &format!("motion turn(about: tumbler, ratio: {rate}, phase: {phase}deg)"));
        let s = read(&format!("{}{}{}",tools::CYLINDER,motion,motions::swept("turn",-60.,60.)));
        let b = bands(&s); let c = Correspondence::new(&s,&b,options()).unwrap();
        let reversed = Correspondence::new(&s,&[b[1],b[0]],options()).unwrap();
        assert!(c.unresolved().is_empty()); assert!(reversed.unresolved().is_empty());
        for (i,r) in c.regions().iter().enumerate() {
            let d = r.domain(); let p = d.map(|b| 0.5*(b[0]+b[1]));
            let q = c.forward(i,p).unwrap();
            let inverse = c.inverse(i,q.coordinates,p,1e-10).unwrap();
            assert!(harness::distance(q.evaluation.position,inverse.evaluation.position) < 1e-10);
            let j = reversed.regions().iter().position(|r2|
                r2.band() == 1-r.band() && r2.domain() == d && r2.chart() == r.chart()).unwrap();
            assert_eq!(q.evaluation.position,reversed.forward(j,p).unwrap().evaluation.position);
        }
    }
}
#[test]
fn folds_angular_chart_changes_and_budgets_keep_the_native_cover() {
    let s = read(&format!("{}{}{}",tools::CYLINDER,motions::TUMBLE,motions::swept("turn",-180.,180.)));
    let b = [Band {edge:0,domain:[[0.,1.],s.domain()]}];
    for budget in [0,9,4096] {
        let c = Correspondence::new(&s,&b,Options {max_depth:10,max_cells:budget}).unwrap();
        partition(&c); assert!(c.evaluations() <= budget); assert!(!c.unresolved().is_empty());
        if budget == 0 { assert!(c.regions().is_empty()); assert_eq!(c.unresolved()[0].reason,Limit::Budget); }
        if budget == 4096 {
            assert!(!c.regions().is_empty());
            assert!(c.unresolved().iter().any(|r| r.reason == Limit::FoldOrPole));
            assert!(c.regions().iter().any(|r| r.chart().positive));
            assert!(c.regions().iter().any(|r| !r.chart().positive));
            for (i,r) in c.regions().iter().enumerate() {
                let p = r.domain().map(|b| 0.5*(b[0]+b[1]));
                let m = c.forward(i,p).unwrap();
                let inverse = c.inverse(i,m.coordinates,p,1e-9).unwrap();
                assert!(harness::distance(m.evaluation.position,inverse.evaluation.position) < 1e-9);
            }
        }
    }
}
#[test]
fn a_nearby_skew_axis_is_not_snapped_to_the_shared_sphere() {
    // Moving the axis out of the circle's centre/axis plane breaks the relation.
    // Even the smallest nonzero offset used here must refuse structural identity.
    for offset in [0.,1e-12,1e-6] {
        let motion = format!("private point h0 in std.front hint(x: 3, y: {offset})\n\
            private point h1 in std.front hint(x: 4, y: {offset})\n\
            ground h0\nground h1\nconstruction line hinge(h0, h1)\nmotion turn(about: hinge)\n");
        let s = read(&format!("{}{}{}",tools::CYLINDER,motion,motions::swept("turn",-30.,30.)));
        let c = Correspondence::new(&s,&bands(&s),options());
        if offset == 0. { assert!(c.is_ok(),"{:?} motion={:?}",c.err(),s.motion()); }
        else { assert!(matches!(c,Err(Error::UnestablishedCarrier))); }
    }
}
#[test]
fn chart_membership_and_unsupported_geometry_do_not_manufacture_maps() {
    let s = read(&creases::tumbling_cylinder());
    let c = Correspondence::new(&s,&bands(&s),options()).unwrap();
    assert!(c.forward(0,[0.5,0.]).is_err());
    assert!(c.forward(0,[0.03,2.]).is_err());
    assert!(c.inverse(0,[100.,100.],[0.03,0.],1e-10).is_err());
    assert!(c.inverse(0,[3.9,0.],[0.03,0.],f64::NAN).is_err());
    let mut b = bands(&s); b[0].domain[0] = [-0.01,0.01];
    assert!(matches!(Correspondence::new(&s,&b,options()),Err(Error::InvalidInput)));
    let box_s = read(&format!("{}{}{}",tools::BOX,motions::TUMBLE,motions::swept("turn",-30.,30.)));
    assert!(matches!(Correspondence::new(&box_s,&bands(&box_s),options()),Err(Error::UnsupportedSource)));
    let translating = read(&format!("{}{}{}",tools::CYLINDER,motions::slide_z(1.),motions::swept("feed",-30.,30.)));
    assert!(matches!(Correspondence::new(&translating,&bands(&translating),options()),Err(Error::UnsupportedMotion)));
}

#[test]
fn a_shared_carrier_does_not_identify_opposite_hemispheres_or_roll_domains() {
    let s = read(&creases::tumbling_cylinder());
    let mut b = bands(&s); b[1] = Band {edge:1,domain:b[0].domain};
    let c = Correspondence::new(&s,&b,options()).unwrap();
    assert_eq!(c.regions().len(),2); assert!(c.unresolved().is_empty());
    assert_ne!(c.regions()[0].chart().positive,c.regions()[1].chart().positive);
    assert!(matches!(c.transfer(0,1,[0.03,0.],[0.03,0.],1e-10),Err(Error::OutsideDomain)));
    let mut b = bands(&s); b[1].domain[1] = [0.1,0.2];
    let c = Correspondence::new(&s,&b,options()).unwrap();
    assert!(c.transfer(0,1,[0.03,0.19],[0.97,0.15],1e-10).is_err());
    // A tilted axis is not silently snapped to a coordinate direction.
    let s = read(&format!("{}{}{}",tools::CYLINDER,motions::turn_about(3.,0.,4.,1e-12),
        motions::swept("turn",-30.,30.)));
    assert!(matches!(Correspondence::new(&s,&bands(&s),options()),Err(Error::UnsupportedMotion)));
}

#[test]
fn a_source_circle_through_the_carrier_pole_keeps_its_singular_limit() {
    let tool = tools::CYLINDER.replace("y: -1)","y: 0)").replace("y: 1)","y: 2)");
    let s = read(&format!("{tool}{}{}",motions::TUMBLE,motions::swept("turn",-30.,30.)));
    let b = [Band {edge:0,domain:[[0.,0.05],s.domain()]}];
    let c = Correspondence::new(&s,&b,Options {max_depth:10,max_cells:1000}).unwrap();
    assert_eq!(c.radius_terms(),[0.,0.,1.]); partition(&c);
    assert!(!c.regions().is_empty());
    assert!(c.unresolved().iter().any(|r| r.domain[0][0] == 0. && r.reason == Limit::FoldOrPole));
    assert!(c.regions().iter().all(|r| r.domain()[0][0] > 0.));
}
