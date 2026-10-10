//! **A field asked in millimetres** (`solid::Millimetres`): the B-rep kernel builds a swept body in
//! millimetres while a field reads the document's units, and this is the one place the two meet.
//! A disc of radius 2 cm, 6 mm deep, drawn in centimetres: asked in millimetres it holds a point
//! 15 mm out and not one 25 mm out, and reads the field's value in millimetres — where the field
//! itself, asked at the same numbers, stands 15 cm out and reads the disc as missed.
use crate::common::build;
use gcs_core::solid::{MaterialField,Millimetres,ProbeState,SpatialField};
use gcs_core::interval::minimum::Options;

const DISC: &str = "\
unit cm
use std
in std.front {
o := point
fix((0, 0)) o
c := circle(center: o) hint(r: 2)
radius(2cm) c
disc := face(c)
blank := solid(disc, depth: 6mm)
}
";

#[test]
fn a_field_asked_in_millimetres_is_the_field_asked_in_its_own_units() {
    let mut e = build(DISC);
    let r = gcs_core::solve::solve(&mut e.sketch,gcs_core::solve::SolveOpts::default());
    assert!(r.success,"{}",r.message);
    let blank = e.map.ent_named("blank").unwrap().i();
    let raw = SpatialField::read(&e.sketch,blank,1e-10).unwrap();
    let mm = Millimetres::<SpatialField>::read(&e.sketch,blank,1e-10).unwrap();
    // the slab's middle along the page's normal, whichever way the depth runs
    let y = if mm.value([0.,3.,0.]) < 0. { 3. } else { -3. };
    assert!(mm.value([0.,y,0.]) < 0.,"the disc's middle is inside it");
    // the same field, a point in millimetres divided into centimetres, its value multiplied back
    for p in [[15.,y,0.],[25.,y,0.],[0.,y,19.5],[3.,-y,-7.],[-40.,10.,2.]] {
        let want = raw.value(p.map(|x| x/10.))*10.;
        assert!((mm.value(p)-want).abs() <= 1e-12*want.abs().max(1.),"at {p:?}: {} against {want}",mm.value(p));
    }
    assert!(mm.value([15.,y,0.]) < 0. && mm.value([25.,y,0.]) > 0.,"15 mm out is in the disc, 25 mm out is not");
    // asked unconverted, the field reads 15 as centimetres: what `judge` once did
    assert!(raw.value([15.,y,0.]) > 0.);
    // the whole material field probed in millimetres, a ball of 1 mm about each point
    let material = Millimetres::<MaterialField>::read(&e.sketch,blank,1e-10).unwrap();
    let mut probe = material.evaluator(64);
    let options = Options {value_tolerance:0.25,max_evaluations:40000};
    assert_eq!(probe.probe([15.,y,0.],[1.,0.,0.],1.,options).unwrap(),ProbeState::InteriorBall);
    assert_eq!(probe.probe([25.,y,0.],[1.,0.,0.],1.,options).unwrap(),ProbeState::ExteriorBall);
    // a ball reaching past the rim is neither: its radius is read in millimetres too
    assert!(!matches!(probe.probe([19.5,y,0.],[1.,0.,0.],1.,options).unwrap(),ProbeState::InteriorBall|ProbeState::ExteriorBall));
}
