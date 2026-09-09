use super::*;
use gcs_core::{envelope::{self,Motion,Error},solid::{SweepContacts,RevolvedSurface}};
use std::f64::consts::PI;
mod meridian;
mod curves;

fn swept(extra: &str) -> program::Elaborated {
    read(&format!("{SOURCE}\n{extra}\nsolid swept(tool,under: generating,from: -60deg,to: 60deg)\n"))
}

#[test]
fn source_contacts_of_an_orbiting_sphere_are_the_exact_torus() {
    let e = swept("");
    let sweep = SweepContacts::read(&e.sketch,e.map.ent_named("swept").unwrap().i(),1e-10).unwrap();
    assert_eq!(sweep.patches().len(),1,"the axis diameter is not a boundary face");
    for u in [0.1,0.3,0.5,0.7,0.9] { for t in [-0.7,0.,0.7] {
        let contacts = sweep.at(0,u,t,1e-10).unwrap();
        assert_eq!(contacts.len(),2);
        for c in contacts {
            let p = c.contact.position;
            assert!(((p[0].hypot(p[1])-3.).hypot(p[2])-1.).abs() < 1e-12);
            assert!((p[0]*t.sin()-p[1]*t.cos()).abs() < 1e-12);
            assert!(c.contact.normal_velocity.abs() < 1e-12);
        }
    } }
    assert_eq!(sweep.at(0,0.5,2.,1e-10).unwrap_err(),Error::OutsideDomain);
    assert_eq!(sweep.at(1,0.5,0.,1e-10).unwrap_err(),Error::OutsideDomain);
    assert_eq!(sweep.at(0,0.5,0.,0.).unwrap_err(),Error::InvalidOptions);
}

#[test]
fn contact_coefficients_include_translation_and_both_moving_frames() {
    let e = swept("");
    let sweep = SweepContacts::read(&e.sketch,e.map.ent_named("swept").unwrap().i(),1e-10).unwrap();
    let surface = &sweep.patches()[0];
    let (mut checked,mut empty) = (0,0);
    for u in [0.15,0.35,0.5,0.65,0.85] { for t in [-0.4,0.,0.3] {
        let motion = Motion::rotation([1.,2.,3.],t,1.).unwrap()
            .then(Motion::translation([2.,-3.,1.],[0.4,-0.2,0.3]).unwrap())
            .then(Motion::rotation([1.,0.,0.],-2.*t,-2.).unwrap().inverse());
        let roots = surface.contacts(u,motion,1e-10).unwrap();
        let values: Vec<_> = (0..=1000).map(|i| envelope::contact(
            surface.at(u,i as f64/1000.).unwrap(),motion).unwrap().normal_velocity).collect();
        let crossings = values.windows(2).filter(|v| v[0]*v[1] < 0.).count();
        assert_eq!(roots.len(),crossings);
        if roots.is_empty() { empty += 1; }
        for root in roots {
            assert!(root.contact.normal_velocity.abs() < 1e-10);
            checked += 1;
        }
    } }
    assert!(checked > 0 && empty > 0,"the fixture must exercise both contact and no-contact rings");
}

#[test]
fn source_placements_and_boolean_sharing_preserve_contact_geometry() {
    let src = format!("{SOURCE}\nmotion index(about: axis,phase: 90deg)\n\
        solid placed(tool,under: index,at: 0deg)\n\
        solid outside(tool)\nplaced cut outside\n\
        solid lens(tool)\noutside cut lens\n\
        solid swept(lens,under: generating,from: -60deg,to: 60deg)\n");
    let e = read(&src);
    let sweep = SweepContacts::read(&e.sketch,e.map.ent_named("swept").unwrap().i(),1e-10).unwrap();
    assert_eq!(sweep.patches().len(),2,"shared Boolean operands contribute their faces once");
    let mut centers = Vec::new();
    for (i,patch) in sweep.patches().iter().enumerate() {
        let a = patch.at(0.,0.).unwrap().position;
        let b = patch.at(1.,0.).unwrap().position;
        centers.push(std::array::from_fn::<_,3,_>(|k| (a[k]+b[k])/2.));
        for root in sweep.at(i,0.5,0.3,1e-10).unwrap() {
            let p = root.contact.position;
            assert!(((p[0].hypot(p[1])-3.).abs()-1.).abs() < 1e-12);
            let angle = centers[i][1].atan2(centers[i][0])+0.3;
            assert!((p[0]*angle.sin()-p[1]*angle.cos()).abs() < 1e-12);
            let v = root.contact.velocity;
            assert!((v[0]+p[1]).abs() < 1e-12 && (v[1]-p[0]).abs() < 1e-12);
        }
    }
    assert!(centers.iter().any(|p| (p[0]-3.).abs() < 1e-12 && p[1].abs() < 1e-12));
    assert!(centers.iter().any(|p| p[0].abs() < 1e-12 && (p[1]-3.).abs() < 1e-12));
}

#[test]
fn restricted_spans_empty_rings_and_topology_events_are_distinguished() {
    let e = swept("surface half(tool,rim,from: 90deg,to: 270deg)");
    let surface = RevolvedSurface::named(&e.sketch,e.map.ent_named("half").unwrap().i()).unwrap();
    let turn = Motion::rotation([0.,0.,1.],0.,1.).unwrap();
    let roots = surface.contacts(0.5,turn,1e-10).unwrap();
    assert_eq!(roots.len(),1); assert!((roots[0].v-0.5).abs() < 1e-12);
    assert_eq!(surface.contacts(0.5,Motion::identity(),1e-10).unwrap_err(),Error::Degenerate);
    let translation = Motion::translation([0.;3],[0.,0.,1.]).unwrap();
    assert!(surface.contacts(0.25,translation,1e-10).unwrap().is_empty());
    assert_eq!(surface.contacts(0.5,translation,1e-10).unwrap_err(),Error::Degenerate);
    // At u=1/4 the sphere normal has equal radial and axial components.
    // Translation along (1,0,1) makes the contact equation have a double root.
    let tangent = Motion::translation([0.;3],[1.,0.,1.]).unwrap();
    assert_eq!(surface.contacts(0.25,tangent,1e-10).unwrap_err(),Error::Degenerate);
    assert!((sweep_span(&e)-2.*PI/3.).abs() < 1e-12);
}
fn sweep_span(e: &program::Elaborated) -> f64 {
    let sweep = SweepContacts::read(&e.sketch,e.map.ent_named("swept").unwrap().i(),1e-10).unwrap();
    let [lo,hi] = sweep.domain(); hi-lo
}

#[test]
fn analytic_branch_labels_do_not_swap_at_the_periodic_seam() {
    let e = swept("");
    let sweep = SweepContacts::read(&e.sketch,e.map.ent_named("swept").unwrap().i(),1e-10).unwrap();
    let patch = &sweep.patches()[0];
    let mut previous: Option<[[f64;3];2]> = None;
    for i in -10..=10 {
        let phase = PI/2.+i as f64*0.001;
        let motion = Motion::translation([0.;3],[phase.cos(),phase.sin(),0.]).unwrap();
        let roots = patch.contacts(0.5,motion,1e-10).unwrap();
        assert_eq!(roots.len(),2);
        let p = [roots[0].contact.position,roots[1].contact.position];
        assert_eq!([roots[0].branch,roots[1].branch],[0,1]);
        if let Some(previous) = previous {
            for j in 0..2 {
                let d: [f64;3] = std::array::from_fn(|k| p[j][k]-previous[j][k]);
                assert!(d[0].hypot(d[1]).hypot(d[2]) < 0.002);
            }
        }
        previous = Some(p);
    }
}

#[test]
fn clockwise_source_revolution_keeps_the_same_contact_set() {
    let a = swept("");
    let b = read(&format!("{}\nsolid swept(tool,under: generating,from: -60deg,to: 60deg)\n",
        SOURCE.replace("about: diameter)","about: diameter,sense: cw)")));
    let read = |e: &program::Elaborated| SweepContacts::read(&e.sketch,
        e.map.ent_named("swept").unwrap().i(),1e-10).unwrap();
    let (a,b) = (read(&a),read(&b));
    for u in [0.1,0.4,0.8] {
        let left = a.at(0,u,0.3,1e-10).unwrap();
        let right = b.at(0,u,0.3,1e-10).unwrap();
        assert_eq!(left.len(),right.len());
        for p in left {
            assert!(right.iter().any(|q| {
                let a = p.contact.position; let b = q.contact.position;
                (a[0]-b[0]).hypot(a[1]-b[1]).hypot(a[2]-b[2]) < 1e-12
            }));
        }
    }
}
