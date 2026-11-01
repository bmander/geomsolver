//! **A surface's contact under a screw, the kernel's** (#721 F2): every kernel `Surface` answers
//! where it is in contact under an instantaneous twist, `n · v = 0` (`brep::contact`). In closed
//! form on a ring of a surface of revolution (`A cos u + B sin u + C`), along a ruling whose normal
//! stands still (`a + b v`) and across a plane (`a + b u + c v`); numerically on any other. Each
//! closed form is held here to the contact function itself (`Surface::normal_velocity`, the unit
//! normal dotted with the twist's velocity) at sampled parameters under seeded twists, each ring's
//! roots to being in contact and to the numerical reading of the same ring, and the degenerate
//! contacts the walk is built round (#684 X, AF, O) to the root kind that names them. Sampled.
use gcs_core::brep::contact::{Form,RingRoots,RulingRoot};
use gcs_core::brep::geom::{Blend,Carry,Curve,Frame,Surface};
use gcs_core::brep::trajectory::{Motion,Twist};
use gcs_core::envelope::{self,SurfacePoint};
use gcs_core::rng::Rng;
use gcs_core::space::{cross,dot,unit};
use std::f64::consts::{PI,TAU};
use std::sync::Arc;

/// A frame placed off the world's, turned and moved, so that nothing reads a world axis.
fn frame(rng: &mut Rng) -> Frame {
    let z = unit([rng.uniform(-1.,1.),rng.uniform(-1.,1.),rng.uniform(-1.,1.)]);
    Frame::new([rng.uniform(-20.,20.),rng.uniform(-20.,20.),rng.uniform(-20.,20.)],z,[rng.uniform(-1.,1.),1.,rng.uniform(-1.,1.)])
}

fn twist(rng: &mut Rng) -> Twist {
    Twist {omega:[rng.uniform(-1.,1.),rng.uniform(-1.,1.),rng.uniform(-1.,1.)],
        v0:[rng.uniform(-10.,10.),rng.uniform(-10.,10.),rng.uniform(-10.,10.)]}
}

/// One of each surface the closed forms read, placed by `f`, and the stretch of `v` it is read over.
fn surfaces(f: Frame) -> Vec<(&'static str,Surface,[f64;2])> {
    let arm = f.at([12.,0.,3.]);
    vec![
        ("plane",Surface::Plane(f),[-5.,5.]),
        ("cylinder",Surface::Cylinder(f,4.),[-5.,5.]),
        ("cone",Surface::Cone(f,4.,0.4),[-3.,5.]),
        ("sphere",Surface::Sphere(f,6.),[-1.4,1.4]),
        ("torus",Surface::Torus(f,9.,2.5),[0.,TAU]),
        ("extruded circle",Surface::Extrusion(f,Arc::new(Curve::Circle(Frame {o:f.at([3.,1.,0.]),..f},5.))),[-5.,5.]),
        ("extruded line",Surface::Extrusion(f,Arc::new(Curve::Line {p:f.o,d:unit(f.dir([1.,2.,0.]))})),[-5.,5.]),
        ("revolved line",Surface::Revolution(f,Arc::new(Curve::Line {p:arm,d:unit(f.dir([0.5,0.,1.]))})),[-4.,4.]),
        ("revolved circle",Surface::Revolution(f,Arc::new(Curve::Circle(Frame::new(arm,f.y,f.x),2.))),[0.,TAU]),
    ]
}

/// `a` and `b` agree to a part in 10⁹ of the size of what they read.
fn agree(a: f64,b: f64,what: &str) {
    assert!((a-b).abs() <= 1e-9*(1.+b.abs()),"{what}: {a} against {b}");
}

#[test]
fn each_closed_form_is_the_contact_function() {
    let mut rng = Rng::new(721);
    let mut read = [0usize;3];
    for _ in 0..6 {
        let f = frame(&mut rng);
        for (name,s,[v0,v1]) in surfaces(f) {
            let w = twist(&mut rng);
            for i in 0..9 {
                let u = TAU*(i as f64+0.3)/9.;
                for j in 0..7 {
                    let v = v0+(v1-v0)*(j as f64+0.4)/7.;
                    let Some(g) = s.normal_velocity([u,v],&w) else { continue };
                    if let Some(ring) = s.ring_contact(v,&w) { agree(ring.at(u),g,&format!("{name}'s ring at {v}")); read[0] += 1 }
                    if let Some(ruling) = s.ruling_contact(u,&w) {
                        // the ruling's normal is the surface's up to its sign, which turns past an apex
                        let own = s.normal([u,0.]).or_else(|| s.normal([u,1.])).unwrap();
                        agree(dot(own,s.normal([u,v]).unwrap()).signum()*ruling.at(v),g,&format!("{name}'s ruling at {u}"));
                        read[1] += 1
                    }
                    if let Some([a,b,c]) = s.plane_contact(&w) { agree(a+b*u+c*v,g,&format!("{name} at ({u}, {v})")); read[2] += 1 }
                }
            }
            match s.contact_form() {
                Form::Linear => assert!(s.plane_contact(&w).is_some() && s.ruling_contact(0.,&w).is_some(),"{name}"),
                Form::Rings {rulings} => assert!(s.ring_contact(0.5*(v0+v1),&w).is_some() && s.ruling_contact(0.,&w).is_some() == rulings,"{name}"),
                Form::Rulings => assert!(s.ruling_contact(0.,&w).is_some() && s.ring_contact(0.,&w).is_none(),"{name}"),
                Form::Numeric => panic!("{name} has a closed form"),
            }
        }
    }
    eprintln!("rings {}, rulings {}, planes {}",read[0],read[1],read[2]);
    assert!(read.iter().all(|&n| n > 50),"{read:?}");
}

#[test]
fn a_rings_roots_are_in_contact_and_found_numerically_too() {
    let mut rng = Rng::new(7210);
    let mut two = 0;
    for _ in 0..6 {
        let f = frame(&mut rng);
        for (name,s,[v0,v1]) in surfaces(f) {
            let w = twist(&mut rng);
            for j in 0..5 {
                let v = v0+(v1-v0)*(j as f64+0.5)/5.;
                let Some(ring) = s.ring_contact(v,&w) else { continue };
                let numeric = s.contact_along([0.,v],[TAU,v],&w,720,1e-14);
                match ring.roots(1e-12) {
                    RingRoots::Two(us) => {
                        two += 1;
                        for u in us { assert!(s.normal_velocity([u,v],&w).unwrap().abs() <= 1e-9*(1.+ring.c.abs()),"{name} at {u}") }
                        assert_eq!(numeric.len(),2,"{name}: {us:?} against {numeric:?}");
                        let mut closed = us.map(|u| u/TAU);
                        closed.sort_by(f64::total_cmp);
                        for k in 0..2 { assert!((closed[k]-numeric[k]).abs() <= 1e-9,"{name}: {closed:?} against {numeric:?}") }
                    }
                    RingRoots::None => assert!(numeric.is_empty(),"{name}: none against {numeric:?}"),
                    r => panic!("{name}: a seeded twist met a degenerate ring, {r:?}"),
                }
            }
        }
    }
    assert!(two > 40,"{two}");
}

#[test]
fn a_rulings_root_is_in_contact() {
    let mut rng = Rng::new(72100);
    let mut one = 0;
    for _ in 0..6 {
        let f = frame(&mut rng);
        for (name,s,_) in surfaces(f) {
            let w = twist(&mut rng);
            for i in 0..8 {
                let u = TAU*(i as f64+0.2)/8.;
                let Some(ruling) = s.ruling_contact(u,&w) else { continue };
                let RulingRoot::One(v) = ruling.root(1e-12) else { panic!("{name}: a seeded twist met a degenerate ruling") };
                if let Some(g) = s.normal_velocity([u,v],&w) { assert!(g.abs() <= 1e-9*(1.+ruling.a.abs()),"{name} at ({u}, {v}): {g}"); one += 1 }
            }
        }
    }
    assert!(one > 100,"{one}");
}

#[test]
fn the_degenerate_contacts_are_read_as_what_they_are() {
    let f = Frame::new([1.,2.,3.],[0.,0.,1.],[1.,0.,0.]);
    let along_z = Twist {omega:[0.;3],v0:[0.,0.,2.]};
    let about_z = Twist {omega:[0.,0.,0.7],v0:cross([1.,2.,3.],[0.,0.,0.7])};
    let across = Twist {omega:[0.;3],v0:[3.,0.,0.]};
    let tol = 1e-12;
    // a cylinder slid along its axis, or turned about it: every ring in contact all round (#684 X)
    let cylinder = Surface::Cylinder(f,4.);
    assert_eq!(cylinder.ring_contact(1.,&along_z).unwrap().roots(tol),RingRoots::All);
    assert_eq!(cylinder.ring_contact(1.,&about_z).unwrap().roots(tol),RingRoots::All);
    // slid square to its axis: its silhouette at a quarter turn either side of the slide, each of
    // those rulings in contact all along it and every other nowhere (#684 AF)
    let RingRoots::Two(us) = cylinder.ring_contact(1.,&across).unwrap().roots(tol) else { panic!() };
    for u in us { assert!((u-PI/2.).abs() < 1e-12 || (u-3.*PI/2.).abs() < 1e-12,"{us:?}") }
    assert_eq!(cylinder.ruling_contact(PI/2.,&across).unwrap().root(tol),RulingRoot::All);
    assert_eq!(cylinder.ruling_contact(0.3,&across).unwrap().root(tol),RulingRoot::None);
    // a sphere turned about a line through its centre: in contact all round every ring
    let sphere = Surface::Sphere(f,6.);
    let through = Twist {omega:[0.3,-0.2,0.5],v0:cross([1.,2.,3.],[0.3,-0.2,0.5])};
    for v in [-1.,0.,0.7] { assert_eq!(sphere.ring_contact(v,&through).unwrap().roots(tol),RingRoots::All) }
    // a plane slid in itself: all of it in contact (#684 O); slid along its normal, none of it
    let plane = Surface::Plane(f);
    assert_eq!(plane.plane_contact(&across).unwrap(),[0.;3]);
    assert!(plane.plane_contact(&along_z).unwrap()[0] > 0.);
    // a torus slid along its axis: its equators in contact all round, every other ring nowhere,
    // the characteristic the ring of greatest radius alone (#684 X)
    let torus = Surface::Torus(f,9.,2.5);
    for v in [0.,PI] { assert_eq!(torus.ring_contact(v,&along_z).unwrap().roots(1e-9),RingRoots::All) }
    for v in [0.4,2.,4.] { assert_eq!(torus.ring_contact(v,&along_z).unwrap().roots(1e-9),RingRoots::None) }
}

#[test]
fn a_free_form_face_is_read_numerically() {
    let f = Frame::new([0.;3],[0.,0.,1.],[1.,0.,0.]);
    // a ruled blend between two circles of unequal radius, carried up a screw: no closed form
    let blend = Blend {a:Curve::Circle(f,5.),ta:[0.,TAU],b:Curve::Circle(Frame {o:[1.,0.,0.],..f},3.),tb:[0.,TAU],
        carry:Carry::Screw {center:[0.;3],axis:[0.,0.,1.],angle:0.6,advance:8.},closed:true};
    let s = Surface::Blend(f,Arc::new(blend));
    assert_eq!(s.contact_form(),Form::Numeric);
    let w = Twist {omega:[0.2,0.,0.1],v0:[0.,1.,0.5]};
    assert!(s.ring_contact(0.5,&w).is_none() && s.ruling_contact(0.5,&w).is_none() && s.plane_contact(&w).is_none());
    let mut found = 0;
    for v in [0.1,0.5,0.9] {
        for x in s.contact_along([0.,v],[1.,v],&w,400,1e-14) {
            assert!(s.normal_velocity([x,v],&w).unwrap().abs() <= 1e-8,"at ({x}, {v})");
            found += 1;
        }
    }
    assert!(found >= 6,"{found}");
}

#[test]
fn a_carried_surfaces_contact_is_read_on_the_surface_as_written() {
    // the world's envelope equation at a pose (`envelope::contact`) is the surface's own contact
    // under the twist read on what the motion carries (`Twist::body`)
    let mut rng = Rng::new(17);
    let s = Surface::Torus(frame(&mut rng),9.,2.5);
    for _ in 0..20 {
        let m = Motion::rotation([rng.uniform(-1.,1.),rng.uniform(-1.,1.),1.],rng.uniform(-3.,3.),rng.uniform(-2.,2.)).unwrap()
            .then(Motion::translation([rng.uniform(-5.,5.),1.,2.],[rng.uniform(-3.,3.),0.5,-1.]).unwrap())
            .then(Motion::rotation([1.,0.,rng.uniform(-1.,1.)],rng.uniform(-3.,3.),rng.uniform(-2.,2.)).unwrap());
        let uv = [rng.uniform(0.,TAU),rng.uniform(0.,TAU)];
        let (position,du,dv) = s.d1(uv);
        let world = envelope::contact(SurfacePoint {position,du,dv},m).unwrap();
        let own = s.normal_velocity(uv,&Twist::body(m)).unwrap();
        agree(own,world.normal_velocity,"the body twist's contact");
        // and the world's twist moves the carried point as the pose's rate does
        let spatial = Twist::spatial(m).velocity(m.point(position));
        let rate = m.velocity(position);
        for k in 0..3 { agree(spatial[k],rate[k],"the spatial twist") }
        assert!(dot(world.normal,world.normal) > 0.5);
    }
}
