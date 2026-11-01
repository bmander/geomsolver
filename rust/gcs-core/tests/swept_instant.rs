//! **Contacts read at each instant** (#721 K2, `brep::swept::instant`): a face of revolution's
//! chart over `(t, h, ±)` under a motion no constant twist takes, each ring's two roots of
//! `A cos θ + B sin θ + C = 0` against the instant's twist, and the branch locus where they meet.
//! Held to what the chart does not compute: each root grazing in the world (the world twist, not the
//! body twist it read), a screw's roots fixed on the tool, the roots meeting on the locus, and a
//! ball's contacts on its swept solid's boundary by the kernel's membership.
use gcs_core::brep::build::sphere;
use gcs_core::brep::geom::Surface;
use gcs_core::brep::swept::instant::{branch_locus,grazing,ring_chart};
use gcs_core::brep::swept::member::{Solid,Swept};
use gcs_core::brep::trajectory::{Screw,Trajectory};

/// A turn about `axis` through `o` by `ratio` radians a unit of roll.
fn turn(o: [f64;3],axis: [f64;3],ratio: f64) -> Trajectory {
    Trajectory::Twist(Screw {origin:o,axis:gcs_core::space::unit(axis),ratio,advance:0.})
}

/// A motion no constant twist takes: a turn about the world's `z`, seen from a turn about a line
/// across it (the generating class's relative rotation, its instant's screw moving with it).
fn tumble() -> Trajectory {
    Trajectory::Relative {source:Box::new(turn([0.;3],[0.,0.,1.],1.)),observer:Box::new(turn([0.,0.,-1.],[1.,0.,0.],0.4))}
}

/// The ball of radius `r` about `c`, and its one face's surface.
fn ball(c: [f64;3],r: f64) -> (gcs_core::brep::topo::Brep,Surface) {
    let b = sphere(c,[0.,0.,1.],r).unwrap();
    let s = b.faces[0].surface.clone();
    (b,s)
}

#[test]
fn every_root_of_the_chart_grazes_in_the_world() {
    let (_,face) = ball([2.,0.3,0.],0.5);
    let path = tumble();
    let chart = ring_chart(&face,&path,[0.,1.],[-1.3,1.3],21,15,1e-12).unwrap();
    let mut read = 0;
    for (i,&t) in chart.times.iter().enumerate() {
        for j in 0..chart.heights.len() {
            for &(_,p,n) in chart.roots[i*chart.heights.len()+j].iter().flatten() {
                let g = grazing(&path,t,p,n).unwrap();
                assert!(g.abs() <= 1e-9,"at roll {t}: n·v / |v| = {g}");
                read += 1;
            }
        }
    }
    assert!(read > 400,"{read} roots read");
}

#[test]
fn a_screws_roots_stand_still_on_the_tool() {
    let (_,face) = ball([2.,0.,0.],0.5);
    let path = Trajectory::Twist(Screw {origin:[0.;3],axis:[0.,0.,1.],ratio:1.,advance:2.});
    let chart = ring_chart(&face,&path,[0.,1.],[-1.,1.],7,9,1e-12).unwrap();
    let nh = chart.heights.len();
    for j in 0..nh {
        let first = chart.roots[j];
        for i in 1..chart.times.len() {
            for k in 0..2 {
                match (first[k],chart.roots[i*nh+j][k]) {
                    (Some((u,..)),Some((w,..))) => assert!((u-w).abs() <= 1e-12,"ring {j}: {u} then {w}"),
                    (None,None) => {}
                    other => panic!("ring {j}: {other:?}"),
                }
            }
        }
    }
}

#[test]
fn the_roots_meet_on_the_branch_locus() {
    // a ball under the tumble, which carries it along its own axis: a ring near a pole, its normals
    // near the axis, misses the velocity, and its roots meet at the latitude where it stops
    let (_,face) = ball([2.,0.3,0.],0.5);
    let path = tumble();
    let locus = branch_locus(&face,&path,[0.,1.],[-1.55,1.55],11,200).unwrap();
    assert!(locus.len() >= 11,"the ball's rings branch at every roll: {locus:?}");
    for &(t,h) in &locus {
        let twist = gcs_core::brep::trajectory::Twist::body(path.at(t).unwrap());
        let k = face.ring_contact(h,&twist).unwrap();
        let (amp,c) = (k.a.hypot(k.b),k.c.abs());
        assert!((amp-c).abs() <= 1e-9*(1.+amp),"at ({t}, {h}): amplitude {amp}, constant {c}");
    }
}

#[test]
fn a_balls_contacts_bound_what_it_sweeps() {
    let (tool,face) = ball([2.,0.3,0.],0.5);
    let path = tumble();
    let roll = [0.,1.];
    let solid = Solid::new(&tool,0.001,1.).unwrap();
    let swept = Swept {tool:&solid,path:&path,roll};
    let chart = ring_chart(&face,&path,[0.1,0.9],[-1.2,1.2],9,9,1e-12).unwrap();
    let hair = 20.*solid.sag+1e-4;
    let mut held = 0;
    for root in chart.roots.iter().flatten() {
        let Some((_,p,n)) = *root else { continue };
        let at = |s: f64| std::array::from_fn(|k| p[k]+s*n[k]);
        let (inside,outside) = (swept.member(at(-hair)),swept.member(at(hair)));
        assert_eq!(inside,Ok(true),"{p:?}: a hair inside");
        assert_eq!(outside,Ok(false),"{p:?}: a hair outside");
        held += 1;
    }
    assert!(held > 100,"{held} contacts held");
}
