//! **How deep another instant covers a point of a candidate** (#942, the depth-based trim,
//! `swept::depth`): brep-bench's tumbling tori read by the far cover — a point of a tube's sheet
//! against the instants outside its own contact, a point of the tool at an end pose against the
//! instants after (before) it.

use gcs_core::brep::swept::depth::{Reading,cover,label_at,sheet};
use gcs_core::brep::swept::rings::Ring;
use gcs_core::brep::topo::Brep;
use gcs_core::brep::trajectory::{Screw,Trajectory};

/// brep-bench's tumbling torus turned 0.375 a radian about one line and back a radian about
/// another (`half` the roll each way, in degrees): the half turn's (90) and the full turn's (180),
/// its tool, motion, roll and tolerance.
fn tumbling(half: f64) -> (Brep,Trajectory,[f64;2],f64) {
    let tool = gcs_core::brep::build::torus([3.,0.,2.],[0.,0.,1.],1.,0.16).unwrap();
    let (sl,cl) = 30f64.to_radians().sin_cos();
    let (sa,ca) = 20f64.to_radians().sin_cos();
    let own = Trajectory::Twist(Screw {origin:[2.,0.,0.],axis:[0.,0.,1.],ratio:0.375,advance:0.});
    let frame = Trajectory::Twist(Screw {origin:[0.,0.7071*cl,0.7071*sl],axis:[ca,sa*cl,sa*sl],ratio:1.,
        advance:0.});
    let path = Trajectory::Relative {source:Box::new(own),observer:Box::new(frame)};
    (tool,path,[-half.to_radians(),half.to_radians()],0.16/50_000.)
}

/// The far cover of the half turn's sheet at `(θ, t)`, on the contact nearest the place `σ` round
/// the meridian.
fn far(ring: &Ring,path: &Trajectory,roll: [f64;2],(th,t,sg): (f64,f64,f64),tol: f64) -> Reading {
    let l = label_at(ring,path,th,t,sg).unwrap();
    let (x,bend) = sheet(ring,path,roll,th,t,l).unwrap();
    assert!(bend > 0.,"({th}, {t}): the sheet touching, s'' {bend}");
    cover(ring,path,roll,x,t,tol).unwrap()
}

/// **The far cover agrees with the double curve** (#942): two pairs of the half turn's double
/// curve as `rings::double_curve_from_apex` traces it (its points to seven places, each within a
/// tenth of the tolerance of the curve) — each point covered by its partner's instant, by no more
/// than the tolerance either way: the curve where another instant's cover begins, its partner face
/// and instant named by the cover, not by a meeting of surfaces.
#[test]
fn the_far_cover_of_the_half_turns_sheet_begins_on_its_double_curve() {
    let (tool,path,roll,tol) = tumbling(90.);
    let ring = Ring::read(&tool).unwrap();
    for (p,partner) in [((6.2302900,1.5655403,0.4781),1.4621677),((6.4444197,1.4623041,0.4051),1.5674549)] {
        let Reading::Far(c) = far(&ring,&path,roll,p,tol) else { panic!("{p:?}: no far cover") };
        assert!(c.depth.abs() <= tol,"{p:?}: covered {:e} deep",c.depth);
        assert!((c.at-partner).abs() <= 1e-5,"{p:?}: covered at {} for its partner's {partner}",c.at);
    }
}

/// **The swallowtail's tail is covered less than the tolerance near its point** (#942): between
/// the half turn's double curve and its fold, 6.2e-3 of an angle from the swallowtail point, the
/// tail is held by another instant some 5e-9 deep, within the tolerance (3.2e-6) — the sliver the
/// ring road's trim was refused at, 8e-4 from the point, no feature at the tolerance; 7.2e-2 away
/// it is held 8e-5 deep, a feature a trim must cut.
#[test]
fn the_half_turns_tail_is_covered_less_than_the_tolerance_near_its_swallowtail_point() {
    let (tool,path,roll,tol) = tumbling(90.);
    let ring = Ring::read(&tool).unwrap();
    let Reading::Far(near) = far(&ring,&path,roll,(6.343211,1.486769,0.4441),tol) else { panic!("no far cover near the point") };
    assert!(near.depth < 0. && near.depth > -tol,"near the swallowtail point: covered {:e} deep",near.depth);
    let Reading::Far(away) = far(&ring,&path,roll,(6.276129,1.5362,0.4459),tol) else { panic!("no far cover away from the point") };
    assert!(away.depth < -10.*tol,"away from the swallowtail point: covered {:e} deep",away.depth);
}

/// **Where the full turn's end poses touch, both are buried** (#942): the tool where the roll
/// starts and where it stops touch near (2.256, 2.808, −0.219); there the start pose is held 0.047
/// deep by an instant just before the roll's end, and the end pose by the instants just before it
/// — no trim is needed there. And the start pose's equator near (2.5965, 3.3856, −0.2104), where the
/// ring road's trim left an open edge, is held by the instants after it: no piece of it is kept.
#[test]
fn where_the_full_turns_end_poses_touch_both_are_buried() {
    let (tool,path,roll,tol) = tumbling(180.);
    let ring = Ring::read(&tool).unwrap();
    // the torus's point at the angles `φ` about its axis and `ψ` round its tube, at a pose
    let at = |t: f64,phi: f64,psi: f64| {
        let r = 1.+0.16*psi.cos();
        path.pose(t).unwrap().point([3.+r*phi.cos(),r*phi.sin(),2.+0.16*psi.sin()])
    };
    let start = at(roll[0],2.355154,-1.569185);
    match cover(&ring,&path,roll,start,roll[0],tol).unwrap() {
        Reading::Far(c) => assert!(c.depth < -0.04 && c.at > 3.,"the start pose at the touch: {c:?}"),
        other => panic!("the start pose at the touch: {other:?}"),
    }
    let end = at(roll[1],-2.355278,-1.567330);
    assert!(gcs_core::brep::util::space::distance(start,end) < 1e-6,"the poses touch: {start:?} {end:?}");
    assert!(matches!(cover(&ring,&path,roll,end,roll[1],tol).unwrap(),Reading::Near),"the end pose at the touch");
    let equator = at(roll[0],1.760338,-0.000259);
    assert!(matches!(cover(&ring,&path,roll,equator,roll[0],tol).unwrap(),Reading::Near),"the start pose's equator");
}
