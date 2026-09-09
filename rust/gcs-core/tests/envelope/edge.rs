use super::*;
use gcs_core::envelope::{EdgePoint,edge_contact};
use std::f64::consts::{PI,FRAC_PI_2,SQRT_2};

fn corner(scale: f64) -> EdgePoint {
    EdgePoint {position:[scale,scale,0.],tangent:[0.,0.,1.],
        normals:[[1.,0.,0.],[0.,1.,0.]],dihedral:-FRAC_PI_2}
}

#[test]
fn convex_edge_generates_the_known_diagonally_swept_box_face() {
    for scale in [1e-6,1.,1e6] { for t in [-0.7,0.,0.9] {
        let motion = Motion::translation([t*scale,-t*scale,0.],[scale,-scale,0.]).unwrap();
        let edge = corner(scale);
        let c = edge_contact(edge,motion,1e-8,scale*1e-10).unwrap().unwrap();
        near(c.normal,[1./SQRT_2,1./SQRT_2,0.],1e-12);
        near(c.position,[scale*(1.+t),scale*(1.-t),0.],scale*1e-12);
        assert!(c.normal_velocity.abs() < scale*1e-12);
        // Every translated source box has x+y <= 2*scale. The returned face
        // lies on that support plane, while an inward point is inside at t.
        assert!((c.position[0]+c.position[1]-2.*scale).abs() < scale*1e-12);
        let inside = motion.inverse().point(std::array::from_fn(|k|
            c.position[k]-scale*0.01*c.normal[k]));
        assert!(inside.iter().all(|x| x.abs() < scale));
        let pose = Motion::rotation([1.,2.,3.],0.7,0.).unwrap()
            .then(Motion::translation([3.*scale,-4.*scale,2.*scale],[0.;3]).unwrap());
        let transformed = edge_contact(edge,motion.then(pose),1e-8,scale*1e-10).unwrap().unwrap();
        near(transformed.position,pose.point(c.position),scale*1e-12);
        near(transformed.normal,pose.vector(c.normal),1e-12);
        let swapped = EdgePoint {normals:[edge.normals[1],edge.normals[0]],
            tangent:edge.tangent.map(|x| -x),..edge};
        near(edge_contact(swapped,motion,1e-8,scale*1e-10).unwrap().unwrap().normal,c.normal,1e-12);
    } }
}

#[test]
fn normal_cones_distinguish_inactive_concave_smooth_and_singular_edges() {
    let edge = corner(1.);
    let motion = |v| Motion::translation([0.;3],v).unwrap();
    for v in [[1.,1.,0.],[-1.,-1.,0.]] {
        assert!(edge_contact(edge,motion(v),1e-8,1e-10).unwrap().is_none());
    }
    let active = motion([1.,-1.,0.]);
    assert!(edge_contact(EdgePoint {dihedral:FRAC_PI_2,..edge},active,1e-8,1e-10).unwrap().is_none());
    assert!(edge_contact(EdgePoint {dihedral:0.,normals:[edge.normals[0];2],..edge},active,1e-8,1e-10)
        .unwrap().is_none());
    for v in [[0.;3],[0.,0.,1.]] {
        assert_eq!(edge_contact(edge,motion(v),1e-8,1e-10).unwrap_err(),Error::Degenerate);
    }
    // A cone endpoint joins the smooth face's contact sheet.
    let c = edge_contact(edge,motion([0.,1.,0.]),1e-8,1e-10).unwrap().unwrap();
    near(c.normal,[1.,0.,0.],1e-12);
    assert_eq!(edge_contact(edge,active,0.,1e-10).unwrap_err(),Error::InvalidOptions);
    assert_eq!(edge_contact(edge,active,1e-8,0.).unwrap_err(),Error::InvalidOptions);
    assert_eq!(edge_contact(EdgePoint {position:[f64::NAN;3],..edge},active,1e-8,1e-10)
        .unwrap_err(),Error::NonFinite);
    assert_eq!(edge_contact(EdgePoint {tangent:[1.,0.,0.],..edge},active,1e-8,1e-10)
        .unwrap_err(),Error::Degenerate);
    // Comparing cosines alone would accept this inconsistent shallow crease
    // and then silently classify it as smooth.
    let inconsistent = EdgePoint {dihedral:-1e-7,
        normals:[[1.,0.,0.],[0.001_f64.cos(),0.001_f64.sin(),0.]],..edge};
    assert_eq!(edge_contact(inconsistent,active,1e-6,1e-10).unwrap_err(),Error::Degenerate);
}

#[test]
fn thin_and_shallow_convex_wedges_keep_the_outward_normal() {
    for angle in [1e-4,0.3,FRAC_PI_2,PI-1e-4] {
        let n = [angle.cos(),angle.sin(),0.];
        let edge = EdgePoint {normals:[[1.,0.,0.],n],dihedral:-angle,..corner(1.)};
        let motion = Motion::translation([0.;3],[-n[1],1.+n[0],0.]).unwrap();
        let c = edge_contact(edge,motion,1e-8,1e-10).unwrap().unwrap();
        near(c.normal,[(angle/2.).cos(),(angle/2.).sin(),0.],1e-8);
    }
    let angle = PI-1e-9;
    let edge = EdgePoint {normals:[[1.,0.,0.],[angle.cos(),angle.sin(),0.]],dihedral:-angle,..corner(1.)};
    assert_eq!(edge_contact(edge,Motion::identity(),1e-8,1e-10).unwrap_err(),Error::Degenerate);
}

#[test]
fn a_normal_cone_candidate_can_still_be_covered_by_another_pose() {
    // Box [-1,1] x [0,2] x [-1,1], rotated about z. Its lower-right edge
    // satisfies the cone condition on one incident face's normal ray.
    let edge = EdgePoint {position:[1.,0.,0.],normals:[[1.,0.,0.],[0.,-1.,0.]],..corner(1.)};
    let motion = |t| Motion::rotation([0.,0.,1.],t,1.).unwrap();
    let candidate = edge_contact(edge,motion(0.),1e-8,1e-10).unwrap().unwrap();
    near(candidate.normal,[1.,0.,0.],1e-12);
    let source = motion(-0.1).inverse().point(candidate.position);
    assert!(source[0].abs() < 1. && source[1] > 0. && source[1] < 2. && source[2].abs() < 1.);
    // Therefore this candidate lies inside the sweep. Normal-cone eligibility
    // alone must never authorize exporting an exposed boundary face.
}
