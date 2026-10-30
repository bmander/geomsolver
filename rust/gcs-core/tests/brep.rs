//! The Rust B-rep kernel's primitives (docs/rust-kernel-plan.md, rung 1), held to arithmetic:
//! every prism and revolution a valid closed boundary, and its volume — read off its loops by
//! Green's theorem, nothing meshed — the closed form to rounding.
use gcs_core::brep::build::{prism,revolve,Profile,ProfileEdge};
use gcs_core::brep::geom::{Frame,Rigid,V};
use gcs_core::brep::props::volume;
use std::f64::consts::{PI,TAU};

fn line(a: V,b: V) -> ProfileEdge { ProfileEdge::Line {a,b} }
/// An arc in the plane with normal `n`, turning about it from `a0` to `a1`, angles from `x`.
fn arc(c: V,r: f64,n: V,x: V,span: Option<[f64;2]>) -> ProfileEdge {
    ProfileEdge::Arc {frame:Frame::new(c,n,x),r,span}
}
fn poly(pts: &[V]) -> Vec<ProfileEdge> { (0..pts.len()).map(|i| line(pts[i],pts[(i+1)%pts.len()])).collect() }

fn close(a: f64,b: f64) { assert!((a-b).abs() <= 1e-11*b.abs().max(1.),"{a} against {b} ({:e})",(a-b)/b); }

const XY: V = [0.,0.,1.];
/// The plane of x and z, seen from +y: an angle about it turns x toward −z.
const XZ: V = [0.,1.,0.];

#[test]
fn a_rectangle_swept_is_its_area_times_its_depth() {
    let p = Profile {names:vec![],origin:[0.;3],normal:XY,loops:vec![poly(&[[0.,0.,0.],[3.,0.,0.],[3.,2.,0.],[0.,2.,0.]])]};
    let b = prism(&p,-1.5,2.5).unwrap();
    b.check(1e-9).unwrap();
    assert_eq!((b.faces.len(),b.edges.len(),b.vertices.len()),(6,12,8));
    close(volume(&b),24.);
    // either way round, and written clockwise
    let q = Profile {loops:vec![poly(&[[0.,0.,0.],[0.,2.,0.],[3.,2.,0.],[3.,0.,0.]])],..p.clone()};
    close(volume(&prism(&q,2.5,-1.5).unwrap()),24.);
}

#[test]
fn a_bored_and_rounded_plate_is_its_area_times_its_depth() {
    // a slot: two sides and two half-circle ends, with a round hole and a square one
    let (w,r) = (6.,1.5);
    let mut outer = vec![line([0.,-r,0.],[w,-r,0.]),line([w,r,0.],[0.,r,0.])];
    outer.push(arc([w,0.,0.],r,XY,[1.,0.,0.],Some([-PI/2.,PI/2.])));
    outer.push(arc([0.,0.,0.],r,XY,[1.,0.,0.],Some([PI/2.,3.*PI/2.])));
    let hole = vec![arc([0.,0.,0.],0.5,XY,[1.,0.,0.],None)];
    let square = poly(&[[3.,-0.5,0.],[4.,-0.5,0.],[4.,0.5,0.],[3.,0.5,0.]]);
    let p = Profile {names:vec![],origin:[0.;3],normal:XY,loops:vec![outer,hole,square]};
    let b = prism(&p,0.,2.).unwrap();
    b.check(1e-9).unwrap();
    close(volume(&b),2.*(w*2.*r+PI*r*r-PI*0.25-1.));
}

#[test]
fn a_circle_swept_is_a_cylinder_seamed_once() {
    let p = Profile {names:vec![],origin:[1.,2.,3.],normal:[1.,1.,1.],loops:vec![vec![arc([1.,2.,3.],2.,[1.,1.,1.],[1.,-1.,0.],None)]]};
    let b = prism(&p,0.,5.).unwrap();
    b.check(1e-9).unwrap();
    assert_eq!((b.faces.len(),b.edges.len(),b.vertices.len()),(3,3,2));
    close(volume(&b),PI*4.*5.);
}

#[test]
fn turned_whole_profiles_are_their_solids_of_revolution() {
    let turn = |loops: Vec<Vec<ProfileEdge>>,angle: f64| {
        let b = revolve(&Profile {names:vec![],origin:[0.;3],normal:XZ,loops},[0.;3],[0.,0.,1.],angle).unwrap();
        b.check(1e-9).unwrap();
        b
    };
    let (r,h,r0) = (2.,3.,0.5);
    // a cylinder, from a rectangle against the axis
    let b = turn(vec![poly(&[[0.,0.,0.],[r,0.,0.],[r,0.,h],[0.,0.,h]])],TAU);
    assert_eq!(b.faces.len(),3);
    close(volume(&b),PI*r*r*h);
    // a tube, from one off it
    close(volume(&turn(vec![poly(&[[r0,0.,0.],[r,0.,0.],[r,0.,h],[r0,0.,h]])],TAU)),PI*(r*r-r0*r0)*h);
    // a cone, and a truncated one
    close(volume(&turn(vec![poly(&[[0.,0.,0.],[r,0.,0.],[0.,0.,h]])],TAU)),PI*r*r*h/3.);
    close(volume(&turn(vec![poly(&[[0.,0.,0.],[r,0.,0.],[r0,0.,h],[0.,0.,h]])],TAU)),PI*h*(r*r+r*r0+r0*r0)/3.);
    // an upside-down cone, its apex below
    close(volume(&turn(vec![poly(&[[0.,0.,0.],[r,0.,h],[0.,0.,h]])],TAU)),PI*r*r*h/3.);
    // a ball: a half-circle against the axis
    let ball = turn(vec![vec![arc([0.,0.,1.],r,XZ,[1.,0.,0.],Some([-PI/2.,PI/2.])),line([0.,0.,1.-r],[0.,0.,1.+r])]],TAU);
    assert_eq!(ball.faces.len(),1);
    close(volume(&ball),4./3.*PI*r*r*r);
    // a torus: a circle off the axis
    let torus = turn(vec![vec![arc([3.,0.,1.],1.,XZ,[1.,0.,0.],None)]],TAU);
    assert_eq!((torus.faces.len(),torus.edges.len()),(1,2));
    close(volume(&torus),2.*PI*PI*3.*1.);
    // a turned rounded ring: a rectangle whose outer corners are quarter-rounds
    let (a,b0,c,rr) = (1.,4.,2.,0.5);
    let ring = vec![line([a,0.,0.],[b0-rr,0.,0.]),arc([b0-rr,0.,rr],rr,XZ,[1.,0.,0.],Some([0.,PI/2.])),
        line([b0,0.,rr],[b0,0.,c-rr]),arc([b0-rr,0.,c-rr],rr,XZ,[1.,0.,0.],Some([-PI/2.,0.])),
        line([b0-rr,0.,c],[a,0.,c]),line([a,0.,c],[a,0.,0.])];
    // Pappus: 2π times the moment about the axis — the rectangle's, less each corner cut off,
    // whose centroid sits (10 − 3π)/(12 − 3π) of the radius in from the sharp corner
    let quarter = (1.-PI/4.)*rr*rr;
    let corner = quarter*(b0-rr*(10.-3.*PI)/(12.-3.*PI));
    close(volume(&turn(vec![ring],TAU)),TAU*((b0*b0-a*a)/2.*c-2.*corner));
}

#[test]
fn a_partial_turn_is_its_share_of_the_whole_with_two_caps() {
    let (r,h,r0) = (2.,3.,0.5);
    let tube = vec![poly(&[[r0,0.,0.],[r,0.,0.],[r,0.,h],[r0,0.,h]])];
    let solid = vec![poly(&[[0.,0.,0.],[r,0.,0.],[r,0.,h],[0.,0.,h]])];
    for (loops,whole) in [(tube,PI*(r*r-r0*r0)*h),(solid,PI*r*r*h)] {
        for angle in [PI/2.,-PI/3.,5.] {
            let b = revolve(&Profile {names:vec![],origin:[0.;3],normal:XZ,loops:loops.clone()},[0.;3],[0.,0.,1.],angle).unwrap();
            b.check(1e-9).unwrap();
            close(volume(&b),whole*angle.abs()/TAU);
        }
    }
    // a quarter of a ball and of a torus
    let ball = vec![arc([0.,0.,0.],r,XZ,[1.,0.,0.],Some([-PI/2.,PI/2.])),line([0.,0.,-r],[0.,0.,r])];
    let b = revolve(&Profile {names:vec![],origin:[0.;3],normal:XZ,loops:vec![ball]},[0.;3],[0.,0.,1.],PI/2.).unwrap();
    b.check(1e-9).unwrap();
    close(volume(&b),PI*r*r*r/3.);
    let torus = vec![vec![arc([3.,0.,1.],1.,XZ,[1.,0.,0.],None)]];
    let b = revolve(&Profile {names:vec![],origin:[0.;3],normal:XZ,loops:torus},[0.;3],[0.,0.,1.],PI/2.).unwrap();
    b.check(1e-9).unwrap();
    close(volume(&b),PI*PI*3./2.);
}

#[test]
fn a_moved_solid_is_the_same_solid() {
    let p = Profile {names:vec![],origin:[0.;3],normal:XZ,loops:vec![vec![arc([3.,0.,1.],1.,XZ,[1.,0.,0.],None)]]};
    let b = revolve(&p,[0.;3],[0.,0.,1.],TAU).unwrap();
    let m = Rigid::turn([1.,2.,3.],[1.,-2.,0.5],0.7).then(&Rigid {r:Rigid::identity().r,t:[5.,-1.,2.]});
    let moved = b.moved(&m);
    moved.check(1e-9).unwrap();
    close(volume(&moved),volume(&b));
}

#[test]
fn points_are_placed_in_on_or_out_of_each_primitive() {
    use gcs_core::brep::query::{Located,Place::*};
    let block = prism(&Profile {names:vec![],origin:[0.;3],normal:XY,loops:vec![poly(&[[0.,0.,0.],[3.,0.,0.],[3.,2.,0.],[0.,2.,0.]]),
        vec![arc([1.,1.,0.],0.5,XY,[1.,0.,0.],None)]]},0.,1.).unwrap();
    let torus = revolve(&Profile {names:vec![],origin:[0.;3],normal:XZ,loops:vec![vec![arc([3.,0.,1.],1.,XZ,[1.,0.,0.],None)]]},
        [0.;3],[0.,0.,1.],TAU).unwrap();
    let ball = revolve(&Profile {names:vec![],origin:[0.;3],normal:XZ,loops:vec![vec![arc([0.,0.,0.],2.,XZ,[1.,0.,0.],Some([-PI/2.,PI/2.])),
        line([0.,0.,-2.],[0.,0.,2.])]]},[0.;3],[0.,0.,1.],PI/2.).unwrap();
    let cases: [(&gcs_core::brep::topo::Brep,V,_);14] = [
        (&block,[2.,1.,0.5],In),(&block,[1.,1.,0.5],Out),(&block,[1.5,1.,0.5],On),(&block,[3.,1.,0.5],On),
        (&block,[2.,1.,1.],On),(&block,[4.,1.,0.5],Out),(&block,[2.,1.,1.0001],Out),
        (&torus,[3.,0.,1.],In),(&torus,[0.,0.,1.],Out),(&torus,[0.,4.,1.],On),(&torus,[-3.,0.,2.5],Out),
        (&ball,[0.5,0.5,0.],In),(&ball,[-0.5,0.5,0.],Out),(&ball,[0.,0.,1.],On),
    ];
    for (b,p,want) in cases {
        let at = Located::new(b,1e-9);
        assert_eq!(at.solid_place(p),want,"{p:?}");
    }
}


#[test]
fn surfaces_meet_in_curves_on_both() {
    use gcs_core::brep::geom::{Curve,Surface};
    use gcs_core::brep::ssi::{intersect,Ssi};
    let f = |o: V,z: V,x: V| Frame::new(o,z,x);
    let (x,y,z) = ([1.,0.,0.],[0.,1.,0.],[0.,0.,1.]);
    let tilted = [0.3,-0.2,0.93];
    let cases: Vec<(Surface,Surface,usize)> = vec![
        (Surface::Plane(f([0.,0.,1.],z,x)),Surface::Plane(f([0.,2.,0.],tilted,x)),1),
        (Surface::Plane(f([0.,0.,1.],z,x)),Surface::Cylinder(f([0.,0.,-3.],z,x),2.),1),
        (Surface::Plane(f([0.,0.,1.],tilted,x)),Surface::Cylinder(f([0.,0.,-3.],z,x),2.),1),
        (Surface::Plane(f([0.,1.,0.],y,x)),Surface::Cylinder(f([0.,0.,-3.],z,x),2.),2),
        (Surface::Plane(f([0.,0.,1.],z,x)),Surface::Cone(f([0.,0.,-3.],z,x),2.,0.3),1),
        // a plane through a cone's axis: the lines through its apex (its meridian either side)
        (Surface::Plane(f([0.,0.,4.],y,x)),Surface::Cone(f([0.,0.,-3.],z,x),2.,0.3),2),
        (Surface::Plane(f([0.,0.,1.],z,x)),Surface::Sphere(f([0.,0.,0.],z,x),2.),1),
        (Surface::Plane(f([0.5,0.,1.],tilted,x)),Surface::Sphere(f([0.,0.,0.],z,x),2.),1),
        (Surface::Plane(f([0.,0.,0.5],z,x)),Surface::Torus(f([0.,0.,0.],z,x),3.,1.),2),
        (Surface::Cylinder(f([0.,0.,0.],z,x),2.),Surface::Sphere(f([0.,0.,1.],z,x),3.),2),
        (Surface::Cylinder(f([0.,0.,0.],z,x),2.),Surface::Cone(f([0.,0.,1.],scale3(z,-1.),x),1.,0.4),1),
        (Surface::Cone(f([0.,0.,0.],z,x),1.,0.4),Surface::Torus(f([0.,0.,2.],z,x),2.5,1.),2),
        (Surface::Torus(f([0.,0.,0.],z,x),3.,1.),Surface::Torus(f([0.,0.,1.],z,x),3.5,1.),2),
        (Surface::Sphere(f([0.,0.,0.],z,x),2.),Surface::Sphere(f([1.,1.,1.],z,x),2.5),1),
        (Surface::Cylinder(f([0.,0.,0.],z,x),2.),Surface::Cylinder(f([1.,1.,5.],z,y),1.5),2),
        // one radius, axes crossing (square and slanted, the second's origin off the crossing)
        (Surface::Cylinder(f([0.,0.,0.],z,x),2.),Surface::Cylinder(f([3.,0.,1.],x,y),2.),2),
        (Surface::Cylinder(f([1.,2.,0.],z,x),2.),Surface::Cylinder(f([1.,2.,1.],tilted,y),2.),2),
    ];
    for (a,b,n) in cases {
        let Ssi::Curves(cs) = intersect(&a,&b,1e-9) else { panic!("{a:?} {b:?}") };
        assert_eq!(cs.len(),n,"{a:?} {b:?}: {cs:?}");
        for c in cs {
            let ts: Vec<f64> = if matches!(c,Curve::Line {..}) { (-5..=5).map(|i| i as f64).collect() } else { (0..16).map(|i| i as f64*TAU/16.).collect() };
            for t in ts {
                let p = c.point(t);
                assert!(a.implicit(p).abs() < 1e-9 && b.implicit(p).abs() < 1e-9,"{a:?} {b:?} {c:?} at {t}: {} {}",a.implicit(p),b.implicit(p));
            }
        }
    }
    // perpendicular cylinders of two radii, or of one with axes passing apart, have no closed form
    assert_eq!(intersect(&Surface::Cylinder(f([0.,0.,0.],z,x),2.),&Surface::Cylinder(f([0.,0.5,0.],x,y),2.),1e-9),Ssi::Traced);
    assert_eq!(intersect(&Surface::Cylinder(f([0.,0.,0.],z,x),2.),&Surface::Cylinder(f([0.,0.,0.],x,y),1.),1e-9),Ssi::Traced);
    // the same surface, written from another frame
    assert_eq!(intersect(&Surface::Cylinder(f([0.,0.,0.],z,x),2.),&Surface::Cylinder(f([0.,0.,7.],scale3(z,-1.),y),2.),1e-9),Ssi::Same);
    assert_eq!(intersect(&Surface::Plane(f([1.,2.,3.],z,x)),&Surface::Plane(f([5.,-2.,3.],scale3(z,-1.),y)),1e-9),Ssi::Same);
}

/// **A spline extruded and a surface the same all along it meet in closed form** (#644): a plane
/// along the extrusion and a cylinder about a line along it in lines along it through where the
/// spline crosses them, a cone about such a line in the spline carried onto it — a quintic over
/// the spline's own parameter, refined only where it must be — each on both surfaces; a plane or a
/// cylinder across the extrusion is traced as before.
#[test]
fn an_extruded_spline_meets_surfaces_along_it_in_closed_form() {
    use gcs_core::brep::geom::{Curve,Surface};
    use gcs_core::brep::nurbs::BSpline;
    use gcs_core::brep::ssi::{intersect,Ssi};
    use std::sync::Arc;
    let f = |o: V,z: V,x: V| Frame::new(o,z,x);
    let (x,y,z) = ([1.,0.,0.],[0.,1.,0.],[0.,0.,1.]);
    // a cubic of two spans in z = 0, from (-3, 0.5) to (3, 0.5), bowing up then down
    let spline = BSpline::new(3,vec![0.,0.,0.,0.,0.5,1.,1.,1.,1.],
        vec![[-3.,0.5,0.],[-1.5,1.,0.],[0.,0.6,0.],[1.5,0.,0.],[3.,0.5,0.]]).unwrap();
    let swept = Surface::Extrusion(f([0.,0.,0.],z,x),Arc::new(Curve::BSpline(Arc::new(spline.clone()))));
    let lines = [
        // the radius 2 about the axis, crossed once each side
        (Surface::Cylinder(f([0.,0.,-4.],z,x),2.),2),
        // about a parallel axis, its axis written the other way
        (Surface::Cylinder(f([1.,0.,7.],scale3(z,-1.),y),1.),2),
        // y = 0.6 along the extrusion: the bow crosses it twice
        (Surface::Plane(f([0.,0.6,0.],y,x)),2),
        (Surface::Plane(f([0.,0.,0.],[0.6,0.8,0.],z)),1),
    ];
    for (other,n) in lines {
        for (a,b) in [(&swept,&other),(&other,&swept)] {
            let Ssi::Curves(cs) = intersect(a,b,1e-9) else { panic!("{other:?}: traced") };
            assert_eq!(cs.len(),n,"{other:?}: {cs:?}");
            for c in cs {
                let Curve::Line {d,..} = c else { panic!("{other:?}: {c:?}") };
                assert_eq!(d,z);
                for t in (-5..=5).map(|i| i as f64) {
                    let p = c.point(t);
                    assert!(a.implicit(p).abs() < 1e-9 && b.implicit(p).abs() < 1e-9,"{other:?} at {t}: {} {}",a.implicit(p),b.implicit(p));
                }
            }
        }
    }
    // a cone about the axis either way up, and about a parallel axis: the spline, standing off the
    // axis as a gear's flank does, carried onto it
    let spline = BSpline::new(3,spline.knots.clone(),spline.poles.iter().map(|p| [p[0],p[1]+5.,p[2]]).collect()).unwrap();
    let swept = Surface::Extrusion(f([0.,0.,0.],z,x),Arc::new(Curve::BSpline(Arc::new(spline.clone()))));
    let cones = [Surface::Cone(f([0.,0.,1.],z,x),1.,0.4),Surface::Cone(f([0.,0.,1.],scale3(z,-1.),x),2.5,-0.7),
        Surface::Cone(f([0.5,-0.5,0.],z,y),0.5,0.9)];
    for cone in cones {
        let Ssi::Curves(cs) = intersect(&cone,&swept,1e-9) else { panic!("{cone:?}: traced") };
        let [Curve::BSpline(b)] = &cs[..] else { panic!("{cone:?}: {cs:?}") };
        // (a cubic took 250 to 330, a trace thousands)
        assert!(b.degree == 5 && b.poles.len() < 80,"{cone:?}: {} poles",b.poles.len());
        let [t0,t1] = spline.domain();
        for k in 0..=200 {
            let t = t0+(t1-t0)*k as f64/200.;
            let p = b.point(t);
            assert!(cone.implicit(p).abs() < 1e-9 && swept.implicit(p).abs() < 1e-9,"{cone:?} at {t}: {} {}",cone.implicit(p),swept.implicit(p));
            // over the spline's own parameter: the spline's point carried along the extrusion
            let q = spline.point(t);
            assert!((p[0]-q[0]).abs() < 1e-9 && (p[1]-q[1]).abs() < 1e-9,"{cone:?} at {t}: {p:?} over {q:?}");
        }
    }
    // across the extrusion: traced
    let tilted = [0.3,-0.2,0.93];
    assert_eq!(intersect(&swept,&Surface::Cylinder(f([0.,0.,0.],tilted,x),2.),1e-9),Ssi::Traced);
    assert_eq!(intersect(&swept,&Surface::Plane(f([0.,0.,0.],[0.,0.6,0.8],x)),1e-9),Ssi::Traced);
    // along it but touching the bow at its top (a tool's side in the pocket it cut), or crossing it
    // twice between two of the samples crossings are read at: traced, where a touch is read
    let top = (0..=100_000).map(|k| spline.point(k as f64/1e5)[1]).fold(f64::MIN,f64::max);
    for below in [0.,1e-4] {
        assert_eq!(intersect(&swept,&Surface::Plane(f([0.,top-below,0.],y,x)),1e-9),Ssi::Traced,"{below}");
    }
}

fn scale3(a: V,s: f64) -> V { [a[0]*s,a[1]*s,a[2]*s] }

fn block(lo: V,hi: V) -> gcs_core::brep::topo::Brep {
    let p = Profile {names:vec![],origin:[0.,0.,0.],normal:XY,loops:vec![poly(&[[lo[0],lo[1],0.],[hi[0],lo[1],0.],[hi[0],hi[1],0.],[lo[0],hi[1],0.]])]};
    prism(&p,lo[2],hi[2]).unwrap()
}
fn rod(c: V,r: f64,z: [f64;2]) -> gcs_core::brep::topo::Brep {
    prism(&Profile {names:vec![],origin:[0.,0.,0.],normal:XY,loops:vec![vec![arc([c[0],c[1],0.],r,XY,[1.,0.,0.],None)]]},z[0],z[1]).unwrap()
}
fn ball(c: V,r: f64) -> gcs_core::brep::topo::Brep {
    let p = Profile {names:vec![],origin:c,normal:XZ,loops:vec![vec![arc(c,r,XZ,[1.,0.,0.],Some([-PI/2.,PI/2.])),
        line([c[0],c[1],c[2]-r],[c[0],c[1],c[2]+r])]]};
    revolve(&p,c,[0.,0.,1.],TAU).unwrap()
}

#[test]
fn booleans_of_boxes_crossing_in_general_position() {
    use gcs_core::brep::boolean::{boolean,Op};
    let a = block([0.,0.,0.],[2.,2.,2.]);
    let b = block([1.,0.5,1.5],[3.,2.5,3.5]);
    let shared = 1.*1.5*0.5;
    for (op,want) in [(Op::Union,16.-shared),(Op::Cut,8.-shared),(Op::Common,shared)] {
        let r = boolean(&a,&b,op,1e-9).unwrap_or_else(|e| panic!("{op:?}: {e}"));
        r.check(1e-8).unwrap_or_else(|e| panic!("{op:?}: {e}"));
        close(volume(&r),want);
    }
}

/// **A Boolean's work is where its solids meet** (#589): a face of one clear of the other's box is
/// carried whole, kept or dropped without being read (`boolean::Arranged::clear`), and a run of
/// Booleans on one solid hands each step's knowledge of the faces it carried to the next
/// (`boolean::combined_known`), so a face no operand comes near is boxed and checked once. The
/// solids are what the same Booleans make one at a time, whole and checked.
#[test]
fn a_run_of_cuts_hands_its_far_faces_on() {
    use gcs_core::brep::boolean::{boolean,combined,combined_known,Op};
    // a solid far from the other, every face of each clear of it
    let a = block([0.,0.,0.],[2.,2.,2.]);
    let far = block([10.,10.,10.],[11.,12.,13.]);
    for (op,want,faces) in [(Op::Union,14.,12),(Op::Cut,8.,6),(Op::Common,0.,0)] {
        let r = boolean(&a,&far,op,1e-9).unwrap_or_else(|e| panic!("{op:?}: {e}"));
        assert_eq!(r.faces.len(),faces,"{op:?}");
        if faces > 0 { r.check(1e-8).unwrap_or_else(|e| panic!("{op:?}: {e}")); }
        close(volume(&r),want);
    }
    // a bar notched 24 times across its top, each notch 2 × 10 × 1, each Boolean meeting two of the
    // bar's faces and carrying the rest whole
    let bar = block([0.,0.,0.],[100.,10.,2.]);
    let notch = |k: usize| block([4.*k as f64+1.,-1.,1.],[4.*k as f64+3.,11.,3.]);
    let (mut one,mut run,mut known) = (bar.clone(),bar.clone(),None);
    for k in 0..24 {
        one = combined(&one,&notch(k),Op::Cut,0.).unwrap_or_else(|e| panic!("notch {k}: {e}"));
        let (r,handed) = combined_known(&run,known.as_ref(),&notch(k),Op::Cut,0.)
            .unwrap_or_else(|e| panic!("notch {k}: {e}"));
        (run,known) = (r,Some(handed));
    }
    // each checked whole: what the run left unchecked was checked before
    run.check(1e-8).unwrap();
    close(volume(&one),2000.-24.*20.);
    close(volume(&run),volume(&one));
    assert_eq!((run.faces.len(),run.edges.len()),(one.faces.len(),one.edges.len()));
    // the bar's bottom, ends, front and back, its top in the 25 pieces between the notches, and
    // each notch's floor and walls
    assert_eq!(run.faces.len(),5+25+24*3);
}

#[test]
fn a_pocket_and_a_bore() {
    use gcs_core::brep::boolean::{boolean,Op};
    let a = block([0.,0.,0.],[4.,4.,4.]);
    // a pocket from above
    let r = boolean(&a,&block([1.,1.,2.],[3.,3.,6.]),Op::Cut,1e-9).unwrap();
    r.check(1e-8).unwrap();
    close(volume(&r),64.-8.);
    // a bore straight through, and the rod it would leave
    let bore = rod([2.,2.,0.],1.,[-1.,5.]);
    let r = boolean(&a,&bore,Op::Cut,1e-9).unwrap();
    r.check(1e-8).unwrap();
    close(volume(&r),64.-4.*PI);
    let r = boolean(&a,&bore,Op::Union,1e-9).unwrap();
    r.check(1e-8).unwrap();
    close(volume(&r),64.+2.*PI);
    let r = boolean(&a,&bore,Op::Common,1e-9).unwrap();
    r.check(1e-8).unwrap();
    close(volume(&r),4.*PI);
    // a ball half sunk in the top
    let r = boolean(&a,&ball([2.,2.,4.],1.5),Op::Common,1e-9).unwrap();
    r.check(1e-8).unwrap();
    close(volume(&r),2./3.*PI*1.5f64.powi(3));
}

#[test]
fn booleans_of_faces_on_one_surface() {
    use gcs_core::brep::boolean::{boolean,Op};
    let a = block([0.,0.,0.],[4.,4.,4.]);
    let cases: Vec<(&str,gcs_core::brep::topo::Brep,Op,f64)> = vec![
        // a pocket flush with the top, and one flush with a side as well
        ("flush pocket",block([1.,1.,2.],[3.,3.,4.]),Op::Cut,64.-8.),
        ("corner notch",block([2.,2.,2.],[4.,4.,4.]),Op::Cut,64.-8.),
        // a boss standing on the top, and one flush with a side
        ("boss",block([1.,1.,4.],[3.,3.,6.]),Op::Union,64.+8.),
        ("side-flush boss",block([2.,0.,4.],[4.,2.,6.]),Op::Union,64.+8.),
        // two blocks sharing a face whole
        ("stacked",block([0.,0.,4.],[4.,4.,6.]),Op::Union,64.+32.),
        // a bore flush through both faces, and a blind one flush with the top
        ("flush bore",rod([2.,2.,0.],1.,[0.,4.]),Op::Cut,64.-4.*PI),
        ("blind bore",rod([2.,2.,0.],1.,[1.,4.]),Op::Cut,64.-3.*PI),
        ("flush rod",rod([2.,2.,0.],1.,[0.,4.]),Op::Common,4.*PI),
        ("overlapping",block([2.,1.,0.],[6.,3.,4.]),Op::Union,64.+8.*4.-2.*2.*4.),
    ];
    for (what,b,op,want) in cases {
        let r = boolean(&a,&b,op,1e-9).unwrap_or_else(|e| panic!("{what}: {e}"));
        r.check(1e-8).unwrap_or_else(|e| panic!("{what}: {e}"));
        let v = volume(&r);
        assert!((v-want).abs() <= 1e-10*want,"{what}: {v} against {want}");
    }
}

#[test]
fn a_boss_tangent_to_a_face_inside_it() {
    // a round boss in a block, flush with its top and bottom or not, touching its side along a
    // line through that face's middle (#597): every middle of the side's spans is on the boss
    use gcs_core::brep::boolean::{boolean,Op};
    use gcs_core::brep::mesh::mesh;
    let a = block([-20.,-20.,0.],[20.,20.,10.]);
    for (z,depth) in [([0.,10.],10.),([2.,8.],6.)] {
        let boss = rod([-16.,0.,0.],4.,z);
        let bossed = 16.*PI*depth;
        for (op,want) in [(Op::Union,16000.),(Op::Cut,16000.-bossed),(Op::Common,bossed)] {
            let what = format!("{op:?} of a boss {z:?}");
            let r = boolean(&a,&boss,op,1e-9).unwrap_or_else(|e| panic!("{what}: {e}"));
            r.check(1e-8).unwrap_or_else(|e| panic!("{what}: {e}"));
            let v = volume(&r);
            assert!((v-want).abs() <= 1e-10*want,"{what}: {v} against {want}");
            mesh(&r,1e-2,0.1).unwrap_or_else(|e| panic!("{what}: meshed: {e}"));
        }
    }
}

#[test]
fn a_flush_boss_tangent_to_a_blocks_side_builds_as_the_block() {
    // the issue's document (#597): a screw boss in a block's corner, flush with its wall
    let e = crate::common::text::solved("\
unit mm
use std
use std.shapes
in std.top {
  c := point hint((0, 0))
  fix((0mm, 0mm)) c
  sq := std.shapes.CenteredRectangle(c, w: 40mm, h: 40mm)
  o := point hint((-16, 0))
  c distance(-16mm, along: u) o
  c level(v) o
  rim := radius(4mm) circle(center: o)
}
box := solid(face(sq.loop), from: 0mm, to: 10mm)
boss := solid(face(rim), from: 0mm, to: 10mm)
part := solid(box)
boss union part
");
    let x = e.sketch.exact_solid(crate::common::ent(&e,"part").i()).unwrap_or_else(|m| panic!("`part`: {m}"));
    x.brep.check(1e-8).unwrap();
    let v = volume(&x.brep);
    assert!((v-16000.).abs() <= 1e-9*16000.,"{v}");
    gcs_core::brep::mesh::mesh(&x.brep,1e-2,0.1).unwrap();
}


/// `∫_a^b f` by Simpson's rule on `n` (even) steps.
pub(crate) fn simpson(a: f64,b: f64,n: usize,f: impl Fn(f64) -> f64) -> f64 {
    let h = (b-a)/n as f64;
    (0..=n).map(|i| f(a+h*i as f64)*if i == 0 || i == n { 1. } else if i%2 == 1 { 4. } else { 2. }).sum::<f64>()*h/3.
}

#[test]
fn traced_intersections_cross_bores_and_a_pierced_ball() {
    use gcs_core::brep::boolean::{boolean,Op};
    use gcs_core::brep::build::prism;
    // a horizontal rod along x, radius r, through (y, z) = (2, 2)
    let across = |r: f64| prism(&Profile {names:vec![],origin:[0.;3],normal:[1.,0.,0.],loops:vec![vec![arc([0.,2.,2.],r,[1.,0.,0.],[0.,1.,0.],None)]]},-1.,5.).unwrap();
    let (big,small) = (1.5,1.);
    // the two rods' common volume, perpendicular axes crossing: 8 ∫₀ʳ √(r² − t²) √(R² − t²) dt
    // (substituting t = r sin θ keeps the integrand smooth to its ends)
    let common = 8.*simpson(0.,PI/2.,2000,|th: f64| {
        let t = small*th.sin();
        (small*small-t*t).max(0.).sqrt()*(big*big-t*t).max(0.).sqrt()*small*th.cos()
    });
    let r = boolean(&rod([2.,2.,0.],big,[-1.,5.]),&across(small),Op::Common,1e-9).unwrap();
    r.check(1e-8).unwrap();
    assert!((volume(&r)-common).abs() <= 1e-8*common,"{} against {common}",volume(&r));
    // a block bored down and across
    let a = block([0.,0.,0.],[4.,4.,4.]);
    let down = boolean(&a,&rod([2.,2.,0.],big,[-1.,5.]),Op::Cut,1e-9).unwrap();
    let both = boolean(&down,&across(small),Op::Cut,1e-9).unwrap();
    both.check(1e-8).unwrap();
    let want = 64.-PI*big*big*4.-(PI*small*small*4.-common);
    assert!((volume(&both)-want).abs() <= 1e-8*want,"{} against {want}",volume(&both));
    // a ball of radius 3 pierced by a bore of radius 1 off its centre by 1.5 (clear of its poles)
    let pierced = boolean(&ball([0.,0.,0.],3.),&rod([1.5,0.,0.],1.,[-4.,4.]),Op::Cut,1e-9).unwrap();
    pierced.check(1e-8).unwrap();
    // the part of the bore inside the ball: over the bore's disk, the chord 2√(9 − ρ²) of the ball
    let inside = simpson(0.,1.,400,|s: f64| simpson(0.,TAU,400,|th: f64| {
        let (x,y) = (1.5+s*th.cos(),s*th.sin());
        2.*(9.-x*x-y*y).max(0.).sqrt()*s
    }));
    let want = 4./3.*PI*27.-inside;
    assert!((volume(&pierced)-want).abs() <= 1e-7*want,"{} against {want}",volume(&pierced));
}


/// Two rods of one radius whose axes cross meet in two ellipses, in closed form: in common they are
/// the bicylinder, `16r³ / (3 sin θ)` for axes at θ, and a block's corner cut by both is the
/// corner less their two volumes plus that common part.
#[test]
fn rods_of_one_radius_crossing_meet_in_ellipses() {
    use gcs_core::brep::boolean::{boolean,Op};
    let r = 1.5;
    let upright = rod([0.;3],r,[-4.,4.]);
    for th in [PI/2.,PI/3.,1.1] {
        let d = [th.sin(),0.,th.cos()];
        let slanted = prism(&Profile {names:vec![],origin:[0.;3],normal:d,
            loops:vec![vec![arc([0.;3],r,d,[0.,1.,0.],None)]]},-9.,9.).unwrap();
        let both = boolean(&upright,&slanted,Op::Common,1e-9).unwrap_or_else(|e| panic!("{th}: {e}"));
        both.check(1e-8).unwrap();
        close(volume(&both),16.*r*r*r/(3.*th.sin()));
    }
    // the two rods a square corner's fillet runs round, their axes a radius in from both faces,
    // crossing there: united, they are their sum less the whole bicylinder
    let along_x = prism(&Profile {names:vec![],origin:[0.;3],normal:[1.,0.,0.],
        loops:vec![vec![arc([0.,r,-r],r,[1.,0.,0.],[0.,1.,0.],None)]]},-1.,6.).unwrap();
    let along_y = prism(&Profile {names:vec![],origin:[0.;3],normal:[0.,1.,0.],
        loops:vec![vec![arc([r,0.,-r],r,[0.,1.,0.],[1.,0.,0.],None)]]},-1.,6.).unwrap();
    let u = boolean(&along_x,&along_y,Op::Union,1e-9).unwrap();
    u.check(1e-8).unwrap();
    close(volume(&u),2.*PI*r*r*7.-16.*r*r*r/3.);
}

/// A line meets a cylinder in closed form: a chord's two roots, one touch where it is tangent (the
/// two roots standing off by no more than the tolerance — sampled, that double root lands the
/// square root of the tolerance away), the surface itself along the axis.
#[test]
fn a_line_meets_a_cylinder_in_closed_form() {
    use gcs_core::brep::geom::{Curve,Surface};
    use gcs_core::brep::query::{curve_surface,Meets};
    let cyl = Surface::Cylinder(Frame::new([1.,2.,0.],[0.,0.,1.],[1.,0.,0.]),1.5);
    let slant = scale3([1.,0.,0.3],1./1.09f64.sqrt());
    // tangent: through the foot (1, 3.5, 2) square to the radius there, slanted along the axis
    let touch = Curve::Line {p:[1.,3.5,2.],d:slant};
    let Meets::At(roots) = curve_surface(&touch,[-4.,4.],&cyl,1e-9) else { panic!("along") };
    assert_eq!(roots.len(),1,"{roots:?}");
    assert!(roots[0].0.abs() <= 1e-15 && roots[0].1,"{roots:?}");
    // a chord at 0.9 from the axis: ±1.2 across it
    let chord = Curve::Line {p:[1.,2.9,2.],d:slant};
    let Meets::At(roots) = curve_surface(&chord,[-9.,9.],&cyl,1e-9) else { panic!("along") };
    let want = 1.2*1.09f64.sqrt();
    assert_eq!(roots.len(),2);
    assert!((roots[0].0+want).abs() <= 1e-12 && (roots[1].0-want).abs() <= 1e-12,"{roots:?}");
    // clipped to the stretch asked, missing, and lying along the surface
    let Meets::At(roots) = curve_surface(&chord,[0.,9.],&cyl,1e-9) else { panic!("along") };
    assert_eq!(roots.len(),1);
    assert!(matches!(curve_surface(&Curve::Line {p:[1.,3.6,0.],d:slant},[-9.,9.],&cyl,1e-9),Meets::At(r) if r.is_empty()));
    assert!(matches!(curve_surface(&Curve::Line {p:[2.5,2.,0.],d:[0.,0.,1.]},[-9.,9.],&cyl,1e-9),Meets::Along));
}

/// A traced curve's parameter span: a fit's domain, or a trace's steps.
fn span(c: &gcs_core::brep::geom::Curve) -> [f64;2] {
    use gcs_core::brep::geom::Curve;
    match c {
        Curve::BSpline(b) => b.domain(),
        Curve::Traced(t) => [0.,(t.pts.len()-1) as f64],
        other => panic!("a {} where a trace was expected",other.kind()),
    }
}

#[test]
fn a_trace_leaving_a_sheet_ends_where_its_edge_crosses() {
    use gcs_core::brep::geom::Surface;
    // a curved sheet over [0, 10] × [0, 10], cut by the plane x = 4: the meeting runs from the sheet's
    // edge at v = 0 to its edge at v = 1, each crossing a seed; traced from one, it ends at both
    let f = |u: f64,v: f64| [10.*u,10.*v,(2.*u).sin()+v*v];
    let (net,_) = gcs_core::brep::nurbs::fit_net(&f,3,3,16,16,1e-10).unwrap();
    let sheet = Surface::BSpline(Frame::new([0.;3],[0.,0.,1.],[1.,0.,0.]),std::sync::Arc::new(net.clone()));
    let plane = Surface::Plane(Frame::about([4.,0.,0.],[1.,0.,0.]));
    let tol = 1e-9;
    let ends = [net.point(0.4,0.),net.point(0.4,1.)];
    for seeds in [vec![ends[0]],vec![ends[0],ends[1]],vec![ends[1],f(0.4,0.5)]] {
        let curves = gcs_core::brep::ssi::trace(&plane,&sheet,&seeds,[-1.;3],[11.;3],tol).unwrap();
        assert_eq!(curves.len(),1);
        let c = &curves[0];
        let span = span(c);
        let (a,z) = (c.point(span[0]),c.point(span[1]));
        // its ends the crossings, either way round: a seed exactly, else where the last step,
        // halved, still lands on the sheet (within the march's slack)
        let at = |p: V,q: V| (0..3).map(|k| (p[k]-q[k]).abs()).fold(0.,f64::max);
        let (a,z) = if at(a,ends[0]) <= at(z,ends[0]) { (a,z) } else { (z,a) };
        for (p,q) in [(a,ends[0]),(z,ends[1])] {
            let bar = if seeds.contains(&q) { tol } else { 16.*tol };
            assert!(at(p,q) <= bar,"{seeds:?}: end {p:?} off {q:?} by {:e}",at(p,q));
        }
        // and on both surfaces all the way, its ends too (a fit through points crowded at an end
        // hooks off between them)
        for k in 0..=1000 {
            let p = c.point(span[0]+(span[1]-span[0])*k as f64/1000.);
            assert!(plane.implicit(p).abs() <= 16.*tol && sheet.implicit(p).abs() <= 16.*tol,"{seeds:?}: {k}: {p:?}");
        }
    }
}

/// **A trace keeps a sharp turn at its end** (#684 E): a sheet whose ridge turns at a radius of
/// 0.009 mm, cut by a plane in a V 3 mm long whose apex (of radius 0.003 mm, turning 143°) stands
/// 0.03 mm from where it leaves the sheet's edge, while the march's step `h0`, read off the sheet's
/// size, is 0.2 mm: the steps round the apex crowd that end, and are kept where they carry the
/// turn. Dropped by their distance alone, a chord was left across the apex. Traced from either
/// end the curve is one, through the apex, and on both surfaces.
#[test]
fn a_trace_keeps_a_sharp_turn_at_its_end() {
    use gcs_core::brep::geom::Surface;
    // z = y − y0 + s (√((x − cx)² + ε²) − ε), its x laid out by a sinh so the net's poles crowd
    // at the ridge: x − cx = ε sinh w
    let (s,eps,cx,y0): (f64,f64,f64,f64) = (3.,0.009,0.99,39.96);
    let a = ((1.-cx)/eps).asinh()+(cx/eps).asinh();
    let uc = (cx/eps).asinh()/a;
    let f = |u: f64,v: f64| {
        let (w,y) = (a*(u-uc),30.+10.*v);
        [cx+eps*w.sinh(),y,y-y0+s*eps*(w.cosh()-1.)]
    };
    let (net,_) = gcs_core::brep::nurbs::fit_net(&f,3,3,16,4,1e-10).unwrap();
    let sheet = Surface::BSpline(Frame::new([0.;3],[0.,0.,1.],[1.,0.,0.]),std::sync::Arc::new(net.clone()));
    let plane = Surface::Plane(Frame::about([0.;3],[0.,0.,1.]));
    let tol = 1e-6;
    // where it leaves: the edge x = 1 just past the apex, and the edge x = 0 down the far leg
    let q = |x: f64| s*(((x-cx)*(x-cx)+eps*eps).sqrt()-eps);
    let ends = [[1.,y0-q(1.),0.],[0.,y0-q(0.),0.]];
    let apex = [cx,y0,0.];
    for seeds in [vec![ends[1]],vec![ends[0]],vec![ends[1],ends[0]]] {
        let curves = gcs_core::brep::ssi::trace(&plane,&sheet,&seeds,[-1.,29.,-5.],[2.,41.,5.],tol).unwrap();
        assert_eq!(curves.len(),1,"{seeds:?}");
        let c = &curves[0];
        let span = span(c);
        let near = gcs_core::space::distance(c.point(c.inverse(apex)),apex);
        assert!(near <= 16.*tol,"{seeds:?}: the curve passes {near:e} from the apex");
        for k in 0..=4000 {
            let p = c.point(span[0]+(span[1]-span[0])*k as f64/4000.);
            let off = plane.implicit(p).abs().max(sheet.implicit(p).abs());
            assert!(off <= tol,"{seeds:?}: {k}: {p:?} off by {off:e}");
        }
    }
}

/// The volume a mesh encloses: the signed tetrahedra from the origin to each triangle.
fn mesh_volume(m: &gcs_core::brep::mesh::Mesh) -> f64 {
    m.tris.iter().map(|t| {
        let [a,b,c] = t.map(|i| m.pts[i as usize]);
        (a[0]*(b[1]*c[2]-b[2]*c[1])-a[1]*(b[0]*c[2]-b[2]*c[0])+a[2]*(b[0]*c[1]-b[1]*c[0]))/6.
    }).sum()
}

#[test]
fn meshes_are_closed_and_within_their_sag() {
    use gcs_core::brep::boolean::{boolean,Op};
    use gcs_core::brep::mesh::mesh;
    let across = prism(&Profile {names:vec![],origin:[0.;3],normal:[1.,0.,0.],loops:vec![vec![arc([0.,2.,2.],1.,[1.,0.,0.],[0.,1.,0.],None)]]},-1.,5.).unwrap();
    let a = block([0.,0.,0.],[4.,4.,4.]);
    let solids: Vec<(&str,gcs_core::brep::topo::Brep)> = vec![
        ("block",a.clone()),
        ("plate with four holes",{
            let mut loops = vec![poly(&[[-33.,-21.,0.],[33.,-21.,0.],[33.,21.,0.],[-33.,21.,0.]])];
            for c in [[-24.,-14.],[28.,-14.],[28.,14.],[-24.,14.]] { loops.push(vec![arc([c[0],c[1],0.],3.,XY,[1.,0.,0.],None)]); }
            prism(&Profile {names:vec![],origin:[0.;3],normal:XY,loops},0.,5.).unwrap()
        }),
        ("rod",rod([0.,0.,0.],2.,[0.,3.])),
        ("ball",ball([1.,2.,3.],2.)),
        ("torus",revolve(&Profile {names:vec![],origin:[0.;3],normal:XZ,loops:vec![vec![arc([3.,0.,1.],1.,XZ,[1.,0.,0.],None)]]},[0.;3],[0.,0.,1.],TAU).unwrap()),
        ("quarter ball",revolve(&Profile {names:vec![],origin:[0.;3],normal:XZ,loops:vec![vec![arc([0.,0.,0.],2.,XZ,[1.,0.,0.],Some([-PI/2.,PI/2.])),
            line([0.,0.,-2.],[0.,0.,2.])]]},[0.;3],[0.,0.,1.],PI/2.).unwrap()),
        ("cone",revolve(&Profile {names:vec![],origin:[0.;3],normal:XZ,loops:vec![poly(&[[0.,0.,0.],[2.,0.,0.],[0.,0.,3.]])]},[0.;3],[0.,0.,1.],TAU).unwrap()),
        ("bored block",boolean(&a,&rod([2.,2.,0.],1.,[-1.,5.]),Op::Cut,1e-9).unwrap()),
        ("cross bored",boolean(&boolean(&a,&rod([2.,2.,0.],1.5,[-1.,5.]),Op::Cut,1e-9).unwrap(),&across,Op::Cut,1e-9).unwrap()),
        ("pierced ball",boolean(&ball([0.,0.,0.],3.),&rod([1.5,0.,0.],1.,[-4.,4.]),Op::Cut,1e-9).unwrap()),
    ];
    for (what,s) in solids {
        for bar in [0.01,0.001] {
            let m = mesh(&s,bar,0.2).unwrap_or_else(|e| panic!("{what}: {e}"));
            let stl = gcs_core::mesh::stl_of(&m.triangles(),what);
            gcs_core::mesh::stl_shells(&stl).unwrap_or_else(|e| panic!("{what} at {bar}: {e}"));
            assert!(m.sag <= bar,"{what}: sags {} against {bar}",m.sag);
            assert_eq!(m.turned,0,"{what}: triangles facing against their surfaces");
            let (v,w) = (mesh_volume(&m),volume(&s));
            // a chord's sag loses at most its area times itself: well within the bar over the size
            assert!((v-w).abs() <= 4.*bar*w.abs().powf(2./3.)*6.,"{what} at {bar}: {v} against {w}");
            eprintln!("{what} at {bar}: {} triangles, sag {:.2e}, volume {v:.6} against {w:.6}",m.tris.len(),m.sag);
        }
    }
}


fn dump(b: &gcs_core::brep::topo::Brep) {
    use gcs_core::brep::topo::EdgeCurve;
    for (i,f) in b.faces.iter().enumerate() {
        eprintln!("  face {i} {} rev {}: {:?}",f.surface.kind(),f.reversed,f.loops.iter().map(|l| l.iter()
            .map(|c| format!("{}{}",if c.reversed { "-" } else { "+" },c.edge)).collect::<Vec<_>>()).collect::<Vec<_>>());
    }
    for (i,e) in b.edges.iter().enumerate() {
        let kind = match &e.curve { EdgeCurve::Curve(c) => c.kind(),_ => "pole" };
        eprintln!("  edge {i}: {kind} {:?} from {:?} to {:?}",e.t,b.vertices[e.v[0] as usize].p,b.vertices[e.v[1] as usize].p);
    }
}

/// A B-spline of degree three over `knots`' interior (clamped at 0 and 1) through `poles`.
fn cubic(poles: &[V],interior: &[f64]) -> std::sync::Arc<gcs_core::brep::geom::BSpline> {
    let mut knots = vec![0.;4];
    knots.extend(interior);
    knots.extend([1.;4]);
    std::sync::Arc::new(gcs_core::brep::geom::BSpline::new(3,knots,poles.to_vec()).unwrap())
}
/// `∫ f` over each knot span of `s` by ten-point Gauss–Legendre (exact on a polynomial of degree
/// nineteen or less, which every integrand here is between knots).
fn over_spans(s: &gcs_core::brep::geom::BSpline,[a,b]: [f64;2],f: impl Fn(f64) -> f64) -> f64 {
    const G: [(f64,f64);5] = [(0.1488743389816312,0.2955242247147529),(0.4333953941292472,0.2692667193099963),
        (0.6794095682990244,0.2190863625159820),(0.8650633666889845,0.1494513491505806),(0.9739065285171717,0.0666713443086881)];
    let mut cuts = vec![a];
    cuts.extend(s.breaks([a,b]));
    cuts.push(b);
    cuts.windows(2).map(|w| {
        let (m,h) = ((w[0]+w[1])/2.,(w[1]-w[0])/2.);
        G.iter().map(|&(x,wt)| wt*(f(m-h*x)+f(m+h*x))).sum::<f64>()*h
    }).sum()
}

#[test]
fn splines_swept_and_turned_are_their_closed_forms() {
    use gcs_core::brep::boolean::{boolean,Op};
    use gcs_core::brep::mesh::mesh;
    let spline = |s: &std::sync::Arc<gcs_core::brep::geom::BSpline>| ProfileEdge::Spline(s.clone());
    // a plate: a base, a spline of two spans round to the back, and the back
    let lobe = cubic(&[[10.,0.,0.],[12.,8.,0.],[9.,13.,0.],[6.,14.,0.],[0.,10.,0.]],&[0.4]);
    let plate = Profile {names:vec![],origin:[0.;3],normal:XY,loops:vec![vec![line([0.,0.,0.],[10.,0.,0.]),spline(&lobe),line([0.,10.,0.],[0.,0.,0.])]]};
    // its area by Green's theorem: the lines through the origin enclose nothing
    let area = over_spans(&lobe,[0.,1.],|t| { let (p,d,_) = lobe.d2(t); (p[0]*d[1]-p[1]*d[0])/2. });
    let b = prism(&plate,0.,3.).unwrap();
    b.check(1e-9).unwrap();
    assert_eq!(b.faces.iter().filter(|f| f.surface.kind() == "extrusion").count(),1);
    close(volume(&b),3.*area);
    // sliced level with its caps, where the plane meets the extrusion in its spline, traced
    let r = boolean(&b,&block([-5.,-5.,1.],[20.,20.,5.]),Op::Cut,1e-9).unwrap();
    if let Err(e) = r.check(1e-8) { dump(&r); panic!("{e}") }
    // (a traced edge is integrated to its central difference's ~1e-10 of its speed)
    assert!((volume(&r)-area).abs() <= 1e-8*area,"{} against {area}",volume(&r));
    // two prisms of the plate overlapping in depth share their spline side, one surface
    let (lower,upper) = (prism(&plate,0.,3.).unwrap(),prism(&plate,2.,5.).unwrap());
    for (op,want) in [(Op::Union,5.*area),(Op::Common,area),(Op::Cut,2.*area)] {
        let r = boolean(&lower,&upper,op,1e-9).unwrap_or_else(|e| panic!("{op:?}: {e}"));
        r.check(1e-8).unwrap_or_else(|e| panic!("{op:?}: {e}"));
        close(volume(&r),want);
    }
    // a rod standing on the spline edge: the disk's share of the plate, by Green's theorem round
    // the spline inside the disk and the circle's arc inside the plate
    let (c,rr) = { let p = lobe.point(0.5); ([p[0],p[1]],1.5) };
    let off = |t: f64| { let p = lobe.point(t); (p[0]-c[0]).hypot(p[1]-c[1])-rr };
    let root = |mut lo: f64,mut hi: f64| { for _ in 0..200 { let m = (lo+hi)/2.; if (off(m) < 0.) == (off(lo) < 0.) { lo = m } else { hi = m } } (lo+hi)/2. };
    let (t1,t2) = (root(0.,0.5),root(0.5,1.));
    let along = over_spans(&lobe,[t1,t2],|t| { let (p,d,_) = lobe.d2(t); (p[0]*d[1]-p[1]*d[0])/2. });
    let angle = |t: f64| { let p = lobe.point(t); (p[1]-c[1]).atan2(p[0]-c[0]) };
    // from where the spline leaves the disk round (counter-clockwise, the plate's side) to where it entered
    let (a2,mut a1) = (angle(t2),angle(t1));
    while a1 < a2 { a1 += TAU; }
    let arc = (rr*rr*(a1-a2)+rr*c[0]*(a1.sin()-a2.sin())-rr*c[1]*(a1.cos()-a2.cos()))/2.;
    let r = boolean(&b,&rod([c[0],c[1],0.],rr,[-1.,4.]),Op::Common,1e-9).unwrap();
    r.check(1e-8).unwrap();
    let want = 3.*(along+arc);
    assert!((volume(&r)-want).abs() <= 1e-8*want,"{} against {want}",volume(&r));
    // a vase: a foot, a spline wall and a rim, turned about z
    let wall = cubic(&[[12.,0.,0.],[20.,0.,12.],[2.,0.,24.],[10.,0.,36.]],&[]);
    let vase = Profile {names:vec![],origin:[0.;3],normal:XZ,loops:vec![vec![line([0.,0.,0.],[12.,0.,0.]),spline(&wall),
        line([10.,0.,36.],[0.,0.,36.]),line([0.,0.,36.],[0.,0.,0.])]]};
    let v = revolve(&vase,[0.;3],[0.,0.,1.],TAU).unwrap();
    v.check(1e-9).unwrap();
    // Pappus, as π ∮ ρ² dz: only the wall climbs
    let pappus = |b: f64| PI*over_spans(&wall,[0.,b],|t| { let (p,d,_) = wall.d2(t); p[0]*p[0]*d[2] });
    close(volume(&v),pappus(1.));
    // a quarter turn is a quarter of it
    close(volume(&revolve(&vase,[0.;3],[0.,0.,1.],PI/2.).unwrap()),pappus(1.)/4.);
    // cut level at half height, where the wall's parameter is a half: a plane square to the axis
    // meets it in a circle, found where the meridian crosses the plane's level
    let r = boolean(&v,&block([-30.,-30.,18.],[30.,30.,40.]),Op::Cut,1e-9).unwrap();
    r.check(1e-8).unwrap();
    assert!(r.edges.iter().any(|e| matches!(&e.curve,gcs_core::brep::topo::EdgeCurve::Curve(c) if c.kind() == "circle")));
    close(volume(&r),pappus(0.5));
    // halved by a plane through its axis, which meets it in its own meridians
    let r = boolean(&v,&block([0.,-30.,-5.],[30.,30.,40.]),Op::Common,1e-9).unwrap();
    r.check(1e-8).unwrap();
    assert!((volume(&r)-pappus(1.)/2.).abs() <= 1e-8*pappus(1.),"{} against {}",volume(&r),pappus(1.)/2.);
    // a prism and a copy of it moved along its depth share their spline side: one surface,
    // though the copy's curve stands elsewhere
    let raised = b.moved(&Rigid {r:Rigid::identity().r,t:[0.,0.,2.]});
    let r = boolean(&b,&raised,Op::Union,1e-9).unwrap();
    r.check(1e-8).unwrap();
    close(volume(&r),5.*area);
    // a blend whose edges run opposite ways at the same pace has no u-tangent halfway along it:
    // its side there is still a number and a direction, never a panic
    let (p,q) = ([0.,0.,0.],[4.,0.,0.]);
    let flat = gcs_core::brep::geom::Blend {a:gcs_core::brep::geom::Curve::Line {p,d:[1.,0.,0.]},ta:[0.,4.],
        b:gcs_core::brep::geom::Curve::Line {p,d:[1.,0.,0.]},tb:[4.,0.],
        carry:gcs_core::brep::geom::Carry::Line {delta:[0.,0.,2.]},closed:false};
    let surface = gcs_core::brep::geom::Surface::Blend(Frame::about([0.;3],[0.,0.,1.]),std::sync::Arc::new(flat));
    for probe in [[2.,0.5,1.],[1.,0.,1.],q] {
        assert!(surface.implicit(probe).is_finite() && surface.gradient(probe).iter().all(|x| x.is_finite()));
    }
    // meshed closed, within the bar, enclosing the same volume to the bar's order
    for (what,s) in [("plate",&b),("vase",&v)] {
        let m = mesh(s,0.01,0.2).unwrap_or_else(|e| panic!("{what}: {e}"));
        assert!(m.sag <= 0.01,"{what}: sags {}",m.sag);
        assert_eq!(m.turned,0,"{what}: triangles facing against their surfaces");
        gcs_core::mesh::stl_shells(&gcs_core::mesh::stl_of(&m.triangles(),what)).unwrap_or_else(|e| panic!("{what}: {e}"));
        assert!((mesh_volume(&m)-volume(s)).abs() <= 2e-3*volume(s),"{what}: {} against {}",mesh_volume(&m),volume(s));
    }
}

/// A box handed over as a native kernel's JSON (`brep::json`, phase 1): its top a cubic B-spline
/// face on a domain away from 0, one of its pcurves a kernel's gap off its neighbours (closed in the
/// parameters by `props`, or the flux would be off by the gap times `G` there), its bottom written
/// reversed and its uses out of walking order — read, checked, and its volume `a·b·c` wherever it is.
#[test]
fn a_box_read_from_json_is_its_closed_form() {
    let (a,b,c,o) = (3.,2.,1.5,[1000.,-2000.,500.]);
    let at = |x: f64,y: f64,z: f64| [o[0]+x,o[1]+y,o[2]+z];
    let corners: Vec<V> = (0..8).map(|k| at(if k&1 == 0 { 0. } else { a },if k&2 == 0 { 0. } else { b },if k&4 == 0 { 0. } else { c })).collect();
    let sub = |p: V,q: V| [p[0]-q[0],p[1]-q[1],p[2]-q[2]];
    let dot = |p: V,q: V| p[0]*q[0]+p[1]*q[1]+p[2]*q[2];
    let edges: Vec<[usize;2]> = (0..8).flat_map(|i| [1,2,4].into_iter().filter(move |&m| i&m == 0).map(move |m| [i,i|m])).collect();
    let v3 = |p: V| format!("[{},{},{}]",p[0],p[1],p[2]);
    let mut json = format!("{{\"vertices\":[{}],\"edges\":[",corners.iter().map(|&p| format!("{{\"p\":{},\"tol\":0}}",v3(p))).collect::<Vec<_>>().join(","));
    json += &edges.iter().map(|&[i,j]| {
        let d = sub(corners[j],corners[i]);
        let len = dot(d,d).sqrt();
        format!("{{\"v\":[{i},{j}],\"t\":[0,{len}],\"tol\":0,\"curve\":{{\"kind\":\"line\",\"p\":{},\"d\":{}}}}}",v3(corners[i]),v3([d[0]/len,d[1]/len,d[2]/len]))
    }).collect::<Vec<_>>().join(",");
    json += "],\"faces\":[";
    // each face: its corners, its parameters (an affine map of a point), its surface, and whether
    // it is written reversed; the loop runs counter-clockwise in the parameters as written
    let plane = |q: V,x: V,y: V,z: V| format!("{{\"kind\":\"plane\",\"frame\":{{\"o\":{},\"x\":{},\"y\":{},\"z\":{}}}}}",v3(q),v3(x),v3(y),v3(z));
    // the top: x in u over [2, 5] with a knot at 3, from 300 times the box's width before it (so the
    // face is a strip at the end of its surface's chart, far from where `G` starts), y in v over
    // [−1, 1] with one at ½, its poles at the Greville abscissae so the cubic net is exactly the
    // plane, linearly parameterised
    let (uk,vk) = ([2.,2.,2.,2.,3.,5.,5.,5.,5.],[-1.,-1.,-1.,-1.,0.5,1.,1.,1.,1.]);
    let greville = |k: &[f64],i: usize| (k[i+1]+k[i+2]+k[i+3])/3.;
    let poles = (0..5).map(|i| format!("[{}]",(0..5).map(|j| v3(at(-300.*a+301.*a*(greville(&uk,i)-2.)/3.,b*(greville(&vk,j)+1.)/2.,c))).collect::<Vec<_>>().join(","))).collect::<Vec<_>>().join(",");
    let top = format!("{{\"kind\":\"bspline\",\"du\":3,\"dv\":3,\"uknots\":{uk:?},\"vknots\":{vk:?},\"poles\":[{poles}]}}");
    let top_uv = move |p: V| [2.+3.*(p[0]-o[0]+300.*a)/(301.*a),-1.+2.*(p[1]-o[1])/b];
    type Uv = Box<dyn Fn(V) -> [f64;2]>;
    let frame_uv = |r: V,x: V,y: V| -> Uv { Box::new(move |p: V| { let q = [p[0]-r[0],p[1]-r[1],p[2]-r[2]]; [q[0]*x[0]+q[1]*x[1]+q[2]*x[2],q[0]*y[0]+q[1]*y[1]+q[2]*y[2]] }) };
    let faces: Vec<(Vec<usize>,Uv,String,bool)> = vec![
        // the bottom, its frame facing in and the face written reversed
        (vec![0,1,2,3],frame_uv(o,[1.,0.,0.],[0.,1.,0.]),plane(o,[1.,0.,0.],[0.,1.,0.],[0.,0.,1.]),true),
        (vec![4,5,6,7],Box::new(top_uv),top,false),
        (vec![0,1,4,5],frame_uv(o,[1.,0.,0.],[0.,0.,1.]),plane(o,[1.,0.,0.],[0.,0.,1.],[0.,-1.,0.]),false),
        (vec![2,3,6,7],frame_uv(at(0.,b,0.),[0.,0.,1.],[1.,0.,0.]),plane(at(0.,b,0.),[0.,0.,1.],[1.,0.,0.],[0.,1.,0.]),false),
        (vec![0,2,4,6],frame_uv(o,[0.,0.,1.],[0.,1.,0.]),plane(o,[0.,0.,1.],[0.,1.,0.],[-1.,0.,0.]),false),
        (vec![1,3,5,7],frame_uv(at(a,0.,0.),[0.,1.,0.],[0.,0.,1.]),plane(at(a,0.,0.),[0.,1.,0.],[0.,0.,1.],[1.,0.,0.]),false),
    ];
    json += &faces.iter().enumerate().map(|(fi,(ks,uv,surface,reversed))| {
        // counter-clockwise about the centre in the parameters
        let mid = ks.iter().fold([0.,0.],|m,&k| { let p = uv(corners[k]); [m[0]+p[0]/4.,m[1]+p[1]/4.] });
        let mut ring = ks.clone();
        ring.sort_by(|&i,&j| { let (p,q) = (uv(corners[i]),uv(corners[j])); (p[1]-mid[1]).atan2(p[0]-mid[0]).total_cmp(&(q[1]-mid[1]).atan2(q[0]-mid[0])) });
        let mut uses: Vec<String> = (0..4).map(|n| {
            let (p,q) = (ring[n],ring[(n+1)%4]);
            let e = edges.iter().position(|&[i,j]| (i,j) == (p,q) || (i,j) == (q,p)).unwrap();
            let [i,j] = edges[e];
            let (mut s,t) = (uv(corners[i]),uv(corners[j]));
            // a kernel's gap: the top's first pcurve 1e-7 off its neighbours in v
            if fi == 1 && n == 0 { s[1] += 1e-7; }
            let len = dot(sub(corners[j],corners[i]),sub(corners[j],corners[i])).sqrt();
            format!("{{\"edge\":{e},\"reversed\":{},\"pcurve\":{{\"kind\":\"line\",\"p\":[{},{},0],\"d\":[{},{},0]}}}}",(i,j) == (q,p),s[0],s[1],(t[0]-s[0])/len,(t[1]-s[1])/len)
        }).collect();
        uses.swap(1,3);
        format!("{{\"reversed\":{reversed},\"surface\":{surface},\"loops\":[{{\"outer\":true,\"uses\":[{}]}}]}}",uses.join(","))
    }).collect::<Vec<_>>().join(",");
    json += "]}";
    let brep = gcs_core::brep::json::read(&json).unwrap();
    brep.check(1e-9).unwrap();
    assert!(brep.edges.iter().all(|e| e.tol < 2e-7),"{:?}",brep.edges.iter().map(|e| e.tol).collect::<Vec<_>>());
    // within what the gap itself moves the boundary (its length times it, times the distance from
    // the origin), where unclosed it would be off by the gap times the strip's distance along the chart
    let near = |v: f64| assert!((v-a*b*c).abs() < 1e-4,"{v} against {}",a*b*c);
    near(volume(&brep));
    // and wherever it stands
    near(volume(&brep.moved(&Rigid {t:[-3000.,700.,40.],..Rigid::identity()})));
}

/// A net cut to a box is the same surface there: the same point at the same parameters, for a box
/// that cuts across spans and one that starts at the domain's own start.
#[test]
fn a_net_segmented_is_the_same_surface() {
    use gcs_core::brep::nurbs::Net;
    let (uknots,vknots) = (vec![0.,0.,0.,0.,0.3,0.45,1.,1.,1.,1.],vec![-1.,-1.,-1.,0.2,0.5,2.,2.,2.]);
    let poles: Vec<Vec<V>> = (0..6).map(|i| (0..5).map(|j| {
        let (x,y) = (i as f64,j as f64);
        [x+0.3*(x*y).sin(),y-0.2*x*x,0.1*x*y+(x+2.*y).cos()]
    }).collect()).collect();
    let net = Net {du:3,dv:2,uknots,vknots,poles,weights:None};
    for (bu,bv) in [([0.1,0.7],[-0.5,1.2]),([0.,0.4],[0.5,2.]),([0.3,0.45],[-1.,0.2])] {
        let cut = net.segment(bu,bv);
        assert_eq!(cut.domain(),[bu,bv]);
        for i in 0..=6 { for j in 0..=6 {
            let (u,v) = (bu[0]+(bu[1]-bu[0])*i as f64/6.,bv[0]+(bv[1]-bv[0])*j as f64/6.);
            let (a,b) = (net.d1(u,v),cut.d1(u,v));
            for (p,q) in [(a.0,b.0),(a.1,b.1),(a.2,b.2)] {
                assert!((0..3).all(|c| (p[c]-q[c]).abs() <= 1e-11),"{bu:?} {bv:?} at ({u}, {v}): {p:?} against {q:?}");
            }
        } }
        assert!(cut.poles.len() <= net.poles.len()+3 && cut.poles[0].len() <= net.poles[0].len()+2);
    }
}

/// A net carried on past its domain (`Net::extended`) is its own polynomials continued: a net cut
/// short within its end span and carried back out to where it was cut is the uncut net there, at
/// either end and along either parameter — over the cut's domain the same surface as the cut.
#[test]
fn a_net_extended_is_its_own_polynomials_carried_on() {
    use gcs_core::brep::nurbs::Net;
    let (uknots,vknots) = (vec![0.,0.,0.,0.,0.3,0.45,1.,1.,1.,1.],vec![-1.,-1.,-1.,0.2,0.5,2.,2.,2.]);
    let poles: Vec<Vec<V>> = (0..6).map(|i| (0..5).map(|j| {
        let (x,y) = (i as f64,j as f64);
        [x+0.3*(x*y).sin(),y-0.2*x*x,0.1*x*y+(x+2.*y).cos()]
    }).collect()).collect();
    let net = Net {du:3,dv:2,uknots,vknots,poles,weights:None};
    let agree = |a: &Net,b: &Net,[u0,u1]: [f64;2],[v0,v1]: [f64;2],what: &str| {
        for i in 0..=8 { for j in 0..=8 {
            let (u,v) = (u0+(u1-u0)*i as f64/8.,v0+(v1-v0)*j as f64/8.);
            let (p,q) = (a.d1(u,v),b.d1(u,v));
            for (x,y) in [(p.0,q.0),(p.1,q.1),(p.2,q.2)] {
                assert!((0..3).all(|c| (x[c]-y[c]).abs() <= 1e-9*(1.+x[c].abs())),"{what} at ({u}, {v}): {x:?} against {y:?}");
            }
        } }
    };
    // (the cut's box, the parameter, the end, how far: the cut ends within the uncut net's end span)
    for (bu,bv,k,high,by) in [([0.,0.8],[-1.,2.],0,true,0.2),([0.1,1.],[-1.,2.],0,false,0.1),
        ([0.,1.],[-1.,1.5],1,true,0.5),([0.,1.],[-0.5,2.],1,false,0.5)] {
        let cut = net.segment(bu,bv);
        let wider = cut.extended(k,high,by).unwrap();
        wider.check().unwrap();
        let what = format!("{bu:?} × {bv:?} carried {by} along {k} past its {}",if high { "end" } else { "start" });
        let [[w0,w1],[x0,x1]] = wider.domain();
        let grown = if k == 0 { ([w0,w1],[x0,x1]) == (if high { [bu[0],bu[1]+by] } else { [bu[0]-by,bu[1]] },bv) }
            else { ([w0,w1],[x0,x1]) == (bu,if high { [bv[0],bv[1]+by] } else { [bv[0]-by,bv[1]] }) };
        assert!(grown,"{what}: domain {:?}",wider.domain());
        agree(&cut,&wider,bu,bv,&what);
        let beyond = match (k,high) { (0,true) => [bu[1],bu[1]+by],(0,false) => [bu[0]-by,bu[0]],(_,true) => [bv[1],bv[1]+by],_ => [bv[0]-by,bv[0]] };
        if k == 0 { agree(&net,&wider,beyond,bv,&what) } else { agree(&net,&wider,bu,beyond,&what) }
    }
    // a rational net is not carried on
    assert!(Net {weights:Some(vec![vec![1.;5];6]),..net}.extended(0,true,0.1).is_none());
}

/// A sector turned into its copies by identity (`brep::pattern`, phase 2): a torus and a triangle's
/// revolution (three cones) revolved through a pitch and patterned are the whole revolutions — one
/// ring a face of the sector's, each closed on a seam, the volume Pappus's, the mesh made of the
/// sector's turned closed.
#[test]
fn a_sector_patterned_is_its_whole_revolution() {
    use gcs_core::brep::pattern::pattern;
    let torus = vec![vec![arc([3.,0.,1.],1.,XZ,[1.,0.,0.],None)]];
    let triangle = poly(&[[2.,0.,0.],[4.,0.,1.],[2.5,0.,3.]]);
    let pappus = 2.75*TAU*8.5/3.;
    for (loops,whole,rings) in [(torus,TAU*PI*3.,1),(vec![triangle],pappus,3)] {
        for n in [3,6,7] {
            let sector = revolve(&Profile {names:vec![],origin:[0.;3],normal:XZ,loops:loops.clone()},[0.;3],[0.,0.,1.],TAU/n as f64).unwrap();
            let built = pattern(&sector,[0.;3],[0.,0.,1.],n,1e-9).unwrap_or_else(|e| panic!("{n} copies: {e}"));
            let b = &built.solid;
            b.check(1e-9).unwrap_or_else(|e| panic!("{n} copies: {e}"));
            assert_eq!(b.faces.len(),rings,"{n} copies: a face a ring");
            close(volume(b),whole);
            close(volume(b),n as f64*volume(&sector));
            let m = built.mesh(0.01,0.2).unwrap();
            assert!(m.sag <= 0.01 && m.turned == 0,"{n} copies: sag {} with {} turned",m.sag,m.turned);
            gcs_core::mesh::stl_shells(&gcs_core::mesh::stl_of(&m.triangles(),"ring")).unwrap_or_else(|e| panic!("{n} copies: {e}"));
        }
    }
}

#[test]
fn interior_samples_are_inside_and_no_deeper_than_they_are() {
    use gcs_core::brep::query::interior;
    // a block with a ball cut from it: every sample is in the block and out of the ball, and its
    // depth is no more than its distance from either
    let (lo,hi) = ([0.,0.,0.],[10.,6.,4.]);
    let (c,r) = ([5.,3.,2.],1.5);
    let solid = gcs_core::brep::boolean::boolean(&block(lo,hi),&ball(c,r),gcs_core::brep::boolean::Op::Cut,1e-9).unwrap();
    let samples = interior(&solid,8).unwrap();
    assert_eq!(samples.len(),8);
    for (p,d) in &samples {
        let to_box = (0..3).map(|k| (p[k]-lo[k]).min(hi[k]-p[k])).fold(f64::INFINITY,f64::min);
        let to_ball = ((p[0]-c[0]).powi(2)+(p[1]-c[1]).powi(2)+(p[2]-c[2]).powi(2)).sqrt()-r;
        assert!(to_box > 0. && to_ball > 0.,"{p:?} outside");
        assert!(*d <= to_box.min(to_ball)+1e-12 && *d > 0.,"{p:?} said {d} deep, {to_box} and {to_ball} from the walls");
    }
    // the deepest is nearly as deep as the deepest point of the solid: a corner region 2 from the walls
    assert!(samples[0].1 > 1.2,"{samples:?}");
}

#[test]
fn a_grid_interpolated_passes_through_it_and_makes_a_sheet() {
    use gcs_core::brep::nurbs::{interpolate_net,Parametrization};
    // a saddle sampled unevenly: rows bunched toward one end
    let (rows,cols) = (9,7);
    let pts: Vec<V> = (0..rows).flat_map(|i| (0..cols).map(move |j| {
        let (x,y) = ((i as f64/8.).powf(1.5)*10.,j as f64*1.5);
        [x,y,0.05*(x*x-y*y)]
    })).collect();
    for kind in [Parametrization::Even,Parametrization::ChordLength,Parametrization::Centripetal] {
        let net = interpolate_net(&pts,rows,cols,kind).unwrap();
        assert_eq!((net.du,net.dv),(3,3));
        // through every grid point at the parameters it was given: found again by the surface's inverse
        let s = gcs_core::brep::geom::Surface::BSpline(gcs_core::brep::geom::Frame::new([0.;3],[0.,0.,1.],[1.,0.,0.]),std::sync::Arc::new(net.clone()));
        for &p in &pts {
            let q = s.point(s.inverse(p));
            assert!(gcs_core::space::distance(p,q) < 1e-9,"{kind:?}: {p:?} against {q:?}");
        }
        // the sheet's boundary is the grid's: its corners the grid's corners
        let b = gcs_core::brep::build::sheet(net).unwrap();
        assert_eq!(b.faces.len(),1);
        for (k,&c) in [0,(rows-1)*cols,rows*cols-1,cols-1].iter().enumerate() {
            assert!(gcs_core::space::distance(b.vertices[k].p,pts[c]) < 1e-12);
        }
        // each use's pcurve and its edge agree along it
        let f = &b.faces[0];
        for c in &f.loops[0] {
            let e = &b.edges[c.edge as usize];
            for t in [0.,0.3,0.7,1.] {
                let x = e.t[0]+(e.t[1]-e.t[0])*t;
                let uv = c.pcurve.at(x,e,&f.surface,&b.vertices);
                assert!(gcs_core::space::distance(f.surface.point(uv),e.point(x,&b.vertices)) < 1e-12);
            }
        }
    }
}

/// A grid creased along a row (`nurbs::interpolate_creased`, #684 O): a floor 6 wide (z = 0) bent
/// up a right angle into a wall (x = 6), rows across the bend and columns along y, creased at the
/// bend's row. The net passes through every point at its `net_parameters`, its knot at the crease
/// stands three times over, and each side is its own stretch's fit: the floor lies in its plane and
/// the wall in its own to rounding, where one smooth fit through the bend rings out of both. Cut at
/// the crease (`build::grid_sheet`), the two faces share its row as an edge.
#[test]
fn a_grid_creased_along_a_row_is_its_stretches_fits_joined_there() {
    use gcs_core::brep::nurbs::{interpolate_creased,interpolate_net,net_parameters,Parametrization};
    let (rows,cols,crease) = (11,5,6);
    let pts: Vec<V> = (0..rows).flat_map(|i| (0..cols).map(move |j| {
        let y = j as f64*2.;
        if i <= crease { [i as f64,y,0.] } else { [6.,y,(i-crease) as f64] }
    })).collect();
    let kind = Parametrization::ChordLength;
    let net = interpolate_creased(&pts,rows,cols,kind,&[crease]).unwrap();
    let (tu,tv) = net_parameters(&pts,rows,cols,kind).unwrap();
    assert_eq!(net.uknots.iter().filter(|&&k| k == tu[crease]).count(),3);
    for i in 0..rows { for j in 0..cols {
        assert!(gcs_core::space::distance(net.point(tu[i],tv[j]),pts[i*cols+j]) < 1e-12,"row {i}, column {j}");
    } }
    // each side in its own plane, read between the rows and the columns
    let off = |net: &gcs_core::brep::nurbs::Net| (0..=200).flat_map(|a| (0..=20).map(move |b| (a,b))).map(|(a,b)| {
        let (u,v) = (a as f64/200.,b as f64/20.);
        let p = net.point(u,v);
        if u <= tu[crease] { p[2].abs() } else { (p[0]-6.).abs() }
    }).fold(0_f64,f64::max);
    assert!(off(&net) < 1e-12,"{}",off(&net));
    assert!(off(&interpolate_net(&pts,rows,cols,kind).unwrap()) > 1e-2);
    let sheet = gcs_core::brep::build::grid_sheet(&net,&[0.,tu[crease],1.],&[0.,1.],&["floor".into(),"wall".into()],false).unwrap();
    assert_eq!((sheet.faces.len(),sheet.edges.len(),sheet.vertices.len()),(2,7,6));
}

/// A net closed round an axis is cut into a ring of faces (`build::grid_sheet`, #684 N): a cylinder of
/// radius 3 sampled once round, its last column its first, cut in two along it and in four round
/// it. Neighbours share their edges and corners and the last round closes on the first's, so only
/// the rims' edges are used once; each use's pcurve lies on its edge; and a block split by it
/// leaves the cylinder's core, π r² h, and the rest.
#[test]
fn a_net_closed_round_an_axis_is_a_ring_of_faces() {
    use gcs_core::brep::nurbs::{interpolate_net,Parametrization};
    let (rows,cols) = (7,37);
    let pts: Vec<V> = (0..rows).flat_map(|i| (0..cols).map(move |j| {
        let a = TAU*(j%(cols-1)) as f64/(cols-1) as f64;
        [3.*a.cos(),3.*a.sin(),-5.+10.*i as f64/(rows-1) as f64]
    })).collect();
    let net = interpolate_net(&pts,rows,cols,Parametrization::ChordLength).unwrap();
    let [[a,z],[c,d]] = net.domain();
    let vs: Vec<f64> = (0..=4).map(|k| c+(d-c)*k as f64/4.).collect();
    let sheet = gcs_core::brep::build::grid_sheet(&net,&[a,(a+z)/2.,z],&vs,&["low".into(),"high".into()],true).unwrap();
    assert_eq!((sheet.faces.len(),sheet.edges.len(),sheet.vertices.len()),(8,20,12));
    assert_eq!(sheet.faces.iter().filter(|f| f.name == "high").count(),4);
    let mut uses = vec![0;sheet.edges.len()];
    for f in &sheet.faces {
        for c in &f.loops[0] {
            uses[c.edge as usize] += 1;
            let e = &sheet.edges[c.edge as usize];
            for t in [0.,0.3,0.7,1.] {
                let x = e.t[0]+(e.t[1]-e.t[0])*t;
                let uv = c.pcurve.at(x,e,&f.surface,&sheet.vertices);
                assert!(gcs_core::space::distance(f.surface.point(uv),e.point(x,&sheet.vertices)) < 1e-9);
            }
        }
    }
    assert_eq!((uses.iter().filter(|&&n| n == 1).count(),uses.iter().filter(|&&n| n == 2).count()),(8,12));
    let block = prism(&gcs_core::brep::build::square([0.;3],[0.,0.,1.],4.),-2.,2.).unwrap();
    let mut cells: Vec<f64> = gcs_core::brep::boolean::split(&block,&sheet,1e-6).unwrap().iter().map(volume).collect();
    cells.sort_by(f64::total_cmp);
    let core = PI*9.*4.;
    assert_eq!(cells.len(),2,"{cells:?}");
    assert!((cells[0]-core).abs() < 1e-5*core && (cells[1]-(256.-core)).abs() < 1e-5*core,"{cells:?} against {core}");
}

/// An interpolation of many points (solved in its band, past the dense solve's size) passes through
/// every one, at its own parameter.
#[test]
fn a_long_interpolation_passes_through_its_points() {
    let pts: Vec<[f64;3]> = (0..300).map(|k| { let a = k as f64*0.05; [10.*a.cos(),10.*a.sin(),0.3*a] }).collect();
    let mut t = vec![0.];
    for w in pts.windows(2) { t.push(t.last().unwrap()+gcs_core::space::distance(w[0],w[1])); }
    let s = gcs_core::brep::nurbs::interpolate(&pts,&t,3).unwrap();
    let worst = pts.iter().zip(&t).map(|(p,&u)| gcs_core::space::distance(s.point(u),*p)).fold(0.,f64::max);
    assert!(worst < 1e-9,"{worst}");
}

/// A meeting that runs off a sheet's patch ends at the patch's edge: past it the sheet's signed
/// distance runs on along its tangent extension, and a trace read by that alone went back and forth
/// at the edge a hundred thousand times.
#[test]
fn a_trace_off_a_sheets_patch_ends_at_its_edge() {
    use gcs_core::brep::geom::Surface;
    // a flat sheet over the unit square, and a ball centred on its edge x = 1
    let points: Vec<V> = (0..6).flat_map(|i| (0..6).map(move |j| [i as f64/5.,j as f64/5.,0.])).collect();
    let net = gcs_core::brep::nurbs::interpolate_net(&points,6,6,gcs_core::brep::nurbs::Parametrization::Even).unwrap();
    let sheet = Surface::BSpline(Frame::about([0.;3],[0.,0.,1.]),std::sync::Arc::new(net));
    let ball = Surface::Sphere(Frame::about([1.,0.5,0.],[0.,0.,1.]),0.3);
    let started = std::time::Instant::now();
    let curves = gcs_core::brep::ssi::trace(&ball,&sheet,&[[0.7,0.5,0.]],[-1.;3],[2.;3],1e-9).unwrap();
    assert!(started.elapsed().as_secs_f64() < 5.,"{:?}",started.elapsed());
    assert_eq!(curves.len(),1);
    // the half circle on the patch, its ends at x = 1 (within a step's halving at the edge)
    let c = &curves[0];
    let [t0,t1] = span(c);
    let ends = [c.point(t0),c.point(t1)];
    for e in ends { assert!((e[0]-1.).abs() < 1e-3 && e[0] <= 1.+1e-6,"{ends:?}"); }
}

/// A ball about the axis through `c` along the unit `axis`, its profile in the plane square to
/// `normal` (square to the axis): the seam where that plane cuts it.
fn ball_about(c: V,r: f64,axis: V,normal: V) -> gcs_core::brep::topo::Brep {
    use gcs_core::space::{add,cross};
    let pole = |s: f64| add(c,scale3(axis,s));
    let p = Profile {names:vec![],origin:c,normal,loops:vec![vec![arc(c,r,normal,cross(axis,normal),Some([-PI/2.,PI/2.])),
        line(pole(-r),pole(r))]]};
    revolve(&p,c,axis,TAU).unwrap()
}

/// A torus about z through the origin, of major radius `big` and tube radius `r`.
fn torus(big: f64,r: f64) -> gcs_core::brep::topo::Brep {
    let p = Profile {names:vec![],origin:[0.;3],normal:XZ,loops:vec![vec![arc([big,0.,0.],r,XZ,[1.,0.,0.],None)]]};
    revolve(&p,[0.;3],[0.,0.,1.],TAU).unwrap()
}

/// The volume a ball about `c` of radius `r` shares with the torus about z of radii `big`, `tube`:
/// over the ball's disk seen from above, the overlap of the two vertical chords.
fn ball_in_torus(c: V,r: f64,big: f64,tube: f64) -> f64 {
    simpson(0.,1.,800,|s: f64| simpson(0.,TAU,800,|th: f64| {
        // s² spaced radially, so the integrand is smooth at the disk's rim
        let d = r*(1.-(1.-s)*(1.-s));
        let (x,y) = (c[0]+d*th.cos(),c[1]+d*th.sin());
        let ball = (r*r-d*d).max(0.).sqrt();
        let rho = x.hypot(y);
        let ring = (tube*tube-(rho-big)*(rho-big)).max(0.).sqrt();
        let chord = ((c[2]+ball).min(ring)-(c[2]-ball).max(-ring)).max(0.);
        chord*d*r*2.*(1.-s)
    }))
}

#[test]
fn a_closed_intersection_crossing_no_edge_is_found_whichever_way_a_seam_runs() {
    // issue #58: a small ball pressed into a torus meets it in one closed curve. About y its seam
    // misses that curve, so no edge of either crosses the other's face; about z it crosses it.
    use gcs_core::brep::boolean::{boolean,Op};
    let (big,tube) = (3.,1.);
    let ring = torus(big,tube);
    let whole = 2.*PI*PI*big*tube*tube;
    close(volume(&ring),whole);
    let at = 3./2f64.sqrt();
    let r = 0.2;
    let sphere = 4./3.*PI*r*r*r;
    for (what,z,inside) in [("a shallow pocket",1.1,None),("half sunk",0.9,None),("disjoint",1.3,Some(0.)),
        ("contained",0.5,Some(sphere))] {
        let c = [at,at,z];
        let shared = inside.unwrap_or_else(|| ball_in_torus(c,r,big,tube));
        for (seam,axis,normal) in [("about y",[0.,1.,0.],XY),("about z",[0.,0.,1.],XZ),("about x",[1.,0.,0.],XY)] {
            let ball = ball_about(c,r,axis,normal);
            for (op,want) in [(Op::Cut,whole-shared),(Op::Common,shared),(Op::Union,whole+sphere-shared)] {
                let out = boolean(&ring,&ball,op,1e-9).unwrap_or_else(|e| panic!("{what}, {seam}, {op:?}: {e}"));
                out.check(1e-8).unwrap_or_else(|e| panic!("{what}, {seam}, {op:?}: {e}"));
                let v = volume(&out);
                // the reference is a quadrature over a kinked integrand: good to ~1e-7 of the ball
                assert!((v-want).abs() <= 1e-6*sphere,"{what}, {seam}, {op:?}: {v} against {want} ({:e} of the ball)",(v-want)/sphere);
            }
        }
    }
}

#[test]
fn one_closed_intersection_found_does_not_hide_another() {
    // a ball about a point of a torus's core circle, the tube through it: two loops, where the tube
    // goes in and where it comes out. Traced from a point on one, the search finds the other.
    use gcs_core::brep::geom::{Curve,Frame,Surface};
    use gcs_core::brep::ssi::{trace,unseen};
    let tol = 1e-9;
    let ring = Surface::Torus(Frame::new([0.;3],[0.,0.,1.],[1.,0.,0.]),3.,1.);
    let ball = Surface::Sphere(Frame::new([3.,0.,0.],[0.,0.,1.],[1.,0.,0.]),1.5);
    let (lo,hi) = ([-5.;3],[5.;3]);
    let whole = [[0.,TAU],[0.,TAU]];
    // along the top of the tube, 1.5 from the ball's centre
    let u = ((19.-2.25)/18f64).acos();
    let first = trace(&ring,&ball,&[[3.*u.cos(),3.*u.sin(),1.]],lo,hi,tol).unwrap();
    assert_eq!(first.len(),1);
    assert!(matches!(&first[0],Curve::Traced(t) if t.closed));
    let other = unseen(&ring,whole,&ball,&first,lo,hi,tol).unwrap().expect("the second loop");
    assert!(other[1] < -0.5 && ring.implicit(other).abs() <= 1e-8 && ball.implicit(other).abs() <= 1e-8,"{other:?}");
    let mut both = first.clone();
    both.extend(trace(&ring,&ball,&[other],lo,hi,tol).unwrap());
    assert_eq!(unseen(&ring,whole,&ball,&both,lo,hi,tol).unwrap(),None);
    // from the ball's side too
    assert!(unseen(&ball,[[0.,TAU],[-PI/2.,PI/2.]],&ring,&first,lo,hi,tol).unwrap().is_some_and(|p| p[1] < -0.5));
    // and nothing where the two are clear of each other
    let clear = Surface::Sphere(Frame::new([3.,0.,1.5],[0.,0.,1.],[1.,0.,0.]),0.4);
    assert_eq!(unseen(&ring,whole,&clear,&[],lo,hi,tol).unwrap(),None);
    assert_eq!(unseen(&clear,[[0.,TAU],[-PI/2.,PI/2.]],&ring,&[],lo,hi,tol).unwrap(),None);
}

#[test]
fn a_ball_resting_on_a_torus_touches_it_alone_and_one_sunk_in_is_not_missed() {
    // resting on the top of the tube the two touch at one point, read to second order wherever
    // the ball's seam runs: the meeting there is that point alone, and the ring is unchanged
    use gcs_core::brep::boolean::{boolean,Op};
    let ring = torus(3.,1.);
    let at = 3./2f64.sqrt();
    for (seam,axis,normal) in [("about y",[0.,1.,0.],XY),("about z",[0.,0.,1.],XZ)] {
        let ball = ball_about([at,at,1.2],0.2,axis,normal);
        let out = boolean(&ring,&ball,Op::Cut,1e-9).unwrap_or_else(|e| panic!("{seam}: {e}"));
        out.check(1e-8).unwrap();
        close(volume(&out),volume(&ring));
        // sunk a thousandth in, they cross in a small circle — no touch, though nearly one: traced,
        // never read as a point alone, the ring less a lens no larger than the ball's cap that deep
        let sunk = ball_about([at,at,1.199],0.2,axis,normal);
        let out = boolean(&ring,&sunk,Op::Cut,1e-9).unwrap_or_else(|e| panic!("{seam}: {e}"));
        let (lens,cap) = (volume(&ring)-volume(&out),PI*1e-6*(3.*0.2-1e-3)/3.);
        assert!(lens > 0.1*cap && lens < cap,"{seam}: {lens} against a cap of {cap}");
    }
}

#[test]
fn the_dimpled_ring_example_is_the_ring_less_the_ball_it_shares() {
    // examples/solid_dimpled_ring.sv: the same pocket at ten times the size, written in Solvent,
    // the ball's seam above the ring
    let src = include_str!("../../examples/solid_dimpled_ring.sv");
    let (prog,errs,_) = gcs_core::library::parse_linked(src);
    assert!(errs.is_empty(),"{errs:?}");
    let mut e = gcs_core::program::elaborate(&prog);
    assert!(e.ok(),"{:?}",e.errors().map(|d| d.message.clone()).collect::<Vec<_>>());
    assert!(gcs_core::solve::solve(&mut e.sketch,gcs_core::solve::SolveOpts::default()).success);
    let part = e.map.ent_named("part").unwrap().i();
    let b = gcs_core::brep::recipe::build(&gcs_core::solid::cad::recipe(&e.sketch,part).unwrap()).unwrap();
    b.check(1e-6).unwrap();
    let at = 30./2f64.sqrt();
    let want = 2.*PI*PI*30.*100.-ball_in_torus([at,at,11.],2.,30.,10.);
    assert!((volume(&b)-want).abs() <= 1e-6*4./3.*PI*8.,"{} against {want}",volume(&b));
}

/// Every point of a mesh of `b` (each on its surface) within `b.bounds()`, and the box `want` to
/// rounding where it is known (issue #59: a box short of its solid's extremes made a `through:`
/// cutter stop short of the stock).
fn boxed(b: &gcs_core::brep::topo::Brep,want: Option<(V,V)>) {
    let (lo,hi) = b.bounds();
    assert!(lo.iter().chain(&hi).all(|x| x.is_finite()),"{lo:?} {hi:?}");
    let slack = 1e-12*(1.+gcs_core::space::distance(lo,hi));
    let m = gcs_core::brep::mesh::mesh(b,1e-3*b.size(),0.1).unwrap();
    for p in &m.pts {
        assert!((0..3).all(|k| p[k] >= lo[k]-slack && p[k] <= hi[k]+slack),"{p:?} outside {lo:?} {hi:?}");
    }
    if let Some((a,z)) = want {
        for k in 0..3 {
            assert!((lo[k]-a[k]).abs() <= slack && (hi[k]-z[k]).abs() <= slack,"{lo:?} {hi:?} against {a:?} {z:?}");
        }
    }
}

#[test]
fn curve_boxes_hold_every_point_and_reach_the_extremes() {
    use gcs_core::brep::geom::Curve;
    let n = [0.3,-0.5,0.8];
    let len = (0.09f64+0.25+0.64).sqrt();
    for k in 0..37 {
        let phase = TAU*k as f64/37.+PI/16.;
        let x = [phase.cos(),phase.sin(),0.];
        let f = Frame::new([1.,-2.,3.],n,x);
        for c in [Curve::Circle(f,10.),Curve::Ellipse(f,10.,4.)] {
            // the whole curve: in each coordinate the centre ± the amplitude, closed form
            let (lo,hi) = c.bounds([0.,TAU]);
            if let Curve::Circle(..) = c {
                for i in 0..3 {
                    let half = 10.*(1.-(n[i]/len).powi(2)).sqrt();
                    assert!((hi[i]-(f.o[i]+half)).abs() < 1e-12 && (lo[i]-(f.o[i]-half)).abs() < 1e-12,"{phase}: {lo:?} {hi:?}");
                }
            }
            // and arcs of it, starting anywhere and running any distance up to and past a turn
            for span in [[0.3,1.1],[-2.,0.5],[phase,phase+4.],[5.,12.]] {
                let (lo,hi) = c.bounds(span);
                let (mut a,mut z) = ([f64::INFINITY;3],[f64::NEG_INFINITY;3]);
                for j in 0..=4096 {
                    let p = c.point(span[0]+(span[1]-span[0])*j as f64/4096.);
                    for i in 0..3 { a[i] = a[i].min(p[i]); z[i] = z[i].max(p[i]); }
                }
                for i in 0..3 {
                    assert!(lo[i] <= a[i]+1e-12 && hi[i] >= z[i]-1e-12,"{span:?}: {lo:?} {hi:?} misses {a:?} {z:?}");
                    // no looser than the sampling's own shortfall
                    assert!(a[i]-lo[i] < 1e-5 && hi[i]-z[i] < 1e-5,"{span:?}: {lo:?} {hi:?} against {a:?} {z:?}");
                }
            }
        }
    }
    // a B-spline's stretch: within its poles' hull, so never short of a point
    let s = cubic(&[[10.,0.,0.],[12.,8.,1.],[9.,13.,-2.],[6.,14.,0.],[0.,10.,3.]],&[0.4]);
    let c = Curve::BSpline(s);
    for span in [[0.,1.],[0.1,0.35],[0.3,0.9]] {
        let (lo,hi) = c.bounds(span);
        for j in 0..=4096 {
            let p = c.point(span[0]+(span[1]-span[0])*j as f64/4096.);
            assert!((0..3).all(|i| p[i] >= lo[i]-1e-12 && p[i] <= hi[i]+1e-12),"{span:?}: {p:?} outside {lo:?} {hi:?}");
        }
    }
}

#[test]
fn a_cylinder_is_boxed_by_its_rim_whichever_way_its_circle_starts() {
    // the issue's stock: its circle's frame turned by phases between any samples' (11.25° among
    // them), the solid the same, its box the same
    for k in 0..=64 {
        let phase = TAU*k as f64/64.+if k == 64 { PI/16. } else { 0. };
        let rim = arc([0.;3],10.,XY,[phase.cos(),phase.sin(),0.],None);
        let b = prism(&Profile {names:vec![],origin:[0.;3],normal:XY,loops:vec![vec![rim]]},0.,2.).unwrap();
        boxed(&b,Some(([-10.,-10.,0.],[10.,10.,2.])));
    }
    // standing on a slant: each cap's centre ± its radius across the axis in each coordinate
    let n = [1.,1.,1.];
    let b = prism(&Profile {names:vec![],origin:[1.,2.,3.],normal:n,loops:vec![vec![arc([1.,2.,3.],2.,n,[1.,-1.,0.],None)]]},0.,5.).unwrap();
    let (across,up) = (2.*(2f64/3.).sqrt(),5./3f64.sqrt());
    boxed(&b,Some(([1.-across,2.-across,3.-across],[1.+up+across,2.+up+across,3.+up+across])));
}

#[test]
fn balls_tori_and_turns_are_boxed_by_their_closed_forms() {
    boxed(&ball([1.,2.,3.],4.),Some(([-3.,-2.,-1.],[5.,6.,7.])));
    boxed(&ball_about([1.,2.,3.],4.,[0.6,0.,0.8],[0.,1.,0.]),Some(([-3.,-2.,-1.],[5.,6.,7.])));
    boxed(&torus(5.,1.5),Some(([-6.5,-6.5,-1.5],[6.5,6.5,1.5])));
    // turned off its axes: R across the axis in each coordinate, and r every way
    let m = Rigid::turn([0.;3],[1.,-2.,0.5],0.7);
    let a = m.vector([0.,0.,1.]);
    let half: V = std::array::from_fn(|k| 5.*(1.-a[k]*a[k]).sqrt()+1.5);
    boxed(&torus(5.,1.5).moved(&m),Some((half.map(|h| -h),half)));
    // a tube turned through one radian, then back by 0.3: it runs from −0.3 to 0.7 about z, its
    // greatest x (4, at no turn) inside its face and on none of its edges
    let p = Profile {names:vec![],origin:[0.;3],normal:XZ,loops:vec![vec![arc([3.,0.,1.],1.,XZ,[1.,0.,0.],None)]]};
    let back = Rigid::turn([0.;3],[0.,0.,1.],-0.3);
    let tube = revolve(&p,[0.;3],[0.,0.,1.],1.).unwrap().moved(&back);
    boxed(&tube,Some(([2.*0.7f64.cos(),-4.*0.3f64.sin(),0.],[4.,4.*0.7f64.sin(),2.])));
    // a spline turned the same way: a surface of revolution, its box held to its mesh
    let s = cubic(&[[2.,0.,0.],[5.,0.,1.],[3.,0.,2.],[2.,0.,3.]],&[]);
    let p = Profile {names:vec![],origin:[0.;3],normal:XZ,loops:vec![vec![ProfileEdge::Spline(s),line([2.,0.,3.],[2.,0.,0.])]]};
    let turned = revolve(&p,[0.;3],[0.,0.,1.],1.).unwrap().moved(&back);
    assert!(turned.faces.iter().any(|f| f.surface.kind() == "revolution"));
    boxed(&turned,None);
}

#[test]
fn a_cut_along_a_traced_edge_is_boxed() {
    use gcs_core::brep::boolean::{boolean,Op};
    let lobe = cubic(&[[10.,0.,0.],[12.,8.,0.],[9.,13.,0.],[6.,14.,0.],[0.,10.,0.]],&[0.4]);
    let plate = Profile {names:vec![],origin:[0.;3],normal:XY,loops:vec![vec![line([0.,0.,0.],[10.,0.,0.]),ProfileEdge::Spline(lobe),line([0.,10.,0.],[0.,0.,0.])]]};
    let b = prism(&plate,0.,3.).unwrap();
    boxed(&b,None);
    // a ball through it meets the spline's extrusion in traced curves
    let r = boolean(&b,&ball([10.,8.,1.5],2.5),Op::Cut,1e-9).unwrap();
    r.check(1e-8).unwrap();
    assert!(r.edges.iter().any(|e| matches!(&e.curve,gcs_core::brep::topo::EdgeCurve::Curve(c) if c.kind() == "traced")));
    boxed(&r,None);
}

/// A cylinder of radius `r` from `z0` to `z1` about z, its profile in the half-plane at angle `at`
/// from x: its seam, and its circles' vertices, stand there.
fn seamed_cylinder(r: f64,z0: f64,z1: f64,at: f64) -> gcs_core::brep::topo::Brep {
    let (c,s) = (at.cos(),at.sin());
    let p = |x: f64,z: f64| [x*c,x*s,z];
    let q = Profile {names:vec![],origin:[0.;3],normal:[-s,c,0.],loops:vec![poly(&[p(0.,z0),p(r,z0),p(r,z1),p(0.,z1)])]};
    revolve(&q,[0.;3],[0.,0.,1.],TAU).unwrap()
}

/// Two cylinders on one axis standing end to end, each seamed at its own angle, united: their
/// shared end circle is one curve that each solid's edges cut at different vertices, and the
/// Boolean cuts each edge at the other's vertices so the two become one.
#[test]
fn coaxial_cylinders_seamed_apart_unite_end_to_end() {
    use gcs_core::brep::boolean::{boolean,Op};
    let (a,b) = (seamed_cylinder(5.,0.,6.,0.),seamed_cylinder(5.,-3.,0.,0.7));
    let u = boolean(&a,&b,Op::Union,1e-9).unwrap();
    u.check(1e-8).unwrap();
    close(volume(&u),PI*25.*9.);
    // a narrower one on the wider one's end, seamed apart too: the circles differ, the planes agree
    let c = seamed_cylinder(3.,-3.,0.,2.1);
    let v = boolean(&a,&c,Op::Union,1e-9).unwrap();
    v.check(1e-8).unwrap();
    close(volume(&v),PI*25.*6.+PI*9.*3.);
}

/// A cone turned from a profile whose apex is on the axis only to rounding (across it, here) is
/// placed at its apex with radius zero, never the negative a file's reader refuses.
#[test]
fn a_cone_turned_from_an_apex_on_its_axis_has_radius_zero_there() {
    use gcs_core::brep::geom::Surface;
    // walked either way round and turned either way, the cone is placed at either end of its line
    for pts in [[[-1e-11,0.,3.],[2.,0.,0.],[0.,0.,0.]],[[0.,0.,0.],[2.,0.,0.],[-1e-11,0.,3.]]] {
        for axis in [[0.,0.,1.],[0.,0.,-1.]] {
            let q = Profile {names:vec![],origin:[0.;3],normal:XZ,loops:vec![poly(&pts)]};
            let b = revolve(&q,[0.;3],axis,TAU).unwrap();
            let radii: Vec<f64> = b.faces.iter().filter_map(|f| match f.surface { Surface::Cone(_,r,_) => Some(r),_ => None }).collect();
            assert!(!radii.is_empty() && radii.iter().all(|&r| r >= 0.),"{radii:?}");
        }
    }
}

/// A face thinner than its edges' chords sag — a circle's arc, cut by a coarse bar and turn into
/// two chords whose middles sag 0.35 inside it, and a corner of the other side 0.2 inside it there —
/// has sampled loops that cross: the mesher splits the stretches that cross until its loops close a
/// triangulation, as a display mesh of the drill's end face needed.
#[test]
fn a_face_thinner_than_its_chords_sag_still_meshes() {
    let r: f64 = 5.;
    let at = |a: f64,d: f64| [d*a.cos(),d*a.sin(),0.];
    let (p1,p2,corner) = (at(-0.3,r),at(1.2,r),at(0.075,4.8));
    let p = Profile {names:vec![],origin:[0.;3],normal:XY,loops:vec![vec![
        arc([0.;3],r,XY,[1.,0.,0.],Some([-0.3,1.2])),line(p2,corner),line(corner,p1),
    ]]};
    let b = prism(&p,0.,1.).unwrap();
    b.check(1e-9).unwrap();
    let m = gcs_core::brep::mesh::mesh(&b,0.5,0.8).unwrap();
    assert!(m.tris.len() > 4);
}

#[test]
fn curves_cross_in_closed_form() {
    use gcs_core::brep::geom::Curve;
    use gcs_core::brep::query::curve_curve;
    let near = |got: Vec<(f64,V)>,want: &[(f64,V)]| {
        assert_eq!(got.len(),want.len(),"{got:?} against {want:?}");
        for ((t,p),(u,q)) in got.iter().zip(want) {
            assert!((t-u).abs() < 1e-12 && (0..3).all(|k| (p[k]-q[k]).abs() < 1e-12),"{got:?} against {want:?}");
        }
    };
    let x = Curve::Line {p:[0.,0.,0.],d:[1.,0.,0.]};
    near(curve_curve(&x,[0.,10.],&Curve::Line {p:[5.,-5.,0.],d:[0.,1.,0.]},[0.,10.],1e-9).unwrap(),&[(5.,[5.,0.,0.])]);
    // skew, and crossing beyond the other's stretch
    near(curve_curve(&x,[0.,10.],&Curve::Line {p:[5.,-5.,1.],d:[0.,1.,0.]},[0.,10.],1e-9).unwrap(),&[]);
    near(curve_curve(&x,[0.,10.],&Curve::Line {p:[5.,1.,0.],d:[0.,1.,0.]},[0.,10.],1e-9).unwrap(),&[]);
    let circle = Curve::Circle(Frame::new([0.;3],XY,[1.,0.,0.]),2.);
    // a line through the circle's plane, and one in it
    near(curve_curve(&Curve::Line {p:[2.,0.,-5.],d:XY},[0.,10.],&circle,[0.,TAU],1e-9).unwrap(),&[(5.,[2.,0.,0.])]);
    let s3 = 3f64.sqrt();
    near(curve_curve(&Curve::Line {p:[-5.,1.,0.],d:[1.,0.,0.]},[0.,10.],&circle,[0.,TAU],1e-9).unwrap(),
        &[(5.-s3,[-s3,1.,0.]),(5.+s3,[s3,1.,0.])]);
    // two circles in one plane, and in planes square to one another; a half circle has one of two
    let beside = Curve::Circle(Frame::new([2.,0.,0.],XY,[1.,0.,0.]),2.);
    let got = curve_curve(&circle,[0.,TAU],&beside,[0.,TAU],1e-9).unwrap();
    let mut pts: Vec<V> = got.iter().map(|g| g.1).collect();
    pts.sort_by(|a,b| a[1].total_cmp(&b[1]));
    assert!((pts[0][0]-1.).abs() < 1e-12 && (pts[0][1]+s3).abs() < 1e-12 && (pts[1][1]-s3).abs() < 1e-12,"{got:?}");
    let upright = Curve::Circle(Frame::new([0.;3],[1.,0.,0.],[0.,1.,0.]),2.);
    assert_eq!(curve_curve(&circle,[0.,TAU],&upright,[0.,TAU],1e-9).unwrap().len(),2);
    near(curve_curve(&circle,[0.,PI],&upright,[0.,TAU],1e-9).unwrap(),&[(PI/2.,[0.,2.,0.])]);
    // a tangency is one touch, where it is: not two crossings the square root of a rounding apart
    // either side of it (a fillet's ball touching the circle it rolls on)
    near(curve_curve(&circle,[0.,TAU],&Curve::Circle(Frame::new([3.,0.,0.],XY,[1.,0.,0.]),1.),[0.,TAU],1e-9).unwrap(),
        &[(0.,[2.,0.,0.])]);
    let r = 6.0f64;
    let tilt = Curve::Circle(Frame::new([0.;3],XY,[1.,0.,0.]),r);
    let touching = Curve::Circle(Frame::new([r+2.,1e-12,0.],XY,[1.,0.,0.]),2.);
    let got = curve_curve(&tilt,[-1.,1.],&touching,[0.,TAU],1e-9).unwrap();
    assert!(got.len() == 1 && (got[0].1[0]-r).abs() < 1e-9 && got[0].1[1].abs() < 1e-9,"{got:?}");
    // no closed form for any other curve
    assert!(curve_curve(&Curve::Ellipse(Frame::new([0.;3],XY,[1.,0.,0.]),2.,1.),[0.,TAU],&x,[0.,10.],1e-9).is_none());
}

#[test]
fn a_point_on_a_seam_is_in_its_face() {
    use gcs_core::brep::query::{Located,Place};
    let b = rod([0.;3],2.,[0.,4.]);
    let located = Located::new(&b,1e-9);
    let wall = b.faces.iter().position(|f| f.surface.kind() == "cylinder").unwrap();
    // the seam runs up the wall at its first parameter, here +x: no boundary of the face
    assert_eq!(located.face_place(wall,[2.,0.,2.]),Place::In);
    assert_eq!(located.face_place(wall,[0.,2.,2.]),Place::In);
    assert_eq!(located.face_place(wall,[2.,0.,4.]),Place::On);
}

/// A fillet's section in the meridian of the line along z: the corner at radius 8, the ball of
/// radius 3 touching the plane `z = 0` and the cylinder of radius 8.
fn wedge_section(origin: V) -> Profile {
    let at = |p: V| [p[0]+origin[0],p[1]+origin[1],p[2]+origin[2]];
    let (e,tp,tw,c) = (at([8.,0.,0.]),at([11.,0.,0.]),at([8.,0.,3.]),at([11.,0.,3.]));
    // seen from +y an angle turns x toward −z: the plane's touch at π/2, the wall's at π
    Profile {names:vec![],origin:e,normal:XZ,loops:vec![vec![line(e,tp),arc(c,3.,XZ,[1.,0.,0.],Some([PI/2.,PI])),line(tw,e)]]}
}

#[test]
fn a_section_with_cusps_meshes_at_every_bar() {
    // a line meeting an arc tangent to it is a cusp, where a thin face's boundary runs past the
    // corners of the triangle the mesh starts in
    let b = prism(&wedge_section([0.;3]),0.,40.).unwrap();
    b.check(1e-9).unwrap();
    close(volume(&b),(1.-PI/4.)*9.*40.);
    for k in 0..12 {
        let bar = 1e-1/2f64.powi(k);
        gcs_core::brep::mesh::mesh(&b,bar,TAU/64.).unwrap_or_else(|e| panic!("at bar {bar}: {e}"));
    }
}

#[test]
fn a_ring_on_a_coincident_wall_joins_it() {
    use gcs_core::brep::boolean::Op;
    use gcs_core::brep::boolean::combined;
    // a rod standing on a block, and the ring a ball rolled round its foot fills: one side of the
    // ring lies on the rod's wall and is seamed elsewhere, its contact circle crossing the rod's seam
    let ring = revolve(&wedge_section([0.;3]),[0.;3],XY,TAU).unwrap();
    ring.check(1e-9).unwrap();
    let foot = combined(&block([-30.,-30.,-10.],[30.,30.,0.]),&rod([0.;3],8.,[0.,20.]),Op::Union,0.).unwrap();
    let whole = combined(&foot,&ring,Op::Union,0.).unwrap();
    let moment = 9.*(8.+1.5)-PI*9./4.*(8.+3.-4.*3./(3.*PI));
    close(volume(&ring),TAU*moment);
    close(volume(&whole),36000.+PI*64.*20.+TAU*moment);
}

#[test]
fn a_surface_offset_moves_its_signed_distance_by_the_offset() {
    use gcs_core::brep::geom::Surface;
    let f = Frame::new([1.,-2.,0.5],[0.3,0.4,1.],[1.,0.,0.]);
    let surfaces = [Surface::Plane(f),Surface::Cylinder(f,4.),Surface::Cone(f,3.,0.4),Surface::Sphere(f,5.),
        Surface::Torus(f,6.,2.)];
    let points = [[0.,0.,0.],[3.,1.,-2.],[-4.,6.,3.],[7.,-1.,4.]];
    for s in &surfaces {
        for d in [-1.5,0.25,2.] {
            let o = s.offset(d).unwrap_or_else(|| panic!("a {} offset by {d}",s.kind()));
            for p in points { close(o.implicit(p),s.implicit(p)-d); }
        }
    }
    // past its axis or its centre a surface offset inward is no surface
    assert!(Surface::Cylinder(f,4.).offset(-4.).is_none());
    assert!(Surface::Torus(f,6.,2.).offset(-3.).is_none());
}

/// The main pipe of a tee, radius 10 along x with its seam along +z, and its branch, radius 6
/// along z; and the upper loop where they meet.
fn tee_loop() -> (gcs_core::brep::geom::Surface,gcs_core::brep::geom::Curve) {
    use gcs_core::brep::geom::Surface;
    use gcs_core::brep::ssi::trace;
    let main = Surface::Cylinder(Frame::new([0.;3],[1.,0.,0.],[0.,0.,1.]),10.);
    let stem = Surface::Cylinder(Frame::new([0.;3],[0.,0.,1.],[1.,0.,0.]),6.);
    let seed = [0.,6.,8.];
    let mut loops = trace(&main,&stem,&[seed],[-20.;3],[20.;3],1e-9).unwrap();
    loops.sort_by(|a,b| a.point(a.inverse(seed)).iter().zip(seed).map(|(x,y)| (x-y).abs()).sum::<f64>()
        .total_cmp(&b.point(b.inverse(seed)).iter().zip(seed).map(|(x,y)| (x-y).abs()).sum::<f64>()));
    (main,loops.swap_remove(0))
}

#[test]
fn a_traced_edge_lying_in_a_cylinder_crosses_its_seam() {
    use gcs_core::brep::geom::Curve;
    use gcs_core::brep::query::crossings_in;
    let (main,crotch) = tee_loop();
    let period = crotch.period().expect("a closed loop");
    // the main pipe's seam, where the loop crosses it at x = ±6
    let seam = Curve::Line {p:[0.,0.,10.],d:[1.,0.,0.]};
    let mut found = crossings_in(&main,&crotch,[0.,period],&seam,[-30.,30.],1e-9).expect("a line carries its crossings");
    found.sort_by(|a,b| a.1[0].total_cmp(&b.1[0]));
    assert_eq!(found.len(),2,"{found:?}");
    for ((_,q),x) in found.iter().zip([-6.,6.]) {
        for (k,want) in [x,0.,10.].into_iter().enumerate() { assert!((q[k]-want).abs() <= 1e-7,"{q:?}"); }
    }
    // and read from the line's side, the same two
    assert_eq!(crossings_in(&main,&seam,[-30.,30.],&crotch,[0.,period],1e-9).unwrap().len(),2);
}

/// A rolled fillet's canal touches each operand along a whole contact edge: the union traces the
/// canal against the pipes away from that contact, where their gradients are parallel, and reads
/// the pair as meeting only there.
#[test]
fn a_canal_touching_its_operands_along_an_edge_is_unioned_with_them() {
    use gcs_core::brep::boolean::{boolean,Op};
    let main = prism(&Profile {names:vec![],origin:[0.;3],normal:[1.,0.,0.],
        loops:vec![vec![arc([0.,0.,0.],10.,[1.,0.,0.],[0.,0.,1.],None)]]},-30.,30.).unwrap();
    let tee = boolean(&main,&rod([0.,0.,0.],6.,[0.,25.]),Op::Union,1e-9).unwrap();
    let rolled = gcs_core::brep::fillet::roll(&tee,[6.,0.,10.],2.,1e-9).unwrap();
    assert!(rolled.concave);
    rolled.piece.check(1e-6).unwrap();
    let whole = boolean(&tee,&rolled.piece,Op::Union,1e-9).unwrap();
    whole.check(1e-6).unwrap();
    let (got,want) = (volume(&whole),volume(&tee)+volume(&rolled.piece));
    assert!((got-want).abs() <= 1e-9*want,"{got} against {want}");
    // the pipes' faces it rounds are kept, cut back to the contacts: nothing is left of the edge
    assert!(whole.faces.len() > tee.faces.len());
}

/// Two solids ending in one face, material either side of it, are joined there: the face goes,
/// its edges are shared by the faces beside them, nothing is intersected.
#[test]
fn solids_sharing_a_face_are_glued_along_it() {
    use gcs_core::brep::boolean::glued;
    let square = vec![poly(&[[1.,0.,0.],[2.,0.,0.],[2.,0.,1.],[1.,0.,1.]])];
    let p = Profile {names:vec![],origin:[0.;3],normal:XZ,loops:square};
    // a quarter turn about z, from the square on into +y, and a prism behind it into −y
    let turned = revolve(&p,[0.;3],[0.,0.,1.],PI/2.).unwrap();
    let straight = prism(&p,-3.,0.).unwrap();
    for (a,b) in [(&turned,&straight),(&straight,&turned)] {
        let g = glued(a,b,1e-9).expect("they share the square");
        g.check(1e-9).unwrap();
        close(volume(&g),3.+PI*(4.-1.)/4.);
        assert_eq!(g.faces.len(),a.faces.len()+b.faces.len()-2);
    }
    // two solids that only touch share no face
    assert!(glued(&straight,&block([5.,5.,5.],[6.,6.,6.]),1e-9).is_none());
}

/// Two solids sharing a face whose edges each holds in pieces of its own, cut at different points,
/// are glued in either order (#424): one side runs whole where the other is in two, and both in two
/// where they are cut apart; each is cut where the other is, its pieces shared.
#[test]
fn solids_sharing_a_face_cut_differently_are_glued_either_way() {
    use gcs_core::brep::boolean::glued;
    let at = |pts: &[[f64;2]],z: [f64;2]| {
        let pts: Vec<V> = pts.iter().map(|&[x,y]| [x,y,0.]).collect();
        prism(&Profile {names:vec![],origin:[0.;3],normal:XY,loops:vec![poly(&pts)]},z[0],z[1]).unwrap()
    };
    // below, the square's near side in two at x = 0.5; above, at x = 0.3, and its left side in two
    let below = at(&[[0.,0.],[0.5,0.],[1.,0.],[1.,1.],[0.,1.]],[0.,1.]);
    let above = at(&[[0.,0.],[0.3,0.],[1.,0.],[1.,1.],[0.,1.],[0.,0.6]],[1.,2.]);
    for (a,b) in [(&below,&above),(&above,&below)] {
        let g = glued(a,b,1e-9).expect("they share the square");
        g.check(1e-9).unwrap();
        close(volume(&g),2.);
        assert_eq!(g.faces.len(),a.faces.len()+b.faces.len()-2);
        // the square's sides, five edges below and six above, in seven pieces between them (the
        // near side in three, the left in two), each used by a face either side
        assert_eq!(g.edges.len(),a.edges.len()+b.edges.len()-5-6+7);
    }
}

/// Where two faces touch at a point of both their boundaries and their meeting runs into neither
/// beyond it, they meet there alone: a quarter torus whose start touches a block's side at its
/// inner equator (the plane there crosses the torus in two branches, off both faces) is their
/// union, as it was before the second-order reading refused it as nearly touching; and where one
/// branch runs into both, it is traced up to the touch.
#[test]
fn a_quarter_torus_touching_a_block_at_a_corner_meets_it_there_alone() {
    use gcs_core::brep::boolean::{boolean,Op};
    // the torus about z, its tube of radius 1 about the circle of radius 3, from the x axis on
    // into +y; the block below y = 0 with its side x = 2 at the inner equator
    let tube = vec![vec![arc([3.,0.,0.],1.,XZ,[1.,0.,0.],None)]];
    let quarter = revolve(&Profile {names:vec![],origin:[0.;3],normal:XZ,loops:tube},[0.;3],[0.,0.,1.],PI/2.).unwrap();
    let b = boolean(&block([-1.,-3.,-1.],[2.,0.,1.]),&quarter,Op::Union,1e-9).unwrap();
    b.check(1e-8).unwrap();
    close(volume(&b),18.+1.5*PI*PI);
    // the block run on past y = 0: the torus cuts into it through that point, along a branch into
    // both faces — traced up to the touch and ended there (#218's 1). What they share: at each
    // distance ρ from the axis the tube's section turned from where it leaves x ≤ 2 to where it
    // leaves y ≤ 1, `asin(1/ρ) − acos(2/ρ)`, over ρ in (2, √5), by quadrature (ρ = 2 + (√5 − 2) u²)
    let crossed = boolean(&block([-1.,-3.,-1.],[2.,1.,1.]),&quarter,Op::Union,1e-9).unwrap();
    crossed.check(1e-8).unwrap();
    let (lo,hi) = (2f64,5f64.sqrt());
    let n = 20_000;
    let g = |u: f64| {
        let rho = lo+(hi-lo)*u*u;
        let turn = ((1./rho).min(1.).asin()-(2./rho).min(1.).acos()).max(0.);
        turn*rho*2.*(1.-(rho-3.)*(rho-3.)).max(0.).sqrt()*2.*(hi-lo)*u
    };
    let shared = simpson(0.,1.,n,g);
    assert!((volume(&crossed)-(24.+1.5*PI*PI-shared)).abs() <= 1e-9*24.,"{} against {}",volume(&crossed),24.+1.5*PI*PI-shared);
}

/// A corner where a fillet rounds all three edges: the cell between the vertex and the ball's
/// centre less the ball — `r³ − πr³/6` on a square corner; on a slanted one, the cell fanned into
/// tetrahedra from the centre, less the ball within it (its solid angle times `r³/3`).
#[test]
fn a_spherical_corner_is_its_cell_less_the_ball() {
    use gcs_core::brep::fillet::corner;
    use gcs_core::space::{add,cross,dot,norm,scale,sub};
    let unit = |a: V| scale(a,1./norm(a));
    let r = 1.5;
    let square = corner([0.;3],[r;3],r,[[1.,0.,0.],[0.,1.,0.],[0.,0.,1.]],1e-9).unwrap();
    square.check(1e-9).unwrap();
    assert_eq!(square.faces.len(),7);
    close(volume(&square),r*r*r-PI*r*r*r/6.);
    // a slanted corner: the ball's centre a radius from each face
    let m = [[1.,0.,0.],unit([0.4,1.,0.]),unit([0.2,-0.3,1.])];
    let p: V = [2.,-1.,0.5];
    let rows = m.map(|mi| (mi,dot(mi,p)+r));
    // Cramer's rule for (c · m_i) = (p · m_i) + r
    let det = |a: V,b: V,c: V| dot(a,cross(b,c));
    let d = det(rows[0].0,rows[1].0,rows[2].0);
    let (mx,my,mz) = ([m[0][0],m[1][0],m[2][0]],[m[0][1],m[1][1],m[2][1]],[m[0][2],m[1][2],m[2][2]]);
    let b = [rows[0].1,rows[1].1,rows[2].1];
    let c = [det(b,my,mz)/d,det(mx,b,mz)/d,det(mx,my,b)/d];
    let slanted = corner(p,c,r,m,1e-9).unwrap();
    slanted.check(1e-9).unwrap();
    let along = sub(c,p);
    let edge = |i: usize,j: usize| { let e = unit(cross(m[i],m[j])); if dot(e,along) < 0. { scale(e,-1.) } else { e } };
    let foot = |i: usize,j: usize| { let e = edge(i,j); add(p,scale(e,dot(along,e))) };
    let touch = |i: usize| sub(c,scale(m[i],r));
    let tet = |a: V,b: V,q: V| det(sub(a,c),sub(b,c),sub(q,c)).abs()/6.;
    let mut cell = 0.;
    for (i,j,k) in [(0,1,2),(1,2,0),(2,0,1)] {
        cell += tet(p,foot(i,j),touch(i))+tet(p,touch(i),foot(i,k));
    }
    let omega = 2.*(det(m[0],m[1],m[2]).abs()/(1.+dot(m[0],m[1])+dot(m[1],m[2])+dot(m[2],m[0]))).atan();
    close(volume(&slanted),cell-omega*r*r*r/3.);
}

/// **A strip across a tangent seam, glued into the body it lies on** (#422): a plate of 30 whose
/// top's rim is rounded 3 — the top going on tangent into a torus at 27 — and a sector of a strip
/// lying on both, from 25 out over the seam onto the torus to 30° round its tube, 1 tall over the
/// top and a radian round.  Their union's faces meet the plate's where the strip's do, across the
/// plate's tangent edge: a valid boundary, the plate's volume and the strip's.
#[test]
fn a_strip_across_a_tangent_seam_unions_into_its_body() {
    use gcs_core::brep::boolean::{combined,Op};
    let o = [0.;3];
    let (big,r) = (27.,3.);
    // in the plane of x and z: an arc's angles turn x toward −z, so the tube's top is −π/2
    let plate = Profile {names:vec![],origin:o,normal:XZ,loops:vec![vec![
        line([0.,0.,0.],[30.,0.,0.]),line([30.,0.,0.],[30.,0.,17.]),
        arc([big,0.,17.],r,XZ,[1.,0.,0.],Some([-PI/2.,0.])),line([big,0.,20.],[0.,0.,20.]),line([0.,0.,20.],[0.,0.,0.])]]};
    let plate = revolve(&plate,o,[0.,0.,1.],TAU).unwrap();
    plate.check(1e-9).unwrap();
    let a = PI/6.;
    let foot = [big+r*a.sin(),0.,17.+r*a.cos()];
    let strip = Profile {names:vec![],origin:o,normal:XZ,loops:vec![vec![
        line([25.,0.,20.],[big,0.,20.]),arc([big,0.,17.],r,XZ,[1.,0.,0.],Some([-PI/2.,-PI/2.+a])),
        line(foot,[foot[0],0.,21.]),line([foot[0],0.,21.],[25.,0.,21.]),line([25.,0.,21.],[25.,0.,20.])]]};
    let strip = revolve(&strip,o,[0.,0.,1.],1.).unwrap();
    strip.check(1e-9).unwrap();
    let union = combined(&plate,&strip,Op::Union,0.).unwrap();
    union.check(1e-7).unwrap();
    let want = volume(&plate)+volume(&strip);
    assert!((volume(&union)-want).abs() <= 1e-9*want,"{} against {want}",volume(&union));
}

/// **Touching is not open** (`Brep::check`): two blocks meeting along one edge leave it used twice
/// each way, material touching itself along a line (`Kind::NotManifold`, what the body's stratum
/// reads as rounds of separate statements meeting); a block missing a face leaves its edges used
/// once one way only, a boundary the kernel left open (`Kind::Open`), never a meeting.
#[test]
fn an_edge_used_twice_each_way_touches_and_one_used_once_is_open() {
    use gcs_core::brep::boolean::{boolean,Op};
    use gcs_core::brep::Kind;
    let block = |x: f64,y: f64| {
        let p = Profile {names:vec![],origin:[0.;3],normal:XY,
            loops:vec![poly(&[[x,y,0.],[x+1.,y,0.],[x+1.,y+1.,0.],[x,y+1.,0.]])]};
        prism(&p,0.,1.).unwrap()
    };
    let touching = boolean(&block(0.,0.),&block(1.,1.),Op::Union,1e-9).unwrap();
    let e = touching.check(1e-9).unwrap_err();
    assert_eq!(e.kind,Kind::NotManifold,"{e}");
    let mut open = block(0.,0.);
    open.faces.pop();
    let e = open.check(1e-9).unwrap_err();
    assert_eq!(e.kind,Kind::Open,"{e}");
    assert!(e.msg.starts_with("not closed: "),"{e}");
}
