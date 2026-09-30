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
    let p = Profile {origin:[0.;3],normal:XY,loops:vec![poly(&[[0.,0.,0.],[3.,0.,0.],[3.,2.,0.],[0.,2.,0.]])]};
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
    let p = Profile {origin:[0.;3],normal:XY,loops:vec![outer,hole,square]};
    let b = prism(&p,0.,2.).unwrap();
    b.check(1e-9).unwrap();
    close(volume(&b),2.*(w*2.*r+PI*r*r-PI*0.25-1.));
}

#[test]
fn a_circle_swept_is_a_cylinder_seamed_once() {
    let p = Profile {origin:[1.,2.,3.],normal:[1.,1.,1.],loops:vec![vec![arc([1.,2.,3.],2.,[1.,1.,1.],[1.,-1.,0.],None)]]};
    let b = prism(&p,0.,5.).unwrap();
    b.check(1e-9).unwrap();
    assert_eq!((b.faces.len(),b.edges.len(),b.vertices.len()),(3,3,2));
    close(volume(&b),PI*4.*5.);
}

#[test]
fn turned_whole_profiles_are_their_solids_of_revolution() {
    let turn = |loops: Vec<Vec<ProfileEdge>>,angle: f64| {
        let b = revolve(&Profile {origin:[0.;3],normal:XZ,loops},[0.;3],[0.,0.,1.],angle).unwrap();
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
            let b = revolve(&Profile {origin:[0.;3],normal:XZ,loops:loops.clone()},[0.;3],[0.,0.,1.],angle).unwrap();
            b.check(1e-9).unwrap();
            close(volume(&b),whole*angle.abs()/TAU);
        }
    }
    // a quarter of a ball and of a torus
    let ball = vec![arc([0.,0.,0.],r,XZ,[1.,0.,0.],Some([-PI/2.,PI/2.])),line([0.,0.,-r],[0.,0.,r])];
    let b = revolve(&Profile {origin:[0.;3],normal:XZ,loops:vec![ball]},[0.;3],[0.,0.,1.],PI/2.).unwrap();
    b.check(1e-9).unwrap();
    close(volume(&b),PI*r*r*r/3.);
    let torus = vec![vec![arc([3.,0.,1.],1.,XZ,[1.,0.,0.],None)]];
    let b = revolve(&Profile {origin:[0.;3],normal:XZ,loops:torus},[0.;3],[0.,0.,1.],PI/2.).unwrap();
    b.check(1e-9).unwrap();
    close(volume(&b),PI*PI*3./2.);
}

#[test]
fn a_moved_solid_is_the_same_solid() {
    let p = Profile {origin:[0.;3],normal:XZ,loops:vec![vec![arc([3.,0.,1.],1.,XZ,[1.,0.,0.],None)]]};
    let b = revolve(&p,[0.;3],[0.,0.,1.],TAU).unwrap();
    let m = Rigid::turn([1.,2.,3.],[1.,-2.,0.5],0.7).then(&Rigid {r:Rigid::identity().r,t:[5.,-1.,2.]});
    let moved = b.moved(&m);
    moved.check(1e-9).unwrap();
    close(volume(&moved),volume(&b));
}

#[test]
fn points_are_placed_in_on_or_out_of_each_primitive() {
    use gcs_core::brep::query::{Located,Place::*};
    let block = prism(&Profile {origin:[0.;3],normal:XY,loops:vec![poly(&[[0.,0.,0.],[3.,0.,0.],[3.,2.,0.],[0.,2.,0.]]),
        vec![arc([1.,1.,0.],0.5,XY,[1.,0.,0.],None)]]},0.,1.).unwrap();
    let torus = revolve(&Profile {origin:[0.;3],normal:XZ,loops:vec![vec![arc([3.,0.,1.],1.,XZ,[1.,0.,0.],None)]]},
        [0.;3],[0.,0.,1.],TAU).unwrap();
    let ball = revolve(&Profile {origin:[0.;3],normal:XZ,loops:vec![vec![arc([0.,0.,0.],2.,XZ,[1.,0.,0.],Some([-PI/2.,PI/2.])),
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
        (Surface::Plane(f([0.,0.,1.],z,x)),Surface::Sphere(f([0.,0.,0.],z,x),2.),1),
        (Surface::Plane(f([0.5,0.,1.],tilted,x)),Surface::Sphere(f([0.,0.,0.],z,x),2.),1),
        (Surface::Plane(f([0.,0.,0.5],z,x)),Surface::Torus(f([0.,0.,0.],z,x),3.,1.),2),
        (Surface::Cylinder(f([0.,0.,0.],z,x),2.),Surface::Sphere(f([0.,0.,1.],z,x),3.),2),
        (Surface::Cylinder(f([0.,0.,0.],z,x),2.),Surface::Cone(f([0.,0.,1.],scale3(z,-1.),x),1.,0.4),1),
        (Surface::Cone(f([0.,0.,0.],z,x),1.,0.4),Surface::Torus(f([0.,0.,2.],z,x),2.5,1.),2),
        (Surface::Torus(f([0.,0.,0.],z,x),3.,1.),Surface::Torus(f([0.,0.,1.],z,x),3.5,1.),2),
        (Surface::Sphere(f([0.,0.,0.],z,x),2.),Surface::Sphere(f([1.,1.,1.],z,x),2.5),1),
        (Surface::Cylinder(f([0.,0.,0.],z,x),2.),Surface::Cylinder(f([1.,1.,5.],z,y),1.5),2),
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
    // perpendicular cylinders have no closed form
    assert_eq!(intersect(&Surface::Cylinder(f([0.,0.,0.],z,x),2.),&Surface::Cylinder(f([0.,0.,0.],x,y),1.),1e-9),Ssi::Traced);
    // the same surface, written from another frame
    assert_eq!(intersect(&Surface::Cylinder(f([0.,0.,0.],z,x),2.),&Surface::Cylinder(f([0.,0.,7.],scale3(z,-1.),y),2.),1e-9),Ssi::Same);
    assert_eq!(intersect(&Surface::Plane(f([1.,2.,3.],z,x)),&Surface::Plane(f([5.,-2.,3.],scale3(z,-1.),y)),1e-9),Ssi::Same);
}

fn scale3(a: V,s: f64) -> V { [a[0]*s,a[1]*s,a[2]*s] }

fn block(lo: V,hi: V) -> gcs_core::brep::topo::Brep {
    let p = Profile {origin:[0.,0.,0.],normal:XY,loops:vec![poly(&[[lo[0],lo[1],0.],[hi[0],lo[1],0.],[hi[0],hi[1],0.],[lo[0],hi[1],0.]])]};
    prism(&p,lo[2],hi[2]).unwrap()
}
fn rod(c: V,r: f64,z: [f64;2]) -> gcs_core::brep::topo::Brep {
    prism(&Profile {origin:[0.,0.,0.],normal:XY,loops:vec![vec![arc([c[0],c[1],0.],r,XY,[1.,0.,0.],None)]]},z[0],z[1]).unwrap()
}
fn ball(c: V,r: f64) -> gcs_core::brep::topo::Brep {
    let p = Profile {origin:c,normal:XZ,loops:vec![vec![arc(c,r,XZ,[1.,0.,0.],Some([-PI/2.,PI/2.])),
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



/// `∫_a^b f` by Simpson's rule on `n` (even) steps.
fn simpson(a: f64,b: f64,n: usize,f: impl Fn(f64) -> f64) -> f64 {
    let h = (b-a)/n as f64;
    (0..=n).map(|i| f(a+h*i as f64)*if i == 0 || i == n { 1. } else if i%2 == 1 { 4. } else { 2. }).sum::<f64>()*h/3.
}

#[test]
fn traced_intersections_cross_bores_and_a_pierced_ball() {
    use gcs_core::brep::boolean::{boolean,Op};
    use gcs_core::brep::build::prism;
    // a horizontal rod along x, radius r, through (y, z) = (2, 2)
    let across = |r: f64| prism(&Profile {origin:[0.;3],normal:[1.,0.,0.],loops:vec![vec![arc([0.,2.,2.],r,[1.,0.,0.],[0.,1.,0.],None)]]},-1.,5.).unwrap();
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
    let across = prism(&Profile {origin:[0.;3],normal:[1.,0.,0.],loops:vec![vec![arc([0.,2.,2.],1.,[1.,0.,0.],[0.,1.,0.],None)]]},-1.,5.).unwrap();
    let a = block([0.,0.,0.],[4.,4.,4.]);
    let solids: Vec<(&str,gcs_core::brep::topo::Brep)> = vec![
        ("block",a.clone()),
        ("plate with four holes",{
            let mut loops = vec![poly(&[[-33.,-21.,0.],[33.,-21.,0.],[33.,21.,0.],[-33.,21.,0.]])];
            for c in [[-24.,-14.],[28.,-14.],[28.,14.],[-24.,14.]] { loops.push(vec![arc([c[0],c[1],0.],3.,XY,[1.,0.,0.],None)]); }
            prism(&Profile {origin:[0.;3],normal:XY,loops},0.,5.).unwrap()
        }),
        ("rod",rod([0.,0.,0.],2.,[0.,3.])),
        ("ball",ball([1.,2.,3.],2.)),
        ("torus",revolve(&Profile {origin:[0.;3],normal:XZ,loops:vec![vec![arc([3.,0.,1.],1.,XZ,[1.,0.,0.],None)]]},[0.;3],[0.,0.,1.],TAU).unwrap()),
        ("quarter ball",revolve(&Profile {origin:[0.;3],normal:XZ,loops:vec![vec![arc([0.,0.,0.],2.,XZ,[1.,0.,0.],Some([-PI/2.,PI/2.])),
            line([0.,0.,-2.],[0.,0.,2.])]]},[0.;3],[0.,0.,1.],PI/2.).unwrap()),
        ("cone",revolve(&Profile {origin:[0.;3],normal:XZ,loops:vec![poly(&[[0.,0.,0.],[2.,0.,0.],[0.,0.,3.]])]},[0.;3],[0.,0.,1.],TAU).unwrap()),
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
            let (v,w) = (mesh_volume(&m),volume(&s));
            // a chord's sag loses at most its area times itself: well within the bar over the size
            assert!((v-w).abs() <= 4.*bar*w.abs().powf(2./3.)*6.,"{what} at {bar}: {v} against {w}");
            eprintln!("{what} at {bar}: {} triangles, sag {:.2e}, volume {v:.6} against {w:.6}",m.tris.len(),m.sag);
        }
    }
}


#[test]
fn lofts_and_guided_sweeps_are_their_closed_forms() {
    use gcs_core::brep::build::{loft,Guide};
    let square = |c: V,half: f64,z: f64| poly(&[[c[0]-half,c[1]-half,z],[c[0]+half,c[1]-half,z],[c[0]+half,c[1]+half,z],[c[0]-half,c[1]+half,z]]);
    let section = |z: f64,outer: f64,inner: f64| Profile {origin:[0.,0.,z],normal:XY,loops:vec![square([0.;3],outer,z),square([0.;3],inner,z)]};
    let (h,a,b0,w) = (40.,12.,6.,2.);
    // a hollow square frustum: each side a plane; the prismoidal formula, outer less inner
    let frustum = |a: f64,b: f64| h/3.*(4.*a*a+4.*b*b+4.*a*b);
    let s = loft(&section(0.,a,a-w),Some(&section(h,b0,b0-w)),&Guide::Line {delta:[0.,0.,h]}).unwrap();
    s.check(1e-8).unwrap();
    close(volume(&s),frustum(a,b0)-frustum(a-w,b0-w));
    // a round reducer: a cone frustum with a hole
    let ring = |z: f64,r: f64,hole: f64| Profile {origin:[0.,0.,z],normal:XY,loops:vec![
        vec![arc([0.,0.,z],r,XY,[1.,0.,0.],None)],vec![arc([0.,0.,z],hole,XY,[0.,1.,0.],None)]]};
    let s = loft(&ring(0.,10.,4.),Some(&ring(h,6.,3.)),&Guide::Line {delta:[0.,0.,h]}).unwrap();
    s.check(1e-8).unwrap();
    let cone = |r: f64,q: f64| PI*h/3.*(r*r+r*q+q*q);
    close(volume(&s),cone(10.,6.)-cone(4.,3.));
    // a square elbow: its section turned a quarter about the bend's axis (Pappus)
    let bend = 30.;
    let elbow = Profile {origin:[bend,0.,0.],normal:XZ,loops:vec![
        poly(&[[bend-9.,0.,-9.],[bend+9.,0.,-9.],[bend+9.,0.,9.],[bend-9.,0.,9.]]),
        poly(&[[bend-7.,0.,-7.],[bend+7.,0.,-7.],[bend+7.,0.,7.],[bend-7.,0.,7.]])]};
    let s = loft(&elbow,None,&Guide::Arc {center:[0.;3],axis:[0.,0.,1.],angle:PI/2.}).unwrap();
    s.check(1e-8).unwrap();
    close(volume(&s),(18.*18.-14.*14.)*bend*PI/2.);
    // a twisted loft, a square to a diamond: each section the polygon of its corners' mixture, of
    // an area quadratic in the height, so Simpson's rule is exact
    let shoelace = |p: &[[f64;2]]| (0..p.len()).map(|i| { let (a,b) = (p[i],p[(i+1)%p.len()]); a[0]*b[1]-a[1]*b[0] }).sum::<f64>()/2.;
    let (sq,dia) = ([[-6.,-6.],[6.,-6.],[6.,6.],[-6.,6.]],[[0.,-8.],[8.,0.],[0.,8.],[-8.,0.]]);
    let mix = |t: f64| (0..4).map(|k| [sq[k][0]*(1.-t)+dia[k][0]*t,sq[k][1]*(1.-t)+dia[k][1]*t]).collect::<Vec<_>>();
    let turned = Profile {origin:[0.,0.,h],normal:XY,loops:vec![poly(&[[0.,-8.,h],[8.,0.,h],[0.,8.,h],[-8.,0.,h]])]};
    let s = loft(&Profile {loops:vec![square([0.;3],6.,0.)],..section(0.,a,a-w)},Some(&turned),&Guide::Line {delta:[0.,0.,h]}).unwrap();
    s.check(1e-8).unwrap();
    assert!(s.faces.iter().any(|f| f.surface.kind() == "blend"));
    close(volume(&s),h/6.*(shoelace(&mix(0.))+4.*shoelace(&mix(0.5))+shoelace(&mix(1.))));
    // an elbow narrowing as it bends: the section blended while it turns, so the volume is the
    // turn times the integral of the section's first moment about the axis, a cubic in the blend
    let moment = |p: &[[f64;2]]| (0..p.len()).map(|i| { let (a,b) = (p[i],p[(i+1)%p.len()]);
        (a[0]+b[0])*(a[0]*b[1]-b[0]*a[1]) }).sum::<f64>()/6.;
    let ring2 = |half: f64| [[bend-half,-half],[bend+half,-half],[bend+half,half],[bend-half,half]];
    let blend2 = |o: f64,i: f64,t: f64| {
        let m = |p: [[f64;2];4],q: [[f64;2];4]| (0..4).map(|k| [p[k][0]*(1.-t)+q[k][0]*t,p[k][1]*(1.-t)+q[k][1]*t]).collect::<Vec<_>>();
        moment(&m(ring2(9.),ring2(o)))-moment(&m(ring2(7.),ring2(i)))
    };
    let narrow = Profile {origin:[0.,bend,0.],normal:[1.,0.,0.],loops:vec![
        poly(&ring2(6.).map(|q| [0.,q[0],q[1]])),poly(&ring2(4.).map(|q| [0.,q[0],q[1]]))]};
    let s = loft(&elbow,Some(&narrow),&Guide::Arc {center:[0.;3],axis:[0.,0.,1.],angle:PI/2.}).unwrap();
    s.check(1e-8).unwrap();
    let want = PI/2./6.*(blend2(6.,4.,0.)+4.*blend2(6.,4.,0.5)+blend2(6.,4.,1.));
    assert!((volume(&s)-want).abs() <= 1e-9*want,"{} against {want}",volume(&s));
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
    let plate = Profile {origin:[0.;3],normal:XY,loops:vec![vec![line([0.,0.,0.],[10.,0.,0.]),spline(&lobe),line([0.,10.,0.],[0.,0.,0.])]]};
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
    let vase = Profile {origin:[0.;3],normal:XZ,loops:vec![vec![line([0.,0.,0.],[12.,0.,0.]),spline(&wall),
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
    // lofted to itself at half the size along z: every section the plate scaled by 1 − t/2, so the
    // volume is h A ∫(1 − t/2)² dt = 7 h A / 12; the lines join in planes, the splines in a blend
    let half = cubic(&lobe.poles.iter().map(|p| [p[0]/2.,p[1]/2.,6.]).collect::<Vec<_>>(),&[0.4]);
    let top = Profile {origin:[0.,0.,6.],normal:XY,loops:vec![vec![line([0.,0.,6.],[5.,0.,6.]),spline(&half),line([0.,5.,6.],[0.,0.,6.])]]};
    let lofted = gcs_core::brep::build::loft(&plate,Some(&top),&gcs_core::brep::build::Guide::Line {delta:[0.,0.,6.]}).unwrap();
    lofted.check(1e-9).unwrap();
    assert!(lofted.faces.iter().any(|f| f.surface.kind() == "blend") && lofted.faces.iter().filter(|f| f.surface.kind() == "plane").count() == 4);
    assert!((volume(&lofted)-7.*6.*area/12.).abs() <= 1e-9*area,"{} against {}",volume(&lofted),7.*6.*area/12.);
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
    for (what,s) in [("plate",&b),("vase",&v),("lofted",&lofted)] {
        let m = mesh(s,0.01,0.2).unwrap_or_else(|e| panic!("{what}: {e}"));
        assert!(m.sag <= 0.01,"{what}: sags {}",m.sag);
        gcs_core::mesh::stl_shells(&gcs_core::mesh::stl_of(&m.triangles(),what)).unwrap_or_else(|e| panic!("{what}: {e}"));
        assert!((mesh_volume(&m)-volume(s)).abs() <= 2e-3*volume(s),"{what}: {} against {}",mesh_volume(&m),volume(s));
    }
}
