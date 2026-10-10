//! Rational B-splines in the B-rep kernel (`brep::nurbs`), held to closed forms: a weighted
//! quadratic is a circle exactly, a tensor of two a sphere or a torus, their derivatives what their
//! points' differences say, a net cut or a curve's knot put in the same surface; and solids bounded
//! by them — swept and turned from an exact arc, or read with a rational side — are their volumes,
//! meshed closed, and written to STEP as the file the checker reads back.
use gcs_core::brep::build::{prism,revolve,sheet,Profile,ProfileEdge};
use gcs_core::brep::geom::{Frame,V};
use gcs_core::brep::nurbs::{arc,BSpline,Net};
use gcs_core::brep::props::volume;
use gcs_core::brep::topo::Brep;
use std::f64::consts::{FRAC_PI_2,PI,TAU};
use std::sync::Arc;

fn dist(a: V,b: V) -> f64 { ((a[0]-b[0]).powi(2)+(a[1]-b[1]).powi(2)+(a[2]-b[2]).powi(2)).sqrt() }
fn close(a: f64,b: f64,tol: f64) { assert!((a-b).abs() <= tol*b.abs().max(1.),"{a} against {b} ({:e})",(a-b)/b.abs().max(1.)); }
/// `a` within `tol` of `b`, relative to `scale`.
fn near(a: V,b: V,tol: f64,what: &str) { assert!(dist(a,b) <= tol,"{what}: {a:?} against {b:?} ({:e})",dist(a,b)); }

const XY: V = [0.,0.,1.];
const XZ: V = [0.,1.,0.];

/// The unit circle in the plane of x and y, a turn from angle 0.
fn unit_circle() -> BSpline { arc(&Frame::new([0.;3],XY,[1.,0.,0.]),1.,[0.,TAU]) }

#[test]
fn an_arc_is_its_circle_exactly() {
    let f = Frame::new([1.,-2.,3.],[1.,1.,1.],[1.,-1.,0.]);
    let r = 2.5;
    for span in [[0.,TAU],[0.3,1.2],[-1.,2.5],[0.,FRAC_PI_2],[0.,PI]] {
        let c = arc(&f,r,span);
        assert!(c.is_rational() && c.degree == 2);
        assert_eq!(c.domain(),span);
        // every point on the circle, and at each knot the point at that angle
        let [a,z] = span;
        for k in 0..=200 {
            let p = c.point(a+(z-a)*k as f64/200.);
            close(dist(p,f.o),r,2e-15);
            let l = f.local(p);
            assert!(l[2].abs() <= 1e-14,"off its plane by {}",l[2]);
        }
        for &t in &c.knots {
            near(c.point(t),[0,1,2].map(|k| f.o[k]+r*(t.cos()*f.x[k]+t.sin()*f.y[k])),1e-14,"a knot");
        }
        // its derivatives are its points' differences: the tangent along the circle, C'' too
        let h = 1e-5;
        for k in 1..40 {
            let t = a+(z-a)*k as f64/40.;
            if c.knots.iter().any(|&kn| (kn-t).abs() < 2.*h) { continue }
            let (p,d,dd) = c.d2(t);
            let (p0,p1) = (c.point(t-h),c.point(t+h));
            near(d,[0,1,2].map(|k| (p1[k]-p0[k])/(2.*h)),1e-8,"C'");
            let (d0,d1) = (c.d2(t-h).1,c.d2(t+h).1);
            near(dd,[0,1,2].map(|k| (d1[k]-d0[k])/(2.*h)),1e-7,"C''");
            // square to the radius, which a circle's tangent is
            let q = [0,1,2].map(|k| p[k]-f.o[k]);
            assert!((q[0]*d[0]+q[1]*d[1]+q[2]*d[2]).abs() <= 1e-12*r*(d[0]*d[0]+d[1]*d[1]+d[2]*d[2]).sqrt());
        }
        // within its poles' hull: each span's box holds its points
        let (lo,hi) = gcs_core::brep::geom::Curve::BSpline(Arc::new(c.clone())).bounds(span);
        for k in 0..=50 {
            let p = c.point(a+(z-a)*k as f64/50.);
            assert!((0..3).all(|i| p[i] >= lo[i]-1e-12 && p[i] <= hi[i]+1e-12));
        }
    }
}

#[test]
fn weights_of_one_are_the_polynomial_spline() {
    let knots = vec![0.,0.,0.,0.,0.4,1.,1.,1.,1.];
    let poles: Vec<V> = (0..5).map(|i| { let x = i as f64; [x,(x*1.3).sin(),0.2*x*x] }).collect();
    let plain = BSpline::new(3,knots.clone(),poles.clone()).unwrap();
    let ones = BSpline::rational(3,knots,poles,vec![1.;5]).unwrap();
    for k in 0..=20 {
        let t = k as f64/20.;
        let (a,b) = (plain.d2(t),ones.d2(t));
        for (p,q) in [(a.0,b.0),(a.1,b.1),(a.2,b.2)] { near(p,q,1e-13,"weights of one"); }
    }
}

#[test]
fn weights_that_are_not_finite_and_positive_are_refused() {
    let c = unit_circle();
    for bad in [vec![1.;8],vec![1.,0.,1.,1.,1.,1.,1.,1.,1.],vec![1.,-0.5,1.,1.,1.,1.,1.,1.,1.],vec![1.,f64::NAN,1.,1.,1.,1.,1.,1.,1.],
        vec![1.,f64::INFINITY,1.,1.,1.,1.,1.,1.,1.]] {
        assert!(BSpline::rational(2,c.knots.clone(),c.poles.clone(),bad.clone()).is_err(),"{bad:?}");
    }
    let mut net = sphere(1.);
    assert!(net.check().is_ok());
    net.weights.as_mut().unwrap()[2][1] = 0.;
    assert!(net.check().is_err());
    net.weights.as_mut().unwrap()[2].pop();
    assert!(net.check().is_err());
}

/// The surface of revolution of the meridian `m` (its poles `(ρ, 0, z)`) about z, a whole turn: the
/// tensor of the unit circle and `m`, each pole the circle's scaled by the meridian's `ρ` and lifted
/// to its `z`, weighed by both (Piegl and Tiller A8.1).
fn turned(m: &BSpline) -> Net {
    let c = unit_circle();
    let (cw,mw) = (c.weights.as_ref().unwrap(),m.weights.as_ref().unwrap());
    let poles = c.poles.iter().map(|q| m.poles.iter().map(|p| [q[0]*p[0],q[1]*p[0],p[2]]).collect()).collect();
    let weights = cw.iter().map(|&a| mw.iter().map(|&b| a*b).collect()).collect();
    Net {du:2,dv:2,uknots:c.knots.clone(),vknots:m.knots.clone(),poles,weights:Some(weights)}
}
/// The sphere of radius `r` about the origin: its meridian a half-circle from the south pole up.
fn sphere(r: f64) -> Net { turned(&arc(&Frame::new([0.;3],[0.,-1.,0.],[1.,0.,0.]),r,[-FRAC_PI_2,FRAC_PI_2])) }
/// The torus about z through the origin, its tube of radius `r` about the circle of radius `big`.
fn torus(big: f64,r: f64) -> Net { turned(&arc(&Frame::new([big,0.,0.],[0.,-1.,0.],[1.,0.,0.]),r,[0.,TAU])) }

/// Each derivative of a net against its points' (or its first derivatives') differences.
fn derivatives_are_differences(n: &Net,what: &str) {
    let [[u0,u1],[v0,v1]] = n.domain();
    let h = 1e-5;
    let interior = |t: f64,k: &[f64]| k.iter().all(|&x| (x-t).abs() > 2.*h);
    for i in 1..12 { for j in 1..12 {
        let (u,v) = (u0+(u1-u0)*i as f64/12.+1e-3,v0+(v1-v0)*j as f64/12.+1e-3);
        if !interior(u,&n.uknots) || !interior(v,&n.vknots) { continue }
        let [s,su,sv,suu,suv,svv] = n.d2(u,v);
        let (s1,su1,sv1) = n.d1(u,v);
        near(s,s1,1e-14,what); near(su,su1,1e-12,what); near(sv,sv1,1e-12,what);
        let diff = |f: &dyn Fn(f64,f64) -> V,du: f64,dv: f64| -> V {
            let (a,b) = (f(u-du,v-dv),f(u+du,v+dv));
            [0,1,2].map(|k| (b[k]-a[k])/(2.*h))
        };
        near(su,diff(&|u,v| n.point(u,v),h,0.),1e-8,what);
        near(sv,diff(&|u,v| n.point(u,v),0.,h),1e-8,what);
        near(suu,diff(&|u,v| n.d1(u,v).1,h,0.),1e-7,what);
        near(suv,diff(&|u,v| n.d1(u,v).1,0.,h),1e-7,what);
        near(suv,diff(&|u,v| n.d1(u,v).2,h,0.),1e-7,what);
        near(svv,diff(&|u,v| n.d1(u,v).2,0.,h),1e-7,what);
    } }
}

#[test]
fn tensors_of_arcs_are_spheres_and_tori() {
    let (r,big,tube) = (3.,4.,1.25);
    let (s,t) = (sphere(r),torus(big,tube));
    for k in 0..=40 { for j in 0..=40 {
        let (u,v) = (TAU*k as f64/40.,-FRAC_PI_2+PI*j as f64/40.);
        close(dist(s.point(u,v),[0.;3]),r,2e-15);
        let p = t.point(u,TAU*j as f64/40.);
        close(((p[0].hypot(p[1])-big).powi(2)+p[2]*p[2]).sqrt(),tube,1e-14);
    } }
    derivatives_are_differences(&s,"the sphere");
    derivatives_are_differences(&t,"the torus");
}

#[test]
fn a_rational_net_cut_is_the_same_surface() {
    let t = torus(4.,1.25);
    for (bu,bv) in [([0.4,2.],[0.3,4.]),([0.,FRAC_PI_2],[PI,TAU]),([1.,1.3],[2.,2.2])] {
        let cut = t.segment(bu,bv);
        assert!(cut.is_rational());
        assert!(cut.check().is_ok());
        assert_eq!(cut.domain(),[bu,bv]);
        for i in 0..=6 { for j in 0..=6 {
            let (u,v) = (bu[0]+(bu[1]-bu[0])*i as f64/6.,bv[0]+(bv[1]-bv[0])*j as f64/6.);
            let (a,b) = (t.d1(u,v),cut.d1(u,v));
            for (p,q) in [(a.0,b.0),(a.1,b.1),(a.2,b.2)] { near(p,q,1e-11,"a cut torus"); }
        } }
        // its sheet's edges lie on it, weighed as it is
        let b = sheet(cut).unwrap();
        let f = &b.faces[0];
        for c in &f.loops[0] {
            let e = &b.edges[c.edge as usize];
            for k in 0..=10 {
                let x = e.t[0]+(e.t[1]-e.t[0])*k as f64/10.;
                near(f.surface.point(c.pcurve.at(x,e,&f.surface,&b.vertices)),e.point(x,&b.vertices),1e-12,"a sheet's edge");
            }
        }
    }
}

fn mesh_volume(m: &gcs_core::brep::mesh::Mesh) -> f64 {
    m.tris.iter().map(|t| {
        let [a,b,c] = t.map(|i| m.pts[i as usize]);
        (a[0]*(b[1]*c[2]-b[2]*c[1])-a[1]*(b[0]*c[2]-b[2]*c[0])+a[2]*(b[0]*c[1]-b[1]*c[0]))/6.
    }).sum()
}

/// A solid with rational faces or edges: valid, its volume `whole`, meshed closed within its bar,
/// and written as the STEP file the checker reads back face for face.
fn holds(what: &str,b: &Brep,whole: f64) {
    b.check(1e-9).unwrap_or_else(|e| panic!("{what}: {e}"));
    close(volume(b),whole,1e-11);
    for bar in [0.01,0.001] {
        let m = gcs_core::brep::mesh::mesh(b,bar,0.2).unwrap_or_else(|e| panic!("{what}: {e}"));
        gcs_core::mesh::stl_shells(&gcs_core::mesh::stl_of(&m.triangles(),what)).unwrap_or_else(|e| panic!("{what} at {bar}: {e}"));
        assert!(m.sag <= bar,"{what}: sags {} against {bar}",m.sag);
        assert_eq!(m.turned,0,"{what}: triangles facing against their surfaces");
        let v = mesh_volume(&m);
        assert!((v-whole).abs() <= 24.*bar*whole.abs().powf(2./3.),"{what} at {bar}: {v} against {whole}");
    }
    let text = gcs_core::brep::step::write(b,what,1e-4).unwrap_or_else(|e| panic!("{what}: {e}"));
    assert!(text.contains("RATIONAL_B_SPLINE"),"{what}: nothing rational written");
    gcs_core::brep::step_check::verify(&text,&gcs_core::brep::step_check::Solid::of(b)).unwrap_or_else(|e| panic!("{what}: {e}"));
}

#[test]
fn solids_swept_and_turned_from_exact_arcs_are_their_closed_forms() {
    let spline = |c: BSpline| ProfileEdge::Spline(Arc::new(c));
    // a cylinder: a whole circle swept
    let (r,h) = (2.,5.);
    let circle = arc(&Frame::new([1.,2.,0.],XY,[1.,0.,0.]),r,[0.,TAU]);
    let rod = prism(&Profile {names:vec![],origin:[1.,2.,0.],normal:XY,loops:vec![vec![spline(circle)]]},0.,h).unwrap();
    holds("rod",&rod,PI*r*r*h);
    // a ball: a half-circle and its diameter turned
    let half = arc(&Frame::new([0.;3],XZ,[1.,0.,0.]),r,[-FRAC_PI_2,FRAC_PI_2]);
    let ball = revolve(&Profile {names:vec![],origin:[0.;3],normal:XZ,loops:vec![vec![spline(half),
        ProfileEdge::Line {a:[0.,0.,-r],b:[0.,0.,r]}]]},[0.;3],[0.,0.,1.],TAU).unwrap();
    holds("ball",&ball,4./3.*PI*r*r*r);
    // a torus: a whole circle turned, Pappus's
    let tube = arc(&Frame::new([3.,0.,1.],XZ,[1.,0.,0.]),1.,[0.,TAU]);
    let ring = revolve(&Profile {names:vec![],origin:[0.;3],normal:XZ,loops:vec![vec![spline(tube)]]},[0.;3],[0.,0.,1.],TAU).unwrap();
    holds("ring",&ring,TAU*3.*PI);
}

/// A cylinder as another kernel hands one over: its side a degree 2 × 1 rational net, its rims that
/// net's circles, its caps planes whose pcurves are those circles in the plane's coordinates.
fn cylinder_json(r: f64,h: f64) -> String {
    let c = arc(&Frame::new([0.;3],XY,[1.,0.,0.]),r,[0.,TAU]);
    let w = c.weights.as_ref().unwrap();
    let list = |x: &[f64]| format!("[{}]",x.iter().map(|v| format!("{v:?}")).collect::<Vec<_>>().join(","));
    let v3 = |p: V| list(&p);
    let rim = |z: f64| format!("{{\"kind\":\"bspline\",\"degree\":2,\"knots\":{},\"poles\":[{}],\"weights\":{}}}",
        list(&c.knots),c.poles.iter().map(|p| v3([p[0],p[1],z])).collect::<Vec<_>>().join(","),list(w));
    let side = format!("{{\"kind\":\"bspline\",\"du\":2,\"dv\":1,\"uknots\":{},\"vknots\":[0,0,{h:?},{h:?}],\"poles\":[{}],\"weights\":[{}]}}",
        list(&c.knots),c.poles.iter().map(|p| format!("[{},{}]",v3([p[0],p[1],0.]),v3([p[0],p[1],h]))).collect::<Vec<_>>().join(","),
        w.iter().map(|&x| list(&[x,x])).collect::<Vec<_>>().join(","));
    let line = |p: [f64;2],d: [f64;2]| format!("{{\"kind\":\"line\",\"p\":[{:?},{:?},0],\"d\":[{:?},{:?},0]}}",p[0],p[1],d[0],d[1]);
    let plane = |z: f64| format!("{{\"kind\":\"plane\",\"frame\":{{\"o\":[0,0,{z:?}],\"x\":[1,0,0],\"y\":[0,1,0],\"z\":[0,0,1]}}}}");
    let used = |e: usize,reversed: bool,pcurve: String| format!("{{\"edge\":{e},\"reversed\":{reversed},\"pcurve\":{pcurve}}}");
    format!("{{\"vertices\":[{{\"p\":{},\"tol\":0}},{{\"p\":{},\"tol\":0}}],\"edges\":[\
        {{\"v\":[0,0],\"t\":[0,{TAU:?}],\"tol\":0,\"curve\":{}}},\
        {{\"v\":[1,1],\"t\":[0,{TAU:?}],\"tol\":0,\"curve\":{}}},\
        {{\"v\":[0,1],\"t\":[0,{h:?}],\"tol\":0,\"curve\":{{\"kind\":\"line\",\"p\":{},\"d\":[0,0,1]}}}}],\"faces\":[\
        {{\"reversed\":false,\"surface\":{side},\"loops\":[{{\"outer\":true,\"uses\":[{},{},{},{}]}}]}},\
        {{\"reversed\":true,\"surface\":{},\"loops\":[{{\"outer\":true,\"uses\":[{}]}}]}},\
        {{\"reversed\":false,\"surface\":{},\"loops\":[{{\"outer\":true,\"uses\":[{}]}}]}}]}}",
        v3([r,0.,0.]),v3([r,0.,h]),rim(0.),rim(h),v3([r,0.,0.]),
        used(0,false,line([0.,0.],[1.,0.])),used(2,false,line([TAU,0.],[0.,1.])),
        used(1,true,line([0.,h],[1.,0.])),used(2,true,line([0.,0.],[0.,1.])),
        plane(0.),used(0,false,rim(0.)),plane(h),used(1,false,rim(0.)))
}

#[test]
fn a_cylinder_read_with_a_rational_side_is_its_closed_form() {
    let (r,h) = (1.5,4.);
    let b = gcs_core::brep::json::read(&cylinder_json(r,h)).unwrap();
    assert!(b.faces.iter().any(|f| matches!(&f.surface,gcs_core::brep::geom::Surface::BSpline(_,n) if n.is_rational())));
    assert!(b.edges.iter().all(|e| e.tol < 1e-12),"{:?}",b.edges.iter().map(|e| e.tol).collect::<Vec<_>>());
    holds("cylinder",&b,PI*r*r*h);
    // a weight of zero is refused where it is read
    let bad = cylinder_json(r,h).replacen("\"weights\":[1.0,","\"weights\":[0.0,",1);
    assert!(bad != cylinder_json(r,h));
    assert!(gcs_core::brep::json::read(&bad).is_err());
}

#[test]
fn a_plane_square_to_a_rational_sweep_meets_it_in_its_weighed_curve() {
    use gcs_core::brep::geom::{Curve,Surface};
    use gcs_core::brep::ssi::{intersect,Ssi};
    // a cylinder swept from an exact circle, cut by a plane square to the sweep: the circle itself,
    // weights and all (without them the polynomial through the same poles bulges off it)
    let r = 2.;
    let circle = arc(&Frame::new([1.,2.,0.],XY,[1.,0.,0.]),r,[0.,TAU]);
    let side = Surface::Extrusion(Frame::new([0.;3],XY,[1.,0.,0.]),Arc::new(Curve::BSpline(Arc::new(circle))));
    let cap = Surface::Plane(Frame::new([0.,0.,1.5],XY,[1.,0.,0.]));
    for (a,b) in [(&side,&cap),(&cap,&side)] {
        let Ssi::Curves(cs) = intersect(a,b,1e-9) else { panic!("no closed form") };
        assert_eq!(cs.len(),1);
        for k in 0..=64 {
            let p = cs[0].point(TAU*k as f64/64.);
            close(dist(p,[1.,2.,1.5]),r,1e-14);
            assert!(side.implicit(p).abs() <= 1e-12 && cap.implicit(p).abs() <= 1e-12,"{p:?} off the surfaces");
        }
    }
    // and a half rod of it cut through by a block's face square to its axis keeps the half below
    // (with the polynomial curve for the cut, an edge stood 0.07 off the side it bounds)
    use gcs_core::brep::boolean::{boolean,Op};
    let h = 5.;
    let line = |a: V,b: V| ProfileEdge::Line {a,b};
    let half = arc(&Frame::new([1.,2.,0.],XY,[1.,0.,0.]),r,[0.,PI]);
    let rod = prism(&Profile {names:vec![],origin:[1.,2.,0.],normal:XY,
        loops:vec![vec![ProfileEdge::Spline(Arc::new(half)),line([-1.,2.,0.],[3.,2.,0.])]]},0.,h).unwrap();
    let square = vec![line([-5.,-5.,0.],[5.,-5.,0.]),line([5.,-5.,0.],[5.,5.,0.]),line([5.,5.,0.],[-5.,5.,0.]),line([-5.,5.,0.],[-5.,-5.,0.])];
    let block = prism(&Profile {names:vec![],origin:[0.;3],normal:XY,loops:vec![square]},h/2.,2.*h).unwrap();
    let kept = boolean(&rod,&block,Op::Cut,1e-9).unwrap_or_else(|e| panic!("{e}"));
    kept.check(1e-8).unwrap_or_else(|e| panic!("{e}"));
    close(volume(&kept),PI*r*r*h/4.,1e-11);
}

