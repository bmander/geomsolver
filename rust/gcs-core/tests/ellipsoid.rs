//! The ellipsoid as a surface of the kernel's own (`geom::Surface::Ellipsoid`): a quadric with the
//! sphere's parameters, its foot found exactly (`geom::ellipsoid_foot`), a plane's meeting with it
//! an ellipse in closed form, its solid one face (`build::ellipsoid`), and written to STEP as its
//! exact rational net (`nurbs::ellipsoid_net`) — brep-bench's geometry/ellipsoid and
//! geometry/ellipsoid-half, both with semi-axes 6, 4 and 3, held to their closed forms.
use gcs_core::brep::boolean::{combined,Op};
use gcs_core::brep::build::{ellipsoid,prism,square,ProfileEdge,Profile};
use gcs_core::brep::geom::{ellipsoid_foot,Curve,Frame,Surface,V};
use gcs_core::brep::props::{area,volume};
use gcs_core::brep::topo::Brep;
use gcs_core::rng::Rng;
use std::f64::consts::{FRAC_PI_2,PI,TAU};

fn close(a: f64,b: f64,tol: f64) { assert!((a-b).abs() <= tol*b.abs().max(1.),"{a} against {b} ({:e})",(a-b)/b.abs().max(1.)); }
fn dist(a: V,b: V) -> f64 { ((a[0]-b[0]).powi(2)+(a[1]-b[1]).powi(2)+(a[2]-b[2]).powi(2)).sqrt() }
fn whole_area(b: &Brep) -> f64 { b.faces.iter().map(|f| area(b,f)).sum() }

/// brep-bench's: `x²/36 + y²/16 + z²/9 ≤ 1`.
const SEMI: [f64;3] = [6.,4.,3.];
/// Its area by Legendre's closed form (the case's own number).
const AREA: f64 = 231.24821501597344;

/// The solid is one face on its ellipsoid with the sphere's topology, valid, and its volume and
/// area the closed forms; placed anywhere, turned any way, and with its seam about either of the
/// half ellipse's axes, it stays so, its box the ellipsoid's own.
#[test]
fn an_ellipsoid_is_its_closed_form() {
    let b = ellipsoid([0.;3],[0.,0.,1.],[1.,0.,0.],SEMI).unwrap();
    b.check(1e-9).unwrap();
    assert_eq!((b.faces.len(),b.edges.len(),b.vertices.len()),(1,3,2));
    assert!(matches!(b.faces[0].surface,Surface::Ellipsoid(_,e) if e == SEMI));
    close(volume(&b),96.*PI,1e-12);
    close(whole_area(&b),AREA,1e-10);
    let (lo,hi) = b.bounds();
    for k in 0..3 { close(lo[k],-SEMI[k],1e-12); close(hi[k],SEMI[k],1e-12); }
    // its seam about its longer axis whichever that is, placed and turned off the world's axes
    let placed = [([2.,3.,5.],[1.,-2.,3.],[0.3,-0.4,0.8],[1.,1.,0.]),([5.,1.,2.],[-4.,0.,1.],[0.,1.,0.],[0.,0.,1.])];
    for (semi,o,z,x) in placed {
        let b = ellipsoid(o,z,x,semi).unwrap();
        b.check(1e-9).unwrap();
        close(volume(&b),4./3.*PI*semi[0]*semi[1]*semi[2],1e-12);
        // its box: about each coordinate the support `√Σ (eᵢ aᵢₖ)²` along the frame's axes
        let f = Frame::new(o,z,x);
        let (lo,hi) = b.bounds();
        for k in 0..3 {
            let half = ((semi[0]*f.x[k]).powi(2)+(semi[1]*f.y[k]).powi(2)+(semi[2]*f.z[k]).powi(2)).sqrt();
            close(lo[k],o[k]-half,1e-9);
            close(hi[k],o[k]+half,1e-9);
        }
    }
    // a sphere's semi-axes make the sphere's area
    close(whole_area(&ellipsoid([0.;3],[0.,0.,1.],[1.,0.,0.],[2.;3]).unwrap()),16.*PI,1e-10);
    // what is no ellipsoid is refused
    assert!(ellipsoid([0.;3],[0.,0.,1.],[1.,0.,0.],[1.,0.,1.]).is_err());
    assert!(ellipsoid([0.;3],[0.,0.,1.],[1.,0.,0.],[1.,f64::NAN,1.]).is_err());
    assert!(ellipsoid([0.;3],[0.;3],[1.,0.,0.],[1.;3]).is_err());
    assert!(ellipsoid([0.;3],[0.,0.,1.],[0.,0.,2.],[1.;3]).is_err());
}

/// The foot of a point is on the ellipsoid, the point on its normal there, and no point of the
/// ellipsoid is nearer: inside and out, on its planes of symmetry and its axes, at its centre.
#[test]
fn an_ellipsoids_foot_is_its_nearest_point() {
    let mut rng = Rng::new(7);
    let e = SEMI;
    let mut points: Vec<V> = (0..300).map(|_| std::array::from_fn(|k| rng.uniform(-1.6,1.6)*e[k])).collect();
    points.extend([[0.;3],[0.,0.,1.],[2.,0.,0.],[0.,3.9,0.],[5.,1.,0.],[1.,0.,2.],[0.,1.,-1.],[9.,0.,0.],
        [0.5,0.2,0.]]);
    // the ellipsoid sampled finely, to find what is nearer than the foot by search
    let n = 400;
    let grid: Vec<V> = (0..=n).flat_map(|i| (0..2*n).map(move |j| {
        let (u,v) = (TAU*j as f64/(2*n) as f64,-FRAC_PI_2+PI*i as f64/n as f64);
        [e[0]*v.cos()*u.cos(),e[1]*v.cos()*u.sin(),e[2]*v.sin()]
    })).collect();
    for q in points {
        let x = ellipsoid_foot(e,q);
        let level = (x[0]/e[0]).powi(2)+(x[1]/e[1]).powi(2)+(x[2]/e[2]).powi(2);
        assert!((level-1.).abs() <= 1e-14,"{q:?}: its foot {x:?} is off the ellipsoid by {:e}",level-1.);
        // `q − x` along the normal `x / e²`
        let n = [x[0]/(e[0]*e[0]),x[1]/(e[1]*e[1]),x[2]/(e[2]*e[2])];
        let d = [q[0]-x[0],q[1]-x[1],q[2]-x[2]];
        let c = [d[1]*n[2]-d[2]*n[1],d[2]*n[0]-d[0]*n[2],d[0]*n[1]-d[1]*n[0]];
        let nn = (n[0]*n[0]+n[1]*n[1]+n[2]*n[2]).sqrt();
        assert!((c[0]*c[0]+c[1]*c[1]+c[2]*c[2]).sqrt() <= 1e-12*nn*(1.+dist(q,x)),
            "{q:?}: off the normal at its foot {x:?}");
        let found = grid.iter().map(|&g| dist(q,g)).fold(f64::INFINITY,f64::min);
        assert!(dist(q,x) <= found+1e-12,"{q:?}: a sample {found} nearer than its foot {}",dist(q,x));
    }
}

/// The surface's derivatives are its point's, its inverse undoes it, and its signed distance is
/// the distance to its foot, negative inside, its gradient the unit normal there.
#[test]
fn an_ellipsoids_closed_forms_agree() {
    let f = Frame::new([1.,2.,-1.],[0.2,-0.3,1.],[1.,0.4,0.]);
    let s = Surface::Ellipsoid(f,SEMI);
    let mut rng = Rng::new(11);
    for _ in 0..200 {
        let uv = [rng.uniform(0.,TAU),rng.uniform(-1.5,1.5)];
        let (p,su,sv) = s.d1(uv);
        let h = 1e-6;
        for k in 0..3 {
            close((s.point([uv[0]+h,uv[1]])[k]-s.point([uv[0]-h,uv[1]])[k])/(2.*h),su[k],1e-8);
            close((s.point([uv[0],uv[1]+h])[k]-s.point([uv[0],uv[1]-h])[k])/(2.*h),sv[k],1e-8);
        }
        let back = s.inverse(p);
        assert!(dist(s.point(back),p) <= 1e-12,"{uv:?} read back as {back:?}");
        close(back[0],uv[0],1e-10);
        close(back[1],uv[1],1e-10);
        // off it along its outward normal, either way
        let n = s.normal(uv).unwrap();
        for d in [0.3,-0.2,1e-7,-1e-7] {
            let q = std::array::from_fn(|k| p[k]+d*n[k]);
            close(s.implicit(q),d,1e-9);
            let g = s.gradient(q);
            for k in 0..3 { close(g[k],n[k],1e-8); }
            assert!(dist(s.point(s.inverse(q)),p) <= 1e-9,"the foot of a point on the normal is where it stands off");
        }
    }
}

/// The net STEP writes is the ellipsoid: at the net's parameters of any `(u, v)` it is the
/// ellipsoid's point there, its quarter turns where the two parameters agree.
#[test]
fn an_ellipsoids_net_is_the_ellipsoid() {
    use gcs_core::brep::nurbs::{arc_parameter,ellipsoid_net};
    let f = Frame::new([1.,2.,-1.],[0.2,-0.3,1.],[1.,0.4,0.]);
    let (s,net) = (Surface::Ellipsoid(f,SEMI),ellipsoid_net(&f,SEMI));
    net.check().unwrap();
    let mut rng = Rng::new(3);
    for k in 0..400 {
        // the quarter turns first, then anywhere
        let uv = if k < 25 { [FRAC_PI_2*(k%5) as f64,-FRAC_PI_2+FRAC_PI_2*(k/5) as f64/2.] }
            else { [rng.uniform(0.,TAU),rng.uniform(-FRAC_PI_2,FRAC_PI_2)] };
        let (a,da) = arc_parameter([0.,TAU],uv[0]);
        let (b,db) = arc_parameter([-FRAC_PI_2,FRAC_PI_2],uv[1]);
        assert!(dist(net.point(a,b),s.point(uv)) <= 1e-12,"{uv:?} on the net at {a}, {b}");
        // the rates are the parameters' own: a step in angle is the step the rate says on the net
        let h = 1e-6;
        let (a1,_) = arc_parameter([0.,TAU],(uv[0]+h).min(TAU));
        let (b1,_) = arc_parameter([-FRAC_PI_2,FRAC_PI_2],(uv[1]+h).min(FRAC_PI_2));
        if uv[0]+h <= TAU { close((a1-a)/h,da,1e-5); }
        if uv[1]+h <= FRAC_PI_2 { close((b1-b)/h,db,1e-5); }
    }
}

/// A plane meets the ellipsoid in an ellipse in closed form, on both surfaces all round; a plane
/// tangent to it, or past it, in no curve.
#[test]
fn a_plane_meets_an_ellipsoid_in_an_ellipse() {
    use gcs_core::brep::ssi::{intersect,Ssi};
    let f = Frame::new([1.,2.,-1.],[0.2,-0.3,1.],[1.,0.4,0.]);
    let s = Surface::Ellipsoid(f,SEMI);
    let mut rng = Rng::new(5);
    for _ in 0..50 {
        let n: V = std::array::from_fn(|_| rng.uniform(-1.,1.));
        let p: V = std::array::from_fn(|k| f.o[k]+rng.uniform(-2.,2.));
        let plane = Surface::Plane(Frame::about(p,n));
        let Ssi::Curves(c) = intersect(&plane,&s,1e-9) else { panic!("a plane and an ellipsoid traced") };
        let [c] = &c[..] else { panic!("{} curves where a plane through its middle cuts it",c.len()) };
        assert!(matches!(c,Curve::Ellipse(..) | Curve::Circle(..)));
        for k in 0..64 {
            let q = c.point(TAU*k as f64/64.);
            assert!(plane.implicit(q).abs() <= 1e-12 && s.implicit(q).abs() <= 1e-12,"{q:?} off the meeting");
        }
    }
    // the plane square to an axis at its end touches the ellipsoid at a point; past it, nothing
    for h in [3.,3.5] {
        let plane = Surface::Plane(Frame::about(f.at([0.,0.,h]),f.z));
        assert!(matches!(intersect(&plane,&s,1e-9),Ssi::Curves(c) if c.is_empty()),"at {h}");
    }
    // a line through it meets it twice, in closed form, and tangent to it once
    use gcs_core::brep::query::{curve_surface,Meets};
    let line = Curve::Line {p:f.at([0.,0.,0.]),d:f.x};
    let Meets::At(at) = curve_surface(&line,[-10.,10.],&s,1e-9) else { panic!("a line along its surface") };
    assert_eq!(at.len(),2);
    close(at[0].0,-6.,1e-12);
    close(at[1].0,6.,1e-12);
    let line = Curve::Line {p:f.at([0.,4.,0.]),d:f.z};
    let Meets::At(at) = curve_surface(&line,[-10.,10.],&s,1e-9) else { panic!("a line along its surface") };
    assert!(matches!(at[..],[(t,true)] if t.abs() <= 1e-12),"a tangent line: {at:?}");
}

/// brep-bench's geometry/ellipsoid-half: the ellipsoid less everything below `z = 0`, cut by a box
/// reaching past it everywhere but there: the dome and its floor, half the volume, half the area
/// and the floor's `π a b`, meshed closed.
#[test]
fn an_ellipsoid_cut_at_its_equator_is_its_half() {
    let b = ellipsoid([0.;3],[0.,0.,1.],[1.,0.,0.],SEMI).unwrap();
    let above = prism(&square([0.;3],[0.,0.,1.],20.),0.,10.).unwrap();
    let half = combined(&b,&above,Op::Common,0.).unwrap();
    half.check(1e-9).unwrap();
    assert_eq!(half.faces.len(),2);
    close(volume(&half),48.*PI,1e-10);
    close(whole_area(&half),AREA/2.+24.*PI,1e-9);
    let m = gcs_core::brep::mesh::mesh(&half,1e-3,0.1).unwrap();
    gcs_core::mesh::stl_shells(&gcs_core::mesh::stl_of(&m.triangles(),"half")).unwrap();
}

/// A bore along its longest axis, through its seam: traced against the cylinder, the solid left
/// valid and its volume the ellipsoid's less the bore's, `2a ∬ √(1 − y²/b² − z²/c²)` over its disc.
#[test]
fn an_ellipsoid_bored_along_its_axis_is_its_closed_form() {
    let b = ellipsoid([0.;3],[0.,0.,1.],[1.,0.,0.],SEMI).unwrap();
    let r = 1.;
    let disc = Profile {origin:[-10.,0.,0.],normal:[1.,0.,0.],names:vec![],
        loops:vec![vec![ProfileEdge::Arc {frame:Frame::new([-10.,0.,0.],[1.,0.,0.],[0.,0.,1.]),r,span:None}]]};
    let bore = prism(&disc,0.,20.).unwrap();
    let bored = combined(&b,&bore,Op::Cut,0.).unwrap();
    bored.check(1e-9).unwrap();
    // the core: over the disc in polar coordinates, `∫₀¹ √(1 − k ρ²) ρ dρ = (1 − (1 − k)^{3/2}) / 3k`
    // for `k = cos²θ / b² + sin²θ / c²` (r = 1), summed evenly in θ, which is exact for a periodic
    // integrand to rounding
    let [a,e1,e2] = SEMI;
    let n = 4096;
    let core = 2.*a*(0..n).map(|i| {
        let t = TAU*i as f64/n as f64;
        let k = (t.cos()/e1).powi(2)+(t.sin()/e2).powi(2);
        (1.-(1.-k).powf(1.5))/(3.*k)
    }).sum::<f64>()*TAU/n as f64;
    close(volume(&bored),96.*PI-core,1e-9);
}

/// Written as STEP, the ellipsoid is its rational net, its seam's pcurves fitted in the net's
/// parameters, and the checker reads the solid back face for face; its half likewise.
#[test]
fn an_ellipsoid_is_written_to_step_as_its_net() {
    let b = ellipsoid([1.,2.,3.],[0.2,-0.3,1.],[1.,0.4,0.],SEMI).unwrap();
    let above = prism(&square([0.,0.,3.],[0.,0.,1.],30.),0.,20.).unwrap();
    let half = combined(&b,&above,Op::Common,0.).unwrap();
    for (what,solid) in [("ellipsoid",&b),("ellipsoid-half",&half)] {
        let text = gcs_core::brep::step::write(solid,what,1e-4).unwrap_or_else(|e| panic!("{what}: {e}"));
        assert!(text.contains("RATIONAL_B_SPLINE_SURFACE"),"{what}: no rational net written");
        gcs_core::brep::step_check::verify(&text,&gcs_core::brep::step_check::Solid::of(solid))
            .unwrap_or_else(|e| panic!("{what}: {e}"));
    }
}

/// Moved and reflected, it is the same ellipsoid where it was taken.
#[test]
fn an_ellipsoid_moved_or_reflected_keeps_its_measure() {
    use gcs_core::brep::geom::Rigid;
    let b = ellipsoid([1.,2.,3.],[0.2,-0.3,1.],[1.,0.4,0.],SEMI).unwrap();
    let turned = b.moved(&Rigid::turn([0.;3],[1.,1.,0.],0.7));
    turned.check(1e-9).unwrap();
    close(volume(&turned),96.*PI,1e-12);
    let mirror = Rigid {r:[[-1.,0.,0.],[0.,1.,0.],[0.,0.,1.]],t:[0.;3]};
    let flipped = b.reflected(&mirror).unwrap();
    flipped.check(1e-9).unwrap();
    close(volume(&flipped),96.*PI,1e-12);
    close(whole_area(&flipped),AREA,1e-10);
}
