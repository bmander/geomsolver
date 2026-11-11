//! **Enclosures, the tests behind the boundary** (#814): every kind of curve and surface is held by
//! its own enclosure over random boxes — B-splines at every knot multiplicity, rational ones,
//! degenerate blends — every sample of its point and its derivatives inside; the two tests a
//! certified search can read a box by find the same roots; a certified root that cannot be settled
//! is refused, never returned; and the general reading every pair has is held to each closed form
//! (`curve_surface_searched` against `curve_surface`), and both to the roots' own points.
use gcs_core::brep::enclose::{Enclosed,Patch};
use gcs_core::brep::family::{Run,Sheet,Stations,NODES};
use gcs_core::brep::geom::{Blend,Carry,Curve,Frame,SheetCut,Surface,Traced};
use gcs_core::brep::nurbs::{BSpline,Net};
use gcs_core::brep::query::{curve_surface,curve_surface_by,curve_surface_searched,Meets,Reader,Uncertified};
use gcs_core::interval::Iv;
use gcs_core::rng::Rng;
use std::sync::Arc;

type V = [f64;3];

/// Whether `x` (a float reading, rounded) lies in `iv`, a rounding's slack either side.
fn holds(iv: Iv,x: f64) -> bool {
    let slack = 1e-9*(1.+x.abs());
    iv.lo()-slack <= x && x <= iv.hi()+slack
}

/// Whether the enclosure holds a point and its derivatives.
fn held<const P: usize>(e: &Patch<P>,at: V,d: [V;P]) -> bool {
    (0..3).all(|k| holds(e.at[k],at[k])) && (0..P).all(|j| (0..3).all(|k| holds(e.d[j][k],d[j][k])))
}

/// A random stretch within `[a, b]`: often narrow, sometimes the whole, sometimes a point.
fn stretch(rng: &mut Rng,[a,b]: [f64;2]) -> [f64;2] {
    let x = rng.uniform(a,b);
    match rng.uniform(0.,1.) {
        r if r < 0.1 => [a,b],
        r if r < 0.2 => [x,x],
        _ => { let w = (b-a)*rng.uniform(0.,1.).powi(3); [x,(x+w).min(b)] }
    }
}

fn unit(v: V) -> V { gcs_core::space::normalised(v).unwrap() }

/// The curves of every kind, each with the stretch it is read over.
fn curves() -> Vec<(&'static str,Curve,[f64;2])> {
    use std::f64::consts::TAU;
    let tilted = Frame::new([1.,-2.,0.5],unit([0.3,0.4,0.85]),unit([1.,0.,-0.3/0.85*0.]));
    let tilted = Frame::new(tilted.o,tilted.z,unit(gcs_core::space::cross(tilted.z,[0.,1.,0.])));
    let cubic = |knots: Vec<f64>| {
        let n = knots.len()-4;
        let poles: Vec<V> = (0..n).map(|i| { let s = i as f64; [s,(1.3*s).sin()*2.,0.4*s*s-s] }).collect();
        Curve::BSpline(Arc::new(BSpline::new(3,knots,poles).unwrap()))
    };
    // a quarter circle as a rational quadratic, and a rational cubic with uneven weights
    let w = std::f64::consts::FRAC_1_SQRT_2;
    let arc = BSpline::rational(2,vec![0.,0.,0.,1.,1.,1.],vec![[1.,0.,0.],[1.,1.,0.],[0.,1.,0.]],vec![1.,w,1.]).unwrap();
    let wavy = BSpline::rational(3,vec![0.,0.,0.,0.,0.5,0.5,1.,1.,1.,1.],
        vec![[0.,0.,0.],[1.,2.,0.],[2.,-1.,1.],[3.,1.,0.],[4.,0.,2.],[5.,1.,1.]],vec![1.,0.3,2.,1.,4.,0.7]).unwrap();
    let screw = Carry::Screw {center:[0.,0.,0.],axis:[0.,0.,1.],angle:2.5,advance:3.};
    let line = Curve::Line {p:[2.,0.,0.],d:unit([0.2,1.,0.3])};
    let screwed = Arc::new(Blend {a:line.clone(),ta:[0.,1.5],b:line.clone(),tb:[0.,1.5],carry:screw,closed:false});
    let cut = SheetCut::new(screwed.clone(),unit([0.1,0.2,1.]),1.4,Vec::new()).expect("a screwed line's sheet");
    let map = gcs_core::brep::affine::Affine::new([[2.,0.3,0.],[0.,0.7,0.1],[0.2,0.,1.4]],[0.5,1.,-1.]).unwrap();
    let mapped = gcs_core::brep::affine::MappedCurve::new(map,Curve::Circle(tilted,1.5),[1.,0.]);
    let contact = gcs_core::brep::turned::Contact {o:[4.,-1.5,0.3],a:unit([0.42,-0.1,0.9]),r:1.3,sa:-(0.2f64.sin()),ca:0.2f64.cos(),side:1.};
    let turned = gcs_core::brep::turned::Turned::meridian(Frame::WORLD,contact,[-1.,1.]);
    let slid = gcs_core::brep::slid::halves(Frame::WORLD,3.,1.,unit([1.,0.,0.3]),1.,true)[0];
    let ring: Vec<V> = (0..=48).map(|k| { let a = TAU*k as f64/48.; [2.*a.cos(),2.*a.sin(),0.] }).collect();
    let traced = Traced {a:Surface::Plane(Frame::WORLD),b:Surface::Cylinder(Frame::WORLD,2.),pts:ring,closed:true};
    let wall = Arc::new(family());
    vec![
        ("a line",Curve::Line {p:[1.,2.,3.],d:unit([0.3,-0.5,0.8])},[-5.,7.]),
        ("a circle",Curve::Circle(tilted,2.5),[0.,TAU]),
        ("a circle past a turn",Curve::Circle(tilted,2.5),[-9.,14.]),
        ("an ellipse",Curve::Ellipse(tilted,3.,1.2),[0.,TAU]),
        ("a cubic, simple knots",cubic(vec![0.,0.,0.,0.,1.,2.,3.,4.,4.,4.,4.]),[0.,4.]),
        ("a cubic, a double knot",cubic(vec![0.,0.,0.,0.,1.,2.,2.,4.,4.,4.,4.]),[0.,4.]),
        ("a cubic, a kink",cubic(vec![0.,0.,0.,0.,1.,2.,2.,2.,4.,4.,4.,4.]),[0.,4.]),
        ("a cubic, a break",cubic(vec![0.,0.,0.,0.,1.,2.,2.,2.,2.,4.,4.,4.,4.]),[0.,4.]),
        ("a rational arc",Curve::BSpline(Arc::new(arc)),[0.,1.]),
        ("a rational cubic",Curve::BSpline(Arc::new(wavy)),[0.,1.]),
        ("a screwed line's rail",Curve::Iso(screwed.clone(),0.6),[0.,1.]),
        ("a plane's cut of a screwed sheet",Curve::SheetCut(Arc::new(cut)),[0.05,0.95]),
        ("a family's rail",Curve::Rail(wall.clone(),0.3),wall.span()),
        ("a family's rim",Curve::Rim(wall.clone(),0.4),[0.,1.]),
        ("an affine image",Curve::Affine(Arc::new(mapped)),[0.,TAU]),
        ("a turned contact",Curve::Turned(Arc::new(turned)),[-1.,1.]),
        ("a slid contour",Curve::Slid(Arc::new(slid)),[-1.,1.]),
        ("a trace",Curve::Traced(Arc::new(traced)),[0.,48.]),
    ]
}

/// A face family's wall: a line's ends running over one piece of `[0, 1]`.
fn family() -> Sheet {
    let at: [f64;6] = NODES.map(|t| 0.5+0.5*t);
    let stations = Arc::new(Stations::new(vec![[0.,1.]],vec![at]).unwrap());
    let nodes: Vec<Vec<f64>> = at.iter().map(|&s| vec![s,0.,s*s,1.+0.3*s,1.,(2.*s).sin()]).collect();
    Sheet::new(Run::Line,stations,nodes,false).unwrap()
}

/// **Every kind of curve is held by its enclosure** (#814 P2): over random stretches — narrow,
/// whole, a point — its point and derivative at samples across each lie in the enclosure. A
/// trace's is its own measured bound, and says so.
#[test]
fn every_kind_of_curve_is_held_by_its_enclosure() {
    let mut rng = Rng::new(814);
    for (what,c,span) in curves() {
        for _ in 0..60 {
            let [a,b] = stretch(&mut rng,span);
            let e = c.enclose([[a,b]]);
            assert!(e.bounded(),"{what} over [{a}, {b}]: no enclosure {e:?}");
            assert_eq!(e.proved,!matches!(c,Curve::Traced(_)),"{what}: proved");
            for k in 0..=16 {
                let t = a+(b-a)*k as f64/16.;
                let (p,d) = c.d1(t);
                assert!(held(&e,p,[d]),"{what} over [{a}, {b}] at {t}: {p:?} {d:?} outside {e:?}");
            }
        }
    }
}

/// The surfaces of every kind, each with the parameters it is read over.
fn surfaces() -> Vec<(&'static str,Surface,[[f64;2];2])> {
    use std::f64::consts::{FRAC_PI_2,TAU};
    let f = Frame::new([0.5,-1.,2.],unit([0.2,-0.3,1.]),unit([1.,0.,-0.2]));
    let meridian = Curve::BSpline(Arc::new(BSpline::new(2,vec![0.,0.,0.,0.5,1.,1.,1.],vec![[1.,0.,-1.],[2.5,0.,0.],[0.5,0.,1.],[1.5,0.,2.]]).unwrap()));
    let profile = Curve::BSpline(Arc::new(BSpline::new(3,vec![0.,0.,0.,0.,1.,1.,2.,2.,2.,2.],
        vec![[0.,0.,0.],[1.,2.,0.],[2.,-1.,0.],[3.,1.,0.],[4.,0.,0.],[5.,1.,0.]]).unwrap()));
    let contact = gcs_core::brep::turned::Contact {o:[5.,1.2,-0.5],a:unit([0.42,-0.1,0.9]),r:1.,sa:0.,ca:1.,side:1.};
    let turned = Curve::Turned(Arc::new(gcs_core::brep::turned::Turned::meridian(Frame::WORLD,contact,[-1.,1.])));
    let circle = Curve::Circle(Frame::about([3.,0.,0.],[0.,1.,0.]),0.8);
    let pipe = Blend {a:circle.clone(),ta:[0.,TAU],b:circle,tb:[0.,TAU],
        carry:Carry::Screw {center:[0.,0.,0.],axis:[0.,0.,1.],angle:4.,advance:2.},closed:true};
    let drafted = Blend {a:Curve::Line {p:[0.,0.,0.],d:[1.,0.,0.]},ta:[0.,3.],b:Curve::Circle(Frame::about([1.,0.,0.],[0.,0.,1.]),2.),tb:[0.,1.],
        carry:Carry::Line {delta:[0.2,0.1,3.]},closed:false};
    // a cubic net with a kink one way (a knot of the degree's multiplicity) and a double knot the
    // other, and a rational biquadratic (a quarter of a torus's tube, its weights uneven)
    let grid = |nu: usize,nv: usize| -> Vec<Vec<V>> { (0..nu).map(|i| (0..nv).map(|j| { let (x,y) = (i as f64,j as f64); [x,y,(0.7*x).sin()+0.2*x*y-0.1*y*y] }).collect()).collect() };
    let kinked = Net {du:3,dv:3,uknots:vec![0.,0.,0.,0.,0.7,0.7,0.7,2.,2.,2.,2.],vknots:vec![0.,0.,0.,0.,0.4,0.4,1.,1.,1.,1.],poles:grid(7,6),weights:None};
    // a quintic by quartic net over several spans each way, a knot doubled, as a ring's tube is fitted
    let quintic = Net {du:5,dv:4,uknots:vec![0.,0.,0.,0.,0.,0.,0.3,0.6,0.6,1.,1.,1.,1.,1.,1.],
        vknots:vec![0.,0.,0.,0.,0.,0.5,1.,1.,1.,1.,1.],poles:grid(9,6),weights:None};
    let w = std::f64::consts::FRAC_1_SQRT_2;
    let rational = Net {du:2,dv:2,uknots:vec![0.,0.,0.,1.,1.,1.],vknots:vec![0.,0.,0.,1.,1.,1.],
        poles:vec![vec![[3.,0.,0.],[3.,0.,1.],[2.,0.,1.]],vec![[3.,3.,0.],[3.,3.,1.],[2.,2.,1.]],vec![[0.,3.,0.],[0.,3.,1.],[0.,2.,1.]]],
        weights:Some(vec![vec![1.,w,1.],vec![w,0.5,w],vec![1.,w,1.]])};
    let map = gcs_core::brep::affine::Affine::new([[2.,0.3,0.],[0.,0.7,0.1],[0.2,0.,1.4]],[0.5,1.,-1.]).unwrap();
    let ellipsoid = gcs_core::brep::affine::Mapped::new(map,Surface::Torus(f,3.,1.));
    let wall = family();
    let ws = wall.span();
    vec![
        ("a plane",Surface::Plane(f),[[-5.,5.],[-3.,4.]]),
        ("a cylinder",Surface::Cylinder(f,2.),[[0.,TAU],[-3.,3.]]),
        ("a cone",Surface::Cone(f,1.5,0.4),[[0.,TAU],[-2.,3.]]),
        ("a ball",Surface::Sphere(f,2.),[[0.,TAU],[-FRAC_PI_2,FRAC_PI_2]]),
        ("a torus",Surface::Torus(f,3.,1.),[[0.,TAU],[0.,TAU]]),
        ("an extrusion",Surface::Extrusion(Frame::WORLD,Arc::new(profile)),[[0.,2.],[-1.,2.]]),
        ("a revolution",Surface::Revolution(Frame::WORLD,Arc::new(meridian)),[[0.,TAU],[0.,1.]]),
        ("a turned contact's revolution",Surface::Revolution(Frame::WORLD,Arc::new(turned)),[[0.,TAU],[-1.,1.]]),
        ("a pipe",Surface::Blend(Frame::WORLD,Arc::new(pipe)),[[0.,1.],[0.,1.]]),
        ("a drafted blend",Surface::Blend(Frame::WORLD,Arc::new(drafted)),[[0.,1.],[0.,1.]]),
        ("a kinked net",Surface::BSpline(Frame::WORLD,Arc::new(kinked)),[[0.,2.],[0.,1.]]),
        ("a quintic net",Surface::BSpline(Frame::WORLD,Arc::new(quintic)),[[0.,1.],[0.,1.]]),
        ("a rational net",Surface::BSpline(Frame::WORLD,Arc::new(rational)),[[0.,1.],[0.,1.]]),
        ("a family's wall",Surface::Family(Frame::WORLD,Arc::new(wall)),[[0.,1.],ws]),
        ("an affine image",Surface::Affine(Arc::new(ellipsoid)),[[0.,TAU],[0.,TAU]]),
    ]
}

/// **Every kind of surface is held by its enclosure** (#814 P2): over random boxes, its point and
/// both partial derivatives at a grid of samples each lie in the enclosure — nets with a kink, a
/// double knot and uneven weights, blends carried along a line and by a screw.
#[test]
fn every_kind_of_surface_is_held_by_its_enclosure() {
    let mut rng = Rng::new(812);
    for (what,s,[du,dv]) in surfaces() {
        for _ in 0..40 {
            let (u,v) = (stretch(&mut rng,du),stretch(&mut rng,dv));
            let e = s.enclose([u,v]);
            assert!(e.bounded(),"{what} over {u:?} × {v:?}: no enclosure {e:?}");
            for i in 0..=6 { for j in 0..=6 {
                let uv = [u[0]+(u[1]-u[0])*i as f64/6.,v[0]+(v[1]-v[0])*j as f64/6.];
                let (p,su,sv) = s.d1(uv);
                assert!(held(&e,p,[su,sv]),"{what} over {u:?} × {v:?} at {uv:?}: {p:?} {su:?} {sv:?} outside {e:?}");
            } }
        }
    }
}

/// **The two tests a box is read by find the same roots** (#814 P1): random systems `x² + y² − r²`
/// and a line through them, each root found by the Krawczyk operator (its Jacobian enclosed) and by
/// Kantorovich's theorem (from its true second derivatives), alike, each on the system to
/// rounding.
#[test]
fn the_tests_behind_the_boundary_find_the_same_roots() {
    use gcs_core::isolate::{solve,solve_by,Closures,Kantorovich};
    let mut rng = Rng::new(796);
    for _ in 0..40 {
        let (cx,cy,r) = (rng.uniform(-0.3,0.3),rng.uniform(-0.3,0.3),rng.uniform(0.2,0.9));
        let (a,b,c) = (rng.uniform(-1.,1.),rng.uniform(-1.,1.),rng.uniform(-0.3,0.3));
        let at = |x: [f64;2]| ([(x[0]-cx).powi(2)+(x[1]-cy).powi(2)-r*r,a*x[0]+b*x[1]-c],[[2.*(x[0]-cx),2.*(x[1]-cy)],[a,b]]);
        let over = |bx: &[[f64;2];2]| {
            let (x,y) = (Iv::new(bx[0][0],bx[0][1]),Iv::new(bx[1][0],bx[1][1]));
            let (dx,dy) = (x-Iv::point(cx),y-Iv::point(cy));
            ([dx*dx+dy*dy-Iv::point(r*r),Iv::point(a)*x+Iv::point(b)*y-Iv::point(c)],
             [[dx.times(2.),dy.times(2.)],[Iv::point(a),Iv::point(b)]])
        };
        let sys = Closures {at:&at,over:&over};
        let near = |x: [f64;2]| at(x).0.iter().all(|v| v.abs() <= 1e-10);
        let domain = [[-1.5,1.5],[-1.5,1.5]];
        let k = solve(&sys,domain,1e-12,&near).expect("Krawczyk's search stops");
        let second = |_: &[[f64;2];2]| [4.,0.];
        let t = solve_by(&Kantorovich(&second),&sys,domain,1e-12,&near).expect("Kantorovich's search stops");
        let mut k: Vec<[f64;2]> = k.iter().filter(|p| !p.touch).map(|p| p.at).collect();
        let mut t: Vec<[f64;2]> = t.iter().filter(|p| !p.touch).map(|p| p.at).collect();
        k.sort_by(|x,y| x[0].total_cmp(&y[0]));
        t.sort_by(|x,y| x[0].total_cmp(&y[0]));
        assert_eq!(k.len(),t.len(),"{k:?} against {t:?}");
        for (x,y) in k.iter().zip(&t) {
            assert!((x[0]-y[0]).abs() <= 1e-9 && (x[1]-y[1]).abs() <= 1e-9,"{k:?} against {t:?}");
            assert!(at(*x).0.iter().all(|v| v.abs() <= 1e-12),"{x:?} off the system");
        }
    }
}

/// **A root that cannot be settled is refused, never returned** (#814 P0, #812): Kantorovich's
/// theorem handed a bound of nought on the second derivatives of `sin 20x − ½` (as `second_bounds`
/// read nought across a kink) certifies balls its Newton steps leave. Whatever it returns is a
/// root; what it cannot settle it gives up on, with the box — never the last iterate presented as
/// one. The Krawczyk operator, which needs no such bound, finds all seven.
#[test]
fn a_root_that_cannot_be_settled_is_refused_never_returned() {
    use gcs_core::isolate::{solve,solve_by,Closures,GaveUp,Kantorovich};
    let at = |x: [f64;1]| ([(20.*x[0]).sin()-0.5],[[20.*(20.*x[0]).cos()]]);
    let over = |b: &[[f64;2];1]| { let (s,c) = Iv::new(b[0][0],b[0][1]).times(20.).sin_cos_iv(); ([s-Iv::point(0.5)],[[c.times(20.)]]) };
    let sys = Closures {at:&at,over:&over};
    let near = |x: [f64;1]| at(x).0[0].abs() <= 1e-9;
    let lying = |_: &[[f64;2];1]| [0.];
    match solve_by(&Kantorovich(&lying),&sys,[[0.,1.]],1e-12,&near) {
        Ok(found) => for p in found { assert!(at(p.at).0[0].abs() <= 1e-9,"returned off the function: {p:?}") },
        Err(e) => assert!(matches!(e,GaveUp::Unsettled {..}),"{e:?}"),
    }
    let found = solve(&sys,[[0.,1.]],1e-12,&near).expect("Krawczyk's search stops");
    let want: Vec<f64> = (0..4).flat_map(|k| { let base = std::f64::consts::PI*2.*k as f64; [(base+std::f64::consts::FRAC_PI_6)/20.,(base+5.*std::f64::consts::FRAC_PI_6)/20.] })
        .filter(|&x| x <= 1.).collect();
    assert_eq!(found.len(),want.len(),"{found:?}");
    for (p,x) in found.iter().zip(&want) { assert!(!p.touch && (p.at[0]-x).abs() <= 1e-12,"{found:?} against {want:?}") }
}

/// Sine and cosine enclosed, for the tests' systems.
trait SinCos { fn sin_cos_iv(self) -> (Iv,Iv); }
impl SinCos for Iv { fn sin_cos_iv(self) -> (Iv,Iv) { use gcs_core::interval::Num; self.sin_cos() } }

/// **The general reading holds every closed form** (#814 P4): random lines, circles and splines
/// against a plane, a cylinder, a cone, a ball, a torus, a level circle's screwed sheet and two
/// straight edges' (a leaning edge screwed, and a sheet between two edges carried by a screw), and
/// lines against a conic's and a turned contact's revolutions and a slid contour's cylinder, read by
/// the closed forms where the pair has one (`curve_surface`) and by the general reading alone
/// (`curve_surface_searched`), find the same meetings, each on both curve and surface.
#[test]
fn the_general_reading_holds_every_closed_form() {
    use std::f64::consts::TAU;
    let f = Frame::new([0.2,-0.1,0.3],unit([0.1,-0.2,1.]),unit([1.,0.,-0.1]));
    let rim = Curve::Circle(Frame::about([0.,0.,0.],[0.,0.,1.]),1.5);
    let sheet = Blend {a:rim.clone(),ta:[0.,TAU],b:rim,tb:[0.,TAU],carry:Carry::Screw {center:[0.3,0.,0.],axis:[0.,0.,1.],angle:3.,advance:4.},closed:true};
    // a conic meridian (a rational quadratic) and a turned contact's, turned; a slid contour's
    // cylinder
    let w = std::f64::consts::FRAC_1_SQRT_2;
    let conic = Curve::BSpline(Arc::new(BSpline::rational(2,vec![0.,0.,0.,1.,1.,1.],vec![[2.,0.,-1.],[1.,0.,0.],[2.,0.,1.]],vec![1.,w,1.]).unwrap()));
    let contact = gcs_core::brep::turned::Contact {o:[5.,1.2,-0.5],a:unit([0.42,-0.1,0.9]),r:1.,sa:0.,ca:1.,side:1.};
    let turned = Curve::Turned(Arc::new(gcs_core::brep::turned::Turned::meridian(Frame::WORLD,contact,[-1.,1.])));
    let slide = unit([1.,0.,0.3]);
    let slid = Curve::Slid(Arc::new(gcs_core::brep::slid::halves(Frame::WORLD,3.,1.,slide,1.,true)[0]));
    // a leaning edge screwed two radians about an axis off the origin, and a sheet between two
    // edges carried by a screw (`query::line_screwed_line`)
    let leaning = Curve::Line {p:[1.5,-1.,-0.5],d:unit([0.3,1.,0.8])};
    let screwed = Blend {a:leaning.clone(),ta:[0.,2.5],b:leaning,tb:[0.,2.5],carry:Carry::Screw {center:[0.2,0.1,0.],axis:[0.,0.,1.],angle:2.,advance:1.5},closed:false};
    let between = Blend {a:Curve::Line {p:[2.,0.,-1.],d:unit([0.,1.,0.6])},ta:[-1.,2.],b:Curve::Line {p:[1.,0.5,0.5],d:unit([0.2,1.,-0.3])},tb:[0.,2.],
        carry:Carry::Screw {center:[0.;3],axis:unit([0.1,0.,1.]),angle:-1.2,advance:2.},closed:false};
    let surfaces = [Surface::Plane(f),Surface::Cylinder(f,2.),Surface::Cone(f,1.5,0.4),Surface::Sphere(f,2.),Surface::Torus(f,3.,1.),
        Surface::Blend(Frame::WORLD,Arc::new(sheet)),Surface::Revolution(Frame::WORLD,Arc::new(conic)),Surface::Revolution(Frame::WORLD,Arc::new(turned)),
        Surface::Extrusion(Frame::about([0.;3],slide),Arc::new(slid)),Surface::Blend(Frame::WORLD,Arc::new(screwed)),
        Surface::Blend(Frame::WORLD,Arc::new(between))];
    let mut rng = Rng::new(4);
    let (mut met,mut read) = (0,0);
    for s in &surfaces {
        for k in 0..30 {
            let p = [rng.uniform(-4.,4.),rng.uniform(-4.,4.),rng.uniform(-2.,4.)];
            // (lines and circles against a conic turned, lines alone against a turned contact and a
            // slid contour: the closed forms there read those)
            let conic_turned = matches!(s,Surface::Revolution(_,c) if matches!(**c,Curve::BSpline(..)));
            let k = if conic_turned { k%2 } else if matches!(s,Surface::Revolution(..) | Surface::Extrusion(..)) { 0 } else { k };
            let curve = match k%3 {
                0 => Curve::Line {p,d:unit([rng.uniform(-1.,1.),rng.uniform(-1.,1.),rng.uniform(-1.,1.)])},
                1 => Curve::Circle(Frame::about(p,unit([rng.uniform(-1.,1.),rng.uniform(-1.,1.),rng.uniform(-1.,1.)])),rng.uniform(0.5,4.)),
                _ => Curve::BSpline(Arc::new(BSpline::new(3,vec![0.,0.,0.,0.,1.,2.,2.,2.,2.],
                    (0..5).map(|_| [rng.uniform(-4.,4.),rng.uniform(-4.,4.),rng.uniform(-2.,4.)]).collect()).unwrap())),
            };
            let span = match &curve { Curve::Line {..} => [-6.,6.],Curve::Circle(..) => [0.,TAU],_ => [0.,2.] };
            let (one,two) = (curve_surface(&curve,span,s,1e-9),curve_surface_searched(&curve,span,s,1e-9));
            let (Meets::At(one),Meets::At(two)) = (&one,&two) else { panic!("{} × {}: {one:?} against {two:?}",curve.kind(),s.kind()) };
            read += 1;
            let crossings = |r: &[(f64,bool)]| r.iter().filter(|x| !x.1).map(|x| x.0).collect::<Vec<f64>>();
            let (x,y) = (crossings(one),crossings(two));
            assert_eq!(x.len(),y.len(),"{} × {}: {one:?} against {two:?}",curve.kind(),s.kind());
            for (a,b) in x.iter().zip(&y) {
                assert!((a-b).abs() <= 1e-7,"{} × {}: {one:?} against {two:?}",curve.kind(),s.kind());
                assert!(s.implicit(curve.point(*b)).abs() <= 1e-7,"{} × {}: {b} off the surface",curve.kind(),s.kind());
            }
            met += x.len();
        }
    }
    assert!(read == 330 && met > 110,"{read} read, {met} met");
}

/// **A reading that proves nothing is refused, with its witness** (#814 P3), whatever reads it:
/// a reader swapped in behind the boundary (`Reader`) that cannot prove its roots makes the
/// reading `Refused`, its witness the curve, the surface and the box, said in words.
#[test]
fn a_reading_that_proves_nothing_is_refused_with_its_witness() {
    struct Unsure;
    impl Reader for Unsure {
        fn read(&self,_: &Curve,_: [f64;2],_: &Surface,_: f64) -> Result<(Meets,bool),Uncertified> {
            Err(Uncertified::Gave(gcs_core::isolate::GaveUp::Unsettled {at:vec![[0.25,0.5],[0.,1.],[0.5,0.75]]}))
        }
    }
    let line = Curve::Line {p:[0.,0.,-5.],d:[0.,0.,1.]};
    let Meets::Refused(r) = curve_surface_by(&Unsure,&line,[0.,10.],&Surface::Plane(Frame::WORLD),1e-9) else { panic!("read") };
    assert_eq!((r.curve,r.surface,r.stretch,r.at.len()),("line","plane",[0.,10.],3),"{r:?}");
    let said = r.to_string();
    assert!(said.contains("line") && said.contains("plane") && said.contains("[0.25, 0.5]"),"{said}");
}

/// **A curve lying along a surface for a while is one run, proved — not refused, not sampled**
/// (#814): a polyline lying in a flat net over its middle stretch and leaving it either side, and
/// the same against the plane read by the general reading alone. No box can part the run's roots nor exclude
/// them; the search proves the run — the curve held within the tolerance of the surface along a
/// path of its parameters, piece by piece in intervals — reads its ends, and the reading settles
/// it as one meeting within the run's stretch.
#[test]
fn a_curve_lying_along_a_surface_for_a_while_is_one_run_proved() {
    let flat = Net {du:1,dv:1,uknots:vec![0.,0.,1.,1.],vknots:vec![0.,0.,1.,1.],
        poles:vec![vec![[0.,0.,0.],[0.,1.,0.]],vec![[1.,0.,0.],[1.,1.,0.]]],weights:None};
    let sheet = Surface::BSpline(Frame::WORLD,Arc::new(flat));
    let line = Curve::BSpline(Arc::new(BSpline::new(1,vec![0.,0.,1.,2.,3.,3.],vec![[-1.,0.5,1.],[0.,0.5,0.],[1.,0.5,0.],[2.,0.5,1.]]).unwrap()));
    let m = curve_surface(&line,[0.,3.],&sheet,1e-9);
    let Meets::At(roots) = &m else { panic!("{m:?}") };
    assert!(!roots.is_empty() && roots.iter().all(|r| r.0 >= 1.-1e-9 && r.0 <= 2.+1e-9),"{m:?}");
    let bent = Curve::BSpline(Arc::new(BSpline::new(1,vec![0.,0.,1.,2.,3.,3.],vec![[-1.,0.,1.],[0.,0.,0.],[1.,0.,0.],[2.,0.,1.]]).unwrap()));
    let m = curve_surface_searched(&bent,[0.,3.],&Surface::Plane(Frame::WORLD),1e-9);
    let Meets::At(roots) = &m else { panic!("{m:?}") };
    assert!(!roots.is_empty() && roots.iter().all(|r| r.0 >= 1.-1e-9 && r.0 <= 2.+1e-9),"{m:?}");
}

/// **A spline edge hugging a fine fitted net is found where it crosses it** (#814): a cubic net of
/// 240 × 66 poles — a height field, its poles standing at their knots' Greville abscissae so `x = u`
/// and `y = v` exactly, its heights a wave with a fitting's noise on it — against a cubic edge
/// hugging it within microns over a few dozen spans, crossing it shallowly, as the hypoid pinion's
/// side hugs its cut's sheet (`brep_export::the_pinion_as_written_is_built_by_sectors`). Each
/// crossing is found once, against the heights' difference along the edge bisected apart from the
/// kernel (`z(t) − S(x(t), y(t))`), each on both.
#[test]
fn a_spline_edge_hugging_a_fine_net_is_found_where_it_crosses() {
    let mut rng = Rng::new(9);
    let (nu,nv,deg) = (240,66,3);
    let knots = |n: usize| -> Vec<f64> {
        let inner = n-deg;
        (0..n+deg+1).map(|i| i.saturating_sub(deg).min(inner) as f64).collect()
    };
    let (uk,vk) = (knots(nu),knots(nv));
    let greville = |k: &[f64],i: usize| (1..=deg).map(|j| k[i+j]).sum::<f64>()/deg as f64;
    let poles: Vec<Vec<V>> = (0..nu).map(|i| (0..nv).map(|j| {
        let (x,y) = (greville(&uk,i),greville(&vk,j));
        [x,y,0.4*(0.37*x).sin()*(0.21*y).cos()+rng.uniform(-0.03,0.03)]
    }).collect()).collect();
    let net = Net {du:deg,dv:deg,uknots:uk,vknots:vk,poles,weights:None};
    net.check().unwrap();
    let sheet = Surface::BSpline(Frame::WORLD,Arc::new(net.clone()));
    // the edge: a cubic through points on the sheet along a diagonal a few dozen spans long, each
    // lifted or sunk a few microns, so it hugs the sheet and crosses it shallowly between
    let n: usize = 30;
    let edge_poles: Vec<V> = (0..n).map(|i| {
        let s = i as f64/(n-1) as f64;
        let (x,y) = (60.+40.*s,20.+12.*s);
        [x,y,net.point(x,y)[2]+if i%2 == 0 { 4e-3 } else { -4e-3 }]
    }).collect();
    let inner = n-3;
    let edge_knots: Vec<f64> = (0..n+4).map(|i| i.saturating_sub(3).min(inner) as f64/inner as f64).collect();
    let edge = Curve::BSpline(Arc::new(BSpline::new(3,edge_knots,edge_poles).unwrap()));
    let tol = 1e-6;
    // the heights' difference along the edge, bisected wherever it changes sign
    let gap = |t: f64| { let q = edge.point(t); q[2]-net.point(q[0],q[1])[2] };
    let steps = 40_000;
    let bisected: Vec<f64> = (0..steps).filter_map(|i| {
        let (mut a,mut b) = (i as f64/steps as f64,(i+1) as f64/steps as f64);
        if gap(a)*gap(b) > 0. { return None }
        for _ in 0..80 { let c = 0.5*(a+b); if (gap(c) < 0.) == (gap(a) < 0.) { a = c } else { b = c } }
        Some(0.5*(a+b))
    }).collect();
    assert!(bisected.len() >= 10,"{bisected:?}");
    let m = curve_surface(&edge,[0.,1.],&sheet,tol);
    let Meets::At(found) = &m else { panic!("{m:?}") };
    assert_eq!(found.len(),bisected.len(),"{found:?} against {bisected:?}");
    for (&(t,touch),&s) in found.iter().zip(&bisected) {
        assert!(!touch,"{found:?}");
        let q = edge.point(t);
        assert!(gcs_core::space::distance(q,edge.point(s)) <= 1e-5,"at {t}, {q:?}, against {s}");
        assert!(gap(t).abs() <= 1e-5,"at {t}, {q:?} off the sheet by {}",gap(t));
    }
}
