//! **A round face about an axis leaning to a turn's** (#780 C3, `brep::turned`,
//! `brep::swept::closed`): its contact on each ring is `A cos u + B sin u = C`, a curve along the
//! face, and turned about the turn's axis it sweeps a surface of revolution whose meridian is read
//! exactly (`geom::Curve::Turned`) — its jets one formula read in floats and in outward-rounded
//! intervals. Held here: the contact is where the face's normal is square to the turn's velocity,
//! its jets agree with differences and its intervals hold its samples; a line meets the turned
//! surface where sampling finds it (`query::line_turned`, certified); and the closed class builds
//! leaning tools turned, each to the volume of its slices' radial spans worked apart from the build
//! and to membership.
use crate::brep::{block,simpson};
use gcs_core::brep::build::{disc,prism};
use gcs_core::brep::geom::{Curve,Frame,Surface};
use gcs_core::brep::props::volume;
use gcs_core::brep::swept::closed::closed;
use gcs_core::brep::swept::member::{Solid,Swept};
use gcs_core::brep::topo::Brep;
use gcs_core::brep::trajectory::{Screw,Trajectory};
use gcs_core::brep::turned::{Contact,Jet,Turned};
use gcs_core::space::{add,cross,dot,norm,scale,sub,unit};
use std::f64::consts::{PI,TAU};

type V = [f64;3];

/// The turn by `angle` about the world's `z` axis over the roll `[0, 1]`.
fn turn(angle: f64) -> Trajectory { Trajectory::Twist(Screw {origin:[0.;3],axis:[0.,0.,1.],ratio:angle,advance:0.}) }

/// The contacts of a drum and a frustum leaning to the world's `z`, each side's (their faces'
/// axes through `o` along `a`, ring frames `Frame::new(o, a, ring(a))`).
fn contacts() -> Vec<(&'static str,Contact)> {
    let a = unit([0.42,-0.1,0.9]);
    let (sa,ca) = (0.2f64.sin(),0.2f64.cos());
    let mut out = Vec::new();
    for side in [1.,-1.] {
        out.push(("a drum",Contact {o:[5.,1.2,-0.5],a,r:1.,sa:0.,ca:1.,side}));
        out.push(("a frustum",Contact {o:[4.,-1.5,0.3],a,r:1.3,sa:-sa,ca,side}));
    }
    out
}

/// **A leaning round face's contact under a turn is where its normal is square to the turn's
/// velocity**: at each slant its point lies on the face's ring there and the face's normal
/// `cos α ρ̂ − sin α a` meets `z × x` at nought; its jets agree with central differences of its
/// value; and the interval jet over a stretch holds every sample of the float jet within it.
#[test]
fn a_leaning_rounds_contact_is_square_to_the_turns_velocity() {
    for (what,c) in contacts() {
        for k in 0..=20 {
            let v = -1.+0.1*k as f64;
            let (x,disc) = c.point(Jet::at(v));
            assert!(disc.v > 0.,"{what} at {v}: no contact");
            let x = x.map(|j| j.v);
            let m = add(c.o,scale(c.a,v*c.ca));
            let radial = sub(sub(x,m),scale(c.a,dot(sub(x,m),c.a)));
            assert!(dot(sub(x,m),c.a).abs() <= 1e-12,"{what}: off the ring");
            assert!((norm(radial)-(c.r+v*c.sa)).abs() <= 1e-12,"{what}: off the face");
            let n = sub(scale(unit(radial),c.ca),scale(c.a,c.sa));
            let speed = cross([0.,0.,1.],x);
            assert!(dot(n,speed).abs() <= 1e-12*norm(speed),"{what} at {v}: n · v = {}",dot(n,speed));
            // the meridian's jets against differences
            let t = Turned::meridian(Frame::WORLD,c,[-1.,1.]);
            let h = 1e-5;
            let (p,d,dd) = t.d2(v);
            let (pm,dm,_) = t.d2(v-h);
            let (pp,dp,_) = t.d2(v+h);
            for i in 0..3 {
                assert!((d[i]-(pp[i]-pm[i])/(2.*h)).abs() <= 1e-8,"{what}: M' {i}");
                assert!((dd[i]-(dp[i]-dm[i])/(2.*h)).abs() <= 1e-6,"{what}: M'' {i}");
            }
            assert!(p[1].abs() <= 1e-15 && (p[0]-x[0].hypot(x[1])).abs() <= 1e-12 && (p[2]-x[2]).abs() <= 1e-15);
        }
        let t = Turned::meridian(Frame::WORLD,c,[-1.,1.]);
        let [[r,dr,ddr],[z,dz,ddz]] = t.bounded([-0.3,0.2]).unwrap();
        for k in 0..=50 {
            let v = -0.3+0.5*k as f64/50.;
            let [jr,jz] = c.meridian(Jet::at(v));
            for (b,x) in [(r,jr.v),(dr,jr.d),(ddr,jr.dd),(z,jz.v),(dz,jz.d),(ddz,jz.dd)] {
                assert!(b[0] <= x && x <= b[1],"{what}: {x} outside {b:?}");
            }
        }
        assert!(t.defined([-1.,1.]),"{what}: a curve all along");
    }
}

/// **A plane cutting every generator of a cone meets it in an ellipse, exactly**
/// (`ssi::intersect`): cones opening either way and planes leaning less than their generators,
/// each meeting one ellipse whose points lie on both surfaces to rounding; a plane through the apex,
/// or leaning past a generator, still traced.
#[test]
fn a_plane_cutting_every_generator_of_a_cone_meets_it_in_an_ellipse() {
    use gcs_core::brep::ssi::{Ssi,intersect};
    let mut rng = gcs_core::rng::Rng::new(5);
    let mut ellipses = 0;
    for _ in 0..40 {
        let axis = unit([rng.uniform(-1.,1.),rng.uniform(-1.,1.),rng.uniform(-1.,1.)]);
        let half = rng.uniform(-0.6,0.6);
        if half.abs() < 0.05 { continue }
        let cone = Surface::Cone(Frame::about([rng.uniform(-2.,2.),rng.uniform(-2.,2.),rng.uniform(-2.,2.)],axis),rng.uniform(0.5,2.),half);
        let lean = rng.uniform(0.,1.4);
        let side = Frame::about([0.;3],axis).x;
        let n = unit(add(scale(axis,lean.cos()),scale(side,lean.sin())));
        let plane = Surface::Plane(Frame::about(add(cone.point([0.,rng.uniform(-1.,1.)]),scale(n,rng.uniform(-1.,1.))),n));
        let cuts = lean < std::f64::consts::FRAC_PI_2-half.abs()-1e-3;
        match intersect(&plane,&cone,1e-9) {
            // (the ellipse on the other nappe than the surface's: no meeting)
            Ssi::Curves(c) if cuts && c.is_empty() => {}
            Ssi::Curves(c) if cuts => {
                assert_eq!(c.len(),1);
                let Curve::Ellipse(..) = c[0] else { panic!("{:?}",c[0]) };
                for k in 0..32 {
                    let p = c[0].point(k as f64*TAU/32.);
                    assert!(plane.implicit(p).abs() <= 1e-9 && cone.implicit(p).abs() <= 1e-9,"off: {} {}",plane.implicit(p),cone.implicit(p));
                }
                ellipses += 1;
            }
            Ssi::Traced if !cuts => {}
            other => panic!("leaning {lean} on a cone of {half}: {other:?}"),
        }
    }
    assert!(ellipses > 10,"{ellipses} ellipses");
}

/// A way square to `a`.
fn ring(a: V) -> V { Frame::about([0.;3],a).x }

/// The leaning drum's contact turned, as a surface of revolution about the world's `z`.
fn turned_surface(c: Contact) -> (Turned,Surface) {
    let t = Turned::meridian(Frame::WORLD,c,[-2.,2.]);
    (t,Surface::Revolution(Frame::WORLD,std::sync::Arc::new(Curve::Turned(std::sync::Arc::new(t)))))
}

/// **A line meets a turned contact's surface where sampling finds it** (`query::line_turned`,
/// certified by interval jets): lines through it at many slopes — climbing, steep and level — each
/// read certified, and against a dense sampling of the meridian's radius against the line's at the
/// same height, every crossing found and each certified one on the surface.
#[test]
fn a_line_meets_a_turned_contacts_surface_where_sampling_finds_it() {
    use gcs_core::brep::query::{Meets,curve_surface};
    let (t,s) = turned_surface(contacts()[0].1);
    let mut rng = gcs_core::rng::Rng::new(11);
    let mut met = 0;
    for _ in 0..60 {
        let p: V = [rng.uniform(-7.,7.),rng.uniform(-7.,7.),rng.uniform(-1.5,1.5)];
        let level = rng.uniform(0.,1.) < 0.2;
        let d = unit([rng.uniform(-1.,1.),rng.uniform(-1.,1.),if level { 0. } else { rng.uniform(-1.,1.) }]);
        let line = Curve::Line {p,d};
        let Meets::At(found) = curve_surface(&line,[-20.,20.],&s,1e-9) else { panic!("a line lying in it") };
        for &(at,_) in &found {
            let x = add(p,scale(d,at));
            let foot = s.point(s.inverse(x));
            assert!(norm(sub(foot,x)) <= 1e-7,"a crossing {} off the surface",norm(sub(foot,x)));
        }
        // the sampled crossings: the line's radius against the meridian's at its height
        if level { met += found.len(); continue }
        let n = 20000;
        let gap = |tt: f64| -> Option<f64> {
            let x = add(p,scale(d,tt));
            // the meridian's radius at x's height (it climbs: one slant each height)
            let [a,b] = t.span;
            let z = |v: f64| t.d2(v).0[2];
            let (za,zb) = (z(a),z(b));
            if (x[2]-za)*(x[2]-zb) > 0. { return None }
            let (mut lo,mut hi) = (a,b);
            for _ in 0..80 { let m = 0.5*(lo+hi); if (z(m)-x[2])*(za-x[2]) > 0. { lo = m } else { hi = m } }
            Some(x[0].hypot(x[1])-t.d2(0.5*(lo+hi)).0[0])
        };
        let mut sampled = 0;
        let mut last: Option<f64> = None;
        for k in 0..=n {
            let g = gap(-20.+40.*k as f64/n as f64);
            if let (Some(a),Some(b)) = (last,g) { if a*b < 0. { sampled += 1 } }
            last = g;
        }
        assert_eq!(found.iter().filter(|r| !r.1).count(),sampled,"at {p:?} along {d:?}: {found:?}");
        met += found.len();
    }
    assert!(met > 20,"{met} crossings read");
}

/// **A leaning face at a pose of a turn meets the surface its contact turns into along its contact
/// turned, in closed form** (`ssi`'s `own_contact`): a drum and a frustum leaning to the turn's
/// axis, each turned by an angle and read against the surface of revolution of each of its contact's
/// meridians, meet it in the one curve `Turned::posed` — its points on both surfaces, the face's
/// normal square to the turn's velocity there — and a face turned off its own axis line, or of
/// another radius, is traced as before.
#[test]
fn a_leaning_face_at_a_pose_meets_its_contacts_surface_along_its_contact_turned() {
    use gcs_core::brep::geom::Rigid;
    use gcs_core::brep::ssi::{Ssi,intersect};
    for (what,c) in contacts() {
        let (t,revolution) = turned_surface(c);
        let theta = 1.3;
        let pose = Rigid::turn([0.;3],[0.,0.,1.],theta);
        let f = Frame::new(c.o,c.a,ring(c.a)).moved(&pose);
        let face = if c.sa == 0. { Surface::Cylinder(f,c.r) } else { Surface::Cone(f,c.r,c.sa.atan2(c.ca)) };
        let Ssi::Curves(curves) = intersect(&face,&revolution,1e-9) else { panic!("{what}: traced") };
        assert_eq!(curves.len(),1,"{what}");
        let Curve::Turned(ref posed) = curves[0] else { panic!("{what}: {:?}",curves[0]) };
        let turned = posed.posed.expect("posed");
        assert!(((turned-theta).rem_euclid(TAU)).min(TAU-(turned-theta).rem_euclid(TAU)) <= 1e-12,"{what}: posed at {turned}");
        for k in 0..=16 {
            let v = t.span[0]+(t.span[1]-t.span[0])*k as f64/16.;
            let p = curves[0].point(v);
            assert!(face.implicit(p).abs() <= 1e-9,"{what}: off the face by {}",face.implicit(p));
            assert!(norm(sub(revolution.point(revolution.inverse(p)),p)) <= 1e-9,"{what}: off the surface");
            let n = face.gradient(p);
            assert!(dot(unit(n),unit(cross([0.,0.,1.],p))).abs() <= 1e-9,"{what}: no contact");
        }
        // another radius: no contact of this face's, traced
        let other = if c.sa == 0. { Surface::Cylinder(f,c.r*1.1) } else { Surface::Cone(f,c.r*1.1,c.sa.atan2(c.ca)) };
        assert!(!matches!(intersect(&other,&revolution,1e-9),Ssi::Curves(ref k) if k.iter().any(|c| matches!(c,Curve::Turned(_)))),"{what}");
    }
}

/// **A circle and an ellipse in one plane cross where the ellipse meets the circle's cylinder,
/// certified** (`query::curve_curve`): an ellipse tangent to a circle from outside and from inside
/// is one crossing at the point they touch, an ellipse crossing it four or two, one clear of it none.
#[test]
fn a_circle_and_an_ellipse_in_one_plane_cross_where_certified() {
    use gcs_core::brep::query::curve_curve;
    let circle = Curve::Circle(Frame::WORLD,2.);
    let at = |x: f64,a: f64,b: f64| Curve::Ellipse(Frame {o:[x,0.,0.],..Frame::WORLD},a,b);
    let all = [0.,TAU];
    for (what,ellipse,count,touch) in [("touching outside",at(3.,1.,0.5),1,Some([2.,0.,0.])),("touching inside",at(1.,1.,0.4),1,Some([2.,0.,0.])),
        ("crossing four times",at(0.,2.5,1.5),4,None),("crossing twice",at(2.,1.,0.5),2,None),("clear",at(5.,1.,0.5),0,None)] {
        let found = curve_curve(&circle,all,&ellipse,all,1e-9).expect(what);
        assert_eq!(found.len(),count,"{what}: {found:?}");
        if let Some(q) = touch { assert!(norm(sub(found[0].1,q)) <= 1e-7,"{what}: {:?}",found[0].1) }
        let back = curve_curve(&ellipse,all,&circle,all,1e-9).expect(what);
        assert_eq!(back.len(),count,"{what} read the other way");
    }
}

/// A leaning drum: radius 1 about the axis through `(5, 1.2, 0)` along `a`, cut by the level planes
/// `z = ±1.5` — its rims there ellipses in level planes.
fn leaning_drum(a: V) -> Brep {
    let long = prism(&disc(sub([5.,1.2,0.],scale(a,3.)),a,1.),0.,6.).unwrap();
    gcs_core::brep::boolean::combined(&long,&block([-10.,-10.,-1.5],[10.,10.,1.5]),gcs_core::brep::boolean::Op::Common,1e-9).unwrap()
}

/// The radii the level slice at height `z` of the leaning drum spans about the world's `z` axis:
/// its ellipse, centre on the drum's axis at that height, its radii `1 / cos β` along the axis's
/// shadow and 1 across it, read at 720 angles and the nearest and farthest polished by Newton on
/// the radius's rate.
fn drum_spans(a: V,z: f64) -> (f64,f64) {
    let centre = add([5.,1.2,0.],scale(a,z/a[2]));
    let shadow = unit([a[0],a[1],0.]);
    let across = [-shadow[1],shadow[0],0.];
    let big = 1./a[2];
    let at = |phi: f64| add(centre,add(scale(shadow,big*phi.cos()),scale(across,phi.sin())));
    let r2 = |phi: f64| { let p = at(phi); p[0]*p[0]+p[1]*p[1] };
    let polish = |mut phi: f64| {
        for _ in 0..40 {
            let h = 1e-4;
            let (d1,d2) = ((r2(phi+h)-r2(phi-h))/(2.*h),(r2(phi+h)-2.*r2(phi)+r2(phi-h))/(h*h));
            if d2 == 0. { break }
            phi -= d1/d2;
        }
        r2(phi).sqrt()
    };
    let samples: Vec<f64> = (0..720).map(|k| k as f64*TAU/720.).collect();
    let near = samples.iter().copied().min_by(|x,y| r2(*x).total_cmp(&r2(*y))).unwrap();
    let far = samples.iter().copied().max_by(|x,y| r2(*x).total_cmp(&r2(*y))).unwrap();
    (polish(near),polish(far))
}

/// `built` read against membership at seeded points of its box.
fn agrees(built: &Brep,tool: &Brep,path: &Trajectory) {
    let (solid,body) = (Solid::new(tool,0.002,1.).unwrap(),Solid::new(built,0.002,1.).unwrap());
    let swept = Swept {tool:&solid,path,roll:[0.,1.]};
    let (lo,hi) = built.bounds();
    let mut rng = gcs_core::rng::Rng::new(6);
    let mut held = 0;
    for _ in 0..200 {
        let p: V = std::array::from_fn(|k| rng.uniform(lo[k]-0.2,hi[k]+0.2));
        let d = body.signed_distance(p);
        if d.abs() < 0.01 { continue }
        let Ok(member) = swept.member(p) else { continue };
        assert_eq!(member,d < 0.,"at {p:?}: membership {member}, the built solid's distance {d}");
        held += 1;
    }
    assert!(held > 150,"{held} points held");
}

/// **A Monte Carlo of the sweep's definition**, apart from the kernel's membership: the swept set
/// is the tool's points turned by every angle of the roll, so a point is in it where turned back by
/// some angle in `[0, angle]` it is in the tool — read off `inside` (the tool's own analytic test,
/// below nought within it) at every quarter of a degree of the roll. `n` points seeded over `built`'s
/// box: the volume they say and its standard error.
fn monte_carlo(built: &Brep,inside: &dyn Fn(V) -> f64,angle: f64,n: usize) -> (f64,f64) {
    let (lo,hi) = built.bounds();
    let (lo,hi): (V,V) = (lo.map(|x| x-0.1),hi.map(|x| x+0.1));
    let steps = (angle.to_degrees()*4.).ceil() as usize;
    let mut rng = gcs_core::rng::Rng::new(29);
    let mut hits = 0;
    for _ in 0..n {
        let p: V = std::array::from_fn(|k| rng.uniform(lo[k],hi[k]));
        let held = (0..=steps).any(|k| {
            let (s,c) = (-angle*k as f64/steps as f64).sin_cos();
            inside([c*p[0]-s*p[1],s*p[0]+c*p[1],p[2]]) <= 0.
        });
        if held { hits += 1 }
    }
    let boxed = (0..3).map(|k| hi[k]-lo[k]).product::<f64>();
    let f = hits as f64/n as f64;
    (boxed*f,boxed*(f*(1.-f)/n as f64).sqrt())
}

/// The part of a turn the gates turn their tools through besides the whole, 100°.
const PART: f64 = 100.*PI/180.;

/// **`tool` turned whole and part of a turn**, each built (`closed`) and held: turned whole, to the
/// volume of its slices' radial spans `whole`, worked apart from the build; turned `PART`, where
/// `convex` (each circle about the axis meets its slices in one arc, at most `2π − PART` wide), to
/// its own volume and `PART / 2π` of `whole` — each slice sweeps itself and, round each circle, its
/// arc carried the turn on, `∫ ρ (w(ρ) + Θ) dρ`, its area and `Θ (ρ_max² − ρ_min²) / 2` — and else
/// refused by name (`Unbuilt::Unsettled`, never built wrong); each built one to membership and to a
/// Monte Carlo of the sweep's definition (`monte_carlo`) within four standard errors.
fn turned_both_ways(what: &str,tool: &Brep,inside: &dyn Fn(V) -> f64,whole: f64,convex: bool) {
    turned_whole(what,tool,inside,whole);
    turned_part(what,tool,inside,whole,convex);
}

/// `turned_both_ways`'s whole turn.
fn turned_whole(what: &str,tool: &Brep,inside: &dyn Fn(V) -> f64,whole: f64) {
    let path = turn(TAU);
    let built = closed(tool,&path,[0.,1.],0.002,1e-6).unwrap_or_else(|e| panic!("{what} turned whole: {e:?}"));
    assert!((volume(&built)-whole).abs() <= 1e-9*whole,"{what} turned whole: {} against {whole}",volume(&built));
    agrees(&built,tool,&path);
    let (sampled,error) = monte_carlo(&built,inside,TAU,20000);
    assert!((sampled-whole).abs() <= 4.*error,"{what} turned whole: sampled {sampled} ± {error} against {whole}");
}

/// `turned_both_ways`'s part of a turn.
fn turned_part(what: &str,tool: &Brep,inside: &dyn Fn(V) -> f64,whole: f64,convex: bool) {
    use gcs_core::brep::swept::closed::Unbuilt;
    let path = turn(PART);
    match closed(tool,&path,[0.,1.],0.002,1e-6) {
        Ok(built) => {
            agrees(&built,tool,&path);
            let (sampled,error) = monte_carlo(&built,inside,PART,20000);
            assert!((sampled-volume(&built)).abs() <= 4.*error,"{what} turned {PART}: sampled {sampled} ± {error} against {}",volume(&built));
            if convex {
                let expected = volume(tool)+PART/TAU*whole;
                assert!((volume(&built)-expected).abs() <= 1e-9*expected,"{what} turned {PART}: {} against {expected}",volume(&built));
            }
        }
        Err(Unbuilt::Unsettled {..}) if !convex => {}
        Err(e) => panic!("{what} turned {PART}: {e:?}"),
    }
}

/// **A leaning drum turned closes into its sweep** (#780 C3's gate, a tilted cylinder turned): a
/// drum of radius 1 leaning 25° from the turn's axis and 1.2 beside the plane through it, cut by two
/// level planes. Its side touches along two curves (`A cos u + B sin u = 0` on each ring), each
/// turned into a surface of revolution held exactly; its rims, ellipses in level planes, turn within
/// their planes; turned part of a turn, its side at each end pose touches those surfaces along its
/// contact turned there (`ssi`'s closed form). Turned whole, its volume is `∫ π (ρ_max² − ρ_min²)
/// dz` over the slices' radial spans, each slice an ellipse worked apart from the build
/// (`drum_spans`), by Simpson's rule; part of a turn, its own and `Θ/2π` of that
/// (`turned_both_ways`).
#[test]
fn a_leaning_drum_turned_closes_into_its_sweep() {
    let beta = 25f64.to_radians();
    let a = [beta.sin(),0.,beta.cos()];
    let tool = leaning_drum(a);
    let whole = simpson(-1.5,1.5,400,|z| { let (n,f) = drum_spans(a,z); PI*(f*f-n*n) });
    let o = [5.,1.2,0.];
    let inside = |p: V| { let q = sub(p,o); let t = dot(q,a); (norm(sub(q,scale(a,t)))-1.).max(t.abs()-3.).max(p[2].abs()-1.5) };
    turned_both_ways("a leaning drum",&tool,&inside,whole,true);
}

/// The radii the level slice at height `z` of a convex tool spans about the world's `z` axis, apart
/// from any build: the tool `{p : inside(p) ≤ 0}` (`inside` convex), its slice's boundary found
/// along rays from `centre` (a point of the slice) by bisection, at 120 angles, the nearest and
/// farthest polished by golden sections. The slice is held clear of the axis.
fn convex_spans(inside: &dyn Fn(V) -> f64,centre: [f64;2],z: f64) -> (f64,f64) {
    let edge = |phi: f64| -> [f64;2] {
        let u = [phi.cos(),phi.sin()];
        let (mut lo,mut hi) = (0.,1.);
        while inside([centre[0]+hi*u[0],centre[1]+hi*u[1],z]) <= 0. { hi *= 2. }
        for _ in 0..56 { let m = 0.5*(lo+hi); if inside([centre[0]+m*u[0],centre[1]+m*u[1],z]) <= 0. { lo = m } else { hi = m } }
        [centre[0]+lo*u[0],centre[1]+lo*u[1]]
    };
    let rho = |phi: f64| { let p = edge(phi); p[0].hypot(p[1]) };
    let golden = |k: usize,sense: f64| -> f64 {
        let step = TAU/120.;
        let (mut a,mut b) = ((k as f64-1.)*step,(k as f64+1.)*step);
        let g = 0.5*(5f64.sqrt()-1.);
        for _ in 0..56 {
            let (c,d) = (b-g*(b-a),a+g*(b-a));
            if sense*rho(c) > sense*rho(d) { b = d } else { a = c }
        }
        rho(0.5*(a+b))
    };
    let samples: Vec<f64> = (0..120).map(|k| rho(k as f64*TAU/120.)).collect();
    let (near,far) = (0..120).fold((0,0),|(n,f),k| (if samples[k] < samples[n] { k } else { n },if samples[k] > samples[f] { k } else { f }));
    assert!(inside([0.,0.,z]) > 0.,"the slice at {z} holds the axis");
    (golden(near,-1.),golden(far,1.))
}

/// The radius of `ball_on_a_stem`'s stem.
const STEM: f64 = 0.6;

/// A ball of radius 1 on a stem of radius `STEM`, its axis leaning `beta` from the world's `z` in
/// the plane `y = 1.2` and the ball's centre at `(5, 1.2, −1)`, the stem cut off by the level plane
/// `z = 1.5` (a rim there, an ellipse): the ball, the stem's side, its level top, and the circle
/// where the stem enters the ball (concave, bounding nothing of the sweep).
fn ball_on_a_stem(beta: f64) -> (Brep,V,V) {
    use gcs_core::brep::build::{Profile,ProfileEdge,revolve};
    use gcs_core::brep::geom::Rigid;
    let f = Frame::about([0.;3],[0.,0.,1.]);
    // the ball's meridian from its lowest point up to where the stem leaves it, `√(1 − STEM²)` up
    let seam = (1.-STEM*STEM).sqrt();
    let profile = Profile {origin:f.o,normal:f.y,names:Vec::new(),loops:vec![vec![
        // (the arc's angle from the axis up: `STEM` out at the seam, the lowest point at π)
        ProfileEdge::Arc {frame:Frame::new(f.o,f.y,f.z),r:1.,span:Some([STEM.atan2(seam),std::f64::consts::PI])},
        ProfileEdge::Line {a:f.at([STEM,0.,seam]),b:f.at([STEM,0.,5.])},
        ProfileEdge::Line {a:f.at([STEM,0.,5.]),b:f.at([0.,0.,5.])},ProfileEdge::Line {a:f.at([0.,0.,5.]),b:f.at([0.,0.,-1.])}]]};
    let m = Rigid::turn([0.;3],[0.,1.,0.],beta).then(&Rigid {t:[5.,1.2,-1.],..Rigid::identity()});
    let stem = revolve(&profile,f.o,f.z,TAU).unwrap().moved(&m);
    let tool = gcs_core::brep::boolean::combined(&stem,&block([-10.,-10.,-10.],[10.,10.,1.5]),gcs_core::brep::boolean::Op::Common,1e-9).unwrap();
    (tool,m.point([0.;3]),m.vector([0.,0.,1.]))
}

/// **A ball tilted on a stem, off the turn's axis, turned closes into its sweep** (#780 C3's gate):
/// the ball's contact is its great circle in the plane through the axis and its centre, turned into
/// a torus; the stem's, a drum of radius 0.6 leaning 25° and 1.2 beside the plane through the axis,
/// two curves along it turned into surfaces of revolution held exactly; its level rim turns within
/// its plane; the circle where the stem enters the ball bounds nothing. Turned whole, its volume is
/// `∫ π (ρ_max² − ρ_min²) dz` over the slices of the tool — the ball's a disc, the stem's read apart
/// from the build (`convex_spans`), their radial spans united (two apart near the ball's top, where
/// the stem has leant out of the ball's slice) — by Simpson's rule in `w`, `z = z₀ + w²` from the
/// ball's lowest point (where the span grows as a square root), cut where the spans' ends cross.
/// Held to it and to membership and a Monte Carlo of the definition (`turned_whole`).
#[test]
fn a_ball_tilted_on_a_stem_turned_whole_closes_into_its_solid_of_revolution() {
    let beta = 25f64.to_radians();
    let (tool,c,a) = ball_on_a_stem(beta);
    // the ball's slice a disc; the stem's, from its axis's point at the height, a convex slice of
    // the drum `t ≥ 0` (where that point is below the ball's centre, all the stem's slice is in
    // the ball's): the union's radial spans the two spans united — two apart near the ball's top,
    // where the stem has leant out of the ball's slice
    let stem = |p: V| { let w = sub(p,c); let t = dot(w,a); (norm(sub(w,scale(a,t)))-STEM).max(-t).max(p[2]-1.5) };
    let parts = |z: f64| -> Vec<(f64,f64)> {
        let h = z-c[2];
        let mut spans: Vec<(f64,f64)> = Vec::new();
        if h.abs() < 1. { let (s,d) = ((1.-h*h).sqrt(),c[0].hypot(c[1])); spans.push((d-s,d+s)) }
        let t = h/a[2];
        if t > 0. && z <= 1.5 {
            let q = add(c,scale(a,t));
            spans.push(convex_spans(&stem,[q[0],q[1]],z));
        }
        spans
    };
    let area = |z: f64| -> f64 {
        let mut spans = parts(z);
        spans.sort_by(|x,y| x.0.total_cmp(&y.0));
        let mut total = 0.;
        let mut reach = f64::NEG_INFINITY;
        for (n,f) in spans {
            let n = n.max(reach);
            if f > n { total += PI*(f*f-n*n) }
            reach = reach.max(f);
        }
        total
    };
    let z0 = c[2]-1.;
    let w1 = (1.5-z0).sqrt();
    // (Simpson's rule in `w`, `z = z₀ + w²`, between the heights where the ball's top and the
    // stem's first slice part the spans' count: the ball's top `c_z + 1`, the stem's axis leaving
    // the ball at `c_z + a_z`, and the stem's lowest slice at its axis's foot)
    let mut cuts: Vec<f64> = vec![0.,(c[2]-z0).sqrt(),(c[2]+a[2]-z0).sqrt(),(c[2]+1.-z0).sqrt(),w1];
    cuts.sort_by(f64::total_cmp);
    // and where the ball's span and the stem's cross ends — their union's ends changing hands, or
    // parting — each found where the two ends' difference changes sign, by bisection
    let ends = |z: f64| -> Option<[f64;4]> { let p = parts(z); (p.len() == 2).then(|| [p[0].0,p[0].1,p[1].0,p[1].1]) };
    let (zl,zh) = (c[2]+1e-9,(c[2]+1.).min(1.5)-1e-9);
    let mut more = Vec::new();
    for (i,j) in [(0,2),(1,3),(1,2),(0,3)] {
        let d = |z: f64| ends(z).map_or(f64::NAN,|e| e[i]-e[j]);
        let zs: Vec<f64> = (0..=400).map(|k| zl+(zh-zl)*k as f64/400.).collect();
        for q in zs.windows(2) {
            let (a0,b0) = (d(q[0]),d(q[1]));
            if !(a0*b0 < 0.) { continue }
            let (mut lo,mut hi) = (q[0],q[1]);
            for _ in 0..80 { let m = 0.5*(lo+hi); if d(m)*a0 > 0. { lo = m } else { hi = m } }
            more.push((0.5*(lo+hi)-z0).sqrt());
        }
    }
    cuts.extend(more);
    cuts.sort_by(f64::total_cmp);
    let whole: f64 = cuts.windows(2).map(|q| simpson(q[0],q[1],300,|w| { let z = z0+w*w; if w == 0. { return 0. } area(z)*2.*w })).sum();
    turned_whole("a ball on a stem",&tool,&|p| stemmed(c,a,p),whole);
}

/// `ball_on_a_stem`'s own test, below nought within it: within 1 of the ball's centre `c`, or of its
/// stem's axis along `a` above it within `STEM`, and below the level cut.
fn stemmed(c: V,a: V,p: V) -> f64 {
    let w = sub(p,c);
    let t = dot(w,a);
    (norm(w)-1.).min((norm(sub(w,scale(a,t)))-STEM).max(-t)).max(p[2]-1.5)
}

/// **A ball tilted on a stem turned part of a turn is cut right or refused by name**: turned 100°,
/// its stem's side at each end pose touches its contact's surfaces of revolution along the contact
/// turned there, and a cell at the concave seam where the stem enters the ball is narrower than the
/// mesh's sag can settle: refused (`Unbuilt::Unsettled`), never built wrong — built, held to
/// membership and a Monte Carlo of the definition (`turned_part`).
#[test]
fn a_ball_tilted_on_a_stem_turned_part_of_a_turn_is_cut_right_or_refused_by_name() {
    let beta = 25f64.to_radians();
    let (tool,c,a) = ball_on_a_stem(beta);
    turned_part("a ball on a stem",&tool,&|p| stemmed(c,a,p),0.,false);
}

/// **A frustum leaning to the turn's axis turned closes into its sweep**: a frustum of radii 1.2 and
/// 0.7, 6 along its axis, leaning 25° from the turn's and 1.2 beside the plane through it, cut by the
/// level planes `z = ±1.5` (its rims ellipses: a plane cutting every generator of a cone, in closed
/// form). Its side touches where `A cos u + B sin u = C` on each ring — `C` no longer nought, its
/// half-angle's — along two curves, each turned into a surface of revolution held exactly. Turned
/// whole, its volume `∫ π (ρ_max² − ρ_min²) dz`, its slices read apart from the build
/// (`convex_spans`) by Simpson's rule; part of a turn, its own and `Θ/2π` of that
/// (`turned_both_ways`).
#[test]
fn a_leaning_frustum_turned_closes_into_its_sweep() {
    let beta = 25f64.to_radians();
    let a = [beta.sin(),0.,beta.cos()];
    let base = sub([5.,1.2,0.],scale(a,3.));
    let (r0,r1,h) = (1.2,0.7,6.);
    let long = gcs_core::brep::build::cone(base,a,r0,r1,h).unwrap();
    let tool = gcs_core::brep::boolean::combined(&long,&block([-10.,-10.,-1.5],[10.,10.,1.5]),gcs_core::brep::boolean::Op::Common,1e-9).unwrap();
    let inside = |p: V| {
        let w = sub(p,base);
        let t = dot(w,a);
        let off = norm(sub(w,scale(a,t)));
        (off-(r0+(r1-r0)*t/h)).max(-t).max(t-h).max(p[2].abs()-1.5)
    };
    let centre = |z: f64| { let q = add([5.,1.2,0.],scale(a,z/a[2])); [q[0],q[1]] };
    let whole = simpson(-1.5,1.5,1000,|z| { let (n,f) = convex_spans(&inside,centre(z),z); PI*(f*f-n*n) });
    turned_both_ways("a leaning frustum",&tool,&inside,whole,true);
}

/// A rounded key leaning to the world's `z`: the stadium of radius `KEY[0]` about the segment of
/// half-length `KEY[1]` across its axis, square to the unit `a` through `(5, 1.2, 0)`, drawn along
/// `a` 3 each way and cut by the level planes `z = ±1.5` — two leaning half drums on its ends,
/// two leaning flats running on smoothly into them, its rims there in level planes. With the
/// stadium's own way across, `w`.
fn rounded_key(a: V) -> (Brep,V) {
    use gcs_core::brep::build::{Profile,ProfileEdge};
    let [r,l] = KEY;
    let c = sub([5.,1.2,0.],scale(a,3.));
    let w = unit(cross(a,[0.,1.,0.]));
    let v = cross(a,w);
    let (c1,c2) = (add(c,scale(w,l)),sub(c,scale(w,l)));
    let half = std::f64::consts::FRAC_PI_2;
    let profile = Profile {origin:c,normal:a,names:Vec::new(),loops:vec![vec![
        ProfileEdge::Arc {frame:Frame::new(c1,a,w),r,span:Some([-half,half])},
        ProfileEdge::Line {a:add(c1,scale(v,r)),b:add(c2,scale(v,r))},
        ProfileEdge::Arc {frame:Frame::new(c2,a,w),r,span:Some([half,3.*half])},
        ProfileEdge::Line {a:sub(c2,scale(v,r)),b:sub(c1,scale(v,r))}]]};
    let long = prism(&profile,0.,6.).unwrap();
    (gcs_core::brep::boolean::combined(&long,&block([-10.,-10.,-1.5],[10.,10.,1.5]),gcs_core::brep::boolean::Op::Common,1e-9).unwrap(),w)
}

/// `rounded_key`'s radius and its segment's half-length.
const KEY: [f64;2] = [0.5,0.6];

/// **A rounded key leaning to the turn's axis turned closes into its sweep** (#780 C3's gate, a
/// tilted cylinder turned): its ends' half drums touch along curves turned into surfaces of
/// revolution held exactly; its flats, leaning planes, along lines turned into hyperboloids; the
/// lines where its flats run into its ends bound nothing; its rims turn within their level planes.
/// Turned whole, its volume `∫ π (ρ_max² − ρ_min²) dz`, its slices — the points within `KEY[0]` of
/// the segment across it in each plane square to its axis, between the level cuts — read apart from
/// the build (`convex_spans`) by Simpson's rule (`turned_whole`).
#[test]
fn a_rounded_key_leaning_to_the_axis_turned_whole_closes_into_its_solid_of_revolution() {
    let (tool,inside,whole) = keyed();
    turned_whole("a rounded key",&tool,&inside,whole);
}

/// **A rounded key leaning to the turn's axis turned part of a turn closes into its sweep**: turned
/// 100°, its own volume and `Θ/2π` of its whole turn's, membership and a Monte Carlo of the
/// definition (`turned_part`).
#[test]
#[cfg_attr(not(feature = "slow"),ignore = "slow tier, about 11 s: a rounded key turned 100°, its membership read at 200 points and sampled")]
fn a_rounded_key_leaning_to_the_axis_turned_part_of_a_turn_closes_into_its_sweep() {
    let (tool,inside,whole) = keyed();
    turned_part("a rounded key",&tool,&inside,whole,true);
}

/// `rounded_key` leaning 25°, its own test (below nought within it), and the volume it sweeps
/// turned whole, worked apart from any build.
fn keyed() -> (Brep,impl Fn(V) -> f64,f64) {
    let beta = 25f64.to_radians();
    let a = [beta.sin(),0.,beta.cos()];
    let (tool,w) = rounded_key(a);
    let o = [5.,1.2,0.];
    let inside = move |p: V| {
        let q = sub(p,o);
        let t = dot(q,a);
        let flat = sub(q,scale(a,t));
        let along = dot(flat,w).clamp(-KEY[1],KEY[1]);
        (norm(sub(flat,scale(w,along)))-KEY[0]).max(t.abs()-3.).max(p[2].abs()-1.5)
    };
    let centre = |z: f64| { let q = add(o,scale(a,z/a[2])); [q[0],q[1]] };
    let whole = simpson(-1.5,1.5,1000,|z| { let (n,f) = convex_spans(&inside,centre(z),z); PI*(f*f-n*n) });
    (tool,inside,whole)
}
