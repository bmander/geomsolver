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
use gcs_core::brep::topo::Brep;
use gcs_core::brep::trajectory::Trajectory;
use gcs_core::brep::turned::{Contact,Exact,Jet,Turned};
use gcs_core::space::{add,cross,dot,norm,scale,sub,unit};
use std::f64::consts::{PI,TAU};

type V = [f64;3];

/// The turn by `angle` about the world's `z` axis over the roll `[0, 1]`.
fn turn(angle: f64) -> Trajectory { crate::swept_closed::turn([0.;3],[0.,0.,1.],angle) }

/// The way the gates' tools lean: 25° from the turn's axis, in the plane `y = 0`.
fn leaning() -> V { let beta = 25f64.to_radians(); [beta.sin(),0.,beta.cos()] }

/// `long` cut between the level planes `z = bottom` and `z = 1.5`.
fn level_cut(long: &Brep,bottom: f64) -> Brep {
    gcs_core::brep::boolean::combined(long,&block([-10.,-10.,bottom],[10.,10.,1.5]),gcs_core::brep::boolean::Op::Common,1e-9).unwrap()
}

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
            let v = gcs_core::roots::bracketed_root(|v| Some(z(v)-x[2]),a,za-x[2],b,zb-x[2],1e-15);
            Some(x[0].hypot(x[1])-t.d2(v).0[0])
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
        assert!(posed.posed,"{what}: posed");
        let turned = posed.frame.x[1].atan2(posed.frame.x[0]);
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
fn leaning_drum(a: V) -> Brep { level_cut(&prism(&disc(sub([5.,1.2,0.],scale(a,3.)),a,1.),0.,6.).unwrap(),-1.5) }

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

/// **A Monte Carlo of the sweep's definition**, apart from the kernel's membership: the swept set
/// is the tool's points turned by every angle of the roll, so a point is in it where turned back by
/// some angle in `[0, angle]` it is in the tool — read off `inside` (the tool's own analytic test,
/// below nought within it) at every quarter of a degree of the roll. `n` points seeded over `built`'s
/// box: the volume they say and its standard error.
fn monte_carlo(built: &Brep,inside: &dyn Fn(V) -> f64,angle: f64,n: usize) -> (f64,f64) {
    let (lo,hi) = built.bounds();
    let steps = (angle.to_degrees()*4.).ceil() as usize;
    let back: Vec<gcs_core::brep::geom::Rigid> = (0..=steps).map(|k| gcs_core::brep::geom::Rigid::turn([0.;3],[0.,0.,1.],-angle*k as f64/steps as f64)).collect();
    crate::swept_closed::monte_carlo(lo.map(|x| x-0.1),hi.map(|x| x+0.1),n,29,&|p| back.iter().any(|m| inside(m.point(p)) <= 0.))
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
    crate::swept_closed::agrees(&built,tool,&path);
    let (sampled,error) = monte_carlo(&built,inside,TAU,20000);
    assert!((sampled-whole).abs() <= 4.*error,"{what} turned whole: sampled {sampled} ± {error} against {whole}");
}

/// `turned_both_ways`'s part of a turn.
fn turned_part(what: &str,tool: &Brep,inside: &dyn Fn(V) -> f64,whole: f64,convex: bool) {
    use gcs_core::brep::swept::closed::Unbuilt;
    let path = turn(PART);
    match closed(tool,&path,[0.,1.],0.002,1e-6) {
        Ok(built) => {
            crate::swept_closed::agrees(&built,tool,&path);
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
    let a = leaning();
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
        // (by bisection: `inside` is a most of several, kinked where they change hands)
        let along = |m: f64| inside([centre[0]+m*u[0],centre[1]+m*u[1],z]);
        let (mut lo,mut hi) = (0.,1.);
        while along(hi) <= 0. { hi *= 2. }
        for _ in 0..56 { let m = 0.5*(lo+hi); if along(m) <= 0. { lo = m } else { hi = m } }
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
    let tool = level_cut(&stem,-10.);
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
    let a = leaning();
    let base = sub([5.,1.2,0.],scale(a,3.));
    let (r0,r1,h) = (1.2,0.7,6.);
    let long = gcs_core::brep::build::cone(base,a,r0,r1,h).unwrap();
    let tool = level_cut(&long,-1.5);
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
    (level_cut(&long,-1.5),w)
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
    let a = leaning();
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

/// A flat-ended drum: radius 1 about the axis through `(5, 1.2, 0)` along `a`, 1.5 each way, its
/// ends square to its axis — its rims sharp circles leaning to the turn's axis (#780 L2).
fn flat_drum(a: V) -> Brep { prism(&disc(sub([5.,1.2,0.],scale(a,1.5)),a,1.),0.,3.).unwrap() }

/// `flat_drum`'s own test, below nought within it.
fn in_flat_drum(a: V,p: V) -> f64 {
    let q = sub(p,[5.,1.2,0.]);
    let t = dot(q,a);
    (norm(sub(q,scale(a,t)))-1.).max(t.abs()-1.5)
}

/// The slice at height `z` of the cylinder of radius 1 about the axis through `(5, 1.2, 0)` along
/// `a` (`a_y = 0`): the ellipse `E(φ) = m + (cos φ / cos β) s + sin φ w` about the axis's point `m`
/// at the height (`s` the axis's shadow, `w` across it), and the angles where `ρ²` along it is least
/// or most — the roots of its rate, closed form in `φ`, bracketed between 3600 samples and settled
/// (`roots::bracketed_root`).
fn slice_ellipse(a: V,z: f64) -> (impl Fn(f64) -> V,Vec<f64>) {
    let ca = a[2];
    let m = add([5.,1.2,0.],scale(a,z/ca));
    let (s,w) = (unit([a[0],a[1],0.]),[-a[1]/a[0].hypot(a[1]),a[0]/a[0].hypot(a[1]),0.]);
    let e = move |phi: f64| add(m,add(scale(s,phi.cos()/ca),scale(w,phi.sin())));
    let rate = |phi: f64| { let p = e(phi); let d = add(scale(s,-phi.sin()/ca),scale(w,phi.cos())); 2.*(p[0]*d[0]+p[1]*d[1]) };
    let n = 3600;
    let critical = (0..n).filter_map(|k| {
        let (p0,p1) = (k as f64*TAU/n as f64,(k+1) as f64*TAU/n as f64);
        let (f0,f1) = (rate(p0),rate(p1));
        (f0*f1 <= 0. && f0 != f1).then(|| gcs_core::roots::bracketed_root(|x| Some(rate(x)),p0,f0,p1,f1,1e-15))
    }).collect();
    (e,critical)
}

/// The radii squared the level slice at height `z` of `flat_drum` (leaning along `a` in the plane
/// `y = 1.2`, `a_y = 0`) spans about the world's `z`, in closed form apart from any build. The
/// slice of its side's cylinder is the ellipse `E(φ) = m + (cos φ / cos β) s + sin φ w` about its
/// axis's point `m` at the height (`s` the axis's shadow, `w` across it), whose points stand at
/// `t = z / cos β + tan β cos φ` along the axis: the slice is the ellipse's region where `|t| ≤ 1.5`,
/// arcs where `cos φ` lies between the ends' values, closed by chords along the end planes. `ρ²` is
/// convex, so it is greatest on an arc (where its rate round the ellipse is nought, found by
/// Newton from 3600 samples) or at a corner (`cos φ` an end's value), and least there or at a
/// chord's foot.
fn flat_drum_spans(a: V,z: f64) -> (f64,f64) {
    let (ca,ta) = (a[2],a[0]/a[2]);
    let (e,critical) = slice_ellipse(a,z);
    let r2 = |phi: f64| { let p = e(phi); p[0]*p[0]+p[1]*p[1] };
    // the ellipse's points within the ends: `cos φ` within `[lo, hi]`
    let (lo,hi) = ((-1.5-z/ca)/ta,(1.5-z/ca)/ta);
    let held = |phi: f64| { let c = phi.cos(); c >= lo-1e-15 && c <= hi+1e-15 };
    // the arcs' critical points
    let mut values: Vec<f64> = critical.into_iter().filter(|&phi| held(phi)).map(r2).collect();
    // the corners, and each chord's foot where it falls between its corners
    for c in [lo,hi] {
        if !(c > -1. && c < 1.) { continue }
        let phi = c.acos();
        let (p,q) = (e(phi),e(-phi));
        values.extend([r2(phi),r2(-phi)]);
        let d = sub(q,p);
        let f = -(p[0]*d[0]+p[1]*d[1])/(d[0]*d[0]+d[1]*d[1]);
        if f > 0. && f < 1. { let x = add(p,scale(d,f)); values.push(x[0]*x[0]+x[1]*x[1]) }
    }
    let near = values.iter().copied().fold(f64::INFINITY,f64::min);
    let far = values.iter().copied().fold(f64::NEG_INFINITY,f64::max);
    (near,far)
}

/// **The volume `flat_drum` sweeps turned whole about the world's `z`, worked apart from any
/// build**: `∫ π (ρ_max² − ρ_min²) dz` over its level slices, each convex and clear of the axis, its
/// radii in closed form (`flat_drum_spans`). The integrand is cut where its form changes — the
/// drum's lowest and highest points and where each rim begins and ends to cut the slices
/// (`±1.5 cos β ± sin β`), and where each rim's points are where the side's contact reaches it (the
/// rim's points whose side normal is square to the turn's velocity, found by bisection), where the
/// farthest or nearest point of a slice passes from its side onto its corner — and each piece read
/// by Simpson's rule under `z = z₀ + (z₁ − z₀)(1 − cos πs)/2`, which takes the square-root growth at
/// a slice's first and last height out of the integrand.
pub(crate) fn flat_drum_whole(a: V) -> f64 {
    let o = [5.,1.2,0.];
    let up = unit(sub([0.,0.,1.],scale(a,a[2])));
    let (e1,e2) = (unit(cross(a,[0.,1.,0.])),[0.,1.,0.]);
    let mut cuts = vec![];
    for side in [1.,-1.] {
        let c = add(o,scale(a,1.5*side));
        cuts.extend([c[2]-up[2],c[2]+up[2]]);
        // the rim's points where the side's normal is square to the turn's velocity
        let g = |phi: f64| { let n = add(scale(e1,phi.cos()),scale(e2,phi.sin())); dot(n,cross([0.,0.,1.],add(c,n))) };
        let at = |phi: f64| add(c,add(scale(e1,phi.cos()),scale(e2,phi.sin())))[2];
        for k in 0..720 {
            let (lo,hi) = (k as f64*TAU/720.,(k+1) as f64*TAU/720.);
            let (glo,ghi) = (g(lo),g(hi));
            if glo*ghi > 0. || glo == ghi { continue }
            cuts.push(at(gcs_core::roots::bracketed_root(|x| Some(g(x)),lo,glo,hi,ghi,1e-15)));
        }
    }
    cuts.sort_by(f64::total_cmp);
    cuts.windows(2).map(|q| {
        let (z0,z1) = (q[0],q[1]);
        simpson(0.,1.,400,|s| {
            let z = z0+(z1-z0)*0.5*(1.-(PI*s).cos());
            let dz = (z1-z0)*0.5*PI*(PI*s).sin();
            if dz == 0. { return 0. }
            let (n,f) = flat_drum_spans(a,z);
            PI*(f-n)*dz
        })
    }).sum()
}

/// **A flat-ended drum leaning to a turn's axis turned whole closes into its solid of revolution**
/// (#780 L2's gate, X4: a flat end whose corners dig past it): a drum of radius 1 leaning 25° from
/// the turn's axis and 1.2 beside the plane through it, its ends square to its own axis. Its side
/// touches along two curves turned into surfaces of revolution (#780 C3); its ends' contact lines
/// miss them; each rim is a sharp circle whose fan holds a normal square to the turn's velocity
/// along an arc of it, which turns into the ring the circle sweeps (`turned::Rim`, its meridian read
/// exactly), tangent to the side's along the circles where the side's contact reaches the rim.
/// Turned whole, its volume is `∫ π (ρ_max² − ρ_min²) dz` over its slices, worked apart from the
/// build (`flat_drum_whole`); held to it, to membership and to a Monte Carlo of the definition
/// (`turned_whole`).
#[test]
fn a_flat_ended_drum_leaning_to_the_axis_turned_whole_closes_into_its_solid_of_revolution() {
    let a = leaning();
    turned_whole("a flat-ended drum",&flat_drum(a),&|p| in_flat_drum(a,p),flat_drum_whole(a));
}

/// **A flat-ended drum leaning to a turn's axis turned part of a turn closes into its sweep**
/// (#780 L2's gate, X4 part of a turn): turned 100°, its ends and side at each end pose, its side's
/// contacts turned and its rims' rings. At each end pose the end's plane meets its rim's ring in
/// that circle and a second of its radius, the side meets it in the circle and the loop crossing
/// it where the side's contact reaches the rim (`turned::Crossing`), and the plane cuts the side's
/// contact turned in closed form (`turned::Section`), tangent there to the rim: every meeting at
/// that point exact. Its own volume and `Θ/2π` of its whole turn's (each slice convex and clear of
/// the axis), membership and a Monte Carlo of the definition (`turned_part`).
#[test]
#[cfg_attr(not(feature = "slow"),ignore = "slow tier, about 20 s: a flat-ended drum turned 100°, its membership read and sampled")]
fn a_flat_ended_drum_leaning_to_the_axis_turned_part_of_a_turn_closes_into_its_sweep() {
    let a = leaning();
    turned_part("a flat-ended drum",&flat_drum(a),&|p| in_flat_drum(a,p),flat_drum_whole(a),true);
}

/// `flat_drum`'s rim at `t` along its axis (`±1.5`) as a `turned::Rim` in the world (the turn's
/// frame), read from its lowest point as the drum's disc reads it — where a plane's cut of its ring
/// runs across the place the ring's meridian starts.
fn flat_rim(a: V,t: f64) -> gcs_core::brep::turned::Rim {
    let x = unit(sub(scale(a,a[2]),[0.,0.,1.]));
    gcs_core::brep::turned::Rim {o:add([5.,1.2,0.],scale(a,t)),x,y:cross(a,x),r:1.}
}

/// The ring `flat_drum`'s rim at `t` turns into, as `closed` enters it: its meridian turned once.
fn flat_ring(a: V,t: f64) -> Brep {
    use gcs_core::brep::build::{Profile,ProfileEdge,revolve};
    let g = Frame::WORLD;
    let profile = Profile {origin:g.o,normal:g.y,names:Vec::new(),loops:vec![vec![ProfileEdge::Exact(Curve::Turned(std::sync::Arc::new(Turned::rim(g,flat_rim(a,t),[0.,TAU]))))]]};
    revolve(&profile,g.o,g.z,TAU).unwrap()
}

/// **Whether every candidate's cells fill it** in their arrangement within `domain`: each candidate
/// (held inside the domain) the sum of the volumes of the cells whose points are in it (by its own
/// exact point location, three a cell), to 1e-6 of its volume. A meeting lost leaves a cell
/// reaching through a candidate's boundary, and its volume counted on the wrong side.
fn fills(domain: &Brep,candidates: &[&Brep]) -> Vec<f64> {
    use gcs_core::brep::query::{Located,Place};
    let c = gcs_core::brep::boolean::arrangement_within(domain,candidates,1e-6).unwrap();
    let cells = c.cells().unwrap();
    candidates.iter().map(|k| {
        let at = Located::new(k,1e-6);
        let held: f64 = (1..cells.len()).filter(|&i| {
            let pts = c.inside_points(i,3).unwrap();
            2*pts.iter().filter(|&&x| at.solid_place(x) == Place::In).count() > pts.len()
        }).map(|i| cells[i].volume).sum();
        held-volume(k)
    }).collect()
}

/// **A flat end's side at an end pose and its rim's ring are arranged** (#780 L2, reduced from the
/// gate): the side turned 100° (a cylinder past its rims) holds the rim the ring holds, and meets the
/// ring in that circle and the loop crossing it at two points where the two touch
/// (`turned::Crossing`, its period `2π`: read as an open curve from one end to the other, it had been
/// dropped as no edge, and both solids' cells lost); each fills its cells.
#[test]
fn a_flat_ends_side_at_an_end_pose_and_its_rims_ring_are_arranged() {
    use gcs_core::brep::geom::Rigid;
    let a = leaning();
    let pose = Rigid::turn([0.;3],[0.,0.,1.],PART);
    let side = prism(&disc(pose.point(add([5.,1.2,0.],scale(a,-1.9))),pose.vector(a),1.),0.,3.8).unwrap();
    for t in [-1.5,1.5] {
        let gaps = fills(&block([-9.,-9.,-4.],[9.,9.,4.]),&[&side,&flat_ring(a,t)]);
        assert!(gaps.iter().all(|g| g.abs() <= 1e-6),"the rim at {t}: {gaps:?}");
    }
}

/// **A plane cutting a rim's ring, traced across its meridian's start, closes** (#780 L2, reduced
/// from the gate): the far end's plane at the end pose turned on cuts the near rim's ring in loops
/// running across the place its meridian starts, at its lowest point; the ring's slant read round its period
/// there (`Turned::newton`), not held to its span, each loop closes, and the ring fills its cells.
#[test]
fn a_plane_cutting_a_rims_ring_across_its_meridians_start_closes() {
    use gcs_core::brep::geom::Rigid;
    let a = leaning();
    // (the far end's plane at the end pose turned on: the loops the ring's slant, held to its
    // span, stopped short of closing)
    let pose = Rigid::turn([0.;3],[0.,0.,1.],PART);
    let end = gcs_core::brep::build::kept(&block([-12.,-12.,-6.],[12.,12.,6.]),pose.point(add([5.,1.2,0.],scale(a,-1.5))),pose.vector(a),0.).unwrap();
    let gaps = fills(&block([-9.,-9.,-4.],[9.,9.,4.]),&[&flat_ring(a,1.5),&end]);
    assert!(gaps[0].abs() <= 1e-6,"{gaps:?}");
}

/// **The meetings where a flat end at an end pose holds its rim are exact** (#780 L2), each on both
/// surfaces to rounding and its jets agreeing with differences: the end's plane against its rim's
/// ring, the rim and a second circle of its radius (`ssi`'s `plane_holding_rim`); the side's cylinder
/// against it, the rim and the loop crossing it where the two touch (`turned::Crossing`, at
/// `τ = π/2, 3π/2` on the rim); and the plane against the side's contact turned, its cut read round
/// `τ` (`turned::Section`).
#[test]
fn the_meetings_where_a_flat_end_at_an_end_pose_holds_its_rim_are_exact() {
    use gcs_core::brep::geom::Rigid;
    use gcs_core::brep::ssi::{Ssi,intersect};
    let a = leaning();
    let pose = Rigid::turn([0.;3],[0.,0.,1.],PART);
    let (g,rim) = (Frame::WORLD,flat_rim(a,1.5));
    let c = rim.o;
    let ring = Surface::Revolution(g,std::sync::Arc::new(Curve::Turned(std::sync::Arc::new(Turned::rim(g,rim,[0.,TAU])))));
    let (pc,pa) = (pose.point(c),pose.vector(a));
    let plane = Surface::Plane(Frame::about(pc,pa));
    let side = Surface::Cylinder(Frame::about(pose.point([5.,1.2,0.]),pa),1.);
    let contact = Contact {o:[5.,1.2,0.],a,r:1.,sa:0.,ca:1.,side:1.};
    let turned = Surface::Revolution(g,std::sync::Arc::new(Curve::Turned(std::sync::Arc::new(Turned::meridian(g,contact,[-1.65,1.65])))));
    let on = |s: &Surface,p: V| norm(sub(s.point(s.inverse(p)),p));
    for (what,x,y,kinds) in [("the plane and the ring",&plane,&ring,vec!["circle","circle"]),("the side and the ring",&side,&ring,vec!["circle","turned"]),
        ("the plane and the side's contact",&plane,&turned,vec!["turned"])] {
        let Ssi::Curves(curves) = intersect(x,y,1e-9) else { panic!("{what}: traced") };
        assert_eq!(curves.iter().map(|c| c.kind()).collect::<Vec<_>>(),kinds,"{what}");
        for c in &curves {
            let span = match c { Curve::Turned(t) => t.span,_ => [0.,TAU] };
            for k in 0..=64 {
                let u = span[0]+(span[1]-span[0])*k as f64/64.;
                let p = c.point(u);
                assert!(on(x,p) <= 1e-9 && on(y,p) <= 1e-9,"{what}: a {} off by {:.1e} and {:.1e} at {u}",c.kind(),on(x,p),on(y,p));
                if k == 0 || k == 64 { continue }
                let (h,(_,d,dd)) = (1e-5,c.d2(u));
                let (pm,dm) = { let (p,d,_) = c.d2(u-h); (p,d) };
                let (pp,dp) = { let (p,d,_) = c.d2(u+h); (p,d) };
                for i in 0..3 {
                    assert!((d[i]-(pp[i]-pm[i])/(2.*h)).abs() <= 1e-6*(1.+norm(d)),"{what}: {}'s first derivative at {u}",c.kind());
                    assert!((dd[i]-(dp[i]-dm[i])/(2.*h)).abs() <= 1e-4*(1.+norm(dd)),"{what}: {}'s second derivative at {u}",c.kind());
                }
            }
        }
        // the loop crosses the rim where the side touches the ring: at `τ = π/2` and `3π/2`, on it
        if let Some(looped @ Curve::Turned(t)) = curves.get(1) {
            assert!(matches!(t.source,gcs_core::brep::turned::Source::Crossing(_)),"{what}: a loop");
            for u in [std::f64::consts::FRAC_PI_2,3.*std::f64::consts::FRAC_PI_2] {
                let p = looped.point(u);
                assert!((norm(sub(p,pc))-1.).abs() <= 1e-12 && dot(sub(p,pc),pa).abs() <= 1e-12,"{what}: the loop at {u} off the rim");
            }
        }
    }
}

/// **A sharp circle turned is a ring whose meridian is read exactly** (#780 L2, `turned::Rim`): a
/// circle of radius 0.8 leaning to the world's `z` and beside the plane through it. Its meridian's
/// jets agree with central differences and its point is the circle's at the same angle carried to
/// its `(ρ, z)`; its interval jet over a stretch holds every float sample within it; it is simple
/// (its meridian a closed curve clear of the axis), where a level circle, one whose normal lies in
/// the plane through the axis and its centre (its halves one surface), and one through the axis are
/// not. Turned once it is a ring of `2π² r² |n · (c × z)|`, `π` times its ellipse's area, and the
/// dump writes it and reads it back bit for bit. And a line meets its surface of revolution
/// (`query::line_turned`, certified) where
/// the ring's implicit form, worked apart from the kernel, changes sign along it: measured from the
/// axis's origin `ρ² + z²` and `z` are affine in `(cos w, sin w)`, so the ring is where
/// `|L⁻¹ ((ρ² + z², z) − m)| = 1`, `m` and `L` that affine map's.
#[test]
fn a_sharp_circle_turned_is_a_ring_read_exactly() {
    use gcs_core::brep::query::{Meets,curve_surface};
    use gcs_core::brep::turned::Rim;
    let n = unit([0.3,0.5,0.8]);
    let (x,y) = { let f = Frame::about([0.;3],n); (f.x,f.y) };
    let rim = Rim {o:[4.,1.,0.5],x,y,r:0.8};
    let t = Turned::rim(Frame::WORLD,rim,[0.,TAU]);
    let point = |w: f64| add(rim.o,add(scale(x,0.8*w.cos()),scale(y,0.8*w.sin())));
    for k in 0..=24 {
        let w = TAU*k as f64/24.;
        let (p,d,dd) = t.d2(w);
        let q = point(w);
        assert!(p[1].abs() <= 1e-15 && (p[0]-q[0].hypot(q[1])).abs() <= 1e-12 && (p[2]-q[2]).abs() <= 1e-15,"at {w}: {p:?} against {q:?}");
        let h = 1e-5;
        let ((pm,dm,_),(pp,dp,_)) = (t.d2(w-h),t.d2(w+h));
        for i in 0..3 {
            assert!((d[i]-(pp[i]-pm[i])/(2.*h)).abs() <= 1e-8,"M' {i} at {w}");
            assert!((dd[i]-(dp[i]-dm[i])/(2.*h)).abs() <= 1e-6,"M'' {i} at {w}");
        }
    }
    let [[r,dr,ddr],[z,dz,ddz]] = t.bounded([1.,1.6]).unwrap();
    for k in 0..=50 {
        let w = 1.+0.6*k as f64/50.;
        let [jr,jz] = t.source.meridian(Jet::at(w));
        for (b,v) in [(r,jr.v),(dr,jr.d),(ddr,jr.dd),(z,jz.v),(dz,jz.d),(ddz,jz.dd)] {
            assert!(b[0] <= v && v <= b[1],"{v} outside {b:?}");
        }
    }
    assert!(rim.simple(1e-9),"a leaning circle beside the axis turns into a ring");
    let level = Rim {o:[4.,1.,0.5],x:[1.,0.,0.],y:[0.,1.,0.],r:0.8};
    assert!(!level.simple(1e-9),"a level circle turns within its plane");
    let (a,b) = { let f = Frame::about([0.;3],unit([0.,0.4,1.])); (f.x,f.y) };
    assert!(!Rim {o:[0.,4.,0.5],x:a,y:b,r:0.8}.simple(1e-9),"a circle symmetric about the plane through the axis");
    assert!(!Rim {o:sub([0.,0.,0.5],scale(x,0.8)),x,y,r:0.8}.simple(1e-9),"a circle through the axis");
    // the ring's implicit form: `(ρ² + z², z)` carried back through the circle's affine map
    let m = [dot(rim.o,rim.o)+0.64,rim.o[2]];
    let l = [[2.*0.8*dot(rim.o,x),2.*0.8*dot(rim.o,y)],[0.8*x[2],0.8*y[2]]];
    let det = l[0][0]*l[1][1]-l[0][1]*l[1][0];
    let implicit = |p: V| {
        let q = [dot(p,p)-m[0],p[2]-m[1]];
        let c = [(l[1][1]*q[0]-l[0][1]*q[1])/det,(-l[1][0]*q[0]+l[0][0]*q[1])/det];
        c[0]*c[0]+c[1]*c[1]-1.
    };
    for k in 0..24 { assert!(implicit(point(k as f64*0.3)).abs() <= 1e-12) }
    // its ring, the meridian turned once: in `(ρ², z)` a turn's volume element is `π dP dz`, and
    // `(ρ² + z², z)` moves it with no stretch, so the ring holds `π` times its ellipse's area,
    // `π · π r² |det L|` = `2π² r² |n · (c × z)|`; written and read back bit for bit (`swept::dump`)
    {
        use gcs_core::brep::build::{Profile,ProfileEdge,revolve};
        use gcs_core::brep::swept::dump::{read,write};
        let profile = Profile {origin:[0.;3],normal:[0.,1.,0.],names:Vec::new(),loops:vec![vec![ProfileEdge::Exact(Curve::Turned(std::sync::Arc::new(t)))]]};
        let ring = revolve(&profile,[0.;3],[0.,0.,1.],TAU).unwrap();
        let expected = 2.*PI*PI*0.64*dot(n,cross(rim.o,[0.,0.,1.])).abs();
        assert!((volume(&ring)-expected).abs() <= 1e-9*expected,"the ring holds {} against {expected}",volume(&ring));
        let text = write(&ring).unwrap();
        let back = read(&text).unwrap();
        assert_eq!(write(&back).unwrap(),text);
        assert_eq!(volume(&back),volume(&ring));
    }
    let s = Surface::Revolution(Frame::WORLD,std::sync::Arc::new(Curve::Turned(std::sync::Arc::new(t))));
    let mut rng = gcs_core::rng::Rng::new(7);
    let mut met = 0;
    for _ in 0..60 {
        let p: V = [rng.uniform(-6.,6.),rng.uniform(-6.,6.),rng.uniform(-1.5,1.5)];
        let d = unit([rng.uniform(-1.,1.),rng.uniform(-1.,1.),rng.uniform(-1.,1.)]);
        let Meets::At(found) = curve_surface(&Curve::Line {p,d},[-20.,20.],&s,1e-9) else { panic!("a line lying in a ring") };
        for &(at,_) in &found {
            let q = add(p,scale(d,at));
            assert!(implicit(q).abs() <= 1e-6,"a crossing off the ring: {}",implicit(q));
        }
        let samples: Vec<f64> = (0..=40000).map(|k| implicit(add(p,scale(d,-20.+40.*k as f64/40000.)))).collect();
        let sampled = samples.windows(2).filter(|w| w[0]*w[1] < 0.).count();
        assert_eq!(found.iter().filter(|r| !r.1).count(),sampled,"at {p:?} along {d:?}: {found:?}");
        met += found.len();
    }
    assert!(met > 20,"{met} crossings read");
}

/// **The radii squared a slice of a leaning drum cut by planes spans about the world's `z`**, in
/// closed form apart from any build (#780 L2's arcs): the slice at height `z` of the drum of radius
/// 1 about the axis through `(5, 1.2, 0)` along `a` (`a_y = 0`) is the ellipse `E(φ) = m + (cos φ /
/// cos β) s + sin φ w` (`flat_drum_spans`) cut by the half-spaces `n · p ≤ d`, each a line in the
/// slice. `ρ²` is convex, so its least and most over that convex region are among: the ellipse's
/// critical points (by bisection on the rate of `ρ²` from 3600 samples), its corners on each line
/// (`A cos φ + B sin φ = C`), two lines' crossings, and each line's foot nearest the axis — each
/// kept where it lies in every other half-space and the ellipse. With which kind of point held each
/// end (`key`): where that changes, the integrand's form changes.
fn cut_drum_spans(a: V,cuts: &[(V,f64)],z: f64) -> Option<(f64,f64,[(u8,usize,usize);2])> {
    let ca = a[2];
    let m = add([5.,1.2,0.],scale(a,z/ca));
    let (s,w) = (unit([a[0],a[1],0.]),[-a[1]/a[0].hypot(a[1]),a[0]/a[0].hypot(a[1]),0.]);
    let (e,critical) = slice_ellipse(a,z);
    let r2 = |p: V| p[0]*p[0]+p[1]*p[1];
    let slack = 1e-12;
    let inside_ellipse = |p: V| { let q = sub(p,m); let (x,y) = (dot(q,s)*ca,dot(q,w)); x*x+y*y <= 1.+slack };
    let kept = |p: V,skip: &[usize]| (0..cuts.len()).all(|i| skip.contains(&i) || dot(cuts[i].0,p) <= cuts[i].1+slack);
    let mut found: Vec<(f64,(u8,usize,usize))> = critical.into_iter().map(&e).filter(|&p| kept(p,&[])).map(|p| (r2(p),(0,0,0))).collect();
    for (i,&(nn,d)) in cuts.iter().enumerate() {
        // the corners: n · E(φ) = d, `A cos φ + B sin φ = C`
        let (aa,bb,cc) = (dot(nn,s)/ca,dot(nn,w),d-dot(nn,m));
        let rr = aa.hypot(bb);
        if rr > 0. && cc.abs() <= rr {
            let (phi,delta) = (bb.atan2(aa),(cc/rr).clamp(-1.,1.).acos());
            for f in [phi+delta,phi-delta] { let p = e(f); if kept(p,&[i]) { found.push((r2(p),(1,i,0))) } }
        }
        // the line in the slice: its foot nearest the axis
        let nh = [nn[0],nn[1]];
        let h2 = nh[0]*nh[0]+nh[1]*nh[1];
        let level = (d-nn[2]*z)/h2;
        let foot = [nh[0]*level,nh[1]*level,z];
        if inside_ellipse(foot) && kept(foot,&[i]) { found.push((r2(foot),(2,i,0))) }
        // two lines' crossing
        for (j,&(mm,dj)) in cuts.iter().enumerate().skip(i+1) {
            let det = nn[0]*mm[1]-nn[1]*mm[0];
            if det.abs() < 1e-12 { continue }
            let (bi,bj) = (d-nn[2]*z,dj-mm[2]*z);
            let p = [(bi*mm[1]-bj*nn[1])/det,(nn[0]*bj-mm[0]*bi)/det,z];
            if inside_ellipse(p) && kept(p,&[i,j]) { found.push((r2(p),(3,i,j))) }
        }
    }
    let near = found.iter().copied().min_by(|x,y| x.0.total_cmp(&y.0))?;
    let far = found.iter().copied().max_by(|x,y| x.0.total_cmp(&y.0))?;
    Some((near.0,far.0,[near.1,far.1]))
}

/// **The volume a leaning drum cut by planes sweeps turned whole, worked apart from any build**:
/// `∫ π (ρ_max² − ρ_min²) dz` over its slices (`cut_drum_spans`), cut where the kind of point holding
/// either end changes (found from 2000 samples over `[z0, z1]`, each change bisected to rounding) and
/// each piece read by Simpson's rule under `z = z₀ + (z₁ − z₀)(1 − cos πs)/2`.
fn cut_drum_whole(a: V,cuts: &[(V,f64)],[z0,z1]: [f64;2]) -> f64 {
    let key = |z: f64| cut_drum_spans(a,cuts,z).map(|s| s.2);
    let n = 2000;
    let zs: Vec<f64> = (0..=n).map(|k| z0+(z1-z0)*k as f64/n as f64).collect();
    let mut breaks = vec![z0];
    for w in zs.windows(2) {
        let (ka,kb) = (key(w[0]),key(w[1]));
        if ka == kb { continue }
        let (lo,hi) = gcs_core::roots::bisect(w[0],w[1],|x,y| 0.5*(x+y),|x,y| 0.5*(x+y) != x && 0.5*(x+y) != y,|z| key(z) == ka);
        breaks.push(0.5*(lo+hi));
    }
    breaks.push(z1);
    breaks.windows(2).map(|q| simpson(0.,1.,200,|t| {
        let z = q[0]+(q[1]-q[0])*0.5*(1.-(PI*t).cos());
        let dz = (q[1]-q[0])*0.5*PI*(PI*t).sin();
        match cut_drum_spans(a,cuts,z) { Some((n,f,_)) if dz != 0. => PI*(f-n)*dz,_ => 0. }
    })).sum()
}

/// The planes `flat_drum` is cut by, as half-spaces `n · p ≤ d`: its ends square to its axis 1.5
/// from its middle, and `more`.
fn drum_cuts(a: V,more: &[(V,f64)]) -> Vec<(V,f64)> {
    let o = [5.,1.2,0.];
    let mut out = vec![(a,dot(a,o)+1.5),(scale(a,-1.),-dot(a,o)+1.5)];
    out.extend_from_slice(more);
    out
}

/// **The worked volume of a cut drum turned whole is the flat-ended drum's where nothing more cuts
/// it**: `cut_drum_whole`, its breaks found rather than listed, against `flat_drum_whole`'s, to 1e-10.
#[test]
fn the_worked_volume_of_a_cut_drum_is_the_flat_ended_drums() {
    let a = leaning();
    let (x,y) = (cut_drum_whole(a,&drum_cuts(a,&[]),[-1.8,1.8]),flat_drum_whole(a));
    assert!((x-y).abs() <= 1e-10*y,"{x} against {y}");
}

/// A half drum: `flat_drum` cut along its axis by the plane through it along `y`, kept on the side
/// away from the turn's axis — its rims half circles, sharp arcs leaning to the turn's axis, joined
/// by its cut's lines — with the cut's half-space and the tool's own test.
fn half_drum(a: V) -> (Brep,(V,f64),impl Fn(V) -> f64) {
    let o = [5.,1.2,0.];
    let w = unit(cross(a,[0.,1.,0.]));
    let tool = gcs_core::brep::build::kept(&flat_drum(a),o,scale(w,-1.),0.).unwrap();
    (tool,(w,dot(w,o)),move |p: V| in_flat_drum(a,p).max(dot(sub(p,o),w)))
}

/// **A half drum leaning to a turn's axis turned whole closes into its solid of revolution** (#780
/// L2, a sharp arc under a turn): its rims are half circles, each turned into its whole circle's
/// ring (`turned::Rim`), its sites the arc's own points, membership keeping what bounds the sweep;
/// its cut's lines into hyperboloids, its cut's plane's contact turned. Held to `∫ π (ρ_max² − ρ_min²)
/// dz` over its slices worked apart (`cut_drum_whole`), to membership and a Monte Carlo of the
/// definition (`turned_whole`).
#[test]
fn a_half_drum_leaning_to_the_axis_turned_whole_closes_into_its_solid_of_revolution() {
    let a = leaning();
    let (tool,cut,inside) = half_drum(a);
    turned_whole("a half drum",&tool,&inside,cut_drum_whole(a,&drum_cuts(a,&[cut]),[-1.8,1.8]));
}

/// **A half drum leaning to a turn's axis turned part of a turn is cut right or refused by name**
/// (#780 L2, left open): turned 100°, the end of each rim's arc at the cut's edge — its path's
/// point at an end pose, where the end's plane, the cut's, the rim's ring and the cut's lines'
/// surfaces all pass — leaves a cell thinner than the mesh's sag can settle: refused
/// (`Unbuilt::Unsettled`), never built wrong; built, its own volume and `Θ/2π` of its whole turn's
/// (convex), membership and a Monte Carlo of the definition (`turned_part`).
#[test]
#[cfg_attr(not(feature = "slow"),ignore = "slow tier, about 35 s: a half drum turned 100°, refused where its arcs end")]
fn a_half_drum_leaning_to_the_axis_turned_part_of_a_turn_is_cut_right_or_refused_by_name() {
    use gcs_core::brep::swept::closed::Unbuilt;
    let a = leaning();
    let (tool,cut,inside) = half_drum(a);
    let path = turn(PART);
    match closed(&tool,&path,[0.,1.],0.002,1e-6) {
        Ok(_) => turned_part("a half drum",&tool,&inside,cut_drum_whole(a,&drum_cuts(a,&[cut]),[-1.8,1.8]),true),
        Err(Unbuilt::Unsettled {..}) => {}
        Err(e) => panic!("a half drum turned {PART}: {e:?}"),
    }
}

/// **A leaning drum turned part of a turn builds through its given touches** (#810, item 5): with
/// `ssi`'s recognition of a leaning round face at a pose against its own contact turned bypassed
/// (`Recognised::OwnContact`, C3's `own_contact`), `a_leaning_drum_turned_closes_into_its_sweep`'s
/// part turn holds its volume to the same `Θ/2π` of the whole and the drum's own, to membership and
/// to a Monte Carlo — the side at each end pose and its contact's surfaces meeting as the closed
/// class gives them (`boolean::Given`).
#[test]
fn a_leaning_drum_turned_builds_through_its_given_touches() {
    use gcs_core::brep::ssi::{Recognised,bypassing};
    let a = leaning();
    let tool = leaning_drum(a);
    let whole = simpson(-1.5,1.5,400,|z| { let (n,f) = drum_spans(a,z); PI*(f*f-n*n) });
    let o = [5.,1.2,0.];
    let inside = |p: V| { let q = sub(p,o); let t = dot(q,a); (norm(sub(q,scale(a,t)))-1.).max(t.abs()-3.).max(p[2].abs()-1.5) };
    bypassing(&[Recognised::OwnContact],|| turned_part("a leaning drum",&tool,&inside,whole,true));
}

/// **A flat-ended drum turned part of a turn builds through its given touches** (#810, item 5):
/// with `ssi`'s recognition of a plane or a cylinder holding a sharp circle against that circle's
/// ring bypassed (`Recognised::RimHeld`, L2's `plane_holding_rim` and `side_holding_rim`),
/// `a_flat_ended_drum_leaning_to_the_axis_turned_part_of_a_turn_closes_into_its_sweep` holds to the
/// same figure — each end's plane and the side at each end pose meeting the rims' rings as the
/// closed class gives them (`boolean::Given`).
#[test]
#[cfg_attr(not(feature = "slow"),ignore = "slow tier, about 20 s: a flat-ended drum turned 100°, its membership read and sampled")]
fn a_flat_ended_drum_turned_builds_through_its_given_touches() {
    use gcs_core::brep::ssi::{Recognised,bypassing};
    let a = leaning();
    bypassing(&[Recognised::RimHeld],|| turned_part("a flat-ended drum",&flat_drum(a),&|p| in_flat_drum(a,p),flat_drum_whole(a),true));
}
