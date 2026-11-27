//! **A traced meeting as a smooth curve** (#1034): two surfaces meeting in no closed form are
//! traced, and the trace read as the meeting itself — its rates the meeting's tangent and
//! curvature in closed form, its pace running on through its points, its enclosure proved. Each
//! is held here to a meeting whose curve has a closed form apart from the kernel: two crossed
//! cylinders (the lesser's angle gives the point) and a torus cut by a plane through every
//! meridian (a spiric curve, the meridian's angle gives the point).
use gcs_core::brep::boolean::{combined,Op};
use gcs_core::brep::enclose::Enclosed;
use gcs_core::brep::geom::{Curve,Frame,Surface,Traced,V};
use gcs_core::brep::props::volume;
use gcs_core::brep::ssi::{intersect,trace,Ssi};
use gcs_core::meter::{self as meter,Budget,Meter};
use gcs_core::rng::Rng;
use gcs_core::space::{add,cross,dot,norm,scale,sub};
use std::f64::consts::PI;
use gcs_core::taylor::Jet;

/// A closed form's jets along its parameter (`taylor::Jet`): a constant, the parameter, its sine
/// and cosine, and the inverse sine and tangent by `atan2`.
fn c(v: f64) -> Jet { Jet::constant(v) }
fn sin(x: Jet) -> Jet { x.sin_cos().0 }
fn cos(x: Jet) -> Jet { x.sin_cos().1 }
fn asin(x: Jet) -> Jet { Jet::atan2(x,(c(1.)-x*x).sqrt()) }
fn atan(x: Jet) -> Jet { Jet::atan2(x,c(1.)) }

/// A curve's point, unit tangent and curvature vector `(C'' − (C''·T) T)/|C'|²` from its jet.
fn closed_jet(e: [Jet;3]) -> (V,V,V) {
    let (d,dd) = (e.map(|j| j.nth(1)),e.map(|j| j.nth(2)));
    let s2 = dot(d,d);
    let t = scale(d,1./s2.sqrt());
    (e.map(|j| j.nth(0)),t,scale(sub(dd,scale(t,dot(dd,t))),1./s2))
}

/// The cylinder of radius 2 about `z` and the one of radius 1 about `x` through it.
fn crossed() -> (Surface,Surface) {
    (Surface::Cylinder(Frame::new([0.;3],[0.,0.,1.],[1.,0.,0.]),2.),Surface::Cylinder(Frame::new([0.;3],[1.,0.,0.],[0.,1.,0.]),1.))
}

/// The crossed cylinders' meeting through `p`, in closed form over the lesser's angle `θ`:
/// `(±√(4 − cos² θ), cos θ, sin θ)`.
fn crossed_closed(p: V) -> (V,V,V) {
    let th = Jet::var(p[2].atan2(p[1]));
    let x = (c(4.)-cos(th)*cos(th)).sqrt()*p[0].signum();
    closed_jet([x,cos(th),sin(th)])
}

const BIG: f64 = 5.;
const TUBE: f64 = 2.;
const SLANT: f64 = 12.*PI/180.;
const RISE: f64 = 0.4;

/// The torus about `z` of radii 5 and 2 and the plane `z = 0.4 + x tan 12°`, which cuts every
/// meridian circle: two spiric curves round the ring.
fn ring_and_plane() -> (Surface,Surface) {
    let n = [SLANT.sin(),0.,-SLANT.cos()];
    (Surface::Torus(Frame::new([0.;3],[0.,0.,1.],[1.,0.,0.]),BIG,TUBE),Surface::Plane(Frame::new([0.,0.,RISE],n,cross([0.,1.,0.],n))))
}

/// The spiric curve through `p`, in closed form over the meridian's angle `u`: the line
/// `z = c + ρ k` (`k = tan β cos u`) meets the meridian `ρ = R + r cos v`, `z = r sin v` at
/// `v = φ + π − asin s` and `v = φ + 2π + asin s`, `φ = atan k`, `s = (R k + c)/(r √(1 + k²))` — the
/// one nearer `p`'s own.
fn spiric_closed(p: V) -> (V,V,V) {
    let u = Jet::var(p[1].atan2(p[0]));
    let v0 = p[2].atan2(p[0].hypot(p[1])-BIG);
    let k = cos(u)*SLANT.tan();
    let s = (k*BIG+c(RISE))/((c(1.)+k*k).sqrt()*TUBE);
    let (va,vb) = (atan(k)+c(PI)-asin(s),atan(k)+c(2.*PI)+asin(s));
    let off = |v: Jet| { let d = (v.nth(0)-v0).rem_euclid(2.*PI); d.min(2.*PI-d) };
    let v = if off(va) < off(vb) { va } else { vb };
    let rho = c(BIG)+cos(v)*TUBE;
    closed_jet([rho*cos(u),rho*sin(u),sin(v)*TUBE])
}

/// The crossed cylinders' two loops (`ssi::intersect`'s own trace) and the spiric section's two
/// (traced from a seed on each), with their surfaces and closed forms.
fn traces() -> Vec<(&'static str,Curve,Surface,Surface,fn(V) -> (V,V,V))> {
    let mut out: Vec<(&'static str,Curve,Surface,Surface,fn(V) -> (V,V,V))> = Vec::new();
    let (big,small) = crossed();
    let Ssi::Curves(loops) = intersect(&big,&small,1e-9) else { panic!("crossed cylinders: traced") };
    assert_eq!(loops.len(),2);
    for c in loops { out.push(("a crossed cylinders' loop",c,big.clone(),small.clone(),crossed_closed)) }
    let (ring,plane) = ring_and_plane();
    let probe = Traced::new(ring.clone(),plane.clone(),vec![],false);
    let seeds = [probe.project([7.,0.,1.5]),probe.project([3.,0.,0.8])];
    let curves = trace(&ring,&plane,&seeds,[-10.;3],[10.;3],1e-9).unwrap();
    assert_eq!(curves.len(),2);
    for c in curves { out.push(("a spiric section",c,ring.clone(),plane.clone(),spiric_closed)) }
    out
}

fn span(c: &Curve) -> [f64;2] {
    let Curve::Traced(t) = c else { panic!("{c:?}: traced") };
    assert!(t.closed);
    [0.,(t.pts.len()-1) as f64]
}

/// **A trace's rates are its meeting's own, in closed form** (#1034): along each trace, at a
/// thousand places, its point lies on both surfaces and on the closed form's curve, its unit
/// tangent and its curvature vector `(C'' − (C''·T) T)/|C'|²` agree with the closed form's to 1e-9
/// — read before as a straight line, `C''` nothing — its rate agrees with the difference of its
/// points, and its speed is the same all along it to a part in a million, through every point of
/// the trace — its
/// parameter runs evenly along its length (it stepped at each point, the parameter counting the
/// points over chords of their own lengths).
#[test]
fn a_traced_meetings_rates_are_its_closed_forms() {
    for (what,c,a,b,closed) in traces() {
        let [t0,t1] = span(&c);
        let n = 1000;
        for i in 0..=n {
            let t = t0+(t1-t0)*i as f64/n as f64;
            let (p,d,dd) = c.d2(t);
            assert!(a.implicit(p).abs() < 1e-12 && b.implicit(p).abs() < 1e-12,"{what} at {t}: {p:?} off its surfaces");
            let (q,tq,kq) = closed(p);
            assert!(norm(sub(p,q)) < 1e-9,"{what} at {t}: {p:?} off the closed form's {q:?}");
            let speed = norm(d);
            let tn = scale(d,1./speed);
            assert!(norm(sub(tn,tq)).min(norm(add(tn,tq))) < 1e-9,"{what} at {t}: tangent {tn:?}, the closed form's {tq:?}");
            let k = scale(sub(dd,scale(tn,dot(dd,tn))),1./(speed*speed));
            assert!(norm(sub(k,kq)) < 1e-9*(1.+norm(kq)),"{what} at {t}: curvature {k:?}, the closed form's {kq:?}");
            let h = 1e-5;
            if t-h >= t0 && t+h <= t1 {
                let fd = scale(sub(c.point(t+h),c.point(t-h)),0.5/h);
                assert!(norm(sub(fd,d)) < 1e-7*speed,"{what} at {t}: rate {d:?}, differenced {fd:?}");
            }
        }
        let Curve::Traced(tr) = &c else { unreachable!() };
        for k in 1..t1 as usize {
            let t = tr.parameter_of(k);
            let (before,after) = (c.d1(t-1e-12).1,c.d1(t).1);
            assert!(norm(sub(before,after)) < 1e-9*norm(after),"{what}: its rate steps at point {k}: {before:?} to {after:?}");
        }
        // its parameter runs evenly along it: the same speed everywhere, to a part in a million (the
        // quintic between two points runs along its length to the fifth power of the step)
        let speeds: Vec<f64> = (0..=n).map(|i| norm(c.d1(t0+(t1-t0)*i as f64/n as f64).1)).collect();
        let (lo,hi) = speeds.iter().fold((f64::INFINITY,0f64),|(l,h),&v| (l.min(v),h.max(v)));
        assert!(hi-lo <= 1e-6*hi,"{what}: its speed runs from {lo} to {hi}");
    }
}

/// **A trace's enclosure is proved** (#1034): over random stretches — a point, a piece of one
/// stretch between points, across several, round the whole — each trace's enclosure says it is
/// proved and holds its point and rate at seventeen places across.
#[test]
fn a_traced_meetings_enclosure_is_proved() {
    let mut rng = Rng::new(1034);
    for (what,c,..) in traces() {
        let [t0,t1] = span(&c);
        for j in 0..120 {
            let len = (t1-t0)*[0.,1e-4,0.01,0.1,0.5,1.][j%6]*rng.uniform(0.,1.);
            let a = rng.uniform(t0,t1-len);
            let b = a+len;
            let e = c.enclose([[a,b]]);
            assert!(e.proved && e.bounded(),"{what} over [{a}, {b}]: measured");
            for k in 0..=16 {
                let t = a+(b-a)*k as f64/16.;
                let (p,d) = c.d1(t);
                let held = (0..3).all(|i| e.at[i].lo() <= p[i] && p[i] <= e.at[i].hi() && e.d[0][i].lo() <= d[i] && d[i] <= e.d[0][i].hi());
                assert!(held,"{what} over [{a}, {b}] at {t}: {p:?} {d:?} outside {e:?}");
            }
        }
    }
}

/// **A Boolean through a traced meeting is proved** (#1034): the torus cut aslant
/// (`brep_offset::cut_torus`), its edges the plane's spiric curves, kept on one side of the plane
/// `y = 0` its own symmetry plane: every traced edge's crossing with the block's faces is proved —
/// none read through a measured enclosure, as each was before — and the half kept is half the
/// ring's volume.
#[test]
fn a_boolean_through_a_traced_meeting_is_proved() {
    let ring = crate::brep_offset::cut_torus(2.,SLANT,RISE);
    let half = crate::brep::block([-10.,-10.,-10.],[10.,0.,10.]);
    let m = Meter::new(Budget::UNLIMITED);
    let kept = meter::within(&m,|| combined(&ring,&half,Op::Common,1e-9)).unwrap();
    kept.check(1e-7).unwrap();
    let [proved,measured] = m.crossings("traced","plane");
    assert!(proved > 0 && measured == 0,"traced × plane: {proved} proved, {measured} measured\n{}",m.report());
    assert_eq!(m.measured(),0,"a search measured\n{}",m.report());
    let (whole,part) = (volume(&ring),volume(&kept));
    assert!((part-whole/2.).abs() < 1e-9*whole,"half the ring: {part} of {whole}");
}

/// **A wall ruled between traced rails is written as a smooth net** (#1034): the torus's face cut
/// aslant, lifted out and thickened (`brep_offset::torus_face_cut_aslant_thickened`), is walled by
/// the ruled sheets of the normals along its spiric edges, each a loft between a trace and the trace
/// carried along the normal. Each is written as a net fitted smooth across the traces' points within
/// the file's `FIT` — no knot of the degree's multiplicity anywhere inside it — where, while the
/// trace's pace stepped at every point, the net broke at each (`nurbs::fit_net_broken`, gone).
#[test]
fn a_wall_between_traced_rails_is_a_smooth_net() {
    use gcs_core::brep::step::{blend_net,FIT};
    let walled = crate::brep_offset::torus_face_cut_aslant_thickened();
    let mut walls = 0;
    for f in &walled.faces {
        let Surface::Blend(_,b) = &f.surface else { continue };
        assert!(matches!(b.a,Curve::Traced(_)),"a wall's rail: {}",b.a.kind());
        let (net,miss) = blend_net(b).unwrap();
        assert!(miss <= FIT,"a wall fitted {miss:e} off");
        assert!(net.kinks(0).is_empty() && net.kinks(1).is_empty(),"a wall's net breaks at {:?}",net.kinks(0));
        walls += 1;
    }
    assert!(walls >= 2,"{walls} walls");
}
