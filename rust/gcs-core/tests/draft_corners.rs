//! **Draft where an arc meets a side at an angle** (#962, item 8b; #245): a drafted prism's
//! section at ordinate `h` is its face moved `|h| tan θ` into its material, every side along its
//! own normal, each corner where the moved sides meet. Where an arc meets a line or another arc at
//! an angle that corner moves along a conic — where the side's plane meets the arc's cone, or two
//! coaxial-in-direction cones of one half-angle meet, always in a plane — and the exact kernel
//! builds that edge (`build::Moving`). Held here, over the family: each solid's volume against the
//! integral over its height of its section's area, read in closed form from the shapes the section
//! is (a disc and a half-plane, two discs, a rectangle less a disc), by Gauss–Legendre quadrature —
//! to 1e-9 — for lines meeting convex arcs at a right angle, an acute and an obtuse one, lines
//! meeting concave arcs, two arcs meeting the same way round and opposite ways, a hole whose
//! corners are such (the draft grows it), a D, a lens and a vane; one side of the parting plane and
//! across it, drafts of 0.5° and 15° and outward, turned, mirrored and stood in another plane; a
//! corner a hair off tangent; each corner's edge on both faces it joins; the body checked, meshed,
//! written to STEP and read back; and the field's reading of the same drawing agreeing with it.
use gcs_core::brep::build::{drafted,Profile,ProfileEdge};
use gcs_core::brep::geom::{Curve,Frame,Rigid,Surface};
use gcs_core::brep::props;
use gcs_core::brep::topo::{Brep,EdgeCurve};
use std::f64::consts::PI;

type P = [f64;2];

/// A profile's edge in the `(x, y)` plane: a line, or an arc about `c` of radius `r` turning
/// counter-clockwise from `a0` to `a1`.
#[derive(Clone,Copy)]
enum E { L(P,P),A(P,f64,f64,f64) }

/// A drawing in the `(x, y)` plane, its loops (the first the outer), each edge named.
struct Drawing { loops: Vec<Vec<(E,&'static str)>> }

impl Drawing {
    /// The same drawing mirrored across the `y` axis.
    fn mirrored(&self) -> Drawing {
        let m = |p: P| [-p[0],p[1]];
        Drawing {loops:self.loops.iter().map(|l| l.iter().map(|&(e,n)| (match e {
            E::L(a,b) => E::L(m(a),m(b)),
            E::A(c,r,a0,a1) => E::A(m(c),r,PI-a1,PI-a0),
        },n)).collect()).collect()}
    }
    /// The kernel's profile, in the plane `z = 0` with its normal `+z`, carried by `place`.
    fn profile(&self,place: &Rigid) -> Profile {
        let v = |p: P| [p[0],p[1],0.];
        let loops = self.loops.iter().map(|l| l.iter().map(|&(e,_)| match e {
            E::L(a,b) => ProfileEdge::Line {a:v(a),b:v(b)},
            E::A(c,r,a0,a1) => ProfileEdge::Arc {frame:Frame::new(v(c),[0.,0.,1.],[1.,0.,0.]),r,span:Some([a0,a1])},
        }).collect()).collect();
        let names = self.loops.iter().map(|l| l.iter().map(|&(_,n)| n.to_string()).collect()).collect();
        Profile {origin:[0.;3],normal:[0.,0.,1.],loops,names}.moved(place)
    }
}

/// A point at angle `a` on the circle about `c` of radius `r`.
fn on(c: P,r: f64,a: f64) -> P { [c[0]+r*a.cos(),c[1]+r*a.sin()] }

/// The area of the disc of radius `r` beyond the line `c` from its centre (`|c| < r`).
fn segment(r: f64,c: f64) -> f64 { r*r*(c/r).acos()-c*(r*r-c*c).sqrt() }

/// The area two discs of radii `a` and `b` whose centres stand `d` apart have in common (they cross).
fn lens(a: f64,b: f64,d: f64) -> f64 {
    a*a*((d*d+a*a-b*b)/(2.*d*a)).acos()+b*b*((d*d+b*b-a*a)/(2.*d*b)).acos()
        -0.5*((-d+a+b)*(d+a-b)*(d-a+b)*(d+a+b)).sqrt()
}

/// Gauss–Legendre nodes and weights on `[-1, 1]`, by Newton's method on the Legendre polynomial.
fn legendre(n: usize) -> Vec<(f64,f64)> {
    (0..n).map(|i| {
        let mut x = (PI*(i as f64+0.75)/(n as f64+0.5)).cos();
        loop {
            let (mut p0,mut p1) = (1.,x);
            for k in 2..=n { let p2 = ((2*k-1) as f64*x*p1-(k-1) as f64*p0)/k as f64; p0 = p1; p1 = p2; }
            let dp = n as f64*(x*p1-p0)/(x*x-1.);
            let dx = p1/dp;
            x -= dx;
            if dx.abs() < 1e-16 {
                let (mut p0,mut p1) = (1.,x);
                for k in 2..=n { let p2 = ((2*k-1) as f64*x*p1-(k-1) as f64*p0)/k as f64; p0 = p1; p1 = p2; }
                let dp = n as f64*(x*p1-p0)/(x*x-1.);
                return (x,2./((1.-x*x)*dp*dp))
            }
        }
    }).collect()
}

/// `∫ area(|h| tan θ) dh` from `h0` to `h1`, each side of the parting plane apart (the area is
/// smooth in the offset, not in `h` across it), on eight panels of twenty points each.
fn volume_of(area: &dyn Fn(f64) -> f64,h0: f64,h1: f64,theta: f64) -> f64 {
    let t = theta.tan();
    let rule = legendre(20);
    let span = |a: f64,b: f64| -> f64 {
        (0..8).map(|k| {
            let (lo,hi) = (a+(b-a)*k as f64/8.,a+(b-a)*(k+1) as f64/8.);
            rule.iter().map(|&(x,w)| { let h = (lo+hi)/2.+x*(hi-lo)/2.; w*area(h.abs()*t) }).sum::<f64>()*(hi-lo)/2.
        }).sum()
    };
    if h0 < 0. && h1 > 0. { span(h0,0.)+span(0.,h1) } else { span(h0,h1) }
}

/// Where the drawing is stood: as drawn, turned about `z` and moved, and turned onto another plane.
fn placements() -> Vec<(&'static str,Rigid)> {
    let id = Rigid::turn([0.;3],[0.,0.,1.],0.);
    vec![("as drawn",id),
         ("turned 37° and moved",Rigid::turn([0.;3],[0.,0.,1.],37f64.to_radians()).then(&Rigid {r:id.r,t:[3.,-7.,11.]})),
         ("stood on another plane",Rigid::turn([1.,2.,3.],[0.6,0.,0.8],1.1))]
}

/// The drafted solid's checks: its volume to `want`, valid, each side named, and every corner edge
/// that is a conic lying on both faces it joins; how many such conic edges it has.
fn holds(what: &str,b: &Brep,want: f64,names: &[&str]) -> usize {
    let got = props::volume(b);
    assert!((got-want).abs() <= 1e-9*want.abs(), "{what}: the volume {got} against {want} ({:e})",(got-want)/want);
    b.check(1e-7*b.size()).unwrap_or_else(|e| panic!("{what}: {e:?}"));
    for n in names {
        assert!(b.faces.iter().any(|f| f.name == *n), "{what}: no face `{n}` in {:?}",b.faces.iter().map(|f| &f.name).collect::<Vec<_>>());
    }
    let mut conics = 0;
    for (ei,e) in b.edges.iter().enumerate() {
        let EdgeCurve::Curve(c @ Curve::BSpline(_)) = &e.curve else { continue };
        conics += 1;
        let faces: Vec<&Surface> = b.faces.iter().filter(|f| f.loops.iter().flatten().any(|u| u.edge as usize == ei))
            .map(|f| &f.surface).collect();
        assert_eq!(faces.len(),2,"{what}: a corner edge on {} faces",faces.len());
        for k in 0..=16 {
            let p = c.point(e.t[0]+(e.t[1]-e.t[0])*k as f64/16.);
            for s in &faces {
                let q = s.point(s.inverse(p));
                let gap = ((p[0]-q[0]).powi(2)+(p[1]-q[1]).powi(2)+(p[2]-q[2]).powi(2)).sqrt();
                assert!(gap <= 1e-9*b.size(), "{what}: a corner edge stands {gap:e} off its {}",s.kind());
            }
        }
    }
    conics
}

/// Each drawing of the family, with its section's area at offset `d`.
struct Case { what: &'static str,drawing: Drawing,area: Box<dyn Fn(f64) -> f64>,names: Vec<&'static str>,conics: bool }

fn family() -> Vec<Case> {
    let mut out = Vec::new();
    // a disc of radius 10 above the line y = c: a right angle at c = 0, acute above, obtuse below
    for (what,c) in [("a D, its chord square to its arc",0.),("a D, acute",4.),("a D, obtuse",-4.)] {
        let (r,c): (f64,f64) = (10.,c);
        let phi = (c/r).asin();
        let (a,b) = (on([0.,0.],r,phi),on([0.,0.],r,PI-phi));
        out.push(Case {what,drawing:Drawing {loops:vec![vec![(E::L(b,a),"chord"),(E::A([0.,0.],r,phi,PI-phi),"arc")]]},
            area:Box::new(move |d| segment(r-d,c+d)),names:vec!["chord","arc"],conics:true});
    }
    // a rectangle 40 × 20 with a bite of radius 6 out of its top side, about a centre `e` above it:
    // a line meeting a concave arc square to it, and at an acute and an obtuse angle
    for (what,e) in [("a bite, square",0.),("a shallow bite",3.),("a deep bite",-3.)] {
        let (w,h,rho,e): (f64,f64,f64,f64) = (20.,20.,6.,e);
        let c = [0.,h+e];
        let s = (rho*rho-e*e).sqrt();
        let (a0,a1) = ((-e).atan2(-s)+2.*PI,(-e).atan2(s)+2.*PI);
        let (a0,a1) = if a0 > a1 { (a0-2.*PI,a1) } else { (a0,a1) };
        let loops = vec![vec![(E::L([-w,0.],[w,0.]),"floor"),(E::L([w,0.],[w,h]),"east"),(E::L([w,h],[s,h]),"top_e"),
            (E::A(c,rho,a0,a1),"bite"),(E::L([-s,h],[-w,h]),"top_w"),(E::L([-w,h],[-w,0.]),"west")]];
        out.push(Case {what,drawing:Drawing {loops},area:Box::new(move |d| (2.*(w-d))*(h-2.*d)-segment(rho+d,e+d)),
            names:vec!["floor","east","top_e","bite","top_w","west"],conics:true});
    }
    // two convex arcs meeting at an angle: a lens of radii 10 and 8, centres 12 apart
    {
        let (r1,r2,dd): (f64,f64,f64) = (10.,8.,12.);
        let x = (dd*dd+r1*r1-r2*r2)/(2.*dd);
        let y = (r1*r1-x*x).sqrt();
        let t1 = y.atan2(x);
        let t2 = y.atan2(x-dd);
        let loops = vec![vec![(E::A([0.,0.],r1,-t1,t1),"right"),(E::A([dd,0.],r2,t2,2.*PI-t2),"left")]];
        out.push(Case {what:"a lens",drawing:Drawing {loops},area:Box::new(move |d| lens(r1-d,r2-d,dd)),names:vec!["right","left"],conics:true});
    }
    // a convex arc meeting a concave one: a disc of 10 less a disc of 5 about a centre 11 off
    {
        let (r1,r2,dd): (f64,f64,f64) = (10.,5.,11.);
        let x = (dd*dd+r1*r1-r2*r2)/(2.*dd);
        let y = (r1*r1-x*x).sqrt();
        let t1 = y.atan2(x);
        let t2 = y.atan2(x-dd);
        let loops = vec![vec![(E::A([0.,0.],r1,t1,2.*PI-t1),"rim"),(E::A([dd,0.],r2,t2,2.*PI-t2),"bite")]];
        out.push(Case {what:"a crescent",drawing:Drawing {loops},area:Box::new(move |d| PI*(r1-d).powi(2)-lens(r1-d,r2+d,dd)),
            names:vec!["rim","bite"],conics:true});
    }
    // a hole of that kind: a plate 60 × 40 with a D of radius 8 above y = -3 cut through it; the
    // draft grows the hole, its corners reflex in the plate, met where the moved sides meet
    {
        let (r,c): (f64,f64) = (8.,-3.);
        let phi = (c/r).asin();
        let (a,b) = (on([0.,0.],r,phi),on([0.,0.],r,PI-phi));
        let loops = vec![vec![(E::L([-30.,-20.],[30.,-20.]),"south"),(E::L([30.,-20.],[30.,20.]),"east"),
            (E::L([30.,20.],[-30.,20.]),"north"),(E::L([-30.,20.],[-30.,-20.]),"west")],
            vec![(E::L(b,a),"flat"),(E::A([0.,0.],r,phi,PI-phi),"bore")]];
        out.push(Case {what:"a plate with a D-shaped hole",drawing:Drawing {loops},
            area:Box::new(move |d| (60.-2.*d)*(40.-2.*d)-segment(r+d,c-d)),names:vec!["flat","bore","south"],conics:true});
    }
    // a vane: 70 long and 4 thick, each end an arc of radius 4.25 bulging 0.5 past its sides' ends,
    // meeting them at an angle (fixture 147's)
    {
        let (x0,x1,t,b): (f64,f64,f64,f64) = (20.,90.,2.,0.5);
        let rr = (t*t+b*b)/(2.*b);
        let (cr,cl) = (x1+b-rr,x0-b+rr);
        let a = (t/rr).asin();
        let loops = vec![vec![(E::L([x0,-t],[x1,-t]),"south"),(E::A([cr,0.],rr,-a,a),"tip"),(E::L([x1,t],[x0,t]),"north"),
            (E::A([cl,0.],rr,PI-a,PI+a),"root")]];
        let area = move |d: f64| { let (t,r) = (t-d,rr-d); 2.*t*(cr-cl)+2.*(t*(r*r-t*t).sqrt()+r*r*(t/r).asin()) };
        out.push(Case {what:"a vane",drawing:Drawing {loops},area:Box::new(area),names:vec!["south","tip","north","root"],conics:true});
    }
    // a disc of 10 with a flat a hair off tangent to it: its chord turning ε from the arc at each end
    for (what,eps) in [("a flat 1e-3 off tangent",1e-3),("a flat 1e-5 off tangent",1e-5),("a flat 1e-7 off tangent",1e-7)] {
        let r: f64 = 10.;
        let c = r*(eps as f64).cos();
        let phi = (c/r).asin();
        let (a,b) = (on([0.,0.],r,phi),on([0.,0.],r,PI-phi));
        out.push(Case {what,drawing:Drawing {loops:vec![vec![(E::L(a,b),"flat"),(E::A([0.,0.],r,PI-phi,2.*PI+phi),"round")]]},
            area:Box::new(move |d| PI*(r-d).powi(2)-segment(r-d,c-d)),names:vec!["flat","round"],conics:true});
    }
    out
}

/// Every drawing of the family, drafted 3° one side of its parting plane and across it, stood
/// three ways and mirrored: its volume the integral of its section's area to 1e-9, its corner
/// edges conics on both their faces, the body valid and its sides named.
#[test]
fn a_drafted_corner_where_an_arc_meets_a_side_is_its_conic() {
    let theta = 3f64.to_radians();
    for case in family() {
        for mirrored in [false,true] {
            let drawing = if mirrored { case.drawing.mirrored() } else { Drawing {loops:case.drawing.loops.clone()} };
            for (place,m) in placements() {
                for (h0,h1) in [(0.,6.),(-4.,5.)] {
                    let what = format!("{}{}, {place}, from {h0} to {h1}",case.what,if mirrored { " mirrored" } else { "" });
                    let b = drafted(&drawing.profile(&m),h0,h1,theta).unwrap_or_else(|e| panic!("{what}: {e}"));
                    let want = volume_of(&*case.area,h0,h1,theta);
                    let conics = holds(&what,&b,want,&case.names);
                    assert!(!case.conics || conics > 0, "{what}: no conic corner");
                }
            }
        }
    }
}

/// Drafts small and large, and outward (a negative draft): the same closed forms.
#[test]
fn drafts_small_large_and_outward_are_their_closed_forms() {
    for case in family() {
        for (deg,h0,h1) in [(0.5,-8.,8.),(15.,0.,2.),(15.,-1.5,1.5),(-4.,-3.,5.)] {
            let what = format!("{}, {deg}° from {h0} to {h1}",case.what);
            let theta = (deg as f64).to_radians();
            let b = drafted(&case.drawing.profile(&placements()[0].1),h0,h1,theta).unwrap_or_else(|e| panic!("{what}: {e}"));
            holds(&what,&b,volume_of(&*case.area,h0,h1,theta),&case.names);
        }
    }
}

/// The exact solid `name` of a document, built as `solventc` builds it, checked, written to STEP
/// and read back, meshed and written as glTF, and held to its own field (`agreement::gate`): its
/// volume.
fn exported(e: &gcs_core::program::Elaborated,name: &str) -> f64 {
    use gcs_core::brep::{export,sweep::Say};
    let (sk,body) = (&e.sketch,crate::common::ent(e,name).i());
    let say = Say {stage:&|_: &str| {},mark:&|_| {}};
    let exact = export::exact(sk,body,None,None,&say).unwrap_or_else(|r| panic!("{name}: {}",r.message));
    exact.solid.check(1e-7*exact.solid.size()).unwrap_or_else(|e| panic!("{name}: {e:?}"));
    let text = export::step(&exact,name,None,&say).unwrap_or_else(|r| panic!("{name}: {}",r.message));
    gcs_core::brep::step_check::verify(&text,&gcs_core::brep::step_check::Solid::of(&exact.solid))
        .unwrap_or_else(|e| panic!("{name}: {e}"));
    gcs_core::brep::mesh::mesh(&exact.solid,1e-3*exact.solid.size(),0.1).unwrap_or_else(|e| panic!("{name}: {e}"));
    assert_eq!(&gcs_core::gltf::glb(sk,&[body],0.1)[..4],b"glTF");
    let field = gcs_core::solid::MaterialField::read(sk,body,gcs_core::solid::cad::AXIS_TOLERANCE).unwrap();
    gcs_core::solid::agreement::gate(&field,&exact.solid,None,name).unwrap_or_else(|e| panic!("{name}: {e}"));
    props::volume(&exact.solid)
}

fn near(what: &str,got: f64,want: f64) {
    assert!((got-want).abs() <= 1e-9*want.abs(), "{what}: {got} against {want} ({:e})",(got-want)/want);
}

/// `rounds/drafted_vane.sv` (fixture 147's vane): a vane whose arc ends meet its sides at an angle, drafted
/// 2° on a disc — the vane and the impeller its union with the disc, each its closed form, exported
/// and read alike by the field.
#[test]
fn the_drafted_vane_is_built_and_read_alike() {
    let e = crate::common::text::solved(include_str!("../../examples/rounds/drafted_vane.sv"));
    let (x0,x1,t,b) = (20f64,90f64,2f64,0.5f64);
    let rr = (t*t+b*b)/(2.*b);
    let (cr,cl) = (x1+b-rr,x0-b+rr);
    let area = move |d: f64| { let (t,r) = (t-d,rr-d); 2.*t*(cr-cl)+2.*(t*(r*r-t*t).sqrt()+r*r*(t/r).asin()) };
    let vane = volume_of(&area,0.,30.,2f64.to_radians());
    near("the vane",exported(&e,"vane"),vane);
    near("the impeller",exported(&e,"impeller"),vane+PI*100f64.powi(2)*3.);
}

/// `rounds/dboss.sv`: a D-shaped boss drafted 3° on a plate, its flat meeting its round side at an
/// angle — the boss and the part, each its closed form, exported and read alike by the field.
#[test]
fn the_drafted_d_boss_is_built_and_read_alike() {
    let e = crate::common::text::solved(include_str!("../../examples/rounds/dboss.sv"));
    let boss = volume_of(&|d| segment(10.-d,-4.+d),0.,12.,3f64.to_radians());
    near("the boss",exported(&e,"boss"),boss);
    near("the part",exported(&e,"part"),boss+60.*40.*5.);
}

/// What a draft still cannot build is said in the author's terms: a D's chord shrinking to nothing
/// as its arc closes on it (at `d = 1`, `1 / tan 3°` from the parting plane) is refused by the
/// elaborator, naming the side, and by the kernel, saying how far from the parting plane.
#[test]
fn a_corner_drafted_away_is_refused_saying_where() {
    let (r,c): (f64,f64) = (10.,8.);
    let phi = (c/r).asin();
    let (a,b) = (on([0.,0.],r,phi),on([0.,0.],r,PI-phi));
    let d = Drawing {loops:vec![vec![(E::L(b,a),"chord"),(E::A([0.,0.],r,phi,PI-phi),"arc")]]};
    let why = drafted(&d.profile(&placements()[0].1),0.,30.,3f64.to_radians()).unwrap_err();
    let at = 1./3f64.to_radians().tan();
    // (the chord comes to nothing where its line leaves the arc's circle: either is said)
    assert!((why.contains("to nothing") || why.contains("parts")) && why.contains("from the parting plane"), "{why}");
    assert!(why.contains(&format!("{:.2}",at)), "{why}: not at {at}");
    let e = crate::common::text::solved("unit mm
use std
use std.solids
in std.top {
  o := point
  fix((0, 0)) o
  a := point
  b := point
  fix((6, 8)) a
  fix((-6, 8)) b
  chord := line(b, a)
  arc := arc(center: o, start: a, end: b)
}
d := solid(std.solids.Drafted(face(chord, arc), from: 0mm, to: 30mm, draft: 3deg))
");
    let why = e.sketch.evaluated_solid(crate::common::ent(&e,"d").i(),gcs_core::solid::ApproximationPolicy::Report).unwrap_err();
    assert!(why.contains("to nothing") && why.contains("`chord`"), "{why}");
}
