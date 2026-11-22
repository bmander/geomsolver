//! **Two cylinders whose axes cross, or pass a hair apart, at radii equal or nearly** (#994,
//! fixture 146: a Ø12 groove whose axis lies in a pocket's floor through the pocket end's axis,
//! refused with "a shell of the arrangement is kept on one side of a plane" or "a cylinder's
//! pieces do not close"). Their meeting is the Steinmetz pair of ellipses crossing at two necks,
//! or two loops that come within `2 √(2 r δ)` of each other there, `δ` the radii's difference.
//! `ssi::crossed_cylinders` reads the ellipses for `δ` and the axes' gap within the tolerance, and
//! past it, where the lesser goes right through the other, the two loops as graphs over the
//! lesser's angle, every point on both surfaces and sampled so that no projection between two
//! crosses a neck to the other loop. Every Boolean of the two rods, either way round, is held to
//! the closed form of what they share: at the height `t` along the axes' common normal the rods'
//! sections are strips `2 √(r² − t²)` and `2 √(s² − (t − e)²)` wide crossing at the lean `θ`, a
//! parallelogram of their product over `sin θ`, integrated over `t` by composite Gauss–Legendre
//! after `t = r sin φ` (smooth to the ends).
use gcs_core::brep::boolean::{boolean,combined,Op};
use gcs_core::brep::build::{prism,Profile,ProfileEdge};
use gcs_core::brep::geom::{Frame,V};
use gcs_core::brep::props::volume;
use gcs_core::brep::topo::Brep;
use std::f64::consts::PI;

use crate::common::rounds::gauss_legendre;
use crate::cylinders_touching::TOL;

/// A rod of radius `r` about the line through `c` along `d` (unit), `half` either side.
fn rod(c: V,d: V,r: f64,half: f64) -> Brep {
    let x = if d[2].abs() < 0.9 { [d[1],-d[0],0.] } else { [1.,0.,0.] };
    let n = (x[0]*x[0]+x[1]*x[1]+x[2]*x[2]).sqrt();
    let circle = ProfileEdge::Arc {frame:Frame::new(c,d,[x[0]/n,x[1]/n,x[2]/n]),r,span:None};
    prism(&Profile {names:vec![],origin:c,normal:d,loops:vec![vec![circle]]},-half,half).unwrap()
}

/// `∫ f` over `[a, b]` by Gauss–Legendre, eight points on each of `n` panels.
fn gauss(a: f64,b: f64,n: usize,f: impl Fn(f64) -> f64) -> f64 {
    let rule = gauss_legendre(8);
    let h = (b-a)/n as f64;
    (0..n).map(|k| {
        let m = a+h*(k as f64+0.5);
        rule.iter().map(|&(x,w)| w*f(m+0.5*h*x)).sum::<f64>()*0.5*h
    }).sum()
}

/// What two infinite rods share: radii `r` and `s`, axes leaning `theta` apart with a gap `e`
/// along their common normal.
fn shared(r: f64,s: f64,theta: f64,e: f64) -> f64 {
    let strip = |t: f64| (r*r-t*t).max(0.).sqrt()*(s*s-(t-e)*(t-e)).max(0.).sqrt();
    // over φ, t = r sin φ, where the first strip is; the second's ends inside it, if any, are its
    // own panels' ends
    let lo = ((e-s)/r).clamp(-1.,1.).asin();
    let hi = ((e+s)/r).clamp(-1.,1.).asin();
    4./theta.sin()*gauss(lo,hi,400,|phi| strip(r*phi.sin())*r*phi.cos())
}

/// The rods: radius 6 along x, and `6 + dr` turned `theta` from it about z, lifted `e`.
fn pair(dr: f64,theta: f64,e: f64) -> (Brep,Brep,f64) {
    let half = 30.+6./theta.sin();
    let a = rod([0.;3],[1.,0.,0.],6.,half);
    let b = rod([0.,0.,e],[theta.cos(),theta.sin(),0.],6.+dr,half);
    (a,b,shared(6.,6.+dr,theta,e))
}

/// Every Boolean of the rods either way round: closed, and their volumes the closed form's.
fn held(dr: f64,deg: f64,e: f64) {
    let (a,b,common) = pair(dr,deg.to_radians(),e);
    let (va,vb) = (volume(&a),volume(&b));
    let what = format!("radii 6 and 6{dr:+e} at {deg}°, {e:e} apart");
    // the rods' own volumes and the closed form's measured alike, to the bar of the traced loops
    let bar = 1e-7*common;
    for (x,y,vx,vy) in [(&a,&b,va,vb),(&b,&a,vb,va)] {
        for (op,want) in [(Op::Union,vx+vy-common),(Op::Cut,vx-common),(Op::Common,common)] {
            let out = boolean(x,y,op,TOL).unwrap_or_else(|m| panic!("{what}, {op:?}: {m}"));
            out.check(1e-8).unwrap_or_else(|m| panic!("{what}, {op:?}: {m}"));
            let v = volume(&out);
            assert!((v-want).abs() <= bar,"{what}, {op:?}: {v} against {want} ({:.1e})",(v-want)/common);
        }
    }
}

const DR: [f64;17] = [0.,1e-8,-1e-8,1.5e-7,-1.5e-7,5e-7,-5e-7,1e-6,-1e-6,5e-6,-5e-6,1e-5,-1e-5,1e-4,-1e-4,1e-2,-1e-2];

#[test]
fn rods_crossing_square_at_radii_nearly_equal() { for dr in DR { held(dr,90.,0.) } }

#[test]
fn rods_crossing_at_60_at_radii_nearly_equal() { for dr in DR { held(dr,60.,0.) } }

#[test]
fn rods_crossing_at_30_at_radii_nearly_equal() { for dr in DR { held(dr,30.,0.) } }

/// Axes passing a hair apart, the radii further apart than that: the lesser still goes right
/// through the other, two loops. (Of one radius, the axes' gap opens one neck and closes the other
/// into a turn: one loop, traced as before, and refused — see `brep.md`.)
#[test]
fn rods_passing_a_hair_apart() {
    for e in [1e-7,1e-6,1e-4] {
        for deg in [90.,60.,30.] {
            for dr in [1e-2,-1e-2,1e-3,-1e-3] { held(dr,deg,e) }
        }
    }
}

/// **Fixture 146's shape**: a block 100 × 60 × 20; an obround pocket from x 24 to 66 across
/// `y = 30`, its ends half cylinders of 6 about upright axes at x 30 and 60, clipped flat at the
/// floor `z = 10`; a groove of radius `rho` along x whose axis lies in that floor, `dy` beside the
/// ends' axes — cut in each of three orders. What is left is the block less the pocket and the
/// groove, plus what they share: the groove's upper half within the pocket's outline, at each `y`
/// the obround's length `30 + 2 √(36 − (y − 30)²)` times the half disc's height there.
#[test]
fn a_groove_in_a_pockets_floor_through_its_ends_axes() {
    let xy: V = [0.,0.,1.];
    let line = |a: V,b: V| ProfileEdge::Line {a,b};
    let arc = |c: [f64;2],a0: f64,a1: f64| ProfileEdge::Arc {frame:Frame::new([c[0],c[1],0.],xy,[1.,0.,0.]),r:6.,span:Some([a0,a1])};
    let block = {
        let p = Profile {names:vec![],origin:[0.;3],normal:xy,loops:vec![vec![line([0.,0.,0.],[100.,0.,0.]),line([100.,0.,0.],[100.,60.,0.]),
            line([100.,60.,0.],[0.,60.,0.]),line([0.,60.,0.],[0.,0.,0.])]]};
        prism(&p,0.,20.).unwrap()
    };
    let pocket = {
        let e = vec![line([30.,24.,0.],[60.,24.,0.]),arc([60.,30.],-PI/2.,PI/2.),line([60.,36.,0.],[30.,36.,0.]),arc([30.,30.],PI/2.,1.5*PI)];
        prism(&Profile {names:vec![],origin:[0.;3],normal:xy,loops:vec![e]},10.,21.).unwrap()
    };
    // (radii 1.5e-7 to 5e-7 apart, a tolerance to four, are still refused: the groove then stands
    // that far through, or short of, the pocket's sides and its ends' floor edges, a sliver of the
    // side between lines `2 √(2 r δ)` apart no deeper than the tolerance, which `crossed_cylinders`
    // does not read; see `brep.md`)
    for (rho,dy) in [(6.,0.),(6.+1e-6,0.),(6.-1e-6,0.),(6.+5e-6,0.),(6.-5e-6,0.),(6.+1e-4,0.),(6.-1e-4,0.),(6.,0.1),(6.,-0.1),
        (6.1,0.),(5.9,0.)] {
        let groove = {
            let circle = ProfileEdge::Arc {frame:Frame::new([0.,30.+dy,10.],[1.,0.,0.],[0.,1.,0.]),r:rho,span:None};
            prism(&Profile {names:vec![],origin:[0.;3],normal:[1.,0.,0.],loops:vec![vec![circle]]},-1.,101.).unwrap()
        };
        let (lo,hi) = (24f64.max(30.+dy-rho),36f64.min(30.+dy+rho));
        let shared = gauss(-PI/2.,PI/2.,400,|phi| {
            // y over the stretch where both are, by y = m + h sin φ
            let (m,h) = (0.5*(lo+hi),0.5*(hi-lo));
            let y = m+h*phi.sin();
            let length = 30.+2.*(36.-(y-30.)*(y-30.)).max(0.).sqrt();
            let height = (rho*rho-(y-30.-dy)*(y-30.-dy)).max(0.).sqrt();
            length*height*h*phi.cos()
        });
        let want = 120000.-(360.+36.*PI)*10.-100.*PI*rho*rho+shared;
        let what = format!("a groove of {rho} {dy} beside the ends' axes");
        let orders: [(&str,Box<dyn Fn() -> Result<Brep,gcs_core::brep::KernelError>>);3] = [
            ("the pocket first",Box::new(|| combined(&combined(&block,&pocket,Op::Cut,0.)?,&groove,Op::Cut,0.))),
            ("the groove first",Box::new(|| combined(&combined(&block,&groove,Op::Cut,0.)?,&pocket,Op::Cut,0.))),
            ("both at once",Box::new(|| combined(&block,&combined(&pocket,&groove,Op::Union,0.)?,Op::Cut,0.))),
        ];
        for (order,build) in orders {
            let out = build().unwrap_or_else(|m| panic!("{what}, {order}: {m}"));
            let v = volume(&out);
            assert!((v-want).abs() <= 1e-10*want,"{what}, {order}: {v} against {want}");
        }
    }
}

